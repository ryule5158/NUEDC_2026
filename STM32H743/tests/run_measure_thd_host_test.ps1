$ErrorActionPreference = "Stop"

$projectRoot = Split-Path -Parent $PSScriptRoot

# 优先用vswhere查找，再扫描常见版本/版本类型，避免绑定开发机安装路径。
$vcvars = $null
$vswhere = Join-Path ${env:ProgramFiles(x86)} `
    'Microsoft Visual Studio\Installer\vswhere.exe'
if (Test-Path -LiteralPath $vswhere) {
    $installationPath = & $vswhere -latest -products '*' `
        -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 `
        -property installationPath
    if ($installationPath) {
        $candidate = Join-Path $installationPath 'VC\Auxiliary\Build\vcvars64.bat'
        if (Test-Path -LiteralPath $candidate) {
            $vcvars = $candidate
        }
    }
}
if (-not $vcvars) {
    foreach ($root in @($env:ProgramFiles, ${env:ProgramFiles(x86)})) {
        foreach ($year in @('2022', '2019', '2017')) {
            foreach ($edition in @('Community', 'Professional',
                                    'Enterprise', 'BuildTools')) {
                $candidate = Join-Path $root `
                    "Microsoft Visual Studio\$year\$edition\VC\Auxiliary\Build\vcvars64.bat"
                if (Test-Path -LiteralPath $candidate) {
                    $vcvars = $candidate
                    break
                }
            }
            if ($vcvars) { break }
        }
        if ($vcvars) { break }
    }
}

$buildDir = Join-Path $env:TEMP "nuedc_measure_thd_host_test"
New-Item -ItemType Directory -Force -Path $buildDir | Out-Null
$testSource = Join-Path $PSScriptRoot "measure_thd_host_test.c"
$measureSource = Join-Path $projectRoot "DSP\Measure.c"
$stubInclude = Join-Path $PSScriptRoot "stubs"
$dspInclude = Join-Path $projectRoot "DSP"
$testExe = Join-Path $buildDir "measure_thd_host_test.exe"
$compileLog = Join-Path $buildDir "compile.log"

if ($vcvars) {
    $compileCommand = "call `"$vcvars`" >nul && cl /nologo /std:c11 /W4 " +
        "/I`"$stubInclude`" /I`"$dspInclude`" " +
        "`"$testSource`" `"$measureSource`" /Fe:`"$testExe`" " +
        "/link /OUT:`"$testExe`""
    $savedErrorPreference = $ErrorActionPreference
    $ErrorActionPreference = "Continue"
    cmd.exe /d /c $compileCommand 2>&1 | Tee-Object -FilePath $compileLog
    $compileExitCode = $LASTEXITCODE
    $ErrorActionPreference = $savedErrorPreference
    if ($compileExitCode -ne 0) {
        throw "Host compile failed. See $compileLog"
    }

    & $testExe
    if ($LASTEXITCODE -ne 0) {
        throw "THD C host test failed with exit code $LASTEXITCODE"
    }
} else {
    Write-Host "MSVC C compiler is unavailable; running the portable numerical regression only."
}

$python = (Get-Command python -ErrorAction Stop).Source
& $python (Join-Path $PSScriptRoot "measure_thd_numeric_test.py")
if ($LASTEXITCODE -ne 0) {
    throw "THD numerical regression failed with exit code $LASTEXITCODE"
}
