$ErrorActionPreference = "Stop"

$projectRoot = Split-Path -Parent $PSScriptRoot
$vcvars = "C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvars64.bat"
if (-not (Test-Path -LiteralPath $vcvars)) {
    throw "Visual Studio vcvars64.bat not found: $vcvars"
}

$buildDir = Join-Path $env:TEMP "nuedc_measure_thd_host_test"
New-Item -ItemType Directory -Force -Path $buildDir | Out-Null
$testSource = Join-Path $PSScriptRoot "measure_thd_host_test.c"
$measureSource = Join-Path $projectRoot "DSP\Measure.c"
$stubInclude = Join-Path $PSScriptRoot "stubs"
$dspInclude = Join-Path $projectRoot "DSP"
$testExe = Join-Path $buildDir "measure_thd_host_test.exe"
$compileLog = Join-Path $buildDir "compile.log"
$msvcMathHeader = "C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Tools\MSVC\14.43.34808\include\math.h"

if (Test-Path -LiteralPath $msvcMathHeader) {
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
    Write-Host "MSVC C runtime headers are unavailable; embedded C is covered by the Keil rebuild."
}

$python = (Get-Command python -ErrorAction Stop).Source
& $python (Join-Path $PSScriptRoot "measure_thd_numeric_test.py")
if ($LASTEXITCODE -ne 0) {
    throw "THD numerical regression failed with exit code $LASTEXITCODE"
}
