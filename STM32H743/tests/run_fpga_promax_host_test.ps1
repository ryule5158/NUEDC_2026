$ErrorActionPreference = 'Stop'

# 自动寻找任意盘符下Vivado 2018.3自带的MinGW，避免绑定开发机路径。
$gccCandidates = @()
if ($env:XILINX_VIVADO) {
    $gccCandidates += (Join-Path $env:XILINX_VIVADO `
        'msys64\mingw64\bin\gcc.exe')
}
$pathGcc = Get-Command gcc.exe -ErrorAction SilentlyContinue
if ($pathGcc) {
    $gccCandidates += $pathGcc.Source
}
foreach ($drive in Get-PSDrive -PSProvider FileSystem) {
    foreach ($relativeRoot in @(
        'Vivado\2018.3',
        'Xilinx\Vivado\2018.3',
        'Tools\Xilinx\Vivado\2018.3'
    )) {
        $gccCandidates += (Join-Path `
            (Join-Path $drive.Root $relativeRoot) `
            'msys64\mingw64\bin\gcc.exe')
    }
}

$gcc = $null
foreach ($candidate in $gccCandidates) {
    if ($candidate -and (Test-Path -LiteralPath $candidate)) {
        $gcc = (Resolve-Path -LiteralPath $candidate).Path
        break
    }
}
if (-not $gcc) {
    throw 'Vivado 2018.3 MinGW gcc.exe was not found on PATH, XILINX_VIVADO, or common drive locations.'
}

# 生成物仅放入系统临时目录，并在测试后删除。
$testRoot = Split-Path -Parent $MyInvocation.MyCommand.Path
$moduleRoot = Join-Path (Split-Path -Parent $testRoot) 'Modules'
$buildRoot = Join-Path ([System.IO.Path]::GetTempPath()) `
    ("nuedc_fpga_promax_test_{0}" -f $PID)
$resolvedTemp = [System.IO.Path]::GetFullPath([System.IO.Path]::GetTempPath())
$resolvedBuild = [System.IO.Path]::GetFullPath($buildRoot)
if (-not $resolvedBuild.StartsWith($resolvedTemp,
        [System.StringComparison]::OrdinalIgnoreCase)) {
    throw 'The generated build path is outside the system TEMP directory.'
}

New-Item -ItemType Directory -Path $resolvedBuild -Force | Out-Null
$originalPath = $env:PATH
$env:PATH = (Split-Path -Parent $gcc) + ';' + $env:PATH
try {
    # 将生产源码复制到纯ASCII路径，兼容旧版MinGW运行库。
    Copy-Item -LiteralPath (Join-Path $testRoot 'fpga_promax_host_test.c') `
        -Destination $resolvedBuild
    Copy-Item -LiteralPath (Join-Path $moduleRoot 'fpga_promax.c') `
        -Destination $resolvedBuild
    Copy-Item -LiteralPath (Join-Path $moduleRoot 'fpga_promax.h') `
        -Destination $resolvedBuild
    Copy-Item -LiteralPath (Join-Path $moduleRoot 'fpga_promax_link_map.h') `
        -Destination $resolvedBuild

    $testExe = Join-Path $resolvedBuild 'fpga_promax_host_test.exe'
    Push-Location $resolvedBuild
    try {
        & $gcc -std=c11 -Wall -Wextra -Wpedantic -Werror `
            -finput-charset=UTF-8 `
            fpga_promax_host_test.c fpga_promax.c `
            -lm -o $testExe 2>&1 | ForEach-Object { Write-Output $_ }
        if ($LASTEXITCODE -ne 0) {
            throw "Host test compilation failed with exit code $LASTEXITCODE."
        }

        & $testExe
        if ($LASTEXITCODE -ne 0) {
            throw "Host test failed with exit code $LASTEXITCODE."
        }
    }
    finally {
        Pop-Location
    }
}
finally {
    $env:PATH = $originalPath
    if (Test-Path -LiteralPath $resolvedBuild) {
        Remove-Item -LiteralPath $resolvedBuild -Recurse -Force
    }
}
