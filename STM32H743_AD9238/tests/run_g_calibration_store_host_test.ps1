$ErrorActionPreference = 'Stop'
$gccCandidates = @()
if ($env:XILINX_VIVADO) {
    $gccCandidates += (Join-Path $env:XILINX_VIVADO `
        'msys64\mingw64\bin\gcc.exe')
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
$gcc = $gccCandidates |
    Where-Object { $_ -and (Test-Path -LiteralPath $_) } |
    Select-Object -First 1
if (-not $gcc) {
    throw 'Vivado 2018.3 MinGW GCC was not found.'
}
$gcc = (Resolve-Path -LiteralPath $gcc).Path
$originalPath = $env:PATH
$env:PATH = (Split-Path -Parent $gcc) + ';' + $env:PATH

$testRoot = Split-Path -Parent $MyInvocation.MyCommand.Path
$buildRoot = Join-Path ([System.IO.Path]::GetTempPath()) `
    ("nuedc_g_calibration_store_test_{0}" -f $PID)
New-Item -ItemType Directory -Path $buildRoot -Force | Out-Null
try {
    Copy-Item -LiteralPath `
        (Join-Path $testRoot 'g_calibration_store_host_test.c') `
        -Destination $buildRoot
    Copy-Item -LiteralPath `
        (Join-Path (Split-Path -Parent $testRoot) `
            'BSP\g_calibration_store.c') `
        -Destination $buildRoot
    foreach ($header in @(
        'g_calibration_store.h',
        'bsp_qspi_flash.h',
        'bsp_storage.h'
    )) {
        Copy-Item -LiteralPath `
            (Join-Path (Join-Path (Split-Path -Parent $testRoot) 'BSP') `
                $header) `
            -Destination $buildRoot
    }
    $exe = Join-Path $buildRoot 'g_calibration_store_host_test.exe'
    & $gcc -std=c11 -O2 -Wall -Wextra -Wpedantic -Werror `
        (Join-Path $buildRoot 'g_calibration_store_host_test.c') `
        -o $exe 2>&1 | ForEach-Object { Write-Output $_ }
    if ($LASTEXITCODE -ne 0) {
        throw 'Calibration store host test compile failed.'
    }
    & $exe
    if ($LASTEXITCODE -ne 0) {
        throw 'Calibration store host test failed.'
    }
} finally {
    if (Test-Path -LiteralPath $buildRoot) {
        Remove-Item -LiteralPath $buildRoot -Recurse -Force
    }
    $env:PATH = $originalPath
}
