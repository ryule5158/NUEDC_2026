$ErrorActionPreference = 'Stop'
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
$projectRoot = Split-Path -Parent $testRoot
$buildRoot = Join-Path ([System.IO.Path]::GetTempPath()) `
    ("nuedc_g_tjc_ad9238_plot_test_{0}" -f $PID)
New-Item -ItemType Directory -Path $buildRoot -Force | Out-Null
try {
    Copy-Item -LiteralPath `
        (Join-Path $projectRoot 'BSP\g_tjc_plot_scale.h') `
        -Destination $buildRoot
    Copy-Item -LiteralPath `
        (Join-Path $testRoot 'g_tjc_plot_scale_host_test.c') `
        -Destination $buildRoot
    $exe = Join-Path $buildRoot 'g_tjc_plot_scale_host_test.exe'
    & $gcc -std=c11 -O2 -Wall -Wextra -Wpedantic -Werror `
        (Join-Path $buildRoot 'g_tjc_plot_scale_host_test.c') `
        -I $buildRoot -lm -o $exe 2>&1 | ForEach-Object {
        Write-Output $_
    }
    if ($LASTEXITCODE -ne 0) {
        throw 'G TJC AD9238 plot host test compile failed.'
    }
    & $exe
    if ($LASTEXITCODE -ne 0) {
        throw 'G TJC AD9238 plot host test failed.'
    }
} finally {
    if (Test-Path -LiteralPath $buildRoot) {
        Remove-Item -LiteralPath $buildRoot -Recurse -Force
    }
    $env:PATH = $originalPath
}
