$ErrorActionPreference = 'Stop'

$testRoot = $PSScriptRoot
$projectRoot = Split-Path -Parent $testRoot

# Prefer Vivado 2018.3 MinGW so an incomplete MSVC installation cannot break the test.
$gccCandidates = @()
if ($env:XILINX_VIVADO) {
    $gccCandidates += (Join-Path $env:XILINX_VIVADO 'msys64\mingw64\bin\gcc.exe')
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
    throw 'Vivado 2018.3 MinGW gcc.exe was not found.'
}

# Copy sources to TEMP because this older MinGW does not handle non-ASCII paths reliably.
$buildRoot = Join-Path ([System.IO.Path]::GetTempPath()) `
    ("nuedc_measure_thd_test_{0}" -f $PID)
$resolvedTemp = [System.IO.Path]::GetFullPath([System.IO.Path]::GetTempPath())
$resolvedBuild = [System.IO.Path]::GetFullPath($buildRoot)
if (-not $resolvedBuild.StartsWith(
        $resolvedTemp, [System.StringComparison]::OrdinalIgnoreCase)) {
    throw 'The host-test directory is outside the system TEMP directory.'
}

New-Item -ItemType Directory -Path $resolvedBuild -Force | Out-Null
$originalPath = $env:PATH
$env:PATH = (Split-Path -Parent $gcc) + ';' + $originalPath
try {
    Copy-Item -LiteralPath (Join-Path $testRoot 'measure_thd_host_test.c') `
        -Destination $resolvedBuild
    Copy-Item -LiteralPath (Join-Path $projectRoot 'DSP\Measure.c') `
        -Destination $resolvedBuild
    Copy-Item -LiteralPath (Join-Path $projectRoot 'DSP\Measure.h') `
        -Destination $resolvedBuild
    Copy-Item -LiteralPath (Join-Path $testRoot 'stubs\arm_math.h') `
        -Destination $resolvedBuild
    Copy-Item -LiteralPath (Join-Path $testRoot 'stubs\main.h') `
        -Destination $resolvedBuild

    $testExe = Join-Path $resolvedBuild 'measure_thd_host_test.exe'
    Push-Location $resolvedBuild
    try {
        & $gcc -std=c11 -Wall -Wextra -Wpedantic -Werror `
            -finput-charset=UTF-8 `
            measure_thd_host_test.c Measure.c -lm -o $testExe
        if ($LASTEXITCODE -ne 0) {
            throw "THD host-test compilation failed with exit code $LASTEXITCODE."
        }

        & $testExe
        if ($LASTEXITCODE -ne 0) {
            throw "THD C host test failed with exit code $LASTEXITCODE."
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

$python = (Get-Command python -ErrorAction Stop).Source
$env:PYTHONIOENCODING = 'utf-8'
& $python (Join-Path $testRoot 'measure_thd_numeric_test.py')
if ($LASTEXITCODE -ne 0) {
    throw "THD numerical regression failed with exit code $LASTEXITCODE."
}
