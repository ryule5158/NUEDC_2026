$ErrorActionPreference = 'Stop'

# Compile and call FilterEx.c directly; Python only reports the C test result.
$testRoot = $PSScriptRoot
$projectRoot = Split-Path -Parent $testRoot
$buildRoot = Join-Path $testRoot 'build'
$stubRoot = Join-Path $testRoot 'host_stubs'

# Discover the installed 64-bit MSVC instead of binding to a specific toolset version.
$compiler = Get-ChildItem `
    -Path "${env:ProgramFiles}\Microsoft Visual Studio\*\*\VC\Tools\MSVC\*\bin\Hostx64\x64\cl.exe" `
    -ErrorAction SilentlyContinue |
    Sort-Object FullName -Descending |
    Select-Object -First 1
if (-not $compiler) {
    throw 'Visual Studio 64-bit C compiler cl.exe was not found.'
}
$compiler = $compiler.FullName
$linker = Join-Path (Split-Path -Parent $compiler) 'link.exe'
if (-not (Test-Path -LiteralPath $linker)) {
    throw 'The link.exe paired with cl.exe was not found.'
}

$resolvedTestRoot = [System.IO.Path]::GetFullPath($testRoot)
$resolvedBuild = [System.IO.Path]::GetFullPath($buildRoot)
if (-not $resolvedBuild.StartsWith(
        $resolvedTestRoot, [System.StringComparison]::OrdinalIgnoreCase)) {
    throw 'The fractional-delay test output is outside the tests directory.'
}

$filterObject = Join-Path $resolvedBuild 'FilterEx.obj'
$testObject = Join-Path $resolvedBuild 'fractional_delay_host_test.obj'
$testDll = Join-Path $resolvedBuild 'fractional_delay_host_test.dll'

New-Item -ItemType Directory -Path $resolvedBuild -Force | Out-Null
try {
    & $compiler /nologo /utf-8 /TC /O2 /Oi- /GS- /Zl /Gy `
        /I $stubRoot /I (Join-Path $projectRoot 'DSP') /c `
        (Join-Path $projectRoot 'DSP\FilterEx.c') "/Fo$filterObject"
    if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }

    & $compiler /nologo /utf-8 /TC /O2 /Oi- /GS- /Zl /Gy `
        /I $stubRoot /I (Join-Path $projectRoot 'DSP') /c `
        (Join-Path $testRoot 'fractional_delay_host_test.c') "/Fo$testObject"
    if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }

    & $linker /nologo /DLL /NOENTRY /NODEFAULTLIB /OPT:REF `
        "/OUT:$testDll" $filterObject $testObject
    if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }

    $env:PYTHONIOENCODING = 'utf-8'
    & python (Join-Path $testRoot 'fractional_delay_host_runner.py')
    if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
}
finally {
    if (Test-Path -LiteralPath $resolvedBuild) {
        Remove-Item -LiteralPath $resolvedBuild -Recurse -Force
    }
}
