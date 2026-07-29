$ErrorActionPreference = 'Stop'
$gcc = 'E:\Vivado\2018.3\msys64\mingw64\bin\gcc.exe'
if (-not (Test-Path -LiteralPath $gcc)) { throw 'Vivado MinGW GCC not found.' }
$originalPath = $env:PATH
$env:PATH = (Split-Path -Parent $gcc) + ';' + $env:PATH
$testRoot = Split-Path -Parent $MyInvocation.MyCommand.Path
$projectRoot = Split-Path -Parent $testRoot
$buildRoot = Join-Path ([System.IO.Path]::GetTempPath()) `
  ("nuedc_g_analyzer_test_{0}" -f $PID)
New-Item -ItemType Directory -Path $buildRoot -Force | Out-Null
try {
  Copy-Item -LiteralPath (Join-Path $projectRoot 'DSP\periodic_analyzer.c') `
    -Destination $buildRoot
  Copy-Item -LiteralPath (Join-Path $projectRoot 'DSP\periodic_analyzer.h') `
    -Destination $buildRoot
  Copy-Item -LiteralPath (Join-Path $testRoot 'fft_host_stub.c') `
    -Destination $buildRoot
  Copy-Item -LiteralPath (Join-Path $testRoot 'fft_host_stub.h') `
    -Destination (Join-Path $buildRoot 'FFT.h')
  Copy-Item -LiteralPath (Join-Path $testRoot 'periodic_analyzer_host_test.c') `
    -Destination $buildRoot
  $exe = Join-Path $buildRoot 'periodic_analyzer_host_test.exe'
  & $gcc -std=c11 -O2 -Wall -Wextra -Werror `
    (Join-Path $buildRoot 'periodic_analyzer.c') `
    (Join-Path $buildRoot 'fft_host_stub.c') `
    (Join-Path $buildRoot 'periodic_analyzer_host_test.c') `
    -I $buildRoot -lm -o $exe 2>&1 | ForEach-Object {
      Write-Output $_ }
  if ($LASTEXITCODE -ne 0) { throw 'Host test compile failed.' }
  & $exe
  if ($LASTEXITCODE -ne 0) { throw 'Host test failed.' }
} finally {
  if (Test-Path -LiteralPath $buildRoot) {
    Remove-Item -LiteralPath $buildRoot -Recurse -Force
  }
  $env:PATH = $originalPath
}
