param(
    [string]$KeilUv4Path = $env:NUEDC_KEIL_UV4
)

$ErrorActionPreference = "Stop"

$projectRoot = Split-Path -Parent $MyInvocation.MyCommand.Path
$keilRoot = Join-Path $projectRoot "MDK-ARM"
$project = Join-Path $keilRoot "STM32H743_AD9238.uvprojx"
$log = Join-Path $keilRoot "build.log"
$output = Join-Path $projectRoot "output"

if ([string]::IsNullOrWhiteSpace($KeilUv4Path)) {
    $uv4Candidates = @(
        (Join-Path $env:LOCALAPPDATA "Keil_MDK\UV4\uVision.com"),
        (Join-Path $env:LOCALAPPDATA "Keil_v5\UV4\uVision.com"),
        "C:\Keil_v5\UV4\uVision.com",
        "C:\Keil_v5\UV4\UV4.exe"
    )
    $KeilUv4Path = $uv4Candidates |
        Where-Object { Test-Path -LiteralPath $_ } |
        Select-Object -First 1
}

if ([string]::IsNullOrWhiteSpace($KeilUv4Path) -or
    (-not (Test-Path -LiteralPath $KeilUv4Path))) {
    throw "Keil uVision not found. Set NUEDC_KEIL_UV4 or pass -KeilUv4Path."
}

if (-not (Test-Path -LiteralPath $project)) {
    throw "Keil project not found: $project"
}

Push-Location $keilRoot
try {
    if (Test-Path -LiteralPath $log) {
        Remove-Item -LiteralPath $log
    }
    <#
      uVision.com can hand the build to an already-running UV4.exe. In that
      case waiting on the returned process never finishes even though the
      build log and artifacts are complete. Poll the newly-created log for
      the final summary instead; do not terminate the user's GUI instance.
    #>
    $process = Start-Process -FilePath $KeilUv4Path `
        -ArgumentList '-r "STM32H743_AD9238.uvprojx" -j0 -o "build.log"' `
        -WindowStyle Hidden -PassThru
    $deadline = [DateTime]::UtcNow.AddSeconds(180)
    $buildResult = ''
    do {
        if (Test-Path -LiteralPath $log) {
            $buildResult = Get-Content -Raw -LiteralPath $log
            if ($buildResult -match
                '"[^"]+\.axf" - [0-9]+ Error\(s\), [0-9]+ Warning\(s\)\.') {
                break
            }
        }
        if ($process.HasExited -and ($process.ExitCode -ne 0) -and
            (-not (Test-Path -LiteralPath $log))) {
            throw "Keil rebuild failed before producing a log (exit $($process.ExitCode))."
        }
        Start-Sleep -Milliseconds 250
    } while ([DateTime]::UtcNow -lt $deadline)

    if ($buildResult -notmatch
        '"[^"]+\.axf" - [0-9]+ Error\(s\), [0-9]+ Warning\(s\)\.') {
        throw "Keil rebuild timed out before a final summary. See: $log"
    }
    Write-Output $buildResult
    if ($buildResult -notmatch '0 Error\(s\), 0 Warning\(s\)') {
        throw "Keil rebuild failed or produced warnings. See: $log"
    }
} finally {
    Pop-Location
}

# 保存可直接烧录和复核的最终产物。
New-Item -ItemType Directory -Path $output -Force | Out-Null
$artifacts = @(
    @{ Source = "MDK-ARM\STM32H743_AD9238\STM32H743_AD9238.axf"; Name = "STM32H743_AD9238.axf" },
    @{ Source = "MDK-ARM\STM32H743_AD9238\STM32H743_AD9238.hex"; Name = "STM32H743_AD9238.hex" },
    @{ Source = "MDK-ARM\STM32H743_AD9238\STM32H743_AD9238.map"; Name = "STM32H743_AD9238.map" },
    @{ Source = "MDK-ARM\build.log"; Name = "build.log" }
)
$hashLines = foreach ($artifact in $artifacts) {
    $source = Join-Path $projectRoot $artifact.Source
    if (-not (Test-Path -LiteralPath $source)) {
        throw "Expected build artifact not found: $source"
    }
    $target = Join-Path $output $artifact.Name
    Copy-Item -LiteralPath $source -Destination $target -Force
    $hash = (Get-FileHash -LiteralPath $target -Algorithm SHA256).Hash
    "$hash  $($artifact.Name)"
}
$hashLines | Set-Content -LiteralPath (Join-Path $output "SHA256SUMS.txt") -Encoding ASCII
