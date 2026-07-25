param(
    [switch]$CleanOnly,
    [string]$KeilUv4Path = $env:NUEDC_KEIL_UV4
)

$ErrorActionPreference = "Stop"

$projectRoot = Split-Path -Parent $MyInvocation.MyCommand.Path
$project = Join-Path $projectRoot "Keil\TI_MSPM0G3507.uvprojx"
$log = Join-Path $projectRoot "Keil\build.log"
$output = Join-Path $projectRoot "output"

if ([string]::IsNullOrWhiteSpace($KeilUv4Path)) {
    $uv4Candidates = @(
        (Join-Path $env:LOCALAPPDATA "Keil_MDK\UV4\UV4.exe"),
        "C:\Keil_v5\UV4\UV4.exe"
    )
    $KeilUv4Path = $uv4Candidates |
        Where-Object { Test-Path -LiteralPath $_ } |
        Select-Object -First 1
}
$uv4 = $KeilUv4Path

function Clear-TiBuildIntermediates {
    $intermediatePaths = @("$projectRoot\Keil\Objects", "$projectRoot\Keil\TI_MSPM0G3507.map", "$projectRoot\Keil\build.log")
    foreach ($intermediatePath in $intermediatePaths) {
        if (Test-Path -LiteralPath $intermediatePath) {
            Remove-Item -LiteralPath $intermediatePath -Recurse -Force
        }
    }

    # Keil's *.uvguix.* / *.uvgui.* files hold the user's live workspace state.
    # Deleting them while uVision is open causes stale-project save prompts.
    $staleToolLogs = Get-ChildItem -LiteralPath (Join-Path $projectRoot "Keil") -File |
        Where-Object {
            ($_.Name -like "flash_*.log") -or
            ($_.Name -like "build_*_verify*.log")
        }
    foreach ($staleToolLog in $staleToolLogs) {
        Remove-Item -LiteralPath $staleToolLog.FullName -Force
    }

    $emptyDirectories = @("$projectRoot\Board", "$projectRoot\Inc", "$projectRoot\tests", "$projectRoot\SysConfigGeneratedTest")
    foreach ($emptyDirectory in $emptyDirectories) {
        if ((Test-Path -LiteralPath $emptyDirectory) -and
            ((Get-ChildItem -LiteralPath $emptyDirectory -Force).Count -eq 0)) {
            Remove-Item -LiteralPath $emptyDirectory -Force
        }
    }
}

if ($CleanOnly) {
    Clear-TiBuildIntermediates
    Write-Output "TI project intermediates removed."
    return
}

if ([string]::IsNullOrWhiteSpace($uv4) -or (-not (Test-Path -LiteralPath $uv4))) {
    throw "Keil uVision not found. Set NUEDC_KEIL_UV4 or pass -KeilUv4Path."
}

if (-not (Test-Path $project)) {
    throw "Keil project not found: $project"
}

Push-Location (Join-Path $projectRoot "Keil")
try {
    if (Test-Path $log) {
        Remove-Item -LiteralPath $log
    }
    Start-Process -FilePath $uv4 `
        -ArgumentList '-r "TI_MSPM0G3507.uvprojx" -j0 -o "build.log"' `
        -WindowStyle Hidden -Wait | Out-Null
    $buildResult = Get-Content -Raw $log
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
    @{ Source = "Keil\Objects\TI_MSPM0G3507.axf"; Name = "TI_MSPM0G3507.axf" },
    @{ Source = "Keil\Objects\TI_MSPM0G3507.hex"; Name = "TI_MSPM0G3507.hex" },
    @{ Source = "Keil\TI_MSPM0G3507.map"; Name = "TI_MSPM0G3507.map" },
    @{ Source = "Keil\build.log"; Name = "build.log" }
)
$hashLines = foreach ($artifact in $artifacts) {
    $source = Join-Path $projectRoot $artifact.Source
    $target = Join-Path $output $artifact.Name
    Copy-Item -LiteralPath $source -Destination $target -Force
    $hash = (Get-FileHash -LiteralPath $target -Algorithm SHA256).Hash
    "$hash  $($artifact.Name)"
}
$hashLines | Set-Content -LiteralPath (Join-Path $output "SHA256SUMS.txt") -Encoding ASCII

# 最终产物已经保存到 output，工程目录不保留可重建的中间文件。
Start-Sleep -Milliseconds 750
Clear-TiBuildIntermediates
