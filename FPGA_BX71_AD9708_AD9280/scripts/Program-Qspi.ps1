# Program the complete Zynq boot image into BX71 QSPI without an SDK workspace.
[CmdletBinding()]
param(
    [string]$Image,
    [string]$Fsbl,
    [string]$ProgramFlash,
    [string]$HwServer,
    [string]$Url = 'tcp:localhost:3121',
    [ValidateSet(
        'qspi-x1-single',
        'qspi-x2-single',
        'qspi-x4-single',
        'qspi-x1-dual_stacked',
        'qspi-x2-dual_stacked',
        'qspi-x4-dual_stacked',
        'qspi-x8-dual_parallel'
    )]
    [string]$FlashType = 'qspi-x1-single',
    [switch]$BlankCheck,
    [switch]$NoVerify,
    [switch]$ProbeOnly
)

Set-StrictMode -Version 2.0
$ErrorActionPreference = 'Stop'

. (Join-Path $PSScriptRoot 'XilinxTools.ps1')

$fpgaRoot = Split-Path -Parent $PSScriptRoot
$outputDir = Join-Path $fpgaRoot 'output'
if ([string]::IsNullOrWhiteSpace($Image)) {
    $Image = Join-Path $outputDir 'BOOT_PS7.bin'
}
if ([string]::IsNullOrWhiteSpace($Fsbl)) {
    $asciiFsbl = Join-Path $fpgaRoot 'boot\FSBL.elf'
    if (Test-Path -LiteralPath $asciiFsbl -PathType Leaf) {
        $Fsbl = $asciiFsbl
    } else {
        $fsblCandidates = @(
            Get-ChildItem `
                -LiteralPath $fpgaRoot `
                -Recurse `
                -File `
                -Filter 'FSBL.elf' `
                -ErrorAction SilentlyContinue |
                Where-Object {
                    $_.FullName -notmatch '\\Vivado\\|\\\.metadata\\'
                }
        )
        if ($fsblCandidates.Count -ne 1) {
            $candidateList = $fsblCandidates.FullName -join [Environment]::NewLine
            throw @"
Unable to select a unique project FSBL. Pass -Fsbl explicitly.
Candidates:
$candidateList
"@
        }
        $Fsbl = $fsblCandidates[0].FullName
    }
}

$ProgramFlash = Resolve-XilinxTool `
    -ToolName 'program_flash.bat' `
    -Product SDK `
    -Version '2018.3' `
    -SpecifiedPath $ProgramFlash
$HwServer = Resolve-XilinxTool `
    -ToolName 'hw_server.bat' `
    -Product SDK `
    -Version '2018.3' `
    -SpecifiedPath $HwServer

foreach ($required in @($Image, $Fsbl, $ProgramFlash, $HwServer)) {
    if (-not (Test-Path -LiteralPath $required -PathType Leaf)) {
        throw "Required file was not found: $required"
    }
}

$Image = (Resolve-Path -LiteralPath $Image).Path
$Fsbl = (Resolve-Path -LiteralPath $Fsbl).Path
$imageItem = Get-Item -LiteralPath $Image
if ($imageItem.Length -le 0) {
    throw "The QSPI image is empty: $Image"
}

function Get-JtagTargetProbe {
    param(
        [string]$Programmer,
        [string]$ServerUrl
    )

    $probeLines = @(& $Programmer -jtagtargets -url $ServerUrl 2>&1)
    $probeExitCode = $LASTEXITCODE
    [PSCustomObject]@{
        ExitCode = $probeExitCode
        Text = ($probeLines | Out-String)
    }
}

Write-Host "Xilinx flash programmer: $ProgramFlash"
Write-Host "QSPI image:             $Image"
Write-Host "FSBL:                   $Fsbl"
Write-Host "Flash type:             $FlashType"
Write-Host "TCF server:             $Url"
Write-Host ''

$probe = Get-JtagTargetProbe -Programmer $ProgramFlash -ServerUrl $Url
if (($probe.ExitCode -ne 0) -or ($probe.Text -notmatch 'Available targets and devices')) {
    Write-Host 'No usable hw_server was found; starting a local hidden instance...'
    Start-Process `
        -FilePath $HwServer `
        -ArgumentList @('-s', 'tcp::3121', '-p0', '-q') `
        -WindowStyle Hidden | Out-Null

    $probe = $null
    for ($attempt = 1; $attempt -le 8; $attempt++) {
        Start-Sleep -Milliseconds 750
        $probe = Get-JtagTargetProbe -Programmer $ProgramFlash -ServerUrl $Url
        if (($probe.ExitCode -eq 0) -and
            ($probe.Text -match 'Available targets and devices')) {
            break
        }
    }
}

if (($null -eq $probe) -or
    ($probe.ExitCode -ne 0) -or
    ($probe.Text -notmatch 'Available targets and devices')) {
    throw "Unable to connect to hw_server at $Url.`n$($probe.Text)"
}

Write-Host $probe.Text.Trim()
if ($probe.Text -notmatch '\(name\s+xc7z020\s+idcode\s+23727093\)') {
    throw 'The connected JTAG chain does not contain the expected XC7Z020 device.'
}

$armTargetMatches = @(
    [regex]::Matches(
        $probe.Text,
        '(?m)^\s*(\d+)\s+(\S+)\s+\(name\s+arm_dap\s+idcode\s+4ba00477\)'
    )
)
if ($armTargetMatches.Count -eq 0) {
    throw 'No Zynq ARM DAP target was found on the JTAG chain.'
}
if ($armTargetMatches.Count -gt 1) {
    throw @"
More than one Zynq target is connected.
Disconnect every board except the BX71 that should be programmed, then re-run.
"@
}

# program_flash 2018.3 rejects -target_name even when passed the exact name
# returned by -jtagtargets. Use the numeric ARM DAP ID from the probe output.
$targetId = $armTargetMatches[0].Groups[1].Value
$detectedTarget = $armTargetMatches[0].Groups[2].Value
Write-Host "Validated Zynq target:  $detectedTarget"
Write-Host "ARM DAP target ID:      $targetId"

if ($ProbeOnly) {
    Write-Host ''
    Write-Host 'QSPI PROGRAMMER PROBE PASSED.'
    Write-Host 'No flash contents were changed.'
    return
}

# Xilinx 2018.3 tools are sensitive to long and non-ASCII paths. Stage only
# the two programming inputs in a short, ASCII-only temporary directory.
$stageDir = Join-Path $env:TEMP ("BX71_QSPI_{0}" -f $PID)
$stagedImage = Join-Path $stageDir 'BOOT_PS7.bin'
$stagedFsbl = Join-Path $stageDir 'FSBL.elf'
$logFile = Join-Path $outputDir (
    'program_qspi_{0}.log' -f (Get-Date -Format 'yyyyMMdd_HHmmss')
)
$manifest = Join-Path $outputDir 'LAST_PROGRAMMED.txt'
$transcriptStarted = $false

try {
    New-Item -ItemType Directory -Path $stageDir -Force | Out-Null
    Copy-Item -LiteralPath $Image -Destination $stagedImage -Force
    Copy-Item -LiteralPath $Fsbl -Destination $stagedFsbl -Force
    New-Item -ItemType Directory -Path $outputDir -Force | Out-Null
    if (Test-Path -LiteralPath $manifest) {
        Remove-Item -LiteralPath $manifest -Force
    }

    try {
        Start-Transcript -LiteralPath $logFile -Force | Out-Null
        $transcriptStarted = $true
    } catch {
        Write-Warning "Could not start transcript log: $($_.Exception.Message)"
    }

    $programArguments = @(
        '-f', $stagedImage,
        '-offset', '0',
        '-flash_type', $FlashType,
        '-fsbl', $stagedFsbl
    )
    if ($BlankCheck) {
        $programArguments += '-blank_check'
    }
    if (-not $NoVerify) {
        $programArguments += '-verify'
    }
    $programArguments += @('-target_id', $targetId, '-url', $Url)

    Write-Host ''
    Write-Host 'Programming the complete QSPI boot image...'
    Write-Host 'Do not disconnect power or JTAG until this command finishes.'
    $programOutput = @(& $ProgramFlash @programArguments 2>&1)
    $programExitCode = $LASTEXITCODE
    $programOutput | ForEach-Object { Write-Host $_ }
    $programText = $programOutput | Out-String
    if ($programExitCode -ne 0) {
        throw "program_flash failed with exit code $programExitCode`n$programText"
    }
    if ($programText -match '(?im)^\s*(\[ERROR\]|ERROR:)') {
        throw "program_flash reported an error even though its exit code was 0.`n$programText"
    }
    if ($programText -notmatch '(?i)Flash Operation Successful') {
        throw "program_flash did not report final flash success.`n$programText"
    }
    if ((-not $NoVerify) -and
        ($programText -notmatch '(?i)Verify Operation successful')) {
        throw "program_flash did not report verify success.`n$programText"
    }

    $imageHash = (Get-FileHash -LiteralPath $Image -Algorithm SHA256).Hash
    @(
        "Time:       $(Get-Date -Format 'yyyy-MM-dd HH:mm:ss zzz')"
        "Image:      $Image"
        "Length:     $($imageItem.Length)"
        "SHA256:     $imageHash"
        "FlashType:  $FlashType"
        "Target:     $detectedTarget"
        "Verified:   $(-not $NoVerify)"
        "Log:        $logFile"
    ) | Set-Content -LiteralPath $manifest -Encoding UTF8

    Write-Host ''
    Write-Host 'QSPI PROGRAMMING PASSED.'
    Write-Host "Image length: $($imageItem.Length) bytes"
    Write-Host "SHA256:       $imageHash"
    Write-Host "Log:          $logFile"
    Write-Host ''
    Write-Host 'For the final cold-boot test: power off, select QSPI boot, then power on.'
} finally {
    if ($transcriptStarted) {
        try {
            Stop-Transcript | Out-Null
        } catch {
            # The important operation has already completed or failed.
        }
    }
    if (Test-Path -LiteralPath $stageDir) {
        Remove-Item -LiteralPath $stageDir -Recurse -Force
    }
}
