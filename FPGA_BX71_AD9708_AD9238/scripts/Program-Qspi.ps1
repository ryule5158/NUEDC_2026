# Program and verify the complete G-analyzer Zynq boot image in BX71 QSPI.
[CmdletBinding()]
param(
    [string]$Image,
    [string]$Fsbl,
    [string]$ProgramFlash,
    [string]$HwServer,
    [string]$Xsdb,
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
    [switch]$ProbeOnly,
    [switch]$ValidateOnly
)

Set-StrictMode -Version 2.0
$ErrorActionPreference = 'Stop'

. (Join-Path $PSScriptRoot 'XilinxTools.ps1')

$fpgaRoot = Split-Path -Parent $PSScriptRoot
$outputDir = Join-Path $fpgaRoot 'output'
$projectImage = Join-Path $outputDir 'BOOT_G_ANALYZER_AD9238.bin'
$buildManifest = Join-Path $outputDir 'LATEST_G_AD9238_QSPI_BUILD.txt'
$buildValidator = Join-Path $PSScriptRoot 'Build-QspiBoot.ps1'
if ([string]::IsNullOrWhiteSpace($Image)) {
    $Image = $projectImage
}
if ($ProbeOnly -and $ValidateOnly) {
    throw 'ProbeOnly and ValidateOnly cannot be used together.'
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

if (-not $ProbeOnly) {
    if (-not (Test-Path -LiteralPath $buildValidator -PathType Leaf)) {
        throw "Release-image validator was not found: $buildValidator"
    }

    # Re-discover current RTL/XDC/IP/BD/FSBL inputs and require the existing
    # routed bitstream to match their recorded source digest.  SkipVivado does
    # not accept a stale bitstream: it only re-packages a manifest-matched one.
    & $buildValidator -SkipVivado -Fsbl $Fsbl

    foreach ($requiredReleaseFile in @(
            $projectImage,
            $buildManifest
        )) {
        if (-not (Test-Path -LiteralPath $requiredReleaseFile -PathType Leaf)) {
            throw "Validated release file was not found: $requiredReleaseFile"
        }
    }

    $projectImage = (Resolve-Path -LiteralPath $projectImage).Path
    $requestedImage = (Resolve-Path -LiteralPath $Image).Path
    $projectImageHash =
        (Get-FileHash -LiteralPath $projectImage -Algorithm SHA256).Hash
    $requestedImageHash =
        (Get-FileHash -LiteralPath $requestedImage -Algorithm SHA256).Hash
    $buildManifestText = Get-Content -Raw -LiteralPath $buildManifest
    $manifestBootHashMatch = [regex]::Match(
        $buildManifestText,
        '(?im)^BootSHA256:\s*([0-9A-F]{64})\s*$'
    )
    $manifestSourceDigestMatch = [regex]::Match(
        $buildManifestText,
        '(?im)^SourceDigest:\s*([0-9A-F]{64})\s*$'
    )
    if ((-not $manifestBootHashMatch.Success) -or
        (-not $manifestSourceDigestMatch.Success)) {
        throw "Release manifest is incomplete: $buildManifest"
    }
    $manifestBootHash =
        $manifestBootHashMatch.Groups[1].Value.ToUpperInvariant()
    $releaseSourceDigest =
        $manifestSourceDigestMatch.Groups[1].Value.ToUpperInvariant()
    if ($projectImageHash -ne $manifestBootHash) {
        throw @"
STALE IMAGE BLOCKED: project BOOT image does not match the latest build manifest.
Image:    $projectImageHash
Manifest: $manifestBootHash
"@
    }
    if ($requestedImageHash -ne $projectImageHash) {
        throw @"
WRONG IMAGE BLOCKED: requested QSPI image is not the current project image.
Requested: $requestedImage ($requestedImageHash)
Current:   $projectImage ($projectImageHash)
"@
    }
    $Image = $requestedImage
    Write-Host 'LATEST PROJECT IMAGE VALIDATION PASSED.'
    Write-Host "Source digest: $releaseSourceDigest"
    Write-Host "Image SHA256:  $projectImageHash"
    Write-Host ''

    if ($ValidateOnly) {
        Write-Host 'Validation-only mode: JTAG and QSPI were not accessed.'
        return
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
$Xsdb = Resolve-XilinxTool `
    -ToolName 'xsdb.bat' `
    -Product SDK `
    -Version '2018.3' `
    -SpecifiedPath $Xsdb

foreach ($required in @($Image, $Fsbl, $ProgramFlash, $HwServer, $Xsdb)) {
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
    [PSCustomObject]@{
        ExitCode = $LASTEXITCODE
        Text = ($probeLines | Out-String)
    }
}

function Get-ZynqBootMode {
    param(
        [string]$XsdbTool,
        [string]$ServerUrl
    )

    $modeStageDir = Join-Path $env:TEMP ("BX71_BOOTMODE_{0}" -f $PID)
    $modeScript = Join-Path $modeStageDir 'read_boot_mode.tcl'
    try {
        New-Item -ItemType Directory -Path $modeStageDir -Force | Out-Null
        @"
connect -url $ServerUrl
targets -set -filter {name =~ "ARM Cortex-A9 MPCore #0"}
puts [mrd 0xF800025C 1]
disconnect
exit
"@ | Set-Content -LiteralPath $modeScript -Encoding ASCII

        $modeOutput = @(& $XsdbTool $modeScript 2>&1)
        $modeExitCode = $LASTEXITCODE
        $modeText = $modeOutput | Out-String
        if ($modeExitCode -ne 0) {
            throw "XSDB boot-mode read failed with exit code $modeExitCode.`n$modeText"
        }
        $modeMatch = [regex]::Match(
            $modeText,
            '(?im)F800025C:\s*([0-9A-Fa-f]{8})'
        )
        if (-not $modeMatch.Success) {
            throw "XSDB did not return Zynq boot-mode register F800025C.`n$modeText"
        }
        return $modeMatch.Groups[1].Value.ToUpperInvariant()
    } finally {
        if (Test-Path -LiteralPath $modeStageDir) {
            Remove-Item -LiteralPath $modeStageDir -Recurse -Force
        }
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

$targetId = $armTargetMatches[0].Groups[1].Value
$detectedTarget = $armTargetMatches[0].Groups[2].Value
Write-Host "Validated Zynq target:  $detectedTarget"
Write-Host "ARM DAP target ID:      $targetId"

# Writing QSPI while the board is booted in QSPI/SD/NAND mode can make the
# flash operation unreliable. Require exact JTAG mode after a real power-cycle.
$bootMode = Get-ZynqBootMode -XsdbTool $Xsdb -ServerUrl $Url
Write-Host "Boot-mode F800025C:     0x$bootMode"
if ($bootMode -ne '00000000') {
    throw @"
QSPI PROGRAMMING BLOCKED: the board is not in JTAG boot mode.
Power off the BX71, set its boot switches/jumpers to JTAG, power it on,
and run Build-And-Program-Qspi.cmd again. Detected F800025C=0x$bootMode.
"@
}

if ($ProbeOnly) {
    Write-Host ''
    Write-Host 'QSPI PROGRAMMER AND JTAG BOOT-MODE PROBE PASSED.'
    Write-Host 'No flash contents were changed.'
    return
}

$stageDir = Join-Path $env:TEMP ("BX71_G_QSPI_{0}" -f $PID)
$stagedImage = Join-Path $stageDir 'BOOT_G_ANALYZER_AD9238.bin'
$stagedFsbl = Join-Path $stageDir 'FSBL.elf'
$logFile = Join-Path $outputDir (
    'program_g_qspi_{0}.log' -f (Get-Date -Format 'yyyyMMdd_HHmmss')
)
$manifest = Join-Path $outputDir 'LAST_PROGRAMMED_G_QSPI.txt'
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
    Write-Host 'Programming and verifying the complete G-analyzer QSPI image...'
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
        "SourceDigest: $releaseSourceDigest"
        "BuildManifest: $buildManifest"
        "FlashType:  $FlashType"
        "Target:     $detectedTarget"
        "BootMode:   0x$bootMode"
        "Verified:   $(-not $NoVerify)"
        "Log:        $logFile"
    ) | Set-Content -LiteralPath $manifest -Encoding UTF8

    Write-Host ''
    Write-Host 'QSPI PROGRAMMING AND READBACK VERIFY PASSED.'
    Write-Host "Image length: $($imageItem.Length) bytes"
    Write-Host "SHA256:       $imageHash"
    Write-Host "Log:          $logFile"
    Write-Host ''
    Write-Host 'Cold-boot proof still required: power off, select QSPI boot, then power on.'
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
