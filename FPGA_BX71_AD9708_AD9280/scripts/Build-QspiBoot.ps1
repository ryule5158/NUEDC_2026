# Rebuild the G-analyzer PS7 bitstream and package a persistent QSPI image.
[CmdletBinding()]
param(
    [switch]$SkipVivado,
    [string]$Fsbl,
    [string]$Vivado,
    [string]$Bootgen
)

Set-StrictMode -Version 2.0
$ErrorActionPreference = 'Stop'

. (Join-Path $PSScriptRoot 'XilinxTools.ps1')

$Vivado = Resolve-XilinxTool `
    -ToolName 'vivado.bat' `
    -Product Vivado `
    -Version '2018.3' `
    -SpecifiedPath $Vivado
$Bootgen = Resolve-XilinxTool `
    -ToolName 'bootgen.bat' `
    -Product SDK `
    -Version '2018.3' `
    -SpecifiedPath $Bootgen

$fpgaRoot = Split-Path -Parent $PSScriptRoot
$buildTcl = Join-Path $PSScriptRoot 'build_ps7_project.tcl'
$outputDir = Join-Path $fpgaRoot 'output'
$bitFile = Join-Path $outputDir 'BX71_G_ANALYZER_PS7.bit'
$bootFile = Join-Path $outputDir 'BOOT_G_ANALYZER.bin'
$hashFile = Join-Path $outputDir 'SHA256SUMS_G_QSPI.txt'
$manifestFile = Join-Path $outputDir 'LATEST_G_QSPI_BUILD.txt'
$sourceSnapshotFile = Join-Path $outputDir 'SOURCE_SNAPSHOT_G_QSPI.txt'

function Get-ReleaseInputSnapshot {
    param(
        [string]$Root,
        [string]$FsblPath,
        [string]$BuildScriptPath
    )

    $extensions = @(
        '.v', '.sv', '.vhd', '.vhdl', '.vh', '.svh',
        '.xdc', '.bd', '.xci', '.xcix', '.dcp', '.edf', '.edif'
    )
    $files = @()
    foreach ($relativeRoot in @('rtl', 'rtl_ps7', 'constraints', 'ip')) {
        $inputRoot = Join-Path $Root $relativeRoot
        if (Test-Path -LiteralPath $inputRoot -PathType Container) {
            $files += Get-ChildItem -LiteralPath $inputRoot -Recurse -File |
                Where-Object {
                    $extensions -contains $_.Extension.ToLowerInvariant()
                }
        }
    }
    $bdSource = Join-Path $Root 'ps7\sys.bd'
    if (-not (Test-Path -LiteralPath $bdSource -PathType Leaf)) {
        throw "Required PS7 source was not found: $bdSource"
    }
    $files += Get-Item -LiteralPath $bdSource
    $files += Get-Item -LiteralPath $BuildScriptPath
    $files += Get-Item -LiteralPath $FsblPath
    $files = @($files | Sort-Object -Property FullName -Unique)
    if ($files.Count -eq 0) {
        throw 'No release inputs were found for source fingerprinting.'
    }

    $rootPrefix = [System.IO.Path]::GetFullPath($Root).TrimEnd('\') + '\'
    $lines = @(
        foreach ($file in $files) {
            $fullName = [System.IO.Path]::GetFullPath($file.FullName)
            if ($fullName.StartsWith(
                    $rootPrefix,
                    [System.StringComparison]::OrdinalIgnoreCase)) {
                $relativeName = $fullName.Substring($rootPrefix.Length)
            } else {
                $relativeName = 'external/' + $file.Name
            }
            $relativeName = $relativeName.Replace('\', '/')
            $fileHash = (Get-FileHash -LiteralPath $fullName -Algorithm SHA256).Hash
            "$fileHash  $relativeName"
        }
    )
    $payload = ($lines -join "`n") + "`n"
    $sha256 = [System.Security.Cryptography.SHA256]::Create()
    try {
        $digestBytes = $sha256.ComputeHash(
            [System.Text.Encoding]::UTF8.GetBytes($payload)
        )
    } finally {
        $sha256.Dispose()
    }
    $digest = ([System.BitConverter]::ToString($digestBytes)).Replace('-', '')
    [PSCustomObject]@{
        Count = $files.Count
        Digest = $digest
        Lines = $lines
    }
}
if ([string]::IsNullOrWhiteSpace($Fsbl)) {
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
        throw "Unable to select a unique project FSBL. Pass -Fsbl explicitly.`n$candidateList"
    }
    $Fsbl = $fsblCandidates[0].FullName
}

foreach ($required in @($Vivado, $Bootgen, $Fsbl, $buildTcl)) {
    if (-not (Test-Path -LiteralPath $required -PathType Leaf)) {
        throw "Required file not found: $required"
    }
}

$sourceSnapshotBefore = Get-ReleaseInputSnapshot `
    -Root $fpgaRoot `
    -FsblPath $Fsbl `
    -BuildScriptPath $buildTcl
New-Item -ItemType Directory -Path $outputDir -Force | Out-Null
$buildStartedUtc = [DateTime]::UtcNow
if (-not $SkipVivado) {
    # Delete only named deliverables. If Vivado fails, no stale file remains
    # available for bootgen or the programming stage.
    foreach ($staleOutput in @($bitFile, $bootFile, $hashFile, $manifestFile, $sourceSnapshotFile)) {
        if (Test-Path -LiteralPath $staleOutput) {
            Remove-Item -LiteralPath $staleOutput -Force
        }
    }
}

# Vivado 2018.3 child processes can fail when the project path contains
# non-ASCII characters. Use a short temporary junction only when required.
$toolFpgaRoot = $fpgaRoot
$asciiLink = $null
if ($fpgaRoot -match '[^\x00-\x7F]') {
    $asciiLink = Join-Path $env:TEMP 'NF'
    if (Test-Path -LiteralPath $asciiLink) {
        $linkItem = Get-Item -LiteralPath $asciiLink -Force
        if (($linkItem.LinkType -ne 'Junction') -or
            ($linkItem.Target -notcontains $fpgaRoot)) {
            throw "ASCII build path is occupied: $asciiLink"
        }
        [System.IO.Directory]::Delete($asciiLink)
    }
    New-Item -ItemType Junction -Path $asciiLink -Target $fpgaRoot | Out-Null
    $toolFpgaRoot = $asciiLink
}

try {
    $toolBuildTcl = Join-Path $toolFpgaRoot 'scripts\build_ps7_project.tcl'
    if (-not $SkipVivado) {
        Write-Host 'Rebuilding the clean G-analyzer PS7 bitstream...'
        & $Vivado -mode batch -source $toolBuildTcl
        $vivadoExitCode = $LASTEXITCODE
        if ($vivadoExitCode -ne 0) {
            throw "Vivado failed with exit code $vivadoExitCode"
        }
    }
} finally {
    if (($null -ne $asciiLink) -and (Test-Path -LiteralPath $asciiLink)) {
        [System.IO.Directory]::Delete($asciiLink)
    }
}

if (-not (Test-Path -LiteralPath $bitFile -PathType Leaf)) {
    throw "Fresh G-analyzer PS7 bitstream not found: $bitFile"
}
$bitItem = Get-Item -LiteralPath $bitFile
if (($bitItem.Length -le 0) -or
    ((-not $SkipVivado) -and
     ($bitItem.LastWriteTimeUtc -lt $buildStartedUtc.AddSeconds(-2)))) {
    throw "The bitstream is empty or older than this build: $bitFile"
}
$sourceSnapshotAfter = Get-ReleaseInputSnapshot `
    -Root $fpgaRoot `
    -FsblPath $Fsbl `
    -BuildScriptPath $buildTcl
if ($sourceSnapshotAfter.Digest -ne $sourceSnapshotBefore.Digest) {
    throw @"
FPGA release inputs changed while Vivado was running.
No image will be packaged or programmed. Re-run the full build from a stable tree.
Before: $($sourceSnapshotBefore.Digest)
After:  $($sourceSnapshotAfter.Digest)
"@
}
$bitHash = (Get-FileHash -Algorithm SHA256 -LiteralPath $bitFile).Hash
if ($SkipVivado) {
    if (-not (Test-Path -LiteralPath $manifestFile -PathType Leaf)) {
        throw 'SkipVivado requires a previous LATEST_G_QSPI_BUILD.txt manifest.'
    }
    $previousManifest = Get-Content -Raw -LiteralPath $manifestFile
    $digestMatch = [regex]::Match(
        $previousManifest,
        '(?im)^SourceDigest:\s*([0-9A-F]{64})\s*$'
    )
    $bitHashMatch = [regex]::Match(
        $previousManifest,
        '(?im)^BitSHA256:\s*([0-9A-F]{64})\s*$'
    )
    if ((-not $digestMatch.Success) -or
        ($digestMatch.Groups[1].Value -ne $sourceSnapshotAfter.Digest)) {
        throw 'SkipVivado blocked: current sources do not match the previous build.'
    }
    if ((-not $bitHashMatch.Success) -or
        ($bitHashMatch.Groups[1].Value -ne $bitHash)) {
        throw 'SkipVivado blocked: the existing bitstream does not match the manifest.'
    }
}
# Keep every bootgen input/output under a short ASCII path. Vivado 2018.3
# bootgen can silently corrupt non-ASCII paths and still return exit code zero.
$bootStageDir = Join-Path $env:TEMP ("BX71_G_BOOT_{0}" -f $PID)
$stagedFsbl = Join-Path $bootStageDir 'FSBL.elf'
$stagedBit = Join-Path $bootStageDir 'design.bit'
$stagedBif = Join-Path $bootStageDir 'boot.bif'
$stagedBoot = Join-Path $bootStageDir 'BOOT_G_ANALYZER.bin'
try {
    New-Item -ItemType Directory -Path $bootStageDir -Force | Out-Null
    Copy-Item -LiteralPath $Fsbl -Destination $stagedFsbl -Force
    Copy-Item -LiteralPath $bitFile -Destination $stagedBit -Force

    $fsblBif = $stagedFsbl.Replace('\', '/')
    $bitBif = $stagedBit.Replace('\', '/')
    @"
the_ROM_image:
{
  [bootloader] $fsblBif
  $bitBif
}
"@ | Set-Content -LiteralPath $stagedBif -Encoding ASCII

    Write-Host 'Packaging BOOT_G_ANALYZER.bin...'
    $bootgenOutput = @(
        & $Bootgen -image $stagedBif -arch zynq -o $stagedBoot -w on 2>&1
    )
    $bootgenExitCode = $LASTEXITCODE
    $bootgenOutput | ForEach-Object { Write-Host $_ }
    $bootgenText = $bootgenOutput | Out-String
    if ($bootgenExitCode -ne 0) {
        throw "bootgen failed with exit code $bootgenExitCode"
    }
    if ($bootgenText -match '(?im)^\s*(\[ERROR\]|ERROR:)') {
        throw "bootgen reported an error even though its exit code was 0.`n$bootgenText"
    }
    if (-not (Test-Path -LiteralPath $stagedBoot -PathType Leaf)) {
        throw 'bootgen returned success but did not create BOOT_G_ANALYZER.bin'
    }
    if ((Get-Item -LiteralPath $stagedBoot).Length -le 0) {
        throw 'bootgen created an empty BOOT_G_ANALYZER.bin'
    }

    Copy-Item -LiteralPath $stagedBoot -Destination $bootFile -Force
} finally {
    if (Test-Path -LiteralPath $bootStageDir) {
        Remove-Item -LiteralPath $bootStageDir -Recurse -Force
    }
}

$bootItem = Get-Item -LiteralPath $bootFile
$bootHash = (Get-FileHash -Algorithm SHA256 -LiteralPath $bootFile).Hash
@(
    "SourceDigest: $($sourceSnapshotAfter.Digest)"
    "SourceCount:  $($sourceSnapshotAfter.Count)"
    ''
) + $sourceSnapshotAfter.Lines |
    Set-Content -LiteralPath $sourceSnapshotFile -Encoding ASCII
@(
    "$bitHash  BX71_G_ANALYZER_PS7.bit"
    "$bootHash  BOOT_G_ANALYZER.bin"
) | Set-Content -LiteralPath $hashFile -Encoding ASCII
@(
    "BuildTime:  $(Get-Date -Format 'yyyy-MM-dd HH:mm:ss zzz')"
    "SourceCount:  $($sourceSnapshotAfter.Count)"
    "SourceDigest: $($sourceSnapshotAfter.Digest)"
    "SourceList:   SOURCE_SNAPSHOT_G_QSPI.txt"
    "Bitstream:  BX71_G_ANALYZER_PS7.bit"
    "BitLength:  $($bitItem.Length)"
    "BitSHA256:  $bitHash"
    "BootImage:  BOOT_G_ANALYZER.bin"
    "BootLength: $($bootItem.Length)"
    "BootSHA256: $bootHash"
) | Set-Content -LiteralPath $manifestFile -Encoding ASCII

Write-Host ''
Write-Host 'FRESH G-ANALYZER QSPI IMAGE READY.'
Write-Host "Bitstream: $bitFile"
Write-Host "BOOT image: $bootFile"
Write-Host "Length:     $($bootItem.Length) bytes"
Write-Host "SHA256:     $bootHash"
