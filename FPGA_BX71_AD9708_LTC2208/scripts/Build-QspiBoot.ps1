# Rebuild the G-analyzer PS7 bitstream and package a persistent QSPI image.
[CmdletBinding()]
param(
    [switch]$SkipVivado,
    [switch]$ValidateOnly,
    [string]$Fsbl,
    [string]$Vivado,
    [string]$Bootgen
)

Set-StrictMode -Version 2.0
$ErrorActionPreference = 'Stop'

if ($ValidateOnly) {
    $SkipVivado = $true
}

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
$releaseManifest = Join-Path $PSScriptRoot `
    'ltc2208_release_inputs.txt'
$outputDir = Join-Path $fpgaRoot 'output'
$bitFile = Join-Path $outputDir 'BX71_G_ANALYZER_LTC2208_PS7.bit'
$bootFile = Join-Path $outputDir 'BOOT_G_ANALYZER_LTC2208.bin'
$hashFile = Join-Path $outputDir 'SHA256SUMS_G_LTC2208_QSPI.txt'
$manifestFile = Join-Path $outputDir 'LATEST_G_LTC2208_QSPI_BUILD.txt'
$sourceSnapshotFile = Join-Path $outputDir 'SOURCE_SNAPSHOT_G_LTC2208_QSPI.txt'

function Get-ReleaseInputSnapshot {
    param(
        [string]$Root,
        [string]$FsblPath,
        [string]$BuildScriptPath,
        [string]$ReleaseManifestPath
    )

    if (-not (Test-Path -LiteralPath $ReleaseManifestPath -PathType Leaf)) {
        throw "Release input manifest was not found: $ReleaseManifestPath"
    }
    $rootPrefix = [System.IO.Path]::GetFullPath($Root).TrimEnd('\') + '\'
    $relativePaths = @(
        Get-Content -LiteralPath $ReleaseManifestPath |
            ForEach-Object { $_.Trim() } |
            Where-Object { ($_ -ne '') -and (-not $_.StartsWith('#')) }
    )
    if ($relativePaths.Count -eq 0) {
        throw 'The release input manifest is empty.'
    }
    if (@($relativePaths | Sort-Object -Unique).Count -ne
        $relativePaths.Count) {
        throw 'The release input manifest contains duplicate paths.'
    }

    $files = @()
    foreach ($relativePath in $relativePaths) {
        $candidate = Join-Path $Root $relativePath.Replace('/', '\')
        $fullName = [System.IO.Path]::GetFullPath($candidate)
        if (-not $fullName.StartsWith(
                $rootPrefix,
                [System.StringComparison]::OrdinalIgnoreCase)) {
            throw "Release input escapes the project root: $relativePath"
        }
        if (-not (Test-Path -LiteralPath $fullName -PathType Leaf)) {
            throw "Required release input was not found: $relativePath"
        }
        if ($relativePath -match '(?i)ad9238') {
            throw "AD9238 input is forbidden in the LTC2208 release: $relativePath"
        }
        $files += Get-Item -LiteralPath $fullName
    }
    $files += Get-Item -LiteralPath $ReleaseManifestPath
    $files += Get-Item -LiteralPath $BuildScriptPath
    $files += Get-Item -LiteralPath $FsblPath
    $files = @($files | Sort-Object -Property FullName -Unique)

    $fsblFullName = [System.IO.Path]::GetFullPath($FsblPath)
    $lines = @(
        foreach ($file in $files) {
            $fullName = [System.IO.Path]::GetFullPath($file.FullName)
            if ($fullName.Equals(
                    $fsblFullName,
                    [System.StringComparison]::OrdinalIgnoreCase)) {
                # Keep the ASCII manifest reproducible even when the project
                # stores FSBL.elf below a non-ASCII directory name.
                $relativeName = 'boot/FSBL.elf'
            } elseif ($fullName.StartsWith(
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

foreach ($required in @(
        $Vivado, $Bootgen, $Fsbl, $buildTcl, $releaseManifest)) {
    if (-not (Test-Path -LiteralPath $required -PathType Leaf)) {
        throw "Required file not found: $required"
    }
}

$sourceSnapshotBefore = Get-ReleaseInputSnapshot `
    -Root $fpgaRoot `
    -FsblPath $Fsbl `
    -BuildScriptPath $buildTcl `
    -ReleaseManifestPath $releaseManifest
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
# non-ASCII characters. A junction is not sufficient because Tcl file
# normalization resolves it back to the Unicode target. Use a temporary subst
# drive so every Vivado parent and child process sees a genuinely ASCII path.
$toolFpgaRoot = $fpgaRoot
$asciiDrive = $null
$substExe = Join-Path $env:SystemRoot 'System32\subst.exe'
if ($fpgaRoot -match '[^\x00-\x7F]') {
    if (-not (Test-Path -LiteralPath $substExe -PathType Leaf)) {
        throw "Windows subst.exe was not found: $substExe"
    }
    $usedDriveNames = @(
        Get-PSDrive -PSProvider FileSystem | ForEach-Object {
            "$($_.Name):"
        }
    )
    foreach ($candidate in @('Z:', 'Y:', 'X:', 'W:', 'V:', 'U:', 'T:', 'S:', 'R:', 'Q:', 'P:')) {
        if ($candidate -notin $usedDriveNames) {
            $asciiDrive = $candidate
            break
        }
    }
    if ($null -eq $asciiDrive) {
        throw 'No free drive letter is available for the ASCII Vivado build path.'
    }
    $substOutput = @(& $substExe $asciiDrive $fpgaRoot 2>&1)
    if ($LASTEXITCODE -ne 0) {
        throw "Unable to create ASCII build drive $asciiDrive`n$($substOutput | Out-String)"
    }
    $toolFpgaRoot = "$asciiDrive\"
    if (-not (Test-Path -LiteralPath $toolFpgaRoot -PathType Container)) {
        throw "ASCII build drive was not mounted: $toolFpgaRoot"
    }
    Write-Host "Vivado ASCII build root: $toolFpgaRoot"
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
    if ($null -ne $asciiDrive) {
        $substRemoveOutput = @(& $substExe $asciiDrive /D 2>&1)
        if ($LASTEXITCODE -ne 0) {
            Write-Warning "Unable to remove temporary build drive $asciiDrive`n$($substRemoveOutput | Out-String)"
        }
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
    -BuildScriptPath $buildTcl `
    -ReleaseManifestPath $releaseManifest
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
        throw 'SkipVivado requires a previous LATEST_G_LTC2208_QSPI_BUILD.txt manifest.'
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

    if ($ValidateOnly) {
        foreach ($releaseFile in @(
                $bootFile,
                $hashFile,
                $sourceSnapshotFile)) {
            if (-not (Test-Path -LiteralPath $releaseFile -PathType Leaf)) {
                throw "Validation failed: release file was not found: $releaseFile"
            }
        }

        $bootHash = (Get-FileHash -Algorithm SHA256 -LiteralPath $bootFile).Hash
        $bootHashMatch = [regex]::Match(
            $previousManifest,
            '(?im)^BootSHA256:\s*([0-9A-F]{64})\s*$'
        )
        if ((-not $bootHashMatch.Success) -or
            ($bootHashMatch.Groups[1].Value -ne $bootHash)) {
            throw 'Validation failed: the existing BOOT image does not match the manifest.'
        }

        $hashText = Get-Content -Raw -LiteralPath $hashFile
        if (($hashText -notmatch [regex]::Escape($bitHash)) -or
            ($hashText -notmatch [regex]::Escape($bootHash))) {
            throw 'Validation failed: SHA256SUMS does not match the release artifacts.'
        }

        Write-Host 'LTC2208 QSPI RELEASE VALIDATION PASSED.'
        Write-Host "Source digest: $($sourceSnapshotAfter.Digest)"
        Write-Host "Bit SHA256:   $bitHash"
        Write-Host "BOOT SHA256:  $bootHash"
        return
    }
}
# Keep every bootgen input/output under a short ASCII path. Vivado 2018.3
# bootgen can silently corrupt non-ASCII paths and still return exit code zero.
$bootStageDir = Join-Path $env:TEMP ("BX71_G_BOOT_{0}" -f $PID)
$stagedFsbl = Join-Path $bootStageDir 'FSBL.elf'
$stagedBit = Join-Path $bootStageDir 'design.bit'
$stagedBif = Join-Path $bootStageDir 'boot.bif'
$stagedBoot = Join-Path $bootStageDir 'BOOT_G_ANALYZER_LTC2208.bin'
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

    Write-Host 'Packaging BOOT_G_ANALYZER_LTC2208.bin...'
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
        throw 'bootgen returned success but did not create BOOT_G_ANALYZER_LTC2208.bin'
    }
    if ((Get-Item -LiteralPath $stagedBoot).Length -le 0) {
        throw 'bootgen created an empty BOOT_G_ANALYZER_LTC2208.bin'
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
    "$bitHash  BX71_G_ANALYZER_LTC2208_PS7.bit"
    "$bootHash  BOOT_G_ANALYZER_LTC2208.bin"
) | Set-Content -LiteralPath $hashFile -Encoding ASCII
@(
    "BuildTime:  $(Get-Date -Format 'yyyy-MM-dd HH:mm:ss zzz')"
    "SourceCount:  $($sourceSnapshotAfter.Count)"
    "SourceDigest: $($sourceSnapshotAfter.Digest)"
    "SourceList:   SOURCE_SNAPSHOT_G_LTC2208_QSPI.txt"
    "Bitstream:  BX71_G_ANALYZER_LTC2208_PS7.bit"
    "BitLength:  $($bitItem.Length)"
    "BitSHA256:  $bitHash"
    "BootImage:  BOOT_G_ANALYZER_LTC2208.bin"
    "BootLength: $($bootItem.Length)"
    "BootSHA256: $bootHash"
) | Set-Content -LiteralPath $manifestFile -Encoding ASCII

Write-Host ''
Write-Host 'FRESH G-ANALYZER LTC2208 QSPI IMAGE READY.'
Write-Host "Bitstream: $bitFile"
Write-Host "BOOT image: $bootFile"
Write-Host "Length:     $($bootItem.Length) bytes"
Write-Host "SHA256:     $bootHash"
