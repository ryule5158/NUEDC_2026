# 构建开关及Xilinx工具、FSBL的可选显式路径。
[CmdletBinding()]
param(
    [switch]$SkipVivado,
    [string]$Fsbl,
    [string]$Vivado,
    [string]$Bootgen
)

$ErrorActionPreference = 'Stop' # 任一命令失败即停止打包。

. (Join-Path $PSScriptRoot 'XilinxTools.ps1')

# 自动定位任意盘符/常见安装目录下的Vivado 2018.3和配套bootgen。
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

# FPGA工程根目录、构建入口和输出文件路径。
$fpgaRoot = Split-Path -Parent $PSScriptRoot
$buildTcl = Join-Path $PSScriptRoot 'build_ps7_project.tcl'
$bitFile = Join-Path $fpgaRoot 'output\BX71_V4_U3_tail_AD9708_PS7.bit'
$outputDir = Join-Path $fpgaRoot 'output'
$bootFile = Join-Path $outputDir 'BOOT_PS7.bin'
$vivadoProjectDir = Join-Path $fpgaRoot 'Vivado\BX71_AD9708_AD9280_PS7'

# 未显式指定时使用FPGA工程自己的启动支持文件。
if ([string]::IsNullOrWhiteSpace($Fsbl)) {
    $Fsbl = Join-Path $fpgaRoot '启动支持\boot\FSBL.elf'
}

# 构建开始前一次性检查所有必需文件。
foreach ($required in @($Vivado, $Bootgen, $Fsbl, $buildTcl)) {
    if (-not (Test-Path -LiteralPath $required)) {
        throw "Required file not found: $required"
    }
}

# Vivado 2018.3不能通过目录联接自动创建上级目录，先在真实工程中建好固定目录。
if (-not $SkipVivado) {
    New-Item -ItemType Directory -Path $vivadoProjectDir -Force | Out-Null
}

# 工具使用的ASCII工程入口，规避Vivado 2018.3的中文路径子进程故障。
$toolFpgaRoot = $fpgaRoot
$asciiLink = $null
if ($fpgaRoot -match '[^\x00-\x7F]') {
    # 名称刻意保持很短，避免Vivado 2018.3运行目录触发Windows MAX_PATH限制。
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
    # 将工程内路径映射到工具可访问的ASCII入口。
    $toolBuildTcl = Join-Path $toolFpgaRoot 'scripts\build_ps7_project.tcl'

    # 默认先生成含PS7的位流，调试时可选择复用已有位流。
    if (-not $SkipVivado) {
        Write-Host 'Building the PS7-integrated bitstream...'
        & $Vivado -mode batch -source $toolBuildTcl
        if ($LASTEXITCODE -ne 0) {
            throw "Vivado failed with exit code $LASTEXITCODE"
        }
    }

} finally {
    # 无论成功或失败都清理临时目录联接。
    if (($null -ne $asciiLink) -and (Test-Path -LiteralPath $asciiLink)) {
        [System.IO.Directory]::Delete($asciiLink)
    }
}

if (-not (Test-Path -LiteralPath $bitFile -PathType Leaf)) {
    throw "PS7-integrated bitstream not found: $bitFile"
}

# Bootgen 2018.3会把中文路径静默转换成“?”，并可能错误返回0。
# 因此把FSBL、位流、BIF和生成结果全部放入纯ASCII临时目录。
$bootStageDir = Join-Path $env:TEMP ("BX71_BOOT_{0}" -f $PID)
$stagedFsbl = Join-Path $bootStageDir 'FSBL.elf'
$stagedBit = Join-Path $bootStageDir 'design.bit'
$stagedBif = Join-Path $bootStageDir 'boot.bif'
$stagedBoot = Join-Path $bootStageDir 'BOOT_PS7.bin'
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

    if (Test-Path -LiteralPath $stagedBoot) {
        Remove-Item -LiteralPath $stagedBoot -Force
    }

    Write-Host 'Packaging BOOT_PS7.bin...'
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
        throw 'bootgen returned success but did not create BOOT_PS7.bin'
    }

    $stagedBootItem = Get-Item -LiteralPath $stagedBoot
    if ($stagedBootItem.Length -le 0) {
        throw 'bootgen created an empty BOOT_PS7.bin'
    }

    New-Item -ItemType Directory -Path $outputDir -Force | Out-Null
    Copy-Item -LiteralPath $stagedBoot -Destination $bootFile -Force
} finally {
    if (Test-Path -LiteralPath $bootStageDir) {
        Remove-Item -LiteralPath $bootStageDir -Recurse -Force
    }
}

# 输出镜像长度和SHA-256，便于烧录后完整读回校验。
$bootItem = Get-Item -LiteralPath $bootFile
$bitHash = (Get-FileHash -Algorithm SHA256 -LiteralPath $bitFile).Hash
$bootHash = (Get-FileHash -Algorithm SHA256 -LiteralPath $bootFile).Hash
$hashFile = Join-Path $outputDir 'SHA256SUMS.txt'
@(
    "$bitHash  BX71_V4_U3_tail_AD9708_PS7.bit"
    "$bootHash  BOOT_PS7.bin"
) | Set-Content -LiteralPath $hashFile -Encoding ASCII
Write-Host ''
Write-Host 'QSPI boot image is ready:'
Write-Host "  $($bootItem.FullName)"
Write-Host "  Length: $($bootItem.Length) bytes"
Write-Host "  SHA256: $bootHash"
Write-Host ''
Write-Host 'To program without an SDK workspace, double-click scripts\Program-Qspi.cmd.'
