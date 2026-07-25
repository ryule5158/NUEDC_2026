param(
    [string]$SysConfigToolRoot = $env:NUEDC_TI_SYSCONFIG_ROOT,
    [string]$Mspm0SdkRoot = $env:NUEDC_TI_MSPM0_SDK_ROOT
)

$ErrorActionPreference = "Stop"

$projectRoot = Split-Path -Parent $MyInvocation.MyCommand.Path

function Resolve-TiPath ([string]$EnvVar, [string]$VersionDir, [string]$MarkerFile) {
    if (-not [string]::IsNullOrWhiteSpace($EnvVar)) {
        $testPath = Join-Path $EnvVar $MarkerFile
        if (Test-Path -LiteralPath $testPath) { return $EnvVar }
        Write-Warning "$EnvVar is set but '$MarkerFile' not found, trying drives..."
    }
    foreach ($drv in @('E', 'D', 'C')) {
        $testPath = "${drv}:\ti\$VersionDir\$MarkerFile"
        if (Test-Path -LiteralPath $testPath) { return "${drv}:\ti\$VersionDir" }
    }
    throw "Cannot find ti/$VersionDir under C:/ti, D:/ti, E:/ti. Set `$env:$EnvVar."
}
if ([string]::IsNullOrWhiteSpace($SysConfigToolRoot)) {
    $SysConfigToolRoot = Resolve-TiPath $env:NUEDC_TI_SYSCONFIG_ROOT "sysconfig_1.26.2" "sysconfig_gui.bat"
}
if ([string]::IsNullOrWhiteSpace($Mspm0SdkRoot)) {
    $Mspm0SdkRoot = Resolve-TiPath $env:NUEDC_TI_MSPM0_SDK_ROOT "mspm0_sdk_2_10_00_04" ".metadata/product.json"
}

$sysConfigGui = Join-Path $SysConfigToolRoot "sysconfig_gui.bat"
$productJson = Join-Path $Mspm0SdkRoot ".metadata\product.json"
$configuration = Join-Path $projectRoot "SysConfig\MSP_LITO_G3507_Board.syscfg"
$generatedDirectory = Join-Path $projectRoot "SysConfig\Generated"

if (-not (Test-Path -LiteralPath $sysConfigGui)) {
    throw "TI SysConfig GUI not found: $sysConfigGui"
}
if (-not (Test-Path -LiteralPath $productJson)) {
    throw "MSPM0 SDK product metadata not found: $productJson"
}
if (-not (Test-Path -LiteralPath $configuration)) {
    throw "SysConfig project not found: $configuration"
}

New-Item -ItemType Directory -Path $generatedDirectory -Force | Out-Null

& $sysConfigGui `
    --product $productJson `
    --compiler keil `
    --output $generatedDirectory `
    $configuration
