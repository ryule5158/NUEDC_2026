# Common Xilinx tool discovery for Windows PowerShell 5.1.
#
# The project must not depend on a specific drive letter or SDK workspace.
# Tools are resolved in this order:
#   1. Explicit command-line path.
#   2. XILINX_VIVADO / XILINX_SDK environment variables.
#   3. Current PATH.
#   4. Common Xilinx layouts on every mounted local drive.

Set-StrictMode -Version 2.0

function Resolve-XilinxTool {
    [CmdletBinding()]
    param(
        [Parameter(Mandatory = $true)]
        [string]$ToolName,

        [ValidateSet('Vivado', 'SDK')]
        [string]$Product,

        [string]$Version = '2018.3',
        [string]$SpecifiedPath
    )

    if (-not [string]::IsNullOrWhiteSpace($SpecifiedPath)) {
        if (-not (Test-Path -LiteralPath $SpecifiedPath -PathType Leaf)) {
            throw "$ToolName was not found at the explicitly specified path: $SpecifiedPath"
        }
        return (Resolve-Path -LiteralPath $SpecifiedPath).Path
    }

    $candidates = New-Object System.Collections.Generic.List[string]
    $seen = @{}

    function Add-XilinxToolCandidate {
        param([string]$Path)

        if ([string]::IsNullOrWhiteSpace($Path)) {
            return
        }

        $key = $Path.ToLowerInvariant()
        if (-not $seen.ContainsKey($key)) {
            $seen[$key] = $true
            $candidates.Add($Path)
        }
    }

    $environmentRoot = if ($Product -eq 'Vivado') {
        $env:XILINX_VIVADO
    } else {
        $env:XILINX_SDK
    }
    if (-not [string]::IsNullOrWhiteSpace($environmentRoot)) {
        Add-XilinxToolCandidate (Join-Path $environmentRoot "bin\$ToolName")
    }

    $pathCommand = Get-Command $ToolName -ErrorAction SilentlyContinue |
        Select-Object -First 1
    if ($null -ne $pathCommand) {
        Add-XilinxToolCandidate $pathCommand.Source
    }

    $relativeLayouts = @(
        "Xilinx\$Product\$Version\bin\$ToolName",
        "$Product\$Version\bin\$ToolName",
        "Tools\Xilinx\$Product\$Version\bin\$ToolName"
    )
    foreach ($drive in (Get-PSDrive -PSProvider FileSystem -ErrorAction SilentlyContinue)) {
        foreach ($relativeLayout in $relativeLayouts) {
            Add-XilinxToolCandidate (Join-Path $drive.Root $relativeLayout)
        }
    }

    foreach ($candidate in $candidates) {
        if (Test-Path -LiteralPath $candidate -PathType Leaf) {
            return (Resolve-Path -LiteralPath $candidate).Path
        }
    }

    $searched = $candidates -join [Environment]::NewLine
    throw @"
$ToolName $Version was not found.
Install Xilinx $Product $Version, run its settings64.bat before this script,
or pass the full tool path explicitly.
Searched:
$searched
"@
}

