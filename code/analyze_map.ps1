param(
    [string]$MapFile = ""
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

# Default map file path (relative to this script)
if ($MapFile -eq "") {
    $MapFile = Join-Path $PSScriptRoot 'project\MDK_V5\listings\code.map'
}

if (-not (Test-Path -LiteralPath $MapFile)) {
    Write-Host "[ERROR] Map file not found: $MapFile"
    exit 1
}

$lines = Get-Content -LiteralPath $MapFile

$flashSize = 0; $flashMax = 0
$ramSize   = 0; $ramMax   = 0

foreach ($line in $lines) {
    if ($line -match 'LR_IROM1.*Size:\s+(0x[0-9A-Fa-f]+).*Max:\s+(0x[0-9A-Fa-f]+)') {
        $flashSize = [Convert]::ToInt64($Matches[1].Substring(2), 16)
        $flashMax  = [Convert]::ToInt64($Matches[2].Substring(2), 16)
    }
    elseif ($line -match 'RW_IRAM1.*Size:\s+(0x[0-9A-Fa-f]+).*Max:\s+(0x[0-9A-Fa-f]+)') {
        $ramSize = [Convert]::ToInt64($Matches[1].Substring(2), 16)
        $ramMax  = [Convert]::ToInt64($Matches[2].Substring(2), 16)
    }
}

if ($flashMax -eq 0 -or $ramMax -eq 0) {
    Write-Host "[ERROR] Could not parse flash/RAM regions from: $MapFile"
    exit 1
}

function Format-Bytes {
    param([double]$Bytes)
    if ($Bytes -ge 1048576) { return ('{0:N2} MB' -f ($Bytes / 1048576)) }
    if ($Bytes -ge 1024)    { return ('{0:N2} KB' -f ($Bytes / 1024)) }
    return ('{0} B' -f $Bytes)
}

$flashPct = [math]::Round(($flashSize * 100.0) / $flashMax, 2)
$ramPct   = [math]::Round(($ramSize   * 100.0) / $ramMax, 2)

$barLen = 30
$flashBar = ('#' * [math]::Min($barLen, [int][math]::Floor($flashPct * $barLen / 100))).PadRight($barLen, '.')
$ramBar   = ('#' * [math]::Min($barLen, [int][math]::Floor($ramPct   * $barLen / 100))).PadRight($barLen, '.')

Write-Host ""
Write-Host "============ Memory Usage (from map) ============"
Write-Host ("  Flash (ROM): {0,10} / {1,-10}  {2,6}%  [{3}]" -f (Format-Bytes $flashSize), (Format-Bytes $flashMax), $flashPct, $flashBar)
Write-Host ("  RAM  (SRAM): {0,10} / {1,-10}  {2,6}%  [{3}]" -f (Format-Bytes $ramSize), (Format-Bytes $ramMax), $ramPct, $ramBar)
Write-Host "================================================="
Write-Host ""