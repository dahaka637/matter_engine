param(
    [Parameter(Mandatory = $true)]
    [string]$Path,

    [ValidateSet("auto", "box", "sphere", "capsule", "manual_compound", "static_mesh")]
    [string]$CollisionMode = "auto",

    [ValidateRange(0, 32)]
    [int]$HullBudget = 0
)

$resolvedPath = (Resolve-Path -LiteralPath $Path).Path
$bytes = [System.IO.File]::ReadAllBytes($resolvedPath)
if ($bytes.Length -lt 20 -or
    [BitConverter]::ToUInt32($bytes, 0) -ne 0x46546C67) {
    throw "Arquivo GLB invalido: $resolvedPath"
}

$jsonLength = [BitConverter]::ToUInt32($bytes, 12)
$jsonType = [BitConverter]::ToUInt32($bytes, 16)
if ($jsonType -ne 0x4E4F534A) {
    throw "O primeiro chunk do GLB nao e JSON: $resolvedPath"
}

$jsonText = [Text.Encoding]::UTF8.GetString(
    $bytes, 20, $jsonLength).TrimEnd(" ", [char]0)
$document = $jsonText | ConvertFrom-Json
$updatedNodes = 0
foreach ($node in $document.nodes) {
    if ($null -eq $node.extras -or
        $null -eq $node.extras.physical_material) {
        continue
    }
    $node.extras | Add-Member -NotePropertyName collision_mode `
        -NotePropertyValue $CollisionMode -Force
    if ($HullBudget -gt 0 -and $CollisionMode -eq "auto") {
        $node.extras | Add-Member -NotePropertyName collision_hulls `
            -NotePropertyValue $HullBudget -Force
    } else {
        $node.extras.PSObject.Properties.Remove("collision_hulls")
    }
    $updatedNodes++
}
if ($updatedNodes -eq 0) {
    throw "Nenhum node fisico foi encontrado em: $resolvedPath"
}

$newJson = $document | ConvertTo-Json -Depth 100 -Compress
$newJsonBytes = [Text.Encoding]::UTF8.GetBytes($newJson)
$paddedJsonLength = [int](($newJsonBytes.Length + 3) -band -4)
$stream = [System.IO.MemoryStream]::new()
$writer = [System.IO.BinaryWriter]::new($stream)
try {
    $writer.Write([uint32]0x46546C67)
    $writer.Write([uint32]2)
    $writer.Write([uint32]0)
    $writer.Write([uint32]$paddedJsonLength)
    $writer.Write([uint32]0x4E4F534A)
    $writer.Write($newJsonBytes)
    for ($index = $newJsonBytes.Length;
        $index -lt $paddedJsonLength; $index++) {
        $writer.Write([byte]0x20)
    }
    $remainingOffset = 20 + $jsonLength
    if ($remainingOffset -lt $bytes.Length) {
        $writer.Write($bytes, $remainingOffset,
            $bytes.Length - $remainingOffset)
    }
    $writer.Flush()
    $output = $stream.ToArray()
    [BitConverter]::GetBytes([uint32]$output.Length).CopyTo($output, 8)
    $temporaryPath = $resolvedPath + ".collision.tmp"
    [System.IO.File]::WriteAllBytes($temporaryPath, $output)
    Move-Item -LiteralPath $temporaryPath -Destination $resolvedPath -Force
} finally {
    $writer.Dispose()
    $stream.Dispose()
}

Write-Output "$([IO.Path]::GetFileName($resolvedPath)): mode=$CollisionMode hulls=$HullBudget ($updatedNodes node(s))"
