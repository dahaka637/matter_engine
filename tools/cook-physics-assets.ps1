param(
    [ValidateSet("Debug", "Release", "RelWithDebInfo")]
    [string]$Configuration = "Release"
)

$ErrorActionPreference = "Stop"
$root = Split-Path -Parent $PSScriptRoot

Push-Location $root
try {
    cmake --build build-modern --config $Configuration `
        --target MatterPhysicsAssetCooker --parallel
    if ($LASTEXITCODE -ne 0) {
        throw "Falha ao compilar MatterPhysicsAssetCooker."
    }
    & ".\build-modern\$Configuration\MatterPhysicsAssetCooker.exe" `
        ".\assets"
    if ($LASTEXITCODE -ne 0) {
        throw "Falha ao gerar os caches fisicos."
    }
} finally {
    Pop-Location
}
