param(
    [ValidateSet("Debug", "RelWithDebInfo", "Release")]
    [string]$Configuration = "Debug"
)

$root = Split-Path -Parent $PSScriptRoot
$executable = Join-Path $root "build-modern\$Configuration\MatterEngine.exe"
if (-not (Test-Path $executable)) {
    throw "Executavel nao encontrado. Rode tools/build.ps1 primeiro."
}

Push-Location $root
try {
    & $executable
} finally {
    Pop-Location
}
