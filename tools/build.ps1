param(
    [ValidateSet("Debug", "RelWithDebInfo", "Release")]
    [string]$Configuration = "Debug",
    [switch]$SkipTests
)

$ErrorActionPreference = "Stop"
$root = Split-Path -Parent $PSScriptRoot
$build = Join-Path $root "build-modern"

cmake -S $root -B $build -G "Visual Studio 17 2022" -A x64
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }

cmake --build $build --config $Configuration --target MatterEngineApp MatterEngineTests MatterAudioTests -j 8
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }

if (-not $SkipTests) {
    ctest --test-dir $build -C $Configuration --output-on-failure
    if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
}

Write-Host "Build pronta: $build\$Configuration\MatterEngine.exe"
