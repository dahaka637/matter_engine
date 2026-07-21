param()

$ErrorActionPreference = "Stop"
$root = Split-Path -Parent $PSScriptRoot

if (-not (Get-Command rg -ErrorAction SilentlyContinue)) {
    throw "rg (ripgrep) nao foi encontrado no PATH."
}

$checks = @(
    @{
        Name = "Workbench nao pode acessar SDL diretamente"
        Arguments = @("-n", "#include <SDL|SDL_[A-Za-z]|SDL_Event", "src/Workbench")
    },
    @{
        Name = "Tipos Vulkan devem ficar no backend Vulkan"
        Arguments = @("-n", "\bVk[A-Z]|<volk|<vulkan|vk_mem", "src", "-g", "!src/Engine/RHI/Vulkan/**")
    },
    @{
        Name = "Tipos PhysX devem ficar no backend PhysX"
        Arguments = @("-n", "\bphysx::|#include.*(Px|characterkinematic|cooking/)", "src", "-g", "!src/Engine/Physics/PhysX/**")
    },
    @{
        Name = "O solver fisico removido nao pode retornar"
        Arguments = @("-n", "PhysicsWorld3D|RigidBody3D|StaticCollisionWorld3D|TriangleMeshCollider3D|KinematicCharacter3D|PhysicsHandle3D|JointConstraint3D|CollisionCooking3D|MassProperties3D", "CMakeLists.txt", "src", "tests")
    },
    @{
        Name = "O runtime ativo nao usa OpenGL"
        Arguments = @("-n", "SDL_opengl|\bgl(Begin|End|Vertex|Color|LineWidth)|opengl32", "src", "CMakeLists.txt")
    },
    @{
        Name = "Engine nao pode depender do Workbench"
        Arguments = @("-n", "#include.*Workbench/", "src/Engine")
    },
    @{
        Name = "Codigo do jogo antigo nao pode voltar ao runtime"
        Arguments = @("-n", "#include.*Game/|src[/\\]Game", "CMakeLists.txt", "src", "tests")
    },
    @{
        Name = "Identidade antiga e prototipos removidos nao podem retornar"
        Arguments = @("-n", "SoccerFall|SOCCERFALL|Footwork|PhysicalBiped|AnimatorScreen", "CMakeLists.txt", "src", "tests")
    }
)

$failed = $false
Push-Location $root
try {
    foreach ($check in $checks) {
        $matches = & rg @($check.Arguments) 2>&1
        $code = $LASTEXITCODE
        if ($code -eq 0) {
            Write-Host "[FALHA] $($check.Name)" -ForegroundColor Red
            $matches | ForEach-Object { Write-Host "  $_" }
            $failed = $true
        } elseif ($code -eq 1) {
            Write-Host "[OK] $($check.Name)" -ForegroundColor Green
        } else {
            throw "rg falhou durante a verificacao '$($check.Name)' (codigo $code)."
        }
    }
} finally {
    Pop-Location
}

if ($failed) {
    exit 1
}

Write-Host "Fronteiras arquiteturais verificadas." -ForegroundColor Green
