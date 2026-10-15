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
        Name = "O backend PhysX removido nao pode retornar"
        Arguments = @("-n", "\bphysx::|#include.*(Px|characterkinematic|cooking/)", "src")
    },
    @{
        Name = "Tipos Jolt devem ficar no backend Jolt"
        Arguments = @("-n", "\bJPH::|#include.*<Jolt/", "src", "-g", "!src/Engine/Physics/Jolt/**")
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
        Name = "Nucleo da modelagem nao depende da engine nem da interface"
        Arguments = @("-n", '#include "Engine/(Animation|Audio|Character|Control|Data|Environment|Geometry|Input|Locomotion|Physics|Platform|Render|RHI|UI)/|#include .imgui|#include <SDL', "src/Engine/Modeling", "-g", "!src/Engine/Modeling/Rig/**")
    },
    @{
        Name = "Tela de inspecao fala com a GPU so pelo CanvasTarget"
        Arguments = @("-n", '#include "Engine/(Render|RHI)/', "src/Workbench/PlayerInspector", "-g", "!src/Workbench/PlayerInspector/UiCanvasTexture.*")
    },
    @{
        Name = "Atletas (Engine/Athlete) nao dependem de render, interface, audio, plataforma nem entrada"
        Arguments = @("-n", '#include "Engine/(Render|RHI|UI|Audio|Platform|Input)/|#include .imgui|#include <SDL', "src/Engine/Athlete")
    }
    @{
        Name = "Atletas: so o legado e a casca de migracao incluem os controladores antigos"
        Arguments = @("-n", '#include "Engine/(Character|Control|Locomotion)/', "src/Engine/Athlete",
            "-g", "!src/Engine/Athlete/Legacy/**", "-g", "!src/Engine/Athlete/AthleteRuntime3D.*",
            "-g", "!src/Engine/Athlete/AthleteSettings3D.hpp", "-g", "!src/Engine/Athlete/AthleteAssets3D.*")
    }
    @{
        Name = "Atletas: planejamento, sintese, acoes, pedidos e bola trabalham com dados (sem a cena fisica nem o Jolt; so os sentidos leem a fisica)"
        Arguments = @("-n", '#include "Engine/Physics/(PhysicsScene3D|PhysicsEngine3D)|Jolt', "src/Engine/Athlete/Planning",
            "src/Engine/Athlete/Synthesis", "src/Engine/Athlete/Actions", "src/Engine/Athlete/Intent",
            "src/Engine/Athlete/Ball", "src/Engine/Athlete/Core")
    }
    @{
        Name = "Nucleo de RL nao depende de render, interface, audio, plataforma nem entrada"
        Arguments = @("-n", '#include "Engine/(Render|RHI|UI|Audio|Platform|Input)/|#include .imgui|#include <SDL', "src/Engine/RL")
    }
    @{
        Name = "So a ponte do treinador (bindings/python) inclui Python"
        Arguments = @("-n", '#include .(Python\.h|pybind11|nanobind)', "src", "tests", "tools", "-g", "!tools/rl/**")
    }
    @{
        Name = "Mundos de treino e politica do RL nao dependem da interface nem do treinador"
        Arguments = @("-n", '#include "(Workbench|Engine/RL/Supervisor)/', "src/Engine/RL/Environment", "src/Engine/RL/Body", "src/Engine/RL/Policy")
    }
    @{
        Name = "A politica exportada (RL/Policy) nao depende da fisica"
        Arguments = @("-n", '#include "Engine/(Physics|RL/Environment|RL/Body)/', "src/Engine/RL/Policy")
    }
    @{
        Name = "Contrato do Corpo e corpo de RL (vao para o jogo) nao dependem dos mundos de treino, do estilo, das notas, do piloto, da imitacao nem do treinador"
        Arguments = @("-n", '#include "Engine/RL/(Environment|Style|Rewards|Commands|Imitation|Supervisor|Calibration)/', "src/Engine/RL/Contract", "src/Engine/RL/Body")
    }
    @{
        Name = "O piloto e o adaptador do controle (RL/Commands) sao dados puros: sem fisica, Jolt nem as pecas do treino"
        Arguments = @("-n", '#include "Engine/(Physics|RL/(Environment|Style|Rewards|Imitation|Supervisor|Calibration|Body))/|#include <Jolt/', "src/Engine/RL/Commands")
    }
    @{
        Name = "As notas do Corpo (RL/Rewards) nao dependem dos mundos de treino, do estilo nem da imitacao"
        Arguments = @("-n", '#include "Engine/RL/(Environment|Style|Imitation|Supervisor)/', "src/Engine/RL/Rewards")
    }
    @{
        Name = "O cerebro do Corpo no jogo (RL/Brain) e so o contrato e a politica: nada do treino"
        Arguments = @("-n", '#include "Engine/RL/(Environment|Style|Rewards|Commands|Imitation|Supervisor|Calibration)/', "src/Engine/RL/Brain")
    }
    @{
        Name = "O estilo (o juiz e o acervo, RL/Style) so entra nos mundos de treino"
        Arguments = @("-n", '#include "Engine/RL/Style/', "src/Engine/RL/Policy", "src/Engine/RL/Commands", "src/Engine/RL/Rewards", "src/Engine/RL/Imitation", "src/Engine/RL/Supervisor", "src/Engine/RL/Calibration", "src/Engine/Athlete", "src/Workbench")
    }
    @{
        Name = "Tela de RL fala com o treinador so pelo Supervisor (sem sockets nem processos)"
        Arguments = @("-n", '#include <(sys/|spawn\.h|unistd\.h)', "src/Workbench")
    }
    @{
        Name = "Tela de RL fala com a GPU so pelo CanvasTarget"
        Arguments = @("-n", 'RHI::|Renderer&|#include "Engine/(RHI|Render)/', "src/Workbench/ReinforcementLearning")
    }
    @{
        Name = "Identidade antiga e prototipos removidos nao podem retornar"
        Arguments = @("-n", "SoccerFall|SOCCERFALL|LegacyFootworkRig|PhysicalBiped|AnimatorScreen", "CMakeLists.txt", "src", "tests")
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
