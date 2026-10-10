#!/usr/bin/env bash
set -uo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$root" || exit 1

failed=0
check_absent() {
    local name="$1"
    shift
    local matches
    if matches="$(rg -n "$@" 2>&1)"; then
        printf '[FALHA] %s\n%s\n' "$name" "$matches"
        failed=1
    else
        local code=$?
        if (( code == 1 )); then
            printf '[OK] %s\n' "$name"
        else
            printf 'rg falhou durante "%s" (codigo %d):\n%s\n' \
                "$name" "$code" "$matches" >&2
            exit "$code"
        fi
    fi
}

check_absent "Workbench nao pode acessar SDL diretamente" \
    '#include <SDL|SDL_[A-Za-z]|SDL_Event' src/Workbench
check_absent "Tipos Vulkan devem ficar no backend Vulkan" \
    '\bVk[A-Z]|<volk|<vulkan|vk_mem' src -g '!src/Engine/RHI/Vulkan/**'
check_absent "O backend PhysX removido nao pode retornar" \
    '\bphysx::|#include.*(Px|characterkinematic|cooking/)' src
check_absent "Tipos Jolt devem ficar no backend Jolt" \
    '\bJPH::|#include.*<Jolt/' src \
    -g '!src/Engine/Physics/Jolt/**'
check_absent "O solver fisico removido nao pode retornar" \
    'PhysicsWorld3D|RigidBody3D|StaticCollisionWorld3D|TriangleMeshCollider3D|KinematicCharacter3D|PhysicsHandle3D|JointConstraint3D|CollisionCooking3D|MassProperties3D' \
    CMakeLists.txt src tests
check_absent "O runtime ativo nao usa OpenGL" \
    'SDL_opengl|\bgl(Begin|End|Vertex|Color|LineWidth)|opengl32' src CMakeLists.txt
check_absent "Engine nao pode depender do Workbench" \
    '#include.*Workbench/' src/Engine
check_absent "Codigo do jogo antigo nao pode voltar ao runtime" \
    '#include.*Game/|src[/\\]Game' CMakeLists.txt src tests
check_absent "Nucleo da modelagem nao depende da engine nem da interface" \
    '#include "Engine/(Animation|Audio|Character|Control|Data|Environment|Geometry|Input|Locomotion|Physics|Platform|Render|RHI|UI)/|#include .imgui|#include <SDL' \
    src/Engine/Modeling -g '!src/Engine/Modeling/Rig/**'
check_absent "Camadas de baixo da modelagem nao dependem da montagem, das pecas, do rig nem da inspecao" \
    '#include "Engine/Modeling/(Assembly|Rig|Parts|Inspection)/' \
    src/Engine/Modeling/Mesh src/Engine/Modeling/Skin src/Engine/Modeling/Import src/Engine/Modeling/Texture \
    src/Engine/Modeling/Paint src/Engine/Modeling/Geometry src/Engine/Modeling/Raster src/Engine/Modeling/Skeleton \
    src/Engine/Modeling/Analysis
check_absent "A montagem do jogador nao depende do rig do jogo nem da inspecao" \
    '#include "Engine/Modeling/(Rig|Inspection)/' src/Engine/Modeling/Assembly
check_absent "Tela de inspecao fala com a GPU so pelo CanvasTarget" \
    '#include "Engine/(Render|RHI)/' \
    src/Workbench/PlayerInspector -g '!src/Workbench/PlayerInspector/UiCanvasTexture.*'
check_absent "Tela de objetos fala com a GPU so pelo CanvasTarget" \
    '#include "Engine/(Render|RHI)/' src/Workbench/ObjectInspector
check_absent "Objetos procedurais (modelagem) nao dependem da montagem, das pecas, do rig nem da inspecao" \
    '#include "Engine/Modeling/(Assembly|Rig|Parts|Inspection)/' src/Engine/Modeling/Objects
check_absent "Atletas (Engine/Athlete) nao dependem de render, interface, audio, plataforma nem entrada" \
    '#include "Engine/(Render|RHI|UI|Audio|Platform|Input)/|#include .imgui|#include <SDL' src/Engine/Athlete
check_absent "Atletas: so o legado e a casca de migracao incluem os controladores antigos" \
    '#include "Engine/(Character|Control|Locomotion)/' src/Engine/Athlete \
    -g '!src/Engine/Athlete/Legacy/**' -g '!src/Engine/Athlete/AthleteRuntime3D.*' \
    -g '!src/Engine/Athlete/AthleteSettings3D.hpp' -g '!src/Engine/Athlete/AthleteAssets3D.*'
check_absent "Atletas: planejamento e sintese trabalham com dados (sem a cena fisica nem o Jolt)" \
    '#include "Engine/Physics/(PhysicsScene3D|PhysicsEngine3D)|Jolt' src/Engine/Athlete/Planning src/Engine/Athlete/Synthesis
check_absent "Identidade antiga e prototipos removidos nao podem retornar" \
    'SoccerFall|SOCCERFALL|LegacyFootworkRig|AnimatorScreen' \
    CMakeLists.txt src tests

(( failed == 0 )) || exit 1
echo "Fronteiras arquiteturais verificadas."
