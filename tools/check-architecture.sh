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
check_absent "Identidade antiga e prototipos removidos nao podem retornar" \
    'SoccerFall|SOCCERFALL|LegacyFootworkRig|AnimatorScreen' \
    CMakeLists.txt src tests

(( failed == 0 )) || exit 1
echo "Fronteiras arquiteturais verificadas."
