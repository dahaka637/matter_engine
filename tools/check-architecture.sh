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
check_absent "O treino da locomocao (Athlete/Training) usa o atleta so pelo runtime: nem o legado nem os controladores antigos" \
    '#include "Engine/(Character|Control|Locomotion)/|#include "Engine/Athlete/Legacy/' src/Engine/Athlete/Training
check_absent "O treino da locomocao nao entra no jogo: o motor e o laboratorio nao o incluem" \
    '#include "Engine/Athlete/Training/' src/Engine src/Workbench -g '!src/Engine/Athlete/Training/**'
check_absent "O treino da locomocao nao fala com o treinador, o estilo nem a interface" \
    '#include "(Workbench|Engine/RL/(Supervisor|Style))/' src/Engine/Athlete/Training
check_absent "O treino da locomocao usa do RL so o piloto e o layout (nunca o lote, o corpo, a imitacao nem a politica do RL do zero: o ReferenceMotion3D deles e outro)" \
    '#include "Engine/RL/(Environment|Body|Imitation|Policy|Rewards|Brain|Calibration)/' src/Engine/Athlete/Training \
    bindings/python/MatterLocomotionNative.cpp
check_absent "Atletas: planejamento, sintese, acoes, pedidos e bola trabalham com dados (sem a cena fisica nem o Jolt; so os sentidos leem a fisica)" \
    '#include "Engine/Physics/(PhysicsScene3D|PhysicsEngine3D)|Jolt' src/Engine/Athlete/Planning src/Engine/Athlete/Synthesis \
    src/Engine/Athlete/Actions src/Engine/Athlete/Intent src/Engine/Athlete/Ball src/Engine/Athlete/Core \
    src/Engine/Athlete/Dribble
check_absent "Nucleo de RL nao depende de render, interface, audio, plataforma nem entrada" \
    '#include "Engine/(Render|RHI|UI|Audio|Platform|Input)/|#include .imgui|#include <SDL' src/Engine/RL
check_absent "So a ponte do treinador (bindings/python) inclui Python" \
    '#include .(Python\.h|pybind11|nanobind)' src tests tools -g '!tools/rl/**'
check_absent "Mundos de treino e politica do RL nao dependem da interface nem do treinador" \
    '#include "(Workbench|Engine/RL/Supervisor)/' src/Engine/RL/Environment src/Engine/RL/Body src/Engine/RL/Policy src/Engine/RL/Imitation
check_absent "Mundos de treino e politica do RL nao dependem do atleta nem da locomocao do jogo" \
    '#include "Engine/(Athlete|Character|Control|Locomotion)/' src/Engine/RL/Environment src/Engine/RL/Body src/Engine/RL/Policy src/Engine/RL/Imitation
check_absent "A politica exportada (RL/Policy) nao depende da fisica" \
    '#include "Engine/(Physics|RL/Environment|RL/Body)/' src/Engine/RL/Policy
check_absent "Contrato do Corpo e corpo de RL (vao para o jogo) nao dependem dos mundos de treino, do estilo, das notas, do piloto, da imitacao nem do treinador" \
    '#include "Engine/RL/(Environment|Style|Rewards|Commands|Imitation|Supervisor|Calibration)/' \
    src/Engine/RL/Contract src/Engine/RL/Body
check_absent "O piloto e o adaptador do controle (RL/Commands) sao dados puros: sem fisica, Jolt nem as pecas do treino" \
    '#include "Engine/(Physics|RL/(Environment|Style|Rewards|Imitation|Supervisor|Calibration|Body))/|#include <Jolt/' \
    src/Engine/RL/Commands
check_absent "As notas do Corpo (RL/Rewards) nao dependem dos mundos de treino, do estilo nem da imitacao" \
    '#include "Engine/RL/(Environment|Style|Imitation|Supervisor)/' src/Engine/RL/Rewards
check_absent "O cerebro do Corpo no jogo (RL/Brain) e so o contrato e a politica: nada do treino" \
    '#include "Engine/RL/(Environment|Style|Rewards|Commands|Imitation|Supervisor|Calibration)/' \
    src/Engine/RL/Brain
check_absent "O estilo (o juiz e o acervo, RL/Style) so entra nos mundos de treino" \
    '#include "Engine/RL/Style/' src/Engine/RL/Policy src/Engine/RL/Commands src/Engine/RL/Rewards \
    src/Engine/RL/Imitation src/Engine/RL/Supervisor src/Engine/RL/Calibration \
    src/Engine/Athlete src/Workbench
check_absent "O contrato 1 do RL (o lote antigo, as tarefas H e o professor) foi removido e nao volta" \
    '\b(TrainingBatch3D|TrainingTask3D|TrainingTeacher3D|LocomotionTeacher3D)\b|Engine/RL/Teacher/' src tests tools bindings \
    -g '!tools/check-architecture.*'
check_absent "Tela de RL fala com o treinador so pelo Supervisor (sem sockets nem processos)" \
    '#include <(sys/|spawn\.h|unistd\.h)' src/Workbench
check_absent "Tela de RL fala com a GPU so pelo CanvasTarget" \
    'RHI::|Renderer&|#include "Engine/(RHI|Render)/' src/Workbench/ReinforcementLearning
check_absent "O rastreador (Engine/Tracking) nao depende de render, interface, audio, plataforma nem entrada" \
    '#include "Engine/(Render|RHI|UI|Audio|Platform|Input)/|#include .imgui|#include <SDL' src/Engine/Tracking
check_absent "O rastreador segue o padrao e o corpo do jogo: nem o legado, nem o treino da locomocao antiga, nem o RL do zero" \
    '#include "Engine/Athlete/(Legacy|Training|Contract|Policy)/|#include "Engine/RL/(Environment|Style|Rewards|Imitation|Supervisor|Calibration|Brain|Commands)/' \
    src/Engine/Tracking bindings/python/MatterTrackingNative.cpp
check_absent "Identidade antiga e prototipos removidos nao podem retornar" \
    'SoccerFall|SOCCERFALL|LegacyFootworkRig|AnimatorScreen' \
    CMakeLists.txt src tests

(( failed == 0 )) || exit 1
echo "Fronteiras arquiteturais verificadas."
