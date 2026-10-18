cmake_minimum_required(VERSION 3.20)

project(MatterEngine VERSION 0.1.0 LANGUAGES CXX C)

if(WIN32)
    enable_language(RC)
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set_property(GLOBAL PROPERTY USE_FOLDERS ON)

# The project uses no C++20 modules, so module dependency scanning only adds
# Ninja dyndep overhead and a known GCC/CMake interaction bug: the scanner's
# include-translation query chokes on vendored headers that quote-include a
# "./relative" path (e.g. openal-soft's bundled GSL), aborting the build with
# "malformed string 'ERROR ... INCLUDE-TRANSLATE ...'" even though no module
# is involved.
set(CMAKE_CXX_SCAN_FOR_MODULES OFF CACHE BOOL "" FORCE)

include(FetchContent)

# A ponte Python do treinador de RL (bindings/python) e um modulo
# compartilhado: tudo o que entra nele, inclusive o Jolt, precisa ser PIC.
# Fica desligada na build normal; tools/rl/setup.sh a liga numa build propria.
option(MATTERENGINE_BUILD_RL_PYTHON "Build the Python bridge of the RL trainer" OFF)
if(MATTERENGINE_BUILD_RL_PYTHON)
    set(CMAKE_POSITION_INDEPENDENT_CODE ON)
endif()


# The Vulkan toolchain is fetched explicitly so contributors only need a C++
# compiler and a Vulkan-capable driver. A system-wide Vulkan SDK remains useful
# for validation layers and debugging tools, but is not required to build.
set(VULKAN_HEADERS_ENABLE_TESTS OFF CACHE BOOL "" FORCE)
FetchContent_Declare(
    VulkanHeaders
    GIT_REPOSITORY https://github.com/KhronosGroup/Vulkan-Headers.git
    GIT_TAG v1.4.357
    GIT_SHALLOW TRUE
)
FetchContent_MakeAvailable(VulkanHeaders)

FetchContent_Declare(
    volk
    GIT_REPOSITORY https://github.com/zeux/volk.git
    GIT_TAG vulkan-sdk-1.4.350.1
    GIT_SHALLOW TRUE
)
FetchContent_MakeAvailable(volk)

FetchContent_Declare(
    VulkanMemoryAllocator
    GIT_REPOSITORY https://github.com/GPUOpen-LibrariesAndSDKs/VulkanMemoryAllocator.git
    GIT_TAG v3.4.0
    GIT_SHALLOW TRUE
)
FetchContent_MakeAvailable(VulkanMemoryAllocator)

set(BUILD_TESTING OFF CACHE BOOL "" FORCE)
set(ENABLE_CTEST OFF CACHE BOOL "" FORCE)
set(ENABLE_GLSLANG_BINARIES ON CACHE BOOL "" FORCE)
set(ENABLE_SPVREMAPPER OFF CACHE BOOL "" FORCE)
set(ENABLE_HLSL OFF CACHE BOOL "" FORCE)
set(ENABLE_OPT OFF CACHE BOOL "" FORCE)
FetchContent_Declare(
    glslang
    GIT_REPOSITORY https://github.com/KhronosGroup/glslang.git
    GIT_TAG 16.4.0
    GIT_SHALLOW TRUE
)
FetchContent_MakeAvailable(glslang)

set(SDL_TEST_LIBRARY OFF CACHE BOOL "" FORCE)
set(SDL_TESTS OFF CACHE BOOL "" FORCE)
set(SDL_EXAMPLES OFF CACHE BOOL "" FORCE)
set(SDL_SHARED OFF CACHE BOOL "" FORCE)
set(SDL_STATIC ON CACHE BOOL "" FORCE)
set(SDL_OPENGL OFF CACHE BOOL "" FORCE)
set(SDL_OPENGLES OFF CACHE BOOL "" FORCE)
set(SDL_VULKAN ON CACHE BOOL "" FORCE)

FetchContent_Declare(
    SDL3
    GIT_REPOSITORY https://github.com/libsdl-org/SDL.git
    GIT_TAG release-3.4.12
    GIT_SHALLOW TRUE
)

FetchContent_Declare(
    imgui
    GIT_REPOSITORY https://github.com/ocornut/imgui.git
    GIT_TAG v1.92.8
    GIT_SHALLOW TRUE
)

FetchContent_Declare(
    sqlite_amalgamation
    URL https://www.sqlite.org/2026/sqlite-amalgamation-3530300.zip
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
)

# Asset import: cgltf parses glTF 2.0 (.gltf/.glb), stb_image decodes the
# PNG/JPEG textures embedded or referenced inside it. Both are vendored as
# single-header C libraries with no build system of their own, so each gets
# wrapped in a tiny static library exposing just its include directory - the
# same pattern already used for sqlite3's amalgamation below.
FetchContent_Declare(
    cgltf
    GIT_REPOSITORY https://github.com/jkuhlmann/cgltf.git
    GIT_TAG v1.14
    GIT_SHALLOW TRUE
)

FetchContent_Declare(
    stb
    GIT_REPOSITORY https://github.com/nothings/stb.git
    GIT_TAG 31c1ad37456438565541f4919958214b6e762fb4
)

# LODs e otimização de cache usam uma biblioteca especializada, empregada
# offline/no carregamento; o frame nunca executa simplificação geométrica.
set(MESHOPT_BUILD_DEMO OFF CACHE BOOL "" FORCE)
set(MESHOPT_BUILD_GLTFPACK OFF CACHE BOOL "" FORCE)
FetchContent_Declare(
    meshoptimizer
    GIT_REPOSITORY https://github.com/zeux/meshoptimizer.git
    GIT_TAG v1.2
    GIT_SHALLOW TRUE
)

# Perfis físicos orientados a dados usam JSON completo e validado. A
# dependência é header-only, fica privada ao módulo de física e está fixada
# numa release para manter builds reproduzíveis.
set(JSON_BuildTests OFF CACHE BOOL "" FORCE)
set(JSON_Install OFF CACHE BOOL "" FORCE)
FetchContent_Declare(
    nlohmann_json
    GIT_REPOSITORY https://github.com/nlohmann/json.git
    GIT_TAG v3.11.3
    GIT_SHALLOW TRUE
)

# O SDK de fisica (Jolt) e buscado e configurado em cmake/PhysicsBackend.cmake,
# incluido mais abaixo: depende do target `vhacd`.

# V-HACD transforma uma malha visual concava em um pequeno conjunto de hulls
# convexos. Esse processamento acontece no carregamento/cooking do asset, nao
# no loop de 120 Hz; a simulacao recebe somente formas apropriadas a corpos
# dinamicos. A revisao tambem e imutavel para preservar o cache de colliders.
FetchContent_Declare(
    vhacd_source
    GIT_REPOSITORY https://github.com/kmammou/v-hacd.git
    GIT_TAG 22ec20a7f8ea221ab600df869369b9c6a258cb10
)

# OpenAL Soft e o backend de mixagem 3D (HRTF binaural + EFX). O projeto nao
# corta releases com frequencia (a ultima tag oficial e de 2021); fixamos um
# commit especifico do branch principal, o mesmo padrao ja usado acima para o
# V-HACD, para manter reprodutibilidade sem abrir mao de anos de correcoes.
set(LIBTYPE "STATIC" CACHE STRING "" FORCE)
set(ALSOFT_UTILS OFF CACHE BOOL "" FORCE)
set(ALSOFT_EXAMPLES OFF CACHE BOOL "" FORCE)
set(ALSOFT_TESTS OFF CACHE BOOL "" FORCE)
set(ALSOFT_INSTALL OFF CACHE BOOL "" FORCE)
set(ALSOFT_INSTALL_CONFIG OFF CACHE BOOL "" FORCE)
set(ALSOFT_INSTALL_HRTF_DATA OFF CACHE BOOL "" FORCE)
set(ALSOFT_INSTALL_AMBDEC_PRESETS OFF CACHE BOOL "" FORCE)
set(ALSOFT_INSTALL_EXAMPLES OFF CACHE BOOL "" FORCE)
set(ALSOFT_INSTALL_UTILS OFF CACHE BOOL "" FORCE)
set(ALSOFT_EMBED_HRTF_DATA ON CACHE BOOL "" FORCE)
# Sem isso o CRT do OpenAL Soft ficaria em /MT enquanto o resto do projeto usa
# /MD - divergencia de runtime que quebra o link no MSVC.
set(FORCE_STATIC_VCRT OFF CACHE BOOL "" FORCE)
# Modulos C++20 sao um caminho de build experimental e recente na lib; nada
# aqui precisa deles (consumimos so os headers C AL/ALC/EFX), entao
# desligamos para nao herdar um eixo de risco de build que nao usamos.
set(ALSOFT_ENABLE_MODULES OFF CACHE BOOL "" FORCE)
# No Windows, usamos apenas o backend nativo moderno WASAPI. Em outras
# plataformas, deixamos o OpenAL Soft selecionar os backends nativos que ele
# detectar (PipeWire/PulseAudio/ALSA no Linux, por exemplo).
if(WIN32)
    set(ALSOFT_BACKEND_WASAPI ON CACHE BOOL "" FORCE)
    set(ALSOFT_REQUIRE_WASAPI ON CACHE BOOL "" FORCE)
    set(ALSOFT_BACKEND_WINMM OFF CACHE BOOL "" FORCE)
    set(ALSOFT_BACKEND_DSOUND OFF CACHE BOOL "" FORCE)
else()
    set(ALSOFT_REQUIRE_WASAPI OFF CACHE BOOL "" FORCE)
endif()
FetchContent_Declare(
    openal-soft
    GIT_REPOSITORY https://github.com/kcat/openal-soft.git
    GIT_TAG 7f9c6de1162b1d4604b9a2f8bcd6c53021566221
)

FetchContent_MakeAvailable(
    SDL3 imgui sqlite_amalgamation cgltf stb meshoptimizer nlohmann_json
    vhacd_source openal-soft)

add_library(vhacd INTERFACE)
target_include_directories(vhacd INTERFACE ${vhacd_source_SOURCE_DIR}/include)

# Backend de fisica: busca o SDK escolhido, aplica as supressoes de aviso que
# ele exigir e publica as fontes/bibliotecas para o target MatterPhysics.
include(${CMAKE_CURRENT_LIST_DIR}/cmake/PhysicsBackend.cmake)

add_library(sqlite3 STATIC ${sqlite_amalgamation_SOURCE_DIR}/sqlite3.c)
target_include_directories(sqlite3 PUBLIC ${sqlite_amalgamation_SOURCE_DIR})
target_compile_definitions(sqlite3 PUBLIC SQLITE_THREADSAFE=1 SQLITE_OMIT_LOAD_EXTENSION)
if(MSVC)
    target_compile_options(sqlite3 PRIVATE /W1)
endif()

add_library(cgltf STATIC src/Engine/Geometry/CgltfImpl.cpp)
target_include_directories(cgltf PUBLIC ${cgltf_SOURCE_DIR})
if(MSVC)
    target_compile_options(cgltf PRIVATE /W1)
endif()

add_library(stb_image STATIC src/Engine/Geometry/StbImageImpl.cpp)
target_include_directories(stb_image PUBLIC ${stb_SOURCE_DIR})
if(MSVC)
    target_compile_options(stb_image PRIVATE /W1)
endif()

add_library(imgui STATIC
    ${imgui_SOURCE_DIR}/imgui.cpp
    ${imgui_SOURCE_DIR}/imgui_draw.cpp
    ${imgui_SOURCE_DIR}/imgui_tables.cpp
    ${imgui_SOURCE_DIR}/imgui_widgets.cpp
    ${imgui_SOURCE_DIR}/backends/imgui_impl_sdl3.cpp
    ${imgui_SOURCE_DIR}/backends/imgui_impl_vulkan.cpp
)

target_include_directories(imgui PUBLIC
    ${imgui_SOURCE_DIR}
    ${imgui_SOURCE_DIR}/backends
)

target_compile_definitions(imgui PUBLIC IMGUI_IMPL_VULKAN_USE_VOLK)
target_link_libraries(imgui PUBLIC SDL3::SDL3-static volk::volk)

set(MATTERENGINE_SHADER_OUTPUT_DIR "${CMAKE_CURRENT_BINARY_DIR}/shaders")
file(MAKE_DIRECTORY "${MATTERENGINE_SHADER_OUTPUT_DIR}")
set(MATTERENGINE_SHADER_OUTPUTS)
foreach(shader render2d.vert render2d.frag sprite.vert sprite.frag
        scene3d_sky.vert scene3d_sky.frag
        scene3d_mesh.vert scene3d_mesh.frag scene3d_mesh_shadow.vert
        scene3d_mesh_depth.vert
        tonemap.vert tonemap.frag auto_exposure.frag
        gtao.frag bloom_glare.frag
        taa_resolve.frag
        ssr_trace_resolve.frag ssr_composite.frag
        ocean_surface.vert ocean_surface.frag)
    set(shader_source "${CMAKE_SOURCE_DIR}/assets/shaders/${shader}")
    set(shader_output "${MATTERENGINE_SHADER_OUTPUT_DIR}/${shader}.spv")
    add_custom_command(
        OUTPUT "${shader_output}"
        COMMAND $<TARGET_FILE:glslang-standalone> -V --target-env vulkan1.4 -o "${shader_output}" "${shader_source}"
        DEPENDS glslang-standalone "${shader_source}"
        COMMENT "Compiling Vulkan shader ${shader}"
        VERBATIM
    )
    list(APPEND MATTERENGINE_SHADER_OUTPUTS "${shader_output}")
endforeach()
add_custom_target(MatterEngineShaders DEPENDS ${MATTERENGINE_SHADER_OUTPUTS})

# MaterialLibrary e a matematica basica (Vec2, Vec3, Quaternion, Mat4) vivem
# aqui (a base da hierarquia) porque MatterPhysics, MatterAudio e o backend
# MatterVulkan dependem deles e nenhum pode depender da MatterEngine.
# Frustum3D continua em MatterEngine: nada abaixo dela usa culling.
add_library(MatterCore STATIC
    src/Engine/Core/Log.cpp
    src/Engine/Core/TaskScheduler.cpp
    src/Engine/Materials/MaterialLibrary.cpp
    src/Engine/Math/Hash.cpp
    src/Engine/Math/Vec2.cpp
    src/Engine/Math/Vec3.cpp
    src/Engine/Math/Quaternion.cpp
    src/Engine/Math/Mat4.cpp
    # A casca convexa (quickhull) so depende da matematica: a chuteira do
    # exportador e a divisao do pe do corpo de RL (sem a engine grafica).
    src/Engine/Geometry/ConvexHull3D.cpp
)
target_include_directories(MatterCore PUBLIC src)

add_library(MatterRHI INTERFACE)
target_include_directories(MatterRHI INTERFACE src)

# API publica neutra + backend Jolt privado. Nenhum consumidor recebe os
# includes ou tipos nativos do SDK; trocar/configurar o backend nao contamina a
# camada Workbench nem o restante da engine.
add_library(MatterPhysics STATIC
    src/Engine/Physics/PhysicalBodyBuilder3D.cpp
    src/Engine/Physics/InflatedBall3D.cpp
    src/Engine/Physics/RagdollProfile3D.cpp
    src/Engine/Physics/WindShelter3D.cpp
    src/Engine/Physics/Articulation/ArticulationIndexing3D.cpp
    src/Engine/Physics/Articulation/ArticulationGravity3D.cpp
    ${MATTERENGINE_PHYSICS_BACKEND_SOURCES}
)
target_include_directories(MatterPhysics PUBLIC src)
target_link_libraries(MatterPhysics
    PUBLIC MatterCore
    PRIVATE ${MATTERENGINE_PHYSICS_BACKEND_LIBRARIES} vhacd
            nlohmann_json::nlohmann_json
)
target_compile_definitions(MatterPhysics PRIVATE
    ${MATTERENGINE_PHYSICS_BACKEND_DEFINITIONS})
if(MSVC)
    # Dependencias de terceiros sao compiladas com seus proprios perfis. Estas
    # opcoes mantem diagnosticos fortes no adaptador escrito pela MatterEngine.
    target_compile_options(MatterPhysics PRIVATE /W4 /permissive- /utf-8)
else()
    target_compile_options(MatterPhysics PRIVATE -Wall -Wextra -Wpedantic)
endif()

# Os clipes de animacao no formato do ragdoll (matter-ragdoll-animation-1):
# carregar, amostrar e projetar na articulacao. Saiu do MatterEngine para que
# o treino de RL (sem janela) use os mesmos clipes que o jogo.
add_library(MatterAnimationClips STATIC
    src/Engine/Animation/AnimationClip3D.cpp
)
target_include_directories(MatterAnimationClips PUBLIC src)
target_link_libraries(MatterAnimationClips
    PUBLIC MatterPhysics
    PRIVATE nlohmann_json::nlohmann_json)

# Fundacao isolada do experimento de Reinforcement Learning. Esta opcao
# compila apenas contratos/ferramentas C++ headless: nao adiciona Python,
# PyTorch, renderer ou Workbench ao grafo de dependencias.
option(MATTERENGINE_BUILD_RL "Build isolated MatterEngine RL foundations" ON)
if(MATTERENGINE_BUILD_RL)
    add_library(MatterRLCalibration STATIC
        src/Engine/RL/Calibration/RagdollCalibrationAudit3D.cpp
        src/Engine/RL/Calibration/RagdollJointLimitLab3D.cpp
        src/Engine/RL/Calibration/RagdollColliderWireframe3D.cpp
        src/Engine/RL/Calibration/RagdollDynamicLab3D.cpp
    )
    target_include_directories(MatterRLCalibration PUBLIC src)
    target_link_libraries(MatterRLCalibration
        PUBLIC MatterPhysics
        PRIVATE nlohmann_json::nlohmann_json)
    if(MSVC)
        target_compile_options(MatterRLCalibration PRIVATE
            /W4 /permissive- /utf-8)
    else()
        target_compile_options(MatterRLCalibration PRIVATE
            -Wall -Wextra -Wpedantic)
    endif()

    # A ligacao da tela com o treinador (processo, socket local) e o
    # historico do banco do treino, so leitura. POSIX; sem interface.
    add_library(MatterRLSupervisor STATIC
        src/Engine/RL/Supervisor/TrainerLink3D.cpp
        src/Engine/RL/Supervisor/TrainingHistory3D.cpp
    )
    target_include_directories(MatterRLSupervisor PUBLIC src)
    target_link_libraries(MatterRLSupervisor PRIVATE nlohmann_json::nlohmann_json sqlite3)
    if(NOT MSVC)
        target_compile_options(MatterRLSupervisor PRIVATE -Wall -Wextra -Wpedantic)
    endif()

    # O corpo de RL e o contrato do Corpo v2 (o que a rede ve e faz): vai para
    # o jogo junto com a politica, entao nao depende dos mundos de treino.
    add_library(MatterRLContract STATIC
        src/Engine/RL/Body/RLBody3D.cpp
        src/Engine/RL/Body/RLFootSplit3D.cpp
        src/Engine/RL/Contract/BodyContract3D.cpp
        src/Engine/RL/Contract/Mirror3D.cpp
    )
    target_include_directories(MatterRLContract PUBLIC src)
    target_link_libraries(MatterRLContract
        PUBLIC MatterPhysics MatterAnimationClips MatterCore
        PRIVATE nlohmann_json::nlohmann_json)
    if(MSVC)
        target_compile_options(MatterRLContract PRIVATE /W4 /permissive- /utf-8)
    else()
        target_compile_options(MatterRLContract PRIVATE -Wall -Wextra -Wpedantic)
    endif()

    # O cerebro do Corpo no jogo: o contrato v2 e a politica exportada (sem
    # nada do treino).
    add_library(MatterRLBrain STATIC
        src/Engine/RL/Brain/BodyBrain3D.cpp
    )
    target_include_directories(MatterRLBrain PUBLIC src)
    target_link_libraries(MatterRLBrain PUBLIC MatterRLContract MatterRLPolicy)
    if(MSVC)
        target_compile_options(MatterRLBrain PRIVATE /W4 /permissive- /utf-8)
    else()
        target_compile_options(MatterRLBrain PRIVATE -Wall -Wextra -Wpedantic)
    endif()

    # O adaptador do controle e o piloto (dados puros, sem fisica): o jogo e a
    # IA reaproveitam; o treino aperta o controle por ele.
    add_library(MatterRLCommands STATIC
        src/Engine/RL/Commands/ControlAdapter3D.cpp
        src/Engine/RL/Commands/CommandPilot3D.cpp
        src/Engine/RL/Commands/PilotScriptJson3D.cpp
    )
    target_include_directories(MatterRLCommands PUBLIC src)
    target_link_libraries(MatterRLCommands PUBLIC MatterCore nlohmann_json::nlohmann_json)
    if(MSVC)
        target_compile_options(MatterRLCommands PRIVATE /W4 /permissive- /utf-8)
    else()
        target_compile_options(MatterRLCommands PRIVATE -Wall -Wextra -Wpedantic)
    endif()

    # O treino da locomocao (a etapa B do plano do RL): a tarefa, a nota e o
    # lote de RL sobre o atleta do jogo. Sem janela nem render: a ponte Python
    # o usa.
    add_library(MatterAthleteTraining STATIC
        src/Engine/Athlete/Training/LocomotionTask3D.cpp
        src/Engine/Athlete/Training/LocomotionReward3D.cpp
        src/Engine/Athlete/Training/LocomotionTraining3D.cpp
    )
    target_include_directories(MatterAthleteTraining PUBLIC src)
    target_link_libraries(MatterAthleteTraining
        PUBLIC MatterAthlete MatterRLCommands
        PRIVATE nlohmann_json::nlohmann_json)
    if(MSVC)
        target_compile_options(MatterAthleteTraining PRIVATE /W4 /permissive- /utf-8)
    else()
        target_compile_options(MatterAthleteTraining PRIVATE -Wall -Wextra -Wpedantic)
    endif()

    # O rastreador fisico (DReCon + DeepMimic, vault
    # 04_locomocao/referencia/pesquisas/12_especificacao_rastreador.md): o
    # lider (o padrao do jogo), o contrato, a nota e o lote de treino. Usa o
    # atleta (o padrao, os recursos) e do RL so o corpo (RLBody3D).
    add_library(MatterTracking STATIC
        src/Engine/Tracking/TrackingLeader3D.cpp
        src/Engine/Tracking/TrackingFeatures3D.cpp
        src/Engine/Tracking/TrackingContract3D.cpp
        src/Engine/Tracking/TrackingReward3D.cpp
        src/Engine/Tracking/TrackingTraining3D.cpp
    )
    target_include_directories(MatterTracking PUBLIC src)
    target_link_libraries(MatterTracking PUBLIC MatterAthlete MatterRLContract)
    if(MSVC)
        target_compile_options(MatterTracking PRIVATE /W4 /permissive- /utf-8)
    else()
        target_compile_options(MatterTracking PRIVATE -Wall -Wextra -Wpedantic)
    endif()

    # Os mundos de treino do Corpo: a tarefa, a nota e o lote paralelo.
    # Depende so da fisica, dos clipes, do contrato e do piloto (nada de
    # janela, render ou Python).
    add_library(MatterRLEnvironment STATIC
        src/Engine/RL/Environment/BodyTask3D.cpp
        src/Engine/RL/Environment/BodyTrainingBatch3D.cpp
        src/Engine/RL/Rewards/BodyReward3D.cpp
        src/Engine/RL/Style/StyleFeatures3D.cpp
        src/Engine/RL/Style/MotionDataset3D.cpp
        src/Engine/RL/Imitation/ReferenceMotion3D.cpp
    )
    target_include_directories(MatterRLEnvironment PUBLIC src)
    target_link_libraries(MatterRLEnvironment
        PUBLIC MatterRLContract MatterRLCommands MatterPhysics MatterAnimationClips
        PRIVATE nlohmann_json::nlohmann_json)
    if(MSVC)
        target_compile_options(MatterRLEnvironment PRIVATE
            /W4 /permissive- /utf-8)
    else()
        target_compile_options(MatterRLEnvironment PRIVATE
            -Wall -Wextra -Wpedantic)
    endif()

    if(MATTERENGINE_BUILD_RL_PYTHON)
        # O Python e o do ambiente do treinador (tools/rl/.venv), que traz o
        # nanobind; o modulo sai em <build>/python.
        find_package(Python 3.14 COMPONENTS Interpreter Development.Module REQUIRED)
        execute_process(COMMAND "${Python_EXECUTABLE}" -m nanobind --cmake_dir
            OUTPUT_STRIP_TRAILING_WHITESPACE OUTPUT_VARIABLE nanobind_ROOT)
        find_package(nanobind CONFIG REQUIRED)
        nanobind_add_module(matter_rl_native NB_STATIC bindings/python/MatterRLNative.cpp)
        target_link_libraries(matter_rl_native PRIVATE MatterRLEnvironment MatterRLPolicy)
        set_target_properties(matter_rl_native PROPERTIES
            LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/python")
        # O treino da locomocao (o atleta do jogo, contrato v2): um modulo a
        # parte (o RL do zero tem outro ReferenceMotion3D).
        nanobind_add_module(matter_locomotion_native NB_STATIC bindings/python/MatterLocomotionNative.cpp)
        target_link_libraries(matter_locomotion_native PRIVATE MatterAthleteTraining)
        set_target_properties(matter_locomotion_native PROPERTIES
            LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/python")
        # O rastreador fisico (o lote do treino dele).
        nanobind_add_module(matter_tracking_native NB_STATIC bindings/python/MatterTrackingNative.cpp)
        target_link_libraries(matter_tracking_native PRIVATE MatterTracking)
        set_target_properties(matter_tracking_native PROPERTIES
            LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/python")
    endif()
endif()

add_library(MatterVulkan STATIC
    src/Engine/Render/SceneLightPacking.cpp
    src/Engine/RHI/Vulkan/ScenePassGraph.cpp
    src/Engine/RHI/Vulkan/VulkanDevice.cpp
)
target_include_directories(MatterVulkan PUBLIC src)
target_compile_definitions(MatterVulkan PRIVATE VK_NO_PROTOTYPES VMA_STATIC_VULKAN_FUNCTIONS=0 VMA_DYNAMIC_VULKAN_FUNCTIONS=1 MATTERENGINE_SHADER_DIR="${MATTERENGINE_SHADER_OUTPUT_DIR}")
target_link_libraries(MatterVulkan
    PUBLIC MatterRHI
    PRIVATE MatterCore SDL3::SDL3-static imgui volk::volk GPUOpen::VulkanMemoryAllocator
)
add_dependencies(MatterVulkan MatterEngineShaders)

# Modulo de audio isolado: nenhum consumidor enxerga o backend de mixagem
# (OpenAL Soft). Mesma fronteira de MatterPhysics (API neutra publica,
# biblioteca de terceiros privada) - nenhum header AL/ALC/EFX atravessa
# AudioDevice3D.hpp. Sem arquivos de audio: o som e gerado por codigo (por
# enquanto so o vento; o sistema de som sera refeito por receita).
add_library(MatterAudio STATIC
    src/Engine/Audio/AcousticEnvironmentSystem3D.cpp
    src/Engine/Audio/AudioDevice3D.cpp
    src/Engine/Audio/BiquadFilter.cpp
    src/Engine/Audio/Envelope.cpp
    src/Engine/Audio/ProceduralNoise.cpp
    src/Engine/Audio/Waveform.cpp
    src/Engine/Audio/WindAmbienceRamp.cpp
    src/Engine/Audio/WindSound.cpp
)
target_include_directories(MatterAudio PUBLIC src)
target_link_libraries(MatterAudio
    PUBLIC MatterCore
    PRIVATE OpenAL
)
if(MSVC)
    target_compile_options(MatterAudio PRIVATE /W4 /permissive- /utf-8)
else()
    target_compile_options(MatterAudio PRIVATE -Wall -Wextra -Wpedantic)
endif()

# Modelagem do personagem (07/10): malha e kit de modelagem, esqueleto e
# regioes do corpo, analise da malha e o renderizador por software das
# ferramentas de inspecao. So depende do nucleo: roda sem janela (a
# ferramenta MatterModelInspector) e gera a mesma imagem que a tela
# "Inspecao de jogador" mostra.
set(MATTERMODELING_SOURCES
    src/Engine/Modeling/Geometry/ModelGeometry.cpp
    src/Engine/Modeling/Geometry/Parallel.cpp
    src/Engine/Modeling/Geometry/Triangulation2D.cpp
    src/Engine/Modeling/Raster/Image.cpp
    src/Engine/Modeling/Raster/ImageIO.cpp
    src/Engine/Modeling/Raster/Camera.cpp
    src/Engine/Modeling/Raster/Rasterizer.cpp
    src/Engine/Modeling/Raster/TextRenderer.cpp
    src/Engine/Modeling/Raster/StbModelingImpl.cpp
    src/Engine/Modeling/Skeleton/Skeleton.cpp
    src/Engine/Modeling/Skeleton/BodyRegions.cpp
    src/Engine/Modeling/Mesh/Mesh.cpp
    src/Engine/Modeling/Mesh/MeshBuilder.cpp
    src/Engine/Modeling/Mesh/MeshSkinning.cpp
    src/Engine/Modeling/Mesh/ShadingNormals.cpp
    src/Engine/Modeling/Mesh/TriangleBvh.cpp
    src/Engine/Modeling/Mesh/MeshTopology.cpp
    src/Engine/Modeling/Mesh/MeshStitch.cpp
    src/Engine/Modeling/Mesh/MeshImprint.cpp
    src/Engine/Modeling/Mesh/MeshRelief.cpp
    src/Engine/Modeling/Mesh/MeshRepair.cpp
    src/Engine/Modeling/Mesh/MeshCut.cpp
    src/Engine/Modeling/Mesh/MeshSubdivision.cpp
    src/Engine/Modeling/Mesh/MeshSculpt.cpp
    src/Engine/Modeling/Mesh/SurfaceFit.cpp
    src/Engine/Modeling/Import/GltfImport.cpp
    src/Engine/Modeling/Analysis/Report.cpp
    src/Engine/Modeling/Analysis/TopologyCheck.cpp
    src/Engine/Modeling/Analysis/ConnectivityCheck.cpp
    src/Engine/Modeling/Analysis/IntersectionCheck.cpp
    src/Engine/Modeling/Analysis/SymmetryCheck.cpp
    src/Engine/Modeling/Analysis/SkinCheck.cpp
    src/Engine/Modeling/Analysis/CrossSection.cpp
    src/Engine/Modeling/Analysis/DeformationCheck.cpp
    src/Engine/Modeling/Analysis/ModelValidator.cpp
    src/Engine/Modeling/Inspection/InspectionRenderer.cpp
    src/Engine/Modeling/Inspection/ViewFraming.cpp
    src/Engine/Modeling/Inspection/InspectionSession.cpp
    src/Engine/Modeling/Inspection/ContactSheet.cpp
    src/Engine/Modeling/Inspection/ReferenceComparison.cpp
    src/Engine/Modeling/Parts/ProfileCurve.cpp
    src/Engine/Modeling/Parts/RecipeJson.cpp
    src/Engine/Modeling/Parts/BootRecipe.cpp
    src/Engine/Modeling/Parts/BootShape.cpp
    src/Engine/Modeling/Parts/SurfaceDrape.cpp
    src/Engine/Modeling/Parts/BootLaces.cpp
    src/Engine/Modeling/Parts/BootSole.cpp
    src/Engine/Modeling/Parts/BootTongue.cpp
    src/Engine/Modeling/Parts/BootGenerator.cpp
    src/Engine/Modeling/Parts/LegRecipe.cpp
    src/Engine/Modeling/Parts/LegGenerator.cpp
    src/Engine/Modeling/Parts/ShortsRecipe.cpp
    src/Engine/Modeling/Parts/ShortsGenerator.cpp
    src/Engine/Modeling/Parts/ShapeField.cpp
    src/Engine/Modeling/Parts/NumberGlyphs.cpp
    src/Engine/Modeling/Parts/ShirtRecipe.cpp
    src/Engine/Modeling/Parts/ShirtGenerator.cpp
    src/Engine/Modeling/Parts/ArmRecipe.cpp
    src/Engine/Modeling/Parts/ArmGenerator.cpp
    src/Engine/Modeling/Parts/HandRecipe.cpp
    src/Engine/Modeling/Parts/HandGenerator.cpp
    src/Engine/Modeling/Parts/NeckRecipe.cpp
    src/Engine/Modeling/Parts/NeckGenerator.cpp
    src/Engine/Modeling/Parts/HeadRecipe.cpp
    src/Engine/Modeling/Parts/HeadFace.cpp
    src/Engine/Modeling/Parts/HeadEye.cpp
    src/Engine/Modeling/Parts/HeadBrow.cpp
    src/Engine/Modeling/Parts/HeadEar.cpp
    src/Engine/Modeling/Parts/HeadGenerator.cpp
    src/Engine/Modeling/Parts/PartLibrary.cpp
    src/Engine/Modeling/Assembly/AssemblyRecipe.cpp
    src/Engine/Modeling/Assembly/MeshEdit.cpp
    src/Engine/Modeling/Assembly/NeckJoin.cpp
    src/Engine/Modeling/Assembly/EyeSocket.cpp
    src/Engine/Modeling/Assembly/Garment.cpp
    src/Engine/Modeling/Assembly/PlayerAssembly.cpp
    src/Engine/Modeling/Assembly/Uniform.cpp
    src/Engine/Modeling/Assembly/NeckShortening.cpp
    src/Engine/Modeling/Assembly/NeckBand.cpp
    src/Engine/Modeling/Assembly/ReferenceFit.cpp
    src/Engine/Modeling/Assembly/PlayerPaint.cpp
    src/Engine/Modeling/Assembly/FacePaint.cpp
    src/Engine/Modeling/Assembly/PlayerPaintSpaces.cpp
    src/Engine/Modeling/Texture/IslandPacking.cpp
    src/Engine/Modeling/Texture/UvAtlas.cpp
    src/Engine/Modeling/Texture/TextureBake.cpp
    src/Engine/Modeling/Objects/ObjectMesh.cpp
    src/Engine/Modeling/Objects/ObjectRecipe.cpp
    src/Engine/Modeling/Objects/PixelCanvas.cpp
    src/Engine/Modeling/Objects/ProceduralObject.cpp
    src/Engine/Modeling/Objects/WoodCrate.cpp
    src/Engine/Modeling/Paint/Shape2D.cpp
    src/Engine/Modeling/Paint/SvgPath.cpp
    src/Engine/Modeling/Paint/PaintFont.cpp
    src/Engine/Modeling/Paint/PaintImage.cpp
    src/Engine/Modeling/Paint/PaintLayer.cpp
    src/Engine/Modeling/Paint/PaintSpace.cpp
    src/Engine/Modeling/Paint/PaintDesign.cpp
    src/Engine/Modeling/Skin/BoneHeatSkin.cpp
    src/Engine/Modeling/Skin/SkinSmoothing.cpp
)
add_library(MatterModeling STATIC ${MATTERMODELING_SOURCES})
target_include_directories(MatterModeling PUBLIC src)
find_package(Threads REQUIRED)
target_link_libraries(MatterModeling
    PUBLIC MatterCore Threads::Threads
    PRIVATE nlohmann_json::nlohmann_json stb_image cgltf
)
target_compile_definitions(MatterModeling PRIVATE
    MATTERMODELING_FONT_PATH="${CMAKE_SOURCE_DIR}/assets/fonts/roboto/Roboto-Medium.ttf")
# O codigo do stb (PNG e TrueType) compila sem os avisos do projeto.
if(MSVC)
    set_source_files_properties(src/Engine/Modeling/Raster/StbModelingImpl.cpp
        PROPERTIES COMPILE_OPTIONS "/w")
else()
    set_source_files_properties(src/Engine/Modeling/Raster/StbModelingImpl.cpp
        PROPERTIES COMPILE_OPTIONS "-w")
endif()

# A politica exportada pelo treinador, para a tela e o jogo: so a MLP e
# a normalizacao, sem fisica nem treinador. Sempre construida: o atleta do
# jogo roda a rede da locomocao (Athlete/Policy).
add_library(MatterRLPolicy STATIC
    src/Engine/RL/Policy/PolicyRuntime3D.cpp
)
target_include_directories(MatterRLPolicy PUBLIC src)
target_link_libraries(MatterRLPolicy PRIVATE nlohmann_json::nlohmann_json)
if(MSVC)
    target_compile_options(MatterRLPolicy PRIVATE /W4 /permissive- /utf-8)
else()
    # -O3: a conta da rede vetoriza (o -O2 do RelWithDebInfo usa o custo
    # "very-cheap" e deixa o laco escalar).
    target_compile_options(MatterRLPolicy PRIVATE -Wall -Wextra -Wpedantic
        $<$<NOT:$<CONFIG:Debug>>:-O3>)
endif()

# O atleta (o runtime, o controle fisico e o padrao, os planejadores, a ajuda,
# o contrato da rede e o mundo de treino): sem render, janela nem audio - o
# jogo, as sondas e o treino de RL (a ponte Python) usam a mesma biblioteca.
add_library(MatterAthlete STATIC
    src/Engine/Animation/RagdollCharacter3D.cpp
    src/Engine/Athlete/AthleteAssets3D.cpp
    src/Engine/Athlete/AthleteRuntime3D.cpp
    src/Engine/Athlete/AthleteTelemetry3D.cpp
    src/Engine/Athlete/Actions/BallActionPlanner3D.cpp
    src/Engine/Athlete/Actions/BallGesture3D.cpp
    src/Engine/Athlete/Actions/BallHandling3D.cpp
    src/Engine/Athlete/Actions/BallMove3D.cpp
    src/Engine/Athlete/Actions/BallPlay3D.cpp
    src/Engine/Athlete/Ball/BallOutcome3D.cpp
    src/Engine/Athlete/Dribble/BodyForecast3D.cpp
    src/Engine/Athlete/Dribble/ContactPose3D.cpp
    src/Engine/Athlete/Dribble/DribbleBall3D.cpp
    src/Engine/Athlete/Dribble/Dribble3D.cpp
    src/Engine/Athlete/Dribble/SwingTouch3D.cpp
    src/Engine/Athlete/Dribble/TouchBody3D.cpp
    src/Engine/Athlete/Dribble/TouchPlanner3D.cpp
    src/Engine/Athlete/Intent/ActionQueue3D.cpp
    src/Engine/Athlete/Sensing/BallPredictor3D.cpp
    src/Engine/Athlete/Legacy/LegacyBallBridge3D.cpp
    src/Engine/Athlete/Body/ActiveBodyController3D.cpp
    src/Engine/Athlete/Body/ActiveStance3D.cpp
    src/Engine/Athlete/Body/PhysicalFeetAssist3D.cpp
    src/Engine/Athlete/Body/SupportWrench3D.cpp
    src/Engine/Athlete/Core/FootGeometry3D.cpp
    src/Engine/Athlete/Core/Kinematics3D.cpp
    src/Engine/Athlete/Planning/FootSpacing3D.cpp
    src/Engine/Athlete/Planning/FootstepPlanner3D.cpp
    src/Engine/Athlete/Planning/MotionPlanner3D.cpp
    src/Engine/Athlete/Sensing/BodyObservation3D.cpp
    src/Engine/Athlete/Synthesis/LegClearance3D.cpp
    src/Engine/Athlete/Synthesis/LegSynthesis3D.cpp
    src/Engine/Athlete/Legacy/LegacyAthleteStep3D.cpp
    src/Engine/Athlete/Legacy/LegacyReference3D.cpp
    src/Engine/Athlete/Modes/ContactClasses3D.cpp
    src/Engine/Athlete/Modes/ModeRules3D.cpp
    src/Engine/Athlete/Reference/ReferenceFrame3D.cpp
    src/Engine/Athlete/Reference/PhaseLock3D.cpp
    src/Engine/Athlete/Physical/PhysicalCorrections3D.cpp
    src/Engine/Athlete/Physical/DynamicTurnMove3D.cpp
    src/Engine/Athlete/Contract/LocomotionContract3D.cpp
    src/Engine/Athlete/Policy/LocomotionPolicy3D.cpp
    src/Engine/Athlete/Training/LocomotionWorld3D.cpp
    src/Engine/Athlete/Training/LocomotionBatch3D.cpp
    src/Engine/Character/BiomechanicalBipedExperiment3D.cpp
    src/Engine/Character/CharacterControlApplication3D.cpp
    src/Engine/Character/CharacterLocomotion3D.cpp
    src/Engine/Character/CharacterLocomotion3D_Sense.cpp
    src/Engine/Character/CharacterLocomotion3D_Tackle.cpp
    src/Engine/Character/CharacterLocomotion3D_GetUp.cpp
    src/Engine/Character/CharacterLocomotion3D_Authority.cpp
    src/Engine/Character/CharacterLocomotion3D_Sensors.cpp
    src/Engine/Character/CharacterLocomotion3D_Heading.cpp
    src/Engine/Character/CharacterLocomotion3D_Air.cpp
    src/Engine/Character/CharacterLocomotion3D_Cycle.cpp
    src/Engine/Character/CharacterLocomotion3D_Gestures.cpp
    src/Engine/Character/CharacterLocomotion3D_Look.cpp
    src/Engine/Character/CharacterLocomotion3D_Root.cpp
    src/Engine/Character/CharacterLocomotion3D_Feet.cpp
    src/Engine/Character/CharacterLocomotion3D_Legs.cpp
    src/Engine/Character/CharacterLocomotion3D_Drives.cpp
    src/Engine/Character/CharacterLocomotion3D_Outputs.cpp
    src/Engine/Character/RagdollPoseKinematics3D.cpp
    src/Engine/Character/BodilyGetUp3D.cpp
    src/Engine/Character/BallControl3D.cpp
    src/Engine/Character/BallGesturePose3D.cpp
    src/Engine/Character/BallOwnership3D.cpp
    src/Engine/Character/BallTouchPreview3D.cpp
    src/Engine/Character/DefaultLocomotion3D.cpp
    src/Engine/Character/DefaultLocomotion3D_Travel.cpp
    src/Engine/Character/DefaultLocomotion3D_Tackle.cpp
    src/Engine/Character/DefaultLocomotion3D_Gait.cpp
    src/Engine/Character/DefaultLocomotion3D_Air.cpp
    src/Engine/Character/DefaultLocomotion3D_Pose.cpp
    src/Engine/Character/DefaultLocomotion3D_Layers.cpp
    src/Engine/Character/DefaultLocomotion3D_Feet.cpp
    src/Engine/Character/DefaultLocomotion3D_Exits.cpp
    src/Engine/Character/DefaultLocomotion3D_Readout.cpp
    src/Engine/Character/PoseInertializer3D.cpp
    src/Engine/Character/CharacterStateEstimator3D.cpp
    src/Engine/Control/CharacterWholeBody3D.cpp
    src/Engine/Character/ProceduralBipedGait3D.cpp
    src/Engine/Control/RagdollImpactTest3D.cpp
    src/Engine/Control/AdaptivePhysicalCharacter3D.cpp
    src/Engine/Control/ImpactAssist3D.cpp
    src/Engine/Locomotion/ContactFootwork3D.cpp
    src/Engine/Geometry/MeshData3D.cpp
)
target_include_directories(MatterAthlete PUBLIC src)
target_link_libraries(MatterAthlete
    PUBLIC MatterCore MatterPhysics MatterAnimationClips MatterRLPolicy
    PRIVATE nlohmann_json::nlohmann_json
)

add_library(MatterEngine STATIC
    src/Engine/Animation/RagdollVisualPose3D.cpp
    src/Engine/Core/Application.cpp
    src/Engine/Input/Input.cpp
    src/Engine/Platform/SDL3/SDLPlatform.cpp
    src/Engine/Platform/SDL3/SDLInputAdapter.cpp
    src/Engine/Math/Frustum3D.cpp
    src/Engine/Math/JitterSequence.cpp
    src/Engine/Math/ShadowCascade.cpp
    src/Engine/Environment/WindSystem.cpp
    src/Engine/Geometry/ProceduralSoccerBall3D.cpp
    src/Engine/Geometry/GltfLoader.cpp
    src/Engine/Geometry/GltfPhysicsMetadata3D.cpp
    src/Engine/Geometry/GltfAcousticZone3D.cpp
    src/Engine/Geometry/PhysicalAsset3D.cpp
    src/Engine/Render/Render2D.cpp
    src/Engine/Render/Renderer.cpp
    src/Engine/UI/ImGuiFontLibrary.cpp
    src/Engine/UI/ImGuiTheme.cpp
    src/Engine/Data/Database.cpp
    src/Engine/Data/SettingsRepository.cpp
)

target_include_directories(MatterEngine PUBLIC src)
target_link_libraries(MatterEngine
    PUBLIC MatterAthlete MatterCore MatterRHI MatterPhysics MatterAnimationClips MatterAudio
    PRIVATE MatterVulkan SDL3::SDL3-static imgui sqlite3
        cgltf stb_image nlohmann_json::nlohmann_json
)
add_dependencies(MatterEngine MatterEngineShaders)
target_compile_definitions(MatterEngine PRIVATE
    MATTERENGINE_SHADER_DIR="${MATTERENGINE_SHADER_OUTPUT_DIR}"
    MATTERENGINE_ASSETS_DIR="${CMAKE_SOURCE_DIR}/assets"
)

# A ponte entre o personagem do jogo e a modelagem: o esqueleto (links
# fisicos e dedos), as poses das animacoes, o manifesto do jogador e a
# exportacao do jogador para o jogo. Unica parte da modelagem que conhece a
# engine.
# Os objetos procedurais no jogo (a ponte do nucleo Modeling/Objects com a
# engine): a malha da GPU, o corpo fisico e o laboratorio de fisica deles.
add_library(MatterObjects STATIC
    src/Engine/Objects/ObjectAsset3D.cpp
    src/Engine/Objects/ObjectPhysicsLab3D.cpp
)
target_include_directories(MatterObjects PUBLIC src)
target_link_libraries(MatterObjects PUBLIC MatterEngine MatterModeling)

add_library(MatterModelingRig STATIC
    src/Engine/Modeling/Rig/GameRig.cpp
    src/Engine/Modeling/Rig/GameCharacterExport.cpp
    src/Engine/Modeling/Rig/PlayerModelAsset.cpp
    src/Engine/Modeling/Rig/RagdollSkeletonView.cpp
)
target_include_directories(MatterModelingRig PUBLIC src)
target_link_libraries(MatterModelingRig
    PUBLIC MatterModeling MatterEngine
    PRIVATE nlohmann_json::nlohmann_json
)
target_compile_definitions(MatterModelingRig PRIVATE MATTERENGINE_ASSETS_DIR="${CMAKE_SOURCE_DIR}/assets")

add_library(MatterWorkbench STATIC
    src/Workbench/WorkbenchApp.cpp
    src/Workbench/UiKit.cpp
    src/Workbench/Audio/WorldAudioController.cpp
    src/Workbench/Laboratory/PropRuntime.cpp
    src/Workbench/Laboratory/BallRuntime.cpp
    src/Workbench/Laboratory/RagdollRuntime.cpp
    src/Workbench/Laboratory/PhysicsBenchmark.cpp
    src/Workbench/Laboratory/RenderMotionSmoke.cpp
    src/Workbench/Props/PropCatalog.cpp
    src/Workbench/Screens/SettingsScreen.cpp
    src/Workbench/Screens/MainMenuScreen.cpp
    src/Workbench/Screens/LaboratoryScreen.cpp
    src/Workbench/Screens/ObjectInspectorHost.cpp
    src/Workbench/ObjectInspector/ObjectInspectorScreen.cpp
    src/Workbench/Screens/BallTouchViewerScreen.cpp
    src/Workbench/Screens/PlayerInspectorHost.cpp
    src/Workbench/PlayerInspector/PlayerInspectorScreen.cpp
    src/Workbench/PlayerInspector/PlayerAnimations.cpp
    src/Workbench/PlayerInspector/UiCanvasTexture.cpp
    src/Workbench/Screens/ReinforcementLearningHost.cpp
    src/Workbench/ReinforcementLearning/ReinforcementLearningScreen.cpp
    src/Workbench/ReinforcementLearning/LocomotionViewer3D.cpp
)

# A tela de Aprendizado por Reforco usa o modulo de RL inteiro (os mundos, a
# politica, a ligacao com o treinador e os colisores).
if(NOT MATTERENGINE_BUILD_RL)
    message(FATAL_ERROR "O Workbench precisa de MATTERENGINE_BUILD_RL (a tela de Aprendizado por Reforco)")
endif()
target_include_directories(MatterWorkbench PUBLIC src)
target_link_libraries(MatterWorkbench
    PUBLIC MatterEngine MatterModelingRig MatterObjects
    PRIVATE imgui meshoptimizer MatterRLEnvironment MatterRLPolicy MatterRLSupervisor MatterRLCalibration
)
target_compile_definitions(MatterWorkbench PRIVATE MATTERENGINE_ASSETS_DIR="${CMAKE_SOURCE_DIR}/assets")

add_executable(MatterEngineApp src/main.cpp)
set_target_properties(MatterEngineApp PROPERTIES OUTPUT_NAME "MatterEngine")
if(WIN32)
    set(MATTERENGINE_ICON_PATH "${CMAKE_SOURCE_DIR}/assets/app/matter-engine.ico")
    configure_file(cmake/MatterEngine.rc.in "${CMAKE_CURRENT_BINARY_DIR}/MatterEngine.rc" @ONLY)
    target_sources(MatterEngineApp PRIVATE "${CMAKE_CURRENT_BINARY_DIR}/MatterEngine.rc")
endif()
# SDL_main is confined to the executable composition root; application and
# mechanics sources continue to consume only MatterEngine platform APIs.
target_link_libraries(MatterEngineApp PRIVATE MatterWorkbench SDL3::SDL3-static)
add_custom_command(TARGET MatterEngineApp POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
        "${CMAKE_SOURCE_DIR}/assets/fonts"
        "$<TARGET_FILE_DIR:MatterEngineApp>/assets/fonts"
    COMMENT "Copying UI fonts next to MatterEngine"
)


option(MATTERENGINE_BUILD_TESTS "Build MatterEngine tests" ON)
if(MATTERENGINE_BUILD_TESTS)
    enable_testing()
    add_executable(MatterEngineTests tests/EngineFoundationTests.cpp)
    add_executable(MatterAdaptiveTests tests/AdaptivePhysicalCharacterTests.cpp)
    target_link_libraries(MatterAdaptiveTests PRIVATE MatterEngine)
    target_compile_definitions(MatterAdaptiveTests PRIVATE MATTERENGINE_TEST_ASSETS_DIR="${CMAKE_CURRENT_SOURCE_DIR}/assets")
    target_link_libraries(MatterEngineTests PRIVATE MatterEngine)
    target_compile_definitions(MatterEngineTests PRIVATE
        MATTERENGINE_TEST_ASSETS_DIR="${CMAKE_SOURCE_DIR}/assets")
    add_test(NAME MatterEngine.Foundation COMMAND MatterEngineTests)
    add_test(NAME MatterEngine.Character COMMAND MatterEngineTests)
    set_tests_properties(MatterEngine.Character PROPERTIES
        ENVIRONMENT "MATTERENGINE_TEST_FILTER=character")

    # Linkado somente contra MatterAudio (nunca MatterEngine/Vulkan/imgui) para
    # provar que o modulo de audio nao vaza dependencia de renderizacao.
    add_executable(MatterAudioTests tests/AudioFoundationTests.cpp)
    target_link_libraries(MatterAudioTests PRIVATE MatterAudio)
    add_test(NAME MatterEngine.Audio COMMAND MatterAudioTests)
    # Modo fisico adaptativo (experimental): movimento normal no laco do jogo
    # sem a raiz carregada, e o contrato de que perturbacoes agem no corpo.
    add_test(NAME MatterEngine.AdaptivePhysics COMMAND MatterAdaptiveTests)
    # Bola inflada (a bola de futebol): modelo e cena contra a FIFA.
    add_executable(MatterBallTests tests/InflatedBallTests.cpp)
    target_link_libraries(MatterBallTests PRIVATE MatterEngine)
    add_test(NAME MatterEngine.Ball COMMAND MatterBallTests)
    if(MATTERENGINE_BUILD_RL)
        # Primeiro contrato de RL: a auditoria declarativa do corpo compila
        # apenas contra o modulo headless de calibracao/fisica.
        # O treino da locomocao: a nota, a tarefa e o lote (curto, sem rede).
        add_executable(MatterLocomotionTrainingTests tests/Athlete/LocomotionTrainingTests.cpp)
        target_link_libraries(MatterLocomotionTrainingTests PRIVATE MatterAthleteTraining)
        target_compile_definitions(MatterLocomotionTrainingTests PRIVATE
            MATTERENGINE_ASSETS_DIR="${CMAKE_SOURCE_DIR}/assets")
        add_test(NAME MatterEngine.LocomotionTraining COMMAND MatterLocomotionTrainingTests)
        # O rastreador fisico: o contrato, a nota e o lote (curto, sem rede).
        add_executable(MatterTrackingTests tests/Tracking/TrackingTests.cpp)
        target_link_libraries(MatterTrackingTests PRIVATE MatterTracking)
        target_compile_definitions(MatterTrackingTests PRIVATE MATTERENGINE_ASSETS_DIR="${CMAKE_SOURCE_DIR}/assets")
        add_test(NAME MatterEngine.Tracking COMMAND MatterTrackingTests)
        add_executable(MatterRLCalibrationTests
            tests/RL/RagdollCalibrationAuditTests.cpp)
        target_link_libraries(MatterRLCalibrationTests
            PRIVATE MatterRLCalibration)
        target_compile_definitions(MatterRLCalibrationTests PRIVATE
            MATTERENGINE_TEST_ASSETS_DIR="${CMAKE_SOURCE_DIR}/assets")
        add_test(NAME MatterEngine.RLCalibration
            COMMAND MatterRLCalibrationTests)
        add_executable(MatterRLJointLimitLabTests
            tests/RL/RagdollJointLimitLabTests.cpp)
        target_link_libraries(MatterRLJointLimitLabTests
            PRIVATE MatterRLCalibration)
        target_compile_definitions(MatterRLJointLimitLabTests PRIVATE
            MATTERENGINE_TEST_ASSETS_DIR="${CMAKE_SOURCE_DIR}/assets")
        add_test(NAME MatterEngine.RLJointLimits
            COMMAND MatterRLJointLimitLabTests)
        add_executable(MatterRLDynamicsTests
            tests/RL/RagdollDynamicLabTests.cpp)
        target_link_libraries(MatterRLDynamicsTests
            PRIVATE MatterRLCalibration)
        target_compile_definitions(MatterRLDynamicsTests PRIVATE
            MATTERENGINE_TEST_ASSETS_DIR="${CMAKE_SOURCE_DIR}/assets")
        add_test(NAME MatterEngine.RLDynamics
            COMMAND MatterRLDynamicsTests)
        add_executable(MatterRLTrainingTests
            tests/RL/RLBodyTests.cpp
            tests/RL/BodyContractTests.cpp
            tests/RL/BodyBatchTests.cpp
            tests/RL/StyleTests.cpp
            tests/RL/MirrorTests.cpp
            tests/RL/BodyBrainTests.cpp
            tests/RL/CenterOfMassTests.cpp
            tests/RL/AcquisitionTests.cpp
            tests/RL/RLTestMain.cpp)
        target_link_libraries(MatterRLTrainingTests
            PRIVATE MatterRLEnvironment MatterRLBrain)
        target_compile_definitions(MatterRLTrainingTests PRIVATE
            MATTERENGINE_TEST_ASSETS_DIR="${CMAKE_SOURCE_DIR}/assets")
        add_test(NAME MatterEngine.RLTraining
            COMMAND MatterRLTrainingTests)
    endif()
    # Modelagem do personagem: o nucleo (linkado so contra MatterModeling,
    # provando que ele nao depende da engine) e a ponte com o jogo, com a tela
    # "Inspecao de jogador" rodando sem janela.
    add_executable(MatterModelingTests tests/Modeling/ModelingTests.cpp)
    target_link_libraries(MatterModelingTests PRIVATE MatterModeling)
    add_test(NAME MatterEngine.Modeling COMMAND MatterModelingTests)
    add_executable(MatterModelingPartsTests tests/Modeling/PartsTests.cpp)
    target_link_libraries(MatterModelingPartsTests PRIVATE MatterModeling)
    target_compile_definitions(MatterModelingPartsTests PRIVATE
        MATTERMODELING_RECIPE_PATH="${CMAKE_SOURCE_DIR}/assets/characters/player_model/partes/chuteira.json"
        MATTERMODELING_PARTS_DIR="${CMAKE_SOURCE_DIR}/assets/characters/player_model/partes")
    add_test(NAME MatterEngine.ModelingParts COMMAND MatterModelingPartsTests)
    # Os objetos procedurais (o nucleo: receitas, geradores, pixel art) - so
    # contra MatterModeling, como os outros testes do nucleo.
    add_executable(MatterObjectModelingTests tests/Modeling/ObjectModelingTests.cpp)
    target_link_libraries(MatterObjectModelingTests PRIVATE MatterModeling)
    target_compile_definitions(MatterObjectModelingTests PRIVATE
        MATTERENGINE_TEST_ASSETS_DIR="${CMAKE_SOURCE_DIR}/assets")
    add_test(NAME MatterEngine.ObjectModeling COMMAND MatterObjectModelingTests)
    add_executable(MatterObjectPhysicsTests tests/ObjectPhysicsTests.cpp)
    target_link_libraries(MatterObjectPhysicsTests PRIVATE MatterObjects)
    target_compile_definitions(MatterObjectPhysicsTests PRIVATE
        MATTERENGINE_TEST_ASSETS_DIR="${CMAKE_SOURCE_DIR}/assets")
    add_test(NAME MatterEngine.ObjectPhysics COMMAND MatterObjectPhysicsTests)
    add_executable(MatterModelingPaintTests tests/Modeling/PaintTests.cpp)
    target_link_libraries(MatterModelingPaintTests PRIVATE MatterModeling)
    target_compile_definitions(MatterModelingPaintTests PRIVATE
        MATTERPAINT_FONT_PATH="${CMAKE_SOURCE_DIR}/assets/fonts/roboto/Roboto-Medium.ttf")
    add_test(NAME MatterEngine.ModelingPaint COMMAND MatterModelingPaintTests)
    add_executable(MatterModelingAssemblyTests tests/Modeling/AssemblyTests.cpp)
    target_link_libraries(MatterModelingAssemblyTests PRIVATE MatterModeling)
    target_compile_definitions(MatterModelingAssemblyTests PRIVATE
        MATTERMODELING_BASE_DIR="${CMAKE_SOURCE_DIR}/assets/characters/player_model/base")
    add_test(NAME MatterEngine.ModelingAssembly COMMAND MatterModelingAssemblyTests)
    # As telas do Workbench sem janela nos testes: o contexto ImGui com a fonte
    # e o tema, o canvas falso e a foto da tela (MATTERENGINE_UI_SNAPSHOT).
    add_library(MatterTestHeadlessUi STATIC tests/Support/HeadlessUi.cpp)
    target_include_directories(MatterTestHeadlessUi PUBLIC tests)
    target_link_libraries(MatterTestHeadlessUi PUBLIC MatterWorkbench imgui)
    add_executable(MatterPlayerModelTests tests/Modeling/PlayerModelTests.cpp)
    target_link_libraries(MatterPlayerModelTests PRIVATE MatterTestHeadlessUi MatterModelingRig)
    add_test(NAME MatterEngine.PlayerModel COMMAND MatterPlayerModelTests)
    # A tela "Objetos" sem janela (a receita numa copia temporaria).
    add_executable(MatterObjectInspectorTests tests/ObjectInspectorTests.cpp)
    target_link_libraries(MatterObjectInspectorTests PRIVATE MatterTestHeadlessUi)
    target_compile_definitions(MatterObjectInspectorTests PRIVATE
        MATTERENGINE_TEST_ASSETS_DIR="${CMAKE_SOURCE_DIR}/assets")
    add_test(NAME MatterEngine.ObjectInspector COMMAND MatterObjectInspectorTests)
    # A tela "Aprendizado por Reforco" sem janela (um treino de mentira numa
    # pasta temporaria).
    add_executable(MatterReinforcementLearningScreenTests tests/ReinforcementLearningScreenTests.cpp)
    target_link_libraries(MatterReinforcementLearningScreenTests PRIVATE MatterTestHeadlessUi MatterRLPolicy MatterRLEnvironment
        sqlite3 nlohmann_json::nlohmann_json)
    target_compile_definitions(MatterReinforcementLearningScreenTests PRIVATE
        MATTERENGINE_TEST_ASSETS_DIR="${CMAKE_SOURCE_DIR}/assets")
    add_test(NAME MatterEngine.ReinforcementLearningScreen COMMAND MatterReinforcementLearningScreenTests)
endif()

# Smoke test manual: abre uma janela real e percorre resize, minimizar/restaurar
# e modos fullscreen. Fica fora do build padrão para não interromper CI/headless.
add_executable(MatterVulkanSmoke EXCLUDE_FROM_ALL tools/VulkanSmoke.cpp)
target_link_libraries(MatterVulkanSmoke PRIVATE MatterEngine SDL3::SDL3-static)

# Benchmark manual e reproduzivel da carga-alvo atual. Publica percentis; nao
# entra no CTest porque desempenho e dependente do hardware e da configuracao
# da build.
add_executable(MatterPhysicsBenchmark EXCLUDE_FROM_ALL
    tools/PhysicsBenchmark.cpp)
target_link_libraries(MatterPhysicsBenchmark PRIVATE MatterEngine)
target_compile_definitions(MatterPhysicsBenchmark PRIVATE
    MATTERENGINE_ASSETS_DIR="${CMAKE_SOURCE_DIR}/assets")

if(MATTERENGINE_BUILD_RL)
    # Ferramenta headless de baseline: imprime Markdown em stdout e retorna
    # erro somente para perfil invalido (ou aviso com --strict).
    add_executable(MatterRLAudit tools/RLAudit.cpp)
    target_link_libraries(MatterRLAudit PRIVATE MatterRLCalibration)
    target_compile_definitions(MatterRLAudit PRIVATE
        MATTERENGINE_ASSETS_DIR="${CMAKE_SOURCE_DIR}/assets")
    add_executable(MatterRLDynamics tools/RLDynamics.cpp)
    target_link_libraries(MatterRLDynamics PRIVATE MatterRLCalibration)
    target_compile_definitions(MatterRLDynamics PRIVATE
        MATTERENGINE_ASSETS_DIR="${CMAKE_SOURCE_DIR}/assets")
    add_executable(MatterRLJointProbe tools/RLJointProbe.cpp)
    target_link_libraries(MatterRLJointProbe PRIVATE MatterRLCalibration)
    target_compile_definitions(MatterRLJointProbe PRIVATE
        MATTERENGINE_ASSETS_DIR="${CMAKE_SOURCE_DIR}/assets")
    # O Experimento A da reformulacao do RL: os motores do corpo v2 pelo
    # contrato (a reta entre decisoes, o ruido de exploracao, a ponta do pe).
    add_executable(MatterRLMotorProbe tools/RLMotorProbe.cpp)
    target_link_libraries(MatterRLMotorProbe PRIVATE MatterRLContract)
    target_compile_definitions(MatterRLMotorProbe PRIVATE
        MATTERENGINE_ASSETS_DIR="${CMAKE_SOURCE_DIR}/assets")
    # O custo da politica no jogo (22 jogadores a 30 Hz), pelo runtime de hoje.
    add_executable(MatterRLPolicyBench tools/RLPolicyBench.cpp)
    target_link_libraries(MatterRLPolicyBench PRIVATE MatterRLPolicy)
endif()

# Visualizador de toque na bola na linha de comando (a mesma simulacao da
# tela "Toque na bola" do Workbench; os desenhos saem de
# tools/analysis/ball_touch_preview.py).
# Sonda da locomocao padrao (o boneco pela capsula, sem fisica): roteiros
# de teclas, medidas e quadros para os desenhos.
add_executable(MatterLocomotionProbe tools/DefaultLocomotionProbe.cpp)
target_link_libraries(MatterLocomotionProbe PRIVATE MatterEngine)
target_compile_definitions(MatterLocomotionProbe PRIVATE
    MATTERENGINE_ASSETS_DIR="${CMAKE_SOURCE_DIR}/assets")

# Ferramentas de modelagem do personagem na linha de comando (renderizar,
# analisar, medir, cortar, comparar com as referencias, poses): o que o
# Claude e o Codex usam para modelar. Mesmo codigo da tela "Inspecao de
# jogador".
add_executable(MatterModelInspector
    tools/ModelInspector/main.cpp
    tools/ModelInspector/CommandLine.cpp
    tools/ModelInspector/ModelContext.cpp
    tools/ModelInspector/ViewOptions.cpp
    tools/ModelInspector/CommandInfo.cpp
    tools/ModelInspector/CommandRender.cpp
    tools/ModelInspector/CommandCheck.cpp
    tools/ModelInspector/CommandMeasure.cpp
    tools/ModelInspector/CommandUniform.cpp
    tools/ModelInspector/CommandPosture.cpp
    tools/ModelInspector/CommandWeights.cpp
    tools/ModelInspector/CommandSection.cpp
    tools/ModelInspector/CommandCompare.cpp
    tools/ModelInspector/CommandPoses.cpp
    tools/ModelInspector/CommandTexture.cpp
    tools/ModelInspector/CommandMold.cpp
    tools/ModelInspector/CommandInverted.cpp
    tools/ModelInspector/CommandPaint.cpp
    tools/ModelInspector/CommandExport.cpp
    tools/ModelInspector/CommandHelp.cpp
    tools/ModelInspector/CommandFrames.cpp
)
target_link_libraries(MatterModelInspector PRIVATE MatterModelingRig)
if(MATTERENGINE_BUILD_RL)
    target_sources(MatterModelInspector PRIVATE
        tools/ModelInspector/CommandColliders.cpp)
    target_link_libraries(MatterModelInspector PRIVATE MatterRLCalibration)
    target_compile_definitions(MatterModelInspector PRIVATE
        MATTERENGINE_MODEL_INSPECTOR_RL=1
        MATTERENGINE_ASSETS_DIR="${CMAKE_SOURCE_DIR}/assets")
endif()

# Sonda dos atletas (rework de 10/10/2026): o mesmo runtime do laboratorio
# com a fisica de verdade, sem janela; o dump vai para MatterModelInspector quadros.
add_executable(MatterAthleteProbe tools/AthleteProbe.cpp tools/athlete_probe/ProbeL0.cpp)
target_include_directories(MatterAthleteProbe PRIVATE tools)
target_link_libraries(MatterAthleteProbe PRIVATE MatterEngine)
target_compile_definitions(MatterAthleteProbe PRIVATE
    MATTERENGINE_ASSETS_DIR="${CMAKE_SOURCE_DIR}/assets"
)
if(MATTERENGINE_BUILD_TESTS)
    # Os contratos do runtime dos atletas (Corpo Ativo, volta do fisico...).
    add_test(NAME MatterEngine.Athlete COMMAND MatterAthleteProbe --teste)
    # Os modulos novos do atleta (a reescrita da locomocao): modulos puros,
    # testados sozinhos (unidade, a tabela de trocas, propriedades).
    add_executable(MatterAthleteModulesTests tests/Athlete/AthleteModulesTests.cpp)
    target_link_libraries(MatterAthleteModulesTests PRIVATE MatterEngine)
    add_test(NAME MatterEngine.AthleteModules COMMAND MatterAthleteModulesTests)
endif()

add_executable(MatterBallPreview tools/BallTouchPreview.cpp)
target_link_libraries(MatterBallPreview PRIVATE MatterEngine)
target_compile_definitions(MatterBallPreview PRIVATE
    MATTERENGINE_ASSETS_DIR="${CMAKE_SOURCE_DIR}/assets")

set_target_properties(MatterCore MatterRHI MatterPhysics MatterAudio MatterVulkan
    MatterEngine MatterEngineShaders MatterModeling PROPERTIES FOLDER "MatterEngine")
set_target_properties(MatterWorkbench MatterEngineApp
    PROPERTIES FOLDER "MatterEngine Workbench")
set_target_properties(MatterVulkanSmoke PROPERTIES FOLDER "Tests")
set_target_properties(MatterPhysicsBenchmark PROPERTIES FOLDER "Tests")
if(MATTERENGINE_BUILD_RL)
    set_target_properties(MatterRLCalibration MatterRLEnvironment PROPERTIES
        FOLDER "MatterEngine RL")
    set_target_properties(MatterRLAudit MatterRLDynamics MatterRLJointProbe
        PROPERTIES FOLDER "Tools")
endif()
if(MATTERENGINE_BUILD_TESTS)
    set_target_properties(MatterEngineTests MatterAudioTests
        PROPERTIES FOLDER "Tests")
    if(MATTERENGINE_BUILD_RL)
        set_target_properties(MatterRLCalibrationTests
            MatterRLJointLimitLabTests MatterRLDynamicsTests
            MatterRLTrainingTests PROPERTIES
            FOLDER "Tests")
    endif()
endif()

if(MSVC)
    target_compile_options(MatterEngine PRIVATE /W4 /permissive- /utf-8)
    target_compile_options(MatterAthlete PRIVATE /W4 /permissive- /utf-8)
    target_compile_options(MatterVulkan PRIVATE /W4 /permissive- /utf-8)
    target_compile_options(MatterCore PRIVATE /W4 /permissive- /utf-8)
    target_compile_options(MatterModeling PRIVATE /W4 /permissive- /utf-8)
    target_compile_options(MatterModelingRig PRIVATE /W4 /permissive- /utf-8)
    target_compile_options(MatterWorkbench PRIVATE /W4 /permissive- /utf-8)
    target_compile_options(MatterEngineApp PRIVATE /W4 /permissive- /utf-8)
    target_compile_options(MatterVulkanSmoke PRIVATE /W4 /permissive- /utf-8)
    target_compile_options(MatterPhysicsBenchmark PRIVATE /W4 /permissive- /utf-8)
    if(MATTERENGINE_BUILD_TESTS)
        target_compile_options(MatterEngineTests PRIVATE /W4 /permissive- /utf-8)
        target_compile_options(MatterAudioTests PRIVATE /W4 /permissive- /utf-8)
    endif()
else()
    target_compile_options(MatterEngine PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(MatterAthlete PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(MatterVulkan PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(MatterCore PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(MatterModeling PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(MatterModelingRig PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(MatterWorkbench PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(MatterEngineApp PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(MatterVulkanSmoke PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(MatterPhysicsBenchmark PRIVATE -Wall -Wextra -Wpedantic)
    if(MATTERENGINE_BUILD_TESTS)
        target_compile_options(MatterEngineTests PRIVATE -Wall -Wextra -Wpedantic)
        target_compile_options(MatterAudioTests PRIVATE -Wall -Wextra -Wpedantic)
    endif()
endif()
