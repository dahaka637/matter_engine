cmake_minimum_required(VERSION 3.20)

project(MatterEngine VERSION 0.1.0 LANGUAGES CXX C)

if(WIN32)
    enable_language(RC)
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set_property(GLOBAL PROPERTY USE_FOLDERS ON)

include(FetchContent)

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

# PhysX e um detalhe privado do modulo de fisica. Fixamos o commit da versao
# oficial 5.9.0 para que uma atualizacao remota nunca altere a simulacao de uma
# build existente. O backend CPU estatico e a configuracao portavel para o
# editor; GPU rigid bodies poderao ser habilitados futuramente como outro
# perfil, depois de medirmos uma cena que realmente amortize o custo do CUDA.
set(PHYSX_PRESET "" CACHE STRING "" FORCE)
set(PX_GENERATE_GPU_PROJECTS OFF CACHE BOOL "" FORCE)
set(PX_GENERATE_STATIC_LIBRARIES ON CACHE BOOL "" FORCE)
set(PX_GENERATE_GPU_STATIC_LIBRARIES OFF CACHE BOOL "" FORCE)
set(PX_BUILDSNIPPETS OFF CACHE BOOL "" FORCE)
set(PX_BUILDPVDRUNTIME OFF CACHE BOOL "" FORCE)
set(NV_USE_STATIC_WINCRT OFF CACHE BOOL "" FORCE)
FetchContent_Declare(
    PhysX
    GIT_REPOSITORY https://github.com/NVIDIA-Omniverse/PhysX.git
    # Tag oficial: 110.1-omni-and-physx-5.9.0
    GIT_TAG 517a0073715120e114ee055b63b26c95e00d9039
    SOURCE_SUBDIR physx
)

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
# Sem isso o CRT do OpenAL Soft ficaria em /MT enquanto o resto do projeto (e
# o PhysX, via NV_USE_STATIC_WINCRT OFF acima) usa /MD - o mesmo tipo de
# divergencia de runtime que precisou ser resolvido explicitamente pro PhysX.
set(FORCE_STATIC_VCRT OFF CACHE BOOL "" FORCE)
# Modulos C++20 sao um caminho de build experimental e recente na lib; nada
# aqui precisa deles (consumimos so os headers C AL/ALC/EFX), entao
# desligamos para nao herdar um eixo de risco de build que nao usamos.
set(ALSOFT_ENABLE_MODULES OFF CACHE BOOL "" FORCE)
# So o WASAPI fica ligado: e o backend nativo moderno do Windows, os outros
# (WinMM/DirectSound) sao apenas fallback legado que nao precisamos exercitar.
set(ALSOFT_BACKEND_WASAPI ON CACHE BOOL "" FORCE)
set(ALSOFT_REQUIRE_WASAPI ON CACHE BOOL "" FORCE)
set(ALSOFT_BACKEND_WINMM OFF CACHE BOOL "" FORCE)
set(ALSOFT_BACKEND_DSOUND OFF CACHE BOOL "" FORCE)
FetchContent_Declare(
    openal-soft
    GIT_REPOSITORY https://github.com/kcat/openal-soft.git
    GIT_TAG 7f9c6de1162b1d4604b9a2f8bcd6c53021566221
)

FetchContent_MakeAvailable(
    SDL3 imgui sqlite_amalgamation cgltf stb PhysX vhacd_source openal-soft)

add_library(vhacd INTERFACE)
target_include_directories(vhacd INTERFACE ${vhacd_source_SOURCE_DIR}/include)

if(MSVC)
    # MSVC 14.43 passou a emitir C5054/C5055 para operacoes entre enums que o
    # codigo oficial 5.9.0 ainda usa e o PhysX compila seus fontes com /WX.
    # Suprimimos apenas esses dois diagnosticos nos targets de terceiros; os
    # avisos da MatterEngine continuam integralmente ativos.
    foreach(physx_target
            PhysXFoundation PhysXCommon PhysX PhysXExtensions PhysXCooking
            PhysXCharacterKinematic PhysXPvdSDK PhysXTask PhysXVehicle
            LowLevel LowLevelAABB LowLevelDynamics SceneQuery
            SimulationController)
        if(TARGET ${physx_target})
            target_compile_options(${physx_target} PRIVATE /wd5054 /wd5055)
        endif()
    endforeach()
endif()

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
        tonemap.vert tonemap.frag
        taa_resolve.frag)
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

# MaterialLibrary, Vec3 e Quaternion vivem aqui (a base da hierarquia) porque
# tanto MatterPhysics quanto MatterAudio dependem deles (fisica precisa de
# atrito/densidade/orientacao, acustica precisa da resposta a impacto do
# mesmo material e da amostragem de oclusao em Vec3) e nenhum dos dois pode
# depender do outro. Vec2/Mat4/Frustum3D continuam em MatterEngine: nada
# abaixo dela usa camera, projecao ou culling.
add_library(MatterCore STATIC
    src/Engine/Core/Log.cpp
    src/Engine/Core/TaskScheduler.cpp
    src/Engine/Materials/MaterialLibrary.cpp
    src/Engine/Math/Hash.cpp
    src/Engine/Math/Vec3.cpp
    src/Engine/Math/Quaternion.cpp
)
target_include_directories(MatterCore PUBLIC src)

add_library(MatterRHI INTERFACE)
target_include_directories(MatterRHI INTERFACE src)

# API publica neutra + backend PhysX privado. Nenhum consumidor recebe os
# includes ou tipos nativos do SDK; trocar/configurar o backend nao contamina a
# camada Workbench nem o restante da engine.
add_library(MatterPhysics STATIC
    src/Engine/Physics/PhysicalBodyBuilder3D.cpp
    src/Engine/Physics/PhysX/PhysXEngine3D.cpp
    src/Engine/Physics/PhysX/PhysXCooking3D.cpp
    src/Engine/Physics/PhysX/PhysXScene3D.cpp
)
target_include_directories(MatterPhysics PUBLIC src)
target_link_libraries(MatterPhysics
    PUBLIC MatterCore
    PRIVATE physx_lib vhacd
)
target_compile_definitions(MatterPhysics PRIVATE PX_PHYSX_STATIC_LIB)
if(MSVC)
    # Dependencias de terceiros sao compiladas com seus proprios perfis. Estas
    # opcoes mantem diagnosticos fortes no adaptador escrito pela MatterEngine.
    target_compile_options(MatterPhysics PRIVATE /W4 /permissive- /utf-8)
else()
    target_compile_options(MatterPhysics PRIVATE -Wall -Wextra -Wpedantic)
endif()

add_library(MatterVulkan STATIC
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

# Modulo de audio isolado: nenhum consumidor precisa enxergar o backend de
# mixagem (OpenAL Soft) para depender de deteccao/resolucao acustica de
# impactos. Mantem a mesma logica de fronteira de MatterPhysics (API neutra
# publica, biblioteca de terceiros privada) - nenhum header AL/ALC/EFX
# atravessa AudioDevice3D.hpp.
add_library(MatterAudio STATIC
    src/Engine/Audio/AcousticEnvironmentSystem3D.cpp
    src/Engine/Audio/AcousticOcclusion3D.cpp
    src/Engine/Audio/AudioDevice3D.cpp
    src/Engine/Audio/BiquadFilter.cpp
    src/Engine/Audio/Envelope.cpp
    src/Engine/Audio/ImpactAcoustics.cpp
    src/Engine/Audio/ProceduralNoise.cpp
    src/Engine/Audio/Waveform.cpp
    src/Engine/Audio/WindSound.cpp
)
target_include_directories(MatterAudio PUBLIC src)
target_link_libraries(MatterAudio
    PUBLIC MatterCore MatterPhysics
    PRIVATE SDL3::SDL3-static OpenAL
)
if(MSVC)
    target_compile_options(MatterAudio PRIVATE /W4 /permissive- /utf-8)
else()
    target_compile_options(MatterAudio PRIVATE -Wall -Wextra -Wpedantic)
endif()

add_library(MatterEngine STATIC
    src/Engine/Core/Application.cpp
    src/Engine/Input/Input.cpp
    src/Engine/Platform/SDL3/SDLPlatform.cpp
    src/Engine/Platform/SDL3/SDLInputAdapter.cpp
    src/Engine/Math/Vec2.cpp
    src/Engine/Math/Mat4.cpp
    src/Engine/Math/Frustum3D.cpp
    src/Engine/Math/JitterSequence.cpp
    src/Engine/Math/ShadowCascade.cpp
    src/Engine/Environment/WindSystem.cpp
    src/Engine/Geometry/MeshData3D.cpp
    src/Engine/Geometry/GltfLoader.cpp
    src/Engine/Geometry/GltfPhysicsMetadata3D.cpp
    src/Engine/Geometry/GltfAcousticZone3D.cpp
    src/Engine/Geometry/PhysicalAsset3D.cpp
    src/Engine/Render/Render2D.cpp
    src/Engine/Render/Renderer.cpp
    src/Engine/Render/SceneLightPacking.cpp
    src/Engine/UI/ImGuiFontLibrary.cpp
    src/Engine/Data/Database.cpp
    src/Engine/Data/SettingsRepository.cpp
)

target_include_directories(MatterEngine PUBLIC src)
target_link_libraries(MatterEngine
    PUBLIC MatterCore MatterRHI MatterPhysics MatterAudio
    PRIVATE MatterVulkan SDL3::SDL3-static imgui sqlite3 cgltf stb_image
)
add_dependencies(MatterEngine MatterEngineShaders)
target_compile_definitions(MatterEngine PRIVATE
    MATTERENGINE_SHADER_DIR="${MATTERENGINE_SHADER_OUTPUT_DIR}"
    MATTERENGINE_ASSETS_DIR="${CMAKE_SOURCE_DIR}/assets"
)

add_library(MatterWorkbench STATIC
    src/Workbench/WorkbenchApp.cpp
    src/Workbench/UiKit.cpp
    src/Workbench/Audio/WorldAudioController.cpp
    src/Workbench/Laboratory/PropRuntime.cpp
    src/Workbench/Props/PropCatalog.cpp
    src/Workbench/Screens/SettingsScreen.cpp
    src/Workbench/Screens/MainMenuScreen.cpp
    src/Workbench/Screens/LaboratoryScreen.cpp
    src/Workbench/Screens/ObjectViewerScreen.cpp
)

target_include_directories(MatterWorkbench PUBLIC src)
target_link_libraries(MatterWorkbench
    PUBLIC MatterEngine
    PRIVATE imgui
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
    target_link_libraries(MatterEngineTests PRIVATE MatterEngine)
    target_compile_definitions(MatterEngineTests PRIVATE
        MATTERENGINE_TEST_ASSETS_DIR="${CMAKE_SOURCE_DIR}/assets")
    add_test(NAME MatterEngine.Foundation COMMAND MatterEngineTests)

    # Linkado somente contra MatterAudio (nunca MatterEngine/Vulkan/imgui) para
    # provar que o modulo de audio nao vaza dependencia de renderizacao.
    add_executable(MatterAudioTests tests/AudioFoundationTests.cpp)
    target_link_libraries(MatterAudioTests PRIVATE MatterAudio)
    target_compile_definitions(MatterAudioTests PRIVATE
        MATTERENGINE_TEST_ASSETS_DIR="${CMAKE_SOURCE_DIR}/assets")
    add_test(NAME MatterEngine.Audio COMMAND MatterAudioTests)
endif()

# Smoke test manual: abre uma janela real e percorre resize, minimizar/restaurar
# e modos fullscreen. Fica fora do build padrão para não interromper CI/headless.
add_executable(MatterVulkanSmoke EXCLUDE_FROM_ALL tools/VulkanSmoke.cpp)
target_link_libraries(MatterVulkanSmoke PRIVATE MatterEngine SDL3::SDL3-static)

# Ferramenta offline: gera caches V-HACD versionados ao lado dos GLBs. Ela nao
# faz parte do build padrao nem do runtime e evita cooking pesado na abertura
# do Workbench.
add_executable(MatterPhysicsAssetCooker EXCLUDE_FROM_ALL
    tools/PhysicsAssetCooker.cpp)
target_link_libraries(MatterPhysicsAssetCooker PRIVATE MatterEngine)
target_compile_definitions(MatterPhysicsAssetCooker PRIVATE
    MATTERENGINE_ASSETS_DIR="${CMAKE_SOURCE_DIR}/assets")

# Benchmark manual e reproduzivel da carga-alvo atual. Usa os colliders reais
# dos seis props e publica percentis; nao entra no CTest porque desempenho e
# dependente do hardware e da configuracao da build.
add_executable(MatterPhysicsBenchmark EXCLUDE_FROM_ALL
    tools/PhysicsBenchmark.cpp)
target_link_libraries(MatterPhysicsBenchmark PRIVATE MatterEngine)
target_compile_definitions(MatterPhysicsBenchmark PRIVATE
    MATTERENGINE_ASSETS_DIR="${CMAKE_SOURCE_DIR}/assets")

set_target_properties(MatterCore MatterRHI MatterPhysics MatterAudio MatterVulkan
    MatterEngine MatterEngineShaders PROPERTIES FOLDER "MatterEngine")
set_target_properties(MatterWorkbench MatterEngineApp
    PROPERTIES FOLDER "MatterEngine Workbench")
set_target_properties(MatterVulkanSmoke PROPERTIES FOLDER "Tests")
set_target_properties(MatterPhysicsAssetCooker PROPERTIES FOLDER "Tools")
set_target_properties(MatterPhysicsBenchmark PROPERTIES FOLDER "Tests")
if(MATTERENGINE_BUILD_TESTS)
    set_target_properties(MatterEngineTests MatterAudioTests
        PROPERTIES FOLDER "Tests")
endif()

if(MSVC)
    target_compile_options(MatterEngine PRIVATE /W4 /permissive- /utf-8)
    target_compile_options(MatterVulkan PRIVATE /W4 /permissive- /utf-8)
    target_compile_options(MatterCore PRIVATE /W4 /permissive- /utf-8)
    target_compile_options(MatterWorkbench PRIVATE /W4 /permissive- /utf-8)
    target_compile_options(MatterEngineApp PRIVATE /W4 /permissive- /utf-8)
    target_compile_options(MatterVulkanSmoke PRIVATE /W4 /permissive- /utf-8)
    target_compile_options(MatterPhysicsAssetCooker PRIVATE /W4 /permissive- /utf-8)
    target_compile_options(MatterPhysicsBenchmark PRIVATE /W4 /permissive- /utf-8)
    if(MATTERENGINE_BUILD_TESTS)
        target_compile_options(MatterEngineTests PRIVATE /W4 /permissive- /utf-8)
        target_compile_options(MatterAudioTests PRIVATE /W4 /permissive- /utf-8)
    endif()
else()
    target_compile_options(MatterEngine PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(MatterVulkan PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(MatterCore PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(MatterWorkbench PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(MatterEngineApp PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(MatterVulkanSmoke PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(MatterPhysicsAssetCooker PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(MatterPhysicsBenchmark PRIVATE -Wall -Wextra -Wpedantic)
    if(MATTERENGINE_BUILD_TESTS)
        target_compile_options(MatterEngineTests PRIVATE -Wall -Wextra -Wpedantic)
        target_compile_options(MatterAudioTests PRIVATE -Wall -Wextra -Wpedantic)
    endif()
endif()
