# Backend de fisica: Jolt.
#
# A interface neutra (src/Engine/Physics/*.hpp) esconde o SDK. Este arquivo
# busca, configura e compila o Jolt e publica tres variaveis para o target
# MatterPhysics:
#
#   MATTERENGINE_PHYSICS_BACKEND_SOURCES      fontes do adaptador
#   MATTERENGINE_PHYSICS_BACKEND_LIBRARIES    bibliotecas privadas do backend
#   MATTERENGINE_PHYSICS_BACKEND_DEFINITIONS  defines privados do backend
#
# O backend PhysX foi removido em 02/10/2026, depois da migracao para o Jolt.
# Copias de estudo: archive/2026-09-25-physx-backend/ (versao da migracao) e
# archive/2026-10-02-physx-backend-final/ (ultima versao antes da remocao).
#
# Incluido depois do target `vhacd`, porque o cooking de colisao dinamica
# depende dele.

message(STATUS "MatterEngine: backend de fisica = Jolt")

# Jolt e um detalhe privado do modulo de fisica. A tag da
# release e fixada pelo commit para que uma atualizacao remota nunca altere
# a simulacao de uma build existente.
#
# As opcoes abaixo nao sao cosmeticas: varios padroes do Jolt brigam com
# este projeto e precisam ser explicitados.
set(OVERRIDE_CXX_FLAGS OFF CACHE BOOL "" FORCE)      # o padrao ON sobrescreve CMAKE_CXX_FLAGS_DEBUG/RELEASE globais
set(ENABLE_ALL_WARNINGS OFF CACHE BOOL "" FORCE)     # o padrao ON e warnings-as-errors em codigo de terceiro
# Backends de compute do SDK (usados por cabelo/soft body em GPU). Ligados
# por padrao, puxam dependencia de compilador de shader (DXC) que esta
# integracao de corpos rigidos em CPU nao usa.
set(JPH_USE_VK OFF CACHE BOOL "" FORCE)
set(JPH_USE_DX12 OFF CACHE BOOL "" FORCE)
set(JPH_USE_MTL OFF CACHE BOOL "" FORCE)
set(JPH_USE_CPU_COMPUTE OFF CACHE BOOL "" FORCE)
set(ENABLE_OBJECT_STREAM OFF CACHE BOOL "" FORCE)    # serializacao por RTTI que nao usamos
set(ENABLE_INSTALL OFF CACHE BOOL "" FORCE)          # nada e instalado
set(DOUBLE_PRECISION OFF CACHE BOOL "" FORCE)        # mundo de ~3 km: float basta
set(CROSS_PLATFORM_DETERMINISTIC OFF CACHE BOOL "" FORCE) # custa performance e so paga em lockstep de rede
set(INTERPROCEDURAL_OPTIMIZATION OFF CACHE BOOL "" FORCE) # LTO fica a cargo do perfil de build do projeto
set(FLOATING_POINT_EXCEPTIONS_ENABLED OFF CACHE BOOL "" FORCE)
# O Jolt compila sem excecoes por padrao. O projeto lanca por convencao, e
# uma excecao que atravesse um frame compilado com -fno-exceptions e
# comportamento indefinido. Ligar isto so adiciona tabelas de unwind ao SDK
# (ele nao passa a usar excecoes) e torna a propagacao bem definida.
# Os callbacks que o Jolt chama de volta continuam escritos para nao lancar.
set(CPP_EXCEPTIONS_ENABLED ON CACHE BOOL "" FORCE)
# RTTI fica desligado, como o SDK prefere. Consequencia que importa: nao
# existe typeinfo para as classes do Jolt, portanto `dynamic_cast` sobre
# tipos dele nao linka. O backend identifica material de superficie pelo
# nome declarado na propria shape, nao por RTTI.
set(CPP_RTTI_ENABLED OFF CACHE BOOL "" FORCE)
# Asserts do SDK ligados em Debug: e uma integracao nova, e o assert do
# Jolt pega uso indevido de API (corpo nao travado, camada invalida) que de
# outro modo vira corrupcao silenciosa. JPH_ENABLE_ASSERTS e PUBLIC no
# target, portanto chega igual as TUs do adaptador.
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    set(USE_ASSERTS ON CACHE BOOL "" FORCE)
else()
    set(USE_ASSERTS OFF CACHE BOOL "" FORCE)
endif()
# SSE4.2/AVX/AVX2/F16C/FMADD ficam nos padroes ligados do Jolt: o alvo de
# desenvolvimento (Ryzen 5 3600) suporta todos. Os flags e os defines
# correspondentes sao PUBLIC no target Jolt, entao linkar o target garante
# que a lib e as TUs do adaptador compartilhem exatamente o mesmo layout de
# Vec3/Mat44 - a divergencia silenciosa mais perigosa dessa integracao.
# Revisar quando houver distribuicao para maquinas pre-2013.

FetchContent_Declare(
    JoltPhysics
    GIT_REPOSITORY https://github.com/jrouwe/JoltPhysics.git
    # Release oficial v5.6.0
    GIT_TAG e77f175595e64cb44218cc9d9d56fc365ad0e36a
    SOURCE_SUBDIR Build
)
FetchContent_MakeAvailable(JoltPhysics)

# Uma responsabilidade por arquivo, teto de ~450 linhas.
set(MATTERENGINE_PHYSICS_BACKEND_SOURCES
    src/Engine/Physics/Jolt/JoltLayers.cpp
    src/Engine/Physics/Jolt/JoltJobSystem.cpp
    src/Engine/Physics/Jolt/JoltEngine3D.cpp
    src/Engine/Physics/Jolt/JoltShapes3D.cpp
    src/Engine/Physics/Jolt/JoltCooking3D.cpp
    src/Engine/Physics/Jolt/JoltScene3D.cpp
    src/Engine/Physics/Jolt/JoltBodies3D.cpp
    src/Engine/Physics/Jolt/JoltQueries3D.cpp
    src/Engine/Physics/Jolt/JoltContacts3D.cpp
    src/Engine/Physics/Jolt/JoltExternalForces3D.cpp
    src/Engine/Physics/Jolt/JoltCharacter3D.cpp
    src/Engine/Physics/Jolt/JoltGrab3D.cpp
    src/Engine/Physics/Jolt/JoltRagdoll3D.cpp
    src/Engine/Physics/Jolt/JoltRagdollDrives3D.cpp
    src/Engine/Physics/Jolt/JoltRagdollState3D.cpp
)
# Derived SDK classes must also compile without RTTI: otherwise their
# vtables refer to typeinfo that the Jolt library deliberately omits.
if(MSVC)
    set_source_files_properties(${MATTERENGINE_PHYSICS_BACKEND_SOURCES}
        PROPERTIES COMPILE_OPTIONS "/GR-")
else()
    set_source_files_properties(${MATTERENGINE_PHYSICS_BACKEND_SOURCES}
        PROPERTIES COMPILE_OPTIONS "-fno-rtti")
endif()
set(MATTERENGINE_PHYSICS_BACKEND_LIBRARIES Jolt)
set(MATTERENGINE_PHYSICS_BACKEND_DEFINITIONS
    MATTERENGINE_PHYSICS_BACKEND_JOLT=1)
