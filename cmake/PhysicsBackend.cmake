# Selecao do backend de fisica.
#
# A interface neutra (src/Engine/Physics/*.hpp) e identica para os dois
# backends. Este arquivo decide apenas qual implementacao e buscada, compilada
# e linkada, e publica tres variaveis para o target MatterPhysics:
#
#   MATTERENGINE_PHYSICS_BACKEND_SOURCES      fontes do adaptador escolhido
#   MATTERENGINE_PHYSICS_BACKEND_LIBRARIES    bibliotecas privadas do backend
#   MATTERENGINE_PHYSICS_BACKEND_DEFINITIONS  defines privados do backend
#
# Durante a migracao PhysX -> Jolt os dois coexistem de proposito: a mesma
# suite de testes roda contra ambos (ela passa pela interface neutra, nao pelo
# SDK), entao qualquer divergencia de comportamento e medida em vez de
# suposta. O PhysX sai da arvore quando a paridade estiver provada.
#
# Incluido depois do target `vhacd`, porque o cooking de colisao dinamica
# depende dele nos dois backends.

# Jolt e o backend padrao desde que a suite passou nele em Debug e em
# RelWithDebInfo. PhysX continua selecionavel apenas como referencia de
# comportamento ate a aprovacao visual do usuario; depois sai da arvore (a copia
# de estudo fica em archive/2026-09-25-physx-backend/).
set(MATTERENGINE_PHYSICS_BACKEND "Jolt" CACHE STRING
    "Backend de fisica: Jolt (atual) ou PhysX (referencia, em remocao)")
set_property(CACHE MATTERENGINE_PHYSICS_BACKEND PROPERTY STRINGS Jolt PhysX)

if(NOT MATTERENGINE_PHYSICS_BACKEND MATCHES "^(Jolt|PhysX)$")
    message(FATAL_ERROR
        "MATTERENGINE_PHYSICS_BACKEND invalido: '${MATTERENGINE_PHYSICS_BACKEND}'"
        " (esperado Jolt ou PhysX)")
endif()

message(STATUS "MatterEngine: backend de fisica = ${MATTERENGINE_PHYSICS_BACKEND}")

if(MATTERENGINE_PHYSICS_BACKEND STREQUAL "Jolt")

    # Jolt e um detalhe privado do modulo de fisica, como o PhysX era. A tag da
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

    # Uma responsabilidade por arquivo, teto de ~450 linhas. O backend PhysX
    # concentrava 3.532 linhas num unico .cpp, e isso foi uma das razoes
    # declaradas para a reescrita em vez de traducao mecanica.
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

else()

    # PhysX e um detalhe privado do modulo de fisica. Fixamos o commit da
    # versao oficial 5.9.0 para que uma atualizacao remota nunca altere a
    # simulacao de uma build existente. O backend CPU estatico e a configuracao
    # portavel para o editor; GPU rigid bodies poderao ser habilitados
    # futuramente como outro perfil, depois de medirmos uma cena que realmente
    # amortize o custo do CUDA.
    #
    # LEGADO EM REMOCAO: este caminho existe apenas para comparar comportamento
    # com o backend Jolt durante a migracao. Uma copia completa, com os headers
    # neutros da epoca, esta em archive/2026-09-25-physx-backend/.
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
    FetchContent_MakeAvailable(PhysX)

    set(MATTERENGINE_PHYSX_THIRD_PARTY_TARGETS
        PhysXFoundation PhysXCommon PhysX PhysXExtensions PhysXCooking
        PhysXCharacterKinematic PhysXPvdSDK PhysXTask PhysXVehicle
        LowLevel LowLevelAABB LowLevelDynamics SceneQuery
        SimulationController)

    if(MSVC)
        # MSVC 14.43 passou a emitir C5054/C5055 para operacoes entre enums que
        # o codigo oficial 5.9.0 ainda usa e o PhysX compila seus fontes com
        # /WX. Suprimimos apenas esses dois diagnosticos nos targets de
        # terceiros; os avisos da MatterEngine continuam integralmente ativos.
        foreach(physx_target ${MATTERENGINE_PHYSX_THIRD_PARTY_TARGETS})
            if(TARGET ${physx_target})
                target_compile_options(${physx_target} PRIVATE /wd5054 /wd5055)
            endif()
        endforeach()
    elseif(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
        # O PhysX 5.9 ainda incrementa um contador volatile no backend Unix.
        # GCC 16 diagnostica isso como -Wvolatile e o SDK promove todos os
        # avisos a erro. A supressao fica restrita aos targets de terceiros.
        foreach(physx_target ${MATTERENGINE_PHYSX_THIRD_PARTY_TARGETS})
            if(TARGET ${physx_target})
                target_compile_options(${physx_target} PRIVATE
                    -Wno-error
                    -Wno-volatile)
            endif()
        endforeach()
    endif()

    set(MATTERENGINE_PHYSICS_BACKEND_SOURCES
        src/Engine/Physics/PhysX/PhysXEngine3D.cpp
        src/Engine/Physics/PhysX/PhysXCooking3D.cpp
        src/Engine/Physics/PhysX/PhysXScene3D.cpp
    )
    set(MATTERENGINE_PHYSICS_BACKEND_LIBRARIES physx_lib)
    set(MATTERENGINE_PHYSICS_BACKEND_DEFINITIONS
        PX_PHYSX_STATIC_LIB
        MATTERENGINE_PHYSICS_BACKEND_PHYSX=1)

endif()
