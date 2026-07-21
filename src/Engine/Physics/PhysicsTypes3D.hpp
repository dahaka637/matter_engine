#pragma once

#include "Engine/Math/Quaternion.hpp"

#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

namespace MatterEngine {

// Handles geracionais impedem que uma entidade removida passe a controlar um
// ator novo que reutilizou o mesmo slot. Nenhum ponteiro do PhysX atravessa a
// API publica da engine.
struct PhysicsBodyHandle3D {
    std::uint32_t index = std::numeric_limits<std::uint32_t>::max();
    std::uint32_t generation = 0;

    [[nodiscard]] constexpr bool valid() const {
        return index != std::numeric_limits<std::uint32_t>::max();
    }
    constexpr explicit operator bool() const { return valid(); }
    constexpr bool operator==(const PhysicsBodyHandle3D&) const = default;
};

enum class PhysicsMotionType3D : std::uint8_t {
    Static,
    Dynamic,
    Kinematic
};

enum class PhysicsCollisionMode3D : std::uint8_t {
    Discrete,
    Continuous
};

enum class AcousticBodyStructure3D : std::uint8_t {
    Solid,
    Hollow,
    ThinShell,
    Soft,
    Inflated
};

// Propriedades que pertencem ao corpo, e nao a uma forma de colisao. Materiais
// de contato continuam registrados por shape; estes dados alimentam massa,
// amortecimento, audio e limites numericos do ator dinamico.
struct PhysicsBodyDefinition3D {
    std::uint64_t entityId = 0;
    PhysicsMotionType3D motionType = PhysicsMotionType3D::Dynamic;
    Vec3 position;
    Quaternion orientation;
    Vec3 linearVelocity;
    Vec3 angularVelocity;
    float massKg = 1.0f;
    float linearDamping = 0.035f;
    float angularDamping = 0.08f;
    bool aerodynamicDragEnabled = true;
    float aerodynamicDragCoefficient = 1.0f;
    float aerodynamicReferenceAreaSquareMeters = 0.0f;
    // CCD completo e reservado a corpos pequenos/rapidos. Tornar todo prop
    // continuo multiplica o custo da deteccao sem elevar a precisao
    // perceptivel de objetos comuns simulados a 120 Hz.
    PhysicsCollisionMode3D collisionMode = PhysicsCollisionMode3D::Discrete;
    bool allowSleeping = true;
    bool startAwake = true;
    std::uint32_t collisionLayer = 1u;
    std::uint32_t collisionMask = 0xFFFFFFFFu;

    // Metadados acusticos ficam junto do ator para que o callback de contato
    // produza eventos objetivos sem consultar objetos da camada de aplicacao.
    std::string materialId = "default";
    float characteristicSizeMeters = 1.0f;
    float acousticGain = 1.0f;
    float acousticDamping = 1.0f;
    AcousticBodyStructure3D acousticStructure =
        AcousticBodyStructure3D::Solid;
};

struct PhysicsBodyState3D {
    Vec3 position;
    Quaternion orientation;
    Vec3 linearVelocity;
    Vec3 angularVelocity;
    bool sleeping = false;
    bool frozen = false;
};

struct PhysicsRayHit3D {
    PhysicsBodyHandle3D body;
    Vec3 position;
    Vec3 normal { 0.0f, 0.0f, 1.0f };
    float distance = 0.0f;
    std::string materialId = "default";
};

// Snapshot barato das estatisticas publicadas pelo PhysX. Ele serve para o
// profiler e para testes de escala; nunca participa das decisoes da simulacao.
struct PhysicsStepDiagnostics3D {
    std::size_t staticBodyCount = 0;
    std::size_t dynamicBodyCount = 0;
    std::size_t activeDynamicBodyCount = 0;
    std::size_t sleepingDynamicBodyCount = 0;
    std::size_t broadPhaseAdds = 0;
    std::size_t broadPhaseRemoves = 0;
    std::size_t discreteContactPairs = 0;
    std::size_t ccdPairs = 0;
    std::uint32_t physicsWorkerCount = 1;
    float simulationDispatchMilliseconds = 0.0f;
    float simulationWaitMilliseconds = 0.0f;
    float totalStepMilliseconds = 0.0f;
};

// Estado somente dos atores que mudaram no ultimo passo. Consumidores de
// renderizacao podem manter um snapshot e atualizar O(ativos), em vez de
// consultar milhares de corpos adormecidos a cada frame grafico.
struct PhysicsBodyStateUpdate3D {
    PhysicsBodyHandle3D body;
    PhysicsBodyState3D state;
};

constexpr std::size_t InvalidPhysicsBodyIndex =
    static_cast<std::size_t>(-1);

// Impacto medido no callback do solver. A camada acustica decide se a energia
// e relevante e como transforma estas grandezas em som; a fisica nao toca
// arquivos de audio nem aplica limiares de apresentacao.
struct ContactImpactEvent3D {
    std::size_t bodyA = InvalidPhysicsBodyIndex;
    std::size_t bodyB = InvalidPhysicsBodyIndex;
    std::uint64_t bodyIdA = 0;
    std::uint64_t bodyIdB = 0;
    std::string materialA = "default";
    std::string materialB = "default";
    Vec3 position;
    Vec3 normal { 0.0f, 0.0f, 1.0f };
    float normalImpulseNewtonSeconds = 0.0f;
    float tangentialImpulseNewtonSeconds = 0.0f;
    float approachSpeedMetersPerSecond = 0.0f;
    float effectiveMassKg = 0.0f;
    float transferredEnergyJoules = 0.0f;
    float massA = 0.0f;
    float massB = 0.0f;
    float characteristicSizeA = 1.0f;
    float characteristicSizeB = 1.0f;
    float acousticGainA = 1.0f;
    float acousticGainB = 1.0f;
    float acousticDampingA = 1.0f;
    float acousticDampingB = 1.0f;
    AcousticBodyStructure3D structureA = AcousticBodyStructure3D::Solid;
    AcousticBodyStructure3D structureB = AcousticBodyStructure3D::Solid;
    bool staticA = true;
    bool staticB = true;
};

} // namespace MatterEngine
