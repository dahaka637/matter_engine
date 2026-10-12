#pragma once

#include "Engine/Physics/PhysicsScene3D.hpp"
#include "Engine/RL/Calibration/RagdollCalibrationAudit3D.hpp"
#include "Engine/RL/RLConstants3D.hpp"

#include <cstddef>
#include <string>
#include <vector>

namespace MatterEngine {

struct RagdollDynamicLabSettings3D {
    float freeFallDurationSeconds = 0.50f;
    float supportedDurationSeconds = 4.0f;
    float dropDurationSeconds = 4.0f;
    float supportedGroundClearanceMeters = 0.015f;
    float dropHeightMeters = 0.35f;
    float supportedRigidityPercent = 100.0f;
    // Zero desliga a captura. Quando ligada, o laboratorio guarda o estado
    // inicial, cada N passos, o primeiro contato e o estado final. A captura e
    // somente diagnostica: nao muda a frequencia nem escreve na simulacao.
    std::size_t captureEverySteps = 0;
};

struct RagdollDynamicFrame3D {
    float timeSeconds = 0.0f;
    std::vector<PhysicsBodyState3D> links;
    std::vector<RagdollContactPoint3D> contacts;
};

struct RagdollDynamicTrial3D {
    std::string id;
    std::string description;
    std::size_t stepCount = 0;
    float durationSeconds = 0.0f;
    Vec3 initialCenterOfMassWorld;
    Vec3 finalCenterOfMassWorld;
    Vec3 initialCenterOfMassVelocity;
    Vec3 finalCenterOfMassVelocity;
    float measuredVerticalAccelerationMetersPerSecondSquared = 0.0f;
    float horizontalCenterOfMassDriftMeters = 0.0f;
    float pelvisHeightLossMeters = 0.0f;
    float maximumLinkSpeedMetersPerSecond = 0.0f;
    float maximumLinkAngularSpeedRadiansPerSecond = 0.0f;
    float maximumJointAnchorSeparationMeters = 0.0f;
    float maximumBilateralPositionErrorMeters = 0.0f;
    float maximumContactImpulseNewtonSeconds = 0.0f;
    std::size_t contactStepCount = 0;
    std::size_t maximumContactPointCount = 0;
    float firstContactSeconds = -1.0f;
    float averagePhysicsStepMilliseconds = 0.0f;
    float maximumPhysicsStepMilliseconds = 0.0f;
    bool finalSleeping = false;
    bool finite = true;
    std::vector<RagdollDynamicFrame3D> capturedFrames;
};

struct RagdollDynamicLabResult3D {
    std::string profileId;
    float fixedStepSeconds = ReinforcementLearningFixedStepSeconds;
    RagdollEffectiveMass3D effectiveMassProperties;
    std::vector<RagdollDynamicTrial3D> trials;
    std::vector<CalibrationFinding3D> findings;

    [[nodiscard]] std::size_t findingCount(CalibrationFindingSeverity3D severity) const;
    [[nodiscard]] bool hasErrors() const;
};

// Cria uma cena headless sem avancar o tempo e devolve exatamente as
// propriedades de massa que o backend aplicou. Util para auditorias rapidas
// que nao precisam executar os tres ensaios dinamicos completos.
[[nodiscard]] RagdollEffectiveMass3D measureRagdollEffectiveMass3D(const RagdollProfile3D& profile);

// Executa cenas independentes e deterministicas, sem renderer: queda livre,
// sustentacao muscular passiva e queda sobre piso plano. Nenhum ensaio escreve
// transforms do ragdoll nem introduz ajuda externa.
[[nodiscard]] RagdollDynamicLabResult3D runRagdollDynamicLab3D(
    const RagdollProfile3D& profile, const RagdollDynamicLabSettings3D& settings = {});

[[nodiscard]] std::string renderRagdollDynamicLabMarkdown3D(const RagdollDynamicLabResult3D& result);

// Serializa as capturas no formato de poses consumido por
// `MatterModelInspector quadros`. Retorna vazio quando a captura estava
// desligada. Os registros P incluem ponto, link, normal e impulso estimado.
[[nodiscard]] std::string renderRagdollDynamicLabPoseDump3D(
    const RagdollDynamicLabResult3D& result, const RagdollProfile3D& profile);

} // namespace MatterEngine
