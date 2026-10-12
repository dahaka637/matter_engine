#pragma once

#include "Engine/Physics/PhysicsTypes3D.hpp"
#include "Engine/RL/Calibration/RagdollCalibrationAudit3D.hpp"
#include "Engine/Physics/RagdollProfile3D.hpp"
#include "Engine/RL/RLConstants3D.hpp"

#include <cstddef>
#include <string>
#include <vector>

namespace MatterEngine {

struct RagdollJointLimitLabSettings3D {
    float durationSeconds = 1.0f;
    float stiffnessScale = 0.35f;
    float dampingScale = 1.0f;
    float maximumTorqueScale = 1.0f;
};

enum class RagdollJointLimitEndpoint3D { Minimum, Maximum };

struct RagdollJointLimitTrial3D {
    std::string linkId;
    RagdollAxis3D axis = RagdollAxis3D::Twist;
    RagdollJointLimitEndpoint3D endpoint = RagdollJointLimitEndpoint3D::Minimum;
    std::size_t stepCount = 0;
    float targetRadians = 0.0f;
    float finalRadians = 0.0f;
    float minimumObservedRadians = 0.0f;
    float maximumObservedRadians = 0.0f;
    float maximumMotorTorqueNewtonMeters = 0.0f;
    float maximumConstraintTorqueNewtonMeters = 0.0f;
    float appliedMotorTorqueLimitNewtonMeters = 0.0f;
    bool finite = true;
    bool targetApplied = false;
    // Duas poses backend-neutral para auditoria visual: imediatamente apos o
    // spawn e no fim da janela. Nao sao usadas para aprovar o eixo.
    std::vector<PhysicsBodyState3D> initialLinks;
    std::vector<PhysicsBodyState3D> finalLinks;
};

struct RagdollJointLimitLabResult3D {
    std::string profileId;
    float fixedStepSeconds = ReinforcementLearningFixedStepSeconds;
    std::vector<RagdollJointLimitTrial3D> trials;
    std::vector<CalibrationFinding3D> findings;

    [[nodiscard]] std::size_t findingCount(CalibrationFindingSeverity3D severity) const;
    [[nodiscard]] bool hasErrors() const;
};

// Cada extremo roda numa cena nova, sem gravidade/chao e sem escrita de
// transform. Todos os outros DOFs recebem alvo neutro. O ensaio verifica o
// limite realmente publicado pelo backend e a telemetria do motor; nao prova
// que o movimento tenha semantica anatomica correta.
[[nodiscard]] RagdollJointLimitLabResult3D runRagdollJointLimitLab3D(
    const RagdollProfile3D& profile, const RagdollJointLimitLabSettings3D& settings = {});

[[nodiscard]] std::string renderRagdollJointLimitLabMarkdown3D(const RagdollJointLimitLabResult3D& result);

// Formato textual consumido por `MatterModelInspector quadros`. Cada
// eixo/extremo vira um cenario com a pose neutra e a pose final.
[[nodiscard]] std::string renderRagdollJointLimitLabPoseDump3D(
    const RagdollJointLimitLabResult3D& result, const RagdollProfile3D& profile);

} // namespace MatterEngine
