#pragma once

#include "Engine/Locomotion/FootworkSystem3D.hpp"
#include "Engine/Physics/PhysicsScene3D.hpp"

#include <array>
#include <cstddef>
#include <vector>

namespace MatterEngine {

enum class PhysicalFootworkMode3D : std::uint8_t {
    Manual,
    AutonomousFollow,
    // A raiz virtual acompanha a pelve física, mas a intenção de movimento
    // vem de um controlador externo de equilíbrio. É o elo entre o Footwork
    // isolado da Etapa 1 e o corpo inteiro da Etapa 2.
    ReactiveBalance
};

struct PhysicalFootworkConfig3D {
    // A bacia recebe somente a parcela de sustentação que pernas ainda não
    // conseguem produzir. A autoridade é modulada por apoio/postura e some
    // durante uma queda, portanto esta força não funciona como levitação.
    float pelvisHeightMeters = 0.94f;
    float gravitySupportFraction = 0.78f;
    float verticalStiffnessNewtonsPerMeter = 1550.0f;
    float verticalDampingNewtonSecondsPerMeter = 245.0f;
    float horizontalStiffnessNewtonsPerMeter = 620.0f;
    float horizontalDampingNewtonSecondsPerMeter = 145.0f;
    float uprightStiffnessNewtonMeters = 520.0f;
    float uprightDampingNewtonMeterSeconds = 82.0f;
    float yawStiffnessNewtonMeters = 245.0f;
    float yawDampingNewtonMeterSeconds = 52.0f;
    float maximumRootForceNewtons = 1150.0f;
    float maximumRootTorqueNewtonMeters = 330.0f;

    float jointPositionGain = 18.0f;
    float jointMaximumVelocityRadiansPerSecond = 10.0f;
    float jointStiffnessScale = 1.15f;
    float jointDampingScale = 1.28f;
    float jointMaximumTorqueScale = 1.0f;
    float footTaskPositionGainNewtonsPerMeter = 1050.0f;
    float footTaskVelocityGainNewtonSecondsPerMeter = 92.0f;
    float maximumFootTaskForceNewtons = 520.0f;
    // Tarefa rotacional do efetor final. A posição da sola sozinha não
    // impede que a caixa do pé cumpra o alvo apoiada numa quina. Estes
    // ganhos fazem o conjunto quadril/joelho/tornozelo alinhar a planta à
    // orientação planejada, ainda exclusivamente por torques articulares.
    float footOrientationStiffnessNewtonMetersPerRadian = 185.0f;
    float footOrientationDampingNewtonMeterSeconds = 30.0f;
    float maximumFootOrientationTorqueNewtonMeters = 165.0f;
    float plantedFootOrientationScale = 1.20f;
    float footJointTorqueAuthorityScale = 1.0f;
    float plantedFootDriveStiffnessScale = 1.0f;
    float plantedFootDriveDampingScale = 1.0f;
    float plantedFootDriveTorqueScale = 1.0f;
    // Alguns milímetros abaixo do plano criam pré-carga de contato e evitam
    // o equilíbrio frágil na ponta do pé sem colar ou travar a sola.
    float plantedSolePreloadMeters = 0.0035f;
    float plantedContactPreloadNewtons = 55.0f;
};

struct PhysicalFootworkTelemetry3D {
    bool initialized = false;
    PhysicalFootworkMode3D mode = PhysicalFootworkMode3D::Manual;
    float supportConfidence = 0.0f;
    float uprightDot = 0.0f;
    float pelvisHeightErrorMeters = 0.0f;
    float rootForceNewtons = 0.0f;
    float rootTorqueNewtonMeters = 0.0f;
    float leftSoleUpDot = 1.0f;
    float rightSoleUpDot = 1.0f;
    float stanceWidthMeters = 0.0f;
    std::uint32_t supportedFootCount = 0;
    bool assistanceActive = false;
    bool fallen = false;
};

struct PhysicalFootworkOutput3D {
    std::vector<RagdollDriveTarget3D> driveTargets;
    Vec3 rootForceNewtons;
    Vec3 rootTorqueNewtonMeters;
    bool applyRootForce = false;
    bool gravityCompensationEnabled = true;
};

// Converte o planejador de passadas em controle de uma articulation física.
// Não conhece PhysX, input ou renderização e nunca escreve transforms.
class PhysicalFootworkController3D final {
public:
    [[nodiscard]] bool reset(const RagdollProfile3D& profile,
        const RagdollState3D& state, float facingYawRadians,
        const FootworkTerrainProbe3D& terrain,
        PhysicalFootworkMode3D mode);
    void clear();
    void setInput(const FootworkInput3D& input);
    void setPaused(bool paused);
    // Mantém a raiz virtual do planejador numa trajetória de locomoção
    // deliberada. A bacia física pode oscilar para equilibrar sem deslocar,
    // passo após passo, as duas faixas laterais dos pés.
    void setLocomotionPath(bool enabled, Vec3 origin,
        Vec3 forward, float maximumLateralDeviationMeters = 0.04f);
    [[nodiscard]] bool requestRecoveryStep(FootSide3D side,
        Vec3 captureTarget, bool emergency,
        const FootworkTerrainProbe3D& terrain);
    [[nodiscard]] bool recoverUnsupportedFoot(FootSide3D side,
        Vec3 captureTarget, bool emergency,
        const FootworkTerrainProbe3D& terrain);
    [[nodiscard]] bool retargetActiveStep(Vec3 captureTarget,
        bool allowCrossing, const FootworkTerrainProbe3D& terrain);
    void update(const RagdollProfile3D& profile,
        const RagdollState3D& state, float deltaTime,
        const FootworkTerrainProbe3D& terrain);

    [[nodiscard]] PhysicalFootworkConfig3D& config() { return m_config; }
    [[nodiscard]] const PhysicalFootworkConfig3D& config() const {
        return m_config;
    }
    [[nodiscard]] FootworkSystem3D& planner() { return m_planner; }
    [[nodiscard]] const FootworkSystem3D& planner() const {
        return m_planner;
    }
    [[nodiscard]] const PhysicalFootworkOutput3D& output() const {
        return m_output;
    }
    [[nodiscard]] const PhysicalFootworkTelemetry3D& telemetry() const {
        return m_telemetry;
    }
    [[nodiscard]] PhysicalFootworkMode3D mode() const { return m_mode; }

private:
    struct LinkIndices {
        std::size_t pelvis = 0;
        std::array<std::size_t, 2> thigh { 1, 4 };
        std::array<std::size_t, 2> shin { 2, 5 };
        std::array<std::size_t, 2> foot { 3, 6 };
    };

    [[nodiscard]] FootPose3D physicalSolePose(
        const RagdollProfile3D& profile,
        const RagdollState3D& state, std::size_t footIndex) const;
    void buildNeutralTargets(const RagdollProfile3D& profile);
    void applyLegTask(const RagdollProfile3D& profile,
        const RagdollState3D& state, FootSide3D side,
        const FootPose3D& desiredSole, float deltaTime);
    void buildRootAssistance(const RagdollProfile3D& profile,
        const RagdollState3D& state, float deltaTime);

    PhysicalFootworkConfig3D m_config;
    FootworkSystem3D m_planner;
    PhysicalFootworkOutput3D m_output;
    PhysicalFootworkTelemetry3D m_telemetry;
    LinkIndices m_links;
    FootworkInput3D m_input;
    std::array<Vec3, 2> m_previousDesiredFootPosition;
    std::array<bool, 2> m_hasPreviousDesiredFootPosition {};
    PhysicalFootworkMode3D m_mode = PhysicalFootworkMode3D::Manual;
    float m_supportConfidence = 0.0f;
    float m_elapsedSeconds = 0.0f;
    Vec3 m_pathOrigin;
    Vec3 m_pathForward { 1.0f, 0.0f, 0.0f };
    float m_pathMaximumLateralDeviationMeters = 0.04f;
    bool m_pathConstraintEnabled = false;
    bool m_paused = false;
};

} // namespace MatterEngine
