#pragma once

#include "Engine/Control/NaturalBalanceSystem3D.hpp"
#include "Engine/Control/WholeBodyController3D.hpp"
#include "Engine/Locomotion/PhysicalFootworkController3D.hpp"

#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>

namespace MatterEngine {

enum class ActiveRagdollPhase3D : std::uint8_t {
    Settling,
    Standing,
    Balancing,
    Walking,
    RecoveryStep,
    Falling
};

struct ActiveRagdollConfig3D {
    bool enableFreeBaseGravityCompensation = true;
    // Mantido como controle público por compatibilidade com o laboratório.
    // Na Etapa 2 ele liga/desliga a assistência natural graduada; não existe
    // mais uma pose ou caminho "mágico" separado.
    bool useMagicPelvisStabilization = false;
    bool useWholeBodyDynamics = true;

    float standingStiffnessScale = 1.18f;
    float standingDampingScale = 1.42f;
    float legTorqueAuthorityScale = 1.45f;
    float spineTorqueAuthorityScale = 1.34f;
    float armTorqueAuthorityScale = 1.10f;
    float jointPositionGain = 22.0f;
    float maximumJointVelocityRadiansPerSecond = 12.0f;
    float walkSpeedMetersPerSecond = 0.84f;

    float ankleTorqueResponseNewtonMetersPerMeter = 760.0f;
    float maximumAnkleBalanceTorqueNewtonMeters = 210.0f;
    float torsoUprightTorqueNewtonMetersPerRadian = 480.0f;
    float pelvisUprightTorqueNewtonMetersPerRadian = 340.0f;
    float angularMomentumDampingGainPerSecond = 4.2f;
    float armCounterRotationResponse = 1.0f;
    float neckPostureStiffnessScale = 1.05f;
    float neckPostureDampingScale = 1.34f;
    float headPostureStiffnessScale = 0.92f;
    float headPostureDampingScale = 1.28f;
    // O olhar permanece quase vertical; a cabeça participa do reflexo sem
    // virar um peso jogado para trás junto com o tórax.
    float headCounterLeanScale = 0.34f;
    float restingVerticalAssistScale = 0.47f;

    float recoveryStepTriggerUrgency = 0.075f;
    // Um passo humano precisa comecar enquanto o ponto de captura ainda esta
    // alguns centimetros DENTRO da base. Esperar margem negativa significa
    // iniciar a perna depois que a queda ja se tornou inevitavel.
    float recoveryStepMinimumCaptureMarginMeters = 0.035f;
    float recoveryStepMinimumHorizontalSpeedMetersPerSecond = 0.08f;
    float recoveryStepEmergencyCaptureMarginMeters = -0.145f;
    float recoveryStepStaticOutsideDelaySeconds = 0.18f;
    float emergencyFastStepUrgency = 0.52f;
    float recoverySettlingSeconds = 0.20f;
    float minimumRecoverySupportConfidence = 0.56f;
    // Após touchdown a nova base precisa realmente receber carga antes de
    // retirar novamente o mesmo pé. Este intervalo é só para encadear
    // passadas; a primeira reação ao impacto continua imediata.
    float recoveryStepCooldownSeconds = 0.10f;
    float recoveryLandingOvershootMeters = 0.095f;
    // Sob um impacto forte o humano nao tenta zerar instantaneamente todo o
    // momento: acompanha parte da velocidade com uma sequencia de passos e
    // freia ao longo deles. Esta referencia e apenas uma meta do WBC.
    // Um impacto forte nao pode ser tratado como se o corpo devesse parar no
    // mesmo ponto: essa meta exigiria uma ZMP impossivel e dobraria o tronco.
    // O controlador transforma parte da velocidade adquirida em locomocao
    // temporaria, deixa os pes ultrapassarem o COM e freia ao longo das
    // passadas seguintes. Nao ha forca externa auxiliar aqui; esta e apenas
    // a velocidade de referencia do controlador dinamico de corpo inteiro.
    float impactTravelVelocityFraction = 0.78f;
    float maximumImpactTravelSpeedMetersPerSecond = 1.75f;
    float impactTravelAccelerationMetersPerSecondSquared = 22.0f;
    float impactTravelBrakingMetersPerSecondSquared = 2.8f;
    float walkCompletionToleranceMeters = 0.12f;
    float minimumStandingBeforeWalkSeconds = 0.32f;
    float walkPreparationSeconds = 0.34f;

    // A base estreita é fisicamente frágil. O controlador não arrasta os
    // pés para fora: depois de confirmar a postura estreita, solicita uma
    // passada curta para reconstruir uma base de apoio real.
    float desiredStandingStanceWidthMeters = 0.30f;
    float minimumStableStanceWidthMeters = 0.235f;
    float stableStanceReleaseWidthMeters = 0.260f;
    float narrowStanceConfirmationSeconds = 0.22f;
    // Reparo passivo (não queda): evita iniciar uma passada enquanto a
    // Physgun ainda mantém um membro suspenso. Impactos usam o caminho de
    // captura reflexiva acima e não esperam esta confirmação.
    float unsupportedFootReplantDelaySeconds = 0.48f;
    float stanceAdjustmentCooldownSeconds = 0.55f;
    float stanceSagittalOffsetMeters = 0.035f;

    float fallProtectionBodyUpDot = 0.68f;
    float fallProtectionCaptureMarginMeters = -0.30f;
    float fallProtectionMinimumSpeedMetersPerSecond = 0.58f;
};

struct ActiveRagdollTelemetry3D {
    ActiveRagdollPhase3D phase = ActiveRagdollPhase3D::Settling;
    Vec3 centerOfMass;
    Vec3 centerOfMassVelocity;
    Vec3 centerOfPressure;
    Vec3 capturePoint;
    Vec3 supportMinimum;
    Vec3 supportMaximum;
    float captureMarginMeters = 0.0f;
    float bodyUpDot = 1.0f;
    float torsoUpDot = 1.0f;
    float neckUpDot = 1.0f;
    float headUpDot = 1.0f;
    float timeInPhaseSeconds = 0.0f;
    Vec3 centroidalAngularMomentum;
    float hipStrategyWeight = 0.0f;
    float admissibilityProjectionScale = 1.0f;
    bool copAdmissible = true;
    bool frictionAdmissible = true;
    std::size_t supportContactCount = 0;
    bool leftFootSupported = false;
    bool rightFootSupported = false;
    bool recoveryStepActive = false;
    FootSide3D recoverySwingSide = FootSide3D::Left;
    float recoveryStepProgress = 0.0f;
    FootPose3D recoveryStepTarget;
    FootPose3D recoveryStepPose;
    float recoveryUrgency = 0.0f;
    float assistanceAuthority = 0.0f;
    float pelvisAssistForceNewtons = 0.0f;
    float spineAssistForceNewtons = 0.0f;
    bool wholeBodySolved = false;
    bool wholeBodySolverOptimal = false;
    std::uint32_t wholeBodySolverIterations = 0;
    float wholeBodyDynamicsResidual = 0.0f;
    float wholeBodySolverPrimalResidual = 0.0f;
    float wholeBodySolverDualResidual = 0.0f;
    float wholeBodyMaximumTorqueNewtonMeters = 0.0f;
    float leftGroundReactionNewtons = 0.0f;
    float rightGroundReactionNewtons = 0.0f;
    Vec3 desiredRootLinearAcceleration;
    Vec3 solvedRootLinearAcceleration;
    Vec3 solvedCenterOfMassAcceleration;
    Vec3 desiredCenterOfMass;
    Vec3 dynamicsCenterOfMass;
    Vec3 dynamicsCenterOfMassVelocity;
    Vec3 wholeBodyDesiredHorizontalVelocity;
    Vec3 desiredContactAcceleration;
    Vec3 desiredSwingFootAcceleration;
    Vec3 solvedSwingFootAcceleration;
    float footworkWeightTransferProgress = 1.0f;
    bool wholeBodySwingReleased = false;
    float toeOffSagittalErrorMeters = 0.0f;
    float toeOffLateralErrorMeters = 0.0f;
    Vec3 leftContactForceNewtons;
    Vec3 rightContactForceNewtons;
    float leftSoleUpDot = 1.0f;
    float rightSoleUpDot = 1.0f;
    float stanceWidthMeters = 0.0f;
    bool fallen = false;
    bool walking = false;
    bool walkPending = false;
    float walkRequestedMeters = 0.0f;
    float walkDistanceMeters = 0.0f;
    float walkRemainingMeters = 0.0f;
    std::uint32_t completedWalkSteps = 0;
    bool fallProtectionActive = false;
    Vec3 recoveryDirectionWorld;
};

struct ActiveRagdollControlOutput3D {
    std::vector<RagdollDriveTarget3D> driveTargets;
    bool gravityCompensationEnabled = true;
    bool applyRootForce = false;
    Vec3 rootForceNewtons;
    Vec3 rootTorqueNewtonMeters;
    bool applySpineForce = false;
    std::uint32_t spineLinkIndex = 0;
    Vec3 spineForceNewtons;
    Vec3 spineTorqueNewtonMeters;
};

// Orquestrador da Etapa 2. O Footwork físico cuida das pernas e dos passos;
// NaturalBalanceSystem3D estima apoio/captura e dosa reações; este módulo
// compõe os alvos do corpo inteiro sem conhecer PhysX ou escrever transforms.
class ActiveRagdollController3D final {
public:
    void reset(const RagdollProfile3D& profile,
        Quaternion worldOrientation);
    void update(const RagdollProfile3D& profile,
        const RagdollState3D& state, float deltaTime,
        const FootworkTerrainProbe3D* terrain = nullptr,
        const RagdollDynamics3D* dynamics = nullptr);
    void requestWalkDistance(float distanceMeters);
    void cancelWalk();
    // A Physgun continua interagindo com uma articulation ativa, mas uma
    // manipulação explícita de membro não deve ser classificada como queda e
    // disparar uma passada contra o próprio grab.
    void setExternalManipulationActive(bool active,
        std::uint32_t linkIndex = std::numeric_limits<std::uint32_t>::max()) {
        if (m_externalManipulationActive && !active) {
            // Soltar a Physgun deixa uma velocidade residual no membro. Uma
            // passada de captura no primeiro frame reage ao handle, não ao
            // estado corporal já livre, e costuma escolher a perna errada.
            // Damos tempo curto para o amortecimento físico e então a rotina
            // persistente de postura decide se é necessário replantar.
            m_recoveryCooldownSeconds = std::max(
                m_recoveryCooldownSeconds, 0.35f);
            m_outsideSupportSeconds = 0.0f;
        }
        m_externalManipulationActive = active;
        m_externalManipulatedLinkIndex = active
            ? linkIndex : std::numeric_limits<std::uint32_t>::max();
    }

    [[nodiscard]] ActiveRagdollConfig3D& config() { return m_config; }
    [[nodiscard]] const ActiveRagdollConfig3D& config() const {
        return m_config;
    }
    [[nodiscard]] NaturalBalanceConfig3D& balanceConfig() {
        return m_balance.config();
    }
    [[nodiscard]] const NaturalBalanceConfig3D& balanceConfig() const {
        return m_balance.config();
    }
    [[nodiscard]] PhysicalFootworkConfig3D& footworkConfig() {
        return m_footwork.config();
    }
    [[nodiscard]] const ActiveRagdollTelemetry3D& telemetry() const {
        return m_telemetry;
    }
    [[nodiscard]] const ActiveRagdollControlOutput3D& output() const {
        return m_output;
    }

private:
    void transitionTo(ActiveRagdollPhase3D phase);
    void configurePhysicalFootwork();
    void buildFallbackTargets(const RagdollProfile3D& profile);
    void buildWholeBodyTargets(const RagdollProfile3D& profile,
        const RagdollState3D& state);
    void applyWorldTorque(const RagdollProfile3D& profile,
        const RagdollState3D& state, std::size_t linkIndex,
        Vec3 torqueWorld, float scale);
    void applyLegEndpointForce(const RagdollProfile3D& profile,
        const RagdollState3D& state, bool left,
        Vec3 forceWorld, float scale);
    [[nodiscard]] RagdollDriveTarget3D* findTarget(
        std::size_t linkIndex, RagdollAxis3D axis);

    ActiveRagdollConfig3D m_config;
    ActiveRagdollTelemetry3D m_telemetry;
    ActiveRagdollControlOutput3D m_output;
    NaturalBalanceSystem3D m_balance;
    PhysicalFootworkController3D m_footwork;
    WholeBodyController3D m_wholeBody;
    NaturalBalanceLinkMap3D m_links;
    Quaternion m_spawnOrientation;
    Vec3 m_forward { 1.0f, 0.0f, 0.0f };
    Vec3 m_right { 0.0f, -1.0f, 0.0f };
    Vec3 m_walkStart;
    Vec3 m_recoveryVelocityReference;
    float m_ageSeconds = 0.0f;
    float m_walkRequestedMeters = 0.0f;
    float m_lastWalkDistanceMeters = 0.0f;
    float m_walkPreparationSeconds = 0.0f;
    float m_recoveryCooldownSeconds = 0.0f;
    float m_outsideSupportSeconds = 0.0f;
    float m_narrowStanceSeconds = 0.0f;
    float m_leftUnsupportedSeconds = 0.0f;
    float m_rightUnsupportedSeconds = 0.0f;
    float m_leftSupportStableSeconds = 0.0f;
    float m_rightSupportStableSeconds = 0.0f;
    float m_stanceAdjustmentCooldownSeconds = 0.0f;
    float m_confirmedLandingSeconds = 0.0f;
    std::uint64_t m_lastCompletedFootworkSteps = 0;
    std::uint64_t m_walkStartCompletedFootworkSteps = 0;
    std::uint64_t m_lastCaptureRedirectedStep =
        std::numeric_limits<std::uint64_t>::max();
    bool m_footworkInitialized = false;
    bool m_reactiveRecoveryStep = false;
    bool m_reactiveRecoveryTrailingStep = false;
    FootSide3D m_lastReactiveRecoverySide = FootSide3D::Left;
    bool m_hasLastReactiveRecoverySide = false;
    bool m_externalManipulationActive = false;
    std::uint32_t m_externalManipulatedLinkIndex =
        std::numeric_limits<std::uint32_t>::max();
    bool m_walkPending = false;
    bool m_walking = false;
};

} // namespace MatterEngine
