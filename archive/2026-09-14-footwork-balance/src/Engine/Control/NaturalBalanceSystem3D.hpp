#pragma once

#include "Engine/Physics/PhysicsScene3D.hpp"

#include <cstddef>
#include <cstdint>

namespace MatterEngine {

enum class NaturalBalanceRegime3D : std::uint8_t {
    Settling,
    Stable,
    CounterRotating,
    RecoveryStep,
    Fallen
};

struct NaturalBalanceLinkMap3D {
    std::size_t pelvis = 0;
    std::size_t abdomen = 1;
    std::size_t chest = 2;
    std::size_t upperChest = 3;
    std::size_t neck = 4;
    std::size_t head = 5;
    std::size_t leftUpperArm = 6;
    std::size_t leftForearm = 7;
    std::size_t leftHand = 8;
    std::size_t rightUpperArm = 9;
    std::size_t rightForearm = 10;
    std::size_t rightHand = 11;
    std::size_t leftThigh = 12;
    std::size_t leftShin = 13;
    std::size_t leftFoot = 14;
    std::size_t rightThigh = 15;
    std::size_t rightShin = 16;
    std::size_t rightFoot = 17;
};

struct NaturalBalanceConfig3D {
    float contactImpulseThresholdNewtonSeconds = 0.002f;
    // Contatos de repouso podem não gerar impulso reportável em todo tick.
    // Mantemos uma memória curta por pé para não desligar a perna de apoio
    // entre dois frames consecutivos do solver.
    float footContactGraceSeconds = 0.065f;
    float safeCaptureMarginMeters = 0.075f;
    float stepCaptureMarginMeters = -0.025f;
    float emergencyCaptureMarginMeters = -0.180f;
    float minimumRecoverySpeedMetersPerSecond = 0.34f;
    float maximumRecoverySpeedMetersPerSecond = 1.65f;

    float nominalPelvisHeightMeters = 0.98f;
    float fallenPelvisHeightMeters = 0.43f;
    float fallenUpDot = 0.24f;
    float fallenConfirmationSeconds = 0.12f;

    // O auxílio externo não substitui as pernas: com apoio e postura bons,
    // retira somente parte do peso. Sem contato, ou depois da queda, sua
    // autoridade converge rapidamente a zero.
    bool assistanceEnabled = true;
    float pelvisWeightAssistFraction = 0.30f;
    float pelvisHeightStiffnessNewtonsPerMeter = 850.0f;
    float pelvisHeightDampingNewtonSecondsPerMeter = 150.0f;
    float pelvisMaximumAssistForceNewtons = 380.0f;
    float pelvisHorizontalAssistNewtonsPerMeter = 420.0f;
    float pelvisHorizontalAssistDampingNewtonSecondsPerMeter = 115.0f;
    float pelvisMaximumHorizontalAssistNewtons = 125.0f;
    float pelvisUprightStiffnessNewtonMeters = 340.0f;
    float pelvisUprightDampingNewtonMeterSeconds = 64.0f;
    float pelvisMaximumAssistTorqueNewtonMeters = 210.0f;
    // Com duas plantas no chão as pernas carregam a maior parte do peso.
    // Durante a curta transferência para apoio simples, a autoridade volta
    // ao valor biomecânico já validado pela marcha, sem levitar em repouso.
    float bilateralSupportVerticalAssistScale = 0.47f;
    float singleSupportVerticalAssistScale = 1.0f;
    float singleSupportMaximumAssistScale = 1.0f;
    float singleSupportHeightGainScale = 1.0f;

    // A coluna recebe uma parcela deliberadamente menor. É uma distribuição
    // do auxílio pelo corpo, não uma segunda raiz suspensa.
    float spineWeightAssistFraction = 0.045f;
    float spineUprightStiffnessNewtonMeters = 52.0f;
    float spineUprightDampingNewtonMeterSeconds = 11.0f;
    float spineMaximumAssistForceNewtons = 42.0f;
    float spineHorizontalAssistFraction = 0.10f;
    float spineMaximumAssistTorqueNewtonMeters = 38.0f;

    float torsoInternalUprightStiffnessNewtonMeters = 390.0f;
    float torsoInternalUprightDampingNewtonMeterSeconds = 58.0f;
    float pelvisInternalUprightStiffnessNewtonMeters = 430.0f;
    float pelvisInternalUprightDampingNewtonMeterSeconds = 72.0f;
    float angularMomentumDampingPerSecond = 4.2f;
    float maximumInternalTorsoTorqueNewtonMeters = 330.0f;
    float maximumInternalPelvisTorqueNewtonMeters = 420.0f;
    float maximumInternalArmTorqueNewtonMeters = 62.0f;
    float supportTorqueNewtonMetersPerMeter = 760.0f;
    float maximumSupportTorqueNewtonMeters = 210.0f;
    // A perna em apoio não apenas segura o pé no alvo. Ela empurra o chão
    // na direção do movimento/erro de captura para que a reação do solo
    // desacelere o corpo no sentido oposto.
    float stancePushNewtonsPerMeter = 820.0f;
    float stancePushDampingNewtonSecondsPerMeter = 155.0f;
    // Mesmo dentro do polígono de apoio, o corpo parado converge ao centro
    // da base. Sem isso qualquer pose inclinada "ainda estável" era aceita.
    float stanceCenteringNewtonsPerMeter = 520.0f;
    float movingStanceCenteringScale = 0.0f;
    float maximumStancePushNewtons = 420.0f;
};

struct NaturalBalanceState3D {
    NaturalBalanceRegime3D regime = NaturalBalanceRegime3D::Settling;
    Vec3 centerOfMass;
    Vec3 centerOfMassVelocity;
    Vec3 centerOfPressure;
    Vec3 capturePoint;
    Vec3 supportCenter;
    Vec3 supportMinimum;
    Vec3 supportMaximum;
    Vec3 centroidalAngularMomentum;
    Vec3 recoveryDirectionWorld;
    float captureMarginMeters = 0.0f;
    float recoveryUrgency = 0.0f;
    float bodyUpDot = 1.0f;
    float torsoUpDot = 1.0f;
    float pelvisHeightMeters = 0.0f;
    float supportConfidence = 0.0f;
    float assistanceAuthority = 0.0f;
    std::size_t supportContactCount = 0;
    bool leftFootSupported = false;
    bool rightFootSupported = false;
    bool fallen = false;
};

struct NaturalBalanceResponse3D {
    Vec3 recoveryVelocityWorld;
    Vec3 internalSupportTorqueWorld;
    Vec3 stancePushForceWorld;
    Vec3 internalPelvisTorqueWorld;
    Vec3 internalTorsoTorqueWorld;
    Vec3 internalArmTorqueWorld;
    Vec3 pelvisAssistForceNewtons;
    Vec3 pelvisAssistTorqueNewtonMeters;
    Vec3 spineAssistForceNewtons;
    Vec3 spineAssistTorqueNewtonMeters;
    float assistanceAuthority = 0.0f;
};

// Estimador e planejador reativo de equilíbrio. Consome apenas snapshots
// físicos e produz intenções/forças limitadas; nunca altera transforms.
class NaturalBalanceSystem3D final {
public:
    void reset(Quaternion worldOrientation);
    void setDesiredHorizontalVelocity(Vec3 velocityWorld);
    void update(const RagdollProfile3D& profile,
        const RagdollState3D& state,
        const NaturalBalanceLinkMap3D& links, float deltaTime);

    [[nodiscard]] NaturalBalanceConfig3D& config() { return m_config; }
    [[nodiscard]] const NaturalBalanceConfig3D& config() const {
        return m_config;
    }
    [[nodiscard]] const NaturalBalanceState3D& state() const {
        return m_state;
    }
    [[nodiscard]] const NaturalBalanceResponse3D& response() const {
        return m_response;
    }
    [[nodiscard]] Vec3 forward() const { return m_forward; }
    [[nodiscard]] Vec3 lateral() const { return m_lateral; }

private:
    NaturalBalanceConfig3D m_config;
    NaturalBalanceState3D m_state;
    NaturalBalanceResponse3D m_response;
    Vec3 m_forward { 1.0f, 0.0f, 0.0f };
    Vec3 m_lateral { 0.0f, 1.0f, 0.0f };
    float m_ageSeconds = 0.0f;
    float m_supportConfidence = 0.0f;
    float m_assistanceAuthority = 0.0f;
    float m_fallenSeconds = 0.0f;
    float m_leftFootContactGraceSeconds = 0.0f;
    float m_rightFootContactGraceSeconds = 0.0f;
    Vec3 m_desiredHorizontalVelocityWorld;
};

} // namespace MatterEngine
