#pragma once

#include "Engine/Locomotion/FootworkTypes3D.hpp"
#include "Engine/Physics/PhysicsScene3D.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <vector>

namespace MatterEngine {

struct WholeBodyControllerConfig3D {
    // Centro de massa nominal abaixo da extensão máxima das pernas:
    // mantém joelho/quadril fora do singular "travado reto" e preserva
    // autoridade para reagir nos dois sentidos.
    float nominalCenterOfMassHeightMeters = 0.930f;
    float pelvisPositionGain = 82.0f;
    float pelvisVelocityGain = 18.0f;
    float pelvisVerticalPositionGain = 118.0f;
    float pelvisVerticalVelocityGain = 24.0f;
    float pelvisOrientationGain = 92.0f;
    float pelvisAngularVelocityGain = 19.0f;
    float jointPositionGain = 72.0f;
    float jointVelocityGain = 15.0f;
    float plantedFootPositionGain = 260.0f;
    float plantedFootVelocityGain = 38.0f;
    float plantedFootOrientationGain = 240.0f;
    float plantedFootAngularVelocityGain = 34.0f;
    float headOrientationGain = 96.0f;
    float headAngularVelocityGain = 20.0f;
    float frictionCoefficient = 0.78f;
    // Corrida e passos de captura humanos produzem picos verticais acima de
    // duas vezes o peso. 1.85 saturava exatamente quando a segunda perna
    // precisava frear um impacto; isto é limite do contato, não assistência.
    float maximumContactForceWeightScale = 2.55f;
    float torqueLimitScale = 1.58f;
    // Passos reflexivos precisam deslocar uma extremidade leve antes que o
    // apoio antigo ceda. Este teto limita tarefas cartesianas/articulares,
    // não a aceleração da raiz; 120 m/s² é um pico curto de pé em voo e os
    // torques continuam limitados pelo perfil anatômico.
    float generalizedAccelerationLimit = 120.0f;
};

struct WholeBodyControllerTelemetry3D {
    bool dynamicsAvailable = false;
    bool solved = false;
    bool solverOptimal = false;
    std::uint32_t contactCount = 0;
    std::uint32_t solverIterations = 0;
    float maximumTorqueNewtonMeters = 0.0f;
    float leftNormalForceNewtons = 0.0f;
    float rightNormalForceNewtons = 0.0f;
    float dynamicsResidual = 0.0f;
    float solverPrimalResidual = 0.0f;
    float solverDualResidual = 0.0f;
    float assistanceRatio = 0.0f;
    Vec3 desiredCenterOfMass;
    Vec3 measuredCenterOfMass;
    Vec3 measuredCenterOfMassVelocity;
    Vec3 desiredHorizontalVelocity;
    Vec3 leftContactForceNewtons;
    Vec3 rightContactForceNewtons;
    Vec3 desiredRootLinearAcceleration;
    Vec3 solvedRootLinearAcceleration;
    Vec3 solvedCenterOfMassAcceleration;
    Vec3 desiredSwingFootAcceleration;
    Vec3 solvedSwingFootAcceleration;
    std::uint32_t swingFootLink = RagdollDynamics3D::InvalidIndex;
};

struct WholeBodyMotionIntent3D {
    Vec3 desiredHorizontalVelocity;
    // Aceleração corporal solicitada pela estratégia de apoio. É o oposto
    // horizontal da força que a perna deseja exercer no chão, dividido pela
    // massa. O WBC ainda a projeta no ZMP/atrito reais; portanto isto não é
    // força externa nem assistência aplicada à raiz.
    Vec3 desiredContactAccelerationWorld;
    // Direção vertical desejada para o tórax. Em repouso é +Z; durante uma
    // perturbação o controlador pode inclinar a cadeia axial contra a queda
    // para deslocar COM e momento angular sem força externa na raiz.
    Vec3 desiredTorsoUpWorld { 0.0f, 0.0f, 1.0f };
    // Referência vestibular independente do tórax. A cabeça não pode ser
    // apenas o último peso pendurado na coluna: em repouso olha o horizonte e,
    // durante uma perturbação, participa da contrarrotação sem quebrar o
    // pescoço para trás.
    Vec3 desiredHeadUpWorld { 0.0f, 0.0f, 1.0f };
    float balanceRecoveryUrgency = 0.0f;
    bool fallProtectionActive = false;
    // Mantém a hierarquia/autoridade própria da locomoção mesmo durante a
    // transferência quase-estática, quando a velocidade desejada precisa ser
    // exatamente zero para colocar o DCM dentro da sola de apoio.
    bool locomotionActive = false;
    // Em apoio simples o centro de pressão, e não uma atração cartesiana
    // direta ao pé, determina a aceleração horizontal fisicamente possível.
    bool capturePointSupport = false;
    std::array<FootPose3D, 2> desiredFootSoles;
    std::array<bool, 2> hasDesiredFootSole {};
    // Perna escolhida para a próxima passada. Durante a transferência de
    // peso ela ainda é contato e não deve obedecer antecipadamente à pose
    // de voo; depois do toe-off, trackSwingFoot assume a tarefa cartesiana.
    std::array<bool, 2> unloadingFoot {};
    std::array<bool, 2> trackSwingFoot {};
    // Escala hierárquica da tarefa cartesiana. O shuffle que recompõe a
    // base é deliberadamente secundário ao equilíbrio do COM; uma passada
    // externa de captura mantém autoridade integral.
    std::array<float, 2> swingFootTaskWeightScale { 1.0f, 1.0f };
    // A Physgun pode deslocar uma perna enquanto o restante do corpo segue
    // ativo. O WBC deve retirar somente essa cadeia das preferências de pose
    // e do contato; disputar o membro com o handle transmite uma força
    // artificial enorme à bacia e derruba o ragdoll inteiro.
    std::array<bool, 2> externallyManipulatedLeg {};
    // Pé que deveria receber a transferência de peso, mas perdeu contato.
    // É uma tarefa de aquisição de apoio, distinta do pé de swing.
    std::array<bool, 2> urgentPlantFoot {};
};

// Controlador de dinâmica inversa flutuante. Resolve acelerações, torques e
// wrenches de contato em um único QP e escreve somente torques articulares.
// Não aplica forças na raiz e não conhece PhysX.
class WholeBodyController3D final {
public:
    WholeBodyController3D();
    ~WholeBodyController3D();
    WholeBodyController3D(WholeBodyController3D&&) noexcept;
    WholeBodyController3D& operator=(WholeBodyController3D&&) noexcept;
    WholeBodyController3D(const WholeBodyController3D&) = delete;
    WholeBodyController3D& operator=(const WholeBodyController3D&) = delete;

    void reset(Quaternion facingOrientation);
    [[nodiscard]] bool update(const RagdollProfile3D& profile,
        const RagdollState3D& state,
        const RagdollDynamics3D& dynamics,
        bool leftFootSupported, bool rightFootSupported,
        std::size_t leftFootLink, std::size_t rightFootLink,
        Vec3 supportCenter, const WholeBodyMotionIntent3D& intent,
        float deltaTime,
        std::vector<RagdollDriveTarget3D>& targets);

    [[nodiscard]] WholeBodyControllerConfig3D& config() {
        return m_config;
    }
    [[nodiscard]] const WholeBodyControllerTelemetry3D& telemetry() const {
        return m_telemetry;
    }

private:
    struct Impl;
    std::unique_ptr<Impl> m_impl;
    WholeBodyControllerConfig3D m_config;
    WholeBodyControllerTelemetry3D m_telemetry;
    Quaternion m_facingOrientation;
    std::array<Vec3, 2> m_contactPosition;
    std::array<Quaternion, 2> m_contactOrientation;
    std::array<bool, 2> m_contactAnchored {};
    std::array<Vec3, 2> m_previousSwingTarget;
    std::array<bool, 2> m_hasPreviousSwingTarget {};
    std::array<std::vector<float>, 2> m_previousContactJacobian;
};

} // namespace MatterEngine
