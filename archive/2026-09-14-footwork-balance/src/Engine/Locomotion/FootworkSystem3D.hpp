#pragma once

#include "Engine/Locomotion/FootworkTypes3D.hpp"

#include <array>

namespace MatterEngine {

// Planejador/executor cinemático isolado. Ele produz alvos 6DoF para dois pés
// e telemetria; não move rigid bodies, não lê input global e não renderiza.
class FootworkSystem3D final {
public:
    FootworkSystem3D() = default;
    explicit FootworkSystem3D(FootworkConfig3D config);

    [[nodiscard]] FootworkConfig3D& config() { return m_config; }
    [[nodiscard]] const FootworkConfig3D& config() const {
        return m_config;
    }

    [[nodiscard]] bool reset(Vec3 rootGroundPosition, float rootYawRadians,
        const FootworkTerrainProbe3D& terrain);
    void clear();
    // Reancora apenas a raiz virtual, preservando os contatos planejados dos
    // pés. Usado pelo meio-esqueleto autônomo quando uma força externa
    // (Physgun, impacto) desloca a bacia e os pés precisam persegui-la.
    void synchronizeRoot(Vec3 rootGroundPosition, Vec3 rootVelocity,
        float rootYawRadians);
    void setAutomaticSteppingEnabled(bool enabled) {
        m_automaticSteppingEnabled = enabled;
    }
    // O planejador pode desenhar a transferência, mas um controlador físico
    // decide quando o COM/capture point realmente entrou na nova base. O
    // laboratório cinemático permanece liberado por padrão.
    void setActiveStepReleaseAllowed(bool allowed) {
        m_activeStepReleaseAllowed = allowed;
    }
    // Opcional para controladores físicos. Sem feedback (laboratório
    // cinemático), a curva termina normalmente; com feedback, TouchDown só
    // conclui depois que o pé correspondente realmente toca o chão.
    void setPhysicalContactFeedback(bool leftSupported,
        bool rightSupported) {
        m_hasPhysicalContactFeedback = true;
        m_physicalFootSupported = {
            leftSupported, rightSupported
        };
        m_hasPhysicalFootPose = false;
    }
    // Variante usada pelo ragdoll: além do contato, fornece onde as solas
    // realmente estão. Sem isto o planner podia aceitar um heel-strike
    // prematuro e registrar o ALVO como posição plantada, embora a perna
    // física tivesse pousado dezenas de centímetros antes.
    void setPhysicalContactFeedback(bool leftSupported,
        bool rightSupported,
        const std::array<FootPose3D, 2>& physicalFootPoses) {
        m_hasPhysicalContactFeedback = true;
        m_physicalFootSupported = {
            leftSupported, rightSupported
        };
        m_physicalFootPose = physicalFootPoses;
        m_hasPhysicalFootPose = true;
    }
    void clearPhysicalContactFeedback() {
        m_hasPhysicalContactFeedback = false;
        m_physicalFootSupported = { false, false };
        m_hasPhysicalFootPose = false;
    }
    // Inicia uma única passada para um alvo de captura explícito. Diferente
    // do input contínuo de caminhada, não encadeia outra passada ao pousar.
    [[nodiscard]] bool requestRecoveryStep(FootSide3D side,
        Vec3 captureTarget, bool emergency,
        const FootworkTerrainProbe3D& terrain);
    // Reclassifica o apoio quando o pé que deveria sustentar o corpo perde
    // contato enquanto o pé planejado para voo já está no chão. Essa troca
    // não é um novo passo arbitrário: corrige a máquina de estados para a
    // topologia física que realmente existe.
    [[nodiscard]] bool recoverUnsupportedFoot(FootSide3D side,
        Vec3 captureTarget, bool emergency,
        const FootworkTerrainProbe3D& terrain);
    [[nodiscard]] bool retargetActiveStep(Vec3 captureTarget,
        bool allowCrossing, const FootworkTerrainProbe3D& terrain);
    void update(const FootworkInput3D& input, float deltaTime,
        const FootworkTerrainProbe3D& terrain);

    [[nodiscard]] const FootworkDebugState3D& state() const {
        return m_debug;
    }

private:
    struct RuntimeFoot {
        FootPose3D pose;
        FootPose3D home;
        FootPose3D swingStart;
        FootPose3D swingTarget;
        FootPhase3D phase = FootPhase3D::Planted;
        float progress = 0.0f;
        float durationSeconds = 0.34f;
        float weightTransferElapsedSeconds = 0.0f;
        float weightTransferDurationSeconds = 0.0f;
        float clearanceMeters = 0.105f;
        float terrainQuality = 1.0f;
        bool fastStep = false;
        // Um contato existente no lift-off não é um touchdown. Passadas com
        // arco só podem terminar depois que a sola realmente descarregou e
        // saiu do piso ao menos uma vez. O shuffle rente ao chão é a exceção
        // deliberada e possui critérios próprios de progresso/proximidade.
        bool physicallyReleased = false;
    };

    void sanitizeConfig();
    void updateHomes(Vec3 desiredVelocity, bool fast);
    void updateDebugState();
    void rebuildSupportPolygon();
    [[nodiscard]] bool beginStep(FootSide3D side,
        const FootworkTerrainProbe3D& terrain,
        Vec3 desiredVelocity, Vec2 movementLocal, bool fast);
    [[nodiscard]] bool replanActiveStep(
        const FootworkTerrainProbe3D& terrain,
        Vec3 desiredVelocity, Vec2 movementLocal, bool fast);
    void updateActiveStep(const FootworkInput3D& input, float deltaTime,
        const FootworkTerrainProbe3D& terrain, Vec3 desiredVelocity);
    void enforceActiveFootSeparation(FootPose3D& pose) const;
    void buildSwingTrajectory();

    FootworkConfig3D m_config;
    std::array<RuntimeFoot, 2> m_feet;
    FootworkDebugState3D m_debug;
    Vec3 m_rootGroundPosition;
    Vec3 m_rootVelocity;
    float m_rootYawRadians = 0.0f;
    FootSide3D m_lastCompletedSide = FootSide3D::Right;
    FootSide3D m_activeSide = FootSide3D::Left;
    bool m_hasActiveStep = false;
    bool m_activeRecoveryStep = false;
    bool m_automaticSteppingEnabled = true;
    bool m_activeStepReleaseAllowed = true;
    bool m_hasPhysicalContactFeedback = false;
    std::array<bool, 2> m_physicalFootSupported {};
    std::array<FootPose3D, 2> m_physicalFootPose;
    bool m_hasPhysicalFootPose = false;
    bool m_inputWasMoving = false;
    float m_targetChaseAccumulatorSeconds = 0.0f;
    Vec3 m_activePlanVelocity;
    Vec2 m_activePlanMovementLocal;
    float m_activePlanYawRadians = 0.0f;
    std::uint64_t m_completedStepCount = 0;
};

} // namespace MatterEngine
