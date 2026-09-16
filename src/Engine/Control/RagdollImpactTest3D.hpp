#pragma once

#include <cstdint>

namespace MatterEngine {

// Gerador deterministico de perturbacoes para validar controladores de
// equilibrio. Ele deliberadamente nao conhece PhysX nem ragdolls: apenas
// publica uma forca e um angulo relativos ao corpo. O laboratorio e os testes
// automatizados consomem a mesma maquina de estados e aplicam a forca atraves
// do contrato backend-neutral de PhysicsScene3D.
enum class RagdollImpactMode3D {
    DirectPulse,
    Continuous,
    Random
};

struct RagdollImpactTestConfig3D {
    RagdollImpactMode3D mode = RagdollImpactMode3D::DirectPulse;
    float forceNewtons = 320.0f;
    // 0 = frente do ragdoll; 90 = esquerda; 180 = costas; 270 = direita.
    float directionDegrees = 90.0f;
    float intervalSeconds = 3.0f;
    float pulseDurationSeconds = 0.12f;

    float randomMinimumForceNewtons = 120.0f;
    float randomMaximumForceNewtons = 420.0f;
    float randomMinimumIntervalSeconds = 1.5f;
    float randomMaximumIntervalSeconds = 4.0f;
    float randomMinimumDurationSeconds = 0.08f;
    float randomMaximumDurationSeconds = 1.25f;
};

struct RagdollImpactTestOutput3D {
    bool applying = false;
    float forceNewtons = 0.0f;
    float directionDegrees = 0.0f;
    float secondsRemaining = 0.0f;
    float secondsUntilNext = 0.0f;
    std::uint64_t eventCount = 0;
};

class RagdollImpactTest3D {
public:
    [[nodiscard]] RagdollImpactTestConfig3D& config() { return m_config; }
    [[nodiscard]] const RagdollImpactTestConfig3D& config() const {
        return m_config;
    }
    [[nodiscard]] const RagdollImpactTestOutput3D& output() const {
        return m_output;
    }
    [[nodiscard]] bool running() const { return m_running; }

    void setRunning(bool running);
    void triggerNow();
    void reset(std::uint32_t randomSeed = 0x4D415454u);
    void update(float deltaTime);

private:
    [[nodiscard]] float randomUnit();
    [[nodiscard]] float randomRange(float minimum, float maximum);
    void beginConfiguredPulse();
    void beginRandomEvent();
    void sanitizeConfig();

    RagdollImpactTestConfig3D m_config;
    RagdollImpactTestOutput3D m_output;
    RagdollImpactMode3D m_activeMode = RagdollImpactMode3D::DirectPulse;
    bool m_running = false;
    bool m_triggerRequested = false;
    float m_activeSeconds = 0.0f;
    float m_countdownSeconds = 0.0f;
    float m_activeForceNewtons = 0.0f;
    float m_activeDirectionDegrees = 0.0f;
    std::uint32_t m_randomState = 0x4D415454u;
    std::uint64_t m_eventCount = 0;
};

} // namespace MatterEngine
