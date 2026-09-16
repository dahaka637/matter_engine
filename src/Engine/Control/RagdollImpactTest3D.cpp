#include "Engine/Control/RagdollImpactTest3D.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace MatterEngine {

namespace {

constexpr float MinimumDurationSeconds = 1.0f / 120.0f;

float finiteOr(float value, float fallback) {
    return std::isfinite(value) ? value : fallback;
}

} // namespace

void RagdollImpactTest3D::setRunning(bool running) {
    if (m_running == running) return;
    m_running = running;
    if (running) {
        // Comecar deve produzir uma resposta imediatamente; o operador nao
        // precisa esperar o primeiro intervalo para descobrir se configurou
        // a direcao correta.
        m_countdownSeconds = 0.0f;
        m_activeSeconds = 0.0f;
    } else {
        m_activeSeconds = 0.0f;
        m_output.applying = false;
        m_output.forceNewtons = 0.0f;
    }
}

void RagdollImpactTest3D::triggerNow() {
    m_triggerRequested = true;
}

void RagdollImpactTest3D::reset(std::uint32_t randomSeed) {
    m_output = {};
    m_activeMode = m_config.mode;
    m_running = false;
    m_triggerRequested = false;
    m_activeSeconds = 0.0f;
    m_countdownSeconds = 0.0f;
    m_activeForceNewtons = 0.0f;
    m_activeDirectionDegrees = 0.0f;
    m_randomState = randomSeed == 0 ? 0x4D415454u : randomSeed;
    m_eventCount = 0;
}

float RagdollImpactTest3D::randomUnit() {
    // xorshift32: pequeno, reproduzivel e suficiente para um laboratorio.
    std::uint32_t value = m_randomState;
    value ^= value << 13u;
    value ^= value >> 17u;
    value ^= value << 5u;
    m_randomState = value == 0 ? 0x4D415454u : value;
    return static_cast<float>(m_randomState & 0x00FFFFFFu)
        / static_cast<float>(0x01000000u);
}

float RagdollImpactTest3D::randomRange(float minimum, float maximum) {
    if (minimum > maximum) std::swap(minimum, maximum);
    return minimum + (maximum - minimum) * randomUnit();
}

void RagdollImpactTest3D::sanitizeConfig() {
    m_config.forceNewtons = std::clamp(
        finiteOr(m_config.forceNewtons, 320.0f), 0.0f, 5'000.0f);
    m_config.directionDegrees = std::fmod(
        finiteOr(m_config.directionDegrees, 0.0f), 360.0f);
    if (m_config.directionDegrees < 0.0f) {
        m_config.directionDegrees += 360.0f;
    }
    m_config.intervalSeconds = std::clamp(
        finiteOr(m_config.intervalSeconds, 3.0f), 0.10f, 60.0f);
    m_config.pulseDurationSeconds = std::clamp(
        finiteOr(m_config.pulseDurationSeconds, 0.12f),
        MinimumDurationSeconds, 10.0f);

    m_config.randomMinimumForceNewtons = std::clamp(
        finiteOr(m_config.randomMinimumForceNewtons, 120.0f),
        0.0f, 5'000.0f);
    m_config.randomMaximumForceNewtons = std::clamp(
        finiteOr(m_config.randomMaximumForceNewtons, 420.0f),
        0.0f, 5'000.0f);
    if (m_config.randomMinimumForceNewtons
            > m_config.randomMaximumForceNewtons) {
        std::swap(m_config.randomMinimumForceNewtons,
            m_config.randomMaximumForceNewtons);
    }
    m_config.randomMinimumIntervalSeconds = std::clamp(
        finiteOr(m_config.randomMinimumIntervalSeconds, 1.5f),
        0.10f, 60.0f);
    m_config.randomMaximumIntervalSeconds = std::clamp(
        finiteOr(m_config.randomMaximumIntervalSeconds, 4.0f),
        0.10f, 60.0f);
    if (m_config.randomMinimumIntervalSeconds
            > m_config.randomMaximumIntervalSeconds) {
        std::swap(m_config.randomMinimumIntervalSeconds,
            m_config.randomMaximumIntervalSeconds);
    }
    m_config.randomMinimumDurationSeconds = std::clamp(
        finiteOr(m_config.randomMinimumDurationSeconds, 0.08f),
        MinimumDurationSeconds, 10.0f);
    m_config.randomMaximumDurationSeconds = std::clamp(
        finiteOr(m_config.randomMaximumDurationSeconds, 1.25f),
        MinimumDurationSeconds, 10.0f);
    if (m_config.randomMinimumDurationSeconds
            > m_config.randomMaximumDurationSeconds) {
        std::swap(m_config.randomMinimumDurationSeconds,
            m_config.randomMaximumDurationSeconds);
    }
}

void RagdollImpactTest3D::beginConfiguredPulse() {
    m_activeForceNewtons = m_config.forceNewtons;
    m_activeDirectionDegrees = m_config.directionDegrees;
    m_activeSeconds = m_config.pulseDurationSeconds;
    m_countdownSeconds = m_config.intervalSeconds;
    ++m_eventCount;
}

void RagdollImpactTest3D::beginRandomEvent() {
    m_activeForceNewtons = randomRange(
        m_config.randomMinimumForceNewtons,
        m_config.randomMaximumForceNewtons);
    m_activeDirectionDegrees = randomRange(0.0f, 360.0f);
    m_activeSeconds = randomRange(
        m_config.randomMinimumDurationSeconds,
        m_config.randomMaximumDurationSeconds);
    m_countdownSeconds = randomRange(
        m_config.randomMinimumIntervalSeconds,
        m_config.randomMaximumIntervalSeconds);
    ++m_eventCount;
}

void RagdollImpactTest3D::update(float deltaTime) {
    sanitizeConfig();
    const float dt = std::clamp(finiteOr(deltaTime, 0.0f), 0.0f, 0.25f);

    if (m_activeMode != m_config.mode) {
        m_activeMode = m_config.mode;
        m_activeSeconds = 0.0f;
        m_countdownSeconds = 0.0f;
        m_triggerRequested = false;
    }

    if (m_config.mode == RagdollImpactMode3D::Continuous) {
        m_activeForceNewtons = m_config.forceNewtons;
        m_activeDirectionDegrees = m_config.directionDegrees;
        if (m_triggerRequested) {
            ++m_eventCount;
            m_triggerRequested = false;
        }
        m_output.applying = m_running;
        m_output.forceNewtons = m_running ? m_activeForceNewtons : 0.0f;
        m_output.directionDegrees = m_activeDirectionDegrees;
        m_output.secondsRemaining = m_running ? -1.0f : 0.0f;
        m_output.secondsUntilNext = 0.0f;
        m_output.eventCount = m_eventCount;
        return;
    }

    m_activeSeconds = std::max(0.0f, m_activeSeconds - dt);
    m_countdownSeconds = std::max(0.0f, m_countdownSeconds - dt);
    const bool shouldStart = m_triggerRequested
        || (m_running && m_countdownSeconds <= 0.0f);
    if (shouldStart) {
        if (m_config.mode == RagdollImpactMode3D::Random) {
            beginRandomEvent();
        } else {
            beginConfiguredPulse();
        }
        m_triggerRequested = false;
    }

    m_output.applying = m_activeSeconds > 0.0f;
    m_output.forceNewtons = m_output.applying
        ? m_activeForceNewtons : 0.0f;
    m_output.directionDegrees = m_activeDirectionDegrees;
    m_output.secondsRemaining = m_activeSeconds;
    m_output.secondsUntilNext = m_countdownSeconds;
    m_output.eventCount = m_eventCount;
}

} // namespace MatterEngine
