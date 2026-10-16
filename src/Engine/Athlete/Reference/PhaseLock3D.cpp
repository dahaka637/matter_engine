#include "Engine/Athlete/Reference/PhaseLock3D.hpp"

#include <algorithm>
#include <cmath>

namespace MatterEngine {

void PhaseLock3D::reset() { *this = PhaseLock3D {}; }

float PhaseLock3D::update(std::array<bool, 2> touching, float referencePhase, float referenceMovingShare,
    bool referenceActive, bool enabled, float deltaTime, const PhaseLockParams3D& params) {
    std::array<bool, 2> landed { false, false };
    for (std::size_t side = 0; side < 2; ++side) {
        landed[side] = touching[side] && m_footAir[side] >= params.touchdownAirSeconds;
        m_footAir[side] = touching[side] ? 0.0f : m_footAir[side] + deltaTime;
    }
    if (landed[0] != landed[1]) {
        m_bodyPhase = landed[0] ? 0.0f : 0.5f;
        m_valid = true;
        m_sinceTouchdown = 0.0f;
    }
    const auto wrap = [](float x) { return x - std::floor(x + 0.5f); };
    if (referenceActive && m_sinceTouchdown > 0.0f) {
        // (Entre os pousos, a fase do corpo anda no ritmo da referencia; um
        // salto dela para tras - raro: troca de variacao - so para a fase do
        // corpo naquele tick.)
        const float advance = wrap(referencePhase - m_referencePhase);
        m_bodyPhase = m_bodyPhase + std::max(0.0f, advance) - std::floor(m_bodyPhase + std::max(0.0f, advance));
    }
    m_sinceTouchdown += deltaTime;
    float rate = 1.0f;
    if (enabled && referenceActive && m_valid && m_sinceTouchdown < params.touchdownMemorySeconds
        && referenceMovingShare > params.movingShare) {
        const float error = wrap(m_bodyPhase - referencePhase);
        rate = std::clamp(1.0f + params.gain * error, params.minimumRate, params.maximumRate);
    }
    m_referencePhase = referencePhase;
    return rate;
}

} // namespace MatterEngine
