#include "Engine/Athlete/Dribble/DribbleBall3D.hpp"

#include <algorithm>
#include <cmath>

namespace MatterEngine {

void DribbleBallPath3D::build(Vec3 position, Vec3 velocity, float groundHeight, float radius, float start,
    float horizon, const BallPredictorSettings3D& settings) {
    m_start = start;
    std::vector<BallPrediction3D> table;
    predictBallTable3D(position, velocity, groundHeight, radius, m_step, std::max(m_step, horizon), settings, table);
    m_positions.clear();
    m_velocities.clear();
    for (const BallPrediction3D& entry : table) {
        m_positions.push_back(entry.position);
        m_velocities.push_back(entry.velocity);
    }
}

Vec3 DribbleBallPath3D::position(float t) const {
    if (m_positions.empty()) return {};
    const float u = std::clamp(t - m_start, 0.0f, m_step * static_cast<float>(m_positions.size() - 1)) / m_step;
    const std::size_t a = std::min(static_cast<std::size_t>(u), m_positions.size() - 1);
    const std::size_t b = std::min(a + 1, m_positions.size() - 1);
    const float f = u - static_cast<float>(a);
    return m_positions[a] + (m_positions[b] - m_positions[a]) * f;
}

Vec3 DribbleBallPath3D::velocity(float t) const {
    if (m_velocities.empty()) return {};
    const float u = std::clamp(t - m_start, 0.0f, m_step * static_cast<float>(m_velocities.size() - 1)) / m_step;
    const std::size_t a = std::min(static_cast<std::size_t>(u), m_velocities.size() - 1);
    const std::size_t b = std::min(a + 1, m_velocities.size() - 1);
    const float f = u - static_cast<float>(a);
    return m_velocities[a] + (m_velocities[b] - m_velocities[a]) * f;
}

Vec3 dribbleTouchVelocity3D(const BallOutcomeRequest3D& intent, Vec3 ballVelocity, Vec3 bodyVelocity,
    Vec3 aheadVelocity, float ballRadius, const BallOutcomeSettings3D& settings) {
    return computeBallOutcome3D(intent, ballVelocity, bodyVelocity, aheadVelocity, ballRadius, settings).velocity;
}

} // namespace MatterEngine
