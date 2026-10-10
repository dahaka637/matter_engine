#include "Engine/Character/PoseInertializer3D.hpp"

#include <algorithm>
#include <cmath>

namespace MatterEngine {

namespace {
// Vetor de rotacao (log do quaternion).
Vec3 rotationVector(Quaternion q) {
    q = q.normalized();
    if (q.w < 0.0f) q = { -q.x, -q.y, -q.z, -q.w };
    const Vec3 v { q.x, q.y, q.z };
    const float n = v.length();
    if (n < 1e-7f) return {};
    return v * (2.0f * std::atan2(n, q.w) / n);
}
} // namespace

namespace {
// Vetor de rotacao (eixo * angulo) -> rotacao.
Quaternion fromRotationVector(Vec3 v) {
    const float angle = v.length();
    if (angle < 1e-6f) return Quaternion { 0.5f * v.x, 0.5f * v.y, 0.5f * v.z, 1.0f }.normalized();
    const float s = std::sin(0.5f * angle) / angle;
    return Quaternion { v.x * s, v.y * s, v.z * s, std::cos(0.5f * angle) };
}
// Um passo exato da mola criticamente amortecida (x para zero).
void decay(Vec3& x, Vec3& v, float omega, float dt) {
    const Vec3 j = v + x * omega;
    const float e = std::exp(-omega * dt);
    x = (x + j * dt) * e;
    v = (v - j * (omega * dt)) * e;
}
} // namespace

void PoseInertializer3D::start(const std::vector<Vec3>& coordinates, const std::vector<Vec3>& coordinateVelocities,
    Vec3 rootPosition, Quaternion rootOrientation, Vec3 rootVelocity, Vec3 rootAngularVelocity,
    float halfLifeSeconds, float durationSeconds) {
    m_from = coordinates;
    m_fromVelocity = coordinateVelocities;
    m_fromVelocity.resize(m_from.size());
    m_fromRoot = rootPosition;
    m_fromOrientation = rootOrientation.normalized();
    m_fromRootVelocity = rootVelocity;
    m_fromAngularVelocity = rootAngularVelocity;
    // Meia-vida de x(t) = (1 + w t) e^(-w t): w ~ 1,678 / meia-vida.
    m_omega = 1.678f / std::max(0.01f, halfLifeSeconds);
    m_duration = std::max(0.05f, durationSeconds);
    m_elapsed = 0.0f;
    m_ticks = 0;
    m_active = true;
}

void PoseInertializer3D::apply(std::vector<Vec3>& coordinates, Vec3& rootPosition, Quaternion& rootOrientation,
    float deltaTime) {
    if (!m_active) return;
    const float dt = std::max(1e-4f, deltaTime);
    const std::size_t count = std::min(coordinates.size(), m_from.size());
    if (m_ticks == 0) {
        // A diferenca e a velocidade do corpo (a da pose pedida entra no
        // proximo tick, quando ela e conhecida).
        m_offset.assign(coordinates.size(), {});
        m_offsetVelocity.assign(coordinates.size(), {});
        for (std::size_t i = 0; i < count; ++i) {
            m_offset[i] = m_from[i] - coordinates[i];
            m_offsetVelocity[i] = m_fromVelocity[i];
        }
        m_rootOffset = m_fromRoot - rootPosition;
        m_rootOffsetVelocity = m_fromRootVelocity;
        Quaternion target = rootOrientation.normalized();
        m_turnOffset = rotationVector(m_fromOrientation * target.conjugate());
        m_turnOffsetVelocity = m_fromAngularVelocity;
    }
    if (m_ticks > 0) {
        if (m_ticks == 1) {
            // Menos a velocidade da pose pedida (agora conhecida): o corpo
            // segue so com a propria velocidade. (So a velocidade: tirando
            // tambem o quanto ela andou, o movimento dela contava duas vezes
            // e a pelve voltava num quadro - -0,9 m/s correndo a 4,5.)
            for (std::size_t i = 0; i < count && i < m_previousTarget.size(); ++i)
                m_offsetVelocity[i] -= (coordinates[i] - m_previousTarget[i]) * (1.0f / dt);
            m_rootOffsetVelocity -= (rootPosition - m_previousRoot) * (1.0f / dt);
            m_turnOffsetVelocity -= rotationVector(rootOrientation.normalized() * m_previousOrientation.conjugate())
                * (1.0f / dt);
        }
    }
    // (Ja no primeiro: a pose de partida e a do fim do passo anterior.)
    m_elapsed += dt;
    for (std::size_t i = 0; i < count; ++i) decay(m_offset[i], m_offsetVelocity[i], m_omega, dt);
    decay(m_rootOffset, m_rootOffsetVelocity, m_omega, dt);
    decay(m_turnOffset, m_turnOffsetVelocity, m_omega, dt);
    m_previousTarget = coordinates;
    m_previousRoot = rootPosition;
    m_previousOrientation = rootOrientation.normalized();
    ++m_ticks;
    // No fim, o que sobra sai suave nos ultimos 30% (sem degrau).
    const float tail = 1.0f - std::clamp((m_elapsed / m_duration - 0.7f) / 0.3f, 0.0f, 1.0f);
    const float keep = tail * tail * (3.0f - 2.0f * tail);
    for (std::size_t i = 0; i < count; ++i) coordinates[i] += m_offset[i] * keep;
    rootPosition += m_rootOffset * keep;
    rootOrientation = (fromRotationVector(m_turnOffset * keep) * rootOrientation).normalized();
    if (m_elapsed >= m_duration) m_active = false;
}

} // namespace MatterEngine
