#include "Engine/Animation/MotionMatching/MotionMatchingTrajectory3D.hpp"

#include <algorithm>
#include <cmath>

namespace MatterEngine {

namespace {

Vec3 flat(Vec3 v) { return { v.x, v.y, 0.0f }; }
Vec3 rotateZ(float yaw, Vec3 v) {
    const float c = std::cos(yaw), s = std::sin(yaw);
    return { c * v.x - s * v.y, s * v.x + c * v.y, v.z };
}
// O rumo `from` andando ate `to` no maximo `step` rad.
float turnToward(float from, float to, float step) {
    const float d = std::remainder(to - from, 6.283185307f);
    return std::abs(d) <= step ? from + d : from + std::copysign(step, d);
}

} // namespace

std::vector<MotionMatchingTrajectoryPoint3D> predictMotionTrajectory3D(const MotionMatchingMovementSettings3D& settings,
    const MotionMatchingMovementState3D& start, std::span<const float> offsets) {
    std::vector<MotionMatchingTrajectoryPoint3D> out;
    out.reserve(offsets.size());
    MotionMatchingMovementState3D s = start;
    s.velocity = flat(s.velocity);
    const float step = std::max(1.0e-3f, settings.stepSeconds);
    float time = 0.0f;
    std::size_t next = 0;
    while (next < offsets.size() && offsets[next] <= 0.0f) ++next;
    while (next < offsets.size()) {
        const float dt = std::min(step, offsets[next] - time);
        // A posicao com a velocidade de antes (Euler explicito, como o PoseSearch).
        s.position = s.position + s.velocity * dt;
        s.desiredDirection = rotateZ(s.desiredDirectionYawRate * dt, s.desiredDirection);
        const Vec3 direction = flat(s.desiredDirection);
        const float directionLength = direction.length();
        if (directionLength < 1.0e-4f || s.desiredSpeed <= 0.0f) {
            // A frenagem em subpassos: atrito e desaceleracao contra a velocidade; para na inversao.
            float left = dt;
            while (left > 1.0e-6f && s.velocity.lengthSquared() > 1.0e-8f) {
                const float sub = std::min(std::clamp(settings.brakingSubStepSeconds, 1.0f / 75.0f, 1.0f / 20.0f), left);
                const Vec3 v = s.velocity;
                const Vec3 after = v + (v * -settings.brakingFriction - v.normalized() * settings.brakingDeceleration) * sub;
                s.velocity = dot(after, v) <= 0.0f ? Vec3 {} : after;
                left -= sub;
            }
        } else {
            const Vec3 unit = direction / directionLength;
            const float speed = s.velocity.length();
            s.velocity = s.velocity - (s.velocity - unit * speed) * std::min(dt * settings.groundFriction, 1.0f);
            s.velocity = s.velocity + unit * (settings.maxAcceleration * dt);
            const float now = s.velocity.length();
            if (now > s.desiredSpeed && now > 1.0e-6f) s.velocity = s.velocity * (s.desiredSpeed / now);
        }
        const float turn = settings.rotationRadiansPerSecond < 0.0f ? 1.0e9f : settings.rotationRadiansPerSecond * dt;
        if (s.hasDesiredFacing) {
            s.desiredFacingYaw += s.desiredFacingYawRate * dt;
            s.facingYaw = turnToward(s.facingYaw, s.desiredFacingYaw, turn);
        } else if (settings.orientToMovement && s.velocity.lengthSquared() > 1.0e-4f) {
            s.facingYaw = turnToward(s.facingYaw, std::atan2(s.velocity.y, s.velocity.x), turn);
        }
        time += dt;
        if (time >= offsets[next] - 1.0e-6f) {
            out.push_back({ offsets[next], s.position, s.facingYaw, s.velocity });
            ++next;
        }
    }
    return out;
}

void MotionMatchingTrajectoryHistory3D::record(float timeSeconds, Vec3 position, float yaw, Vec3 velocity) {
    if (!m_points.empty() && timeSeconds <= m_points.back().time) m_points.clear();
    m_points.push_back({ timeSeconds, position, yaw, velocity });
    while (m_points.size() > 2 && m_points.back().time - m_points[1].time >= m_maxSeconds) m_points.pop_front();
}

bool MotionMatchingTrajectoryHistory3D::sample(float offsetSeconds, MotionMatchingTrajectoryPoint3D& out) const {
    if (m_points.empty()) return false;
    const float t = m_points.back().time + offsetSeconds;
    const auto make = [&](const Record& r) { out = { offsetSeconds, r.position, r.yaw, r.velocity }; };
    if (t <= m_points.front().time) {
        make(m_points.front());
        return true;
    }
    for (std::size_t i = m_points.size() - 1; i > 0; --i) {
        const Record& a = m_points[i - 1];
        const Record& b = m_points[i];
        if (t < a.time) continue;
        const float span = b.time - a.time;
        const float u = span > 1.0e-6f ? (t - a.time) / span : 1.0f;
        out.offsetSeconds = offsetSeconds;
        out.position = a.position + (b.position - a.position) * u;
        out.velocity = a.velocity + (b.velocity - a.velocity) * u;
        out.yaw = a.yaw + std::remainder(b.yaw - a.yaw, 6.283185307f) * u;
        return true;
    }
    make(m_points.back());
    return true;
}

} // namespace MatterEngine
