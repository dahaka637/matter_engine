#include "Engine/Athlete/Dribble/BodyForecast3D.hpp"

#include <algorithm>
#include <cmath>

namespace MatterEngine {
namespace {

Vec3 planar(Vec3 v) { return { v.x, v.y, 0.0f }; }

Vec3 moveToward(Vec3 from, Vec3 to, float most) {
    const Vec3 change = to - from;
    const float length = change.length();
    if (length <= most || length < 1e-6f) return to;
    return from + change * (most / length);
}

Vec3 rotateZ(Vec3 v, float angle) {
    const float c = std::cos(angle), s = std::sin(angle);
    return { c * v.x - s * v.y, s * v.x + c * v.y, v.z };
}

} // namespace

// Um passo da capsula no chao (a regra do controle de personagem, com o giro).
Vec3 bodyForecastCapsuleStep3D(Vec3 current, Vec3 desired, const BodyForecastInput3D& in, float dt) {
    const float speed = current.length();
    const float wanted = desired.length();
    if (wanted < 1e-4f)
        return speed > 1e-4f ? current * (std::max(0.0f, speed - in.deceleration * dt) / speed) : Vec3 {};
    if (speed < 0.3f) return moveToward(current, desired, in.acceleration * dt);
    const Vec3 from = current * (1.0f / speed);
    const Vec3 to = desired * (1.0f / wanted);
    const float angle = std::atan2(from.x * to.y - from.y * to.x, from.x * to.x + from.y * to.y);
    if (std::abs(angle) > in.reverseRadians) return current * (std::max(0.0f, speed - in.brake * dt) / speed);
    const float turn = std::clamp(angle, -in.turnRate * dt, in.turnRate * dt);
    const float rate = wanted > speed ? in.acceleration : in.deceleration;
    const float next = speed + std::clamp(wanted - speed, -rate * dt, rate * dt);
    return rotateZ(from, turn) * next;
}

void BodyForecast3D::build(const BodyForecastInput3D& input) {
    m_step = std::max(1e-3f, input.step);
    m_hipInBody = input.hipInBody;
    const int count = std::max(2, static_cast<int>(std::ceil(input.horizon / m_step)) + 1);
    m_samples.assign(static_cast<std::size_t>(count), {});
    Sample s;
    s.position = input.pelvisGround;
    s.velocity = planar(input.velocity);
    s.yaw = input.facingYaw;
    const Vec3 requested = planar(input.requested);
    float rate = input.facingRate;
    const float fade = std::exp(-m_step / std::max(0.02f, input.facingRateSeconds));
    for (int i = 0; i < count; ++i) {
        m_samples[static_cast<std::size_t>(i)] = s;
        // (A posicao pela velocidade media do passo.)
        const Vec3 desired = input.requestedTurnsWithFacing ? rotateZ(requested, s.yaw - input.facingYaw) : requested;
        const Vec3 next = bodyForecastCapsuleStep3D(s.velocity, desired, input, m_step);
        s.position += (s.velocity + next) * (0.5f * m_step);
        s.velocity = next;
        s.yaw += rate * m_step;
        rate *= fade;
    }
}

BodyForecast3D::Sample BodyForecast3D::at(float t) const {
    if (m_samples.empty()) return {};
    const float u = std::clamp(t, 0.0f, m_step * static_cast<float>(m_samples.size() - 1)) / m_step;
    const std::size_t a = std::min(static_cast<std::size_t>(u), m_samples.size() - 1);
    const std::size_t b = std::min(a + 1, m_samples.size() - 1);
    const float f = u - static_cast<float>(a);
    const Sample& p = m_samples[a];
    const Sample& q = m_samples[b];
    Sample r;
    r.position = p.position + (q.position - p.position) * f;
    r.velocity = p.velocity + (q.velocity - p.velocity) * f;
    r.yaw = p.yaw + (q.yaw - p.yaw) * f;
    return r;
}

Vec3 BodyForecast3D::pelvisGround(float t) const { return at(t).position; }
Vec3 BodyForecast3D::velocity(float t) const { return at(t).velocity; }
float BodyForecast3D::yaw(float t) const { return at(t).yaw; }

Vec3 BodyForecast3D::hip(int side, float t) const {
    return toWorld(m_hipInBody[static_cast<std::size_t>(std::clamp(side, 0, 1))], t);
}

Vec3 BodyForecast3D::toWorld(Vec3 body, float t) const {
    const Sample s = at(t);
    return s.position + rotateZ(body, s.yaw);
}

Vec3 BodyForecast3D::toBody(Vec3 world, float t) const {
    const Sample s = at(t);
    return rotateZ(world - s.position, -s.yaw);
}

Vec3 BodyForecast3D::directionToWorld(Vec3 body, float t) const { return rotateZ(body, at(t).yaw); }
Vec3 BodyForecast3D::directionToBody(Vec3 world, float t) const { return rotateZ(world, -at(t).yaw); }

} // namespace MatterEngine
