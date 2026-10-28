#include "Engine/Character/BodyReaction3D.hpp"

#include <algorithm>
#include <cmath>

namespace MatterEngine {

namespace {

// Um pendulo amortecido em volta do equilibrio que a aceleracao empurra (a
// inercia: acelerando para um lado, ele fica para o outro), integrado em
// Euler semi-implicito e com teto.
void pendulum(float& angle, float& rate, float acceleration, float hertz, float damping, float degreesPerAcceleration,
    float maximumDegrees, float deltaTime) {
    constexpr float Radians = 3.14159265f / 180.0f;
    const float limit = std::max(0.0f, maximumDegrees) * Radians;
    const float omega = 2.0f * 3.14159265f * std::max(0.1f, hertz);
    const float rest = std::clamp(-acceleration * degreesPerAcceleration * Radians, -limit, limit);
    rate += (-omega * omega * (angle - rest) - 2.0f * damping * omega * rate) * deltaTime;
    angle = std::clamp(angle + rate * deltaTime, -limit, limit);
    if (std::abs(angle) >= limit) rate = 0.0f;
}

} // namespace

void BodyReaction3D::step(float forwardAcceleration, float leftAcceleration, float deltaTime,
    const BodyReactionSettings3D& s) {
    const float dt = std::clamp(deltaTime, 0.0f, 0.05f);
    const float soft = std::max(0.1f, s.accelerationSoftLimit);
    forwardAcceleration = soft * std::tanh(forwardAcceleration / soft);
    leftAcceleration = soft * std::tanh(leftAcceleration / soft);
    pendulum(trunkPitch, trunkPitchRate, forwardAcceleration, s.trunkHertz, s.trunkDamping, s.trunkDegreesPerAcceleration,
        s.trunkMaximumDegrees, dt);
    pendulum(trunkRoll, trunkRollRate, leftAcceleration, s.trunkHertz, s.trunkDamping, s.trunkDegreesPerAcceleration,
        s.trunkMaximumDegrees, dt);
    pendulum(armSwing, armSwingRate, forwardAcceleration, s.armHertz, s.armDamping, s.armDegreesPerAcceleration,
        s.armMaximumDegrees, dt);
    pendulum(armSide, armSideRate, leftAcceleration, s.armHertz, s.armDamping, s.armDegreesPerAcceleration,
        s.armMaximumDegrees, dt);
}

} // namespace MatterEngine
