#include "Engine/Athlete/Training/LocomotionReward3D.hpp"

#include <algorithm>
#include <cmath>

namespace MatterEngine {

namespace {

constexpr float Gravity = 9.81f;
constexpr float Degrees = 3.14159265359f / 180.0f;

float kernel(float error, float sigma) { return std::exp(-(error * error) / (sigma * sigma)); }

float finiteOr(float value) { return std::isfinite(value) ? value : 0.0f; }

} // namespace

std::array<float, LocomotionRewardTermCount3D> locomotionRewardTerms3D(const LocomotionStepMeasure3D& m,
    const LocomotionRewardWeights3D& w, float dt) {
    std::array<float, LocomotionRewardTermCount3D> terms {};
    const auto set = [&](LocomotionRewardTerm3D term, float value) {
        terms[static_cast<std::size_t>(term)] = finiteOr(value);
    };
    set(LocomotionRewardTerm3D::Velocity, w.velocity * kernel(m.velocityError, w.velocitySigma) * dt);
    set(LocomotionRewardTerm3D::Facing, w.facing * kernel(m.facingError, w.facingSigma) * dt);
    set(LocomotionRewardTerm3D::Style, m.styleValid ? w.style * kernel(m.styleError, w.styleSigma) * dt : 0.0f);
    // (Por pouso, nao por segundo.)
    set(LocomotionRewardTerm3D::Contact, w.contact * m.contactScore);
    set(LocomotionRewardTerm3D::Slip, w.slip * m.slip * dt);
    set(LocomotionRewardTerm3D::Work, w.work * (m.power / 50.0f) * dt);
    set(LocomotionRewardTerm3D::Smooth, w.smooth * m.jerk * dt);
    set(LocomotionRewardTerm3D::Assist, w.assist * m.assist * dt);
    set(LocomotionRewardTerm3D::Posture, w.posture * m.posture * dt);
    set(LocomotionRewardTerm3D::Residual, w.residual * m.residual * dt);
    set(LocomotionRewardTerm3D::Still, w.still * m.still * dt);
    set(LocomotionRewardTerm3D::Fall, m.fell ? w.fall : 0.0f);
    return terms;
}

float locomotionReward3D(const std::array<float, LocomotionRewardTermCount3D>& terms) {
    float reward = 0.0f;
    for (std::size_t i = 0; i < terms.size(); ++i) reward += i < LocomotionRewardPositiveTerms3D ? terms[i] : -terms[i];
    return reward;
}

float locomotionPostureExcess3D(const LocomotionPostureInput3D& input) {
    const Vec3 up = input.chestUp;
    if (!(up.z > 0.05f)) return 1.0f;
    // A inclinacao para a frente e para o lado (rad; + frente, + esquerda).
    const float forward = std::atan2(up.x, up.z);
    const float side = std::atan2(up.y, up.z);
    // A demanda por semieixo: a maior entre a medida e a do comando.
    const auto demand = [&](float measured, float commanded) { return std::max({ 0.0f, measured, commanded }); };
    const float ahead = demand(input.measuredAcceleration.x, input.commandAcceleration.x);
    const float behind = demand(-input.measuredAcceleration.x, -input.commandAcceleration.x);
    const float left = demand(input.measuredAcceleration.y, input.commandAcceleration.y);
    const float right = demand(-input.measuredAcceleration.y, -input.commandAcceleration.y);
    const auto open = [](float acceleration, float limitDegrees) {
        return std::min(std::atan(acceleration / Gravity), limitDegrees * Degrees);
    };
    // Os pisos e tetos de antes (o tronco inteiro): parado 0..15 graus, em
    // movimento o piso sobe a 3 e o teto cresce 1,5 graus por m/s.
    const float moving = std::clamp(input.speed, 0.0f, 1.0f);
    const float floor = (-6.0f + 3.0f * moving) * Degrees - open(behind, 18.0f);
    const float ceiling = (15.0f + 1.5f * input.speed) * Degrees + open(ahead, 25.0f);
    const float sideLeft = 5.0f * Degrees + open(left, 25.0f);
    const float sideRight = 5.0f * Degrees + open(right, 25.0f);
    // A cabeca torta no parado (18/10): a folga de 10 graus e do pescoco vivo.
    const float neck = std::max(0.0f, input.neckBend - 10.0f * Degrees);
    return std::max(0.0f, forward - ceiling) + std::max(0.0f, floor - forward) + std::max(0.0f, side - sideLeft)
        + std::max(0.0f, -side - sideRight) + neck;
}

} // namespace MatterEngine
