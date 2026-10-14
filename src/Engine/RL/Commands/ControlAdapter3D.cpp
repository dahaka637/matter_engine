#include "Engine/RL/Commands/ControlAdapter3D.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>

namespace MatterEngine {
namespace {

constexpr float Pi = std::numbers::pi_v<float>;

float wrap(float angle) {
    return std::atan2(std::sin(angle), std::cos(angle));
}

} // namespace

float SpeedCaps3D::cap(float relative, float sprintCap) const {
    const float table[5] = { front, diagonalFront, side, diagonalBack, back };
    const float u = std::abs(wrap(relative)) / (Pi / 4.0f);
    const int i = std::clamp(static_cast<int>(u), 0, 3);
    const float f = std::clamp(u - static_cast<float>(i), 0.0f, 1.0f);
    return sprintCap * (table[i] + (table[i + 1] - table[i]) * f);
}

std::array<float, 2> controlKeys3D(bool forward, bool back, bool left, bool right) {
    const float x = (forward ? 1.0f : 0.0f) - (back ? 1.0f : 0.0f);
    const float y = (left ? 1.0f : 0.0f) - (right ? 1.0f : 0.0f);
    const float length = std::hypot(x, y);
    if (length < 1.0e-6f) return { 0.0f, 0.0f };
    return { x / length, y / length };
}

BodyCommand3D ControlAdapter3D::command(const ControlSettings3D& settings, const ControlInput3D& input,
    ControlState3D& state, PlayerAttributes3D& player, float dt) {
    BodyCommand3D command;
    const float amount = std::min(1.0f, std::hypot(input.stickForward, input.stickLeft));
    player.capNow = settings.caps.cap(0.0f, player.sprintCap);
    if (amount > settings.deadZone && input.speed > 0.0f) {
        const float relative = std::atan2(input.stickLeft, input.stickForward);
        player.capNow = settings.caps.cap(relative, player.sprintCap);
        const float speed = std::min(input.speed * amount, player.capNow);
        const float direction = input.cameraHeading + relative;
        command.moveVelocity = Vec3 { std::cos(direction), std::sin(direction), 0.0f } * speed;
    }
    command.faceHeading = wrap(input.cameraHeading);
    command.faceRate = input.cameraRate;
    // O pulo: o aperto fica valendo ate sair do chao ou acabar a janela.
    if (state.jumpPending) {
        state.jumpAge += dt;
        if (state.jumpAge > settings.jumpWindowSeconds) state.jumpPending = false;
    }
    if (input.jumpPressed && !state.jumpPending) {
        state.jumpPending = true;
        state.jumpAge = 0.0f;
        state.jumpRise = input.jumpRise;
    }
    command.jumpPending = state.jumpPending;
    command.jumpRise = state.jumpPending ? state.jumpRise : 0.0f;
    command.jumpAge = state.jumpPending ? state.jumpAge : 0.0f;
    return command;
}

void ControlAdapter3D::jumpTookOff(ControlState3D& state) {
    state.jumpPending = false;
    state.jumpAge = 0.0f;
}

} // namespace MatterEngine
