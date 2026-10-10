#include "Engine/Athlete/Sensing/BallPredictor3D.hpp"

#include <algorithm>
#include <cmath>

namespace MatterEngine {
namespace {

void advance(BallPrediction3D& ball, float groundHeight, float radius, float h, const BallPredictorSettings3D& s) {
    const float floor = groundHeight + radius;
    ball.grounded = ball.position.z <= floor + 0.005f && ball.velocity.z <= 0.3f;
    if (ball.grounded) {
        // Rolando: so freia no plano.
        ball.position.z = floor;
        ball.velocity.z = 0.0f;
        const Vec3 flat { ball.velocity.x, ball.velocity.y, 0.0f };
        const float speed = flat.length();
        if (speed > 1e-4f) {
            const float slow = (s.rollingDeceleration + s.rollingDragPerSpeed * speed) * h;
            const float next = std::max(0.0f, speed - slow);
            ball.velocity.x *= next / speed;
            ball.velocity.y *= next / speed;
        }
        ball.position += Vec3 { ball.velocity.x, ball.velocity.y, 0.0f } * h;
        return;
    }
    const float speed = ball.velocity.length();
    ball.velocity += Vec3 { 0.0f, 0.0f, -s.gravity } * h;
    if (speed > 1e-4f) ball.velocity -= ball.velocity * (s.airDragPerSpeed * speed * h);
    ball.position += ball.velocity * h;
    if (ball.position.z < floor) {
        ball.position.z = floor;
        if (ball.velocity.z < 0.0f) ball.velocity.z = -ball.velocity.z * s.restitution;
    }
}

} // namespace

BallPrediction3D predictBall3D(Vec3 position, Vec3 velocity, float groundHeight, float radius, float seconds,
    const BallPredictorSettings3D& settings) {
    BallPrediction3D ball { position, velocity, true };
    const float h = std::max(1e-3f, settings.step);
    float left = std::max(0.0f, seconds);
    while (left > 1e-6f) {
        const float step = std::min(h, left);
        advance(ball, groundHeight, radius, step, settings);
        left -= step;
    }
    return ball;
}

void predictBallTable3D(Vec3 position, Vec3 velocity, float groundHeight, float radius, float interval,
    float horizon, const BallPredictorSettings3D& settings, std::vector<BallPrediction3D>& table) {
    table.clear();
    BallPrediction3D ball { position, velocity, true };
    const float h = std::max(1e-3f, settings.step);
    const int count = static_cast<int>(std::ceil(std::max(0.0f, horizon) / std::max(1e-3f, interval)));
    table.push_back(ball);
    for (int k = 1; k <= count; ++k) {
        float left = interval;
        while (left > 1e-6f) {
            const float step = std::min(h, left);
            advance(ball, groundHeight, radius, step, settings);
            left -= step;
        }
        table.push_back(ball);
    }
}

} // namespace MatterEngine
