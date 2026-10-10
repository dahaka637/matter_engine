#include "Engine/Athlete/Ball/BallOutcome3D.hpp"

#include <algorithm>
#include <cmath>

namespace MatterEngine {
namespace {

constexpr float Pi = 3.14159265f;

Vec3 planar(Vec3 v) { return { v.x, v.y, 0.0f }; }

// O toque: a forca da barra (e a da velocidade, correndo), a velocidade do
// corpo no rumo dele (a de agora ou a que ele vai ter), o que o corpo indo
// para o outro lado empresta, e o que fica da que a bola trazia (o V3,
// aprovado).
Vec3 dribbleTouchVelocity(Vec3 direction, float power, Vec3 bodyVelocity, Vec3 aheadVelocity, Vec3 ballVelocity,
    bool sole, const BallOutcomeSettings3D& f) {
    direction = planar(direction);
    if (direction.lengthSquared() < 1e-6f) return planar(ballVelocity);
    direction = direction.normalized();
    const float bar = std::clamp(power, 0.0f, 1.0f);
    const Vec3 body = planar(bodyVelocity), ahead = planar(aheadVelocity);
    const float speed = std::max(body.length(), ahead.length());
    const float force = (sole ? f.soleMinimum + f.solePower * std::pow(bar, std::max(0.05f, f.soleCurve))
        : f.touchMinimum + f.touchPower * std::pow(bar, std::max(0.05f, f.touchCurve)))
        + std::max(0.0f, f.speedPush) * speed;
    const float along = dot(body, direction);
    const float carry = std::max(0.0f, std::max(along, dot(ahead, direction))) * std::max(0.0f, f.carry);
    const Vec3 ball = planar(ballVelocity);
    const float ballAlong = dot(ball, direction);
    const float reverse = std::max(0.0f, f.reverseBoost) * (std::max(0.0f, -along) + std::max(0.0f, -ballAlong));
    const float defined = carry + reverse + force;
    const Vec3 relative = ball - body;
    const Vec3 side = relative - direction * dot(relative, direction);
    const float extra = f.touchAheadKeep * std::max(0.0f, ballAlong - defined);
    return direction * (defined + extra) + side * f.touchSideKeep;
}

} // namespace

Vec3 keepBallMotion3D(Vec3 defined, Vec3 ballVelocity, float sideKeep, float bounce, float aheadKeep) {
    const float speed = defined.length();
    if (speed < 1e-4f) return defined + ballVelocity * sideKeep;
    const Vec3 normal = defined * (1.0f / speed);
    const float along = dot(ballVelocity, normal);
    const Vec3 side = ballVelocity - normal * along;
    const float extra = along < 0.0f ? bounce * -along : aheadKeep * std::max(0.0f, along - speed);
    return defined + normal * extra + side * sideKeep;
}

BallOutcome3D computeBallOutcome3D(const BallOutcomeRequest3D& request, Vec3 ballVelocity, Vec3 bodyVelocity,
    Vec3 aheadVelocity, float ballRadius, const BallOutcomeSettings3D& f) {
    BallOutcome3D out;
    if (request.kind == BallActionKind3D::Shot) {
        const float bar = std::clamp(request.power, 0.0f, 1.0f);
        const float base = f.shotElevationMinimum + f.shotElevationPower * bar;
        const float elevation = (request.lift >= 0.0f ? base + (f.shotHighElevation - base) * request.lift
            : base + (f.shotLowElevation - base) * -request.lift) * Pi / 180.0f;
        const float speed = f.shotMinimum + f.shotPower * std::pow(bar, std::max(0.05f, f.shotCurve));
        // Curva para a direita sai um pouco para a esquerda (e o contrario).
        const float yaw = request.aimYawRadians + request.curve * f.shotCurveLaunch * Pi / 180.0f;
        const Vec3 ahead { std::cos(yaw), std::sin(yaw), 0.0f };
        out.velocity = keepBallMotion3D(ahead * (speed * std::cos(elevation))
                + Vec3 { 0.0f, 0.0f, speed * std::sin(elevation) },
            ballVelocity, f.shotSideKeep, f.shotBounce, f.shotAheadKeep);
        // Giro: de lado (Magnus para o lado: giro -z leva para a direita) e
        // por cima (cima x frente e o giro de quem rola para a frente).
        const Vec3 topAxis = cross(Vec3 { 0.0f, 0.0f, 1.0f }, ahead);
        const float top = request.lift >= 0.0f ? -f.shotBackSpin * request.lift : f.shotTopSpin * -request.lift;
        out.spin = Vec3 { 0.0f, 0.0f, -f.shotCurveSpin * request.curve } + topAxis * top;
        return out;
    }
    const Vec3 direction = planar(request.direction).lengthSquared() > 1e-6f ? planar(request.direction).normalized()
        : Vec3 { 1.0f, 0.0f, 0.0f };
    out.velocity = request.touchSpeed >= 0.0f ? direction * request.touchSpeed
        : dribbleTouchVelocity(direction, request.power, bodyVelocity, aheadVelocity, ballVelocity,
            request.surface == BallTouchSurface3D::Sole, f);
    // (Rolando: o giro de quem rola no rumo do toque.)
    out.spin = cross(Vec3 { 0.0f, 0.0f, 1.0f }, planar(out.velocity)) * (1.0f / std::max(0.05f, ballRadius));
    return out;
}

} // namespace MatterEngine
