#include "Engine/Athlete/Ball/BallOutcome3D.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>

namespace MatterEngine {
namespace {

constexpr float Pi = 3.14159265f;

Vec3 planar(Vec3 v) { return { v.x, v.y, 0.0f }; }

// O toque: a forca da barra (e a da velocidade, correndo), a velocidade do
// corpo no rumo dele (a de agora ou a que ele vai ter), o que o corpo indo
// para o outro lado empresta, e o que fica da que a bola trazia (o V3,
// aprovado).
Vec3 dribbleTouchVelocity(Vec3 direction, float power, Vec3 bodyVelocity, Vec3 aheadVelocity, Vec3 ballVelocity,
    const BallOutcomeSettings3D& f) {
    direction = planar(direction);
    if (direction.lengthSquared() < 1e-6f) return planar(ballVelocity);
    direction = direction.normalized();
    const float bar = std::clamp(power, 0.0f, 1.0f);
    const Vec3 body = planar(bodyVelocity), ahead = planar(aheadVelocity);
    const float speed = std::max(body.length(), ahead.length());
    const float force = (f.touchMinimum + f.touchPower * std::pow(bar, std::max(0.05f, f.touchCurve)))
        + std::max(0.0f, f.speedPush) * speed;
    const float along = dot(body, direction);
    const float carry = std::max(0.0f, std::max(along, dot(ahead, direction))) * std::max(0.0f, f.carry);
    const Vec3 ball = planar(ballVelocity);
    const float ballAlong = dot(ball, direction);
    const float defined = carry + force;
    const Vec3 relative = ball - body;
    const Vec3 side = relative - direction * dot(relative, direction);
    // (Amortece o menor entre o que a bola vem contra o toque no mundo e em
    // relacao ao jogador - 21/10: a que corre junto com ele volta inteira, a
    // conduzida a frente nao amortece, a que vem contra ele ou foge dele parado
    // amortece.)
    const float against = std::min(std::max(0.0f, -ballAlong), std::max(0.0f, -dot(relative, direction)));
    float result = defined + f.touchAheadKeep * std::max(0.0f, ballAlong - defined)
        - std::max(0.0f, f.touchAgainstKeep) * against;
    // O poder de parada (24/10, pedido do usuario: facilitar parar a bola em
    // movimento): o toque mais fraco que a bola que vem contra ele corta
    // stopPower do que sobraria dela seguindo (1: ela morre no pe); o mais forte
    // inverte o rumo com o que sobra da forca, sem reforco. 0: o de antes.
    if (result < 0.0f) result *= 1.0f - std::clamp(f.stopPower, 0.0f, 1.0f);
    return direction * result + side * f.touchSideKeep;
}

float tune(const char* name, float fallback) {
    const char* value = std::getenv(name);
    return value != nullptr ? std::strtof(value, nullptr) : fallback;
}

BallOutcomeSettings3D buildDefaults() {
    BallOutcomeSettings3D f;
    f.touchMinimum = tune("T_BallBase", f.touchMinimum);
    f.touchPower = tune("T_BallPower", f.touchPower);
    f.touchCurve = tune("T_BallCurve", f.touchCurve);
    f.carry = tune("T_BallCarry", f.carry);
    f.speedPush = tune("T_BallSpeedPush", f.speedPush);
    f.stopPower = tune("T_BallStopPower", f.stopPower);
    f.touchSideKeep = tune("T_BallKeepSide", f.touchSideKeep);
    f.touchAheadKeep = tune("T_BallKeepAhead", f.touchAheadKeep);
    f.touchAgainstKeep = tune("T_BallKeepAgainst", f.touchAgainstKeep);
    f.shotMinimum = tune("T_ShotMin", f.shotMinimum);
    f.shotPower = tune("T_ShotPower", f.shotPower);
    f.shotCurve = tune("T_ShotCurve", f.shotCurve);
    f.shotElevationMinimum = tune("T_ShotElevMin", f.shotElevationMinimum);
    f.shotElevationPower = tune("T_ShotElev", f.shotElevationPower);
    f.shotHighElevation = tune("T_ShotHigh", f.shotHighElevation);
    f.shotLowElevation = tune("T_ShotLow", f.shotLowElevation);
    f.shotCurveSpin = tune("T_ShotCurveSpin", f.shotCurveSpin);
    f.shotCurveLaunch = tune("T_ShotCurveLaunch", f.shotCurveLaunch);
    f.shotBackSpin = tune("T_ShotBackSpin", f.shotBackSpin);
    f.shotTopSpin = tune("T_ShotTopSpin", f.shotTopSpin);
    f.shotSideKeep = tune("T_ShotKeepSide", tune("T_BallKeepSide", f.shotSideKeep));
    f.shotBounce = tune("T_BallKeepBounce", f.shotBounce);
    f.shotAheadKeep = tune("T_ShotKeepAhead", tune("T_BallKeepAhead", f.shotAheadKeep));
    f.receiveMinimum = tune("T_ReceiveMin", f.receiveMinimum);
    f.receivePower = tune("T_ReceivePower", f.receivePower);
    f.receiveCarry = tune("T_ReceiveCarry", f.receiveCarry);
    f.receiveClosing = tune("T_ReceiveClosing", f.receiveClosing);
    f.chestShare = tune("T_ChestShare", f.chestShare);
    f.chestStop = tune("T_ChestStop", f.chestStop);
    f.headShare = tune("T_HeadShare", f.headShare);
    f.raiseMinimum = tune("T_RaiseMin", f.raiseMinimum);
    f.raisePower = tune("T_RaisePower", f.raisePower);
    f.raiseCurve = tune("T_RaiseCurve", f.raiseCurve);
    f.raiseAhead = tune("T_RaiseAhead", f.raiseAhead);
    return f;
}

} // namespace

BallOutcomeSettings3D defaultBallOutcomeSettings3D() {
    static const BallOutcomeSettings3D defaults = buildDefaults();
    return defaults;
}

Vec3 keepBallMotion3D(Vec3 defined, Vec3 ballVelocity, float sideKeep, float bounce, float aheadKeep) {
    const float speed = defined.length();
    if (speed < 1e-4f) return defined + ballVelocity * sideKeep;
    const Vec3 normal = defined * (1.0f / speed);
    const float along = dot(ballVelocity, normal);
    const Vec3 side = ballVelocity - normal * along;
    // (O quique negativo amortece: nunca mais que a velocidade definida.)
    const float extra = along < 0.0f ? std::max(-speed, bounce * -along) : aheadKeep * std::max(0.0f, along - speed);
    return defined + normal * extra + side * sideKeep;
}

BallOutcome3D computeBallOutcome3D(const BallOutcomeRequest3D& request, Vec3 ballVelocity, Vec3 bodyVelocity,
    Vec3 aheadVelocity, float ballRadius, const BallOutcomeSettings3D& settings) {
    // O peito e a cabeca: o toque e o chute com o maximo (minimo + forca) ate
    // a fracao deles, o minimo igual - lido dos valores de agora.
    BallOutcomeSettings3D f = settings;
    if (request.part != BallBodyPart3D::Foot) {
        const float share = std::clamp(request.part == BallBodyPart3D::Chest ? f.chestShare : f.headShare, 0.0f, 1.0f);
        f.touchPower = std::max(0.0f, share * (f.touchMinimum + f.touchPower) - f.touchMinimum);
        f.shotPower = std::max(0.0f, share * (f.shotMinimum + f.shotPower) - f.shotMinimum);
        // (Sem efeito: nem curva nem giro. A cabeca manda para baixo: o gesto do
        // mouse todo para baixo, 25 graus abaixo do horizontal.)
        f.shotCurveSpin = f.shotBackSpin = f.shotTopSpin = f.shotCurveLaunch = 0.0f;
        if (request.part == BallBodyPart3D::Head) f.shotLowElevation = -25.0f;
    }
    BallOutcome3D out;
    const auto rolling = [&](Vec3 velocity) {
        return cross(Vec3 { 0.0f, 0.0f, 1.0f }, planar(velocity)) * (1.0f / std::max(0.05f, ballRadius));
    };
    // Levantar: para cima pela barra e, no rumo das teclas, a frente (sobre a
    // velocidade do corpo); sem tecla, reto para cima. Sem giro.
    if (request.kind == BallActionKind3D::Raise) {
        const float up = f.raiseMinimum
            + f.raisePower * std::pow(std::clamp(request.power, 0.0f, 1.0f), std::max(0.05f, f.raiseCurve));
        const Vec3 flat = planar(request.direction);
        const Vec3 ahead = flat.lengthSquared() > 1e-6f ? flat.normalized() * (f.raiseAhead * up) : Vec3 {};
        out.velocity = planar(bodyVelocity) + ahead + Vec3 { 0.0f, 0.0f, up };
        out.spin = {};
        return out;
    }
    // O peito: a bola para (chestStop do que ela trazia contra o corpo) e
    // sai so o empurrao da fracao - o toque no rumo das teclas (sem tecla,
    // cai a frente dele), o chute na mira, baixo.
    if (request.part == BallBodyPart3D::Chest) {
        const Vec3 relative = ballVelocity - planar(bodyVelocity);
        const Vec3 stopped = planar(bodyVelocity) + relative * (1.0f - std::clamp(f.chestStop, 0.0f, 1.0f));
        const float bar = std::clamp(request.power, 0.0f, 1.0f);
        Vec3 push;
        if (request.kind == BallActionKind3D::Shot) {
            const float speed = f.shotMinimum + f.shotPower * std::pow(bar, std::max(0.05f, f.shotCurve));
            const float elevation = (f.shotElevationMinimum + f.shotElevationPower * bar) * Pi / 180.0f;
            push = Vec3 { std::cos(request.aimYawRadians), std::sin(request.aimYawRadians), 0.0f } * (speed * std::cos(elevation))
                + Vec3 { 0.0f, 0.0f, speed * std::sin(elevation) };
        } else if (planar(request.direction).lengthSquared() > 1e-6f) {
            push = planar(request.direction).normalized()
                * (f.touchMinimum + f.touchPower * std::pow(bar, std::max(0.05f, f.touchCurve)));
        }
        out.velocity = stopped + push;
        out.spin = rolling(out.velocity);
        return out;
    }
    if (request.receive) {
        const Vec3 flat = planar(request.direction);
        const Vec3 rest = flat.lengthSquared() > 1e-6f
            ? flat.normalized() * (f.receiveMinimum + f.receivePower * std::clamp(request.power, 0.0f, 1.0f))
            : Vec3 {};
        out.velocity = planar(bodyVelocity) * std::clamp(f.receiveCarry, 0.0f, 1.0f) + rest;
        out.spin = rolling(out.velocity);
        return out;
    }
    if (request.kind == BallActionKind3D::Shot) {
        const float bar = std::clamp(request.power, 0.0f, 1.0f);
        const float base = f.shotElevationMinimum + f.shotElevationPower * bar;
        const float elevation = (request.lift >= 0.0f ? base + (f.shotHighElevation - base) * request.lift
            : base + (f.shotLowElevation - base) * -request.lift) * Pi / 180.0f;
        const float speed = f.shotMinimum + f.shotPower * std::pow(bar, std::max(0.05f, f.shotCurve));
        // Curva para a direita sai um pouco para a esquerda (e o contrario).
        const float yaw = request.aimYawRadians + request.curve * f.shotCurveLaunch * Pi / 180.0f;
        const Vec3 ahead { std::cos(yaw), std::sin(yaw), 0.0f };
        // (O poder de parada tambem no chute: o fraco amortece a bola que vem, o
        // forte a devolve com o quique.)
        const float stop = std::clamp(f.stopPower, 0.0f, 1.0f) * (1.0f - bar);
        out.velocity = keepBallMotion3D(ahead * (speed * std::cos(elevation))
                + Vec3 { 0.0f, 0.0f, speed * std::sin(elevation) },
            ballVelocity, f.shotSideKeep, f.shotBounce - stop, f.shotAheadKeep);
        // Giro: de lado (Magnus para o lado: giro -z leva para a direita) e
        // por cima (cima x frente e o giro de quem rola para a frente).
        const Vec3 topAxis = cross(Vec3 { 0.0f, 0.0f, 1.0f }, ahead);
        const float top = request.lift >= 0.0f ? -f.shotBackSpin * request.lift : f.shotTopSpin * -request.lift;
        out.spin = Vec3 { 0.0f, 0.0f, -f.shotCurveSpin * request.curve } + topAxis * top;
        return out;
    }
    const Vec3 direction = planar(request.direction).lengthSquared() > 1e-6f ? planar(request.direction).normalized()
        : Vec3 { 1.0f, 0.0f, 0.0f };
    // O toque de cabeca: sai no rumo pedido, um pouco para cima (o do pe
    // amortecia contra a bola que vinha e ela voltava por cima dele).
    if (request.part == BallBodyPart3D::Head) {
        const float speed = f.touchMinimum + f.touchPower * std::pow(std::clamp(request.power, 0.0f, 1.0f), std::max(0.05f, f.touchCurve));
        out.velocity = planar(bodyVelocity) + direction * speed + Vec3 { 0.0f, 0.0f, 0.25f * speed };
        out.spin = {};
        return out;
    }
    out.velocity = request.touchSpeed >= 0.0f ? direction * request.touchSpeed
        : dribbleTouchVelocity(direction, request.power, bodyVelocity, aheadVelocity, ballVelocity, f);
    // (Rolando: o giro de quem rola no rumo do toque.)
    out.spin = rolling(out.velocity);
    return out;
}

} // namespace MatterEngine
