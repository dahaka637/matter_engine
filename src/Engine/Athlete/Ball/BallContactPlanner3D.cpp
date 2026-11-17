#include "Engine/Athlete/Ball/BallContactPlanner3D.hpp"

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <vector>

namespace MatterEngine {
namespace {

constexpr float Gravity = 9.81f;
constexpr float Step = 0.01f;
constexpr float Pi = 3.14159265f;

float wrapAngle(float angle) {
    while (angle > Pi) angle -= 2.0f * Pi;
    while (angle < -Pi) angle += 2.0f * Pi;
    return angle;
}

// A bola a cada Step s: voando (gravidade), quicando (55% do vertical, 90% do
// horizontal; abaixo de 1,2 m/s ela assenta) e rolando (freando).
struct BallPath {
    std::vector<Vec3> positions;
};

BallPath simulateBall(Vec3 position, Vec3 velocity, float radius, float groundZ, float rolling, float seconds) {
    BallPath path;
    const int steps = std::max(1, static_cast<int>(std::ceil(seconds / Step)));
    path.positions.reserve(static_cast<std::size_t>(steps) + 1);
    path.positions.push_back(position);
    const float rest = groundZ + radius;
    for (int i = 0; i < steps; ++i) {
        const bool flying = position.z > rest + 0.005f || velocity.z > 0.05f;
        if (flying) {
            velocity.z -= Gravity * Step;
            position = position + velocity * Step;
            if (position.z <= rest && velocity.z < 0.0f) {
                position.z = rest;
                velocity.z = -velocity.z > 1.2f ? -velocity.z * 0.55f : 0.0f;
                velocity.x *= 0.9f;
                velocity.y *= 0.9f;
            }
        } else {
            position.z = rest;
            velocity.z = 0.0f;
            const float speed = std::hypot(velocity.x, velocity.y);
            if (speed > 1e-4f) {
                const float slower = std::max(0.0f, speed - rolling * Step) / speed;
                velocity.x *= slower;
                velocity.y *= slower;
            }
            position = position + Vec3 { velocity.x, velocity.y, 0.0f } * Step;
        }
        path.positions.push_back(position);
    }
    return path;
}

Vec3 sampleBall(const BallPath& path, float seconds) {
    const float u = std::max(0.0f, seconds) / Step;
    const std::size_t a = std::min(path.positions.size() - 1, static_cast<std::size_t>(u));
    const std::size_t b = std::min(path.positions.size() - 1, a + 1);
    const float f = std::clamp(u - static_cast<float>(a), 0.0f, 1.0f);
    return path.positions[a] * (1.0f - f) + path.positions[b] * f;
}

// O pe da passada no instante t (a previsao do ciclo; fora dele, o de agora).
Vec3 forecastFoot(const BallContactLeg3D& leg, float t) {
    if (!leg.cycling) return leg.footNow;
    const float u = std::max(0.0f, t) / std::max(leg.forecastStep, 1e-3f);
    const std::size_t last = leg.forecast.size() - 1;
    const std::size_t a = std::min(last, static_cast<std::size_t>(u));
    const std::size_t b = std::min(last, a + 1);
    const float f = std::clamp(u - static_cast<float>(a), 0.0f, 1.0f);
    return leg.forecast[a] * (1.0f - f) + leg.forecast[b] * f;
}

// O pe no ar no instante t (no ciclo): os balancos de agora em diante.
bool swingingAt(const BallContactLeg3D& leg, float t) {
    if (!leg.cycling || leg.cycleSeconds <= 0.0f || leg.swingSeconds <= 0.0f) return false;
    float start = leg.plantedNow ? leg.secondsToLiftoff : leg.secondsToTouchdown - leg.swingSeconds;
    if (start < -leg.cycleSeconds) start = 0.0f;
    for (int k = 0; k < 4; ++k) {
        const float a = start + static_cast<float>(k) * leg.cycleSeconds;
        // (Ate um pouco depois do pouso: o toque no fim do balanco.)
        if (t >= a && t <= a + leg.swingSeconds + 0.03f) return true;
    }
    return false;
}

} // namespace

PredictedBody3D predictBodyMotion3D(Vec3 velocity, Vec3 wanted, float timeConstant, float headingRate, float seconds) {
    PredictedBody3D out;
    Vec3 v { velocity.x, velocity.y, 0.0f };
    const Vec3 w { wanted.x, wanted.y, 0.0f };
    const float tau = std::max(0.02f, timeConstant);
    const float wantedSpeed = std::hypot(w.x, w.y);
    const float wantedYaw = std::atan2(w.y, w.x);
    constexpr float Dt = 0.01f;
    for (float t = 0.0f; t < seconds - 1e-5f; t += Dt) {
        const float dt = std::min(Dt, seconds - t);
        float speed = std::hypot(v.x, v.y);
        float yaw = speed > 0.05f ? std::atan2(v.y, v.x) : wantedYaw;
        // (Acelerando, a resposta exponencial; freando, a desaceleracao constante
        // medida da capsula, ~19 m/s2 - do sprint ela para em ~0,4 s.)
        if (wantedSpeed < speed) speed = std::max(wantedSpeed, speed - 19.0f * dt);
        else speed += (wantedSpeed - speed) * (1.0f - std::exp(-dt / tau));
        // (Indo para o lado oposto: ele freia primeiro e o rumo vira no fim.)
        if (wantedSpeed > 0.05f) {
            const float turn = wrapAngle(wantedYaw - yaw);
            if (std::abs(turn) > 2.6f && std::hypot(v.x, v.y) > 0.6f) speed = std::max(0.0f, speed - 14.0f * dt);
            else yaw += std::clamp(turn, -headingRate * dt, headingRate * dt);
        }
        v = { std::cos(yaw) * speed, std::sin(yaw) * speed, 0.0f };
        out.travel = out.travel + v * dt;
    }
    out.velocity = v;
    return out;
}

Vec3 predictBallPosition3D(Vec3 position, Vec3 velocity, float radius, float groundZ, float rollingDeceleration,
    float seconds) {
    return sampleBall(simulateBall(position, velocity, radius, groundZ, rollingDeceleration, seconds), seconds);
}

BallContactPlan3D planBallContact3D(const BallContactQuery3D& q) {
    BallContactPlan3D best;
    // (XPLANO=1: as recusas, para a sonda.)
    static const bool report = std::getenv("XPLANO") != nullptr;
    int refusedHeight = 0, refusedAlong = 0, refusedSide = 0, refusedDeviation = 0;
    float closestDeviation = 1e9f;
    const float latest = std::max(q.earliest, q.latest);
    const BallPath ball = simulateBall(q.ballPosition, q.ballVelocity, q.ballRadius, q.feetBase,
        q.rollingDeceleration, latest + Step);
    const float turn = wrapAngle(q.targetYaw - q.yaw);
    const Vec3 forward0 { std::cos(q.yaw), std::sin(q.yaw), 0.0f };
    const Vec3 left0 { -forward0.y, forward0.x, 0.0f };
    for (float t = std::max(0.0f, q.earliest); t <= latest + 1e-4f; t += 0.02f) {
        // O corpo: a rapidez indo para a pedida, o rumo do movimento girando
        // para o pedido (a resposta medida da locomocao), o rumo do corpo
        // girando para o olhar.
        const PredictedBody3D motion = predictBodyMotion3D(q.bodyVelocity, q.wantedVelocity, q.velocityTimeConstant,
            q.headingRate, t);
        const Vec3 body = q.pelvis + motion.travel;
        const float yaw = q.yaw + std::clamp(turn, -q.yawRate * t, q.yawRate * t);
        const Vec3 forward { std::cos(yaw), std::sin(yaw), 0.0f };
        const Vec3 left { -forward.y, forward.x, 0.0f };
        const Vec3 at = sampleBall(ball, t);
        const Vec3 offset { at.x - body.x, at.y - body.y, 0.0f };
        const Vec3 inBody { dot(offset, forward), dot(offset, left), at.z - q.feetBase };
        if (inBody.z > q.highestBall) { ++refusedHeight; continue; }
        if (inBody.x < -0.30f || inBody.x > 0.75f) { ++refusedAlong; continue; }
        for (int side = 0; side < 2; ++side) {
            if (q.onlySide >= 0 && side != q.onlySide) continue;
            const BallContactLeg3D& leg = q.legs[static_cast<std::size_t>(side)];
            const float sign = side == 0 ? 1.0f : -1.0f;
            // (Sem cruzar: a bola no maximo 12 cm para o lado da outra perna.)
            const float out = sign * inBody.y;
            if (out < -0.12f || out > 0.62f) { ++refusedSide; continue; }
            // (17/11 14h40, o contato por plano) O alcance de verdade da perna, do
            // quadril (10 cm para o lado dela): rente ao chao uns 0,62 m em volta
            // dele, encolhendo com a altura da bola (o voleio a ~1 m, 0,36 m).
            const float height = std::max(0.0f, inBody.z);
            const float reach = height <= 0.30f ? 0.62f : std::max(0.20f, 0.62f - 0.37f * (height - 0.30f));
            if (std::hypot(inBody.x, inBody.y - sign * 0.10f) > reach) { ++refusedDeviation; continue; }
            // O meio da chuteira na passada (a origem do link e o tornozelo): o
            // quanto o pe sai dela pesa no custo (a passada natural e preferida).
            const Vec3 foot = forecastFoot(leg, t) + Vec3 { 0.10f, 0.0f, 0.0f };
            const float deviation = std::hypot(inBody.x - foot.x, inBody.y - foot.y);
            closestDeviation = std::min(closestDeviation, deviation);
            if (deviation > std::max(q.maxDeviation, 0.9f)) { ++refusedDeviation; continue; }
            // O pe no ar (no ciclo) toca natural; o apoiado precisa sair do chao.
            const float lifting = leg.cycling ? (swingingAt(leg, t) ? 0.0f : 0.15f) : 0.04f;
            // (O caminho do pe e de onde ele esta agora no mundo ate a bola: o
            // apoiado fica no lugar enquanto o corpo anda.)
            const Vec3 footNow = leg.footNow + Vec3 { 0.10f, 0.0f, 0.0f };
            const float travel = std::hypot(at.x - (q.pelvis.x + forward0.x * footNow.x + left0.x * footNow.y),
                at.y - (q.pelvis.y + forward0.y * footNow.x + left0.y * footNow.y));
            // (O menor: o pe no balanco ja vai com a passada - o desvio dela;
            // o apoiado sai de onde esta - o caminho.)
            const float needed = q.naturalContact + (q.footSpeed > 0.0f ? std::min(deviation, travel) / q.footSpeed : 0.0f);
            const float squeezed = std::max(0.0f, needed - t);
            // (Comprimir o gesto custa mais que esperar por ele: a perna rapida demais e
            // a "perna louca"; so a janela curta de verdade comprime.)
            const float cost = q.timeWeight * t + 0.9f * deviation + 0.4f * std::abs(out - 0.16f) + lifting
                + (q.timeWeight + 2.0f) * squeezed + 30.0f * squeezed * squeezed;
            static const bool trace = std::getenv("XPLANO_T") != nullptr;
            if (trace)
                std::printf("  cand t %.2f perna %d bola %+.2f %+.2f %.2f desvio %.2f caminho %.2f lev %.2f aperto %.2f custo %.3f\n", t,
                    side, inBody.x, inBody.y, inBody.z, deviation, travel, lifting, squeezed, cost);
            if (!best.valid || cost < best.cost) {
                best.valid = true;
                best.side = side;
                best.seconds = t;
                best.ballWorld = at;
                best.ballInBody = inBody;
                best.deviation = deviation;
                best.cost = cost;
            }
        }
    }
    if (report && !best.valid)
        std::printf("plano recusado: altura %d frente %d lado %d desvio %d (menor desvio %.2f)\n", refusedHeight, refusedAlong,
            refusedSide, refusedDeviation, closestDeviation);
    return best;
}

} // namespace MatterEngine
