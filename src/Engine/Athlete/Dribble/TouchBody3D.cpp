#include "Engine/Athlete/Dribble/TouchBody3D.hpp"

#include <algorithm>
#include <cmath>

namespace MatterEngine {
namespace {

// As chaves do V3 (BallControl3D::plan): quanto entra pela forca.
constexpr float TouchBodyBase = 0.45f;
constexpr float KickBodyBase = 0.35f;
// A variacao de cada toque: canais e punhos.
constexpr float ChannelVariation = 0.20f;
constexpr float WristVariation = 0.05f;

float hermite(float p0, float p1, float m0, float m1, float u) {
    const float u2 = u * u, u3 = u2 * u;
    return (2.0f * u3 - 3.0f * u2 + 1.0f) * p0 + (u3 - 2.0f * u2 + u) * m0 + (-2.0f * u3 + 3.0f * u2) * p1
        + (u3 - u2) * m1;
}

// Catmull-Rom pelos nos (inclinacao zero nas pontas), como no V3.
float bodyCurve(const float* times, const float* values, int count, float t) {
    if (t <= times[0]) return values[0];
    if (t >= times[count - 1]) return values[count - 1];
    int k = 0;
    while (k + 1 < count - 1 && t > times[k + 1]) ++k;
    const float h = std::max(1e-4f, times[k + 1] - times[k]);
    const float u = (t - times[k]) / h;
    const auto tangent = [&](int i) {
        if (i <= 0 || i >= count - 1) return 0.0f;
        return (values[i + 1] - values[i - 1]) / std::max(1e-4f, times[i + 1] - times[i - 1]);
    };
    return hermite(values[k], values[k + 1], tangent(k) * h, tangent(k + 1) * h, u);
}

// Um numero de -1 a 1 pela semente e pelo canal (o jeito deste toque).
float jitter(std::uint32_t seed, std::uint32_t channel) {
    std::uint32_t x = seed * 747796405u + channel * 2891336453u + 1u;
    x ^= x >> 16;
    x *= 0x7feb352du;
    x ^= x >> 15;
    x *= 0x846ca68bu;
    x ^= x >> 16;
    return static_cast<float>(x & 0xffffu) / 32767.5f - 1.0f;
}

} // namespace

TouchBodyPlan3D touchBodyPlan3D(const TouchPlan3D& plan, bool moving, std::uint32_t seed, float fadeSeconds) {
    TouchBodyPlan3D b;
    if (!plan.valid) return b;
    b.valid = true;
    b.side = plan.side;
    b.push = plan.push;
    b.moving = moving;
    b.shot = plan.shot;
    const float out = plan.side == 0 ? 1.0f : -1.0f;
    const float p = std::clamp(plan.power, 0.0f, 1.0f);
    const float amount = plan.shot ? KickBodyBase + (1.0f - KickBodyBase) * p : TouchBodyBase + (1.0f - TouchBodyBase) * p;
    const bool kickLike = plan.shot && plan.surface != BallTouchSurface3D::Heel && plan.surface != BallTouchSurface3D::Sole;
    TouchBodyKey3D& bw = b.windup;
    TouchBodyKey3D& bc = b.contact;
    TouchBodyKey3D& bf = b.follow;
    if (kickLike) {
        // Armando: o quadril do chute vai para tras, o tronco para tras e para
        // o lado do apoio, o braco do apoio abre e o do chute vem a frente;
        // batendo, a pelve gira para a bola e o tronco vem por cima dela.
        bw = { 0.025f, out * 0.30f, -0.04f, -out * 0.05f, 0.45f, -out * 0.40f, -0.10f, -out * 0.12f,
            { 0.16f, 0.12f, -0.38f }, { -0.06f, 0.44f, -0.06f }, 1.0f };
        bc = { 0.065f, -out * 0.06f, 0.05f, -out * 0.06f, 0.55f, out * 0.02f, 0.14f, -out * 0.16f,
            { -0.18f, 0.20f, -0.36f }, { 0.20f, 0.34f, -0.18f }, 1.0f };
        bf = { 0.040f, -out * 0.26f, 0.06f, -out * 0.04f, 0.40f, out * 0.06f, 0.20f, -out * 0.10f,
            { -0.30f, 0.16f, -0.30f }, { 0.30f, -0.02f, -0.30f }, 1.0f };
    } else if (plan.style == ContactStyle3D::BehindOut) {
        // Atras, para fora: o tronco inclina para a frente (contrapeso) e para
        // o lado do apoio, os bracos abrem bem.
        bw = { 0.050f, out * 0.10f, 0.12f, -out * 0.05f, 0.50f, -out * 0.05f, 0.26f, -out * 0.10f,
            { 0.10f, 0.34f, -0.24f }, { 0.10f, 0.38f, -0.20f }, 1.0f };
        bc = { 0.090f, out * 0.16f, 0.22f, -out * 0.08f, 0.55f, -out * 0.08f, 0.46f, -out * 0.14f,
            { 0.08f, 0.42f, -0.14f }, { 0.14f, 0.42f, -0.14f }, 1.0f };
        bf = { 0.070f, out * 0.10f, 0.16f, -out * 0.06f, 0.45f, -out * 0.05f, 0.34f, -out * 0.10f,
            { 0.08f, 0.38f, -0.18f }, { 0.12f, 0.38f, -0.18f }, 1.0f };
    } else if (plan.surface == BallTouchSurface3D::Sole || plan.surface == BallTouchSurface3D::Heel) {
        // A sola e o calcanhar: o corpo um pouco para tras, os bracos a frente
        // e abertos.
        bw = { 0.02f, 0.0f, -0.02f, 0.0f, 0.35f, 0.0f, -0.05f, -out * 0.04f,
            { 0.12f, 0.16f, -0.40f }, { 0.14f, 0.24f, -0.34f }, 1.0f };
        bc = { 0.04f, 0.0f, -0.03f, 0.0f, 0.45f, 0.0f, -0.08f, -out * 0.06f,
            { 0.16f, 0.18f, -0.38f }, { 0.16f, 0.28f, -0.30f }, 1.0f };
        bf = { 0.03f, 0.0f, -0.02f, 0.0f, 0.35f, 0.0f, -0.04f, -out * 0.04f,
            { 0.08f, 0.16f, -0.42f }, { 0.10f, 0.24f, -0.36f }, 1.0f };
        if (plan.style == ContactStyle3D::BehindIn) {
            bw.trunkBend = 0.10f;
            bc.trunkBend = 0.16f;
            bf.trunkBend = 0.10f;
            bw.pelvisPitch = bc.pelvisPitch = bf.pelvisPitch = 0.04f;
        }
    } else if (plan.surface == BallTouchSurface3D::Outside) {
        // De fora: a pelve fecha um pouco, o corpo inclina para o apoio, o
        // braco do apoio abre.
        bw = { 0.015f, -out * 0.06f, 0.0f, -out * 0.03f, 0.35f, out * 0.04f, 0.03f, -out * 0.06f,
            { 0.04f, 0.20f, -0.40f }, { 0.04f, 0.32f, -0.30f }, 1.0f };
        bc = { 0.030f, -out * 0.10f, 0.03f, -out * 0.05f, 0.45f, out * 0.06f, 0.07f, -out * 0.10f,
            { -0.06f, 0.24f, -0.36f }, { 0.10f, 0.36f, -0.24f }, 1.0f };
        bf = { 0.020f, -out * 0.08f, 0.03f, -out * 0.03f, 0.35f, out * 0.05f, 0.06f, -out * 0.07f,
            { -0.08f, 0.22f, -0.38f }, { 0.12f, 0.30f, -0.30f }, 1.0f };
    } else {
        // De dentro (e o peito do pe no toque): a pelve abre com o pe, os
        // ombros ficam na bola, o tronco vem por cima dela; os bracos abrem.
        const float open = plan.surface == BallTouchSurface3D::Inside ? 1.0f : 0.3f;
        bw = { 0.020f, out * 0.26f * open, 0.02f, -out * 0.03f, 0.35f, -out * 0.18f * open, 0.04f, -out * 0.07f,
            { -0.04f, 0.26f, -0.36f }, { 0.08f, 0.32f, -0.28f }, 1.0f };
        bc = { 0.040f, out * 0.20f * open, 0.04f, -out * 0.04f, 0.45f, -out * 0.14f * open, 0.10f, -out * 0.09f,
            { -0.10f, 0.28f, -0.34f }, { 0.14f, 0.34f, -0.24f }, 1.0f };
        bf = { 0.030f, out * 0.08f * open, 0.04f, -out * 0.03f, 0.35f, -out * 0.06f * open, 0.10f, -out * 0.06f,
            { -0.14f, 0.24f, -0.34f }, { 0.18f, 0.26f, -0.28f }, 1.0f };
    }
    std::uint32_t channel = 0;
    for (TouchBodyKey3D* key : { &bw, &bc, &bf }) {
        // (Cada toque do seu jeito: a mesma variacao nas tres chaves.)
        channel = 0;
        for (float* value : { &key->pelvisDrop, &key->pelvisYaw, &key->pelvisPitch, &key->pelvisRoll, &key->trunkTwist,
                 &key->trunkBend, &key->trunkSide })
            *value *= amount * (1.0f + ChannelVariation * jitter(seed, channel++));
        key->arms *= amount;
        for (Vec3* wrist : { &key->kickArm, &key->supportArm }) {
            *wrist += Vec3 { jitter(seed, channel), jitter(seed, channel + 1), jitter(seed, channel + 2) } * WristVariation;
            channel += 3;
        }
    }
    // (A descida que o alcance pediu vem a parte, direto - pela mola ela
    // chegava atrasada e o pe parava antes da bola.)
    // (Sem a pelve ir para o apoio quando so assim o pe alcanca a bola.)
    if (plan.noShift) bw.pelvisToSupport = bc.pelvisToSupport = bf.pelvisToSupport = 0.0f;
    const float follow = std::max(0.04f, plan.followEnd - plan.contact);
    b.times[0] = plan.start;
    b.times[1] = plan.start + 0.4f * std::max(0.02f, plan.contact - plan.start);
    b.times[2] = plan.contact;
    b.times[3] = plan.contact + 0.4f * follow;
    b.times[4] = plan.followEnd + std::max(0.02f, fadeSeconds);
    return b;
}

float touchBodyPose3D(const TouchBodyPlan3D& b, float now, float share, Vec3 forward, Vec3 toSupport, DribbleBody3D& out) {
    if (!b.valid || now >= b.times[4] || now <= b.times[0]) return 0.0f;
    const auto body = [&](float TouchBodyKey3D::*field) {
        const float values[5] = { 0.0f, b.windup.*field, b.contact.*field, b.follow.*field, 0.0f };
        return bodyCurve(b.times, values, 5, now) * share;
    };
    const auto wrist = [&](Vec3 TouchBodyKey3D::*field) {
        // (Constante antes da primeira e depois da ultima chave: o peso do
        // braco faz a mistura com a pose da locomocao.)
        const Vec3 a = b.windup.*field, c = b.contact.*field, f = b.follow.*field;
        const float xs[5] = { a.x, a.x, c.x, f.x, f.x };
        const float ys[5] = { a.y, a.y, c.y, f.y, f.y };
        const float zs[5] = { a.z, a.z, c.z, f.z, f.z };
        return Vec3 { bodyCurve(b.times, xs, 5, now), bodyCurve(b.times, ys, 5, now), bodyCurve(b.times, zs, 5, now) };
    };
    const float out1 = b.side == 0 ? 1.0f : -1.0f;
    out.pelvisDrop += body(&TouchBodyKey3D::pelvisDrop);
    out.pelvisYaw += body(&TouchBodyKey3D::pelvisYaw);
    out.pelvisPitch += body(&TouchBodyKey3D::pelvisPitch);
    out.pelvisRoll += body(&TouchBodyKey3D::pelvisRoll);
    out.pelvisShift += Vec3 { toSupport.x, toSupport.y, 0.0f } * body(&TouchBodyKey3D::pelvisToSupport)
        + b.push * body(&TouchBodyKey3D::pelvisForward);
    out.trunkTwist += body(&TouchBodyKey3D::trunkTwist);
    out.trunkBend += body(&TouchBodyKey3D::trunkBend);
    out.trunkSide += body(&TouchBodyKey3D::trunkSide);
    const Vec3 up { 0.0f, 0.0f, 1.0f };
    const Vec3 ahead = Vec3 { forward.x, forward.y, 0.0f }.lengthSquared() > 1e-6f
        ? Vec3 { forward.x, forward.y, 0.0f }.normalized() : Vec3 { 1.0f, 0.0f, 0.0f };
    const Vec3 left { -ahead.y, ahead.x, 0.0f };
    const Vec3 kick = wrist(&TouchBodyKey3D::kickArm);
    const Vec3 support = wrist(&TouchBodyKey3D::supportArm);
    const std::size_t kickSide = static_cast<std::size_t>(b.side & 1);
    out.wrist[kickSide] = ahead * kick.x + left * (kick.y * out1) + up * kick.z;
    out.wrist[1 - kickSide] = ahead * support.x - left * (support.y * out1) + up * support.z;
    // (Os bracos entram em no minimo 0,12 s; no toque andando, so um pouco:
    // o balanco da corrida continua.)
    const float arms = body(&TouchBodyKey3D::arms) * std::clamp((now - b.times[0]) / 0.12f, 0.0f, 1.0f)
        * (b.moving && !b.shot ? 0.35f : 1.0f);
    return std::clamp(arms, 0.0f, 1.0f);
}

} // namespace MatterEngine
