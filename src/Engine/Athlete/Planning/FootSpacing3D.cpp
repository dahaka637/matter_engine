#include "Engine/Athlete/Planning/FootSpacing3D.hpp"

#include <algorithm>
#include <cmath>

namespace MatterEngine {
namespace {

float smooth(float s) {
    s = std::clamp(s, 0.0f, 1.0f);
    return s * s * (3.0f - 2.0f * s);
}

} // namespace

// Os pontos mais perto entre dois segmentos no plano.
void closestPlanar(Vec3 a0, Vec3 a1, Vec3 b0, Vec3 b1, Vec3& pa, Vec3& pb) {
    a0.z = a1.z = b0.z = b1.z = 0.0f;
    const Vec3 d1 = a1 - a0, d2 = b1 - b0, r = a0 - b0;
    const float a = dot(d1, d1), e = dot(d2, d2), f = dot(d2, r);
    float s = 0.0f, t = 0.0f;
    if (a > 1e-9f || e > 1e-9f) {
        if (a <= 1e-9f) {
            t = std::clamp(f / e, 0.0f, 1.0f);
        } else {
            const float c = dot(d1, r);
            if (e <= 1e-9f) {
                s = std::clamp(-c / a, 0.0f, 1.0f);
            } else {
                const float b = dot(d1, d2), denominator = a * e - b * b;
                s = denominator > 1e-9f ? std::clamp((b * f - c * e) / denominator, 0.0f, 1.0f) : 0.0f;
                t = (b * s + f) / e;
                if (t < 0.0f) { t = 0.0f; s = std::clamp(-c / a, 0.0f, 1.0f); }
                else if (t > 1.0f) { t = 1.0f; s = std::clamp((b - c) / a, 0.0f, 1.0f); }
            }
        }
    }
    pa = a0 + d1 * s;
    pb = b0 + d2 * t;
}

bool footSpacingCrossed3D(const FootSole3D& moving, const FootSole3D& fixed, int side, Vec3 forward,
    const FootSpacingSettings3D& settings) {
    const Vec3 left { -forward.y, forward.x, 0.0f };
    const float sign = side == 0 ? 1.0f : -1.0f;
    const Vec3 apart = moving.center() - fixed.center();
    return dot(Vec3 { apart.x, apart.y, 0.0f }, left * sign) < -settings.wrongSideSlack;
}

Vec3 footSpacingPush3D(const FootSole3D& moving, const FootSole3D& fixed, int side, Vec3 forward, bool crossed,
    const FootSpacingSettings3D& settings) {
    const Vec3 left { -forward.y, forward.x, 0.0f };
    const float sign = side == 0 ? 1.0f : -1.0f;
    const Vec3 outward = left * sign;
    const Vec3 centers = moving.center() - fixed.center();
    const Vec3 apart { centers.x, centers.y, 0.0f };
    const float lateral = dot(apart, outward);
    const float along = dot(apart, forward);
    Vec3 push;
    // Cada pe do seu lado (o que ja estava do lado errado, so perto dele:
    // mais longe, ele contorna pelo disco).
    if (!crossed || lateral > -settings.wrongSideSlack) {
        const float near = 1.0f - smooth((std::abs(along) - settings.passZone)
            / std::max(0.01f, settings.fadeZone - settings.passZone));
        const float required = settings.lineGap + (settings.passGap - settings.lineGap) * near;
        if (lateral < required) push = outward * (required - lateral);
    }
    // As solas nunca encostam (pelo caminho mais curto para fora).
    Vec3 pa, pb;
    closestPlanar(moving.heel + push, moving.toe + push, fixed.heel, fixed.toe, pa, pb);
    const Vec3 gap = pa - pb;
    const float distance = gap.length();
    if (distance < settings.clearRadius) {
        const Vec3 direction = distance > 1e-4f ? gap * (1.0f / distance) : outward;
        push += direction * (settings.clearRadius - distance);
    }
    // A canela do outro: a linha da perna dele na altura da sola que passa
    // (no alto, a sola longe da do outro ainda batia na canela).
    if (fixed.leg && fixed.hip.z > fixed.ankle.z + 0.1f) {
        const Vec3 middle = moving.center();
        const float u = std::clamp((middle.z - fixed.ankle.z) / (fixed.hip.z - fixed.ankle.z), 0.0f, 1.0f);
        if (u > 0.05f) {
            const Vec3 point = fixed.ankle + (fixed.hip - fixed.ankle) * u;
            Vec3 a, b;
            closestPlanar(moving.heel + push, moving.toe + push, point, point, a, b);
            const Vec3 away = a - b;
            const float clearance = away.length();
            if (clearance < settings.legClearRadius) {
                const Vec3 direction = clearance > 1e-4f ? away * (1.0f / clearance) : outward;
                push += direction * (settings.legClearRadius - clearance);
            }
        }
    }
    return push;
}

Vec3 footSpacingStep3D(Vec3 applied, Vec3 wanted, float deltaTime, const FootSpacingSettings3D& settings) {
    const float dt = std::max(deltaTime, 1e-4f);
    Vec3 change = (wanted - applied) * std::min(1.0f, dt / std::max(0.005f, settings.pushSeconds));
    const float most = settings.maximumPushSpeed * dt;
    const float length = change.length();
    if (length > most && length > 1e-9f) change = change * (most / length);
    return applied + change;
}

} // namespace MatterEngine
