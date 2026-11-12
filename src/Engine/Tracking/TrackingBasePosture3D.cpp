#include "Engine/Tracking/TrackingBasePosture3D.hpp"

#include "Engine/Tracking/TrackingGetUp3D.hpp"

#include <algorithm>
#include <cmath>

namespace MatterEngine {
namespace {

constexpr float Pi = 3.14159265358979f;

float wrap(float a) {
    while (a > Pi) a -= 2.0f * Pi;
    while (a < -Pi) a += 2.0f * Pi;
    return a;
}

// Um ponto no quadro horizontal de um rumo: frente e lado.
std::array<float, 2> local(Vec3 v, float heading) {
    const float c = std::cos(heading), s = std::sin(heading);
    return { c * v.x + s * v.y, -s * v.x + c * v.y };
}

Vec3 up(const RagdollLinkDefinition3D& link, const PhysicsBodyState3D& state) {
    return (state.orientation * link.modelOrientation.conjugate()).normalized().rotate({ 0.0f, 0.0f, 1.0f });
}

} // namespace

float TrackingBasePosture3D::distance() const {
    const float b = baseMeters / BaseWidth, p = pelvisYaw / YawWidth, c = chestYaw / YawWidth, t = chestTilt / TiltWidth,
                d = std::max(0.0f, drop) / DropWidth;
    return std::sqrt(b * b + p * p + c * c + t * t + d * d);
}

float TrackingBasePosture3D::attraction() const {
    const float d = distance();
    return 1.0f / (1.0f + d * d);
}

// (11/11, E5-12) O rumo pelo eixo lateral do osso, projetado no chao: nao muda com a inclinacao para a frente nem para o lado. O
// trackingBodyYaw3D soma a frente e o "cima" - feito para o corpo deitado -, e no peito o "cima" segue a coluna: inclinar a
// coluna de lado mudava o giro medido, e o atrator pagava para entortar a coluna (a E5-9 e a E5-10 assentavam com a coluna 12
// graus de lado e o peito 28-49 graus fora). E o eixo da suite (paradas.giro).
static float lateralYaw(const RagdollLinkDefinition3D& link, const PhysicsBodyState3D& state) {
    const Quaternion r = (state.orientation * link.modelOrientation.conjugate()).normalized();
    const Vec3 lateral = r.rotate({ 0.0f, 1.0f, 0.0f });
    return std::abs(lateral.x) + std::abs(lateral.y) > 1e-6f ? std::atan2(lateral.y, lateral.x) - 0.5f * 3.14159265358979f : 0.0f;
}

TrackingBasePosture3D measureTrackingBasePosture3D(const RagdollProfile3D& profile, std::span<const PhysicsBodyState3D> body,
    std::span<const PhysicsBodyState3D> idle, std::uint32_t pelvis, std::uint32_t chest, std::array<std::uint32_t, 2> feet) {
    TrackingBasePosture3D out;
    const std::size_t n = std::min(body.size(), idle.size());
    if (pelvis >= n || chest >= n || feet[0] >= n || feet[1] >= n) return out;
    const float bh = lateralYaw(profile.links[pelvis], body[pelvis]);
    const float ih = lateralYaw(profile.links[pelvis], idle[pelvis]);
    float base = 0.0f;
    for (const std::uint32_t f : feet) {
        const auto a = local(body[f].position - body[pelvis].position, bh);
        const auto b = local(idle[f].position - idle[pelvis].position, ih);
        base += std::hypot(a[0] - b[0], a[1] - b[1]);
    }
    out.baseMeters = 0.5f * base;
    out.pelvisYaw = std::abs(wrap(bh - ih));
    out.chestYaw = std::abs(wrap(lateralYaw(profile.links[chest], body[chest]) - lateralYaw(profile.links[chest], idle[chest])));
    out.chestTilt = std::acos(std::clamp(dot(up(profile.links[chest], body[chest]), up(profile.links[chest], idle[chest])), -1.0f, 1.0f));
    out.drop = idle[pelvis].position.z - body[pelvis].position.z;
    return out;
}

} // namespace MatterEngine
