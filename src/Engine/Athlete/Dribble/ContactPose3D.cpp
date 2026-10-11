#include "Engine/Athlete/Dribble/ContactPose3D.hpp"

#include <algorithm>
#include <cmath>

namespace MatterEngine {
namespace {

constexpr float Pi = 3.14159265358979323846f;

Vec3 planar(Vec3 v) { return { v.x, v.y, 0.0f }; }

Vec3 unitPlanar(Vec3 v, Vec3 fallback) {
    const Vec3 p = planar(v);
    return p.lengthSquared() > 1e-8f ? p.normalized() : fallback;
}

Vec3 rotateZ(Vec3 v, float angle) {
    const float c = std::cos(angle), s = std::sin(angle);
    return { c * v.x - s * v.y, s * v.x + c * v.y, v.z };
}

float signedAngle(Vec3 from, Vec3 to) {
    return std::atan2(from.x * to.y - from.y * to.x, from.x * to.x + from.y * to.y);
}

// (O V3: a ponta do pe nos limites do quadril, em relacao a frente do corpo,
// + para fora da perna.)
constexpr float LacesPitch = 0.80f;
constexpr float LacesPitchPower = 0.15f;
// (Os giros que a perna do boneco faz de fato - o tornozelo nao gira de lado
// e a coxa gira +-45 graus: medidos nos toques da sonda, o pe chegava ate
// ~50 graus; pedir mais deixava a beira do pe longe da bola e a perna girava
// rapido demais na volta.)
constexpr float InsideTurnMaximum = 0.75f;
constexpr float OutsideTurnMaximum = 0.60f;

} // namespace

ContactFeet3D resolveContactFeet3D(const RagdollProfile3D& profile) {
    ContactFeet3D feet;
    feet.legs = resolveAthleteLegs3D(profile);
    if (!feet.legs.valid) return feet;
    feet.geometry = resolveAthleteFootGeometry3D(profile, feet.legs, 0.08f);
    if (!feet.geometry.valid) return feet;
    for (std::size_t side = 0; side < 2; ++side) {
        const std::size_t link = feet.legs.foot[side];
        const RagdollCapsuleDefinition3D& collider = profile.links[link].collider;
        if (collider.shape != RagdollColliderShape3D::Box) return feet;
        feet.boxCenter[side] = collider.localPosition;
        feet.boxOrientation[side] = collider.localOrientation;
        feet.boxHalf[side] = collider.boxHalfExtents;
        feet.legLength[side] = feet.legs.thighLength[side] + feet.legs.shinLength[side];
        // A caixa no referencial do pe plano olhando +x (mundo): o centro e o
        // tamanho dela nos eixos do mundo.
        const Quaternion flat = feet.geometry.flat[side];
        const Vec3 center = flat.rotate(collider.localPosition);
        Vec3 extent {};
        const Vec3 h = collider.boxHalfExtents;
        const Vec3 axes[3] = { flat.rotate(collider.localOrientation.rotate({ 1.0f, 0.0f, 0.0f })),
            flat.rotate(collider.localOrientation.rotate({ 0.0f, 1.0f, 0.0f })),
            flat.rotate(collider.localOrientation.rotate({ 0.0f, 0.0f, 1.0f })) };
        const float halves[3] = { h.x, h.y, h.z };
        for (int i = 0; i < 3; ++i) {
            extent.x += std::abs(axes[i].x) * halves[i];
            extent.y += std::abs(axes[i].y) * halves[i];
            extent.z += std::abs(axes[i].z) * halves[i];
        }
        // Lado de dentro: o pe esquerdo para -y (a direita do corpo), o
        // direito para +y.
        const float medial = side == 0 ? -1.0f : 1.0f;
        struct Part { Vec3 at; Vec3 normal; };
        const Part parts[5] = {
            // O peito do pe: em cima, um pouco a frente do meio.
            { { 0.25f * extent.x, 0.0f, extent.z }, { 0.0f, 0.0f, 1.0f } },
            // O arco: do lado de dentro, no meio do pe.
            { { 0.0f, medial * extent.y, -0.15f * extent.z }, { 0.0f, medial, 0.0f } },
            // Do lado de fora, mais para a frente (o quinto metatarso).
            { { 0.35f * extent.x, -medial * extent.y, -0.15f * extent.z }, { 0.0f, -medial, 0.0f } },
            // A sola sob a planta do pe.
            { { 0.35f * extent.x, 0.0f, -extent.z }, { 0.0f, 0.0f, -1.0f } },
            // O calcanhar: atras, na altura da sola.
            { { -extent.x, 0.0f, -0.35f * extent.z }, { -1.0f, 0.0f, 0.0f } },
        };
        static_assert(static_cast<int>(BallTouchSurface3D::Laces) == 0 && static_cast<int>(BallTouchSurface3D::Inside) == 1
            && static_cast<int>(BallTouchSurface3D::Outside) == 2 && static_cast<int>(BallTouchSurface3D::Sole) == 3
            && static_cast<int>(BallTouchSurface3D::Heel) == 4);
        for (std::size_t k = 0; k < 5; ++k) {
            feet.surfacePoint[side][k] = flat.conjugate().rotate(center + parts[k].at);
            feet.surfaceNormal[side][k] = flat.conjugate().rotate(parts[k].normal);
        }
        // A altura do tornozelo com o pe plano no chao.
        const float sole = std::min(flat.rotate(feet.geometry.heel[side]).z, flat.rotate(feet.geometry.toeBase[side]).z);
        feet.ankleHeight[side] = flat.rotate(feet.legs.ankleInFoot[side]).z - sole;
    }
    feet.valid = true;
    return feet;
}

Quaternion contactFootOrientation3D(const ContactFeet3D& feet, int side, Vec3 toe, float pitchDown) {
    const Vec3 heading = unitPlanar(toe, { 1.0f, 0.0f, 0.0f });
    const float yaw = std::atan2(heading.y, heading.x);
    return (Quaternion::fromAxisAngle({ 0.0f, 0.0f, 1.0f }, yaw) * Quaternion::fromAxisAngle({ 0.0f, 1.0f, 0.0f }, pitchDown)
        * feet.geometry.flat[static_cast<std::size_t>(std::clamp(side, 0, 1))])
        .normalized();
}

ContactPose3D contactPose3D(const ContactFeet3D& feet, const ContactRequest3D& r) {
    ContactPose3D pose;
    if (!feet.valid) return pose;
    const std::size_t s = static_cast<std::size_t>(std::clamp(r.side, 0, 1));
    const Vec3 up { 0.0f, 0.0f, 1.0f };
    const float out = r.side == 0 ? 1.0f : -1.0f;
    const Vec3 push = unitPlanar(r.push, { 1.0f, 0.0f, 0.0f });
    const Vec3 forward = unitPlanar(r.bodyForward, push);
    // O rumo da ponta do pe pela superficie, nos limites do quadril.
    const auto turned = [&](Vec3 toe, float outward, float inward) {
        const float angle = std::clamp(signedAngle(forward, toe) * out, -inward, outward);
        return rotateZ(forward, angle * out);
    };
    Vec3 toe = push;
    float pitch = 0.0f;
    switch (r.surface) {
    case BallTouchSurface3D::Laces:
        toe = push;
        // (Bola alta: o pe mais deitado, por baixo dela.)
        pitch = LacesPitch + LacesPitchPower * r.power - 0.25f * std::max(0.0f, r.lift);
        break;
    case BallTouchSurface3D::Inside:
        // O lado de dentro encara o empurrao: a ponta virada para fora.
        toe = turned(rotateZ(push, out * Pi * 0.5f), InsideTurnMaximum, 0.6f);
        pitch = -0.06f;
        break;
    case BallTouchSurface3D::Outside:
        // O de fora encara o empurrao: a ponta virada para dentro e um pouco
        // para baixo.
        toe = turned(rotateZ(push, -out * Pi * 0.5f), 0.35f, OutsideTurnMaximum);
        pitch = 0.28f;
        break;
    case BallTouchSurface3D::Sole:
        toe = forward;
        pitch = -0.22f;
        break;
    case BallTouchSurface3D::Heel:
        toe = forward;
        pitch = 0.15f;
        break;
    }
    if (r.style == ContactStyle3D::BehindOut) {
        toe = turned(rotateZ(push, -out * Pi * 0.5f), 0.45f, 0.30f);
        pitch = 0.85f;
    } else if (r.style == ContactStyle3D::BehindIn) {
        toe = forward;
        pitch = 0.05f;
    }
    pose.orientation = contactFootOrientation3D(feet, r.side, toe, pitch);
    const std::size_t k = static_cast<std::size_t>(r.surface);
    const Vec3 point = pose.orientation.rotate(feet.surfacePoint[s][k]);
    const Vec3 normal = pose.orientation.rotate(feet.surfaceNormal[s][k]);
    Vec3 contact;
    if (r.surface == BallTouchSurface3D::Sole) {
        // A sola em cima da bola, um pouco para o lado contrario ao puxar.
        contact = r.ball + (up * 0.94f - push * 0.34f) * r.radius;
    } else if (r.surface == BallTouchSurface3D::Laces) {
        // O peito do pe atras da bola, na altura do centro dela (na bola alta,
        // um pouco abaixo: o pe passa por baixo).
        contact = r.ball - push * r.radius - up * (0.04f * std::max(0.0f, r.lift));
    } else {
        // A face do pe encostada na bola, no ponto mais perto do centro.
        contact = r.ball - unitPlanar(normal, push) * r.radius;
    }
    pose.foot = contact - point;
    // A caixa do pe inteira fora da bola: o ponto da superficie encosta, mas
    // com o pe inclinado (o peito do pe a ~48 graus) a ponta entrava por baixo
    // dela ate 11 cm - o pe sai pela normal da separacao.
    for (int pass = 0; pass < 3; ++pass) {
        const Quaternion box = (pose.orientation * feet.boxOrientation[s]).normalized();
        const Vec3 center = pose.foot + pose.orientation.rotate(feet.boxCenter[s]);
        const Vec3 local = box.conjugate().rotate(r.ball - center);
        const Vec3 h = feet.boxHalf[s];
        const Vec3 nearest { std::clamp(local.x, -h.x, h.x), std::clamp(local.y, -h.y, h.y), std::clamp(local.z, -h.z, h.z) };
        const Vec3 away = local - nearest;
        const float distance = away.length();
        const float inside = r.radius - distance;
        if (inside <= 0.002f) break;
        pose.foot -= distance > 1e-4f ? box.rotate(away * (1.0f / distance)) * (inside + 0.002f) : push * (r.radius + 0.002f);
    }
    // A caixa do pe nunca dentro do chao: o pe sobe (o contato sobe na bola).
    pose.foot.z += contactGroundDeficit3D(feet, r.side, pose.foot, pose.orientation, r.groundHeight, 0.008f);
    // (Subindo, a ponta do pe inclinado voltava para dentro da bola: de novo
    // fora dela, agora so no plano - para tras no rumo do toque.)
    for (int pass = 0; pass < 4; ++pass) {
        const Quaternion box = (pose.orientation * feet.boxOrientation[s]).normalized();
        const Vec3 center = pose.foot + pose.orientation.rotate(feet.boxCenter[s]);
        const Vec3 local = box.conjugate().rotate(r.ball - center);
        const Vec3 h = feet.boxHalf[s];
        const Vec3 nearest { std::clamp(local.x, -h.x, h.x), std::clamp(local.y, -h.y, h.y), std::clamp(local.z, -h.z, h.z) };
        const float inside = r.radius - (local - nearest).length();
        if (inside <= 0.002f) break;
        Vec3 away = r.ball - (center + box.rotate(nearest));
        away.z = 0.0f;
        away = away.lengthSquared() > 1e-8f ? away.normalized() : push;
        pose.foot -= away * (inside + 0.002f);
    }
    pose.ankle = pose.foot + pose.orientation.rotate(feet.legs.ankleInFoot[s]);
    pose.surfacePoint = pose.foot + point;
    pose.surfaceNormal = normal;
    return pose;
}

BallTouchSurface3D contactSurface3D(int side, Vec3 push, Vec3 bodyForward, BallActionKind3D kind, bool moving,
    bool ballBehind, float curve) {
    const float out = side == 0 ? 1.0f : -1.0f;
    const float angle = signedAngle(unitPlanar(bodyForward, { 1.0f, 0.0f, 0.0f }), unitPlanar(push, bodyForward)) * out;
    if (ballBehind) return std::abs(angle) > 2.0f ? BallTouchSurface3D::Heel : BallTouchSurface3D::Sole;
    if (kind == BallActionKind3D::Shot) {
        if (std::abs(curve) >= 0.3f) {
            const bool towardLeft = curve < 0.0f;
            return towardLeft == (side == 1) ? BallTouchSurface3D::Inside : BallTouchSurface3D::Outside;
        }
        if (angle > 1.0f) return BallTouchSurface3D::Outside;
        if (angle < -1.0f) return BallTouchSurface3D::Inside;
        return BallTouchSurface3D::Laces;
    }
    const float back = moving ? 1.95f : 2.05f;
    if (std::abs(angle) > back) return BallTouchSurface3D::Sole;
    if (moving && std::abs(angle) <= 0.42f) return BallTouchSurface3D::Laces;
    if (angle > 0.5f) return BallTouchSurface3D::Outside;
    return BallTouchSurface3D::Inside;
}

bool contactStyle3D(int side, Vec3 ball, Vec3 pelvis, Vec3 forward, Vec3 push, bool moving, BallActionKind3D kind,
    float curve, ContactStyle3D& style, BallTouchSurface3D& surface) {
    const float out = side == 0 ? 1.0f : -1.0f;
    const bool shot = kind == BallActionKind3D::Shot;
    const Vec3 f = unitPlanar(forward, { 1.0f, 0.0f, 0.0f });
    const float along = dot(planar(ball - pelvis), f);
    const float angle = signedAngle(f, unitPlanar(push, f)) * out;
    style = ContactStyle3D::Front;
    bool behind = false;
    if (along < -0.06f) {
        // Atras: para tras, o calcanhar; para fora desta perna, ela vai para
        // tras e para fora; para dentro, a sola curta; para a frente, a sola
        // trazendo-a.
        if (std::abs(angle) > 2.2f) behind = true;
        else if (angle >= 0.9f) style = ContactStyle3D::BehindOut;
        else if (angle <= -0.9f) style = ContactStyle3D::BehindIn;
        else behind = true;
    }
    if (shot && style != ContactStyle3D::Front) return false;
    surface = contactSurface3D(side, push, f, kind, moving, behind, curve);
    if (style == ContactStyle3D::BehindOut) surface = BallTouchSurface3D::Outside;
    if (style == ContactStyle3D::BehindIn) surface = BallTouchSurface3D::Sole;
    if (behind && shot && surface != BallTouchSurface3D::Heel) return false;
    return true;
}

float contactFootGap3D(const ContactFeet3D& feet, int side, Vec3 footPosition, Quaternion footOrientation, Vec3 ball,
    float radius) {
    if (!feet.valid) return 10.0f;
    const std::size_t s = static_cast<std::size_t>(std::clamp(side, 0, 1));
    const Quaternion q = (footOrientation * feet.boxOrientation[s]).normalized();
    const Vec3 center = footPosition + footOrientation.rotate(feet.boxCenter[s]);
    const Vec3 local = q.conjugate().rotate(ball - center);
    const Vec3 h = feet.boxHalf[s];
    const Vec3 nearest { std::clamp(local.x, -h.x, h.x), std::clamp(local.y, -h.y, h.y), std::clamp(local.z, -h.z, h.z) };
    return std::max(0.0f, (local - nearest).length() - radius);
}

float contactGroundDeficit3D(const ContactFeet3D& feet, int side, Vec3 footPosition, Quaternion footOrientation,
    float groundHeight, float clearance) {
    if (!feet.valid) return 0.0f;
    const std::size_t s = static_cast<std::size_t>(std::clamp(side, 0, 1));
    const Quaternion q = (footOrientation * feet.boxOrientation[s]).normalized();
    const Vec3 center = footPosition + footOrientation.rotate(feet.boxCenter[s]);
    const Vec3 h = feet.boxHalf[s];
    float lowest = 1e9f;
    for (int i = 0; i < 8; ++i) {
        const Vec3 corner { (i & 1) ? h.x : -h.x, (i & 2) ? h.y : -h.y, (i & 4) ? h.z : -h.z };
        lowest = std::min(lowest, (center + q.rotate(corner)).z);
    }
    return std::max(0.0f, groundHeight + clearance - lowest);
}

} // namespace MatterEngine
