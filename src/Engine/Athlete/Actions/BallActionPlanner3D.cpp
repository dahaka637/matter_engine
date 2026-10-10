#include "Engine/Athlete/Actions/BallActionPlanner3D.hpp"

#include <algorithm>
#include <cmath>

namespace MatterEngine {
namespace {

constexpr float Pi = 3.14159265f;

Vec3 planar(Vec3 v) { return { v.x, v.y, 0.0f }; }

Vec3 rotateZ(Vec3 v, float angle) {
    const float c = std::cos(angle), s = std::sin(angle);
    return { c * v.x - s * v.y, s * v.x + c * v.y, v.z };
}

float signedAngle(Vec3 from, Vec3 to) {
    return std::atan2(from.x * to.y - from.y * to.x, from.x * to.x + from.y * to.y);
}

// O link do pe com a ponta em `heading` e inclinado pitchDown (+ a ponta
// para baixo). (Os links do perfil olham para +x no modelo.)
Quaternion yawPitch(Vec3 heading, float pitchDown) {
    const float yaw = std::atan2(heading.y, heading.x);
    return (Quaternion::fromAxisAngle({ 0.0f, 0.0f, 1.0f }, yaw)
        * Quaternion::fromAxisAngle({ 0.0f, 1.0f, 0.0f }, pitchDown)).normalized();
}

// O quanto a caixa do pe entra no chao (m) nesta pose, com folga.
float groundDeficit(const BallFootBox3D& box, int side, Vec3 position, Quaternion orientation, float groundHeight,
    float clearance) {
    const std::size_t s = static_cast<std::size_t>(side);
    const Quaternion world = (orientation * box.orientation[s]).normalized();
    const Vec3 center = position + orientation.rotate(box.center[s]);
    const Vec3 axes[3] = { world.rotate({ 1.0f, 0.0f, 0.0f }), world.rotate({ 0.0f, 1.0f, 0.0f }),
        world.rotate({ 0.0f, 0.0f, 1.0f }) };
    const Vec3 half = box.half[s];
    const float low = center.z - std::abs(axes[0].z) * half.x - std::abs(axes[1].z) * half.y
        - std::abs(axes[2].z) * half.z;
    return std::max(0.0f, groundHeight + clearance - low);
}

} // namespace

BallFootBox3D resolveBallFootBox3D(const RagdollProfile3D& profile, const AthleteLegs3D& legs) {
    BallFootBox3D box;
    if (!legs.valid) return box;
    for (std::size_t side = 0; side < 2; ++side) {
        const std::size_t index = legs.foot[side];
        if (index >= profile.links.size()) return box;
        const auto& collider = profile.links[index].collider;
        if (collider.shape != RagdollColliderShape3D::Box) return box;
        box.center[side] = collider.localPosition;
        box.orientation[side] = collider.localOrientation.normalized();
        box.half[side] = collider.boxHalfExtents;
    }
    box.valid = true;
    return box;
}

void ballFootSurface3D(const BallFootBox3D& box, int side, BallTouchSurface3D surface, Vec3& point, Vec3& normal) {
    const std::size_t s = static_cast<std::size_t>(side);
    const Vec3 half = box.half[s];
    // O lado de dentro: o pe esquerdo para -y (a direita do corpo), o direito
    // para +y.
    const float medial = side == 0 ? -1.0f : 1.0f;
    Vec3 local;
    switch (surface) {
    case BallTouchSurface3D::Laces:
        // O peito do pe: em cima, um pouco a frente do meio.
        local = { half.x * 0.25f, 0.0f, half.z };
        normal = { 0.0f, 0.0f, 1.0f };
        break;
    case BallTouchSurface3D::Inside:
        // O arco: do lado de dentro, no meio do pe.
        local = { 0.0f, medial * half.y, -half.z * 0.15f };
        normal = { 0.0f, medial, 0.0f };
        break;
    case BallTouchSurface3D::Outside:
        // Do lado de fora, mais para a frente (o quinto metatarso).
        local = { half.x * 0.35f, -medial * half.y, -half.z * 0.15f };
        normal = { 0.0f, -medial, 0.0f };
        break;
    case BallTouchSurface3D::Sole:
        local = { half.x * 0.35f, 0.0f, -half.z };
        normal = { 0.0f, 0.0f, -1.0f };
        break;
    case BallTouchSurface3D::Heel:
        local = { -half.x, 0.0f, -half.z * 0.35f };
        normal = { -1.0f, 0.0f, 0.0f };
        break;
    }
    point = box.center[s] + box.orientation[s].rotate(local);
    normal = box.orientation[s].rotate(normal);
}

BallTouchSurface3D chooseBallSurface3D(int side, Vec3 push, Vec3 bodyForward, BallActionKind3D kind, bool moving,
    bool ballBehind, float curve) {
    // O angulo do empurrao no corpo, + para fora da perna que joga.
    const float out = side == 0 ? 1.0f : -1.0f;
    const float angle = signedAngle(bodyForward, push) * out;
    // A bola atras do corpo: para tras, de calcanhar; para a frente, a sola
    // por cima dela, trazendo-a.
    if (ballBehind) return std::abs(angle) > 2.0f ? BallTouchSurface3D::Heel : BallTouchSurface3D::Sole;
    if (kind == BallActionKind3D::Shot) {
        // Com efeito: curva para o lado de dentro do pe, a parte de dentro;
        // para o de fora, a trivela.
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

void ballContactFootPose3D(const BallFootBox3D& box, int side, BallTouchSurface3D surface, Vec3 ball, float radius,
    Vec3 push, Vec3 bodyForward, float power, float lift, float groundHeight, Vec3& position, Quaternion& orientation) {
    const Vec3 up { 0.0f, 0.0f, 1.0f };
    const float out = side == 0 ? 1.0f : -1.0f;
    push = planar(push).lengthSquared() > 1e-6f ? planar(push).normalized() : planar(bodyForward).normalized();
    bodyForward = planar(bodyForward).normalized();
    // O rumo da ponta do pe pela superficie, nos limites do quadril (em
    // relacao ao rumo do corpo: + para fora).
    const auto turned = [&](Vec3 toe, float outward, float inward) {
        const float angle = std::clamp(signedAngle(bodyForward, toe) * out, -inward, outward);
        return rotateZ(bodyForward, angle * out);
    };
    Vec3 toe = push;
    float pitch = 0.0f;
    switch (surface) {
    case BallTouchSurface3D::Laces:
        toe = push;
        // (Bola alta: o pe mais deitado, por baixo dela.)
        pitch = 0.80f + 0.15f * power - 0.25f * std::max(0.0f, lift);
        break;
    case BallTouchSurface3D::Inside:
        // O lado de dentro encara o empurrao: a ponta virada para fora.
        toe = turned(rotateZ(push, out * Pi * 0.5f), 1.25f, 0.6f);
        pitch = -0.06f;
        break;
    case BallTouchSurface3D::Outside:
        // O de fora encara o empurrao: a ponta virada para dentro e um pouco
        // para baixo.
        toe = turned(rotateZ(push, -out * Pi * 0.5f), 0.35f, 0.80f);
        pitch = 0.28f;
        break;
    case BallTouchSurface3D::Sole:
        toe = bodyForward;
        pitch = -0.22f;
        break;
    case BallTouchSurface3D::Heel:
        toe = bodyForward;
        pitch = 0.15f;
        break;
    }
    orientation = yawPitch(toe, pitch);
    Vec3 point, normal;
    ballFootSurface3D(box, side, surface, point, normal);
    const Vec3 pointWorld = orientation.rotate(point);
    Vec3 contact;
    if (surface == BallTouchSurface3D::Sole) {
        // A sola em cima da bola, um pouco para o lado contrario ao puxar.
        contact = ball + (up * 0.94f - push * 0.34f) * radius;
    } else if (surface == BallTouchSurface3D::Laces) {
        // O peito do pe atras da bola, na altura do centro (na bola alta, um
        // pouco abaixo: o pe passa por baixo).
        contact = ball - push * radius - up * (0.04f * std::max(0.0f, lift));
    } else {
        // A face do pe encostada na bola, no ponto mais perto do centro.
        Vec3 n = planar(orientation.rotate(normal));
        n = n.lengthSquared() > 1e-6f ? n.normalized() : push;
        contact = ball - n * radius;
    }
    position = contact - pointWorld;
    // A caixa do pe nunca dentro do chao: o pe sobe.
    position.z += groundDeficit(box, side, position, orientation, groundHeight, 0.008f);
}

BallPlan3D planBallAction3D(const BallOutcomeRequest3D& intent, const BallPlannerBody3D& body,
    const std::vector<BallPrediction3D>& table, float tableInterval, const AthleteLegs3D& legs,
    const BallFootBox3D& box, float ballRadius, int previousSide, const BallPlannerSettings3D& settings) {
    BallPlan3D best;
    best.reason = "nenhum instante viavel";
    if (!legs.valid || !box.valid || table.empty()) {
        best.reason = "sem pernas ou sem bola";
        return best;
    }
    const Vec3 forward { std::cos(body.facingYaw), std::sin(body.facingYaw), 0.0f };
    const Vec3 left { -forward.y, forward.x, 0.0f };
    const bool shot = intent.kind == BallActionKind3D::Shot;
    const Vec3 push = shot ? Vec3 { std::cos(intent.aimYawRadians), std::sin(intent.aimYawRadians), 0.0f }
        : (planar(intent.direction).lengthSquared() > 1e-6f ? planar(intent.direction).normalized() : forward);
    const float power = std::clamp(intent.power, 0.0f, 1.0f);
    const float baseWindup = shot ? settings.shotWindup + settings.shotWindupPower * power
        : settings.touchWindup + settings.touchWindupPower * power;
    const float strike = shot ? settings.shotStrike + settings.shotStrikePower * power
        : settings.touchStrike + settings.touchStrikePower * power;
    const float horizon = shot ? settings.shotHorizon : settings.touchHorizon;
    // A perna do lado da bola (agora).
    const float ballSide = dot(planar(table.front().position - body.pelvis), left);
    const int preferred = ballSide >= 0.0f ? 0 : 1;
    best.cost = 1e9f;
    for (std::size_t k = 0; k < table.size(); ++k) {
        const float t = static_cast<float>(k) * tableInterval;
        if (t > horizon + 1e-4f) break;
        const BallPrediction3D& ball = table[k];
        // A bola no alcance de uma perna (a cintura, no maximo).
        if (ball.position.z - body.groundHeight > 0.75f) continue;
        for (int side = 0; side < 2; ++side) {
            const std::size_t s = static_cast<std::size_t>(side);
            const int support = 1 - side;
            // O apoio fica no chao ate o contato.
            if (body.plantedUntil[static_cast<std::size_t>(support)] < t + 0.03f) continue;
            const Vec3 pelvisThen = body.pelvis + Vec3 { body.velocity.x, body.velocity.y, 0.0f } * t;
            const Vec3 relative = planar(ball.position - pelvisThen);
            const bool behind = dot(relative, forward) < -0.06f;
            const BallTouchSurface3D surface = chooseBallSurface3D(side, push, forward, intent.kind, body.moving,
                behind, intent.curve);
            if (shot && behind) continue;
            Vec3 footPosition;
            Quaternion footOrientation;
            ballContactFootPose3D(box, side, surface, ball.position, ballRadius, push, forward, power, intent.lift,
                body.groundHeight, footPosition, footOrientation);
            // O tempo da perna: do momento em que o pe pode sair ate o contato
            // cabe o armado (mais longo com o pe longe do contato) e a batida
            // (um armado curto, nunca zero).
            const Vec3 footThen = body.foot[s] + Vec3 { body.velocity.x, body.velocity.y, 0.0f } * (body.moving ? t : 0.0f);
            const float windup = std::max(baseWindup,
                (footPosition - footThen).length() / std::max(0.5f, settings.windupTravelSpeed) - strike);
            const float lead = t - std::max(0.0f, body.freeFrom[s]);
            if (lead < 0.6f * windup + strike) continue;
            // Sem cruzar: o pe que joga fica do lado dele do de apoio.
            const Vec3 supportFoot = body.foot[static_cast<std::size_t>(support)]
                + Vec3 { body.velocity.x, body.velocity.y, 0.0f } * (body.moving ? t : 0.0f);
            const float lateral = dot(planar(footPosition - supportFoot), left) * (side == 0 ? 1.0f : -1.0f);
            if (lateral < settings.sideGap * 0.5f) continue;
            // O alcance: do quadril ao tornozelo.
            const Vec3 hip = body.hip[s] + Vec3 { body.velocity.x, body.velocity.y, 0.0f } * t;
            const Vec3 ankle = footPosition + footOrientation.rotate(legs.ankleInFoot[s]);
            const float length = legs.thighLength[s] + legs.shinLength[s];
            const float distance = (ankle - hip).length();
            const float need = std::max(0.0f, distance - settings.reachLimit * length);
            if (need > settings.pelvisReach || distance < settings.minimumReach) continue;
            const float reach = distance / length;
            // O tempo: apressar abaixo da duracao natural do gesto custa caro;
            // esperar, pouco.
            const float natural = windup + strike + std::max(0.0f, body.freeFrom[s]);
            float cost = settings.costTime * (2.0f * std::max(0.0f, natural - t) / std::max(0.05f, natural)
                    + 0.3f * t / std::max(0.05f, horizon))
                + settings.costReach * std::pow(std::max(0.0f, reach - 0.85f), 2.0f)
                + settings.costPelvis * need / std::max(0.01f, settings.pelvisReach)
                + (side != preferred ? settings.costOtherSide : 0.0f)
                + (previousSide >= 0 && side != previousSide ? settings.costSwitch : 0.0f);
            if (cost < best.cost) {
                best.valid = true;
                best.reason = "";
                best.side = side;
                best.surface = surface;
                best.contactTime = t;
                best.ball = ball.position;
                best.push = push;
                best.footPosition = footPosition;
                best.footOrientation = footOrientation;
                best.windupSeconds = std::min(windup, lead - strike);
                best.strikeSeconds = strike;
                best.reach = reach;
                best.pelvisNeed = need;
                best.cost = cost;
            }
        }
    }
    return best;
}

} // namespace MatterEngine
