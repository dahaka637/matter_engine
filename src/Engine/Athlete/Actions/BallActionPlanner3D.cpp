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

// Onde o pe da passada estara em t (s): parado ate sair do chao, depois um
// Hermite ate o pouso (quase parado nele, como o pe que pousa).
Vec3 predictSwingFoot(const BallPlannerBody3D& body, std::size_t side, float t) {
    const float lift = std::max(0.0f, body.swingLift[side]);
    const float land = body.swingLandTime[side];
    const Vec3 from = body.swingFoot[side];
    if (t <= lift) return from;
    const Vec3 v0 = lift > 0.0f ? Vec3 {} : body.swingVelocity[side];
    const float T = std::max(0.02f, land - lift);
    const float u = std::clamp((t - lift) / T, 0.0f, 1.0f);
    const float u2 = u * u, u3 = u2 * u;
    return from * (2.0f * u3 - 3.0f * u2 + 1.0f) + v0 * (T * (u3 - 2.0f * u2 + u))
        + body.swingLand[side] * (-2.0f * u3 + 3.0f * u2);
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
    // A bola atras do corpo: para tras, de calcanhar; para a frente (ou para
    // dentro), a sola por cima dela, trazendo-a.
    // (Para o lado de fora da perna, o lado de fora do pe: a perna vai para
    // tras e para fora.)
    if (ballBehind) {
        if (std::abs(angle) > 2.0f) return BallTouchSurface3D::Heel;
        if (angle > 0.6f) return BallTouchSurface3D::Outside;
        return BallTouchSurface3D::Sole;
    }
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
    const float baseWindup = (shot ? settings.shotWindup + settings.shotWindupPower * power
        : settings.touchWindup + settings.touchWindupPower * power) * (body.moving ? settings.movingWindupShare : 1.0f);
    const float strike = shot ? settings.shotStrike + settings.shotStrikePower * power
        : settings.touchStrike + settings.touchStrikePower * power;
    const float horizon = shot ? settings.shotHorizon : settings.touchHorizon;
    // A perna do lado da bola (agora).
    const float ballSide = dot(planar(table.front().position - body.pelvis), left);
    const int preferred = ballSide >= 0.0f ? 0 : 1;
    best.cost = 1e9f;
    // Por que os candidatos caem (o motivo do desfecho quando nenhum serve).
    enum Cut { CutHigh, CutSwing, CutSupport, CutBehind, CutTime, CutCross, CutReach, CutPath, CutPlant, CutCount };
    std::array<int, CutCount> cuts {};
    static constexpr const char* CutReason[CutCount] = { "a bola acima da cintura",
        "o contato fora do balanco do pe", "o apoio no ar no contato", "a bola atras do corpo no chute",
        "sem tempo para o armado", "cruzaria as pernas", "fora do alcance da perna",
        "longe do caminho do pe na passada", "o pe de apoio nao chega ao lado da bola" };
    for (std::size_t k = 0; k < table.size(); ++k) {
        const float t = static_cast<float>(k) * tableInterval;
        if (t > horizon + 1e-4f) break;
        const BallPrediction3D& ball = table[k];
        // A bola no alcance de uma perna (a cintura, no maximo).
        if (ball.position.z - body.groundHeight > 0.75f) {
            ++cuts[CutHigh];
            continue;
        }
        // (Correndo, o chute vai pelas opcoes de passada de apoio; senao,
        // cada pe.)
        const bool plantMode = shot && body.plantStep && body.plantOptionCount > 0;
        const int candidates = plantMode ? body.plantOptionCount : 2;
        for (int c = 0; c < candidates; ++c) {
            const BallPlantOption3D* option = plantMode ? &body.plantOptions[static_cast<std::size_t>(c)] : nullptr;
            const int side = option != nullptr ? 1 - option->side : c;
            const std::size_t s = static_cast<std::size_t>(side);
            const int support = 1 - side;
            const std::size_t sup = static_cast<std::size_t>(support);
            const float freeFrom = option != nullptr ? option->kickFrom : body.freeFrom[s];
            const float freeUntil = option != nullptr ? option->kickUntil : body.freeUntil[s];
            // O pe que joga no balanco dele no contato; o apoio no chao.
            if (t > freeUntil + 1e-4f) {
                ++cuts[CutSwing];
                continue;
            }
            // (A passada de apoio: o apoio pousa antes da batida, sem ficar
            // preso demais.)
            const bool plant = option != nullptr;
            if (plant ? t < option->landTime + settings.plantLeadMin || t > option->landTime + settings.plantLeadMax
                      : body.plantedUntil[sup] < t + 0.03f || body.plantedFrom[sup] > t + 1e-4f) {
                ++cuts[CutSupport];
                continue;
            }
            // A passada de apoio ainda por pousar: a corrida que poe a pelve
            // no pouso natural atras do apoio (ao lado da bola), e o pouso do
            // estilo nessa corrida; no apoio, o corpo freia.
            float runSpeed = -1.0f;
            Vec3 plantLand;
            Vec3 pelvisThen = body.pelvis + Vec3 { body.velocity.x, body.velocity.y, 0.0f } * t;
            const Vec3 runDirection = planar(body.velocity).lengthSquared() > 0.01f ? planar(body.velocity).normalized()
                                                                                     : forward;
            const Vec3 plantOutward = left * (support == 0 ? 1.0f : -1.0f);
            if (plant && !option->landed) {
                const Vec3 beside = Vec3 { ball.position.x, ball.position.y, 0.0f } + plantOutward * settings.plantSideGap
                    - push * settings.plantBehind;
                const float distance = dot(beside - planar(body.pelvis), runDirection);
                const float leadPerSpeed = body.landLead.x / std::max(0.5f, body.landLeadSpeed);
                runSpeed = distance / std::max(0.05f, option->landTime + leadPerSpeed);
                runSpeed = std::clamp(runSpeed, settings.plantSpeedMin * body.runSpeed, settings.plantSpeedMax * body.runSpeed);
                const float lead = body.landLead.x * std::clamp(runSpeed / std::max(0.5f, body.landLeadSpeed), 0.5f, 2.0f);
                plantLand = planar(body.pelvis) + runDirection * (runSpeed * option->landTime + lead)
                    + plantOutward * body.landLead.y + Vec3 { 0.0f, 0.0f, body.groundHeight + body.landLead.z };
                const float travel = runSpeed * std::min(t, option->landTime)
                    + runSpeed * settings.plantBrake * std::max(0.0f, t - option->landTime);
                pelvisThen = body.pelvis + runDirection * travel;
            }
            const Vec3 relative = planar(ball.position - pelvisThen);
            const bool behind = dot(relative, forward) < -0.06f;
            const BallTouchSurface3D surface = chooseBallSurface3D(side, push, forward, intent.kind, body.moving,
                behind, intent.curve);
            if (shot && behind) {
                ++cuts[CutBehind];
                continue;
            }
            Vec3 footPosition;
            Quaternion footOrientation;
            ballContactFootPose3D(box, side, surface, ball.position, ballRadius, push, forward, power, intent.lift,
                body.groundHeight, footPosition, footOrientation);
            // O tempo da perna: do momento em que o pe pode sair ate o contato
            // cabe o armado (mais longo com o pe longe do contato) e a batida
            // (um armado curto, nunca zero).
            // (No toque no balanco nao ha armado: o pe so desvia da passada,
            // e o tempo e o do desvio.)
            const bool swingTouch = !shot && body.swingLandTime[s] > 0.0f;
            float swingPath = 0.0f;
            if (swingTouch) {
                const Vec3 path = predictSwingFoot(body, s, t);
                swingPath = std::hypot(footPosition.x - path.x, footPosition.y - path.y);
                if (swingPath > settings.swingDeviation) {
                    ++cuts[CutPath];
                    continue;
                }
            }
            const Vec3 footThen = body.foot[s] + Vec3 { body.velocity.x, body.velocity.y, 0.0f } * (body.moving ? t : 0.0f);
            const float travelSpeed = std::max(0.5f, settings.windupTravelSpeed);
            const float windup = swingTouch ? 0.0f
                                            : std::max(baseWindup, (footPosition - footThen).length() / travelSpeed - strike);
            const float lead = t - std::max(0.0f, freeFrom);
            // (Correndo, o chute arma no proprio balanco da passada: o pe sai
            // do chao atras do corpo.)
            const float needed = swingTouch ? std::max(0.5f * strike, swingPath / travelSpeed)
                : (body.moving ? 0.3f : 0.6f) * windup + strike;
            if (lead < needed) {
                ++cuts[CutTime];
                continue;
            }
            // A passada de apoio: o pe de apoio ao lado da bola, perto de onde
            // o estilo o pousaria.
            Vec3 plantTarget;
            float plantShift = 0.0f;
            if (plant && option->landed) {
                // (Ja pousado: fica onde esta.)
                plantTarget = option->landPosition;
            } else if (plant) {
                plantTarget = Vec3 { ball.position.x, ball.position.y, plantLand.z }
                    + plantOutward * settings.plantSideGap - push * settings.plantBehind;
                plantShift = std::hypot(plantTarget.x - plantLand.x, plantTarget.y - plantLand.y);
                const float stretch = dot(planar(plantTarget - plantLand), runDirection);
                if (plantShift > settings.plantShift || stretch > settings.plantStretch) {
                    ++cuts[CutPlant];
                    continue;
                }
            }
            // Sem cruzar: o pe que joga fica do lado dele do de apoio.
            const Vec3 supportFoot = plant ? plantTarget
                : body.foot[static_cast<std::size_t>(support)]
                    + Vec3 { body.velocity.x, body.velocity.y, 0.0f } * (body.moving ? t : 0.0f);
            const float lateral = dot(planar(footPosition - supportFoot), left) * (side == 0 ? 1.0f : -1.0f);
            if (lateral < settings.sideGap * 0.5f) {
                ++cuts[CutCross];
                continue;
            }
            // O alcance: do quadril ao tornozelo.
            const Vec3 hip = body.hip[s] + (pelvisThen - body.pelvis);
            const Vec3 ankle = footPosition + footOrientation.rotate(legs.ankleInFoot[s]);
            const float length = legs.thighLength[s] + legs.shinLength[s];
            const float distance = (ankle - hip).length();
            const float need = std::max(0.0f, distance - settings.reachLimit * length);
            if (need > settings.pelvisReach || distance < settings.minimumReach) {
                ++cuts[CutReach];
                continue;
            }
            const float reach = distance / length;
            // O tempo: apressar abaixo da duracao natural do gesto custa caro;
            // esperar, pouco.
            const float natural = (swingTouch ? needed : windup + strike) + std::max(0.0f, freeFrom);
            float cost = settings.costTime * (2.0f * std::max(0.0f, natural - t) / std::max(0.05f, natural)
                    + 0.3f * t / std::max(0.05f, horizon))
                + settings.costReach * std::pow(std::max(0.0f, reach - 0.85f), 2.0f)
                + settings.costPelvis * need / std::max(0.01f, settings.pelvisReach)
                + (side != preferred ? settings.costOtherSide : 0.0f)
                + (previousSide >= 0 && side != previousSide ? settings.costSwitch : 0.0f)
                + settings.costHold * std::max(0.0f, t - body.plantedNatural[static_cast<std::size_t>(support)]) / 0.05f
                + settings.costSwingPath * (swingPath / 0.15f) * (swingPath / 0.15f)
                + settings.costPlant * (plantShift / 0.2f) * (plantShift / 0.2f);
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
                // (O toque no balanco sai ja: o desvio da passada se espalha
                // ate o contato.)
                best.windupSeconds = swingTouch ? 0.0f : std::min(windup, lead - strike);
                best.strikeSeconds = swingTouch ? t : strike;
                best.reach = reach;
                best.pelvisNeed = need;
                best.cost = cost;
                best.plantSide = plant ? support : -1;
                best.plantTime = plant ? option->landTime : 0.0f;
                best.plantTarget = plantTarget;
                best.plantShift = plant && !option->landed
                    ? Vec3 { plantTarget.x - plantLand.x, plantTarget.y - plantLand.y, 0.0f }
                    : Vec3 {};
                best.runSpeed = runSpeed;
            }
        }
    }
    if (!best.valid) {
        int worst = -1;
        for (int k = 0; k < CutCount; ++k)
            if (cuts[static_cast<std::size_t>(k)] > 0 && (worst < 0 || cuts[static_cast<std::size_t>(k)] > cuts[static_cast<std::size_t>(worst)]))
                worst = k;
        if (worst >= 0) best.reason = CutReason[worst];
    }
    return best;
}

} // namespace MatterEngine
