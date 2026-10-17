#include "Engine/Control/AdaptivePhysicalCharacter3D.hpp"
#include <algorithm>
#include <cstdio>
#include <cstdlib> // TEMPTUNE
#include <cmath>
#include <limits>
#include <vector>

namespace MatterEngine {
namespace {
constexpr float Gravity = 9.81f;
Vec3 planar(Vec3 v) { v.z = 0; return v; }
Vec3 limit(Vec3 v, float maximum) {
    const float n = v.length();
    return n > maximum && n > 1e-6f ? v * (maximum / n) : v;
}
float saturate(float v) { return std::clamp(v, 0.0f, 1.0f); }
float approach(float from, float to, float rate, float dt) {
    return from + (to - from) * (1 - std::exp(-rate * dt));
}
// Distancia com sinal do ponto p ao casco convexo (plano XY) de pontos: +
// dentro, - fora. Casco pela cadeia monotona de Andrew.
float signedDistanceToHull(std::vector<Vec3> points, Vec3 p) {
    if (points.size() < 3) return -1.0f;
    std::sort(points.begin(), points.end(), [](const Vec3& a, const Vec3& b) {
        return a.x < b.x || (a.x == b.x && a.y < b.y);
    });
    const auto turn = [](const Vec3& o, const Vec3& a, const Vec3& b) {
        return (a.x - o.x) * (b.y - o.y) - (a.y - o.y) * (b.x - o.x);
    };
    std::vector<Vec3> hull(2 * points.size());
    std::size_t k = 0;
    for (std::size_t i = 0; i < points.size(); ++i) {
        while (k >= 2 && turn(hull[k - 2], hull[k - 1], points[i]) <= 0) --k;
        hull[k++] = points[i];
    }
    for (std::size_t i = points.size() - 1, t = k + 1; i > 0; --i) {
        while (k >= t && turn(hull[k - 2], hull[k - 1], points[i - 1]) <= 0) --k;
        hull[k++] = points[i - 1];
    }
    hull.resize(k - 1);
    if (hull.size() < 3) return -1.0f;
    bool inside = true;
    float nearest = std::numeric_limits<float>::infinity();
    for (std::size_t i = 0; i < hull.size(); ++i) {
        const Vec3 a = hull[i], b = hull[(i + 1) % hull.size()];
        if (turn(a, b, p) < 0) inside = false;
        const float ex = b.x - a.x, ey = b.y - a.y;
        const float length = ex * ex + ey * ey;
        const float t = length > 1e-9f
            ? std::clamp(((p.x - a.x) * ex + (p.y - a.y) * ey) / length, 0.0f, 1.0f) : 0.0f;
        nearest = std::min(nearest, std::hypot(p.x - a.x - t * ex, p.y - a.y - t * ey));
    }
    return inside ? nearest : -nearest;
}

// O ponto do casco convexo (plano XY) de `points` mais perto de p (o
// proprio p se estiver dentro). Com 1-2 pontos, o ponto/segmento.
// As pernas andando (E5): acima de que velocidade da referencia, o ganho e a
// aceleracao que elas pedem ao chao, e a mola de altura.
constexpr float LegWalkingMinimumSpeed = 0.30f;
float tuneTempApc(const char* name, float fallback) {  // TEMPTUNE
    const char* v = std::getenv(name);
    return v ? static_cast<float>(std::atof(v)) : fallback;
}

const float LegWalkingVelocityGain = tuneTempApc("T_LegWalkGain", 4.0f);  // TEMPTUNE
const float LegWalkingAcceleration = tuneTempApc("T_LegWalkAcc", 4.0f);  // TEMPTUNE
const float LegWalkingHeightFrequency = tuneTempApc("T_LegWalkHF", 3.0f);  // TEMPTUNE
const float LegWalkingSpringUp = tuneTempApc("T_LegWalkUp", 0.5f);  // TEMPTUNE
const float LegWalkingSpringDown = tuneTempApc("T_LegWalkDown", 0.25f);  // TEMPTUNE

Vec3 closestInHull(std::vector<Vec3> points, Vec3 p) {
    if (points.empty()) return p;
    std::sort(points.begin(), points.end(), [](const Vec3& a, const Vec3& b) {
        return a.x < b.x || (a.x == b.x && a.y < b.y);
    });
    const auto turn = [](const Vec3& o, const Vec3& a, const Vec3& b) {
        return (a.x - o.x) * (b.y - o.y) - (a.y - o.y) * (b.x - o.x);
    };
    std::vector<Vec3> hull;
    if (points.size() >= 3) {
        hull.resize(2 * points.size());
        std::size_t k = 0;
        for (std::size_t i = 0; i < points.size(); ++i) {
            while (k >= 2 && turn(hull[k - 2], hull[k - 1], points[i]) <= 0) --k;
            hull[k++] = points[i];
        }
        for (std::size_t i = points.size() - 1, t = k + 1; i > 0; --i) {
            while (k >= t && turn(hull[k - 2], hull[k - 1], points[i - 1]) <= 0) --k;
            hull[k++] = points[i - 1];
        }
        hull.resize(k - 1);
    } else {
        hull = points;
    }
    bool inside = hull.size() >= 3;
    Vec3 best = hull.front();
    float bestDistance = std::numeric_limits<float>::infinity();
    for (std::size_t i = 0; i < hull.size(); ++i) {
        const Vec3 a = hull[i], b = hull[(i + 1) % hull.size()];
        if (hull.size() >= 3 && turn(a, b, p) < 0) inside = false;
        const float ex = b.x - a.x, ey = b.y - a.y;
        const float length = ex * ex + ey * ey;
        const float t = length > 1e-9f
            ? std::clamp(((p.x - a.x) * ex + (p.y - a.y) * ey) / length, 0.0f, 1.0f) : 0.0f;
        const Vec3 q { a.x + t * ex, a.y + t * ey, p.z };
        const float d = std::hypot(p.x - q.x, p.y - q.y);
        if (d < bestDistance) { bestDistance = d; best = q; }
    }
    return inside ? p : best;
}

Vec3 rotationError(Quaternion target, Quaternion current) {
    auto q = (target * current.conjugate()).normalized();
    if (q.w < 0) q = {-q.x, -q.y, -q.z, -q.w};
    const Vec3 axis {q.x, q.y, q.z};
    const float n = axis.length();
    return n > 1e-6f ? axis * (2 * std::atan2(n, q.w) / n) : Vec3 {};
}
}

CharacterMotorSettings3D adaptiveCharacterMotorSettings3D(
    const CharacterMotorSettings3D& settings, const PhysicsCharacterState3D& state,
    const CharacterMotorCommand3D& command, float accelerationLimit) {
    auto result = settings;
    const Vec3 velocity = planar(state.velocity);
    const Vec3 direction = planar(command.moveDirection);
    if (state.grounded && velocity.lengthSquared() > 0.04f
        && direction.lengthSquared() > 0.001f) {
        // Pedindo o sentido oposto, a capsula freia com a desaceleracao de
        // parada (18 m/s2: do sprint a zero em ~0,4 s). Com 6 m/s2 (o limite
        // anterior, para o corpo nao cair) a inversao levava 1,3 s; quem
        // derrubava era a pelve girando no meio da freada (ver a locomocao:
        // freando forte, o setor da passada fica).
        const float alignment = dot(velocity.normalized(), direction.normalized());
        const float turn = saturate((0.7f - alignment) / 1.2f);
        result.groundAcceleration = settings.groundAcceleration
            + (std::max(settings.groundAcceleration, settings.groundDeceleration)
                - settings.groundAcceleration) * turn;
    }
    if (accelerationLimit > 0.0f) {
        result.groundAcceleration = std::min(result.groundAcceleration, accelerationLimit);
        result.groundDeceleration = std::min(result.groundDeceleration, accelerationLimit);
        result.groundBrakeDeceleration = std::min(result.groundBrakeDeceleration, accelerationLimit);
    }
    return result;
}

float physicalNavigationAcceleration3D(float walkingAssist) {
    static const float base = tuneTempApc("T_NavAccBase", 4.0f);  // TEMPTUNE
    static const float gain = tuneTempApc("T_NavAccGain", 6.0f);  // TEMPTUNE
    return std::min(PhysicalV2NavigationAcceleration, base + gain * std::clamp(walkingAssist, 0.0f, 1.0f));
}

float physicalNavigationLead3D(float walkingAssist) {
    static const float base = tuneTempApc("T_NavLeadBase", 0.8f);  // TEMPTUNE
    static const float gain = tuneTempApc("T_NavLeadGain", 6.0f);  // TEMPTUNE
    static const float releaseFrom = tuneTempApc("T_NavLeadFree", 0.30f);  // TEMPTUNE
    const float assist = std::clamp(walkingAssist, 0.0f, 1.0f);
    // Acima de ~25% de ajuda a pelve ja acompanha: presa ao corpo, a capsula
    // fazia os pousos de pulo e o empurrao de lado correndo cairem (a 30%,
    // 0-1 -> 2-6 em 6).
    const float x = std::clamp((assist - releaseFrom) / 0.12f, 0.0f, 1.0f);
    return base + gain * assist * assist + 100.0f * x * x * (3.0f - 2.0f * x);
}

float physicalNavigationSpeedScale3D(Vec3 moveDirection, float speedScale, float nominalSpeed,
    Vec3 bodyVelocity, float lead) {
    moveDirection.z = 0.0f;
    if (moveDirection.lengthSquared() < 1e-6f || nominalSpeed <= 0.0f) return speedScale;
    const Vec3 direction = moveDirection.normalized();
    const float along = std::max(0.0f, bodyVelocity.x * direction.x + bodyVelocity.y * direction.y);
    // (Mais solta correndo mais rapido: T_NavLeadSpeed por m/s do corpo.)
    static const float leadSpeed = tuneTempApc("T_NavLeadSpeed", 0.0f);  // TEMPTUNE
    return std::min(speedScale, (along + std::max(0.0f, lead) + leadSpeed * along) / nominalSpeed);
}

float physicalNavigationTurnScale3D(Vec3 currentVelocity, Vec3 moveDirection, float turnRate,
    float walkingAssist) {
    static const float cutFrom = tuneTempApc("T_NavCutFrom", 0.45f);  // TEMPTUNE (rad)
    static const float cutSpan = tuneTempApc("T_NavCutSpan", 1.2f);  // TEMPTUNE (rad)
    static const float cutFloor = tuneTempApc("T_NavCutFloor", 0.35f);  // TEMPTUNE
    static const float lateral = tuneTempApc("T_NavLateral", 5.0f);  // TEMPTUNE (m/s2)
    static const float share = tuneTempApc("T_NavTurnShare", 1.0f);  // TEMPTUNE
    currentVelocity.z = 0.0f;
    moveDirection.z = 0.0f;
    const float speed = currentVelocity.length();
    if (speed < 1.0f || moveDirection.lengthSquared() < 1e-6f) return 1.0f;
    const Vec3 a = currentVelocity * (1.0f / speed), b = moveDirection.normalized();
    const float angle = std::acos(std::clamp(dot(a, b), -1.0f, 1.0f));
    float scale = 1.0f - (1.0f - cutFloor) * saturate((angle - cutFrom) / std::max(0.01f, cutSpan));
    // Curva continua: a velocidade que deixa virar na taxa pedida.
    if (turnRate > 0.1f) scale = std::min(scale, std::max(cutFloor, lateral / (turnRate * speed)));
    const float legs = (1.0f - std::clamp(walkingAssist, 0.0f, 1.0f)) * share;
    return 1.0f - (1.0f - scale) * saturate(legs);
}

void AdaptivePhysicalCharacter3D::reset(const RagdollProfile3D& profile) {
    m_output = {};
    m_wholeBody.prepare(profile);
    m_legShare = m_balanceShare = m_postureShare = 0.0f;
    m_walkingPostureShare = 0.0f;
    m_walkCarryShare = 0.0f;
    m_legPulse = 1.0f;
    m_contactGate = 1.0f;
    m_contactDuty = 1.0f;
    m_contactTurning = 0.0f;
    m_output.jointFeedforward.assign(profile.links.size(), { 0.0f, 0.0f, 0.0f });
    m_compliance.assign(profile.links.size(), 0);
    m_output.jointStrength.assign(profile.links.size(), 1);
    m_secondsWithoutSupport = 0.0f;
    m_failureSeconds = m_falling = m_perturbation = 0;
    m_wasSuspended = false;
    m_sinceSuspended = 10.0f;
    m_sinceExternalHit = 10.0f;
    m_sinceExternalTouch = 10.0f;
    m_assistEnvelope = 1.0f;
    m_wasAirborne = false;
    m_sinceAirborne = 10.0f;
    m_jumpPushElapsed = -1.0f;
    m_supportDuty = 1;
    m_secondsWithoutFootContact = 1;
    m_footContactAge = { 1, 1 };
    m_previousReferenceVelocity = {};
    m_referenceVelocityValid = false;
}

void AdaptivePhysicalCharacter3D::update(const RagdollProfile3D& profile,
    const RagdollState3D& state, const RagdollAnimationConstraint3D& reference,
    const AdaptivePhysicalIntent3D& intent, float dt) {
    if (m_compliance.size() != profile.links.size()) reset(profile);
    auto& out = m_output;
    out.valid = false;
    out.rootForceWorld = out.rootTorqueWorld = out.requestedForceWorld = {};
    out.proxyFollowVelocityWorld = {};
    if (!std::isfinite(dt) || dt <= 0 || dt > 0.05f || state.links.empty()
        || state.links.size() != profile.links.size()) return;
    const auto& root = state.links.front();
    const float mass = std::max(1.0f, profile.totalMassKg);
    out.centerOfMassWorld = out.centerOfMassVelocityWorld = {};
    float weight = 0;
    if (intent.physicalState != nullptr && intent.physicalState->valid) {
        out.centerOfMassWorld = intent.physicalState->centerOfMassWorld;
        out.centerOfMassVelocityWorld = intent.physicalState->centerOfMassVelocityWorld;
    } else for (std::size_t i = 0; i < state.links.size(); ++i) {
        const auto& body = state.links[i];
        const float w = profile.links[i].massFraction;
        out.centerOfMassWorld += (body.position
            + body.orientation.rotate(profile.links[i].centerOfMassLocal)) * w;
        // Backend linearVelocity is the COM velocity, not the link origin velocity.
        out.centerOfMassVelocityWorld += body.linearVelocity * w;
        weight += w;
    }
    if (weight > 0) {
        out.centerOfMassWorld *= 1.0f / weight;
        out.centerOfMassVelocityWorld *= 1.0f / weight;
    }
    // Apoio, captura e inclinacao: do estimador comum (o mesmo que a
    // locomocao e o planejador de passos leem). Os pes so contam por contato
    // de verdade neste tick (sem memoria nem evidencia geometrica).
    const CharacterPhysicalState3D* physical = intent.physicalState != nullptr
        && intent.physicalState->valid ? intent.physicalState : nullptr;
    out.supportCount = 0;
    if (physical != nullptr) {
        for (const FootSupportEstimate3D& foot : physical->feet) {
            if (foot.supporting && !foot.heldFromMemory
                && foot.evidence != ContactEvidence3D::Geometric) ++out.supportCount;
        }
    }
    out.captureErrorMeters = physical != nullptr && physical->supported
        ? physical->captureOutsideMeters : 0.0f;
    out.bodyTiltRadians = physical != nullptr ? physical->rootTiltRadians
        : std::acos(std::clamp(root.orientation.rotate({0, 0, 1}).z, -1.0f, 1.0f));
    m_secondsWithoutSupport = out.supportCount > 0 ? 0.0f
        : std::min(60.0f, m_secondsWithoutSupport + dt);
    // Parada so para a estabilidade: andando, o ponto de captura sai do apoio
    // a cada passada (voo planejado), e isso nao e risco.
    const bool balanceRecoveryEnabled = intent.proxyVelocityWorld.length() < 0.25f;
    const float upright = root.orientation.rotate({0, 0, 1}).z;
    // Apoio medido: um pe tocando algo por baixo (piso, degrau, caixa),
    // pelo stream de interacoes de todos os links. A assistencia atravessa o
    // voo curto da passada; num pulo ou numa queda ela some e o corpo segue
    // so a propria fisica.
    bool footContact = false;
    std::array<bool, 2> touching {};
    std::array<std::size_t, 2> footLink { profile.links.size(), profile.links.size() };
    for (std::size_t i = 0; i < profile.links.size(); ++i) {
        if (profile.links[i].id == "LeftFoot") footLink[0] = i;
        if (profile.links[i].id == "RightFoot") footLink[1] = i;
    }
    for (const auto& contact : state.interactions) {
        if (contact.normalWorld.z <= 0.5f) continue;
        for (std::size_t side = 0; side < 2; ++side) {
            if (contact.linkIndex == footLink[side]) {
                touching[side] = true;
                footContact = true;
            }
        }
    }
    for (std::size_t side = 0; side < 2; ++side) {
        m_footContactAge[side] = touching[side] ? 0.0f
            : std::min(10.0f, m_footContactAge[side] + dt);
    }
    // Mao ou antebraco apoiados numa superficie (em cima ou na lateral de um
    // obstaculo): apoio para se endireitar empurrando.
    std::array<std::size_t, 2> bracedLink { profile.links.size(), profile.links.size() };
    for (const auto& contact : state.interactions) {
        if (contact.normalWorld.z <= 0.3f || contact.linkIndex >= profile.links.size()) continue;
        const auto& id = profile.links[contact.linkIndex].id;
        const bool hand = id == "LeftHand" || id == "RightHand";
        if (!hand && id != "LeftForearm" && id != "RightForearm") continue;
        const std::size_t side = id.rfind("Left", 0) == 0 ? 0 : 1;
        if (hand || bracedLink[side] >= profile.links.size()) bracedLink[side] = contact.linkIndex;
    }
    const bool braced = bracedLink[0] < profile.links.size() || bracedLink[1] < profile.links.size();
    // Base de apoio: as solas dos pes em contato (ou ate pouco atras), no
    // plano. A projecao do centro de massa dentro dela diz quanto torque o
    // chao consegue dar para endireitar o corpo.
    std::vector<Vec3> soles;
    for (std::size_t side = 0; side < 2; ++side) {
        if (footLink[side] >= profile.links.size()
            || m_footContactAge[side] > m_settings.footSupportMemorySeconds) continue;
        const auto& link = profile.links[footLink[side]];
        const auto& body = state.links[footLink[side]];
        const Quaternion box = (body.orientation * link.collider.localOrientation).normalized();
        const Vec3 center = body.position + body.orientation.rotate(link.collider.localPosition);
        if (ragdollColliderHasBox3D(link.collider)) {
            // O contorno de verdade da sola (a casca da chuteira afina no
            // calcanhar e na ponta; o retangulo dava apoio onde nao ha sola).
            for (const Vec3& point : ragdollColliderSolePoints3D(link.collider)) {
                Vec3 corner = center + box.rotate({ point.x, point.y, 0.0f });
                corner.z = 0;
                soles.push_back(corner);
            }
        } else {
            const Vec3 half { 0.12f, 0.05f, 0.03f };
            for (const float sx : { -1.0f, 1.0f }) {
                for (const float sy : { -1.0f, 1.0f }) {
                    Vec3 corner = center + box.rotate({ sx * half.x, sy * half.y, 0.0f });
                    corner.z = 0;
                    soles.push_back(corner);
                }
            }
        }
    }
    m_secondsWithoutFootContact = footContact ? 0.0f
        : std::min(10.0f, m_secondsWithoutFootContact + dt);
    const float memory = std::max(0.01f, m_settings.supportMemorySeconds);
    const float fade = saturate((m_secondsWithoutFootContact - memory) / memory);
    out.supportAuthority = 1.0f - fade * fade * (3.0f - 2.0f * fade);
    // Movimento livre: sustentado pela ajuda com a capsula no chao, sem
    // depender dos pes (no voo da corrida, um pe no ar, passando a perna).
    out.secondsWithoutFootContact = m_secondsWithoutFootContact;
    const bool supported = out.supportAuthority > 0.0f;
    {
        Vec3 projection = out.centerOfMassWorld;
        projection.z = 0;
        out.balanceMargin = signedDistanceToHull(soles, projection);
        const float inside = std::max(0.001f, m_settings.balanceInsideMargin);
        const float outside = std::max(0.001f, m_settings.balanceOutsideMargin);
        const float x = saturate((out.balanceMargin + outside) / (inside + outside));
        out.balanceAuthority = soles.empty() ? 0.0f : x * x * (3 - 2 * x);
    }
    m_supportDuty = approach(m_supportDuty, supported ? 1.0f : 0.0f, 3, dt);
    out.strongestImpulse = 0;
    out.strongestInteractionLink = RagdollDynamics3D::InvalidIndex;
    float disturbance = 0;
    bool externalTouch = false;
    for (auto& c : m_compliance) c = approach(c, 0, 5, dt);
    for (const auto& contact : state.interactions) {
        if (contact.linkIndex >= profile.links.size()) continue;
        const auto& id = profile.links[contact.linkIndex].id;
        if ((id == "LeftFoot" || id == "RightFoot") && contact.normalWorld.z > 0.62f) continue;
        // Logo depois de uma fase em que a locomocao conduziu o corpo pelo
        // chao (carrinho, levantar), o chao tocando quadril, joelho ou mao
        // nao e pancada: era isso que, na volta do carrinho, punha a ajuda em
        // "perturbado" (autoridade cortada ate 70%) e ele reagia como quem
        // vai cair.
        if (m_sinceSuspended < 1.0f && contact.normalWorld.z > 0.62f) continue;
        // Encostando em algo de fora (a bola, leve, quase nao conta).
        if (contact.normalImpulseNewtonSeconds > 0.3f) externalTouch = true;
        if (contact.normalImpulseNewtonSeconds > out.strongestImpulse) {
            out.strongestImpulse = contact.normalImpulseNewtonSeconds;
            out.strongestInteractionLink = contact.linkIndex;
            out.interactionImpulseEstimated = contact.impulseEstimated;
        }
        // Impulse is context, not accumulated momentum or a fall trigger.
        const float strength = saturate(contact.normalImpulseNewtonSeconds / (mass * 0.15f));
        // Contato externo de verdade (fora o apoio dos pes): a partir de
        // ~2 N.s (um esbarrao) a memoria de perturbacao vai inteira. A
        // estimativa de impulso do Jolt e pre-solver e baixa demais para
        // dosar a fisica (um bloco de 60 kg a 2 m/s marcava 25 N.s).
        // TEMPTUNE HitFull: graduado pelo impulso - encostar ou esbarrar de
        // leve em outro boneco (5-9 N.s estimados) tirava 40% da ajuda por
        // ~1,5 s, ele balancava, encostava de novo: a ajuda oscilava perto de
        // outro corpo e ele caia mais facil so por estar perto.
        static const float hitMinimum = tuneTempApc("T_HitMin", 2.0f);
        static const float hitFull = tuneTempApc("T_HitFull", 25.0f);
        if (contact.normalImpulseNewtonSeconds > hitMinimum)
            disturbance = std::max(disturbance, hitFull <= hitMinimum ? 1.0f
                : saturate((contact.normalImpulseNewtonSeconds - hitMinimum) / (hitFull - hitMinimum)));
        int index = static_cast<int>(contact.linkIndex);
        float share = 1;
        for (int depth = 0; depth < 3 && index >= 0; ++depth) {
            m_compliance[index] = std::max(m_compliance[index], strength * share);
            index = profile.links[index].parentIndex;
            share *= 0.45f;
        }
    }
    // Memoria do contato externo (~1 s): enquanto ela dura, o desvio que o
    // contato causou se resolve com fisica completa (alavanca do chao).
    m_perturbation = approach(m_perturbation, disturbance,
        disturbance > m_perturbation ? 40 : 1.5f, dt);
    static const float hitCalm = tuneTempApc("T_HitCalm", 0.5f);
    m_sinceExternalHit = disturbance > hitCalm ? 0.0f
        : std::min(10.0f, m_sinceExternalHit + dt);
    m_sinceExternalTouch = externalTouch ? 0.0f : std::min(10.0f, m_sinceExternalTouch + dt);
    // Falling needs physical loss of viability sustained in time. A big
    // impact alone cannot trigger it, nor can an ordinary airborne stride.
    // Caido de vez: tronco/joelho/mao no chao com a pelve tombada, a pelve
    // baixa sobre o apoio, ou quase deitado sem apoio.
    const float bodyHeight = std::clamp(profile.standingRootHeightMeters, 0.2f, 4.0f);
    const bool likelyFallen = physical != nullptr && !intent.braceRecovering
        && ((physical->nonFootGroundContact && upright < 0.55f)
            || (out.supportCount > 0 && upright < 0.55f
                && root.position.z - physical->supportHeight < bodyHeight * 0.44f)
            || (upright < 0.12f && m_secondsWithoutSupport > 0.25f));
    Vec3 tiltRate = root.angularVelocity;
    tiltRate.z = 0.0f;
    const bool unrecoverable = likelyFallen
        || (!intent.braceRecovering && upright < 0.45f && tiltRate.length() > 1.5f)
        || (!intent.airborne && m_secondsWithoutSupport > 0.35f
            && upright < 0.65f && root.linearVelocity.z < -0.6f);
    m_failureSeconds = unrecoverable ? m_failureSeconds + dt : std::max(0.0f, m_failureSeconds - dt * 2);
    m_falling = approach(m_falling, m_failureSeconds > 0.10f ? 1.0f : 0.0f,
        m_failureSeconds > 0.10f ? 12 : 2, dt);
    // A locomocao desistiu de salvar a postura: cai de vez, ja.
    if (intent.postureLost && !intent.suspended) {
        m_failureSeconds = std::max(m_failureSeconds, 0.2f);
        m_falling = 1.0f;
    }
    // Caido/levantando (a locomocao conduz): nada de "falha" acumulando. Com
    // 3-5 s deitado ela levava 2-3 s para esvaziar depois de levantar, e a
    // ajuda de pe ficava perto de zero justo quando o corpo mais precisava -
    // a do levantar parava e esta ainda nao tinha voltado.
    if (intent.suspended) {
        m_failureSeconds = 0.0f;
        m_falling = 0.0f;
        // Deitado no chao nao e pancada: sem isso, ao levantar a memoria de
        // contato externo ainda estava cheia e o corpo "reagia" a si mesmo.
        m_perturbation = 0.0f;
        m_sinceExternalHit = 10.0f;
        m_wasSuspended = true;
        m_sinceSuspended = 0.0f;
    } else {
        m_sinceSuspended = m_wasSuspended ? 0.0f
            : std::min(10.0f, m_sinceSuspended + dt);
        m_wasSuspended = false;
    }
    {
        const float x = saturate(m_sinceSuspended
            / std::max(0.01f, m_settings.recoveryBoostSeconds));
        out.recoveryBoost = 1.0f - x * x * (3 - 2 * x);
    }
    // O reforco do gesto proprio (o toque/chute na bola): so sem contato de
    // fora recente, sem queda em curso, nem suspenso ou agarrado.
    {
        const float quiet = std::max(0.01f, m_settings.actionQuietSeconds);
        const float touchQuiet = saturate((m_sinceExternalTouch - quiet) / 0.15f);
        const float hitQuiet = saturate((m_sinceExternalHit - quiet) / 0.15f);
        out.actionStability = intent.suspended || intent.manipulated ? 0.0f
            : saturate(intent.actionStability) * touchQuiet * hitQuiet * (1.0f - m_falling);
    }
    const float captureRisk = balanceRecoveryEnabled
        ? saturate((out.captureErrorMeters - 0.025f) / 0.25f) : 0;
    out.stability = supported ? (1 - captureRisk) * saturate((upright - 0.4f) / 0.55f) : 0;
    out.recoverability = (1 - m_falling) * saturate((upright - 0.15f) / 0.65f);
    out.phase = m_falling > 0.5f ? PhysicalControlPhase3D::Falling
        : intent.airborne || !supported ? PhysicalControlPhase3D::Airborne
        : captureRisk > 0.15f ? PhysicalControlPhase3D::Recovering
        : m_perturbation > 0.1f ? PhysicalControlPhase3D::Perturbed
        : PhysicalControlPhase3D::Stable;
    const float available = intent.manipulated || intent.suspended
        ? 0.0f : 1 - m_falling;
    out.locomotionAuthority = available * (1 - 0.7f * std::max(m_perturbation, captureRisk));
    out.postureAuthority = 1 - 0.9f * m_falling;
    out.uprightAuthority = supported ? available : 0;
    // So a cedencia local, perto de um contato. O tonus de quem cai, esta
    // deitado ou levanta e da locomocao (reflexos, tonus deitado, levantar):
    // com a postura (10% caindo) aqui, as juntas chegavam ao levantar com
    // 10-30% da forca, e so a ajuda erguia o corpo.
    for (std::size_t i = 0; i < m_compliance.size(); ++i)
        out.jointStrength[i] = 1 - 0.7f * m_compliance[i];

    // TEMPTUNE: andando/correndo, a ajuda segura o corpo com molas mais
    // macias - com as de parado (14 no plano, 12 na altura) ela apagava o
    // empurrao e o impacto de cada passada (o corpo deslizava "carregado").
    static const float gaitPlanarScale = tuneTempApc("T_GaitPlanarScale", 1.0f);
    static const float gaitHeightScale = tuneTempApc("T_GaitHeightScale", 1.0f);
    const bool gaitFeel = std::hypot(intent.requestedVelocityWorld.x, intent.requestedVelocityWorld.y) > 0.3f
        && !intent.airborne;
    const float frequency = m_settings.planarFrequency * (gaitFeel ? gaitPlanarScale : 1.0f);
    const Vec3 positionError = reference.rootPositionWorld - root.position;
    const Vec3 velocityError = reference.rootLinearVelocityWorld - root.linearVelocity;
    // Aceleracao da propria referencia (a arrancada, a curva): antecipada.
    // Sem o seguimento da capsula: ele nao e intencao. Com ele, um corpo
    // freado por um impacto (o pouso de um pulo) fazia a capsula frear
    // atras, a referencia desacelerava e a "intencao" freava o corpo mais -
    // realimentacao que o parava de 3 m/s a zero em 0,25 s.
    const Vec3 intendedReferenceVelocity = reference.rootLinearVelocityWorld
        - planar(intent.followVelocityWorld);
    Vec3 feedforward {};
    if (m_referenceVelocityValid) {
        feedforward = limit((intendedReferenceVelocity
            - m_previousReferenceVelocity) / dt,
            m_settings.maximumFeedforwardAcceleration)
            * std::clamp(reference.rootFeedforwardShare, 0.0f, 1.0f);
    }
    m_previousReferenceVelocity = intendedReferenceVelocity;
    m_referenceVelocityValid = true;
    // Tres partes, cada uma onde ela age num corpo de verdade:
    //  - a INTENCAO do jogador (a aceleracao da referencia: arrancar, frear,
    //    virar) vai na pelve - e a ajuda de jogo que mantem a resposta;
    //  - a CORRECAO de desvio (empurrao, tropeco, atraso) vai pelos pes em
    //    contato, como atrito do chao, ate ~1,25 g: o pe freia e o tronco
    //    continua, e o giro que isso da aparece sozinho;
    //  - a POSTURA (endireitar, virar) e um par interno pelve x pes: o chao
    //    so segura enquanto o apoio permite - a fisica do contato decide (o
    //    pe tomba na borda se o centro de massa sai da base).
    // Inclinacao da pelve alem da referencia (so o tombar, sem o giro).
    {
        const Vec3 up = root.orientation.rotate({0, 0, 1});
        const Vec3 wantedUp = reference.rootOrientationWorld.rotate({0, 0, 1});
        out.tiltErrorDegrees = std::acos(std::clamp(dot(up, wantedUp), -1.0f, 1.0f)) * 57.2958f;
        // Sem contato externo recente, a inclinacao vem so do proprio
        // movimento (zigue-zague, corte, freada): a postura aguenta muito
        // mais antes de ceder - ele nunca cai sozinho so por se mexer. Com
        // contato (empurrao, tropeco num obstaculo, batida) a faixa e a
        // normal e a fisica vence. Recem-levantado, o mesmo reforco.
        // So com o jogador se movendo: parado e desequilibrado por outra causa
        // (nasceu tombado, por exemplo) vale a faixa normal.
        const float commanded = saturate((planar(intent.proxyVelocityWorld).length()
            - 0.3f) / 0.5f);
        // So ~1,5 s depois do ultimo contato externo: no meio de um tropeco
        // (pulou e bateu num obstaculo, por exemplo) a fisica ainda decide.
        const float quiet = saturate((m_sinceExternalHit - 1.5f) / 0.7f);
        const float calm = quiet * quiet * (3 - 2 * quiet) * commanded;
        m_selfMotionCalm = calm;
        const float extra = std::max({ out.recoveryBoost, calm, out.actionStability });
        const float start = m_settings.uprightAssistStartDegrees + 15.0f * extra;
        const float end = m_settings.uprightAssistEndDegrees + 25.0f * extra;
        const float span = std::max(1.0f, end - start);
        const float x = saturate((out.tiltErrorDegrees - start) / span);
        out.uprightAuthorityShare = 1.0f - x * x * (3 - 2 * x);
        // A referencia tambem pode estar muito inclinada (o clipe e a
        // capsula continuam avancando durante um tropeco). Nessa situacao,
        // erro contra a referencia quase zero nao significa postura viavel:
        // a ajuda deve ceder para os passos salvarem o corpo ou a queda
        // acontecer fisicamente.
        const float absoluteTilt = std::acos(std::clamp(up.z, -1.0f, 1.0f))
            * 57.2958f;
        // TEMPTUNE: no movimento proprio (sem contato externo recente), a
        // faixa absoluta tambem se estende - correndo, um mergulho de 40-55
        // graus (curva, zigue-zague, pe enganchado) ainda e recuperavel.
        static const float calmAbsoluteExtra = tuneTempApc("T_UprightCalmAbs", 15.0f);
        const float absoluteStart = 38.0f + calmAbsoluteExtra * std::max(calm, out.actionStability);
        const float absoluteX = saturate((absoluteTilt - absoluteStart) / 18.0f);
        const float absoluteShare = 1.0f
            - absoluteX * absoluteX * (3.0f - 2.0f * absoluteX);
        out.uprightAuthorityShare = std::min(out.uprightAuthorityShare,
            absoluteShare);
        // Apoiado nas maos, ele ainda consegue se endireitar empurrando - ate
        // bem inclinado (debrucado sobre uma caixa, ficava parado ali).
        if (braced && !intent.suspended) {
            const float b = saturate((out.tiltErrorDegrees - 50.0f) / 35.0f);
            out.uprightAuthorityShare = std::max(out.uprightAuthorityShare,
                1.0f - b * b * (3 - 2 * b));
        }
    }
    // Uma pancada tira a ajuda na hora, e ela volta aos poucos: no meio, o
    // tropeco e da fisica - os passos salvam ou nao. A intencao do jogador
    // (arrancar, virar) continua valendo; o que cede e o que segura o corpo:
    // a postura e a correcao de desvio. A inclinacao ja tira a sustentacao e
    // a correcao pela faixa acima; tirar a postura por ela tambem virava um
    // ciclo (inclina, perde a postura, inclina mais) que derrubava num
    // meio-fio. Caido ou levantando, a locomocao conduz.
    {
        const float wanted = intent.suspended || intent.manipulated ? 1.0f
            : 1.0f - std::max(m_settings.assistContactDrop * saturate(m_perturbation),
                m_falling);
        m_assistEnvelope = wanted < m_assistEnvelope ? wanted
            : std::min(wanted, m_assistEnvelope + m_settings.assistRecoveryPerSecond * dt);
        out.assistEnvelope = m_assistEnvelope;
    }
    const float authority = available * out.supportAuthority;
    // As pernas (E4) continuam de pe com o corpo agarrado pela PhysGun: sao
    // as juntas dele, nao ajuda de fora (a ajuda externa na pelve some quando
    // ele e manipulado; as pernas, so suspenso ou caindo).
    const float legsAuthority = (intent.suspended ? 0.0f : 1.0f - m_falling) * out.supportAuthority;

    const Vec3 intentAcceleration = limit(planar(feedforward),
        m_settings.maximumPlanarAcceleration) * (out.locomotionAuthority * out.supportAuthority);
    // Depois de um contato externo o corpo nao volta para onde a capsula
    // estava (a capsula e que vai ate ele): so frear e reequilibrar. Puxar
    // de volta fazia o corpo rebater um bloco de 60 kg a 1 m/s.
    float anchor = 1.0f - saturate(m_perturbation);
    // Experimento v2 (T_RunAnchor): correndo, a correcao segue a velocidade
    // e a posicao so nesta fracao - a distancia que sobra (o corpo que
    // atrasou no pouso de um pulo, por exemplo) e a capsula que fecha,
    // indo ate ele. Pela mola de posicao, ~0,3 m atras depois do pouso, a
    // pelve seguia empurrada para a frente ja mais rapida que a capsula (8,2
    // contra 7,5 m/s) e o corpo mergulhava com os pes atras.
    {
        static const float runAnchor = tuneTempApc("T_RunAnchor", 1.0f);  // TEMPTUNE
        const float runSpeed = std::hypot(intent.requestedVelocityWorld.x, intent.requestedVelocityWorld.y);
        if (runAnchor < 1.0f && !intent.airborne)
            anchor *= 1.0f + (runAnchor - 1.0f) * saturate((runSpeed - 1.5f) / 1.5f);
    }
    const Vec3 correctionAcceleration = limit(
        planar(positionError) * (frequency * frequency * anchor)
            + planar(velocityError) * (2 * frequency),
        m_settings.frictionAccelerationLimit)
        * (out.locomotionAuthority * out.supportAuthority * out.uprightAuthorityShare
            * m_assistEnvelope);
    out.supportAcceleration = 0;
    float legVerticalAcceleration = 0.0f;
    // As pernas carregando o corpo andando (E5): a referencia anda. Correndo
    // e freando, a velocidade e a do perfil de freada.
    const float brakeRunShare = std::clamp(intent.brakeRunShare, 0.0f, 1.0f);
    const Vec3 requestedVelocity = brakeRunShare > 0.001f
        ? planar(intent.brakeRunVelocityWorld) : planar(intent.requestedVelocityWorld);
    // TEMPTUNE: soltando o comando com o corpo ainda rapido, o controle
    // seguia no modo parado na hora (peso, equilibrio e postura do parado) -
    // com pouca ajuda o corpo a 2,5 m/s desabava em 0,2 s. Fica no modo
    // andando ate o corpo desacelerar.
    static const float keepWalkingSpeed = [] { const char* v = std::getenv("T_LegWalkKeep"); return v ? static_cast<float>(std::atof(v)) : 0.0f; }();
    const bool legsWalking = m_settings.jointWhileWalking
        && (requestedVelocity.length() > LegWalkingMinimumSpeed
            || (keepWalkingSpeed > 0.0f && physical != nullptr
                && planar(physical->centerOfMassVelocityWorld).length() > keepWalkingSpeed));
    bool pelvisSupportInLegsMode = false;
    // Decolagem: a janela de impulso comeca quando a capsula decola com o
    // corpo apoiado e acaba quando ele sobe na velocidade dela.
    if (intent.airborne && !m_wasAirborne) {
        m_jumpPushElapsed = out.supportAuthority > 0.5f ? 0.0f : -1.0f;
    }
    m_wasAirborne = intent.airborne;
    m_sinceAirborne = intent.airborne ? 0.0f : std::min(10.0f, m_sinceAirborne + dt);
    bool pushing = false;
    if (intent.airborne && m_jumpPushElapsed >= 0.0f) {
        m_jumpPushElapsed += dt;
        // Pulo carregado: a janela se estende (a capsula sai mais rapido);
        // alcancada a velocidade dela, so pausa.
        const float window = m_settings.jumpPushSeconds
            + m_settings.jumpHoldPushSeconds * std::clamp(intent.jumpCharge, 0.0f, 1.0f);
        if (m_jumpPushElapsed >= window) {
            m_jumpPushElapsed = -1.0f;
        } else {
            pushing = root.linearVelocity.z < reference.rootLinearVelocityWorld.z * 0.95f;
        }
    } else if (!intent.airborne) {
        m_jumpPushElapsed = -1.0f;
    }
    if (supported && intent.airborne && !footContact && !pushing) {
        // Voo balistico: nada ergue o corpo sem os pes no chao.
    } else if (supported && pushing) {
        const float charge = std::clamp(intent.jumpCharge, 0.0f, 1.0f);
        out.supportAcceleration = std::clamp(
            (reference.rootLinearVelocityWorld.z - root.linearVelocity.z) * 20.0f
                + Gravity * 0.5f,
            0.0f, m_settings.maximumJumpAcceleration
                + m_settings.jumpChargeExtraAcceleration * charge)
            * out.uprightAuthorityShare;
    } else if (supported) {
        // As pernas sustentam o peso (IK pela pelve desejada); a pelve so
        // recebe uma fracao do peso e a correcao de altura (no modo antigo).
        const float hf = m_settings.heightFrequency * (gaitFeel ? gaitHeightScale : 1.0f);
        // TEMPTUNE: correndo, a mola de altura da pelve nao ergue o corpo
        // afundado de uma vez (ate 0,9 g na pelve) - junto com a perna que
        // estende, ela lancava o corpo nas curvas (os saltinhos).
        static const float runSpringUpCap = tuneTempApc("T_RunSpringCap", -1.0f);
        float heightSpring = positionError.z * hf * hf + velocityError.z * 2 * hf;
        if (runSpringUpCap >= 0.0f && legsWalking && !intent.airborne
            && requestedVelocity.length() > 2.5f && brakeRunShare <= 0.001f)
            heightSpring = std::min(heightSpring, runSpringUpCap);
        out.supportAcceleration = std::clamp(
            Gravity * saturate(m_settings.gravityAssistFraction)
                + (intent.airborne ? feedforward.z : 0.0f)
                + heightSpring,
            0.0f, m_settings.maximumSupportAcceleration)
            * authority * out.uprightAuthorityShare;
        // Loaded legs act as dampers as well as springs. Absorb upward
        // rebound relative to the planned ascent; no damping of an intended
        // jump, and no assistance after support memory expires.
        if (!intent.airborne) {
            const float excessRise = root.linearVelocity.z
                - std::max(0.0f, reference.rootLinearVelocityWorld.z) - 0.10f;
            out.supportAcceleration -= std::clamp(excessRise * 12.0f,
                0.0f, m_settings.maximumReboundDampingAcceleration)
                * authority * out.uprightAuthorityShare;
        }
        // E4: a mola de altura e o amortecimento do rebote vao pelas pernas
        // (forca do chao nos pes) na parte que as pernas carregam; a fracao
        // de gravidade na pelve so no modo antigo (ver abaixo).
        if (m_settings.jointSupport) {
            // Nas pernas: o peso inteiro (abaixo) e uma mola de altura mais
            // macia que a da pelve, ate +1 g - a de 12 Hz pedia 3 pesos numa
            // perna so, fora do centro de massa, e tombava o corpo.
            // Andando (E5), mais macia: a altura da referencia sobe e desce
            // com o clipe de corrida e a mola seguia isso, pedindo de meio a
            // dois pesos ao chao a cada passada.
            const float lf = legsWalking ? LegWalkingHeightFrequency : m_settings.legHeightFrequency;
            float legSpring = positionError.z * lf * lf + velocityError.z * 2.0f * lf;
            static const float runLegSpringUpCap = tuneTempApc("T_RunLegSpringCap", -1.0f);
            if (runLegSpringUpCap >= 0.0f && legsWalking && !intent.airborne
                && requestedVelocity.length() > 2.5f && brakeRunShare <= 0.001f)
                legSpring = std::min(legSpring, runLegSpringUpCap);
            if (!intent.airborne) {
                const float excessRise = root.linearVelocity.z
                    - std::max(0.0f, reference.rootLinearVelocityWorld.z) - 0.10f;
                legSpring -= std::clamp(excessRise * 12.0f, 0.0f, m_settings.maximumReboundDampingAcceleration);
            }
            legVerticalAcceleration = std::clamp(legSpring, (legsWalking ? -LegWalkingSpringDown : -0.5f) * Gravity,
                    (legsWalking ? LegWalkingSpringUp : 1.0f) * Gravity)
                * legsAuthority * out.uprightAuthorityShare;
            pelvisSupportInLegsMode = true;
        }
    }
    out.requestedForceWorld = (intentAcceleration + correctionAcceleration
        + Vec3{0, 0, out.supportAcceleration}) * mass;
    const bool correctionAtPelvis = m_settings.correctionMode == 1
        || m_settings.correctionMode == 3;
    // E4, em partes (cada uma sai da pelve quando as pernas a fazem):
    //  - o PESO e a altura pelas pernas, parado;
    //  - o EQUILIBRIO planar (correcao de desvio e intencao) pelo centro de
    //    pressao, parado - onde o planejador e o dono dos pes; andando, a
    //    passada do clipe ainda nao equilibra o corpo sozinha (E5);
    //  - a POSTURA pelos tornozelos (centro de pressao na sola), parado.
    // Cada troca e gradual (~0,15 s).
    const bool planValid = intent.contactPlan != nullptr && intent.contactPlan->valid;
    const bool standingPlan = planValid && (m_settings.jointWhileWalking
        || (!intent.contactPlan->feet[0].followingGait && !intent.contactPlan->feet[1].followingGait));
    const bool legsCanCarry = supported && !intent.airborne && !intent.suspended;
    // As pernas assumem ate (1 - legsAssistRetained): o resto da ajuda segue
    // na pelve.
    static const bool walkShareDecouple = tuneTempApc("T_WalkDecouple", 1.0f) > 0.5f;  // TEMPTUNE
    // (O gesto proprio reforcado: a ajuda vai ate actionAssist - so com as
    // pernas carregando o corpo, onde ela e a escolhida pelo jogador; sem
    // isso a "ajuda parado" e outra coisa (0 desliga a forca planar da pelve)
    // e subi-la empurrava o corpo no meio do gesto. O torque de postura e a
    // faixa de inclinacao valem sempre.)
    const float actionAssist = m_settings.jointSupport || m_settings.jointWhileWalking
        ? std::clamp(m_settings.actionAssist, 0.0f, 1.0f) * out.actionStability : 0.0f;
    // A ajuda auxiliar (17/10, consenso Claude + Codex; vault
    // 04_locomocao/ajuda_auxiliar.md): so as forcas de FORA (sem reacao no
    // corpo), e 0% e zero de verdade em todo canal. O reforco do gesto proprio
    // nao fura o zero: ele leva a ajuda escolhida a actionAssist na proporcao
    // dela (igual quando o reforco e 0, o caso de hoje).
    const auto withAction = [&](float chosen) {
        chosen = std::clamp(chosen, 0.0f, 1.0f);
        return chosen + (std::max(chosen, actionAssist) - chosen) * chosen;
    };
    const float legsMaximumShare = 1.0f - withAction(m_settings.legsAssistRetained);
    const auto share = [&](float& value, bool enabled, bool wanted) {
        // Andando, a parte das pernas e a da ajuda andando (o peso, a postura
        // e o empurrao ja se dividem por ela): presa ao slider de parado, com
        // 100% parado e pouca ajuda andando as pernas nao carregavam nada e a
        // pelve subia e descia 13-22 cm.
        value = enabled && legsCanCarry ? approach(value, wanted ? (legsWalking && walkShareDecouple ? 1.0f
            : legsMaximumShare) : 0.0f, 7.0f, dt) : 0.0f;
        return value;
    };
    // (Andando, o peso pelas pernas com a propulsao ainda na pelve tomba o
    // corpo para a frente - medido: 132 quedas em 216 e o trote caindo; peso
    // e propulsao vao juntos para as pernas com a passada nova, na E5.)
    const float weightShare = share(m_legShare, m_settings.jointSupport, standingPlan);
    const float balanceShare = share(m_balanceShare, m_settings.jointSupport && m_settings.jointBalance,
        standingPlan);
    // TEMPTUNE: andando, a postura tambem pelas pernas (a de apoio segura a
    // pelve em pe) - com a ajuda baixa a pelve tombava para tras.
    static const bool postureWalk = [] { const char* v = std::getenv("T_PostureWalk"); return !v || std::atof(v) > 0.5; }();
    const float postureShare = share(m_postureShare,
        m_settings.jointSupport && m_settings.jointPosture,
        standingPlan && !legsWalking);
    out.legShare = weightShare;
    out.balanceShare = balanceShare;
    const bool legsCarry = weightShare > 0.001f;
    const float discreet = withAction(m_settings.walkingAssistScale);
    const float standingAssist = withAction(m_settings.legsAssistRetained);
    // A ajuda e a que o jogador escolheu, sempre. (Antes, com degrau,
    // meio-fio, escada ou qualquer desnivel de 6 cm a frente - uma barreira
    // tambem -, ela voltava inteira enquanto passava: com 0% escolhido, perto
    // de uma parede o corpo andava com 100%.)
    const float runningAssist = discreet;
    // (O peso andando: a ajuda escolhida sem a queda do impacto.)
    const float weightRunning = m_settings.walkingWeightAssistScale >= 0.0f
        ? withAction(m_settings.walkingWeightAssistScale) : runningAssist;
    const float walkingAssist = !legsWalking ? standingAssist
        : brakeRunShare > 0.001f ? standingAssist + (runningAssist - standingAssist) * brakeRunShare
        : runningAssist;
    const float physicalWalkingShare = legsWalking ? 1.0f - walkingAssist : 1.0f;
    // Nas pernas a correcao de posicao e mais lenta (4 Hz, contra 14 na
    // pelve): pelo centro de pressao o corpo so faz o que cabe nos pes; o
    // resto e passo do planejador.
    // A correcao de posicao pelas pernas e feita abaixo, contra o apoio
    // medido (o centro de massa sobre os pes), nao contra a capsula: a
    // capsula segue o corpo, e com ela como alvo nada trazia o corpo de volta
    // quando ele derivava.
    // Andando, as pernas seguem a VELOCIDADE pedida com o que o chao da (a
    // arrancada da capsula, ~14 m/s2, nao cabe num pe: pedida, o corpo tombava
    // para a frente na saida) - o lugar do pe faz o resto.
    float walkingDriveRamp = 1.0f;
    if (legsWalking) {
        const Vec3 wanted = requestedVelocity;
        const float wantedSpeed = wanted.length();
        const float along = wantedSpeed > 0.001f
            ? std::max(0.0f, dot(planar(physical->centerOfMassVelocityWorld),
                wanted * (1.0f / wantedSpeed))) : 0.0f;
        // Primeiro transfere massa e inclina, depois cresce a propulsao das
        // pernas. Ainda responde de imediato (35%), mas nao joga os pes para
        // a frente do peito no primeiro apoio.
        const float startup = saturate(along / 1.35f);
        walkingDriveRamp = 0.35f + 0.65f
            * startup * startup * (3.0f - 2.0f * startup);
    }
    Vec3 legPlanarAcceleration = legsWalking
        ? limit(planar(requestedVelocity - physical->centerOfMassVelocityWorld)
            * LegWalkingVelocityGain, LegWalkingAcceleration)
                * (balanceShare * walkingDriveRamp * physicalWalkingShare)
        : intentAcceleration * balanceShare;
    // O gancho do contato (o RL): na parte das pernas.
    if (intent.rlContact) legPlanarAcceleration += planar(intent.rlContactAccelerationWorld) * balanceShare;
    // O peso pelas pernas: a fracao delas vezes quanto do peso elas assumem.
    // Em marcha, o experimento pode conservar peso na pelve para evitar a
    // oscilacao vertical observada. Parado, o slider e a fonte de verdade:
    // 0% transfere todo o peso para as pernas; senao a interface mostraria
    // zero enquanto uma mola vertical integral ainda sustentava a pelve.
    // TEMPTUNE: andando, o peso tambem segue a ajuda - a mola vertical na
    // pelve sustentava o corpo inteiro com a ajuda a 5% (ate 1200 N com os
    // dois pes no ar: o corpo flutuava e o pe de apoio pairava sem carga).
    static const float weightWalk = [] { const char* v = std::getenv("T_WeightWalk"); return v ? static_cast<float>(std::atof(v)) : 0.5f; }();
    // (Pela ajuda andando do jogador, sem a mistura com a de parado da
    // freada: com ela, freando a 100% as pernas passavam a carregar parte do
    // peso e o corte de 90 graus no trote voltava a cair.)
    // TEMPTUNE: e uma parte do peso sempre nas pernas andando, mesmo com a
    // ajuda cheia - e o que faz sentir a pisada e o empurrao (a 5% a pelve
    // subia e descia +-3,3 cm e a velocidade pulsava +-0,3 m/s; a 100%, com
    // o peso todo na mola da pelve, +-1,9 cm e +-0,12).
    static const float weightWalkFloor = tuneTempApc("T_WeightFloor", 0.6f);
    // Andando, as pernas carregam o que a ajuda nao leva, em linha reta: a
    // pelve leva a x (o que levava a 100%). Antes era max(0,5 x (1 - a), piso a
    // partir de 50%): a 0% a pelve ainda levava metade do peso e o meio da
    // regua nao era monotonico (0,5 / 0,75 / 0,4). A 100%, a mesma conta de
    // antes.
    const float legsWeightFull = std::max({ std::clamp(m_settings.legsWeightFraction, 0.0f, 1.0f),
        weightWalk * (1.0f - 1.0f), weightWalkFloor * std::clamp((1.0f - 0.5f) / 0.4f, 0.0f, 1.0f) });
    const float legsWeight = !legsWalking ? 1.0f
        : weightRunning >= 1.0f ? std::max({ std::clamp(m_settings.legsWeightFraction, 0.0f, 1.0f),
            weightWalk * (1.0f - weightRunning),
            weightWalkFloor * std::clamp((weightRunning - 0.5f) / 0.4f, 0.0f, 1.0f) })
        : 1.0f - weightRunning * (1.0f - legsWeightFull);
    // Andando, as pernas carregam o piso de peso mesmo com a ajuda parado
    // cheia: a parte delas, que vem do slider de parado, ia a zero com 100% e
    // o piso nao fazia nada (o corpo seguia "carregado" pela pelve).
    // (So com a ajuda alta: com pouca, a parte das pernas ja e a da ajuda que
    // falta, e o piso a mais deixava o trote a 5% lento - 1,95 -> 1,31 m/s.)
    const float floorAssist = std::clamp((weightRunning - 0.5f) / 0.4f, 0.0f, 1.0f);
    m_walkCarryShare = approach(m_walkCarryShare,
        legsWalking && legsCanCarry && m_settings.jointSupport
            ? (weightRunning >= 1.0f ? weightWalkFloor * floorAssist * floorAssist * (3.0f - 2.0f * floorAssist)
                : legsWeight) : 0.0f, 7.0f, dt);
    const float weightByLegs = std::max(weightShare * legsWeight, m_walkCarryShare);
    legVerticalAcceleration = legVerticalAcceleration * weightByLegs;
    // A pelve leva o que as pernas nao levam; sem apoio (no ar, levantado),
    // a sustentacao de fora e so a da ajuda: a 0% ninguem segura o corpo.
    // Abaixo de 100%, a pelve nunca leva mais do que a ajuda deixa: parado, a
    // fracao escolhida; andando, a x a parte da pelve a 100%; sem apoio, a
    // fracao inteira escolhida. (Sem o teto, enquanto as pernas assumiam o
    // peso - 0,15 s no comeco e depois de cada voo - a pelve segurava o resto
    // mesmo a 0%.)
    if (pelvisSupportInLegsMode) {
        const float pelvisShare = 1.0f - weightByLegs;
        const float weightAssist = legsWalking && brakeRunShare <= 0.001f ? weightRunning : walkingAssist;
        const float allowed = !legsCanCarry ? weightAssist
            : legsWalking ? weightAssist * (1.0f - legsWeightFull) : weightAssist;
        out.supportAcceleration *= weightAssist >= 1.0f ? pelvisShare : std::min(pelvisShare, allowed);
    }
    // Andando pela passada, a ajuda discreta usa um unico envelope para
    // propulsao, correcao de posicao e postura. Deixar a intencao inteira
    // enquanto a postura era reduzida fazia "30%" empurrar a pelve com a
    // arrancada de 100%, mas oferecer apenas 30% do torque que a mantinha em
    // pe: o corpo era lancado para a frente e caia de rosto.
    // TEMPTUNE: andando, a ajuda no plano pulsa com as pernas - mais quando o
    // pe de apoio esta atras do corpo (o empurrao), menos logo que ele pousa
    // a frente (a pisada freia) e no voo. Continua, ela fazia o corpo deslizar
    // na velocidade pedida e o empurrao de cada passada nao aparecia.
    float legPulse = 1.0f;
    {
        static const float pushGain = tuneTempApc("T_PushGain", 1.6f);
        static const float landGain = tuneTempApc("T_LandGain", 0.5f);
        static const float flightGain = tuneTempApc("T_FlightGain", 0.7f);
        float wanted = 1.0f;
        // (Nao freando: ali o pe a frente do corpo e quem freia, e cortar a
        // ajuda nele deixava a parada lenta - um sprint nao parava a tempo.)
        const bool slowingDown = brakeRunShare > 0.001f
            || requestedVelocity.length() < 0.5f;
        if (legsWalking && !slowingDown && physical != nullptr) {
            const Vec3 velocity = planar(physical->centerOfMassVelocityWorld);
            const float speed = velocity.length();
            if (speed > 0.5f) {
                const Vec3 direction = velocity * (1.0f / speed);
                bool behind = false, ahead = false;
                for (std::size_t side = 0; side < 2; ++side) {
                    if (!physical->feet[side].supporting) continue;
                    const float along = dot(planar(physical->feet[side].soleCenterWorld
                        - physical->centerOfMassWorld), direction);
                    behind = behind || along < -0.02f;
                    ahead = ahead || along > 0.02f;
                }
                wanted = behind ? pushGain : ahead ? landGain : (behind || ahead) ? 1.0f : flightGain;
            }
        }
        static const float landNoPulse = tuneTempApc("T_LandNoBoost", 0.0f);  // TEMPTUNE
        if (landNoPulse > 0.0f)
            wanted = 1.0f + (wanted - 1.0f) * saturate((m_sinceAirborne - landNoPulse) / 0.15f);
        m_legPulse += (wanted - m_legPulse) * (1.0f - std::exp(-dt / 0.03f));
        legPulse = m_legPulse;
    }
    Vec3 planarAssist = (intentAcceleration * walkingAssist
                * walkingDriveRamp
            + (correctionAtPelvis ? correctionAcceleration * walkingAssist : Vec3 {})) * mass;
    // So o empurrao ao longo do movimento pulsa; o de lado (o que curva o
    // corpo) fica inteiro - pulsando tudo, metade do tempo a curva ficava
    // sem ajuda e o zigue-zague em sprint caia (3 em 8).
    if (legPulse != 1.0f && physical != nullptr) {
        const Vec3 velocity = planar(physical->centerOfMassVelocityWorld);
        const float speed = velocity.length();
        if (speed > 0.5f) {
            const Vec3 direction = velocity * (1.0f / speed);
            const Vec3 along = direction * dot(planarAssist, direction);
            planarAssist = planarAssist - along + along * legPulse;
        }
    }
    // TEMPTUNE: andando, a ajuda age junto com os pes - so com pe no chao.
    // No voo o corpo segue balistico (sem empurrao nem sustentacao de fora):
    // empurrado e sustentado no ar, ele deslizava e flutuava entre as
    // pisadas, sem coincidir com o movimento das pernas.
    {
        static const float contactShare = tuneTempApc("T_AssistContact", 1.0f);
        bool touching = false;
        if (physical != nullptr)
            for (std::size_t side = 0; side < 2; ++side)
                touching = touching || physical->feet[side].supporting;
        m_contactGate += ((touching ? 1.0f : 0.0f) - m_contactGate)
            * (1.0f - std::exp(-dt / (touching ? 0.015f : 0.03f)));
        const bool moving = legsWalking && !intent.airborne && brakeRunShare <= 0.001f
            && requestedVelocity.length() > 0.5f;
        // O que a ajuda dava no voo vai para o apoio: com o pe no chao ela
        // empurra na proporcao do tempo no ar (1/fracao em contato, ate 2,5),
        // e a media fica a mesma - so ela deixava o sprint em 6,6 de 7,5 m/s.
        static const float contactNormalize = tuneTempApc("T_AssistContactNorm", 1.0f);
        // (v2 - ou T_JumpDuty no v1: o voo de um pulo nao entra na fracao
        // em contato - ela caia a ~0,3 num pulo e o reforco ia ao maximo, x2,5,
        // justo no pouso: a correcao da velocidade perdida virava ~4 g na
        // pelve com os pes plantados e o corpo tombava para a frente.)
        static const bool jumpDutyEnv = tuneTempApc("T_JumpDuty", 0.0f) > 0.5f;  // TEMPTUNE
        if (!(m_settings.v2 || jumpDutyEnv) || !intent.airborne)
            m_contactDuty += (m_contactGate - m_contactDuty) * (1.0f - std::exp(-dt / 0.5f));
        const float boost = 1.0f + contactNormalize
            * (std::min(2.5f, 1.0f / std::max(0.4f, m_contactDuty)) - 1.0f);
        // Virando forte (o pedido a mais de ~10-30 graus do rumo do corpo), a
        // ajuda volta a ser continua: com ela so nos apoios, o zigue-zague e os
        // cortes em sprint caiam (1-2 em 8). Reto, ela anda com os pes.
        float turning = 0.0f;
        if (physical != nullptr) {
            const Vec3 velocity = planar(physical->centerOfMassVelocityWorld);
            const float speed = velocity.length(), wantedSpeed = requestedVelocity.length();
            if (speed > 0.5f && wantedSpeed > 0.5f) {
                const float cosine = std::clamp(dot(velocity, requestedVelocity) / (speed * wantedSpeed), -1.0f, 1.0f);
                turning = saturate((std::acos(cosine) - 0.17f) / 0.35f);
            }
        }
        m_contactTurning += (turning - m_contactTurning) * (1.0f - std::exp(-dt / (turning > m_contactTurning ? 0.02f : 0.25f)));
        // (So com a ajuda alta - a 5% a passada aprovada ficava mais lenta,
        // 1,6 -> 1,3 m/s.)
        // (A forma da ajuda no apoio e a do 100% em qualquer nivel: a regua so
        // muda a intensidade - antes ela mudava de continua para pulsada a
        // partir de 50%.)
        const float highAssist = runningAssist > 0.0f ? 1.0f : 0.0f;
        const float share = contactShare * (1.0f - m_contactTurning)
            * highAssist * highAssist * (3.0f - 2.0f * highAssist);
        // (Experimento v2, T_LandNoBoost = s: logo depois do pouso de um
        // pulo, nem o reforco do apoio nem o pulso - a correcao da velocidade
        // perdida no toque vai inteira, sem multiplicar.)
        static const float landNoBoost = tuneTempApc("T_LandNoBoost", 0.0f);  // TEMPTUNE
        const float landedShare = landNoBoost > 0.0f
            ? saturate((m_sinceAirborne - landNoBoost) / 0.15f) : 1.0f;
        const float gate = moving ? 1.0f + ((1.0f - share * (1.0f - m_contactGate))
            * (1.0f + (boost - 1.0f) * share * m_contactGate) - 1.0f) * landedShare : 1.0f;
        // (Ao longo do movimento e o peso; o de lado, que curva o corpo, fica
        // inteiro: sem ele no voo a curva em sprint caia.)
        if (moving && physical != nullptr) {
            const Vec3 velocity = planar(physical->centerOfMassVelocityWorld);
            const float speed = velocity.length();
            if (speed > 0.5f) {
                const Vec3 direction = velocity * (1.0f / speed);
                const Vec3 along = direction * dot(planarAssist, direction);
                planarAssist = planarAssist - along + along * gate;
            } else {
                planarAssist = planarAssist * gate;
            }
        }
        static const float verticalShare = tuneTempApc("T_AssistContactVert", 1.0f);
        // TEMPTUNE: na corrida calma (sem contato externo ha 1,5 s, o
        // jogador mandando correr) o reforco vertical no apoio (o peso do voo
        // passado para o contato, ate 2,5x) sai: ele lancava o corpo para cima
        // a cada apoio de sprint (+1,96 m/s por apoio contra ~1,24 do voo do
        // clipe; 41% dos apoios acima de 1,9 m/s) - os "saltinhos". Sem ele:
        // +1,39 m/s e mais empurrao para a frente (+0,21 m/s por apoio).
        // Depois de uma batida (tropeco, empurrao) ele volta: e o que segura o
        // corpo que tropecou (sem ele, tropeco do sprint 0 -> 15 quedas em 192).
        static const bool verticalBoostOnlyAfterHit = tuneTempApc("T_VertBoostHit", 1.0f) > 0.5f;
        // (O pouso de um pulo tambem conta como batida: o reforco segura o
        // corpo que chega rapido e gira para a frente sobre o pe que freia -
        // sem ele, pulo em sprint 0 -> 5 quedas em 12.)
        static const float boostLandHold = tuneTempApc("T_BoostLandHold", 0.6f);
        static const float boostLandFade = tuneTempApc("T_BoostLandFade", 0.6f);
        const float landed = boostLandHold < 0.0f ? 1.0f
            : saturate((m_sinceAirborne - boostLandHold) / std::max(0.01f, boostLandFade));
        const float calmShare = verticalBoostOnlyAfterHit
            ? m_selfMotionCalm * landed * landed * (3.0f - 2.0f * landed) : 0.0f;
        // (So o reforco, acima de 1, depende da batida: o corte no voo,
        // abaixo de 1, vale sempre. Multiplicando tudo pela calma, correndo
        // a pelve era empurrada para cima em pleno voo - ate 0,9 g - e o voo
        // esticava: os saltinhos das curvas.)
        float verticalFactor = 1.0f + (gate - 1.0f) * verticalShare;
        if (verticalFactor > 1.0f) verticalFactor = 1.0f + (verticalFactor - 1.0f) * (1.0f - calmShare);
        // v2: correndo, a sustentacao (de baixo para cima) nao estica o voo
        // alem do da passada. Com a excecao da curva (a ajuda continua no voo
        // virando forte) ela segurava o corpo no ar no zigue-zague - ate 0,4 s
        // de voo sem pulo, "pendurado". (Cortada desde o comeco do voo, o
        // zigue-zague caia: ela segura o corpo enquanto o pe chega.)
        if (m_settings.v2 && moving && out.supportAcceleration > 0.0f)
            verticalFactor *= 1.0f - saturate((m_secondsWithoutFootContact - 0.18f) / 0.06f);
        out.supportAcceleration *= verticalFactor;
        m_apcDebugGate = gate;
    }
    if (std::getenv("XAPCDBG"))
        std::printf("APC erro z %+.3f vz %+.2f | sust %+.1f | gate %.2f contato %.2f virando %.2f | pernas %.2f | ar %d pulo %d pe %d\n",
            positionError.z, velocityError.z, out.supportAcceleration, m_apcDebugGate, m_contactGate, m_contactTurning,
            weightByLegs, intent.airborne ? 1 : 0, pushing ? 1 : 0, footContact ? 1 : 0);
    // v2: a ajuda nunca empurra para a frente (no rumo da referencia) um corpo
    // que ja vai mais rapido que ela - a mola de posicao e o pulso do apoio
    // seguiam empurrando depois do pouso de um pulo (8,2 m/s com a
    // referencia a 7,5): ~1400 N na pelve e o corpo mergulhava.
    if (m_settings.v2 && physical != nullptr) {
        const Vec3 referenceVelocity = planar(reference.rootLinearVelocityWorld);
        const float referenceSpeed = referenceVelocity.length();
        if (referenceSpeed > 0.5f) {
            const Vec3 direction = referenceVelocity * (1.0f / referenceSpeed);
            const float bodyAlong = dot(planar(physical->centerOfMassVelocityWorld), direction);
            const float push = dot(planarAssist, direction);
            if (bodyAlong > referenceSpeed && push > 0.0f) {
                // (Dosado em 0,3 m/s: na velocidade, ainda um pouco.)
                const float over = saturate((bodyAlong - referenceSpeed) / 0.3f);
                planarAssist -= direction * (push * over);
            }
        }
    }
    // Experimento v2 (T_PlanarCap, m/s2): a ajuda no plano, ja com o pulso e
    // o reforco do apoio, nao passa do que o chao daria pelo atrito - os
    // reforcos multiplicavam tambem a correcao (o pouso de um pulo em sprint
    // perdia 1,2 m/s e a pelve levava ~2,5 g para a frente com os pes
    // plantados: o corpo tombava para a frente).
    {
        static const float planarCap = tuneTempApc("T_PlanarCap", -1.0f);  // TEMPTUNE
        static const float planarCapAssist = tuneTempApc("T_PlanarCapAssist", 0.0f);  // TEMPTUNE
        if (planarCap > 0.0f) {
            const float scale = planarCapAssist > 0.5f ? std::max(0.05f, walkingAssist) : 1.0f;
            planarAssist = limit(planar(planarAssist), planarCap * mass * scale);
        }
    }
    out.rootForceWorld = planarAssist + Vec3{0, 0, out.supportAcceleration} * mass;
    // A correcao vem do chao: aplicada na pelve, ela precisa do giro que a
    // mesma forca daria la embaixo, nos pes (o pe freia, o tronco continua).
    Vec3 correctionLever {};
    // So para o desvio que veio de FORA (contato externo recente): o que o
    // proprio jogador causou (frear o sprint a 18 m/s2, mais do que um corpo
    // consegue) continua coberto pela ajuda de jogo.
    if (balanceShare < 0.999f && m_settings.correctionMode == 3 && !soles.empty() && m_perturbation > 0.001f) {
        Vec3 ground {};
        for (const Vec3& corner : soles) ground += corner;
        ground *= 1.0f / static_cast<float>(soles.size());
        ground.z = out.centerOfMassWorld.z - std::max(0.3f,
            out.centerOfMassWorld.z - (state.links[footLink[0]].position.z
                + state.links[footLink[1]].position.z) * 0.5f);
        correctionLever = cross(ground - root.position, correctionAcceleration * mass)
            * (saturate(m_perturbation) * (1.0f - balanceShare))
            // (De fora, so na proporcao da ajuda: a 0% o corpo empurrado se vira sozinho.)
            * (walkingAssist >= 1.0f ? 1.0f : walkingAssist);
        correctionLever.z = 0;
    }
    // Pes que tocam algo agora dividem a correcao (sem contato, nada: no ar
    // nao ha atrito).
    int touchingCount = 0;
    for (std::size_t side = 0; side < 2; ++side) touchingCount += touching[side] ? 1 : 0;
    for (std::size_t side = 0; side < 2; ++side) {
        out.footLinkIndex[side] = footLink[side] < profile.links.size()
            ? static_cast<std::uint32_t>(footLink[side]) : RagdollDynamics3D::InvalidIndex;
        out.footForceWorld[side] = touching[side] && m_settings.correctionMode == 2
            ? correctionAcceleration * (mass / static_cast<float>(touchingCount)) : Vec3 {};
        out.footTorqueWorld[side] = {};
    }
    const Vec3 error = rotationError(reference.rootOrientationWorld, root.orientation);
    const Vec3 rate = reference.rootAngularVelocityWorld - root.angularVelocity;
    const Vec3 torque = (error * 35 + rate * 5) * mass;
    const float tiltTorquePerKg = m_settings.v2 ? m_settings.v2TiltTorquePerKg : m_settings.maximumTiltTorquePerKg;
    Vec3 postureRaw = limit(planar(torque), mass * tiltTorquePerKg
        * (1.0f + std::max({ out.recoveryBoost, 0.75f * m_selfMotionCalm,
            m_settings.actionTorqueBoost * out.actionStability }))) * m_assistEnvelope;
    postureRaw.z = std::clamp(torque.z, -mass * m_settings.maximumYawTorquePerKg,
        mass * m_settings.maximumYawTorquePerKg);
    // Andando, walkingAssist limita diretamente a postura auxiliar. Parado,
    // postureShare ja faz a divisao pernas/pelve; aplicar o percentual aqui
    // de novo elevaria ao quadrado (30% viraria 9%).
    // (v2 - ou T_PostureFull no v1: o par de postura e um esforco interno - quadril
    // e tornozelo segurando o tronco -, nao ajuda de fora; andando ele nao
    // encolhe com a ajuda. Escalado por ela, a 0% andando ninguem segurava o
    // tronco: freando ou virando forte ele mergulhava.)
    static const float postureFull = tuneTempApc("T_PostureFull", 0.0f);  // TEMPTUNE
    Vec3 posture = postureRaw * (authority
        * (legsWalking ? std::max(walkingAssist, m_settings.v2 ? 1.0f : postureFull) : 1.0f));
    // No ar (sem apoio), o corpo tenta cair reto: par interno pelve x
    // pernas, modesto - so redistribui o giro que ele ja tem.
    Vec3 airPosture {};
    {
        const float air = 1.0f - out.supportAuthority;
        if (air > 0.0f && !intent.suspended && !intent.manipulated) {
            // Descendo depressa na horizontal, chega um pouco inclinado para
            // tras: o atrito no pe gira o corpo para a frente no toque (a
            // 7,5 m/s, reto no toque ele ia a 55 graus para a frente).
            Vec3 airError = error;
            const Vec3 travel = planar(root.linearVelocity);
            const float speed = travel.length();
            if (root.linearVelocity.z < 0.0f && speed > 2.0f) {
                const float lean = std::min(0.26f, 0.035f * speed);
                const Quaternion target = (Quaternion::fromAxisAngle(
                    cross(travel * (1.0f / speed), Vec3 { 0, 0, 1 }), lean)
                    * reference.rootOrientationWorld).normalized();
                airError = rotationError(target, root.orientation);
            }
            // So corrige o que uma pessoa corrigiria no ar: pleno ate ~20
            // graus fora da pose, nada a partir de ~45 (tombado de verdade, ou
            // logo depois de uma batida, ele cai - e espernea). Sem isso ele
            // endireitava 50 graus em meio segundo de queda, como um gato.
            const float x = saturate((out.tiltErrorDegrees - 20.0f) / 25.0f);
            // Atingido ha pouco (< ~0,8 s), nada: pulou e bateu em algo, cai.
            const float settled = saturate((m_sinceExternalHit - 0.8f) / 0.6f);
            const float humanly = (1.0f - x * x * (3 - 2 * x))
                * settled * settled * (3 - 2 * settled);
            airPosture = limit(planar(airError * 12.0f + rate * 2.5f) * mass,
                mass * m_settings.maximumAirTorquePerKg) * (air * humanly);
        }
    }
    // (Tentado: com a postura pelas pernas, levar este par pelos quadris -
    // pelve contra as coxas. Piorou a varredura de empurroes no modo so
    // pelas pernas, 25 -> 34 quedas: o "ar" aqui inclui o apoio parcial do
    // tropeco, onde o par se apoia no pe que ainda toca. Fica para a E7.)
    posture += airPosture;
    if (m_settings.postureMode == 0) posture = {};
    // E4: nas pernas a postura vira torque do chao nos pes (os tornozelos,
    // o centro de pressao dentro da sola), nao um par pelve x pes - a reacao
    // aplicada direto nos pes os fazia escorregar parados (~1,5 cm/s).
    // Andando, a postura se divide pela ajuda: a pelve fica com a parte da
    // ajuda (ja aplicada acima) e a perna de apoio segura o resto - com a
    // ajuda baixa a pelve tombava para tras, sem ninguem a segurar; com a
    // ajuda cheia nada muda (tudo na pelve, como antes).
    const float walkingLegPosture = postureWalk && legsWalking && legsCanCarry && brakeRunShare <= 0.001f
        && m_settings.jointSupport && m_settings.jointPosture
        ? (1.0f - walkingAssist) * (walkShareDecouple ? 1.0f : legsMaximumShare) : 0.0f;
    m_walkingPostureShare += (walkingLegPosture - m_walkingPostureShare) * std::min(1.0f, 7.0f * dt);
    const Vec3 legPosture = postureRaw * (legsAuthority * (postureShare + m_walkingPostureShare));
    out.legPostureRequested = legPosture;
    out.legPostureDelivered = {};
    posture = posture * (1.0f - postureShare);
    out.rootTorqueWorld = posture + correctionLever;
    out.perturbation = m_perturbation;
    out.linkTorqueWorld.assign(profile.links.size(), Vec3 {});
    out.correctionLeverWorld = correctionLever;
    // A reacao vai nos pes: nos que tocam, ou dividida entre os dois (no
    // ar o par so gira as pernas contra a pelve, como num corpo de verdade).
    // Maos apoiadas dividem a reacao com os pes.
    int bracedCount = 0;
    for (std::size_t side = 0; side < 2; ++side) {
        out.handLinkIndex[side] = RagdollDynamics3D::InvalidIndex;
        out.handTorqueWorld[side] = {};
        if (bracedLink[side] < profile.links.size() && m_settings.postureMode == 2) ++bracedCount;
    }
    const int supportCount = touchingCount + bracedCount;
    // (Movimento livre: a postura e de fora, na pelve so - sem reacao nas
    // maos nem nos pes.)
    const bool postureReaction = m_settings.postureMode == 2;
    for (std::size_t side = 0; side < 2; ++side) {
        if (bracedLink[side] >= profile.links.size() || !postureReaction
            || supportCount == 0) continue;
        out.handLinkIndex[side] = static_cast<std::uint32_t>(bracedLink[side]);
        out.handTorqueWorld[side] = posture * (-1.0f / static_cast<float>(supportCount));
    }
    for (std::size_t side = 0; side < 2; ++side) {
        if (out.footLinkIndex[side] == RagdollDynamics3D::InvalidIndex
            || !postureReaction) continue;
        const float share = supportCount > 0
            ? (touching[side] ? 1.0f / static_cast<float>(supportCount) : 0.0f) : 0.5f;
        out.footTorqueWorld[side] = posture * -share;
    }
    // O momento linear pelas pernas (E4): o chao sustenta o corpo inteiro e o
    // acelera pelos pes de apoio - os planejados em apoio que tocam de fato.
    // Forca total M (a + g), com a aceleracao pedida (altura, correcao de
    // desvio, intencao do jogador); o centro de pressao que da essa
    // aceleracao no pendulo invertido (a = g' (COM - CoP) / h) fica dentro do
    // poligono dos pes e do atrito - o que nao cabe, o corpo nao consegue. A
    // forca de cada pe passa pelo centro de massa (nao gira o corpo). As
    // juntas de cada perna fazem o torque que a transmite ate a pelve; nada
    // e aplicado na pelve.
    for (auto& torques : out.jointFeedforward) torques = { 0.0f, 0.0f, 0.0f };
    if (out.jointFeedforward.size() != profile.links.size())
        out.jointFeedforward.assign(profile.links.size(), { 0.0f, 0.0f, 0.0f });
    out.contactWrench = {};
    out.allocatedComAccelerationWorld = {};
    out.requestedComAccelerationWorld = {};
    out.allocationResidualWorld = {};
    out.observedComAccelerationWorld = physical != nullptr ? physical->centerOfMassAccelerationWorld : Vec3 {};
    out.jointSupportNewtons = 0.0f;
    out.jointFeedforwardActive = false;
    if (legsCarry && physical != nullptr && intent.contactPlan != nullptr
        && intent.contactPlan->valid && !intent.suspended && supported) {
        // (R1 do pacote v2: so o contato observado estrito deveria carregar
        // esforco. Medido antes de corrigir o voo curto da marcha, piorava -
        // 1,5 m/s 2 -> 5 quedas: a memoria de 50 ms segurava o pe de tras que
        // o voo tirava do chao. Volta depois da R4.)
        const auto observedSupport = [&](std::size_t side) {
            return physical->feet[side].supporting;
        };
        std::array<bool, 2> stance {};
        int stanceCount = 0;
        std::vector<Vec3> polygon;
        float ground = 0.0f;
        for (std::size_t side = 0; side < 2; ++side) {
            // Carrega peso o pe que esta apoiado DE FATO (contato medido): a
            // marca da passada do clipe solta o pe pelo clipe, com ele ainda
            // no chao carregando o corpo (no trote, "os dois no ar" com os
            // dois apoiados). O plano so exclui o pe que o planejador esta
            // tirando do chao.
            const FootPlan3D& plan = intent.contactPlan->feet[side];
            const bool plannerLifting = !plan.followingGait
                && (plan.phase == FootPhase3D::Swing || plan.phase == FootPhase3D::TouchdownSearch);
            stance[side] = observedSupport(side) && footLink[side] < profile.links.size()
                && !plannerLifting;
        }
        // O pe que o planejador ia tirar do chao e o unico apoiado de fato (o
        // corpo caiu sobre ele e o outro saiu do chao): ele e o apoio. Sem
        // isso as pernas ficavam sem apoio nenhum e o corpo desabava.
        if (!stance[0] && !stance[1])
            for (std::size_t side = 0; side < 2; ++side)
                stance[side] = observedSupport(side) && footLink[side] < profile.links.size();
        for (std::size_t side = 0; side < 2; ++side) {
            if (!stance[side]) continue;
            ++stanceCount;
            ground += physical->feet[side].soleCenterWorld.z;
            for (const Vec3& corner : physical->feet[side].soleCornersWorld) polygon.push_back(corner);
        }
        // O centro de pressao sai do peso inteiro (o equilibrio pelos pes nao
        // depende de quem carrega o peso); pelas juntas vai o peso que as
        // pernas assumem.
        // TEMPTUNE: correndo, o peso que as pernas carregam vem todo nos
        // apoios - dividido pela fracao do tempo com pe no chao (ate 2,5x),
        // como a ajuda no plano. Pelo peso de pe parado, o empurrao medio era
        // metade do peso: o resto vinha da perna cedendo (a mola das juntas
        // so empurra comprimida), a perna nao estendia na saida, o corpo
        // decolava sem subir e caia o voo inteiro (11 cm na curva).
        static const float legDutyNormalize = tuneTempApc("T_LegDutyNorm", 0.0f);
        float legDutyBoost = 1.0f;
        if (legDutyNormalize > 0.0f && legsWalking && !intent.airborne
            && brakeRunShare <= 0.001f && requestedVelocity.length() > 0.5f)
            legDutyBoost = 1.0f + legDutyNormalize
                * (std::min(2.5f, 1.0f / std::max(0.4f, m_contactDuty)) - 1.0f);
        const float verticalAcceleration = std::max(0.0f,
            Gravity * legsAuthority * (legsWeight > 0.0f ? std::max(weightShare, m_walkCarryShare) : 1.0f)
                * legDutyBoost + legVerticalAcceleration);
        const float appliedVertical = std::max(0.0f,
            Gravity * legsAuthority * weightByLegs * legDutyBoost + legVerticalAcceleration);
        if (stanceCount > 0 && verticalAcceleration > 0.0f) {
            ground /= static_cast<float>(stanceCount);
            const Vec3 com = physical->centerOfMassWorld;
            const float height = std::max(0.3f, com.z - ground);
            // Equilibrio parado (estrategia do tornozelo): o centro de massa
            // para o centro do apoio medido, a velocidade dele para zero. O que
            // nao cabe no centro de pressao e passo do planejador.
            Vec3 support {};
            for (const Vec3& corner : polygon) support += corner;
            support *= 1.0f / static_cast<float>(std::max<std::size_t>(1, polygon.size()));
            // Um pe descarregando para um passo: o centro de massa vai para o
            // outro (a troca de peso do planejador). Mirando o meio dos dois,
            // o equilibrio desfazia a troca, o pe saia carregado (pelo tempo)
            // e o corpo rolava sobre ele. (Medido tambem: so o deslocamento
            // que o planejador pede a pelve, 25-45% ate 5-14 cm, derivou mais
            // parado: 0,22 m contra 0,13.)
            // (A transferencia vem do planejador - a mesma que a referencia
            // corporal usa para mover o corpo.)
            bool unloadingForStep = false;
            const auto& transfer = intent.contactPlan->supportTransfer;
            if (stanceCount == 2 && transfer.outgoingFoot >= 0) {
                unloadingForStep = true;
                support = support + (transfer.supportPointWorld - support) * transfer.progress;
            }
            // So traz o centro de massa de volta quando ele sai de uma margem
            // dentro do apoio (o poligono encolhido 35% para o centro): parado,
            // uma pessoa nao centraliza o corpo, so o mantem na base. Mirando
            // o centro dos pes (a base em alerta e escalonada e a pose tem o
            // centro de massa em outro ponto), o equilibrio brigava com a
            // pose, jogava o peso inteiro num pe e depois no outro, e ele nao
            // parava de dar passos parado.
            std::vector<Vec3> inner = polygon;
            for (Vec3& corner : inner) corner = support + (corner - support) * 0.65f;
            Vec3 inside = closestInHull(inner, Vec3 { com.x, com.y, support.z });
            // (a troca de peso da descarga mira o ponto pedido, sem margem)
            if (unloadingForStep) inside = support;
            const float f = m_settings.legPlanarFrequency;
            // Andando, a troca de peso nao freia o movimento pedido: so a
            // velocidade atravessada e o que passa da pedida. (Freando tudo,
            // cada descarga pedia -3 a -4 m/s2 e a marcha perdia ~0,25 m/s por
            // passo - regime em ~57% do pedido.) Na arrancada (parado) e igual.
            Vec3 dampedVelocity = planar(physical->centerOfMassVelocityWorld);
            if (legsWalking) {
                const Vec3 wanted = requestedVelocity;
                const float wantedSpeed = wanted.length();
                if (wantedSpeed > 0.001f) {
                    const Vec3 direction = wanted * (1.0f / wantedSpeed);
                    dampedVelocity -= direction * std::clamp(dot(dampedVelocity, direction), 0.0f, wantedSpeed);
                }
            }
            // (Andando nao - o centro de massa vai a frente dos pes de
            // proposito -, a nao ser a troca de peso na descarga de um passo:
            // sem ela o primeiro passo da arrancada saia carregado e arrastava.)
            const Vec3 balance = legsWalking && !unloadingForStep ? Vec3 {} : limit(planar(inside - com) * (f * f)
                    - dampedVelocity * (1.8f * f),
                m_settings.frictionAccelerationLimit)
                * (out.supportAuthority * out.uprightAuthorityShare * balanceShare);
            const Vec3 planarAcceleration = planar(legPlanarAcceleration) + balance;
            out.requestedComAccelerationWorld = planarAcceleration;
            // Centro de pressao pedido, dentro dos pes e do atrito.
            Vec3 cop = com - planarAcceleration * (height / verticalAcceleration);
            cop.z = ground;
            cop = closestInHull(polygon, cop);
            Vec3 lever { com.x - cop.x, com.y - cop.y, 0.0f };
            const float friction = 0.8f * height;
            if (lever.length() > friction) {
                lever = lever * (friction / lever.length());
                cop = { com.x - lever.x, com.y - lever.y, ground };
            }
            const float fz = mass * verticalAcceleration;
            const Vec3 force { lever.x * (fz / height), lever.y * (fz / height), mass * appliedVertical };
            out.allocatedComAccelerationWorld = Vec3 { force.x, force.y, fz } * (1.0f / mass) - Vec3 { 0.0f, 0.0f, Gravity };
            out.allocationResidualWorld = planar(out.requestedComAccelerationWorld)
                - planar(out.allocatedComAccelerationWorld);
            // Dividido entre os pes: a parte de cada um e onde o CoP cai
            // entre eles; o resto do CoP (de lado) vai igual nos dois.
            std::array<ContactWrench3D, 2> wrenches {};
            std::size_t count = 0;
            std::array<float, 2> share { stance[0] ? 1.0f : 0.0f, stance[1] ? 1.0f : 0.0f };
            Vec3 offset = cop - physical->feet[stance[0] ? 0 : 1].soleCenterWorld;
            if (stanceCount == 2) {
                const Vec3 p0 = physical->feet[0].soleCenterWorld, p1 = physical->feet[1].soleCenterWorld;
                Vec3 axis = p1 - p0;
                axis.z = 0.0f;
                const float length = axis.lengthSquared();
                const float t = length > 1e-4f
                    ? std::clamp(dot(planar(cop - p0), axis) / length, 0.0f, 1.0f) : 0.5f;
                share = { 1.0f - t, t };
                offset = cop - (p0 + (p1 - p0) * t);
            }
            offset.z = 0.0f;
            // A forca de lado de cada pe nao passa do atrito da carga que ele
            // tem DE FATO (a do plano, pela alavanca do centro de massa), com
            // margem: pedida pela carga desejada, o pe mais leve escorregava
            // e o corpo ia junto (3 cm/s parado, sem parar).
            const std::array<float, 2> realShare = stanceCount == 2
                ? intent.contactPlan->loadShare : std::array<float, 2> { share[0], share[1] };
            for (std::size_t side = 0; side < 2; ++side) {
                if (!stance[side]) continue;
                ContactWrench3D wrench;
                wrench.linkIndex = static_cast<std::uint32_t>(footLink[side]);
                wrench.pointWorld = physical->feet[side].soleCenterWorld + offset;
                wrench.forceWorld = force * share[side];
                const float traction = 0.5f * mass * Gravity * std::clamp(realShare[side], 0.0f, 1.0f);
                Vec3 sideways { wrench.forceWorld.x, wrench.forceWorld.y, 0.0f };
                if (sideways.length() > traction) {
                    sideways = sideways * (traction / std::max(1e-4f, sideways.length()));
                    wrench.forceWorld.x = sideways.x;
                    wrench.forceWorld.y = sideways.y;
                }
                // Postura: o torque do chao nos pes de apoio, dividido igual,
                // sem limite pela sola - o contato e o orcamento das juntas
                // limitam. Com os dois no chao a cadeia e fechada e o excesso
                // vira troca de carga entre os pes; num pe so, o pe comeca a
                // virar na quina (fisico, visivel). Medido na varredura de
                // 216 empurroes (modo so pelas pernas): limitar pelo centro de
                // pressao de cada sola deu 86 quedas; so em apoio simples,
                // 79; sem limite, 41.
                const float normal = mass * Gravity * std::clamp(realShare[side], 0.0f, 1.0f);
                const Vec3 tilt = Vec3 { legPosture.x, legPosture.y, 0.0f }
                    * (1.0f / static_cast<float>(stanceCount));
                const float yaw = std::clamp(legPosture.z / static_cast<float>(stanceCount),
                    -0.08f * normal, 0.08f * normal);
                wrench.torqueWorld = { tilt.x, tilt.y, yaw };
                out.contactWrench[side] = wrench;
                out.legPostureDelivered += wrench.torqueWorld;
                wrenches[count++] = wrench;
            }
            // Giro (em volta da vertical) com os dois pes no chao: o que a
            // torcao das solas nao da vem de um binario - um pe empurra para
            // um lado, o outro para o outro (d x F = M). So pela torcao
            // (~60 N.m), a caixa empurrando o peito de lado girava o corpo
            // quase 180 graus.
            // (Agarrado pela PhysGun, nao: quem agarra gira o corpo e ele cede
            // - brigando pelo rumo, o arrasto caia mais, 3 -> 6 em 32.)
            if (count == 2 && !intent.manipulated) {
                Vec3 span = wrenches[1].pointWorld - wrenches[0].pointWorld;
                span.z = 0.0f;
                const float length2 = span.lengthSquared();
                const float remaining = legPosture.z
                    - (wrenches[0].torqueWorld.z + wrenches[1].torqueWorld.z);
                if (length2 > 0.01f && std::abs(remaining) > 1.0f) {
                    Vec3 couple = cross(Vec3 { 0.0f, 0.0f, 1.0f }, span) * (remaining / length2);
                    const float coupleLimit = 0.5f * mass * Gravity
                        * std::clamp(std::min(realShare[0], realShare[1]), 0.0f, 1.0f);
                    if (couple.length() > coupleLimit)
                        couple = couple * (coupleLimit / couple.length());
                    wrenches[1].forceWorld += couple;
                    wrenches[0].forceWorld -= couple;
                    for (std::size_t i = 0; i < 2; ++i)
                        out.contactWrench[wrenches[i].linkIndex == static_cast<std::uint32_t>(footLink[0]) ? 0 : 1] = wrenches[i];
                }
            }
            m_wholeBody.addContactTorques(state, std::span<const ContactWrench3D>(wrenches.data(), count),
                out.jointFeedforward);
            out.jointSupportNewtons = mass * appliedVertical;
            out.jointFeedforwardActive = true;
        }
    }
    out.jointFeedforwardActive = out.jointFeedforwardActive || out.jointSupportNewtons > 0.0f;
    const Vec3 proxyError = planar(root.position - intent.navigationRootWorld);
    out.proxyErrorMeters = proxyError.length();
    out.proxyFollowVelocityWorld = limit(proxyError * m_settings.proxyFollowRate,
        m_settings.maximumProxyFollowSpeed);
    out.valid = true;
}
} // namespace MatterEngine
