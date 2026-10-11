#include "Engine/Athlete/Actions/BallPlay3D.hpp"

#include <algorithm>
#include <cmath>

namespace MatterEngine {
namespace {

Vec3 planar(Vec3 v) { return { v.x, v.y, 0.0f }; }

float signedAngle(Vec3 from, Vec3 to) {
    return std::atan2(from.x * to.y - from.y * to.x, from.x * to.x + from.y * to.y);
}

Vec3 clampLength(Vec3 v, float maximum) {
    const float length = v.length();
    return length > maximum && length > 1e-9f ? v * (maximum / length) : v;
}

float smooth(float s) {
    s = std::clamp(s, 0.0f, 1.0f);
    return s * s * (3.0f - 2.0f * s);
}

// Para onde aponta o joelho: perpendicular a perna, no plano dela com o
// vertical, do lado da frente, girado no eixo da perna o quanto a ponta do pe
// gira em relacao ao corpo.
Vec3 kneeDirection(Vec3 leg, Vec3 forward, Vec3 toe) {
    const float length = leg.length();
    if (length < 1e-4f) return forward;
    const Vec3 along = leg * (1.0f / length);
    const Vec3 knee = cross(along, cross(Vec3 { 0.0f, 0.0f, 1.0f }, forward));
    if (knee.lengthSquared() < 1e-6f) return forward;
    const float yaw = toe.lengthSquared() > 1e-6f ? std::clamp(signedAngle(forward, toe), -1.6f, 1.6f) : 0.0f;
    return Quaternion::fromAxisAngle(along * -1.0f, yaw).rotate(knee.normalized());
}

// Quanto falta para o clipe pousar o pe (s; pelo ritmo de agora).
float clipTouchdown(const BallHandlingGait3D& gait, std::size_t side) {
    const float swing = std::max(0.05f, gait.swingSeconds[side]);
    return gait.planted[side] ? std::max(0.0f, gait.secondsToLiftoff[side]) + swing
                              : std::max(0.0f, gait.secondsToTouchdown[side]);
}

} // namespace

bool BallPlay3D::inArea(Vec3 ball, float radius, Vec3 pelvis, const BallPlaySettings3D& settings, Vec3 velocity) {
    if (ball.z - radius > pelvis.z + settings.waistAbovePelvis) return false;
    // (Andando, a area tambem vai a frente no rumo do movimento - a de sempre
    // mais a parte a frente: correndo, a bola um pouco a frente, a do toque
    // anterior, ainda vale; atras nada se perde.)
    if (std::hypot(ball.x - pelvis.x, ball.y - pelvis.y) <= settings.areaRadius) return true;
    const Vec3 flat { velocity.x, velocity.y, 0.0f };
    const float speed = flat.length();
    if (speed <= 0.3f) return false;
    const Vec3 lead = flat * (settings.areaLead * std::min(1.0f, speed / std::max(0.5f, settings.areaLeadSpeed)) / speed);
    return std::hypot(ball.x - pelvis.x - lead.x, ball.y - pelvis.y - lead.y) <= settings.areaRadius;
}

void BallPlay3D::reset(const RagdollProfile3D& profile) {
    m_legs = resolveAthleteLegs3D(profile);
    m_geometry = resolveAthleteFootGeometry3D(profile, m_legs, 0.08f);
    m_ready = m_legs.valid && m_geometry.valid;
    if (m_ready) {
        // A altura do tornozelo com o pe plano no chao.
        for (std::size_t s = 0; s < 2; ++s) {
            const Quaternion flat = m_geometry.flat[s];
            const float sole = std::min(flat.rotate(m_geometry.heel[s]).z, flat.rotate(m_geometry.toeBase[s]).z);
            m_ankleHeight[s] = flat.rotate(m_legs.ankleInFoot[s]).z - sole;
        }
    }
    m_queue.clear();
    m_move.stop();
    m_sinceInArea = 10.0f;
    m_cooldown = 0.0f;
    m_busyWait = 0.0f;
    m_busyRecord = 0;
    m_interrupting = false;
    m_gaitRate = m_gaitRateWanted = 1.0f;
    m_gaitCoupled = false;
    m_tail = 0.0f;
    m_support = -1;
    m_quickPlant = false;
    m_bodyLive = false;
    m_appliedShift = {};
    m_appliedDrop = 0.0f;
    m_settleLeft = 0.0f;
    m_styleFootKnown = false;
}

std::uint64_t BallPlay3D::targetBallKey() const {
    if (const BallActionRecord3D* record = m_queue.active()) return record->ballKey;
    // (Enquanto o corpo sai do movimento, o olhar ainda segue a bola dele.)
    return m_move.active() || m_bodyLive ? m_ballKey : 0;
}

BallMoveLive3D BallPlay3D::live(const BallHandlingInput3D& input, int side) const {
    BallMoveLive3D now;
    now.pelvisGround = { input.body.pelvis.x, input.body.pelvis.y, input.body.groundHeight };
    now.facingYaw = input.body.facingYaw;
    now.bodyVelocity = planar(input.bodyVelocity);
    now.hip = input.body.hip[static_cast<std::size_t>(std::clamp(side, 0, 1))];
    now.support = input.body.foot[static_cast<std::size_t>(1 - std::clamp(side, 0, 1))];
    now.supportKnown = true;
    return now;
}

void BallPlay3D::startMove(const BallHandlingInput3D& input, const BallPlaySettings3D& settings) {
    const BallPlannerBody3D& body = input.body;
    const BallHandlingGait3D& gait = input.gait;
    const Vec3 forward { std::cos(body.facingYaw), std::sin(body.facingYaw), 0.0f };
    const Vec3 left { -forward.y, forward.x, 0.0f };
    // O rumo do toque: o das teclas (o mesmo da forca); sem tecla, o pedido.
    Vec3 direction = planar(input.aheadVelocity).length() > 0.3f ? planar(input.aheadVelocity) : planar(m_intent.direction);
    if (m_intent.kind == BallActionKind3D::Shot)
        direction = { std::cos(m_intent.aimYawRadians), std::sin(m_intent.aimYawRadians), 0.0f };
    // (Perto do meio, a perna livre: a que esta no ar, ou a que sai primeiro.)
    int freeSide = -1;
    if (gait.cycling) {
        if (!gait.planted[0] && gait.planted[1]) freeSide = 0;
        else if (!gait.planted[1] && gait.planted[0]) freeSide = 1;
    }
    const float lateral = input.ball.valid ? dot(planar(input.ball.position - body.pelvis), left) : 0.0f;
    const BallMoveChoice3D choice = chooseBallMove3D(m_intent.kind, direction, body.facingYaw, lateral, freeSide);
    const std::size_t s = static_cast<std::size_t>(choice.side);
    BallMoveStart3D begin;
    begin.choice = choice;
    begin.power = m_intent.power;
    begin.foot = body.foot[s];
    begin.footOrientation = body.footOrientation[s];
    // (O pe apoiado parte parado; no ar, com a velocidade do pe do estilo.)
    begin.footVelocity = gait.planted[s] || !gait.cycling ? Vec3 {} : m_styleFootVelocity[s];
    begin.footPlanted = gait.planted[s] || !gait.cycling;
    begin.pelvisGround = { body.pelvis.x, body.pelvis.y, body.groundHeight };
    begin.facingYaw = body.facingYaw;
    begin.bodyVelocity = planar(input.bodyVelocity);
    begin.ankleInFoot = m_legs.ankleInFoot[s];
    begin.flatFoot = m_geometry.flat[s];
    begin.ankleHeight = m_ankleHeight[s];
    begin.legLength = m_legs.thighLength[s] + m_legs.shinLength[s];
    // Andando, o movimento dura ate o pouso daquele pe no clipe (entre os
    // limites): o pe do toque pousa no ritmo da passada, sem esperar no ar.
    if (gait.cycling)
        begin.totalSeconds = std::clamp(clipTouchdown(gait, s), settings.moveMinimumSeconds, settings.moveMaximumSeconds);
    m_move.start(begin, settings.move);
    m_kneeStart = body.knee[s];
    m_bodySide = choice.side;
    m_support = 1 - choice.side;
    m_interrupting = false;
    m_settleLeft = 0.0f;
    // Andando: o movimento entra na passada (o ritmo do clipe) e, com o pe do
    // toque apoiado e o outro no ar, o outro pousa ja.
    m_gaitCoupled = gait.cycling;
    m_clipSwung = !gait.planted[s];
    m_tail = 0.0f;
    m_quickPlant = gait.cycling && gait.planted[s] && !gait.planted[static_cast<std::size_t>(m_support)];
}

void BallPlay3D::applyForce(const BallHandlingInput3D& input, const BallOutcomeSettings3D& outcome,
    const BallPlaySettings3D& settings, BallHandlingOutput3D& output) {
    BallOutcomeRequest3D intent = m_intent;
    // O toque vai para onde o jogador vai agora (as teclas na hora da forca).
    if (intent.kind != BallActionKind3D::Shot && planar(input.aheadVelocity).length() > 0.3f)
        intent.direction = planar(input.aheadVelocity).normalized();
    // A parte do pe (so a sola muda a conta): a do movimento.
    intent.surface = m_move.choice().surface;
    // A velocidade "a frente" e a do corpo daqui a carryAhead, como no manejo
    // V3 (onde a forca foi calibrada).
    Vec3 carry = planar(input.bodyVelocity);
    if (input.animatedBody) {
        const Vec3 change = planar(input.aheadVelocity) - carry;
        const float amount = change.length();
        if (amount > 1e-3f)
            carry += change * std::min(1.0f, settings.carryAcceleration * std::max(0.0f, outcome.carryAhead) / amount);
    }
    const BallOutcome3D result = computeBallOutcome3D(intent, input.ball.velocity, input.bodyVelocity, carry,
        input.ball.radius, outcome);
    const float inertia = (2.0f / 3.0f) * input.ball.mass * input.ball.radius * input.ball.radius;
    output.impulse = true;
    output.impulseBallKey = m_ballKey;
    output.linearImpulse = (result.velocity - input.ball.velocity) * input.ball.mass;
    output.angularImpulse = (result.spin - input.ball.spin) * inertia;
    output.outcomeVelocity = result.velocity;
    output.touched = true;
}

void BallPlay3D::update(const BallHandlingInput3D& input, const BallOutcomeSettings3D& outcome,
    const BallPlaySettings3D& settings, BallHandlingOutput3D& output) {
    output = {};
    if (!m_ready) return;
    const float dt = std::max(input.deltaTime, 1e-4f);
    // Os pes do estilo andam: a velocidade deles (o movimento parte do pe no
    // ar com ela), filtrada.
    for (std::size_t s = 0; s < 2; ++s) {
        const Vec3 raw = m_styleFootKnown ? (input.gait.styleFoot[s] - m_styleFootPrevious[s]) * (1.0f / dt) : Vec3 {};
        m_styleFootVelocity[s] = m_styleFootKnown
            ? m_styleFootVelocity[s] + (raw - m_styleFootVelocity[s]) * std::min(1.0f, dt / settings.styleVelocitySeconds)
            : raw;
        m_styleFootPrevious[s] = input.gait.styleFoot[s];
    }
    m_styleFootKnown = true;
    m_cooldown = std::max(0.0f, m_cooldown - dt);
    const bool ballValid = input.ball.valid;
    const bool inside = ballValid && inArea(input.ball.position, input.ball.radius, input.body.pelvis, settings,
        input.bodyVelocity);
    m_sinceInArea = inside ? 0.0f : m_sinceInArea + dt;
    const bool reachable = ballValid && (inside || m_sinceInArea <= settings.guaranteeGrace);

    // 1) O pedido novo (a bola mais perto, travada): substitui o guardado.
    if (input.request) m_queue.push(input.intent, input.requestBallKey);
    m_queue.advance(dt);
    if (const BallActionRecord3D* active = m_queue.active(); active == nullptr || active->id != m_busyRecord) {
        m_busyWait = 0.0f;
        m_busyRecord = active != nullptr ? active->id : 0;
    }

    // 2) O pedido guardado: com a bola na area, sai na hora - a forca e o
    // movimento (a garantia); passado o tempo guardado sem ela entrar, acaba.
    // Ocupado (o movimento anterior antes do pouso, o intervalo), o tempo
    // guardado para.
    if (BallActionRecord3D* record = m_queue.active(); record != nullptr && record->stage == BallActionStage3D::Seeking) {
        const bool busy = m_cooldown > 0.0f || (m_move.active() && m_move.phase() != BallMovePhase3D::Land);
        if (busy) m_busyWait += dt;
        if (!ballValid || input.ball.key != record->ballKey) {
            m_queue.finish(BallActionOutcome3D::InvalidTarget, "a bola do pedido sumiu");
        } else if (!busy && reachable) {
            m_intent = record->intent;
            m_ballKey = record->ballKey;
            m_requestedAtAccept = planar(input.aheadVelocity);
            startMove(input, settings);
            applyForce(input, outcome, settings, output);
            m_cooldown = settings.cooldown;
            record->stage = BallActionStage3D::Committed;
            record->latency = std::max(0.0f, record->age - dt);
            m_queue.finish(BallActionOutcome3D::Touched, inside ? "a bola na area" : "a bola saindo da area");
        } else if (record->age - m_busyWait > settings.requestBuffer) {
            m_queue.finish(BallActionOutcome3D::Expired, "a bola nao entrou na area no tempo guardado");
        }
    }

    // 3) O movimento. Depois da forca, o movimento pedido mudando (arrancar,
    // parar, virar, inverter): ele vai direto ao pouso, no rumo novo.
    bool landedNow = false;
    if (m_move.active()) {
        const int side = m_move.side();
        if (!m_interrupting) {
            const Vec3 now = planar(input.aheadVelocity);
            const float a = now.length(), b = m_requestedAtAccept.length();
            const bool turned = a > 1.0f && b > 1.0f && std::abs(signedAngle(m_requestedAtAccept, now)) > settings.interruptTurn;
            if (turned || (now - m_requestedAtAccept).length() > settings.interruptSpeed) {
                m_interrupting = true;
                m_move.hurry(live(input, side), settings.move);
            }
        }
        m_move.update(dt, live(input, side), settings.move);
        landedNow = m_move.landed();
        if (landedNow) {
            m_lastSide = side;
            m_lastAnkle = m_move.ankle();
            m_lastFoot = m_move.orientation();
            // (Com as pegadas o pe fica preso por elas no pouso e so a pose
            // da perna sai do gesto aos poucos - de uma vez, o joelho trocava
            // de lado e o pe do corpo saltava; sem elas, o pe tambem.)
            m_settleLeft = input.footsteps ? settings.handOverStepsSeconds : settings.handOverSeconds;
            m_settleSpan = m_settleLeft;
        }
    }

    // 4) O ritmo da passada: o clipe pousa o pe do toque quando o movimento
    // pousa (a outra perna pisa mais rapido, ou espera). Depois do pouso do
    // movimento, ainda ate o pouso do clipe (gaitTailSeconds no maximo).
    m_gaitRateWanted = 1.0f;
    if (m_gaitCoupled && input.gait.cycling) {
        const std::size_t s = static_cast<std::size_t>(std::clamp(m_bodySide, 0, 1));
        if (!input.gait.planted[s]) m_clipSwung = true;
        const bool clipLanded = m_clipSwung && input.gait.planted[s];
        if (!m_move.active()) m_tail += dt;
        if (clipLanded || m_tail > settings.gaitTailSeconds) {
            m_gaitCoupled = false;
        } else {
            // (O tempo do clipe vem no ritmo de agora: no ritmo normal, vezes o
            // fator aplicado.)
            const float clip = clipTouchdown(input.gait, s) * m_gaitRate;
            const float remaining = m_move.active() ? m_move.timeToLanding() : 0.0f;
            m_gaitRateWanted = remaining > 0.02f
                ? std::clamp(clip / remaining, settings.gaitRateMinimum, settings.gaitRateMaximum)
                : settings.gaitRateMaximum;
        }
    } else {
        m_gaitCoupled = false;
    }
    m_gaitRate += (m_gaitRateWanted - m_gaitRate) * std::min(1.0f, dt / std::max(0.005f, settings.gaitRateSeconds));
    output.gaitRateScale = m_gaitRate;

    // 5) O apoio: o outro pe fica no chao durante o movimento (e pousa ja, se
    // estava no ar com o do toque apoiado).
    // (Parado, o apoio fica no chao durante o movimento; andando, nao - o
    // corpo vai embora e a perna presa esticava: as pegadas o levam pela
    // passada, com a saida pelo alcance.)
    if (m_move.active()) {
        if (!input.gait.cycling) output.reservedSide = m_support;
        if (m_quickPlant && m_move.phase() == BallMovePhase3D::Strike) output.quickPlantSide = m_support;
    } else {
        m_quickPlant = false;
    }

    bodyTargets(input, settings, output);
    output.interacting = m_queue.active() != nullptr || m_move.active();
    output.interactingBallKey = targetBallKey();
}

void BallPlay3D::bodyTargets(const BallHandlingInput3D& input, const BallPlaySettings3D& settings,
    BallHandlingOutput3D& output) {
    const float dt = std::max(input.deltaTime, 1e-4f);
    const BallPlannerBody3D& body = input.body;
    const Vec3 forward { std::cos(body.facingYaw), std::sin(body.facingYaw), 0.0f };
    const Vec3 left { -forward.y, forward.x, 0.0f };
    const Vec3 up { 0.0f, 0.0f, 1.0f };
    const bool acting = m_move.active();
    // (Sem as pegadas, a perna sai do pouso aos poucos para a locomocao.)
    const bool settling = !acting && m_settleLeft > 0.0f;
    if (settling) m_settleLeft = std::max(0.0f, m_settleLeft - dt);
    const float wantedWeight = acting ? 1.0f : 0.0f;
    const int side = m_bodySide;
    const int support = 1 - side;
    if (!m_bodyLive && !acting && !settling) {
        m_appliedShift = {};
        m_appliedDrop = 0.0f;
        return;
    }

    // Os alvos: o centro de massa sobre o apoio (parado), a pelve abaixa um
    // pouco e abre para o rumo do toque, o tronco inclina para o lado
    // contrario da perna, os bracos equilibram pelo balanco do pe e o olhar
    // vai na bola.
    const std::size_t sup = static_cast<std::size_t>(support);
    const Vec3 base = input.centerOfMass - m_appliedShift * settings.pelvisMassShare;
    const Vec3 shift = clampLength(planar(body.foot[sup] - base) * (1.0f / std::max(0.3f, settings.pelvisMassShare)),
        settings.maximumPelvisShift);
    const bool shot = m_move.choice().touch == BallMoveTouch3D::Shot;
    const float drop = shot ? 0.06f : 0.045f;
    const float sideSign = side == 0 ? 1.0f : -1.0f;
    const Vec3 push = m_move.push();
    const float yaw = std::clamp(signedAngle(forward, push) * 0.2f, -0.25f, 0.25f);
    const float bend = shot && acting ? 0.08f : 0.04f;
    const float swing = std::clamp(0.15f + (acting ? m_move.velocity().length() : 0.0f) / 16.0f, 0.0f, 0.85f);
    const float open = m_armSwing.value();
    const Vec3 outwardSupport = left * (support == 0 ? 1.0f : -1.0f);
    const Vec3 outwardKick = left * sideSign;
    std::array<Vec3, 2> wrist {};
    wrist[sup] = forward * (0.12f + 0.20f * open) + outwardSupport * (0.22f + 0.25f * open) - up * (0.42f - 0.30f * open);
    wrist[static_cast<std::size_t>(side)] = forward * -(0.06f + 0.22f * open) + outwardKick * (0.18f + 0.12f * open)
        - up * (0.44f - 0.18f * open);
    // O olhar: na bola, no maximo 60 graus para o lado da frente do corpo e a
    // pelo menos 0,5 m da pelve.
    Vec3 look = m_look.value();
    if (input.ball.valid) {
        const Vec3 offset = planar(input.ball.position - body.pelvis);
        const float distance = std::max(0.5f, offset.length());
        const float raw = offset.lengthSquared() > 1e-6f ? signedAngle(forward, offset) : 0.0f;
        const float lookYaw = std::clamp(raw, -1.05f, 1.05f) * (1.0f - smooth((std::abs(raw) - 1.05f) / 1.05f));
        const Vec3 direction = forward * std::cos(lookYaw) + left * std::sin(lookYaw);
        look = Vec3 { body.pelvis.x, body.pelvis.y, input.ball.position.z } + direction * distance;
    }
    if (!m_bodyLive) {
        m_bodyLive = true;
        m_weight.reset(0.0f);
        m_shift.reset(shift);
        m_drop.reset(drop);
        m_yaw.reset(yaw);
        m_trunkSide.reset(-sideSign * 0.10f);
        m_trunkBend.reset(bend);
        m_armSwing.reset(swing);
        m_wrist[0].reset(wrist[0]);
        m_wrist[1].reset(wrist[1]);
        m_look.reset(look);
        m_lookWeight.reset(input.ball.valid ? 0.85f : 0.0f);
    }
    m_weight.step(wantedWeight, settings.prepareSeconds, dt);
    m_shift.step(shift, settings.targetSeconds, dt);
    m_drop.step(drop, settings.targetSeconds, dt);
    m_yaw.step(yaw, settings.targetSeconds, dt);
    m_trunkSide.step(-sideSign * 0.10f, settings.targetSeconds, dt);
    m_trunkBend.step(bend, settings.targetSeconds, dt);
    m_armSwing.step(swing, settings.armSeconds, dt);
    m_wrist[0].step(wrist[0], settings.targetSeconds, dt);
    m_wrist[1].step(wrist[1], settings.targetSeconds, dt);
    m_look.step(look, settings.lookSeconds, dt);
    m_lookWeight.step(input.ball.valid ? 0.85f : 0.0f, settings.lookSeconds, dt);
    const float weight = std::clamp(m_weight.value(), 0.0f, 1.0f);
    if (weight <= 0.001f && !acting && !settling) {
        m_bodyLive = false;
        m_appliedShift = {};
        m_appliedDrop = 0.0f;
        return;
    }

    BallBodyTargets3D& t = output.body;
    t.active = true;
    t.side = side;
    t.weight = weight;
    if (acting) {
        // A perna inteira no movimento (sem mistura: ele parte do pe da pose,
        // com a velocidade dele).
        const std::size_t s = static_cast<std::size_t>(side);
        t.ankle = m_move.ankle();
        t.foot = m_move.orientation();
        t.knee = kneeDirection(t.ankle - body.hip[s], forward, planar(m_move.orientation().rotate({ 1.0f, 0.0f, 0.0f })));
        {
            // (O polo sai do joelho da pose aos poucos.)
            const float u = smooth(m_move.time() / std::max(0.01f, settings.kneeBlendSeconds));
            const Vec3 blended = m_kneeStart + (t.knee - m_kneeStart) * u;
            if (blended.lengthSquared() > 1e-6f) t.knee = blended.normalized();
        }
        t.legWeight = 1.0f;
        output.actingSide = side;
        m_lastKnee = t.knee;
    } else if (settling) {
        // Sem as pegadas: do pouso para a locomocao, aos poucos.
        t.side = m_lastSide;
        t.ankle = m_lastAnkle;
        t.foot = m_lastFoot;
        t.knee = m_lastKnee;
        t.legWeight = smooth(m_settleLeft / std::max(0.02f, m_settleSpan));
        // (Com as pegadas o pe ja e delas: o lado nao age mais.)
        if (!input.footsteps) output.actingSide = m_lastSide;
    }
    // (Andando, a corrida manda no corpo: o do gesto pesa menos.)
    const float share = 1.0f - settings.runningBodyShare * std::clamp(input.gait.movingShare, 0.0f, 1.0f);
    t.pelvisShift = m_shift.value() * share;
    t.pelvisDrop = m_drop.value() * share;
    t.pelvisYaw = m_yaw.value() * share;
    t.trunkTwist = -0.7f * m_yaw.value() * share;
    t.trunkSide = m_trunkSide.value() * share;
    t.trunkBend = m_trunkBend.value() * share;
    t.wrist = { m_wrist[0].value(), m_wrist[1].value() };
    // (Andando, os bracos do gesto pesam a metade: eles brigavam com o
    // balanco da corrida - as maos davam o tranco.)
    const float arms = share * (1.0f - 0.5f * std::clamp(input.gait.movingShare, 0.0f, 1.0f));
    t.armWeight = { arms, arms };
    t.look = m_look.value();
    // (Correndo, o olhar pesa menos: a bola passando de um lado ao outro
    // virava a cabeca de supetao.)
    t.lookWeight = std::clamp(m_lookWeight.value(), 0.0f, 1.0f)
        * (1.0f - 0.4f * std::clamp(input.gait.movingShare, 0.0f, 1.0f));
    // (As pernas nao se cruzam pelo desenho do movimento - cada pe do seu
    // lado - e pelas pegadas.)
    t.separateWeight = 0.0f;
    m_appliedShift = planar(t.pelvisShift) * weight;
    m_appliedDrop = t.pelvisDrop * weight;
}

} // namespace MatterEngine
