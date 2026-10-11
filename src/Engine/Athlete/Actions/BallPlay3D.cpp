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

// O polo do joelho de uma perna no movimento: pelo tornozelo e a ponta do pe,
// indo ao joelho do clipe durante a volta.
Vec3 moveKnee(const BallMove3D& move, Vec3 hip, Vec3 forward, Vec3 clipKnee) {
    Vec3 knee = kneeDirection(move.ankle() - hip, forward, planar(move.orientation().rotate({ 1.0f, 0.0f, 0.0f })));
    const float r = smooth(move.rejoinProgress());
    if (r > 0.0f && clipKnee.lengthSquared() > 1e-6f) {
        const Vec3 blended = knee + (clipKnee.normalized() - knee) * r;
        if (blended.lengthSquared() > 1e-6f) knee = blended.normalized();
    }
    return knee;
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
    m_previous.stop();
    m_sinceInArea = 10.0f;
    m_cooldown = 0.0f;
    m_busyWait = 0.0f;
    m_busyRecord = 0;
    m_interrupting = false;
    m_support = -1;
    m_speedScale = 1.0f;
    m_bodyLive = false;
    m_appliedShift = {};
    m_appliedDrop = 0.0f;
    m_settleLeft = 0.0f;
    m_lastJoined = false;
    m_feetKnown = false;
}

float BallPlay3D::random() {
    // (xorshift: o mesmo sorteio a cada execucao, outro a cada toque.)
    m_seed ^= m_seed << 13;
    m_seed ^= m_seed >> 17;
    m_seed ^= m_seed << 5;
    return static_cast<float>(m_seed & 0xFFFFFFu) / 16777215.0f;
}

std::uint64_t BallPlay3D::targetBallKey() const {
    if (const BallActionRecord3D* record = m_queue.active()) return record->ballKey;
    // (Enquanto o corpo sai do movimento, o olhar ainda segue a bola dele.)
    return m_move.active() || m_bodyLive ? m_ballKey : 0;
}

BallMoveLive3D BallPlay3D::live(const BallHandlingInput3D& input, int side) const {
    const std::size_t s = static_cast<std::size_t>(std::clamp(side, 0, 1));
    BallMoveLive3D now;
    // (O corpo vem da pose do tick anterior: um passo a frente - parado nele,
    // no primeiro tick do movimento o pe nao andava com o corpo e no segundo
    // andava o dobro.)
    const Vec3 ahead = planar(input.bodyVelocity) * input.deltaTime;
    now.pelvisGround = Vec3 { input.body.pelvis.x, input.body.pelvis.y, input.body.groundHeight } + ahead;
    now.facingYaw = input.body.facingYaw;
    now.bodyVelocity = planar(input.bodyVelocity);
    now.hip = input.body.hip[s] + ahead;
    now.support = input.body.foot[1 - s];
    now.supportKnown = true;
    // O pe do clipe: so com a pose do padrao (no fisico os pes sao do corpo e
    // o gesto termina no pouso). Ele vem do tick anterior: um passo a frente.
    now.clipKnown = input.animatedBody || input.physicalBody;
    now.clipAnkle = m_clipAnkle[s] + m_clipAnkleVelocity[s] * input.deltaTime;
    now.clipVelocity = m_clipAnkleVelocity[s];
    now.clipAcceleration = m_clipAnkleAcceleration[s];
    now.clipOrientation = input.gait.clipFootOrientation[s];
    now.clipPlanted = !input.gait.cycling || input.gait.planted[s];
    now.clipSecondsToTouchdown = input.gait.secondsToTouchdown[s];
    now.cycling = input.gait.cycling;
    now.supportPlanted = !input.gait.cycling || input.gait.planted[1 - s];
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
    // De onde o pe parte: do movimento ainda nesta perna (o corrente ou o
    // anterior voltando), com o estado dele; senao do pe da pose.
    const BallMove3D* from = m_move.active() && m_move.side() == choice.side ? &m_move
        : m_previous.active() && m_previous.side() == choice.side       ? &m_previous
                                                                         : nullptr;
    BallMoveStart3D begin;
    begin.choice = choice;
    begin.power = m_intent.power;
    if (from != nullptr) {
        begin.foot = from->foot();
        begin.footOrientation = from->orientation();
        begin.footVelocity = from->velocity();
        begin.footPlanted = from->phase() == BallMovePhase3D::Hold;
    } else {
        begin.foot = body.foot[s];
        begin.footOrientation = body.footOrientation[s];
        // (O pe apoiado parte parado; no ar, com a velocidade que ele tinha na
        // pose - a do estilo dava um tranco no primeiro tick.)
        begin.footVelocity = gait.planted[s] || !gait.cycling ? Vec3 {} : m_poseFootVelocity[s];
        begin.footPlanted = gait.planted[s] || !gait.cycling;
    }
    begin.pelvisGround = { body.pelvis.x, body.pelvis.y, body.groundHeight };
    begin.hip = body.hip[s];
    begin.facingYaw = body.facingYaw;
    begin.bodyVelocity = planar(input.bodyVelocity);
    begin.ankleInFoot = m_legs.ankleInFoot[s];
    begin.flatFoot = m_geometry.flat[s];
    begin.ankleHeight = m_ankleHeight[s];
    begin.heel = m_geometry.heel[s];
    begin.toe = m_geometry.toeBase[s];
    begin.legLength = m_legs.thighLength[s] + m_legs.shinLength[s];
    // O movimento corrente na outra perna segue a volta dele (vira o
    // anterior); o que estava nesta perna da lugar ao novo.
    if (m_move.active() && m_move.side() != choice.side) {
        m_previous = m_move;
    } else if (m_previous.active() && m_previous.side() == choice.side) {
        m_previous.stop();
    }
    // (A mistura do fim de um movimento nesta perna acaba: o novo parte da
    // pose dela.)
    if (m_settleLeft > 0.0f && m_lastSide == choice.side) m_settleLeft = 0.0f;
    m_move.start(begin, settings.move);
    m_kneeStart = body.knee[s];
    m_bodySide = choice.side;
    m_support = 1 - choice.side;
    m_interrupting = false;
    // O jeito do corpo neste toque: pela intensidade (o toque fraco mexe
    // pouco o corpo) e cada parte variando um pouco - nunca dois iguais.
    const float power = std::clamp(m_intent.power, 0.0f, 1.0f);
    const float floor = std::clamp(settings.bodyPowerFloor, 0.0f, 1.0f);
    m_style.amount = floor + (1.0f - floor) * power;
    const auto vary = [&](float amount) { return 1.0f + amount * (2.0f * random() - 1.0f); };
    const float v = settings.bodyVariation;
    m_style.shift = vary(v);
    m_style.drop = vary(v);
    m_style.yaw = vary(v);
    m_style.side = vary(v);
    m_style.bend = vary(v);
    m_style.arms = vary(v);
    m_style.look = 1.0f - 0.25f * random();
    m_style.pace = vary(0.5f * v);
    for (Vec3& wrist : m_style.wrist)
        wrist = Vec3 { 2.0f * random() - 1.0f, 2.0f * random() - 1.0f, 2.0f * random() - 1.0f } * settings.wristVariation;
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

void BallPlay3D::settle(const BallMove3D& move, const BallPlaySettings3D& settings) {
    m_lastSide = move.side();
    m_lastAnkle = move.ankle();
    m_lastFoot = move.orientation();
    m_lastKnee = &move == &m_move ? m_lastKnee : m_previousKnee;
    m_lastJoined = move.joined();
    m_settleLeft = m_lastJoined ? settings.joinedHandOverSeconds : settings.handOverSeconds;
    m_settleSpan = m_settleLeft;
}

void BallPlay3D::update(const BallHandlingInput3D& input, const BallOutcomeSettings3D& outcome,
    const BallPlaySettings3D& settings, BallHandlingOutput3D& output) {
    output = {};
    if (!m_ready) return;
    const float dt = std::max(input.deltaTime, 1e-4f);
    // Os pes do clipe e os da pose andam: a velocidade deles (a volta anda com
    // a do clipe; o movimento parte do pe no ar com a da pose), filtrada.
    {
        const float blend = std::min(1.0f, dt / settings.styleVelocitySeconds);
        for (std::size_t s = 0; s < 2; ++s) {
            const Vec3 clip = input.gait.clipFoot[s] + input.gait.clipFootOrientation[s].rotate(m_legs.ankleInFoot[s]);
            // (A do pe do clipe sem filtro: a volta soma o caminho dele a
            // diferenca que some, e com ela atrasada o pe dava um tranco.)
            const Vec3 clipVelocity = m_feetKnown ? (clip - m_clipAnkle[s]) * (1.0f / dt) : Vec3 {};
            const Vec3 accelerationRaw = m_feetKnown ? (clipVelocity - m_clipAnkleVelocity[s]) * (1.0f / dt) : Vec3 {};
            m_clipAnkleAcceleration[s] = m_feetKnown
                ? m_clipAnkleAcceleration[s] + (accelerationRaw - m_clipAnkleAcceleration[s]) * blend : Vec3 {};
            m_clipAnkleVelocity[s] = clipVelocity;
            m_clipAnkle[s] = clip;
            const Vec3 poseRaw = m_feetKnown ? (input.body.foot[s] - m_poseFootPrevious[s]) * (1.0f / dt) : Vec3 {};
            m_poseFootVelocity[s] = m_feetKnown
                ? m_poseFootVelocity[s] + (poseRaw - m_poseFootVelocity[s]) * blend : poseRaw;
            m_poseFootPrevious[s] = input.body.foot[s];
        }
    }
    m_feetKnown = true;
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
    // Ocupado (o gesto anterior antes do fim do acompanhamento, o intervalo),
    // o tempo guardado para.
    if (BallActionRecord3D* record = m_queue.active(); record != nullptr && record->stage == BallActionStage3D::Seeking) {
        const bool busy = m_cooldown > 0.0f || m_move.gesturing();
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

    // 3) Os movimentos. O anterior segue a volta dele na outra perna. Depois
    // da forca, o movimento pedido mudando (arrancar, parar, virar,
    // inverter): o corrente vai direto a volta.
    if (m_previous.active()) {
        m_previous.update(dt, live(input, m_previous.side()), settings.move);
        if (m_previous.landed() || m_previous.joined()) settle(m_previous, settings);
    }
    if (m_move.active()) {
        const int side = m_move.side();
        if (!m_interrupting) {
            const Vec3 now = planar(input.aheadVelocity);
            const float a = now.length(), b = m_requestedAtAccept.length();
            const bool turned = a > 1.0f && b > 1.0f && std::abs(signedAngle(m_requestedAtAccept, now)) > settings.interruptTurn;
            if (turned || (now - m_requestedAtAccept).length() > settings.interruptSpeed) {
                m_interrupting = true;
                m_move.hurry(dt, live(input, side), settings.move);
            }
        }
        m_move.update(dt, live(input, side), settings.move);
        if (m_move.landed() || m_move.joined()) settle(m_move, settings);
    }

    // 4) O corpo desacelera um pouco durante o gesto (ate o pouso).
    {
        const bool gesture = m_move.active() && (m_move.gesturing() || m_move.phase() == BallMovePhase3D::Land);
        const bool shot = m_move.choice().touch == BallMoveTouch3D::Shot;
        const float wanted = gesture ? (shot ? settings.shotSpeedScale : settings.touchSpeedScale) : 1.0f;
        m_speedScale += (wanted - m_speedScale) * std::min(1.0f, dt / std::max(0.01f, settings.speedScaleSeconds));
        output.speedScale = m_speedScale;
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
    // O corpo do gesto (pelve, tronco, bracos, olhar) so ate o pouso: na
    // espera e na volta ele ja sai.
    const bool bodyActing = acting && (m_move.gesturing() || m_move.phase() == BallMovePhase3D::Land);
    // A perna saindo do fim de um movimento, aos poucos.
    if (m_settleLeft > 0.0f) m_settleLeft = std::max(0.0f, m_settleLeft - dt);
    const bool settling = m_settleLeft > 0.0f;
    const bool releasing = m_previous.active() || (acting && settling && m_lastSide != m_move.side());
    const float wantedWeight = bodyActing ? 1.0f : 0.0f;
    const int side = m_bodySide;
    const int support = 1 - side;
    if (!m_bodyLive && !acting && !settling && !releasing) {
        m_appliedShift = {};
        m_appliedDrop = 0.0f;
        return;
    }

    // Os alvos: o centro de massa sobre o apoio (parado), a pelve abaixa um
    // pouco e abre para o rumo do toque, o tronco inclina para o lado
    // contrario da perna, os bracos equilibram pelo balanco do pe e o olhar
    // vai na bola - tudo pelo jeito deste toque (a intensidade e a variacao;
    // no corpo fisico, menos).
    const float amount = m_style.amount * (input.animatedBody ? 1.0f : settings.physicalBodyShare);
    const std::size_t sup = static_cast<std::size_t>(support);
    const Vec3 base = input.centerOfMass - m_appliedShift * settings.pelvisMassShare;
    const Vec3 shift = clampLength(planar(body.foot[sup] - base) * (1.0f / std::max(0.3f, settings.pelvisMassShare)),
                           settings.maximumPelvisShift)
        * ((0.6f + 0.4f * amount) * m_style.shift);
    const bool shot = m_move.choice().touch == BallMoveTouch3D::Shot;
    const float drop = (shot ? 0.06f : 0.045f) * amount * m_style.drop;
    const float sideSign = side == 0 ? 1.0f : -1.0f;
    const Vec3 push = m_move.push();
    const float yaw = std::clamp(signedAngle(forward, push) * 0.2f, -0.25f, 0.25f) * amount * m_style.yaw;
    const float bend = (shot && bodyActing ? 0.08f : 0.04f) * amount * m_style.bend;
    const float trunkSide = -sideSign * 0.10f * amount * m_style.side;
    const float swing = std::clamp(0.15f + (bodyActing ? m_move.velocity().length() : 0.0f) / 16.0f, 0.0f, 0.85f)
        * amount * m_style.arms;
    const float open = m_armSwing.value();
    const Vec3 outwardSupport = left * (support == 0 ? 1.0f : -1.0f);
    const Vec3 outwardKick = left * sideSign;
    std::array<Vec3, 2> wrist {};
    wrist[sup] = forward * (0.12f + 0.20f * open) + outwardSupport * (0.22f + 0.25f * open) - up * (0.42f - 0.30f * open)
        + m_style.wrist[sup] * amount;
    wrist[static_cast<std::size_t>(side)] = forward * -(0.06f + 0.22f * open) + outwardKick * (0.18f + 0.12f * open)
        - up * (0.44f - 0.18f * open) + m_style.wrist[static_cast<std::size_t>(side)] * amount;
    const float lookWeight = input.ball.valid ? 0.85f * (0.6f + 0.4f * amount) * m_style.look : 0.0f;
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
        m_trunkSide.reset(trunkSide);
        m_trunkBend.reset(bend);
        m_armSwing.reset(swing);
        m_wrist[0].reset(wrist[0]);
        m_wrist[1].reset(wrist[1]);
        m_look.reset(look);
        m_lookWeight.reset(lookWeight);
    }
    const float pace = std::max(0.5f, m_style.pace);
    m_weight.step(wantedWeight, settings.prepareSeconds * pace, dt);
    m_shift.step(shift, settings.targetSeconds * pace, dt);
    m_drop.step(drop, settings.targetSeconds * pace, dt);
    m_yaw.step(yaw, settings.targetSeconds * pace, dt);
    m_trunkSide.step(trunkSide, settings.targetSeconds * pace, dt);
    m_trunkBend.step(bend, settings.targetSeconds * pace, dt);
    m_armSwing.step(swing, settings.armSeconds * pace, dt);
    m_wrist[0].step(wrist[0], settings.targetSeconds * pace, dt);
    m_wrist[1].step(wrist[1], settings.targetSeconds * pace, dt);
    m_look.step(look, settings.lookSeconds * pace, dt);
    m_lookWeight.step(lookWeight, settings.lookSeconds * pace, dt);
    const float weight = std::clamp(m_weight.value(), 0.0f, 1.0f);
    if (weight <= 0.001f && !acting && !settling && !releasing) {
        m_bodyLive = false;
        m_appliedShift = {};
        m_appliedDrop = 0.0f;
        return;
    }

    BallBodyTargets3D& t = output.body;
    t.active = true;
    t.side = side;
    t.weight = weight;
    // A perna que termina um movimento (a mistura dele para a locomocao):
    // junto do clipe, ela vai com o pe e o joelho do clipe.
    const auto settled = [&](Vec3& ankle, Quaternion& foot, Vec3& knee) {
        const std::size_t s = static_cast<std::size_t>(m_lastSide);
        if (m_lastJoined) {
            ankle = m_clipAnkle[s] + m_clipAnkleVelocity[s] * dt;
            foot = input.gait.clipFootOrientation[s];
            knee = input.gait.clipKnee[s].lengthSquared() > 1e-6f ? input.gait.clipKnee[s].normalized() : m_lastKnee;
        } else {
            ankle = m_lastAnkle;
            foot = m_lastFoot;
            knee = m_lastKnee;
        }
    };
    if (acting) {
        // A perna inteira no movimento (sem mistura: ele parte do pe da pose,
        // com a velocidade dele).
        const std::size_t s = static_cast<std::size_t>(side);
        t.ankle = m_move.ankle();
        t.foot = m_move.orientation();
        t.knee = moveKnee(m_move, body.hip[s], forward, input.gait.clipKnee[s]);
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
        t.side = m_lastSide;
        settled(t.ankle, t.foot, t.knee);
        t.legWeight = smooth(m_settleLeft / std::max(0.02f, m_settleSpan));
        output.actingSide = m_lastSide;
    }
    // A outra perna: o movimento anterior voltando, ou terminando a mistura.
    if (m_previous.active()) {
        const std::size_t r = static_cast<std::size_t>(m_previous.side());
        t.releaseSide = m_previous.side();
        t.releaseAnkle = m_previous.ankle();
        t.releaseFoot = m_previous.orientation();
        t.releaseKnee = moveKnee(m_previous, body.hip[r], forward, input.gait.clipKnee[r]);
        t.releaseWeight = 1.0f;
        m_previousKnee = t.releaseKnee;
    } else if (acting && settling && m_lastSide != m_move.side()) {
        t.releaseSide = m_lastSide;
        settled(t.releaseAnkle, t.releaseFoot, t.releaseKnee);
        t.releaseWeight = smooth(m_settleLeft / std::max(0.02f, m_settleSpan));
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
    // balanco da corrida - as maos davam o tranco. E pela intensidade: no
    // toque fraco o braco quase nao sai do da locomocao.)
    const float arms = share * (1.0f - 0.5f * std::clamp(input.gait.movingShare, 0.0f, 1.0f)) * amount;
    t.armWeight = { arms, arms };
    t.look = m_look.value();
    // (Correndo, o olhar pesa menos: a bola passando de um lado ao outro
    // virava a cabeca de supetao.)
    t.lookWeight = std::clamp(m_lookWeight.value(), 0.0f, 1.0f)
        * (1.0f - 0.4f * std::clamp(input.gait.movingShare, 0.0f, 1.0f));
    // (As pernas nao se cruzam pelo desenho do movimento - cada pe do seu
    // lado.)
    t.separateWeight = 0.0f;
    m_appliedShift = planar(t.pelvisShift) * weight;
    m_appliedDrop = t.pelvisDrop * weight;
}

} // namespace MatterEngine
