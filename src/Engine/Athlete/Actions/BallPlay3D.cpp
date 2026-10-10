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

// Quando o pe do estilo pousa (s a partir de agora): no ar, o que falta; no
// apoio, a saida mais o balanco.
float styleTouchdown(const BallHandlingGait3D& gait, std::size_t side) {
    const float swing = std::max(0.1f, gait.swingSeconds[side]);
    return gait.planted[side] ? std::max(0.0f, gait.secondsToLiftoff[side]) + swing
                              : std::max(0.0f, gait.secondsToTouchdown[side]);
}

// Para onde aponta o joelho: perpendicular a perna, no plano dela com o
// vertical, do lado da frente, girado no eixo da perna o quanto a ponta do pe
// gira em relacao ao corpo (o mesmo do manejo da Etapa 5).
Vec3 kneeDirection(Vec3 leg, Vec3 forward, Vec3 toe) {
    const float length = leg.length();
    if (length < 1e-4f) return forward;
    const Vec3 along = leg * (1.0f / length);
    const Vec3 knee = cross(along, cross(Vec3 { 0.0f, 0.0f, 1.0f }, forward));
    if (knee.lengthSquared() < 1e-6f) return forward;
    const float yaw = toe.lengthSquared() > 1e-6f ? std::clamp(signedAngle(forward, toe), -1.6f, 1.6f) : 0.0f;
    return Quaternion::fromAxisAngle(along * -1.0f, yaw).rotate(knee.normalized());
}

// A pelve daqui a `at` s: a velocidade de agora indo a pedida em ~0,3 s.
Vec3 pelvisAhead(Vec3 pelvis, Vec3 now, Vec3 asked, float at) {
    constexpr float Ramp = 0.3f;
    const Vec3 change = planar(asked) - planar(now);
    return pelvis + planar(now) * at + change * (at < Ramp ? at * at / (2.0f * Ramp) : at - 0.5f * Ramp);
}

} // namespace

BallPlaySettings3D::BallPlaySettings3D() {
    // O gesto rapido (do pedido ao fim do toque ~0,25 s): acompanhamento e
    // volta mais curtos que os da Etapa 5.
    gesture.touchFollowSeconds = 0.07f;
    gesture.shotFollowSeconds = 0.11f;
    gesture.returnSeconds = 0.11f;
    gesture.returnSecondsPower = 0.05f;
    gesture.swingStrikeShare = 0.1f;
}

bool BallPlay3D::inArea(Vec3 ball, float radius, Vec3 pelvis, const BallPlaySettings3D& settings) {
    if (ball.z - radius > pelvis.z + settings.waistAbovePelvis) return false;
    return std::hypot(ball.x - pelvis.x, ball.y - pelvis.y) <= settings.areaRadius;
}

void BallPlay3D::reset(const RagdollProfile3D& profile) {
    m_legs = resolveAthleteLegs3D(profile);
    m_box = resolveBallFootBox3D(profile, m_legs);
    m_ready = m_legs.valid && m_box.valid;
    m_queue.clear();
    m_gesture.stop();
    m_action = false;
    m_forced = false;
    m_waitingFoot = false;
    m_virtualBall = false;
    m_imagined = false;
    m_sinceInArea = 10.0f;
    m_cooldown = 0.0f;
    m_busyWait = 0.0f;
    m_busyRecord = 0;
    m_interrupting = false;
    m_exit = 1.0f;
    m_previousSide = -1;
    m_bodyLive = false;
    m_appliedShift = {};
    m_appliedDrop = 0.0f;
    m_styleFootKnown = false;
}

std::uint64_t BallPlay3D::targetBallKey() const {
    if (const BallActionRecord3D* record = m_queue.active()) return record->ballKey;
    if (m_action) return m_ballKey;
    // (Enquanto o corpo sai do gesto, o olhar ainda segue a bola dele.)
    return m_gesture.active() || m_bodyLive ? m_gestureBall : 0;
}

float BallPlay3D::areaExit(const BallHandlingInput3D& input, float horizon, const BallPlaySettings3D& settings) const {
    if (!input.ball.valid) return 0.0f;
    for (float at = 0.0f; at <= horizon + 1e-4f; at += 0.01f) {
        const BallPrediction3D ball = predictBall3D(input.ball.position, input.ball.velocity, input.body.groundHeight,
            input.ball.radius, at, settings.predictor);
        const Vec3 pelvis = pelvisAhead(input.body.pelvis, input.bodyVelocity, input.aheadVelocity, at);
        if (!inArea(ball.position, input.ball.radius, pelvis, settings)) return at;
    }
    return -1.0f;
}

BallPlayGesturePlan3D BallPlay3D::planGesture(const BallHandlingInput3D& input, Vec3 ballPosition,
    Vec3 ballVelocity, const BallPlaySettings3D& settings, int forcedSide, bool startNow) const {
    const BallPlannerBody3D& body = input.body;
    const BallHandlingGait3D& gait = input.gait;
    const Vec3 forward { std::cos(body.facingYaw), std::sin(body.facingYaw), 0.0f };
    const Vec3 left { -forward.y, forward.x, 0.0f };
    const bool shot = m_intent.kind == BallActionKind3D::Shot;
    const float power = std::clamp(m_intent.power, 0.0f, 1.0f);
    Vec3 push = shot ? Vec3 { std::cos(m_intent.aimYawRadians), std::sin(m_intent.aimYawRadians), 0.0f }
                     : planar(m_intent.direction);
    push = push.lengthSquared() > 1e-6f ? push.normalized() : forward;
    const float speed = planar(body.velocity).length();
    const bool moving = gait.cycling && speed > 0.6f;
    const float radius = input.ball.radius;

    // O desempate (o da V3): o pe do lado da bola; perto do meio, o que
    // empurra com o lado de dentro (ou o direito, para a frente).
    int preferred = 1;
    {
        const float lateral = dot(planar(ballPosition - body.pelvis), left);
        const float across = dot(push, left);
        preferred = std::abs(lateral) >= 0.07f ? (lateral >= 0.0f ? 0 : 1) : std::abs(across) > 0.3f ? (across > 0.0f ? 1 : 0) : 1;
    }

    struct Candidate {
        bool available = false;
        bool real = false;
        float cost = 1e9f;
        float start = 0.0f;
        BallGestureFrame3D frame = BallGestureFrame3D::World;
        BallPlan3D plan;
    };
    Candidate best;
    for (int side = 0; side < 2; ++side) {
        if (forcedSide >= 0 && side != forcedSide) continue;
        const std::size_t s = static_cast<std::size_t>(side);
        Candidate c;
        c.available = true;
        c.frame = !moving ? BallGestureFrame3D::World : shot ? BallGestureFrame3D::Body : BallGestureFrame3D::StyleFoot;
        if (moving && !startNow) {
            // Andando, o gesto vai no balanco: o pe no ar com tempo ate pousar
            // faz ja; o apoiado que sai logo espera sair.
            if (!gait.planted[s]) {
                c.available = gait.secondsToTouchdown[s] >= settings.minimumSwingLeft;
            } else {
                c.available = gait.secondsToLiftoff[s] >= 0.0f && gait.secondsToLiftoff[s] <= settings.liftWait;
                c.start = std::max(0.0f, gait.secondsToLiftoff[s]);
            }
        }
        if (!c.available) continue;
        const bool swing = c.frame == BallGestureFrame3D::StyleFoot;
        const float strike = swing ? settings.swingStrike
            : shot ? settings.shotStrike + settings.shotStrikePower * power
                   : settings.touchStrike + settings.touchStrikePower * power;
        float windup = swing ? 0.0f
            : shot ? settings.shotWindup + settings.shotWindupPower * power
                   : settings.touchWindup + settings.touchWindupPower * power;
        const float legLength = m_legs.thighLength[s] + m_legs.shinLength[s];
        // (Duas passadas: o armado cresce com o caminho do pe ate o contato.)
        Vec3 ball, footPosition, pelvisThen, hipThen;
        Quaternion footOrientation;
        BallTouchSurface3D surface = BallTouchSurface3D::Inside;
        for (int pass = 0; pass < 2; ++pass) {
            const float at = c.start + windup + strike;
            ball = predictBall3D(ballPosition, ballVelocity, body.groundHeight, radius, at, settings.predictor).position;
            pelvisThen = moving ? pelvisAhead(body.pelvis, body.velocity, input.aheadVelocity, at) : body.pelvis;
            hipThen = body.hip[s] + (pelvisThen - body.pelvis);
            const bool behind = dot(planar(ball - pelvisThen), forward) < -0.06f;
            surface = chooseBallSurface3D(side, push, forward, m_intent.kind, moving, behind, m_intent.curve);
            ballContactFootPose3D(m_box, side, surface, ball, radius, push, forward, power, m_intent.lift,
                body.groundHeight, footPosition, footOrientation);
            if (swing) break;
            // (O toque arma logo atras do contato: o pe anda ate la no armado.
            // O chute arma onde o pe esta - so sobe -, e a batida leva o
            // caminho todo.)
            const Vec3 windupPoint = shot ? body.foot[s]
                : footPosition - push * (settings.gesture.touchBack + settings.gesture.touchBackPower * power);
            const float travel = (windupPoint - body.foot[s]).length();
            windup = std::max(shot ? settings.shotWindup + settings.shotWindupPower * power
                                   : settings.touchWindup + settings.touchWindupPower * power,
                travel / std::max(0.5f, settings.windupTravelSpeed));
        }
        // O contato de verdade cabe: do lado da perna, ao alcance, nao alto
        // demais e, no chute, a bola nao atras dele.
        const Vec3 ankle = footPosition + footOrientation.rotate(m_legs.ankleInFoot[s]);
        const float reach = (ankle - hipThen).length();
        const float lateral = dot(planar(ball - pelvisThen), left);
        const bool ownSide = side == 0 ? lateral >= -settings.sideSlack : lateral <= settings.sideSlack;
        const bool ahead = !shot || dot(planar(ball - pelvisThen), forward) > -0.02f;
        const bool low = ball.z - radius - body.groundHeight <= settings.maximumHeight;
        c.real = ownSide && ahead && low && reach <= settings.reachLimit * legLength && reach >= settings.minimumReach;
        c.cost = c.start + (c.real ? 0.0f : 0.25f) + (side == preferred ? 0.0f : 0.04f);
        c.plan.valid = true;
        c.plan.side = side;
        c.plan.surface = surface;
        c.plan.contactTime = windup + strike;
        c.plan.ball = ball;
        c.plan.push = push;
        c.plan.footPosition = footPosition;
        c.plan.footOrientation = footOrientation;
        c.plan.windupSeconds = windup;
        c.plan.strikeSeconds = strike;
        c.plan.reach = reach / std::max(0.1f, legLength);
        c.plan.cost = c.cost;
        if (c.cost < best.cost) best = c;
    }
    if (!best.available) {
        // (Nenhum pe livre a tempo - os dois no chao longe de sair, ou no fim
        // do balanco: o gesto ja, com o corpo andando, no pe do lado da bola.)
        BallPlayGesturePlan3D fallback = planGesture(input, ballPosition, ballVelocity, settings,
            forcedSide >= 0 ? forcedSide : preferred, true);
        fallback.frame = moving ? BallGestureFrame3D::Body : BallGestureFrame3D::World;
        return fallback;
    }

    BallPlayGesturePlan3D result;
    result.moving = moving;
    result.startIn = best.start;
    result.frame = best.frame;
    result.plan = best.plan;
    if (!best.real) {
        // O gesto nao conseguiria comecar na bola de verdade: o gesto numa
        // bola imaginada (so o visual - a forca e na de verdade). Longe ou
        // alta, ela vem para o quadril no rumo da de verdade ate a perna
        // alcancar (a perna vai ate onde da, na direcao dela); do outro lado,
        // fica do lado da perna; atras no chute, ou sem jeito, logo a frente.
        const int side = best.plan.side;
        const std::size_t s = static_cast<std::size_t>(side);
        const float out = side == 0 ? 1.0f : -1.0f;
        const float at = best.start + best.plan.contactTime;
        const Vec3 pelvisThen = moving ? pelvisAhead(body.pelvis, body.velocity, input.aheadVelocity, at) : body.pelvis;
        const Vec3 hipThen = body.hip[s] + (pelvisThen - body.pelvis);
        const float legLength = std::max(0.1f, m_legs.thighLength[s] + m_legs.shinLength[s]);
        const BallTouchSurface3D surface = chooseBallSurface3D(side, push, forward, m_intent.kind, moving, false,
            m_intent.curve);
        Vec3 footPosition;
        Quaternion footOrientation;
        const auto pose = [&](Vec3 ball) {
            ballContactFootPose3D(m_box, side, surface, ball, radius, push, forward, power, m_intent.lift,
                body.groundHeight, footPosition, footOrientation);
            return (footPosition + footOrientation.rotate(m_legs.ankleInFoot[s]) - hipThen).length();
        };
        // A de verdade no chao e do lado da perna.
        Vec3 real = best.plan.ball;
        real.z = body.groundHeight + radius;
        {
            const float lateral = dot(planar(real - pelvisThen), left) * out;
            if (lateral < settings.imaginedSide) real += left * ((settings.imaginedSide - lateral) * out);
        }
        const bool behind = dot(planar(real - pelvisThen), forward) < (m_intent.kind == BallActionKind3D::Shot ? 0.05f : -0.45f);
        Vec3 imagined;
        if (behind) {
            const Vec3 velocity = planar(input.aheadVelocity).length() > planar(body.velocity).length()
                ? planar(input.aheadVelocity) : planar(body.velocity);
            const Vec3 travel = velocity.length() > 1.0f ? velocity.normalized() : forward;
            imagined = Vec3 { pelvisThen.x, pelvisThen.y, body.groundHeight + radius } + forward * settings.imaginedAhead
                + left * (settings.imaginedSide * out) + travel * settings.imaginedLead;
        } else {
            // (O quanto ela vem do quadril ate a perna alcancar: busca binaria
            // no plano, da de verdade ao quadril.)
            const Vec3 hipGround { hipThen.x, hipThen.y, body.groundHeight + radius };
            float lo = 0.0f, hi = 1.0f;
            for (int k = 0; k < 12; ++k) {
                const float mid = 0.5f * (lo + hi);
                if (pose(hipGround + (real - hipGround) * mid) <= settings.reachLimit * legLength * 0.98f) lo = mid;
                else hi = mid;
            }
            imagined = hipGround + (real - hipGround) * lo;
            // (Nunca colada no outro pe: pelo menos o lado minimo.)
            const float lateral = dot(planar(imagined - pelvisThen), left) * out;
            if (lateral < settings.imaginedSide) imagined += left * ((settings.imaginedSide - lateral) * out);
        }
        const float reach = pose(imagined);
        result.plan.surface = surface;
        result.plan.ball = imagined;
        result.plan.footPosition = footPosition;
        result.plan.footOrientation = footOrientation;
        result.plan.reach = reach / legLength;
        result.imagined = true;
    }
    return result;
}

void BallPlay3D::startGesture(const BallHandlingInput3D& input, const BallPlayGesturePlan3D& plan,
    const BallPlaySettings3D& settings) {
    const BallPlannerBody3D& body = input.body;
    const int side = plan.plan.side;
    const std::size_t s = static_cast<std::size_t>(side);
    BallGestureStart3D begin;
    begin.plan = plan.plan;
    begin.kind = m_intent.kind;
    begin.power = m_intent.power;
    begin.lift = m_intent.lift;
    begin.bodyVelocity = input.bodyVelocity;
    // (O gesto anterior ainda no ar na mesma perna: o novo parte dele.)
    if (m_gesture.active() && m_gesture.side() == side) {
        begin.foot = m_gesture.position();
        begin.footVelocity = m_gesture.velocity();
        begin.footOrientation = m_gesture.orientation();
    } else {
        begin.foot = body.foot[s];
        begin.footVelocity = body.footVelocity[s];
        begin.footOrientation = body.footOrientation[s];
    }
    begin.groundHeight = body.groundHeight;
    begin.hip = body.hip[s];
    begin.ankleInFoot = m_legs.ankleInFoot[s];
    begin.legLength = m_legs.thighLength[s] + m_legs.shinLength[s];
    begin.moving = plan.moving;
    begin.frame = plan.frame;
    begin.styleFoot = input.gait.styleFoot[s];
    begin.styleFootVelocity = m_styleFootVelocity[s];
    begin.styleTouchdown = styleTouchdown(input.gait, s);
    begin.windupAhead = m_intent.kind != BallActionKind3D::Shot;
    begin.passThrough = true;
    m_gesture.start(begin, m_box, settings.gesture);
    m_plan = plan.plan;
    m_gestureIntent = m_intent;
    m_gestureBall = m_ballKey;
    m_gestureMoving = plan.moving;
    m_gestureFrame = plan.frame;
    m_kneeStart = body.knee[s];
    m_previousSide = side;
    m_imagined = plan.imagined;
    m_interrupting = false;
    m_exit = 1.0f;
    m_bodySide = side;
    m_settleLeft = 0.0f;
    m_lastLeg = 0.0f;
}

void BallPlay3D::applyForce(const BallHandlingInput3D& input, const BallOutcomeSettings3D& outcome,
    BallHandlingOutput3D& output) {
    BallOutcomeRequest3D intent = m_intent;
    intent.surface = m_plan.surface;
    const BallOutcome3D result = computeBallOutcome3D(intent, input.ball.velocity, input.bodyVelocity,
        input.aheadVelocity, input.ball.radius, outcome);
    const float inertia = (2.0f / 3.0f) * input.ball.mass * input.ball.radius * input.ball.radius;
    output.impulse = true;
    output.impulseBallKey = m_ballKey;
    output.linearImpulse = (result.velocity - input.ball.velocity) * input.ball.mass;
    output.angularImpulse = (result.spin - input.ball.spin) * inertia;
    output.outcomeVelocity = result.velocity;
    output.touched = true;
    m_forced = true;
    if (m_waitingFoot) {
        // (O gesto ainda vai comecar: ele vai onde a bola estaria sem a
        // forca.)
        m_virtualBall = true;
        m_virtualPosition = input.ball.position;
        m_virtualVelocity = input.ball.velocity;
        m_virtualClock = 0.0f;
    }
}

void BallPlay3D::update(const BallHandlingInput3D& input, const BallOutcomeSettings3D& outcome,
    const BallPlaySettings3D& settings, BallHandlingOutput3D& output) {
    output = {};
    if (!m_ready) return;
    const float dt = std::max(input.deltaTime, 1e-4f);
    // Os pes do estilo andam: a velocidade deles (o gesto no balanco vai com
    // eles), filtrada (no corte o pe do estilo gira com a pelve).
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
    const bool inside = ballValid && inArea(input.ball.position, input.ball.radius, input.body.pelvis, settings);
    m_sinceInArea = inside ? 0.0f : m_sinceInArea + dt;
    const bool reachable = ballValid && (inside || m_sinceInArea <= settings.guaranteeGrace);

    // 1) O pedido novo (a bola mais perto, travada): substitui o guardado.
    if (input.request) m_queue.push(input.intent, input.requestBallKey);
    m_queue.advance(dt);
    if (const BallActionRecord3D* active = m_queue.active(); active == nullptr || active->id != m_busyRecord) {
        m_busyWait = 0.0f;
        m_busyRecord = active != nullptr ? active->id : 0;
    }

    // 2) O pedido guardado: com a bola na area, sai (a garantia); passado o
    // tempo guardado sem ela entrar, acaba com o motivo. Ocupado (a acao
    // anterior sem a forca, o gesto dela antes do contato, o intervalo), o
    // tempo guardado para.
    bool acceptedNow = false;
    if (BallActionRecord3D* record = m_queue.active(); record != nullptr && record->stage == BallActionStage3D::Seeking) {
        const bool busy = (m_action && !m_forced) || m_cooldown > 0.0f
            || (m_gesture.active() && m_gesture.time() < m_gesture.contactTime());
        if (busy) m_busyWait += dt;
        if (!ballValid || input.ball.key != record->ballKey) {
            m_queue.finish(BallActionOutcome3D::InvalidTarget, "a bola do pedido sumiu");
        } else if (!busy && reachable) {
            m_intent = record->intent;
            m_ballKey = record->ballKey;
            m_requestedAtAccept = planar(input.aheadVelocity);
            m_action = true;
            m_actionClock = 0.0f;
            m_forced = false;
            m_virtualBall = false;
            // (Um gesto ainda no ar: o novo vai na mesma perna, sem largar a
            // outra no meio do caminho.)
            const int forcedSide = m_gesture.active() ? m_gesture.side() : -1;
            const BallPlayGesturePlan3D plan = planGesture(input, input.ball.position, input.ball.velocity, settings,
                forcedSide, false);
            m_estimatedContact = plan.startIn + plan.plan.contactTime;
            m_forceAt = std::clamp(settings.contactTiming, 0.0f, 1.0f) * m_estimatedContact;
            // (A bola indo sair da area antes: a forca vem antes, com ela
            // ainda dentro.)
            const float exit = areaExit(input, m_estimatedContact, settings);
            if (exit >= 0.0f) m_forceAt = std::min(m_forceAt, std::max(0.0f, exit - 0.02f));
            m_imagined = plan.imagined;
            if (plan.startIn <= 1e-4f) {
                startGesture(input, plan, settings);
                m_waitingFoot = false;
            } else {
                m_waitingFoot = true;
                m_startIn = plan.startIn;
                m_waitSide = plan.plan.side;
                m_plan = plan.plan;
                m_bodySide = plan.plan.side;
            }
            record->stage = BallActionStage3D::Committed;
            record->latency = std::max(0.0f, record->age - dt);
            acceptedNow = true;
        } else if (record->age - m_busyWait > settings.requestBuffer) {
            m_queue.finish(BallActionOutcome3D::Expired, "a bola nao entrou na area no tempo guardado");
        }
    }

    // 3) A acao aceita, antes da forca: ela sai na fracao da regua, ou ja com
    // a bola saindo da area.
    if (m_action && !m_forced) {
        if (!acceptedNow) m_actionClock += dt;
        if (!ballValid || input.ball.key != m_ballKey) {
            m_action = false;
            if (BallActionRecord3D* record = m_queue.active(); record != nullptr && record->stage == BallActionStage3D::Committed)
                m_queue.finish(BallActionOutcome3D::InvalidTarget, "a bola da acao sumiu");
        } else {
            const bool leaving = !inside || areaExit(input, 2.0f * dt, settings) >= 0.0f;
            if (m_actionClock >= m_forceAt - 1e-5f || (leaving && reachable)) {
                applyForce(input, outcome, output);
                m_action = false;
                m_cooldown = settings.cooldown;
                if (BallActionRecord3D* record = m_queue.active();
                    record != nullptr && record->stage == BallActionStage3D::Committed)
                    m_queue.finish(BallActionOutcome3D::Touched, "a bola na area");
            } else if (!reachable) {
                // (A bola saiu entre um tick e outro, sem previsao: a forca
                // nela ainda perto.)
                applyForce(input, outcome, output);
                m_action = false;
                m_cooldown = settings.cooldown;
                if (BallActionRecord3D* record = m_queue.active();
                    record != nullptr && record->stage == BallActionStage3D::Committed)
                    m_queue.finish(BallActionOutcome3D::Touched, "a bola saindo da area");
            }
        }
    }

    // 4) O gesto esperando o pe sair do chao (andando).
    if (m_waitingFoot) {
        m_startIn -= dt;
        if (m_virtualBall) m_virtualClock += dt;
        const std::size_t s = static_cast<std::size_t>(std::max(0, m_waitSide));
        const bool lifted = !input.gait.planted[s] || !input.gait.cycling;
        if (lifted || m_startIn <= 0.0f) {
            Vec3 position = input.ball.position, velocity = input.ball.velocity;
            if (m_virtualBall) {
                position = m_virtualPosition + planar(m_virtualVelocity) * m_virtualClock;
                velocity = m_virtualVelocity;
            }
            const BallPlayGesturePlan3D plan = planGesture(input, position, velocity, settings, m_waitSide, true);
            startGesture(input, plan, settings);
            m_waitingFoot = false;
            m_virtualBall = false;
        }
    }

    // 5) O gesto: antes da forca o contato acompanha a bola de verdade
    // (pouco); a perna avanca; depois da forca, mudando o movimento, ela sai.
    if (m_gesture.active()) {
        const BallPlannerBody3D& body = input.body;
        const Vec3 forward { std::cos(body.facingYaw), std::sin(body.facingYaw), 0.0f };
        if (!m_imagined && !m_forced && ballValid && input.ball.key == m_gestureBall
            && (m_gesture.phase() == BallGesturePhase3D::Windup || m_gesture.phase() == BallGesturePhase3D::Strike)) {
            const float remaining = std::max(0.0f, m_gesture.contactTime() - m_gesture.time());
            const BallPrediction3D ball = predictBall3D(input.ball.position, input.ball.velocity, body.groundHeight,
                input.ball.radius, remaining, settings.predictor);
            Vec3 footPosition;
            Quaternion footOrientation;
            ballContactFootPose3D(m_box, m_gesture.side(), m_plan.surface, ball.position, input.ball.radius,
                m_gesture.push(), forward, m_gestureIntent.power, m_gestureIntent.lift, body.groundHeight, footPosition,
                footOrientation);
            m_gesture.retarget(footPosition, footOrientation, dt, settings.gesture);
        }
        const std::size_t s = static_cast<std::size_t>(m_gesture.side());
        BallGestureLive3D live;
        live.hip = body.hip[s];
        live.restHip = live.hip - m_appliedShift + Vec3 { 0.0f, 0.0f, m_appliedDrop };
        live.bodyVelocity = input.bodyVelocity;
        live.styleFoot = input.gait.styleFoot[s];
        live.styleFootVelocity = m_styleFootVelocity[s];
        live.styleFootOrientation = input.gait.styleFootOrientation[s];
        m_gesture.update(dt, live, settings.gesture);
    }
    // Depois da forca (e do contato visual), o movimento pedido mudando
    // (arrancar, parar, virar, inverter): a perna sai do gesto - ou da volta
    // dele - numa mistura curta.
    if (m_forced && !m_interrupting && ((m_gesture.active() && m_gesture.time() >= m_gesture.contactTime())
            || (!m_gesture.active() && m_settleLeft > 0.0f))) {
        // (Comparado com o movimento de quando o pedido foi aceito: a mudanca
        // pode vir antes da forca.)
        const Vec3 now = planar(input.aheadVelocity);
        const float a = now.length(), b = m_requestedAtAccept.length();
        const bool turned = a > 1.0f && b > 1.0f && std::abs(signedAngle(m_requestedAtAccept, now)) > settings.interruptTurn;
        if (turned || (now - m_requestedAtAccept).length() > settings.interruptSpeed) m_interrupting = true;
    }
    if (m_interrupting) {
        m_exit = std::max(0.0f, m_exit - dt / std::max(0.02f, settings.interruptSeconds));
        m_settleLeft = std::min(m_settleLeft, settings.handOverSeconds * m_exit);
        if (m_gesture.active() && m_exit <= 0.0f) {
            m_gesture.stop();
            m_lastLeg = 0.0f;
        }
    }

    bodyTargets(input, settings, output);
    output.interacting = m_queue.active() != nullptr || m_action || m_gesture.active();
    output.interactingBallKey = targetBallKey();
    output.plan = m_plan;
}

void BallPlay3D::bodyTargets(const BallHandlingInput3D& input, const BallPlaySettings3D& settings,
    BallHandlingOutput3D& output) {
    const float dt = std::max(input.deltaTime, 1e-4f);
    const BallPlannerBody3D& body = input.body;
    const Vec3 forward { std::cos(body.facingYaw), std::sin(body.facingYaw), 0.0f };
    const Vec3 left { -forward.y, forward.x, 0.0f };
    const Vec3 up { 0.0f, 0.0f, 1.0f };
    const bool acting = m_gesture.active();
    const bool preparing = m_action || m_waitingFoot;
    // A perna: inteira no gesto ate a volta, saindo com ela; a interrupcao
    // a tira mais cedo.
    // (Parado, a perna entra no gesto aos poucos - a do IK nao e identica a
    // do clipe; no balanco o gesto ja parte do pe do estilo, com a
    // velocidade dele.)
    const float blendIn = m_gestureFrame == BallGestureFrame3D::StyleFoot ? 1.0f
        : smooth(m_gesture.time() / std::max(0.01f, settings.legBlendSeconds));
    // (A saida pela interrupcao numa curva sem quina.)
    const float leg = acting ? (m_gesture.phase() == BallGesturePhase3D::Return ? m_gesture.weight() : 1.0f) * smooth(m_exit)
            * blendIn
                             : 0.0f;
    // (O gesto acabou com a perna ainda nele: ela volta a locomocao aos
    // poucos; o pe solto segue com a velocidade do fim do gesto - no ar com o
    // corpo, ou com o pe do estilo que pousa.)
    const bool settling = !acting && m_settleLeft > 0.0f && m_lastLeg > 0.001f;
    if (settling) {
        m_settleLeft = std::max(0.0f, m_settleLeft - dt);
        m_lastAnkle += m_lastVelocity * dt;
    }
    const float wantedWeight = acting ? leg : settling ? m_lastLeg * smooth(m_settleLeft / std::max(0.02f, settings.handOverSeconds))
        : preparing ? 1.0f : 0.0f;
    const int side = m_bodySide;
    const int support = 1 - side;
    if (!m_bodyLive && wantedWeight <= 0.0f) {
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
    const float drop = 0.02f + 0.05f * std::clamp((m_plan.reach - 0.80f) / 0.15f, 0.0f, 1.0f);
    const float sideSign = side == 0 ? 1.0f : -1.0f;
    const Vec3 push = acting ? m_gesture.push() : m_plan.push;
    const float yaw = std::clamp(signedAngle(forward, push) * 0.25f, -0.30f, 0.30f);
    const float bend = m_gestureIntent.kind == BallActionKind3D::Shot && acting ? 0.08f : 0.04f;
    const float swing = std::clamp(0.15f + (acting ? m_gesture.velocity().length() : 0.0f) / 16.0f, 0.0f, 0.85f);
    const float open = m_armSwing.value();
    const Vec3 outwardSupport = left * (support == 0 ? 1.0f : -1.0f);
    const Vec3 outwardKick = left * sideSign;
    std::array<Vec3, 2> wrist {};
    wrist[sup] = forward * (0.12f + 0.20f * open) + outwardSupport * (0.22f + 0.25f * open) - up * (0.42f - 0.30f * open);
    wrist[static_cast<std::size_t>(side)] = forward * -(0.06f + 0.22f * open) + outwardKick * (0.18f + 0.12f * open)
        - up * (0.44f - 0.18f * open);
    // O olhar: na bola, mas no maximo 60 graus para o lado da frente do
    // corpo e a pelo menos 0,5 m da pelve (a bola passando por tras ou por
    // baixo do corpo virava a cabeca de um lado para o outro).
    Vec3 look = m_look.value();
    if (input.ball.valid) {
        const Vec3 offset = planar(input.ball.position - body.pelvis);
        const float distance = std::max(0.5f, offset.length());
        // (Atras do corpo o olhar volta para a frente aos poucos - de 60 a
        // 120 graus: limitado so a 60, a bola cruzando bem atras trocava o
        // lado de uma vez.)
        const float raw = offset.lengthSquared() > 1e-6f ? signedAngle(forward, offset) : 0.0f;
        const float yaw = std::clamp(raw, -1.05f, 1.05f) * (1.0f - smooth((std::abs(raw) - 1.05f) / 1.05f));
        const Vec3 direction = forward * std::cos(yaw) + left * std::sin(yaw);
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
    if (weight <= 0.001f && wantedWeight <= 0.0f && !acting && !settling) {
        m_bodyLive = false;
        m_appliedShift = {};
        m_appliedDrop = 0.0f;
        return;
    }

    BallBodyTargets3D& t = output.body;
    t.active = true;
    t.side = side;
    t.weight = weight;
    if (acting && leg > 0.0f) {
        const std::size_t s = static_cast<std::size_t>(side);
        t.ankle = m_gesture.position() + m_gesture.orientation().rotate(m_legs.ankleInFoot[s]);
        t.foot = m_gesture.orientation();
        t.knee = kneeDirection(t.ankle - body.hip[s], forward, planar(m_gesture.orientation().rotate({ 1.0f, 0.0f, 0.0f })));
        {
            // (O polo sai do joelho da pose aos poucos.)
            const float u = smooth(m_gesture.time() / std::max(0.01f, settings.kneeBlendSeconds));
            const Vec3 blended = m_kneeStart + (t.knee - m_kneeStart) * u;
            if (blended.lengthSquared() > 1e-6f) t.knee = blended.normalized();
        }
        t.legWeight = leg;
        output.actingSide = side;
        m_lastAnkle = t.ankle;
        m_lastVelocity = m_gesture.velocity();
        m_lastFoot = t.foot;
        m_lastKnee = t.knee;
        m_lastLeg = leg;
        m_lastSide = side;
        m_settleLeft = settings.handOverSeconds;
    } else if (acting) {
        m_lastLeg = 0.0f;
    } else if (settling) {
        t.side = m_lastSide;
        t.ankle = m_lastAnkle;
        t.foot = m_lastFoot;
        t.knee = m_lastKnee;
        t.legWeight = m_lastLeg * smooth(m_settleLeft / std::max(0.02f, settings.handOverSeconds));
        output.actingSide = m_lastSide;
        if (m_settleLeft <= 0.0f) m_lastLeg = 0.0f;
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
    t.armWeight = { share, share };
    t.look = m_look.value();
    t.lookWeight = std::clamp(m_lookWeight.value(), 0.0f, 1.0f);
    // As pernas nunca se cruzam.
    t.separateGap = settings.sideGap;
    t.separateForward = forward;
    t.separateWeight = 1.0f;
    m_appliedShift = planar(t.pelvisShift) * weight;
    m_appliedDrop = t.pelvisDrop * weight;
}

} // namespace MatterEngine
