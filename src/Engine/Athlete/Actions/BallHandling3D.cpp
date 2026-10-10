#include "Engine/Athlete/Actions/BallHandling3D.hpp"

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

// Quando o pe do estilo pousa (s a partir de agora): no ar, o que falta; no
// apoio, a saida mais o balanco.
float styleTouchdown(const BallHandlingGait3D& gait, std::size_t side) {
    const float swing = std::max(0.1f, gait.swingSeconds[side]);
    return gait.planted[side] ? std::max(0.0f, gait.secondsToLiftoff[side]) + swing
                              : std::max(0.0f, gait.secondsToTouchdown[side]);
}

// O quanto o chute correndo freia o corpo (0..1 da velocidade), pela forca.
float shotSlowdown(const BallOutcomeRequest3D& intent, const BallHandlingSettings3D& settings) {
    return std::clamp(settings.runningShotSlowdown, 0.0f, 0.9f) * (0.6f + 0.4f * std::clamp(intent.power, 0.0f, 1.0f));
}

// Para onde aponta o joelho: perpendicular a perna (quadril -> tornozelo), no
// plano dela com o vertical, do lado da frente - a perna para baixo, para a
// frente; esticada a frente, para cima - e girado no eixo da perna o quanto a
// ponta do pe gira em relacao ao corpo. (Pela ponta do pe projetada, o polo
// virava de cima para baixo num tick quando o tornozelo passava a altura do
// quadril: a perna ficava paralela a ponta.)
Vec3 kneeDirection(Vec3 leg, Vec3 forward, Vec3 toe) {
    const float length = leg.length();
    if (length < 1e-4f) return forward;
    const Vec3 along = leg * (1.0f / length);
    const Vec3 knee = cross(along, cross(Vec3 { 0.0f, 0.0f, 1.0f }, forward));
    if (knee.lengthSquared() < 1e-6f) return forward;
    const float yaw = toe.lengthSquared() > 1e-6f ? std::clamp(signedAngle(forward, toe), -1.6f, 1.6f) : 0.0f;
    return Quaternion::fromAxisAngle(along * -1.0f, yaw).rotate(knee.normalized());
}

} // namespace

void BallHandling3D::reset(const RagdollProfile3D& profile) {
    m_legs = resolveAthleteLegs3D(profile);
    m_box = resolveBallFootBox3D(profile, m_legs);
    m_ready = m_legs.valid && m_box.valid;
    m_queue.clear();
    m_gesture.stop();
    m_plan = {};
    m_previousSide = -1;
    m_bodyLive = false;
    m_appliedShift = {};
    m_appliedDrop = 0.0f;
    m_settleLeft = 0.0f;
    m_slow.reset(0.0f);
    m_gestureMoving = false;
}

std::uint64_t BallHandling3D::targetBallKey() const {
    if (const BallActionRecord3D* record = m_queue.active()) return record->ballKey;
    // (Enquanto o corpo sai do gesto, o olhar ainda segue a bola dele.)
    return m_gesture.active() || m_bodyLive ? m_gestureBall : 0;
}

void BallHandling3D::update(const BallHandlingInput3D& input, const BallOutcomeSettings3D& outcome,
    const BallHandlingSettings3D& settings, BallHandlingOutput3D& output) {
    output = {};
    if (!m_ready) return;
    const float dt = std::max(input.deltaTime, 1e-4f);
    const BallPlannerBody3D& body = input.body;
    const Vec3 forward { std::cos(body.facingYaw), std::sin(body.facingYaw), 0.0f };
    const Vec3 left { -forward.y, forward.x, 0.0f };
    const Vec3 up { 0.0f, 0.0f, 1.0f };
    // Os pes do estilo andam: a velocidade deles (o toque no balanco vai com
    // eles).
    for (std::size_t s = 0; s < 2; ++s) {
        m_styleFootVelocity[s] = m_styleFootKnown
            ? (input.gait.styleFoot[s] - m_styleFootPrevious[s]) * (1.0f / dt) : Vec3 {};
        m_styleFootPrevious[s] = input.gait.styleFoot[s];
        // (O pouso do pe do estilo, em relacao a pelve: o ultimo de qualquer
        // pe, com o lado sem sinal, e a velocidade dele.)
        if (input.gait.cycling && input.gait.planted[s] && !m_stylePlantedPrevious[s]) {
            const Vec3 rel = input.gait.styleFoot[s] - body.pelvis;
            m_landLead = { dot(planar(rel), forward), std::abs(dot(planar(rel), left)),
                input.gait.styleFoot[s].z - body.groundHeight };
            m_landLeadSpeed = planar(body.velocity).length();
            m_landLeadKnown = true;
        }
        m_stylePlantedPrevious[s] = input.gait.planted[s];
    }
    m_styleFootKnown = true;

    // 1) O contato: o primeiro encosto do pe que joga na bola, na janela da
    // batida, vira o evento unico do pedido - o resultado aprovado, pela
    // velocidade da bola daquele instante, como impulso.
    if (m_gesture.active() && !m_gestureTouched && input.ball.valid && input.ball.key == m_gestureBall
        && input.ball.sensor.touched && m_gesture.inContactWindow(settings.gesture)
        && input.ball.sensor.link == input.footLinks[static_cast<std::size_t>(m_gesture.side())]) {
        BallOutcomeRequest3D intent = m_gestureIntent;
        intent.surface = m_plan.surface;
        const BallOutcome3D result = computeBallOutcome3D(intent, input.ball.velocity, input.bodyVelocity,
            input.aheadVelocity, input.ball.radius, outcome);
        const float inertia = (2.0f / 3.0f) * input.ball.mass * input.ball.radius * input.ball.radius;
        output.impulse = true;
        output.impulseBallKey = m_gestureBall;
        output.linearImpulse = (result.velocity - input.ball.velocity) * input.ball.mass;
        output.angularImpulse = (result.spin - input.ball.spin) * inertia;
        output.outcomeVelocity = result.velocity;
        output.touched = true;
        m_gestureTouched = true;
        if (!m_gestureResolved) {
            m_queue.finish(BallActionOutcome3D::Touched, "o pe encostou na bola");
            m_gestureResolved = true;
        }
    }

    // 2) O pedido novo (a bola mais perto, travada).
    if (input.request) m_queue.push(input.intent, input.requestBallKey);
    m_queue.advance(dt);

    // 3) O pedido em andamento, sem gesto: busca o plano; comeca o gesto
    // quando o armado precisa sair. Nada some: prazo, alvo e alcance dao o
    // desfecho com o motivo.
    BallActionRecord3D* record = m_queue.active();
    const bool gestureBusy = m_gesture.active() && m_gesture.phase() != BallGesturePhase3D::Return;
    if (record != nullptr && record->stage == BallActionStage3D::Seeking && !gestureBusy) {
        const bool shot = record->intent.kind == BallActionKind3D::Shot;
        const float horizon = shot ? settings.planner.shotHorizon : settings.planner.touchHorizon;
        if (!input.ball.valid || input.ball.key != record->ballKey) {
            m_queue.finish(BallActionOutcome3D::InvalidTarget, "a bola do pedido sumiu");
        } else {
            predictBallTable3D(input.ball.position, input.ball.velocity, body.groundHeight, input.ball.radius,
                settings.planner.interval, horizon, settings.predictor, m_table);
            BallPlannerBody3D planned = body;
            planned.moving = input.gait.cycling;
            if (input.gait.cycling) {
                // Andando: o contato cabe no balanco do pe que joga (esticado
                // ate swingStretch), com o outro apoiado ate ele - a passada
                // manda. O chute correndo freia o corpo.
                // (O apoio de um pe pela passada: o balanco menos os dois voos
                // - com um pe no chao e o outro no ar, o voo e do sair de um
                // ao pousar do outro.)
                float stance = 0.6f * std::max(0.1f, input.gait.swingSeconds[0]);
                for (std::size_t s = 0; s < 2; ++s) {
                    const std::size_t o = 1 - s;
                    if (input.gait.planted[s] && !input.gait.planted[o] && input.gait.secondsToLiftoff[s] >= 0.0f
                        && input.gait.secondsToTouchdown[o] >= 0.0f) {
                        const float flight = input.gait.secondsToTouchdown[o] - input.gait.secondsToLiftoff[s];
                        stance = std::clamp(std::max(0.1f, input.gait.swingSeconds[o]) - 2.0f * flight, 0.08f, 0.6f);
                    }
                }
                // (A espera do apoio pelo alcance: com o corpo freado.)
                const float braked = planar(body.velocity).length() * (shot ? 1.0f - shotSlowdown(record->intent, settings) : 1.0f);
                const float hold = std::min(settings.supportHoldSeconds, settings.supportHoldReach / std::max(0.5f, braked));
                for (std::size_t s = 0; s < 2; ++s) {
                    const float swing = std::max(0.1f, input.gait.swingSeconds[s]);
                    // (O toque vai no balanco do pe do estilo e volta a ele
                    // antes do pouso; o chute pode esticar o balanco.)
                    const float touchdown = styleTouchdown(input.gait, s);
                    if (input.gait.planted[s]) {
                        const float lift = std::max(0.0f, input.gait.secondsToLiftoff[s]);
                        planned.freeFrom[s] = lift;
                        planned.freeUntil[s] = shot ? lift + swing * settings.swingStretch
                                                    : touchdown - settings.gesture.swingMinimumFollow;
                        planned.plantedFrom[s] = 0.0f;
                        planned.plantedNatural[s] = lift;
                    } else {
                        const float land = std::max(0.0f, input.gait.secondsToTouchdown[s]);
                        planned.freeFrom[s] = 0.0f;
                        planned.freeUntil[s] = shot ? land + swing * (settings.swingStretch - 1.0f)
                                                    : touchdown - settings.gesture.swingMinimumFollow;
                        planned.plantedFrom[s] = land;
                        planned.plantedNatural[s] = land + stance;
                    }
                    planned.plantedUntil[s] = planned.plantedNatural[s] + hold + 0.03f;
                    // (O toque correndo nao precisa do apoio no chao: sai
                    // tambem na fase de voo, como no jogo.)
                    if (!shot) {
                        planned.plantedFrom[s] = 0.0f;
                        planned.plantedUntil[s] = 1e9f;
                        planned.plantedNatural[s] = 1e9f;
                    }
                }
                if (shot) planned.velocity = body.velocity * (1.0f - shotSlowdown(record->intent, settings));
                // O toque no balanco: onde o pe da passada vai passar (do
                // pe de agora ao pouso, no lugar do ultimo pouso dele).
                // (O avanco do pouso cresce com a velocidade.)
                const float speed = planar(body.velocity).length();
                const float leadScale = std::clamp(speed / std::max(0.5f, m_landLeadSpeed), 0.5f, 2.0f);
                for (std::size_t s = 0; s < 2 && !shot && m_landLeadKnown; ++s) {
                    const float land = styleTouchdown(input.gait, s);
                    const Vec3 pelvisThen = body.pelvis + planar(body.velocity) * land;
                    planned.swingFoot[s] = input.gait.styleFoot[s];
                    planned.swingVelocity[s] = m_styleFootVelocity[s];
                    planned.swingLift[s] = input.gait.planted[s] ? std::max(0.0f, input.gait.secondsToLiftoff[s]) : 0.0f;
                    planned.swingLand[s] = Vec3 { pelvisThen.x, pelvisThen.y, body.groundHeight }
                        + forward * (m_landLead.x * leadScale) + left * (m_landLead.y * (s == 0 ? 1.0f : -1.0f))
                        + up * m_landLead.z;
                    planned.swingLandTime[s] = land;
                }
            } else {
                // Parado: o gesto toma o pe que joga e reserva o de apoio (o
                // planejador de pegadas o deixa no chao).
                planned.freeFrom = { 0.0f, 0.0f };
                planned.freeUntil = { 1e9f, 1e9f };
                planned.plantedFrom = { 0.0f, 0.0f };
                planned.plantedUntil = { 1e9f, 1e9f };
                planned.plantedNatural = { 1e9f, 1e9f };
            }
            const BallPlan3D plan = planBallAction3D(record->intent, planned, m_table, settings.planner.interval,
                m_legs, m_box, input.ball.radius, m_previousSide, settings.planner);

            m_plan = plan;
            if (plan.valid) {
                m_prepareSide = plan.side;
                // (A tabela anda a 1/30 s: dentro de um intervalo, comeca ja e o
                // armado absorve a diferenca.)
                const float start = plan.contactTime - plan.windupSeconds - plan.strikeSeconds;
                if (start <= settings.planner.interval + 1e-4f) {
                    const std::size_t s = static_cast<std::size_t>(plan.side);
                    BallGestureStart3D begin;
                    begin.plan = plan;
                    begin.kind = record->intent.kind;
                    begin.power = record->intent.power;
                    begin.lift = record->intent.lift;
                    begin.bodyVelocity = input.bodyVelocity;
                    begin.foot = body.foot[s];
                    begin.footVelocity = body.footVelocity[s];
                    begin.footOrientation = body.footOrientation[s];
                    begin.groundHeight = body.groundHeight;
                    begin.hip = body.hip[s];
                    begin.ankleInFoot = m_legs.ankleInFoot[s];
                    begin.legLength = m_legs.thighLength[s] + m_legs.shinLength[s];
                    begin.frame = !input.gait.cycling ? BallGestureFrame3D::World
                        : shot ? BallGestureFrame3D::Body : BallGestureFrame3D::StyleFoot;
                    begin.styleFoot = input.gait.styleFoot[s];
                    begin.styleFootVelocity = m_styleFootVelocity[s];
                    begin.styleTouchdown = styleTouchdown(input.gait, s);
                    m_gesture.start(begin, m_box, settings.gesture);
                    m_gestureMoving = input.gait.cycling;
                    m_kneeStart = body.knee[s];
                    m_gestureIntent = record->intent;
                    m_gestureBall = record->ballKey;
                    m_gestureTouched = false;
                    m_gestureResolved = false;
                    m_previousSide = plan.side;
                    record->stage = BallActionStage3D::Committed;
                    record->latency = record->age;
                }
            } else if (record->age > settings.acquireSeconds + (shot && input.gait.cycling ? settings.runningShotAcquireExtra : 0.0f)) {
                m_queue.finish(BallActionOutcome3D::Unreachable, plan.reason);
            }
        }
        if (record = m_queue.active(); record != nullptr && record->stage == BallActionStage3D::Seeking
            && record->age > horizon + settings.acquireSeconds)
            m_queue.finish(BallActionOutcome3D::Expired, "passou o prazo sem gesto");
    }

    // 4) O gesto em andamento: o contato acompanha a bola (pequeno), a perna
    // avanca; sem encosto ate o fim da janela, "errou".
    if (m_gesture.active()) {
        if (input.ball.valid && input.ball.key == m_gestureBall && !m_gestureTouched
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
        // (O quadril de repouso: sem a descida e o deslocamento que o corpo
        // do gesto pos na pose.)
        const std::size_t s = static_cast<std::size_t>(m_gesture.side());
        BallGestureLive3D live;
        live.hip = body.hip[s];
        live.restHip = live.hip - m_appliedShift + up * m_appliedDrop;
        live.bodyVelocity = input.bodyVelocity;
        live.styleFoot = input.gait.styleFoot[s];
        live.styleFootVelocity = m_styleFootVelocity[s];
        live.styleFootOrientation = input.gait.styleFootOrientation[s];
        m_gesture.update(dt, live, settings.gesture);
        if (!m_gestureTouched && !m_gestureResolved && m_gesture.active()
            && !m_gesture.inContactWindow(settings.gesture) && m_gesture.phase() != BallGesturePhase3D::Windup) {
            m_queue.finish(BallActionOutcome3D::Missed, "a janela do contato passou sem encosto");
            m_gestureResolved = true;
        }
        if (!m_gesture.active() && !m_gestureResolved) {
            m_queue.finish(BallActionOutcome3D::Missed, "o gesto acabou sem encosto");
            m_gestureResolved = true;
        }
    }

    // O chute correndo freia o corpo, do pedido ao fim do acompanhamento.
    {
        const BallActionRecord3D* pending = m_queue.active();
        const bool seekingShot = pending != nullptr && pending->stage == BallActionStage3D::Seeking
            && pending->intent.kind == BallActionKind3D::Shot;
        const bool shooting = m_gesture.active() && m_gestureIntent.kind == BallActionKind3D::Shot
            && m_gesture.phase() != BallGesturePhase3D::Return;
        float slow = 0.0f;
        if (input.gait.cycling && seekingShot) slow = shotSlowdown(pending->intent, settings);
        else if (m_gestureMoving && shooting) slow = shotSlowdown(m_gestureIntent, settings);
        m_slow.step(slow, settings.slowdownSeconds, dt);
        output.speedScale = std::clamp(1.0f - m_slow.value(), 0.1f, 1.0f);
    }

    // 5) O corpo: prepara desde o pedido (olhar, peso no apoio) e segue o
    // gesto. O peso e os alvos vao por molas criticas: o alvo muda de uma vez
    // (plano novo, o lado que troca, o olhar depois do contato) e o corpo
    // nao salta.
    const BallActionRecord3D* now = m_queue.active();
    const bool preparing = now != nullptr && now->stage == BallActionStage3D::Seeking && m_plan.valid;
    const bool acting = m_gesture.active();
    // (Na volta o peso desce com o da perna e acaba com ela.)
    const float wantedWeight = acting
        ? (m_gesture.phase() == BallGesturePhase3D::Return ? m_gesture.weight() : 1.0f)
        : preparing ? 1.0f : 0.0f;
    // Depois do gesto: o pe ja pousou e as pegadas o travaram; a perna do
    // gesto sai aos poucos com ele preso (sem o estalo do joelho voltando de
    // uma vez ao do clipe).
    const bool settlingLeg = !acting && m_settleLeft > 0.0f;
    if (settlingLeg) {
        m_settleLeft = std::max(0.0f, m_settleLeft - dt);
        // (Andando, o pe solto no ar vai com o corpo enquanto a perna do
        // gesto sai: parado no mundo, ele ficava para tras.)
        if (m_gestureMoving) m_lastAnkle += planar(input.bodyVelocity) * dt;
    }
    const int side = acting ? m_gesture.side() : settlingLeg ? m_lastSide : m_prepareSide;
    const int support = 1 - side;
    output.interacting = now != nullptr || acting;
    output.interactingBallKey = targetBallKey();
    output.plan = m_plan;
    if (!m_bodyLive && wantedWeight <= 0.0f && !settlingLeg) {
        m_appliedShift = {};
        m_appliedDrop = 0.0f;
        return;
    }

    // Os alvos.
    // Equilibrio: o centro de massa sobre a sola do apoio. O da pose anterior
    // ja tem o deslocamento aplicado nela (a pelve leva pelvisMassShare da
    // massa); tirado ele, o que poe o centro sobre o apoio.
    const std::size_t sup = static_cast<std::size_t>(support);
    const Vec3 base = input.centerOfMass - m_appliedShift * settings.pelvisMassShare;
    const Vec3 shift = clampLength(planar(body.foot[sup] - base) * (1.0f / std::max(0.3f, settings.pelvisMassShare)),
        settings.maximumPelvisShift);
    // O joelho de apoio dobra com o alcance (o corpo abaixa).
    const float reachDemand = std::clamp((m_plan.reach - 0.80f) / 0.15f, 0.0f, 1.0f);
    const float drop = 0.02f + 0.06f * reachDemand + 0.4f * m_plan.pelvisNeed;
    // O tronco inclina para o lado contrario da perna e a pelve abre para
    // ela (o peito devolve parte do giro).
    const float sideSign = side == 0 ? 1.0f : -1.0f;
    const Vec3 push = acting ? m_gesture.push() : m_plan.push;
    const float yaw = std::clamp(signedAngle(forward, push) * 0.25f, -0.30f, 0.30f);
    const float bend = m_gestureIntent.kind == BallActionKind3D::Shot && acting ? 0.08f : 0.04f;
    // Os bracos pelo giro da perna (o balanco: 0,15 parado, ate 0,85 com o
    // pe a ~11 m/s - o toque abre pouco, o chute muito): o do lado do apoio
    // abre e vem a frente (no chute, quase na altura do ombro, o
    // equilibrio), o do lado da perna vai para tras.
    const float swing = std::clamp(0.15f + (acting ? m_gesture.velocity().length() : 0.0f) / 16.0f, 0.0f, 0.85f);
    const float open = m_armSwing.value();
    const Vec3 outwardSupport = left * (support == 0 ? 1.0f : -1.0f);
    const Vec3 outwardKick = left * sideSign;
    std::array<Vec3, 2> wrist {};
    wrist[sup] = forward * (0.12f + 0.20f * open) + outwardSupport * (0.22f + 0.25f * open) - up * (0.42f - 0.30f * open);
    wrist[static_cast<std::size_t>(side)] = forward * -(0.06f + 0.22f * open) + outwardKick * (0.18f + 0.12f * open)
        - up * (0.44f - 0.18f * open);
    // O olhar: na bola, antes e depois do contato (ela sai andando, e o
    // olhar vai junto; um ponto a frente no rumo dela saltava ~2 m no toque
    // e, com a bola atras, cruzava por baixo da cabeca girando-a de uma vez).
    const Vec3 look = input.ball.valid ? input.ball.position : m_look.value();

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
    if (weight <= 0.001f && wantedWeight <= 0.0f && !acting && !settlingLeg) {
        m_bodyLive = false;
        m_appliedShift = {};
        m_appliedDrop = 0.0f;
        return;
    }

    BallBodyTargets3D& t = output.body;
    t.active = true;
    t.side = side;
    t.weight = weight;
    if (settlingLeg) {
        const float u = std::clamp(m_settleLeft / std::max(0.01f, settings.handOverSeconds), 0.0f, 1.0f);
        t.ankle = m_lastAnkle;
        t.foot = m_lastFoot;
        t.knee = m_lastKnee;
        t.legWeight = u * u * (3.0f - 2.0f * u);
    }
    if (acting) {
        const std::size_t s = static_cast<std::size_t>(side);
        t.ankle = m_gesture.position() + m_gesture.orientation().rotate(m_legs.ankleInFoot[s]);
        t.foot = m_gesture.orientation();
        t.knee = kneeDirection(t.ankle - body.hip[s], forward,
            planar(m_gesture.orientation().rotate({ 1.0f, 0.0f, 0.0f })));
        {
            // (O polo sai do joelho da pose aos poucos: no balanco da corrida
            // ele apontava diferente e a canela saltava no primeiro tick.)
            const float u = std::clamp(m_gesture.time() / std::max(0.01f, settings.kneeBlendSeconds), 0.0f, 1.0f);
            const Vec3 blended = m_kneeStart + (t.knee - m_kneeStart) * (u * u * (3.0f - 2.0f * u));
            if (blended.lengthSquared() > 1e-6f) t.knee = blended.normalized();
        }
        t.legWeight = 1.0f;
        output.actingSide = side;
        // (O apoio preso: parado, ate a volta; andando, so ate o contato - a
        // passada segue e ele sai quando o estilo pedir.)
        // (Correndo, o toque nao prende o apoio.)
        const bool beforeContact = m_gesture.phase() == BallGesturePhase3D::Windup
            || m_gesture.phase() == BallGesturePhase3D::Strike;
        const bool hold = m_gestureMoving ? beforeContact && m_gestureIntent.kind == BallActionKind3D::Shot
                                          : m_gesture.phase() != BallGesturePhase3D::Return;
        output.reservedSide = hold ? support : -1;
        m_lastAnkle = t.ankle;
        m_lastFoot = t.foot;
        m_lastKnee = t.knee;
        m_lastSide = side;
        m_settleLeft = settings.handOverSeconds;
        if (m_gesture.phase() != BallGesturePhase3D::Return && !m_gestureTouched) {
            output.sensorActive = true;
            output.sensorBallKey = m_gestureBall;
            output.sensorLink = input.footLinks[s];
            output.sensorMargin = settings.sensorMargin;
        }
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
    // (Sem a bola, o olhar sai numa mola: parado no ultimo ponto, a cabeca
    // virava para ele com o corpo passando por cima.)
    t.lookWeight = std::clamp(m_lookWeight.value(), 0.0f, 1.0f);
    // As pernas nunca se cruzam.
    t.separateGap = settings.planner.sideGap;
    t.separateForward = forward;
    t.separateWeight = 1.0f;
    m_appliedShift = planar(t.pelvisShift) * weight;
    m_appliedDrop = t.pelvisDrop * weight;
}

} // namespace MatterEngine
