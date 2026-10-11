#include "Engine/Athlete/Dribble/Dribble3D.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>

namespace MatterEngine {
namespace {

Vec3 planar(Vec3 v) { return { v.x, v.y, 0.0f }; }

Vec3 rotateZ(Vec3 v, float angle) {
    const float c = std::cos(angle), s = std::sin(angle);
    return { c * v.x - s * v.y, s * v.x + c * v.y, v.z };
}

float wrap(float angle) { return std::remainder(angle, 6.2831853f); }

// Um passo da mola criticamente amortecida (o tempo dela em seconds).
void spring(float& value, float& rate, float target, float seconds, float dt) {
    const float omega = 1.0f / std::max(0.005f, seconds);
    const float x = value - target;
    const float e = std::exp(-omega * dt);
    const float temp = (rate + omega * x) * dt;
    rate = (rate - omega * temp) * e;
    value = target + (x + temp) * e;
}

void spring(Vec3& value, Vec3& rate, Vec3 target, float seconds, float dt) {
    spring(value.x, rate.x, target.x, seconds, dt);
    spring(value.y, rate.y, target.y, seconds, dt);
    spring(value.z, rate.z, target.z, seconds, dt);
}

} // namespace

void Dribble3D::reset(const RagdollProfile3D& profile) {
    m_feet = resolveContactFeet3D(profile);
    m_clock = 0.0f;
    m_requests.clear();
    m_swings = {};
    m_clipKnown = false;
    m_yawKnown = false;
    m_yawRate = 0.0f;
    m_bodyWeight = 0.0f;
    m_bodyPlans = {};
    m_bodyValue = {};
    m_bodyRate = {};
    m_reachDrop = 0.0f;
    m_reachDropRate = 0.0f;
    m_lookKnown = false;
    m_ballKey = 0;
    m_sinceTouch = 10.0f;
}

BallOutcomeRequest3D Dribble3D::intentOf(const Request& request, const DribbleSettings3D& settings) const {
    if (!request.plan.setup) return request.intent;
    BallOutcomeRequest3D setup = request.intent;
    setup.kind = BallActionKind3D::Dribble;
    setup.power = settings.setupPower;
    setup.lift = 0.0f;
    setup.curve = 0.0f;
    setup.direction = Vec3 { std::cos(request.intent.aimYawRadians), std::sin(request.intent.aimYawRadians), 0.0f };
    return setup;
}

SwingLive3D Dribble3D::live(const BallHandlingInput3D& input, int side) const {
    const std::size_t s = static_cast<std::size_t>(std::clamp(side, 0, 1));
    const float dt = std::max(input.deltaTime, 1e-4f);
    // (O corpo e o pe do clipe vem da pose do tick anterior: um passo a
    // frente.)
    const Vec3 ahead = planar(input.bodyVelocity) * dt;
    SwingLive3D live;
    live.pelvisGround = Vec3 { input.body.pelvis.x, input.body.pelvis.y, input.body.groundHeight } + ahead;
    live.yaw = input.body.facingYaw;
    live.bodyVelocity = planar(input.bodyVelocity);
    live.hip = input.body.hip[s] + ahead;
    live.clipAnkle = m_clipAnkle[s] + m_clipVelocity[s] * dt;
    live.clipVelocity = m_clipVelocity[s];
    live.clipAcceleration = m_clipAcceleration[s];
    live.clipOrientation = input.gait.clipFootOrientation[s];
    live.clipKnee = input.gait.clipKnee[s];
    live.forecast = &m_forecast;
    live.groundHeight = input.body.groundHeight;
    return live;
}

void Dribble3D::applyForce(const Request& request, BallTouchSurface3D surface, const BallHandlingInput3D& input,
    const BallOutcomeSettings3D& outcome, const DribbleSettings3D& settings, DribbleOutput3D& output) {
    if (!input.ball.valid) return;
    BallOutcomeRequest3D intent = request.intent;
    // O toque vai para onde o jogador vai agora (as teclas na hora da forca).
    if (intent.kind != BallActionKind3D::Shot && planar(input.aheadVelocity).length() > 0.3f)
        intent.direction = planar(input.aheadVelocity).normalized();
    intent.surface = surface;
    // A velocidade "a frente": a do corpo indo a pedida por carryAhead (a do
    // manejo V3, onde a forca foi calibrada).
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
    output.impulseBallKey = request.ballKey != 0 ? request.ballKey : input.ball.key;
    output.linearImpulse = (result.velocity - input.ball.velocity) * input.ball.mass;
    output.angularImpulse = (result.spin - input.ball.spin) * inertia;
    output.outcomeVelocity = result.velocity;
    output.touched = true;
    output.touchSurface = surface;
}

void Dribble3D::update(const BallHandlingInput3D& input, const DribbleGaitForecast3D& gait,
    const DribbleCapsule3D& capsule, const BallOutcomeSettings3D& outcome, const DribbleSettings3D& settings,
    DribbleOutput3D& output) {
    output = {};
    if (!m_feet.valid) return;
    const float dt = std::max(input.deltaTime, 1e-4f);
    m_clock += dt;
    // O pe do clipe (o tornozelo) e a velocidade e a aceleracao dele.
    for (std::size_t s = 0; s < 2; ++s) {
        const Vec3 ankle = input.gait.clipFoot[s] + input.gait.clipFootOrientation[s].rotate(m_feet.legs.ankleInFoot[s]);
        const Vec3 velocity = m_clipKnown ? (ankle - m_clipAnkle[s]) * (1.0f / dt) : Vec3 {};
        const Vec3 acceleration = m_clipKnown ? (velocity - m_clipVelocity[s]) * (1.0f / dt) : Vec3 {};
        m_clipAcceleration[s] = m_clipKnown ? m_clipAcceleration[s] + (acceleration - m_clipAcceleration[s]) * 0.5f : Vec3 {};
        m_clipVelocity[s] = velocity;
        m_clipAnkle[s] = ankle;
    }
    m_clipKnown = true;
    {
        const float yaw = input.body.facingYaw;
        const float rate = m_yawKnown ? wrap(yaw - m_lastYaw) / dt : 0.0f;
        m_yawRate += (rate - m_yawRate) * std::min(1.0f, dt / 0.05f);
        m_lastYaw = yaw;
        m_yawKnown = true;
    }
    const float radius = input.ball.valid ? input.ball.radius : 0.11f;
    const Vec3 ahead = planar(input.bodyVelocity) * dt;
    const Vec3 pelvisGround = Vec3 { input.body.pelvis.x, input.body.pelvis.y, input.body.groundHeight } + ahead;
    const float pelvisHeight = std::max(0.3f, input.body.pelvis.z - input.body.groundHeight);
    const bool ballInside = input.ball.valid
        && dribbleInArea3D(input.ball.position, radius, input.body.pelvis, input.bodyVelocity, settings);

    // No corpo fisico (ate o E9): so a forca, com a bola na area.
    if (!input.animatedBody) {
        if (input.request && ballInside) {
            Request bare;
            bare.intent = input.intent;
            bare.ballKey = input.requestBallKey;
            applyForce(bare, BallTouchSurface3D::Inside, input, outcome, settings, output);
            output.touchGuaranteed = true;
        }
        m_requests.clear();
        m_swings[0].stop();
        m_swings[1].stop();
        m_bodyWeight = 0.0f;
        return;
    }

    // 1) O clique novo entra na fila (cheia, o ultimo pendente vira ele).
    if (input.request) {
        Request request;
        request.id = m_nextId++;
        request.intent = input.intent;
        request.ballKey = input.requestBallKey;
        request.sawBall = ballInside;
        int waiting = 0;
        for (const Request& r : m_requests) waiting += !r.committed && !r.done ? 1 : 0;
        if (waiting >= std::max(1, settings.queueSize)) {
            for (auto it = m_requests.rbegin(); it != m_requests.rend(); ++it)
                if (!it->committed && !it->done && !it->armed) {
                    // (O pendente mais novo vira o clique de agora: a intencao
                    // mais recente; ele ja tinha visto a bola.)
                    request.sawBall = request.sawBall || it->sawBall;
                    *it = request;
                    ++output.merged;
                    break;
                }
        } else {
            m_requests.push_back(request);
        }
        m_ballKey = request.ballKey;
    }

    // 2) As previsoes: o corpo e a bola.
    {
        BodyForecastInput3D in;
        in.pelvisGround = pelvisGround;
        in.velocity = planar(input.body.velocity);
        in.requested = planar(input.aheadVelocity);
        in.acceleration = capsule.acceleration;
        in.deceleration = capsule.deceleration;
        in.brake = capsule.brake;
        in.turnRate = capsule.turnRate;
        in.facingYaw = input.body.facingYaw;
        in.facingRate = m_yawRate;
        for (std::size_t s = 0; s < 2; ++s)
            in.hipInBody[s] = rotateZ(input.body.hip[s] + ahead - pelvisGround, -input.body.facingYaw);
        in.horizon = settings.horizon + 0.25f;
        m_forecast.build(in);
    }
    // (A bola no tempo da pose: o que sai deste tick aparece daqui a um tick,
    // e a bola de agora esta um tick antes disso.)
    DribbleBallPath3D realBall;
    if (input.ball.valid)
        realBall.build(input.ball.position, input.ball.velocity, input.body.groundHeight, radius, -dt,
            settings.horizon + 0.5f);
    const auto planInput = [&](const Request& request, const DribbleBallPath3D& ball) {
        TouchPlanInput3D in;
        in.feet = &m_feet;
        in.body = &m_forecast;
        in.ball = &ball;
        in.gait = &gait;
        in.settings = &settings;
        in.intent = request.intent;
        in.ballRadius = radius;
        in.now = m_clock;
        in.earliest = m_clock + dt;
        in.tick = dt;
        in.cycling = input.gait.cycling && gait.valid;
        in.bodyVelocity = planar(input.bodyVelocity);
        in.pelvisHeight = pelvisHeight;
        in.footNow = input.body.foot;
        for (std::size_t s = 0; s < 2; ++s)
            in.footYawNow[s] = athleteFootHeading3D(m_feet.geometry, static_cast<int>(s), input.gait.clipFootOrientation[s]);
        return in;
    };
    // A bola depois de um toque planejado (o encadeamento).
    const auto chainAfter = [&](const TouchPlan3D& plan, const BallOutcomeRequest3D& intent,
                                const DribbleBallPath3D& before, DribbleBallPath3D& after) {
        const float t = plan.contact - m_clock;
        BallOutcomeRequest3D predicted = intent;
        predicted.surface = plan.surface;
        const Vec3 velocity = dribbleTouchVelocity3D(predicted, before.velocity(t), m_forecast.velocity(t),
            planar(input.aheadVelocity), radius, outcome);
        after.build(plan.ball, velocity, input.body.groundHeight, radius, std::max(0.0f, t), settings.horizon + 0.5f);
    };

    // 2b) A forca do toque cuja pose de contato saiu no tick anterior (a pose
    // aparece agora). A bola longe da prevista (a previsao errou: o toque
    // anterior saiu diferente, a bola bateu em algo): nada de forca a
    // distancia - o pedido volta para a fila.
    bool impulseUsed = false;
    for (Request& request : m_requests) {
        if (!request.armed || request.done) continue;
        request.armed = false;
        if (!input.ball.valid) continue;
        if (planar(input.ball.position - request.plan.ball).length() > settings.missDistance) {
            request.committed = false;
            request.plan = {};
            ++output.missed;
            continue;
        }
        if (impulseUsed) continue;
        Request touch = request;
        touch.intent = intentOf(request, settings);
        applyForce(touch, request.plan.surface, input, outcome, settings, output);
        impulseUsed = true;
        output.touchSide = request.plan.side;
        output.touchMiss = (input.ball.position - request.plan.ball).length();
        if (request.plan.setup) {
            // (O preparo: o chute vem depois, no mesmo pedido.)
            request.setupDone = true;
            request.committed = false;
            request.plan = {};
        } else {
            request.done = true;
        }
    }

    // 3) Os pedidos em ordem: cada um planejado na bola que o anterior vai
    // mandar, depois dele e em outro balanco.
    if (input.ball.valid) {
        float earliest = m_clock + dt;
        std::array<float, 2> busy {};
        std::array<float, 2> used { -1.0f, -1.0f };
        for (std::size_t s = 0; s < 2; ++s) {
            if (!m_swings[s].active()) continue;
            busy[s] = m_swings[s].plan().end;
            if (!m_swings[s].plan().step) used[s] = m_swings[s].plan().end;
        }
        DribbleBallPath3D chain = realBall;
        for (Request& request : m_requests) {
            if (request.done) continue;
            if (request.committed) {
                earliest = std::max(earliest, request.plan.contact + settings.touchGap);
                DribbleBallPath3D next;
                chainAfter(request.plan, intentOf(request, settings), chain, next);
                chain = next;
                continue;
            }
            TouchPlanInput3D in = planInput(request, chain);
            in.earliest = earliest;
            in.busyUntil = busy;
            in.usedWindowEnd = used;
            request.plan = planTouch3D(in);
            // (O chute sem jeito com a bola onde esta - atras do corpo -: antes
            // a sola a traz para a frente.)
            if (!request.plan.valid && request.intent.kind == BallActionKind3D::Shot && !request.setupDone
                && request.plan.candidates > 0 && request.plan.rejected[0] == request.plan.candidates) {
                Request setup = request;
                setup.plan.setup = true;
                in.intent = intentOf(setup, settings);
                request.plan = planTouch3D(in);
                request.plan.setup = request.plan.valid;
            }
            if (!request.plan.valid) break;
            const std::size_t s = static_cast<std::size_t>(request.plan.side);
            earliest = request.plan.contact + settings.touchGap;
            busy[s] = std::max(busy[s], request.plan.end);
            if (!request.plan.step) used[s] = request.plan.end;
            DribbleBallPath3D next;
            chainAfter(request.plan, intentOf(request, settings), chain, next);
            chain = next;
        }
    }

    // 4) O compromisso: o primeiro pedido pendente, quando a janela dele
    // comeca (a perna livre).
    for (Request& request : m_requests) {
        if (request.done || request.committed) continue;
        if (!request.plan.valid) break;
        const std::size_t s = static_cast<std::size_t>(request.plan.side);
        if (!m_swings[s].active() && request.plan.start <= m_clock + 1e-4f) {
            m_swings[s].begin(request.plan, request.id, m_clock, live(input, request.plan.side), m_feet, settings);
            request.committed = true;
        }
        break;
    }

    // 5) Antes da trava, cada toque comprometido se ajusta (o mesmo balanco),
    // na bola que o toque anterior vai mandar (o encadeamento: so o primeiro
    // se ajustando, o seguinte ia para a bola prevista antes e errava).
    if (input.ball.valid) {
        DribbleBallPath3D chain = realBall;
        float earliest = m_clock + dt;
        for (Request& request : m_requests) {
            if (request.done) continue;
            if (!request.committed || !request.plan.valid) break;
            const std::size_t s = static_cast<std::size_t>(request.plan.side);
            SwingTouch3D& swing = m_swings[s];
            if (swing.active() && swing.request() == request.id && !swing.contacted()
                && swing.plan().contact - m_clock > settings.lockLead) {
                TouchPlanInput3D in = planInput(request, chain);
                in.intent = intentOf(request, settings);
                in.earliest = earliest;
                in.onlySide = static_cast<int>(s);
                in.onlyWindowEnd = swing.plan().step ? -1.0f : swing.plan().end;
                in.committedContact = swing.plan().contact;
                in.committedStart = swing.plan().start;
                in.swingFoot = swing.ankle() - swing.orientation().rotate(m_feet.legs.ankleInFoot[s]);
                in.swingFootYaw = athleteFootHeading3D(m_feet.geometry, static_cast<int>(s), swing.orientation());
                in.committedStyle = swing.plan().style;
                in.committedSurface = swing.plan().surface;
                const TouchPlan3D again = planTouch3D(in);
                if (again.valid) {
                    TouchPlan3D kept = again;
                    kept.setup = swing.plan().setup;
                    kept.start = swing.plan().start;
                    kept.end = swing.plan().step ? again.end : swing.plan().end;
                    kept.followEnd = std::min(kept.end, again.followEnd);
                    swing.retarget(kept);
                    request.plan = swing.plan();
                }
            }
            // (Nao coube mais - antes ou depois da trava: desiste.)
            if (swing.active() && swing.request() == request.id && !swing.contacted()) {
                TouchPlanInput3D in = planInput(request, chain);
                in.swingFoot = swing.ankle() - swing.orientation().rotate(m_feet.legs.ankleInFoot[s]);
                if (!touchPlanFeasible3D(in, swing.plan(), settings.abortPaceShare)) {
                    if (std::getenv("XDRIBBLE"))
                        std::printf("  t %.3f DESISTE perna %zu | contato em %+.3f fim %+.3f\n", m_clock, s,
                            swing.plan().contact - m_clock, swing.plan().end - m_clock);
                    swing.abort(m_clock, live(input, static_cast<int>(s)), settings.abortFollow);
                    ++output.aborted;
                    request.committed = false;
                    request.plan = {};
                    break;
                }
            }
            earliest = request.plan.contact + settings.touchGap;
            DribbleBallPath3D next;
            chainAfter(request.plan, intentOf(request, settings), chain, next);
            chain = next;
        }
    }

    // 6) Os balancos; no contato, a forca fica armada para o proximo tick.
    for (std::size_t s = 0; s < 2; ++s) {
        SwingTouch3D& swing = m_swings[s];
        if (!swing.active()) continue;
        swing.update(m_clock, live(input, static_cast<int>(s)), m_feet, settings, output.legs[s]);
        if (!swing.contactedNow()) continue;
        auto it = std::find_if(m_requests.begin(), m_requests.end(), [&](const Request& r) { return r.id == swing.request(); });
        if (it == m_requests.end() || it->done) continue;
        // (O pe nao chegou na pose do contato - o corpo nao foi para onde a
        // previsao dizia e o plano refeito nao coube: a trava do alcance o
        // segurou: errou, o pedido volta para a fila.)
        if ((output.legs[s].ankle - swing.plan().pose.ankle).length() > settings.missDistance) {
            it->committed = false;
            it->plan = {};
            ++output.missed;
            continue;
        }
        if (std::getenv("XDRIBBLE")) {
            const Vec3 p = swing.plan().pose.ankle, a = output.legs[s].ankle;
            std::printf("  t %.3f CONTATO perna %zu | plano %.3f %.3f %.3f saida %.3f %.3f %.3f (%.3f m) | bola prevista %.3f %.3f %.3f sup %d estilo %d\n",
                m_clock, s, p.x, p.y, p.z, a.x, a.y, a.z, (p - a).length(), swing.plan().ball.x, swing.plan().ball.y,
                swing.plan().ball.z, static_cast<int>(swing.plan().surface), static_cast<int>(swing.plan().style));
            const Quaternion q = swing.plan().pose.orientation;
            std::printf("  t %.3f CONTATO pe plano rumo %.1f graus (pe do clipe %.1f, corpo %.1f) | joelho %.2f %.2f\n", m_clock,
                athleteFootYaw3D(m_feet.geometry, static_cast<int>(s), q) * 57.2958f,
                athleteFootYaw3D(m_feet.geometry, static_cast<int>(s), input.gait.clipFootOrientation[s]) * 57.2958f,
                input.body.facingYaw * 57.2958f, output.legs[s].knee.x, output.legs[s].knee.y);
        }
        it->armed = true;
        it->plan = swing.plan();
    }

    // 7) A garantia (a bola na area sempre vira forca) e os pedidos vencidos.
    for (Request& request : m_requests) {
        if (request.done) continue;
        if (ballInside) request.sawBall = true;
        request.outside = ballInside ? 0.0f : request.outside + dt;
        request.age += dt;
        if (request.armed) continue;
        bool forceNow = false;
        // (A bola saindo da area antes do contato: a forca antes dela sair - a
        // regra da V3. Fora isso o pedido espera a bola ficar ao alcance do pe:
        // com ela na area, ate guaranteeLife; fora, ate requestLife.)
        const bool leaving = [&] {
            if (!ballInside || !realBall.valid()) return false;
            if (request.committed && request.plan.inArea) return false;
            const float soon = 2.0f * dt;
            const Vec3 pelvisSoon = m_forecast.pelvisGround(soon) + Vec3 { 0.0f, 0.0f, pelvisHeight };
            return !dribbleInArea3D(realBall.position(soon), radius, pelvisSoon, m_forecast.velocity(soon), settings);
        }();
        if (leaving) {
            forceNow = true;
        } else if (!request.committed) {
            if (ballInside && request.age > settings.guaranteeLife) forceNow = true;
            else if (!ballInside && (request.sawBall ? request.outside : request.age) > settings.requestLife)
                request.done = true;
        }
        if (forceNow && !impulseUsed && input.ball.valid) {
            const BallTouchSurface3D surface = request.plan.valid ? request.plan.surface : BallTouchSurface3D::Inside;
            applyForce(request, surface, input, outcome, settings, output);
            output.touchGuaranteed = true;
            request.done = true;
            impulseUsed = true;
            // (Os outros pedidos que esperavam a mesma bola sair viram essa
            // forca: antes cada um saia num tick, tres empurroes seguidos.)
            if (leaving)
                for (Request& other : m_requests)
                    if (!other.done && !other.committed && !other.armed) other.done = true;
        } else if (forceNow && leaving && impulseUsed) {
            request.done = true;
        }
    }
    m_requests.erase(std::remove_if(m_requests.begin(), m_requests.end(),
                         [&](const Request& r) {
                             if (!r.done) return false;
                             for (const SwingTouch3D& swing : m_swings)
                                 if (swing.active() && swing.request() == r.id && !swing.contacted()) return false;
                             return true;
                         }),
        m_requests.end());

    if (std::getenv("XDRIBBLE")) {
        for (std::size_t s = 0; s < 2; ++s) {
            if (!m_swings[s].active() && output.legs[s].weight <= 0.0f) continue;
            const SwingLive3D l = live(input, static_cast<int>(s));
            const Vec3 a = output.legs[s].ankle;
            std::printf("  t %.3f balanco %zu contato %d | alvo %.3f %.3f %.3f | clipe %.3f %.3f %.3f | quadril %.3f %.3f %.3f alcance %.2f | contato em %+.3f fim %+.3f | joelho %.2f %.2f %.2f clipe %.2f %.2f %.2f peso %.2f\n",
                m_clock, s, m_swings[s].contacted() ? 1 : 0, a.x, a.y, a.z, l.clipAnkle.x, l.clipAnkle.y, l.clipAnkle.z,
                l.hip.x, l.hip.y, l.hip.z, (a - l.hip).length() / std::max(0.1f, m_feet.legLength[s]),
                m_swings[s].plan().contact - m_clock, m_swings[s].plan().end - m_clock, output.legs[s].knee.x,
                output.legs[s].knee.y, output.legs[s].knee.z, l.clipKnee.x, l.clipKnee.y, l.clipKnee.z, output.legs[s].weight);
        }
        std::printf("  t %.3f corpo %.2f m/s ciclo %d\n", m_clock, planar(input.bodyVelocity).length(), input.gait.cycling ? 1 : 0);
        for (const Request& r : m_requests)
            std::printf("  r%d idade %.2f comp %d feito %d viu %d | plano %d perna %d passo %d janela %.3f..%.3f contato %+.3f custo %.2f area %d | cand %d sem estilo %d alcance %d descida %d aprox %d ritmo %d passo %d cruza %d fora %d giro %d\n",
                r.id, r.age, r.committed ? 1 : 0, r.done ? 1 : 0, r.sawBall ? 1 : 0, r.plan.valid ? 1 : 0, r.plan.side,
                r.plan.step ? 1 : 0, r.plan.start - m_clock, r.plan.end - m_clock, r.plan.contact - m_clock, r.plan.cost,
                r.plan.inArea ? 1 : 0, r.plan.candidates, r.plan.rejected[0], r.plan.rejected[1], r.plan.rejected[2],
                r.plan.rejected[3], r.plan.rejected[4], r.plan.rejected[5], r.plan.rejected[6], r.plan.rejected[7],
                r.plan.rejected[8]);
    }
    // 8) O corpo, a passada de conducao, o dono da bola e a telemetria.
    bodyLayer(input, settings, output);
    m_sinceTouch = output.touched ? 0.0f : m_sinceTouch + dt;
    output.dribbleStyle = !m_requests.empty() || anySwing() || m_sinceTouch < settings.dribbleLinger ? 1.0f : 0.0f;
    if (std::getenv("XCORPO"))
        std::printf("  t %.3f corpo peso %.3f desce %.3f | olhar %.3f %.3f %.3f peso %.3f | bracos %.3f %.3f punho %.2f %.2f %.2f / %.2f %.2f %.2f | tronco %.3f %.3f %.3f pelve %.3f %.3f %.3f\n", m_clock, output.body.weight,
            output.body.pelvisDrop, output.body.look.x, output.body.look.y, output.body.look.z, output.body.lookWeight,
            output.body.armWeight[0], output.body.armWeight[1], output.body.wrist[0].x, output.body.wrist[0].y,
            output.body.wrist[0].z, output.body.wrist[1].x, output.body.wrist[1].y, output.body.wrist[1].z,
            output.body.trunkTwist, output.body.trunkBend, output.body.trunkSide, output.body.pelvisYaw,
            output.body.pelvisPitch, output.body.pelvisRoll);
    output.interacting = !m_requests.empty() || anySwing();
    output.interactingBallKey = m_ballKey;
    for (const Request& r : m_requests) output.pending += r.done ? 0 : 1;
}

void Dribble3D::bodyLayer(const BallHandlingInput3D& input, const DribbleSettings3D& settings, DribbleOutput3D& output) {
    const float dt = std::max(input.deltaTime, 1e-4f);
    // O corpo de cada toque (o plano dele refeito a cada tick enquanto o
    // balanco anda; depois, ate esmaecer).
    const float speed = planar(input.bodyVelocity).length();
    for (std::size_t s = 0; s < 2; ++s)
        if (m_swings[s].active())
            m_bodyPlans[s] = touchBodyPlan3D(m_swings[s].plan(), speed > 0.6f,
                static_cast<std::uint32_t>(m_swings[s].request()), settings.bodyFade);
    const float share = 1.0f - (1.0f - std::clamp(settings.bodyRunningShare, 0.0f, 1.0f))
        * std::clamp((speed - 1.0f) / 3.0f, 0.0f, 1.0f);
    const Vec3 forward { std::cos(input.body.facingYaw), std::sin(input.body.facingYaw), 0.0f };
    DribbleBody3D keys;
    float arms = 0.0f;
    bool keyed = false;
    for (std::size_t s = 0; s < 2; ++s) {
        TouchBodyPlan3D& body = m_bodyPlans[s];
        if (!body.valid) continue;
        if (m_clock >= body.times[4]) {
            body.valid = false;
            continue;
        }
        keyed = true;
        arms = std::max(arms, touchBodyPose3D(body, m_clock, share, forward,
            m_clipAnkle[1 - s] - input.body.pelvis, keys));
    }
    // (Pelas molas.)
    {
        const float k = settings.bodySpringSeconds;
        spring(m_bodyValue.pelvisDrop, m_bodyRate.pelvisDrop, keys.pelvisDrop, k, dt);
        spring(m_bodyValue.pelvisShift, m_bodyRate.pelvisShift, keys.pelvisShift, k, dt);
        spring(m_bodyValue.pelvisYaw, m_bodyRate.pelvisYaw, keys.pelvisYaw, k, dt);
        spring(m_bodyValue.pelvisPitch, m_bodyRate.pelvisPitch, keys.pelvisPitch, k, dt);
        spring(m_bodyValue.pelvisRoll, m_bodyRate.pelvisRoll, keys.pelvisRoll, k, dt);
        spring(m_bodyValue.trunkTwist, m_bodyRate.trunkTwist, keys.trunkTwist, k, dt);
        spring(m_bodyValue.trunkBend, m_bodyRate.trunkBend, keys.trunkBend, k, dt);
        spring(m_bodyValue.trunkSide, m_bodyRate.trunkSide, keys.trunkSide, k, dt);
        // (Os punhos so andam com o braco no gesto: sem ele, ficam onde
        // estao - o peso do braco faz a mistura.)
        for (std::size_t s = 0; s < 2; ++s)
            if (arms > 0.0f) spring(m_bodyValue.wrist[s], m_bodyRate.wrist[s], keys.wrist[s], k, dt);
        spring(m_bodyValue.armWeight[0], m_bodyRate.armWeight[0], arms, k, dt);
        m_bodyValue.armWeight[0] = std::clamp(m_bodyValue.armWeight[0], 0.0f, 1.0f);
        keys = m_bodyValue;
        arms = m_bodyValue.armWeight[0];
        keyed = keyed || std::abs(arms) > 1e-3f || std::abs(keys.pelvisDrop) > 1e-4f || std::abs(keys.trunkSide) > 1e-4f
            || std::abs(keys.trunkTwist) > 1e-4f || std::abs(keys.pelvisYaw) > 1e-4f;
    }
    const float blend = std::min(1.0f, dt / std::max(0.01f, settings.bodySeconds));
    const bool live = keyed || anySwing() || !m_requests.empty();
    m_bodyWeight += ((live ? 1.0f : 0.0f) - m_bodyWeight) * blend;
    m_lookWeight += ((live && input.ball.valid ? settings.lookWeight : 0.0f) - m_lookWeight)
        * std::min(1.0f, dt / std::max(0.01f, settings.lookSeconds));
    if (input.ball.valid) {
        if (!m_lookKnown) m_look = input.ball.position;
        m_look += (input.ball.position - m_look) * std::min(1.0f, dt / 0.08f);
        m_lookKnown = true;
    }
    if (m_bodyWeight < 0.001f && m_lookWeight < 0.001f) return;
    // (A camada multiplica pelo peso: os valores vem divididos por ele - e
    // com o peso ainda baixo, ele so nao amplifica.)
    const float weight = std::max(m_bodyWeight, 0.001f);
    const float inverse = 1.0f / std::max(weight, 0.3f);
    // A descida que o alcance pediu, direto pelo tempo do balanco: inteira no
    // contato, entrando na ida e saindo na volta.
    // (Do comeco de fato do balanco - a janela podia ja ter comecado - e numa
    // mola curta: o plano refeito mudava a descida a cada tick.)
    float reachDrop = 0.0f;
    for (const SwingTouch3D& swing : m_swings) {
        if (!swing.active()) continue;
        const TouchPlan3D& plan = swing.plan();
        const float start = swing.startTime();
        const float in = (m_clock - start) / std::max(0.02f, plan.contact - start);
        const float out = (m_clock - plan.contact) / std::max(0.02f, plan.followEnd - plan.contact);
        const float u = m_clock < plan.contact ? std::clamp(in, 0.0f, 1.0f) : 1.0f - std::clamp(out, 0.0f, 1.0f);
        reachDrop = std::max(reachDrop, plan.drop * u * u * (3.0f - 2.0f * u));
    }
    spring(m_reachDrop, m_reachDropRate, reachDrop, 0.03f, dt);
    reachDrop = std::max(0.0f, m_reachDrop);
    output.body = keys;
    output.body.weight = weight;
    output.body.pelvisDrop = (keys.pelvisDrop + reachDrop) * inverse;
    output.body.pelvisShift = keys.pelvisShift * inverse;
    output.body.pelvisYaw = keys.pelvisYaw * inverse;
    output.body.pelvisPitch = keys.pelvisPitch * inverse;
    output.body.pelvisRoll = keys.pelvisRoll * inverse;
    output.body.trunkTwist = keys.trunkTwist * inverse;
    output.body.trunkBend = keys.trunkBend * inverse;
    output.body.trunkSide = keys.trunkSide * inverse;
    output.body.armWeight = { std::min(1.0f, arms * inverse), std::min(1.0f, arms * inverse) };
    output.body.look = m_look;
    output.body.lookWeight = std::min(1.0f, m_lookWeight / weight);
}

} // namespace MatterEngine
