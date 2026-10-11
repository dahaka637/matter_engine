#include "Engine/Athlete/Dribble/TouchPlanner3D.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>

namespace MatterEngine {
namespace {

Vec3 planar(Vec3 v) { return { v.x, v.y, 0.0f }; }

Vec3 unitPlanar(Vec3 v, Vec3 fallback) {
    const Vec3 p = planar(v);
    return p.lengthSquared() > 1e-8f ? p.normalized() : fallback;
}

// O pe do clipe no instante t (s a partir de agora), no mundo: a previsao da
// locomocao (no referencial do corpo) levada pelo corpo previsto.
bool clipFootAt(const TouchPlanInput3D& in, int side, float t, Vec3& foot) {
    if (in.gait == nullptr || !in.gait->valid) return false;
    const DribbleGaitForecast3D& g = *in.gait;
    const float u = std::clamp(t / std::max(1e-3f, g.forecastStep), 0.0f,
        static_cast<float>(DribbleGaitForecast3D::ForecastCount - 1));
    const std::size_t a = static_cast<std::size_t>(u);
    const std::size_t b = std::min(a + 1, static_cast<std::size_t>(DribbleGaitForecast3D::ForecastCount - 1));
    const float f = u - static_cast<float>(a);
    const auto& table = g.foot[static_cast<std::size_t>(side)];
    foot = in.body->toWorld(table[a] + (table[b] - table[a]) * f, t);
    return true;
}

// O rumo do pe do clipe no instante t (rad, mundo).
bool clipFootYawAt(const TouchPlanInput3D& in, int side, float t, float& yaw) {
    if (in.gait == nullptr || !in.gait->valid || in.feet == nullptr) return false;
    const DribbleGaitForecast3D& g = *in.gait;
    const float u = std::clamp(t / std::max(1e-3f, g.forecastStep), 0.0f,
        static_cast<float>(DribbleGaitForecast3D::ForecastCount - 1));
    const std::size_t a = static_cast<std::size_t>(std::round(u));
    const Quaternion turn = g.footTurn[static_cast<std::size_t>(side)][a];
    yaw = in.body->yaw(t) + athleteFootHeading3D(in.feet->geometry, side, turn);
    return true;
}

} // namespace

bool dribbleInArea3D(Vec3 ball, float radius, Vec3 pelvis, Vec3 velocity, const DribbleSettings3D& s) {
    if (ball.z - radius > pelvis.z + s.waistAbovePelvis) return false;
    if (std::hypot(ball.x - pelvis.x, ball.y - pelvis.y) <= s.areaRadius) return true;
    const Vec3 flat = planar(velocity);
    const float speed = flat.length();
    if (speed <= 0.3f) return false;
    const Vec3 lead = flat * (s.areaLead * std::min(1.0f, speed / std::max(0.5f, s.areaLeadSpeed)) / speed);
    return std::hypot(ball.x - pelvis.x - lead.x, ball.y - pelvis.y - lead.y) <= s.areaRadius;
}

TouchPlan3D planTouch3D(const TouchPlanInput3D& in) {
    TouchPlan3D best;
    if (in.feet == nullptr || !in.feet->valid || in.body == nullptr || in.ball == nullptr || in.settings == nullptr
        || !in.body->valid() || !in.ball->valid())
        return best;
    const DribbleSettings3D& s = *in.settings;
    const bool shot = in.intent.kind == BallActionKind3D::Shot;
    const bool retarget = in.committedContact >= 0.0f;
    const float committed = in.committedContact - in.now;
    const float bodySpeed = planar(in.bodyVelocity).length();
    const bool moving = bodySpeed > 0.6f;
    const auto pushAt = [&](float t) {
        const float yaw = in.body->yaw(t);
        const Vec3 forward { std::cos(yaw), std::sin(yaw), 0.0f };
        if (shot) return Vec3 { std::cos(in.intent.aimYawRadians), std::sin(in.intent.aimYawRadians), 0.0f };
        return unitPlanar(in.intent.direction, forward);
    };
    // Um candidato: a perna `side` encostando na bola em t (s a partir de
    // agora), no balanco que comeca em windowStart e pousa em windowEnd.
    int candidates = 0;
    std::array<int, 9> rejected {};
    const float tick = std::max(1.0f / 1000.0f, in.tick);
    const float dt = tick * std::max(1.0f, std::round(s.candidateStep / tick));
    // (O primeiro candidato na grade: o tick seguinte a `from`.)
    const auto onGrid = [&](float from) { return std::ceil(from / tick - 1e-3f) * tick; };
    const auto evaluate = [&](int side, float t, float windowStart, float windowEnd, bool step) {
        ++candidates;
        const std::size_t sd = static_cast<std::size_t>(side);
        const float out = side == 0 ? 1.0f : -1.0f;
        const Vec3 ball = in.ball->position(t);
        const Vec3 pelvisGround = in.body->pelvisGround(t);
        const Vec3 pelvis = pelvisGround + Vec3 { 0.0f, 0.0f, in.pelvisHeight };
        const float yaw = in.body->yaw(t);
        const Vec3 forward { std::cos(yaw), std::sin(yaw), 0.0f };
        const Vec3 left { -forward.y, forward.x, 0.0f };
        const Vec3 velocity = in.body->velocity(t);
        const Vec3 push = pushAt(t);
        ContactStyle3D style = ContactStyle3D::Front;
        BallTouchSurface3D surface = BallTouchSurface3D::Inside;
        if (retarget) {
            style = in.committedStyle;
            surface = in.committedSurface;
        } else if (!contactStyle3D(side, ball, pelvis, forward, push, moving, in.intent.kind, in.intent.curve, style, surface)) {
            ++rejected[0];
            return;
        }
        ContactRequest3D request;
        request.side = side;
        request.surface = surface;
        request.style = style;
        request.ball = ball;
        request.radius = in.ballRadius;
        request.push = push;
        // (O rumo do pe no contato parte do rumo do pe do clipe nesse instante,
        // nao da frente do corpo: a volta para ele depois do toque fica curta -
        // com a frente prevista, as teclas trocando a cada 0,2 s deixavam o pe
        // 90 graus fora do clipe e a perna girava 60 rad/s na volta.)
        request.bodyForward = forward;
        {
            float clipYaw = in.footYawNow[sd];
            if (step || clipFootYawAt(in, side, t, clipYaw))
                request.bodyForward = Vec3 { std::cos(clipYaw), std::sin(clipYaw), 0.0f };
        }
        request.power = std::clamp(in.intent.power, 0.0f, 1.0f);
        request.lift = in.intent.lift;
        request.groundHeight = pelvisGround.z;
        const ContactPose3D pose = contactPose3D(*in.feet, request);
        // O alcance (a pelve pode descer um pouco).
        const Vec3 hip = in.body->hip(side, t);
        const float legReach = s.reach * in.feet->legLength[sd];
        const Vec3 fromHip = pose.ankle - hip;
        float drop = 0.0f;
        if (fromHip.length() > legReach) {
            const float flat = planar(fromHip).length();
            if (flat >= legReach) {
                ++rejected[1];
                return;
            }
            const float height = std::sqrt(legReach * legReach - flat * flat);
            drop = (hip.z - pose.ankle.z) - height;
            if (drop > (step ? s.dropStanding : s.dropMoving)) {
                ++rejected[2];
                return;
            }
            drop = std::max(0.0f, drop);
        }
        float cost = s.waitWeight * t + 0.3f * drop;
        // (A bola atras tocada para a frente com a sola - o toque do V3 - so se
        // nao houver jeito melhor.)
        if (style == ContactStyle3D::Front && (surface == BallTouchSurface3D::Sole || surface == BallTouchSurface3D::Heel)
            && dot(push, forward) > 0.0f)
            cost += 0.3f;
        // As pernas nunca se cruzam: o tornozelo do lado dele contra o pe de
        // apoio (o do clipe da outra perna; parado, onde ele esta) e quase do
        // lado dele da linha do meio do corpo.
        {
            const float lateral = dot(planar(pose.ankle - pelvisGround), left) * out;
            if (lateral < -s.midlineSlack) cost += 10.0f;
            Vec3 support = in.footNow[1 - sd];
            if (!step) clipFootAt(in, 1 - side, t, support);
            const float apart = dot(planar(pose.foot - support), left) * out;
            const float along = std::abs(dot(planar(pose.foot - support), forward));
            // (So lado a lado: um a frente do outro, de lado tanto faz.)
            if (along < 0.25f && apart < s.footGap) cost += 10.0f * (s.footGap - apart) / s.footGap + 1.0f;
        }
        // A bola do lado da outra perna custa.
        const float ballSide = dot(planar(ball - pelvisGround), left) * out;
        cost += 0.6f * std::max(0.0f, -ballSide);
        // O conforto: a bola a uma distancia boa a frente do quadril.
        const Vec3 travel = planar(velocity).length() > 1.0f ? planar(velocity).normalized() : forward;
        const float ahead = dot(planar(ball - hip), travel);
        const float ideal = surface == BallTouchSurface3D::Sole || surface == BallTouchSurface3D::Heel ? s.idealAheadSole
                                                                                                    : s.idealAhead;
        cost += std::abs(ahead - ideal) + s.lateralWeight * std::max(0.0f, std::abs(dot(planar(ball - pelvisGround), left)) - s.lateralFree);
        // O desvio do caminho natural do pe e o quanto ele precisa andar.
        Vec3 startFoot = in.footNow[sd];
        float startTime = 0.0f;
        if (!step) {
            Vec3 clip;
            startTime = std::max(0.0f, windowStart);
            if (clipFootAt(in, side, t, clip)) {
                const float deviation = (pose.foot - clip).length();
                cost += s.deviationWeight * deviation;
                // (A velocidade a mais que o pe precisa, alem do caminho do
                // clipe, na ida e na volta: no sprint o pe saia do chute de
                // calcanhar e chegava na bola em 0,1 s, a perna a 35 rad/s.)
                const float extra = std::max(deviation / std::max(0.03f, t - startTime),
                    deviation / std::max(0.03f, std::min(windowEnd, t + s.followMaximum) - t));
                cost += s.deviationRateWeight * std::max(0.0f, extra - s.deviationRateFree);
            }
            clipFootAt(in, side, startTime, startFoot);
        } else {
            cost += 0.5f * s.deviationWeight * (pose.foot - in.footNow[sd]).length();
        }
        if (retarget) {
            // (Ja no ar: o pe sai de onde esta agora.)
            startFoot = in.swingFoot;
            startTime = 0.0f;
            cost += s.retargetWeight * std::abs(t - committed);
        }
        const Vec3 startBody = in.body->toBody(startFoot, startTime);
        const Vec3 contactBody = in.body->toBody(pose.foot, t);
        if (!retarget && !step && t - startTime < s.minimumApproach) {
            ++rejected[3];
            return;
        }
        const float travelTime = std::max(0.03f, t - startTime);
        const float pace = (contactBody - startBody).length() / travelTime;
        const float limit = s.footSpeed + s.footSpeedPerBody * bodySpeed;
        if ((!step || retarget) && pace > s.paceLimitShare * limit) {
            ++rejected[4];
            return;
        }
        // A volta: do contato ao pouso (andando, o pe do clipe no fim da
        // janela; parado, perto de onde o passo saiu), no mesmo limite - sem
        // ela, um contato a 0,9 m do pe do clipe voltava em 0,06 s.
        {
            Vec3 landing = in.footNow[sd];
            float landingTime = t + (shot ? s.shotFollow : s.stepFollow);
            if (!step) {
                landingTime = std::min(windowEnd, t + s.followMaximum);
                clipFootAt(in, side, landingTime, landing);
            }
            const float back = (in.body->toBody(landing, landingTime) - contactBody).length()
                / std::max(0.03f, landingTime - t);
            if (back > s.paceLimitShare * limit) {
                ++rejected[4];
                return;
            }
            cost += 0.1f * std::max(0.0f, back - limit);
        }
        if (step) {
            // (Parado o passo comeca a tempo: o pe anda ate o contato. Refeito,
            // o resto do caminho no mesmo passo.)
            const float natural = std::max(retarget ? 0.0f : s.stepMinimum,
                (contactBody - startBody).length() / std::max(0.5f, s.stepSpeed));
            if (retarget) {
                // (Refeito, o tempo do passo ja foi decidido no compromisso: o
                // "resto natural" medido do pe que mal saiu empurrava o
                // contato para depois a cada tick e o chute nunca saia.)
            } else {
                if (t - natural < windowStart - 1e-4f) {
                    ++rejected[5];
                    return;
                }
                // (A menos de um passo da grade, o passo comeca ja: sem isso o
                // comeco ficava sempre um pouco a frente e nunca chegava.)
                if (t - natural - windowStart >= dt) windowStart = t - natural;
            }
        } else {
            cost += 0.1f * std::max(0.0f, pace - limit);
        }
        // O giro da perna: o rumo do pe do comeco do balanco ate o contato e
        // dele ate o pouso, em media.
        {
            const float contactYaw = athleteFootHeading3D(in.feet->geometry, side, pose.orientation);
            float startYaw = in.footYawNow[sd], endYaw = in.footYawNow[sd];
            float approach = std::max(0.03f, t - startTime);
            float follow = shot ? s.shotFollow : s.stepFollow;
            if (!step) {
                const float followEnd = std::min(windowEnd, t + s.followMaximum);
                clipFootYawAt(in, side, startTime, startYaw);
                clipFootYawAt(in, side, followEnd, endYaw);
                follow = followEnd - t;
            } else if (!retarget) {
                approach = std::max(0.03f, t - windowStart);
            }
            // (Refeito, do pe no ar agora: com o corpo virando depois do
            // compromisso - a camera -, o pe do contato girava com ele e a
            // perna dava meia volta, 200 rad/s.)
            if (retarget) {
                startYaw = in.swingFootYaw;
                approach = std::max(0.03f, t);
            }
            const float rate = std::max(std::abs(athleteWrapAngle3D(contactYaw - startYaw)) / approach,
                std::abs(athleteWrapAngle3D(contactYaw - endYaw)) / std::max(0.03f, follow));
            if (rate > s.legTurnMaximum) {
                ++rejected[8];
                return;
            }
            cost += s.legTurnWeight * std::max(0.0f, rate - s.legTurnFree);
        }
        const bool inside = dribbleInArea3D(ball, in.ballRadius, pelvis, velocity, s);
        if (!inside) cost += 10.0f;
        if (cost >= 10.0f) ++rejected[inside ? 6 : 7];
        if (cost >= best.cost) return;
        best.valid = true;
        best.side = side;
        best.step = step;
        best.start = in.now + windowStart;
        best.contact = in.now + t;
        best.end = in.now + (step ? t + (shot ? s.shotFollow : s.stepFollow) : windowEnd);
        best.followEnd = step ? best.end : in.now + std::min(windowEnd, t + s.followMaximum);
        best.surface = surface;
        best.style = style;
        best.push = push;
        best.ball = ball;
        best.pose = pose;
        best.drop = drop;
        best.inArea = inside;
        best.power = request.power;
        best.shot = shot;
        best.cost = cost;
    };
    const float earliest = std::max(0.0f, in.earliest - in.now);
    for (int side = 0; side < 2; ++side) {
        if (in.onlySide >= 0 && side != in.onlySide) continue;
        const std::size_t sd = static_cast<std::size_t>(side);
        if (in.cycling && in.gait != nullptr && in.gait->valid) {
            for (const DribbleWindow3D& window : in.gait->windows[sd]) {
                if (window.end <= 0.0f || window.end <= window.start) continue;
                if (in.onlyWindowEnd >= 0.0f) {
                    // (O toque ja comprometido: so a janela dele.)
                    if (std::abs(in.now + window.end - in.onlyWindowEnd) > 0.05f) continue;
                    const float span = window.end - window.start;
                    float from = std::max({ earliest, window.start + s.swingEarliest * span, dt });
                    float to = std::min(window.end - s.followMinimum, s.horizon);
                    if (retarget) {
                        // (Adiantar o contato encurta a ida ja comecada: so
                        // um pouco.)
                        from = std::max({ from, committed - s.retargetEarlier, s.lockLead });
                        to = std::min(to, committed + s.retargetBand);
                    }
                    for (float t = onGrid(from); t <= to + 1e-4f; t += dt) evaluate(side, t, window.start, window.end, false);
                    continue;
                }
                if (in.now + window.end <= in.busyUntil[sd] + 1e-3f) continue;
                if (in.now + window.start < in.busyUntil[sd] - 1e-3f) continue;
                if (in.usedWindowEnd[sd] >= 0.0f && std::abs(in.now + window.end - in.usedWindowEnd[sd]) < 0.03f) continue;
                const float span = window.end - window.start;
                const float from = std::max({ earliest, window.start + s.swingEarliest * span, dt });
                const float to = std::min(window.end - s.followMinimum, s.horizon);
                for (float t = onGrid(from); t <= to + 1e-4f; t += dt) evaluate(side, t, window.start, window.end, false);
            }
        } else {
            // (Refeito, o passo ja comecou: o contato perto do comprometido.)
            const float free = retarget ? in.committedStart - in.now : std::max(0.0f, in.busyUntil[sd] - in.now);
            const float from = retarget ? std::max({ earliest, committed - s.retargetEarlier, s.lockLead, free + s.stepMinimum, dt })
                                        : std::max({ earliest, free + s.stepMinimum, dt });
            const float to = retarget ? std::min(s.horizon, committed + s.retargetBand) : s.horizon;
            for (float t = onGrid(from); t <= to + 1e-4f; t += dt) evaluate(side, t, free, 0.0f, true);
        }
    }
    best.candidates = candidates;
    best.rejected = rejected;
    return best;
}

bool touchPlanFeasible3D(const TouchPlanInput3D& in, const TouchPlan3D& plan, float paceShare) {
    if (!plan.valid || in.body == nullptr || in.settings == nullptr || !in.body->valid()) return false;
    const DribbleSettings3D& s = *in.settings;
    const float limit = paceShare * (s.footSpeed + s.footSpeedPerBody * planar(in.bodyVelocity).length());
    const float contact = plan.contact - in.now;
    const float end = plan.followEnd - in.now;
    const Vec3 contactBody = in.body->toBody(plan.pose.foot, std::max(0.0f, contact));
    // (A ida pela media do que falta: a quintica comeca devagar e a media
    // sobe ate quase o dobro no meio - so desiste muito alem do limite e longe
    // do contato.)
    if (contact > s.lockLead) {
        const float ahead = (contactBody - in.body->toBody(in.swingFoot, 0.0f)).length() / std::max(0.03f, contact);
        if (ahead > 1.5f * limit) {
            if (std::getenv("XDRIBBLE")) std::printf("  ida %.2f m/s limite %.2f\n", ahead, limit);
            return false;
        }
    }
    Vec3 landing = in.footNow[static_cast<std::size_t>(plan.side)];
    if (!plan.step) clipFootAt(in, plan.side, std::max(0.0f, end), landing);
    const float back = (in.body->toBody(landing, std::max(0.0f, end)) - contactBody).length()
        / std::max(0.03f, end - std::max(0.0f, contact));
    if (std::getenv("XDRIBBLE") && back > limit)
        std::printf("  volta %.2f m/s limite %.2f | pouso %.3f %.3f %.3f contato %.3f %.3f %.3f\n", back, limit, landing.x,
            landing.y, landing.z, plan.pose.foot.x, plan.pose.foot.y, plan.pose.foot.z);
    return back <= limit;
}

} // namespace MatterEngine
