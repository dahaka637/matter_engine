#include "Engine/Athlete/Planning/FootstepPlanner3D.hpp"

#include <algorithm>
#include <cmath>

namespace MatterEngine {
namespace {

float minimumJerk(float s) {
    s = std::clamp(s, 0.0f, 1.0f);
    return s * s * s * (10.0f - 15.0f * s + 6.0f * s * s);
}

float smooth(float s) {
    s = std::clamp(s, 0.0f, 1.0f);
    return s * s * (3.0f - 2.0f * s);
}

Quaternion nlerp(Quaternion a, Quaternion b, float t) {
    if (a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w < 0.0f) b = { -b.x, -b.y, -b.z, -b.w };
    return Quaternion { a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t, a.z + (b.z - a.z) * t, a.w + (b.w - a.w) * t }
        .normalized();
}

float planarDistance(Vec3 a, Vec3 b) { return std::hypot(a.x - b.x, a.y - b.y); }

Vec3 planar(Vec3 v) { return { v.x, v.y, 0.0f }; }

} // namespace

void FootstepPlanner3D::reset(const RagdollProfile3D& profile, const FootstepSettings3D& settings) {
    m_legs = resolveAthleteLegs3D(profile);
    m_geometry = resolveAthleteFootGeometry3D(profile, m_legs, settings.toeRollMeters);
    m_ready = true;
    m_feet = {};
    m_live = false;
    m_sinceLanding = 0.0f;
    m_groundedTime = 10.0f;
    m_previousActing = -1;
    m_telemetry = {};
}

float FootstepPlanner3D::lowestSole(int side, Vec3 position, Quaternion orientation) const {
    const std::size_t s = static_cast<std::size_t>(side);
    return std::min((position + orientation.rotate(m_geometry.heel[s])).z,
        (position + orientation.rotate(m_geometry.toeBase[s])).z);
}

float FootstepPlanner3D::reach(int side, const FootstepStyle3D& style) const {
    const std::size_t s = static_cast<std::size_t>(side);
    const Foot& foot = m_feet[s];
    const Vec3 hip = style.rootPosition + style.rootOrientation.rotate(m_legs.hipInPelvis[s]);
    const Vec3 ankle = foot.position + foot.orientation.rotate(m_legs.ankleInFoot[s]);
    return (ankle - hip).length() / std::max(0.1f, m_legs.thighLength[s] + m_legs.shinLength[s]);
}

void FootstepPlanner3D::seed(const std::array<Vec3, 2>& position, const std::array<Quaternion, 2>& orientation,
    const std::array<bool, 2>& grounded) {
    for (int side = 0; side < 2; ++side) {
        Foot& foot = m_feet[static_cast<std::size_t>(side)];
        foot = {};
        foot.position = position[static_cast<std::size_t>(side)];
        foot.orientation = orientation[static_cast<std::size_t>(side)].normalized();
        if (grounded[static_cast<std::size_t>(side)]) {
            lock(side, foot.position, foot.orientation);
        } else {
            // (Livre: a diferenca para o estilo e medida no primeiro plano.)
            foot.mode = FootMode3D::Free;
            foot.progress = -1.0f;
        }
    }
    m_live = true;
    m_sinceLanding = 0.0f;
    m_groundedTime = 10.0f;
}

void FootstepPlanner3D::lock(int side, Vec3 position, Quaternion orientation) {
    Foot& foot = m_feet[static_cast<std::size_t>(side)];
    foot.mode = FootMode3D::Stance;
    // (O desvio da passada de apoio e a correcao do espaco ja estao no lugar
    // travado.)
    foot.landingShift.reset({});
    foot.treadShift.reset({});
    foot.landTotal = 0.0f;
    foot.spacing = {};
    foot.quick = false;
    foot.pivoting = false;
    foot.position = position;
    foot.orientation = orientation;
    foot.yaw = athleteFootYaw3D(m_geometry, side, orientation);
    const Vec3 heel = position + orientation.rotate(m_geometry.heel[static_cast<std::size_t>(side)]);
    const Vec3 toe = position + orientation.rotate(m_geometry.toeBase[static_cast<std::size_t>(side)]);
    // Toca o mais baixo dos dois.
    foot.toePivot = toe.z <= heel.z;
    foot.pivot = foot.toePivot ? toe : heel;
    m_sinceLanding = 0.0f;
}

void FootstepPlanner3D::startOffset(int side, FootMode3D mode, const FootstepStyle3D& style) {
    Foot& foot = m_feet[static_cast<std::size_t>(side)];
    const std::size_t s = static_cast<std::size_t>(side);
    // (A diferenca para o estilo de antes deste tick: o pe ainda nao andou
    // neste tick e o estilo ja - medindo do estilo de agora, o primeiro tick
    // perdia o passo do estilo e o pe ficava para tras de uma vez.)
    foot.offset = foot.position - (foot.haveHistory ? foot.stylePrevious : style.position[s]);
    foot.liftReach = reach(side, style);
    foot.offsetYaw = athleteWrapAngle3D(athleteFootYaw3D(m_geometry, side, foot.orientation)
        - athleteFootYaw3D(m_geometry, side, style.orientation[s]));
    foot.progress = 0.0f;
    foot.elapsed = 0.0f;
    foot.styleSwung = !style.planted[s];
    foot.liftVelocity = foot.haveHistory ? foot.velocity - foot.styleVelocity : Vec3 {};
    foot.quick = false;
    foot.treadShift.reset({});
    foot.landTotal = 0.0f;
    foot.mode = mode;
    if (mode == FootMode3D::Swing) {
        const float correction = std::hypot(foot.offset.x, foot.offset.y);
        m_telemetry.liftCorrection[s] = correction;
        m_telemetry.worstLiftCorrection = std::max(m_telemetry.worstLiftCorrection, correction);
    }
}

void FootstepPlanner3D::startFree(int side, const FootstepStyle3D& style) {
    startOffset(side, FootMode3D::Free, style);
    {
        // (Livre parte de onde o pe esta, contra o estilo de agora: no ar o
        // estilo salta - o clipe do voo - e o pe saltava junto.)
        Foot& foot = m_feet[static_cast<std::size_t>(side)];
        foot.offset = foot.position - style.position[static_cast<std::size_t>(side)];
    }
    // (O gesto da bola parte do pe daqui, com a velocidade dele: o estilo ja
    // e ele, sem mistura - a diferenca de um tick segurava o pe parado.)
    if (side == style.actingSide) {
        Foot& foot = m_feet[static_cast<std::size_t>(side)];
        foot.offset = {};
        foot.offsetYaw = 0.0f;
        foot.progress = 1.0f;
    }
}

void FootstepPlanner3D::startStep(int side, float seconds, float height) {
    Foot& foot = m_feet[static_cast<std::size_t>(side)];
    foot.mode = FootMode3D::Step;
    foot.stepTime = 0.0f;
    foot.stepSeconds = std::max(0.05f, seconds);
    foot.stepHeight = height;
    foot.stepFrom = foot.position;
    foot.stepFromOrientation = foot.orientation;
}

void FootstepPlanner3D::startLand(int side, Vec3 to, Quaternion orientation, float seconds, Vec3 styleNow) {
    Foot& foot = m_feet[static_cast<std::size_t>(side)];
    // (A correcao do espaco entra no comeco da descida: dali o pe vai reto.)
    foot.position = foot.position + foot.spacing;
    foot.spacing = {};
    foot.mode = FootMode3D::Land;
    foot.stepTime = 0.0f;
    foot.stepSeconds = std::max(0.02f, seconds);
    foot.stepFrom = foot.position;
    foot.stepFromOrientation = foot.orientation;
    foot.landTo = to;
    foot.landOrientation = orientation;
    foot.landShift = planar(to - styleNow);
    ++m_telemetry.lands;
}

FootSole3D FootstepPlanner3D::soleOf(int side, Vec3 position, Quaternion orientation,
    const FootstepSettings3D& settings) const {
    // A sola: do calcanhar a ponta dos dedos.
    const std::size_t s = static_cast<std::size_t>(side);
    const Vec3 heel = position + orientation.rotate(m_geometry.heel[s]);
    const Vec3 toeBase = position + orientation.rotate(m_geometry.toeBase[s]);
    Vec3 along = toeBase - heel;
    along = along.lengthSquared() > 1e-8f ? along.normalized() : Vec3 {};
    FootSole3D sole { heel, toeBase + along * settings.toeRollMeters };
    sole.ankle = position + orientation.rotate(m_legs.ankleInFoot[s]);
    sole.hip = m_hip[s];
    sole.leg = m_hipKnown;
    return sole;
}

FootstepPlanner3D::Tread FootstepPlanner3D::treadUnder(int side, Vec3 position, Quaternion orientation,
    const FootstepStyle3D& style, const FootstepSettings3D& settings) const {
    Tread tread;
    if (!style.groundAt) return tread;
    // (A sola inteira: o calcanhar, a base dos dedos e a ponta - so ate a
    // base, a ponta entrava no espelho do degrau seguinte.)
    const FootSole3D sole = soleOf(side, position, orientation, settings);
    const Vec3 points[3] = { sole.heel, position + orientation.rotate(m_geometry.toeBase[static_cast<std::size_t>(side)]),
        sole.toe };
    float lowest = 1e9f, highest = -1e9f;
    int known = 0;
    for (const Vec3& point : points) {
        const FootGround3D ground = style.groundAt(point);
        if (!ground.known) continue;
        ++known;
        lowest = std::min(lowest, ground.height);
        highest = std::max(highest, ground.height);
    }
    if (known == 0) return tread;
    tread.known = true;
    tread.height = highest;
    tread.level = known == 3 && highest - lowest <= settings.treadTolerance;
    tread.edge = known < 3 || highest - lowest > settings.edgeHeight;
    return tread;
}

bool FootstepPlanner3D::plant(int side, const FootstepStyle3D& style, const FootstepSettings3D& settings, bool quick) {
    Foot& foot = m_feet[static_cast<std::size_t>(side)];
    const std::size_t s = static_cast<std::size_t>(side);
    const Vec3 base = foot.position + foot.spacing;
    const Quaternion orientation = foot.orientation;
    // (Sem o chao de verdade, o do estilo - os bonecos sem a sonda.)
    const auto styleRule = [&]() {
        if (m_styleGround < 1e8f && lowestSole(side, base, orientation) >= m_styleGround + settings.landingHeight)
            return false;
        lock(side, base, orientation);
        return true;
    };
    if (!style.groundAt) return styleRule();
    // No degrau: onde a sola inteira cabe numa altura so, o mais perto
    // possivel (ao longo do pe), nunca numa quina.
    Vec3 along = planar(orientation.rotate(m_geometry.toeBase[s] - m_geometry.heel[s]));
    along = along.lengthSquared() > 1e-8f ? along.normalized() : Vec3 { 1.0f, 0.0f, 0.0f };
    Vec3 target = base;
    Tread tread = treadUnder(side, base, orientation, style, settings);
    if (tread.known && !tread.level) {
        for (float shift = 0.02f; shift <= settings.treadSearch + 1e-4f; shift += 0.02f) {
            bool found = false;
            for (const float sign : { 1.0f, -1.0f }) {
                const Vec3 candidate = base + along * (shift * sign);
                const Tread t = treadUnder(side, candidate, orientation, style, settings);
                if (t.known && t.level) {
                    target = candidate;
                    tread = t;
                    found = true;
                    break;
                }
            }
            if (found) break;
        }
    }
    if (!tread.known) return styleRule();
    // Numa borda alta (plataforma, degrau) sem lugar plano perto: nada de
    // apoiar na quina com o pe para fora - segue o estilo e tenta de novo.
    if (!tread.level && tread.edge) return false;
    {
        // O lugar do pouso respeita o espaco entre os pes por inteiro (travado,
        // ele nao se mexe mais): do lado dele e longe da sola do outro.
        const Foot& other = m_feet[static_cast<std::size_t>(1 - side)];
        const Vec3 forward { std::cos(style.facingYaw), std::sin(style.facingYaw), 0.0f };
        const Vec3 push = footSpacingPush3D(soleOf(side, target, orientation, settings),
            soleOf(1 - side, other.position + other.spacing, other.orientation, settings), side, forward, foot.crossed,
            settings.spacing);
        if (push.lengthSquared() > 1e-8f) {
            target = target + planar(push);
            const Tread moved = treadUnder(side, target, orientation, style, settings);
            if (moved.known) tread = moved;
        }
    }
    const float gap = lowestSole(side, target, orientation) - tread.height;
    // Longe acima do chao (a borda de uma plataforma, o fim de uma queda):
    // nada trava, o pe segue o estilo.
    if (gap > settings.landMaxGap || gap < -0.25f) return false;
    target.z -= gap;
    const float travel = (target - base).length();
    if (!quick && travel <= settings.lockSnap) {
        lock(side, target, orientation);
        return true;
    }
    const float seconds = quick ? settings.quickPlantSeconds
        : std::clamp(travel / std::max(0.1f, settings.landSpeed), settings.landMinimumSeconds, settings.landMaximumSeconds);
    startLand(side, target, orientation, seconds, style.position[s]);
    foot.quick = quick;
    foot.landFollow = !quick && gap > settings.landFollowHeight;
    if (quick) ++m_telemetry.quickPlants;
    return true;
}

FootTarget3D FootstepPlanner3D::stance(int side, const FootstepStyle3D& style, const FootstepSettings3D& settings,
    float deltaTime) {
    Foot& foot = m_feet[static_cast<std::size_t>(side)];
    const std::size_t s = static_cast<std::size_t>(side);
    // A inclinacao do estilo (o calcanhar subindo no fim, os dedos no
    // comeco) no rumo travado.
    const float styleYaw = athleteFootYaw3D(m_geometry, side, style.orientation[s]);
    {
        // O giro brusco: o estilo quer o pe virado muito alem do travado - ele
        // gira no ponto que toca o chao (a ponta ou o calcanhar), sem torcer a
        // perna.
        const float difference = athleteWrapAngle3D(styleYaw - foot.yaw);
        const bool turn = std::abs(difference) > settings.pivotStartRadians
            || (foot.pivoting && std::abs(difference) > settings.pivotStopRadians);
        if (turn && !foot.pivoting) ++m_telemetry.pivots;
        foot.pivoting = turn;
        if (turn) {
            const float most = settings.pivotRate * std::max(deltaTime, 1e-4f);
            foot.yaw = athleteWrapAngle3D(foot.yaw + std::clamp(difference, -most, most));
        }
    }
    const Quaternion orientation = (athleteYawRotation3D(athleteWrapAngle3D(foot.yaw - styleYaw))
        * style.orientation[s]).normalized();
    const Vec3 heel = orientation.rotate(m_geometry.heel[s]);
    const Vec3 toe = orientation.rotate(m_geometry.toeBase[s]);
    bool toePivot = foot.toePivot;
    if (heel.z > toe.z + settings.pivotSwitchHeight) toePivot = true;
    else if (toe.z > heel.z + settings.pivotSwitchHeight) toePivot = false;
    if (toePivot != foot.toePivot) {
        // (A troca acontece com o pe quase plano: o ponto novo fica onde esta
        // agora, na altura do chao do apoio.)
        const Vec3 now = foot.position + foot.orientation.rotate(toePivot ? m_geometry.toeBase[s] : m_geometry.heel[s]);
        foot.pivot = { now.x, now.y, foot.pivot.z };
        foot.toePivot = toePivot;
    }
    foot.orientation = orientation;
    foot.position = foot.pivot - (toePivot ? toe : heel);
    return { FootMode3D::Stance, foot.position, foot.orientation };
}

std::array<FootTarget3D, 2> FootstepPlanner3D::plan(const FootstepStyle3D& style, const FootstepSettings3D& settings,
    float deltaTime) {
    std::array<FootTarget3D, 2> out;
    for (std::size_t side = 0; side < 2; ++side) out[side] = { FootMode3D::Free, style.position[side], style.orientation[side] };
    if (!m_geometry.valid) return out;
    const float dt = std::max(deltaTime, 1e-4f);
    if (!m_live) {
        // Do estilo (o corpo guiado esta nele): os apoiados travam onde estao.
        const bool ground = style.grounded && !style.scripted;
        seed(style.position, style.orientation,
            { ground && style.planted[0] && style.actingSide != 0, ground && style.planted[1] && style.actingSide != 1 });
    }
    m_sinceLanding += dt;
    // O corpo no chao ha quanto tempo: depois do ar, nada trava antes de
    // groundedSettleSeconds (a capsula que raspa uma borda num tick).
    m_groundedTime = style.grounded && !style.scripted ? m_groundedTime + dt : 0.0f;
    const bool settled = m_groundedTime >= settings.groundedSettleSeconds;
    // A altura do chao pelo estilo (so sem o chao de verdade): a sola mais
    // baixa dos pes que ele diz apoiados.
    m_styleGround = 1e9f;
    for (int side = 0; side < 2; ++side)
        if (style.planted[static_cast<std::size_t>(side)])
            m_styleGround = std::min(m_styleGround, lowestSole(side, style.position[static_cast<std::size_t>(side)],
                style.orientation[static_cast<std::size_t>(side)]));
    const bool moving = style.movingShare >= settings.settleMovingShare;
    for (std::size_t side = 0; side < 2; ++side) {
        Foot& foot = m_feet[side];
        if (foot.haveHistory) foot.styleVelocity = (style.position[side] - foot.stylePrevious) * (1.0f / dt);
        // (Os quadris do estilo: a canela de cada pe para o espaco entre eles.)
        m_hip[side] = style.rootPosition + style.rootOrientation.rotate(m_legs.hipInPelvis[side]);
    }
    m_hipKnown = m_legs.valid;
    std::array<Vec3, 2> before { m_feet[0].position + m_feet[0].spacing, m_feet[1].position + m_feet[1].spacing };

    // O pe que o gesto soltou: com o estilo no balanco, entra no balanco dele
    // (preso no chao, a perna esticava alem do alcance e o pe era arrastado -
    // plano v2); com o estilo apoiado, vai ao chao de verdade embaixo dele e
    // espera o estilo pousar o dele; longe do chao, a mistura curta ate o
    // estilo.
    if (m_previousActing >= 0 && style.actingSide != m_previousActing) {
        const std::size_t released = static_cast<std::size_t>(m_previousActing);
        Foot& foot = m_feet[released];
        if (foot.mode == FootMode3D::Free) {
            if (!style.planted[released] && style.grounded && !style.scripted) {
                startOffset(m_previousActing, FootMode3D::Swing, style);
            } else if (style.grounded && !style.scripted && settled && plant(m_previousActing, style, settings)) {
                foot.releaseWait = settings.releaseWaitSeconds;
            } else {
                startOffset(m_previousActing, FootMode3D::Free, style);
            }
        }
    }
    m_previousActing = style.actingSide;

    // As trocas de modo pelo que o estilo diz (e pelo alcance da perna).
    for (int side = 0; side < 2; ++side) {
        Foot& foot = m_feet[static_cast<std::size_t>(side)];
        const std::size_t s = static_cast<std::size_t>(side);
        const bool free = style.scripted || !style.grounded || side == style.actingSide;
        const bool reserved = side == style.reservedSide;
        // O pouso rapido do gesto: o pe no ar vai ao chao ja.
        if (side == style.quickPlantSide && !free && settled
            && (foot.mode == FootMode3D::Swing || foot.mode == FootMode3D::Free || foot.mode == FootMode3D::Step))
            plant(side, style, settings, true);
        switch (foot.mode) {
        case FootMode3D::Stance:
            if (style.planted[s]) foot.releaseWait = 0.0f;
            if (free) {
                startFree(side, style);
            } else if (reserved) {
                // (O apoio do gesto: fica.)
            } else if (foot.releaseWait > 0.0f) {
                // (Pousado pelo gesto antes do estilo: espera o dele.)
                foot.releaseWait = std::max(0.0f, foot.releaseWait - dt);
            } else if (!style.planted[s]) {
                startOffset(side, FootMode3D::Swing, style);
            } else if (moving && style.secondsToLiftoff[s] >= 0.0f && reach(side, style) > settings.reachLimit) {
                // A perna no fim do alcance: o pe sai antes do estilo.
                startOffset(side, FootMode3D::Swing, style);
                ++m_telemetry.earlyLifts;
            } else if (moving && planarDistance(foot.position, style.position[s]) > settings.driftRelease) {
                // O estilo levou o pe para longe do lugar travado: o passo ate
                // ele.
                startOffset(side, FootMode3D::Swing, style);
                ++m_telemetry.earlyLifts;
            }
            break;
        case FootMode3D::Land:
            if (free) {
                startFree(side, style);
            } else if (!reserved && !foot.quick
                && (!style.planted[s] || (moving && reach(side, style) > settings.reachLimit))) {
                // (O estilo ja levanta o pe, ou a perna chegou ao fim do alcance
                // descendo - o corpo correndo vai embora: o pe sai.)
                startOffset(side, FootMode3D::Swing, style);
            }
            break;
        case FootMode3D::Step:
            if (free) startFree(side, style);
            else if (!style.planted[s]) startOffset(side, FootMode3D::Swing, style);
            break;
        case FootMode3D::Swing:
            if (free) startFree(side, style);
            break;
        case FootMode3D::Free:
            if (foot.progress < 0.0f) startOffset(side, FootMode3D::Free, style);
            break;
        }
    }

    // Sem o ciclo do estilo (parado, ou arrancando antes dele comecar), com
    // os dois pes apoiados: o pe mais longe do lugar do estilo da um passo.
    // Parado, a acomodacao; com a capsula andando, o passo rapido da
    // arrancada.
    const bool cycling = style.secondsToLiftoff[0] >= 0.0f || style.secondsToLiftoff[1] >= 0.0f
        || style.secondsToTouchdown[0] >= 0.0f || style.secondsToTouchdown[1] >= 0.0f;
    const float speed = std::hypot(style.velocity.x, style.velocity.y);
    const bool starting = !cycling && speed > settings.startSpeed;
    if (!cycling && (settings.settleSteps || starting) && style.grounded && !style.scripted && style.actingSide < 0
        && style.reservedSide < 0 && (starting || m_sinceLanding > settings.settleDelaySeconds)
        && m_feet[0].mode == FootMode3D::Stance && m_feet[1].mode == FootMode3D::Stance && style.planted[0]
        && style.planted[1]) {
        int worst = -1;
        float worstScore = 1.0f;
        for (int side = 0; side < 2; ++side) {
            const Foot& foot = m_feet[static_cast<std::size_t>(side)];
            const std::size_t s = static_cast<std::size_t>(side);
            const float distance = planarDistance(foot.position, style.position[s]);
            const float yaw = std::abs(athleteWrapAngle3D(foot.yaw - athleteFootYaw3D(m_geometry, side, style.orientation[s])));
            const float score = starting ? distance / std::max(1e-3f, settings.startLag)
                : std::max(distance / std::max(1e-3f, settings.settleDistance),
                    yaw / std::max(1e-3f, settings.settleYawRadians));
            if (score > worstScore) {
                worstScore = score;
                worst = side;
            }
        }
        if (worst >= 0) {
            if (starting) {
                startStep(worst, settings.startSeconds, settings.startHeight);
                ++m_telemetry.startSteps;
            } else {
                startStep(worst, settings.settleSeconds, settings.settleHeight);
                ++m_telemetry.settleSteps;
            }
        }
    }

    for (int side = 0; side < 2; ++side) {
        Foot& foot = m_feet[static_cast<std::size_t>(side)];
        const std::size_t s = static_cast<std::size_t>(side);
        const bool free = style.scripted || !style.grounded || side == style.actingSide;
        switch (foot.mode) {
        case FootMode3D::Stance:
            out[s] = stance(side, style, settings, dt);
            break;
        case FootMode3D::Swing: {
            // Ate o pouso do estilo, pelo tempo: a diferenca da saida some no
            // balanco todo, sem salto no comeco.
            foot.elapsed += dt;
            if (!style.planted[s]) foot.styleSwung = true;
            float remaining = -1.0f;
            if (!style.planted[s]) remaining = style.secondsToTouchdown[s];
            else if (!foot.styleSwung && style.secondsToLiftoff[s] >= 0.0f && style.swingSeconds[s] > 0.0f)
                remaining = style.secondsToLiftoff[s] + style.swingSeconds[s];
            {
                // (O tempo do estilo salta; o balanco nao.)
                const float most = foot.progress + dt / std::max(0.02f, settings.minimumSwingSeconds);
                if (remaining >= 0.0f)
                    foot.progress = std::clamp(foot.elapsed / std::max(1e-3f, foot.elapsed + remaining), foot.progress, most);
                else if (style.planted[s] && foot.styleSwung)
                    foot.progress = std::min(1.0f, most);
            }
            const float keep = 1.0f - minimumJerk(foot.progress);
            {
                // A forma do estilo, com a diferenca da saida sumindo; perto
                // do pouso, o pe desacelera (no plano, |v| ate
                // landingDeceleration vezes o que falta): o clipe pousa com
                // o pe andando - ate 10 m/s nas trocas de direcao.
                Vec3 wanted = style.position[s] + foot.offset * keep;
                {
                    // A passada de apoio: o desvio do pouso entra ate ele.
                    const Vec3 shift = side == style.landingSide ? style.landingShift : Vec3 {};
                    const float until = remaining >= 0.0f ? remaining : settings.minimumSwingSeconds;
                    foot.landingShift.step(shift, std::max(0.06f, 0.8f * until), dt);
                    wanted += foot.landingShift.value();
                }
                {
                    // O degrau: perto do pouso, o pe ja vai para onde a sola
                    // inteira cabe numa altura so (no pouso ele nao precisa
                    // mais andar).
                    Vec3 shift;
                    if (style.groundAt && remaining >= 0.0f && remaining < settings.treadLookahead) {
                        Vec3 along = planar(foot.orientation.rotate(m_geometry.toeBase[s] - m_geometry.heel[s]));
                        along = along.lengthSquared() > 1e-8f ? along.normalized() : Vec3 {};
                        const Tread here = treadUnder(side, wanted, foot.orientation, style, settings);
                        if (here.known && !here.level) {
                            for (float d = 0.02f; d <= settings.treadSearch + 1e-4f; d += 0.02f) {
                                bool found = false;
                                for (const float sign : { 1.0f, -1.0f }) {
                                    const Tread t = treadUnder(side, wanted + along * (d * sign), foot.orientation, style,
                                        settings);
                                    if (t.known && t.level) {
                                        shift = along * (d * sign);
                                        found = true;
                                        break;
                                    }
                                }
                                if (found) break;
                            }
                        }
                    }
                    foot.treadShift.step(shift, settings.treadShiftSeconds, dt);
                    wanted += foot.treadShift.value();
                }
                if (foot.elapsed < settings.liftBlendSeconds) {
                    // (Hermite: comeca com a velocidade do pe e some parado.)
                    const float t = foot.elapsed;
                    const float fall = 1.0f - t / settings.liftBlendSeconds;
                    wanted += foot.liftVelocity * (t * fall * fall);
                }
                Vec3 step { wanted.x - foot.position.x, wanted.y - foot.position.y, 0.0f };
                const float left = remaining >= 0.0f ? remaining
                    : std::max(0.0f, (1.0f - foot.progress) * settings.minimumSwingSeconds);
                const float most = std::max(settings.landingMinimumSpeed, settings.landingDeceleration * left) * dt;
                if (foot.styleSwung && step.length() > most) step = step * (most / step.length());
                foot.position = { foot.position.x + step.x, foot.position.y + step.y, wanted.z };
            }
            foot.orientation = (athleteYawRotation3D(foot.offsetYaw * keep) * style.orientation[s]).normalized();
            if (settings.swingSoftReach) {
                // O alcance macio na perna que ficou para tras (no rumo do
                // movimento): o tornozelo vem para o quadril, numa curva sem
                // quina, o que passar do alcance. Na frente do quadril (o
                // pouso) nada muda.
                const Vec3 hip = style.rootPosition + style.rootOrientation.rotate(m_legs.hipInPelvis[s]);
                const float length = std::max(0.1f, m_legs.thighLength[s] + m_legs.shinLength[s]);
                const Vec3 ankle = foot.position + foot.orientation.rotate(m_legs.ankleInFoot[s]);
                const float distance = (ankle - hip).length();
                const float end = settings.swingSoftReachEnd;
                const float start = std::min(std::max(settings.swingSoftReachStart, foot.liftReach), end - 0.004f);
                const float soft = athleteSoftReach3D(distance / length, start, end) * length;
                if (distance > soft + 1e-5f) {
                    Vec3 travel { style.velocity.x, style.velocity.y, 0.0f };
                    if (travel.lengthSquared() < 0.25f) travel = { std::cos(style.facingYaw), std::sin(style.facingYaw), 0.0f };
                    const float behind = -dot(ankle - hip, travel.normalized());
                    const float weight = smooth(behind / 0.15f);
                    foot.position = foot.position + (ankle - hip) * ((soft / distance - 1.0f) * weight);
                }
            }
            out[s] = { FootMode3D::Swing, foot.position, foot.orientation };
            if (style.planted[s] && (foot.styleSwung || remaining < 0.0f) && (foot.progress >= 1.0f || !foot.styleSwung)) {
                // O pouso do estilo (ou ele parou de andar com o pe no ar): ao
                // chao de verdade embaixo dele; longe do chao (ou o corpo
                // recem-chegado do ar), segue o estilo ate poder.
                if (!(settled && plant(side, style, settings))) startOffset(side, FootMode3D::Free, style);
                out[s] = { foot.mode, foot.position, foot.orientation };
            }
            break;
        }
        case FootMode3D::Step: {
            // Para onde o estilo pora o pe no fim do passo (parado no estilo,
            // ele anda com a capsula).
            foot.stepTime += dt;
            const float t = std::clamp(foot.stepTime / foot.stepSeconds, 0.0f, 1.0f);
            const float u = minimumJerk(t);
            const float remaining = std::max(0.0f, foot.stepSeconds - foot.stepTime);
            const Vec3 to = style.position[s] + Vec3 { style.velocity.x, style.velocity.y, 0.0f } * remaining;
            foot.position = foot.stepFrom + (to - foot.stepFrom) * u
                + Vec3 { 0.0f, 0.0f, foot.stepHeight * std::sin(3.14159265f * t) };
            foot.orientation = nlerp(foot.stepFromOrientation, style.orientation[s], u);
            out[s] = { FootMode3D::Step, foot.position, foot.orientation };
            if (t >= 1.0f) {
                if (!plant(side, style, settings)) startOffset(side, FootMode3D::Free, style);
                out[s] = { foot.mode, foot.position, foot.orientation };
            }
            break;
        }
        case FootMode3D::Land: {
            // A descida ate o chao de verdade (o pouso, o degrau, o pouso
            // rapido do gesto): reta, sem salto, e trava.
            foot.stepTime += dt;
            foot.landTotal += dt;
            if (!foot.quick && foot.landTotal > settings.landGiveUpSeconds) {
                // (Sem lugar para apoiar a tempo: segue o estilo.)
                startFree(side, style);
                out[s] = { FootMode3D::Free, foot.position, foot.orientation };
                break;
            }
            const float t = std::clamp(foot.stepTime / foot.stepSeconds, 0.0f, 1.0f);
            const float u = minimumJerk(t);
            if (foot.landFollow) {
                // (No plano com o estilo, ate a velocidade maxima; a altura, a
                // do chao escolhido.)
                const Vec3 follow = style.position[s] + foot.landShift;
                Vec3 move { follow.x - foot.landTo.x, follow.y - foot.landTo.y, 0.0f };
                const float most = settings.landFollowSpeed * dt;
                if (move.length() > most) move = move * (most / move.length());
                foot.landTo = foot.landTo + move;
            }
            foot.position = foot.stepFrom + (foot.landTo - foot.stepFrom) * u;
            foot.orientation = nlerp(foot.stepFromOrientation, foot.landOrientation, u);
            out[s] = { FootMode3D::Land, foot.position, foot.orientation };
            if (t >= 1.0f) {
                // (Trava com a correcao do espaco que a descida ja tinha.)
                const bool quick = foot.quick;
                const Vec3 at = foot.landTo + planar(foot.spacing);
                // Acompanhando o estilo, o fim da descida pode ter saido do chao
                // escolhido (a borda de uma plataforma, o degrau seguinte):
                // ali, o pouso de novo (ou o estilo, longe do chao).
                const Tread there = quick ? Tread {} : treadUnder(side, at, foot.landOrientation, style, settings);
                const bool moved = there.known
                    && (!there.level || std::abs(lowestSole(side, at, foot.landOrientation) - there.height) > settings.lockHeight);
                if (moved) {
                    foot.position = at;
                    foot.spacing = {};
                    foot.orientation = foot.landOrientation;
                    if (!plant(side, style, settings)) startOffset(side, FootMode3D::Free, style);
                } else {
                    lock(side, at, foot.landOrientation);
                    // (O pouso rapido espera o estilo pousar o dele.)
                    if (quick) foot.releaseWait = settings.releaseWaitSeconds;
                }
                out[s] = { foot.mode, foot.position, foot.orientation };
            }
            break;
        }
        case FootMode3D::Free: {
            foot.progress = std::min(1.0f, foot.progress + dt / std::max(0.01f, settings.freeBlendSeconds));
            const float keep = 1.0f - smooth(foot.progress);
            foot.position = style.position[s] + foot.offset * keep;
            foot.orientation = (athleteYawRotation3D(foot.offsetYaw * keep) * style.orientation[s]).normalized();
            out[s] = { FootMode3D::Free, foot.position, foot.orientation };
            if (!free && style.planted[s] && foot.progress >= 1.0f && settled && plant(side, style, settings)) {
                // De volta ao chao (o pouso do pulo, o fim do gesto): no chao
                // de verdade embaixo dele.
                out[s] = { foot.mode, foot.position, foot.orientation };
            }
            break;
        }
        }
    }

    // O espaco entre os pes: o que anda fica do seu lado e contorna o outro
    // (no rumo do corpo). Os dois andando (o voo da corrida), cada um faz a
    // metade; o que pousa leva a correcao inteira no pouso (plant).
    {
        const Vec3 forward { std::cos(style.facingYaw), std::sin(style.facingYaw), 0.0f };
        const bool grounded = style.grounded && !style.scripted;
        std::array<Vec3, 2> wanted {};
        const auto sole = [&](int side, Vec3 position, Quaternion orientation) {
            return soleOf(side, position, orientation, settings);
        };
        const auto movingFoot = [&](int side) {
            const FootMode3D mode = m_feet[static_cast<std::size_t>(side)].mode;
            return mode != FootMode3D::Stance && side != style.actingSide;
        };
        if (grounded) {
            for (int side = 0; side < 2; ++side) {
                if (!movingFoot(side)) continue;
                const std::size_t s = static_cast<std::size_t>(side);
                // (A descida ja vai ao lugar corrigido: nela, a regra so
                // contra o que mudou depois.)
                Vec3 push = footSpacingPush3D(sole(side, out[s].position, out[s].orientation),
                    sole(1 - side, out[1 - s].position, out[1 - s].orientation), side, forward, m_feet[s].crossed,
                    settings.spacing);
                // (O que desce ao chao ja esta no lugar dele: conta como
                // apoiado.)
                if (movingFoot(1 - side) && m_feet[1 - s].mode != FootMode3D::Land) push = push * 0.5f;
                wanted[s] = push;
            }
        }
        for (std::size_t side = 0; side < 2; ++side) {
            Foot& foot = m_feet[side];
            if (foot.mode == FootMode3D::Stance) continue;
            foot.spacing = footSpacingStep3D(foot.spacing, wanted[side], dt, settings.spacing);
            out[side].position = out[side].position + foot.spacing;
            m_telemetry.worstSpacingPush = std::max(m_telemetry.worstSpacingPush, foot.spacing.length());
        }
        for (int side = 0; side < 2; ++side) {
            const std::size_t s = static_cast<std::size_t>(side);
            m_feet[s].crossed = footSpacingCrossed3D(sole(side, out[s].position, out[s].orientation),
                sole(1 - side, out[1 - s].position, out[1 - s].orientation), side, forward, settings.spacing);
        }
    }

    for (std::size_t side = 0; side < 2; ++side) {
        Foot& foot = m_feet[side];
        foot.velocity = foot.haveHistory ? (out[side].position - before[side]) * (1.0f / dt) : Vec3 {};
        foot.stylePrevious = style.position[side];
        foot.haveHistory = true;
    }
    return out;
}

} // namespace MatterEngine
