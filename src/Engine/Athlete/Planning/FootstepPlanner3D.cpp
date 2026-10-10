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

} // namespace

void FootstepPlanner3D::reset(const RagdollProfile3D& profile, const FootstepSettings3D& settings) {
    m_legs = resolveAthleteLegs3D(profile);
    m_geometry = resolveAthleteFootGeometry3D(profile, m_legs, settings.toeRollMeters);
    m_ready = true;
    m_feet = {};
    m_live = false;
    m_sinceLanding = 0.0f;
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
}

void FootstepPlanner3D::lock(int side, Vec3 position, Quaternion orientation) {
    Foot& foot = m_feet[static_cast<std::size_t>(side)];
    foot.mode = FootMode3D::Stance;
    // (O desvio da passada de apoio ja esta no lugar travado.)
    foot.landingShift.reset({});
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
    foot.offset = foot.position - style.position[s];
    foot.offsetYaw = athleteWrapAngle3D(athleteFootYaw3D(m_geometry, side, foot.orientation)
        - athleteFootYaw3D(m_geometry, side, style.orientation[s]));
    foot.progress = 0.0f;
    foot.elapsed = 0.0f;
    foot.styleSwung = !style.planted[s];
    foot.liftVelocity = foot.haveHistory ? foot.velocity - foot.styleVelocity : Vec3 {};
    foot.mode = mode;
    if (mode == FootMode3D::Swing) {
        const float correction = std::hypot(foot.offset.x, foot.offset.y);
        m_telemetry.liftCorrection[s] = correction;
        m_telemetry.worstLiftCorrection = std::max(m_telemetry.worstLiftCorrection, correction);
    }
}

void FootstepPlanner3D::startFree(int side, const FootstepStyle3D& style) {
    startOffset(side, FootMode3D::Free, style);
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

FootTarget3D FootstepPlanner3D::stance(int side, const FootstepStyle3D& style, const FootstepSettings3D& settings) {
    Foot& foot = m_feet[static_cast<std::size_t>(side)];
    const std::size_t s = static_cast<std::size_t>(side);
    // A inclinacao do estilo (o calcanhar subindo no fim, os dedos no
    // comeco) no rumo travado.
    const float styleYaw = athleteFootYaw3D(m_geometry, side, style.orientation[s]);
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
    // A altura do chao: a sola mais baixa dos pes que o estilo diz apoiados
    // (parando, ele diz os dois apoiados com um ainda descendo).
    float ground = 1e9f;
    for (int side = 0; side < 2; ++side)
        if (style.planted[static_cast<std::size_t>(side)])
            ground = std::min(ground, lowestSole(side, style.position[static_cast<std::size_t>(side)],
                style.orientation[static_cast<std::size_t>(side)]));
    const auto onGround = [&](int side, Vec3 position, Quaternion orientation) {
        return ground > 1e8f || lowestSole(side, position, orientation) < ground + settings.landingHeight;
    };
    const bool moving = style.movingShare >= settings.settleMovingShare;
    for (std::size_t side = 0; side < 2; ++side) {
        Foot& foot = m_feet[side];
        if (foot.haveHistory) foot.styleVelocity = (style.position[side] - foot.stylePrevious) * (1.0f / dt);
    }
    const std::array<Vec3, 2> before { m_feet[0].position, m_feet[1].position };

    // O pe que o gesto soltou: no chao, trava onde pousou (o estilo sem o
    // gesto o poria em outro lugar); no ar, a mistura curta ate o estilo.
    if (m_previousActing >= 0 && style.actingSide != m_previousActing) {
        Foot& foot = m_feet[static_cast<std::size_t>(m_previousActing)];
        if (foot.mode == FootMode3D::Free) {
            if (onGround(m_previousActing, foot.position, foot.orientation)) {
                lock(m_previousActing, foot.position, foot.orientation);
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
        && (starting || m_sinceLanding > settings.settleDelaySeconds) && m_feet[0].mode == FootMode3D::Stance
        && m_feet[1].mode == FootMode3D::Stance && style.planted[0] && style.planted[1]) {
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
            out[s] = stance(side, style, settings);
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
            out[s] = { FootMode3D::Swing, foot.position, foot.orientation };
            if (style.planted[s] && (foot.styleSwung || remaining < 0.0f) && (foot.progress >= 1.0f || !foot.styleSwung)) {
                // O pouso do estilo (ou ele parou de andar com o pe no ar).
                if (foot.styleSwung && onGround(side, foot.position, foot.orientation)) {
                    lock(side, foot.position, foot.orientation);
                    out[s].mode = FootMode3D::Stance;
                } else {
                    startStep(side, settings.catchSeconds, 0.5f * settings.settleHeight);
                    ++m_telemetry.catchSteps;
                    out[s].mode = FootMode3D::Step;
                }
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
                lock(side, foot.position, foot.orientation);
                out[s].mode = FootMode3D::Stance;
            }
            break;
        }
        case FootMode3D::Free: {
            foot.progress = std::min(1.0f, foot.progress + dt / std::max(0.01f, settings.freeBlendSeconds));
            const float keep = 1.0f - smooth(foot.progress);
            foot.position = style.position[s] + foot.offset * keep;
            foot.orientation = (athleteYawRotation3D(foot.offsetYaw * keep) * style.orientation[s]).normalized();
            out[s] = { FootMode3D::Free, foot.position, foot.orientation };
            if (!free && style.planted[s] && foot.progress >= 1.0f) {
                // De volta ao chao (o pouso do pulo, o fim do gesto): trava
                // onde esta; ainda no ar, o passo curto ate o estilo.
                if (onGround(side, foot.position, foot.orientation)) {
                    lock(side, foot.position, foot.orientation);
                    out[s].mode = FootMode3D::Stance;
                } else {
                    startStep(side, settings.catchSeconds, 0.5f * settings.settleHeight);
                    ++m_telemetry.catchSteps;
                    out[s].mode = FootMode3D::Step;
                }
            }
            break;
        }
        }
    }
    for (std::size_t side = 0; side < 2; ++side) {
        Foot& foot = m_feet[side];
        foot.velocity = foot.haveHistory ? (foot.position - before[side]) * (1.0f / dt) : Vec3 {};
        foot.stylePrevious = style.position[side];
        foot.haveHistory = true;
    }
    return out;
}

} // namespace MatterEngine
