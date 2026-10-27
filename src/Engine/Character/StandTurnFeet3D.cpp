#include "Engine/Character/StandTurnFeet3D.hpp"

#include "Engine/Character/CharacterFootGeometry3D.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>

namespace MatterEngine {

namespace {

constexpr float Pi = 3.14159265358979323846f;
// Os do ContactFootwork3D, calibrados para o padrao (o usuario, 19/10: "so
// bonitos, parecerem reais; o pe pode deslizar; suave nao e lento"): a torcao
// (ou o que falta) que dispara o giro (12 graus no fisico), o giro maximo por
// passo, o empate de "mais atrasado" e o passo do segundo pe.
constexpr float TwistTrigger = 0.45f;
constexpr float StepLimit = 1.25f;
constexpr float BehindTie = 0.17f;
constexpr float SecondStepDrift = 0.05f;
constexpr float SecondStepYaw = 0.17f;
// Parado de novo: o pe mais fora da pose da um passo de acomodacao.
constexpr float SettleDrift = 0.04f;
constexpr float SettleYaw = 0.12f;
// O pe apoiado acompanha esta parte do giro da pelve rodando sobre a bola
// (desliza; menos passos, mais calmos) e nunca torce mais que o limite.
constexpr float PlantedFollow = 0.5f;
constexpr float PlantedTwistLimit = 0.8f;
// A coleira (21/10, pedido do usuario: girando muito rapido as pernas
// embolavam; "os pes podem deslizar de forma artificial, o movimento so
// precisa ser bonito e limitado"): a base dos pes nunca fica mais que isto
// atras da pelve - o excesso leva os dois pes deslizando em volta do corpo, e
// os passos seguem no ritmo deles.
constexpr float StanceLeash = 0.9f;
// A ultima trava: o pe esquerdo sempre ao menos isto a esquerda do direito,
// pela frente da pelve (m); o que falta desliza os dois para fora.
constexpr float SideGap = 0.12f;
// O pouso do pe no ar anda ate o pedido no maximo a isto (rad/s).
constexpr float GoalYawRate = 6.0f;
// O tempo do passo (s): o do planejador no giro lento (0,18 a 0,29 no fisico)
// e no mais rapido (0,10 a 0,17 no fisico, sobre-humano no padrao).
constexpr float StepSecondsMinimum = 0.28f;
constexpr float StepSecondsMaximum = 0.40f;
constexpr float StepSecondsFast = 0.22f;

float wrap(float angle) { return std::remainder(angle, 2.0f * Pi); }

Quaternion spinZ(float radians) { return Quaternion::fromAxisAngle({ 0.0f, 0.0f, 1.0f }, radians); }

// O pe levado em volta do corpo (a vertical pela raiz).
StandTurnFoot3D orbit(StandTurnFoot3D foot, Vec3 root, float radians) {
    const Quaternion spin = spinZ(radians);
    const Vec3 offset = spin.rotate(Vec3 { foot.position.x - root.x, foot.position.y - root.y, 0.0f });
    foot.position.x = root.x + offset.x;
    foot.position.y = root.y + offset.y;
    foot.orientation = (spin * foot.orientation).normalized();
    return foot;
}

// O pe girado em volta da vertical sobre a bola dele (ela fica; o calcanhar roda).
StandTurnFoot3D pivotOnBall(const RagdollLinkDefinition3D& link, StandTurnFoot3D foot, float radians) {
    const Vec3 ball = footBallLocal3D(link);
    const Vec3 at = foot.position + foot.orientation.rotate(ball);
    foot.orientation = (spinZ(radians) * foot.orientation).normalized();
    const Vec3 moved = at - foot.orientation.rotate(ball);
    foot.position.x = moved.x;
    foot.position.y = moved.y;
    return foot;
}

} // namespace

void StandTurnFeet3D::reset() {
    m_feet = {};
    m_pending = -1;
    m_engaged = false;
    m_weight = 0.0f;
}

float StandTurnFeet3D::landingYaw(const StandTurnFeetInput3D& input, const Foot& foot) const {
    // A pose parada ja no rumo final da pelve, ate StepLimit a partir do rumo
    // em que o passo comecou, no sentido em que ele comecou.
    float remaining = wrap(input.targetYawRadians - foot.fromFacing);
    if (foot.sign > 0.0f && remaining < -0.5f * Pi) remaining += 2.0f * Pi;
    if (foot.sign < 0.0f && remaining > 0.5f * Pi) remaining -= 2.0f * Pi;
    return foot.fromFacing + std::clamp(remaining, -StepLimit, StepLimit);
}

StandTurnFoot3D StandTurnFeet3D::landing(const RagdollProfile3D& profile, std::size_t link,
    const StandTurnFeetInput3D& input, std::size_t side, const Foot& foot) const {
    return landingAt(profile, link, input, side, landingYaw(input, foot));
}

StandTurnFoot3D StandTurnFeet3D::landingAt(const RagdollProfile3D& profile, std::size_t link,
    const StandTurnFeetInput3D& input, std::size_t side, float finalYaw) const {
    const Quaternion spin = spinZ(wrap(finalYaw - input.facingYawRadians));
    const StandTurnFoot3D& reference = input.reference[side];
    StandTurnFoot3D result;
    result.orientation = (spin * reference.orientation).normalized();
    result.position = input.rootWorld + spin.rotate(reference.position - input.rootWorld);
    result.position.z = footLockHeight3D(profile.links[link], input.groundHeight, result.orientation);
    return result;
}

void StandTurnFeet3D::start(const RagdollProfile3D& profile, std::size_t link, const StandTurnFeetInput3D& input,
    std::size_t side, Path path) {
    Foot& foot = m_feet[side];
    foot.swinging = true;
    foot.path = path;
    foot.seconds = 0.0f;
    foot.from = foot.anchor;
    foot.fromFacing = input.facingYawRadians;
    foot.sign = wrap(input.targetYawRadians - input.facingYawRadians) >= 0.0f ? 1.0f : -1.0f;
    // O tempo do passo de giro do planejador, pela distancia e pelo giro do
    // pe, e mais curto quanto mais rapido o giro (o da ajuda cheia).
    foot.goalYaw = landingYaw(input, foot);
    const StandTurnFoot3D to = landingAt(profile, link, input, side, foot.goalYaw);
    const RagdollLinkDefinition3D& definition = profile.links[link];
    const float distance = std::hypot(to.position.x - foot.from.position.x, to.position.y - foot.from.position.y);
    const float yaw = std::abs(wrap(footYaw3D(definition, to.orientation) - footYaw3D(definition, foot.from.orientation)));
    const float base = std::clamp(0.24f + 0.24f * distance + 0.06f * yaw / (0.5f * Pi), StepSecondsMinimum,
        StepSecondsMaximum);
    const float need = std::clamp(std::abs(input.turnRateRadiansPerSecond) / 6.0f
        + std::abs(wrap(input.targetYawRadians - input.facingYawRadians)) / 1.5f, 0.0f, 1.0f);
    foot.duration = base + (StepSecondsFast - base) * need;
}

void StandTurnFeet3D::update(const RagdollProfile3D& profile, std::array<std::size_t, 2> feet,
    const StandTurnFeetInput3D& input) {
    const float dt = std::max(1e-4f, input.deltaTime);
    if (feet[0] == 0 || feet[1] == 0 || feet[0] >= profile.links.size() || feet[1] >= profile.links.size()) {
        reset();
        return;
    }
    const auto onGround = [&](std::size_t side, StandTurnFoot3D foot) {
        foot.position.z = footLockHeight3D(profile.links[feet[side]], input.groundHeight, foot.orientation);
        return foot;
    };
    const float ahead = wrap(input.targetYawRadians - input.facingYawRadians);
    const bool turning = std::abs(input.turnRateRadiansPerSecond) > 0.25f || std::abs(ahead) > 0.05f;
    // Um pe fora da pose parada (mais do que o passo de acomodacao tolera).
    bool displaced = false;
    for (std::size_t side = 0; side < 2 && input.shownValid; ++side) {
        const RagdollLinkDefinition3D& link = profile.links[feet[side]];
        const StandTurnFoot3D& shown = input.shown[side];
        const StandTurnFoot3D& reference = input.reference[side];
        displaced = displaced
            || std::hypot(shown.position.x - reference.position.x, shown.position.y - reference.position.y) > SettleDrift
            || std::abs(wrap(footYaw3D(link, shown.orientation) - footYaw3D(link, reference.orientation))) > SettleYaw;
    }
    if (!input.standing || (!m_engaged && !turning && !displaced)) {
        // Andando (ou parado sem giro): os pes sao os da locomocao; o que
        // estava aqui sai em 0,08 s (andando) ou 0,15 s (assentado).
        m_engaged = false;
        m_pending = -1;
        m_feet[0].swinging = m_feet[1].swinging = false;
        m_weight = std::max(0.0f, m_weight - dt / (input.standing ? 0.15f : 0.08f));
        return;
    }
    if (!m_engaged) {
        // O giro (ou a acomodacao) comeca: os pes apoiados onde estao.
        for (std::size_t side = 0; side < 2; ++side) {
            m_feet[side] = {};
            m_feet[side].anchor = onGround(side, input.shownValid ? input.shown[side] : input.reference[side]);
            m_pose[side] = m_feet[side].anchor;
        }
        m_pending = -1;
        m_engaged = true;
        m_lastFacing = input.facingYawRadians;
    }
    m_weight = std::min(1.0f, m_weight + dt / 0.08f);
    // O pe apoiado acompanha parte do giro (deslizando sobre a bola).
    {
        const float turned = wrap(input.facingYawRadians - m_lastFacing) * PlantedFollow;
        m_lastFacing = input.facingYawRadians;
        for (std::size_t side = 0; side < 2; ++side)
            if (!m_feet[side].swinging && std::abs(turned) > 1e-6f)
                m_feet[side].anchor = onGround(side, pivotOnBall(profile.links[feet[side]], m_feet[side].anchor, turned));
    }

    // O rumo de cada pe apoiado sem o quanto a pose parada o abre, e a base.
    std::array<float, 2> open {}, footFacing {};
    float stance = 0.0f;
    const auto measure = [&]() {
        float sine = 0.0f, cosine = 0.0f;
        for (std::size_t side = 0; side < 2; ++side) {
            const RagdollLinkDefinition3D& link = profile.links[feet[side]];
            open[side] = wrap(footYaw3D(link, input.reference[side].orientation) - input.facingYawRadians);
            footFacing[side] = footYaw3D(link, m_feet[side].anchor.orientation) - open[side];
            sine += std::sin(footFacing[side]);
            cosine += std::cos(footFacing[side]);
        }
        stance = std::atan2(sine, cosine);
    };
    measure();
    // A coleira: a base atrasada demais desliza com a pelve (o pe no ar leva
    // o comeco e o alvo do passo junto).
    {
        const float lag = wrap(input.facingYawRadians - stance);
        if (std::abs(lag) > StanceLeash) {
            const float excess = lag - std::copysign(StanceLeash, lag);
            for (std::size_t side = 0; side < 2; ++side) {
                Foot& foot = m_feet[side];
                foot.anchor = onGround(side, orbit(foot.anchor, input.rootWorld, excess));
                if (foot.swinging) {
                    foot.from = orbit(foot.from, input.rootWorld, excess);
                    foot.fromFacing += excess;
                    foot.goalYaw += excess;
                }
            }
            measure();
        }
    }
    const float twist = wrap(input.facingYawRadians - stance);
    const float remaining = wrap(input.targetYawRadians - stance);

    // O pe apoiado que torce demais contra a pelve roda sobre a bola.
    for (std::size_t side = 0; side < 2; ++side) {
        Foot& foot = m_feet[side];
        if (foot.swinging) continue;
        const float error = wrap(input.facingYawRadians - footFacing[side]);
        if (std::abs(error) > PlantedTwistLimit)
            foot.anchor = onGround(side, pivotOnBall(profile.links[feet[side]], foot.anchor,
                error - std::copysign(PlantedTwistLimit, error)));
    }

    // Os passos (um pe de cada vez).
    if (!m_feet[0].swinging && !m_feet[1].swinging) {
        const auto drift = [&](std::size_t side, const StandTurnFoot3D& wanted) {
            const RagdollLinkDefinition3D& link = profile.links[feet[side]];
            const StandTurnFoot3D& anchor = m_feet[side].anchor;
            return std::array<float, 2> {
                std::hypot(wanted.position.x - anchor.position.x, wanted.position.y - anchor.position.y),
                std::abs(wrap(footYaw3D(link, wanted.orientation) - footYaw3D(link, anchor.orientation))) };
        };
        if (m_pending >= 0) {
            // O segundo pe, logo depois do primeiro, se ainda esta fora da
            // base nova: contorna o de apoio ate ela.
            const std::size_t side = static_cast<std::size_t>(m_pending);
            m_pending = -1;
            Foot probe = m_feet[side];
            probe.fromFacing = input.facingYawRadians;
            probe.sign = ahead >= 0.0f ? 1.0f : -1.0f;
            const auto [distance, yaw] = drift(side, landing(profile, feet[side], input, side, probe));
            if (distance > SecondStepDrift || yaw > SecondStepYaw) start(profile, feet[side], input, side, Path::Around);
        } else if (std::abs(twist) > TwistTrigger || std::abs(remaining) > TwistTrigger) {
            // Sai primeiro o pe mais atrasado no giro; empatados, na base
            // escalonada o de tras (contornando o da frente), lado a lado o do
            // lado do giro (abrindo sobre a bola do pe).
            std::array<float, 2> behind {};
            for (std::size_t side = 0; side < 2; ++side)
                behind[side] = std::abs(wrap(input.targetYawRadians - footFacing[side]));
            const Vec3 forward { std::cos(input.facingYawRadians), std::sin(input.facingYawRadians), 0.0f };
            const float leftAhead = dot(input.reference[0].position - input.reference[1].position, forward);
            const bool staggered = std::abs(leftAhead) > 0.15f;
            const std::size_t rear = leftAhead > 0.0f ? 1 : 0;
            const std::size_t first = std::abs(behind[0] - behind[1]) > BehindTie ? (behind[0] > behind[1] ? 0 : 1)
                : staggered ? rear : (remaining > 0.0f ? 0 : 1);
            start(profile, feet[first], input, first, staggered && first == rear ? Path::Around : Path::Pivot);
            m_pending = static_cast<int>(1 - first);
        } else if (!turning) {
            // Assentou: o pe mais fora da pose da um passo de acomodacao; com
            // os dois no lugar, os pes voltam a locomocao.
            int worst = -1;
            float worstScore = 1.0f;
            for (std::size_t side = 0; side < 2; ++side) {
                const auto [distance, yaw] = drift(side, onGround(side, input.reference[side]));
                const float score = std::max(distance / SettleDrift, yaw / SettleYaw);
                if (score > worstScore) { worstScore = score; worst = static_cast<int>(side); }
            }
            if (worst >= 0) start(profile, feet[static_cast<std::size_t>(worst)], input,
                static_cast<std::size_t>(worst), Path::Around);
            else m_engaged = false;
        }
    }

    if (std::getenv("XGIRO"))
        std::printf("XGIRO rumo %.2f alvo %.2f base %.2f torcao %.2f taxa %.1f | E %d %d %.2f/%.2f | D %d %d %.2f/%.2f\n",
            input.facingYawRadians, input.targetYawRadians, stance, twist, input.turnRateRadiansPerSecond,
            m_feet[0].swinging, static_cast<int>(m_feet[0].path), m_feet[0].seconds, m_feet[0].duration,
            m_feet[1].swinging, static_cast<int>(m_feet[1].path), m_feet[1].seconds, m_feet[1].duration);
    // O balanco de cada pe (os caminhos do planejador) e a pose.
    for (std::size_t side = 0; side < 2; ++side) {
        Foot& foot = m_feet[side];
        if (!foot.swinging) {
            m_pose[side] = foot.anchor;
            continue;
        }
        const RagdollLinkDefinition3D& link = profile.links[feet[side]];
        foot.seconds += dt;
        const float progress = std::clamp(foot.seconds / std::max(0.05f, foot.duration), 0.0f, 1.0f);
        const float eased = progress * progress * progress * (10.0f + progress * (-15.0f + 6.0f * progress));
        const float arch = std::sin(Pi * progress);
        foot.goalYaw += std::clamp(wrap(landingYaw(input, foot) - foot.goalYaw), -GoalYawRate * dt, GoalYawRate * dt);
        const StandTurnFoot3D to = landingAt(profile, feet[side], input, side, foot.goalYaw);
        Quaternion orientation = slerp3D(foot.from.orientation, to.orientation, eased);
        const Vec3 ball = footBallLocal3D(link);
        Vec3 position;
        float toeDown = 0.0f;
        if (foot.path == Path::Pivot) {
            // Abre girando sobre a bola do pe: ela quase nao sai do lugar, o
            // calcanhar sobe e roda em volta dela.
            const Vec3 ballFrom = foot.from.position + foot.from.orientation.rotate(ball);
            const Vec3 ballTo = to.position + to.orientation.rotate(ball);
            Vec3 ballNow = ballFrom + (ballTo - ballFrom) * eased;
            ballNow.z += 0.028f * arch;
            position = ballNow - orientation.rotate(ball);
            toeDown = 0.26f * arch;
        } else {
            // Contorna o pe de apoio em arco (angulo e distancia em volta
            // dele, nao a reta pela outra perna); o calcanhar sai primeiro.
            const Vec3 pivot = m_feet[1 - side].anchor.position;
            const Vec3 offsetFrom = foot.from.position - pivot;
            const Vec3 offsetTo = to.position - pivot;
            const float fromRadius = std::hypot(offsetFrom.x, offsetFrom.y);
            const float toRadius = std::hypot(offsetTo.x, offsetTo.y);
            if (fromRadius > 0.05f && toRadius > 0.05f) {
                const float fromAngle = std::atan2(offsetFrom.y, offsetFrom.x);
                const float angle = fromAngle + wrap(std::atan2(offsetTo.y, offsetTo.x) - fromAngle) * eased;
                const float radius = std::max(fromRadius + (toRadius - fromRadius) * eased, std::min(fromRadius, toRadius));
                position = pivot + Vec3 { std::cos(angle) * radius, std::sin(angle) * radius, 0.0f };
            } else {
                position = foot.from.position + (to.position - foot.from.position) * eased;
            }
            position.z = foot.from.position.z + (to.position.z - foot.from.position.z) * eased + 0.06f * arch;
            toeDown = 0.18f * std::sin(Pi * std::min(1.0f, progress * 1.4f));
        }
        if (toeDown > 0.0f) {
            // Ponta para baixo girando em volta da bola do pe.
            Vec3 lateral = cross({ 0.0f, 0.0f, 1.0f }, footForwardWorld3D(link, orientation));
            if (lateral.lengthSquared() > 1e-4f) {
                const Vec3 ballWorld = position + orientation.rotate(ball);
                orientation = (Quaternion::fromAxisAngle(lateral.normalized(), toeDown) * orientation).normalized();
                position = ballWorld - orientation.rotate(ball);
            }
        }
        m_pose[side] = { position, orientation };
        if (progress >= 1.0f) {
            foot.swinging = false;
            foot.anchor = to;
        }
    }
    // As pernas nunca cruzam (o pe apoiado leva o deslize junto).
    {
        const Vec3 left { -std::sin(input.facingYawRadians), std::cos(input.facingYawRadians), 0.0f };
        const float apart = dot(m_pose[0].position - m_pose[1].position, left);
        if (apart < SideGap) {
            const Vec3 push = left * (0.5f * (SideGap - apart));
            for (std::size_t side = 0; side < 2; ++side) {
                const Vec3 move = side == 0 ? push : push * -1.0f;
                m_pose[side].position = m_pose[side].position + move;
                if (!m_feet[side].swinging) m_feet[side].anchor.position = m_feet[side].anchor.position + move;
            }
        }
    }
}

} // namespace MatterEngine
