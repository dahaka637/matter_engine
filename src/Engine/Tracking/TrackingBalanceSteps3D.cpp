#include "Engine/Tracking/TrackingBalanceSteps3D.hpp"

#include "Engine/Character/CharacterFootGeometry3D.hpp"

#include <algorithm>
#include <cmath>

namespace MatterEngine {
namespace {

constexpr float Pi = 3.14159265358979323846f;

float wrap(float angle) { return std::remainder(angle, 2.0f * Pi); }
Quaternion spinZ(float radians) { return Quaternion::fromAxisAngle({ 0.0f, 0.0f, 1.0f }, radians); }
// No rumo do corpo (x a frente, y a esquerda) e de volta.
Vec3 toLocal(Vec3 v, float yaw) {
    const float c = std::cos(yaw), s = std::sin(yaw);
    return { c * v.x + s * v.y, -s * v.x + c * v.y, 0.0f };
}
Vec3 toWorld(Vec3 v, float yaw) {
    const float c = std::cos(yaw), s = std::sin(yaw);
    return { c * v.x - s * v.y, s * v.x + c * v.y, 0.0f };
}
float planar(Vec3 v) { return std::hypot(v.x, v.y); }
float quintic(float t) {
    t = std::clamp(t, 0.0f, 1.0f);
    return t * t * t * (t * (6.0f * t - 15.0f) + 10.0f);
}
// O pe plano no chao: o rumo do pe de agora com a inclinacao do pe do idle (que e plano).
Quaternion flatFoot(const RagdollLinkDefinition3D& link, Quaternion actual, Quaternion idle) {
    return (spinZ(wrap(footYaw3D(link, actual) - footYaw3D(link, idle))) * idle).normalized();
}

} // namespace

void TrackingBalanceSteps3D::reset() {
    const float limit = maximumSwingLoad, weight = loadWeight;
    *this = TrackingBalanceSteps3D {};
    maximumSwingLoad = limit;
    loadWeight = weight;
}

TrackingBalanceSteps3D::Plan TrackingBalanceSteps3D::plan(const RagdollProfile3D& profile,
    std::array<std::uint32_t, 2> footLinks, const TrackingBalanceStepsInput3D& input, int only) const {
    (void)profile;
    (void)footLinks;
    const TrackingBalance3D& b = input.balance;
    const float yaw = input.facingYaw;
    const float seconds = StepSeconds + StepSecondsShift * std::clamp(input.command.time, -1.0f, 1.0f);
    // (O ponto de captura no fim do passo, com a pressao no pe apoiado: ele diverge dali por e^(omega T).)
    const float grow = std::exp(std::clamp(b.omega, 1.0f, 6.0f) * seconds);
    Plan best;
    for (int side = 0; side < 2; ++side) {
        if (only >= 0 && side != only) continue;
        const int other = 1 - side;
        if (!b.footDown[static_cast<std::size_t>(other)]) continue;
        const Vec3 p = b.sole[static_cast<std::size_t>(other)];
        const Vec3 icp { p.x + (b.capture.x - p.x) * grow, p.y + (b.capture.y - p.y) * grow, p.z };
        Vec3 away { icp.x - p.x, icp.y - p.y, 0.0f };
        const float length = planar(away);
        away = length > 1.0e-3f ? away * (1.0f / length) : Vec3 { std::cos(yaw), std::sin(yaw), 0.0f };
        Vec3 land = icp + away * CaptureOffset
            + toWorld(Vec3 { std::clamp(input.command.landForward, -1.0f, 1.0f), std::clamp(input.command.landLeft, -1.0f, 1.0f),
                          0.0f } * LandShift, yaw);
        // (Sem cruzar: o esquerdo sempre SideGap a esquerda do apoiado, o direito a direita; o passo ate MaximumStep do pe
        // apoiado e o pouso ate MaximumReach da pelve, no plano.)
        Vec3 rel = toLocal(land - p, yaw);
        rel.y = side == 0 ? std::max(rel.y, SideGap) : std::min(rel.y, -SideGap);
        const float step = planar(rel);
        if (step > MaximumStep) rel = rel * (MaximumStep / step);
        land = p + toWorld(rel, yaw);
        Vec3 reach { land.x - input.pelvis.x, land.y - input.pelvis.y, 0.0f };
        const float far = planar(reach);
        if (far > MaximumReach) land = Vec3 { input.pelvis.x, input.pelvis.y, p.z } + reach * (MaximumReach / far);
        land.z = p.z;
        // (A base depois do pouso: a sola apoiada e a do passo transladada ao pouso; a margem do ponto de captura previsto.)
        const Vec3 move { land.x - b.sole[static_cast<std::size_t>(side)].x, land.y - b.sole[static_cast<std::size_t>(side)].y, 0.0f };
        std::array<Vec3, 8> base {};
        for (std::size_t k = 0; k < 4; ++k) {
            base[k] = b.corners[static_cast<std::size_t>(other)][k];
            base[4 + k] = b.corners[static_cast<std::size_t>(side)][k] + move;
        }
        const float after = trackingSignedHullDistance3D(base, icp);
        // (Primeiro capturar; depois o pe que carrega menos - o carregado nem sai do chao (15h20, a sonda: o pe "do passo"
        // ficou no chao o passo inteiro) - e o passo mais curto; o pe ja no ar sai na frente.)
        const float load = b.load[static_cast<std::size_t>(side)];
        if (load > maximumSwingLoad && b.footDown[static_cast<std::size_t>(side)]) continue;
        const float score = std::min(after, 0.05f) - loadWeight * load - 0.15f * planar(move)
            + (b.footDown[static_cast<std::size_t>(side)] ? 0.0f : 0.02f);
        if (score > best.score) {
            best.valid = true;
            best.side = side;
            best.score = score;
            // (O alvo e a origem do link do pe: a sola esta deslocada dela.)
            const Vec3 offset = input.foot[static_cast<std::size_t>(side)] - b.sole[static_cast<std::size_t>(side)];
            best.to = { land.x + offset.x, land.y + offset.y, input.foot[static_cast<std::size_t>(side)].z };
        }
    }
    return best;
}

void TrackingBalanceSteps3D::begin(const RagdollProfile3D& profile, std::uint32_t link, const TrackingBalanceStepsInput3D& input,
    int side, Vec3 to, float seconds, float lift, bool recovery) {
    const std::size_t s = static_cast<std::size_t>(side);
    m_swing = side;
    m_recovery = recovery;
    m_touches = 0;
    m_elapsed = 0.0f;
    m_duration = std::max(0.15f, seconds);
    m_lift = lift;
    m_from = input.foot[s];
    m_fromOrientation = input.footOrientation[s];
    m_toOrientation = input.idleOrientation[s];
    m_to = to;
    m_to.z = footLockHeight3D(profile.links[link], input.groundHeight, m_toOrientation);
}

void TrackingBalanceSteps3D::update(const RagdollProfile3D& profile, std::array<std::uint32_t, 2> footLinks,
    const TrackingBalanceStepsInput3D& input) {
    const float dt = std::max(1.0e-4f, input.deltaTime);
    const TrackingBalance3D& b = input.balance;
    const float yaw = input.facingYaw;
    m_recoveryStarted = m_settleStarted = false;
    m_yaw = yaw;
    m_center = b.centerOfMass;
    const auto link = [&](std::size_t side) -> const RagdollLinkDefinition3D& { return profile.links[footLinks[side]]; };
    const auto plant = [&](std::size_t side, Vec3 at, Quaternion orientation) {
        m_anchorOrientation[side] = flatFoot(link(side), orientation, input.idleOrientation[side]);
        m_anchor[side] = { at.x, at.y, footLockHeight3D(link(side), input.groundHeight, m_anchorOrientation[side]) };
    };
    if (!m_started) {
        for (std::size_t side = 0; side < 2; ++side) plant(side, input.foot[side], input.footOrientation[side]);
        m_started = true;
        m_sinceLanding = 1.0f;
    }
    // O passo no ar: replanejado ate 60% (no maximo 0,9 m/s de mudanca do pouso - o alvo nao salta) e pousado no toque.
    if (m_swing >= 0) {
        const std::size_t s = static_cast<std::size_t>(m_swing);
        m_elapsed += dt;
        const float tau = m_elapsed / m_duration;
        if (m_recovery && tau < 0.6f) {
            const Plan p = plan(profile, footLinks, input, m_swing);
            if (p.valid) {
                Vec3 d { p.to.x - m_to.x, p.to.y - m_to.y, 0.0f };
                const float length = planar(d), limit = 0.9f * dt;
                if (length > limit) d = d * (limit / length);
                m_to.x += d.x;
                m_to.y += d.y;
            }
        }
        // (O pouso com o contato sustentado: duas decisoes seguidas no chao depois de 60% do passo.)
        m_touches = b.footDown[s] && tau > 0.6f ? m_touches + 1 : 0;
        const bool touched = m_touches >= 2;
        if ((touched && tau > 0.85f) || tau >= 1.3f) {
            plant(s, touched ? input.foot[s] : m_to, touched ? input.footOrientation[s] : m_toOrientation);
            m_swing = -1;
            m_sinceLanding = 0.0f;
        }
    } else {
        m_sinceLanding = std::min(1.0f, m_sinceLanding + dt);
    }
    // Os pes no chao ficam onde estao (escorregando, a ancora vai junto).
    for (std::size_t side = 0; side < 2; ++side)
        if (static_cast<int>(side) != m_swing && b.footDown[side]) plant(side, input.foot[side], input.footOrientation[side]);
    // A base contra a do idle: o direito contra onde o idle o quer em relacao ao esquerdo, cada uma no rumo dos proprios pes (a
    // media dos dois) - no da pelve, que balanca no idle, a base "girava" sem nenhum pe sair do lugar.
    const auto feetYaw = [&](Quaternion left, Quaternion right) {
        const float a = footYaw3D(link(0), left), c = footYaw3D(link(1), right);
        return std::atan2(std::sin(a) + std::sin(c), std::cos(a) + std::cos(c));
    };
    const float idleYaw = feetYaw(input.idleOrientation[0], input.idleOrientation[1]);
    const Vec3 idleRight = toLocal(input.idleFoot[1] - input.idleFoot[0], idleYaw);
    const Vec3 nowRight = toLocal(m_anchor[1] - m_anchor[0], feetYaw(m_anchorOrientation[0], m_anchorOrientation[1]));
    m_stanceOffset = { nowRight.x - idleRight.x, nowRight.y - idleRight.y, 0.0f };
    m_stanceError = planar(m_stanceOffset);
    const Plan suggestion = plan(profile, footLinks, input, -1);
    m_suggested = suggestion.valid ? suggestion.to : b.capture;
    // O passo de recuperacao: a margem do ponto de captura alem do limiar (que a rede desloca).
    // (A revisao do Codex, 15h25: segurar ate -6 cm era tarde demais com o atraso do PD - a rede antecipa ate +4 cm e segura
    // no maximo ate -3 cm.)
    const float go = std::clamp(input.command.go, -1.0f, 1.0f);
    const float threshold = TriggerMargin + (go >= 0.0f ? TriggerShift * go : 0.02f * go);
    // (Subindo - o fim do levantar que decola - nao: o passo sai do chao com o corpo indo para cima.)
    if (m_swing < 0 && m_sinceLanding >= MinimumDoubleSupport && (b.footDown[0] || b.footDown[1]) && b.margin < threshold
        && b.velocity.z < MaximumRiseSpeed && suggestion.valid) {
        const std::size_t s = static_cast<std::size_t>(suggestion.side);
        const float seconds = StepSeconds + StepSecondsShift * std::clamp(input.command.time, -1.0f, 1.0f);
        const float lift = std::clamp(0.04f + 0.08f * planar(suggestion.to - input.foot[s]), 0.04f, 0.10f);
        begin(profile, footLinks[s], input, suggestion.side, suggestion.to, seconds, lift, true);
        ++recoverySteps;
        m_recoveryStarted = true;
    }
    // O passo de estabilizacao: quieto, com a base fora da do idle, o pe mais longe do centro de massa (o que carrega
    // menos) volta para onde o idle o quer em relacao ao outro.
    // (05/11 02h10, ele: "ele levanta e fica todo torto, ai subitamente se corrige para o IDLE alerta... deveria, assim que se
    // levanta, ja tentar ficar no idle alerta sem perder o auto-equilibrio": medido, depois de levantar ele so ficava quieto
    // em 36% do tempo; o afrouxar destes limites foi testado e nao adotado - ver o .hpp.)
    const bool quiet = b.margin > SettleMargin && planar(b.velocity) < SettleSpeed;
    m_quietSeconds = quiet ? m_quietSeconds + dt : 0.0f;
    if (m_swing < 0 && m_quietSeconds >= SettleQuietSeconds && m_stanceError > SettleDrift && m_sinceLanding > 0.25f
        && b.footDown[0] && b.footDown[1]) {
        const int side = planar(input.foot[0] - b.centerOfMass) > planar(input.foot[1] - b.centerOfMass) ? 0 : 1;
        const std::size_t s = static_cast<std::size_t>(side), o = 1 - s;
        // (No rumo do pe que fica, menos a abertura dele no idle: o passo devolve a base inteira.)
        const float stay = footYaw3D(link(o), m_anchorOrientation[o]) - footYaw3D(link(o), input.idleOrientation[o]) + idleYaw;
        const Vec3 rel = toWorld(side == 1 ? idleRight : idleRight * -1.0f, stay);
        begin(profile, footLinks[s], input, side, Vec3 { m_anchor[o].x + rel.x, m_anchor[o].y + rel.y, input.foot[s].z },
            SettleSeconds, SettleLift, false);
        ++settleSteps;
        m_settleStarted = true;
        m_quietSeconds = 0.0f;
    }
    // O peso do IK: com um passo ou a base fora da do idle; volta ao clipe puro em 0,3 s com a base de volta.
    const bool want = m_swing >= 0 || m_stanceError > ReleaseDrift;
    // (05/11: a volta ao clipe puro em ReleaseSeconds - em 0,3 s era o "conserto" subito.)
    m_weight = want ? std::min(1.0f, m_weight + dt / 0.1f) : std::max(0.0f, m_weight - dt / ReleaseSeconds);
    // A pelve da referencia para cima da base (o idle quer a pelve neste ponto em relacao ao meio dos pes); no passo, nada.
    m_rootShift = {};
    if (m_swing < 0) {
        const Vec3 offset = input.pelvis - (input.idleFoot[0] + input.idleFoot[1]) * 0.5f;
        const Vec3 target = (m_anchor[0] + m_anchor[1]) * 0.5f + offset;
        Vec3 d { target.x - input.pelvis.x, target.y - input.pelvis.y, 0.0f };
        const float length = planar(d);
        if (length > RootPull) d = d * (RootPull / length);
        m_rootShift = d * m_weight;
    }
    m_foot = input.foot;
}

void TrackingBalanceSteps3D::feet(float ahead, std::array<FootTarget3D, 2>& out) const {
    for (std::size_t side = 0; side < 2; ++side) {
        if (static_cast<int>(side) == m_swing) {
            const float tau = std::clamp((m_elapsed + ahead) / m_duration, 0.0f, 1.0f);
            const float k = quintic(tau);
            Vec3 p = m_from + (m_to - m_from) * k;
            p.z = m_from.z + (m_to.z - m_from.z) * k + m_lift * std::sin(Pi * tau);
            out[side] = { FootMode3D::Swing, p, slerp3D(m_fromOrientation, m_toOrientation, k) };
        } else {
            out[side] = { FootMode3D::Stance, m_anchor[side], m_anchorOrientation[side] };
        }
    }
}

void TrackingBalanceSteps3D::features(std::span<float> out) const {
    if (out.size() < FeatureCount) return;
    const Vec3 planned = m_swing >= 0 ? toLocal(m_to - m_foot[static_cast<std::size_t>(m_swing)], m_yaw) : Vec3 {};
    const Vec3 suggested = toLocal(m_suggested - m_center, m_yaw);
    out[0] = m_swing == 0 ? 1.0f : 0.0f;
    out[1] = m_swing == 1 ? 1.0f : 0.0f;
    out[2] = m_swing >= 0 ? std::clamp(m_elapsed / m_duration, 0.0f, 1.0f) : 0.0f;
    out[3] = planned.x;
    out[4] = planned.y;
    out[5] = std::clamp(suggested.x, -1.5f, 1.5f);
    out[6] = std::clamp(suggested.y, -1.5f, 1.5f);
    out[7] = m_sinceLanding;
    out[8] = m_stanceOffset.x;
    out[9] = m_stanceOffset.y;
}

} // namespace MatterEngine
