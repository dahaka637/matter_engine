#include "Engine/Athlete/Modes/ModeRules3D.hpp"

#include <algorithm>
#include <cmath>

namespace MatterEngine {

namespace {

constexpr std::uint8_t Standing = modeBit3D(BodyModeKind3D::Animated) | modeBit3D(BodyModeKind3D::Returning);
constexpr std::uint8_t BridgeOnly = modeBit3D(BodyModeKind3D::Bridge);
constexpr std::uint8_t Guided = Standing | BridgeOnly;

// A tabela (prioridade menor vence; empate, a ordem da tabela). O Ativo
// (experimental, desligado) fica fora: segue as regras dele.
constexpr ModeRule3D Table[] = {
    { Guided, BodyModeKind3D::Physical, 2, ModeEvent3D::Grab },
    { Guided, BodyModeKind3D::Physical, 3, ModeEvent3D::Impact },
    { Guided, BodyModeKind3D::Physical, 3, ModeEvent3D::Penetration },
    { Guided, BodyModeKind3D::Physical, 3, ModeEvent3D::ActionContact },
    { Standing, BodyModeKind3D::Physical, 3, ModeEvent3D::CapsuleWall },
    { BridgeOnly, BodyModeKind3D::Physical, 3, ModeEvent3D::BridgePose },
    { BridgeOnly, BodyModeKind3D::Physical, 3, ModeEvent3D::BridgeTilt },
    { BridgeOnly, BodyModeKind3D::Physical, 3, ModeEvent3D::BridgeSupport },
    { BridgeOnly, BodyModeKind3D::Physical, 3, ModeEvent3D::BridgeAssist },
    { Standing, BodyModeKind3D::Physical, 4, ModeEvent3D::AirTackle },
    { Standing, BodyModeKind3D::Physical, 4, ModeEvent3D::Jump },
};

struct Fired {
    bool fired = false;
    float value = 0.0f;
    float threshold = 0.0f;
};

bool finiteNonNegative(float value) { return std::isfinite(value) && value >= 0.0f; }

} // namespace

const char* modeEventName3D(ModeEvent3D event) {
    switch (event) {
    case ModeEvent3D::None: return "nada";
    case ModeEvent3D::Grab: return "agarrado";
    case ModeEvent3D::Impact: return "impacto";
    case ModeEvent3D::Penetration: return "membro no mundo";
    case ModeEvent3D::CapsuleWall: return "parede";
    case ModeEvent3D::ActionContact: return "contato da acao";
    case ModeEvent3D::AirTackle: return "voadora";
    case ModeEvent3D::Jump: return "pulo";
    case ModeEvent3D::BridgePose: return "ponte: fora da pose";
    case ModeEvent3D::BridgeTilt: return "ponte: tronco tombando";
    case ModeEvent3D::BridgeSupport: return "ponte: sem apoio";
    case ModeEvent3D::BridgeAssist: return "ponte: ajuda caiu";
    }
    return "?";
}

const char* bodyModeName3D(BodyModeKind3D mode) {
    switch (mode) {
    case BodyModeKind3D::Animated: return "animado";
    case BodyModeKind3D::Physical: return "fisico";
    case BodyModeKind3D::Bridge: return "ponte";
    case BodyModeKind3D::Returning: return "volta";
    case BodyModeKind3D::Active: return "ativo";
    }
    return "?";
}

bool ModeRuleParams3D::valid() const {
    return finiteNonNegative(capsuleWallImpact) && bridgePoseDegrees > 0.0f && bridgePoseDegrees < 90.0f
        && bridgePendulumDegrees >= 0.0f && bridgePendulumDegrees < 90.0f
        && finiteNonNegative(bridgePoseSeconds) && finiteNonNegative(bridgeTiltRate) && bridgeNoSupportSeconds > 0.0f
        && bridgeAssist >= 0.0f && bridgeAssist <= 1.0f && finiteNonNegative(abortLockoutSeconds)
        && finiteNonNegative(returnQuietSeconds);
}

std::span<const ModeRule3D> modeRuleTable3D() { return Table; }

float pendulumDegrees3D(Vec3 centerOfMass, Vec3 support) {
    const float height = std::max(0.2f, centerOfMass.z - support.z);
    const float angle = std::atan(std::hypot(centerOfMass.x - support.x, centerOfMass.y - support.y) / height) * 57.29578f;
    return std::isfinite(angle) ? angle : 0.0f;
}

float pendulumDeviationDegrees3D(Vec3 centerOfMass, Vec3 support, Vec3 poseCenterOfMass, Vec3 poseSupport) {
    const auto lean = [](Vec3 com, Vec3 base) {
        const float height = std::max(0.2f, com.z - base.z);
        return Vec3 { (com.x - base.x) / height, (com.y - base.y) / height, 0.0f };
    };
    const Vec3 difference = lean(centerOfMass, support) - lean(poseCenterOfMass, poseSupport);
    const float deviation = std::atan(difference.length()) * 57.29578f;
    return std::isfinite(deviation) ? deviation : 0.0f;
}

void ModeRules3D::reset() {
    m_poseSeconds = 0.0f;
    m_noSupportSeconds = 0.0f;
    m_sinceImpact = 10.0f;
    m_lockout = 0.0f;
}

ModeDecision3D ModeRules3D::decide(BodyModeKind3D current, const ModeSignals3D& signals,
    const ModeRuleParams3D& params, float deltaTime) {
    const float dt = std::max(0.0f, deltaTime);
    m_sinceImpact = signals.contacts.impact ? 0.0f : std::min(10.0f, m_sinceImpact + dt);
    m_lockout = std::max(0.0f, m_lockout - dt);
    if (current == BodyModeKind3D::Bridge) {
        m_poseSeconds = signals.poseDeviationDegrees > params.bridgePoseDegrees
                && signals.pendulumDegrees > params.bridgePendulumDegrees
            ? m_poseSeconds + dt
            : 0.0f;
        m_noSupportSeconds = !signals.supported && !signals.airborneByDesign ? m_noSupportSeconds + dt : 0.0f;
    } else {
        m_poseSeconds = 0.0f;
        m_noSupportSeconds = 0.0f;
    }
    const auto fires = [&](ModeEvent3D event) -> Fired {
        const ContactAssessment3D& c = signals.contacts;
        switch (event) {
        case ModeEvent3D::Grab: return { signals.grabbed, signals.grabbed ? 1.0f : 0.0f, 1.0f };
        case ModeEvent3D::Impact:
            return { c.impact && !c.fromPenetration, std::max(c.bodyDeltaV, c.linkDeltaV), 0.0f };
        case ModeEvent3D::Penetration: return { c.impact && c.fromPenetration, c.penetrationMeters, 0.0f };
        case ModeEvent3D::ActionContact: return { signals.actionContact, c.intentionalImpulse, 0.0f };
        case ModeEvent3D::CapsuleWall:
            return { signals.capsuleWallImpact > params.capsuleWallImpact, signals.capsuleWallImpact,
                params.capsuleWallImpact };
        case ModeEvent3D::BridgePose:
            return { m_poseSeconds >= params.bridgePoseSeconds && m_poseSeconds > 0.0f,
                signals.poseDeviationDegrees, params.bridgePoseDegrees };
        case ModeEvent3D::BridgeTilt:
            return { params.bridgeTiltRate > 0.0f && signals.chestTiltRate > params.bridgeTiltRate,
                signals.chestTiltRate, params.bridgeTiltRate };
        case ModeEvent3D::BridgeSupport:
            return { m_noSupportSeconds > params.bridgeNoSupportSeconds, m_noSupportSeconds,
                params.bridgeNoSupportSeconds };
        case ModeEvent3D::BridgeAssist:
            return { signals.assistLevel < params.bridgeAssist, signals.assistLevel, params.bridgeAssist };
        case ModeEvent3D::AirTackle: return { signals.airTackleStart, 1.0f, 1.0f };
        case ModeEvent3D::Jump: return { signals.jumped, 1.0f, 1.0f };
        case ModeEvent3D::None: break;
        }
        return {};
    };
    ModeDecision3D best;
    const std::uint8_t bit = modeBit3D(current);
    for (const ModeRule3D& rule : Table) {
        if ((rule.from & bit) == 0) continue;
        if (best.change && rule.priority >= best.priority) continue;
        const Fired fired = fires(rule.event);
        if (!fired.fired) continue;
        best.change = true;
        best.to = rule.to;
        best.event = rule.event;
        best.value = fired.value;
        best.threshold = fired.threshold;
        best.priority = rule.priority;
    }
    return best;
}

void ModeRules3D::noteTransition(BodyModeKind3D from, const ModeDecision3D& decision, const ModeRuleParams3D& params) {
    if (!decision.change) return;
    if (from == BodyModeKind3D::Bridge && decision.to == BodyModeKind3D::Physical)
        m_lockout = params.abortLockoutSeconds;
    m_poseSeconds = 0.0f;
    m_noSupportSeconds = 0.0f;
}

bool ModeRules3D::returnAllowed(const ModeRuleParams3D& params) const {
    return m_lockout <= 0.0f && m_sinceImpact > params.returnQuietSeconds;
}

} // namespace MatterEngine
