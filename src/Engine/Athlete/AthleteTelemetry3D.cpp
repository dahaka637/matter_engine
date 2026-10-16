#include "Engine/Athlete/AthleteTelemetry3D.hpp"

namespace MatterEngine {

AthleteTelemetry3D athleteTelemetry3D(const AthleteState3D& athlete) {
    const LegacyAthleteState3D& legacy = athlete.legacy;
    AthleteTelemetry3D out;
    switch (legacy.bodyMode) {
    case LegacyBodyMode3D::Animated: out.mode = BodyModeKind3D::Animated; break;
    case LegacyBodyMode3D::Physical: out.mode = BodyModeKind3D::Physical; break;
    case LegacyBodyMode3D::Returning: out.mode = BodyModeKind3D::Returning; break;
    case LegacyBodyMode3D::Active: out.mode = legacy.bridge ? BodyModeKind3D::Bridge : BodyModeKind3D::Active; break;
    }
    out.modeSeconds = legacy.bodyModeSeconds;
    out.guided = legacy.guided;
    out.assistLevel = legacy.impactAssist.level();
    out.calmSeconds = legacy.calmSeconds;
    const CharacterLocomotionTelemetry3D& locomotion = legacy.locomotion.telemetry();
    const bool physical = out.mode == BodyModeKind3D::Physical;
    out.fallen = physical && locomotion.state == CharacterLocomotionState3D::Fallen;
    out.gettingUp = physical && locomotion.state == CharacterLocomotionState3D::GettingUp;
    out.tackling = (physical && locomotion.state == CharacterLocomotionState3D::Tackling) || legacy.defaultTackling;
    out.stagger = physical ? locomotion.stagger : 0.0f;
    out.perturbation = physical ? legacy.adaptivePhysics.output().perturbation : 0.0f;
    out.contacts = athlete.lastContacts;
    out.lastDecision = athlete.lastDecision;
    out.lastDecisionFrom = athlete.lastDecisionFrom;
    return out;
}

int athleteDumpModeCode3D(BodyModeKind3D mode) {
    switch (mode) {
    case BodyModeKind3D::Animated: return 0;
    case BodyModeKind3D::Physical: return 1;
    case BodyModeKind3D::Returning: return 2;
    case BodyModeKind3D::Bridge:
    case BodyModeKind3D::Active: return 3;
    }
    return 0;
}

} // namespace MatterEngine
