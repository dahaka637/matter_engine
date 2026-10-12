#include "Engine/RL/Calibration/RagdollJointLimitLab3D.hpp"

#include "Engine/Materials/MaterialLibrary.hpp"
#include "Engine/Physics/PhysicsEngine3D.hpp"
#include "Engine/Physics/PhysicsScene3D.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <numbers>
#include <span>
#include <sstream>
#include <stdexcept>
#include <string_view>
#include <utility>

namespace MatterEngine {
namespace {

constexpr float RadiansToDegrees = 180.0f / std::numbers::pi_v<float>;
constexpr float BoundaryToleranceRadians = 2.0f * std::numbers::pi_v<float> / 180.0f;

const char* axisName(RagdollAxis3D axis) {
    switch (axis) {
        case RagdollAxis3D::Twist: return "twist";
        case RagdollAxis3D::Swing1: return "swing1";
        case RagdollAxis3D::Swing2: return "swing2";
    }
    return "unknown";
}

const char* endpointName(RagdollJointLimitEndpoint3D endpoint) {
    return endpoint == RagdollJointLimitEndpoint3D::Minimum ? "minimum" : "maximum";
}

const char* severityName(CalibrationFindingSeverity3D severity) {
    switch (severity) {
        case CalibrationFindingSeverity3D::Information: return "informacao";
        case CalibrationFindingSeverity3D::Warning: return "aviso";
        case CalibrationFindingSeverity3D::Error: return "erro";
    }
    return "desconhecido";
}

bool finite(float value) {
    return std::isfinite(value);
}

std::string fixed(float value, int precision = 2) {
    std::ostringstream output;
    output << std::fixed << std::setprecision(precision) << value;
    return output.str();
}

std::size_t stepsFor(float durationSeconds) {
    return std::max<std::size_t>(
        1, static_cast<std::size_t>(std::ceil(durationSeconds / ReinforcementLearningFixedStepSeconds)));
}

std::size_t findLink(const RagdollProfile3D& profile, std::string_view id) {
    const auto found = std::find_if(profile.links.begin(), profile.links.end(),
        [id](const RagdollLinkDefinition3D& link) { return link.id == id; });
    return found == profile.links.end() ? profile.links.size()
                                        : static_cast<std::size_t>(found - profile.links.begin());
}

std::vector<RagdollDriveTarget3D> neutralTargets(
    const RagdollProfile3D& profile, const RagdollJointLimitLabSettings3D& settings) {
    std::vector<RagdollDriveTarget3D> targets;
    targets.reserve(ragdollDegreesOfFreedom3D(profile));
    for (std::size_t linkIndex = 1; linkIndex < profile.links.size(); ++linkIndex) {
        const RagdollJointDefinition3D& joint = profile.links[linkIndex].inboundJoint;
        for (std::size_t axisIndex = 0; axisIndex < joint.axes.size(); ++axisIndex) {
            if (!joint.axes[axisIndex].enabled) continue;
            RagdollDriveTarget3D target;
            target.linkIndex = static_cast<std::uint32_t>(linkIndex);
            target.axis = static_cast<RagdollAxis3D>(axisIndex);
            target.stiffnessScale = settings.stiffnessScale;
            target.dampingScale = settings.dampingScale;
            target.maximumTorqueScale = settings.maximumTorqueScale;
            target.gravityCompensationScale = 0.0f;
            targets.push_back(target);
        }
    }
    return targets;
}

RagdollJointLimitTrial3D runTrial(PhysicsEngine3D& engine, const MaterialLibrary& materials,
    const RagdollProfile3D& profile, std::size_t linkIndex, RagdollAxis3D axisId, RagdollJointLimitEndpoint3D endpoint,
    const RagdollJointLimitLabSettings3D& settings) {
    const std::size_t axisIndex = static_cast<std::size_t>(axisId);
    const RagdollAxisDefinition3D& axis = profile.links[linkIndex].inboundJoint.axes[axisIndex];

    PhysicsSceneSettings3D sceneSettings;
    sceneSettings.gravity = {};
    sceneSettings.workerThreadCount = 1;
    auto scene = engine.createScene(sceneSettings, materials);
    RagdollSpawnDefinition3D spawn;
    spawn.entityId = 1;
    spawn.pelvisPosition = { 0.0f, 0.0f, 2.0f };
    spawn.rigidityPercent = 0.0f;
    spawn.active = true;
    const RagdollHandle3D ragdoll = scene->createRagdoll(profile, spawn);

    std::vector<RagdollDriveTarget3D> targets = neutralTargets(profile, settings);
    const float targetRadians =
        endpoint == RagdollJointLimitEndpoint3D::Minimum ? axis.minimumRadians : axis.maximumRadians;
    for (RagdollDriveTarget3D& target : targets)
        if (target.linkIndex == static_cast<std::uint32_t>(linkIndex) && target.axis == axisId)
            target.positionRadians = targetRadians;

    RagdollJointLimitTrial3D trial;
    trial.linkId = profile.links[linkIndex].id;
    trial.axis = axisId;
    trial.endpoint = endpoint;
    trial.stepCount = stepsFor(settings.durationSeconds);
    trial.targetRadians = targetRadians;
    trial.minimumObservedRadians = 0.0f;
    trial.maximumObservedRadians = 0.0f;
    trial.initialLinks = scene->ragdollState(ragdoll).links;
    for (std::size_t step = 0; step < trial.stepCount; ++step) {
        scene->setRagdollActiveDriveTargets(ragdoll, targets, false);
        scene->simulate(ReinforcementLearningFixedStepSeconds);
        const RagdollState3D state = scene->ragdollState(ragdoll);
        if (state.joints.size() != profile.links.size()) {
            trial.finite = false;
            continue;
        }
        const RagdollJointState3D& joint = state.joints[linkIndex];
        const float measured = joint.positionRadians[axisIndex];
        trial.finite = trial.finite && finite(measured) && finite(joint.motorTorqueNewtonMeters[axisIndex])
            && finite(joint.constraintTorqueNewtonMeters[axisIndex]);
        trial.minimumObservedRadians = std::min(trial.minimumObservedRadians, measured);
        trial.maximumObservedRadians = std::max(trial.maximumObservedRadians, measured);
        trial.maximumMotorTorqueNewtonMeters =
            std::max(trial.maximumMotorTorqueNewtonMeters, std::abs(joint.motorTorqueNewtonMeters[axisIndex]));
        trial.maximumConstraintTorqueNewtonMeters = std::max(
            trial.maximumConstraintTorqueNewtonMeters, std::abs(joint.constraintTorqueNewtonMeters[axisIndex]));
        trial.appliedMotorTorqueLimitNewtonMeters = joint.motorTorqueLimitNewtonMeters[axisIndex];
        trial.targetApplied =
            trial.targetApplied || std::abs(joint.targetPositionRadians[axisIndex] - targetRadians) < 0.0001f;
        trial.finalRadians = measured;
        trial.finalLinks = state.links;
    }
    return trial;
}

} // namespace

std::size_t RagdollJointLimitLabResult3D::findingCount(CalibrationFindingSeverity3D severity) const {
    return static_cast<std::size_t>(std::count_if(findings.begin(), findings.end(),
        [severity](const CalibrationFinding3D& finding) { return finding.severity == severity; }));
}

bool RagdollJointLimitLabResult3D::hasErrors() const {
    return findingCount(CalibrationFindingSeverity3D::Error) > 0;
}

RagdollJointLimitLabResult3D runRagdollJointLimitLab3D(
    const RagdollProfile3D& profile, const RagdollJointLimitLabSettings3D& settings) {
    validateRagdollProfileOrThrow3D(profile);
    if (!finite(settings.durationSeconds) || settings.durationSeconds <= 0.0f || !finite(settings.stiffnessScale)
        || settings.stiffnessScale < 0.0f || !finite(settings.dampingScale) || settings.dampingScale < 0.0f
        || !finite(settings.maximumTorqueScale) || settings.maximumTorqueScale <= 0.0f)
        throw std::invalid_argument("configuracao invalida do ensaio articular");

    RagdollJointLimitLabResult3D result;
    result.profileId = profile.id;

    PhysicsEngine3D engine;
    MaterialLibrary materials;
    // Todos os eixos ativos do perfil, nos dois extremos.
    for (std::size_t linkIndex = 1; linkIndex < profile.links.size(); ++linkIndex) {
        for (std::size_t axisIndex = 0; axisIndex < 3; ++axisIndex) {
            if (!profile.links[linkIndex].inboundJoint.axes[axisIndex].enabled) continue;
            for (const RagdollJointLimitEndpoint3D endpoint :
                { RagdollJointLimitEndpoint3D::Minimum, RagdollJointLimitEndpoint3D::Maximum }) {
                result.trials.push_back(runTrial(
                    engine, materials, profile, linkIndex, static_cast<RagdollAxis3D>(axisIndex), endpoint, settings));
            }
        }
    }

    for (const RagdollJointLimitTrial3D& trial : result.trials) {
        const std::string path = trial.linkId + "." + axisName(trial.axis) + "." + endpointName(trial.endpoint);
        if (!trial.finite || !trial.targetApplied) {
            result.findings.push_back({ CalibrationFindingSeverity3D::Error, "joint_trial_invalid", path,
                "o solver nao publicou estado finito/alvo aplicado" });
            continue;
        }
        const std::size_t linkIndex = findLink(profile, trial.linkId);
        const RagdollAxisDefinition3D& axis =
            profile.links[linkIndex].inboundJoint.axes[static_cast<std::size_t>(trial.axis)];
        if (trial.minimumObservedRadians < axis.minimumRadians - BoundaryToleranceRadians
            || trial.maximumObservedRadians > axis.maximumRadians + BoundaryToleranceRadians) {
            result.findings.push_back({ CalibrationFindingSeverity3D::Error, "joint_limit_escaped", path,
                "o eixo excedeu o limite configurado por mais de 2 graus" });
        }
        const float expectedTorqueLimit = axis.maximumTorque * settings.maximumTorqueScale;
        if (std::abs(trial.appliedMotorTorqueLimitNewtonMeters - expectedTorqueLimit)
            > std::max(0.1f, expectedTorqueLimit * 0.001f)) {
            result.findings.push_back({ CalibrationFindingSeverity3D::Error, "joint_torque_limit_not_applied", path,
                "o teto de torque publicado pelo backend diverge do pedido" });
        }
        const float progress = std::abs(trial.finalRadians) / std::max(0.001f, std::abs(trial.targetRadians));
        if (std::abs(trial.targetRadians) > 0.05f && progress < 0.25f) {
            result.findings.push_back({ CalibrationFindingSeverity3D::Warning, "joint_target_low_progress", path,
                "o eixo percorreu menos de 25% do alvo durante a janela" });
        }
    }
    result.findings.push_back({ CalibrationFindingSeverity3D::Information, "joint_semantics_not_certified", "trials",
        "o ensaio prova clamp e telemetria do solver, nao que o eixo represente "
        "o movimento anatomico declarado" });
    return result;
}

std::string renderRagdollJointLimitLabMarkdown3D(const RagdollJointLimitLabResult3D& result) {
    std::ostringstream output;
    output << "# Ensaio mecanico de limites `" << result.profileId << "`\n\n"
           << "> Cenas isoladas, sem gravidade e sem escrita de transform, a "
              "120 Hz. Clamp mecanico nao certifica semantica anatomica.\n\n"
           << "| Eixo | Extremo | Alvo | Final | Faixa observada | "
              "Motor/constraint pico | Limite aplicado |\n"
           << "|---|---|---:|---:|---:|---:|---:|\n";
    for (const auto& trial : result.trials)
        output << "| " << trial.linkId << "." << axisName(trial.axis) << " | " << endpointName(trial.endpoint) << " | "
               << fixed(trial.targetRadians * RadiansToDegrees) << " deg | "
               << fixed(trial.finalRadians * RadiansToDegrees) << " deg | "
               << fixed(trial.minimumObservedRadians * RadiansToDegrees) << "/"
               << fixed(trial.maximumObservedRadians * RadiansToDegrees) << " deg | "
               << fixed(trial.maximumMotorTorqueNewtonMeters) << "/" << fixed(trial.maximumConstraintTorqueNewtonMeters)
               << " N.m | " << fixed(trial.appliedMotorTorqueLimitNewtonMeters) << " N.m |\n";
    output << "\n## Achados\n\n| Severidade | Codigo | Caminho | Mensagem |\n"
           << "|---|---|---|---|\n";
    for (const auto& finding : result.findings)
        output << "| " << severityName(finding.severity) << " | `" << finding.code << "` | `" << finding.path << "` | "
               << finding.message << " |\n";
    return output.str();
}

std::string renderRagdollJointLimitLabPoseDump3D(
    const RagdollJointLimitLabResult3D& result, const RagdollProfile3D& profile) {
    validateRagdollProfileOrThrow3D(profile);
    if (result.profileId != profile.id)
        throw std::invalid_argument("resultado articular e perfil precisam ter o mesmo id");

    const auto writeFrame = [&](std::ostringstream& output, float timeSeconds, std::size_t trialIndex,
                                std::span<const PhysicsBodyState3D> states) {
        if (states.size() != profile.links.size())
            throw std::invalid_argument("captura articular nao contem todos os links do perfil");
        output << "F " << std::setprecision(4) << timeSeconds << ' ' << trialIndex << " 0\n";
        for (std::size_t index = 0; index < profile.links.size(); ++index) {
            const RagdollLinkDefinition3D& link = profile.links[index];
            const PhysicsBodyState3D& state = states[index];
            const Vec3 center = state.position + state.orientation.rotate(link.collider.localPosition);
            const Quaternion orientation = (state.orientation * link.collider.localOrientation).normalized();
            output << (link.collider.shape == RagdollColliderShape3D::Box ? "B " : "C ") << link.id << ' '
                   << std::setprecision(5) << center.x << ' ' << center.y << ' ' << center.z << ' ' << orientation.x
                   << ' ' << orientation.y << ' ' << orientation.z << ' ' << orientation.w;
            if (link.collider.shape == RagdollColliderShape3D::Box) {
                output << ' ' << link.collider.boxHalfExtents.x << ' ' << link.collider.boxHalfExtents.y << ' '
                       << link.collider.boxHalfExtents.z;
            } else {
                output << ' ' << ragdollCapsuleHalfSegment3D(link.collider) << ' ' << link.collider.radiusMeters << ' '
                       << ragdollCapsuleRadiusAtPositiveX3D(link.collider);
            }
            output << '\n';
        }
    };

    std::ostringstream output;
    output << std::fixed;
    for (std::size_t trialIndex = 0; trialIndex < result.trials.size(); ++trialIndex) {
        const RagdollJointLimitTrial3D& trial = result.trials[trialIndex];
        if (trial.initialLinks.empty() || trial.finalLinks.empty()) continue;
        output << "S RL_joint_" << trial.linkId << '_' << axisName(trial.axis) << '_' << endpointName(trial.endpoint)
               << '\n';
        writeFrame(output, 0.0f, trialIndex, trial.initialLinks);
        output << "E pose neutra\n";
        writeFrame(output, static_cast<float>(trial.stepCount) * ReinforcementLearningFixedStepSeconds, trialIndex,
            trial.finalLinks);
        output << "E alvo " << fixed(trial.targetRadians * RadiansToDegrees, 2) << " deg\n";
    }
    return output.str();
}

} // namespace MatterEngine
