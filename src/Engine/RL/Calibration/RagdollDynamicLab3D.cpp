#include "Engine/RL/Calibration/RagdollDynamicLab3D.hpp"

#include "Engine/Materials/MaterialLibrary.hpp"
#include "Engine/Physics/PhysicsEngine3D.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string_view>
#include <utility>

namespace MatterEngine {
namespace {

constexpr float MinimumDurationSeconds = ReinforcementLearningFixedStepSeconds;

struct TrialDefinition {
    std::string_view id;
    std::string_view description;
    float durationSeconds = 0.0f;
    float pelvisHeightMeters = 0.0f;
    float rigidityPercent = 0.0f;
    bool groundEnabled = false;
};

bool finite(float value) {
    return std::isfinite(value);
}

bool finite(Vec3 value) {
    return finite(value.x) && finite(value.y) && finite(value.z);
}

std::string fixed(float value, int precision = 4) {
    std::ostringstream output;
    output << std::fixed << std::setprecision(precision) << value;
    return output.str();
}

std::string severityName(CalibrationFindingSeverity3D severity) {
    switch (severity) {
        case CalibrationFindingSeverity3D::Information: return "informacao";
        case CalibrationFindingSeverity3D::Warning: return "aviso";
        case CalibrationFindingSeverity3D::Error: return "erro";
    }
    return "desconhecida";
}

void addFinding(RagdollDynamicLabResult3D& result, CalibrationFindingSeverity3D severity, std::string code,
    std::string path, std::string message) {
    result.findings.push_back({ severity, std::move(code), std::move(path), std::move(message) });
}

void validateSettings(const RagdollDynamicLabSettings3D& settings) {
    const auto validDuration = [](float value) { return std::isfinite(value) && value >= MinimumDurationSeconds; };
    if (!validDuration(settings.freeFallDurationSeconds) || !validDuration(settings.supportedDurationSeconds)
        || !validDuration(settings.dropDurationSeconds)) {
        throw std::invalid_argument("Duracoes do laboratorio dinamico devem ser positivas");
    }
    if (!finite(settings.supportedGroundClearanceMeters) || settings.supportedGroundClearanceMeters < 0.0f
        || !finite(settings.dropHeightMeters) || settings.dropHeightMeters <= 0.0f
        || !finite(settings.supportedRigidityPercent) || settings.supportedRigidityPercent < 0.0f
        || settings.supportedRigidityPercent > 100.0f) {
        throw std::invalid_argument("Configuracao invalida do laboratorio dinamico");
    }
}

PhysicsBodyHandle3D createGround(PhysicsScene3D& scene) {
    PhysicsShape3D shape;
    shape.type = PhysicsShapeType3D::Box;
    shape.halfExtents = { 20.0f, 20.0f, 0.25f };
    shape.materialId = "concrete";

    PhysicsBodyDefinition3D body;
    body.motionType = PhysicsMotionType3D::Static;
    body.position = { 0.0f, 0.0f, -0.25f };
    body.materialId = "concrete";
    return scene.createBody(body, std::span<const PhysicsShape3D>(&shape, 1));
}

std::size_t stepsFor(float durationSeconds) {
    return std::max<std::size_t>(
        1, static_cast<std::size_t>(std::ceil(durationSeconds / ReinforcementLearningFixedStepSeconds)));
}

Vec3 centerOfMassVelocity(const RagdollState3D& state, const RagdollEffectiveMass3D& massProperties) {
    Vec3 velocity;
    if (!massProperties.valid || state.links.size() != massProperties.links.size()) return velocity;
    for (std::size_t index = 0; index < state.links.size(); ++index) {
        velocity += state.links[index].linearVelocity * massProperties.links[index].massKg;
    }
    return velocity / massProperties.totalMassKg;
}

float maximumAnchorSeparation(const RagdollProfile3D& profile, const RagdollState3D& state) {
    float maximum = 0.0f;
    for (std::size_t index = 1; index < profile.links.size(); ++index) {
        const RagdollLinkDefinition3D& childDefinition = profile.links[index];
        if (childDefinition.parentIndex < 0) continue;
        const std::size_t parentIndex = static_cast<std::size_t>(childDefinition.parentIndex);
        if (parentIndex >= state.links.size() || index >= state.links.size()) continue;
        const RagdollLinkDefinition3D& parentDefinition = profile.links[parentIndex];
        const PhysicsBodyState3D& parent = state.links[parentIndex];
        const PhysicsBodyState3D& child = state.links[index];
        const Vec3 parentLocal = parentDefinition.modelOrientation.conjugate().rotate(
            childDefinition.inboundJoint.anchorModelPosition - parentDefinition.modelPosition);
        const Vec3 childLocal = childDefinition.modelOrientation.conjugate().rotate(
            childDefinition.inboundJoint.anchorModelPosition - childDefinition.modelPosition);
        const Vec3 parentAnchor = parent.position + parent.orientation.rotate(parentLocal);
        const Vec3 childAnchor = child.position + child.orientation.rotate(childLocal);
        maximum = std::max(maximum, (parentAnchor - childAnchor).length());
    }
    return maximum;
}

std::size_t findLink(const RagdollProfile3D& profile, std::string_view id) {
    for (std::size_t index = 0; index < profile.links.size(); ++index) {
        if (profile.links[index].id == id) return index;
    }
    return profile.links.size();
}

float bilateralPositionError(const RagdollProfile3D& profile, const RagdollState3D& state) {
    if (state.links.empty()) return 0.0f;
    const PhysicsBodyState3D& root = state.links.front();
    const Quaternion worldToRoot = root.orientation.conjugate();
    float maximum = 0.0f;
    for (std::size_t leftIndex = 0; leftIndex < profile.links.size(); ++leftIndex) {
        constexpr std::string_view LeftPrefix = "Left";
        const std::string& leftId = profile.links[leftIndex].id;
        if (!std::string_view(leftId).starts_with(LeftPrefix)) continue;
        const std::string rightId = "Right" + leftId.substr(LeftPrefix.size());
        const std::size_t rightIndex = findLink(profile, rightId);
        if (rightIndex >= state.links.size() || leftIndex >= state.links.size()) continue;
        Vec3 left = worldToRoot.rotate(state.links[leftIndex].position - root.position);
        const Vec3 right = worldToRoot.rotate(state.links[rightIndex].position - root.position);
        left.y = -left.y;
        maximum = std::max(maximum, (left - right).length());
    }
    return maximum;
}

bool finiteState(const RagdollState3D& state) {
    if (state.links.empty()) return false;
    for (const PhysicsBodyState3D& link : state.links) {
        if (!finite(link.position) || !finite(link.linearVelocity) || !finite(link.angularVelocity)
            || !finite(link.orientation.x) || !finite(link.orientation.y) || !finite(link.orientation.z)
            || !finite(link.orientation.w)) {
            return false;
        }
    }
    return true;
}

RagdollDynamicTrial3D runTrial(PhysicsEngine3D& engine, const MaterialLibrary& materials,
    const RagdollProfile3D& profile, const TrialDefinition& definition, std::size_t captureEverySteps,
    RagdollEffectiveMass3D* capturedMassProperties) {
    PhysicsSceneSettings3D sceneSettings;
    sceneSettings.gravity = { 0.0f, 0.0f, -9.81f };
    sceneSettings.workerThreadCount = 1;
    auto scene = engine.createScene(sceneSettings, materials);
    if (definition.groundEnabled) (void)createGround(*scene);

    RagdollSpawnDefinition3D spawn;
    spawn.entityId = 1;
    spawn.pelvisPosition = { 0.0f, 0.0f, definition.pelvisHeightMeters };
    spawn.rigidityPercent = definition.rigidityPercent;
    spawn.active = false;
    const RagdollHandle3D ragdoll = scene->createRagdoll(profile, spawn);

    const RagdollEffectiveMass3D massProperties = scene->ragdollEffectiveMass(ragdoll);
    if (capturedMassProperties && !capturedMassProperties->valid) *capturedMassProperties = massProperties;

    RagdollDynamicTrial3D trial;
    trial.id = definition.id;
    trial.description = definition.description;
    trial.stepCount = stepsFor(definition.durationSeconds);
    trial.durationSeconds = static_cast<float>(trial.stepCount) * ReinforcementLearningFixedStepSeconds;
    RagdollState3D state = scene->ragdollState(ragdoll);
    const auto capture = [&](float timeSeconds) {
        if (captureEverySteps == 0) return;
        RagdollDynamicFrame3D frame;
        frame.timeSeconds = timeSeconds;
        frame.links = state.links;
        frame.contacts = state.contacts;
        trial.capturedFrames.push_back(std::move(frame));
    };
    capture(0.0f);
    trial.initialCenterOfMassWorld = massProperties.centerOfMassWorld;
    trial.initialCenterOfMassVelocity = centerOfMassVelocity(state, massProperties);
    const float initialPelvisHeight = state.links.empty() ? 0.0f : state.links[0].position.z;

    double totalStepMilliseconds = 0.0;
    bool hadContact = false;
    for (std::size_t step = 0; step < trial.stepCount; ++step) {
        scene->simulate(ReinforcementLearningFixedStepSeconds);
        state = scene->ragdollState(ragdoll);
        trial.finite = trial.finite && finiteState(state);
        for (const PhysicsBodyState3D& link : state.links) {
            trial.maximumLinkSpeedMetersPerSecond =
                std::max(trial.maximumLinkSpeedMetersPerSecond, link.linearVelocity.length());
            trial.maximumLinkAngularSpeedRadiansPerSecond =
                std::max(trial.maximumLinkAngularSpeedRadiansPerSecond, link.angularVelocity.length());
        }
        trial.maximumJointAnchorSeparationMeters =
            std::max(trial.maximumJointAnchorSeparationMeters, maximumAnchorSeparation(profile, state));
        trial.maximumBilateralPositionErrorMeters =
            std::max(trial.maximumBilateralPositionErrorMeters, bilateralPositionError(profile, state));
        const bool hasContact = !state.contacts.empty();
        if (hasContact) {
            if (trial.firstContactSeconds < 0.0f) {
                trial.firstContactSeconds = static_cast<float>(step + 1) * ReinforcementLearningFixedStepSeconds;
            }
            ++trial.contactStepCount;
            trial.maximumContactPointCount = std::max(trial.maximumContactPointCount, state.contacts.size());
            for (const RagdollContactPoint3D& contact : state.contacts) {
                trial.maximumContactImpulseNewtonSeconds =
                    std::max(trial.maximumContactImpulseNewtonSeconds, contact.normalImpulseNewtonSeconds);
            }
        }
        const bool firstContact = hasContact && !hadContact;
        const bool periodicCapture = captureEverySteps != 0 && (step + 1) % captureEverySteps == 0;
        const bool finalCapture = step + 1 == trial.stepCount;
        if (captureEverySteps != 0 && (firstContact || periodicCapture || finalCapture)) {
            const float timeSeconds = static_cast<float>(step + 1) * ReinforcementLearningFixedStepSeconds;
            if (trial.capturedFrames.empty()
                || std::abs(trial.capturedFrames.back().timeSeconds - timeSeconds)
                    > 0.5f * ReinforcementLearningFixedStepSeconds) {
                capture(timeSeconds);
            }
        }
        hadContact = hasContact;
        const float stepMilliseconds = scene->diagnostics().totalStepMilliseconds;
        totalStepMilliseconds += stepMilliseconds;
        trial.maximumPhysicsStepMilliseconds = std::max(trial.maximumPhysicsStepMilliseconds, stepMilliseconds);
    }

    const RagdollEffectiveMass3D finalMassProperties = scene->ragdollEffectiveMass(ragdoll);
    trial.finalCenterOfMassWorld = finalMassProperties.centerOfMassWorld;
    trial.finalCenterOfMassVelocity = centerOfMassVelocity(state, finalMassProperties);
    trial.measuredVerticalAccelerationMetersPerSecondSquared =
        (trial.finalCenterOfMassVelocity.z - trial.initialCenterOfMassVelocity.z) / trial.durationSeconds;
    const Vec3 horizontalDrift = trial.finalCenterOfMassWorld - trial.initialCenterOfMassWorld;
    trial.horizontalCenterOfMassDriftMeters =
        std::sqrt(horizontalDrift.x * horizontalDrift.x + horizontalDrift.y * horizontalDrift.y);
    trial.pelvisHeightLossMeters = state.links.empty() ? 0.0f : initialPelvisHeight - state.links[0].position.z;
    trial.finalSleeping = state.sleeping;
    trial.averagePhysicsStepMilliseconds =
        static_cast<float>(totalStepMilliseconds / static_cast<double>(trial.stepCount));
    trial.finite = trial.finite && finalMassProperties.valid && finite(trial.finalCenterOfMassWorld)
        && finite(trial.finalCenterOfMassVelocity);
    return trial;
}

const RagdollDynamicTrial3D* findTrial(const RagdollDynamicLabResult3D& result, std::string_view id) {
    const auto found = std::find_if(result.trials.begin(), result.trials.end(),
        [id](const RagdollDynamicTrial3D& trial) { return trial.id == id; });
    return found == result.trials.end() ? nullptr : &*found;
}

} // namespace

std::size_t RagdollDynamicLabResult3D::findingCount(CalibrationFindingSeverity3D severity) const {
    return static_cast<std::size_t>(std::count_if(findings.begin(), findings.end(),
        [severity](const CalibrationFinding3D& finding) { return finding.severity == severity; }));
}

bool RagdollDynamicLabResult3D::hasErrors() const {
    return findingCount(CalibrationFindingSeverity3D::Error) > 0;
}

RagdollEffectiveMass3D measureRagdollEffectiveMass3D(const RagdollProfile3D& profile) {
    validateRagdollProfileOrThrow3D(profile);
    PhysicsEngine3D engine;
    MaterialLibrary materials;
    PhysicsSceneSettings3D sceneSettings;
    sceneSettings.gravity = {};
    sceneSettings.workerThreadCount = 1;
    auto scene = engine.createScene(sceneSettings, materials);

    RagdollSpawnDefinition3D spawn;
    spawn.entityId = 1;
    spawn.pelvisPosition = { 0.0f, 0.0f, profile.standingRootHeightMeters };
    spawn.rigidityPercent = 0.0f;
    spawn.active = false;
    const RagdollHandle3D ragdoll = scene->createRagdoll(profile, spawn);
    return scene->ragdollEffectiveMass(ragdoll);
}

RagdollDynamicLabResult3D runRagdollDynamicLab3D(
    const RagdollProfile3D& profile, const RagdollDynamicLabSettings3D& settings) {
    validateRagdollProfileOrThrow3D(profile);
    validateSettings(settings);

    RagdollDynamicLabResult3D result;
    result.profileId = profile.id;
    PhysicsEngine3D engine;
    MaterialLibrary materials;

    const TrialDefinition freeFall { "free_fall",
        "Queda livre sem piso e sem motores; valida "
        "gravidade, massa agregada e deriva.",
        settings.freeFallDurationSeconds, 3.0f, 0.0f, false };
    const TrialDefinition supported { "supported_stance",
        "Pose neutra sobre piso com a rigidez passiva atual no maximo.", settings.supportedDurationSeconds,
        profile.standingRootHeightMeters + settings.supportedGroundClearanceMeters, settings.supportedRigidityPercent,
        true };
    const TrialDefinition drop { "passive_drop",
        "Queda sobre piso sem motores; mede impacto, "
        "coesao e acomodacao passiva.",
        settings.dropDurationSeconds, profile.standingRootHeightMeters + settings.dropHeightMeters, 0.0f, true };

    result.trials.push_back(
        runTrial(engine, materials, profile, freeFall, settings.captureEverySteps, &result.effectiveMassProperties));
    result.trials.push_back(runTrial(engine, materials, profile, supported, settings.captureEverySteps, nullptr));
    result.trials.push_back(runTrial(engine, materials, profile, drop, settings.captureEverySteps, nullptr));

    if (!result.effectiveMassProperties.valid || result.effectiveMassProperties.links.size() != profile.links.size()) {
        addFinding(result, CalibrationFindingSeverity3D::Error, "effective_mass_properties_unavailable",
            "mass_properties", "o backend nao publicou mass properties efetivas para todos os links");
    } else if (std::abs(result.effectiveMassProperties.totalMassKg - profile.totalMassKg) > 0.01f) {
        addFinding(result, CalibrationFindingSeverity3D::Error, "effective_mass_mismatch",
            "mass_properties.totalMassKg", "a massa efetiva do backend diverge da massa total declarada");
    }

    std::size_t isotropicInertiaCount = 0;
    std::size_t conditionedInertiaCount = 0;
    float maximumConditioningRatio = 1.0f;
    std::string maximumConditioningLink;
    for (const RagdollLinkEffectiveMass3D& link : result.effectiveMassProperties.links) {
        const Vec3 inertia = link.principalInertiaKgSquareMeters;
        const float largest = std::max({ inertia.x, inertia.y, inertia.z });
        const float smallest = std::min({ inertia.x, inertia.y, inertia.z });
        if (largest > 0.0f && (largest - smallest) / largest < 0.001f) ++isotropicInertiaCount;
        if (link.maximumInertiaConditioningRatio > 1.001f) ++conditionedInertiaCount;
        if (link.maximumInertiaConditioningRatio > maximumConditioningRatio) {
            maximumConditioningRatio = link.maximumInertiaConditioningRatio;
            maximumConditioningLink = link.id;
        }
    }
    if (conditionedInertiaCount > 0) {
        addFinding(result, CalibrationFindingSeverity3D::Information, "jolt_inertia_conditioning",
            "mass_properties.links",
            std::to_string(conditionedInertiaCount) + " links tiveram a inercia elevada pelo Stabilize; maior fator "
                + fixed(maximumConditioningRatio, 2) + "x em " + maximumConditioningLink);
    }
    if (isotropicInertiaCount > 0) {
        addFinding(result, CalibrationFindingSeverity3D::Information, "isotropic_effective_inertia",
            "mass_properties.links",
            std::to_string(isotropicInertiaCount)
                + " links ficaram com momentos principais praticamente iguais; "
                  "comparar a inercia anterior e posterior ao condicionamento do "
                  "Jolt");
    }

    for (const RagdollDynamicTrial3D& trial : result.trials) {
        if (!trial.finite) {
            addFinding(result, CalibrationFindingSeverity3D::Error, "non_finite_dynamics", "trials." + trial.id,
                "o ensaio produziu estado dinamico nao finito");
        }
        if (trial.maximumJointAnchorSeparationMeters > 0.03f) {
            addFinding(result, CalibrationFindingSeverity3D::Warning, "joint_anchor_separation", "trials." + trial.id,
                "a separacao maxima de ancora excedeu o limite diagnostico de 3 cm");
        }
    }

    if (const RagdollDynamicTrial3D* trial = findTrial(result, "free_fall")) {
        if (std::abs(trial->measuredVerticalAccelerationMetersPerSecondSquared + 9.81f) > 0.35f) {
            addFinding(result, CalibrationFindingSeverity3D::Warning, "unexpected_free_fall_acceleration",
                "trials.free_fall",
                "a aceleracao vertical agregada diverge mais de 0,35 m/s2 da "
                "gravidade");
        }
        if (trial->horizontalCenterOfMassDriftMeters > 0.02f) {
            addFinding(result, CalibrationFindingSeverity3D::Warning, "free_fall_horizontal_drift", "trials.free_fall",
                "o COM derivou mais de 2 cm sem forca horizontal externa");
        }
    }
    if (const RagdollDynamicTrial3D* trial = findTrial(result, "supported_stance")) {
        if (trial->firstContactSeconds < 0.0f) {
            addFinding(result, CalibrationFindingSeverity3D::Error, "supported_stance_without_contact",
                "trials.supported_stance", "a pose apoiada nao produziu contato com o piso");
        }
        if (trial->pelvisHeightLossMeters > 0.15f) {
            addFinding(result, CalibrationFindingSeverity3D::Warning, "passive_stance_collapsed",
                "trials.supported_stance",
                "a rigidez passiva maxima nao preservou a altura neutra da pelve; "
                "isto ainda nao avalia uma politica de equilibrio");
        }
    }
    if (const RagdollDynamicTrial3D* trial = findTrial(result, "passive_drop")) {
        if (trial->firstContactSeconds < 0.0f) {
            addFinding(result, CalibrationFindingSeverity3D::Error, "drop_without_contact", "trials.passive_drop",
                "a queda nao alcancou o piso durante a janela do ensaio");
        }
        if (!trial->finalSleeping) {
            addFinding(result, CalibrationFindingSeverity3D::Information, "drop_not_sleeping", "trials.passive_drop",
                "o corpo ainda estava acordado ao fim da janela; isso nao "
                "implica instabilidade");
        }
    }
    addFinding(result, CalibrationFindingSeverity3D::Information, "contact_impulse_is_estimated", "trials[*].contacts",
        "no backend atual o impulso dos contatos do ragdoll e estimado "
        "antes do solver; "
        "nao deve ser tratado como carga plantar medida");
    addFinding(result, CalibrationFindingSeverity3D::Information, "biomechanics_not_certified", "trials",
        "os limites sao gates numericos preliminares e nao certificam "
        "antropometria, "
        "forca muscular ou equilibrio humano");
    return result;
}

std::string renderRagdollDynamicLabMarkdown3D(const RagdollDynamicLabResult3D& result) {
    std::ostringstream output;
    output << "# Laboratorio dinamico do ragdoll `" << result.profileId << "`\n\n"
           << "> Ensaio headless do corpo realmente instanciado no backend. "
              "Os limites atuais sao diagnosticos, nao certificacao biomecanica.\n\n"
           << "## Resumo\n\n"
           << "- Passo fixo: " << fixed(result.fixedStepSeconds * 1000.0f, 3) << " ms (120 Hz)\n"
           << "- Mass properties efetivas: **" << (result.effectiveMassProperties.valid ? "VALIDAS" : "INDISPONIVEIS")
           << "**\n"
           << "- Massa efetiva total: " << fixed(result.effectiveMassProperties.totalMassKg, 4) << " kg\n"
           << "- Ensaios: " << result.trials.size() << "\n"
           << "- Achados: " << result.findingCount(CalibrationFindingSeverity3D::Error) << " erros, "
           << result.findingCount(CalibrationFindingSeverity3D::Warning) << " avisos, "
           << result.findingCount(CalibrationFindingSeverity3D::Information) << " informacoes\n\n";

    output << "## Inercia antes e depois do condicionamento\n\n"
           << "| Link | Massa | COM local | Antes do Stabilize | Efetiva | Maior "
              "fator |\n"
           << "|---|---:|---|---|---|---:|\n";
    for (const RagdollLinkEffectiveMass3D& link : result.effectiveMassProperties.links) {
        output << "| " << link.id << " | " << fixed(link.massKg, 4) << " kg | `" << fixed(link.centerOfMassLocal.x)
               << ", " << fixed(link.centerOfMassLocal.y) << ", " << fixed(link.centerOfMassLocal.z) << "` m | `"
               << fixed(link.unconditionedPrincipalInertiaKgSquareMeters.x, 6) << ", "
               << fixed(link.unconditionedPrincipalInertiaKgSquareMeters.y, 6) << ", "
               << fixed(link.unconditionedPrincipalInertiaKgSquareMeters.z, 6) << "` kg.m2 | `"
               << fixed(link.principalInertiaKgSquareMeters.x, 6) << ", "
               << fixed(link.principalInertiaKgSquareMeters.y, 6) << ", "
               << fixed(link.principalInertiaKgSquareMeters.z, 6) << "` kg.m2 | "
               << fixed(link.maximumInertiaConditioningRatio, 2) << "x |\n";
    }

    output << "\n## Ensaios\n\n"
           << "| Ensaio | Duracao | Contato inicial | Perda da pelve | "
              "Deriva horizontal | Vel. max. | Giro max. | Gap de junta | "
              "Assimetria max. | Passo medio/max. |\n"
           << "|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|\n";
    for (const RagdollDynamicTrial3D& trial : result.trials) {
        output << "| `" << trial.id << "` | " << fixed(trial.durationSeconds, 3) << " s | ";
        if (trial.firstContactSeconds < 0.0f)
            output << "-";
        else
            output << fixed(trial.firstContactSeconds, 3) << " s";
        output << " | " << fixed(trial.pelvisHeightLossMeters * 100.0f, 2) << " cm | "
               << fixed(trial.horizontalCenterOfMassDriftMeters * 100.0f, 2) << " cm | "
               << fixed(trial.maximumLinkSpeedMetersPerSecond, 2) << " m/s | "
               << fixed(trial.maximumLinkAngularSpeedRadiansPerSecond, 2) << " rad/s | "
               << fixed(trial.maximumJointAnchorSeparationMeters * 1000.0f, 2) << " mm | "
               << fixed(trial.maximumBilateralPositionErrorMeters * 1000.0f, 2) << " mm | "
               << fixed(trial.averagePhysicsStepMilliseconds, 3) << "/"
               << fixed(trial.maximumPhysicsStepMilliseconds, 3) << " ms |\n";
    }

    if (const RagdollDynamicTrial3D* freeFall = findTrial(result, "free_fall")) {
        output << "\nA aceleracao vertical agregada na queda livre foi `"
               << fixed(freeFall->measuredVerticalAccelerationMetersPerSecondSquared, 4)
               << " m/s2`. O amortecimento linear do corpo permanece ativo neste "
                  "ensaio.\n";
    }

    output << "\n### Telemetria complementar\n\n";
    for (const RagdollDynamicTrial3D& trial : result.trials) {
        output << "- `" << trial.id << "`: " << trial.contactStepCount << " passos com contato; pico de "
               << fixed(trial.maximumContactPointCount, 0) << " pontos; impulso normal estimado maximo de "
               << fixed(trial.maximumContactImpulseNewtonSeconds, 3) << " N.s; velocidade final do COM `"
               << fixed(trial.finalCenterOfMassVelocity.x, 3) << ", " << fixed(trial.finalCenterOfMassVelocity.y, 3)
               << ", " << fixed(trial.finalCenterOfMassVelocity.z, 3) << "` m/s; sleep "
               << (trial.finalSleeping ? "sim" : "nao") << ".\n";
    }

    output << "\n## Achados\n\n"
           << "| Severidade | Codigo | Caminho | Mensagem |\n"
           << "|---|---|---|---|\n";
    for (const CalibrationFinding3D& finding : result.findings) {
        output << "| " << severityName(finding.severity) << " | `" << finding.code << "` | `" << finding.path << "` | "
               << finding.message << " |\n";
    }
    return output.str();
}

std::string renderRagdollDynamicLabPoseDump3D(
    const RagdollDynamicLabResult3D& result, const RagdollProfile3D& profile) {
    validateRagdollProfileOrThrow3D(profile);
    if (result.profileId != profile.id)
        throw std::invalid_argument("resultado dinamico e perfil precisam ter o mesmo id");

    std::ostringstream output;
    output << std::fixed;
    for (std::size_t trialIndex = 0; trialIndex < result.trials.size(); ++trialIndex) {
        const RagdollDynamicTrial3D& trial = result.trials[trialIndex];
        if (trial.capturedFrames.empty()) continue;
        output << "S RL " << trial.id << '\n';
        bool previousContact = false;
        for (const RagdollDynamicFrame3D& frame : trial.capturedFrames) {
            if (frame.links.size() != profile.links.size())
                throw std::invalid_argument("captura dinamica nao contem todos os links do perfil");
            output << "F " << std::setprecision(4) << frame.timeSeconds << ' ' << trialIndex << " 0\n";
            for (std::size_t index = 0; index < profile.links.size(); ++index) {
                const RagdollLinkDefinition3D& link = profile.links[index];
                const PhysicsBodyState3D& state = frame.links[index];
                const Vec3 center = state.position + state.orientation.rotate(link.collider.localPosition);
                const Quaternion orientation = (state.orientation * link.collider.localOrientation).normalized();
                output << (ragdollColliderHasBox3D(link.collider) ? "B " : "C ") << link.id << ' '
                       << std::setprecision(5) << center.x << ' ' << center.y << ' ' << center.z << ' ' << orientation.x
                       << ' ' << orientation.y << ' ' << orientation.z << ' ' << orientation.w;
                if (ragdollColliderHasBox3D(link.collider)) {
                    output << ' ' << link.collider.boxHalfExtents.x << ' ' << link.collider.boxHalfExtents.y << ' '
                           << link.collider.boxHalfExtents.z;
                } else {
                    output << ' ' << ragdollCapsuleHalfSegment3D(link.collider) << ' ' << link.collider.radiusMeters
                           << ' ' << ragdollCapsuleRadiusAtPositiveX3D(link.collider);
                }
                output << '\n';
            }
            for (const RagdollContactPoint3D& contact : frame.contacts) {
                output << "P " << std::setprecision(5) << contact.position.x << ' ' << contact.position.y << ' '
                       << contact.position.z << ' ' << contact.linkIndex << ' ' << contact.normal.x << ' '
                       << contact.normal.y << ' ' << contact.normal.z << ' ' << contact.normalImpulseNewtonSeconds
                       << '\n';
            }
            const bool hasContact = !frame.contacts.empty();
            if (hasContact && !previousContact) output << "E primeiro contato\n";
            previousContact = hasContact;
        }
    }
    return output.str();
}

} // namespace MatterEngine
