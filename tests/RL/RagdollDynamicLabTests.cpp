#include "Engine/RL/Calibration/RagdollDynamicLab3D.hpp"

#include <cmath>
#include <exception>
#include <iostream>
#include <stdexcept>
#include <string>

namespace {

using namespace MatterEngine;

void require(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}

RagdollProfile3D loadFootballPlayer() {
    return loadRagdollProfile3D(
        std::string(MATTERENGINE_TEST_ASSETS_DIR) + "/characters/football_player/FootballPlayerV1.ragdoll.json");
}

const RagdollDynamicTrial3D& trialById(const RagdollDynamicLabResult3D& result, const std::string& id) {
    for (const RagdollDynamicTrial3D& trial : result.trials) {
        if (trial.id == id) return trial;
    }
    throw std::runtime_error("Ensaio ausente: " + id);
}

void testEffectiveMassPropertiesAndDynamicScenarios() {
    RagdollDynamicLabSettings3D settings;
    settings.freeFallDurationSeconds = 0.25f;
    settings.supportedDurationSeconds = 1.0f;
    settings.dropDurationSeconds = 1.5f;
    settings.dropHeightMeters = 0.20f;
    const RagdollProfile3D profile = loadFootballPlayer();
    const RagdollDynamicLabResult3D result = runRagdollDynamicLab3D(profile, settings);

    require(!result.hasErrors(), "O laboratorio dinamico encontrou erro estrutural");
    require(std::abs(result.fixedStepSeconds - 1.0f / 120.0f) < 0.000001f, "O laboratorio deixou de usar 120 Hz");
    require(result.effectiveMassProperties.valid, "Mass properties efetivas indisponiveis");
    require(result.effectiveMassProperties.links.size() == profile.links.size(), "Mass properties perderam links");
    require(std::abs(result.effectiveMassProperties.totalMassKg - 80.0f) < 0.01f,
        "O Jolt nao preservou os 80 kg declarados");
    bool observedInertiaConditioning = false;
    for (std::size_t index = 0; index < result.effectiveMassProperties.links.size(); ++index) {
        const RagdollLinkEffectiveMass3D& link = result.effectiveMassProperties.links[index];
        require(link.id == profile.links[index].id, "Ordem dos links mudou");
        require(link.massKg > 0.0f, "Link com massa efetiva invalida");
        require(link.principalInertiaKgSquareMeters.x > 0.0f && link.principalInertiaKgSquareMeters.y > 0.0f
                && link.principalInertiaKgSquareMeters.z > 0.0f,
            "Link com momento principal de inercia invalido");
        require(link.unconditionedPrincipalInertiaKgSquareMeters.x > 0.0f
                && link.unconditionedPrincipalInertiaKgSquareMeters.y > 0.0f
                && link.unconditionedPrincipalInertiaKgSquareMeters.z > 0.0f,
            "Link com inercia anterior ao Stabilize invalida");
        require(std::isfinite(link.maximumInertiaConditioningRatio) && link.maximumInertiaConditioningRatio >= 0.999f,
            "Fator de condicionamento de inercia invalido");
        observedInertiaConditioning = observedInertiaConditioning || link.maximumInertiaConditioningRatio > 1.01f;
        require((link.centerOfMassLocal - profile.links[index].centerOfMassLocal).length() < 0.00001f,
            "O COM local efetivo diverge do perfil");
    }
    require(observedInertiaConditioning, "O teste deixou de exercitar o condicionamento de inercia do Jolt");

    const RagdollDynamicTrial3D& freeFall = trialById(result, "free_fall");
    require(freeFall.finite, "Queda livre produziu estado nao finito");
    require(freeFall.measuredVerticalAccelerationMetersPerSecondSquared < -9.0f
            && freeFall.measuredVerticalAccelerationMetersPerSecondSquared > -10.1f,
        "Queda livre nao respondeu a gravidade esperada");
    require(freeFall.horizontalCenterOfMassDriftMeters < 0.02f, "Queda livre derivou horizontalmente");

    const RagdollDynamicTrial3D& supported = trialById(result, "supported_stance");
    const RagdollDynamicTrial3D& drop = trialById(result, "passive_drop");
    require(supported.finite && drop.finite, "Ensaio com piso produziu estado nao finito");
    require(supported.firstContactSeconds >= 0.0f, "Sustentacao nao tocou o piso");
    require(drop.firstContactSeconds >= 0.0f, "Queda passiva nao tocou o piso");
    require(freeFall.maximumJointAnchorSeparationMeters < 0.03f && supported.maximumJointAnchorSeparationMeters < 0.03f
            && drop.maximumJointAnchorSeparationMeters < 0.03f,
        "O solver perdeu coesao das juntas");
}

void testMarkdownReport() {
    RagdollDynamicLabSettings3D settings;
    settings.freeFallDurationSeconds = 0.10f;
    settings.supportedDurationSeconds = 0.20f;
    settings.dropDurationSeconds = 0.40f;
    settings.dropHeightMeters = 0.10f;
    const RagdollDynamicLabResult3D result = runRagdollDynamicLab3D(loadFootballPlayer(), settings);
    const std::string markdown = renderRagdollDynamicLabMarkdown3D(result);
    require(markdown.find("Inercia antes e depois") != std::string::npos, "Relatorio omitiu inercia efetiva");
    require(markdown.find("free_fall") != std::string::npos, "Relatorio omitiu queda livre");
    require(markdown.find("120 Hz") != std::string::npos, "Relatorio omitiu a frequencia fixa");
}

void testContactPoseCapture() {
    RagdollDynamicLabSettings3D settings;
    settings.freeFallDurationSeconds = 0.10f;
    settings.supportedDurationSeconds = 0.25f;
    settings.dropDurationSeconds = 0.60f;
    settings.dropHeightMeters = 0.10f;
    settings.captureEverySteps = 7;
    const RagdollProfile3D profile = loadFootballPlayer();
    const RagdollDynamicLabResult3D result = runRagdollDynamicLab3D(profile, settings);

    for (const RagdollDynamicTrial3D& trial : result.trials) {
        require(!trial.capturedFrames.empty(), "Captura dinamica omitiu um ensaio");
        require(
            std::abs(trial.capturedFrames.front().timeSeconds) < 0.000001f, "Captura dinamica omitiu o estado inicial");
        require(std::abs(trial.capturedFrames.back().timeSeconds - trial.durationSeconds) < 0.00001f,
            "Captura dinamica omitiu o estado final");
        for (const RagdollDynamicFrame3D& frame : trial.capturedFrames)
            require(frame.links.size() == profile.links.size(), "Quadro capturado perdeu links");
    }
    const RagdollDynamicTrial3D& drop = trialById(result, "passive_drop");
    bool capturedContact = false;
    for (const RagdollDynamicFrame3D& frame : drop.capturedFrames)
        capturedContact = capturedContact || !frame.contacts.empty();
    require(capturedContact, "Captura pulou o primeiro contato da queda");

    const std::string dump = renderRagdollDynamicLabPoseDump3D(result, profile);
    require(dump.find("S RL passive_drop") != std::string::npos, "Dump omitiu o ensaio de queda");
    require(dump.find("\nP ") != std::string::npos, "Dump omitiu os pontos de contato");
    require(dump.find("E primeiro contato") != std::string::npos, "Dump omitiu a marcacao do primeiro contato");
}

} // namespace

int main() {
    try {
        testEffectiveMassPropertiesAndDynamicScenarios();
        testMarkdownReport();
        testContactPoseCapture();
        std::cout << "Ragdoll dynamic lab tests passed\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "Ragdoll dynamic lab tests failed: " << error.what() << '\n';
        return 1;
    }
}
