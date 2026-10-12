#include "Engine/RL/Calibration/RagdollJointLimitLab3D.hpp"

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

// Cada eixo ativo do perfil, nos dois extremos: o solver respeita o limite,
// publica o alvo e o teto de torque, e a telemetria fica finita.
void testAllJointExtremes() {
    const RagdollProfile3D profile = loadRagdollProfile3D(
        std::string(MATTERENGINE_TEST_ASSETS_DIR) + "/characters/football_player/FootballPlayerV1.ragdoll.json");
    std::size_t axes = 0;
    for (std::size_t i = 1; i < profile.links.size(); ++i)
        for (const RagdollAxisDefinition3D& axis : profile.links[i].inboundJoint.axes) axes += axis.enabled ? 1 : 0;
    RagdollJointLimitLabSettings3D settings;
    settings.durationSeconds = 0.35f;
    const RagdollJointLimitLabResult3D result = runRagdollJointLimitLab3D(profile, settings);
    require(!result.hasErrors(), "Ensaio articular encontrou erro mecanico");
    require(result.trials.size() == axes * 2, "Cada eixo ativo deveria gerar dois extremos");
    for (const RagdollJointLimitTrial3D& trial : result.trials) {
        require(trial.finite && trial.targetApplied, "Ensaio nao publicou estado/alvo valido");
        require(trial.appliedMotorTorqueLimitNewtonMeters > 0.0f, "Backend nao publicou teto de torque");
        require(std::isfinite(trial.finalRadians) && std::isfinite(trial.maximumMotorTorqueNewtonMeters),
            "Telemetria articular nao finita");
    }
    const std::string markdown = renderRagdollJointLimitLabMarkdown3D(result);
    require(markdown.find("120 Hz") != std::string::npos, "Relatorio omitiu a frequencia");
    const std::string dump = renderRagdollJointLimitLabPoseDump3D(result, profile);
    require(dump.find("S RL_joint_LeftForearm_twist_") != std::string::npos, "Dump omitiu o cenario do cotovelo");
    require(dump.find("E pose neutra") != std::string::npos, "Dump omitiu as legendas das poses");
    require(dump.find("\nC LeftForearm ") != std::string::npos, "Dump omitiu os links fisicos");
}

} // namespace

int main() {
    try {
        testAllJointExtremes();
        std::cout << "Ragdoll joint limit lab tests passed\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "Ragdoll joint limit lab tests failed: " << error.what() << '\n';
        return 1;
    }
}
