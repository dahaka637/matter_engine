#include "Engine/RL/Calibration/RagdollJointLimitLab3D.hpp"

#include <exception>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <string_view>

// Exercita os dois extremos de cada eixo ativo do perfil no solver, a 120 Hz.
// --dump grava as poses neutra/final para o MatterModelInspector quadros.
namespace {

void usage() {
    std::cout << "Uso: MatterRLJointProbe [--profile <json>] [--quick] [--strict] [--dump <arquivo>]\n";
}

} // namespace

int main(int argc, char** argv) {
    try {
        std::string profilePath =
            std::string(MATTERENGINE_ASSETS_DIR) + "/characters/football_player/FootballPlayerV1.ragdoll.json";
        MatterEngine::RagdollJointLimitLabSettings3D settings;
        bool strict = false;
        std::string dumpPath;
        for (int index = 1; index < argc; ++index) {
            const std::string_view argument = argv[index];
            const auto next = [&](std::string_view option) -> std::string {
                if (++index >= argc) throw std::invalid_argument(std::string(option) + " exige um arquivo");
                return argv[index];
            };
            if (argument == "--help" || argument == "-h") {
                usage();
                return 0;
            }
            if (argument == "--profile") profilePath = next(argument);
            else if (argument == "--quick") settings.durationSeconds = 0.35f;
            else if (argument == "--strict") strict = true;
            else if (argument == "--dump") dumpPath = next(argument);
            else throw std::invalid_argument("argumento desconhecido: " + std::string(argument));
        }
        const MatterEngine::RagdollProfile3D profile = MatterEngine::loadRagdollProfile3D(profilePath);
        const MatterEngine::RagdollJointLimitLabResult3D result =
            MatterEngine::runRagdollJointLimitLab3D(profile, settings);
        std::cout << MatterEngine::renderRagdollJointLimitLabMarkdown3D(result);
        if (!dumpPath.empty()) {
            std::ofstream dump(dumpPath, std::ios::binary);
            if (!dump) throw std::runtime_error("nao foi possivel abrir o dump: " + dumpPath);
            dump << MatterEngine::renderRagdollJointLimitLabPoseDump3D(result, profile);
            if (!dump) throw std::runtime_error("falha ao escrever o dump: " + dumpPath);
            std::cout << "\nCaptura articular: " << dumpPath << '\n';
        }
        if (result.hasErrors()) return 2;
        if (strict && result.findingCount(MatterEngine::CalibrationFindingSeverity3D::Warning) > 0) return 3;
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "MatterRLJointProbe: " << error.what() << '\n';
        return 1;
    }
}
