#include "Engine/RL/Calibration/RagdollDynamicLab3D.hpp"

#include <exception>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <string_view>

namespace {

void printUsage() {
    std::cout << "Uso: MatterRLDynamics [--profile <arquivo>] [--quick] [--strict]\n"
              << "                        [--dump <arquivo>] [--capture-every "
                 "<passos>]\n"
              << "\n"
              << "Executa queda livre, sustentacao e queda com impacto a 120 Hz.\n"
              << "--quick reduz as janelas para uma verificacao curta.\n"
              << "--strict tambem retorna falha quando existem avisos.\n"
              << "--dump grava poses/contatos para MatterModelInspector quadros.\n";
}

} // namespace

int main(int argc, char** argv) {
    try {
        std::string profilePath =
            std::string(MATTERENGINE_ASSETS_DIR) + "/characters/football_player/FootballPlayerV1.ragdoll.json";
        MatterEngine::RagdollDynamicLabSettings3D settings;
        bool strict = false;
        std::string dumpPath;
        for (int index = 1; index < argc; ++index) {
            const std::string_view argument = argv[index];
            if (argument == "--help" || argument == "-h") {
                printUsage();
                return 0;
            }
            if (argument == "--strict") {
                strict = true;
                continue;
            }
            if (argument == "--quick") {
                settings.freeFallDurationSeconds = 0.25f;
                settings.supportedDurationSeconds = 1.0f;
                settings.dropDurationSeconds = 1.5f;
                continue;
            }
            if (argument == "--profile") {
                if (++index >= argc) throw std::invalid_argument("--profile exige o caminho de um arquivo");
                profilePath = argv[index];
                continue;
            }
            if (argument == "--dump") {
                if (++index >= argc) throw std::invalid_argument("--dump exige o caminho de um arquivo");
                dumpPath = argv[index];
                if (settings.captureEverySteps == 0) settings.captureEverySteps = 6;
                continue;
            }
            if (argument == "--capture-every") {
                if (++index >= argc) throw std::invalid_argument("--capture-every exige um inteiro positivo");
                const int value = std::stoi(argv[index]);
                if (value <= 0) throw std::invalid_argument("--capture-every exige um inteiro positivo");
                settings.captureEverySteps = static_cast<std::size_t>(value);
                continue;
            }
            throw std::invalid_argument("argumento desconhecido: " + std::string(argument));
        }

        const MatterEngine::RagdollProfile3D profile = MatterEngine::loadRagdollProfile3D(profilePath);
        const MatterEngine::RagdollDynamicLabResult3D result = MatterEngine::runRagdollDynamicLab3D(profile, settings);
        std::cout << MatterEngine::renderRagdollDynamicLabMarkdown3D(result);
        if (!dumpPath.empty()) {
            std::ofstream dump(dumpPath, std::ios::binary);
            if (!dump) throw std::runtime_error("nao foi possivel abrir o dump: " + dumpPath);
            dump << MatterEngine::renderRagdollDynamicLabPoseDump3D(result, profile);
            if (!dump) throw std::runtime_error("falha ao escrever o dump: " + dumpPath);
            std::cout << "\nCaptura dinamica: " << dumpPath << '\n';
        }
        if (result.hasErrors()) return 2;
        if (strict && result.findingCount(MatterEngine::CalibrationFindingSeverity3D::Warning) > 0) {
            return 3;
        }
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "MatterRLDynamics: " << error.what() << '\n';
        return 1;
    }
}
