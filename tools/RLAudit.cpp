#include "Engine/RL/Calibration/RagdollCalibrationAudit3D.hpp"
#include "Engine/RL/Calibration/RagdollDynamicLab3D.hpp"

#include <exception>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>

// Auditoria do corpo de treino em Markdown no stdout: a estrutura do perfil
// (massas, juntas, tetos, simetria) e as massas/inercias que o backend
// realmente aplicou. Retorna 2 com erro no perfil e 3 com aviso em --strict.
namespace {

void printUsage() {
    std::cout << "Uso: MatterRLAudit [--profile <arquivo>] [--strict]\n";
}

std::string effectiveMassMarkdown(const MatterEngine::RagdollEffectiveMass3D& mass) {
    std::ostringstream out;
    out << "## Massa e inercia efetivas (depois do Stabilize do backend)\n\n"
        << "- Massa total: " << mass.totalMassKg << " kg\n\n"
        << "| Link | Massa (kg) | Momentos da shape (kg.m2) | Momentos efetivos (kg.m2) | Fator |\n"
        << "|---|---:|---|---|---:|\n";
    for (const MatterEngine::RagdollLinkEffectiveMass3D& link : mass.links) {
        const auto& a = link.unconditionedPrincipalInertiaKgSquareMeters;
        const auto& b = link.principalInertiaKgSquareMeters;
        out << "| " << link.id << " | " << link.massKg << " | " << a.x << ", " << a.y << ", " << a.z << " | " << b.x
            << ", " << b.y << ", " << b.z << " | " << link.maximumInertiaConditioningRatio << " |\n";
    }
    return out.str();
}

} // namespace

int main(int argc, char** argv) {
    try {
        std::string profilePath =
            std::string(MATTERENGINE_ASSETS_DIR) + "/characters/football_player/FootballPlayerV1.ragdoll.json";
        bool strict = false;
        for (int index = 1; index < argc; ++index) {
            const std::string_view argument = argv[index];
            if (argument == "--help" || argument == "-h") {
                printUsage();
                return 0;
            }
            if (argument == "--profile" && index + 1 < argc) {
                profilePath = argv[++index];
                continue;
            }
            if (argument == "--strict") {
                strict = true;
                continue;
            }
            throw std::invalid_argument("argumento desconhecido: " + std::string(argument));
        }

        const MatterEngine::RagdollProfile3D profile = MatterEngine::loadRagdollProfile3D(profilePath);
        const MatterEngine::RagdollCalibrationAudit3D audit = MatterEngine::auditRagdollCalibration3D(profile);
        std::cout << MatterEngine::renderRagdollCalibrationMarkdown3D(audit);
        const MatterEngine::RagdollEffectiveMass3D mass = MatterEngine::measureRagdollEffectiveMass3D(profile);
        if (!mass.valid) throw std::runtime_error("o backend nao publicou as propriedades de massa");
        std::cout << "\n---\n\n" << effectiveMassMarkdown(mass);
        if (audit.hasErrors()) return 2;
        if (strict && audit.findingCount(MatterEngine::CalibrationFindingSeverity3D::Warning) > 0) return 3;
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "MatterRLAudit: " << error.what() << '\n';
        return 1;
    }
}
