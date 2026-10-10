#include "Commands.hpp"

#include "Engine/Modeling/Rig/GameCharacterExport.hpp"

#include <iostream>

namespace MatterEngine::ModelInspector {

using namespace Modeling;

int runExport(const ModelContext& context, const CommandLine& line) {
    if (!line.has("destino")) {
        std::cerr << "exportar-jogo: --destino <pasta> (o personagem do laboratorio e "
                     "assets/characters/football_player)\n";
        return 2;
    }
    GameCharacterExportSettings settings;
    settings.creaseDegrees = line.number("quinas", settings.creaseDegrees);
    settings.facet = line.number("facetado", settings.facet);
    GameCharacterExportReport report;
    std::string error;
    const std::filesystem::path destination = line.text("destino");
    if (!exportGameCharacter(context.loaded, destination, settings, report, error)) {
        std::cerr << error << '\n';
        return 1;
    }
    std::cout << "Jogador exportado em " << destination.string() << " (quinas " << settings.creaseDegrees << " graus, facetado "
              << settings.facet << ")\n";
    for (const std::string& entry : report.lines) std::cout << "  " << entry << '\n';
    return 0;
}

} // namespace MatterEngine::ModelInspector
