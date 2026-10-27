// MatterModelInspector: as ferramentas de modelagem do personagem na linha de
// comando (para o Claude e o Codex). Renderiza vistas e folhas, analisa a
// malha, mede, corta, compara com as referencias e testa poses - com o mesmo
// codigo da tela "Inspecao de jogador" do Workbench.
//
// uso: MatterModelInspector <comando> [opcoes]   (ajuda: MatterModelInspector ajuda)

#include "CommandLine.hpp"
#include "Commands.hpp"
#include "ModelContext.hpp"

#include <iostream>

using namespace MatterEngine::ModelInspector;

int main(int argc, char** argv) {
    const CommandLine line = parseCommandLine(argc, argv);
    if (line.command.empty() || line.command == "ajuda" || line.command == "help" || line.has("ajuda"))
        return runHelp();
    std::string error;
    const std::optional<ModelContext> context = loadModelContext(line, &error);
    if (!context) {
        std::cerr << "nao carregou o modelo: " << error << '\n';
        return 1;
    }
    const std::string& command = line.command;
    if (command == "info") return runInfo(*context, line);
    if (command == "render") return runRender(*context, line);
    if (command == "folha") return runSheet(*context, line);
    if (command == "volta") return runTurntable(*context, line);
    if (command == "analisar") return runCheck(*context, line);
    if (command == "medir") return runMeasure(*context, line);
    if (command == "corte") return runSection(*context, line);
    if (command == "comparar") return runCompare(*context, line);
    if (command == "poses") return runPoses(*context, line);
    if (command == "textura") return runTexture(*context, line);
    if (command == "molde") return runMold(*context, line);
    if (command == "avesso") return runInverted(*context, line);
    if (command == "pintura") return runPaint(*context, line);
    if (command == "exportar-jogo") return runExport(*context, line);
    if (command == "exportar-blender") return runBlenderExport(*context, line);
    if (command == "uniforme") return runUniform(*context, line);
    if (command == "postura") return runPosture(*context, line);
    if (command == "pesos") return runWeights(*context, line);
    if (command == "quadros") return runFrames(*context, line);
#if defined(MATTERENGINE_MODEL_INSPECTOR_RL)
    if (command == "colisores") return runColliders(*context, line);
#endif
    std::cerr << "comando desconhecido: " << command << " (MatterModelInspector ajuda)\n";
    return 2;
}
