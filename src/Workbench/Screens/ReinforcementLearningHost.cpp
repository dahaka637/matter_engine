#include "Workbench/WorkbenchApp.hpp"

#include <filesystem>

namespace MatterEngine::Workbench {

// A tela vive em Workbench/ReinforcementLearning; aqui so a ligacao com o app:
// a raiz do repositorio (o treinador e as tarefas), a pasta das capturas e a
// volta ao menu.
void WorkbenchApp::drawReinforcementLearning(Renderer& activeRenderer) {
    if (!m_reinforcementLearning) {
        ReinforcementLearningScreen::Options options;
        options.sourceRoot = std::filesystem::path(MATTERENGINE_ASSETS_DIR).parent_path();
        options.capturesDirectory = options.sourceRoot / "inspecao" / "capturas";
        m_reinforcementLearning = std::make_unique<ReinforcementLearningScreen>(std::move(options),
            [this] { m_screen = Screen::MainMenu; });
    }
    if (!m_reinforcementLearningCanvas) m_reinforcementLearningCanvas = std::make_unique<UiCanvasTexture>(activeRenderer);
    m_reinforcementLearning->draw(*m_reinforcementLearningCanvas, m_uiScale);
}

} // namespace MatterEngine::Workbench
