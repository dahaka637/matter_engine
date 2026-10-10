#include "Workbench/WorkbenchApp.hpp"

#include <filesystem>

namespace MatterEngine::Workbench {

// A tela vive em Workbench/PlayerInspector; aqui so a ligacao com o app: o
// manifesto do jogador, a pasta das capturas e a volta ao menu.
void WorkbenchApp::drawPlayerInspector(Renderer& activeRenderer) {
    if (!m_playerInspector) {
        PlayerInspectorScreen::Options options;
        options.manifest = Modeling::defaultPlayerModelManifest();
        options.capturesDirectory = std::filesystem::path(MATTERENGINE_ASSETS_DIR).parent_path() / "inspecao" / "capturas";
        m_playerInspector = std::make_unique<PlayerInspectorScreen>(std::move(options),
            [this] { m_screen = Screen::MainMenu; });
    }
    if (!m_playerInspectorCanvas) m_playerInspectorCanvas = std::make_unique<UiCanvasTexture>(activeRenderer);
    m_playerInspector->draw(*m_playerInspectorCanvas, m_uiScale);
}

} // namespace MatterEngine::Workbench
