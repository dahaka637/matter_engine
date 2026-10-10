#include "Workbench/WorkbenchApp.hpp"

#include <filesystem>

namespace MatterEngine::Workbench {

// A tela vive em Workbench/ObjectInspector; aqui so a ligacao com o app: a
// pasta das receitas, a das capturas, a fisica e os materiais do app e a
// volta ao menu.
void WorkbenchApp::drawObjectInspector(Renderer& activeRenderer) {
    if (!m_objectInspector) {
        ObjectInspectorScreen::Options options;
        options.objectsDirectory = std::filesystem::path(MATTERENGINE_ASSETS_DIR) / "objects";
        options.capturesDirectory = std::filesystem::path(MATTERENGINE_ASSETS_DIR).parent_path() / "inspecao" / "capturas";
        options.physics = &m_physicsEngine;
        options.materials = &m_materialLibrary;
        m_objectInspector = std::make_unique<ObjectInspectorScreen>(std::move(options),
            [this] { m_screen = Screen::MainMenu; });
    }
    if (!m_objectInspectorCanvas) m_objectInspectorCanvas = std::make_unique<UiCanvasTexture>(activeRenderer);
    m_objectInspector->draw(*m_objectInspectorCanvas, m_uiScale);
}

} // namespace MatterEngine::Workbench
