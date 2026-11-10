#include "Workbench/UiKit.hpp"
#include "Workbench/WorkbenchApp.hpp"
#include "imgui.h"

#include <algorithm>
#include <filesystem>

namespace MatterEngine::Workbench {
void WorkbenchApp::drawLaboratoryMaps() {
    beginFullscreenPanel("LaboratoryMaps");
    const auto &io = ImGui::GetIO();
    auto *draw = ImGui::GetWindowDrawList();
    drawWorkbenchBackdrop(draw, io.DisplaySize, m_uiScale);
    drawWorkbenchHeader(draw, io.DisplaySize, m_uiScale, "LABORATÓRIO", "MAPAS");
    ImGui::SetCursorPos({ui(24), headerHeight(m_uiScale) + ui(18)});
    if (ImGui::Button("Voltar", {ui(110), ui(36)}))
        m_screen = Screen::MainMenu;
    ImGui::Dummy({1, ui(14)});
    const char *ids[] = {"dev_map", "galpao_01"};
    const char *names[] = {"Dev Map", "Galpão 01"};
    const char *descriptions[] = {"Plataforma de testes · 60 × 60 m", "Quadra coberta · 40 × 20 m"};
    const float contentLeft = ImGui::GetCursorPosX();
    const float contentWidth = ImGui::GetContentRegionAvail().x;
    const float available = std::max(1.0f, contentWidth - ui(32));
    const bool twoColumns = available >= ui(640);
    const float width = std::min(ui(580), twoColumns ? (available - ui(20)) * .5f : available);
    const float groupWidth = twoColumns ? width * 2 + ui(20) : width;
    const float groupLeft = contentLeft + (contentWidth - groupWidth) * .5f;
    ImGui::SetCursorPosX(groupLeft);
    ImGui::TextUnformatted("Escolha um mapa");
    ImGui::Dummy({1, ui(12)});
    for (int i = 0; i < 2; ++i) {
        if (i && twoColumns)
            ImGui::SameLine(0, ui(20));
        else
            ImGui::SetCursorPosX(groupLeft);
        ImGui::PushID(i);
        ImGui::BeginChild("MapCard", {width, width * 9.0f / 16 + ui(130)}, true);
        if (!m_mapPreviews[i]) {
            const auto path =
                std::filesystem::path(MATTERENGINE_ASSETS_DIR) / "maps" / "previews" / (std::string(ids[i]) + ".png");
            if (std::filesystem::exists(path))
                m_mapPreviews[i] = m_renderer->loadUiTexture(path.string());
        }
        const float imageWidth = ImGui::GetContentRegionAvail().x;
        if (m_mapPreviews[i])
            ImGui::Image(static_cast<ImTextureID>(m_mapPreviews[i].id), {imageWidth, imageWidth * 9 / 16});
        else
            ImGui::Dummy({imageWidth, imageWidth * 9 / 16});
        ImGui::TextUnformatted(names[i]);
        ImGui::TextDisabled("%s", descriptions[i]);
        ImGui::Dummy({1, ui(8)});
        if (ImGui::Button("Abrir mapa", {-1, ui(36)}))
            selectLaboratoryMap(i == 1);
        ImGui::EndChild();
        ImGui::PopID();
    }
    ImGui::End();
}
} // namespace MatterEngine::Workbench
