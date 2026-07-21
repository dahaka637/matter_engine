#include "Workbench/WorkbenchApp.hpp"
#include "Workbench/UiKit.hpp"

#include "Engine/UI/FontAwesome.hpp"
#include "imgui.h"

#include <algorithm>
#include <string>

namespace MatterEngine::Workbench {

void WorkbenchApp::drawObjectViewer() {
    beginFullscreenPanel("MatterEngineObjectViewer");
    const ImGuiIO& io = ImGui::GetIO();
    ImDrawList* drawList = ImGui::GetWindowDrawList();
    drawWorkbenchBackdrop(drawList, io.DisplaySize, m_uiScale);

    const auto& definitions = m_propCatalog.definitions();
    const std::string status = definitions.empty()
        ? "CARREGANDO" : std::to_string(definitions.size()) + " OBJETOS";
    drawWorkbenchHeader(drawList, io.DisplaySize, m_uiScale,
        "VISUALIZADOR", status.c_str());

    const float margin = ui(12.0f);
    const float top = headerHeight(m_uiScale) + margin;
    const float height = io.DisplaySize.y - top - margin;
    const float libraryWidth = sidebarWidth(m_uiScale);

    ImGui::SetCursorPos({ margin, top });
    ImGui::BeginChild("ObjectLibrary", { libraryWidth, height }, true,
        ImGuiWindowFlags_NoScrollbar);
    panelHeader("OBJETOS");
    ImGui::Dummy({ 1.0f, ui(8.0f) });
    if (definitions.empty()) {
        centeredTextColored({ 0.40f, 0.49f, 0.56f, 1.0f },
            m_propAssetsLoadFailed ? "Falha no catálogo" : "Carregando");
    } else {
        m_objectViewerSelectedIndex = std::min(
            m_objectViewerSelectedIndex, definitions.size() - 1);
        for (std::size_t index = 0; index < definitions.size(); ++index) {
            if (navigationButton(definitions[index].displayName.c_str(),
                    index == m_objectViewerSelectedIndex,
                    { -1.0f, ui(42.0f) })) {
                m_objectViewerSelectedIndex = index;
            }
        }
    }

    const std::string back = UI::FontAwesome::label(
        UI::FontAwesome::ArrowLeft, "Menu principal");
    ImGui::SetCursorPosY(std::max(ImGui::GetCursorPosY(),
        height - ui(58.0f)));
    if (navigationButton(back.c_str(), false, { -1.0f, ui(40.0f) })) {
        m_screen = Screen::MainMenu;
    }
    ImGui::EndChild();

    const float viewportLeft = margin * 2.0f + libraryWidth;
    const float viewportWidth = io.DisplaySize.x - viewportLeft - margin;
    ImGui::SetCursorPos({ viewportLeft, top });
    ImGui::BeginChild("ObjectViewport", { viewportWidth, height }, true,
        ImGuiWindowFlags_NoScrollbar);
    panelHeader("VIEWPORT");

    if (!definitions.empty()
        && m_objectViewerSelectedIndex < definitions.size()) {
        const PropDefinition3D& selected =
            definitions[m_objectViewerSelectedIndex];
        const float informationHeight = ui(72.0f);
        const ImVec2 available = ImGui::GetContentRegionAvail();
        const float previewHeight = std::max(ui(120.0f),
            available.y - informationHeight);
        if (m_objectViewerPreview) {
            const float sourceAspect = static_cast<float>(
                m_objectViewerPreview.width)
                / static_cast<float>(m_objectViewerPreview.height);
            const float imageWidth = std::min(available.x,
                previewHeight * sourceAspect);
            const float imageHeight = imageWidth / sourceAspect;
            ImGui::SetCursorPosX(ImGui::GetCursorPosX()
                + (available.x - imageWidth) * 0.5f);
            ImGui::Image(
                static_cast<ImTextureID>(m_objectViewerPreview.id),
                { imageWidth, imageHeight });
        } else {
            ImGui::Dummy({ available.x, previewHeight });
        }
        subtleSeparator();
        ImGui::TextUnformatted(selected.displayName.c_str());
        ImGui::SameLine(0.0f, ui(20.0f));
        ImGui::TextColored({ 0.52f, 0.66f, 0.77f, 1.0f },
            "%s  |  %.2f kg  |  %.2f x %.2f x %.2f m",
            selected.materialId.c_str(), selected.massKg,
            selected.dimensionsMeters.x, selected.dimensionsMeters.y,
            selected.dimensionsMeters.z);
    } else {
        const char* emptyLabel = "NENHUM OBJETO DISPONÍVEL";
        const ImVec2 labelSize = ImGui::CalcTextSize(emptyLabel);
        ImGui::SetCursorPos({ (viewportWidth - labelSize.x) * 0.5f,
            (height - labelSize.y) * 0.5f });
        ImGui::TextColored({ 0.34f, 0.43f, 0.51f, 1.0f },
            "%s", emptyLabel);
    }
    ImGui::EndChild();
    ImGui::End();
}

} // namespace MatterEngine::Workbench
