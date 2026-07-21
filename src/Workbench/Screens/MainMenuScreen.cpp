#include "Workbench/WorkbenchApp.hpp"
#include "Workbench/UiKit.hpp"

#include "Engine/Core/Version.hpp"
#include "Engine/UI/FontAwesome.hpp"
#include "imgui.h"

#include <algorithm>
#include <string>

namespace MatterEngine::Workbench {

void WorkbenchApp::drawMainMenu() {
    beginFullscreenPanel("MatterEngineMainMenu");
    const ImGuiIO& io = ImGui::GetIO();
    ImDrawList* drawList = ImGui::GetWindowDrawList();
    drawWorkbenchBackdrop(drawList, io.DisplaySize, m_uiScale);

    const std::string versionLabel = "v" + std::string(Version);
    drawWorkbenchHeader(drawList, io.DisplaySize, m_uiScale,
        "WORKBENCH", versionLabel.c_str());

    const float margin = ui(12.0f);
    const float top = headerHeight(m_uiScale) + margin;
    const float height = io.DisplaySize.y - top - margin;
    const float navigationWidth = sidebarWidth(m_uiScale);
    const float buttonHeight = ui(42.0f);

    ImGui::SetCursorPos({ margin, top });
    ImGui::BeginChild("MainNavigation", { navigationWidth, height }, true,
        ImGuiWindowFlags_NoScrollbar);
    panelHeader("NAVEGAÇÃO");
    ImGui::Dummy({ 1.0f, ui(5.0f) });

    const std::string laboratory = UI::FontAwesome::label(
        UI::FontAwesome::Play, "Laboratório");
    const std::string viewer = UI::FontAwesome::label(
        UI::FontAwesome::Cube, "Visualizador");
    const std::string settings = UI::FontAwesome::label(
        UI::FontAwesome::Gear, "Configurações");
    const std::string quit = UI::FontAwesome::label(
        UI::FontAwesome::DoorOpen, "Sair");

    if (navigationButton(laboratory.c_str(), false,
            { -1.0f, buttonHeight })) {
        enterLaboratory();
    }
    if (navigationButton(viewer.c_str(), false,
            { -1.0f, buttonHeight })) {
        m_screen = Screen::ObjectViewer;
    }
    if (navigationButton(settings.c_str(), false,
            { -1.0f, buttonHeight })) {
        m_settingsReturnScreen = Screen::MainMenu;
        m_screen = Screen::Settings;
    }
    ImGui::Dummy({ 1.0f, ui(8.0f) });
    subtleSeparator();
    ImGui::Dummy({ 1.0f, ui(8.0f) });
    if (navigationButton(quit.c_str(), false,
            { -1.0f, buttonHeight })) {
        requestQuit();
    }
    ImGui::EndChild();

    const float contentLeft = margin * 2.0f + navigationWidth;
    const float contentWidth = io.DisplaySize.x - contentLeft - margin;
    ImGui::SetCursorPos({ contentLeft, top });
    ImGui::BeginChild("MainWorkspace", { contentWidth, height }, true,
        ImGuiWindowFlags_NoScrollbar);
    panelHeader("INÍCIO");

    const float logoWidth = std::min(ui(320.0f), contentWidth * 0.48f);
    ImGui::Dummy({ 1.0f, std::max(ui(14.0f), height * 0.06f) });
    if (m_menuLogo) {
        const float aspect = static_cast<float>(m_menuLogo.width)
            / static_cast<float>(std::max(1, m_menuLogo.height));
        centerNextItem(logoWidth);
        ImGui::Image(static_cast<ImTextureID>(m_menuLogo.id),
            { logoWidth, logoWidth / aspect });
    } else {
        ImGui::SetWindowFontScale(1.65f);
        centerNextItem(ImGui::CalcTextSize("MATTERENGINE").x);
        ImGui::TextUnformatted("MATTERENGINE");
        ImGui::SetWindowFontScale(1.0f);
    }

    ImGui::Dummy({ 1.0f, ui(16.0f) });
    centeredTextColored({ 0.45f, 0.56f, 0.65f, 1.0f },
        "Ambiente de desenvolvimento");
    ImGui::Dummy({ 1.0f, ui(18.0f) });

    const float actionWidth = std::min(ui(260.0f), contentWidth - ui(48.0f));
    centerNextItem(actionWidth);
    if (ImGui::Button("ABRIR LABORATÓRIO",
            { actionWidth, ui(44.0f) })) {
        enterLaboratory();
    }
    ImGui::EndChild();
    ImGui::End();
}

} // namespace MatterEngine::Workbench
