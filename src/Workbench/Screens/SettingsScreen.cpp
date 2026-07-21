#include "Workbench/WorkbenchApp.hpp"
#include "Workbench/UiKit.hpp"

#include "Engine/Platform/PlatformServices.hpp"
#include "Engine/UI/FontAwesome.hpp"
#include "imgui.h"

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

namespace MatterEngine::Workbench {
namespace {

constexpr float VideoConfirmSeconds = 15.0f;

std::string settingsDatabasePath() {
    return Platform::executableBasePath() + "matterengine.db";
}

} // namespace

void WorkbenchApp::drawSettings() {
    beginFullscreenPanel("MatterEngineSettings");
    const ImGuiIO& io = ImGui::GetIO();
    ImDrawList* drawList = ImGui::GetWindowDrawList();
    drawWorkbenchBackdrop(drawList, io.DisplaySize, m_uiScale);
    drawWorkbenchHeader(drawList, io.DisplaySize, m_uiScale,
        "CONFIGURAÇÕES", "PREFERÊNCIAS");

    const float margin = ui(12.0f);
    const float contentTop = headerHeight(m_uiScale) + margin;
    const float panelHeight = io.DisplaySize.y - contentTop - margin;
    const float navigationWidth = sidebarWidth(m_uiScale);

    ImGui::SetCursorPos({ margin, contentTop });
    ImGui::BeginChild("SettingsNavigation",
        { navigationWidth, panelHeight }, true,
        ImGuiWindowFlags_NoScrollbar);
    panelHeader("CATEGORIAS");
    ImGui::Dummy({ 1.0f, ui(5.0f) });

    const std::string video = UI::FontAwesome::label(
        UI::FontAwesome::Sliders, "Vídeo");
    const std::string general = UI::FontAwesome::label(
        UI::FontAwesome::Gear, "Geral");
    const std::string back = UI::FontAwesome::label(
        UI::FontAwesome::ArrowLeft, "Voltar");
    if (navigationButton(video.c_str(), m_settingsTab == 0,
            { -1.0f, ui(42.0f) })) {
        m_settingsTab = 0;
    }
    if (navigationButton(general.c_str(), m_settingsTab == 1,
            { -1.0f, ui(42.0f) })) {
        m_settingsTab = 1;
    }
    ImGui::SetCursorPosY(std::max(ImGui::GetCursorPosY(),
        panelHeight - ui(58.0f)));
    if (navigationButton(back.c_str(), false, { -1.0f, ui(40.0f) })) {
        m_screen = m_settingsReturnScreen;
    }
    ImGui::EndChild();

    const float panelLeft = margin * 2.0f + navigationWidth;
    const float panelWidth = io.DisplaySize.x - panelLeft - margin;
    ImGui::SetCursorPos({ panelLeft, contentTop });
    ImGui::BeginChild("SettingsContent", { panelWidth, panelHeight }, true);
    panelHeader(m_settingsTab == 0 ? "VÍDEO" : "GERAL");
    ImGui::Dummy({ 1.0f, ui(14.0f) });

    if (m_settingsTab == 0) {
        drawSettingsVideoTab();
    } else {
        drawSettingsGeneralTab();
    }

    ImGui::EndChild();
    ImGui::End();
}

void WorkbenchApp::drawSettingsVideoTab() {
    if (!m_resolutionsPopulated) {
        populateAvailableResolutions();
        m_resolutionsPopulated = true;
    }

    const float width = std::min(ui(520.0f), ImGui::GetContentRegionAvail().x);
    sectionLabel("RESOLUÇÃO");
    ImGui::Dummy({ 1.0f, ui(5.0f) });

    std::vector<std::string> labels;
    std::vector<const char*> labelPointers;
    labels.reserve(m_availableResolutions.size());
    labelPointers.reserve(m_availableResolutions.size());
    for (const auto& [resolutionWidth, resolutionHeight] : m_availableResolutions) {
        labels.push_back(std::to_string(resolutionWidth) + " × "
            + std::to_string(resolutionHeight));
    }
    for (const std::string& label : labels) {
        labelPointers.push_back(label.c_str());
    }

    const bool borderless = m_pendingDisplayMode
        == static_cast<int>(DisplayMode::BorderlessFullscreen);
    ImGui::SetNextItemWidth(width);
    if (borderless) {
        ImGui::BeginDisabled();
    }
    ImGui::Combo("##Resolution", &m_pendingResolutionIndex,
        labelPointers.data(), static_cast<int>(labelPointers.size()));
    if (borderless) {
        ImGui::EndDisabled();
    }

    ImGui::Dummy({ 1.0f, ui(18.0f) });
    subtleSeparator();
    ImGui::Dummy({ 1.0f, ui(18.0f) });
    sectionLabel("MODO DE EXIBIÇÃO");
    ImGui::Dummy({ 1.0f, ui(5.0f) });
    ImGui::RadioButton("Janela", &m_pendingDisplayMode,
        static_cast<int>(DisplayMode::Windowed));
    ImGui::RadioButton("Tela cheia", &m_pendingDisplayMode,
        static_cast<int>(DisplayMode::Fullscreen));
    ImGui::RadioButton("Janela sem bordas", &m_pendingDisplayMode,
        static_cast<int>(DisplayMode::BorderlessFullscreen));

    ImGui::Dummy({ 1.0f, ui(18.0f) });
    if (ImGui::Button("Aplicar", { ui(170.0f), ui(40.0f) })) {
        applyAndSaveVideoSettings();
    }
    if (!m_videoApplyStatus.empty()) {
        ImGui::Dummy({ 1.0f, ui(7.0f) });
        ImGui::TextColored({ 0.35f, 0.78f, 0.60f, 1.0f }, "%s",
            m_videoApplyStatus.c_str());
    }
}

void WorkbenchApp::drawSettingsGeneralTab() {
    sectionLabel("INTERFACE");
    ImGui::Dummy({ 1.0f, ui(8.0f) });
    ImGui::Checkbox("Exibir FPS no laboratório", &m_showPerformanceOverlay);
    ImGui::Dummy({ 1.0f, ui(14.0f) });
    subtleSeparator();

    ImGui::Dummy({ 1.0f, ui(14.0f) });
    sectionLabel("ÁUDIO");
    ImGui::Dummy({ 1.0f, ui(8.0f) });
    bool hrtfEnabled = m_hrtfEnabled;
    if (ImGui::Checkbox("Áudio binaural (HRTF)", &hrtfEnabled)) {
        setHrtfEnabled(hrtfEnabled);
    }
    ImGui::Dummy({ 1.0f, ui(14.0f) });
    subtleSeparator();
}

void WorkbenchApp::populateAvailableResolutions() {
    m_availableResolutions = m_renderer != nullptr
        ? m_renderer->availableFullscreenResolutions()
        : std::vector<std::pair<int, int>> {};
    if (m_availableResolutions.empty()) {
        m_availableResolutions = {
            { 1280, 720 }, { 1600, 900 }, { 1920, 1080 },
            { 2560, 1440 }, { 3840, 2160 }
        };
    }

    const int currentWidth = m_renderer != nullptr ? m_renderer->width() : 0;
    const int currentHeight = m_renderer != nullptr ? m_renderer->height() : 0;
    m_pendingResolutionIndex = 0;
    for (std::size_t index = 0; index < m_availableResolutions.size(); ++index) {
        if (m_availableResolutions[index].first == currentWidth
            && m_availableResolutions[index].second == currentHeight) {
            m_pendingResolutionIndex = static_cast<int>(index);
            break;
        }
    }
    m_pendingDisplayMode = m_renderer != nullptr
        ? static_cast<int>(m_renderer->displayMode()) : 0;
}

void WorkbenchApp::applyAndSaveVideoSettings() {
    if (m_renderer == nullptr || m_availableResolutions.empty()) {
        return;
    }
    const std::size_t index = static_cast<std::size_t>(std::clamp(
        m_pendingResolutionIndex, 0,
        static_cast<int>(m_availableResolutions.size()) - 1));
    const auto [width, height] = m_availableResolutions[index];
    const DisplayMode mode = static_cast<DisplayMode>(
        std::clamp(m_pendingDisplayMode, 0, 2));

    m_videoConfirm.previous = {
        m_renderer->width(), m_renderer->height(), m_renderer->displayMode()
    };
    m_renderer->applyVideoSettings(width, height, mode);
    m_videoConfirm.applied = { width, height, mode };
    m_videoConfirm.active = true;
    m_videoConfirm.secondsLeft = VideoConfirmSeconds;
    m_videoApplyStatus.clear();
}

void WorkbenchApp::confirmVideoSettings() {
    Data::SettingsRepository(settingsDatabasePath()).saveVideoSettings(
        m_videoConfirm.applied);
    m_videoConfirm.active = false;
    m_videoApplyStatus = "Configurações salvas";
    populateAvailableResolutions();
}

void WorkbenchApp::revertVideoSettings() {
    if (m_renderer != nullptr) {
        m_renderer->applyVideoSettings(m_videoConfirm.previous.width,
            m_videoConfirm.previous.height, m_videoConfirm.previous.mode);
    }
    m_videoConfirm.active = false;
    m_videoApplyStatus = "Alterações revertidas";
    populateAvailableResolutions();
}

void WorkbenchApp::setHrtfEnabled(bool enabled) {
    if (m_hrtfEnabled == enabled) return;
    m_hrtfEnabled = enabled;
    Data::SettingsRepository(settingsDatabasePath())
        .setInt("audio.hrtf_enabled", enabled ? 1 : 0);
    // HRTF só pode ser pedido na criação do contexto OpenAL; reabrir o
    // dispositivo é a única forma de aplicar a troca imediatamente, sem
    // exigir reiniciar o Workbench inteiro.
    m_worldAudio.shutdown();
    static_cast<void>(m_worldAudio.initialize(
        MATTERENGINE_ASSETS_DIR, m_hrtfEnabled));
}

void WorkbenchApp::drawVideoConfirmPopup() {
    const ImGuiIO& io = ImGui::GetIO();
    ImGui::SetNextWindowPos({ 0.0f, 0.0f }, ImGuiCond_Always);
    ImGui::SetNextWindowSize(io.DisplaySize, ImGuiCond_Always);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, { 0.0f, 0.0f });
    ImGui::Begin("VideoConfirmation", nullptr,
        ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove
            | ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoBackground);
    ImDrawList* drawList = ImGui::GetWindowDrawList();
    drawList->AddRectFilled({ 0.0f, 0.0f }, io.DisplaySize,
        IM_COL32(1, 5, 10, 205));

    const float width = ui(430.0f);
    const float height = ui(158.0f);
    const ImVec2 minimum { (io.DisplaySize.x - width) * 0.5f,
        (io.DisplaySize.y - height) * 0.5f };
    const ImVec2 maximum { minimum.x + width, minimum.y + height };
    drawList->AddRectFilled(minimum, maximum, UiPanelElevated, ui(10.0f));
    drawList->AddRect(minimum, maximum, UiBorder, ui(10.0f));

    ImGui::SetCursorScreenPos({ minimum.x + ui(20.0f), minimum.y + ui(18.0f) });
    ImGui::BeginGroup();
    ImGui::TextUnformatted("Manter estas configurações?");
    ImGui::TextDisabled("Reversão automática em %d s",
        static_cast<int>(std::ceil(std::max(0.0f,
            m_videoConfirm.secondsLeft))));
    ImGui::Dummy({ 1.0f, ui(14.0f) });
    const float buttonWidth = (width - ui(50.0f)) * 0.5f;
    if (ImGui::Button("Manter", { buttonWidth, ui(38.0f) })) {
        confirmVideoSettings();
    }
    ImGui::SameLine(0.0f, ui(10.0f));
    if (ImGui::Button("Reverter", { buttonWidth, ui(38.0f) })) {
        revertVideoSettings();
    }
    ImGui::EndGroup();
    ImGui::End();
    ImGui::PopStyleVar();
}

} // namespace MatterEngine::Workbench
