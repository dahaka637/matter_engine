#include "Workbench/UiKit.hpp"

#include <algorithm>

namespace MatterEngine::Workbench {

void beginFullscreenPanel(const char* name) {
    const ImGuiIO& io = ImGui::GetIO();
    ImGui::SetNextWindowPos({ 0.0f, 0.0f }, ImGuiCond_Always);
    ImGui::SetNextWindowSize(io.DisplaySize, ImGuiCond_Always);
    ImGui::Begin(name, nullptr,
        ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove
            | ImGuiWindowFlags_NoSavedSettings
            | ImGuiWindowFlags_NoBringToFrontOnFocus);
}

void drawWorkbenchBackdrop(ImDrawList* drawList, ImVec2 extent, float scale) {
    drawList->AddRectFilledMultiColor({ 0.0f, 0.0f }, extent,
        IM_COL32(6, 11, 17, 255), IM_COL32(8, 15, 22, 255),
        IM_COL32(5, 9, 14, 255), IM_COL32(7, 13, 19, 255));

    // A referência usa profundidade por superfícies, não por ornamentos. Este
    // brilho muito discreto separa a área útil sem competir com os painéis.
    const ImVec2 glow { extent.x * 0.72f, extent.y * 0.30f };
    for (int radius = 520; radius >= 160; radius -= 60) {
        const int alpha = std::max(1, 5 - radius / 150);
        drawList->AddCircleFilled(glow, radius * scale,
            IM_COL32(16, 104, 181, alpha), 72);
    }
}

float headerHeight(float scale) {
    return 54.0f * scale;
}

float sidebarWidth(float scale) {
    return 218.0f * scale;
}

void drawWorkbenchHeader(ImDrawList* drawList, ImVec2 extent, float scale,
    const char* context, const char* status) {
    const float height = headerHeight(scale);
    drawList->AddRectFilled({ 0.0f, 0.0f }, { extent.x, height }, UiTopBar);
    drawList->AddLine({ 0.0f, height - 1.0f },
        { extent.x, height - 1.0f }, UiBorder);

    // Marca compacta construída como um M de traço grosso. O desenho vetorial
    // permanece nítido em qualquer escala e não depende de um glifo específico.
    const ImVec2 mark[] {
        { 17.0f * scale, 37.0f * scale },
        { 17.0f * scale, 17.0f * scale },
        { 28.0f * scale, 31.0f * scale },
        { 39.0f * scale, 17.0f * scale },
        { 39.0f * scale, 37.0f * scale }
    };
    drawList->AddPolyline(mark, 5, UiAccent, ImDrawFlags_None, 4.0f * scale);

    const float textY = (height - ImGui::GetFontSize()) * 0.5f;
    drawList->AddText({ 51.0f * scale, textY }, UiText, "MATTERENGINE");
    const ImVec2 brandSize = ImGui::CalcTextSize("MATTERENGINE");
    drawList->AddLine(
        { 62.0f * scale + brandSize.x, 17.0f * scale },
        { 62.0f * scale + brandSize.x, height - 17.0f * scale }, UiBorder);
    drawList->AddText({ 76.0f * scale + brandSize.x, textY },
        UiMuted, context);

    if (status != nullptr && status[0] != '\0') {
        const ImVec2 statusSize = ImGui::CalcTextSize(status);
        const float right = extent.x - 18.0f * scale;
        drawList->AddText({ right - statusSize.x, textY }, UiAccent, status);
    }
}

float menuWidth(float scale) {
    const ImGuiIO& io = ImGui::GetIO();
    return std::min(360.0f * scale,
        std::max(260.0f * scale, io.DisplaySize.x - 56.0f * scale));
}

void sectionLabel(const char* text) {
    ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.25f, 0.66f, 1.0f, 1.0f));
    ImGui::TextUnformatted(text);
    ImGui::PopStyleColor();
}

void panelHeader(const char* text) {
    sectionLabel(text);
    ImGui::Dummy({ 1.0f, 2.0f });
    subtleSeparator();
}

void subtleSeparator() {
    ImGui::PushStyleColor(ImGuiCol_Separator, ImVec4(0.14f, 0.24f, 0.33f, 1.0f));
    ImGui::Separator();
    ImGui::PopStyleColor();
}

bool tabButton(const char* label, bool active, ImVec2 size) {
    ImGui::PushStyleColor(ImGuiCol_Button,
        active ? ImVec4(0.08f, 0.31f, 0.58f, 1.0f)
               : ImVec4(0.055f, 0.095f, 0.14f, 1.0f));
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered,
        active ? ImVec4(0.09f, 0.40f, 0.76f, 1.0f)
               : ImVec4(0.08f, 0.16f, 0.24f, 1.0f));
    ImGui::PushStyleColor(ImGuiCol_ButtonActive,
        ImVec4(0.08f, 0.46f, 0.90f, 1.0f));
    ImGui::PushStyleColor(ImGuiCol_Text,
        active ? ImVec4(0.92f, 0.97f, 1.0f, 1.0f)
               : ImVec4(0.55f, 0.66f, 0.76f, 1.0f));
    const bool clicked = ImGui::Button(label, size);
    ImGui::PopStyleColor(4);
    return clicked;
}

bool navigationButton(const char* label, bool active, ImVec2 size) {
    ImGui::PushStyleVar(ImGuiStyleVar_ButtonTextAlign, { 0.08f, 0.5f });
    const bool clicked = tabButton(label, active, size);
    ImGui::PopStyleVar();
    return clicked;
}

void centerNextItem(float width) {
    const float available = ImGui::GetContentRegionAvail().x;
    if (width < available) {
        ImGui::SetCursorPosX(ImGui::GetCursorPosX()
            + (available - width) * 0.5f);
    }
}

void centeredTextColored(const ImVec4& color, const char* text) {
    centerNextItem(ImGui::CalcTextSize(text).x);
    ImGui::TextColored(color, "%s", text);
}

void centeredSectionLabel(const char* text) {
    centerNextItem(ImGui::CalcTextSize(text).x);
    sectionLabel(text);
}

void centeredSeparator(float width) {
    centerNextItem(width);
    const ImVec2 position = ImGui::GetCursorScreenPos();
    ImGui::GetWindowDrawList()->AddLine(position,
        { position.x + width, position.y }, UiBorder);
    ImGui::Dummy({ width, 1.0f });
}

bool centeredRadioButton(const char* label, int* value, int button) {
    const float width = ImGui::GetFrameHeight()
        + ImGui::GetStyle().ItemInnerSpacing.x + ImGui::CalcTextSize(label).x;
    centerNextItem(width);
    return ImGui::RadioButton(label, value, button);
}

bool centeredCheckbox(const char* label, bool* value) {
    const float width = ImGui::GetFrameHeight()
        + ImGui::GetStyle().ItemInnerSpacing.x + ImGui::CalcTextSize(label).x;
    centerNextItem(width);
    return ImGui::Checkbox(label, value);
}

} // namespace MatterEngine::Workbench
