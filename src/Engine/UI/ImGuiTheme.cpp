#include "Engine/UI/ImGuiTheme.hpp"

#include "imgui.h"

namespace MatterEngine::UI {

void applyMatterEngineStyle() {
    ImGuiStyle& style = ImGui::GetStyle();
    style.WindowRounding = 4.0f;
    style.ChildRounding = 4.0f;
    style.FrameRounding = 3.0f;
    style.PopupRounding = 4.0f;
    style.ScrollbarRounding = 2.0f;
    style.GrabRounding = 2.0f;
    style.TabRounding = 2.0f;
    style.WindowBorderSize = 1.0f;
    style.FrameBorderSize = 1.0f;
    style.ChildBorderSize = 1.0f;
    style.WindowPadding = ImVec2(12.0f, 12.0f);
    style.FramePadding = ImVec2(10.0f, 6.0f);
    style.ItemSpacing = ImVec2(8.0f, 7.0f);
    style.ItemInnerSpacing = ImVec2(7.0f, 6.0f);
    style.WindowTitleAlign = ImVec2(0.03f, 0.5f);

    ImVec4* colors = style.Colors;
    colors[ImGuiCol_Text] = ImVec4(0.82f, 0.87f, 0.91f, 1.00f);
    colors[ImGuiCol_TextDisabled] = ImVec4(0.43f, 0.51f, 0.58f, 1.00f);
    colors[ImGuiCol_WindowBg] = ImVec4(0.031f, 0.052f, 0.071f, 0.98f);
    colors[ImGuiCol_ChildBg] = ImVec4(0.042f, 0.073f, 0.102f, 0.97f);
    colors[ImGuiCol_PopupBg] = ImVec4(0.047f, 0.078f, 0.108f, 0.99f);
    colors[ImGuiCol_Border] = ImVec4(0.10f, 0.17f, 0.23f, 1.00f);
    colors[ImGuiCol_FrameBg] = ImVec4(0.035f, 0.063f, 0.086f, 1.00f);
    colors[ImGuiCol_FrameBgHovered] = ImVec4(0.055f, 0.13f, 0.20f, 1.00f);
    colors[ImGuiCol_FrameBgActive] = ImVec4(0.06f, 0.20f, 0.33f, 1.00f);
    colors[ImGuiCol_TitleBg] = ImVec4(0.027f, 0.047f, 0.063f, 1.00f);
    colors[ImGuiCol_TitleBgActive] = ImVec4(0.035f, 0.075f, 0.11f, 1.00f);
    colors[ImGuiCol_Button] = ImVec4(0.05f, 0.18f, 0.30f, 1.00f);
    colors[ImGuiCol_ButtonHovered] = ImVec4(0.07f, 0.34f, 0.58f, 1.00f);
    colors[ImGuiCol_ButtonActive] = ImVec4(0.08f, 0.46f, 0.82f, 1.00f);
    colors[ImGuiCol_Header] = ImVec4(0.055f, 0.18f, 0.29f, 1.00f);
    colors[ImGuiCol_HeaderHovered] = ImVec4(0.07f, 0.31f, 0.52f, 1.00f);
    colors[ImGuiCol_HeaderActive] = ImVec4(0.08f, 0.42f, 0.74f, 1.00f);
    colors[ImGuiCol_CheckMark] = ImVec4(0.16f, 0.57f, 1.00f, 1.00f);
    colors[ImGuiCol_SliderGrab] = ImVec4(0.12f, 0.49f, 0.89f, 1.00f);
    colors[ImGuiCol_SliderGrabActive] = ImVec4(0.22f, 0.67f, 1.00f, 1.00f);
    colors[ImGuiCol_Separator] = ImVec4(0.10f, 0.17f, 0.23f, 1.00f);
    colors[ImGuiCol_Tab] = ImVec4(0.035f, 0.09f, 0.14f, 1.00f);
    colors[ImGuiCol_TabHovered] = ImVec4(0.06f, 0.28f, 0.47f, 1.00f);
    colors[ImGuiCol_TabSelected] = ImVec4(0.06f, 0.24f, 0.42f, 1.00f);
}

} // namespace MatterEngine::UI
