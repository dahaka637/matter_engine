#pragma once

#include "imgui.h"

namespace MatterEngine::Workbench {

constexpr ImU32 UiBackground = IM_COL32(7, 12, 19, 255);
constexpr ImU32 UiTopBar = IM_COL32(7, 13, 19, 252);
constexpr ImU32 UiPanel = IM_COL32(11, 20, 29, 248);
constexpr ImU32 UiPanelElevated = IM_COL32(14, 25, 36, 255);
constexpr ImU32 UiBorder = IM_COL32(25, 43, 58, 255);
constexpr ImU32 UiText = IM_COL32(211, 222, 232, 255);
constexpr ImU32 UiMuted = IM_COL32(119, 137, 153, 255);
constexpr ImU32 UiAccent = IM_COL32(36, 145, 255, 255);
constexpr ImU32 UiAccentSoft = IM_COL32(16, 57, 91, 255);
constexpr ImU32 UiSuccess = IM_COL32(59, 201, 126, 255);

void beginFullscreenPanel(const char* name);
void drawWorkbenchBackdrop(ImDrawList* drawList, ImVec2 extent, float scale);
void drawWorkbenchHeader(ImDrawList* drawList, ImVec2 extent, float scale,
    const char* context, const char* status = nullptr);
float headerHeight(float scale);
float sidebarWidth(float scale);
float menuWidth(float scale);
void sectionLabel(const char* text);
void panelHeader(const char* text);
void subtleSeparator();
bool tabButton(const char* label, bool active, ImVec2 size);
bool navigationButton(const char* label, bool active, ImVec2 size);

void centerNextItem(float width);
void centeredTextColored(const ImVec4& color, const char* text);
void centeredSectionLabel(const char* text);
void centeredSeparator(float width);
bool centeredRadioButton(const char* label, int* value, int button);
bool centeredCheckbox(const char* label, bool* value);

} // namespace MatterEngine::Workbench
