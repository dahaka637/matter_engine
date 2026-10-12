#include "Support/HeadlessUi.hpp"

#include "Engine/Modeling/Raster/ImageIO.hpp"
#include "Engine/UI/ImGuiFontLibrary.hpp"
#include "Engine/UI/ImGuiTheme.hpp"

#include "imgui.h"
#include "imgui_internal.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <iostream>

namespace MatterEngine::TestSupport {

namespace {

std::vector<std::string> imguiErrors;

} // namespace

std::uint64_t FakeCanvas::present(const Modeling::Image& image, bool changed) {
    ++presents;
    if (changed || image.width() != width || image.height() != height) ++uploads;
    width = image.width();
    height = image.height();
    last = image;
    return 1;
}

Modeling::Image rasterizeImGui(const ImDrawData* data, const Modeling::Image& canvas) {
    using Modeling::Color;
    const int width = static_cast<int>(data->DisplaySize.x), height = static_cast<int>(data->DisplaySize.y);
    Modeling::Image image(width, height, Modeling::rgb(0, 0, 0));
    const auto unpack = [](ImU32 color) {
        return Color { static_cast<float>(color & 0xFF) / 255.0f, static_cast<float>((color >> 8) & 0xFF) / 255.0f,
            static_cast<float>((color >> 16) & 0xFF) / 255.0f, static_cast<float>((color >> 24) & 0xFF) / 255.0f };
    };
    for (const ImDrawList* list : data->CmdLists) {
        for (const ImDrawCmd& command : list->CmdBuffer) {
            if (command.UserCallback != nullptr) continue;
            const ImTextureData* texture = command.TexRef._TexData;
            const bool useCanvas = texture == nullptr && command.TexRef._TexID == 1 && !canvas.empty();
            const float clipX0 = std::max(0.0f, command.ClipRect.x - data->DisplayPos.x);
            const float clipY0 = std::max(0.0f, command.ClipRect.y - data->DisplayPos.y);
            const float clipX1 = std::min(static_cast<float>(width), command.ClipRect.z - data->DisplayPos.x);
            const float clipY1 = std::min(static_cast<float>(height), command.ClipRect.w - data->DisplayPos.y);
            for (unsigned i = 0; i + 2 < command.ElemCount; i += 3) {
                const ImDrawVert& a = list->VtxBuffer[static_cast<int>(command.VtxOffset + list->IdxBuffer[static_cast<int>(command.IdxOffset + i)])];
                const ImDrawVert& b = list->VtxBuffer[static_cast<int>(command.VtxOffset + list->IdxBuffer[static_cast<int>(command.IdxOffset + i + 1)])];
                const ImDrawVert& c = list->VtxBuffer[static_cast<int>(command.VtxOffset + list->IdxBuffer[static_cast<int>(command.IdxOffset + i + 2)])];
                const float area = (b.pos.x - a.pos.x) * (c.pos.y - a.pos.y) - (b.pos.y - a.pos.y) * (c.pos.x - a.pos.x);
                if (std::abs(area) < 1e-8f) continue;
                const int x0 = static_cast<int>(std::max(clipX0, std::floor(std::min({ a.pos.x, b.pos.x, c.pos.x }))));
                const int x1 = static_cast<int>(std::min(clipX1, std::ceil(std::max({ a.pos.x, b.pos.x, c.pos.x }))));
                const int y0 = static_cast<int>(std::max(clipY0, std::floor(std::min({ a.pos.y, b.pos.y, c.pos.y }))));
                const int y1 = static_cast<int>(std::min(clipY1, std::ceil(std::max({ a.pos.y, b.pos.y, c.pos.y }))));
                const Color ca = unpack(a.col), cb = unpack(b.col), cc = unpack(c.col);
                for (int y = y0; y < y1; ++y)
                    for (int x = x0; x < x1; ++x) {
                        const float px = static_cast<float>(x) + 0.5f, py = static_cast<float>(y) + 0.5f;
                        const float w0 = ((c.pos.x - b.pos.x) * (py - b.pos.y) - (c.pos.y - b.pos.y) * (px - b.pos.x)) / area;
                        const float w1 = ((a.pos.x - c.pos.x) * (py - c.pos.y) - (a.pos.y - c.pos.y) * (px - c.pos.x)) / area;
                        const float w2 = 1.0f - w0 - w1;
                        if (w0 < 0.0f || w1 < 0.0f || w2 < 0.0f) continue;
                        const float u = w0 * a.uv.x + w1 * b.uv.x + w2 * c.uv.x;
                        const float v = w0 * a.uv.y + w1 * b.uv.y + w2 * c.uv.y;
                        Color color { w0 * ca.r + w1 * cb.r + w2 * cc.r, w0 * ca.g + w1 * cb.g + w2 * cc.g, w0 * ca.b + w1 * cb.b + w2 * cc.b,
                            w0 * ca.a + w1 * cb.a + w2 * cc.a };
                        if (texture != nullptr && texture->Pixels != nullptr) {
                            const int tx = std::clamp(static_cast<int>(u * texture->Width), 0, texture->Width - 1);
                            const int ty = std::clamp(static_cast<int>(v * texture->Height), 0, texture->Height - 1);
                            const unsigned char* texel
                                = texture->Pixels + (static_cast<std::size_t>(ty) * texture->Width + tx) * texture->BytesPerPixel;
                            if (texture->Format == ImTextureFormat_Alpha8) color.a *= texel[0] / 255.0f;
                            else {
                                color.r *= texel[0] / 255.0f;
                                color.g *= texel[1] / 255.0f;
                                color.b *= texel[2] / 255.0f;
                                color.a *= texel[3] / 255.0f;
                            }
                        } else if (useCanvas) {
                            const Color sample = canvas.pixel(std::clamp(static_cast<int>(u * canvas.width()), 0, canvas.width() - 1),
                                std::clamp(static_cast<int>(v * canvas.height()), 0, canvas.height() - 1));
                            color.r *= sample.r;
                            color.g *= sample.g;
                            color.b *= sample.b;
                        }
                        image.blendPixel(x, y, { color.r, color.g, color.b, 1.0f }, color.a);
                    }
            }
        }
    }
    return image;
}

void snapshot(const FakeCanvas& canvas, const char* name) {
    const char* directory = std::getenv("MATTERENGINE_UI_SNAPSHOT");
    if (directory == nullptr || *directory == 0) return;
    const Modeling::Image image = rasterizeImGui(ImGui::GetDrawData(), canvas.last);
    const std::filesystem::path path = std::filesystem::path(directory) / name;
    if (Modeling::writePng(image, path.string())) std::cout << "foto da tela: " << path.string() << '\n';
}

HeadlessUi::HeadlessUi(float width, float height) {
    imguiErrors.clear();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.DisplaySize = { width, height };
    // A mesma fonte e o mesmo tema do Workbench.
    UI::configureImGuiFonts();
    UI::applyMatterEngineStyle();
    io.BackendFlags |= ImGuiBackendFlags_RendererHasTextures;
    io.IniFilename = nullptr;
    io.ConfigErrorRecoveryEnableAssert = false;
    GImGui->ErrorCallback = [](ImGuiContext*, void*, const char* message) { imguiErrors.emplace_back(message); };
}

HeadlessUi::~HeadlessUi() {
    ImGui::DestroyContext();
}

std::vector<std::string> sweepIdConflicts(const std::function<void()>& drawFrame, float step) {
    ImGuiIO& io = ImGui::GetIO();
    const ImGuiContext& g = *ImGui::GetCurrentContext();
    std::vector<std::string> conflicts;
    for (float y = step * 0.5f; y < io.DisplaySize.y; y += step) {
        for (float x = step * 0.5f; x < io.DisplaySize.x; x += step) {
            io.MousePos = { x, y };
            int count = 0;
            for (int frame = 0; frame < 2; ++frame) {
                io.DeltaTime = 1.0f / 60.0f;
                ImGui::NewFrame();
                drawFrame();
                count = g.HoveredIdPreviousFrameItemCount;
                ImGui::Render();
            }
            if (count > 1)
                conflicts.push_back(std::to_string(count) + " itens com o mesmo ID em (" + std::to_string(static_cast<int>(x))
                    + ", " + std::to_string(static_cast<int>(y)) + ")");
        }
    }
    io.MousePos = { -FLT_MAX, -FLT_MAX };
    return conflicts;
}

const std::vector<std::string>& HeadlessUi::errors() const {
    return imguiErrors;
}

} // namespace MatterEngine::TestSupport
