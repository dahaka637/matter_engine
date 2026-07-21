#include "Engine/UI/ImGuiFontLibrary.hpp"

#include "Engine/Core/Log.hpp"
#include "Engine/Platform/PlatformServices.hpp"

#include "imgui.h"

#include <filesystem>
#include <string>

namespace MatterEngine::UI {

namespace {

constexpr const char* FontRelativePath = "assets/fonts/font-awesome/fa-solid-900.otf";

std::filesystem::path locateFontAwesome() {
    const std::filesystem::path bundled =
        std::filesystem::path(Platform::executableBasePath()) / FontRelativePath;
    if (std::filesystem::exists(bundled)) {
        return bundled;
    }

    const std::filesystem::path development =
        std::filesystem::path(MATTERENGINE_ASSETS_DIR) / "fonts/font-awesome/fa-solid-900.otf";
    return std::filesystem::exists(development) ? development : std::filesystem::path {};
}

} // namespace

void configureImGuiFonts() {
    ImGuiIO& io = ImGui::GetIO();
    // ImGui 1.92 permite tamanhos dinâmicos. Como o Font Awesome abaixo usa
    // uma referência explícita de 13 px, a fonte de destino deve declarar a
    // mesma referência antes do merge; misturar tamanho implícito e explícito
    // é rejeitado pelo atlas para evitar escala ambígua.
    ImFontConfig defaultConfig;
    defaultConfig.SizePixels = 13.0f;
    io.Fonts->AddFontDefaultVector(&defaultConfig);

    const std::filesystem::path fontPath = locateFontAwesome();
    if (fontPath.empty()) {
        Log::warn("Font Awesome Free font not found; UI icons will use fallback glyphs.");
        return;
    }

    ImFontConfig config;
    config.MergeMode = true;
    config.PixelSnapH = true;
    config.GlyphMinAdvanceX = 13.0f;

    const std::string path = fontPath.string();
    if (io.Fonts->AddFontFromFileTTF(path.c_str(), 13.0f, &config) == nullptr) {
        Log::warn("Font Awesome Free could not be added to the ImGui font atlas.");
        return;
    }
    Log::info("Font Awesome Free 7.3.0 merged into the ImGui font atlas.");
}

} // namespace MatterEngine::UI
