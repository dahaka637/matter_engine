// O .ttf da aparencia: o stb_truetype estatico (a modelagem e o ImGui tem as
// copias deles; esta nao exporta simbolo nenhum).
#define STBTT_STATIC
#define STB_TRUETYPE_IMPLEMENTATION
#include "stb_truetype.h"

#include "Engine/Appearance/Compose/KitDesign.hpp"

#include <fstream>
#include <iterator>

namespace MatterEngine::Appearance {

std::optional<GlyphFont> loadTrueTypeFont(const std::filesystem::path& path, std::string* error) {
    std::ifstream file(path, std::ios::binary);
    const std::vector<unsigned char> bytes((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
    stbtt_fontinfo info;
    if (bytes.empty() || !stbtt_InitFont(&info, bytes.data(), stbtt_GetFontOffsetForIndex(bytes.data(), 0))) {
        if (error != nullptr) *error = "fonte TrueType invalida: " + path.string();
        return std::nullopt;
    }
    // (A altura das maiusculas - a do "H" - vira 1.)
    int x0 = 0, y0 = 0, x1 = 0, y1 = 0;
    if (!stbtt_GetCodepointBox(&info, 'H', &x0, &y0, &x1, &y1) || y1 <= 0) {
        if (error != nullptr) *error = "fonte TrueType sem o H: " + path.string();
        return std::nullopt;
    }
    const float scale = 1.0f / static_cast<float>(y1);
    GlyphFont font;
    font.id = path.stem().string();
    const auto add = [&](char32_t code) {
        if (stbtt_FindGlyphIndex(&info, static_cast<int>(code)) == 0) return;
        int advance = 0, bearing = 0;
        stbtt_GetCodepointHMetrics(&info, static_cast<int>(code), &advance, &bearing);
        stbtt_vertex* vertices = nullptr;
        const int count = stbtt_GetCodepointShape(&info, static_cast<int>(code), &vertices);
        GlyphFont::Glyph glyph;
        glyph.advance = static_cast<float>(advance) * scale;
        Vec2 pen { 0.0f, 0.0f };
        std::vector<Vec2>* contour = nullptr;
        for (int i = 0; i < count; ++i) {
            const stbtt_vertex& v = vertices[i];
            const Vec2 p { v.x * scale, v.y * scale };
            if (v.type == STBTT_vmove) {
                contour = &glyph.shape.contours.emplace_back();
                contour->push_back(p);
            } else if (contour != nullptr && v.type == STBTT_vline) {
                contour->push_back(p);
            } else if (contour != nullptr && v.type == STBTT_vcurve) {
                const Vec2 c { v.cx * scale, v.cy * scale };
                for (int k = 1; k <= 8; ++k) {
                    const float t = static_cast<float>(k) / 8.0f, u = 1.0f - t;
                    contour->push_back(pen * (u * u) + c * (2.0f * u * t) + p * (t * t));
                }
            } else if (contour != nullptr && v.type == STBTT_vcubic) {
                const Vec2 c1 { v.cx * scale, v.cy * scale }, c2 { v.cx1 * scale, v.cy1 * scale };
                for (int k = 1; k <= 8; ++k) {
                    const float t = static_cast<float>(k) / 8.0f, u = 1.0f - t;
                    contour->push_back(pen * (u * u * u) + c1 * (3.0f * u * u * t) + c2 * (3.0f * u * t * t) + p * (t * t * t));
                }
            }
            pen = p;
        }
        stbtt_FreeShape(&info, vertices);
        glyph.shape.updateBounds();
        if (!glyph.shape.empty() || code == U' ') font.glyphs.emplace(code, std::move(glyph));
    };
    // ASCII e o Latin-1 (os acentos dos nomes).
    for (char32_t code = 32; code < 127; ++code) add(code);
    for (char32_t code = 0xC0; code <= 0xFF; ++code) add(code);
    return font;
}

} // namespace MatterEngine::Appearance
