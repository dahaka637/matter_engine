#include "Engine/Modeling/Paint/PaintFont.hpp"

#include "stb_truetype.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iterator>
#include <vector>

namespace MatterEngine::Modeling {

namespace {

using Loop = std::vector<Point2>;

// O retangulo com os cantos chanfrados (o de fora dos algarismos).
Loop chamfered(float x0, float y0, float x1, float y1, float c) {
    return { { x0 + c, y0 }, { x1 - c, y0 }, { x1, y0 + c }, { x1, y1 - c }, { x1 - c, y1 }, { x0 + c, y1 }, { x0, y1 - c }, { x0, y0 + c } };
}

// Um furo: o retangulo de cantos arredondados no sentido horario.
Loop counter(float x0, float y0, float x1, float y1, float radius) {
    Shape2D box = rectangleShape({ x0, y0 }, { x1, y1 }, 0.0f, radius);
    Loop loop = box.contours.front();
    std::reverse(loop.begin(), loop.end());
    return loop;
}

PaintFont::Glyph glyphOf(std::vector<Loop> fills, std::vector<Loop> holes, float advance) {
    PaintFont::Glyph glyph;
    for (Loop& fill : fills) {
        if (contourArea(fill) < 0.0f) std::reverse(fill.begin(), fill.end());
        glyph.shape.contours.push_back(std::move(fill));
    }
    for (Loop& hole : holes) {
        if (contourArea(hole) > 0.0f) std::reverse(hole.begin(), hole.end());
        glyph.shape.contours.push_back(std::move(hole));
    }
    glyph.advance = advance;
    return glyph;
}

// O proximo codigo do texto UTF-8 (o invalido vira "?").
char32_t nextCodepoint(std::string_view text, std::size_t& at) {
    const auto byte = [&](std::size_t i) { return static_cast<unsigned char>(text[i]); };
    const unsigned char first = byte(at++);
    if (first < 0x80) return first;
    int extra = first >= 0xF0 ? 3 : first >= 0xE0 ? 2 : first >= 0xC0 ? 1 : -1;
    if (extra < 0 || at + static_cast<std::size_t>(extra) > text.size()) return U'?';
    char32_t code = first & (0x3F >> extra);
    for (int k = 0; k < extra; ++k) code = (code << 6) | (byte(at++) & 0x3F);
    return code;
}

} // namespace

const PaintFont& PaintFont::block() {
    static const PaintFont font = [] {
        PaintFont out;
        // Medidas da referencia (o "10" das costas, 110 px de altura): o
        // traco 0,19; o chanfro de fora 0,136; o "0" com 0,598 de largura e o
        // furo de cantos redondos (0,032); o "1" com a haste de 0,197, a base
        // de 0,186 de altura e a bandeira chanfrada a 45 graus, com o lado de
        // baixo subindo ate a haste.
        const float s = 0.19f, w = 0.598f, c = 0.136f, r = 0.032f;
        const float lo = 0.5f - 0.5f * s, hi = 0.5f + 0.5f * s;
        out.m_glyphs[U'0'] = glyphOf({ chamfered(0.0f, 0.0f, w, 1.0f, c) }, { counter(0.199f, 0.19f, 0.405f, 0.817f, r) }, w);
        out.m_glyphs[U'1'] = glyphOf({ { { 0.019f, 0.0f }, { 0.42f, 0.0f }, { 0.42f, 0.186f }, { 0.314f, 0.186f }, { 0.314f, 1.0f },
                                         { 0.128f, 1.0f }, { 0.0f, 0.872f }, { 0.0f, 0.702f }, { 0.117f, 0.766f }, { 0.117f, 0.186f },
                                         { 0.019f, 0.186f } } },
            {}, 0.42f);
        // Os outros no mesmo desenho (barras de "s", chanfros de "c").
        out.m_glyphs[U'2'] = glyphOf({ { { 0.0f, 0.0f }, { w, 0.0f }, { w, s }, { s, s }, { s, lo }, { w - c, lo }, { w, lo + c }, { w, 1.0f - c },
                                         { w - c, 1.0f }, { c, 1.0f }, { 0.0f, 1.0f - c }, { 0.0f, 1.0f - s - 0.12f }, { s, 1.0f - s - 0.12f },
                                         { s, 1.0f - s }, { w - s, 1.0f - s }, { w - s, hi }, { 0.0f, hi } } },
            {}, w);
        out.m_glyphs[U'3'] = glyphOf({ { { 0.0f, 0.0f }, { w - c, 0.0f }, { w, c }, { w, 1.0f - c }, { w - c, 1.0f }, { 0.0f, 1.0f },
                                         { 0.0f, 1.0f - s }, { w - s, 1.0f - s }, { w - s, hi }, { 0.15f, hi }, { 0.15f, lo }, { w - s, lo },
                                         { w - s, s }, { 0.0f, s } } },
            {}, w);
        out.m_glyphs[U'4'] = glyphOf({ { { w - s, 0.0f }, { w, 0.0f }, { w, 1.0f }, { w - s, 1.0f }, { w - s, hi }, { s, hi }, { s, 1.0f },
                                         { 0.0f, 1.0f }, { 0.0f, lo }, { w - s, lo } } },
            {}, w);
        out.m_glyphs[U'5'] = glyphOf({ { { 0.0f, 0.0f }, { w - c, 0.0f }, { w, c }, { w, hi - c }, { w - c, hi }, { s, hi }, { s, 1.0f - s },
                                         { w, 1.0f - s }, { w, 1.0f }, { 0.0f, 1.0f }, { 0.0f, lo }, { w - s, lo }, { w - s, s }, { 0.0f, s } } },
            {}, w);
        out.m_glyphs[U'6'] = glyphOf({ { { c, 0.0f }, { w - c, 0.0f }, { w, c }, { w, hi - c }, { w - c, hi }, { s, hi }, { s, 1.0f - s },
                                         { w, 1.0f - s }, { w, 1.0f }, { c, 1.0f }, { 0.0f, 1.0f - c }, { 0.0f, c } } },
            { counter(s, s, w - s, lo, r) }, w);
        out.m_glyphs[U'7'] = glyphOf({ { { w - s, 0.0f }, { w, 0.0f }, { w, 1.0f - c }, { w - c, 1.0f }, { 0.0f, 1.0f }, { 0.0f, 1.0f - s },
                                         { w - s, 1.0f - s } } },
            {}, w);
        out.m_glyphs[U'8'] = glyphOf({ chamfered(0.0f, 0.0f, w, 1.0f, c) }, { counter(s, s, w - s, lo, r), counter(s, hi, w - s, 1.0f - s, r) }, w);
        out.m_glyphs[U'9'] = glyphOf({ { { 0.0f, 0.0f }, { w - c, 0.0f }, { w, c }, { w, 1.0f - c }, { w - c, 1.0f }, { c, 1.0f },
                                         { 0.0f, 1.0f - c }, { 0.0f, lo + c }, { c, lo }, { w - s, lo }, { w - s, s }, { 0.0f, s } } },
            { counter(s, hi, w - s, 1.0f - s, r) }, w);
        out.m_glyphs[U' '] = glyphOf({}, {}, 0.3f);
        return out;
    }();
    return font;
}

std::optional<PaintFont> PaintFont::loadTrueType(const std::filesystem::path& path, std::string* error) {
    std::ifstream file(path, std::ios::binary);
    if (!file) {
        if (error != nullptr) *error = "fonte: nao abriu " + path.string();
        return std::nullopt;
    }
    const std::vector<unsigned char> data((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
    stbtt_fontinfo info;
    if (data.empty() || !stbtt_InitFont(&info, data.data(), stbtt_GetFontOffsetForIndex(data.data(), 0))) {
        if (error != nullptr) *error = "fonte: TrueType invalida " + path.string();
        return std::nullopt;
    }
    // A altura das maiusculas (o alto do "H") vira 1.
    int x0 = 0, y0 = 0, x1 = 0, y1 = 0;
    if (!stbtt_GetCodepointBox(&info, 'H', &x0, &y0, &x1, &y1) || y1 <= 0) {
        int ascent = 0, descent = 0, gap = 0;
        stbtt_GetFontVMetrics(&info, &ascent, &descent, &gap);
        y1 = std::max(1, static_cast<int>(0.7f * static_cast<float>(ascent)));
    }
    const float scale = 1.0f / static_cast<float>(y1);
    PaintFont font;
    std::vector<char32_t> codes;
    for (char32_t code = 32; code < 256; ++code) codes.push_back(code);
    for (const char32_t code : codes) {
        if (stbtt_FindGlyphIndex(&info, static_cast<int>(code)) == 0 && code != U' ') continue;
        Glyph glyph;
        int advance = 0, bearing = 0;
        stbtt_GetCodepointHMetrics(&info, static_cast<int>(code), &advance, &bearing);
        glyph.advance = static_cast<float>(advance) * scale;
        stbtt_vertex* vertices = nullptr;
        const int count = stbtt_GetCodepointShape(&info, static_cast<int>(code), &vertices);
        Loop loop;
        Point2 last {};
        const auto close = [&] {
            if (loop.size() >= 3) glyph.shape.contours.push_back(loop);
            loop.clear();
        };
        for (int i = 0; i < count; ++i) {
            const stbtt_vertex& v = vertices[i];
            const Point2 to { static_cast<float>(v.x) * scale, static_cast<float>(v.y) * scale };
            if (v.type == STBTT_vmove) {
                close();
                loop.push_back(to);
            } else if (v.type == STBTT_vline) {
                loop.push_back(to);
            } else if (v.type == STBTT_vcurve) {
                const Point2 control { static_cast<float>(v.cx) * scale, static_cast<float>(v.cy) * scale };
                for (int k = 1; k <= 8; ++k) {
                    const float t = static_cast<float>(k) / 8.0f, u = 1.0f - t;
                    loop.push_back({ u * u * last.x + 2.0f * u * t * control.x + t * t * to.x, u * u * last.y + 2.0f * u * t * control.y + t * t * to.y });
                }
            } else if (v.type == STBTT_vcubic) {
                const Point2 c0 { static_cast<float>(v.cx) * scale, static_cast<float>(v.cy) * scale };
                const Point2 c1 { static_cast<float>(v.cx1) * scale, static_cast<float>(v.cy1) * scale };
                for (int k = 1; k <= 10; ++k) {
                    const float t = static_cast<float>(k) / 10.0f, u = 1.0f - t;
                    const float a = u * u * u, b = 3.0f * u * u * t, cc = 3.0f * u * t * t, d = t * t * t;
                    loop.push_back({ a * last.x + b * c0.x + cc * c1.x + d * to.x, a * last.y + b * c0.y + cc * c1.y + d * to.y });
                }
            }
            last = to;
        }
        close();
        if (vertices != nullptr) stbtt_FreeShape(&info, vertices);
        // O primeiro ponto repetido no fim (os contornos fecham sozinhos).
        for (Loop& contour : glyph.shape.contours)
            if (contour.size() > 3 && std::abs(contour.front().x - contour.back().x) < 1e-6f && std::abs(contour.front().y - contour.back().y) < 1e-6f)
                contour.pop_back();
        font.m_glyphs[code] = std::move(glyph);
    }
    for (const char32_t a : codes)
        for (const char32_t b : codes) {
            if (!font.m_glyphs.count(a) || !font.m_glyphs.count(b)) continue;
            const int kern = stbtt_GetCodepointKernAdvance(&info, static_cast<int>(a), static_cast<int>(b));
            if (kern != 0) font.m_kerning[{ a, b }] = static_cast<float>(kern) * scale;
        }
    return font;
}

const PaintFont::Glyph* PaintFont::glyph(char32_t codepoint) const {
    const auto found = m_glyphs.find(codepoint);
    return found != m_glyphs.end() ? &found->second : nullptr;
}

Shape2D PaintFont::layout(std::string_view utf8, float spacing, float* width) const {
    Shape2D shape;
    float x = 0.0f, extent = 0.0f;
    char32_t previous = 0;
    for (std::size_t at = 0; at < utf8.size();) {
        const char32_t code = nextCodepoint(utf8, at);
        const Glyph* g = glyph(code);
        if (g == nullptr) g = glyph(U'?');
        if (g == nullptr) continue;
        if (previous != 0) {
            x += spacing;
            if (const auto kern = m_kerning.find({ previous, code }); kern != m_kerning.end()) x += kern->second;
        }
        shape.add(g->shape.transformed(Affine2::translation(x, 0.0f)));
        x += g->advance;
        extent = x;
        previous = code;
    }
    if (width != nullptr) *width = extent;
    return shape;
}

} // namespace MatterEngine::Modeling
