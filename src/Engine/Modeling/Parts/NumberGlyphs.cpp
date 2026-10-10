#include "Engine/Modeling/Parts/NumberGlyphs.hpp"

#include <algorithm>

namespace MatterEngine::Modeling {

namespace {

using Loop = std::vector<Point2>;

// O retangulo [x0, x1] x [y0, y1] com os cantos chanfrados (c).
Loop chamferedBox(float x0, float y0, float x1, float y1, float c) {
    return { { x0 + c, y0 }, { x1 - c, y0 }, { x1, y0 + c }, { x1, y1 - c }, { x1 - c, y1 }, { x0 + c, y1 }, { x0, y1 - c },
        { x0, y0 + c } };
}

} // namespace

DigitGlyph digitGlyph(char digit, float stroke) {
    const float s = std::clamp(stroke, 0.08f, 0.28f);
    const float w = 0.6f, c = 0.17f, ci = 0.04f;
    // A barra do meio (centrada um pouco acima da metade, como na fonte).
    const float m = 0.5f, lo = m - s * 0.5f, hi = m + s * 0.5f;
    DigitGlyph glyph;
    glyph.width = w;
    switch (digit) {
    case '1': {
        // A haste com a bandeira no alto (para a esquerda) e a base larga.
        glyph.width = 0.42f;
        const float left = 0.16f, right = left + std::max(s, 0.17f), base = 0.18f;
        glyph.fills.push_back({ { 0.0f, 0.0f }, { 0.42f, 0.0f }, { 0.42f, base }, { right, base }, { right, 1.0f },
            { 0.07f, 1.0f }, { 0.0f, 0.93f }, { 0.0f, 0.72f }, { left, 0.8f }, { left, base }, { 0.0f, base } });
        break;
    }
    case '2':
        glyph.fills.push_back({ { 0.0f, 0.0f }, { w, 0.0f }, { w, s }, { s, s }, { s, lo }, { w - c, lo }, { w, lo + c },
            { w, 1.0f - c }, { w - c, 1.0f }, { c, 1.0f }, { 0.0f, 1.0f - c }, { 0.0f, 1.0f - s - 0.12f },
            { s, 1.0f - s - 0.12f }, { s, 1.0f - s }, { w - s, 1.0f - s }, { w - s, hi }, { 0.0f, hi } });
        break;
    case '3':
        glyph.fills.push_back({ { 0.0f, 0.0f }, { w - c, 0.0f }, { w, c }, { w, 1.0f - c }, { w - c, 1.0f }, { 0.0f, 1.0f },
            { 0.0f, 1.0f - s }, { w - s, 1.0f - s }, { w - s, hi }, { 0.15f, hi }, { 0.15f, lo }, { w - s, lo }, { w - s, s },
            { 0.0f, s } });
        break;
    case '4':
        glyph.fills.push_back({ { w - s, 0.0f }, { w, 0.0f }, { w, 1.0f }, { w - s, 1.0f }, { w - s, hi }, { s, hi }, { s, 1.0f },
            { 0.0f, 1.0f }, { 0.0f, lo }, { w - s, lo } });
        break;
    case '5':
        glyph.fills.push_back({ { 0.0f, 0.0f }, { w - c, 0.0f }, { w, c }, { w, hi - c }, { w - c, hi }, { s, hi },
            { s, 1.0f - s }, { w, 1.0f - s }, { w, 1.0f }, { 0.0f, 1.0f }, { 0.0f, lo }, { w - s, lo }, { w - s, s },
            { 0.0f, s } });
        break;
    case '6':
        glyph.fills.push_back({ { c, 0.0f }, { w - c, 0.0f }, { w, c }, { w, hi - c }, { w - c, hi }, { s, hi }, { s, 1.0f - s },
            { w, 1.0f - s }, { w, 1.0f }, { c, 1.0f }, { 0.0f, 1.0f - c }, { 0.0f, c } });
        glyph.holes.push_back(chamferedBox(s, s, w - s, lo, ci));
        break;
    case '7':
        glyph.fills.push_back({ { w - s, 0.0f }, { w, 0.0f }, { w, 1.0f - c }, { w - c, 1.0f }, { 0.0f, 1.0f }, { 0.0f, 1.0f - s },
            { w - s, 1.0f - s } });
        break;
    case '8':
        glyph.fills.push_back(chamferedBox(0.0f, 0.0f, w, 1.0f, c));
        glyph.holes.push_back(chamferedBox(s, s, w - s, lo, ci));
        glyph.holes.push_back(chamferedBox(s, hi, w - s, 1.0f - s, ci));
        break;
    case '9':
        glyph.fills.push_back({ { 0.0f, 0.0f }, { w - c, 0.0f }, { w, c }, { w, 1.0f - c }, { w - c, 1.0f }, { c, 1.0f },
            { 0.0f, 1.0f - c }, { 0.0f, lo + c }, { c, lo }, { w - s, lo }, { w - s, s }, { 0.0f, s } });
        glyph.holes.push_back(chamferedBox(s, hi, w - s, 1.0f - s, ci));
        break;
    default:
        // O 0: o octogono alongado com o furo de cantos quebrados.
        glyph.fills.push_back(chamferedBox(0.0f, 0.0f, w, 1.0f, c));
        glyph.holes.push_back(chamferedBox(s + 0.015f, s - 0.01f, w - s, 1.0f - s + 0.01f, ci));
        break;
    }
    return glyph;
}

std::vector<ImprintLoop> numberLoops(std::string_view text, const NumberLayout& layout, int numberMaterial, int pieceMaterial) {
    std::vector<DigitGlyph> glyphs;
    float total = 0.0f;
    for (const char c : text) {
        glyphs.push_back(digitGlyph(c, layout.stroke));
        total += glyphs.back().width + (glyphs.size() > 1 ? layout.gap : 0.0f);
    }
    const auto place = [&](Point2 p, float offset) {
        const float u = (offset + p.x - total * 0.5f) * layout.height, v = (p.y - 0.5f) * layout.height;
        return layout.center + layout.right * u + layout.up * v;
    };
    std::vector<ImprintLoop> loops;
    float offset = 0.0f;
    for (const DigitGlyph& glyph : glyphs) {
        for (const auto& [shapes, material] :
            { std::pair { &glyph.fills, numberMaterial }, std::pair { &glyph.holes, pieceMaterial } })
            for (const Loop& shape : *shapes) {
                ImprintLoop loop;
                for (const Point2& p : shape) loop.points.push_back(place(p, offset));
                loop.hole = false;
                loop.material = material;
                loops.push_back(std::move(loop));
            }
        offset += glyph.width + layout.gap;
    }
    return loops;
}

} // namespace MatterEngine::Modeling
