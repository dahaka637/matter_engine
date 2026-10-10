#include "Engine/Modeling/Paint/PaintImage.hpp"

#include <algorithm>
#include <cmath>

namespace MatterEngine::Modeling {

PaintImage::PaintImage(const Image& image) {
    if (image.empty()) return;
    Level first;
    first.width = image.width();
    first.height = image.height();
    first.texels.resize(static_cast<std::size_t>(first.width) * static_cast<std::size_t>(first.height));
    for (int y = 0; y < first.height; ++y)
        for (int x = 0; x < first.width; ++x) {
            const Color c = image.pixel(x, y);
            first.texels[static_cast<std::size_t>(y) * static_cast<std::size_t>(first.width) + static_cast<std::size_t>(x)] = { c.r * c.a, c.g * c.a, c.b * c.a, c.a };
        }
    m_levels.push_back(std::move(first));
    // As reducoes: a media de cada quadrado de 2 x 2 (a ultima coluna ou linha
    // impar entra na vizinha).
    while (m_levels.back().width > 1 || m_levels.back().height > 1) {
        const Level& above = m_levels.back();
        Level next;
        next.width = std::max(1, above.width / 2);
        next.height = std::max(1, above.height / 2);
        next.texels.assign(static_cast<std::size_t>(next.width) * static_cast<std::size_t>(next.height), Color { 0.0f, 0.0f, 0.0f, 0.0f });
        std::vector<float> weight(next.texels.size(), 0.0f);
        for (int y = 0; y < above.height; ++y)
            for (int x = 0; x < above.width; ++x) {
                const std::size_t to = static_cast<std::size_t>(std::min(next.height - 1, y / 2)) * static_cast<std::size_t>(next.width)
                    + static_cast<std::size_t>(std::min(next.width - 1, x / 2));
                const Color c = above.texels[static_cast<std::size_t>(y) * static_cast<std::size_t>(above.width) + static_cast<std::size_t>(x)];
                next.texels[to] = { next.texels[to].r + c.r, next.texels[to].g + c.g, next.texels[to].b + c.b, next.texels[to].a + c.a };
                weight[to] += 1.0f;
            }
        for (std::size_t i = 0; i < next.texels.size(); ++i) {
            const float w = std::max(1.0f, weight[i]);
            next.texels[i] = { next.texels[i].r / w, next.texels[i].g / w, next.texels[i].b / w, next.texels[i].a / w };
        }
        m_levels.push_back(std::move(next));
    }
}

Color PaintImage::bilinear(const Level& level, float u, float v) const {
    const float x = u * static_cast<float>(level.width) - 0.5f, y = v * static_cast<float>(level.height) - 0.5f;
    const int x0 = static_cast<int>(std::floor(x)), y0 = static_cast<int>(std::floor(y));
    const float tx = x - static_cast<float>(x0), ty = y - static_cast<float>(y0);
    const auto at = [&](int px, int py) {
        px = std::clamp(px, 0, level.width - 1);
        py = std::clamp(py, 0, level.height - 1);
        return level.texels[static_cast<std::size_t>(py) * static_cast<std::size_t>(level.width) + static_cast<std::size_t>(px)];
    };
    const auto lerp = [](Color p, Color q, float t) {
        return Color { p.r + (q.r - p.r) * t, p.g + (q.g - p.g) * t, p.b + (q.b - p.b) * t, p.a + (q.a - p.a) * t };
    };
    return lerp(lerp(at(x0, y0), at(x0 + 1, y0), tx), lerp(at(x0, y0 + 1), at(x0 + 1, y0 + 1), tx), ty);
}

Color PaintImage::sample(float u, float v, float size) const {
    if (m_levels.empty() || u < 0.0f || u > 1.0f || v < 0.0f || v > 1.0f) return { 0.0f, 0.0f, 0.0f, 0.0f };
    // O nivel com o pixel do tamanho da amostra (entre dois, a media).
    const float pixels = std::max(1.0f, size * static_cast<float>(width()));
    const float level = std::clamp(std::log2(pixels), 0.0f, static_cast<float>(m_levels.size() - 1));
    const auto low = static_cast<std::size_t>(level);
    const std::size_t high = std::min(m_levels.size() - 1, low + 1);
    const float t = level - static_cast<float>(low);
    const Color a = bilinear(m_levels[low], u, v), b = bilinear(m_levels[high], u, v);
    const Color c { a.r + (b.r - a.r) * t, a.g + (b.g - a.g) * t, a.b + (b.b - a.b) * t, a.a + (b.a - a.a) * t };
    if (c.a <= 1e-6f) return { 0.0f, 0.0f, 0.0f, 0.0f };
    return { c.r / c.a, c.g / c.a, c.b / c.a, c.a };
}

} // namespace MatterEngine::Modeling
