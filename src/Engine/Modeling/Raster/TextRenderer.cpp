#include "Engine/Modeling/Raster/TextRenderer.hpp"

#include "stb_truetype.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iterator>
#include <mutex>
#include <unordered_map>
#include <vector>

#ifndef MATTERMODELING_FONT_PATH
#define MATTERMODELING_FONT_PATH ""
#endif

namespace MatterEngine::Modeling {

namespace {

struct Glyph {
    int width = 0;
    int height = 0;
    int offsetX = 0;
    int offsetY = 0;
    float advance = 0.0f;
    std::vector<std::uint8_t> alpha;
};

// Proximo codepoint de um texto UTF-8 (sequencia invalida vira '?').
std::uint32_t nextCodepoint(std::string_view text, std::size_t& index) {
    const auto byteAt = [&](std::size_t i) { return static_cast<std::uint8_t>(text[i]); };
    const std::uint8_t first = byteAt(index);
    if (first < 0x80) {
        ++index;
        return first;
    }
    std::size_t extra = 0;
    std::uint32_t codepoint = 0;
    if ((first & 0xE0) == 0xC0) { extra = 1; codepoint = first & 0x1Fu; }
    else if ((first & 0xF0) == 0xE0) { extra = 2; codepoint = first & 0x0Fu; }
    else if ((first & 0xF8) == 0xF0) { extra = 3; codepoint = first & 0x07u; }
    else {
        ++index;
        return '?';
    }
    if (index + extra >= text.size()) {
        index = text.size();
        return '?';
    }
    for (std::size_t i = 1; i <= extra; ++i) {
        const std::uint8_t continuation = byteAt(index + i);
        if ((continuation & 0xC0) != 0x80) {
            index += i;
            return '?';
        }
        codepoint = (codepoint << 6) | (continuation & 0x3Fu);
    }
    index += extra + 1;
    return codepoint;
}

} // namespace

struct TextRenderer::Impl {
    std::vector<unsigned char> data;
    stbtt_fontinfo info {};
    bool loaded = false;
    int ascent = 0;
    int descent = 0;
    int lineGap = 0;
    mutable std::mutex mutex;
    mutable std::unordered_map<std::uint64_t, Glyph> cache;

    const Glyph& glyph(std::uint32_t codepoint, float pixelHeight) const {
        const std::uint64_t sizeKey = static_cast<std::uint64_t>(std::lround(pixelHeight * 4.0f));
        const std::uint64_t key = (sizeKey << 32) | codepoint;
        const auto found = cache.find(key);
        if (found != cache.end()) return found->second;
        Glyph glyph;
        const float scale = stbtt_ScaleForPixelHeight(&info, pixelHeight);
        int advance = 0, bearing = 0;
        stbtt_GetCodepointHMetrics(&info, static_cast<int>(codepoint), &advance, &bearing);
        glyph.advance = static_cast<float>(advance) * scale;
        unsigned char* bitmap = stbtt_GetCodepointBitmap(&info, scale, scale, static_cast<int>(codepoint),
            &glyph.width, &glyph.height, &glyph.offsetX, &glyph.offsetY);
        if (bitmap != nullptr) {
            glyph.alpha.assign(bitmap, bitmap + static_cast<std::size_t>(glyph.width) * glyph.height);
            stbtt_FreeBitmap(bitmap, nullptr);
        }
        return cache.emplace(key, std::move(glyph)).first->second;
    }
};

TextRenderer::TextRenderer() : m_impl(std::make_unique<Impl>()) {}
TextRenderer::~TextRenderer() = default;

bool TextRenderer::load(const std::string& fontPath) {
    std::ifstream file(fontPath, std::ios::binary);
    if (!file) return false;
    std::vector<unsigned char> data { std::istreambuf_iterator<char>(file), std::istreambuf_iterator<char>() };
    if (data.empty()) return false;
    std::lock_guard lock(m_impl->mutex);
    m_impl->data = std::move(data);
    m_impl->cache.clear();
    m_impl->loaded = stbtt_InitFont(&m_impl->info, m_impl->data.data(),
                         stbtt_GetFontOffsetForIndex(m_impl->data.data(), 0))
        != 0;
    if (m_impl->loaded) stbtt_GetFontVMetrics(&m_impl->info, &m_impl->ascent, &m_impl->descent, &m_impl->lineGap);
    return m_impl->loaded;
}

bool TextRenderer::loaded() const {
    return m_impl->loaded;
}

TextSize TextRenderer::measure(std::string_view text, float pixelHeight) const {
    if (!m_impl->loaded || text.empty()) return { 0.0f, pixelHeight };
    std::lock_guard lock(m_impl->mutex);
    float width = 0.0f;
    for (std::size_t index = 0; index < text.size();) width += m_impl->glyph(nextCodepoint(text, index), pixelHeight).advance;
    return { width, pixelHeight };
}

void TextRenderer::draw(Image& image, float x, float y, std::string_view text, float pixelHeight,
    Color color) const {
    if (!m_impl->loaded || text.empty()) return;
    std::lock_guard lock(m_impl->mutex);
    const float scale = stbtt_ScaleForPixelHeight(&m_impl->info, pixelHeight);
    const float baseline = y + static_cast<float>(m_impl->ascent) * scale;
    float penX = x;
    for (std::size_t index = 0; index < text.size();) {
        const Glyph& glyph = m_impl->glyph(nextCodepoint(text, index), pixelHeight);
        const int originX = static_cast<int>(std::lround(penX)) + glyph.offsetX;
        const int originY = static_cast<int>(std::lround(baseline)) + glyph.offsetY;
        for (int row = 0; row < glyph.height; ++row) {
            for (int column = 0; column < glyph.width; ++column) {
                const std::uint8_t alpha = glyph.alpha[static_cast<std::size_t>(row) * glyph.width + column];
                if (alpha != 0) image.blendPixel(originX + column, originY + row, color, alpha / 255.0f);
            }
        }
        penX += glyph.advance;
    }
}

void TextRenderer::drawOutlined(Image& image, float x, float y, std::string_view text, float pixelHeight,
    Color color, Color outline) const {
    const float offset = std::max(1.0f, pixelHeight / 14.0f);
    for (int dy = -1; dy <= 1; ++dy)
        for (int dx = -1; dx <= 1; ++dx)
            if (dx != 0 || dy != 0)
                draw(image, x + static_cast<float>(dx) * offset, y + static_cast<float>(dy) * offset, text,
                    pixelHeight, outline);
    draw(image, x, y, text, pixelHeight, color);
}

TextSize TextRenderer::drawLabel(Image& image, float x, float y, std::string_view text, float pixelHeight,
    Color color, Color background, float padding) const {
    const TextSize size = measure(text, pixelHeight);
    const TextSize box { size.width + padding * 2.0f, pixelHeight * 1.18f + padding * 2.0f };
    image.fillRect(static_cast<int>(std::lround(x)), static_cast<int>(std::lround(y)),
        static_cast<int>(std::lround(x + box.width)), static_cast<int>(std::lround(y + box.height)), background);
    draw(image, x + padding, y + padding, text, pixelHeight, color);
    return box;
}

const TextRenderer& TextRenderer::standard() {
    static TextRenderer renderer;
    static std::once_flag once;
    std::call_once(once, [] { renderer.load(MATTERMODELING_FONT_PATH); });
    return renderer;
}

} // namespace MatterEngine::Modeling
