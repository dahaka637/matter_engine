#include "Engine/Modeling/Objects/PixelCanvas.hpp"

#include <algorithm>
#include <cmath>

namespace MatterEngine::Modeling {

float pixelNoise(int x, int y, std::uint32_t seed) {
    std::uint32_t h = static_cast<std::uint32_t>(x) * 0x8da6b343u ^ static_cast<std::uint32_t>(y) * 0xd8163841u
        ^ (seed + 0x9e3779b9u) * 0xcb1ab31fu;
    h ^= h >> 13;
    h *= 0x5bd1e995u;
    h ^= h >> 15;
    h *= 0x27d4eb2du;
    h ^= h >> 16;
    return static_cast<float>(h & 0xFFFFFFu) / static_cast<float>(0x1000000u);
}

PixelCanvas::PixelCanvas(int width, int height, Color fill) : m_image(std::max(1, width), std::max(1, height), fill) {}

Color PixelCanvas::at(int x, int y) const {
    return m_image.pixel(x, y);
}

void PixelCanvas::set(int x, int y, Color color) {
    m_image.setPixel(x, y, color);
}

void PixelCanvas::fillRect(int x0, int y0, int x1, int y1, Color color) {
    for (int y = std::max(0, std::min(y0, y1)); y <= std::min(height() - 1, std::max(y0, y1)); ++y)
        for (int x = std::max(0, std::min(x0, x1)); x <= std::min(width() - 1, std::max(x0, x1)); ++x) set(x, y, color);
}

void PixelCanvas::strokeRect(int x0, int y0, int x1, int y1, Color color) {
    for (int x = std::min(x0, x1); x <= std::max(x0, x1); ++x) {
        set(x, y0, color);
        set(x, y1, color);
    }
    for (int y = std::min(y0, y1); y <= std::max(y0, y1); ++y) {
        set(x0, y, color);
        set(x1, y, color);
    }
}

void PixelCanvas::line(float x0, float y0, float x1, float y1, float thickness, Color color) {
    const float half = std::max(0.5f, thickness * 0.5f);
    const int minimumX = static_cast<int>(std::floor(std::min(x0, x1) - half)) - 1;
    const int maximumX = static_cast<int>(std::ceil(std::max(x0, x1) + half)) + 1;
    const int minimumY = static_cast<int>(std::floor(std::min(y0, y1) - half)) - 1;
    const int maximumY = static_cast<int>(std::ceil(std::max(y0, y1) + half)) + 1;
    const float dx = x1 - x0, dy = y1 - y0;
    const float lengthSquared = std::max(1e-6f, dx * dx + dy * dy);
    for (int y = minimumY; y <= maximumY; ++y)
        for (int x = minimumX; x <= maximumX; ++x) {
            const float px = static_cast<float>(x) + 0.5f, py = static_cast<float>(y) + 0.5f;
            const float t = std::clamp(((px - x0) * dx + (py - y0) * dy) / lengthSquared, 0.0f, 1.0f);
            const float ex = px - (x0 + dx * t), ey = py - (y0 + dy * t);
            if (ex * ex + ey * ey <= half * half) set(x, y, color);
        }
}

void PixelCanvas::shade(int x, int y, float factor) {
    if (!m_image.inside(x, y)) return;
    set(x, y, scaled(at(x, y), factor));
}

void PixelCanvas::shadeRect(int x0, int y0, int x1, int y1, float factor) {
    for (int y = std::min(y0, y1); y <= std::max(y0, y1); ++y)
        for (int x = std::min(x0, x1); x <= std::max(x0, x1); ++x) shade(x, y, factor);
}

void PixelCanvas::jitter(int x0, int y0, int x1, int y1, float amount, std::uint32_t seed) {
    for (int y = std::min(y0, y1); y <= std::max(y0, y1); ++y)
        for (int x = std::min(x0, x1); x <= std::max(x0, x1); ++x)
            shade(x, y, 1.0f + (pixelNoise(x, y, seed) * 2.0f - 1.0f) * amount);
}

void PixelCanvas::blit(const PixelCanvas& source, int x, int y) {
    for (int sy = 0; sy < source.height(); ++sy)
        for (int sx = 0; sx < source.width(); ++sx) set(x + sx, y + sy, source.at(sx, sy));
}

Image PixelCanvas::upscaled(int factor) const {
    factor = std::max(1, factor);
    Image result(width() * factor, height() * factor);
    for (int y = 0; y < result.height(); ++y)
        for (int x = 0; x < result.width(); ++x) result.setPixel(x, y, at(x / factor, y / factor));
    return result;
}

} // namespace MatterEngine::Modeling
