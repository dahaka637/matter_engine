#include "Engine/Modeling/Raster/Image.hpp"

#include "Engine/Modeling/Geometry/Parallel.hpp"

#include <algorithm>
#include <cmath>

namespace MatterEngine::Modeling {

namespace {

std::uint8_t toByte(float value) {
    return static_cast<std::uint8_t>(std::lround(std::clamp(value, 0.0f, 1.0f) * 255.0f));
}

Pixel toPixel(Color color) {
    return { toByte(color.r), toByte(color.g), toByte(color.b), toByte(color.a) };
}

Color toColor(Pixel pixel) {
    return { pixel.r / 255.0f, pixel.g / 255.0f, pixel.b / 255.0f, pixel.a / 255.0f };
}

} // namespace

Color categoricalColor(int index) {
    // Matiz pelo angulo aureo: vizinhos sempre distantes no circulo.
    const float hue = std::fmod(0.07f + static_cast<float>(index) * 0.618034f, 1.0f);
    const float saturation = index % 2 == 0 ? 0.55f : 0.42f;
    const float value = index % 3 == 0 ? 0.92f : 0.80f;
    const float h = hue * 6.0f;
    const float c = value * saturation;
    const float x = c * (1.0f - std::abs(std::fmod(h, 2.0f) - 1.0f));
    const float m = value - c;
    float r = 0, g = 0, b = 0;
    if (h < 1) { r = c; g = x; }
    else if (h < 2) { r = x; g = c; }
    else if (h < 3) { g = c; b = x; }
    else if (h < 4) { g = x; b = c; }
    else if (h < 5) { r = x; b = c; }
    else { r = c; b = x; }
    return { r + m, g + m, b + m, 1.0f };
}

Color heatColor(float value) {
    constexpr Color Stops[] = { rgb(30, 60, 200), rgb(30, 190, 230), rgb(60, 210, 90), rgb(250, 220, 40),
        rgb(240, 50, 40) };
    const float t = std::clamp(value, 0.0f, 1.0f) * 4.0f;
    const int index = std::min(3, static_cast<int>(t));
    return mix(Stops[index], Stops[index + 1], t - static_cast<float>(index));
}

Image::Image(int width, int height, Color fill)
    : m_width(std::max(0, width)), m_height(std::max(0, height)),
      m_pixels(static_cast<std::size_t>(m_width) * static_cast<std::size_t>(m_height), toPixel(fill)) {}

Color Image::pixel(int x, int y) const {
    if (!inside(x, y)) return {};
    return toColor(m_pixels[static_cast<std::size_t>(y) * m_width + x]);
}

Color Image::sample(Uv uv) const {
    if (m_pixels.empty()) return {};
    const float x = uv.u * static_cast<float>(m_width) - 0.5f, y = uv.v * static_cast<float>(m_height) - 0.5f;
    const float fx = std::floor(x), fy = std::floor(y);
    const float tx = x - fx, ty = y - fy;
    const auto at = [&](int px, int py) {
        px = std::clamp(px, 0, m_width - 1);
        py = std::clamp(py, 0, m_height - 1);
        return toColor(m_pixels[static_cast<std::size_t>(py) * m_width + px]);
    };
    const int x0 = static_cast<int>(fx), y0 = static_cast<int>(fy);
    const Color a = at(x0, y0), b = at(x0 + 1, y0), c = at(x0, y0 + 1), d = at(x0 + 1, y0 + 1);
    const auto mix = [](Color p, Color q, float t) {
        return Color { p.r + (q.r - p.r) * t, p.g + (q.g - p.g) * t, p.b + (q.b - p.b) * t, p.a + (q.a - p.a) * t };
    };
    return mix(mix(a, b, tx), mix(c, d, tx), ty);
}

void Image::setPixel(int x, int y, Color color) {
    if (!inside(x, y)) return;
    m_pixels[static_cast<std::size_t>(y) * m_width + x] = toPixel(color);
}

void Image::blendPixel(int x, int y, Color color, float coverage) {
    if (!inside(x, y)) return;
    const float alpha = std::clamp(color.a * coverage, 0.0f, 1.0f);
    if (alpha <= 0.0f) return;
    Pixel& target = m_pixels[static_cast<std::size_t>(y) * m_width + x];
    const auto blend = [alpha](std::uint8_t destination, float source) {
        return toByte(source * alpha + (destination / 255.0f) * (1.0f - alpha));
    };
    target.r = blend(target.r, color.r);
    target.g = blend(target.g, color.g);
    target.b = blend(target.b, color.b);
    target.a = toByte(alpha + (target.a / 255.0f) * (1.0f - alpha));
}

void Image::fill(Color color) {
    std::fill(m_pixels.begin(), m_pixels.end(), toPixel(color));
}

void Image::fillVerticalGradient(Color top, Color bottom) {
    for (int y = 0; y < m_height; ++y) {
        const float t = m_height > 1 ? static_cast<float>(y) / static_cast<float>(m_height - 1) : 0.0f;
        const Pixel row = toPixel(mix(top, bottom, t));
        std::fill_n(m_pixels.begin() + static_cast<std::ptrdiff_t>(y) * m_width, m_width, row);
    }
}

void Image::fillRect(int x0, int y0, int x1, int y1, Color color) {
    x0 = std::max(0, x0);
    y0 = std::max(0, y0);
    x1 = std::min(m_width, x1);
    y1 = std::min(m_height, y1);
    for (int y = y0; y < y1; ++y)
        for (int x = x0; x < x1; ++x) blendPixel(x, y, color);
}

void Image::drawImage(const Image& source, int x, int y, float opacity) {
    for (int sy = 0; sy < source.height(); ++sy) {
        for (int sx = 0; sx < source.width(); ++sx) {
            const Color color = source.pixel(sx, sy);
            blendPixel(x + sx, y + sy, color, opacity);
        }
    }
}

Image Image::downsampled(int factor) const {
    if (factor <= 1) return *this;
    Image result(m_width / factor, m_height / factor);
    const int samples = factor * factor;
    // Em paralelo por linhas (cada thread escreve as dela).
    parallelFor(static_cast<std::size_t>(result.m_height), 16, [&](std::size_t firstRow, std::size_t lastRow) {
        for (int y = static_cast<int>(firstRow); y < static_cast<int>(lastRow); ++y) {
            for (int x = 0; x < result.m_width; ++x) {
                int r = 0, g = 0, b = 0, a = 0;
                for (int sy = 0; sy < factor; ++sy) {
                    const Pixel* row = &m_pixels[static_cast<std::size_t>(y * factor + sy) * m_width + x * factor];
                    for (int sx = 0; sx < factor; ++sx) {
                        r += row[sx].r;
                        g += row[sx].g;
                        b += row[sx].b;
                        a += row[sx].a;
                    }
                }
                result.m_pixels[static_cast<std::size_t>(y) * result.m_width + x] = {
                    static_cast<std::uint8_t>((r + samples / 2) / samples),
                    static_cast<std::uint8_t>((g + samples / 2) / samples),
                    static_cast<std::uint8_t>((b + samples / 2) / samples),
                    static_cast<std::uint8_t>((a + samples / 2) / samples) };
            }
        }
    });
    return result;
}

Image Image::resized(int width, int height) const {
    Image result(width, height);
    if (empty() || width <= 0 || height <= 0) return result;
    for (int y = 0; y < height; ++y) {
        const float sy = (static_cast<float>(y) + 0.5f) * m_height / height - 0.5f;
        const int y0 = std::clamp(static_cast<int>(std::floor(sy)), 0, m_height - 1);
        const int y1 = std::min(y0 + 1, m_height - 1);
        const float fy = std::clamp(sy - static_cast<float>(y0), 0.0f, 1.0f);
        for (int x = 0; x < width; ++x) {
            const float sx = (static_cast<float>(x) + 0.5f) * m_width / width - 0.5f;
            const int x0 = std::clamp(static_cast<int>(std::floor(sx)), 0, m_width - 1);
            const int x1 = std::min(x0 + 1, m_width - 1);
            const float fx = std::clamp(sx - static_cast<float>(x0), 0.0f, 1.0f);
            const Color top = mix(pixel(x0, y0), pixel(x1, y0), fx);
            const Color bottom = mix(pixel(x0, y1), pixel(x1, y1), fx);
            result.setPixel(x, y, mix(top, bottom, fy));
        }
    }
    return result;
}

Image Image::cropped(int x, int y, int width, int height) const {
    Image result(width, height);
    for (int row = 0; row < height; ++row)
        for (int column = 0; column < width; ++column)
            if (inside(x + column, y + row)) result.setPixel(column, row, pixel(x + column, y + row));
    return result;
}

Image Image::fromRgba8(int width, int height, std::span<const std::uint8_t> rgba) {
    Image result(width, height);
    const std::size_t count = std::min(result.m_pixels.size(), rgba.size() / 4);
    for (std::size_t i = 0; i < count; ++i)
        result.m_pixels[i] = { rgba[i * 4], rgba[i * 4 + 1], rgba[i * 4 + 2], rgba[i * 4 + 3] };
    return result;
}

} // namespace MatterEngine::Modeling
