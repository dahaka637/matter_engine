#include "Engine/Modeling/Raster/Rasterizer.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

namespace MatterEngine::Modeling {

namespace {

constexpr float NoDepth = -std::numeric_limits<float>::infinity();

enum Kind : int { Flat = 0, Gradient = 1, DepthOnly = 2, Accumulate = 3, Overlay = 4, Textured = 5, Lit = 6, TexturedLit = 7 };

Pixel packed(Color color) {
    // (Arredonda como lround: os valores sao positivos.)
    const auto byte = [](float value) {
        return static_cast<std::uint8_t>(std::clamp(value, 0.0f, 1.0f) * 255.0f + 0.5f);
    };
    return { byte(color.r), byte(color.g), byte(color.b), byte(color.a) };
}

struct ClipVertex {
    Vec3 camera;
    Color color;
    Vec3 normal;
};

// Sutherland-Hodgman contra z >= near: 0, 3 ou 4 vertices.
int clipNear(const std::array<ClipVertex, 3>& input, float near, std::array<ClipVertex, 4>& output) {
    int count = 0;
    for (int i = 0; i < 3; ++i) {
        const ClipVertex& current = input[static_cast<std::size_t>(i)];
        const ClipVertex& next = input[static_cast<std::size_t>((i + 1) % 3)];
        const bool currentInside = current.camera.z >= near;
        const bool nextInside = next.camera.z >= near;
        if (currentInside && count < 4) output[static_cast<std::size_t>(count++)] = current;
        if (currentInside != nextInside && count < 4) {
            const float t = (near - current.camera.z) / (next.camera.z - current.camera.z);
            output[static_cast<std::size_t>(count++)] = { lerp(current.camera, next.camera, t),
                mix(current.color, next.color, t), lerp(current.normal, next.normal, t) };
        }
    }
    return count;
}

// Percorre os pixels do triangulo (centros), com os pesos baricentricos.
template <typename Shade>
void rasterTriangle(int width, int rowBegin, int rowEnd, const ScreenPoint& a, const ScreenPoint& b,
    const ScreenPoint& c, Shade&& shade) {
    const float area = (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
    if (std::abs(area) < 1e-10f) return;
    const float inverseArea = 1.0f / area;
    const int minX = std::max(0, static_cast<int>(std::floor(std::min({ a.x, b.x, c.x }))));
    const int maxX = std::min(width - 1, static_cast<int>(std::ceil(std::max({ a.x, b.x, c.x }))));
    const int minY = std::max(rowBegin, static_cast<int>(std::floor(std::min({ a.y, b.y, c.y }))));
    const int maxY = std::min(rowEnd - 1, static_cast<int>(std::ceil(std::max({ a.y, b.y, c.y }))));
    if (minX > maxX || minY > maxY) return;
    // Pesos como funcoes lineares de (x, y): incremento constante por pixel.
    const float w0dx = -(c.y - b.y) * inverseArea;
    const float w1dx = -(a.y - c.y) * inverseArea;
    for (int y = minY; y <= maxY; ++y) {
        const float py = static_cast<float>(y) + 0.5f;
        const float startX = static_cast<float>(minX) + 0.5f;
        float w0 = ((c.x - b.x) * (py - b.y) - (c.y - b.y) * (startX - b.x)) * inverseArea;
        float w1 = ((a.x - c.x) * (py - c.y) - (a.y - c.y) * (startX - c.x)) * inverseArea;
        for (int x = minX; x <= maxX; ++x, w0 += w0dx, w1 += w1dx) {
            const float w2 = 1.0f - w0 - w1;
            if (w0 < 0.0f || w1 < 0.0f || w2 < 0.0f) continue;
            shade(x, y, w0, w1, w2);
        }
    }
}

// Distancia de (px, py) ao segmento e o parametro do mais proximo.
float segmentDistance2D(float px, float py, float x0, float y0, float x1, float y1, float& t) {
    const float dx = x1 - x0, dy = y1 - y0;
    const float lengthSquared = dx * dx + dy * dy;
    t = lengthSquared > 1e-12f ? std::clamp(((px - x0) * dx + (py - y0) * dy) / lengthSquared, 0.0f, 1.0f) : 0.0f;
    const float qx = x0 + dx * t - px, qy = y0 + dy * t - py;
    return std::sqrt(qx * qx + qy * qy);
}

// Os pixels perto de um segmento grosso: anda pelo eixo maior e, em cada
// linha (ou coluna), so na faixa em volta do centro - nao na caixa inteira
// (uma linha da grade cruzando a imagem varria a imagem toda).
template <typename Plot>
void forEachLinePixel(float x0, float y0, float x1, float y1, float halfWidth, int width, int rowBegin, int rowEnd,
    Plot&& plot) {
    const float dx = x1 - x0, dy = y1 - y0;
    const float reach = halfWidth + 1.0f;
    if (std::abs(dy) >= std::abs(dx)) {
        const float slope = std::abs(dy) > 1e-6f ? dx / dy : 0.0f;
        const float spread = reach * std::sqrt(1.0f + slope * slope);
        const int minY = std::max(rowBegin, static_cast<int>(std::floor(std::min(y0, y1) - reach)));
        const int maxY = std::min(rowEnd - 1, static_cast<int>(std::ceil(std::max(y0, y1) + reach)));
        for (int y = minY; y <= maxY; ++y) {
            const float t = std::abs(dy) > 1e-6f ? std::clamp((static_cast<float>(y) + 0.5f - y0) / dy, 0.0f, 1.0f) : 0.0f;
            const float center = x0 + dx * t;
            const int minX = std::max(0, static_cast<int>(std::floor(center - spread)));
            const int maxX = std::min(width - 1, static_cast<int>(std::ceil(center + spread)));
            for (int x = minX; x <= maxX; ++x) plot(x, y);
        }
    } else {
        const float slope = dy / dx;
        const float spread = reach * std::sqrt(1.0f + slope * slope);
        const int minX = std::max(0, static_cast<int>(std::floor(std::min(x0, x1) - reach)));
        const int maxX = std::min(width - 1, static_cast<int>(std::ceil(std::max(x0, x1) + reach)));
        for (int x = minX; x <= maxX; ++x) {
            const float t = std::clamp((static_cast<float>(x) + 0.5f - x0) / dx, 0.0f, 1.0f);
            const float center = y0 + dy * t;
            const int minY = std::max(rowBegin, static_cast<int>(std::floor(center - spread)));
            const int maxY = std::min(rowEnd - 1, static_cast<int>(std::ceil(center + spread)));
            for (int y = minY; y <= maxY; ++y) plot(x, y);
        }
    }
}

} // namespace

RasterTarget::RasterTarget(int width, int height)
    : color(width, height), depth(static_cast<std::size_t>(std::max(0, width)) * std::max(0, height), NoDepth) {}

float RasterTarget::depthAt(int x, int y) const {
    return depth[static_cast<std::size_t>(y) * width() + x];
}

void RasterTarget::clearDepth() {
    std::fill(depth.begin(), depth.end(), NoDepth);
}

Rasterizer::Rasterizer(RasterTarget& target, const Projector& projector)
    : m_target(target), m_projector(projector), m_rowEnd(target.height()) {}

void Rasterizer::restrictRows(int begin, int end) {
    m_rowBegin = std::clamp(begin, 0, m_target.height());
    m_rowEnd = std::clamp(end, m_rowBegin, m_target.height());
}

void Rasterizer::triangle(Vec3 a, Vec3 b, Vec3 c, Color color, bool depthWrite) {
    rasterClipped(a, b, c, Flat, color, color, color, 0.0f, nullptr, depthWrite);
}

void Rasterizer::triangleGradient(Vec3 a, Vec3 b, Vec3 c, Color colorA, Color colorB, Color colorC) {
    rasterClipped(a, b, c, Gradient, colorA, colorB, colorC, 0.0f, nullptr, true);
}

void Rasterizer::triangleTextured(Vec3 a, Vec3 b, Vec3 c, Uv uvA, Uv uvB, Uv uvC, const Image& texture, float shade) {
    // As coordenadas vao nas cores (o recorte no plano proximo as interpola
    // como interpola as cores).
    rasterClipped(a, b, c, Textured, { uvA.u, uvA.v, 0.0f, 1.0f }, { uvB.u, uvB.v, 0.0f, 1.0f }, { uvC.u, uvC.v, 0.0f, 1.0f },
        shade, nullptr, true, 0.0f, &texture);
}

void Rasterizer::triangleLit(Vec3 a, Vec3 b, Vec3 c, const std::array<Vec3, 3>& normals, Color color, const PixelLight& light) {
    rasterClipped(a, b, c, Lit, color, color, color, 0.0f, nullptr, true, 0.0f, nullptr, &normals, &light);
}

void Rasterizer::triangleTexturedLit(Vec3 a, Vec3 b, Vec3 c, Uv uvA, Uv uvB, Uv uvC, const std::array<Vec3, 3>& normals,
    const Image& texture, const PixelLight& light) {
    rasterClipped(a, b, c, TexturedLit, { uvA.u, uvA.v, 0.0f, 1.0f }, { uvB.u, uvB.v, 0.0f, 1.0f }, { uvC.u, uvC.v, 0.0f, 1.0f },
        0.0f, nullptr, true, 0.0f, &texture, &normals, &light);
}

void Rasterizer::depthOnly(Vec3 a, Vec3 b, Vec3 c) {
    rasterClipped(a, b, c, DepthOnly, {}, {}, {}, 0.0f, nullptr, true);
}

void Rasterizer::overlayTriangle(Vec3 a, Vec3 b, Vec3 c, Color color, float biasMeters) {
    rasterClipped(a, b, c, Overlay, color, color, color, 0.0f, nullptr, false, biasMeters);
}

void Rasterizer::accumulate(Vec3 a, Vec3 b, Vec3 c, float amount, std::vector<float>& buffer) {
    rasterClipped(a, b, c, Accumulate, {}, {}, {}, amount, &buffer, false);
}

void Rasterizer::rasterClipped(Vec3 a, Vec3 b, Vec3 c, int kind, Color color, Color colorB, Color colorC,
    float amount, std::vector<float>* buffer, bool depthWrite, float biasMeters, const Image* texture,
    const std::array<Vec3, 3>* normals, const PixelLight* light) {
    const std::array<Vec3, 3> none {};
    const std::array<Vec3, 3>& n = normals != nullptr ? *normals : none;
    const std::array<ClipVertex, 3> input { { { m_projector.toCamera(a), color, n[0] },
        { m_projector.toCamera(b), colorB, n[1] }, { m_projector.toCamera(c), colorC, n[2] } } };
    std::array<ClipVertex, 4> clipped {};
    const int count = clipNear(input, m_projector.nearPlane(), clipped);
    if (count < 3) return;
    std::array<ScreenPoint, 4> screen {};
    for (int i = 0; i < count; ++i)
        screen[static_cast<std::size_t>(i)] = m_projector.projectCamera(clipped[static_cast<std::size_t>(i)].camera);

    const int width = m_target.width();
    Pixel* pixels = m_target.color.pixels().data();
    float* depth = m_target.depth.data();
    const bool opaque = color.a >= 0.999f;
    const Pixel flat = packed(color);
    const bool perspective = !m_projector.orthographic();

    for (int fan = 1; fan + 1 < count; ++fan) {
        const ScreenPoint& p0 = screen[0];
        const ScreenPoint& p1 = screen[static_cast<std::size_t>(fan)];
        const ScreenPoint& p2 = screen[static_cast<std::size_t>(fan + 1)];
        const Color& c0 = clipped[0].color;
        const Color& c1 = clipped[static_cast<std::size_t>(fan)].color;
        const Color& c2 = clipped[static_cast<std::size_t>(fan + 1)].color;
        const Vec3& n0 = clipped[0].normal;
        const Vec3& n1 = clipped[static_cast<std::size_t>(fan)].normal;
        const Vec3& n2 = clipped[static_cast<std::size_t>(fan + 1)].normal;
        rasterTriangle(width, m_rowBegin, m_rowEnd, p0, p1, p2, [&](int x, int y, float w0, float w1, float w2) {
            const std::size_t index = static_cast<std::size_t>(y) * width + x;
            if (kind == Accumulate) {
                (*buffer)[index] += amount;
                return;
            }
            const float key = w0 * p0.depth + w1 * p1.depth + w2 * p2.depth;
            if (kind == Overlay) {
                if (m_projector.towardCamera(key, biasMeters) >= depth[index]) m_target.color.blendPixel(x, y, color);
                return;
            }
            if (key <= depth[index]) return;
            if (depthWrite) depth[index] = key;
            if (kind == DepthOnly) return;
            if (kind == Flat) {
                if (opaque) pixels[index] = flat;
                else m_target.color.blendPixel(x, y, color);
                return;
            }
            // Gradiente: pesos corrigidos pela perspectiva (a chave e 1/z).
            float u0 = w0, u1 = w1, u2 = w2;
            if (perspective) {
                u0 *= p0.depth;
                u1 *= p1.depth;
                u2 *= p2.depth;
                const float sum = u0 + u1 + u2;
                if (sum > 0.0f) {
                    u0 /= sum;
                    u1 /= sum;
                    u2 /= sum;
                }
            }
            if (kind == Textured) {
                const Color texel = texture->sample({ c0.r * u0 + c1.r * u1 + c2.r * u2, c0.g * u0 + c1.g * u1 + c2.g * u2 });
                pixels[index] = packed({ texel.r * amount, texel.g * amount, texel.b * amount, 1.0f });
                return;
            }
            if (kind == Lit || kind == TexturedLit) {
                const float k = light->intensity(safeNormalized(n0 * u0 + n1 * u1 + n2 * u2, n0));
                const Color base = kind == TexturedLit
                    ? texture->sample({ c0.r * u0 + c1.r * u1 + c2.r * u2, c0.g * u0 + c1.g * u1 + c2.g * u2 }) : color;
                pixels[index] = packed({ base.r * k, base.g * k, base.b * k, 1.0f });
                return;
            }
            const Color shaded { c0.r * u0 + c1.r * u1 + c2.r * u2, c0.g * u0 + c1.g * u1 + c2.g * u2,
                c0.b * u0 + c1.b * u1 + c2.b * u2, 1.0f };
            pixels[index] = packed(shaded);
        });
    }
}

void Rasterizer::line(Vec3 a, Vec3 b, float thickness, Color color, DepthMode mode, float ghostAlpha,
    float biasMeters) {
    Vec3 ca = m_projector.toCamera(a), cb = m_projector.toCamera(b);
    const float near = m_projector.nearPlane();
    if (ca.z < near && cb.z < near) return;
    if (ca.z < near) ca = lerp(ca, cb, (near - ca.z) / (cb.z - ca.z));
    else if (cb.z < near) cb = lerp(cb, ca, (near - cb.z) / (ca.z - cb.z));
    const ScreenPoint p0 = m_projector.projectCamera(ca), p1 = m_projector.projectCamera(cb);
    const float halfWidth = std::max(0.5f, thickness * 0.5f);
    forEachLinePixel(p0.x, p0.y, p1.x, p1.y, halfWidth, m_target.width(), m_rowBegin, m_rowEnd, [&](int x, int y) {
        float t = 0.0f;
        const float distanceToLine = segmentDistance2D(static_cast<float>(x) + 0.5f, static_cast<float>(y) + 0.5f, p0.x,
            p0.y, p1.x, p1.y, t);
        float coverage = std::clamp(halfWidth - distanceToLine + 0.5f, 0.0f, 1.0f);
        if (coverage <= 0.0f) return;
        if (mode != DepthMode::Ignore) {
            const float key = p0.depth + (p1.depth - p0.depth) * t;
            const bool visible = m_projector.towardCamera(key, biasMeters) >= m_target.depthAt(x, y);
            if (!visible) {
                if (mode == DepthMode::Test) return;
                coverage *= ghostAlpha;
            }
        }
        m_target.color.blendPixel(x, y, color, coverage);
    });
}

void Rasterizer::disc(Vec3 center, float radius, Color color, DepthMode mode, float ghostAlpha, float biasMeters) {
    const Vec3 camera = m_projector.toCamera(center);
    if (camera.z < m_projector.nearPlane()) return;
    const ScreenPoint p = m_projector.projectCamera(camera);
    const int width = m_target.width();
    const int minX = std::max(0, static_cast<int>(std::floor(p.x - radius - 1.0f)));
    const int maxX = std::min(width - 1, static_cast<int>(std::ceil(p.x + radius + 1.0f)));
    const int minY = std::max(m_rowBegin, static_cast<int>(std::floor(p.y - radius - 1.0f)));
    const int maxY = std::min(m_rowEnd - 1, static_cast<int>(std::ceil(p.y + radius + 1.0f)));
    const float biased = m_projector.towardCamera(p.depth, biasMeters);
    for (int y = minY; y <= maxY; ++y) {
        for (int x = minX; x <= maxX; ++x) {
            const float dx = static_cast<float>(x) + 0.5f - p.x, dy = static_cast<float>(y) + 0.5f - p.y;
            float coverage = std::clamp(radius - std::sqrt(dx * dx + dy * dy) + 0.5f, 0.0f, 1.0f);
            if (coverage <= 0.0f) continue;
            if (mode != DepthMode::Ignore && biased < m_target.depthAt(x, y)) {
                if (mode == DepthMode::Test) continue;
                coverage *= ghostAlpha;
            }
            m_target.color.blendPixel(x, y, color, coverage);
        }
    }
}

void drawLine2D(Image& image, float x0, float y0, float x1, float y1, float thickness, Color color) {
    const float halfWidth = std::max(0.5f, thickness * 0.5f);
    forEachLinePixel(x0, y0, x1, y1, halfWidth, image.width(), 0, image.height(), [&](int x, int y) {
        float t = 0.0f;
        const float d = segmentDistance2D(static_cast<float>(x) + 0.5f, static_cast<float>(y) + 0.5f, x0, y0, x1, y1, t);
        const float coverage = std::clamp(halfWidth - d + 0.5f, 0.0f, 1.0f);
        if (coverage > 0.0f) image.blendPixel(x, y, color, coverage);
    });
}

void fillDisc2D(Image& image, float x, float y, float radius, Color color) {
    const int minX = std::max(0, static_cast<int>(std::floor(x - radius - 1.0f)));
    const int maxX = std::min(image.width() - 1, static_cast<int>(std::ceil(x + radius + 1.0f)));
    const int minY = std::max(0, static_cast<int>(std::floor(y - radius - 1.0f)));
    const int maxY = std::min(image.height() - 1, static_cast<int>(std::ceil(y + radius + 1.0f)));
    for (int py = minY; py <= maxY; ++py) {
        for (int px = minX; px <= maxX; ++px) {
            const float dx = static_cast<float>(px) + 0.5f - x, dy = static_cast<float>(py) + 0.5f - y;
            const float coverage = std::clamp(radius - std::sqrt(dx * dx + dy * dy) + 0.5f, 0.0f, 1.0f);
            if (coverage > 0.0f) image.blendPixel(px, py, color, coverage);
        }
    }
}

void strokeRect2D(Image& image, float x0, float y0, float x1, float y1, float thickness, Color color) {
    drawLine2D(image, x0, y0, x1, y0, thickness, color);
    drawLine2D(image, x1, y0, x1, y1, thickness, color);
    drawLine2D(image, x1, y1, x0, y1, thickness, color);
    drawLine2D(image, x0, y1, x0, y0, thickness, color);
}

} // namespace MatterEngine::Modeling
