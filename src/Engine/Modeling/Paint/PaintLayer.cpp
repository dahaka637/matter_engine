#include "Engine/Modeling/Paint/PaintLayer.hpp"

#include <algorithm>
#include <cmath>

namespace MatterEngine::Modeling {

namespace {

Color alongStops(const std::vector<ColorStop>& stops, float t, Color fallback) {
    if (stops.empty()) return fallback;
    if (t <= stops.front().at) return stops.front().color;
    for (std::size_t i = 1; i < stops.size(); ++i)
        if (t <= stops[i].at) {
            const float span = std::max(1e-6f, stops[i].at - stops[i - 1].at);
            return mix(stops[i - 1].color, stops[i].color, (t - stops[i - 1].at) / span);
        }
    return stops.back().color;
}

Color blended(Color below, Color paint, Blend blend, float amount) {
    const Color target = blend == Blend::Multiply ? Color { below.r * paint.r, below.g * paint.g, below.b * paint.b, below.a } : paint;
    const float k = std::clamp(amount * paint.a, 0.0f, 1.0f);
    return { below.r + (target.r - below.r) * k, below.g + (target.g - below.g) * k, below.b + (target.b - below.b) * k, below.a };
}

} // namespace

Color Fill::at(Point2 point, float footprint) const {
    switch (kind) {
    case Kind::Solid:
        return color;
    case Kind::Linear: {
        const float dx = to.x - from.x, dy = to.y - from.y;
        const float t = ((point.x - from.x) * dx + (point.y - from.y) * dy) / std::max(1e-12f, dx * dx + dy * dy);
        return alongStops(stops, t, color);
    }
    case Kind::Radial: {
        const float c = std::cos(-angle), s = std::sin(-angle);
        const float x = point.x - center.x, y = point.y - center.y;
        const float u = (x * c - y * s) / std::max(1e-9f, radius.x), v = (x * s + y * c) / std::max(1e-9f, radius.y);
        return alongStops(stops, std::sqrt(u * u + v * v), color);
    }
    case Kind::Image: {
        if (!image) return { 0.0f, 0.0f, 0.0f, 0.0f };
        const float width = to.x - from.x, height = to.y - from.y;
        if (std::abs(width) < 1e-9f || std::abs(height) < 1e-9f) return { 0.0f, 0.0f, 0.0f, 0.0f };
        const float c = std::cos(-angle), s = std::sin(-angle);
        const float x = point.x - 0.5f * (from.x + to.x), y = point.y - 0.5f * (from.y + to.y);
        // (A linha de cima do arquivo no alto da caixa.)
        return image->sample((x * c - y * s) / width + 0.5f, 0.5f - (x * s + y * c) / height, footprint / std::abs(width));
    }
    }
    return color;
}

Color paintOver(Color below, const PaintLayer& layer, Point2 point, float footprint) {
    if (layer.shape.empty() || layer.opacity <= 0.0f) return below;
    Color out = below;
    if (layer.outline > 0.0f) {
        // O contorno: a forma engordada, por baixo da tinta.
        const float ring = layer.shape.coverage(point, footprint, layer.outline);
        if (ring > 0.0f) out = blended(out, layer.outlineColor, Blend::Normal, ring * layer.opacity);
    }
    const float cover = layer.shape.coverage(point, footprint);
    if (cover > 0.0f) out = blended(out, layer.fill.at(point, footprint), layer.blend, cover * layer.opacity);
    return out;
}

} // namespace MatterEngine::Modeling
