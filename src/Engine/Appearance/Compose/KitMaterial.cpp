#include "Engine/Appearance/Compose/KitMaterial.hpp"

#include <algorithm>
#include <cmath>

namespace MatterEngine::Appearance {

float kitLevelOfDetail(std::array<float, 2> dx, std::array<float, 2> dy, int width, int height) {
    const float w = static_cast<float>(width), h = static_cast<float>(height);
    const float x = std::hypot(dx[0] * w, dx[1] * h), y = std::hypot(dy[0] * w, dy[1] * h);
    return std::log2(std::max(1e-9f, std::max(x, y)));
}

Rgba kitSample(const KitTexture& texture, float u, float v) {
    const float x = u * static_cast<float>(texture.width) - 0.5f, y = v * static_cast<float>(texture.height) - 0.5f;
    const int x0 = static_cast<int>(std::floor(x)), y0 = static_cast<int>(std::floor(y));
    const float fx = x - static_cast<float>(x0), fy = y - static_cast<float>(y0);
    Rgba out {};
    for (int j = 0; j < 2; ++j)
        for (int i = 0; i < 2; ++i) {
            const int cx = std::clamp(x0 + i, 0, texture.width - 1), cy = std::clamp(y0 + j, 0, texture.height - 1);
            const float w = (i != 0 ? fx : 1.0f - fx) * (j != 0 ? fy : 1.0f - fy);
            const std::uint8_t* p = &texture.rgba[(static_cast<std::size_t>(cy) * static_cast<std::size_t>(texture.width)
                                                      + static_cast<std::size_t>(cx)) * 4];
            for (std::size_t k = 0; k < 4; ++k) out[k] += w * static_cast<float>(p[k]) / 255.0f;
        }
    return out;
}

Rgba kitTrilinear(const std::vector<KitTexture>& levels, int maximumLevel, float lod, float u, float v) {
    const float l = std::clamp(lod, 0.0f, static_cast<float>(maximumLevel));
    const int l0 = static_cast<int>(std::floor(l)), l1 = std::min(maximumLevel, l0 + 1);
    const float f = l - static_cast<float>(l0);
    const Rgba a = kitSample(levels[static_cast<std::size_t>(l0)], u, v);
    if (f <= 0.0f || l1 == l0) return a;
    const Rgba b = kitSample(levels[static_cast<std::size_t>(l1)], u, v);
    Rgba out {};
    for (std::size_t k = 0; k < 4; ++k) out[k] = a[k] + (b[k] - a[k]) * f;
    return out;
}

Rgba kitMaterialColor(const ComposedKit& kit, const SurfaceTemplate& surface, std::size_t t, std::array<float, 3> b, float kitLod,
    float identityLod) {
    if (t >= surface.trianglePanel.size() || surface.trianglePanel[t] < 0 || kit.kit.empty()) return { 0.0f, 0.0f, 0.0f, 0.0f };
    const std::array<Vec2, 3>& uv = surface.triangleKitUv[t];
    Rgba color = kitTrilinear(kit.kit, kit.kitSafeLevel, kitLod, b[0] * uv[0].x + b[1] * uv[1].x + b[2] * uv[2].x,
        b[0] * uv[0].y + b[1] * uv[1].y + b[2] * uv[2].y);
    const Rgba& average = kit.regionAverage[surface.triangleRegion[t]];
    const float toAverage = std::clamp(kitLod - static_cast<float>(kit.kitSafeLevel), 0.0f, 1.0f);
    for (std::size_t k = 0; k < 4; ++k) color[k] += (average[k] - color[k]) * toAverage;
    const int zone = surface.triangleIdentity[t];
    if (zone >= 0 && !kit.identity.empty()) {
        const std::array<Vec2, 3>& id = surface.triangleIdentityUv[t];
        const float u = b[0] * id[0].x + b[1] * id[1].x + b[2] * id[2].x, v = b[0] * id[0].y + b[1] * id[1].y + b[2] * id[2].y;
        const SurfaceZone& rect = surface.zones[static_cast<std::size_t>(zone)];
        const float width = static_cast<float>(surface.identityLayout.width), height = static_cast<float>(surface.identityLayout.height);
        // (O retangulo interno do bloco no UV1: o mesmo limite do shader.)
        if (u >= static_cast<float>(rect.block[0] + rect.inset) / width && u <= static_cast<float>(rect.block[0] + rect.block[2] - rect.inset) / width
            && v >= static_cast<float>(rect.block[1] + rect.inset) / height
            && v <= static_cast<float>(rect.block[1] + rect.block[3] - rect.inset) / height) {
            Rgba ink = kitTrilinear(kit.identity, kit.identitySafeLevel, identityLod, u, v);
            const float fade = 1.0f - std::clamp(identityLod - static_cast<float>(kit.identitySafeLevel), 0.0f, 1.0f);
            for (float& channel : ink) channel *= fade;
            for (std::size_t k = 0; k < 4; ++k) color[k] = ink[k] + color[k] * (1.0f - ink[3]);
        }
    }
    return color;
}

} // namespace MatterEngine::Appearance
