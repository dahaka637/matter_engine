#include "Engine/Modeling/Assembly/NeckBand.hpp"

#include <algorithm>
#include <cmath>

namespace MatterEngine::Modeling {

float NeckBand::base(Vec3 point) const {
    return std::max(lines.collarHeight(point), lines.ground + lines.neck.liftedHeight(point.x, frontBase));
}

float NeckBand::headLine(Vec3 point) const {
    return lines.ground + lines.neck.top(point.x) + headAbove;
}

float NeckBand::along(Vec3 point) const {
    const float bottom = base(point), top = headLine(point);
    if (top - bottom < 1e-3f) return -1.0f;
    return (point.z - bottom) / (top - bottom);
}

std::optional<SkinWeights> neckBandWeights(const NeckBand& band, Vec3 point, int chest, int neck, int head) {
    if (chest < 0 || neck < 0 || head < 0 || std::abs(point.y) >= band.width) return std::nullopt;
    const float u = band.along(point);
    if (!(u > 0.0f && u < 1.0f)) return std::nullopt;
    const float middle = std::clamp(band.middle, 0.05f, 0.95f);
    SkinWeights weights {};
    if (u < middle) {
        const float t = u / middle;
        weights.joints = { static_cast<std::uint16_t>(chest), static_cast<std::uint16_t>(neck), 0, 0 };
        weights.weights = { 1.0f - t, t, 0.0f, 0.0f };
    } else {
        const float t = (u - middle) / (1.0f - middle);
        weights.joints = { static_cast<std::uint16_t>(neck), static_cast<std::uint16_t>(head), 0, 0 };
        weights.weights = { 1.0f - t, t, 0.0f, 0.0f };
    }
    return weights;
}

} // namespace MatterEngine::Modeling
