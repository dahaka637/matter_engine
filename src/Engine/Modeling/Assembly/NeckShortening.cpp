#include "Engine/Modeling/Assembly/NeckShortening.hpp"

#include <algorithm>
#include <cmath>

namespace MatterEngine::Modeling {

float NeckShortening::top(float x) const {
    const float span = topFront[0] - topBack[0];
    const float slope = std::abs(span) > 1e-6f ? (topFront[1] - topBack[1]) / span : 0.0f;
    return topFront[1] + (x - topFront[0]) * slope;
}

float NeckShortening::lift(float x, float height) const {
    if (amount == 0.0f || from <= 0.0f) return 0.0f;
    if (height <= from) return amount * std::max(0.0f, height) / from;
    const float upper = top(x);
    if (height >= upper || upper <= from) return 0.0f;
    return amount * (upper - height) / (upper - from);
}

Vec3 NeckShortening::apply(Vec3 point, float ground) const {
    return { point.x, point.y, point.z + lift(point.x, point.z - ground) };
}

} // namespace MatterEngine::Modeling
