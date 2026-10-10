#include "Engine/Athlete/Planning/MotionPlanner3D.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>

namespace MatterEngine {

float athleteTravelSpeed3D(const std::array<float, 5>& trot, const std::array<float, 5>& sprint,
    float travelRelativeToLookRadians, bool sprinting) {
    const std::array<float, 5>& table = sprinting ? sprint : trot;
    const float position = std::abs(std::remainder(travelRelativeToLookRadians, 6.28318531f)) / 0.785398163f;
    const std::size_t k = std::min<std::size_t>(3, static_cast<std::size_t>(position));
    const float t = std::clamp(position - static_cast<float>(k), 0.0f, 1.0f);
    return table[k] + (table[k + 1] - table[k]) * t;
}

AthleteCapsuleResponse3D athleteCapsuleResponse3D(float responsePercent) {
    const float scale = std::clamp(responsePercent, 10.0f, 300.0f) * 0.01f;
    AthleteCapsuleResponse3D response;
    response.acceleration *= scale;
    response.deceleration *= scale;
    response.brake *= scale;
    response.turnRate *= scale;
    return response;
}

} // namespace MatterEngine
