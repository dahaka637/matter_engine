#include "Engine/Math/Hash.hpp"

namespace MatterEngine {

std::uint32_t pcgHash(std::uint32_t value) {
    value = value * 747796405u + 2891336453u;
    const std::uint32_t word =
        ((value >> ((value >> 28u) + 4u)) ^ value) * 277803737u;
    return (word >> 22u) ^ word;
}

float hashToUnitFloat(std::uint32_t seed) {
    return static_cast<float>(pcgHash(seed)) * (1.0f / 4294967295.0f);
}

} // namespace MatterEngine
