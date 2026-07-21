#include "Engine/Math/JitterSequence.hpp"

namespace MatterEngine {

float haltonSequence(std::uint32_t index, std::uint32_t base) {
    float result = 0.0f;
    float fraction = 1.0f / static_cast<float>(base);
    while (index > 0) {
        result += fraction * static_cast<float>(index % base);
        index /= base;
        fraction /= static_cast<float>(base);
    }
    return result;
}

Vec2 haltonJitter(std::uint32_t index) {
    const std::uint32_t shifted = index + 1;
    return { haltonSequence(shifted, 2), haltonSequence(shifted, 3) };
}

} // namespace MatterEngine
