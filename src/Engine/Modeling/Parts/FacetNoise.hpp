#pragma once

#include <cstdint>

namespace MatterEngine::Modeling {

// Pseudoaleatorio estavel em [-1, 1]: o facetado low-poly das pecas nao muda
// entre geracoes (mesma semente, mesmo indice, mesmo valor).
[[nodiscard]] inline float facetNoise(unsigned seed, int a, int b, int channel) {
    std::uint32_t h = seed * 0x9E3779B1u;
    h ^= static_cast<std::uint32_t>(a) * 0x85EBCA77u;
    h ^= static_cast<std::uint32_t>(b) * 0xC2B2AE3Du;
    h ^= static_cast<std::uint32_t>(channel) * 0x27D4EB2Fu;
    h ^= h >> 15;
    h *= 0x2C1B3C6Du;
    h ^= h >> 12;
    h *= 0x297A2D39u;
    h ^= h >> 15;
    return static_cast<float>(h & 0xFFFFFFu) / static_cast<float>(0xFFFFFFu) * 2.0f - 1.0f;
}

} // namespace MatterEngine::Modeling
