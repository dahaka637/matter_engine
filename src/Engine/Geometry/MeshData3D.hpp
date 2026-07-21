#pragma once

#include "Engine/Math/Vec2.hpp"
#include "Engine/Math/Vec3.hpp"

#include <cstdint>
#include <vector>

namespace MatterEngine {

// Formato canônico de vértice para meshes estáticas importadas. A estrutura
// não conhece Vulkan nem o formato de origem; o importador preenche os dados e
// o Renderer os envia à RHI por buffers backend-neutral.
struct MeshVertex3D {
    Vec3 position;
    Vec3 normal;
    Vec2 uv;
    Vec3 color { 1.0f, 1.0f, 1.0f };
};

struct MeshData3D {
    std::vector<MeshVertex3D> vertices;
    std::vector<std::uint32_t> indices;
    Vec3 boundsMin;
    Vec3 boundsMax;
};

void recomputeBounds(MeshData3D& mesh);

} // namespace MatterEngine
