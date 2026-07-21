#include "Engine/Geometry/MeshData3D.hpp"

#include <algorithm>
#include <limits>

namespace MatterEngine {

void recomputeBounds(MeshData3D& mesh) {
    if (mesh.vertices.empty()) {
        mesh.boundsMin = {};
        mesh.boundsMax = {};
        return;
    }

    Vec3 minimum {
        std::numeric_limits<float>::max(),
        std::numeric_limits<float>::max(),
        std::numeric_limits<float>::max()
    };
    Vec3 maximum {
        std::numeric_limits<float>::lowest(),
        std::numeric_limits<float>::lowest(),
        std::numeric_limits<float>::lowest()
    };
    for (const MeshVertex3D& vertex : mesh.vertices) {
        minimum = {
            std::min(minimum.x, vertex.position.x),
            std::min(minimum.y, vertex.position.y),
            std::min(minimum.z, vertex.position.z)
        };
        maximum = {
            std::max(maximum.x, vertex.position.x),
            std::max(maximum.y, vertex.position.y),
            std::max(maximum.z, vertex.position.z)
        };
    }
    mesh.boundsMin = minimum;
    mesh.boundsMax = maximum;
}

} // namespace MatterEngine
