#pragma once

#include "Engine/Geometry/GltfLoader.hpp"

namespace MatterEngine {

struct MapSpawn3D {
    Vec3 position;
    float yaw = 0.0f;
};

struct MapField3D {
    Vec3 center;
    float yaw = 0.0f;
    float length = 40.0f;
    float width = 20.0f;
};

struct AuthoredMap3D {
    std::vector<MapSpawn3D> spawns;
    std::vector<MapField3D> fields;
};

// Gameplay markers use the same metres/Z-up convention as the imported
// geometry.
[[nodiscard]] AuthoredMap3D parseAuthoredMap3D(const LoadedGltfModel &model);
[[nodiscard]] MeshData3D buildFutsalMarkings3D(const MapField3D &field);

} // namespace MatterEngine
