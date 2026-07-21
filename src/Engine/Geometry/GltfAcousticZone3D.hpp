#pragma once

#include "Engine/Audio/AcousticZone3D.hpp"
#include "Engine/Geometry/GltfLoader.hpp"

#include <vector>

namespace MatterEngine {

// Le entidades com entity_type == "acoustic_zone" (mesmo pipeline de extras
// ja usado por parseGltfPhysicsMetadata/spawnpoint); outros tipos sao
// ignorados silenciosamente. Uma zona malformada lanca std::invalid_argument,
// no mesmo estilo de parseGltfPhysicsMetadata.
[[nodiscard]] std::vector<AcousticZoneDefinition3D> parseAcousticZones(
    const std::vector<LoadedGltfEntity>& entities);

} // namespace MatterEngine
