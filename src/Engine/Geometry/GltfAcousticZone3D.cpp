#include "Engine/Geometry/GltfAcousticZone3D.hpp"

#include <charconv>
#include <cmath>
#include <stdexcept>

namespace MatterEngine {
namespace {

float parsePositiveFloat(const GltfExtras& extras, std::string_view key,
    float fallback, bool allowZero = false) {
    const std::string* text = extras.find(key);
    if (text == nullptr) return fallback;
    float value = 0.0f;
    const char* begin = text->data();
    const char* end = begin + text->size();
    const auto parsed = std::from_chars(begin, end, value);
    if (parsed.ec != std::errc {} || parsed.ptr != end
        || !std::isfinite(value)
        || (allowZero ? value < 0.0f : value <= 0.0f)) {
        throw std::invalid_argument(
            "Zona acustica com valor invalido em: " + std::string(key));
    }
    return value;
}

AcousticReverbPreset3D parseReverbPreset(const GltfExtras& extras) {
    const std::string* value = extras.find("reverb_preset");
    if (value == nullptr || *value == "generic") {
        return AcousticReverbPreset3D::Generic;
    }
    if (*value == "small_room") return AcousticReverbPreset3D::SmallRoom;
    if (*value == "hallway") return AcousticReverbPreset3D::Hallway;
    if (*value == "cave") return AcousticReverbPreset3D::Cave;
    if (*value == "outdoor") return AcousticReverbPreset3D::Outdoor;
    if (*value == "warehouse") return AcousticReverbPreset3D::Warehouse;
    throw std::invalid_argument("reverb_preset desconhecido: " + *value);
}

} // namespace

std::vector<AcousticZoneDefinition3D> parseAcousticZones(
    const std::vector<LoadedGltfEntity>& entities) {
    std::vector<AcousticZoneDefinition3D> zones;
    for (const LoadedGltfEntity& entity : entities) {
        const std::string* type = entity.extras.find("entity_type");
        if (type == nullptr || *type != "acoustic_zone") continue;

        AcousticZoneDefinition3D zone;
        zone.name = entity.name;
        zone.center = entity.position;
        zone.radiusMeters = parsePositiveFloat(
            entity.extras, "radius", 8.0f);
        zone.blendDistanceMeters = parsePositiveFloat(
            entity.extras, "blend_distance", 3.0f, true);
        zone.preset = parseReverbPreset(entity.extras);
        zone.wetSendGain = parsePositiveFloat(
            entity.extras, "wet_send", 1.0f, true);
        zones.push_back(std::move(zone));
    }
    return zones;
}

} // namespace MatterEngine
