#include "Engine/Geometry/GltfPhysicsMetadata3D.hpp"

#include <charconv>
#include <algorithm>
#include <cmath>
#include <cctype>
#include <stdexcept>
#include <string_view>

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
        throw std::invalid_argument("Invalid glTF physics property: "
            + std::string(key));
    }
    return value;
}

std::optional<std::uint32_t> parseOptionalHullBudget(
    const GltfExtras& extras) {
    const std::string* text = extras.find("collision_hulls");
    if (text == nullptr) return std::nullopt;
    std::uint32_t value = 0;
    const char* begin = text->data();
    const char* end = begin + text->size();
    const auto parsed = std::from_chars(begin, end, value);
    if (parsed.ec != std::errc {} || parsed.ptr != end
        || value == 0 || value > 32) {
        throw std::invalid_argument(
            "collision_hulls deve estar entre 1 e 32");
    }
    return value;
}

ImportedCollisionMode3D parseCollisionMode(const GltfExtras& extras) {
    const std::string* value = extras.find("collision_mode");
    if (value == nullptr || *value == "auto") {
        return ImportedCollisionMode3D::Automatic;
    }
    if (*value == "box") return ImportedCollisionMode3D::Box;
    if (*value == "sphere") return ImportedCollisionMode3D::Sphere;
    if (*value == "capsule") return ImportedCollisionMode3D::Capsule;
    if (*value == "manual_compound") {
        return ImportedCollisionMode3D::ManualCompound;
    }
    if (*value == "static_mesh") {
        return ImportedCollisionMode3D::StaticTriangleMesh;
    }
    throw std::invalid_argument("Unknown collision_mode: " + *value);
}

bool parseGenerateCollision(const GltfExtras& extras) {
    const std::string* value = extras.find("generate_collision");
    if (value == nullptr) {
        // Alias legivel aceito para arquivos ja autorados dessa forma. O
        // snake_case acima permanece canonico e e o recomendado no Blender.
        value = extras.find("Generate Collision");
    }
    if (value == nullptr) return true;
    std::string normalized = *value;
    std::transform(normalized.begin(), normalized.end(), normalized.begin(),
        [](unsigned char character) {
            return static_cast<char>(std::tolower(character));
        });
    if (normalized == "true" || normalized == "yes"
        || normalized == "on" || normalized == "1") {
        return true;
    }
    if (normalized == "false" || normalized == "no"
        || normalized == "off" || normalized == "0") {
        return false;
    }
    throw std::invalid_argument(
        "Invalid glTF physics property: generate_collision");
}

AcousticBodyStructure3D parseAcousticStructure(std::string_view value) {
    if (value == "solid") return AcousticBodyStructure3D::Solid;
    if (value == "hollow") return AcousticBodyStructure3D::Hollow;
    if (value == "thin_shell") return AcousticBodyStructure3D::ThinShell;
    if (value == "soft") return AcousticBodyStructure3D::Soft;
    if (value == "inflated") return AcousticBodyStructure3D::Inflated;
    throw std::invalid_argument("Unknown acoustic_structure: "
        + std::string(value));
}

} // namespace

GltfPhysicsMetadata3D parseGltfPhysicsMetadata(const GltfExtras& extras) {
    GltfPhysicsMetadata3D result;
    if (const std::string* canonical = extras.find("physical_material")) {
        if (canonical->empty()) {
            throw std::invalid_argument("physical_material cannot be empty");
        }
        result.materialId = *canonical;
    } else if (const std::string* legacy = extras.find("material_type")) {
        if (!legacy->empty()) {
            // O primeiro mapa usava "metal" antes de o catalogo distinguir
            // ligas. Somente a chave legada recebe esta migracao; a chave
            // canonica exige o ID fisico explicito "steel".
            result.materialId = *legacy == "metal" ? "steel" : *legacy;
        }
    }

    // `mass` e o contrato atual e suficiente por si so. `mass_mode` com
    // `mass_kg` continua aceito para abrir assets produzidos anteriormente.
    if (extras.find("mass") != nullptr) {
        result.bodyBinding.massMode = BodyMassMode3D::OverrideKilograms;
        result.bodyBinding.massOverrideKg = parsePositiveFloat(
            extras, "mass", 0.0f);
    } else if (const std::string* massMode = extras.find("mass_mode")) {
        if (*massMode == "automatic") {
            result.bodyBinding.massMode =
                BodyMassMode3D::AutomaticFromDensity;
        } else if (*massMode == "override") {
            result.bodyBinding.massMode = BodyMassMode3D::OverrideKilograms;
            result.bodyBinding.massOverrideKg = parsePositiveFloat(
                extras, "mass_kg", 0.0f);
        } else {
            throw std::invalid_argument("Unknown mass_mode: " + *massMode);
        }
    }
    result.generateCollision = parseGenerateCollision(extras);
    result.collisionMode = parseCollisionMode(extras);
    result.maximumCollisionHulls = parseOptionalHullBudget(extras);
    if (const std::string* structure = extras.find("acoustic_structure")) {
        if (*structure != "automatic") {
            result.bodyBinding.acousticStructureOverride =
                parseAcousticStructure(*structure);
        }
    }
    if (extras.find("drag_coefficient") != nullptr) {
        result.bodyBinding.dragCoefficientOverride = parsePositiveFloat(
            extras, "drag_coefficient", 0.0f, true);
    }
    if (extras.find("drag_area_m2") != nullptr) {
        result.bodyBinding.referenceAreaOverrideSquareMeters =
            parsePositiveFloat(extras, "drag_area_m2", 0.0f, true);
    }
    result.bodyBinding.acousticGain = parsePositiveFloat(
        extras, "acoustic_gain", 1.0f, true);
    result.bodyBinding.acousticDampingScale = parsePositiveFloat(
        extras, "acoustic_damping", 1.0f, true);
    return result;
}

} // namespace MatterEngine
