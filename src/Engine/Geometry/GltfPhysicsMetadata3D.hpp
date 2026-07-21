#pragma once

#include "Engine/Geometry/GltfLoader.hpp"
#include "Engine/Physics/PhysicalBodyBuilder3D.hpp"

#include <cstdint>
#include <optional>
#include <string>

namespace MatterEngine {

enum class ImportedCollisionMode3D {
    Automatic,
    Box,
    Sphere,
    Capsule,
    ManualCompound,
    StaticTriangleMesh
};

// Contrato unico entre Blender/glTF e a construcao de corpos. Todas as chaves
// sao escalares porque GltfExtras preserva esse subconjunto sem um parser JSON
// especifico da aplicacao.
struct GltfPhysicsMetadata3D {
    std::string materialId = "default";
    BodyMaterialBinding3D bodyBinding;
    // Ausente/true: a malha visual gera o collider. False: somente as
    // geometrias de colisao explicitamente autoradas no Blender sao usadas.
    bool generateCollision = true;
    ImportedCollisionMode3D collisionMode =
        ImportedCollisionMode3D::Automatic;
    // Override autorado do budget V-HACD. Ausente usa o padrao da engine;
    // primitivas analiticas ignoram este campo.
    std::optional<std::uint32_t> maximumCollisionHulls;
};

// Le `physical_material` como chave canonica. `material_type` permanece como
// alias temporario para os assets antigos do laboratorio.
[[nodiscard]] GltfPhysicsMetadata3D parseGltfPhysicsMetadata(
    const GltfExtras& extras);

} // namespace MatterEngine
