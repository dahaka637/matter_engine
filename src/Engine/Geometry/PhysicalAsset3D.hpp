#pragma once

#include "Engine/Geometry/GltfLoader.hpp"
#include "Engine/Geometry/GltfPhysicsMetadata3D.hpp"
#include "Engine/Materials/MaterialLibrary.hpp"
#include "Engine/Physics/PhysicsEngine3D.hpp"

#include <string>

namespace MatterEngine {

// Asset glTF pronto para ser instanciado como um corpo rígido. O importador
// centraliza o contrato Blender -> engine: a aplicação não precisa recalcular
// bounds, interpretar extras ou duplicar regras de massa e colisão.
struct PhysicalAsset3D {
    LoadedGltfModel model;
    GltfPhysicsMetadata3D metadata;
    CookedDynamicCollision3D collision;
    PhysicsBodyDefinition3D bodyTemplate;
    Vec3 dimensionsMeters;
    float sourceVolumeCubicMeters = 0.0f;
};

// Importa e valida um prop dinâmico. As posições dos vértices são recentradas
// no centro físico do asset; assim o mesmo transform posiciona render e
// collider durante toda a vida da instância.
[[nodiscard]] PhysicalAsset3D loadPhysicalAsset3D(
    const std::string& path, const MaterialLibrary& materials,
    PhysicsEngine3D& physics);

} // namespace MatterEngine
