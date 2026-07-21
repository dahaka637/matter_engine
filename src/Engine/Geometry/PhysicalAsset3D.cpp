#include "Engine/Geometry/PhysicalAsset3D.hpp"

#include "Engine/Physics/PhysicalBodyBuilder3D.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <stdexcept>
#include <string_view>

namespace MatterEngine {
namespace {

constexpr float GeometryEpsilon = 1.0e-6f;
constexpr float Pi = 3.14159265358979323846f;
constexpr std::size_t MaximumAuthoredCollisionParts = 32;
constexpr std::size_t MaximumCompoundShapes = 64;

bool containsAuthoredPhysics(const GltfExtras& extras) {
    return extras.find("physical_material") != nullptr
        || extras.find("material_type") != nullptr
        || extras.find("mass_mode") != nullptr
        || extras.find("mass") != nullptr
        || extras.find("collision_mode") != nullptr
        || extras.find("collision_hulls") != nullptr
        || extras.find("generate_collision") != nullptr
        || extras.find("Generate Collision") != nullptr;
}

bool parseBoolean(std::string value, std::string_view propertyName) {
    std::transform(value.begin(), value.end(), value.begin(),
        [](unsigned char character) {
            return static_cast<char>(std::tolower(character));
        });
    if (value == "true" || value == "yes" || value == "on"
        || value == "1") return true;
    if (value == "false" || value == "no" || value == "off"
        || value == "0") return false;
    throw std::runtime_error(std::string(propertyName)
        + " deve ser true/false no glTF");
}

bool authoredCollisionPart(const LoadedGltfPart& part) {
    if (const std::string* flag = part.extras.find("collision_geometry")) {
        return parseBoolean(*flag, "collision_geometry");
    }
    std::string name = part.nodeName;
    std::transform(name.begin(), name.end(), name.begin(),
        [](unsigned char character) {
            return static_cast<char>(std::toupper(character));
        });
    constexpr std::string_view Prefixes[] {
        "COLLIDER_", "UCX_", "UBX_", "USP_", "UCP_"
    };
    return std::any_of(std::begin(Prefixes), std::end(Prefixes),
        [&](std::string_view prefix) { return name.starts_with(prefix); });
}

GltfPhysicsMetadata3D readConsistentMetadata(
    const LoadedGltfModel& model) {
    const LoadedGltfPart* canonicalPart = nullptr;
    for (const LoadedGltfPart& part : model.parts) {
        if (containsAuthoredPhysics(part.extras)) {
            canonicalPart = &part;
            break;
        }
    }
    if (canonicalPart == nullptr) {
        return model.parts.empty() ? GltfPhysicsMetadata3D {}
            : parseGltfPhysicsMetadata(model.parts.front().extras);
    }

    const GltfPhysicsMetadata3D canonical =
        parseGltfPhysicsMetadata(canonicalPart->extras);
    for (const LoadedGltfPart& part : model.parts) {
        if (!containsAuthoredPhysics(part.extras)) continue;
        const GltfPhysicsMetadata3D candidate =
            parseGltfPhysicsMetadata(part.extras);
        if (candidate.materialId != canonical.materialId
            || candidate.generateCollision != canonical.generateCollision
            || candidate.collisionMode != canonical.collisionMode
            || candidate.maximumCollisionHulls
                != canonical.maximumCollisionHulls
            || candidate.bodyBinding.massMode != canonical.bodyBinding.massMode
            || std::abs(candidate.bodyBinding.massOverrideKg
                - canonical.bodyBinding.massOverrideKg) > GeometryEpsilon) {
            throw std::runtime_error(
                "Partes do prop possuem metadados fisicos incompatíveis");
        }
    }
    return canonical;
}

MeshData3D mergeMeshes(const std::vector<LoadedGltfPart>& parts) {
    MeshData3D merged;
    for (const LoadedGltfPart& part : parts) {
        const std::uint32_t base =
            static_cast<std::uint32_t>(merged.vertices.size());
        merged.vertices.insert(merged.vertices.end(),
            part.mesh.vertices.begin(), part.mesh.vertices.end());
        for (const std::uint32_t index : part.mesh.indices) {
            merged.indices.push_back(base + index);
        }
    }
    recomputeBounds(merged);
    return merged;
}

void recenterModel(LoadedGltfModel& model, Vec3 center) {
    for (LoadedGltfPart& part : model.parts) {
        for (MeshVertex3D& vertex : part.mesh.vertices) {
            vertex.position -= center;
        }
        part.worldPosition -= center;
        recomputeBounds(part.mesh);
    }
}

CookedDynamicCollision3D primitiveCollision(const MeshData3D& mesh,
    ImportedCollisionMode3D mode) {
    CookedDynamicCollision3D result;
    result.localCenter = (mesh.boundsMin + mesh.boundsMax) * 0.5f;
    result.dimensionsMeters = mesh.boundsMax - mesh.boundsMin;
    const Vec3 halfExtents = result.dimensionsMeters * 0.5f;
    PhysicsShape3D shape;
    switch (mode) {
    case ImportedCollisionMode3D::Box:
        shape.type = PhysicsShapeType3D::Box;
        shape.halfExtents = halfExtents;
        result.volumeCubicMeters = result.dimensionsMeters.x
            * result.dimensionsMeters.y * result.dimensionsMeters.z;
        break;
    case ImportedCollisionMode3D::Sphere:
        shape.type = PhysicsShapeType3D::Sphere;
        shape.radius = std::max({ halfExtents.x, halfExtents.y,
            halfExtents.z });
        result.volumeCubicMeters = 4.0f / 3.0f * Pi
            * shape.radius * shape.radius * shape.radius;
        break;
    case ImportedCollisionMode3D::Capsule:
        shape.type = PhysicsShapeType3D::Capsule;
        shape.radius = std::max(halfExtents.x, halfExtents.z);
        shape.capsuleHalfHeight = std::max(0.0f,
            halfExtents.y - shape.radius);
        result.volumeCubicMeters = Pi * shape.radius * shape.radius
            * (shape.capsuleHalfHeight * 2.0f)
            + 4.0f / 3.0f * Pi
                * shape.radius * shape.radius * shape.radius;
        break;
    default:
        throw std::logic_error("Modo nao e uma primitiva dinamica");
    }
    result.shapes.push_back(std::move(shape));
    return result;
}

CookedDynamicCollision3D cookAuthoredCompound(
    const std::vector<LoadedGltfPart>& authoredParts,
    Vec3 commonCenter, Vec3 visualDimensions,
    PhysicsEngine3D& physics) {
    if (authoredParts.size() > MaximumAuthoredCollisionParts) {
        throw std::runtime_error(
            "Collider autorado excede o limite de 32 partes");
    }
    CookedDynamicCollision3D result;
    result.localCenter = commonCenter;
    result.dimensionsMeters = visualDimensions;
    ConvexDecompositionSettings3D partSettings;
    partSettings.maximumHulls = 8;
    partSettings.voxelResolution = 100000;
    for (const LoadedGltfPart& part : authoredParts) {
        CookedDynamicCollision3D cooked = physics.cookDynamicCollision(
            part.mesh, partSettings);
        const Vec3 partOffset = cooked.localCenter - commonCenter;
        for (PhysicsShape3D& shape : cooked.shapes) {
            shape.localPosition += partOffset;
            result.shapes.push_back(std::move(shape));
        }
        if (result.shapes.size() > MaximumCompoundShapes) {
            throw std::runtime_error(
                "Collider autorado excede o orcamento de 64 shapes convexas");
        }
        result.volumeCubicMeters += cooked.volumeCubicMeters;
        result.hullCount += cooked.hullCount;
    }
    return result;
}

} // namespace

PhysicalAsset3D loadPhysicalAsset3D(const std::string& path,
    const MaterialLibrary& materials, PhysicsEngine3D& physics) {
    PhysicalAsset3D result;
    result.model = loadGltfModel(path);
    if (result.model.parts.empty()) {
        throw std::runtime_error("O glTF nao contem malha triangular");
    }
    result.metadata = readConsistentMetadata(result.model);
    const SurfaceMaterial* material = materials.find(result.metadata.materialId);
    if (material == nullptr) {
        throw std::runtime_error("Material fisico nao registrado: "
            + result.metadata.materialId);
    }

    std::vector<LoadedGltfPart> authoredParts;
    std::vector<LoadedGltfPart> visualParts;
    authoredParts.reserve(result.model.parts.size());
    visualParts.reserve(result.model.parts.size());
    for (LoadedGltfPart& part : result.model.parts) {
        if (authoredCollisionPart(part)) {
            authoredParts.push_back(std::move(part));
        } else {
            visualParts.push_back(std::move(part));
        }
    }
    result.model.parts = std::move(visualParts);
    if (result.model.parts.empty()) {
        throw std::runtime_error("O glTF nao contem malha visual");
    }

    const MeshData3D visualMesh = mergeMeshes(result.model.parts);
    result.dimensionsMeters = visualMesh.boundsMax - visualMesh.boundsMin;
    if (result.dimensionsMeters.x <= GeometryEpsilon
        || result.dimensionsMeters.y <= GeometryEpsilon
        || result.dimensionsMeters.z <= GeometryEpsilon) {
        throw std::runtime_error("O prop possui dimensoes invalidas");
    }
    if (result.metadata.collisionMode
        == ImportedCollisionMode3D::StaticTriangleMesh) {
        throw std::runtime_error(
            "static_mesh nao pode ser instanciada como prop dinamico");
    }

    if (!result.metadata.generateCollision
        || result.metadata.collisionMode
            == ImportedCollisionMode3D::ManualCompound) {
        if (authoredParts.empty()) {
            throw std::runtime_error(
                "generate_collision=no exige objetos UCX_/COLLIDER_ no glTF");
        }
        const Vec3 center = (visualMesh.boundsMin + visualMesh.boundsMax) * 0.5f;
        result.collision = cookAuthoredCompound(authoredParts, center,
            result.dimensionsMeters, physics);
    } else if (result.metadata.collisionMode == ImportedCollisionMode3D::Box
        || result.metadata.collisionMode == ImportedCollisionMode3D::Sphere
        || result.metadata.collisionMode == ImportedCollisionMode3D::Capsule) {
        result.collision = primitiveCollision(visualMesh,
            result.metadata.collisionMode);
    } else {
        // O modo automatico usa a malha visual somente como fonte do cooking.
        // O runtime recebe exclusivamente um compound de hulls convexos.
        // O cache fica ao lado do asset para ser gerado pela ferramenta
        // offline e distribuido junto do GLB. O hash interno invalida o arquivo
        // automaticamente quando geometria ou parametros mudam.
        ConvexDecompositionSettings3D cookingSettings;
        if (result.metadata.maximumCollisionHulls) {
            cookingSettings.maximumHulls =
                *result.metadata.maximumCollisionHulls;
        }
        result.collision = physics.cookDynamicCollision(visualMesh,
            cookingSettings, path + ".mecollider");
    }
    if (result.collision.shapes.empty()) {
        throw std::runtime_error("Cooking nao produziu shapes dinamicas");
    }
    for (PhysicsShape3D& shape : result.collision.shapes) {
        shape.materialId = material->id;
    }

    result.sourceVolumeCubicMeters = result.collision.volumeCubicMeters;
    recenterModel(result.model, result.collision.localCenter);
    result.collision.localCenter = {};
    result.bodyTemplate = buildDynamicBodyDefinition(*material,
        result.metadata.bodyBinding, result.sourceVolumeCubicMeters,
        result.dimensionsMeters);
    return result;
}

} // namespace MatterEngine
