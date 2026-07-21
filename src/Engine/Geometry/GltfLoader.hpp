#pragma once

#include "Engine/Geometry/MeshData3D.hpp"
#include "Engine/Math/Quaternion.hpp"

#include <cstddef>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace MatterEngine {

// Decoded RGBA8 pixels, ready for Renderer::createTexture2D. Kept CPU-side
// here - GltfLoader never touches the RHI/Vulkan backend.
struct LoadedGltfImage {
    int width = 0;
    int height = 0;
    std::vector<std::byte> rgbaPixels;
};

// Visao plana do objeto extras de um node ou material glTF. Propriedades
// personalizadas do Blender chegam por este contrato.
struct GltfExtras {
    std::vector<std::pair<std::string, std::string>> values;
    std::string rawJson;

    [[nodiscard]] const std::string* find(std::string_view key) const;
};

struct LoadedGltfMaterial {
    std::string name;
    // Already the tint baked into every vertex of parts using this material
    // (see LoadedGltfPart) - re-exposed here mainly for inspection/UI.
    Vec3 baseColor { 1.0f, 1.0f, 1.0f };
    // Index into LoadedGltfModel::images, or -1 when the material has no
    // base color texture (e.g. a flat-color-only material authored in
    // Blender with no UV work at all).
    int imageIndex = -1;
    // Canal G = rugosidade e canal B = metalicidade, conforme glTF 2.0.
    // Mantido separado do albedo para o renderer aplicar ambos por pixel.
    int metallicRoughnessImageIndex = -1;
    // Straight from glTF's pbrMetallicRoughness (defaults match the spec's
    // own defaults: fully metallic, fully rough, when a material omits
    // them). Not consumed by the procedural box/sphere shader path, only by
    // MeshRender3D's analytic sky reflection - see appendLaboratoryMapRenderables.
    float metallicFactor = 1.0f;
    float roughnessFactor = 1.0f;
    GltfExtras extras;
};

struct LoadedGltfPart {
    MeshData3D mesh;
    int materialIndex = -1;
    // The Blender object (glTF node) this part came from - one node can
    // produce more than one part (one per material on the object), and all
    // of them share this same name and extras.
    std::string nodeName;
    GltfExtras extras;

    // The node's own world-space origin and rotation (Z-up, same convention
    // as everything else here), plus this part's bounds in the node's own
    // local frame (before that rotation/translation - local axes already
    // permuted the same way world positions are, and any node scale already
    // folded in, so a caller can rebuild a tight oriented box collider as
    // worldPosition + worldOrientation.rotate(localCenter) / halfExtents =
    // (localBoundsMax-localBoundsMin)*0.5 without re-deriving any of this).
    // hasOrientation follows the same rule as LoadedGltfEntity: false when
    // the node (or an ancestor) has no decomposed TRS rotation to read, so a
    // caller can fall back to treating it as unrotated rather than silently
    // getting it wrong. mesh.boundsMin/boundsMax (world-space, from already-
    // baked vertices) remain the right source for anything that just wants
    // "where roughly is this in the world" (camera clamps, the ground-flush
    // check, ...).
    Vec3 worldPosition;
    Quaternion worldOrientation;
    bool hasOrientation = false;
    Vec3 localBoundsMin;
    Vec3 localBoundsMax;
};

// A non-mesh node - typically a Blender "empty" placed to mark a gameplay
// location (a player spawn point, a trigger volume, ...) rather than
// something rendered. Position/orientation are already converted into the
// engine's Z-up world space, exactly like LoadedGltfPart's vertices.
struct LoadedGltfEntity {
    std::string name;
    Vec3 position;
    // Only meaningful when hasOrientation is true - a node with no explicit
    // rotation (or one nested under an ancestor using a raw matrix instead
    // of decomposed TRS, which this loader does not attempt to decompose)
    // leaves this at identity and hasOrientation false, so callers can tell
    // "faces nowhere in particular" apart from "faces exactly +X".
    Quaternion orientation;
    bool hasOrientation = false;
    GltfExtras extras;
};

struct LoadedGltfModel {
    std::vector<LoadedGltfPart> parts;
    std::vector<LoadedGltfMaterial> materials;
    std::vector<LoadedGltfImage> images;
    std::vector<LoadedGltfEntity> entities;
};

// Loads every triangle primitive out of a .gltf/.glb file (auto-detected),
// baking each mesh node's world transform into its vertices and converting
// glTF's Y-up axis convention to the engine's Z-up. Each material's
// baseColorFactor is folded into that material's vertex colors, so a
// texture-free, flat-color Blender export (upper/sole style materials)
// renders correctly through the exact same shader path as a textured one -
// see MeshRender3D. Non-mesh nodes are collected into LoadedGltfModel::
// entities instead of being dropped. Extras de material e node sao mesclados
// em cada parte (o node prevalece); entidades preservam os extras do proprio
// node. Throws std::runtime_error on any parse/read failure.
[[nodiscard]] LoadedGltfModel loadGltfModel(const std::string& path);

} // namespace MatterEngine
