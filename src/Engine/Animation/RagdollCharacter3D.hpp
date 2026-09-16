#pragma once

#include "Engine/Geometry/MeshData3D.hpp"
#include "Engine/Math/Mat4.hpp"
#include "Engine/Physics/RagdollProfile3D.hpp"
#include <span>

namespace MatterEngine {

// A character pairs a physical profile with a weighted visual surface.
// Bone names are resolved once on load, never guessed during playback.
struct RagdollCharacter3D {
    std::string id;
    std::string displayName;
    std::string thumbnailPath;
    std::string albedoPath;
    std::string idleClipId, runClipId, stopClipId;
    RagdollProfile3D profile;
    MeshData3D mesh;
    std::vector<std::size_t> boneLinkIndices;
    std::vector<Mat4> inverseBindMatrices;
    bool flatShaded = true;
};

[[nodiscard]] RagdollCharacter3D loadRagdollCharacter3D(const std::string& manifestPath);
[[nodiscard]] std::vector<Mat4> buildRagdollSkinMatrices3D(
    const RagdollCharacter3D& character,
    std::span<const Vec3> positions, std::span<const Quaternion> orientations);
// Reference CPU deformation used by asset validation, independent of GPU.
[[nodiscard]] Vec3 skinVertexPosition3D(const MeshVertex3D& vertex,
    std::span<const Mat4> palette);

} // namespace MatterEngine
