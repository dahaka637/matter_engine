#pragma once

#include "Engine/Geometry/MeshData3D.hpp"
#include "Engine/Math/Mat4.hpp"
#include "Engine/Physics/RagdollProfile3D.hpp"
#include <span>

namespace MatterEngine {

// Osso visual (os dedos): segue um osso pai e nao tem fisica. A pose de
// ligacao fica guardada em relacao ao pai; restRotation dobra o osso na pose
// de repouso (a mao relaxada, semifechada). Animar a mao - o goleiro, no
// futuro - e trocar essas rotacoes.
struct RagdollVisualBone3D {
    std::string name;
    std::size_t parentBone = 0; // na paleta: fisicos primeiro, depois visuais
    Vec3 localBindPosition;
    Quaternion localBindOrientation;
    Quaternion restRotation;
};

// A character pairs a physical profile with a weighted visual surface.
// Bone names are resolved once on load, never guessed during playback.
struct RagdollCharacter3D {
    std::string id;
    std::string displayName;
    std::string thumbnailPath;
    std::string albedoPath;
    // Papéis de locomoção (manifesto: "locomotion"). idle e walk são
    // obrigatórios; walkBackward e sprint são opcionais - sem eles o corpo
    // gira para o movimento e não há sprint. sprintBackward (opcional) é o
    // recuo com sprint; sem ele, recuando, segurar sprint não corre.
    std::string idleClipId, walkClipId, walkBackwardClipId, sprintClipId;
    std::string sprintBackwardClipId;
    // Strafe (opcionais): correr de lado de frente para a camera.
    std::string strafeLeftClipId, strafeRightClipId;
    std::string sprintStrafeLeftClipId, sprintStrafeRightClipId;
    // Pulos (opcionais): parado e por setor de movimento.
    std::string jumpStandingClipId, jumpForwardClipId, jumpBackwardClipId;
    std::string jumpLeftClipId, jumpRightClipId;
    std::string standUpFrontClipId, standUpBackClipId;
    RagdollProfile3D profile;
    MeshData3D mesh;
    // Paleta: um osso por link fisico (boneLinkIndices), depois os visuais.
    std::vector<std::size_t> boneLinkIndices;
    std::vector<RagdollVisualBone3D> visualBones;
    std::vector<Mat4> inverseBindMatrices; // fisicos e visuais
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
