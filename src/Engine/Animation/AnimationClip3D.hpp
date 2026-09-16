#pragma once

#include "Engine/Math/Quaternion.hpp"

#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

namespace MatterEngine {

struct RagdollProfile3D;

enum class AnimationTrackSpace3D {
    Root,
    Joint
};

// Formato canônico entre importadores de animação e os consumidores do
// motor. Um importador futuro (glTF, FBX convertido, BVH etc.) só precisa
// retargetear os nomes/eixos do arquivo para os IDs do RagdollProfile3D e
// produzir este contrato; Workbench, controle físico e playback não precisam
// conhecer o formato de origem.
struct AnimationKeyframe3D {
    float timeSeconds = 0.0f;
    // Os dois campos seguintes são usados exclusivamente pelo canal root.
    Vec3 translationOffsetMeters;
    Quaternion rotationDelta;
    // Para links articulados, coordenadas exponenciais da junta física:
    // x=twist, y=swing1, z=swing2. Interpolar estes escalares mantém cada
    // quadro intermediário no espaço legal do ragdoll.
    Vec3 jointPositionRadians;
};

struct AnimationTrack3D {
    std::string targetLinkId;
    AnimationTrackSpace3D space = AnimationTrackSpace3D::Joint;
    std::vector<AnimationKeyframe3D> keyframes;
};

struct AnimationRetargetReport3D {
    bool available = false;
    bool passed = false;
    float directionRmsDegrees = 0.0f;
    float directionMaxDegrees = 0.0f;
    float limbDirectionMaxDegrees = 0.0f;
    float maximumJointStepDegrees = 0.0f;
    std::uint32_t limitHitCount = 0;
};

struct AnimationClip3D {
    std::string id;
    std::string displayName;
    std::string targetRigId;
    // Conserva a procedência para que ajustes de retarget possam ser refeitos
    // sem adivinhar de qual arquivo veio o clipe normalizado.
    std::string sourceAssetPath;
    float durationSeconds = 0.0f;
    float sourceSampleRateHz = 0.0f;
    bool loops = true;
    // Original displacement remains available even when the preview is in-place.
    Vec3 sourceRootDisplacementMeters;
    AnimationRetargetReport3D retargetReport;
    std::vector<AnimationTrack3D> tracks;
};

struct AnimationTransformSample3D {
    Vec3 translationOffsetMeters;
    Quaternion rotationDelta;
    Vec3 jointPositionRadians;
};

struct RagdollAnimationPose3D {
    std::vector<Vec3> linkPositions;
    std::vector<Quaternion> linkOrientations;
};

struct AnimationClipValidationIssue3D {
    std::string path;
    std::string message;
};

// Carrega o formato canônico matter-ragdoll-animation-1. Erros de arquivo,
// JSON, schema ou validação são comunicados por std::runtime_error.
[[nodiscard]] AnimationClip3D loadAnimationClip3D(std::string_view filePath);
[[nodiscard]] AnimationTransformSample3D sampleAnimationTrack3D(
    const AnimationTrack3D& track, float timeSeconds,
    float clipDurationSeconds, bool loop);
[[nodiscard]] std::vector<AnimationClipValidationIssue3D>
    validateAnimationClip3D(const AnimationClip3D& clip);
[[nodiscard]] std::vector<AnimationClipValidationIssue3D>
    validateAnimationClipForRagdoll3D(
        const AnimationClip3D& clip, const RagdollProfile3D& profile);
[[nodiscard]] const AnimationTrack3D* findAnimationTrack3D(
    const AnimationClip3D& clip, std::string_view targetLinkId);
// Amostra e projeta um clipe na articulação física real. Links filhos não
// recebem translação livre: as duas âncoras coincidem por construção, e cada
// componente angular é limitada pelos DOFs do RagdollProfile3D.
[[nodiscard]] RagdollAnimationPose3D sampleRagdollAnimationPose3D(
    const RagdollProfile3D& profile, const AnimationClip3D* clip,
    float timeSeconds, bool loop);

} // namespace MatterEngine
