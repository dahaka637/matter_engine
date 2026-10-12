#pragma once

#include "Engine/Modeling/Mesh/Mesh.hpp"
#include "Engine/Modeling/Skeleton/Skeleton.hpp"

#include <cstddef>
#include <filesystem>
#include <optional>
#include <span>
#include <string>
#include <vector>

namespace MatterEngine::Modeling {

// Pistas de autoria pertencem ao modelo visual, nao a referencia cientifica.
// Elas podem ser medidas e desenhadas, mas so passam a alimentar a calibracao
// depois de uma promocao explicita no asset antropometrico.
enum class AnatomicalLandmarkCandidateSource3D {
  SkeletonJoint,
  MeshPartExtreme,
};

enum class AnatomicalLandmarkExtreme3D {
  Minimum,
  Maximum,
};

struct AnatomicalLandmarkCandidateDefinition3D {
  std::string landmarkId;
  AnatomicalLandmarkCandidateSource3D source =
      AnatomicalLandmarkCandidateSource3D::SkeletonJoint;
  std::string jointName;
  std::string meshPartName;
  std::string axisFromJointName;
  std::string axisToJointName;
  AnatomicalLandmarkExtreme3D extreme = AnatomicalLandmarkExtreme3D::Minimum;
  float selectionToleranceMeters = 0.0f;
  std::string provenance;
  std::string notes;
};

struct AnatomicalLandmarkCandidateSet3D {
  int schemaVersion = 0;
  std::string id;
  std::string profileId;
  std::string referenceId;
  std::vector<std::string> notes;
  std::vector<AnatomicalLandmarkCandidateDefinition3D> candidates;
};

struct ResolvedAnatomicalLandmarkCandidate3D {
  std::string landmarkId;
  AnatomicalLandmarkCandidateSource3D source =
      AnatomicalLandmarkCandidateSource3D::SkeletonJoint;
  Vec3 modelPosition;
  bool resolved = false;
  std::size_t supportPointCount = 0;
  std::string provenance;
  std::string notes;
  std::string reason;
};

[[nodiscard]] const char *anatomicalLandmarkCandidateSourceName3D(
    AnatomicalLandmarkCandidateSource3D source);

[[nodiscard]] std::optional<AnatomicalLandmarkCandidateSet3D>
loadAnatomicalLandmarkCandidateSet3D(const std::filesystem::path &path,
                                     std::string *error = nullptr);

[[nodiscard]] std::vector<std::string> validateAnatomicalLandmarkCandidateSet3D(
    const AnatomicalLandmarkCandidateSet3D &set);

[[nodiscard]] std::vector<ResolvedAnatomicalLandmarkCandidate3D>
resolveAnatomicalLandmarkCandidates3D(
    const AnatomicalLandmarkCandidateSet3D &set, const Skeleton &skeleton,
    const SkeletonPose &pose, const Mesh &mesh,
    std::span<const Vec3> meshPositions);

} // namespace MatterEngine::Modeling
