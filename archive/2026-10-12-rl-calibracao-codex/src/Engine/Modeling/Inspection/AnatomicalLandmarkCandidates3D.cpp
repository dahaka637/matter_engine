#include "Engine/Modeling/Inspection/AnatomicalLandmarkCandidates3D.hpp"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <sstream>
#include <unordered_set>

namespace MatterEngine::Modeling {
namespace {

using Json = nlohmann::json;

AnatomicalLandmarkCandidateSource3D sourceFromName(const std::string &name) {
  if (name == "skeleton_joint")
    return AnatomicalLandmarkCandidateSource3D::SkeletonJoint;
  if (name == "mesh_part_extreme")
    return AnatomicalLandmarkCandidateSource3D::MeshPartExtreme;
  throw std::invalid_argument("fonte de candidato desconhecida: " + name);
}

AnatomicalLandmarkExtreme3D extremeFromName(const std::string &name) {
  if (name == "minimum")
    return AnatomicalLandmarkExtreme3D::Minimum;
  if (name == "maximum")
    return AnatomicalLandmarkExtreme3D::Maximum;
  throw std::invalid_argument("extremo de candidato desconhecido: " + name);
}

AnatomicalLandmarkCandidateDefinition3D readCandidate(const Json &source) {
  AnatomicalLandmarkCandidateDefinition3D candidate;
  candidate.landmarkId = source.at("landmarkId").get<std::string>();
  candidate.source = sourceFromName(source.at("source").get<std::string>());
  candidate.jointName = source.value("jointName", std::string{});
  candidate.meshPartName = source.value("meshPartName", std::string{});
  candidate.axisFromJointName =
      source.value("axisFromJointName", std::string{});
  candidate.axisToJointName = source.value("axisToJointName", std::string{});
  candidate.extreme =
      extremeFromName(source.value("extreme", std::string("minimum")));
  candidate.selectionToleranceMeters =
      source.value("selectionToleranceMeters", 0.0f);
  candidate.provenance = source.at("provenance").get<std::string>();
  candidate.notes = source.value("notes", std::string{});
  return candidate;
}

} // namespace

const char *anatomicalLandmarkCandidateSourceName3D(
    AnatomicalLandmarkCandidateSource3D source) {
  switch (source) {
  case AnatomicalLandmarkCandidateSource3D::SkeletonJoint:
    return "skeleton_joint";
  case AnatomicalLandmarkCandidateSource3D::MeshPartExtreme:
    return "mesh_part_extreme";
  }
  return "unknown";
}

std::optional<AnatomicalLandmarkCandidateSet3D>
loadAnatomicalLandmarkCandidateSet3D(const std::filesystem::path &path,
                                     std::string *error) {
  try {
    std::ifstream input(path);
    if (!input)
      throw std::runtime_error("nao abriu " + path.string());
    Json root;
    input >> root;
    AnatomicalLandmarkCandidateSet3D set;
    set.schemaVersion = root.at("schemaVersion").get<int>();
    set.id = root.at("id").get<std::string>();
    set.profileId = root.at("profileId").get<std::string>();
    set.referenceId = root.at("referenceId").get<std::string>();
    set.notes = root.value("notes", std::vector<std::string>{});
    for (const Json &candidate : root.at("candidates"))
      set.candidates.push_back(readCandidate(candidate));
    const std::vector<std::string> issues =
        validateAnatomicalLandmarkCandidateSet3D(set);
    if (!issues.empty()) {
      std::ostringstream message;
      message << "asset invalido";
      for (const std::string &issue : issues)
        message << "\n- " << issue;
      throw std::runtime_error(message.str());
    }
    return set;
  } catch (const std::exception &exception) {
    if (error != nullptr)
      *error = "candidatos anatomicos: " + std::string(exception.what());
    return std::nullopt;
  }
}

std::vector<std::string> validateAnatomicalLandmarkCandidateSet3D(
    const AnatomicalLandmarkCandidateSet3D &set) {
  std::vector<std::string> issues;
  if (set.schemaVersion != 1)
    issues.push_back("schemaVersion: a versao suportada e 1");
  if (set.id.empty())
    issues.push_back("id: nao pode ser vazio");
  if (set.profileId.empty())
    issues.push_back("profileId: nao pode ser vazio");
  if (set.referenceId.empty())
    issues.push_back("referenceId: nao pode ser vazio");
  if (set.candidates.empty())
    issues.push_back("candidates: precisa conter ao menos uma pista");
  std::unordered_set<std::string> landmarkIds;
  for (std::size_t index = 0; index < set.candidates.size(); ++index) {
    const AnatomicalLandmarkCandidateDefinition3D &candidate =
        set.candidates[index];
    const std::string path = "candidates[" + std::to_string(index) + "]";
    if (candidate.landmarkId.empty() ||
        !landmarkIds.insert(candidate.landmarkId).second)
      issues.push_back(path + ".landmarkId: precisa ser unico e nao vazio");
    if (candidate.provenance.empty())
      issues.push_back(path + ".provenance: nao pode ser vazia");
    if (candidate.source ==
        AnatomicalLandmarkCandidateSource3D::SkeletonJoint) {
      if (candidate.jointName.empty())
        issues.push_back(path + ".jointName: nao pode ser vazio");
    } else {
      if (candidate.meshPartName.empty())
        issues.push_back(path + ".meshPartName: nao pode ser vazio");
      if (candidate.axisFromJointName.empty() ||
          candidate.axisToJointName.empty() ||
          candidate.axisFromJointName == candidate.axisToJointName)
        issues.push_back(
            path + ": mesh_part_extreme exige duas juntas de eixo distintas");
      if (!std::isfinite(candidate.selectionToleranceMeters) ||
          candidate.selectionToleranceMeters < 0.0f ||
          candidate.selectionToleranceMeters > 0.05f)
        issues.push_back(
            path + ".selectionToleranceMeters: precisa estar em [0, 0.05]");
    }
  }
  return issues;
}

std::vector<ResolvedAnatomicalLandmarkCandidate3D>
resolveAnatomicalLandmarkCandidates3D(
    const AnatomicalLandmarkCandidateSet3D &set, const Skeleton &skeleton,
    const SkeletonPose &pose, const Mesh &mesh,
    std::span<const Vec3> meshPositions) {
  std::vector<ResolvedAnatomicalLandmarkCandidate3D> result;
  result.reserve(set.candidates.size());
  const bool validPose = pose.joints.size() == skeleton.joints.size();
  const bool validMeshPositions = meshPositions.size() == mesh.positions.size();
  for (const AnatomicalLandmarkCandidateDefinition3D &candidate :
       set.candidates) {
    ResolvedAnatomicalLandmarkCandidate3D resolved;
    resolved.landmarkId = candidate.landmarkId;
    resolved.source = candidate.source;
    resolved.provenance = candidate.provenance;
    resolved.notes = candidate.notes;
    if (!validPose) {
      resolved.reason = "a pose nao cobre o esqueleto";
      result.push_back(std::move(resolved));
      continue;
    }
    if (candidate.source ==
        AnatomicalLandmarkCandidateSource3D::SkeletonJoint) {
      const int joint = skeleton.find(candidate.jointName);
      if (joint < 0) {
        resolved.reason = "junta visual inexistente: " + candidate.jointName;
      } else {
        resolved.modelPosition =
            pose.joints[static_cast<std::size_t>(joint)].position;
        resolved.resolved = true;
        resolved.supportPointCount = 1;
      }
      result.push_back(std::move(resolved));
      continue;
    }

    if (!validMeshPositions) {
      resolved.reason = "as posicoes deformadas nao cobrem a malha";
      result.push_back(std::move(resolved));
      continue;
    }
    const int part = mesh.findPart(candidate.meshPartName);
    const int from = skeleton.find(candidate.axisFromJointName);
    const int to = skeleton.find(candidate.axisToJointName);
    if (part < 0 || from < 0 || to < 0) {
      resolved.reason =
          part < 0 ? "parte de malha inexistente: " + candidate.meshPartName
                   : "junta do eixo inexistente";
      result.push_back(std::move(resolved));
      continue;
    }
    const Vec3 origin = pose.joints[static_cast<std::size_t>(from)].position;
    const Vec3 axisVector =
        pose.joints[static_cast<std::size_t>(to)].position - origin;
    if (axisVector.length() <= 0.000001f) {
      resolved.reason = "as juntas do eixo produzem comprimento nulo";
      result.push_back(std::move(resolved));
      continue;
    }
    const Vec3 axis = axisVector.normalized();
    const std::vector<std::uint32_t> vertices = mesh.partVertices(part);
    float extreme = candidate.extreme == AnatomicalLandmarkExtreme3D::Minimum
                        ? std::numeric_limits<float>::max()
                        : -std::numeric_limits<float>::max();
    for (const std::uint32_t vertex : vertices) {
      if (vertex >= meshPositions.size())
        continue;
      const float projection = dot(meshPositions[vertex] - origin, axis);
      extreme = candidate.extreme == AnatomicalLandmarkExtreme3D::Minimum
                    ? std::min(extreme, projection)
                    : std::max(extreme, projection);
    }
    Vec3 sum;
    for (const std::uint32_t vertex : vertices) {
      if (vertex >= meshPositions.size())
        continue;
      const float projection = dot(meshPositions[vertex] - origin, axis);
      if (std::abs(projection - extreme) <=
          candidate.selectionToleranceMeters + 0.000001f) {
        sum += meshPositions[vertex];
        ++resolved.supportPointCount;
      }
    }
    if (resolved.supportPointCount == 0) {
      resolved.reason = "a parte nao forneceu vertices no extremo";
    } else {
      resolved.modelPosition =
          sum / static_cast<float>(resolved.supportPointCount);
      resolved.resolved = true;
    }
    result.push_back(std::move(resolved));
  }
  return result;
}

} // namespace MatterEngine::Modeling
