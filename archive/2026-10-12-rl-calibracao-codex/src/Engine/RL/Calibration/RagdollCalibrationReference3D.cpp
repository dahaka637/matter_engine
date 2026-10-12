#include "Engine/RL/Calibration/RagdollCalibrationReference3D.hpp"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <functional>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>
#include <utility>

namespace MatterEngine {
namespace {

using Json = nlohmann::json;

bool finite(float value) { return std::isfinite(value); }

void addIssue(std::vector<RagdollCalibrationReferenceValidationIssue3D> &issues,
              std::string path, std::string message) {
  issues.push_back({std::move(path), std::move(message)});
}

void addFinding(RagdollCalibrationReferenceComparisonResult3D &result,
                CalibrationFindingSeverity3D severity, std::string code,
                std::string path, std::string message) {
  result.findings.push_back(
      {severity, std::move(code), std::move(path), std::move(message)});
}

const char *severityName(CalibrationFindingSeverity3D severity) {
  switch (severity) {
  case CalibrationFindingSeverity3D::Information:
    return "informacao";
  case CalibrationFindingSeverity3D::Warning:
    return "aviso";
  case CalibrationFindingSeverity3D::Error:
    return "erro";
  }
  return "desconhecido";
}

std::string fixed(float value, int precision = 4) {
  std::ostringstream output;
  output << std::fixed << std::setprecision(precision) << value;
  return output.str();
}

std::string joined(const std::vector<std::string> &values) {
  std::ostringstream output;
  for (std::size_t index = 0; index < values.size(); ++index) {
    if (index > 0)
      output << ", ";
    output << values[index];
  }
  return output.str();
}

RagdollCalibrationReferenceSource3D readSource(const Json &source) {
  RagdollCalibrationReferenceSource3D result;
  result.id = source.at("id").get<std::string>();
  result.title = source.at("title").get<std::string>();
  result.doi = source.value("doi", std::string{});
  result.url = source.value("url", std::string{});
  result.notes = source.value("notes", std::string{});
  return result;
}

RagdollMassReferenceGroup3D readMassGroup(const Json &source) {
  RagdollMassReferenceGroup3D result;
  result.id = source.at("id").get<std::string>();
  result.linkIds = source.at("linkIds").get<std::vector<std::string>>();
  result.targetMassFraction = source.at("targetMassFraction").get<float>();
  result.reviewToleranceRelative =
      source.at("reviewToleranceRelative").get<float>();
  result.sourceIds = source.at("sourceIds").get<std::vector<std::string>>();
  result.mappingNotes = source.value("mappingNotes", std::string{});
  return result;
}

RagdollCalibrationLandmarkKind3D readLandmarkKind(const std::string &value) {
  if (value == "pending")
    return RagdollCalibrationLandmarkKind3D::Pending;
  if (value == "profile_joint_center")
    return RagdollCalibrationLandmarkKind3D::ProfileJointCenter;
  if (value == "model_point")
    return RagdollCalibrationLandmarkKind3D::ModelPoint;
  if (value == "midpoint")
    return RagdollCalibrationLandmarkKind3D::Midpoint;
  throw std::invalid_argument("tipo de landmark desconhecido: " + value);
}

RagdollCalibrationLandmarkStatus3D
readLandmarkStatus(const std::string &value) {
  if (value == "pending")
    return RagdollCalibrationLandmarkStatus3D::Pending;
  if (value == "candidate")
    return RagdollCalibrationLandmarkStatus3D::Candidate;
  if (value == "approved")
    return RagdollCalibrationLandmarkStatus3D::Approved;
  throw std::invalid_argument("estado de landmark desconhecido: " + value);
}

RagdollCalibrationLandmark3D readLandmark(const Json &source) {
  RagdollCalibrationLandmark3D result;
  result.id = source.at("id").get<std::string>();
  result.label = source.at("label").get<std::string>();
  result.kind = readLandmarkKind(source.at("kind").get<std::string>());
  result.status = readLandmarkStatus(source.at("status").get<std::string>());
  result.jointChildLinkId = source.value("jointChildLinkId", std::string{});
  if (source.contains("modelPosition")) {
    const std::array<float, 3> position =
        source.at("modelPosition").get<std::array<float, 3>>();
    result.modelPosition = {position[0], position[1], position[2]};
    result.hasModelPosition = true;
  }
  result.midpointFirstLandmarkId =
      source.value("midpointFirstLandmarkId", std::string{});
  result.midpointSecondLandmarkId =
      source.value("midpointSecondLandmarkId", std::string{});
  result.provenance = source.value("provenance", std::string{});
  result.notes = source.value("notes", std::string{});
  return result;
}

RagdollInertialReferenceGroup3D readInertialGroup(const Json &source) {
  RagdollInertialReferenceGroup3D result;
  result.id = source.at("id").get<std::string>();
  result.originLandmarkId = source.at("originLandmarkId").get<std::string>();
  result.otherLandmarkId = source.at("otherLandmarkId").get<std::string>();
  result.targetCenterOfMassFromOriginFraction =
      source.at("targetCenterOfMassFromOriginFraction").get<float>();
  const Json &radii = source.at("targetRadiusOfGyration");
  result.targetRadiusOfGyration.sagittalFraction =
      radii.at("sagittalFraction").get<float>();
  result.targetRadiusOfGyration.transverseFraction =
      radii.at("transverseFraction").get<float>();
  result.targetRadiusOfGyration.longitudinalFraction =
      radii.at("longitudinalFraction").get<float>();
  result.centerOfMassReviewToleranceAbsolute =
      source.at("centerOfMassReviewToleranceAbsolute").get<float>();
  result.inertiaReviewToleranceRelative =
      source.at("inertiaReviewToleranceRelative").get<float>();
  result.sourceIds = source.at("sourceIds").get<std::vector<std::string>>();
  result.mappingNotes = source.value("mappingNotes", std::string{});
  return result;
}

const RagdollLinkDefinition3D *findProfileLink(const RagdollProfile3D &profile,
                                               std::string_view id) {
  const auto found = std::find_if(
      profile.links.begin(), profile.links.end(),
      [id](const RagdollLinkDefinition3D &link) { return link.id == id; });
  return found == profile.links.end() ? nullptr : &*found;
}

const RagdollLinkEffectiveMass3D *
findEffectiveLink(const RagdollEffectiveMass3D *properties,
                  std::string_view id) {
  if (!properties || !properties->valid)
    return nullptr;
  const auto found = std::find_if(
      properties->links.begin(), properties->links.end(),
      [id](const RagdollLinkEffectiveMass3D &link) { return link.id == id; });
  return found == properties->links.end() ? nullptr : &*found;
}

float momentAboutAxis(const Vec3 &principalMoments,
                      const Quaternion &principalAxesLocal, Vec3 axisLocal) {
  axisLocal = axisLocal.normalized();
  const Vec3 inPrincipalFrame =
      principalAxesLocal.conjugate().rotate(axisLocal);
  return principalMoments.x * inPrincipalFrame.x * inPrincipalFrame.x +
         principalMoments.y * inPrincipalFrame.y * inPrincipalFrame.y +
         principalMoments.z * inPrincipalFrame.z * inPrincipalFrame.z;
}

bool validFraction(float value) {
  return finite(value) && value > 0.0f && value <= 1.0f;
}

} // namespace

const char *
ragdollInertialLandmarkMappingName3D(RagdollInertialLandmarkMapping3D mapping) {
  switch (mapping) {
  case RagdollInertialLandmarkMapping3D::PendingLandmarks:
    return "pending_landmarks";
  case RagdollInertialLandmarkMapping3D::LandmarkRegistry:
    return "landmark_registry";
  }
  return "unknown";
}

const char *
ragdollCalibrationLandmarkKindName3D(RagdollCalibrationLandmarkKind3D kind) {
  switch (kind) {
  case RagdollCalibrationLandmarkKind3D::Pending:
    return "pending";
  case RagdollCalibrationLandmarkKind3D::ProfileJointCenter:
    return "profile_joint_center";
  case RagdollCalibrationLandmarkKind3D::ModelPoint:
    return "model_point";
  case RagdollCalibrationLandmarkKind3D::Midpoint:
    return "midpoint";
  }
  return "unknown";
}

const char *ragdollCalibrationLandmarkStatusName3D(
    RagdollCalibrationLandmarkStatus3D status) {
  switch (status) {
  case RagdollCalibrationLandmarkStatus3D::Pending:
    return "pending";
  case RagdollCalibrationLandmarkStatus3D::Candidate:
    return "candidate";
  case RagdollCalibrationLandmarkStatus3D::Approved:
    return "approved";
  }
  return "unknown";
}

std::size_t RagdollCalibrationReferenceComparisonResult3D::findingCount(
    CalibrationFindingSeverity3D severity) const {
  return static_cast<std::size_t>(
      std::count_if(findings.begin(), findings.end(),
                    [severity](const CalibrationFinding3D &finding) {
                      return finding.severity == severity;
                    }));
}

bool RagdollCalibrationReferenceComparisonResult3D::hasErrors() const {
  return findingCount(CalibrationFindingSeverity3D::Error) > 0;
}

RagdollCalibrationReference3D
loadRagdollCalibrationReference3D(std::string_view filePath) {
  std::ifstream input{std::string(filePath)};
  if (!input) {
    throw std::runtime_error(
        "Nao foi possivel abrir referencia de calibracao: " +
        std::string(filePath));
  }

  Json root;
  try {
    input >> root;
  } catch (const Json::exception &error) {
    throw std::runtime_error("JSON invalido em " + std::string(filePath) +
                             ": " + error.what());
  }

  RagdollCalibrationReference3D reference;
  try {
    reference.schemaVersion = root.at("schemaVersion").get<int>();
    reference.id = root.at("id").get<std::string>();
    reference.profileId = root.at("profileId").get<std::string>();
    reference.status = root.at("status").get<std::string>();
    reference.population = root.at("population").get<std::string>();
    reference.notes = root.value("notes", std::vector<std::string>{});
    for (const Json &source : root.at("sources"))
      reference.sources.push_back(readSource(source));
    for (const Json &landmark : root.at("landmarks"))
      reference.landmarks.push_back(readLandmark(landmark));
    for (const Json &group : root.at("massGroups"))
      reference.massGroups.push_back(readMassGroup(group));
    for (const Json &group : root.at("inertialGroups"))
      reference.inertialGroups.push_back(readInertialGroup(group));
  } catch (const Json::exception &error) {
    throw std::runtime_error("Referencia de calibracao incompleta em " +
                             std::string(filePath) + ": " + error.what());
  } catch (const std::invalid_argument &error) {
    throw std::runtime_error("Referencia de calibracao invalida em " +
                             std::string(filePath) + ": " + error.what());
  }

  validateRagdollCalibrationReferenceOrThrow3D(reference);
  return reference;
}

std::vector<RagdollResolvedCalibrationLandmark3D>
resolveRagdollCalibrationLandmarks3D(
    const RagdollProfile3D &profile,
    const RagdollCalibrationReference3D &reference) {
  std::vector<RagdollResolvedCalibrationLandmark3D> resolved;
  resolved.reserve(reference.landmarks.size());
  std::unordered_map<std::string, std::size_t> indices;
  for (const RagdollCalibrationLandmark3D &landmark : reference.landmarks) {
    indices.emplace(landmark.id, resolved.size());
    RagdollResolvedCalibrationLandmark3D output;
    output.id = landmark.id;
    output.label = landmark.label;
    output.kind = landmark.kind;
    output.status = landmark.status;
    output.provenance = landmark.provenance;
    output.notes = landmark.notes;
    resolved.push_back(std::move(output));
  }

  std::vector<unsigned char> state(reference.landmarks.size(), 0);
  std::function<bool(std::size_t)> resolve = [&](std::size_t index) {
    if (state[index] == 2)
      return resolved[index].resolved;
    if (state[index] == 1) {
      resolved[index].reason = "ciclo entre landmarks derivados";
      return false;
    }
    state[index] = 1;
    const RagdollCalibrationLandmark3D &landmark = reference.landmarks[index];
    RagdollResolvedCalibrationLandmark3D &output = resolved[index];
    switch (landmark.kind) {
    case RagdollCalibrationLandmarkKind3D::Pending:
      output.reason = "marco anatomico ainda nao localizado";
      break;
    case RagdollCalibrationLandmarkKind3D::ProfileJointCenter: {
      const RagdollLinkDefinition3D *child =
          findProfileLink(profile, landmark.jointChildLinkId);
      if (!child || child->parentIndex < 0) {
        output.reason = "link filho da junta nao existe ou pertence a raiz";
        break;
      }
      output.modelPosition = child->inboundJoint.anchorModelPosition;
      output.resolved = true;
      break;
    }
    case RagdollCalibrationLandmarkKind3D::ModelPoint:
      output.modelPosition = landmark.modelPosition;
      output.resolved = true;
      break;
    case RagdollCalibrationLandmarkKind3D::Midpoint: {
      const auto first = indices.find(landmark.midpointFirstLandmarkId);
      const auto second = indices.find(landmark.midpointSecondLandmarkId);
      if (first == indices.end() || second == indices.end()) {
        output.reason = "landmark-base inexistente";
        break;
      }
      if (!resolve(first->second) || !resolve(second->second)) {
        output.reason = "landmark-base ainda nao resolvido";
        break;
      }
      output.modelPosition = (resolved[first->second].modelPosition +
                              resolved[second->second].modelPosition) *
                             0.5f;
      output.resolved = true;
      break;
    }
    }
    state[index] = 2;
    return output.resolved;
  };
  for (std::size_t index = 0; index < resolved.size(); ++index)
    (void)resolve(index);
  return resolved;
}

std::vector<RagdollCalibrationReferenceValidationIssue3D>
validateRagdollCalibrationReference3D(
    const RagdollCalibrationReference3D &reference) {
  std::vector<RagdollCalibrationReferenceValidationIssue3D> issues;
  if (reference.schemaVersion != 3)
    addIssue(issues, "schemaVersion", "a versao suportada e 3");
  if (reference.id.empty())
    addIssue(issues, "id", "nao pode ser vazio");
  if (reference.profileId.empty())
    addIssue(issues, "profileId", "nao pode ser vazio");
  if (reference.status.empty())
    addIssue(issues, "status", "nao pode ser vazio");
  if (reference.population.empty())
    addIssue(issues, "population",
             "precisa declarar a populacao de referencia");
  if (reference.sources.empty())
    addIssue(issues, "sources", "precisa conter ao menos uma fonte");
  if (reference.landmarks.empty())
    addIssue(issues, "landmarks", "precisa conter os marcos anatomicos");
  if (reference.massGroups.empty())
    addIssue(issues, "massGroups", "precisa conter ao menos um grupo");
  if (reference.inertialGroups.empty())
    addIssue(issues, "inertialGroups", "precisa conter ao menos um grupo");

  std::unordered_set<std::string> sourceIds;
  for (std::size_t index = 0; index < reference.sources.size(); ++index) {
    const RagdollCalibrationReferenceSource3D &source =
        reference.sources[index];
    const std::string path = "sources[" + std::to_string(index) + "]";
    if (source.id.empty() || !sourceIds.insert(source.id).second)
      addIssue(issues, path + ".id", "precisa ser unico e nao vazio");
    if (source.title.empty())
      addIssue(issues, path + ".title", "nao pode ser vazio");
    if (source.doi.empty() && source.url.empty())
      addIssue(issues, path, "precisa declarar DOI ou URL");
  }

  std::unordered_map<std::string, const RagdollCalibrationLandmark3D *>
      landmarksById;
  for (std::size_t index = 0; index < reference.landmarks.size(); ++index) {
    const RagdollCalibrationLandmark3D &landmark = reference.landmarks[index];
    const std::string path = "landmarks[" + std::to_string(index) + "]";
    if (landmark.id.empty() ||
        !landmarksById.emplace(landmark.id, &landmark).second)
      addIssue(issues, path + ".id", "precisa ser unico e nao vazio");
    if (landmark.label.empty())
      addIssue(issues, path + ".label", "nao pode ser vazio");
    if (landmark.kind == RagdollCalibrationLandmarkKind3D::Pending) {
      if (landmark.status != RagdollCalibrationLandmarkStatus3D::Pending)
        addIssue(issues, path + ".status",
                 "landmark pending precisa ter estado pending");
    } else {
      if (landmark.status == RagdollCalibrationLandmarkStatus3D::Pending)
        addIssue(issues, path + ".status",
                 "landmark localizado precisa ser candidate ou approved");
      if (landmark.provenance.empty())
        addIssue(issues, path + ".provenance",
                 "landmark localizado precisa declarar procedencia");
    }
    switch (landmark.kind) {
    case RagdollCalibrationLandmarkKind3D::Pending:
      break;
    case RagdollCalibrationLandmarkKind3D::ProfileJointCenter:
      if (landmark.jointChildLinkId.empty())
        addIssue(issues, path + ".jointChildLinkId",
                 "nao pode ser vazio para profile_joint_center");
      break;
    case RagdollCalibrationLandmarkKind3D::ModelPoint:
      if (!landmark.hasModelPosition || !finite(landmark.modelPosition.x) ||
          !finite(landmark.modelPosition.y) ||
          !finite(landmark.modelPosition.z))
        addIssue(issues, path + ".modelPosition",
                 "precisa conter tres coordenadas finitas");
      break;
    case RagdollCalibrationLandmarkKind3D::Midpoint:
      if (landmark.midpointFirstLandmarkId.empty() ||
          landmark.midpointSecondLandmarkId.empty() ||
          landmark.midpointFirstLandmarkId == landmark.midpointSecondLandmarkId)
        addIssue(issues, path, "midpoint exige dois landmarks-base distintos");
      break;
    }
  }
  std::unordered_map<std::string, unsigned char> landmarkVisit;
  std::function<void(const std::string &, const std::string &)> visitLandmark =
      [&](const std::string &id, const std::string &path) {
        const auto found = landmarksById.find(id);
        if (found == landmarksById.end()) {
          addIssue(issues, path, "referencia um landmark inexistente");
          return;
        }
        unsigned char &state = landmarkVisit[id];
        if (state == 2)
          return;
        if (state == 1) {
          addIssue(issues, path, "forma um ciclo entre landmarks derivados");
          return;
        }
        state = 1;
        const RagdollCalibrationLandmark3D &landmark = *found->second;
        if (landmark.kind == RagdollCalibrationLandmarkKind3D::Midpoint) {
          visitLandmark(landmark.midpointFirstLandmarkId,
                        "landmarks." + id + ".midpointFirstLandmarkId");
          visitLandmark(landmark.midpointSecondLandmarkId,
                        "landmarks." + id + ".midpointSecondLandmarkId");
        }
        state = 2;
      };
  for (const auto &[id, landmark] : landmarksById) {
    (void)landmark;
    visitLandmark(id, "landmarks." + id);
  }

  std::unordered_set<std::string> groupIds;
  std::unordered_set<std::string> mappedLinks;
  float targetSum = 0.0f;
  for (std::size_t index = 0; index < reference.massGroups.size(); ++index) {
    const RagdollMassReferenceGroup3D &group = reference.massGroups[index];
    const std::string path = "massGroups[" + std::to_string(index) + "]";
    if (group.id.empty() || !groupIds.insert(group.id).second)
      addIssue(issues, path + ".id", "precisa ser unico e nao vazio");
    if (group.linkIds.empty())
      addIssue(issues, path + ".linkIds", "nao pode ser vazio");
    for (const std::string &linkId : group.linkIds) {
      if (linkId.empty() || !mappedLinks.insert(linkId).second) {
        addIssue(
            issues, path + ".linkIds",
            "cada link precisa ser nao vazio e aparecer em um unico grupo");
      }
    }
    if (!finite(group.targetMassFraction) || group.targetMassFraction <= 0.0f ||
        group.targetMassFraction > 1.0f) {
      addIssue(issues, path + ".targetMassFraction",
               "precisa estar no intervalo (0, 1]");
    } else {
      targetSum += group.targetMassFraction;
    }
    if (!finite(group.reviewToleranceRelative) ||
        group.reviewToleranceRelative < 0.0f ||
        group.reviewToleranceRelative > 1.0f) {
      addIssue(issues, path + ".reviewToleranceRelative",
               "precisa estar no intervalo [0, 1]");
    }
    if (group.sourceIds.empty())
      addIssue(issues, path + ".sourceIds", "nao pode ser vazio");
    for (const std::string &sourceId : group.sourceIds) {
      if (!sourceIds.contains(sourceId))
        addIssue(issues, path + ".sourceIds",
                 "referencia uma fonte inexistente");
    }
  }
  if (std::abs(targetSum - 1.0f) > 0.0001f) {
    addIssue(issues, "massGroups", "as fracoes-alvo precisam somar 1");
  }

  std::unordered_set<std::string> inertialGroupIds;
  for (std::size_t index = 0; index < reference.inertialGroups.size();
       ++index) {
    const RagdollInertialReferenceGroup3D &group =
        reference.inertialGroups[index];
    const std::string path = "inertialGroups[" + std::to_string(index) + "]";
    if (group.id.empty() || !inertialGroupIds.insert(group.id).second)
      addIssue(issues, path + ".id", "precisa ser unico e nao vazio");
    if (!groupIds.contains(group.id))
      addIssue(issues, path + ".id",
               "precisa corresponder a um grupo de massa existente");
    if (group.originLandmarkId.empty() || group.otherLandmarkId.empty() ||
        group.originLandmarkId == group.otherLandmarkId) {
      addIssue(issues, path, "precisa referenciar dois landmarks distintos");
    }
    if (!landmarksById.contains(group.originLandmarkId))
      addIssue(issues, path + ".originLandmarkId",
               "referencia um landmark inexistente");
    if (!landmarksById.contains(group.otherLandmarkId))
      addIssue(issues, path + ".otherLandmarkId",
               "referencia um landmark inexistente");
    if (!validFraction(group.targetCenterOfMassFromOriginFraction))
      addIssue(issues, path + ".targetCenterOfMassFromOriginFraction",
               "precisa estar no intervalo (0, 1]");
    if (!validFraction(group.targetRadiusOfGyration.sagittalFraction) ||
        !validFraction(group.targetRadiusOfGyration.transverseFraction) ||
        !validFraction(group.targetRadiusOfGyration.longitudinalFraction)) {
      addIssue(issues, path + ".targetRadiusOfGyration",
               "todas as fracoes precisam estar no intervalo (0, 1]");
    }
    if (!finite(group.centerOfMassReviewToleranceAbsolute) ||
        group.centerOfMassReviewToleranceAbsolute < 0.0f ||
        group.centerOfMassReviewToleranceAbsolute > 1.0f) {
      addIssue(issues, path + ".centerOfMassReviewToleranceAbsolute",
               "precisa estar no intervalo [0, 1]");
    }
    if (!finite(group.inertiaReviewToleranceRelative) ||
        group.inertiaReviewToleranceRelative < 0.0f ||
        group.inertiaReviewToleranceRelative > 1.0f) {
      addIssue(issues, path + ".inertiaReviewToleranceRelative",
               "precisa estar no intervalo [0, 1]");
    }
    if (group.sourceIds.empty())
      addIssue(issues, path + ".sourceIds", "nao pode ser vazio");
    for (const std::string &sourceId : group.sourceIds) {
      if (!sourceIds.contains(sourceId))
        addIssue(issues, path + ".sourceIds",
                 "referencia uma fonte inexistente");
    }
  }
  for (const std::string &groupId : groupIds) {
    if (!inertialGroupIds.contains(groupId))
      addIssue(issues, "inertialGroups",
               "nao contem o grupo de massa " + groupId);
  }
  return issues;
}

void validateRagdollCalibrationReferenceOrThrow3D(
    const RagdollCalibrationReference3D &reference) {
  const std::vector<RagdollCalibrationReferenceValidationIssue3D> issues =
      validateRagdollCalibrationReference3D(reference);
  if (issues.empty())
    return;
  std::ostringstream message;
  message << "Referencia de calibracao invalida:";
  for (const RagdollCalibrationReferenceValidationIssue3D &issue : issues)
    message << "\n- " << issue.path << ": " << issue.message;
  throw std::runtime_error(message.str());
}

RagdollCalibrationReferenceComparisonResult3D
compareRagdollCalibrationReference3D(
    const RagdollProfile3D &profile,
    const RagdollCalibrationReference3D &reference,
    const RagdollEffectiveMass3D *effectiveMassProperties) {
  RagdollCalibrationReferenceComparisonResult3D result;
  result.referenceId = reference.id;
  result.referenceStatus = reference.status;
  result.profileId = profile.id;
  result.population = reference.population;
  result.totalMassKg = profile.totalMassKg;
  result.sources = reference.sources;

  const std::vector<RagdollCalibrationReferenceValidationIssue3D>
      referenceIssues = validateRagdollCalibrationReference3D(reference);
  for (const RagdollCalibrationReferenceValidationIssue3D &issue :
       referenceIssues) {
    addFinding(result, CalibrationFindingSeverity3D::Error, "invalid_reference",
               issue.path, issue.message);
  }
  if (!referenceIssues.empty())
    return result;

  if (profile.id != reference.profileId) {
    addFinding(result, CalibrationFindingSeverity3D::Error,
               "reference_profile_mismatch", "profileId",
               "a referencia espera " + reference.profileId + ", mas recebeu " +
                   profile.id);
    return result;
  }

  std::unordered_map<std::string, float> profileFractions;
  profileFractions.reserve(profile.links.size());
  for (const RagdollLinkDefinition3D &link : profile.links)
    profileFractions.emplace(link.id, link.massFraction);

  std::unordered_set<std::string> consumedLinks;
  std::unordered_map<std::string, const RagdollMassReferenceGroup3D *>
      referenceMassGroups;
  for (const RagdollMassReferenceGroup3D &group : reference.massGroups) {
    referenceMassGroups.emplace(group.id, &group);
    RagdollMassReferenceComparison3D row;
    row.groupId = group.id;
    row.linkIds = group.linkIds;
    row.targetMassFraction = group.targetMassFraction;
    row.reviewToleranceRelative = group.reviewToleranceRelative;
    bool complete = true;
    for (const std::string &linkId : group.linkIds) {
      const auto found = profileFractions.find(linkId);
      if (found == profileFractions.end()) {
        complete = false;
        addFinding(result, CalibrationFindingSeverity3D::Error,
                   "reference_link_missing",
                   "massGroups." + group.id + ".linkIds",
                   "o perfil nao possui o link " + linkId);
        continue;
      }
      consumedLinks.insert(linkId);
      row.actualMassFraction += found->second;
    }
    row.differenceMassFraction =
        row.actualMassFraction - row.targetMassFraction;
    row.relativeDifference =
        row.differenceMassFraction / row.targetMassFraction;
    row.targetMassKg = row.targetMassFraction * profile.totalMassKg;
    row.actualMassKg = row.actualMassFraction * profile.totalMassKg;
    row.withinReviewTolerance =
        complete && std::abs(row.relativeDifference) <=
                        row.reviewToleranceRelative + 0.000001f;
    result.targetMassFractionSum += row.targetMassFraction;
    result.actualMassFractionSum += row.actualMassFraction;
    if (complete && !row.withinReviewTolerance) {
      addFinding(result, CalibrationFindingSeverity3D::Warning,
                 "mass_group_outside_review_band", "massGroups." + group.id,
                 "a fracao atual difere " +
                     fixed(std::abs(row.relativeDifference) * 100.0f, 1) +
                     "% do alvo; revisar a distribuicao antes de altera-la");
    }
    result.massGroups.push_back(std::move(row));
  }

  for (const RagdollLinkDefinition3D &link : profile.links) {
    if (!consumedLinks.contains(link.id)) {
      addFinding(result, CalibrationFindingSeverity3D::Error,
                 "profile_link_not_mapped", "links." + link.id,
                 "o link do perfil nao aparece na referencia");
    }
  }

  std::size_t mappedInertialGroupCount = 0;
  std::size_t pendingInertialGroupCount = 0;
  std::size_t comparedInertiaCount = 0;
  const std::vector<RagdollResolvedCalibrationLandmark3D> resolvedLandmarks =
      resolveRagdollCalibrationLandmarks3D(profile, reference);
  result.landmarks = resolvedLandmarks;
  std::unordered_map<std::string, const RagdollResolvedCalibrationLandmark3D *>
      resolvedLandmarksById;
  for (const RagdollResolvedCalibrationLandmark3D &landmark : resolvedLandmarks)
    resolvedLandmarksById.emplace(landmark.id, &landmark);
  for (const RagdollInertialReferenceGroup3D &group :
       reference.inertialGroups) {
    RagdollInertialReferenceComparison3D row;
    row.groupId = group.id;
    const auto originFound = resolvedLandmarksById.find(group.originLandmarkId);
    const auto otherFound = resolvedLandmarksById.find(group.otherLandmarkId);
    if (originFound != resolvedLandmarksById.end())
      row.sourceOrigin = originFound->second->label;
    if (otherFound != resolvedLandmarksById.end())
      row.sourceOther = otherFound->second->label;
    row.targetCenterOfMassFromOriginFraction =
        group.targetCenterOfMassFromOriginFraction;
    row.targetRadiusOfGyration = group.targetRadiusOfGyration;
    row.centerOfMassReviewToleranceAbsolute =
        group.centerOfMassReviewToleranceAbsolute;
    row.inertiaReviewToleranceRelative = group.inertiaReviewToleranceRelative;
    row.mappingNotes = group.mappingNotes;

    const auto massGroupFound = referenceMassGroups.find(group.id);
    if (massGroupFound == referenceMassGroups.end()) {
      addFinding(result, CalibrationFindingSeverity3D::Error,
                 "inertial_mass_group_missing", "inertialGroups." + group.id,
                 "o grupo inercial nao possui grupo de massa correspondente");
      result.inertialGroups.push_back(std::move(row));
      continue;
    }
    row.linkIds = massGroupFound->second->linkIds;
    const bool approvedAndResolved =
        originFound != resolvedLandmarksById.end() &&
        otherFound != resolvedLandmarksById.end() &&
        originFound->second->status ==
            RagdollCalibrationLandmarkStatus3D::Approved &&
        otherFound->second->status ==
            RagdollCalibrationLandmarkStatus3D::Approved &&
        originFound->second->resolved && otherFound->second->resolved;
    if (!approvedAndResolved) {
      row.mapping = RagdollInertialLandmarkMapping3D::PendingLandmarks;
      ++pendingInertialGroupCount;
      const bool approvedButUnresolved =
          originFound != resolvedLandmarksById.end() &&
          otherFound != resolvedLandmarksById.end() &&
          originFound->second->status ==
              RagdollCalibrationLandmarkStatus3D::Approved &&
          otherFound->second->status ==
              RagdollCalibrationLandmarkStatus3D::Approved;
      if (approvedButUnresolved) {
        addFinding(result, CalibrationFindingSeverity3D::Error,
                   "approved_landmark_unresolved", "inertialGroups." + group.id,
                   "um landmark aprovado nao pode ser resolvido no perfil");
      }
      result.inertialGroups.push_back(std::move(row));
      continue;
    }
    row.mapping = RagdollInertialLandmarkMapping3D::LandmarkRegistry;
    ++mappedInertialGroupCount;

    const Vec3 origin = originFound->second->modelPosition;
    const Vec3 other = otherFound->second->modelPosition;
    const Vec3 segment = other - origin;
    row.segmentLengthMeters = segment.length();
    if (!(row.segmentLengthMeters > 0.000001f)) {
      addFinding(result, CalibrationFindingSeverity3D::Error,
                 "inertial_segment_length_invalid",
                 "inertialGroups." + group.id,
                 "os centros articulares produzem comprimento nulo");
      result.inertialGroups.push_back(std::move(row));
      continue;
    }
    const Vec3 longitudinalAxis = segment / row.segmentLengthMeters;

    Vec3 weightedCenter;
    float groupWeight = 0.0f;
    bool centerComplete = true;
    for (const std::string &linkId : row.linkIds) {
      const RagdollLinkDefinition3D *link = findProfileLink(profile, linkId);
      if (!link) {
        centerComplete = false;
        continue;
      }
      const RagdollLinkEffectiveMass3D *effective =
          findEffectiveLink(effectiveMassProperties, linkId);
      const Vec3 centerLocal =
          effective ? effective->centerOfMassLocal : link->centerOfMassLocal;
      const float weight = effective ? effective->massKg : link->massFraction;
      weightedCenter +=
          (link->modelPosition + link->modelOrientation.rotate(centerLocal)) *
          weight;
      groupWeight += weight;
    }
    if (centerComplete && groupWeight > 0.0f) {
      const Vec3 center = weightedCenter / groupWeight;
      const float along = dot(center - origin, longitudinalAxis);
      const Vec3 projected = origin + longitudinalAxis * along;
      row.actualCenterOfMassFromOriginFraction =
          along / row.segmentLengthMeters;
      row.centerOfMassAxisOffsetMeters = (center - projected).length();
      row.centerOfMassComparable = true;
      row.centerOfMassWithinReviewTolerance =
          std::abs(row.actualCenterOfMassFromOriginFraction -
                   row.targetCenterOfMassFromOriginFraction) <=
          row.centerOfMassReviewToleranceAbsolute + 0.000001f;
      if (!row.centerOfMassWithinReviewTolerance) {
        addFinding(
            result, CalibrationFindingSeverity3D::Warning,
            "center_of_mass_outside_review_band", "inertialGroups." + group.id,
            "o COM longitudinal atual difere " +
                fixed(std::abs(row.actualCenterOfMassFromOriginFraction -
                               row.targetCenterOfMassFromOriginFraction) *
                          100.0f,
                      1) +
                " pontos percentuais do alvo");
      }
    }

    if (row.linkIds.size() == 1 && effectiveMassProperties &&
        effectiveMassProperties->valid) {
      const RagdollLinkDefinition3D *link =
          findProfileLink(profile, row.linkIds.front());
      const RagdollLinkEffectiveMass3D *effective =
          findEffectiveLink(effectiveMassProperties, row.linkIds.front());
      if (link && effective && effective->massKg > 0.0f) {
        const Vec3 longitudinalLocal =
            link->modelOrientation.conjugate().rotate(longitudinalAxis);
        const float unconditionedMoment = momentAboutAxis(
            effective->unconditionedPrincipalInertiaKgSquareMeters,
            effective->unconditionedPrincipalAxesLocal, longitudinalLocal);
        const float effectiveMoment =
            momentAboutAxis(effective->principalInertiaKgSquareMeters,
                            effective->principalAxesLocal, longitudinalLocal);
        if (unconditionedMoment > 0.0f && effectiveMoment > 0.0f) {
          row.unconditionedLongitudinalRadiusFraction =
              std::sqrt(unconditionedMoment / effective->massKg) /
              row.segmentLengthMeters;
          row.effectiveLongitudinalRadiusFraction =
              std::sqrt(effectiveMoment / effective->massKg) /
              row.segmentLengthMeters;
          row.inertiaComparable = true;
          ++comparedInertiaCount;
          const float relativeDifference =
              (row.unconditionedLongitudinalRadiusFraction -
               row.targetRadiusOfGyration.longitudinalFraction) /
              row.targetRadiusOfGyration.longitudinalFraction;
          row.unconditionedInertiaWithinReviewTolerance =
              std::abs(relativeDifference) <=
              row.inertiaReviewToleranceRelative + 0.000001f;
          if (!row.unconditionedInertiaWithinReviewTolerance) {
            addFinding(
                result, CalibrationFindingSeverity3D::Warning,
                "longitudinal_inertia_outside_review_band",
                "inertialGroups." + group.id,
                "o raio de giracao longitudinal anterior ao Stabilize difere " +
                    fixed(std::abs(relativeDifference) * 100.0f, 1) +
                    "% do alvo");
          }
        }
      }
    }
    result.inertialGroups.push_back(std::move(row));
  }

  if (pendingInertialGroupCount > 0) {
    addFinding(result, CalibrationFindingSeverity3D::Information,
               "inertial_landmarks_pending", "inertialGroups",
               std::to_string(pendingInertialGroupCount) +
                   " grupos preservam os dados publicados, mas aguardam "
                   "marcos anatomicos explicitos no asset");
  }
  if (mappedInertialGroupCount > 0 && comparedInertiaCount == 0) {
    addFinding(result, CalibrationFindingSeverity3D::Information,
               "effective_mass_required", "inertialGroups",
               "a comparacao de inercia longitudinal exige as propriedades "
               "de massa publicadas pelo backend");
  }
  addFinding(result, CalibrationFindingSeverity3D::Information,
             "inertial_axis_scope", "inertialGroups",
             "somente o eixo longitudinal e comparado; os alvos sagital e "
             "transversal ficam versionados ate existir um frame anatomico "
             "auditavel");
  addFinding(result, CalibrationFindingSeverity3D::Information,
             "reference_is_not_individual", "population",
             "a referencia descreve uma populacao publicada, nao as medidas "
             "individuais "
             "do personagem");
  if (reference.status != "approved") {
    addFinding(result, CalibrationFindingSeverity3D::Information,
               "reference_not_approved", "status",
               "o conjunto esta marcado como " + reference.status +
                   " e nao autoriza alteracao automatica do perfil");
  }
  return result;
}

std::string renderRagdollCalibrationReferenceMarkdown3D(
    const RagdollCalibrationReferenceComparisonResult3D &comparison) {
  std::ostringstream output;
  output << "# Comparacao antropometrica `" << comparison.referenceId << "`\n\n"
         << "> Comparacao de revisao. Nao altera o ragdoll e nao certifica um "
            "individuo.\n\n"
         << "## Resumo\n\n"
         << "- Perfil: `" << comparison.profileId << "`\n"
         << "- Estado da referencia: `" << comparison.referenceStatus << "`\n"
         << "- Populacao declarada: " << comparison.population << "\n"
         << "- Massa do perfil: " << fixed(comparison.totalMassKg, 3) << " kg\n"
         << "- Soma alvo / atual mapeada: "
         << fixed(comparison.targetMassFractionSum, 6) << " / "
         << fixed(comparison.actualMassFractionSum, 6) << "\n"
         << "- Achados: "
         << comparison.findingCount(CalibrationFindingSeverity3D::Error)
         << " erros, "
         << comparison.findingCount(CalibrationFindingSeverity3D::Warning)
         << " avisos, "
         << comparison.findingCount(CalibrationFindingSeverity3D::Information)
         << " informacoes\n\n";

  output << "## Fontes\n\n"
         << "| Id | Titulo | DOI/URL | Limites registrados |\n"
         << "|---|---|---|---|\n";
  for (const RagdollCalibrationReferenceSource3D &source : comparison.sources) {
    output << "| `" << source.id << "` | " << source.title << " | ";
    if (!source.doi.empty())
      output << "`" << source.doi << "`";
    else
      output << source.url;
    output << " | " << source.notes << " |\n";
  }

  output << "\n## Landmarks anatomicos\n\n"
         << "| Id | Marco | Tipo | Estado | Posicao de modelo | Procedencia / "
            "pendencia |\n"
         << "|---|---|---|---|---|---|\n";
  for (const RagdollResolvedCalibrationLandmark3D &landmark :
       comparison.landmarks) {
    output << "| `" << landmark.id << "` | " << landmark.label << " | `"
           << ragdollCalibrationLandmarkKindName3D(landmark.kind) << "` | `"
           << ragdollCalibrationLandmarkStatusName3D(landmark.status) << "` | ";
    if (landmark.resolved) {
      output << "`" << fixed(landmark.modelPosition.x) << ", "
             << fixed(landmark.modelPosition.y) << ", "
             << fixed(landmark.modelPosition.z) << "` m";
    } else {
      output << "-";
    }
    output << " | ";
    if (!landmark.provenance.empty())
      output << landmark.provenance;
    else if (!landmark.notes.empty())
      output << landmark.notes;
    else
      output << landmark.reason;
    output << " |\n";
  }
  output << "\nSomente landmarks `approved` e resolvidos participam das "
            "comparacoes numericas. Candidatos e pendencias continuam "
            "visiveis no relatorio, sem virar constantes silenciosas.\n\n";

  output << "## Distribuicao de massa\n\n"
         << "| Grupo | Links Matter | Alvo | Atual | Diferenca | Alvo/atual em "
            "80 kg | "
            "Banda de revisao |\n"
         << "|---|---|---:|---:|---:|---:|:---:|\n";
  for (const RagdollMassReferenceComparison3D &row : comparison.massGroups) {
    output << "| `" << row.groupId << "` | " << joined(row.linkIds) << " | "
           << fixed(row.targetMassFraction * 100.0f, 2) << "% | "
           << fixed(row.actualMassFraction * 100.0f, 2) << "% | "
           << (row.relativeDifference >= 0.0f ? "+" : "")
           << fixed(row.relativeDifference * 100.0f, 1) << "% | "
           << fixed(row.targetMassKg, 2) << " / " << fixed(row.actualMassKg, 2)
           << " kg | " << (row.withinReviewTolerance ? "dentro" : "revisar")
           << " |\n";
  }

  output << "\nA banda acima e uma tolerancia de triagem definida pela engine; "
            "nao e "
            "intervalo biologico da publicacao.\n\n";

  output << "## Centro de massa e inercia\n\n"
         << "| Grupo | Marcos da fonte | Mapeamento Matter | Comprimento | COM "
            "alvo / atual | Offset do eixo | r sag./trans./long. alvo | r "
            "long. shape / solver | Revisao |\n"
         << "|---|---|---|---:|---:|---:|---:|---:|:---:|\n";
  for (const RagdollInertialReferenceComparison3D &row :
       comparison.inertialGroups) {
    output << "| `" << row.groupId << "` | " << row.sourceOrigin << " -> "
           << row.sourceOther << " | `"
           << ragdollInertialLandmarkMappingName3D(row.mapping) << "` | ";
    if (row.centerOfMassComparable) {
      output << fixed(row.segmentLengthMeters, 4) << " m | "
             << fixed(row.targetCenterOfMassFromOriginFraction * 100.0f, 2)
             << "% / "
             << fixed(row.actualCenterOfMassFromOriginFraction * 100.0f, 2)
             << "% | " << fixed(row.centerOfMassAxisOffsetMeters * 1000.0f, 1)
             << " mm | ";
    } else {
      output << "- | "
             << fixed(row.targetCenterOfMassFromOriginFraction * 100.0f, 2)
             << "% / - | - | ";
    }
    output << fixed(row.targetRadiusOfGyration.sagittalFraction * 100.0f, 1)
           << "% / "
           << fixed(row.targetRadiusOfGyration.transverseFraction * 100.0f, 1)
           << "% / "
           << fixed(row.targetRadiusOfGyration.longitudinalFraction * 100.0f, 1)
           << "% | ";
    if (row.inertiaComparable) {
      output << fixed(row.unconditionedLongitudinalRadiusFraction * 100.0f, 1)
             << "% / "
             << fixed(row.effectiveLongitudinalRadiusFraction * 100.0f, 1)
             << "% | "
             << (row.centerOfMassWithinReviewTolerance &&
                         row.unconditionedInertiaWithinReviewTolerance
                     ? "dentro"
                     : "revisar");
    } else {
      output << "- / - | "
             << (row.mapping ==
                         RagdollInertialLandmarkMapping3D::PendingLandmarks
                     ? "marcos pendentes"
                     : "massa efetiva pendente");
    }
    output << " |\n";
  }
  output << "\n`shape` usa a inercia calculada do collider antes do "
            "condicionamento; `solver` mostra o tensor realmente usado apos "
            "`Stabilize`. Apenas o raio longitudinal e comparado nesta versao. "
            "As bandas de COM e inercia sao triagem da engine, nao intervalos "
            "biologicos da publicacao.\n\n"
         << "## Achados\n\n"
         << "| Severidade | Codigo | Caminho | Mensagem |\n"
         << "|---|---|---|---|\n";
  if (comparison.findings.empty()) {
    output << "| informacao | none | - | Nenhum achado. |\n";
  } else {
    for (const CalibrationFinding3D &finding : comparison.findings) {
      output << "| " << severityName(finding.severity) << " | `" << finding.code
             << "` | `" << finding.path << "` | " << finding.message << " |\n";
    }
  }
  return output.str();
}

} // namespace MatterEngine
