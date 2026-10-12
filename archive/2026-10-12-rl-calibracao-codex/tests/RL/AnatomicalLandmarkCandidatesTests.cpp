#include "Engine/Modeling/Inspection/AnatomicalLandmarkCandidates3D.hpp"
#include "Engine/Modeling/Rig/PlayerModelAsset.hpp"
#include "Engine/RL/Calibration/RagdollCalibrationReference3D.hpp"

#include <algorithm>
#include <cmath>
#include <exception>
#include <iostream>
#include <stdexcept>
#include <string>
#include <unordered_map>

namespace {

using namespace MatterEngine;
using namespace MatterEngine::Modeling;

void require(bool condition, const char *message) {
  if (!condition)
    throw std::runtime_error(message);
}

const ResolvedAnatomicalLandmarkCandidate3D &candidateById(
    const std::vector<ResolvedAnatomicalLandmarkCandidate3D> &candidates,
    const std::string &id) {
  const auto found = std::find_if(
      candidates.begin(), candidates.end(),
      [&id](const ResolvedAnatomicalLandmarkCandidate3D &candidate) {
        return candidate.landmarkId == id;
      });
  if (found == candidates.end())
    throw std::runtime_error("Candidato ausente: " + id);
  return *found;
}

void testFootballPlayerCandidates() {
  std::string error;
  const std::optional<LoadedPlayerModel> loaded =
      loadPlayerModel(std::string(MATTERENGINE_TEST_ASSETS_DIR) +
                          "/characters/player_model/model.json",
                      {}, &error);
  require(loaded.has_value(), "Modelo do jogador nao carregou");
  const std::optional<AnatomicalLandmarkCandidateSet3D> set =
      loadAnatomicalLandmarkCandidateSet3D(
          loaded->manifest.landmarkCandidatesPath, &error);
  require(set.has_value(), "Asset de candidatos nao carregou");
  require(set->schemaVersion == 1 && set->candidates.size() == 7,
          "Baseline do asset de candidatos mudou");
  require(validateAnatomicalLandmarkCandidateSet3D(*set).empty(),
          "Asset de candidatos nao passou na validacao");

  const RagdollCalibrationReference3D reference =
      loadRagdollCalibrationReference3D(
          std::string(MATTERENGINE_TEST_ASSETS_DIR) +
          "/characters/football_player/FootballPlayerV1.calibration.json");
  require(set->profileId == loaded->character->profile.id &&
              set->referenceId == reference.id,
          "Candidatos, perfil e referencia nao formam o mesmo conjunto");
  std::unordered_map<std::string, RagdollCalibrationLandmarkStatus3D>
      referenceStatus;
  for (const RagdollCalibrationLandmark3D &landmark : reference.landmarks)
    referenceStatus.emplace(landmark.id, landmark.status);
  std::size_t promotedSources = 0;
  for (const AnatomicalLandmarkCandidateDefinition3D &candidate :
       set->candidates) {
    const auto found = referenceStatus.find(candidate.landmarkId);
    require(found != referenceStatus.end(),
            "Candidato nao existe na referencia");
    if (found->second == RagdollCalibrationLandmarkStatus3D::Approved)
      ++promotedSources;
  }
  require(promotedSources == 3, "Baseline das fontes visuais promovidas mudou");

  const SkeletonPose rest = sampleGameRigPose(
      *loaded->character, *loaded->rig, nullptr, 0.0f, RigHandPose::Bind, true);
  const std::vector<Vec3> restMesh = loaded->model->mesh.posedPositions(
      skinningTransforms(loaded->rig->skeleton, rest));
  const std::vector<ResolvedAnatomicalLandmarkCandidate3D> resolved =
      resolveAnatomicalLandmarkCandidates3D(*set, loaded->rig->skeleton, rest,
                                            loaded->model->mesh, restMesh);
  require(resolved.size() == 7, "Resolucao perdeu candidatos");
  require(std::all_of(
              resolved.begin(), resolved.end(),
              [](const ResolvedAnatomicalLandmarkCandidate3D &candidate) {
                return candidate.resolved && candidate.supportPointCount > 0;
              }),
          "Nem todos os candidatos visuais foram resolvidos");

  const auto &vert = candidateById(resolved, "vert");
  require(vert.modelPosition.z >
              loaded->character->profile.links[5].modelPosition.z + 0.1f,
          "VERT candidato nao ficou no alto da cabeca");
  const std::vector<RagdollResolvedCalibrationLandmark3D> landmarks =
      resolveRagdollCalibrationLandmarks3D(loaded->character->profile,
                                           reference);
  const auto approvedById = [&](const std::string &id)
      -> const RagdollResolvedCalibrationLandmark3D & {
    const auto found = std::find_if(
        landmarks.begin(), landmarks.end(),
        [&id](const RagdollResolvedCalibrationLandmark3D &landmark) {
          return landmark.id == id;
        });
    if (found == landmarks.end())
      throw std::runtime_error("Landmark aprovado ausente: " + id);
    return *found;
  };
  for (const std::string &id : {std::string("vert"), std::string("left_met3"),
                                std::string("right_met3")}) {
    const auto &candidate = candidateById(resolved, id);
    const auto &approvedLandmark = approvedById(id);
    require(approvedLandmark.status ==
                    RagdollCalibrationLandmarkStatus3D::Approved &&
                approvedLandmark.resolved &&
                (candidate.modelPosition - approvedLandmark.modelPosition)
                        .length() <= 0.00025f,
            "Fonte visual promovida derivou do model_point versionado");
  }
  const auto &leftHeel = candidateById(resolved, "left_heel");
  const auto &rightHeel = candidateById(resolved, "right_heel");
  const auto &leftTip = candidateById(resolved, "left_ttip");
  const auto &rightTip = candidateById(resolved, "right_ttip");
  require(
      leftHeel.supportPointCount > 1 && rightHeel.supportPointCount > 1,
      "HEEL deveria usar uma faixa de vertices, nao um unico ruido da malha");
  require(leftTip.modelPosition.x > leftHeel.modelPosition.x &&
              rightTip.modelPosition.x > rightHeel.modelPosition.x,
          "HEEL/TTIP nao seguem o eixo posterior-anterior");
  require((leftTip.modelPosition - leftHeel.modelPosition).length() > 0.2f &&
              (leftTip.modelPosition - leftHeel.modelPosition).length() < 0.4f,
          "Comprimento candidato do pe saiu da faixa de auditoria");
  require(std::abs(leftHeel.modelPosition.x - rightHeel.modelPosition.x) <
                  0.005f &&
              std::abs(leftHeel.modelPosition.y + rightHeel.modelPosition.y) <
                  0.005f &&
              std::abs(leftHeel.modelPosition.z - rightHeel.modelPosition.z) <
                  0.005f,
          "Candidatos HEEL perderam simetria bilateral");

  SkeletonPose invalidPose;
  const std::vector<ResolvedAnatomicalLandmarkCandidate3D> invalid =
      resolveAnatomicalLandmarkCandidates3D(*set, loaded->rig->skeleton,
                                            invalidPose, loaded->model->mesh,
                                            restMesh);
  require(
      std::none_of(invalid.begin(), invalid.end(),
                   [](const ResolvedAnatomicalLandmarkCandidate3D &candidate) {
                     return candidate.resolved;
                   }),
      "Pose incompleta resolveu candidato por acidente");
}

void testCandidateValidationRejectsAmbiguity() {
  AnatomicalLandmarkCandidateSet3D set;
  set.schemaVersion = 1;
  set.id = "test";
  set.profileId = "profile";
  set.referenceId = "reference";
  set.candidates = {
      {"same",
       AnatomicalLandmarkCandidateSource3D::SkeletonJoint,
       "A",
       {},
       {},
       {},
       AnatomicalLandmarkExtreme3D::Minimum,
       0.0f,
       "source",
       {}},
      {"same",
       AnatomicalLandmarkCandidateSource3D::SkeletonJoint,
       "B",
       {},
       {},
       {},
       AnatomicalLandmarkExtreme3D::Minimum,
       0.0f,
       "source",
       {}},
  };
  require(!validateAnatomicalLandmarkCandidateSet3D(set).empty(),
          "Landmark candidato duplicado foi aceito");
}

} // namespace

int main() {
  try {
    testFootballPlayerCandidates();
    testCandidateValidationRejectsAmbiguity();
    std::cout << "Anatomical landmark candidate tests passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << "Anatomical landmark candidate tests failed: " << error.what()
              << '\n';
    return 1;
  }
}
