#include "Commands.hpp"

#include "Engine/Modeling/Inspection/AnatomicalLandmarkCandidates3D.hpp"
#include "Engine/Modeling/Inspection/ContactSheet.hpp"
#include "Engine/Modeling/Inspection/ViewFraming.hpp"
#include "Engine/RL/Calibration/RagdollCalibrationReference3D.hpp"

#include <algorithm>
#include <iomanip>
#include <iostream>
#include <set>
#include <unordered_map>

namespace MatterEngine::ModelInspector {

using namespace Modeling;

namespace {

constexpr Color ApprovedColor = rgb(76, 224, 132);

void appendCross(Highlight &highlight, Vec3 point, float radius) {
  highlight.segments.emplace_back(point - Vec3{radius, 0.0f, 0.0f},
                                  point + Vec3{radius, 0.0f, 0.0f});
  highlight.segments.emplace_back(point - Vec3{0.0f, radius, 0.0f},
                                  point + Vec3{0.0f, radius, 0.0f});
  highlight.segments.emplace_back(point - Vec3{0.0f, 0.0f, radius},
                                  point + Vec3{0.0f, 0.0f, radius});
}

struct LandmarkMarker {
  std::string id;
  Vec3 position;
  Color color;
  bool candidate = false;
};

Highlight markerHighlight(const LandmarkMarker &marker, bool label) {
  Highlight highlight;
  highlight.color = marker.color;
  highlight.points.push_back(marker.position);
  if (marker.candidate)
    appendCross(highlight, marker.position, 0.012f);
  highlight.label =
      marker.id + (marker.candidate ? " · candidato" : " · aprovado");
  highlight.labelAt = marker.position;
  highlight.hasLabel = label;
  return highlight;
}

struct LandmarkPanel {
  std::string region;
  std::string view;
  std::string caption;
  std::vector<std::string> labeledIds;
  bool labelAll = false;
};

bool contains(const std::vector<std::string> &values,
              const std::string &value) {
  return std::find(values.begin(), values.end(), value) != values.end();
}

} // namespace

int runLandmarks(const ModelContext &context, const CommandLine &line) {
  if (context.pose) {
    std::cerr
        << "landmarks: a autoria usa a pose fisica de repouso; remova --pose\n";
    return 2;
  }
  const std::filesystem::path referencePath =
      line.has("referencia")
          ? std::filesystem::path(line.text("referencia"))
          : std::filesystem::path(MATTERENGINE_ASSETS_DIR) /
                "characters/football_player/FootballPlayerV1.calibration.json";
  const std::filesystem::path candidatePath =
      line.has("candidatos") ? std::filesystem::path(line.text("candidatos"))
                             : context.loaded.manifest.landmarkCandidatesPath;
  if (candidatePath.empty()) {
    std::cerr << "landmarks: o manifesto nao declara landmarkCandidates\n";
    return 2;
  }

  RagdollProfile3D profile = context.loaded.character->profile;
  if (line.has("perfil"))
    profile = loadRagdollProfile3D(line.text("perfil"));
  const RagdollCalibrationReference3D reference =
      loadRagdollCalibrationReference3D(referencePath.string());
  std::string candidateError;
  const std::optional<AnatomicalLandmarkCandidateSet3D> candidates =
      loadAnatomicalLandmarkCandidateSet3D(candidatePath, &candidateError);
  if (!candidates) {
    std::cerr << candidateError << '\n';
    return 2;
  }
  if (profile.id != context.loaded.character->profile.id ||
      reference.profileId != profile.id ||
      candidates->profileId != profile.id ||
      candidates->referenceId != reference.id) {
    std::cerr << "landmarks: perfil, referencia, candidatos e GameRig precisam "
                 "identificar o mesmo corpo\n";
    return 2;
  }

  std::unordered_map<std::string, const RagdollCalibrationLandmark3D *>
      referenceById;
  for (const RagdollCalibrationLandmark3D &landmark : reference.landmarks)
    referenceById.emplace(landmark.id, &landmark);
  for (const AnatomicalLandmarkCandidateDefinition3D &candidate :
       candidates->candidates) {
    const auto found = referenceById.find(candidate.landmarkId);
    if (found == referenceById.end()) {
      std::cerr << "landmarks: candidato referencia id inexistente: "
                << candidate.landmarkId << '\n';
      return 2;
    }
  }

  const SkeletonPose rest =
      sampleGameRigPose(*context.loaded.character, *context.loaded.rig, nullptr,
                        0.0f, RigHandPose::Bind, true);
  const std::vector<Vec3> restMesh = context.model().mesh.posedPositions(
      skinningTransforms(context.loaded.rig->skeleton, rest));
  const std::vector<RagdollResolvedCalibrationLandmark3D> approved =
      resolveRagdollCalibrationLandmarks3D(profile, reference);
  const std::vector<ResolvedAnatomicalLandmarkCandidate3D> resolvedCandidates =
      resolveAnatomicalLandmarkCandidates3D(*candidates,
                                            context.loaded.rig->skeleton, rest,
                                            context.model().mesh, restMesh);

  std::unordered_map<std::string, const RagdollResolvedCalibrationLandmark3D *>
      resolvedReferenceById;
  for (const RagdollResolvedCalibrationLandmark3D &landmark : approved)
    resolvedReferenceById.emplace(landmark.id, &landmark);

  std::vector<LandmarkMarker> markers;
  std::set<std::string> candidateIds;
  std::size_t approvedCount = 0;
  std::size_t candidateCount = 0;
  std::size_t promotedSourceCount = 0;
  for (const RagdollResolvedCalibrationLandmark3D &landmark : approved) {
    if (landmark.status != RagdollCalibrationLandmarkStatus3D::Approved ||
        !landmark.resolved)
      continue;
    markers.push_back(
        {landmark.id, landmark.modelPosition, ApprovedColor, false});
    ++approvedCount;
  }
  std::cout << "Candidatos anatomicos `" << candidates->id << "`\n"
            << "Referencia: " << reference.id << "\n"
            << "Aprovados resolvidos: " << approvedCount << "\n"
            << "Fontes visuais:\n";
  for (const ResolvedAnatomicalLandmarkCandidate3D &candidate :
       resolvedCandidates) {
    candidateIds.insert(candidate.landmarkId);
    const RagdollCalibrationLandmark3D &referenceLandmark =
        *referenceById.at(candidate.landmarkId);
    const bool promoted = referenceLandmark.status ==
                          RagdollCalibrationLandmarkStatus3D::Approved;
    std::cout << "  " << candidate.landmarkId << " ["
              << anatomicalLandmarkCandidateSourceName3D(candidate.source)
              << "] ";
    if (!candidate.resolved) {
      std::cout << "NAO RESOLVIDO: " << candidate.reason << '\n';
      if (promoted) {
        std::cerr << "landmarks: fonte visual aprovada deixou de resolver: "
                  << candidate.landmarkId << '\n';
        return 2;
      }
      continue;
    }
    std::cout << std::fixed << std::setprecision(4) << candidate.modelPosition.x
              << ", " << candidate.modelPosition.y << ", "
              << candidate.modelPosition.z << " m"
              << " (apoio " << candidate.supportPointCount << ")";
    if (promoted) {
      const auto resolved = resolvedReferenceById.find(candidate.landmarkId);
      if (resolved == resolvedReferenceById.end() ||
          !resolved->second->resolved) {
        std::cerr << "\nlandmarks: ponto promovido nao resolve na referencia: "
                  << candidate.landmarkId << '\n';
        return 2;
      }
      const float drift =
          (candidate.modelPosition - resolved->second->modelPosition).length();
      std::cout << " · fonte aprovada, deriva " << std::setprecision(2)
                << drift * 1000.0f << " mm\n";
      if (drift > 0.00025f) {
        std::cerr << "landmarks: fonte visual aprovada derivou mais de 0,25 "
                     "mm: "
                  << candidate.landmarkId << '\n';
        return 2;
      }
      ++promotedSourceCount;
      continue;
    }
    std::cout << '\n';
    ++candidateCount;
    markers.push_back({candidate.landmarkId, candidate.modelPosition,
                       Palette::Warning, true});
  }

  std::vector<std::string> withoutCandidate;
  for (const RagdollCalibrationLandmark3D &landmark : reference.landmarks) {
    if (landmark.status == RagdollCalibrationLandmarkStatus3D::Pending &&
        !candidateIds.contains(landmark.id))
      withoutCandidate.push_back(landmark.id);
  }
  std::cout << "Pendentes sem candidato visual:";
  for (const std::string &id : withoutCandidate)
    std::cout << ' ' << id;
  std::cout << "\n\nAmarelo significa somente candidato de autoria; verde "
               "e ponto aprovado.\n";

  InspectionScene baseScene = context.scene();
  baseScene.pose = &rest;
  baseScene.meshPositions = restMesh;
  const std::vector<LandmarkPanel> panelDefinitions = {
      {"visao_geral",
       "frente",
       "corpo · verde aprovado · amarelo candidato",
       {},
       false},
      {"cabeca",
       "frente",
       "cabeca · frente · MIDG sem candidato",
       {"vert"},
       false},
      {"cabeca",
       "direita",
       "cabeca · lado · MIDG sem candidato",
       {"vert"},
       false},
      {"peito",
       "frente",
       "tronco · SUPR/XYPH/OMPH sem candidato",
       {"left_sjc", "right_sjc", "left_hjc", "right_hjc", "midh"},
       false},
      {"mao_esquerda",
       "frente",
       "mao esquerda · WJC/MET3",
       {"left_wjc", "left_met3"},
       false},
      {"mao_direita",
       "frente",
       "mao direita · WJC/MET3",
       {"right_wjc", "right_met3"},
       false},
      {"pe_esquerdo",
       "direita",
       "pe esquerdo · AJC/HEEL/TTIP",
       {"left_ajc", "left_heel", "left_ttip"},
       false},
      {"pe_direito",
       "direita",
       "pe direito · AJC/HEEL/TTIP",
       {"right_ajc", "right_heel", "right_ttip"},
       false},
  };
  const int panelWidth = std::clamp(line.integer("largura", 460), 160, 1600);
  const int panelHeight = std::clamp(line.integer("altura", 520), 160, 1600);
  std::vector<Image> images;
  std::vector<std::string> captions;
  images.reserve(panelDefinitions.size());
  captions.reserve(panelDefinitions.size());
  for (const LandmarkPanel &panel : panelDefinitions) {
    std::vector<Highlight> highlights;
    highlights.reserve(markers.size());
    for (const LandmarkMarker &marker : markers) {
      const bool selected =
          panel.labeledIds.empty() || contains(panel.labeledIds, marker.id);
      if (!selected)
        continue;
      const bool label = panel.labelAll || !panel.labeledIds.empty();
      highlights.push_back(markerHighlight(marker, label));
    }
    InspectionScene scene = baseScene;
    scene.highlights = highlights;
    CommandLine panelLine = line;
    panelLine.options["regiao"] = panel.region;
    panelLine.options["vista"] = panel.view;
    panelLine.options["modo"] = line.text("modo", "raiox");
    panelLine.options["orto"] = "1";
    panelLine.options["sem-hud"] = "1";
    panelLine.options["sem-ossos"] = "1";
    panelLine.options["largura"] = std::to_string(panelWidth);
    panelLine.options["altura"] = std::to_string(panelHeight);
    const std::optional<ViewRequest> request =
        viewFromOptions(context, panelLine, panelWidth, panelHeight);
    if (!request)
      return 2;
    InspectionStyle style = request->style;
    style.skeleton = false;
    style.joints = false;
    OrbitCamera camera = request->camera;
    const Box3 bounds = regionBounds(scene, request->region);
    frameBox(camera, bounds,
             static_cast<float>(panelWidth) / static_cast<float>(panelHeight),
             1.35f);
    images.push_back(
        renderInspection(scene, style, camera, panelWidth, panelHeight));
    captions.push_back(panel.caption);
  }

  const int columns = std::clamp(line.integer("colunas", 4), 1,
                                 static_cast<int>(images.size()));
  const std::string title =
      "Landmarks " + profile.id + " · " + std::to_string(approvedCount) +
      " aprovados · " + std::to_string(candidateCount) + " candidatos · " +
      std::to_string(promotedSourceCount) + " fontes conferidas · " +
      std::to_string(withoutCandidate.size()) + " sem candidato";
  const Image sheet = composeSheet(images, captions, columns, title);
  return saveImage(sheet, context.output(line, "landmark_candidates.png")) ? 0
                                                                           : 1;
}

} // namespace MatterEngine::ModelInspector
