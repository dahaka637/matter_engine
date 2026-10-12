#include "Engine/RL/Calibration/RagdollCalibrationReference3D.hpp"
#include "Engine/RL/Calibration/RagdollDynamicLab3D.hpp"

#include <algorithm>
#include <cmath>
#include <exception>
#include <iostream>
#include <stdexcept>
#include <string>

namespace {

using namespace MatterEngine;

void require(bool condition, const char *message) {
  if (!condition)
    throw std::runtime_error(message);
}

RagdollProfile3D loadFootballPlayer() {
  return loadRagdollProfile3D(
      std::string(MATTERENGINE_TEST_ASSETS_DIR) +
      "/characters/football_player/FootballPlayerV1.ragdoll.json");
}

RagdollCalibrationReference3D loadFootballReference() {
  return loadRagdollCalibrationReference3D(
      std::string(MATTERENGINE_TEST_ASSETS_DIR) +
      "/characters/football_player/FootballPlayerV1.calibration.json");
}

const RagdollMassReferenceComparison3D &
groupById(const RagdollCalibrationReferenceComparisonResult3D &comparison,
          const std::string &id) {
  for (const RagdollMassReferenceComparison3D &group : comparison.massGroups) {
    if (group.groupId == id)
      return group;
  }
  throw std::runtime_error("Grupo ausente: " + id);
}

const RagdollInertialReferenceComparison3D &inertialGroupById(
    const RagdollCalibrationReferenceComparisonResult3D &comparison,
    const std::string &id) {
  for (const RagdollInertialReferenceComparison3D &group :
       comparison.inertialGroups) {
    if (group.groupId == id)
      return group;
  }
  throw std::runtime_error("Grupo inercial ausente: " + id);
}

std::size_t findingCountByCode(
    const RagdollCalibrationReferenceComparisonResult3D &comparison,
    const std::string &code) {
  return static_cast<std::size_t>(
      std::count_if(comparison.findings.begin(), comparison.findings.end(),
                    [&code](const CalibrationFinding3D &finding) {
                      return finding.code == code;
                    }));
}

void testVersionedReferenceAndCurrentComparison() {
  const RagdollProfile3D profile = loadFootballPlayer();
  const RagdollCalibrationReference3D reference = loadFootballReference();
  require(reference.schemaVersion == 3, "Versao inesperada da referencia");
  require(reference.status == "draft_reference",
          "A referencia deixou de ser rascunho");
  require(validateRagdollCalibrationReference3D(reference).empty(),
          "A referencia versionada e invalida");
  require(reference.landmarks.size() == 24,
          "Registro anatomico perdeu landmarks");
  const std::vector<RagdollResolvedCalibrationLandmark3D> landmarks =
      resolveRagdollCalibrationLandmarks3D(profile, reference);
  require(
      std::count_if(landmarks.begin(), landmarks.end(),
                    [](const RagdollResolvedCalibrationLandmark3D &landmark) {
                      return landmark.resolved &&
                             landmark.status ==
                                 RagdollCalibrationLandmarkStatus3D::Approved;
                    }) == 16,
      "Landmarks aprovados nao foram todos resolvidos");

  const RagdollEffectiveMass3D effectiveMass =
      measureRagdollEffectiveMass3D(profile);
  require(effectiveMass.valid,
          "Backend nao publicou propriedades de massa para o teste");
  const RagdollCalibrationReferenceComparisonResult3D comparison =
      compareRagdollCalibrationReference3D(profile, reference, &effectiveMass);
  require(!comparison.hasErrors(),
          "A comparacao antropometrica encontrou erro de contrato");
  require(comparison.massGroups.size() == 16,
          "Mapeamento nao cobre os 16 grupos esperados");
  require(std::abs(comparison.targetMassFractionSum - 1.0f) < 0.00001f,
          "As fracoes antropometricas nao somam 1");
  require(std::abs(comparison.actualMassFractionSum - 1.0f) < 0.00001f,
          "O mapeamento nao consumiu toda a massa atual");
  require(findingCountByCode(comparison, "mass_group_outside_review_band") == 6,
          "O baseline das diferencas de massa mudou");
  require(comparison.inertialGroups.size() == 16,
          "Referencia nao cobre os 16 grupos inerciais");
  require(std::count_if(comparison.inertialGroups.begin(),
                        comparison.inertialGroups.end(),
                        [](const RagdollInertialReferenceComparison3D &group) {
                          return group.centerOfMassComparable;
                        }) == 10,
          "Os dez segmentos com landmarks aprovados deveriam comparar COM");
  require(std::count_if(comparison.inertialGroups.begin(),
                        comparison.inertialGroups.end(),
                        [](const RagdollInertialReferenceComparison3D &group) {
                          return group.inertiaComparable;
                        }) == 10,
          "Os dez segmentos diretos deveriam comparar inercia longitudinal");
  require(
      findingCountByCode(comparison, "center_of_mass_outside_review_band") == 6,
      "O baseline dos desvios de COM mudou");
  require(findingCountByCode(comparison,
                             "longitudinal_inertia_outside_review_band") == 0,
          "A inercia geometrica longitudinal saiu da banda de referencia");

  const RagdollMassReferenceComparison3D &head =
      groupById(comparison, "head_neck");
  require(std::abs(head.targetMassFraction - 0.0694f) < 0.00001f,
          "Alvo de cabeca/pescoco mudou");
  require(std::abs(head.actualMassFraction - 0.101f) < 0.00001f,
          "Massa atual de cabeca/pescoco mudou");
  require(!head.withinReviewTolerance,
          "Diferenca conhecida de cabeca/pescoco deixou de ser sinalizada");

  const RagdollMassReferenceComparison3D &upperArm =
      groupById(comparison, "left_upper_arm");
  require(upperArm.withinReviewTolerance,
          "Braco superior atual deveria permanecer dentro da banda de triagem");

  const RagdollInertialReferenceComparison3D &leftShank =
      inertialGroupById(comparison, "left_shank");
  require(leftShank.mapping ==
              RagdollInertialLandmarkMapping3D::LandmarkRegistry,
          "Perna deveria usar o registro de landmarks");
  require(std::abs(leftShank.targetCenterOfMassFromOriginFraction - 0.4395f) <
              0.00001f,
          "Alvo KJC-AJC alternativo da perna mudou");
  require(std::abs(leftShank.targetRadiusOfGyration.longitudinalFraction -
                   0.102f) < 0.00001f,
          "Raio de giracao longitudinal KJC-AJC mudou");
  require(leftShank.actualCenterOfMassFromOriginFraction > 0.48f &&
              leftShank.actualCenterOfMassFromOriginFraction < 0.50f,
          "Baseline longitudinal do COM da perna mudou");
  require(leftShank.unconditionedLongitudinalRadiusFraction > 0.09f &&
              leftShank.unconditionedLongitudinalRadiusFraction < 0.11f,
          "Baseline da inercia geometrica da perna mudou");
  require(leftShank.effectiveLongitudinalRadiusFraction >
              leftShank.unconditionedLongitudinalRadiusFraction * 1.8f,
          "Condicionamento efetivo da perna deixou de ser exposto");

  const RagdollInertialReferenceComparison3D &leftHand =
      inertialGroupById(comparison, "left_hand");
  require(leftHand.mapping ==
                  RagdollInertialLandmarkMapping3D::LandmarkRegistry &&
              leftHand.centerOfMassComparable && leftHand.inertiaComparable &&
              leftHand.actualCenterOfMassFromOriginFraction > 0.89f,
          "MET3 aprovado nao habilitou a auditoria da mao esquerda");

  const RagdollInertialReferenceComparison3D &headInertial =
      inertialGroupById(comparison, "head_neck");
  require(headInertial.mapping ==
                  RagdollInertialLandmarkMapping3D::PendingLandmarks &&
              !headInertial.centerOfMassComparable &&
              !headInertial.inertiaComparable,
          "Cabeca/pescoco nao pode ganhar landmarks por aproximacao");
}

void testReferenceRejectsInvalidLandmarkGraph() {
  RagdollCalibrationReference3D reference = loadFootballReference();
  const auto pending = std::find_if(
      reference.landmarks.begin(), reference.landmarks.end(),
      [](const RagdollCalibrationLandmark3D &landmark) {
        return landmark.kind == RagdollCalibrationLandmarkKind3D::Pending;
      });
  require(pending != reference.landmarks.end(),
          "Referencia deixou de ter landmark pendente para o teste");
  pending->status = RagdollCalibrationLandmarkStatus3D::Approved;
  require(!validateRagdollCalibrationReference3D(reference).empty(),
          "Landmark pending aprovado sem localizacao foi aceito");

  reference = loadFootballReference();
  RagdollCalibrationLandmark3D &midpoint = reference.landmarks[19];
  require(midpoint.id == "midh", "Baseline do landmark MIDH mudou");
  midpoint.midpointFirstLandmarkId = midpoint.id;
  require(!validateRagdollCalibrationReference3D(reference).empty(),
          "Ciclo entre landmarks derivados foi aceito");
}

void testReferenceRejectsAmbiguousLinkMapping() {
  RagdollCalibrationReference3D reference = loadFootballReference();
  reference.massGroups[1].linkIds.push_back("Neck");
  require(!validateRagdollCalibrationReference3D(reference).empty(),
          "Link duplicado entre grupos nao foi rejeitado");

  const RagdollCalibrationReferenceComparisonResult3D comparison =
      compareRagdollCalibrationReference3D(loadFootballPlayer(), reference);
  require(comparison.hasErrors(),
          "Comparacao aceitou referencia estruturalmente invalida");
}

void testReferenceRejectsIncompleteInertialCoverage() {
  RagdollCalibrationReference3D reference = loadFootballReference();
  reference.inertialGroups.pop_back();
  require(!validateRagdollCalibrationReference3D(reference).empty(),
          "Cobertura inercial incompleta nao foi rejeitada");

  reference = loadFootballReference();
  reference.inertialGroups.front().targetRadiusOfGyration.longitudinalFraction =
      0.0f;
  require(!validateRagdollCalibrationReference3D(reference).empty(),
          "Raio de giracao invalido nao foi rejeitado");
}

void testMarkdownStatesSafetyBoundary() {
  const RagdollProfile3D profile = loadFootballPlayer();
  const RagdollEffectiveMass3D effectiveMass =
      measureRagdollEffectiveMass3D(profile);
  const RagdollCalibrationReferenceComparisonResult3D comparison =
      compareRagdollCalibrationReference3D(profile, loadFootballReference(),
                                           &effectiveMass);
  const std::string markdown =
      renderRagdollCalibrationReferenceMarkdown3D(comparison);
  require(markdown.find("draft_reference") != std::string::npos,
          "Relatorio omitiu o estado da referencia");
  require(markdown.find("Nao altera o ragdoll") != std::string::npos,
          "Relatorio omitiu a fronteira de seguranca");
  require(markdown.find("head_neck") != std::string::npos,
          "Relatorio omitiu os grupos de massa");
  require(markdown.find("Centro de massa e inercia") != std::string::npos,
          "Relatorio omitiu a auditoria inercial");
  require(markdown.find("Landmarks anatomicos") != std::string::npos &&
              markdown.find("profile_joint_center") != std::string::npos &&
              markdown.find("midh") != std::string::npos,
          "Relatorio omitiu o registro anatomico versionado");
  require(markdown.find("pending_landmarks") != std::string::npos,
          "Relatorio omitiu marcos anatomicos pendentes");
  require(markdown.find("shape / solver") != std::string::npos,
          "Relatorio nao separa inercia geometrica e efetiva");
  require(markdown.find("10.1016/0021-9290(95)00178-6") != std::string::npos,
          "Relatorio omitiu a proveniencia");
}

} // namespace

int main() {
  try {
    testVersionedReferenceAndCurrentComparison();
    testReferenceRejectsAmbiguousLinkMapping();
    testReferenceRejectsIncompleteInertialCoverage();
    testReferenceRejectsInvalidLandmarkGraph();
    testMarkdownStatesSafetyBoundary();
    std::cout << "Ragdoll calibration reference tests passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << "Ragdoll calibration reference tests failed: " << error.what()
              << '\n';
    return 1;
  }
}
