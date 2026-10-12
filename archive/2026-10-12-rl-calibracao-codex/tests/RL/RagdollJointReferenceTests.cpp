#include "Engine/RL/Calibration/RagdollJointReference3D.hpp"

#include <algorithm>
#include <cmath>
#include <exception>
#include <iostream>
#include <stdexcept>
#include <string>
#include <string_view>

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

RagdollJointReference3D loadFootballJointReference() {
  return loadRagdollJointReference3D(
      std::string(MATTERENGINE_TEST_ASSETS_DIR) +
      "/characters/football_player/FootballPlayerV1.joints.json");
}

void testVersionedReferenceCoversEveryDegreeOfFreedom() {
  const RagdollProfile3D profile = loadFootballPlayer();
  const RagdollJointReference3D reference = loadFootballJointReference();
  require(reference.schemaVersion == 1,
          "Versao inesperada da referencia articular");
  require(reference.status == "draft_reference",
          "Referencia articular deixou de ser rascunho");
  require(reference.sources.size() == 5, "Proveniencia articular incompleta");
  require(reference.motionBenchmarks.size() == 11,
          "Benchmarks de ROM incompletos");
  require(reference.strengthBenchmarks.size() == 2,
          "Benchmarks de forca incompletos");
  require(validateRagdollJointReference3D(reference).empty(),
          "Referencia articular versionada e invalida");

  const RagdollJointReferenceComparisonResult3D comparison =
      compareRagdollJointReference3D(profile, reference);
  require(!comparison.hasErrors(),
          "Referencia articular nao cobre o perfil atual");
  require(comparison.axes.size() == 39, "Nem todos os DOFs foram cobertos");
  require(std::count_if(comparison.axes.begin(), comparison.axes.end(),
                        [](const RagdollJointAxisComparison3D &axis) {
                          return axis.status ==
                                 RagdollJointReviewStatus3D::Candidate;
                        }) == 2,
          "Baseline de mapeamentos candidatos mudou");
  require(std::count_if(comparison.axes.begin(), comparison.axes.end(),
                        [](const RagdollJointAxisComparison3D &axis) {
                          return axis.status ==
                                 RagdollJointReviewStatus3D::Approved;
                        }) == 4,
          "Baseline de mapeamentos aprovados mudou");
  require(comparison.directionalComparisons.size() == 8,
          "Comparacao direcional nao cobre os quatro eixos aprovados");
  require(std::all_of(comparison.directionalComparisons.begin(),
                      comparison.directionalComparisons.end(),
                      [](const RagdollJointDirectionalComparison3D &item) {
                        return std::isfinite(item.differenceFromMeanDegrees);
                      }),
          "Comparacao direcional publicou diferenca nao finita");
  const auto statusFor = [&](std::string_view linkId, RagdollAxis3D axisId) {
    const auto axis = std::find_if(
        comparison.axes.begin(), comparison.axes.end(),
        [linkId, axisId](const RagdollJointAxisComparison3D &item) {
          return item.linkId == linkId && item.axis == axisId;
        });
    require(axis != comparison.axes.end(), "Eixo revisado ausente");
    return axis->status;
  };
  require(statusFor("LeftForearm", RagdollAxis3D::Twist) ==
                  RagdollJointReviewStatus3D::Approved &&
              statusFor("RightForearm", RagdollAxis3D::Twist) ==
                  RagdollJointReviewStatus3D::Approved &&
              statusFor("LeftShin", RagdollAxis3D::Twist) ==
                  RagdollJointReviewStatus3D::Approved &&
              statusFor("RightShin", RagdollAxis3D::Twist) ==
                  RagdollJointReviewStatus3D::Approved,
          "Cotovelos/joelhos perderam a aprovacao de mapeamento");
  require(statusFor("LeftThigh", RagdollAxis3D::Swing1) ==
                  RagdollJointReviewStatus3D::Candidate &&
              statusFor("RightThigh", RagdollAxis3D::Swing1) ==
                  RagdollJointReviewStatus3D::Candidate,
          "Quadris foram promovidos antes do teste combinado");

  const auto knee = std::find_if(comparison.axes.begin(), comparison.axes.end(),
                                 [](const RagdollJointAxisComparison3D &axis) {
                                   return axis.linkId == "LeftShin";
                                 });
  require(knee != comparison.axes.end() &&
              knee->currentMaximumTorqueNewtonMeters == 750.0f &&
              knee->strengthBenchmarkIds.size() == 2,
          "Revisao do joelho perdeu o teto atual ou os benchmarks");
}

void testInvalidReferenceAndIncompleteCoverageAreRejected() {
  RagdollJointReference3D reference = loadFootballJointReference();
  reference.motionBenchmarks.front().sourceIds = {"missing"};
  require(!validateRagdollJointReference3D(reference).empty(),
          "Fonte inexistente foi aceita");

  reference = loadFootballJointReference();
  reference.axisReviews.pop_back();
  const RagdollJointReferenceComparisonResult3D comparison =
      compareRagdollJointReference3D(loadFootballPlayer(), reference);
  require(comparison.hasErrors(), "Cobertura incompleta de DOFs foi aceita");

  reference = loadFootballJointReference();
  reference.axisReviews.front().status = RagdollJointReviewStatus3D::Approved;
  reference.axisReviews.front().anatomicalMotion.clear();
  require(!validateRagdollJointReference3D(reference).empty(),
          "Aprovacao sem movimento anatomico foi aceita");
}

void testMarkdownKeepsBenchmarksSeparateFromMotorLimits() {
  const RagdollJointReferenceComparisonResult3D comparison =
      compareRagdollJointReference3D(loadFootballPlayer(),
                                     loadFootballJointReference());
  const std::string markdown =
      renderRagdollJointReferenceMarkdown3D(comparison);
  require(markdown.find("Benchmarks de ROM") != std::string::npos &&
              markdown.find("Benchmarks de forca") != std::string::npos,
          "Relatorio omitiu benchmarks articulares");
  require(markdown.find("nao alteram o ragdoll") != std::string::npos,
          "Relatorio omitiu a fronteira de seguranca");
  require(markdown.find("Comparacao direcional dos mapeamentos aprovados") !=
                  std::string::npos &&
              markdown.find("nao e faixa individual") != std::string::npos,
          "Relatorio omitiu os limites da comparacao direcional");
  require(markdown.find("soccer_knee_extension_concentric_60") !=
                  std::string::npos &&
              markdown.find("750.0 N.m") != std::string::npos,
          "Relatorio nao separa benchmark e teto atual");
}

} // namespace

int main() {
  try {
    testVersionedReferenceCoversEveryDegreeOfFreedom();
    testInvalidReferenceAndIncompleteCoverageAreRejected();
    testMarkdownKeepsBenchmarksSeparateFromMotorLimits();
    std::cout << "Ragdoll joint reference tests passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << "Ragdoll joint reference tests failed: " << error.what()
              << '\n';
    return 1;
  }
}
