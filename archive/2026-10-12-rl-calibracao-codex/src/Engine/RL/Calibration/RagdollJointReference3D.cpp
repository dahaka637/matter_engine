#include "Engine/RL/Calibration/RagdollJointReference3D.hpp"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <numbers>
#include <sstream>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>
#include <utility>

namespace MatterEngine {
namespace {

using Json = nlohmann::json;
constexpr float RadiansToDegrees = 180.0f / std::numbers::pi_v<float>;

bool finite(float value) { return std::isfinite(value); }

RagdollAxis3D readAxis(const std::string &value) {
  if (value == "twist")
    return RagdollAxis3D::Twist;
  if (value == "swing1")
    return RagdollAxis3D::Swing1;
  if (value == "swing2")
    return RagdollAxis3D::Swing2;
  throw std::invalid_argument("eixo de junta desconhecido: " + value);
}

const char *axisName(RagdollAxis3D axis) {
  switch (axis) {
  case RagdollAxis3D::Twist:
    return "twist";
  case RagdollAxis3D::Swing1:
    return "swing1";
  case RagdollAxis3D::Swing2:
    return "swing2";
  }
  return "unknown";
}

RagdollJointReviewStatus3D readStatus(const std::string &value) {
  if (value == "pending")
    return RagdollJointReviewStatus3D::Pending;
  if (value == "candidate")
    return RagdollJointReviewStatus3D::Candidate;
  if (value == "approved")
    return RagdollJointReviewStatus3D::Approved;
  throw std::invalid_argument("estado de revisao articular desconhecido: " +
                              value);
}

RagdollJointReferenceSource3D readSource(const Json &source) {
  return {source.at("id").get<std::string>(),
          source.at("title").get<std::string>(),
          source.value("doi", std::string{}),
          source.value("url", std::string{}),
          source.value("purpose", std::string{}),
          source.value("limitations", std::string{})};
}

RagdollJointMotionBenchmark3D readMotion(const Json &source) {
  RagdollJointMotionBenchmark3D result;
  result.id = source.at("id").get<std::string>();
  result.joint = source.at("joint").get<std::string>();
  result.motion = source.at("motion").get<std::string>();
  result.population = source.at("population").get<std::string>();
  result.meanDegrees = source.at("meanDegrees").get<float>();
  result.confidenceLowDegrees =
      source.at("confidence95Degrees").at(0).get<float>();
  result.confidenceHighDegrees =
      source.at("confidence95Degrees").at(1).get<float>();
  result.sourceIds = source.at("sourceIds").get<std::vector<std::string>>();
  result.notes = source.value("notes", std::string{});
  return result;
}

RagdollJointStrengthBenchmark3D readStrength(const Json &source) {
  RagdollJointStrengthBenchmark3D result;
  result.id = source.at("id").get<std::string>();
  result.joint = source.at("joint").get<std::string>();
  result.motion = source.at("motion").get<std::string>();
  result.population = source.at("population").get<std::string>();
  result.contraction = source.at("contraction").get<std::string>();
  result.angularVelocityRadiansPerSecond =
      source.at("angularVelocityRadiansPerSecond").get<float>();
  result.reportedMeanMinimumNewtonMeters =
      source.at("reportedMeanRangeNewtonMeters").at(0).get<float>();
  result.reportedMeanMaximumNewtonMeters =
      source.at("reportedMeanRangeNewtonMeters").at(1).get<float>();
  result.sourceIds = source.at("sourceIds").get<std::vector<std::string>>();
  result.notes = source.value("notes", std::string{});
  return result;
}

RagdollJointAxisReview3D readReview(const Json &source) {
  RagdollJointAxisReview3D result;
  result.id = source.at("id").get<std::string>();
  result.linkIds = source.at("linkIds").get<std::vector<std::string>>();
  result.axis = readAxis(source.at("axis").get<std::string>());
  result.status = readStatus(source.at("status").get<std::string>());
  result.anatomicalMotion = source.value("anatomicalMotion", std::string{});
  result.negativeMotionBenchmarkId =
      source.value("negativeMotionBenchmarkId", std::string{});
  result.positiveMotionBenchmarkId =
      source.value("positiveMotionBenchmarkId", std::string{});
  result.strengthBenchmarkIds =
      source.value("strengthBenchmarkIds", std::vector<std::string>{});
  result.sourceIds = source.value("sourceIds", std::vector<std::string>{});
  result.notes = source.value("notes", std::string{});
  return result;
}

void addIssue(std::vector<RagdollJointReferenceValidationIssue3D> &issues,
              std::string path, std::string message) {
  issues.push_back({std::move(path), std::move(message)});
}

bool contains(const std::unordered_set<std::string> &values,
              const std::string &value) {
  return value.empty() || values.contains(value);
}

std::string fixed(float value, int precision = 1) {
  std::ostringstream output;
  output << std::fixed << std::setprecision(precision) << value;
  return output.str();
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

} // namespace

const char *ragdollJointReviewStatusName3D(RagdollJointReviewStatus3D status) {
  switch (status) {
  case RagdollJointReviewStatus3D::Pending:
    return "pending";
  case RagdollJointReviewStatus3D::Candidate:
    return "candidate";
  case RagdollJointReviewStatus3D::Approved:
    return "approved";
  }
  return "unknown";
}

std::size_t RagdollJointReferenceComparisonResult3D::findingCount(
    CalibrationFindingSeverity3D severity) const {
  return static_cast<std::size_t>(
      std::count_if(findings.begin(), findings.end(),
                    [severity](const CalibrationFinding3D &finding) {
                      return finding.severity == severity;
                    }));
}

bool RagdollJointReferenceComparisonResult3D::hasErrors() const {
  return findingCount(CalibrationFindingSeverity3D::Error) > 0;
}

RagdollJointReference3D loadRagdollJointReference3D(std::string_view filePath) {
  std::ifstream input{std::string(filePath)};
  if (!input)
    throw std::runtime_error("Nao foi possivel abrir referencia articular: " +
                             std::string(filePath));
  Json root;
  try {
    input >> root;
    RagdollJointReference3D result;
    result.schemaVersion = root.at("schemaVersion").get<int>();
    result.id = root.at("id").get<std::string>();
    result.profileId = root.at("profileId").get<std::string>();
    result.status = root.at("status").get<std::string>();
    result.notes = root.value("notes", std::vector<std::string>{});
    for (const Json &source : root.at("sources"))
      result.sources.push_back(readSource(source));
    for (const Json &motion : root.at("motionBenchmarks"))
      result.motionBenchmarks.push_back(readMotion(motion));
    for (const Json &strength : root.at("strengthBenchmarks"))
      result.strengthBenchmarks.push_back(readStrength(strength));
    for (const Json &review : root.at("axisReviews"))
      result.axisReviews.push_back(readReview(review));
    validateRagdollJointReferenceOrThrow3D(result);
    return result;
  } catch (const Json::exception &error) {
    throw std::runtime_error("JSON articular invalido em " +
                             std::string(filePath) + ": " + error.what());
  }
}

std::vector<RagdollJointReferenceValidationIssue3D>
validateRagdollJointReference3D(const RagdollJointReference3D &reference) {
  std::vector<RagdollJointReferenceValidationIssue3D> issues;
  if (reference.schemaVersion != 1)
    addIssue(issues, "schemaVersion", "esperado schema 1");
  if (reference.id.empty() || reference.profileId.empty() ||
      reference.status.empty())
    addIssue(issues, "metadata", "id, profileId e status sao obrigatorios");

  std::unordered_set<std::string> sourceIds;
  for (std::size_t index = 0; index < reference.sources.size(); ++index) {
    const auto &source = reference.sources[index];
    if (source.id.empty() || source.title.empty() || source.purpose.empty() ||
        source.limitations.empty())
      addIssue(issues, "sources[" + std::to_string(index) + "]",
               "fonte exige id, titulo, finalidade e limitacoes");
    if (!sourceIds.insert(source.id).second)
      addIssue(issues, "sources[" + std::to_string(index) + "].id",
               "id duplicado");
  }

  std::unordered_set<std::string> motionIds;
  for (std::size_t index = 0; index < reference.motionBenchmarks.size();
       ++index) {
    const auto &motion = reference.motionBenchmarks[index];
    const std::string path = "motionBenchmarks[" + std::to_string(index) + "]";
    if (motion.id.empty() || motion.joint.empty() || motion.motion.empty() ||
        motion.population.empty() || motion.sourceIds.empty())
      addIssue(issues, path, "benchmark de ROM incompleto");
    if (!motionIds.insert(motion.id).second)
      addIssue(issues, path + ".id", "id duplicado");
    if (!finite(motion.meanDegrees) || !finite(motion.confidenceLowDegrees) ||
        !finite(motion.confidenceHighDegrees) ||
        motion.confidenceLowDegrees > motion.meanDegrees ||
        motion.meanDegrees > motion.confidenceHighDegrees)
      addIssue(issues, path, "media/intervalo de ROM invalido");
    for (const std::string &sourceId : motion.sourceIds)
      if (!sourceIds.contains(sourceId))
        addIssue(issues, path + ".sourceIds", "fonte inexistente");
  }

  std::unordered_set<std::string> strengthIds;
  for (std::size_t index = 0; index < reference.strengthBenchmarks.size();
       ++index) {
    const auto &strength = reference.strengthBenchmarks[index];
    const std::string path =
        "strengthBenchmarks[" + std::to_string(index) + "]";
    if (strength.id.empty() || strength.joint.empty() ||
        strength.motion.empty() || strength.population.empty() ||
        strength.contraction.empty() || strength.sourceIds.empty())
      addIssue(issues, path, "benchmark de forca incompleto");
    if (!strengthIds.insert(strength.id).second)
      addIssue(issues, path + ".id", "id duplicado");
    if (!finite(strength.angularVelocityRadiansPerSecond) ||
        strength.angularVelocityRadiansPerSecond <= 0.0f ||
        !finite(strength.reportedMeanMinimumNewtonMeters) ||
        !finite(strength.reportedMeanMaximumNewtonMeters) ||
        strength.reportedMeanMinimumNewtonMeters <= 0.0f ||
        strength.reportedMeanMinimumNewtonMeters >
            strength.reportedMeanMaximumNewtonMeters)
      addIssue(issues, path, "velocidade/faixa de torque invalida");
    for (const std::string &sourceId : strength.sourceIds)
      if (!sourceIds.contains(sourceId))
        addIssue(issues, path + ".sourceIds", "fonte inexistente");
  }

  std::unordered_set<std::string> reviewIds;
  std::unordered_set<std::string> coveredAxes;
  for (std::size_t index = 0; index < reference.axisReviews.size(); ++index) {
    const auto &review = reference.axisReviews[index];
    const std::string path = "axisReviews[" + std::to_string(index) + "]";
    if (review.id.empty() || review.linkIds.empty())
      addIssue(issues, path, "revisao exige id e links");
    if (!reviewIds.insert(review.id).second)
      addIssue(issues, path + ".id", "id duplicado");
    for (const std::string &linkId : review.linkIds) {
      const std::string key = linkId + "." + axisName(review.axis);
      if (linkId.empty() || !coveredAxes.insert(key).second)
        addIssue(issues, path + ".linkIds", "link/eixo vazio ou duplicado");
    }
    if (!contains(motionIds, review.negativeMotionBenchmarkId) ||
        !contains(motionIds, review.positiveMotionBenchmarkId))
      addIssue(issues, path, "benchmark de ROM inexistente");
    for (const std::string &strengthId : review.strengthBenchmarkIds)
      if (!strengthIds.contains(strengthId))
        addIssue(issues, path + ".strengthBenchmarkIds",
                 "benchmark de forca inexistente");
    for (const std::string &sourceId : review.sourceIds)
      if (!sourceIds.contains(sourceId))
        addIssue(issues, path + ".sourceIds", "fonte inexistente");
    if (review.status == RagdollJointReviewStatus3D::Approved &&
        (review.anatomicalMotion.empty() || review.sourceIds.empty()))
      addIssue(issues, path,
               "revisao aprovada exige movimento anatomico e fontes");
  }
  return issues;
}

void validateRagdollJointReferenceOrThrow3D(
    const RagdollJointReference3D &reference) {
  const auto issues = validateRagdollJointReference3D(reference);
  if (issues.empty())
    return;
  throw std::invalid_argument("Referencia articular invalida em " +
                              issues.front().path + ": " +
                              issues.front().message);
}

RagdollJointReferenceComparisonResult3D
compareRagdollJointReference3D(const RagdollProfile3D &profile,
                               const RagdollJointReference3D &reference) {
  validateRagdollProfileOrThrow3D(profile);
  validateRagdollJointReferenceOrThrow3D(reference);
  RagdollJointReferenceComparisonResult3D result;
  result.referenceId = reference.id;
  result.referenceStatus = reference.status;
  result.profileId = profile.id;
  result.sources = reference.sources;
  result.motionBenchmarks = reference.motionBenchmarks;
  result.strengthBenchmarks = reference.strengthBenchmarks;
  if (reference.profileId != profile.id) {
    result.findings.push_back(
        {CalibrationFindingSeverity3D::Error, "joint_profile_mismatch",
         "profileId", "a referencia articular pertence a outro perfil"});
    return result;
  }

  std::unordered_map<std::string, const RagdollJointAxisReview3D *> reviews;
  for (const auto &review : reference.axisReviews)
    for (const std::string &linkId : review.linkIds)
      reviews.emplace(linkId + "." + axisName(review.axis), &review);
  std::unordered_map<std::string, const RagdollJointMotionBenchmark3D *>
      motionBenchmarks;
  for (const RagdollJointMotionBenchmark3D &benchmark :
       reference.motionBenchmarks)
    motionBenchmarks.emplace(benchmark.id, &benchmark);

  std::size_t pending = 0, candidates = 0, approved = 0;
  for (std::size_t linkIndex = 1; linkIndex < profile.links.size();
       ++linkIndex) {
    const auto &link = profile.links[linkIndex];
    for (std::size_t axisIndex = 0; axisIndex < link.inboundJoint.axes.size();
         ++axisIndex) {
      const auto &axis = link.inboundJoint.axes[axisIndex];
      if (!axis.enabled)
        continue;
      const RagdollAxis3D axisId = static_cast<RagdollAxis3D>(axisIndex);
      const std::string key = link.id + "." + axisName(axisId);
      const auto found = reviews.find(key);
      if (found == reviews.end()) {
        result.findings.push_back(
            {CalibrationFindingSeverity3D::Error, "joint_axis_unmapped", key,
             "o DOF ativo nao possui registro de revisao"});
        continue;
      }
      const RagdollJointAxisReview3D &review = *found->second;
      result.axes.push_back(
          {review.id, link.id, axisId, review.status, review.anatomicalMotion,
           axis.minimumRadians * RadiansToDegrees,
           axis.maximumRadians * RadiansToDegrees, axis.maximumTorque,
           review.negativeMotionBenchmarkId, review.positiveMotionBenchmarkId,
           review.strengthBenchmarkIds});
      if (review.status == RagdollJointReviewStatus3D::Approved) {
        const auto appendDirectional = [&](const std::string &benchmarkId,
                                           float currentDegrees) {
          if (benchmarkId.empty())
            return;
          const auto benchmark = motionBenchmarks.find(benchmarkId);
          if (benchmark == motionBenchmarks.end())
            return;
          const RagdollJointMotionBenchmark3D &published = *benchmark->second;
          const float magnitude = std::abs(currentDegrees);
          result.directionalComparisons.push_back(
              {review.id, link.id, axisId, published.motion, published.id,
               magnitude, published.meanDegrees, published.confidenceLowDegrees,
               published.confidenceHighDegrees,
               magnitude - published.meanDegrees});
        };
        appendDirectional(review.negativeMotionBenchmarkId,
                          axis.minimumRadians * RadiansToDegrees);
        appendDirectional(review.positiveMotionBenchmarkId,
                          axis.maximumRadians * RadiansToDegrees);
      }
      switch (review.status) {
      case RagdollJointReviewStatus3D::Pending:
        ++pending;
        break;
      case RagdollJointReviewStatus3D::Candidate:
        ++candidates;
        break;
      case RagdollJointReviewStatus3D::Approved:
        ++approved;
        break;
      }
    }
  }
  if (result.axes.size() != ragdollDegreesOfFreedom3D(profile) ||
      reviews.size() != ragdollDegreesOfFreedom3D(profile))
    result.findings.push_back(
        {CalibrationFindingSeverity3D::Error, "joint_axis_coverage",
         "axisReviews", "a referencia nao cobre exatamente todos os DOFs"});
  result.findings.push_back({CalibrationFindingSeverity3D::Information,
                             "joint_review_status", "axisReviews",
                             std::to_string(approved) + " aprovados, " +
                                 std::to_string(candidates) + " candidatos e " +
                                 std::to_string(pending) + " pendentes"});
  result.findings.push_back(
      {CalibrationFindingSeverity3D::Information,
       "joint_benchmarks_do_not_mutate_profile", "benchmarks",
       "ROM clinico e torque isocinetico permanecem benchmarks; nenhum "
       "valor altera limites ou motores automaticamente"});
  return result;
}

std::string renderRagdollJointReferenceMarkdown3D(
    const RagdollJointReferenceComparisonResult3D &comparison) {
  std::ostringstream output;
  output << "# Revisao de ROM e torque `" << comparison.profileId << "`\n\n"
         << "> Referencia `" << comparison.referenceId << "` (`"
         << comparison.referenceStatus
         << "`). Benchmarks clinicos/isocineticos nao sao limites do motor e "
            "nao alteram o ragdoll.\n\n"
         << "## Fontes e limites de uso\n\n"
         << "| Fonte | Finalidade | Limites |\n|---|---|---|\n";
  for (const auto &source : comparison.sources)
    output << "| " << source.title
           << (source.doi.empty() ? "" : " (" + source.doi + ")") << " | "
           << source.purpose << " | " << source.limitations << " |\n";

  output << "\n## Benchmarks de ROM\n\n"
         << "| Id | Junta / movimento | Populacao | Media (IC 95%) |\n"
         << "|---|---|---|---:|\n";
  for (const auto &motion : comparison.motionBenchmarks)
    output << "| `" << motion.id << "` | " << motion.joint << " / "
           << motion.motion << " | " << motion.population << " | "
           << fixed(motion.meanDegrees) << " deg ("
           << fixed(motion.confidenceLowDegrees) << "-"
           << fixed(motion.confidenceHighDegrees) << ") |\n";

  output << "\n## Benchmarks de forca\n\n"
         << "| Id | Junta / movimento | Protocolo | Faixa de medias |\n"
         << "|---|---|---|---:|\n";
  for (const auto &strength : comparison.strengthBenchmarks)
    output << "| `" << strength.id << "` | " << strength.joint << " / "
           << strength.motion << " | " << strength.contraction << " a "
           << fixed(strength.angularVelocityRadiansPerSecond, 2) << " rad/s | "
           << fixed(strength.reportedMeanMinimumNewtonMeters) << "-"
           << fixed(strength.reportedMeanMaximumNewtonMeters) << " N.m |\n";

  output << "\n## Cobertura dos eixos atuais\n\n"
         << "| Link | Eixo | Estado | Movimento | Atual min/max | Torque atual "
            "| Benchmarks ligados |\n"
         << "|---|---|---|---|---:|---:|---|\n";
  for (const auto &axis : comparison.axes) {
    std::string benchmarks = axis.negativeMotionBenchmarkId;
    if (!axis.positiveMotionBenchmarkId.empty())
      benchmarks +=
          (benchmarks.empty() ? "" : ", ") + axis.positiveMotionBenchmarkId;
    for (const std::string &strength : axis.strengthBenchmarkIds)
      benchmarks += (benchmarks.empty() ? "" : ", ") + strength;
    output << "| " << axis.linkId << " | " << axisName(axis.axis) << " | `"
           << ragdollJointReviewStatusName3D(axis.status) << "` | "
           << (axis.anatomicalMotion.empty() ? "-" : axis.anatomicalMotion)
           << " | " << fixed(axis.currentMinimumDegrees) << "/"
           << fixed(axis.currentMaximumDegrees) << " deg | "
           << fixed(axis.currentMaximumTorqueNewtonMeters) << " N.m | "
           << (benchmarks.empty() ? "-" : benchmarks) << " |\n";
  }

  output << "\n## Comparacao direcional dos mapeamentos aprovados\n\n"
         << "> Comparacao descritiva. O IC 95% abaixo e da media "
            "populacional publicada; nao e faixa individual, clamp seguro "
            "ou autorizacao para alterar o ragdoll.\n\n"
         << "| Link / eixo | Movimento | Atual | Media publicada (IC 95%) | "
            "Diferenca |\n"
         << "|---|---|---:|---:|---:|\n";
  for (const RagdollJointDirectionalComparison3D &direction :
       comparison.directionalComparisons) {
    output << "| " << direction.linkId << "." << axisName(direction.axis)
           << " | " << direction.direction << " | "
           << fixed(direction.currentMagnitudeDegrees) << " deg | "
           << fixed(direction.publishedMeanDegrees) << " deg ("
           << fixed(direction.publishedConfidenceLowDegrees) << "-"
           << fixed(direction.publishedConfidenceHighDegrees) << ") | "
           << (direction.differenceFromMeanDegrees >= 0.0f ? "+" : "")
           << fixed(direction.differenceFromMeanDegrees) << " deg |\n";
  }

  output << "\n## Achados\n\n"
         << "| Severidade | Codigo | Caminho | Mensagem |\n"
         << "|---|---|---|---|\n";
  for (const CalibrationFinding3D &finding : comparison.findings)
    output << "| " << severityName(finding.severity) << " | `" << finding.code
           << "` | `" << finding.path << "` | " << finding.message << " |\n";
  return output.str();
}

} // namespace MatterEngine
