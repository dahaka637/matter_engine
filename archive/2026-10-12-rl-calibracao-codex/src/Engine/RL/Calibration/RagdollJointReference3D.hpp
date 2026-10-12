#pragma once

#include "Engine/Physics/RagdollProfile3D.hpp"
#include "Engine/RL/Calibration/RagdollCalibrationAudit3D.hpp"

#include <cstddef>
#include <string>
#include <string_view>
#include <vector>

namespace MatterEngine {

enum class RagdollJointReviewStatus3D { Pending, Candidate, Approved };

struct RagdollJointReferenceSource3D {
  std::string id;
  std::string title;
  std::string doi;
  std::string url;
  std::string purpose;
  std::string limitations;
};

// Medida clinica de uma direcao anatomica. Nao e automaticamente um limite
// duro: o protocolo, a populacao e a convencao angular permanecem anexos.
struct RagdollJointMotionBenchmark3D {
  std::string id;
  std::string joint;
  std::string motion;
  std::string population;
  float meanDegrees = 0.0f;
  float confidenceLowDegrees = 0.0f;
  float confidenceHighDegrees = 0.0f;
  std::vector<std::string> sourceIds;
  std::string notes;
};

// Faixa de medias publicadas, sob um protocolo de contracao e velocidade
// especifico. Nao representa torque instantaneo universal nem teto do motor.
struct RagdollJointStrengthBenchmark3D {
  std::string id;
  std::string joint;
  std::string motion;
  std::string population;
  std::string contraction;
  float angularVelocityRadiansPerSecond = 0.0f;
  float reportedMeanMinimumNewtonMeters = 0.0f;
  float reportedMeanMaximumNewtonMeters = 0.0f;
  std::vector<std::string> sourceIds;
  std::string notes;
};

// Um registro pode cobrir os dois lados quando os frames e parametros sao
// deliberadamente espelhados. Candidate permite revisar uma hipotese sem
// habilitar comparacao numerica; apenas Approved tem autoridade de referencia.
struct RagdollJointAxisReview3D {
  std::string id;
  std::vector<std::string> linkIds;
  RagdollAxis3D axis = RagdollAxis3D::Twist;
  RagdollJointReviewStatus3D status = RagdollJointReviewStatus3D::Pending;
  std::string anatomicalMotion;
  std::string negativeMotionBenchmarkId;
  std::string positiveMotionBenchmarkId;
  std::vector<std::string> strengthBenchmarkIds;
  std::vector<std::string> sourceIds;
  std::string notes;
};

struct RagdollJointReference3D {
  int schemaVersion = 0;
  std::string id;
  std::string profileId;
  std::string status;
  std::vector<std::string> notes;
  std::vector<RagdollJointReferenceSource3D> sources;
  std::vector<RagdollJointMotionBenchmark3D> motionBenchmarks;
  std::vector<RagdollJointStrengthBenchmark3D> strengthBenchmarks;
  std::vector<RagdollJointAxisReview3D> axisReviews;
};

struct RagdollJointReferenceValidationIssue3D {
  std::string path;
  std::string message;
};

struct RagdollJointAxisComparison3D {
  std::string reviewId;
  std::string linkId;
  RagdollAxis3D axis = RagdollAxis3D::Twist;
  RagdollJointReviewStatus3D status = RagdollJointReviewStatus3D::Pending;
  std::string anatomicalMotion;
  float currentMinimumDegrees = 0.0f;
  float currentMaximumDegrees = 0.0f;
  float currentMaximumTorqueNewtonMeters = 0.0f;
  std::string negativeMotionBenchmarkId;
  std::string positiveMotionBenchmarkId;
  std::vector<std::string> strengthBenchmarkIds;
};

// Comparacao descritiva habilitada somente depois que eixo, sinal e lado
// foram aprovados. O IC pertence a media populacional publicada; nao e uma
// faixa de seguranca nem um gate para um individuo.
struct RagdollJointDirectionalComparison3D {
  std::string reviewId;
  std::string linkId;
  RagdollAxis3D axis = RagdollAxis3D::Twist;
  std::string direction;
  std::string benchmarkId;
  float currentMagnitudeDegrees = 0.0f;
  float publishedMeanDegrees = 0.0f;
  float publishedConfidenceLowDegrees = 0.0f;
  float publishedConfidenceHighDegrees = 0.0f;
  float differenceFromMeanDegrees = 0.0f;
};

struct RagdollJointReferenceComparisonResult3D {
  std::string referenceId;
  std::string referenceStatus;
  std::string profileId;
  std::vector<RagdollJointReferenceSource3D> sources;
  std::vector<RagdollJointMotionBenchmark3D> motionBenchmarks;
  std::vector<RagdollJointStrengthBenchmark3D> strengthBenchmarks;
  std::vector<RagdollJointAxisComparison3D> axes;
  std::vector<RagdollJointDirectionalComparison3D> directionalComparisons;
  std::vector<CalibrationFinding3D> findings;

  [[nodiscard]] std::size_t
  findingCount(CalibrationFindingSeverity3D severity) const;
  [[nodiscard]] bool hasErrors() const;
};

[[nodiscard]] RagdollJointReference3D
loadRagdollJointReference3D(std::string_view filePath);

[[nodiscard]] std::vector<RagdollJointReferenceValidationIssue3D>
validateRagdollJointReference3D(const RagdollJointReference3D &reference);

void validateRagdollJointReferenceOrThrow3D(
    const RagdollJointReference3D &reference);

[[nodiscard]] RagdollJointReferenceComparisonResult3D
compareRagdollJointReference3D(const RagdollProfile3D &profile,
                               const RagdollJointReference3D &reference);

[[nodiscard]] const char *
ragdollJointReviewStatusName3D(RagdollJointReviewStatus3D status);

[[nodiscard]] std::string renderRagdollJointReferenceMarkdown3D(
    const RagdollJointReferenceComparisonResult3D &comparison);

} // namespace MatterEngine
