#pragma once

#include "Engine/Physics/PhysicsScene3D.hpp"
#include "Engine/RL/Calibration/RagdollCalibrationAudit3D.hpp"

#include <cstddef>
#include <string>
#include <string_view>
#include <vector>

namespace MatterEngine {

// Uma fonte bibliografica e seus limites de uso. O contrato mantem a
// proveniencia junto dos numeros para impedir "constantes magicas" de
// antropometria no codigo.
struct RagdollCalibrationReferenceSource3D {
  std::string id;
  std::string title;
  std::string doi;
  std::string url;
  std::string notes;
};

// Um segmento da literatura pode corresponder a mais de um link do ragdoll.
// A tolerancia relativa e uma faixa de revisao da engine, nao um intervalo
// biologico alegado pela fonte.
struct RagdollMassReferenceGroup3D {
  std::string id;
  std::vector<std::string> linkIds;
  float targetMassFraction = 0.0f;
  float reviewToleranceRelative = 0.0f;
  std::vector<std::string> sourceIds;
  std::string mappingNotes;
};

// Situacao do mapeamento entre os marcos anatomicos publicados e a topologia
// da engine. "PendingLandmarks" e um estado valido e deliberado: guardar o
// dado da fonte nao autoriza inventar um endpoint a partir do collider.
enum class RagdollInertialLandmarkMapping3D {
  PendingLandmarks,
  LandmarkRegistry,
};

// Como um marco anatomico e obtido no espaco de modelo. Pontos oriundos do
// rig ou da malha devem ser gravados como ModelPoint por uma ferramenta de
// autoria: o modulo de calibracao continua sem depender de Modeling/render.
enum class RagdollCalibrationLandmarkKind3D {
  Pending,
  ProfileJointCenter,
  ModelPoint,
  Midpoint,
};

enum class RagdollCalibrationLandmarkStatus3D {
  Pending,
  Candidate,
  Approved,
};

struct RagdollCalibrationLandmark3D {
  std::string id;
  std::string label;
  RagdollCalibrationLandmarkKind3D kind =
      RagdollCalibrationLandmarkKind3D::Pending;
  RagdollCalibrationLandmarkStatus3D status =
      RagdollCalibrationLandmarkStatus3D::Pending;
  std::string jointChildLinkId;
  Vec3 modelPosition;
  bool hasModelPosition = false;
  std::string midpointFirstLandmarkId;
  std::string midpointSecondLandmarkId;
  std::string provenance;
  std::string notes;
};

struct RagdollResolvedCalibrationLandmark3D {
  std::string id;
  std::string label;
  RagdollCalibrationLandmarkKind3D kind =
      RagdollCalibrationLandmarkKind3D::Pending;
  RagdollCalibrationLandmarkStatus3D status =
      RagdollCalibrationLandmarkStatus3D::Pending;
  Vec3 modelPosition;
  bool resolved = false;
  std::string provenance;
  std::string notes;
  std::string reason;
};

struct RagdollRadiusOfGyrationReference3D {
  float sagittalFraction = 0.0f;
  float transverseFraction = 0.0f;
  float longitudinalFraction = 0.0f;
};

// COM e raios de giracao publicados para o mesmo segmento representado por
// um massGroup. As fracoes sao relativas ao comprimento entre sourceOrigin e
// sourceOther. Apenas o eixo longitudinal pode ser comparado com seguranca
// antes de existir um frame anatomico completo no asset.
struct RagdollInertialReferenceGroup3D {
  std::string id;
  std::string originLandmarkId;
  std::string otherLandmarkId;
  float targetCenterOfMassFromOriginFraction = 0.0f;
  RagdollRadiusOfGyrationReference3D targetRadiusOfGyration;
  float centerOfMassReviewToleranceAbsolute = 0.0f;
  float inertiaReviewToleranceRelative = 0.0f;
  std::vector<std::string> sourceIds;
  std::string mappingNotes;
};

struct RagdollCalibrationReference3D {
  int schemaVersion = 0;
  std::string id;
  std::string profileId;
  std::string status;
  std::string population;
  std::vector<std::string> notes;
  std::vector<RagdollCalibrationReferenceSource3D> sources;
  std::vector<RagdollCalibrationLandmark3D> landmarks;
  std::vector<RagdollMassReferenceGroup3D> massGroups;
  std::vector<RagdollInertialReferenceGroup3D> inertialGroups;
};

struct RagdollCalibrationReferenceValidationIssue3D {
  std::string path;
  std::string message;
};

struct RagdollMassReferenceComparison3D {
  std::string groupId;
  std::vector<std::string> linkIds;
  float targetMassFraction = 0.0f;
  float actualMassFraction = 0.0f;
  float differenceMassFraction = 0.0f;
  float relativeDifference = 0.0f;
  float targetMassKg = 0.0f;
  float actualMassKg = 0.0f;
  float reviewToleranceRelative = 0.0f;
  bool withinReviewTolerance = false;
};

struct RagdollInertialReferenceComparison3D {
  std::string groupId;
  std::vector<std::string> linkIds;
  std::string sourceOrigin;
  std::string sourceOther;
  RagdollInertialLandmarkMapping3D mapping =
      RagdollInertialLandmarkMapping3D::PendingLandmarks;
  float targetCenterOfMassFromOriginFraction = 0.0f;
  RagdollRadiusOfGyrationReference3D targetRadiusOfGyration;
  float centerOfMassReviewToleranceAbsolute = 0.0f;
  float inertiaReviewToleranceRelative = 0.0f;
  float segmentLengthMeters = 0.0f;
  float actualCenterOfMassFromOriginFraction = 0.0f;
  float centerOfMassAxisOffsetMeters = 0.0f;
  bool centerOfMassComparable = false;
  bool centerOfMassWithinReviewTolerance = false;
  float unconditionedLongitudinalRadiusFraction = 0.0f;
  float effectiveLongitudinalRadiusFraction = 0.0f;
  bool inertiaComparable = false;
  bool unconditionedInertiaWithinReviewTolerance = false;
  std::string mappingNotes;
};

struct RagdollCalibrationReferenceComparisonResult3D {
  std::string referenceId;
  std::string referenceStatus;
  std::string profileId;
  std::string population;
  float totalMassKg = 0.0f;
  float targetMassFractionSum = 0.0f;
  float actualMassFractionSum = 0.0f;
  std::vector<RagdollCalibrationReferenceSource3D> sources;
  std::vector<RagdollResolvedCalibrationLandmark3D> landmarks;
  std::vector<RagdollMassReferenceComparison3D> massGroups;
  std::vector<RagdollInertialReferenceComparison3D> inertialGroups;
  std::vector<CalibrationFinding3D> findings;

  [[nodiscard]] std::size_t
  findingCount(CalibrationFindingSeverity3D severity) const;
  [[nodiscard]] bool hasErrors() const;
};

[[nodiscard]] RagdollCalibrationReference3D
loadRagdollCalibrationReference3D(std::string_view filePath);

[[nodiscard]] std::vector<RagdollCalibrationReferenceValidationIssue3D>
validateRagdollCalibrationReference3D(
    const RagdollCalibrationReference3D &reference);

void validateRagdollCalibrationReferenceOrThrow3D(
    const RagdollCalibrationReference3D &reference);

[[nodiscard]] RagdollCalibrationReferenceComparisonResult3D
compareRagdollCalibrationReference3D(
    const RagdollProfile3D &profile,
    const RagdollCalibrationReference3D &reference,
    const RagdollEffectiveMass3D *effectiveMassProperties = nullptr);

// Resolve inclusive candidatos para permitir inspecao visual. Consumidores
// numericos devem exigir status Approved; compareRagdoll... faz isso.
[[nodiscard]] std::vector<RagdollResolvedCalibrationLandmark3D>
resolveRagdollCalibrationLandmarks3D(
    const RagdollProfile3D &profile,
    const RagdollCalibrationReference3D &reference);

[[nodiscard]] const char *
ragdollInertialLandmarkMappingName3D(RagdollInertialLandmarkMapping3D mapping);

[[nodiscard]] const char *
ragdollCalibrationLandmarkKindName3D(RagdollCalibrationLandmarkKind3D kind);
[[nodiscard]] const char *ragdollCalibrationLandmarkStatusName3D(
    RagdollCalibrationLandmarkStatus3D status);

[[nodiscard]] std::string renderRagdollCalibrationReferenceMarkdown3D(
    const RagdollCalibrationReferenceComparisonResult3D &comparison);

} // namespace MatterEngine
