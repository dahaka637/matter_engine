#pragma once

#include "Engine/Physics/RagdollProfile3D.hpp"

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace MatterEngine {

enum class CalibrationFindingSeverity3D : std::uint8_t { Information, Warning, Error };

struct CalibrationFinding3D {
    CalibrationFindingSeverity3D severity = CalibrationFindingSeverity3D::Information;
    std::string code;
    std::string path;
    std::string message;
};

struct RagdollLinkCalibration3D {
    std::size_t index = 0;
    std::string id;
    std::string parentId;
    float massFraction = 0.0f;
    float massKg = 0.0f;
    Vec3 centerOfMassModel;
    RagdollColliderShape3D colliderShape = RagdollColliderShape3D::Capsule;
    float approximateColliderVolumeCubicMeters = 0.0f;
    float approximateBulkDensityKgPerCubicMeter = 0.0f;
    float capsuleLengthMeters = 0.0f;
    float capsuleRadiusAtNegativeXMeters = 0.0f;
    float capsuleRadiusAtPositiveXMeters = 0.0f;
    Vec3 boxHalfExtents;
    bool contactSensor = false;
    std::size_t degreesOfFreedom = 0;
};

struct RagdollAxisCalibration3D {
    std::size_t linkIndex = 0;
    std::string linkId;
    RagdollAxis3D axis = RagdollAxis3D::Twist;
    float minimumRadians = 0.0f;
    float maximumRadians = 0.0f;
    float stiffness = 0.0f;
    float damping = 0.0f;
    float maximumTorqueNewtonMeters = 0.0f;
};

struct RagdollSymmetryCalibration3D {
    std::string leftLinkId;
    std::string rightLinkId;
    float massDifferenceKg = 0.0f;
    float mirroredLinkPositionErrorMeters = 0.0f;
    float mirroredCenterOfMassErrorMeters = 0.0f;
    float colliderSizeDifferenceMeters = 0.0f;
    float jointParameterMaximumDifference = 0.0f;
    bool colliderShapesMatch = true;
    bool jointAxesMatch = true;
};

struct RagdollCalibrationAudit3D {
    std::string profileId;
    float totalMassKg = 0.0f;
    float massFractionSum = 0.0f;
    float standingRootHeightMeters = 0.0f;
    Vec3 centerOfMassModel;
    std::size_t linkCount = 0;
    std::size_t jointCount = 0;
    std::size_t degreesOfFreedom = 0;
    std::size_t contactSensorCount = 0;
    std::size_t ignoredSelfCollisionPairCount = 0;
    std::vector<RagdollLinkCalibration3D> links;
    std::vector<RagdollAxisCalibration3D> axes;
    std::vector<RagdollSymmetryCalibration3D> symmetryPairs;
    std::vector<CalibrationFinding3D> findings;

    [[nodiscard]] std::size_t findingCount(CalibrationFindingSeverity3D severity) const;
    [[nodiscard]] bool hasErrors() const;
};

// Audita os dados declarados no perfil. Inercias efetivas condicionadas pelo
// backend e comportamento dinamico pertencem a uma etapa de calibracao
// posterior; este contrato nao finge medi-los a partir do JSON.
[[nodiscard]] RagdollCalibrationAudit3D auditRagdollCalibration3D(const RagdollProfile3D& profile);

// Relatorio deterministico e legivel, apropriado para baseline e revisao.
[[nodiscard]] std::string renderRagdollCalibrationMarkdown3D(const RagdollCalibrationAudit3D& audit);

} // namespace MatterEngine
