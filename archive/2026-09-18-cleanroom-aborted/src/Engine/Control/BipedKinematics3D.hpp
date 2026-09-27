#pragma once

#include "Engine/Physics/PhysicsScene3D.hpp"

#include <span>
#include <vector>

namespace MatterEngine {

struct BipedKinematicPose3D {
    std::vector<Vec3> positionsWorld;
    std::vector<Quaternion> orientationsWorld;
};

struct BipedIkSettings3D {
    unsigned iterations = 10;
    unsigned maximumChainDepth = 4;
    float finiteDifferenceRadians = 0.002f;
    float damping = 0.025f;
    float maximumDeltaRadians = 0.16f;
    float positionWeight = 1.0f;
    float orientationWeight = 0.20f;
};

[[nodiscard]] std::vector<Vec3> bipedJointCoordinatesFromState3D(
    const RagdollProfile3D& profile,
    const RagdollState3D& state);

void clampBipedJointCoordinates3D(
    const RagdollProfile3D& profile,
    std::vector<Vec3>& coordinates);

[[nodiscard]] BipedKinematicPose3D buildBipedKinematicPose3D(
    const RagdollProfile3D& profile,
    std::span<const Vec3> coordinates,
    Vec3 rootPositionWorld,
    Quaternion rootOrientationWorld);

[[nodiscard]] bool solveBipedEndEffectorDls3D(
    const RagdollProfile3D& profile,
    std::vector<Vec3>& coordinates,
    Vec3 rootPositionWorld,
    Quaternion rootOrientationWorld,
    std::size_t endEffectorLink,
    Vec3 targetPositionWorld,
    Quaternion targetOrientationWorld,
    bool constrainOrientation,
    float taskWeight,
    const BipedIkSettings3D& settings = {});

} // namespace MatterEngine
