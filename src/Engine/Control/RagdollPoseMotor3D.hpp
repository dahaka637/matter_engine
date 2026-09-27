#pragma once
#include "Engine/Animation/AnimationClip3D.hpp"
#include "Engine/Physics/RagdollProfile3D.hpp"
#include <span>

namespace MatterEngine {

// PhysX-compatible angular velocity in the current child joint frame.
// Exponential-coordinate derivatives are not themselves angular velocity.
[[nodiscard]] Vec3 ragdollJointTargetVelocity3D(Vec3 targetCoordinates,
                                                Vec3 coordinateVelocity,
                                                Vec3 measuredCoordinates);
// FK and bounded IK in root-local coordinates; these never touch physics.
void buildRagdollJointPose3D(const RagdollProfile3D &profile,
                             std::span<const Vec3> coordinates,
                             RagdollAnimationPose3D &pose);
void fitRagdollFootTarget3D(const RagdollProfile3D &profile,
                            std::vector<Vec3> &coordinates, std::size_t foot,
                            Vec3 positionBody, Quaternion orientationBody,
                            float weight);
} // namespace MatterEngine
