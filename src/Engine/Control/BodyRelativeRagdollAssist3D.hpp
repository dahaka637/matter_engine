#pragma once

#include "Engine/Physics/PhysicsScene3D.hpp"

namespace MatterEngine {

struct BodyRelativeRagdollAssistSettings3D {
  bool assistanceEnabled = true;
  float poseStiffnessPerSecondSquared = 12.0f;
  float poseDampingPerSecond = 2.0f;
  float velocityResponsePerSecond = 0.5f;
  // Provisional tuning budgets, not restrictions on direction. They apply
  // to the SUM over all links, including the reaction wrenches.
  float maximumAssistWeightFraction = 0.12f;
  float maximumAssistTorqueWeightLengthFraction = 0.035f;
};

struct BodyRelativeRagdollReference3D {
  // Link COMs relative to the ACTUAL pelvis COM, in its rotating frame.
  // Velocities are derivatives in that frame, not world-space differences.
  std::vector<Vec3> positionsBody;
  std::vector<Vec3> velocitiesBody;
  // Heading frame: current body's horizontal facing direction, gravity up.
  // No desired root position, path origin or positional error is accepted.
  Vec3 velocityHeading;
  Vec3 upHeading{0, 0, 1};
  float poseAuthority = 1.0f;
  float movementAuthority = 1.0f;
  float balanceAuthority = 1.0f;
};

struct RagdollAssistWrench3D {
  std::uint32_t linkIndex = 0;
  Vec3 forceNewtons;
  Vec3 torqueNewtonMeters;
};

struct BodyRelativeRagdollAssist3D {
  std::vector<RagdollAssistWrench3D> wrenches;
  Vec3 netForceWorld;
  float forceSumNewtons = 0;
  float torqueSumNewtonMeters = 0;
  float budgetScale = 1;
};

// Stateless by design: cannot remember where a character used to be.
// Shape correction has zero resultant force AND moment. Separate movement
// and balance terms can change whole-body momentum in XYZ; all share a budget.
[[nodiscard]] BodyRelativeRagdollAssist3D computeBodyRelativeRagdollAssist3D(
    const RagdollProfile3D &profile, const RagdollState3D &state,
    const BodyRelativeRagdollReference3D &reference,
    const BodyRelativeRagdollAssistSettings3D &settings);

} // namespace MatterEngine
