#pragma once

#include "Engine/Physics/PhysicsScene3D.hpp"

namespace MatterEngine {

// Residual assistance for the active ragdoll.
//
// DESIGN CONTRACT
// ---------------
// This module is deliberately NOT the primary balance controller. The primary
// controller is made of joint drives, gravity compensation, support-aware
// ankle/hip/spine reactions and, later, recovery stepping. This file only
// supplies small residual forces/torques when the simplified physical model
// cannot quite realize the requested motion by itself.
//
// Important consequences:
// - No world-space position target is ever stored or pursued.
// - No assistance is allowed while unsupported/airborne by default.
// - Animation pose is never reproduced by external forces. Joint motors own it.
// - Lift is a temporary, support-gated aid against gravity during get-up.
// - Posture torque is a last-mile residual, not an invisible upright servo.
// - Yaw is never held to a world heading.
struct BodyRelativeRagdollAssistSettings3D {
  bool assistanceEnabled = true;

  // -----------------------------------------------------------------------
  // SHAPE RESIDUAL
  // -----------------------------------------------------------------------
  // Normally disabled. The joint drives already track animation, therefore an
  // external per-link pose servo would be redundant and can create the exact
  // "floating puppet" behaviour we want to avoid. It is retained as an opt-in
  // diagnostic/fallback and is constrained to zero net force and zero net
  // moment around the whole-body COM.
  bool shapeAssistEnabled = false;
  bool shapeRequiresSupport = true;
  float poseStiffnessPerSecondSquared = 7.0f;
  float poseDampingPerSecond = 1.8f;
  float maximumShapeAssistWeightFraction = 0.035f;
  float maximumShapeAssistTorqueWeightLengthFraction = 0.012f;

  // -----------------------------------------------------------------------
  // MOVEMENT RESIDUAL
  // -----------------------------------------------------------------------
  // Small velocity-level aid only. This is never a positional rail. It is
  // support-gated so throwing the ragdoll through the air cannot cause it to
  // accelerate toward a locomotion command.
  bool movementAssistEnabled = true;
  bool movementRequiresSupport = true;
  float velocityResponsePerSecond = 0.65f;
  float maximumAssistWeightFraction = 0.12f;
  float maximumMovementAccelerationMetersPerSecondSquared = 3.2f;

  // -----------------------------------------------------------------------
  // GET-UP LIFT RESIDUAL
  // -----------------------------------------------------------------------
  // Explicit gravity-up help used by GET-UP only. This is intentionally much
  // stronger than ordinary residual movement assistance: its job is to unload
  // a large part of body weight so the joint motors can physically execute the
  // stand-up clip. It is still NOT a position servo: there is no target height,
  // no saved world point, and no orientation correction hidden in this force.
  // The controller must provide real ground-contact authority; airborne => 0 N.
  bool liftAssistEnabled = true;
  bool liftRequiresSupport = true;
  float liftVelocityResponsePerSecond = 3.8f;
  float maximumLiftAssistWeightFraction = 0.95f;
  float maximumLiftAccelerationMetersPerSecondSquared = 11.0f;

  // Apply the residual mostly to the core. This makes it behave like help to
  // the body COM rather than a set of balloons tied to every limb.
  float pelvisLiftWeight = 1.00f;
  float spineLiftWeight = 0.32f;
  float chestLiftWeight = 0.12f;
  float thighLiftWeight = 0.00f;
  float otherLiftWeight = 0.00f;
  float maximumLiftMomentCancellationWeightLengthFraction = 0.22f;

  // -----------------------------------------------------------------------
  // POSTURE RESIDUAL
  // -----------------------------------------------------------------------
  // Primary posture control belongs to the internal joints. This torque only
  // activates once the body is becoming meaningfully unstable and only while
  // there is physical support capable of reacting against it.
  bool postureAssistEnabled = true;
  bool postureRequiresSupport = true;
  float postureStiffnessWeightLengthPerRadian = 0.26f;
  float postureDampingWeightLengthSeconds = 0.075f;
  float balanceResponseScale = 1.0f;
  float postureDeadZoneRadians = 0.015f;
  float postureFullAuthorityRadians = 0.34f;
  float postureFullAuthorityAngularSpeed = 2.2f;
  float postureResidualActivationUrgency = 0.22f;
  float recoveryPostureGainMultiplier = 1.0f;

  // The total external posture budget is intentionally much smaller than the
  // joint-motor torque budget. If this saturates often, the correct fix is the
  // internal balance controller/contacts, not another larger invisible servo.
  float maximumAssistTorqueWeightLengthFraction = 0.16f;
  float maximumPostureTorquePerLinkWeightLengthFraction = 0.085f;

  // Only the core receives external posture residual by default. Limbs are
  // controlled by their joints and contacts.
  float pelvisPostureWeight = 1.00f;
  float spinePostureWeight = 0.65f;
  float chestPostureWeight = 0.45f;
  float neckPostureWeight = 0.0f;
  float headPostureWeight = 0.0f;
  float legPostureWeight = 0.0f;
  float armPostureWeight = 0.0f;
  float otherPostureWeight = 0.0f;
};

struct BodyRelativeRagdollReference3D {
  // Optional body-relative shape reference. Kept for diagnostics/backwards
  // compatibility; with shapeAssistEnabled=false it has no external effect.
  std::vector<Vec3> positionsBody;
  std::vector<Vec3> velocitiesBody;

  // Desired COM velocity/acceleration in the CURRENT heading frame. These are
  // derivatives only: there is never an integrated world-space destination.
  Vec3 velocityHeading;
  Vec3 accelerationHeading;

  // Desired structural up direction. It controls tilt only, never yaw.
  Vec3 upHeading{0, 0, 1};
  std::vector<Vec3> postureUpHeading;
  std::vector<Vec3> postureUpWorld;

  // Get-up lift request. Positive gravity-up only; no target height. The
  // acceleration component may deliberately unload gravity while supported.
  float liftVelocityUpMetersPerSecond = 0.0f;
  float liftAccelerationUpMetersPerSecondSquared = 0.0f;

  // 0 = no physically useful support, 1 = strong support. Every external
  // residual channel can independently require this value. This is the safety
  // barrier that prevents an airborne ragdoll from magically righting itself.
  float supportAuthority = 0.0f;
  // Optional per-channel overrides. The effective support is max(generic,
  // channelSpecific), which preserves the simple generic contract while letting
  // get-up use hands/knees for lift but require feet for upright posture.
  float shapeSupportAuthority = 0.0f;
  float movementSupportAuthority = 0.0f;
  float liftSupportAuthority = 0.0f;
  float postureSupportAuthority = 0.0f;

  bool recoveryMode = false;

  float poseAuthority = 0.0f;
  float movementAuthority = 0.0f;
  float liftAuthority = 0.0f;
  float balanceAuthority = 0.0f;
};

struct RagdollAssistWrench3D {
  std::uint32_t linkIndex = 0;
  Vec3 forceNewtons;
  Vec3 torqueNewtonMeters;
};

struct BodyRelativeRagdollAssist3D {
  std::vector<RagdollAssistWrench3D> wrenches;

  Vec3 netForceWorld;
  Vec3 netTorqueWorld;
  float forceSumNewtons = 0.0f;
  float torqueSumNewtonMeters = 0.0f;

  float supportAuthority = 0.0f;
  float budgetScale = 1.0f;
  float shapeForceScale = 1.0f;
  float shapeTorqueScale = 1.0f;
  float movementForceScale = 1.0f;
  float liftForceScale = 1.0f;
  float liftMomentScale = 1.0f;
  float postureTorqueScale = 1.0f;

  float postureUrgency = 0.0f;
  float maximumPostureTiltRadians = 0.0f;
  Vec3 movementForceWorld;
  Vec3 liftForceWorld;
  Vec3 postureTorqueWorld;
};

// Stateless by design. Nothing here owns a remembered world position or world
// heading. The function can only react to the current physical snapshot plus
// derivative-level intent supplied by the controller.
[[nodiscard]] BodyRelativeRagdollAssist3D computeBodyRelativeRagdollAssist3D(const RagdollProfile3D &profile, const RagdollState3D &state, const BodyRelativeRagdollReference3D &reference, const BodyRelativeRagdollAssistSettings3D &settings);

} // namespace MatterEngine
