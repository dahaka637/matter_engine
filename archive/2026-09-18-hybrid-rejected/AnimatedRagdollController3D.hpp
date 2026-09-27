#pragma once

#include "Engine/Animation/AnimationClip3D.hpp"
#include "Engine/Control/HybridRagdollAssist3D.hpp"
#include "Engine/Locomotion/ContactFootwork3D.hpp"
#include "Engine/Physics/PhysicsScene3D.hpp"

namespace MatterEngine {

enum class AnimatedRagdollPhase3D {
  Idle,
  Running,
  Stopping,
  Falling,
  Walking,
  Fallen,
  GettingUpFront,
  GettingUpBack
};

struct AnimatedRagdollClips3D {
  const AnimationClip3D *idle = nullptr;
  const AnimationClip3D *run = nullptr;
  const AnimationClip3D *stop = nullptr;
  const AnimationClip3D *walkForward = nullptr;
  const AnimationClip3D *walkBackward = nullptr;
  const AnimationClip3D *walkLeft = nullptr;
  const AnimationClip3D *walkRight = nullptr;
  const AnimationClip3D *runBackward = nullptr;
  const AnimationClip3D *standUpFront = nullptr;
  const AnimationClip3D *standUpBack = nullptr;
  [[nodiscard]] bool compatible(const RagdollProfile3D &profile) const;
};

struct AnimatedRagdollSettings3D {
  bool assistanceEnabled = true;
  // Independent normalized authorities; 1 selects the finite maximum budget.
  float auxiliaryAuthority = 0.95f;
  float muscleAuthority = 0.85f;
  float runSpeedMetersPerSecond = 0.0f;
  float walkSpeedMetersPerSecond = 0.65f;

  // Locomotion intent must react immediately. The old controller accelerated at
  // roughly 1.1 m/s^2, which made every command look tired. These rates affect
  // desired velocity only; the body still has to realize it through footwork.
  float walkAccelerationMetersPerSecondSquared = 4.8f;
  float runAccelerationMetersPerSecondSquared = 7.2f;
  float stoppingDecelerationMetersPerSecondSquared = 9.0f;

  // Base joint strength. The V6 relies on joint motors first and residual
  // external assistance second, therefore this is intentionally stronger than
  // the older controller while remaining bounded by each joint's own torque cap.
  float muscleStrength = 2.40f;
  float standingMuscleStrengthMultiplier = 1.34f;

  // Reactive drive boost. Balance urgency raises muscle authority, reference
  // bandwidth and damping BEFORE a fall is declared. This is what makes the
  // character actively fight a disturbance instead of following it passively.
  float reactiveMuscleBoost = 1.15f;
  float reactiveDriveStiffnessBoost = 0.75f;
  float reactiveDriveDampingBoost = 0.32f;
  float locomotionReferenceOmega = 27.0f;
  float emergencyReferenceOmega = 34.0f;
  float locomotionMaximumJointSpeedRadiansPerSecond = 8.0f;
  float walkingAssistWeightFraction = 0.10f;

  // ---------------------------------------------------------------------
  // INTERNAL STANDING BALANCE V6
  // ---------------------------------------------------------------------
  // Balance is primarily realized through joint torques: ankles/feet react at
  // the support, hips transmit corrective torque to the pelvis, and spine joints
  // stabilize the torso. This is the normal controller, not the residual layer.
  float standingFullAuthorityTiltRadians = 0.26f;
  float standingFullAuthorityAngularSpeed = 1.75f;
  float standingFullAuthorityCaptureErrorMeters = 0.105f;
  float standingInternalBalanceTiltGain = 1.12f;
  float standingInternalBalanceCaptureGain = 1.62f;
  float standingInternalBalanceVelocityGain = 0.70f;
  float standingInternalBalanceAngularDamping = 0.29f;
  float standingInternalBalanceMaximumWeightLengthFraction = 0.74f;
  float standingHipBalanceShare = 0.40f;
  float standingAnkleBalanceShare = 0.40f;
  float standingSpineBalanceShare = 0.20f;

  // Residual external posture is deliberately small and support-gated. It only
  // helps after internal control is already working and instability is real.
  float standingResidualMaximumAuthority = 0.85f;
  float standingResidualActivationUrgency = 0.20f;
  float standingAssistTorqueWeightLengthFraction = 0.22f;
  float standingPostureTorquePerLinkWeightLengthFraction = 0.11f;

  // Falling is still an active recovery attempt while useful foot support
  // remains. Airborne bodies keep joint impedance but receive NO external
  // righting torque or lift.
  float fallingMuscleStrengthMultiplier = 1.12f;
  float fallingResidualMaximumAuthority = 0.80f;
  float fallingMinimumRecoveryUpDot = 0.42f;
  float fallingMinimumRecoveryHeightFraction = 0.52f;
  float fallingToFallenDelaySeconds = 0.40f;

  // ---------------------------------------------------------------------
  // GET-UP V6
  // ---------------------------------------------------------------------
  // Absolute cap for the dedicated get-up lift. Near the floor the controller
  // may unload most (but not more than this fraction) of body weight. This is
  // deliberate game assistance and exists ONLY during supported get-up.
  float recoveryAssistWeightFraction = 0.92f;

  // External shape tracking is OFF during get-up. The clip drives the joints;
  // no per-link force is allowed to drag the body into animation positions.
  float recoveryShapeAssistWeightFraction = 0.0f;
  float recoveryShapeAssistTorqueWeightLengthFraction = 0.0f;
  float recoveryShapeAuthority = 0.0f;

  // Dedicated get-up lift. While the body is physically low/non-upright and
  // touching the ground, we deliberately unload gravity at the pelvis/core.
  // This lets the animation's JOINT MOTORS perform the actual stand-up instead
  // of asking external per-link pose forces to drag the ragdoll into place.
  // The lift fades from PHYSICAL height/up, never from a world-space target.
  float recoveryRiseVelocityMetersPerSecond = 0.55f;
  float recoveryLiftVelocityResponsePerSecond = 3.8f;
  float recoveryLiftBaseGravityFraction = 0.56f;
  float recoveryLiftExtraGravityFraction = 0.27f;
  float recoveryMaximumLiftAccelerationMetersPerSecondSquared = 11.0f;
  // Ground-contact telemetry exists only on opt-in links. During get-up we also
  // accept actual collider proximity to the supplied ground plane as support.
  // This makes hands/knees/torso usable without turning every link into a costly
  // contact sensor. The instant all colliders clear this tolerance, lift is off.
  float recoveryGroundClearanceMeters = 0.045f;
  float recoveryLiftFadeStartHeightFraction = 0.56f;
  float recoveryLiftFadeEndHeightFraction = 0.84f;
  float recoveryLiftFadeStartUpDot = 0.48f;
  float recoveryLiftFadeEndUpDot = 0.90f;
  // Kept for source/API compatibility with older laboratory controls. They no
  // longer gate lift by animation progress. Physical state is authoritative.
  float recoveryLiftBiasAccelerationMetersPerSecondSquared = 0.0f;
  float recoveryLiftStartProgress = 0.0f;
  float recoveryLiftEndProgress = 1.0f;

  // External posture is disabled through the early/middle get-up. Near standing
  // it may provide a small support-gated residual while internal ankle/hip/spine
  // balance takes over. It NEVER tracks the authored root orientation.
  float recoveryResidualPostureStartUpDot = 0.58f;
  float recoveryResidualPostureStartHeightFraction = 0.58f;
  float recoveryResidualPostureMaximumAuthority = 0.55f;
  float recoveryResidualPostureTorqueWeightLengthFraction = 0.13f;

  // Joint motors are the actual muscles executing the get-up.
  float recoveryMuscleStrengthMultiplier = 1.90f;
  float recoveryReferenceOmega = 14.0f;
  float recoveryMaximumJointSpeedRadiansPerSecond = 4.8f;
  float recoveryBlendSeconds = 0.48f;

  // State/validation of recovery.
  float fallenSettleSeconds = 0.45f;
  float recoveryRetryDelaySeconds = 0.70f;
  float recoveryCompletionGraceSeconds = 0.85f;
  float recoveryUnsupportedTimeoutSeconds = 0.55f;
  float recoverySuccessHoldSeconds = 0.25f;
  float recoverySuccessUpDot = 0.82f;
  float recoverySuccessHeightFraction = 0.72f;
  float recoverySuccessMaximumLinearSpeed = 1.10f;
  float recoverySuccessMaximumAngularSpeed = 2.60f;

  bool footworkEnabled = true;
  bool steppingEnabled = true;
  bool automaticRecoveryEnabled = false;
};

using AnimatedRagdollForce3D = RagdollAssistWrench3D;

struct AnimatedRagdollOutput3D {
  std::vector<RagdollDriveTarget3D> driveTargets;
  std::vector<AnimatedRagdollForce3D> assistance;
  RagdollAnimationPose3D targetPose;
  bool gravityCompensationEnabled = false;
};

struct AnimatedRagdollTelemetry3D {
  AnimatedRagdollPhase3D phase = AnimatedRagdollPhase3D::Idle;
  float phaseSeconds = 0;
  float runSecondsRemaining = 0;
  float speedMetersPerSecond = 0;
  float travelledMeters = 0;
  float jointRmsDegrees = 0;
  float assistanceForceNewtons = 0;
  float assistanceTorqueNewtonMeters = 0;
  Vec3 assistanceNetForceWorld;
  float assistanceAuthority = 0;
  float supportLoadNewtons = 0;
  float bodyUpDot = 1;
  bool blocked = false;
  bool manipulated = false;
  float movementSecondsRemaining = 0;
  unsigned recoveryAttempts = 0;
  int supportFootCount = 0;
  float captureErrorMeters = 0;
  float muscleTorqueScale = 0;
  float assistanceBudgetWeightFraction = 0;

  // Standing/balance diagnostics.
  float balanceUrgency = 0;
  float internalBalanceTorqueNewtonMeters = 0;
  float residualSupportAuthority = 0;
  float standingPostureAuthority = 0;
  float bodyTiltRadians = 0;
  float tiltAngularSpeedRadiansPerSecond = 0;

  // Get-up diagnostics.
  float recoveryProgress = 0;
  float recoverySupportAuthority = 0;
  float recoveryStableSeconds = 0;
  float recoveryBestHeightMeters = 0;
  float recoveryBestUpDot = -1;
  float footSupportLoadNewtons = 0;
  float nonFootSupportLoadNewtons = 0;
  float recoveryLiftForceNewtons = 0;
  float recoveryMinimumGroundClearanceMeters = 0;
  bool recoveryGeometricSupport = false;
  unsigned recoveryFailures = 0;
  bool recoveryStandingCandidate = false;

  ContactFootworkPhase3D footworkPhase = ContactFootworkPhase3D::Unsupported;
  int swingFoot = -1;
  FootstepReason3D footstepReason = FootstepReason3D::None;
  float footworkStepUrgency = 0;
  float plannedSwingDurationSeconds = 0;
  Vec3 plannedStepDirectionHeading;
  float swingClearanceMeters = 0;
  float desiredSwingClearanceMeters = 0;
  bool waitingForClearance = false;
  Vec3 centerOfMassHeading, supportCenterHeading;
};

class AnimatedRagdollController3D {
public:
  void reset(const RagdollProfile3D &profile, const RagdollState3D &state, float groundHeight);
  [[nodiscard]] bool requestRun(const RagdollState3D &state, float seconds = 5.0f);
  [[nodiscard]] bool requestWalk(const RagdollState3D &state, float seconds = 15.0f, Vec3 directionBody = {1, 0, 0});
  void update(const RagdollProfile3D &profile, const AnimatedRagdollClips3D &clips, const RagdollState3D &state, float deltaTime, float groundHeight, bool manipulated = false, bool obstacleAhead = false, std::uint32_t grabbedLink = RagdollDynamics3D::InvalidIndex);
  [[nodiscard]] Vec3 movementDirectionBody() const { return m_directionBody; }
  [[nodiscard]] AnimatedRagdollSettings3D &settings() { return m_settings; }
  [[nodiscard]] ContactFootworkSettings3D &footworkSettings() { return m_footwork.settings(); }
  [[nodiscard]] const ContactFootworkSettings3D &footworkSettings() const { return m_footwork.settings(); }
  [[nodiscard]] const AnimatedRagdollOutput3D &output() const { return m_output; }
  [[nodiscard]] const AnimatedRagdollTelemetry3D &telemetry() const { return m_telemetry; }

private:
  void transition(AnimatedRagdollPhase3D phase);

  AnimatedRagdollSettings3D m_settings;
  AnimatedRagdollOutput3D m_output;
  AnimatedRagdollTelemetry3D m_telemetry;
  ContactFootwork3D m_footwork;
  HybridRagdollAssist3D m_hybridAssist;

  std::vector<Vec3> m_coordinates, m_coordinateVelocities, m_blendFrom;
  std::vector<Vec3> m_previousBodyPositions;
  bool m_previousBodyPositionsValid = false;

  Vec3 m_commandOrigin;
  Quaternion m_heading;
  Vec3 m_directionBody{1, 0, 0};

  float m_stillSeconds = 0, m_fallSeconds = 0, m_recoveryClock = 0;
  float m_unsupportedRecoverySeconds = 0;
  float m_recoveryStableSeconds = 0, m_recoveryClipFinishedSeconds = 0;
  float m_recoveryRetryCooldownSeconds = 0, m_recoverySupportAuthority = 0;
  float m_recoveryBestHeightMeters = 0, m_recoveryBestUpDot = -1;
  unsigned m_recoveryFailures = 0;

  float m_gaitClock = 0;
  const AnimationClip3D *m_gaitSource = nullptr;
  std::array<float, 2> m_swingPeakSeconds{};
  bool m_stoppingWalk = false;

  float m_phaseTime = 0, m_blendTime = 0, m_requestedSeconds = 5;
  float m_speed = 0, m_stopRate = 1;
  bool m_initialized = false, m_wasManipulated = false;
};

} // namespace MatterEngine
