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
  float auxiliaryAuthority = 1.0f;
  float muscleAuthority = 1.0f;
  // Zero selects the selected clip's horizontal authored travel speed at 1x.
  // Positive values explicitly override it and retime the clip to match.
  float runSpeedMetersPerSecond = 0.0f;
  float walkSpeedMetersPerSecond = 0.0f;
  float walkAccelerationMetersPerSecondSquared = 5.0f;
  float runAccelerationMetersPerSecondSquared = 12.0f;
  float stoppingDecelerationMetersPerSecondSquared = 12.0f;
  bool footworkEnabled = true;
  bool steppingEnabled = true;
  bool automaticRecoveryEnabled = false; // Reserved; get-up is deliberately deferred.
};

using AnimatedRagdollForce3D = RagdollAssistWrench3D;

struct AnimatedRagdollOutput3D {
  std::vector<RagdollDriveTarget3D> driveTargets;
  std::vector<AnimatedRagdollForce3D> assistance;
  RagdollAnimationPose3D targetPose;
  RagdollAnimationConstraint3D animationConstraint;
  bool gravityCompensationEnabled = false;
};

struct AnimatedRagdollTelemetry3D {
  AnimatedRagdollPhase3D phase = AnimatedRagdollPhase3D::Idle;
  float phaseSeconds = 0;
  float runSecondsRemaining = 0;
  float speedMetersPerSecond = 0;
  float animationPlaybackRate = 1;
  float commandedSpeedMetersPerSecond = 0;
  float travelledMeters = 0;
  float jointRmsDegrees = 0;
  float assistanceForceNewtons = 0;
  float assistanceTorqueNewtonMeters = 0;
  Vec3 assistanceNetForceWorld;
  float assistanceAuthority = 0;
  RagdollTraversalMode3D traversalMode = RagdollTraversalMode3D::Airborne;
  GroundProbeResult3D ground;
  RagdollAssistChannels3D assistChannels;
  Vec3 proxyRequestedDisplacement;
  Vec3 proxyAllowedDisplacement;
  Vec3 physicalRootVelocity;
  Vec3 proxyDesiredVelocity;
  bool proxyBlocked = false;
  bool groundAdhesionActive = false;
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
  void reset(const RagdollProfile3D &profile, const RagdollState3D &state);
  [[nodiscard]] bool requestRun(const RagdollState3D &state, float seconds = 5.0f);
  [[nodiscard]] bool requestWalk(const RagdollState3D &state, float seconds = 15.0f, Vec3 directionBody = {1, 0, 0});
  void update(const RagdollProfile3D &profile,
      const AnimatedRagdollClips3D &clips, const RagdollState3D &state,
      const CapsuleTraversalResult3D &traversal, float deltaTime,
      const std::array<GroundProbeResult3D, 2> &footGround = {},
      bool manipulated = false,
      std::uint32_t grabbedLink = RagdollDynamics3D::InvalidIndex);
  [[nodiscard]] Vec3 desiredPlanarVelocityWorld(
      const RagdollState3D &state) const;
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
  Vec3 m_commandOrigin;
  Quaternion m_heading;
  Vec3 m_directionBody{1,0,0};
  float m_phaseTime=0, m_blendTime=0, m_requestedSeconds=0;
  float m_speed=0, m_gaitClock=0;
  Quaternion m_previousRootTilt, m_blendRootRotationFrom;
  Vec3 m_previousRootTranslation, m_blendRootTranslationFrom;
  const AnimationClip3D* m_gaitSource=nullptr;
  std::array<float,2> m_swingPeakSeconds{};
  bool m_initialized=false, m_wasManipulated=false, m_stoppingWalk=false;
};

} // namespace MatterEngine
