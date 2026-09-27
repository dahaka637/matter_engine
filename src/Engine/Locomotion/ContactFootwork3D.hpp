#pragma once

#include "Engine/Physics/PhysicsScene3D.hpp"

#include <array>
#include <cstdint>

namespace MatterEngine {

enum class ContactFootworkPhase3D : std::uint8_t {
  Supported,
  WeightShift,
  Swing,
  Touchdown,
  Unsupported
};

enum class FootstepReason3D : std::uint8_t {
  None,
  Locomotion,
  BalanceRecovery
};

// Input to the planner. The desired velocity is always expressed in the
// character's CURRENT heading frame. The planner may still step with zero
// desired velocity when balance requires it.
struct ContactFootworkIntent3D {
  Vec3 desiredVelocityHeading;
  GroundProbeResult3D ground;
  std::array<GroundProbeResult3D, 2> footGround;
  bool allowSteps = true;
  bool locomotionEnabled = true;
  bool balanceRecoveryEnabled = true;
};

struct ContactFootworkSettings3D {
  float gravityMetersPerSecondSquared = 9.81f;
  float minimumSupportNormalZ = 0.62f;
  float minimumSupportWeightFraction = 0.010f;

  // Support transfer. Normal gait waits for a clean stance leg; emergency
  // recovery is allowed to release earlier so the controller does not look
  // passive while the body is already leaving the support polygon.
  float minimumStanceWeightFraction = 0.56f;
  float emergencyMinimumStanceWeightFraction = 0.34f;
  float minimumWeightShiftSeconds = 0.055f;
  float maximumWeightShiftSeconds = 0.24f;
  float emergencyWeightShiftSeconds = 0.018f;

  // Capture-point / balance thresholds.
  float captureMarginMeters = 0.025f;
  float fullRecoveryCaptureErrorMeters = 0.12f;
  float recoveryTiltStartRadians = 0.07f;
  float recoveryTiltFullRadians = 0.34f;
  float recoveryAngularSpeedStartRadiansPerSecond = 0.25f;
  float recoveryAngularSpeedFullRadiansPerSecond = 2.6f;
  float recoveryStepUrgencyThreshold = 0.20f;

  // Locomotion thresholds and reach.
  float locomotionStartSpeedMetersPerSecond = 0.08f;
  float maximumComCorrectionMeters = 0.18f;
  float maximumStepReachHeightFraction = 0.68f;
  float minimumFootSeparationMeters = 0.14f;
  float locomotionLeadSeconds = 0.23f;
  float recoveryLeadSeconds = 0.10f;
  float velocityErrorLeadSeconds = 0.10f;

  // Swing timing. Recovery can become much faster than an ordinary walk step.
  float normalSwingDurationSeconds = 0.31f;
  float minimumRecoverySwingDurationSeconds = 0.14f;
  float maximumSwingDurationSeconds = 0.46f;
  float stepHeightMeters = 0.12f;
  float recoveryExtraStepHeightMeters = 0.035f;
  float touchdownTimeoutSeconds = 0.16f;
  float supportLossTimeoutSeconds = 0.14f;
  float goalReplanSpeedMetersPerSecond = 2.2f;
  float recoveryGoalReplanSpeedMetersPerSecond = 4.5f;
};

struct ContactFootworkFoot3D {
  std::uint32_t linkIndex = RagdollDynamics3D::InvalidIndex;
  bool supported = false;
  float normalForceNewtons = 0;
  float supportLoad = 0;
  Vec3 contactPointHeading;
  Vec3 targetPositionHeading;
  float targetWeight = 0;
  float swingWeight = 0;
};

struct ContactFootworkOutput3D {
  bool valid = false;
  Quaternion heading;
  std::array<ContactFootworkFoot3D, 2> feet;

  Vec3 centerOfMassHeading;
  Vec3 centerOfMassVelocityHeading;
  Vec3 supportCenterHeading;
  Vec3 centerOfPressureHeading;
  Vec3 capturePointHeading;
  Vec3 supportErrorHeading;
  Vec3 desiredComCorrectionHeading;

  ContactFootworkPhase3D phase = ContactFootworkPhase3D::Unsupported;
  FootstepReason3D stepReason = FootstepReason3D::None;
  int swingFoot = -1;
  int supportCount = 0;
  float phaseProgress = 0;
  float swingClearanceMeters = 0;
  float desiredSwingClearanceMeters = 0;
  bool waitingForClearance = false;
  float secondsWithoutSupport = 0;
  float uprightDot = 0;
  float bodyTiltRadians = 0;
  float tiltAngularSpeedRadiansPerSecond = 0;
  float rootHeightAboveSupport = 0;
  float stepUrgency = 0;
  float plannedSwingDurationSeconds = 0;
  Vec3 plannedStepDirectionHeading;
  bool nonFootGroundContact = false;
  bool likelyFallen = false;
  bool stepTimedOut = false;
};

// ContactFootwork3D is a reactive footstep planner, not an animation player.
// It observes real support, COM and momentum, predicts loss of balance and
// produces body-relative foot targets. The caller turns these targets into
// joint motion through IK and physical joint drives.
class ContactFootwork3D {
public:
  void reset(const RagdollProfile3D &profile);
  void update(const RagdollProfile3D &profile, const RagdollState3D &state,
              float dt, const ContactFootworkIntent3D &intent);
  [[nodiscard]] ContactFootworkSettings3D &settings() { return m_settings; }
  [[nodiscard]] const ContactFootworkSettings3D &settings() const { return m_settings; }
  [[nodiscard]] const ContactFootworkOutput3D &output() const { return m_output; }

private:
  ContactFootworkSettings3D m_settings;
  ContactFootworkOutput3D m_output;
  std::array<std::uint32_t, 2> m_footIndices{RagdollDynamics3D::InvalidIndex,
                                             RagdollDynamics3D::InvalidIndex};
  std::array<Vec3, 2> m_neutralFeet{};
  std::array<float, 2> m_soleHeight{};
  std::size_t m_linkCount = 0;

  ContactFootworkPhase3D m_phase = ContactFootworkPhase3D::Unsupported;
  FootstepReason3D m_stepReason = FootstepReason3D::None;
  float m_phaseSeconds = 0;
  float m_withoutSupportSeconds = 0;
  float m_swingDuration = 0.31f;
  int m_swingFoot = -1;
  int m_lastSwingFoot = 1;
  bool m_swingReleased = false;
  bool m_clearanceReached = false;
  float m_liftWaitSeconds = 0;
  Quaternion m_previousHeading;
  bool m_frameValid = false;
  Vec3 m_swingStartHeading;
  Vec3 m_swingGoalHeading;
  Vec3 m_stepDirectionHeading;
};

} // namespace MatterEngine
