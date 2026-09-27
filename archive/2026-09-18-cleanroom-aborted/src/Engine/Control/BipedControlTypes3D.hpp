#pragma once

#include "Engine/Math/Quaternion.hpp"
#include "Engine/Physics/PhysicsScene3D.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <vector>

namespace MatterEngine {

inline constexpr std::size_t InvalidBipedLink3D =
    std::numeric_limits<std::size_t>::max();

enum class BipedSide3D : std::uint8_t { Left = 0, Right = 1 };

enum class BipedControllerPhase3D : std::uint8_t {
    Standing,
    Transfer,
    Swing,
    DoubleSupport,
    Falling,
    GetUp
};

enum class BipedBalanceStrategy3D : std::uint8_t {
    None,
    AnkleHip,
    Step,
    Fall
};

enum class BipedSupportMode3D : std::uint8_t {
    Airborne,
    DoubleSupport,
    LeftSupport,
    RightSupport,
    MultiContactBody
};

enum class BipedStepPhase3D : std::uint8_t {
    Idle,
    Transfer,
    Swing,
    Settle
};

enum class BipedGetUpStage3D : std::uint8_t {
    None,
    EstablishSupport,
    RaiseTorso,
    BringFeetUnderCom,
    Crouch,
    Extend,
    Complete
};

enum class BipedFallPose3D : std::uint8_t {
    Unknown,
    Supine,
    Prone,
    LeftSide,
    RightSide
};

struct BipedTerrainHit3D {
    bool valid = false;
    Vec3 position;
    Vec3 normal { 0.0f, 0.0f, 1.0f };
};

struct BipedTerrainQuery3D {
    std::function<bool(Vec3, float, float, BipedTerrainHit3D&)> sampleGround;
};

struct BipedSupportContact3D {
    std::size_t linkIndex = InvalidBipedLink3D;
    Vec3 positionWorld;
    Vec3 normalWorld { 0.0f, 0.0f, 1.0f };
    float normalLoadNewtons = 0.0f;
    float tangentialSpeedMetersPerSecond = 0.0f;
};

struct BipedFootObservation3D {
    std::size_t linkIndex = InvalidBipedLink3D;
    bool contact = false;
    bool slipping = false;
    Vec3 contactCenterWorld;
    Vec3 contactNormalWorld { 0.0f, 0.0f, 1.0f };
    Vec3 solePositionWorld;
    float normalLoadNewtons = 0.0f;
    float loadRatio = 0.0f;
    float tangentialSpeedMetersPerSecond = 0.0f;
    std::vector<Vec3> contactPointsWorld;
};

struct BipedSupportObservation3D {
    BipedSupportMode3D mode = BipedSupportMode3D::Airborne;
    Vec3 centerWorld;
    float referenceHeightMeters = 0.0f;
    float totalNormalLoadNewtons = 0.0f;
    std::vector<Vec3> footSupportPolygonWorld;
    std::vector<Vec3> allSupportPolygonWorld;
    std::vector<BipedSupportContact3D> contacts;
};

struct BipedObservation3D {
    bool valid = false;
    Vec3 centerOfMassWorld;
    Vec3 centerOfMassVelocityWorld;
    Vec3 linearMomentumWorld;
    Vec3 angularMomentumWorld;
    Vec3 forwardWorld { 1.0f, 0.0f, 0.0f };
    Vec3 leftWorld { 0.0f, 1.0f, 0.0f };
    Vec3 capturePointWorld;
    Vec3 pelvisUpWorld { 0.0f, 0.0f, 1.0f };
    Vec3 torsoUpWorld { 0.0f, 0.0f, 1.0f };
    float torsoUpDot = 1.0f;
    float centerOfMassHeightMeters = 0.0f;
    float captureMarginMeters = 0.0f;
    // Reaction/load ratio measured from the solved articulation. This may
    // include joint-limit/contact reactions and is therefore diagnostic only.
    float jointReactionTorqueRatioPeak = 0.0f;
    bool bodyGroundContact = false;
    bool uprightCandidate = false;
    bool fallenCandidate = false;
    std::array<BipedFootObservation3D, 2> feet;
    BipedSupportObservation3D support;
};

struct BipedIntent3D {
    Vec3 desiredVelocityBody;
    float desiredHeadingRadians = 0.0f;
    bool locomotionEnabled = false;
    bool sprint = false;
    bool allowStepping = true;
};

struct BipedBalanceDecision3D {
    BipedBalanceStrategy3D strategy = BipedBalanceStrategy3D::None;
    float urgency = 0.0f;
    bool mustStep = false;
};

struct BipedStepPlan3D {
    BipedStepPhase3D phase = BipedStepPhase3D::Idle;
    int stanceFoot = -1;
    int swingFoot = -1;
    bool recoveryStep = false;
    bool validLanding = false;
    Vec3 landingSoleWorld;
    Quaternion landingFootOrientationWorld;
    Vec3 swingSoleTargetWorld;
    Quaternion swingFootOrientationWorld;
    Vec3 desiredComWorld;
    float phaseProgress = 0.0f;
    float durationSeconds = 0.0f;
};

struct BipedBalanceCommand3D {
    BipedBalanceStrategy3D strategy = BipedBalanceStrategy3D::None;
    Vec3 desiredComForceWorld;
    Vec3 desiredTorsoTorqueWorld;
    Quaternion desiredTorsoOrientationWorld;
    float urgency = 0.0f;
};

struct BipedEndEffectorTask3D {
    std::size_t linkIndex = InvalidBipedLink3D;
    Vec3 positionWorld;
    Quaternion orientationWorld;
    bool constrainOrientation = false;
    float positionWeight = 1.0f;
    float orientationWeight = 0.0f;
};

struct BipedGetUpCommand3D {
    BipedGetUpStage3D stage = BipedGetUpStage3D::None;
    BipedFallPose3D initialPose = BipedFallPose3D::Unknown;
    std::vector<BipedEndEffectorTask3D> endEffectors;
    Quaternion desiredTorsoOrientationWorld;
    float desiredTorsoWeight = 0.0f;
    float desiredComHeightWorld = 0.0f;
    float desiredComVerticalVelocity = 0.0f;
    bool complete = false;
    bool residualEligible = false;
};

struct BipedExternalWrench3D {
    std::uint32_t linkIndex = 0;
    Vec3 forceNewtons;
    Vec3 torqueNewtonMeters;
};

struct BipedControllerOutput3D {
    std::vector<RagdollDriveTarget3D> driveTargets;
    std::vector<BipedExternalWrench3D> externalWrenches;
    bool gravityCompensationEnabled = true;
};

struct BipedTelemetry3D {
    BipedControllerPhase3D phase = BipedControllerPhase3D::Standing;
    BipedBalanceStrategy3D balanceStrategy = BipedBalanceStrategy3D::None;
    BipedStepPhase3D stepPhase = BipedStepPhase3D::Idle;
    BipedGetUpStage3D getUpStage = BipedGetUpStage3D::None;
    BipedFallPose3D fallPose = BipedFallPose3D::Unknown;
    BipedSupportMode3D supportMode = BipedSupportMode3D::Airborne;
    bool manipulated = false;
    bool commandActive = false;
    bool recoveryStep = false;
    int stanceFoot = -1;
    int swingFoot = -1;
    float commandSecondsRemaining = 0.0f;
    float desiredSpeedMetersPerSecond = 0.0f;
    float measuredSpeedMetersPerSecond = 0.0f;
    float centerOfMassHeightMeters = 0.0f;
    float captureMarginMeters = 0.0f;
    float supportLoadNewtons = 0.0f;
    float leftFootLoadRatio = 0.0f;
    float rightFootLoadRatio = 0.0f;
    float balanceUrgency = 0.0f;
    // Reaction/load ratio measured from the solved articulation. This may
    // include joint-limit/contact reactions and is therefore diagnostic only.
    float jointReactionTorqueRatioPeak = 0.0f;
    // Unclamped motor+feedforward+gravity demand divided by the configured
    // actuator envelope. This is the quantity used by the residual gate.
    float actuatorDemandPeak = 0.0f;
    float residualForceNewtons = 0.0f;
    float residualImpulseNewtonSeconds = 0.0f;
    unsigned getUpReplans = 0;
    Vec3 centerOfMassWorld;
    Vec3 centerOfMassVelocityWorld;
    Vec3 capturePointWorld;
    Vec3 plannedLandingWorld;
};

} // namespace MatterEngine
