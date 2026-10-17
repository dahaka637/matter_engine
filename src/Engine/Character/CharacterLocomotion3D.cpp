#include "Engine/Character/CharacterLocomotion3DInternal.hpp"

#include <chrono>

namespace MatterEngine {


bool CharacterLocomotionAnimations3D::compatible(
    const RagdollProfile3D& profile) const {
    return validClip(idle, profile) && validClip(walk, profile);
}

CharacterGaitSpeeds3D characterGaitSpeeds3D(
    const CharacterLocomotionAnimations3D& animations) {
    CharacterGaitSpeeds3D speeds;
    if (animations.walk != nullptr) {
        speeds.walk = authoredHorizontalSpeed(*animations.walk);
    }
    speeds.walkBackward = animations.walkBackward != nullptr
        ? authoredHorizontalSpeed(*animations.walkBackward) : speeds.walk;
    speeds.sprint = animations.sprint != nullptr
        ? authoredHorizontalSpeed(*animations.sprint) : speeds.walk;
    speeds.sprintBackward = animations.sprintBackward != nullptr
        ? authoredHorizontalSpeed(*animations.sprintBackward)
        : speeds.walkBackward;
    // O strafe vem de uma referencia para cada lado, com passadas um pouco
    // diferentes (2,9 e 2,4 m/s no trote). De lado o personagem anda na media
    // dos dois; cada clipe acompanha pelo ritmo (playbackRate).
    const auto sideSpeed = [](const AnimationClip3D& left,
        const AnimationClip3D& right) {
        return 0.5f * (authoredHorizontalSpeed(left)
            + authoredHorizontalSpeed(right));
    };
    if (animations.strafeLeft != nullptr && animations.strafeRight != nullptr) {
        speeds.strafe = sideSpeed(*animations.strafeLeft,
            *animations.strafeRight);
        speeds.sprintStrafe = animations.sprintStrafeLeft != nullptr
                && animations.sprintStrafeRight != nullptr
            ? sideSpeed(*animations.sprintStrafeLeft,
                *animations.sprintStrafeRight)
            : speeds.strafe;
    }
    return speeds;
}

float characterGaitSpeed3D(const CharacterGaitSpeeds3D& speeds,
    float travelRelativeToLookRadians, bool sprinting) {
    const float forward = sprinting ? speeds.sprint : speeds.walk;
    const float backward = sprinting && speeds.sprintBackward > 0.0f
        ? speeds.sprintBackward : speeds.walkBackward;
    const float angle = std::abs(wrapAngle(travelRelativeToLookRadians));
    // As mesmas faixas de histerese que a locomocao usa para trocar de
    // setor: a velocidade passa de um ciclo para o outro dentro delas.
    const auto blend = [](float from, float to, float angle, float border,
        float hysteresis) {
        const float share = std::clamp(
            (angle - (border - hysteresis)) / (2.0f * hysteresis), 0.0f, 1.0f);
        return from + (to - from) * share;
    };
    if (speeds.strafe <= 0.0f) {
        return blend(forward, backward, angle,
            PlainSectorLimitRadians, PlainSectorHysteresisRadians);
    }
    const float side = sprinting ? speeds.sprintStrafe : speeds.strafe;
    return angle <= Pi * 0.5f
        ? blend(forward, side, angle, StrafeFrontLimitRadians,
            StrafeSectorHysteresisRadians)
        : blend(side, backward, angle, StrafeBackLimitRadians,
            StrafeSectorHysteresisRadians);
}

void CharacterLocomotion3D::reset(const RagdollProfile3D& profile,
    const RagdollState3D& state) {
    m_output = {};
    m_telemetry = {};
    m_coordinates.assign(profile.links.size(), {});
    m_coordinateVelocities.assign(profile.links.size(), {});
    m_transitionFrom.assign(profile.links.size(), {});
    m_poseVelocities.assign(profile.links.size(), {});
    m_transitionVelocity.assign(profile.links.size(), {});
    m_lean = {};
    m_leanVelocity = {};
    m_spineBalance = {};
    m_spineBalanceVelocity = {};
    m_spawnHeightOffset = 0.0f;
    m_spawnHeightCaptured = false;
    m_rootTilt = {};
    m_transitionFromRootTilt = {};
    m_rootTiltVelocity = {};
    m_transitionRootTiltVelocity = {};
    m_rootOffsetVelocity = {};
    m_transitionRootOffsetVelocity = {};
    m_previousClip = nullptr;
    m_previousSampledClip = nullptr;
    m_previousClipCoordinates.assign(profile.links.size(), {});
    m_rootOffset = {};
    m_transitionFromRootOffset = {};
    m_groundLift = 0.0f;
    m_terrainSlope = 0.0f;
    m_terrainSpeedCap = 9.0f;
    m_requestYaw = 0.0f;
    m_requestYawValid = false;
    m_requestTurnRate = 0.0f;
    m_curveSpeedCap = 1000.0f;
    m_swingLowWeight = {};
    m_onInclineHold = 0.0f;
    m_pelvisOffset = 0.0f;
    m_pelvisOffsetVelocity = 0.0f;
    m_baseStepOffset = 0.0f;
    m_previousBaseGround = 0.0f;
    m_baseTracked = false;
    m_baseWasGrounded = false;
    m_gaitDirection = CharacterGaitDirection3D::Forward;
    m_feet = {};
    m_stateSeconds = 0.0f;
    m_transitionSeconds = 0.0f;
    m_transitionDurationSeconds = DefaultTransitionSeconds;
    m_cyclePhase = 0.0f;
    m_startClipActive = false;
    m_startClipSprint = false;
    m_startClipSeconds = 0.0f;
    m_authoredCurveBlend = 0.0f;
    m_authoredCurveKind = 0;
    m_idleSeconds = 0.0f;
    m_varietyStrides = 0.0f;
    m_ankleBalanceShift = {};
    m_turningRate = 0.0f;
    m_bodyCurveLean = 0.0f;
    m_spineCurveRoll = 0.0f;
    m_curveSprintShare = 0.0f;
    m_bodyHeading = 0.0f;
    m_bodyHeadingValid = false;
    m_bodyTurnRate = 0.0f;
    m_bodyCurveAcceleration = 0.0f;
    m_reactionStrength = 0.0f;
    m_physicsBlend = 0.0f;
    m_physicsHoldSeconds = 0.0f;
    m_previousDesiredVelocity = {};
    m_smoothedAcceleration = {};
    m_brakeDirection = {};
    m_brakeEntrySpeed = 0.0f;
    m_brakeReaction = {};
    m_brakeReactionVelocity = {};
    m_brakeGaitActive = false;
    m_brakeGait = 0.0f;
    m_brakeGaitSpeed = 0.0f;
    m_brakeGaitDirection = {};
    m_brakeCrouch = 0.0f;
    m_brakeCrouchVelocity = 0.0f;
    m_brakeStepping = false;
    m_brakeReversing = false;
    m_pivotSign = 0.0f;
    m_brakeRunShare = 0.0f;
    m_brakeSeconds = 0.0f;
    m_sinceRunRequest = 10.0f;
    m_brakeReferenceValid = false;
    m_brakeReferencePosition = {};
    m_wasBrakeStepping = false;
    m_stopRoot = {};
    m_stopRootBlend = 0.0f;
    m_stopVelocity = {};
    m_stopHolding = false;
    m_brakeStepWait = 0.0f;
    m_brakeAnchor = 0.0f;
    m_sinceBrakeSeconds = 10.0f;
    m_previousGrounded = true;
    m_rlRootHookGate = 1.0f;
    m_initialized = !profile.links.empty()
        && state.links.size() == profile.links.size()
        && state.joints.size() == profile.links.size();
    if (!m_initialized) return;
    m_previousCenterOfMass = state.links.front().position;
    m_filteredCenterOfMassVelocity = {};
    m_gaitSpeedCap = 1000.0f;
    m_torsoTwist = 0.0f;
    m_headTurn = 0.0f;
    m_filteredLookPitch = 0.0f;
    m_lookValid = false;
    m_lookYawVelocity = 0.0f;
    m_lookPitchVelocity = 0.0f;
    m_poseCoordinates.clear();
    m_standingTurnTarget = 0.0f;
    m_clipPelvisYaw = 0.0f;
    m_clipPelvisToFootYaw = {};
    m_liftoffSpeed = 0.0f;
    m_liftoffVerticalSpeed = 0.0f;
    m_airborneFallSpeed = 0.0f;
    m_landingImpactSpeed = 0.0f;
    m_landingSettleSeconds = 0.18f;
    m_landingSinkPeak = 0.0f;
    m_sinceTouchdownSeconds = 10.0f;
    m_jumpKind = CharacterJumpKind3D::None;
    m_jumpClip = nullptr;
    m_leapClip = nullptr;
    m_flightPhase = 0.0f;
    m_elapsedSeconds = 0.0f;
    m_getUpPhase = CharacterGetUpPhase3D::None;
    m_tackling = false;
    m_tackleClip = nullptr;
    m_tackleCooldown = 0.0f;
    m_previousCrouchIntent = false;
    m_tacklePending = false;
    m_tacklePendingSeconds = 0.0f;
    m_tacklePreviousFoot = {};
    m_sinceTackleSeconds = 10.0f;
    m_fallOrientation = CharacterFallOrientation3D::None;
    m_getUpPhaseSeconds = 0.0f;
    m_recoveredStandingSeconds = 0.0f;
    m_getUpAttempt = 0;
    m_fallReflex = 0.0f;
    m_handCatchSeconds = {};
    m_fallSmoothCoordinates.clear();
    m_fetal = 0.0f;
    m_airBalance = 0.0f;
    m_fallCommit = 0.0f;
    m_tripRecovery = 0.0f;
    m_bracedMemory = m_braceAttemptSeconds = m_braceStallSeconds = 0.0f;
    m_braceRecovering = false;
    m_postureHeldSeconds = 0.0f;
    m_motionSeedPending = false;
    m_postureLost = false;
    m_stagger = 0.0f;
    m_staggerHold = 0.0f;
    m_tripRecoverySeconds = 0.0f;
    m_stumbleValid = false;
    m_stumblePitch = m_stumblePitchRate = m_stumbleBaseline = 0.0f;
    m_stumbleAlong = m_stumbleDecel = m_stumbleRisk = 0.0f;
    m_stumbleReaction = 0.0f;
    m_stumbleDirection = { 1.0f, 0.0f, 0.0f };
    m_stumbleStepping = false;
    m_stumbleStepSeconds = 0.0f;
    m_sinceStumbleStep = 10.0f;
    m_stumbleStepDirection = { 1.0f, 0.0f, 0.0f };
    m_startStepRequested = false;
    m_startStepActive = false;
    m_startDoseCap = 1000.0f;
    m_bodyRestSeconds = 0.0f;
    m_tripFoot = -1;
    m_tripElevating = false;
    m_tripDirection = { 1.0f, 0.0f, 0.0f };
    m_settleTone = 1.0f;
    m_settleRestSeconds = 0.0f;
    m_settlePoseValid = false;
    m_getUpAssistRamp = 0.0f;
    m_sinceGetUpSeconds = 10.0f;
    m_noGroundSeconds = 0.0f;
    m_previousTargetUp = { 0.0f, 0.0f, 1.0f };
    m_balanceReaction = 0.0f;
    m_reactiveShare = 0.0f;
    m_estimator.reset(profile);
    m_capturePoint = {};
    m_captureOutside = 0.0f;
    m_captureDirection = {};
    m_footwork.reset(profile);
    m_swingFromMeasuredPelvis = {};
    m_sprintClipAllowed = true;
    m_legRootFromMeasured = 0.0f;
    m_bodyRefValid = false;
    m_bodyRefHeight = 0.0f;
    m_plantFeetFromBody = false;
    m_handLinks = { findLink(profile, "LeftHand"),
        findLink(profile, "RightHand") };
    m_shinLinks = { findLink(profile, "LeftShin"),
        findLink(profile, "RightShin") };
    m_chestLink = findLink(profile, "Chest");
    for (std::size_t index = 1; index < profile.links.size(); ++index) {
        m_coordinates[index] = {
            state.joints[index].positionRadians[0],
            state.joints[index].positionRadians[1],
            state.joints[index].positionRadians[2]
        };
        m_transitionFrom[index] = m_coordinates[index];
        m_previousClipCoordinates[index] = m_coordinates[index];
        if (profile.links[index].id == "LeftFoot") {
            m_feet[0].linkIndex = index;
            m_feet[0].valid = true;
        } else if (profile.links[index].id == "RightFoot") {
            m_feet[1].linkIndex = index;
            m_feet[1].valid = true;
        }
    }
    m_previousRootPosition = state.links.front().position;
    m_previousRootOrientation = state.links.front().orientation;
    m_previousRisingPose = false;
    m_previousReferencePhase = CharacterGetUpPhase3D::None;
    m_previousRootBase = state.links.front().position;
    m_previousGuideVelocity = {};
    m_previousGuideAngularVelocity = {};
    const Vec3 forward = state.links.front().orientation.rotate(
        { 1.0f, 0.0f, 0.0f });
    m_facingYaw = std::atan2(forward.y, forward.x);
}

void CharacterLocomotion3D::seedMotion(const RagdollProfile3D& profile, const RagdollState3D& state,
    const CharacterLocomotionSeed3D& seed) {
    if (!m_initialized || state.links.empty()) return;
    m_motionSeedPending = true;
    m_seedJoints.assign(profile.links.size(), {});
    for (std::size_t i = 1; i < profile.links.size() && i < state.joints.size(); ++i) {
        const auto& joint = state.joints[i].positionRadians;
        m_seedJoints[i] = { joint[0], joint[1], joint[2] };
    }
    // Ja andando: sem a arrancada (o estado anterior nao e o parado).
    if (seed.moving) m_telemetry.state = CharacterLocomotionState3D::Walking;
    if (seed.tackleClip != nullptr && seed.tackleClip->durationSeconds > 0.0f) {
        // No meio do carrinho do padrao: o carrinho fisico continua do mesmo
        // ponto (como o comeco dele, abaixo em update, mas no tempo dado).
        m_motionSeedPending = false;
        ++m_tackleCount;
        m_tackleClip = seed.tackleClip;
        m_tackleRate = seed.tackleRate;
        m_tackleDistanceScale = seed.tackleScale;
        m_tackleEntrySpeed = seed.tackleEntrySpeed;
        m_tackleYaw = seed.tackleYaw;
        m_tackleStart = { seed.tackleStart.x, seed.tackleStart.y, state.links.front().position.z };
        m_tackling = true;
        m_tacklePending = false;
        m_tackleLandSpeed = -1.0f;
        m_tackleBehindSeconds = 0.0f;
        m_tackleHoldSeconds = 0.0f;
        // (Como enterGetUpPhase em update.)
        m_getUpPhase = CharacterGetUpPhase3D::Rising;
        m_getUpPhaseSeconds = 0.0f;
        m_getUpFootwork = false;
        m_getUpStepSide = -1;
        m_getUpBalanceHold = false;
        m_getUpBalanceHoldSeconds = 0.0f;
        m_getUpBalanceSteps = 0;
        m_getUpClipSeconds = std::clamp(seed.tackleTime, 0.0f, seed.tackleClip->durationSeconds);
        m_getUpAnchorWorld = m_tackleStart;
        m_getUpAnchorYaw = seed.tackleYaw;
        m_fallOrientation = CharacterFallOrientation3D::None;
        m_settleGather = 1.0f;
        m_settleTone = 1.0f;
        m_getUpRate = m_tackleRate;
        m_getUpAttempt = 0;
        m_getUpExpectedValid = false;
        m_getUpStruggle = 0.0f;
        m_getUpStallSeconds = 0.0f;
        m_getUpHoldSeconds = 0.0f;
        m_bodilyRising = false;
        m_risingFromMeasured.assign(state.joints.size(), Vec3 {});
        for (std::size_t link = 1; link < state.joints.size(); ++link)
            m_risingFromMeasured[link] = { state.joints[link].positionRadians[0],
                state.joints[link].positionRadians[1], state.joints[link].positionRadians[2] };
        for (FootPlant& foot : m_feet) {
            foot.planted = false;
            foot.repositioning = false;
            foot.recoveryStep = false;
        }
        m_telemetry.state = CharacterLocomotionState3D::Tackling;
        return;
    }
    const PhysicsBodyState3D& pelvis = state.links.front();
    const Vec3 forward = (pelvis.orientation * profile.links.front().modelOrientation.conjugate())
        .rotate({ 1.0f, 0.0f, 0.0f });
    const Quaternion unyaw = Quaternion::fromAxisAngle({ 0.0f, 0.0f, 1.0f }, -std::atan2(forward.y, forward.x));
    m_seedSpeed = std::hypot(pelvis.linearVelocity.x, pelvis.linearVelocity.y);
    for (std::size_t side = 0; side < 2; ++side) {
        if (!m_feet[side].valid || m_feet[side].linkIndex >= state.links.size()) continue;
        const PhysicsBodyState3D& foot = state.links[m_feet[side].linkIndex];
        m_seedFeet[side] = unyaw.rotate(foot.position - pelvis.position);
        m_seedFeetVelocity[side] = unyaw.rotate(foot.linearVelocity - pelvis.linearVelocity);
    }
}

const CharacterLocomotion3D::ClipFootEvents&
CharacterLocomotion3D::footEventsFor(const RagdollProfile3D& profile,
    const AnimationClip3D& clip) {
    for (const ClipFootEvents& cached : m_clipFootEvents) {
        if (cached.clip == &clip) return cached;
    }
    ClipFootEvents events;
    events.clip = &clip;
    const float duration = std::max(0.05f, clip.durationSeconds);
    const AnimationTrack3D* rootTrack = findAnimationTrack3D(clip,
        profile.links.front().id);
    constexpr int Samples = 240;
    for (std::size_t side = 0; side < 2; ++side) {
        const AnimationScalarTrack3D& contact = side == 0
            ? clip.leftFootContact : clip.rightFootContact;
        const FootPlant& foot = m_feet[side];
        if (contact.keyframes.empty() || !foot.valid) continue;
        const int shin = profile.links[foot.linkIndex].parentIndex;
        const int thigh = shin > 0
            ? profile.links[static_cast<std::size_t>(shin)].parentIndex : -1;
        if (thigh <= 0) continue;
        std::array<float, Samples> weight {};
        for (int i = 0; i < Samples; ++i) {
            weight[static_cast<std::size_t>(i)] = sampleAnimationScalarTrack3D(
                contact, duration * static_cast<float>(i) / Samples, duration,
                true);
        }
        const auto at = [&](int i) {
            return weight[static_cast<std::size_t>((i % Samples + Samples)
                % Samples)];
        };
        for (int i = 0; i < Samples; ++i) {
            if (!(at(i - 1) <= 0.55f && at(i) > 0.55f)) continue;
            FootTouchdown touchdown;
            touchdown.touchdown = duration * static_cast<float>(i) / Samples;
            touchdown.liftoff = touchdown.touchdown;
            for (int back = 1; back < Samples; ++back) {
                if (at(i - back) < 0.30f && at(i - back - 1) >= 0.30f) {
                    touchdown.liftoff = duration
                        * static_cast<float>(((i - back) % Samples + Samples)
                            % Samples) / Samples;
                    break;
                }
            }
            std::vector<Vec3> coordinates(profile.links.size());
            for (std::size_t index = 1; index < profile.links.size(); ++index) {
                const AnimationTrack3D* track = findAnimationTrack3D(clip,
                    profile.links[index].id);
                if (track == nullptr) continue;
                coordinates[index] = sampleAnimationTrack3D(*track,
                    touchdown.touchdown, duration, clip.loops)
                        .jointPositionRadians;
            }
            Vec3 translation;
            Quaternion rotation;
            if (rootTrack != nullptr) {
                const AnimationTransformSample3D root = sampleAnimationTrack3D(
                    *rootTrack, touchdown.touchdown, duration, clip.loops);
                translation = root.translationOffsetMeters;
                rotation = root.rotationDelta.normalized();
            }
            RagdollAnimationPose3D pose;
            buildLocalPose(profile, coordinates, pose);
            const Vec3 hip = profile.links[0].modelOrientation.conjugate()
                .rotate(profile.links[static_cast<std::size_t>(thigh)]
                    .inboundJoint.anchorModelPosition
                    - profile.links[0].modelPosition);
            touchdown.footInClip = translation
                + rotation.rotate(pose.linkPositions[foot.linkIndex]);
            touchdown.hipInClip = translation + rotation.rotate(hip);
            touchdown.forwardInClip = (rotation
                * pose.linkOrientations[foot.linkIndex]).rotate(
                    profile.links[foot.linkIndex].modelOrientation.conjugate()
                        .rotate({ 1.0f, 0.0f, 0.0f }));
            touchdown.footInClip.z = 0.0f;
            touchdown.hipInClip.z = 0.0f;
            events.touchdowns[side].push_back(touchdown);
        }
    }
    m_clipFootEvents.push_back(std::move(events));
    return m_clipFootEvents.back();
}

void CharacterLocomotion3D::enterState(CharacterLocomotionState3D state) {
    if (state == m_telemetry.state) return;
    // O crossfade parte da pose do CLIPE do tick anterior, sem o giro do
    // olhar e sem o IK dos pes, que sao somados de novo por cima: partindo da
    // pose final, o olhar entrava duas vezes e cabeca e tronco davam um
    // tranco a cada troca de estado (medido: cabeca a 50 rad/s ao girar a
    // camera parado, trocando entre parado e girando no lugar).
    m_transitionFrom = m_poseCoordinates.size() == m_coordinates.size()
        ? m_poseCoordinates : m_coordinates;
    m_transitionFromRootTilt = m_rootTilt;
    m_transitionVelocity = m_poseVelocities;
    m_transitionRootTiltVelocity = m_rootTiltVelocity;
    m_transitionRootOffsetVelocity = m_rootOffsetVelocity;
    m_transitionFromRootOffset = m_rootOffset;
    m_telemetry.state = state;
    m_stateSeconds = 0.0f;
    m_transitionSeconds = 0.0f;
    m_transitionDurationSeconds = DefaultTransitionSeconds;
    // Caindo -> deitado e deitado -> levantar: o corpo muda de pose inteiro;
    // em 0,14 s os membros davam um tranco.
    if (state == CharacterLocomotionState3D::Fallen
        || state == CharacterLocomotionState3D::GettingUp) {
        m_transitionDurationSeconds = 0.40f;
    }
    // Carrinho: da corrida direto para o mergulho.
    if (state == CharacterLocomotionState3D::Tackling) {
        m_transitionDurationSeconds = TackleTransition;
    }
    if (state == CharacterLocomotionState3D::Idle) {
        m_cyclePhase = 0.0f;
    }
    if (state == CharacterLocomotionState3D::Airborne
        || state == CharacterLocomotionState3D::JumpStarting
        || state == CharacterLocomotionState3D::Fallen) {
        for (FootPlant& foot : m_feet) {
            foot.planted = false;
            foot.repositioning = false;
            foot.recoveryStep = false;
        }
    }
}

void CharacterLocomotion3D::update(const RagdollProfile3D& profile,
    const CharacterLocomotionAnimations3D& animations,
    const RagdollState3D& state,
    const RagdollDynamics3D& dynamics,
    const CharacterLocomotionInput3D& input,
    float deltaTime) {
    // A reescrita: o update em blocos (CharacterLocomotion3D_<bloco>.cpp), com a mesma conta de
    // antes; o que um bloco passa aos seguintes mora no Tick.
    // (O custo por bloco: so o relogio, nada muda na conta.)
    using Clock = std::chrono::steady_clock;
    auto mark = Clock::now();
    std::size_t section = 0;
    const auto timed = [&]() {
        const auto now = Clock::now();
        m_sectionSeconds[section++] = std::chrono::duration<float>(now - mark).count();
        mark = now;
    };
    Tick t { profile, animations, state, dynamics, input };
    // (Saida zerada a cada tick, mas os vetores guardam a capacidade: antes o
    // '= {}' os liberava e os push_back realocavam todo tick.)
    {
        auto targets = std::move(m_output.driveTargets);
        auto links = std::move(m_output.assistReactionLinks);
        auto torques = std::move(m_output.assistReactionTorquesWorld);
        targets.clear();
        links.clear();
        torques.clear();
        m_output = {};
        m_output.driveTargets = std::move(targets);
        m_output.assistReactionLinks = std::move(links);
        m_output.assistReactionTorquesWorld = std::move(torques);
    }
    // Locomocao fisica v2 (ver input.physicalV2).
    t.PostureSupportPendulum = input.physicalV2 || PostureSupportPendulumEnv;
    const bool& PostureSupportPendulum = t.PostureSupportPendulum;
    t.StartOnlyFromRest = input.physicalV2 || StartOnlyFromRestEnv;
    const bool& StartOnlyFromRest = t.StartOnlyFromRest;
    t.LandingResync = input.physicalV2 || LandingResyncEnv;
    const bool& LandingResync = t.LandingResync;
    static const float stanceBandV2Env = tuneTemp("T_StanceBandV2", StanceRootBandV2);  // TEMPTUNE
    t.StanceRootBand = input.physicalV2 ? stanceBandV2Env : StanceRootBandEnv;
    const float& StanceRootBand = t.StanceRootBand;
    t.StanceDampCurveSpan = input.physicalV2 ? 0.0f : StanceDampCurveSpanEnv;
    const float& StanceDampCurveSpan = t.StanceDampCurveSpan;
    if (m_validatedAnimations != &animations || m_validatedProfile != &profile) {
        m_validatedCompatible = animations.compatible(profile);
        m_validatedAnimations = &animations;
        m_validatedProfile = &profile;
    }
    const auto cheapValid = [&](const AnimationClip3D* clip) {
        return clip != nullptr && clip->targetRigId == profile.id && std::isfinite(clip->durationSeconds)
            && clip->durationSeconds > 0.0f;
    };
    if (!std::isfinite(deltaTime) || deltaTime <= 0.0f
        || !m_validatedCompatible || !cheapValid(animations.idle) || !cheapValid(animations.walk)
        || state.links.size() != profile.links.size()
        || state.joints.size() != profile.links.size()
        || !finite(input.rootPositionWorld)
        || !finite(input.rootVelocityWorld)
        || !finite(input.proxyVelocityWorld)
        || !std::isfinite(input.facingYawRadians)
        || !std::isfinite(input.lookPitchRadians)) {
        return;
    }
    if (!m_initialized) reset(profile, state);
    if (!m_initialized) return;
    deltaTime = std::min(deltaTime, 0.05f);
    t.deltaTime = deltaTime;
    t.previousCoordinates = m_coordinates;
    const std::vector<Vec3>& previousCoordinates = t.previousCoordinates;
    timed();
    // Estado fisico comum deste tick (tambem lido pela assistencia).
    t.physical = &(m_estimator.update(profile, state,
        input.footGround, deltaTime));
    const CharacterPhysicalState3D& physical = *t.physical;
    timed();

    stepSense(t);
    timed();
    stepTackle(t);
    timed();
    stepGetUp(t);
    timed();
    stepAuthority(t);
    timed();
    stepSensors(t);
    timed();
    stepHeading(t);
    timed();
    stepAir(t);
    timed();
    stepCycle(t);
    timed();
    stepGestures(t);
    timed();
    stepLook(t);
    timed();
    stepRoot(t);
    timed();
    stepFeet(t);
    timed();
    stepLegs(t);
    timed();
    stepDrives(t);
    timed();
    stepOutputs(t);
    timed();
}

const char* CharacterLocomotion3D::sectionName(std::size_t index) {
    static constexpr const char* names[SectionCount] = { "Preparo", "Estimador", "Sense", "Tackle", "GetUp", "Authority",
        "Sensors", "Heading", "Air", "Cycle", "Gestures", "Look", "Root", "Feet", "Legs", "Drives", "Outputs" };
    return index < SectionCount ? names[index] : "";
}

// Para a locomocao padrao (DefaultLocomotion3D, 04/10): o corpo do chute e a
// "vida" do modo fisico sobre as mesmas coordenadas de junta.
void characterSolveFootPose3D(const RagdollProfile3D& profile, std::vector<Vec3>& coordinates,
    std::size_t footIndex, Vec3 targetPosition, Quaternion targetOrientation, float weight, bool holdYaw) {
    solveFootPose(profile, coordinates, footIndex, targetPosition, targetOrientation, weight, holdYaw);
}

void characterKickPose3D(const RagdollProfile3D& profile, const RagdollState3D& state,
    std::vector<Vec3>& coordinates, std::array<std::size_t, 2> hands, const BallKickPose3D& kick) {
    applyKickPose(profile, state, coordinates, hands, kick);
}

void characterGaitVariety3D(const RagdollProfile3D& profile, const AnimationClip3D& clip, float cyclePhase,
    float armPhase, float strides, std::uint32_t seed, float amount, std::vector<Vec3>& coordinates) {
    // (Locomocao padrao: somada, nas fases do clipe - substituindo, o tronco e
    // os bracos saltavam inteiros para a pose de outro clipe quando o
    // dominante mudava.)
    applyGaitVariety(profile, clip, cyclePhase, armPhase, strides, seed, amount, coordinates, true, true);
}

void characterIdleVariety3D(const RagdollProfile3D& profile, float seconds, std::uint32_t seed, float amount,
    std::vector<Vec3>& coordinates) {
    applyIdleVariety(profile, seconds, seed, amount, coordinates);
}

} // namespace MatterEngine
