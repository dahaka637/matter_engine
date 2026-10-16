#include "Engine/Character/CharacterLocomotion3DInternal.hpp"

namespace MatterEngine {

void CharacterLocomotion3D::stepFeet(Tick& t) {
    const RagdollProfile3D& profile = t.profile;
    const RagdollState3D& state = t.state;
    const CharacterLocomotionInput3D& input = t.input;
    float& deltaTime = t.deltaTime;
    const CharacterPhysicalState3D& physical = *t.physical;
    const float& groundHeightGuess = t.groundHeightGuess;
    const bool& getUpActive = t.getUpActive;
    float& gaitVelocityScale = t.gaitVelocityScale;
    Vec3& brakeNavigationVelocity = t.brakeNavigationVelocity;
    const float& speed = t.speed;
    const float& curveIntent = t.curveIntent;
    bool& grounded = t.grounded;
    CharacterLocomotionState3D& desiredState = t.desiredState;
    const Quaternion& heading = t.heading;
    const bool& gaitMoving = t.gaitMoving;
    const bool& gaitStarting = t.gaitStarting;
    const float& gaitCycleSpeed = t.gaitCycleSpeed;
    const float& signedCurveAcceleration = t.signedCurveAcceleration;
    const AnimationClip3D*& selectedClip = t.selectedClip;
    float& playbackRate = t.playbackRate;
    float& strideScale = t.strideScale;
    std::array<float, 2>& contactWeights = t.contactWeights;
    std::array<float, 2>& legSampleTime = t.legSampleTime;
    const float& curveShare = t.curveShare;
    const Quaternion& rootTilt = t.rootTilt;
    const Quaternion& accelerationTilt = t.accelerationTilt;
    const Quaternion& targetRootOrientation = t.targetRootOrientation;
    Vec3& navigationRoot = t.navigationRoot;
    const float& lowAssist = t.lowAssist;
    Vec3& targetRootPosition = t.targetRootPosition;
    const bool& airbornePose = t.airbornePose;
    RagdollAnimationPose3D& localPose = t.localPose;
    // =====================================================================
    // Pes no terreno.
    //
    // Cada pe tem o seu chao. No apoio o pe fica travado no mundo, plano sobre
    // o chao que a sonda achou embaixo dele (calcanhar, meio e ponta; vale o
    // mais alto, para o pe nunca afundar na quina de um degrau). No balanco
    // ele sai da altura de onde levantou e chega na altura de onde VAI pisar:
    // o proprio clipe diz quando e onde o pe toca (o proximo contato), a
    // velocidade da capsula diz onde a pelve estara, e a sonda mede o chao ali
    // antes de o pe chegar. Subindo, o pe ganha a altura cedo e com folga;
    // descendo, so depois de passar a quina. A pelve desce o que for preciso
    // para o pe mais baixo alcancar o chao dele.
    //
    // O ciclo acompanha a distancia que a capsula anda: o ritmo tem limites
    // e o que falta ou sobra vira passo mais curto ou mais longo (stride
    // warping). Parado, se a pelve se afasta de um pe travado (parou no meio
    // da passada, levou um esbarrao), o pe da um passo - o corpo nao anda
    // sem as pernas andarem.
    // =====================================================================
    const float capsuleBase = groundHeightGuess;
    // Degrau da capsula suavizado: o controlador sobe e desce degrau num
    // salto de altura; a pelve vai junto, mas em ~0,1 s.
    if (m_baseTracked && input.grounded && m_baseWasGrounded && !getUpActive) {
        const float jump = capsuleBase - m_previousBaseGround;
        if (std::abs(jump) > 0.025f && std::abs(jump) < 0.6f) {
            m_baseStepOffset -= jump;
        }
    }
    if (!input.grounded || getUpActive) m_baseStepOffset = 0.0f;
    m_baseStepOffset *= std::exp(-12.0f * deltaTime);
    m_previousBaseGround = capsuleBase;
    m_baseTracked = true;
    m_baseWasGrounded = input.grounded;
    targetRootPosition.z += m_baseStepOffset;
    // The spawn may have clearance above the floor. Preserve its initial
    // height, then settle onto the guide before IK, rather than snapping
    // the entire articulated body down on the first physics tick.
    if (!m_spawnHeightCaptured) {
        m_spawnHeightOffset = state.links.front().position.z - targetRootPosition.z;
        m_spawnHeightCaptured = true;
    }
    targetRootPosition.z += m_spawnHeightOffset
        * (1.0f - smoothStep(std::max(0.0f, m_elapsedSeconds - deltaTime) / 0.30f));
    t.baseGround = capsuleBase + m_baseStepOffset;
    const float& baseGround = t.baseGround;

    // Caindo sem volta, os pes nao miram passos: estao nos reflexos. So
    // desequilibrado, continuam dando passos (e podem salvar o equilibrio).
    t.footworkEnabled = !getUpActive && m_fallCommit < 0.5f;
    const bool& footworkEnabled = t.footworkEnabled;
    t.onGround = input.grounded && !airbornePose;
    const bool& onGround = t.onGround;
    const auto groundUnder = [&](std::size_t side, Vec3 center,
        Vec3 forward) -> GroundSample {
        GroundSample result;
        if (input.groundAt) {
            // A sonda procura a partir da altura do chao da capsula (quem
            // implementa groundAt comeca um pouco acima dela): uma viga na
            // altura do peito nao vira chao.
            center.z = baseGround;
            forward.z = 0.0f;
            forward = forward.lengthSquared() > 0.000001f
                ? forward.normalized() : heading.rotate({ 1.0f, 0.0f, 0.0f });
            for (const float reach : { -0.09f, 0.0f, 0.09f }) {
                const GroundProbeResult3D probe =
                    input.groundAt(center + forward * reach);
                if (!probe.hasSurface || !probe.walkable) continue;
                // Evaluate the surface plane at the sole center: selecting
                // the highest toe sample adds artificial height on ramps.
                const float height = probe.pointWorld.z
                    - (probe.normalWorld.x * (center.x - probe.pointWorld.x)
                        + probe.normalWorld.y * (center.y - probe.pointWorld.y))
                        / std::max(0.2f, probe.normalWorld.z);
                if (!result.valid || height > result.height) {
                    result.valid = true;
                    result.height = height;
                    result.normal = probe.normalWorld;
                }
            }
            if (result.valid) return result;
        }
        const GroundProbeResult3D& legacy = input.footGround[side];
        result.valid = true;
        if (legacy.hasSurface) {
            result.height = legacy.pointWorld.z;
            result.normal = legacy.normalWorld;
        } else {
            result.height = baseGround;
        }
        return result;
    };
    // Pe inteiro num nivel: calcanhar, meio e ponta na mesma altura. Numa
    // quina de degrau (o pe tem 28 cm, o degrau tambem), o ponto de pouso
    // anda o minimo ao longo do pe ate caber inteiro num degrau.
    const auto fitFoot = [&](Vec3 center, Vec3 forward) -> Vec3 {
        if (!input.groundAt) return {};
        forward.z = 0.0f;
        if (forward.lengthSquared() < 0.000001f) return {};
        forward = forward.normalized();
        const auto spread = [&](Vec3 at, float& highest) {
            float low = std::numeric_limits<float>::infinity();
            highest = -std::numeric_limits<float>::infinity();
            for (const float reach : { -0.12f, 0.0f, 0.12f }) {
                Vec3 point = at + forward * reach;
                point.z = baseGround;
                const GroundProbeResult3D probe = input.groundAt(point);
                if (!probe.hasSurface) return 1.0f;
                low = std::min(low, probe.pointWorld.z);
                highest = std::max(highest, probe.pointWorld.z);
            }
            return highest - low;
        };
        float level = 0.0f;
        if (spread(center, level) < 0.02f) return {};
        Vec3 best {};
        float bestLevel = -std::numeric_limits<float>::infinity();
        for (float distance = 0.02f; distance <= 0.20f; distance += 0.02f) {
            for (const float sign : { 1.0f, -1.0f }) {
                float height = 0.0f;
                const Vec3 shift = forward * (sign * distance);
                if (spread(center + shift, height) < 0.015f
                    && height > bestLevel) {
                    best = shift;
                    bestLevel = height;
                }
            }
            if (std::isfinite(bestLevel)) return best;
        }
        return {};
    };
    // Rumo do clipe para o pe, sem a inclinacao de aceleracao: o pe de apoio
    // fica com a sola no chao, nao inclinado junto com o corpo.
    const auto clipFootOrientation = [&](std::size_t linkIndex) {
        return (heading * rootTilt * localPose.linkOrientations[linkIndex])
            .normalized();
    };
    const auto lockHeight = [&](const FootPlant& foot, float ground,
        Quaternion orientation) {
        const RagdollLinkDefinition3D& link = profile.links[foot.linkIndex];
        const Quaternion collider = (orientation
            * link.collider.localOrientation).normalized();
        return ground + lowestColliderOffset(link, collider)
            - orientation.rotate(link.collider.localPosition).z + 0.001f;
    };
    const auto forwardOf = [&](const FootPlant& foot, Quaternion orientation) {
        return orientation.rotate(profile.links[foot.linkIndex]
            .modelOrientation.conjugate().rotate({ 1.0f, 0.0f, 0.0f }));
    };
    const ClipFootEvents* events = gaitMoving && selectedClip != nullptr
        ? &footEventsFor(profile, *selectedClip) : nullptr;
    const float clipTravel = selectedClip != nullptr
        ? selectedClip->travelDirectionRadians : 0.0f;
    const Vec3 travelAxisLocal = (accelerationTilt * rootTilt).conjugate()
        .rotate({ std::cos(clipTravel), std::sin(clipTravel), 0.0f });
    t.planarVelocity = Vec3 { input.rootVelocityWorld.x,
        input.rootVelocityWorld.y, 0.0f } * gaitVelocityScale;
    Vec3& planarVelocity = t.planarVelocity;
    if (gaitStarting && speed > 0.001f)
        planarVelocity *= gaitCycleSpeed / speed;
    planarVelocity += (brakeNavigationVelocity - planarVelocity) * m_brakeGait;

    t.goals = {};
    std::array<FootGoal, 2>& goals = t.goals;

    // A perna ainda alcanca o pe apoiado? Descendo escada o corpo segue e o
    // degrau de tras fica longe: segurar a trava deixava o pe pendurado no
    // limite do IK (26 cm acima do degrau), marcado como apoiado. Tambem: o
    // IK nao chegou la no tick anterior (limite de junta).
    const auto legOutOfReach = [&](const FootPlant& foot) {
        const RagdollLinkDefinition3D& link = profile.links[foot.linkIndex];
        const int shin = link.parentIndex;
        const int thigh = shin > 0 ? profile.links[
            static_cast<std::size_t>(shin)].parentIndex : -1;
        if (thigh <= 0) return foot.reachError > 0.06f;
        const RagdollLinkDefinition3D& shinLink =
            profile.links[static_cast<std::size_t>(shin)];
        const RagdollLinkDefinition3D& thighLink =
            profile.links[static_cast<std::size_t>(thigh)];
        const float legLength = (shinLink.inboundJoint.anchorModelPosition
                - thighLink.inboundJoint.anchorModelPosition).length()
            + (link.inboundJoint.anchorModelPosition
                - shinLink.inboundJoint.anchorModelPosition).length();
        const Vec3 hip = targetRootPosition + Vec3 { 0.0f, 0.0f, m_pelvisOffset }
            + targetRootOrientation.rotate(profile.links[0].modelOrientation
                .conjugate().rotate(thighLink.inboundJoint.anchorModelPosition
                    - profile.links[0].modelPosition));
        const Vec3 ankle = foot.plantedGoalPosition
            + foot.plantedGoalOrientation.rotate(link.modelOrientation.conjugate()
                .rotate(link.inboundJoint.anchorModelPosition - link.modelPosition));
        return (hip - ankle).length() > legLength * 1.01f || foot.reachError > 0.06f;
    };

    // Pes: o planejador unico de passos (ContactFootwork3D) decide e executa
    // os passos com o corpo parado - acomodacao, giro no lugar, recuperacao
    // de empurrao/pressao/arrasto, refazer a base depois de levantar - e so
    // pousa um pe por contato medido. Andando, a passada do clipe (abaixo)
    // ainda decide os pes; o planejador acompanha para assumir sem salto.
    // Parado de fato: sem passada e sem o jogador pedindo movimento. Na
    // troca de direcao (strafe para um lado e para o outro) a capsula passa
    // por velocidade zero num tick; tomar isso por "parado" entregava os dois
    // pes ao planejador no meio da passada.
    t.movementRequested =
        input.intent.requestedDirectionWorld.lengthSquared() > 0.01f;
    const bool& movementRequested = t.movementRequested;
    t.standing = footworkEnabled && !gaitMoving && !movementRequested
        && input.grounded
        && desiredState != CharacterLocomotionState3D::Landing
        && !airbornePose;
    const bool& standing = t.standing;
    // Quase-queda correndo: o planejador da um passo rapido que pega o corpo
    // e devolve os pes a passada (ver ContactFootworkInput3D::stumbling).
    // Pelo ciclo do clipe nao dava: pelve baixa, pouso adiantado, cadencia
    // ou a perna mirando pela pelve de verdade, todos derrubavam mais no pe
    // enganchado em sprint (26 -> 36-50 em 128) - o passo do clipe tem o
    // tempo e o tamanho da corrida, nao o de quem se salva.
    m_sinceStumbleStep = std::min(10.0f, m_sinceStumbleStep + deltaTime);
    {
        const float bodySpeed = std::hypot(m_filteredCenterOfMassVelocity.x, m_filteredCenterOfMassVelocity.y);
        const bool allowed = StumbleStepTrigger > 0.0f && footworkEnabled && onGround && gaitMoving
            && input.forceDrivenRoot && !getUpActive && m_fallCommit < 0.5f
            && m_brakeGait <= 0.0f && !m_brakeGaitActive;
        const Vec3 requestedDirection { input.intent.requestedDirectionWorld.x, input.intent.requestedDirectionWorld.y, 0.0f };
        const bool startAllowed = StartStepEnabled && footworkEnabled && onGround && gaitMoving
            && input.forceDrivenRoot && !getUpActive && m_fallCommit < 0.5f && !m_brakeGaitActive
            && m_gaitDirection == CharacterGaitDirection3D::Forward && requestedDirection.lengthSquared() > 0.01f;
        if (m_startStepRequested && !m_stumbleStepping && startAllowed && bodySpeed < 1.0f) {
            m_stumbleStepping = true;
            m_startStepActive = true;
            m_stumbleStepSeconds = 0.0f;
            m_stumbleStepDirection = requestedDirection.normalized();
        } else if (!m_stumbleStepping && allowed && m_stumbleRisk >= StumbleStepTrigger && bodySpeed > 1.5f
            && m_sinceStumbleStep > StumbleStepCooldown) {
            m_stumbleStepping = true;
            m_startStepActive = false;
            m_stumbleStepSeconds = 0.0f;
            m_stumbleStepDirection = m_stumbleDirection;
        } else if (m_stumbleStepping && !(m_startStepActive ? startAllowed : allowed)) {
            m_stumbleStepping = false;
        }
        m_startStepRequested = false;
        if (!m_stumbleStepping) m_startStepActive = false;
        if (m_stumbleStepping) m_stumbleStepSeconds += deltaTime;
    }
    {
        ContactFootworkInput3D footInput;
        footInput.physical = &physical;
        footInput.state = &state;
        footInput.standing = standing;
        footInput.allowSteps = footworkEnabled && onGround;
        // Recuperacao so no modo fisico, sem queda comprometida e ha mais de
        // 0,8 s de pe (recem-levantado, a base se refaz pelos passos de
        // acomodacao, um pe de cada vez). Segurado pela PhysGun tambem:
        // arrastado, ele da passos.
        footInput.allowRecovery = m_fallCommit < 0.5f
            && m_sinceGetUpSeconds > 0.8f;
        m_telemetry.footworkStanding = standing;
        m_telemetry.footworkAllowSteps = footInput.allowSteps;
        m_telemetry.footworkAllowRecovery = footInput.allowRecovery;
        m_telemetry.captureDirectionWorld = m_captureDirection;
        footInput.seedFromBody = m_plantFeetFromBody && onGround && footworkEnabled;
        footInput.facingYawRadians = m_facingYaw;
        footInput.standingTurnTargetYawRadians = m_standingTurnTarget;
        footInput.standingTurnRateRadiansPerSecond = m_turningRate;
        footInput.pelvisYawOffsetRadians = m_clipPelvisYaw;
        footInput.heading = heading;
        footInput.referenceRootWorld = targetRootPosition;
        footInput.capturePointWorld = m_capturePoint;
        footInput.captureOutsideMeters = m_captureOutside;
        footInput.captureDirectionWorld = m_captureDirection;
        footInput.planningComVelocityWorld = m_filteredCenterOfMassVelocity;
        footInput.locomoting = movementRequested;
        footInput.desiredVelocityWorld = input.intent.requestedVelocityWorld;
        footInput.braking = m_brakeStepping && footworkEnabled && onGround;
        // Recem-freado (a base ainda e a da freada, as vezes com o corpo na
        // borda de tras dela): acomodar ou girar tirava do chao justo o pe que
        // segurava o corpo.
        // (Ate a camera girar de verdade: ai o giro parado volta a valer.)
        // Ajuda cheia segurando (sem perturbacao de fora): o equilibrio e dela.
        footInput.assistHold = 0.0f;
        // (Com ela, a base se fecha logo depois da freada: a espera existia
        // porque o corpo podia estar na borda de tras da base.)
        footInput.settleHold = m_sinceBrakeSeconds < BrakeSettleHoldSeconds
            && std::abs(wrapAngle(m_lookYaw - m_facingYaw)) < 0.55f
            && footInput.assistHold < 0.5f;
        footInput.quietStop = (m_brakeGaitActive || m_sinceBrakeSeconds < BrakeSettleHoldSeconds)
            && input.externalPerturbation < 0.2f;
        footInput.brakeDirectionWorld = m_brakeGaitDirection;
        footInput.brakeDecelerationMetersPerSecondSquared = BrakeDeceleration;
        footInput.stumbling = m_stumbleStepping && !footInput.braking;
        footInput.stumbleDirectionWorld = m_stumbleStepDirection;
        footInput.stumbleAheadMeters = m_startStepActive ? StartStepAhead
            : StumbleStepAhead;
        footInput.stumbleSwingSeconds = m_startStepActive ? StartStepSwing
            : StumbleStepSwing;
        footInput.stumbleStepDelaySeconds = m_startStepActive ? StartPushSeconds : 0.0f;
        footInput.stumbleLiftMeters = m_startStepActive ? StartStepLift : 0.0f;
        for (std::size_t side = 0; side < m_feet.size(); ++side) {
            const FootPlant& foot = m_feet[side];
            if (!foot.valid) continue;
            footInput.reference[side].poseWorld = targetRootPosition
                + targetRootOrientation.rotate(localPose.linkPositions[foot.linkIndex]);
            footInput.reference[side].poseOrientationWorld =
                clipFootOrientation(foot.linkIndex);
            footInput.anchorUnreachable[side] = foot.planted && legOutOfReach(foot);
            m_telemetry.footworkAnchorUnreachable[side] = footInput.anchorUnreachable[side];
            footInput.gait[side].planted = foot.planted;
            footInput.gait[side].lockedWorld = foot.lockedPositionWorld;
            footInput.gait[side].lockedOrientationWorld = foot.lockedOrientationWorld;
            footInput.gait[side].surface = { true, foot.groundHeight, foot.groundNormal };
        }
        // Giro parado pelo clipe: os pes sao do clipe (a pose de referencia ja
        // tem as pernas dele) - no ar pelo contato do clipe, no chao travados.
        if (m_clipTurn.active && m_clipTurn.clip != nullptr && m_clipTurnWeight > 0.5f) {
            const AnimationClip3D& clip = *m_clipTurn.clip;
            for (std::size_t side = 0; side < 2 && side < m_feet.size(); ++side) {
                if (!m_feet[side].valid) continue;
                const float contact = sampleAnimationScalarTrack3D(side == 0 ? clip.leftFootContact : clip.rightFootContact,
                    m_clipTurn.time, clip.durationSeconds, false);
                ScriptedFoot3D& script = footInput.scripted[side];
                script.active = true;
                script.swing = contact < 0.5f;
                script.positionWorld = footInput.reference[side].poseWorld;
                script.orientationWorld = footInput.reference[side].poseOrientationWorld;
            }
        }
        // Toque parado na bola: o pe dirigido pelo manejo.
        if (input.ballScript.active && input.ballScript.side >= 0 && input.ballScript.side < 2
            && m_feet[static_cast<std::size_t>(input.ballScript.side)].valid) {
            ScriptedFoot3D& script = footInput.scripted[static_cast<std::size_t>(input.ballScript.side)];
            script.active = true;
            script.swing = input.ballScript.swing;
            script.positionWorld = input.ballScript.position;
            script.orientationWorld = input.ballScript.orientation;
            script.supportShift = input.ballScript.supportShift;
        }
        footInput.surfaceAt = [&](std::size_t side, Vec3 at, Vec3 forward) {
            const GroundSample ground = groundUnder(side, at, forward);
            return FootSurface3D { ground.valid, ground.height, ground.normal };
        };
        footInput.fitOnLevel = [&](Vec3 at, Vec3 forward) { return fitFoot(at, forward); };
        m_footwork.update(profile, footInput, deltaTime);
        if (footInput.seedFromBody) m_plantFeetFromBody = false;
        // Devolucao a passada: com o segundo passo no ar e o primeiro ja
        // apoiando, no ponto do balanco em que o clipe tem esse mesmo pe
        // quando o outro acaba de tocar o chao - o ciclo vai a fase desse
        // toque e os dois pes casam com o clipe (no sprint cada pe apoia so
        // ~18% do ciclo: em qualquer outro ponto um deles estaria "no ar" no
        // clipe, apoiado de fato, e era arrastado).
        if (footInput.stumbling) {
            const CharacterContactPlan3D& plan = m_footwork.plan();
            const AnimationClip3D* clip = m_previousClip;
            const auto crossings = [&](const AnimationScalarTrack3D& contact, float& liftoff, float& touchdown) {
                constexpr int Samples = 240;
                liftoff = touchdown = -1.0f;
                float previous = sampleAnimationScalarTrack3D(contact,
                    clip->durationSeconds * (Samples - 1) / Samples, clip->durationSeconds, true);
                for (int i = 0; i < Samples; ++i) {
                    const float now = sampleAnimationScalarTrack3D(contact,
                        clip->durationSeconds * i / Samples, clip->durationSeconds, true);
                    if (previous >= 0.5f && now < 0.5f) liftoff = static_cast<float>(i) / Samples;
                    if (previous < 0.5f && now >= 0.5f) touchdown = static_cast<float>(i) / Samples;
                    previous = now;
                }
            };
            bool handed = false;
            for (std::size_t side = 0; side < 2 && clip != nullptr && clip->durationSeconds > 0.0f; ++side) {
                const FootPlan3D& foot = plan.feet[side];
                const FootPlan3D& other = plan.feet[1 - side];
                const bool otherDown = other.phase == FootPhase3D::Stance || other.phase == FootPhase3D::Loading;
                if (foot.phase != FootPhase3D::Swing || !otherDown || m_footwork.stumbleStepsTaken() < 2) continue;
                float swingLift, swingTouch, stanceLift, stanceTouch;
                crossings(side == 0 ? clip->leftFootContact : clip->rightFootContact, swingLift, swingTouch);
                crossings(side == 0 ? clip->rightFootContact : clip->leftFootContact, stanceLift, stanceTouch);
                if (swingLift < 0.0f || swingTouch < 0.0f || stanceTouch < 0.0f) continue;
                const float span = std::max(0.05f, std::fmod(swingTouch - swingLift + 1.0f, 1.0f));
                const float handback = std::fmod(stanceTouch - swingLift + 1.0f, 1.0f) / span;
                const float progress = foot.swingSeconds / std::max(0.05f, foot.swingDurationSeconds);
                if (progress < handback) continue;
                const float offset = stanceAlignedPhase(*clip, 0.0f);
                m_cyclePhase = std::fmod(stanceTouch + 0.01f - offset + 2.0f, 1.0f);
                handed = true;
            }
            if (handed || m_stumbleStepSeconds > 1.0f) {
                m_stumbleStepping = false;
                m_sinceStumbleStep = 0.0f;
            }
        }
    }
    const CharacterContactPlan3D& contactPlan = m_footwork.plan();
    t.recovering = contactPlan.recovering;
    const bool& recovering = t.recovering;
    m_reactiveShare = recovering ? 1.0f : 0.0f;
    m_telemetry.reactiveShare = m_reactiveShare;
    // O estado oficial dos pes do planejador, copiado no registro que o IK,
    // o limite de torcao do quadril e o aterramento leem.
    for (std::size_t side = 0; side < m_feet.size(); ++side) {
        if (!m_footwork.owns(side)) {
            // Com a passada: nenhum passo do planejador neste pe.
            m_feet[side].repositioning = false;
            m_feet[side].recoveryStep = false;
            m_feet[side].turnStep = false;
            m_feet[side].pivotStep = false;
            continue;
        }
        FootPlant& foot = m_feet[side];
        const FootPlan3D& plan = contactPlan.feet[side];
        const bool down = plan.phase == FootPhase3D::Stance
            || plan.phase == FootPhase3D::Unloading
            || plan.phase == FootPhase3D::Loading;
        foot.planted = down;
        foot.repositioning = !down;
        foot.recoveryStep = plan.reason == StepReason3D::Recovery;
        foot.turnStep = plan.reason == StepReason3D::Turn;
        foot.pivotStep = !down && plan.path == SwingPath3D::Pivot;
        foot.stepPending = false;
        foot.releaseBlend = 0.0f;
        foot.replantDelay = 0.0f;
        foot.takeoffValid = false;
        foot.activeShift = {};
        foot.fitValid = false;
        if (down) {
            foot.lockedPositionWorld = plan.anchorWorld;
            foot.lockedOrientationWorld = plan.anchorOrientationWorld;
            if (plan.surface.valid) {
                foot.groundHeight = plan.surface.height;
                foot.groundNormal = plan.surface.normal;
            }
        } else {
            foot.repositionFromWorld = plan.liftoffWorld;
            foot.repositionFromOrientation = plan.liftoffOrientationWorld;
            foot.repositionToWorld = plan.goalWorld;
            foot.takeoffGroundHeight = plan.liftoffGroundHeight;
        }
    }
    // Peso para o pe de apoio num passo comum (pedido do planejador).
    t.standingShift = contactPlan.supportShiftWorld;
    Vec3& standingShift = t.standingShift;
    // E5 (animacao + pes conscientes): quanto a velocidade do corpo real se
    // afastou da da referencia (a capsula), em ponto de captura (/ omega). O
    // pe em balanco da passada pousa deslocado disso - a animacao continua
    // decidindo o passo; andando bem, a diferenca e quase nula. So a
    // velocidade: pela posicao tambem, com a pelve um pouco atras da capsula
    // (o atraso normal do corpo), o pe pousava atras sem desequilibrio
    // nenhum e freava o corpo (ajuda a 40%: uma queda a mais). Filtrado
    // ~50 ms, ate 15 cm.
    Vec3 balanceShift {};
    // Numa recuperacao forte, o pe no ar pode ampliar a base no sentido da
    // freada. A mola precisa ter passado de metade da reacao antes disso:
    // residuos pequenos ficam no tronco e nunca criam a passada projetada.
    const float emergencyBrakeStep = smoothStep(
        (m_brakeReaction.x - 0.55f) / 0.35f) * (1.0f - m_brakeGait);
    if (input.forceDrivenRoot && onGround && emergencyBrakeStep > 0.0f)
        balanceShift += m_brakeDirection * (0.14f * emergencyBrakeStep);
    // Recuperacao sem perder velocidade: amplia a passada no rumo do
    // movimento. Se o obstaculo pegou cedo, o proprio pe continua por cima;
    // se pegou tarde, este deslocamento permanece para o passo seguinte.
    if (input.forceDrivenRoot && onGround && m_tripRecovery > 0.0f)
        balanceShift += m_tripDirection * (0.26f * StumbleShiftScale * m_tripRecovery);
    if (input.balanceFootPlacement && input.forceDrivenRoot && onGround) {
        const float omega = std::clamp(m_estimator.state().captureOmega, 1.0f, 6.0f);
        const PhysicsBodyState3D& pelvis = state.links.front();
        const Vec3 velocityError { pelvis.linearVelocity.x - planarVelocity.x,
            pelvis.linearVelocity.y - planarVelocity.y, 0.0f };
        Vec3 wanted = velocityError * (BalancePlacementGain / omega);
        float placementLimit = BalancePlacementLimitMeters;
        float captureShare = CapturePlacement && (m_brakeGait <= 0.0f || (CaptureInBrakeRun && !m_brakeStepping))
            ? std::max(CapturePlacementAllAssist, smoothStep((0.5f - std::clamp(input.walkingAssist, 0.0f, 1.0f)) / 0.4f)) : 0.0f;
        // Experimento v2 (T_CapConsistent): o pe corrige a parte que a ajuda
        // nao corrige (1 - ajuda).
        static const float captureConsistent = tuneTemp("T_CapConsistent", 0.0f);  // TEMPTUNE
        if (captureConsistent > 0.0f && CapturePlacement && (m_brakeGait <= 0.0f || (CaptureInBrakeRun && !m_brakeStepping)))
            captureShare = std::max(captureShare, captureConsistent * (1.0f - std::clamp(input.walkingAssist, 0.0f, 1.0f)));
        // Na quase-queda, em qualquer ajuda: o pe vai ao ponto de captura,
        // mais longe que o limite comum.
        float stumbleShare = 0.0f;
        if (StumbleDetection && m_brakeGait <= 0.0f) {
            stumbleShare = std::clamp(StumblePlacement * m_tripRecovery, 0.0f, 1.0f);
            captureShare = std::max(captureShare, stumbleShare);
        }
        if (captureShare > 0.0f) {
            // Pelo ponto de captura inteiro (posicao do centro de massa mais
            // velocidade/omega) contra o da referencia da passada: com pouca
            // ajuda a capsula acompanha o corpo e so a diferenca de
            // velocidade sumia - o corpo mergulhava para a frente com o ponto
            // de captura 0,5 m a frente e o pe pousava embaixo dele. (Fora da
            // freada: ela tem o proprio pouso, e os dois brigavam.)
            const Vec3 com = m_estimator.state().centerOfMassWorld;
            const Vec3 comVelocity = m_filteredCenterOfMassVelocity;
            const Vec3 capture { com.x + comVelocity.x / omega, com.y + comVelocity.y / omega, 0.0f };
            const Vec3 referenceCapture { navigationRoot.x + planarVelocity.x / omega,
                navigationRoot.y + planarVelocity.y / omega, 0.0f };
            // Para tras (o corpo mais lento que a passada) so uma parte: o pe
            // puxado para baixo do corpo nao levava a perna a frente e o corpo
            // nao acelerava (quanto mais puxava, mais lento: 1,0 -> 1,6 m/s,
            // 0,5 -> 2,0 a 5%); de lado ao corpo, no strafe, girava a pelve.
            // Para a frente (mais rapido que a passada, caindo para a frente)
            // e atravessado, inteiro.
            Vec3 shift = (capture - referenceCapture) * CapturePlacementGain;
            const float planarSpeed = std::hypot(planarVelocity.x, planarVelocity.y);
            if (planarSpeed > 0.3f && m_gaitDirection == CharacterGaitDirection3D::Forward) {
                const Vec3 travel { planarVelocity.x / planarSpeed, planarVelocity.y / planarSpeed, 0.0f };
                float along = dot(shift, travel);
                // v2: so o atraso alem do da coleira puxa o pe. Com pouca ajuda
                // a capsula anda de proposito ~1 m/s e ~16 cm a frente do corpo
                // (physicalNavigationLead3D), e o ponto de captura lia isso como
                // corpo atrasado: o pe ia os 30 cm inteiros para tras o tempo
                // todo e as pernas corriam atras do corpo (o usuario viu a 0%).
                if (along < 0.0f && input.physicalV2 && CaptureBackDeadZone > 0.0f) {
                    const float kept = std::min(0.0f, along + CaptureBackDeadZone);
                    shift -= travel * (along - kept);
                    along = kept;
                }
                if (along < 0.0f) shift -= travel * (along * (1.0f - CapturePullBackShare));
            }
            wanted += (shift - wanted) * captureShare;
            placementLimit += (CapturePlacementLimit - placementLimit) * captureShare;
            placementLimit = std::max(placementLimit, StumblePlacementLimit * stumbleShare);
        }
        if (wanted.length() > placementLimit)
            wanted = wanted * (placementLimit / wanted.length());
        m_balancePlacementShift += (wanted - m_balancePlacementShift)
            * (1.0f - std::exp(-deltaTime / 0.05f));
        balanceShift += m_balancePlacementShift;
    } else {
        m_balancePlacementShift = {};
    }

    for (std::size_t side = 0; side < m_feet.size(); ++side) {
        FootPlant& foot = m_feet[side];
        m_telemetry.footPlanted[side] = false;
        m_telemetry.footSwingProgress[side] = -1.0f;
        m_telemetry.footClearanceMeters[side] = 0.0f;
        if (!foot.valid || !footworkEnabled) continue;
        const RagdollLinkDefinition3D& link = profile.links[foot.linkIndex];
        FootGoal& goal = goals[side];
        goal.active = true;

        // Pe animado, com o passo encurtado ou alongado ao longo da direcao
        // do ciclo, em torno do quadril.
        Vec3 footLocal = localPose.linkPositions[foot.linkIndex];
        if (gaitMoving && std::abs(strideScale - 1.0f) > 0.005f) {
            const int shin = link.parentIndex;
            const int thigh = shin > 0 ? profile.links[
                static_cast<std::size_t>(shin)].parentIndex : -1;
            if (thigh > 0) {
                const Vec3 hip = profile.links[0].modelOrientation.conjugate()
                    .rotate(profile.links[static_cast<std::size_t>(thigh)]
                        .inboundJoint.anchorModelPosition
                        - profile.links[0].modelPosition);
                const float along = dot(footLocal - hip, travelAxisLocal);
                footLocal += travelAxisLocal * ((strideScale - 1.0f) * along);
            }
        }
        const Vec3 animatedWorld = targetRootPosition
            + targetRootOrientation.rotate(footLocal);
        Quaternion animatedOrientation =
            clipFootOrientation(foot.linkIndex);
        // Na curva o pe pousa girado para dentro dela, o de dentro mais: ele
        // ajuda a virar em vez de apontar reto. Pela necessidade real - a
        // aceleracao centripeta que o corpo faz de fato -, nao pelo pedido:
        // pela curva pedida o pe ja girava ~30 graus ao comecar a virar a
        // camera, com o corpo ainda reto, e numa curva suave. Nas
        // referencias: Sprint Turn, 4,1 m/s2, o de dentro ~27 graus alem da
        // abertura da corrida reta; Running Right Turn, 3,2 m/s2, quase nada.
        // Gira durante o balanco e fica assim no apoio (a trava do pouso
        // guarda a orientacao).
        {
            const bool inner = (m_bodyCurveAcceleration > 0.0f) == (side == 0);
            const float need = smoothStep((std::abs(m_bodyCurveAcceleration) - CurveFootYawStart)
                / CurveFootYawSpan);
            const float footLead = std::copysign(CurveFootYawMaximum * need
                    * (inner ? 1.0f : CurveFootYawOuterShare), m_bodyCurveAcceleration)
                * smoothStep((speed - 1.5f) / 1.5f);
            // Freando ou parado nao ha curva: o giro sai (preso num pe, ele
            // ligava o giro da coxa nos passos de freada).
            if (!gaitMoving || m_brakeGaitActive || curveIntent < 0.5f)
                foot.curveYaw *= std::exp(-CurveFootYawRate * deltaTime);
            else if (!foot.planted)
                foot.curveYaw += (footLead - foot.curveYaw) * (1.0f - std::exp(-CurveFootYawRate * deltaTime));
            if (std::abs(foot.curveYaw) > 1e-4f)
                animatedOrientation = (Quaternion::fromAxisAngle({ 0.0f, 0.0f, 1.0f }, foot.curveYaw)
                    * animatedOrientation).normalized();
        }
        goal.animatedPosition = animatedWorld;
        goal.animatedOrientation = animatedOrientation;
        const Vec3 animatedForward = forwardOf(foot, animatedOrientation);

        if (m_footwork.owns(side)) {
            foot.previousAnimatedPositionWorld = animatedWorld;
            foot.previousAnimatedPositionValid = true;
            if (!foot.planted) {
                // No ar pelo planejador: a trajetoria dele, ate o contato.
                const FootPlan3D& plan = contactPlan.feet[side];
                goal.position = plan.targetWorld;
                goal.orientation = plan.targetOrientationWorld;
                goal.pelvisGround = plan.goalSurface.valid
                    ? std::min(plan.liftoffGroundHeight, plan.goalSurface.height)
                    : plan.liftoffGroundHeight;
                goal.reach = onGround ? 1.0f : 0.0f;
                continue;
            }
        } else {
            const float sole = footSoleOffset(link);
            // Folga do pe em relacao ao chao do CLIPE (a base da capsula): e o que
            // diz se a pose esta pisando, independente do degrau embaixo.
            const float clearance = animatedWorld.z - baseGround - sole;
            m_telemetry.footClearanceMeters[side] = clearance;
            const Vec3 animatedVelocity = foot.previousAnimatedPositionValid
                ? (animatedWorld - foot.previousAnimatedPositionWorld) / deltaTime
                : Vec3 {};
            const bool authoredStance = onGround
                && desiredState != CharacterLocomotionState3D::JumpStarting
                && contactWeights[side] > 0.55f;
            // A velocidade do pe animado escala com o ritmo (a mesma passada em
            // menos tempo); o portao tambem, para a trava valer em toda a faixa.
            const float velocityGateScale = std::max(1.0f, playbackRate);
            // Passada pelo contato: o pe que o clipe ja poe no apoio e tocou o
            // chao de verdade trava ali, mesmo com o pe animado andando - com o
            // ciclo parado esperando o toque, ele anda com o corpo, e pelo
            // portao de velocidade o pe nunca travava (sprint: nenhum apoio).
            // (TEMPTUNE TouchPlantInCurves: tambem fora da corrida estabelecida
            // - na curva o pedido gira a frente do corpo e conta como troca de
            // passada; o pe apoiado sem trava seguia o balanco e o quadril
            // estendia de -79 a +27 graus com o pe no chao, chutando o corpo
            // para cima - os saltinhos da curva. So nao na freada.)
            const bool touchedInStance = ContactPhase && input.forceDrivenRoot && gaitMoving
                && (m_contactSteady || (TouchPlantInCurves && m_brakeGait <= 0.0f && !m_brakeStepping))
                && side < physical.feet.size()
                && (physical.feet[side].touching || physical.feet[side].supporting);
            const bool stanceCandidate = authoredStance
                && clearance < 0.105f && clearance > -0.065f
                && ((std::hypot(animatedVelocity.x, animatedVelocity.y)
                    < 0.72f * velocityGateScale
                && std::abs(animatedVelocity.z) < 1.1f * velocityGateScale) || touchedInStance);
            foot.replantDelay = std::max(0.0f, foot.replantDelay - deltaTime);
            // O primeiro toque do pe fisico neste balanco (descendo, perto do
            // fim dele).
            if (!foot.planted && input.forceDrivenRoot && side < physical.feet.size()) {
                const FootSupportEstimate3D& touch = physical.feet[side];
                if (!(touch.touching || touch.supporting)) {
                    if (clearance > 0.03f) foot.firstTouchValid = false;
                } else if (!foot.firstTouchValid && LockAtFirstTouch) {
                    foot.firstTouchValid = true;
                    foot.firstTouchWorld = state.links[foot.linkIndex].position;
                }
            }
            if (!foot.planted && stanceCandidate && foot.replantDelay <= 0.0f) {
                // Encaixe de novo no toque: a previsao erra alguns centimetros
                // quando a capsula muda de velocidade (cada degrau subido a
                // freia). Ate 10 cm; alem disso o pe fica onde a pose pos.
                Vec3 touchdown = animatedWorld + foot.activeShift;
                const Vec3 correction = fitFoot(touchdown, animatedForward);
                if (correction.lengthSquared() <= 0.01f) {
                    touchdown += correction;
                    foot.activeShift += correction;
                }
                const GroundSample ground = groundUnder(side, touchdown,
                    animatedForward);
                const Quaternion orientation = (rotationBetween(
                    { 0.0f, 0.0f, 1.0f }, ground.normal) * animatedOrientation)
                        .normalized();
                foot.planted = true;
                foot.lockedOrientationWorld = orientation;
                foot.lockedPositionWorld = animatedWorld + foot.activeShift;
                // Trava onde o pe fisico tocou (como o pouso do planejador):
                // o balanco nao acompanha o recuo do pe no fim do balanco e o
                // pe tocava ate 16 cm a frente da pose - travado na pose, a
                // perna arrastava o corpo ate a diferenca (com a sola que
                // agarra, empurrando-o para cima: o "saltinho"). Ainda no ar,
                // a trava espera o toque um instante.
                foot.lockFollowTouch = 0.0f;
                if (LockAtTouch && input.forceDrivenRoot && side < physical.feet.size()
                    && (!LockTouchOnlyInChange || m_gaitChange > 0.0f)) {
                    const FootSupportEstimate3D& touch = physical.feet[side];
                    if (touch.touching || touch.supporting) {
                        // (So no mesmo nivel: tocando a quina de outro degrau
                        // vale o encaixe de cima.)
                        const Vec3 at = foot.firstTouchValid ? foot.firstTouchWorld
                            : state.links[foot.linkIndex].position;
                        const GroundSample there = groundUnder(side, at, animatedForward);
                        if (there.valid && std::abs(there.height - ground.height) < 0.03f
                            && std::hypot(at.x - foot.lockedPositionWorld.x, at.y - foot.lockedPositionWorld.y) < 0.30f) {
                            // A pose fica ate LockTouchPull do toque: a perna
                            // ainda "puxa o chao" (o pe que pousa indo para a
                            // frente nao escorrega - no sprint, 2 cm por apoio
                            // contra 12-16 travando no toque), sem a diferenca
                            // inteira que virava vara.
                            const Vec3 pull = clampMagnitude(Vec3 { foot.lockedPositionWorld.x - at.x,
                                foot.lockedPositionWorld.y - at.y, 0.0f }, LockTouchPull);
                            foot.lockedPositionWorld.x = at.x + pull.x;
                            foot.lockedPositionWorld.y = at.y + pull.y;
                        }
                    } else {
                        foot.lockFollowTouch = LockTouchWaitSeconds;
                    }
                }
                foot.firstTouchValid = false;
                foot.activeShift = {};
                foot.fitValid = false;
                // O proximo balanco sai daqui (gravado ao soltar a trava).
                foot.takeoffValid = false;
                foot.lockedPositionWorld.z = lockHeight(foot, ground.height,
                    orientation);
                foot.groundHeight = ground.height;
                foot.groundNormal = ground.normal;
                foot.plantedGoalPosition = foot.lockedPositionWorld;
                foot.plantedGoalOrientation = orientation;
            } else if (foot.planted) {
                if (foot.lockFollowTouch > 0.0f) {
                    foot.lockFollowTouch = std::max(0.0f, foot.lockFollowTouch - deltaTime);
                    const FootSupportEstimate3D& touch = physical.feet[side];
                    if (touch.touching || touch.supporting) {
                        const Vec3 at = state.links[foot.linkIndex].position;
                        const GroundSample there = groundUnder(side, at, animatedForward);
                        if (there.valid && std::abs(there.height - foot.groundHeight) < 0.03f
                            && std::hypot(at.x - foot.lockedPositionWorld.x, at.y - foot.lockedPositionWorld.y) < 0.30f) {
                            const Vec3 pull = clampMagnitude(Vec3 { foot.lockedPositionWorld.x - at.x,
                                foot.lockedPositionWorld.y - at.y, 0.0f }, LockTouchPull);
                            foot.lockedPositionWorld.x = at.x + pull.x;
                            foot.lockedPositionWorld.y = at.y + pull.y;
                            foot.plantedGoalPosition.x = foot.lockedPositionWorld.x;
                            foot.plantedGoalPosition.y = foot.lockedPositionWorld.y;
                        }
                        foot.lockFollowTouch = 0.0f;
                    }
                }
                const float horizontalDrift = std::hypot(
                    animatedWorld.x - foot.lockedPositionWorld.x,
                    animatedWorld.y - foot.lockedPositionWorld.y);
                // Fora de alcance (ver legOutOfReach), o pe solta e vira balanco.
                const bool outOfReach = legOutOfReach(foot);
                // Solta quando o clipe diz que o apoio acabou. Com folga (ate o
                // contato cair a 0,30) o pe ficava preso enquanto a pose ja
                // levava o pe embora; na soltura ele corria atras dela (medido:
                // 12 m/s no sprint de lado, com o corpo a 3,7).
                // (Parado, quem decide os pes e o planejador de passos - a
                // passada nao espera mais a marca de recuperacao dele.)
                // Arrancando, o pe que ainda carrega o corpo empurra ate
                // descarregar: o ciclo comeca com o de tras a 42% do balanco, e
                // ele saia arrastado no chao.
                const bool startHold = StartLoadHoldSeconds > 0.0f && m_startClipActive
                    && m_startClipSeconds < StartLoadHoldSeconds && input.forceDrivenRoot
                    && side < physical.feet.size() && physical.feet[side].supporting
                    && physical.feet[side].loadNewtons > StartLoadHoldShare * profile.totalMassKg * 9.81f
                    && !outOfReach;
                // Correndo, o pe que o clipe solta mas que ainda carrega o corpo
                // (sem o outro no chao) empurra mais um pouco: soltando na hora,
                // a ponta dele era arrastada no chao (na curva fechada, com o
                // corpo baixo, 1-3 m/s por 0,05-0,1 s - a "bicuda" na saida).
                const std::size_t other = 1 - side;
                const bool liftHold = LiftAfterUnloadSeconds > 0.0f && input.forceDrivenRoot && gaitMoving
                    && contactWeights[side] < 0.55f && !outOfReach && side < physical.feet.size()
                    && physical.feet[side].supporting
                    && physical.feet[side].loadNewtons > LiftAfterUnloadShare * profile.totalMassKg * 9.81f
                    && !physical.feet[other].supporting
                    && foot.liftHoldSeconds < LiftAfterUnloadSeconds;
                foot.liftHoldSeconds = liftHold ? foot.liftHoldSeconds + deltaTime
                    : contactWeights[side] >= 0.55f ? 0.0f : foot.liftHoldSeconds;
                // TEMPTUNE: correndo, o pe que pousou atrasado (ou ainda nem
                // encostou) fica no apoio ate cumprir um apoio minimo - o
                // clipe solta pelo relogio, e o pe que mal tocava (ou nem
                // tocava) ja era levado para o passo seguinte. So este pe
                // espera: o ciclo segue, a outra perna balanca normal (parar
                // o ciclo congelava a perna de tras baixa enquanto o corpo
                // andava ~0,5 m, e ela pegava a barra do tropeco).
                // Folga pela ponta da sola (o ponto mais baixo do pe animado):
                // rolando sobre a ponta no fim do apoio, o centro do pe sobe
                // varios cm com a ponta ainda no chao - pelo centro, o pe era
                // solto no meio do empurrao.
                const float soleClearance = animatedWorld.z
                    - lockHeight(foot, baseGround, animatedOrientation);
                const bool wouldRelease = contactWeights[side] < 0.55f || outOfReach
                    || (!stanceCandidate && (ReleaseBySoleClearance ? soleClearance : clearance) > 0.055f)
                    || horizontalDrift > 0.48f;
                const bool touchHold = TouchHoldSeconds > 0.0f && input.forceDrivenRoot && gaitMoving
                    && wouldRelease && !outOfReach && side < physical.feet.size()
                    && physical.feet[side].supportingSeconds < TouchMinStanceSeconds
                    && foot.touchHoldSeconds < TouchHoldSeconds
                    && m_tripRecovery <= 0.001f && !m_stumbleStepping
                    && (!TouchHoldSteadyOnly || (m_brakeGait <= 0.0f && !m_brakeStepping && !m_startClipActive
                        && m_gaitChange <= 0.0f && m_terrainSlope < 0.05f
                        && std::hypot(input.intent.requestedVelocityWorld.x,
                            input.intent.requestedVelocityWorld.y) > 1.0f));
                foot.touchHoldSeconds = touchHold ? foot.touchHoldSeconds + deltaTime
                    : !wouldRelease ? 0.0f : TouchHoldSeconds;
                // Devolvido apoiado pelo planejador: espera o clipe chegar ao
                // apoio dele (o ciclo de corrida tem voo entre a saida de um pe
                // e o pouso do outro).
                foot.handbackHoldSeconds = contactWeights[side] >= 0.55f ? 0.0f
                    : std::max(0.0f, foot.handbackHoldSeconds - deltaTime);
                const bool handbackHold = foot.handbackHoldSeconds > 0.0f && side < physical.feet.size()
                    && physical.feet[side].supporting && !outOfReach;
                const bool releaseByGait = !startHold && !liftHold && !handbackHold && !touchHold && wouldRelease;
                if (std::getenv("XHANDDBG") && (releaseByGait || !input.grounded) && (foot.handbackHoldSeconds > 0.0f
                        || (m_startClipActive && m_startClipSeconds < 0.3f)))
                    std::printf("HANDBACK release side %zu hold %.2f start %.2f contact %.2f sup %d load %.0f out %d drift %.2f cand %d clr %.3f grounded %d starthold %d\n", side,
                        foot.handbackHoldSeconds, m_startClipSeconds, contactWeights[side], physical.feet[side].supporting ? 1 : 0,
                        physical.feet[side].loadNewtons, outOfReach ? 1 : 0,
                        horizontalDrift, stanceCandidate ? 1 : 0, clearance, input.grounded ? 1 : 0, startHold ? 1 : 0);
                if (releaseByGait || !input.grounded) {
                    foot.planted = false;
                    // Solto porque a perna nao alcanca: nao trava de novo no tick
                    // seguinte. Travando e soltando a cada tick, cada trava nova
                    // puxava o alvo para a pose (pe a 6,6 m/s depois de levantar).
                    if (outOfReach) foot.replantDelay = 0.12f;
                    foot.releaseBlend = 1.0f;
                    foot.releasePositionWorld = foot.plantedGoalPosition;
                    foot.releaseOrientationWorld = foot.plantedGoalOrientation;
                    // De onde o balanco sai: altura e ponto. Sem isto ficava o de
                    // um passo antigo (medido: saida 2,5 m para tras), e o pe subia
                    // para o degrau de pouso logo no comeco do balanco.
                    foot.takeoffGroundHeight = foot.groundHeight;
                    foot.takeoffPosition = foot.plantedGoalPosition;
                    foot.takeoffValid = true;
                    foot.swingFloor = foot.groundHeight;
                }
            }
            foot.previousAnimatedPositionWorld = animatedWorld;
            foot.previousAnimatedPositionValid = true;
        }

        if (foot.planted) {
            foot.releaseBlend = 0.0f;
            // Travado, o pe ainda rola: quando o clipe ergue o calcanhar (a
            // saida) ele gira sobre a ponta, e quando pousa de calcanhar,
            // sobre o calcanhar - o ponto de apoio fica parado no mundo e o
            // rumo do pe fica o da trava. Congelado plano, o pe so rolava ao
            // soltar a trava, de uma vez (medido: chicote de 8,9 m/s no
            // strafe).
            const Vec3 normal = foot.groundNormal;
            Quaternion rolled = (rotationBetween({ 0.0f, 0.0f, 1.0f }, normal)
                * animatedOrientation).normalized();
            Vec3 lockedForward = forwardOf(foot, foot.lockedOrientationWorld);
            Vec3 rolledForward = forwardOf(foot, rolled);
            lockedForward -= normal * dot(lockedForward, normal);
            rolledForward -= normal * dot(rolledForward, normal);
            if (lockedForward.lengthSquared() > 0.0001f
                && rolledForward.lengthSquared() > 0.0001f) {
                const Vec3 a = lockedForward.normalized();
                const Vec3 b = rolledForward.normalized();
                const float yaw = std::atan2(dot(cross(b, a), normal),
                    dot(a, b));
                rolled = (Quaternion::fromAxisAngle(normal, yaw) * rolled)
                    .normalized();
            }
            const Vec3 half = ragdollColliderHasBox3D(link.collider)
                ? link.collider.boxHalfExtents : Vec3 {};
            const Vec3 toe = link.collider.localPosition
                + link.collider.localOrientation.rotate(
                    { half.x, 0.0f, -half.z });
            const Vec3 heel = link.collider.localPosition
                + link.collider.localOrientation.rotate(
                    { -half.x, 0.0f, -half.z });
            const float toeDrop = dot(rolled.rotate(toe)
                - foot.lockedOrientationWorld.rotate(toe), normal);
            const float heelDrop = dot(rolled.rotate(heel)
                - foot.lockedOrientationWorld.rotate(heel), normal);
            Vec3 position = foot.lockedPositionWorld;
            if (half.x > 0.0f && (toeDrop < -0.001f || heelDrop < -0.001f)) {
                const Vec3 pivot = toeDrop < heelDrop ? toe : heel;
                const Vec3 pivotWorld = foot.lockedPositionWorld
                    + foot.lockedOrientationWorld.rotate(pivot);
                position = pivotWorld - rolled.rotate(pivot);
            } else {
                rolled = foot.lockedOrientationWorld;
            }
            foot.plantedGoalPosition = position;
            foot.plantedGoalOrientation = rolled;
            goal.position = position;
            goal.orientation = rolled;
            goal.pelvisGround = foot.groundHeight;
            goal.reach = 1.0f;
            m_telemetry.footPlanted[side] = true;
            m_telemetry.footSecondsToTouchdown[side] = -1.0f;
            continue;
        }

        // Balanco. Chao de onde o pe saiu e de onde ele vai pisar.
        float swingGround = baseGround;
        float pelvisGround = baseGround;
        Quaternion swingOrientation = animatedOrientation;
        float lift = 0.0f;
        const FootTouchdown* next = nullptr;
        float progress = 0.0f;
        float secondsToTouchdown = 0.0f;
        float swingRise = 0.0f;
        float swingPathShare = 0.0f;
        Vec3 swingLanding;
        if (events != nullptr && onGround) {
            const float duration = std::max(0.05f,
                selectedClip->durationSeconds);
            float best = std::numeric_limits<float>::infinity();
            for (const FootTouchdown& candidate : events->touchdowns[side]) {
                const float ahead = std::fmod(candidate.touchdown
                    - legSampleTime[side] + duration * 2.0f, duration);
                if (ahead < best) {
                    best = ahead;
                    next = &candidate;
                }
            }
            // A trava vem alguns ticks depois do toque do clipe (contato
            // acima de 0,55, pe animado parado). Nesse meio tempo o pe ainda
            // esta pousando naquele toque: passar para o toque seguinte
            // jogava fora o encaixe no degrau (o pe voltava 12 cm, para a
            // quina) e a altura de pouso.
            for (const FootTouchdown& candidate : events->touchdowns[side]) {
                const float swing = std::fmod(candidate.touchdown
                    - candidate.liftoff + duration * 2.0f, duration);
                const float since = std::fmod(legSampleTime[side]
                    - candidate.touchdown + duration * 2.0f, duration);
                if (since < 0.5f * (duration - swing)) {
                    next = &candidate;
                    best = 0.0f;
                    break;
                }
            }
            if (next != nullptr && best == 0.0f) {
                progress = 1.0f;
                secondsToTouchdown = 0.0f;
            } else if (next != nullptr) {
                const float swing = std::fmod(next->touchdown - next->liftoff
                    + duration * 2.0f, duration);
                const float elapsed = std::fmod(legSampleTime[side] - next->liftoff
                    + duration * 2.0f, duration);
                progress = swing > 0.001f && elapsed <= swing
                    ? std::clamp(elapsed / swing, 0.0f, 1.0f) : 0.0f;
                secondsToTouchdown = best / std::max(0.2f, playbackRate);
            }
        }
        if (next != nullptr) {
            // Onde o pe vai pisar: a pelve la na hora do toque (velocidade da
            // capsula), mais o pe do clipe naquele instante, com o mesmo
            // encurtamento de passo.
            Vec3 footInClip = next->footInClip;
            const float along = dot(footInClip - next->hipInClip,
                Vec3 { std::cos(clipTravel), std::sin(clipTravel), 0.0f });
            footInClip += Vec3 { std::cos(clipTravel), std::sin(clipTravel),
                0.0f } * ((strideScale - 1.0f) * along);
            // TEMPTUNE CurveNarrow: na curva os dois pes pousam na mesma linha
            // para fora do corpo (o apoio passa pelo centro de massa
            // inclinado). Com a largura de passo do clipe o pe de fora caia
            // 0,4-0,6 m para fora e o de dentro quase embaixo do corpo: a
            // perna de dentro, vertical, mandava a forca toda para cima - o
            // galope na perna de dentro (70% dos apoios), os saltinhos.
            if (CurveNarrow > 0.0f && CurveLeanMaximum > 0.0f) {
                const Vec3 across { -std::sin(clipTravel), std::cos(clipTravel), 0.0f };
                const float lateral = dot(footInClip, across);
                const float share = std::min(1.0f, CurveNarrow * std::abs(m_bodyCurveLean) / CurveLeanMaximum);
                footInClip -= across * (lateral * share);
            }
            Vec3 landingWorld = navigationRoot
                + planarVelocity * secondsToTouchdown
                + heading.rotate(footInClip) + balanceShift;
            // A pelve nao estara no prolongamento reto da velocidade atual:
            // numa curva, ela percorre um arco. A previsao centripeta leva o
            // pouso para dentro da curva e ja orienta o pe pelo rumo esperado
            // no toque. Sem isso os pes continuavam correndo reto enquanto o
            // tronco girava por cima deles.
            const float arcOffset = std::clamp(0.5f
                * signedCurveAcceleration * secondsToTouchdown
                * secondsToTouchdown, -0.22f, 0.22f);
            landingWorld += heading.rotate({0.0f, arcOffset, 0.0f});
            // Inclinado na curva, a perna gira em volta do pe: ele pousa
            // para fora do quadril h sen(theta) e o quadril desce h (1 -
            // cos(theta)) (bloco da altura) - a forca do chao passa pelo
            // corpo inclinado.
            landingWorld += heading.rotate({ 0.0f,
                -CurveLeanHeight * std::sin(CurveFootShare * m_bodyCurveLean), 0.0f });
            // Chute: o pouso natural (telemetria, para o manejo planejar) e o
            // pe de apoio indo para o lado da bola, ate um limite.
            m_telemetry.footLandingWorld[side] = landingWorld;
            m_telemetry.footSecondsToTouchdown[side] = secondsToTouchdown;
            {
                Vec3 wanted;
                if (input.ballPlant.active && input.ballPlant.side == static_cast<int>(side)) {
                    const Vec3 ahead = heading.rotate({ 1.0f, 0.0f, 0.0f });
                    const Vec3 aside = heading.rotate({ 0.0f, 1.0f, 0.0f });
                    const Vec3 offset { input.ballPlant.position.x - landingWorld.x,
                        input.ballPlant.position.y - landingWorld.y, 0.0f };
                    wanted = (ahead * std::clamp(dot(offset, ahead), -0.35f, 0.30f)
                        + aside * std::clamp(dot(offset, aside), -0.25f, 0.25f))
                        * std::clamp(input.ballPlant.weight, 0.0f, 1.0f);
                }
                Vec3& plantShift = m_ballPlantShift[side];
                const float tau = wanted.lengthSquared() > 1e-6f ? 0.05f : 0.15f;
                plantShift += (wanted - plantShift) * (1.0f - std::exp(-deltaTime / tau));
                landingWorld += plantShift;
            }
            const Quaternion touchdownTurn = Quaternion::fromAxisAngle(
                {0.0f, 0.0f, 1.0f}, m_turningRate * secondsToTouchdown);
            Vec3 landingForward = touchdownTurn.rotate(
                heading.rotate(next->forwardInClip));
            // O encaixe no degrau e caro (varias sondas): refeito so quando o
            // pouso previsto anda mais de 3 cm.
            const Vec3 moved { landingWorld.x - foot.fitCenter.x,
                landingWorld.y - foot.fitCenter.y, 0.0f };
            if (!foot.fitValid || moved.lengthSquared() > 0.0009f) {
                foot.fitShift = fitFoot(landingWorld, landingForward);
                foot.fitCenter = landingWorld;
                foot.fitValid = true;
            }
            landingWorld += foot.fitShift;
            const GroundSample landing = groundUnder(side, landingWorld,
                landingForward);
            if (!foot.takeoffValid) {
                foot.takeoffGroundHeight = foot.groundHeight;
                foot.takeoffPosition = foot.releaseBlend > 0.0f
                    ? foot.releasePositionWorld : animatedWorld;
                foot.takeoffValid = true;
                foot.swingFloor = foot.groundHeight;
            }
            foot.landingGroundHeight = landing.height;
            foot.landingNormal = landing.normal;
            const float from = foot.takeoffGroundHeight;
            const float to = landing.height;
            const float rise = to - from;
            // Quanto do caminho ate o pouso o pe ja andou, na horizontal.
            // Sem a sonda de terreno, a altura do degrau novo acompanha isso:
            // subindo, chega a 70% do caminho; descendo, so depois da metade.
            // Com a sonda, quem decide e o chao a frente do pe (mais abaixo).
            const Vec3 path { landingWorld.x - foot.takeoffPosition.x,
                landingWorld.y - foot.takeoffPosition.y, 0.0f };
            foot.swingDirection = path.lengthSquared() > 0.0004f
                ? path.normalized() : Vec3 {};
            const Vec3 travelled { animatedWorld.x - foot.takeoffPosition.x,
                animatedWorld.y - foot.takeoffPosition.y, 0.0f };
            const float pathShare = path.lengthSquared() > 0.0004f
                ? std::clamp(dot(travelled, path) / path.lengthSquared(),
                    0.0f, 1.0f)
                : progress;
            const float share = rise > 0.01f
                ? smoothStep(pathShare / 0.7f)
                : smoothStep((pathShare - 0.35f) / 0.65f);
            swingGround = from + rise * share;
            lift = std::max(0.0f, rise) * 0.2f * std::sin(Pi * pathShare);
            swingRise = rise;
            swingPathShare = pathShare;
            swingLanding = landingWorld;
            goal.anticipate = smoothStep((progress - 0.3f) / 0.5f);
            // Paralelo ao chao de pouso, so com o rumo do pe do clipe.
            const Quaternion landingSurface = rotationBetween({ 0.0f, 0.0f, 1.0f },
                landing.normal);
            const Vec3 animatedAhead = animatedOrientation.rotate({ 1.0f, 0.0f, 0.0f });
            const Quaternion landingFlat = (landingSurface * Quaternion::fromAxisAngle(
                { 0.0f, 0.0f, 1.0f }, std::atan2(animatedAhead.y, animatedAhead.x))).normalized();
            goal.landingOrientation = (landingSurface * animatedOrientation).normalized();
            if (input.footSpring && input.forceDrivenRoot)
                goal.landingOrientation = slerpQuaternion(goal.landingOrientation,
                    landingFlat, FootLandingFlatten).normalized();
            goal.landingPosition = { landingWorld.x, landingWorld.y,
                lockHeight(foot, landing.height, goal.landingOrientation) };
            goal.hipTravel = planarVelocity * secondsToTouchdown;
            goal.authoredLandingHorizontal = std::hypot(
                footInClip.x - next->hipInClip.x,
                footInClip.y - next->hipInClip.y);
            goal.authoredLandingVertical = next->hipInClip.z - footInClip.z;
            pelvisGround = from + rise * progress;
            // O pe chega orientado pelo chao de destino: comeca a se alinhar
            // com ele a 30% do balanco e esta alinhado a 75% (era de 60% ate o
            // toque: numa rampa ele ainda girava ao pousar).
            const float settle = input.footSpring
                ? smoothStep((progress - 0.30f) / 0.45f)
                : smoothStep((progress - 0.6f) / 0.4f);
            swingOrientation = (slerpQuaternion(Quaternion {},
                rotationBetween({ 0.0f, 0.0f, 1.0f }, landing.normal), settle)
                * animatedOrientation).normalized();
            // Pe com mola: perto do toque ele vai ficando paralelo ao chao de
            // pouso - fica 30% da inclinacao do clipe. Pelo clipe o trote
            // pousava com a ponta 25 graus para baixo (o tornozelo atrasado
            // disfarcava), e a ponta batia na quina do meio-fio.
            if (input.footSpring && input.forceDrivenRoot)
                swingOrientation = slerpQuaternion(swingOrientation, landingFlat,
                    FootLandingFlatten * smoothStep((progress - 0.50f) / 0.40f)).normalized();
        } else if (onGround) {
            const GroundSample ground = groundUnder(side, animatedWorld,
                animatedForward);
            swingGround = ground.height;
            pelvisGround = ground.height;
            foot.takeoffValid = false;
        } else {
            foot.takeoffValid = false;
        }
        if (next == nullptr || progress >= 0.999f) foot.takeoffValid = false;
        if (next != nullptr) m_telemetry.footSwingProgress[side] = progress;
        // O pe vai sendo levado para o ponto de pouso encaixado no degrau.
        Vec3 shift = next != nullptr && foot.fitValid
            ? foot.fitShift * smoothStep(progress / 0.8f) : Vec3 {};
        // O deslocamento do equilibrio entra ao longo do passo (sem salto na
        // saida); a trava do pouso o inclui (activeShift), o pe nao escorrega.
        if (next != nullptr) shift += balanceShift * smoothStep(progress / 0.6f);
        if (next != nullptr) shift += m_ballPlantShift[side];
        // TEMPTUNE: passada pelo contato - o pe que espera o toque desce no
        // lugar: desliza para tras em relacao ao corpo na velocidade dele (no
        // chao, quase parado), como o corredor puxa o pe no fim do balanco.
        // Parado em relacao a pelve (o ciclo esperando), ele chegava ao chao a
        // ~7 m/s para a frente e cravava como vara: a velocidade para a frente
        // virava salto (sprint: de -0,3 a +1,05 m/s para cima em 25 ms, perdendo
        // 1 m/s para a frente).
        // TEMPTUNE (ContactLandOffset > 0, no lugar do anterior): o pe que
        // espera vai para uma distancia fixa a frente do centro de massa e a
        // mantem ate tocar. Ancorado no ponto de pouso planejado, quanto mais o
        // voo durava mais o corpo passava por cima dele: depois de um voo longo
        // o pe tocava embaixo do corpo (12 cm), depois de um curto, longe (33
        // cm) - e cravava e lancava o corpo, alongando o voo seguinte: o sprint
        // mancava (um passo alto, um baixo).
        {
            Vec3& paw = m_waitPawShift[side];
            if (ContactLandOffset > 0.0f && m_waitingLeg == static_cast<int>(side) && !foot.planted) {
                const Vec3 body { physical.centerOfMassVelocityWorld.x, physical.centerOfMassVelocityWorld.y, 0.0f };
                const float bodySpeed = body.length();
                if (bodySpeed > 0.5f) {
                    const Vec3 direction = body * (1.0f / bodySpeed);
                    const Vec3 target = animatedWorld + shift + paw;
                    const float ahead = dot(Vec3 { target.x - physical.centerOfMassWorld.x,
                        target.y - physical.centerOfMassWorld.y, 0.0f }, direction);
                    paw += direction * ((ContactLandOffset - ahead) * (1.0f - std::exp(-deltaTime / 0.03f)));
                }
            } else if (ContactPaw && m_waitingLeg == static_cast<int>(side) && !foot.planted) {
                const Vec3 body { physical.centerOfMassVelocityWorld.x, physical.centerOfMassVelocityWorld.y, 0.0f };
                paw -= body * deltaTime;
                if (paw.length() > ContactPawMaximum) paw = paw * (ContactPawMaximum / paw.length());
            } else if (foot.planted) {
                paw = {};
            } else {
                paw = paw * std::exp(-deltaTime / 0.08f);
            }
            shift += paw;
        }
        // Sem cruzar as pernas (com pouca ajuda, correndo para a frente ou
        // para tras): o pe em balanco nao passa para o outro lado do pe de
        // apoio. O ponto de captura empurrava o pe esquerdo para a direita do
        // direito (0,4 m), as pernas cruzavam e ele caia na arrancada. (No
        // strafe a perna de tras cruza de proposito.)
        if (next != nullptr && lowAssist > 0.0f && CrossGuardWidth > 0.0f
            && (m_gaitDirection == CharacterGaitDirection3D::Forward
                || m_gaitDirection == CharacterGaitDirection3D::Backward)
            && m_feet[1 - side].valid && m_feet[1 - side].linkIndex < state.links.size()) {
            const Vec3 left = heading.rotate({ 0.0f, 1.0f, 0.0f });
            const Vec3 other = state.links[m_feet[1 - side].linkIndex].position;
            const float outward = side == 0 ? 1.0f : -1.0f;
            const Vec3 target = animatedWorld + shift;
            const float width = outward * dot(Vec3 { target.x - other.x, target.y - other.y, 0.0f }, left);
            if (width < CrossGuardWidth) shift += left * (outward * (CrossGuardWidth - width) * lowAssist);
        }
        foot.activeShift = shift;
        Vec3 swingTarget = animatedWorld + shift;
        // Chao embaixo do pe agora (os degraus do caminho).
        float floorNow = swingGround;
        if (next != nullptr && input.groundAt) {
            // O pe passa por cima do terreno que tem pela frente, nao sobe
            // antes para o degrau de pouso. Sondas a frente da ponta, no
            // rumo do caminho ate o pouso: cada espelho de degrau vira uma
            // rampa (1,1 de inclinacao, 3 cm de folga na quina), e atras
            // do calcanhar, para ele nao raspar a borda descendo. Subir para
            // o degrau de pouso cedo (a regra de antes) levava o pe, ainda
            // atras do quadril, a 48 cm do degrau embaixo dele - o coice.
            const Vec3 direction = foot.swingDirection.lengthSquared() > 0.5f
                ? foot.swingDirection : animatedForward;
            const GroundSample under = groundUnder(side, swingTarget,
                direction);
            floorNow = under.height;
            const auto probe = [&](float reach) {
                Vec3 point = swingTarget + direction * reach;
                point.z = baseGround;
                const GroundProbeResult3D hit = input.groundAt(point);
                return hit.hasSurface && hit.walkable ? hit.pointWorld.z
                    : -std::numeric_limits<float>::infinity();
            };
            // So o terreno ate o pouso: o degrau depois dele nao conta (o pe
            // pousava pendurado 12 cm acima). Atras, so dentro do pe (o pe
            // encaixado cabe inteiro no degrau de pouso).
            constexpr float HalfFoot = 0.14f, RampSlope = 1.1f,
                NoseClearance = 0.03f;
            const float remaining = dot(swingLanding - swingTarget,
                direction);
            // Atras, so segura o pe na altura de onde ele saiu ate o
            // calcanhar passar a borda (descendo); nunca o ergue.
            float needed = std::max(under.height, std::min(probe(-0.12f),
                foot.takeoffGroundHeight));
            for (const float ahead : { 0.02f, 0.08f, 0.14f, 0.20f }) {
                if (ahead > remaining) break;
                needed = std::max(needed, probe(HalfFoot + ahead)
                    + NoseClearance - RampSlope * ahead);
            }
            // Sobe na hora (a rampa ja da a distancia), no maximo 6 m/s.
            // Desce no ritmo que chega ao chao do pouso no toque.
            if (needed >= foot.swingFloor) {
                foot.swingFloor = std::min(needed,
                    foot.swingFloor + 6.0f * deltaTime);
            } else {
                const float fallRate = std::max(1.5f,
                    (foot.swingFloor - foot.landingGroundHeight)
                        / std::max(0.06f, secondsToTouchdown));
                foot.swingFloor = std::max(needed,
                    foot.swingFloor - fallRate * deltaTime);
            }
            swingGround = foot.swingFloor;
            // Chega ao degrau de pouso a tempo, mesmo se a sonda nao o viu.
            swingGround += (foot.landingGroundHeight - swingGround)
                * smoothStep((progress - 0.8f) / 0.2f);
            lift = 0.0f;
        }
        foot.groundHeight = swingGround;
        // Passo curto, pe mais baixo: a altura que o clipe da ao pe em
        // balanco (o calcanhar subindo atras, o joelho a frente) e a de uma
        // passada inteira. Com o passo encurtado (escada, devagar) ela
        // encolhe junto, ate 40%.
        const float flatHeight = lockHeight(foot, 0.0f, animatedOrientation);
        const float verticalScale = strideScale < 1.0f
            ? 1.0f - 0.6f * (1.0f - strideScale) : 1.0f;
        const float clipHeight = animatedWorld.z - baseGround - flatHeight;
        swingTarget.z = swingGround + flatHeight
            + std::max(0.0f, clipHeight) * verticalScale
            + std::min(0.0f, clipHeight) + lift;
        // Folga minima do balanco (modo fisico): no meio do passo o pe passa
        // pelo menos ~13 cm acima do chao. O strafe da referencia ergue o pe
        // so 7-12 cm; com a perna fisica acompanhando um pouco atrasada, o pe
        // rocava o chao a passada inteira (arrastando: 1,3 s de 2,2 s). So no
        // strafe: para a frente o clipe ja ergue 17 cm, e mais alto ele
        // passava por cima de obstaculos em que devia tropecar.
        const bool strafing = m_gaitDirection == CharacterGaitDirection3D::Left
            || m_gaitDirection == CharacterGaitDirection3D::Right;
        if (input.forceDrivenRoot && strafing && next != nullptr && onGround) {
            // Tira do chao cedo (ate ~30% do passo), segura e so desce no fim:
            // num arco em seno o pe saia devagar, ja deslizando de lado.
            const float p = std::clamp(progress, 0.0f, 1.0f);
            const float arch = smoothStep(p / 0.30f)
                * (1.0f - smoothStep((p - 0.65f) / 0.35f));
            const float clearance = swingTarget.z - (swingGround + flatHeight);
            const float wanted = SwingMinimumClearance * arch;
            if (clearance < wanted) swingTarget.z += wanted - clearance;
        }
        // Trote frontal: recolhe a ponta durante o meio do balanco. A altura
        // adicional e pequena e progressiva; evita que o atraso fisico do
        // tornozelo transforme a passada em uma bicuda no piso. Num tropeco
        // cedo, a estrategia de elevacao ergue o mesmo pe e continua o passo.
        // No sprint em curva tambem: o corpo inclinado e mais baixo, e a ponta
        // do pe de dentro raspava o chao saindo do apoio (~1,6 vez por curva
        // fechada).
        const float sprintCurveClear = SprintCurveClearStart > 0.0f && input.sprinting
            ? smoothStep((std::abs(m_bodyCurveAcceleration) - SprintCurveClearStart) / SprintCurveClearSpan) : 0.0f;
        const bool forwardJog = input.forceDrivenRoot && next != nullptr
            && onGround && m_gaitDirection == CharacterGaitDirection3D::Forward
            && (!input.sprinting || sprintCurveClear > 0.0f);
        if (forwardJog) {
            const float p = std::clamp(progress, 0.0f, 1.0f);
            const float curveShare = input.sprinting ? sprintCurveClear : 1.0f;
            const float early = std::abs(m_bodyCurveAcceleration) > 2.0f ? CurveClearEarly : 0.24f;
            const float arch = smoothStep(p / early)
                * (1.0f - smoothStep((p - 0.66f) / 0.27f)) * curveShare;
            const float clearance = swingTarget.z - (swingGround + flatHeight);
            float wanted = 0.18f * arch;
            if (m_tripElevating && m_tripFoot == static_cast<int>(side))
                wanted += 0.10f * m_tripRecovery
                    * (1.0f - smoothStep((p - 0.72f) / 0.20f));
            if (clearance < wanted) swingTarget.z += wanted - clearance;
            // Dorsiflexao: ponta aproximadamente 7 graus acima no meio do
            // balanco, voltando gradualmente ao plano do pouso.
            const Vec3 lateral = swingOrientation.rotate({ 0.0f, 1.0f, 0.0f });
            swingOrientation = (Quaternion::fromAxisAngle(lateral,
                -0.12f * arch) * swingOrientation).normalized();
        }
        // Com pouca ajuda o pe fisico fica para tras do alvo (a perna nao
        // acompanha o corpo que vem rapido, e na freada o alvo salta para a
        // frente): pelo tempo do clipe ele ja descia e tocava o chao 0,3-0,5 m
        // antes do pouso, atras do corpo - e o corpo tropecava nele. Enquanto
        // falta caminho, o alvo fica no alto: o pe "tenta" chegar e so desce
        // perto de onde vai pousar.
        if (ReachLiftPerMeter > 0.0f && input.forceDrivenRoot && next != nullptr && onGround
            && foot.linkIndex < state.links.size() && m_gaitDirection == CharacterGaitDirection3D::Forward) {
            const float lowAssistReach = smoothStep((0.5f - std::clamp(input.walkingAssist, 0.0f, 1.0f)) / 0.4f);
            const Vec3 physicalFoot = state.links[foot.linkIndex].position;
            const Vec3 direction = foot.swingDirection.lengthSquared() > 0.5f
                ? foot.swingDirection : animatedForward;
            const float behind = dot(Vec3 { swingTarget.x - physicalFoot.x, swingTarget.y - physicalFoot.y, 0.0f },
                Vec3 { direction.x, direction.y, 0.0f });
            const float wanted = std::min(ReachLiftMaximum,
                ReachLiftPerMeter * std::max(0.0f, behind - ReachLiftStart)) * lowAssistReach
                * (ReachLiftBrakeOnly ? m_brakeGait : 1.0f);
            const float clearance = swingTarget.z - (swingGround + flatHeight);
            if (wanted > 0.0f && clearance < wanted) swingTarget.z += wanted - clearance;
        }
        // Nenhum pe do alvo abaixo do chao que esta passando por baixo dele.
        const float lowestAllowed = lockHeight(foot, floorNow,
            swingOrientation) + (floorNow > swingGround + 0.01f ? 0.02f : 0.0f);
        swingTarget.z = std::max(swingTarget.z, lowestAllowed);
        if (foot.releaseBlend > 0.0f) {
            // A trava nao some num tick: o pe volta para o balanco em ~0,12 s.
            constexpr float ReleaseSeconds = 0.12f;
            foot.releaseBlend = std::max(0.0f,
                foot.releaseBlend - deltaTime / ReleaseSeconds);
            const float hold = smoothStep(foot.releaseBlend);
            swingTarget = swingTarget
                + (foot.releasePositionWorld - swingTarget) * hold;
            swingOrientation = slerpQuaternion(swingOrientation,
                foot.releaseOrientationWorld, hold);
        }
        goal.position = swingTarget;
        goal.orientation = swingOrientation;
        m_telemetry.footClipPitch[side] = -std::asin(std::clamp(
            animatedOrientation.rotate({ 1.0f, 0.0f, 0.0f }).z, -1.0f, 1.0f)) * 57.2958f;
        m_telemetry.footGoalPitch[side] = -std::asin(std::clamp(
            swingOrientation.rotate({ 1.0f, 0.0f, 0.0f }).z, -1.0f, 1.0f)) * 57.2958f;
        goal.pelvisGround = pelvisGround;
        // TEMPTUNE: correndo no plano (o pouso na altura de onde o pe saiu),
        // o pe no ar baixa a pelve no maximo SwingReachFlatCap. Na curva o
        // pouso mira longe para fora, e a pelve alvo descia ate 12 cm no voo
        // para a perna alcanca-lo; no apoio seguinte a perna levava o corpo
        // de volta para cima e ele decolava (saltinhos da curva). A regra e
        // para alcancar degrau mais baixo - em degrau ela continua inteira.
        if (SwingReachFlatCap >= 0.0f && next != nullptr && gaitMoving
            && std::abs(swingGround - baseGround) < 0.02f && std::abs(pelvisGround - baseGround) < 0.02f)
            goal.dropCap = SwingReachFlatCap;
        goal.reach = !onGround ? 0.0f
            : next != nullptr ? smoothStep((progress - 0.4f) / 0.5f)
            : 1.0f;
    }

}

} // namespace MatterEngine
