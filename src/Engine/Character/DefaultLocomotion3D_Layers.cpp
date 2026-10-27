#include "Engine/Character/DefaultLocomotion3DInternal.hpp"

namespace MatterEngine {

void DefaultLocomotion3D::stepLayers(Tick& t) {
    const RagdollProfile3D& profile = t.profile;
    const CharacterLocomotionAnimations3D& animations = t.animations;
    const DefaultLocomotionInput3D& input = t.input;
    const DefaultLocomotionSettings3D& s = m_settings;
    const RagdollLinkDefinition3D& rootLink = *t.rootLink;
    const float& dt = t.dt;
    const std::size_t& links = t.links;
    const float& viewYaw = t.viewYaw;
    const float& speed = t.speed;
    float& upperIdleWeight = t.upperIdleWeight;
    const Quaternion& yaw = t.yaw;
    const Vec3& forward = t.forward;
    const Vec3& left = t.left;
    float& landTrunk = t.landTrunk;
    float& landUpper = t.landUpper;
    float& landHead = t.landHead;
    Vec3& rootOffset = t.rootOffset;
    const float& ground = t.ground;
    Quaternion& clipRootRotation = t.clipRootRotation;
    const float& curveShare = t.curveShare;
    // "Vida": passada a passada nos bracos, tronco e cabeca; parado, a
    // respiracao e o acomodar. Cada ciclo com o seu peso (o do corpo de cima)
    // e o parado com o dele, somados - so do dominante, o tronco e os bracos
    // saltavam quando ele mudava ou o corpo passava de meio parado.
    if (input.motionVariety > 0.0f) {
        const float amount = std::clamp(input.motionVariety, 0.0f, 2.0f);
        const float strides = m_strides + m_phase;
        std::vector<Vec3> delta(links);
        const auto add = [&](float weight) {
            for (std::size_t i = 1; i < links; ++i) m_coordinates[i] += delta[i] * weight;
        };
        for (std::size_t g = 0; g < GaitCount; ++g) {
            const float weight = m_upperWeights[g] * ground;
            if (weight <= 1e-3f || m_gaits[g].clip == nullptr) continue;
            std::fill(delta.begin(), delta.end(), Vec3 {});
            const float clipPhase = fraction(m_phase + m_gaits[g].touchdown);
            characterGaitVariety3D(profile, *m_gaits[g].clip, clipPhase, clipPhase, strides, input.varietySeed,
                amount, delta);
            add(weight);
        }
        if (upperIdleWeight * ground > 1e-3f) {
            std::fill(delta.begin(), delta.end(), Vec3 {});
            characterIdleVariety3D(profile, m_idleTime, input.varietySeed, amount, delta);
            add(upperIdleWeight * ground);
        }
    }

    // Vida do tronco e da cabeca: cada marcha da frente e de costas com a sua
    // classe (trote/sprint), pelo peso dela - a altura pelo das pernas, o
    // tronco e a cabeca pelo do corpo de cima.
    if (s.trunkMotion && s.trunkMotionAmount > 0.0f && ground > 0.0f) {
        const float twice = 4.0f * Pi * m_phase, once = 2.0f * Pi * m_phase;
        float pitch = 0.0f, headPitch = 0.0f, headRoll = 0.0f;
        m_trunkHeight = 0.0f;
        for (std::size_t g = 0; g < GaitCount; ++g) {
            if (m_gaits[g].clip == nullptr) continue;
            const float c = std::cos(m_gaits[g].heading);
            if (std::abs(c) < 0.7f) continue;
            const DefaultTrunkMotion3D& m = s.trunkMotions[(g >= Directions ? 1u : 0u) + (c < 0.0f ? 2u : 0u)];
            const float legs = m_weights[g], upper = m_upperWeights[g];
            m_trunkHeight += legs * (m.heightCos2 * std::cos(twice) + m.heightSin2 * std::sin(twice));
            pitch += upper * (m.pitchMean + m.pitchCos2 * std::cos(twice) + m.pitchSin2 * std::sin(twice));
            headPitch += upper * (m.headPitchMean + m.headPitchCos2 * std::cos(twice)
                + m.headPitchSin2 * std::sin(twice));
            headRoll += upper * (m.headRollCos1 * std::cos(once) + m.headRollSin1 * std::sin(once));
        }
        const float amount = s.trunkMotionAmount * ground;
        m_trunkHeight *= amount;
        addCoordinate(profile, m_coordinates, "Abdomen", 1, pitch * amount * 0.30f);
        addCoordinate(profile, m_coordinates, "Chest", 1, pitch * amount * 0.35f);
        addCoordinate(profile, m_coordinates, "UpperChest", 1, pitch * amount * 0.35f);
        addCoordinate(profile, m_coordinates, "Neck", 1, headPitch * amount * 0.40f);
        addCoordinate(profile, m_coordinates, "Head", 1, headPitch * amount * 0.60f);
        addCoordinate(profile, m_coordinates, "Neck", 2, headRoll * amount * 0.40f);
        addCoordinate(profile, m_coordinates, "Head", 2, headRoll * amount * 0.60f);
    } else {
        m_trunkHeight = 0.0f;
    }

    // O olhar: a coluna devolve a maior parte do giro da pelve para a camera,
    // a cabeca completa (ela mais rapido). A inclinacao do olhar no peito,
    // no pescoco e na cabeca. Parado, o pescoco e a cabeca eretos.
    {
        const float sprinting = frontSector(m_sector) ? smoothStep((speed - 5.0f) / 1.5f) : 0.0f;
        const float torsoLimit = radians(s.torsoTwistDegrees
            + (s.sprintTorsoTwistDegrees - s.torsoTwistDegrees) * sprinting);
        const float headLimit = radians(s.headTurnDegrees);
        const float error = std::clamp(wrapAngle(viewYaw - m_yaw), -(torsoLimit + headLimit), torsoLimit + headLimit);
        const float torsoTarget = std::clamp(error * s.torsoLookShare, -torsoLimit, torsoLimit);
        if (s.adaptiveTransitions)
            spring(m_torsoTwist, m_torsoTwistRate, torsoTarget, s.torsoLookSeconds, dt);
        else
            m_torsoTwist += (torsoTarget - m_torsoTwist) * (1.0f - std::exp(-s.torsoLookRate * dt));
        const float headTarget = std::clamp(error - m_torsoTwist, -headLimit, headLimit);
        if (s.adaptiveTransitions)
            spring(m_headTurn, m_headTurnRate, headTarget, s.headLookSeconds, dt);
        else
            m_headTurn += (headTarget - m_headTurn) * (1.0f - std::exp(-s.headLookRate * dt));
        addCoordinate(profile, m_coordinates, "Abdomen", 0, m_torsoTwist * 0.30f);
        addCoordinate(profile, m_coordinates, "Chest", 0, m_torsoTwist * 0.35f);
        addCoordinate(profile, m_coordinates, "UpperChest", 0, m_torsoTwist * 0.35f);
        addCoordinate(profile, m_coordinates, "Neck", 0, m_headTurn * 0.40f);
        addCoordinate(profile, m_coordinates, "Head", 0, m_headTurn * 0.60f);
        const float pitch = input.lookPitchRadians;
        addCoordinate(profile, m_coordinates, "UpperChest", 1, -pitch * 0.14f);
        addCoordinate(profile, m_coordinates, "Neck", 1, -pitch * 0.36f);
        addCoordinate(profile, m_coordinates, "Head", 1, -pitch * 0.56f);
        m_upright += ((1.0f - m_moving) * ground - m_upright) * (1.0f - std::exp(-dt / 0.3f));
        addCoordinate(profile, m_coordinates, "Neck", 1, -0.20f * m_upright);
        addCoordinate(profile, m_coordinates, "Head", 1, -0.07f * m_upright);
        addCoordinate(profile, m_coordinates, "UpperChest", 1, -0.08f * m_upright);
        addCoordinate(profile, m_coordinates, "Chest", 1, -0.04f * m_upright);
    }
    if (landTrunk > 0.0f || landUpper > 0.0f) {
        // A aterrissagem: a coluna flexiona (baixa e alta) e a cabeca volta
        // um pouco.
        addCoordinate(profile, m_coordinates, "Abdomen", 1, landTrunk * 0.5f);
        addCoordinate(profile, m_coordinates, "Chest", 1, landTrunk * 0.5f);
        addCoordinate(profile, m_coordinates, "UpperChest", 1, landUpper);
        addCoordinate(profile, m_coordinates, "Neck", 1, -landHead * 0.5f);
        addCoordinate(profile, m_coordinates, "Head", 1, -landHead * 0.5f);
    }

    // Inclinacao: na curva o corpo inteiro vai para dentro (atan(v w / g),
    // pela velocidade); arrancada e freada um pouco para a frente e para
    // tras. Em volta dos pes.
    const float lateralAcceleration = speed * m_pathTurnRate;
    const float leaningAcceleration = std::copysign(std::max(0.0f, std::abs(lateralAcceleration) - s.curveLeanDeadZone),
        lateralAcceleration);
    const float bodyCurveLean = std::clamp(std::atan(leaningAcceleration / Gravity) * curveShare,
        -s.curveLeanMaximum, s.curveLeanMaximum) * ground;
    {
        const float forwardAcceleration = dot(m_acceleration, forward);
        Vec3 wantedLean { bodyCurveLean,
            std::clamp(std::atan(forwardAcceleration / Gravity) * s.forwardLeanShare,
                s.forwardLeanMinimum, s.forwardLeanMaximum) * ground, 0.0f };
        const float omega = s.leanRate;
        const float decay = std::exp(-omega * dt);
        for (int axis = 0; axis < 2; ++axis) {
            float& value = axis == 0 ? m_lean.x : m_lean.y;
            float& rateValue = axis == 0 ? m_leanRate.x : m_leanRate.y;
            const float target = axis == 0 ? wantedLean.x : wantedLean.y;
            const float error = value - target;
            const float impulse = rateValue + error * omega;
            value = target + (error + impulse * dt) * decay;
            rateValue = (rateValue - impulse * omega * dt) * decay;
        }
    }

    // A raiz no mundo: a base da capsula mais a altura da pelve em pe (a
    // origem do modelo e a pelve), girada pelo rumo, mais o que o clipe move
    // a pelve; descida do terreno; inclinacao em volta dos pes.
    t.feet = input.feetPositionWorld;
    const Vec3& feet = t.feet;
    {
        // O degrau da capsula: no chao, a base salta de uma vez (mais que 1 cm
        // num tick - a rampa anda menos que isso); a pelve segue o salto numa
        // mola critica de ~0,15 s.
        if (m_previousBaseKnown && input.grounded) {
            const float jump = feet.z - m_previousBase;
            if (std::abs(jump) > 0.01f && std::abs(jump) < 0.6f) m_stepOffset += jump;
        } else if (!input.grounded) {
            m_stepOffset = 0.0f;
            m_stepOffsetRate = 0.0f;
        }
        m_previousBase = feet.z;
        m_previousBaseKnown = true;
        constexpr float StepSeconds = 0.15f;
        const float omega = 4.0f / StepSeconds;
        const float slope = m_stepOffsetRate + m_stepOffset * omega;
        const float decay = std::exp(-omega * dt);
        m_stepOffset = (m_stepOffset + slope * dt) * decay;
        m_stepOffsetRate = (m_stepOffsetRate - slope * omega * dt) * decay;
    }
    t.rootOrientation = (yaw * rootLink.modelOrientation * clipRootRotation).normalized();
    Quaternion& rootOrientation = t.rootOrientation;
    t.rootPosition = feet + Vec3 { 0.0f, 0.0f, profile.standingRootHeightMeters - m_terrainDrop - m_stepOffset + m_dip
        + m_trunkHeight - (m_landTime >= 0.0f ? m_landPrepCarry : s.landingPrep ? m_landingPrep * s.landingPelvisDrop : 0.0f)
        - m_landingSink }
        + yaw.rotate(rootLink.modelPosition + rootOffset);
    Vec3& rootPosition = t.rootPosition;
    {
        const Vec3 axis = left * m_lean.y - forward * m_lean.x;
        const float angle = axis.length();
        if (angle > 1e-5f) {
            const Quaternion tilt = Quaternion::fromAxisAngle(axis * (1.0f / angle), angle);
            rootPosition = feet + tilt.rotate(rootPosition - feet);
            rootOrientation = (tilt * rootOrientation).normalized();
        }
    }
    // Preparo do pulo: o clipe dobra os joelhos sem descer a raiz - os pes
    // subiam ~20 cm e ele ficava agachado no ar (visto pelo usuario, 05/10
    // noite). A pelve desce ate o pe mais baixo tocar o chao - tambem na
    // saida do chao, enquanto o preparo se desfaz (as pernas estendem no
    // impulso; cortando no primeiro tick no ar a pelve saltava 20 cm).
    if (m_crouch > 0.001f && animations.jumpPrep != nullptr && m_feet[0] != 0 && m_feet[1] != 0) {
        RagdollAnimationPose3D crouched;
        defaultLocomotionPose3D(profile, rootPosition, rootOrientation, m_coordinates, crouched);
        float lowest = 1e9f;
        for (std::size_t side = 0; side < 2; ++side)
            lowest = std::min(lowest, soleHeight(profile.links[m_feet[side]], crouched.linkPositions[m_feet[side]],
                crouched.linkOrientations[m_feet[side]]));
        rootPosition.z -= std::max(0.0f, lowest - feet.z) * smoothStep(m_crouch / 0.15f);
    }

    t.gestureFeet = {};
    stepGesture(t);
    t.rootInverse = rootOrientation.conjugate();
    const Quaternion& rootInverse = t.rootInverse;

}

} // namespace MatterEngine
