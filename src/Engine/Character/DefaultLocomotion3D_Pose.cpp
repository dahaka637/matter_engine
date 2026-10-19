#include "Engine/Character/DefaultLocomotion3DInternal.hpp"

namespace MatterEngine {

void DefaultLocomotion3D::stepPose(Tick& t) {
    const RagdollProfile3D& profile = t.profile;
    const CharacterLocomotionAnimations3D& animations = t.animations;
    const DefaultLocomotionInput3D& input = t.input;
    const DefaultLocomotionSettings3D& s = m_settings;
    const float& dt = t.dt;
    const std::size_t& links = t.links;
    const Vec3& velocity = t.velocity;
    const float& speed = t.speed;
    std::array<float, GaitCount>& wanted = t.wanted;
    float& upperIdleWeight = t.upperIdleWeight;
    const Vec3& forward = t.forward;
    const auto gaitAmplitude = [this](std::size_t g) { return gaitAmplitudeOf(g); };
    // A pose: as juntas e a raiz de cada clipe, com os pesos.
    m_coordinates.assign(links, {});
    t.rootOffset = {};
    Vec3& rootOffset = t.rootOffset;
    QuaternionSum rootRotation;
    t.contact = {};
    std::array<float, 2>& contact = t.contact;
    const auto sampleInto = [&](const AnimationClip3D& clip, float time, bool loop, float weight,
        float upperWeight, const Gait* gait = nullptr, float amplitude = 1.0f, bool rootAndContact = true) {
        if (weight <= 1e-4f && upperWeight <= 1e-4f) return;
        for (std::size_t i = 1; i < links; ++i) {
            const float w = i < m_upperLinks.size() && m_upperLinks[i] ? upperWeight : weight;
            if (w <= 1e-4f) continue;
            if (const AnimationTrack3D* track = findAnimationTrack3D(clip, profile.links[i].id)) {
                Vec3 value = sampleTrack(*track, time, clip.durationSeconds, loop, s.smoothClipSampling).jointPositionRadians;
                // O trote mais amplo em volta da media do ciclo, cabendo nos
                // limites da junta: num eixo que passaria do limite, a mesma
                // escala em volta de um centro deslocado (o movimento cresce
                // para o lado com folga - o quadril, que estende so ate 30
                // graus, flexiona mais a frente); se nem assim cabe, a escala
                // que cabe. Cortado no limite, o movimento fazia uma quina
                // (picos de ~700 m/s2 no pe, contra ~300).
                if (gait != nullptr && amplitude != 1.0f && i < gait->mean.size() && i < m_amplitudeShare.size()
                    && m_amplitudeShare[i] != 0.0f) {
                    const float share = m_amplitudeShare[i] > 0.0f ? m_amplitudeShare[i] : s.gaitAmplitudeSpine;
                    const float wanted = 1.0f + (amplitude - 1.0f) * share;
                    const auto& axes = profile.links[i].inboundJoint.axes;
                    const float means[3] = { gait->mean[i].x, gait->mean[i].y, gait->mean[i].z };
                    const float lows[3] = { gait->minimum[i].x, gait->minimum[i].y, gait->minimum[i].z };
                    const float highs[3] = { gait->maximum[i].x, gait->maximum[i].y, gait->maximum[i].z };
                    float* parts[3] = { &value.x, &value.y, &value.z };
                    constexpr float Margin = 0.03f;
                    for (std::size_t a = 0; a < 3; ++a) {
                        float center = means[a], scale = wanted;
                        if (axes[a].enabled && axes[a].maximumRadians - axes[a].minimumRadians > 2.0f * Margin
                            && scale > 1.0f + 1e-4f) {  // (encolhendo, sempre cabe)
                            const float lo = axes[a].minimumRadians + Margin, hi = axes[a].maximumRadians - Margin;
                            const float top = center + (highs[a] - center) * scale;
                            const float bottom = center + (lows[a] - center) * scale;
                            if (top > hi && bottom < lo) {
                                scale = std::max(1.0f, (hi - lo) / std::max(1e-4f, highs[a] - lows[a]));
                                center = scale > 1.0f + 1e-4f ? (scale * lows[a] - lo) / (scale - 1.0f) : center;
                            } else if (top > hi) {
                                center = (scale * highs[a] - hi) / (scale - 1.0f);
                            } else if (bottom < lo) {
                                center = (scale * lows[a] - lo) / (scale - 1.0f);
                            }
                        }
                        *parts[a] = center + (*parts[a] - center) * scale;
                    }
                }
                m_coordinates[i] += value * w;
            }
        }
        if (weight <= 1e-4f || !rootAndContact) return;
        const AnimationTransformSample3D root = rootAt(profile, clip, time, loop, s.smoothClipSampling);
        rootOffset += root.translationOffsetMeters * weight;
        rootRotation.add(root.rotationDelta.normalized(), weight);
        contact[0] += (clip.leftFootContact.keyframes.empty() ? 1.0f
            : sampleAnimationScalarTrack3D(clip.leftFootContact, time, clip.durationSeconds, loop)) * weight;
        contact[1] += (clip.rightFootContact.keyframes.empty() ? 1.0f
            : sampleAnimationScalarTrack3D(clip.rightFootContact, time, clip.durationSeconds, loop)) * weight;
    };
    t.ground = 1.0f - m_airWeight;
    const float& ground = t.ground;
    for (std::size_t g = 0; g < GaitCount; ++g) {
        if (m_gaits[g].clip == nullptr) continue;
        const AnimationClip3D& clip = *m_gaits[g].clip;
        sampleInto(clip, fraction(m_phase + m_gaits[g].touchdown) * clip.durationSeconds, true, m_weights[g] * ground,
            m_upperWeights[g] * ground, &m_gaits[g], gaitAmplitude(g));
    }
    // O giro parado pelo clipe: as pernas dele no lugar das do idle, com a raiz dele (o deslocamento, feito no rumo
    // do comeco do passo, trazido ao de agora, e a rotacao sem o giro, que ja e o rumo da pelve) e os contatos.
    const float turnShare = m_turnClip != nullptr ? m_turnWeight : 0.0f;
    if (animations.idle != nullptr)
        sampleInto(*animations.idle, std::fmod(m_idleTime, std::max(0.1f, animations.idle->durationSeconds)), true,
            m_idleWeight * ground * (1.0f - turnShare), upperIdleWeight * ground);
    if (turnShare > 1e-4f) {
        const AnimationClip3D& clip = *m_turnClip;
        const float w = m_idleWeight * ground * turnShare;
        sampleInto(clip, m_turnTime, false, w, 0.0f, nullptr, 1.0f, false);
        const AnimationTransformSample3D root = rootAt(profile, clip, m_turnTime, false, s.smoothClipSampling);
        const float back = -wrapAngle(m_yaw - m_turnStartYaw), c = std::cos(back), sn = std::sin(back);
        const Vec3 offset = root.translationOffsetMeters;
        rootOffset += Vec3 { offset.x * c - offset.y * sn, offset.x * sn + offset.y * c, offset.z } * w;
        // (A rotacao da raiz dele sem o giro: sem ela, com o idle fora, a raiz saltava para a do bind.)
        const Quaternion bind = profile.links.front().modelOrientation;
        rootRotation.add((bind.conjugate() * Quaternion::fromAxisAngle({ 0.0f, 0.0f, 1.0f }, back) * bind
            * root.rotationDelta.normalized()).normalized(), w);
        contact[0] += (clip.leftFootContact.keyframes.empty() ? 1.0f
            : sampleAnimationScalarTrack3D(clip.leftFootContact, m_turnTime, clip.durationSeconds, false)) * w;
        contact[1] += (clip.rightFootContact.keyframes.empty() ? 1.0f
            : sampleAnimationScalarTrack3D(clip.rightFootContact, m_turnTime, clip.durationSeconds, false)) * w;
    }
    if (m_airClip != nullptr && m_airWeight > 1e-4f)
        sampleInto(*m_airClip, m_airPhase * m_airClip->durationSeconds, false, m_airWeight, m_airWeight);
    t.clipRootRotation = rootRotation.result();
    Quaternion& clipRootRotation = t.clipRootRotation;

    // O gesto da arrancada (o idle_to_sprint como delta no tronco, nos bracos
    // e na cabeca, por cima do ciclo - o mesmo do fisico).
    if (m_startTime >= 0.0f && animations.idleToSprint != nullptr) {
        const AnimationClip3D& start = *animations.idleToSprint;
        const float duration = m_startSprint ? 0.62f : 0.34f;
        if (m_startTime < duration) {
            const float u = std::clamp(m_startTime / duration, 0.0f, 1.0f);
            const float envelope = smoothStep(u / 0.20f) * (1.0f - smoothStep((u - 0.62f) / 0.38f)) * ground;
            const float sourceTime = std::min(m_startTime * (m_startSprint ? 1.0f : 0.75f), start.durationSeconds);
            const float limit = m_startSprint ? 0.42f : 0.20f;
            for (std::size_t i = 1; i < links; ++i) {
                const std::string& id = profile.links[i].id;
                const bool spine = id == "Abdomen" || id == "Chest" || id == "UpperChest";
                const bool arm = id.find("UpperArm") != std::string::npos || id.find("Forearm") != std::string::npos;
                const bool head = id == "Neck" || id == "Head";
                const float weight = envelope * s.startGestureAmount * (spine ? (m_startSprint ? 0.55f : 0.28f)
                    : arm ? (m_startSprint ? 0.62f : 0.30f) : head ? (m_startSprint ? 0.25f : 0.12f) : 0.0f);
                if (weight <= 0.0f) continue;
                const AnimationTrack3D* track = findAnimationTrack3D(start, id);
                if (track == nullptr) continue;
                const Vec3 first = sampleTrack(*track, 0.0f, start.durationSeconds, false, s.smoothClipSampling).jointPositionRadians;
                const Vec3 now = sampleTrack(*track, sourceTime, start.durationSeconds, false, s.smoothClipSampling)
                    .jointPositionRadians;
                const Vec3 delta { std::clamp(now.x - first.x, -limit, limit), std::clamp(now.y - first.y, -limit, limit),
                    std::clamp(now.z - first.z, -limit, limit) };
                m_coordinates[i] += delta * weight;
            }
        }
        m_startTime += dt;
    }

    // Curva: a corrida em arco na coluna (o banco e a torcao; a flexao fica a
    // da marcha) e um pouco nas pernas.
    const float longitudinal = dot(velocity, forward);
    t.curveAcceleration = longitudinal * m_pathTurnRate;
    const float& curveAcceleration = t.curveAcceleration;
    // Quanto da curva vale pela velocidade: nada devagar, pouco no trote,
    // inteira correndo rapido.
    t.curveShare = smoothStep((speed - 1.0f) / 2.0f)
        * (s.curveTrotShare + (1.0f - s.curveTrotShare)
            * smoothStep((speed - s.curveSprintFrom) / std::max(0.1f, s.curveSprintSpan)));
    const float& curveShare = t.curveShare;
    {
        const float desired = std::clamp(smoothStep((std::abs(curveAcceleration) - 0.35f - s.curveLeanDeadZone) / 3.5f),
            0.0f, 1.0f) * s.curveClipMaximum * curveShare * ground;
        int kind = -1;
        if (desired > 0.001f && m_moving > 0.5f) {
            if (frontSector(m_sector)) kind = curveAcceleration > 0.0f ? 0 : 1;
            else if (m_sector >= 3 && m_sector <= 5 && curveAcceleration < 0.0f) kind = 2;
            if (kind >= 0 && m_arcs[static_cast<std::size_t>(kind)] == nullptr) kind = -1;
        }
        const bool changing = kind != m_arcKind;
        const float target = changing ? 0.0f : desired;
        const float limit = (target > m_arcBlend ? 2.5f : 4.0f) * dt;
        m_arcBlend += std::clamp(target - m_arcBlend, -limit, limit);
        if (changing && m_arcBlend <= 0.001f) m_arcKind = kind;
        if (m_arcKind >= 0 && m_arcBlend > 0.0f) {
            const AnimationClip3D& arc = *m_arcs[static_cast<std::size_t>(m_arcKind)];
            const float time = fraction(m_phase + m_arcTouchdown[static_cast<std::size_t>(m_arcKind)]) * arc.durationSeconds;
            for (std::size_t i = 1; i < links; ++i) {
                const std::string& id = profile.links[i].id;
                const bool spine = id == "Abdomen" || id == "Chest" || id == "UpperChest";
                const bool leg = id.find("Thigh") != std::string::npos || id.find("Shin") != std::string::npos
                    || id.find("Foot") != std::string::npos;
                if (!spine && !leg) continue;
                const AnimationTrack3D* track = findAnimationTrack3D(arc, id);
                if (track == nullptr) continue;
                Vec3 pose = sampleTrack(*track, time, arc.durationSeconds, true, s.smoothClipSampling).jointPositionRadians;
                if (spine) pose.y = m_coordinates[i].y;
                m_coordinates[i] += (pose - m_coordinates[i]) * (m_arcBlend * (spine ? 1.0f : s.curveClipLegShare));
            }
        }
    }

    // Carga do pulo: o agachar do preparo, mais fundo com a carga.
    {
        const float wantedCrouch = input.grounded ? std::clamp(input.jumpCrouch, 0.0f, 1.0f) : 0.0f;
        m_crouch += (wantedCrouch - m_crouch) * (1.0f - std::exp(-dt / 0.05f));
        if (m_crouch > 0.001f && animations.jumpPrep != nullptr) {
            const AnimationClip3D& prep = *animations.jumpPrep;
            const float weight = smoothStep(m_crouch / 0.15f);
            const float time = m_crouch * prep.durationSeconds;
            for (std::size_t i = 1; i < links; ++i) {
                const AnimationTrack3D* track = findAnimationTrack3D(prep, profile.links[i].id);
                if (track == nullptr) continue;
                const Vec3 pose = sampleTrack(*track, time, prep.durationSeconds, false, s.smoothClipSampling).jointPositionRadians;
                m_coordinates[i] += (pose - m_coordinates[i]) * (weight * (trunkLink(profile.links[i].id) ? 0.2f : 1.0f));
            }
            const AnimationTransformSample3D root = rootAt(profile, prep, time, false, s.smoothClipSampling);
            clipRootRotation = nlerp(clipRootRotation, root.rotationDelta.normalized(), weight);
            rootOffset.z += (root.translationOffsetMeters.z - rootOffset.z) * weight;
        }
    }

}

} // namespace MatterEngine
