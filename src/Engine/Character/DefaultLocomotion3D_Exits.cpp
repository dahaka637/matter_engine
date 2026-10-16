#include "Engine/Character/DefaultLocomotion3DInternal.hpp"

namespace MatterEngine {

void DefaultLocomotion3D::stepExits(Tick& t) {
    const RagdollProfile3D& profile = t.profile;
    const DefaultLocomotionSettings3D& s = m_settings;
    const RagdollLinkDefinition3D& rootLink = *t.rootLink;
    const float& dt = t.dt;
    const std::size_t& links = t.links;
    const Vec3& left = t.left;
    const bool& seedTick = t.seedTick;
    const Vec3& feet = t.feet;
    Quaternion& rootOrientation = t.rootOrientation;
    Vec3& rootPosition = t.rootPosition;
    // Saida da arrancada agachada: pela inercia (nao no primeiro passe do tick
    // da semente, que e refeito).
    if (m_exitInertia.active() && !seedTick)
        m_exitInertia.apply(m_coordinates, rootPosition, rootOrientation, m_exitInertiaDt > 0.0f ? m_exitInertiaDt : dt);
    m_exitInertiaDt = 0.0f;
    // Saida do carrinho: da pose do fim dele ate a da locomocao.
    if (m_tackleExit < 1.0f) {
        m_tackleExit = std::min(1.0f, m_tackleExit + dt / std::max(0.01f, s.tackleExitSeconds));
        const float share = smoothStep(m_tackleExit);
        if (m_exitFrom.size() == links)
            for (std::size_t i = 1; i < links; ++i)
                m_coordinates[i] = m_exitFrom[i] + (m_coordinates[i] - m_exitFrom[i]) * share;
        const Vec3 from = feet + m_exitRootOffset;
        rootPosition = from + (rootPosition - from) * share;
        rootOrientation = nlerp(m_exitRootOrientation, rootOrientation, share);
    }

    // A voadora no ar: a pose do carrinho no comeco da fase deitada, no rumo
    // do voo, em ~0,12 s (a pelve no maximo 15 cm abaixo da do voo: as pernas
    // nao passam da base da capsula).
    m_output.tackling = false;
    m_output.airTackle = m_airTackleClip != nullptr;
    if (m_airTackleClip != nullptr) {
        const AnimationClip3D& clip = *m_airTackleClip;
        m_airTackleWeight = std::min(1.0f, m_airTackleWeight + dt / std::max(0.02f, m_airTackleBlendSeconds));
        const float w = smoothStep(m_airTackleWeight);
        // O tempo do clipe: do mergulho ate a fase deitada, saindo e chegando
        // devagar (parando de uma vez, a pelve girava a 30 mil graus/s2).
        m_airTackleElapsed += dt;
        for (std::size_t k = 0; k < m_tackles.size(); ++k)
            if (m_tackles[k] == &clip) {
                const float begin = m_tackleShape[k].diveStart;
                const float span = std::max(0.0f, m_tackleShape[k].lowStart - 0.02f - begin);
                m_airTackleTime = begin + span * smoothStep(m_airTackleElapsed / std::max(0.05f, m_airTackleDiveSeconds));
            }
        for (std::size_t i = 1; i < links; ++i)
            if (const AnimationTrack3D* track = findAnimationTrack3D(clip, profile.links[i].id))
                m_coordinates[i] += (sampleTrack(*track, m_airTackleTime, clip.durationSeconds, false, s.smoothClipSampling)
                    .jointPositionRadians - m_coordinates[i]) * w;
        const AnimationTransformSample3D r = rootAt(profile, clip, m_airTackleTime, false, s.smoothClipSampling);
        const Quaternion airOrientation = (yawRotation(m_airTackleYaw) * rootLink.modelOrientation * r.rotationDelta.normalized())
            .normalized();
        rootOrientation = nlerp(rootOrientation, airOrientation, w);
        // Deitando para tras no voo (a perna esticada sobe para a linha da
        // pelve, como uma voadora), em volta da pelve, pelo andar do mergulho.
        {
            float progress = 1.0f;
            for (std::size_t k = 0; k < m_tackles.size(); ++k)
                if (m_tackles[k] == &clip) {
                    const float span = m_tackleShape[k].lowStart - 0.02f - m_tackleShape[k].diveStart;
                    progress = span > 1e-3f ? std::clamp((m_airTackleTime - m_tackleShape[k].diveStart) / span, 0.0f, 1.0f) : 1.0f;
                }
            const Vec3 left { -std::sin(m_airTackleYaw), std::cos(m_airTackleYaw), 0.0f };
            rootOrientation = (Quaternion::fromAxisAngle(left, -radians(s.airTacklePitchDegrees) * w * smoothStep(progress))
                * rootOrientation).normalized();
        }
        rootPosition.z += std::clamp(r.translationOffsetMeters.z, -0.15f, 0.0f) * w;
        // A perna de apoio do clipe e esticada (a pelve do voo e mais baixa
        // que a de pe): o pe nao passa da base da capsula (com o peso: logo
        // depois de sair do chao o pe do pulo ainda esta abaixo dela).
        defaultLocomotionPose3D(profile, rootPosition, rootOrientation, m_coordinates, m_output.pose);
        rootPosition.z += feetBelowGround(profile, m_output.pose, m_feet, feet.z) * w;
        m_output.tackling = true;
        // A ancora do caminho do clipe (a raiz dele no tempo da pose sob a
        // pelve), para o fisico continuar dali.
        const Vec3 along = yawRotation(m_airTackleYaw).rotate({ r.translationOffsetMeters.x * m_tackleScale,
            r.translationOffsetMeters.y * m_tackleScale, 0.0f });
        m_tackleStart = { rootPosition.x - along.x, rootPosition.y - along.y, 0.0f };
        m_airPreviousCoordinates = m_output.coordinates;
        m_airPreviousRoot = m_output.rootPosition;
        m_airPreviousOrientation = m_output.rootOrientation;
    }
    defaultLocomotionPose3D(profile, rootPosition, rootOrientation, m_coordinates, m_output.pose);
    m_output.rootPosition = rootPosition;
    m_output.rootOrientation = rootOrientation;
    m_output.coordinates = m_coordinates;
}

} // namespace MatterEngine
