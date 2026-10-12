#include "Engine/Modeling/Rig/RagdollSkeletonView.hpp"

#include <algorithm>

namespace MatterEngine::Modeling {

RagdollSkeletonView::RagdollSkeletonView(std::shared_ptr<const RagdollCharacter3D> character,
    std::shared_ptr<const GameRig> rig)
    : m_character(std::move(character)), m_rig(std::move(rig)) {
    const RagdollProfile3D& profile = m_character->profile;
    // O chao do espaco do modelo do jogo: a pelve de repouso na posicao dela
    // e o chao standingRootHeightMeters abaixo.
    m_gameFloor = profile.links.empty() ? 0.0f
        : profile.links.front().modelPosition.z - profile.standingRootHeightMeters;
}

Vec3 RagdollSkeletonView::toGame(Vec3 world) const {
    const Vec3 pelvis = m_character->profile.links.empty() ? Vec3 {} : m_character->profile.links.front().modelPosition;
    return { world.x - m_anchor.x + pelvis.x, world.y - m_anchor.y + pelvis.y, world.z - m_floor + m_gameFloor };
}

Vec3 RagdollSkeletonView::toSkeleton(Vec3 world) const { return gameToSkeleton(toGame(world)); }

Vec3 RagdollSkeletonView::gameToSkeleton(Vec3 game) const {
    if (!m_rig->fitted) return game;
    // A raiz medida do modelo novo sobe/escala como em sampleGameRigPose.
    for (const Joint& joint : m_rig->skeleton.joints)
        if (joint.parent < 0)
            return joint.bind.position + Vec3 { 0.0f, 0.0f, m_rig->rootRestLift }
                + (game - m_rig->rootGameBind.position) * m_rig->rootScale;
    return game;
}

const SkeletonPose& RagdollSkeletonView::update(std::span<const Vec3> linkPositions,
    std::span<const Quaternion> linkOrientations, Vec3 anchor, float floor, float deltaTime) {
    m_anchor = anchor;
    m_floor = floor;
    m_links.linkPositions.resize(linkPositions.size());
    m_links.linkOrientations.assign(linkOrientations.begin(), linkOrientations.end());
    for (std::size_t i = 0; i < linkPositions.size(); ++i) m_links.linkPositions[i] = toGame(linkPositions[i]);
    RagdollVisualPoseInput3D input;
    input.linkPositions = m_links.linkPositions;
    input.linkOrientations = m_links.linkOrientations;
    input.groundHeight = { m_gameFloor, m_gameFloor };
    input.deltaTime = std::max(1.0f / 240.0f, deltaTime);
    const std::vector<Quaternion>& rotations = m_visual.update(*m_character, input);
    m_pose = gameRigPoseFromLinks(*m_character, *m_rig, m_links, rotations, false);
    return m_pose;
}

} // namespace MatterEngine::Modeling
