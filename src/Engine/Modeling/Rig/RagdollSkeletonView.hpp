#pragma once

#include "Engine/Animation/RagdollVisualPose3D.hpp"
#include "Engine/Modeling/Rig/GameRig.hpp"

#include <memory>
#include <span>

namespace MatterEngine::Modeling {

// O jogador de verdade (a pele) na pose de um ragdoll do mundo: os links
// fisicos vao para o espaco do modelo do jogo e dali para o esqueleto do
// modelo, com os ossos visuais (maos, dedos dos pes) por cima. E o que a folha
// de quadros e a tela de Aprendizado por Reforco desenham.
class RagdollSkeletonView {
public:
    RagdollSkeletonView(std::shared_ptr<const RagdollCharacter3D> character, std::shared_ptr<const GameRig> rig);

    // anchor: o ponto do mundo que fica no lugar da pelve de repouso (x, y);
    // floor: o z do chao no mundo. deltaTime alimenta os ossos visuais.
    const SkeletonPose& update(std::span<const Vec3> linkPositions, std::span<const Quaternion> linkOrientations,
        Vec3 anchor, float floor, float deltaTime);
    // Um ponto do mundo no espaco do esqueleto (para os desenhos por cima),
    // com o anchor e o chao do ultimo update.
    [[nodiscard]] Vec3 toSkeleton(Vec3 world) const;
    // Um ponto do espaco do modelo do jogo (o dos gameLinks) no do esqueleto.
    [[nodiscard]] Vec3 gameToSkeleton(Vec3 game) const;
    // Os links do ultimo update, no espaco do modelo do jogo.
    [[nodiscard]] const RagdollAnimationPose3D& gameLinks() const { return m_links; }
    [[nodiscard]] const SkeletonPose& pose() const { return m_pose; }
    void reset() { m_visual.reset(); }

private:
    [[nodiscard]] Vec3 toGame(Vec3 world) const;

    std::shared_ptr<const RagdollCharacter3D> m_character;
    std::shared_ptr<const GameRig> m_rig;
    RagdollVisualPose3D m_visual;
    RagdollAnimationPose3D m_links;
    SkeletonPose m_pose;
    Vec3 m_anchor;
    float m_floor = 0.0f;
    float m_gameFloor = 0.0f;
};

} // namespace MatterEngine::Modeling
