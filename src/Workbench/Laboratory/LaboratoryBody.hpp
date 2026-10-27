#pragma once

#include "Engine/Animation/AnimationClip3D.hpp"
#include "Engine/Animation/RagdollCharacter3D.hpp"
#include "Engine/Animation/RagdollCharacterBody3D.hpp"
#include "Engine/Physique/PlayerPhysique3D.hpp"

#include <map>
#include <utility>
#include <vector>

namespace MatterEngine::Workbench {

// O corpo variavel do laboratorio (26/10, pedido do usuario): um corpo so para
// todos os bonecos humanos. A receita do AthleteBundle3D (o personagem do
// corpo com a folga nos clipes de locomocao e cada clipe no corpo), com a
// malha no tonus: o laboratorio desenha esta malha (o caminho da CPU).
struct LaboratoryBody {
    PlayerBody3D body;
    RagdollCharacter3D character;
    std::vector<AnimationClip3D> clips;
};

class LaboratoryBodyCache {
public:
    // O corpo de altura e peso (cm e kg, arredondados e nas faixas), feito do
    // personagem e da biblioteca de hoje (sempre os mesmos objetos). No
    // 180/80, nullptr: quem chama volta aos originais. Os corpos feitos ficam
    // vivos ate o fim: a locomocao padrao e a cinematica guardam tabelas pelo
    // endereco do perfil e dos clipes, e um endereco nunca volta com outro
    // corpo.
    [[nodiscard]] const LaboratoryBody* select(const RagdollCharacter3D& base,
        const std::vector<AnimationClip3D>& baseClips, float heightCm, float massKg);

private:
    RagdollToneDeltas3D m_tone;
    // shortcut: ~3 MB por corpo distinto, nunca liberado (pior caso ~3 GB nas
    // 961 combinacoes); trocar por invalidar os caches da locomocao e da
    // cinematica se o laboratorio passar a varrer corpos.
    std::map<std::pair<int, int>, LaboratoryBody> m_bodies;
};

} // namespace MatterEngine::Workbench
