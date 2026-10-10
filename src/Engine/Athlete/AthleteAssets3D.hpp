#pragma once

#include "Engine/Animation/AnimationClip3D.hpp"
#include "Engine/Animation/RagdollCharacter3D.hpp"
#include "Engine/Character/CharacterLocomotion3D.hpp"
#include "Engine/Physics/PhysicsScene3D.hpp"

#include <string>
#include <vector>

namespace MatterEngine {

// A montagem do personagem que o laboratorio, o jogo e a sonda sem janela
// fazem igual: a biblioteca de clipes do rig, os papeis da locomocao e a
// capsula do jogador.

// Os clipes de uma pasta (*.matteranim.json) feitos para o rig do perfil e
// validos contra ele, em ordem de nome. Os que falham entram em `errors`.
[[nodiscard]] std::vector<AnimationClip3D> loadAthleteClipLibrary3D(const std::string& clipDirectory,
    const RagdollProfile3D& profile, std::vector<std::string>* errors = nullptr);

// Os clipes de cada papel da locomocao (o manifesto do personagem). O
// levantar de brucos e o de costas escolhidos no painel (0 o do Kimodo, o
// padrao; 1 o antigo), para comparar.
[[nodiscard]] CharacterLocomotionAnimations3D athleteLocomotionClips3D(const RagdollCharacter3D& character,
    const std::vector<AnimationClip3D>& clips, int getUpFrontChoice = 0, int getUpBackChoice = 0);

// A capsula de navegacao do jogador (medidas e respostas do laboratorio; as
// velocidades de andar e correr saem dos clipes em quem chama).
[[nodiscard]] CharacterMotorSettings3D athleteAvatarMotorSettings3D();

} // namespace MatterEngine
