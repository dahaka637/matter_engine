#pragma once

#include "Engine/Character/CharacterLocomotion3D.hpp"
#include "Engine/Control/AdaptivePhysicalCharacter3D.hpp"

namespace MatterEngine {

// Escreve, uma vez por personagem e tick, tudo o que o controle fisico pede
// ao backend: motores das juntas, guia, forcas da assistencia adaptativa
// (pelve, pes, maos apoiadas), a ajuda do levantar no peito e as reacoes dela
// nos segmentos apoiados. O laboratorio e os testes chamam esta mesma
// funcao - antes o teste aplicava so parte disto no boneco nao controlado e
// nao reproduzia o jogo.
//
// `adaptive` nulo: modo antigo (raiz carregada), so as forcas da locomocao.
// `down`: caido ou levantando - a assistencia adaptativa fica suspensa e a
// locomocao conduz.
void applyCharacterControl3D(PhysicsScene3D& scene, RagdollHandle3D ragdoll,
    const CharacterLocomotionOutput3D& locomotion,
    const AdaptivePhysicalOutput3D* adaptive, bool down);

// Velocidade com que a navegacao (capsula do jogador, ancora virtual do
// boneco solto) vai atras da pelve fisica: nada dentro de `slack`, depois
// 12/s da distancia excedente, ate 6 m/s. `reference` e o ponto com que o
// corpo e comparado (so o plano conta).
Vec3 navigationFollowVelocity3D(Vec3 body, Vec3 reference, float slack);

} // namespace MatterEngine
