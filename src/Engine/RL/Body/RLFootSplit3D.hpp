#pragma once

#include "Engine/Physics/RagdollProfile3D.hpp"

#include <string>
#include <vector>

namespace MatterEngine {

// A junta dos dedos do corpo de RL (a reformulacao do RL, plano_claude_rl.md
// secao 5.10): a casca da chuteira de cada pe e dividida na linha dos dedos;
// o pe fica com o calcanhar e um link novo (LeftToe, RightToe), filho dele, com
// a ponta. A junta tem um eixo so (o twist do quadro do tornozelo, de lado):
// positivo dobra a ponta para baixo, negativo para cima. O perfil do jogo nao
// muda; so o corpo de RL que pede ("toes" no arquivo do corpo) usa o dividido.
struct RLToeSettings3D {
    // A linha dos dedos: a fracao do comprimento da sola a partir do
    // calcanhar (o osso dos dedos do modelo visual fica a 70%) e a altura do
    // eixo acima da sola (m).
    float jointFromHeel = 0.70f;
    float jointHeight = 0.025f;
    // A fracao da massa do pe que vai para a ponta.
    float massShare = 0.2f;
    float minimumRadians = -1.0472f;
    float maximumRadians = 0.4363f;
    // O drive do eixo novo no perfil (os do corpo de RL entram por cima).
    float stiffness = 60.0f;
    float damping = 2.0f;
    float maximumTorque = 60.0f;
};

// Divide os pes (LeftFoot e RightFoot, cascas convexas) e acrescenta os links
// da ponta no fim do perfil. Mantem a massa total e o centro de massa de cada
// pe. Lanca se um pe nao for casca ou se a divisao nao der duas cascas.
// report recebe uma linha por pe (as medidas da divisao).
void splitRagdollToes3D(RagdollProfile3D& profile, const RLToeSettings3D& settings,
    std::vector<std::string>* report = nullptr);

} // namespace MatterEngine
