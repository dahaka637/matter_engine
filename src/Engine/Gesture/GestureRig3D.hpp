#pragma once

#include "Engine/Math/Quaternion.hpp"
#include "Engine/Math/Vec3.hpp"

#include <array>

namespace MatterEngine {

// O que o gesto precisa do esqueleto (a locomocao preenche pelo perfil):
// tudo no referencial da pelve de pe (x frente, y esquerda, z cima; a origem
// na pelve). A perna de dois ossos: o quadril, os comprimentos da coxa (quadril
// ao joelho) e da canela (joelho ao tornozelo) e a altura do tornozelo sobre a
// sola com o pe chato. hipFrame leva o giro da coxa (na pelve) as coordenadas
// da junta: coordenada = log(hipFrame^-1 x giro x hipFrame). Convencoes das
// outras juntas (as do jogador; o teste da locomocao confere): o joelho e um
// eixo so (+ dobra, a canela vai para tras); o tornozelo, o primeiro eixo +
// leva a ponta para baixo e o segundo + vira a sola para a direita do corpo.
struct GestureLegRig3D {
    Vec3 hip;
    float thigh = 0.418f;
    float shin = 0.414f;
    float ankleHeight = 0.116f;
    Quaternion hipFrame;
};

struct GestureRig3D {
    bool valid = false;
    // 0 esquerda, 1 direita.
    std::array<GestureLegRig3D, 2> legs {};
    // A pelve sobre o chao, de pe (m).
    float pelvisHeight = 0.978f;
    // O giro que leva o referencial do corpo (x no rumo, z para cima) ao da
    // pelve no comeco do gesto: o catalogo pensa no corpo e resolve na pelve.
    // (Parado, a pose da pelve fica 22 graus para a direita e 6 inclinada;
    // pensado direto na pelve, o chute de direita saia para fora, o de
    // esquerda cruzava para o outro lado e o contato errava a bola.)
    Quaternion bodyToPelvis;
};

} // namespace MatterEngine
