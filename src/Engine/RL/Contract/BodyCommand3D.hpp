#pragma once

#include "Engine/Math/Vec3.hpp"

namespace MatterEngine {

// O que o jogo manda ao Corpo (a reformulacao do RL, plano_claude_rl.md secao
// 5.1): o controle do jogador (as teclas no referencial da camera, a camera e
// os botoes) ou a IA (pelo piloto). Nada de destino, proximo ponto ou nome de
// exercicio: so o que o jogo tem na hora.
struct BodyCommand3D {
    // A velocidade desejada no plano, no mundo (m/s; z ignorado). Zero = parar.
    // O adaptador do controle a monta pelas teclas (diagonais normalizadas) no
    // referencial da camera, presa no teto do jogador para aquela direcao.
    Vec3 moveVelocity;
    // O rumo pedido para o peito (rad, no mundo: a camera) e o giro dele (rad/s).
    float faceHeading = 0.0f;
    float faceRate = 0.0f;
    // O pulo: o botao foi apertado e o pulo ainda nao saiu; a subida pedida da
    // pelve (m); o tempo desde o aperto (s).
    bool jumpPending = false;
    float jumpRise = 0.0f;
    float jumpAge = 0.0f;
};

} // namespace MatterEngine
