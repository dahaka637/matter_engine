#pragma once

#include "Engine/RL/Contract/BodyCommand3D.hpp"
#include "Engine/RL/Contract/PlayerAttributes3D.hpp"

#include <array>

namespace MatterEngine {

// O adaptador do controle (plano_claude_rl.md secoes 5.1 e 9.1): o que o
// jogador aperta vira o comando do Corpo. O mesmo codigo no jogo (o controle
// de verdade, a IA) e no treino (o piloto aperta por ele). Dados puros, sem
// fisica.

// Os tetos de velocidade por direcao em relacao ao olhar (a camera), como
// fracao do sprint do jogador: frente, diagonal da frente, lado, diagonal de
// tras, costas (plano secao 11.4). Regra de jogo.
struct SpeedCaps3D {
    float front = 1.0f;
    float diagonalFront = 0.9f;
    float side = 0.65f;
    float diagonalBack = 0.6f;
    float back = 0.55f;
    // O teto (m/s) para andar relative rad a esquerda da camera (0 = frente,
    // pi = costas) num jogador com esse sprint. Linear entre as 5 direcoes.
    [[nodiscard]] float cap(float relative, float sprintCap) const;
};

struct ControlSettings3D {
    SpeedCaps3D caps;
    // O direcional abaixo disto e solto.
    float deadZone = 0.05f;
    // Quanto tempo o aperto do pulo espera a decolagem (s; plano secao 7.3).
    float jumpWindowSeconds = 0.6f;
};

// O que o jogador aperta numa decisao.
struct ControlInput3D {
    // O direcional no referencial da camera: x para frente, y para a esquerda.
    // As teclas dao as 8 direcoes (controlKeys3D, diagonais normalizadas); o
    // analogico, qualquer angulo e modulo ate 1. Zero = solto (parar).
    float stickForward = 0.0f;
    float stickLeft = 0.0f;
    // A velocidade com o direcional todo (m/s): o modo de velocidade do jogo
    // (andar, trote, corrida, sprint). O teto da direcao prende.
    float speed = 0.0f;
    // A camera: o rumo (rad, no mundo) e o giro dela (rad/s).
    float cameraHeading = 0.0f;
    float cameraRate = 0.0f;
    // O botao de pulo apertado nesta decisao e a subida pedida da pelve (m).
    bool jumpPressed = false;
    float jumpRise = 0.0f;
};

// O que o adaptador lembra entre decisoes: o pulo apertado esperando sair.
struct ControlState3D {
    bool jumpPending = false;
    float jumpAge = 0.0f;
    float jumpRise = 0.0f;
};

// O direcional das teclas (frente, tras, esquerda, direita): as 8 direcoes,
// com as diagonais normalizadas; opostas se anulam.
[[nodiscard]] std::array<float, 2> controlKeys3D(bool forward, bool back, bool left, bool right);

class ControlAdapter3D {
public:
    // O comando da decisao (dt: a duracao dela). Preenche player.capNow com o
    // teto da direcao pedida (o de frente, com o direcional solto).
    [[nodiscard]] static BodyCommand3D command(const ControlSettings3D& settings, const ControlInput3D& input,
        ControlState3D& state, PlayerAttributes3D& player, float dt);
    // O pulo saiu do chao: o aperto foi atendido.
    static void jumpTookOff(ControlState3D& state);
};

} // namespace MatterEngine
