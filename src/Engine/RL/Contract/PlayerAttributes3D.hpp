#pragma once

#include <array>

namespace MatterEngine {

// Os dados do jogador que o Corpo recebe (plano_claude_rl.md secao 5.2): o jogo
// os conhece, entao a rede nao precisa adivinhar. Ate a F3 ficam no nominal.
struct PlayerAttributes3D {
    float heightMeters = 1.80f;
    float massKg = 80.0f;
    // A forca de cada grupo (fator sobre os tetos de torque): pernas (coxa,
    // canela, pe, ponta), tronco (abdomen ao pescoco e cabeca) e bracos.
    std::array<float, 3> strength { 1.0f, 1.0f, 1.0f };
    // A velocidade maxima de frente (m/s) e o teto na direcao pedida, em
    // relacao ao olhar (o adaptador do controle preenche pela tabela de tetos).
    float sprintCap = 6.5f;
    float capNow = 6.5f;
};

// Os grupos da forca, pelo id do link.
enum class StrengthGroup3D : int { Legs = 0, Trunk = 1, Arms = 2 };

} // namespace MatterEngine
