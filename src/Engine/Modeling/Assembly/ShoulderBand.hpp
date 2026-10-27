#pragma once

#include "Engine/Modeling/Mesh/Mesh.hpp"
#include "Engine/Modeling/Skeleton/Skeleton.hpp"

#include <array>
#include <functional>

// O ombro desenhado do jogador montado (22/10/2026): os pesos de calor
// levavam o braco pelo flanco ate as costelas e o ombro passava direto do
// peito ao braco (a meia-junta sem peso), o braco erguido rasgava o peito.
// Aqui os pesos da regiao saem de um campo no referencial do braco na
// ligacao: ao longo dele (s) a passagem peito -> meia-junta -> braco, com o
// centro e a largura mudando em volta dele (frente, alto/fora, costas,
// axila, como a faixa do quadril); longe do eixo (o flanco, o esterno) volta
// ao tronco; depois da faixa, ate "rigidUntil", so o braco (a manga e a barra
// rigidas com ele). No maximo quatro ossos por construcao.
namespace MatterEngine::Modeling {

struct ShoulderBand {
    // Frente, alto/fora, costas, axila (metros, no referencial do braco):
    // o centro da passagem ao longo do braco a partir da junta, a largura
    // dela, a distancia do eixo ate onde vale inteira e onde some.
    std::array<float, 4> center {};
    std::array<float, 4> width {};
    std::array<float, 4> full {};
    std::array<float, 4> end {};
    // Perto do meio (|y|, metros) o campo some: o esterno e a coluna ficam.
    std::array<float, 2> midline { 0.06f, 0.10f };
    // Ate onde (ao longo do braco) o campo vale; depois, os pesos de antes
    // (o cotovelo).
    float rigidUntil = 0.18f;
    // A gola costurada na pele do pescoco: o campo some de [1] a [0] metros
    // abaixo dela (a pele do outro lado da costura nao muda).
    std::array<float, 2> collarGuard { 0.02f, 0.05f };

    [[nodiscard]] bool enabled() const { return width[0] > 0.0f && width[1] > 0.0f && width[2] > 0.0f && width[3] > 0.0f; }
};

// Os pesos do ombro direito na malha (a metade direita; o espelho leva): o
// tronco e o braco que cada vertice ja tem ficam (o pescoco, o cotovelo), so a
// proporcao entre eles e a meia-junta (RightShoulderHelper) vem do campo.
// Devolve quantos vertices mudaram.
// "collarHeight": a altura (z) da gola na direcao do ponto.
std::size_t applyShoulderBand(Mesh& mesh, const Skeleton& skeleton, const ShoulderBand& band,
    const std::function<float(Vec3)>& collarHeight);

} // namespace MatterEngine::Modeling
