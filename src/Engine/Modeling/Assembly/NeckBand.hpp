#pragma once

#include "Engine/Modeling/Assembly/Uniform.hpp"
#include "Engine/Modeling/Mesh/Mesh.hpp"

#include <optional>

// A faixa do pescoco do jogador montado: da base do pescoco (a gola,
// UniformLines::collarHeight; dentro do V, nunca abaixo de "frontBase": a pele
// do V e peito) ate a linha onde o rosto passa a andar so com a cabeca (a
// linha de cima do pescoco, NeckShortening::top, mais "headAbove"). A malha base tem poucas
// fileiras ali e, pelo calor dos ossos, o giro do pescoco caia quase inteiro
// na fileira da gola (o pescoco torcia como corda): ao longo da faixa os pesos
// passam em linha do alto do peito ao pescoco e dele a cabeca, o giro e a
// dobra repartidos por todas as fileiras.
namespace MatterEngine::Modeling {

struct NeckBand {
    UniformLines lines;
    float headAbove = 0.012f;
    // A base do pescoco na frente (altura do chao no corpo base, como as da
    // receita): o fundo do V da gola fica abaixo dela.
    float frontBase = 1.5f;
    // So perto do meio (|y| menor que isso): o ombro fica de fora.
    float width = 0.075f;
    // Onde o pescoco pesa inteiro (0 = a gola, 1 = a linha da cabeca): abaixo
    // o alto do peito passa ao pescoco, acima o pescoco passa a cabeca.
    float middle = 0.5f;

    // A altura (z) da base do pescoco e a da linha da cabeca na direcao do
    // ponto.
    [[nodiscard]] float base(Vec3 point) const;
    [[nodiscard]] float headLine(Vec3 point) const;
    // 0 na base, 1 na linha da cabeca (fora de 0..1: fora da faixa).
    [[nodiscard]] float along(Vec3 point) const;
};

// Os pesos de um ponto dentro da faixa (perto do meio): o alto do peito, o
// pescoco e a cabeca, em linha ao longo dela; nada fora dela.
[[nodiscard]] std::optional<SkinWeights> neckBandWeights(const NeckBand& band, Vec3 point, int chest, int neck, int head);

} // namespace MatterEngine::Modeling
