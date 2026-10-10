#pragma once

#include "Engine/Modeling/Assembly/AssemblyRecipe.hpp"
#include "Engine/Modeling/Assembly/NeckShortening.hpp"
#include "Engine/Modeling/Mesh/Mesh.hpp"
#include "Engine/Modeling/Skeleton/Skeleton.hpp"

#include <string>

// O uniforme do jogador montado (a camisa e o calcao, ver Garment) na
// metade direita do corpo ja pesado: as copias da pele cortadas nas barras,
// alisadas e infladas; a pele coberta sai; os recuos costurados nas barras.
namespace MatterEngine::Modeling {

// Onde o uniforme corta (o mesmo para o corte e para a pintura), na metade
// direita (o lado esquerdo e o espelho: y para -|y|).
struct UniformLines {
    float ground = 0.0f;
    // A gola em volta do pescoco (angulo 0 = frente, 180 = nuca, em volta do
    // eixo de pe pela junta do pescoco): o V sobe em linha do fundo ate o lado
    // do pescoco e dali vai macio ate a altura de tras.
    float neckX = 0.0f;
    float collarDepth = 0.0f, collarSide = 0.0f, collarBack = 0.0f, collarOpening = 1.0f;
    // A barra da manga (plano perpendicular ao braco) e a da perna do calcao
    // (perpendicular a coxa); as alturas da barra da camisa e do alto do calcao.
    Vec3 sleevePoint, armAlong;
    Vec3 legHemPoint, legAlong;
    float shirtHem = 0.0f, shortsTop = 0.0f;
    // O pescoco mais curto: as alturas da receita sao as do corpo base.
    NeckShortening neck;

    // A altura (z) da borda da gola na direcao do ponto.
    [[nodiscard]] float collarHeight(Vec3 point) const;
    // Uma altura do corpo base (do chao, abaixo do pescoco) no corpo montado.
    [[nodiscard]] float lifted(float height) const { return neck.liftedHeight(0.0f, height); }
};

[[nodiscard]] UniformLines uniformLines(const AssemblyRecipe& recipe, const Skeleton& skeleton);

// As pecas entram como as partes "camisa" e "calcao" (com os materiais de
// mesmo nome), ligadas ao corpo, a cabeca e uma a outra.
bool buildUniform(Mesh& rightHalf, const AssemblyRecipe& recipe, const Skeleton& skeleton, std::string& error);

} // namespace MatterEngine::Modeling
