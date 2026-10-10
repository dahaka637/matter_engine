#pragma once

#include "Engine/Modeling/Mesh/MeshBuilder.hpp"
#include "Engine/Modeling/Parts/HeadRecipe.hpp"

#include <cstdint>
#include <vector>

// A orelha (como em rosto.png): uma aba que sai da pele do lado da cabeca.
// Da volta da raiz (o furo na pele) ela sobe pelas costas lisas ate a borda
// da helice, dobra por cima e desce pela frente - a aba da helice, o sulco,
// a anti-helice - ate a concha funda e o canal escuro no fundo. Uma
// superficie so com a cabeca: cada anel continua do anterior, vertice com
// vertice.
namespace MatterEngine::Modeling {

struct EarPlacement {
    // A volta da raiz (o furo na pele), um ponto por ponto do contorno da
    // orelha, anti-horaria vista de fora (do lado da orelha).
    std::vector<std::uint32_t> root;
    // O referencial da orelha: o centro (na pele), para fora da cabeca, para
    // a frente (o rosto) e para cima.
    Vec3 center;
    Vec3 out;
    Vec3 forward;
    Vec3 up;
    int headJoint = -1;
};

// Ergue a orelha na parte corrente do builder (o fundo da concha e o canal
// no material "orelha_canal", na sombra, criado aqui). Pesos: a cabeca.
void buildEar(MeshBuilder& builder, const HeadRecipe& recipe, const EarPlacement& ear);

} // namespace MatterEngine::Modeling
