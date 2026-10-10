#pragma once

#include "Engine/Modeling/Geometry/ModelGeometry.hpp"
#include "Engine/Modeling/Parts/BootRecipe.hpp"

#include <array>

// A forma lisa da chuteira (antes do facetado), no espaco dela: x ao longo do
// pe (t vezes o comprimento), y o lado (v, + para dentro) e z do chao.
namespace MatterEngine::Modeling {

// O anel do cabedal: 15 pontos da base de dentro, pelo topo, ate a base de
// fora (a base fecha por baixo, sobre a sola). Na abertura, 4-5 e 9-10 sao a
// borda grossa e 5..9 descem para o interior; no cadarco, 5 e 9 sao as abas
// dos ilhoses e 6..8 o canal.
inline constexpr int BootRingSize = 15;
using BootRing = std::array<Point2, BootRingSize>;

struct BootHalfWidths {
    float medial = 0.0f;
    float lateral = 0.0f;
};

// A secao em t: meia largura na base (com as pontas arredondadas), a base
// (logo acima da sola) e a altura ate o topo.
struct BootSection {
    float medial = 0.0f;
    float lateral = 0.0f;
    float base = 0.0f;
    float height = 0.0f;
    float exponent = 2.0f;
    // O lado estufa no meio da altura (o contraforte em forma de ovo).
    float bulge = 0.0f;
};

[[nodiscard]] float bootSmoothstep(float edge0, float edge1, float x);
// O quanto a largura (vista de cima) e a altura do bico (vista de lado)
// encolhem nas pontas: 1 no meio, endScale no ultimo anel.
[[nodiscard]] float bootOutlineScale(const BootRecipe& recipe, float t);
[[nodiscard]] float bootNoseScale(const BootRecipe& recipe, float t);
// Quanto a borda de fora da abertura fica abaixo da de dentro em t.
[[nodiscard]] float bootLateralCollarDrop(const BootRecipe& recipe, float t);
[[nodiscard]] float bootSoleTop(const BootRecipe& recipe, float t);
// A meia largura do cabedal na base (as pontas ja arredondadas).
[[nodiscard]] BootHalfWidths bootBaseHalfWidths(const BootRecipe& recipe, float t);
// A da sola: a base do cabedal mais a borda que aparece.
[[nodiscard]] BootHalfWidths bootSoleHalfWidths(const BootRecipe& recipe, float t);
[[nodiscard]] BootSection bootSection(const BootRecipe& recipe, float t);
// 0 fechado (calcanhar atras, lingua, peito do pe), 1 aberto.
[[nodiscard]] float bootOpening(const BootRecipe& recipe, float t);
// O mesmo para o ponto k do anel: do lado do calcanhar o meio (5..9) abre
// antes, e o fundo da abertura fica em U.
[[nodiscard]] float bootOpeningAt(const BootRecipe& recipe, float t, int k);
// O quanto t esta na regiao do cadarco (0 fora, 1 dentro).
[[nodiscard]] float bootLacing(const BootRecipe& recipe, float t);
// A meia distancia entre os ilhoses em t.
[[nodiscard]] float bootLaceHalfSpacing(const BootRecipe& recipe, float t);
// Quanto o canal do cadarco afunda em (t, v): no meio, entre as fileiras, e
// some nas abas dos ilhoses.
[[nodiscard]] float bootLaceChannel(const BootRecipe& recipe, float t, float v);
// O anel do cabedal em t; phase 0 ou 0,5 desencontra os aneis alternados (os
// triangulos ficam quase equilateros, nao faixas compridas).
[[nodiscard]] BootRing bootRing(const BootRecipe& recipe, float t, float phase);
// O contraforte: quanto um ponto anda para a frente pela altura relativa no
// anel (0 na base, 1 no topo); negativo = estufa para tras.
[[nodiscard]] float bootHeelShift(const BootRecipe& recipe, float t, float heightFraction);

} // namespace MatterEngine::Modeling
