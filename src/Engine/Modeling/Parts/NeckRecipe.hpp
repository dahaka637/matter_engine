#pragma once

#include "Engine/Modeling/Parts/ProfileCurve.hpp"
#include "Engine/Modeling/Raster/Color.hpp"

#include <optional>
#include <string>
#include <string_view>

namespace MatterEngine::Modeling {

// O pescoco em numeros (partes/pescoco.json): a pele da volta de dentro da
// gola (no V, o alto do peito) ate a base da cabeca (aberta: a cabeca vem
// dali). Alturas em metros do chao.
struct NeckRecipe {
    // A secao por altura (superelipse de quatro quadrantes em volta do
    // centro): o x do centro, frente, tras, meia largura e o expoente.
    ProfileCurve centerX;
    ProfileCurve front;
    ProfileCurve back;
    ProfileCurve side;
    ProfileCurve exponent;
    // A abertura da cabeca: um plano inclinado, mais baixo na frente (sob o
    // queixo) que atras (a nuca).
    float headFrontHeight = 1.66f;
    float headBackHeight = 1.7f;
    // Os aneis entre a gola e a abertura, e o facetado.
    int rings = 3;
    float jitter = 0.0015f;
    unsigned seed = 61;

    Color skinColor = rgb(212, 152, 106);

    [[nodiscard]] static NeckRecipe defaults();
};

[[nodiscard]] std::optional<NeckRecipe> parseNeckRecipe(std::string_view json, std::string* error = nullptr);

} // namespace MatterEngine::Modeling
