#pragma once

#include "Engine/Modeling/Parts/ProfileCurve.hpp"
#include "Engine/Modeling/Raster/Color.hpp"

#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace MatterEngine::Modeling {

// A perna em numeros (partes/perna.json): da gola da chuteira ate a barra do
// calcao, o meiao, a faixa, o joelho e a coxa, uma superficie so. Alturas em
// metros a partir do chao; as distancias da secao sao a partir do eixo do
// osso (a linha do quadril ao tornozelo, de pe).
struct LegRecipe {
    // A secao por altura: ate onde a superficie vai para a frente, para
    // tras, para dentro (o lado do meio do corpo) e para fora; e o expoente
    // da superelipse (2 = elipse; maior = mais quadrada).
    ProfileCurve front;
    ProfileCurve back;
    ProfileCurve medial;
    ProfileCurve lateral;
    ProfileCurve exponent;
    // As alturas dos aneis, do primeiro acima da gola ate a barra (o ultimo).
    std::vector<float> rings;
    int ringPoints = 20;
    // Os aneis que saem da gola: a forma dela vira a da perna aos poucos;
    // o primeiro fica collarLift acima da borda da gola.
    int collarRings = 2;
    float collarLift = 0.008f;
    // O meiao vai ate sockTop; a faixa de sockTop ate bandTop; pele acima.
    float sockTop = 0.48f;
    float bandTop = 0.525f;
    // O joelho (alturas a partir da junta do joelho): a patela salta na
    // frente (o centro, a meia altura, a meia abertura em radianos e quanto
    // sai), a tuberosidade da tibia logo abaixo e o oco atras.
    float patellaCenter = 0.026f;
    float patellaHalfHeight = 0.03f;
    float patellaHalfAngle = 0.5f;
    float patellaProtrusion = 0.011f;
    float tuberosityCenter = -0.045f;
    float tuberosityHalfHeight = 0.016f;
    float tuberosityProtrusion = 0.003f;
    float poplitealDepth = 0.004f;
    // O facetado: o quanto cada vertice anda para dentro/fora e para
    // cima/baixo (m; o vertical nunca passa de um quarto do passo entre os
    // aneis) e a semente (estavel entre geracoes).
    float jitter = 0.0026f;
    float verticalJitter = 0.004f;
    unsigned seed = 11;

    Color sockColor = rgb(232, 228, 224);
    Color bandColor = rgb(34, 34, 37);
    Color skinColor = rgb(212, 152, 106);

    // A receita padrao (o desenho a partir das referencias).
    [[nodiscard]] static LegRecipe defaults();
};

// Le os campos presentes no JSON por cima do padrao (o que faltar fica).
[[nodiscard]] std::optional<LegRecipe> parseLegRecipe(std::string_view json, std::string* error = nullptr);

} // namespace MatterEngine::Modeling
