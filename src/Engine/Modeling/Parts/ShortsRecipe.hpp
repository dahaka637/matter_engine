#pragma once

#include "Engine/Modeling/Parts/ProfileCurve.hpp"
#include "Engine/Modeling/Raster/Color.hpp"

#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace MatterEngine::Modeling {

// O calcao em numeros (partes/calcao.json): uma camada so, da barra (onde a
// coxa entra) ate o cos (a borda de cima, aberta para o tronco). Alturas em
// metros a partir do chao.
struct ShortsRecipe {
    // A perna do calcao em volta do eixo da coxa, por altura (da barra ate o
    // assento): ate onde vai para a frente, para tras, para dentro e para fora.
    ProfileCurve legFront;
    ProfileCurve legBack;
    ProfileCurve legMedial;
    ProfileCurve legLateral;
    // O quadril em volta do centro do corpo, por altura (do assento ao cos):
    // frente, tras, a meia largura e o expoente da superelipse.
    ProfileCurve hipFront;
    ProfileCurve hipBack;
    ProfileCurve hipSide;
    ProfileCurve hipExponent;
    // A barra (a altura do anel de cima da perna), o ponto mais baixo da
    // virilha (no meio, entre as pernas), o assento (o anel do quadril onde
    // as pernas se juntam) e o topo do cos.
    float hemHeight = 0.705f;
    float crotchHeight = 0.875f;
    float seatHeight = 0.9f;
    float waistHeight = 1.1f;
    // O cos: a faixa de cima, saltada para fora (a costura dele).
    float waistbandHeight = 0.035f;
    float waistbandStep = 0.0025f;
    // As alturas dos aneis da perna (da barra, exclusive, ate abaixo do
    // assento) e do quadril (acima do assento ate abaixo do cos).
    std::vector<float> legRings;
    std::vector<float> hipRings;
    // Pontos: na metade do quadril (de tras ate a frente, por fora) e na
    // costura da virilha (entre a frente e as costas, por baixo).
    int hipPoints = 20;
    int crotchPoints = 6;
    // A faixa lateral (centrada na costura do lado): a largura.
    float stripeWidth = 0.05f;
    // O facetado: para dentro/fora e para cima/baixo (m) e a semente.
    float jitter = 0.0024f;
    float verticalJitter = 0.006f;
    unsigned seed = 23;
    // O numero na frente da perna esquerda: o texto, a altura dos digitos,
    // o centro (acima da barra, e o angulo em volta da coxa: 0 = frente,
    // negativo = para fora) e a espessura do traco.
    std::string number = "10";
    float numberHeight = 0.075f;
    float numberAboveHem = 0.045f;
    float numberAngle = -0.55f;
    float numberStroke = 0.011f;

    Color shortsColor = rgb(236, 234, 232);
    Color stripeColor = rgb(34, 34, 37);
    Color insideColor = rgb(206, 204, 201);
    Color numberColor = rgb(30, 30, 33);

    [[nodiscard]] static ShortsRecipe defaults();
};

// Le os campos presentes no JSON por cima do padrao (o que faltar fica).
[[nodiscard]] std::optional<ShortsRecipe> parseShortsRecipe(std::string_view json, std::string* error = nullptr);

} // namespace MatterEngine::Modeling
