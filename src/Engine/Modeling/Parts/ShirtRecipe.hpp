#pragma once

#include "Engine/Modeling/Parts/ProfileCurve.hpp"
#include "Engine/Modeling/Raster/Color.hpp"

#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace MatterEngine::Modeling {

// A camisa em numeros (partes/camisa.json): uma camada so, da barra (por
// cima do calcao) ate a gola e os punhos. A forma vem de um campo (o tronco
// em secoes e a manga em volta do braco, unidos com mistura suave); a malha
// e um molde de costura: aneis do tronco ate a axila, os paineis da frente e
// de tras ate o ombro, a manga saindo da cava. Alturas em metros do chao;
// ao longo do braco, distancias desde a junta do ombro.
struct ShirtRecipe {
    // O tronco do campo, por altura: o x do centro (a costura do lado), ate
    // onde vai para a frente e para tras a partir dele, a meia largura e os
    // expoentes da superelipse: o da frente e costas (2 = elipse; maior =
    // mais retas) e o do lado (menor = o lado mais fechado: a faixa aparece
    // de frente e de costas, como na referencia).
    ProfileCurve torsoCenterX;
    ProfileCurve torsoFront;
    ProfileCurve torsoBack;
    ProfileCurve torsoSide;
    ProfileCurve torsoExponent;
    ProfileCurve torsoSideExponent;
    // A manga do campo ao longo do braco: o raio, quanto o centro sobe (z) e
    // vai para a frente (x) em relacao ao osso; e a mistura com o tronco (a
    // axila e o ombro arredondados).
    ProfileCurve sleeveRadius;
    ProfileCurve sleeveLift;
    ProfileCurve sleeveForward;
    float sleeveBlend = 0.035f;

    // A barra, a axila (o ultimo anel inteiro do tronco) e os aneis entre
    // elas; e ate onde o calcao sobe por dentro (a dobra da barra costura
    // nele ali).
    float hemHeight = 1.01f;
    float armpitHeight = 1.43f;
    std::vector<float> rings;
    float tuckHeight = 1.045f;
    // A folga minima da barra por fora do calcao (o desenho do tronco pode
    // ser mais fechado que o calcao: ali a barra abre o que faltar).
    float hemClearance = 0.012f;
    // Colunas da metade do tronco: as de tras (do meio das costas ate a
    // cava) e as da frente (da cava ate o meio da frente); a faixa do lado
    // (centrada na costura, meia largura por altura).
    int backColumns = 7;
    int frontColumns = 7;
    ProfileCurve stripeHalfWidth;

    // A cava: o plano dela (a distancia do meio do corpo, no y) e os passos
    // de cada lado (da axila ate o ombro, na frente e atras). O ponto do
    // ombro fica no alto do contorno (graus a partir do topo; + para tras).
    float armholeDepth = 0.19f;
    int armholeRows = 8;
    float shoulderPointAngle = 0.0f;

    // A gola: a borda de fora vista de frente (a altura do V pela distancia
    // do meio) e de costas (a da volta de tras), e o ponto do pescoco no alto
    // do ombro (x e a distancia do meio, vistos de cima), onde as duas se
    // encontram; os passos do V e de tras; a largura da faixa, quanto ela
    // salta da camisa e quanto a borda de dentro desce ate a pele.
    ProfileCurve vNeckHeight;
    ProfileCurve backNeckHeight;
    float neckPointX = -0.045f;
    float neckPointSide = 0.097f;
    int vSteps = 4;
    int backNeckSteps = 4;
    float collarWidth = 0.025f;
    float collarRaise = 0.002f;
    float collarDepth = 0.005f;

    // A manga: os aneis (distancia desde o ombro), o comprimento ate a ponta
    // (no eixo), a inclinacao da ponta (radianos; o alto vai mais longe), a
    // largura do punho, o raio da abertura (onde o braco sai) e o quanto o
    // centro dela vai para a frente do osso.
    std::vector<float> sleeveRings;
    float sleeveLength = 0.262f;
    float sleeveEndTilt = 0.2f;
    float cuffWidth = 0.036f;
    float cuffOpening = 0.051f;
    float cuffOpeningForward = 0.0f;

    // O numero (so a cor): o texto e, na frente e nas costas, a altura dos
    // digitos e a do centro.
    std::string number = "10";
    float frontNumberHeight = 0.106f;
    float frontNumberCenter = 1.389f;
    float backNumberHeight = 0.176f;
    float backNumberCenter = 1.388f;
    // A grossura do traco dos digitos (fracao da altura).
    float numberStroke = 0.19f;

    // O facetado: para dentro/fora e ao longo da superficie (m; para cima e
    // para baixo nos aneis), o quanto as colunas andam ao longo do anel
    // (fracao do passo) e a semente; e as passadas de alisamento dos paineis
    // (o espacamento fica igual).
    float jitter = 0.0015f;
    float slideJitter = 0.012f;
    float ringSlide = 0.25f;
    unsigned seed = 37;
    int smoothing = 12;

    Color shirtColor = rgb(236, 234, 232);
    Color trimColor = rgb(34, 34, 37);
    Color insideColor = rgb(206, 204, 201);
    Color numberColor = rgb(30, 30, 33);

    [[nodiscard]] static ShirtRecipe defaults();
};

// Le os campos presentes no JSON por cima do padrao (o que faltar fica).
[[nodiscard]] std::optional<ShirtRecipe> parseShirtRecipe(std::string_view json, std::string* error = nullptr);

} // namespace MatterEngine::Modeling
