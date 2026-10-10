#pragma once

#include "Engine/Modeling/Parts/ProfileCurve.hpp"
#include "Engine/Modeling/Raster/Color.hpp"

#include <array>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace MatterEngine::Modeling {

// A cabeca em numeros (partes/cabeca.json): o cranio e o rosto da abertura
// do pescoco ate o alto, uma superficie so com ele. Alturas em metros do
// chao; para o lado, a distancia do meio (o lado direito; o esquerdo e o
// espelho); para a frente, o x do modelo.
struct HeadRecipe {
    // A forma base por altura: secoes horizontais de quatro quadrantes em
    // volta de (centerX, 0) - ate onde vai para a frente (o rosto) e para
    // tras (a nuca) a partir do centro, a meia largura e os expoentes (o do
    // lado: frente e costas mais retas quanto maior; o da frente: os lados
    // mais fechados quanto menor).
    ProfileCurve centerX;
    ProfileCurve front;
    ProfileCurve back;
    ProfileCurve side;
    ProfileCurve exponent;
    ProfileCurve sideExponent;
    // Os expoentes da metade da frente (o rosto): menores = o rosto mais
    // redondo visto de cima (os olhos e os cantos da boca mais para tras).
    ProfileCurve frontExponent;
    ProfileCurve frontSideExponent;
    // O alto da cabeca.
    float crownHeight = 1.905f;

    // A malha: os tracos do rosto sao vertices fixos e o resto e
    // preenchido (triangulado em volta deles) com este espacamento no rosto,
    // perto dos tracos (fracao do do rosto) e no cranio; o cranio fecha numa
    // fileira (altura, pontos) e no alto.
    float faceSpacing = 0.014f;
    float detailSpacing = 0.5f;
    float skullSpacing = 0.024f;
    float capHeight = 1.884f;
    int capColumns = 12;

    // O perfil do meio sem o nariz (o queixo, os labios, a base do nariz, a
    // raiz entre os olhos, a glabela e a testa): o x exato no meio. Os
    // outros campos moldam os lados; este acerta o meio, espalhado ate a meia
    // largura dada (some fora dela).
    ProfileCurve profile;
    ProfileCurve profileWidth;
    // O nariz por cima do rosto: o x do meio (da base, entre os labios, ate
    // a raiz entre os olhos), a meia largura em que encontra o rosto e o
    // expoente do corte (maior = frente mais reta e lados mais a pique).
    ProfileCurve noseProfile;
    ProfileCurve noseWidth;
    ProfileCurve noseShape;
    // As asas do nariz (o centro, os raios para o lado e para cima e o
    // quanto saltam do rosto) e as narinas por baixo da ponta (o centro, os
    // raios e o quanto entram).
    float alaLateral = 0.013f;
    float alaHeight = 1.746f;
    float alaRadius = 0.0062f;
    float alaHalfHeight = 0.0074f;
    float alaDepth = 0.011f;
    float nostrilLateral = 0.0076f;
    float nostrilHeight = 1.7405f;
    float nostrilRadius = 0.0054f;
    float nostrilHalfHeight = 0.0029f;
    float nostrilDepth = 0.0015f;
    // A boca: a linha no meio (a altura), a meia largura ate o canto, o
    // quanto a linha entra no meio e no canto; a borda do labio de cima no
    // meio e nos picos do arco (acima da linha) e a do de baixo (abaixo).
    float mouthHeight = 1.715f;
    float mouthWidth = 0.0315f;
    float mouthDepth = 0.0018f;
    float mouthCornerDepth = 0.003f;
    float upperLip = 0.006f;
    float lipPeak = 0.0085f;
    float lowerLip = 0.009f;
    // O quanto os labios afinam ate os cantos (0 = a mesma altura em toda a
    // boca; 1 = no canto, a metade) e as almofadas deles (a meia largura e o
    // quanto saltam): o labio de cima e o de baixo, mais cheio e mais
    // estreito que a boca.
    float lipTaper = 1.1f;
    float upperPadWidth = 0.024f;
    float upperPadDepth = 0.002f;
    float lowerPadWidth = 0.019f;
    float lowerPadDepth = 0.004f;
    // O olho: o centro, a meia largura, a abertura na pele (de cima a
    // baixo; o globo aparece dentro dela menos a borda da palpebra), o
    // quanto o canto de fora sobe e o quanto a abertura fica mais funda que a
    // pele em volta.
    float eyeLateral = 0.0328f;
    float eyeHeight = 1.7878f;
    float eyeHalfWidth = 0.0184f;
    float eyeOpening = 0.0146f;
    float eyeTilt = 0.0f;
    float eyeDepth = 0.002f;
    // O olho por dentro da abertura (HeadEye): a grossura da borda da
    // palpebra, o recuo do globo atras dela, o quanto o meio do globo salta
    // (o domo da iris) e o raio da iris.
    float lidThickness = 0.0014f;
    float eyeGap = 0.0006f;
    float eyeBulge = 0.0012f;
    float irisRadius = 0.0076f;
    // As sobrancelhas (HeadBrow): a borda de cima e a de baixo (a altura
    // pela distancia do meio; a primeira chave e o fim de dentro, cortado em
    // diagonal), a ponta de fora, a grossura do bloco no meio e na borda.
    ProfileCurve browTop;
    ProfileCurve browBottom;
    float browTipLateral = 0.063f;
    float browTipHeight = 1.799f;
    float browThickness = 0.003f;
    float browEdge = 0.0009f;
    // O arco da testa sob as sobrancelhas (a altura, a meia altura, ate onde
    // vai para o lado e o quanto salta), a orbita em volta do olho (os raios
    // e o quanto afunda) e as macas do rosto (o centro, o raio e o quanto
    // saltam).
    float ridgeHeight = 1.803f;
    float ridgeHalfHeight = 0.009f;
    float ridgeReach = 0.06f;
    float ridgeDepth = 0.003f;
    float socketWidth = 0.022f;
    float socketHeight = 0.014f;
    float socketDepth = 0.003f;
    float cheekLateral = 0.051f;
    float cheekHeight = 1.769f;
    float cheekRadius = 0.022f;
    float cheekDepth = 0.005f;
    // A bochecha cavada sob a maca, do lado da boca (o centro, o raio e o
    // quanto afunda).
    float hollowLateral = 0.05f;
    float hollowHeight = 1.727f;
    float hollowRadius = 0.02f;
    float hollowDepth = 0.0025f;
    // A mandibula: a linha de baixo (a altura pelo angulo em volta, em
    // graus: do meio do queixo ao canto e a subida ate a orelha), o quanto a
    // borda salta logo acima dela (e ate que altura acima) e a curva da pele
    // embaixo, do pescoco a linha (maior = reta no pescoco e virando para
    // baixo so perto da quina).
    ProfileCurve jawLine;
    float jawRidge = 0.0015f;
    float jawRidgeHeight = 0.012f;
    float jawCurve = 3.0f;

    // A orelha (HeadEar): o centro da concha (o x do modelo e a altura), o
    // contorno visto de lado ([para a frente, para cima, o quanto se afasta
    // da cabeca] a partir do centro, anti-horario visto de fora), os raios da
    // borda da concha, a raiz (a volta que gruda na pele, em vezes a borda da
    // concha) e a grossura da borda da helice.
    float earX = -0.0397f;
    float earHeight = 1.761f;
    std::vector<std::array<float, 3>> earOutline;
    float earConchaWidth = 0.009f;
    float earConchaHeight = 0.0085f;
    float earRoot = 1.25f;
    float earRim = 0.003f;

    // O facetado (para dentro/fora, metros) e a semente do preenchimento.
    float jitter = 0.0005f;
    unsigned seed = 83;

    Color skinColor = rgb(212, 152, 106);
    Color scleraColor = rgb(204, 186, 164);
    Color irisColor = rgb(30, 24, 22);
    Color lashColor = rgb(52, 38, 32);
    Color browColor = rgb(44, 36, 32);
    Color nostrilColor = rgb(84, 54, 42);
    Color canalColor = rgb(150, 98, 70);

    [[nodiscard]] static HeadRecipe defaults();
};

[[nodiscard]] std::optional<HeadRecipe> parseHeadRecipe(std::string_view json, std::string* error = nullptr);

} // namespace MatterEngine::Modeling
