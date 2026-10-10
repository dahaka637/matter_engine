#pragma once

#include "Engine/Modeling/Geometry/ModelGeometry.hpp"
#include "Engine/Modeling/Mesh/Mesh.hpp"
#include "Engine/Modeling/Raster/Color.hpp"

#include <array>
#include <optional>
#include <string>
#include <vector>

// O rosto pintado (a cabeca base fica como veio, sem olho modelado): os
// olhos (a amendoa com o branco, a iris, a pupila, a linha dos cilios e a
// sombra da dobra da palpebra), as sobrancelhas e os labios, como funcao do
// ponto da superficie. Medidas em metros; alturas do chao; o lado direito
// (y < 0), o esquerdo e o espelho.
namespace MatterEngine::Modeling {

struct FacePaintShape {
    // O olho: o centro (y e altura), para onde ele olha (graus para fora) e
    // a amendoa: a meia largura, a altura de cima e a de baixo (do centro) e
    // a inclinacao (o canto de fora sobe isso por metro de largura).
    float eyeY = -0.0375f;
    float eyeHeight = 1.692f;
    float eyeYaw = 14.0f;
    float eyeHalfWidth = 0.0148f;
    float eyeUpper = 0.0042f;
    float eyeLower = 0.0026f;
    float eyeTilt = 0.08f;
    float irisRadius = 0.0056f;
    float pupilRadius = 0.0022f;
    // A curva da palpebra de cima (o expoente: menor = mais reta, o olho
    // "encoberto" de cacador).
    float upperLidCurve = 0.62f;
    // A linha dos cilios (em cima) e a sombra da dobra acima dela (a largura
    // e o quanto ela escurece).
    float lashWidth = 0.0013f;
    float creaseWidth = 0.0035f;
    float creaseShade = 0.2f;
    // A sobrancelha: de "browInner" (para dentro, do centro do olho) a
    // "browOuter" (para fora); a borda de baixo acima do centro e a
    // espessura, por dentro e por fora.
    float browInner = 0.024f;
    float browOuter = 0.021f;
    float browLowInner = 0.0088f;
    float browLowOuter = 0.0108f;
    float browThickInner = 0.0085f;
    float browThickOuter = 0.0045f;
    // A boca nas arestas da cabeca base, cada linha do meio ao canto em
    // pontos [|y|, altura] (os vertices dela): a fresta (o vinco entre a faixa
    // de cima e a face clara) e o alto do labio de cima (o arco do cupido).
    // As duas terminam no canto da boca. Abaixo da fresta e pele: a face
    // clara nao e labio.
    std::vector<std::array<float, 2>> mouthSlit { { 0.0f, 1.6223f }, { 0.0088f, 1.6215f }, { 0.0266f, 1.6163f } };
    std::vector<std::array<float, 2>> upperLipTop { { 0.0f, 1.6269f }, { 0.0067f, 1.6289f }, { 0.0266f, 1.6163f } };
    // A orelha: a placa de fora e o que fica mais longe do meio que isso
    // (earPlate; a pintura dela e uma camada do desenho, PaintDesign).
    float earOutside = 0.076f;

    Color sclera = rgb(214, 206, 194);
    Color iris = rgb(70, 48, 32);
    Color pupil = rgb(16, 12, 10);
    Color lash = rgb(34, 25, 20);
    Color brow = rgb(40, 30, 24);
    Color mouthLine = rgb(104, 58, 46);
};

// O rosto pronto para pintar: a forma e onde a superficie do olho esta
// (o x da frente no centro do olho, medido na malha).
struct FaceLayout {
    FacePaintShape shape;
    float ground = 0.0f;
    float eyeSurfaceX = 0.0f;
};

// A placa da orelha direita: o contorno no plano [x, altura do chao] e a
// elipse da mesma area e dos mesmos momentos (o centro, o eixo maior unitario
// e os dois semieixos).
struct EarPlate {
    std::vector<std::array<float, 2>> outline;
    std::array<float, 2> center {};
    std::array<float, 2> axis { 1.0f, 0.0f };
    float major = 0.0f;
    float minor = 0.0f;
};

// A placa da orelha direita: os triangulos da parte "headPart" virados para
// fora (o lado direito, -y) mais longe do meio que "minSide", a maior peca
// ligada deles, a volta da borda dela no plano [x, altura do chao] e a elipse
// dela. Sem contorno se nao achar.
[[nodiscard]] EarPlate earPlate(const Mesh& mesh, int headPart, float minSide, float ground);

// A cor do rosto no ponto (o lado esquerdo vira o direito), ou nada fora
// dos olhos, das sobrancelhas e da boca. "skin": a cor da pele ali.
[[nodiscard]] std::optional<Color> paintFace(const FaceLayout& layout, Vec3 point, Vec3 normal, Color skin);

} // namespace MatterEngine::Modeling
