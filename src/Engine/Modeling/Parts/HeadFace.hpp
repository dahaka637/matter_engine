#pragma once

#include "Engine/Modeling/Parts/HeadRecipe.hpp"
#include "Engine/Modeling/Parts/SuperSection.hpp"

#include <array>
#include <cstddef>
#include <string>
#include <vector>

// O rosto sobre a forma base da cabeca. Dois pedacos:
//  - a superficie: a secao por altura mais os campos do rosto (o arco da
//    testa, a orbita, as macas, o perfil do meio espalhado para os lados, o
//    nariz com as asas e as narinas, a quina da mandibula e a pele embaixo
//    dela ate o pescoco), para qualquer ponto (angulo em volta, altura);
//  - os tracos: os vertices fixos (o meio, o olho, a sobrancelha, o nariz,
//    a boca, o queixo, a mandibula), as arestas obrigatorias entre eles e as
//    regioes de outro material (as narinas); a malha da cabeca e
//    triangulada em volta deles.
namespace MatterEngine::Modeling {

// A secao da cabeca numa altura (do chao) e os campos do rosto.
struct HeadShape {
    const HeadRecipe& recipe;
    SectionCurves curves;
    float ground = 0.0f;
    // A altura da volta de cima do pescoco na frente e atras (a pele
    // embaixo da mandibula desce ate ela).
    float neckFront = 1.66f;
    float neckBack = 1.7f;

    [[nodiscard]] SuperSection at(float height) const { return curves.at(height, ground); }
    // O angulo (do meio da frente, lado direito) em que a secao chega a
    // "lateral" do meio.
    [[nodiscard]] float angleForLateral(float height, float lateral) const;
    // O angulo (de 0,3 pi para tras) em que a secao chega ao x dado (atras
    // do meio do lado: a orelha).
    [[nodiscard]] float angleForX(float height, float x) const;
    // O raio medio da secao (o passo do angulo em metros, no dominio).
    [[nodiscard]] float meanRadius(float height) const;
    // O ponto da pele (lado direito) no angulo e na altura, com os campos do
    // rosto e a pele embaixo da mandibula.
    [[nodiscard]] Vec3 surface(float angle, float height) const;
    // So a secao e os campos do rosto (sem a pele embaixo da mandibula).
    [[nodiscard]] Vec3 faceSurface(float angle, float height) const;
    // A normal da pele (para fora) ali.
    [[nodiscard]] Vec3 normal(float angle, float height) const;
};

// Um vertice fixo do rosto: onde (angulo, altura) e a posicao (a pele mais
// o ajuste do traco).
struct FaceVertex {
    float angle = 0.0f;
    float height = 0.0f;
    Vec3 position;
};

// Uma regiao fechada (indices em vertices) de outro material.
struct FaceRegion {
    std::vector<int> loop;
    std::string material;
};

struct FaceLayout {
    std::vector<FaceVertex> vertices;
    // Os do meio da frente (indices em vertices), de baixo para cima.
    std::vector<int> middle;
    // A abertura do olho (8, anti-horaria vista de fora) e o centro dela.
    std::array<int, 8> eye {};
    Vec3 eyeCenter;
    // A pegada da sobrancelha (furo; o bloco sobe dela): a ponta de fora e
    // as estacoes de fora para dentro, a borda de baixo e a de cima.
    int browTip = -1;
    std::vector<int> browBottom;
    std::vector<int> browTop;
    // A raiz da orelha (furo; a orelha sobe dela em HeadEar), um ponto por
    // ponto do contorno, anti-horaria vista de fora.
    std::vector<int> earRoot;
    // As arestas obrigatorias (a linha da boca, os contornos dos labios, o
    // nariz, a mandibula, o sulco do nariz a boca).
    std::vector<std::array<int, 2>> segments;
    std::vector<FaceRegion> regions;

    // A volta da sobrancelha anti-horaria vista de fora: a ponta, a borda de
    // baixo para dentro e a de cima voltando.
    [[nodiscard]] std::vector<int> browLoop() const;
};

[[nodiscard]] FaceLayout faceLayout(const HeadShape& shape);

// O quanto um ponto (lateral, altura) esta perto dos tracos finos do rosto
// (o olho, o nariz, a boca): 1 em cima, 0 longe (o preenchimento fica mais
// denso ali).
[[nodiscard]] float faceDetail(const HeadRecipe& recipe, float lateral, float height);

} // namespace MatterEngine::Modeling
