#pragma once

#include "Engine/Modeling/Mesh/Mesh.hpp"
#include "Engine/Modeling/Mesh/MeshCut.hpp"

#include <functional>
#include <span>
#include <string>
#include <vector>

// O uniforme feito da propria pele do corpo: cada peca e uma copia da regiao
// que ela cobre, alisada (para parecer pano) e inflada um pouco para fora (a
// folga), cortada nas barras; a pele por baixo sai. Em cada barra uma faixa
// liga a borda da peca a pele (ou a outra peca) um pouco para dentro dela: o
// recuo - olhando para dentro, continuidade em vez de corte seco.
namespace MatterEngine::Modeling {

// Um corte local: a superficie (fica o lado positivo; um plano por
// planeField), a curva dela que passa perto de "center" (a "radius" dele) e
// um ponto do lado que sai.
struct RegionCut {
    CutField field;
    Vec3 center;
    float radius = 0.2f;
    Vec3 beyond;
};

struct RegionResult {
    bool ok = false;
    std::string error;
    // A borda de cada corte (na ordem dada), na ordem de borda do lado que
    // fica.
    std::vector<CutBorder> borders;
};

// Corta a malha nos planos e tira o que fica alem de cada um.
[[nodiscard]] RegionResult keepRegion(Mesh& mesh, std::span<const RegionCut> cuts, const CutSettings& settings = {});
// Corta a malha nos planos (fica o lado da frente de cada um) e tira o
// pedaco que ficou fechado entre eles (o que contem "covered").
[[nodiscard]] RegionResult removeCovered(Mesh& mesh, std::span<const RegionCut> cuts, Vec3 covered,
    const CutSettings& settings = {});

// Como o pano sai da pele: a folga sobe aos poucos ("steps" passos) e a cada
// passo ele alisa so para fora, na direcao da normal ("smoothing" vezes), e
// e empurrado para fora do corpo - passa por cima dos vincos (o gluteo, a
// virilha, a axila) em vez de entrar neles, e nada dobra sobre si.
// O pano caindo: em volta de um eixo (a perna, o tronco), abaixo de uma
// saliencia ele nao volta para dentro mais que "slope" metros por metro de
// descida (o calcao cai reto do gluteo e do quadril em vez de entrar no
// vinco embaixo deles). So os vertices que "joins" aceita contam.
struct ClothHang {
    Vec3 axisPoint;
    Vec3 axisUp { 0.0f, 0.0f, 1.0f };
    float slope = 0.25f;
    // Quanto acima um vertice ainda segura o de baixo e a abertura do angulo
    // (graus) em volta do eixo em que ele segura.
    float reach = 0.3f;
    float angle = 18.0f;
    std::function<bool(std::uint32_t)> joins;
};

struct ClothShape {
    // Voltas de Taubin antes de inflar (arredonda os vincos fundos) e o
    // quanto cada vertice entra nelas (0 a 1; vazio = todos inteiros): so os
    // vincos (a axila, a gola) arredondam e o resto (o ombro) fica com a forma
    // do corpo.
    int relax = 3;
    std::function<float(std::uint32_t)> relaxWeight;
    int steps = 6;
    int smoothing = 2;
    // Voltas no fim para soltar o que ainda se cruza (o pano com ele mesmo).
    int untangle = 20;
    // A queda (depois de inflar, antes de soltar o que se cruza).
    std::vector<ClothHang> hangs;
    // Os vertices fora do meio nao passam de -midlineGap no y (a metade
    // direita: as pernas do calcao nao se cruzam no meio).
    float midlineGap = 0.004f;
};

// Leva os vertices usados pelos triangulos de "cloth" a "loose(vertice)"
// metros para fora de "body" (a pele por baixo; a folga e lida uma vez, nas
// posicoes de antes). A borda (aresta de um triangulo so) infla mas nao
// alisa; os do meio (y = 0) andam so no plano do meio.
void inflateShell(Mesh& cloth, const Mesh& body, const std::function<float(std::uint32_t)>& loose, const ClothShape& shape = {});

// A faixa do recuo entre a borda da peca e a de dentro (a pele ou a outra
// peca), voltas fechadas ou trechos do meio da frente ao de tras; os
// triangulos na parte e no material dados, com a frente certa (a das bordas).
bool stitchRecess(Mesh& mesh, const CutBorder& garmentEdge, const CutBorder& inner, std::uint16_t part,
    std::uint16_t material, std::string* error = nullptr);

} // namespace MatterEngine::Modeling
