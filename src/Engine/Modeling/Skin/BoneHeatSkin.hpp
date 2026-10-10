#pragma once

#include "Engine/Modeling/Mesh/Mesh.hpp"
#include "Engine/Modeling/Skeleton/Skeleton.hpp"

#include <functional>
#include <optional>
#include <span>
#include <string>

// Os pesos do skin de uma malha pronta (que veio sem eles), pelo "calor dos
// ossos": cada vertice comeca no osso mais perto que ele enxerga por dentro
// da malha (o dedo nao pega o dedo vizinho, a coxa nao pega a outra) e os
// pesos se espalham pela superficie como calor, mais nas partes grossas -
// as juntas dobram macias. Cada junta e um osso ate cada filha dela (a
// pelve ate o abdomen e as coxas; a mao ate a base de cada dedo).
namespace MatterEngine::Modeling {

struct BoneHeatSettings {
    // Quanto o vertice fica preso ao osso mais perto (maior = passagens mais
    // curtas nas juntas) e quantas voltas a difusao da.
    float heat = 12.0f;
    int iterations = 400;
    // Pesos menores que isso saem (e os que ficam somam 1).
    float minimumWeight = 0.02f;
    // Ossos a vista mais longe que o mais perto por menos que isso (fracao da
    // distancia dele, mais 1,5 mm) dividem o vertice com ele: o empate do anel
    // em cima de uma junta.
    float tieBand = 0.03f;
    // Um osso que continua outro (a junta dele e a ponta do osso do pai) nao
    // pega o que fica antes do comeco dele mais que isso (fracao do
    // comprimento): a cintura acima da junta da coxa fica com a pelve.
    float behindStart = 0.02f;
    // Vertices com os pesos pedidos (o resto se acomoda em volta deles); nada
    // = o calor decide.
    std::function<std::optional<SkinWeights>(std::uint32_t)> pin;
    // So os ossos do meio e da direita (a metade direita, espelhada depois).
    bool rightHalf = false;
};

struct BoneHeatResult {
    bool ok = false;
    std::string error;
    // Vertices sem osso a vista (ficaram com o mais perto).
    std::size_t hidden = 0;
};

// Pesa os vertices dos triangulos das partes dadas (vazio = todas) com as
// juntas do esqueleto (menos as pontas e as auxiliares).
[[nodiscard]] BoneHeatResult boneHeatSkin(Mesh& mesh, const Skeleton& skeleton, std::span<const int> parts = {},
    const BoneHeatSettings& settings = {});

} // namespace MatterEngine::Modeling
