#pragma once

#include "Engine/Modeling/Mesh/Mesh.hpp"

#include <functional>
#include <span>
#include <string>

// Os pesos do skin espalhados pela superficie: cada volta leva os pesos de
// cada vertice uma fracao para a media dos vizinhos (as quatro maiores ficam,
// somando 1). Para o pano: a passagem de uma junta para a outra fica larga e
// o tecido estica por cima da dobra em vez de vincar.
namespace MatterEngine::Modeling {

void smoothSkinWeights(Mesh& mesh, int iterations, float amount = 0.5f);
// O mesmo so nos vertices que "moves" aceita (os outros seguram os pesos e
// servem de vizinhos).
void smoothSkinWeights(Mesh& mesh, int iterations, float amount, const std::function<bool(std::uint32_t)>& moves);

// Na metade direita de um corpo espelhado: perto do meio (a menos de
// "distance" do plano y = 0) os pesos vao aos poucos para a media com o
// espelho deles (no meio, metade de cada lado): a passagem de uma coxa para
// a outra no gancho fica larga em vez de mudar de lado em milimetros.
// "jointMirror": a junta do outro lado de cada junta (a do meio e ela mesma).
void blendMidlineWeights(Mesh& mesh, std::span<const int> jointMirror, float distance);

// Perto dos pontos dados (a menos de "radius" do mais perto, sumindo ate
// la) os vertices dao a "joint" uma fracao "amount" dos pesos que nao sao
// dela: a costura do gancho do calcao presa na pelve (as coxas cruzando nao
// viram o pano entre elas).
void pinWeights(Mesh& mesh, std::span<const Vec3> anchors, float radius, int joint, float amount);

// No vertice, a fracao "fraction" dos pesos das juntas "from" passa para "to"
// (as quatro maiores ficam, somando 1).
void moveWeights(Mesh& mesh, std::uint32_t vertex, std::span<const int> from, int to, float fraction);

// No vertice, o peso que ele tem nas juntas de uma cadeia (a de cima, a
// auxiliar no meio, a de baixo: pelve, quadril, coxa) vai para o ponto "t"
// dela (0 a de cima, 0,5 a do meio, 1 a de baixo, em linha entre vizinhas);
// o peso nas outras juntas fica. Numa faixa, t subindo devagar espalha a
// dobra da junta pela faixa toda. "amount" mistura com o que o vertice tinha
// (0 fica como estava, 1 so a cadeia): a faixa volta macia aos pesos de
// antes nas pontas. "straight" troca essa fracao dos aneis pela passagem
// direta da primeira para a ultima junta (1 - t e t, sem as do meio): o pano
// esticado em linha reta de uma a outra em vez do arco em volta da junta.
void chainWeights(Mesh& mesh, std::uint32_t vertex, std::span<const int> chain, float t, float amount = 1.0f, float straight = 0.0f);

// Os pesos espalhados so em volta de pontos medidos (o avesso que uma pose
// mostra, corrigido no lugar): os vertices a menos do raio de algum ponto,
// "iterations" voltas de smoothSkinWeights, os outros parados. Com "part", so
// os vertices que sao so dela (a costura com outra parte fica: a chuteira
// rigida nao pega peso da canela).
struct WeightRelaxPoint {
    Vec3 at;
    float radius = 0.02f;
    std::string part;
};
void relaxWeights(Mesh& mesh, std::span<const WeightRelaxPoint> points, int iterations);

} // namespace MatterEngine::Modeling
