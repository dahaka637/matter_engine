#pragma once

#include "Engine/Modeling/Mesh/Mesh.hpp"

#include <cstddef>
#include <functional>
#include <span>

// Reparos que nao mudam a superficie: so a triangulacao.
namespace MatterEngine::Modeling {

// Desfaz as agulhas planas: o triangulo de area quase zero com um vertice no
// meio da aresta comprida (a juncao em T de um recorte, por exemplo) troca
// essa aresta pela que liga o vertice ao outro lado do vizinho. A superficie
// fica a mesma (a agulha nao tinha area); os dois triangulos novos ficam com
// a parte e o material do vizinho. Sem isso, na pose o skin tira o vertice
// do meio da reta e a agulha vira do avesso. "flatness": a altura maxima do
// vertice do meio, em fracao da aresta (e no maximo "maximumHeight" metros:
// a superficie muda no maximo isso). Devolve quantas foram desfeitas.
std::size_t flipFlatNeedles(Mesh& mesh, float flatness = 0.012f, float maximumHeight = 0.001f);

// Triangulos melhores pela troca da diagonal (o criterio de Delaunay): dois
// vizinhos da mesma parte quase no mesmo plano (dobrados no maximo
// "maximumBend" graus) trocam a aresta entre eles pela outra diagonal do
// quadrilatero quando o menor angulo dos dois sobe pelo menos "gain" graus.
// Os vertices nao mudam (nem os pesos); a superficie muda no maximo a dobra
// do quadrilatero. So as partes dadas (vazio = todas). Devolve as trocas.
struct TriangleFlipSettings {
    float maximumBend = 30.0f;
    float gain = 4.0f;
    int passes = 10;
};
std::size_t improveTriangles(Mesh& mesh, std::span<const int> parts = {}, const TriangleFlipSettings& settings = {});

// Junta as arestas curtas demais para a vizinhanca (menores que "ratio"
// vezes a media das arestas em volta das duas pontas): um vertice vai para o
// outro e os triangulos da aresta saem. Quem fica: o que "keep" segura (a
// borda de um corte, o meio de uma metade) - se os dois sao seguros, a aresta
// fica. So quando a malha continua uma variedade e nenhum triangulo vira
// nem fica fino (menor angulo de 12 graus).
// O vertice que sai fica solto (compactar depois). Devolve as juncoes.
std::size_t collapseShortEdges(Mesh& mesh, float ratio, const std::function<bool(std::uint32_t)>& keep);

// Os vertices que "goes" aceita vao para o vizinho mais perto que "into"
// aceita (a faixa colada numa costura vai para a costura), com as mesmas
// garantias (variedade, nada vira, nenhum triangulo com menor angulo abaixo
// de "minimumAngle" graus). Devolve as juncoes.
std::size_t collapseInto(Mesh& mesh, const std::function<bool(std::uint32_t)>& goes, const std::function<bool(std::uint32_t)>& into,
    float minimumAngle = 6.0f);

} // namespace MatterEngine::Modeling
