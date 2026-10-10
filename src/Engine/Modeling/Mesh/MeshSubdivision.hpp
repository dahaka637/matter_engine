#pragma once

#include "Engine/Modeling/Mesh/Mesh.hpp"
#include "Engine/Modeling/Mesh/MeshCut.hpp"

#include <cstddef>
#include <functional>
#include <span>

// Malha mais densa sem mudar a forma: cada triangulo vira quatro pelos meios
// das arestas (os novos ficam no plano da faceta). O skin dobra uma malha
// grossa em quinas - a dobra so acontece nas arestas que existem; com mais
// aneis, a mesma faixa de pesos dobra em curva, como pano.
namespace MatterEngine::Modeling {

// "levels" vezes. Com "refine", so os triangulos que ele aceita (pelo indice,
// a cada nivel) e os que ficariam com duas ou tres arestas partidas viram
// quatro; os vizinhos com uma aresta partida viram dois (sem vertice solto no
// meio de uma aresta: a malha continua fechada). Do segundo nivel em diante,
// so os filhos de um que virou quatro, com os vizinhos tambem assim (a
// metade de uma metade nao parte de novo: viraria agulha). O vertice novo leva a media
// dos pesos das pontas da aresta (e o mapa, se houver, os meios dos cantos);
// a parte e o material ficam os do triangulo. As bordas dadas (cortes) ganham
// os meios das arestas delas que partiram, na ordem. Devolve quantos
// vertices novos.
std::size_t subdivideTriangles(Mesh& mesh, int levels, std::span<CutBorder* const> borders = {},
    const std::function<bool(std::size_t)>& refine = {});

} // namespace MatterEngine::Modeling
