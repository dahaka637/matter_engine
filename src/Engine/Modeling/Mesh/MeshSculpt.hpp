#pragma once

#include "Engine/Modeling/Mesh/Mesh.hpp"

#include <functional>
#include <span>
#include <string>

// Ajustes finos de forma numa malha pronta: cada puxao leva os vertices perto
// de um ponto junto com ele, cheio ate 40% do raio e sumindo macio ate o raio
// (o queixo mais marcado, a papada recolhida). Na metade de um corpo
// espelhado, os vertices do meio (y = 0) ficam no meio.
namespace MatterEngine::Modeling {

struct SculptPull {
    Vec3 at;
    float radius = 0.01f;
    Vec3 move;
};

// "affects" escolhe os vertices (nada = todos); "midline": os de y = 0 nao
// saem do plano do meio.
void sculptMesh(Mesh& mesh, std::span<const SculptPull> pulls, const std::function<bool(std::uint32_t)>& affects = {},
    bool midline = true);

// Um vertice levado a um lugar exato: o mais perto de "at" vai para "to" (a
// forma de um rosto low-poly sao os vertices dele; cada um e uma quina).
struct VertexMove {
    Vec3 at;
    Vec3 to;
};

// Cada movimento acha o vertice mais perto de "at" (a menos de "tolerance";
// senao, erro: a malha mudou) entre os que "affects" aceita. Os do meio
// (y = 0) ficam no meio. Todos leem as posicoes de antes.
bool moveVertices(Mesh& mesh, std::span<const VertexMove> moves, float tolerance, std::string& error,
    const std::function<bool(std::uint32_t)>& affects = {});


} // namespace MatterEngine::Modeling
