#pragma once

#include "Engine/Modeling/Mesh/MeshBuilder.hpp"
#include "Engine/Modeling/Mesh/MeshCut.hpp"

#include <string>
#include <vector>

// A cabeca no pescoco do corpo (metade direita), os dois cortados no mesmo
// plano: o pescoco da cabeca vai tomando, de baixo para cima, a forma do
// corte do corpo (a borda dela cai exatamente na borda do corpo) e as duas
// bordas se encaixam como um ziper - cada lado ganha os vertices do outro, os
// que caem quase juntos viram um so. Nenhuma faixa nova: a costura e uma linha.
namespace MatterEngine::Modeling {

struct NeckJoin {
    // As duas bordas abertas do corte, cada uma do meio da frente ao meio de
    // tras (em qualquer sentido): a de baixo (o corpo) e a de cima (a cabeca).
    std::vector<std::uint32_t> lower;
    std::vector<std::uint32_t> upper;
    // Os vertices de cima que podem ser ajustados, o plano do corte (a
    // normal para cima) e em quantos metros acima dele o ajuste some.
    std::vector<std::uint32_t> upperVertices;
    CutPlane plane;
    float blend = 0.04f;
    // Vertices das duas bordas mais perto que isso viram um so.
    float weld = 0.006f;
};

// Ajusta e encaixa. Os triangulos divididos ficam nas partes deles.
bool joinNeck(Mesh& mesh, const NeckJoin& join, std::string* error = nullptr);

} // namespace MatterEngine::Modeling
