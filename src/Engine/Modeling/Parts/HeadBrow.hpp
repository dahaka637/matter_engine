#pragma once

#include "Engine/Modeling/Mesh/MeshBuilder.hpp"
#include "Engine/Modeling/Parts/HeadRecipe.hpp"

#include <cstdint>
#include <vector>

// A sobrancelha em bloco (como em rosto.png): sobe da pegada recortada na
// pele - a parede em volta, a face de cima e a da frente se encontrando numa
// quina ao longo dela, o fim de dentro fechado. Uma superficie so com o
// rosto: o bloco continua da volta da pegada, vertice com vertice.
namespace MatterEngine::Modeling {

struct BrowPlacement {
    // A volta da pegada, anti-horaria vista de fora: a ponta de fora, a
    // borda de baixo de fora para dentro e a de cima voltando.
    std::vector<std::uint32_t> footprint;
    // As estacoes (de fora para dentro): a de baixo em footprint[1 + k] e a
    // de cima em footprint[footprint.size() - 1 - k].
    std::size_t stations = 0;
    // A normal da pele (para fora) em cada ponto da pegada.
    std::vector<Vec3> normals;
    int headJoint = -1;
};

// Ergue a sobrancelha na parte corrente do builder (material "sobrancelha",
// criado aqui). Pesos: a cabeca.
void buildBrow(MeshBuilder& builder, const HeadRecipe& recipe, const BrowPlacement& brow);

} // namespace MatterEngine::Modeling
