#pragma once

#include "Engine/Modeling/Analysis/Report.hpp"
#include "Engine/Modeling/Mesh/Mesh.hpp"

#include <vector>

namespace MatterEngine::Modeling {

struct SymmetrySettings {
    // Desvio aceito entre um vertice espelhado (y -> -y) e a malha.
    float tolerance = 0.003f;
};

// Quanto cada vertice espelhado fica longe da superficie (0 = simetrico).
[[nodiscard]] std::vector<float> mirrorDeviation(const Mesh& mesh, float searchRadius);
// Partes que nao batem com o proprio espelho (detalhes de um lado so, como
// o numero no calcao, aparecem aqui de proposito).
void checkSymmetry(const Mesh& mesh, Report& report, const SymmetrySettings& settings = {});

} // namespace MatterEngine::Modeling
