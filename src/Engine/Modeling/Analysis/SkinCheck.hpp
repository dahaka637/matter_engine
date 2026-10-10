#pragma once

#include "Engine/Modeling/Analysis/Report.hpp"
#include "Engine/Modeling/Mesh/Mesh.hpp"
#include "Engine/Modeling/Skeleton/BodyRegions.hpp"

#include <span>

namespace MatterEngine::Modeling {

// Pesos de skin: soma diferente de 1, negativos, sem peso, junta inexistente,
// peso numa ponta, peso dominante de outra parte do corpo (regiao nao
// vizinha) e vertice longe do osso que mais o puxa.
void checkSkin(const Mesh& mesh, const Skeleton& skeleton, const BodyRegions* regions,
    std::span<const int> jointRegion, Report& report);

// Se duas regioes se tocam no esqueleto (uma nasce de uma junta da outra).
[[nodiscard]] bool regionsAdjacent(const Skeleton& skeleton, std::span<const int> jointRegion, int a, int b);

} // namespace MatterEngine::Modeling
