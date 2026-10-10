#pragma once

#include "Engine/Modeling/Analysis/Report.hpp"
#include "Engine/Modeling/Mesh/Mesh.hpp"

#include <span>
#include <string_view>

namespace MatterEngine::Modeling {

struct IntersectionSettings {
    // Pontos guardados por grupo (para destacar e enquadrar).
    std::size_t maximumSamples = 40;
};

// Triangulos que se cruzam: na mesma parte e autointersecao, entre partes
// uma atravessando a outra; na ligacao os dois sao erro (as pecas se ligam
// por costura, nunca por sobreposicao), numa pose sao aviso. Vizinhos
// (vertice em comum, ou no mesmo lugar) nao contam. "positions" permite
// checar numa pose; context entra no codigo e na mensagem (o nome da pose).
void checkIntersections(const Mesh& mesh, std::span<const Vec3> positions, Report& report,
    std::string_view context = {}, const IntersectionSettings& settings = {});

} // namespace MatterEngine::Modeling
