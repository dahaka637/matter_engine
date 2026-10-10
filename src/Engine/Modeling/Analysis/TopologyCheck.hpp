#pragma once

#include "Engine/Modeling/Analysis/Report.hpp"
#include "Engine/Modeling/Mesh/Mesh.hpp"

namespace MatterEngine::Modeling {

struct TopologySettings {
    // Triangulo "agulha": menor angulo abaixo disso. A agulha no plano de uma
    // vizinha (ate flatSliverDegrees: um pedaco da mesma face) e so observacao.
    float minimumAngleDegrees = 2.0f;
    float flatSliverDegrees = 2.0f;
    float minimumArea = 1e-10f;
};

// Arestas nao-manifold, vizinhos com orientacao trocada, triangulos
// degenerados ou agulha, vertices fora de triangulos e pedacos fechados com
// as normais para dentro. (Bordas, costuras e pecas soltas: ConnectivityCheck.)
void checkTopology(const Mesh& mesh, Report& report, const TopologySettings& settings = {});

} // namespace MatterEngine::Modeling
