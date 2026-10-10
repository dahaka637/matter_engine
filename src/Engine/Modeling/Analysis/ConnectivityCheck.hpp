#pragma once

#include "Engine/Modeling/Analysis/Report.hpp"
#include "Engine/Modeling/Mesh/Mesh.hpp"

namespace MatterEngine::Modeling {

struct ConnectivitySettings {
    // Vertices mais perto que isso e nao soldados: costura falsa.
    float duplicateTolerance = 1e-5f;
};

// A malha como uma superficie so (a regra de que nada fica solto):
//  - peca solta: um pedaco ligado que nao chega a nenhuma parte raiz;
//  - costura faltando: partes declaradas ligadas (MeshPart::connectedTo)
//    sem nenhuma aresta em comum;
//  - borda aberta: volta de borda alem das aberturas declaradas da parte
//    (MeshPart::openings);
//  - costura sem solda: vertices no mesmo lugar que nao sao o mesmo;
//  - vertice pincado: a superficie se tocando so por um vertice.
void checkConnectivity(const Mesh& mesh, Report& report, const ConnectivitySettings& settings = {});

} // namespace MatterEngine::Modeling
