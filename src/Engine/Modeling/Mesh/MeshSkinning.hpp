#pragma once

#include "Engine/Modeling/Mesh/Mesh.hpp"
#include "Engine/Modeling/Skeleton/BodyRegions.hpp"

#include <span>
#include <vector>

namespace MatterEngine::Modeling {

struct AutoSkinSettings {
    // Peso ~ 1 / distancia^power ate os ossos permitidos.
    float falloffPower = 4.0f;
    // Passadas de suavizacao pelos vizinhos na malha.
    int smoothIterations = 2;
    float smoothStrength = 0.5f;
    // Pesos menores que isso somem (e os outros sao renormalizados).
    float minimumWeight = 0.02f;
};

// Pesos automaticos: cada vertice olha os ossos da regiao da parte dele (mais
// o pai da regiao e as regioes filhas, para a juncao), pela distancia aos
// segmentos de cada junta ate as filhas. Pontas (JointKind::Tip) nao pesam.
void computeAutoSkin(Mesh& mesh, const Skeleton& skeleton, const BodyRegions* regions,
    std::span<const int> jointRegion, const AutoSkinSettings& settings = {});

// A junta de maior peso de cada vertice (-1 sem skin).
[[nodiscard]] std::vector<int> dominantJoints(const Mesh& mesh);
// O peso que o vertice da a junta (0 se nao tem).
[[nodiscard]] float weightFor(const SkinWeights& weights, int joint);

} // namespace MatterEngine::Modeling
