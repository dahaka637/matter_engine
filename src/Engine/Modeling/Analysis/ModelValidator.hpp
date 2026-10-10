#pragma once

#include "Engine/Modeling/Analysis/ConnectivityCheck.hpp"
#include "Engine/Modeling/Analysis/DeformationCheck.hpp"
#include "Engine/Modeling/Analysis/IntersectionCheck.hpp"
#include "Engine/Modeling/Analysis/Report.hpp"
#include "Engine/Modeling/Analysis/SymmetryCheck.hpp"
#include "Engine/Modeling/Analysis/TopologyCheck.hpp"

#include <span>

namespace MatterEngine::Modeling {

struct ValidationOptions {
    TopologySettings topology;
    ConnectivitySettings connectivity;
    bool intersections = true;
    bool symmetry = true;
    SymmetrySettings symmetrySettings;
    bool skin = true;
    bool jointCentering = true;
    // Poses para a checagem de deformacao (vazio = nao checa).
    std::span<const PoseSample> poses;
    DeformationSettings deformation;
};

// Todas as checagens da malha (e dela com o esqueleto), num relatorio so,
// com as estatisticas por parte.
[[nodiscard]] Report validateModel(const Mesh& mesh, const Skeleton* skeleton, const BodyRegions* regions,
    std::span<const int> jointRegion, const ValidationOptions& options = {});

} // namespace MatterEngine::Modeling
