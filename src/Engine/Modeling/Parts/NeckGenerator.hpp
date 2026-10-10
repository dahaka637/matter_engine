#pragma once

#include "Engine/Modeling/Mesh/Mesh.hpp"
#include "Engine/Modeling/Parts/NeckRecipe.hpp"

#include <string>
#include <vector>

namespace MatterEngine::Modeling {

// Onde o pescoco fica e onde ele costura: a metade direita, da volta de
// dentro da gola (do meio da frente, no fundo do V, ao meio de tras) ate a
// abertura da cabeca. Os vertices do meio sao os mesmos dos dois lados.
struct NeckPlacement {
    float ground = 0.0f;
    int upperChestJoint = -1;
    int neckJoint = -1;
    int headJoint = -1;
    std::vector<std::uint32_t> collar;
    std::string collarPart;
    std::string part = "pescoco";
    std::string region = "pescoco";
    // Sem a cabeca, a abertura fica declarada.
    bool headOpening = true;
};

struct NeckSeams {
    bool ok = false;
    std::string error;
    std::string part;
    // A abertura da cabeca (metade direita, do meio da frente ao de tras).
    std::vector<std::uint32_t> top;
};

// A pele do pescoco (e do alto do peito no V) da gola ate a cabeca.
[[nodiscard]] NeckSeams buildNeckHalf(Mesh& mesh, const NeckRecipe& recipe, const NeckPlacement& placement);

} // namespace MatterEngine::Modeling
