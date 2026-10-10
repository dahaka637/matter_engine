#pragma once

#include "Engine/Modeling/Mesh/Mesh.hpp"
#include "Engine/Modeling/Parts/HeadRecipe.hpp"

#include <string>
#include <vector>

namespace MatterEngine::Modeling {

// Onde a cabeca fica e onde ela costura: a metade direita, da abertura de
// cima do pescoco (do meio da frente ao meio de tras) ate o alto.
struct HeadPlacement {
    float ground = 0.0f;
    int neckJoint = -1;
    int headJoint = -1;
    // A volta de cima do pescoco (a metade direita, da frente para tras).
    std::vector<std::uint32_t> neckRing;
    std::string neckPart;
    std::string part = "cabeca";
    std::string region = "cabeca";
};

struct HeadSeams {
    bool ok = false;
    std::string error;
    std::string part;
};

// A metade direita da cabeca: o cranio e o rosto numa grade que passa pelos
// tracos, costurada no pescoco e fechada no alto.
[[nodiscard]] HeadSeams buildHeadHalf(Mesh& mesh, const HeadRecipe& recipe, const HeadPlacement& placement);

} // namespace MatterEngine::Modeling
