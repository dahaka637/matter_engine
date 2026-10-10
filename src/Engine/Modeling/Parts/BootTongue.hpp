#pragma once

#include "Engine/Modeling/Mesh/MeshRelief.hpp"
#include "Engine/Modeling/Parts/BootRecipe.hpp"
#include "Engine/Modeling/Parts/SurfaceDrape.hpp"

namespace MatterEngine::Modeling {

// A almofada da lingua acima do cadarco, em relevo sobre o cabedal (no
// espaco da chuteira): o contorno vem de tongueHalfWidth (o intervalo de t
// tambem), com os cantos e as bordas chanfrados; o topo segue a superficie
// do cabedal (upper) tongueThickness acima. A pegada e o contorno de baixo.
[[nodiscard]] ReliefPiece buildBootTongue(const BootRecipe& recipe, const DrapeField& upper);

} // namespace MatterEngine::Modeling
