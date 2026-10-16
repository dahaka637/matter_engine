#pragma once

#include "Engine/Athlete/Reference/ReferenceFrame3D.hpp"
#include "Engine/Character/DefaultLocomotion3D.hpp"

namespace MatterEngine {

// A casca de migracao da referencia (a reescrita, R4): a saida do padrao de
// hoje (DefaultLocomotion3D) como o movimento que a referencia le. Sem copia
// da pose: os ponteiros valem ate o proximo update do padrao.
[[nodiscard]] ReferenceMotion3D legacyReferenceMotion3D(const DefaultLocomotionOutput3D& output);

} // namespace MatterEngine
