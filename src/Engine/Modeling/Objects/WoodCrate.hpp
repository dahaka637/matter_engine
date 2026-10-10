#pragma once

#include "Engine/Modeling/Objects/ProceduralObject.hpp"

// A caixa de madeira (o tipo "caixa"): um bloco so (6 faces, 12 triangulos)
// com a textura em pixel art pintada pela receita - nos lados a moldura
// (montantes e travessas), as tabuas do painel, o reforco em diagonal e os
// pregos; em cima e embaixo a moldura e as tabuas. A densidade de pixels e a
// mesma em todas as faces (uma caixa comprida tem mais pixels no lado
// comprido). Colisao: uma caixa do tamanho dela.
namespace MatterEngine::Modeling {

[[nodiscard]] ObjectType woodCrateObjectType();

} // namespace MatterEngine::Modeling
