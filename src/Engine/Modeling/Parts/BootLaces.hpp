#pragma once

#include "Engine/Modeling/Parts/BootRecipe.hpp"
#include "Engine/Modeling/Parts/SurfaceDrape.hpp"

#include <vector>

namespace MatterEngine::Modeling {

// O cadarco no espaco da chuteira: cordoes com as duas pontas entrando no
// cabedal, como um cadarco de verdade.
struct BootLacePieces {
    // O X embaixo das tiras: primeiro as diagonais de baixo, depois as de
    // cima (que passam por cima das de baixo); as pontas entram no cabedal
    // escondidas debaixo das tiras de cima.
    std::vector<WeldedStrap> crosses;
    // As tiras de cima, esticadas em arco, as pontas entrando nos ilhoses.
    std::vector<WeldedStrap> bars;
    // As fendas escuras dos ilhoses (so a cor muda), uma em volta de cada
    // ponta de tira.
    std::vector<std::vector<Vec3>> eyelets;
};

// Monta o cadarco sobre o campo (que comeca com o cabedal e a lingua). Cada
// camada entra no campo: as diagonais de cima passam por cima das de baixo
// e as tiras por cima de tudo, sempre com folga; so as pontas tocam o
// cabedal (entram nele). Vazio com "error" se alguma tira nao fechar.
[[nodiscard]] BootLacePieces buildBootLaces(const BootRecipe& recipe, DrapeField& field, std::string* error = nullptr);

} // namespace MatterEngine::Modeling
