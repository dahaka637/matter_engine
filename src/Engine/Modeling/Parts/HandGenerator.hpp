#pragma once

#include "Engine/Modeling/Mesh/Mesh.hpp"
#include "Engine/Modeling/Parts/HandRecipe.hpp"

#include <array>
#include <string>
#include <vector>

namespace MatterEngine::Modeling {

// Um dedo no esqueleto: as juntas da base (o MCP; no polegar, a do carpo),
// do meio e da ponta, e a ponta; o dorso de cada falange (para onde a unha
// olha: o contrario da flexao) e as juntas do skin delas.
struct DigitPlacement {
    std::array<Vec3, 4> points {};
    std::array<Vec3, 3> dorsal {};
    std::array<int, 3> joints { -1, -1, -1 };
};

// Onde a mao (a direita) fica e onde ela costura: comeca no anel do pulso do
// antebraco (o mesmo numero de pontos) e vai ate a ponta dos dedos.
struct HandPlacement {
    // A junta da mao (o pulso) e as do skin dela.
    Vec3 wrist;
    int handJoint = -1;
    int wristHelperJoint = -1;
    // Indicador, medio, anelar e minimo; o polegar.
    std::array<DigitPlacement, 4> fingers {};
    DigitPlacement thumb;
    // O anel do pulso (anti-horario visto da mao) e a parte do antebraco.
    std::vector<std::uint32_t> wristRing;
    std::string forearmPart;
    std::string part = "mao_direita";
    std::string region = "mao_direita";
};

struct HandSeams {
    bool ok = false;
    std::string error;
    std::string part;
};

// A pele da mao: a palma (do pulso ate a linha dos nos), as membranas, os
// quatro dedos e o polegar (saindo por uma janela na lateral da palma),
// fechada nas pontas.
[[nodiscard]] HandSeams buildHand(Mesh& mesh, const HandRecipe& recipe, const HandPlacement& placement);

} // namespace MatterEngine::Modeling
