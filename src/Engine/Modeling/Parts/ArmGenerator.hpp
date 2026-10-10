#pragma once

#include "Engine/Modeling/Mesh/Mesh.hpp"
#include "Engine/Modeling/Parts/ArmRecipe.hpp"

#include <string>
#include <vector>

namespace MatterEngine::Modeling {

// Onde o braco fica e onde ele costura: comeca na abertura do punho da
// manga (o mesmo numero de pontos em volta) e vai ate o pulso, passando o
// cotovelo (a dobra inteira e desta peca).
struct ArmPlacement {
    // As juntas do ombro e do cotovelo (o osso do braco).
    Vec3 shoulder;
    Vec3 elbow;
    int upperArmJoint = -1;
    int elbowHelperJoint = -1;
    int forearmJoint = -1;
    // O pulso: a auxiliar dele e a mao (a pele perto do pulso ja dobra com
    // ela).
    int wristHelperJoint = -1;
    int handJoint = -1;
    float wristStation = 0.0f;
    // A abertura do punho (anti-horaria vista de fora do braco) e a parte
    // da manga.
    std::vector<std::uint32_t> cuff;
    std::string sleevePart;
    std::string part = "braco_direito";
    std::string region = "braco_direito";
    std::string forearmPart = "antebraco_direito";
    std::string forearmRegion = "antebraco_direito";
    // Sem a mao, o pulso fica aberto (declarado).
    bool wristOpening = true;
};

struct ArmSeams {
    bool ok = false;
    std::string error;
    std::string part;
    std::string forearmPart;
    // O anel do pulso (na junta da mao, anti-horario visto de fora): a mao
    // comeca ali.
    std::vector<std::uint32_t> wrist;
};

// A pele do braco da saida da manga ate o pulso.
[[nodiscard]] ArmSeams buildArm(Mesh& mesh, const ArmRecipe& recipe, const ArmPlacement& placement);

} // namespace MatterEngine::Modeling
