#pragma once

#include "Engine/Modeling/Mesh/Mesh.hpp"
#include "Engine/Modeling/Parts/ShapeField.hpp"
#include "Engine/Modeling/Parts/ShirtRecipe.hpp"

#include <string>
#include <vector>

namespace MatterEngine::Modeling {

// Onde a camisa fica e onde ela costura. A metade direita e montada (y <= 0)
// e o espelho completa o resto: os vertices do meio (frente e costas, o
// fundo do V e o meio da gola atras) sao os mesmos dos dois lados.
struct ShirtPlacement {
    float ground = 0.0f;
    // O braco direito (a junta do ombro e a do cotovelo): a manga em volta.
    Vec3 shoulder;
    Vec3 elbow;
    // As juntas do skin: a coluna (com as alturas delas, do chao) e o ombro
    // (e a auxiliar dele).
    float pelvisHeight = 0.0f;
    float abdomenHeight = 0.0f;
    float chestHeight = 0.0f;
    float upperChestHeight = 0.0f;
    int pelvisJoint = -1;
    int abdomenJoint = -1;
    int chestJoint = -1;
    int upperChestJoint = -1;
    int upperArmJoint = -1;
    int shoulderHelperJoint = -1;
    // O anel de cima do calcao (do meio das costas ao meio da frente, por
    // fora, anti-horario visto de cima) e a parte dele: a dobra da barra
    // costura ali.
    std::vector<std::uint32_t> tuck;
    std::string tuckPart;
    // Sem as pecas de pele, o punho e a gola ficam abertos (declarados).
    bool cuffOpening = true;
    bool collarOpening = true;
    std::string part = "camisa";
    std::string region = "peito";
    std::string side = "direita";
};

struct ShirtSeams {
    bool ok = false;
    std::string error;
    std::string part;
    std::string sleevePart;
    std::string collarPart;
    // A abertura do punho direito (anti-horaria vista de fora do braco, no
    // plano da ponta da manga) e a volta de dentro da gola (metade direita,
    // do fundo do V ate o meio de tras): as pecas de pele comecam ali.
    std::vector<std::uint32_t> cuff;
    std::vector<std::uint32_t> collar;
};

// A forma da camisa (o tronco e a manga direita unidos).
[[nodiscard]] ShapeField buildShirtField(const ShirtRecipe& recipe, const ShirtPlacement& placement);

// A metade direita da camisa, costurada no calcao pela dobra da barra: o
// tronco com a faixa do lado, os paineis do ombro, a manga com o punho e a
// faixa da gola.
[[nodiscard]] ShirtSeams buildShirtHalf(Mesh& mesh, const ShirtRecipe& recipe, const ShirtPlacement& placement);

// Os numeros (so a cor) no peito e nas costas, ja com as duas metades (o
// numero cruza o meio).
[[nodiscard]] bool imprintShirtNumbers(Mesh& mesh, const ShirtRecipe& recipe, float ground, const std::string& part,
    std::string* error = nullptr);

} // namespace MatterEngine::Modeling
