#pragma once

#include "Engine/Modeling/Mesh/Mesh.hpp"
#include "Engine/Modeling/Parts/ShortsRecipe.hpp"

#include <string>
#include <vector>

namespace MatterEngine::Modeling {

// Onde o calcao fica e onde ele costura. A metade direita e montada (y <= 0)
// e o espelho completa o resto: os vertices do plano do meio (a frente, as
// costas e a virilha) sao os mesmos dos dois lados.
struct ShortsPlacement {
    // O eixo da coxa direita (visto de cima), o x do centro do corpo e o chao.
    Point2 legAxis;
    float bodyX = 0.0f;
    float ground = 0.0f;
    // As juntas do skin (os do meio levam as duas coxas por igual).
    int pelvisJoint = -1;
    int abdomenJoint = -1;
    int rightThighJoint = -1;
    int leftThighJoint = -1;
    // As auxiliares da virilha (a metade do giro da coxa): a perna do calcao
    // perto do assento passa por elas.
    int rightHipHelperJoint = -1;
    int leftHipHelperJoint = -1;
    // O anel de cima da perna direita (anti-horario visto de cima, na altura
    // da barra) e a parte dela: a barra costura ali.
    std::vector<std::uint32_t> legTop;
    std::string legPart;
    // Com a camisa por cima: o calcao para nessa altura (sem o cos), onde a
    // dobra da barra dela costura; 0 = sem camisa (o cos fica aberto).
    float coverHeight = 0.0f;
    // A faixa (alturas do chao) em que a cintura passa da pelve para o
    // abdomen; vazia (0, 0) = o cos puxa um pouco o abdomen so no alto.
    float abdomenFrom = 0.0f;
    float abdomenTo = 0.0f;
    std::string part = "calcao";
    std::string region = "quadril";
};

struct ShortsSeams {
    bool ok = false;
    std::string error;
    std::string part;
    // O anel de cima (do meio de tras ao meio da frente, anti-horario visto
    // de cima): o cos aberto, ou onde a camisa costura.
    std::vector<std::uint32_t> top;
};

// A metade direita do calcao, costurada na coxa pela barra: a perna do
// calcao (com a faixa do lado), a virilha (a costura no meio, entre as
// pernas), o quadril e o cos aberto em cima (declarado: o tronco vem dali),
// ou, com a camisa, o anel onde ela costura.
[[nodiscard]] ShortsSeams buildShortsHalf(Mesh& mesh, const ShortsRecipe& recipe, const ShortsPlacement& placement);

// O numero desenhado (so a cor) na frente da perna esquerda do calcao, ja
// espelhado: "leftLegAxis" e o eixo da coxa esquerda.
[[nodiscard]] bool imprintShortsNumber(Mesh& mesh, const ShortsRecipe& recipe, Point2 leftLegAxis, float ground,
    const std::string& part, std::string* error = nullptr);

} // namespace MatterEngine::Modeling
