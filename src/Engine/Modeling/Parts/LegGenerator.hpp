#pragma once

#include "Engine/Modeling/Mesh/Mesh.hpp"
#include "Engine/Modeling/Parts/LegRecipe.hpp"

#include <string>
#include <vector>

namespace MatterEngine::Modeling {

// Onde a perna fica, como ela dobra e onde ela costura.
struct LegPlacement {
    std::string side = "direita";
    std::string shinRegion = "canela_direita";
    std::string thighRegion = "coxa_direita";
    // O eixo do osso visto de cima (x, y) e a altura do chao (z do modelo).
    Point2 axis;
    float ground = 0.0f;
    // A altura (do chao) das juntas e as juntas do skin.
    float ankleHeight = 0.13f;
    float kneeHeight = 0.54f;
    int footJoint = -1;
    int shinJoint = -1;
    int thighJoint = -1;
    // As auxiliares (giram a metade): a frente do joelho fica na do
    // joelho; o meiao saindo da chuteira passa pela do tornozelo.
    int kneeHelperJoint = -1;
    int ankleHelperJoint = -1;
    // O lado de dentro (medial) e +y (+1, perna direita) ou -y (-1).
    float medialSign = 1.0f;
    // A borda da gola da chuteira (anti-horaria vista de cima) e a parte
    // dela: a perna comeca dali.
    std::vector<std::uint32_t> collar;
    std::string bootPart;
    // Sem calcao para costurar no alto, a barra fica declarada aberta.
    bool topOpening = true;
};

// O que a perna deixa para o calcao costurar.
struct LegSeams {
    bool ok = false;
    std::string error;
    // O ultimo anel (a altura da barra), anti-horario visto de cima.
    std::vector<std::uint32_t> top;
    std::string sockPart;
    std::string skinPart;
};

// A perna inteira, uma superficie so costurada na gola da chuteira: o meiao
// (branco, saindo da gola), a faixa preta, a pele do joelho e da coxa ate a
// barra do calcao. Facetada como a referencia e com os pesos de skin (pe,
// canela e coxa, com as transicoes no tornozelo e no joelho).
[[nodiscard]] LegSeams buildLeg(Mesh& mesh, const LegRecipe& recipe, const LegPlacement& placement);

} // namespace MatterEngine::Modeling
