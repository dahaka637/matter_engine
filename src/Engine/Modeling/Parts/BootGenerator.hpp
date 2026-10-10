#pragma once

#include "Engine/Modeling/Mesh/Mesh.hpp"
#include "Engine/Modeling/Parts/BootRecipe.hpp"

#include <string>
#include <vector>

namespace MatterEngine::Modeling {

// Onde a chuteira fica e como ela dobra.
struct BootPlacement {
    // Do espaco da chuteira para o do modelo: origem no chao atras do
    // calcanhar; x para o bico, y para dentro do pe, z para cima.
    Transform frame;
    // O fim dos nomes das partes (direita/esquerda) e a regiao do corpo.
    std::string side = "direita";
    std::string region = "pe_direito";
    // As juntas do skin: o pe e os dedos (a frente dobra com eles).
    int footJoint = -1;
    int toeJoint = -1;
    // Onde a dobra fica (m a partir do calcanhar) e a largura da transicao.
    float toeBendAt = 0.2f;
    float toeBendWidth = 0.026f;
    // Sem perna para costurar na gola, a abertura fica declarada (borda
    // aberta de proposito).
    bool collarOpening = true;
    // O cadarco e os ilhoses como malha (as tiras em relevo e os ilhoses
    // desenhados no cabedal); desligado, ficam para a textura (a chuteira
    // leve: o cabedal, a sola, as travas e a lingua).
    bool laceGeometry = true;
};

// O que a chuteira deixa para a peca seguinte costurar.
struct BootSeams {
    bool ok = false;
    std::string error;
    // A borda de dentro da gola, onde a meia sai: anti-horaria vista de cima.
    std::vector<std::uint32_t> collar;
    // As partes: o cabedal (onde a meia costura) e a sola.
    std::string upperPart;
    std::string solePart;
};

// A chuteira inteira, uma superficie so: o cabedal (pontas arredondadas, a
// gola com a borda grossa, o canal do cadarco) costurado na sola pela base;
// as travas saindo de furos na sola; a lingua, o cruzado e as tiras do
// cadarco em relevo sobre o cabedal; os ilhoses desenhados nele. A gola fica
// aberta para a meia. Facetada como a referencia e com os pesos de skin.
[[nodiscard]] BootSeams buildBoot(Mesh& mesh, const BootRecipe& recipe, const BootPlacement& placement);

} // namespace MatterEngine::Modeling
