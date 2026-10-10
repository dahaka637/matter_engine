#pragma once

#include "Engine/Modeling/Mesh/MeshBuilder.hpp"
#include "Engine/Modeling/Parts/HeadRecipe.hpp"

#include <array>
#include <cstdint>

// O olho na abertura amendoada do rosto: a borda da palpebra entra, a linha
// dos cilios, o globo (o branco) e a iris escura no meio. Uma superficie so
// com o rosto: o olho continua da volta da abertura, vertice com vertice.
namespace MatterEngine::Modeling {

struct EyePlacement {
    // A volta da abertura (8 pontos, anti-horaria vista de fora).
    std::array<std::uint32_t, 8> opening {};
    // O centro da abertura, o olhar (para fora) e o rumo do canto de dentro
    // para o de fora.
    Vec3 center;
    Vec3 forward;
    Vec3 across;
    int headJoint = -1;
};

// Ergue o olho na parte e no material correntes do builder (os materiais
// do olho sao criados aqui). Pesos: a cabeca.
void buildEye(MeshBuilder& builder, const HeadRecipe& recipe, const EyePlacement& eye);

} // namespace MatterEngine::Modeling
