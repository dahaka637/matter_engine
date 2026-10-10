#pragma once

#include "Engine/Modeling/Mesh/MeshBuilder.hpp"
#include "Engine/Modeling/Parts/BootRecipe.hpp"

#include <span>
#include <string>

namespace MatterEngine::Modeling {

// A sola costurada no cabedal: a borda de cima dela sao os vertices da base
// do cabedal (medialBase e lateralBase: um por estacao, do calcanhar ao
// bico). Dali ela sai na borda que aparece, desce pela lateral e fecha por
// baixo com as nervuras; as pontas fecham em leque ate a base do cabedal.
// Vai para a parte e o material correntes, no referencial da chuteira.
void buildBootSole(MeshBuilder& builder, const Transform& frame, const BootRecipe& recipe,
    std::span<const std::uint32_t> medialBase, std::span<const std::uint32_t> lateralBase);

// As travas saindo da sola: a pegada de cada uma recortada na face de baixo
// (os triangulos soleTriangles) e a trava continuando do furo ate a ponta
// chanfrada no chao. Parte e material correntes.
bool buildBootStuds(MeshBuilder& builder, const Transform& frame, const BootRecipe& recipe,
    std::span<const std::uint32_t> soleTriangles, std::string* error = nullptr);

} // namespace MatterEngine::Modeling
