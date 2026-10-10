#pragma once

#include "Engine/Modeling/Mesh/Mesh.hpp"
#include "Engine/Modeling/Skeleton/Skeleton.hpp"

#include <filesystem>
#include <optional>
#include <span>
#include <string>

namespace MatterEngine::Modeling {

// As pecas procedurais do jogador, cada uma descrita num arquivo de receita
// (partes/<peca>.json, com "type"): "boot" (a chuteira) e "leg" (o meiao, o
// joelho e a coxa). A metade direita e montada de baixo para cima, cada peca
// costurada na borda da anterior (a perna na gola da chuteira), posicionada
// pelo esqueleto; a esquerda sai do espelho (com "mirror" da chuteira).
[[nodiscard]] std::optional<Mesh> buildProceduralParts(std::span<const std::filesystem::path> files,
    const Skeleton& skeleton, std::string* error = nullptr);

} // namespace MatterEngine::Modeling
