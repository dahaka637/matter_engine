#pragma once

#include "Engine/Math/Vec2.hpp"

#include <cstdint>

namespace MatterEngine {

// Sequencia de Halton (radical inverse) na base informada - gera uma
// distribuicao de baixa discrepancia em [0,1): cobre o intervalo de forma
// mais uniforme ao longo de poucos quadros do que uma sequencia aleatoria
// pura, o que importa pro TAA porque cada amostra de jitter precisa
// preencher um pedaco diferente do pixel sem deixar buracos nem aglomerar
// amostras enquanto o historico acumula.
[[nodiscard]] float haltonSequence(std::uint32_t index, std::uint32_t base);

// Par (base 2, base 3) padrao pra jitter de TAA (Karis 2014, "High Quality
// Temporal Supersampling"). index comeca em 0, mas e deslocado internamente
// em +1 - Halton(0, qualquer base) sempre da 0, o que faria o primeiro
// quadro nao ter jitter nenhum.
[[nodiscard]] Vec2 haltonJitter(std::uint32_t index);

} // namespace MatterEngine
