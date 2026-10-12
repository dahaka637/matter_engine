#pragma once

namespace MatterEngine {

// A simulacao de treino acelera processando mais passos por segundo de
// relogio; o passo fisico em si nunca muda.
inline constexpr float ReinforcementLearningFixedStepSeconds = 1.0f / 120.0f;

} // namespace MatterEngine
