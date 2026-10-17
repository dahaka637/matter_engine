#pragma once

#include "Engine/RL/Commands/CommandPilot3D.hpp"

#include <nlohmann/json_fwd.hpp>

#include <string>

namespace MatterEngine {

// O roteiro do piloto lido do JSON de uma tarefa (o objeto "pilot" de um
// exercicio): o mesmo formato no lote do Corpo v2 e no da locomocao. Uma
// faixa e [minimo, maximo]; os angulos da camera em graus. Erro de formato:
// std::runtime_error com o exercicio.
[[nodiscard]] PilotScript3D pilotScriptFromJson3D(const nlohmann::json& pilot, const std::string& drill);

} // namespace MatterEngine
