#pragma once

#include "Engine/Athlete/Legacy/LegacyAthleteState3D.hpp"

namespace MatterEngine {

// O estado de controle de um atleta: tudo o que o controle guarda entre os
// ticks. Um por atleta, sem nada global (bots, muitos jogadores, a sonda sem
// janela). Hoje so o caminho legado; as pecas novas do rework entram aqui ao
// lado dele.
struct AthleteState3D {
    LegacyAthleteState3D legacy;
};

} // namespace MatterEngine
