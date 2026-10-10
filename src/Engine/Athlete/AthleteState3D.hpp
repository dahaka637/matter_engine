#pragma once

#include "Engine/Athlete/Body/ActiveBodyController3D.hpp"
#include "Engine/Athlete/Body/ActiveStance3D.hpp"
#include "Engine/Athlete/Legacy/LegacyAthleteState3D.hpp"
#include "Engine/Athlete/Sensing/BodyObservation3D.hpp"

namespace MatterEngine {

// O estado de controle de um atleta: tudo o que o controle guarda entre os
// ticks. Um por atleta, sem nada global (bots, muitos jogadores, a sonda sem
// janela). Hoje so o caminho legado; as pecas novas do rework entram aqui ao
// lado dele.
struct AthleteState3D {
    LegacyAthleteState3D legacy;
    // Corpo Ativo (o estado intermediario, Etapa 2): a leitura do corpo, o
    // controlador e a ultima saida dele (telemetria e desenho).
    BodyObserver3D observer;
    ActiveBodyController3D active;
    ActiveStance3D stance;
    ActiveBodyOutput3D activeOutput;
};

} // namespace MatterEngine
