#pragma once

#include "Engine/Athlete/Actions/BallHandling3D.hpp"
#include "Engine/Athlete/Actions/BallPlay3D.hpp"
#include "Engine/Athlete/Body/ActiveBodyController3D.hpp"
#include "Engine/Athlete/Body/ActiveStance3D.hpp"
#include "Engine/Athlete/Body/PhysicalFeetAssist3D.hpp"
#include "Engine/Athlete/Legacy/LegacyAthleteState3D.hpp"
#include "Engine/Athlete/Planning/FootstepPlanner3D.hpp"
#include "Engine/Athlete/Sensing/BodyObservation3D.hpp"
#include "Engine/Athlete/Synthesis/LegSynthesis3D.hpp"

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
    // A ajuda nos pes do Fisico total: a ultima saida (telemetria).
    PhysicalFeetAssistOutput3D feetAssistOutput;
    // Locomocao por pegadas (Etapa 3): o dono dos pes, os alvos deste tick e
    // o que a perna alcancou.
    FootstepPlanner3D footsteps;
    std::array<FootTarget3D, 2> footTargets {};
    LegSynthesisResult3D legSynthesis;
    // Manejo novo: o v2 (BallPlay3D, plano v2) e a ultima saida dele (o pe
    // que joga, telemetria). (O da Etapa 5 fica ate a limpeza.)
    BallHandling3D ballHandling;
    BallPlay3D ballPlay;
    BallHandlingOutput3D ballOutput;
};

} // namespace MatterEngine
