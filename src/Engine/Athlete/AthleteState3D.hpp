#pragma once

#include "Engine/Athlete/Actions/BallHandling3D.hpp"
#include "Engine/Athlete/Actions/BallPlay3D.hpp"
#include "Engine/Athlete/Dribble/Dribble3D.hpp"
#include "Engine/Athlete/Body/ActiveBodyController3D.hpp"
#include "Engine/Athlete/Body/ActiveStance3D.hpp"
#include "Engine/Athlete/Body/PhysicalFeetAssist3D.hpp"
#include "Engine/Athlete/Legacy/LegacyAthleteState3D.hpp"
#include "Engine/Athlete/Modes/ModeRules3D.hpp"
#include "Engine/Athlete/Physical/PhysicalCorrections3D.hpp"
#include "Engine/Athlete/Planning/FootstepPlanner3D.hpp"
#include "Engine/Athlete/Sensing/BodyObservation3D.hpp"
#include "Engine/Athlete/Synthesis/LegClearance3D.hpp"
#include "Engine/Athlete/Synthesis/LegSynthesis3D.hpp"

namespace MatterEngine {

// A ajuda auxiliar deste tick: so as forcas de FORA aplicadas pelo controle
// (sem reacao dentro do corpo), por canal (vault 04_locomocao/ajuda_auxiliar.md).
// Os motores das juntas, a compensacao de gravidade e os pares internos (a
// postura pelve x pes) sao o corpo e nao entram; o torque liquido dos pares
// fica em netTorque (zero quando so ha pares internos). A 0% da regua, tudo o
// que e dela e zero.
struct AthleteExternalAssist3D {
    float pelvisNewtons = 0.0f;   // a pelve: sustentacao e empurrao (parado e andando)
    float feetNewtons = 0.0f;     // os pes: a ajuda nos pes e a correcao pelos pes
    float getUpNewtons = 0.0f;    // o levantar: a pelve e o peito
    float netTorque = 0.0f;       // o torque liquido (N.m) dos canais da regua
    // Fora da regua (acoes e o modo padrao): o carrinho e o Corpo Ativo.
    float tackleNewtons = 0.0f;
    float tackleTorque = 0.0f;
    float activeNewtons = 0.0f;
    [[nodiscard]] float sliderNewtons() const { return pelvisNewtons + feetNewtons + getUpNewtons; }
};

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
    // A ajuda auxiliar aplicada neste tick (medida, por canal).
    AthleteExternalAssist3D externalAssist;
    // Locomocao por pegadas (Etapa 3): o dono dos pes, os alvos deste tick e
    // o que a perna alcancou.
    FootstepPlanner3D footsteps;
    std::array<FootTarget3D, 2> footTargets {};
    LegSynthesisResult3D legSynthesis;
    // As pernas nunca se atravessam na pose do padrao (plano v4).
    LegClearance3D legClearance;
    // Manejo novo: o v2 (BallPlay3D, plano v2) e a ultima saida dele (o pe
    // que joga, telemetria). (O da Etapa 5 fica ate a limpeza.)
    BallHandling3D ballHandling;
    BallPlay3D ballPlay;
    // Plano v5: o manejo refeito e a ultima saida dele.
    Dribble3D dribble;
    DribbleOutput3D dribbleOutput;
    BallHandlingOutput3D ballOutput;
    // A reescrita (R3): o corpo da classificacao de contatos, as regras de
    // troca de modo e o ultimo tick delas (a telemetria neutra: o que bateu,
    // a ultima troca e o motivo).
    ContactBody3D contactBody;
    ModeRules3D modeRules;
    // As janelas curtas dos impulsos: o empurrao de fora (a troca de modo) e
    // o choque inteiro (a queda da ajuda).
    ImpulseWindow3D pushWindow;
    ImpulseWindow3D impactWindow;
    // As correcoes do RL nas costuras do controle fisico deste atleta (sem
    // autoridade nenhuma, nem passam: o classico exato).
    PhysicalCorrections3D corrections;
    // A capsula virtual de um atleta comandado sem a capsula do jogador: a
    // velocidade do plano dela (sem o "ir atras do corpo").
    Vec3 commandPlanVelocity;
    // O giro do rumo pedido (rad/s, filtrado em 0,25 s, como no laboratorio):
    // virando forte, a velocidade cai como a de quem corre cortando.
    Vec3 commandDirection;
    bool commandDirectionValid = false;
    float commandTurnRate = 0.0f;
    // O chao embaixo da pelve no ultimo passo (z).
    float groundHeight = 0.0f;
    // A politica da locomocao (o contrato do RL, Athlete/Contract): as acoes
    // das ultimas decisoes (a mais nova primeiro, LocomotionContract3D::
    // HistoryDecisions x o tamanho da acao) e a rampa entre decisoes (a acao
    // de antes, a nova e os ticks desde a decisao). Mora no atleta: o
    // instante guardado leva junto, e o jogo usa a mesma.
    std::vector<float> policyHistory;
    std::vector<float> policyFrom;
    std::vector<float> policyTo;
    int policyTicks = 0;
    // O valor fisico aplicado de cada canal da saida (depois do teto, da
    // autoridade e do limite de taxa): o estado do filtro.
    std::vector<float> policyApplied;
    // A politica e a da rede do runtime (AthleteRuntime3D::setLocomotionPolicy):
    // desligada, so ela e esquecida (as correcoes postas por fora ficam).
    bool policyFromRuntime = false;
    // O custo do ultimo passo (s; o orcamento por modulo): o controle fisico
    // (por bloco em legacy.locomotion.sectionSeconds()) e a ajuda.
    float locomotionSeconds = 0.0f;
    float assistSeconds = 0.0f;
    float referenceSeconds = 0.0f;
    ContactAssessment3D lastContacts;
    ModeDecision3D lastDecision;
    BodyModeKind3D lastDecisionFrom = BodyModeKind3D::Animated;
};

} // namespace MatterEngine
