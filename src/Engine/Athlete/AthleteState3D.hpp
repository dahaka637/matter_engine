#pragma once

#include "Engine/Athlete/Ball/BallOutcome3D.hpp"
#include "Engine/Athlete/Ball/ShotConduction3D.hpp"
#include "Engine/Athlete/Body/AnimatedGetUp3D.hpp"
#include "Engine/Athlete/Body/ActiveBodyController3D.hpp"
#include "Engine/Gesture/GestureTypes3D.hpp"
#include "Engine/Athlete/Body/ActiveStance3D.hpp"
#include "Engine/Athlete/Body/PhysicalFeetAssist3D.hpp"
#include "Engine/Athlete/Legacy/LegacyAthleteState3D.hpp"
#include "Engine/Athlete/Modes/ModeRules3D.hpp"
#include "Engine/Athlete/Physical/PhysicalCorrections3D.hpp"
#include "Engine/Athlete/Planning/FootstepPlanner3D.hpp"
#include "Engine/Athlete/Sensing/BodyObservation3D.hpp"
#include "Engine/Athlete/Synthesis/LegClearance3D.hpp"
#include "Engine/Athlete/Synthesis/LegSynthesis3D.hpp"
#include "Engine/Physics/InflatedBall3D.hpp"

#include <cstdint>

namespace MatterEngine {

// A ajuda auxiliar deste tick: so as forcas de FORA aplicadas pelo controle
// (sem reacao dentro do corpo), por canal (vault 04_locomocao/ajuda_auxiliar.md).
// Os motores das juntas, a compensacao de gravidade e os pares internos (a
// postura pelve x pes) sao o corpo e nao entram; o torque liquido dos pares
// fica em netTorque (zero quando so ha pares internos). A 0% da regua, tudo o
// que e dela e zero.
struct AthleteExternalAssist3D {
    float pelvisNewtons = 0.0f;   // a pelve: sustentacao e empurrao (parado e andando)
    float pelvisVerticalNewtons = 0.0f;  // ... a parte vertical (a sustentacao)
    float pelvisPlanarNewtons = 0.0f;    // ... a parte no plano (o empurrao e a correcao)
    float feetNewtons = 0.0f;     // os pes: a ajuda nos pes e a correcao pelos pes
    float getUpNewtons = 0.0f;    // o levantar: a pelve e o peito
    // O torque de fora: a soma VETORIAL de todos os torques dos canais da regua
    // (pares internos se cancelam nela), e a magnitude no fim.
    Vec3 torqueWorld;
    float netTorque = 0.0f;
    float correctionTorque = 0.0f;  // a alavanca da correcao de desvio (de fora)
    // O par interno de postura (pelve x pes/maos, o "musculo virtual" do
    // controle classico): medido a parte; cancela na soma, nao e ajuda.
    float postureCoupleTorque = 0.0f;
    // A volta ao padrao (returnBoost > 0): fora da regua, so no modo Padrao.
    bool handover = false;
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
    // Quantos gestos de bola ele ja fez (a semente do jeito de cada um), a
    // perna do ultimo e o tempo desde ele (s; conduzindo, as pernas se
    // alternam).
    std::uint32_t gestureCount = 0;
    int lastGestureSide = -1;
    float sinceGesture = 1e9f;
    // Conduzindo, o toque e do passo seguinte da perna dele: com ela
    // plantada, a animacao espera ela sair do chao (a forca ja saiu no
    // comando) - o gesto guardado e o tempo esperado (s).
    struct PendingGesture {
        bool active = false;
        GestureSpec3D spec;
        float waited = 0.0f;
        // (17/11, o planejador de contato) Comeca depois deste tempo (s; < 0: quando
        // a perna sair do chao), e o contato marcado conta a partir do pedido.
        float startIn = -1.0f;
    } pendingGesture;
    // A recepcao armada (24/10, pedido do usuario): o toque ou o chute fraco
    // apertado com a bola vindo e ainda fora do alcance espera ate o tempo
    // dela (left, s) com o que foi pedido; a forca sai no tick em que a bola
    // entra no alcance. Desde 28/10 (pedido do usuario) tambem o chute na bola
    // vindo (receive falso): sai como chute, o de primeira.
    struct ArmedReception {
        bool active = false;
        bool shot = false;
        float power = 0.0f;
        float lift = 0.0f;
        float curve = 0.0f;
        float aimYaw = 0.0f;
        float left = 0.0f;
        bool receive = true;
        // O gesto ja comecou, antes da bola chegar (28/10, a previsao).
        bool started = false;
        // O levantar guardado (28/10: o de cabeca na bola que cai).
        bool raise = false;
        // (17/11, o toque assistido) O clique guardado sem encontro possivel
        // ainda (ate 0,5 s): sai quando o pe puder chegar na bola.
        bool buffered = false;
    } armedReception;
    // A conducao do chute (28/10, pedido do usuario; Ball/ShotConduction3D):
    // a bola chutada, enquanto ela esta no alcance.
    struct Conduction {
        bool active = false;
        InflatedBallHandle3D ball;
        ShotConduction3D shot;
    } conduction;
    // (17/11, o toque realista - pedido dele) A forca do pe so sai quando a
    // chuteira encosta na bola: o toque pedido fica guardado (a bola, o pedido
    // do resultado, a perna do gesto e quanto ainda espera, s) e sai no tick
    // do encosto; sem encosto, nada (o pe nao chegou).
    struct PendingTouch {
        bool active = false;
        InflatedBallHandle3D ball;
        BallOutcomeRequest3D request;
        Vec3 ahead;
        int side = 0;
        float left = 0.0f;
        bool shotConduction = false;
        float shotPower = 0.0f;
        float shotYaw = 0.0f;
        // O mais perto que a chuteira chegou da superficie dela (m; a sonda).
        float closest = 1.0e9f;
        // O toque da conducao automatica: replanejado no encosto, com qualquer
        // pe; o rumo pedido quando ele foi planejado (uma virada o cancela).
        bool automatic = false;
        Vec3 plannedWanted;
        // (17/11, o toque assistido) O contato planejado (s desde o pedido; < 0
        // sem plano), o tempo desde o pedido e a folga do pe ao tick anterior: perto
        // do instante planejado, o ponto mais proximo da passagem do pe (ate 30 cm)
        // tambem e o toque - o pe que chegou perto nao erra.
        float plannedContact = -1.0f;
        float elapsed = 0.0f;
        float previousGap = 1.0e9f;
        // (O contato por plano) Onde a bola estaria no contato (mundo): no instante
        // planejado, com ela ali (ate 35 cm), o toque sai - o pe esta chegando nela.
        Vec3 plannedBall;
        // O encontro so com o corpo indo ate a bola (a "puxada" suave, pedido dele).
        bool approach = false;
        // O dominio (o drible sem tecla): no encosto, a bola rola e para logo a
        // frente dos pes de onde ele vai parar.
        bool trap = false;
        // (17/11, ele: "ir ajustando em tempo real conforme o movimento... para
        // garantir o contato") O gesto do toque, o fim da armada (s desde o pedido)
        // e ha quanto foi o ultimo ajuste: a cada tick o encontro e replanejado
        // e, mudando, o gesto e corrigido no meio.
        GestureSpec3D gesture;
        float windupEnd = 0.0f;
        float sinceRetarget = 0.0f;
        // (17/11 ~15h) A velocidade de onde sai o ajuste curto do encontro: no
        // dominio, a dele no aperto (ele segue um instante ate a bola).
        Vec3 approachBase;
    } pendingTouch;
    // (17/11, a conducao automatica - pedido dele) O drible fraco liga os
    // toques sozinhos: a bola, ha quanto ela esta longe e desde o ultimo toque
    // (s), e a desaceleracao dela rolando (m/s2, medida enquanto rola solta).
    struct AutoConduction {
        bool active = false;
        InflatedBallHandle3D ball;
        float away = 0.0f;
        float sinceTouch = 1e9f;
        float deceleration = 0.5f;
        float lastSpeed = -1.0f;
        // O rumo pedido e quanto ele gira (rad/s, media curta): a virada em
        // andamento - o toque manda a bola para onde ele vai estar.
        float wantedYaw = 0.0f;
        float wantedYawRate = 0.0f;
        bool wantedYawValid = false;
        // O rumo pedido no ultimo toque planejado (uma virada desde ele cancela o
        // toque e o gesto ainda guardados).
        Vec3 plannedWanted;
        // Desde a ultima mudanca brusca do comando (s): o rumo saltando mais de
        // 25 graus num tick ou a rapidez mais de 1,5 m/s - o hitbox maior.
        float sinceSharp = 1e9f;
        Vec3 lastWanted;
    } autoConduction;
    // O corpo fisico do padrao (28/10, item 10): a guia de cada link (1 nas
    // pernas e na pelve, a do corpo de cima nos outros).
    std::vector<float> guideAuthority;
    // O levantar pelo padrao (28/10, item 16): o clipe guiado e quanto tempo
    // ele esta deitado e assentado (s; < 0 espera depois de uma base ruim).
    AnimatedGetUp3D animatedGetUp;
    float animatedGetUpCalm = 0.0f;
    // A entrega da rede do levantar ao classico (01/11; Athlete/Body/GetUpPolicy3D): quanto falta da passagem dos alvos
    // dela aos do classico (s) e os ultimos dela.
    struct GetUpHandoff {
        float blendLeft = 0.0f;
        std::vector<RagdollDriveTarget3D> drives;
    } getUpHandoff;
    // A rede do levantar dirigindo (01/11, AthleteRuntimeSettings3D::getUpPolicy): se esta, os passos desde que assumiu,
    // os alvos de agora, ha quanto tempo ele esta de pe e firme (s) e a inclinacao do tronco na decisao anterior (a queda
    // iminente pelo tronco, medida a cada GetUpPolicy3D::TicksPerDecision passos).
    struct GetUpRecovery {
        bool active = false;
        std::uint32_t ticks = 0;
        std::vector<RagdollDriveTarget3D> drives;
        float firmSeconds = 0.0f;
        float seconds = 0.0f;
        float trunkTilt = 0.0f;
        bool trunkTiltValid = false;
        std::uint32_t tiltTicks = 0;
        // (01/11, ele: "o boneco fica congelado em uma pose se ao tentar levantar ha contato": ha quanto tempo ela esta
        // parada sem subir (s) e, depois de soltar por isso ou pelo tempo, a espera de ele ficar de pe pelo classico
        // antes de assumir de novo - no chao, a queda iminente disparava de novo no passo seguinte, sem fim.)
        float stuckSeconds = 0.0f;
        bool waitStand = false;
        // Ja no chao desde que assumiu (a troca andando, GetUpPolicy3D::MovingHandoffSeconds, so depois do chao).
        bool down = false;
    } getUpRecovery;
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
