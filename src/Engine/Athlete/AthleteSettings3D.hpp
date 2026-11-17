#pragma once

#include "Engine/Athlete/Synthesis/LegClearance3D.hpp"
#include "Engine/Athlete/Body/ActiveBodyController3D.hpp"
#include "Engine/Athlete/Body/ActiveStance3D.hpp"
#include "Engine/Athlete/Body/PhysicalFeetAssist3D.hpp"
#include "Engine/Athlete/Modes/ModeRules3D.hpp"
#include "Engine/Athlete/Physical/DynamicTurnMove3D.hpp"
#include "Engine/Athlete/Planning/FootstepPlanner3D.hpp"
#include "Engine/Athlete/Ball/BallOutcome3D.hpp"
#include "Engine/Gesture/GestureChoice3D.hpp"

namespace MatterEngine {

// Os ajustes do controle dos atletas (o que o laboratorio chamava de m_* no
// WorkbenchApp): um lugar so, sem variavel de ambiente. O laboratorio liga os
// sliders aqui; a sonda sem janela usa os padroes ou --ajuste. Os padroes sao
// os aprovados pelo usuario ate 10/10/2026.
struct AthleteRuntimeSettings3D {
    // Modo do corpo: o padrao (animado, pela capsula) e o fisico adaptativo.
    bool defaultLocomotion = true;
    bool adaptivePhysics = true;
    // Os bonecos soltos (nao controlados e sem comando) tambem no padrao, cada
    // um com a sua capsula virtual (de dados: sem colisao, como a do jogador no
    // padrao puro), com as mesmas trocas de modo do controlado - os contatos,
    // a ponte, o fisico, o levantar e a volta (28/10, pedido do usuario: "faca
    // os outros bonecos estarem na locomocao padrao, e sofrerem as mesmas
    // transformacoes que eu"). Desligado, eles ficam no fisico como antes (o
    // treino do RL e a sonda nao mudam).
    bool looseDefaultLocomotion = false;
    // Assistencias e musculos (o menu do personagem e o painel Ragdoll).
    bool ragdollForceControlOverride = false;
    float ragdollAuxiliaryPercent = 100.0f;
    float ragdollMusclePercent = 100.0f;
    float characterMusclePercent = 100.0f;
    // Locomocao fisica (E4/E5, v2).
    bool legsWalkingE5 = true;
    bool legsOnlyStanding = true;
    bool legsPostureStanding = true;
    bool physicalLocomotionV2 = true;
    float e5StandingAssistPercent = 100.0f;
    float e5WalkingAssistPercent = 100.0f;
    bool e5FootPlacement = true;
    bool staggerRecovery = true;
    // Experimento biomecanico (so o personagem controlado).
    bool biomechanicalExperiment = false;
    float biomechanicalBalanceAssistPercent = 100.0f;
    // Levantar.
    // (0-100, como as outras ajudas; 100% = a ajuda de levantar aprovada, que
    // era o 130% da regua antiga: GetUpAssistFull.)
    float getUpAssistPercent = 100.0f;
    // O par de postura (pelve x pes) tambem na regua: 0% e sem ajuda nenhuma
    // (o regime B, 17/10). false: o regime A (classico), o par sempre
    // inteiro. Vault 04_locomocao/ajuda_auxiliar.md.
    bool postureCoupleFollowsAssist = true;
    // A regua de ajuda: true = a historica (o "zero antigo", a lei de antes de
    // 5f2c4b6 no motor de hoje, o par sempre inteiro; padrao do jogo e do RL
    // desde 21/10, plano 04_locomocao/plano_rl_sobre_classico.md B); false = a
    // linear de 17/10 (0% = zero de verdade). O levantar e normalizado nas duas.
    bool historicalAssistRuler = true;
    // A virada brusca em movimento (experimento do laboratorio, 17/10; so no
    // Fisico total; vault 04_locomocao/virada_brusca.md). Desligada, o atleta
    // e identico bit a bit. A intensidade (0..1) e dos testes.
    bool dynamicTurnMove = false;
    // A meia-volta correndo pelo clipe (27/10; CharacterLocomotionInput3D::runningTurn).
    bool runningTurnClip = false;
    // A virada no tempo do corpo (27/10; CharacterLocomotionInput3D::movingHipTurn).
    bool movingHipTurn = false;
    TurnMoveSettings3D dynamicTurnMoveSettings;
    bool getUpHitWeakens = true;
    float getUpHitAssistPercent = 25.0f;
    float getUpHitRecoverSeconds = 1.5f;
    float getUpStiffnessPercent = 55.0f;
    float getUpStrengthPercent = 70.0f;
    // Carrinho e voadora.
    bool tackleEnabled = true;
    bool tackledAssistLow = true;
    float tackledRecoverSeconds = 2.5f;
    bool defaultAirTackle = true;
    float defaultAirTacklePitch = 30.0f;
    // O impulso extra do carrinho no padrao (%, da velocidade de entrada; 29/10).
    float defaultTackleBoostPercent = 30.0f;
    bool defaultVoadoraContactOnly = true;
    // O corpo fisico do padrao (28/10, item 10; ele escolhe na pratica): 0
    // desligado, 1 a camada de reacoes por molas, 2 a fisica de verdade sem
    // exagero (o corpo de cima com a guia mais solta).
    int physicalFeel = 0;
    // A fisica: a guia do corpo de cima (coluna, pescoco, cabeca e bracos; 1 o
    // exato, como as pernas) - abaixo de 1 o link carrega o proprio embalo e
    // atrasa (0,9 ~28 ms, 0,8 ~50 ms).
    float physicalUpperAuthority = 0.85f;
    // As molas: a intensidade das reacoes (%, sobre as do BodyReaction3D).
    float physicalReactionPercent = 100.0f;
    // O levantar pelo padrao (28/10, item 16): caido no fisico, com base
    // solida, o corpo guiado segue o clipe de levantar; contato volta ao
    // fisico. Desligado no runtime (o levantar e o treino dele); o
    // laboratorio liga.
    bool animatedGetUp = false;
    // A rede do levantar junto da rede da locomocao (01/11, ele: no "Fisico com RL" do laboratorio, "a troca harmonica
    // entre os modelos de RL de locomocao e levantar-se"): com as duas (AthleteRuntime3D::setGetUpPolicy e
    // locomotionPolicy), no fisico, na queda iminente - o criterio do fim do episodio do treino da locomocao e o tronco
    // tombando (GetUpPolicy3D) - ela dirige o corpo como no treino dela e, de pe e firme, devolve a rede da locomocao.
    bool getUpPolicy = false;
    // Com ela, a queda da locomocao fisica antes (02/11, ele: "quando ele caisse, ficasse naquele nosso euphoria style
    // ragdoll, e ai fosse para a RL de levantar so na hora de levantar"; "nao da para usar apenas a mesma que a locomocao
    // fisica ja usa?"): caindo, o classico cuida da queda inteira, do jeito dele - os reflexos no ar e o vivo no chao -, e
    // a rede assume so quando ele comeca a levantar. Desligada, a rede assume ja na queda iminente, como antes.
    // (Desligada desde 02/11 20h, ele: "vamos desativar por enquanto o ragdoll euphoria do RL... o RL passa a evitar
    // quedas, sendo a rede do RL final atuar no evitamento de quedas tambem".)
    bool protectiveFallBeforeGetUp = false;
    // O ritmo do clipe dele (28/10, pedido do usuario: "levantar-se rapido no
    // geral"; era 1).
    float animatedGetUpSpeed = 1.3f;
    // O levantar fisico torto zera e tenta de novo (28/10, item 16b; ver
    // CharacterLocomotionInput3D::getUpRetryCrooked). Desligado no runtime (o
    // levantar e o treino dele); o laboratorio liga.
    bool getUpRetryCrooked = false;
    // "Vida", pes e tornozelos.
    bool motionVariety = true;
    float motionVarietyPercent = 100.0f;
    bool footSpring = true;
    bool ankleBalance = true;
    bool toeRoll = true;
    // Modo padrao.
    float defaultGuidedExitSeconds = 0.12f;
    bool defaultSeedFromBody = true;
    bool defaultPhysicalJump = false;
    bool defaultGetUpHandoff = true;
    bool defaultAdaptiveTransitions = true;
    bool defaultTrunkMotion = true;
    float defaultTrunkMotionPercent = 100.0f;
    // O gesto da arrancada no padrao (o idle_to_sprint no tronco, nos bracos e na
    // cabeca ao sair do parado; 0 desliga, o padrao do motor).
    float defaultStartGesturePercent = 0.0f;
    bool defaultLandingAbsorb = true;
    float defaultCurveLeanDeadZone = 3.5f;
    float defaultTrotSpeed = 4.5f;
    float defaultSprintForwardSpeed = 7.8f;
    float defaultSprintSideSpeed = 5.2f;
    float defaultSprintBackwardSpeed = 5.2f;
    // O sprint maximo (Shift duplo; so a frente e as diagonais da frente).
    float defaultMaxSprintSpeed = 10.0f;
    // A regua "Resposta" da capsula no modo padrao (%; ver
    // athleteCapsuleResponse3D): 100 e o aprovado (16 / 22 / 30 m/s2, 10 rad/s).
    float capsuleResponsePercent = 100.0f;
    // A troca de direcao viva da capsula no padrao (0 a 100; 0: a de antes, o
    // padrao do motor): ao inverter o rumo ela freia e arranca ate 1,5 vez mais
    // forte (a 100%), ainda parando antes de sair para o outro lado.
    float capsuleReversalPercent = 0.0f;
    // Corpo Ativo (o estado intermediario, Etapa 2 do rework): ligado, as
    // pancadas leves, a parede e o membro no mundo levam ao Ativo (e nao ao
    // fisico pleno) e a volta do fisico passa por ele. Impulso (N.s) de algo
    // que se move a partir do qual o corpo vai ao Ativo e a partir do qual vai
    // direto ao fisico pleno; a batida da capsula (m/s) que vai ao fisico.
    // Desligado desde o teste do usuario (10/10, noite): andando ou correndo
    // depois do levantar e no encostao ele derrubava o jogador de novo (a
    // ponte fisico -> padrao do Plano_Claude_v2 o substitui).
    bool activeBody = false;
    // A reescrita (R3; vault 04_locomocao/reescrita_da_locomocao.md): as
    // trocas de modo pela tabela (ModeRules3D) e as classes de contato
    // (ContactClasses3D). A mesma regra de impacto no padrao, na volta e na
    // ponte; a perna no topo de algo conta; a ponte pode ser abortada (e a
    // guia dela cai com a ajuda do impacto); a ajuda cai pelos impactos de
    // verdade. Desligado: as regras de antes (a chave A/B).
    // A mesma intencao nos dois modos (a reescrita, R5): no fisico, a capsula
    // pede a velocidade da tabela do padrao (a aba "Velocidades": trote 4,5 em
    // todos os rumos, corrida 7,8 / 5,2 / 5,2) em vez da natural dos clipes da
    // locomocao fisica (trote ~3 m/s e bem menos de lado e de costas). Quem
    // chama (o laboratorio, a sonda) escolhe a tabela por isto.
    // Ligado desde 16/10 (medido: com 100% de ajuda o fisico anda 93-96% do
    // padrao nas direcoes, antes 64-90%; nenhuma queda nova na sonda; com 30%,
    // uma queda a mais no corte rapido).
    bool physicalUsesDefaultSpeeds = true;
    bool modeTable = true;
    // A reescrita (L0 do RL): no fisico, o padrao roda a sombra pelo comando
    // de agora (a capsula do jogador ou a virtual do comandado) e da a
    // referencia de estilo do contrato da rede (AthleteControlView3D). Nao
    // mexe no corpo; desligada, nada muda.
    bool physicalReference = false;
    // O relogio da sombra segue a fase do corpo (os pousos de verdade).
    bool physicalReferencePhaseLock = true;
    // A rede da locomocao (o RL, contrato v2): ligada, o controlado e os
    // comandados no fisico seguem a politica carregada no runtime
    // (AthleteRuntime3D::setLocomotionPolicy). Liga tambem a referencia a
    // sombra (a entrada da rede a le).
    bool locomotionPolicy = false;
    ContactClassParams3D contactClasses;
    ModeRuleParams3D modeRules;
    // A ajuda depois do impacto com a tabela: o choque comum ate 20 N.s tira
    // ate 20% (o encostao segue de pe); dali ate 40 N.s, ate 90% (o choque
    // forte derruba de verdade, em vez de so amolecer).
    float impactKneeImpulse = 20.0f;
    float impactKneeDrop = 0.2f;
    float impactFullImpulse = 40.0f;
    float impactMaximumDrop = 0.9f;
    // A volta da ajuda depois do choque no padrao (28/10, pedido do usuario:
    // "ir do fisico -> default mais rapidamente"): com o corpo ja assentado de
    // pe, ela volta este tanto de vezes mais rapido (a volta ao padrao espera
    // por ela). Caindo ou cambaleando, e no fisico total (e no treino do RL),
    // a de sempre (ImpactAssistSettings3D: 0,5 + 3 x a queda).
    float defaultCalmRecoverSpeedup = 2.5f;
    // A rapidez da volta ao padrao (28/10, pedido do usuario: "permitir eu
    // ajustar manualmente"): divide as esperas da volta - a da ajuda (ja
    // assentado), o minimo no fisico, a calma, a subida da ajuda e a ponte.
    // 1: a calibrada.
    float defaultReturnRate = 1.0f;
    // A janela curta dos impulsos (s): o choque espalhado por alguns ticks e
    // o empurrao que dura contam inteiros (na ajuda e na troca de modo).
    float impactWindowSeconds = 0.03f;
    // A ponte fisico -> padrao (plano v2, fase D): saindo do fisico (o fim
    // do levantar, o pouso de pe), o corpo segue fisico por bridgeSeconds
    // (bridgeMovingSeconds com as teclas) - os musculos do Ativo indo a pose
    // do padrao, que ja roda, a ajuda de passagem e o guiamento dos ossos
    // subindo de bridgeAuthorityStart a cheio - e entao o guiado. O contato
    // no padrao entra nela pela ponte de contato (abaixo).
    bool physicalBridge = true;
    // (16/11, ele: "o momento de transicao da locomocao fisica para a default... os movimentos de transicao sao muito bruscos, do corpo sendo
    // colocado no lugar a forca") A volta quando quem devolve e o cerebro unico do RL (LegacyAthleteState3D::rlReturn): o corpo ja chega
    // equilibrado, entao sem a ponte nem o Corpo Ativo (que o arrastavam: a pelve a 0,4-0,9 m/s parado, as maos a 57 m/s2) - direto ao
    // guiado, partindo do estado exato do corpo (juntas, velocidades, pelve) e chegando a pose do padrao pela mola da inercia, mais
    // lenta que a da volta de sempre: meia-vida (s) e duracao (s).
    float rlReturnHalfLife = 0.30f;
    float rlReturnSeconds = 1.20f;
    // (E a velocidade: a capsula - e a pose do padrao presa nela - sai da do corpo e vai a pedida nesta aceleracao (m/s2), tambem sem
    // tecla - parada, a capsula freava 0,8 m/s em 50 ms e levava o corpo junto -, por ate rlReturnRampSeconds.)
    float rlReturnAcceleration = 4.0f;
    float rlReturnRampSeconds = 1.5f;
    // (16/11) Com tecla, a da volta pelo cerebro: a da capsula do padrao (capsuleResponsePercent - no laboratorio ~24 m/s2), comecando
    // suave em rlReturnMovingOnsetSeconds (sem degrau). (Ele, 16h50: "fica lento quando levanta ou volta ao modo default por um
    // pouquinho": a 5 m/s2 - e a 9, a de antes - o padrao levava ~1 s para chegar a velocidade das teclas.) > 0: este valor fixo (m/s2).
    float rlReturnMovingAcceleration = 0.0f;
    float rlReturnMovingOnsetSeconds = 0.15f;
    // (0,45 ate 28/10: a volta ao padrao mais rapida, pedido do usuario.)
    float bridgeSeconds = 0.30f;
    float bridgeMovingSeconds = 0.30f;
    float bridgeAuthorityStart = 0.15f;
    // Os contatos inteligentes (28/10, pedido do usuario: "nao perca
    // velocidade... um modo fisico parcial que pode evoluir para fisico se
    // houver perda de base"): o impacto de algo que se move (outro boneco, um
    // objeto, um empurrao), no chao e fora do carrinho, leva o padrao a ponte
    // de contato - o corpo fisico seguindo o padrao, que continua rodando sem
    // recomecar a passada - e nao ao fisico pleno; na ponte, outro contato
    // assim recomeca a guia. O fisico pleno so na perda de base (as regras da
    // ponte), na pancada forte (a janela de impulsos acima de
    // contactBridgeHardDeltaV x a massa), no agarrao, no carrinho, no ar e com
    // o mundo parado. A guia sobe em contactBridgeBaseSeconds +
    // contactBridgeSecondsPerDeltaV x Dv (m/s), ate contactBridgeMaxSeconds.
    bool contactBridge = true;
    float contactBridgeHardDeltaV = 1.2f;
    // (16/11, ele: "puxando o boneco... arrastando") Os bonecos soltos no padrao, com o fisico pelo cerebro do RL: o encostao e o
    // empurrao de quem se move vao ao fisico - o cerebro segura com passos de verdade e devolve pela volta harmonica -, nao a ponte de
    // contato nem ao empurrao da capsula virtual (os dois deslizavam os pes: 1,3 m a 12 m/s parado e 2,2 m na ponte, num empurrao de 1 s).
    // Desligado (o padrao): como antes - o jogador fica sempre na ponte.
    bool looseContactsToPhysical = false;
    float contactBridgeBaseSeconds = 0.25f;
    float contactBridgeSecondsPerDeltaV = 0.25f;
    float contactBridgeMaxSeconds = 0.8f;
    // (A navegacao leva o golpe: o impulso do contato com o que se move sobre a
    // massa soma no golpe pendente (m/s), que empurra a capsula junto com o
    // corpo e se gasta em contactBridgeKickSeconds - o atleta absorve o
    // empurrao em uns passos, a locomocao segue na velocidade pedida -; ele
    // tambem e a regua da pancada forte, com a janela. A capsula segue o corpo
    // com a folga contactBridgeFollowSlack (m; a do Ativo e activeFollowSlack).)
    float contactBridgeKickSeconds = 0.3f;
    float contactBridgeFollowSlack = 0.03f;
    // (A decisao (s): a ponte de contato entra solta - o choque bate num corpo
    // fisico - e, passado este tempo com o golpe medido, vai ao fisico na
    // pancada forte ou sobe a guia na proporcao dele. No primeiro tick o golpe
    // ainda nao se formou: 0,2 m/s da caixa que derruba.)
    float contactDecisionSeconds = 0.025f;
    // A volta suave (plano v4, etapa C; desligada, a de antes): a capsula
    // embaixo do corpo e com a velocidade dele, a pedida subindo aos poucos,
    // a pose saindo do corpo mais devagar.
    bool returnSmoothing = true;
    // Na volta, a velocidade pedida sai da do corpo e vai a das teclas nesta
    // aceleracao (m/s2), por ate returnRampSeconds (s) (plano v4, etapa C: a
    // capsula ia a pedida a 14 m/s2 e o corpo, mais devagar no fisico,
    // disparava atras dela).
    // (02/11, ele: a troca para o padrao comecava "amarrada, lenta, pesada"; a
    // 2,5 m/s2 ir de 2 a 8 m/s levava >2 s. O padrao acelera a ~14.)
    float returnAcceleration = 9.0f;
    float returnRampSeconds = 0.5f;
    ActiveBodySettings3D activeBodySettings;
    ActiveStanceSettings3D activeStanceSettings;
    float activeEnterImpulse = 1.0f;
    float fullPhysicalImpulse = 25.0f;
    float fullPhysicalWallImpact = 5.0f;
    // No Ativo, o ponto de captura mais que isto fora da base (m) leva ao
    // fisico pleno: sem os passos de equilibrio (ActiveStanceSettings3D), a
    // pancada que pede passo; com eles, a que um passo nao segura.
    float activeHandoverCapture = 0.06f;
    float activeHandoverCaptureStepping = 0.20f;
    // No Ativo andando, a capsula vai atras do corpo com esta folga (m);
    // parado, ela vai para onde a base real quer a raiz (ActiveStance3D). E a
    // volta ao padrao com os pes ainda fora do lugar do clipe acontece depois
    // deste tempo calmo (s): a inercia da volta os leva.
    float activeFollowSlack = 0.10f;
    float activeReturnTimeout = 1.2f;
    // A ajuda nos pes do corpo fisico (%; pedida em 10/10 noite, aprovada
    // pelo usuario em 11/10): a forca que leva o corpo a referencia passa
    // pelos pes apoiados, no que a ajuda da pelve nao deu
    // (PhysicalFeetAssist3D). Cai com o impacto e volta como a ajuda andando e
    // parado. 0 desliga.
    float physicalFeetAssistPercent = 100.0f;
    PhysicalFeetAssistSettings3D physicalFeetAssist;
    // A guiagem de ossos do Fisico total (%; experimento do usuario, 11/10): a
    // pose do modo padrao roda junto e os ossos vao ate ela nesta fracao - 0 e
    // a locomocao fisica de hoje, 100 e como o padrao. Caido, levantando e no
    // carrinho, nada.
    float physicalBoneGuidePercent = 0.0f;
    // Locomocao por pegadas (Etapa 3; refeita no plano v3): no modo padrao o
    // clipe vira estilo e os pes apoiados ficam onde pousaram
    // (FootstepPlanner3D). Fora do jogo desde 11/10 (o usuario: "causa
    // movimentos roboticos e ruins") - so a sonda ainda a liga.
    bool footstepLocomotion = false;
    FootstepSettings3D footsteps;
    // As pernas nunca se atravessam na pose do padrao (plano v4).
    LegClearanceSettings3D legClearance;
    // A bola (24/10, a reescrita dos gestos): a forca (a aba "Forca na
    // bola"), o alcance e os baldes da escolha do gesto e o tempo de cada
    // gesto (s; o laboratorio tem os dois).
    BallOutcomeSettings3D ballForce = defaultBallOutcomeSettings3D();
    GestureReach3D gestureReach;
    // (24/10: os tempos que o usuario calibrou no laboratorio.)
    float touchGestureSeconds = 0.45f;
    float shotGestureSeconds = 0.58f;
    // (17/11, pedido dele; as duas ligadas por padrao, com checkbox no
    // laboratorio) O toque realista: o gesto vai ate a bola de verdade e a
    // forca do pe so sai quando a chuteira encosta nela. A conducao
    // automatica: o drible com a barra ate autoConductionBar liga os toques
    // sozinhos, de verdade e o minimo deles, mantendo a bola logo a frente dos
    // pes.
    bool realisticTouch = true;
    // (17/11 14h20, ele: "por hora o manejo automatico fica de fora" - desligada
    // por padrao; a checkbox liga para testar.)
    bool autoConduction = false;
    float autoConductionBar = 0.35f;
};

} // namespace MatterEngine
