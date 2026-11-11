#pragma once

#include "Engine/Athlete/Body/LocomotionPartner3D.hpp"
#include "Engine/Physics/PhysicsScene3D.hpp"
#include "Engine/Physics/RagdollProfile3D.hpp"
#include "Engine/Tracking/TrackingGetUp3D.hpp"

#include <cstdint>
#include <array>
#include <memory>
#include <span>
#include <string>
#include <utility>
#include <vector>

namespace MatterEngine {

struct TrackingGetUpAmpSettings3D {
    std::string assetsDirectory;
    std::uint32_t environments = 512;
    std::uint32_t threads = 0;
    std::uint64_t seed = 1;
    // A avaliacao: so as quedas (sem os comecos nos clipes).
    bool evaluation = false;
    // Duas cabecas numa politica (a recuperacao e a locomocao): a ultima entrada diz qual age (1: a locomocao, de pe
    // com pedido de deslocamento).
    bool separateSkills = false;
    // O rastreio ruim sustentado encerra (o early termination por desvio): a media movel da nota (~0,5 s) abaixo disto,
    // depois de 1 s. 0: so a queda.
    float trackingTermination = 0.0f;
    // A forca auxiliar da locomocao (04/11 00h, a ordem dele: "forcar a forca auxiliar em 100% para os exercicios de
    // locomocao, assim ele sera obrigado a aprender"): com a cabeca da locomocao no rastreio de um clipe, a mao de fora
    // na pelve puxa a altura e a inclinacao/rumo da pelve do clipe e a velocidade pedida (PD; 1 = a mao inteira, 0 =
    // nenhuma). O treinador a afrouxa so com o dominio medido.
    float locomotionAssist = 0.0f;
    // (04/11 04h35) A mao linear de verdade: a 100% no maximo o peso do corpo (sustentar), metade dele (empurrar) e o
    // torque de tombar (peso x 1 m); a x%, x% disso; 0% nada. false: a mao de antes (cortada em 2000 N / 1500 N.m).
    bool linearHand = true;
    // (04/11 05h45) A mao tambem no tronco de cima: o peito alto, a cabeca e os bracos para a orientacao do clipe.
    bool upperHand = false;
    // (04/11 06h00) A mao em todos os ossos: a coluna, o pescoco, a cabeca, os bracos, as maos, as pernas e os pes.
    bool bodyHand = false;
    // O pe de equilibrio (04/11, ele: "usar o footplanner que ja temos na locomocao fisica"): com o clipe nos motores
    // (locomotionResidual), o pe do clipe no ar pousa deslocado pelo ponto de captura do corpo real contra o pedido -
    // o balanceFootPlacement do modo fisico - e o IK das pernas (synthesizeAthleteLegs3D) leva a perna ate la.
    bool balanceFeet = false;
    // A nota do rastreio e so a v19 desde 10/11; o campo ficou como o portao da inclinacao da referencia (leanSide, runPitch:
    // so com 10 ou mais). shortcut: o lote nasce com 1 e o treinador poe 19 no configurar - os comecos sorteados no construtor
    // saem sem a inclinacao, e o ouro da prova depende disso; tirar num passo proprio com ouro novo.
    int trackingReward = 1;
    // O parado em alerta (04/11 04h10): o peso da imitacao do clipe IDLE ALERTA na regua estrita (DeepMimic, suavidade 1),
    // no rumo do proprio corpo; 0 = como antes (o imitationWeight, se houver).
    float idleImitationWeight = 0.0f;
    // (04/11 04h50) O idle depois de ficar de pe (o IDLE ALERTA vira a referencia ao ficar de pe sem pedido) e o parado na
    // cabeca da locomocao (o clipe do idle nos motores e a correcao da rede; caido, a recuperacao).
    bool idleAfterStand = false;
    bool idleOnLocomotion = false;
    // O alcance da correcao da rede no idle da cabeca da locomocao (a fracao do de sempre).
    float idleResidualScale = 1.0f;
    // (04/11 09h00) A entrega ao idle depois de levantar: na hora, com os alvos dos motores e a mao indo ao idle neste tempo
    // (s; 0 = a de antes, com o corpo quieto IdleHandoffSeconds), e o alcance da correcao nesse tempo (a fracao do de sempre;
    // < 0: o do idle), que volta ao do idle junto.
    float idleBlendSeconds = 0.0f;
    // (05/11 02h15, ele: "ja deveria ter a emenda direta: assim que ele esta ficando de pe ja cortasse para o idle alerta... sem
    // esperar ficar de pe do levantar-se, mas o ficar em pe do idle alerta") A entrega na subida: depois de levantar do chao,
    // com os dois pes no chao, a cabeca acima disto (m), a pelve acima de RiseHandoffPelvis e o tronco acima de
    // RiseHandoffUpright, o idle assume subindo - o clipe do idle nos motores (com a transicao idleBlendSeconds) termina de
    // levanta-lo. 0: so de pe e sem subir, como antes.
    float riseHandoffHead = 0.0f;
    // (04/11 11h25) A naturalidade da recuperacao (o levantar e o evitar): a multa por decisao com toda a aspereza e todas as
    // juntas no limite (0 = sem).
    float recoveryNatural = 0.0f;
    // (04/11 12h) A mao linear empurra no plano no trote de lado (2 e 6) - false: como nas costas, que nao puxam desde 07h00
    // (medido: sem o puxao o trote de lado melhora - semelhanca 0,81 -> 0,86, pes 0,050 -> 0,027; a corrida de lado, sem ele,
    // ainda nao anda).
    bool lateralHandPush = true;
    // (04/11 12h, ele: "para a corrida lateral vamos manter a velocidade, de mais permissividade no movimento das pernas para
    // possibilitar movimentos mais amplos") Na corrida de lado (2 e 6), o alcance da correcao nas pernas vezes isto, e a
    // semelhanca das pernas e dos pes cobrada pela metade.
    float lateralRunLegReach = 1.0f;
    // (04/11 12h35, ele: "a velocidade do trote 4 m/s (todas direcoes); na corrida 6 m/s para frente (e diagonais frente),
    // 5,5 para tras e 4,5 lados - fazer ele aprender a pegar impulso, fazer forca contra o solo, sem tentar apenas aumentar a
    // cadencia") No rastreio, o pedido fixo por rumo do trote e da corrida (m/s; 0: o do clipe +-15%, como antes). Com ele o
    // clipe fica na cadencia dele (escala 1): a velocidade a mais tem de vir da passada e do empurrao.
    std::array<float, 8> trotSpeeds {};
    std::array<float, 8> runSpeeds {};
    // (04/11 13h40) A coluna da referencia na largada de costas, de pe (rad, < 0 para tras; 0: so a BackLean de sempre).
    float backStartLean = 0.0f;
    // (04/11 14h) Nas curvas (5-7): > 0, o rumo persegue o percurso desenhado e mais longe dele que isto (m) por 0,5 s
    // encerra como a queda ("fora do percurso"); 0, o rumo gira no relogio do plano, como antes.
    float courseLimit = 0.0f;
    // (04/11 14h05) Nas curvas, a referencia inclinada para dentro na inclinacao que equilibra a curva (atan(v x giro / g)).
    bool curveLean = false;
    // (05/11 20h10, ele: "a pisada do trote... ele pisa em linha reta... verifica se isso esta correto, pois e estranho": a
    // rede pisa com 2,7 cm entre os pes contra 18,6 do clipe.) > 0: nos rumos retos e no sprint, a largura entre os pes (de
    // lado, no rumo do corpo) diferente da do clipe derruba a semelhanca: exp(-(diferenca / isto)^2); 0: sem (m).
    float footWidthSlack = 0.0f;
    // (05/11 21h, ele: "atacar as trocas de direcao") Nas trocas que usam a meia-volta (media, forte e a livre grande), na
    // corrida: antes dela a freada ate o trote no mesmo rumo, a meia-volta trotando (a da corrida freava de 5 m/s a quase 0 em
    // 0,3 s, impossivel ate com a mao cheia: 11-20% de quedas por troca contra 0% no trote) e a corrida de volta 0,6 s depois.
    bool runTurnBrake = false;
    // (05/11 21h40) A meia-volta entra no pe certo: espera ate um ciclo o pe que o clipe planta primeiro pousar no instante
    // do plantio do clipe (36% de quedas no pe trocado contra 0% no certo).
    bool runTurnAlign = false;
    // (05/11 22h40, ele: a media nao para a inercia, a forte engata direto sem arrancada) A troca assentada: de costas, a
    // freada ate 1,5 m/s antes da meia-volta (todas as quedas medidas na forca de agora comecam de costas).
    bool turnSettle = false;
    // (05/11 23h20, ele: "a forte engata direto sem arrancada gradual... o clipe agachado tem boa base para arrancada em base
    // baixa") Na media e na forte, a saida da meia-volta pela arrancada agachada (crouched_to_sprint) e dela o passo.
    bool turnLaunch = false;
    // (07/11, o "Levantar correndo") No levantar em movimento, de pe e com o pedido de velocidade, quem dirige e a cabeca da
    // locomocao (a recompensa continua a do levantar em movimento: correr ate o alvo); sem, a recuperacao do comeco ao fim.
    bool movingHandoff = false;
    // (06/11 04h40, ele: na corrida de frente "comecasse com a velocidade progressiva... so no comecinho, para evitar esse pulo
    // que ele da no comeco para comecar a correr... apos o clipe de pegar impulso") A largada progressiva da corrida: com o
    // pedido fixo, o impulso (idle_to_sprint) nesta escala (0: 1, como antes); o ciclo entra na cadencia em que ele acaba e
    // sobe ate a cheia em runLaunchEaseSeconds; e o pedido sobe dali a runLaunchAcceleration (0: StartAcceleration).
    float runLaunchScale = 0.0f, runLaunchEaseSeconds = 0.0f, runLaunchAcceleration = 0.0f;
    // (06/11 04h55) A cauda do rumo: o fator "reto" da nota da velocidade com esta fracao numa regua larga (0,6 rad) - so a
    // estreita (CourseSlack 0,10 rad) zerava acima de ~12 graus e a largada torta de 20-60 graus sem a mao nao tinha
    // gradiente para endireitar. 0: so a estreita, como antes.
    float courseTail = 0.0f;
    // (08/11 18h45) O rumo da nota numa velocidade suavizada mais longa (s; 0: a de 0,25 s de sempre).
    float courseSmoothing = 0.0f;
    // (20h05) Nos retos, o deslocamento fora do rumo pedido mais que isto (rad; 0: nao) por StraySeconds encerra como a queda.
    float courseAngleLimit = 0.0f;
    // (08/11 23h55) Os potenciais do levantar (Ng 1999) tambem nos passos da cabeca da locomocao. De pe valem 24, e
    // 0,99 x 24 - 24 cobrava 0,24 por passo de qualquer corrida (a nota do trote lateral e 0,18). false: sem os
    // potenciais ali, e a queda de pe custa 2x FallPenalty, o que o potencial cobrava junto na queda.
    bool locomotionShaping = true;
    // (23h55) A velocidade manda: andando, a nota vezes speedGate + (1 - speedGate) x a nota da velocidade (0: nao).
    // Num empurrao e ate 1,5 s depois, nunca abaixo de SpeedGatePushed.
    float speedGate = 0.0f;
    // (09/11 01h20) O joelho para dentro da linha quadril-tornozelo alem do clipe (+ 2 cm de margem) derruba a semelhanca:
    // exp(-(excesso / kneeInwardSlack)^2), andando. 0: nao.
    float kneeInwardSlack = 0.0f;
    // (09/11 03h40) No idle da cabeca da locomocao, a pelve abaixo da do idle alem de 1,5 cm derruba a semelhanca da pose:
    // exp(-(excesso / idleHeightSlack)^2). 0: nao.
    float idleHeightSlack = 0.0f;
    // (09/11 03h50) Na parada final, o idle so com o corpo tambem abaixo de StopBodySpeed (ou StopBodyTimeout depois da ordem).
    bool stopWaitBody = false;
    // (07/11, a receita nova) A faixa da velocidade: no treino, o pedido fixo do rumo vezes um sorteio por episodio em
    // [speedRangeLow, 1] - a rede treina em pedidos alcancaveis e aprende a velocidade como um controle continuo; a
    // avaliacao sempre na meta. 1: o pedido fixo, como antes.
    float speedRangeLow = 1.0f;
    // (07/11, ele: "no Parado, seria interessante que a quietude fosse recompensada, sem afetar e claro o equilibrio") A
    // quietude do parado: o custo das juntas alem da folga (o respirar do idle) e do sobe-desce do centro de massa; o peso
    // dela na nota do parado sobe de 0,15 para 0,25. 0: a quietude de antes.
    float idleQuiet = 0.0f;
    // (07/11, a sonda das trocas: a forte acabava virada - 10 de 24 - ou caida, a media caida em 15 de 24, logo depois da troca;
    // o pedido freava e reacelerava a 6 m/s^2, nivel de atleta de elite sem girar o peito.) A aceleracao da velocidade de
    // referencia nas manobras, m/s^2 (era a constante ManeuverAcceleration).
    float maneuverAcceleration = 6.0f;
    // (07/11, ele: "tentar fazer o nosso levantar nao dar esse pulinho no final") O custo de chegar de pe subindo rapido: perto
    // do topo (o centro de massa acima de RiseTopHeight), k (vz - RiseTopSpeed)^2 por decisao, com a recuperacao no comando.
    float riseOvershoot = 0.0f;
    // (06/11 06h) A cauda da inclinacao de lado: o fator da semelhanca pela inclinacao do peito contra a do clipe com esta
    // fracao numa regua larga (0,35 rad) - so a estreita (RollSlack 0,08 rad) zerava acima de ~10 graus e na curva de corrida
    // sem a mao a rede inclinada 12-28 graus para FORA nao tinha gradiente para inclinar para dentro. 0: como antes.
    float rollTail = 0.0f;
    // (06/11 07h, a investigacao: "tirar os penhascos da recompensa") Sem os penhascos: a inclinacao de lado cobrada tambem
    // nas curvas (5-7), contra a do clipe (que inclina para dentro), numa regua larga (0,35 rad) - nas curvas ela nao era
    // cobrada e a rede corria inclinada para FORA; o antebraco ("Forearm") nos bracos - caia no tronco e o cotovelo era
    // cobrado na regua estreita da coluna; e uma cauda larga (1/4) no pulo da pelve (morria a 7 cm - o salto da arrancada
    // voltou: 13 cm custavam o mesmo que 7) e no pe cruzado da corrida (morria a 5 cm). Desligado: como antes.
    bool cliffFree = false;
    // (06/11 08h15, o A/B: as partes em separado - a inclinacao nas curvas e o pe cruzado ajudaram as curvas e a corrida
    // 0/1/7; a cauda do pulo PIOROU o salto da arrancada (70 -> 95%: amaciou a punicao dos pulos grandes) e o antebraco e o
    // pulo, que valem em todos, sao os suspeitos da piora nos trotes 3/5 e nas corridas 2/6.)
    bool cliffCurveRoll = false, cliffFootCross = false, cliffForearm = false, cliffHop = false;
    // (06/11 08h25, o salto da arrancada voltou - 20% -> 70% das largadas com a pelve acima de 0,92 m em 2 h: o pulo da pelve
    // na nota satura (um de 13 cm custa o mesmo que um de 7) e na arrancada a velocidade pesa mais) Na janela da arrancada da
    // corrida, a nota cai em linha reta com a pelve acima da do clipe alem de 3 cm, ate 60% a launchRiseSpan alem; 0: sem.
    float launchRiseSpan = 0.0f;
    // (08/11, ele: "sem semelhanca alguma com os clipes... sem atender a velocidade": a 0% o relogio do clipe nunca esta na
    // fase do passo do corpo - a semelhanca no relogio ~0 em toda rede, a professora inclusive - e as comparacoes com o
    // clipe no relogio (a pose, o pulo da pelve, a inclinacao, o giro) cobram fase em vez de forma: o passo com voo, fora
    // de fase, paga como pulo. Ligado: no pouso do pe esquerdo nos rumos retos, a fase do clipe volta ao pouso esquerdo dele.)
    bool phaseSync = false;
    // (08/11 03h45: a imitacao da v18 morta longe do clipe - as pernas a 1,24 pagavam 0,30 exp(-3,7) = 0,007, a coluna e a
    // cabeca ~0; so os pes tinham inclinacao. Andando, os expoentes da imitacao divididos por isto - a regua larga do
    // comeco -; parado e no giro, como antes. 1: sem.)
    float imitationLoosen = 1.0f;
    // (08/11 05h15: nas trocas a falha e o peito virar - 14-29% das manobras acabam "viradas" (o peito a mais de 100 graus
    // da vista por 0,5 s), quase nenhuma cai - e so o encerramento cobrava: nenhum sinal antes. Andando, a nota vezes
    // 1 - g + g exp(-(excesso / 0,6 rad)^2), o excesso o rumo da pelve contra o do clipe alem de 20 graus; 0: sem.)
    float facingGate = 0.0f;
    // (08/11 18h40) O peito para a vista nos diagonais retos: o portao do peito alto do corpo contra o da referencia.
    float diagonalChestGate = 0.0f;
    // (08/11 08h40, ele: "PRINCIPALMENTE corrigir esses movimentos zuados dos bracos": na locomocao livre os bracos viraram
    // ferramenta de equilibrio - a mao sobe a cabeca uma vez por passada. >= 0: andando, com a rede no volante e um clipe
    // seguido, os motores dos bracos recebem o alvo e a velocidade do clipe (na fase do passo, com a fase presa) mais a
    // correcao da rede, isto vezes (saida - meio); < 0: livres.)
    float armClipReach = -1.0f;
    // (08/11 10h40) O mesmo para o pescoco e a cabeca (< 0: livres).
    float headClipReach = -1.0f;
    // (06/11 08h50, a vigilia: o trote lateral a 0% cai na largada - em 1,2 s, o corpo girando, 0,6 m/s contra 4 pedidos -: de
    // pe a 4 m/s de lado em 0,8 s sem a mao ele nao arranca) A largada dos rumos de lado (2 e 6, trote e corrida) a esta
    // aceleracao (m/s2; a rampa dura o pedido / ela); 0: a de todos, StartAcceleration (5).
    float lateralStartAcceleration = 0.0f;
    // (06/11 09h40, a vigilia: o levantar a 0% nem sai do chao - o clipe puro sem a mao ergue a pelve so a 0,42 m (com a mao, a
    // 0,98): os motores do rastreio seguem o clipe com 20% da rigidez (TrackingStiffnessScale) e a rede so desvia um pouco os
    // alvos. A rigidez dos motores no levantar guiado (a escala; 0: a do rastreio, 0,2) - o musculo, nao uma forca de fora.)
    float riseStiffness = 0.0f;
    // (06/11, a locomocao livre: sem o clipe nos motores a velocidade-alvo era 0 e o amortecimento freava a passada inteira -
    // a professora no modo livre de pe so 8-24% no trote de frente, contra 100% com o clipe. Com isto, na cabeca da
    // locomocao a velocidade-alvo dos motores e a do proprio alvo da rede - (alvo de agora - o da decisao anterior) / dt -,
    // como o clipe dava a dele; sem clipe nenhum.)
    bool freeVelocityFeedforward = false;
    // (06/11 17h25) A escala do amortecimento dos motores da cabeca da locomocao sem clipe (0: a do rastreio, 0,5).
    float freeDampingScale = 0.0f;
    // (06/11 17h40, ele: "liberdade total nos motores") A rede da locomocao livre da tambem a velocidade-alvo de cada eixo:
    // a acao ganha, depois dos canais do passo, um eixo de velocidade por eixo (rad/s, ate VelocityActionLimit). Sem ela, a
    // velocidade-alvo era 0 e nem a copia perfeita da professora andava (de pe 0,83 -> 0,25): o clipe dava a dele.
    bool velocityActions = false;
    // (06/11 17h55) Com velocityActions, a cabeca da recuperacao tambem usa as velocidades da acao (o levantar).
    bool recoveryVelocity = false;
    // (06/11 22h10) Os pesos do sorteio das 7 manobras no treino (troca suave, media, forte, livre, curva suave, S, fechada).
    std::array<float, 7> maneuverWeights { 0.12f, 0.16f, 0.16f, 0.16f, 0.12f, 0.14f, 0.14f };
    // (06/11 10h50, a vigilia: as trocas media/forte paradas em ~15% ha 2 h - no atlas, a meia-volta saindo de LADO cai 30-69%
    // no trote e 100% correndo (de frente 5-23%): de lado ela entra a 4 m/s, a do trote) De lado (2 e 6), a freada ate esta
    // velocidade antes da meia-volta (m/s; 0: sem - em 05/11 a 1,5 m/s, na forca de entao, derrubava no passo lateral).
    float turnSettleLateralSpeed = 0.0f;
    // (09/11 04h30) De frente (0, 1, 7), a freada ate esta velocidade antes da meia-volta das trocas; 0: sem.
    float turnSettleFrontSpeed = 0.0f;
    // (09/11 04h35) A meia-volta das trocas tocada nesta fracao da velocidade do clipe (1: como ele; 0,6: 1,7x mais longa).
    float runTurnTimeScale = 1.0f;
    // (09/11 05h) Na parada final, a passada que desacelera (a cadencia com a velocidade, o idle abaixo de 0,5 m/s) em todos os
    // rumos - antes so nos laterais.
    bool stopGaitAll = false;
    // (09/11 06h15) No idle da cabeca da locomocao, fora do empurrao, a nota inteira vezes 1 - g + g x a pose (a altura e o peito alto
    // contra os do idle: 6 graus de folga, largura 10). 0: nao.
    float idlePoseGate = 0.0f;
    // (09/11 08h55) Com stopClips, os clipes de parada so de frente e nas diagonais da frente (de costas e de lado, a freada antiga).
    bool stopClipsFront = false;
    // (09/11 10h10) Com stopClips, o soltar da tecla do comando tambem pelo clipe de parada (nos rumos em que ele vale).
    bool stopClipsCommand = false;
    // (09/11 10h50) Freando (a parada final e o soltar da tecla), o portao da velocidade nunca abaixo disto; 0: como andando.
    float stopGateFloor = 0.0f;
    // (09/11 14h, ele: "nao acho que 'seguir' clipe vai ser uma boa ideia para frear, pois e algo relacionado a recuperacao
    // tambem.. e isso precisa ser emergente do RL") A parada livre: na parada final dos retos e no soltar da tecla do comando, a
    // cabeca da locomocao dirige ate ficar parada de pe (a recuperacao so caido ou baixo) e a nota e o objetivo - a velocidade a
    // zero, de pe, o peito na vista, a qualidade do parado -, sem referencia que freia, clipe de parada nem pes presos ao idle.
    bool freeStop = false;
    // (10/11 04h, ele: "conseguiu a freada, mas sacrificou a postura") A postura na parada livre: a nota inteira da parada vezes
    // 1 - G + G x pose, pose = o peito alto e a cabeca contra os do idle (folgas 5,7 e 8,6 graus), do comeco ao fim da parada. Sem
    // ela o tronco ia livre ate 20 graus e a cabeca da locomocao - a mesma do trote e da corrida - dobrou em tudo (diario 42). 0: sem.
    float stopPosture = 0.0f;
    // (11/11 03h29, ele: "ao frear fica todo molengo e nunca volta a pose do idle alerta"; "a pose deve ser cobrada sempre, mas
    // sem prejudicar o auto-equilibrio". Medido no codigo: parado, a nota da parada livre e 0,70 + 0,30*stand - 70% so por ter
    // parado, a pose do idle 6%.) E5-7: conforme ele para (near), a nota passa a ser a pose do idle alerta na regua estrita
    // (o DeepMimic inteiro, a coluna junta a junta inclusive): (1 - k*near)*nota + k*near*face*(0,25 + 0,75*pose),
    // pose = metade a semelhanca estrita e metade a cauda longa por junta. 0: sem.
    float stopIdlePose = 0.0f;
    // (10/11 04h, ele: "desde agora precisamos cobrar a coluna correta, cabeca, bracos com mais rigor durante as locomocoes e
    // parado") A regua da imitacao da coluna, da cabeca e dos bracos vezes isto (andando, no giro e nas trocas); 1: sem.
    float upperRigor = 1.0f;
    // (10/11 04h, ele: "a arrancada da corrida que e estranha, deveria ser a mesma arrancada igual do unreal engine") De pe, nos retos
    // da corrida: a arrancada da Unreal do rumo (corrida_start_<rumo>_<pe>, o pe pelo idle) antes do ciclo, a velocidade pedida a da
    // raiz dela, a saida a 92% da velocidade de cruzeiro e o ciclo no quadro com as pernas mais parecidas; sem: o idle_to_sprint na
    // frente e a rampa nos outros rumos.
    bool unrealStarts = false;
    // (10/11 06h05) A escala de reproducao da arrancada da Unreal (0: a que leva a velocidade de saida dela a pedida, ate 1,25).
    float unrealStartScale = 0.0f;
    // (10/11 06h35, ele: "nada de passinhos nem lentidao, apenas o IMPULSO real da corrida") Na janela da largada: o progresso inteiro na
    // velocidade (todos os rumos) e a imitacao pelo quadro mais parecido do clipe de largada (+-0,3 s do relogio).
    bool liveLaunch = false;
    // (10/11 07h05, ele: "o maior ganho de velocidade inicial com o menor numero de passos, sem bracos estabanados e pulinhos") Na
    // janela da largada, cada pe que pousa paga (tempo no ar - 0,15 s) x isto: o passinho perde, a passada longa ganha. 0: sem.
    float launchStepReward = 0.0f;
    // (10/11 07h10, ele: "executar literalmente o movimento do clipe e reforcar em largada, pelo menos ate ele aprender") A parcela do
    // clipe nos motores na janela da largada (o alvo e a velocidade-alvo); o treino a desce ate 0. 0: sem.
    float launchClipShare = 0.0f;
    // (10/11 07h15, aprovado por ele: "mao so na largada") O nivel da mao na pelve na janela da largada (linear); o treino o desce a 0.
    float launchHand = 0.0f;
    // (10/11 08h20, ele: "se conseguir pelo menos tirar os passinhos no lugar e os bracos estabanados na largada, estara otimo") Na
    // janela da largada: cada pe que pousa sem ter andado 15 cm no rumo pedido custa ate isto (para tras, o maximo); 0: sem.
    float launchInPlacePenalty = 0.0f;
    // E os bracos da pose do idle a do clipe com a velocidade do corpo (cheios a 1,5 m/s).
    bool launchCalmArms = false;
    // (10/11 10h40, a coluna acoplada - jointCouplings do perfil) Nos eixos acoplados da coluna, a velocidade-alvo e a do proprio
    // alvo de posicao da rede (a diferenca dele de uma decisao para outra), nao a do canal livre da acao: medido, a rede usava o
    // canal como torque puro contra o batente (a coluna travada no maximo, 82-100% do parado). false: o canal livre, como antes.
    bool spineVelocityFeedforward = false;
    // (10/11 11h45, ele: "quando parado, sempre o IDLE alerta e o alvo... recobrar a pose assim que possivel, com o minimo de
    // passos", "nao e troca de objetivo"; TrackingBasePosture3D) A postura base como atrator, com o pedido de velocidade zero (da
    // ordem de parar ao parado, nos empurroes): estar perto do idle alerta pela regua que nao satura (na parada livre e na base do
    // parado), o progresso para ele premiado a cada decisao (o potencial -d, Ng 1999) com este peso, na proporcao do quanto o corpo
    // ja freou, e cada passo de ajuste custando BasePostureStepCost. 0: como antes.
    float basePosture = 0.0f;
    // (10/11 13h45, ele: "as poses gerais ficaram molengas desengoncadas... ele precisa ficar mais rigido... ao frear e pos-freio ele
    // fica todo molengo") O tonus do corpo: a rigidez e o amortecimento de todos os motores vezes isto (o limite de torque - a forca -
    // nao muda). 1: como antes.
    float motorTone = 1.0f;
    // (10/11 14h, ele: "ao frear e pos-freio ele fica todo molengo"; medido: na freada a lombar no batente (44 graus) e o tronco
    // caido em C, e a nota nao puxava de volta - os nucleos do tronco (gaussianas de ~10 graus) davam ~0 a 45 graus.) A inclinacao
    // da pelve, do peito e da cabeca contra o clipe na regua de cauda longa, 1/(1+(d/10 graus)^2), nesta parcela da nota inteira do
    // rastreio (locomocao, freada, giro): longe do clipe ainda puxa. 0: como antes.
    float postureTail = 0.0f;
    // (10/11 14h25, ele: "pos-freio... parece a coluna quebrada") O tonus so da coluna (Abdomen, Chest, UpperChest, todos os eixos):
    // a rigidez e o amortecimento vezes isto. Medido: as 3 juntas de 440 N.m/rad em serie dao ~150 N.m/rad no tronco, contra ~250-440
    // in vivo com os musculos pre-ativados (Moorhouse & Granata 2005; Andersen 2004) e ~785 (Cholewicki 2000). 1: como antes.
    float spineTone = 1.0f;
    // (10/11 14h30, ele: "pos-freio parece a coluna quebrada", "a coluna torta"; medido: a rede empurra as juntas da coluna contra os
    // batentes para se enrijecer - a lombar no batente de flexao na freada, a torcao no batente no parado.) O custo do batente: por
    // decisao, o torque que os batentes da coluna fazem (todos os eixos) alem de JamSlack, em unidades de JamScale, vezes isto.
    // Encostar no fim nao custa; empurrar o osso contra ele, sim (como o termo de limite de junta do legged_gym). 0: como antes.
    float jamCost = 0.0f;
    // (10/11 15h, ele: "ele nao consegue ficar imovel, os pes deslizando... eu quero ele estatico quando parado"; medido: apoiados,
    // os pes deslizam 5,5 cm/s e giram 13 graus/s sem nunca sair do chao.) Com o pedido zero e o corpo parado (voltando a nada ate 0,5
    // m/s), cada pe apoiado que desliza ou gira no chao custa isto vezes min(2, v/5 cm/s + giro/0,5 rad/s) por decisao. O pe no ar
    // (o passo de reequilibrar) nao paga. 0: como antes.
    float footSlipCost = 0.0f;
    // (10/11 15h25; medido: o clipe do idle tocado direto nos motores, sem a rede, nao fica de pe parado - 11 passos, os pes deslizam
    // 247 cm em 8,5 s: sem compensar a gravidade, cada motor e uma mola que o peso afunda (o joelho ~9 graus) e nenhuma pose fica.) A
    // compensacao da gravidade nos motores da cabeca da locomocao: o torque que segura o peso de cada osso na pose de agora (o tonus
    // antigravitacional), dentro da mesma forca maxima do motor. false: como antes.
    bool motorGravity = false;
    // (10/11 04h40, ele: "ensinar a fazer pelos clipes, para ter base e DEPOIS de estar fazendo o movimento dos clipes deixar liberdade")
    // Com stopClips e freeStop: na parada final dos retos, a parada da Unreal do rumo (onde ha uma boa) e, quando ela assenta, a livre
    // (a cabeca da locomocao ate ficar de pe); sem clipe para o rumo, a livre direto.
    bool stopClipsBase = false;
    // (10/11 05h, ele: "ve qual dos tres fica melhor") A parada de frente por tres referencias - a da Unreal, a tocha da Mixamo e a "Running
    // To Turn" sem o giro -, sorteada a cada parada no treino (stopFrontVariant -1) ou fixa (0, 1, 2) na medida.
    bool stopFrontVariants = false;
    int stopFrontVariant = -1;
    // (05h10) As variantes no sorteio do treino (bits: 1 Unreal, 2 tocha, 4 virada, 8 "Running To Stop").
    int stopFrontVariantMask = 7;
    // (10/11 05h25, ele: "faca" - a "Running To Stop" com a entrada gradual) A entrada no clipe de parada: a passada se mistura nele por
    // stopEntryBlend s (a inclinacao para tras cresce aos poucos) e o instante de entrada nos primeiros stopEntryWindow s dele. 0,2 e 0,3: como antes.
    float stopEntryBlend = 0.2f;
    float stopEntryWindow = 0.3f;
    // (09/11 15h, a etapa 2 do mesmo principio) A troca livre: nas trocas media, forte e livre e na troca de tecla do comando, o
    // pedido novo (a tecla, ou a vista girando na meia-volta) vira o objetivo por uma janela de tempo fixo pelo tamanho da troca -
    // a velocidade nova, o peito na vista, de pe -, sem passada nem meia-volta para seguir; no fim dela o rastreio volta no ciclo
    // do rumo novo, na fase das pernas do corpo.
    bool freeTroca = false;
    // (09/11 15h40) Na parada livre, ja parado (perto de zero), a nota inteira vezes 1 - freeStandGate (1 - pose), pose = a altura
    // da pelve vezes o peito alto reto contra os do idle - as folhas da r75500 paravam de pe, mas curvadas e agachadas 4-6 cm, a
    // base mais estavel pagava quase o mesmo. 0: so a parcela somada de antes.
    float freeStandGate = 0.0f;
    // (09/11 17h30, a etapa 3a) O parado livre: no parado com a cabeca da locomocao (o exercicio parado e o do equilibrio com
    // empurroes), a nota e a do parado da parada livre - quieto, de pe e ereto na altura do idle, o peito na vista, a pose do idle
    // so como termo leve -, sem o gerador de passos e sem o fim por pose perdida (os pes fora do lugar do idle); cair encerra.
    bool freeIdle = false;
    // (09/11 19h15, ele: "dar uma reforçada nas POSES em geral... a coluna mole, arqueada e arrancadas estranhas... a pose
    // parado se recontorce") O reforço das poses: andando, a nota inteira vezes 1 - poseBoost (1 - pose), pose = a coluna (o erro
    // por junta contra o clipe), a inclinação alem da do clipe e a pelve abaixo da dele; na arrancada a coluna e a cabeça
    // cobradas inteiras; no parado (da parada livre e do exercício parado), a coluna contra a do idle no portão da pose. 0: sem.
    float poseBoost = 0.0f;
    // (10/11 00h20, ele: "cobrar semelhanca alta com o CLIPE SEM interferir nos momentos de que ele necessita sair do clipe para se
    // recuperar, como pos empurroes, recuperacoes e freadas") O empurrao livre: andando, durante o empurrao e PushFreeSeconds
    // depois, sem imitacao - a nota e voltar ao movimento pedido, de pe, com o peito na vista.
    bool freePush = false;
    // (10/11 00h35, ele: "o Parado precisa ser mais reforcado ainda essa semelhanca com o clipe") No parado livre e no parado da
    // parada livre, a semelhanca com o idle (a regua do contrato com suavidade 2) vale este peso da qualidade de pe; o resto
    // (altura, peito reto, quieto) o complemento. 0: a qualidade de antes (a semelhanca 0,20).
    float freeIdleMatch = 0.0f;
    // (10/11 00h35, ele: "os passos normais... nas freadas e recuperacoes, nao precisem ser esses passos elevados para cima (trotar
    // parado)... algo mais natural") Nas fases livres (parada, parado, pos-empurrao), o pe acima da altura de repouso alem desta
    // folga (m) custa ate 40% da nota (largura 6 cm). 0: sem.
    float freeStepHeight = 0.0f;
    // (10/11 01h30) O parado dobrado encerra como a queda, sozinho (sem o resto do poseBoost): parado, abaixo de 0,5 m/s e fora dos
    // 2 s depois de um empurrao, o peito alto a mais de 34 graus do idle por 0,5 s. Sem ele o parado voltou a dobrar na cintura.
    bool idleBentEnd = false;
    // (11/11, E5-4) A linha final do episodio com o feedback do passo final: o begin() do recomeco roda dentro do step e
    // zerava o locomotionFeedback antes de o Python le-lo - a nota (o teto do custo), o portao do estilo e as medias rastreio_*
    // contavam um zero. Ligado, o feedback do passo final volta depois do begin() (o proximo decide o zera de novo).
    bool finalRow = false;
    // (10/11 01h45) O parado visto como o parado da parada livre: no parado da cabeca da locomocao, sem o relogio do clipe do idle
    // na observacao e com a vista contra o peito de verdade (antes fixa em "de frente"). A mesma rede fica de pe depois de uma
    // parada livre e se dobra na cintura no exercicio parado - o teste de que o contexto liga o modo dobrado.
    bool idleLikeStop = false;
    // (10/11 01h50, ele: "Esta totalmente errado a forma do parado.. investigue profundamente e corrija") Medido: o parado agachado
    // 15 cm e o peito 18 graus, e ali a nota plana - a altura num nucleo de 3 cm, a semelhanca 0,03. A altura e o peito alto com o
    // nucleo fino mais uma rampa larga (ate 20 cm e 40 graus), e a semelhanca com o idle tambem na regua larga (suavidade 6).
    bool idleWide = false;
    // (10/11 02h30) A simetria nos clipes ciclo_*: o espelho (mirror_info) so ligava nos nomes antigos (jog, jog_strafe_left...) e
    // os retos de agora usam ciclo_trote_* e ciclo_corrida_* - a perda de simetria estava desligada (o espelho em 0% dos retos), e o
    // lateral direito ia a 92-94% da pedida com o esquerdo a 11-40%. Meio ciclo: frente e costas; pares: esquerda e direita.
    bool mirrorCycles = false;
    // (06/11 12h, o atlas: correndo, a meia-volta saindo de costas ou de lado cai 93-100% NO MEIO do giro (~0,3 s depois da
    // troca), com 1,5 m/s para tras ou 4 de lado contra o giro - de frente, 8-23%) De costas, a freada ate esta velocidade
    // antes da meia-volta (m/s; 0: TurnSettleSpeed, 1,5).
    float turnSettleBackSpeed = 0.0f;
    // (06/11 13h05, a vigilia - para ELE decidir: a meia-volta da media/forte saindo de costas ou de lado cai 93-100% correndo
    // no meio do giro, e freada nenhuma resolveu) Saindo de costas ou de lado (rumos 2-6), a media e a forte trocam o rumo de
    // peito fixo, como a suave (sem a meia-volta); de frente seguem pela meia-volta. Desligado: como antes.
    bool trocaFixedChestNonForward = false;
    // (06/11 01h10, ele: "a troca de direcoes ainda esta muito fragil... pense bem o que fazer para ajudar ele a aprender
    // melhor") A pratica das trocas: o rumo de partida das trocas 1-3 pelos pesos dados (o treino manda, pelas quedas de
    // cada rumo; todos 0: por igual), o intervalo entre trocas 1-2 s e a troca que chega no meio da anterior esperando.
    bool trocaPractice = false;
    std::array<float, 8> trocaHeadingWeights {};
    // (05/11 22h20/22h55, ele: "na corrida ele esta cruzando passos") > 0: na corrida para a frente e nas diagonais da frente,
    // o pe apoiado mais para dentro que o mesmo pe do clipe (alem de 1,5 cm) derruba a semelhanca: exp(-(falta / isto)^2) (m).
    float footCrossSlack = 0.0f;
    // (05/11 21h10, o levantar) A estabilidade medida no idle (o portao do levantar e do parado) fora da janela de cada
    // empurrao e com a ancora recomecando depois dele; a recompensa nao muda.
    bool idleReanchor = false;
    // (04/11 23h40, ele: "nas trocas de direcao nao ha movimento de pegar no tranco... nem de freada e inversao") Nas trocas
    // (1-4), a coluna da referencia (nos motores) inclina com a aceleracao da velocidade pedida no rumo do peito, atan(a / g),
    // ate este angulo (rad): freando para tras, arrancando no rumo novo para a frente - qualquer angulo de troca; 0: sem.
    float accelLean = 0.0f;
    // (04/11 14h15) No trote de frente e nas diagonais da frente: a coluna da referencia para a frente (rad) e, na largada de
    // pe, a da largada, voltando a ela com a janela da arrancada (0: como o clipe).
    float frontLean = 0.0f;
    float frontStartLean = 0.0f;
    // (05/11 00h40, ele: "no corrida, nao seria melhor se o corpo todo ficasse mais inclinado para frente? nao a coluna apenas
    // mas o corpo todo") Na corrida de frente e nas diagonais da frente (e nas curvas da corrida): a referencia inclinada
    // runLean (rad) - a pelve e a coluna retas, como a inclinacao do trote - com runLeanLegs dela nas pernas (0: so o dorso;
    // 1: o corpo todo girado nos quadris). 0: como o clipe.
    float runLean = 0.0f;
    float runLeanLegs = 0.0f;
    // (05/11 04h10, ele: "como fizemos com a corrida de frente, inclinar o corpo todo... nesse caso para tras, algo sutil")
    // Na corrida de costas e nas diagonais de costas (rumos 3-5): a referencia inclinada runBackLean (rad) para tras, com
    // runBackLeanLegs dela nas pernas, como a runLean.
    float runBackLean = 0.0f;
    float runBackLeanLegs = 0.0f;
    // (05/11 01h05, ele: "vamos fazer um treino de SPRINT com a velocidade meta de 8 m/s, e ai usamos essa nova pose" - a
    // imagem "CORRETA": o corpo todo tombado para a frente, a perna de tras em linha com o tronco) O sprint (a faixa 3): so
    // para a frente, o clipe sprint_maximo largando de pe pela arrancada, no pedido fixo sprintSpeed (m/s; 0: o do clipe),
    // com a inclinacao sprintLean (rad, o dorso reto) e sprintLeanLegs dela nas pernas, como a runLean.
    float sprintSpeed = 0.0f;
    float sprintLean = 0.0f;
    float sprintLeanLegs = 0.0f;
    float idleBlendReach = -1.0f;
    // (04/11 15h, ele: "os passos de recuperacao e estabilizacao, predict de movimento e etc. para o auto-equilibrio... nos dois
    // cerebros"; vault 04_locomocao/plano_passos_de_recuperacao_04_11.md) O equilibrio:
    //  - balanceObservation: as entradas do equilibrio (o layout levantar-v7, antes da habilidade) - o ponto de captura contra a
    //    base e os pes, a margem, a ameaca, omega, o giro do corpo (TrackingGetUpContract3D::observeBalance) e o estado do
    //    gerador de passos (zeros fora do idle da locomocao);
    //  - stepChannels: os 4 canais da rede para o passo, depois dos eixos da acao (crus, -1..1);
    //  - balanceSteps: o gerador de passos (TrackingBalanceSteps3D) no idle da cabeca da locomocao, com a nota do idle que
    //    deixa o passo acontecer (a semelhanca contra a referencia do passo, o equilibrio, a base do idle);
    //  - idleThroughPushes: o idle segue no empurrao (a locomocao o segura com o passo; caindo, a recuperacao);
    //  - balanceDrillFraction/balancePushLevel: a fatia dos comecos de pe (o parado) com empurroes, sem a mao - o equilibrio
    //    parado -, e o nivel deles (0-1 das faixas pushMinimum/MaximumNewtons e do continuo; o curriculo do treinador).
    // (balanceObservation e stepChannels mudam os tamanhos: so no construtor.)
    bool balanceObservation = false;
    bool stepChannels = false;
    // (11/11, E5-6: a nota do parado cobra a deriva da ancora - idleAnchor, onde o parado comecou, re-ancorada depois de cada passo
    // do gerador -, mas a entrada nao a tinha.) idleAnchorObservation: 2 entradas antes da habilidade - a ancora na frente e ao
    // lado da pelve, em 10 cm (ate 3); zeros fora do parado ancorado da locomocao. Muda o tamanho: so no construtor.
    bool idleAnchorObservation = false;
    bool balanceSteps = false;
    bool idleThroughPushes = false;
    float balanceDrillFraction = 0.0f;
    float balancePushLevel = 0.5f;
    // (As sondas do gerador: a carga maxima do pe que sai, o peso da carga e o pe no ar pelo IK a partir da pelve de agora.)
    float stepLoadLimit = 0.8f, stepLoadWeight = 0.4f;
    // (04/11 15h30, a sonda: o levantar acaba num pulinho - os dois pes no ar ~0,35 s, a cabeca 14 cm acima de de pe - e entrega
    // o corpo embalado) O levantar que chega equilibrado: na entrega ao idle, este premio vezes a qualidade da chegada (o ponto
    // de captura dentro da base, sem velocidade no plano, sem subir); de pe quase (cabeca acima de 1,2 m) com a recuperacao,
    // os dois pes no ar custam isto por decisao (0: desligados).
    float arrivalBonus = 0.0f;
    float flightPenalty = 0.0f;
    // (04/11 16h40, a folha do levantar: ele sobe explosivo, os bracos para o alto, passa do ponto e cai de costas - entregue no
    // meio do tombo, a 0,4-1,1 m/s e o ponto de captura 10-88 cm fora da base; o premio da chegada e de uma decisao so) A
    // subida por cima dos pes: com a recuperacao no comando e a cabeca acima de RiseHeadHeight, o ponto de captura fora da
    // base (ate 30 cm) e a velocidade do centro de massa no plano (ate 1 m/s) custam ate isto por decisao (0: desligada).
    float riseBalancePenalty = 0.0f;
    // (04/11 21h35: com a multa da subida o levantar sobe 98%, mas a entrega ainda pega o corpo a 0,5-1 m/s e 84-90% caem
    // no idle) A entrega calma: depois de levantar, a recuperacao so entrega com o ponto de captura dentro da base (ate 5 cm
    // fora) e o centro de massa abaixo de 0,4 m/s no plano - ou este prazo (s) de pe; 0: a entrega na hora de antes.
    float calmHandoffSeconds = 0.0f;
    // (04/11 15h45, a avaliacao do treino: parado sem a mao, sem empurrao, sem cair 0-12% - o idle so fica de pe com a mao; o
    // alcance da correcao no idle, 0,02 rad nas pernas, nao cabe nem a estrategia do tornozelo) A autoridade sob ameaca: no
    // idle, o alcance vai do de sempre (idleResidualScale, a pose) a este (a fracao do alcance cheio) com a ameaca do ponto de
    // captura (1 - margem/8 cm: de 0,4 - 4,8 cm dentro da borda - ao cheio na borda). 0: desligada.
    float idleThreatReach = 0.0f;
    // (04/11 23h10, ele: "APENAS o levantar-se em cabeca separada, o ato de ficar em pe na cabeca da locomocao") O evitar a
    // queda no idle da cabeca da locomocao: de pe no comeco ela segura, o empurrao incluido; a recuperacao so caido.
    bool avoidOnIdle = false;
    bool stepSwingFromBody = false;
    // A forca auxiliar da corrida (a faixa 2 nos rumos; < 0: a mesma da locomotionAssist): a corrida comeca com a mao
    // dela, sem esperar o trote.
    float runAssist = -1.0f;
    // As manobras do rastreio (04/11, ele: "adaptar os treinos de TROCAS de direcoes e curvas que possuiamos no b21"): a
    // fatia dos episodios de rumo com manobra e, na avaliacao, a manobra (-1 nenhuma; 0 uma por mundo em ciclo; 1 troca
    // suave 45 graus, 2 media 90, 3 forte 135-180 - as tres de peito fixo -, 4 troca livre - o olhar gira para o rumo novo
    // a 4 rad/s e o peito vai junto -, 5 curva suave R 12-18 m, 6 curva em S R 5-10, 7 curva fechada R 3-8 - as curvas
    // com o peito no caminho). E a forca auxiliar delas (< 0: a do rumo).
    float maneuverFraction = 0.0f;
    int evaluationManeuver = -1;
    float maneuverAssist = -1.0f;
    // A forca auxiliar por exercicio (04/11 03h45, ele: "um sistema em que POR TREINO ele ve o quao bem ele esta adaptado
    // aquele nivel de forca auxiliar... e vai diminuindo dinamicamente"): 0-7 os rumos do trote, 8-15 os da corrida, 16-22
    // as manobras (1-7), 23 o giro parado; < 0 usa a do grupo (locomotionAssist/runAssist/maneuverAssist).
    // (24, 04/11 04h55: o parado - e o idle depois de levantar - na cabeca da locomocao.)
    // (25-27, 04/11 11h50, ele: "adicionar os exercicios das curvas agora em corridas": as curvas 5-7 na faixa da corrida.)
    // (28, 05/11 01h05, ele: "vamos fazer um treino de SPRINT com a velocidade meta de 8 m/s"): o sprint para a frente.
    // (29, 05/11 04h40, ele: "vamos fazer seguir o clipe para o levantar; no futuro, depois que tiver bonitinho e funcional,
    // vamos soltando devagar"): o levantar pelo clipe (getUpTracking).
    // (30-35, 08/11: a estrela - trote fixo, livre, girando; corrida fixo, livre, girando.)
    // (36, 08/11 14h40: o comando.)
    static constexpr std::size_t DrillAssistCount = 37;
    static constexpr std::size_t StarDrill = 30;
    static constexpr std::size_t CommandDrill = 36;
    static constexpr std::size_t SprintDrill = 28;
    static constexpr std::size_t GetUpDrill = 29;
    std::array<float, DrillAssistCount> drillAssist { -1.0f, -1.0f, -1.0f, -1.0f, -1.0f, -1.0f, -1.0f, -1.0f, -1.0f, -1.0f,
        -1.0f, -1.0f, -1.0f, -1.0f, -1.0f, -1.0f, -1.0f, -1.0f, -1.0f, -1.0f, -1.0f, -1.0f, -1.0f, -1.0f, -1.0f, -1.0f, -1.0f,
        -1.0f, -1.0f, -1.0f, -1.0f, -1.0f, -1.0f, -1.0f, -1.0f, -1.0f, -1.0f };
    // A cabeca da locomocao corrige o clipe (03/11 14h10): com ela agindo, o alvo de cada junta e o do clipe do passo
    // (no quadro do fim da decisao) mais a acao, em radianos - a correcao residual (o v8 do RL v1, a unica marcha com cara
    // humana do projeto); sem isto, a mistura do clipBlend.
    bool locomotionResidual = false;
    // Fixed curriculum layout in the laboratory; -1 preserves the benchmark.
    int sceneryPreset = -1;
    // O episodio (s) e, no treino, a fatia que comeca num instante sorteado de um clipe de levantar.
    float episodeSeconds = 6.0f;
    float clipStartFraction = 0.25f;
    // A fatia que comeca de pe, num instante do clipe parado (31/10: ele nao se mantinha de pe depois de levantar).
    float standingStartFraction = 0.15f;
    // O levantar v2 (31/10 17h, ele): a fatia que comeca caida e levanta ja indo para um alvo (em arrancada, virado
    // para ele) e a do evitar a queda (de pe, um empurrao que o derrubaria: evitar o chao e se recompor rapido).
    float movingFraction = 0.0f;
    float avoidFraction = 0.0f;
    // A fatia que comeca na arrancada agachada (o clipe crouched_to_sprint, virado para um alvo, com o pedido de ir a
    // ele; 31/10, ele: "nao e apenas um levantar e entao ir, mas sim um levantar preparado ja indo").
    float sprintStartFraction = 0.0f;
    // O alvo do levantar em movimento: a distancia (m) e a velocidade pedida (m/s).
    float targetMinimumMeters = 4.0f;
    float targetMaximumMeters = 8.0f;
    float moveMinimumSpeed = 2.5f;
    float moveMaximumSpeed = 5.0f;
    // O empurrao do evitar a queda: o curto (N, 0,1-0,2 s) e, numa fatia, o continuo (N, s).
    float avoidMinimumNewtons = 1200.0f;
    float avoidMaximumNewtons = 1800.0f;
    float avoidHoldFraction = 0.3f;
    float avoidHoldMinimumNewtons = 350.0f;
    float avoidHoldMaximumNewtons = 550.0f;
    float avoidHoldMinimumSeconds = 0.5f;
    float avoidHoldMaximumSeconds = 1.2f;
    // O empurrao de pe do jogo de pes e do autoequilibrio parado (N, 0,1-0,2 s) e, numa fatia, o continuo (N, s).
    float pushMinimumNewtons = 400.0f;
    float pushMaximumNewtons = 900.0f;
    float pushHoldFraction = 0.0f;
    float pushHoldMinimumNewtons = 150.0f;
    float pushHoldMaximumNewtons = 350.0f;
    float pushHoldMinimumSeconds = 1.0f;
    float pushHoldMaximumSeconds = 3.0f;
    // A locomocao (31/10 22h35, ele: so esta rede daqui em diante, com "os treinos de parado, tras, direita, esquerda,
    // diagonais frente e diagonais costas e o parado girando suave"; o autoequilibrio segue o principal: "se locomover
    // resistindo a impactos e adversidades"): os exercicios basicos do B21. A fatia dos rumos (andar ou trotar num dos 8
    // rumos em relacao ao rumo do corpo, mantendo-o) e a do giro parado suave.
    float locomotionFraction = 0.0f;
    float turnFraction = 0.0f;
    // As faixas do B21: andar (nesta fatia) e trotar (m/s; o minimo de 2 do trote, o pedido dele de 31/10 07h45). A
    // avaliacao pede o meio do trote.
    float walkFraction = 1.0f / 3.0f;
    float walkMinimumSpeed = 0.8f;
    float walkMaximumSpeed = 1.4f;
    float trotMinimumSpeed = 2.0f;
    float trotMaximumSpeed = 3.0f;
    // O rumo pedido (0 a frente, de 45 em 45 graus para a esquerda, ate 7; < 0: sorteado no treino e, na avaliacao, o
    // do mundo, de 8 em 8).
    int heading = -1;
    // Os empurroes em movimento (a regua do comeco do B21: curtos de 300-600 N por 0,1-0,2 s e, numa fatia, continuos de
    // 100-180 N por 1-2 s), nos rumos e no giro; na avaliacao, um so, de 2 a 3 s depois do comeco. Sem forca (as duas
    // faixas em 0), sem empurrao (31/10 22h45, ele: "por hora tira os empurroes - so vai entrar depois que ele estiver se
    // locomovendo bem").
    float movingPushMinimumNewtons = 300.0f;
    float movingPushMaximumNewtons = 600.0f;
    float movingPushHoldFraction = 0.3f;
    float movingPushHoldMinimumNewtons = 100.0f;
    float movingPushHoldMaximumNewtons = 180.0f;
    // (08/11, ele: "em TODOS os treinos atuais, as pancadas e empurroes... os empurroes continuos de 3-5 segundos e as pancadas
    // secas... a fase de fogo") A duracao dos continuos em movimento (s; antes 1-2) e a fatia dos episodios do treino com
    // empurroes (1: todos, como antes).
    float movingPushHoldMinimumSeconds = 1.0f;
    float movingPushHoldMaximumSeconds = 2.0f;
    float pushEpisodeShare = 1.0f;
    // (08/11) O ponto do empurrao sorteado (o tronco repartido, o peito alto, a pelve ou um ombro); sem, o tronco repartido.
    bool pushVariety = false;
    // (08/11 12h30) O fogo por grupo (no treino): as faixas (pancada min, max, continuo min, max; N) da frente, dos lados, das
    // costas, do giro, do parado e do resto; sem, as faixas unicas.
    bool fireGrouped = false;
    // (08/11 18h05) O laboratorio com a caixa "Empurroes": numa avaliacao, os empurroes como no treino (o grupo, o intervalo).
    bool fireShown = false;
    // (08/11 13h05) Nos retos de lado, a regua das pernas e dos pes vezes isto (1: a de todos).
    float lateralLegStrict = 1.0f;
    // (08/11 13h50) Durante o empurrao e ate 1,5 s depois, a regua da imitacao pela metade.
    bool pushImitationRelief = false;
    std::array<std::array<float, 4>, 6> fireGroupBands {};
    // (08/11) No levantar, o corpo inteiro no ar depois de tocar o chao (a cabeca ate 1,2 m): a multa por decisao.
    float riseFlightPenalty = 0.0f;
    // (08/11) Nos retos, andar no rumo abaixo desta fracao da velocidade de referencia por 1 s (depois de 2,5 s, fora do
    // empurrao) encerra como a queda (o fim 7, "lento"); 0: sem.
    float slowLimit = 0.0f;
    // (12h25) O piso por grupo: a frente (0, 1, 7), os lados (2, 6), as costas (3, 4, 5); slowLimit e o maior (> 0 liga).
    // (09/11 01h20) E por faixa: 0-2 o trote, 3-5 a corrida - o trote levava o piso da frente a 0,80 e metade das corridas
    // acabava por lento. Com 3 valores no setSlowLimits, os dois iguais (como antes).
    std::array<float, 6> slowLimits { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };
    // (08/11, ele: "6 exercicios novos, da estrela de oito pontas: trote peito livre, fixo e girando; corrida peito livre,
    // fixo e girando... garanta bem a boa estruturacao das fases de aprendizado... inicialmente vamos deslocar uma % dos
    // treinos a estrela") A fatia dos episodios de rumo na estrela; os pesos e as fases (1-4) dos 6, na ordem trote fixo,
    // livre, girando, corrida fixo, livre, girando; a duracao do episodio dela (s). Na avaliacao, evaluationManeuver 8 peito
    // fixo, 9 livre, 10 girando, 11 os tres em ciclo pelo mundo.
    float starFraction = 0.0f;
    std::array<float, 6> starWeights { 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f };
    std::array<int, 6> starPhases { 1, 1, 1, 1, 1, 1 };
    float starSeconds = 14.0f;
    // (08/11 14h40, ele: a rede preparada para os comandos do jogo) O comando - o jogador emulado (teclas, shift, mouse, paradas):
    // a fatia dos episodios de rumo, a fase (1-4) e a duracao do episodio (s). Na avaliacao, evaluationManeuver 12.
    float commandFraction = 0.0f;
    int commandPhase = 1;
    float commandSeconds = 20.0f;
    // (08/11 15h20, ele: "ao final da rodada a PARADA") A parada no fim dos retos: a fatia dos episodios retos do treino com ela,
    // a janela do instante (s), a freada da referencia (m/s^2) e se a avaliacao tambem para (no meio da janela).
    float stopFraction = 0.0f;
    float stopEarliest = 4.5f, stopLatest = 5.5f;
    float stopDeceleration = 5.0f;
    bool stopInEvaluation = false;
    // (08/11 16h20) A referencia da parada pelas paradas da Unreal (trote_run_stop_* e corrida_stop_*, 8 rumos, os dois pes):
    // o clipe da freada de verdade - o passo que planta e assenta - em vez do passo da corrida com a velocidade caindo.
    bool stopClips = false;
    // O premio do passo (01/11, ele: "nenhum passo ate agora"; o feet air time de Rudin et al. 2021, "Learning to Walk
    // in Minutes"): com o pedido de deslocamento, de pe, a cada pe que pousa, este peso vezes o tempo no ar (ate 0,5 s)
    // menos 0,15 s - passo de verdade ganha, arrastar perde. 0: desligado.
    float stepReward = 0.0f;
    // A rl_final (01/11, ele: "precisamos achar uma abordagem que consigamos fazer ela se locomover! e BEM, com
    // estabilidade e velocidade e excelente auto-equilibrio"; vault 04_locomocao/plano_rl_final_01_11.md). Nos rumos, as
    // faixas pelos clipes (GaitTable): a fatia da corrida e a do sprint (andar em walkFraction, o resto trota), a
    // velocidade pedida a do clipe do passo sorteado +-15%; a fatia dos comecos ja no clipe, num instante sorteado e na
    // velocidade dele (o RSI do DeepMimic e do AMP, o que ensina a correr); o tempo do episodio dos rumos e do giro (s; 0:
    // episodeSeconds); a queda encerrando esses episodios (o early termination do DeepMimic); e a troca do pedido no meio
    // deles a cada um tempo sorteado nesta faixa (s; 0 0: so a parada no meio de antes) - parar, mudar de faixa e de rumo,
    // virar correndo.
    float runFraction = 0.0f;
    float sprintFraction = 0.0f;
    float locomotionClipFraction = 0.5f;
    float locomotionSeconds = 0.0f;
    bool locomotionTermination = false;
    // Na avaliacao dos rumos, a faixa (-1: o meio do trote nos 8 rumos, como antes; 0 andar, 1 trote, 2 corrida, 3
    // sprint: para a frente, de pe no parado em alerta, na velocidade do primeiro clipe da faixa).
    int evaluationBand = -1;
    // A rampa da velocidade com pedido ate a pedida: (feita/pedida) elevado a isto (0,5, a raiz: um passo pequeno ja
    // vale; 1, linear: mais ganho de 2 para 4 m/s num pedido de 6 - a rl_final, 01/11 09h30, o plato das faixas altas).
    float speedRampPower = 0.5f;
    // A imitacao do clipe do passo na locomocao (01/11 14h50, ele: "o movimento dele no andar/trote e bem diferente do
    // clipe... so passinhos curtinhos e ele torto... uma solucao rapida que corrija isso, depois voltamos a soltar"): a
    // nota do DeepMimic (TrackingGetUpContract3D::reward) contra o quadro do clipe do passo pedido, no tempo dele desde o
    // comeco (em ciclo), alinhado a posicao e ao rumo do corpo, vezes este peso, somada a nota de estado. 0: desligada.
    float imitationWeight = 0.0f;
    // O imitador de um clipe so (01/11 18h, ele: "essa locomocao e INACEITAVEL"): nos rumos, so os passos deste clipe
    // (vazio: todos pelas fatias).
    std::string onlyClip;
    // O clipe nos motores (01/11 18h45, ele: "toda vez voce da a mesma desculpa para ela nao seguir nada do clipe"): na
    // locomocao, o alvo de cada junta e o do clipe do passo (no quadro do fim da decisao) vezes (1 - clipBlend) mais o da
    // rede vezes clipBlend - o clipe comanda as juntas e a rede so corrige. 1: a rede livre (como antes); soltar = subir.
    float clipBlend = 1.0f;
    // O rolar pelo clipe (02/11 01h40, ele: "nos ja tinhamos a principio o movimento do rolar-se no chao, porque nao esta
    // sendo usado para referencia?"): deitado de costas, o trecho do rolamento do levantar de costas (stand_up_back_kimodo,
    // 0 a 0,75 s: a barriga de cima para baixo) nos motores - o alvo = clipe x (1 - rollBlend) + rede x rollBlend -, sem
    // nota de imitacao (nada a trapacear); o resto do levantar com a rede livre. 1: desligado; o desmame sobe ate 1.
    float rollBlend = 1.0f;
    // A queda programada (01/11 22h50, ele: "quando ele de fato cai, eu preferia aquele nosso ragdoll que temos na
    // locomocao fisica... tenta cobrir a cabeca e se encolher... fica meio mole... e ai o ato de levantar ocorresse apos
    // isso"): fora da locomocao, a queda sem volta de quem ja ficou de pe (e os comecos ja caindo) e a da locomocao
    // fisica (ProtectiveFall3D) ate assentar, sem a rede - os passos dela dentro de uma decisao -; dai a rede levanta.
    bool protectiveFall = false;
    // No laboratorio, a queda programada passo a passo (uma decisao por vez, a acao da rede ignorada ate assentar): no
    // treino ela roda inteira dentro de uma decisao, e na tela o corpo pulava de caindo para deitado.
    bool protectiveFallStepwise = false;
    // O levantar guiado pelo clipe (01/11 23h10, ele: "nossa meta vai ser 1.5 - 1.8 segundos para ficar de pe"; "os
    // movimentos dele estao MUITO zuados... devem se aproximar mais dos clipes, e ele hoje ta ficando em pe TODO TORTO ao
    // inves de aproximar-se do IDLE ALERTA apos levantar-se"): deitado, com a rede no comando, o clipe de levantar mais
    // parecido com a pose de agora (de brucos ou de costas, normal ou espelhado, no instante mais proximo) e seguido -
    // nos motores com a mistura clipBlend e na nota com a imitacao imitationWeight -, acelerado para o clipe inteiro durar
    // getUpSeconds; no fim dele, de pe, o parado em alerta como no parado (a imitacao, a quietude e o ponto).
    bool getUpGuide = false;
    float getUpSeconds = 1.7f;
    // (05/11 04h40) O levantar pelo clipe - a receita da locomocao (03-04/11): com o levantar guiado, o clipe de levantar
    // inteiro nos motores e a cabeca da locomocao corrigindo por cima (a correcao residual), a nota do rastreio e a mao de
    // fora na pelve e no corpo na forca do exercicio GetUpDrill, descendo pelo dominio; no fim do clipe, de pe, o idle.
    // (O guiado de 01/11 so misturava 30% do clipe e nao tinha mao: ele aprendeu a ficar deitado imitando o comeco.)
    bool getUpTracking = false;
    // A troca de rede ao ficar de pe (01/11, ele: o levantar volta a ser so levantar e evitar quedas, "fazendo a troca
    // de RL no momento que recem fica de pe"): o episodio acaba (truncado, como o fim do tempo) quando ele fica de pe e
    // firme por 0,5 s - o mesmo da entrega no jogo - depois de ter estado no chao ou, no evitar a queda, do empurrao.
    bool handoffOnStand = false;
    // As quedas da locomocao (01/11, a interligacao das redes: "para que justamente um consiga aprender com o outro"): a
    // fatia que comeca no instante em que a rede da locomocao caiu no treino dela (setFallStarts; sem elas, uma queda).
    float locomotionFallFraction = 0.0f;
    // A rede da locomocao como parceira (01/11, ele: "ambas as redes aprendendo em conjunto em todos os treinos";
    // setLocomotionPartner): depois da troca ela dirige parada por partnerSeconds - se ela cair nesse tempo, a troca foi
    // ruim: partnerFallPenalty e o fim do episodio (so no treino) -; e, com partnerAvoid, o evitar a queda comeca com ela
    // de pe levando o empurrao forte, a rede do levantar assumindo na queda iminente dela (no treino e na avaliacao).
    float partnerSeconds = 2.0f;
    float partnerFallPenalty = 10.0f;
    bool partnerAvoid = false;
    // Nos comecos da queda iminente (o evitar a queda com a parceira e as quedas da locomocao), a punicao da primeira vez
    // que o tronco, o pescoco ou a cabeca tocam o chao (o evitar a queda de verdade; 0: so a do potencial).
    float touchDownPenalty = 0.0f;
    // A protecao na queda (01/11, ele: "quando ele via cair no chao tentasse se proteger e evitar de cair, levando as
    // maos, joelhos, girando quadril, coluna... para conseguir se recobrar antes de cair"): nesses comecos, ate o tronco
    // tocar o chao ou ficar de pe, o premio do primeiro apoio das maos, dos bracos ou dos joelhos no chao (uma vez).
    float supportBonus = 0.0f;
    // A flexao na queda (01/11, ele: "assim que ele cai, ele puxa os bracos e pernas dele para tras... o correto seria
    // ao contrario, para frente"): na protecao e por FlexAfterTouchSeconds depois do primeiro toque do tronco, este peso
    // vezes a flexao do passo - as maos a frente do peito e os joelhos a frente da pelve (ate 0,25 m: +1) menos a coluna
    // arqueada para tras (30 graus: -1).
    float flexReward = 0.0f;
    // O cenario sujo (01/11, ele: "nossa rede tem dificuldade de levantar-se se o cenario nao e limpo - tem algo
    // esbarrando nele, ou um boneco por cima"): no treino, a fatia dos episodios com uma parede ou caixa fixa encostada
    // no corpo (a 4-44 cm dele, de 0,4 a 1,5 m de altura, num lado sorteado) e a com um peso comprido solto caindo de
    // atravessado em cima dele (20-60 kg, como um boneco por cima).
    float obstacleFraction = 0.0f;
    float weightFraction = 0.0f;
    // Os ambientes (02/11, ele: "parede ao lado, paredes aos lados, pilares, corpos ragdoll embaixo dele (1..2 ou mais) e
    // tambem em cima dele, em terrenos inclinados" e acidentados), so no treino, cada um sorteado por si: as paredes dos
    // dois lados (o corredor, 4-44 cm do corpo, 1-2,5 m de altura), 1-4 pilares em volta, a rampa (6-18 graus) e o chao
    // acidentado (calombos de 2-8 cm) embaixo da queda, 1-3 corpos soltos embaixo da queda e um corpo solto caindo em
    // cima dele ja no chao.
    float corridorFraction = 0.0f;
    float pillarFraction = 0.0f;
    float slopeFraction = 0.0f;
    float roughFraction = 0.0f;
    float bodiesUnderFraction = 0.0f;
    float bodyOnTopFraction = 0.0f;
    // A variante ambientada do evitar a queda (02/11, ele: "com obstaculos, paredes, corpos, pilares, rampas e tudo mais,
    // bem diversificado, para ele aprender a evitar quedas em ambientes diversos"): a fatia dos episodios do evitar com
    // um ambiente garantido (parede, corredor, pilares, rampa suave, chao acidentado em volta ou corpos caidos em volta)
    // mais os outros nas fatias de cima; o chao entra antes de ele ficar de pe.
    float avoidSceneryFraction = 0.0f;
    // Os ambientes tambem na avaliacao (o laboratorio, o exercicio "levantar_ambientes"); o treino nunca liga. Cada
    // episodio dela tem um dos ambientes garantido (sorteado), mais os outros nas fatias.
    bool evaluationScenery = false;
    // Na avaliacao, o que se mede: 0 as quedas (o levantar), 1 o levantar em movimento, 2 o evitar a queda, 3 o
    // equilibrio parado (de pe, um empurrao com a regua do parado com empurrao forte do B21: o curto de 900-1200 N ou,
    // em 30%, o continuo de 205-275 N por 1-3 s), 4 os rumos (de pe, no meio do trote, mantendo o rumo do corpo, um
    // empurrao em movimento), 5 o parado (de pe, sem empurrao), 6 o giro parado suave (0,6 rad/s, metade para cada lado,
    // um empurrao em movimento), 7 as quedas da locomocao (a do mundo entre as de setFallStarts), 8 o equilibrio parado
    // com empurroes, 9 e 10 o levantar deitado de costas e de brucos (com setLyingStarts).
    std::uint32_t evaluationKind = 0;
    // A professora da locomocao (03/11 18h27, ele: "gravar os movimentos que o b21 faz para se locomover nas 8
    // direcoes... FORCAR absolutamente o nosso rl_final a fazer EXATOS mesmos movimentos, e aprender com eles, e ai apos
    // ele dominar isso nos vamos afrouxando"; setLocomotionTeacher): nos episodios dos rumos, a fatia teacherShare com
    // ela no volante do comeco ao fim (o movimento EXATO dela; o resto, a aluna dirige com ela em sombra - o rotulo - e
    // retomando no desvio). Com ela os rumos saem de pe, num rumo sorteado de 0 a 7, sem paradas, trocas nem rampa, no
    // pedido da tabela: teacherSpeeds o pedido a ela (m/s, por rumo) e teacherStudentSpeeds o da aluna (o que ela faz).
    float teacherShare = 1.0f;
    // Os rumos como no treino com a professora, mesmo sem ela (o laboratorio e a prova da aluna sozinha): de pe, no
    // pedido teacherStudentSpeeds, sem rampa, fase, paradas nem trocas. (03/11 20h30, ele: o giro parado e o parado tambem
    // - "ele precisa aprender melhor a ficar na pose IDLE ALERTA quando parado": com isto ou com a professora, o giro e o
    // parado sao da cabeca da locomocao, com o musculo dela, e a professora ensina os dois.)
    bool taughtHeadings = false;
    // O parado tambem ensinado (da cabeca da locomocao). 03/11 22h35, ele: "ate o parado esta caindo... sendo que ele JA
    // SABE se equilibrar parado": o parado ensinado tirava o parado da cabeca que sabe (a recuperacao, com o musculo dela).
    // Falso: o parado e da recuperacao, como antes.
    bool taughtStanding = false;
    // No giro parado com a professora, a media do peito que dispara a retomada (graus; ela mesma atrasa girando).
    float teacherTakeoverTurnChest = 40.0f;
    // O parado ensinado com o proprio IDLE ALERTA (03/11 20h50: a B21 parada mede pose 0,64 e coluna 0,27 contra ele - a
    // rl_final de antes, ~0,67 e ~0,69 -; e o clipe sozinho nos motores, sem equilibrio, desaba em ~3 s): a professora
    // hibrida - as pernas da B21 (o equilibrio) e o tronco, os bracos e a cabeca na pose do clipe (o musculo do tronco
    // dela e o da aluna). Falso: a B21 inteira tambem no parado.
    // 0: a B21 inteira; 1: o tronco, os bracos e a cabeca no clipe; 2: todas as juntas no clipe, com as antecipacoes de
    // contato dela nas pernas (o equilibrio dela).
    int teacherStillClip = 2;
    // Nos episodios da aluna, a professora dirige o comeco (s, sorteado; 0: nao) e entrega no embalo dela.
    float teacherLeadMinimum = 0.0f;
    float teacherLeadMaximum = 0.0f;
    // A professora em sombra depois de entregar (o rotulo em sombra e a retomada). Falso e sem retomada: ela para de rodar
    // quando entrega - o rotulo em sombra nao e usado com a imitacao so no volante (22h25: as decisoes por segundo).
    bool teacherShadow = true;
    std::array<float, 8> teacherSpeeds { 3.0f, 3.0f, 1.5f, 1.5f, 1.5f, 1.5f, 1.5f, 3.0f };
    std::array<float, 8> teacherStudentSpeeds { 1.9f, 1.45f, 1.05f, 1.2f, 1.25f, 1.2f, 1.1f, 1.45f };
    // A intervencao (a aluna no volante; o revisor de 03/11: na queda iminente de 25 graus a propria B21 ja cai): o
    // tronco alem de teacherTakeoverTilt (graus) ou tombando mais rapido que teacherTakeoverTiltRate (graus/s), o peito
    // alem de teacherTakeoverChest da vista (graus, media de ~0,25 s), ou o deslocamento alem de teacherTakeoverTravel do
    // rumo (graus, acima de 0,5 m/s): ela assume por teacherTakeoverSeconds (0: sem intervencao).
    float teacherTakeoverTilt = 15.0f;
    float teacherTakeoverTiltRate = 45.0f;
    float teacherTakeoverChest = 25.0f;
    float teacherTakeoverTravel = 20.0f;
    float teacherTakeoverSeconds = 1.0f;
    // shortcut: sem efeito desde a limpeza de 10/11 (a locomocao e sempre o rastreio); so o Workbench
    // (TrackingViewer3D.cpp) ainda o escreve. Sair junto com essa linha dele.
    bool locomotionTracking = true;
};

// Um episodio do levantar livre que acabou (no fim do tempo).
struct TrackingGetUpAmpEpisode3D {
    std::uint32_t world = 0;
    // 0: uma queda; 1: um instante de um clipe de levantar; 2: de pe (o parado); 3: uma queda com alvo (o levantar em
    // movimento); 4: de pe com o empurrao forte (o evitar a queda); 5: a arrancada agachada com alvo; 6: um rumo (o rumo
    // pedido, 0 a 7, vem em startSide); 7: o giro parado suave; 8: uma queda da locomocao.
    std::uint32_t startKind = 0;
    // Como comecou: 0 de brucos, 1 de costas, 2 de lado (ou sentado, ajoelhado).
    std::uint32_t startSide = 0;
    float startHeadHeight = 0.0f;
    float seconds = 0.0f;
    // A primeira vez de pe (s desde o comeco; < 0: nao ficou).
    float firstStandSeconds = -1.0f;
    float meanTaskReward = 0.0f;
    float headHeight = 0.0f;
    float pelvisHeight = 0.0f;
    bool standing = false;
    // O levantar v2: a velocidade pedida no comeco (0: ficar), quando chegou ao alvo (s; < 0: nao chegou), a
    // velocidade media rumo ao alvo enquanto de pe com o pedido (m/s), se caiu depois de ja ter ficado de pe e, no
    // evitar a queda, quando se recompos (s desde o fim do empurrao; < 0: nao se recompos).
    float commandSpeed = 0.0f;
    float reachedSeconds = -1.0f;
    float meanTowardSpeed = 0.0f;
    bool fellAfterStanding = false;
    float recomposedSeconds = -1.0f;
    // A semelhanca com a pose do parado em alerta no fim (0 se nao esta de pe).
    float endPoseMatch = 0.0f;
    // Nos rumos e no giro: o erro medio do rumo do corpo (graus), de pe.
    float meanFaceError = 0.0f;
    // A coluna como a do parado em alerta no fim (TrackingGetUpPostureProbe3D::idleSpineMatch; 0 se nao esta de pe).
    float endSpineMatch = 0.0f;
    // A troca de rede (s desde o comeco; < 0: o episodio acabou sem ela).
    float handoffSeconds = -1.0f;
    // Depois da troca, se a rede da locomocao caiu no tempo dela e quanto ela dirigiu (s; < 0: sem a parceira).
    bool partnerFell = false;
    float partnerSeconds = -1.0f;
    // Se o tronco, o pescoco ou a cabeca tocaram o chao em algum momento (o evitar a queda: nao) e se, protegendo-se,
    // ele se apoiou nas maos, nos bracos ou nos joelhos antes.
    bool touchedDown = false;
    bool supported = false;
    // A flexao media na queda (a de flexReward, sem o peso; 0 sem a queda iminente).
    float meanFlex = 0.0f;
    // Se o episodio teve a parede ou o peso (o cenario sujo).
    bool obstructed = false;
    // Nos rumos, a faixa do passo do comeco (0 andar, 1 trote, 2 corrida, 3 sprint; a rl_final).
    std::uint32_t band = 0;
    int sceneryPreset = -1;
    // A professora (setLocomotionTeacher): a fracao das decisoes com ela no volante, quantas vezes ela retomou, as
    // decisoes rotuladas e a maior ajuda de fora dela (N; tem de ser 0). Nos rumos, o desvio do deslocamento medio contra
    // o rumo pedido (graus, + a esquerda).
    float teacherShare = 0.0f;
    std::uint32_t takeovers = 0;
    std::uint32_t labeled = 0;
    float teacherAssist = 0.0f;
    float travelError = 0.0f;
    // (04/11) Por que acabou: 0 o tempo, 1 o tronco no chao, 2 a perda do equilibrio sem tocar o chao (cabeca abaixo de
    // 1,1 m ou pelve abaixo de 0,5 m), 3 o corpo virado alem do rumo do clipe, 4 o rastreio ruim, 5 outro (parceiro).
    std::uint32_t endCause = 0;
    // (04/11) As medias da semelhanca com o clipe e da estabilidade da nota do rastreio no episodio (0-1; -1 sem rastreio).
    float meanImitation = -1.0f, meanStability = -1.0f;
    // (04/11 15h, os passos) Os passos de recuperacao e de estabilizacao do gerador, se foi o equilibrio parado (com
    // empurroes), quantos empurroes, o pior ponto de captura fora da base de pe no idle (m) e quando ficou quieto de novo
    // depois do ultimo empurrao (s; < 0 nao ficou).
    std::uint32_t recoverySteps = 0, settleSteps = 0;
    bool balanceDrill = false;
    std::uint32_t pushes = 0;
    float worstCaptureOut = 0.0f;
    float quietAgainSeconds = -1.0f;
    // A qualidade da chegada na entrega ao idle depois de levantar (0-1; < 0 sem entrega).
    float arrivalQuality = -1.0f;
    // (05/11 23h) Na corrida para a frente e nas diagonais da frente, a fracao dos apoios com o pe apoiado mais de 3 cm para
    // dentro que o mesmo pe do clipe - o passo cruzado (< 0: sem apoios medidos).
    float footCross = -1.0f;
    // (08/11) Na estrela, as pernas feitas e a fase; e se o episodio acabou caindo durante um empurrao ou ate 2 s depois.
    std::uint32_t starLegs = 0;
    int starPhase = 0;
    bool fellInPush = false;
    // (08/11) A distancia da pelve ao ponto de parada de referencia no fim (m; < 0 sem parada).
    float stopError = -1.0f;
};

// O que o visualizador do laboratorio desenha de um mundo: como comecou, o alvo (o levantar em movimento, enquanto o
// pedido dura) e o empurrao de agora.
struct TrackingGetUpAmpWorldView3D {
    std::uint32_t startKind = 0;
    bool hasTarget = false;
    Vec3 target;
    float commandSpeed = 0.0f;
    // O pedido de deslocamento (m/s, no chao; zero sem pedido) e, nos rumos e no giro, o rumo pedido do corpo (rad).
    Vec3 moveVelocity;
    bool hasFace = false;
    float faceYaw = 0.0f;
    bool pushing = false;
    Vec3 pushForce;
    // (04/11 11h50, ele: "corrigir o problema de nao aparecer as marcacoes no cenario das curvas") O caminho pedido a frente
    // nas curvas (pontos no chao a cada ~1 m: o arco que falta e a reta ate o proximo, que e sorteado).
    std::vector<Vec3> path;
    // (08/11) A estrela: o centro e as 8 pontas (o centro primeiro), o ponto de agora (-1 o centro) e o seguinte, a perna, a
    // fase, o peito (8 fixo, 9 livre, 10 girando), o angulo da proxima troca (graus) e a velocidade de chegada pedida (m/s).
    bool star = false;
    std::vector<Vec3> starPoints;
    int starFrom = -1, starGoal = -1, starNext = -1, starLegs = 0, starPhase = 0, starChest = 0;
    float starTurnDegrees = 0.0f, starArrive = 0.0f;
    // (08/11 14h40) O comando de agora (o jogador emulado), em texto: as teclas, o shift, o estado e a vista.
    std::string commandText;
};

// O lote do levantar livre (vault 04_locomocao/plano_compostos_31_10.md, "O levantar livre"): o ragdoll do jogo a 0%
// comecando de quedas sorteadas (uma pose de um clipe, o corpo inteiro num sentido sorteado, solto com os musculos
// fracos ate assentar no chao) e, no treino, tambem de instantes dos clipes de levantar. A acao e a do levantar do
// DeepMimic (o alvo dos 39 eixos); a observacao, so o corpo; a nota daqui e a da tarefa (TrackingGetUpPostureProbe3D),
// e a do estilo vem do discriminador do treinador, com os pares do AMP de cada passo e os dos clipes.
class TrackingGetUpAmpTraining3D final {
public:
    static constexpr int TicksPerDecision = 4;
    static constexpr float TickSeconds = 1.0f / 120.0f;
    // Os clipes do estilo: os de levantar (e os espelhados) tambem dao os comecos; o parado so o estilo.
    static constexpr const char* GetUpClips[4] { "stand_up_front_kimodo", "stand_up_back_kimodo",
        "stand_up_front_kimodo_espelho", "stand_up_back_kimodo_espelho" };
    // (31/10 18h, ele: o padrao de ficar de pe parado e o parado em alerta, nao o natural; com o espelho, o apoio no
    // outro pe.) (07/11, ele: "o pose de pe passa a ser a parada da Unreal": a base parada do Game Animation Sample, em
    // ciclo e no lugar, e o espelho dela.)
    // (08/11 18h05, ele: "vamos trocar a pose do parado IDLE para o nosso antigo IDLE ALERTA... ao inves do da Unreal, pois eu
    // sentia que aquela pose tinha uma base mais forte".)
    static constexpr const char* StandingClip = "alert_idle";
    static constexpr const char* StandingMirrorClip = "alert_idle_espelho";
    // Os clipes do estilo do levantar em movimento (a arrancada e a corrida; no lugar, com a velocidade nominal somada
    // aos pares do discriminador): so o estilo e os comecos das quedas.
    static constexpr const char* MovingClips[5] { "idle_to_sprint", "crouched_to_sprint", "mixamo_running",
        "run_forward_medium", "sprint_forward" };
    // Os clipes do trote, os do modo padrao do jogo (character.json; 03/11: os trot_* do trot.py, rejeitados em 05/10,
    // sairam): a frente, a esquerda, de costas e a direita. As diagonais usam o da frente ou o de costas com o corpo
    // virado para o rumo, como o modo padrao.
    static constexpr const char* HeadingClips[4] { "jog", "jog_strafe_left", "jog_backward", "jog_strafe_right" };
    // (01/11 19h, ele: "por que temos um exercicio de andar, sendo que nao usamos andar no jogo? apenas trote, corrida e
    // sprint": os clipes de andar sairam do estilo e dos passos; ficam os giros parados.)
    static constexpr const char* LocomotionStyleClips[2] { "turn_left", "turn_right" };
    // (04/11 09h50) O giro parado do rastreio: o do modo padrao, do idle alerta (os passos do StandTurnFeet3D).
    static constexpr const char* TrackedTurnClips[2] { "giro_idle_esquerda", "giro_idle_direita" };
    // A rl_final: as quedas com protecao (01/11; ele: "quando ele via cair no chao tentasse se proteger... levando as
    // maos, joelhos, girando quadril, coluna"; o Kimodo e os tropecos da Mixamo, com os espelhos) - o estilo e, com o
    // evitar a queda ligado, metade dos comecos no clipe, num instante da queda, com a protecao valendo (como no evitar).
    static constexpr const char* ProtectionClips[16] { "queda_protecao_frente", "queda_protecao_frente_espelho",
        "queda_protecao_tras", "queda_protecao_tras_espelho", "queda_protecao_lado", "queda_protecao_lado_espelho",
        "queda_protecao_tropeco", "queda_protecao_tropeco_espelho", "queda_protecao_tropeco_corrida",
        "queda_protecao_tropeco_corrida_espelho", "queda_protecao_tropeco_queda", "queda_protecao_tropeco_queda_espelho",
        "queda_protecao_empurrao_lado", "queda_protecao_empurrao_lado_espelho", "queda_protecao_empurrao_tras",
        "queda_protecao_empurrao_tras_espelho" };
    // A rl_final: os clipes da corrida e do sprint (o estilo e os comecos dessas faixas).
    static constexpr const char* RunClips[5] { "jog", "jog_backward", "run_forward_medium", "mixamo_running",
        "run_backward" };
    // (05/11 03h15, ele: a corrida de costas "mais semelhante com o trote costas... o clipe atual deixa a base muito
    // baixa": run_backward_trote no lugar do sprint_backward.)
    // (04/11, ele: "adicionar os treinos de CORRIDA nas 8 direcoes" - no rastreio, a faixa da corrida (~4,1-4,4 m/s): a
    // frente, a esquerda, de costas e a direita; as diagonais pela cintura, como no trote. So rastreados: fora do juiz
    // de estilo da recuperacao.)
    static constexpr const char* TrackedRunClips[4] { "run_forward_medium", "sprint_strafe_left", "run_backward_trote",
        "sprint_strafe_right" };
    // (08/11, a velocidade: a rede corria na velocidade do clipe dos motores - o trote de frente ~3,5 m/s com o jog de 3,0; a
    // corrida ~4,3 com a run_forward_medium de 4,33 - contra metas de 4,5 e 6,0) Os ciclos nas velocidades-meta
    // (tools/animation/clips/ciclos_meta.py, os nossos regerados na meta): trote frente, esquerda, costas, direita; corrida
    // frente, esquerda, costas, direita. Com setGaitCycles, no lugar dos passos do trote e da corrida.
    static constexpr const char* CycleClips[8] { "ciclo_trote_frente", "ciclo_trote_esquerda", "ciclo_trote_costas",
        "ciclo_trote_direita", "ciclo_corrida_frente", "ciclo_corrida_esquerda", "ciclo_corrida_costas", "ciclo_corrida_direita" };
    static constexpr const char* SprintClips[8] { "sprint_forward", "mixamo_sprint", "sprint", "sprint_maximo",
        "sprint_backward", "sprint_backward_long", "sprint_strafe_left", "sprint_strafe_right" };
    // As entradas do comando, depois das do corpo: o rumo do deslocamento pedido em relacao ao corpo (cos, sen), a
    // distancia (/10 m, ate 1; nos rumos, 1), a velocidade pedida (/5 m/s; 0: ficar onde esta) e o rumo pedido do corpo
    // em relacao a ele (cos, sen: no levantar em movimento, o alvo; nos rumos, o do comeco; no giro, o que gira).
    static constexpr std::size_t CommandSize = 6;
    // Depois do comando, o corpo contra o chao (o contrato v3, TrackingGetUpContract3D::observeGround).
    static constexpr std::size_t GroundObservationSize = TrackingGetUpContract3D::GroundObservationSize;
    // Depois do chao, a fase do clipe do passo (sen, cos de 2 pi t/T; zeros sem clipe): a entrada de fase do DeepMimic,
    // sem a qual a imitacao no tempo do clipe nao tem como sincronizar (01/11 18h, o layout levantar-v4).
    static constexpr std::size_t PhaseSize = 2;
    static constexpr std::size_t SkillModeSize = 1;
    // Depois da fase, o ambiente (02/11, o layout levantar-v5): a altura de cada parte ao que esta embaixo dela, a
    // inclinacao do chao e os raios horizontais no quadril e no peito (TrackingGetUpContract3D::observeEnvironment).
    static constexpr std::size_t EnvironmentSize = TrackingGetUpContract3D::EnvironmentObservationSize;

    explicit TrackingGetUpAmpTraining3D(const TrackingGetUpAmpSettings3D& settings);
    ~TrackingGetUpAmpTraining3D();
    TrackingGetUpAmpTraining3D(const TrackingGetUpAmpTraining3D&) = delete;
    TrackingGetUpAmpTraining3D& operator=(const TrackingGetUpAmpTraining3D&) = delete;

    [[nodiscard]] std::size_t environmentCount() const;
    [[nodiscard]] std::size_t observationSize() const;
    [[nodiscard]] std::size_t actionSize() const;
    [[nodiscard]] std::size_t ampSize() const;
    [[nodiscard]] const std::vector<std::string>& observationNames() const;
    [[nodiscard]] const std::vector<std::string>& actionNames() const;
    [[nodiscard]] float episodeSeconds() const;
    // O minimo e o maximo (rad) de cada eixo da acao (2 por eixo).
    [[nodiscard]] std::vector<float> actionRanges() const;

    // As quedas da locomocao para os comecos do tipo 8: o corpo inteiro no instante da queda, com as velocidades (os de
    // outro numero de links ficam de fora). Troca so entre os steps.
    void setFallStarts(std::vector<std::vector<PhysicsBodyState3D>> states);
    // Quais mundos estao protegendo-se na queda agora (os comecos da queda iminente, ate o tronco tocar o chao ou ficar de
    // pe; 1 ou 0): o treinador alivia ai os custos de suavidade - o reflexo de protecao e rapido.
    void protecting(std::span<std::uint8_t> out) const;
    // Quais mundos estao num episodio de locomocao (os rumos e o giro; 1 ou 0): o treinador pesa ai a tarefa, o estilo e
    // a suavidade da locomocao (a rl_final).
    void locomoting(std::span<std::uint8_t> out) const;
    // O rastreio da ultima decisao de cada mundo (8 por mundo; zeros fora da locomocao): a nota, os erros da pelve, das
    // pernas (pior), do tronco, dos bracos, dos pes (pior), da velocidade e a velocidade no rumo pedido (m/s).
    // (04/11 16h30, mais 9: os pilares da firmeza, a margem de Hof (m), o momento angular alem do clipe, a firmeza, o
    // escorregao no ponto de contato (m/s) e o tropeco.)
    // (08/11 23h55, mais 1: o portao da velocidade daquela nota - 0 quando a nota andando nao foi calculada.)
    // (09/11 01h20, mais 1: o joelho para dentro alem do clipe, m - 0 quando nao medido.)
    static constexpr std::size_t TrackingFeedbackSize = 35;
    void locomotionFeedback(std::span<float> out) const;
    // O alvo do clipe do passo (a acao que reproduz o quadro do fim da decisao; zeros sem clipe): o teste de mergulho.
    void locomotionTargets(std::span<float> out) const;
    // (05/11, a perda de simetria) O espelho de cada mundo, 2 por mundo: se o estado de agora tem espelho (1/0: a locomocao
    // dirigindo um rumo reto sem manobra, num clipe periodico com espelho, fora da arrancada e das transicoes) e o
    // deslocamento da fase do clipe no espelho (pi nos clipes que servem aos dois lados; 0 nos laterais, cujo par e o
    // espelho exato no mesmo instante). Ler junto com as observacoes (antes do step).
    static constexpr std::size_t MirrorInfoSize = 3;
    void mirrorInfo(std::span<float> out) const;
    // A rede da locomocao parceira (vazia: sem). Troca so entre os steps.
    void setLocomotionPartner(std::shared_ptr<const LocomotionPartner3D> partner);
    // A professora da locomocao (begin/tick; nula: sem ela) e a fatia dos rumos com ela no volante (o beta, mudavel no
    // treino).
    void setLocomotionTeacher(std::shared_ptr<const LocomotionPartner3D> teacher);
    void setTeacherShare(float share);
    // A forca auxiliar da locomocao (TrackingGetUpAmpSettings3D::locomotionAssist), mudada pelo treinador.
    void setLocomotionAssist(float assist);
    void setBalanceFeet(bool on);
    void setTrackingReward(int version);
    void setPhaseSync(bool on);
    void setImitationLoosen(float factor);
    void setFacingGate(float gate);
    void setArmClipReach(float reach);
    void setLinearHand(bool on);
    void setUpperHand(bool on);
    void setBodyHand(bool on);
    void setIdle(bool afterStand, bool onLocomotion, float residualScale = 1.0f);
    void setIdleImitation(float weight);
    void setIdleHandoff(float blendSeconds, float blendReach);
    void setRiseHandoff(float headHeight);
    void setRecoveryNatural(float weight);
    void setLateralHandPush(bool push);
    void setLateralRunLegReach(float scale);
    void setGaitSpeeds(std::array<float, 8> trot, std::array<float, 8> run);
    void setBackStartLean(float radians);
    void setCourseLimit(float meters);
    void setCurveLean(bool lean);
    void setFootWidth(float slackMeters);
    void setRunTurnBrake(bool brake);
    void setRunTurnAlign(bool align);
    void setTurnSettle(bool settle);
    void setTurnLaunch(bool launch);
    void setRunLaunchEase(float scale, float easeSeconds, float acceleration);
    void setCourseTail(float weight);
    void setSpeedRange(float low);
    void setIdleQuiet(float weight);
    void setManeuverAcceleration(float acceleration);
    void setRiseOvershoot(float weight);
    void setRollTail(float weight);
    void setCliffFree(bool on);
    void setCliffParts(bool curveRoll, bool footCross, bool forearm, bool hop);
    void setLaunchRise(float spanMeters);
    void setLateralStart(float acceleration);
    void setRiseStiffness(float scale);
    // (06/11) A velocidade-alvo dos motores pelo proprio alvo da cabeca da locomocao, sem clipe (freeVelocityFeedforward).
    void setFreeVelocityFeedforward(bool on);
    void setFreeDampingScale(float scale);
    // (06/11 17h40) A velocidade-alvo na acao (velocityActions); muda o tamanho e os nomes da acao - chamar antes de usa-los.
    void setVelocityActions(bool on);
    void setRecoveryVelocity(bool on);
    void setManeuverWeights(std::array<float, 7> weights);
    static constexpr float VelocityActionLimit = 12.0f;
    void setTurnSettleLateral(float speed);
    void setTurnSettleFront(float speed);
    void setRunTurnTimeScale(float scale);
    void setStopGaitAll(bool on);
    void setIdlePoseGate(float gate);
    void setStopClipsFront(bool on);
    void setStopClipsCommand(bool on);
    void setStopGateFloor(float floor);
    void setFreeStop(bool on);
    void setStopPosture(float gate);
    void setStopIdlePose(float weight);
    void setUpperRigor(float factor);
    void setUnrealStarts(bool on);
    void setUnrealStartScale(float scale);
    void setLiveLaunch(bool on);
    void setLaunchStepReward(float weight);
    void setLaunchClipShare(float share);
    void setLaunchHand(float level);
    void setLaunchInPlacePenalty(float weight);
    void setLaunchCalmArms(bool on);
    void setSpineVelocityFeedforward(bool on);
    void setBasePosture(float weight);
    void setMotorTone(float tone);
    void setPostureTail(float share);
    void setSpineTone(float tone);
    void setJamCost(float weight);
    void setFootSlipCost(float weight);
    void setMotorGravity(bool on);
    void setStopClipsBase(bool on);
    void setStopFrontVariants(bool on, int variant);
    void setStopFrontVariantMask(int mask);
    void setStopEntry(float blendSeconds, float windowSeconds);
    void setFreeTroca(bool on);
    void setFreeStandGate(float gate);
    // shortcut: sem efeito desde a limpeza de 10/11; so o Workbench (TrackingViewer3D.cpp) ainda chama. Sair junto com ele.
    void setCommandChestGate(float) {}
    void setFreeIdle(bool on);
    void setPoseBoost(float gate);
    void setFreePush(bool on);
    void setFreeIdleMatch(float weight);
    void setFreeStepHeight(float meters);
    void setIdleBentEnd(bool on);
    void setFinalRow(bool on);
    void setIdleLikeStop(bool on);
    // shortcut: sem efeito desde a limpeza de 10/11; so o Workbench (TrackingViewer3D.cpp) ainda chama. Sair junto com ele.
    void setUpperImitation(bool) {}
    void setIdleWide(bool on);
    void setMirrorCycles(bool on);
    void setTurnSettleBack(float speed);
    void setTrocaFixedChest(bool on);
    void setTrocaPractice(bool practice, std::array<float, 8> headingWeights);
    void setFootCross(float slackMeters);
    void setIdleReanchor(bool reanchor);
    void setAccelLean(float radians);
    void setFrontLean(float radians, float startRadians);
    void setRunLean(float radians, float legs);
    void setRunBackLean(float radians, float legs);
    void setGetUpTracking(bool on);
    void setSprint(float speed, float leanRadians, float legs);
    // (04/11 15h) O gerador de passos no idle e o idle seguindo nos empurroes; o equilibrio parado (a fatia e o nivel).
    void setBalanceSteps(bool steps, bool throughPushes);
    void setBalanceDrill(float fraction, float pushLevel);
    // (08/11, a fase de fogo) Os empurroes em todos os exercicios, de pe e em movimento: a fatia dos episodios do treino com
    // eles, as pancadas (N, 0,1-0,2 s), os continuos (N), a duracao deles (s) e a fatia dos continuos.
    void setFire(float episodeShare, std::array<float, 2> hitNewtons, std::array<float, 2> holdNewtons,
        std::array<float, 2> holdSeconds, float holdFraction);
    // (08/11) A estrela de 8 pontas: a fatia dos episodios de rumo, os pesos e as fases dos 6 e a duracao do episodio (s).
    void setStar(float fraction, std::array<float, 6> weights, std::array<int, 6> phases, float seconds);
    // (08/11) O comando: a fatia dos episodios de rumo, a fase (1-4) e a duracao (s).
    void setCommand(float fraction, int phase, float seconds);
    // (08/11) A parada no fim dos retos: a fatia, a janela (s), a freada (m/s^2) e se vale na avaliacao.
    void setFinalStop(float fraction, float earliest, float latest, float deceleration, bool inEvaluation);
    void setStopClips(bool on);
    void setFireShown(bool on);
    void setDiagonalChest(float gate);
    void setCourseSmoothing(float seconds);
    void setCourseAngleLimit(float radians);
    void setLocomotionShaping(bool on);
    void setSpeedGate(float floor);
    void setKneeInward(float slackMeters);
    void setIdleHeight(float slackMeters);
    void setStopWaitBody(bool on);
    // (08/11) Os passos do trote e da corrida pelos ciclos nas velocidades-meta (CycleClips) ou pelos clipes de antes.
    void setGaitCycles(bool on);
    void setPushVariety(bool on);
    void setFireGroups(const std::vector<float>& bands);
    void setLateralLegStrict(float factor);
    void setPushImitationRelief(bool on);
    void setRiseFlight(float penalty);
    void setSlowLimit(float fraction);
    void setSlowLimits(std::vector<float> fractions);
    void setHeadClipReach(float reach);
    void setStepTuning(float loadLimit, float loadWeight, bool swingFromBody);
    void setGetUpArrival(float bonus, float flightPenalty, float riseBalance = 0.0f, float calmHandoff = 0.0f);
    void setAvoidOnIdle(bool on);
    void setIdleThreatReach(float reach);
    // O tamanho do estado do equilibrio na observacao (0 sem balanceObservation) e dos canais do passo (0 sem stepChannels).
    [[nodiscard]] std::size_t balanceObservationSize() const;
    [[nodiscard]] std::size_t stepChannelCount() const;
    // Os eixos dos motores (o comeco da acao; depois vem os canais do passo e, com velocityActions, as velocidades).
    [[nodiscard]] std::size_t axisCount() const;
    void setRunAssist(float assist);
    void setManeuvers(float fraction, int evaluationManeuver);
    void setManeuverAssist(float assist);
    void setDrillAssist(std::span<const float> levels);
    // A aluna-oraculo (o teste da exatidao): 0 a rede; 1 o rotulo da professora em sombra nos motores da aluna no tick 0
    // de cada decisao, segurado (como a rede a 30 Hz); 2 a cada tick.
    void setTeacherOracle(int mode);
    void setTeacherLead(float minimum, float maximum);
    void setTeacherShadow(bool shadow);
    // O parado ensinado: 0 a B21 inteira, 1 o tronco no IDLE ALERTA, 2 todas as juntas nele com o equilibrio dela.
    void setTeacherStillClip(int mode);
    // O musculo da cabeca da locomocao (03/11: o do rastreador - o perfil a 1x, sem gravidade - nao reproduz a B21 nem
    // com o rotulo perfeito): as escalas por eixo da acao (rigidez, amortecimento e teto, as do backend) e a compensacao
    // de gravidade (torque das juntas, como na B21 a 0%). Vazio: o do rastreador.
    void setStudentActuator(std::span<const float> stiffness, std::span<const float> damping,
        std::span<const float> maximumTorque, bool gravity);
    // As escalas medias dos drives da professora por eixo da acao: [eixo][rigidez, amortecimento, teto, gravidade,
    // |antecipacao| N.m].
    [[nodiscard]] std::vector<float> teacherDriveStats() const;
    // Os pedidos por rumo (a ela e a aluna, m/s) e a intervencao (teacherTakeoverTilt, ...TiltRate, ...Chest, ...Travel,
    // ...Seconds) - os campos teacher* dos ajustes; valem nos proximos comecos.
    void setTeacherSettings(std::array<float, 8> speeds, std::array<float, 8> studentSpeeds, std::array<float, 5> takeover);
    // O rotulo da ultima decisao de cada mundo (teacherTargetWidth() por mundo): o alvo da aluna que daria o torque da
    // professora (actionSize; a media dos 4 ticks), o alvo dela cru (actionSize, a media), se vale, se ela estava no
    // volante e se retomou nesta decisao. (Guardado fora do recomeco: vale para a decisao que acabou de rodar.)
    [[nodiscard]] std::size_t teacherTargetWidth() const;
    void teacherTargets(std::span<float> out) const;
    [[nodiscard]] std::size_t fallStartCount() const;

    void reset(std::uint64_t seed);
    void observations(std::span<float> out) const;
    // A observacao do fim dos episodios que acabaram no ultimo passo (para o valor do corte).
    void finalObservations(std::span<float> out) const;
    // rewards: a nota da tarefa.
    void step(std::span<const float> actions, std::span<float> rewards, std::span<std::uint8_t> terminated,
        std::span<std::uint8_t> truncated);
    // O par do AMP do ultimo passo de cada mundo (antes e depois, 2 x ampSize), antes de um recomeco.
    void ampPairs(std::span<float> out) const;
    // Os pares dos clipes do estilo, de 1/120 em 1/120 s (2 x ampSize cada; o passo e o de uma decisao).
    [[nodiscard]] std::vector<float> referencePairs() const;
    // So os pares destes clipes (vazio: todos os da tarefa).
    [[nodiscard]] std::vector<float> referencePairsOf(const std::vector<std::string>& clips) const;
    // Carrega clipes so para o juiz (ids da pasta de clipes, com subpasta), no fim da lista; devolve quantos entraram.
    std::size_t addStyleClips(const std::vector<std::string>& names);
    // Os comecos deitados (07/11): o comeco no clipe (1) no quadro 0 de um destes (na avaliacao 9 o primeiro, 10 o segundo:
    // de costas, de brucos) e o levantar em movimento (3) no quadro 0 de um destes; carregados antes (addStyleClips).
    void setLyingStarts(const std::vector<std::string>& standClips, const std::vector<std::string>& runClips);
    void setMovingHandoff(bool handoff);
    // (07/11) O juiz com o Motion Matching (vault 04_locomocao; o conjunto denso do Game Animation Sample): os grupos (nome,
    // esquema "default"/"stop"/"idle"/"unreal", os clipes - carregados antes -, os vieses base/ciclo/continuar e a categoria
    // da escolha: 0 parado, 1 giro no lugar, 2-6 partidas/ciclos/pivos/paradas/giro de 180 do trote, 7-10
    // partidas/ciclos/pivos/paradas da corrida) e o grupo principal (o contexto do juiz). Devolve o numero de quadros.
    std::size_t buildMotionMatching(const std::vector<std::string>& names, const std::vector<std::string>& schemas,
        const std::vector<std::vector<std::string>>& clips, const std::vector<std::array<float, 3>>& biases,
        const std::vector<int>& categories, std::size_t mainGroup);
    // (07/11, so para ver no laboratorio) O corpo dos exercicios de locomocao e do parado posto no quadro que o Motion
    // Matching escolhe a cada decisao, sem fisica (precisa de buildMotionMatching).
    void setKinematicMotion(bool on);
    // (07/11, so para ver no laboratorio) A sequencia de clipes da Unreal de cada exercicio, por regra, sem fisica (os clipes
    // carregados antes por addStyleClips).
    void setClipSequence(bool on);
    // (07/11, so para ver) A estrela de 8 pontas na sequencia de clipes: 0 desligada, 1 peito fixo, 2 peito livre.
    void setStarDrill(int mode);
    [[nodiscard]] std::size_t kinematicJumps(std::size_t world) const;
    [[nodiscard]] std::string kinematicClip(std::size_t world) const;
    // A previsao da trajetoria (m/s^2, 1/s, rad/s; rotacao negativa = na hora).
    void setMotionMovement(float maxAcceleration, float brakingDeceleration, float groundFriction, float brakingFriction,
        float rotationRadiansPerSecond);
    [[nodiscard]] std::size_t motionGroupCount() const;
    [[nodiscard]] std::size_t motionGroupEntries(std::size_t group) const;
    [[nodiscard]] std::size_t motionFeatureSize(std::size_t group) const;
    [[nodiscard]] std::size_t motionQueryOffset(std::size_t group) const;
    [[nodiscard]] std::size_t motionQueryWidth() const;
    [[nodiscard]] std::size_t motionMainGroup() const;
    // Os quadros de um grupo ja ponderados (quadros x tamanho), os pares do juiz deles (quadros x 2 ampSize) e, por quadro, o
    // viés, a fonte (o indice do clipe no grupo) e o instante.
    void motionGroupFeatures(std::size_t group, std::span<float> out) const;
    void motionGroupPairs(std::size_t group, std::span<float> out) const;
    void motionGroupEntryInfo(std::size_t group, std::span<float> addends, std::span<float> sources, std::span<float> times) const;
    // Por mundo, a consulta da ultima decisao (todos os grupos, ponderada; motionQueryOffset), os grupos permitidos (os bits,
    // como numero) e se vale: mundos x (motionQueryWidth + 2).
    void motionQueries(std::span<float> out) const;
    [[nodiscard]] std::vector<TrackingGetUpAmpEpisode3D> takeEpisodes();

    [[nodiscard]] const RagdollState3D& body(std::size_t world) const;
    // Os alvos de junta de agora de um mundo (os da ultima acao).
    [[nodiscard]] std::span<const RagdollDriveTarget3D> driveTargets(std::size_t world) const;
    [[nodiscard]] const RagdollProfile3D& profile() const;
    [[nodiscard]] std::string frameLines(std::size_t world) const;
    // (05/11 06h50, as ferramentas de inspecao - tools/rl/inspecao): a referencia seguida agora no mundo - o quadro do clipe
    // posto sob a pelve do corpo, como a nota o compara -, nas mesmas linhas "C"/"B" do frameLines, e uma linha "Q" com o
    // clipe, o instante dele, a forca auxiliar, quem dirige (1 a cabeca da locomocao), o pedido (m/s) e o rumo dele (rad).
    // Vazio sem clipe seguido.
    [[nodiscard]] std::string referenceLines(std::size_t world) const;
    [[nodiscard]] TrackingGetUpAmpWorldView3D worldView(std::size_t world) const;
    // O cenario do mundo em arame (no mundo): as arestas das caixas (a rampa cortada em 2,5 m em volta do meio).
    void scenerySegments(std::size_t world, std::vector<std::pair<Vec3, Vec3>>& out) const;
    // Os corpos soltos do mundo (do mesmo perfil do corpo; validos ate o proximo passo).
    void sceneryBodies(std::size_t world, std::vector<const RagdollState3D*>& out) const;

private:
    struct Impl;
    std::unique_ptr<Impl> m_impl;
};

} // namespace MatterEngine
