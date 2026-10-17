#pragma once

#include "Engine/Animation/AnimationClip3D.hpp"
#include "Engine/Character/BallGesturePose3D.hpp"
#include "Engine/Character/CharacterLocomotion3D.hpp"
#include "Engine/Character/PoseInertializer3D.hpp"
#include "Engine/Math/Quaternion.hpp"
#include "Engine/Math/Vec3.hpp"
#include "Engine/Physics/PhysicsScene3D.hpp"
#include "Engine/Physics/RagdollProfile3D.hpp"

#include <array>
#include <cstdint>
#include <functional>
#include <string>
#include <utility>
#include <vector>

namespace MatterEngine {

// Locomocao padrao (pedido do usuario, 04/10): o boneco sem fisica, levado
// pela capsula, como na Unreal e na maioria dos jogos. A capsula decide onde
// ele esta e a que velocidade (a definida, com troca de direcao imediata); o
// corpo e a animacao em cima dela, com o desenho do modo fisico adaptado
// para fluidez e resposta:
//  - oito setores de marcha pela direcao em relacao a camera (frente,
//    diagonais da frente, lados, diagonais de costas, costas), cada um com o
//    seu clipe: a pelve gira para o movimento o que falta para o rumo do
//    clipe (sem o clipe da diagonal, o da frente ou o de costas, e a pelve
//    gira mais); a coluna e a cabeca devolvem o olhar para a camera (e a
//    inclinacao dele). Parado, a camera girando, a coluna e a cabeca giram
//    antes e a pelve so vai quando o giro passa do que elas cobrem;
//  - trote e sprint misturados pela velocidade, numa fase unica (os ciclos
//    alinhados pelo pouso do pe esquerdo); a passada se adapta a velocidade
//    da capsula pelo ritmo e pelo comprimento (os pes acompanham o chao);
//  - curva: o corpo inclina para dentro (atan(v w / g)) em volta dos pes, a
//    pelve rola uma parte e a coluna o resto; a pelve adianta para dentro,
//    os clipes de corrida em arco entram na coluna e nas pernas e o pe de
//    dentro gira para a curva;
//  - arrancada e freada inclinam o corpo para a frente e para tras;
//  - terreno: cada pe vai a altura do chao embaixo dele (a pelve desce o que
//    a perna nao alcanca);
//  - carrinho (o clipe, no caminho da raiz dele, escalado pela velocidade de
//    entrada), pulo pela fase do voo, o agachar da carga do pulo, o corpo do
//    chute (bracos e tronco) e a "vida" do movimento.
// Sem planejador de pes, sem equilibrio: o corpo nunca treme nem
// desequilibra. O modo fisico (CharacterLocomotion3D com os motores) entra
// so em momentos especificos (pancada, pouso).
// Vida do tronco e da cabeca de uma classe de marcha, somada ao clipe, na
// fase do ciclo (0 = pouso do pe esquerdo): altura da pelve (m) e balanco do
// tronco e da cabeca para a frente (rad, + para a frente) por passo (duas
// vezes por ciclo: cos/sen de 4 pi fase), inclinacao media do tronco e da
// cabeca (rad) e o tombo da cabeca para os lados por ciclo (cos/sen de 2 pi
// fase, + para a esquerda). Calibrada pela diferenca entre as referencias e
// a nossa saida (ver trunkMotion).
struct DefaultTrunkMotion3D {
    float heightCos2 = 0.0f, heightSin2 = 0.0f;
    float pitchMean = 0.0f, pitchCos2 = 0.0f, pitchSin2 = 0.0f;
    float headPitchMean = 0.0f, headPitchCos2 = 0.0f, headPitchSin2 = 0.0f;
    float headRollCos1 = 0.0f, headRollSin1 = 0.0f;
};

struct DefaultLocomotionSettings3D {
    // Rumo da pelve: constante (s) andando e velocidade maxima (rad/s).
    float turnSeconds = 0.10f;
    float maximumTurnRateRadiansPerSecond = 10.0f;
    // Transicoes adaptativas (pedido do usuario, 05/10: trocas suaves, que
    // coincidam com o movimento, como um corpo):
    //  - a pelve gira numa mola de segunda ordem (constante turnSpringSeconds,
    //    parado standTurnSpringSeconds) com aceleracao limitada - sem ela a
    //    velocidade do giro saltava do zero ao maximo num tick (56 000 graus/s2
    //    nas diagonais);
    //  - frente, lados e costas se misturam pelo angulo entre o movimento real
    //    da capsula e a pelve (as pernas sempre no movimento que o corpo faz;
    //    o setor so decide para onde a pelve vai);
    //  - a coluna e a cabeca seguem o olhar em molas de segunda ordem.
    // Desligado, o modo de antes (setor discreto, giro de primeira ordem).
    bool adaptiveTransitions = true;
    // Clipes amostrados numa curva entre as chaves (ver
    // sampleAnimationTrackSmooth3D): os de 30 quadros/s - strafe, arcos, pulos,
    // o parado - tremiam um pouco a cada chave.
    bool smoothClipSampling = true;
    float turnSpringSeconds = 0.05f;
    float standTurnSpringSeconds = 0.08f;
    float maximumTurnAcceleration = 100.0f;
    // Entre dois clipes vizinhos a mistura e linear no angulo. Com uma zona
    // morta (curva em S: ate esta fracao do angulo, o clipe mais perto puro)
    // o deslize dos pes no zigue-zague piorou (p95 5,6 -> 7,5 m/s); fica 0.
    float directionBlendDeadZone = 0.0f;
    float torsoLookSeconds = 0.045f;
    // Vida do tronco e da cabeca (pedido do usuario, 05/10: no padrao o
    // tronco ficava reto e imovel). Por classe: trote e sprint, para a frente
    // e de costas (os lados ficam como o clipe). trunkMotionAmount escala tudo.
    // Calibrada (05/10) pela diferenca entre as referencias Mixamo e a nossa
    // saida, por harmonico, na fase do ciclo. A altura por passo fica a do
    // clipe (zero aqui): ampliada ate a da referencia (~6 cm), o usuario
    // achou "ridiculo" o sobe-e-desce. Balanco do tronco e da cabeca por
    // passo, o do tronco da referencia; inclinacao media do tronco a 60% da
    // diferenca (a cabeca no mundo desce metade disso). A cabeca nao balanca
    // nem tomba a mais: as juntas dela descontam o balanco que a coluna da
    // (fica estavel no mundo, como a de quem corre) - com o balanco e o tombo
    // das referencias, o usuario achou que ela balancava rapido demais. Os
    // harmonicos de uma vez por ciclo na altura e no balanco para a frente
    // ficam de fora (nas referencias sao assimetria, parecia mancar).
    bool trunkMotion = true;
    float trunkMotionAmount = 1.0f;
    std::array<DefaultTrunkMotion3D, 4> trunkMotions {
        // trote para a frente (Jog Forward)
        DefaultTrunkMotion3D { 0.0f, 0.0f, 0.2109f, -0.0234f, -0.0064f,
            -0.1582f, 0.0234f, 0.0064f, 0.0f, 0.0f },
        // sprint para a frente (Running)
        DefaultTrunkMotion3D { 0.0f, 0.0f, 0.0618f, 0.0328f, 0.0732f,
            -0.0463f, -0.0328f, -0.0732f, 0.0f, 0.0f },
        // trote de costas (Jog Backward)
        DefaultTrunkMotion3D { 0.0f, 0.0f, -0.0525f, 0.0069f, 0.0811f,
            0.0394f, -0.0069f, -0.0811f, 0.0f, 0.0f },
        // sprint de costas (Running Backward)
        DefaultTrunkMotion3D { 0.0f, 0.0f, 0.0315f, 0.0498f, 0.0750f,
            -0.0237f, -0.0498f, -0.0750f, 0.0f, 0.0f }
    };
    // Trava do pe apoiado: o pe que o clipe diz apoiado fica onde pousou (a
    // perna torce e dobra por IK) e solta em footLockBlendSeconds no fim do
    // apoio, ou se ficar a mais de footLockReleaseMeters de onde a animacao o
    // poe. Medido (05/10) assim: o pe animado se afasta da trava no meio do
    // apoio, ela solta e a volta da picos nas pernas (circulo 455 -> 1558
    // m/s2) e mais deslize. Desligada ate ser refeita.
    bool footLock = false;
    float footLockReleaseMeters = 0.18f;
    // Os dedos do pe dobram no chao (07/10, pedido do usuario): a sola conta so
    // ate a base dos dedos - toeRollMeters atras da frente da caixa do pe (no
    // jogador, 8 cm) -; no fim do apoio o pe gira ali, nao na ponta, e os dedos
    // (o osso visual) ficam deitados no chao.
    bool toeRoll = true;
    float toeRollMeters = 0.08f;
    float footLockBlendSeconds = 0.10f;
    // Parando (sem pedido, abaixo de stopSettleSpeed m/s): o passo termina -
    // o ciclo vai ate o proximo cruzamento das pernas (pes juntos embaixo do
    // corpo), freando nesta constante (s). Antes as pernas ficavam abertas
    // no meio da passada e depois escorregavam ate o parado. Medido (05/10):
    // o ritmo do ciclo salta para alcancar o cruzamento (os bracos a 4x) -
    // picos de 25 000 graus/s2 no peito. Desligada ate frear o ritmo sem salto.
    bool settleStop = false;
    float stopSettleSpeed = 1.5f;
    float stopSettleSeconds = 0.12f;
    float headLookSeconds = 0.05f;
    // Parado: a pelve vai a camera so quando o erro passa de standTurnStart
    // (rad), e para standTurnKeep (rad) dela - o resto e da coluna e da
    // cabeca -, nesta constante (s).
    float standTurnStart = 0.55f;
    float standTurnKeep = 0.31f;
    float standTurnSeconds = 0.16f;
    // Setores: oito, de 45 graus em volta de cada rumo (0 = a camera); folga
    // (graus) para trocar.
    float sectorHysteresisDegrees = 10.0f;
    // Velocidade da capsula (m/s) por rumo em relacao a camera - 0, 45, 90,
    // 135 e 180 graus, os dois lados iguais, interpolada entre eles. O trote
    // e o sprint se misturam por ela. Pedido do usuario (05/10): de costas na
    // diagonal o sprint e o trote nao perdem tanta velocidade.
    // Pedido do usuario (05/10): trote um pouco mais rapido e igual em todos
    // os rumos; corrida um pouco mais rapida; de lado e de costas bem mais
    // (de lado, o mesmo sprint de lado tocado mais rapido - uma passada
    // saltada no lugar dele ficou "feia e robotica"; de costas, o
    // sprint_backward_long, perto da corrida para a frente).
    // Recalibrado (pedido do usuario, 06/10) pelo corpo no modo fisico (ajuda
    // 100%, sweepvelocidades): o sprint um pouco acima dele em cada rumo (o
    // fisico: 7,5 a frente e na diagonal, 4,1 de lado, 4,4 na diagonal de
    // costas e de costas - eram 8,0 / 8,0 / 5,5 / 7,0 / 7,0, rapidos demais de
    // lado e de costas); o trote um pouco mais rapido (era 3,4; o fisico ~3,0).
    // Calibradas pelo usuario no laboratorio (06/10, aba "Velocidades"): o
    // trote 4,5 em todos os rumos; a corrida 7,8 a frente, 5,2 de lado e de
    // costas.
    std::array<float, 5> trotSpeeds { 4.5f, 4.5f, 4.5f, 4.5f, 4.5f };
    std::array<float, 5> sprintSpeeds { 7.8f, 7.8f, 5.2f, 5.2f, 5.2f };
    // A marcha em outra velocidade que a do clipe (pedido do usuario, 06/10:
    // o trote a 4,5 m/s, com o clipe feito para ~3, era um "movimentinho
    // curtinho e rapidinho"; depois tambem a corrida, sobretudo de lado): o
    // movimento do quadril, dos joelhos e dos bracos cresce (ou encolhe) em
    // volta da media do ciclo - a razao de velocidades elevada a
    // gaitAmplitudeExponent, entre gaitAmplitudeMinimum e gaitAmplitudeMaximum
    // - e o ritmo leva o resto (os pes nao escorregam). A coluna,
    // gaitAmplitudeSpine disso; pescoco, cabeca e pes nao.
    bool gaitAmplitude = true;
    float gaitAmplitudeExponent = 0.5f;
    float gaitAmplitudeMinimum = 0.85f;
    float gaitAmplitudeMaximum = 1.3f;
    float gaitAmplitudeSpine = 0.5f;
    // De lado, so esta parte: no trote (o strafe de passo cruzado) inteira as
    // pernas abriam e cruzavam demais; na corrida de lado, inteira.
    float trotAmplitudeSide = 0.5f;
    float sprintAmplitudeSide = 1.0f;
    // A amplitude muda no maximo neste tempo (s, filtro).
    float gaitAmplitudeSeconds = 0.25f;
    // Plano v5: a passada de conducao (entrada dribbleStyle). A cadencia sobe
    // ate os pousos alternados ficarem a dribbleHalfCycle (s) - no maximo
    // dribbleCadenceMaximum vezes - e a amplitude divide pelo mesmo: passos
    // curtos e rapidos, sem o pe escorregar (andando a 1 m/s o trote pousava a
    // cada 0,63 s e cada perna so tocava a bola uma vez por 1,3 s). Muda no
    // maximo em dribbleCadenceSeconds.
    float dribbleHalfCycle = 0.30f;
    float dribbleCadenceMaximum = 2.2f;
    float dribbleCadenceSeconds = 0.25f;
    // Mistura das marchas (s, molas criticamente amortecidas): a forma do
    // ciclo (setor, trote/sprint) e o parado <-> andando.
    float gaitBlendSeconds = 0.12f;
    float idleBlendSeconds = 0.12f;
    // Trocas de clipe em camadas: o tronco, os bracos e a cabeca podem chegar
    // a pose nova mais devagar que as pernas (nestes tempos). Testado em 05/10
    // com 0,22/0,26 s, o afundar do peso (3,5 cm) e o gesto da arrancada -
    // o usuario achou grotesco; desligados (iguais as pernas, zero, falso).
    float upperGaitBlendSeconds = 0.12f;
    float upperIdleBlendSeconds = 0.12f;
    float transitionDipMeters = 0.0f;
    float transitionDipSeconds = 0.09f;
    bool startGesture = false;
    // Abaixo disto (m/s) entra a pose parada.
    float idleSpeed = 1.0f;
    // A passada na velocidade da capsula: o comprimento leva a raiz quadrada
    // da razao de velocidades (dentro dos limites dele) e o ritmo o resto
    // (dentro dos dele). O comprimento (os pes por IK ao longo do ciclo) fica
    // desligado (1..1): medido, os picos das pernas iam de ~500 a ~2000 m/s2
    // no sprint e o deslize quase nao mudava.
    float minimumPlayRate = 0.55f;
    float maximumPlayRate = 1.6f;
    float minimumStrideScale = 1.0f;
    float maximumStrideScale = 1.0f;
    // O olhar: a coluna devolve ate torsoTwist (graus; correndo rapido,
    // sprintTorsoTwist) com torsoLookShare do erro, a cabeca completa ate
    // headTurn; ritmo (1/s) de cada um.
    float torsoTwistDegrees = 45.0f;
    float sprintTorsoTwistDegrees = 20.0f;
    float headTurnDegrees = 75.0f;
    float torsoLookShare = 0.8f;
    float torsoLookRate = 20.0f;
    float headLookRate = 16.0f;
    // Curva (pedido do usuario, 04/10: o corpo todo inclina, nao so o tronco;
    // correndo rapido, no trote so um pouco): inclinacao maxima (rad) do corpo
    // inteiro em volta dos pes; a parte no trote (curveTrotShare) e a cheia
    // a partir de curveSprintFrom + curveSprintSpan m/s; adiantamento da
    // pelve (s de giro, maximo rad).
    float curveLeanMaximum = 0.18f;
    // Zona morta (m/s2 de aceleracao lateral; pedido do usuario, 05/10
    // noite): curvas leves e pequenas mudancas de direcao nao inclinam - so
    // as fechadas, e a partir de zero ali. (Sem ela, correndo a 8 m/s uma
    // curva de raio 20 m ja pedia a inclinacao maxima.) Vale tambem para os
    // clipes de corrida em arco.
    float curveLeanDeadZone = 3.5f;
    // Carrinho: os dois deslizam o mesmo para a mesma velocidade de entrada.
    bool tackleEqualSlide = true;
    // Carrinho no ar (a voadora; pedido do usuario, 05/10 noite): o pedido no
    // voo poe o corpo na pose do mergulho do carrinho, deitando para tras
    // (graus) com a perna esticada a frente; no pouso o deslize continua.
    bool airTackle = true;
    float airTacklePitchDegrees = 30.0f;
    float curveTrotShare = 0.15f;
    float curveSprintFrom = 3.2f;
    float curveSprintSpan = 3.3f;
    float pelvisCurveLeadSeconds = 0.25f;
    float pelvisCurveLeadMaximum = 0.20f;
    // Na curva o pe pousa girado para dentro dela, como no modo fisico (rad no
    // maximo; entra entre start e start + span m/s2 da aceleracao que o corpo
    // faz na curva; o de fora, outerShare dele; o ritmo, 1/s). Gira no
    // balanco e fica assim no apoio; a perna inteira gira pela coxa.
    float footCurveYawMaximum = 0.25f;
    float footCurveYawStart = 4.0f;
    float footCurveYawSpan = 6.0f;
    float footCurveYawOuterShare = 0.2f;
    float footCurveYawRate = 10.0f;
    // Clipes de corrida em arco: peso maximo (vezes a parte da curva pela
    // velocidade) e a parte das pernas.
    float curveClipMaximum = 0.35f;
    float curveClipLegShare = 0.15f;
    // Arrancada/freada: inclinacao para a frente (fracao de atan(a/g), limites
    // em rad) e a mola (1/s) da inclinacao.
    float forwardLeanShare = 0.12f;
    float forwardLeanMinimum = -0.08f;
    float forwardLeanMaximum = 0.10f;
    float leanRate = 12.0f;
    // Filtro da aceleracao e do giro do caminho (s).
    float accelerationFilterSeconds = 0.05f;
    float pathTurnFilterSeconds = 0.25f;
    // Terreno: a pelve desce no maximo isto (m) para o pe alcancar.
    float terrainDropMaximum = 0.35f;
    // Preparo do pouso (pedido do usuario, 05/10: como no modo fisico). Pelo
    // progresso do voo (da decolagem ao pouso previsto pela queda balistica),
    // de landingPrepStart a landingPrepFull numa curva suave, e o peso numa
    // mola de landingPrepSeconds - o usuario pediu antes e mais suave que pela
    // queda (comecava depois do ponto mais alto e subia em 0,2 s). A perna
    // aponta para onde o corpo vai bater - na linha da velocidade do pouso
    // previsto, landingAlign do angulo dela com a vertical, no maximo
    // landingMaxAngle (seguindo correndo, landingRunMaxAngle e so a perna mais
    // baixa) - com o joelho dobrado (landingLegShare do comprimento), e a
    // coluna na mesma linha: o tronco inclina para tras landingTrunkAlign do
    // angulo da perna (de lado, landingTrunkSide disso). Mais forte e mais
    // cedo que no fisico (0,25 s, 0,7, 0,6 rad, tronco 0,5): la puxar as
    // pernas cedo girava o corpo (momento angular); aqui nao, e o usuario
    // pediu o corpo inteiro na linha da queda, a descida toda.
    bool landingPrep = true;
    float landingPrepStart = 0.15f;
    float landingPrepFull = 0.70f;
    float landingPrepSeconds = 0.08f;
    float landingAlign = 0.9f;
    float landingMaxAngle = 0.75f;
    float landingRunMaxAngle = 0.55f;
    float landingLegShare = 0.93f;
    float landingTrunkAlign = 0.8f;
    float landingTrunkSide = 0.3f;
    // A pelve desce isto (m) com o preparo: com a perna inclinada para a
    // frente, o pe alcanca menos para baixo e, sem descer, ficava 37 cm acima
    // do chao quando a capsula tocava.
    float landingPelvisDrop = 0.18f;
    // Aterrissagem (pedido do usuario, 05/10 noite; referencia Falling To
    // Landing, sem os bracos): no toque a pelve afunda e volta (os pes
    // apoiados ficam, o joelho dobra), a coluna flexiona - a parte de cima
    // mais e um pouco depois - e a cabeca compensa; um clipe rapido por cima
    // da locomocao, sem parar o movimento. A intensidade vem da velocidade
    // vertical do toque (landingSpeedMinimum..landingSpeedFull m/s): pulos
    // pequenos, sutil e rapido. Na referencia (toque a 3,8 m/s): pelve 34 cm,
    // joelhos +70 graus, tronco +29, o fundo em 0,20 s e a volta em 0,6 s.
    bool landingAbsorb = true;
    float landingSpeedMinimum = 1.5f;
    float landingSpeedFull = 7.0f;
    float landingDepthMaximum = 0.32f;
    float landingSecondsMinimum = 0.22f;
    float landingSecondsMaximum = 0.65f;
    // Fracao do tempo ate o fundo.
    float landingCompressShare = 0.33f;
    // Flexao (rad) no maximo: a coluna baixa (abdomen e peito), o peito alto
    // e a compensacao do pescoco e da cabeca (para tras).
    float landingTrunkMaximum = 0.50f;
    float landingUpperMaximum = 0.40f;
    float landingHeadMaximum = 0.25f;
    // Correndo (a partir de 2 m/s, cheio em 5), a absorcao vai a esta parte.
    float landingRunningShare = 0.65f;
    // O pulo entra/sai nesta constante (s); acima de travelJumpSpeed (m/s)
    // o pulo com deslocamento.
    float airBlendSeconds = 0.08f;
    float travelJumpSpeed = 1.5f;
    // Carrinho: velocidade minima para a frente (m/s), intervalo entre dois
    // (s), entrada e saida misturadas (s).
    float tackleMinimumSpeed = 2.5f;
    float tackleCooldownSeconds = 0.35f;
    float tackleBlendInSeconds = 0.18f;
    float tackleExitSeconds = 0.30f;
};

struct DefaultLocomotionInput3D {
    // A capsula: os pes (a base dela), a velocidade e se esta no chao.
    Vec3 feetPositionWorld;
    Vec3 velocityWorld;
    bool grounded = true;
    // O pedido do jogador (m/s, no plano): o setor sai dele quando ha.
    Vec3 requestedVelocityWorld;
    // A camera: para onde o corpo olha e a inclinacao do olhar.
    float facingYawRadians = 0.0f;
    float lookPitchRadians = 0.0f;
    float deltaTime = 1.0f / 120.0f;
    // O chao num ponto (opcional; sem, o plano da base da capsula).
    std::function<GroundProbeResult3D(Vec3)> groundAt;
    // Carga do pulo (0..1): o agachar de preparo.
    float jumpCrouch = 0.0f;
    // Carrinho: o pedido (borda) e se esta liberado.
    bool tackleRequest = false;
    bool tackleEnabled = true;
    // O Ctrl segurado e, no ar, Shift + W (pedido do usuario, 05/10 noite: a
    // voadora pede os tres juntos, em qualquer ordem).
    bool tackleHeld = false;
    bool airTackleCombo = false;
    // O contrato do gesto (a reescrita, R4): a camada de pose que uma acao
    // poe por cima da locomocao - hoje o gesto do corpo inteiro do manejo de
    // bola (rework de 06/10). A locomocao nao conhece a bola: so esta camada.
    BallGesturePose3D ballGesture;
    // "Vida" (0..2; 0 desliga) e a semente do jeito de cada um.
    float motionVariety = 0.0f;
    std::uint32_t varietySeed = 0;
    // O ritmo da passada vezes isto (o gesto com a bola entra na passada: o
    // pe do toque pousa no clipe quando o gesto volta). 1: o normal.
    float phaseRateScale = 1.0f;
    // Plano v5: a passada de conducao (0..1: a cadencia sobe com os passos
    // mais curtos, como num drible).
    float dribbleStyle = 0.0f;
};

// Plano v5: uma janela de balanco de um pe (s a partir de agora, no ritmo
// aplicado: do pe sair do chao ao pouso).
struct DefaultSwingWindow3D {
    float start = 0.0f;
    float end = 0.0f;
};

struct DefaultLocomotionOutput3D {
    bool valid = false;
    // A pose de cada link (origem e orientacao), no mundo; e a mesma pose
    // como raiz (mundo) e coordenadas de junta (as dos clipes e motores).
    RagdollAnimationPose3D pose;
    Vec3 rootPosition;
    Quaternion rootOrientation;
    std::vector<Vec3> coordinates;
    // Rumo da pelve.
    float facingYawRadians = 0.0f;
    // Fase do ciclo (0..1; 0 = pouso do pe esquerdo) e o quanto o corpo
    // anda (0 parado, 1 andando).
    float cyclePhase = 0.0f;
    float movingShare = 0.0f;
    bool airborne = false;
    // Aterrissagem: a intensidade (0..1; 0 nenhuma) e a descida da pelve (m).
    float landingIntensity = 0.0f;
    float landingSink = 0.0f;
    // Os pes pela animacao: apoio (0..1), plantado, progresso do balanco
    // (-1 apoiado) e quanto falta para pousar (s; -1 apoiado).
    std::array<float, 2> footContact { 1.0f, 1.0f };
    std::array<bool, 2> footPlanted { true, true };
    std::array<float, 2> swingProgress { -1.0f, -1.0f };
    std::array<float, 2> secondsToTouchdown { -1.0f, -1.0f };
    // As pegadas (Athlete/FootstepPlanner3D): andando (no ciclo), o apoio de
    // cada pe pela fase da marcha dominante (o footPlanted vem do contato
    // misturado com o parado e discorda dela nas misturas), quanto falta para
    // sair e para pousar (s; -1 fora do ciclo) e quanto dura o balanco (s).
    bool cycling = false;
    std::array<bool, 2> cyclePlanted { true, true };
    std::array<float, 2> secondsToLiftoff { -1.0f, -1.0f };
    std::array<float, 2> cycleSecondsToTouchdown { -1.0f, -1.0f };
    std::array<float, 2> swingSeconds { -1.0f, -1.0f };
    // Os pes do estilo antes do gesto da bola, no mundo (o manejo novo,
    // correndo, toca a bola no balanco deles).
    std::array<Vec3, 2> styleFootPosition {};
    std::array<Quaternion, 2> styleFootOrientation {};
    // Cada pe como a locomocao o poria sem o gesto (o do estilo com a passada
    // e o chao) e para onde o joelho do estilo aponta (mundo): o gesto volta a
    // eles (plano v4).
    std::array<Vec3, 2> locomotionFootPosition {};
    std::array<Quaternion, 2> locomotionFootOrientation {};
    std::array<Vec3, 2> styleKneeDirection { Vec3 { 1.0f, 0.0f, 0.0f }, Vec3 { 1.0f, 0.0f, 0.0f } };
    // Plano v5 (o manejo refeito): andando (cycling), as janelas de balanco de
    // cada pe - a atual, se no ar (comeco negativo), e as proximas -, o ciclo
    // (s) e o pe do estilo no futuro (a origem do link e a orientacao), no
    // referencial do corpo: a pelve no chao, o rumo dela (x frente, y
    // esquerda), a cada forecastStep s a partir de agora (o primeiro e agora),
    // pela tabela das marchas (sem a amplitude) com a passada de agora.
    static constexpr int SwingWindowCount = 3;
    static constexpr int ForecastCount = 16;
    std::array<std::array<DefaultSwingWindow3D, SwingWindowCount>, 2> swingWindows {};
    float cycleSeconds = -1.0f;
    float forecastStep = 0.04f;
    std::array<std::array<Vec3, ForecastCount>, 2> footForecast {};
    std::array<std::array<Quaternion, ForecastCount>, 2> footForecastOrientation {};
    // Inclinacao aplicada (rad: x para a esquerda, y para a frente).
    Vec3 lean;
    // Carrinho: em andamento (a pelve vai pelo clipe; a capsula vai embaixo
    // dela), a pelve no mundo e a velocidade dela; terminou neste tick. A
    // arrancada agachada (startCrouchSprint) usa o mesmo caminho: rootScripted
    // vale para os dois, tackling so para o carrinho.
    bool tackling = false;
    // A voadora (no ar, ou o deslize que veio dela): o laboratorio a leva ao
    // modo fisico ja (pedido do usuario, 05/10 noite).
    bool airTackle = false;
    bool rootScripted = false;
    bool crouchStarting = false;
    Vec3 tackleRootWorld;
    Vec3 tackleVelocity;
    bool tackleEnded = false;
};

// A velocidade da capsula no modo padrao para um movimento que aponta
// travelRelativeToLookRadians em relacao a camera (ver trotSpeeds).
[[nodiscard]] float defaultLocomotionSpeed3D(const DefaultLocomotionSettings3D& settings,
    float travelRelativeToLookRadians, bool sprinting);

// Os clipes do modo padrao: os da locomocao fisica com os papeis do
// manifesto ("defaultLocomotion": papel -> clipe) por cima.
[[nodiscard]] CharacterLocomotionAnimations3D defaultLocomotionAnimations3D(
    CharacterLocomotionAnimations3D physical, const std::vector<std::pair<std::string, std::string>>& roles,
    const std::vector<AnimationClip3D>& clips);

// Carrinho (pedido do usuario, 05/10 noite): para onde o clipe vira o corpo
// (+1 esquerda, -1 direita: o rumo da pelve no meio do deslize) e quanto ele
// desliza (m: do comeco ate a pelve voltar a subir), pela raiz.
struct TackleClipShape3D {
    int turnSign = 0;
    float slideMeters = 0.0f;
    // A fase deitada (a pelve 40 cm abaixo da de pe): comeco e fim no clipe
    // (s) e a velocidade da raiz (m/s) ao entrar e ao sair dela.
    float lowStart = 0.0f, lowEnd = 0.0f, lowEntrySpeed = 0.0f, lowExitSpeed = 0.0f;
    // E a distancia (m) nela.
    float lowMeters = 0.0f;
    // O mergulho: o tempo (s) em que a pelve comeca a descer para a fase
    // deitada (6 cm abaixo da de pe; a voadora percorre dali ate ela no ar).
    float diveStart = 0.0f;
};
[[nodiscard]] TackleClipShape3D tackleClipShape3D(const RagdollProfile3D& profile, const AnimationClip3D& clip);
// Qual carrinho (indice; -1 nenhum): o que vira o corpo para o lado em que a
// camera esta girando (rad/s, + esquerda); parada, para o lado da corrida
// (lateralTravel: seno do rumo contra a camera, + esquerda); senao, ao acaso.
[[nodiscard]] int chooseTackleClip3D(const std::array<const AnimationClip3D*, 2>& clips,
    const std::array<TackleClipShape3D, 2>& shapes, float cameraTurnRate, float lateralTravel, std::uint32_t random);

// A pose no mundo a partir da raiz e das coordenadas de junta.
void defaultLocomotionPose3D(const RagdollProfile3D& profile, Vec3 rootPosition, Quaternion rootOrientation,
    const std::vector<Vec3>& coordinates, RagdollAnimationPose3D& pose);

// De onde o corpo esta ao passar do fisico para o padrao: o rumo da pelve,
// a velocidade da capsula e os pes e a pelve no mundo.
struct DefaultLocomotionSeed3D {
    float pelvisYawRadians = 0.0f;
    Vec3 velocityWorld;
    Vec3 pelvisWorld;
    Vec3 pelvisVelocityWorld;
    std::array<Vec3, 2> feetWorld {};
    std::array<Vec3, 2> feetVelocityWorld {};
    bool feetValid = false;
};

class DefaultLocomotion3D {
public:
    // Comeca parado, olhando para facingYaw.
    void reset(float facingYawRadians);
    // Depois do reset: comeca de onde o corpo esta (a volta do fisico) - a
    // pelve no rumo dela, as misturas ja na velocidade e a passada na fase
    // em que os pes do clipe mais se parecem com os do corpo. Sem isso o
    // padrao nascia parado, olhando para a camera, no comeco do ciclo.
    void seed(const DefaultLocomotionSeed3D& seed);
    // Arrancada agachada (pedido do usuario, 05/10 noite: o levantar com
    // pressa vai ate perto do agachado e troca para o Crouched To Sprinting):
    // no proximo update o clipe animations.crouchToSprint, no rumo travelYaw,
    // a pelve pelo caminho dele (como o carrinho); no fim a passada continua
    // (a fase pelos pes do fim do clipe). Correndo (sprint), o caminho inteiro;
    // trotando, encurtado para a velocidade do trote.
    void startCrouchSprint(float travelYawRadians, bool sprint);
    void update(const RagdollProfile3D& profile,
        const CharacterLocomotionAnimations3D& animations,
        const DefaultLocomotionInput3D& input);
    [[nodiscard]] const DefaultLocomotionOutput3D& output() const { return m_output; }
    // O carrinho em andamento (para o modo fisico continuar dele, numa
    // pancada): o clipe (nullptr: nenhum), o tempo nele (s), o ritmo, a
    // escala do caminho, o rumo, a origem do caminho (mundo) e a velocidade
    // de entrada.
    struct Tackle {
        const AnimationClip3D* clip = nullptr;
        float time = 0.0f, rate = 1.0f, scale = 1.0f, yaw = 0.0f, entrySpeed = 0.0f;
        Vec3 start;
    };
    // (No ar, a voadora: o fisico continua dela ja na pose do mergulho - a
    // perna esticada; o carrinho dele a segura ate o corpo descer.)
    [[nodiscard]] Tackle tackle() const {
        if (m_airTackleClip != nullptr)
            return { m_airTackleClip, m_airTackleTarget, m_tackleRate, m_tackleScale, m_airTackleYaw,
                m_tackleEntryVelocity.length(), m_tackleStart };
        return { m_tackleClip, m_tackleTime, m_tackleRate, m_tackleScale, m_tackleYaw,
            m_tackleEntryVelocity.length(), m_tackleStart };
    }
    [[nodiscard]] DefaultLocomotionSettings3D& settings() { return m_settings; }

private:
    // As dezesseis marchas: trote (0..7) e sprint (8..15) nos oito rumos, de
    // 45 em 45 graus a partir da frente, para a esquerda.
    static constexpr std::size_t Directions = 8;
    static constexpr std::size_t GaitCount = 2 * Directions;
    struct Gait {
        const AnimationClip3D* clip = nullptr;
        float speed = 0.0f;
        // Para onde o clipe anda em relacao a pelve (rad): a pelve fica no
        // movimento menos isto.
        float heading = 0.0f;
        // Onde o pe esquerdo pousa (fracao do clipe): a fase 0 do ciclo.
        float touchdown = 0.0f;
        // Quando cada pe sai e pousa (fracao, ja na fase do ciclo).
        std::array<float, 2> liftOff {};
        std::array<float, 2> land {};
        // A fase (do ciclo) em que os pes passam um pelo outro embaixo do
        // corpo - a mais perto da pose parada: a saida do parado comeca nela.
        float crossing = 0.0f;
        // Plano v5: os pes pelo ciclo (a origem do link e a orientacao em
        // relacao a pelve, no rumo dela) em FeetSamples fases - a previsao.
        static constexpr int FeetSamples = 48;
        std::array<std::array<Vec3, FeetSamples>, 2> feet {};
        std::array<std::array<Quaternion, FeetSamples>, 2> feetTurn {};
        // (E a media de cada pe no ciclo: a amplitude da marcha escala o pe
        // em volta dela.)
        std::array<Vec3, 2> feetMean {};
        // A media de cada junta no ciclo (a amplitude do trote cresce em
        // volta dela; ver trotAmplitude).
        std::vector<Vec3> mean;
        // E o minimo e o maximo de cada junta no ciclo (a amplitude cabe nos
        // limites da junta: ver sampleInto).
        std::vector<Vec3> minimum, maximum;
    };
    // O quanto cada link entra na amplitude da marcha (quadril, joelhos e
    // bracos 1, a coluna gaitAmplitudeSpine, o resto 0) e a do tick, do trote
    // e da corrida.
    std::vector<float> m_amplitudeShare;
    std::array<float, 2> m_gaitAmplitude { 1.0f, 1.0f };
    // A cadencia da passada de conducao (1: a normal).
    float m_dribbleCadence = 1.0f;
    void prepare(const RagdollProfile3D& profile, const CharacterLocomotionAnimations3D& animations);
    // As tabelas do prepare sao funcao so do perfil e dos clipes: feitas uma
    // vez por processo e copiadas (antes, cada atleta e cada episodio do treino
    // refaziam ~5-8 ms de amostragem dos clipes).
    void computePrepared(const RagdollProfile3D& profile, const CharacterLocomotionAnimations3D& animations);
    struct PreparedTables;
    // A reescrita (R4): o update em blocos (DefaultLocomotion3D_<bloco>.cpp).
    // O Tick e o que um bloco passa para os seguintes num tick (antes, as
    // variaveis locais da funcao de 1.700 linhas).
    struct Tick {
        const RagdollProfile3D& profile;
        const CharacterLocomotionAnimations3D& animations;
        const DefaultLocomotionInput3D& input;
        const RagdollLinkDefinition3D* rootLink = nullptr;
        float dt {};
        std::size_t links {};
        float viewYaw {};
        Vec3 viewForward {};
        Vec3 velocity {};
        float speed {};
        float touchdown {};
        Vec3 requested {};
        Vec3 travel {};
        std::array<float, GaitCount> wanted {};
        float sprint {};
        float shapeTotal {};
        float total {};
        std::size_t dominant {};
        float upperIdleWeight {};
        Quaternion yaw {};
        Vec3 forward {};
        Vec3 left {};
        float rate {};
        float strideScale {};
        bool seedTick {};
        float landTrunk {};
        float landUpper {};
        float landHead {};
        Vec3 rootOffset {};
        std::array<float, 2> contact {};
        float ground {};
        Quaternion clipRootRotation {};
        float curveAcceleration {};
        float curveShare {};
        Vec3 feet {};
        Quaternion rootOrientation {};
        Vec3 rootPosition {};
        std::array<bool, 2> gestureFeet {};
        Quaternion rootInverse {};
    };
    void stepTravel(Tick& t);
    bool stepTackle(Tick& t);
    void stepGait(Tick& t);
    void stepAir(Tick& t);
    void stepPose(Tick& t);
    void stepLayers(Tick& t);
    void stepFeet(Tick& t);
    void stepExits(Tick& t);
    void stepReadout(Tick& t);
    [[nodiscard]] float gaitAmplitudeOf(std::size_t g) const;

    DefaultLocomotionSettings3D m_settings;
    DefaultLocomotionOutput3D m_output;
    std::array<Gait, GaitCount> m_gaits {};
    // Os clipes de arco (frente esquerda, frente direita, costas direita) e o
    // pouso do pe esquerdo deles.
    std::array<const AnimationClip3D*, 3> m_arcs {};
    std::array<float, 3> m_arcTouchdown {};
    // Carrinhos: o pe com que cada um comeca apoiado e a velocidade do
    // comeco dele (m/s).
    std::array<const AnimationClip3D*, 2> m_tackles {};
    std::array<int, 2> m_tackleSide { -1, -1 };
    std::array<float, 2> m_tackleClipSpeed {};
    // O lado e o deslize de cada carrinho, o giro da camera (rad/s,
    // filtrado) e quantos carrinhos ja (o acaso).
    std::array<TackleClipShape3D, 2> m_tackleShape {};
    float m_viewTurnRate = 0.0f;
    float m_previousViewYaw = 0.0f;
    bool m_previousViewValid = false;
    std::uint32_t m_tackleCount = 0;
    // Carrinho aereo (a voadora; pedido do usuario, 05/10 noite): no ar o
    // corpo vai a pose do carrinho no comeco da fase deitada (perna esticada
    // a frente, tronco para tras), no rumo do voo; no pouso o deslize do
    // carrinho continua dali. O clipe, o tempo da pose (anda do mergulho ate
    // a fase deitada, no ritmo do clipe), o rumo e o peso.
    const AnimationClip3D* m_airTackleClip = nullptr;
    float m_airTackleTime = 0.0f;
    float m_airTackleYaw = 0.0f;
    float m_airTackleWeight = 0.0f;
    // A mistura para a pose do ar (s): mais curta pedida perto do pouso.
    float m_airTackleBlendSeconds = 0.12f;
    // Uma voadora por voo; e se o carrinho em andamento veio dela.
    bool m_airTackleSpent = false;
    bool m_tackleFromAir = false;
    float m_airTackleElapsed = 0.0f;
    // Onde a pose do ar chega no clipe (s): o comeco da fase deitada; e em
    // quanto tempo (s) ela vai do mergulho ate la.
    float m_airTackleTarget = 0.0f;
    float m_airTackleDiveSeconds = 0.25f;
    // O pouso dela pela inercia: juntas e raiz seguem com a velocidade do voo
    // ate o deslize (a mistura de entrada partia de uma foto parada e a pelve
    // caia ~0,6 m de uma vez). A pose do ar no tick anterior (a velocidade).
    PoseInertializer3D m_landTackleInertia;
    std::vector<Vec3> m_airPreviousCoordinates;
    Vec3 m_airPreviousRoot;
    Quaternion m_airPreviousOrientation;
    // O tempo do clipe em que o carrinho em andamento comecou (a entrada
    // mistura a partir dali: no aereo, o deslize comeca no meio do clipe).
    float m_tackleEnterStart = 0.0f;
    // O deslize comum (tackleEqualSlide): a fase deitada media (s), a fracao
    // que a velocidade cai nela, a velocidade de entrada, o rumo do caminho e
    // a posicao no plano (relativa a m_tackleStart).
    float m_tackleCommonLowSeconds = 0.0f;
    float m_tackleCommonDrop = 0.0f;
    float m_tackleEntrySpeedSlide = 0.0f;
    Vec3 m_tacklePathDirection { 1.0f, 0.0f, 0.0f };
    Vec3 m_tacklePlanar;
    const AnimationClip3D* m_preparedFor = nullptr;
    // (A chave completa do preparo: todos os clipes, nao so o de andar.)
    CharacterLocomotionAnimations3D m_preparedAnimations {};
    std::array<std::size_t, 2> m_feet {};
    std::array<std::size_t, 2> m_hands {};
    // Setor (0 frente, 1 frente-esquerda, 2 esquerda ... 7 frente-direita;
    // o rumo dele e setor * 45 graus) e os pesos.
    int m_sector = 0;
    std::array<float, GaitCount> m_weights {};
    std::array<float, GaitCount> m_shape {};
    std::array<float, GaitCount> m_shapeRates {};
    float m_moving = 0.0f;
    float m_movingRate = 0.0f;
    // O corpo de cima (tronco, bracos, cabeca), mais devagar.
    std::array<float, GaitCount> m_upperShape {};
    std::array<float, GaitCount> m_upperShapeRates {};
    std::array<float, GaitCount> m_upperWeights {};
    float m_upperMoving = 0.0f;
    float m_upperMovingRate = 0.0f;
    // Arrancada do parado: o gesto do idle_to_sprint (tempo; < 0 parado) e
    // se e para o sprint. O afundar do peso (m, para baixo) e a velocidade.
    float m_startTime = -1.0f;
    bool m_startSprint = false;
    float m_dip = 0.0f;
    float m_dipRate = 0.0f;
    // Links do corpo de cima (tronco, bracos, cabeca).
    std::vector<char> m_upperLinks;
    float m_idleWeight = 1.0f;
    float m_phase = 0.0f;
    float m_strides = 0.0f;
    float m_idleTime = 0.0f;
    // Rumo da pelve (e a velocidade dele), olhar (coluna e cabeca, e as
    // velocidades), giro parado em andamento; o rumo do movimento em relacao
    // a pelve.
    float m_yaw = 0.0f;
    float m_yawRate = 0.0f;
    float m_torsoTwistRate = 0.0f;
    float m_headTurnRate = 0.0f;
    float m_localTravel = 0.0f;
    // A trava de cada pe: ativa, ja usada neste apoio (soltou por distancia),
    // onde (mundo) e o peso (1 travado, caindo ate 0 ao soltar).
    struct FootLock {
        bool active = false;
        bool spent = false;
        Vec3 position;
        float weight = 0.0f;
    };
    std::array<FootLock, 2> m_footLocks {};
    // Parando: a fase do cruzamento em que o passo termina (-1 sem parada).
    float m_stopTarget = -1.0f;
    // Quanto do preparo do pouso vale neste tick (0..1) e o rumo e o angulo
    // da perna do ultimo instante no ar (depois do toque ele sai aos poucos).
    float m_landingPrep = 0.0f;
    float m_landingPrepRate = 0.0f;
    Vec3 m_landingDirection;
    float m_landingBeta = 0.0f;
    float m_landingRunningOn = 0.0f;
    // A perna que vai ao chao seguindo correndo (a mais a frente no rumo no
    // comeco do preparo; -1 nenhuma).
    int m_landingLeadSide = -1;
    // A altura que a vida do tronco soma a pelve neste tick (m) e o quanto
    // cada pe desconta dela (o apoio numa mola: sem degrau na troca).
    float m_trunkHeight = 0.0f;
    std::array<float, 2> m_plantWeight {};
    std::array<float, 2> m_plantRate {};
    float m_torsoTwist = 0.0f;
    float m_headTurn = 0.0f;
    float m_upright = 1.0f;
    bool m_standTurning = false;
    bool m_started = false;
    // A semente (seed): aplicada no proximo update.
    bool m_seedPending = false;
    // A arrancada agachada: pedida (startCrouchSprint), o rumo e se corre; e
    // se o clipe em andamento (m_tackleClip) e ela, nao o carrinho.
    bool m_crouchStartPending = false;
    float m_crouchStartYaw = 0.0f;
    bool m_crouchStartSprint = false;
    bool m_crouchStarting = false;
    std::array<Vec3, 2> m_crouchPreviousFeet {};
    // Onde a arrancada termina (s no clipe: no pico da velocidade da raiz -
    // os ultimos quadros do arquivo freiam, da suavizacao na borda) e a
    // saida dela pela inercia (com a velocidade das juntas e da raiz; da foto
    // parada do fim, ele dava uma "travadinha" ao comecar a correr).
    float m_crouchExitTime = 0.0f;
    Quaternion m_crouchPreviousRoot;
    PoseInertializer3D m_exitInertia;
    // O tick de verdade no passe refeito da semente (ele roda com 0,1 ms; a
    // inercia anda o tick inteiro - senao a pelve parava um quadro).
    float m_exitInertiaDt = 0.0f;
    DefaultLocomotionSeed3D m_seed;
    // Caminho: velocidade, aceleracao, giro.
    Vec3 m_previousVelocity;
    Vec3 m_acceleration;
    float m_previousPathYaw = 0.0f;
    bool m_pathYawValid = false;
    float m_pathTurnRate = 0.0f;
    // Inclinacao (x lado, y frente) e a velocidade dela; a parte da coluna.
    Vec3 m_lean;
    Vec3 m_leanRate;
    float m_spineRoll = 0.0f;
    // Clipe de arco em uso (-1 nenhum) e o peso.
    int m_arcKind = -1;
    float m_arcBlend = 0.0f;
    // Giro dos pes na curva e a descida da pelve no terreno.
    std::array<float, 2> m_footYaw {};
    float m_terrainDrop = 0.0f;
    // Os degraus da capsula (ela sobe e desce o degrau de uma vez, no chao):
    // a parte do salto que a pelve ainda nao seguiu (m) e a velocidade dela,
    // numa mola - sem ela, o corpo guiado subia a ~8 m/s num tick e, indo ao
    // fisico ali, voava (plano v2, fase E). E a base do tick anterior.
    float m_stepOffset = 0.0f;
    float m_stepOffsetRate = 0.0f;
    // O pe no balanco por cima do degrau a frente: o quanto a sola sobe (m) e
    // a velocidade disso, por pe.
    std::array<float, 2> m_stepClear {};
    std::array<float, 2> m_stepClearRate {};
    float m_previousBase = 0.0f;
    bool m_previousBaseKnown = false;
    // No ar: o tempo, a velocidade vertical na saida e o peso do pulo.
    float m_airTime = 0.0f;
    float m_takeoffSpeed = 0.0f;
    float m_airWeight = 0.0f;
    float m_airRate = 0.0f;
    const AnimationClip3D* m_airClip = nullptr;
    float m_airPhase = 0.0f;
    float m_crouch = 0.0f;
    // Aterrissagem (landingAbsorb): o tempo desde o toque (s; < 0 nenhuma),
    // a intensidade (0..1), a duracao, a ultima velocidade de queda no ar,
    // quanto dela vale ainda (sai no ar), a descida do preparo no toque e a
    // descida da pelve agora (m).
    float m_landTime = -1.0f;
    float m_landIntensity = 0.0f;
    float m_landDuration = 0.4f;
    float m_fallSpeed = 0.0f;
    float m_landFade = 1.0f;
    float m_landingSink = 0.0f;
    // E o quanto os pes vao ao chao agora (0..1) e o pe mais baixo no toque
    // (que vai inteiro; correndo, o outro menos).
    float m_landPull = 0.0f;
    int m_landLeadSide = -1;
    float m_landRunning = 0.0f;
    // A descida do preparo no toque (m) e a parte dela que ainda vale: sai
    // junto com a compressao (as pernas e a coluna do preparo saem em 0,06 s).
    float m_landPrepDrop = 0.0f;
    float m_landPrepCarry = 0.0f;
    // Carrinho: clipe, tempo, ritmo, escala do caminho, rumo, comeco, a pose
    // de onde entra e a de onde sai (raiz em relacao a capsula), intervalo.
    const AnimationClip3D* m_tackleClip = nullptr;
    float m_tackleTime = 0.0f;
    float m_tackleRate = 1.0f;
    float m_tackleScale = 1.0f;
    float m_tackleYaw = 0.0f;
    Vec3 m_tackleStart;
    Vec3 m_tacklePrevious;
    float m_tackleCooldown = 0.0f;
    float m_tackleExit = 1.0f;
    std::vector<Vec3> m_tackleFrom;
    Vec3 m_tackleFromRoot;
    Vec3 m_tackleEntryVelocity;
    Quaternion m_tackleFromOrientation;
    std::vector<Vec3> m_exitFrom;
    Vec3 m_exitRootOffset;
    Quaternion m_exitRootOrientation;
    std::vector<Vec3> m_coordinates;
    // O corpo do chute com o peso suavizado (a pose de bracos e tronco entra
    // e sai sem salto) e o ultimo pedido dela.
    // As pernas e bracos para o gesto com a bola (IK analitico).
    BallGestureLegs3D m_gestureLegs;
};

} // namespace MatterEngine
