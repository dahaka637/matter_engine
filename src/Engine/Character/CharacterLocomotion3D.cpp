#include "Engine/Character/CharacterLocomotion3D.hpp"
#include "Engine/Character/DefaultLocomotion3D.hpp"
#include "Engine/Character/CharacterFootGeometry3D.hpp"
#include "Engine/Character/RagdollPoseKinematics3D.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib> // TEMPTUNE
#include <limits>

namespace MatterEngine {
namespace {
// TEMPTUNE (remover): leitura de ambiente para a calibragem.
inline float tuneTemp(const char* name, float fallback) {
    const char* value = std::getenv(name);
    return value != nullptr ? std::strtof(value, nullptr) : fallback;
}
const float PelvisCurveLeadSeconds = tuneTemp("T_PelvisLead", 0.25f);  // TEMPTUNE
const float PelvisCurveLeadMaximum = tuneTemp("T_PelvisLeadMax", 0.35f);  // TEMPTUNE
const float PelvisCurveFeedForward = tuneTemp("T_PelvisFF", 1.0f);  // TEMPTUNE
// Curva fechada: correndo para a frente, a pelve segue o caminho do corpo e
// so se adianta para o pedido ate este angulo (rad), devagar e no sprint.
const float PathLeadSlow = tuneTemp("T_PathLeadSlow", 1.2f);  // TEMPTUNE
// Levantar: escala das ajudas externas (pelve, peito, endireitar) e teto da
// forca dos musculos (multiplo da nominal do perfil).
const float GetUpAssistScale = tuneTemp("T_GetUpAssist", 1.0f);  // TEMPTUNE
// Levantar mais rapido (pedido do usuario, 05/10 noite): o clipe anda ate
// GetUpSpeed vezes o ritmo dele (o corpo ainda o segura se ficar para tras)
// e deitado ele espera no minimo GetUpSettleSeconds. Com as teclas de
// andar, mais rapido ainda (GetUpHurrySpeed; com Shift, GetUpSprintSpeed),
// espera menos e sai andando/correndo para onde elas pedem, em qualquer
// direcao: termina o clipe antes (quase em pe) ja com o rumo pedido e a
// arrancada da locomocao.
// Fase final do levantar (de pe, o clipe quase reto): teto (vezes a massa)
// e ganho a mais na ajuda que ergue o peito.
const float GetUpChestLateCap = tuneTemp("T_GetUpChestCap", 5.0f);  // TEMPTUNE
const float GetUpLateHeight = tuneTemp("T_GetUpLateHeight", 0.75f);  // TEMPTUNE (m)
// Fim do levantar: o quanto a coluna endireita (0..1) e a parte da flexao
// do clipe que fica.
const float GetUpEndStraighten = tuneTemp("T_GetUpStraighten", 1.0f);  // TEMPTUNE
// E o teto (vezes a massa) a mais na ajuda que endireita a pelve ali.
const float GetUpPelvisLateCap = tuneTemp("T_GetUpPelvisCap", 6.0f);  // TEMPTUNE
const float GetUpPelvisLateGain = tuneTemp("T_GetUpPelvisGain", 2.0f);  // TEMPTUNE
// Perto da altura de pe, a ajuda de subida tambem freia (fracao do teto).
const float GetUpLiftBrake = tuneTemp("T_GetUpLiftBrake", 1.0f);  // TEMPTUNE
const float GetUpEndSpineShare = tuneTemp("T_GetUpSpineShare", 0.25f);  // TEMPTUNE
const float GetUpChestLateGain = tuneTemp("T_GetUpChestGain", 1.5f);  // TEMPTUNE
const bool GetUpHurryEnabled = tuneTemp("T_GetUpHurryOn", 1.0f) > 0.5f;  // TEMPTUNE
// Carrinho fisico: o clipe pela camera/corrida e o mesmo deslize (05/10).
const bool TackleChoiceByView = tuneTemp("T_TackleChoice", 1.0f) > 0.5f;  // TEMPTUNE
const float GetUpSpeed = tuneTemp("T_GetUpSpeed", 1.0f);  // TEMPTUNE
const float GetUpHurrySpeed = tuneTemp("T_GetUpHurry", 3.5f);  // TEMPTUNE
const float GetUpSprintSpeed = tuneTemp("T_GetUpSprint", 3.5f);  // TEMPTUNE
const float GetUpSettleSeconds = tuneTemp("T_GetUpSettle", 0.55f);  // TEMPTUNE
// Deitado: o quanto o corpo precisa estar parado para comecar (vezes os
// limites de velocidade do centro e do membro mais rapido).
const float GetUpCalmScale = tuneTemp("T_GetUpCalm", 1.6f);  // TEMPTUNE
// A subida da pelve no clipe acelerado fica no maximo nisto (m/s).
const float GetUpMaxRise = tuneTemp("T_GetUpMaxRise", 0.9f);  // TEMPTUNE
// E as juntas das pernas no clipe acelerado, no maximo nisto (rad/s).
const float GetUpMaxLegSpeed = tuneTemp("T_GetUpMaxLeg", 5.0f);  // TEMPTUNE
const float GetUpHurrySettleSeconds = tuneTemp("T_GetUpHurrySettle", 0.25f);  // TEMPTUNE
// Saindo com pressa: o clipe termina a partir desta fracao, com o corpo
// quase em pe (pelve endireitada alem de GetUpHurryUpright).
const float GetUpHurryExit = tuneTemp("T_GetUpHurryExit", 0.85f);  // TEMPTUNE
const float GetUpHurryUpright = tuneTemp("T_GetUpHurryUpright", 0.85f);  // TEMPTUNE
// Com pressa e o modo padrao (laboratorio): a partir de quando o corpo esta
// sobre os pes (fracao do clipe, altura da pelve, inclinacao dela em rad) a
// locomocao padrao pode assumir ja andando - telemetry.getUpHandoffReady.
const float GetUpHandoffProgress = tuneTemp("T_GetUpHandoffProg", 0.35f);  // TEMPTUNE
const float GetUpHandoffHeight = tuneTemp("T_GetUpHandoffHeight", 0.50f);  // TEMPTUNE
const float GetUpHandoffTilt = tuneTemp("T_GetUpHandoffTilt", 1.05f);  // TEMPTUNE
const float GetUpHandoffChestTilt = tuneTemp("T_GetUpHandoffChest", 1.66f);  // TEMPTUNE
// (Os pes a no maximo isto a frente/atras da pelve, m: de bruços o levantar
// passa por um afundo, um pe bem a frente.)
const float GetUpHandoffFeet = tuneTemp("T_GetUpHandoffFeet", 0.70f);  // TEMPTUNE
const float GetUpTorqueCap = tuneTemp("T_GetUpTorqueCap", 3.0f);  // TEMPTUNE
// Carrinho (04/10). TEMPTUNE: liga; velocidade minima na direcao do olhar
// (m/s); crossfade da corrida para o clipe (s); preparo (quanto a pelve
// baixa esperando o pouso, m; prazo maximo do preparo, s); deslize (o atrito
// do corpo no chao enquanto desliza, x o do material; a pelve do clipe abaixo
// disto e deslize, m; a pose do deslize fica enquanto ele anda mais rapido
// que isto, m/s); e a volta para a locomocao (pelve do clipe acima desta
// parte da altura de pe, depois do deslize, com o corpo de pe).
const bool TackleEnabled = tuneTemp("T_Tackle", 1.0f) > 0.5f;
const float TackleMinSpeed = tuneTemp("T_TackleMin", 2.5f);
const float TackleTransition = tuneTemp("T_TackleBlend", 0.12f);
const float TackleGatherDrop = tuneTemp("T_TackleGather", 0.08f);
// Preparo: o passo de pouso um pouco mais longo (firmar o pe a frente) e
// quanto do tronco, bracos e cabeca ja vai para a pose do comeco do clipe.
const float TacklePlantStride = tuneTemp("T_TacklePlantStride", 0.10f);
// TEMPTUNE: troca de dono do pe no ar (planejador <-> passada): o alvo novo
// entra misturado a partir do ultimo neste tempo (s; 0 desliga).
const float GoalHandoffSeconds = tuneTemp("T_GoalHandoff", 0.12f);
// Repulsao do pe pela bola: folga entre a caixa do pe e a bola (m).
const float BallRepelMargin = tuneTemp("T_BallRepelMargin", 0.025f);
// TEMPTUNE: giro parado pelo clipe (ajuda cheia; pedido do usuario, 04/10):
// liga com o giro pedido acima de ClipTurnEnter (rad); o ciclo do clipe dura o
// giro sobre ClipTurnSpeed (rad/s), no minimo ClipTurnMinSeconds e no maximo a
// duracao do clipe; as pernas entram/saem em ClipTurnBlendIn/Out (s).
const bool ClipTurnEnabled = tuneTemp("T_ClipTurn", 0.0f) > 0.5f;
const float ClipTurnEnter = tuneTemp("T_ClipTurnEnter", 0.30f);
const float ClipTurnSpeed = tuneTemp("T_ClipTurnSpeed", 6.0f);
const float ClipTurnMinSeconds = tuneTemp("T_ClipTurnMin", 0.24f);
const float ClipTurnBlendIn = tuneTemp("T_ClipTurnBlendIn", 0.03f);
const float ClipTurnBlendOut = tuneTemp("T_ClipTurnBlendOut", 0.12f);
// TEMPTUNE: corpo do chute (rad no chute cheio): giro do tronco, para tras no
// armado, por cima da bola batendo, para o lado do apoio.
const float KickTrunkTwist = tuneTemp("T_KickTwist", 0.30f);
const float KickTrunkBack = tuneTemp("T_KickTrunkBack", 0.12f);
const float KickTrunkOver = tuneTemp("T_KickTrunkOver", 0.12f);
const float KickTrunkSide = tuneTemp("T_KickTrunkSide", 0.14f);
// Bracos do chute pela forca (04/10): abaixo de KickArmFloor nao entram, em
// KickArmFull o desenho inteiro.
const float KickArmFloor = tuneTemp("T_KickArmFloor", 0.25f);
const float KickArmFull = tuneTemp("T_KickArmFull", 0.95f);
const float TacklePreBlend = tuneTemp("T_TacklePreBlend", 0.6f);
const float TacklePrepMax = tuneTemp("T_TacklePrep", 0.40f);
// Forca do carrinho pela velocidade: no chao ele desliza D = a * v^p (v a
// velocidade ao tocar o chao; trote ~1,1 m, sprint ~4 m) - o atrito do corpo
// e o que da essa distancia (mu = v^2 / (2 g D), sobre o atrito de base do
// corpo na grama, ~0,93 medido).
const float TackleSlideA = tuneTemp("T_TackleSlideA", 0.30f);
const float TackleSlideP = tuneTemp("T_TackleSlideP", 1.3f);
const float TackleBaseFriction = tuneTemp("T_TackleBaseMu", 0.93f);
// Impulso: na saida para o mergulho, com o pe de apoio no chao, a perna
// empurra o corpo na direcao do carrinho ate esta parte da velocidade de
// entrada (o teto da forca, x peso).
const float TackleImpulseShare = tuneTemp("T_TackleImpulse", 1.15f);
const float TackleImpulseForce = tuneTemp("T_TackleImpulseForce", 0.8f);
const float TackleSlideHeight = tuneTemp("T_TackleSlideH", 0.55f);
const float TackleSlideRelease = tuneTemp("T_TackleSlideV", 2.2f);
const float TackleHandoffHeight = tuneTemp("T_TackleHandoff", 0.74f);
// Atitude da pelve no carrinho (torque para a inclinacao do clipe; 0 sem).
// 1,5 desde o jogador montado (09/10): na saida do deslize o centro de massa
// fica mais a frente dos pes que no corpo antigo, e o tronco assentava onde
// o torque iguala o tombamento - ~36 graus com 1,0, alem do limite de "de
// pe" (35): os 5 carrinhos em sprint viravam queda (o antigo assentava em
// ~30). Com 1,5, nenhum; o deslize no chao fica o mesmo.
const float TackleAttitude = tuneTemp("T_TackleAttitude", 1.5f);
// Ajuda para levantar do deslize (altura, para cima; propria do carrinho).
const float TackleRecoveryAssist = tuneTemp("T_TackleRecovery", 1.0f);
// Levantar (04/10): rolar para ficar de bruços antes de levantar, e os pes do
// equilibrio no levantar de bruços. TEMPTUNE.
const bool GetUpRollEnabled = tuneTemp("T_GetUpRoll", 1.0f) > 0.5f;
const bool GetUpRollFromBack = tuneTemp("T_GetUpRollBack", 0.0f) > 0.5f;
// De lado: o "para a frente" do peito a menos disto da horizontal (|z|).
const float GetUpRollSideFacing = tuneTemp("T_GetUpRollSide", 0.55f);
// Ajuda do rolar (N.m por kg, vezes o slider da ajuda para levantar).
const float GetUpRollTorque = tuneTemp("T_GetUpRollTorque", 2.5f);
const float GetUpRollTimeout = tuneTemp("T_GetUpRollTime", 3.0f);
const bool GetUpFootworkEnabled = tuneTemp("T_GetUpFeet", 0.0f) > 0.5f;
// Progresso do clipe de bruços: pes do equilibrio (quatro apoios) e o ponto
// em que as maos soltam (o clipe espera ali com o corpo a frente dos pes).
const float GetUpFootworkFrom = tuneTemp("T_GetUpFeetFrom", 0.36f);
const float GetUpHandRelease = tuneTemp("T_GetUpHandRelease", 0.46f);
// Do tornozelo a ponta do pe (0,22 m no modelo), com folga.
const float GetUpToeAhead = tuneTemp("T_GetUpToe", 0.18f);
const float GetUpStepMargin = tuneTemp("T_GetUpStepMargin", 0.04f);
// O tornozelo pousa atras do centro de massa (o pe fica embaixo dele).
const float GetUpStepAhead = tuneTemp("T_GetUpStepAhead", -0.06f);
const float GetUpStepWidth = tuneTemp("T_GetUpStepWidth", 0.11f);
const float GetUpStepSeconds = tuneTemp("T_GetUpStepSec", 0.35f);
const float GetUpStepLift = tuneTemp("T_GetUpStepLift", 0.10f);
const float GetUpStepMax = tuneTemp("T_GetUpStepMax", 0.45f);
const float GetUpHoldMax = tuneTemp("T_GetUpHoldMax", 3.0f);
// A pelve do IK das pernas presas se adianta para a do clipe ate isto
// (altura, m; giro, rad): o descompasso e o empurrao das pernas.
const float GetUpRootLead = tuneTemp("T_GetUpLead", 0.12f);
const float GetUpTurnLead = tuneTemp("T_GetUpTurn", 0.12f);
// Mao de apoio: o clipe pede a mao no chao (punho abaixo disto do chao); o
// ponto e a frente e para fora do ombro; ida ao chao e volta ao clipe (s).
const bool GetUpHandsEnabled = tuneTemp("T_GetUpHands", 0.0f) > 0.5f;
const float GetUpHandClipDown = tuneTemp("T_GetUpHandDown", 0.10f);
const float GetUpHandForward = tuneTemp("T_GetUpHandFwd", 0.12f);
const float GetUpHandOutward = tuneTemp("T_GetUpHandOut", 0.08f);
const float GetUpHandWristHeight = 0.05f;
const float GetUpHandReachSeconds = tuneTemp("T_GetUpHandReach", 0.30f);
const float GetUpHandReleaseSeconds = tuneTemp("T_GetUpHandRelease", 0.25f);
// Levantar pelo corpo (BodilyGetUp3D) no lugar do clipe com ajuda.
const bool GetUpBodily = tuneTemp("T_GetUpBodily", 0.0f) > 0.5f;  // TEMPTUNE
// Postura mais ereta: o quanto a pelve-alvo sobe (m) parado, trotando e no
// sprint - com os pes travados no chao, o IK estica os joelhos.
const float PostureLiftIdle = tuneTemp("T_LiftIdle", 0.0f);  // TEMPTUNE
const float PostureLiftRun = tuneTemp("T_LiftRun", 0.0f);  // TEMPTUNE
const float PostureLiftSprint = tuneTemp("T_LiftSprint", 0.0f);  // TEMPTUNE
const float StanceLegStiffness = tuneTemp("T_StanceStiff", 0.55f);  // TEMPTUNE
const float ReachComfort = tuneTemp("T_ReachComfort", 0.985f);  // TEMPTUNE
const bool ContactPhase = tuneTemp("T_ContactPhase", 1.0f) > 0.5f;  // TEMPTUNE
const float ContactWaitSeconds = tuneTemp("T_ContactWait", 0.08f);  // TEMPTUNE
const float ContactTrailCap = tuneTemp("T_ContactTrail", 0.55f);  // TEMPTUNE
const float ContactMaxLead = tuneTemp("T_ContactLead", 0.25f);  // TEMPTUNE
const float ContactCatchUp = tuneTemp("T_ContactCatchUp", 0.5f);  // TEMPTUNE
const float ContactEarlyProgress = tuneTemp("T_ContactEarly", 0.8f);  // TEMPTUNE
const float ContactEarlyBoost = tuneTemp("T_ContactEarlyBoost", 1.6f);  // TEMPTUNE
const bool ContactLandReach = tuneTemp("T_ContactReach", 1.0f) > 0.5f;  // TEMPTUNE
const bool ContactPaw = tuneTemp("T_ContactPaw", 0.0f) > 0.5f;  // TEMPTUNE
// v2: salto correndo pela passada aberta do ciclo (ver a decolagem).
const bool RunLeap = tuneTemp("T_RunLeap", 1.0f) > 0.5f;  // TEMPTUNE
const float RunLeapMinimumSpeed = tuneTemp("T_RunLeapSpeed", 4.0f);  // TEMPTUNE
const float RunLeapMinimumAssist = tuneTemp("T_RunLeapAssist", 0.45f);  // TEMPTUNE
const float RunLeapMaximumAssist = tuneTemp("T_RunLeapAssistMax", 0.85f);  // TEMPTUNE
const float RunLeapHold = tuneTemp("T_RunLeapHold", 0.3f);  // TEMPTUNE
const float ContactLandOffset = tuneTemp("T_ContactLand", 0.0f);  // TEMPTUNE
const bool ContactRateSmooth = tuneTemp("T_RateSmooth", 1.0f) > 0.5f;  // TEMPTUNE
const float ContactRateDownSeconds = tuneTemp("T_RateDown", 0.015f);  // TEMPTUNE
const float ContactRateUpSeconds = tuneTemp("T_RateUp", 0.06f);  // TEMPTUNE
const float ArmPhaseSmoothSeconds = tuneTemp("T_ArmPhase", 0.08f);  // TEMPTUNE
const float ArmPhaseLockSeconds = tuneTemp("T_ArmLock", 0.25f);  // TEMPTUNE
const float ContactPawMaximum = tuneTemp("T_ContactPawMax", 0.35f);  // TEMPTUNE
const float ContactLandRelease = tuneTemp("T_ContactReachRelease", 0.12f);  // TEMPTUNE
const float ContactLandReachMax = tuneTemp("T_ContactReachMax", 0.02f);  // TEMPTUNE
const float TouchHoldSeconds = tuneTemp("T_TouchHold", 0.12f);  // TEMPTUNE
const float TouchMinStanceSeconds = tuneTemp("T_TouchMinStance", 0.08f);  // TEMPTUNE
const bool TouchHoldSteadyOnly = tuneTemp("T_TouchSteady", 1.0f) > 0.5f;  // TEMPTUNE
const bool ReleaseBySoleClearance = tuneTemp("T_SoleRelease", 1.0f) > 0.5f;  // TEMPTUNE
const float PushStiffness = tuneTemp("T_PushStiff", 2.0f);  // TEMPTUNE
const float PushAfterLandingSeconds = tuneTemp("T_PushLand", 1.0f);  // TEMPTUNE
const float LandingLegTone = tuneTemp("T_LandTone", 1.0f);  // TEMPTUNE
const float LandRunYield = tuneTemp("T_LandRunYield", 1.0f);  // TEMPTUNE (x agachamento previsto; 0 desliga)
const float LandRunDepth = tuneTemp("T_LandRunDepth", 0.5f);  // TEMPTUNE
const float LandingLegToneSeconds = tuneTemp("T_LandToneSec", 0.6f);  // TEMPTUNE
const float ThrustGain = tuneTemp("T_Thrust", 0.0f);  // TEMPTUNE
const float ThrustMaximum = tuneTemp("T_ThrustMax", 0.15f);  // TEMPTUNE
const float ThrustApexMargin = tuneTemp("T_ThrustMargin", 0.0f);  // TEMPTUNE
const float StanceVerticalDamping = tuneTemp("T_StanceDamp", 0.1f);  // TEMPTUNE
const bool StanceDampFree = tuneTemp("T_StanceDampFree", 1.0f) > 0.5f;  // TEMPTUNE
const float StanceDampCurveSpanEnv = tuneTemp("T_StanceDampCurve", 0.2f);  // TEMPTUNE (rad; 0 = sempre)
const float StanceRootBandEnv = tuneTemp("T_StanceBand", 0.0f);  // TEMPTUNE (m; 0 desliga)
const bool TouchPlantInCurves = tuneTemp("T_TouchPlantCurve", 0.0f) > 0.5f;  // TEMPTUNE
const float CurveNarrow = tuneTemp("T_CurveNarrow", 0.0f);  // TEMPTUNE
const float CurveSpineShare = tuneTemp("T_CurveSpine", 0.45f);  // TEMPTUNE
const bool VarietyAdditive = tuneTemp("T_VarietyAdd", 0.0f) > 0.5f;  // TEMPTUNE
const float CurveSpineSign = tuneTemp("T_CurveSpineSign", 1.0f);  // TEMPTUNE
const float PushBehindMeters = tuneTemp("T_PushBehind", 0.05f);  // TEMPTUNE
const bool PushVelocity = tuneTemp("T_PushVel", 0.0f) > 0.5f;  // TEMPTUNE
const bool PushRamp = tuneTemp("T_PushRamp", 1.0f) > 0.5f;  // TEMPTUNE
const float ReachAnticipateScale = tuneTemp("T_ReachAnticipate", 1.0f);  // TEMPTUNE
const float ReachPelvisOmega = tuneTemp("T_ReachOmega", 18.0f);  // TEMPTUNE
const float SwingReachFlatCap = tuneTemp("T_SwingReachCap", -1.0f);  // TEMPTUNE (m; <0 desliga)
const float PathLeadFast = tuneTemp("T_PathLeadFast", 0.45f);  // TEMPTUNE
const float PathLeadFastSpeed = tuneTemp("T_PathLeadFastSpeed", 6.5f);  // TEMPTUNE
// Curva fechada: teto de velocidade a(lateral)/taxa de giro do pedido.
const float CurveSpeedAcceleration = tuneTemp("T_CurveSpeedAc", 10.0f);  // TEMPTUNE
const float CurveSpeedAccelerationLow = tuneTemp("T_CurveSpeedAcLow", 7.0f);  // TEMPTUNE
// A pelve limitada ao caminho depois do adiantamento da curva (o total).
const bool PathLeadAfterCurve = tuneTemp("T_PathLeadAfter", 1.0f) > 0.5f;  // TEMPTUNE
const float CurveSpeedMinimum = tuneTemp("T_CurveSpeedMin", 2.5f);  // TEMPTUNE
const float CurveSpeedRecovery = tuneTemp("T_CurveSpeedRec", 6.0f);  // TEMPTUNE
const float RequestTurnFilterSeconds = tuneTemp("T_ReqTurnFilter", 0.25f);  // TEMPTUNE
const float RequestTurnRateClamp = tuneTemp("T_ReqTurnClamp", 6.0f);  // TEMPTUNE
// Correndo, o pe apoiado segura o rumo do pouso (a pelve gira por cima dele
// pelo quadril) em vez de girar com ela no chao.
const bool GaitStanceHoldYaw = tuneTemp("T_StanceHoldYaw", 0.0f) > 0.5f;  // TEMPTUNE
// Quase-queda em movimento: deteccao (inclinacao para a frente alem da de
// regime, subindo; freada do corpo alem da pedida) e a resposta.
// Arrancada: o ciclo comeca com o pe de tras saindo do chao (fase do clipe
// em que o contato dele cai), nao na fase 0.
const bool StartFromRearFoot = tuneTemp("T_StartRear", 0.0f) > 0.5f;  // TEMPTUNE
// Perna da passada no ar pela altura da pelve de verdade quando ela esta
// abaixo da de referencia (fracao; 0 desliga).
const float SwingFromLowBody = tuneTemp("T_SwingLow", 0.7f);  // TEMPTUNE
const float SwingFromLowBodyRelease = tuneTemp("T_SwingLowRelease", 0.15f);  // TEMPTUNE
// ... e pela inclinacao de verdade da pelve (so a inclinacao, sem o rumo).
const float SwingFromBodyTilt = tuneTemp("T_SwingTilt", 0.0f);  // TEMPTUNE
const float SwingLowLandingStart = tuneTemp("T_SwingLowLand", 0.65f);  // TEMPTUNE
const float SwingLowLandingSpan = tuneTemp("T_SwingLowLandSpan", 0.3f);  // TEMPTUNE
// O pe da passada que o clipe solta ainda carregando o corpo (o outro sem
// apoio) espera descarregar, ate este tempo (s).
const float LiftAfterUnloadSeconds = tuneTemp("T_LiftUnload", 0.0f);  // TEMPTUNE
const float LiftAfterUnloadShare = tuneTemp("T_LiftUnloadShare", 0.25f);  // TEMPTUNE
// Folga minima e ponta recolhida do trote tambem no sprint em curva (pela
// aceleracao lateral real do corpo, m/s2, de onde comeca e em quanto chega).
const float SprintCurveClearStart = tuneTemp("T_SprintClear", 2.0f);  // TEMPTUNE
const float SprintCurveClearSpan = tuneTemp("T_SprintClearSpan", 4.0f);  // TEMPTUNE
const float CurveClearEarly = tuneTemp("T_ClearEarly", 0.14f);  // TEMPTUNE
const float StartPhaseLead = tuneTemp("T_StartLead", 0.0f);  // TEMPTUNE
// Arrancada: o pe que o ciclo ja quer no ar, mas que ainda carrega o corpo,
// so sai quando descarrega (ate este tempo, s).
const float StartLoadHoldSeconds = tuneTemp("T_StartHold", 0.0f);  // TEMPTUNE
const float StartLoadHoldShare = tuneTemp("T_StartHoldShare", 0.30f);  // TEMPTUNE
const bool StumbleDetection = tuneTemp("T_Stumble", 1.0f) > 0.5f;  // TEMPTUNE
const bool StumbleAgainstReference = tuneTemp("T_StumbleRef", 1.0f) > 0.5f;  // TEMPTUNE
const bool StumbleFullResponse = tuneTemp("T_StumbleFull", 0.0f) > 0.5f;  // TEMPTUNE
// Passo de quase-queda pelo planejador: risco que dispara, quanto a frente do
// corpo no toque o pe pousa (m), duracao do balanco (s), quanto a pelve desce
// (m) e o intervalo minimo entre dois (s).
const float StumbleStepTrigger = tuneTemp("T_StumbleStep", 0.0f);  // TEMPTUNE
const float StumbleStepAhead = tuneTemp("T_StumbleAhead", 0.20f);  // TEMPTUNE
const float StumbleStepSwing = tuneTemp("T_StumbleStepSwing", 0.22f);  // TEMPTUNE
const float StumbleStepDrop = tuneTemp("T_StumbleStepDrop", 0.10f);  // TEMPTUNE
const float StumbleStepCooldown = tuneTemp("T_StumbleCooldown", 0.5f);  // TEMPTUNE
// Arrancada: o primeiro passo e do planejador (pe de tras, pousando a frente
// do corpo), com a pelve descendo e a ajuda dosada (teto de velocidade que
// cresce: StartSpeedBase + StartSpeedRate t, ate o passo pousar).
const bool StartStepEnabled = tuneTemp("T_StartStep", 0.0f) > 0.5f;  // TEMPTUNE
const float StartStepAhead = tuneTemp("T_StartAhead", 0.20f);  // TEMPTUNE
const float StartStepSwing = tuneTemp("T_StartSwing", 0.28f);  // TEMPTUNE
const float StartStepDrop = tuneTemp("T_StartDrop", 0.06f);  // TEMPTUNE
const float StartSpeedBase = tuneTemp("T_StartSpeedBase", 0.8f);  // TEMPTUNE
const float StartSpeedRate = tuneTemp("T_StartSpeedRate", 6.0f);  // TEMPTUNE
const float StartPushSeconds = tuneTemp("T_StartPush", 0.12f);  // TEMPTUNE
// Arrancada pelo ciclo, com a ajuda dosada nos primeiros instantes (s; 0
// desliga): o teto StartSpeedBase + StartSpeedRate t.
const float StartDoseSeconds = tuneTemp("T_StartDose", 0.30f);  // TEMPTUNE
const float StartStepLift = tuneTemp("T_StartLift", 0.14f);  // TEMPTUNE
const float StumblePitchStart = tuneTemp("T_StumblePitch", 0.15f);  // TEMPTUNE
const float StumblePitchSpan = tuneTemp("T_StumblePitchSpan", 0.15f);  // TEMPTUNE
const float StumbleRateStart = tuneTemp("T_StumbleRate", 1.0f);  // TEMPTUNE
const float StumbleRateSpan = tuneTemp("T_StumbleRateSpan", 1.0f);  // TEMPTUNE
const float StumbleDeepStart = tuneTemp("T_StumbleDeep", 0.35f);  // TEMPTUNE
const float StumbleDecelStart = tuneTemp("T_StumbleDecel", 15.0f);  // TEMPTUNE
const float StumbleDecelSpan = tuneTemp("T_StumbleDecelSpan", 10.0f);  // TEMPTUNE
const float StumbleBaselineSeconds = tuneTemp("T_StumbleBase", 1.0f);  // TEMPTUNE
const float StumbleSwingMeasured = tuneTemp("T_StumbleSwing", 1.0f);  // TEMPTUNE
const float StumbleCadence = tuneTemp("T_StumbleCadence", 0.0f);  // TEMPTUNE
const float StumblePlacement = tuneTemp("T_StumblePlace", 0.0f);  // TEMPTUNE
const float StumblePlacementLimit = tuneTemp("T_StumblePlaceMax", 0.6f);  // TEMPTUNE
const float StumbleDrop = tuneTemp("T_StumbleDrop", 0.065f);  // TEMPTUNE
const float StumbleTrunkScale = tuneTemp("T_StumbleTrunk", 1.0f);  // TEMPTUNE
const float StumbleShiftScale = tuneTemp("T_StumbleShift", 1.0f);  // TEMPTUNE
const float StumbleArmsScale = tuneTemp("T_StumbleArms", 0.75f);  // TEMPTUNE
const float CurveLeanMaximum = tuneTemp("T_CurveLeanMax", 0.45f);  // TEMPTUNE
const float CurveRootShare = tuneTemp("T_CurveRoot", 0.6f);  // TEMPTUNE
const float CurveRootSprint = tuneTemp("T_CurveRootSprint", 0.15f);  // TEMPTUNE
const float CurveRootSprintFrom = tuneTemp("T_CurveRootFrom", 3.5f);  // TEMPTUNE (m/s)
const float CurveRootSprintSpan = tuneTemp("T_CurveRootSpan", 2.5f);  // TEMPTUNE (m/s)
const float CurveFootShare = tuneTemp("T_CurveFoot", 1.0f);  // TEMPTUNE
const float CurveDropShare = tuneTemp("T_CurveDrop", 0.6f);  // TEMPTUNE
const float CurveLeanHeight = tuneTemp("T_CurveHeight", 0.70f);  // TEMPTUNE
const bool CurveNew = tuneTemp("T_CurveNew", 1.0f) > 0.5f;  // TEMPTUNE
const float CurveFootYawMaximum = tuneTemp("T_FootYawMax", 0.25f);  // TEMPTUNE
const float CurveFootYawStart = tuneTemp("T_FootYawStart", 4.0f);  // TEMPTUNE
const float CurveFootYawSpan = tuneTemp("T_FootYawSpan", 6.0f);  // TEMPTUNE
const float CurveFootYawOuterShare = tuneTemp("T_FootYawOut", 0.2f);  // TEMPTUNE
const float BodyTurnFilterSeconds = tuneTemp("T_BodyTurnFilter", 0.2f);  // TEMPTUNE
const float CurveFootYawRate = tuneTemp("T_FootYawRate", 10.0f);  // TEMPTUNE
const bool CurveFootHoldYaw = tuneTemp("T_FootHold", 1.0f) > 0.5f;  // TEMPTUNE
const bool BrakeFootingStandingOnly = tuneTemp("T_FootingFix", 1.0f) > 0.5f;  // TEMPTUNE
const float PivotSpeed = tuneTemp("T_PivotSpeed", 9.0f);  // TEMPTUNE
const float PivotFullRateBelow = tuneTemp("T_PivotFull", 5.0f);  // TEMPTUNE
const float ReverseDeceleration = tuneTemp("T_RevDecel", 16.0f);  // TEMPTUNE
const float SettleToneRise = tuneTemp("T_ToneRise", 1.6f);  // TEMPTUNE
const float FallSmoothSeconds = tuneTemp("T_FallSmooth", 0.12f);  // TEMPTUNE
const bool FetalEnabled = tuneTemp("T_Fetal", 1.0f) > 0.5f;  // TEMPTUNE
const float FetalHip = tuneTemp("T_FetalHip", 1.35f);  // TEMPTUNE
const float FetalKnee = tuneTemp("T_FetalKnee", 1.85f);  // TEMPTUNE
const bool AirBalanceNew = tuneTemp("T_AirBalance", 1.0f) > 0.5f;  // TEMPTUNE
const float AirArmOpen = tuneTemp("T_AirArmOpen", 0.60f);  // TEMPTUNE
const float FallAirTone = tuneTemp("T_AirTone", 0.90f);  // TEMPTUNE
const float AirWindmill = tuneTemp("T_Windmill", 0.55f);  // TEMPTUNE
const float AirWindmillRate = tuneTemp("T_WindmillRate", 0.9f);  // TEMPTUNE
const float AirPedalRate = tuneTemp("T_PedalRate", 0.8f);  // TEMPTUNE
const float GroundLife = tuneTemp("T_GroundLife", 1.0f);  // TEMPTUNE
const float GroundLifeRate = tuneTemp("T_GroundLifeRate", 0.55f);  // TEMPTUNE
const float FallLife = tuneTemp("T_FallLife", 1.0f);  // TEMPTUNE
const bool LandingPrepEnabled = tuneTemp("T_LandPrep", 1.0f) > 0.5f;  // TEMPTUNE
const float LandingAlign = tuneTemp("T_LandAlign", 0.7f);  // TEMPTUNE
const float LandingMaxAngle = tuneTemp("T_LandMax", 0.60f);  // TEMPTUNE
const float LandingLegShare = tuneTemp("T_LandLeg", 0.90f);  // TEMPTUNE
const float LandingLeadSeconds = tuneTemp("T_LandLead", 0.25f);  // TEMPTUNE
const float LandingTrunkAlign = tuneTemp("T_LandTrunk", 0.5f);  // TEMPTUNE
const float LandingRollSign = tuneTemp("T_LandRollSign", -1.0f);  // TEMPTUNE
const float LandingTrunkSide = tuneTemp("T_LandTrunkSide", 0.2f);  // TEMPTUNE
const bool LandRunLead = tuneTemp("T_LandRunLead", 1.0f) > 0.5f;  // TEMPTUNE
const float LandRunMaxAngle = tuneTemp("T_LandRunMax", 0.35f);  // TEMPTUNE (rad)
const float IdleNeckExtension = tuneTemp("T_NeckExt", 0.20f);  // TEMPTUNE
const float IdleHeadExtension = tuneTemp("T_HeadExt", 0.07f);  // TEMPTUNE
const float IdleChestExtension = tuneTemp("T_ChestExt", 0.08f);  // TEMPTUNE
const float IdlePostureMovingShare = tuneTemp("T_PostureMoving", 0.0f);  // TEMPTUNE
const bool BalanceArmFix = tuneTemp("T_BalArmFix", 0.0f) > 0.5f;  // TEMPTUNE
const float BalanceTrunkPitch = tuneTemp("T_BalTrunk", -0.13f);  // TEMPTUNE
const float BalanceTrunkRoll = tuneTemp("T_BalTrunkRoll", -0.08f);  // TEMPTUNE
const float SpineRollGainSlow = tuneTemp("T_SpineRollSlow", 1.0f);  // TEMPTUNE
const float SpineRollGainFast = tuneTemp("T_SpineRollFast", 1.0f);  // TEMPTUNE
const float SpineImpactFlex = tuneTemp("T_SpineImpact", 0.05f);  // TEMPTUNE
const float SpineRollLimit = tuneTemp("T_SpineRollLimit", 0.42f);  // TEMPTUNE
const bool GaitLeashEnabled = tuneTemp("T_GaitLeash", 1.0f) > 0.5f;  // TEMPTUNE
const float GaitLeashMeters = tuneTemp("T_GaitLeashM", 0.10f);  // TEMPTUNE
const bool CapturePlacement = tuneTemp("T_CapPlace", 1.0f) > 0.5f;  // TEMPTUNE
// O pouso pelo ponto de captura inteiro tambem com a ajuda alta (fracao).
const float CapturePlacementAllAssist = tuneTemp("T_CapPlaceAll", 0.0f);  // TEMPTUNE
const float CapturePlacementGain = tuneTemp("T_CapGain", 1.0f);  // TEMPTUNE
const float CapturePlacementLimit = tuneTemp("T_CapLimit", 0.30f);  // TEMPTUNE
const float BalanceGaitShare = tuneTemp("T_BalGait", 1.0f);  // TEMPTUNE
const bool BalanceGaitNoLegs = tuneTemp("T_BalGaitNoLegs", 1.0f) > 0.5f;  // TEMPTUNE
const bool TripDetection = tuneTemp("T_Trip", 1.0f) > 0.5f;  // TEMPTUNE
const float GaitHoldYaw = tuneTemp("T_GaitHoldYaw", 0.0f);  // TEMPTUNE
const float StandingTurnRateHigh = tuneTemp("T_TurnRateHigh", 12.0f);  // TEMPTUNE
const float StandingTurnAccelerationHigh = tuneTemp("T_TurnAccHigh", 120.0f);  // TEMPTUNE
// (Giro parado, 04/10: a pelve espera os pes - torcao ate 0,45 rad sobre a
// base e o pe apoiado nunca gira no chao. Com 0,9 e o pe girando junto, a
// pelve seguia a camera a ate 12 rad/s arrastando os pes: esfregando,
// cruzando - 11,6 ticks/caso, 55 a 180 graus/s - e uma queda a 360 graus/s.)
const float StandingHipTwistHigh = tuneTemp("T_HipTwistHigh", 0.45f);  // TEMPTUNE
const float StandingFootPivot = tuneTemp("T_FootPivot", 0.0f);  // TEMPTUNE
const float StandingTurnGainHigh = tuneTemp("T_TurnGainHigh", 18.0f);  // TEMPTUNE
const float GaitHoldYawFrom = tuneTemp("T_GaitHoldYawFrom", 0.5f);  // TEMPTUNE
const float JumpCrouchDepth = tuneTemp("T_JumpCrouch", 0.22f);  // TEMPTUNE
const float JumpPrepTrunkShare = tuneTemp("T_JumpPrepTrunk", 0.2f);  // TEMPTUNE
const float LowAssistTurnCentripetal = tuneTemp("T_LowTurnAc", 100.0f);  // TEMPTUNE
const float LowAssistTurnMinimum = tuneTemp("T_LowTurnMin", 1.2f);  // TEMPTUNE
const float CutBrakeCosine = tuneTemp("T_CutCos", -0.5f);  // TEMPTUNE
const float CutBrakeMinimumSpeed = tuneTemp("T_CutSpeed", 2.5f);  // TEMPTUNE
const float CapturePullBackShare = tuneTemp("T_CapBack", 1.0f);  // TEMPTUNE
const float CaptureBackDeadZone = tuneTemp("T_CapBackDead", 0.25f);  // TEMPTUNE (m, v2)
const float StanceStiffness = tuneTemp("T_StanceStiffness", 2.5f);  // TEMPTUNE (v2)
const float GaitLeadSpeed = tuneTemp("T_GaitLeadSpeed", 0.0f);  // TEMPTUNE
const float SprintClipSpeed = tuneTemp("T_SprintClip", 0.0f);  // TEMPTUNE
const bool SprintClipAllAssist = tuneTemp("T_SprintClipAll", 0.0f) > 0.5f;  // TEMPTUNE
// Piso de ritmo do ciclo de sprint em qualquer ajuda (o corpo abaixo da
// velocidade do clipe: curva fechada, arrancada).
const float SprintRateFloor = tuneTemp("T_SprintRateFloor", 0.55f);  // TEMPTUNE
const float LowAssistBrakeDeceleration = tuneTemp("T_LowBrakeDec", 12.0f);  // TEMPTUNE
const float LowAssistBrakeLeash = tuneTemp("T_LowBrakeLeash", 0.8f);  // TEMPTUNE
const bool BrakeAnchorNoFeedforward = tuneTemp("T_BrakeNoFF", 1.0f) > 0.5f;  // TEMPTUNE
const bool SectorHoldWhileBraking = tuneTemp("T_SectorHold", 1.0f) > 0.5f;  // TEMPTUNE
const float StrafeBrakeEngageSpeed = tuneTemp("T_StrafeBrake", 2.0f);  // TEMPTUNE
const bool LockAtTouch = tuneTemp("T_LockTouch", 1.0f) > 0.5f;  // TEMPTUNE
const float LockTouchWaitSeconds = tuneTemp("T_LockTouchWait", 0.10f);  // TEMPTUNE
const float StanceRiseLimit = tuneTemp("T_StanceRise", 0.03f);  // TEMPTUNE
const bool GaitSwingMeasuredPelvis = tuneTemp("T_GaitSwingMeasured", 0.0f) > 0.5f;  // TEMPTUNE
const bool LockAtFirstTouch = tuneTemp("T_FirstTouch", 1.0f) > 0.5f;  // TEMPTUNE
const float LockTouchPull = tuneTemp("T_LockPull", 0.08f);  // TEMPTUNE
const float GaitSwingReleaseSeconds = tuneTemp("T_GaitSwingRelease", 0.15f);  // TEMPTUNE
// 0 posicao e orientacao medidas; 1 so a posicao; 2 so a orientacao; 3 o
// plano e a orientacao; 4 so o plano.
const int GaitSwingMeasuredMode = static_cast<int>(tuneTemp("T_GaitSwingMode", 0.0f));  // TEMPTUNE
const bool GaitSwingOnlyInChange = tuneTemp("T_GaitSwingChange", 1.0f) > 0.5f;  // TEMPTUNE
const bool GaitSwingLatch = tuneTemp("T_GaitSwingLatch", 1.0f) > 0.5f;  // TEMPTUNE
const bool LockTouchOnlyInChange = tuneTemp("T_LockTouchChange", 1.0f) > 0.5f;  // TEMPTUNE
const float GaitChangeVelocityError = tuneTemp("T_GaitChangeError", 1.0f);  // TEMPTUNE
const float SwingTrackGain = tuneTemp("T_SwingTrack", 0.0f);  // TEMPTUNE
const float SwingTrackMaximum = tuneTemp("T_SwingTrackMax", 0.15f);  // TEMPTUNE
const float LowAssistBrakeReferenceLeash = tuneTemp("T_LowBrakeRef", 0.30f);  // TEMPTUNE
const bool CaptureInBrakeRun = tuneTemp("T_CapBrake", 0.0f) > 0.5f;  // TEMPTUNE
const float LowAssistBrakeStepSpeed = tuneTemp("T_LowBrakeStep", 1.8f);  // TEMPTUNE
const float GaitLeadMinimumRequest = tuneTemp("T_GaitLeadMin", 3.5f);  // TEMPTUNE
const float LowAssistRateFloor = tuneTemp("T_LowRateFloor", 0.55f);  // TEMPTUNE
const float ReachLiftPerMeter = tuneTemp("T_ReachLift", 0.0f);  // TEMPTUNE
const float ReachLiftStart = tuneTemp("T_ReachLiftStart", 0.10f);  // TEMPTUNE
const float ReachLiftMaximum = tuneTemp("T_ReachLiftMax", 0.15f);  // TEMPTUNE
const bool ReachLiftBrakeOnly = tuneTemp("T_ReachLiftBrake", 0.0f) > 0.5f;  // TEMPTUNE
const float SwingLowAssistStiffness = tuneTemp("T_SwingStiff", 1.0f);  // TEMPTUNE
const float CrossGuardWidth = tuneTemp("T_CrossGuard", 0.0f);  // TEMPTUNE
const bool PlannerSwingLowAssist = tuneTemp("T_PlannerSwing", 0.0f) > 0.5f;  // TEMPTUNE
const float SwingLowAssistTorque = tuneTemp("T_SwingTorque", 1.0f);  // TEMPTUNE
const bool LowAssistRateFloorInBrake = tuneTemp("T_LowRateBrake", 1.0f) > 0.5f;  // TEMPTUNE
const bool GroundAlive = tuneTemp("T_GroundAlive", 1.0f) > 0.5f;  // TEMPTUNE
const bool FallProtectNew = tuneTemp("T_FallNew", 1.0f) > 0.5f;  // TEMPTUNE
const bool FallLegsNew = tuneTemp("T_FallLegs", 1.0f) > 0.5f;  // TEMPTUNE
const bool FallArmsNew = tuneTemp("T_FallArms", 1.0f) > 0.5f;  // TEMPTUNE
const float FallKneeForward = tuneTemp("T_KneeFwd", 1.10f);  // TEMPTUNE
const float FallKneeBackward = tuneTemp("T_KneeBack", 1.30f);  // TEMPTUNE
const float FallKneeSide = tuneTemp("T_KneeSide", 0.90f);  // TEMPTUNE
const float FallHipForward = tuneTemp("T_HipFwd", 0.55f);  // TEMPTUNE
const float FallHipBackward = tuneTemp("T_HipBack", 1.10f);  // TEMPTUNE
const float FallHipSide = tuneTemp("T_HipSide", 0.50f);  // TEMPTUNE
const float FallLegTone = tuneTemp("T_FallLegTone", 0.50f);  // TEMPTUNE
const float FallTrunkTone = tuneTemp("T_FallTrunkTone", 0.40f);  // TEMPTUNE
const float FallElbowReach = tuneTemp("T_ElbowReach", 0.35f);  // TEMPTUNE
const float FallElbowAbsorb = tuneTemp("T_ElbowAbsorb", 0.90f);  // TEMPTUNE
const float FallAbsorbSeconds = tuneTemp("T_AbsorbSeconds", 0.50f);  // TEMPTUNE
const float FallCatchTone = tuneTemp("T_CatchTone", 1.10f);  // TEMPTUNE
const float FallArmExtension = tuneTemp("T_ArmExt", 0.98f);  // TEMPTUNE
const float FallArmForward = tuneTemp("T_ArmFwd", 0.75f);  // TEMPTUNE
const float FallArmIterations = tuneTemp("T_ArmIter", 12.0f);  // TEMPTUNE
const float FallArmReadyForward = tuneTemp("T_ArmReadyFwd", 0.30f);  // TEMPTUNE
const float FallArmReadyDrop = tuneTemp("T_ArmReadyDrop", 0.20f);  // TEMPTUNE
const float FallArmNearHeight = tuneTemp("T_ArmNear", 0.0f);  // TEMPTUNE
const float FallArmSideReach = tuneTemp("T_ArmSide", 0.30f);  // TEMPTUNE
const float FallArmBackReach = tuneTemp("T_ArmBack", 0.20f);  // TEMPTUNE
const float FallFarArmShare = tuneTemp("T_FarArm", 0.7f);  // TEMPTUNE
const float FallTwistRadians = tuneTemp("T_FallTwist", 0.55f);  // TEMPTUNE
const float PivotCrouchMeters = tuneTemp("T_PivotCrouch", 0.12f);  // TEMPTUNE
constexpr float Pi = 3.14159265358979323846f;
constexpr float TwoPi = Pi * 2.0f;

// Olhar x movimento: setores de direcao em relacao a camera. A direcao
// decide o ciclo, nunca a ordem das teclas. O teclado da oito direcoes a
// 45 graus uma da outra; as fronteiras ficam no meio entre elas:
//   frente e diagonais da frente (0, +-45)  -> corrida, pelve girada
//   lados (+-90)                            -> strafe
//   costas e diagonais de tras (+-135, 180) -> recuo, pelve girada
// Fronteiras em 67,5 e 112,5 graus. Com as fronteiras em 45 e 135 (setores
// iguais de 90), cada diagonal do teclado caia em cima de uma delas e quem
// decidia era a histerese - W e depois A dava corrida; A e depois W, strafe.
// A histerese (a folga para trocar de setor depois de passar a fronteira)
// fica so para o analogico perto da fronteira; nenhuma direcao do teclado
// chega nela. Sem strafe, dois setores (frente, costas), fronteira em 90.
constexpr float StrafeFrontLimitRadians = 67.5f * Pi / 180.0f;
constexpr float StrafeBackLimitRadians = 112.5f * Pi / 180.0f;
constexpr float StrafeSectorHysteresisRadians = 10.0f * Pi / 180.0f;
constexpr float PlainSectorLimitRadians = 90.0f * Pi / 180.0f;
constexpr float PlainSectorHysteresisRadians = 15.0f * Pi / 180.0f;

float gaitSectorCenter(CharacterGaitDirection3D direction) {
    switch (direction) {
    case CharacterGaitDirection3D::Left: return Pi * 0.5f;
    case CharacterGaitDirection3D::Right: return -Pi * 0.5f;
    case CharacterGaitDirection3D::Backward: return Pi;
    default: return 0.0f;
    }
}

// Quanto a direcao (em relacao ao olhar) passa do setor; zero dentro dele.
float gaitSectorOverrun(CharacterGaitDirection3D direction,
    float travelFromView, bool strafe) {
    const float angle = std::abs(travelFromView);
    const float front = strafe ? StrafeFrontLimitRadians
        : PlainSectorLimitRadians;
    const float back = strafe ? StrafeBackLimitRadians
        : PlainSectorLimitRadians;
    switch (direction) {
    case CharacterGaitDirection3D::Forward:
        return std::max(angle - front, 0.0f);
    case CharacterGaitDirection3D::Backward:
        return std::max(back - angle, 0.0f);
    case CharacterGaitDirection3D::Left:
    case CharacterGaitDirection3D::Right: {
        // O lado certo, entre as duas fronteiras; o outro lado esta longe.
        const bool leftSide = travelFromView >= 0.0f;
        const bool wanted = direction == CharacterGaitDirection3D::Left;
        if (leftSide != wanted) return Pi;
        return std::max({front - angle, angle - back, 0.0f});
    }
    }
    return Pi;
}
// Quanto do olhar a coluna devolve e quanto a cabeca completa, em relacao a
// pelve. Coluna: 45 graus dos 67 que Abdomen, Chest e UpperChest somam - o
// resto fica de folga para a rotacao do proprio ciclo. Cabeca: 75, o limite
// de pescoco (30) mais cabeca (45). Pelve a ate 105 graus da camera (a borda
// do recuo) ainda deixa a cabeca olhando para ela.
constexpr float TorsoTwistLimitRadians = 45.0f * Pi / 180.0f;
// Correndo, o tronco fica com as pernas e quem olha e a cabeca: com a coluna
// girada 45 graus, o braco balanca num plano torto em relacao a passada e
// bate na coxa que sobe (medido: antebraco 61 graus fora do alvo num sprint
// em diagonal). Com 20, cabeca e coluna ainda cobrem 95 graus.
constexpr float SprintTorsoTwistLimitRadians = 20.0f * Pi / 180.0f;
constexpr float HeadTurnLimitRadians = 75.0f * Pi / 180.0f;
constexpr float TorsoLookShare = 0.8f;
constexpr float DefaultTransitionSeconds = 0.14f;
constexpr float GaitFlipTransitionSeconds = 0.35f;

// Fall protection keeps some tone, then yields while the body settles.
// Recovery ramps from this relaxed state over the existing 0.5 s envelope.
// Modo fisico, de pe: tonus das juntas (fora as pernas, que tem o proprio
// ajuste) em relacao a forca nominal do perfil. Era 2,0.
constexpr float UprightMuscleTone = 1.3f;
// Modo fisico: altura minima do pe no meio de um passo (m, acima do chao).
constexpr float SwingMinimumClearance = 0.13f;
const float SettleRestingTone = tuneTemp("T_RestTone", 0.40f);  // TEMPTUNE
// Caido e parado, quanto tempo fica largado antes de se arrumar para levantar.
const float SettleRestBeforeGather = tuneTemp("T_SettleRest", 0.20f);  // TEMPTUNE
// Numa nova tentativa de levantar (a anterior falhou), quanto espera.
const float RetryRestBeforeGather = tuneTemp("T_RetryRest", 0.10f);  // TEMPTUNE
// Caindo: tonus dos bracos/pescoco (reflexo) e do resto (queda sem volta).
// O corpo (era 0,38) cai solto; os bracos protegem - com 0,65 a mao que ia
// ao chao cedia no tombo e a cabeca batia antes (queda de lado).
const float FallReflexLimbTone = tuneTemp("T_ReflexTone", 0.85f);  // TEMPTUNE
// Reflexo das maos ao chao caindo (os bracos prontos a frente e depois
// estendidos ate o chao): tirado a pedido do usuario (05/10).
const bool FallArmsReflex = tuneTemp("T_FallArms", 0.0f) > 0.5f;  // TEMPTUNE
// Postura perdida (pedido do usuario, 05/10): perdendo a postura, a ajuda
// segurava o corpo torto por ate 1,5 s - o corpo a 45-74 graus fora dos pes,
// os pes batendo no chao, as maos paradas - e o puxava de volta. Chegando a
// esse ponto, agora ele cai: a ajuda sai e as pernas cedem no mesmo tick.
// (Um resgate - passo rapido no rumo da queda e a coluna para tras - foi
// tentado e descartado pelo usuario.)
const bool PostureLossFall = tuneTemp("T_PostureLoss", 1.0f) > 0.5f;  // TEMPTUNE
// A postura e o pendulo: o centro de massa contra o meio dos pes (rad da
// vertical). Trotando e correndo ele nao passa de 15-17 graus; os tropecos
// que se salvam sozinhos, de ~30 graus por 0,2-0,3 s. Alem de PostureLostTilt
// e queda; alem de PostureHeldTilt por mais de PostureHeldSeconds tambem
// (segurado a 31-40 graus por 0,5-0,7 s) - desde que fora da pose pedida
// (PostureHeldDeviation, pelo mesmo pendulo nela): no impulso do sprint os
// dois pes ficam atras e ele passa de 25 graus de proposito.
const float PostureLostTilt = tuneTemp("T_PostureLost", 0.73f);  // TEMPTUNE (rad)
const float PostureHeldTilt = tuneTemp("T_PostureHeld", 0.44f);  // TEMPTUNE (rad)
const float PostureHeldDeviation = tuneTemp("T_PostureHeldDev", 0.35f);  // TEMPTUNE (rad)
const float PostureHeldSeconds = tuneTemp("T_PostureHeldSec", 0.30f);  // TEMPTUNE (s)
// O peito tombado alem disto (rad, da vertical) tambem e queda.
const float PostureChestLost = tuneTemp("T_PostureChest", 0.96f);  // TEMPTUNE (rad)
// Cambaleio (pedido do usuario, 05/10 noite: "mesmo ragdolizado, tentar
// manter a postura e se recobrar"): o choque deste tick (soma dos contatos
// fora o apoio dos pes, N.s) de Min a Full amolece o corpo de 0 a 1 na hora;
// o tonus volta a RecoverPerSecond (mais rapido em pe). Amolecido, cada junta
// fica com o seu piso de tonus (pernas seguram um pouco o corpo; tronco e
// bracos soltam) e os limites da postura perdida e da queda sem volta ganham
// TiltRelief (rad) - os passos de equilibrio seguem tentando; so o chao (o
// corpo tombado ou baixo tocando-o) decide a queda.
const float StaggerImpulseMin = tuneTemp("T_StaggerMin", 8.0f);  // TEMPTUNE (N.s)
const float StaggerImpulseFull = tuneTemp("T_StaggerFull", 45.0f);  // TEMPTUNE (N.s)
const float StaggerRecoverPerSecond = tuneTemp("T_StaggerRecover", 0.75f);  // TEMPTUNE (1/s)
const float StaggerLegTone = tuneTemp("T_StaggerLeg", 0.20f);  // TEMPTUNE
const float StaggerTorsoTone = tuneTemp("T_StaggerTorso", 0.10f);  // TEMPTUNE
const float StaggerArmTone = tuneTemp("T_StaggerArm", 0.04f);  // TEMPTUNE
// No choque o corpo fica solto por este tempo (s, vezes o tamanho dele)
// antes de o tonus comecar a voltar; tronco e bracos perdem ate esta fracao
// da compensacao de gravidade.
const float StaggerHoldSeconds = tuneTemp("T_StaggerHold", 0.25f);  // TEMPTUNE
const float StaggerGravityDrop = tuneTemp("T_StaggerGravity", 0.9f);  // TEMPTUNE
// E o quanto mais tombado (no cosseno da pelve) ele ainda nao conta como
// caido com um pe no chao.
const float StaggerFallenRelief = tuneTemp("T_StaggerFallen", 0.25f);  // TEMPTUNE
const float StaggerTiltRelief = tuneTemp("T_StaggerTilt", 0.35f);  // TEMPTUNE (rad)
// Volta a valer (o corpo de pe de novo) abaixo disto.
const float PostureRegainedTilt = tuneTemp("T_PostureOk", 0.21f);  // TEMPTUNE (rad)
// v2: o pendulo da postura contra os pes apoiados (ver o bloco da postura).
// Locomocao fisica v2 (CharacterLocomotionInput3D::physicalV2; as variaveis
// so para experimento, com o v1): ver cada bloco.
const bool PostureSupportPendulumEnv = tuneTemp("T_PostureSupport", 0.0f) > 0.5f;  // TEMPTUNE
const bool StartOnlyFromRestEnv = tuneTemp("T_StartFromRest", 0.0f) > 0.5f;  // TEMPTUNE
const bool LandingResyncEnv = tuneTemp("T_LandResync", 0.0f) > 0.5f;  // TEMPTUNE
const float PostureGaitPelvisLost = tuneTemp("T_PostureGaitPelvis", 0.87f);  // TEMPTUNE (rad)
// v2: a perna de apoio da corrida pede no maximo 6 cm acima (ou abaixo) da
// pelve de verdade, e o amortecimento vertical vale fora das curvas tambem
// (os saltinhos de quem pousa fundo e estica a perna de uma vez).
constexpr float StanceRootBandV2 = 0.06f;
// v2, recuperacao apoiada: altura minima do apoio acima do chao (m), memoria
// do contato (s), tempo maximo da tentativa e sem progresso (s), inclinacao
// maxima da pelve (cos: ~80 graus).
const float BraceMinimumHeight = tuneTemp("T_BraceHeight", 0.40f);  // TEMPTUNE
const float BraceMemorySeconds = tuneTemp("T_BraceMemory", 0.30f);  // TEMPTUNE
const float BraceAttemptSeconds = tuneTemp("T_BraceAttempt", 4.0f);  // TEMPTUNE
const float BraceStallSeconds = tuneTemp("T_BraceStall", 1.5f);  // TEMPTUNE
const float BraceMinimumUpright = tuneTemp("T_BraceUpright", 0.17f);  // TEMPTUNE
constexpr float FallBodyTone = 0.25f;
// Largado (caindo sem volta, deitado): quanto da compensacao de gravidade
// resta - os membros pesam. Bracos protegendo a queda seguram mais.
constexpr float FallBodyGravity = 0.25f;
constexpr float FallLimbGravity = 0.80f;
constexpr float LyingGravity = 0.10f;
// Tonus ao bater no chao (cai dele ate SettleRestingTone em ~0,1 s).
const float SettleImpactTone = tuneTemp("T_ImpactTone", 0.60f);  // TEMPTUNE
constexpr float RisingArmStiffness = 2.0f;
constexpr float RisingTorsoStiffness = 1.3f;
constexpr float RisingLegStiffness = 1.6f;
constexpr float RisingArmTorque = 2.0f;
constexpr float RisingTorsoTorque = 1.5f;
constexpr float RisingLegTorque = 1.8f;

bool finite(Vec3 value) {
    return std::isfinite(value.x) && std::isfinite(value.y)
        && std::isfinite(value.z);
}

float component(Vec3 value, std::size_t axis) {
    return axis == 0 ? value.x : axis == 1 ? value.y : value.z;
}

void setComponent(Vec3& value, std::size_t axis, float componentValue) {
    if (axis == 0) value.x = componentValue;
    else if (axis == 1) value.y = componentValue;
    else value.z = componentValue;
}

float smoothStep(float value) {
    value = std::clamp(value, 0.0f, 1.0f);
    return value * value * (3.0f - 2.0f * value);
}

float wrapAngle(float radians) {
    while (radians > Pi) radians -= TwoPi;
    while (radians < -Pi) radians += TwoPi;
    return radians;
}

std::size_t findLink(const RagdollProfile3D& profile, std::string_view id) {
    for (std::size_t index = 0; index < profile.links.size(); ++index) {
        if (profile.links[index].id == id) return index;
    }
    return profile.links.size();
}


Vec3 uprightOrientationError(Quaternion current, float desiredYaw,
    CharacterFallOrientation3D fallOrientation) {
    const Vec3 currentForward = current.rotate({ 1.0f, 0.0f, 0.0f });
    const Vec3 currentLeft = current.rotate({ 0.0f, 1.0f, 0.0f });
    const Vec3 currentUp = current.rotate({ 0.0f, 0.0f, 1.0f });
    const Vec3 desiredForward {
        std::cos(desiredYaw), std::sin(desiredYaw), 0.0f
    };
    const Vec3 desiredLeft {
        -std::sin(desiredYaw), std::cos(desiredYaw), 0.0f
    };
    Vec3 error = (cross(currentForward, desiredForward)
        + cross(currentLeft, desiredLeft)
        + cross(currentUp, Vec3 { 0.0f, 0.0f, 1.0f })) * 0.5f;
    if (error.lengthSquared() < 0.0025f && currentUp.z < -0.65f) {
        error = desiredLeft * (fallOrientation
                == CharacterFallOrientation3D::FaceDown ? -1.0f : 1.0f);
    }
    return error;
}

Quaternion exponentialRotation(Vec3 coordinates) {
    return ragdollJointRotation3D(coordinates);
}

Vec3 exponentialJacobian(Vec3 coordinates, Vec3 velocity) {
    return ragdollJointJacobian3D(coordinates, velocity);
}

Vec3 jointAngularVelocity(Vec3 target, Vec3 coordinateVelocity,
    Vec3 measured) {
    if (!finite(target) || !finite(coordinateVelocity) || !finite(measured)) {
        return {};
    }
    return exponentialRotation(measured).conjugate().rotate(
        exponentialJacobian(target, coordinateVelocity));
}

// O pendulo do corpo: centro de massa (pelas fracoes de massa) menos o meio
// dos dois pes.
template <typename Position>
Vec3 pendulumArm(const RagdollProfile3D& profile, std::size_t leftFoot, std::size_t rightFoot, Position position) {
    Vec3 center {};
    float mass = 0.0f;
    for (std::size_t i = 0; i < profile.links.size(); ++i) {
        center = center + position(i) * profile.links[i].massFraction;
        mass += profile.links[i].massFraction;
    }
    center = center * (1.0f / std::max(1e-4f, mass));
    return center - (position(leftFoot) + position(rightFoot)) * 0.5f;
}

void buildLocalPose(const RagdollProfile3D& profile,
    const std::vector<Vec3>& coordinates, RagdollAnimationPose3D& pose) {
    buildRagdollLocalPose3D(profile, coordinates, pose);
}

// Position-only damped least squares: brings a point fixed to endIndex
// (pointInEnd, in that link's frame) to a target, moving the joints of
// endIndex and its ancestors up to `depth` links, within their limits.
void solveChainPoint(const RagdollProfile3D& profile,
    std::vector<Vec3>& coordinates, std::size_t footIndex, Vec3 pointInEnd,
    int depth, Vec3 targetRootLocal, float weight, int iterations = 7) {
    solveRagdollChainPoint3D(profile, coordinates, footIndex, pointInEnd,
        depth, targetRootLocal, weight, iterations);
}

// Keeps the foot link on a support point (stance lock, turn steps).
void solveFootPosition(const RagdollProfile3D& profile,
    std::vector<Vec3>& coordinates, std::size_t footIndex,
    Vec3 targetRootLocal, float weight) {
    solveChainPoint(profile, coordinates, footIndex, {}, 3, targetRootLocal,
        weight);
}

Quaternion rotationBetween(Vec3 from, Vec3 to) {
    from = from.normalized();
    to = to.normalized();
    const float cosine = std::clamp(dot(from, to), -1.0f, 1.0f);
    const Vec3 axis = cross(from, to);
    const float sine = axis.length();
    if (sine < 0.000001f) {
        if (cosine > 0.0f) return {};
        const Vec3 other = std::abs(from.x) < 0.9f
            ? Vec3 { 1.0f, 0.0f, 0.0f } : Vec3 { 0.0f, 1.0f, 0.0f };
        return Quaternion::fromAxisAngle(cross(from, other).normalized(), Pi);
    }
    return Quaternion::fromAxisAngle(axis / sine, std::atan2(sine, cosine));
}

Quaternion slerpQuaternion(Quaternion from, Quaternion to, float t) {
    t = std::clamp(t, 0.0f, 1.0f);
    float cosine = from.x * to.x + from.y * to.y + from.z * to.z
        + from.w * to.w;
    if (cosine < 0.0f) {
        to = { -to.x, -to.y, -to.z, -to.w };
        cosine = -cosine;
    }
    if (cosine > 0.9995f) {
        return Quaternion { from.x + (to.x - from.x) * t,
            from.y + (to.y - from.y) * t, from.z + (to.z - from.z) * t,
            from.w + (to.w - from.w) * t }.normalized();
    }
    const float angle = std::acos(std::clamp(cosine, -1.0f, 1.0f));
    const float sine = std::sin(angle);
    const float a = std::sin((1.0f - t) * angle) / sine;
    const float b = std::sin(t * angle) / sine;
    return Quaternion { from.x * a + to.x * b, from.y * a + to.y * b,
        from.z * a + to.z * b, from.w * a + to.w * b }.normalized();
}

// Pe inteiro: posicao e orientacao do link do pe, no espaco da raiz. A
// coxa e a canela levam o tornozelo (a ancora da junta do pe) onde o pe
// pedido precisa dele; o tornozelo, com os dois eixos que tem, orienta o pe,
// com prioridade para a sola (o vetor "para cima" do pe decide o contato
// com o chao) e a ponta em segundo.
Vec3 rotationVector(Quaternion value);

// TEMPTUNE: iteracoes do IK do pe guiado pela bola (o resto, 7).
const int BallFootIterations = static_cast<int>(std::getenv("T_BallIkIterations")
    ? std::strtof(std::getenv("T_BallIkIterations"), nullptr) : 24.0f);

void solveFootPose(const RagdollProfile3D& profile,
    std::vector<Vec3>& coordinates, std::size_t footIndex,
    Vec3 targetPosition, Quaternion targetOrientation, float weight,
    bool holdYaw, int iterations = 7) {
    if (footIndex == 0 || footIndex >= profile.links.size()
        || coordinates.size() != profile.links.size() || weight <= 0.0f
        || !finite(targetPosition)) {
        return;
    }
    const RagdollLinkDefinition3D& foot = profile.links[footIndex];
    if (foot.parentIndex <= 0) return;
    const std::size_t shinIndex = static_cast<std::size_t>(foot.parentIndex);
    const RagdollLinkDefinition3D& shin = profile.links[shinIndex];
    const Vec3 anchor = foot.inboundJoint.anchorModelPosition;
    const Vec3 anchorInFoot = foot.modelOrientation.conjugate().rotate(
        anchor - foot.modelPosition);
    const Vec3 anchorInShin = shin.modelOrientation.conjugate().rotate(
        anchor - shin.modelPosition);
    const std::vector<Vec3> original = coordinates;
    const Vec3 ankleTarget = targetPosition
        + targetOrientation.rotate(anchorInFoot);
    solveChainPoint(profile, coordinates, shinIndex, anchorInShin, 2,
        ankleTarget, 1.0f, iterations);

    RagdollAnimationPose3D pose;
    // Rumo do pe pelo quadril. O tornozelo so tem flexao e inversao: o rumo
    // do pe em relacao a pelve vem da rotacao da coxa. Resolvendo so a
    // posicao do tornozelo, o pe apoiado girava junto com a pelve - no chao
    // (medido: pe "travado" rodando 32 graus no primeiro passo de um giro
    // parado, os mesmos da pelve). Gira a perna inteira em volta da linha
    // quadril-tornozelo (o tornozelo fica onde esta) ate o pe apontar para
    // onde deve, dentro dos limites do quadril.
    //
    // So parado (giro no lugar, acomodacao, idle). Andando, o pe segue o
    // rumo da pelve como sempre seguiu: com a abertura de 7 graus da pose
    // respeitada, o pe de 28 cm nao cabia mais no degrau de 28 cm e a
    // quina afundava (medido: descendo escada, 7 amostras a ate 11 cm).
    if (holdYaw && shin.parentIndex > 0) {
        const std::size_t thighIndex =
            static_cast<std::size_t>(shin.parentIndex);
        const RagdollLinkDefinition3D& thigh = profile.links[thighIndex];
        const std::size_t pelvisIndex =
            static_cast<std::size_t>(std::max(0, thigh.parentIndex));
        const RagdollLinkDefinition3D& pelvis = profile.links[pelvisIndex];
        buildLocalPose(profile, coordinates, pose);
        const Quaternion parentFrame = pose.linkOrientations[pelvisIndex]
            * pelvis.modelOrientation.conjugate()
            * thigh.inboundJoint.frameModelOrientation;
        const Quaternion childFrame = (thigh.modelOrientation.conjugate()
            * thigh.inboundJoint.frameModelOrientation).conjugate();
        const Vec3 hip = pose.linkPositions[pelvisIndex]
            + (pose.linkOrientations[pelvisIndex]
                * pelvis.modelOrientation.conjugate()).rotate(
                    thigh.inboundJoint.anchorModelPosition
                        - pelvis.modelPosition);
        const Vec3 ankle = pose.linkPositions[footIndex]
            + pose.linkOrientations[footIndex].rotate(anchorInFoot);
        Vec3 axis = ankle - hip;
        const Vec3 forwardLocal = foot.modelOrientation.conjugate().rotate(
            { 1.0f, 0.0f, 0.0f });
        if (axis.lengthSquared() > 0.01f) {
            axis = axis.normalized();
            Vec3 current = pose.linkOrientations[footIndex].rotate(
                forwardLocal);
            Vec3 wanted = targetOrientation.rotate(forwardLocal);
            current -= axis * dot(current, axis);
            wanted -= axis * dot(wanted, axis);
            if (current.lengthSquared() > 0.01f
                && wanted.lengthSquared() > 0.01f) {
                const float swivel = std::atan2(
                    dot(cross(current, wanted), axis), dot(current, wanted));
                const Quaternion thighWorld = pose.linkOrientations[thighIndex];
                const Quaternion turned = (Quaternion::fromAxisAngle(axis,
                    swivel) * thighWorld).normalized();
                Vec3 local = rotationVector(parentFrame.conjugate() * turned
                    * childFrame.conjugate());
                for (std::size_t index = 0; index < 3; ++index) {
                    const RagdollAxisDefinition3D& limit =
                        thigh.inboundJoint.axes[index];
                    setComponent(local, index, limit.enabled
                        ? std::clamp(component(local, index),
                            limit.minimumRadians, limit.maximumRadians)
                        : 0.0f);
                }
                coordinates[thighIndex] = local;
                // No limite do quadril o tornozelo sai do lugar: de volta.
                solveChainPoint(profile, coordinates, shinIndex, anchorInShin,
                    2, ankleTarget, 1.0f, iterations);
            }
        }
    }

    // Tornozelo: Gauss-Newton nos eixos habilitados do pe.
    buildLocalPose(profile, coordinates, pose);
    const Quaternion shinFrame = pose.linkOrientations[shinIndex]
        * shin.modelOrientation.conjugate() * foot.inboundJoint.frameModelOrientation;
    const Quaternion childFrame = (foot.modelOrientation.conjugate()
        * foot.inboundJoint.frameModelOrientation).conjugate();
    const Vec3 upLocal = foot.modelOrientation.conjugate().rotate(
        { 0.0f, 0.0f, 1.0f });
    const Vec3 forwardLocal = foot.modelOrientation.conjugate().rotate(
        { 1.0f, 0.0f, 0.0f });
    const Vec3 wantedUp = targetOrientation.rotate(upLocal);
    const Vec3 wantedForward = targetOrientation.rotate(forwardLocal);
    const auto residual = [&](Vec3 angles, float out[6]) {
        const Quaternion world = shinFrame * exponentialRotation(angles)
            * childFrame;
        const Vec3 up = world.rotate(upLocal) - wantedUp;
        const Vec3 forward = (world.rotate(forwardLocal) - wantedForward)
            * 0.25f;
        out[0] = up.x; out[1] = up.y; out[2] = up.z;
        out[3] = forward.x; out[4] = forward.y; out[5] = forward.z;
    };
    std::array<std::size_t, 3> axes {};
    std::size_t free = 0;
    for (std::size_t axis = 0; axis < 3; ++axis) {
        if (foot.inboundJoint.axes[axis].enabled) axes[free++] = axis;
    }
    Vec3 angles = coordinates[footIndex];
    for (int iteration = 0; iteration < 8 && free > 0; ++iteration) {
        float r[6];
        residual(angles, r);
        float jacobian[6][3] {};
        for (std::size_t column = 0; column < free; ++column) {
            Vec3 probe = angles;
            setComponent(probe, axes[column],
                component(probe, axes[column]) + 0.0005f);
            float shifted[6];
            residual(probe, shifted);
            for (int row = 0; row < 6; ++row) {
                jacobian[row][column] = (shifted[row] - r[row]) / 0.0005f;
            }
        }
        // (J^T J + lambda) step = -J^T r
        float normal[3][4] {};
        for (std::size_t a = 0; a < free; ++a) {
            for (std::size_t b = 0; b < free; ++b) {
                for (int row = 0; row < 6; ++row) {
                    normal[a][b] += jacobian[row][a] * jacobian[row][b];
                }
            }
            normal[a][a] += 0.001f;
            for (int row = 0; row < 6; ++row) {
                normal[a][free] -= jacobian[row][a] * r[row];
            }
        }
        float step[3] {};
        if (free == 1) {
            step[0] = normal[0][1] / normal[0][0];
        } else if (free == 2) {
            const float determinant = normal[0][0] * normal[1][1]
                - normal[0][1] * normal[1][0];
            if (std::abs(determinant) < 1e-9f) break;
            step[0] = (normal[0][2] * normal[1][1]
                - normal[0][1] * normal[1][2]) / determinant;
            step[1] = (normal[0][0] * normal[1][2]
                - normal[1][0] * normal[0][2]) / determinant;
        } else {
            break;
        }
        float size = 0.0f;
        for (std::size_t column = 0; column < free; ++column) {
            const RagdollAxisDefinition3D& limit =
                foot.inboundJoint.axes[axes[column]];
            const float next = std::clamp(component(angles, axes[column])
                    + std::clamp(step[column], -0.2f, 0.2f),
                limit.minimumRadians, limit.maximumRadians);
            size += std::abs(next - component(angles, axes[column]));
            setComponent(angles, axes[column], next);
        }
        if (size < 0.00005f) break;
    }
    coordinates[footIndex] = angles;

    weight = std::clamp(weight, 0.0f, 1.0f);
    if (weight < 1.0f) {
        for (int link : { static_cast<int>(footIndex),
                static_cast<int>(shinIndex), shin.parentIndex }) {
            if (link <= 0) continue;
            const auto index = static_cast<std::size_t>(link);
            coordinates[index] = original[index]
                + (coordinates[index] - original[index]) * weight;
        }
    }
}

bool validClip(const AnimationClip3D* clip,
    const RagdollProfile3D& profile) {
    return clip != nullptr && clip->targetRigId == profile.id
        && std::isfinite(clip->durationSeconds)
        && clip->durationSeconds > 0.0f
        && validateAnimationClipForRagdoll3D(*clip, profile).empty();
}

float authoredHorizontalSpeed(const AnimationClip3D& clip) {
    if (clip.nominalSpeedMetersPerSecond > 0.0f) {
        return clip.nominalSpeedMetersPerSecond;
    }
    const Vec3 displacement = clip.sourceRootDisplacementMeters;
    return clip.durationSeconds > 0.0f
        ? std::hypot(displacement.x, displacement.y) / clip.durationSeconds
        : 0.0f;
}

// Giro parado pelo clipe: o giro (rad, relativo ao comeco) e o deslocamento
// no plano da raiz do clipe no tempo t (a raiz do turn_in_place.py e so isso).
float clipRootYaw(const AnimationClip3D& clip, const RagdollProfile3D& profile, float time) {
    const AnimationTrack3D* root = findAnimationTrack3D(clip, profile.links.front().id);
    if (root == nullptr) return 0.0f;
    const Quaternion q = sampleAnimationTrack3D(*root, std::clamp(time, 0.0f, clip.durationSeconds),
        clip.durationSeconds, false).rotationDelta;
    return 2.0f * std::atan2(q.z, q.w);
}

Vec3 clipRootPlanar(const AnimationClip3D& clip, const RagdollProfile3D& profile, float time) {
    const AnimationTrack3D* root = findAnimationTrack3D(clip, profile.links.front().id);
    if (root == nullptr) return {};
    const Vec3 t = sampleAnimationTrack3D(*root, std::clamp(time, 0.0f, clip.durationSeconds),
        clip.durationSeconds, false).translationOffsetMeters;
    return { t.x, t.y, 0.0f };
}

float stanceAlignedPhase(const AnimationClip3D& clip, float cyclePhase) {
    const auto& contacts = clip.leftFootContact.keyframes;
    if (contacts.empty() || clip.durationSeconds <= 0.0f) {
        return cyclePhase;
    }
    const auto strongest = std::max_element(contacts.begin(), contacts.end(),
        [](const AnimationScalarKeyframe3D& left,
            const AnimationScalarKeyframe3D& right) {
            return left.value < right.value;
        });
    const float offset = strongest->timeSeconds / clip.durationSeconds;
    return std::fmod(cyclePhase + offset, 1.0f);
}

constexpr float LocomotionAcceptanceStiffness = 0.75f;
// Pe com mola (input.footSpring): freio do tornozelo no ar, e no toque a
// mola (fracao da rigidez), o freio extra e quanto tempo dura.
constexpr float AnkleAirDamping = 0.6f;
constexpr float AnkleLandingStiffness = 0.4f;
constexpr float AnkleLandingDamping = 0.4f;
constexpr float AnkleLandingSeconds = 0.15f;
// Quanto o pe que vai pousar fica paralelo ao chao de pouso (o resto e
// a inclinacao do clipe).
constexpr float FootLandingFlatten = 0.7f;
// Tornozelo contra a queda: rad de pressao por metro de desvio do ponto de
// captura e os limites (ponta/calcanhar, borda).
constexpr float AnkleBalanceGain = 3.5f;
constexpr float AnkleBalancePitchLimit = 0.10f;
constexpr float AnkleBalanceRollLimit = 0.04f;
constexpr float AnkleBalanceDeadband = 0.012f;
// E5 (animacao + pes conscientes): fracao da diferenca de ponto de captura
// que o pouso corrige, e o limite (m).
constexpr float BalancePlacementGain = 0.6f;
constexpr float BalancePlacementLimitMeters = 0.15f;
// Janela do empurrao: a partir desta fracao do balanco do pe que vem.
constexpr float LocomotionPushWindow = 0.67f;

// Freada (modo fisico): o corpo leva um momento da propria corrida que o
// jogador nao pede mais (soltou o comando ou pediu o contrario). Calibrada
// pela varredura de freada (MATTERENGINE_TEST_FILTER=sweepbrake, 64 casos)
// e medida contra as referencias Run To Stop e Running To Stop.
// Entra acima desta velocidade do corpo com tanto a mais que o pedido no
// rumo dele (m/s); sai abaixo desta (ou quando o pedido alcanca o corpo).
constexpr float BrakeEngageSpeed = 2.0f;
constexpr float BrakeEngageExcessSpeed = 1.2f;
constexpr float BrakeReleaseSpeed = 0.5f;
// Pedindo o sentido contrario, a freada acaba com o corpo abaixo disto (m/s)
// e a passada ja arranca para o outro lado. Inversao na varredura de marcha
// (XGAITINVERT): 1,0 levava 0,76/0,44 s (sprint/trote, 1 queda em 8); 2,0,
// 0,69/0,38 s sem queda; 2,5 e 3,0 mais lentos (a passada inverte mal
// rapida).
constexpr float BrakeReverseReleaseSpeed = 2.0f;
// Freando ha mais que isto (s), termina de qualquer jeito.
constexpr float BrakeMaximumSeconds = 4.0f;
// Depois da freada, tanto tempo (s) sem acomodar nem girar os pes.
constexpr float BrakeSettleHoldSeconds = 1.5f;
// Perfil de freada da corrida (m/s2) e quanto ele fica abaixo do corpo no
// maximo (m/s). (6 parava o sprint em ~1,4 s; 12, o mesmo sem quedas, em
// ~1,1 s - o que freia de fato e o corpo, o perfil so nao segura.)
constexpr float BrakeDeceleration = 12.0f;
constexpr float BrakeProfileLeash = 0.8f;
// Abaixo disto (m/s do corpo) a corrida para e comecam os passos de freada
// pelas pernas: 2,2 deu 0-3 quedas em 32 sprints; 2,5 e 2,9, 7-22 (o passo
// de freada a 3 m/s nao alcanca a frente do corpo a tempo); 1,8, nenhuma e
// menos passos.
constexpr float BrakeStepSpeed = 1.8f;
// A troca para os passos espera um pe no chao no maximo isto atras do
// centro de massa (m), ou ate este tempo (s) abaixo da velocidade dos passos.
constexpr float BrakePlantedAhead = -0.05f;
constexpr float BrakeStepWaitSeconds = 0.15f;
// Os passos de freada nao deixam a pelve subir so com o corpo rapido: o
// teto some entre estas velocidades (m/s, inicio e faixa).
constexpr float BrakeCeilingSlowSpeed = 0.5f;
constexpr float BrakeCeilingSpan = 0.7f;
// Parado sobre os pes, a base freia o resto da velocidade do corpo nesta
// desaceleracao (m/s2), presa a ele por esta folga (m). (4 e 8 m/s2 e
// folga de 5 cm nao mudaram ou pioraram os passos depois da parada.)
constexpr float StopHoldDeceleration = 2.0f;
constexpr float StopHoldLeash = 0.10f;
// A freada some em (s) depois de terminar; a ajuda de corrida, depois de
// os passos comecarem. (0,1 s nao mudou a resposta ao comando no meio da
// freada: a diferenca e o corpo ja mais lento.)
constexpr float BrakeFadeSeconds = 0.30f;
constexpr float BrakeRunFadeSeconds = 0.15f;
// Correndo e freando, o ciclo do sprint so acima disto (m/s); abaixo, o
// trote.
constexpr float BrakeSprintClipSpeed = 4.6f;
constexpr float BrakeReactionScale = 1.0f;
// Coleira da referencia da corrida freando ao corpo (m).
constexpr float BrakeReferenceLeash = 0.30f;
// Agachamento dos passos de freada (m), a faixa de velocidade em que ele
// entra (m/s acima de 0,6) e a mola (rad/s). Nas referencias a pelve desce
// ~25 cm; 22 cm aqui nao mudou a varredura.
constexpr float BrakeCrouchMeters = 0.16f;
constexpr float BrakeCrouchSpeedSpan = 2.4f;
constexpr float BrakeCrouchOmega = 14.0f;
// Nos passos de freada a pelve de referencia fica no maximo isto acima da
// medida (m): a perna de apoio cede ou segura, nunca estende.
constexpr float BrakeRiseMeters = 0.01f;

float footSoleOffset(const RagdollLinkDefinition3D& link) {
    return link.collider.shape == RagdollColliderShape3D::Box
        ? link.collider.boxHalfExtents.z
        : link.collider.radiusMeters;
}

// How far the collider's lowest point sits below its centre, for the
// orientation it is actually in. Treating a foot as a flat slab of
// halfExtents.z underestimates this the moment the ankle pitches - a 12
// degree pitch on a 30 cm foot puts the toe ~3 cm lower than that - and the
// grounding below then leaves the toe buried, which the floor answers with
// an enormous contact force.
float lowestColliderOffset(const RagdollLinkDefinition3D& link,
    Quaternion colliderWorldOrientation) {
    const Vec3 up = colliderWorldOrientation.conjugate().rotate(
        { 0.0f, 0.0f, 1.0f });
    if (link.collider.shape == RagdollColliderShape3D::Box) {
        const Vec3 half = link.collider.boxHalfExtents;
        return std::abs(up.x) * half.x + std::abs(up.y) * half.y
            + std::abs(up.z) * half.z;
    }
    // Capsule: the cylinder runs along local X, capped by the radius.
    return std::abs(up.x) * std::max(0.0f,
        link.collider.lengthMeters * 0.5f - link.collider.radiusMeters)
        + link.collider.radiusMeters;
}

Vec3 rotationVector(Quaternion value) {
    value = value.normalized();
    if (value.w < 0.0f) {
        value = { -value.x, -value.y, -value.z, -value.w };
    }
    const Vec3 imaginary { value.x, value.y, value.z };
    const float length = imaginary.length();
    if (length < 0.000001f) return {};
    const float angle = 2.0f * std::atan2(length, value.w);
    return imaginary * (angle / length);
}

// Raiz de um clipe num instante (deslocamento e rotacao, referencial do
// clipe).
AnimationTransformSample3D clipRootAt(const RagdollProfile3D& profile,
    const AnimationClip3D& clip, float time) {
    AnimationTransformSample3D sample;
    if (const AnimationTrack3D* track = findAnimationTrack3D(clip, profile.links.front().id)) {
        sample = sampleAnimationTrack3D(*track, std::clamp(time, 0.0f, clip.durationSeconds),
            clip.durationSeconds, false);
    }
    return sample;
}

Vec3 clampMagnitude(Vec3 value, float maximumLength) {
    const float squared = value.lengthSquared();
    if (squared <= maximumLength * maximumLength || squared < 0.000001f) {
        return value;
    }
    return value * (maximumLength / std::sqrt(squared));
}

void addJointCoordinate(const RagdollProfile3D& profile,
    std::vector<Vec3>& coordinates, std::string_view linkId,
    std::size_t axis, float amount) {
    for (std::size_t index = 1; index < profile.links.size(); ++index) {
        if (profile.links[index].id != linkId || index >= coordinates.size()) {
            continue;
        }
        const RagdollAxisDefinition3D& limit =
            profile.links[index].inboundJoint.axes[axis];
        if (!limit.enabled) return;
        setComponent(coordinates[index], axis,
            std::clamp(component(coordinates[index], axis) + amount,
                limit.minimumRadians, limit.maximumRadians));
        return;
    }
}

// Uma coordenada de junta levada para `wanted` por `amount` (0..1), dentro
// dos limites.
void blendJointCoordinate(const RagdollProfile3D& profile,
    std::vector<Vec3>& coordinates, std::size_t index, std::size_t axis,
    float wanted, float amount) {
    if (index == 0 || index >= profile.links.size()
        || index >= coordinates.size()) return;
    const RagdollAxisDefinition3D& limit =
        profile.links[index].inboundJoint.axes[axis];
    if (!limit.enabled) return;
    const float current = component(coordinates[index], axis);
    setComponent(coordinates[index], axis, std::clamp(
        current + (wanted - current) * amount,
        limit.minimumRadians, limit.maximumRadians));
}

float varietyNoise(std::uint32_t seed, std::uint32_t channel, float position);

// Reflexos de quem esta caindo, so alvos de junta (os motores fazem o
// resto; nada e escrito no corpo): as maos vao ao chao no rumo da queda, a
// cabeca se protege (queixo no peito caindo de costas, erguida caindo de
// frente, inclinada para fora de lado), a coluna enrola um pouco caindo de
// costas, joelhos e quadris cedem. `amount` mistura com a pose de antes.
void applyFallReflexPose(const RagdollProfile3D& profile,
    const RagdollState3D& state, std::vector<Vec3>& coordinates,
    std::array<std::size_t, 2> hands, std::size_t chest, Vec3 fallDirection,
    float groundHeight, float amount, float legAmount,
    std::array<float, 2> catchSeconds, float time, std::uint32_t seed) {
    if (amount <= 0.001f || state.links.size() != profile.links.size()
        || chest >= state.links.size()) return;
    const auto indexOf = [&](std::string_view id) {
        for (std::size_t i = 1; i < profile.links.size(); ++i)
            if (profile.links[i].id == id) return i;
        return std::size_t { 0 };
    };
    const auto planarUnit = [](Vec3 v, Vec3 fallback) {
        v.z = 0.0f;
        return v.lengthSquared() > 0.0004f ? v.normalized() : fallback;
    };
    const Vec3 fall = planarUnit(fallDirection, { 1.0f, 0.0f, 0.0f });
    const Vec3 chestForward = planarUnit(
        state.links[chest].orientation.rotate({ 1.0f, 0.0f, 0.0f }), fall);
    const Vec3 chestLeft = planarUnit(
        state.links[chest].orientation.rotate({ 0.0f, 1.0f, 0.0f }),
        cross({ 0.0f, 0.0f, 1.0f }, chestForward));
    const float ahead = dot(fall, chestForward);
    const float aside = dot(fall, chestLeft);
    const float backward = std::clamp(-ahead, 0.0f, 1.0f);
    const float forward = std::clamp(ahead, 0.0f, 1.0f);
    // Cabeca e pescoco (swing1 + abaixa o queixo; swing2 + inclina para a
    // esquerda): caindo para a esquerda, a cabeca vai para a direita.
    const float nod = 0.95f * backward - 0.55f * forward + 0.20f * std::abs(aside);
    const float tilt = -0.75f * aside;
    blendJointCoordinate(profile, coordinates, indexOf("Neck"), 1, nod * 0.4f, amount);
    blendJointCoordinate(profile, coordinates, indexOf("Head"), 1, nod * 0.6f, amount);
    blendJointCoordinate(profile, coordinates, indexOf("Neck"), 2, tilt * 0.4f, amount);
    blendJointCoordinate(profile, coordinates, indexOf("Head"), 2, tilt * 0.6f, amount);
    // Coluna: enrola caindo de costas, inclina para fora de lado.
    for (const char* id : { "Abdomen", "Chest", "UpperChest" }) {
        blendJointCoordinate(profile, coordinates, indexOf(id), 1, 0.14f * backward, amount);
        blendJointCoordinate(profile, coordinates, indexOf(id), 2, -0.10f * aside, amount);
    }
    // Tenta girar o corpo para o lado da queda, virando o peito e as maos
    // para o chao: de costas ele cai de lado (quadril e mao), nao chapado;
    // de lado vira o peito para o chao; de frente, pouco. Caindo reto para
    // tras, gira para a esquerda.
    if (FallProtectNew && FallTwistRadians > 0.0f) {
        const float side = std::clamp(std::abs(aside), 0.0f, 1.0f);
        const float turn = (aside >= 0.0f ? 1.0f : -1.0f) * FallTwistRadians
            * std::clamp(backward + 0.6f * side + 0.25f * forward, 0.0f, 1.0f) / 3.0f;
        // (So com a queda sem volta: no desequilibrio o tronco torcido
        // atrapalhava o passo que ainda salvava.)
        for (const char* id : { "Abdomen", "Chest", "UpperChest" })
            blendJointCoordinate(profile, coordinates, indexOf(id), 0, turn, legAmount);
    }
    // Pernas cedendo (quadril: swing1 - flexiona; joelho: twist + flexiona),
    // so com a queda sem volta: antes elas ainda estao dando passos.
    // Pelo rumo: de frente ajoelha (cai nos joelhos e nas maos, como na
    // referencia Tripping), de costas senta (cai sentado, nao de costas
    // retas), de lado os joelhos cedem - menos altura para cair.
    if (FallProtectNew && FallLegsNew) {
        const float side = std::clamp(std::abs(aside), 0.0f, 1.0f);
        const float knee = FallKneeForward * forward + FallKneeBackward * backward + FallKneeSide * side;
        const float hip = FallHipForward * forward + FallHipBackward * backward + FallHipSide * side;
        // Cada perna se ajeita no seu ritmo (procurando apoio), nao parada.
        const char* thighs[2] = { "LeftThigh", "RightThigh" };
        const char* shins[2] = { "LeftShin", "RightShin" };
        for (std::size_t leg = 0; leg < 2; ++leg) {
            // (So com a queda ja certa: parcialmente entregue, as pernas ainda
            // dao o passo que salva - mexendo, custavam 2 quedas em 216.)
            const float drift = FallLife * smoothStep((legAmount - 0.7f) / 0.3f)
                * varietyNoise(seed, 60 + static_cast<std::uint32_t>(leg), time * 0.9f + 2.3f * leg);
            blendJointCoordinate(profile, coordinates, indexOf(thighs[leg]), 1, -hip + 0.30f * drift, legAmount);
            blendJointCoordinate(profile, coordinates, indexOf(shins[leg]), 0, knee + 0.40f * drift, legAmount);
        }
    } else {
    for (const char* id : { "LeftThigh", "RightThigh" })
        blendJointCoordinate(profile, coordinates, indexOf(id), 1, -0.35f, legAmount * 0.7f);
    for (const char* id : { "LeftShin", "RightShin" })
        blendJointCoordinate(profile, coordinates, indexOf(id), 0, 0.55f, legAmount * 0.7f);
    }
    // Bracos: a mao vai para o chao no rumo da queda, cada uma do seu lado,
    // ate onde o braco alcanca (desligado: ver FallArmsReflex).
    const auto& root = state.links.front();
    const Quaternion rootInverse = root.orientation.conjugate();
    for (std::size_t side = 0; side < 2 && FallArmsReflex; ++side) {
        const std::size_t hand = hands[side];
        if (hand == 0 || hand >= profile.links.size()) continue;
        const int forearm = profile.links[hand].parentIndex;
        const int upper = forearm > 0
            ? profile.links[static_cast<std::size_t>(forearm)].parentIndex : -1;
        if (upper <= 0) continue;
        // O braco tenta: estende ate o chao no rumo da queda (de lado e de
        // costas tambem - nao precisa segurar o corpo, so tentar).
        const bool extendArm = FallProtectNew && FallArmsNew;
        Vec3 shoulder = state.links[static_cast<std::size_t>(upper)].position;
        float armReach = 0.52f;
        const float s = side == 0 ? 1.0f : -1.0f;
        if (extendArm) {
            // Do ombro de verdade (a articulacao, nao o meio do braco) e com o
            // braco quase todo estendido: medido do link, o alvo ficava perto
            // e o cotovelo ia a ~95 graus antes de tocar, a mao na altura do
            // quadril (na referencia, ~50 graus e a mao bem a frente).
            const RagdollLinkDefinition3D& upperLink = profile.links[static_cast<std::size_t>(upper)];
            const RagdollLinkDefinition3D& forearmLink = profile.links[static_cast<std::size_t>(forearm)];
            const RagdollLinkDefinition3D& handLink = profile.links[hand];
            const PhysicsBodyState3D& upperBody = state.links[static_cast<std::size_t>(upper)];
            shoulder = upperBody.position + upperBody.orientation.rotate(upperLink.modelOrientation.conjugate()
                .rotate(upperLink.inboundJoint.anchorModelPosition - upperLink.modelPosition));
            armReach = FallArmExtension * ((forearmLink.inboundJoint.anchorModelPosition
                    - upperLink.inboundJoint.anchorModelPosition).length()
                + (handLink.inboundJoint.anchorModelPosition - forearmLink.inboundJoint.anchorModelPosition).length());
        }
        // Quanto a mao vai no rumo da queda: de frente longe (as maos bem a
        // frente, como na referencia); de lado e para tras perto, embaixo do
        // ombro (o braco vira coluna; esticado 0,7 m para fora, o ombro e a
        // cabeca passavam dele e a cabeca batia a 2,5 m/s). O braco do lado
        // de cima, numa queda de lado, so acompanha.
        float fallReach = 0.32f;
        if (extendArm) {
            const float sideAmount = std::clamp(std::abs(aside), 0.0f, 1.0f);
            fallReach = FallArmForward * forward + FallArmSideReach * sideAmount + FallArmBackReach * backward;
            const bool farArm = aside * s < 0.0f;
            if (farArm) fallReach *= 1.0f - FallFarArmShare * sideAmount;
        }
        Vec3 target = shoulder + fall * fallReach + chestLeft * (s * 0.16f)
            // A mao procura o chao: o ponto anda um pouco (ruido lento).
            + (fall * varietyNoise(seed, 62 + static_cast<std::uint32_t>(side), time * 1.1f)
                + chestLeft * varietyNoise(seed, 64 + static_cast<std::uint32_t>(side), time * 1.1f + 5.0f))
                * (0.12f * FallLife * legAmount);
        target.z = groundHeight + 0.06f;
        if (extendArm) {
            // Em duas etapas, como na referencia Tripping: no desequilibrio
            // os bracos sobem a frente, dobrados, prontos (a mao na frente do
            // peito); so com a queda sem volta (legAmount) estendem ate o
            // chao. Esticados ao chao logo no comeco, eles levavam o centro
            // de massa para o lado da queda e o passo nao salvava mais
            // (empurroes: 42 -> 46 quedas em 216).
            // (Caindo de costas, prontos a frente do peito, nao no rumo da
            // queda: os bracos para tras levavam o centro de massa junto e um
            // empurrao de 300 N.s que os passos salvavam o derrubava; a frente
            // eles contrabalancam, como quem cai de costas faz.)
            Vec3 readyDirection = fall * (1.0f - backward) + chestForward * backward;
            readyDirection.z = 0.0f;
            readyDirection = readyDirection.lengthSquared() > 1e-4f ? readyDirection.normalized() : chestForward;
            Vec3 ready = shoulder + readyDirection * FallArmReadyForward + chestLeft * (s * 0.10f);
            // Prontos, mas buscando o equilibrio: sobem e descem, abrem e
            // fecham devagar.
            // (So com a queda certa: antes, ainda dando para se salvar com os
            // passos, os bracos mexendo custavam 2 quedas em 216 empurroes.)
            const float lifeNow = FallLife * legAmount;
            ready += chestLeft * (s * 0.12f * lifeNow * varietyNoise(seed, 66 + static_cast<std::uint32_t>(side), time * 1.2f));
            ready.z = shoulder.z - FallArmReadyDrop
                + 0.14f * lifeNow * varietyNoise(seed, 68 + static_cast<std::uint32_t>(side), time * 1.2f + 3.0f);
            // (Ou com o ombro ja perto do chao: num empurrao forte a mao
            // tocava antes de a queda ser dada como certa, ainda dobrada na
            // pose pronta - cotovelo a ~80 graus.)
            const float near = smoothStep((FallArmNearHeight - (shoulder.z - groundHeight)) / 0.40f);
            const float ground = std::max(smoothStep(legAmount), near);
            target = ready + (target - ready) * ground;
        }
        Vec3 reach = target - shoulder;
        if (reach.length() > armReach) target = shoulder + reach * (armReach / reach.length());
        if (extendArm) {
            // O punho ate o alvo movendo cotovelo e ombro (a partir do
            // antebraco). Pela mao com profundidade 2 o IK mexia o punho e o
            // cotovelo - nunca o ombro: o braco nao apontava para o chao e a
            // mao so chegava perto dobrando o cotovelo a ~95-125 graus.
            const RagdollLinkDefinition3D& forearmLink = profile.links[static_cast<std::size_t>(forearm)];
            const Vec3 wristInForearm = forearmLink.modelOrientation.conjugate().rotate(
                profile.links[hand].inboundJoint.anchorModelPosition - forearmLink.modelPosition);
            solveChainPoint(profile, coordinates, static_cast<std::size_t>(forearm), wristInForearm, 2,
                rootInverse.rotate(target - root.position), amount, static_cast<int>(FallArmIterations));
        } else {
            solveChainPoint(profile, coordinates, hand, {}, 2,
                rootInverse.rotate(target - root.position), amount);
        }
        // Cotovelo nunca travado reto: cede no impacto.
        const float elbow = component(coordinates[static_cast<std::size_t>(forearm)], 0);
        if (!FallProtectNew) {
            if (elbow < 0.30f)
                blendJointCoordinate(profile, coordinates, static_cast<std::size_t>(forearm), 0, 0.30f, amount);
        } else {
            // Estende para o chao (pouco dobrado) e, depois que a mao toca,
            // cede aos poucos: o braco amortece a descida e a cabeca chega
            // devagar (na referencia o cotovelo vai de ~55 a ~85 graus).
            const float absorb = smoothStep(catchSeconds[side] / FallAbsorbSeconds);
            const float wanted = FallElbowReach + FallElbowAbsorb * absorb;
            if (elbow < wanted)
                blendJointCoordinate(profile, coordinates, static_cast<std::size_t>(forearm), 0, wanted, amount);
        }
    }
}

// No chao, rolando ou parado depois de cair (o "fetal" do Euphoria): se
// fecha - joelhos ao peito, coluna enrolada, queixo no peito e as maos na
// frente do rosto protegendo a cabeca. Pose estavel, sem depender do rumo
// da queda: os reflexos, que miram o rumo, no chao faziam os alvos pularem.
void applyFetalPose(const RagdollProfile3D& profile,
    const RagdollState3D& state, std::vector<Vec3>& coordinates,
    std::array<std::size_t, 2> hands, std::size_t chest, float amount,
    float time, std::uint32_t seed, float struggle) {
    if (amount <= 0.001f || state.links.size() != profile.links.size()
        || chest >= state.links.size()) return;
    const auto indexOf = [&](std::string_view id) {
        for (std::size_t i = 1; i < profile.links.size(); ++i)
            if (profile.links[i].id == id) return i;
        return std::size_t { 0 };
    };
    // Nao fica parado: cada perna no seu ritmo (ruido lento, ~0,5 Hz) - um
    // joelho sobe ao peito enquanto o outro estica e empurra o chao -, a
    // cabeca vira e o tronco torce um pouco. Mais forte enquanto ainda rola.
    const float life = GroundLife * (0.6f + 0.4f * struggle);
    const float pace = time * GroundLifeRate;
    const char* thighs[2] = { "LeftThigh", "RightThigh" };
    const char* shins[2] = { "LeftShin", "RightShin" };
    for (std::size_t side = 0; side < 2; ++side) {
        const float drift = varietyNoise(seed, 40 + static_cast<std::uint32_t>(side), pace + 3.7f * side);
        blendJointCoordinate(profile, coordinates, indexOf(thighs[side]), 1,
            -FetalHip + 0.70f * life * drift, amount);
        blendJointCoordinate(profile, coordinates, indexOf(shins[side]), 0,
            FetalKnee + 0.85f * life * drift + 0.25f * life * varietyNoise(seed, 42 + static_cast<std::uint32_t>(side), pace * 1.3f), amount);
    }
    for (const char* id : { "Abdomen", "Chest", "UpperChest" })
        blendJointCoordinate(profile, coordinates, indexOf(id), 0,
            0.12f * life * varietyNoise(seed, 44, pace * 0.7f), amount);
    blendJointCoordinate(profile, coordinates, indexOf("Head"), 0,
        0.45f * life * varietyNoise(seed, 45, pace * 0.6f), amount);
    blendJointCoordinate(profile, coordinates, indexOf("Abdomen"), 1, 0.40f, amount);
    blendJointCoordinate(profile, coordinates, indexOf("Chest"), 1, 0.35f, amount);
    blendJointCoordinate(profile, coordinates, indexOf("UpperChest"), 1, 0.30f, amount);
    blendJointCoordinate(profile, coordinates, indexOf("Neck"), 1, 0.28f, amount);
    blendJointCoordinate(profile, coordinates, indexOf("Head"), 1, 0.40f, amount);
    // Maos na frente do rosto, cotovelos dobrados junto ao peito.
    const std::size_t head = indexOf("Head");
    if (head == 0) return;
    const auto& root = state.links.front();
    const Quaternion rootInverse = root.orientation.conjugate();
    const Quaternion chestOrientation = state.links[chest].orientation;
    const Vec3 forward = chestOrientation.rotate({ 1.0f, 0.0f, 0.0f });
    const Vec3 left = chestOrientation.rotate({ 0.0f, 1.0f, 0.0f });
    const Vec3 up = chestOrientation.rotate({ 0.0f, 0.0f, 1.0f });
    for (std::size_t side = 0; side < 2; ++side) {
        const std::size_t hand = hands[side];
        if (hand == 0 || hand >= profile.links.size()) continue;
        const int forearm = profile.links[hand].parentIndex;
        if (forearm <= 0) continue;
        const float s = side == 0 ? 1.0f : -1.0f;
        // As maos rondam a cabeca e de vez em quando vao ao chao empurrar.
        const float reach = varietyNoise(seed, 46 + static_cast<std::uint32_t>(side), pace * 0.8f + 1.9f * side);
        const Vec3 target = state.links[head].position + forward * (0.14f + 0.16f * life * reach)
            + left * (s * (0.09f + 0.12f * life * varietyNoise(seed, 48 + static_cast<std::uint32_t>(side), pace)))
            - up * (0.06f + 0.22f * life * std::max(0.0f, reach));
        const RagdollLinkDefinition3D& forearmLink = profile.links[static_cast<std::size_t>(forearm)];
        const Vec3 wristInForearm = forearmLink.modelOrientation.conjugate().rotate(
            profile.links[hand].inboundJoint.anchorModelPosition - forearmLink.modelPosition);
        solveChainPoint(profile, coordinates, static_cast<std::size_t>(forearm), wristInForearm, 2,
            rootInverse.rotate(target - root.position), amount, 12);
    }
}

// No ar, caindo sem tocar nada: tenta manter uma pose e se equilibrar, como
// os bonecos do GTA (Euphoria) - bracos abertos para os lados um pouco abaixo
// da horizontal, cotovelos meio dobrados, uma perna com o joelho erguido e a
// outra estendida, trocando devagar; movimento pequeno e lento, firme. (O
// espernear antigo, rapido e em fases soltas, parecia convulsao.)
void applyAirBalance(const RagdollProfile3D& profile,
    std::vector<Vec3>& coordinates, float time, float amount, float spin) {
    if (amount <= 0.0f) return;
    const auto indexOf = [&](std::string_view id) {
        for (std::size_t i = 1; i < profile.links.size(); ++i)
            if (profile.links[i].id == id) return i;
        return std::size_t { 0 };
    };
    constexpr float Turn = 2.0f * Pi;
    // Bracos: abertos (eixo 1 ~+35 graus; pendurados sao ~+70), um pouco a
    // frente, balancando pouco e devagar, em oposicao.
    // Tentando se equilibrar: os bracos giram em moinho (circulos, um meio
    // ciclo atras do outro) no sentido do giro do corpo - quanto mais ele
    // gira, maior o circulo.
    const float struggle = std::clamp(std::abs(spin) / 3.0f, 0.0f, 1.0f);
    const float radius = AirWindmill * (0.5f + 0.5f * struggle);
    const float phase = (spin >= 0.0f ? 1.0f : -1.0f) * Turn * AirWindmillRate * time;
    blendJointCoordinate(profile, coordinates, indexOf("LeftUpperArm"), 1, AirArmOpen + radius * std::sin(phase), amount);
    blendJointCoordinate(profile, coordinates, indexOf("RightUpperArm"), 1, AirArmOpen + radius * std::sin(phase + Pi), amount);
    blendJointCoordinate(profile, coordinates, indexOf("LeftUpperArm"), 2, 0.20f + radius * std::cos(phase), amount);
    blendJointCoordinate(profile, coordinates, indexOf("RightUpperArm"), 2, 0.20f + radius * std::cos(phase + Pi), amount);
    blendJointCoordinate(profile, coordinates, indexOf("LeftUpperArm"), 0, 0.0f, amount);
    blendJointCoordinate(profile, coordinates, indexOf("RightUpperArm"), 0, 0.0f, amount);
    blendJointCoordinate(profile, coordinates, indexOf("LeftForearm"), 0, 0.55f, amount);
    blendJointCoordinate(profile, coordinates, indexOf("RightForearm"), 0, 0.55f, amount);
    // Pernas pedalando (uma sobe enquanto a outra desce e estica), mais
    // largas quanto mais ele gira.
    const float pedal = Turn * AirPedalRate * time;
    const float stride = 0.6f + 0.4f * struggle;
    const auto leg = [&](const char* thigh, const char* shin, float offset) {
        const float cycle = pedal + offset;
        blendJointCoordinate(profile, coordinates, indexOf(thigh), 1,
            -(0.55f + 0.40f * stride * std::sin(cycle)), amount);
        blendJointCoordinate(profile, coordinates, indexOf(shin), 0,
            0.95f + 0.55f * stride * std::sin(cycle + 1.2f), amount);
    };
    leg("LeftThigh", "LeftShin", 0.0f);
    leg("RightThigh", "RightShin", Pi);
    // Tronco e cabeca: retos, olhando um pouco para baixo (para o chao).
    for (const char* id : { "Abdomen", "Chest", "UpperChest" })
        blendJointCoordinate(profile, coordinates, indexOf(id), 1, 0.05f, amount);
    blendJointCoordinate(profile, coordinates, indexOf("Neck"), 1, 0.10f, amount);
    blendJointCoordinate(profile, coordinates, indexOf("Head"), 1, 0.15f, amount);
}

void setJointCoordinate(const RagdollProfile3D& profile,
    std::vector<Vec3>& coordinates, std::string_view linkId,
    std::size_t axis, float value) {
    for (std::size_t index = 1; index < profile.links.size(); ++index) {
        if (profile.links[index].id != linkId || index >= coordinates.size()) {
            continue;
        }
        const RagdollAxisDefinition3D& limit =
            profile.links[index].inboundJoint.axes[axis];
        if (!limit.enabled) return;
        setComponent(coordinates[index], axis,
            std::clamp(value, limit.minimumRadians, limit.maximumRadians));
        return;
    }
}

// Perdendo o equilibrio (antes de cair): quem tenta nao cair abre os bracos
// e gira em moinho contra o sentido da queda, dobra o tronco para o lado
// oposto, cede os joelhos e acompanha com a cabeca. So alvos de junta - os
// motores fazem, a fisica decide se adianta. (Como nos personagens do
// Euphoria: ninguem fica duro esperando cair.)
void applyBalanceReaction(const RagdollProfile3D& profile,
    const RagdollState3D& state, std::vector<Vec3>& coordinates,
    std::array<std::size_t, 2> hands, std::size_t chest, Vec3 fallDirection,
    float amount, float time, std::array<bool, 2> braced, float legAmount) {
    if (amount <= 0.001f || state.links.size() != profile.links.size()
        || chest >= state.links.size()) return;
    const auto planarUnit = [](Vec3 v, Vec3 fallback) {
        v.z = 0.0f;
        return v.lengthSquared() > 0.0004f ? v.normalized() : fallback;
    };
    const Vec3 fall = planarUnit(fallDirection, { 1.0f, 0.0f, 0.0f });
    const Vec3 chestForward = planarUnit(
        state.links[chest].orientation.rotate({ 1.0f, 0.0f, 0.0f }), fall);
    const Vec3 chestLeft = planarUnit(
        state.links[chest].orientation.rotate({ 0.0f, 1.0f, 0.0f }),
        cross({ 0.0f, 0.0f, 1.0f }, chestForward));
    const float ahead = dot(fall, chestForward);
    const float aside = dot(fall, chestLeft);
    // Tronco contra a queda (flexao + para a frente; lateral + para a
    // esquerda): caindo para a frente, estende; para tras, dobra.
    for (const char* id : { "Abdomen", "Chest", "UpperChest" }) {
        addJointCoordinate(profile, coordinates, id, 1, BalanceTrunkPitch * ahead * amount);
        addJointCoordinate(profile, coordinates, id, 2, BalanceTrunkRoll * aside * amount);
    }
    // Cabeca olha para onde vai e baixa um pouco (procura o chao).
    addJointCoordinate(profile, coordinates, "Neck", 0, 0.20f * aside * amount);
    addJointCoordinate(profile, coordinates, "Head", 0, 0.25f * aside * amount);
    addJointCoordinate(profile, coordinates, "Head", 1, 0.12f * amount);
    // Joelhos e quadril cedem: baixa o centro de massa.
    // (So de pe: na passada as pernas sao dela - correndo com pouca ajuda, o
    // joelho cedendo encurtava as pernas, os pes nao alcancavam o chao por
    // 0,5 s e ele caia de joelhos.)
    for (const char* id : { "LeftThigh", "RightThigh" })
        addJointCoordinate(profile, coordinates, id, 1, -0.16f * legAmount);
    for (const char* id : { "LeftShin", "RightShin" })
        addJointCoordinate(profile, coordinates, id, 0, 0.28f * legAmount);
    // Bracos: abertos, erguidos e contra a queda, girando em moinho (um fora
    // de fase do outro).
    const auto& root = state.links.front();
    const Quaternion rootInverse = root.orientation.conjugate();
    const Vec3 up { 0.0f, 0.0f, 1.0f };
    for (std::size_t side = 0; side < 2; ++side) {
        const std::size_t hand = hands[side];
        if (hand == 0 || hand >= profile.links.size()) continue;
        const int forearm = profile.links[hand].parentIndex;
        const int upper = forearm > 0
            ? profile.links[static_cast<std::size_t>(forearm)].parentIndex : -1;
        if (upper <= 0) continue;
        // Mao apoiada num obstaculo fica la e empurra (cotovelo estendendo):
        // e com ela que ele se afasta e se endireita.
        if (braced[side]) {
            blendJointCoordinate(profile, coordinates,
                static_cast<std::size_t>(forearm), 0, 0.15f, 0.8f * amount);
            continue;
        }
        const Vec3 shoulder = state.links[static_cast<std::size_t>(upper)].position;
        const float s = side == 0 ? 1.0f : -1.0f;
        const float phase = 2.0f * Pi * 1.7f * time + (side == 0 ? 0.0f : Pi);
        const float radius = 0.15f * amount;
        Vec3 target = shoulder + chestLeft * (s * 0.40f)
            + up * (0.02f + 0.14f * amount) - fall * (0.16f * amount)
            + fall * (radius * std::cos(phase)) + up * (radius * std::sin(phase));
        const Vec3 reach = target - shoulder;
        constexpr float ArmReach = 0.55f;
        if (reach.length() > ArmReach) target = shoulder + reach * (ArmReach / reach.length());
        if (BalanceArmFix) {
            // Do antebraco (cotovelo e ombro): pela mao com profundidade 2 o
            // IK so mexia punho e cotovelo - o ombro nunca girava e o moinho
            // quase nao acontecia.
            const RagdollLinkDefinition3D& forearmLink = profile.links[static_cast<std::size_t>(forearm)];
            const Vec3 wristInForearm = forearmLink.modelOrientation.conjugate().rotate(
                profile.links[hand].inboundJoint.anchorModelPosition - forearmLink.modelPosition);
            solveChainPoint(profile, coordinates, static_cast<std::size_t>(forearm), wristInForearm, 2,
                rootInverse.rotate(target - root.position), 0.85f * amount, 12);
        } else {
            solveChainPoint(profile, coordinates, hand, {}, 2,
                rootInverse.rotate(target - root.position), 0.85f * amount);
        }
    }
}

// Chute (04/10): o corpo do chute por cima da passada, pela forca. O braco
// do lado do apoio abre para fora e para a frente (equilibrio) enquanto a
// perna arma e varre para tras no acompanhamento; o do lado do chute vai para
// tras no armado e vem para a frente depois. O tronco gira abrindo o ombro do
// chute no armado e fecha batendo; inclina para o lado do apoio, para tras na
// bola alta e por cima dela na rasteira. A cabeca olha a bola.
// Repulsao do pe pela bola (04/10): quanto mover, no plano, o alvo do pe
// (origem do link, com a orientacao dele) para a caixa do pe ficar fora da
// bola com uma folga. So no plano: por cima ele pisaria nela.
Vec3 footBallPushOut(const RagdollLinkDefinition3D& link, Vec3 position, Quaternion orientation,
    Vec3 ball, float radius, float margin) {
    const RagdollCapsuleDefinition3D& collider = link.collider;
    if (collider.shape != RagdollColliderShape3D::Box) return {};
    const Vec3 half = collider.boxHalfExtents;
    const Quaternion box = (orientation * collider.localOrientation).normalized();
    Vec3 total {};
    for (int pass = 0; pass < 3; ++pass) {
        const Vec3 center = position + total + orientation.rotate(collider.localPosition);
        const Vec3 local = box.conjugate().rotate(ball - center);
        const Vec3 clamped { std::clamp(local.x, -half.x, half.x), std::clamp(local.y, -half.y, half.y),
            std::clamp(local.z, -half.z, half.z) };
        const Vec3 closest = center + box.rotate(clamped);
        const float deficit = radius + margin - (closest - ball).length();
        if (deficit <= 0.0f) break;
        Vec3 away { closest.x - ball.x, closest.y - ball.y, 0.0f };
        if (away.lengthSquared() < 1e-6f) away = { center.x - ball.x, center.y - ball.y, 0.0f };
        if (away.lengthSquared() < 1e-6f) away = { 1.0f, 0.0f, 0.0f };
        total += away.normalized() * deficit;
    }
    return total;
}

void applyKickPose(const RagdollProfile3D& profile, const RagdollState3D& state,
    std::vector<Vec3>& coordinates, std::array<std::size_t, 2> hands, const BallKickPose3D& kick) {
    if (!kick.active || kick.weight <= 0.001f || state.links.size() != profile.links.size()) return;
    const float w = std::clamp(kick.weight, 0.0f, 1.0f);
    const float p = std::clamp(kick.power, 0.0f, 1.0f);
    const float stage = kick.stage;
    const float windup = stage < 1.0f ? smoothStep(stage) : stage < 2.0f ? 1.0f - smoothStep(stage - 1.0f) : 0.0f;
    const float strike = stage < 1.0f ? 0.0f : stage < 2.0f ? smoothStep(stage - 1.0f) : 1.0f - smoothStep(stage - 2.0f);
    const float follow = stage < 2.0f ? 0.0f : smoothStep((stage - 2.0f) / 0.6f);
    const float open = stage < 1.0f ? smoothStep(stage) : stage < 2.0f ? 1.0f : 1.0f - 0.5f * smoothStep(stage - 2.0f);
    const float k = kick.side == 0 ? 1.0f : -1.0f;  // lado do chute, + esquerda
    const float high = std::max(0.0f, kick.lift), low = std::max(0.0f, -kick.lift);
    // Tronco (twist + gira para a esquerda; swing1 + flexiona; swing2 +
    // inclina para a esquerda), repartido nas tres juntas.
    const float twist = (KickTrunkTwist * windup - KickTrunkTwist * 0.8f * follow) * k * (0.3f + 0.7f * p);
    const float bend = (-KickTrunkBack * windup + (KickTrunkOver * (1.0f + low) - KickTrunkOver * 2.5f * high) * strike)
        * (0.3f + 0.7f * p);
    const float lateral = -k * KickTrunkSide * open * (0.3f + 0.7f * p);
    for (const char* id : { "Abdomen", "Chest", "UpperChest" }) {
        addJointCoordinate(profile, coordinates, id, 0, twist / 3.0f * w);
        addJointCoordinate(profile, coordinates, id, 1, bend / 3.0f * w);
        addJointCoordinate(profile, coordinates, id, 2, lateral / 3.0f * w);
    }
    addJointCoordinate(profile, coordinates, "Head", 1, 0.18f * (windup + strike) * w);
    // Bracos.
    const auto& root = state.links.front();
    const Quaternion rootInverse = root.orientation.conjugate();
    const Vec3 up { 0.0f, 0.0f, 1.0f };
    Vec3 aim { kick.aim.x, kick.aim.y, 0.0f };
    aim = aim.lengthSquared() > 1e-4f ? aim.normalized() : Vec3 { 1.0f, 0.0f, 0.0f };
    const Vec3 left = cross(up, aim);
    for (std::size_t side = 0; side < 2; ++side) {
        const std::size_t hand = hands[side];
        if (hand == 0 || hand >= profile.links.size()) continue;
        const int forearm = profile.links[hand].parentIndex;
        const int upper = forearm > 0 ? profile.links[static_cast<std::size_t>(forearm)].parentIndex : -1;
        if (upper <= 0) continue;
        const Vec3 shoulder = state.links[static_cast<std::size_t>(upper)].position;
        const float s = side == 0 ? 1.0f : -1.0f;
        // Bracos (04/10): o desenho da referencia (Goalkeeper Drop Kick - o
        // do apoio abre para o lado armando, vem para a frente na batida e
        // cruza acompanhando; o do chute vai para tras), mais contido e pela
        // forca, a pedido do usuario: chute fraco quase nada (abaixo de
        // KickArmFloor nem entra), so o forte abre de verdade; o rapido, menos.
        // O punho em relacao ao ombro, na mira: frente, para fora, cima.
        struct ArmKey { float forward; float out; float up; };
        const bool supportArm = static_cast<int>(side) != kick.side;
        const std::array<ArmKey, 4> support { ArmKey { 0.02f, 0.14f, -0.50f }, ArmKey { -0.12f, 0.34f, -0.12f },
            ArmKey { 0.26f, 0.22f, -0.28f }, ArmKey { 0.28f, -0.06f, -0.30f } };
        const std::array<ArmKey, 4> kicking { ArmKey { 0.02f, 0.14f, -0.50f }, ArmKey { -0.02f, 0.28f, -0.30f },
            ArmKey { -0.26f, 0.20f, -0.36f }, ArmKey { -0.30f, 0.06f, -0.38f } };
        const std::array<ArmKey, 4>& arm = supportArm ? support : kicking;
        // No tempo do gesto (0..1 armando, 1..2 batendo, 2..2,6 acompanhando),
        // por Catmull-Rom: o braco anda continuo, sem parar em cada quadro.
        const float times[4] = { 0.0f, 1.0f, 2.0f, 2.6f };
        const float at = std::clamp(stage, 0.0f, 2.6f);
        int segment = at < 1.0f ? 0 : at < 2.0f ? 1 : 2;
        const float h = times[segment + 1] - times[segment];
        const float u = (at - times[segment]) / h;
        const auto value = [&](int k, int which) {
            const ArmKey& a = arm[static_cast<std::size_t>(k)];
            return which == 0 ? a.forward : which == 1 ? a.out : a.up;
        };
        const auto tangent = [&](int k, int which) {
            if (k <= 0 || k >= 3) return 0.0f;
            return (value(k + 1, which) - value(k - 1, which)) / (times[k + 1] - times[k - 1]);
        };
        const float u2 = u * u, u3 = u2 * u;
        float curve[3];
        for (int which = 0; which < 3; ++which)
            curve[which] = (2.0f * u3 - 3.0f * u2 + 1.0f) * value(segment, which)
                + (u3 - 2.0f * u2 + u) * h * tangent(segment, which)
                + (-2.0f * u3 + 3.0f * u2) * value(segment + 1, which)
                + (u3 - u2) * h * tangent(segment + 1, which);
        // Quanto do desenho: pela forca (e menos no rapido; no toque parado,
        // quase nada).
        float share = smoothStep((p - KickArmFloor) / (KickArmFull - KickArmFloor));
        if (kick.quick) share *= 0.6f;
        if (!kick.shot) share *= 0.35f;
        const ArmKey rest = arm[0];
        const ArmKey key { rest.forward + (curve[0] - rest.forward) * share, rest.out + (curve[1] - rest.out) * share,
            rest.up + (curve[2] - rest.up) * share };
        Vec3 target = shoulder + aim * key.forward + left * (s * key.out) + up * key.up;
        const float amount = share;
        const Vec3 reach = target - shoulder;
        constexpr float ArmReach = 0.55f;
        if (reach.length() > ArmReach) target = shoulder + reach * (ArmReach / reach.length());
        const RagdollLinkDefinition3D& forearmLink = profile.links[static_cast<std::size_t>(forearm)];
        const Vec3 wristInForearm = forearmLink.modelOrientation.conjugate().rotate(
            profile.links[hand].inboundJoint.anchorModelPosition - forearmLink.modelPosition);
        solveChainPoint(profile, coordinates, static_cast<std::size_t>(forearm), wristInForearm, 2,
            rootInverse.rotate(target - root.position), std::clamp(amount * w, 0.0f, 1.0f), 10);
    }
}

// "Vida" no ragdoll, no ar: caindo sem tocar nada, bracos em moinho fora de
// fase e pernas pedalando - quem perdeu o chao tenta se equilibrar, nao fica
// estatico. So alvos de junta; o corpo faz com os motores. (Deitado nao ha
// movimento proprio: os "surtos" de tentar se recuperar pareciam um AVC, a
// pedido do usuario ficou so o ragdoll com um pouco de tonus.)
void applyAirborneFlail(const RagdollProfile3D& profile,
    std::vector<Vec3>& coordinates, float time, float flail) {
    constexpr float Turn = 2.0f * Pi;
    if (flail > 0.0f) {
        const float arm = Turn * 1.3f * time;
        addJointCoordinate(profile, coordinates, "LeftUpperArm", 1,
            0.60f * flail * std::sin(arm));
        addJointCoordinate(profile, coordinates, "RightUpperArm", 1,
            -0.60f * flail * std::sin(arm + 2.1f));
        addJointCoordinate(profile, coordinates, "LeftForearm", 0,
            0.35f * flail * (0.5f + 0.5f * std::sin(arm + 1.0f)));
        addJointCoordinate(profile, coordinates, "RightForearm", 0,
            0.35f * flail * (0.5f + 0.5f * std::sin(arm + 3.1f)));
        const float leg = Turn * 1.05f * time;
        addJointCoordinate(profile, coordinates, "LeftThigh", 1,
            -0.28f * flail * (0.6f + std::sin(leg)));
        addJointCoordinate(profile, coordinates, "RightThigh", 1,
            -0.28f * flail * (0.6f + std::sin(leg + Pi)));
        addJointCoordinate(profile, coordinates, "LeftShin", 0,
            0.45f * flail * (0.7f + 0.6f * std::sin(leg + 0.8f)));
        addJointCoordinate(profile, coordinates, "RightShin", 0,
            0.45f * flail * (0.7f + 0.6f * std::sin(leg + Pi + 0.8f)));
    }
}

// A compact, symmetric flight pose: no clip is sampled for it (the Mixamo
// jump references have big, one-sided arm swings this game does not want),
// only a hand-authored tuck calibrated against the ranges those references
// showed for hips/knees. runningShare blends toward a slightly flatter,
// forward-carried tuck for a jump taken at speed; tuckAmount is 0 at
// liftoff/landing and peaks mid-air.
void applyFlightPose(const RagdollProfile3D& profile,
    std::vector<Vec3>& coordinates, float tuckAmount, float runningShare) {
    tuckAmount = std::clamp(tuckAmount, 0.0f, 1.0f);
    runningShare = std::clamp(runningShare, 0.0f, 1.0f);
    const float hipFlexion = -(0.62f + runningShare * 0.18f) * tuckAmount;
    const float trailingHipFlexion = -(0.34f + runningShare * 0.30f)
        * tuckAmount;
    const float kneeFlexion = (1.55f - runningShare * 0.25f) * tuckAmount;
    const float trailingKneeFlexion = (0.95f - runningShare * 0.20f)
        * tuckAmount;
    const float anklePitch = -0.18f * tuckAmount;
    setJointCoordinate(profile, coordinates, "LeftThigh", 1, hipFlexion);
    setJointCoordinate(profile, coordinates, "LeftShin", 0, kneeFlexion);
    setJointCoordinate(profile, coordinates, "LeftFoot", 0, anklePitch);
    setJointCoordinate(profile, coordinates, "RightThigh", 1,
        trailingHipFlexion);
    setJointCoordinate(profile, coordinates, "RightShin", 0,
        trailingKneeFlexion);
    setJointCoordinate(profile, coordinates, "RightFoot", 0, anklePitch);
    setJointCoordinate(profile, coordinates, "Abdomen", 1,
        (0.09f + runningShare * 0.10f) * tuckAmount);
    setJointCoordinate(profile, coordinates, "Chest", 1,
        (0.06f + runningShare * 0.08f) * tuckAmount);
    // Arms stay symmetric and modest on purpose: braced, not flailing.
    setJointCoordinate(profile, coordinates, "LeftUpperArm", 1,
        0.55f * tuckAmount);
    setJointCoordinate(profile, coordinates, "RightUpperArm", 1,
        -0.55f * tuckAmount);
    setJointCoordinate(profile, coordinates, "LeftForearm", 0,
        0.95f * tuckAmount);
    setJointCoordinate(profile, coordinates, "RightForearm", 0,
        0.95f * tuckAmount);
}

// Variacao de movimento ("vida"): ruido de valor suave e deterministico. A
// mesma semente, canal e posicao dao sempre o mesmo valor (os testes se
// repetem); cada personagem tem a sua semente - o seu jeito. Sem periodo:
// um valor novo a cada unidade de `position`, ligados por uma curva suave.
float varietyValue(std::uint32_t seed, std::uint32_t channel, std::int64_t lattice) {
    std::uint64_t h = (static_cast<std::uint64_t>(seed) << 32 | channel)
        ^ static_cast<std::uint64_t>(lattice) * 0x9E3779B97F4A7C15ull;
    h ^= h >> 30; h *= 0xBF58476D1CE4E5B9ull;
    h ^= h >> 27; h *= 0x94D049BB133111EBull;
    h ^= h >> 31;
    return static_cast<float>(h >> 40) / 8388607.5f - 1.0f;
}

float varietyNoise(std::uint32_t seed, std::uint32_t channel, float position) {
    const float base = std::floor(position);
    const float t = position - base;
    const std::int64_t lattice = static_cast<std::int64_t>(base);
    const float a = varietyValue(seed, channel, lattice);
    const float b = varietyValue(seed, channel, lattice + 1);
    return a + (b - a) * t * t * (3.0f - 2.0f * t);
}

// Andando e correndo, cada passada um pouco diferente. O balanco dos bracos
// (a parte do clipe que se inverte em meio ciclo) cresce ou diminui - o
// jeito do personagem (+-10%) mais a passada (+-14%, cada braco o seu) -,
// os bracos adiantam ou atrasam um pouco em relacao as pernas, o cotovelo
// dobra mais ou menos, o tronco gira um pouco mais ou menos e a cabeca nao
// fica cravada. So bracos, coluna e cabeca: pernas e pelve seguem o clipe
// (os pes, a passada e o equilibrio nao mudam).
// additive: soma a diferenca (senao substitui a pose pela do clipe);
// clipPhase: as fases ja sao do clipe (senao, do ciclo alinhado ao apoio).
void applyGaitVariety(const RagdollProfile3D& profile,
    const AnimationClip3D& clip, float cyclePhase, float armPhase, float strides,
    std::uint32_t seed, float amount, std::vector<Vec3>& sampled,
    bool additive = VarietyAdditive, bool clipPhase = false) {
    const auto at = [&](std::string_view id, float phase) {
        const AnimationTrack3D* track = findAnimationTrack3D(clip, id);
        if (track == nullptr) return Vec3 {};
        const float wrapped = phase - std::floor(phase);
        return sampleAnimationTrack3D(*track,
            (clipPhase ? wrapped : stanceAlignedPhase(clip, wrapped)) * clip.durationSeconds,
            clip.durationSeconds, clip.loops).jointPositionRadians;
    };
    // Amostra no ciclo (deslocado) e realca ou abranda o que se inverte em
    // meio ciclo: s(t) + k * (s(t) - s(t + 1/2)) / 2.
    // (Somando a diferenca para a amostra do ciclo, nao substituindo: a
    // substituicao apagava tudo o que veio antes - a inclinacao da coluna do
    // clipe de curva, o tronco da curva.)
    const auto vary = [&](std::string_view id, float phase, float gain, float basePhase) {
        const std::size_t link = findLink(profile, id);
        if (link == 0 || link >= sampled.size()
            || findAnimationTrack3D(clip, id) == nullptr) return;
        const Vec3 now = at(id, phase);
        const Vec3 opposite = at(id, phase + 0.5f);
        const Vec3 varied = now + (now - opposite) * (0.5f * (gain - 1.0f));
        if (additive) sampled[link] += varied - at(id, basePhase);
        else sampled[link] = varied;
    };
    constexpr std::array<std::string_view, 3> LeftArm {
        "LeftUpperArm", "LeftForearm", "LeftHand" };
    constexpr std::array<std::string_view, 3> RightArm {
        "RightUpperArm", "RightForearm", "RightHand" };
    const float traitSwing = 0.10f * varietyValue(seed, 1, 0);
    const float traitLead = 0.02f * varietyValue(seed, 2, 0);
    const float traitElbow = 0.08f * varietyValue(seed, 3, 0);
    for (std::uint32_t side = 0; side < 2; ++side) {
        const float gain = 1.0f + amount * (traitSwing
            + 0.14f * varietyNoise(seed, 10 + side, strides)
            + 0.05f * varietyNoise(seed, 12, strides * 0.5f));
        const float lead = amount * (traitLead
            + 0.015f * varietyNoise(seed, 14 + side, strides * 0.7f));
        for (std::string_view id : side == 0 ? LeftArm : RightArm)
            vary(id, armPhase + lead, gain, armPhase);
        const std::size_t forearm = findLink(profile, side == 0
            ? "LeftForearm" : "RightForearm");
        if (forearm < sampled.size())
            sampled[forearm].x += amount * (traitElbow
                + 0.06f * varietyNoise(seed, 16 + side, strides * 0.6f));
    }
    const float torsoGain = 1.0f + amount * (0.06f * varietyValue(seed, 4, 0)
        + 0.10f * varietyNoise(seed, 20, strides * 0.8f));
    for (std::string_view id : { "Abdomen", "Chest", "UpperChest" })
        vary(id, cyclePhase, torsoGain, cyclePhase);
    const std::size_t head = findLink(profile, "Head");
    if (head < sampled.size()) {
        sampled[head].x += amount * 0.03f * varietyNoise(seed, 22, strides * 0.9f);
        sampled[head].y += amount * 0.02f * varietyNoise(seed, 23, strides * 0.7f);
    }
}

// Parado: respira (o peito sobe e desce, ~0,25 Hz, cada um no seu ritmo, a
// cabeca devolvendo), os bracos e a coluna se acomodam devagar (variacao de
// alguns segundos), e de vez em quando ele desvia um pouco o olhar. So
// tronco, bracos e cabeca: os pes e o equilibrio ficam com o planejador.
void applyIdleVariety(const RagdollProfile3D& profile, float seconds,
    std::uint32_t seed, float amount, std::vector<Vec3>& sampled) {
    const auto add = [&](std::string_view id, std::size_t axis, float value) {
        const std::size_t link = findLink(profile, id);
        if (link == 0 || link >= sampled.size()) return;
        setComponent(sampled[link], axis, component(sampled[link], axis) + value);
    };
    const float breathRate = 0.25f + 0.04f * varietyValue(seed, 30, 0);
    const float breath = std::sin(2.0f * Pi * breathRate * seconds
        + Pi * varietyValue(seed, 31, 0));
    add("Chest", 1, -0.014f * amount * breath);
    add("UpperChest", 1, -0.018f * amount * breath);
    add("Neck", 1, 0.020f * amount * breath);
    const float drift = seconds / 2.6f;
    add("Abdomen", 0, 0.030f * amount * varietyNoise(seed, 32, drift));
    add("Chest", 2, 0.020f * amount * varietyNoise(seed, 33, drift * 0.8f));
    for (std::uint32_t side = 0; side < 2; ++side) {
        const std::string_view upper = side == 0 ? "LeftUpperArm" : "RightUpperArm";
        const std::string_view forearm = side == 0 ? "LeftForearm" : "RightForearm";
        add(upper, 0, 0.035f * amount * varietyNoise(seed, 34 + side, drift * 0.9f));
        add(upper, 2, 0.050f * amount * varietyNoise(seed, 36 + side, drift * 0.7f));
        add(forearm, 0, 0.10f * amount * varietyNoise(seed, 38 + side, drift * 0.6f));
    }
    // Olhar: um pouco sempre, e de tempos em tempos (onde o ruido lento
    // passa de 0,55) um desvio maior, ate ~9 graus.
    const float wander = varietyNoise(seed, 40, seconds / 4.5f);
    const float glance = std::copysign(
        smoothStep((std::abs(wander) - 0.55f) / 0.30f), wander);
    add("Head", 0, amount * (0.04f * varietyNoise(seed, 41, drift) + 0.16f * glance));
    add("Neck", 0, amount * 0.05f * glance);
    add("Head", 1, amount * 0.03f * varietyNoise(seed, 42, drift * 1.1f));
}

} // namespace

bool CharacterLocomotionAnimations3D::compatible(
    const RagdollProfile3D& profile) const {
    return validClip(idle, profile) && validClip(walk, profile);
}

CharacterGaitSpeeds3D characterGaitSpeeds3D(
    const CharacterLocomotionAnimations3D& animations) {
    CharacterGaitSpeeds3D speeds;
    if (animations.walk != nullptr) {
        speeds.walk = authoredHorizontalSpeed(*animations.walk);
    }
    speeds.walkBackward = animations.walkBackward != nullptr
        ? authoredHorizontalSpeed(*animations.walkBackward) : speeds.walk;
    speeds.sprint = animations.sprint != nullptr
        ? authoredHorizontalSpeed(*animations.sprint) : speeds.walk;
    speeds.sprintBackward = animations.sprintBackward != nullptr
        ? authoredHorizontalSpeed(*animations.sprintBackward)
        : speeds.walkBackward;
    // O strafe vem de uma referencia para cada lado, com passadas um pouco
    // diferentes (2,9 e 2,4 m/s no trote). De lado o personagem anda na media
    // dos dois; cada clipe acompanha pelo ritmo (playbackRate).
    const auto sideSpeed = [](const AnimationClip3D& left,
        const AnimationClip3D& right) {
        return 0.5f * (authoredHorizontalSpeed(left)
            + authoredHorizontalSpeed(right));
    };
    if (animations.strafeLeft != nullptr && animations.strafeRight != nullptr) {
        speeds.strafe = sideSpeed(*animations.strafeLeft,
            *animations.strafeRight);
        speeds.sprintStrafe = animations.sprintStrafeLeft != nullptr
                && animations.sprintStrafeRight != nullptr
            ? sideSpeed(*animations.sprintStrafeLeft,
                *animations.sprintStrafeRight)
            : speeds.strafe;
    }
    return speeds;
}

float characterGaitSpeed3D(const CharacterGaitSpeeds3D& speeds,
    float travelRelativeToLookRadians, bool sprinting) {
    const float forward = sprinting ? speeds.sprint : speeds.walk;
    const float backward = sprinting && speeds.sprintBackward > 0.0f
        ? speeds.sprintBackward : speeds.walkBackward;
    const float angle = std::abs(wrapAngle(travelRelativeToLookRadians));
    // As mesmas faixas de histerese que a locomocao usa para trocar de
    // setor: a velocidade passa de um ciclo para o outro dentro delas.
    const auto blend = [](float from, float to, float angle, float border,
        float hysteresis) {
        const float share = std::clamp(
            (angle - (border - hysteresis)) / (2.0f * hysteresis), 0.0f, 1.0f);
        return from + (to - from) * share;
    };
    if (speeds.strafe <= 0.0f) {
        return blend(forward, backward, angle,
            PlainSectorLimitRadians, PlainSectorHysteresisRadians);
    }
    const float side = sprinting ? speeds.sprintStrafe : speeds.strafe;
    return angle <= Pi * 0.5f
        ? blend(forward, side, angle, StrafeFrontLimitRadians,
            StrafeSectorHysteresisRadians)
        : blend(side, backward, angle, StrafeBackLimitRadians,
            StrafeSectorHysteresisRadians);
}

void CharacterLocomotion3D::reset(const RagdollProfile3D& profile,
    const RagdollState3D& state) {
    m_output = {};
    m_telemetry = {};
    m_coordinates.assign(profile.links.size(), {});
    m_coordinateVelocities.assign(profile.links.size(), {});
    m_transitionFrom.assign(profile.links.size(), {});
    m_poseVelocities.assign(profile.links.size(), {});
    m_transitionVelocity.assign(profile.links.size(), {});
    m_lean = {};
    m_leanVelocity = {};
    m_spineBalance = {};
    m_spineBalanceVelocity = {};
    m_spawnHeightOffset = 0.0f;
    m_spawnHeightCaptured = false;
    m_rootTilt = {};
    m_transitionFromRootTilt = {};
    m_rootTiltVelocity = {};
    m_transitionRootTiltVelocity = {};
    m_rootOffsetVelocity = {};
    m_transitionRootOffsetVelocity = {};
    m_previousClip = nullptr;
    m_previousSampledClip = nullptr;
    m_previousClipCoordinates.assign(profile.links.size(), {});
    m_rootOffset = {};
    m_transitionFromRootOffset = {};
    m_groundLift = 0.0f;
    m_terrainSlope = 0.0f;
    m_terrainSpeedCap = 9.0f;
    m_requestYaw = 0.0f;
    m_requestYawValid = false;
    m_requestTurnRate = 0.0f;
    m_curveSpeedCap = 1000.0f;
    m_swingLowWeight = {};
    m_onInclineHold = 0.0f;
    m_pelvisOffset = 0.0f;
    m_pelvisOffsetVelocity = 0.0f;
    m_baseStepOffset = 0.0f;
    m_previousBaseGround = 0.0f;
    m_baseTracked = false;
    m_baseWasGrounded = false;
    m_gaitDirection = CharacterGaitDirection3D::Forward;
    m_feet = {};
    m_stateSeconds = 0.0f;
    m_transitionSeconds = 0.0f;
    m_transitionDurationSeconds = DefaultTransitionSeconds;
    m_cyclePhase = 0.0f;
    m_startClipActive = false;
    m_startClipSprint = false;
    m_startClipSeconds = 0.0f;
    m_authoredCurveBlend = 0.0f;
    m_authoredCurveKind = 0;
    m_idleSeconds = 0.0f;
    m_varietyStrides = 0.0f;
    m_ankleBalanceShift = {};
    m_turningRate = 0.0f;
    m_bodyCurveLean = 0.0f;
    m_spineCurveRoll = 0.0f;
    m_curveSprintShare = 0.0f;
    m_bodyHeading = 0.0f;
    m_bodyHeadingValid = false;
    m_bodyTurnRate = 0.0f;
    m_bodyCurveAcceleration = 0.0f;
    m_reactionStrength = 0.0f;
    m_physicsBlend = 0.0f;
    m_physicsHoldSeconds = 0.0f;
    m_previousDesiredVelocity = {};
    m_smoothedAcceleration = {};
    m_brakeDirection = {};
    m_brakeEntrySpeed = 0.0f;
    m_brakeReaction = {};
    m_brakeReactionVelocity = {};
    m_brakeGaitActive = false;
    m_brakeGait = 0.0f;
    m_brakeGaitSpeed = 0.0f;
    m_brakeGaitDirection = {};
    m_brakeCrouch = 0.0f;
    m_brakeCrouchVelocity = 0.0f;
    m_brakeStepping = false;
    m_brakeReversing = false;
    m_pivotSign = 0.0f;
    m_brakeRunShare = 0.0f;
    m_brakeSeconds = 0.0f;
    m_sinceRunRequest = 10.0f;
    m_brakeReferenceValid = false;
    m_brakeReferencePosition = {};
    m_wasBrakeStepping = false;
    m_stopRoot = {};
    m_stopRootBlend = 0.0f;
    m_stopVelocity = {};
    m_stopHolding = false;
    m_brakeStepWait = 0.0f;
    m_brakeAnchor = 0.0f;
    m_sinceBrakeSeconds = 10.0f;
    m_previousGrounded = true;
    m_initialized = !profile.links.empty()
        && state.links.size() == profile.links.size()
        && state.joints.size() == profile.links.size();
    if (!m_initialized) return;
    m_previousCenterOfMass = state.links.front().position;
    m_filteredCenterOfMassVelocity = {};
    m_gaitSpeedCap = 1000.0f;
    m_torsoTwist = 0.0f;
    m_headTurn = 0.0f;
    m_filteredLookPitch = 0.0f;
    m_lookValid = false;
    m_lookYawVelocity = 0.0f;
    m_lookPitchVelocity = 0.0f;
    m_poseCoordinates.clear();
    m_standingTurnTarget = 0.0f;
    m_clipPelvisYaw = 0.0f;
    m_clipPelvisToFootYaw = {};
    m_liftoffSpeed = 0.0f;
    m_liftoffVerticalSpeed = 0.0f;
    m_airborneFallSpeed = 0.0f;
    m_landingImpactSpeed = 0.0f;
    m_landingSettleSeconds = 0.18f;
    m_landingSinkPeak = 0.0f;
    m_sinceTouchdownSeconds = 10.0f;
    m_jumpKind = CharacterJumpKind3D::None;
    m_jumpClip = nullptr;
    m_leapClip = nullptr;
    m_flightPhase = 0.0f;
    m_elapsedSeconds = 0.0f;
    m_getUpPhase = CharacterGetUpPhase3D::None;
    m_tackling = false;
    m_tackleClip = nullptr;
    m_tackleCooldown = 0.0f;
    m_previousCrouchIntent = false;
    m_tacklePending = false;
    m_tacklePendingSeconds = 0.0f;
    m_tacklePreviousFoot = {};
    m_sinceTackleSeconds = 10.0f;
    m_fallOrientation = CharacterFallOrientation3D::None;
    m_getUpPhaseSeconds = 0.0f;
    m_recoveredStandingSeconds = 0.0f;
    m_getUpAttempt = 0;
    m_fallReflex = 0.0f;
    m_handCatchSeconds = {};
    m_fallSmoothCoordinates.clear();
    m_fetal = 0.0f;
    m_airBalance = 0.0f;
    m_fallCommit = 0.0f;
    m_tripRecovery = 0.0f;
    m_bracedMemory = m_braceAttemptSeconds = m_braceStallSeconds = 0.0f;
    m_braceRecovering = false;
    m_postureHeldSeconds = 0.0f;
    m_motionSeedPending = false;
    m_postureLost = false;
    m_stagger = 0.0f;
    m_staggerHold = 0.0f;
    m_tripRecoverySeconds = 0.0f;
    m_stumbleValid = false;
    m_stumblePitch = m_stumblePitchRate = m_stumbleBaseline = 0.0f;
    m_stumbleAlong = m_stumbleDecel = m_stumbleRisk = 0.0f;
    m_stumbleReaction = 0.0f;
    m_stumbleDirection = { 1.0f, 0.0f, 0.0f };
    m_stumbleStepping = false;
    m_stumbleStepSeconds = 0.0f;
    m_sinceStumbleStep = 10.0f;
    m_stumbleStepDirection = { 1.0f, 0.0f, 0.0f };
    m_startStepRequested = false;
    m_startStepActive = false;
    m_startDoseCap = 1000.0f;
    m_bodyRestSeconds = 0.0f;
    m_tripFoot = -1;
    m_tripElevating = false;
    m_tripDirection = { 1.0f, 0.0f, 0.0f };
    m_settleTone = 1.0f;
    m_settleRestSeconds = 0.0f;
    m_settlePoseValid = false;
    m_getUpAssistRamp = 0.0f;
    m_sinceGetUpSeconds = 10.0f;
    m_noGroundSeconds = 0.0f;
    m_previousTargetUp = { 0.0f, 0.0f, 1.0f };
    m_balanceReaction = 0.0f;
    m_reactiveShare = 0.0f;
    m_estimator.reset(profile);
    m_capturePoint = {};
    m_captureOutside = 0.0f;
    m_captureDirection = {};
    m_footwork.reset(profile);
    m_swingFromMeasuredPelvis = {};
    m_sprintClipAllowed = true;
    m_legRootFromMeasured = 0.0f;
    m_bodyRefValid = false;
    m_bodyRefHeight = 0.0f;
    m_plantFeetFromBody = false;
    m_handLinks = { findLink(profile, "LeftHand"),
        findLink(profile, "RightHand") };
    m_shinLinks = { findLink(profile, "LeftShin"),
        findLink(profile, "RightShin") };
    m_chestLink = findLink(profile, "Chest");
    for (std::size_t index = 1; index < profile.links.size(); ++index) {
        m_coordinates[index] = {
            state.joints[index].positionRadians[0],
            state.joints[index].positionRadians[1],
            state.joints[index].positionRadians[2]
        };
        m_transitionFrom[index] = m_coordinates[index];
        m_previousClipCoordinates[index] = m_coordinates[index];
        if (profile.links[index].id == "LeftFoot") {
            m_feet[0].linkIndex = index;
            m_feet[0].valid = true;
        } else if (profile.links[index].id == "RightFoot") {
            m_feet[1].linkIndex = index;
            m_feet[1].valid = true;
        }
    }
    m_previousRootPosition = state.links.front().position;
    m_previousRootOrientation = state.links.front().orientation;
    m_previousRisingPose = false;
    m_previousReferencePhase = CharacterGetUpPhase3D::None;
    m_previousRootBase = state.links.front().position;
    m_previousGuideVelocity = {};
    m_previousGuideAngularVelocity = {};
    const Vec3 forward = state.links.front().orientation.rotate(
        { 1.0f, 0.0f, 0.0f });
    m_facingYaw = std::atan2(forward.y, forward.x);
}

void CharacterLocomotion3D::seedMotion(const RagdollProfile3D& profile, const RagdollState3D& state,
    const CharacterLocomotionSeed3D& seed) {
    if (!m_initialized || state.links.empty()) return;
    m_motionSeedPending = true;
    m_seedJoints.assign(profile.links.size(), {});
    for (std::size_t i = 1; i < profile.links.size() && i < state.joints.size(); ++i) {
        const auto& joint = state.joints[i].positionRadians;
        m_seedJoints[i] = { joint[0], joint[1], joint[2] };
    }
    // Ja andando: sem a arrancada (o estado anterior nao e o parado).
    if (seed.moving) m_telemetry.state = CharacterLocomotionState3D::Walking;
    if (seed.tackleClip != nullptr && seed.tackleClip->durationSeconds > 0.0f) {
        // No meio do carrinho do padrao: o carrinho fisico continua do mesmo
        // ponto (como o comeco dele, abaixo em update, mas no tempo dado).
        m_motionSeedPending = false;
        ++m_tackleCount;
        m_tackleClip = seed.tackleClip;
        m_tackleRate = seed.tackleRate;
        m_tackleDistanceScale = seed.tackleScale;
        m_tackleEntrySpeed = seed.tackleEntrySpeed;
        m_tackleYaw = seed.tackleYaw;
        m_tackleStart = { seed.tackleStart.x, seed.tackleStart.y, state.links.front().position.z };
        m_tackling = true;
        m_tacklePending = false;
        m_tackleLandSpeed = -1.0f;
        m_tackleBehindSeconds = 0.0f;
        m_tackleHoldSeconds = 0.0f;
        // (Como enterGetUpPhase em update.)
        m_getUpPhase = CharacterGetUpPhase3D::Rising;
        m_getUpPhaseSeconds = 0.0f;
        m_getUpFootwork = false;
        m_getUpStepSide = -1;
        m_getUpBalanceHold = false;
        m_getUpBalanceHoldSeconds = 0.0f;
        m_getUpBalanceSteps = 0;
        m_getUpClipSeconds = std::clamp(seed.tackleTime, 0.0f, seed.tackleClip->durationSeconds);
        m_getUpAnchorWorld = m_tackleStart;
        m_getUpAnchorYaw = seed.tackleYaw;
        m_fallOrientation = CharacterFallOrientation3D::None;
        m_settleGather = 1.0f;
        m_settleTone = 1.0f;
        m_getUpRate = m_tackleRate;
        m_getUpAttempt = 0;
        m_getUpExpectedValid = false;
        m_getUpStruggle = 0.0f;
        m_getUpStallSeconds = 0.0f;
        m_getUpHoldSeconds = 0.0f;
        m_bodilyRising = false;
        m_risingFromMeasured.assign(state.joints.size(), Vec3 {});
        for (std::size_t link = 1; link < state.joints.size(); ++link)
            m_risingFromMeasured[link] = { state.joints[link].positionRadians[0],
                state.joints[link].positionRadians[1], state.joints[link].positionRadians[2] };
        for (FootPlant& foot : m_feet) {
            foot.planted = false;
            foot.repositioning = false;
            foot.recoveryStep = false;
        }
        m_telemetry.state = CharacterLocomotionState3D::Tackling;
        return;
    }
    const PhysicsBodyState3D& pelvis = state.links.front();
    const Vec3 forward = (pelvis.orientation * profile.links.front().modelOrientation.conjugate())
        .rotate({ 1.0f, 0.0f, 0.0f });
    const Quaternion unyaw = Quaternion::fromAxisAngle({ 0.0f, 0.0f, 1.0f }, -std::atan2(forward.y, forward.x));
    m_seedSpeed = std::hypot(pelvis.linearVelocity.x, pelvis.linearVelocity.y);
    for (std::size_t side = 0; side < 2; ++side) {
        if (!m_feet[side].valid || m_feet[side].linkIndex >= state.links.size()) continue;
        const PhysicsBodyState3D& foot = state.links[m_feet[side].linkIndex];
        m_seedFeet[side] = unyaw.rotate(foot.position - pelvis.position);
        m_seedFeetVelocity[side] = unyaw.rotate(foot.linearVelocity - pelvis.linearVelocity);
    }
}

const CharacterLocomotion3D::ClipFootEvents&
CharacterLocomotion3D::footEventsFor(const RagdollProfile3D& profile,
    const AnimationClip3D& clip) {
    for (const ClipFootEvents& cached : m_clipFootEvents) {
        if (cached.clip == &clip) return cached;
    }
    ClipFootEvents events;
    events.clip = &clip;
    const float duration = std::max(0.05f, clip.durationSeconds);
    const AnimationTrack3D* rootTrack = findAnimationTrack3D(clip,
        profile.links.front().id);
    constexpr int Samples = 240;
    for (std::size_t side = 0; side < 2; ++side) {
        const AnimationScalarTrack3D& contact = side == 0
            ? clip.leftFootContact : clip.rightFootContact;
        const FootPlant& foot = m_feet[side];
        if (contact.keyframes.empty() || !foot.valid) continue;
        const int shin = profile.links[foot.linkIndex].parentIndex;
        const int thigh = shin > 0
            ? profile.links[static_cast<std::size_t>(shin)].parentIndex : -1;
        if (thigh <= 0) continue;
        std::array<float, Samples> weight {};
        for (int i = 0; i < Samples; ++i) {
            weight[static_cast<std::size_t>(i)] = sampleAnimationScalarTrack3D(
                contact, duration * static_cast<float>(i) / Samples, duration,
                true);
        }
        const auto at = [&](int i) {
            return weight[static_cast<std::size_t>((i % Samples + Samples)
                % Samples)];
        };
        for (int i = 0; i < Samples; ++i) {
            if (!(at(i - 1) <= 0.55f && at(i) > 0.55f)) continue;
            FootTouchdown touchdown;
            touchdown.touchdown = duration * static_cast<float>(i) / Samples;
            touchdown.liftoff = touchdown.touchdown;
            for (int back = 1; back < Samples; ++back) {
                if (at(i - back) < 0.30f && at(i - back - 1) >= 0.30f) {
                    touchdown.liftoff = duration
                        * static_cast<float>(((i - back) % Samples + Samples)
                            % Samples) / Samples;
                    break;
                }
            }
            std::vector<Vec3> coordinates(profile.links.size());
            for (std::size_t index = 1; index < profile.links.size(); ++index) {
                const AnimationTrack3D* track = findAnimationTrack3D(clip,
                    profile.links[index].id);
                if (track == nullptr) continue;
                coordinates[index] = sampleAnimationTrack3D(*track,
                    touchdown.touchdown, duration, clip.loops)
                        .jointPositionRadians;
            }
            Vec3 translation;
            Quaternion rotation;
            if (rootTrack != nullptr) {
                const AnimationTransformSample3D root = sampleAnimationTrack3D(
                    *rootTrack, touchdown.touchdown, duration, clip.loops);
                translation = root.translationOffsetMeters;
                rotation = root.rotationDelta.normalized();
            }
            RagdollAnimationPose3D pose;
            buildLocalPose(profile, coordinates, pose);
            const Vec3 hip = profile.links[0].modelOrientation.conjugate()
                .rotate(profile.links[static_cast<std::size_t>(thigh)]
                    .inboundJoint.anchorModelPosition
                    - profile.links[0].modelPosition);
            touchdown.footInClip = translation
                + rotation.rotate(pose.linkPositions[foot.linkIndex]);
            touchdown.hipInClip = translation + rotation.rotate(hip);
            touchdown.forwardInClip = (rotation
                * pose.linkOrientations[foot.linkIndex]).rotate(
                    profile.links[foot.linkIndex].modelOrientation.conjugate()
                        .rotate({ 1.0f, 0.0f, 0.0f }));
            touchdown.footInClip.z = 0.0f;
            touchdown.hipInClip.z = 0.0f;
            events.touchdowns[side].push_back(touchdown);
        }
    }
    m_clipFootEvents.push_back(std::move(events));
    return m_clipFootEvents.back();
}

void CharacterLocomotion3D::enterState(CharacterLocomotionState3D state) {
    if (state == m_telemetry.state) return;
    // O crossfade parte da pose do CLIPE do tick anterior, sem o giro do
    // olhar e sem o IK dos pes, que sao somados de novo por cima: partindo da
    // pose final, o olhar entrava duas vezes e cabeca e tronco davam um
    // tranco a cada troca de estado (medido: cabeca a 50 rad/s ao girar a
    // camera parado, trocando entre parado e girando no lugar).
    m_transitionFrom = m_poseCoordinates.size() == m_coordinates.size()
        ? m_poseCoordinates : m_coordinates;
    m_transitionFromRootTilt = m_rootTilt;
    m_transitionVelocity = m_poseVelocities;
    m_transitionRootTiltVelocity = m_rootTiltVelocity;
    m_transitionRootOffsetVelocity = m_rootOffsetVelocity;
    m_transitionFromRootOffset = m_rootOffset;
    m_telemetry.state = state;
    m_stateSeconds = 0.0f;
    m_transitionSeconds = 0.0f;
    m_transitionDurationSeconds = DefaultTransitionSeconds;
    // Caindo -> deitado e deitado -> levantar: o corpo muda de pose inteiro;
    // em 0,14 s os membros davam um tranco.
    if (state == CharacterLocomotionState3D::Fallen
        || state == CharacterLocomotionState3D::GettingUp) {
        m_transitionDurationSeconds = 0.40f;
    }
    // Carrinho: da corrida direto para o mergulho.
    if (state == CharacterLocomotionState3D::Tackling) {
        m_transitionDurationSeconds = TackleTransition;
    }
    if (state == CharacterLocomotionState3D::Idle) {
        m_cyclePhase = 0.0f;
    }
    if (state == CharacterLocomotionState3D::Airborne
        || state == CharacterLocomotionState3D::JumpStarting
        || state == CharacterLocomotionState3D::Fallen) {
        for (FootPlant& foot : m_feet) {
            foot.planted = false;
            foot.repositioning = false;
            foot.recoveryStep = false;
        }
    }
}

void CharacterLocomotion3D::update(const RagdollProfile3D& profile,
    const CharacterLocomotionAnimations3D& animations,
    const RagdollState3D& state,
    const RagdollDynamics3D& dynamics,
    const CharacterLocomotionInput3D& input,
    float deltaTime) {
    m_output = {};
    // Locomocao fisica v2 (ver input.physicalV2).
    const bool PostureSupportPendulum = input.physicalV2 || PostureSupportPendulumEnv;
    const bool StartOnlyFromRest = input.physicalV2 || StartOnlyFromRestEnv;
    const bool LandingResync = input.physicalV2 || LandingResyncEnv;
    static const float stanceBandV2Env = tuneTemp("T_StanceBandV2", StanceRootBandV2);  // TEMPTUNE
    const float StanceRootBand = input.physicalV2 ? stanceBandV2Env : StanceRootBandEnv;
    const float StanceDampCurveSpan = input.physicalV2 ? 0.0f : StanceDampCurveSpanEnv;
    if (!std::isfinite(deltaTime) || deltaTime <= 0.0f
        || !animations.compatible(profile)
        || state.links.size() != profile.links.size()
        || state.joints.size() != profile.links.size()
        || !finite(input.rootPositionWorld)
        || !finite(input.rootVelocityWorld)
        || !finite(input.proxyVelocityWorld)
        || !std::isfinite(input.facingYawRadians)
        || !std::isfinite(input.lookPitchRadians)) {
        return;
    }
    if (!m_initialized) reset(profile, state);
    if (!m_initialized) return;
    deltaTime = std::min(deltaTime, 0.05f);
    const std::vector<Vec3> previousCoordinates = m_coordinates;
    // Estado fisico comum deste tick (tambem lido pela assistencia).
    const CharacterPhysicalState3D& physical = m_estimator.update(profile, state,
        input.footGround, deltaTime);

    // Centro de massa pela massa de cada link (do estimador comum), para
    // todos os personagens. Antes, sem RagdollDynamics3D valido - o caso de
    // todo boneco nao controlado -, o centro de massa era a pelve.
    // Tracked every tick, independent of authority, so the balance-assist
    // velocity below never sees a stale gap when it turns on mid-collision.
    const Vec3 centerOfMass = physical.valid ? physical.centerOfMassWorld
        : state.links.front().position;
    Vec3 measuredComVelocity = physical.valid ? physical.centerOfMassVelocityWorld
        : Vec3 {};
    (void) dynamics;
    if (!finite(measuredComVelocity)) measuredComVelocity = {};
    const float comVelocityBlend = 1.0f - std::exp(-16.0f * deltaTime);
    m_filteredCenterOfMassVelocity += (measuredComVelocity
        - m_filteredCenterOfMassVelocity) * comVelocityBlend;
    // Curva real do corpo (nao a pedida): o rumo da velocidade do centro de
    // massa girando, filtrado em ~0,2 s (tira o balanco de cada passada).
    {
        const float bodySpeed = std::hypot(m_filteredCenterOfMassVelocity.x,
            m_filteredCenterOfMassVelocity.y);
        const float bodyHeading = std::atan2(m_filteredCenterOfMassVelocity.y,
            m_filteredCenterOfMassVelocity.x);
        const float rate = bodySpeed > 1.0f && m_bodyHeadingValid
            ? wrapAngle(bodyHeading - m_bodyHeading) / deltaTime : 0.0f;
        m_bodyHeading = bodyHeading;
        m_bodyHeadingValid = bodySpeed > 1.0f;
        m_bodyTurnRate += (rate - m_bodyTurnRate) * (1.0f - std::exp(-deltaTime / BodyTurnFilterSeconds));
        m_bodyCurveAcceleration = bodySpeed * m_bodyTurnRate;
    }
    m_previousCenterOfMass = centerOfMass;
    m_elapsedSeconds += deltaTime;
    // O giro da camera (o carrinho vira o corpo para o lado dele).
    {
        const float rate = m_previousViewValid ? wrapAngle(input.facingYawRadians - m_previousViewYaw)
            / std::max(deltaTime, 1e-4f) : 0.0f;
        m_viewTurnRate += (std::clamp(rate, -12.0f, 12.0f) - m_viewTurnRate) * (1.0f - std::exp(-deltaTime / 0.12f));
        m_previousViewYaw = input.facingYawRadians;
        m_previousViewValid = true;
    }

    // Fall detection: a real trip/collision this controller could not
    // absorb through the reaction/interference softening below. Contacts
    // are the physical ground truth here — hands/knees/torso touching the
    // floor, not the authored-clip foot weights the normal locomotion foot
    // lock further down uses for a completely different purpose (knowing
    // when a walking step has landed).
    bool anyGroundContact = false;
    std::array<bool, 2> handGroundContact {};
    std::array<bool, 2> kneeGroundContact {};
    std::array<bool, 2> feetGroundContact {};
    float contactFloor = std::numeric_limits<float>::infinity();
    float lowerBodyFloor = std::numeric_limits<float>::infinity();
    float nonFootFloorHeight = std::numeric_limits<float>::infinity();
    bool touchingAnything = false;
    for (const RagdollContactPoint3D& contact : state.contacts) {
        if (contact.normalImpulseNewtonSeconds > 0.00001f) touchingAnything = true;
        // Chao e superficie parada: uma caixa atingindo o corpo no ar, por
        // baixo, fazia a queda ser declarada ainda no ar.
        if (contact.normal.z < 0.40f || contact.otherBodyDynamic
            || contact.normalImpulseNewtonSeconds <= 0.00001f) {
            continue;
        }
        anyGroundContact = true;
        contactFloor = std::min(contactFloor, contact.position.z);
        // (v2: o chao de verdade e o da metade de baixo do corpo - pelve,
        // coxas, canelas, pes. O tronco ou os bracos apoiados no alto de uma
        // mureta nao sao chao: medido contra o topo dela, o corpo debrucado
        // parecia deitado - "caido" com o centro de massa a 0,9 m do chao.)
        if (contact.linkIndex < profile.links.size()) {
            const std::string& id = profile.links[contact.linkIndex].id;
            const bool foot = id.find("Foot") != std::string::npos;
            if (contact.linkIndex == 0 || id.find("Thigh") != std::string::npos
                || id.find("Shin") != std::string::npos || foot)
                lowerBodyFloor = std::min(lowerBodyFloor, contact.position.z);
            if (!foot) nonFootFloorHeight = std::min(nonFootFloorHeight, contact.position.z);
        }
        for (std::size_t side = 0; side < 2; ++side) {
            if (contact.linkIndex == m_handLinks[side]) {
                handGroundContact[side] = true;
            }
            if (contact.linkIndex == m_shinLinks[side]) {
                kneeGroundContact[side] = true;
            }
        }
    }
    // Pes: o apoio medido pelo estimador (contato com a normal para cima,
    // chao parado), o mesmo criterio de antes num lugar so.
    for (std::size_t side = 0; side < 2; ++side) {
        const FootSupportEstimate3D& foot = physical.feet[side];
        feetGroundContact[side] = m_feet[side].valid && foot.supporting
            && foot.evidence != ContactEvidence3D::Geometric && !foot.onDynamicBody;
    }

    // Caindo, ha quanto tempo cada mao segura o chao.
    for (std::size_t side = 0; side < 2; ++side)
        m_handCatchSeconds[side] = handGroundContact[side] && m_fallReflex > 0.3f
            ? std::min(2.0f, m_handCatchSeconds[side] + deltaTime)
            : m_fallReflex > 0.3f ? m_handCatchSeconds[side] : 0.0f;
    // Sem tocar nada (no ar): para o espernear da queda.
    m_noGroundSeconds = touchingAnything ? 0.0f
        : std::min(10.0f, m_noGroundSeconds + deltaTime);
    float rootUpright = 1.0f;
    if (!state.links.empty()) {
        rootUpright = state.links.front().orientation
            .rotate({ 0.0f, 0.0f, 1.0f }).z;
    }
    // The capsule's implied ground (feet height it stands at) doubles as a
    // ground estimate here; contacts already gate whether anything is
    // actually down, this only sizes how far down it is.
    const float groundHeightGuess =
        input.rootPositionWorld.z - profile.standingRootHeightMeters;
    // No modo fisico, contra a superficie que o corpo toca de fato: a
    // capsula decola antes no pulo, e medido contra ela o corpo no ar (o pe
    // ainda empurrando o chao) parecia "caido" - queda falsa na decolagem.
    // (v2: o mais baixo entre os contatos da metade de baixo, o apoio dos pes
    // e o chao da capsula - um pe no topo da mureta, com o outro no chao,
    // nao faz do topo o chao: em pe, 6 graus, ele era dado como caido.)
    const float fallenFloor = input.physicalV2
        ? std::min({ lowerBodyFloor, physical.supported ? physical.supportHeight
            : std::numeric_limits<float>::infinity(), groundHeightGuess }) : contactFloor;
    const float measuredComHeight = std::max(0.08f, centerOfMass.z
        - (input.forceDrivenRoot && std::isfinite(fallenFloor)
            ? fallenFloor : groundHeightGuess));
    // Only meaningful while physics is the one holding the body up. In
    // animation mode the authored pose says the character is standing, so
    // "fallen" is not a state it can observe - and reading the simulated
    // bodies anyway is what produced the fall/stand-up/fall loop: those
    // bodies are still lying down for a few ticks after a recovery hands the
    // pose back, which instantly re-triggered another fall.
    // Com a raiz livre (forceDrivenRoot) o corpo e sempre o fisico: a queda
    // e reconhecida pelo estado dele, sem depender de um impacto antes.
    const bool physicsOwnsBody = input.forceDrivenRoot || m_physicsBlend > 0.5f;
    // v2 - recuperacao apoiada (pedido do usuario, 06/10): o tronco, os bracos
    // ou as maos apoiados numa superficie parada no alto (o topo de uma
    // mureta, uma parede, um objeto), com um pe no chao. Debrucado assim,
    // ele ainda pode se recompor - pes para baixo do corpo, quadril e
    // joelhos estendendo, o tronco e os bracos empurrando o apoio (o par de
    // postura, interno: sem forca de fora). Enquanto tenta (BraceAttemptSeconds,
    // e progredindo), a postura nao e dada como perdida e nao e queda; antes,
    // a postura perdida amolecia as pernas na hora e ele ficava pendurado na
    // mureta (o estado de caido espera o corpo deitado).
    {
        bool bracedNow = false;
        if (input.physicalV2 && physicsOwnsBody && (feetGroundContact[0] || feetGroundContact[1])) {
            const float floor = std::isfinite(lowerBodyFloor) ? lowerBodyFloor : groundHeightGuess;
            for (const RagdollContactPoint3D& contact : state.contacts) {
                if (contact.otherBodyDynamic || contact.normalImpulseNewtonSeconds <= 0.00001f
                    || contact.normal.z < -0.3f || contact.linkIndex >= profile.links.size()
                    || contact.linkIndex == 0) continue;
                const std::string& id = profile.links[contact.linkIndex].id;
                if (id.find("Thigh") != std::string::npos || id.find("Shin") != std::string::npos
                    || id.find("Foot") != std::string::npos) continue;
                if (contact.position.z < floor + BraceMinimumHeight) continue;
                bracedNow = true;
                break;
            }
        }
        m_bracedMemory = bracedNow ? BraceMemorySeconds : std::max(0.0f, m_bracedMemory - deltaTime);
        const float tilt = std::acos(std::clamp(rootUpright, -1.0f, 1.0f));
        if (m_bracedMemory > 0.0f) {
            m_braceAttemptSeconds += deltaTime;
            // Progresso: a inclinacao caindo. Sem melhorar por
            // BraceStallSeconds, desiste (cai de verdade).
            if (tilt < m_braceBestTilt - 0.03f) {
                m_braceBestTilt = tilt;
                m_braceStallSeconds = 0.0f;
            } else {
                m_braceStallSeconds += deltaTime;
            }
        } else if (rootUpright > 0.85f || !(feetGroundContact[0] || feetGroundContact[1])) {
            m_braceAttemptSeconds = 0.0f;
            m_braceStallSeconds = 0.0f;
            m_braceBestTilt = tilt;
        }
        m_braceRecovering = m_bracedMemory > 0.0f && m_getUpPhase == CharacterGetUpPhase3D::None
            && m_braceAttemptSeconds < BraceAttemptSeconds && m_braceStallSeconds < BraceStallSeconds
            && rootUpright > BraceMinimumUpright && measuredComHeight > 0.40f;
        m_telemetry.braceRecovering = m_braceRecovering;
    }
    // (Logo depois do carrinho ele ainda vem baixo, com a mao ou o joelho
    // tocando o chao: ai so conta como queda tombado de verdade.)
    // (Cambaleando com um pe no chao - meio apoiado numa mureta, num boneco -
    // ainda nao e queda: ele tenta se endireitar ate StaggerFallenRelief a
    // mais de inclinacao, enquanto o cambaleio dura.)
    const float fallenUpright = 0.45f - StaggerFallenRelief * m_stagger
        * (feetGroundContact[0] || feetGroundContact[1] ? 1.0f : 0.0f);
    // (v2: baixo, mas agachado sobre os pes - tronco ate ~53 graus, nada alem
    // dos pes tocando o chao - nao e queda: dali ele ainda levanta. Amolecido
    // por um choque, sem ajuda, ele descia num agachamento fundo e era dado
    // como caido com os pes no chao.)
    const bool squattingOnFeet = input.physicalV2 && (feetGroundContact[0] || feetGroundContact[1])
        && rootUpright > 0.6f && !(nonFootFloorHeight < fallenFloor + 0.30f) && measuredComHeight > 0.30f;
    const bool fallenObservation = physicsOwnsBody && !m_braceRecovering
        && m_elapsedSeconds > 0.6f
        && anyGroundContact
        && (rootUpright < fallenUpright || (measuredComHeight < 0.52f && !squattingOnFeet))
        && (m_sinceTackleSeconds > 0.8f || rootUpright < 0.30f);

    const auto enterGetUpPhase = [&](CharacterGetUpPhase3D phase) {
        m_getUpPhase = phase;
        m_getUpPhaseSeconds = 0.0f;
        m_getUpFootwork = false;
        m_getUpStepSide = -1;
        m_getUpBalanceHold = false;
        m_getUpBalanceHoldSeconds = 0.0f;
        m_getUpBalanceSteps = 0;
        if (phase == CharacterGetUpPhase3D::Settling) {
            m_getUpRolling = false;
            m_getUpRollDone = false;
            m_getUpRollSeconds = 0.0f;
        }
    };
    const auto classifyFallOrientation = [&]() {
        const std::size_t reference = m_chestLink < state.links.size()
            ? m_chestLink : 0;
        const Vec3 bodyForward = state.links[reference]
            .orientation.rotate({ 1.0f, 0.0f, 0.0f });
        m_fallOrientation = bodyForward.z < 0.0f
            ? CharacterFallOrientation3D::FaceDown
            : CharacterFallOrientation3D::FaceUp;
    };
    // Carrinho: correndo, agachar (a borda do pedido) arma o carrinho para
    // onde ele olha: mais devagar o de lado (Soccer Tackle), mais rapido o
    // sentado (Running Slide, que desliza mais). O preparo: ele segue
    // correndo, a pelve baixa um pouco, e o mergulho comeca no POUSO do pe
    // com que o clipe comeca apoiado - a passada encaixa no clipe (antes
    // comecava em qualquer ponto da passada, com o pe errado no chao). So no
    // chao, de pe, indo para a frente do olhar com embalo - parado, recuando
    // ou de lado o Ctrl so agacha.
    {
        const bool crouchPressed = input.intent.crouch && !m_previousCrouchIntent;
        m_previousCrouchIntent = input.intent.crouch;
        m_tackleCooldown = std::max(0.0f, m_tackleCooldown - deltaTime);
        m_sinceTackleSeconds = m_tackling ? 0.0f : std::min(10.0f, m_sinceTackleSeconds + deltaTime);
        const float yaw = input.facingYawRadians;
        const Vec3 direction { std::cos(yaw), std::sin(yaw), 0.0f };
        const float along = dot(Vec3 { m_filteredCenterOfMassVelocity.x,
            m_filteredCenterOfMassVelocity.y, 0.0f }, direction);
        const bool moving = m_telemetry.state == CharacterLocomotionState3D::Walking
            || m_telemetry.state == CharacterLocomotionState3D::Running;
        const bool haveClip = animations.tackles[0] != nullptr || animations.tackles[1] != nullptr;
        const bool canTackle = TackleEnabled && input.tackleEnabled && haveClip
            && m_getUpPhase == CharacterGetUpPhase3D::None && !m_tackling
            && input.grounded && moving && along >= TackleMinSpeed * (m_tacklePending ? 0.8f : 1.0f)
            && m_fallCommit < 0.3f && !state.links.empty();
        // O pe com que cada clipe comeca apoiado (o mais baixo no comeco):
        // o de lado firma o esquerdo e da o bote com a direita; o sentado
        // firma o direito e estica a esquerda.
        const auto clipStartSide = [&](const AnimationClip3D* clip) {
            if (clip == nullptr) return -1;
            const RagdollAnimationPose3D first = sampleRagdollAnimationPose3D(profile, clip, 0.0f, false);
            if (m_feet[0].linkIndex >= first.linkPositions.size()
                || m_feet[1].linkIndex >= first.linkPositions.size()) return -1;
            return first.linkPositions[m_feet[0].linkIndex].z
                < first.linkPositions[m_feet[1].linkIndex].z ? 0 : 1;
        };
        const auto clipForSide = [&](int side) {
            for (const AnimationClip3D* clip : animations.tackles)
                if (clip != nullptr && clipStartSide(clip) == side) return clip;
            return animations.tackles[0] != nullptr ? animations.tackles[0] : animations.tackles[1];
        };
        if (crouchPressed && canTackle && m_tackleCooldown <= 0.0f && !m_tacklePending) {
            m_tacklePending = true;
            m_tacklePendingSeconds = 0.0f;
            // A situacao decide: o pe que vai pousar no proximo passo (o que
            // esta no ar; no voo, o da frente) escolhe o carrinho que comeca
            // firmando ele - o mergulho sai nesse pouso, sem esperar outro.
            int next = 0;
            if (feetGroundContact[0] != feetGroundContact[1]) {
                next = feetGroundContact[0] ? 1 : 0;
            } else if (m_feet[0].linkIndex < state.links.size() && m_feet[1].linkIndex < state.links.size()) {
                next = dot(state.links[m_feet[0].linkIndex].position, direction)
                    > dot(state.links[m_feet[1].linkIndex].position, direction) ? 0 : 1;
            }
            m_tackleStartSide = next;
            m_tackleClip = clipForSide(next);
            // Qual (pedido do usuario, 05/10 noite; como no padrao): o que
            // vira o corpo para o lado do giro da camera; parada, para o lado
            // da corrida; senao, ao acaso. Fica esse (sem trocar pelo pe).
            if (TackleChoiceByView) {
                std::array<TackleClipShape3D, 2> shapes {};
                for (std::size_t k = 0; k < shapes.size(); ++k)
                    if (animations.tackles[k] != nullptr) shapes[k] = tackleClipShape3D(profile, *animations.tackles[k]);
                const Vec3 travel { input.intent.requestedVelocityWorld.x, input.intent.requestedVelocityWorld.y, 0.0f };
                const float lateral = travel.lengthSquared() > 1e-4f
                    ? std::sin(wrapAngle(std::atan2(travel.y, travel.x) - input.facingYawRadians)) : 0.0f;
                const int chosen = chooseTackleClip3D(animations.tackles, shapes, m_viewTurnRate, lateral,
                    (input.varietySeed * 2654435761u) ^ (m_tackleCount * 40503u) ^ static_cast<std::uint32_t>(m_elapsedSeconds * 1000.0f));
                if (chosen >= 0) {
                    m_tackleClip = animations.tackles[static_cast<std::size_t>(chosen)];
                    m_tackleStartSide = clipStartSide(m_tackleClip);
                    m_tackleChosen = true;
                }
            }
            m_tackleVariant = m_tackleClip == animations.tackles[1] ? 1 : 0;
        }
        if (m_tacklePending && !canTackle) {
            m_tacklePending = false;
            m_tackleClip = nullptr;
        }
        // A voadora no modo fisico (pedido do usuario, 06/10; no padrao ela ja
        // existia): no ar, Ctrl com Shift + W, uma por voo. O carrinho entra ja
        // na pose do mergulho (a perna esticada, do mesmo ponto em que o
        // fisico continua a voadora do padrao) e a segura ate o corpo descer;
        // ele nao empurra nem freia o corpo, entao o voo e o embalo do pulo.
        if (input.grounded) m_airTackleSpent = false;
        bool airStart = false;
        float startClipTime = 0.0f, startSpeed = along, startYaw = yaw;
        {
            const Vec3 keys = input.intent.requestedDirectionWorld;
            const bool combo = input.intent.crouch && input.intent.effort > 0.5f
                && keys.x * direction.x + keys.y * direction.y > 0.7f;
            const bool flying = !input.grounded && m_telemetry.state == CharacterLocomotionState3D::Airborne
                && m_stateSeconds > 0.05f;
            if (combo && flying && input.airTackleEnabled && !m_airTackleSpent && TackleEnabled && input.tackleEnabled
                && haveClip && m_getUpPhase == CharacterGetUpPhase3D::None && !m_tackling && !m_tacklePending
                && m_tackleCooldown <= 0.0f && m_fallCommit < 0.3f && !state.links.empty()) {
                m_airTackleSpent = true;
                std::array<TackleClipShape3D, 2> shapes {};
                for (std::size_t k = 0; k < shapes.size(); ++k)
                    if (animations.tackles[k] != nullptr) shapes[k] = tackleClipShape3D(profile, *animations.tackles[k]);
                const Vec3 flat { m_filteredCenterOfMassVelocity.x, m_filteredCenterOfMassVelocity.y, 0.0f };
                const float flatSpeed = flat.length();
                const float lateral = flatSpeed > 0.01f
                    ? std::sin(wrapAngle(std::atan2(flat.y, flat.x) - input.facingYawRadians)) : 0.0f;
                int chosen = chooseTackleClip3D(animations.tackles, shapes, m_viewTurnRate, lateral,
                    (input.varietySeed * 2654435761u) ^ (m_tackleCount * 40503u)
                        ^ static_cast<std::uint32_t>(m_elapsedSeconds * 1000.0f));
                if (chosen < 0) chosen = animations.tackles[0] != nullptr ? 0 : 1;
                const std::size_t k = static_cast<std::size_t>(chosen);
                m_tackleClip = animations.tackles[k];
                m_tackleStartSide = clipStartSide(m_tackleClip);
                m_tackleVariant = static_cast<int>(k);
                // (Do fim do mergulho, como o do padrao: do comeco dele, o
                // corpo ficava em pe no ar com uma perna a frente e pousava
                // sentado.)
                startClipTime = std::clamp(std::max(shapes[k].diveStart, shapes[k].lowStart - 0.02f),
                    0.0f, m_tackleClip->durationSeconds);
                startSpeed = std::max(2.0f, flatSpeed);
                startYaw = flatSpeed > 1.0f ? std::atan2(flat.y, flat.x) : input.facingYawRadians;
                airStart = true;
            }
        }
        const std::array<bool, 2> touchdown { feetGroundContact[0] && !m_tacklePreviousFoot[0],
            feetGroundContact[1] && !m_tacklePreviousFoot[1] };
        m_tacklePreviousFoot = { feetGroundContact[0], feetGroundContact[1] };
        bool start = false;
        if (m_tacklePending) {
            m_tacklePendingSeconds += deltaTime;
            // Pousou um pe: o carrinho dele (se pousou o outro antes do
            // previsto, troca para o dele). Passando do prazo, o pe que esta
            // no chao.
            int side = -1;
            if (touchdown[0] != touchdown[1]) side = touchdown[0] ? 0 : 1;
            else if (touchdown[0]) side = m_tackleStartSide;
            else if (m_tacklePendingSeconds > TacklePrepMax)
                side = feetGroundContact[0] != feetGroundContact[1] ? (feetGroundContact[0] ? 0 : 1)
                    : m_tackleStartSide;
            if (side >= 0) {
                if (side != m_tackleStartSide && !m_tackleChosen) {
                    m_tackleStartSide = side;
                    m_tackleClip = clipForSide(side);
                    m_tackleVariant = m_tackleClip == animations.tackles[1] ? 1 : 0;
                }
                start = true;
            }
        }
        if (airStart) start = true;
        if (start && m_tackleClip != nullptr) {
            m_tacklePending = false;
            ++m_tackleCount;
            // Ritmo e percurso pela velocidade de entrada contra a da
            // corrida do comeco do clipe: mais devagar, o clipe toca um
            // pouco mais lento e o caminho encurta (as duas pela raiz).
            const AnimationTransformSample3D r0 = clipRootAt(profile, *m_tackleClip, 0.0f);
            const AnimationTransformSample3D r1 = clipRootAt(profile, *m_tackleClip, 0.2f);
            const float clipSpeed = std::max(1.0f, std::hypot(
                r1.translationOffsetMeters.x - r0.translationOffsetMeters.x,
                r1.translationOffsetMeters.y - r0.translationOffsetMeters.y) / 0.2f);
            const float ratio = std::clamp(startSpeed / clipSpeed, 0.4f, 1.3f);
            m_tackleRate = std::clamp(std::sqrt(ratio), 0.75f, 1.12f);
            m_tackleDistanceScale = ratio / m_tackleRate;
            m_tackleChosen = false;
            // Os dois deslizam o mesmo: o caminho de cada um vai a fase
            // deitada media deles.
            {
                float sum = 0.0f;
                int count = 0;
                TackleClipShape3D mine {};
                for (const AnimationClip3D* other : animations.tackles) {
                    if (other == nullptr) continue;
                    const TackleClipShape3D shape = tackleClipShape3D(profile, *other);
                    if (other == m_tackleClip) mine = shape;
                    if (shape.lowMeters > 0.3f) { sum += shape.lowMeters; ++count; }
                }
                if (count > 0 && mine.lowMeters > 0.3f && TackleChoiceByView)
                    m_tackleDistanceScale *= (sum / static_cast<float>(count)) / mine.lowMeters;
            }
            m_tackleEntrySpeed = startSpeed;
            m_tackleYaw = startYaw;
            // O caminho do clipe comeca onde a pelve esta (no ar, no ponto do
            // mergulho).
            const Vec3 pelvis = state.links.front().position;
            const AnimationTransformSample3D rStart = airStart ? clipRootAt(profile, *m_tackleClip, startClipTime) : r0;
            const Vec3 first = Quaternion::fromAxisAngle({ 0.0f, 0.0f, 1.0f }, startYaw).rotate({
                rStart.translationOffsetMeters.x * m_tackleDistanceScale,
                rStart.translationOffsetMeters.y * m_tackleDistanceScale, 0.0f });
            m_tackleStart = { pelvis.x - first.x, pelvis.y - first.y, pelvis.z };
            m_tackling = true;
            m_tackleLandSpeed = -1.0f;
            m_tackleBehindSeconds = 0.0f;
            m_tackleHoldSeconds = 0.0f;
            enterGetUpPhase(CharacterGetUpPhase3D::Rising);
            m_getUpClipSeconds = startClipTime;
            m_getUpAnchorWorld = m_tackleStart;
            m_getUpAnchorYaw = startYaw;
            m_fallOrientation = CharacterFallOrientation3D::None;
            m_settleGather = 1.0f;
            m_settleTone = 1.0f;
            m_getUpRate = m_tackleRate;
            m_getUpAttempt = 0;
            m_getUpExpectedValid = false;
            m_getUpStruggle = 0.0f;
            m_getUpStallSeconds = 0.0f;
            m_getUpHoldSeconds = 0.0f;
            m_bodilyRising = false;
            m_risingFromMeasured.assign(state.joints.size(), Vec3 {});
            for (std::size_t link = 1; link < state.joints.size(); ++link) {
                m_risingFromMeasured[link] = { state.joints[link].positionRadians[0],
                    state.joints[link].positionRadians[1],
                    state.joints[link].positionRadians[2] };
            }
            for (FootPlant& foot : m_feet) {
                foot.planted = false;
                foot.repositioning = false;
                foot.recoveryStep = false;
            }
        }
    }
    // Pressa para levantar (as teclas de andar; com Shift, correndo): ver
    // GetUpSpeed.
    const bool getUpKeys = input.intent.requestedVelocityWorld.x * input.intent.requestedVelocityWorld.x
        + input.intent.requestedVelocityWorld.y * input.intent.requestedVelocityWorld.y > 0.01f;
    // (O Shift pela intencao: caido, a capsula esta parada e input.sprinting
    // - que so liga com ela andando - nao o via.)
    // Sem pressa, o levantar no ritmo dele, detalhado (pedido do usuario,
    // 05/10 noite); com as teclas de andar ou o espaco, rapido; andando com
    // Shift, correndo.
    const int getUpHurry = !GetUpHurryEnabled || (!getUpKeys && !input.getUpHurry) ? 0
        : getUpKeys && (input.sprinting || input.intent.effort > 0.5f) ? 2 : 1;
    const float getUpSpeed = getUpHurry == 2 ? GetUpSprintSpeed : getUpHurry == 1 ? GetUpHurrySpeed : GetUpSpeed;
    m_getUpSpeedNow = getUpSpeed;
    if (m_getUpPhase != CharacterGetUpPhase3D::Rising) m_getUpHandoffReady = false;
    if (m_getUpPhase == CharacterGetUpPhase3D::None) {
        if (fallenObservation && std::getenv("XQUEDADBG"))
            std::printf("QUEDA t %.2f upright %.2f (limite %.2f) com %.2f chao %.2f contato %d\n", m_elapsedSeconds,
                rootUpright, fallenUpright, measuredComHeight, contactFloor, anyGroundContact ? 1 : 0);
        if (fallenObservation) {
            m_getUpHurriedExit = false;
            classifyFallOrientation();
            enterGetUpPhase(CharacterGetUpPhase3D::Settling);
            ++m_getUpAttempt;
            m_recoveredStandingSeconds = 0.0f;
            m_settleTone = SettleImpactTone;
            m_settleRestSeconds = 0.0f;
            m_settlePoseValid = false;
            for (FootPlant& foot : m_feet) {
                foot.planted = false;
                foot.repositioning = false;
                foot.recoveryStep = false;
            }
        }
    } else if (!input.manipulated) {
        // Being held by the PhysGun freezes recovery instead of fighting the
        // player back toward a stand-up target.
        m_getUpPhaseSeconds += deltaTime;
        if (m_tackling) {
            // Carrinho: o clipe anda no tempo (o mergulho e o deslize sao o
            // embalo, nao esperam o corpo), no ritmo da entrada.
            const AnimationClip3D* clip = m_tackleClip;
            const float duration = clip != nullptr ? clip->durationSeconds : 1.0f;
            const float height = state.links.front().position.z - groundHeightGuess;
            // Deslizando (a pelve do clipe e a do corpo baixas, o clipe sem
            // subir ainda), a pose do deslize fica enquanto o corpo anda: o
            // comprimento do deslize e o do embalo e do atrito, nao o do
            // clipe. Devagar, o clipe segue para o levantar.
            const float bodySpeed = std::hypot(state.links.front().linearVelocity.x,
                state.links.front().linearVelocity.y);
            const bool sliding = m_tackleExpectedHeight < TackleSlideHeight
                && height < TackleSlideHeight + 0.10f && m_tackleExpectedRise < 0.4f;
            const float slideRate = sliding ? std::clamp(1.0f - (bodySpeed - TackleSlideRelease) / 1.5f,
                0.12f, 1.0f) : 1.0f;
            // Descendo, o clipe espera o corpo: o ator se joga ao chao mais
            // rapido que a queda livre (0,6 m em 0,15 s; caindo, 0,35 s) - o
            // clipe chegava a pose de deslize com o corpo ainda no alto, as
            // pernas iam a frente no ar e, quando ele tocava o chao, o clipe
            // ja estava levantando (quase nao deslizava).
            const float dropRate = m_tackleExpectedRise < 0.0f
                ? std::clamp(1.0f - (height - m_tackleExpectedHeight - 0.10f) / 0.20f, 0.20f, 1.0f) : 1.0f;
            const float paceShare = std::min(slideRate, dropRate);
            m_getUpClipSeconds = std::min(duration, m_getUpClipSeconds + deltaTime * m_tackleRate * paceShare);
            m_getUpRate = m_tackleRate * paceShare;
            const float expected = clip != nullptr ? profile.standingRootHeightMeters
                + clipRootAt(profile, *clip, m_getUpClipSeconds).translationOffsetMeters.z
                : profile.standingRootHeightMeters;
            // A altura da pelve no clipe e quanto ela sobe (m/s, no ritmo):
            // os musculos e a ajuda leem as fases (descendo, subindo).
            if (clip != nullptr) {
                const float ahead = std::min(duration, m_getUpClipSeconds + 1.0f / 30.0f);
                const float rise = profile.standingRootHeightMeters
                    + clipRootAt(profile, *clip, ahead).translationOffsetMeters.z - expected;
                m_tackleExpectedRise = ahead > m_getUpClipSeconds
                    ? rise * 30.0f * m_tackleRate : 0.0f;
            }
            m_tackleExpectedHeight = expected;
            // Na volta (da metade do clipe em diante) a pelve muito abaixo de
            // onde o clipe ja subiu, ou o fim sem estar de pe: nao deu - vira
            // queda e ele levanta pelo levantar normal.
            const bool recovering = m_getUpClipSeconds > 0.5f * duration;
            m_tackleBehindSeconds = recovering && height < expected - 0.40f
                ? m_tackleBehindSeconds + deltaTime
                : std::max(0.0f, m_tackleBehindSeconds - deltaTime);
            // De pe o bastante para a locomocao assumir: ela segura o peso e
            // termina de endireitar (no carrinho a ajuda de postura dela fica
            // suspensa - exigindo o corpo ereto, ele ficava meio agachado e
            // inclinado ~40 graus, sem forca para subir, ate virar queda).
            // (Ele termina de se endireitar ainda no carrinho - a ajuda de
            // levantar dele vale ate o corpo chegar a altura do clipe;
            // demorando, aceita mais inclinado em vez de virar queda.)
            const bool upright = (rootUpright > 0.82f
                    && height > profile.standingRootHeightMeters * 0.70f)
                || (m_tackleHoldSeconds > 0.4f && rootUpright > 0.65f
                    && height > profile.standingRootHeightMeters * 0.60f);
            // De volta em pe no clipe (passou do deslize e a pelve do clipe ja
            // na altura de pe) e no corpo: a corrida volta a ser da
            // locomocao - os passos e o equilibrio dela seguram a saida
            // correndo; presa ao fim do clipe, a 7 m/s o corpo pendia ~45
            // graus para a frente.
            // (Logo que ele levanta do deslize, antes da passada de saida do
            // clipe: ela nao se adapta - o pe pousava atras do centro de massa
            // e ele tombava para a frente. A locomocao poe o pe pelo ponto de
            // captura.)
            const bool clipStanding = m_getUpClipSeconds > 0.6f * duration
                && m_tackleExpectedHeight > profile.standingRootHeightMeters * TackleHandoffHeight;
            if (m_getUpClipSeconds >= duration && !upright) m_tackleHoldSeconds += deltaTime;
            if (m_tackleBehindSeconds > 1.0f || m_tackleHoldSeconds > 1.0f) {
                m_tackling = false;
                m_tackleClip = nullptr;
                classifyFallOrientation();
                enterGetUpPhase(CharacterGetUpPhase3D::Settling);
                ++m_getUpAttempt;
                m_recoveredStandingSeconds = 0.0f;
                m_settleTone = SettleImpactTone;
                m_settleRestSeconds = 0.0f;
                m_settlePoseValid = false;
                for (FootPlant& foot : m_feet) {
                    foot.planted = false;
                    foot.repositioning = false;
                    foot.recoveryStep = false;
                }
            } else if ((m_getUpClipSeconds >= duration || clipStanding) && upright) {
                m_tackling = false;
                m_tackleClip = nullptr;
                m_getUpPhase = CharacterGetUpPhase3D::None;
                m_fallOrientation = CharacterFallOrientation3D::None;
                m_getUpPhaseSeconds = 0.0f;
                m_getUpClipSeconds = 0.0f;
                m_recoveredStandingSeconds = 0.0f;
                m_getUpAttempt = 0;
                m_getUpExpectedValid = false;
                m_tackleCooldown = 0.35f;
                // Segue para onde o corpo terminou virado; os pes ficam onde
                // estao (os passos refazem a base), o olhar volta aos poucos.
                const Vec3 forward = state.links.front().orientation.rotate({ 1.0f, 0.0f, 0.0f });
                m_facingYaw = std::hypot(forward.x, forward.y) > 0.2f
                    ? std::atan2(forward.y, forward.x) - m_clipPelvisYaw : m_tackleYaw;
                m_turningRate = 0.0f;
                m_plantFeetFromBody = true;
                m_torsoTwist = 0.0f;
                m_headTurn = 0.0f;
            }
        } else if (m_getUpPhase == CharacterGetUpPhase3D::Settling) {
            // Deitado: se rolou, a pose deitada (e o levantar) troca - com
            // folga, para nao ficar trocando de lado.
            if (m_chestLink < state.links.size()) {
                const float facing = state.links[m_chestLink].orientation
                    .rotate({ 1.0f, 0.0f, 0.0f }).z;
                if (m_fallOrientation == CharacterFallOrientation3D::FaceUp
                    && facing < -0.25f) {
                    m_fallOrientation = CharacterFallOrientation3D::FaceDown;
                } else if (m_fallOrientation
                        == CharacterFallOrientation3D::FaceDown
                    && facing > 0.25f) {
                    m_fallOrientation = CharacterFallOrientation3D::FaceUp;
                }
            }
            // Let the body actually come to rest before committing to a
            // recovery: which clip to play depends on which way it lands, and
            // that is not decided while it is still tumbling.
            float fastestLink = 0.0f;
            for (const PhysicsBodyState3D& link : state.links) {
                fastestLink = std::max(fastestLink,
                    link.linearVelocity.length());
            }
            // Tonus deitado ("ressustentacao"): enquanto o corpo ainda rola
            // ou desliza, os musculos cedem - a energia fica nas juntas em vez
            // de virar cambalhota do corpo inteiro, duro. Parado de vez, ele
            // fica largado ~0,7 s e o tonus volta aos poucos (~0,75 s),
            // enquanto ele se arruma para levantar (pose, mais abaixo); so
            // entao levanta.
            //
            // "Parado" e o tronco (pelve e peito), nao o membro mais rapido:
            // o proprio tonus voltando leva bracos e pernas para a pose
            // deitada, e medido pelos membros isso contava como "ainda
            // rolando" - o tonus desabava de novo, em ciclo, ate o limite de
            // tempo. Depois que ele comecou a voltar, so um tranco de verdade
            // no tronco o derruba de novo.
            float coreSpeed = state.links.front().linearVelocity.length();
            if (m_chestLink < state.links.size()) {
                coreSpeed = std::max(coreSpeed,
                    state.links[m_chestLink].linearVelocity.length());
            }
            // Na primeira vez fica largado mexendo-se; numa nova tentativa (ja
            // acordado, tentando) espera pouco.
            const float restBeforeGather = m_getUpAttempt > 1
                ? RetryRestBeforeGather : SettleRestBeforeGather;
            const bool recovering = m_settleRestSeconds > restBeforeGather;
            // Arrumando-se para levantar ele mesmo mexe o tronco (de lado para
            // de costas e um rolar): so um tranco de verdade interrompe.
            // Depois de parar, os proprios movimentos deitado (erguer a
            // cabeca, apoiar nos cotovelos) tambem nao contam como rolar.
            const float restLimit = recovering ? 1.50f
                : m_settleRestSeconds > 0.25f ? 0.80f : 0.35f;
            m_settleRestSeconds = coreSpeed < restLimit
                ? m_settleRestSeconds + deltaTime : 0.0f;
            m_settleTone = m_settleRestSeconds > restBeforeGather
                ? std::min(1.0f, m_settleTone + deltaTime * SettleToneRise * getUpSpeed)
                : std::max(SettleRestingTone, m_settleTone - deltaTime * 2.5f);
            // Rolar antes de levantar (TEMPTUNE GetUpRoll): de lado (ou de
            // costas, com GetUpRollFromBack), arrumando-se, ele rola ate
            // ficar de bruços - o levantar de bruços parte de uma pose
            // boa. Como no Prone Roll de referencia: a perna e o braco de
            // cima cruzam por cima para o lado da barriga e o quadril puxa o
            // tronco (uns 20 graus a frente dos ombros).
            if (GetUpRollEnabled && m_chestLink < state.links.size()) {
                const Vec3 chestForward = state.links[m_chestLink].orientation.rotate({ 1.0f, 0.0f, 0.0f });
                const Vec3 spine = (state.links[m_chestLink].position - state.links.front().position).normalized();
                if (!m_getUpRolling && !m_getUpRollDone && recovering && m_settleTone > 0.6f
                    && (std::abs(chestForward.z) < GetUpRollSideFacing
                        || (GetUpRollFromBack && chestForward.z > 0.0f))) {
                    // Gira em torno da coluna no sentido que leva a barriga
                    // para o chao: d(frente.z)/d(angulo) = (coluna x frente).z.
                    const float lean = cross(spine, chestForward).z;
                    m_getUpRollSign = lean > 0.0f ? -1.0f : 1.0f;
                    // O lado de cima: o quadril que sobe com o giro.
                    const std::size_t thighs[2] = { findLink(profile, "LeftThigh"), findLink(profile, "RightThigh") };
                    m_getUpRollTop = -1;
                    float best = -1e9f;
                    for (int side = 0; side < 2; ++side) {
                        if (thighs[side] == 0 || thighs[side] >= state.links.size()) continue;
                        const Vec3 hip = state.links[thighs[side]].position - state.links.front().position;
                        const float rise = cross(spine * m_getUpRollSign, hip).z;
                        if (rise > best) { best = rise; m_getUpRollTop = side; }
                    }
                    m_getUpRolling = m_getUpRollTop >= 0;
                    m_getUpRollSeconds = 0.0f;
                }
                if (m_getUpRolling) {
                    m_getUpRollSeconds += deltaTime;
                    const bool prone = chestForward.z < -0.6f;
                    if (prone || m_getUpRollSeconds > GetUpRollTimeout) {
                        m_getUpRolling = false;
                        m_getUpRollDone = true;
                        if (prone) m_fallOrientation = CharacterFallOrientation3D::FaceDown;
                        else classifyFallOrientation();
                        // Arruma-se de novo a partir de onde rolou.
                        m_settlePoseValid = false;
                    }
                }
            }
            // Levantar so com o corpo deitado: invertido (pernas por cima da
            // cabeca, pelve alem de ~125 graus) o clipe comecava de uma pose
            // que o corpo nao tinha e ele rolava de novo.
            const float settledTilt = std::acos(std::clamp(rootUpright, -1.0f, 1.0f));
            // E com o corpo ja na pose de levantar (se arrumou): um braco
            // preso embaixo do corpo nao chegava, e ao levantar os ganhos
            // o arrancavam de la (mao a 14 m/s).
            const bool settled = m_getUpPhaseSeconds > (getUpHurry > 0 ? GetUpHurrySettleSeconds : GetUpSettleSeconds)
                && !m_getUpRolling
                && coreSpeed < 0.35f * (getUpHurry > 0 ? GetUpCalmScale : 1.0f)
                && fastestLink < 1.0f * (getUpHurry > 0 ? GetUpCalmScale : 1.0f)
                && m_settleTone > 0.97f && settledTilt < 2.2f
                && m_telemetry.jointErrorRmsDegrees < 12.0f;
            // No modo fisico o corpo cai com mais energia e desliza mais:
            // comecar o levantar com ele ainda andando (o limite de 2 s)
            // puxava a raiz de uma vez para o clipe - membros a 77 m/s.
            const float settleLimit = input.forceDrivenRoot ? 4.0f : 2.0f;
            if ((settled || m_getUpPhaseSeconds > settleLimit + (m_getUpRollDone ? 2.0f : 0.0f))
                && !m_getUpRolling) {
                // O lado (costas/bruços) e o do deitado, que ja tem folga:
                // reclassificar aqui, de lado (peito na vertical), trocava o
                // levantar para o outro clipe com o corpo arrumado para este
                // (mao a 14 m/s, levantar emperrado).
                m_getUpClipSeconds = 0.0f;
                if (!state.links.empty()) {
                    // The clip plays where the body is lying, not where the
                    // capsule happens to be standing.
                    const Vec3 root = state.links.front().position;
                    m_getUpAnchorWorld = { root.x, root.y, root.z };
                    const Vec3 forward = state.links.front()
                        .orientation.rotate({ 1.0f, 0.0f, 0.0f });
                    const float planar = std::hypot(forward.x, forward.y);
                    m_getUpAnchorYaw = planar > 0.05f
                        ? std::atan2(forward.y, forward.x) : m_facingYaw;
                }
                m_getUpRate = 1.0f;
                m_getUpStruggle = 0.0f;
                m_getUpStallSeconds = 0.0f;
                m_getUpHoldSeconds = 0.0f;
                m_getUpExpectedValid = false;
                m_risingFromMeasured.assign(state.joints.size(), Vec3 {});
                for (std::size_t link = 1; link < state.joints.size(); ++link) {
                    m_risingFromMeasured[link] = { state.joints[link].positionRadians[0],
                        state.joints[link].positionRadians[1],
                        state.joints[link].positionRadians[2] };
                }
                enterGetUpPhase(CharacterGetUpPhase3D::Rising);
                m_bodilyRising = GetUpBodily;
                if (m_bodilyRising) {
                    m_bodilyGetUp.begin(profile, state, std::isfinite(contactFloor)
                        ? contactFloor : groundHeightGuess);
                }
            }
        } else if (m_getUpPhase == CharacterGetUpPhase3D::Rising && m_bodilyRising) {
            // Levantar pelo corpo: as etapas decidem; nada carrega o corpo.
            const bool standing = m_bodilyGetUp.update(profile, state, std::isfinite(contactFloor)
                ? contactFloor : groundHeightGuess, deltaTime, m_bodilyPose);
            if (m_bodilyGetUp.failed()) {
                classifyFallOrientation();
                enterGetUpPhase(CharacterGetUpPhase3D::Settling);
                ++m_getUpAttempt;
                m_settleRestSeconds = 0.0f;
                m_settlePoseValid = false;
                m_bodilyRising = false;
            } else if (standing) {
                m_getUpPhase = CharacterGetUpPhase3D::None;
                m_fallOrientation = CharacterFallOrientation3D::None;
                m_getUpPhaseSeconds = 0.0f;
                m_getUpClipSeconds = 0.0f;
                m_recoveredStandingSeconds = 0.0f;
                m_getUpAttempt = 0;
                m_bodilyRising = false;
                const Vec3 forward = state.links.front().orientation.rotate({ 1.0f, 0.0f, 0.0f });
                m_facingYaw = std::hypot(forward.x, forward.y) > 0.2f
                    ? std::atan2(forward.y, forward.x) - m_clipPelvisYaw : m_facingYaw;
                m_turningRate = 0.0f;
                m_plantFeetFromBody = true;
                m_torsoTwist = 0.0f;
                m_headTurn = 0.0f;
            }
        } else if (m_getUpPhase == CharacterGetUpPhase3D::Rising) {
            // Levantar biomecanico: o clipe e o roteiro das juntas; quem
            // levanta e o corpo. O tempo do clipe anda no ritmo que o corpo
            // acompanha (altura e inclinacao da pelve contra as do clipe):
            // atrasado, o clipe espera. Nada carrega a raiz - antes ela era
            // levada ao clipe, o "teleporte" (pelve a 677 mm num tick).
            if (m_getUpExpectedValid && !state.links.empty()) {
                const float ground = input.rootPositionWorld.z
                    - profile.standingRootHeightMeters;
                const float height = state.links.front().position.z - ground;
                const float tilt = std::acos(std::clamp(rootUpright, -1.0f, 1.0f));
                const float lowBy = m_getUpExpectedHeight - height;
                const float tiltedBy = tilt - m_getUpExpectedTilt;
                const float heightRate = 1.0f - (lowBy - 0.06f) / 0.14f;
                // Na fase final (o clipe ja quase em pe) a inclinacao pesa
                // mais: com a pelve inclinada alem do clipe ele nao segue
                // subindo - primeiro endireita, depois fica de pe. Antes o
                // clipe seguia e ele subia como uma prancha inclinada.
                const bool finalPhase = m_getUpExpectedTilt < 0.8f
                    && height > 0.55f;
                // (Ja na altura de pe - pelve acima de GetUpLateHeight - o
                // clipe nao espera mais a inclinacao: esperando, os alvos do
                // quadril e da coluna ficavam dobrados e o corpo, de pe e
                // inclinado, ~0,55 s ate a ajuda o erguer. Seguindo, eles vao
                // a pose reta e erguem o tronco.)
                const float tiltRate = finalPhase && height > GetUpLateHeight ? 1.0f
                    : finalPhase
                    ? 1.0f - (tiltedBy - 0.12f) / 0.25f
                    : 1.0f - (tiltedBy - 0.21f) / 0.44f;
                // As pernas tambem: se coxas e canelas estao longe da pose
                // do clipe (os joelhos ainda nao vieram para baixo do corpo),
                // o clipe espera - subir a pelve com as pernas esticadas para
                // tras deixava o corpo numa ponte de cabeca no chao.
                float legsBehind = 0.0f;
                for (std::size_t index = 1; index < profile.links.size()
                        && index < state.joints.size()
                        && index < m_coordinates.size(); ++index) {
                    const std::string& id = profile.links[index].id;
                    if (id.find("Thigh") == std::string::npos
                        && id.find("Shin") == std::string::npos) continue;
                    const Vec3 measured { state.joints[index].positionRadians[0],
                        state.joints[index].positionRadians[1],
                        state.joints[index].positionRadians[2] };
                    legsBehind = std::max(legsBehind,
                        (m_coordinates[index] - measured).length());
                }
                const float legsRate = 1.0f - (legsBehind - 0.45f) / 0.50f;
                // O peito tambem: de bruços, a flexao de bracos (peito fora do
                // chao) vem antes de puxar os joelhos. Sem esperar por ela o
                // clipe seguia com o peito no chao e o corpo se encolhia de
                // cabeca no chao, arrastando-a.
                float chestRate = 1.0f;
                if (m_chestLink < state.links.size()) {
                    const float chest = state.links[m_chestLink].position.z - ground;
                    chestRate = 1.0f
                        - (m_getUpExpectedChestHeight - chest - 0.08f) / 0.15f;
                }
                // O ritmo acelerado (getUpSpeed) so onde o clipe nao sobe
                // rapido: onde a pelve dele sobe, no maximo GetUpMaxRise m/s
                // (pedido do usuario, 05/10 noite: a 1,9x o empurrao dos
                // bracos e o ficar de pe pediam ~2,1 m/s e o corpo saltava,
                // ~0,25 s fora do chao em cada um).
                float speedHere = getUpSpeed;
                {
                    const AnimationClip3D* riseClip = m_fallOrientation == CharacterFallOrientation3D::FaceDown
                        ? animations.standUpFront : animations.standUpBack;
                    if (riseClip != nullptr && riseClip->durationSeconds > 0.1f) {
                        const float now = std::min(m_getUpClipSeconds, riseClip->durationSeconds - 0.06f);
                        const float rise = (clipRootAt(profile, *riseClip, now + 0.06f).translationOffsetMeters.z
                            - clipRootAt(profile, *riseClip, now).translationOffsetMeters.z) / 0.06f;
                        if (rise > 1e-3f) speedHere = std::clamp(GetUpMaxRise / rise, 1.0f, getUpSpeed);
                        // E as pernas: esticando o joelho e o quadril rapido o
                        // corpo saltava (no fim, de pe, a 2,5x - a pelve a
                        // 1,05 m), mesmo com a pelve do clipe quase parada.
                        float legSpeed = 0.0f;
                        for (std::size_t index = 1; index < profile.links.size(); ++index) {
                            const std::string& id = profile.links[index].id;
                            if (id.find("Thigh") == std::string::npos && id.find("Shin") == std::string::npos) continue;
                            if (const AnimationTrack3D* track = findAnimationTrack3D(*riseClip, id)) {
                                const Vec3 a = sampleAnimationTrack3D(*track, now, riseClip->durationSeconds, false).jointPositionRadians;
                                const Vec3 b = sampleAnimationTrack3D(*track, now + 0.06f, riseClip->durationSeconds, false).jointPositionRadians;
                                legSpeed = std::max(legSpeed, (b - a).length() / 0.06f);
                            }
                        }
                        if (legSpeed > 1e-3f) speedHere = std::min(speedHere, std::max(1.0f, GetUpMaxLegSpeed / legSpeed));
                    }
                }
                m_getUpRate = std::clamp(std::min({ heightRate, tiltRate,
                    legsRate, chestRate }), 0.1f, 1.0f) * speedHere;
            } else {
                m_getUpRate = 1.0f;
            }
            // Emperrado, o esforco (e o teto da ajuda) sobe devagar.
            // (Sobe em ~0,5 s: mais devagar, o corpo desabava antes de a
            // ajuda responder - fase de sentar do levantar de costas.)
            m_getUpStruggle = m_getUpRate < 0.5f
                ? std::min(1.0f, m_getUpStruggle + deltaTime * 2.0f)
                : std::max(0.0f, m_getUpStruggle - deltaTime
                    * (m_getUpRate > 0.8f ? 1.0f : 0.0f));
            // Emperrado de verdade: o clipe esperando E o corpo sem subir nem
            // endireitar. Progredindo devagar (a pelve subindo, a inclinacao
            // caindo), ele nao desiste - antes desistia no meio da subida.
            bool progressing = false;
            if (!state.links.empty()) {
                const float tiltNow = std::acos(std::clamp(rootUpright, -1.0f, 1.0f));
                progressing = state.links.front().linearVelocity.z > 0.05f
                    || (m_previousTilt - tiltNow) / std::max(deltaTime, 1e-4f) > 0.08f;
            }
            m_getUpStallSeconds = m_getUpRate <= 0.2f && !progressing
                ? m_getUpStallSeconds + deltaTime : std::max(0.0f,
                    m_getUpStallSeconds - (progressing ? deltaTime : 3.0f));
            {
                const AnimationClip3D* frontClip = animations.standUpFront;
                const float release = GetUpHandRelease * (frontClip && frontClip->durationSeconds > 0.05f
                    ? frontClip->durationSeconds : 1.6f);
                const float before = m_getUpClipSeconds;
                m_getUpClipSeconds += deltaTime * m_getUpRate;
                // Os pes do equilibrio pedem: as maos so soltam com o corpo
                // sobre os pes (o clipe espera antes de solta-las).
                if (m_getUpBalanceHold && m_fallOrientation == CharacterFallOrientation3D::FaceDown)
                    m_getUpClipSeconds = std::max(before, std::min(m_getUpClipSeconds, release));
            }
            const AnimationClip3D* clip = m_fallOrientation
                    == CharacterFallOrientation3D::FaceDown
                ? animations.standUpFront : animations.standUpBack;
            const float duration = clip && clip->durationSeconds > 0.05f
                ? clip->durationSeconds : 1.6f;
            // Fim do clipe so conta com o corpo em pe de verdade; senao ele
            // segura a pose final (com a ajuda) um pouco, e desiste.
            const bool upright = rootUpright > 0.94f && !state.links.empty()
                && state.links.front().position.z
                    - (input.rootPositionWorld.z - profile.standingRootHeightMeters)
                    > profile.standingRootHeightMeters * 0.80f;
            // Perto da pose agachada do comeco da arrancada (Crouched To
            // Sprinting: pelve a 0,55 m e 45 graus, peito a 66, os pes no chao
            // embaixo da pelve): o padrao pode assumir com ela.
            m_getUpHandoffReady = false;
            if (getUpHurry > 0 && m_getUpClipSeconds >= GetUpHandoffProgress * duration && !state.links.empty()) {
                const float ground = input.rootPositionWorld.z - profile.standingRootHeightMeters;
                const Vec3 pelvisPosition = state.links.front().position;
                float chestTilt = 0.0f;
                if (m_chestLink < state.links.size()) {
                    const Vec3 chestUp = (state.links[m_chestLink].orientation
                        * profile.links[m_chestLink].modelOrientation.conjugate()).rotate({ 0.0f, 0.0f, 1.0f });
                    chestTilt = std::acos(std::clamp(chestUp.z, -1.0f, 1.0f));
                }
                const Vec3 forward { std::cos(m_getUpAnchorYaw), std::sin(m_getUpAnchorYaw), 0.0f };
                bool feetUnder = true;
                for (const FootPlant& foot : m_feet) {
                    if (!foot.valid || foot.linkIndex >= state.links.size()) { feetUnder = false; continue; }
                    const Vec3 at = state.links[foot.linkIndex].position;
                    feetUnder = feetUnder && at.z - ground < 0.22f
                        && std::abs(dot(Vec3 { at.x - pelvisPosition.x, at.y - pelvisPosition.y, 0.0f }, forward)) < GetUpHandoffFeet;
                }
                m_getUpHandoffReady = pelvisPosition.z - ground > GetUpHandoffHeight
                    && rootUpright > std::cos(GetUpHandoffTilt) && chestTilt < GetUpHandoffChestTilt && feetUnder;
            }
            // Com pressa: quase em pe ja sai (o resto do clipe e endireitar
            // parado, que a arrancada faz andando).
            const bool hurryExit = getUpHurry > 0 && m_getUpClipSeconds >= GetUpHurryExit * duration
                && rootUpright > GetUpHurryUpright && !state.links.empty()
                && state.links.front().position.z
                    - (input.rootPositionWorld.z - profile.standingRootHeightMeters)
                    > profile.standingRootHeightMeters * 0.75f;
            if (m_getUpClipSeconds >= duration && !upright) {
                m_getUpClipSeconds = duration;
                m_getUpHoldSeconds += deltaTime;
            }
            // (E um teto: oscilando para sempre sem terminar, tenta de novo.)
            if (m_getUpStallSeconds > 3.0f || m_getUpHoldSeconds > 1.5f
                || m_getUpPhaseSeconds > 9.0f) {
                // Nao deu: volta a deitar e tenta de novo (pode escolher o
                // outro clipe, se rolou).
                classifyFallOrientation();
                enterGetUpPhase(CharacterGetUpPhase3D::Settling);
                ++m_getUpAttempt;
                m_settleRestSeconds = 0.0f;
                m_settlePoseValid = false;
                m_getUpStallSeconds = 0.0f;
                m_getUpHoldSeconds = 0.0f;
                m_getUpStruggle = 0.0f;
            } else if ((m_getUpClipSeconds >= duration && upright) || hurryExit) {
                m_getUpPhase = CharacterGetUpPhase3D::None;
                m_fallOrientation = CharacterFallOrientation3D::None;
                m_getUpPhaseSeconds = 0.0f;
                m_getUpClipSeconds = 0.0f;
                m_recoveredStandingSeconds = 0.0f;
                m_getUpAttempt = 0;
                m_getUpHoldSeconds = 0.0f;
                m_getUpStallSeconds = 0.0f;
                m_getUpStruggle = 0.0f;
                m_getUpExpectedValid = false;
                // O rumo e o do corpo em pe, onde ele terminou de levantar.
                {
                    const Vec3 forward = state.links.front().orientation
                        .rotate({ 1.0f, 0.0f, 0.0f });
                    m_facingYaw = std::hypot(forward.x, forward.y) > 0.2f
                        ? std::atan2(forward.y, forward.x) - m_clipPelvisYaw
                        : m_getUpAnchorYaw;
                }
                m_getUpHurriedExit = getUpHurry > 0;
                m_turningRate = 0.0f;
                // Os pes ficam onde estao, travados no chao (abaixo, onde a
                // sonda do chao existe). Soltos, o IK levava os dois de uma vez
                // para a base parada - o "espacate" da troca. Os passos de
                // acomodacao refazem a base, um pe de cada vez.
                m_plantFeetFromBody = true;
                // O olhar entra de novo aos poucos (o filtro continua de zero).
                m_torsoTwist = 0.0f;
                m_headTurn = 0.0f;
            }
        }
    }
    const bool getUpActive = m_getUpPhase != CharacterGetUpPhase3D::None;
    m_sinceGetUpSeconds = getUpActive ? 0.0f
        : std::min(10.0f, m_sinceGetUpSeconds + deltaTime);
    // Cambaleio (ver StaggerImpulseMin): o choque deste tick e o tonus.
    {
        float impulse = 0.0f;
        if (input.staggerRecovery && physicsOwnsBody && !getUpActive)
            for (const RagdollInteraction3D& contact : state.interactions) {
                if (contact.linkIndex >= profile.links.size()) continue;
                const std::string& id = profile.links[contact.linkIndex].id;
                if ((id == "LeftFoot" || id == "RightFoot") && contact.normalWorld.z > 0.62f) continue;
                // (O chao logo depois de levantar ou do carrinho nao e choque.)
                if (contact.normalWorld.z > 0.62f && (m_sinceGetUpSeconds < 1.0f || m_sinceTackleSeconds < 0.8f))
                    continue;
                impulse += contact.normalImpulseNewtonSeconds;
            }
        if (!input.staggerRecovery || getUpActive) {
            m_stagger = 0.0f;
        } else {
            const float hit = std::clamp((impulse - StaggerImpulseMin)
                / std::max(1.0f, StaggerImpulseFull - StaggerImpulseMin), 0.0f, 1.0f);
            if (hit > m_stagger) {
                m_stagger = hit;
                m_staggerHold = StaggerHoldSeconds * hit;
            }
            const float upright = smoothStep((rootUpright - 0.60f) / 0.35f);
            if (m_staggerHold > 0.0f) m_staggerHold -= deltaTime;
            else m_stagger = std::max(0.0f, m_stagger - deltaTime * StaggerRecoverPerSecond * (0.35f + 0.65f * upright));
        }
        m_telemetry.stagger = m_stagger;
        m_telemetry.staggerImpulse = impulse;
    }
    // (Os limites da postura e da queda sem volta, mais tolerantes enquanto
    // ele esta amolecido: ele tenta se manter.)
    const float staggerRelief = StaggerTiltRelief * m_stagger;

    // Reflexos de queda: com o corpo tombando de verdade (pelve alem de ~40
    // graus da vertical; no modo antigo, a partir de 25 graus depois de uma
    // pancada) os reflexos entram e as juntas ficam com tonus - nao a 3% da
    // forca, que era o "corpo morto molengo". Deitado, fica inteiro; no
    // levantar, sai.
    {
        const float tilt = std::acos(std::clamp(rootUpright, -1.0f, 1.0f));
        // Tombando rapido (mais de 1,5 rad/s) os reflexos entram ja aos 25
        // graus: numa queda de lado o corpo vai ao chao em ~0,35 s, e
        // esperando os 40 graus as maos ainda estavam a 30 cm do chao quando
        // a cabeca batia. Inclinacao parada (a corrida inclina ate ~35) nao.
        const float tiltRate = (tilt - m_previousTilt) / std::max(deltaTime, 1e-4f);
        m_previousTilt = tilt;
        const float quickTipping = smoothStep((tilt - 0.35f) / 0.26f)
            * smoothStep((tiltRate - 1.0f) / 1.0f);
        const float tipping = !physicsOwnsBody ? 0.0f
            : input.forceDrivenRoot
                ? std::max(smoothStep((tilt - 0.70f) / 0.26f), quickTipping)
                : std::max(smoothStep((tilt - 0.44f) / 0.26f), quickTipping)
                    * smoothStep((m_physicsBlend - 0.3f) / 0.4f);
        const float wanted = m_getUpPhase == CharacterGetUpPhase3D::Settling
            ? 1.0f : m_getUpPhase == CharacterGetUpPhase3D::Rising ? 0.0f
            : tipping;
        m_fallReflex += (wanted - m_fallReflex) * std::clamp(
            deltaTime * (wanted > m_fallReflex ? 18.0f : 3.0f), 0.0f, 1.0f);
        // Desequilibrado nao e caido: com o corpo fisico, os bracos protegem
        // ja no comeco (acima), mas as pernas seguem no passo, com o tonus de
        // pe, ate a queda ficar sem volta - tombado alem de ~50 graus e ainda
        // indo (ou 40 graus girando muito rapido). Ir logo para as pernas
        // moles tirava a chance de o passo salvar o equilibrio e o comando do
        // jogador parava de valer no meio do tropeco.
        const float committing = input.forceDrivenRoot
            ? std::max(smoothStep((tilt - 0.87f - staggerRelief) / 0.22f)
                    * smoothStep((tiltRate + 0.3f) / 0.6f),
                smoothStep((tilt - 0.70f - staggerRelief) / 0.26f)
                    * smoothStep((tiltRate - 2.5f) / 1.0f))
            : tipping;
        const float commitWanted = m_getUpPhase == CharacterGetUpPhase3D::Settling
            ? 1.0f : m_getUpPhase == CharacterGetUpPhase3D::Rising ? 0.0f
            : std::max(committing, input.forceDrivenRoot && m_fallCommit > 0.5f
                ? smoothStep((tilt - 0.70f - staggerRelief) / 0.26f) : 0.0f);
        // De volta em pe (tronco a menos de ~25 graus), as pernas voltam logo
        // ao passo e ao tonus: com a volta lenta ele se empurrava de cima de
        // uma caixa, ficava reto, e caia de costas de pernas ainda moles.
        const float commitRelease = m_getUpPhase == CharacterGetUpPhase3D::None
            && tilt < 0.44f && (feetGroundContact[0] || feetGroundContact[1])
            ? 10.0f : 3.0f;
        // (Apoiado tentando se recompor: as pernas ficam com o tonus de pe.)
        const float commitTarget = m_braceRecovering ? 0.0f : commitWanted;
        m_fallCommit += (commitTarget - m_fallCommit) * std::clamp(
            deltaTime * (commitTarget > m_fallCommit ? 18.0f : (m_braceRecovering ? 10.0f : commitRelease)),
            0.0f, 1.0f);
        // Rumo da queda: para onde o centro de massa vai; devagar, para onde
        // o corpo tomba.
        Vec3 fall = m_filteredCenterOfMassVelocity;
        fall.z = 0.0f;
        if (fall.lengthSquared() < 0.16f && !state.links.empty()) {
            fall = state.links.front().orientation.rotate({ 0.0f, 0.0f, 1.0f });
            fall.z = 0.0f;
        }
        if (fall.lengthSquared() > 0.0004f) {
            const Vec3 unit = fall.normalized();
            m_fallDirection = (m_fallDirection + (unit - m_fallDirection)
                * std::clamp(deltaTime * 12.0f, 0.0f, 1.0f));
            if (m_fallDirection.lengthSquared() > 1e-6f)
                m_fallDirection = m_fallDirection.normalized();
        }
        // Postura perdida (ver PostureLossFall): com o corpo fisico, de pe
        // (sem levantar, carrinho nem pulo).
        const CharacterLocomotionState3D previousState = m_telemetry.state;
        const bool postureStanding = PostureLossFall && physicsOwnsBody && input.forceDrivenRoot
            && m_getUpPhase == CharacterGetUpPhase3D::None
            && previousState != CharacterLocomotionState3D::Tackling
            && previousState != CharacterLocomotionState3D::Fallen
            && previousState != CharacterLocomotionState3D::GettingUp
            && previousState != CharacterLocomotionState3D::Airborne
            && previousState != CharacterLocomotionState3D::JumpStarting
            && previousState != CharacterLocomotionState3D::Landing;
        // O pendulo (centro de massa contra o meio dos pes) da vertical e
        // contra o da pose pedida; e o peito da vertical.
        float deviation = 0.0f, pendulum = 0.0f, chestTilt = 0.0f;
        bool supportedPendulum = false;
        if (m_feet.size() >= 2 && m_feet[0].valid && m_feet[1].valid
            && m_feet[0].linkIndex < state.links.size() && m_feet[1].linkIndex < state.links.size()) {
            const Vec3 arm = pendulumArm(profile, m_feet[0].linkIndex, m_feet[1].linkIndex,
                [&](std::size_t i) { return state.links[i].position; });
            const Vec3 wanted = m_referencePendulumValid ? m_referencePendulum : Vec3 { 0.0f, 0.0f, 1.0f };
            const Vec3 armUnit = arm * (1.0f / std::max(1e-3f, arm.length()));
            const Vec3 wantedUnit = wanted * (1.0f / std::max(1e-3f, wanted.length()));
            deviation = std::acos(std::clamp(dot(armUnit, wantedUnit), -1.0f, 1.0f));
            pendulum = std::acos(std::clamp(armUnit.z, -1.0f, 1.0f));
            // v2 (T_PostureSupport): o pendulo e contra os pes APOIADOS (o do
            // alvo, contra os mesmos pes da pose alvo). Pelo meio dos dois,
            // correndo, o pe no ar - a 0,7 m para o lado no zigue-zague - o
            // inflava: a postura era dada como perdida com o corpo a 34 graus
            // e a ajuda saia de vez (a 100%, 15 de 174 rodadas caiam assim).
            // Sem pe apoiado (o voo da passada), o tempo segurado nao conta.
            // (So na passada - andando ou correndo, sem os passos de freada:
            // nos passos do planejador, parado, o pe no ar e o que vai segurar
            // o corpo - empurrado para tras, contra o pe apoiado, ele "perdia
            // a postura" no meio do passo que salvava.)
            const bool inGait = (previousState == CharacterLocomotionState3D::Walking
                || previousState == CharacterLocomotionState3D::Running) && !m_brakeStepping;
            if (PostureSupportPendulum && inGait) {
                const bool left = feetGroundContact[0], right = feetGroundContact[1];
                if (left || right) {
                    float total = 0.0f;
                    Vec3 center {}, wantedCenter {};
                    for (std::size_t i = 0; i < profile.links.size(); ++i) {
                        center = center + state.links[i].position * profile.links[i].massFraction;
                        total += profile.links[i].massFraction;
                    }
                    center = center * (1.0f / std::max(1e-4f, total));
                    const Vec3 support = left && right
                        ? (state.links[m_feet[0].linkIndex].position + state.links[m_feet[1].linkIndex].position) * 0.5f
                        : state.links[m_feet[left ? 0 : 1].linkIndex].position;
                    const Vec3 wantedSupport = left && right
                        ? (m_referenceFeetWorld[0] + m_referenceFeetWorld[1]) * 0.5f
                        : m_referenceFeetWorld[left ? 0 : 1];
                    const Vec3 supportArm = center - support;
                    const Vec3 wantedArm = m_referencePendulumValid ? m_referenceCenterOfMassWorld - wantedSupport
                        : Vec3 { 0.0f, 0.0f, 1.0f };
                    const Vec3 supportUnit = supportArm * (1.0f / std::max(1e-3f, supportArm.length()));
                    const Vec3 wantedSupportUnit = wantedArm * (1.0f / std::max(1e-3f, wantedArm.length()));
                    deviation = std::acos(std::clamp(dot(supportUnit, wantedSupportUnit), -1.0f, 1.0f));
                    pendulum = std::acos(std::clamp(supportUnit.z, -1.0f, 1.0f));
                    supportedPendulum = true;
                }
            }
        }
        if (m_chestLink != 0 && m_chestLink < state.links.size()) {
            const Quaternion bind = profile.links[m_chestLink].modelOrientation.conjugate();
            const Vec3 chestUp = (state.links[m_chestLink].orientation * bind).rotate({ 0.0f, 0.0f, 1.0f });
            chestTilt = std::acos(std::clamp(chestUp.z, -1.0f, 1.0f));
        }
        // O tempo segurado vaza devagar em vez de zerar: segurado em dois
        // lobos de ~0,3 s (39 graus, o peito a 48, sem pe no chao), com um
        // vale de 0,1 s entre eles, ele nunca contava.
        const bool pendulumCounts = !PostureSupportPendulum || supportedPendulum
            || !((previousState == CharacterLocomotionState3D::Walking
                || previousState == CharacterLocomotionState3D::Running) && !m_brakeStepping);
        if (!postureStanding) m_postureHeldSeconds = 0.0f;
        else if (!pendulumCounts) {}
        else if (pendulum > PostureHeldTilt + staggerRelief && deviation > PostureHeldDeviation + staggerRelief)
            m_postureHeldSeconds += deltaTime;
        else m_postureHeldSeconds = std::max(0.0f, m_postureHeldSeconds - deltaTime);
        if (m_braceRecovering) {
            // Apoiado e tentando se recompor: nao e postura perdida.
            m_postureLost = false;
            m_postureHeldSeconds = 0.0f;
        } else if (m_postureLost) {
            // Caiu de vez: ate ir ao chao (o levantar comeca) ou se, mesmo
            // assim, ficar de pe.
            if (m_getUpPhase != CharacterGetUpPhase3D::None || (pendulumCounts && pendulum < PostureRegainedTilt))
                m_postureLost = false;
        } else if (postureStanding && ((pendulumCounts && pendulum > PostureLostTilt + staggerRelief
                && !(PostureSupportPendulum && supportedPendulum))
            // (Na passada, pelo pendulo so o sustentado - abaixo: na hora,
            // um pe pousando longe a frente numa freada dava 43 graus com o
            // corpo indo para cima dele. Na hora vale a pelve tombada.)
            || (PostureSupportPendulum && supportedPendulum
                && std::acos(std::clamp(rootUpright, -1.0f, 1.0f)) > PostureGaitPelvisLost + staggerRelief)
            || chestTilt > PostureChestLost + staggerRelief
            || m_postureHeldSeconds > PostureHeldSeconds + 0.4f * m_stagger)) {
            m_postureLost = true;
        }
        if (m_postureLost) {
            // As pernas cedem e os reflexos entram ja (a ajuda sai pelo
            // controlador: telemetry.postureLost).
            m_fallCommit = 1.0f;
            m_fallReflex = 1.0f;
        }
        m_telemetry.postureDeviation = deviation;
        m_telemetry.posturePendulum = pendulum;
        m_telemetry.postureChestTilt = chestTilt;
        m_telemetry.postureLost = m_postureLost;
        m_telemetry.getUpHurried = m_getUpHurriedExit && m_sinceGetUpSeconds < 2.0f;
        m_telemetry.getUpHandoffReady = m_getUpHandoffReady && m_getUpPhase == CharacterGetUpPhase3D::Rising;
    }

    // How much of the simulated body the player actually sees. Zero is the
    // normal case and means the rendered pose IS the authored pose, bit for
    // bit — the bodies still exist and still collide, they just do not get a
    // say in how the character looks. This is the same split Unreal draws
    // with PhysicsBlendWeight (see PhysAnim.cpp): animation owns the pose,
    // physics is blended in per body by weight, and at weight zero the
    // simulated result is discarded entirely instead of being "mostly
    // suppressed". Letting physics always contribute a little is what made
    // the character stand crooked, lean while walking and jitter.
    //
    // Only a dynamic body can raise it: another character, a prop, a
    // scripted moment later on. Walls, floors and curbs are static and
    // deliberately never push the character out of animation - clipping a
    // wall is a pose problem, not a reason to go limp.
    float dynamicImpulse = 0.0f;
    for (const RagdollContactPoint3D& contact : state.contacts) {
        if (!contact.otherBodyDynamic) continue;
        dynamicImpulse += std::max(0.0f, contact.normalImpulseNewtonSeconds);
    }
    // Scaled against what it actually takes to move this body, not against
    // its weight over a tick: a crate merely leaning on the character
    // delivers about mass*g*dt of impulse every tick forever, and measuring
    // against that made any resting contact read as a full-force hit and
    // drop the character on the spot. The gate is a real change in momentum
    // - ignore anything under ~0.05 m/s worth, reach full physics at
    // ~0.45 m/s worth, which is a genuine shove.
    const float mass = std::max(1.0f, profile.totalMassKg);
    const float deadzone = mass * 0.05f;
    const float fullScale = mass * 0.45f;
    // Com a raiz livre nao ha limiar de impacto: a colisao age direto no
    // corpo e a cedencia local vem do controlador fisico.
    const float impact = input.forceDrivenRoot ? 0.0f : std::clamp(
        (dynamicImpulse - deadzone) / std::max(0.01f, fullScale - deadzone),
        0.0f, 1.0f);
    if (impact > 0.0f) {
        // Proportional to the hit, and it never snaps: a shove pushes the
        // weight up quickly, then it holds while contact lasts.
        const float wanted = impact;
        m_physicsBlend = std::max(m_physicsBlend,
            std::min(1.0f, m_physicsBlend + wanted * deltaTime * 9.0f));
        m_physicsHoldSeconds = std::max(m_physicsHoldSeconds,
            0.25f + wanted * 0.75f);
    } else if (m_physicsHoldSeconds > 0.0f) {
        m_physicsHoldSeconds = std::max(0.0f,
            m_physicsHoldSeconds - deltaTime);
    } else {
        // Deliberately slower than the way in: coming back the instant
        // contact breaks is what made bumping another character read as
        // nothing happening at all.
        m_physicsBlend = std::max(0.0f, m_physicsBlend - deltaTime * 0.65f);
    }
    if (input.manipulated) {
        // Held by the PhysGun: the player is moving the body itself, so the
        // body is what they must see. Rendering the animation here made a
        // grab look like it was doing nothing while the bodies flew around.
        m_physicsBlend = 1.0f;
        m_physicsHoldSeconds = std::max(m_physicsHoldSeconds, 0.35f);
    }
    if (m_getUpPhase == CharacterGetUpPhase3D::Settling) {
        m_physicsBlend = 1.0f;
    } else if (m_getUpPhase == CharacterGetUpPhase3D::Rising) {
        // Hand the pose back to the clip over a short cross-fade so the
        // recovery starts from where the body actually is instead of popping
        // to the clip's first frame.
        m_physicsBlend = std::max(0.0f,
            1.0f - std::clamp(m_getUpClipSeconds / 0.30f, 0.0f, 1.0f));
    }
    m_reactionStrength = m_physicsBlend;
    const float requestedAuthority = std::clamp(
        input.poseAuthority, 0.0f, 1.0f);
    // At zero blend the guide holds the bodies exactly on the authored pose.
    // They still collide and still push props around; they simply have
    // nothing to contribute back, because the renderer is reading the
    // authored pose directly (see SpawnedRagdollInstance::animationPose).
    // Carried or free - never in between. A partial authority makes the
    // backend rewrite the articulation's positions AND velocities every
    // tick (applyCache), which injects energy on every frame: that is the
    // constant shiver, and it also wipes the friction the feet had just
    // accumulated, so the character slid like soap when something pushed
    // him. Handing the body over is a physical event, not a dial: while it
    // is carried nothing is written except the root, and the moment physics
    // takes over nothing is written at all.
    // Com a raiz livre, so o levantar (um clipe com trilha de raiz) ainda
    // carrega a pelve.
    // O levantar tambem nao carrega mais a pelve: e o corpo que levanta.
    const bool carried = !input.manipulated && !input.forceDrivenRoot
        && !getUpActive && m_physicsBlend < 0.35f;
    const float rootAuthority = carried ? requestedAuthority : 0.0f;

    Vec3 desiredPlanarVelocity { input.proxyVelocityWorld.x,
        input.proxyVelocityWorld.y, 0.0f };
    // Com pouca ajuda a passada acompanha a inercia do corpo: no maximo
    // GaitLeadSpeed acima do que ele anda no rumo pedido. A capsula corre no
    // pedido desde o primeiro tick; no passo dela, o sprint inteiro (7,5 m/s)
    // saia com o corpo a 0,5 m/s - o pe pousava 0,8 m a frente e ele caia de
    // costas. Assim a cadencia e o passo crescem com o corpo.
    if (GaitLeadSpeed > 0.0f && input.forceDrivenRoot && !getUpActive && m_brakeGait <= 0.0f) {
        const float lowAssistLead = smoothStep((0.5f - std::clamp(input.walkingAssist, 0.0f, 1.0f)) / 0.4f);
        const float proxySpeed = std::hypot(desiredPlanarVelocity.x, desiredPlanarVelocity.y);
        // So correndo (pedido acima do trote): no trote a folga freava o
        // corpo (2,0 -> 1,8 m/s e paradas caindo).
        const float requestedSpeed = std::hypot(input.intent.requestedVelocityWorld.x, input.intent.requestedVelocityWorld.y);
        if (lowAssistLead > 0.0f && proxySpeed > 0.3f && requestedSpeed > GaitLeadMinimumRequest) {
            const Vec3 direction = desiredPlanarVelocity * (1.0f / proxySpeed);
            const float along = std::max(0.0f, dot(Vec3 { m_filteredCenterOfMassVelocity.x,
                m_filteredCenterOfMassVelocity.y, 0.0f }, direction));
            const float led = std::min(proxySpeed, along + GaitLeadSpeed);
            desiredPlanarVelocity *= 1.0f + (led / proxySpeed - 1.0f) * lowAssistLead;
        }
    }
    float gaitVelocityScale = 1.0f;
    // Depois de uma pancada ou desequilibrio, as pernas continuam no rumo
    // pedido, mas no passo do corpo: o que ele anda de verdade naquele rumo
    // mais uma folga. Na passada do comando (correndo a 7 m/s com o corpo
    // parado numa caixa, ou indo para tras), os dois pes ficavam no ar na
    // fase de voo e nenhum passo segurava o corpo. Livre de novo, o teto sobe
    // no ritmo de uma arrancada.
    if (input.forceDrivenRoot) {
        const float disturbed = std::max(
            std::clamp(input.externalPerturbation, 0.0f, 1.0f), m_fallReflex);
        const float wantedSpeed = std::hypot(desiredPlanarVelocity.x,
            desiredPlanarVelocity.y);
        float along = 0.0f;
        if (wantedSpeed > 0.05f) {
            along = dot(Vec3 { m_filteredCenterOfMassVelocity.x,
                m_filteredCenterOfMassVelocity.y, 0.0f },
                desiredPlanarVelocity * (1.0f / wantedSpeed));
        }
        const float target = disturbed > 0.2f && !getUpActive
            ? std::max(0.0f, along) + 1.2f : 1000.0f;
        m_gaitSpeedCap = target < m_gaitSpeedCap
            ? std::max(target, m_gaitSpeedCap - 30.0f * deltaTime)
            : std::min(target, m_gaitSpeedCap + 6.0f * deltaTime);
        // Sem limitar, o teto acompanha o pedido logo acima dele (e numa
        // pancada cai dali): uma arrancada normal nunca e contida.
        m_gaitSpeedCap = std::min(m_gaitSpeedCap, wantedSpeed + 1.0f);
        if (target > 999.0f && m_gaitSpeedCap >= wantedSpeed)
            m_gaitSpeedCap = wantedSpeed + 1.0f;
        if (wantedSpeed > m_gaitSpeedCap) {
            gaitVelocityScale = m_gaitSpeedCap / wantedSpeed;
            desiredPlanarVelocity = desiredPlanarVelocity * gaitVelocityScale;
        }
    }
    // Freada pela passada (modo fisico). Soltando o comando correndo (ou
    // pedindo bem menos, ou o sentido contrario), a capsula freia a 18 m/s2
    // e para em ~0,4 s; o corpo, so pelos pes, freia ~5-6 m/s2. Seguindo a
    // capsula, a passada encurtava e parava com o corpo ainda a ~5 m/s: o
    // planejador de parado dava passos de recuperacao curtos e o corpo
    // passava por cima deles (sprint e para, ajuda parada em 0%: 6 quedas em
    // 8 direcoes). E o pe que pousava era posto em relacao a capsula, que vem
    // arrastada ate 0,5-0,7 m atras do corpo: pousava embaixo dele ou atras.
    // Enquanto o corpo leva um momento que o jogador nao pede, a passada
    // continua no passo do corpo real, num perfil de freada que ele consegue
    // cumprir, a referencia fica presa ao corpo (nao a capsula) e o pe pousa
    // a frente do centro de massa; a pelve desce e o tronco segura atras
    // (reacao de freada, mais abaixo). Devagar, o planejador de parado assume.
    Vec3 brakeNavigationVelocity {};
    {
        const Vec3 bodyPlanar { m_filteredCenterOfMassVelocity.x,
            m_filteredCenterOfMassVelocity.y, 0.0f };
        const float bodySpeed = bodyPlanar.length();
        const Vec3 requested { input.intent.requestedVelocityWorld.x,
            input.intent.requestedVelocityWorld.y, 0.0f };
        const Vec3 direction = bodySpeed > 0.05f
            ? bodyPlanar * (1.0f / bodySpeed) : m_brakeGaitDirection;
        const float wantedAlong = dot(requested, direction);
        // O momento que o jogador nao pede: o que passa da velocidade pedida
        // no rumo do corpo (pedindo o contrario, tudo).
        const float unwanted = bodySpeed - std::max(0.0f, wantedAlong);
        // Para seguir freando o chao da capsula nao conta: arrastada atras do
        // corpo ela pisca "no ar" e a freada terminava no meio dos passos.
        const bool continuable = input.forceDrivenRoot && !getUpActive
            && !input.manipulated && m_fallCommit < 0.5f;
        const bool eligible = continuable && input.grounded;
        // So soltando o comando (ou quase) ou pedindo o sentido contrario:
        // numa curva ou num corte de lado a passada continua e vira com a
        // capsula, como antes.
        const float requestedSpeed = requested.length();
        // Com pouca ajuda, um corte forte (60 graus ou mais) com o corpo rapido
        // tambem e freada: quem leva o corpo para o novo rumo sao os pes, e a
        // 6 m/s eles curvam ~1 rad/s - virando com a capsula, a passada ia
        // para o rumo novo com o corpo seguindo reto, as pernas cruzavam e
        // ele mergulhava (corte de 90 graus a 30%: 6-8 quedas em 8). Freia
        // plantando o pe, pivota e sai no rumo novo (como na inversao).
        const float cutAssist = smoothStep((0.6f - std::clamp(input.walkingAssist, 0.0f, 1.0f)) / 0.5f);
        const bool cutting = cutAssist > 0.3f && bodySpeed > CutBrakeMinimumSpeed
            && dot(requested, direction) < CutBrakeCosine * requestedSpeed;
        const bool stopping = requestedSpeed < 0.5f
            || dot(requested, direction) < -0.5f * requestedSpeed || cutting;
        // E so o momento da propria corrida (o jogador pedia movimento ha
        // pouco e nada de fora bateu nele): um empurrao forte parado (300
        // N.s, o corpo a ~4 m/s) e da recuperacao de empurrao - pela freada
        // ele perdia os passos de recuperacao e caia.
        m_sinceRunRequest = requestedSpeed > 1.0f ? 0.0f
            : std::min(10.0f, m_sinceRunRequest + deltaTime);
        const bool ownMomentum = m_sinceRunRequest < 0.6f
            && input.externalPerturbation < 0.2f;
        if (!m_brakeGaitActive) {
            // De lado (strafe), a freada pela passada entra mais devagar: o
            // strafe tem passo cruzado e, parando sem ela a 1,6 m/s, os
            // passos de parado herdavam os pes cruzados com o corpo indo para
            // o lado - a 5%, 14 quedas em 16 parando de um strafe.
            const bool sideways = m_gaitDirection == CharacterGaitDirection3D::Left
                || m_gaitDirection == CharacterGaitDirection3D::Right;
            const float engageSpeed = sideways ? StrafeBrakeEngageSpeed : BrakeEngageSpeed;
            const float engageExcess = sideways ? std::min(BrakeEngageExcessSpeed, StrafeBrakeEngageSpeed * 0.6f)
                : BrakeEngageExcessSpeed;
            if (eligible && stopping && ownMomentum && bodySpeed > engageSpeed
                && unwanted > engageExcess) {
                m_brakeGaitActive = true;
                m_brakeGaitSpeed = bodySpeed;
            }
        } else if (!continuable
            // A freada nunca prende o comando: pedindo movimento para um lado
            // qualquer (que nao o contrario) ela acaba na hora e a passada
            // assume; pedindo o contrario, so ate o corpo ficar devagar.
            || (requestedSpeed >= 0.5f && (!stopping || bodySpeed < BrakeReverseReleaseSpeed))
            || (m_brakeStepping && m_footwork.plan().brakeYield)
            || ((bodySpeed < BrakeReleaseSpeed || unwanted < 0.5f)
                && (!m_brakeStepping || m_footwork.plan().brakeSettled
                    || m_brakeSeconds > BrakeMaximumSeconds))) {
            // Nos passos, so termina com o corpo parando sobre a base (o ponto
            // de captura entre os pes, a frente do de tras - o planejador
            // diz): parado com a base toda adiante do corpo, ele escorregava
            // de volta e os passos de parado, sem ajuda, davam dois ou tres
            // passos para tras (a "cambaleada").
            m_brakeGaitActive = false;
        }
        m_brakeSeconds = m_brakeGaitActive ? m_brakeSeconds + deltaTime : 0.0f;
        m_brakeReversing = m_brakeGaitActive && requestedSpeed >= 0.5f;
        if (m_brakeGaitActive) {
            // Quase parado, a direcao da velocidade gira a toa (e ate inverte):
            // fica a de quando ele ainda andava.
            if (bodySpeed > 0.8f || m_brakeGaitDirection.lengthSquared() < 0.5f)
                m_brakeGaitDirection = direction;
            // Perfil de freada: desce ate o pedido no rumo (ou zero) no ritmo
            // que os pes conseguem, sem se afastar do corpo real mais do que
            // uma folga - a passada tem de acompanhar o chao que o corpo anda.
            // Com pouca ajuda quem freia sao so as pernas (~2-4 m/s2): a
            // passada no perfil de 12 m/s2, ate 0,8 m/s abaixo do corpo, ficava
            // curta e lenta para o corpo que seguia rapido - os pes ficavam
            // atras dele e ele mergulhava. Ela fica perto do corpo.
            const float lowAssistBrake = smoothStep((0.5f - std::clamp(input.walkingAssist, 0.0f, 1.0f)) / 0.4f);
            const float deceleration = (m_brakeReversing ? ReverseDeceleration : BrakeDeceleration)
                + (LowAssistBrakeDeceleration - (m_brakeReversing ? ReverseDeceleration : BrakeDeceleration)) * lowAssistBrake;
            const float profileLeash = BrakeProfileLeash + (LowAssistBrakeLeash - BrakeProfileLeash) * lowAssistBrake;
            m_brakeGaitSpeed = std::max(std::max(0.0f, wantedAlong),
                m_brakeGaitSpeed - deceleration * deltaTime);
            m_brakeGaitSpeed = std::clamp(m_brakeGaitSpeed,
                std::max(0.0f, bodySpeed - profileLeash), bodySpeed);
        }
        // Rapido, a corrida desacelera (a perna de corrida e quem traz o pe a
        // frente a tempo); devagar, passos de freada sem voo ate parar.
        // (Pela velocidade do corpo: a do perfil fica abaixo dela e os passos
        // comecavam com o corpo ainda a 3,7 m/s, os pes atras dele.)
        // Com pouca ajuda (TEMPTUNE: em teste) os passos de freada podem
        // comecar mais cedo: a corrida freando depende do tempo do clipe (18%
        // de apoio por pe), e o corpo, sem ajuda, fica mais tempo no chao -
        // o pe que o clipe queria levar a frente ainda carregava o corpo.
        const float lowAssistSteps = smoothStep((0.5f - std::clamp(input.walkingAssist, 0.0f, 1.0f)) / 0.4f);
        const float stepSpeed = BrakeStepSpeed + (LowAssistBrakeStepSpeed - BrakeStepSpeed) * lowAssistSteps;
        // A troca espera um pe da passada no chao, a frente do corpo (o
        // apoio que ja freia - no maximo uma fase de voo): trocando com os
        // dois no ar, os dois pousavam como passos de freada, um atras do
        // outro, e a parada tinha sempre dois passos. Assim o pe plantado
        // freia e so o outro pousa - um passo.
        // (O apoio fisico do estimador: o "plantado" do clipe nem sempre marca
        // o pe que ja esta no chao, e a espera ia ate o limite com o corpo
        // freando a 0,4 m/s ainda na passada.)
        bool plantedAhead = false;
        for (const FootSupportEstimate3D& foot : physical.feet) {
            if (!foot.supporting) continue;
            plantedAhead = plantedAhead || dot(Vec3 { foot.soleCenterWorld.x - physical.centerOfMassWorld.x,
                foot.soleCenterWorld.y - physical.centerOfMassWorld.y, 0.0f }, direction) > BrakePlantedAhead;
        }
        m_brakeStepWait = m_brakeGaitActive && !m_brakeStepping && bodySpeed < stepSpeed
            ? m_brakeStepWait + deltaTime : 0.0f;
        const bool stepReady = plantedAhead || m_brakeStepWait > BrakeStepWaitSeconds;
        const bool enteringSteps = m_brakeGaitActive && !m_brakeStepping && bodySpeed < stepSpeed && stepReady;
        m_brakeStepping = m_brakeGaitActive
            && (m_brakeStepping || (bodySpeed < stepSpeed && stepReady));
        // Freando ja devagar (do trote), os passos comecam na hora, com a
        // base no corpo desde o primeiro tick.
        if (enteringSteps && m_brakeGait <= 0.0f) m_brakeAnchor = 1.0f;
        m_sinceBrakeSeconds = m_brakeGaitActive ? 0.0f
            : std::min(10.0f, m_sinceBrakeSeconds + deltaTime);
        m_brakeGait = m_brakeGaitActive
            ? std::min(1.0f, m_brakeGait + deltaTime / 0.08f)
            : std::max(0.0f, m_brakeGait - deltaTime / BrakeFadeSeconds);
        // Correndo e freando, a corrida ainda e corrida: a ajuda e a de quem
        // corre (a passada de corrida ainda depende dela), e vai para a de
        // parado conforme a velocidade cai ate os passos de freada.
        // A ajuda de corrida fica inteira enquanto a passada de corrida freia
        // (trotar sem ela ainda nao se sustenta) e sai depressa quando os
        // passos de freada comecam - como numa freada que ja parte do trote.
        const bool running = m_brakeGaitActive && !m_brakeStepping;
        m_brakeRunShare = running
            ? std::min(1.0f, m_brakeRunShare + deltaTime / 0.08f)
            : std::max(0.0f, m_brakeRunShare - deltaTime / BrakeRunFadeSeconds);
        m_telemetry.brakeRunShare = m_brakeRunShare;
        m_telemetry.brakePhase = !m_brakeGaitActive ? 0 : m_brakeStepping ? 2 : 1;
        // (Terminada a freada, a navegacao volta direto a capsula: o resto do
        // perfil fazia o estado virar "andando" por um instante, o clipe de
        // trote pegava os pes de volta e desmontava a base ja parado.)
        if (m_brakeGait > 0.0f && m_brakeGaitActive) {
            brakeNavigationVelocity = m_brakeGaitDirection * m_brakeGaitSpeed;
            desiredPlanarVelocity += (brakeNavigationVelocity - desiredPlanarVelocity)
                * m_brakeGait;
        } else if (m_brakeGait > 0.0f) {
            brakeNavigationVelocity = desiredPlanarVelocity;
        }
        m_telemetry.brakeRunVelocityWorld = brakeNavigationVelocity;
    }
    const Vec3 rawAcceleration = clampMagnitude(
        (desiredPlanarVelocity - m_previousDesiredVelocity) / deltaTime,
        24.0f);
    m_previousDesiredVelocity = desiredPlanarVelocity;
    const float accelerationBlend = 1.0f - std::exp(-9.0f * deltaTime);
    m_smoothedAcceleration +=
        (rawAcceleration - m_smoothedAcceleration) * accelerationBlend;

    // Sensor de equilibrio para os pes reagirem a empurrao, pressao e
    // tranco: o ponto de captura (centro de massa + velocidade/omega, onde o
    // corpo para se pisar ali) contra o apoio real (as solas no chao) e a
    // forca externa dos contatos. Parado, os passos de recuperacao (bloco
    // dos pes, mais abaixo) miram esse ponto; andando, o pouso do clipe e
    // deslocado para ele. Antes, parado, so havia o passo de acomodacao (o
    // pe 12 cm longe da pose, a pose presa a capsula que vem atras do
    // corpo): empurrado pelas costas a 0,8 m/s, os pes ficavam 0,8 s
    // travados e ele caia de frente. A passada dos clipes nao serve para
    // isso: todos sao de corrida (trote, recuo, strafe), com os dois pes no
    // ar a cada passada.
    float captureOutside = 0.0f;
    if (input.forceDrivenRoot && physical.valid) {
        // Apoio, ponto de captura e forca externa vem do estimador comum. Aqui
        // so a decisao de planejamento: com uma forca externa sustentada F o
        // ponto de equilibrio anda F/(m omega^2) = F h/(m g) no rumo dela
        // (150 N no peito: ~20 cm) - o passo sai antes e pousa onde segura.
        std::vector<std::array<float, 2>> support;
        float sumX = 0.0f, sumY = 0.0f;
        for (std::uint32_t i = 0; i < physical.supportPointCount; ++i) {
            const Vec3 p = physical.supportPointsWorld[i];
            support.push_back({ p.x, p.y });
            sumX += p.x;
            sumY += p.y;
        }
        Vec3 captureDirection {};
        if (!support.empty()) {
            const float omega = physical.captureOmega;
            const Vec3 forceShift = clampMagnitude(physical.externalForceWorld
                * (1.0f / (std::max(1.0f, profile.totalMassKg) * omega * omega)), 0.30f);
            // O planejamento usa a velocidade do centro de massa filtrada
            // (~60 ms): a crua do estimador reage ao tremor de cada tick e
            // disparava passos que tiravam a base (7 quedas na varredura de
            // 72 empurroes, contra 3).
            const float captureX = physical.centerOfMassWorld.x
                + m_filteredCenterOfMassVelocity.x / omega + forceShift.x;
            const float captureY = physical.centerOfMassWorld.y
                + m_filteredCenterOfMassVelocity.y / omega + forceShift.y;
            captureOutside = planarDistanceOutsideHull3D(support, captureX, captureY);
            const float count = static_cast<float>(support.size());
            captureDirection = { captureX - sumX / count, captureY - sumY / count, 0.0f };
            if (captureDirection.lengthSquared() > 1e-6f)
                captureDirection = captureDirection.normalized();
            m_capturePoint = { captureX, captureY, physical.supportHeight };
        }
        m_captureOutside = captureOutside;
        m_captureDirection = captureDirection;
    }
    m_telemetry.externalForceWorld = physical.externalForceWorld;
    m_telemetry.captureOutsideMeters = captureOutside;

    const float speed = std::hypot(desiredPlanarVelocity.x,
        desiredPlanarVelocity.y);
    const bool moving = speed > 0.08f;
    // Tropeco de marcha: um pe que deveria estar no meio do balanco toca sem
    // sustentar carga. Nao confundir com o toque normal (ultimos 25% do
    // passo) nem com o pe de apoio. A fase e a do tick anterior, suficiente
    // para classificar cedo/meio contra tarde sem antecipar o solver.
    bool trippedThisTick = false;
    if (TripDetection && input.forceDrivenRoot && moving && input.grounded && !getUpActive
        && speed > 1.5f) {
        for (std::size_t side = 0; side < m_feet.size(); ++side) {
            const float phase = m_telemetry.footSwingProgress[side];
            const FootSupportEstimate3D& seen = physical.feet[side];
            const float soleSpeed = std::hypot(seen.soleVelocityWorld.x,
                seen.soleVelocityWorld.y);
            if (!m_feet[side].planted && phase >= 0.12f && phase < 0.75f
                && seen.touching && !seen.supporting && soleSpeed > 0.55f) {
                m_tripRecovery = 1.0f;
                m_tripRecoverySeconds = 0.0f;
                m_tripFoot = static_cast<int>(side);
                m_tripElevating = phase < 0.58f;
                m_tripDirection = desiredPlanarVelocity / speed;
                m_fallDirection = m_tripDirection;
                trippedThisTick = true;
                break;
            }
        }
    }
    if (!trippedThisTick) {
        m_tripRecoverySeconds += deltaTime;
        // A decisao dura aproximadamente uma passada; a intensidade cai
        // devagar o bastante para o pe seguinte terminar a recuperacao.
        m_tripRecovery = std::max(0.0f,
            m_tripRecovery - deltaTime / 0.48f);
        if (m_tripRecovery <= 0.001f) m_tripFoot = -1;
    }
    // Quase-queda em movimento: sem esperar o pe raspar no meio do balanco.
    // O corpo girando para a frente alem da corrida (a pelve inclinando mais
    // que a de regime e ainda subindo) ou freado de repente alem do que a
    // passada pede (o pe bateu em algo) entra na mesma recuperacao do
    // tropeco: pelve baixa, passada longa e rapida no rumo, bracos abertos.
    // Medido no tropeco num obstaculo a 3 m/s: o pe pegou aos 2,62 s, a pelve
    // foi de 6 a 45 graus em 0,33 s, e o tropeco so era visto aos 2,98 s -
    // com os dois pes 0,7 m atras do corpo, tarde demais.
    m_stumbleRisk = 0.0f;
    if (StumbleDetection && input.forceDrivenRoot && moving && input.grounded && !getUpActive
        && !state.links.empty() && m_brakeGait <= 0.0f) {
        const Vec3 bodyPlanar { m_filteredCenterOfMassVelocity.x, m_filteredCenterOfMassVelocity.y, 0.0f };
        const float bodySpeed = bodyPlanar.length();
        const Vec3 travel = bodySpeed > 0.8f ? bodyPlanar * (1.0f / bodySpeed)
            : speed > 0.1f ? desiredPlanarVelocity * (1.0f / speed) : Vec3 {};
        if (travel.lengthSquared() > 0.5f) {
            const Vec3 upModel = profile.links.front().modelOrientation.conjugate().rotate({ 0.0f, 0.0f, 1.0f });
            const Vec3 up = state.links.front().orientation.rotate(upModel);
            // Contra a pelve de referencia do tick anterior: a inclinacao que a
            // passada pede (a arrancada inclina de proposito) nao e susto.
            const Vec3 referenceUp = m_output.guide.rootOrientationWorld.rotate(upModel);
            const float pitch = std::asin(std::clamp(dot(up, travel), -1.0f, 1.0f))
                - (StumbleAgainstReference ? std::asin(std::clamp(dot(referenceUp, travel), -1.0f, 1.0f)) : 0.0f);
            const float along = dot(bodyPlanar, travel);
            const float rawRate = m_stumbleValid ? (pitch - m_stumblePitch) / deltaTime : 0.0f;
            // Freada do corpo alem da da referencia (a capsula freando numa
            // curva ou parada nao conta).
            const float rawDecel = m_stumbleValid
                ? (m_stumbleAlong - along) / deltaTime + dot(m_smoothedAcceleration, travel) : 0.0f;
            if (!m_stumbleValid) m_stumbleBaseline = pitch;
            m_stumblePitch = pitch;
            m_stumbleAlong = along;
            m_stumblePitchRate += (rawRate - m_stumblePitchRate) * (1.0f - std::exp(-deltaTime / 0.03f));
            m_stumbleDecel += (rawDecel - m_stumbleDecel) * (1.0f - std::exp(-deltaTime / 0.06f));
            m_stumbleValid = true;
            const float excess = pitch - m_stumbleBaseline;
            // (So ja correndo: na arrancada a pelve inclina mais que a de
            // referencia enquanto o corpo ganha velocidade.)
            // Nem acelerando: arrancando, a pelve inclina ~18 graus alem da de
            // referencia enquanto a capsula acelera a 14 m/s2.
            const float running = smoothStep((bodySpeed - 1.3f) / 0.7f)
                * (1.0f - smoothStep((dot(m_smoothedAcceleration, travel) - 3.0f) / 3.0f));
            const float pitchRisk = smoothStep((excess - StumblePitchStart) / StumblePitchSpan)
                * smoothStep((m_stumblePitchRate - StumbleRateStart) / StumbleRateSpan) * running;
            const float deepRisk = smoothStep((excess - StumbleDeepStart) / 0.15f) * running;
            const Vec3 requested { input.intent.requestedVelocityWorld.x, input.intent.requestedVelocityWorld.y, 0.0f };
            const float wantsOn = smoothStep((dot(requested, travel) - 0.5f * bodySpeed) / 0.5f);
            const float decelRisk = smoothStep((m_stumbleDecel - StumbleDecelStart) / StumbleDecelSpan)
                * smoothStep((bodySpeed - 1.5f) / 1.0f) * wantsOn;
            m_stumbleRisk = std::max(std::max(pitchRisk, deepRisk), decelRisk);
            // A de regime acompanha a corrida devagar (~1 s), sempre: o susto e
            // rapido (0,2-0,3 s) e quase nao entra nela. (So fora do susto ela
            // congelava na arrancada e o alarme se realimentava.)
            m_stumbleBaseline += (pitch - m_stumbleBaseline)
                * (1.0f - std::exp(-deltaTime / std::max(0.05f, StumbleBaselineSeconds)));
            if (m_stumbleRisk > m_stumbleReaction) m_stumbleDirection = travel;
            // A recuperacao completa do tropeco (pelve baixa, pouso adiantado,
            // tronco) so com StumbleFullResponse: medido no pe enganchado em
            // sprint, ela derrubava mais (26 -> 36-50 em 128).
            if (StumbleFullResponse && m_stumbleRisk > m_tripRecovery) {
                m_tripRecovery = m_stumbleRisk;
                m_tripRecoverySeconds = 0.0f;
                m_tripDirection = travel;
            }
        }
    } else {
        m_stumbleValid = false;
        m_stumblePitchRate = 0.0f;
        m_stumbleDecel = 0.0f;
    }
    m_stumbleReaction = std::max(m_stumbleRisk, m_stumbleReaction - deltaTime / 0.5f);
    const float viewYaw = input.facingYawRadians;
    // O angulo da camera chega em degraus: muda uma vez por quadro da tela, e
    // a fisica roda a 120 Hz. Para o olhar (cabeca, tronco, pelve girando no
    // lugar), uma mola criticamente amortecida o alisa - velocidade continua,
    // ~65 ms de atraso. Com o filtro de primeira ordem de antes, a velocidade
    // pedida a cabeca e ao tronco virava um dente de serra a cada quadro. A
    // escolha do setor do movimento continua com o angulo cru.
    if (!m_lookValid) {
        m_lookYaw = viewYaw;
        m_lookYawVelocity = 0.0f;
        m_lookPitch = std::clamp(input.lookPitchRadians, -0.60f, 0.50f);
        m_lookPitchVelocity = 0.0f;
        m_lookValid = true;
    }
    constexpr float LookOmega = 30.0f;
    m_lookYawVelocity += (LookOmega * LookOmega
            * wrapAngle(viewYaw - m_lookYaw)
        - 2.0f * LookOmega * m_lookYawVelocity) * deltaTime;
    m_lookYaw = wrapAngle(m_lookYaw + m_lookYawVelocity * deltaTime);
    m_lookPitchVelocity += (LookOmega * LookOmega
            * (std::clamp(input.lookPitchRadians, -0.60f, 0.50f) - m_lookPitch)
        - 2.0f * LookOmega * m_lookPitchVelocity) * deltaTime;
    m_lookPitch += m_lookPitchVelocity * deltaTime;
    const float lookYaw = m_lookYaw;
    // While fallen/getting up, pelvis yaw is meaningless — orientation is
    // physics', not this integrator's — so none of it runs, and the turn
    // step accumulator (a stationary-only concept) is kept at zero instead
    // of quietly building up a step to fire the instant recovery finishes.
    bool turningInPlace = false;
    if (!moving || getUpActive) {
        m_gaitDirection = CharacterGaitDirection3D::Forward;
    }
    // Taxa de giro do caminho (rad/s): a velocidade de navegacao girando
    // pela aceleracao filtrada (lateral / velocidade).
    const float pathSpeed = std::hypot(desiredPlanarVelocity.x, desiredPlanarVelocity.y);
    // So com o jogador pedindo movimento: parando (sem pedido), a
    // desaceleracao da navegacao um pouco desalinhada parecia curva e ele
    // inclinava e virava a pelve na parada. Freando pela passada, tambem nao
    // (o perfil de freada nao e curva).
    const float curveIntent = smoothStep(std::hypot(input.intent.requestedDirectionWorld.x,
        input.intent.requestedDirectionWorld.y) / 0.5f) * (1.0f - m_brakeGait);
    const float pathTurnRate = pathSpeed > 0.8f
        ? (desiredPlanarVelocity.x * m_smoothedAcceleration.y
            - desiredPlanarVelocity.y * m_smoothedAcceleration.x) / (pathSpeed * pathSpeed) * curveIntent
        : 0.0f;
    bool pivoting = false;
    if (!getUpActive) {
        const float viewErrorBeforeTurn = wrapAngle(lookYaw - m_facingYaw);
        float targetPelvisYaw = m_facingYaw;
        if (moving) {
            // Olhar x movimento. O personagem olha para onde a camera aponta
            // (viewYaw); o movimento cai num setor e o setor escolhe o ciclo:
            // para a frente corre, de lado faz o strafe (de frente para a
            // camera), para tras recua - nunca da meia-volta. Dentro do setor
            // a pelve gira para o movimento; o tronco e a cabeca devolvem o
            // giro para a camera (bloco do olhar, mais abaixo).
            // A direcao pedida (as teclas), quando ha: a da capsula gira aos
            // poucos e a pelve passava pela diagonal ao trocar de "frente"
            // para "lado" antes de assentar no strafe.
            // Freando pela passada, o rumo e o do corpo: o pedido (nenhum, ou
            // o contrario) giraria a pelve no meio da freada.
            // Invertendo, abaixo da velocidade de pivo a pelve ja gira para o
            // rumo novo (na referencia Running Turn 180 ela gira desde a
            // freada: 37 graus ate 1,2 m/s, 100 no meio da troca).
            pivoting = m_brakeReversing && m_brakeGaitActive
                && std::hypot(m_filteredCenterOfMassVelocity.x, m_filteredCenterOfMassVelocity.y) < PivotSpeed;
            // (So enquanto a freada dura: no fim dela, com a mistura ainda
            // acima de meio, o rumo voltava a ser o da capsula - ainda o da
            // corrida - e o setor piscava frente/costas/frente numa inversao,
            // cada troca recomecando o clipe e soltando o pe que freava.)
            const bool brakeTravel = m_brakeGait > 0.5f && !pivoting
                && (!SectorHoldWhileBraking || m_brakeGaitActive);
            const Vec3 intended = brakeTravel ? Vec3 {}
                : Vec3 { input.intent.requestedDirectionWorld.x,
                    input.intent.requestedDirectionWorld.y, 0.0f };
            const float travelYaw = intended.lengthSquared() > 0.01f
                ? std::atan2(intended.y, intended.x)
                : std::atan2(desiredPlanarVelocity.y, desiredPlanarVelocity.x);
            const float travelFromView = wrapAngle(travelYaw - viewYaw);
            const bool strafe = animations.strafeLeft != nullptr
                && animations.strafeRight != nullptr;
            const float hysteresis = strafe ? StrafeSectorHysteresisRadians
                : PlainSectorHysteresisRadians;
            const auto available = [&](CharacterGaitDirection3D direction) {
                switch (direction) {
                case CharacterGaitDirection3D::Left:
                case CharacterGaitDirection3D::Right: return strafe;
                case CharacterGaitDirection3D::Backward:
                    return animations.walkBackward != nullptr;
                default: return true;
                }
            };
            const auto overrun = [&](CharacterGaitDirection3D direction) {
                return gaitSectorOverrun(direction, travelFromView, strafe);
            };
            // Freando forte (o jogador pediu o sentido oposto), o corpo freia
            // de frente para onde vinha e so troca de setor devagar; e, rapido,
            // nunca troca para o setor oposto (meia-volta da pelve): com a
            // camera virada 180 graus no sprint, a velocidade ainda para a
            // frente caia no setor "de costas" e a pelve girava na hora - ele
            // "corria de costas" a 7 m/s e caia. Ele freia de frente e vira
            // quando estiver devagar. So no modo fisico (no antigo a pose nao
            // carrega o corpo).
            const bool hardBraking = !pivoting && input.forceDrivenRoot && speed > 1.8f
                && dot(m_smoothedAcceleration, desiredPlanarVelocity) < -3.0f * speed;
            // Freando, o setor fica o da corrida ate a freada acabar: o corpo
            // freia de frente para onde vinha (entrando na freada, a mistura
            // ainda abaixo de meio pegava o rumo pedido e trocava o ciclo).
            const bool brakeHold = SectorHoldWhileBraking && m_brakeGaitActive && !pivoting
                && available(m_gaitDirection);
            if (!hardBraking && !brakeHold && (!available(m_gaitDirection)
                || overrun(m_gaitDirection) > hysteresis)) {
                // O setor que contem a direcao (sem recuo, o mais proximo).
                float best = std::numeric_limits<float>::infinity();
                CharacterGaitDirection3D chosen = m_gaitDirection;
                for (const CharacterGaitDirection3D candidate : {
                        CharacterGaitDirection3D::Forward,
                        CharacterGaitDirection3D::Left,
                        CharacterGaitDirection3D::Right,
                        CharacterGaitDirection3D::Backward }) {
                    if (available(candidate) && overrun(candidate) < best) {
                        best = overrun(candidate);
                        chosen = candidate;
                    }
                }
                const bool halfTurn = std::abs(wrapAngle(gaitSectorCenter(chosen)
                    - gaitSectorCenter(m_gaitDirection))) > 2.0f;
                if (!(input.forceDrivenRoot && halfTurn && speed > 2.5f
                        && available(m_gaitDirection))) {
                    m_gaitDirection = chosen;
                }
            }
            // O ciclo do setor anda na direcao do centro dele em relacao a
            // pelve: a pelve fica no movimento menos esse centro.
            targetPelvisYaw = wrapAngle(travelYaw
                - gaitSectorCenter(m_gaitDirection));
            // Correndo para a frente, a pelve vai no caminho do corpo: no
            // maximo um adiantamento para o pedido, menor quanto mais rapido.
            // Seguindo o pedido (a camera girando a 180-300 graus/s no
            // sprint), ela ficava 80-90 graus a frente do rumo real, o ciclo
            // de sprint punha os pes de lado para o movimento e ele tombava
            // para fora da curva; e o pe de apoio girava junto com ela no chao.
            const auto boundToPath = [&](float yaw) {
                const float bodySpeed = std::hypot(m_filteredCenterOfMassVelocity.x,
                    m_filteredCenterOfMassVelocity.y);
                const float share = smoothStep((bodySpeed - 1.0f) / 1.0f);
                if (share <= 0.0f) return yaw;
                const float pathYaw = std::atan2(m_filteredCenterOfMassVelocity.y,
                    m_filteredCenterOfMassVelocity.x);
                const float lead = PathLeadSlow + (PathLeadFast - PathLeadSlow)
                    * smoothStep((bodySpeed - 2.0f) / std::max(0.5f, PathLeadFastSpeed - 2.0f));
                const float offset = wrapAngle(yaw - pathYaw);
                const float bounded = std::clamp(offset, -lead, lead);
                return wrapAngle(yaw + (bounded - offset) * share);
            };
            const bool pathBound = PathLeadSlow < 3.0f && input.forceDrivenRoot && !pivoting
                && m_gaitDirection == CharacterGaitDirection3D::Forward;
            if (pathBound && !PathLeadAfterCurve) {
                targetPelvisYaw = boundToPath(targetPelvisYaw);
            }
            // Correndo em curva a pelve vai a frente do rumo, para dentro
            // (na referencia Sprint Turn, ~13 graus a 43 graus/s): o quadril
            // ja aponta para onde o caminho vai.
            if (m_gaitDirection == CharacterGaitDirection3D::Forward)
                targetPelvisYaw = wrapAngle(targetPelvisYaw + std::clamp(PelvisCurveLeadSeconds * pathTurnRate,
                    -PelvisCurveLeadMaximum, PelvisCurveLeadMaximum) * smoothStep((speed - 1.5f) / 1.5f));
            if (pathBound && PathLeadAfterCurve) targetPelvisYaw = boundToPath(targetPelvisYaw);
        } else if (std::abs(viewErrorBeforeTurn) > 0.55f
            || (std::abs(m_turningRate) > 0.05f
                && std::abs(viewErrorBeforeTurn) > 0.32f)) {
            // Olhar/coluna conservam cerca de 18 graus. O restante é
            // resolvido pela base com um passo de rotação, sem colar a
            // pelve na câmera. Depois de comecar, o giro vai ate os 18
            // graus: parar ao entrar nos 31 (a regra de antes) deixava o
            // alvo do giro mudando no meio dos passos.
            targetPelvisYaw = lookYaw - std::copysign(0.31f,
                viewErrorBeforeTurn);
        }
        float pelvisYawError = wrapAngle(targetPelvisYaw - m_facingYaw);
        // Giro parado pelo clipe (ajuda cheia; pedido do usuario, 04/10): no
        // lugar dos passos de giro do planejador, o Left/Right Turn tocado na
        // velocidade do giro pedido - a pelve segue o giro do clipe (escalado
        // ao que falta, ate ~86 graus por ciclo; mais que isso, outro ciclo) e
        // as pernas e os pes sao os do clipe.
        bool clipTurnControl = false;
        {
            // (O giro pelo clipe so existia com a ajuda cheia, removida em
            // 04/10: o modo fisico gira pelos passos do planejador.)
            const bool allowed = false;
            if (!allowed) m_clipTurn.active = false;
            const auto begin = [&](float error) {
                m_clipTurn.active = true;
                m_clipTurn.sign = error >= 0.0f ? 1.0f : -1.0f;
                m_clipTurn.clip = error >= 0.0f ? animations.turnLeft : animations.turnRight;
                m_clipTurn.span = std::max(0.3f, std::abs(clipRootYaw(*m_clipTurn.clip, profile,
                    m_clipTurn.clip->durationSeconds)));
                m_clipTurn.total = std::min(std::abs(error), m_clipTurn.span);
                m_clipTurn.time = 0.0f;
                m_clipTurn.startYaw = m_facingYaw;
                m_clipTurn.startPelvisYaw = m_facingYaw + m_clipPelvisYaw;
            };
            if (allowed && !m_clipTurn.active && std::abs(pelvisYawError) > ClipTurnEnter)
                begin(pelvisYawError);
            if (m_clipTurn.active) {
                clipTurnControl = true;
                const AnimationClip3D& clip = *m_clipTurn.clip;
                const float duration = std::max(0.05f, clip.durationSeconds);
                // A camera seguiu girando no comeco do ciclo: ele gira mais
                // (nunca menos, nem para o outro lado).
                if (m_clipTurn.time < 0.5f * duration) {
                    const float wanted = wrapAngle(targetPelvisYaw - m_clipTurn.startYaw) * m_clipTurn.sign;
                    m_clipTurn.total = std::clamp(wanted, m_clipTurn.total, m_clipTurn.span);
                }
                const float cycleSeconds = std::clamp(m_clipTurn.total / std::max(0.5f, ClipTurnSpeed),
                    ClipTurnMinSeconds, duration);
                m_clipTurn.time = std::min(duration, m_clipTurn.time + deltaTime * duration / cycleSeconds);
                const float progress = std::clamp(std::abs(clipRootYaw(clip, profile, m_clipTurn.time))
                    / m_clipTurn.span, 0.0f, 1.0f);
                const float yaw = wrapAngle(m_clipTurn.startYaw + m_clipTurn.sign * m_clipTurn.total * progress);
                m_turningRate = wrapAngle(yaw - m_facingYaw) / deltaTime;
                m_facingYaw = yaw;
                // O deslocamento da raiz do clipe (no rumo da pelve no comeco),
                // na mesma escala do giro.
                const Vec3 planarOffset = clipRootPlanar(clip, profile, m_clipTurn.time) * (m_clipTurn.total / m_clipTurn.span);
                const float c = std::cos(m_clipTurn.startPelvisYaw), sn = std::sin(m_clipTurn.startPelvisYaw);
                m_clipTurnRootOffset = { planarOffset.x * c - planarOffset.y * sn, planarOffset.x * sn + planarOffset.y * c, 0.0f };
                if (m_clipTurn.time >= duration - 1e-4f) {
                    m_clipTurn.active = false;
                    m_clipTurnRootOffset = {};
                    // Falta giro: o proximo ciclo (a base no fim e a do comeco,
                    // girada).
                    const float rest = wrapAngle(targetPelvisYaw - m_facingYaw);
                    if (allowed && std::abs(rest) > ClipTurnEnter) begin(rest);
                }
                pelvisYawError = wrapAngle(targetPelvisYaw - m_facingYaw);
            } else {
                m_clipTurnRootOffset = {};
            }
            const float wantedWeight = m_clipTurn.active ? 1.0f : 0.0f;
            m_clipTurnWeight = wantedWeight > m_clipTurnWeight
                ? std::min(1.0f, m_clipTurnWeight + deltaTime / std::max(0.01f, ClipTurnBlendIn))
                : std::max(0.0f, m_clipTurnWeight - deltaTime / std::max(0.01f, ClipTurnBlendOut));
        }
        if (!clipTurnControl) {
        // Virada (pivo): o lado e escolhido uma vez - o do pe apoiado (na
        // referencia o direito planta e ele gira para a direita) - e mantido:
        // a 180 graus o erro alternava entre +180 e -180 e o giro trocava de
        // sentido sem sair do lugar.
        if (!pivoting) {
            m_pivotSign = 0.0f;
        } else {
            if (m_pivotSign == 0.0f) {
                const auto& feetNow = physical.feet;
                m_pivotSign = std::abs(pelvisYawError) < 2.6f ? (pelvisYawError >= 0.0f ? 1.0f : -1.0f)
                    : feetNow[0].supporting != feetNow[1].supporting ? (feetNow[0].supporting ? 1.0f : -1.0f)
                    : m_turningRate >= 0.0f ? 1.0f : -1.0f;
            }
            if (pelvisYawError * m_pivotSign < 0.0f && std::abs(pelvisYawError) > 2.0f)
                pelvisYawError += m_pivotSign * 2.0f * Pi;
        }
        // Recem-levantado, o corpo primeiro firma a base: parado, nao gira
        // para a camera no primeiro meio segundo, e o giro entra aos poucos
        // ate ~1,5 s. Girando 100 graus logo na troca (taxa cheia), com um pe
        // ainda solto, ele inclinava 20 graus e caia de novo.
        // Recem-freado tambem: primeiro assenta a base da freada.
        // (So parado: pedindo movimento, logo depois de uma inversao pela
        // freada, o giro a 35% deixava a pelve para tras e ele saia correndo
        // de costas. Sem pedido, terminando a parada, ele ainda conta como
        // andando por um instante - ali o giro fica contido.)
        const bool requestingMove = std::hypot(input.intent.requestedDirectionWorld.x,
            input.intent.requestedDirectionWorld.y) > 0.1f;
        const float footing = smoothStep((m_sinceGetUpSeconds - 0.5f) / 1.0f)
            * (moving && requestingMove && BrakeFootingStandingOnly ? 1.0f
                : smoothStep((m_sinceBrakeSeconds - 0.3f) / 0.6f));
        // Parado, o giro e rapido (5 rad/s, 30 rad/s^2): a 2,35 rad/s e 10
        // rad/s^2 a pelve fazia ~25 graus no primeiro meio segundo, e o giro
        // parado parecia preso. Quem limita o que passa do apoio e o limite de
        // torcao do quadril (abaixo) e os passos de giro.
        // Pivo ainda rapido: o giro vai entrando conforme o corpo freia.
        const float pivotRate = pivoting ? 0.3f + 0.7f * smoothStep((PivotFullRateBelow
            - std::hypot(m_filteredCenterOfMassVelocity.x, m_filteredCenterOfMassVelocity.y)) / 3.0f) : 1.0f;
        // Com a ajuda parado alta, o giro parado e quase imediato: a pelve tem
        // forca para girar e os pes acompanham pivotando (abaixo).
        const float turnAssist = std::clamp(input.standingAssist, 0.0f, 1.0f);
        const float standingTurnRate = 5.0f + (StandingTurnRateHigh - 5.0f) * turnAssist;
        // Andando com pouca ajuda, a pelve gira no ritmo em que os pes
        // conseguem curvar o corpo: a aceleracao centripeta que o chao da
        // (~atrito x g) sobre a velocidade - a 5,5 m/s, ~1 rad/s, contra os 4,8
        // do pedido. Mais rapido que isso o corpo nao acompanhava, a pelve
        // passava da referencia e ele caia (zigue-zague a 30%).
        const float movingTurnAssist = smoothStep((0.6f - std::clamp(input.walkingAssist, 0.0f, 1.0f)) / 0.5f);
        const float bodyPlanarSpeed = std::hypot(m_filteredCenterOfMassVelocity.x, m_filteredCenterOfMassVelocity.y);
        const float physicalTurnRate = std::max(LowAssistTurnMinimum,
            LowAssistTurnCentripetal / std::max(0.5f, bodyPlanarSpeed));
        const float movingTurnRate = (input.sprinting ? 4.8f : 3.8f);
        const float maximumTurnRate = (moving
                ? movingTurnRate + (std::min(movingTurnRate, physicalTurnRate) - movingTurnRate) * movingTurnAssist
                : standingTurnRate)
            * (moving ? 0.35f + 0.65f * footing : footing) * pivotRate;
        // Correndo para a frente, a pelve gira junto com o caminho (a taxa
        // dele entra direto) e o ganho so corrige o resto: so pelo ganho ela
        // ficava atrasada taxa/7 numa curva (16 graus a 90 graus/s, apontando
        // para fora) e os pes pousavam virados para fora. (Nos outros setores
        // nao: trocando da frente para o strafe o caminho gira mas a pelve
        // fica, e ela ia para a diagonal.)
        const float wantedTurnRate = std::clamp(
            pelvisYawError * (moving ? 7.0f : 10.0f + (StandingTurnGainHigh - 10.0f) * turnAssist)
                + (moving && m_gaitDirection == CharacterGaitDirection3D::Forward
                    ? PelvisCurveFeedForward * pathTurnRate : 0.0f),
            -maximumTurnRate, maximumTurnRate);
        const float maximumTurnAcceleration = moving ? 18.0f
            : 30.0f + (StandingTurnAccelerationHigh - 30.0f) * turnAssist;
        m_turningRate += std::clamp(wantedTurnRate - m_turningRate,
            -maximumTurnAcceleration * deltaTime,
            maximumTurnAcceleration * deltaTime);
        float yawStep = m_turningRate * deltaTime;
        if (std::abs(yawStep) > std::abs(pelvisYawError)
            && yawStep * pelvisYawError > 0.0f) {
            yawStep = pelvisYawError;
            m_turningRate = 0.0f;
        }
        m_facingYaw = wrapAngle(m_facingYaw + yawStep);
        // Parado, a pelve nao gira alem do que o quadril gira sobre um pe
        // apoiado (45 graus; 40 com folga): o giro vem dos pes. Sem isso a
        // pelve seguia a camera a ate 135 graus/s, e o pe apoiado ia junto,
        // esfregando no chao, ou a perna torcia no limite. So segura o que
        // passaria do limite neste tick; nunca empurra a pelve de volta.
        if (!moving && std::abs(yawStep) > 0.0f) {
            // A base em alerta ja traz cerca de 18 graus de rotacao entre
            // pelve e pes. Este limite e para o giro ADICIONAL que ainda
            // cabe sobre o apoio; 28 + 18 mantem a perna abaixo do gate
            // anatomico de 50 graus enquanto o outro pe contorna a base.
            const float HipTwistLimit = 0.48f + (StandingHipTwistHigh - 0.48f) * turnAssist;
            for (std::size_t side = 0; side < m_feet.size(); ++side) {
                const FootPlant& foot = m_feet[side];
                if (!foot.valid || !foot.planted || foot.repositioning) {
                    continue;
                }
                const Vec3 forward = foot.lockedOrientationWorld.rotate(
                    profile.links[foot.linkIndex].modelOrientation
                        .conjugate().rotate({ 1.0f, 0.0f, 0.0f }));
                const float twist = wrapAngle(m_facingYaw + m_clipPelvisYaw
                    - std::atan2(forward.y, forward.x)
                    - m_clipPelvisToFootYaw[side]);
                if (std::abs(twist) > HipTwistLimit && twist * yawStep > 0.0f) {
                    const float excess = std::min(std::abs(yawStep),
                        std::abs(twist) - HipTwistLimit);
                    // Com a ajuda parado alta o pe gira junto, pivotando no
                    // chao: o giro nao espera os passos.
                    const float pivot = excess * std::clamp(StandingFootPivot * turnAssist, 0.0f, 1.0f);
                    if (pivot > 0.0f && m_footwork.owns(side))
                        m_footwork.pivotAnchor(profile, side, std::copysign(pivot, twist));
                    m_facingYaw = wrapAngle(m_facingYaw
                        - std::copysign(excess - pivot, twist));
                    // O pe bloqueou o deslocamento deste tick, mas o giro
                    // continua solicitado. Preservar a taxa mantem a
                    // histerese de giro ativa e permite que o planejador
                    // solte o proximo passo; zerar aqui abandonava uma
                    // camera lenta ainda com ~30 graus por resolver.
                }
            }
        }
        }
        if (std::abs(pelvisYawError) < 0.001f
            && std::abs(m_turningRate) < 0.02f && !clipTurnControl) {
            m_turningRate = 0.0f;
        }
        // Com histerese: sem ela, girando devagar o estado alternava entre
        // parado e girando no lugar a cada tick.
        turningInPlace = !moving && std::abs(m_turningRate)
            > (m_telemetry.state == CharacterLocomotionState3D::Turning
                ? 0.03f : 0.10f);
        m_standingTurnTarget = targetPelvisYaw;
    } else {
        m_turningRate = 0.0f;
        m_standingTurnTarget = m_facingYaw;
    }

    // Crouch and jump are deactivated: there is no authored clip for either
    // right now. Fallen/GettingUp preempt everything else — a real trip or
    // collision the reaction softening below could not absorb — so this
    // state machine reaches Idle, Turning, Walking, Running (a soccer
    // game's brisk jog and its sprint — see clip selection below),
    // Airborne, Landing, Fallen and GettingUp. Losing the ground (a fall, a
    // push, an edge) still needs a sane fallback pose regardless of
    // whether a jump was ever pressed, so Airborne/Landing stay
    // independent of the jump input itself.
    // Chao "de verdade" para os estados de voo. No modo fisico a verdade e o
    // corpo, nao a capsula: os dois decolam juntos, mas pousam em momentos
    // diferentes (medido: a capsula tocava o chao 0,28 s antes, e o corpo
    // chegava ao chao ainda com a pose de passada - pouso passivo). A
    // capsula no chao com o corpo ainda no ar segue em voo (pernas
    // procurando o chao); o corpo tocando o chao antes da capsula ja pousa.
    bool grounded = input.grounded;
    if (input.forceDrivenRoot && !getUpActive && !state.links.empty()) {
        const PhysicsBodyState3D& pelvis = state.links.front();
        float floor = std::numeric_limits<float>::infinity();
        for (const GroundProbeResult3D& probe : input.footGround) {
            if (probe.hasSurface) floor = std::min(floor, probe.pointWorld.z);
        }
        const bool feetDown = feetGroundContact[0] || feetGroundContact[1];
        const bool bodyHigh = !std::isfinite(floor) || pelvis.position.z - floor
            > profile.standingRootHeightMeters + 0.06f;
        const bool flying = m_telemetry.state == CharacterLocomotionState3D::Airborne;
        if (flying && grounded && !feetDown && bodyHigh) {
            grounded = false;
        } else if (!grounded && feetDown && pelvis.linearVelocity.z < 0.5f
            && (!flying || m_stateSeconds > 0.15f)) {
            // Pes no chao e o corpo sem subir: chao, em qualquer estado. Na
            // decolagem, o voo so comeca quando o corpo sobe de fato; no
            // pouso antes da capsula, ele segue pousando (antes voltava a
            // "voar" no tick seguinte, com a capsula ainda no ar).
            grounded = true;
        }
    }
    CharacterLocomotionState3D desiredState = CharacterLocomotionState3D::Idle;
    if (getUpActive) {
        desiredState = m_tackling ? CharacterLocomotionState3D::Tackling
            : m_getUpPhase == CharacterGetUpPhase3D::Settling
            ? CharacterLocomotionState3D::Fallen
            : CharacterLocomotionState3D::GettingUp;
    } else if (!grounded) {
        desiredState = CharacterLocomotionState3D::Airborne;
    } else if ((!m_previousGrounded
        || (m_telemetry.state == CharacterLocomotionState3D::Landing
            && m_stateSeconds < m_landingSettleSeconds))
        // Pousando correndo, a passada continua: o estado de pouso trava os
        // dois pes onde tocaram, e a 7 m/s o corpo passava 1,7 m a frente
        // deles (35-50 graus de inclinacao). O amortecimento vale do mesmo
        // jeito (a pelve cede depois de qualquer toque, mais abaixo).
        && !(input.forceDrivenRoot && moving && speed > 2.5f)) {
        desiredState = CharacterLocomotionState3D::Landing;
    } else if (m_brakeStepping) {
        // Freando pelos passos: a passada de corrida para; os pes sao do
        // planejador (passos de freada) e o corpo fica na pose parada com a
        // reacao de freada por cima.
        desiredState = CharacterLocomotionState3D::Idle;
    } else if (moving) {
        // Sprint para a frente e de lado; recuando, so com o sprint de
        // costas (sem ele, segurar sprint nao corre). Correndo e freando, o
        // ciclo e o da velocidade do corpo (o sprint ate ~4,6 m/s).
        const bool sprintPace = m_brakeGaitActive
            ? m_brakeGaitSpeed > BrakeSprintClipSpeed : input.sprinting;
        desiredState = sprintPace
                && (m_gaitDirection != CharacterGaitDirection3D::Backward
                    || animations.sprintBackward != nullptr)
            ? CharacterLocomotionState3D::Running
            : CharacterLocomotionState3D::Walking;
    } else if (turningInPlace) {
        desiredState = CharacterLocomotionState3D::Turning;
    }
    if (desiredState == CharacterLocomotionState3D::Airborne
        && m_telemetry.state != CharacterLocomotionState3D::Airborne) {
        m_liftoffSpeed = speed;
        m_liftoffVerticalSpeed = input.rootVelocityWorld.z;
        // O pulo e escolhido na decolagem e vale ate o fim da aterrissagem:
        // parado se quase nao havia movimento, senao o do setor em que ele
        // corria (frente, costas, lados).
        const auto pick = [&](CharacterJumpKind3D kind,
            const AnimationClip3D* clip) {
            if (clip == nullptr) return false;
            m_jumpKind = kind;
            m_jumpClip = clip;
            return true;
        };
        m_jumpKind = CharacterJumpKind3D::None;
        m_jumpClip = nullptr;
        bool picked = false;
        if (speed > 1.0f) {
            switch (m_gaitDirection) {
            case CharacterGaitDirection3D::Backward:
                picked = pick(CharacterJumpKind3D::Backward,
                    animations.jumpBackward);
                break;
            case CharacterGaitDirection3D::Left:
                picked = pick(CharacterJumpKind3D::Left, animations.jumpLeft);
                break;
            case CharacterGaitDirection3D::Right:
                picked = pick(CharacterJumpKind3D::Right, animations.jumpRight);
                break;
            default:
                picked = pick(CharacterJumpKind3D::Forward,
                    animations.jumpForward);
                break;
            }
        }
        if (!picked) {
            pick(CharacterJumpKind3D::Standing, animations.jumpStanding);
        }
        // v2: correndo para a frente, o voo e a passada aberta do proprio
        // ciclo - do ponto da decolagem ao toque seguinte -, esticada no tempo
        // do pulo: a perna de tras sai estendida, a da frente chega para o
        // pouso e a corrida segue. Com o pulo para a frente (pernas juntas
        // e dobradas, bracos a frente) o pulo correndo parecia o de quem
        // pula parado e pousava nos dois pes.
        m_leapClip = nullptr;
        // (No sprint e com ajuda: no trote, pulos seguidos giravam a pelve
        // ate ~45 graus no ar; com pouca ajuda, pousando num pe so de um pulo
        // alto, a perna cedia - pulo em sprint e vira a 30%, 0 -> 11 em 12.
        // Com a ajuda cheia ela leva o voo a ~7,6 m/s e, pousando num pe so
        // com um corte pedido, ele tombava - sequencia mista, 0 -> 2 em 12.)
        if (RunLeap && input.physicalV2 && m_jumpKind == CharacterJumpKind3D::Forward
            && speed > RunLeapMinimumSpeed && input.walkingAssist >= RunLeapMinimumAssist
            && input.walkingAssist <= RunLeapMaximumAssist
            && (m_telemetry.state == CharacterLocomotionState3D::Running
                || m_telemetry.state == CharacterLocomotionState3D::Walking)
            && m_previousClip != nullptr && m_previousClip != animations.idle
            && m_previousClip->loops && m_previousClip->durationSeconds > 0.05f) {
            const ClipFootEvents& events = footEventsFor(profile, *m_previousClip);
            const float duration = m_previousClip->durationSeconds;
            const float from = stanceAlignedPhase(*m_previousClip, m_cyclePhase);
            float best = 2.0f;
            for (std::size_t side = 0; side < 2; ++side)
                for (const FootTouchdown& touchdown : events.touchdowns[side]) {
                    const float ahead = std::fmod(touchdown.touchdown / duration - from + 2.0f, 1.0f);
                    if (ahead > 0.03f && ahead < best) best = ahead;
                }
            if (best <= 1.0f) {
                m_leapClip = m_previousClip;
                m_leapFrom = from;
                m_leapSpan = best;
            }
        }
    }
    // Fase do voo pela velocidade vertical: 0 na decolagem, 0,5 no apice, 1
    // ao voltar a altura de onde saiu. Vale para qualquer altura de pulo. Sem
    // impulso para cima (caiu de um degrau), o voo ja comeca descendo.
    if (desiredState == CharacterLocomotionState3D::Airborne) {
        const float verticalSpeed = input.forceDrivenRoot && !state.links.empty()
            ? state.links.front().linearVelocity.z : input.rootVelocityWorld.z;
        if (input.forceDrivenRoot) {
            // A subida do corpo e a que conta (ele ainda ganha velocidade
            // depois que a capsula decolou).
            m_liftoffVerticalSpeed = std::max(m_liftoffVerticalSpeed, verticalSpeed);
        }
        m_flightPhase = m_liftoffVerticalSpeed > 1.0f
            ? std::clamp((m_liftoffVerticalSpeed - verticalSpeed)
                / (2.0f * m_liftoffVerticalSpeed), 0.0f, 1.0f)
            : 0.5f + 0.5f * std::clamp(-verticalSpeed / 4.0f, 0.0f, 1.0f);
    } else if (desiredState != CharacterLocomotionState3D::Landing) {
        m_jumpKind = CharacterJumpKind3D::None;
        m_jumpClip = nullptr;
        m_leapClip = nullptr;
    }
    // Toque no chao depois de um voo (com ou sem o estado de pouso).
    const bool enteringLanding = m_telemetry.state
            == CharacterLocomotionState3D::Airborne
        && desiredState != CharacterLocomotionState3D::Airborne
        && !getUpActive;
    // v2 (T_LandResync): pousando de um pulo direto na passada, o ciclo
    // recomeca no toque do pe que chegou primeiro - ele em apoio, o outro ja
    // no balanco para o passo seguinte. Sem isso o ciclo voltava na fase da
    // decolagem: o clipe pedia o outro pe no chao, os dois ficavam plantados
    // e o corpo, a 7 m/s, girava para a frente por cima deles.
    if (LandingResync && enteringLanding && input.forceDrivenRoot
        && (desiredState == CharacterLocomotionState3D::Walking || desiredState == CharacterLocomotionState3D::Running)) {
        int first = -1;
        if (feetGroundContact[0] != feetGroundContact[1]) first = feetGroundContact[0] ? 0 : 1;
        else if (m_feet[0].valid && m_feet[1].valid && !state.links.empty()) {
            // Os dois (ou nenhum): o mais a frente no rumo do corpo, ou o
            // mais baixo.
            const Vec3 velocity { m_filteredCenterOfMassVelocity.x, m_filteredCenterOfMassVelocity.y, 0.0f };
            const Vec3 a = state.links[m_feet[0].linkIndex].position, b = state.links[m_feet[1].linkIndex].position;
            first = feetGroundContact[0] && velocity.lengthSquared() > 0.25f
                ? (dot(a - b, velocity) > 0.0f ? 0 : 1) : (a.z < b.z ? 0 : 1);
        }
        m_landingResyncFoot = first;
    }
    const CharacterLocomotionState3D previousState = m_telemetry.state;
    enterState(desiredState);
    const bool enteredForwardGait = (previousState == CharacterLocomotionState3D::Idle
            || previousState == CharacterLocomotionState3D::Turning)
        && (desiredState == CharacterLocomotionState3D::Walking
            || desiredState == CharacterLocomotionState3D::Running)
        && m_gaitDirection == CharacterGaitDirection3D::Forward
        && animations.idleToSprint != nullptr;
    if (enteredForwardGait && StartStepEnabled) m_startStepRequested = true;
    // Ajuda dosada na arrancada parada: o teto comeca baixo e cresce sem
    // parar (sem janela: terminando de uma vez, o pedido saltava de ~3,5 para
    // 7,5 m/s no meio do sprint).
    // (Armado ja parado: o jogo le o teto do tick anterior, e armando so na
    // entrada da passada o primeiro tick saia com o pedido inteiro.)
    const bool idleLike = desiredState == CharacterLocomotionState3D::Idle
        || desiredState == CharacterLocomotionState3D::Turning;
    // (So com o corpo parado de fato: armado nas paradas curtas no meio de uma
    // troca - inversao, giro - ele atrasava a mudanca de direcao.)
    m_bodyRestSeconds = std::hypot(m_filteredCenterOfMassVelocity.x, m_filteredCenterOfMassVelocity.y) < 0.3f
        ? std::min(10.0f, m_bodyRestSeconds + deltaTime) : 0.0f;
    if (StartDoseSeconds > 0.0f && input.forceDrivenRoot && !getUpActive && idleLike && m_bodyRestSeconds > 0.3f)
        m_startDoseCap = StartSpeedBase;
    else if (m_startDoseCap < 100.0f)
        // Dosada so no comeco (StartDoseSeconds): depois o teto sobe rapido
        // ate sumir - devagar ate o fim, o sprint levava 0,93 s a 80%.
        m_startDoseCap += (m_startDoseCap < StartSpeedBase + StartSpeedRate * StartDoseSeconds
            ? StartSpeedRate : 30.0f) * deltaTime;

    if (enteredForwardGait) {
        m_startClipActive = true;
        m_startClipSprint = desiredState == CharacterLocomotionState3D::Running;
        m_startClipSeconds = 0.0f;
        m_cyclePhase = 0.0f;
        // O primeiro passo e do pe de tras (no rumo pedido), como na
        // referencia Idle To Sprint: o ciclo comeca quando o contato dele cai
        // no clipe. Na fase 0 o direito ja estava a 42% do balanco - o alvo o
        // queria no ar com o pe ainda carregado no chao, e ele saia
        // arrastado; com o direito na frente, o pe da frente dava o passo.
        const AnimationClip3D* startClip = desiredState == CharacterLocomotionState3D::Running
            && animations.sprint != nullptr && m_sprintClipAllowed ? animations.sprint : animations.walk;
        const Vec3 wanted { input.intent.requestedDirectionWorld.x, input.intent.requestedDirectionWorld.y, 0.0f };
        if (StartFromRearFoot && input.forceDrivenRoot && startClip != nullptr && startClip->durationSeconds > 0.0f
            && wanted.lengthSquared() > 0.01f && physical.valid) {
            const Vec3 direction = wanted.normalized();
            const float back[2] = {
                dot(physical.feet[0].soleCenterWorld - physical.centerOfMassWorld, direction),
                dot(physical.feet[1].soleCenterWorld - physical.centerOfMassWorld, direction) };
            // Quase lado a lado: sai o menos carregado.
            const int rear = std::abs(back[0] - back[1]) > 0.05f ? (back[0] < back[1] ? 0 : 1)
                : (physical.feet[0].loadNewtons < physical.feet[1].loadNewtons ? 0 : 1);
            const AnimationScalarTrack3D& contact = rear == 0 ? startClip->leftFootContact : startClip->rightFootContact;
            const float duration = startClip->durationSeconds;
            constexpr int Samples = 240;
            float liftoff = -1.0f;
            float previous = sampleAnimationScalarTrack3D(contact, duration * (Samples - 1) / Samples, duration, true);
            for (int i = 0; i < Samples; ++i) {
                const float now = sampleAnimationScalarTrack3D(contact, duration * i / Samples, duration, true);
                if (previous >= 0.5f && now < 0.5f) { liftoff = static_cast<float>(i) / Samples; break; }
                previous = now;
            }
            if (liftoff >= 0.0f) {
                // Fase do ciclo (a do clipe menos o deslocamento do apoio do
                // esquerdo - ver stanceAlignedPhase).
                const float offset = stanceAlignedPhase(*startClip, 0.0f);
                m_cyclePhase = std::fmod(liftoff - offset - StartPhaseLead + 2.0f, 1.0f);
            }
        }
    } else if (desiredState != CharacterLocomotionState3D::Walking
        && desiredState != CharacterLocomotionState3D::Running) {
        m_startClipActive = false;
    }
    // Pouso: a velocidade de queda do corpo no toque decide quanto ele
    // agacha e por quanto tempo (0,18 s num degrau; ~0,5 s caindo de um
    // pulo alto). Medida no voo, antes do toque (no tick do toque ela ja
    // caiu pela metade).
    if (desiredState == CharacterLocomotionState3D::Airborne
        && !state.links.empty()) {
        m_airborneFallSpeed = std::max(0.0f, input.forceDrivenRoot
            ? -state.links.front().linearVelocity.z
            : -input.rootVelocityWorld.z);
    }
    m_sinceTouchdownSeconds = enteringLanding ? 0.0f
        : std::min(10.0f, m_sinceTouchdownSeconds + deltaTime);
    if (enteringLanding) {
        m_landingImpactSpeed = m_airborneFallSpeed;
        m_landingSettleSeconds = input.forceDrivenRoot
            // (subida do agachamento em >= ~0,2 s: mais rapido, a ajuda de
            // altura ia ao maximo para acompanhar e o corpo quicava)
            ? 0.34f + 0.09f * std::clamp(m_landingImpactSpeed - 1.0f, 0.0f, 5.0f)
            : 0.18f;
    }
    m_previousGrounded = grounded;
    m_stateSeconds += deltaTime;
    if (m_startClipActive) {
        m_startClipSeconds += deltaTime;
        // A captura contem uma passada inteira de sprint, inclusive voo. No
        // trote usamos so a transferencia inicial; reproduzi-la inteira e
        // mais rapido deixava ambos os pes no ar por ~0,4 s.
        const float duration = m_startClipSprint ? 0.62f : 0.34f;
        if (animations.idleToSprint == nullptr
            || m_gaitDirection != CharacterGaitDirection3D::Forward
            || m_startClipSeconds >= std::min(
                duration, animations.idleToSprint->durationSeconds)) {
            m_startClipActive = false;
        }
    }
    m_transitionSeconds += deltaTime;
    m_idleSeconds += deltaTime;

    const Quaternion heading = Quaternion::fromAxisAngle(
        { 0.0f, 0.0f, 1.0f }, m_facingYaw);
    const Vec3 localVelocity = heading.conjugate().rotate(
        desiredPlanarVelocity);

    // Um ciclo por vez, sem mistura por direcao: a pelve ja se alinha com o
    // movimento (ou contra ele, recuando), entao o ciclo sempre anda para a
    // frente ou para tras em relacao a ela. Walking toca a caminhada rapida
    // (ou o recuo), Running o sprint quando existe - senao a caminhada.
    // Todo outro estado (giro parado, que nao tem clipe proprio, e
    // Airborne/Landing, procedurais - ver applyFlightPose) usa o idle.
    const bool gaitMoving = desiredState == CharacterLocomotionState3D::Walking
        || desiredState == CharacterLocomotionState3D::Running;
    // Arrancada fisica em tres tempos, medida na referencia Idle To Sprint:
    // comprime primeiro (quase sem deslocamento), transfere peito/COM sobre o
    // apoio e so entao entrega o ciclo cheio. O sprint usa a leitura inteira;
    // o trote conserva a mesma mecanica com metade da amplitude e menos tempo.
    Vec3 startupDirection = desiredPlanarVelocity;
    startupDirection.z = 0.0f;
    const float startupWantedSpeed = startupDirection.length();
    if (startupWantedSpeed > 0.001f) startupDirection *= 1.0f / startupWantedSpeed;
    const float startupBodySpeed = startupWantedSpeed > 0.001f
        ? std::max(0.0f, dot(Vec3 { m_filteredCenterOfMassVelocity.x,
            m_filteredCenterOfMassVelocity.y, 0.0f }, startupDirection)) : 0.0f;
    // v2 (T_StartFromRest): a arrancada so vale saindo de parado. Pousando
    // de um pulo em sprint o estado tambem recomeca (Airborne -> Running) com
    // o corpo abaixo de 85% do pedido, e ele corria o comeco do ciclo como
    // quem sai parado.
    if (enteredForwardGait) m_gaitStartFromRest = true;
    else if (desiredState != previousState) m_gaitStartFromRest = false;
    const bool gaitStarting = input.forceDrivenRoot && gaitMoving
        && (!StartOnlyFromRest || m_gaitStartFromRest)
        && m_stateSeconds < 0.65f && startupWantedSpeed > 1.0f
        && startupBodySpeed < 0.85f * startupWantedSpeed;
    const float startupTime = gaitStarting
        ? std::clamp(m_stateSeconds / 0.65f, 0.0f, 1.0f) : 1.0f;
    const float startupRise = smoothStep(startupTime / 0.20f);
    // O primeiro quadro da referencia ja traz ~4,4 graus de avanco. Partir
    // de zero enquanto a perna comeca o ciclo deixa a massa atras dos pes;
    // ha pre-carga imediata, e o restante cresce com a compressao.
    const float startupProgress = gaitStarting
        ? smoothStep(m_stateSeconds / 0.55f) : 1.0f;
    const bool sprintTier = desiredState == CharacterLocomotionState3D::Running
        && animations.sprint != nullptr;
    const bool forwardSprintStart = sprintTier
        && m_gaitDirection == CharacterGaitDirection3D::Forward;
    // Mantem a transferencia minima que ja sustentava o corpo no primeiro
    // quadro e cresce um pouco durante a compressao visual, sobretudo no
    // sprint. O envelope fisico termina quando o corpo atinge 85% do pedido.
    const float startupDrive = gaitStarting
        ? (1.0f - startupProgress) * (0.78f + 0.22f * startupRise) : 0.0f;
    const float startupCompression = gaitStarting
        ? smoothStep(startupTime / 0.16f)
            * (1.0f - smoothStep((startupTime - 0.56f) / 0.34f))
        : 0.0f;
    // A referencia chega perto de 38 graus porque raiz e coluna avancam
    // juntas. No ragdoll de jogo essa soma deslocaria o COM para fora do pe:
    // a leitura atletica fica principalmente na coluna, com a raiz menor.
    const float startupChestAmount =
        (forwardSprintStart ? 0.11f : 0.065f) * startupDrive;
    const float startupRootLean = (forwardSprintStart ? 0.075f : 0.045f)
        * (1.0f - startupProgress);
    const float startupPelvisDrop =
        (forwardSprintStart ? 0.025f : 0.010f) * startupCompression;
    const float gaitCycleSpeed = gaitStarting
        ? std::min(speed, std::max(1.0f, startupBodySpeed + 0.85f)) : speed;
    // Rising plays its own one-shot clip through the same path as every
    // other clip: sampled joint by joint, with the clip's own root track.
    const AnimationClip3D* standUpClip =
        m_getUpPhase == CharacterGetUpPhase3D::Rising
            ? (m_tackling ? m_tackleClip
                : m_fallOrientation == CharacterFallOrientation3D::FaceDown
                ? animations.standUpFront : animations.standUpBack)
            : nullptr;
    // Com pouca ajuda o clipe de sprint (feito para ~7,5 m/s) so entra com a
    // passada rapida: limitada pela inercia do corpo (2-4 m/s na arrancada),
    // ele tocava a 0,55 do ritmo - a cadencia lentissima nao trazia o pe a
    // frente a tempo e, soltando o comando, os dois pes ficavam atras do
    // corpo. Abaixo disso corre no ciclo de trote, no ritmo dele.
    {
        const float lowAssistClip = smoothStep((0.5f - std::clamp(input.walkingAssist, 0.0f, 1.0f)) / 0.4f);
        if (SprintClipSpeed <= 0.0f || (lowAssistClip < 0.5f && !SprintClipAllAssist)
            || desiredState != CharacterLocomotionState3D::Running)
            m_sprintClipAllowed = true;
        else if (speed > SprintClipSpeed + 0.3f)
            m_sprintClipAllowed = true;
        else if (speed < SprintClipSpeed - 0.3f)
            m_sprintClipAllowed = false;
    }
    const bool sprintingGait =
        desiredState == CharacterLocomotionState3D::Running && m_sprintClipAllowed;
    const AnimationClip3D* gaitClip = animations.walk;
    switch (m_gaitDirection) {
    case CharacterGaitDirection3D::Backward:
        gaitClip = sprintingGait && animations.sprintBackward != nullptr
            ? animations.sprintBackward : animations.walkBackward;
        break;
    case CharacterGaitDirection3D::Left:
        gaitClip = sprintingGait && animations.sprintStrafeLeft != nullptr
            ? animations.sprintStrafeLeft : animations.strafeLeft;
        break;
    case CharacterGaitDirection3D::Right:
        gaitClip = sprintingGait && animations.sprintStrafeRight != nullptr
            ? animations.sprintStrafeRight : animations.strafeRight;
        break;
    default:
        gaitClip = sprintTier && m_sprintClipAllowed ? animations.sprint : animations.walk;
        break;
    }
    const bool jumpPose = m_jumpClip != nullptr
        && (desiredState == CharacterLocomotionState3D::Airborne
            || desiredState == CharacterLocomotionState3D::Landing);
    const bool leapPose = jumpPose && m_leapClip != nullptr;
    // (Com RunLeapHold: vai a meia passada no comeco do voo, segura e so
    // completa ate o toque no fim - as pernas balancando no ar inteiro giravam
    // a pelve ate ~45 graus na descida.)
    const auto leapProgress = [](float phase) {
        if (RunLeapHold <= 0.0f) return phase;
        return RunLeapHold * smoothStep(phase / 0.3f)
            + (1.0f - RunLeapHold) * smoothStep((phase - 0.7f) / 0.3f);
    };
    const AnimationClip3D* idleClip = animations.idle;
    const bool authoredStartActive = m_startClipActive
        && animations.idleToSprint != nullptr && gaitMoving;
    // Curva pelo movimento real, inclusive de costas: v longitudinal com
    // sinal vezes a taxa de giro da pelve. Usar apenas |v| invertia o lado da
    // inclinacao ao correr para tras.
    const float signedCurveAcceleration = localVelocity.x * m_turningRate;
    const float curveSpeedShare = smoothStep((speed - 1.0f) / 2.5f);
    const float desiredCurveBlend = authoredStartActive ? 0.0f
        : std::clamp(smoothStep((std::abs(signedCurveAcceleration) - 0.35f)
            / 3.5f) * curveSpeedShare, 0.0f, 0.72f);
    int desiredCurveKind = 0;
    if (gaitMoving && desiredCurveBlend > 0.001f) {
        if (m_gaitDirection == CharacterGaitDirection3D::Forward) {
            if (signedCurveAcceleration > 0.0f) {
                desiredCurveKind = animations.runForwardArcLeft != nullptr ? 1 : 0;
            } else {
                desiredCurveKind = animations.runForwardArcRight != nullptr ? 2 : 0;
            }
        } else if (m_gaitDirection == CharacterGaitDirection3D::Backward
            && signedCurveAcceleration < 0.0f) {
            desiredCurveKind = animations.runBackwardArcRight != nullptr ? 3 : 0;
        }
    }
    // Nunca troca uma captura por outra com peso alto: primeiro sai da atual,
    // depois entra na nova. Isso evita um chicote de perna ao cruzar um setor
    // ou inverter a curva.
    const bool changingCurve = desiredCurveKind != m_authoredCurveKind;
    const float curveBlendTarget = changingCurve ? 0.0f : desiredCurveBlend;
    const float curveBlendRate = curveBlendTarget > m_authoredCurveBlend
        ? 2.5f : 4.0f;
    m_authoredCurveBlend += std::clamp(curveBlendTarget - m_authoredCurveBlend,
        -curveBlendRate * deltaTime, curveBlendRate * deltaTime);
    if (changingCurve && m_authoredCurveBlend <= 0.001f)
        m_authoredCurveKind = desiredCurveKind;
    const float authoredCurveBlend = m_authoredCurveBlend;
    const int authoredCurveKind = m_authoredCurveKind;
    const AnimationClip3D* authoredCurveClip = authoredCurveKind == 1
        ? animations.runForwardArcLeft : authoredCurveKind == 2
        ? animations.runForwardArcRight : authoredCurveKind == 3
        ? animations.runBackwardArcRight : nullptr;
    const AnimationClip3D* selectedClip = standUpClip != nullptr ? standUpClip
        : jumpPose ? (leapPose ? m_leapClip : m_jumpClip)
        : !gaitMoving ? idleClip : gaitClip;
    // Trocar de ciclo sem trocar de estado (troca de setor, passada <->
    // sprint) precisa do mesmo crossfade que a troca de estado faz em
    // enterState. Entre setores ele e mais longo: o pe muda de trajetoria
    // enquanto a pelve vira ~90 graus, e em 0,14 s o alvo do pe chegava a
    // 15 m/s (medido na varredura de direcao, entre frente e recuo).
    if (selectedClip != m_previousClip) {
        if (m_previousClip != nullptr) {
            m_transitionFrom = m_poseCoordinates.size()
                    == m_coordinates.size()
                ? m_poseCoordinates : m_coordinates;
            m_transitionFromRootTilt = m_rootTilt;
            m_transitionVelocity = m_poseVelocities;
            m_transitionRootTiltVelocity = m_rootTiltVelocity;
            m_transitionRootOffsetVelocity = m_rootOffsetVelocity;
            m_transitionFromRootOffset = m_rootOffset;
            m_transitionSeconds = deltaTime;
            // Troca de setor: o ciclo novo anda em outra direcao em relacao a
            // pelve (frente, lado, costas).
            const bool gaitFlip = gaitMoving && selectedClip != nullptr
                && selectedClip != idleClip
                && m_previousClip != idleClip
                && std::abs(wrapAngle(selectedClip->travelDirectionRadians
                    - m_previousClip->travelDirectionRadians)) > 0.1f;
            m_transitionDurationSeconds = gaitFlip
                ? GaitFlipTransitionSeconds : DefaultTransitionSeconds;
            if (gaitFlip) m_sinceGaitFlip = 0.0f;
        }
        m_previousClip = selectedClip;
    }

    // O ciclo anda com a capsula: ritmo dentro de uma faixa natural, e o
    // resto da velocidade vira passo mais curto ou mais longo (stride
    // warping, nos pes, mais abaixo). Com o ritmo sozinho travado em 0,35 o
    // pe andava mais que o corpo devagar, e o pe de apoio escorregava.
    float playbackRate = 1.0f;
    float strideScale = 1.0f;
    // Escada e ladeira: o chao a frente, na direcao do movimento. Quanto ele
    // sobe (ou desce) por metro limita o passo: cada passo sobe no maximo
    // StepRiseLimit (dois degraus), como quem sobe escada correndo. Na
    // passada inteira do trote (1,1 m) o pe pousava 4 degraus acima ou
    // abaixo - pernas contorcidas subindo, pe pendurado descendo.
    float terrainSlope = 0.0f;
    bool terrainIsRamp = false;
    float terrainDemand = 0.0f;
    Vec3 planarTravel { input.rootVelocityWorld.x,
        input.rootVelocityWorld.y, 0.0f };
    planarTravel += (brakeNavigationVelocity - planarTravel) * m_brakeGait;
    if (input.groundAt && gaitMoving && input.grounded
        && planarTravel.lengthSquared() > 0.01f) {
        const Vec3 ahead = planarTravel.normalized();
        const float base = input.rootPositionWorld.z
            - profile.standingRootHeightMeters;
        // A 35 cm um unico espelho de degrau ja dava inclinacao de 0,5 a 1:
        // medir mais longe mede a escada, nao o degrau.
        // Correndo rapido olha mais longe: do sprint a 7 m/s, vista a 2,1 m,
        // a escada chegava antes de o corpo conseguir frear.
        std::array<float, 7> rise {};
        std::array<float, 7> at {};
        std::size_t count = 0;
        int tilted = 0;
        for (const float distance : { 0.70f, 1.05f, 1.40f, 1.75f, 2.10f, 2.80f, 3.50f }) {
            if (distance > 1.40f && (!input.forceDrivenRoot || speed < 4.0f)) continue;
            if (distance > 2.10f && speed < 5.0f) continue;
            Vec3 point = input.rootPositionWorld + ahead * distance;
            point.z = base;
            const GroundProbeResult3D probe = input.groundAt(point);
            if (!probe.hasSurface) continue;
            rise[count] = probe.pointWorld.z - base;
            at[count] = distance;
            ++count;
            if (probe.slopeDegrees > 4.0f && probe.slopeDegrees < 45.0f) ++tilted;
        }
        // So a subida (ou descida) que continua - ladeira, escada - pede
        // passo curto e corpo mais devagar: a altura cresce com a distancia,
        // sempre no mesmo sentido. Um degrau so (meio-fio, calcada: sobe e
        // fica plano) ou uma barra no caminho nao: o pe passa por cima dele
        // no passo normal (a folga do balanco ve o obstaculo). Tratado como
        // ladeira, um meio-fio de 12 cm levava o sprint de 7,3 a 4,5 m/s.
        for (std::size_t i = 0; i < count; ++i)
            if (std::abs(rise[i]) > 0.06f) terrainDemand = 1.0f;
        // No modo cinematico (a capsula carrega o corpo), a regra de antes:
        // qualquer desnivel a frente encurta o passo.
        if (!input.forceDrivenRoot)
            for (std::size_t i = 0; i < count; ++i)
                terrainSlope = std::max(terrainSlope, std::abs(rise[i]) / at[i]);
        // So a subida (ou descida) que continua - ladeira, escada -: a altura
        // nunca volta contra o sentido e sobe (3 cm ou mais) em pelo menos
        // dois trechos entre sondas (numa escada um trecho pode cair num
        // mesmo piso). Um degrau so (sobe e fica plano), uma barra no caminho
        // ou um degrau que aparece so na sonda mais longe nao contam. A
        // inclinacao e a do trecho do primeiro degrau ate a sonda mais longe.
        if (count >= 3 && input.forceDrivenRoot) {
            const float direction = rise[count - 1] - rise[0];
            int rising = 0;
            bool against = false;
            std::size_t first = count;
            for (std::size_t i = 1; i < count; ++i) {
                const float step = rise[i] - rise[i - 1];
                if (step * direction < 0.0f && std::abs(step) > 0.02f) against = true;
                if (std::abs(step) >= 0.03f && step * direction > 0.0f) {
                    ++rising;
                    if (first == count) first = i - 1;
                }
            }
            if (!against && rising >= 2) {
                terrainSlope = std::abs(rise[count - 1] - rise[first])
                    / std::max(0.1f, at[count - 1] - at[first]);
            } else if (!against && rising >= 1 && std::abs(rise[0]) >= 0.30f) {
                // No meio da subida (o chao a 0,7 m ja 30 cm acima ou abaixo):
                // o fim da escada a vista nao a desfaz - os ultimos degraus
                // ainda sao escada.
                terrainSlope = std::abs(rise[count - 1]) / std::max(0.1f, at[count - 1]);
            }
        }
        // Reconhecida a subida continua, por 0,6 s vale a medida de antes (o
        // maior desnivel sobre a distancia): em cima da escada a sequencia
        // subindo se desfaz entre degraus, o teto subia no meio dela e o
        // corpo tombava.
        if (input.forceDrivenRoot) {
            m_sustainedTerrainHold = terrainSlope > 0.08f ? 0.6f
                : std::max(0.0f, m_sustainedTerrainHold - deltaTime);
            if (m_sustainedTerrainHold > 0.0f)
                for (std::size_t i = 0; i < count; ++i)
                    terrainSlope = std::max(terrainSlope, std::abs(rise[i]) / at[i]);
        }
        // Rampa (a superficie sondada inclinada, sem espelhos): o passo pode
        // subir mais que numa escada.
        terrainIsRamp = tilted * 2 >= static_cast<int>(count) && count > 0;
    }
    // Quanto um passo sobe no maximo: numa escada, um ou dois espelhos; numa
    // rampa (sem espelho), mais - no passo de escada, uma rampa de 20 graus
    // levava o sprint a 39% da velocidade.
    const float StepRiseLimit = !input.forceDrivenRoot ? 0.34f
        : (terrainIsRamp || m_terrainRampHold > 0.0f) ? 0.32f : 0.20f;
    // (A rampa segura a classificacao por um tempo: entre duas sondas o
    // plano da rampa some.)
    if (terrainSlope > 0.08f) terrainDemand = 1.0f;
    m_terrainDemand = terrainDemand > m_terrainDemand ? terrainDemand
        : std::max(0.0f, m_terrainDemand - deltaTime / 0.6f);
    m_telemetry.terrainDemand = m_terrainDemand;
    m_terrainRampHold = terrainIsRamp ? 0.3f : std::max(0.0f, m_terrainRampHold - deltaTime);
    m_terrainIsRamp = m_terrainRampHold > 0.0f;
    // Sobe rapido (o degrau aparece de uma vez) e solta devagar.
    m_terrainSlope += (terrainSlope - m_terrainSlope) * (1.0f - std::exp(
        -(terrainSlope > m_terrainSlope ? 20.0f : 4.0f) * deltaTime));
    float terrainSpeedLimit = std::numeric_limits<float>::infinity();
    if (input.forceDrivenRoot && m_terrainSlope > 0.08f) {
        // Ladeira e escada: o corpo desacelera na proporcao da inclinacao
        // (10 graus ~90% da velocidade pedida, 20 graus ~70%, escada ~55%),
        // com um teto para escada ingreme. (Era um teto fixo de 0,48 /
        // inclinacao, ate 2,8 m/s: uma rampa de 10 graus levava o sprint de
        // 7,3 a 3,3 m/s.)
        // (Absoluto, sobre a velocidade de sprint: o jogo ja corta o pedido
        // por este teto, e calculado do pedido ele descia tick a tick.)
        constexpr float TerrainTopSpeed = 7.5f;
        const float share = std::clamp(1.0f - 1.1f * (m_terrainSlope - 0.08f), 0.55f, 1.0f);
        terrainSpeedLimit = TerrainTopSpeed * share;
        // Escada (subida continua sem ser rampa): cada degrau precisa de
        // tempo para o corpo subir o espelho - o teto de antes, continuo (so
        // acima de uma inclinacao ele chegava rapido demais e caia).
        if (!m_terrainIsRamp)
            terrainSpeedLimit = std::min(terrainSpeedLimit, std::max(0.85f, 0.48f / m_terrainSlope));
    }
    // Parado (sem passada), as pernas voltam ao mesmo ciclo.
    if (!gaitMoving) {
        m_armPhaseValid = false;
        m_phaseRateScale = 1.0f;
        m_contactSteady = false;
        m_legPhaseLead = {};
        m_waitingLeg = -1;
        m_touchdownWaitSeconds = 0.0f;
    }
    if (gaitMoving && selectedClip != nullptr) {
        const float authoredSpeed = authoredHorizontalSpeed(*selectedClip);
        if (authoredSpeed > 0.05f) {
            const float ratio = gaitCycleSpeed / authoredSpeed;
            // Passo do clipe (dois por ciclo) e o maior que o terreno deixa.
            const float authoredStep = authoredSpeed
                * selectedClip->durationSeconds * 0.5f;
            const float strideCap = m_terrainSlope > 0.05f
                ? std::clamp(StepRiseLimit / (m_terrainSlope * authoredStep),
                    0.25f, 1.0f)
                : 1.35f;
            // Com pouca ajuda o corpo acelera devagar e a passada fica bem
            // abaixo do clipe: com o ritmo caindo ate 0,55 a cadencia ficava
            // lentissima com a passada inteira (o pe nao chegava a frente a
            // tempo). Quem arranca da passos rapidos e curtos: o ritmo fica
            // alto e a passada encurta.
            const float lowAssistRate = smoothStep((0.5f - std::clamp(input.walkingAssist, 0.0f, 1.0f)) / 0.4f);
            // So correndo (pedido acima do trote, como a folga da passada): no
            // trote, parando, o ritmo alto derrubava (5 -> 10-21 quedas em 32).
            const bool runRequest = std::hypot(input.intent.requestedVelocityWorld.x,
                input.intent.requestedVelocityWorld.y) > GaitLeadMinimumRequest;
            const float rateFloor = 0.55f + (std::max(0.55f, LowAssistRateFloor) - 0.55f) * lowAssistRate
                * (runRequest ? 1.0f : 0.0f) * (LowAssistRateFloorInBrake ? 1.0f : 1.0f - m_brakeGait);
            const float sprintFloor = selectedClip == animations.sprint
                ? SprintRateFloor * (1.0f - m_brakeGait) + 0.55f * m_brakeGait : 0.55f;
            playbackRate = std::clamp(ratio / std::min(1.0f, strideCap),
                std::max(rateFloor, sprintFloor), 1.45f);
            // Quase-queda: o passo seguinte sai e pousa mais cedo (passada
            // rapida); o comprimento fica o do pouso deslocado.
            if (StumbleDetection && m_tripRecovery > 0.0f)
                playbackRate *= 1.0f + StumbleCadence * m_tripRecovery;
            strideScale = std::clamp(ratio / playbackRate, 0.0f, strideCap);
            // Preparo do carrinho: o passo que pousa e o de firmar o pe.
            if (m_tacklePending)
                strideScale *= 1.0f + TacklePlantStride * smoothStep(m_tacklePendingSeconds / 0.10f);
            // Velocidade que cabe no terreno com o passo limitado e o ritmo
            // ate 15% acima do clipe; o jogo a usa como teto da capsula.
            if (strideCap < 1.0f) {
                terrainSpeedLimit = std::min(terrainSpeedLimit,
                    authoredSpeed * strideCap * 1.15f);
            }
            // TEMPTUNE: passada pelo contato (pedido do usuario, 04/10). Correndo,
            // a sequencia segue os pes, nao o relogio do clipe:
            //  - o pe que vai pousar (o clipe ja o poe no apoio) e ainda esta
            //    no ar, com o outro tambem fora do chao, ESPERA o toque: o
            //    ciclo para nesse ponto, a perna estendida para o chao;
            //  - a perna de tras continua recolhendo no seu ritmo (adianta-se
            //    ao ciclo, ate o meio do balanco). Congelar o ciclo inteiro a
            //    deixava baixa, atras, enquanto o corpo andava ~0,5 m: pegava
            //    a barra do tropeco (7 -> 36 quedas em 96);
            //  - tocou, o ciclo recupera a diferenca (a perna da frente anda
            //    mais rapido no apoio ate as duas casarem de novo);
            //  - o pe que encosta no fim do balanco, antes do toque do clipe,
            //    adianta o ciclo ate o apoio (em vez de arrastar a ponta).
            // A espera vai ate ContactWaitSeconds (0,08 s, o voo normal do
            // sprint): mais longa, depois de bater numa barra o ciclo ficava
            // esperando o equivalente a ~1-2 m de corrida e o pe descia em
            // cima dela (tropeco do sprint: 0 -> 17 quedas em 64 com 0,12 s).
            // So correndo para a frente, estabelecido: nada disso na freada,
            // na arrancada, no tropeco, em degrau ou trocando de passada.
            float phaseRate = playbackRate;
            const float cycleSeconds = std::max(0.1f, selectedClip->durationSeconds);
            const bool stumbling = m_tripRecovery > 0.001f || m_stumbleRisk > 0.2f
                || m_stumbleStepping || m_footwork.plan().recovering;
            // (No movimento livre tambem: a espera nao sustenta o corpo, so nao
            // deixa a perna esticar com o pe ainda chegando - sem ela a pelve
            // subia ate 12 cm acima da referencia no sprint, contra 6.)
            const bool contactSteady = ContactPhase && input.forceDrivenRoot && physical.feet.size() >= 2
                && m_feet.size() >= 2 && !stumbling
                && m_gaitDirection == CharacterGaitDirection3D::Forward
                && m_brakeGait <= 0.0f && !m_brakeStepping && !m_startClipActive && m_gaitChange <= 0.0f
                && m_terrainSlope < 0.05f
                && std::hypot(input.intent.requestedVelocityWorld.x, input.intent.requestedVelocityWorld.y) > 1.0f;
            int waitingLeg = -1;
            bool earlyTouch = false;
            for (std::size_t side = 0; side < 2 && side < physical.feet.size(); ++side) {
                const FootSupportEstimate3D& foot = physical.feet[side];
                const bool down = foot.touching || foot.supporting;
                const bool clipStance = m_telemetry.clipFootContact[side] > 0.55f;
                if (!clipStance) m_legTouchedInStance[side] = false;
                else if (down) m_legTouchedInStance[side] = true;
                if (!contactSteady) continue;
                if (clipStance && !m_legTouchedInStance[side] && !physical.feet[1 - side].supporting)
                    waitingLeg = static_cast<int>(side);
                if (!clipStance && m_telemetry.footSwingProgress[side] >= ContactEarlyProgress && down)
                    earlyTouch = true;
            }
            m_contactSteady = contactSteady;
            m_waitingLeg = waitingLeg >= 0 && m_touchdownWaitSeconds < ContactWaitSeconds ? waitingLeg : -1;
            if (waitingLeg >= 0 && m_touchdownWaitSeconds < ContactWaitSeconds) {
                m_touchdownWaitSeconds += deltaTime;
                // A perna de tras segue recolhendo, ate o meio do balanco.
                const std::size_t trailing = static_cast<std::size_t>(1 - waitingLeg);
                if (m_telemetry.footSwingProgress[trailing] >= 0.0f
                    && m_telemetry.footSwingProgress[trailing] < ContactTrailCap)
                    m_legPhaseLead[trailing] = std::min(ContactMaxLead,
                        m_legPhaseLead[trailing] + deltaTime * playbackRate / cycleSeconds);
                phaseRate = 0.0f;
            } else {
                if (waitingLeg < 0) m_touchdownWaitSeconds = 0.0f;
                if (earlyTouch) phaseRate *= ContactEarlyBoost;
            }
            const float phaseBefore = m_cyclePhase;
            // O ciclo recupera a diferenca das pernas: a parte comum vai para
            // ele de uma vez (nenhuma perna muda), o resto aos poucos.
            {
                const float common = std::min(m_legPhaseLead[0], m_legPhaseLead[1]);
                m_legPhaseLead[0] -= common;
                m_legPhaseLead[1] -= common;
                float extra = common;
                if (phaseRate > 0.0f) {
                    // (Fora da corrida estabelecida - freando, por exemplo - a
                    // diferenca sai mais rapido, mas continua: zerada de uma
                    // vez, a perna que pousava saltava de pose no comeco da
                    // freada.)
                    const float rate = ContactCatchUp * (contactSteady ? 1.0f : 3.0f);
                    for (std::size_t side = 0; side < 2; ++side) {
                        const float absorb = std::min(m_legPhaseLead[side],
                            deltaTime * std::max(playbackRate, 0.5f) / cycleSeconds * rate);
                        m_legPhaseLead[side] -= absorb;
                        extra += absorb;
                    }
                }
                m_cyclePhase = std::fmod(m_cyclePhase + extra + 1.0f, 1.0f);
            }
            m_telemetry.touchdownWaitSeconds = m_touchdownWaitSeconds;
            m_telemetry.legPhaseLead = m_legPhaseLead;
            // TEMPTUNE: o ritmo do ciclo muda aos poucos - freia rapido (a
            // perna da frente para na pose de toque) e retoma devagar. Em
            // degraus (parar, voltar, 3x no toque adiantado) braco e perna
            // davam trancos: aceleracao dos alvos do braco 4x maior no sprint.
            // (No tropeco e na quase-queda nao: ali o passo rapido que salva
            // precisa do ciclo na hora, e os bracos presos a ele - suavizados,
            // o sprint na barra escondida caia muito mais.)
            if (ContactRateSmooth && stumbling) {
                m_phaseRateScale = playbackRate > 1e-4f ? phaseRate / playbackRate : 1.0f;
            } else if (ContactRateSmooth) {
                const float wanted = playbackRate > 1e-4f ? phaseRate / playbackRate : 1.0f;
                const float tau = wanted < m_phaseRateScale ? ContactRateDownSeconds : ContactRateUpSeconds;
                m_phaseRateScale += (wanted - m_phaseRateScale) * (1.0f - std::exp(-deltaTime / std::max(0.005f, tau)));
                phaseRate = playbackRate * m_phaseRateScale;
            }
            // Chute: o ciclo mais devagar (o apoio no chao durante o golpe).
            phaseRate *= std::clamp(input.ballGaitRate, 0.2f, 1.0f);
            m_cyclePhase = std::fmod(m_cyclePhase + deltaTime * phaseRate
                / std::max(0.1f, selectedClip->durationSeconds), 1.0f);
            // TEMPTUNE: os bracos andam numa fase propria, suave: seguem o
            // ritmo medio do ciclo e se prendem a ele devagar. Esperando o
            // toque o ciclo para, mas o corredor nao congela os bracos no ar.
            {
                float advanced = m_cyclePhase - phaseBefore;
                if (advanced < -0.5f) advanced += 1.0f;
                const float globalRate = advanced / std::max(1e-4f, deltaTime);
                if (!m_armPhaseValid || ArmPhaseSmoothSeconds <= 0.0f || stumbling) {
                    m_armPhase = m_cyclePhase;
                    m_armPhaseRate = globalRate;
                    m_armPhaseValid = ArmPhaseSmoothSeconds > 0.0f;
                } else {
                    m_armPhaseRate += (globalRate - m_armPhaseRate) * (1.0f - std::exp(-deltaTime / ArmPhaseSmoothSeconds));
                    float error = m_cyclePhase - m_armPhase;
                    error -= std::round(error);
                    m_armPhase = std::fmod(m_armPhase + (m_armPhaseRate + error / ArmPhaseLockSeconds) * deltaTime + 1.0f, 1.0f);
                }
            }
            m_varietyStrides += deltaTime * phaseRate
                / std::max(0.1f, selectedClip->durationSeconds);
        }
    }

    // Saindo de escada ou rampa o teto sobe aos poucos (2 m/s2) enquanto o
    // corpo ainda esta nela; entrando, baixa na hora. As sondas veem o
    // patamar ~0,7 m antes e o teto saltava com o corpo ainda no ultimo
    // degrau: de 0,55 a 3 m/s em 0,35 s, ele tombava para a frente no topo
    // da escada. (Com o corpo ainda no plano - chegando numa rampa, a
    // estimativa oscilando - o teto acompanha na hora: devagar, ficava preso
    // no menor valor e o sprint chegava a rampa a 5,3 m/s em vez de 7.)
    {
        bool incline = false;
        float lowest = std::numeric_limits<float>::infinity();
        float highest = -lowest;
        int planted = 0;
        for (const FootPlant& plant : m_feet) {
            if (!plant.valid || !plant.planted) continue;
            ++planted;
            lowest = std::min(lowest, plant.groundHeight);
            highest = std::max(highest, plant.groundHeight);
            if (plant.groundNormal.z < 0.9976f) incline = true;  // ~4 graus
        }
        if (planted == 2 && highest - lowest > 0.05f) incline = true;
        m_onInclineHold = incline ? 0.6f : std::max(0.0f, m_onInclineHold - deltaTime);
        constexpr float TerrainCapCeiling = 9.0f, TerrainCapRecovery = 2.0f;
        if (input.forceDrivenRoot) {
            const float wanted = std::min(terrainSpeedLimit, TerrainCapCeiling);
            m_terrainSpeedCap = wanted < m_terrainSpeedCap || m_onInclineHold <= 0.0f ? wanted
                : std::min(wanted, m_terrainSpeedCap + TerrainCapRecovery * deltaTime);
            if (m_terrainSpeedCap < TerrainCapCeiling) terrainSpeedLimit = m_terrainSpeedCap;
        } else {
            m_terrainSpeedCap = TerrainCapCeiling;
        }
    }
    std::vector<Vec3> sampled(profile.links.size(), Vec3 {});
    Quaternion rootRotation;
    Vec3 rootTranslation;
    std::array<float, 2> contactWeights {};
    float clipSampleTime = 0.0f;
    std::array<float, 2> legSampleTime {};
    float armSampleTime = 0.0f;
    // Variacao de movimento (vida), por cima do clipe e antes do crossfade:
    // a troca de estado a mistura como mistura o clipe.
    const float variety = std::clamp(input.motionVariety, 0.0f, 2.0f);
    const bool varietyActive = variety > 0.0f && selectedClip != nullptr
        && standUpClip == nullptr && !jumpPose && !getUpActive;
    // Parado, cada um respira o idle no seu ritmo e do seu ponto (sem
    // isso, varios personagens parados se mexiam em uníssono).
    const bool idleVariety = varietyActive && !gaitMoving
        && (desiredState == CharacterLocomotionState3D::Idle
            || desiredState == CharacterLocomotionState3D::Turning);
    if (m_landingResyncFoot >= 0) {
        if (gaitMoving && selectedClip != nullptr && selectedClip->durationSeconds > 0.0f && !jumpPose) {
            const AnimationScalarTrack3D& contact = m_landingResyncFoot == 0
                ? selectedClip->leftFootContact : selectedClip->rightFootContact;
            constexpr int Samples = 240;
            const float duration = selectedClip->durationSeconds;
            float touchdown = -1.0f;
            float previous = sampleAnimationScalarTrack3D(contact, duration * (Samples - 1) / Samples, duration, true);
            for (int i = 0; i < Samples; ++i) {
                const float now = sampleAnimationScalarTrack3D(contact, duration * i / Samples, duration, true);
                if (previous < 0.5f && now >= 0.5f) { touchdown = static_cast<float>(i) / Samples; break; }
                previous = now;
            }
            if (touchdown >= 0.0f) {
                m_cyclePhase = std::fmod(touchdown + 0.02f - stanceAlignedPhase(*selectedClip, 0.0f) + 2.0f, 1.0f);
                m_legPhaseLead = {};
            }
        }
        m_landingResyncFoot = -1;
    }
    if (m_motionSeedPending) {
        // Vindo do padrao (seedMotion): a fase em que os pes do clipe, em
        // relacao a pelve e no rumo dela, mais se parecem com os do corpo -
        // onde estao e para onde vao -, sem a arrancada; e a pose sai das
        // juntas do corpo com a velocidade delas.
        m_motionSeedPending = false;
        if (gaitMoving && selectedClip != nullptr && standUpClip == nullptr && !jumpPose
            && m_feet[0].valid && m_feet[1].valid && selectedClip->durationSeconds > 0.0f) {
            const AnimationClip3D& clip = *selectedClip;
            const auto clipFeet = [&](float fraction) {
                const RagdollAnimationPose3D pose = sampleRagdollAnimationPose3D(profile, &clip,
                    std::fmod(fraction + 1.0f, 1.0f) * clip.durationSeconds, true);
                const Vec3 pelvisForward = (pose.linkOrientations.front() * profile.links.front().modelOrientation
                    .conjugate()).rotate({ 1.0f, 0.0f, 0.0f });
                const Quaternion back = Quaternion::fromAxisAngle({ 0.0f, 0.0f, 1.0f },
                    -std::atan2(pelvisForward.y, pelvisForward.x));
                std::array<Vec3, 2> out;
                for (std::size_t side = 0; side < 2; ++side)
                    out[side] = back.rotate(pose.linkPositions[m_feet[side].linkIndex] - pose.linkPositions.front());
                return out;
            };
            const float rate = std::clamp(m_seedSpeed / std::max(0.5f, clip.nominalSpeedMetersPerSecond), 0.5f, 2.0f)
                / clip.durationSeconds;
            constexpr int Samples = 48;
            constexpr float Step = 1.0f / 96.0f;
            constexpr float VelocityWeight = 0.08f;
            float best = 1e9f, bestFraction = 0.0f;
            for (int k = 0; k < Samples; ++k) {
                const float fraction = static_cast<float>(k) / static_cast<float>(Samples);
                const auto now = clipFeet(fraction);
                const auto next = clipFeet(fraction + Step);
                float error = 0.0f;
                for (std::size_t side = 0; side < 2; ++side)
                    error += (now[side] - m_seedFeet[side]).lengthSquared()
                        + ((next[side] - now[side]) * (rate / Step) - m_seedFeetVelocity[side]).lengthSquared()
                            * VelocityWeight * VelocityWeight;
                if (error < best) { best = error; bestFraction = fraction; }
            }
            m_cyclePhase = std::fmod(bestFraction - stanceAlignedPhase(clip, 0.0f) + 2.0f, 1.0f);
            m_startClipActive = false;
            m_startStepRequested = false;
        }
        if (m_seedJoints.size() == m_transitionFrom.size()) {
            // O crossfade sai das juntas do corpo, sem a tangente: ela estende
            // a velocidade das juntas por toda a troca (a 20 rad/s no sprint,
            // ~0,9 rad alem) e o sprint caia mais (5 em 8, contra 2). Pela
            // inercia (a da volta ao padrao) a inclinacao e o deslocamento da
            // raiz ficavam sem mistura (o alvo nascia a 18 graus do corpo no
            // trote de lado).
            m_transitionFrom = m_seedJoints;
            m_transitionVelocity.assign(m_transitionFrom.size(), {});
        }
    }
    if (selectedClip != nullptr) {
        const bool loopingState = desiredState == CharacterLocomotionState3D::Idle
            || desiredState == CharacterLocomotionState3D::Turning
            || desiredState == CharacterLocomotionState3D::Airborne;
        // O pulo e amostrado pela fase do voo; na aterrissagem fica na pose
        // de toque (o fim do clipe).
        const float sampleTime = standUpClip != nullptr
            ? std::min(m_getUpClipSeconds, selectedClip->durationSeconds)
            : leapPose
            ? std::fmod(m_leapFrom + m_leapSpan * (desiredState == CharacterLocomotionState3D::Landing
                ? 1.0f : leapProgress(m_flightPhase)), 1.0f) * selectedClip->durationSeconds
            : jumpPose
            ? (desiredState == CharacterLocomotionState3D::Landing ? 1.0f
                : m_flightPhase) * selectedClip->durationSeconds
            : gaitMoving
            ? stanceAlignedPhase(*selectedClip, m_cyclePhase)
                * selectedClip->durationSeconds
            : loopingState && idleVariety
                ? std::fmod(m_stateSeconds * (1.0f + 0.08f * varietyValue(
                    input.varietySeed, 5, 0)) + selectedClip->durationSeconds
                    * (0.5f + 0.5f * varietyValue(input.varietySeed, 6, 0)),
                    selectedClip->durationSeconds)
            : loopingState
                ? std::fmod(m_stateSeconds, selectedClip->durationSeconds)
                : std::min(m_stateSeconds, selectedClip->durationSeconds);
        clipSampleTime = sampleTime;
        // Cada perna no seu tempo (passada pelo contato: a de tras pode estar
        // adiantada enquanto a da frente espera o toque).
        armSampleTime = gaitMoving && standUpClip == nullptr && !jumpPose && m_armPhaseValid
            ? stanceAlignedPhase(*selectedClip, m_armPhase) * selectedClip->durationSeconds : sampleTime;
        for (std::size_t side = 0; side < 2; ++side)
            legSampleTime[side] = gaitMoving && standUpClip == nullptr && !jumpPose
                ? std::fmod(sampleTime + m_legPhaseLead[side] * selectedClip->durationSeconds,
                    std::max(0.05f, selectedClip->durationSeconds))
                : sampleTime;
        if (m_legSideOfLink.size() != profile.links.size()) {
            m_legSideOfLink.assign(profile.links.size(), -1);
            for (std::size_t index = 1; index < profile.links.size(); ++index) {
                const std::string& id = profile.links[index].id;
                const bool legLink = id.find("Thigh") != std::string::npos
                    || id.find("Shin") != std::string::npos || id.find("Foot") != std::string::npos;
                if (legLink) m_legSideOfLink[index] = id.rfind("Left", 0) == 0 ? 0 : id.rfind("Right", 0) == 0 ? 1 : -1;
                else if (id.find("Arm") != std::string::npos || id.find("Forearm") != std::string::npos
                    || id.find("Hand") != std::string::npos) m_legSideOfLink[index] = -2;
            }
        }
        for (std::size_t index = 1; index < profile.links.size(); ++index) {
            const AnimationTrack3D* track = findAnimationTrack3D(
                *selectedClip, profile.links[index].id);
            if (track == nullptr) continue;
            const int legSide = m_legSideOfLink[index];
            sampled[index] = sampleAnimationTrack3D(*track,
                legSide >= 0 ? legSampleTime[static_cast<std::size_t>(legSide)]
                    : legSide == -2 ? armSampleTime : sampleTime,
                selectedClip->durationSeconds, selectedClip->loops)
                    .jointPositionRadians;
        }
        const AnimationTrack3D* rootTrack = findAnimationTrack3D(
            *selectedClip, profile.links.front().id);
        if (rootTrack != nullptr) {
            const AnimationTransformSample3D rootSample =
                sampleAnimationTrack3D(*rootTrack, sampleTime,
                    selectedClip->durationSeconds, selectedClip->loops);
            rootTranslation = rootSample.translationOffsetMeters;
            rootRotation = rootSample.rotationDelta;
        }
        contactWeights[0] = sampleAnimationScalarTrack3D(
            selectedClip->leftFootContact, legSampleTime[0],
            selectedClip->durationSeconds, selectedClip->loops);
        contactWeights[1] = sampleAnimationScalarTrack3D(
            selectedClip->rightFootContact, legSampleTime[1],
            selectedClip->durationSeconds, selectedClip->loops);
    }
    // Pulo parado: no chao, segurando o pulo, o preparo - o agachamento da
    // referencia (bracos para tras, tronco a frente), mais fundo com a carga.
    {
        const auto blendClip = [&](const AnimationClip3D& clip, float time, float weight, float trunkShare) {
            for (std::size_t index = 1; index < profile.links.size(); ++index) {
                const AnimationTrack3D* track = findAnimationTrack3D(clip, profile.links[index].id);
                if (track == nullptr) continue;
                const Vec3 pose = sampleAnimationTrack3D(*track, time, clip.durationSeconds, false).jointPositionRadians;
                const std::string& id = profile.links[index].id;
                const bool trunk = id == "Abdomen" || id == "Chest" || id == "UpperChest" || id == "Neck" || id == "Head";
                sampled[index] += (pose - sampled[index]) * weight * (trunk ? trunkShare : 1.0f);
            }
            if (const AnimationTrack3D* root = findAnimationTrack3D(clip, profile.links.front().id)) {
                const Quaternion tilt = sampleAnimationTrack3D(*root, time, clip.durationSeconds, false).rotationDelta;
                rootRotation = slerpQuaternion(rootRotation, tilt, weight).normalized();
            }
        };
        // Giro parado pelo clipe: as pernas sao as do clipe (o tronco fica com
        // a pose parada e o olhar).
        if (m_clipTurnWeight > 0.001f && m_clipTurn.clip != nullptr && !gaitMoving && !jumpPose
                && !getUpActive && standUpClip == nullptr) {
            const AnimationClip3D& clip = *m_clipTurn.clip;
            for (std::size_t index = 1; index < profile.links.size(); ++index) {
                const std::string& id = profile.links[index].id;
                const bool leg = id.find("Thigh") != std::string::npos || id.find("Shin") != std::string::npos
                    || id.find("Foot") != std::string::npos;
                if (!leg) continue;
                const AnimationTrack3D* track = findAnimationTrack3D(clip, id);
                if (track == nullptr) continue;
                const Vec3 pose = sampleAnimationTrack3D(*track, m_clipTurn.time, clip.durationSeconds, false)
                    .jointPositionRadians;
                sampled[index] += (pose - sampled[index]) * m_clipTurnWeight;
            }
        }
        if (!jumpPose && !getUpActive && standUpClip == nullptr && m_jumpCrouch > 0.001f
                && animations.jumpPrep != nullptr) {
            const float weight = smoothStep(m_jumpCrouch / 0.15f);
            const float depth = std::clamp(m_jumpCrouch, 0.0f, 1.0f);
            // O tronco so com parte da flexao da referencia: o impulso sai
            // pela pelve, e com o tronco dobrado 50 graus a frente (o centro de
            // massa a frente dela) ele saia girando de costas e caia deitado.
            blendClip(*animations.jumpPrep, depth * animations.jumpPrep->durationSeconds, weight, JumpPrepTrunkShare);
        }
    }
    // A arrancada capturada participa como DELTA de gesto, por cima do ciclo
    // de trote/sprint que continua dono das pernas, dos contatos e da raiz.
    // Copiar a pose absoluta do FBX trazia para o guia 33 graus de root pitch,
    // 10 cm de agachamento e uma fase de voo antes de o corpo acelerar.
    if (authoredStartActive && animations.idleToSprint != nullptr) {
        const float duration = m_startClipSprint ? 0.62f : 0.34f;
        const float u = std::clamp(m_startClipSeconds / duration, 0.0f, 1.0f);
        const float envelope = smoothStep(u / 0.20f)
            * (1.0f - smoothStep((u - 0.62f) / 0.38f));
        const float sourceTime = std::min(
            m_startClipSeconds * (m_startClipSprint ? 1.0f : 0.75f),
            animations.idleToSprint->durationSeconds);
        const auto segmentWeight = [&](const std::string& id) {
            const bool spine = id == "Abdomen" || id == "Chest"
                || id == "UpperChest";
            const bool arm = id == "LeftUpperArm" || id == "RightUpperArm"
                || id == "LeftForearm" || id == "RightForearm";
            const bool head = id == "Neck" || id == "Head";
            if (spine) return m_startClipSprint ? 0.55f : 0.28f;
            if (arm) return m_startClipSprint ? 0.62f : 0.30f;
            if (head) return m_startClipSprint ? 0.25f : 0.12f;
            return 0.0f;
        };
        for (std::size_t index = 1; index < profile.links.size(); ++index) {
            const float weight = segmentWeight(profile.links[index].id)
                * envelope;
            if (weight <= 0.0f) continue;
            const AnimationTrack3D* track = findAnimationTrack3D(
                *animations.idleToSprint, profile.links[index].id);
            if (track == nullptr) continue;
            const Vec3 first = sampleAnimationTrack3D(*track, 0.0f,
                animations.idleToSprint->durationSeconds, false)
                    .jointPositionRadians;
            const Vec3 current = sampleAnimationTrack3D(*track, sourceTime,
                animations.idleToSprint->durationSeconds, false)
                    .jointPositionRadians;
            const float limit = m_startClipSprint ? 0.42f : 0.20f;
            const Vec3 delta {
                std::clamp(current.x - first.x, -limit, limit),
                std::clamp(current.y - first.y, -limit, limit),
                std::clamp(current.z - first.z, -limit, limit)
            };
            sampled[index] += delta * weight;
        }
    }
    // A curva real entra apenas nos segmentos que definem apoio e banco do
    // corpo. Cabeca e bracos ficam no ciclo normal; isso tambem exclui
    // explicitamente os canais defeituosos da referencia Arc Left.
    if (authoredCurveClip != nullptr && authoredCurveBlend > 0.0f) {
        const float curveTime = stanceAlignedPhase(*authoredCurveClip,
            m_cyclePhase) * authoredCurveClip->durationSeconds;
        const auto curveSegment = [](const std::string& id) {
            return id == "Abdomen" || id == "Chest" || id == "UpperChest"
                || id == "LeftThigh" || id == "LeftShin" || id == "LeftFoot"
                || id == "RightThigh" || id == "RightShin" || id == "RightFoot";
        };
        for (std::size_t index = 1; index < profile.links.size(); ++index) {
            if (!curveSegment(profile.links[index].id)) continue;
            const AnimationTrack3D* track = findAnimationTrack3D(
                *authoredCurveClip, profile.links[index].id);
            if (track == nullptr) continue;
            const Vec3 curvePose = sampleAnimationTrack3D(*track, curveTime,
                authoredCurveClip->durationSeconds, false).jointPositionRadians;
            const bool spine = profile.links[index].id == "Abdomen"
                || profile.links[index].id == "Chest"
                || profile.links[index].id == "UpperChest";
            // A coluna pode ler a captura com peso alto. Nas pernas ela e
            // uma referencia de estilo menor: o alvo de pouso em arco e o
            // contato fisico continuam soberanos, evitando chicotear o pe ao
            // sincronizar dois ciclos gravados em fases diferentes.
            const float weight = authoredCurveBlend * (spine ? 1.0f : 0.15f);
            Vec3 wanted = curvePose;
            if (spine) {
                // A captura da curva esquerda traz ~30 graus de corcunda como
                // baseline. Curva deve fornecer banco/torcao; a flexao
                // sagital continua sendo a do trote ou sprint selecionado.
                wanted.y = sampled[index].y;
            }
            sampled[index] += (wanted - sampled[index]) * weight;
        }
    }
    if (varietyActive && gaitMoving) {
        applyGaitVariety(profile, *selectedClip, m_cyclePhase,
            m_armPhaseValid ? m_armPhase : m_cyclePhase,
            m_varietyStrides, input.varietySeed, variety, sampled);
    } else if (idleVariety) {
        applyIdleVariety(profile, m_elapsedSeconds, input.varietySeed,
            variety, sampled);
    }
    // TEMPTUNE CurveSpine: na curva o tronco inclina para dentro pela coluna
    // (a parte da inclinacao que a pelve nao rola; depois da variedade, que
    // reamostra o tronco do clipe). Rolando a pelve, o
    // quadril de fora subia e a perna de fora nao alcancava o chao: a de
    // dentro fazia 70% dos apoios, galopando - os saltinhos da curva.
    {
        const float wanted = gaitMoving ? CurveSpineShare * m_curveSprintShare * m_bodyCurveLean : 0.0f;
        m_spineCurveRoll += (wanted - m_spineCurveRoll) * (1.0f - std::exp(-deltaTime / 0.08f));
        if (std::abs(m_spineCurveRoll) > 1e-4f) {
            for (std::size_t index = 1; index < profile.links.size(); ++index) {
                const std::string& id = profile.links[index].id;
                if (id == "Abdomen" || id == "Chest" || id == "UpperChest")
                    sampled[index].z += CurveSpineSign * m_spineCurveRoll / 3.0f;
            }
        }
    }
    // Fim do levantar: coluna, pescoco e bracos vao aos poucos para a
    // postura parada (o primeiro quadro do idle, que e onde ele comeca). O
    // levantar de bruços termina ereto demais (tronco a ~2 graus, pes
    // juntos) e o idle tem o tronco a ~25 graus para a frente: depois da
    // troca o corpo ficava torto "encaixando" a diferenca. As pernas seguem
    // o clipe (os pes ficam onde estao e os passos refazem a base).
    if (standUpClip != nullptr && !m_tackling && animations.idle != nullptr) {
        const float duration = standUpClip->durationSeconds > 0.05f
            ? standUpClip->durationSeconds : 1.6f;
        const float toStance = smoothStep(
            (m_getUpClipSeconds / duration - 0.60f) / 0.40f);
        if (toStance > 0.0f) {
            for (std::size_t index = 1; index < profile.links.size()
                    && index < sampled.size(); ++index) {
                const std::string& id = profile.links[index].id;
                if (id.find("Thigh") != std::string::npos
                    || id.find("Shin") != std::string::npos
                    || id.find("Foot") != std::string::npos) continue;
                const AnimationTrack3D* track =
                    findAnimationTrack3D(*animations.idle, id);
                if (track == nullptr) continue;
                const Vec3 stance = sampleAnimationTrack3D(*track, 0.0f,
                    animations.idle->durationSeconds, animations.idle->loops)
                        .jointPositionRadians;
                sampled[index] += (stance - sampled[index]) * toStance;
            }
        }
    }

    // Preparo do carrinho: armado, tronco, bracos e cabeca ja vao para a
    // pose do comeco do clipe escolhido - no pouso a troca e pequena e o
    // mergulho sai da passada, fluido. As pernas seguem a passada.
    if (m_tacklePending && m_tackleClip != nullptr && standUpClip == nullptr) {
        const float w = TacklePreBlend * smoothStep(m_tacklePendingSeconds / 0.12f);
        for (std::size_t index = 1; index < profile.links.size() && index < sampled.size(); ++index) {
            const std::string& id = profile.links[index].id;
            if (id.find("Thigh") != std::string::npos || id.find("Shin") != std::string::npos
                || id.find("Foot") != std::string::npos) continue;
            const AnimationTrack3D* track = findAnimationTrack3D(*m_tackleClip, id);
            if (track == nullptr) continue;
            const Vec3 first = sampleAnimationTrack3D(*track, 0.0f, m_tackleClip->durationSeconds, false)
                .jointPositionRadians;
            sampled[index] += (first - sampled[index]) * w;
        }
    }

    // Quanto a pelve desce no pouso (modo antigo). No modo fisico o
    // agachamento vai direto na pelve alvo, mais abaixo.
    const auto landingSink = [&](float settle) {
        return input.forceDrivenRoot ? 0.0f : 0.08f * std::sin(Pi * settle);
    };
    float landingBraceForward = 0.0f;
    float landingBraceLateral = 0.0f;
    // Pulo autoral: o voo vem do clipe (acima). Na aterrissagem os pes sao
    // plantados pela trava de pe (contato forcado) e a pelve desce: e o IK
    // que dobra os joelhos, absorvendo, em vez de posicoes fixas de junta.
    // Sem clipes de pulo, o voo e a pose procedural de antes.
    if (jumpPose && desiredState == CharacterLocomotionState3D::Landing) {
        const float settle = std::clamp(
            m_stateSeconds / m_landingSettleSeconds, 0.0f, 1.0f);
        rootTranslation.z -= landingSink(settle);
        contactWeights = { 1.0f, 1.0f };
    } else if (jumpPose) {
        // Voo: nada a acrescentar.
    } else if (desiredState == CharacterLocomotionState3D::Airborne) {
        const float verticalVelocity = input.rootVelocityWorld.z;
        const float riseTuck = smoothStep(m_stateSeconds / 0.14f);
        // As descent speed builds, ease the tuck back open to meet the
        // ground with the legs already reaching for it.
        const float fallRelease = std::clamp(1.0f
            - std::max(0.0f, -verticalVelocity) * 0.10f, 0.30f, 1.0f);
        const float runningShare = smoothStep(m_liftoffSpeed / 2.2f);
        applyFlightPose(profile, sampled, riseTuck * fallRelease,
            runningShare);
    } else if (desiredState == CharacterLocomotionState3D::Landing) {
        const float settle = std::clamp(
            m_stateSeconds / m_landingSettleSeconds, 0.0f, 1.0f);
        const float compression = (1.0f - smoothStep(settle)) * 0.55f;
        const float runningShare = smoothStep(m_liftoffSpeed / 2.2f);
        applyFlightPose(profile, sampled, compression, runningShare);
    } else if (m_getUpPhase == CharacterGetUpPhase3D::Settling) {
        // Caido, em tres tempos. Primeiro ragdoll de verdade: o alvo das
        // juntas segue o proprio corpo com atraso (~0,2 s) e tonus baixo - ele
        // cede para onde a queda o leva, mas os musculos amortecem (nao e
        // corpo morto). Parado de vez, fica largado um instante. Depois, com o
        // tonus voltando, se arruma aos poucos ate a pose inicial do levantar
        // certo (de costas ou de bruços), de onde o levantar comeca sem salto.
        // Antes a pose de levantar era o alvo desde o primeiro instante: ele
        // caia ja "arrumado" para levantar.
        const AnimationClip3D* lying = m_fallOrientation
                == CharacterFallOrientation3D::FaceDown
            ? animations.standUpFront : animations.standUpBack;
        if (!m_settlePoseValid || m_settlePose.size() != sampled.size()) {
            m_settlePose.assign(sampled.size(), Vec3 {});
            m_settleGather = 0.0f;
            for (std::size_t link = 1;
                    link < sampled.size() && link < state.joints.size(); ++link) {
                m_settlePose[link] = { state.joints[link].positionRadians[0],
                    state.joints[link].positionRadians[1],
                    state.joints[link].positionRadians[2] };
            }
            m_settlePoseValid = true;
        }
        const bool gathering = m_settleRestSeconds > (m_getUpAttempt > 1
            ? RetryRestBeforeGather : SettleRestBeforeGather);
        // Atraso curto: o alvo que persegue o corpo atua como freio (mola x
        // atraso); com 0,2 s um antebraco em pe caia a ~0,1 rad/s.
        const float follow = gathering ? 0.0f
            : 1.0f - std::exp(-deltaTime / 0.04f);
        // Arrumando-se, acompanha o tonus; interrompido (um tranco no
        // tronco), volta a ceder em ~0,15 s, sem salto no alvo.
        const float gatherTarget = gathering ? smoothStep((m_settleTone
            - SettleRestingTone) / (1.0f - SettleRestingTone)) : 0.0f;
        m_settleGather = gatherTarget >= m_settleGather ? gatherTarget
            : m_settleGather + (gatherTarget - m_settleGather)
                * (1.0f - std::exp(-deltaTime / 0.15f));
        const float gather = m_settleGather;
        for (std::size_t link = 1;
                link < sampled.size() && link < state.joints.size(); ++link) {
            const Vec3 measured { state.joints[link].positionRadians[0],
                state.joints[link].positionRadians[1],
                state.joints[link].positionRadians[2] };
            m_settlePose[link] += (measured - m_settlePose[link]) * follow;
            // (Pelo corpo, deitado ele so recupera o tonus onde esta: quem
            // arruma a pose e o levantar.)
            const AnimationTrack3D* track = lying != nullptr && !GetUpBodily
                ? findAnimationTrack3D(*lying, profile.links[link].id) : nullptr;
            const Vec3 start = track != nullptr
                ? sampleAnimationTrack3D(*track, 0.0f, lying->durationSeconds,
                    false).jointPositionRadians
                : measured;
            sampled[link] = m_settlePose[link]
                + (start - m_settlePose[link]) * gather;
        }
        // Rolando (TEMPTUNE GetUpRoll): sobre a pose deitada, a perna de
        // cima dobra e cruza para o lado da barriga e o braco de cima alcanca
        // o chao por cima do peito - o peso vai para a frente e o corpo vira.
        // (No referencial da pelve: +x barriga, +y esquerda, +z cabeca.)
        if (m_getUpRolling && m_getUpRollTop >= 0) {
            const float weight = smoothStep(m_getUpRollSeconds / 0.4f);
            const int top = m_getUpRollTop;
            const float across = top == 0 ? -1.0f : 1.0f;  // para o outro lado
            const char* thighId = top == 0 ? "LeftThigh" : "RightThigh";
            const char* shinId = top == 0 ? "LeftShin" : "RightShin";
            const char* upperId = top == 0 ? "LeftUpperArm" : "RightUpperArm";
            const char* foreId = top == 0 ? "LeftForearm" : "RightForearm";
            const char* handId = top == 0 ? "LeftHand" : "RightHand";
            const std::size_t thigh = findLink(profile, thighId), shin = findLink(profile, shinId);
            const std::size_t upper = findLink(profile, upperId), fore = findLink(profile, foreId);
            const std::size_t hand = findLink(profile, handId);
            const auto anchorIn = [&](std::size_t child) {
                const RagdollLinkDefinition3D& link = profile.links[child];
                const RagdollLinkDefinition3D& parent = profile.links[static_cast<std::size_t>(link.parentIndex)];
                return parent.modelOrientation.conjugate().rotate(link.inboundJoint.anchorModelPosition - parent.modelPosition);
            };
            if (thigh > 0 && shin > 0 && upper > 0 && fore > 0 && hand > 0) {
                std::vector<Vec3> roll = sampled;
                RagdollAnimationPose3D frame;
                buildRagdollLocalPose3D(profile, roll, frame);
                // Joelho de cima a frente e para o outro lado.
                const Vec3 hip = frame.linkPositions[thigh]
                    + frame.linkOrientations[thigh].rotate(profile.links[thigh].modelOrientation.conjugate().rotate(
                        profile.links[thigh].inboundJoint.anchorModelPosition - profile.links[thigh].modelPosition));
                const Vec3 knee = hip + Vec3 { 0.28f, across * 0.12f, -0.27f };
                solveRagdollChainPoint3D(profile, roll, thigh, anchorIn(shin), 1, knee, 1.0f, 10);
                setComponent(roll[shin], 0, std::clamp(1.3f,
                    profile.links[shin].inboundJoint.axes[0].minimumRadians,
                    profile.links[shin].inboundJoint.axes[0].maximumRadians));
                // Mao de cima no chao, a frente do peito, do outro lado.
                buildRagdollLocalPose3D(profile, roll, frame);
                const Vec3 shoulder = frame.linkPositions[upper];
                const Vec3 reach = shoulder + Vec3 { 0.40f, across * 0.30f, -0.05f };
                solveRagdollChainPoint3D(profile, roll, fore, anchorIn(hand), 2, reach, 1.0f, 10);
                for (const std::size_t link : { thigh, shin, upper, fore })
                    sampled[link] += (roll[link] - sampled[link]) * weight;
            }
        }
    }

    // Preparacao de impacto pela velocidade real. Durante a descida o peito
    // vai contra o movimento horizontal: aterrissando para a frente, fica
    // atras da pelve; para a esquerda, desloca-se para a direita. Assim a
    // resultante do contato atravessa melhor a base e os joelhos podem
    // comprimir sem o tronco continuar tombando por inercia.
    if (!getUpActive && (desiredState == CharacterLocomotionState3D::Airborne
            || desiredState == CharacterLocomotionState3D::Landing)) {
        const Vec3 bodyVelocity = input.forceDrivenRoot && !state.links.empty()
            ? state.links.front().linearVelocity : input.rootVelocityWorld;
        const Vec3 localImpact = heading.conjugate().rotate(
            {bodyVelocity.x, bodyVelocity.y, 0.0f});
        const float horizontalSpeed = std::hypot(localImpact.x, localImpact.y);
        const float fallSpeed = desiredState == CharacterLocomotionState3D::Airborne
            ? std::max(0.0f, -bodyVelocity.z) : m_landingImpactSpeed;
        const float descent = desiredState == CharacterLocomotionState3D::Airborne
            ? smoothStep((m_flightPhase - 0.48f) / 0.34f)
                * smoothStep((fallSpeed - 0.35f) / 1.8f)
            : 1.0f - smoothStep(m_stateSeconds
                / std::max(0.08f, m_landingSettleSeconds));
        const float brace = std::clamp(horizontalSpeed
            * (0.025f + 0.007f * std::min(fallSpeed, 5.0f)), 0.0f, 0.30f)
            * descent;
        if (horizontalSpeed > 0.05f) {
            landingBraceForward = -brace * localImpact.x / horizontalSpeed;
            landingBraceLateral = -brace * localImpact.y / horizontalSpeed;
            addJointCoordinate(profile, sampled, "Abdomen", 1,
                landingBraceForward * 0.30f);
            addJointCoordinate(profile, sampled, "Chest", 1,
                landingBraceForward * 0.35f);
            addJointCoordinate(profile, sampled, "UpperChest", 1,
                landingBraceForward * 0.35f);
            addJointCoordinate(profile, sampled, "Abdomen", 2,
                landingBraceLateral * 0.30f);
            addJointCoordinate(profile, sampled, "Chest", 2,
                landingBraceLateral * 0.35f);
            addJointCoordinate(profile, sampled, "UpperChest", 2,
                landingBraceLateral * 0.35f);
        }
    }

    // Fim do levantar (pedido do usuario, 05/10 noite): os clipes terminam
    // com a coluna curvada (de bruços, ~28 graus somando abdomen, peito e
    // peito alto, a pose de alerta) e as pernas ja esticadas - de pe e
    // dobrado, ate a postura de pe entrar so no fim, de uma vez. Ja de pe
    // (a pelve alta e o clipe quase reto), a coluna vai a GetUpEndSpineShare
    // da flexao do clipe e o pescoco e a cabeca a extensao da postura de pe.
    if (m_getUpPhase == CharacterGetUpPhase3D::Rising && !m_tackling && !state.links.empty()) {
        const float ground = input.rootPositionWorld.z - profile.standingRootHeightMeters;
        const float height = state.links.front().position.z - ground;
        const float upright = smoothStep((0.8f - m_getUpExpectedTilt) / 0.4f)
            * smoothStep((height - 0.55f) / 0.25f) * GetUpEndStraighten;
        if (upright > 0.0f) {
            for (const char* id : { "Abdomen", "Chest", "UpperChest" }) {
                const std::size_t index = findLink(profile, id);
                if (index == 0 || index >= sampled.size()) continue;
                sampled[index].y *= 1.0f - (1.0f - GetUpEndSpineShare) * upright;
            }
            addJointCoordinate(profile, sampled, "Neck", 1, -IdleNeckExtension * upright);
            addJointCoordinate(profile, sampled, "Head", 1, -IdleHeadExtension * upright);
        }
    }
    const float transition = smoothStep(
        m_transitionSeconds / m_transitionDurationSeconds);
    const float transitionTime = std::clamp(
        m_transitionSeconds / m_transitionDurationSeconds, 0.0f, 1.0f);
    // Hermite outgoing tangent: preserve the motion at interruption and
    // release it continuously as the new clip takes over. Only authored
    // coordinates participate; IK and look must not feed back into it.
    const float tangentWeight = m_transitionDurationSeconds * transitionTime
        * (1.0f - transitionTime) * (1.0f - transitionTime);
    // Levantar a partir de onde o corpo esta: nos primeiros 0,35 s os alvos
    // saem das juntas medidas ao comecar, nao de onde a pose de se arrumar
    // queria que estivessem.
    const float risingFromMeasured = m_getUpPhase == CharacterGetUpPhase3D::Rising
            && m_risingFromMeasured.size() == profile.links.size()
        ? 1.0f - smoothStep(m_getUpPhaseSeconds / 0.35f) : 0.0f;
    for (std::size_t index = 1; index < profile.links.size(); ++index) {
        Vec3 wanted = m_transitionFrom[index] * (1.0f - transition)
            + sampled[index] * transition
            + m_transitionVelocity[index] * tangentWeight;
        if (risingFromMeasured > 0.0f)
            wanted += (m_risingFromMeasured[index] - wanted) * risingFromMeasured;
        for (std::size_t axis = 0; axis < 3; ++axis) {
            const RagdollAxisDefinition3D& limit =
                profile.links[index].inboundJoint.axes[axis];
            setComponent(wanted, axis, limit.enabled
                ? std::clamp(component(wanted, axis),
                    limit.minimumRadians, limit.maximumRadians)
                : 0.0f);
        }
        m_coordinates[index] = wanted;
        m_poseVelocities[index] = m_poseCoordinates.size() == m_coordinates.size()
            ? clampMagnitude((wanted - m_poseCoordinates[index]) / deltaTime, 30.0f)
            : Vec3 {};
    }
    m_poseCoordinates = m_coordinates;

    // O olhar. A pelve pode estar a ate ~105 graus da camera (andando de
    // lado, ou na borda do recuo); o personagem continua olhando para ela.
    // A coluna devolve a maior parte, ate 45 graus, e a cabeca completa, ate
    // 75 - o tronco fica perto da direcao do olhar e a cabeca nela. A
    // cabeca acompanha a camera rapido; o tronco vem atras num ritmo
    // proprio, mais lento, e um giro brusco de camera le como "a cabeca
    // olha, o corpo vem junto", nao como a coluna inteira girando de uma
    // vez no mesmo quadro.
    const float viewYawError = std::clamp(wrapAngle(lookYaw - m_facingYaw),
        -(TorsoTwistLimitRadians + HeadTurnLimitRadians),
        TorsoTwistLimitRadians + HeadTurnLimitRadians);
    const float headLookBlend = 1.0f - std::exp(-15.0f * deltaTime);
    const float torsoLookRate = 4.2f;
    const float torsoLookBlend = 1.0f - std::exp(-torsoLookRate * deltaTime);
    const float torsoLimit = desiredState == CharacterLocomotionState3D::Running
        ? SprintTorsoTwistLimitRadians : TorsoTwistLimitRadians;
    const float torsoTarget = std::clamp(viewYawError * TorsoLookShare,
        -torsoLimit, torsoLimit);
    m_torsoTwist += (torsoTarget - m_torsoTwist) * torsoLookBlend;
    const float headTarget = std::clamp(viewYawError - m_torsoTwist,
        -HeadTurnLimitRadians, HeadTurnLimitRadians);
    m_headTurn += (headTarget - m_headTurn) * headLookBlend;
    m_filteredLookPitch = m_lookPitch;
    // Filters keep running so they are not stale on handoff, but the
    // look-at twist itself only applies to the normal locomotion pose —
    // piling it on top of the get-up sequence's own carefully phased
    // spine/neck targets would just fight them.
    if (!getUpActive) {
        // Twist + gira a frente do segmento para a esquerda, na coluna e no
        // pescoco (medido pela FK em tools/animation). Divisao pelos
        // limites de cada junta: coluna 30/35/35, pescoco 40 e cabeca 60.
        addJointCoordinate(profile, m_coordinates, "Abdomen", 0,
            m_torsoTwist * 0.30f);
        addJointCoordinate(profile, m_coordinates, "Chest", 0,
            m_torsoTwist * 0.35f);
        addJointCoordinate(profile, m_coordinates, "UpperChest", 0,
            m_torsoTwist * 0.35f);
        addJointCoordinate(profile, m_coordinates, "Neck", 0,
            m_headTurn * 0.40f);
        addJointCoordinate(profile, m_coordinates, "Head", 0,
            m_headTurn * 0.60f);
        // Postura parado: a pose em alerta curva o peito superior ~23 graus e
        // o pescoco e a cabeca continuavam essa linha - a cabeca 7 cm a
        // frente da base do pescoco, corcunda. O pescoco e a cabeca estendem
        // (pescoco ~8 graus, cabeca quase reta, olhos na horizontal) e o
        // peito superior curva um pouco menos. Andando, metade.
        {
            const CharacterLocomotionState3D postureState = m_telemetry.state;
            const float uprightShare = postureState == CharacterLocomotionState3D::Idle
                    || postureState == CharacterLocomotionState3D::Turning ? 1.0f
                : postureState == CharacterLocomotionState3D::Walking
                    || postureState == CharacterLocomotionState3D::Running ? IdlePostureMovingShare : 0.0f;
            m_postureUpright += (uprightShare - m_postureUpright) * (1.0f - std::exp(-deltaTime / 0.3f));
            addJointCoordinate(profile, m_coordinates, "Neck", 1, -IdleNeckExtension * m_postureUpright);
            addJointCoordinate(profile, m_coordinates, "Head", 1, -IdleHeadExtension * m_postureUpright);
            addJointCoordinate(profile, m_coordinates, "UpperChest", 1, -IdleChestExtension * m_postureUpright);
            addJointCoordinate(profile, m_coordinates, "Chest", 1, -0.5f * IdleChestExtension * m_postureUpright);
        }
        addJointCoordinate(profile, m_coordinates, "UpperChest", 1,
            -m_filteredLookPitch * 0.14f);
        addJointCoordinate(profile, m_coordinates, "Neck", 1,
            -m_filteredLookPitch * 0.36f);
        addJointCoordinate(profile, m_coordinates, "Head", 1,
            -m_filteredLookPitch * 0.56f);
        // A coluna reage a pelve FISICA, nao apenas a inclinacao ideal da
        // animacao. Com ajuda parcial a pelve pode adiantar/tombar muito mais
        // que a referencia; a regra antiga via no maximo os poucos graus
        // pedidos pela arrancada e o tronco continuava soldado a ela.
        //
        // Esta e uma tarefa articular amortecida: a pelve continua livre e a
        // coluna devolve parte do erro para manter o peito proximo da vertical,
        // sem virar uma barra rigida nem apagar toda a inclinacao da corrida.
        Vec3 spineTarget {};
        if (input.forceDrivenRoot && !state.links.empty()) {
            const Vec3 pelvisUp = state.links.front().orientation.rotate({ 0.0f, 0.0f, 1.0f });
            const Vec3 headingForward = heading.rotate({ 1.0f, 0.0f, 0.0f });
            const Vec3 headingLeft = heading.rotate({ 0.0f, 1.0f, 0.0f });
            const float pelvisForward = std::asin(std::clamp(
                dot(pelvisUp, headingForward), -1.0f, 1.0f));
            const float pelvisSide = std::asin(std::clamp(
                dot(pelvisUp, headingLeft), -1.0f, 1.0f));
            spineTarget.x = std::clamp(-0.62f * pelvisForward, -0.24f, 0.24f)
                * startupProgress;
            spineTarget.y = std::clamp(-0.48f * pelvisSide, -0.20f, 0.20f);
            // Coluna viva na passada: os ombros inclinam ao contrario da pelve
            // (o tronco vai sobre a perna de apoio) e o tronco cede um pouco a
            // cada pisada, como mola. Os ciclos autorais tem a coluna quase
            // parada de lado (+-1 grau por segmento): medido correndo, ombros
            // x quadris inclinavam +-4,5 graus, contra +-8 (trote) e +-17
            // (sprint) nas referencias - parecia um bloco so.
            // So a parte rapida da rolagem da pelve (a da passada, ~2 Hz): a
            // lenta e a inclinacao de proposito (curva), que a coluna nao
            // pode desfazer - com ela toda o tronco da curva caia de 12 para
            // 2-7 graus.
            m_pelvisSideSlow += (pelvisSide - m_pelvisSideSlow) * (1.0f - std::exp(-deltaTime / 0.35f));
            if (gaitMoving) {
                const float pace = smoothStep((speed - 3.0f) / 3.5f);
                const float rollGain = SpineRollGainSlow + (SpineRollGainFast - SpineRollGainSlow) * pace;
                spineTarget.y = std::clamp(-0.48f * m_pelvisSideSlow
                    - rollGain * (pelvisSide - m_pelvisSideSlow), -SpineRollLimit, SpineRollLimit);
                // Na pisada a pelve freia para baixo (aceleracao para cima):
                // o tronco flexiona um pouco e volta.
                const float impact = std::max(0.0f, m_estimator.state().centerOfMassAccelerationWorld.z) / 9.81f;
                spineTarget.x += std::min(SpineImpactFlex * impact, 0.12f);
            }
        }
        const Vec3 spineError = m_spineBalance - spineTarget;
        const Vec3 spineImpulse = m_spineBalanceVelocity + spineError * 12.0f;
        const float spineDecay = std::exp(-12.0f * deltaTime);
        m_spineBalance = spineTarget
            + (spineError + spineImpulse * deltaTime) * spineDecay;
        m_spineBalanceVelocity = (m_spineBalanceVelocity
            - spineImpulse * (12.0f * deltaTime)) * spineDecay;
        const float spineForward = m_spineBalance.x;
        const float spineSide = m_spineBalance.y;
        addJointCoordinate(profile, m_coordinates, "Abdomen", 1, spineForward * 0.30f);
        addJointCoordinate(profile, m_coordinates, "Chest", 1, spineForward * 0.35f);
        addJointCoordinate(profile, m_coordinates, "UpperChest", 1, spineForward * 0.35f);
        addJointCoordinate(profile, m_coordinates, "Abdomen", 2, spineSide * 0.30f);
        addJointCoordinate(profile, m_coordinates, "Chest", 2, spineSide * 0.35f);
        addJointCoordinate(profile, m_coordinates, "UpperChest", 2, spineSide * 0.35f);
        // Na arrancada o peito acompanha a massa para a frente; a
        // contra-inclinacao que estabiliza a marcha entra conforme o corpo
        // ganha velocidade. No tropeco a inercia tambem continua levando o
        // tronco para a frente, como na referencia estudada: o passo longo e
        // a pelve baixa e que o capturam, nao uma extensao instantanea.
        const float startupChest = startupChestAmount;
        const Vec3 tripLocal = heading.conjugate().rotate(m_tripDirection);
        const float tripForward = 0.12f * StumbleTrunkScale * m_tripRecovery * tripLocal.x;
        const float tripSide = -0.10f * m_tripRecovery * tripLocal.y;
        addJointCoordinate(profile, m_coordinates, "Abdomen", 1,
            (startupChest + tripForward) * 0.30f);
        addJointCoordinate(profile, m_coordinates, "Chest", 1,
            (startupChest + tripForward) * 0.35f);
        addJointCoordinate(profile, m_coordinates, "UpperChest", 1,
            (startupChest + tripForward) * 0.35f);
        addJointCoordinate(profile, m_coordinates, "Abdomen", 2, tripSide * 0.30f);
        addJointCoordinate(profile, m_coordinates, "Chest", 2, tripSide * 0.35f);
        addJointCoordinate(profile, m_coordinates, "UpperChest", 2, tripSide * 0.35f);
        if (m_tripFoot >= 0 && m_tripRecovery > 0.0f) {
            const char* thigh = m_tripFoot == 0 ? "LeftThigh" : "RightThigh";
            const char* shin = m_tripFoot == 0 ? "LeftShin" : "RightShin";
            const float elevation = m_tripElevating ? 1.0f : 0.45f;
            addJointCoordinate(profile, m_coordinates, thigh, 1,
                -0.24f * m_tripRecovery * elevation);
            addJointCoordinate(profile, m_coordinates, shin, 0,
                0.55f * m_tripRecovery * elevation);
        }
    }

    // A freada especial e uma recuperacao de emergencia, nao a animacao
    // normal de parar. Ela so aparece quando ainda existe momento fisico de
    // corrida maxima E a intencao desacelera forte. No trote e em trajetos
    // curtos, o ciclo apenas encurta os passos e assenta: projetar um pe
    // muito a frente nessas condicoes parece uma derrapagem sem causa.
    //
    // A reacao continua sendo uma mola de pose e nao segura a capsula. Vale
    // em qualquer direcao: o sentido vem do movimento de entrada.
    {
        if (speed > 0.3f) m_brakeDirection = desiredPlanarVelocity / speed;
        m_brakeEntrySpeed = std::max(speed,
            m_brakeEntrySpeed - 2.5f * deltaTime);
        const bool braceable = !getUpActive && input.grounded
            && desiredState != CharacterLocomotionState3D::Airborne
            && desiredState != CharacterLocomotionState3D::JumpStarting;
        const float deceleration = std::max(0.0f,
            -dot(m_smoothedAcceleration, m_brakeDirection));
        const Vec3 physicalPlanarVelocity { m_filteredCenterOfMassVelocity.x,
            m_filteredCenterOfMassVelocity.y, 0.0f };
        const float physicalMomentum = std::max(0.0f,
            dot(physicalPlanarVelocity, m_brakeDirection));
        // Comeca apenas perto da corrida maxima. Os dois portoes evitam que
        // uma oscilacao de velocidade do trote seja confundida com freada.
        const float sprintMomentum = smoothStep(
            (std::min(m_brakeEntrySpeed, physicalMomentum) - 5.5f) / 1.0f);
        const float hardDeceleration = smoothStep((deceleration - 8.0f) / 8.0f);
        // Freando pela passada: pela velocidade que ainda falta tirar (do
        // trote, metade; do sprint, inteira).
        const float gaitBrake = m_brakeGait * BrakeReactionScale
            * smoothStep((m_brakeGaitSpeed - 1.0f) / 4.5f);
        const float intensity = braceable
            ? std::max(sprintMomentum * hardDeceleration, gaitBrake)
            : 0.0f;
        const Vec3 drive = heading.conjugate().rotate(
            m_brakeDirection * intensity);
        constexpr float BrakeOmega = 11.0f, BrakeDamping = 0.55f;
        m_brakeReactionVelocity += (BrakeOmega * BrakeOmega
                * (Vec3 { drive.x, drive.y, 0.0f } - m_brakeReaction)
            - 2.0f * BrakeDamping * BrakeOmega * m_brakeReactionVelocity)
            * deltaTime;
        m_brakeReaction += m_brakeReactionVelocity * deltaTime;
        m_brakeReaction = clampMagnitude(m_brakeReaction, 1.2f);
        m_telemetry.brakeReaction = std::hypot(m_brakeReaction.x,
            m_brakeReaction.y);
        m_telemetry.brakeReactionForward = m_brakeReaction.x;
    }
    if (!getUpActive) {
        // Tronco para onde a inercia leva (flexao + para a frente,
        // inclinacao + para a esquerda), a cabeca devolvendo parte para o
        // olhar ficar no horizonte, e os cotovelos abrindo.
        // Freando: o tronco estende um pouco para tras junto com a pelve (a
        // cabeca devolve, o olhar fica no horizonte), nao cai para a frente.
        const float flexion = -0.15f * m_brakeReaction.x;
        const float lateral = -0.08f * m_brakeReaction.y;
        const float amount = m_telemetry.brakeReaction;
        addJointCoordinate(profile, m_coordinates, "Abdomen", 1,
            flexion * 0.30f);
        addJointCoordinate(profile, m_coordinates, "Chest", 1,
            flexion * 0.35f);
        addJointCoordinate(profile, m_coordinates, "UpperChest", 1,
            flexion * 0.35f);
        addJointCoordinate(profile, m_coordinates, "Abdomen", 2,
            lateral * 0.30f);
        addJointCoordinate(profile, m_coordinates, "Chest", 2,
            lateral * 0.35f);
        addJointCoordinate(profile, m_coordinates, "UpperChest", 2,
            lateral * 0.35f);
        addJointCoordinate(profile, m_coordinates, "Neck", 1,
            -flexion * 0.40f);
        addJointCoordinate(profile, m_coordinates, "Head", 1,
            -flexion * 0.25f);
        addJointCoordinate(profile, m_coordinates, "LeftForearm", 0,
            -0.35f * std::min(1.0f, amount));
        addJointCoordinate(profile, m_coordinates, "RightForearm", 0,
            -0.35f * std::min(1.0f, amount));
    }
    // Reflexos de queda por cima da pose (e, no comeco do deitado, por cima
    // da pose deitada: as maos ainda seguram o impacto e depois relaxam).
    {
        float reflex = m_fallReflex;
        float legReflex = std::min(m_fallReflex, m_fallCommit);
        if (m_getUpPhase == CharacterGetUpPhase3D::Settling) {
            // No chao os reflexos da queda (que miram o rumo dela, sem
            // sentido rolando no chao - os alvos pulavam e ele parecia
            // convulsionar) saem em 0,3 s; quem assume e a posicao fetal
            // (abaixo).
            reflex *= GroundAlive ? 1.0f - smoothStep(m_getUpPhaseSeconds / 0.3f)
                : 1.0f - smoothStep(m_getUpPhaseSeconds / 0.6f);
            legReflex = reflex;
        } else if (m_getUpPhase == CharacterGetUpPhase3D::Rising) {
            reflex = 0.0f;
            legReflex = 0.0f;
        }
        applyFallReflexPose(profile, state, m_coordinates, m_handLinks,
            m_chestLink, m_fallDirection, groundHeightGuess, reflex, legReflex,
            m_handCatchSeconds, m_elapsedSeconds, input.varietySeed);
        // Vida: esperneando no ar caindo (sem tocar nada ha ~0,1 s).
        const float flail = m_getUpPhase == CharacterGetUpPhase3D::None
            ? m_fallReflex * smoothStep((m_noGroundSeconds - 0.10f) / 0.20f)
            : 0.0f;
        m_airBalance = AirBalanceNew ? flail : 0.0f;
        // O giro do corpo (no eixo de lado do peito): sentido e forca do moinho.
        float bodySpin = 0.0f;
        if (m_chestLink < state.links.size()) {
            const Vec3 chestLeft = state.links[m_chestLink].orientation.rotate({ 0.0f, 1.0f, 0.0f });
            bodySpin = dot(state.links[m_chestLink].angularVelocity, chestLeft);
        }
        if (AirBalanceNew) applyAirBalance(profile, m_coordinates, m_elapsedSeconds, flail, bodySpin);
        else applyAirborneFlail(profile, m_coordinates, m_elapsedSeconds, flail);
        // No chao: posicao fetal enquanto ainda nao se arruma para levantar
        // (entra em ~0,3 s e da lugar a pose de levantar quando ele se
        // arruma).
        // (De costas ou de lado; de bruços ele nao se enrola sobre a barriga
        // - protege a cabeca com os bracos e apoia as maos, que ja e a pose de
        // onde o levantar de bruços parte. Enrolado de bruços, o levantar
        // tinha de desmontar a pose e levava 8,6 s em vez de ~5.)
        const float fetalWanted = FetalEnabled && m_getUpPhase == CharacterGetUpPhase3D::Settling
                && m_fallOrientation == CharacterFallOrientation3D::FaceUp
            ? 1.0f - m_settleGather : 0.0f;
        m_fetal += (fetalWanted - m_fetal) * (1.0f - std::exp(-deltaTime / 0.12f));
        {
            float coreSpeedNow = state.links.empty() ? 0.0f : state.links.front().linearVelocity.length();
            applyFetalPose(profile, state, m_coordinates, m_handLinks, m_chestLink,
                smoothStep(m_fetal) * (m_getUpPhase == CharacterGetUpPhase3D::Settling ? 1.0f : 0.0f),
                m_elapsedSeconds, input.varietySeed, std::clamp(coreSpeedNow / 1.5f, 0.0f, 1.0f));
        }
        m_telemetry.lifeFlail = flail;
        // Perdendo o equilibrio, de pe: inclinado alem da pose (a partir de
        // ~12 graus) ou atingido (contato externo recente, o corpo indo para
        // onde o jogador nao pediu). Sai quando o reflexo de queda assume.
        float balanceWanted = 0.0f;
        Vec3 balanceDirection = m_balanceDirection;
        if (m_getUpPhase == CharacterGetUpPhase3D::None && !state.links.empty()
            && desiredState != CharacterLocomotionState3D::Airborne) {
            const Vec3 up = state.links.front().orientation.rotate({ 0.0f, 0.0f, 1.0f });
            Vec3 lean = up - m_previousTargetUp;
            lean.z = 0.0f;
            const float tiltError = std::acos(std::clamp(
                dot(up, m_previousTargetUp), -1.0f, 1.0f));
            const float tiltTerm = smoothStep((tiltError - 0.21f) / 0.35f);
            // Movimento nao pedido: ir mais devagar (ou bloqueado) no sentido
            // pedido nao conta - correndo contra uma parede ele nao gira os
            // bracos; ser jogado de lado, para tras ou estando parado, conta.
            Vec3 bodyVelocity = m_filteredCenterOfMassVelocity;
            bodyVelocity.z = 0.0f;
            Vec3 wanted = input.proxyVelocityWorld;
            wanted.z = 0.0f;
            Vec3 drift = bodyVelocity;
            if (wanted.length() > 0.2f) {
                const Vec3 along = wanted.normalized();
                drift -= along * std::clamp(dot(bodyVelocity, along), 0.0f,
                    wanted.length() + 0.5f);
            }
            const float hitTerm = std::clamp(input.externalPerturbation, 0.0f, 1.0f)
                * smoothStep((drift.length() - 0.3f) / 1.0f);
            balanceWanted = std::max(tiltTerm, hitTerm) * (1.0f - m_fallReflex);
            Vec3 direction {};
            if (lean.lengthSquared() > 1e-6f) direction += lean.normalized() * tiltTerm;
            if (drift.lengthSquared() > 1e-6f) direction += drift.normalized() * hitTerm;
            if (direction.lengthSquared() > 1e-6f) balanceDirection = direction.normalized();
            // Tropeco: os bracos saem do balanco comum e abrem/giram para
            // administrar o momento. O reflexo existente gera essa resposta
            // e continua livre para virar protecao das maos se o passo falhar.
            if (m_tripRecovery > balanceWanted) {
                balanceWanted = std::max(balanceWanted,
                    0.75f * m_tripRecovery) * (1.0f - m_fallReflex);
                balanceDirection = m_tripDirection;
            }
            // Quase-queda em movimento: os bracos abrem para administrar o
            // momento (referencia Jogging Stumble: mao a altura do ombro, 37
            // cm para o lado).
            if (StumbleArmsScale * m_stumbleReaction > balanceWanted) {
                balanceWanted = StumbleArmsScale * m_stumbleReaction * (1.0f - m_fallReflex);
                balanceDirection = m_stumbleDirection;
            }
        }
        m_balanceReaction += (balanceWanted - m_balanceReaction) * (1.0f
            - std::exp(-deltaTime * (balanceWanted > m_balanceReaction ? 12.0f : 3.0f)));
        {
            const Vec3 blended = m_balanceDirection + (balanceDirection - m_balanceDirection)
                * (1.0f - std::exp(-8.0f * deltaTime));
            if (blended.lengthSquared() > 1e-6f) m_balanceDirection = blended.normalized();
        }
        std::array<bool, 2> braced {};
        for (const RagdollInteraction3D& contact : state.interactions) {
            if (contact.normalWorld.z <= 0.3f || contact.linkIndex >= profile.links.size()) continue;
            for (std::size_t side = 0; side < 2; ++side) {
                const std::size_t hand = m_handLinks[side];
                if (hand == 0 || hand >= profile.links.size()) continue;
                if (contact.linkIndex == hand || static_cast<int>(contact.linkIndex)
                        == profile.links[hand].parentIndex) braced[side] = true;
            }
        }
        const float gaitReaction = gaitMoving ? BalanceGaitShare : 1.0f;
        applyBalanceReaction(profile, state, m_coordinates, m_handLinks,
            m_chestLink, m_balanceDirection, m_balanceReaction * gaitReaction, m_elapsedSeconds,
            braced, m_balanceReaction * (gaitMoving && BalanceGaitNoLegs ? 0.0f : gaitReaction));
        m_telemetry.balanceReaction = m_balanceReaction;
    }
    // Chute: bracos e tronco do chute por cima.
    applyKickPose(profile, state, m_coordinates, m_handLinks, input.ballKick);
    // Manejo de bola (rework de 06/10): tronco, bracos e cabeca do gesto por
    // cima (os pes vao pelo planejador: ballFoot/ballScript).
    if (input.ballGesture.active && !state.links.empty()) {
        if (!m_gestureLegs.valid) m_gestureLegs = resolveBallGestureLegs3D(profile);
        Vec3 rootPosition = state.links.front().position;
        Quaternion rootOrientation = state.links.front().orientation;
        applyBallGesturePose3D(profile, m_gestureLegs, input.ballGesture, m_coordinates, rootPosition,
            rootOrientation, false, false);
    }

    // Banking into a turn: a runner carving a curve leans into it and
    // leans forward a little more, roughly in line with the lean/twist
    // magnitudes measured off a real running-turn reference (calibrated by
    // hand against that capture, not sampled from it — see
    // assets/animations/source/mixamo/study/README.txt).
    // v * yawRate is centripetal acceleration. Small curves get a soft
    // onset, and rotating in place does not bank like a running turn.
    const float curveAcceleration = signedCurveAcceleration;
    const float curveShare = smoothStep(std::abs(curveAcceleration) / 3.0f);
    const float turnBank = std::atan(curveAcceleration / 9.81f)
        * 0.72f * curveShare;
    // Freando, a pelve inclina para tras so 15% do que inclinaria (a
    // inclinacao satura: com 40%, uma freada de sprint ainda dava os 12,6
    // graus inteiros): o tronco vai para a frente pela inercia (bloco da
    // freada, acima).
    // Freando, a pelve inclina para tras na proporcao da desaceleracao (ate
    // ~9 graus): o corpo fica atras dos pes para frear (era 15% disso, e o
    // tronco ia para a frente pela inercia).
    const Vec3 brakingAcceleration = m_brakeDirection
        * std::min(0.0f, dot(m_smoothedAcceleration, m_brakeDirection));
    (void)brakingAcceleration;
    const Vec3 localAcceleration = heading.conjugate().rotate(m_smoothedAcceleration);
    // Arrancando, inclina menos (ate ~7 graus, era 12,6: somado a pose da
    // corrida, o peito passava de 30 graus no comeco do trote/sprint).
    const float wantedForwardLean = std::clamp(
        std::atan(localAcceleration.x / 9.81f) * 0.22f
            + std::abs(turnBank) * 0.08f
            + startupRootLean,
        -0.16f, 0.20f);
    // De lado (curva, troca de rumo) pela velocidade: andando e trotando,
    // pouco; so correndo rapido a inclinacao inteira (ate ~11 graus, era 15
    // em qualquer velocidade: mexendo a camera andando ele tombava de lado).
    const float bankSpeedShare = 0.25f + 0.75f * smoothStep((speed - 2.5f) / 4.0f);
    // Curva: o corpo inteiro inclina para dentro como a fisica pede, theta =
    // atan(a_c/g) pela aceleracao centripeta do caminho (na referencia
    // Sprint Turn, do pe de apoio ao quadril: 24 graus a 4,1 m/s2 - atan
    // 23). O pe pousa para fora (bloco do pouso), a pelve rola uma parte
    // (11-19 graus la) e o tronco vai junto com ela. Andando devagar quase
    // nada: girando a camera andando ele tombava de lado.
    // (Pela taxa do caminho, que so existe com pedido e sem freada: a
    // desaceleracao forte, um pouco desalinhada da velocidade, parecia curva
    // - 5 graus de 12 m/s2 sao ~1 m/s2 de lado - e ele inclinava e jogava o
    // pe para fora nos passos de freada.)
    m_bodyCurveLean = CurveNew ? std::clamp(std::atan(pathSpeed * pathTurnRate / 9.81f),
        -CurveLeanMaximum, CurveLeanMaximum) * smoothStep((speed - 1.2f) / 1.6f) : 0.0f;
    // (A pelve rola menos no sprint, TEMPTUNE CurveRootSprint: rolando, o
    // quadril de fora subia e a perna de fora nao alcancava o chao - a de
    // dentro fazia 70% dos apoios, galopando. A parte que a pelve deixa vai
    // pela coluna (bloco do tronco). No trote ela rola inteira: menos, o
    // joelho de apoio dobrava 7 graus a mais na curva.)
    m_curveSprintShare = smoothStep((speed - CurveRootSprintFrom) / std::max(0.1f, CurveRootSprintSpan));
    const float curveRootShare = CurveRootShare + (CurveRootSprint - CurveRootShare) * m_curveSprintShare;
    const float wantedLateralLean = CurveNew
        ? std::clamp(curveRootShare * m_bodyCurveLean, -0.30f, 0.30f)
        : std::clamp(
        (std::atan(localAcceleration.y / 9.81f) * 0.18f + turnBank) * bankSpeedShare,
        -0.19f, 0.19f);
    // Exact critically damped spring: continuous lean velocity on turn
    // reversal without delaying the capsule or changing running speed.
    const Vec3 leanTarget { wantedForwardLean, wantedLateralLean, 0.0f };
    const Vec3 leanError = m_lean - leanTarget;
    const Vec3 leanImpulse = m_leanVelocity + leanError * 22.0f;
    const float leanDecay = std::exp(-22.0f * deltaTime);
    m_lean = leanTarget + (leanError + leanImpulse * deltaTime) * leanDecay;
    m_leanVelocity = (m_leanVelocity - leanImpulse * (22.0f * deltaTime)) * leanDecay;
    // Longitudinal acceleration is already filtered upstream. A second
    // lag here moves the hip behind the foot plan when descending steps.
    float forwardLean = wantedForwardLean;
    float lateralLean = m_lean.y;
    // v2 (T_LeanNeed): com pouca ajuda quem acelera o corpo sao as pernas, e
    // o chao so empurra para a frente sem girar o corpo com o centro de massa
    // a frente do apoio (a inclinacao atan(a/g)). A referencia inclinava so
    // pela aceleracao da capsula (22% dela) - presa ao corpo, quase nada: o
    // tronco ficava reto e o corpo nao passava de ~1,5 m/s sem ajuda. Inclina
    // pela aceleracao que falta (o pedido menos o corpo, no que as pernas
    // conseguem), na parte das pernas (1 - ajuda andando).
    {
        static const float leanNeedGain = tuneTemp("T_LeanNeed", 0.0f);  // TEMPTUNE
        static const float leanNeedRate = tuneTemp("T_LeanNeedRate", 1.5f);  // TEMPTUNE (1/s)
        static const float leanNeedMaximum = tuneTemp("T_LeanNeedMax", 5.0f);  // TEMPTUNE (m/s2)
        static const float leanNeedPower = tuneTemp("T_LeanNeedPower", 18.0f);  // TEMPTUNE (W/kg)
        const bool gaitNow = desiredState == CharacterLocomotionState3D::Walking
            || desiredState == CharacterLocomotionState3D::Running;
        Vec3 wanted {};
        if (leanNeedGain > 0.0f && input.forceDrivenRoot && gaitNow && !getUpActive) {
            const Vec3 requested { input.intent.requestedVelocityWorld.x, input.intent.requestedVelocityWorld.y, 0.0f };
            const Vec3 body { m_filteredCenterOfMassVelocity.x, m_filteredCenterOfMassVelocity.y, 0.0f };
            Vec3 need = clampMagnitude((requested - body) * leanNeedRate, leanNeedMaximum);
            // Ganhar velocidade correndo rapido custa mais (a potencia): ~3
            // m/s2 a 6 m/s.
            const float speed = body.length();
            if (speed > 0.5f) {
                const Vec3 unit = body * (1.0f / speed);
                const float along = dot(need, unit);
                if (along > 0.0f) {
                    const float limit = leanNeedPower / std::max(1.0f, speed);
                    if (along > limit) need -= unit * (along - limit);
                }
            }
            const float legs = 1.0f - std::clamp(input.walkingAssist, 0.0f, 1.0f);
            const Vec3 local = heading.conjugate().rotate(need) * (legs * leanNeedGain);
            wanted = { std::atan(local.x / 9.81f), std::atan(local.y / 9.81f), 0.0f };
        }
        m_leanNeed += (wanted - m_leanNeed) * (1.0f - std::exp(-deltaTime / 0.12f));
        forwardLean = std::clamp(forwardLean + m_leanNeed.x, -0.30f, 0.45f);
        lateralLean = std::clamp(lateralLean + m_leanNeed.y, -0.35f, 0.35f);
    }

    // A direcao vem do referencial da locomocao. Dos ciclos (autorais, que
    // olham para +X) fica a rotacao inteira da pelve - inclinacao, queda do
    // quadril e o giro de ate 8 graus que acompanha a perna que avanca. Do
    // levantar (importado, com rumo arbitrario) sai o rumo da fonte e fica
    // so a inclinacao.
    // Rumo da fonte estavel deitado: frente + cima * (-frente.z). Em pe e a
    // frente; de bruços (frente para o chao) e de costas (frente para o
    // ceu) a frente no plano some e, passando de 90 graus, virava 180 graus
    // de um quadro para o outro - o cima completa (de costas, para os pes).
    const Vec3 sourceForward = rootRotation.rotate({ 1.0f, 0.0f, 0.0f });
    const Vec3 sourceUp = rootRotation.rotate({ 0.0f, 0.0f, 1.0f });
    const Quaternion sourceHeading = Quaternion::fromAxisAngle(
        { 0.0f, 0.0f, 1.0f }, std::atan2(sourceForward.y - sourceUp.y * sourceForward.z,
            sourceForward.x - sourceUp.x * sourceForward.z));
    // (No carrinho o giro do clipe fica: ele desliza de lado em relacao a
    // direcao do carrinho, e o clipe ja vem no rumo do comeco.)
    const Quaternion clipRootTilt = standUpClip != nullptr && !m_tackling
        ? (sourceHeading.conjugate() * rootRotation).normalized()
        : rootRotation.normalized();
    // Mesmo crossfade das juntas (ver `transition` acima). Trocar de clipe
    // nunca pode saltar a pelve: o guia deriva velocidade por diferenca
    // finita, e um salto vira velocidade angular aplicada ao corpo inteiro.
    Quaternion tiltTarget = clipRootTilt;
    if (m_transitionFromRootTilt.x * tiltTarget.x
            + m_transitionFromRootTilt.y * tiltTarget.y
            + m_transitionFromRootTilt.z * tiltTarget.z
            + m_transitionFromRootTilt.w * tiltTarget.w < 0.0f) {
        tiltTarget = { -tiltTarget.x, -tiltTarget.y, -tiltTarget.z,
            -tiltTarget.w };
    }
    const Quaternion blendedRootTilt = Quaternion {
        m_transitionFromRootTilt.x
            + (tiltTarget.x - m_transitionFromRootTilt.x) * transition,
        m_transitionFromRootTilt.y
            + (tiltTarget.y - m_transitionFromRootTilt.y) * transition,
        m_transitionFromRootTilt.z
            + (tiltTarget.z - m_transitionFromRootTilt.z) * transition,
        m_transitionFromRootTilt.w
            + (tiltTarget.w - m_transitionFromRootTilt.w) * transition
    }.normalized();
    const Quaternion rootTilt = (exponentialRotation(
        m_transitionRootTiltVelocity * tangentWeight) * blendedRootTilt).normalized();
    m_rootTiltVelocity = clampMagnitude(rotationVector(
        rootTilt * m_rootTilt.conjugate()) / deltaTime, 10.0f);
    m_rootTilt = rootTilt;
    {
        const Vec3 tiltForward = rootTilt.rotate({ 1.0f, 0.0f, 0.0f });
        m_clipPelvisYaw = std::atan2(tiltForward.y, tiltForward.x);
    }
    const Quaternion accelerationTilt = (
        Quaternion::fromAxisAngle({ 0.0f, 1.0f, 0.0f }, forwardLean)
        * Quaternion::fromAxisAngle({ 1.0f, 0.0f, 0.0f }, -lateralLean))
            .normalized();
    // A recovery plays where the body is lying, not where the capsule is
    // standing: the anchor was captured when the body settled, and the
    // clip's own root track carries it from the floor back to its feet.
    const bool risingPose = m_getUpPhase == CharacterGetUpPhase3D::Rising;
    const Quaternion targetRootOrientation = risingPose
        ? (Quaternion::fromAxisAngle({ 0.0f, 0.0f, 1.0f }, m_getUpAnchorYaw)
            * rootTilt).normalized()
        : (heading * accelerationTilt * rootTilt).normalized();
    m_previousTargetUp = targetRootOrientation.rotate({ 0.0f, 0.0f, 1.0f });
    // Freando, a base da referencia no plano nao e a capsula: ela freia a
    // 18 m/s2 e vem arrastada ate 0,5-0,7 m atras do corpo, e a referencia
    // presa a ela fazia o pe pousar embaixo do corpo ou atras.
    //  - Correndo e freando: uma referencia com estado, que anda no perfil
    //    de freada e fica presa ao corpo por uma coleira - a capsula de uma
    //    freada que o corpo consegue cumprir.
    //  - Nos passos de freada: o proprio corpo (a pelve medida menos o
    //    balanco do clipe); quem freia sao os pes.
    Vec3 navigationRoot = input.rootPositionWorld;
    if (m_brakeGait > 0.0f && !state.links.empty()) {
        const Vec3 pelvis = state.links.front().position;
        const Vec3 sway = heading.rotate({ m_rootOffset.x, m_rootOffset.y, 0.0f });
        const Vec3 body { pelvis.x - sway.x, pelvis.y - sway.y, 0.0f };
        if (!m_brakeReferenceValid) {
            m_brakeReferencePosition = { input.rootPositionWorld.x, input.rootPositionWorld.y, 0.0f };
            m_brakeReferenceValid = true;
        }
        m_brakeReferencePosition += Vec3 { brakeNavigationVelocity.x, brakeNavigationVelocity.y, 0.0f }
            * deltaTime;
        const Vec3 away = m_brakeReferencePosition - body;
        // Com pouca ajuda o corpo nao volta para a referencia (ninguem o
        // freia na pelve): 30 cm atras dele, todos os alvos dos pes ficavam
        // atras e ele mergulhava por cima deles. Fica presa ao corpo.
        const float lowAssistReference = smoothStep((0.5f - std::clamp(input.walkingAssist, 0.0f, 1.0f)) / 0.4f);
        const float leash = BrakeReferenceLeash + (LowAssistBrakeReferenceLeash - BrakeReferenceLeash) * lowAssistReference;
        if (away.length() > leash) m_brakeReferencePosition = body + away * (leash / away.length());
        m_brakeAnchor = m_brakeStepping ? std::min(1.0f, m_brakeAnchor + deltaTime / 0.15f)
            : std::max(0.0f, m_brakeAnchor - deltaTime / 0.15f);
        const Vec3 braking = m_brakeReferencePosition + (body - m_brakeReferencePosition) * m_brakeAnchor;
        navigationRoot.x += (braking.x - navigationRoot.x) * m_brakeGait;
        navigationRoot.y += (braking.y - navigationRoot.y) * m_brakeGait;
    } else {
        m_brakeReferenceValid = false;
        m_brakeAnchor = 0.0f;
    }
    // Andando, a referencia da passada fica presa ao corpo com uma folga
    // curta no rumo do movimento: com pouca ajuda o corpo ficava para tras e
    // a capsula corria ~0,3 m a frente - cada pe pousava (a partir dela) 0,3
    // m longe demais, freava o corpo, ele inclinava para tras e caia. A
    // folga e o que a perna de apoio empurra.
    // (Cresce conforme a ajuda cai: com a ajuda cheia a pelve leva o corpo
    // ate a capsula e nada muda.)
    const float lowAssist = smoothStep((0.5f - std::clamp(input.walkingAssist, 0.0f, 1.0f)) / 0.4f);
    if (GaitLeashEnabled && lowAssist > 0.0f && input.forceDrivenRoot && gaitMoving && m_brakeGait <= 0.0f
        && !getUpActive && !state.links.empty()) {
        const Vec3 pelvis = state.links.front().position;
        const Vec3 sway = heading.rotate({ m_rootOffset.x, m_rootOffset.y, 0.0f });
        const Vec3 body { pelvis.x - sway.x, pelvis.y - sway.y, 0.0f };
        const Vec3 lead { navigationRoot.x - body.x, navigationRoot.y - body.y, 0.0f };
        Vec3 leashed = lead;
        if (leashed.length() > GaitLeashMeters) leashed = leashed * (GaitLeashMeters / leashed.length());
        navigationRoot.x = body.x + lead.x + (leashed.x - lead.x) * lowAssist;
        navigationRoot.y = body.y + lead.y + (leashed.y - lead.y) * lowAssist;
    }
    // Parado depois de uma freada, a base da referencia fica onde o corpo
    // parou: a capsula vem ate 10 cm atras dele (a folga com que ela segue o
    // corpo) e, com a referencia voltando para ela, as pernas puxavam o corpo
    // para tras ate ela - ele escorregava e a recuperacao de parado dava dois
    // ou tres passos para tras. Com o pedido de movimento (ou depois de um
    // empurrao) volta para a capsula.
    if (!state.links.empty()) {
        const Vec3 pelvis = state.links.front().position;
        const Vec3 sway = heading.rotate({ m_rootOffset.x, m_rootOffset.y, 0.0f });
        const Vec3 body { pelvis.x - sway.x, pelvis.y - sway.y, 0.0f };
        // Com os pes assentados (o planejador diz que o ponto de captura ja
        // cabe na base), a base passa a frear o resto da velocidade: as
        // pernas seguram o corpo sobre os pes (tornozelo e quadril), em vez
        // de deixa-lo seguir a 0,5 m/s ate um passo de recuperacao.
        const bool settled = m_brakeGaitActive && m_brakeStepping
            && m_footwork.plan().brakeSettled;
        const bool leaving = input.intent.requestedDirectionWorld.lengthSquared() > 0.01f
            || getUpActive || input.externalPerturbation > 0.2f
            || (m_brakeGaitActive && !settled);
        if (!leaving && !m_stopHolding
            && (settled || (m_wasBrakeStepping && !m_brakeGaitActive))) {
            m_stopRoot = body;
            m_stopVelocity = { m_filteredCenterOfMassVelocity.x, m_filteredCenterOfMassVelocity.y, 0.0f };
            m_stopRootBlend = 1.0f;
            m_stopHolding = true;
        }
        if (leaving) m_stopHolding = false;
        m_wasBrakeStepping = m_brakeGaitActive && m_brakeStepping;
        m_stopRootBlend = leaving ? std::max(0.0f, m_stopRootBlend - deltaTime / 0.25f) : m_stopRootBlend;
        if (m_stopRootBlend > 0.0f) {
            const float speed = m_stopVelocity.length();
            const float slower = std::max(0.0f, speed - StopHoldDeceleration * deltaTime);
            m_stopVelocity = speed > 1e-4f ? m_stopVelocity * (slower / speed) : Vec3 {};
            m_stopRoot += m_stopVelocity * deltaTime;
            // Presa ao corpo com a mesma folga do proxy.
            const Vec3 away = m_stopRoot - body;
            if (away.length() > StopHoldLeash) m_stopRoot = body + away * (StopHoldLeash / away.length());
            navigationRoot.x += (m_stopRoot.x - navigationRoot.x) * m_stopRootBlend;
            navigationRoot.y += (m_stopRoot.y - navigationRoot.y) * m_stopRootBlend;
        }
    }
    // Giro parado pelo clipe: a raiz anda o que a do clipe anda.
    navigationRoot += m_clipTurnRootOffset * m_clipTurnWeight;
    Vec3 targetRootPosition = risingPose
        ? Vec3 { m_getUpAnchorWorld.x, m_getUpAnchorWorld.y,
            input.rootPositionWorld.z }
        : navigationRoot;
    // Tropecando, abaixa o centro de massa enquanto a perna livre encurta e
    // passa por cima. A referencia estudada faz isso de forma muito grande;
    // aqui usamos apenas o necessario para manter agilidade e alcance.
    if (!risingPose) {
        targetRootPosition.z -= (StumbleDetection ? StumbleDrop : 0.065f) * m_tripRecovery;
        // Passo de quase-queda: a pelve desce (referencia Jogging Stumble:
        // o quadril ~30 cm mais baixo no passo que salva).
        if (StumbleStepTrigger > 0.0f && !m_startStepActive)
            targetRootPosition.z -= StumbleStepDrop * (m_stumbleStepping ? std::max(0.5f, m_stumbleReaction) : 0.0f);
        // Arrancada pelo passo: agacha enquanto o pe de tras vem (referencia
        // Idle To Sprint: o quadril 13 cm mais baixo nos primeiros passos).
        if (m_startStepActive)
            targetRootPosition.z -= StartStepDrop * smoothStep(m_stumbleStepSeconds / 0.12f);
        // A compressao precede a arrancada: joelhos/quadris recebem carga
        // antes da extensao, como na referencia, em vez de o peito saltar
        // imediatamente para a inclinacao final. O trote usa menos da metade
        // da descida observada; o sprint ainda fica abaixo dos ~17 cm medidos.
        targetRootPosition.z -= startupPelvisDrop;
        // Preparo do carrinho: armado, ele baixa um pouco antes de mergulhar.
        if (m_tacklePending)
            targetRootPosition.z -= TackleGatherDrop * smoothStep(m_tacklePendingSeconds / 0.15f);
    }
    // Rebase: a base da raiz troca de referencial (levantar comeca/termina)
    // ou salta num tick (teletransporte, ancora refeita). A diferenca entre
    // as duas bases nao e movimento pedido - era ela que virava milhares de
    // m/s de "velocidade" na saida do levantar do boneco solto.
    // Toda troca de fase do deitado/levantar tambem: caindo, a altura da
    // referencia passa a ser a da pelve deitada (-98 m/s num tick).
    const bool referenceRebase = risingPose != m_previousRisingPose
        || m_getUpPhase != m_previousReferencePhase
        || (Vec3 { targetRootPosition.x - m_previousRootBase.x,
               targetRootPosition.y - m_previousRootBase.y, 0.0f }).length()
            > 0.25f;
    m_previousRisingPose = risingPose;
    m_previousReferencePhase = m_getUpPhase;
    m_previousRootBase = targetRootPosition;
    const bool airbornePose = desiredState == CharacterLocomotionState3D::JumpStarting
        || desiredState == CharacterLocomotionState3D::Airborne;
    // Deslocamento da raiz vindo do clipe: a altura (a pelve sobe e desce
    // com a passada) e o balanco para o pe de apoio. No referencial do clipe
    // e no MESMO tempo das pernas, com o crossfade das juntas nas trocas.
    //
    // Era filtrado (16/s). A altura ficava atrasada em relacao as pernas, o
    // pe de apoio do alvo afundava no chao e o aterramento (mais abaixo)
    // empurrava a pelve de volta num tick: dente de serra de ate 2,6 cm em
    // 8 ms no sprint - o tremor que se via no corpo andando.
    //
    // O personagem controlado tem a capsula como ancora. No boneco solto,
    // XY vem da pelve fisica: somar balanco ali realimentaria a posicao.
    // Sua altura de repouso usa a base nominal e o ajuste de alcance dos
    // pes. Levantar precisa da trilha vertical para chegar aos pes.
    const bool planarAllowed = !risingPose && !airbornePose
        && input.controlled;
    const bool verticalAllowed = risingPose
        || (input.controlled && !airbornePose);
    const Vec3 authoredOffset {
        planarAllowed ? rootTranslation.x : 0.0f,
        planarAllowed ? rootTranslation.y : 0.0f,
        verticalAllowed ? rootTranslation.z : 0.0f };
    const Vec3 nextRootOffset = m_transitionFromRootOffset
        + (authoredOffset - m_transitionFromRootOffset) * transition
        + m_transitionRootOffsetVelocity * tangentWeight;
    m_rootOffsetVelocity = clampMagnitude((nextRootOffset - m_rootOffset) / deltaTime, 3.0f);
    m_rootOffset = nextRootOffset;
    // Deitado (fisico), a altura da referencia e a do corpo: ela ficava na de
    // pe e, ao comecar o levantar, despencava ate a do clipe em ~0,1 s
    // (referencia a -7,6 m/s). Assim o levantar parte de onde a pelve esta.
    if (input.forceDrivenRoot && m_getUpPhase == CharacterGetUpPhase3D::Settling
        && !state.links.empty()) {
        m_rootOffset.z = state.links.front().position.z - input.rootPositionWorld.z;
        m_rootOffsetVelocity.z = state.links.front().linearVelocity.z;
    }
    targetRootPosition += heading.rotate({ m_rootOffset.x, m_rootOffset.y,
        0.0f });
    targetRootPosition.z += m_rootOffset.z;
    // Carrinho: no plano, o caminho do clipe (na escala da entrada) a partir
    // da ancora, no rumo do carrinho.
    if (m_tackling && standUpClip != nullptr) {
        const Vec3 path = Quaternion::fromAxisAngle({ 0.0f, 0.0f, 1.0f }, m_tackleYaw).rotate({
            rootTranslation.x * m_tackleDistanceScale, rootTranslation.y * m_tackleDistanceScale, 0.0f });
        targetRootPosition.x = m_tackleStart.x + path.x;
        targetRootPosition.y = m_tackleStart.y + path.y;
    }
    // Postura mais ereta (pedido do usuario, 03/10): os clipes e o controle
    // deixavam a pelve 7-10 cm abaixo da de pe, "sempre semi-agachado". A
    // pelve-alvo sobe; os pes estao travados, entao o IK estica os joelhos.
    // So de pe e andando/correndo (agachado, no ar, aterrissando e
    // levantando, nao).
    // Preparo do pulo (agachar segurando o espaco): so parado. Em movimento
    // (passada ou movimento pedido) nada - correndo, o agachamento travava
    // as pernas; ao soltar, o pulo sai no padrao.
    const bool movingForJump = gaitMoving
        || std::hypot(input.intent.requestedDirectionWorld.x, input.intent.requestedDirectionWorld.y) > 0.1f;
    const float jumpCrouchInput = movingForJump ? 0.0f : std::clamp(input.jumpCrouch, 0.0f, 1.0f);
    {
        const float bodySpeed = std::hypot(m_filteredCenterOfMassVelocity.x,
            m_filteredCenterOfMassVelocity.y);
        const float running = smoothStep((bodySpeed - 0.6f) / 1.6f);
        const float sprinting = smoothStep((bodySpeed - 4.0f) / 2.0f);
        const bool upright = !getUpActive && !airbornePose
            && (desiredState == CharacterLocomotionState3D::Idle
                || desiredState == CharacterLocomotionState3D::Turning
                || desiredState == CharacterLocomotionState3D::Walking
                || desiredState == CharacterLocomotionState3D::Running);
        const float wanted = upright
            ? (PostureLiftIdle + (PostureLiftRun - PostureLiftIdle) * running
                + (PostureLiftSprint - PostureLiftRun) * sprinting)
                * (1.0f - jumpCrouchInput)
            : 0.0f;
        m_postureLift += (wanted - m_postureLift) * (1.0f - std::exp(-deltaTime / 0.25f));
        targetRootPosition.z += m_postureLift;
    }
    // Freada: a pelve desce com a reacao (as pernas dobram pelo IK dos pes).
    // Freando pelos passos, o agachamento vem antes do pe de freada pousar:
    // um pe a 0,4-0,5 m a frente do quadril so alcanca o chao com a pelve
    // 15-20 cm mais baixa (na referencia Run To Stop ela desce 25 cm). Sem
    // isso o pe ficava pendurado procurando o chao e pousava tarde e perto.
    {
        float wanted = m_brakeStepping && !getUpActive && !airbornePose
            ? BrakeCrouchMeters
                * smoothStep((m_brakeGaitSpeed - 0.6f) / BrakeCrouchSpeedSpan)
            : 0.0f;
        // Invertendo, ele agacha para frear e girar sobre o pe, ate a
        // arrancada para o outro lado (na referencia a pelve desce 33 cm e
        // fica embaixo enquanto a velocidade troca de sentido).
        if (m_brakeReversing && !getUpActive && !airbornePose)
            wanted = std::max(wanted, PivotCrouchMeters);
        const float omega = BrakeCrouchOmega;
        m_brakeCrouchVelocity += (omega * omega * (wanted - m_brakeCrouch)
            - 2.0f * omega * m_brakeCrouchVelocity) * deltaTime;
        m_brakeCrouch = std::max(0.0f, m_brakeCrouch + m_brakeCrouchVelocity * deltaTime);
    }
    if (!getUpActive && !airbornePose) {
        // Inclinado na curva, o quadril fica mais perto do chao.
        m_telemetry.pelvisDropCurveMeters = CurveDropShare * CurveLeanHeight * (1.0f - std::cos(m_bodyCurveLean));
        m_telemetry.pelvisDropBrakeMeters = std::max(m_brakeCrouch, 0.12f * std::min(1.0f,
            m_telemetry.brakeReaction));
        targetRootPosition.z -= m_telemetry.pelvisDropCurveMeters;
        targetRootPosition.z -= m_telemetry.pelvisDropBrakeMeters;
        // Carregando o pulo: agacha (as pernas dobram pelo IK dos pes). So
        // parado: em movimento nao ha preparo (o pulo sai no padrao).
        m_jumpCrouch += (jumpCrouchInput - m_jumpCrouch)
            * (1.0f - std::exp(-12.0f * deltaTime));
        // (Mais fundo carregado: as referencias descem 19 cm no pulo normal e
        // 45 no forte.)
        targetRootPosition.z -= JumpCrouchDepth * m_jumpCrouch;
    } else {
        m_jumpCrouch = 0.0f;
        m_telemetry.pelvisDropCurveMeters = m_telemetry.pelvisDropBrakeMeters = 0.0f;
    }
    // Pouso no modo fisico: a pelve de referencia cede junto com o corpo no
    // toque (nunca fica acima dele empurrando - era isso que o fazia quicar:
    // no toque ela estava 20 cm acima e a ajuda de altura ia ao maximo), vai
    // ao agachamento previsto para o impacto, segura e sobe devagar. Direto
    // na pelve alvo: pelo crossfade da pose ele chegava tarde.
    if (input.forceDrivenRoot && !getUpActive && !airbornePose
        && m_sinceTouchdownSeconds < m_landingSettleSeconds
        && !state.links.empty()) {
        const float settle = std::clamp(m_sinceTouchdownSeconds
            / m_landingSettleSeconds, 0.0f, 1.0f);
        // TEMPTUNE LandRun: pousando correndo (o jogador segue pedindo a
        // corrida), as pernas sao mola - agacham o previsto, menos, e nao
        // acompanham o corpo que afunda (a pelve alvo descendo com ele, nada
        // segurava: 30 cm abaixo, joelho a 130 graus, e o tronco tombava).
        const float runningLanding = LandRunYield > 0.0f ? smoothStep((std::hypot(
            m_filteredCenterOfMassVelocity.x, m_filteredCenterOfMassVelocity.y) - 2.5f) / 2.0f)
            * smoothStep(std::hypot(input.intent.requestedVelocityWorld.x,
                input.intent.requestedVelocityWorld.y) / 1.5f) : 0.0f;
        const float planned = std::clamp(0.035f * m_landingImpactSpeed,
            0.04f, 0.22f) * (1.0f - (1.0f - LandRunDepth) * runningLanding);
        const float yielded = std::min(std::clamp(targetRootPosition.z
            - state.links.front().position.z, 0.0f, 0.25f),
            0.25f + (planned * LandRunYield - 0.25f) * runningLanding);
        if (settle < 0.45f) {
            m_landingSinkPeak = std::max({ m_landingSinkPeak, yielded,
                planned * smoothStep(settle / 0.15f) });
        }
        m_telemetry.pelvisDropLandingMeters = m_landingSinkPeak
            * (1.0f - smoothStep((settle - 0.45f) / 0.55f));
        targetRootPosition.z -= m_telemetry.pelvisDropLandingMeters;
    } else {
        m_landingSinkPeak = 0.0f;
        m_telemetry.pelvisDropLandingMeters = 0.0f;
    }

    RagdollAnimationPose3D localPose;
    buildLocalPose(profile, m_coordinates, localPose);
    if (m_feet.size() >= 2 && m_feet[0].valid && m_feet[1].valid
        && m_feet[0].linkIndex < localPose.linkPositions.size()
        && m_feet[1].linkIndex < localPose.linkPositions.size()) {
        m_poseFeetCenterLocal = (localPose.linkPositions[m_feet[0].linkIndex]
            + localPose.linkPositions[m_feet[1].linkIndex]) * 0.5f;
        m_poseFeetCenterValid = true;
    }
    for (std::size_t side = 0; side < m_feet.size(); ++side) {
        const FootPlant& foot = m_feet[side];
        if (!foot.valid
            || foot.linkIndex >= localPose.linkOrientations.size()) {
            continue;
        }
        const Quaternion footOrientation = (rootTilt
            * localPose.linkOrientations[foot.linkIndex]).normalized();
        const Vec3 footForward = footOrientation.rotate(
            profile.links[foot.linkIndex].modelOrientation
                .conjugate().rotate({ 1.0f, 0.0f, 0.0f }));
        const float footYaw = std::atan2(footForward.y, footForward.x);
        m_clipPelvisToFootYaw[side] = wrapAngle(
            m_clipPelvisYaw - footYaw);
    }

    // =====================================================================
    // Pes no terreno.
    //
    // Cada pe tem o seu chao. No apoio o pe fica travado no mundo, plano sobre
    // o chao que a sonda achou embaixo dele (calcanhar, meio e ponta; vale o
    // mais alto, para o pe nunca afundar na quina de um degrau). No balanco
    // ele sai da altura de onde levantou e chega na altura de onde VAI pisar:
    // o proprio clipe diz quando e onde o pe toca (o proximo contato), a
    // velocidade da capsula diz onde a pelve estara, e a sonda mede o chao ali
    // antes de o pe chegar. Subindo, o pe ganha a altura cedo e com folga;
    // descendo, so depois de passar a quina. A pelve desce o que for preciso
    // para o pe mais baixo alcancar o chao dele.
    //
    // O ciclo acompanha a distancia que a capsula anda: o ritmo tem limites
    // e o que falta ou sobra vira passo mais curto ou mais longo (stride
    // warping). Parado, se a pelve se afasta de um pe travado (parou no meio
    // da passada, levou um esbarrao), o pe da um passo - o corpo nao anda
    // sem as pernas andarem.
    // =====================================================================
    const float capsuleBase = groundHeightGuess;
    // Degrau da capsula suavizado: o controlador sobe e desce degrau num
    // salto de altura; a pelve vai junto, mas em ~0,1 s.
    if (m_baseTracked && input.grounded && m_baseWasGrounded && !getUpActive) {
        const float jump = capsuleBase - m_previousBaseGround;
        if (std::abs(jump) > 0.025f && std::abs(jump) < 0.6f) {
            m_baseStepOffset -= jump;
        }
    }
    if (!input.grounded || getUpActive) m_baseStepOffset = 0.0f;
    m_baseStepOffset *= std::exp(-12.0f * deltaTime);
    m_previousBaseGround = capsuleBase;
    m_baseTracked = true;
    m_baseWasGrounded = input.grounded;
    targetRootPosition.z += m_baseStepOffset;
    // The spawn may have clearance above the floor. Preserve its initial
    // height, then settle onto the guide before IK, rather than snapping
    // the entire articulated body down on the first physics tick.
    if (!m_spawnHeightCaptured) {
        m_spawnHeightOffset = state.links.front().position.z - targetRootPosition.z;
        m_spawnHeightCaptured = true;
    }
    targetRootPosition.z += m_spawnHeightOffset
        * (1.0f - smoothStep(std::max(0.0f, m_elapsedSeconds - deltaTime) / 0.30f));
    const float baseGround = capsuleBase + m_baseStepOffset;

    // Caindo sem volta, os pes nao miram passos: estao nos reflexos. So
    // desequilibrado, continuam dando passos (e podem salvar o equilibrio).
    const bool footworkEnabled = !getUpActive && m_fallCommit < 0.5f;
    const bool onGround = input.grounded && !airbornePose;
    struct GroundSample {
        bool valid = false;
        float height = 0.0f;
        Vec3 normal { 0.0f, 0.0f, 1.0f };
    };
    const auto groundUnder = [&](std::size_t side, Vec3 center,
        Vec3 forward) -> GroundSample {
        GroundSample result;
        if (input.groundAt) {
            // A sonda procura a partir da altura do chao da capsula (quem
            // implementa groundAt comeca um pouco acima dela): uma viga na
            // altura do peito nao vira chao.
            center.z = baseGround;
            forward.z = 0.0f;
            forward = forward.lengthSquared() > 0.000001f
                ? forward.normalized() : heading.rotate({ 1.0f, 0.0f, 0.0f });
            for (const float reach : { -0.09f, 0.0f, 0.09f }) {
                const GroundProbeResult3D probe =
                    input.groundAt(center + forward * reach);
                if (!probe.hasSurface || !probe.walkable) continue;
                // Evaluate the surface plane at the sole center: selecting
                // the highest toe sample adds artificial height on ramps.
                const float height = probe.pointWorld.z
                    - (probe.normalWorld.x * (center.x - probe.pointWorld.x)
                        + probe.normalWorld.y * (center.y - probe.pointWorld.y))
                        / std::max(0.2f, probe.normalWorld.z);
                if (!result.valid || height > result.height) {
                    result.valid = true;
                    result.height = height;
                    result.normal = probe.normalWorld;
                }
            }
            if (result.valid) return result;
        }
        const GroundProbeResult3D& legacy = input.footGround[side];
        result.valid = true;
        if (legacy.hasSurface) {
            result.height = legacy.pointWorld.z;
            result.normal = legacy.normalWorld;
        } else {
            result.height = baseGround;
        }
        return result;
    };
    // Pe inteiro num nivel: calcanhar, meio e ponta na mesma altura. Numa
    // quina de degrau (o pe tem 28 cm, o degrau tambem), o ponto de pouso
    // anda o minimo ao longo do pe ate caber inteiro num degrau.
    const auto fitFoot = [&](Vec3 center, Vec3 forward) -> Vec3 {
        if (!input.groundAt) return {};
        forward.z = 0.0f;
        if (forward.lengthSquared() < 0.000001f) return {};
        forward = forward.normalized();
        const auto spread = [&](Vec3 at, float& highest) {
            float low = std::numeric_limits<float>::infinity();
            highest = -std::numeric_limits<float>::infinity();
            for (const float reach : { -0.12f, 0.0f, 0.12f }) {
                Vec3 point = at + forward * reach;
                point.z = baseGround;
                const GroundProbeResult3D probe = input.groundAt(point);
                if (!probe.hasSurface) return 1.0f;
                low = std::min(low, probe.pointWorld.z);
                highest = std::max(highest, probe.pointWorld.z);
            }
            return highest - low;
        };
        float level = 0.0f;
        if (spread(center, level) < 0.02f) return {};
        Vec3 best {};
        float bestLevel = -std::numeric_limits<float>::infinity();
        for (float distance = 0.02f; distance <= 0.20f; distance += 0.02f) {
            for (const float sign : { 1.0f, -1.0f }) {
                float height = 0.0f;
                const Vec3 shift = forward * (sign * distance);
                if (spread(center + shift, height) < 0.015f
                    && height > bestLevel) {
                    best = shift;
                    bestLevel = height;
                }
            }
            if (std::isfinite(bestLevel)) return best;
        }
        return {};
    };
    // Rumo do clipe para o pe, sem a inclinacao de aceleracao: o pe de apoio
    // fica com a sola no chao, nao inclinado junto com o corpo.
    const auto clipFootOrientation = [&](std::size_t linkIndex) {
        return (heading * rootTilt * localPose.linkOrientations[linkIndex])
            .normalized();
    };
    const auto lockHeight = [&](const FootPlant& foot, float ground,
        Quaternion orientation) {
        const RagdollLinkDefinition3D& link = profile.links[foot.linkIndex];
        const Quaternion collider = (orientation
            * link.collider.localOrientation).normalized();
        return ground + lowestColliderOffset(link, collider)
            - orientation.rotate(link.collider.localPosition).z + 0.001f;
    };
    const auto forwardOf = [&](const FootPlant& foot, Quaternion orientation) {
        return orientation.rotate(profile.links[foot.linkIndex]
            .modelOrientation.conjugate().rotate({ 1.0f, 0.0f, 0.0f }));
    };
    const ClipFootEvents* events = gaitMoving && selectedClip != nullptr
        ? &footEventsFor(profile, *selectedClip) : nullptr;
    const float clipTravel = selectedClip != nullptr
        ? selectedClip->travelDirectionRadians : 0.0f;
    const Vec3 travelAxisLocal = (accelerationTilt * rootTilt).conjugate()
        .rotate({ std::cos(clipTravel), std::sin(clipTravel), 0.0f });
    Vec3 planarVelocity = Vec3 { input.rootVelocityWorld.x,
        input.rootVelocityWorld.y, 0.0f } * gaitVelocityScale;
    if (gaitStarting && speed > 0.001f)
        planarVelocity *= gaitCycleSpeed / speed;
    planarVelocity += (brakeNavigationVelocity - planarVelocity) * m_brakeGait;

    struct FootGoal {
        bool active = false;
        Vec3 position;
        Quaternion orientation;
        float pelvisGround = 0.0f;
        // Quanto este pe conta para a pelve alcanca-lo: 1 apoiado ou
        // pousando, subindo no fim do balanco, 0 no ar.
        float reach = 0.0f;
        // O pe do clipe, sem o desnivel do terreno: a pelve so desce o que o
        // terreno pede a mais que a animacao.
        Vec3 animatedPosition;
        Quaternion animatedOrientation;
        // Pe em balanco: onde vai pousar (tornozelo, no mundo), quanto o
        // quadril anda ate la e a perna do clipe no toque. A pelve comeca a
        // descer antes do toque; descendo o ultimo degrau, o pe pousava
        // fora do alcance da perna e a trava soltava e voltava.
        float anticipate = 0.0f;
        Vec3 landingPosition;
        Quaternion landingOrientation;
        Vec3 hipTravel;
        float authoredLandingHorizontal = 0.0f;
        float authoredLandingVertical = 0.0f;
        // Teto da descida que este pe pede (m): o pe no ar, correndo no
        // plano, pede pouco.
        float dropCap = std::numeric_limits<float>::infinity();
    };
    std::array<FootGoal, 2> goals {};

    // A perna ainda alcanca o pe apoiado? Descendo escada o corpo segue e o
    // degrau de tras fica longe: segurar a trava deixava o pe pendurado no
    // limite do IK (26 cm acima do degrau), marcado como apoiado. Tambem: o
    // IK nao chegou la no tick anterior (limite de junta).
    const auto legOutOfReach = [&](const FootPlant& foot) {
        const RagdollLinkDefinition3D& link = profile.links[foot.linkIndex];
        const int shin = link.parentIndex;
        const int thigh = shin > 0 ? profile.links[
            static_cast<std::size_t>(shin)].parentIndex : -1;
        if (thigh <= 0) return foot.reachError > 0.06f;
        const RagdollLinkDefinition3D& shinLink =
            profile.links[static_cast<std::size_t>(shin)];
        const RagdollLinkDefinition3D& thighLink =
            profile.links[static_cast<std::size_t>(thigh)];
        const float legLength = (shinLink.inboundJoint.anchorModelPosition
                - thighLink.inboundJoint.anchorModelPosition).length()
            + (link.inboundJoint.anchorModelPosition
                - shinLink.inboundJoint.anchorModelPosition).length();
        const Vec3 hip = targetRootPosition + Vec3 { 0.0f, 0.0f, m_pelvisOffset }
            + targetRootOrientation.rotate(profile.links[0].modelOrientation
                .conjugate().rotate(thighLink.inboundJoint.anchorModelPosition
                    - profile.links[0].modelPosition));
        const Vec3 ankle = foot.plantedGoalPosition
            + foot.plantedGoalOrientation.rotate(link.modelOrientation.conjugate()
                .rotate(link.inboundJoint.anchorModelPosition - link.modelPosition));
        return (hip - ankle).length() > legLength * 1.01f || foot.reachError > 0.06f;
    };

    // Pes: o planejador unico de passos (ContactFootwork3D) decide e executa
    // os passos com o corpo parado - acomodacao, giro no lugar, recuperacao
    // de empurrao/pressao/arrasto, refazer a base depois de levantar - e so
    // pousa um pe por contato medido. Andando, a passada do clipe (abaixo)
    // ainda decide os pes; o planejador acompanha para assumir sem salto.
    // Parado de fato: sem passada e sem o jogador pedindo movimento. Na
    // troca de direcao (strafe para um lado e para o outro) a capsula passa
    // por velocidade zero num tick; tomar isso por "parado" entregava os dois
    // pes ao planejador no meio da passada.
    const bool movementRequested =
        input.intent.requestedDirectionWorld.lengthSquared() > 0.01f;
    const bool standing = footworkEnabled && !gaitMoving && !movementRequested
        && input.grounded
        && desiredState != CharacterLocomotionState3D::Landing
        && !airbornePose;
    // Quase-queda correndo: o planejador da um passo rapido que pega o corpo
    // e devolve os pes a passada (ver ContactFootworkInput3D::stumbling).
    // Pelo ciclo do clipe nao dava: pelve baixa, pouso adiantado, cadencia
    // ou a perna mirando pela pelve de verdade, todos derrubavam mais no pe
    // enganchado em sprint (26 -> 36-50 em 128) - o passo do clipe tem o
    // tempo e o tamanho da corrida, nao o de quem se salva.
    m_sinceStumbleStep = std::min(10.0f, m_sinceStumbleStep + deltaTime);
    {
        const float bodySpeed = std::hypot(m_filteredCenterOfMassVelocity.x, m_filteredCenterOfMassVelocity.y);
        const bool allowed = StumbleStepTrigger > 0.0f && footworkEnabled && onGround && gaitMoving
            && input.forceDrivenRoot && !getUpActive && m_fallCommit < 0.5f
            && m_brakeGait <= 0.0f && !m_brakeGaitActive;
        const Vec3 requestedDirection { input.intent.requestedDirectionWorld.x, input.intent.requestedDirectionWorld.y, 0.0f };
        const bool startAllowed = StartStepEnabled && footworkEnabled && onGround && gaitMoving
            && input.forceDrivenRoot && !getUpActive && m_fallCommit < 0.5f && !m_brakeGaitActive
            && m_gaitDirection == CharacterGaitDirection3D::Forward && requestedDirection.lengthSquared() > 0.01f;
        if (m_startStepRequested && !m_stumbleStepping && startAllowed && bodySpeed < 1.0f) {
            m_stumbleStepping = true;
            m_startStepActive = true;
            m_stumbleStepSeconds = 0.0f;
            m_stumbleStepDirection = requestedDirection.normalized();
        } else if (!m_stumbleStepping && allowed && m_stumbleRisk >= StumbleStepTrigger && bodySpeed > 1.5f
            && m_sinceStumbleStep > StumbleStepCooldown) {
            m_stumbleStepping = true;
            m_startStepActive = false;
            m_stumbleStepSeconds = 0.0f;
            m_stumbleStepDirection = m_stumbleDirection;
        } else if (m_stumbleStepping && !(m_startStepActive ? startAllowed : allowed)) {
            m_stumbleStepping = false;
        }
        m_startStepRequested = false;
        if (!m_stumbleStepping) m_startStepActive = false;
        if (m_stumbleStepping) m_stumbleStepSeconds += deltaTime;
    }
    {
        ContactFootworkInput3D footInput;
        footInput.physical = &physical;
        footInput.state = &state;
        footInput.standing = standing;
        footInput.allowSteps = footworkEnabled && onGround;
        // Recuperacao so no modo fisico, sem queda comprometida e ha mais de
        // 0,8 s de pe (recem-levantado, a base se refaz pelos passos de
        // acomodacao, um pe de cada vez). Segurado pela PhysGun tambem:
        // arrastado, ele da passos.
        footInput.allowRecovery = m_fallCommit < 0.5f
            && m_sinceGetUpSeconds > 0.8f;
        m_telemetry.footworkStanding = standing;
        m_telemetry.footworkAllowSteps = footInput.allowSteps;
        m_telemetry.footworkAllowRecovery = footInput.allowRecovery;
        m_telemetry.captureDirectionWorld = m_captureDirection;
        footInput.seedFromBody = m_plantFeetFromBody && onGround && footworkEnabled;
        footInput.facingYawRadians = m_facingYaw;
        footInput.standingTurnTargetYawRadians = m_standingTurnTarget;
        footInput.standingTurnRateRadiansPerSecond = m_turningRate;
        footInput.pelvisYawOffsetRadians = m_clipPelvisYaw;
        footInput.heading = heading;
        footInput.referenceRootWorld = targetRootPosition;
        footInput.capturePointWorld = m_capturePoint;
        footInput.captureOutsideMeters = m_captureOutside;
        footInput.captureDirectionWorld = m_captureDirection;
        footInput.planningComVelocityWorld = m_filteredCenterOfMassVelocity;
        footInput.locomoting = movementRequested;
        footInput.desiredVelocityWorld = input.intent.requestedVelocityWorld;
        footInput.braking = m_brakeStepping && footworkEnabled && onGround;
        // Recem-freado (a base ainda e a da freada, as vezes com o corpo na
        // borda de tras dela): acomodar ou girar tirava do chao justo o pe que
        // segurava o corpo.
        // (Ate a camera girar de verdade: ai o giro parado volta a valer.)
        // Ajuda cheia segurando (sem perturbacao de fora): o equilibrio e dela.
        footInput.assistHold = 0.0f;
        // (Com ela, a base se fecha logo depois da freada: a espera existia
        // porque o corpo podia estar na borda de tras da base.)
        footInput.settleHold = m_sinceBrakeSeconds < BrakeSettleHoldSeconds
            && std::abs(wrapAngle(m_lookYaw - m_facingYaw)) < 0.55f
            && footInput.assistHold < 0.5f;
        footInput.quietStop = (m_brakeGaitActive || m_sinceBrakeSeconds < BrakeSettleHoldSeconds)
            && input.externalPerturbation < 0.2f;
        footInput.brakeDirectionWorld = m_brakeGaitDirection;
        footInput.brakeDecelerationMetersPerSecondSquared = BrakeDeceleration;
        footInput.stumbling = m_stumbleStepping && !footInput.braking;
        footInput.stumbleDirectionWorld = m_stumbleStepDirection;
        footInput.stumbleAheadMeters = m_startStepActive ? StartStepAhead
            : StumbleStepAhead;
        footInput.stumbleSwingSeconds = m_startStepActive ? StartStepSwing
            : StumbleStepSwing;
        footInput.stumbleStepDelaySeconds = m_startStepActive ? StartPushSeconds : 0.0f;
        footInput.stumbleLiftMeters = m_startStepActive ? StartStepLift : 0.0f;
        for (std::size_t side = 0; side < m_feet.size(); ++side) {
            const FootPlant& foot = m_feet[side];
            if (!foot.valid) continue;
            footInput.reference[side].poseWorld = targetRootPosition
                + targetRootOrientation.rotate(localPose.linkPositions[foot.linkIndex]);
            footInput.reference[side].poseOrientationWorld =
                clipFootOrientation(foot.linkIndex);
            footInput.anchorUnreachable[side] = foot.planted && legOutOfReach(foot);
            m_telemetry.footworkAnchorUnreachable[side] = footInput.anchorUnreachable[side];
            footInput.gait[side].planted = foot.planted;
            footInput.gait[side].lockedWorld = foot.lockedPositionWorld;
            footInput.gait[side].lockedOrientationWorld = foot.lockedOrientationWorld;
            footInput.gait[side].surface = { true, foot.groundHeight, foot.groundNormal };
        }
        // Giro parado pelo clipe: os pes sao do clipe (a pose de referencia ja
        // tem as pernas dele) - no ar pelo contato do clipe, no chao travados.
        if (m_clipTurn.active && m_clipTurn.clip != nullptr && m_clipTurnWeight > 0.5f) {
            const AnimationClip3D& clip = *m_clipTurn.clip;
            for (std::size_t side = 0; side < 2 && side < m_feet.size(); ++side) {
                if (!m_feet[side].valid) continue;
                const float contact = sampleAnimationScalarTrack3D(side == 0 ? clip.leftFootContact : clip.rightFootContact,
                    m_clipTurn.time, clip.durationSeconds, false);
                ScriptedFoot3D& script = footInput.scripted[side];
                script.active = true;
                script.swing = contact < 0.5f;
                script.positionWorld = footInput.reference[side].poseWorld;
                script.orientationWorld = footInput.reference[side].poseOrientationWorld;
            }
        }
        // Toque parado na bola: o pe dirigido pelo manejo.
        if (input.ballScript.active && input.ballScript.side >= 0 && input.ballScript.side < 2
            && m_feet[static_cast<std::size_t>(input.ballScript.side)].valid) {
            ScriptedFoot3D& script = footInput.scripted[static_cast<std::size_t>(input.ballScript.side)];
            script.active = true;
            script.swing = input.ballScript.swing;
            script.positionWorld = input.ballScript.position;
            script.orientationWorld = input.ballScript.orientation;
            script.supportShift = input.ballScript.supportShift;
        }
        footInput.surfaceAt = [&](std::size_t side, Vec3 at, Vec3 forward) {
            const GroundSample ground = groundUnder(side, at, forward);
            return FootSurface3D { ground.valid, ground.height, ground.normal };
        };
        footInput.fitOnLevel = [&](Vec3 at, Vec3 forward) { return fitFoot(at, forward); };
        m_footwork.update(profile, footInput, deltaTime);
        if (footInput.seedFromBody) m_plantFeetFromBody = false;
        // Devolucao a passada: com o segundo passo no ar e o primeiro ja
        // apoiando, no ponto do balanco em que o clipe tem esse mesmo pe
        // quando o outro acaba de tocar o chao - o ciclo vai a fase desse
        // toque e os dois pes casam com o clipe (no sprint cada pe apoia so
        // ~18% do ciclo: em qualquer outro ponto um deles estaria "no ar" no
        // clipe, apoiado de fato, e era arrastado).
        if (footInput.stumbling) {
            const CharacterContactPlan3D& plan = m_footwork.plan();
            const AnimationClip3D* clip = m_previousClip;
            const auto crossings = [&](const AnimationScalarTrack3D& contact, float& liftoff, float& touchdown) {
                constexpr int Samples = 240;
                liftoff = touchdown = -1.0f;
                float previous = sampleAnimationScalarTrack3D(contact,
                    clip->durationSeconds * (Samples - 1) / Samples, clip->durationSeconds, true);
                for (int i = 0; i < Samples; ++i) {
                    const float now = sampleAnimationScalarTrack3D(contact,
                        clip->durationSeconds * i / Samples, clip->durationSeconds, true);
                    if (previous >= 0.5f && now < 0.5f) liftoff = static_cast<float>(i) / Samples;
                    if (previous < 0.5f && now >= 0.5f) touchdown = static_cast<float>(i) / Samples;
                    previous = now;
                }
            };
            bool handed = false;
            for (std::size_t side = 0; side < 2 && clip != nullptr && clip->durationSeconds > 0.0f; ++side) {
                const FootPlan3D& foot = plan.feet[side];
                const FootPlan3D& other = plan.feet[1 - side];
                const bool otherDown = other.phase == FootPhase3D::Stance || other.phase == FootPhase3D::Loading;
                if (foot.phase != FootPhase3D::Swing || !otherDown || m_footwork.stumbleStepsTaken() < 2) continue;
                float swingLift, swingTouch, stanceLift, stanceTouch;
                crossings(side == 0 ? clip->leftFootContact : clip->rightFootContact, swingLift, swingTouch);
                crossings(side == 0 ? clip->rightFootContact : clip->leftFootContact, stanceLift, stanceTouch);
                if (swingLift < 0.0f || swingTouch < 0.0f || stanceTouch < 0.0f) continue;
                const float span = std::max(0.05f, std::fmod(swingTouch - swingLift + 1.0f, 1.0f));
                const float handback = std::fmod(stanceTouch - swingLift + 1.0f, 1.0f) / span;
                const float progress = foot.swingSeconds / std::max(0.05f, foot.swingDurationSeconds);
                if (progress < handback) continue;
                const float offset = stanceAlignedPhase(*clip, 0.0f);
                m_cyclePhase = std::fmod(stanceTouch + 0.01f - offset + 2.0f, 1.0f);
                handed = true;
            }
            if (handed || m_stumbleStepSeconds > 1.0f) {
                m_stumbleStepping = false;
                m_sinceStumbleStep = 0.0f;
            }
        }
    }
    const CharacterContactPlan3D& contactPlan = m_footwork.plan();
    const bool recovering = contactPlan.recovering;
    m_reactiveShare = recovering ? 1.0f : 0.0f;
    m_telemetry.reactiveShare = m_reactiveShare;
    // O estado oficial dos pes do planejador, copiado no registro que o IK,
    // o limite de torcao do quadril e o aterramento leem.
    for (std::size_t side = 0; side < m_feet.size(); ++side) {
        if (!m_footwork.owns(side)) {
            // Com a passada: nenhum passo do planejador neste pe.
            m_feet[side].repositioning = false;
            m_feet[side].recoveryStep = false;
            m_feet[side].turnStep = false;
            m_feet[side].pivotStep = false;
            continue;
        }
        FootPlant& foot = m_feet[side];
        const FootPlan3D& plan = contactPlan.feet[side];
        const bool down = plan.phase == FootPhase3D::Stance
            || plan.phase == FootPhase3D::Unloading
            || plan.phase == FootPhase3D::Loading;
        foot.planted = down;
        foot.repositioning = !down;
        foot.recoveryStep = plan.reason == StepReason3D::Recovery;
        foot.turnStep = plan.reason == StepReason3D::Turn;
        foot.pivotStep = !down && plan.path == SwingPath3D::Pivot;
        foot.stepPending = false;
        foot.releaseBlend = 0.0f;
        foot.replantDelay = 0.0f;
        foot.takeoffValid = false;
        foot.activeShift = {};
        foot.fitValid = false;
        if (down) {
            foot.lockedPositionWorld = plan.anchorWorld;
            foot.lockedOrientationWorld = plan.anchorOrientationWorld;
            if (plan.surface.valid) {
                foot.groundHeight = plan.surface.height;
                foot.groundNormal = plan.surface.normal;
            }
        } else {
            foot.repositionFromWorld = plan.liftoffWorld;
            foot.repositionFromOrientation = plan.liftoffOrientationWorld;
            foot.repositionToWorld = plan.goalWorld;
            foot.takeoffGroundHeight = plan.liftoffGroundHeight;
        }
    }
    // Peso para o pe de apoio num passo comum (pedido do planejador).
    Vec3 standingShift = contactPlan.supportShiftWorld;
    // E5 (animacao + pes conscientes): quanto a velocidade do corpo real se
    // afastou da da referencia (a capsula), em ponto de captura (/ omega). O
    // pe em balanco da passada pousa deslocado disso - a animacao continua
    // decidindo o passo; andando bem, a diferenca e quase nula. So a
    // velocidade: pela posicao tambem, com a pelve um pouco atras da capsula
    // (o atraso normal do corpo), o pe pousava atras sem desequilibrio
    // nenhum e freava o corpo (ajuda a 40%: uma queda a mais). Filtrado
    // ~50 ms, ate 15 cm.
    Vec3 balanceShift {};
    // Numa recuperacao forte, o pe no ar pode ampliar a base no sentido da
    // freada. A mola precisa ter passado de metade da reacao antes disso:
    // residuos pequenos ficam no tronco e nunca criam a passada projetada.
    const float emergencyBrakeStep = smoothStep(
        (m_brakeReaction.x - 0.55f) / 0.35f) * (1.0f - m_brakeGait);
    if (input.forceDrivenRoot && onGround && emergencyBrakeStep > 0.0f)
        balanceShift += m_brakeDirection * (0.14f * emergencyBrakeStep);
    // Recuperacao sem perder velocidade: amplia a passada no rumo do
    // movimento. Se o obstaculo pegou cedo, o proprio pe continua por cima;
    // se pegou tarde, este deslocamento permanece para o passo seguinte.
    if (input.forceDrivenRoot && onGround && m_tripRecovery > 0.0f)
        balanceShift += m_tripDirection * (0.26f * StumbleShiftScale * m_tripRecovery);
    if (input.balanceFootPlacement && input.forceDrivenRoot && onGround) {
        const float omega = std::clamp(m_estimator.state().captureOmega, 1.0f, 6.0f);
        const PhysicsBodyState3D& pelvis = state.links.front();
        const Vec3 velocityError { pelvis.linearVelocity.x - planarVelocity.x,
            pelvis.linearVelocity.y - planarVelocity.y, 0.0f };
        Vec3 wanted = velocityError * (BalancePlacementGain / omega);
        float placementLimit = BalancePlacementLimitMeters;
        float captureShare = CapturePlacement && (m_brakeGait <= 0.0f || (CaptureInBrakeRun && !m_brakeStepping))
            ? std::max(CapturePlacementAllAssist, smoothStep((0.5f - std::clamp(input.walkingAssist, 0.0f, 1.0f)) / 0.4f)) : 0.0f;
        // Experimento v2 (T_CapConsistent): o pe corrige a parte que a ajuda
        // nao corrige (1 - ajuda).
        static const float captureConsistent = tuneTemp("T_CapConsistent", 0.0f);  // TEMPTUNE
        if (captureConsistent > 0.0f && CapturePlacement && (m_brakeGait <= 0.0f || (CaptureInBrakeRun && !m_brakeStepping)))
            captureShare = std::max(captureShare, captureConsistent * (1.0f - std::clamp(input.walkingAssist, 0.0f, 1.0f)));
        // Na quase-queda, em qualquer ajuda: o pe vai ao ponto de captura,
        // mais longe que o limite comum.
        float stumbleShare = 0.0f;
        if (StumbleDetection && m_brakeGait <= 0.0f) {
            stumbleShare = std::clamp(StumblePlacement * m_tripRecovery, 0.0f, 1.0f);
            captureShare = std::max(captureShare, stumbleShare);
        }
        if (captureShare > 0.0f) {
            // Pelo ponto de captura inteiro (posicao do centro de massa mais
            // velocidade/omega) contra o da referencia da passada: com pouca
            // ajuda a capsula acompanha o corpo e so a diferenca de
            // velocidade sumia - o corpo mergulhava para a frente com o ponto
            // de captura 0,5 m a frente e o pe pousava embaixo dele. (Fora da
            // freada: ela tem o proprio pouso, e os dois brigavam.)
            const Vec3 com = m_estimator.state().centerOfMassWorld;
            const Vec3 comVelocity = m_filteredCenterOfMassVelocity;
            const Vec3 capture { com.x + comVelocity.x / omega, com.y + comVelocity.y / omega, 0.0f };
            const Vec3 referenceCapture { navigationRoot.x + planarVelocity.x / omega,
                navigationRoot.y + planarVelocity.y / omega, 0.0f };
            // Para tras (o corpo mais lento que a passada) so uma parte: o pe
            // puxado para baixo do corpo nao levava a perna a frente e o corpo
            // nao acelerava (quanto mais puxava, mais lento: 1,0 -> 1,6 m/s,
            // 0,5 -> 2,0 a 5%); de lado ao corpo, no strafe, girava a pelve.
            // Para a frente (mais rapido que a passada, caindo para a frente)
            // e atravessado, inteiro.
            Vec3 shift = (capture - referenceCapture) * CapturePlacementGain;
            const float planarSpeed = std::hypot(planarVelocity.x, planarVelocity.y);
            if (planarSpeed > 0.3f && m_gaitDirection == CharacterGaitDirection3D::Forward) {
                const Vec3 travel { planarVelocity.x / planarSpeed, planarVelocity.y / planarSpeed, 0.0f };
                float along = dot(shift, travel);
                // v2: so o atraso alem do da coleira puxa o pe. Com pouca ajuda
                // a capsula anda de proposito ~1 m/s e ~16 cm a frente do corpo
                // (physicalNavigationLead3D), e o ponto de captura lia isso como
                // corpo atrasado: o pe ia os 30 cm inteiros para tras o tempo
                // todo e as pernas corriam atras do corpo (o usuario viu a 0%).
                if (along < 0.0f && input.physicalV2 && CaptureBackDeadZone > 0.0f) {
                    const float kept = std::min(0.0f, along + CaptureBackDeadZone);
                    shift -= travel * (along - kept);
                    along = kept;
                }
                if (along < 0.0f) shift -= travel * (along * (1.0f - CapturePullBackShare));
            }
            wanted += (shift - wanted) * captureShare;
            placementLimit += (CapturePlacementLimit - placementLimit) * captureShare;
            placementLimit = std::max(placementLimit, StumblePlacementLimit * stumbleShare);
        }
        if (wanted.length() > placementLimit)
            wanted = wanted * (placementLimit / wanted.length());
        m_balancePlacementShift += (wanted - m_balancePlacementShift)
            * (1.0f - std::exp(-deltaTime / 0.05f));
        balanceShift += m_balancePlacementShift;
    } else {
        m_balancePlacementShift = {};
    }

    for (std::size_t side = 0; side < m_feet.size(); ++side) {
        FootPlant& foot = m_feet[side];
        m_telemetry.footPlanted[side] = false;
        m_telemetry.footSwingProgress[side] = -1.0f;
        m_telemetry.footClearanceMeters[side] = 0.0f;
        if (!foot.valid || !footworkEnabled) continue;
        const RagdollLinkDefinition3D& link = profile.links[foot.linkIndex];
        FootGoal& goal = goals[side];
        goal.active = true;

        // Pe animado, com o passo encurtado ou alongado ao longo da direcao
        // do ciclo, em torno do quadril.
        Vec3 footLocal = localPose.linkPositions[foot.linkIndex];
        if (gaitMoving && std::abs(strideScale - 1.0f) > 0.005f) {
            const int shin = link.parentIndex;
            const int thigh = shin > 0 ? profile.links[
                static_cast<std::size_t>(shin)].parentIndex : -1;
            if (thigh > 0) {
                const Vec3 hip = profile.links[0].modelOrientation.conjugate()
                    .rotate(profile.links[static_cast<std::size_t>(thigh)]
                        .inboundJoint.anchorModelPosition
                        - profile.links[0].modelPosition);
                const float along = dot(footLocal - hip, travelAxisLocal);
                footLocal += travelAxisLocal * ((strideScale - 1.0f) * along);
            }
        }
        const Vec3 animatedWorld = targetRootPosition
            + targetRootOrientation.rotate(footLocal);
        Quaternion animatedOrientation =
            clipFootOrientation(foot.linkIndex);
        // Na curva o pe pousa girado para dentro dela, o de dentro mais: ele
        // ajuda a virar em vez de apontar reto. Pela necessidade real - a
        // aceleracao centripeta que o corpo faz de fato -, nao pelo pedido:
        // pela curva pedida o pe ja girava ~30 graus ao comecar a virar a
        // camera, com o corpo ainda reto, e numa curva suave. Nas
        // referencias: Sprint Turn, 4,1 m/s2, o de dentro ~27 graus alem da
        // abertura da corrida reta; Running Right Turn, 3,2 m/s2, quase nada.
        // Gira durante o balanco e fica assim no apoio (a trava do pouso
        // guarda a orientacao).
        {
            const bool inner = (m_bodyCurveAcceleration > 0.0f) == (side == 0);
            const float need = smoothStep((std::abs(m_bodyCurveAcceleration) - CurveFootYawStart)
                / CurveFootYawSpan);
            const float footLead = std::copysign(CurveFootYawMaximum * need
                    * (inner ? 1.0f : CurveFootYawOuterShare), m_bodyCurveAcceleration)
                * smoothStep((speed - 1.5f) / 1.5f);
            // Freando ou parado nao ha curva: o giro sai (preso num pe, ele
            // ligava o giro da coxa nos passos de freada).
            if (!gaitMoving || m_brakeGaitActive || curveIntent < 0.5f)
                foot.curveYaw *= std::exp(-CurveFootYawRate * deltaTime);
            else if (!foot.planted)
                foot.curveYaw += (footLead - foot.curveYaw) * (1.0f - std::exp(-CurveFootYawRate * deltaTime));
            if (std::abs(foot.curveYaw) > 1e-4f)
                animatedOrientation = (Quaternion::fromAxisAngle({ 0.0f, 0.0f, 1.0f }, foot.curveYaw)
                    * animatedOrientation).normalized();
        }
        goal.animatedPosition = animatedWorld;
        goal.animatedOrientation = animatedOrientation;
        const Vec3 animatedForward = forwardOf(foot, animatedOrientation);

        if (m_footwork.owns(side)) {
            foot.previousAnimatedPositionWorld = animatedWorld;
            foot.previousAnimatedPositionValid = true;
            if (!foot.planted) {
                // No ar pelo planejador: a trajetoria dele, ate o contato.
                const FootPlan3D& plan = contactPlan.feet[side];
                goal.position = plan.targetWorld;
                goal.orientation = plan.targetOrientationWorld;
                goal.pelvisGround = plan.goalSurface.valid
                    ? std::min(plan.liftoffGroundHeight, plan.goalSurface.height)
                    : plan.liftoffGroundHeight;
                goal.reach = onGround ? 1.0f : 0.0f;
                continue;
            }
        } else {
            const float sole = footSoleOffset(link);
            // Folga do pe em relacao ao chao do CLIPE (a base da capsula): e o que
            // diz se a pose esta pisando, independente do degrau embaixo.
            const float clearance = animatedWorld.z - baseGround - sole;
            m_telemetry.footClearanceMeters[side] = clearance;
            const Vec3 animatedVelocity = foot.previousAnimatedPositionValid
                ? (animatedWorld - foot.previousAnimatedPositionWorld) / deltaTime
                : Vec3 {};
            const bool authoredStance = onGround
                && desiredState != CharacterLocomotionState3D::JumpStarting
                && contactWeights[side] > 0.55f;
            // A velocidade do pe animado escala com o ritmo (a mesma passada em
            // menos tempo); o portao tambem, para a trava valer em toda a faixa.
            const float velocityGateScale = std::max(1.0f, playbackRate);
            // Passada pelo contato: o pe que o clipe ja poe no apoio e tocou o
            // chao de verdade trava ali, mesmo com o pe animado andando - com o
            // ciclo parado esperando o toque, ele anda com o corpo, e pelo
            // portao de velocidade o pe nunca travava (sprint: nenhum apoio).
            // (TEMPTUNE TouchPlantInCurves: tambem fora da corrida estabelecida
            // - na curva o pedido gira a frente do corpo e conta como troca de
            // passada; o pe apoiado sem trava seguia o balanco e o quadril
            // estendia de -79 a +27 graus com o pe no chao, chutando o corpo
            // para cima - os saltinhos da curva. So nao na freada.)
            const bool touchedInStance = ContactPhase && input.forceDrivenRoot && gaitMoving
                && (m_contactSteady || (TouchPlantInCurves && m_brakeGait <= 0.0f && !m_brakeStepping))
                && side < physical.feet.size()
                && (physical.feet[side].touching || physical.feet[side].supporting);
            const bool stanceCandidate = authoredStance
                && clearance < 0.105f && clearance > -0.065f
                && ((std::hypot(animatedVelocity.x, animatedVelocity.y)
                    < 0.72f * velocityGateScale
                && std::abs(animatedVelocity.z) < 1.1f * velocityGateScale) || touchedInStance);
            foot.replantDelay = std::max(0.0f, foot.replantDelay - deltaTime);
            // O primeiro toque do pe fisico neste balanco (descendo, perto do
            // fim dele).
            if (!foot.planted && input.forceDrivenRoot && side < physical.feet.size()) {
                const FootSupportEstimate3D& touch = physical.feet[side];
                if (!(touch.touching || touch.supporting)) {
                    if (clearance > 0.03f) foot.firstTouchValid = false;
                } else if (!foot.firstTouchValid && LockAtFirstTouch) {
                    foot.firstTouchValid = true;
                    foot.firstTouchWorld = state.links[foot.linkIndex].position;
                }
            }
            if (!foot.planted && stanceCandidate && foot.replantDelay <= 0.0f) {
                // Encaixe de novo no toque: a previsao erra alguns centimetros
                // quando a capsula muda de velocidade (cada degrau subido a
                // freia). Ate 10 cm; alem disso o pe fica onde a pose pos.
                Vec3 touchdown = animatedWorld + foot.activeShift;
                const Vec3 correction = fitFoot(touchdown, animatedForward);
                if (correction.lengthSquared() <= 0.01f) {
                    touchdown += correction;
                    foot.activeShift += correction;
                }
                const GroundSample ground = groundUnder(side, touchdown,
                    animatedForward);
                const Quaternion orientation = (rotationBetween(
                    { 0.0f, 0.0f, 1.0f }, ground.normal) * animatedOrientation)
                        .normalized();
                foot.planted = true;
                foot.lockedOrientationWorld = orientation;
                foot.lockedPositionWorld = animatedWorld + foot.activeShift;
                // Trava onde o pe fisico tocou (como o pouso do planejador):
                // o balanco nao acompanha o recuo do pe no fim do balanco e o
                // pe tocava ate 16 cm a frente da pose - travado na pose, a
                // perna arrastava o corpo ate a diferenca (com a sola que
                // agarra, empurrando-o para cima: o "saltinho"). Ainda no ar,
                // a trava espera o toque um instante.
                foot.lockFollowTouch = 0.0f;
                if (LockAtTouch && input.forceDrivenRoot && side < physical.feet.size()
                    && (!LockTouchOnlyInChange || m_gaitChange > 0.0f)) {
                    const FootSupportEstimate3D& touch = physical.feet[side];
                    if (touch.touching || touch.supporting) {
                        // (So no mesmo nivel: tocando a quina de outro degrau
                        // vale o encaixe de cima.)
                        const Vec3 at = foot.firstTouchValid ? foot.firstTouchWorld
                            : state.links[foot.linkIndex].position;
                        const GroundSample there = groundUnder(side, at, animatedForward);
                        if (there.valid && std::abs(there.height - ground.height) < 0.03f
                            && std::hypot(at.x - foot.lockedPositionWorld.x, at.y - foot.lockedPositionWorld.y) < 0.30f) {
                            // A pose fica ate LockTouchPull do toque: a perna
                            // ainda "puxa o chao" (o pe que pousa indo para a
                            // frente nao escorrega - no sprint, 2 cm por apoio
                            // contra 12-16 travando no toque), sem a diferenca
                            // inteira que virava vara.
                            const Vec3 pull = clampMagnitude(Vec3 { foot.lockedPositionWorld.x - at.x,
                                foot.lockedPositionWorld.y - at.y, 0.0f }, LockTouchPull);
                            foot.lockedPositionWorld.x = at.x + pull.x;
                            foot.lockedPositionWorld.y = at.y + pull.y;
                        }
                    } else {
                        foot.lockFollowTouch = LockTouchWaitSeconds;
                    }
                }
                foot.firstTouchValid = false;
                foot.activeShift = {};
                foot.fitValid = false;
                // O proximo balanco sai daqui (gravado ao soltar a trava).
                foot.takeoffValid = false;
                foot.lockedPositionWorld.z = lockHeight(foot, ground.height,
                    orientation);
                foot.groundHeight = ground.height;
                foot.groundNormal = ground.normal;
                foot.plantedGoalPosition = foot.lockedPositionWorld;
                foot.plantedGoalOrientation = orientation;
            } else if (foot.planted) {
                if (foot.lockFollowTouch > 0.0f) {
                    foot.lockFollowTouch = std::max(0.0f, foot.lockFollowTouch - deltaTime);
                    const FootSupportEstimate3D& touch = physical.feet[side];
                    if (touch.touching || touch.supporting) {
                        const Vec3 at = state.links[foot.linkIndex].position;
                        const GroundSample there = groundUnder(side, at, animatedForward);
                        if (there.valid && std::abs(there.height - foot.groundHeight) < 0.03f
                            && std::hypot(at.x - foot.lockedPositionWorld.x, at.y - foot.lockedPositionWorld.y) < 0.30f) {
                            const Vec3 pull = clampMagnitude(Vec3 { foot.lockedPositionWorld.x - at.x,
                                foot.lockedPositionWorld.y - at.y, 0.0f }, LockTouchPull);
                            foot.lockedPositionWorld.x = at.x + pull.x;
                            foot.lockedPositionWorld.y = at.y + pull.y;
                            foot.plantedGoalPosition.x = foot.lockedPositionWorld.x;
                            foot.plantedGoalPosition.y = foot.lockedPositionWorld.y;
                        }
                        foot.lockFollowTouch = 0.0f;
                    }
                }
                const float horizontalDrift = std::hypot(
                    animatedWorld.x - foot.lockedPositionWorld.x,
                    animatedWorld.y - foot.lockedPositionWorld.y);
                // Fora de alcance (ver legOutOfReach), o pe solta e vira balanco.
                const bool outOfReach = legOutOfReach(foot);
                // Solta quando o clipe diz que o apoio acabou. Com folga (ate o
                // contato cair a 0,30) o pe ficava preso enquanto a pose ja
                // levava o pe embora; na soltura ele corria atras dela (medido:
                // 12 m/s no sprint de lado, com o corpo a 3,7).
                // (Parado, quem decide os pes e o planejador de passos - a
                // passada nao espera mais a marca de recuperacao dele.)
                // Arrancando, o pe que ainda carrega o corpo empurra ate
                // descarregar: o ciclo comeca com o de tras a 42% do balanco, e
                // ele saia arrastado no chao.
                const bool startHold = StartLoadHoldSeconds > 0.0f && m_startClipActive
                    && m_startClipSeconds < StartLoadHoldSeconds && input.forceDrivenRoot
                    && side < physical.feet.size() && physical.feet[side].supporting
                    && physical.feet[side].loadNewtons > StartLoadHoldShare * profile.totalMassKg * 9.81f
                    && !outOfReach;
                // Correndo, o pe que o clipe solta mas que ainda carrega o corpo
                // (sem o outro no chao) empurra mais um pouco: soltando na hora,
                // a ponta dele era arrastada no chao (na curva fechada, com o
                // corpo baixo, 1-3 m/s por 0,05-0,1 s - a "bicuda" na saida).
                const std::size_t other = 1 - side;
                const bool liftHold = LiftAfterUnloadSeconds > 0.0f && input.forceDrivenRoot && gaitMoving
                    && contactWeights[side] < 0.55f && !outOfReach && side < physical.feet.size()
                    && physical.feet[side].supporting
                    && physical.feet[side].loadNewtons > LiftAfterUnloadShare * profile.totalMassKg * 9.81f
                    && !physical.feet[other].supporting
                    && foot.liftHoldSeconds < LiftAfterUnloadSeconds;
                foot.liftHoldSeconds = liftHold ? foot.liftHoldSeconds + deltaTime
                    : contactWeights[side] >= 0.55f ? 0.0f : foot.liftHoldSeconds;
                // TEMPTUNE: correndo, o pe que pousou atrasado (ou ainda nem
                // encostou) fica no apoio ate cumprir um apoio minimo - o
                // clipe solta pelo relogio, e o pe que mal tocava (ou nem
                // tocava) ja era levado para o passo seguinte. So este pe
                // espera: o ciclo segue, a outra perna balanca normal (parar
                // o ciclo congelava a perna de tras baixa enquanto o corpo
                // andava ~0,5 m, e ela pegava a barra do tropeco).
                // Folga pela ponta da sola (o ponto mais baixo do pe animado):
                // rolando sobre a ponta no fim do apoio, o centro do pe sobe
                // varios cm com a ponta ainda no chao - pelo centro, o pe era
                // solto no meio do empurrao.
                const float soleClearance = animatedWorld.z
                    - lockHeight(foot, baseGround, animatedOrientation);
                const bool wouldRelease = contactWeights[side] < 0.55f || outOfReach
                    || (!stanceCandidate && (ReleaseBySoleClearance ? soleClearance : clearance) > 0.055f)
                    || horizontalDrift > 0.48f;
                const bool touchHold = TouchHoldSeconds > 0.0f && input.forceDrivenRoot && gaitMoving
                    && wouldRelease && !outOfReach && side < physical.feet.size()
                    && physical.feet[side].supportingSeconds < TouchMinStanceSeconds
                    && foot.touchHoldSeconds < TouchHoldSeconds
                    && m_tripRecovery <= 0.001f && !m_stumbleStepping
                    && (!TouchHoldSteadyOnly || (m_brakeGait <= 0.0f && !m_brakeStepping && !m_startClipActive
                        && m_gaitChange <= 0.0f && m_terrainSlope < 0.05f
                        && std::hypot(input.intent.requestedVelocityWorld.x,
                            input.intent.requestedVelocityWorld.y) > 1.0f));
                foot.touchHoldSeconds = touchHold ? foot.touchHoldSeconds + deltaTime
                    : !wouldRelease ? 0.0f : TouchHoldSeconds;
                // Devolvido apoiado pelo planejador: espera o clipe chegar ao
                // apoio dele (o ciclo de corrida tem voo entre a saida de um pe
                // e o pouso do outro).
                foot.handbackHoldSeconds = contactWeights[side] >= 0.55f ? 0.0f
                    : std::max(0.0f, foot.handbackHoldSeconds - deltaTime);
                const bool handbackHold = foot.handbackHoldSeconds > 0.0f && side < physical.feet.size()
                    && physical.feet[side].supporting && !outOfReach;
                const bool releaseByGait = !startHold && !liftHold && !handbackHold && !touchHold && wouldRelease;
                if (std::getenv("XHANDDBG") && (releaseByGait || !input.grounded) && (foot.handbackHoldSeconds > 0.0f
                        || (m_startClipActive && m_startClipSeconds < 0.3f)))
                    std::printf("HANDBACK release side %zu hold %.2f start %.2f contact %.2f sup %d load %.0f out %d drift %.2f cand %d clr %.3f grounded %d starthold %d\n", side,
                        foot.handbackHoldSeconds, m_startClipSeconds, contactWeights[side], physical.feet[side].supporting ? 1 : 0,
                        physical.feet[side].loadNewtons, outOfReach ? 1 : 0,
                        horizontalDrift, stanceCandidate ? 1 : 0, clearance, input.grounded ? 1 : 0, startHold ? 1 : 0);
                if (releaseByGait || !input.grounded) {
                    foot.planted = false;
                    // Solto porque a perna nao alcanca: nao trava de novo no tick
                    // seguinte. Travando e soltando a cada tick, cada trava nova
                    // puxava o alvo para a pose (pe a 6,6 m/s depois de levantar).
                    if (outOfReach) foot.replantDelay = 0.12f;
                    foot.releaseBlend = 1.0f;
                    foot.releasePositionWorld = foot.plantedGoalPosition;
                    foot.releaseOrientationWorld = foot.plantedGoalOrientation;
                    // De onde o balanco sai: altura e ponto. Sem isto ficava o de
                    // um passo antigo (medido: saida 2,5 m para tras), e o pe subia
                    // para o degrau de pouso logo no comeco do balanco.
                    foot.takeoffGroundHeight = foot.groundHeight;
                    foot.takeoffPosition = foot.plantedGoalPosition;
                    foot.takeoffValid = true;
                    foot.swingFloor = foot.groundHeight;
                }
            }
            foot.previousAnimatedPositionWorld = animatedWorld;
            foot.previousAnimatedPositionValid = true;
        }

        if (foot.planted) {
            foot.releaseBlend = 0.0f;
            // Travado, o pe ainda rola: quando o clipe ergue o calcanhar (a
            // saida) ele gira sobre a ponta, e quando pousa de calcanhar,
            // sobre o calcanhar - o ponto de apoio fica parado no mundo e o
            // rumo do pe fica o da trava. Congelado plano, o pe so rolava ao
            // soltar a trava, de uma vez (medido: chicote de 8,9 m/s no
            // strafe).
            const Vec3 normal = foot.groundNormal;
            Quaternion rolled = (rotationBetween({ 0.0f, 0.0f, 1.0f }, normal)
                * animatedOrientation).normalized();
            Vec3 lockedForward = forwardOf(foot, foot.lockedOrientationWorld);
            Vec3 rolledForward = forwardOf(foot, rolled);
            lockedForward -= normal * dot(lockedForward, normal);
            rolledForward -= normal * dot(rolledForward, normal);
            if (lockedForward.lengthSquared() > 0.0001f
                && rolledForward.lengthSquared() > 0.0001f) {
                const Vec3 a = lockedForward.normalized();
                const Vec3 b = rolledForward.normalized();
                const float yaw = std::atan2(dot(cross(b, a), normal),
                    dot(a, b));
                rolled = (Quaternion::fromAxisAngle(normal, yaw) * rolled)
                    .normalized();
            }
            const Vec3 half = link.collider.shape == RagdollColliderShape3D::Box
                ? link.collider.boxHalfExtents : Vec3 {};
            const Vec3 toe = link.collider.localPosition
                + link.collider.localOrientation.rotate(
                    { half.x, 0.0f, -half.z });
            const Vec3 heel = link.collider.localPosition
                + link.collider.localOrientation.rotate(
                    { -half.x, 0.0f, -half.z });
            const float toeDrop = dot(rolled.rotate(toe)
                - foot.lockedOrientationWorld.rotate(toe), normal);
            const float heelDrop = dot(rolled.rotate(heel)
                - foot.lockedOrientationWorld.rotate(heel), normal);
            Vec3 position = foot.lockedPositionWorld;
            if (half.x > 0.0f && (toeDrop < -0.001f || heelDrop < -0.001f)) {
                const Vec3 pivot = toeDrop < heelDrop ? toe : heel;
                const Vec3 pivotWorld = foot.lockedPositionWorld
                    + foot.lockedOrientationWorld.rotate(pivot);
                position = pivotWorld - rolled.rotate(pivot);
            } else {
                rolled = foot.lockedOrientationWorld;
            }
            foot.plantedGoalPosition = position;
            foot.plantedGoalOrientation = rolled;
            goal.position = position;
            goal.orientation = rolled;
            goal.pelvisGround = foot.groundHeight;
            goal.reach = 1.0f;
            m_telemetry.footPlanted[side] = true;
            m_telemetry.footSecondsToTouchdown[side] = -1.0f;
            continue;
        }

        // Balanco. Chao de onde o pe saiu e de onde ele vai pisar.
        float swingGround = baseGround;
        float pelvisGround = baseGround;
        Quaternion swingOrientation = animatedOrientation;
        float lift = 0.0f;
        const FootTouchdown* next = nullptr;
        float progress = 0.0f;
        float secondsToTouchdown = 0.0f;
        float swingRise = 0.0f;
        float swingPathShare = 0.0f;
        Vec3 swingLanding;
        if (events != nullptr && onGround) {
            const float duration = std::max(0.05f,
                selectedClip->durationSeconds);
            float best = std::numeric_limits<float>::infinity();
            for (const FootTouchdown& candidate : events->touchdowns[side]) {
                const float ahead = std::fmod(candidate.touchdown
                    - legSampleTime[side] + duration * 2.0f, duration);
                if (ahead < best) {
                    best = ahead;
                    next = &candidate;
                }
            }
            // A trava vem alguns ticks depois do toque do clipe (contato
            // acima de 0,55, pe animado parado). Nesse meio tempo o pe ainda
            // esta pousando naquele toque: passar para o toque seguinte
            // jogava fora o encaixe no degrau (o pe voltava 12 cm, para a
            // quina) e a altura de pouso.
            for (const FootTouchdown& candidate : events->touchdowns[side]) {
                const float swing = std::fmod(candidate.touchdown
                    - candidate.liftoff + duration * 2.0f, duration);
                const float since = std::fmod(legSampleTime[side]
                    - candidate.touchdown + duration * 2.0f, duration);
                if (since < 0.5f * (duration - swing)) {
                    next = &candidate;
                    best = 0.0f;
                    break;
                }
            }
            if (next != nullptr && best == 0.0f) {
                progress = 1.0f;
                secondsToTouchdown = 0.0f;
            } else if (next != nullptr) {
                const float swing = std::fmod(next->touchdown - next->liftoff
                    + duration * 2.0f, duration);
                const float elapsed = std::fmod(legSampleTime[side] - next->liftoff
                    + duration * 2.0f, duration);
                progress = swing > 0.001f && elapsed <= swing
                    ? std::clamp(elapsed / swing, 0.0f, 1.0f) : 0.0f;
                secondsToTouchdown = best / std::max(0.2f, playbackRate);
            }
        }
        if (next != nullptr) {
            // Onde o pe vai pisar: a pelve la na hora do toque (velocidade da
            // capsula), mais o pe do clipe naquele instante, com o mesmo
            // encurtamento de passo.
            Vec3 footInClip = next->footInClip;
            const float along = dot(footInClip - next->hipInClip,
                Vec3 { std::cos(clipTravel), std::sin(clipTravel), 0.0f });
            footInClip += Vec3 { std::cos(clipTravel), std::sin(clipTravel),
                0.0f } * ((strideScale - 1.0f) * along);
            // TEMPTUNE CurveNarrow: na curva os dois pes pousam na mesma linha
            // para fora do corpo (o apoio passa pelo centro de massa
            // inclinado). Com a largura de passo do clipe o pe de fora caia
            // 0,4-0,6 m para fora e o de dentro quase embaixo do corpo: a
            // perna de dentro, vertical, mandava a forca toda para cima - o
            // galope na perna de dentro (70% dos apoios), os saltinhos.
            if (CurveNarrow > 0.0f && CurveLeanMaximum > 0.0f) {
                const Vec3 across { -std::sin(clipTravel), std::cos(clipTravel), 0.0f };
                const float lateral = dot(footInClip, across);
                const float share = std::min(1.0f, CurveNarrow * std::abs(m_bodyCurveLean) / CurveLeanMaximum);
                footInClip -= across * (lateral * share);
            }
            Vec3 landingWorld = navigationRoot
                + planarVelocity * secondsToTouchdown
                + heading.rotate(footInClip) + balanceShift;
            // A pelve nao estara no prolongamento reto da velocidade atual:
            // numa curva, ela percorre um arco. A previsao centripeta leva o
            // pouso para dentro da curva e ja orienta o pe pelo rumo esperado
            // no toque. Sem isso os pes continuavam correndo reto enquanto o
            // tronco girava por cima deles.
            const float arcOffset = std::clamp(0.5f
                * signedCurveAcceleration * secondsToTouchdown
                * secondsToTouchdown, -0.22f, 0.22f);
            landingWorld += heading.rotate({0.0f, arcOffset, 0.0f});
            // Inclinado na curva, a perna gira em volta do pe: ele pousa
            // para fora do quadril h sen(theta) e o quadril desce h (1 -
            // cos(theta)) (bloco da altura) - a forca do chao passa pelo
            // corpo inclinado.
            landingWorld += heading.rotate({ 0.0f,
                -CurveLeanHeight * std::sin(CurveFootShare * m_bodyCurveLean), 0.0f });
            // Chute: o pouso natural (telemetria, para o manejo planejar) e o
            // pe de apoio indo para o lado da bola, ate um limite.
            m_telemetry.footLandingWorld[side] = landingWorld;
            m_telemetry.footSecondsToTouchdown[side] = secondsToTouchdown;
            {
                Vec3 wanted;
                if (input.ballPlant.active && input.ballPlant.side == static_cast<int>(side)) {
                    const Vec3 ahead = heading.rotate({ 1.0f, 0.0f, 0.0f });
                    const Vec3 aside = heading.rotate({ 0.0f, 1.0f, 0.0f });
                    const Vec3 offset { input.ballPlant.position.x - landingWorld.x,
                        input.ballPlant.position.y - landingWorld.y, 0.0f };
                    wanted = (ahead * std::clamp(dot(offset, ahead), -0.35f, 0.30f)
                        + aside * std::clamp(dot(offset, aside), -0.25f, 0.25f))
                        * std::clamp(input.ballPlant.weight, 0.0f, 1.0f);
                }
                Vec3& plantShift = m_ballPlantShift[side];
                const float tau = wanted.lengthSquared() > 1e-6f ? 0.05f : 0.15f;
                plantShift += (wanted - plantShift) * (1.0f - std::exp(-deltaTime / tau));
                landingWorld += plantShift;
            }
            const Quaternion touchdownTurn = Quaternion::fromAxisAngle(
                {0.0f, 0.0f, 1.0f}, m_turningRate * secondsToTouchdown);
            Vec3 landingForward = touchdownTurn.rotate(
                heading.rotate(next->forwardInClip));
            // O encaixe no degrau e caro (varias sondas): refeito so quando o
            // pouso previsto anda mais de 3 cm.
            const Vec3 moved { landingWorld.x - foot.fitCenter.x,
                landingWorld.y - foot.fitCenter.y, 0.0f };
            if (!foot.fitValid || moved.lengthSquared() > 0.0009f) {
                foot.fitShift = fitFoot(landingWorld, landingForward);
                foot.fitCenter = landingWorld;
                foot.fitValid = true;
            }
            landingWorld += foot.fitShift;
            const GroundSample landing = groundUnder(side, landingWorld,
                landingForward);
            if (!foot.takeoffValid) {
                foot.takeoffGroundHeight = foot.groundHeight;
                foot.takeoffPosition = foot.releaseBlend > 0.0f
                    ? foot.releasePositionWorld : animatedWorld;
                foot.takeoffValid = true;
                foot.swingFloor = foot.groundHeight;
            }
            foot.landingGroundHeight = landing.height;
            foot.landingNormal = landing.normal;
            const float from = foot.takeoffGroundHeight;
            const float to = landing.height;
            const float rise = to - from;
            // Quanto do caminho ate o pouso o pe ja andou, na horizontal.
            // Sem a sonda de terreno, a altura do degrau novo acompanha isso:
            // subindo, chega a 70% do caminho; descendo, so depois da metade.
            // Com a sonda, quem decide e o chao a frente do pe (mais abaixo).
            const Vec3 path { landingWorld.x - foot.takeoffPosition.x,
                landingWorld.y - foot.takeoffPosition.y, 0.0f };
            foot.swingDirection = path.lengthSquared() > 0.0004f
                ? path.normalized() : Vec3 {};
            const Vec3 travelled { animatedWorld.x - foot.takeoffPosition.x,
                animatedWorld.y - foot.takeoffPosition.y, 0.0f };
            const float pathShare = path.lengthSquared() > 0.0004f
                ? std::clamp(dot(travelled, path) / path.lengthSquared(),
                    0.0f, 1.0f)
                : progress;
            const float share = rise > 0.01f
                ? smoothStep(pathShare / 0.7f)
                : smoothStep((pathShare - 0.35f) / 0.65f);
            swingGround = from + rise * share;
            lift = std::max(0.0f, rise) * 0.2f * std::sin(Pi * pathShare);
            swingRise = rise;
            swingPathShare = pathShare;
            swingLanding = landingWorld;
            goal.anticipate = smoothStep((progress - 0.3f) / 0.5f);
            // Paralelo ao chao de pouso, so com o rumo do pe do clipe.
            const Quaternion landingSurface = rotationBetween({ 0.0f, 0.0f, 1.0f },
                landing.normal);
            const Vec3 animatedAhead = animatedOrientation.rotate({ 1.0f, 0.0f, 0.0f });
            const Quaternion landingFlat = (landingSurface * Quaternion::fromAxisAngle(
                { 0.0f, 0.0f, 1.0f }, std::atan2(animatedAhead.y, animatedAhead.x))).normalized();
            goal.landingOrientation = (landingSurface * animatedOrientation).normalized();
            if (input.footSpring && input.forceDrivenRoot)
                goal.landingOrientation = slerpQuaternion(goal.landingOrientation,
                    landingFlat, FootLandingFlatten).normalized();
            goal.landingPosition = { landingWorld.x, landingWorld.y,
                lockHeight(foot, landing.height, goal.landingOrientation) };
            goal.hipTravel = planarVelocity * secondsToTouchdown;
            goal.authoredLandingHorizontal = std::hypot(
                footInClip.x - next->hipInClip.x,
                footInClip.y - next->hipInClip.y);
            goal.authoredLandingVertical = next->hipInClip.z - footInClip.z;
            pelvisGround = from + rise * progress;
            // O pe chega orientado pelo chao de destino: comeca a se alinhar
            // com ele a 30% do balanco e esta alinhado a 75% (era de 60% ate o
            // toque: numa rampa ele ainda girava ao pousar).
            const float settle = input.footSpring
                ? smoothStep((progress - 0.30f) / 0.45f)
                : smoothStep((progress - 0.6f) / 0.4f);
            swingOrientation = (slerpQuaternion(Quaternion {},
                rotationBetween({ 0.0f, 0.0f, 1.0f }, landing.normal), settle)
                * animatedOrientation).normalized();
            // Pe com mola: perto do toque ele vai ficando paralelo ao chao de
            // pouso - fica 30% da inclinacao do clipe. Pelo clipe o trote
            // pousava com a ponta 25 graus para baixo (o tornozelo atrasado
            // disfarcava), e a ponta batia na quina do meio-fio.
            if (input.footSpring && input.forceDrivenRoot)
                swingOrientation = slerpQuaternion(swingOrientation, landingFlat,
                    FootLandingFlatten * smoothStep((progress - 0.50f) / 0.40f)).normalized();
        } else if (onGround) {
            const GroundSample ground = groundUnder(side, animatedWorld,
                animatedForward);
            swingGround = ground.height;
            pelvisGround = ground.height;
            foot.takeoffValid = false;
        } else {
            foot.takeoffValid = false;
        }
        if (next == nullptr || progress >= 0.999f) foot.takeoffValid = false;
        if (next != nullptr) m_telemetry.footSwingProgress[side] = progress;
        // O pe vai sendo levado para o ponto de pouso encaixado no degrau.
        Vec3 shift = next != nullptr && foot.fitValid
            ? foot.fitShift * smoothStep(progress / 0.8f) : Vec3 {};
        // O deslocamento do equilibrio entra ao longo do passo (sem salto na
        // saida); a trava do pouso o inclui (activeShift), o pe nao escorrega.
        if (next != nullptr) shift += balanceShift * smoothStep(progress / 0.6f);
        if (next != nullptr) shift += m_ballPlantShift[side];
        // TEMPTUNE: passada pelo contato - o pe que espera o toque desce no
        // lugar: desliza para tras em relacao ao corpo na velocidade dele (no
        // chao, quase parado), como o corredor puxa o pe no fim do balanco.
        // Parado em relacao a pelve (o ciclo esperando), ele chegava ao chao a
        // ~7 m/s para a frente e cravava como vara: a velocidade para a frente
        // virava salto (sprint: de -0,3 a +1,05 m/s para cima em 25 ms, perdendo
        // 1 m/s para a frente).
        // TEMPTUNE (ContactLandOffset > 0, no lugar do anterior): o pe que
        // espera vai para uma distancia fixa a frente do centro de massa e a
        // mantem ate tocar. Ancorado no ponto de pouso planejado, quanto mais o
        // voo durava mais o corpo passava por cima dele: depois de um voo longo
        // o pe tocava embaixo do corpo (12 cm), depois de um curto, longe (33
        // cm) - e cravava e lancava o corpo, alongando o voo seguinte: o sprint
        // mancava (um passo alto, um baixo).
        {
            Vec3& paw = m_waitPawShift[side];
            if (ContactLandOffset > 0.0f && m_waitingLeg == static_cast<int>(side) && !foot.planted) {
                const Vec3 body { physical.centerOfMassVelocityWorld.x, physical.centerOfMassVelocityWorld.y, 0.0f };
                const float bodySpeed = body.length();
                if (bodySpeed > 0.5f) {
                    const Vec3 direction = body * (1.0f / bodySpeed);
                    const Vec3 target = animatedWorld + shift + paw;
                    const float ahead = dot(Vec3 { target.x - physical.centerOfMassWorld.x,
                        target.y - physical.centerOfMassWorld.y, 0.0f }, direction);
                    paw += direction * ((ContactLandOffset - ahead) * (1.0f - std::exp(-deltaTime / 0.03f)));
                }
            } else if (ContactPaw && m_waitingLeg == static_cast<int>(side) && !foot.planted) {
                const Vec3 body { physical.centerOfMassVelocityWorld.x, physical.centerOfMassVelocityWorld.y, 0.0f };
                paw -= body * deltaTime;
                if (paw.length() > ContactPawMaximum) paw = paw * (ContactPawMaximum / paw.length());
            } else if (foot.planted) {
                paw = {};
            } else {
                paw = paw * std::exp(-deltaTime / 0.08f);
            }
            shift += paw;
        }
        // Sem cruzar as pernas (com pouca ajuda, correndo para a frente ou
        // para tras): o pe em balanco nao passa para o outro lado do pe de
        // apoio. O ponto de captura empurrava o pe esquerdo para a direita do
        // direito (0,4 m), as pernas cruzavam e ele caia na arrancada. (No
        // strafe a perna de tras cruza de proposito.)
        if (next != nullptr && lowAssist > 0.0f && CrossGuardWidth > 0.0f
            && (m_gaitDirection == CharacterGaitDirection3D::Forward
                || m_gaitDirection == CharacterGaitDirection3D::Backward)
            && m_feet[1 - side].valid && m_feet[1 - side].linkIndex < state.links.size()) {
            const Vec3 left = heading.rotate({ 0.0f, 1.0f, 0.0f });
            const Vec3 other = state.links[m_feet[1 - side].linkIndex].position;
            const float outward = side == 0 ? 1.0f : -1.0f;
            const Vec3 target = animatedWorld + shift;
            const float width = outward * dot(Vec3 { target.x - other.x, target.y - other.y, 0.0f }, left);
            if (width < CrossGuardWidth) shift += left * (outward * (CrossGuardWidth - width) * lowAssist);
        }
        foot.activeShift = shift;
        Vec3 swingTarget = animatedWorld + shift;
        // Chao embaixo do pe agora (os degraus do caminho).
        float floorNow = swingGround;
        if (next != nullptr && input.groundAt) {
            // O pe passa por cima do terreno que tem pela frente, nao sobe
            // antes para o degrau de pouso. Sondas a frente da ponta, no
            // rumo do caminho ate o pouso: cada espelho de degrau vira uma
            // rampa (1,1 de inclinacao, 3 cm de folga na quina), e atras
            // do calcanhar, para ele nao raspar a borda descendo. Subir para
            // o degrau de pouso cedo (a regra de antes) levava o pe, ainda
            // atras do quadril, a 48 cm do degrau embaixo dele - o coice.
            const Vec3 direction = foot.swingDirection.lengthSquared() > 0.5f
                ? foot.swingDirection : animatedForward;
            const GroundSample under = groundUnder(side, swingTarget,
                direction);
            floorNow = under.height;
            const auto probe = [&](float reach) {
                Vec3 point = swingTarget + direction * reach;
                point.z = baseGround;
                const GroundProbeResult3D hit = input.groundAt(point);
                return hit.hasSurface && hit.walkable ? hit.pointWorld.z
                    : -std::numeric_limits<float>::infinity();
            };
            // So o terreno ate o pouso: o degrau depois dele nao conta (o pe
            // pousava pendurado 12 cm acima). Atras, so dentro do pe (o pe
            // encaixado cabe inteiro no degrau de pouso).
            constexpr float HalfFoot = 0.14f, RampSlope = 1.1f,
                NoseClearance = 0.03f;
            const float remaining = dot(swingLanding - swingTarget,
                direction);
            // Atras, so segura o pe na altura de onde ele saiu ate o
            // calcanhar passar a borda (descendo); nunca o ergue.
            float needed = std::max(under.height, std::min(probe(-0.12f),
                foot.takeoffGroundHeight));
            for (const float ahead : { 0.02f, 0.08f, 0.14f, 0.20f }) {
                if (ahead > remaining) break;
                needed = std::max(needed, probe(HalfFoot + ahead)
                    + NoseClearance - RampSlope * ahead);
            }
            // Sobe na hora (a rampa ja da a distancia), no maximo 6 m/s.
            // Desce no ritmo que chega ao chao do pouso no toque.
            if (needed >= foot.swingFloor) {
                foot.swingFloor = std::min(needed,
                    foot.swingFloor + 6.0f * deltaTime);
            } else {
                const float fallRate = std::max(1.5f,
                    (foot.swingFloor - foot.landingGroundHeight)
                        / std::max(0.06f, secondsToTouchdown));
                foot.swingFloor = std::max(needed,
                    foot.swingFloor - fallRate * deltaTime);
            }
            swingGround = foot.swingFloor;
            // Chega ao degrau de pouso a tempo, mesmo se a sonda nao o viu.
            swingGround += (foot.landingGroundHeight - swingGround)
                * smoothStep((progress - 0.8f) / 0.2f);
            lift = 0.0f;
        }
        foot.groundHeight = swingGround;
        // Passo curto, pe mais baixo: a altura que o clipe da ao pe em
        // balanco (o calcanhar subindo atras, o joelho a frente) e a de uma
        // passada inteira. Com o passo encurtado (escada, devagar) ela
        // encolhe junto, ate 40%.
        const float flatHeight = lockHeight(foot, 0.0f, animatedOrientation);
        const float verticalScale = strideScale < 1.0f
            ? 1.0f - 0.6f * (1.0f - strideScale) : 1.0f;
        const float clipHeight = animatedWorld.z - baseGround - flatHeight;
        swingTarget.z = swingGround + flatHeight
            + std::max(0.0f, clipHeight) * verticalScale
            + std::min(0.0f, clipHeight) + lift;
        // Folga minima do balanco (modo fisico): no meio do passo o pe passa
        // pelo menos ~13 cm acima do chao. O strafe da referencia ergue o pe
        // so 7-12 cm; com a perna fisica acompanhando um pouco atrasada, o pe
        // rocava o chao a passada inteira (arrastando: 1,3 s de 2,2 s). So no
        // strafe: para a frente o clipe ja ergue 17 cm, e mais alto ele
        // passava por cima de obstaculos em que devia tropecar.
        const bool strafing = m_gaitDirection == CharacterGaitDirection3D::Left
            || m_gaitDirection == CharacterGaitDirection3D::Right;
        if (input.forceDrivenRoot && strafing && next != nullptr && onGround) {
            // Tira do chao cedo (ate ~30% do passo), segura e so desce no fim:
            // num arco em seno o pe saia devagar, ja deslizando de lado.
            const float p = std::clamp(progress, 0.0f, 1.0f);
            const float arch = smoothStep(p / 0.30f)
                * (1.0f - smoothStep((p - 0.65f) / 0.35f));
            const float clearance = swingTarget.z - (swingGround + flatHeight);
            const float wanted = SwingMinimumClearance * arch;
            if (clearance < wanted) swingTarget.z += wanted - clearance;
        }
        // Trote frontal: recolhe a ponta durante o meio do balanco. A altura
        // adicional e pequena e progressiva; evita que o atraso fisico do
        // tornozelo transforme a passada em uma bicuda no piso. Num tropeco
        // cedo, a estrategia de elevacao ergue o mesmo pe e continua o passo.
        // No sprint em curva tambem: o corpo inclinado e mais baixo, e a ponta
        // do pe de dentro raspava o chao saindo do apoio (~1,6 vez por curva
        // fechada).
        const float sprintCurveClear = SprintCurveClearStart > 0.0f && input.sprinting
            ? smoothStep((std::abs(m_bodyCurveAcceleration) - SprintCurveClearStart) / SprintCurveClearSpan) : 0.0f;
        const bool forwardJog = input.forceDrivenRoot && next != nullptr
            && onGround && m_gaitDirection == CharacterGaitDirection3D::Forward
            && (!input.sprinting || sprintCurveClear > 0.0f);
        if (forwardJog) {
            const float p = std::clamp(progress, 0.0f, 1.0f);
            const float curveShare = input.sprinting ? sprintCurveClear : 1.0f;
            const float early = std::abs(m_bodyCurveAcceleration) > 2.0f ? CurveClearEarly : 0.24f;
            const float arch = smoothStep(p / early)
                * (1.0f - smoothStep((p - 0.66f) / 0.27f)) * curveShare;
            const float clearance = swingTarget.z - (swingGround + flatHeight);
            float wanted = 0.18f * arch;
            if (m_tripElevating && m_tripFoot == static_cast<int>(side))
                wanted += 0.10f * m_tripRecovery
                    * (1.0f - smoothStep((p - 0.72f) / 0.20f));
            if (clearance < wanted) swingTarget.z += wanted - clearance;
            // Dorsiflexao: ponta aproximadamente 7 graus acima no meio do
            // balanco, voltando gradualmente ao plano do pouso.
            const Vec3 lateral = swingOrientation.rotate({ 0.0f, 1.0f, 0.0f });
            swingOrientation = (Quaternion::fromAxisAngle(lateral,
                -0.12f * arch) * swingOrientation).normalized();
        }
        // Com pouca ajuda o pe fisico fica para tras do alvo (a perna nao
        // acompanha o corpo que vem rapido, e na freada o alvo salta para a
        // frente): pelo tempo do clipe ele ja descia e tocava o chao 0,3-0,5 m
        // antes do pouso, atras do corpo - e o corpo tropecava nele. Enquanto
        // falta caminho, o alvo fica no alto: o pe "tenta" chegar e so desce
        // perto de onde vai pousar.
        if (ReachLiftPerMeter > 0.0f && input.forceDrivenRoot && next != nullptr && onGround
            && foot.linkIndex < state.links.size() && m_gaitDirection == CharacterGaitDirection3D::Forward) {
            const float lowAssistReach = smoothStep((0.5f - std::clamp(input.walkingAssist, 0.0f, 1.0f)) / 0.4f);
            const Vec3 physicalFoot = state.links[foot.linkIndex].position;
            const Vec3 direction = foot.swingDirection.lengthSquared() > 0.5f
                ? foot.swingDirection : animatedForward;
            const float behind = dot(Vec3 { swingTarget.x - physicalFoot.x, swingTarget.y - physicalFoot.y, 0.0f },
                Vec3 { direction.x, direction.y, 0.0f });
            const float wanted = std::min(ReachLiftMaximum,
                ReachLiftPerMeter * std::max(0.0f, behind - ReachLiftStart)) * lowAssistReach
                * (ReachLiftBrakeOnly ? m_brakeGait : 1.0f);
            const float clearance = swingTarget.z - (swingGround + flatHeight);
            if (wanted > 0.0f && clearance < wanted) swingTarget.z += wanted - clearance;
        }
        // Nenhum pe do alvo abaixo do chao que esta passando por baixo dele.
        const float lowestAllowed = lockHeight(foot, floorNow,
            swingOrientation) + (floorNow > swingGround + 0.01f ? 0.02f : 0.0f);
        swingTarget.z = std::max(swingTarget.z, lowestAllowed);
        if (foot.releaseBlend > 0.0f) {
            // A trava nao some num tick: o pe volta para o balanco em ~0,12 s.
            constexpr float ReleaseSeconds = 0.12f;
            foot.releaseBlend = std::max(0.0f,
                foot.releaseBlend - deltaTime / ReleaseSeconds);
            const float hold = smoothStep(foot.releaseBlend);
            swingTarget = swingTarget
                + (foot.releasePositionWorld - swingTarget) * hold;
            swingOrientation = slerpQuaternion(swingOrientation,
                foot.releaseOrientationWorld, hold);
        }
        goal.position = swingTarget;
        goal.orientation = swingOrientation;
        m_telemetry.footClipPitch[side] = -std::asin(std::clamp(
            animatedOrientation.rotate({ 1.0f, 0.0f, 0.0f }).z, -1.0f, 1.0f)) * 57.2958f;
        m_telemetry.footGoalPitch[side] = -std::asin(std::clamp(
            swingOrientation.rotate({ 1.0f, 0.0f, 0.0f }).z, -1.0f, 1.0f)) * 57.2958f;
        goal.pelvisGround = pelvisGround;
        // TEMPTUNE: correndo no plano (o pouso na altura de onde o pe saiu),
        // o pe no ar baixa a pelve no maximo SwingReachFlatCap. Na curva o
        // pouso mira longe para fora, e a pelve alvo descia ate 12 cm no voo
        // para a perna alcanca-lo; no apoio seguinte a perna levava o corpo
        // de volta para cima e ele decolava (saltinhos da curva). A regra e
        // para alcancar degrau mais baixo - em degrau ela continua inteira.
        if (SwingReachFlatCap >= 0.0f && next != nullptr && gaitMoving
            && std::abs(swingGround - baseGround) < 0.02f && std::abs(pelvisGround - baseGround) < 0.02f)
            goal.dropCap = SwingReachFlatCap;
        goal.reach = !onGround ? 0.0f
            : next != nullptr ? smoothStep((progress - 0.4f) / 0.5f)
            : 1.0f;
    }

    // An uncontrolled root uses its physical XY as the next input anchor;
    // feeding the support shift back into it would integrate a displacement.
    if (!input.controlled) standingShift.x = standingShift.y = 0.0f;
    targetRootPosition += standingShift;
    // Toque parado na bola com passo de apoio: a pelve vai junto (absoluto,
    // com peso, suavizado). A ancora da capsula nao inclui isto: a capsula
    // segue o corpo ate la.
    {
        Vec3 wanted {};
        if (input.controlled && input.ballBody.active && input.ballBody.weight > 0.0f) {
            const float w = std::clamp(input.ballBody.weight, 0.0f, 1.0f);
            wanted = { (input.ballBody.position.x - targetRootPosition.x) * w,
                (input.ballBody.position.y - targetRootPosition.y) * w, 0.0f };
        }
        m_ballBodyDelta += (wanted - m_ballBodyDelta) * (1.0f - std::exp(-deltaTime / 0.10f));
        targetRootPosition += m_ballBodyDelta;
    }

    // Pelve pelo alcance das pernas: desce o que for preciso para o quadril
    // alcancar cada pe apoiado (ou pousando) com a perna a 94% de esticada -
    // nem mais, nem menos. Descer ate o pe mais baixo (a regra de antes)
    // agachava o corpo na escada sem precisar; e com a capsula subindo as
    // quinas de forma continua, 8-16 cm acima do degrau de apoio, o pe recem
    // pousado ficava pendurado. Com os dois pes apoiados acima da base da
    // capsula, a pelve sobe.
    float drop = 0.0f;
    float raise = std::numeric_limits<float>::infinity();
    bool bothAbove = true;
    for (std::size_t side = 0; side < goals.size(); ++side) {
        const FootGoal& goal = goals[side];
        if (!goal.active || goal.reach <= 0.0f) {
            bothAbove = false;
            continue;
        }
        const RagdollLinkDefinition3D& footLink =
            profile.links[m_feet[side].linkIndex];
        const int shin = footLink.parentIndex;
        const int thigh = shin > 0
            ? profile.links[static_cast<std::size_t>(shin)].parentIndex : -1;
        if (thigh <= 0) continue;
        const RagdollLinkDefinition3D& shinLink =
            profile.links[static_cast<std::size_t>(shin)];
        const RagdollLinkDefinition3D& thighLink =
            profile.links[static_cast<std::size_t>(thigh)];
        const Vec3 hipLocal = profile.links[0].modelOrientation.conjugate()
            .rotate(thighLink.inboundJoint.anchorModelPosition
                - profile.links[0].modelPosition);
        const float legLength = (shinLink.inboundJoint.anchorModelPosition
                - thighLink.inboundJoint.anchorModelPosition).length()
            + (footLink.inboundJoint.anchorModelPosition
                - shinLink.inboundJoint.anchorModelPosition).length();
        const Vec3 hip = targetRootPosition
            + targetRootOrientation.rotate(hipLocal);
        const Vec3 anchorInFoot = footLink.modelOrientation.conjugate()
            .rotate(footLink.inboundJoint.anchorModelPosition
                - footLink.modelPosition);
        const float comfortable = legLength * ReachComfort;
        const auto needAt = [&](float horizontal, float vertical) {
            return horizontal >= comfortable ? vertical
                : vertical - std::sqrt(comfortable * comfortable
                    - horizontal * horizontal);
        };
        const auto needFor = [&](Vec3 ankle) {
            return needAt(std::hypot(hip.x - ankle.x, hip.y - ankle.y),
                hip.z - ankle.z);
        };
        const float need = needFor(goal.position
            + goal.orientation.rotate(anchorInFoot));
        const float authored = needFor(goal.animatedPosition
            + goal.animatedOrientation.rotate(anchorInFoot));
        drop = std::max(drop, std::min(goal.dropCap, std::max(0.0f, need - std::max(0.0f, authored))
            * goal.reach));
        if (goal.anticipate > 0.0f) {
            const Vec3 hipThen = hip + goal.hipTravel;
            const Vec3 ankle = goal.landingPosition
                + goal.landingOrientation.rotate(anchorInFoot);
            const float needThen = needAt(std::hypot(hipThen.x - ankle.x,
                hipThen.y - ankle.y), hipThen.z - ankle.z);
            const float authoredThen = needAt(goal.authoredLandingHorizontal,
                goal.authoredLandingVertical);
            drop = std::max(drop, std::min(goal.dropCap, std::max(0.0f,
                needThen - std::max(0.0f, authoredThen)) * goal.anticipate * ReachAnticipateScale));
        }
        const float above = goal.pelvisGround - baseGround;
        if (goal.reach < 0.999f || above <= 0.0f) bothAbove = false;
        raise = std::min(raise, above);
    }
    float pelvisTarget = -drop;
    if (bothAbove && std::isfinite(raise)) pelvisTarget += raise;
    if (!onGround || !footworkEnabled) pelvisTarget = 0.0f;
    pelvisTarget = std::clamp(pelvisTarget, -0.35f, 0.25f);
    const float PelvisOmega = ReachPelvisOmega;
    m_pelvisOffsetVelocity += (PelvisOmega * PelvisOmega
            * (pelvisTarget - m_pelvisOffset)
        - 2.0f * PelvisOmega * m_pelvisOffsetVelocity) * deltaTime;
    m_pelvisOffset += m_pelvisOffsetVelocity * deltaTime;
    targetRootPosition.z += m_pelvisOffset;
    m_telemetry.pelvisDropReachMeters = -m_pelvisOffset;
    // Freando, as pernas de apoio cedem ou seguram, nunca estendem: a pelve
    // de referencia nao fica acima da medida. Com o pe de freada a frente a
    // pelve desce para alcanca-lo; quando o corpo passava por cima dele a
    // referencia voltava a subir, a perna de apoio estendia (joelho de 90
    // para 24 graus em 0,1 s) e lancava o corpo como uma vara de salto - 3 m/s
    // para cima e 0,8 s no ar a 4,6 m/s.
    // (So com o corpo ainda rapido: devagar, sem a ajuda, a pelve que cedia
    // um pouco nunca mais subia - a teto vinha junto - e em 0,8 s de passos
    // de lado o centro de massa descia de 0,98 para 0,77 m e ele desabava.)
    if (m_brakeAnchor > 0.0f && input.forceDrivenRoot && !getUpActive && !state.links.empty()) {
        const float ceiling = state.links.front().position.z + BrakeRiseMeters;
        const float fast = smoothStep((std::hypot(m_filteredCenterOfMassVelocity.x,
            m_filteredCenterOfMassVelocity.y) - BrakeCeilingSlowSpeed) / BrakeCeilingSpan);
        if (targetRootPosition.z > ceiling)
            targetRootPosition.z += (ceiling - targetRootPosition.z) * m_brakeAnchor * m_brakeGait * fast;
    }

    // Pernas por IK ate cada pe (posicao e orientacao), com a pelve final.
    // Mesmo com a raiz livre (forceDrivenRoot), o IK parte da pelve
    // DESEJADA: assim a perna de apoio empurra o corpo para onde ele deve
    // estar (o pe esta travado no chao) - e o musculo que sustenta. Pela
    // pelve medida, quando ela cedia o IK dobrava a perna junto e nada a
    // empurrava de volta (medido: parado sem ajuda, so em pe assim).
    // A perna que o planejador tem NO AR e outra coisa: ela precisa pousar o
    // pe no alvo do mundo, e as juntas dela partem da pelve de verdade. Pela
    // desejada (reta, na capsula), com o corpo inclinado 20-40 graus por um
    // empurrao o pe do passo de recuperacao ia girado junto e ficava no ar
    // "procurando o chao" ate o corpo cair. A troca e gradual (a perna nao
    // salta quando o passo comeca nem quando pousa).
    const PhysicsBodyState3D& measuredPelvis = state.links.front();
    {
        m_sinceGaitFlip = std::min(10.0f, m_sinceGaitFlip + deltaTime);
        const Vec3 requested { input.intent.requestedVelocityWorld.x, input.intent.requestedVelocityWorld.y, 0.0f };
        const Vec3 body { m_filteredCenterOfMassVelocity.x, m_filteredCenterOfMassVelocity.y, 0.0f };
        const bool changing = (requested - body).length() > GaitChangeVelocityError
            || m_brakeGait > 0.0f || m_sinceGaitFlip < 0.4f;
        m_gaitChange = changing ? 1.0f : std::max(0.0f, m_gaitChange - deltaTime / 0.3f);
    }
    for (std::size_t side = 0; side < m_feet.size(); ++side) {
        const FootPhase3D phase = m_footwork.plan().feet[side].phase;
        // O balanco da passada tambem (TEMPTUNE): pela pelve de referencia o
        // pe tocava em media 9 cm (trote) e 15 cm (sprint) a frente do alvo -
        // a pelve fisica inclinada alguns graus diferente da de referencia
        // leva o pe junto - e a trava, no alvo, ficava atras do pe real.
        const bool gaitSwing = GaitSwingMeasuredPelvis && input.forceDrivenRoot && footworkEnabled
            && !m_footwork.owns(side) && gaitMoving && m_feet[side].valid && !m_feet[side].planted
            && desiredState != CharacterLocomotionState3D::Airborne;
        const bool swinging = (input.forceDrivenRoot && footworkEnabled && m_footwork.owns(side)
            && (phase == FootPhase3D::Swing || phase == FootPhase3D::TouchdownSearch)) || gaitSwing;
        float& weight = m_swingFromMeasuredPelvis[side];
        // (O da passada so com a ajuda andando alta: com pouca, a referencia
        // nao acompanha o corpo e o pe ia ao alvo dela, fora de baixo do
        // corpo - a 5%, 0 -> 9 quedas em 32 andando.)
        // (E so nas trocas de movimento: correndo estavel, pela altura medida a
        // perna pousava esticada com o corpo acima da referencia e o pe sem
        // carga escorregava - 2 -> 10-12 cm por apoio no sprint.)
        // (Decidido na saida do pe e mantido no balanco inteiro: ligando no
        // meio dele, a base do IK trocava com o pe no ar e o corte de 90
        // graus em sprint caia 3 em 8.)
        const bool gaitLegNow = !(input.forceDrivenRoot && footworkEnabled && m_footwork.owns(side));
        if (gaitSwing && gaitLegNow && !m_gaitSwingLatched[side]) {
            m_gaitSwingLatched[side] = true;
            m_gaitSwingCeiling[side] = smoothStep((std::clamp(input.walkingAssist, 0.0f, 1.0f) - 0.5f) / 0.4f)
                * (GaitSwingOnlyInChange ? m_gaitChange : 1.0f);
        }
        if (!gaitSwing) m_gaitSwingLatched[side] = false;
        const float ceiling = gaitSwing && gaitLegNow
            ? (GaitSwingLatch ? m_gaitSwingCeiling[side]
                : smoothStep((std::clamp(input.walkingAssist, 0.0f, 1.0f) - 0.5f) / 0.4f)
                    * (GaitSwingOnlyInChange ? m_gaitChange : 1.0f)) : 1.0f;
        // (A perna da passada que acabou de pousar sai da pelve medida no
        // ritmo GaitSwingRelease: em 0,15 s o apoio inteiro do sprint, ~0,1
        // s, era a troca de base, e a perna re-mirada empurrava o pe para a
        // frente - 12 cm de deslize por apoio.)
        const bool gaitPlanted = !(input.forceDrivenRoot && footworkEnabled && m_footwork.owns(side))
            && m_feet[side].valid && m_feet[side].planted;
        weight = swinging ? std::min(ceiling, weight + deltaTime / 0.06f)
            : gaitPlanted ? (GaitSwingReleaseSeconds <= 0.0f ? 0.0f
                : GaitSwingReleaseSeconds >= 9.0f ? weight
                : std::max(0.0f, weight - deltaTime / GaitSwingReleaseSeconds))
            : std::max(0.0f, weight - deltaTime / 0.15f);
    }
    // E5 (passos pelo planejador, andando): a capsula so segue o corpo e
    // fica para tras dele - as pernas de apoio resolvidas pela pelve
    // desejada (la atras) puxavam o corpo de volta e freavam a marcha. Andando
    // nesse modo, a posicao da pelve e a medida (no plano). A troca de base
    // e gradual (~0,25 s): parado, a referencia fica a ~5 cm da pelve fisica,
    // e trocar num tick saltava os alvos do quadril (0,06-0,09 rad) na
    // arrancada - o corpo ia para o lado errado antes do primeiro passo.
    {
        // Com a referencia corporal, na parada ela desacelera ate o repouso
        // e so entao devolve a base a referencia parada: trocando logo, a
        // base ia para a pelve de referencia (presa a capsula, que vem atras
        // do corpo) e as pernas puxavam o corpo de volta (+7 m/s2 recuando a
        // 0,9 m/s; ele passava do ponto e caia).
        const Vec3 com = m_estimator.state().centerOfMassVelocityWorld;
        const bool stillMoving = input.bodyReference && m_bodyRefValid
            && (m_bodyRefVelocity.length() > 0.05f || std::hypot(com.x, com.y) > 0.25f);
        // Com a referencia corporal ela e sempre a base das pernas de apoio,
        // andando ou parado: devolver a base a referencia parada (presa a
        // capsula, que so segue o corpo com folga) puxava o corpo na parada
        // (recuando: +4 a +6 m/s2 para a frente, passava do ponto e caia).
        const bool measuredBase = input.forceDrivenRoot && m_footwork.settings().locomotionStepping
            && (movementRequested || input.bodyReference);
        (void)stillMoving;
        m_legRootFromMeasured = measuredBase ? std::min(1.0f, m_legRootFromMeasured + deltaTime / 0.25f)
                                             : std::max(0.0f, m_legRootFromMeasured - deltaTime / 0.25f);
    }
    constexpr float LegBaseLeadSecondsSquared = 1.0f / 19.0f;
    constexpr float BodyRefAcceleration = 4.0f;
    constexpr float BodyRefLeashMeters = 0.10f;
    constexpr float BodyRefWalkMaxSpeed = 1.2f;
    constexpr float LegBaseLeadLimitMeters = 0.10f;
    // Alcance das pernas de apoio (R4): a extensao pedida comeca a ser cortada
    // com a perna a 90% do comprimento esticado e e toda cortada a 96%.
    constexpr float ReachComfortRatio = 0.80f;
    constexpr float ReachCriticalRatio = 0.90f;
    constexpr float BodyRefMaximumDrop = 0.08f;
    constexpr float BodyRefVerticalSpeed = 0.15f;
    Vec3 legRootPosition = targetRootPosition;
    if (m_legRootFromMeasured > 0.0f && input.bodyReference) {
        ++m_telemetry.bodyReferenceTicks;
        // Referencia corporal com estado: a velocidade vai ate a pedida com
        // aceleracao limitada, a posicao a integra, e a coleira a prende a
        // pelve medida (corpo atrasado ou bloqueado: nao acumula).
        const Vec3 pelvis { state.links.front().position.x, state.links.front().position.y, 0.0f };
        if (!m_bodyRefValid) {
            const Vec3 com = m_estimator.state().centerOfMassVelocityWorld;
            m_bodyRefPosition = pelvis;
            m_bodyRefVelocity = { com.x, com.y, 0.0f };
            m_bodyRefValid = true;
        }
        // A referencia pede o factivel: andando, a passada satura no alcance
        // (~0,86 m de percurso do pe) e o teto medido da marcha e ~1,1-1,2
        // m/s - acima disso o corpo passava da marcha (1,7 m/s pedindo 1,5)
        // e nao havia passo que o parasse. Mais rapido e corrida (E5-F). O
        // pedido do jogador nao muda.
        const Vec3 wanted = movementRequested
            ? clampMagnitude(Vec3 { input.intent.requestedVelocityWorld.x, input.intent.requestedVelocityWorld.y, 0.0f },
                BodyRefWalkMaxSpeed)
            : Vec3 {};
        m_bodyRefVelocity += clampMagnitude(wanted - m_bodyRefVelocity, BodyRefAcceleration * deltaTime);
        m_bodyRefPosition += m_bodyRefVelocity * deltaTime;
        const Vec3 error = m_bodyRefPosition - pelvis;
        if (error.length() > BodyRefLeashMeters)
            m_bodyRefPosition = pelvis + error * (BodyRefLeashMeters / error.length());
        // Transferencia de apoio (fase 4, do planejador): o centro de massa
        // vai a meta de apoio (o centro dos pes deslocado para o pe que
        // recebe - a mesma meta do esforco de contato). Na direcao da
        // transferencia a referencia de marcha e reancorada na pelve (senao,
        // parada, ela segurava o corpo: os dois se anulavam a 40% do caminho),
        // e a base ganha o que falta ate a meta menos a velocidade do centro de
        // massa (0,15 s) - zera na meta. (Como velocidade integrada, sem volta
        // de posicao, passava da meta, o outro pe saia do chao e ele caia.)
        Vec3 transferLead {};
        const auto& transfer = m_footwork.plan().supportTransfer;
        if (transfer.outgoingFoot >= 0) {
            const CharacterPhysicalState3D& physical = m_estimator.state();
            const Vec3 center = (physical.feet[0].soleCenterWorld + physical.feet[1].soleCenterWorld) * 0.5f;
            const Vec3 goal = center + (transfer.supportPointWorld - center) * transfer.progress;
            // A meta e do ponto de captura (DCM), nao do centro de massa:
            // andando o corpo ja vem com velocidade, e levar o COM a 85% do
            // caminho a cada passo balancava o corpo de lado (0,2 m/s) e
            // freava a marcha. O DCM inclui o amortecimento pela velocidade.
            const float omega = std::clamp(physical.captureOmega, 1.0f, 6.0f);
            const Vec3 dcm { physical.centerOfMassWorld.x + physical.centerOfMassVelocityWorld.x / omega,
                physical.centerOfMassWorld.y + physical.centerOfMassVelocityWorld.y / omega, 0.0f };
            Vec3 toGoal { goal.x - dcm.x, goal.y - dcm.y, 0.0f };
            // Andando, nunca contra o movimento pedido: para o pe da frente
            // sair (de lado, o do lado para onde vai) o apoio passa ao de tras
            // com o corpo acelerando para LONGE dele - no sentido da marcha -,
            // nao levando o corpo de volta ate ele. (Puxando para tras, a
            // transferencia brigava com a marcha e a descarga estourava o
            // prazo.)
            {
                const float wantedSpeed = std::hypot(wanted.x, wanted.y);
                if (wantedSpeed > 0.1f) {
                    const Vec3 forward = wanted * (1.0f / wantedSpeed);
                    const float backward = dot(toGoal, forward);
                    if (backward < 0.0f) toGoal -= forward * backward;
                }
            }
            if (toGoal.length() > 0.001f) {
                // Reancora a referencia de marcha na direcao da transferencia;
                // andando, so a parte atravessada ao movimento pedido (a
                // frente e da marcha).
                Vec3 along = toGoal * (1.0f / toGoal.length());
                const float wantedSpeed = std::hypot(wanted.x, wanted.y);
                if (wantedSpeed > 0.1f) {
                    const Vec3 forward = wanted * (1.0f / wantedSpeed);
                    along = along - forward * dot(along, forward);
                }
                if (along.length() > 0.01f) {
                    along = along * (1.0f / along.length());
                    m_bodyRefPosition -= along * dot(m_bodyRefPosition - pelvis, along);
                }
                transferLead = clampMagnitude(toGoal, BodyRefLeashMeters);
            }
        }
        legRootPosition.x += (m_bodyRefPosition.x + transferLead.x - legRootPosition.x) * m_legRootFromMeasured;
        legRootPosition.y += (m_bodyRefPosition.y + transferLead.y - legRootPosition.y) * m_legRootFromMeasured;
    } else if (m_legRootFromMeasured > 0.0f) {
        m_bodyRefValid = false;
        legRootPosition.x += (state.links.front().position.x - legRootPosition.x) * m_legRootFromMeasured;
        legRootPosition.y += (state.links.front().position.y - legRootPosition.y) * m_legRootFromMeasured;
        // A base prevista (guia 17): a pelve medida mais o que o controle do
        // corpo pediu de aceleracao ao centro de massa (antes do limite do
        // apoio: a queda passiva do pendulo ja acontece sozinha). Os motores das pernas
        // sao molas rigidas em volta do alvo; resolvida da pelve medida, a
        // perna so segurava o corpo onde ele estava - a troca de peso da
        // descarga e a velocidade pedida, so em torque de antecipacao, nao
        // moviam o corpo (medido: 170 N pedidos por 0,2 s, COM parado; a
        // mesma troca pelos alvos, 8 cm, levou o COM a 0,3 m/s). ~19 s^-2 e a
        // rigidez horizontal medida das pernas de apoio (0,7 Hz).
        Vec3 lead { input.legBaseAccelerationWorld.x, input.legBaseAccelerationWorld.y, 0.0f };
        lead = lead * LegBaseLeadSecondsSquared;
        if (lead.length() > LegBaseLeadLimitMeters) lead = lead * (LegBaseLeadLimitMeters / lead.length());
        legRootPosition += lead * m_legRootFromMeasured;
    } else {
        m_bodyRefValid = false;
    }
    // Alcance das pernas de apoio (R4 do pacote v2). A vantagem no plano da
    // referencia corporal, com a raiz do IK na mesma altura, alongava a
    // perna de tras (quadril a frente e acima do tornozelo): os motores a
    // esticavam ao longo do eixo, que e mais vertical que horizontal, e o
    // corpo subia 3,5 cm e saia do chao andando (voo de ate 67 ms). Aqui a
    // parte da extensao pedida a cada perna de apoio e cortada conforme ela
    // chega perto do alcance, e a raiz desce o que for preciso para o
    // comprimento ficar: a pelve gira em volta do tornozelo (o arco do
    // pendulo) em vez de a perna esticar. Com dois pes no chao vale a perna
    // que pede a raiz mais baixa. So o alvo das juntas muda; nada na pelve.
    const auto legGeometry = [&](std::size_t side, Vec3& hipLocal, Vec3& ankleLocal, float& nominal) {
        const FootPlant& plant = m_feet[side];
        const int shin = plant.valid ? profile.links[plant.linkIndex].parentIndex : -1;
        const int thigh = shin > 0 ? profile.links[static_cast<std::size_t>(shin)].parentIndex : -1;
        if (thigh <= 0) return false;
        const RagdollLinkDefinition3D& footLink = profile.links[plant.linkIndex];
        const Vec3 hipModel = profile.links[static_cast<std::size_t>(thigh)].inboundJoint.anchorModelPosition;
        const Vec3 kneeModel = profile.links[static_cast<std::size_t>(shin)].inboundJoint.anchorModelPosition;
        const Vec3 ankleModel = footLink.inboundJoint.anchorModelPosition;
        hipLocal = profile.links[0].modelOrientation.conjugate().rotate(hipModel - profile.links[0].modelPosition);
        ankleLocal = footLink.modelOrientation.conjugate().rotate(ankleModel - footLink.modelPosition);
        nominal = (kneeModel - hipModel).length() + (ankleModel - kneeModel).length();
        return nominal > 0.1f;
    };
    if (m_legRootFromMeasured > 0.0f && input.bodyReference && input.bodyReachProjection && m_bodyRefValid) {
        // A altura segue a da pelve de referencia a no maximo 0,15 m/s: da
        // base parada em alerta (0,80 m) para a de marcha (0,91) ela subia
        // em 0,3 s, a 0,5 m/s, e as pernas de apoio lancavam o corpo (voo de
        // 67 ms na arrancada).
        if (m_bodyRefHeight <= 0.0f) m_bodyRefHeight = legRootPosition.z;
        m_bodyRefHeight += std::clamp(legRootPosition.z - m_bodyRefHeight,
            -BodyRefVerticalSpeed * deltaTime, BodyRefVerticalSpeed * deltaTime);
        float heightLimit = m_bodyRefHeight;
        for (std::size_t side = 0; side < m_feet.size(); ++side) {
            LegReachTelemetry3D& reach = m_telemetry.legReach[side];
            reach.verticalCorrection = 0.0f;
            reach.projectionGain = 0.0f;
            Vec3 hipLocal, ankleLocal;
            float nominal = 0.0f;
            if (!goals[side].active || !m_feet[side].planted || !m_footwork.owns(side)
                || !legGeometry(side, hipLocal, ankleLocal, nominal)) continue;
            const Vec3 ankle = goals[side].position + goals[side].orientation.rotate(ankleLocal);
            const Vec3 hipOffset = targetRootOrientation.rotate(hipLocal);
            const Vec3 hip = Vec3 { legRootPosition.x, legRootPosition.y, heightLimit } + hipOffset;
            const float length = (hip - ankle).length();
            const float previous = m_previousLegLength[side][1];
            if (previous < 0.0f || length <= previous) continue;
            const float gain = smoothStep((length / nominal - ReachComfortRatio)
                / (ReachCriticalRatio - ReachComfortRatio));
            float allowed = previous + (length - previous) * (1.0f - gain);
            allowed = std::min(allowed, std::max(previous, ReachCriticalRatio * nominal));
            if (allowed >= length) continue;
            const float planar = std::hypot(hip.x - ankle.x, hip.y - ankle.y);
            if (planar >= allowed) continue;
            const float height = ankle.z - hipOffset.z + std::sqrt(allowed * allowed - planar * planar);
            reach.projectionGain = gain;
            if (height < heightLimit) {
                reach.verticalCorrection = heightLimit - height;
                heightLimit = height;
            }
        }
        // A restricao de alcance desce a altura na hora, nunca mais de 8 cm
        // abaixo da referencia (alem disso a perna esta no fim e o passo tem
        // de sair).
        m_bodyRefHeight = std::max(heightLimit, legRootPosition.z - BodyRefMaximumDrop);
        legRootPosition.z += (m_bodyRefHeight - legRootPosition.z) * m_legRootFromMeasured;
    } else {
        m_bodyRefHeight = 0.0f;
    }
    // Tornozelo contra a queda (pe com mola, modo fisico): o corpo saindo da
    // velocidade da referencia (empurrado, tropecando, passando do ponto)
    // leva o ponto de captura para fora. O pe de apoio pressiona do lado
    // para onde ele vai - a ponta caindo para a frente, o calcanhar para
    // tras, a borda de lado -: o alvo do pe gira em volta do tornozelo, o
    // chao nao deixa o pe girar e o motor empurra o corpo de volta (o centro
    // de pressao anda dentro do pe, a estrategia do tornozelo). A posicao ja
    // tem a mola das pernas de apoio (resolvidas da pelve de referencia);
    // isto antecipa pela velocidade. Fora da janela da mola do toque.
    {
        const CharacterPhysicalState3D& physical = m_estimator.state();
        Vec3 wanted {};
        if (input.footSpring && input.ankleBalance && input.forceDrivenRoot && !getUpActive && physical.valid) {
            const float omega = std::clamp(physical.captureOmega, 1.0f, 6.0f);
            wanted = { (physical.centerOfMassVelocityWorld.x - planarVelocity.x) / omega,
                (physical.centerOfMassVelocityWorld.y - planarVelocity.y) / omega, 0.0f };
        }
        m_ankleBalanceShift += (wanted - m_ankleBalanceShift) * (1.0f - std::exp(-deltaTime / 0.05f));
        // Zona morta: o balanco de quem esta parado (alguns cm/s) nao mexe o
        // tornozelo - reagindo a ele, o corpo parado oscilava (contato
        // perdido 0 -> 4 em 15 nascimentos).
        Vec3 pressShift = m_ankleBalanceShift;
        {
            const float size = pressShift.length();
            pressShift = size > AnkleBalanceDeadband
                ? pressShift * ((size - AnkleBalanceDeadband) / size) : Vec3 {};
        }
        m_telemetry.ankleBalancePitch = 0.0f;
        m_telemetry.ankleBalanceRoll = 0.0f;
        for (std::size_t side = 0; side < m_feet.size() && wanted.lengthSquared() > 0.0f; ++side) {
            FootGoal& goal = goals[side];
            const FootSupportEstimate3D& estimate = physical.feet[side];
            Vec3 hipLocal, ankleLocal;
            float nominal = 0.0f;
            if (!goal.active || !m_feet[side].planted || !estimate.supporting
                || !legGeometry(side, hipLocal, ankleLocal, nominal)) continue;
            const float engage = smoothStep((estimate.supportingSeconds - AnkleLandingSeconds) / 0.10f);
            Vec3 forward = goal.orientation.rotate({ 1.0f, 0.0f, 0.0f });
            forward.z = 0.0f;
            if (engage <= 0.0f || forward.lengthSquared() < 1e-4f) continue;
            forward = forward.normalized();
            const Vec3 left = cross(Vec3 { 0.0f, 0.0f, 1.0f }, forward);
            const float pitch = engage * std::clamp(AnkleBalanceGain * dot(pressShift, forward),
                -AnkleBalancePitchLimit, AnkleBalancePitchLimit);
            const float roll = engage * std::clamp(AnkleBalanceGain * dot(pressShift, left),
                -AnkleBalanceRollLimit, AnkleBalanceRollLimit);
            const Quaternion press = (Quaternion::fromAxisAngle(left, pitch)
                * Quaternion::fromAxisAngle(forward, -roll)).normalized();
            const Vec3 ankle = goal.position + goal.orientation.rotate(ankleLocal);
            goal.orientation = (press * goal.orientation).normalized();
            goal.position = ankle + press.rotate(goal.position - ankle);
            if (std::abs(pitch) > std::abs(m_telemetry.ankleBalancePitch)) m_telemetry.ankleBalancePitch = pitch;
            if (std::abs(roll) > std::abs(m_telemetry.ankleBalanceRoll)) m_telemetry.ankleBalanceRoll = roll;
        }
    }
    std::array<Vec3, 2> ikRootPosition { legRootPosition, legRootPosition };
    std::array<Quaternion, 2> ikRootOrientation { targetRootOrientation, targetRootOrientation };
    // Preparo do pouso: descendo, a perna aponta para onde o corpo vai bater
    // - na linha da velocidade (uma fracao do angulo dela com a vertical),
    // com o joelho um pouco dobrado para a mola. Com o pe embaixo do quadril
    // e o corpo ainda indo para a frente, ele girava sobre o pe e era jogado
    // para a frente (o peito ate 52 graus no pulo em sprint); alinhada, a
    // forca do chao passa pelo centro de massa. No referencial da pelve
    // fisica: no ar e a orientacao dela que vale.
    m_landingPrep = 0.0f;
    if (LandingPrepEnabled && input.forceDrivenRoot && desiredState == CharacterLocomotionState3D::Airborne
        && !state.links.empty()) {
        const Vec3 velocity = m_filteredCenterOfMassVelocity;
        const float horizontalSpeed = std::hypot(velocity.x, velocity.y);
        // So perto do toque (~0,25 s antes): no ar, levar as pernas a frente
        // gira o tronco para tras (o momento angular se conserva) - feito
        // cedo, numa queda de 1 m ele girava ate cair de costas.
        float soleHeight = std::numeric_limits<float>::infinity();
        for (std::size_t side = 0; side < m_feet.size(); ++side) {
            const PhysicsBodyState3D& footBody = state.links[m_feet[side].linkIndex];
            const RagdollLinkDefinition3D& footLink = profile.links[m_feet[side].linkIndex];
            const GroundSample ground = groundUnder(side, footBody.position,
                footForwardWorld3D(footLink, footBody.orientation));
            if (ground.valid) soleHeight = std::min(soleHeight, footBody.position.z - ground.height);
        }
        const float fall = std::max(0.0f, -velocity.z);
        const float touchdown = std::isfinite(soleHeight)
            ? (-fall + std::sqrt(fall * fall + 2.0f * 9.81f * std::max(0.0f, soleHeight))) / 9.81f : 0.0f;
        const float prep = smoothStep((-velocity.z - 0.3f) / 1.2f)
            * smoothStep((LandingLeadSeconds - touchdown) / 0.15f);
        if (prep > 0.0f && horizontalSpeed > 0.3f) {
            m_landingPrep = prep;
            const Vec3 direction { velocity.x / horizontalSpeed, velocity.y / horizontalSpeed, 0.0f };
            const float alpha = std::atan2(horizontalSpeed, std::max(0.05f, -velocity.z));
            // TEMPTUNE LandRunLead: pousando para seguir correndo, so a perna
            // mais baixa (a da frente da tesoura do pulo) vai ao chao, menos
            // inclinada; a de tras fica atras, pronta para o passo seguinte.
            // As duas a frente, na linha da velocidade, ele pousava sentado
            // (coxas a -100 graus), freava e o tronco tombava para a frente.
            const float runningOn = LandRunLead
                ? smoothStep((horizontalSpeed - 2.5f) / 1.5f)
                    * smoothStep(std::hypot(input.intent.requestedVelocityWorld.x,
                        input.intent.requestedVelocityWorld.y) / 1.5f) : 0.0f;
            const float beta = std::min(LandingAlign * alpha,
                LandingMaxAngle + (LandRunMaxAngle - LandingMaxAngle) * runningOn);
            int leadSide = -1;
            if (runningOn > 0.5f) {
                float lowest = std::numeric_limits<float>::infinity();
                for (std::size_t side = 0; side < m_feet.size(); ++side) {
                    const float z = state.links[m_feet[side].linkIndex].position.z;
                    if (z < lowest) { lowest = z; leadSide = static_cast<int>(side); }
                }
            }
            const Vec3 legWorld = direction * std::sin(beta) + Vec3 { 0.0f, 0.0f, -std::cos(beta) };
            const Quaternion pelvisInverse = state.links.front().orientation.conjugate();
            const RagdollLinkDefinition3D& pelvisLink = profile.links[0];
            // A coluna na mesma linha: o tronco inclina para tras uma parte do
            // angulo da perna (o centro de massa fica sobre a linha do pe ao
            // longo da velocidade - a mola trabalha nela, sem girar o corpo).
            // Pela inclinacao real da pelve: a coluna e relativa a ela.
            if (LandingTrunkAlign > 0.0f) {
                const Vec3 pelvisUp = state.links.front().orientation.rotate({ 0.0f, 0.0f, 1.0f });
                const float pelvisAngle = std::atan2(dot(pelvisUp, direction), pelvisUp.z);
                // (De lado menos: o quadril abre menos do que flexiona e a perna
                // alinha menos - inclinado o mesmo, de lado ele caia de costas.)
                const Vec3 pelvisForward = state.links.front().orientation.rotate({ 1.0f, 0.0f, 0.0f });
                const float forwardness = std::abs(dot(Vec3 { pelvisForward.x, pelvisForward.y, 0.0f }.normalized(), direction));
                const float wanted = -LandingTrunkAlign * beta
                    * (LandingTrunkSide + (1.0f - LandingTrunkSide) * forwardness);
                const float correction = std::clamp(wanted - pelvisAngle, -0.7f, 0.4f);
                // Rotacao em torno do eixo a esquerda do rumo, levada para a
                // pelve: arfagem (eixo 1 da coluna) e rolagem (eixo 2).
                const Vec3 axisLocal = pelvisInverse.rotate({ -direction.y, direction.x, 0.0f });
                const float pitch = correction * axisLocal.y;
                const float roll = LandingRollSign * correction * axisLocal.x;
                const float shares[3] = { 0.30f, 0.35f, 0.35f };
                const char* spine[3] = { "Abdomen", "Chest", "UpperChest" };
                for (std::size_t i = 0; i < 3; ++i) {
                    for (std::size_t index = 1; index < profile.links.size(); ++index) {
                        if (profile.links[index].id != spine[i]) continue;
                        blendJointCoordinate(profile, m_coordinates, index, 1, pitch * shares[i], prep);
                        blendJointCoordinate(profile, m_coordinates, index, 2, roll * shares[i], prep);
                    }
                }
            }
            for (std::size_t side = 0; side < m_feet.size(); ++side) {
                if (!goals[side].active) continue;
                if (leadSide >= 0 && static_cast<int>(side) != leadSide) continue;
                const RagdollLinkDefinition3D& footLink = profile.links[m_feet[side].linkIndex];
                const int shin = footLink.parentIndex;
                const int thigh = shin > 0 ? profile.links[static_cast<std::size_t>(shin)].parentIndex : -1;
                if (thigh <= 0) continue;
                const Vec3 hipModel = profile.links[static_cast<std::size_t>(thigh)].inboundJoint.anchorModelPosition;
                const Vec3 hipLocal = pelvisLink.modelOrientation.conjugate().rotate(hipModel - pelvisLink.modelPosition);
                const float legLength = LandingLegShare * (hipModel - footLink.inboundJoint.anchorModelPosition).length();
                const Vec3 wantedLocal = hipLocal + pelvisInverse.rotate(legWorld * legLength);
                const Vec3 currentLocal = ikRootOrientation[side].conjugate().rotate(goals[side].position - ikRootPosition[side]);
                goals[side].position = ikRootPosition[side] + ikRootOrientation[side].rotate(
                    currentLocal + (wantedLocal - currentLocal) * prep);
            }
        }
    }
    // TEMPTUNE: altura do voo regulada passo a passo (Raibert). Correndo, o
    // corpo e uma massa numa mola: sem regulagem, o empurrao de cada apoio
    // dependia de quando o pe tocou no ciclo, e uns passos lancavam o dobro
    // de outros (sprint: voos +-5 cm, pior +9; em curva +-11). Cada voo e
    // comparado com o da referencia (a pelve guia no mesmo voo): subiu
    // demais, no apoio seguinte a perna de apoio estende menos (o alvo da
    // pelve dela desce); ficou baixo, estende mais. So pela perna - nenhuma
    // forca de fora.
    {
        const auto& feetNow = m_estimator.state().feet;
        const bool anySupport = feetNow.size() >= 2 && (feetNow[0].supporting || feetNow[1].supporting);
        if (ThrustGain > 0.0f && gaitMoving && input.forceDrivenRoot && !getUpActive) {
            if (!anySupport) {
                m_flightBodyApex = m_inFlight ? std::max(m_flightBodyApex, measuredPelvis.position.z) : measuredPelvis.position.z;
                m_flightGuideApex = m_inFlight ? std::max(m_flightGuideApex, targetRootPosition.z) : targetRootPosition.z;
                m_inFlight = true;
            } else if (m_inFlight) {
                m_inFlight = false;
                m_flightApexError = m_flightBodyApex - m_flightGuideApex;
            }
            const float wanted = anySupport
                ? std::clamp(-ThrustGain * (m_flightApexError - ThrustApexMargin), -ThrustMaximum, ThrustMaximum) : 0.0f;
            m_thrustOffset += (wanted - m_thrustOffset) * (1.0f - std::exp(-deltaTime / 0.03f));
        } else if (ThrustGain <= 0.0f) {
            m_thrustOffset = 0.0f;
        }
        // TEMPTUNE: amortecimento vertical pela perna de apoio. Subindo mais
        // rapido que a referencia, a perna estende menos (o alvo da pelve dela
        // desce); descendo mais rapido (o pouso), segura mais. Sem ele, na
        // curva de sprint a perna seguia estendendo com o corpo ja subindo e
        // ele decolava 15-20 cm acima da referencia, caia o voo inteiro e
        // pousava 15 cm abaixo: o corpo oscilava 40 cm (pelve de 0,69 a 1,08 m).
        {
            const float guideRate = m_stanceDampingValid
                ? (targetRootPosition.z - m_stanceDampingGuideZ) / std::max(1e-4f, deltaTime) : 0.0f;
            m_stanceDampingGuideZ = targetRootPosition.z;
            m_stanceDampingValid = true;
            float wanted = 0.0f;
            // (So na corrida livre: no tropeco o corpo cai e a perna, segurando
            // mais, o empurrava contra a barra - sprint na barra de 26 cm, 13 ->
            // 38 quedas em 96; freando, idem.)
            const bool stumblingNow = m_tripRecovery > 0.001f || m_stumbleRisk > 0.2f
                || m_stumbleStepping || m_footwork.plan().recovering;
            const bool brakingNow = m_brakeGait > 0.0f || m_brakeStepping;
            if (StanceVerticalDamping > 0.0f && anySupport && gaitMoving && input.forceDrivenRoot && !getUpActive
                && !(StanceDampFree && (stumblingNow || brakingNow)))
                wanted = std::clamp(-StanceVerticalDamping * (measuredPelvis.linearVelocity.z - guideRate),
                    -ThrustMaximum, ThrustMaximum)
                    // (So na curva, pela inclinacao: na reta a corrida ja nao
                    // saltava e, achatada, o pe passava mais baixo e enganchava
                    // mais na barra escondida - 13 -> 30 quedas em 192.)
                    * (StanceDampCurveSpan > 0.0f
                        ? smoothStep(std::abs(m_bodyCurveLean) / StanceDampCurveSpan) : 1.0f);
            m_stanceDampingOffset += (wanted - m_stanceDampingOffset) * (1.0f - std::exp(-deltaTime / 0.02f));
            m_thrustOffset = (ThrustGain > 0.0f ? m_thrustOffset : 0.0f) + m_stanceDampingOffset;
        }
        m_telemetry.thrustOffsetMeters = m_thrustOffset;
    }
    for (std::size_t side = 0; side < m_feet.size(); ++side) {
        if (m_thrustOffset != 0.0f && side < m_estimator.state().feet.size()
            && m_estimator.state().feet[side].supporting && !m_footwork.owns(side))
            ikRootPosition[side].z += m_thrustOffset;
        // TEMPTUNE: correndo, a perna de apoio pede no maximo StanceRootBand
        // acima (ou abaixo) da altura de verdade da pelve. Servo rigido da
        // referencia, depois de um pouso 20 cm abaixo dela a perna estendia os
        // 20 cm de uma vez (3 g) e passava do ponto: o voo seguinte ia 30 cm
        // acima, o pouso seguinte mais fundo - a curva de sprint oscilava
        // 50 cm. Assim ela devolve o corpo a altura em alguns passos.
        // Experimento v2 (T_StanceMeasured): com pouca ajuda a perna de apoio
        // da passada e resolvida pela pelve de VERDADE no plano (a altura e a
        // orientacao seguem as da referencia). Pela de referencia o motor da
        // perna e uma mola que prende o corpo nela: o impulso (o esforco das
        // pernas, por J^T) era anulado assim que o corpo saia dela, e sem
        // ajuda o corpo nao passava de ~2 m/s.
        {
            static const float stanceMeasured = tuneTemp("T_StanceMeasured", 0.0f);  // TEMPTUNE
            static const float stanceMeasuredFrom = tuneTemp("T_StanceMeasuredFrom", 0.2f);  // TEMPTUNE
            static const float stanceMeasuredSpan = tuneTemp("T_StanceMeasuredSpan", 0.4f);  // TEMPTUNE
            if (stanceMeasured > 0.0f && gaitMoving && input.forceDrivenRoot && !getUpActive
                && side < m_estimator.state().feet.size() && m_estimator.state().feet[side].supporting
                && !m_footwork.owns(side) && m_feet[side].planted) {
                const float w = stanceMeasured * (1.0f - smoothStep((std::clamp(input.walkingAssist, 0.0f, 1.0f)
                    - stanceMeasuredFrom) / std::max(0.01f, stanceMeasuredSpan)));
                ikRootPosition[side].x += (measuredPelvis.position.x - ikRootPosition[side].x) * w;
                ikRootPosition[side].y += (measuredPelvis.position.y - ikRootPosition[side].y) * w;
            }
        }
        if (StanceRootBand > 0.0f && gaitMoving && input.forceDrivenRoot && !getUpActive
            && side < m_estimator.state().feet.size() && m_estimator.state().feet[side].supporting
            && !m_footwork.owns(side)) {
            ikRootPosition[side].z = std::clamp(ikRootPosition[side].z,
                measuredPelvis.position.z - StanceRootBand, measuredPelvis.position.z + StanceRootBand);
        }
        const float measured = m_swingFromMeasuredPelvis[side];
        if (measured > 0.0f) {
            // Da passada (modos 3 e 4): so no plano - a altura fica a da
            // referencia. Pela altura medida, com o corpo acima dela a perna
            // alcancava o chao esticada, pousava como vara e o devolvia para
            // cima a cada passada (sprint: pelve 8,5 cm acima da referencia
            // no toque, o pe quase sem carga escorregava 10 cm).
            const bool gaitLeg = !m_footwork.owns(side);
            const int mode = gaitLeg ? GaitSwingMeasuredMode : 0;
            if (mode != 2) {
                ikRootPosition[side] = legRootPosition
                    + (measuredPelvis.position - legRootPosition) * measured;
                if (mode == 3 || mode == 4) ikRootPosition[side].z = legRootPosition.z;
                // (Modo 5: medida, mas nunca acima da referencia - com o corpo
                // alto a perna nao estica ate o chao.)
                if (mode == 5) ikRootPosition[side].z = std::min(ikRootPosition[side].z, legRootPosition.z);
            }
            if (mode != 1 && mode != 4)
                ikRootOrientation[side] = slerpQuaternion(targetRootOrientation,
                    measuredPelvis.orientation, measured);
        }
        // Corpo mais baixo que a referencia (curva inclinada, freada, pouso):
        // a perna da passada no ar parte da altura de verdade da pelve. Pela
        // de referencia, o pe no fim do balanco ficava 10-15 cm abaixo do alvo
        // e batia a ponta no chao a 5-9 m/s ("bicuda") - na curva fechada, o
        // pe de dentro tocava o chao no meio do balanco ~2 vezes por curva.
        // Acima da referencia nada muda (a perna nao estica ate o chao).
        {
            const bool gaitSwingLeg = SwingFromLowBody > 0.0f && input.forceDrivenRoot && gaitMoving
                && !m_footwork.owns(side) && m_feet[side].valid && !m_feet[side].planted
                && desiredState != CharacterLocomotionState3D::Airborne;
            // (So com a ajuda andando alta: com pouca, a passada inteira mudava
            // - a 5% andava a 2,3 m/s em vez de 1,6 e caia parando 11 em 32.)
            const float lowCeiling = SwingFromLowBody
                * smoothStep((std::clamp(input.walkingAssist, 0.0f, 1.0f) - 0.5f) / 0.4f);
            float& low = m_swingLowWeight[side];
            low = gaitSwingLeg ? std::min(lowCeiling, low + deltaTime / 0.06f)
                : std::max(0.0f, low - deltaTime / std::max(0.01f, SwingFromLowBodyRelease));
            // TEMPTUNE: so na subida e no meio do balanco (a folga do pe); no
            // fim dele a perna volta a altura da referencia e pousa estendida.
            // Pela altura de verdade ate o toque, com o corpo baixo na curva a
            // perna pousava dobrada (quadril a 45, joelho a 50) e, apoiada, ainda
            // partia da pelve baixa por SwingFromLowBodyRelease - nada levava o
            // corpo de volta para cima: a curva inteira agachada.
            // TEMPTUNE: passada pelo contato - a perna que espera o toque busca
            // o chao a partir da altura de verdade do corpo (estende para
            // alcancar). Pela referencia, ~5 cm abaixo do corpo no voo do
            // sprint, ela esperava dobrada (joelho a 45) e, no toque, o apoio
            // puxava o corpo para baixo (joelho pedido 58). Depois do toque a
            // referencia volta aos poucos e a perna amortece.
            {
                float& reach = m_landingReach[side];
                reach = m_waitingLeg == static_cast<int>(side)
                    ? std::min(1.0f, reach + deltaTime / 0.05f)
                    : std::max(0.0f, reach - deltaTime / std::max(0.01f, ContactLandRelease));
                if (reach > 0.0f && ContactLandReach) {
                    // (Ate ContactLandReachMax acima da referencia: alcancando de
                    // qualquer altura, pousava esticada como vara e o devolvia
                    // mais alto a cada passo - sprint com a pelve a 1,01 m,
                    // acima da de pe.)
                    const float above = std::clamp(measuredPelvis.position.z - ikRootPosition[side].z,
                        0.0f, ContactLandReachMax);
                    ikRootPosition[side].z += above * reach;
                }
            }
            float& landing = m_swingLowLanding[side];
            {
                const float progress = m_telemetry.footSwingProgress[side];
                const float wanted = SwingLowLandingStart >= 1.0f ? 1.0f
                    : m_feet[side].planted || progress < 0.0f ? 0.0f
                    : 1.0f - smoothStep((progress - SwingLowLandingStart) / SwingLowLandingSpan);
                landing += (wanted - landing) * (1.0f - std::exp(-deltaTime / 0.04f));
            }
            if (low > 0.0f && measured <= 0.0f) {
                const float below = std::min(0.0f, measuredPelvis.position.z - ikRootPosition[side].z);
                ikRootPosition[side].z += below * low * landing;
                if (SwingFromBodyTilt > 0.0f) {
                    // O quanto a pelve de verdade esta inclinada alem da de
                    // referencia (sem o giro em torno da vertical).
                    const Vec3 upLocal = profile.links.front().modelOrientation.conjugate().rotate({ 0.0f, 0.0f, 1.0f });
                    const Vec3 upReference = ikRootOrientation[side].rotate(upLocal);
                    const Vec3 upMeasured = measuredPelvis.orientation.rotate(upLocal);
                    const Quaternion tilt = rotationBetween(upReference, upMeasured);
                    const Quaternion partial = slerpQuaternion(Quaternion {}, tilt,
                        std::clamp(SwingFromBodyTilt * low / std::max(0.01f, SwingFromLowBody), 0.0f, 1.0f));
                    ikRootOrientation[side] = (partial * ikRootOrientation[side]).normalized();
                }
            }
        }
        // Quase-queda: a perna da passada no ar mira pela pelve de verdade
        // (so a orientacao). Com o tronco caindo 30-45 graus para a frente,
        // pela pelve de referencia (reta) o mesmo angulo de quadril deixava o
        // pe 0,7 m atras do corpo e ele nao vinha a frente.
        if (StumbleDetection && StumbleSwingMeasured > 0.0f && m_tripRecovery > 0.0f && input.forceDrivenRoot
            && gaitMoving && !m_footwork.owns(side) && m_feet[side].valid && !m_feet[side].planted) {
            const float w = std::clamp(StumbleSwingMeasured * m_tripRecovery, 0.0f, 1.0f);
            if (w > measured)
                ikRootOrientation[side] = slerpQuaternion(targetRootOrientation, measuredPelvis.orientation, w);
        }
        if (!goals[side].active) continue;
        // Balanco da passada em malha fechada no pe (TEMPTUNE): a perna no ar
        // e macia e, no fim do balanco, a canela seguia por inercia quando o
        // alvo freava e recuava - o joelho passava ~7 graus da extensao e o pe
        // tocava ate 6-15 cm a frente do alvo. O alvo do IK vai alem do alvo
        // de verdade o quanto o pe passou dele (o que um musculo faria
        // puxando a canela de volta), com limite.
        if (SwingTrackGain > 0.0f && input.forceDrivenRoot && gaitMoving && !m_feet[side].planted
            && !m_footwork.owns(side) && m_telemetry.footSwingProgress[side] >= 0.0f
            && desiredState != CharacterLocomotionState3D::Airborne) {
            const float late = smoothStep((m_telemetry.footSwingProgress[side] - 0.35f) / 0.30f);
            if (late > 0.0f) {
                m_trackedSwingGoal[side] = goals[side].position;
                m_trackedSwingGoalValid[side] = true;
                const Vec3 physicalFoot = state.links[m_feet[side].linkIndex].position;
                const Vec3 error = goals[side].position - physicalFoot;
                goals[side].position += clampMagnitude(error * (SwingTrackGain * late), SwingTrackMaximum);
            }
        }
        // Perna de apoio da passada complacente: ela nunca e mandada a
        // erguer a pelve mais que isto acima de onde a pelve esta. Pousando
        // com o corpo 6-7 cm abaixo da referencia (fim do voo da corrida, ou
        // o agachamento da freada), a perna resolvida pela pelve de
        // referencia pedia o pe 6 cm dentro do chao - os motores a esticavam
        // de uma vez e o corpo saltava (ate 11 cm acima da referencia,
        // invertendo a 3 m/s). Acima da referencia nada muda: a perna cede.
        if (StanceRiseLimit < 1.0f && input.forceDrivenRoot && gaitMoving && m_feet[side].planted
            && !m_footwork.owns(side) && measured <= 0.0f && physical.feet[side].supporting)
            ikRootPosition[side].z = std::min(ikRootPosition[side].z, measuredPelvis.position.z + StanceRiseLimit);
        // Manejo de bola: o pe no ar vai a bola (o alvo do toque por cima do
        // da passada, com o peso do toque).
        if (!m_feet[side].planted && input.ballAvoid[side].lengthSquared() > 1e-8f)
            goals[side].position += input.ballAvoid[side];
        if (input.ballFoot.active && input.ballFoot.side == static_cast<int>(side)
            && input.ballFoot.weight > 0.0f && !m_feet[side].planted
            && !(input.ballScript.active && input.ballScript.side == static_cast<int>(side))) {
            const float w = std::clamp(input.ballFoot.weight, 0.0f, 1.0f);
            goals[side].position += (input.ballFoot.position - goals[side].position) * w;
            goals[side].orientation = slerpQuaternion(goals[side].orientation, input.ballFoot.orientation, w);
        }
        // O pe no ar nunca entra na bola nem pousa nela (ela nao colide com o
        // pe): o alvo sai dela no plano. O pe que vai tocar fica de fora.
        if (input.ballRepel.active && !m_feet[side].planted && input.ballRepel.exemptSide != static_cast<int>(side))
            goals[side].position += footBallPushOut(profile.links[m_feet[side].linkIndex], goals[side].position,
                goals[side].orientation, input.ballRepel.position, input.ballRepel.radius, BallRepelMargin);
        // Continuidade do alvo: na troca de dono do pe no ar (o planejador
        // parado -> a passada, na arrancada) o alvo saltava ate 15 cm num
        // tick (18 m/s): o pe era jogado para o clipe e raspava. O alvo novo
        // entra misturado a partir do ultimo em GoalHandoffSeconds.
        {
            const bool plannerOwned = m_footwork.owns(side);
            if (GoalHandoffSeconds > 0.0f && m_goalOwnerValid[side] && m_goalPlannerOwned[side] != plannerOwned
                && !m_feet[side].planted) {
                m_goalHandoff[side] = 1.0f;
                m_goalHandoffPosition[side] = m_lastFootGoalPosition[side];
                m_goalHandoffOrientation[side] = m_lastFootGoalOrientation[side];
            }
            m_goalPlannerOwned[side] = plannerOwned;
            m_goalOwnerValid[side] = true;
            if (m_feet[side].planted) m_goalHandoff[side] = 0.0f;
            if (m_goalHandoff[side] > 0.0f) {
                const float hold = smoothStep(m_goalHandoff[side]);
                goals[side].position += (m_goalHandoffPosition[side] - goals[side].position) * hold;
                goals[side].orientation = slerpQuaternion(goals[side].orientation, m_goalHandoffOrientation[side], hold);
                m_goalHandoff[side] = std::max(0.0f, m_goalHandoff[side] - deltaTime / GoalHandoffSeconds);
            }
            m_lastFootGoalPosition[side] = goals[side].position;
            m_lastFootGoalOrientation[side] = goals[side].orientation;
        }
        const Quaternion rootInverse = ikRootOrientation[side].conjugate();
        // O pe da bola (toque e chute) sai longe da pose do clipe: o IK
        // limitado a ~52 graus por tick (7 iteracoes de 0,13 rad) deixava o
        // joelho do chute sem dobrar no armado e o pe 6-25 cm aquem do alvo.
        // So no chute: no drible as 7 de sempre (com mais, o pe chegava
        // diferente e 3 de 32 toques no trote se perdiam).
        const bool ballDriven = input.ballKick.active && input.ballKick.shot && input.ballKick.side == static_cast<int>(side)
            && ((input.ballScript.active && input.ballScript.side == static_cast<int>(side))
                || (input.ballFoot.active && input.ballFoot.side == static_cast<int>(side) && input.ballFoot.weight > 0.0f));
        solveFootPose(profile, m_coordinates, m_feet[side].linkIndex,
            rootInverse.rotate(goals[side].position - ikRootPosition[side]),
            (rootInverse * goals[side].orientation).normalized(), 1.0f,
            // Andando, o rumo do pe so vem da coxa quando ha giro de curva
            // nele (reto, o pe segue a pelve: ver solveFootPose).
            // Com pouca ajuda, o pe de apoio segura o rumo da trava (o da
            // referencia): seguindo a pelve, nada a trazia de volta ao rumo
            // pedido e ela ia girando (de lado a 5%: 30-50 graus, e a passada
            // de lado virava curva). Pela coxa, o quadril de apoio a gira de
            // volta.
            !gaitMoving || (CurveFootHoldYaw && std::abs(m_feet[side].curveYaw) > 0.03f)
                || (GaitStanceHoldYaw && m_feet[side].planted)
                || (GaitHoldYaw > 0.5f && lowAssist > GaitHoldYawFrom
                    && (m_feet[side].planted || GaitHoldYaw > 1.5f)),
            ballDriven ? BallFootIterations : 7);
    }
    // Alcance das pernas com o pe no chao (R3): quadril pela pelve (fisica
    // e do IK), tornozelo pelo pe (fisico e alvo).
    for (std::size_t side = 0; side < m_feet.size(); ++side) {
        LegReachTelemetry3D& reach = m_telemetry.legReach[side];
        const FootPlant& plant = m_feet[side];
        const bool planted = plant.valid && goals[side].active && plant.planted
            && plant.linkIndex < state.links.size();
        const int shin = planted ? profile.links[plant.linkIndex].parentIndex : -1;
        const int thigh = shin > 0 ? profile.links[static_cast<std::size_t>(shin)].parentIndex : -1;
        if (thigh <= 0) {
            reach = {};
            m_previousLegLength[side] = { -1.0f, -1.0f };
            continue;
        }
        const RagdollLinkDefinition3D& footLink = profile.links[plant.linkIndex];
        const Vec3 hipModel = profile.links[static_cast<std::size_t>(thigh)].inboundJoint.anchorModelPosition;
        const Vec3 kneeModel = profile.links[static_cast<std::size_t>(shin)].inboundJoint.anchorModelPosition;
        const Vec3 ankleModel = footLink.inboundJoint.anchorModelPosition;
        const Vec3 hipLocal = profile.links[0].modelOrientation.conjugate()
            .rotate(hipModel - profile.links[0].modelPosition);
        const Vec3 ankleLocal = footLink.modelOrientation.conjugate()
            .rotate(ankleModel - footLink.modelPosition);
        const float nominal = (kneeModel - hipModel).length() + (ankleModel - kneeModel).length();
        const PhysicsBodyState3D& footBody = state.links[plant.linkIndex];
        const Vec3 hipPhysical = measuredPelvis.position + measuredPelvis.orientation.rotate(hipLocal);
        const Vec3 anklePhysical = footBody.position + footBody.orientation.rotate(ankleLocal);
        const Vec3 hipReference = ikRootPosition[side] + ikRootOrientation[side].rotate(hipLocal);
        const Vec3 ankleReference = goals[side].position + goals[side].orientation.rotate(ankleLocal);
        const float physicalLength = (hipPhysical - anklePhysical).length();
        const float referenceLength = (hipReference - ankleReference).length();
        reach.valid = nominal > 0.1f;
        reach.nominalLengthMeters = nominal;
        reach.physicalRatio = reach.valid ? physicalLength / nominal : 0.0f;
        reach.referenceRatio = reach.valid ? referenceLength / nominal : 0.0f;
        reach.physicalAxialSpeed = m_previousLegLength[side][0] >= 0.0f
            ? (physicalLength - m_previousLegLength[side][0]) / deltaTime : 0.0f;
        reach.referenceAxialSpeed = m_previousLegLength[side][1] >= 0.0f
            ? (referenceLength - m_previousLegLength[side][1]) / deltaTime : 0.0f;
        reach.ikRootHeight = ikRootPosition[side].z;
        reach.physicalRootHeight = measuredPelvis.position.z;
        m_previousLegLength[side] = { physicalLength, referenceLength };
    }

    // Diagnostico E5-A: alvo e resultado do IK (antes do limitador).
    const auto footClearance = [&](std::size_t side, Vec3 position, Quaternion orientation, float ground) {
        return position.z - footLockHeight3D(profile.links[m_feet[side].linkIndex], ground, orientation);
    };
    for (std::size_t side = 0; side < m_feet.size(); ++side) {
        FootTrace3D& trace = m_telemetry.footTrace[side];
        trace = {};
        if (!m_footTraceEnabled || !goals[side].active || !m_feet[side].valid) continue;
        const std::size_t link = m_feet[side].linkIndex;
        const PhysicsBodyState3D& body = state.links[link];
        const GroundSample ground = groundUnder(side, body.position,
            forwardOf(m_feet[side], body.orientation));
        trace.groundHeight = ground.valid ? ground.height : 0.0f;
        trace.target = m_trackedSwingGoalValid[side] ? m_trackedSwingGoal[side] : goals[side].position;
        m_trackedSwingGoalValid[side] = false;
        trace.targetClearance = footClearance(side, goals[side].position, goals[side].orientation, trace.groundHeight);
        RagdollAnimationPose3D ikPose;
        buildLocalPose(profile, m_coordinates, ikPose);
        trace.ik = ikRootPosition[side] + ikRootOrientation[side].rotate(ikPose.linkPositions[link]);
        trace.ikClearance = footClearance(side, trace.ik,
            (ikRootOrientation[side] * ikPose.linkOrientations[link]).normalized(), trace.groundHeight);
        trace.physical = body.position;
        trace.ikRoot = ikRootPosition[side];
        trace.ikRootOrientation = ikRootOrientation[side];
        trace.physicalRoot = measuredPelvis.position;
        trace.physicalRootOrientation = measuredPelvis.orientation;
        trace.physicalClearance = footClearance(side, body.position, body.orientation, trace.groundHeight);
        trace.valid = true;
    }

    // Nothing this controller commands may ask a joint to move faster than
    // a body can. The motors are driven with a velocity target taken from
    // the per-tick change in these coordinates, so at 120 Hz any jump is
    // multiplied by 120: a pose-side discontinuity of a few degrees becomes
    // a limb travelling metres per second. Measured before this: feet
    // spiking past 4 m/s while flagged as planted, on a body turning at
    // 0.17 rad/s. Clamping the commanded change bounds that whole class of
    // glitch at the point where it enters the physics, whatever produced it
    // upstream - clip, procedural pose or the foot IK.
    //
    // O teto vale para o que NAO e o movimento do proprio clipe: IK de pe,
    // olhar e a mistura do crossfade. O que a amostra do clipe anda de um
    // tick para o outro passa por cima do teto - o joelho do sprint chega a
    // 20 rad/s por construcao, e cortar isso deformava o ciclo. A mistura
    // nao: medido, deixa-la passar levava o pe a 17 m/s na troca entre
    // recuar e andar para a frente, com a perna indo de uma pose a outra em
    // 0,14 s.
    constexpr float MaximumJointRateRadiansPerSecond = 12.0f;
    const float maximumJointStep =
        MaximumJointRateRadiansPerSecond * deltaTime;
    // A perna que o planejador tem no ar pode flexionar como a de uma
    // passada (o joelho do sprint chega a ~20 rad/s): a 12 rad/s o joelho que
    // saia esticado da descarga levava ~75 ms para dobrar e o pe subia tarde
    // e baixo (medido no passo lateral: 5 cm cortados pelo limitador). So o
    // joelho e o tornozelo: o quadril, com o pe a ~0,9 m, a 25 rad/s fazia a
    // perna do boneco empurrado chicotear (mediana 13,4 m/s no membro, contra
    // 9,5 com ele a 12).
    std::vector<bool> swingLeg(m_coordinates.size(), false);
    for (std::size_t side = 0; side < m_feet.size(); ++side) {
        const FootPlan3D& planFoot = m_footwork.plan().feet[side];
        if (!m_feet[side].valid || planFoot.followingGait || planFoot.phase != FootPhase3D::Swing) continue;
        int index = static_cast<int>(m_feet[side].linkIndex);
        for (int joint = 0; joint < 2 && index > 0; ++joint) {
            swingLeg[static_cast<std::size_t>(index)] = true;
            index = profile.links[static_cast<std::size_t>(index)].parentIndex;
        }
    }
    constexpr float SwingLegJointRateRadiansPerSecond = 25.0f;
    for (std::size_t index = 1; index < m_coordinates.size(); ++index) {
        const Vec3 previous = previousCoordinates[index];
        Vec3& current = m_coordinates[index];
        const float maximumJointStep = (swingLeg[index] ? SwingLegJointRateRadiansPerSecond
            : MaximumJointRateRadiansPerSecond) * deltaTime;
        for (std::size_t axis = 0; axis < 3; ++axis) {
            const float delta = component(current, axis)
                - component(previous, axis);
            const float clipStep = selectedClip == m_previousSampledClip
                ? std::abs(component(sampled[index], axis)
                    - component(m_previousClipCoordinates[index], axis))
                : 0.0f;
            const float allowed = maximumJointStep + clipStep;
            if (std::abs(delta) > allowed) {
                setComponent(current, axis, component(previous, axis)
                    + std::copysign(allowed, delta));
            }
        }
    }
    m_previousClipCoordinates = sampled;

    // Levantar de bruços: a partir dos quatro apoios os pes sao do
    // equilibrio, nao do clipe (TEMPTUNE GetUpFootwork). Ficam presos no chao
    // onde estao - IK a partir da pelve medida, adiantada um pouco para a do
    // clipe (o descompasso e o empurrao das pernas) - e, quando o centro de
    // massa passa da ponta do pe mais a frente, o pe de tras da um passo para
    // baixo dele. As maos so soltam (o clipe espera) com o corpo sobre os
    // pes. Sem isso, sem ajuda, ele soltava as maos com o centro de massa
    // 0,4-0,6 m a frente dos pes e caia de cabeca.
    m_telemetry.getUpBalanceExcess = 0.0f;
    if (GetUpFootworkEnabled && getUpActive && m_getUpPhase == CharacterGetUpPhase3D::Rising
        && !m_bodilyRising && m_fallOrientation == CharacterFallOrientation3D::FaceDown
        && input.forceDrivenRoot && animations.standUpFront != nullptr && !state.links.empty()) {
        const float duration = std::max(0.05f, animations.standUpFront->durationSeconds);
        const float progress = m_getUpClipSeconds / duration;
        const char* footIds[2] = { "LeftFoot", "RightFoot" };
        const char* shinIds[2] = { "LeftShin", "RightShin" };
        std::size_t foot[2] {}, shin[2] {};
        bool linksOk = true;
        for (std::size_t side = 0; side < 2; ++side) {
            foot[side] = findLink(profile, footIds[side]);
            shin[side] = findLink(profile, shinIds[side]);
            linksOk = linksOk && foot[side] > 0 && shin[side] > 0
                && foot[side] < state.links.size() && shin[side] < state.links.size();
        }
        if (linksOk && progress >= GetUpFootworkFrom) {
            const PhysicsBodyState3D& pelvis = state.links.front();
            const auto ankleInShin = [&](std::size_t side) {
                const RagdollLinkDefinition3D& link = profile.links[foot[side]];
                const RagdollLinkDefinition3D& parent = profile.links[shin[side]];
                return parent.modelOrientation.conjugate().rotate(
                    link.inboundJoint.anchorModelPosition - parent.modelPosition);
            };
            const auto ankleWorld = [&](std::size_t side) {
                return state.links[shin[side]].position
                    + state.links[shin[side]].orientation.rotate(ankleInShin(side));
            };
            const float ground = input.rootPositionWorld.z - profile.standingRootHeightMeters;
            // O tornozelo acima da sola com o pe no chao: a ancora do pe no
            // repouso do perfil (13,5 cm no corpo antigo, 10,7 no montado).
            const float ankleHeight = profile.links[foot[0]].inboundJoint.anchorModelPosition.z + profile.standingRootHeightMeters;
            if (!m_getUpFootwork) {
                m_getUpFootwork = true;
                for (std::size_t side = 0; side < 2; ++side) m_getUpFootWorld[side] = ankleWorld(side);
                m_getUpStepSide = -1;
                // A frente do levantar: dos pes para as maos (de bruços a
                // frente da pelve aponta para o chao).
                const std::size_t hands[2] = { findLink(profile, "LeftHand"), findLink(profile, "RightHand") };
                Vec3 handMid {}, footMid = (m_getUpFootWorld[0] + m_getUpFootWorld[1]) * 0.5f;
                int handCount = 0;
                for (const std::size_t hand : hands)
                    if (hand > 0 && hand < state.links.size()) { handMid += state.links[hand].position; ++handCount; }
                Vec3 toHands = handCount > 0 ? handMid * (1.0f / static_cast<float>(handCount)) - footMid : Vec3 {};
                toHands.z = 0.0f;
                m_getUpFootworkForward = toHands.length() > 0.1f ? toHands.normalized()
                    : Vec3 { std::cos(m_getUpAnchorYaw), std::sin(m_getUpAnchorYaw), 0.0f };
                // Um pe ja no ar (o passo do clipe) continua como passo nosso.
                for (std::size_t side = 0; side < 2; ++side) {
                    if (m_getUpFootWorld[side].z > ground + ankleHeight + 0.12f && m_getUpStepSide < 0) {
                        m_getUpStepSide = static_cast<int>(side);
                        m_getUpStepSeconds = GetUpStepSeconds * 0.5f;
                        m_getUpStepFrom = m_getUpFootWorld[side];
                        m_getUpStepFrom.z = ground + ankleHeight;
                        const Vec3 comNow = m_estimator.state().centerOfMassWorld;
                        const float sideSign = side == 0 ? 1.0f : -1.0f;
                        Vec3 to = comNow + m_getUpFootworkForward * GetUpStepAhead
                            + Vec3 { -m_getUpFootworkForward.y, m_getUpFootworkForward.x, 0.0f } * (sideSign * GetUpStepWidth);
                        to.z = ground + ankleHeight;
                        m_getUpStepTo = to;
                        ++m_getUpBalanceSteps;
                    }
                }
            }
            const Vec3 forward = m_getUpFootworkForward;
            const Vec3 leftDirection { -forward.y, forward.x, 0.0f };
            const Vec3 com = m_estimator.state().centerOfMassWorld;
            // Quanto o centro de massa passa da ponta do pe mais a frente (so
            // os pes no chao).
            float front = -1e9f;
            for (std::size_t side = 0; side < 2; ++side) {
                if (static_cast<int>(side) == m_getUpStepSide) continue;
                front = std::max(front, dot(m_getUpFootWorld[side] - com, forward));
            }
            const float excess = -(front + GetUpToeAhead);
            m_telemetry.getUpBalanceExcess = excess;
            if (m_getUpStepSide < 0 && excess > -GetUpStepMargin && m_getUpBalanceSteps < 6) {
                // O pe de tras vai para baixo do centro de massa, do seu lado.
                const int rear = dot(m_getUpFootWorld[0] - com, forward)
                    < dot(m_getUpFootWorld[1] - com, forward) ? 0 : 1;
                m_getUpStepSide = rear;
                m_getUpStepSeconds = 0.0f;
                m_getUpStepFrom = m_getUpFootWorld[rear];
                const float sideSign = rear == 0 ? 1.0f : -1.0f;
                Vec3 to = com + forward * GetUpStepAhead + leftDirection * (sideSign * GetUpStepWidth);
                to.z = ground + ankleHeight;
                Vec3 move = to - m_getUpStepFrom;
                const float planar = std::hypot(move.x, move.y);
                if (planar > GetUpStepMax) {
                    move.x *= GetUpStepMax / planar;
                    move.y *= GetUpStepMax / planar;
                }
                m_getUpStepTo = m_getUpStepFrom + move;
                ++m_getUpBalanceSteps;
            }
            if (m_getUpStepSide >= 0) {
                m_getUpStepSeconds += deltaTime;
                const float s = std::clamp(m_getUpStepSeconds / std::max(0.05f, GetUpStepSeconds), 0.0f, 1.0f);
                Vec3 position = m_getUpStepFrom + (m_getUpStepTo - m_getUpStepFrom) * smoothStep(s);
                position.z += GetUpStepLift * std::sin(Pi * s);
                m_getUpFootWorld[static_cast<std::size_t>(m_getUpStepSide)] = position;
                if (s >= 1.0f) {
                    m_getUpFootWorld[static_cast<std::size_t>(m_getUpStepSide)] = m_getUpStepTo;
                    m_getUpStepSide = -1;
                }
            }
            const bool needHold = m_getUpStepSide >= 0 || excess > -GetUpStepMargin;
            m_getUpBalanceHoldSeconds = needHold ? m_getUpBalanceHoldSeconds + deltaTime : 0.0f;
            m_getUpBalanceHold = needHold && m_getUpBalanceHoldSeconds < GetUpHoldMax;
            // Pelve do IK: a medida, adiantada para a do clipe (limitado).
            Vec3 rootPosition = pelvis.position;
            rootPosition.z += std::clamp(targetRootPosition.z - pelvis.position.z, -GetUpRootLead, GetUpRootLead);
            Quaternion rootOrientation = pelvis.orientation;
            {
                const Vec3 turn = rotationVector((targetRootOrientation * pelvis.orientation.conjugate()).normalized());
                const float angle = turn.length();
                if (angle > 1e-4f)
                    rootOrientation = (exponentialRotation(turn * (std::min(angle, GetUpTurnLead) / angle))
                        * pelvis.orientation).normalized();
            }
            const Quaternion rootInverse = rootOrientation.conjugate();
            const Vec3 upInRoot = rootInverse.rotate({ 0.0f, 0.0f, 1.0f });
            for (std::size_t side = 0; side < 2; ++side) {
                const Vec3 target = rootInverse.rotate(m_getUpFootWorld[side] - rootPosition);
                solveRagdollChainPoint3D(profile, m_coordinates, shin[side], ankleInShin(side), 2, target, 1.0f, 12);
                // O pe que deu passo pousa com a sola no chao; o que ainda nao
                // saiu do lugar fica como o clipe (na ponta dos pes).
                const bool stepped = m_getUpBalanceSteps > 0
                    && (static_cast<int>(side) == m_getUpStepSide
                        || std::abs(m_getUpFootWorld[side].z - (ground + ankleHeight)) < 0.02f);
                if (!stepped) continue;
                const RagdollLinkDefinition3D& link = profile.links[foot[side]];
                float best = -2.0f;
                Vec3 bestAngles = m_coordinates[foot[side]];
                RagdollAnimationPose3D frame;
                for (int i = 0; i <= 12; ++i) {
                    Vec3 angles = m_coordinates[foot[side]];
                    setComponent(angles, 0, std::clamp(-0.85f + 1.85f * static_cast<float>(i) / 12.0f,
                        link.inboundJoint.axes[0].minimumRadians, link.inboundJoint.axes[0].maximumRadians));
                    m_coordinates[foot[side]] = angles;
                    buildRagdollLocalPose3D(profile, m_coordinates, frame);
                    const float alignment = dot(frame.linkOrientations[foot[side]].rotate({ 0.0f, 0.0f, 1.0f }), upInRoot);
                    if (alignment > best) {
                        best = alignment;
                        bestAngles = angles;
                    }
                }
                m_coordinates[foot[side]] = bestAngles;
            }
        }
    }
    // Mao de apoio no levantar de bruços (TEMPTUNE GetUpHands, ideia do
    // usuario 04/10). So pelas juntas, o clipe dizia onde a mao fica em
    // relacao ao corpo do clipe; com o corpo fisico um pouco diferente a mao
    // de apoio ia parar para dentro, embaixo do peito, e nao sustentava -
    // ele caia de lado. Aqui, quando o clipe pede a mao no chao, ela vai a um
    // ponto do chao ao lado e a frente do ombro (no mundo), com a palma para
    // baixo, e fica presa ali (IK a partir da pelve e da coluna medidas)
    // enquanto pes e joelhos vem para a frente. Solta quando o clipe levanta
    // a mao ou o braco nao alcanca mais, voltando ao clipe aos poucos.
    if (GetUpHandsEnabled && getUpActive && m_getUpPhase == CharacterGetUpPhase3D::Rising
        && !m_bodilyRising && m_fallOrientation == CharacterFallOrientation3D::FaceDown
        && input.forceDrivenRoot && !state.links.empty()
        && state.joints.size() == m_coordinates.size()) {
        const char* upperIds[2] = { "LeftUpperArm", "RightUpperArm" };
        const char* foreIds[2] = { "LeftForearm", "RightForearm" };
        const char* handIds[2] = { "LeftHand", "RightHand" };
        const auto anchorIn = [&](std::size_t child) {
            const RagdollLinkDefinition3D& link = profile.links[child];
            const RagdollLinkDefinition3D& parent = profile.links[static_cast<std::size_t>(link.parentIndex)];
            return parent.modelOrientation.conjugate().rotate(link.inboundJoint.anchorModelPosition - parent.modelPosition);
        };
        const PhysicsBodyState3D& pelvis = state.links.front();
        const float ground = input.rootPositionWorld.z - profile.standingRootHeightMeters;
        // Rumo do corpo: frente + cima da pelve no plano (estavel de bruços,
        // quando a frente aponta para o chao, e de pe).
        Vec3 heading = pelvis.orientation.rotate({ 1.0f, 0.0f, 0.0f })
            + pelvis.orientation.rotate({ 0.0f, 0.0f, 1.0f });
        heading.z = 0.0f;
        heading = heading.length() > 0.1f ? heading.normalized() : Vec3 { 1.0f, 0.0f, 0.0f };
        const Vec3 leftward { -heading.y, heading.x, 0.0f };
        // A pose do clipe (alvos das juntas) no referencial da pelve: onde o
        // clipe quer a mao (altura do chao do clipe).
        RagdollAnimationPose3D clipFrame;
        buildRagdollLocalPose3D(profile, m_coordinates, clipFrame);
        // Para o IK, a coluna medida (o ombro de verdade).
        std::vector<Vec3> measuredSpine = m_coordinates;
        for (const char* id : { "Abdomen", "Chest", "UpperChest" }) {
            const std::size_t i = findLink(profile, id);
            if (i > 0 && i < state.joints.size())
                measuredSpine[i] = { state.joints[i].positionRadians[0], state.joints[i].positionRadians[1],
                    state.joints[i].positionRadians[2] };
        }
        for (std::size_t side = 0; side < 2; ++side) {
            GetUpHand& hand = m_getUpHands[side];
            const std::size_t upper = findLink(profile, upperIds[side]);
            const std::size_t fore = findLink(profile, foreIds[side]);
            const std::size_t palm = findLink(profile, handIds[side]);
            if (upper == 0 || fore == 0 || palm == 0 || palm >= state.links.size()) continue;
            const Vec3 wristInFore = anchorIn(palm);
            const float sideSign = side == 0 ? 1.0f : -1.0f;
            // Mao do clipe: altura do punho acima do chao do clipe.
            const Vec3 clipWristLocal = clipFrame.linkPositions[fore] + clipFrame.linkOrientations[fore].rotate(wristInFore);
            const float clipWrist = m_getUpExpectedHeight + targetRootOrientation.rotate(clipWristLocal).z;
            const bool clipDown = m_getUpExpectedValid && clipWrist < GetUpHandClipDown;
            const Vec3 wristNow = state.links[fore].position + state.links[fore].orientation.rotate(wristInFore);
            const std::size_t chest = static_cast<std::size_t>(profile.links[upper].parentIndex);
            const Vec3 shoulder = state.links[chest].position + state.links[chest].orientation.rotate(anchorIn(upper));
            const float armLength = (profile.links[fore].inboundJoint.anchorModelPosition
                    - profile.links[upper].inboundJoint.anchorModelPosition).length()
                + (profile.links[palm].inboundJoint.anchorModelPosition
                    - profile.links[fore].inboundJoint.anchorModelPosition).length();
            hand.clipUpSeconds = clipDown ? 0.0f : hand.clipUpSeconds + deltaTime;
            if (hand.stage == 0 && clipDown) {
                hand.stage = 1;
                hand.seconds = 0.0f;
                hand.from = wristNow;
                Vec3 spot = shoulder + heading * GetUpHandForward + leftward * (sideSign * GetUpHandOutward);
                spot.z = ground + GetUpHandWristHeight;
                hand.spot = spot;
            }
            if (hand.stage == 1 || hand.stage == 2) {
                hand.seconds += deltaTime;
                const float out = (hand.spot - shoulder).length();
                if (hand.clipUpSeconds > 0.1f || out > 0.98f * armLength) {
                    hand.stage = 3;
                    hand.seconds = 0.0f;
                }
            }
            if (hand.stage == 1 && hand.seconds >= GetUpHandReachSeconds) hand.stage = 2;
            // Encostou antes no chao, num lugar bom (para fora do ombro): fica ali.
            if (hand.stage == 1 && wristNow.z < ground + GetUpHandWristHeight + 0.03f
                && dot(wristNow - shoulder, leftward) * sideSign > 0.02f) {
                hand.stage = 2;
                hand.spot = { wristNow.x, wristNow.y, ground + GetUpHandWristHeight };
            }
            const float wanted = hand.stage == 1 || hand.stage == 2 ? 1.0f : 0.0f;
            hand.weight += std::clamp(wanted - hand.weight, -deltaTime / GetUpHandReleaseSeconds,
                deltaTime / 0.12f);
            if (hand.stage == 3 && hand.weight <= 0.0f) hand.stage = 0;
            if (hand.weight <= 0.0f) continue;
            Vec3 target = hand.spot;
            if (hand.stage == 1) {
                const float s = std::clamp(hand.seconds / GetUpHandReachSeconds, 0.0f, 1.0f);
                target = hand.from + (hand.spot - hand.from) * smoothStep(s);
                target.z += 0.08f * std::sin(Pi * s);
            }
            std::vector<Vec3> ik = measuredSpine;
            const Vec3 local = pelvis.orientation.conjugate().rotate(target - pelvis.position);
            solveRagdollChainPoint3D(profile, ik, fore, wristInFore, 2, local, 1.0f, 12);
            // Palma para baixo: procura nos dois eixos do punho.
            const Vec3 upInRoot = pelvis.orientation.conjugate().rotate({ 0.0f, 0.0f, 1.0f });
            const RagdollLinkDefinition3D& link = profile.links[palm];
            float best = -2.0f;
            Vec3 bestAngles = ik[palm];
            RagdollAnimationPose3D frame;
            for (int i = -3; i <= 3; ++i) {
                for (int j = -2; j <= 2; ++j) {
                    Vec3 angles = ik[palm];
                    if (link.inboundJoint.axes[0].enabled)
                        setComponent(angles, 0, std::clamp(0.35f * static_cast<float>(i),
                            link.inboundJoint.axes[0].minimumRadians, link.inboundJoint.axes[0].maximumRadians));
                    if (link.inboundJoint.axes[1].enabled)
                        setComponent(angles, 1, std::clamp(0.25f * static_cast<float>(j),
                            link.inboundJoint.axes[1].minimumRadians, link.inboundJoint.axes[1].maximumRadians));
                    ik[palm] = angles;
                    buildRagdollLocalPose3D(profile, ik, frame);
                    const float alignment = dot(frame.linkOrientations[palm].rotate({ 0.0f, 0.0f, 1.0f }), upInRoot);
                    if (alignment > best) {
                        best = alignment;
                        bestAngles = angles;
                    }
                }
            }
            ik[palm] = bestAngles;
            const float w = smoothStep(hand.weight);
            for (const std::size_t i : { upper, fore, palm })
                m_coordinates[i] += (ik[i] - m_coordinates[i]) * w;
        }
    } else {
        for (GetUpHand& hand : m_getUpHands) hand = {};
    }
    for (std::size_t side = 0; side < 2; ++side) m_telemetry.getUpHandStage[side] = m_getUpHands[side].stage;
    m_telemetry.getUpFootwork = m_getUpFootwork;
    m_telemetry.getUpBalanceSteps = m_getUpBalanceSteps;
    m_telemetry.getUpBalanceHold = m_getUpBalanceHold;
    m_telemetry.getUpRolling = m_getUpRolling;
    m_telemetry.tackling = m_tackling;
    m_telemetry.tacklePending = m_tacklePending;
    m_telemetry.tackleVariant = m_tackling ? m_tackleVariant : -1;
    m_telemetry.tackleEntrySpeed = m_tackleEntrySpeed;
    m_telemetry.tackleRate = m_tackleRate;
    m_telemetry.tackleTravel = m_tackling && !state.links.empty()
        ? std::hypot(state.links.front().position.x - m_tackleStart.x,
            state.links.front().position.y - m_tackleStart.y) : 0.0f;
    m_previousSampledClip = selectedClip;

    buildLocalPose(profile, m_coordinates, localPose);
    // Diagnostico E5-A: o comando final (depois do limitador), no referencial
    // do IK e a partir da pelve fisica.
    for (std::size_t side = 0; side < m_feet.size(); ++side) {
        FootTrace3D& trace = m_telemetry.footTrace[side];
        if (!trace.valid) continue;
        const std::size_t link = m_feet[side].linkIndex;
        trace.command = ikRootPosition[side] + ikRootOrientation[side].rotate(localPose.linkPositions[link]);
        trace.commandClearance = footClearance(side, trace.command,
            (ikRootOrientation[side] * localPose.linkOrientations[link]).normalized(), trace.groundHeight);
        trace.commandFromPhysicalPelvis = measuredPelvis.position
            + measuredPelvis.orientation.rotate(localPose.linkPositions[link]);
        trace.commandFromPhysicalPelvisClearance = footClearance(side, trace.commandFromPhysicalPelvis,
            (measuredPelvis.orientation * localPose.linkOrientations[link]).normalized(), trace.groundHeight);
    }

    // Quanto cada pe ficou longe do alvo depois do IK e do limitador: pe
    // travado que a perna nao alcanca (limite do quadril descendo escada)
    // solta no proximo tick.
    for (std::size_t side = 0; side < m_feet.size(); ++side) {
        FootPlant& foot = m_feet[side];
        if (!goals[side].active || !foot.valid) {
            foot.reachError = 0.0f;
            continue;
        }
        const Vec3 solved = ikRootPosition[side] + ikRootOrientation[side].rotate(
            localPose.linkPositions[foot.linkIndex]);
        foot.reachError = (solved - goals[side].position).length();
    }

    // Ground the authored pose before anything is driven from it. The root
    // height comes from the capsule plus the clip's own root track, and the
    // two do not have to agree: measured on the idle clip, the right sole
    // was being commanded 3.4 cm BELOW the floor. The floor answers that by
    // shoving the foot back up, and the leg above buckles to absorb it -
    // which is the crooked-legged stance, not weak motors. Raising the root
    // so the lowest authored sole rests exactly on the ground removes the
    // fight at its source; it never lowers the pose, so a genuine airborne
    // frame is untouched.
    if (!getUpActive && !input.forceDrivenRoot) {
        float deepest = 0.0f;
        for (std::size_t side = 0; side < m_feet.size(); ++side) {
            const FootPlant& foot = m_feet[side];
            if (!foot.valid || foot.linkIndex >= localPose.linkPositions.size()) {
                continue;
            }
            // Com os pes no terreno, so o pe de apoio conta: o pe em balanco
            // tem a propria folga sobre o chao (ver "Pes no terreno"). Contar
            // ele erguia o corpo inteiro quando passava rente a quina do
            // degrau seguinte, e o pe de apoio deixava de alcancar o dele
            // (medido: pelve 16 cm acima, pe de apoio pendurado 18 cm).
            if (footworkEnabled && onGround && !foot.planted) continue;
            // O chao alvo deste pe (terreno, ver "Pes no terreno"); fora do
            // chao, a base da capsula.
            const float floorZ = footworkEnabled && onGround
                ? foot.groundHeight : baseGround;
            const RagdollLinkDefinition3D& link = profile.links[foot.linkIndex];
            const Quaternion colliderWorld = (targetRootOrientation
                * localPose.linkOrientations[foot.linkIndex]
                * link.collider.localOrientation).normalized();
            const Vec3 colliderCentre = targetRootPosition
                + targetRootOrientation.rotate(
                    localPose.linkPositions[foot.linkIndex]
                    + localPose.linkOrientations[foot.linkIndex].rotate(
                        link.collider.localPosition));
            const float soleWorld = colliderCentre.z
                - lowestColliderOffset(link, colliderWorld);
            deepest = std::min(deepest, soleWorld - floorZ);
        }
        // Rede de seguranca, suavizada: sobe rapido e desce devagar. Aplicada
        // num tick so, qualquer diferenca entre a pose e o chao virava um
        // degrau na pelve (e o corpo inteiro tremia com ele).
        const float lift = std::max(0.0f, -deepest);
        const float response = lift > m_groundLift ? 40.0f : 10.0f;
        m_groundLift += (lift - m_groundLift)
            * (1.0f - std::exp(-response * deltaTime));
        targetRootPosition.z += m_groundLift;
    }

    m_output.targetPose = localPose;
    for (std::size_t index = 0; index < profile.links.size(); ++index) {
        m_output.targetPose.linkPositions[index] = targetRootPosition
            + targetRootOrientation.rotate(localPose.linkPositions[index]);
        m_output.targetPose.linkOrientations[index] =
            (targetRootOrientation * localPose.linkOrientations[index])
                .normalized();
    }
    m_referencePendulumValid = m_feet.size() >= 2 && m_feet[0].valid && m_feet[1].valid
        && m_feet[0].linkIndex < localPose.linkPositions.size() && m_feet[1].linkIndex < localPose.linkPositions.size();
    if (m_referencePendulumValid) {
        m_referencePendulum = targetRootOrientation.rotate(
            pendulumArm(profile, m_feet[0].linkIndex, m_feet[1].linkIndex,
                [&](std::size_t i) { return localPose.linkPositions[i]; }));
        float total = 0.0f;
        Vec3 center {};
        for (std::size_t i = 0; i < profile.links.size(); ++i) {
            center = center + m_output.targetPose.linkPositions[i] * profile.links[i].massFraction;
            total += profile.links[i].massFraction;
        }
        m_referenceCenterOfMassWorld = center * (1.0f / std::max(1e-4f, total));
        m_referenceFeetWorld = { m_output.targetPose.linkPositions[m_feet[0].linkIndex],
            m_output.targetPose.linkPositions[m_feet[1].linkIndex] };
    }

    const float muscle = std::clamp(input.muscleAuthority, 0.0f, 1.0f);
    // A real hit lets the joints actually give, not just the root guide
    // above: this is what makes falling possible at all instead of the
    // legs snapping back to the walking pose no matter how hard something
    // hits the character. Get-up phases carry their own tuned profile
    // below instead (including a near-limp Resting), so this only softens
    // normal locomotion.
    // Cede na pancada, mas nunca vira corpo morto: minimo de 35% (era 3%).
    // Com a raiz livre (modo fisico) as pancadas nao passam por aqui (a
    // cedencia e local, por junta, do controlador fisico) - so a PhysGun
    // ainda subia esta mistura, e agarrado o corpo inteiro ia a 35% da
    // forca (e assim ficava ~1,5 s depois de soltar): arrastado, a perna
    // nao erguia o pe e ele caia (varredura de arrasto: 20 de 32).
    const float impactSoftening = getUpActive || input.forceDrivenRoot ? 1.0f
        : std::clamp(1.0f - m_reactionStrength * 0.97f, 0.35f, 1.0f);
    float squaredError = 0.0f;
    std::uint32_t errorCount = 0;
    std::array<bool, 2> physicalSupport {};
    for (const auto& contact : state.interactions) {
        if (contact.normalWorld.z < 0.5f) continue;
        for (std::size_t side = 0; side < m_feet.size(); ++side)
            if (contact.linkIndex == m_feet[side].linkIndex) physicalSupport[side] = true;
    }
    std::array<float, 2> touchdownShare {};
    if (input.forceDrivenRoot && !getUpActive) {
        for (std::size_t side = 0; side < m_feet.size(); ++side) {
            const auto& foot = m_feet[side];
            if (!foot.valid || foot.linkIndex >= state.links.size()) continue;
            const auto& body = state.links[foot.linkIndex];
            const auto ground = groundUnder(side, body.position,
                forwardOf(foot, body.orientation));
            if (!ground.valid || body.linearVelocity.z > 0.15f) continue;
            // Um passo do planejador ainda no comeco do balanco nao esta
            // pousando: num passo de marcha baixo (8 cm) o corte valia o
            // balanco inteiro, a perna seguia so por posicao, atrasada, e o
            // pe subia metade do pedido e pousava curto.
            const FootPlan3D& planFoot = m_footwork.plan().feet[side];
            if (!planFoot.followingGait && planFoot.phase == FootPhase3D::Swing
                && planFoot.swingSeconds < 0.8f * planFoot.swingDurationSeconds) continue;
            // Passo de marcha (E5): a perna desce no tempo do plano ate o
            // contato - o amortecimento e da fase de carga, depois do toque.
            // Com o corte (velocidade-alvo zerada e mola 30% mais mole nos
            // ultimos 20%), no passo lateral o joelho estendia ~80 ms atrasado,
            // o pe ficava 3 cm acima com o alvo ja no chao e o pouso atrasava
            // 0,15 s procurando.
            if (!planFoot.followingGait && planFoot.reason == StepReason3D::Locomotion
                && (planFoot.phase == FootPhase3D::Swing || planFoot.phase == FootPhase3D::TouchdownSearch))
                continue;
            const float clearance = body.position.z
                - lockHeight(foot, ground.height, body.orientation);
            touchdownShare[side] = smoothStep((0.12f - clearance) / 0.12f);
        }
    }
    // Caindo e no chao, os alvos das juntas sao suavizados (~0,12 s): as
    // poses de queda trocam de alvo de repente (rumo da queda, mao tocando,
    // fetal, arrumar) e, sem isso, os membros davam trancos - parecia
    // convulsao. Sai aos poucos no comeco do levantar.
    {
        const float rising = m_getUpPhase == CharacterGetUpPhase3D::Rising
            ? 1.0f - smoothStep(m_getUpPhaseSeconds / 0.4f) : 0.0f;
        // (Pernas e tronco so com a queda sem volta: no desequilibrio os
        // passos que ainda salvam nao podem atrasar. Bracos e pescoco ja com
        // o reflexo.)
        // (Deitado o filtro sai conforme ele se arruma: o levantar parte dos
        // alvos exatos da pose de partida.)
        const float lying = std::max(m_getUpPhase == CharacterGetUpPhase3D::Settling
            ? 1.0f - m_settleGather : 0.0f, rising);
        if (m_fallSmoothCoordinates.size() != m_coordinates.size())
            m_fallSmoothCoordinates = m_coordinates;
        const float blend = 1.0f - std::exp(-deltaTime / std::max(FallSmoothSeconds, 1e-3f));
        for (std::size_t index = 1; index < m_coordinates.size(); ++index) {
            const std::string& id = profile.links[index].id;
            const bool reflexLimb = id.find("Arm") != std::string::npos
                || id.find("Forearm") != std::string::npos || id.find("Hand") != std::string::npos
                || id == "Neck" || id == "Head";
            // (Levantando, so a transicao do comeco: o resto do reflexo, que
            // leva ~1 s para sair, atrasava o clipe de levantar e o corpo
            // brigava com ele - de bruços ficava ~2 s sem sair do chao.)
            const float smoothing = FallSmoothSeconds <= 0.0f ? 0.0f
                : m_getUpPhase == CharacterGetUpPhase3D::Rising ? rising
                : std::max(lying, reflexLimb ? m_fallReflex : m_fallCommit);
            m_fallSmoothCoordinates[index] += (m_coordinates[index] - m_fallSmoothCoordinates[index]) * blend;
            if (smoothing > 0.0f)
                m_coordinates[index] += (m_fallSmoothCoordinates[index] - m_coordinates[index]) * smoothing;
            else
                m_fallSmoothCoordinates[index] = m_coordinates[index];
        }
    }
    // Levantar pelo corpo: as juntas sao as dele (BodilyGetUp3D).
    const bool bodilyRising = m_bodilyRising
        && m_getUpPhase == CharacterGetUpPhase3D::Rising
        && m_bodilyPose.coordinates.size() == m_coordinates.size();
    if (bodilyRising) {
        for (std::size_t index = 1; index < m_coordinates.size(); ++index)
            m_coordinates[index] = m_bodilyPose.coordinates[index];
    }
    for (std::size_t index = 1; index < profile.links.size(); ++index) {
        m_coordinateVelocities[index] =
            (m_coordinates[index] - previousCoordinates[index]) / deltaTime;
        const Vec3 measured {
            state.joints[index].positionRadians[0],
            state.joints[index].positionRadians[1],
            state.joints[index].positionRadians[2]
        };
        const Vec3 velocity = jointAngularVelocity(m_coordinates[index],
            m_coordinateVelocities[index], measured);
        const auto& linkId = profile.links[index].id;
        const bool leg = linkId.find("Thigh") != std::string::npos
            || linkId.find("Shin") != std::string::npos
            || linkId.find("Foot") != std::string::npos;
        const std::size_t side = linkId.starts_with("Left") ? 0 : 1;
        // On a loaded foot, discontinuities in IK velocity become a kick
        // against the floor. Position drives retain support; damping absorbs
        // landing energy instead of feeding it back through the velocity goal.
        // (A perna em balanco do planejador que ainda raspa o chao no comeco
        // do passo, solta SEM carga, nao esta apoiada: sem a velocidade-alvo
        // ela seguia so por posicao, atrasada. Solta com carga - a soltura de
        // emergencia da recuperacao -, a velocidade seria o coice contra o
        // chao: no modo so pelas pernas, 26 -> 50 quedas.)
        const FootPlan3D& legPlan = m_footwork.plan().feet[side];
        const bool locomotionStep = legPlan.reason == StepReason3D::Locomotion
            && (legPlan.phase == FootPhase3D::Swing || legPlan.phase == FootPhase3D::TouchdownSearch);
        // (R2 do pacote v2: na marcha pelo planejador, quem deveria liberar a
        // velocidade do balanco e o pe no ar de fato - physicalRelease -, nao
        // a carga estimada na soltura. Medido antes de corrigir o voo curto:
        // misto, inversao 1 -> 3 quedas. Volta depois da R4.)
        const bool releasedForSwing = legPlan.releasedUnloaded;
        const bool plannedSwing = leg && !legPlan.followingGait && releasedForSwing
            && ((legPlan.phase == FootPhase3D::Swing
                    && legPlan.swingSeconds < 0.8f * legPlan.swingDurationSeconds)
                || locomotionStep);
        // Empurrao antes do toque (E5, fase 7): com o pe que vem no ultimo
        // terco do balanco (ou procurando o chao), a perna de apoio segue a
        // velocidade-alvo da referencia em vez de zero - com zero, o motor
        // amortecia o proprio pendulo e freava o corpo justo antes do pouso.
        // Fora dessa janela o apoio segue com zero (o H17 tirava o freio no
        // apoio inteiro e o corpo passava da velocidade pedida).
        // (So empurra com o corpo abaixo da velocidade da referencia no rumo
        // dela: ja na velocidade, empurrar sempre o levava alem do que a
        // marcha segura - 1 m/s: 2 -> 4 quedas; 1,5 m/s: 6 -> 10.)
        const FootPlan3D& otherPlan = m_footwork.plan().feet[1 - side];
        bool belowReference = false;
        if (m_bodyRefValid) {
            const float referenceSpeed = std::hypot(m_bodyRefVelocity.x, m_bodyRefVelocity.y);
            const Vec3 com = m_estimator.state().centerOfMassVelocityWorld;
            belowReference = referenceSpeed > 0.1f
                && (com.x * m_bodyRefVelocity.x + com.y * m_bodyRefVelocity.y) / referenceSpeed < referenceSpeed;
        }
        const bool propulsive = leg && belowReference && m_footwork.settings().locomotionStepping && !legPlan.followingGait
            && legPlan.phase == FootPhase3D::Stance && otherPlan.reason == StepReason3D::Locomotion
            && (otherPlan.phase == FootPhase3D::TouchdownSearch
                || (otherPlan.phase == FootPhase3D::Swing
                    && otherPlan.swingSeconds >= LocomotionPushWindow * otherPlan.swingDurationSeconds));
        // Pe com mola: o tornozelo de um pe no ar (sem contato nenhum) so
        // carrega o proprio pe (~1 kg) - nada a amortecer contra o chao. Ele
        // segue o alvo (posicao e velocidade) ate o toque: com a perna toda
        // amolecida perto do chao e a velocidade-alvo cortada, o tornozelo
        // virava um freio de ~0,3 s e chegava 13-23 graus atrasado no sprint
        // (o pe nao pousava como a animacao e o terreno pediam).
        const FootSupportEstimate3D& footEstimate = m_estimator.state().feet[side];
        const bool ankle = linkId.find("Foot") != std::string::npos;
        const bool footInAir = input.footSpring && input.forceDrivenRoot && !getUpActive
            && !footEstimate.contactObserved && !footEstimate.supporting;
        const bool agileAnkle = footInAir && ankle && gaitMoving;
        // O joelho tambem: amolecido e sem velocidade-alvo nos ultimos 12 cm,
        // ele chegava ao toque 17-23 graus mais dobrado que o alvo - a canela
        // inclinada para tras e o pe pousando com a ponta para baixo mesmo
        // com o tornozelo certo. Ele so cede depois do contato (aceitacao de
        // peso, pouso forte).
        // (So na passada: caindo de um pulo o joelho chega macio - rigido,
        // o pouso quicava, rebote 1,07 -> 1,31 m/s.)
        const bool agileKnee = footInAir && gaitMoving && linkId.find("Shin") != std::string::npos;
        // TEMPTUNE: empurrao - o pe apoiado ja atras do corpo (a segunda
        // metade do apoio, ate a ponta sair). A perna estende com mais forca
        // e segue a velocidade do alvo: mole e freada (velocidade-alvo zero),
        // ela ficava 15-20 graus atras do pedido e saia do chao dobrada (o
        // joelho a ~34 graus no trote). O pouso continua macio: rigida nele,
        // a perna quicava nas curvas.
        bool pushPhase = false;
        float pushWeight = 0.0f;
        // (So na corrida estabelecida, com o jogador pedindo para correr:
        // freando, a perna de tras rigida empurrava o corpo por cima do pe
        // de freada - 5 -> 14 quedas em 256 paradas.)
        // (Nem logo depois do pouso de um pulo: o pe freia, o corpo gira
        // para a frente sobre ele e a perna rigida empurrando por tras o
        // jogava de cara no chao - pulo em sprint, 0 -> 5 quedas em 12.)
        const bool steadyRun = m_brakeGait <= 0.0f && !m_brakeStepping && !m_startClipActive
            && m_gaitChange <= 0.0f && m_tripRecovery <= 0.001f && !m_stumbleStepping
            && m_sinceTouchdownSeconds > PushAfterLandingSeconds
            && std::hypot(input.intent.requestedVelocityWorld.x, input.intent.requestedVelocityWorld.y) > 1.0f;
        if (leg && gaitMoving && input.forceDrivenRoot && !getUpActive && footEstimate.supporting
            && PushStiffness != 1.0f && steadyRun) {
            const CharacterPhysicalState3D& body = m_estimator.state();
            const Vec3 velocity { body.centerOfMassVelocityWorld.x, body.centerOfMassVelocityWorld.y, 0.0f };
            const float speedNow = velocity.length();
            if (speedNow > 1.0f) {
                const float behind = -dot(Vec3 { footEstimate.soleCenterWorld.x - body.centerOfMassWorld.x,
                    footEstimate.soleCenterWorld.y - body.centerOfMassWorld.y, 0.0f },
                    velocity * (1.0f / speedNow));
                pushPhase = behind > PushBehindMeters;
                // TEMPTUNE: em rampa (do pe embaixo do corpo ate 10 cm atras
                // do limite), nao num degrau: ligada de uma vez, a rigidez
                // dobrava num tick e a perna dava um tranco.
                pushWeight = PushRamp ? smoothStep((behind - (PushBehindMeters - 0.05f)) / 0.10f)
                    : (pushPhase ? 1.0f : 0.0f);
                pushPhase = pushWeight > 0.0f;
            }
        }
        const float velocityShare = input.forceDrivenRoot && !getUpActive && leg
            ? (agileAnkle || agileKnee || (pushPhase && PushVelocity) ? transition
                : physicalSupport[side] && !plannedSwing && !propulsive ? 0.0f
                : transition * (propulsive ? 1.0f : 1.0f - touchdownShare[side])) : 1.0f;
        // Same per-category get-up gains as
        // BiomechanicalBipedExperiment3D's: soft and passive while
        // Resting, firm (legs firmest) while actively recovering.
        // Default mode holds the pose as hard as the drives allow, and lets
        // go in step with physicsBlend. This is the "force the pose when
        // nothing is happening, yield when something is" dial: at blend 0
        // the motors run at their ceiling, at blend 1 they are back to the
        // profile's nominal strength and the body argues back.
        float poseHold = 1.0f + (1.0f - m_physicsBlend);
        // No modo fisico o corpo e sempre o fisico: sem "segurar a pose" no
        // dobro da forca (herdado do modo antigo). Mais macio, ele sente as
        // pancadas (esbarrar numa parede, num boneco) em vez de rebater duro.
        if (input.forceDrivenRoot) poseHold = UprightMuscleTone;
        // Caindo: tonus de quem se protege - firme o bastante para os
        // reflexos agirem, sem a rigidez de quem esta de pe. Bracos e
        // pescoco rapidos (as maos tem de chegar ao chao antes da cabeca).
        {
            const std::string& id = profile.links[index].id;
            const bool reflexLimb = id.find("Arm") != std::string::npos
                || id.find("Forearm") != std::string::npos
                || id == "Neck" || id == "Head";
            // Tronco e pernas amolecem so com a queda sem volta; os bracos
            // e o pescoco ja com o reflexo.
            const bool legLink = id.find("Thigh") != std::string::npos
                || id.find("Shin") != std::string::npos || id.find("Foot") != std::string::npos;
            const bool armLink = id.find("Arm") != std::string::npos || id.find("Forearm") != std::string::npos;
            // A mao do lado segurando o chao: o braco firma um pouco para
            // amortecer (cede pelo alvo do cotovelo, nao mole).
            const bool catching = FallProtectNew && armLink
                && m_handCatchSeconds[id.rfind("Left", 0) == 0 ? 0 : 1] > 0.0f;
            const float fallTone = !FallProtectNew ? (reflexLimb ? FallReflexLimbTone : FallBodyTone)
                : catching ? FallCatchTone : reflexLimb ? FallReflexLimbTone
                : legLink ? (FallLegsNew ? FallLegTone : FallBodyTone) : FallTrunkTone;
            poseHold += (fallTone - poseHold) * (reflexLimb ? m_fallReflex : m_fallCommit);
            // No ar a pose e firme (os membros nao carregam peso): bracos e
            // pernas seguram a pose de equilibrio, sem chacoalhar.
            if ((armLink || legLink) && m_airBalance > 0.0f)
                poseHold += (std::max(poseHold, FallAirTone) - poseHold) * m_airBalance;
        }
        float stiffnessMultiplier = poseHold;
        float dampingMultiplier = std::sqrt(poseHold);
        float torqueMultiplier = poseHold;
        // Caindo, freio baixo tambem: a raiz quadrada do tonus deixava metade
        // do freio de pe e o corpo caia "posado", devagar nas juntas.
        {
            const std::string& id = profile.links[index].id;
            const bool reflexLimb = id.find("Arm") != std::string::npos
                || id.find("Forearm") != std::string::npos
                || id == "Neck" || id == "Head";
            const float falling = reflexLimb ? m_fallReflex : m_fallCommit;
            dampingMultiplier += ((reflexLimb ? 0.55f : 0.15f)
                - dampingMultiplier) * falling;
        }
        if (input.forceDrivenRoot && !getUpActive && leg) {
            // Changing a stride can ask for an incompatible extension on a
            // loaded leg. Let it yield while the new support is established.
            // (Caindo sem volta, a perna tem o tonus da queda, sem o
            // amolecer da passada: com os dois, ~14%, ela nem dobrava e ele
            // caia reto como uma tabua.)
            // TEMPTUNE LandingLegTone: no pouso de um pulo a troca de clipe
            // (pulo -> pouso -> corrida) recomecava a mistura e as pernas iam
            // a 35% da forca justo no impacto - o joelho ia a 130 graus, a
            // pelve afundava 30 cm e, agachado a 7,5 m/s, o tronco tombava para
            // a frente (correndo inclinado, segurado pela ajuda).
            const float transitionTone = std::max(transition,
                LandingLegTone > 0.0f && m_sinceTouchdownSeconds < LandingLegToneSeconds
                    && footEstimate.supporting ? LandingLegTone : 0.0f);
            const float transfer = FallProtectNew && FallLegsNew
                ? 0.35f + 0.65f * transitionTone + (1.0f / 0.55f - 0.35f - 0.65f * transitionTone) * m_fallCommit
                : 0.35f + 0.65f * transitionTone;
            if (agileAnkle) {
                // No ar: mola cheia e menos freio (ver agileAnkle).
                dampingMultiplier *= AnkleAirDamping;
            } else if (agileKnee) {
                // No ar: a mola da troca de apoio, sem o amolecer antes do toque.
                stiffnessMultiplier *= transfer * 0.55f;
                torqueMultiplier *= transfer * 0.75f;
            } else {
                // (Perna no chao: StanceLegStiffness - era 0,55 sempre, e o
                // joelho de apoio cedia 5-15 graus alem do pedido: "sempre
                // semi-agachado".)
                stiffnessMultiplier *= transfer * (footEstimate.supporting ? StanceLegStiffness : 0.55f);
                torqueMultiplier *= transfer * 0.75f;
                if (pushPhase) {
                    stiffnessMultiplier *= 1.0f + (PushStiffness - 1.0f) * pushWeight;
                    torqueMultiplier *= 1.0f + (std::max(1.0f, PushStiffness * 0.75f) - 1.0f) * pushWeight;
                }
                // Prepare compliance before impact, while the descending sole
                // approaches its measured surface. Retain damping through touch.
                stiffnessMultiplier *= 1.0f - 0.30f * touchdownShare[side];
                dampingMultiplier *= 1.0f + 0.30f * touchdownShare[side];
            }
            // Com pouca ajuda a perna no ar tem de chegar onde o pe vai pousar
            // (ninguem leva o corpo ate ela): amolecida como quando a pelve
            // levava o corpo, ela ficava 15-20 graus atras do alvo e o pe
            // tocava o chao antes, atras do corpo (TEMPTUNE: em teste).
            // (Tambem nos passos do planejador - freada, recuperacao: parado,
            // o passo de freada a 5% ficava 0,5 s no ar, e o corpo caia de
            // lado sobre o outro pe.)
            const bool plannerStepInAir = PlannerSwingLowAssist && footInAir && !legPlan.followingGait
                && (legPlan.phase == FootPhase3D::Swing || legPlan.phase == FootPhase3D::TouchdownSearch);
            if (lowAssist > 0.0f && ((footInAir && gaitMoving
                    && m_gaitDirection == CharacterGaitDirection3D::Forward) || plannerStepInAir)) {
                stiffnessMultiplier *= 1.0f + (SwingLowAssistStiffness - 1.0f) * lowAssist;
                torqueMultiplier *= 1.0f + (SwingLowAssistTorque - 1.0f) * lowAssist;
            }
            // O tornozelo apoiado continua sendo uma mola, nao uma junta
            // soldada: cede para a sola adaptar-se e devolve energia na
            // rolagem ate o antepe. O equilibrio ainda pode modular o alvo;
            // reduzimos a rigidez/torque, sem soltar o controle do pe.
            if (input.footSpring && ankle && !agileAnkle) {
                stiffnessMultiplier *= 0.82f;
                dampingMultiplier *= 0.92f;
                torqueMultiplier *= 0.86f;
            }
            // Mola do pe no toque: o tornozelo cede e amortece enquanto o peso
            // chega (o pe assenta no chao, o corpo nao bate na perna rigida)
            // e volta a firmar em ~0,15 s, a tempo do equilibrio e do impulso.
            if (input.footSpring && ankle && footEstimate.supporting
                && footEstimate.supportingSeconds < AnkleLandingSeconds) {
                const float settle = smoothStep(footEstimate.supportingSeconds / AnkleLandingSeconds);
                stiffnessMultiplier *= AnkleLandingStiffness + (1.0f - AnkleLandingStiffness) * settle;
                dampingMultiplier *= 1.0f + AnkleLandingDamping * (1.0f - settle);
            }
            // Pouso forte: a perna amortece (menos mola, mais freio) enquanto
            // a pelve desce, e volta a firmar ao subir.
            if (m_sinceTouchdownSeconds < m_landingSettleSeconds) {
                const float settle = std::clamp(m_sinceTouchdownSeconds
                    / m_landingSettleSeconds, 0.0f, 1.0f);
                const float absorb = smoothStep((m_landingImpactSpeed - 1.5f) / 2.5f)
                    * (1.0f - smoothStep((settle - 0.40f) / 0.60f));
                stiffnessMultiplier *= 1.0f - 0.35f * absorb;
                dampingMultiplier *= 1.0f + 0.80f * absorb;
            }
            // Aceitacao de peso (E5, fase 9): a perna que acabou de pousar
            // num passo de marcha e 25% mais mole durante a carga e os
            // primeiros 0,15 s de apoio - rigida, ela entrava como escora e
            // freava o corpo no toque. Grade medida (1 m/s, 16 corridas,
            // rigidez x amortecimento): 1,0 x 1,0 perdia 0,085 m/s na colisao
            // e 0,116 ate a saida do outro pe; 0,75 x 1,0, 0,046 e 0,069, mesmas
            // quedas, regime 76 -> 80%; mais mole (0,45-0,60) caia mais.
            if (m_footwork.settings().acceptanceCompliance
                && !legPlan.followingGait && legPlan.reason == StepReason3D::Locomotion
                && (legPlan.phase == FootPhase3D::Loading
                    || (legPlan.phase == FootPhase3D::Stance
                        && m_estimator.state().feet[side].supportingSeconds < 0.15f)))
                stiffnessMultiplier *= LocomotionAcceptanceStiffness;
        }
        if (getUpActive) {
            const std::string& id = profile.links[index].id;
            const bool isArm = id.find("Arm") != std::string::npos
                || id.find("Forearm") != std::string::npos
                || id.find("Hand") != std::string::npos;
            const bool isTorso = id == "Abdomen" || id == "Chest"
                || id == "UpperChest" || id == "Neck" || id == "Head";
            // Deitado com tonus: cabeca e tronco firmes (a cabeca nao bate
            // solta), bracos e pernas moderados - vezes o tonus do momento
            // (cede rolando, volta antes de levantar). O amortecimento fica:
            // cedendo, a junta ainda freia o movimento (nao e corpo morto).
            const float lyingStiffness = (isTorso ? 0.70f : isArm ? 0.50f : 0.55f)
                * m_settleTone;
            const float lyingTorque = (isTorso ? 0.90f : isArm ? 0.65f : 0.75f)
                * std::max(0.5f, m_settleTone);
            if (m_getUpPhase == CharacterGetUpPhase3D::Settling) {
                // Largado, quase sem freio nem mola: o alvo persegue o corpo,
                // entao mola x atraso + amortecimento viram um freio viscoso.
                // Com ~5 N.m.s/rad um antebraco quase em pe (peso ~1,5 kg)
                // levava ~3 s para tombar; num ragdoll, ~0,3 s. Fica o teto
                // baixo de torque - o "pouquinho de forca". Se arrumando, tudo
                // volta ao nominal com o tonus.
                const float rest = 1.0f - m_settleGather;
                stiffnessMultiplier = lyingStiffness * (1.0f - 0.75f * rest);
                dampingMultiplier = 1.0f - 0.97f * rest;
                torqueMultiplier = lyingTorque * (1.0f - 0.60f * rest);
            } else {
                // Levantar: os bracos sao o apoio (a flexao de bracos de quem
                // levanta de bruços). Entra a partir do tonus deitado em
                // ~0,5 s, nao de uma vez.
                // (No carrinho ele vem correndo, firme: sem a rampa do deitado.)
                const float rising = m_tackling ? 1.0f : smoothStep(m_getUpPhaseSeconds / 0.5f);
                stiffnessMultiplier = lyingStiffness + ((isArm ? RisingArmStiffness
                    : isTorso ? RisingTorsoStiffness : RisingLegStiffness)
                    - lyingStiffness) * rising;
                dampingMultiplier = 1.0f + ((isArm ? 1.45f : isTorso ? 1.25f : 1.30f)
                    - 1.0f) * rising;
                torqueMultiplier = lyingTorque + ((isArm ? RisingArmTorque
                    : isTorso ? RisingTorsoTorque : RisingLegTorque)
                    - lyingTorque) * rising;
                // Rigidez ao levantar (slider do laboratorio): mais relaxado,
                // a mola cede mais; o freio acompanha pela raiz (mesma
                // proporcao de amortecimento) e a forca maxima fica.
                const float tone = m_tackling ? 1.0f : std::clamp(input.getUpStiffness, 0.2f, 1.5f);
                stiffnessMultiplier *= 1.0f + (tone - 1.0f) * rising;
                dampingMultiplier *= 1.0f + (std::sqrt(tone) - 1.0f) * rising;
                // E a forca (slider do laboratorio, pedido do usuario, 05/10
                // noite: no comeco do levantar ele fazia movimentos tao
                // fortes que pulava no chao).
                const float strength = m_tackling ? 1.0f : std::clamp(input.getUpStrength, 0.2f, 1.5f);
                torqueMultiplier *= 1.0f + (strength - 1.0f) * rising;
                // Carrinho: musculos de quem corre, nao os do levantar (com
                // eles a passada de entrada e a de saida viravam saltos - a
                // perna apoiada lancava o corpo por cima dela). Descendo para
                // o chao as pernas cedem (deixar-se cair no deslize); subindo
                // do chao elas ganham forca.
                if (m_tackling) {
                    const bool isLeg = !isArm && !isTorso;
                    const float drop = smoothStep((-m_tackleExpectedRise - 0.4f) / 0.8f);
                    const float push = smoothStep((m_tackleExpectedRise - 0.2f) / 0.6f)
                        * smoothStep((0.80f - m_tackleExpectedHeight) / 0.25f);
                    const float legShare = isLeg ? (1.0f - 0.75f * drop) * (1.0f + 0.5f * push) : 1.0f;
                    stiffnessMultiplier = (isArm ? 1.0f : isTorso ? 1.2f : 1.0f) * legShare;
                    dampingMultiplier = isTorso ? 1.2f : 1.1f;
                    torqueMultiplier = (isArm ? 1.0f : isTorso ? 1.3f : 1.1f) * legShare;
                }
            }
            if (bodilyRising && index < m_bodilyPose.stiffness.size()) {
                stiffnessMultiplier = m_bodilyPose.stiffness[index];
                dampingMultiplier = m_bodilyPose.damping[index];
                torqueMultiplier = m_bodilyPose.torque[index];
            }
            torqueMultiplier = std::min(torqueMultiplier, GetUpTorqueCap);
        }
        // Largado, os membros pesam. Com a gravidade compensada inteira eles
        // ficavam onde estavam, sem peso: o corpo caia e deitava "posado", de
        // costas com as maos a 25-37 cm do chao. Um resto sustenta (tonus);
        // se arrumando para levantar, a compensacao volta com o tonus.
        float gravityScale = 1.0f;
        {
            const std::string& id = profile.links[index].id;
            const bool reflexLimb = id.find("Arm") != std::string::npos
                || id.find("Forearm") != std::string::npos
                || id.find("Hand") != std::string::npos
                || id == "Neck" || id == "Head";
            if (m_getUpPhase == CharacterGetUpPhase3D::Settling) {
                gravityScale = LyingGravity + (1.0f - LyingGravity) * m_settleGather;
            } else if (m_getUpPhase == CharacterGetUpPhase3D::Rising && m_tackling) {
                gravityScale = 1.0f;
            } else if (m_getUpPhase == CharacterGetUpPhase3D::Rising) {
                const float from = LyingGravity + (1.0f - LyingGravity) * m_settleGather;
                gravityScale = from + (1.0f - from)
                    * smoothStep(m_getUpPhaseSeconds / 0.5f);
            } else if (reflexLimb) {
                gravityScale = 1.0f - (1.0f - FallLimbGravity) * m_fallReflex;
            } else {
                gravityScale = 1.0f - (1.0f - FallBodyGravity) * m_fallCommit;
            }
        }
        for (std::size_t axis = 0; axis < 3; ++axis) {
            if (!profile.links[index].inboundJoint.axes[axis].enabled) continue;
            RagdollDriveTarget3D target;
            target.linkIndex = static_cast<std::uint32_t>(index);
            target.axis = static_cast<RagdollAxis3D>(axis);
            target.positionRadians = component(m_coordinates[index], axis);
            // Folga acima do joelho do sprint (~21 rad/s de pico).
            const float recoveryVelocityShare = m_tackling ? 1.0f
                : m_getUpPhase == CharacterGetUpPhase3D::Settling
                ? 0.0f : m_getUpPhase == CharacterGetUpPhase3D::Rising
                    ? smoothStep(m_getUpPhaseSeconds / 0.5f) : 1.0f;
            // (O mergulho do carrinho e rapido: o limite do levantar o
            // atrasava.)
            const float velocityLimit = getUpActive && !m_tackling ? 6.0f : 30.0f;
            target.velocityRadiansPerSecond = std::clamp(
                component(velocity, axis) * velocityShare * recoveryVelocityShare,
                -velocityLimit, velocityLimit);
            // Modo fisico: a junta perto de uma pancada cede (controlador
            // fisico, por link).
            // Caido ou levantando, o tonus e o de cima (deitado/levantar).
            const float yielding = !getUpActive
                    && index < input.jointStrength.size()
                ? std::clamp(input.jointStrength[index], 0.05f, 1.0f) : 1.0f;
            // Cambaleio: o tonus de cada junta vai ao piso dela (pernas
            // seguram um pouco; tronco e bracos soltam, com menos gravidade
            // compensada) e volta com ele.
            float staggerTone = 1.0f, staggerGravity = 1.0f;
            if (m_stagger > 0.0f && !getUpActive) {
                if (m_staggerLimb.size() != profile.links.size()) {
                    m_staggerLimb.assign(profile.links.size(), 0);
                    for (std::size_t link = 0; link < profile.links.size(); ++link) {
                        const std::string& id = profile.links[link].id;
                        m_staggerLimb[link] = id.find("Arm") != std::string::npos || id.find("Forearm") != std::string::npos
                                || id.find("Hand") != std::string::npos ? 2
                            : id == "Abdomen" || id == "Chest" || id == "UpperChest" || id == "Neck" || id == "Head" ? 1 : 0;
                    }
                }
                const std::uint8_t limb = m_staggerLimb[index];
                // (v2: o piso das pernas sobe com menos ajuda - sem ela quem
                // segura o corpo amolecido sao as pernas, e a 20% de tonus
                // nao saia o passo que salva: empurrado parado a 0%, ele
                // sentava para tras com os dois pes plantados.)
                const float legFloor = input.physicalV2
                    ? StaggerLegTone + (0.6f - StaggerLegTone) * (1.0f - std::clamp(input.walkingAssist, 0.0f, 1.0f))
                    : StaggerLegTone;
                // (O tronco tambem: a 10% de tonus, sem ajuda o peito tombava
                // ~75 graus de lado num empurrao correndo.)
                // (Perto de 0% de ajuda, nao: la o tronco mole absorve o
                // empurrao parado - firme, o choque ia ao corpo inteiro e ele
                // caia mais.)
                static const float torsoLowAssist = tuneTemp("T_StaggerTorsoLow", 0.30f);  // TEMPTUNE
                const float walkAssist = std::clamp(input.walkingAssist, 0.0f, 1.0f);
                const float torsoShare = (1.0f - walkAssist) * smoothStep((walkAssist - 0.1f) / 0.15f);
                const float torsoFloor = input.physicalV2
                    ? StaggerTorsoTone + (torsoLowAssist - StaggerTorsoTone) * torsoShare
                    : StaggerTorsoTone;
                const float floor = limb == 2 ? StaggerArmTone : limb == 1 ? torsoFloor : legFloor;
                staggerTone = 1.0f - m_stagger * (1.0f - floor);
                if (limb != 0) staggerGravity = 1.0f - StaggerGravityDrop * m_stagger;
            }
            // v2: com pouca ajuda, a perna de apoio da passada para a frente
            // segura a pose com mais rigidez - ate StanceStiffness vezes perto
            // de 0% (1 - ajuda), sumindo de 30% a 50% (a 70%, 1,45x dobrava as
            // quedas do tropeco na barra escondida, 6 -> 12 em 128). E a mola da pose
            // que leva o corpo na passada (o esforco por J^T mudava pouco); com
            // a rigidez normal o corpo nao passava de ~4 m/s a 20% e o pe
            // escorregava para tras. Mais macia, mais lento; 4x ou mais
            // derrubava mais nas manobras rapidas (mais velocidade nelas). Fora
            // da passada para a frente (de lado, de costas, freando, tropecando
            // ou cambaleando) fica a normal: saindo do passo de lado para a
            // corrida, a perna rigida chutava o corpo.
            float stanceStiffnessScale = 1.0f;
            if (StanceStiffness != 1.0f && input.physicalV2 && input.forceDrivenRoot && !getUpActive
                && (m_telemetry.state == CharacterLocomotionState3D::Walking
                    || m_telemetry.state == CharacterLocomotionState3D::Running)
                && m_gaitDirection == CharacterGaitDirection3D::Forward
                && m_brakeGait <= 0.0f && !m_brakeStepping && m_tripRecovery < 0.05f
                && m_stagger < 0.05f) {
                for (std::size_t leg = 0; leg < m_feet.size(); ++leg) {
                    if (!m_feet[leg].valid || !m_feet[leg].planted) continue;
                    const int foot = static_cast<int>(m_feet[leg].linkIndex);
                    const int shin = profile.links[static_cast<std::size_t>(foot)].parentIndex;
                    const int thigh = shin > 0 ? profile.links[static_cast<std::size_t>(shin)].parentIndex : -1;
                    const int link = static_cast<int>(index);
                    const float assist = std::clamp(input.walkingAssist, 0.0f, 1.0f);
                    if (link == foot || link == shin || link == thigh)
                        stanceStiffnessScale = 1.0f + (StanceStiffness - 1.0f)
                            * (1.0f - assist) * smoothStep((0.5f - assist) / 0.2f);
                }
            }
            target.stiffnessScale =
                muscle * stiffnessMultiplier * impactSoftening * yielding * staggerTone * stanceStiffnessScale;
            target.dampingScale =
                std::sqrt(muscle) * dampingMultiplier * std::sqrt(staggerTone);
            target.maximumTorqueScale =
                muscle * torqueMultiplier * impactSoftening * yielding * staggerTone;
            target.gravityCompensationScale = gravityScale * staggerGravity;
            if (input.manipulated && index == input.grabbedLink) {
                target.stiffnessScale *= 0.20f;
                target.dampingScale *= 0.45f;
                target.maximumTorqueScale *= 0.20f;
            }
            m_output.driveTargets.push_back(target);
            const float error = target.positionRadians
                - component(measured, axis);
            squaredError += error * error;
            ++errorCount;
        }
    }

    // The joints are never written into the articulation: pose authority is
    // what makes the guide overwrite joint positions every tick, which is
    // both the visible "bones teleporting into place" when the body turns
    // and the reason limbs pass through walls - a written position never
    // meets the solver. The motors below do the work instead, so the body
    // tracks the animation AND still collides.
    //
    // What stays carried is the root: it follows the capsule, which is what
    // keeps the character upright and moving where gameplay says. Leaving
    // root rotation to physics is what made the spine lean.
    //
    // The residual on the joints is drift correction, not posing: a blend
    // toward the target each tick, never the hard assignment the guide does
    // at authority 1. Measured tracking error against the authored pose:
    // 9.7 deg RMS at 0.10, 6.1 at 0.35, 5.4 at 0.60 - the curve is flat past
    // 0.35, so this sits just high enough to hold the pose and no higher,
    // leaving contact to win. Raising the motor gains to their ceiling moved
    // it only 9.7 -> 9.0, so this is not a strength problem.
    //
    // Contact already relaxes it further on its own: the guide is staged
    // with releaseOnInteraction, and the PhysX side caps every authority by
    // 1 - 0.9*externalInterference, so touching a wall hands that limb back
    // to the solver instead of writing it through the wall.
    RagdollAnimationConstraint3D& guide = m_output.guide;
    // Never. Every joint the guide writes is a joint the solver did not
    // compute: it shivers, it ignores contact, and during a recovery - when
    // the root is free - writing joints is what threw the body across the
    // map. The motors below are the only thing that moves this character.
    guide.poseAuthority = 0.0f;
    guide.rootTranslationAuthority = rootAuthority;
    guide.rootRotationAuthority = rootAuthority;
    guide.releaseOnInteraction = true;
    guide.rootPositionWorld = targetRootPosition;
    m_output.navigationAnchorWorld = targetRootPosition - m_ballBodyDelta
        + Vec3 { input.rootPositionWorld.x - navigationRoot.x,
            input.rootPositionWorld.y - navigationRoot.y, 0.0f };
    guide.rootOrientationWorld = targetRootOrientation;
    const Vec3 finiteDifferenceVelocity =
        (targetRootPosition - m_previousRootPosition) / deltaTime;
    guide.rootLinearVelocityWorld = finite(finiteDifferenceVelocity)
        ? finiteDifferenceVelocity : input.rootVelocityWorld;
    // Nos passos de freada a raiz e o proprio corpo: a aceleracao dela e a
    // dele, e antecipada pela ajuda empurrava o corpo para onde ele ja ia -
    // freando sobre o pe da frente, ate 2700 N para tras, e ele voltava a
    // 1,2 m/s de costas depois de parar.
    guide.rootFeedforwardShare = BrakeAnchorNoFeedforward ? 1.0f - m_brakeGait * m_brakeAnchor : 1.0f;
    const Vec3 targetAngularVelocity = rotationVector(targetRootOrientation
        * m_previousRootOrientation.conjugate()) / deltaTime;
    guide.rootAngularVelocityWorld = targetAngularVelocity;
    if (referenceRebase) {
        guide.rootLinearVelocityWorld = m_previousGuideVelocity;
        guide.rootAngularVelocityWorld = m_previousGuideAngularVelocity;
    }
    m_previousGuideVelocity = guide.rootLinearVelocityWorld;
    m_previousGuideAngularVelocity = guide.rootAngularVelocityWorld;
    m_telemetry.getUpTiltExcessDegrees = 0.0f;
    m_telemetry.getUpUprightTorque = 0.0f;
    if (getUpActive && !state.links.empty()) {
        m_telemetry.getUpAssistNewtons = 0.0f;
        if (m_tackling && m_tackleClip != nullptr) {
            // Carrinho: nada empurra o corpo pelo espaco - o mergulho e o
            // deslize sao o embalo e o atrito (a ajuda que acompanhava o
            // caminho do clipe encurtava e "arrastava" o deslize). Fica so a
            // atitude: a pelve gira para a inclinacao do clipe (sem ela o
            // embalo capotava o corpo por cima do pe apoiado). E no levantar
            // do deslize, uma ajuda para cima.
            const float mass = std::max(1.0f, profile.totalMassKg);
            const auto& pelvis = state.links.front();
            const float duration = m_tackleClip->durationSeconds;
            constexpr float Step = 1.0f / 30.0f;
            const float t0 = std::min(m_getUpClipSeconds, duration);
            const float t1 = std::min(t0 + Step, duration);
            const AnimationTransformSample3D now = clipRootAt(profile, *m_tackleClip, t0);
            const AnimationTransformSample3D next = clipRootAt(profile, *m_tackleClip, t1);
            const Quaternion turn = Quaternion::fromAxisAngle({ 0.0f, 0.0f, 1.0f }, m_tackleYaw);
            const Quaternion wanted = (turn * now.rotationDelta).normalized();
            const Quaternion wantedNext = (turn * next.rotationDelta).normalized();
            const float rate = t1 > t0 ? m_getUpRate / Step : 0.0f;
            const Vec3 wantedSpin = rotationVector(wantedNext * wanted.conjugate()) * rate;
            const Vec3 orientationError = rotationVector(wanted * pelvis.orientation.conjugate());
            m_output.rootControlTorqueWorld = clampMagnitude(orientationError * (mass * 9.0f)
                + (wantedSpin - pelvis.angularVelocity) * (mass * 1.2f), mass * 8.0f) * TackleAttitude;
            // O impulso do mergulho: o pe de apoio ainda no chao, o corpo no
            // alto e o clipe descendo (a saida), a perna empurra na direcao
            // do carrinho - sem ele o corpo chegava ao chao com metade da
            // velocidade de entrada (o carrinho lento e curto).
            {
                const std::size_t plant = static_cast<std::size_t>(m_tackleStartSide);
                const float bodyHeight = pelvis.position.z - groundHeightGuess;
                const bool takeoff = feetGroundContact[plant] && bodyHeight > 0.40f
                    && m_getUpClipSeconds < 0.45f * duration && m_tackleExpectedRise < 0.2f;
                if (takeoff) {
                    const Vec3 direction { std::cos(m_tackleYaw), std::sin(m_tackleYaw), 0.0f };
                    const float along = dot(Vec3 { pelvis.linearVelocity.x, pelvis.linearVelocity.y, 0.0f }, direction);
                    const float wanted = m_tackleEntrySpeed * TackleImpulseShare;
                    const float push = std::clamp((wanted - along) * 8.0f, 0.0f, 9.81f * TackleImpulseForce);
                    m_output.rootControlForceWorld += direction * (push * mass);
                    m_telemetry.getUpAssistNewtons = m_output.rootControlForceWorld.length();
                }
            }
            const float standing = profile.standingRootHeightMeters;
            const float height = pelvis.position.z - groundHeightGuess;
            // Levantando do deslize (o clipe subindo) e, da metade em diante,
            // o corpo ainda abaixo da altura do clipe (ele termina de subir).
            const float late = m_getUpClipSeconds > 0.5f * duration
                ? smoothStep((m_tackleExpectedHeight - height - 0.04f) / 0.10f) : 0.0f;
            const float recovering = std::max(late, smoothStep((m_tackleExpectedRise - 0.05f) / 0.25f)
                * smoothStep((standing * 0.92f - m_tackleExpectedHeight) / 0.10f));
            if (recovering > 0.0f) {
                const float lift = std::clamp(36.0f * (m_tackleExpectedHeight - height)
                    + 12.0f * (m_tackleExpectedRise - pelvis.linearVelocity.z), 0.0f, 9.81f * 0.5f);
                // (Propria do carrinho, nao a do slider de levantar: com ela em
                // 0, 43% dos carrinhos nao levantavam do deslize.)
                const float assist = recovering * TackleRecoveryAssist;
                m_output.rootControlForceWorld = Vec3 { 0.0f, 0.0f, lift * mass } * assist;
                m_telemetry.getUpAssistNewtons = m_output.rootControlForceWorld.length();
            }
        } else if (m_getUpPhase == CharacterGetUpPhase3D::Rising && !m_bodilyRising) {
            // Ajuda discreta do levantar: quem levanta e o corpo (os motores
            // levando as juntas ao clipe, maos/joelhos/pes empurrando o
            // chao). A pelve so recebe o que falta para acompanhar o clipe -
            // altura e endireitar -, suave, pequena (30% do peso; ate 65% se
            // o corpo emperra) e so com apoio real no chao: sem mao, joelho
            // ou pe tocando, nada (nao levita).
            const float mass = std::max(1.0f, profile.totalMassKg);
            const auto& pelvis = state.links.front();
            const float ground = groundHeightGuess;
            const float expectedHeight = profile.standingRootHeightMeters
                + rootTranslation.z;
            const Vec3 clipUp = rootRotation.rotate({ 0.0f, 0.0f, 1.0f });
            const float expectedTilt = std::acos(std::clamp(clipUp.z, -1.0f, 1.0f));
            const float expectedRise = m_getUpExpectedValid
                ? (expectedHeight - m_getUpExpectedHeight) / deltaTime : 0.0f;
            m_getUpExpectedHeight = expectedHeight;
            m_getUpExpectedTilt = expectedTilt;
            m_getUpExpectedValid = true;
            bool support = false;
            for (const RagdollInteraction3D& contact : state.interactions) {
                if (contact.normalWorld.z < 0.5f
                    || contact.linkIndex >= profile.links.size()) continue;
                const std::string& id = profile.links[contact.linkIndex].id;
                if (id.find("Hand") != std::string::npos
                    || id.find("Forearm") != std::string::npos
                    || id.find("Shin") != std::string::npos
                    || id.find("Foot") != std::string::npos) {
                    support = true;
                }
            }
            m_getUpSupportAge = support ? 0.0f
                : std::min(5.0f, m_getUpSupportAge + deltaTime);
            // So com o centro de massa sobre os apoios de verdade (maos,
            // joelhos, pes no chao): fora deles (> ~8 cm) a ajuda de altura e
            // de endireitar some - com ela o corpo ficava "segurado" como uma
            // prancha, o centro de massa ate 0,6 m fora de qualquer apoio.
            float overSupport = 1.0f;
            {
                std::vector<std::array<float, 2>> points;
                for (const RagdollInteraction3D& contact : state.interactions) {
                    if (contact.normalWorld.z < 0.5f) continue;
                    points.push_back({ contact.positionWorld.x,
                        contact.positionWorld.y });
                }
                if (!points.empty()) {
                    overSupport = 1.0f - smoothStep((planarDistanceOutsideHull3D(points,
                        centerOfMass.x, centerOfMass.y) - 0.08f) / 0.20f);
                }
            }
            m_telemetry.getUpOverSupport = overSupport;
            const float contactShare = 1.0f
                - smoothStep((m_getUpSupportAge - 0.15f) / 0.25f);
            // (overSupport so medido: cortar a ajuda por ele fazia o levantar
            // falhar - os pontos de contato do backend sao o ponto dominante
            // de cada segmento, a base medida fica menor que a real.)
            (void)overSupport;
            const float supportShare = contactShare;
            const float height = pelvis.position.z - ground;
            constexpr float LiftOmega = 6.0f;
            // Cada tentativa que falha, um pouco mais de esforco (ate o
            // dobro na terceira).
            // A ajuda entra aos poucos (~1 s): o corpo comeca sozinho e ela
            // vai chegando, em vez de pegar o corpo no primeiro tick.
            m_getUpAssistRamp = smoothStep(m_getUpPhaseSeconds * m_getUpSpeedNow / 1.0f);
            const float effort = std::min(2.0f, 1.0f
                + 0.5f * static_cast<float>(m_getUpAttempt > 0 ? m_getUpAttempt - 1 : 0))
                * m_getUpAssistRamp;
            const float liftCap = 9.81f * (0.30f + 0.35f * m_getUpStruggle) * effort;
            // (Perto da altura de pe ela tambem freia: com o clipe mais
            // rapido, o embalo da subida e as pernas esticando lancavam o
            // corpo - a pelve a 1,17 m, um pulinho no fim do levantar.)
            const float brakeShare = GetUpLiftBrake * smoothStep((height - 0.70f) / 0.15f);
            const float lift = std::clamp(
                LiftOmega * LiftOmega * (expectedHeight - height)
                    + 2.0f * LiftOmega * (expectedRise - pelvis.linearVelocity.z),
                -liftCap * brakeShare, liftCap) * supportShare;
            // Endireitar: gira a pelve no sentido de ficar em pe, ate a
            // inclinacao que o clipe tem agora.
            const Vec3 up = pelvis.orientation.rotate({ 0.0f, 0.0f, 1.0f });
            const float tilt = std::acos(std::clamp(up.z, -1.0f, 1.0f));
            Vec3 axis = cross(up, { 0.0f, 0.0f, 1.0f });
            Vec3 torque {};
            if (axis.lengthSquared() > 1e-6f) {
                axis = axis.normalized();
                const float excess = std::max(0.0f, tilt - expectedTilt);
                const float spin = dot(pelvis.angularVelocity, axis);
                // Fase final (o clipe ja quase em pe, a pelve alta): mais forca
                // para endireitar - como um par pelve x pes, e a extensao do
                // quadril que traz o tronco para cima e o peso para tras, sobre
                // os pes. Com o teto de antes (~130 N.m) ele ficava ~1 s como
                // uma prancha 20-27 graus alem do clipe, o centro de massa
                // 0,4-0,5 m a frente dos pes, segurado pela ajuda.
                const float late = smoothStep((0.8f - expectedTilt) / 0.4f)
                    * smoothStep((height - 0.55f) / 0.15f);
                const float cap = mass * (1.5f + 1.5f * m_getUpStruggle
                    + GetUpPelvisLateCap * late) * effort;
                torque = axis * std::clamp((excess * 22.0f * (1.0f + GetUpPelvisLateGain * late)
                    - spin * 4.5f * (1.0f + 0.5f * GetUpPelvisLateGain * late)) * mass,
                    -cap, cap) * supportShare;
                m_telemetry.getUpTiltExcessDegrees = excess * 57.2958f;
                m_telemetry.getUpExpectedTiltDegrees = expectedTilt * 57.2958f;
            }
            // O peito tambem: a altura dele no clipe agora (cinematica direta
            // da pose do clipe), com o mesmo teto dividido.
            float chestLift = 0.0f;
            if (m_chestLink < state.links.size()) {
                RagdollAnimationPose3D clipPose;
                buildLocalPose(profile, sampled, clipPose);
                const float expectedChest = profile.standingRootHeightMeters
                    + rootTranslation.z
                    + rootRotation.rotate(clipPose.linkPositions[m_chestLink]).z;
                const float chest = state.links[m_chestLink].position.z - ground;
                m_getUpExpectedChestHeight = expectedChest;
                // E a inclinacao do tronco: de bruços o clipe ergue o tronco
                // (cabeca para cima, maos no chao); sem isso o tronco
                // apontava para o chao e o corpo se encolhia de cabeca baixa.
                {
                    const Quaternion clipChest = (rootRotation
                        * clipPose.linkOrientations[m_chestLink]).normalized();
                    const Vec3 clipChestUp = clipChest.rotate({ 0.0f, 0.0f, 1.0f });
                    const float expectedChestTilt = std::acos(
                        std::clamp(clipChestUp.z, -1.0f, 1.0f));
                    m_telemetry.getUpExpectedChestTiltDegrees = expectedChestTilt * 57.2958f;
                    const auto& chestBody = state.links[m_chestLink];
                    const Vec3 chestUp = chestBody.orientation.rotate({ 0.0f, 0.0f, 1.0f });
                    Vec3 chestAxis = cross(chestUp, { 0.0f, 0.0f, 1.0f });
                    if (chestAxis.lengthSquared() > 1e-6f) {
                        chestAxis = chestAxis.normalized();
                        const float excess = std::max(0.0f, std::acos(
                            std::clamp(chestUp.z, -1.0f, 1.0f)) - expectedChestTilt);
                        const float spin = dot(chestBody.angularVelocity, chestAxis);
                        // (Na fase final, de pe, mais forca: o peito ficava ~25
                        // graus caido alem do clipe e o corpo, de pe e
                        // dobrado no quadril, ~0,55 s ate a ajuda o erguer -
                        // o levantar de bruços. Pedido do usuario, 05/10.)
                        const float chestLate = smoothStep((0.8f - expectedTilt) / 0.4f)
                            * smoothStep((height - 0.55f) / 0.15f);
                        const float cap = mass * (0.8f + 0.8f * m_getUpStruggle + GetUpChestLateCap * chestLate) * effort;
                        m_output.chestAssistTorqueWorld = chestAxis * std::clamp(
                            (excess * 14.0f * (1.0f + GetUpChestLateGain * chestLate) - spin * 3.0f) * mass, -cap, cap)
                            * supportShare;
                    }
                }
                chestLift = std::clamp(
                    LiftOmega * LiftOmega * (expectedChest - chest)
                        + 2.0f * LiftOmega * (expectedRise
                            - state.links[m_chestLink].linearVelocity.z),
                    0.0f, liftCap) * supportShare;
                m_output.chestAssistLink = static_cast<std::uint32_t>(m_chestLink);
            }
            // O teto vale para a soma: pelve e peito dividem a mesma ajuda.
            const float total = lift + chestLift;
            const float scale = total > liftCap && total > 0.0f ? liftCap / total : 1.0f;
            // Erguendo-se (o clipe ja quase em pe): o centro de massa vai para
            // cima dos pes que estao no chao, como quem se equilibra ao
            // levantar - de leve, ate 15% do peso.
            Vec3 balance {};
            if (expectedTilt < 0.8f) {
                Vec3 feet {};
                int touching = 0;
                for (const RagdollInteraction3D& contact : state.interactions) {
                    if (contact.normalWorld.z < 0.5f
                        || contact.linkIndex >= profile.links.size()) continue;
                    if (profile.links[contact.linkIndex].id.find("Foot")
                        == std::string::npos) continue;
                    feet += state.links[contact.linkIndex].position;
                    ++touching;
                }
                if (touching > 0) {
                    feet *= 1.0f / static_cast<float>(touching);
                    Vec3 offset { feet.x - centerOfMass.x, feet.y - centerOfMass.y, 0.0f };
                    Vec3 velocity = m_filteredCenterOfMassVelocity;
                    velocity.z = 0.0f;
                    // (Esta continua fora da base: e ela que traz o centro de
                    // massa de volta para cima dos pes.)
                    balance = clampMagnitude(offset * 30.0f - velocity * 8.0f,
                        9.81f * 0.22f * effort) * (mass * contactShare);
                }
            }
            m_output.rootControlForceWorld = Vec3 { balance.x, balance.y,
                lift * scale * mass };
            m_output.chestAssistForceWorld = { 0.0f, 0.0f, chestLift * scale * mass };
            // Endireitar e um par interno, como um musculo: a pelve (e o
            // peito) giram para cima e a reacao vai para os segmentos que
            // estao apoiados no chao, pela forca de apoio de cada um. Assim
            // ele so sustenta o que os apoios sustentariam - com o centro de
            // massa fora deles, e o pe ou a mao que cede. Como torque externo
            // ele segurava o corpo como uma prancha inclinada ~35 graus: a
            // ajuda ficava evidente.
            {
                std::vector<std::pair<std::uint32_t, float>> supports;
                float supportSum = 0.0f;
                for (const RagdollInteraction3D& contact : state.interactions) {
                    if (contact.normalWorld.z < 0.5f || contact.linkIndex == 0
                        || contact.linkIndex == m_chestLink
                        || contact.linkIndex >= profile.links.size()) continue;
                    const float weight = std::max(0.001f,
                        contact.normalImpulseNewtonSeconds);
                    supportSum += weight;
                    auto existing = std::find_if(supports.begin(), supports.end(),
                        [&](const auto& item) { return item.first == contact.linkIndex; });
                    if (existing != supports.end()) existing->second += weight;
                    else supports.push_back({ contact.linkIndex, weight });
                }
                if (supportSum <= 0.0f) {
                    torque = {};
                    m_output.chestAssistTorqueWorld = {};
                } else {
                    const Vec3 reaction = -(torque + m_output.chestAssistTorqueWorld);
                    for (const auto& [link, weight] : supports) {
                        m_output.assistReactionLinks.push_back(link);
                        m_output.assistReactionTorquesWorld.push_back(
                            reaction * (weight / supportSum));
                    }
                }
            }
            m_output.rootControlTorqueWorld = torque;
            m_telemetry.getUpUprightTorque = torque.length();
            m_telemetry.getUpAssistNewtons = total * scale * mass;
            // A ajuda para levantar (slider do laboratorio, 0 = nenhuma; o
            // TEMPTUNE multiplica, para diagnostico).
            const float getUpAssistScale = GetUpAssistScale * std::clamp(input.getUpAssist, 0.0f, 2.0f);
            if (getUpAssistScale != 1.0f) {
                m_output.rootControlForceWorld *= getUpAssistScale;
                m_output.rootControlTorqueWorld *= getUpAssistScale;
                m_output.chestAssistForceWorld *= getUpAssistScale;
                m_output.chestAssistTorqueWorld *= getUpAssistScale;
                for (Vec3& reaction : m_output.assistReactionTorquesWorld) reaction *= getUpAssistScale;
                m_telemetry.getUpAssistNewtons *= getUpAssistScale;
            }
        }
        // Rolando: a ajuda gira o corpo em torno da coluna, no sentido da
        // barriga para o chao (vezes o slider da ajuda para levantar; a 0% so
        // a perna e o braco de cima rolam o corpo).
        if (m_getUpPhase == CharacterGetUpPhase3D::Settling && m_getUpRolling
            && m_chestLink < state.links.size()) {
            const float mass = std::max(1.0f, profile.totalMassKg);
            const Vec3 spine = (state.links[m_chestLink].position - state.links.front().position).normalized();
            const float spin = dot(state.links.front().angularVelocity, spine) * m_getUpRollSign;
            const float wanted = 2.0f;  // rad/s em torno da coluna
            const float ramp = smoothStep(m_getUpRollSeconds / 0.3f);
            const float torque = std::clamp((wanted - spin) * 0.6f, 0.0f, 1.0f) * GetUpRollTorque * mass * ramp
                * GetUpAssistScale * std::clamp(input.getUpAssist, 0.0f, 2.0f);
            m_output.rootControlTorqueWorld = spine * (m_getUpRollSign * torque);
        }
    } else if (!state.links.empty()) {
        // One path now: the body is always the physical one, so it always
        // gets a root controller. How much help it gets is the only thing
        // that changes, and that is what physicsBlend fades.
        Quaternion rotationError = targetRootOrientation
            * state.links.front().orientation.conjugate();
        const Vec3 orientationError = rotationVector(rotationError);
        const Vec3 angularVelocityError = targetAngularVelocity
            - state.links.front().angularVelocity;
        const float mass = std::max(1.0f, profile.totalMassKg);
        const Vec3 torque = orientationError * (mass * 10.0f)
            + angularVelocityError * (mass * 1.45f);
        m_output.rootControlTorqueWorld = clampMagnitude(
            torque, mass * 13.0f) * rootAuthority;

        // Capture-point balance assist, faded in by exactly how much the
        // body has been handed over to physics. Capped at half: while it is
        // going down it is supposed to be losing the argument at least
        // partly, and a fully assisted fall just looks like the character
        // refusing to fall.
        constexpr float MaximumAssistShare = 0.5f;
        const float assistShare = std::min(MaximumAssistShare,
            std::clamp(m_physicsBlend, 0.0f, 1.0f));
        Vec3 supportCenter {};
        float supportWeight = 0.0f;
        for (const FootPlant& foot : m_feet) {
            if (!foot.planted) continue;
            supportCenter += foot.lockedPositionWorld;
            supportWeight += 1.0f;
        }
        if (assistShare > 0.001f && supportWeight > 0.0f) {
            supportCenter *= 1.0f / supportWeight;
            const float comHeight = std::max(0.35f,
                centerOfMass.z - supportCenter.z);
            const float omega = std::sqrt(9.81f / comHeight);
            Vec3 capturePoint = centerOfMass
                + m_filteredCenterOfMassVelocity / omega;
            capturePoint.z = supportCenter.z;
            Vec3 assistForce = (supportCenter - capturePoint)
                * (mass * omega * 0.85f);
            assistForce.z = 0.0f;
            m_output.rootControlForceWorld = clampMagnitude(assistForce,
                mass * 9.81f * 0.16f) * assistShare;

            const Vec3 rootUp = state.links.front()
                .orientation.rotate({ 0.0f, 0.0f, 1.0f });
            const Vec3 tiltError = cross(rootUp, Vec3 { 0.0f, 0.0f, 1.0f });
            Vec3 planarAngularVelocity = state.links.front().angularVelocity;
            planarAngularVelocity.z = 0.0f;
            Vec3 tiltTorque = tiltError * (mass * 9.5f)
                - planarAngularVelocity * (mass * 0.85f);
            m_output.rootControlTorqueWorld += clampMagnitude(
                tiltTorque, mass * 1.05f) * assistShare;
        }
    }
    m_previousRootPosition = targetRootPosition;
    m_previousRootOrientation = targetRootOrientation;
    m_output.gravityCompensationEnabled = muscle > 0.0001f;
    m_output.physicsBlend = m_physicsBlend;
    // Carrinho: com o corpo baixo no clipe (mergulho e deslize) ele escorrega
    // na grama; subindo do chao, o atrito volta (pes e maos empurram).
    m_output.bodyFrictionScale = 1.0f;
    if (m_tackling && !state.links.empty()) {
        // A velocidade ao tocar o chao (a pelve chegando baixo); antes disso
        // (o joelho pode tocar antes), a de entrada com o impulso.
        const auto& pelvis = state.links.front();
        const float bodyHeight = pelvis.position.z - groundHeightGuess;
        if (m_tackleLandSpeed < 0.0f && bodyHeight < TackleSlideHeight)
            m_tackleLandSpeed = std::hypot(pelvis.linearVelocity.x, pelvis.linearVelocity.y);
        const float speed = std::max(0.5f, m_tackleLandSpeed >= 0.0f ? m_tackleLandSpeed
            : m_tackleEntrySpeed * TackleImpulseShare);
        const float distance = TackleSlideA * std::pow(speed, TackleSlideP);
        const float mu = speed * speed / (2.0f * 9.81f * std::max(0.1f, distance));
        const float slideScale = std::clamp(mu / TackleBaseFriction, 0.10f, 1.0f);
        const float slide = smoothStep((0.70f - m_tackleExpectedHeight) / 0.15f)
            * (1.0f - smoothStep((m_tackleExpectedRise - 0.10f) / 0.20f));
        m_output.bodyFrictionScale = 1.0f - (1.0f - slideScale) * slide;
    }

    m_telemetry.localVelocity = localVelocity;
    m_telemetry.speedMetersPerSecond = speed;
    m_telemetry.facingYawRadians = m_facingYaw;
    m_telemetry.viewYawErrorRadians = viewYawError;
    m_telemetry.gaitDirection = m_gaitDirection;
    m_telemetry.jumpKind = m_jumpKind;
    m_telemetry.flightPhase = m_flightPhase;
    m_telemetry.torsoTwistRadians = m_torsoTwist;
    m_telemetry.headTurnRadians = m_headTurn;
    m_telemetry.turningRateRadiansPerSecond = m_turningRate;
    m_telemetry.forwardLeanRadians = forwardLean;
    m_telemetry.lateralLeanRadians = lateralLean;
    m_telemetry.authoredCurveBlend = authoredCurveBlend;
    m_telemetry.clipFootContact = contactWeights;
    m_telemetry.authoredCurveKind = authoredCurveKind;
    m_telemetry.authoredStartActive = authoredStartActive;
    m_telemetry.landingBraceForwardRadians = landingBraceForward;
    m_telemetry.landingBraceLateralRadians = landingBraceLateral;
    m_telemetry.cyclePhase = m_cyclePhase;
    m_telemetry.playbackRate = playbackRate;
    m_telemetry.strideScale = strideScale;
    m_telemetry.terrainSlope = m_terrainSlope;
    // Curva fechada: o teto de velocidade que cabe na curva pedida - a
    // aceleracao lateral que as pernas fazem sobre a taxa com que o pedido
    // gira (a camera girando com o comando para a frente). A 7,5 m/s, virar a
    // 180 graus/s pede 2,4 g: a ajuda puxava o corpo para o rumo novo, os pes
    // nao acompanhavam (o de dentro era arrastado no chao) e ele caia. Quem
    // corre de verdade desacelera para fechar a curva. O jogo corta o pedido
    // por este teto (o mesmo do terreno); ao sair da curva ele sobe no ritmo
    // de uma arrancada. Um degrau no pedido (troca de tecla) nao e curva: a
    // taxa crua e limitada antes do filtro.
    {
        const Vec3 request { input.intent.requestedDirectionWorld.x,
            input.intent.requestedDirectionWorld.y, 0.0f };
        const bool requesting = request.lengthSquared() > 0.01f;
        const float yaw = requesting ? std::atan2(request.y, request.x) : m_requestYaw;
        const float rate = requesting && m_requestYawValid
            ? std::clamp(wrapAngle(yaw - m_requestYaw) / std::max(1e-4f, deltaTime),
                -RequestTurnRateClamp, RequestTurnRateClamp) : 0.0f;
        m_requestYaw = yaw;
        m_requestYawValid = requesting;
        m_requestTurnRate += (rate - m_requestTurnRate)
            * (1.0f - std::exp(-deltaTime / std::max(0.01f, RequestTurnFilterSeconds)));
        float curveLimit = 1000.0f;
        const float lateral = CurveSpeedAccelerationLow + (CurveSpeedAcceleration - CurveSpeedAccelerationLow)
            * std::clamp(input.walkingAssist, 0.0f, 1.0f);
        if (CurveSpeedAcceleration > 0.0f && input.forceDrivenRoot && !getUpActive
            && std::abs(m_requestTurnRate) > 0.3f)
            curveLimit = std::max(CurveSpeedMinimum, lateral / std::abs(m_requestTurnRate));
        m_curveSpeedCap = curveLimit < m_curveSpeedCap ? curveLimit
            : std::min(curveLimit, m_curveSpeedCap + CurveSpeedRecovery * deltaTime);
        if (m_curveSpeedCap < 100.0f) terrainSpeedLimit = std::min(terrainSpeedLimit, m_curveSpeedCap);
    }
    // Arrancada pelo passo: a ajuda dosada - a capsula (e o pedido) sobe no
    // ritmo de quem arranca (referencia: 2,2 m/s aos 0,37 s), nao a 14 m/s2.
    if (m_startStepActive)
        terrainSpeedLimit = std::min(terrainSpeedLimit, StartSpeedBase + StartSpeedRate * m_stumbleStepSeconds);
    else if (m_startDoseCap < 100.0f)
        terrainSpeedLimit = std::min(terrainSpeedLimit, m_startDoseCap);
    m_telemetry.terrainSpeedLimit = terrainSpeedLimit;
    m_telemetry.tripRecovery = m_tripRecovery;
    m_telemetry.stumbleExcess = m_stumblePitch - m_stumbleBaseline;
    m_telemetry.stumbleRate = m_stumblePitchRate;
    m_telemetry.stumbleDecel = m_stumbleDecel;
    m_telemetry.stumbleRisk = m_stumbleRisk;
    m_telemetry.stumbleStepping = m_stumbleStepping;
    const float longitudinal = std::abs(localVelocity.x);
    const float lateral = std::abs(localVelocity.y);
    const float total = std::max(0.0001f, longitudinal + lateral);
    m_telemetry.forwardWeight = localVelocity.x >= 0.0f
        ? longitudinal / total : 0.0f;
    m_telemetry.backwardWeight = localVelocity.x < 0.0f
        ? longitudinal / total : 0.0f;
    m_telemetry.leftWeight = localVelocity.y >= 0.0f
        ? lateral / total : 0.0f;
    m_telemetry.rightWeight = localVelocity.y < 0.0f
        ? lateral / total : 0.0f;
    m_telemetry.poseAuthority = m_output.guide.poseAuthority;
    m_telemetry.rootAuthority = rootAuthority;
    m_telemetry.rootRotationAuthority = m_output.guide.rootRotationAuthority;
    m_telemetry.muscleAuthority = muscle;
    m_telemetry.reactionStrength = m_reactionStrength;
    m_telemetry.jointErrorRmsDegrees = errorCount > 0
        ? std::sqrt(squaredError / static_cast<float>(errorCount))
            * 180.0f / Pi
        : 0.0f;
    m_telemetry.rootUpright = rootUpright;
    m_telemetry.fallOrientation = m_fallOrientation;
    m_telemetry.getUpPhase = m_getUpPhase;
    m_telemetry.getUpProgress = m_getUpPhase
            == CharacterGetUpPhase3D::Rising
        ? std::clamp(m_getUpClipSeconds / std::max(0.05f,
            standUpClip ? standUpClip->durationSeconds : 1.6f), 0.0f, 1.0f) : 0.0f;
    m_telemetry.getUpAttempt = m_getUpAttempt;
    m_telemetry.fallReflex = m_fallReflex;
    m_telemetry.fallCommit = m_fallCommit;
    m_telemetry.getUpRate = m_getUpPhase == CharacterGetUpPhase3D::Rising
        ? m_getUpRate : 0.0f;
    m_telemetry.getUpStruggle = m_getUpStruggle;
    m_telemetry.muscleTone = m_getUpPhase == CharacterGetUpPhase3D::Settling
        ? m_settleTone : 1.0f;
    m_telemetry.getUpAssistRamp = m_getUpPhase == CharacterGetUpPhase3D::Rising
        ? m_getUpAssistRamp : 0.0f;
    m_telemetry.settleGather = m_getUpPhase == CharacterGetUpPhase3D::Settling
        ? m_settleGather : m_getUpPhase == CharacterGetUpPhase3D::Rising ? 1.0f : 0.0f;
    const bool bodilyTelemetry = m_bodilyRising && m_getUpPhase == CharacterGetUpPhase3D::Rising;
    m_telemetry.bodilyStage = bodilyTelemetry ? m_bodilyGetUp.telemetry().stage : BodilyGetUpStage3D::None;
    m_telemetry.bodilyStageProgress = bodilyTelemetry ? m_bodilyGetUp.telemetry().stageProgress : 0.0f;
    m_telemetry.bodilyRootError = bodilyTelemetry ? m_bodilyGetUp.telemetry().rootError : 0.0f;
}

// Para a locomocao padrao (DefaultLocomotion3D, 04/10): o corpo do chute e a
// "vida" do modo fisico sobre as mesmas coordenadas de junta.
void characterSolveFootPose3D(const RagdollProfile3D& profile, std::vector<Vec3>& coordinates,
    std::size_t footIndex, Vec3 targetPosition, Quaternion targetOrientation, float weight, bool holdYaw) {
    solveFootPose(profile, coordinates, footIndex, targetPosition, targetOrientation, weight, holdYaw);
}

void characterKickPose3D(const RagdollProfile3D& profile, const RagdollState3D& state,
    std::vector<Vec3>& coordinates, std::array<std::size_t, 2> hands, const BallKickPose3D& kick) {
    applyKickPose(profile, state, coordinates, hands, kick);
}

void characterGaitVariety3D(const RagdollProfile3D& profile, const AnimationClip3D& clip, float cyclePhase,
    float armPhase, float strides, std::uint32_t seed, float amount, std::vector<Vec3>& coordinates) {
    // (Locomocao padrao: somada, nas fases do clipe - substituindo, o tronco e
    // os bracos saltavam inteiros para a pose de outro clipe quando o
    // dominante mudava.)
    applyGaitVariety(profile, clip, cyclePhase, armPhase, strides, seed, amount, coordinates, true, true);
}

void characterIdleVariety3D(const RagdollProfile3D& profile, float seconds, std::uint32_t seed, float amount,
    std::vector<Vec3>& coordinates) {
    applyIdleVariety(profile, seconds, seed, amount, coordinates);
}

} // namespace MatterEngine
