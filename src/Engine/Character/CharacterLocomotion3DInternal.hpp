#pragma once

// Os ajudantes internos do controle fisico (a reescrita: o update em blocos, cada um num
// arquivo CharacterLocomotion3D_<bloco>.cpp): as contas e as constantes que antes moravam no
// namespace anonimo de CharacterLocomotion3D.cpp. So os arquivos CharacterLocomotion3D*.cpp incluem.

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

inline float gaitSectorCenter(CharacterGaitDirection3D direction) {
    switch (direction) {
    case CharacterGaitDirection3D::Left: return Pi * 0.5f;
    case CharacterGaitDirection3D::Right: return -Pi * 0.5f;
    case CharacterGaitDirection3D::Backward: return Pi;
    default: return 0.0f;
    }
}

// Quanto a direcao (em relacao ao olhar) passa do setor; zero dentro dele.
inline float gaitSectorOverrun(CharacterGaitDirection3D direction,
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

inline bool finite(Vec3 value) {
    return std::isfinite(value.x) && std::isfinite(value.y)
        && std::isfinite(value.z);
}

inline float component(Vec3 value, std::size_t axis) {
    return axis == 0 ? value.x : axis == 1 ? value.y : value.z;
}

inline void setComponent(Vec3& value, std::size_t axis, float componentValue) {
    if (axis == 0) value.x = componentValue;
    else if (axis == 1) value.y = componentValue;
    else value.z = componentValue;
}

inline float smoothStep(float value) {
    value = std::clamp(value, 0.0f, 1.0f);
    return value * value * (3.0f - 2.0f * value);
}

inline float wrapAngle(float radians) {
    while (radians > Pi) radians -= TwoPi;
    while (radians < -Pi) radians += TwoPi;
    return radians;
}

inline std::size_t findLink(const RagdollProfile3D& profile, std::string_view id) {
    for (std::size_t index = 0; index < profile.links.size(); ++index) {
        if (profile.links[index].id == id) return index;
    }
    return profile.links.size();
}


inline Vec3 uprightOrientationError(Quaternion current, float desiredYaw,
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

inline Quaternion exponentialRotation(Vec3 coordinates) {
    return ragdollJointRotation3D(coordinates);
}

inline Vec3 exponentialJacobian(Vec3 coordinates, Vec3 velocity) {
    return ragdollJointJacobian3D(coordinates, velocity);
}

inline Vec3 jointAngularVelocity(Vec3 target, Vec3 coordinateVelocity,
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
inline Vec3 pendulumArm(const RagdollProfile3D& profile, std::size_t leftFoot, std::size_t rightFoot, Position position) {
    Vec3 center {};
    float mass = 0.0f;
    for (std::size_t i = 0; i < profile.links.size(); ++i) {
        center = center + position(i) * profile.links[i].massFraction;
        mass += profile.links[i].massFraction;
    }
    center = center * (1.0f / std::max(1e-4f, mass));
    return center - (position(leftFoot) + position(rightFoot)) * 0.5f;
}

inline void buildLocalPose(const RagdollProfile3D& profile,
    const std::vector<Vec3>& coordinates, RagdollAnimationPose3D& pose) {
    buildRagdollLocalPose3D(profile, coordinates, pose);
}

// Position-only damped least squares: brings a point fixed to endIndex
// (pointInEnd, in that link's frame) to a target, moving the joints of
// endIndex and its ancestors up to `depth` links, within their limits.
inline void solveChainPoint(const RagdollProfile3D& profile,
    std::vector<Vec3>& coordinates, std::size_t footIndex, Vec3 pointInEnd,
    int depth, Vec3 targetRootLocal, float weight, int iterations = 7) {
    solveRagdollChainPoint3D(profile, coordinates, footIndex, pointInEnd,
        depth, targetRootLocal, weight, iterations);
}

// Keeps the foot link on a support point (stance lock, turn steps).
inline void solveFootPosition(const RagdollProfile3D& profile,
    std::vector<Vec3>& coordinates, std::size_t footIndex,
    Vec3 targetRootLocal, float weight) {
    solveChainPoint(profile, coordinates, footIndex, {}, 3, targetRootLocal,
        weight);
}

inline Quaternion rotationBetween(Vec3 from, Vec3 to) {
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

inline Quaternion slerpQuaternion(Quaternion from, Quaternion to, float t) {
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
inline Vec3 rotationVector(Quaternion value);

inline void solveFootPose(const RagdollProfile3D& profile,
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
    // (A copia so serve para a mistura com peso parcial, no fim.)
    std::vector<Vec3> original;
    if (weight < 1.0f) original = coordinates;
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
        // (So a cadeia ate o pe: le a pelve, a coxa e o pe.)
        buildRagdollChainPose3D(profile, coordinates, pose, footIndex);
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

    // Tornozelo: Gauss-Newton nos eixos habilitados do pe (so le a canela).
    buildRagdollChainPose3D(profile, coordinates, pose, shinIndex);
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

inline bool validClip(const AnimationClip3D* clip,
    const RagdollProfile3D& profile) {
    return clip != nullptr && clip->targetRigId == profile.id
        && std::isfinite(clip->durationSeconds)
        && clip->durationSeconds > 0.0f
        && validateAnimationClipForRagdoll3D(*clip, profile).empty();
}

inline float authoredHorizontalSpeed(const AnimationClip3D& clip) {
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
inline float clipRootYaw(const AnimationClip3D& clip, const RagdollProfile3D& profile, float time) {
    const AnimationTrack3D* root = findAnimationTrack3D(clip, profile.links.front().id);
    if (root == nullptr) return 0.0f;
    const Quaternion q = sampleAnimationTrack3D(*root, std::clamp(time, 0.0f, clip.durationSeconds),
        clip.durationSeconds, false).rotationDelta;
    return 2.0f * std::atan2(q.z, q.w);
}

inline Vec3 clipRootPlanar(const AnimationClip3D& clip, const RagdollProfile3D& profile, float time) {
    const AnimationTrack3D* root = findAnimationTrack3D(clip, profile.links.front().id);
    if (root == nullptr) return {};
    const Vec3 t = sampleAnimationTrack3D(*root, std::clamp(time, 0.0f, clip.durationSeconds),
        clip.durationSeconds, false).translationOffsetMeters;
    return { t.x, t.y, 0.0f };
}

inline float stanceAlignedPhase(const AnimationClip3D& clip, float cyclePhase) {
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

inline float footSoleOffset(const RagdollLinkDefinition3D& link) {
    return ragdollColliderHasBox3D(link.collider)
        ? link.collider.boxHalfExtents.z
        : link.collider.radiusMeters;
}

// How far the collider's lowest point sits below its centre, for the
// orientation it is actually in. Treating a foot as a flat slab of
// halfExtents.z underestimates this the moment the ankle pitches - a 12
// degree pitch on a 30 cm foot puts the toe ~3 cm lower than that - and the
// grounding below then leaves the toe buried, which the floor answers with
// an enormous contact force.
inline float lowestColliderOffset(const RagdollLinkDefinition3D& link,
    Quaternion colliderWorldOrientation) {
    const Vec3 up = colliderWorldOrientation.conjugate().rotate(
        { 0.0f, 0.0f, 1.0f });
    // A casca (a chuteira): o vertice que mais desce.
    if (link.collider.shape == RagdollColliderShape3D::ConvexHull) {
        float lowest = 0.0f;
        for (const Vec3& point : link.collider.hullPoints) lowest = std::max(lowest, -dot(up, point));
        return lowest;
    }
    if (ragdollColliderHasBox3D(link.collider)) {
        const Vec3 half = link.collider.boxHalfExtents;
        return std::abs(up.x) * half.x + std::abs(up.y) * half.y
            + std::abs(up.z) * half.z;
    }
    // Capsule: the cylinder runs along local X, capped by the radius.
    return std::abs(up.x) * std::max(0.0f,
        link.collider.lengthMeters * 0.5f - link.collider.radiusMeters)
        + link.collider.radiusMeters;
}

inline Vec3 rotationVector(Quaternion value) {
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
inline AnimationTransformSample3D clipRootAt(const RagdollProfile3D& profile,
    const AnimationClip3D& clip, float time) {
    AnimationTransformSample3D sample;
    if (const AnimationTrack3D* track = findAnimationTrack3D(clip, profile.links.front().id)) {
        sample = sampleAnimationTrack3D(*track, std::clamp(time, 0.0f, clip.durationSeconds),
            clip.durationSeconds, false);
    }
    return sample;
}

inline Vec3 clampMagnitude(Vec3 value, float maximumLength) {
    const float squared = value.lengthSquared();
    if (squared <= maximumLength * maximumLength || squared < 0.000001f) {
        return value;
    }
    return value * (maximumLength / std::sqrt(squared));
}

inline void addJointCoordinate(const RagdollProfile3D& profile,
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
inline void blendJointCoordinate(const RagdollProfile3D& profile,
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

inline float varietyNoise(std::uint32_t seed, std::uint32_t channel, float position);

// Reflexos de quem esta caindo, so alvos de junta (os motores fazem o
// resto; nada e escrito no corpo): as maos vao ao chao no rumo da queda, a
// cabeca se protege (queixo no peito caindo de costas, erguida caindo de
// frente, inclinada para fora de lado), a coluna enrola um pouco caindo de
// costas, joelhos e quadris cedem. `amount` mistura com a pose de antes.
inline void applyFallReflexPose(const RagdollProfile3D& profile,
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
inline void applyFetalPose(const RagdollProfile3D& profile,
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
inline void applyAirBalance(const RagdollProfile3D& profile,
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

inline void setJointCoordinate(const RagdollProfile3D& profile,
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
inline void applyBalanceReaction(const RagdollProfile3D& profile,
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

// "Vida" no ragdoll, no ar: caindo sem tocar nada, bracos em moinho fora de
// fase e pernas pedalando - quem perdeu o chao tenta se equilibrar, nao fica
// estatico. So alvos de junta; o corpo faz com os motores. (Deitado nao ha
// movimento proprio: os "surtos" de tentar se recuperar pareciam um AVC, a
// pedido do usuario ficou so o ragdoll com um pouco de tonus.)
inline void applyAirborneFlail(const RagdollProfile3D& profile,
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
inline void applyFlightPose(const RagdollProfile3D& profile,
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
inline float varietyValue(std::uint32_t seed, std::uint32_t channel, std::int64_t lattice) {
    std::uint64_t h = (static_cast<std::uint64_t>(seed) << 32 | channel)
        ^ static_cast<std::uint64_t>(lattice) * 0x9E3779B97F4A7C15ull;
    h ^= h >> 30; h *= 0xBF58476D1CE4E5B9ull;
    h ^= h >> 27; h *= 0x94D049BB133111EBull;
    h ^= h >> 31;
    return static_cast<float>(h >> 40) / 8388607.5f - 1.0f;
}

inline float varietyNoise(std::uint32_t seed, std::uint32_t channel, float position) {
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
inline void applyGaitVariety(const RagdollProfile3D& profile,
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
inline void applyIdleVariety(const RagdollProfile3D& profile, float seconds,
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
} // namespace MatterEngine
