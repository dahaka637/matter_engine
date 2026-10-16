# Inventario das constantes de ajuste da locomocao (TEMPTUNE)

> Gerado por `tools/analysis/temptune_inventory.py` - nao editar a mao.

A reescrita da locomocao (vault `04_locomocao/reescrita_da_locomocao.md`, R0) troca estas constantes - lidas do ambiente uma vez e globais para todos os atletas - por parametros tipados por modulo, por atleta e normalizados pelo corpo. Este e o mapa (e o espaco de parametros da calibracao e do RL).

- **437 nomes** em **440 leituras**.
- `src/Engine/Character/CharacterLocomotion3D.cpp`: 384 nomes.
- `src/Engine/Character/DefaultLocomotion3D.cpp`: 4 nomes.
- `src/Engine/Control/AdaptivePhysicalCharacter3D.cpp`: 42 nomes.
- `src/Engine/Locomotion/ContactFootwork3D.cpp`: 7 nomes.

## Os grupos (pelo prefixo do nome)

| Grupo | Nomes |
|---|---|
| Get | 50 |
| Stumble | 24 |
| Tackle | 18 |
| Land | 17 |
| Curve | 16 |
| Start | 14 |
| Gait | 14 |
| Contact | 13 |
| Stance | 12 |
| Posture | 11 |
| Stagger | 11 |
| Nav | 11 |
| Arm | 10 |
| Swing | 10 |
| Fall | 8 |
| Run | 8 |
| Cap | 8 |
| Low | 8 |
| Reach | 7 |
| Foot | 7 |
| Clip | 6 |
| Kick | 6 |
| Push | 6 |
| Leg | 6 |
| Lift | 5 |
| Sprint | 5 |
| Bal | 5 |
| Brace | 5 |
| Path | 4 |
| Touch | 4 |
| Spine | 4 |
| Hip | 4 |
| Lock | 4 |
| Lean | 4 |
| Voadora | 4 |
| Pelvis | 3 |
| Rate | 3 |
| Thrust | 3 |
| Pivot | 3 |
| Fetal | 3 |
| Air | 3 |
| Ground | 3 |
| Turn | 3 |
| Jump | 3 |
| Knee | 3 |
| Hit | 3 |
| Assist | 3 |
| Side | 3 |
| Req | 2 |
| Windmill | 2 |
| Cut | 2 |
| Brake | 2 |
| Elbow | 2 |
| Boost | 2 |
| Planar | 2 |
| Goal | 1 |
| Ball | 1 |
| Sole | 1 |
| Variety | 1 |
| Clear | 1 |
| Body | 1 |
| Footing | 1 |
| Rev | 1 |
| Tone | 1 |
| Pedal | 1 |
| Neck | 1 |
| Head | 1 |
| Chest | 1 |
| Trip | 1 |
| Sector | 1 |
| Strafe | 1 |
| First | 1 |
| Cross | 1 |
| Planner | 1 |
| Absorb | 1 |
| Catch | 1 |
| Far | 1 |
| Rest | 1 |
| Settle | 1 |
| Retry | 1 |
| Reflex | 1 |
| Impact | 1 |
| Upright | 1 |
| Walk | 1 |
| Weight | 1 |
| Flight | 1 |
| Vert | 1 |
| Unreach | 1 |
| Recover | 1 |
| Narrow | 1 |

## `src/Engine/Character/CharacterLocomotion3D.cpp`

| Nome | Valor | Variavel | Linha | Funcao | O que faz (comentario) |
|---|---|---|---|---|---|
| `T_PelvisLead` | 0.25f | PelvisCurveLeadSeconds | 19 | arquivo |  |
| `T_PelvisLeadMax` | 0.35f | PelvisCurveLeadMaximum | 20 | arquivo |  |
| `T_PelvisFF` | 1.0f | PelvisCurveFeedForward | 21 | arquivo |  |
| `T_PathLeadSlow` | 1.2f | PathLeadSlow | 24 | arquivo | Curva fechada: correndo para a frente, a pelve segue o caminho do corpo e so se adianta para o pedido ate este... |
| `T_GetUpAssist` | 1.0f | GetUpAssistScale | 27 | arquivo | Levantar: escala das ajudas externas (pelve, peito, endireitar) e teto da forca dos musculos (multiplo da nomi... |
| `T_GetUpChestCap` | 5.0f | GetUpChestLateCap | 37 | arquivo | Levantar mais rapido (pedido do usuario, 05/10 noite): o clipe anda ate GetUpSpeed vezes o ritmo dele (o corpo... |
| `T_GetUpLateHeight` | 0.75f | GetUpLateHeight | 38 | arquivo |  |
| `T_GetUpStraighten` | 1.0f | GetUpEndStraighten | 41 | arquivo | Fim do levantar: o quanto a coluna endireita (0..1) e a parte da flexao do clipe que fica. |
| `T_GetUpPelvisCap` | 6.0f | GetUpPelvisLateCap | 43 | arquivo | E o teto (vezes a massa) a mais na ajuda que endireita a pelve ali. |
| `T_GetUpPelvisGain` | 2.0f | GetUpPelvisLateGain | 44 | arquivo |  |
| `T_GetUpLiftBrake` | 1.0f | GetUpLiftBrake | 46 | arquivo | Perto da altura de pe, a ajuda de subida tambem freia (fracao do teto). |
| `T_GetUpSpineShare` | 0.25f | GetUpEndSpineShare | 47 | arquivo |  |
| `T_GetUpChestGain` | 1.5f | GetUpChestLateGain | 48 | arquivo |  |
| `T_GetUpHurryOn` | 1.0f | GetUpHurryEnabled | 49 | arquivo |  |
| `T_TackleChoice` | 1.0f | TackleChoiceByView | 51 | arquivo | Carrinho fisico: o clipe pela camera/corrida e o mesmo deslize (05/10). |
| `T_GetUpSpeed` | 1.0f | GetUpSpeed | 52 | arquivo |  |
| `T_GetUpHurry` | 3.5f | GetUpHurrySpeed | 53 | arquivo |  |
| `T_GetUpSprint` | 3.5f | GetUpSprintSpeed | 54 | arquivo |  |
| `T_GetUpSettle` | 0.55f | GetUpSettleSeconds | 55 | arquivo |  |
| `T_GetUpCalm` | 1.6f | GetUpCalmScale | 58 | arquivo | Deitado: o quanto o corpo precisa estar parado para comecar (vezes os limites de velocidade do centro e do mem... |
| `T_GetUpMaxRise` | 0.9f | GetUpMaxRise | 60 | arquivo | A subida da pelve no clipe acelerado fica no maximo nisto (m/s). |
| `T_GetUpMaxLeg` | 5.0f | GetUpMaxLegSpeed | 62 | arquivo | E as juntas das pernas no clipe acelerado, no maximo nisto (rad/s). |
| `T_GetUpHurrySettle` | 0.25f | GetUpHurrySettleSeconds | 63 | arquivo |  |
| `T_GetUpHurryExit` | 0.85f | GetUpHurryExit | 66 | arquivo | Saindo com pressa: o clipe termina a partir desta fracao, com o corpo quase em pe (pelve endireitada alem de G... |
| `T_GetUpHurryUpright` | 0.85f | GetUpHurryUpright | 67 | arquivo |  |
| `T_GetUpHandoffProg` | 0.35f | GetUpHandoffProgress | 71 | arquivo | Com pressa e o modo padrao (laboratorio): a partir de quando o corpo esta sobre os pes (fracao do clipe, altur... |
| `T_GetUpHandoffHeight` | 0.50f | GetUpHandoffHeight | 72 | arquivo |  |
| `T_GetUpHandoffTilt` | 1.05f | GetUpHandoffTilt | 73 | arquivo |  |
| `T_GetUpHandoffChest` | 1.66f | GetUpHandoffChestTilt | 74 | arquivo |  |
| `T_GetUpHandoffFeet` | 0.70f | GetUpHandoffFeet | 77 | arquivo | (Os pes a no maximo isto a frente/atras da pelve, m: de bruços o levantar passa por um afundo, um pe bem a fre... |
| `T_GetUpTorqueCap` | 3.0f | GetUpTorqueCap | 78 | arquivo |  |
| `T_Tackle` | 1.0f | TackleEnabled | 86 | arquivo | Carrinho (04/10). TEMPTUNE: liga; velocidade minima na direcao do olhar (m/s); crossfade da corrida para o cli... |
| `T_TackleMin` | 2.5f | TackleMinSpeed | 87 | arquivo |  |
| `T_TackleBlend` | 0.12f | TackleTransition | 88 | arquivo |  |
| `T_TackleGather` | 0.08f | TackleGatherDrop | 89 | arquivo |  |
| `T_TacklePlantStride` | 0.10f | TacklePlantStride | 92 | arquivo | Preparo: o passo de pouso um pouco mais longo (firmar o pe a frente) e quanto do tronco, bracos e cabeca ja va... |
| `T_GoalHandoff` | 0.12f | GoalHandoffSeconds | 95 | arquivo | TEMPTUNE: troca de dono do pe no ar (planejador <-> passada): o alvo novo entra misturado a partir do ultimo n... |
| `T_BallRepelMargin` | 0.025f | BallRepelMargin | 97 | arquivo | Repulsao do pe pela bola: folga entre a caixa do pe e a bola (m). |
| `T_ClipTurn` | 0.0f | ClipTurnEnabled | 102 | arquivo | TEMPTUNE: giro parado pelo clipe (ajuda cheia; pedido do usuario, 04/10): liga com o giro pedido acima de Clip... |
| `T_ClipTurnEnter` | 0.30f | ClipTurnEnter | 103 | arquivo |  |
| `T_ClipTurnSpeed` | 6.0f | ClipTurnSpeed | 104 | arquivo |  |
| `T_ClipTurnMin` | 0.24f | ClipTurnMinSeconds | 105 | arquivo |  |
| `T_ClipTurnBlendIn` | 0.03f | ClipTurnBlendIn | 106 | arquivo |  |
| `T_ClipTurnBlendOut` | 0.12f | ClipTurnBlendOut | 107 | arquivo |  |
| `T_KickTwist` | 0.30f | KickTrunkTwist | 110 | arquivo | TEMPTUNE: corpo do chute (rad no chute cheio): giro do tronco, para tras no armado, por cima da bola batendo, ... |
| `T_KickTrunkBack` | 0.12f | KickTrunkBack | 111 | arquivo |  |
| `T_KickTrunkOver` | 0.12f | KickTrunkOver | 112 | arquivo |  |
| `T_KickTrunkSide` | 0.14f | KickTrunkSide | 113 | arquivo |  |
| `T_KickArmFloor` | 0.25f | KickArmFloor | 116 | arquivo | Bracos do chute pela forca (04/10): abaixo de KickArmFloor nao entram, em KickArmFull o desenho inteiro. |
| `T_KickArmFull` | 0.95f | KickArmFull | 117 | arquivo |  |
| `T_TacklePreBlend` | 0.6f | TacklePreBlend | 118 | arquivo |  |
| `T_TacklePrep` | 0.40f | TacklePrepMax | 119 | arquivo |  |
| `T_TackleSlideA` | 0.30f | TackleSlideA | 124 | arquivo | Forca do carrinho pela velocidade: no chao ele desliza D = a * v^p (v a velocidade ao tocar o chao; trote ~1,1... |
| `T_TackleSlideP` | 1.3f | TackleSlideP | 125 | arquivo |  |
| `T_TackleBaseMu` | 0.93f | TackleBaseFriction | 126 | arquivo |  |
| `T_TackleImpulse` | 1.15f | TackleImpulseShare | 130 | arquivo | Impulso: na saida para o mergulho, com o pe de apoio no chao, a perna empurra o corpo na direcao do carrinho a... |
| `T_TackleImpulseForce` | 0.8f | TackleImpulseForce | 131 | arquivo |  |
| `T_TackleSlideH` | 0.55f | TackleSlideHeight | 132 | arquivo |  |
| `T_TackleSlideV` | 2.2f | TackleSlideRelease | 133 | arquivo |  |
| `T_TackleHandoff` | 0.74f | TackleHandoffHeight | 134 | arquivo |  |
| `T_TackleAttitude` | 1.5f | TackleAttitude | 141 | arquivo | Atitude da pelve no carrinho (torque para a inclinacao do clipe; 0 sem). 1,5 desde o jogador montado (09/10): ... |
| `T_TackleRecovery` | 1.0f | TackleRecoveryAssist | 143 | arquivo | Ajuda para levantar do deslize (altura, para cima; propria do carrinho). |
| `T_GetUpRoll` | 1.0f | GetUpRollEnabled | 146 | arquivo | Levantar (04/10): rolar para ficar de bruços antes de levantar, e os pes do equilibrio no levantar de bruços. ... |
| `T_GetUpRollBack` | 0.0f | GetUpRollFromBack | 147 | arquivo |  |
| `T_GetUpRollSide` | 0.55f | GetUpRollSideFacing | 149 | arquivo | De lado: o "para a frente" do peito a menos disto da horizontal (/z/). |
| `T_GetUpRollTorque` | 2.5f | GetUpRollTorque | 151 | arquivo | Ajuda do rolar (N.m por kg, vezes o slider da ajuda para levantar). |
| `T_GetUpRollTime` | 3.0f | GetUpRollTimeout | 152 | arquivo |  |
| `T_GetUpFeet` | 0.0f | GetUpFootworkEnabled | 153 | arquivo |  |
| `T_GetUpFeetFrom` | 0.36f | GetUpFootworkFrom | 156 | arquivo | Progresso do clipe de bruços: pes do equilibrio (quatro apoios) e o ponto em que as maos soltam (o clipe esper... |
| `T_GetUpHandRelease` | 0.46f | GetUpHandRelease | 157 | arquivo |  |
| `T_GetUpToe` | 0.18f | GetUpToeAhead | 159 | arquivo | Do tornozelo a ponta do pe (0,22 m no modelo), com folga. |
| `T_GetUpStepMargin` | 0.04f | GetUpStepMargin | 160 | arquivo |  |
| `T_GetUpStepAhead` | -0.06f | GetUpStepAhead | 162 | arquivo | O tornozelo pousa atras do centro de massa (o pe fica embaixo dele). |
| `T_GetUpStepWidth` | 0.11f | GetUpStepWidth | 163 | arquivo |  |
| `T_GetUpStepSec` | 0.35f | GetUpStepSeconds | 164 | arquivo |  |
| `T_GetUpStepLift` | 0.10f | GetUpStepLift | 165 | arquivo |  |
| `T_GetUpStepMax` | 0.45f | GetUpStepMax | 166 | arquivo |  |
| `T_GetUpHoldMax` | 3.0f | GetUpHoldMax | 167 | arquivo |  |
| `T_GetUpLead` | 0.12f | GetUpRootLead | 170 | arquivo | A pelve do IK das pernas presas se adianta para a do clipe ate isto (altura, m; giro, rad): o descompasso e o ... |
| `T_GetUpTurn` | 0.12f | GetUpTurnLead | 171 | arquivo |  |
| `T_GetUpHands` | 0.0f | GetUpHandsEnabled | 174 | arquivo | Mao de apoio: o clipe pede a mao no chao (punho abaixo disto do chao); o ponto e a frente e para fora do ombro... |
| `T_GetUpHandDown` | 0.10f | GetUpHandClipDown | 175 | arquivo |  |
| `T_GetUpHandFwd` | 0.12f | GetUpHandForward | 176 | arquivo |  |
| `T_GetUpHandOut` | 0.08f | GetUpHandOutward | 177 | arquivo |  |
| `T_GetUpHandReach` | 0.30f | GetUpHandReachSeconds | 179 | arquivo |  |
| `T_GetUpHandRelease` | 0.25f | GetUpHandReleaseSeconds | 180 | arquivo |  |
| `T_GetUpBodily` | 0.0f | GetUpBodily | 182 | arquivo | Levantar pelo corpo (BodilyGetUp3D) no lugar do clipe com ajuda. |
| `T_LiftIdle` | 0.0f | PostureLiftIdle | 185 | arquivo | Postura mais ereta: o quanto a pelve-alvo sobe (m) parado, trotando e no sprint - com os pes travados no chao,... |
| `T_LiftRun` | 0.0f | PostureLiftRun | 186 | arquivo |  |
| `T_LiftSprint` | 0.0f | PostureLiftSprint | 187 | arquivo |  |
| `T_StanceStiff` | 0.55f | StanceLegStiffness | 188 | arquivo |  |
| `T_ReachComfort` | 0.985f | ReachComfort | 189 | arquivo |  |
| `T_ContactPhase` | 1.0f | ContactPhase | 190 | arquivo |  |
| `T_ContactWait` | 0.08f | ContactWaitSeconds | 191 | arquivo |  |
| `T_ContactTrail` | 0.55f | ContactTrailCap | 192 | arquivo |  |
| `T_ContactLead` | 0.25f | ContactMaxLead | 193 | arquivo |  |
| `T_ContactCatchUp` | 0.5f | ContactCatchUp | 194 | arquivo |  |
| `T_ContactEarly` | 0.8f | ContactEarlyProgress | 195 | arquivo |  |
| `T_ContactEarlyBoost` | 1.6f | ContactEarlyBoost | 196 | arquivo |  |
| `T_ContactReach` | 1.0f | ContactLandReach | 197 | arquivo |  |
| `T_ContactPaw` | 0.0f | ContactPaw | 198 | arquivo |  |
| `T_RunLeap` | 1.0f | RunLeap | 200 | arquivo | v2: salto correndo pela passada aberta do ciclo (ver a decolagem). |
| `T_RunLeapSpeed` | 4.0f | RunLeapMinimumSpeed | 201 | arquivo |  |
| `T_RunLeapAssist` | 0.45f | RunLeapMinimumAssist | 202 | arquivo |  |
| `T_RunLeapAssistMax` | 0.85f | RunLeapMaximumAssist | 203 | arquivo |  |
| `T_RunLeapHold` | 0.3f | RunLeapHold | 204 | arquivo |  |
| `T_ContactLand` | 0.0f | ContactLandOffset | 205 | arquivo |  |
| `T_RateSmooth` | 1.0f | ContactRateSmooth | 206 | arquivo |  |
| `T_RateDown` | 0.015f | ContactRateDownSeconds | 207 | arquivo |  |
| `T_RateUp` | 0.06f | ContactRateUpSeconds | 208 | arquivo |  |
| `T_ArmPhase` | 0.08f | ArmPhaseSmoothSeconds | 209 | arquivo |  |
| `T_ArmLock` | 0.25f | ArmPhaseLockSeconds | 210 | arquivo |  |
| `T_ContactPawMax` | 0.35f | ContactPawMaximum | 211 | arquivo |  |
| `T_ContactReachRelease` | 0.12f | ContactLandRelease | 212 | arquivo |  |
| `T_ContactReachMax` | 0.02f | ContactLandReachMax | 213 | arquivo |  |
| `T_TouchHold` | 0.12f | TouchHoldSeconds | 214 | arquivo |  |
| `T_TouchMinStance` | 0.08f | TouchMinStanceSeconds | 215 | arquivo |  |
| `T_TouchSteady` | 1.0f | TouchHoldSteadyOnly | 216 | arquivo |  |
| `T_SoleRelease` | 1.0f | ReleaseBySoleClearance | 217 | arquivo |  |
| `T_PushStiff` | 2.0f | PushStiffness | 218 | arquivo |  |
| `T_PushLand` | 1.0f | PushAfterLandingSeconds | 219 | arquivo |  |
| `T_LandTone` | 1.0f | LandingLegTone | 220 | arquivo |  |
| `T_LandRunYield` | 1.0f | LandRunYield | 221 | arquivo |  |
| `T_LandRunDepth` | 0.5f | LandRunDepth | 222 | arquivo |  |
| `T_LandToneSec` | 0.6f | LandingLegToneSeconds | 223 | arquivo |  |
| `T_Thrust` | 0.0f | ThrustGain | 224 | arquivo |  |
| `T_ThrustMax` | 0.15f | ThrustMaximum | 225 | arquivo |  |
| `T_ThrustMargin` | 0.0f | ThrustApexMargin | 226 | arquivo |  |
| `T_StanceDamp` | 0.1f | StanceVerticalDamping | 227 | arquivo |  |
| `T_StanceDampFree` | 1.0f | StanceDampFree | 228 | arquivo |  |
| `T_StanceDampCurve` | 0.2f | StanceDampCurveSpanEnv | 229 | arquivo |  |
| `T_StanceBand` | 0.0f | StanceRootBandEnv | 230 | arquivo |  |
| `T_TouchPlantCurve` | 0.0f | TouchPlantInCurves | 231 | arquivo |  |
| `T_CurveNarrow` | 0.0f | CurveNarrow | 232 | arquivo |  |
| `T_CurveSpine` | 0.45f | CurveSpineShare | 233 | arquivo |  |
| `T_VarietyAdd` | 0.0f | VarietyAdditive | 234 | arquivo |  |
| `T_CurveSpineSign` | 1.0f | CurveSpineSign | 235 | arquivo |  |
| `T_PushBehind` | 0.05f | PushBehindMeters | 236 | arquivo |  |
| `T_PushVel` | 0.0f | PushVelocity | 237 | arquivo |  |
| `T_PushRamp` | 1.0f | PushRamp | 238 | arquivo |  |
| `T_ReachAnticipate` | 1.0f | ReachAnticipateScale | 239 | arquivo |  |
| `T_ReachOmega` | 18.0f | ReachPelvisOmega | 240 | arquivo |  |
| `T_SwingReachCap` | -1.0f | SwingReachFlatCap | 241 | arquivo |  |
| `T_PathLeadFast` | 0.45f | PathLeadFast | 242 | arquivo |  |
| `T_PathLeadFastSpeed` | 6.5f | PathLeadFastSpeed | 243 | arquivo |  |
| `T_CurveSpeedAc` | 10.0f | CurveSpeedAcceleration | 245 | arquivo | Curva fechada: teto de velocidade a(lateral)/taxa de giro do pedido. |
| `T_CurveSpeedAcLow` | 7.0f | CurveSpeedAccelerationLow | 246 | arquivo |  |
| `T_PathLeadAfter` | 1.0f | PathLeadAfterCurve | 248 | arquivo | A pelve limitada ao caminho depois do adiantamento da curva (o total). |
| `T_CurveSpeedMin` | 2.5f | CurveSpeedMinimum | 249 | arquivo |  |
| `T_CurveSpeedRec` | 6.0f | CurveSpeedRecovery | 250 | arquivo |  |
| `T_ReqTurnFilter` | 0.25f | RequestTurnFilterSeconds | 251 | arquivo |  |
| `T_ReqTurnClamp` | 6.0f | RequestTurnRateClamp | 252 | arquivo |  |
| `T_StanceHoldYaw` | 0.0f | GaitStanceHoldYaw | 255 | arquivo | Correndo, o pe apoiado segura o rumo do pouso (a pelve gira por cima dele pelo quadril) em vez de girar com el... |
| `T_StartRear` | 0.0f | StartFromRearFoot | 260 | arquivo | Quase-queda em movimento: deteccao (inclinacao para a frente alem da de regime, subindo; freada do corpo alem ... |
| `T_SwingLow` | 0.7f | SwingFromLowBody | 263 | arquivo | Perna da passada no ar pela altura da pelve de verdade quando ela esta abaixo da de referencia (fracao; 0 desl... |
| `T_SwingLowRelease` | 0.15f | SwingFromLowBodyRelease | 264 | arquivo |  |
| `T_SwingTilt` | 0.0f | SwingFromBodyTilt | 266 | arquivo | ... e pela inclinacao de verdade da pelve (so a inclinacao, sem o rumo). |
| `T_SwingLowLand` | 0.65f | SwingLowLandingStart | 267 | arquivo |  |
| `T_SwingLowLandSpan` | 0.3f | SwingLowLandingSpan | 268 | arquivo |  |
| `T_LiftUnload` | 0.0f | LiftAfterUnloadSeconds | 271 | arquivo | O pe da passada que o clipe solta ainda carregando o corpo (o outro sem apoio) espera descarregar, ate este te... |
| `T_LiftUnloadShare` | 0.25f | LiftAfterUnloadShare | 272 | arquivo |  |
| `T_SprintClear` | 2.0f | SprintCurveClearStart | 275 | arquivo | Folga minima e ponta recolhida do trote tambem no sprint em curva (pela aceleracao lateral real do corpo, m/s2... |
| `T_SprintClearSpan` | 4.0f | SprintCurveClearSpan | 276 | arquivo |  |
| `T_ClearEarly` | 0.14f | CurveClearEarly | 277 | arquivo |  |
| `T_StartLead` | 0.0f | StartPhaseLead | 278 | arquivo |  |
| `T_StartHold` | 0.0f | StartLoadHoldSeconds | 281 | arquivo | Arrancada: o pe que o ciclo ja quer no ar, mas que ainda carrega o corpo, so sai quando descarrega (ate este t... |
| `T_StartHoldShare` | 0.30f | StartLoadHoldShare | 282 | arquivo |  |
| `T_Stumble` | 1.0f | StumbleDetection | 283 | arquivo |  |
| `T_StumbleRef` | 1.0f | StumbleAgainstReference | 284 | arquivo |  |
| `T_StumbleFull` | 0.0f | StumbleFullResponse | 285 | arquivo |  |
| `T_StumbleStep` | 0.0f | StumbleStepTrigger | 289 | arquivo | Passo de quase-queda pelo planejador: risco que dispara, quanto a frente do corpo no toque o pe pousa (m), dur... |
| `T_StumbleAhead` | 0.20f | StumbleStepAhead | 290 | arquivo |  |
| `T_StumbleStepSwing` | 0.22f | StumbleStepSwing | 291 | arquivo |  |
| `T_StumbleStepDrop` | 0.10f | StumbleStepDrop | 292 | arquivo |  |
| `T_StumbleCooldown` | 0.5f | StumbleStepCooldown | 293 | arquivo |  |
| `T_StartStep` | 0.0f | StartStepEnabled | 297 | arquivo | Arrancada: o primeiro passo e do planejador (pe de tras, pousando a frente do corpo), com a pelve descendo e a... |
| `T_StartAhead` | 0.20f | StartStepAhead | 298 | arquivo |  |
| `T_StartSwing` | 0.28f | StartStepSwing | 299 | arquivo |  |
| `T_StartDrop` | 0.06f | StartStepDrop | 300 | arquivo |  |
| `T_StartSpeedBase` | 0.8f | StartSpeedBase | 301 | arquivo |  |
| `T_StartSpeedRate` | 6.0f | StartSpeedRate | 302 | arquivo |  |
| `T_StartPush` | 0.12f | StartPushSeconds | 303 | arquivo |  |
| `T_StartDose` | 0.30f | StartDoseSeconds | 306 | arquivo | Arrancada pelo ciclo, com a ajuda dosada nos primeiros instantes (s; 0 desliga): o teto StartSpeedBase + Start... |
| `T_StartLift` | 0.14f | StartStepLift | 307 | arquivo |  |
| `T_StumblePitch` | 0.15f | StumblePitchStart | 308 | arquivo |  |
| `T_StumblePitchSpan` | 0.15f | StumblePitchSpan | 309 | arquivo |  |
| `T_StumbleRate` | 1.0f | StumbleRateStart | 310 | arquivo |  |
| `T_StumbleRateSpan` | 1.0f | StumbleRateSpan | 311 | arquivo |  |
| `T_StumbleDeep` | 0.35f | StumbleDeepStart | 312 | arquivo |  |
| `T_StumbleDecel` | 15.0f | StumbleDecelStart | 313 | arquivo |  |
| `T_StumbleDecelSpan` | 10.0f | StumbleDecelSpan | 314 | arquivo |  |
| `T_StumbleBase` | 1.0f | StumbleBaselineSeconds | 315 | arquivo |  |
| `T_StumbleSwing` | 1.0f | StumbleSwingMeasured | 316 | arquivo |  |
| `T_StumbleCadence` | 0.0f | StumbleCadence | 317 | arquivo |  |
| `T_StumblePlace` | 0.0f | StumblePlacement | 318 | arquivo |  |
| `T_StumblePlaceMax` | 0.6f | StumblePlacementLimit | 319 | arquivo |  |
| `T_StumbleDrop` | 0.065f | StumbleDrop | 320 | arquivo |  |
| `T_StumbleTrunk` | 1.0f | StumbleTrunkScale | 321 | arquivo |  |
| `T_StumbleShift` | 1.0f | StumbleShiftScale | 322 | arquivo |  |
| `T_StumbleArms` | 0.75f | StumbleArmsScale | 323 | arquivo |  |
| `T_CurveLeanMax` | 0.45f | CurveLeanMaximum | 324 | arquivo |  |
| `T_CurveRoot` | 0.6f | CurveRootShare | 325 | arquivo |  |
| `T_CurveRootSprint` | 0.15f | CurveRootSprint | 326 | arquivo |  |
| `T_CurveRootFrom` | 3.5f | CurveRootSprintFrom | 327 | arquivo |  |
| `T_CurveRootSpan` | 2.5f | CurveRootSprintSpan | 328 | arquivo |  |
| `T_CurveFoot` | 1.0f | CurveFootShare | 329 | arquivo |  |
| `T_CurveDrop` | 0.6f | CurveDropShare | 330 | arquivo |  |
| `T_CurveHeight` | 0.70f | CurveLeanHeight | 331 | arquivo |  |
| `T_CurveNew` | 1.0f | CurveNew | 332 | arquivo |  |
| `T_FootYawMax` | 0.25f | CurveFootYawMaximum | 333 | arquivo |  |
| `T_FootYawStart` | 4.0f | CurveFootYawStart | 334 | arquivo |  |
| `T_FootYawSpan` | 6.0f | CurveFootYawSpan | 335 | arquivo |  |
| `T_FootYawOut` | 0.2f | CurveFootYawOuterShare | 336 | arquivo |  |
| `T_BodyTurnFilter` | 0.2f | BodyTurnFilterSeconds | 337 | arquivo |  |
| `T_FootYawRate` | 10.0f | CurveFootYawRate | 338 | arquivo |  |
| `T_FootHold` | 1.0f | CurveFootHoldYaw | 339 | arquivo |  |
| `T_FootingFix` | 1.0f | BrakeFootingStandingOnly | 340 | arquivo |  |
| `T_PivotSpeed` | 9.0f | PivotSpeed | 341 | arquivo |  |
| `T_PivotFull` | 5.0f | PivotFullRateBelow | 342 | arquivo |  |
| `T_RevDecel` | 16.0f | ReverseDeceleration | 343 | arquivo |  |
| `T_ToneRise` | 1.6f | SettleToneRise | 344 | arquivo |  |
| `T_FallSmooth` | 0.12f | FallSmoothSeconds | 345 | arquivo |  |
| `T_Fetal` | 1.0f | FetalEnabled | 346 | arquivo |  |
| `T_FetalHip` | 1.35f | FetalHip | 347 | arquivo |  |
| `T_FetalKnee` | 1.85f | FetalKnee | 348 | arquivo |  |
| `T_AirBalance` | 1.0f | AirBalanceNew | 349 | arquivo |  |
| `T_AirArmOpen` | 0.60f | AirArmOpen | 350 | arquivo |  |
| `T_AirTone` | 0.90f | FallAirTone | 351 | arquivo |  |
| `T_Windmill` | 0.55f | AirWindmill | 352 | arquivo |  |
| `T_WindmillRate` | 0.9f | AirWindmillRate | 353 | arquivo |  |
| `T_PedalRate` | 0.8f | AirPedalRate | 354 | arquivo |  |
| `T_GroundLife` | 1.0f | GroundLife | 355 | arquivo |  |
| `T_GroundLifeRate` | 0.55f | GroundLifeRate | 356 | arquivo |  |
| `T_FallLife` | 1.0f | FallLife | 357 | arquivo |  |
| `T_LandPrep` | 1.0f | LandingPrepEnabled | 358 | arquivo |  |
| `T_LandAlign` | 0.7f | LandingAlign | 359 | arquivo |  |
| `T_LandMax` | 0.60f | LandingMaxAngle | 360 | arquivo |  |
| `T_LandLeg` | 0.90f | LandingLegShare | 361 | arquivo |  |
| `T_LandLead` | 0.25f | LandingLeadSeconds | 362 | arquivo |  |
| `T_LandTrunk` | 0.5f | LandingTrunkAlign | 363 | arquivo |  |
| `T_LandRollSign` | -1.0f | LandingRollSign | 364 | arquivo |  |
| `T_LandTrunkSide` | 0.2f | LandingTrunkSide | 365 | arquivo |  |
| `T_LandRunLead` | 1.0f | LandRunLead | 366 | arquivo |  |
| `T_LandRunMax` | 0.35f | LandRunMaxAngle | 367 | arquivo |  |
| `T_NeckExt` | 0.20f | IdleNeckExtension | 368 | arquivo |  |
| `T_HeadExt` | 0.07f | IdleHeadExtension | 369 | arquivo |  |
| `T_ChestExt` | 0.08f | IdleChestExtension | 370 | arquivo |  |
| `T_PostureMoving` | 0.0f | IdlePostureMovingShare | 371 | arquivo |  |
| `T_BalArmFix` | 0.0f | BalanceArmFix | 372 | arquivo |  |
| `T_BalTrunk` | -0.13f | BalanceTrunkPitch | 373 | arquivo |  |
| `T_BalTrunkRoll` | -0.08f | BalanceTrunkRoll | 374 | arquivo |  |
| `T_SpineRollSlow` | 1.0f | SpineRollGainSlow | 375 | arquivo |  |
| `T_SpineRollFast` | 1.0f | SpineRollGainFast | 376 | arquivo |  |
| `T_SpineImpact` | 0.05f | SpineImpactFlex | 377 | arquivo |  |
| `T_SpineRollLimit` | 0.42f | SpineRollLimit | 378 | arquivo |  |
| `T_GaitLeash` | 1.0f | GaitLeashEnabled | 379 | arquivo |  |
| `T_GaitLeashM` | 0.10f | GaitLeashMeters | 380 | arquivo |  |
| `T_CapPlace` | 1.0f | CapturePlacement | 381 | arquivo |  |
| `T_CapPlaceAll` | 0.0f | CapturePlacementAllAssist | 383 | arquivo | O pouso pelo ponto de captura inteiro tambem com a ajuda alta (fracao). |
| `T_CapGain` | 1.0f | CapturePlacementGain | 384 | arquivo |  |
| `T_CapLimit` | 0.30f | CapturePlacementLimit | 385 | arquivo |  |
| `T_BalGait` | 1.0f | BalanceGaitShare | 386 | arquivo |  |
| `T_BalGaitNoLegs` | 1.0f | BalanceGaitNoLegs | 387 | arquivo |  |
| `T_Trip` | 1.0f | TripDetection | 388 | arquivo |  |
| `T_GaitHoldYaw` | 0.0f | GaitHoldYaw | 389 | arquivo |  |
| `T_TurnRateHigh` | 12.0f | StandingTurnRateHigh | 390 | arquivo |  |
| `T_TurnAccHigh` | 120.0f | StandingTurnAccelerationHigh | 391 | arquivo |  |
| `T_HipTwistHigh` | 0.45f | StandingHipTwistHigh | 396 | arquivo | (Giro parado, 04/10: a pelve espera os pes - torcao ate 0,45 rad sobre a base e o pe apoiado nunca gira no cha... |
| `T_FootPivot` | 0.0f | StandingFootPivot | 397 | arquivo |  |
| `T_TurnGainHigh` | 18.0f | StandingTurnGainHigh | 398 | arquivo |  |
| `T_GaitHoldYawFrom` | 0.5f | GaitHoldYawFrom | 399 | arquivo |  |
| `T_JumpCrouch` | 0.22f | JumpCrouchDepth | 400 | arquivo |  |
| `T_JumpPrepTrunk` | 0.2f | JumpPrepTrunkShare | 401 | arquivo |  |
| `T_LowTurnAc` | 100.0f | LowAssistTurnCentripetal | 402 | arquivo |  |
| `T_LowTurnMin` | 1.2f | LowAssistTurnMinimum | 403 | arquivo |  |
| `T_CutCos` | -0.5f | CutBrakeCosine | 404 | arquivo |  |
| `T_CutSpeed` | 2.5f | CutBrakeMinimumSpeed | 405 | arquivo |  |
| `T_CapBack` | 1.0f | CapturePullBackShare | 406 | arquivo |  |
| `T_CapBackDead` | 0.25f | CaptureBackDeadZone | 407 | arquivo |  |
| `T_StanceStiffness` | 2.5f | StanceStiffness | 408 | arquivo |  |
| `T_GaitLeadSpeed` | 0.0f | GaitLeadSpeed | 409 | arquivo |  |
| `T_SprintClip` | 0.0f | SprintClipSpeed | 410 | arquivo |  |
| `T_SprintClipAll` | 0.0f | SprintClipAllAssist | 411 | arquivo |  |
| `T_SprintRateFloor` | 0.55f | SprintRateFloor | 414 | arquivo | Piso de ritmo do ciclo de sprint em qualquer ajuda (o corpo abaixo da velocidade do clipe: curva fechada, arra... |
| `T_LowBrakeDec` | 12.0f | LowAssistBrakeDeceleration | 415 | arquivo |  |
| `T_LowBrakeLeash` | 0.8f | LowAssistBrakeLeash | 416 | arquivo |  |
| `T_BrakeNoFF` | 1.0f | BrakeAnchorNoFeedforward | 417 | arquivo |  |
| `T_SectorHold` | 1.0f | SectorHoldWhileBraking | 418 | arquivo |  |
| `T_StrafeBrake` | 2.0f | StrafeBrakeEngageSpeed | 419 | arquivo |  |
| `T_LockTouch` | 1.0f | LockAtTouch | 420 | arquivo |  |
| `T_LockTouchWait` | 0.10f | LockTouchWaitSeconds | 421 | arquivo |  |
| `T_StanceRise` | 0.03f | StanceRiseLimit | 422 | arquivo |  |
| `T_GaitSwingMeasured` | 0.0f | GaitSwingMeasuredPelvis | 423 | arquivo |  |
| `T_FirstTouch` | 1.0f | LockAtFirstTouch | 424 | arquivo |  |
| `T_LockPull` | 0.08f | LockTouchPull | 425 | arquivo |  |
| `T_GaitSwingRelease` | 0.15f | GaitSwingReleaseSeconds | 426 | arquivo |  |
| `T_GaitSwingMode` | 0.0f | - | 429 | arquivo | 0 posicao e orientacao medidas; 1 so a posicao; 2 so a orientacao; 3 o plano e a orientacao; 4 so o plano. |
| `T_GaitSwingChange` | 1.0f | GaitSwingOnlyInChange | 430 | arquivo |  |
| `T_GaitSwingLatch` | 1.0f | GaitSwingLatch | 431 | arquivo |  |
| `T_LockTouchChange` | 1.0f | LockTouchOnlyInChange | 432 | arquivo |  |
| `T_GaitChangeError` | 1.0f | GaitChangeVelocityError | 433 | arquivo |  |
| `T_SwingTrack` | 0.0f | SwingTrackGain | 434 | arquivo |  |
| `T_SwingTrackMax` | 0.15f | SwingTrackMaximum | 435 | arquivo |  |
| `T_LowBrakeRef` | 0.30f | LowAssistBrakeReferenceLeash | 436 | arquivo |  |
| `T_CapBrake` | 0.0f | CaptureInBrakeRun | 437 | arquivo |  |
| `T_LowBrakeStep` | 1.8f | LowAssistBrakeStepSpeed | 438 | arquivo |  |
| `T_GaitLeadMin` | 3.5f | GaitLeadMinimumRequest | 439 | arquivo |  |
| `T_LowRateFloor` | 0.55f | LowAssistRateFloor | 440 | arquivo |  |
| `T_ReachLift` | 0.0f | ReachLiftPerMeter | 441 | arquivo |  |
| `T_ReachLiftStart` | 0.10f | ReachLiftStart | 442 | arquivo |  |
| `T_ReachLiftMax` | 0.15f | ReachLiftMaximum | 443 | arquivo |  |
| `T_ReachLiftBrake` | 0.0f | ReachLiftBrakeOnly | 444 | arquivo |  |
| `T_SwingStiff` | 1.0f | SwingLowAssistStiffness | 445 | arquivo |  |
| `T_CrossGuard` | 0.0f | CrossGuardWidth | 446 | arquivo |  |
| `T_PlannerSwing` | 0.0f | PlannerSwingLowAssist | 447 | arquivo |  |
| `T_SwingTorque` | 1.0f | SwingLowAssistTorque | 448 | arquivo |  |
| `T_LowRateBrake` | 1.0f | LowAssistRateFloorInBrake | 449 | arquivo |  |
| `T_GroundAlive` | 1.0f | GroundAlive | 450 | arquivo |  |
| `T_FallNew` | 1.0f | FallProtectNew | 451 | arquivo |  |
| `T_FallLegs` | 1.0f | FallLegsNew | 452 | arquivo |  |
| `T_FallArms` | 1.0f | FallArmsNew | 453 | arquivo |  |
| `T_KneeFwd` | 1.10f | FallKneeForward | 454 | arquivo |  |
| `T_KneeBack` | 1.30f | FallKneeBackward | 455 | arquivo |  |
| `T_KneeSide` | 0.90f | FallKneeSide | 456 | arquivo |  |
| `T_HipFwd` | 0.55f | FallHipForward | 457 | arquivo |  |
| `T_HipBack` | 1.10f | FallHipBackward | 458 | arquivo |  |
| `T_HipSide` | 0.50f | FallHipSide | 459 | arquivo |  |
| `T_FallLegTone` | 0.50f | FallLegTone | 460 | arquivo |  |
| `T_FallTrunkTone` | 0.40f | FallTrunkTone | 461 | arquivo |  |
| `T_ElbowReach` | 0.35f | FallElbowReach | 462 | arquivo |  |
| `T_ElbowAbsorb` | 0.90f | FallElbowAbsorb | 463 | arquivo |  |
| `T_AbsorbSeconds` | 0.50f | FallAbsorbSeconds | 464 | arquivo |  |
| `T_CatchTone` | 1.10f | FallCatchTone | 465 | arquivo |  |
| `T_ArmExt` | 0.98f | FallArmExtension | 466 | arquivo |  |
| `T_ArmFwd` | 0.75f | FallArmForward | 467 | arquivo |  |
| `T_ArmIter` | 12.0f | FallArmIterations | 468 | arquivo |  |
| `T_ArmReadyFwd` | 0.30f | FallArmReadyForward | 469 | arquivo |  |
| `T_ArmReadyDrop` | 0.20f | FallArmReadyDrop | 470 | arquivo |  |
| `T_ArmNear` | 0.0f | FallArmNearHeight | 471 | arquivo |  |
| `T_ArmSide` | 0.30f | FallArmSideReach | 472 | arquivo |  |
| `T_ArmBack` | 0.20f | FallArmBackReach | 473 | arquivo |  |
| `T_FarArm` | 0.7f | FallFarArmShare | 474 | arquivo |  |
| `T_FallTwist` | 0.55f | FallTwistRadians | 475 | arquivo |  |
| `T_PivotCrouch` | 0.12f | PivotCrouchMeters | 476 | arquivo |  |
| `T_RestTone` | 0.40f | SettleRestingTone | 554 | arquivo |  |
| `T_SettleRest` | 0.20f | SettleRestBeforeGather | 556 | arquivo | Caido e parado, quanto tempo fica largado antes de se arrumar para levantar. |
| `T_RetryRest` | 0.10f | RetryRestBeforeGather | 558 | arquivo | Numa nova tentativa de levantar (a anterior falhou), quanto espera. |
| `T_ReflexTone` | 0.85f | FallReflexLimbTone | 562 | arquivo | Caindo: tonus dos bracos/pescoco (reflexo) e do resto (queda sem volta). O corpo (era 0,38) cai solto; os brac... |
| `T_FallArms` | 0.0f | FallArmsReflex | 565 | arquivo | Reflexo das maos ao chao caindo (os bracos prontos a frente e depois estendidos ate o chao): tirado a pedido d... |
| `T_PostureLoss` | 1.0f | PostureLossFall | 572 | arquivo | Postura perdida (pedido do usuario, 05/10): perdendo a postura, a ajuda segurava o corpo torto por ate 1,5 s -... |
| `T_PostureLost` | 0.73f | PostureLostTilt | 580 | arquivo | A postura e o pendulo: o centro de massa contra o meio dos pes (rad da vertical). Trotando e correndo ele nao ... |
| `T_PostureHeld` | 0.44f | PostureHeldTilt | 581 | arquivo |  |
| `T_PostureHeldDev` | 0.35f | PostureHeldDeviation | 582 | arquivo |  |
| `T_PostureHeldSec` | 0.30f | PostureHeldSeconds | 583 | arquivo |  |
| `T_PostureChest` | 0.96f | PostureChestLost | 585 | arquivo | O peito tombado alem disto (rad, da vertical) tambem e queda. |
| `T_StaggerMin` | 8.0f | StaggerImpulseMin | 594 | arquivo | Cambaleio (pedido do usuario, 05/10 noite: "mesmo ragdolizado, tentar manter a postura e se recobrar"): o choq... |
| `T_StaggerFull` | 45.0f | StaggerImpulseFull | 595 | arquivo |  |
| `T_StaggerRecover` | 0.75f | StaggerRecoverPerSecond | 596 | arquivo |  |
| `T_StaggerLeg` | 0.20f | StaggerLegTone | 597 | arquivo |  |
| `T_StaggerTorso` | 0.10f | StaggerTorsoTone | 598 | arquivo |  |
| `T_StaggerArm` | 0.04f | StaggerArmTone | 599 | arquivo |  |
| `T_StaggerHold` | 0.25f | StaggerHoldSeconds | 603 | arquivo | No choque o corpo fica solto por este tempo (s, vezes o tamanho dele) antes de o tonus comecar a voltar; tronc... |
| `T_StaggerGravity` | 0.9f | StaggerGravityDrop | 604 | arquivo |  |
| `T_StaggerFallen` | 0.25f | StaggerFallenRelief | 607 | arquivo | E o quanto mais tombado (no cosseno da pelve) ele ainda nao conta como caido com um pe no chao. |
| `T_StaggerTilt` | 0.35f | StaggerTiltRelief | 608 | arquivo |  |
| `T_PostureOk` | 0.21f | PostureRegainedTilt | 610 | arquivo | Volta a valer (o corpo de pe de novo) abaixo disto. |
| `T_PostureSupport` | 0.0f | PostureSupportPendulumEnv | 614 | arquivo | v2: o pendulo da postura contra os pes apoiados (ver o bloco da postura). Locomocao fisica v2 (CharacterLocomo... |
| `T_StartFromRest` | 0.0f | StartOnlyFromRestEnv | 615 | arquivo |  |
| `T_LandResync` | 0.0f | LandingResyncEnv | 616 | arquivo |  |
| `T_PostureGaitPelvis` | 0.87f | PostureGaitPelvisLost | 617 | arquivo |  |
| `T_BraceHeight` | 0.40f | BraceMinimumHeight | 625 | arquivo | v2, recuperacao apoiada: altura minima do apoio acima do chao (m), memoria do contato (s), tempo maximo da ten... |
| `T_BraceMemory` | 0.30f | BraceMemorySeconds | 626 | arquivo |  |
| `T_BraceAttempt` | 4.0f | BraceAttemptSeconds | 627 | arquivo |  |
| `T_BraceStall` | 1.5f | BraceStallSeconds | 628 | arquivo |  |
| `T_BraceUpright` | 0.17f | BraceMinimumUpright | 629 | arquivo |  |
| `T_ImpactTone` | 0.60f | SettleImpactTone | 637 | arquivo | Tonus ao bater no chao (cai dele ate SettleRestingTone em ~0,1 s). |
| `T_StanceBandV2` | StanceRootBandV2 | stanceBandV2Env | 2458 | CharacterLocomotion3D::update |  |
| `T_LeanNeed` | 0.0f | leanNeedGain | 6406 | CharacterLocomotion3D::update |  |
| `T_LeanNeedRate` | 1.5f | leanNeedRate | 6407 | CharacterLocomotion3D::update |  |
| `T_LeanNeedMax` | 5.0f | leanNeedMaximum | 6408 | CharacterLocomotion3D::update |  |
| `T_LeanNeedPower` | 18.0f | leanNeedPower | 6409 | CharacterLocomotion3D::update |  |
| `T_CapConsistent` | 0.0f | captureConsistent | 7301 | CharacterLocomotion3D::update | Experimento v2 (T_CapConsistent): o pe corrige a parte que a ajuda nao corrige (1 - ajuda). |
| `T_StanceMeasured` | 0.0f | stanceMeasured | 8770 | CharacterLocomotion3D::update |  |
| `T_StanceMeasuredFrom` | 0.2f | stanceMeasuredFrom | 8771 | CharacterLocomotion3D::update |  |
| `T_StanceMeasuredSpan` | 0.4f | stanceMeasuredSpan | 8772 | CharacterLocomotion3D::update |  |
| `T_StaggerTorsoLow` | 0.30f | torsoLowAssist | 10031 | CharacterLocomotion3D::update | (O tronco tambem: a 10% de tonus, sem ajuda o peito tombava ~75 graus de lado num empurrao correndo.) (Perto d... |

## `src/Engine/Character/DefaultLocomotion3D.cpp`

| Nome | Valor | Variavel | Linha | Funcao | O que faz (comentario) |
|---|---|---|---|---|---|
| `T_VoadoraMeia` | 0.10f | AirTackleLandHalfLife | 419 | DefaultLocomotion3D::reset | O pouso da voadora: a meia-vida e a duracao da inercia (s). |
| `T_VoadoraDur` | 0.45f | AirTackleLandSeconds | 420 | DefaultLocomotion3D::reset |  |
| `T_VoadoraMistMin` | 0.12f | AirTackleBlendMinimum | 421 | DefaultLocomotion3D::reset |  |
| `T_VoadoraMergulho` | 2.0f | AirTackleDiveStretch | 422 | DefaultLocomotion3D::reset |  |

## `src/Engine/Control/AdaptivePhysicalCharacter3D.cpp`

| Nome | Valor | Variavel | Linha | Funcao | O que faz (comentario) |
|---|---|---|---|---|---|
| `T_LegWalkGain` | 4.0f | LegWalkingVelocityGain | 67 | std::clamp |  |
| `T_LegWalkAcc` | 4.0f | LegWalkingAcceleration | 68 | std::clamp |  |
| `T_LegWalkHF` | 3.0f | LegWalkingHeightFrequency | 69 | std::clamp |  |
| `T_LegWalkUp` | 0.5f | LegWalkingSpringUp | 70 | std::clamp |  |
| `T_LegWalkDown` | 0.25f | LegWalkingSpringDown | 71 | std::clamp |  |
| `T_NavAccBase` | 4.0f | base | 151 | std::clamp |  |
| `T_NavAccGain` | 6.0f | gain | 152 | std::clamp |  |
| `T_NavLeadBase` | 0.8f | base | 157 | std::clamp |  |
| `T_NavLeadGain` | 6.0f | gain | 158 | std::clamp |  |
| `T_NavLeadFree` | 0.30f | releaseFrom | 159 | std::clamp |  |
| `T_NavLeadSpeed` | 0.0f | leadSpeed | 175 | std::clamp | (Mais solta correndo mais rapido: T_NavLeadSpeed por m/s do corpo.) |
| `T_NavCutFrom` | 0.45f | cutFrom | 181 | std::clamp |  |
| `T_NavCutSpan` | 1.2f | cutSpan | 182 | std::clamp |  |
| `T_NavCutFloor` | 0.35f | cutFloor | 183 | std::clamp |  |
| `T_NavLateral` | 5.0f | lateral | 184 | std::clamp |  |
| `T_NavTurnShare` | 1.0f | share | 185 | std::clamp |  |
| `T_HitMin` | 2.0f | hitMinimum | 398 | AdaptivePhysicalCharacter3D::update | Contato externo de verdade (fora o apoio dos pes): a partir de ~2 N.s (um esbarrao) a memoria de perturbacao v... |
| `T_HitFull` | 25.0f | hitFull | 399 | AdaptivePhysicalCharacter3D::update |  |
| `T_HitCalm` | 0.5f | hitCalm | 415 | AdaptivePhysicalCharacter3D::update |  |
| `T_GaitPlanarScale` | 1.0f | gaitPlanarScale | 499 | AdaptivePhysicalCharacter3D::update | TEMPTUNE: andando/correndo, a ajuda segura o corpo com molas mais macias - com as de parado (14 no plano, 12 n... |
| `T_GaitHeightScale` | 1.0f | gaitHeightScale | 500 | AdaptivePhysicalCharacter3D::update |  |
| `T_UprightCalmAbs` | 15.0f | calmAbsoluteExtra | 566 | AdaptivePhysicalCharacter3D::update | TEMPTUNE: no movimento proprio (sem contato externo recente), a faixa absoluta tambem se estende - correndo, u... |
| `T_RunAnchor` | 1.0f | runAnchor | 615 | AdaptivePhysicalCharacter3D::update |  |
| `T_RunSpringCap` | -1.0f | runSpringUpCap | 682 | AdaptivePhysicalCharacter3D::update | TEMPTUNE: correndo, a mola de altura da pelve nao ergue o corpo afundado de uma vez (ate 0,9 g na pelve) - jun... |
| `T_RunLegSpringCap` | -1.0f | runLegSpringUpCap | 715 | AdaptivePhysicalCharacter3D::update |  |
| `T_WalkDecouple` | 1.0f | walkShareDecouple | 747 | AdaptivePhysicalCharacter3D::update | As pernas assumem ate (1 - legsAssistRetained): o resto da ajuda segue na pelve. |
| `T_WeightFloor` | 0.6f | weightWalkFloor | 836 | AdaptivePhysicalCharacter3D::update | (Pela ajuda andando do jogador, sem a mistura com a de parado da freada: com ela, freando a 100% as pernas pas... |
| `T_PushGain` | 1.6f | pushGain | 865 | AdaptivePhysicalCharacter3D::update |  |
| `T_LandGain` | 0.5f | landGain | 866 | AdaptivePhysicalCharacter3D::update |  |
| `T_FlightGain` | 0.7f | flightGain | 867 | AdaptivePhysicalCharacter3D::update |  |
| `T_LandNoBoost` | 0.0f | landNoPulse | 889 | AdaptivePhysicalCharacter3D::update |  |
| `T_AssistContact` | 1.0f | contactShare | 915 | AdaptivePhysicalCharacter3D::update |  |
| `T_AssistContactNorm` | 1.0f | contactNormalize | 927 | AdaptivePhysicalCharacter3D::update | O que a ajuda dava no voo vai para o apoio: com o pe no chao ela empurra na proporcao do tempo no ar (1/fracao... |
| `T_JumpDuty` | 0.0f | jumpDutyEnv | 932 | AdaptivePhysicalCharacter3D::update | (v2 - ou T_JumpDuty no v1: o voo de um pulo nao entra na fracao em contato - ela caia a ~0,3 num pulo e o refo... |
| `T_LandNoBoost` | 0.0f | landNoBoost | 958 | AdaptivePhysicalCharacter3D::update | (Experimento v2, T_LandNoBoost = s: logo depois do pouso de um pulo, nem o reforco do apoio nem o pulso - a co... |
| `T_AssistContactVert` | 1.0f | verticalShare | 976 | AdaptivePhysicalCharacter3D::update |  |
| `T_VertBoostHit` | 1.0f | verticalBoostOnlyAfterHit | 985 | AdaptivePhysicalCharacter3D::update | TEMPTUNE: na corrida calma (sem contato externo ha 1,5 s, o jogador mandando correr) o reforco vertical no apo... |
| `T_BoostLandHold` | 0.6f | boostLandHold | 989 | AdaptivePhysicalCharacter3D::update | (O pouso de um pulo tambem conta como batida: o reforco segura o corpo que chega rapido e gira para a frente s... |
| `T_BoostLandFade` | 0.6f | boostLandFade | 990 | AdaptivePhysicalCharacter3D::update |  |
| `T_PlanarCap` | -1.0f | planarCap | 1039 | AdaptivePhysicalCharacter3D::update |  |
| `T_PlanarCapAssist` | 0.0f | planarCapAssist | 1040 | AdaptivePhysicalCharacter3D::update |  |
| `T_PostureFull` | 0.0f | postureFull | 1091 | AdaptivePhysicalCharacter3D::update | Andando, walkingAssist limita diretamente a postura auxiliar. Parado, postureShare ja faz a divisao pernas/pel... |
| `T_LegDutyNorm` | 0.0f | legDutyNormalize | 1242 | AdaptivePhysicalCharacter3D::update | O centro de pressao sai do peso inteiro (o equilibrio pelos pes nao depende de quem carrega o peso); pelas jun... |

## `src/Engine/Locomotion/ContactFootwork3D.cpp`

| Nome | Valor | Variavel | Linha | Funcao | O que faz (comentario) |
|---|---|---|---|---|---|
| `T_BrakeSingle` | 1.0f | BrakeSingleTakeoverStep | 174 | std::atof | Assumindo os dois pes no ar freando, so o mais alto da passo de freada. |
| `T_UnreachHold` | 1.0f | UnreachableDuringSettleHold | 176 | std::atof | Recem-freado, o pe fora do alcance (sem apoio) volta para a base. |
| `T_SideInner` | 0.0f | SidewaysInnerLands | 178 | std::atof | Freando de lado, o pe de dentro assumido no ar so pousa (o de fora abre). |
| `T_SideOpen` | 0.0f | SideOpenFirst | 181 | std::atof | Freando de lado: o ponto de captura alem disto (m) do pe de fora, ele abre antes de o de dentro fechar. |
| `T_SideOpenBeyond` | 0.20f | SideOpenFirstBeyond | 182 | std::atof |  |
| `T_RecoverChosen` | 0.0f | RecoverOnlyChosenFoot | 184 | std::atof | Recuperacao: so o pe escolhido muda de passo no ar. |
| `T_NarrowBase` | 0.06f | SettleHoldNarrowBase | 186 | std::atof | Recem-freado, base mais estreita que isto (m; negativa = cruzada) acomoda. |

