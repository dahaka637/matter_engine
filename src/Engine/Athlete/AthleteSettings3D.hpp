#pragma once

#include "Engine/Athlete/Synthesis/LegClearance3D.hpp"
#include "Engine/Athlete/Actions/BallHandling3D.hpp"
#include "Engine/Athlete/Actions/BallPlay3D.hpp"
#include "Engine/Athlete/Body/ActiveBodyController3D.hpp"
#include "Engine/Athlete/Body/ActiveStance3D.hpp"
#include "Engine/Athlete/Body/PhysicalFeetAssist3D.hpp"
#include "Engine/Athlete/Planning/FootstepPlanner3D.hpp"
#include "Engine/Character/BallControl3D.hpp"

namespace MatterEngine {

// Os ajustes do controle dos atletas (o que o laboratorio chamava de m_* no
// WorkbenchApp): um lugar so, sem variavel de ambiente. O laboratorio liga os
// sliders aqui; a sonda sem janela usa os padroes ou --ajuste. Os padroes sao
// os aprovados pelo usuario ate 10/10/2026.
struct AthleteRuntimeSettings3D {
    // Modo do corpo: o padrao (animado, pela capsula) e o fisico adaptativo.
    bool defaultLocomotion = true;
    bool adaptivePhysics = true;
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
    float getUpAssistPercent = 130.0f;
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
    bool defaultVoadoraContactOnly = true;
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
    bool defaultLandingAbsorb = true;
    float defaultCurveLeanDeadZone = 3.5f;
    float defaultTrotSpeed = 4.5f;
    float defaultSprintForwardSpeed = 7.8f;
    float defaultSprintSideSpeed = 5.2f;
    float defaultSprintBackwardSpeed = 5.2f;
    // A regua "Resposta" da capsula no modo padrao (%; ver
    // athleteCapsuleResponse3D): 100 e o aprovado (16 / 22 / 30 m/s2, 10 rad/s).
    float capsuleResponsePercent = 100.0f;
    // Corpo Ativo (o estado intermediario, Etapa 2 do rework): ligado, as
    // pancadas leves, a parede e o membro no mundo levam ao Ativo (e nao ao
    // fisico pleno) e a volta do fisico passa por ele. Impulso (N.s) de algo
    // que se move a partir do qual o corpo vai ao Ativo e a partir do qual vai
    // direto ao fisico pleno; a batida da capsula (m/s) que vai ao fisico.
    // Desligado desde o teste do usuario (10/10, noite): andando ou correndo
    // depois do levantar e no encostao ele derrubava o jogador de novo (a
    // ponte fisico -> padrao do Plano_Claude_v2 o substitui).
    bool activeBody = false;
    // A ponte fisico -> padrao (plano v2, fase D): saindo do fisico (o fim
    // do levantar, o pouso de pe), o corpo segue fisico por bridgeSeconds
    // (bridgeMovingSeconds com as teclas) - os musculos do Ativo indo a pose
    // do padrao, que ja roda, a ajuda de passagem e o guiamento dos ossos
    // subindo de bridgeAuthorityStart a cheio - e entao o guiado. Contato
    // no padrao nao entra nela.
    bool physicalBridge = true;
    float bridgeSeconds = 0.45f;
    float bridgeMovingSeconds = 0.30f;
    float bridgeAuthorityStart = 0.15f;
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
    // Manejo novo (a chave A/B "Manejo novo"): desde o plano v2 (fase C), o
    // BallPlay3D - a acao pela bola na area, a forca pela regua do tempo ate o
    // contato, o gesto so visual. Com ou sem as pegadas. (O manejo da Etapa
    // 5, BallHandling3D, fica no codigo ate a limpeza, fora do caminho.)
    bool newBallHandling = false;
    // No Ativo, o manejo da Etapa 5 so agia com o centro de massa a menos
    // disto (m/s) da velocidade da capsula.
    float ballActiveReadySpeed = 0.35f;
    [[nodiscard]] bool newBallHandlingActive() const { return newBallHandling; }
    BallHandlingSettings3D ballHandling;
    BallPlaySettings3D ballPlay;
    // As pernas nunca se atravessam na pose do padrao (plano v4).
    LegClearanceSettings3D legClearance;
    // Manejo de bola.
    float ballContactTiming = 0.0f;
    bool ballHitboxContact = true;
    BallForceSettings3D ballForce = defaultBallForceSettings3D();
    bool ballForceOnly = false;
    bool ballStabilityAssist = true;
};

} // namespace MatterEngine
