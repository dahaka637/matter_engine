#pragma once

#include "Engine/Athlete/Actions/BallHandling3D.hpp"
#include "Engine/Athlete/Body/ActiveBodyController3D.hpp"
#include "Engine/Athlete/Body/ActiveStance3D.hpp"
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
    // Locomocao por pegadas (Etapa 3): no modo padrao o clipe vira estilo e os
    // pes apoiados ficam onde pousaram (FootstepPlanner3D). Desligada ate o
    // usuario aprovar (a chave A/B do laboratorio).
    bool footstepLocomotion = false;
    FootstepSettings3D footsteps;
    // Manejo novo (Etapa 5): pedido com desfecho, planejador, gesto pelo
    // estado, sensor do pe e o resultado aprovado como impulso. Desligado ate
    // o usuario aprovar (a chave A/B "Manejo novo"). So vale com a locomocao
    // por pegadas: e ela que reserva o apoio e trava o pe onde o gesto o
    // pousou (sem ela o pe escorregava de volta ao lugar do clipe).
    bool newBallHandling = false;
    // No Ativo, o manejo novo so age com o centro de massa a menos disto
    // (m/s) da velocidade da capsula: desequilibrado, o pedido espera.
    float ballActiveReadySpeed = 0.35f;
    [[nodiscard]] bool newBallHandlingActive() const { return newBallHandling && footstepLocomotion; }
    BallHandlingSettings3D ballHandling;
    // Manejo de bola.
    float ballContactTiming = 1.0f;
    bool ballHitboxContact = true;
    BallForceSettings3D ballForce = defaultBallForceSettings3D();
    bool ballForceOnly = false;
    bool ballStabilityAssist = true;
};

} // namespace MatterEngine
