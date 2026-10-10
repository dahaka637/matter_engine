#pragma once

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
    // Manejo de bola.
    float ballContactTiming = 1.0f;
    bool ballHitboxContact = true;
    BallForceSettings3D ballForce = defaultBallForceSettings3D();
    bool ballForceOnly = false;
    bool ballStabilityAssist = true;
};

} // namespace MatterEngine
