#pragma once

#include "Engine/Character/BallControl3D.hpp"
#include "Engine/Character/BiomechanicalBipedExperiment3D.hpp"
#include "Engine/Character/CharacterLocomotion3D.hpp"
#include "Engine/Character/DefaultLocomotion3D.hpp"
#include "Engine/Character/PoseInertializer3D.hpp"
#include "Engine/Control/AdaptivePhysicalCharacter3D.hpp"
#include "Engine/Control/ImpactAssist3D.hpp"

#include <cstdint>
#include <vector>

namespace MatterEngine {

// O controle de hoje de um atleta (legado do rework de 10/10/2026): os
// controladores e os relogios do modo do corpo que moravam no
// SpawnedRagdollInstance do laboratorio. O runtime novo (AthleteRuntime3D)
// roda este caminho enquanto as pecas novas nao o substituem.
//
// Modo do corpo (locomocao padrao, 04/10): o boneco controlado anda pela
// capsula (o corpo guiado segue a animacao do DefaultLocomotion3D). O modo
// fisico (CharacterLocomotion3D + AdaptivePhysicalCharacter3D, com os
// motores) entra numa pancada, num pouso ou agarrado; quando o corpo assenta
// ele volta ao padrao (Returning: a pose fisica levada ate a animada).
enum class LegacyBodyMode3D : std::uint8_t { Animated, Physical, Returning };

struct LegacyAthleteState3D {
    // Rumo da guia, separado da rotacao fisica da pelve. O idle em alerta
    // abre a pelve em relacao ao olhar; realimentar essa abertura como
    // comando acumularia giro em todo tick.
    float guideFacingYawRadians = 0.0f;
    CharacterLocomotion3D locomotion;
    AdaptivePhysicalCharacter3D adaptivePhysics;
    // Manejo de bola (so o personagem controlado usa) e a escala da
    // velocidade que ele pede (o chute correndo freia um pouco).
    BallControl3D ballControl;
    float ballSpeedScale = 1.0f;
    // A ultima saida do manejo (o desenho de depuracao) e o reforco de
    // estabilidade do gesto (modo fisico, V3).
    BallControlOutput3D ballLastOutput;
    float ballStability = 0.0f;
    DefaultLocomotion3D defaultLocomotion;
    LegacyBodyMode3D bodyMode = LegacyBodyMode3D::Physical;
    // Tempo no modo atual e, no fisico, tempo assentado (sem pancada, de pe,
    // no chao).
    float bodyModeSeconds = 0.0f;
    float calmSeconds = 0.0f;
    // Animado: tempo no ar. Fisico: entrou por um pulo (volta logo depois do
    // pouso).
    float airSeconds = 0.0f;
    bool physicalFromJump = false;
    // Fisico, ja assentado para voltar: a ajuda sobe aos poucos (0..1) ate a
    // cheia e so entao o corpo volta ao padrao.
    float returnBoost = 0.0f;
    // Fisico por um pulo: tempo no chao desde o pouso.
    float landedSeconds = 0.0f;
    // Os links estao guiados (o corpo do modo padrao) e, saindo dele para o
    // fisico sem pancada, quanto falta (s) da autoridade descendo e o total.
    bool guided = false;
    float guidedExitLeft = 0.0f;
    float guidedExitSeconds = 0.0f;
    // O agachar do tick anterior (a borda pede o carrinho) e o carrinho do
    // modo padrao no tick anterior.
    bool crouchHeld = false;
    bool defaultTackling = false;
    // A volta: a pose fisica do comeco - a raiz (em relacao a base da
    // capsula) e as juntas.
    Vec3 returnRootOffset;
    Quaternion returnRootOrientation;
    std::vector<Vec3> returnCoordinates;
    // A volta pela inercia ("Volta do fisico acompanhando o corpo").
    PoseInertializer3D returnInertia;
    // Levou carrinho: 0 no golpe, 1 com a ajuda de volta (so o aviso no log;
    // a ajuda e a do impactAssist).
    float tackledRecovery = 1.0f;
    // A ajuda depois de um impacto (06/10): cai pela forca do choque (ate
    // 50%; num carrinho levado, ate 20%) e volta aos poucos.
    ImpactAssist3D impactAssist;
    // Pancada enquanto levanta: 0 no golpe, 1 com a ajuda de volta.
    float getUpHitRecovery = 1.0f;
    // Fisica adaptativa nos bonecos nao controlados: uma capsula virtual (so
    // posicao no plano) que segue o corpo como a do jogador - e para onde a
    // correcao do corpo puxa.
    Vec3 navigationAnchor;
    Vec3 navigationAnchorVelocity;
    bool navigationAnchorValid = false;
    BiomechanicalBipedExperiment3D biomechanics;
    // A pose de referencia do controle (depuracao) e a parcela fisica da
    // politica antiga (nao e mistura visual entre dois corpos).
    RagdollAnimationPose3D animationPose;
    RagdollAnimationPose3D previousAnimationPose;
    float physicsBlend = 0.0f;
};

} // namespace MatterEngine
