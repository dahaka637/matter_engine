#pragma once

#include "Engine/Athlete/Physical/DynamicTurnMove3D.hpp"
#include "Engine/Athlete/Reference/PhaseLock3D.hpp"
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
// Active (Etapa 2 do rework): o Corpo Ativo - fisico de verdade seguindo a
// pose do modo padrao, com a ajuda so pelos pes apoiados.
enum class LegacyBodyMode3D : std::uint8_t { Animated, Physical, Returning, Active };

struct LegacyAthleteState3D {
    // Rumo da guia, separado da rotacao fisica da pelve. O idle em alerta
    // abre a pelve em relacao ao olhar; realimentar essa abertura como
    // comando acumularia giro em todo tick.
    float guideFacingYawRadians = 0.0f;
    CharacterLocomotion3D locomotion;
    AdaptivePhysicalCharacter3D adaptivePhysics;
    // A virada brusca (experimento): o gerador e as correcoes que o atleta
    // le neste tick (as do RL somadas as dela).
    DynamicTurnMove3D turnMove;
    PhysicalCorrections3D turnCorrections;
    DefaultLocomotion3D defaultLocomotion;
    // O padrao a sombra no fisico (AthleteRuntimeSettings3D::physicalReference):
    // a referencia de estilo do contrato do RL, so calculando a pose. Uma
    // instancia propria: nunca mexe no padrao de verdade nem na volta a ele.
    DefaultLocomotion3D referenceShadow;
    bool referenceShadowActive = false;
    // O relogio da sombra segue o corpo (Athlete/Reference/PhaseLock3D).
    PhaseLock3D referencePhaseLock;
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
    // A velocidade na volta (plano v4, etapa C): parte da do corpo e vai a
    // pedida numa aceleracao suave, por ate returnRampLeft s - a capsula
    // saltava para a pedida e a pose do padrao, presa nela, puxava o corpo.
    Vec3 returnVelocity;
    float returnRampLeft = 0.0f;
    float returnSpeedScale = 1.0f;
    // A ponte fisico -> padrao (plano v2, fase D; o modo Ativo por dentro):
    // o tempo nela e quanto ela dura (s).
    bool bridge = false;
    float bridgeSeconds = 0.0f;
    float bridgeDuration = 0.45f;
    // Neste fisico ele ja esteve levantando (o fim do levantar vai direto a
    // ponte).
    bool getUpSeen = false;
    // A ponte no ultimo tick segurou os pes onde estavam (parado): a pose dela
    // nao e a do padrao, e a saida recomeca a inercia do corpo.
    bool bridgeStance = false;
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
