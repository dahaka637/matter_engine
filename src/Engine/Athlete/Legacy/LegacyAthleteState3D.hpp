#pragma once

#include "Engine/Athlete/Physical/DynamicTurnMove3D.hpp"
#include "Engine/Athlete/Reference/PhaseLock3D.hpp"
#include "Engine/Character/BiomechanicalBipedExperiment3D.hpp"
#include "Engine/Character/CharacterLocomotion3D.hpp"
#include "Engine/Character/DefaultLocomotion3D.hpp"
#include "Engine/Character/PoseInertializer3D.hpp"
#include "Engine/Control/AdaptivePhysicalCharacter3D.hpp"
#include "Engine/Control/ImpactAssist3D.hpp"
#include "Engine/Physics/PhysicsScene3D.hpp"

#include <cstdint>
#include <string>
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
    // (16/11) O fisico do padrao pelo cerebro unico do RL (o jogo): de pe de novo, ele pede a volta ao padrao - o proximo passo
    // volta como na volta de sempre, do corpo como esta.
    bool rlReturn = false;
    // (16/11) Por que o padrao foi ao fisico pela ultima vez (o registro do cerebro do RL no jogo).
    std::string physicalReason;
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
    // (16/11) Na volta pelo cerebro do RL: a diferenca de cada elo entre o corpo e a pose montada das juntas no primeiro tick do guiado (as
    // juntas fisicas cedem milimetros - montada, a pose ficava a ~1 cm dos elos e o guiado a corrigia num tick: a cabeca a 51 m/s2, as
    // maos a 128), somada a pose e sumindo pela mola da volta - o alvo do guiado comeca no corpo. Posicao e rotacao (vetor) por elo,
    // ha quanto (s; < 0 nada) e se falta medir no proximo tick do guiado.
    std::vector<Vec3> rlResidualPosition, rlResidualRotation;
    float rlResidualAge = -1.0f;
    bool rlResidualPending = false;
    // (A rampa da velocidade da volta pelo cerebro: ver AthleteRuntimeSettings3D::rlReturnAcceleration.)
    bool rlRamp = false;
    // (16/11) Na volta pelo cerebro, o padrao ja vem rodando - a sombra que o cerebro levou o corpo a imitar -: a volta nao o recomeca.
    bool rlShadowAdopted = false;
    // (16/11, ele: "a transicao... brusca... principalmente quando o boneco esta virado para outra direcao") Depois da volta pelo cerebro, o
    // rumo pedido ao padrao vai do rumo em que ele assumiu ao da camera (ou do piloto) numa mola com o giro limitado: o rumo e o giro de
    // agora (rad, rad/s).
    bool rlFacingActive = false;
    float rlFacingYaw = 0.0f, rlFacingRate = 0.0f;
    // (A mola desta volta pelo cerebro, dada por ele - curta depois da mistura, que deixa a diferenca pequena; meia-vida e duracao, s; < 0
    // a dos ajustes. A da diferenca dos elos tambem.)
    float rlReturnHalfLife = -1.0f, rlReturnSeconds = -1.0f, rlResidualHalfLife = 0.3f;
    // A velocidade na volta (plano v4, etapa C): parte da do corpo e vai a
    // pedida numa aceleracao suave, por ate returnRampLeft s - a capsula
    // saltava para a pedida e a pose do padrao, presa nela, puxava o corpo.
    Vec3 returnVelocity;
    float returnRampLeft = 0.0f;
    // (Ha quanto a rampa da volta comecou, s: o comeco suave da aceleracao na volta pelo cerebro.)
    float returnRampAge = 0.0f;
    float returnSpeedScale = 1.0f;
    // A ponte fisico -> padrao (plano v2, fase D; o modo Ativo por dentro):
    // o tempo nela e quanto ela dura (s).
    bool bridge = false;
    float bridgeSeconds = 0.0f;
    float bridgeDuration = 0.45f;
    // A ponte nasceu (ou recomecou) de um contato no padrao: a guia nao cai com
    // a ajuda do impacto e parte de bridgeAuthorityFloor, mais baixa quanto
    // mais forte o choque (os contatos inteligentes, 28/10).
    bool contactBridge = false;
    float bridgeAuthorityFloor = 0.15f;
    // A ponte de contato ainda decidindo (s): solta, o choque sendo medido.
    float contactDecisionLeft = 0.0f;
    // O golpe pendente (m/s, no plano): o impulso dos contatos com o que se
    // move sobre a massa, ainda nao absorvido.
    Vec3 contactKick;
    // A capsula virtual do boneco solto no padrao (looseDefaultLocomotion): o
    // estado dela (o centro, como a do jogador) e o quanto ela vai atras do
    // corpo no proximo tick (a do jogador recebe isso de quem a move).
    PhysicsCharacterState3D virtualCapsule;
    bool virtualCapsuleValid = false;
    Vec3 virtualFollowVelocity;
    // O empurrao na capsula virtual (02/11): ao menos a velocidade de quem encosta e se move, na direcao do contato,
    // gasta em VirtualPushSeconds depois, por fora da frenagem dela (sem pedido, ela o apagava no mesmo tick).
    Vec3 virtualPushVelocity;
    // Empurrando outro corpo de pe (0..1; o controlado vai na velocidade dividida com ele).
    float pushingShare = 0.0f;
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
