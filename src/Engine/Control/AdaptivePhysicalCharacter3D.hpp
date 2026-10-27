#pragma once

#include "Engine/Animation/AnimationClip3D.hpp"
#include "Engine/Character/CharacterControlTypes3D.hpp"
#include "Engine/Control/CharacterWholeBody3D.hpp"
#include "Engine/Locomotion/ContactFootwork3D.hpp"

namespace MatterEngine {

// Brake into a sharp direction change before accelerating the new stride.
// Shared by the application and physics scenario tests.
// accelerationLimit (m/s2, 0 = sem): a capsula acelera, freia e vira no
// maximo isto (locomocao fisica v2: 10, o que o corpo consegue pelo chao - a
// 14-18 o zigue-zague pedia ~2 g de lado e a ajuda arrastava o corpo).
CharacterMotorSettings3D adaptiveCharacterMotorSettings3D(
    const CharacterMotorSettings3D& settings, const PhysicsCharacterState3D& state,
    const CharacterMotorCommand3D& command, float accelerationLimit = 0.0f);
// Locomocao fisica v2: o limite de aceleracao da capsula (ver acima).
constexpr float PhysicalV2NavigationAcceleration = 10.0f;
// ... pela ajuda andando: o que o corpo consegue depende de quem o leva (com
// pouca ajuda, as pernas): 4 m/s2 sem ajuda, 10 com ela inteira (4 + 6 x
// ajuda). A 30%, com os 10, o zigue-zague e o para-e-arranca caiam.
float physicalNavigationAcceleration3D(float walkingAssist);

// Locomocao fisica v2: a velocidade pedida a capsula no rumo pedido fica no
// maximo `lead` acima da do corpo nesse rumo (so ganhar velocidade: frear e
// virar seguem livres), no chao. Com pouca ajuda quem leva o corpo sao as
// pernas: a capsula a 7,5 m/s com o corpo a 1 m/s ia metros a frente, a
// passada do sprint pedia a perna esticada para uma pelve que nao existia e
// ele caia. A folga cresce com a ajuda andando (com ajuda alta a pelve leva o
// corpo junto e quase nunca chega a ela). Devolve o speedScale.
float physicalNavigationLead3D(float walkingAssist);
float physicalNavigationSpeedScale3D(Vec3 moveDirection, float speedScale, float nominalSpeed,
    Vec3 bodyVelocity, float lead);
// Locomocao fisica v2: virando forte, a velocidade cai como a de quem corre
// cortando - pedindo um rumo longe do atual, ate ~35% para cortar, e numa
// curva continua a/omega (a aceleracao de lado que o corpo consegue pela taxa
// de giro pedida). Pela parte das pernas (1 - ajuda): com ajuda alta a pelve
// leva o corpo na velocidade inteira. Devolve o fator (0..1).
float physicalNavigationTurnScale3D(Vec3 currentVelocity, Vec3 moveDirection, float turnRate,
    float walkingAssist);

enum class PhysicalControlPhase3D { Stable, Perturbed, Recovering, Airborne, Falling };

struct AdaptivePhysicalIntent3D {
    // Velocidade do proxy de navegacao neste tick (nao o pedido do jogador).
    Vec3 proxyVelocityWorld;
    Vec3 navigationRootWorld;
    bool airborne = false;
    bool manipulated = false;
    // Caido ou levantando: o levantar e um clipe com trilha de raiz; nada
    // desta assistencia age.
    bool suspended = false;
    // Postura perdida sem volta (da locomocao: telemetry().postureLost): cai
    // de vez - a ajuda sai neste tick, em vez de segurar o corpo torto.
    bool postureLost = false;
    // v2: apoiado no alto (mureta, parede) tentando se recompor (da
    // locomocao: telemetry().braceRecovering) - o tronco apoiado nao e
    // "caido" nem "sem volta".
    bool braceRecovering = false;
    // Carga do pulo em curso (0 = toque, 1 = cheia): a janela de impulso na
    // decolagem se estende com ela (a capsula sai mais rapido).
    float jumpCharge = 0.0f;
    std::array<GroundProbeResult3D, 2> footGround;
    // Velocidade que a capsula (ou ancora) recebeu neste tick para ir atras
    // do corpo. Nao e intencao do jogador: fica fora da aceleracao
    // antecipada.
    Vec3 followVelocityWorld;
    // Velocidade pedida pelo jogador (no plano) - a que as pernas seguem
    // andando no modo em que elas carregam o corpo (E5).
    Vec3 requestedVelocityWorld;
    // Freada correndo (da locomocao: telemetry().brakeRunVelocityWorld e
    // brakeRunShare): o corpo ainda corre e a passada de corrida freia. Com
    // a parcela acima de zero, as pernas seguem a velocidade do perfil de
    // freada como seguem a pedida andando, e a ajuda vai da de parado
    // (legsAssistRetained) a de andando (walkingAssistScale) pela parcela.
    Vec3 brakeRunVelocityWorld;
    float brakeRunShare = 0.0f;
    // Estado fisico comum deste tick (CharacterStateEstimator3D, publicado
    // pela locomocao). Nulo: calcula o centro de massa sozinho.
    const CharacterPhysicalState3D* physicalState = nullptr;
    // Plano oficial dos pes (ContactFootwork3D, publicado pela locomocao):
    // quais pes estao em apoio para carregar o peso pelas juntas.
    const CharacterContactPlan3D* contactPlan = nullptr;
    // Pose alvo da locomocao (no mundo, CharacterLocomotionOutput3D::
    // targetPose): a ajuda em todos os ossos segue as orientacoes dela.
    const RagdollAnimationPose3D* targetPose = nullptr;
    // Pe plantado pela locomocao (telemetry().footPlanted; sem o dado,
    // vale o contato): o pe da passada no ar, mesmo raspando o chao, recebe
    // a ajuda do pe.
    bool footPlantedValid = false;
    std::array<bool, 2> footPlanted {};
    // O RL (Athlete/Physical/PhysicalCorrections3D): a aceleracao a mais
    // pedida as pernas pelo contato (no mundo, no plano), so com `rlContact`.
    // Entra na parte das pernas e passa pelo mesmo alocador (centro de
    // pressao nos pes, atrito): o chao limita.
    bool rlContact = false;
    Vec3 rlContactAccelerationWorld;
};

struct AdaptivePhysicalSettings3D {
    // Rastreamento da pelve de referencia (rad/s), com a aceleracao da
    // referencia antecipada. Com 8 rad/s uma arrancada de 14 m/s2 deixava
    // 22 cm de atraso.
    float planarFrequency = 14.0f;
    float heightFrequency = 12.0f;
    // Limites do que o corpo consegue pelo chao. Horizontal: o atrito dos
    // pes (~1,25 g, um pouco acima do humano pela responsividade). Torque
    // para endireitar: tornozelo e quadril, ~2 N.m/kg (140 N.m), e so
    // enquanto o centro de massa projetado esta dentro da base dos pes em
    // contato - fora dela o chao nao segura e o corpo tomba, a menos que um
    // pe chegue la (passo).
    float frictionAccelerationLimit = 12.3f;
    float balanceTorquePerKg = 2.0f;
    // Distancia (m) da projecao do centro de massa ate a borda da base:
    // torque inteiro com a projecao InsideMargin para dentro, nenhum com
    // OutsideMargin para fora.
    float balanceInsideMargin = 0.04f;
    float balanceOutsideMargin = 0.04f;
    // Tombando: a sustentacao e a correcao de desvio na pelve so existem com
    // o corpo em pe - uma perna so sustenta o corpo quando esta embaixo
    // dele. Inteiras ate este erro de inclinacao contra a referencia (graus),
    // zero em Full.
    float uprightAssistStartDegrees = 20.0f;
    float uprightAssistEndDegrees = 45.0f;
    // Recem-levantado (fim de `suspended`): a postura aguenta mais
    // inclinacao e mais torque, e o reforco diminui aos poucos nestes
    // segundos - a ajuda do levantar nao some de uma vez.
    float recoveryBoostSeconds = 2.0f;
    // Envelope da ajuda: um contato externo tira ate assistContactDrop dela
    // na hora; volta a assistRecoveryPerSecond (por segundo, de 0 a 1).
    float assistContactDrop = 0.5f;
    float assistRecoveryPerSecond = 0.7f;
    // Pe conta na base por este tempo depois do ultimo contato.
    float footSupportMemorySeconds = 0.10f;
    // Apoio pelos pes: a assistencia vale inteira ate este tempo sem contato
    // (atravessa o voo da corrida, ~0,13 s) e some no mesmo tanto depois.
    float supportMemorySeconds = 0.25f;
    float maximumFeedforwardAcceleration = 30.0f;
    float maximumPlanarAcceleration = 24.0f;
    float maximumSupportAcceleration = 22.0f;
    // Pulo: a ajuda vertical so age numa janela de impulso na decolagem (ate
    // o corpo subir na velocidade da capsula); depois o voo e balistico.
    // Antes a memoria de apoio (0,25 s) seguia erguendo o corpo com os pes
    // ja no ar - a pelve subia 1,3 m num pulo em sprint.
    float jumpPushSeconds = 0.18f;
    float maximumJumpAcceleration = 40.0f;
    float jumpHoldPushSeconds = 0.14f;
    // Pulo carregado: o impulso tambem fica mais forte (o corpo acompanha a
    // saida mais rapida da capsula).
    float jumpChargeExtraAcceleration = 12.0f;

    float maximumReboundDampingAcceleration = 9.81f;
    // Leave most weight on the real feet; full mg at the pelvis unloads
    // support and makes contact-based balance flicker even in a quiet idle.
    float gravityAssistFraction = 0.25f;
    // Etapa E4 (migracao): o peso vai pelas pernas - forca do chao desejada
    // nos pes de apoio, transmitida por torque nas juntas do tornozelo,
    // joelho e quadril (CharacterWholeBody3D) - e a fracao de gravidade sai
    // da pelve. Desligado: a sustentacao antiga (comparacao). Em
    // desenvolvimento - ainda desligado por padrao.
    bool jointSupport = false;
    // Parado: tambem o equilibrio planar (correcao e intencao, pelo centro de
    // pressao) e a postura (tornozelos) pelas pernas, no lugar da pelve.
    bool jointBalance = false;
    bool jointPosture = false;
    // Experimento E5: as pernas carregam o corpo tambem andando (peso,
    // equilibrio/propulsao e postura pelos pes de apoio da passada).
    bool jointWhileWalking = false;
    // Quanto da ajuda da pelve fica quando as pernas assumem peso,
    // equilibrio e postura (0 = a E4 pura, a pelve sem nada; 0,3 = as pernas
    // fazem 70% e a pelve segue com 30%, discreta).
    float legsAssistRetained = 0.0f;
    // Andando pela passada (os pes com a animacao): envelope unico da
    // propulsao, correcao de posicao e postura na pelve.
    float walkingAssistScale = 1.0f;
    // A ajuda andando no PESO (a sustentacao vertical), sem a queda do impacto:
    // o impacto tira o empurrao de fora (a pancada nao e combatida por uma mao
    // invisivel), nao a sustentacao que o segura em pe cambaleando. < 0: a de
    // cima.
    float walkingWeightAssistScale = -1.0f;
    // A ajuda parada escolhida, tambem sem a queda do impacto (a do par de
    // postura abaixo). < 0: legsAssistRetained.
    float standingWeightAssistScale = -1.0f;
    // O par de postura na pelve (pelve x pes/maos, o "musculo virtual" que
    // passa por cima das juntas) segue a regua escolhida - sem a queda do
    // impacto: a 0%, a postura so pelas juntas (as pernas); a 100%, inteiro.
    // false (sozinho, sem quem saiba a regua): o regime A (classico), o par
    // sempre inteiro; o atleta liga com a regua no comando. Vault
    // 04_locomocao/ajuda_auxiliar.md.
    bool postureCoupleFollowsAssist = false;
    // A regua historica (21/10, plano 04_locomocao/plano_rl_sobre_classico.md
    // B): a lei de ajuda de antes de 5f2c4b6 (o "zero antigo": a 0% a pelve
    // ainda leva parte do peso) no motor de hoje - o reforco por max, o peso
    // e a sustentacao da pelve pela escala depois do impacto, a forma pulsada
    // so acima de 50%, a alavanca e o pulo inteiros, o par sempre inteiro
    // (regime A, ignora postureCoupleFollowsAssist). false: a regua linear.
    bool historicalAssistRuler = false;
    // Quanto do PESO as pernas assumem quando carregam (a parte da fracao
    // delas; 1 = a E4). Com 0 o peso fica com a pelve e os motores, e as
    // pernas fazem so o equilibrio (centro de pressao) e a postura: o peso
    // pelas juntas somado a sustentacao da pelve fazia o corpo tremer parado
    // (oscilacao vertical de ~7 Hz, os pes perdendo o chao).
    float legsWeightFraction = 1.0f;
    // Correcao de posicao e mola de altura pelas pernas (rad/s).
    float legPlanarFrequency = 4.0f;
    float legHeightFrequency = 6.0f;
    // Torque de postura (par pelve x pes, ~hip+tornozelo).
    float maximumTiltTorquePerKg = 4.0f;
    // Onde vai a correcao de desvio: 0 nenhuma (so as pernas, pelos motores
    // com o IK na pelve desejada), 1 na pelve, 2 nos pes em contato (NAO usar:
    // forca externa no pe nao e atrito, so desliza o pe), 3 na pelve com a
    // alavanca do chao depois de contato externo (o escolhido; ver o diario).
    int correctionMode = 3;
    // Postura: 0 nenhuma (so as juntas: nao fica em pe), 1 torque na pelve,
    // 2 par pelve x pes (o escolhido).
    int postureMode = 2;
    float maximumYawTorquePerKg = 3.0f;
    // No ar, um par interno pelve x pernas endireita o corpo (como quem mexe
    // as pernas para girar o tronco: conserva o momento angular). Era 0,5
    // sem uso; com 1,5 ele chega ao chao reto num pulo em sprint.
    float maximumAirTorquePerKg = 1.0f;
    float proxyFollowRate = 8.0f;
    float maximumProxyFollowSpeed = 4.0f;
    // Locomocao fisica v2 (06/10/2026): o voo de um pulo fica fora da fracao
    // em contato (o reforco da ajuda no apoio ia ao maximo no pouso: ~4 g na
    // pelve com os pes plantados) e o torque de postura (par pelve x pes, um
    // esforco interno - quadril e tornozelo) vai a v2TiltTorquePerKg: com 4
    // o tronco nao segurava a aceleracao e o corpo mergulhava para a frente
    // com os pes atras (a 70%: 12 -> 3 rodadas com queda em 174).
    // Andando, o par de postura tambem nao encolhe com a ajuda (e esforco
    // interno): escalado por ela, a 0% ninguem segurava o tronco - na
    // bateria, 30/15/0%: 16/28/83 -> 5/8/9 rodadas com queda.
    bool v2 = false;
    float v2TiltTorquePerKg = 8.0f;
};

struct AdaptivePhysicalOutput3D {
    bool valid = false;
    PhysicalControlPhase3D phase = PhysicalControlPhase3D::Airborne;
    // Pelve: intencao do jogador, sustentacao e o torque de postura.
    Vec3 rootForceWorld;
    Vec3 rootTorqueWorld;
    // Pes: a correcao de desvio (so nos que tocam algo) e a reacao do torque
    // de postura. Aplicar tudo com applyRagdollControlLinkForce.
    std::array<std::uint32_t, 2> footLinkIndex { RagdollDynamics3D::InvalidIndex,
        RagdollDynamics3D::InvalidIndex };
    std::array<Vec3, 2> footForceWorld {};
    std::array<Vec3, 2> footTorqueWorld {};
    // Mao (ou antebraco) apoiada num obstaculo: parte da reacao da postura
    // vai para ela - o corpo empurra o objeto para se endireitar.
    std::array<std::uint32_t, 2> handLinkIndex { RagdollDynamics3D::InvalidIndex,
        RagdollDynamics3D::InvalidIndex };
    std::array<Vec3, 2> handTorqueWorld {};
    Vec3 requestedForceWorld;
    Vec3 proxyFollowVelocityWorld;
    Vec3 centerOfMassWorld;
    Vec3 centerOfMassVelocityWorld;
    float captureErrorMeters = 0.0f;
    float bodyTiltRadians = 0.0f;
    float stability = 0.0f;
    float recoverability = 0.0f;
    float locomotionAuthority = 0.0f;
    float postureAuthority = 1.0f;
    float uprightAuthority = 0.0f;
    float supportAcceleration = 0.0f;
    // 1 com apoio recente dos pes, 0 no ar ha mais de 2x a memoria.
    float supportAuthority = 0.0f;
    // Projecao do centro de massa contra a base dos pes em contato (+ dentro,
    // - fora, m) e a fracao do torque de equilibrio que isso permite.
    float balanceMargin = 0.0f;
    float balanceAuthority = 0.0f;
    // Memoria de contato externo (0..1) e o torque de alavanca aplicado.
    float perturbation = 0.0f;
    Vec3 correctionLeverWorld;
    // Erro de inclinacao da pelve contra a referencia (graus) e a fracao da
    // sustentacao/correcao que isso ainda permite.
    float tiltErrorDegrees = 0.0f;
    float uprightAuthorityShare = 1.0f;
    // Quanto da ajuda (postura e correcao) esta valendo depois de uma pancada
    // ou desequilibrio: cai na hora e volta aos poucos (0..1).
    float assistEnvelope = 1.0f;
    // Reforco de recem-levantado (1 na troca, 0 depois de recoveryBoostSeconds).
    float recoveryBoost = 0.0f;
    float secondsWithoutFootContact = 0.0f;
    float proxyErrorMeters = 0.0f;
    int supportCount = 0;
    std::uint32_t strongestInteractionLink = RagdollDynamics3D::InvalidIndex;
    float strongestImpulse = 0.0f;
    bool interactionImpulseEstimated = false;
    std::vector<float> jointStrength;
    // Torque de antecipacao por junta (link, eixos do frame articular) que
    // leva o esforco de contato desejado dos pes ate a pelve, e esse esforco
    // (chao sobre o pe) em cada pe de apoio.
    std::vector<std::array<float, 3>> jointFeedforward;
    // Ajuda em todos os ossos: torque (mundo) por link, no indice do perfil
    // (aplicar com applyRagdollControlLinkForce).
    std::vector<Vec3> linkTorqueWorld;
    std::array<ContactWrench3D, 2> contactWrench {};
    float jointSupportNewtons = 0.0f;
    // Ha torque de antecipacao nas juntas para aplicar (apoio ou, no ar, a
    // postura pelos quadris).
    bool jointFeedforwardActive = false;
    // Aceleracao do centro de massa (E5, semantica do pacote de revisao):
    // PEDIDA as pernas (no plano: velocidade pedida andando, troca de peso,
    // equilibrio - antes do limite do apoio); ALOCADA (a que o esforco de
    // contato calculado daria, depois do poligono de apoio e do atrito -
    // nao e a alcancada); OBSERVADA (do estimador, filtrada); e o residuo
    // pedida - alocada. O esforco alocado e antecipacao/diagnostico: quem
    // move o corpo sao os alvos das juntas.
    Vec3 requestedComAccelerationWorld;
    Vec3 allocatedComAccelerationWorld;
    Vec3 observedComAccelerationWorld;
    Vec3 allocationResidualWorld;
    // Quanto do peso e do equilibrio as pernas carregam agora (0 a 1).
    float legShare = 0.0f;
    float balanceShare = 0.0f;
    // Postura: o torque pedido para as pernas e o que os pes de apoio levam
    // (depois dos limites da sola), no mundo.
    Vec3 legPostureRequested;
    Vec3 legPostureDelivered;
};

// Computes a finite, assisted floating-base wrench. Never writes transforms
// and never decides a fall from an impact threshold. It can run in shadow mode
// beside the existing guide so budgets can be measured before migration.
class AdaptivePhysicalCharacter3D {
public:
    void reset(const RagdollProfile3D& profile);
    void update(const RagdollProfile3D& profile, const RagdollState3D& state,
        const RagdollAnimationConstraint3D& reference,
        const AdaptivePhysicalIntent3D& intent, float dt);
    const AdaptivePhysicalOutput3D& output() const { return m_output; }
    AdaptivePhysicalSettings3D& settings() { return m_settings; }
    [[nodiscard]] const AdaptivePhysicalSettings3D& settings() const { return m_settings; }
private:
    AdaptivePhysicalSettings3D m_settings;
    AdaptivePhysicalOutput3D m_output;
    CharacterWholeBody3D m_wholeBody;
    // Quanto do peso, do equilibrio planar e da postura as pernas carregam
    // (E4, 0 a 1, gradual).
    float m_legShare = 0.0f;
    float m_balanceShare = 0.0f;
    float m_postureShare = 0.0f;
    // Andando: quanto da postura a perna de apoio segura (o resto da ajuda).
    float m_walkingPostureShare = 0.0f;
    float m_apcDebugGate = 1.0f;
    // Andando: a parte do peso que as pernas carregam sempre (o piso).
    float m_walkCarryShare = 0.0f;
    // Andando: o quanto a ajuda no plano vale agora (pulsa com as pernas).
    float m_legPulse = 1.0f;
    // Andando: algum pe no chao agora (suavizado) - a ajuda age com ele.
    float m_contactGate = 1.0f;
    float m_contactDuty = 1.0f;
    float m_contactTurning = 0.0f;
    std::vector<float> m_compliance;
    // Ajuda em todos os ossos: a pose alvo do tick anterior (velocidade
    // angular do alvo) e a inercia aproximada de cada link.
    // Ajuda cheia: a integral do erro de altura (m.s).
    float m_failureSeconds = 0.0f;
    float m_falling = 0.0f;
    bool m_wasSuspended = false;
    float m_sinceAirborne = 10.0f;
    bool m_wasAirborne = false;
    // 1 = inclinacao so do proprio movimento (sem contato externo recente,
    // jogador se movendo): a postura aguenta mais.
    float m_selfMotionCalm = 0.0f;
public:
    // Diagnostico: o movimento proprio (0..1) do ultimo tick.
    float selfMotionCalm() const { return m_selfMotionCalm; }
private:
    float m_assistEnvelope = 1.0f;
    // Desde o ultimo contato externo de verdade (s): depois de uma batida, a
    // fisica decide o tropeco por um tempo (sem endireitar no ar nem postura
    // reforcada).
    float m_sinceExternalHit = 10.0f;
    float m_jumpPushElapsed = -1.0f;
    float m_sinceSuspended = 10.0f;
    float m_perturbation = 0.0f;
    float m_supportDuty = 1.0f;
    float m_secondsWithoutFootContact = 1.0f;
    // Sem nenhum pe apoiado por contato real (s): parte do criterio de queda.
    float m_secondsWithoutSupport = 0.0f;
    std::array<float, 2> m_footContactAge { 1.0f, 1.0f };
    Vec3 m_previousReferenceVelocity;
    bool m_referenceVelocityValid = false;
};
} // namespace MatterEngine
