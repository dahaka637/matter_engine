#pragma once

#include "Engine/Animation/AnimationClip3D.hpp"
#include "Engine/Character/CharacterStateEstimator3D.hpp"
#include "Engine/Locomotion/ContactFootwork3D.hpp"
#include "Engine/Physics/PhysicsScene3D.hpp"

#include <array>
#include <span>
#include <cstdint>
#include <functional>
#include <vector>

namespace MatterEngine {

enum class CharacterLocomotionState3D : std::uint8_t {
    Idle,
    Turning,
    Walking,
    Running,
    CrouchIdle,
    CrouchWalking,
    JumpStarting,
    Airborne,
    Landing,
    // A real collision/stumble that pose authority and joint stiffness
    // could not absorb (see the reaction/interference handling in
    // update()): the guide lets go and physics actually puts the character
    // on the ground. Fallen is the brief "just went down" moment; GettingUp
    // covers every procedural recovery phase after that (see
    // CharacterGetUpPhase3D).
    Fallen,
    GettingUp
};

// Distinguishes which procedural recovery sequence applies: face down needs
// a push up from the arms, face up needs a tuck-and-roll to sitting first.
// Named separately from BiomechanicalBipedExperiment3D's own
// ProceduralFallOrientation3D/ProceduralGetUpPhase3D — both headers can end
// up included in the same translation unit (WorkbenchApp.hpp does), so a
// same-named enum in the same namespace would collide.
enum class CharacterFallOrientation3D : std::uint8_t {
    None,
    FaceUp,
    FaceDown
};

// Getting up is an authored animation, not a procedural sequence: the body
// settles limp on the floor, then the matching stand-up clip plays and takes
// the pose back over. The nine hand-built phases this replaced never
// produced a believable recovery and fought the physics the whole way.
enum class CharacterGetUpPhase3D : std::uint8_t {
    None,
    Settling,
    Rising
};

// Qual pulo esta no ar (escolhido na decolagem).
enum class CharacterJumpKind3D : std::uint8_t {
    None,
    Standing,
    Forward,
    Backward,
    Left,
    Right
};

// Setor do movimento em relacao ao olhar, e o ciclo que o atende.
enum class CharacterGaitDirection3D : std::uint8_t {
    Forward,
    Left,
    Right,
    Backward
};

// Olhar e movimento sao separados: o personagem olha para onde a camera
// aponta. O movimento cai num setor - frente, lado esquerdo, lado direito,
// costas - e cada setor tem o seu ciclo; dentro do setor a pelve gira para o
// movimento (ate ~55 graus) e o tronco devolve o giro para a camera (ver o
// bloco "olhar x movimento" em update()). Diagonal em relacao a pelve nao
// existe numa passada longa - as pernas se cruzariam. Agachar segue
// desativado.
struct CharacterLocomotionAnimations3D {
    // A unica pose parada do personagem.
    const AnimationClip3D* idle = nullptr;
    // Passada para a frente.
    const AnimationClip3D* walk = nullptr;
    // One-shot de transferencia de peso entre o idle e a primeira passada.
    const AnimationClip3D* idleToSprint = nullptr;
    // Referencias de corrida em curva. Sao misturadas na coluna e nas pernas;
    // rumo e translacao continuam sob o controlador fisico.
    const AnimationClip3D* runForwardArcLeft = nullptr;
    const AnimationClip3D* runForwardArcRight = nullptr;
    const AnimationClip3D* runBackwardArcRight = nullptr;
    // Opcional: sem ele, andar para tras da camera gira o corpo inteiro.
    const AnimationClip3D* walkBackward = nullptr;
    // Opcional: sem ele, segurar sprint acelera a caminhada.
    const AnimationClip3D* sprint = nullptr;
    // Opcional: o recuo com sprint. Sem ele, recuando, segurar sprint nao
    // muda nada.
    const AnimationClip3D* sprintBackward = nullptr;
    // Opcionais: strafe, de lado e de frente para a camera. Sem eles, andar
    // de lado vira a pelve para o movimento (setores frente/costas).
    const AnimationClip3D* strafeLeft = nullptr;
    const AnimationClip3D* strafeRight = nullptr;
    const AnimationClip3D* sprintStrafeLeft = nullptr;
    const AnimationClip3D* sprintStrafeRight = nullptr;
    // Pulos (opcionais): o voo, da decolagem (inicio do clipe) ao toque no
    // chao (fim), tocado pela fase do voo. Sem eles, o voo e procedural.
    const AnimationClip3D* jumpStanding = nullptr;
    const AnimationClip3D* jumpForward = nullptr;
    const AnimationClip3D* jumpBackward = nullptr;
    const AnimationClip3D* jumpLeft = nullptr;
    const AnimationClip3D* jumpRight = nullptr;
    // One-shot recoveries, picked by which way the body ended up facing.
    const AnimationClip3D* standUpBack = nullptr;   // spine on the floor
    const AnimationClip3D* standUpFront = nullptr;  // chest on the floor

    [[nodiscard]] bool compatible(const RagdollProfile3D& profile) const;
};

// Velocidades de deslocamento de cada ciclo, lidas dos proprios clipes.
struct CharacterGaitSpeeds3D {
    float walk = 0.0f;
    float walkBackward = 0.0f;
    float sprint = 0.0f;
    // Igual ao recuo quando o personagem nao tem sprint de costas.
    float sprintBackward = 0.0f;
    // Zero quando o personagem nao tem strafe.
    float strafe = 0.0f;
    float sprintStrafe = 0.0f;
};

[[nodiscard]] CharacterGaitSpeeds3D characterGaitSpeeds3D(
    const CharacterLocomotionAnimations3D& animations);

// Velocidade que a capsula deve pedir para um movimento que aponta
// travelRelativeToLookRadians em relacao ao olhar (0 = para onde a camera
// olha): a do ciclo do setor - frente (passada ou sprint), lado (strafe ou
// sprint de lado), costas (recuo ou sprint de costas). Na faixa de histerese entre
// dois setores a velocidade passa de uma para a outra.
[[nodiscard]] float characterGaitSpeed3D(const CharacterGaitSpeeds3D& speeds,
    float travelRelativeToLookRadians, bool sprinting);

// One fixed-step command assembled by gameplay after the persistent capsule
// has resolved collision, slopes, steps, crouch and jump.
struct CharacterLocomotionInput3D {
    Vec3 rootPositionWorld;
    Vec3 rootVelocityWorld;
    // Velocidade que o proxy de navegacao (capsula do jogador, ancora do
    // boneco solto) teve neste tick. NAO e o pedido do jogador: e o que a
    // capsula conseguiu andar. A passada ainda acompanha a capsula ate a
    // reconstrucao inverter a causalidade (etapa E5).
    Vec3 proxyVelocityWorld;
    // O pedido do jogador/IA, puro. A direcao pedida escolhe o setor da
    // passada e o rumo da pelve (pela velocidade da capsula, que gira aos
    // poucos, a pelve passava pela diagonal ao trocar de "frente" para "lado").
    CharacterIntent3D intent;
    float facingYawRadians = 0.0f;
    float lookPitchRadians = 0.0f;
    bool grounded = true;
    bool crouched = false;
    bool sprinting = false;
    bool controlled = false;
    // Experimental floating-base path: solve world foot goals from the
    // measured pelvis and leave the root entirely to finite-force control.
    bool forceDrivenRoot = false;
    // Opcional (modo fisico): forca de cada junta (indice do link, 0..1),
    // vinda do controlador fisico - as juntas perto de uma pancada cedem.
    std::span<const float> jointStrength;
    // Opcional (modo fisico): memoria de contato externo recente (0..1), do
    // controlador fisico - atingido, o corpo reage (perde o equilibrio).
    float externalPerturbation = 0.0f;
    // Opcional (modo fisico, E5): a aceleracao do centro de massa (no plano)
    // que o controlador fisico pediu as pernas no tick anterior - velocidade
    // pedida, troca de peso, equilibrio (requestedComAccelerationWorld). Andando
    // pelo planejador, as pernas de apoio resolvem de uma base adiantada por
    // ela: os motores das juntas seguem o alvo, e o torque de antecipacao
    // sozinho quase nao move o corpo. (A ja limitada pelo apoio inclui a
    // queda do pendulo sobre o pe: adiantar por ela realimentava a queda -
    // de lado, 0,84 m/s pedindo 0,5.)
    Vec3 legBaseAccelerationWorld;
    // E5 (fase 3 do pacote de revisao): as pernas de apoio resolvem de uma
    // referencia corporal com estado (posicao e velocidade no plano, presa a
    // pelve medida por uma coleira) em vez da base prevista pela aceleracao.
    // Comparacao A/B enquanto valida.
    bool bodyReference = false;
    // R4 do pacote v2 (A/B, so com bodyReference): a altura da referencia
    // corporal com estado (velocidade vertical limitada) e o corte da
    // extensao das pernas de apoio perto do alcance.
    bool bodyReachProjection = false;
    // E5 (animacao + pes conscientes): o pe em balanco da passada do clipe
    // pousa deslocado pelo quanto o ponto de captura do corpo real se afastou
    // do da referencia (empurrado, atrasado, passando do ponto).
    bool balanceFootPlacement = false;
    // Variacao de movimento ("vida", 0..2; 0 desliga): pequenas diferencas
    // de passada a passada nos bracos, no tronco e na cabeca; parado, a
    // respiracao, o acomodar dos bracos e o olhar. A semente da a cada
    // personagem o seu jeito (deterministico: mesma semente, mesmo movimento).
    float motionVariety = 0.0f;
    std::uint32_t varietySeed = 0;
    // Pe com mola (modo fisico): no ar o tornozelo leva so o pe e segue o
    // alvo rapido (chega orientado pelo chao de pouso); no toque cede e
    // amortece por ~0,15 s e volta a firmar.
    bool footSpring = true;
    // Tornozelo contra a queda (com footSpring): o pe de apoio pressiona a
    // ponta, o calcanhar ou a borda conforme o corpo sai da velocidade da
    // referencia.
    bool ankleBalance = true;
    // Carregando o pulo (0..1, o pulo sai ao soltar): ele agacha, preparando.
    float jumpCrouch = 0.0f;
    bool manipulated = false;
    std::uint32_t grabbedLink = RagdollDynamics3D::InvalidIndex;
    float poseAuthority = 1.0f;
    float muscleAuthority = 1.0f;
    // Sonda de chao embaixo de cada pe fisico (legado; vale quando groundAt
    // nao existe).
    std::array<GroundProbeResult3D, 2> footGround;
    // Chao em qualquer ponto do mundo (o z e so a altura aproximada de onde
    // procurar). Os pes a consultam onde estao e onde VAO pisar. Sem ela, os
    // pes usam footGround ou o chao da capsula.
    std::function<GroundProbeResult3D(Vec3 pointWorld)> groundAt;
};

struct CharacterLocomotionOutput3D {
    std::vector<RagdollDriveTarget3D> driveTargets;
    RagdollAnimationConstraint3D guide;
    RagdollAnimationPose3D targetPose;
    Vec3 rootControlTorqueWorld;
    // A soft, capture-point-style push toward the support center, active
    // only while pose authority is actually down (a real collision or
    // fall): a lighter version of BiomechanicalBipedExperiment3D's balance
    // wrench, meant to keep the low-authority window believable instead of
    // leaving joint stiffness alone to resist the disturbance.
    Vec3 rootControlForceWorld;
    // Levantar biomecanico: a ajuda no peito (link chestAssistLink), para o
    // tronco subir junto com a pelve - so na pelve, o corpo dobrava de
    // cabeca no chao. Aplicar com applyRagdollControlLinkForce.
    std::uint32_t chestAssistLink = RagdollDynamics3D::InvalidIndex;
    Vec3 chestAssistForceWorld;
    Vec3 chestAssistTorqueWorld;
    // Levantar: o endireitar (pelve e peito) e um par interno - a reacao vai
    // para os segmentos apoiados no chao, divididos pelo apoio de cada um.
    // Aplicar cada torque com applyRagdollControlLinkForce.
    std::vector<std::uint32_t> assistReactionLinks;
    std::vector<Vec3> assistReactionTorquesWorld;
    bool gravityCompensationEnabled = false;
    // 0 = the renderer should show the authored pose untouched; 1 = show the
    // simulated body. Anything between is the blend, and it is the only
    // thing that decides how much physics the player ever sees.
    float physicsBlend = 0.0f;
};

// Diagnostico de um pe (E5-A): as quatro posicoes da origem do link do pe
// num tick - alvo, resultado do IK antes do limitador de velocidade das
// juntas, comando final (depois dos limites) no referencial do IK e o mesmo
// comando a partir da pelve fisica - e o pe fisico, cada uma com a folga da
// sola ao chao (geometria do colisor, na orientacao de cada uma).
// Alcance da perna de apoio (E5, R3 do pacote v2): quadril-tornozelo sobre
// o comprimento da perna esticada, fisico e do alvo (pelve do IK, tornozelo
// alvo), e quanto cada um estica por segundo (+ estica).
struct LegReachTelemetry3D {
    bool valid = false;
    // Quanto a raiz do IK desceu por esta perna neste tick (R4, m; >= 0) e
    // quanto da extensao pedida foi cortada (0..1).
    float verticalCorrection = 0.0f;
    float projectionGain = 0.0f;
    float nominalLengthMeters = 0.0f;
    float physicalRatio = 0.0f;
    float referenceRatio = 0.0f;
    float physicalAxialSpeed = 0.0f;
    float referenceAxialSpeed = 0.0f;
    float ikRootHeight = 0.0f;
    float physicalRootHeight = 0.0f;
};

struct FootTrace3D {
    bool valid = false;
    float groundHeight = 0.0f;
    Vec3 target;
    Vec3 ik;
    Vec3 command;
    Vec3 commandFromPhysicalPelvis;
    Vec3 physical;
    // Referencial em que o IK resolveu a perna e a pelve fisica no mesmo tick.
    Vec3 ikRoot;
    Quaternion ikRootOrientation;
    Vec3 physicalRoot;
    Quaternion physicalRootOrientation;
    float targetClearance = 0.0f;
    float ikClearance = 0.0f;
    float commandClearance = 0.0f;
    float commandFromPhysicalPelvisClearance = 0.0f;
    float physicalClearance = 0.0f;
};

struct CharacterLocomotionTelemetry3D {
    CharacterLocomotionState3D state = CharacterLocomotionState3D::Idle;
    // Ticks com a referencia corporal ativa (E5; R0 do pacote v2: com o
    // seletor desligado tem de ficar em zero).
    std::uint32_t bodyReferenceTicks = 0;
    // Terreno a frente pedindo cuidado (0..1): degrau ou desnivel de 6 cm ou
    // mais nas sondas, ou ladeira; solta em ~0,6 s depois de passar.
    float terrainDemand = 0.0f;
    Vec3 localVelocity;
    float speedMetersPerSecond = 0.0f;
    float facingYawRadians = 0.0f;
    float viewYawErrorRadians = 0.0f;
    float turningRateRadiansPerSecond = 0.0f;
    float forwardLeanRadians = 0.0f;
    float lateralLeanRadians = 0.0f;
    float authoredCurveBlend = 0.0f;
    int authoredCurveKind = 0; // 0 nenhum, 1 frente E, 2 frente D, 3 costas D
    bool authoredStartActive = false;
    float landingBraceForwardRadians = 0.0f;
    float landingBraceLateralRadians = 0.0f;
    float cyclePhase = 0.0f;
    float playbackRate = 1.0f;
    // Passo em relacao ao do clipe (stride warping), inclinacao do chao a
    // frente (sobe/desce por metro) e a velocidade que cabe no terreno (m/s;
    // infinita no plano): o jogo a usa como teto da capsula - na escada o
    // passo encurta e o corpo desacelera junto.
    float strideScale = 1.0f;
    float terrainSlope = 0.0f;
    float terrainSpeedLimit = 1.0e9f;
    // Reacao a freada (0 = nenhuma, ~1 = sprint parando de vez): para onde a
    // inercia leva o tronco, no referencial da pelve (x frente, y esquerda).
    float brakeReaction = 0.0f;
    float brakeReactionForward = 0.0f;
    float forwardWeight = 0.0f;
    float backwardWeight = 0.0f;
    float leftWeight = 0.0f;
    float rightWeight = 0.0f;
    float poseAuthority = 0.0f;
    float rootAuthority = 0.0f;
    float rootRotationAuthority = 0.0f;
    float muscleAuthority = 0.0f;
    float reactionStrength = 0.0f;
    float jointErrorRmsDegrees = 0.0f;
    // Olhar x movimento: o setor da passada, quanto a coluna gira de volta
    // para a camera e quanto a cabeca completa.
    CharacterGaitDirection3D gaitDirection = CharacterGaitDirection3D::Forward;
    // Pulo no ar (ou o ultimo, durante a aterrissagem) e a fase do voo.
    CharacterJumpKind3D jumpKind = CharacterJumpKind3D::None;
    float flightPhase = 0.0f;
    float torsoTwistRadians = 0.0f;
    float headTurnRadians = 0.0f;
    std::array<bool, 2> footPlanted {};
    // Progresso do balanco do clipe (0..1; -1 com o pe travado no chao).
    std::array<float, 2> footSwingProgress { -1.0f, -1.0f };
    // Diagnostico: inclinacao do alvo do pe no mundo (graus, + ponta para
    // baixo) - a do clipe e a pedida ao IK.
    std::array<float, 2> footClipPitch {};
    std::array<float, 2> footGoalPitch {};
    // Tornozelo contra a queda: quanto o pe de apoio pressiona (rad, ponta
    // + / calcanhar -, e de lado), o maior dos dois pes.
    float ankleBalancePitch = 0.0f;
    float ankleBalanceRoll = 0.0f;
    std::array<float, 2> footClearanceMeters {};
    float rootUpright = 1.0f;
    CharacterFallOrientation3D fallOrientation = CharacterFallOrientation3D::None;
    CharacterGetUpPhase3D getUpPhase = CharacterGetUpPhase3D::None;
    float getUpProgress = 0.0f;
    std::uint32_t getUpAttempt = 0;
    // Corpo caindo: quanto os reflexos (maos ao chao, cabeca protegida,
    // tonus) estao agindo, 0..1.
    float fallReflex = 0.0f;
    // Queda sem volta (tombado alem de ~50 graus e ainda tombando): so entao
    // as pernas param de dar passos, cedem e o corpo amolece. Antes disso,
    // desequilibrado, ele continua andando e respondendo aos comandos.
    float fallCommit = 0.0f;
    // Levantar biomecanico: ritmo do clipe (1 = no tempo dele; menos quando o
    // corpo fica para tras), a forca auxiliar na pelve e o "esforco" (sobe
    // enquanto o corpo emperra, e com ele o teto da ajuda).
    float getUpRate = 0.0f;
    float getUpAssistNewtons = 0.0f;
    float getUpStruggle = 0.0f;
    // Caido: o tonus dos musculos (cai quando o corpo para de vez, volta aos
    // poucos antes de levantar) e quanto da ajuda do levantar ja entrou.
    float muscleTone = 1.0f;
    float getUpAssistRamp = 0.0f;
    // Caido: quanto ele ja se arrumou para a pose inicial do levantar (0 =
    // ragdoll com tonus, 1 = pronto para levantar).
    float settleGather = 0.0f;
    // Levantando: quanto a pelve esta alem da inclinacao do clipe (graus) e
    // o torque de endireitar aplicado (N.m) - a ajuda "segurando" o corpo.
    float getUpTiltExcessDegrees = 0.0f;
    float getUpUprightTorque = 0.0f;
    // Levantando: 1 = centro de massa sobre os apoios; a ajuda cai a 0 fora.
    float getUpOverSupport = 1.0f;
    // "Vida" no ragdoll: espernear no ar (0..1).
    float lifeFlail = 0.0f;
    // Perdendo o equilibrio (0..1): bracos em moinho, tronco contra a queda.
    float balanceReaction = 0.0f;
    // Pes reagindo a empurrao/pressao: 1 enquanto se recupera com passos (e
    // ~0,45 s depois) e o ponto de captura fora do apoio real (m).
    float reactiveShare = 0.0f;
    float captureOutsideMeters = 0.0f;
    // O que o planejador recebeu neste tick: parado (dono dos pes), passos
    // permitidos e recuperacao permitida.
    bool footworkStanding = false;
    bool footworkAllowSteps = false;
    bool footworkAllowRecovery = false;
    Vec3 captureDirectionWorld;
    std::array<bool, 2> footworkAnchorUnreachable {};
    // So com setFootTraceEnabled(true) (diagnostico; custa uma cinematica a
    // mais por tick).
    std::array<FootTrace3D, 2> footTrace {};
    // Pernas com o pe no chao (planejador): alcance e extensao axial (R3).
    std::array<LegReachTelemetry3D, 2> legReach {};
    // Forca externa estimada dos contatos (fora os pes no chao), filtrada (N).
    Vec3 externalForceWorld;
};

class CharacterLocomotion3D final {
public:
    void reset(const RagdollProfile3D& profile,
        const RagdollState3D& state);
    void update(const RagdollProfile3D& profile,
        const CharacterLocomotionAnimations3D& animations,
        const RagdollState3D& state,
        const RagdollDynamics3D& dynamics,
        const CharacterLocomotionInput3D& input,
        float deltaTime);

    [[nodiscard]] const CharacterLocomotionOutput3D& output() const {
        return m_output;
    }
    [[nodiscard]] const CharacterLocomotionTelemetry3D& telemetry() const {
        return m_telemetry;
    }
    // Estado fisico medido neste tick (o mesmo que a assistencia e o
    // planejador leem).
    [[nodiscard]] const CharacterPhysicalState3D& physicalState() const {
        return m_estimator.state();
    }
    // Plano oficial dos pes (fase, motivo, passo, pouso) deste tick.
    [[nodiscard]] const CharacterContactPlan3D& contactPlan() const {
        return m_footwork.plan();
    }
    void setFootTraceEnabled(bool enabled) { m_footTraceEnabled = enabled; }
    [[nodiscard]] ContactFootworkSettings3D& footworkSettings() {
        return m_footwork.settings();
    }

private:
    struct FootPlant {
        std::size_t linkIndex = 0;
        bool valid = false;
        bool planted = false;
        Vec3 lockedPositionWorld;
        Vec3 previousAnimatedPositionWorld;
        bool previousAnimatedPositionValid = false;
        // A deliberate turn-in-place repositioning step, distinct from the
        // locomotion foot lock above: a brief, lifted swing from the old
        // stance spot to one rotated to match the new heading, instead of
        // letting IK simply twist the standing leg past what looks natural.
        // A lock does not end by simply vanishing. The locked point drifts
        // away from the authored one while the body turns, so dropping the
        // IK in a single tick teleports the leg back by that whole distance
        // - measured at ~4 m/s on a body turning at 1 rad/s, and worse the
        // faster it turns. This fades the hold out instead.
        float releaseBlend = 0.0f;
        Vec3 releasePositionWorld;
        Quaternion releaseOrientationWorld;
        bool repositioning = false;
        float repositionSeconds = 0.0f;
        float repositionDuration = 0.26f;
        Vec3 repositionFromWorld;
        Quaternion repositionFromOrientation;
        Vec3 repositionToWorld;
        // Passo do giro parado: mira a pose parada ja no rumo final da pelve.
        // O primeiro pe (o do lado do giro) abre girando sobre a bola do pe;
        // o segundo contorna o pe de apoio em arco.
        bool turnStep = false;
        bool pivotStep = false;
        // Passo de recuperacao (empurrao, pressao, tranco): sai cedo, e rapido
        // e baixo, e pousa sob o corpo.
        bool recoveryStep = false;
        Vec3 recoveryTarget;
        // Rumo da pelve quando o passo comecou: o passo mira ate TurnStepLimit
        // alem dele (e nao alem da pelve de agora, que continua girando).
        float turnFromFacing = 0.0f;
        // Terreno: orientacao do pe travado (plano no chao), o chao dele
        // agora, e no balanco o chao de onde saiu e o de onde vai pisar.
        Quaternion lockedOrientationWorld;
        // Onde o pe travado esta de fato (rolando sobre a ponta ou o
        // calcanhar); e dali que ele sai ao soltar a trava.
        Vec3 plantedGoalPosition;
        Quaternion plantedGoalOrientation;
        float groundHeight = 0.0f;
        Vec3 groundNormal { 0.0f, 0.0f, 1.0f };
        float takeoffGroundHeight = 0.0f;
        Vec3 takeoffPosition;
        bool takeoffValid = false;
        // Chao que o pe em balanco precisa passar, com a rampa ate o proximo
        // espelho de degrau (sobe na hora, desce no ritmo do pouso).
        float swingFloor = 0.0f;
        Vec3 swingDirection;
        float landingGroundHeight = 0.0f;
        Vec3 landingNormal { 0.0f, 0.0f, 1.0f };
        // Encaixe do pouso num degrau: quanto o ponto de pouso anda ao longo
        // do pe, para qual pouso previsto isso foi medido, e quanto do
        // deslocamento o pe em balanco ja leva.
        Vec3 fitShift;
        Vec3 fitCenter;
        bool fitValid = false;
        Vec3 activeShift;
        // Distancia entre o pe resolvido (IK e limitador) e o alvo, no tick
        // anterior.
        float reachError = 0.0f;
        // Depois de soltar por falta de alcance, espera antes de travar de
        // novo (s).
        float replantDelay = 0.0f;
        // Parado, soltou e espera o outro pe firmar para dar o passo.
        bool stepPending = false;

    };

    // Toques de cada pe num clipe de passada: quando levanta, quando toca e
    // onde o pe esta no toque (no referencial do clipe), para prever onde ele
    // vai pisar.
    struct FootTouchdown {
        float liftoff = 0.0f;
        float touchdown = 0.0f;
        Vec3 footInClip;
        Vec3 hipInClip;
        Vec3 forwardInClip { 1.0f, 0.0f, 0.0f };
    };
    struct ClipFootEvents {
        const AnimationClip3D* clip = nullptr;
        std::array<std::vector<FootTouchdown>, 2> touchdowns;
    };
    const ClipFootEvents& footEventsFor(const RagdollProfile3D& profile,
        const AnimationClip3D& clip);

    void enterState(CharacterLocomotionState3D state);

    CharacterLocomotionOutput3D m_output;
    CharacterLocomotionTelemetry3D m_telemetry;
    std::vector<Vec3> m_coordinates;
    std::vector<Vec3> m_coordinateVelocities;
    std::vector<Vec3> m_transitionFrom;
    std::vector<Vec3> m_poseVelocities;
    std::vector<Vec3> m_transitionVelocity;
    Vec3 m_lean;
    Vec3 m_leanVelocity;
    // Flexao reativa da coluna contra a inclinacao fisica da pelve. E uma
    // referencia articular suave; nao aplica forca externa nem gira a raiz.
    Vec3 m_spineBalance;
    Vec3 m_spineBalanceVelocity;
    float m_spawnHeightOffset = 0.0f;
    bool m_spawnHeightCaptured = false;
    // Inclinacao da raiz vinda do clipe, com o mesmo crossfade das juntas.
    // Sem ele, trocar de clipe (andar -> parado) fazia o alvo da pelve pular
    // ~17 graus num tick; a velocidade angular derivada chegava a 35 rad/s e
    // o corpo inteiro levava o tranco.
    Quaternion m_rootTilt;
    Quaternion m_transitionFromRootTilt;
    Vec3 m_rootTiltVelocity;
    Vec3 m_transitionRootTiltVelocity;
    // Troca de ciclo sem troca de estado (frente <-> recuo, andar <->
    // sprint) tambem precisa de crossfade: guarda o clipe do tick anterior.
    const AnimationClip3D* m_previousClip = nullptr;
    // Amostra do clipe no tick anterior, e de qual clipe. O limitador de
    // velocidade de junta deixa passar o que o proprio clipe anda - o joelho
    // de um sprint passa de 20 rad/s - e segura o resto (IK, olhar e a
    // mistura do crossfade).
    std::vector<Vec3> m_previousClipCoordinates;
    const AnimationClip3D* m_previousSampledClip = nullptr;
    // Deslocamento da raiz vindo do clipe (balanco e altura), no referencial
    // do clipe, com o mesmo crossfade das juntas.
    Vec3 m_rootOffset;
    Vec3 m_transitionFromRootOffset;
    Vec3 m_rootOffsetVelocity;
    Vec3 m_transitionRootOffsetVelocity;
    // Aterramento suavizado: quanto a pelve sobe para nenhuma sola ficar
    // abaixo do chao.
    float m_groundLift = 0.0f;
    // Terreno: a pelve desce (ou sobe) para o pe mais baixo alcancar o chao
    // dele (mola), e o degrau da capsula entra suavizado.
    float m_terrainSlope = 0.0f;
    float m_pelvisOffset = 0.0f;
    float m_pelvisOffsetVelocity = 0.0f;
    float m_baseStepOffset = 0.0f;
    float m_previousBaseGround = 0.0f;
    bool m_baseTracked = false;
    bool m_baseWasGrounded = false;
    std::vector<ClipFootEvents> m_clipFootEvents;
    CharacterGaitDirection3D m_gaitDirection = CharacterGaitDirection3D::Forward;
    std::array<FootPlant, 2> m_feet;
    Quaternion m_previousRootOrientation;
    Vec3 m_previousRootPosition;
    // Troca de referencial da raiz (entrar/sair do levantar, base que salta
    // num tick): a velocidade da referencia continua a de antes em vez de
    // virar distancia/dt entre as duas ancoras. Ver update().
    bool m_previousRisingPose = false;
    CharacterGetUpPhase3D m_previousReferencePhase = CharacterGetUpPhase3D::None;
    Vec3 m_previousRootBase;
    Vec3 m_previousGuideVelocity;
    Vec3 m_previousGuideAngularVelocity;
    float m_stateSeconds = 0.0f;
    float m_transitionSeconds = 0.0f;
    float m_transitionDurationSeconds = 0.14f;
    float m_cyclePhase = 0.0f;
    bool m_startClipActive = false;
    bool m_startClipSprint = false;
    float m_startClipSeconds = 0.0f;
    float m_authoredCurveBlend = 0.0f;
    int m_authoredCurveKind = 0;
    // Passadas acumuladas: o ruido da variacao de movimento anda por passada.
    float m_varietyStrides = 0.0f;
    // Tornozelo contra a queda: o desvio do ponto de captura pela velocidade
    // (corpo - referencia), filtrado.
    Vec3 m_ankleBalanceShift {};
    float m_idleSeconds = 0.0f;
    float m_facingYaw = 0.0f;
    float m_turningRate = 0.0f;
    float m_reactionStrength = 0.0f;
    float m_physicsBlend = 0.0f;
    float m_physicsHoldSeconds = 0.0f;
    Vec3 m_previousDesiredVelocity;
    // Teto da passada depois de uma pancada/desequilibrio (m/s): o corpo de
    // verdade mais uma folga.
    float m_gaitSpeedCap = 1000.0f;
    Vec3 m_smoothedAcceleration;
    // Freada: direcao do movimento (mundo), velocidade de entrada e a mola
    // da reacao (no referencial da pelve).
    Vec3 m_brakeDirection;
    float m_brakeEntrySpeed = 0.0f;
    Vec3 m_brakeReaction;
    Vec3 m_brakeReactionVelocity;
    Vec3 m_previousCenterOfMass;
    Vec3 m_filteredCenterOfMassVelocity;
    // Olhar: giro da coluna (tronco em relacao a pelve) e da cabeca (em
    // relacao ao tronco). A cabeca acompanha a camera rapido; o tronco vem
    // atras, mais devagar - ver o bloco do olhar em update().
    float m_torsoTwist = 0.0f;
    float m_headTurn = 0.0f;
    float m_filteredLookPitch = 0.0f;
    // Angulo da camera alisado para o olhar (mola), e a pose do clipe do
    // tick anterior, antes do olhar e do IK - de onde parte o crossfade.
    float m_lookYaw = 0.0f;
    float m_lookYawVelocity = 0.0f;
    float m_lookPitch = 0.0f;
    float m_lookPitchVelocity = 0.0f;
    bool m_lookValid = false;
    std::vector<Vec3> m_poseCoordinates;
    // Giro da pelve do proprio clipe em relacao ao rumo (a base em alerta
    // fica com a pelve a ~-22 graus), do tick anterior.
    float m_clipPelvisYaw = 0.0f;
    // Relacao autoral entre a pelve e cada pe no clipe parado. A base em
    // alerta tem os pes bem abertos em relacao a pelve; isso e postura, nao
    // torcao acumulada pelo giro.
    std::array<float, 2> m_clipPelvisToFootYaw {};
    // Giro parado: para onde a pelve vai (o alvo do olhar).
    float m_standingTurnTarget = 0.0f;
    // Jump: captured at the instant the capsule leaves the ground - how fast
    // it was going, how fast it left upward, and which authored jump plays
    // for the whole flight (and its landing).
    float m_liftoffSpeed = 0.0f;
    float m_liftoffVerticalSpeed = 0.0f;
    // Pouso: velocidade de queda do corpo no voo, a do toque e quanto o
    // pouso dura (cresce com o impacto).
    float m_airborneFallSpeed = 0.0f;
    float m_landingImpactSpeed = 0.0f;
    float m_landingSettleSeconds = 0.18f;
    float m_landingSinkPeak = 0.0f;
    // Desde o ultimo toque no chao depois de um voo (s).
    float m_sinceTouchdownSeconds = 10.0f;
    CharacterJumpKind3D m_jumpKind = CharacterJumpKind3D::None;
    const AnimationClip3D* m_jumpClip = nullptr;
    float m_flightPhase = 0.0f;
    // Fall/get-up: link indices cached once for the contact scan that
    // decides whether the character is actually down (a hand/knee/torso
    // touching the ground), independent of the authored-clip foot contact
    // weights the normal locomotion foot lock above already uses.
    std::array<std::size_t, 2> m_handLinks {};
    std::array<std::size_t, 2> m_shinLinks {};
    std::size_t m_chestLink = 0;
    float m_elapsedSeconds = 0.0f;
    CharacterGetUpPhase3D m_getUpPhase = CharacterGetUpPhase3D::None;
    CharacterFallOrientation3D m_fallOrientation =
        CharacterFallOrientation3D::None;
    float m_getUpPhaseSeconds = 0.0f;
    // Playback head for the stand-up clip, plus where on the floor it plays:
    // the body is lying somewhere the capsule is not, and the recovery has to
    // start from the body.
    float m_getUpClipSeconds = 0.0f;
    Vec3 m_getUpAnchorWorld;
    float m_getUpAnchorYaw = 0.0f;
    float m_recoveredStandingSeconds = 0.0f;
    std::uint32_t m_getUpAttempt = 0;
    // Reflexos de queda e levantar biomecanico (ver update()).
    float m_fallReflex = 0.0f;
    float m_fallCommit = 0.0f;
    float m_previousTilt = 0.0f;
    Vec3 m_fallDirection { 1.0f, 0.0f, 0.0f };
    // Tropeco durante uma passada: contato inesperado do pe no meio do
    // balanco dispara uma janela curta de recuperacao. Cedo, o mesmo pe
    // sobe e continua; tarde, ele pousa e o passo seguinte amplia a base.
    float m_tripRecovery = 0.0f;
    float m_tripRecoverySeconds = 0.0f;
    int m_tripFoot = -1;
    bool m_tripElevating = false;
    Vec3 m_tripDirection { 1.0f, 0.0f, 0.0f };
    float m_getUpRate = 1.0f;
    float m_getUpStruggle = 0.0f;
    // Deitado: tonus (0..1) e ha quanto tempo o corpo esta parado.
    float m_settleTone = 1.0f;
    float m_settleRestSeconds = 0.0f;
    // Caido: alvo das juntas que segue o proprio corpo (ragdoll com tonus),
    // de onde ele se arruma para a pose inicial do levantar.
    std::vector<Vec3> m_settlePose;
    bool m_settlePoseValid = false;
    float m_settleGather = 0.0f;
    bool m_plantFeetFromBody = false;
    float m_getUpAssistRamp = 0.0f;
    // Desde o fim do levantar (s): a base firma antes de girar.
    float m_sinceGetUpSeconds = 10.0f;
    // Sem tocar nada ha quanto tempo (s): o espernear no ar.
    float m_noGroundSeconds = 0.0f;
    // Reacao de quem perde o equilibrio (ver applyBalanceReaction).
    Vec3 m_previousTargetUp { 0.0f, 0.0f, 1.0f };
    float m_balanceReaction = 0.0f;
    // Pes reagindo (ver update(), "Sensor de equilibrio"): se esta se
    // recuperando (0/1, telemetria) e a forca externa filtrada (N, no plano).
    float m_reactiveShare = 0.0f;
    // Estado fisico comum (centro de massa, apoio, captura, forca externa),
    // medido uma vez por tick no comeco de update().
    CharacterStateEstimator3D m_estimator;
    // Juntas medidas ao comecar o levantar: os alvos saem delas (ver
    // update(), "Levantar a partir de onde o corpo esta").
    std::vector<Vec3> m_risingFromMeasured;
    // Ponto de captura (mundo), quanto sai do apoio (m) e para onde
    // (unitario, do centro do apoio).
    Vec3 m_capturePoint;
    float m_captureOutside = 0.0f;
    Vec3 m_captureDirection;
    // Planejador unico de passos: o dono do estado oficial dos pes.
    ContactFootwork3D m_footwork;
    // Por pe, quanto o IK da perna parte da pelve MEDIDA (0 a 1): o pe que o
    // planejador tem no ar vai ao alvo no mundo mesmo com o corpo inclinado.
    std::array<float, 2> m_swingFromMeasuredPelvis {};
    // Comprimentos quadril-tornozelo do tick anterior (fisico, alvo) para a
    // extensao axial (R3); negativo = sem medida.
    std::array<std::array<float, 2>, 2> m_previousLegLength { { { -1.0f, -1.0f }, { -1.0f, -1.0f } } };
    // E5: quanto as pernas de apoio resolvem pela pelve medida (no plano).
    float m_legRootFromMeasured = 0.0f;
    // Referencia corporal virtual (E5): nao escreve nada na raiz - so e de
    // onde as pernas de apoio resolvem o IK (os musculos tentam levar a
    // pelve ate ela; o corpo pode atrasar, e a coleira o limita).
    Vec3 m_bodyRefPosition;
    Vec3 m_bodyRefVelocity;
    bool m_bodyRefValid = false;
    // Altura da referencia corporal (R4 do pacote v2), com estado: segue a
    // altura da pelve de referencia (clipe/capsula) com velocidade vertical
    // limitada, e desce na hora quando o alcance de uma perna de apoio pede.
    float m_bodyRefHeight = 0.0f;
    // Deslocamento do pouso pelo equilibrio (E5, animacao), filtrado.
    Vec3 m_balancePlacementShift;
    // Terreno a frente classificado como rampa (superficie inclinada), com
    // uma folga de 0,3 s.
    bool m_terrainIsRamp = false;
    float m_terrainRampHold = 0.0f;
    // Teto de velocidade do terreno publicado (sobe aos poucos, desce na hora).
    float m_terrainSpeedCap = 9.0f;
    // O corpo sobre a escada ou rampa (pes em alturas diferentes ou chao
    // inclinado embaixo deles), retido por 0,6 s.
    float m_onInclineHold = 0.0f;
    float m_terrainDemand = 0.0f;
    float m_sustainedTerrainHold = 0.0f;
    bool m_footTraceEnabled = false;
    float m_jumpCrouch = 0.0f;
    Vec3 m_balanceDirection { 1.0f, 0.0f, 0.0f };
    float m_getUpStallSeconds = 0.0f;
    float m_getUpHoldSeconds = 0.0f;
    float m_getUpSupportAge = 1.0f;
    float m_getUpExpectedHeight = 0.0f;
    float m_getUpExpectedTilt = 0.0f;
    float m_getUpExpectedChestHeight = 0.0f;
    bool m_getUpExpectedValid = false;
    bool m_previousGrounded = true;
    bool m_initialized = false;
};

} // namespace MatterEngine
