#pragma once

#include "Engine/Athlete/Physical/PhysicalCorrections3D.hpp"
#include "Engine/Animation/AnimationClip3D.hpp"
#include "Engine/Character/BallControl3D.hpp"
#include "Engine/Character/BodilyGetUp3D.hpp"
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
    GettingUp,
    // Carrinho: da corrida ao chao deslizando (um dos clipes de carrinho) e
    // de volta em pe ou correndo. O corpo segue o clipe como no levantar; o
    // deslize e o embalo da corrida e o atrito, com uma ajuda que acompanha
    // o perfil de velocidade do clipe.
    Tackling
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
    // Arrancada agachada (o levantar com pressa no modo padrao).
    const AnimationClip3D* crouchToSprint = nullptr;
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
    // Opcionais, so da locomocao padrao: as diagonais (trote e sprint, para a
    // frente e de costas). Sem elas a diagonal usa o clipe da frente ou o de
    // costas com a pelve girada para o movimento.
    const AnimationClip3D* walkForwardLeft = nullptr;
    const AnimationClip3D* walkForwardRight = nullptr;
    const AnimationClip3D* walkBackwardLeft = nullptr;
    const AnimationClip3D* walkBackwardRight = nullptr;
    const AnimationClip3D* sprintForwardLeft = nullptr;
    const AnimationClip3D* sprintForwardRight = nullptr;
    const AnimationClip3D* sprintBackwardLeft = nullptr;
    const AnimationClip3D* sprintBackwardRight = nullptr;
    // Pulos (opcionais): o voo, da decolagem (inicio do clipe) ao toque no
    // chao (fim), tocado pela fase do voo. Sem eles, o voo e procedural.
    const AnimationClip3D* jumpStanding = nullptr;
    const AnimationClip3D* jumpForward = nullptr;
    // Preparo do pulo parado: o agachamento antes do impulso, percorrido pela
    // carga.
    const AnimationClip3D* jumpPrep = nullptr;
    const AnimationClip3D* jumpBackward = nullptr;
    const AnimationClip3D* jumpLeft = nullptr;
    const AnimationClip3D* jumpRight = nullptr;
    // One-shot recoveries, picked by which way the body ended up facing.
    const AnimationClip3D* standUpBack = nullptr;   // spine on the floor
    const AnimationClip3D* standUpFront = nullptr;  // chest on the floor
    // Giro parado (opcionais, ajuda cheia): ~86 graus da base parada a base
    // parada girada, tocados na velocidade do giro pedido (pernas e pes).
    const AnimationClip3D* turnLeft = nullptr;
    const AnimationClip3D* turnRight = nullptr;
    // Carrinho (opcionais): da corrida ao chao deslizando e de volta em pe
    // (ou correndo). Cada um comeca firmando um pe (o motor le no primeiro
    // quadro); o pe que vai pousar quando o jogador pede escolhe qual.
    std::array<const AnimationClip3D*, 2> tackles {};

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
// O corpo do chute (bracos e tronco pela forca) e a "vida" do movimento
// (variacao de passada a passada; parado, a respiracao e o acomodar) sobre
// coordenadas de junta - os mesmos do modo fisico, para a locomocao padrao.
// O pe (origem e orientacao, referencial da raiz) aonde se pede, com o rumo
// dele pela coxa quando holdYaw: a perna inteira gira em volta da linha
// quadril-tornozelo ate o pe apontar para onde deve (o tornozelo nao tem o
// giro vertical) - o mesmo do modo fisico.
void characterSolveFootPose3D(const RagdollProfile3D& profile, std::vector<Vec3>& coordinates,
    std::size_t footIndex, Vec3 targetPosition, Quaternion targetOrientation, float weight, bool holdYaw);
void characterKickPose3D(const RagdollProfile3D& profile, const RagdollState3D& state,
    std::vector<Vec3>& coordinates, std::array<std::size_t, 2> hands, const BallKickPose3D& kick);
void characterGaitVariety3D(const RagdollProfile3D& profile, const AnimationClip3D& clip, float cyclePhase,
    float armPhase, float strides, std::uint32_t seed, float amount, std::vector<Vec3>& coordinates);
void characterIdleVariety3D(const RagdollProfile3D& profile, float seconds, std::uint32_t seed, float amount,
    std::vector<Vec3>& coordinates);

[[nodiscard]] float characterGaitSpeed3D(const CharacterGaitSpeeds3D& speeds,
    float travelRelativeToLookRadians, bool sprinting);

// One fixed-step command assembled by gameplay after the persistent capsule
// has resolved collision, slopes, steps, crouch and jump.
struct CharacterLocomotionInput3D {
    // As correcoes do RL nas costuras do controle (a reescrita: o jogo de pes,
    // o equilibrio, a pose e a rigidez; Athlete/Physical/PhysicalCorrections3D).
    // Nulo (o padrao): o classico, exato.
    const PhysicalCorrections3D* corrections = nullptr;
    Vec3 rootPositionWorld;
    Vec3 rootVelocityWorld;
    // Locomocao fisica v2 (06/10/2026): a postura dada como perdida na
    // passada pelo pe apoiado e pela pelve tombada (nao pelo meio dos pes, com
    // um deles no ar longe), a arrancada so saindo de parado, o ciclo
    // recomecando no pe que pousou de um pulo, e a perna de apoio da corrida
    // estendendo no maximo 6 cm por vez (com o amortecimento vertical sempre).
    bool physicalV2 = false;
    // Quanto da ajuda andando (forca na pelve) o corpo tem, 0..1. Com pouca
    // ajuda a passada fica presa ao corpo e os passos pousam pelo ponto de
    // captura (com a ajuda cheia, quem leva o corpo e a pelve).
    float walkingAssist = 1.0f;
    // E a ajuda parado (0..1): com ela alta o giro parado e quase imediato
    // (a pelve tem forca para girar; os pes pivotam no chao).
    float standingAssist = 1.0f;
    // Ajuda para levantar do chao (0..1): escala as forcas de fora do
    // levantar - a subida na pelve e no peito e o torque de endireitar. 1 e o
    // levantar atual; 0, nenhuma (so os musculos).
    float getUpAssist = 1.0f;
    // Rigidez dos musculos no levantar (0..1+): 1 e a de antes (rigida);
    // menos, mais relaxado (a forca maxima nao muda).
    float getUpStiffness = 1.0f;
    // Forca dos musculos ao levantar (o torque maximo; 1 a de antes).
    float getUpStrength = 1.0f;
    // Pressa para levantar alem das teclas de andar (no laboratorio, o
    // espaco segurado): o levantar acelera.
    bool getUpHurry = false;
    // Cambaleio (pedido do usuario, 05/10 noite): um choque forte amolece o
    // corpo, mas de pe ele tenta se manter - o tonus volta em ~1,5 s, os
    // passos de equilibrio seguem e so vai ao chao se cair de fato.
    bool staggerRecovery = false;
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
    // Carrinho: correndo, agachar (intent.crouch, a borda) da o carrinho
    // para onde ele olha (intent.requestedFacingYawRadians).
    bool tackleEnabled = true;
    // Carrinho no ar (a voadora): no ar, Ctrl com Shift + W (intent.crouch,
    // effort e a direcao pedida para a frente do olhar), uma por voo.
    bool airTackleEnabled = true;
    // Manejo de bola (BallControl3D): o alvo do pe que vai a bola, por cima
    // do alvo da passada desse pe enquanto ele esta no ar.
    BallFootGoal3D ballFoot;
    // E o desvio dos pes que nao tocam (somado ao alvo do pe no ar).
    std::array<Vec3, 2> ballAvoid {};
    // Chute: o pe de apoio pousa ao lado da bola (o pouso previsto dele vai
    // ate o ponto, ate um limite), o ritmo do ciclo (abaixo de 1 durante o
    // chute: o apoio fica no chao enquanto a outra perna arma, bate e
    // acompanha) e a pose de cima (bracos e tronco).
    BallPlant3D ballPlant;
    float ballGaitRate = 1.0f;
    BallKickPose3D ballKick;
    // O gesto do corpo inteiro (rework de 06/10): aqui so tronco, bracos e
    // cabeca (os pes vao pelo ballFoot/ballScript no planejador).
    BallGesturePose3D ballGesture;
    // Toque parado: o pe dirigido pelo manejo (o planejador o segue) e a
    // pelve indo com o passo de apoio (fora da ancora: a capsula segue).
    BallFootScript3D ballScript;
    BallBodyGoal3D ballBody;
    // A bola repele o alvo dos pes no ar (pe e canela nao colidem com ela).
    BallRepulsion3D ballRepel;
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
    // Onde a pelve de referencia estaria presa ao proxy de navegacao (a
    // capsula), sem a freada: e contra ela que o proxy segue o corpo. Fora
    // da freada e a propria guide.rootPositionWorld; freando, a referencia e
    // o corpo (ou o perfil de freada) e, comparado com ela, o proxy parava de
    // seguir o corpo e ficava ate ~0,9 m para tras - no fim da freada a
    // referencia voltava para ele e as pernas puxavam o corpo de costas.
    Vec3 navigationAnchorWorld;
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
    // Escala do atrito do corpo com o chao (1 = o do material). No deslize do
    // carrinho ele escorrega: o comprimento vem do embalo e do atrito.
    float bodyFrictionScale = 1.0f;
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
    // mais nas sondas, ou ladeira; solta em ~0,6 s depois de passar. So
    // diagnostico: nao muda a ajuda (ela e sempre a escolhida).
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
    // Quanto cada regra baixou (+) a pelve alvo neste tick (m): inclinacao
    // da curva, agachamento da freada, alcance das pernas e pouso.
    float pelvisDropCurveMeters = 0.0f;
    float pelvisDropBrakeMeters = 0.0f;
    float pelvisDropReachMeters = 0.0f;
    float pelvisDropLandingMeters = 0.0f;
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
    // Freada correndo (o corpo ainda rapido, a passada de corrida freando):
    // a velocidade do perfil de freada e quanto a ajuda e ainda a de quem
    // corre (0..1; 0 parado ou nos passos de freada). O controlador fisico
    // os recebe pela intencao (AdaptivePhysicalIntent3D).
    Vec3 brakeRunVelocityWorld;
    float brakeRunShare = 0.0f;
    // Fase da freada: 0 nenhuma, 1 correndo e freando, 2 passos de freada.
    int brakePhase = 0;
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
    // Onde e daqui a quanto o pe no ar vai pousar (o pouso natural, antes do
    // desvio do chute; -1 sem pouso previsto).
    std::array<Vec3, 2> footLandingWorld {};
    std::array<float, 2> footSecondsToTouchdown { -1.0f, -1.0f };
    // Contato de cada pe no clipe, na fase atual (1 apoio, 0 balanco).
    std::array<float, 2> clipFootContact {};
    // Quanto o ciclo ja esperou o pe tocar o chao neste apoio (s).
    float touchdownWaitSeconds = 0.0f;
    std::array<float, 2> legPhaseLead {};
    float thrustOffsetMeters = 0.0f;
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
    // Cambaleio: o quanto ele esta amolecido pelo choque (0..1; cai na hora
    // com o choque e volta com o tonus) e o choque deste tick (N.s, a soma
    // dos contatos fora o apoio dos pes).
    float stagger = 0.0f;
    float staggerImpulse = 0.0f;
    // Levantar biomecanico: ritmo do clipe (1 = no tempo dele; menos quando o
    // corpo fica para tras), a forca auxiliar na pelve e o "esforco" (sobe
    // enquanto o corpo emperra, e com ele o teto da ajuda).
    float getUpRate = 0.0f;
    // Levantar pelo corpo (BodilyGetUp3D): etapa, progresso nela e erro da
    // pelve contra a planejada.
    BodilyGetUpStage3D bodilyStage = BodilyGetUpStage3D::None;
    float bodilyStageProgress = 0.0f;
    float bodilyRootError = 0.0f;
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
    // A inclinacao da pelve e do peito no clipe do levantar agora (graus).
    float getUpExpectedTiltDegrees = 0.0f;
    float getUpExpectedChestTiltDegrees = 0.0f;
    float getUpUprightTorque = 0.0f;
    // Levantando: 1 = centro de massa sobre os apoios; a ajuda cai a 0 fora.
    float getUpOverSupport = 1.0f;
    // Rolando para ficar de bruços; levantando de bruços, os pes do
    // equilibrio (ligados, passos dados, maos esperando, quanto o centro de
    // massa passa da ponta dos pes em m).
    bool getUpRolling = false;
    // Carrinho: ativo, a variacao (indice em tackles), a velocidade de
    // entrada (m/s, na direcao do carrinho), o ritmo do clipe e quanto a
    // pelve andou desde o comeco (m).
    bool tackling = false;
    // Armado: esperando o pouso do pe para mergulhar (o preparo).
    bool tacklePending = false;
    int tackleVariant = -1;
    float tackleEntrySpeed = 0.0f;
    float tackleRate = 1.0f;
    float tackleTravel = 0.0f;
    bool getUpFootwork = false;
    int getUpBalanceSteps = 0;
    bool getUpBalanceHold = false;
    // Mao de apoio (0 livre, 1 indo ao chao, 2 apoiada, 3 voltando).
    std::array<int, 2> getUpHandStage {};
    float getUpBalanceExcess = 0.0f;
    // "Vida" no ragdoll: espernear no ar (0..1).
    float lifeFlail = 0.0f;
    // Perdendo o equilibrio (0..1): bracos em moinho, tronco contra a queda.
    float balanceReaction = 0.0f;
    // Postura perdida: o pendulo (centro de massa contra o meio dos pes) da
    // vertical e contra o da pose pedida, o peito da vertical (rad) e a queda
    // decidida (o controlador tira a ajuda).
    float postureDeviation = 0.0f;
    float postureChestTilt = 0.0f;
    float posturePendulum = 0.0f;
    bool postureLost = false;
    // v2: apoiado no alto (mureta, parede) e tentando se recompor - nem
    // postura perdida nem queda enquanto dura.
    bool braceRecovering = false;
    // Levantou com pressa (as teclas) ha pouco (< 2 s).
    bool getUpHurried = false;
    // Levantando com pressa e ja sobre os pes: a locomocao padrao pode
    // assumir (andando para onde as teclas pedem).
    bool getUpHandoffReady = false;
    // Tropeco/quase-queda em movimento (0..1).
    float tripRecovery = 0.0f;
    float stumbleExcess = 0.0f, stumbleRate = 0.0f, stumbleDecel = 0.0f, stumbleRisk = 0.0f;
    bool stumbleStepping = false;
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

// De onde o corpo esta ao passar do modo padrao (animado) para este: se ja
// anda. Sem isso, o fisico comecava como numa arrancada parada (gesto de
// arrancada, ciclo pelo pe de tras).
struct CharacterLocomotionSeed3D {
    bool moving = false;
    // No meio de um carrinho do modo padrao: ele continua daqui (o clipe, o
    // tempo nele, o ritmo, a escala e a origem do caminho, o rumo e a
    // velocidade de entrada) - a pancada nao o interrompe.
    const AnimationClip3D* tackleClip = nullptr;
    float tackleTime = 0.0f, tackleRate = 1.0f, tackleScale = 1.0f, tackleYaw = 0.0f, tackleEntrySpeed = 0.0f;
    Vec3 tackleStart;
};

class CharacterLocomotion3D final {
public:
    void reset(const RagdollProfile3D& profile,
        const RagdollState3D& state);
    // Depois do reset (vindo do modo padrao): a marcha continua - fase
    // casada com os pes do corpo -, e a pose parte das juntas dele.
    void seedMotion(const RagdollProfile3D& profile, const RagdollState3D& state,
        const CharacterLocomotionSeed3D& seed);
    void update(const RagdollProfile3D& profile,
        const CharacterLocomotionAnimations3D& animations,
        const RagdollState3D& state,
        const RagdollDynamics3D& dynamics,
        const CharacterLocomotionInput3D& input,
        float deltaTime);

    [[nodiscard]] const CharacterLocomotionOutput3D& output() const {
        return m_output;
    }
    // Diagnostico do levantar pelo corpo (a pose que ele pede).
    [[nodiscard]] const BodilyGetUp3D& bodilyGetUp() const { return m_bodilyGetUp; }
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
    // Teste: comeca cambaleando (como logo depois de um choque).
    void debugSetStagger(float stagger) { m_stagger = stagger; m_staggerHold = 0.25f * stagger; }
    void setFootTraceEnabled(bool enabled) { m_footTraceEnabled = enabled; }
    [[nodiscard]] ContactFootworkSettings3D& footworkSettings() {
        return m_footwork.settings();
    }

private:
    // Alvo do pe no balanco antes da correcao pela malha do pe (diagnostico).
    std::array<Vec3, 2> m_trackedSwingGoal {};
    std::array<bool, 2> m_trackedSwingGoalValid {};
    struct FootPlant {
        std::size_t linkIndex = 0;
        bool valid = false;
        bool planted = false;
        // Giro do pe para dentro da curva (rad): vai ao alvo durante o
        // balanco e fica no apoio.
        float curveYaw = 0.0f;
        Vec3 lockedPositionWorld;
        // Travado antes de o pe fisico tocar: a trava ainda vai para onde ele
        // tocar (ate este prazo, s).
        float lockFollowTouch = 0.0f;
        // Quanto o pe ja esperou para sair carregado (s, ver LiftAfterUnload).
        float liftHoldSeconds = 0.0f;
        float touchHoldSeconds = 0.0f;
        // Devolvido pelo planejador apoiado (fim do passo de arrancada): fica
        // travado ate o clipe chegar ao apoio dele, no maximo este tempo (s).
        float handbackHoldSeconds = 0.0f;
        // Primeiro toque fisico neste balanco (o pe pode tocar antes do apoio
        // do clipe e escorregar ate ele): e ali que a trava vai.
        bool firstTouchValid = false;
        Vec3 firstTouchWorld;
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
    // A reescrita: o update em blocos (CharacterLocomotion3D_<bloco>.cpp). Os tipos que eram
    // locais da funcao e o Tick (o que um bloco passa aos seguintes num tick).
    struct GroundSample {
        bool valid = false;
        float height = 0.0f;
        Vec3 normal { 0.0f, 0.0f, 1.0f };
    };
    struct FootGoal {
        bool active = false;
        Vec3 position;
        Quaternion orientation;
        float pelvisGround = 0.0f;
        // Quanto este pe conta para a pelve alcanca-lo: 1 apoiado ou
        // pousando, subindo no fim do balanco, 0 no ar.
        float reach = 0.0f;
        // O pe do clipe, sem o desnivel do terreno: a pelve so desce o que o
        // terreno pede a mais que a animacao.
        Vec3 animatedPosition;
        Quaternion animatedOrientation;
        // Pe em balanco: onde vai pousar (tornozelo, no mundo), quanto o
        // quadril anda ate la e a perna do clipe no toque. A pelve comeca a
        // descer antes do toque; descendo o ultimo degrau, o pe pousava
        // fora do alcance da perna e a trava soltava e voltava.
        float anticipate = 0.0f;
        Vec3 landingPosition;
        Quaternion landingOrientation;
        Vec3 hipTravel;
        float authoredLandingHorizontal = 0.0f;
        float authoredLandingVertical = 0.0f;
        // Teto da descida que este pe pede (m): o pe no ar, correndo no
        // plano, pede pouco.
        float dropCap = std::numeric_limits<float>::infinity();
    };
    struct Tick {
        const RagdollProfile3D& profile;
        const CharacterLocomotionAnimations3D& animations;
        const RagdollState3D& state;
        const RagdollDynamics3D& dynamics;
        const CharacterLocomotionInput3D& input;
        float deltaTime = 0.0f;
        bool PostureSupportPendulum {};
        bool StartOnlyFromRest {};
        bool LandingResync {};
        float StanceRootBand {};
        float StanceDampCurveSpan {};
        std::vector<Vec3> previousCoordinates {};
        const CharacterPhysicalState3D* physical {};
        Vec3 centerOfMass {};
        bool anyGroundContact {};
        std::array<bool, 2> feetGroundContact {};
        float contactFloor {};
        float rootUpright {};
        float groundHeightGuess {};
        float measuredComHeight {};
        bool physicsOwnsBody {};
        float fallenUpright {};
        bool fallenObservation {};
        bool getUpActive {};
        float mass {};
        float impact {};
        float rootAuthority {};
        Vec3 desiredPlanarVelocity {};
        float gaitVelocityScale {};
        Vec3 brakeNavigationVelocity {};
        float speed {};
        bool moving {};
        float viewYaw {};
        float lookYaw {};
        bool turningInPlace {};
        float pathSpeed {};
        float curveIntent {};
        float pathTurnRate {};
        bool grounded {};
        CharacterLocomotionState3D desiredState {};
        Quaternion heading {};
        Vec3 localVelocity {};
        bool gaitMoving {};
        bool gaitStarting {};
        float startupProgress {};
        float startupChestAmount {};
        float startupRootLean {};
        float startupPelvisDrop {};
        float gaitCycleSpeed {};
        const AnimationClip3D* standUpClip {};
        const AnimationClip3D* gaitClip {};
        bool jumpPose {};
        bool leapPose {};
        const AnimationClip3D* idleClip {};
        bool authoredStartActive {};
        float signedCurveAcceleration {};
        float authoredCurveBlend {};
        int authoredCurveKind {};
        const AnimationClip3D* authoredCurveClip {};
        const AnimationClip3D* selectedClip {};
        float playbackRate {};
        float strideScale {};
        float terrainSpeedLimit {};
        std::vector<Vec3> sampled {};
        Quaternion rootRotation {};
        Vec3 rootTranslation {};
        std::array<float, 2> contactWeights {};
        float clipSampleTime {};
        std::array<float, 2> legSampleTime {};
        float armSampleTime {};
        float landingBraceForward {};
        float landingBraceLateral {};
        float transition {};
        float tangentWeight {};
        float viewYawError {};
        float curveShare {};
        float forwardLean {};
        float lateralLean {};
        Quaternion rootTilt {};
        Quaternion accelerationTilt {};
        Quaternion targetRootOrientation {};
        Vec3 navigationRoot {};
        float lowAssist {};
        Vec3 targetRootPosition {};
        bool referenceRebase {};
        bool airbornePose {};
        RagdollAnimationPose3D localPose {};
        float baseGround {};
        bool footworkEnabled {};
        bool onGround {};
        Vec3 planarVelocity {};
        std::array<FootGoal, 2> goals {};
        bool movementRequested {};
        bool standing {};
        bool recovering {};
        Vec3 standingShift {};
        float drop {};
        float muscle {};
        float squaredError {};
        std::uint32_t errorCount {};
    };
    void stepSense(Tick& t);
    void stepTackle(Tick& t);
    void stepGetUp(Tick& t);
    void stepAuthority(Tick& t);
    void stepSensors(Tick& t);
    void stepHeading(Tick& t);
    void stepAir(Tick& t);
    void stepCycle(Tick& t);
    void stepGestures(Tick& t);
    void stepLook(Tick& t);
    void stepRoot(Tick& t);
    void stepFeet(Tick& t);
    void stepLegs(Tick& t);
    void stepDrives(Tick& t);
    void stepOutputs(Tick& t);

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
    // Postura mais ereta: o quanto a pelve-alvo esta acima da do clipe (m).
    float m_postureLift = 0.0f;
    float m_touchdownWaitSeconds = 0.0f;
    // Passada pelo contato: quanto cada perna esta adiantada em relacao ao
    // ciclo (fracao), se o pe ja tocou no apoio atual do clipe, e de que lado
    // e cada link de perna (-1 nao e perna).
    std::array<float, 2> m_legPhaseLead {};
    std::array<bool, 2> m_legTouchedInStance {};
    std::vector<int> m_legSideOfLink;
    int m_waitingLeg = -1;
    bool m_contactSteady = false;
    // Altura do voo regulada (Raibert): topo do corpo e da referencia no voo
    // atual, o erro do ultimo voo e o ajuste da perna de apoio.
    bool m_inFlight = false;
    float m_flightBodyApex = 0.0f;
    float m_flightGuideApex = 0.0f;
    float m_flightApexError = 0.0f;
    float m_thrustOffset = 0.0f;
    float m_stanceDampingOffset = 0.0f;
    float m_stanceDampingGuideZ = 0.0f;
    bool m_stanceDampingValid = false;
    std::array<float, 2> m_landingReach {};
    // Pe que espera o toque: quanto ja deslizou para tras (mundo).
    std::array<Vec3, 2> m_waitPawShift {};
    // Chute: o desvio do pouso do pe de apoio (ao lado da bola), suavizado.
    std::array<Vec3, 2> m_ballPlantShift {};
    // As pernas e bracos para o gesto com a bola (BallGesturePose3D).
    BallGestureLegs3D m_gestureLegs;
    // A pelve do toque parado (planar, suavizada ~0,1 s).
    Vec3 m_ballBodyDelta;
    // O centro dos pes na pose (referencial da raiz, tick anterior): parado
    // com a ajuda cheia, o corpo se assenta sobre a base real.
    // Giro parado pelo clipe (ajuda cheia): o ciclo atual (clipe, tempo,
    // sentido, quanto gira, rumo no comeco), o peso das pernas do clipe na
    // pose e o deslocamento da raiz que o clipe faz.
    struct ClipTurn {
        bool active = false;
        const AnimationClip3D* clip = nullptr;
        float time = 0.0f;
        float sign = 1.0f;
        float total = 0.0f;
        float span = 1.5f;
        float startYaw = 0.0f;
        float startPelvisYaw = 0.0f;
    };
    ClipTurn m_clipTurn;
    float m_clipTurnWeight = 0.0f;
    Vec3 m_clipTurnRootOffset;
    // Continuidade do alvo do pe na troca de dono (planejador <-> passada).
    std::array<bool, 2> m_goalPlannerOwned {};
    std::array<bool, 2> m_goalOwnerValid {};
    std::array<float, 2> m_goalHandoff {};
    std::array<Vec3, 2> m_goalHandoffPosition {};
    std::array<Quaternion, 2> m_goalHandoffOrientation {};
    std::array<Vec3, 2> m_lastFootGoalPosition {};
    std::array<Quaternion, 2> m_lastFootGoalOrientation {};
    Vec3 m_poseFeetCenterLocal;
    bool m_poseFeetCenterValid = false;
    // Ritmo do ciclo suavizado (fracao do ritmo do clipe) e a fase propria
    // dos bracos.
    float m_phaseRateScale = 1.0f;
    float m_armPhase = 0.0f;
    float m_armPhaseRate = 0.0f;
    bool m_armPhaseValid = false;
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
    // Troca de movimento em curso (0..1): troca de setor/ciclo, freada ou o
    // pedido longe da velocidade do corpo. Sobe na hora, cai em ~0,3 s.
    float m_gaitChange = 0.0f;
    std::array<bool, 2> m_gaitSwingLatched {};
    std::array<float, 2> m_gaitSwingCeiling {};
    float m_sinceGaitFlip = 10.0f;
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
    // Inclinacao do corpo inteiro na curva (rad, + para a esquerda): atan da
    // aceleracao centripeta do caminho sobre g.
    float m_spineCurveRoll = 0.0f;
    // Levantar (04/10): rolar para ficar de bruços antes de levantar
    // (arrumando-se deitado) e, levantando de bruços, os pes do equilibrio a
    // partir dos quatro apoios - presos no chao pelo IK e dando passos para
    // baixo do centro de massa; as maos so soltam com ele sobre os pes.
    bool m_getUpRolling = false;
    float m_getUpRollSeconds = 0.0f;
    float m_getUpRollSign = 0.0f;
    int m_getUpRollTop = -1;
    bool m_getUpRollDone = false;
    bool m_getUpFootwork = false;
    Vec3 m_getUpFootworkForward { 1.0f, 0.0f, 0.0f };
    std::array<Vec3, 2> m_getUpFootWorld {};
    int m_getUpStepSide = -1;
    float m_getUpStepSeconds = 0.0f;
    Vec3 m_getUpStepFrom {};
    Vec3 m_getUpStepTo {};
    bool m_getUpBalanceHold = false;
    // Mao de apoio no levantar (04/10, ideia do usuario): quando o clipe pede
    // a mao no chao, ela vai a um ponto ao lado e a frente do ombro (nunca
    // para dentro, embaixo do peito), fica presa ali com a palma no chao e
    // solta quando o clipe a levanta ou o braco nao alcanca.
    struct GetUpHand {
        int stage = 0;          // 0 livre, 1 indo ao chao, 2 apoiada, 3 voltando ao clipe
        float seconds = 0.0f;
        float weight = 0.0f;
        float clipUpSeconds = 0.0f;
        Vec3 from {};
        Vec3 spot {};
    };
    std::array<GetUpHand, 2> m_getUpHands {};
    // Carrinho (ver Tackling): o clipe sorteado, o rumo (para onde olhava),
    // a ancora (onde o caminho do clipe comeca, no mundo), ritmo e escala do
    // percurso pela velocidade de entrada.
    bool m_tackling = false;
    const AnimationClip3D* m_tackleClip = nullptr;
    int m_tackleVariant = -1;
    int m_tackleLastVariant = -1;
    std::uint32_t m_tackleCount = 0;
    float m_tackleYaw = 0.0f;
    Vec3 m_tackleStart {};
    float m_tackleRate = 1.0f;
    float m_tackleDistanceScale = 1.0f;
    float m_tackleEntrySpeed = 0.0f;
    float m_tackleCooldown = 0.0f;
    float m_tackleBehindSeconds = 0.0f;
    float m_tackleHoldSeconds = 0.0f;
    float m_tackleExpectedHeight = 1.0f;
    float m_tackleExpectedRise = 0.0f;
    // Velocidade no plano quando a pelve chegou ao chao (-1 antes).
    float m_tackleLandSpeed = -1.0f;
    bool m_previousCrouchIntent = false;
    // Preparo: armado (esperando o pouso do pe com que o clipe comeca
    // apoiado), ha quanto tempo, que pe e o ultimo estado dos pes.
    bool m_tacklePending = false;
    float m_tacklePendingSeconds = 0.0f;
    int m_tackleStartSide = 0;
    std::array<bool, 2> m_tacklePreviousFoot {};
    float m_sinceTackleSeconds = 10.0f;
    float m_getUpBalanceHoldSeconds = 0.0f;
    int m_getUpBalanceSteps = 0;
    float m_curveSprintShare = 0.0f;
    float m_bodyCurveLean = 0.0f;
    // Curva real do corpo: rumo da velocidade do centro de massa, a taxa de
    // giro dele filtrada (rad/s) e a aceleracao centripeta (m/s2, + para a
    // esquerda).
    float m_bodyHeading = 0.0f;
    bool m_bodyHeadingValid = false;
    float m_bodyTurnRate = 0.0f;
    float m_bodyCurveAcceleration = 0.0f;
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
    // Freada pela passada (modo fisico): o corpo ainda leva um momento que o
    // jogador nao pede. Ligada/desligada (com histerese), o peso dela na
    // passada (0..1), a velocidade do perfil de freada no rumo do corpo e
    // esse rumo (mundo, no plano).
    bool m_brakeGaitActive = false;
    float m_brakeGait = 0.0f;
    float m_brakeGaitSpeed = 0.0f;
    Vec3 m_brakeGaitDirection;
    // Agachamento da freada (m, >= 0) e a velocidade dele: o pe de freada so
    // alcanca o chao a frente com a pelve mais baixa.
    float m_brakeCrouch = 0.0f;
    float m_brakeCrouchVelocity = 0.0f;
    // A freada ja esta nos passos (devagar o bastante): a corrida que
    // desacelerava parou de vez ate a freada acabar.
    bool m_brakeStepping = false;
    // Freando para inverter (o pedido e o sentido contrario): a virada.
    bool m_brakeReversing = false;
    // Lado do pivo da virada (+1 esquerda, -1 direita, 0 nao escolhido).
    float m_pivotSign = 0.0f;
    // Parcela da ajuda de corrida na freada (0..1; sobe correndo e freando,
    // sai depressa nos passos de freada).
    float m_brakeRunShare = 0.0f;
    // Ha quanto tempo esta freando (s).
    float m_brakeSeconds = 0.0f;
    // Desde o ultimo tick em que o jogador pedia correr (> 1 m/s).
    float m_sinceRunRequest = 10.0f;
    // Referencia da corrida freando (no plano, presa ao corpo por uma
    // coleira) e quanto a base ja passou para o corpo (passos de freada).
    bool m_brakeReferenceValid = false;
    Vec3 m_brakeReferencePosition;
    float m_brakeAnchor = 0.0f;
    // Parado depois de uma freada: a base da referencia onde o corpo parou
    // (no plano) e quanto ela vale (volta para a capsula com o pedido de
    // movimento).
    bool m_wasBrakeStepping = false;
    Vec3 m_stopRoot;
    float m_stopRootBlend = 0.0f;
    // A base de parado anda com o resto da velocidade do corpo e freia (as
    // pernas seguram o corpo sobre os pes, sem passo); ativa ate o pedido de
    // movimento.
    Vec3 m_stopVelocity;
    bool m_stopHolding = false;
    // Abaixo da velocidade dos passos, esperando um pe da passada pousar (s).
    float m_brakeStepWait = 0.0f;
    // Desde o fim da ultima freada (s).
    float m_sinceBrakeSeconds = 10.0f;
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
    // v2: salto correndo - o ciclo de passada, do ponto da decolagem
    // (m_leapFrom, fracao do clipe) ao toque seguinte (m_leapSpan a frente).
    const AnimationClip3D* m_leapClip = nullptr;
    float m_leapFrom = 0.0f;
    float m_leapSpan = 0.0f;
    float m_flightPhase = 0.0f;
    // Fall/get-up: link indices cached once for the contact scan that
    // decides whether the character is actually down (a hand/knee/torso
    // touching the ground), independent of the authored-clip foot contact
    // weights the normal locomotion foot lock above already uses.
    std::array<std::size_t, 2> m_handLinks {};
    // Caindo: ha quanto tempo cada mao segura o chao (s; 0 sem contato).
    std::array<float, 2> m_handCatchSeconds {};
    // Caindo e no chao: alvos das juntas suavizados (sem saltos de alvo) e
    // quanto da posicao fetal vale (0..1).
    std::vector<Vec3> m_fallSmoothCoordinates;
    float m_fetal = 0.0f;
    // No ar, caindo: quanto da pose de equilibrio no ar vale (0..1).
    float m_airBalance = 0.0f;
    // Descendo de um voo: quanto do preparo do pouso vale (0..1).
    float m_landingPrep = 0.0f;
    // Postura ereta de pescoco e cabeca parado (0..1, suavizada).
    float m_postureUpright = 0.0f;
    // Rolagem da pelve filtrada (~0,35 s): a parte lenta (inclinacao de
    // proposito, curva) separada da passada.
    float m_pelvisSideSlow = 0.0f;
    std::array<std::size_t, 2> m_shinLinks {};
    std::size_t m_chestLink = 0;
    float m_elapsedSeconds = 0.0f;
    CharacterGetUpPhase3D m_getUpPhase = CharacterGetUpPhase3D::None;
    // Levantar pelo corpo: com ele o Rising nao toca o clipe nem usa ajuda.
    BodilyGetUp3D m_bodilyGetUp;
    BodilyGetUpOutput3D m_bodilyPose;
    bool m_bodilyRising = false;
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
    // Cambaleio (0..1) e a classe de cada link para o tonus dele (0 perna,
    // 1 tronco, 2 braco).
    float m_stagger = 0.0f;
    float m_staggerHold = 0.0f;
    std::vector<std::uint8_t> m_staggerLimb;
    float m_previousTilt = 0.0f;
    Vec3 m_fallDirection { 1.0f, 0.0f, 0.0f };
    // Tropeco durante uma passada: contato inesperado do pe no meio do
    // balanco dispara uma janela curta de recuperacao. Cedo, o mesmo pe
    // sobe e continua; tarde, ele pousa e o passo seguinte amplia a base.
    float m_tripRecovery = 0.0f;
    float m_tripRecoverySeconds = 0.0f;
    // Postura perdida (ver PostureLossFall): ha quanto tempo o corpo esta
    // segurado fora dos pes, o pendulo da pose pedida (centro de massa menos
    // o meio dos pes, mundo, do tick anterior) e a queda decidida.
    float m_postureHeldSeconds = 0.0f;
    Vec3 m_referencePendulum {};
    bool m_referencePendulumValid = false;
    // Os pes da pose alvo (no mundo, do tick anterior): o pendulo da
    // referencia pelos mesmos pes apoiados que o do corpo (v2).
    std::array<Vec3, 2> m_referenceFeetWorld {};
    // A passada atual comecou de parado (arrancada; ver gaitStarting).
    bool m_gaitStartFromRest = false;
    // Pouso de um pulo direto na passada: o pe que tocou primeiro (o ciclo
    // recomeca no toque dele; -1 nada).
    int m_landingResyncFoot = -1;
    // v2: a inclinacao a mais pela aceleracao que as pernas precisam fazer
    // (x para a frente, y para a esquerda, rad; filtrada).
    Vec3 m_leanNeed {};
    // v2, recuperacao apoiada (ver o bloco no update).
    float m_bracedMemory = 0.0f;
    float m_braceAttemptSeconds = 0.0f;
    float m_braceStallSeconds = 0.0f;
    float m_braceBestTilt = 0.0f;
    bool m_braceRecovering = false;
    Vec3 m_referenceCenterOfMassWorld {};
    bool m_postureLost = false;
    // Quase-queda em movimento: inclinacao da pelve para a frente no rumo
    // (rad), a taxa dela, a de regime, e a freada do corpo alem da pedida.
    bool m_stumbleValid = false;
    float m_stumblePitch = 0.0f;
    float m_stumblePitchRate = 0.0f;
    float m_stumbleBaseline = 0.0f;
    float m_stumbleAlong = 0.0f;
    float m_stumbleDecel = 0.0f;
    float m_stumbleRisk = 0.0f;
    // A reacao da quase-queda (0..1): os bracos abrem e a pelve desce.
    float m_stumbleReaction = 0.0f;
    // Passo de quase-queda pelo planejador em andamento, ha quanto tempo, e
    // desde o ultimo (s).
    bool m_stumbleStepping = false;
    float m_stumbleStepSeconds = 0.0f;
    float m_sinceStumbleStep = 10.0f;
    Vec3 m_stumbleStepDirection { 1.0f, 0.0f, 0.0f };
    // O mesmo passo do planejador na arrancada (o primeiro passo, do pe de
    // tras): pedido ao entrar na passada, e se o passo atual e de arrancada.
    bool m_startStepRequested = false;
    // A semente (seedMotion), aplicada no proximo update: as juntas do corpo,
    // os pes e as velocidades deles em relacao a pelve, no rumo dela, e a
    // velocidade dele.
    // Levantar: o ritmo do clipe agora (vezes o dele) e se a ultima saida foi
    // com pressa (as teclas - a volta ao modo padrao vem mais rapido).
    float m_getUpSpeedNow = 1.0f;
    bool m_getUpHurriedExit = false;
    bool m_getUpHandoffReady = false;
    bool m_motionSeedPending = false;
    // O giro da camera (rad/s, filtrado) e se o carrinho pedido ja foi
    // escolhido pela camera/corrida (nao troca pelo pe).
    float m_viewTurnRate = 0.0f;
    float m_previousViewYaw = 0.0f;
    bool m_previousViewValid = false;
    bool m_tackleChosen = false;
    // Uma voadora por voo (no modo fisico).
    bool m_airTackleSpent = false;
    std::vector<Vec3> m_seedJoints;
    std::array<Vec3, 2> m_seedFeet {}, m_seedFeetVelocity {};
    float m_seedSpeed = 0.0f;
    bool m_startStepActive = false;
    // Arrancada com a ajuda dosada: o teto de velocidade que cresce desde a
    // entrada na passada (m/s; alto = sem teto).
    float m_startDoseCap = 1000.0f;
    // Corpo parado (centro de massa a menos de 0,3 m/s) ha quanto tempo (s).
    float m_bodyRestSeconds = 0.0f;
    Vec3 m_stumbleDirection { 1.0f, 0.0f, 0.0f };
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
    // Com pouca ajuda: o ciclo de sprint ja pode tocar (a passada rapida o
    // bastante para ele; abaixo, o de trote).
    bool m_sprintClipAllowed = true;
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
    // Curva fechada: o rumo pedido girando (rad/s, filtrado) e o teto de
    // velocidade que cabe nessa curva.
    float m_requestYaw = 0.0f;
    bool m_requestYawValid = false;
    float m_requestTurnRate = 0.0f;
    float m_curveSpeedCap = 1000.0f;
    // Perna da passada no ar pela altura da pelve de verdade quando ela esta
    // abaixo da de referencia (peso por pe, entra no balanco, sai no apoio).
    std::array<float, 2> m_swingLowWeight {};
    std::array<float, 2> m_swingLowLanding { 1.0f, 1.0f };
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
