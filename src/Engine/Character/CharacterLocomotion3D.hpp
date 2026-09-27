#pragma once

#include "Engine/Animation/AnimationClip3D.hpp"
#include "Engine/Physics/PhysicsScene3D.hpp"

#include <array>
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
    Vec3 desiredVelocityWorld;
    float facingYawRadians = 0.0f;
    float lookPitchRadians = 0.0f;
    bool grounded = true;
    bool crouched = false;
    bool sprinting = false;
    bool controlled = false;
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
    bool gravityCompensationEnabled = false;
    // 0 = the renderer should show the authored pose untouched; 1 = show the
    // simulated body. Anything between is the blend, and it is the only
    // thing that decides how much physics the player ever sees.
    float physicsBlend = 0.0f;
};

struct CharacterLocomotionTelemetry3D {
    CharacterLocomotionState3D state = CharacterLocomotionState3D::Idle;
    Vec3 localVelocity;
    float speedMetersPerSecond = 0.0f;
    float facingYawRadians = 0.0f;
    float viewYawErrorRadians = 0.0f;
    float turningRateRadiansPerSecond = 0.0f;
    float forwardLeanRadians = 0.0f;
    float lateralLeanRadians = 0.0f;
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
    std::array<float, 2> footClearanceMeters {};
    float rootUpright = 1.0f;
    CharacterFallOrientation3D fallOrientation = CharacterFallOrientation3D::None;
    CharacterGetUpPhase3D getUpPhase = CharacterGetUpPhase3D::None;
    float getUpProgress = 0.0f;
    std::uint32_t getUpAttempt = 0;
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
    float m_stateSeconds = 0.0f;
    float m_transitionSeconds = 0.0f;
    float m_transitionDurationSeconds = 0.14f;
    float m_cyclePhase = 0.0f;
    float m_idleSeconds = 0.0f;
    float m_facingYaw = 0.0f;
    float m_turningRate = 0.0f;
    float m_reactionStrength = 0.0f;
    float m_physicsBlend = 0.0f;
    float m_physicsHoldSeconds = 0.0f;
    Vec3 m_previousDesiredVelocity;
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
    // Giro parado: para onde a pelve vai (o alvo do olhar) e o pe que ainda
    // tem de dar o segundo passo do giro (-1: nenhum).
    float m_standingTurnTarget = 0.0f;
    int m_pendingTurnFoot = -1;
    // Jump: captured at the instant the capsule leaves the ground - how fast
    // it was going, how fast it left upward, and which authored jump plays
    // for the whole flight (and its landing).
    float m_liftoffSpeed = 0.0f;
    float m_liftoffVerticalSpeed = 0.0f;
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
    bool m_previousGrounded = true;
    bool m_initialized = false;
};

} // namespace MatterEngine
