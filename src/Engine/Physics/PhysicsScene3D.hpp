#pragma once

#include "Engine/Math/Ray3D.hpp"
#include "Engine/Physics/PhysicsEngine3D.hpp"
#include "Engine/Physics/RagdollProfile3D.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <limits>
#include <memory>
#include <span>
#include <vector>

namespace MatterEngine {

struct PhysicsHandleSettings3D {
    float linearStiffness = 1600.0f;
    float linearDampingRatio = 1.0f;
    float maximumForce = 6000.0f;
    float angularStiffness = 220.0f;
    float angularDampingRatio = 1.0f;
    float maximumTorque = 900.0f;
};

struct PhysicsGrabTarget3D {
    Vec3 position;
    Quaternion orientation;
    bool lockOrientation = false;
};

struct PhysicsRagdollRayHit3D {
    RagdollHandle3D ragdoll;
    std::uint32_t linkIndex = 0;
    Vec3 position;
    Vec3 normal { 0.0f, 0.0f, 1.0f };
    float distance = 0.0f;
};

struct CharacterMotorCommand3D {
    Vec3 moveDirection;
    // Multiplies the selected gait speed without changing acceleration.
    // Avatar locomotion uses it to keep capsule travel synchronized with
    // the authored stride in each cardinal direction.
    float speedScale = 1.0f;
    bool sprint = false;
    bool crouch = false;
    bool jumpPressed = false;
    // Pulo ao soltar: quanto tempo o botao ficou segurado antes deste pulo
    // (s). Um toque e o pulo padrao; segurando mais, mais alto (ver
    // characterJumpSpeed3D).
    float jumpChargeSeconds = 0.0f;
    bool toggleFlight = false;
    // A controlled character owns both a navigation capsule and a rendered
    // articulation. The capsule must not sweep against that representation.
    bool ignoreRagdolls = false;
    // Velocidade plana extra so neste passo, com colisao: a capsula de
    // navegacao indo atras do corpo fisico quando ele e empurrado, tropeca
    // ou cai. Nao entra na inercia do controlador (state.velocity).
    Vec3 followVelocity;
    // A capsula e so navegacao: nao empurra nem e empurrada por corpos
    // dinamicos nem por ragdolls, e o corpo interno dela nao bloqueia nada
    // alem do cenario estatico. Quem colide com o mundo movel e o corpo
    // fisico do personagem (modo fisico adaptativo). Hoje so no Jolt.
    bool navigationProxy = false;
};

struct CharacterMotorSettings3D {
    float radius = 0.35f;
    float standingHeight = 1.80f;
    float crouchedHeight = 1.15f;
    float walkSpeed = 5.2f;
    float sprintSpeed = 8.2f;
    float crouchedSpeed = 2.8f;
    float groundAcceleration = 32.0f;
    float groundDeceleration = 38.0f;
    float airAcceleration = 8.0f;
    float jumpSpeed = 3.8f;
    // Pulo com altura controlada (o pulo sai ao soltar o botao): ate
    // jumpTapSeconds segurado e o pulo padrao; em jumpFullChargeSeconds chega
    // a jumpChargedSpeedScale da velocidade (1,16 ~ +35% de altura).
    float jumpTapSeconds = 0.15f;
    float jumpFullChargeSeconds = 0.60f;
    float jumpChargedSpeedScale = 1.16f;
    float gravityScale = 1.0f;
    float maximumFallSpeed = 48.0f;
    float maximumSlopeDegrees = 50.0f;
    float maximumStepHeight = 0.38f;
    float skinWidth = 0.018f;
    float coyoteTime = 0.10f;
    float jumpBufferTime = 0.12f;
    float flightSpeed = 12.0f;
    float fastFlightSpeed = 28.0f;
    float swimSpeed = 3.4f;
    float fastSwimSpeed = 5.0f;
    float swimAcceleration = 9.0f;
    // Velocidade maxima que um esbarrao do personagem pode transferir a um
    // prop dinamico. O impulso aplicado no contato e sempre massa_do_prop *
    // velocidade_desejada (nunca um impulso fixo em kg*m/s): assim um objeto
    // leve nunca sai arremessado so por ter pouca massa, e um objeto pesado
    // exige mais forca para se mover na mesma velocidade, como no mundo real.
    float maximumPushSpeedMetersPerSecond = 2.2f;
    // Fracao do passo do personagem que precisa ser bloqueada pelo contato
    // para o empurrao atingir a velocidade maxima acima. Toques de raspao
    // (PxControllerShapeHit::length pequeno) resultam em empurroes mais
    // fracos, proporcionalmente a essa penetracao.
    float pushSaturationPenetrationMeters = 0.25f;
};

// Carga do pulo (0 = toque, 1 = cheia) pelo tempo segurado, e a velocidade
// de saida correspondente.
inline float characterJumpCharge3D(const CharacterMotorSettings3D& settings,
    float heldSeconds) {
    const float span = std::max(0.01f,
        settings.jumpFullChargeSeconds - settings.jumpTapSeconds);
    const float x = std::clamp((heldSeconds - settings.jumpTapSeconds) / span, 0.0f, 1.0f);
    return x * x * (3.0f - 2.0f * x);
}

inline float characterJumpSpeed3D(const CharacterMotorSettings3D& settings,
    float heldSeconds) {
    return settings.jumpSpeed * (1.0f + (settings.jumpChargedSpeedScale - 1.0f)
        * characterJumpCharge3D(settings, heldSeconds));
}

struct PhysicsCharacterState3D {
    Vec3 position;
    Vec3 velocity;
    bool grounded = false;
    bool crouched = false;
    bool flying = false;
    bool swimming = false;
    bool flightExitBlocked = false;
};

struct RagdollSpawnDefinition3D {
    std::uint64_t entityId = 0;
    Vec3 pelvisPosition;
    Quaternion orientation;
    float rigidityPercent = 0.0f;
    bool active = true;
};

enum class RagdollContactMotion3D : std::uint8_t { Static, Dynamic, Kinematic };

struct RagdollContactPoint3D {
    std::uint32_t linkIndex = 0;
    Vec3 position;
    // Normal orientada da superfície tocada para o link do ragdoll.
    Vec3 normal { 0.0f, 0.0f, 1.0f };
    float normalImpulseNewtonSeconds = 0.0f;
    float tangentialSpeedMetersPerSecond = 0.0f;
    bool otherBodyDynamic = false;
    // Tipo de movimento do corpo tocado.
    RagdollContactMotion3D otherMotion = RagdollContactMotion3D::Static;
    // O impulso foi estimado antes do solver (Jolt: EstimateCollisionResponse
    // no callback) em vez de lido depois dele. Não é carga medida.
    bool impulseEstimated = false;
};

// External contacts for ALL links, independent of contactSensor and audio.
// At most six aggregates per link: three motion types, upward/other normals.
// Dominant point data is not a center of pressure. Normal impulses are summed
// within this completed tick only; Jolt estimates them before solving.
struct RagdollInteraction3D {
    std::uint32_t linkIndex = 0;
    RagdollContactMotion3D otherMotion = RagdollContactMotion3D::Static;
    std::uint32_t pointCount = 0;
    Vec3 positionWorld;
    Vec3 normalWorld;
    // Velocity of this link's contact point minus the other body's point.
    Vec3 relativeVelocityWorld;
    Vec3 normalImpulseWorld;
    float normalImpulseNewtonSeconds = 0.0f;
    float strongestNormalImpulseNewtonSeconds = 0.0f;
    bool impulseEstimated = false;
    bool velocityBeforeSolve = false;
};

struct RagdollDriveTarget3D {
    std::uint32_t linkIndex = 0;
    RagdollAxis3D axis = RagdollAxis3D::Twist;
    float positionRadians = 0.0f;
    float velocityRadiansPerSecond = 0.0f;
    float feedforwardTorqueNewtonMeters = 0.0f;
    float stiffnessScale = 1.0f;
    float dampingScale = 1.0f;
    float maximumTorqueScale = 1.0f;
    // Quanto da compensacao de gravidade (com ela ligada) vale nesta junta:
    // 1 sustenta o peso do membro; menos, ele pesa (um corpo largado).
    float gravityCompensationScale = 1.0f;
};

enum class RagdollTraversalMode3D : std::uint8_t {
    Grounded,
    Airborne,
    Jumping,
    SlidingSteep,
    PhysicalOverride
};

struct GroundProbeResult3D {
    bool hasSurface = false;
    bool walkable = false;
    Vec3 pointWorld;
    Vec3 normalWorld { 0.0f, 0.0f, 1.0f };
    float distanceMeters = 0.0f;
    float slopeDegrees = 90.0f;
    Vec3 supportVelocityWorld;
};

struct CapsuleTraversalQuery3D {
    Vec3 centerWorld;
    Vec3 desiredDisplacementWorld;
    float radiusMeters = 0.33f;
    float cylinderHalfHeightMeters = 0.54f;
    float maximumSlopeDegrees = 48.0f;
    float stepOffsetMeters = 0.24f;
    float skinWidthMeters = 0.02f;
    float groundProbeDistanceMeters = 0.22f;
    float groundAdhesionDistanceMeters = 0.12f;
    std::uint32_t maximumIterations = 4;
};

struct CapsuleTraversalResult3D {
    Vec3 requestedDisplacementWorld;
    Vec3 allowedDisplacementWorld;
    Vec3 slideDisplacementWorld;
    GroundProbeResult3D ground;
    RagdollTraversalMode3D mode = RagdollTraversalMode3D::Airborne;
    bool blocked = false;
    bool groundAdhesionActive = false;
};

// Guia de pose para uma articulation flutuante. A pose articular, a translação
// e a rotação da raiz podem ceder separadamente. Isso permite conservar a
// trajetória validada pela cápsula enquanto contatos e torque físico resolvem
// a orientação corporal. Autoridade 1 aplica o canal depois do solver; zero
// deixa esse canal inteiramente físico.
struct RagdollAnimationConstraint3D {
    float poseAuthority = 0.0f;
    // Translação/velocidade linear e orientação/velocidade angular possuem
    // autoridades separadas. O avatar físico conserva a trajetória segura da
    // cápsula sem apagar a resposta angular produzida por contatos e torque.
    float rootTranslationAuthority = 0.0f;
    float rootRotationAuthority = 0.0f;
    bool releaseOnInteraction = false;
    Vec3 rootPositionWorld;
    Quaternion rootOrientationWorld;
    Vec3 rootLinearVelocityWorld;
    Vec3 rootAngularVelocityWorld;
};

struct RagdollJointState3D {
    std::array<float, 3> positionRadians {};
    std::array<float, 3> velocityRadiansPerSecond {};
    // Total transmitted torque in the child joint frame after simulation.
    // Includes joint-limit reactions; it is NOT just the motor command.
    std::array<float, 3> transmittedTorqueNewtonMeters {};
    // Telemetria do motor (E5), por eixo da junta, do ultimo passo: o torque
    // medio do motor (impulso do motor / dt), o da restricao angular sem o
    // motor (limites e eixos travados), o torque de antecipacao aplicado
    // (inclui a compensacao de gravidade), o limite nominal do motor e os
    // alvos efetivos (depois do limite da junta). Positivo = torque sobre o
    // filho no sentido do eixo. O feedforward consome o mesmo orcamento: o
    // motor so tem [-limite - ff, limite - ff]. Hoje so o backend Jolt
    // preenche (PhysX deixa zero).
    std::array<float, 3> motorTorqueNewtonMeters {};
    std::array<float, 3> constraintTorqueNewtonMeters {};
    std::array<float, 3> feedforwardTorqueNewtonMeters {};
    std::array<float, 3> motorTorqueLimitNewtonMeters {};
    std::array<float, 3> targetPositionRadians {};
    std::array<float, 3> targetVelocityRadiansPerSecond {};
};

struct RagdollState3D {
    std::vector<PhysicsBodyState3D> links;
    // Mesmo índice de `links`; a raiz e eixos bloqueados permanecem zerados.
    std::vector<RagdollJointState3D> joints;
    // Contatos semânticos do último passo concluído. Apenas links marcados
    // como contactSensor no perfil geram este stream.
    std::vector<RagdollContactPoint3D> contacts;
    std::vector<RagdollInteraction3D> interactions;
    float rigidityPercent = 0.0f;
    float animationAuthority = 0.0f;
    float externalInterference = 0.0f;
    bool active = false;
    bool sleeping = false;
    // Pausa de inspeção: a articulation preserva a pose fora do solver até
    // ser reativada. Não representa sono físico normal.
    bool frozen = false;
};

// Snapshot backend-neutral da dinâmica de uma articulação flutuante. O layout
// generalizado é [raiz linear XYZ, raiz angular XYZ, DOFs articulares]; é por
// isso que `jointGeneralizedDof` conta a partir de 6.
//
// Já houve aqui matriz de massa, Jacobiano denso, matriz de momento centroidal
// e força de bias — todos vindos de graça do `PxArticulationCache`, porque o
// PhysX simula a articulação em coordenadas reduzidas. Foram removidos na
// migração para Jolt, e não por causa dela: uma varredura da árvore mostrou
// **zero** consumidores fora de `Engine/Physics`. O controlador QP de corpo
// inteiro que os justificava (Eigen + ProxQP) saiu da árvore antes, e mesmo
// assim eles continuavam sendo computados por ragdoll, por tick, para serem
// descartados.
//
// Se um controlador de corpo inteiro voltar, a matemática entra em
// `Engine/Physics/Articulation/` — lado neutro, sem SDK, testável sem cena —,
// não num campo preenchido por um backend específico.
struct RagdollDynamics3D {
    static constexpr std::uint32_t InvalidIndex =
        std::numeric_limits<std::uint32_t>::max();

    // Inclui os 6 DOFs livres da raiz flutuante.
    std::uint32_t generalizedDofCount = 0;
    Vec3 centerOfMass;
    // Mesmo índice de links/joints do perfil; o valor já inclui os 6 DOFs
    // livres da raiz. Eixos bloqueados permanecem InvalidIndex.
    std::vector<std::array<std::uint32_t, 3>> jointGeneralizedDof;
    bool valid = false;
};

// Cena autoritativa: atores, consultas, CCT e eventos pertencem ao mesmo
// broad phase. A aplicacao guarda handles e le snapshots depois de simulate /
// fetchResults; ela nunca integra corpos nem escreve transforms diretamente.
class PhysicsScene3D final {
public:
    struct Impl;

    ~PhysicsScene3D();
    PhysicsScene3D(const PhysicsScene3D&) = delete;
    PhysicsScene3D& operator=(const PhysicsScene3D&) = delete;

    [[nodiscard]] PhysicsBodyHandle3D createBody(
        const PhysicsBodyDefinition3D& definition,
        std::span<const PhysicsShape3D> shapes);
    void destroyBody(PhysicsBodyHandle3D body);
    void clear();

    [[nodiscard]] bool contains(PhysicsBodyHandle3D body) const;
    [[nodiscard]] PhysicsBodyState3D bodyState(
        PhysicsBodyHandle3D body) const;
    [[nodiscard]] float bodyMass(PhysicsBodyHandle3D body) const;
    void setBodyFrozen(PhysicsBodyHandle3D body, bool frozen);
    [[nodiscard]] bool bodyFrozen(PhysicsBodyHandle3D body) const;
    void wakeBody(PhysicsBodyHandle3D body);
    void applyForceAtPoint(PhysicsBodyHandle3D body, Vec3 force,
        Vec3 worldPoint);
    void applyTorque(PhysicsBodyHandle3D body, Vec3 torque);

    // Uma articulation é um recurso único, não vinte rigid bodies expostos.
    // O handle e os snapshots mantêm PhysX confinado ao backend.
    [[nodiscard]] RagdollHandle3D createRagdoll(
        const RagdollProfile3D& profile,
        const RagdollSpawnDefinition3D& definition);
    void destroyRagdoll(RagdollHandle3D ragdoll);
    [[nodiscard]] bool contains(RagdollHandle3D ragdoll) const;
    [[nodiscard]] RagdollState3D ragdollState(
        RagdollHandle3D ragdoll) const;
    // Consulta pura: era nao-const porque o caminho PhysX escrevia no
    // PxArticulationCache para satisfazer o contrato das rotinas de dinamica
    // inversa. Sem essas rotinas, nada e mutado.
    [[nodiscard]] RagdollDynamics3D ragdollDynamics(
        RagdollHandle3D ragdoll) const;
    // Alterações de drives são enfileiradas e aplicadas no ponto seguro
    // anterior ao próximo simulate(), nunca durante o solver.
    void setRagdollRigidity(RagdollHandle3D ragdoll,
        float rigidityPercent);
    void captureRagdollPose(RagdollHandle3D ragdoll);
    void setRagdollNeutralPose(RagdollHandle3D ragdoll);
    void releaseRagdollDrives(RagdollHandle3D ragdoll);
    // Alterna entre controle ativo e o laboratório passivo de rigidez.
    // O backend só executa os comandos; planejamento e equilíbrio pertencem
    // ao módulo neutro Engine/Control.
    void setRagdollActive(RagdollHandle3D ragdoll, bool active);
    // Congelamento de diagnóstico. É aplicado no safe point da simulação e
    // preserva a pose inteira sem converter os links em corpos cinemáticos.
    void setRagdollFrozen(RagdollHandle3D ragdoll, bool frozen);
    // Substitui, de forma atômica no próximo safe point, todos os alvos de
    // controle ativo deste ragdoll. A compensação usa somente torques das
    // juntas; as seis forças da raiz flutuante nunca são aplicadas.
    void setRagdollActiveDriveTargets(RagdollHandle3D ragdoll,
        std::span<const RagdollDriveTarget3D> targets,
        bool gravityCompensationEnabled);
    // Staged alongside drive targets; only the backend resolves this constraint.
    // At 1, joints/root follow the animation exactly in ordinary locomotion.
    // Contact/interaction releases it before publishing the simulated state.
    void setRagdollAnimationConstraint(RagdollHandle3D ragdoll,
        const RagdollAnimationConstraint3D& constraint);
    // Legacy research query kept for diagnostic tools. Gameplay locomotion
    // uses the persistent character controller below.
    [[nodiscard]] CapsuleTraversalResult3D solveCapsuleTraversal(
        const CapsuleTraversalQuery3D& query) const;
    [[nodiscard]] GroundProbeResult3D probeGround(
        Vec3 capsuleCenterWorld, float radiusMeters,
        float cylinderHalfHeightMeters, float probeDistanceMeters,
        float maximumSlopeDegrees) const;
    // Como probeGround, mas ve tambem os corpos dinamicos (props, caixas):
    // o terreno que os pes pisam e por onde o passo passa. Nunca ve os
    // personagens (articulados) nem a capsula.
    [[nodiscard]] GroundProbeResult3D probeTerrain(
        Vec3 capsuleCenterWorld, float radiusMeters,
        float cylinderHalfHeightMeters, float probeDistanceMeters,
        float maximumSlopeDegrees) const;
    // Assistência externa deliberada do controlador de animação física.
    // Deve ser limitada, explicitamente instrumentada e desligável.
    // O corpo continua dinâmico: nenhuma assistência escreve transforms.
    void applyRagdollRootForce(RagdollHandle3D ragdoll,
        Vec3 forceNewtons, Vec3 torqueNewtonMeters);
    // Comando interno do controlador corporal. Não é classificado como
    // interferência externa, portanto não aciona a liberação por impacto.
    void applyRagdollControlRootForce(RagdollHandle3D ragdoll,
        Vec3 forceNewtons, Vec3 torqueNewtonMeters);
    // Versão distribuída do mesmo contrato para um link específico. Usada
    // pela assistência residual da coluna: força/torque são aplicados ao
    // centro de massa do link, no mundo, e nunca alteram sua pose.
    void applyRagdollLinkForce(RagdollHandle3D ragdoll,
        std::uint32_t linkIndex, Vec3 forceNewtons,
        Vec3 torqueNewtonMeters);
    // Comando interno do controlador (como applyRagdollControlRootForce), num
    // link qualquer: nao conta como interferencia externa.
    void applyRagdollControlLinkForce(RagdollHandle3D ragdoll,
        std::uint32_t linkIndex, Vec3 forceNewtons,
        Vec3 torqueNewtonMeters);

    [[nodiscard]] bool raycast(const Ray3D& ray, float maximumDistance,
        PhysicsRayHit3D& hit) const;
    // Consulta apenas atores dinamicos gerenciados pela engine. Ferramentas
    // como a Physgun usam esta variante para que piso, paredes e o controller
    // do personagem nao roubem a selecao do prop.
    [[nodiscard]] bool raycastDynamic(const Ray3D& ray,
        float maximumDistance, PhysicsRayHit3D& hit) const;
    [[nodiscard]] bool raycastStatic(const Ray3D& ray,
        float maximumDistance, PhysicsRayHit3D& hit) const;
    // Varredura de uma esfera pequena ao longo do raio, em vez de um raio
    // infinitamente fino. Props dinamicos usam decomposicao convexa (V-HACD),
    // cujo casco de colisao pode ficar visivelmente menor que a malha visual
    // em partes finas/detalhadas — esse "raio gordo" existe para a Physgun
    // (ou qualquer mira) nao exigir precisao de pixel nessas bordas.
    [[nodiscard]] bool sweepSphere(const Ray3D& ray, float radius,
        float maximumDistance, PhysicsRayHit3D& hit) const;
    // Equivalente dinamico da varredura. A oclusao pelo mundo deve ser
    // calculada separadamente com raycastStatic e usada como distancia
    // maxima, preservando paredes sem deixar o piso bloquear a mira assistida.
    [[nodiscard]] bool sweepSphereDynamic(const Ray3D& ray, float radius,
        float maximumDistance, PhysicsRayHit3D& hit) const;
    // Consultas exclusivas para links de articulation. Mantê-las separadas
    // evita fingir que cada osso é um rigid body público independente.
    [[nodiscard]] bool raycastRagdoll(const Ray3D& ray,
        float maximumDistance, PhysicsRagdollRayHit3D& hit) const;
    [[nodiscard]] bool sweepSphereRagdoll(const Ray3D& ray, float radius,
        float maximumDistance, PhysicsRagdollRayHit3D& hit) const;
    // Consulta conservadora de ocupação usada antes de criar entidades.
    // O volume ainda não pertence à cena, portanto não existe ator próprio
    // para ignorar. Retorna true quando qualquer shape estática ou dinâmica
    // (inclusive links de articulation e o controller) toca a caixa.
    [[nodiscard]] bool overlapsBox(Vec3 center, Vec3 halfExtents,
        Quaternion orientation = {}) const;

    // A Physgun usa um D6 drive do proprio PhysX. O alvo e cinematico, mas o
    // prop continua dinamico e chega a ele apenas por impulso/forca do solver.
    [[nodiscard]] bool beginGrab(PhysicsBodyHandle3D body,
        Vec3 localGrabPoint, const PhysicsGrabTarget3D& target,
        const PhysicsHandleSettings3D& settings);
    [[nodiscard]] bool beginRagdollGrab(RagdollHandle3D ragdoll,
        std::uint32_t linkIndex, Vec3 localGrabPoint,
        const PhysicsGrabTarget3D& target,
        const PhysicsHandleSettings3D& settings);
    void updateGrabTarget(const PhysicsGrabTarget3D& target,
        const PhysicsHandleSettings3D& settings);
    void endGrab();
    [[nodiscard]] bool grabbing() const;
    [[nodiscard]] PhysicsBodyHandle3D grabbedBody() const;
    [[nodiscard]] RagdollHandle3D grabbedRagdoll() const;
    [[nodiscard]] std::uint32_t grabbedRagdollLink() const;

    void createCharacter(Vec3 feetPosition,
        const CharacterMotorSettings3D& settings);
    void destroyCharacter();
    void placeCharacter(Vec3 feetPosition,
        const CharacterMotorSettings3D& settings);
    void moveCharacter(const CharacterMotorCommand3D& command,
        const CharacterMotorSettings3D& settings, float deltaTime);
    [[nodiscard]] bool hasCharacter() const;
    [[nodiscard]] const PhysicsCharacterState3D& characterState() const;

    // Sobrescreve PhysicsSceneSettings3D::airVelocity apos a criacao da cena
    // (o resto das configuracoes so e lido uma vez, na construcao). Usado
    // pelo WindSystem para alimentar o arrasto aerodinamico (ja existente em
    // simulate()) com um vento que varia a cada quadro - nenhuma formula de
    // arrasto nova, so o valor de entrada que antes ficava sempre zero.
    void setAirVelocity(Vec3 velocity);

    // Configura o oceano procedural sem adicionar collider sólido ao PhysX.
    void setOcean(const OceanVolume3D& ocean);
    void clearOcean();
    // Mantém a superfície física no mesmo instante da superfície visual.
    void setOceanTimeSeconds(float timeSeconds);

    void simulate(float deltaTime);
    [[nodiscard]] std::span<const ContactImpactEvent3D>
        contactImpacts() const;
    // Contatos ainda tocando neste passo (ver ContactSlideEvent3D) - usado
    // pelo som de arrasto/atrito, populado junto de contactImpacts() dentro
    // de simulate() e limpo a cada passo do mesmo jeito.
    [[nodiscard]] std::span<const ContactSlideEvent3D>
        contactSlides() const;
    [[nodiscard]] const PhysicsStepDiagnostics3D& diagnostics() const;
    [[nodiscard]] std::span<const PhysicsBodyStateUpdate3D>
        activeBodyStates() const;

private:
    PhysicsScene3D(PhysicsEngine3D& engine,
        const PhysicsSceneSettings3D& settings,
        const MaterialLibrary& materials);

    std::unique_ptr<Impl> m_impl;

    friend class PhysicsEngine3D;
};

} // namespace MatterEngine
