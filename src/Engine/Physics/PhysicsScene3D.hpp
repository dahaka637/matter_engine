#pragma once

#include "Engine/Math/Ray3D.hpp"
#include "Engine/Physics/PhysicsEngine3D.hpp"

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

struct CharacterMotorCommand3D {
    Vec3 moveDirection;
    bool sprint = false;
    bool crouch = false;
    bool jumpPressed = false;
    bool toggleFlight = false;
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
    float gravityScale = 1.0f;
    float maximumFallSpeed = 48.0f;
    float maximumSlopeDegrees = 50.0f;
    float maximumStepHeight = 0.38f;
    float skinWidth = 0.018f;
    float coyoteTime = 0.10f;
    float jumpBufferTime = 0.12f;
    float flightSpeed = 12.0f;
    float fastFlightSpeed = 28.0f;
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

struct PhysicsCharacterState3D {
    Vec3 position;
    Vec3 velocity;
    bool grounded = false;
    bool crouched = false;
    bool flying = false;
    bool flightExitBlocked = false;
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

    // A Physgun usa um D6 drive do proprio PhysX. O alvo e cinematico, mas o
    // prop continua dinamico e chega a ele apenas por impulso/forca do solver.
    [[nodiscard]] bool beginGrab(PhysicsBodyHandle3D body,
        Vec3 localGrabPoint, const PhysicsGrabTarget3D& target,
        const PhysicsHandleSettings3D& settings);
    void updateGrabTarget(const PhysicsGrabTarget3D& target,
        const PhysicsHandleSettings3D& settings);
    void endGrab();
    [[nodiscard]] bool grabbing() const;
    [[nodiscard]] PhysicsBodyHandle3D grabbedBody() const;

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

    void simulate(float deltaTime);
    [[nodiscard]] std::span<const ContactImpactEvent3D>
        contactImpacts() const;
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
