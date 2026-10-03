#include "Engine/Physics/Jolt/JoltInternals3D.hpp"
#include "Engine/Physics/Jolt/JoltShapes3D.hpp"
#include "Engine/Environment/OceanSurface.hpp"
#include <Jolt/Physics/Collision/CollisionCollectorImpl.h>
#include <Jolt/Physics/Collision/CollideShape.h>
#include <algorithm>
#include <cmath>
#include <stdexcept>
namespace MatterEngine {
namespace {
Vec3 moveToward(Vec3 value,Vec3 target,float distance) {
    const auto delta=target-value; const float n=delta.length();
    return n<=distance?target:value+delta*(distance/n);
}
class CharacterLayers final : public JPH::ObjectLayerFilter {
public:
    explicit CharacterLayers(bool ignore, bool proxy = false) : ignore(ignore), proxy(proxy) {}
    bool ShouldCollide(JPH::ObjectLayer layer) const override {
        if (proxy) return layer == JoltObjectLayers::NonMoving;
        return layer != JoltObjectLayers::Character && (!ignore || layer != JoltObjectLayers::Ragdoll);
    }
    bool ignore;
    bool proxy;
};
JPH::RefConst<JPH::Shape> capsule(float height,float radius) {
    PhysicsShape3D shape;
    shape.type=PhysicsShapeType3D::Capsule; shape.radius=radius;
    shape.capsuleHalfHeight=std::max(0.001f,height*0.5f-radius);
    shape.localPosition={0,0,height*0.5f};
    shape.localOrientation=Quaternion::fromAxisAngle({1,0,0},JPH::JPH_PI*0.5f);
    return createJoltShape(shape,false);
}
bool capsuleClear(const PhysicsScene3D::Impl& impl,float height,float radius,bool ignore) {
    auto shape=capsule(height,radius);
    JPH::AnyHitCollisionCollector<JPH::CollideShapeCollector> collector;
    JPH::CollideShapeSettings settings;
    // Ignore mere floor contact when deciding whether standing or leaving flight is possible.
    auto feet=impl.character->GetPosition()+JPH::Vec3(0,0,0.025f);
    impl.system->GetNarrowPhaseQuery().CollideShape(shape,JPH::Vec3::sReplicate(1),
        JPH::RMat44::sTranslation(feet+shape->GetCenterOfMass()),settings,JPH::RVec3::sZero(),collector,{},CharacterLayers(ignore));
    return !collector.HadHit();
}
}
void PhysicsScene3D::Impl::releaseCharacter() { character.reset(); characterState={}; coyoteRemaining=jumpBufferRemaining=0; }
void PhysicsScene3D::createCharacter(Vec3 feet,const CharacterMotorSettings3D& settings) {
    m_impl->releaseCharacter();
    JPH::CharacterVirtualSettings config;
    config.mUp=JPH::Vec3::sAxisZ();
    config.mSupportingVolume=JPH::Plane(JPH::Vec3::sAxisZ(),-settings.radius);
    config.mMaxSlopeAngle=settings.maximumSlopeDegrees*JPH::JPH_PI/180;
    config.mCharacterPadding=settings.skinWidth;
    config.mShape=capsule(settings.standingHeight,settings.radius);
    config.mInnerBodyShape=config.mShape;
    config.mInnerBodyLayer=JoltObjectLayers::Character;
    config.mMass=80;
    config.mMaxStrength=80*settings.maximumPushSpeedMetersPerSecond*120;
    m_impl->character=std::make_unique<JPH::CharacterVirtual>(&config,toJolt(feet),JPH::Quat::sIdentity(),
        JoltDetail::packBodyIdentity({JoltDetail::BodyKind::Character,0,0}),m_impl->system.get());
    m_impl->character->SetListener(m_impl.get());
    m_impl->characterSettings=settings;
    m_impl->characterState.position=feet+Vec3{0,0,settings.standingHeight*0.5f};
}
void PhysicsScene3D::destroyCharacter() { m_impl->releaseCharacter(); }
void PhysicsScene3D::placeCharacter(Vec3 feet,const CharacterMotorSettings3D& settings) {
    createCharacter(feet,settings);
}
void PhysicsScene3D::moveCharacter(const CharacterMotorCommand3D& command,
    const CharacterMotorSettings3D& settings, float deltaTime) {
    if (m_impl->character == nullptr || deltaTime <= 0.0f) return;
    PhysicsCharacterState3D& state = m_impl->characterState;
    state.flightExitBlocked = false;

    if (command.toggleFlight) {
        if (state.flying) {
            const float activeHeight = state.crouched
                ? settings.crouchedHeight : settings.standingHeight;
            if (capsuleClear(*m_impl,activeHeight,
                    settings.radius, command.ignoreRagdolls)) {
                state.flying = false;
                // A velocidade de voo no instante da troca e preservada; a
                // gravidade e o controle aereo voltam a atuar normalmente.
            } else {
                state.flightExitBlocked = true;
            }
        } else {
            state.flying = true;
            state.velocity = {};
        }
    }

    const bool wantsCrouch = command.crouch;
    if (wantsCrouch != state.crouched) {
        const float targetHeight = wantsCrouch
            ? settings.crouchedHeight : settings.standingHeight;
        // Reduzir a capsula sempre e seguro. Para levantar, um overlap explicito
        // impede que resize() coloque o jogador dentro de um teto ou prop.
        if (wantsCrouch || capsuleClear(*m_impl,targetHeight,
                settings.radius, command.ignoreRagdolls)) {
            auto shape=capsule(targetHeight,settings.radius);
            if (!m_impl->character->SetShape(shape, 1.0e10f, {}, CharacterLayers(command.ignoreRagdolls), {}, {}, *m_impl->tempAllocator)) return;
            m_impl->character->SetInnerBodyShape(shape);
            state.crouched = wantsCrouch;
        }
    }

    Vec3 inputDirection = command.moveDirection;
    if (inputDirection.lengthSquared() > 1.0f) {
        inputDirection = inputDirection.normalized();
    }

    const float activeBodyHeight = state.crouched
        ? settings.crouchedHeight : settings.standingHeight;
    bool oceanContainsCharacter = false;
    float immersionAtFeet = 0.0f;
    if (m_impl->ocean) {
        const OceanVolume3D& ocean = *m_impl->ocean;
        const Vec2 local {
            state.position.x - ocean.center.x,
            state.position.y - ocean.center.y
        };
        const float feetHeight =
            state.position.z - activeBodyHeight * 0.5f;
        const bool horizontal =
            std::abs(local.x) <= ocean.halfExtents.x
            && std::abs(local.y) <= ocean.halfExtents.y;
        const bool aboveOceanFloor =
            feetHeight >= ocean.meanSeaLevelMeters - ocean.depthMeters;
        if (horizontal && aboveOceanFloor) {
            const float surface = evaluateOceanSurface(
                { state.position.x, state.position.y },
                ocean.meanSeaLevelMeters, m_impl->oceanTimeSeconds)
                    .heightMeters;
            oceanContainsCharacter = true;
            immersionAtFeet = surface - feetHeight;
        }
    }
    if (state.flying || !oceanContainsCharacter) {
        state.swimming = false;
    } else {
        // Água rasa continua permitindo caminhar. A transição para natação
        // acontece quando o tronco está imerso e usa duas profundidades
        // diferentes para não oscilar na superfície.
        state.swimming = state.swimming
            ? immersionAtFeet > 0.38f
            : immersionAtFeet > 0.72f;
    }

    if (state.flying) {
        const float speed = command.sprint
            ? settings.fastFlightSpeed : settings.flightSpeed;
        // Voo editorial deliberadamente sem inercia: soltar a tecla zera o
        // movimento, mantendo controle preciso. A saida do modo preserva a
        // velocidade corrente conforme tratado acima.
        state.velocity = inputDirection * speed;
    } else if (state.swimming) {
        const float speed = command.sprint
            ? settings.fastSwimSpeed : settings.swimSpeed;
        const Vec3 desired = inputDirection * speed;
        // A aproximação vetorial funciona como arrasto hidrodinâmico sem
        // cancelar a inércia em um único quadro.
        state.velocity = moveToward(state.velocity, desired,
            settings.swimAcceleration * deltaTime);
        state.grounded = false;
        m_impl->coyoteRemaining = 0.0f;
        m_impl->jumpBufferRemaining = 0.0f;
    } else {
        const float speed = (state.crouched ? settings.crouchedSpeed
            : command.sprint ? settings.sprintSpeed : settings.walkSpeed)
            * std::clamp(command.speedScale, 0.0f, 2.0f);
        Vec3 desired = inputDirection;
        desired.z = 0.0f;
        if (desired.lengthSquared() > 1.0f) desired = desired.normalized();
        desired *= speed;
        const Vec3 currentPlanar { state.velocity.x, state.velocity.y, 0.0f };
        const float acceleration = state.grounded
            ? (desired.lengthSquared() > 0.0f
                ? settings.groundAcceleration : settings.groundDeceleration)
            : settings.airAcceleration;
        const Vec3 planar = moveToward(currentPlanar, desired,
            acceleration * deltaTime);
        state.velocity.x = planar.x;
        state.velocity.y = planar.y;

        m_impl->coyoteRemaining = state.grounded
            ? settings.coyoteTime
            : std::max(0.0f, m_impl->coyoteRemaining - deltaTime);
        if (command.jumpPressed) {
            m_impl->jumpBufferRemaining = settings.jumpBufferTime;
            m_impl->jumpChargeSeconds = command.jumpChargeSeconds;
        } else {
            m_impl->jumpBufferRemaining = std::max(0.0f,
                m_impl->jumpBufferRemaining - deltaTime);
        }
        const bool standingStill = desired.lengthSquared() < 0.000001f
            && planar.lengthSquared() < 0.000001f;
        if (m_impl->jumpBufferRemaining > 0.0f
            && m_impl->coyoteRemaining > 0.0f) {
            state.velocity.z = characterJumpSpeed3D(settings,
                m_impl->jumpChargeSeconds);
            state.grounded = false;
            m_impl->jumpBufferRemaining = 0.0f;
            m_impl->coyoteRemaining = 0.0f;
        } else if (standingStill && m_impl->characterSupported) {
            // Parado e apoiado: quem segura o corpo sao as pernas, nao a base
            // redonda da capsula. Numa quina de degrau essa base encosta no
            // espelho de cima e escorregava escada abaixo (24 cm em 4 s).
            state.velocity = {};
        } else {
            state.velocity.z = std::max(-settings.maximumFallSpeed,
                state.velocity.z - 9.81f * settings.gravityScale * deltaTime);
        }
    }

    m_impl->characterSettings = settings;
    m_impl->characterMoveVelocity = state.velocity;
    m_impl->characterDeltaTime = deltaTime;
    m_impl->characterIgnoreRagdolls = command.ignoreRagdolls;
    m_impl->characterNavigationProxy = command.navigationProxy;
    m_impl->callbackFailed = false;
    auto& character=*m_impl->character;
    if (state.flying) {
        character.SetPosition(character.GetPosition()+toJolt(state.velocity*deltaTime));
    } else {
        const Vec3 follow { command.followVelocity.x,
            command.followVelocity.y, 0.0f };
        const Vec3 intended = state.velocity;
        character.SetLinearVelocity(toJolt(state.velocity + follow));
        JPH::CharacterVirtual::ExtendedUpdateSettings update;
        // Descendo escada, a capsula fica colada no chao ate um degrau (e
        // um pouco): com 15 cm fixos, cada degrau de 18 cm virava um instante
        // no ar - pose de queda a cada passo.
        update.mStickToFloorStepDown={0,0,state.velocity.z>0?0.0f
            :-std::max(0.15f,settings.maximumStepHeight+0.05f)};
        update.mWalkStairsStepUp={0,0,settings.maximumStepHeight};
        character.ExtendedUpdate(deltaTime,toJolt(m_impl->settings.gravity),update,{},CharacterLayers(command.ignoreRagdolls,command.navigationProxy),{},{},*m_impl->tempAllocator);
        state.velocity=fromJolt(character.GetLinearVelocity());
        if (follow.lengthSquared() > 0.0f) {
            // A inercia do controlador fica sem o seguimento; bater numa
            // parede enquanto segue o corpo nunca inverte o movimento.
            state.velocity.x -= follow.x;
            state.velocity.y -= follow.y;
            if (state.velocity.x * intended.x < 0.0f) state.velocity.x = 0.0f;
            if (state.velocity.y * intended.y < 0.0f) state.velocity.y = 0.0f;
        }
    }
    if (m_impl->callbackFailed) throw std::runtime_error("Jolt: character contact failed");
    state.grounded=!state.flying&&!state.swimming&&character.GetGroundState()==JPH::CharacterBase::EGroundState::OnGround;
    // Encostada em "chao ingreme" com chao pisavel ate um degrau abaixo dos
    // pes, a capsula esta apoiada: e a quina de um degrau vista pela base
    // redonda dela. (No ar ela nunca conta como apoiada - pairaria.)
    m_impl->characterSupported = state.grounded;
    if (!state.grounded && !state.flying && !state.swimming
        && character.GetGroundState()
            == JPH::CharacterBase::EGroundState::OnSteepGround) {
        const float step = settings.maximumStepHeight;
        const float probeRadius = settings.radius * 0.35f;
        const Vec3 feet = fromJolt(character.GetPosition());
        const GroundProbeResult3D below = probeGround(
            feet + Vec3 { 0.0f, 0.0f, step + probeRadius }, probeRadius, 0.0f,
            2.0f * step + probeRadius, settings.maximumSlopeDegrees);
        if (below.hasSurface && below.walkable) {
            m_impl->characterSupported = true;
            state.grounded = true;
        }
    }
    if (state.grounded&&state.velocity.z<0) state.velocity.z=0;
    state.position=fromJolt(character.GetPosition())+Vec3{0,0,activeBodyHeight*0.5f};
}
bool PhysicsScene3D::hasCharacter() const { return m_impl->character!=nullptr; }
const PhysicsCharacterState3D& PhysicsScene3D::characterState() const { return m_impl->characterState; }
void PhysicsScene3D::Impl::OnContactAdded(const JPH::CharacterVirtual*,
    const JPH::CharacterContact& contact, JPH::CharacterContactSettings& config) {
    try { pushCharacterContact(contact, config); } catch (...) { callbackFailed = true; }
}
void PhysicsScene3D::Impl::OnContactPersisted(const JPH::CharacterVirtual*,
    const JPH::CharacterContact& contact, JPH::CharacterContactSettings& config) {
    try { pushCharacterContact(contact, config); } catch (...) { callbackFailed = true; }
}
void PhysicsScene3D::Impl::pushCharacterContact(const JPH::CharacterContact& contact,
    JPH::CharacterContactSettings& config) {
    config.mCanReceiveImpulses = false;
    if (contact.mMotionTypeB != JPH::EMotionType::Dynamic || contact.mIsSensorB) return;
    const auto id = JoltDetail::unpackBodyIdentity(contact.mUserData);
    if (id.kind != JoltDetail::BodyKind::Body || id.slotIndex >= bodySlots.size()) return;
    const auto* r = bodySlots[id.slotIndex].record.get();
    if (!r || r->frozen || r->bodyId != contact.mBodyB) return;
    Vec3 direction = -fromJolt(contact.mContactNormal);
    direction.z = 0;
    if (direction.lengthSquared() < 1e-6f) return;
    direction = direction.normalized();
    const float requested = std::max(0.0f, dot(characterMoveVelocity, direction));
    const float fraction = std::clamp(requested * characterDeltaTime /
        std::max(0.001f, characterSettings.pushSaturationPenetrationMeters), 0.0f, 1.0f);
    const float targetSpeed = std::min(requested, characterSettings.maximumPushSpeedMetersPerSecond) * fraction;
    auto& bi = system->GetBodyInterface();
    const float currentSpeed = dot(fromJolt(bi.GetLinearVelocity(r->bodyId)), direction);
    const float deltaSpeed = std::max(0.0f, targetSpeed-currentSpeed);
    if (deltaSpeed > 0) bi.AddImpulse(r->bodyId, toJolt(direction * (r->definition.massKg * deltaSpeed)), contact.mPosition);
}
} // namespace MatterEngine
