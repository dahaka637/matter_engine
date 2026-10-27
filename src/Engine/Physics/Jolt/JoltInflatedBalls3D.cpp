#include "Engine/Physics/Jolt/JoltInternals3D.hpp"
#include "Engine/Physics/Jolt/JoltShapes3D.hpp"

#include <Jolt/Physics/Body/BodyLock.h>
#include <Jolt/Physics/Collision/BroadPhase/BroadPhaseQuery.h>
#include <Jolt/Physics/Collision/CollideShape.h>
#include <Jolt/Physics/Collision/CollisionCollectorImpl.h>
#include <Jolt/Physics/Collision/CollisionDispatch.h>
#include <Jolt/Physics/Collision/Shape/CompoundShape.h>
#include <Jolt/Physics/Collision/Shape/DecoratedShape.h>
#include <Jolt/Physics/Collision/Shape/SphereShape.h>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <stdexcept>

namespace MatterEngine {

// Bola inflada (InflatedBall3D.hpp). Por tick: os corpos que a varredura da
// bola pode tocar sao coletados uma vez; nos subpassos, cada um entra na pose
// que tinha naquele instante (a do fim do passo do Jolt recuada pela
// velocidade dele - um pe a 20 m/s anda 17 cm por tick) mais o que a reacao da
// bola ja o deslocou. A bola integra forca de contato, ar e gravidade; a
// reacao vai para os corpos dinamicos como impulso antes do passo seguinte.

namespace {

// Repulsao macia: o dono da bola (setInflatedBallOwner) nao bate nela - o
// quadril e as pernas a atravessam (ele maneja sem chuta-la sem querer;
// pedido do usuario, 05/10 noite) e tronco, cabeca e bracos so a empurram se
// ela entrar neles, por uma mola sem atrito que comeca SoftMargin (m) antes
// de encostar: forca = SoftStiffness * (a folga invadida) + SoftDamping * (a
// aproximacao), ate SoftMaximumForce (N); o corpo recebe a reacao igual e
// contraria. (Antes, 04/10, o corpo inteiro repelia, 5 cm antes de encostar:
// a mola nas pernas seguia empurrando a bola no manejo. Com 260 N/m e 60 N
// uma bola a 9 m/s atravessava o corpo; com 2000 N/m e 300 N fica fora,
// freada e desviada, sem quique duro.)
float softTune(const char* name, float fallback) {
    const char* value = std::getenv(name);
    return value != nullptr ? std::strtof(value, nullptr) : fallback;
}
// TEMPTUNE.
const float SoftMargin = softTune("T_BallSoftMargin", 0.0f);
const float SoftStiffness = softTune("T_BallSoftStiffness", 2000.0f);
const float SoftDamping = softTune("T_BallSoftDamping", 15.0f);
const float SoftMaximumForce = softTune("T_BallSoftMax", 300.0f);

// O corpo interno do character controller nao toca a bola: quem chuta e o
// corpo (ragdoll), nao a capsula de navegacao. A barreira dos jogadores
// tambem nao (a bola bate na rede).
class BallObjectLayerFilter final : public JPH::ObjectLayerFilter {
public:
    bool ShouldCollide(JPH::ObjectLayer layer) const override {
        return layer != JoltObjectLayers::Character && layer != JoltObjectLayers::PlayerBarrier
            && layer != JoltObjectLayers::CapsuleBarrier;
    }
};

// O material de uma forma: num composto (a trave: um tubo por cilindro), o da
// primeira peca - o ID de sub-forma vazio nao vale para o composto e lia fora
// do vetor de pecas.
const JPH::PhysicsMaterial* firstShapeMaterial(const JPH::Shape* shape) {
    for (;;) {
        if (shape->GetType() == JPH::EShapeType::Compound) {
            const auto* compound = static_cast<const JPH::CompoundShape*>(shape);
            if (compound->GetNumSubShapes() == 0) return nullptr;
            shape = compound->GetSubShape(0).mShape;
        } else if (shape->GetType() == JPH::EShapeType::Decorated) {
            shape = static_cast<const JPH::DecoratedShape*>(shape)->GetInnerShape();
        } else {
            return shape->GetMaterial(JPH::SubShapeID());
        }
    }
}

struct BallCandidate {
    JPH::BodyID id;
    JPH::RefConst<JPH::Shape> shape;
    JPH::RMat44 endTransform;
    Vec3 linearVelocity;
    Vec3 angularVelocity;
    Vec3 centerOfMassEnd;
    float inverseMass = 0.0f;
    float friction = 0.5f;
    std::string material = "default";
    BallSurfaceProperties surface;
    bool dynamic = false;
    // Link de ragdoll: de quem e qual. soft: o link nao colide com a bola,
    // so a repulsao macia.
    bool ragdoll = false;
    bool soft = false;
    // Sensor (setInflatedBallSensor): so mede o encosto, sem forca.
    bool sensor = false;
    std::uint64_t entityId = 0;
    std::uint32_t link = 0;
    // Dentro do tick: o que a reacao da bola mudou na velocidade e na posicao
    // deste corpo, e o impulso a aplicar depois.
    Vec3 deltaVelocity;
    Vec3 displacement;
    Vec3 impulse;
    Vec3 impulsePointSum;
    float impulseWeight = 0.0f;
    bool touched = false;
    float touchApproach = 0.0f;
    Vec3 touchPoint;
    Vec3 touchNormal;
};

struct BallContact {
    std::size_t candidate = 0;
    Vec3 normal;
    Vec3 point;
    float depth = 0.0f;
};

// Repulsao macia: a folga invadida (m, de 0 a SoftMargin + penetracao).
struct BallSoftContact {
    std::size_t candidate = 0;
    Vec3 normal;
    Vec3 point;
    float overlap = 0.0f;
};

} // namespace

PhysicsScene3D::Impl::InflatedBallRecord* PhysicsScene3D::Impl::inflatedBall(InflatedBallHandle3D h) {
    return h.index < inflatedBallSlots.size() && inflatedBallSlots[h.index].generation == h.generation
        ? inflatedBallSlots[h.index].record.get() : nullptr;
}
const PhysicsScene3D::Impl::InflatedBallRecord* PhysicsScene3D::Impl::inflatedBall(InflatedBallHandle3D h) const {
    return h.index < inflatedBallSlots.size() && inflatedBallSlots[h.index].generation == h.generation
        ? inflatedBallSlots[h.index].record.get() : nullptr;
}

InflatedBallHandle3D PhysicsScene3D::createInflatedBall(const InflatedBallDefinition3D& definition) {
    auto& impl = *m_impl;
    const auto index = impl.freeInflatedBallSlots.empty()
        ? static_cast<std::uint32_t>(impl.inflatedBallSlots.size()) : impl.freeInflatedBallSlots.back();
    if (index == impl.inflatedBallSlots.size()) impl.inflatedBallSlots.emplace_back();
    else impl.freeInflatedBallSlots.pop_back();
    auto record = std::make_unique<Impl::InflatedBallRecord>();
    record->definition = definition;
    record->handle = { index, impl.inflatedBallSlots[index].generation };
    auto& state = record->state;
    state.position = definition.position;
    state.orientation = definition.orientation.normalized();
    state.linearVelocity = definition.linearVelocity;
    state.angularVelocity = definition.angularVelocity;
    state.gaugePressurePascals = definition.profile.gaugePressurePascals;
    const auto handle = record->handle;
    impl.inflatedBallSlots[index].record = std::move(record);
    return handle;
}

void PhysicsScene3D::destroyInflatedBall(InflatedBallHandle3D handle) {
    auto& impl = *m_impl;
    if (!impl.inflatedBall(handle)) return;
    impl.inflatedBallSlots[handle.index].record.reset();
    ++impl.inflatedBallSlots[handle.index].generation;
    impl.freeInflatedBallSlots.push_back(handle.index);
}

bool PhysicsScene3D::contains(InflatedBallHandle3D handle) const {
    return m_impl->inflatedBall(handle) != nullptr;
}

InflatedBallState3D PhysicsScene3D::inflatedBallState(InflatedBallHandle3D handle) const {
    const auto* record = m_impl->inflatedBall(handle);
    return record ? record->state : InflatedBallState3D {};
}

void PhysicsScene3D::setInflatedBallOwner(InflatedBallHandle3D handle, RagdollHandle3D owner) {
    auto* record = m_impl->inflatedBall(handle);
    if (record != nullptr) record->owner = owner;
}

RagdollHandle3D PhysicsScene3D::inflatedBallOwner(InflatedBallHandle3D handle) const {
    const auto* record = m_impl->inflatedBall(handle);
    return record != nullptr ? record->owner : RagdollHandle3D {};
}

void PhysicsScene3D::setInflatedBallMotion(InflatedBallHandle3D handle, Vec3 position,
    Vec3 linearVelocity, Vec3 angularVelocity) {
    auto* record = m_impl->inflatedBall(handle);
    if (!record) return;
    record->state.position = position;
    record->state.linearVelocity = linearVelocity;
    record->state.angularVelocity = angularVelocity;
}

void PhysicsScene3D::applyInflatedBallImpulse(InflatedBallHandle3D handle, Vec3 impulse, Vec3 worldPoint) {
    auto* record = m_impl->inflatedBall(handle);
    if (!record) return;
    const auto& profile = record->definition.profile;
    record->state.linearVelocity += impulse * (1.0f / profile.massKg);
    record->state.angularVelocity += cross(worldPoint - record->state.position, impulse)
        * (1.0f / InflatedBallModel3D::shellInertia(profile));
}

void PhysicsScene3D::applyInflatedBallImpulses(InflatedBallHandle3D handle, Vec3 linearImpulse, Vec3 angularImpulse) {
    auto* record = m_impl->inflatedBall(handle);
    if (!record) return;
    const auto& profile = record->definition.profile;
    record->state.linearVelocity += linearImpulse * (1.0f / profile.massKg);
    record->state.angularVelocity += angularImpulse * (1.0f / InflatedBallModel3D::shellInertia(profile));
}

void PhysicsScene3D::setInflatedBallSensor(InflatedBallHandle3D handle, RagdollHandle3D ragdoll, std::uint64_t linkMask,
    float marginMeters) {
    auto* record = m_impl->inflatedBall(handle);
    if (!record) return;
    record->sensorRagdoll = ragdoll;
    record->sensorLinks = linkMask;
    record->sensorMargin = std::max(0.0f, marginMeters);
}

void PhysicsScene3D::setInflatedBallFrozen(InflatedBallHandle3D handle, bool frozen) {
    auto* record = m_impl->inflatedBall(handle);
    if (!record) return;
    record->state.frozen = frozen;
    if (frozen) {
        record->state.linearVelocity = {};
        record->state.angularVelocity = {};
    }
}

InflatedBallProfile3D PhysicsScene3D::inflatedBallProfile(InflatedBallHandle3D handle) const {
    const auto* record = m_impl->inflatedBall(handle);
    return record ? record->definition.profile : InflatedBallProfile3D {};
}

void PhysicsScene3D::setInflatedBallProfile(InflatedBallHandle3D handle, const InflatedBallProfile3D& profile) {
    auto* record = m_impl->inflatedBall(handle);
    if (!record) return;
    record->definition.profile = profile;
}

bool PhysicsScene3D::raycastInflatedBall(const Ray3D& ray, float maximumDistance,
    InflatedBallHandle3D& outBall, float& outDistance) const {
    bool hit = false;
    outDistance = maximumDistance;
    const Vec3 direction = ray.direction.normalized();
    for (const auto& slot : m_impl->inflatedBallSlots) {
        if (!slot.record) continue;
        const auto& state = slot.record->state;
        const float r = slot.record->definition.profile.radiusMeters;
        const Vec3 toCenter = state.position - ray.origin;
        const float along = dot(toCenter, direction);
        const float closest2 = toCenter.lengthSquared() - along * along;
        if (closest2 > r * r) continue;
        const float t = along - std::sqrt(r * r - closest2);
        if (t < 0.0f || t > outDistance) continue;
        outDistance = t;
        outBall = slot.record->handle;
        hit = true;
    }
    return hit;
}

void PhysicsScene3D::Impl::stepInflatedBalls(float deltaTime) {
    if (inflatedBallSlots.empty() && netSlots.empty()) return;
    auto& bodies = system->GetBodyInterface();
    const BallObjectLayerFilter layerFilter;
    const Vec3 gravity = fromJolt(system->GetGravity());
    const Vec3 wind = settings.airVelocity;

    struct BallTouch {
        std::uint32_t slot = 0;
        float approach = 0.0f;
        Vec3 point;
        Vec3 normal;
    };
    struct BallWork {
        InflatedBallRecord* record = nullptr;
        std::uint32_t slot = 0;
        bool moving = false;
        std::vector<BallCandidate> candidates;
        std::vector<BallContact> contacts;
        std::vector<BallSoftContact> softContacts;
        JPH::RefConst<JPH::Shape> sphere;
        float restVolume = 0.0f;
        float volume = 0.0f;
        float lost = 0.0f;
        float inertia = 0.0f;
        Vec3 force;
        Vec3 torque;
        std::uint32_t published = 0;
        bool touching = false;
        std::vector<BallTouch> touchedBalls;
    };
    // Duas bolas que se tocam: cada casca achata metade da sobreposicao
    // contra o plano do meio (de b para a).
    struct BallPair {
        std::size_t a = 0;
        std::size_t b = 0;
        Vec3 normal;
        float overlap = 0.0f;
    };

    // Todas as bolas avancam juntas, subpasso a subpasso: duas bolas se tocam
    // no mesmo instante, com a forca de uma igual e oposta a da outra.
    std::vector<BallWork> balls;
    std::uint32_t substeps = 1;
    for (std::uint32_t slotIndex = 0; slotIndex < inflatedBallSlots.size(); ++slotIndex) {
        auto& slot = inflatedBallSlots[slotIndex];
        if (!slot.record) continue;
        BallWork ball;
        ball.record = slot.record.get();
        ball.slot = slotIndex;
        auto& state = ball.record->state;
        const auto& profile = ball.record->definition.profile;
        state.compressionMeters = 0.0f;
        state.contactCount = 0;
        state.sensorTouch = {};
        ball.restVolume = InflatedBallModel3D::restVolume(profile);
        ball.volume = ball.restVolume;
        ball.inertia = InflatedBallModel3D::shellInertia(profile);
        ball.moving = !state.frozen;
        if (ball.moving) {
            const float radius = profile.radiusMeters;
            // Candidatos: a varredura da bola no tick, com folga para corpos
            // que chegam rapido (um pe a 20 m/s anda 17 cm por tick).
            JPH::AABox box(toJolt(state.position) - JPH::Vec3::sReplicate(radius),
                toJolt(state.position) + JPH::Vec3::sReplicate(radius));
            const Vec3 ahead = state.position + state.linearVelocity * deltaTime;
            box.Encapsulate(JPH::AABox(toJolt(ahead) - JPH::Vec3::sReplicate(radius),
                toJolt(ahead) + JPH::Vec3::sReplicate(radius)));
            box.ExpandBy(JPH::Vec3::sReplicate(0.4f));
            JPH::AllHitCollisionCollector<JPH::CollideShapeBodyCollector> hits;
            system->GetBroadPhaseQuery().CollideAABox(box, hits, {}, layerFilter);
            ball.candidates.reserve(hits.mHits.size());
            for (const JPH::BodyID& id : hits.mHits) {
                JPH::BodyLockRead lock(system->GetBodyLockInterface(), id);
                if (!lock.Succeeded()) continue;
                const JPH::Body& body = lock.GetBody();
                if (body.IsSensor() || body.IsSoftBody()) continue;
                if (!ball.record->definition.collidesWithRagdolls
                    && body.GetObjectLayer() == JoltObjectLayers::Ragdoll) continue;
                BallCandidate candidate;
                // (O corpo guiado e para a bola o que o cinematico era: nao
                // cede nem leva a reacao dela.)
                bool guidedLink = false;
                const auto identity = JoltDetail::unpackBodyIdentity(body.GetUserData());
                if (identity.kind == JoltDetail::BodyKind::RagdollLink && identity.slotIndex < ragdollSlots.size()) {
                    const RagdollRecord* owner = ragdollSlots[identity.slotIndex].record.get();
                    if (owner != nullptr) {
                        guidedLink = owner->guided;
                        // O sensor: o link pedido so mede (tambem o do dono).
                        const bool sensed = ball.record->sensorLinks != 0
                            && owner->handle == ball.record->sensorRagdoll && identity.linkIndex < 64
                            && (ball.record->sensorLinks >> identity.linkIndex & 1u) != 0;
                        if (sensed) {
                            candidate.sensor = true;
                        // O dono da bola: o corpo todo a atravessa (21/10, pedido do
                        // usuario; antes o tronco, a cabeca e os bracos a repeliam).
                        } else if (owner->handle == ball.record->owner) {
                            continue;
                        }
                        candidate.ragdoll = true;
                        candidate.entityId = owner->entityId;
                        candidate.link = identity.linkIndex;
                    }
                }
                candidate.id = id;
                candidate.shape = body.GetShape();
                candidate.endTransform = body.GetCenterOfMassTransform();
                candidate.centerOfMassEnd = fromJolt(body.GetCenterOfMassPosition());
                candidate.linearVelocity = fromJolt(body.GetLinearVelocity());
                candidate.angularVelocity = fromJolt(body.GetAngularVelocity());
                candidate.dynamic = body.IsDynamic() && !guidedLink;
                candidate.inverseMass = candidate.dynamic ? body.GetMotionProperties()->GetInverseMass() : 0.0f;
                candidate.friction = body.GetFriction();
                candidate.material = std::string(joltShapeMaterialId(firstShapeMaterial(body.GetShape())));
                candidate.surface = contactMaterial(candidate.material).ball;
                ball.candidates.push_back(std::move(candidate));
            }
            ball.sphere = new JPH::SphereShape(radius);
            substeps = std::max(substeps, std::max<std::uint32_t>(1u, profile.substeps));
        }
        balls.push_back(std::move(ball));
    }

    // As redes (JoltNets3D.cpp) andam nos mesmos subpassos: acopladas a bola,
    // pelo menos os dela.
    substeps = std::max(substeps, prepareNets(deltaTime, substeps));
    const float h = deltaTime / static_cast<float>(substeps);
    std::vector<BallPair> pairs;
    for (std::uint32_t step = 0; step < substeps; ++step) {
        const float remaining = deltaTime - static_cast<float>(step + 1) * h;
        const bool lastStep = step + 1 == substeps;
        for (BallWork& ball : balls) {
            ball.contacts.clear();
            ball.softContacts.clear();
            ball.lost = 0.0f;
            ball.published = 0;
            if (!ball.moving) continue;
            const auto& state = ball.record->state;
            const auto& profile = ball.record->definition.profile;
            const float radius = profile.radiusMeters;
            const float maximumDepth = profile.maximumCompressionShare * radius;
            for (std::size_t ci = 0; ci < ball.candidates.size(); ++ci) {
                BallCandidate& candidate = ball.candidates[ci];
                JPH::RMat44 transform = candidate.endTransform;
                transform.SetTranslation(transform.GetTranslation()
                    - toJolt(candidate.linearVelocity * remaining) + toJolt(candidate.displacement));
                JPH::CollideShapeSettings collide;
                collide.mMaxSeparationDistance = candidate.sensor ? ball.record->sensorMargin
                    : candidate.soft ? SoftMargin : 0.0f;
                JPH::AllHitCollisionCollector<JPH::CollideShapeCollector> collector;
                JPH::SubShapeIDCreator first, second;
                JPH::CollisionDispatch::sCollideShapeVsShape(ball.sphere, candidate.shape,
                    JPH::Vec3::sOne(), JPH::Vec3::sOne(), JPH::Mat44::sTranslation(toJolt(state.position)),
                    JPH::Mat44(transform), first, second, collide, collector);
                if (candidate.sensor) {
                    // O sensor: o primeiro encosto do tick, sem forca.
                    auto& touch = ball.record->state.sensorTouch;
                    if (touch.touched) continue;
                    for (const auto& result : collector.mHits) {
                        const JPH::Vec3 axis = result.mPenetrationAxis;
                        if (axis.LengthSq() < 1e-12f) continue;
                        const float gap = -result.mPenetrationDepth;
                        if (gap > ball.record->sensorMargin) continue;
                        const Vec3 normal = fromJolt(axis.Normalized());
                        const Vec3 point = state.position + normal * radius;
                        const Vec3 centerNow = candidate.centerOfMassEnd - candidate.linearVelocity * remaining
                            + candidate.displacement;
                        const Vec3 linkVelocity = candidate.linearVelocity
                            + cross(candidate.angularVelocity, point - centerNow);
                        touch.touched = true;
                        touch.timeSeconds = static_cast<float>(step + 1) * h;
                        touch.pointWorld = point;
                        touch.normalWorld = normal;
                        touch.relativeVelocity = linkVelocity - state.linearVelocity;
                        touch.link = candidate.link;
                        touch.gapMeters = std::max(0.0f, gap);
                        break;
                    }
                    continue;
                }
                if (candidate.soft) {
                    // Repulsao macia: o mais fundo do link.
                    BallSoftContact deepest;
                    bool found = false;
                    for (const auto& result : collector.mHits) {
                        const JPH::Vec3 axis = result.mPenetrationAxis;
                        if (axis.LengthSq() < 1e-12f) continue;
                        const float overlap = std::min(result.mPenetrationDepth + SoftMargin, SoftMargin + maximumDepth);
                        if (overlap <= 0.0f || (found && overlap <= deepest.overlap)) continue;
                        const Vec3 normal = fromJolt(-axis.Normalized());
                        deepest = { ci, normal, state.position - normal * radius, overlap };
                        found = true;
                    }
                    if (found) ball.softContacts.push_back(deepest);
                    continue;
                }
                const std::size_t start = ball.contacts.size();
                for (const auto& result : collector.mHits) {
                    if (result.mPenetrationDepth <= 0.0f) continue;
                    const JPH::Vec3 axis = result.mPenetrationAxis;
                    if (axis.LengthSq() < 1e-12f) continue;
                    const Vec3 normal = fromJolt(-axis.Normalized());
                    const float depth = std::min(result.mPenetrationDepth, maximumDepth);
                    // Malhas: varios triangulos com a mesma normal sao um
                    // contato so (o mais fundo); dois contatos diferentes so
                    // com normais diferentes.
                    bool merged = false;
                    for (std::size_t k = start; k < ball.contacts.size(); ++k) {
                        if (dot(ball.contacts[k].normal, normal) > 0.95f) {
                            if (depth > ball.contacts[k].depth) {
                                ball.contacts[k] = { ci, normal, state.position - normal * radius, depth };
                            }
                            merged = true;
                            break;
                        }
                    }
                    if (!merged) ball.contacts.push_back({ ci, normal, state.position - normal * radius, depth });
                }
            }
            for (const auto& contact : ball.contacts) ball.lost += InflatedBallModel3D::capVolume(profile, contact.depth);
        }

        pairs.clear();
        for (std::size_t a = 0; a < balls.size(); ++a) {
            for (std::size_t b = a + 1; b < balls.size(); ++b) {
                if (!balls[a].moving && !balls[b].moving) continue;
                const auto& stateA = balls[a].record->state;
                const auto& stateB = balls[b].record->state;
                const auto& profileA = balls[a].record->definition.profile;
                const auto& profileB = balls[b].record->definition.profile;
                const float reach = profileA.radiusMeters + profileB.radiusMeters;
                const Vec3 between = stateA.position - stateB.position;
                const float distanceSquared = between.lengthSquared();
                if (distanceSquared >= reach * reach || distanceSquared < 1e-12f) continue;
                const float distance = std::sqrt(distanceSquared);
                const float overlap = std::min(reach - distance,
                    profileA.maximumCompressionShare * profileA.radiusMeters
                        + profileB.maximumCompressionShare * profileB.radiusMeters);
                pairs.push_back({ a, b, between * (1.0f / distance), overlap });
                balls[a].lost += InflatedBallModel3D::capVolume(profileA, 0.5f * overlap);
                balls[b].lost += InflatedBallModel3D::capVolume(profileB, 0.5f * overlap);
            }
        }
        for (BallWork& ball : balls) ball.volume = std::max(0.25f * ball.restVolume, ball.restVolume - ball.lost);

        for (BallWork& ball : balls) {
            auto& state = ball.record->state;
            const auto& profile = ball.record->definition.profile;
            ball.force = {};
            ball.torque = {};
            ball.touching = !ball.contacts.empty();
            if (!ball.moving) continue;
            ball.force = gravity * profile.massKg
                + InflatedBallModel3D::aerodynamicForce(profile, state.linearVelocity - wind, state.angularVelocity);
            for (const auto& contact : ball.contacts) {
                BallCandidate& candidate = ball.candidates[contact.candidate];
                const Vec3 r = contact.point - state.position;
                const Vec3 centerNow = candidate.centerOfMassEnd - candidate.linearVelocity * remaining + candidate.displacement;
                const Vec3 otherVelocity = candidate.linearVelocity + candidate.deltaVelocity
                    + cross(candidate.angularVelocity, contact.point - centerNow);
                const Vec3 relative = state.linearVelocity + cross(state.angularVelocity, r) - otherVelocity;
                const float approach = -dot(relative, contact.normal);
                const float normalForce = InflatedBallModel3D::normalForce(profile, contact.depth, approach, ball.volume,
                    candidate.surface.normalDampingRatio);
                Vec3 contactForce = contact.normal * normalForce;
                const Vec3 tangential = relative - contact.normal * dot(relative, contact.normal);
                const float slip = tangential.length();
                const float mu = std::sqrt(std::max(0.0f, profile.friction * candidate.friction));
                if (slip > 1e-5f) contactForce -= tangential * (mu * normalForce / std::max(slip, 0.05f));
                ball.force += contactForce;
                ball.torque += cross(r, contactForce);
                // Resistencia ao rolar: momento contra o giro, com o braco da
                // bola mais o da superficie (que cresce com a velocidade na
                // grama). Com o atrito no contato, frear o giro freia tambem a
                // translacao (a bola rolando tem as duas energias).
                const float spin = state.angularVelocity.length();
                if (spin > 1e-3f) {
                    const Vec3 travel = state.linearVelocity - otherVelocity;
                    const float along = (travel - contact.normal * dot(travel, contact.normal)).length();
                    const float arm = profile.rollingResistanceMeters + candidate.surface.rollingResistanceMeters
                        + candidate.surface.rollingResistancePerSpeed * along;
                    ball.torque -= state.angularVelocity * (arm * normalForce / spin);
                }
                if (candidate.dynamic) {
                    candidate.deltaVelocity -= contactForce * (h * candidate.inverseMass);
                    candidate.impulse -= contactForce * h;
                    const float weight = normalForce * h;
                    candidate.impulsePointSum += contact.point * weight;
                    candidate.impulseWeight += weight;
                }
                if (!candidate.touched) {
                    candidate.touched = true;
                    candidate.touchApproach = std::max(0.0f, approach);
                    candidate.touchPoint = contact.point;
                    candidate.touchNormal = contact.normal;
                }
                state.compressionMeters = std::max(state.compressionMeters, contact.depth);
                if (lastStep && ball.published < InflatedBallState3D::MaximumContacts) {
                    auto& out = state.contacts[ball.published++];
                    out.pointWorld = contact.point;
                    out.normalWorld = contact.normal;
                    out.compressionMeters = contact.depth;
                    out.normalForceNewtons = normalForce;
                    out.otherDynamic = candidate.dynamic;
                    out.otherRagdoll = candidate.ragdoll;
                    out.otherEntityId = candidate.entityId;
                    out.otherRagdollLink = candidate.link;
                }
            }
        }

        // Repulsao macia (bola e corpo se empurram, sem bater nem atritar).
        for (BallWork& ball : balls) {
            if (!ball.moving) continue;
            const auto& state = ball.record->state;
            for (const auto& soft : ball.softContacts) {
                BallCandidate& candidate = ball.candidates[soft.candidate];
                const Vec3 centerNow = candidate.centerOfMassEnd - candidate.linearVelocity * remaining + candidate.displacement;
                const Vec3 otherVelocity = candidate.linearVelocity + candidate.deltaVelocity
                    + cross(candidate.angularVelocity, soft.point - centerNow);
                const float approach = -dot(state.linearVelocity - otherVelocity, soft.normal);
                const float magnitude = std::clamp(SoftStiffness * soft.overlap + SoftDamping * approach,
                    0.0f, SoftMaximumForce);
                const Vec3 force = soft.normal * magnitude;
                ball.force += force;
                if (candidate.dynamic) {
                    candidate.deltaVelocity -= force * (h * candidate.inverseMass);
                    candidate.impulse -= force * h;
                    const float weight = magnitude * h;
                    candidate.impulsePointSum += soft.point * weight;
                    candidate.impulseWeight += weight;
                }
            }
        }

        for (const BallPair& pair : pairs) {
            BallWork& a = balls[pair.a];
            BallWork& b = balls[pair.b];
            auto& stateA = a.record->state;
            auto& stateB = b.record->state;
            const auto& profileA = a.record->definition.profile;
            const auto& profileB = b.record->definition.profile;
            const float half = 0.5f * pair.overlap;
            const Vec3 n = pair.normal;
            const Vec3 armA = n * -(profileA.radiusMeters - half);
            const Vec3 armB = n * (profileB.radiusMeters - half);
            const Vec3 relative = stateA.linearVelocity + cross(stateA.angularVelocity, armA)
                - stateB.linearVelocity - cross(stateB.angularVelocity, armB);
            const float approach = -dot(relative, n);
            // Cada casca ve o plano do meio se aproximar a metade da
            // velocidade relativa.
            const float normalForce = 0.5f
                * (InflatedBallModel3D::normalForce(profileA, half, 0.5f * approach, a.volume, 0.0f)
                    + InflatedBallModel3D::normalForce(profileB, half, 0.5f * approach, b.volume, 0.0f));
            Vec3 contactForce = n * normalForce;
            const Vec3 tangential = relative - n * dot(relative, n);
            const float slip = tangential.length();
            const float mu = std::sqrt(std::max(0.0f, profileA.friction * profileB.friction));
            if (slip > 1e-5f) contactForce -= tangential * (mu * normalForce / std::max(slip, 0.05f));
            a.force += contactForce;
            a.torque += cross(armA, contactForce);
            b.force -= contactForce;
            b.torque -= cross(armB, contactForce);
            a.touching = true;
            b.touching = true;
            stateA.compressionMeters = std::max(stateA.compressionMeters, half);
            stateB.compressionMeters = std::max(stateB.compressionMeters, half);
            const Vec3 point = stateA.position + armA;
            const auto touch = [&](BallWork& ball, std::uint32_t other, Vec3 normal) {
                for (const BallTouch& existing : ball.touchedBalls) if (existing.slot == other) return;
                ball.touchedBalls.push_back({ other, std::max(0.0f, approach), point, normal });
            };
            touch(a, b.slot, n);
            touch(b, a.slot, -n);
            if (lastStep) {
                const auto publish = [&](BallWork& ball, Vec3 contactPoint, Vec3 normal) {
                    if (ball.published >= InflatedBallState3D::MaximumContacts) return;
                    auto& out = ball.record->state.contacts[ball.published++];
                    out.pointWorld = contactPoint;
                    out.normalWorld = normal;
                    out.compressionMeters = half;
                    out.normalForceNewtons = normalForce;
                    out.otherDynamic = true;
                };
                publish(a, stateA.position + armA, n);
                publish(b, stateB.position + armB, -n);
            }
        }

        for (BallWork& ball : balls) {
            auto& state = ball.record->state;
            const auto& profile = ball.record->definition.profile;
            if (lastStep) state.contactCount = ball.published;
            if (!ball.moving) continue;
            if (!ball.touching && profile.spinDecay > 0.0f)
                state.angularVelocity *= std::max(0.0f, 1.0f - profile.spinDecay * h);
            for (auto& candidate : ball.candidates) candidate.displacement += candidate.deltaVelocity * h;
            state.linearVelocity += ball.force * (h / profile.massKg);
            state.angularVelocity += ball.torque * (h / ball.inertia);
            state.position += state.linearVelocity * h;
            // Orientacao: q' = q + h/2 (w,0) q.
            const Vec3& w = state.angularVelocity;
            const Quaternion& q = state.orientation;
            const Quaternion spin {
                0.5f * h * (w.x * q.w + w.y * q.z - w.z * q.y),
                0.5f * h * (w.y * q.w + w.z * q.x - w.x * q.z),
                0.5f * h * (w.z * q.w + w.x * q.y - w.y * q.x),
                -0.5f * h * (w.x * q.x + w.y * q.y + w.z * q.z) };
            state.orientation = Quaternion { q.x + spin.x, q.y + spin.y, q.z + spin.z, q.w + spin.w }.normalized();
        }
        substepNets(h, step, substeps);
    }
    finishNets(deltaTime);

    for (BallWork& ball : balls) {
        auto& record = *ball.record;
        auto& state = record.state;
        const auto& profile = record.definition.profile;
        state.volumeRatio = ball.volume / ball.restVolume;
        state.gaugePressurePascals = InflatedBallModel3D::gaugePressure(profile, ball.volume);
        state.peakCompressionMeters = std::max(state.peakCompressionMeters, state.compressionMeters);
        // Reacao nos corpos dinamicos e impactos (som) dos contatos novos.
        std::vector<JPH::BodyID> touchedNow;
        for (auto& candidate : ball.candidates) {
            if (!candidate.touched) continue;
            touchedNow.push_back(candidate.id);
            if (candidate.dynamic && candidate.impulseWeight > 0.0f && candidate.impulse.lengthSquared() > 0.0f) {
                const Vec3 point = candidate.impulsePointSum * (1.0f / candidate.impulseWeight);
                bodies.AddImpulse(candidate.id, toJolt(candidate.impulse), toJolt(point));
            }
            const bool fresh = std::find(record.previousContactBodies.begin(), record.previousContactBodies.end(),
                candidate.id) == record.previousContactBodies.end();
            if (fresh && candidate.touchApproach > 0.25f) {
                ContactImpactEvent3D event;
                event.bodyIdA = record.definition.entityId;
                event.materialA = record.definition.materialId;
                event.materialB = candidate.material;
                event.position = candidate.touchPoint;
                event.normal = candidate.touchNormal;
                const float massB = candidate.dynamic && candidate.inverseMass > 0.0f ? 1.0f / candidate.inverseMass : 0.0f;
                const float effective = massB > 0.0f ? profile.massKg * massB / (profile.massKg + massB) : profile.massKg;
                event.approachSpeedMetersPerSecond = candidate.touchApproach;
                event.normalImpulseNewtonSeconds = effective * candidate.touchApproach * 1.85f;
                event.effectiveMassKg = effective;
                event.transferredEnergyJoules = 0.5f * effective * candidate.touchApproach * candidate.touchApproach;
                event.massA = profile.massKg;
                event.massB = massB;
                event.staticA = false;
                event.staticB = !candidate.dynamic;
                event.characteristicSizeA = 2.0f * profile.radiusMeters;
                event.structureA = AcousticBodyStructure3D::Inflated;
                contactImpacts.push_back(std::move(event));
            }
        }
        record.previousContactBodies = std::move(touchedNow);
    }
    // Bola com bola: um evento por par (emitido pela de menor slot).
    for (BallWork& ball : balls) {
        auto& record = *ball.record;
        std::vector<std::uint32_t> touchedNow;
        for (const BallTouch& touch : ball.touchedBalls) {
            touchedNow.push_back(touch.slot);
            const bool fresh = std::find(record.previousContactBalls.begin(), record.previousContactBalls.end(),
                touch.slot) == record.previousContactBalls.end();
            if (!fresh || touch.approach <= 0.25f || touch.slot < ball.slot) continue;
            const auto& other = *inflatedBallSlots[touch.slot].record;
            const auto& profileA = record.definition.profile;
            const auto& profileB = other.definition.profile;
            ContactImpactEvent3D event;
            event.bodyIdA = record.definition.entityId;
            event.bodyIdB = other.definition.entityId;
            event.materialA = record.definition.materialId;
            event.materialB = other.definition.materialId;
            event.position = touch.point;
            event.normal = touch.normal;
            const float effective = profileA.massKg * profileB.massKg / (profileA.massKg + profileB.massKg);
            event.approachSpeedMetersPerSecond = touch.approach;
            event.normalImpulseNewtonSeconds = effective * touch.approach * 1.85f;
            event.effectiveMassKg = effective;
            event.transferredEnergyJoules = 0.5f * effective * touch.approach * touch.approach;
            event.massA = profileA.massKg;
            event.massB = profileB.massKg;
            event.staticA = false;
            event.staticB = false;
            event.characteristicSizeA = 2.0f * profileA.radiusMeters;
            event.characteristicSizeB = 2.0f * profileB.radiusMeters;
            event.structureA = AcousticBodyStructure3D::Inflated;
            event.structureB = AcousticBodyStructure3D::Inflated;
            contactImpacts.push_back(std::move(event));
        }
        record.previousContactBalls = std::move(touchedNow);
    }
}

} // namespace MatterEngine
