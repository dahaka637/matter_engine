#include "Engine/Physics/Jolt/JoltInternals3D.hpp"

#include <Jolt/Physics/Body/BodyLock.h>
#include <Jolt/Physics/Collision/BroadPhase/BroadPhaseQuery.h>
#include <Jolt/Physics/Collision/CollisionCollectorImpl.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/Shape/CapsuleShape.h>
#include <Jolt/Physics/Collision/Shape/CompoundShape.h>
#include <Jolt/Physics/Collision/Shape/OffsetCenterOfMassShape.h>
#include <Jolt/Physics/Collision/Shape/RotatedTranslatedShape.h>
#include <Jolt/Physics/Collision/Shape/SphereShape.h>
#include <Jolt/Physics/Collision/Shape/TaperedCapsuleShape.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>

namespace MatterEngine {

// A rede de fios na cena (ver Net/NetSolver3D.hpp e PhysicsScene3D.hpp). Por
// tick: as bolas e os corpos da cena perto de cada rede viram proxies (as
// formas do Jolt em esfera, capsula e caixa; o resto pela caixa que o
// contem); a rede anda nos subpassos da bola (acoplada a ela, ou a cada
// alguns subpassos se so ela se mexe); os corpos dinamicos levam o impulso de
// volta antes do passo seguinte, como na bola.

namespace {

constexpr std::uint32_t NoBall = std::numeric_limits<std::uint32_t>::max();
// Subpassos da rede sozinha (acordada, sem bola perto) por tick de 120 Hz.
constexpr std::uint32_t FreeSubsteps = 4;
// Com bola perto: no minimo os da bola (o chute de 30 m/s anda 1,6 cm por
// subpasso a 16).
constexpr std::uint32_t CoupledSubsteps = 16;
// A folga em volta da rede para os corpos entrarem na lista (m).
constexpr float NearbyMargin = 0.3f;
class NetObjectLayerFilter final : public JPH::ObjectLayerFilter {
public:
    bool ShouldCollide(JPH::ObjectLayer layer) const override {
        return layer != JoltObjectLayers::Character;
    }
};

Quaternion fromJoltRotation(JPH::QuatArg q) {
    return Quaternion { q.GetX(), q.GetY(), q.GetZ(), q.GetW() }.normalized();
}

// As formas de um corpo como proxies, no referencial do centro de massa dele.
void appendProxies(const JPH::Shape* shape, JPH::Mat44Arg local, std::uint32_t body, float friction,
    std::vector<NetProxy3D>& out) {
    using JPH::EShapeSubType;
    NetProxy3D proxy;
    proxy.body = body;
    proxy.friction = friction;
    proxy.localPosition = fromJolt(local.GetTranslation());
    proxy.localOrientation = fromJoltRotation(local.GetQuaternion());
    switch (shape->GetSubType()) {
    case EShapeSubType::Sphere:
        proxy.shape = NetProxyShape3D::Sphere;
        proxy.radius = static_cast<const JPH::SphereShape*>(shape)->GetRadius();
        out.push_back(proxy);
        return;
    case EShapeSubType::Capsule: {
        // A capsula do Jolt e ao longo do y; a do proxy, do z.
        const auto* capsule = static_cast<const JPH::CapsuleShape*>(shape);
        proxy.shape = NetProxyShape3D::Capsule;
        proxy.radius = capsule->GetRadius();
        proxy.halfHeight = capsule->GetHalfHeightOfCylinder();
        proxy.localOrientation = (proxy.localOrientation
            * Quaternion::fromAxisAngle({ 1.0f, 0.0f, 0.0f }, -0.5f * 3.14159265f)).normalized();
        out.push_back(proxy);
        return;
    }
    case EShapeSubType::TaperedCapsule: {
        // O braco e a perna (afunilados), como capsula com o raio da ponta
        // grossa: as esferas nas pontas pela caixa (em volta do centro de
        // massa), ao longo do y como a capsula.
        const auto* tapered = static_cast<const JPH::TaperedCapsuleShape*>(shape);
        const JPH::AABox bounds = shape->GetLocalBounds();
        const float top = tapered->GetTopRadius(), bottom = tapered->GetBottomRadius();
        const float middle = 0.5f * ((bounds.mMax.GetY() - top) + (bounds.mMin.GetY() + bottom));
        proxy.shape = NetProxyShape3D::Capsule;
        proxy.radius = std::max(top, bottom);
        proxy.halfHeight = tapered->GetHalfHeight();
        proxy.localPosition = fromJolt(local * JPH::Vec3(0.0f, middle, 0.0f));
        proxy.localOrientation = (proxy.localOrientation
            * Quaternion::fromAxisAngle({ 1.0f, 0.0f, 0.0f }, -0.5f * 3.14159265f)).normalized();
        out.push_back(proxy);
        return;
    }
    case EShapeSubType::Box:
        proxy.shape = NetProxyShape3D::Box;
        proxy.halfExtents = fromJolt(static_cast<const JPH::BoxShape*>(shape)->GetHalfExtent());
        out.push_back(proxy);
        return;
    case EShapeSubType::RotatedTranslated: {
        // O centro de massa da de dentro fica no da de fora (so gira).
        const auto* rotated = static_cast<const JPH::RotatedTranslatedShape*>(shape);
        appendProxies(rotated->GetInnerShape(), local * JPH::Mat44::sRotation(rotated->GetRotation()), body, friction, out);
        return;
    }
    case EShapeSubType::OffsetCenterOfMass: {
        const auto* offset = static_cast<const JPH::OffsetCenterOfMassShape*>(shape);
        appendProxies(offset->GetInnerShape(), local * JPH::Mat44::sTranslation(-offset->GetOffset()), body, friction, out);
        return;
    }
    case EShapeSubType::StaticCompound:
    case EShapeSubType::MutableCompound: {
        const auto* compound = static_cast<const JPH::CompoundShape*>(shape);
        for (const JPH::CompoundShape::SubShape& sub : compound->GetSubShapes())
            appendProxies(sub.mShape, local * sub.GetLocalTransformNoScale(JPH::Vec3::sOne()), body, friction, out);
        return;
    }
    default: {
        // O resto (casca convexa: a chuteira; cilindro...): a capsula ao longo
        // do lado maior da caixa em volta, com o raio do maior dos outros dois
        // (a capsula toca as celulas da rede; a caixa, so os nos virtuais, e a
        // chuteira de 10 cm passava entre eles).
        const JPH::AABox bounds = shape->GetLocalBounds();
        if (!bounds.IsValid()) return;
        const Vec3 half = fromJolt(bounds.GetExtent());
        const int longest = half.x >= half.y && half.x >= half.z ? 0 : half.y >= half.z ? 1 : 2;
        const float along = longest == 0 ? half.x : longest == 1 ? half.y : half.z;
        const float across = longest == 0 ? std::max(half.y, half.z) : longest == 1 ? std::max(half.x, half.z) : std::max(half.x, half.y);
        proxy.shape = NetProxyShape3D::Capsule;
        proxy.radius = across;
        proxy.halfHeight = std::max(0.0f, along - across);
        proxy.localPosition = fromJolt(local * bounds.GetCenter());
        // O z local da capsula ao longo do lado maior.
        const Quaternion toAxis = longest == 0 ? Quaternion::fromAxisAngle({ 0.0f, 1.0f, 0.0f }, 0.5f * 3.14159265f)
            : longest == 1 ? Quaternion::fromAxisAngle({ 1.0f, 0.0f, 0.0f }, -0.5f * 3.14159265f)
                           : Quaternion {};
        proxy.localOrientation = (proxy.localOrientation * toAxis).normalized();
        out.push_back(proxy);
        return;
    }
    }
}

} // namespace

PhysicsScene3D::Impl::NetRecord* PhysicsScene3D::Impl::net(NetHandle3D h) {
    return h.index < netSlots.size() && netSlots[h.index].generation == h.generation ? netSlots[h.index].record.get() : nullptr;
}

const PhysicsScene3D::Impl::NetRecord* PhysicsScene3D::Impl::net(NetHandle3D h) const {
    return h.index < netSlots.size() && netSlots[h.index].generation == h.generation ? netSlots[h.index].record.get() : nullptr;
}

NetHandle3D PhysicsScene3D::createNet(const NetDescription3D& description) {
    auto& impl = *m_impl;
    const auto index = impl.freeNetSlots.empty() ? static_cast<std::uint32_t>(impl.netSlots.size()) : impl.freeNetSlots.back();
    if (index == impl.netSlots.size()) impl.netSlots.emplace_back();
    else impl.freeNetSlots.pop_back();
    auto record = std::make_unique<Impl::NetRecord>();
    NetDescription3D settings = description;
    settings.settings.gravity = fromJolt(impl.system->GetGravity());
    record->solver = std::make_unique<NetSolver3D>(settings);
    record->handle = { index, impl.netSlots[index].generation };
    const NetHandle3D handle = record->handle;
    impl.netSlots[index].record = std::move(record);
    return handle;
}

void PhysicsScene3D::destroyNet(NetHandle3D handle) {
    auto& impl = *m_impl;
    if (!impl.net(handle)) return;
    auto& slot = impl.netSlots[handle.index];
    slot.record.reset();
    ++slot.generation;
    impl.freeNetSlots.push_back(handle.index);
}

bool PhysicsScene3D::contains(NetHandle3D handle) const {
    return m_impl->net(handle) != nullptr;
}

std::span<const Vec3> PhysicsScene3D::netPositions(NetHandle3D handle) const {
    const auto* record = m_impl->net(handle);
    return record ? record->solver->positions() : std::span<const Vec3> {};
}

NetStatus3D PhysicsScene3D::netStatus(NetHandle3D handle) const {
    const auto* record = m_impl->net(handle);
    return record ? record->status : NetStatus3D {};
}

void PhysicsScene3D::wakeNet(NetHandle3D handle) {
    if (auto* record = m_impl->net(handle)) record->solver->wake();
}

float PhysicsScene3D::settleNet(NetHandle3D handle, float maximumSeconds) {
    auto* record = m_impl->net(handle);
    if (!record) return 0.0f;
    // Quatro subpassos por tick (os mesmos da rede sozinha na cena): a metade
    // do custo de oito, com a mesma rede parada no fim.
    const float used = record->solver->settle(maximumSeconds, 1.0f / 120.0f, 4);
    record->status.sleeping = record->solver->sleeping();
    return used;
}

std::uint32_t PhysicsScene3D::Impl::prepareNets(float deltaTime, std::uint32_t ballSubsteps) {
    std::uint32_t needed = 0;
    const NetObjectLayerFilter layerFilter;
    for (auto& slot : netSlots) {
        if (!slot.record) continue;
        NetRecord& record = *slot.record;
        const auto begin = std::chrono::steady_clock::now();
        record.bodies.clear();
        record.proxies.clear();
        record.bodyBall.clear();
        record.bodyJolt.clear();
        const Vec3 low = record.solver->boundsMinimum() - Vec3 { NearbyMargin, NearbyMargin, NearbyMargin };
        const Vec3 high = record.solver->boundsMaximum() + Vec3 { NearbyMargin, NearbyMargin, NearbyMargin };
        const auto overlaps = [&](Vec3 a, Vec3 b, float radius) {
            const Vec3 lo { std::min(a.x, b.x) - radius, std::min(a.y, b.y) - radius, std::min(a.z, b.z) - radius };
            const Vec3 hi { std::max(a.x, b.x) + radius, std::max(a.y, b.y) + radius, std::max(a.z, b.z) + radius };
            return lo.x <= high.x && hi.x >= low.x && lo.y <= high.y && hi.y >= low.y && lo.z <= high.z && hi.z >= low.z;
        };
        // As bolas que chegam a ela neste tick (a bola parada nao cede).
        for (std::uint32_t b = 0; b < inflatedBallSlots.size(); ++b) {
            const auto& ballSlot = inflatedBallSlots[b];
            if (!ballSlot.record) continue;
            const InflatedBallState3D& state = ballSlot.record->state;
            const InflatedBallProfile3D& profile = ballSlot.record->definition.profile;
            if (!overlaps(state.position, state.position + state.linearVelocity * deltaTime, profile.radiusMeters)) continue;
            NetBody3D body;
            body.position = state.position;
            body.orientation = state.orientation;
            body.velocity = state.linearVelocity;
            body.angularVelocity = state.angularVelocity;
            if (!state.frozen) {
                body.inverseMass = 1.0f / profile.massKg;
                body.inverseInertia = 1.0f / InflatedBallModel3D::shellInertia(profile);
            }
            NetProxy3D proxy;
            proxy.shape = NetProxyShape3D::Sphere;
            proxy.body = static_cast<std::uint32_t>(record.bodies.size());
            proxy.radius = profile.radiusMeters;
            proxy.friction = profile.friction;
            record.proxies.push_back(proxy);
            record.bodies.push_back(body);
            record.bodyBall.push_back(b);
            record.bodyJolt.push_back(JPH::BodyID());
        }
        const bool anyBall = !record.bodies.empty();
        record.fastestBall = 0.0f;
        record.fastestBody = 0.0f;
        for (const NetBody3D& ball : record.bodies) record.fastestBall = std::max(record.fastestBall, ball.velocity.length());
        // Os corpos da cena (dinamicos, cinematicos, os links dos bonecos) que
        // a caixa da rede pega: na pose do comeco do tick (a do fim recuada).
        JPH::AllHitCollisionCollector<JPH::CollideShapeBodyCollector> hits;
        system->GetBroadPhaseQuery().CollideAABox(JPH::AABox(toJolt(low), toJolt(high)), hits, {}, layerFilter);
        for (const JPH::BodyID& id : hits.mHits) {
            JPH::BodyLockRead lock(system->GetBodyLockInterface(), id);
            if (!lock.Succeeded()) continue;
            const JPH::Body& joltBody = lock.GetBody();
            if (joltBody.IsStatic() || joltBody.IsSensor() || joltBody.IsSoftBody()) continue;
            // O corpo guiado do boneco (o modo padrao) nao cede a rede: ela sai
            // do caminho dele e relata o que ele sentiria solto (ver
            // NetBody3D::guided), que vai como empurrao de fora ao boneco.
            bool guided = false;
            const auto identity = JoltDetail::unpackBodyIdentity(joltBody.GetUserData());
            if (identity.kind == JoltDetail::BodyKind::RagdollLink && identity.slotIndex < ragdollSlots.size()) {
                const RagdollRecord* owner = ragdollSlots[identity.slotIndex].record.get();
                guided = owner != nullptr && owner->guided;
            }
            NetBody3D body;
            body.velocity = fromJolt(joltBody.GetLinearVelocity());
            body.angularVelocity = fromJolt(joltBody.GetAngularVelocity());
            body.position = fromJolt(joltBody.GetCenterOfMassPosition()) - body.velocity * deltaTime;
            body.orientation = fromJoltRotation(joltBody.GetRotation());
            body.advance = true;
            if (joltBody.IsDynamic()) body.inverseMass = joltBody.GetMotionProperties()->GetInverseMass();
            body.guided = guided;
            const auto first = record.proxies.size();
            appendProxies(joltBody.GetShape(), JPH::Mat44::sIdentity(), static_cast<std::uint32_t>(record.bodies.size()),
                joltBody.GetFriction(), record.proxies);
            if (record.proxies.size() == first) continue;
            record.fastestBody = std::max(record.fastestBody, body.velocity.length());
            record.bodies.push_back(body);
            record.bodyBall.push_back(NoBall);
            record.bodyJolt.push_back(id);
        }
        record.solver->beginTick(deltaTime, record.bodies, record.proxies);
        record.active = record.solver->needsSubsteps();
        record.coupled = record.active && anyBall;
        record.status.nearbyBodies = static_cast<std::uint32_t>(record.bodies.size());
        // Um corpo rapido perto (a caixa atirada, a perna no chute): no maximo
        // ~3 cm por subpasso da rede.
        const float travel = record.fastestBody * deltaTime;
        record.bodySubsteps = travel > 0.12f ? 16u : travel > 0.06f ? 8u : FreeSubsteps;
        if (record.active) needed = std::max(needed, record.coupled ? std::max(ballSubsteps, CoupledSubsteps) : record.bodySubsteps);
        record.status.stepMilliseconds = std::chrono::duration<float, std::milli>(std::chrono::steady_clock::now() - begin).count();
    }
    // Os subpassos de verdade do tick (os das bolas tambem): com uma bola rolando longe dela (16
    // subpassos), a rede sozinha acordada andava nos 16 em vez de FreeSubsteps.
    const std::uint32_t total = std::max(needed, ballSubsteps);
    for (auto& slot : netSlots) {
        if (!slot.record || !slot.record->active) continue;
        // A rede anda a cada `stride` subpassos do tick: acoplada, todos no
        // impacto (bola acima de 8 m/s) e menos com a bola devagar (rolando
        // dentro do gol, apoiada na rede); sozinha, FreeSubsteps por tick.
        NetRecord& record = *slot.record;
        // (Na malha de 12 cm, a 4 com a bola a 3-4 m/s rolando contra o pe do
        // fundo, os nos espremidos entre a bola, o chao e a barra esticavam ate
        // 33%; na da fisica de 2 malhas, a 4 ate 3 m/s o fio nao passa de 8,4%
        // na grade de chutes. O impacto a 8 devolvia bolas para fora do gol, e a
        // rede a 2 na cauda balancava o dobro.)
        const std::uint32_t ballStride = record.fastestBall > 8.0f ? 1u : record.fastestBall > 3.0f ? 2u : 4u;
        const std::uint32_t coupledStride = std::min(ballStride, std::max(1u, CoupledSubsteps / record.bodySubsteps));
        record.stride = record.coupled ? std::min(coupledStride, std::max(1u, total / FreeSubsteps))
                                       : std::max(1u, total / record.bodySubsteps);
        // (Os passos da rede cobrem o tick inteiro: o stride divide os subpassos dele. Uma bola com 18
        // e a rede pedindo 4 dariam stride 4, e a rede andava 16/18 do tick.)
        while (total % record.stride != 0) --record.stride;
    }
    return needed;
}

void PhysicsScene3D::Impl::substepNets(float h, std::uint32_t step, std::uint32_t substeps) {
    static_cast<void>(substeps);
    for (auto& slot : netSlots) {
        if (!slot.record || !slot.record->active) continue;
        NetRecord& record = *slot.record;
        if ((step + 1) % record.stride != 0) continue;
        const auto begin = std::chrono::steady_clock::now();
        // As bolas entram no estado de agora e saem corrigidas.
        for (std::size_t b = 0; b < record.bodies.size(); ++b) {
            if (record.bodyBall[b] == NoBall) continue;
            const InflatedBallState3D& state = inflatedBallSlots[record.bodyBall[b]].record->state;
            record.bodies[b].position = state.position;
            record.bodies[b].orientation = state.orientation;
            record.bodies[b].velocity = state.linearVelocity;
            record.bodies[b].angularVelocity = state.angularVelocity;
        }
        record.solver->substep(h * static_cast<float>(record.stride), record.bodies, record.proxies);
        for (std::size_t b = 0; b < record.bodies.size(); ++b) {
            if (record.bodyBall[b] == NoBall || record.bodies[b].inverseMass <= 0.0f) continue;
            InflatedBallState3D& state = inflatedBallSlots[record.bodyBall[b]].record->state;
            state.position = record.bodies[b].position;
            state.linearVelocity = record.bodies[b].velocity;
            state.angularVelocity = record.bodies[b].angularVelocity;
        }
        record.status.stepMilliseconds += std::chrono::duration<float, std::milli>(std::chrono::steady_clock::now() - begin).count();
    }
}

void PhysicsScene3D::Impl::finishNets(float deltaTime) {
    auto& bodyInterface = system->GetBodyInterface();
    for (auto& slot : netSlots) {
        if (!slot.record) continue;
        NetRecord& record = *slot.record;
        const auto begin = std::chrono::steady_clock::now();
        if (record.active) {
            record.solver->endTick(deltaTime, record.bodies);
            // A reacao da rede (antes do passo seguinte). No boneco guiado, como
            // empurrao de fora no link (a forca do impulso no tick, no passo
            // seguinte): o atleta ve o empurrao como qualquer outro - de leve,
            // nada; forte (a rede esticada), vira fisico e a rede o segura. No
            // boneco ja fisico, o impulso comum: apoiado na rede, o peso dele
            // (~6,5 N s por tick em 80 kg) contaria como pancada sem parar e o
            // deixaria preso no impacto. No objeto solto, o impulso e o empurrao:
            // sem o empurrao, o Jolt (que andou o tick sem a rede) deixava o
            // corpo rapido fundo nela, e no tick seguinte ele ja comecava do
            // outro lado.
            for (std::size_t b = 0; b < record.bodies.size(); ++b) {
                const NetBody3D& body = record.bodies[b];
                if (record.bodyBall[b] != NoBall || body.inverseMass <= 0.0f || body.impulseWeight <= 0.0f) continue;
                if (body.impulse.lengthSquared() <= 0.0f) continue;
                const Vec3 point = body.impulsePointSum * (1.0f / body.impulseWeight);
                JPH::BodyLockRead lock(system->GetBodyLockInterface(), record.bodyJolt[b]);
                if (!lock.Succeeded()) continue;
                const JPH::Body& joltBody = lock.GetBody();
                const auto identity = JoltDetail::unpackBodyIdentity(joltBody.GetUserData());
                const bool ragdollLink = identity.kind == JoltDetail::BodyKind::RagdollLink && identity.slotIndex < ragdollSlots.size();
                if (ragdollLink && body.guided) {
                    RagdollRecord* owner = ragdollSlots[identity.slotIndex].record.get();
                    if (owner == nullptr || owner->frozen || identity.linkIndex >= owner->links.size()) continue;
                    const Vec3 force = body.impulse * (1.0f / deltaTime);
                    const Vec3 torque = cross(point - fromJolt(joltBody.GetCenterOfMassPosition()), force);
                    owner->externalForceMagnitude += force.length() + torque.length() / std::max(0.2f, owner->profile.standingRootHeightMeters);
                    pendingBodyWrenches.push_back({ owner->links[identity.linkIndex], force, torque });
                    continue;
                }
                lock.ReleaseLock();
                bodyInterface.AddImpulse(record.bodyJolt[b], toJolt(body.impulse), toJolt(point));
                // (O link do boneco fisico nao e teleportado: as juntas brigariam.)
                if (!ragdollLink && body.shift.lengthSquared() > 1e-10f)
                    bodyInterface.SetPosition(record.bodyJolt[b], bodyInterface.GetPosition(record.bodyJolt[b]) + toJolt(body.shift),
                        JPH::EActivation::Activate);
            }
        }
        const NetTickStats3D& stats = record.solver->stats();
        record.status.sleeping = record.solver->sleeping();
        record.status.substeps = record.active ? stats.substeps : 0;
        record.status.contacts = stats.contacts;
        record.status.maximumSpeed = stats.maximumSpeed;
        record.status.maximumStrain = stats.maximumStrain;
        record.status.stepMilliseconds += std::chrono::duration<float, std::milli>(std::chrono::steady_clock::now() - begin).count();
    }
}

} // namespace MatterEngine
