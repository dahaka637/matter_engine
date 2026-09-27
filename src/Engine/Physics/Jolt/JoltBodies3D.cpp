#include "Engine/Physics/Jolt/JoltInternals3D.hpp"
#include "Engine/Physics/Jolt/JoltShapes3D.hpp"
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <algorithm>
#include <stdexcept>

namespace MatterEngine {
PhysicsScene3D::Impl::BodyRecord* PhysicsScene3D::Impl::body(PhysicsBodyHandle3D h) {
    return h.index < bodySlots.size() && bodySlots[h.index].generation == h.generation
        ? bodySlots[h.index].record.get() : nullptr;
}
const PhysicsScene3D::Impl::BodyRecord* PhysicsScene3D::Impl::body(PhysicsBodyHandle3D h) const {
    return h.index < bodySlots.size() && bodySlots[h.index].generation == h.generation
        ? bodySlots[h.index].record.get() : nullptr;
}
PhysicsScene3D::Impl::BodyRecord* PhysicsScene3D::Impl::bodyFromId(JPH::BodyID id) {
    const auto identity = JoltDetail::unpackBodyIdentity(system->GetBodyInterface().GetUserData(id));
    if (identity.kind != JoltDetail::BodyKind::Body || identity.slotIndex >= bodySlots.size()) return nullptr;
    auto* r = bodySlots[identity.slotIndex].record.get();
    return r && r->bodyId == id ? r : nullptr;
}
PhysicsBodyHandle3D PhysicsScene3D::createBody(const PhysicsBodyDefinition3D& def,
    std::span<const PhysicsShape3D> shapes) {
    const auto motion = def.motionType == PhysicsMotionType3D::Static ? JPH::EMotionType::Static
        : def.motionType == PhysicsMotionType3D::Kinematic ? JPH::EMotionType::Kinematic : JPH::EMotionType::Dynamic;
    auto shape = createJoltBodyShape(shapes, motion == JPH::EMotionType::Static);
    JPH::BodyCreationSettings settings(shape, toJolt(def.position), toJolt(def.orientation), motion,
        motion == JPH::EMotionType::Static ? JoltObjectLayers::NonMoving : JoltObjectLayers::Moving);
    settings.mAllowDynamicOrKinematic = motion != JPH::EMotionType::Static;
    settings.mLinearVelocity = toJolt(def.linearVelocity);
    settings.mAngularVelocity = toJolt(def.angularVelocity);
    settings.mLinearDamping = def.linearDamping;
    settings.mAngularDamping = def.angularDamping;
    settings.mAllowSleeping = def.allowSleeping;
    settings.mMotionQuality = def.collisionMode == PhysicsCollisionMode3D::Continuous && m_impl->settings.enableContinuousCollision
        ? JPH::EMotionQuality::LinearCast : JPH::EMotionQuality::Discrete;
    settings.mOverrideMassProperties = JPH::EOverrideMassProperties::CalculateInertia;
    settings.mMassPropertiesOverride.mMass = std::max(def.massKg, 0.001f);
    const auto& mat = m_impl->contactMaterial(def.materialId);
    settings.mFriction = mat.dynamicFriction;
    settings.mRestitution = mat.restitution;
    const auto index = m_impl->freeBodySlots.empty() ? static_cast<std::uint32_t>(m_impl->bodySlots.size()) : m_impl->freeBodySlots.back();
    settings.mUserData = JoltDetail::packBodyIdentity({JoltDetail::BodyKind::Body, index, 0});
    auto record = std::make_unique<Impl::BodyRecord>();
    record->definition = def;
    record->motionType = motion;
    auto& bi = m_impl->system->GetBodyInterface();
    auto* native = bi.CreateBody(settings);
    if (!native) throw std::runtime_error("Jolt: maximumBodies exceeded");
    record->bodyId = native->GetID();
    if (index == m_impl->bodySlots.size()) m_impl->bodySlots.emplace_back();
    else m_impl->freeBodySlots.pop_back();
    record->handle = { index, m_impl->bodySlots[index].generation };
    const auto handle = record->handle;
    m_impl->bodySlots[index].record = std::move(record);
    bi.AddBody(native->GetID(), def.startAwake && motion != JPH::EMotionType::Static ? JPH::EActivation::Activate : JPH::EActivation::DontActivate);
    return handle;
}
void PhysicsScene3D::destroyBody(PhysicsBodyHandle3D handle) {
    auto* record = m_impl->body(handle);
    if (!record) return;
    if (m_impl->grabBody == handle) endGrab();
    const auto id = record->bodyId;
    std::erase_if(m_impl->pendingBodyWrenches, [id](const auto& w) { return w.body == id; });
    auto& bi = m_impl->system->GetBodyInterface();
    bi.RemoveBody(id);
    bi.DestroyBody(id);
    auto& slot = m_impl->bodySlots[handle.index];
    slot.record.reset();
    ++slot.generation;
    m_impl->freeBodySlots.push_back(handle.index);
}
bool PhysicsScene3D::contains(PhysicsBodyHandle3D h) const { return m_impl->body(h) != nullptr; }
PhysicsBodyState3D PhysicsScene3D::bodyState(PhysicsBodyHandle3D h) const {
    const auto* r = m_impl->body(h);
    if (!r) return {};
    const auto& bi = m_impl->system->GetBodyInterface();
    return {fromJolt(bi.GetPosition(r->bodyId)), fromJolt(bi.GetRotation(r->bodyId)),
        fromJolt(bi.GetLinearVelocity(r->bodyId)), fromJolt(bi.GetAngularVelocity(r->bodyId)),
        !bi.IsActive(r->bodyId), r->frozen};
}
float PhysicsScene3D::bodyMass(PhysicsBodyHandle3D h) const {
    const auto* r = m_impl->body(h);
    return r && r->motionType == JPH::EMotionType::Dynamic ? r->definition.massKg : 0;
}
void PhysicsScene3D::setBodyFrozen(PhysicsBodyHandle3D h, bool frozen) {
    auto* r = m_impl->body(h);
    if (!r || r->motionType == JPH::EMotionType::Static || r->frozen == frozen) return;
    auto& bi = m_impl->system->GetBodyInterface();
    bi.SetLinearAndAngularVelocity(r->bodyId, JPH::Vec3::sZero(), JPH::Vec3::sZero());
    bi.SetMotionType(r->bodyId, frozen ? JPH::EMotionType::Kinematic : r->motionType, JPH::EActivation::Activate);
    r->frozen = frozen;
}
bool PhysicsScene3D::bodyFrozen(PhysicsBodyHandle3D h) const { const auto* r = m_impl->body(h); return r && r->frozen; }
void PhysicsScene3D::wakeBody(PhysicsBodyHandle3D h) { auto* r = m_impl->body(h); if (r && r->motionType != JPH::EMotionType::Static) m_impl->system->GetBodyInterface().ActivateBody(r->bodyId); }
void PhysicsScene3D::applyForceAtPoint(PhysicsBodyHandle3D h, Vec3 force, Vec3 point) {
    auto* r = m_impl->body(h);
    if (!r || r->frozen || r->motionType != JPH::EMotionType::Dynamic) return;
    const Vec3 center = fromJolt(m_impl->system->GetBodyInterface().GetCenterOfMassPosition(r->bodyId));
    m_impl->pendingBodyWrenches.push_back({r->bodyId, force, cross(point-center, force)});
}
void PhysicsScene3D::applyTorque(PhysicsBodyHandle3D h, Vec3 torque) {
    auto* r = m_impl->body(h);
    if (r && !r->frozen && r->motionType == JPH::EMotionType::Dynamic)
        m_impl->pendingBodyWrenches.push_back({r->bodyId, {}, torque});
}
} // namespace MatterEngine
