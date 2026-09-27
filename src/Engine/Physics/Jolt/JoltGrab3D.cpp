#include "Engine/Physics/Jolt/JoltInternals3D.hpp"
#include <Jolt/Physics/Body/BodyLock.h>
#include <Jolt/Physics/Collision/Shape/EmptyShape.h>
#include <algorithm>
#include <cmath>
namespace MatterEngine {
namespace {
bool createGrab(PhysicsScene3D::Impl& impl, JPH::BodyID id, Vec3 localPoint, const PhysicsGrabTarget3D& target) {
    auto& bi=impl.system->GetBodyInterface();
    JPH::RefConst<JPH::Shape> shape=new JPH::EmptyShape;
    JPH::BodyCreationSettings anchor(shape,toJolt(target.position),JPH::Quat::sIdentity(),JPH::EMotionType::Kinematic,JoltObjectLayers::Moving);
    auto* body=bi.CreateBody(anchor);
    if (!body) return false;
    impl.grabAnchorBody=body->GetID();
    bi.AddBody(impl.grabAnchorBody,JPH::EActivation::DontActivate);
    JPH::SixDOFConstraintSettings settings;
    settings.mPosition1=toJolt(target.position);
    settings.mPosition2=bi.GetPosition(id)+bi.GetRotation(id)*toJolt(localPoint);
    // Body-2 joint axes follow the held body's initial orientation.
    settings.mAxisX2=bi.GetRotation(id)*JPH::Vec3::sAxisX();
    settings.mAxisY2=bi.GetRotation(id)*JPH::Vec3::sAxisY();
    impl.grabConstraint=static_cast<JPH::SixDOFConstraint*>(bi.CreateConstraint(&settings,impl.grabAnchorBody,id));
    impl.system->AddConstraint(impl.grabConstraint);
    bi.ActivateBody(id);
    return true;
}
}
bool PhysicsScene3D::beginGrab(PhysicsBodyHandle3D h, Vec3 point,const PhysicsGrabTarget3D& target,const PhysicsHandleSettings3D& settings) {
    endGrab();
    auto* r=m_impl->body(h);
    if (!r || r->motionType != JPH::EMotionType::Dynamic) return false;
    setBodyFrozen(h,false);
    if (!createGrab(*m_impl,r->bodyId,point,target)) return false;
    m_impl->grabBody=h;
    updateGrabTarget(target,settings);
    return true;
}
bool PhysicsScene3D::beginRagdollGrab(RagdollHandle3D h,std::uint32_t link,Vec3 point,const PhysicsGrabTarget3D& target,const PhysicsHandleSettings3D& settings) {
    endGrab();
    auto* r=m_impl->ragdoll(h);
    if (!r || link>=r->links.size()) return false;
    if (r->frozen) { r->ragdoll->AddToPhysicsSystem(JPH::EActivation::Activate); r->frozen=false; }
    if (!createGrab(*m_impl,r->links[link],point,target)) return false;
    m_impl->grabRagdoll=h; m_impl->grabRagdollLink=link;
    updateGrabTarget(target,settings);
    return true;
}
void PhysicsScene3D::updateGrabTarget(const PhysicsGrabTarget3D& target,const PhysicsHandleSettings3D& settings) {
    if (!m_impl->grabConstraint) return;
    auto& c=*m_impl->grabConstraint;
    auto& bi=m_impl->system->GetBodyInterface();
    const auto id=c.GetBody2()->GetID();
    float mass=1, inertia=1;
    {
        JPH::BodyLockRead lock(m_impl->system->GetBodyLockInterface(),id);
        if (!lock.Succeeded()) return;
        const auto* motion=lock.GetBody().GetMotionProperties();
        mass=1/motion->GetInverseMass();
        const auto inverse=motion->GetInverseInertiaDiagonal();
        inertia=(1/std::max(1e-6f,inverse.GetX())+1/std::max(1e-6f,inverse.GetY())+1/std::max(1e-6f,inverse.GetZ()))/3;
    }
    bi.SetPosition(m_impl->grabAnchorBody,toJolt(target.position),JPH::EActivation::Activate);
    c.SetTargetOrientationCS(toJolt(target.orientation));
    for (int a=0;a<6;++a) {
        const auto axis=static_cast<JPH::SixDOFConstraint::EAxis>(a);
        auto& motor=c.GetMotorSettings(axis);
        const bool angular=a>=3;
        const float k=std::max(0.0f,angular?settings.angularStiffness:settings.linearStiffness);
        const float ratio=std::max(0.0f,angular?settings.angularDampingRatio:settings.linearDampingRatio);
        motor.mSpringSettings={JPH::ESpringMode::StiffnessAndDamping,k,2*ratio*std::sqrt(k*(angular?inertia:mass))};
        float limit=std::max(0.0f,angular?settings.maximumTorque:settings.maximumForce);
        if (m_impl->grabRagdoll) limit=std::min(limit,angular?inertia*140:mass*180);
        if (angular) motor.SetTorqueLimit(limit); else motor.SetForceLimit(limit);
        c.SetMotorState(axis,angular&&!target.lockOrientation?JPH::EMotorState::Off:JPH::EMotorState::PositionAndVelocity);
    }
    bi.ActivateBody(id);
}
void PhysicsScene3D::Impl::releaseGrab() {
    if (grabConstraint) { system->RemoveConstraint(grabConstraint); grabConstraint=nullptr; }
    if (!grabAnchorBody.IsInvalid()) { auto& bi=system->GetBodyInterface(); bi.RemoveBody(grabAnchorBody); bi.DestroyBody(grabAnchorBody); grabAnchorBody=JPH::BodyID(); }
    grabBody={}; grabRagdoll={}; grabRagdollLink=std::numeric_limits<std::uint32_t>::max();
}
void PhysicsScene3D::endGrab() { m_impl->releaseGrab(); }
bool PhysicsScene3D::grabbing() const { return m_impl->grabConstraint != nullptr; }
PhysicsBodyHandle3D PhysicsScene3D::grabbedBody() const { return m_impl->grabBody; }
RagdollHandle3D PhysicsScene3D::grabbedRagdoll() const { return m_impl->grabRagdoll; }
std::uint32_t PhysicsScene3D::grabbedRagdollLink() const { return m_impl->grabRagdollLink; }
} // namespace MatterEngine
