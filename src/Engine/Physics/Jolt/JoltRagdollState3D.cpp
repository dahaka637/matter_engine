#include "Engine/Physics/Jolt/JoltInternals3D.hpp"
#include "Engine/Physics/RagdollInteractions3D.hpp"
#include <Jolt/Physics/Body/BodyLock.h>
#include <cmath>
#include <algorithm>
#include <utility>
namespace MatterEngine {
void PhysicsScene3D::Impl::publishRagdoll(RagdollRecord& r) {
    const auto& bi = system->GetBodyInterface();
    r.state.sleeping = true;
    for (std::size_t i = 0; i < r.links.size(); ++i) {
        const auto id = r.links[i];
        auto& state = r.state.links[i];
        state = {fromJolt(bi.GetPosition(id)), fromJolt(bi.GetRotation(id)), fromJolt(bi.GetLinearVelocity(id)), fromJolt(bi.GetAngularVelocity(id)), !bi.IsActive(id), r.frozen};
        r.state.sleeping &= state.sleeping;
        if (!i) continue;
        const auto& link = r.profile.links[i];
        const auto parent = static_cast<std::size_t>(link.parentIndex);
        auto& joint = r.state.joints[i];
        auto q = r.joints[i].constraint->GetRotationInConstraintSpace();
        if (q.GetW() < 0) q = -q;
        const auto v = q.GetXYZ();
        const float n = v.Length();
        const auto rotation = n > 1e-7f ? v * (2 * std::atan2(n, q.GetW()) / n) : JPH::Vec3::sZero();
        const Quaternion frame = state.orientation * link.modelOrientation.conjugate() * link.inboundJoint.frameModelOrientation;
        const auto velocity = toJolt(frame.conjugate().rotate(state.angularVelocity - r.state.links[parent].angularVelocity));
        // Solver lambdas are impulses. Convert to torque; these include limits and motors.
        // (Os lambdas sao do ultimo sub-passo de colisao; divididos pela duracao dele.)
        const float substep = stepDeltaTime / static_cast<float>(collisionSteps);
        const auto constraintTorque = r.joints[i].constraint->GetTotalLambdaRotation() / substep;
        const auto motorTorque = r.joints[i].constraint->GetTotalLambdaMotorRotation() / substep;
        const auto torque = constraintTorque + motorTorque;
        const auto& runtime = r.joints[i];
        for (int a = 0; a < 3; ++a) {
            const bool enabled = link.inboundJoint.axes[a].enabled;
            const auto axis = static_cast<std::size_t>(a);
            joint.positionRadians[a] = enabled ? rotation[a] : 0;
            joint.velocityRadiansPerSecond[a] = enabled ? velocity[a] : 0;
            joint.transmittedTorqueNewtonMeters[a] = torque[a];
            joint.motorTorqueNewtonMeters[a] = enabled ? motorTorque[a] : 0;
            joint.constraintTorqueNewtonMeters[a] = enabled ? constraintTorque[a] : 0;
            joint.feedforwardTorqueNewtonMeters[a] = enabled ? runtime.appliedFeedforward[axis] : 0;
            joint.motorTorqueLimitNewtonMeters[a] = enabled ? runtime.appliedTorqueLimit[axis] : 0;
            joint.targetPositionRadians[a] = enabled ? runtime.appliedTargets[axis] : 0;
            joint.targetVelocityRadiansPerSecond[a] = enabled ? runtime.appliedTargetVelocities[axis] : 0;
        }
    }
    r.state.contacts = r.stepContacts;
    publishRagdollInteractions3D(r.stepInteractions, r.state.interactions);
    r.state.active = r.active;
    r.state.frozen = r.frozen;
    r.state.rigidityPercent = r.rigidityPercent;
    // externalInterference e animationAuthority sao calculadas em
    // resolveRagdollGuides, que precisa delas antes de publicar.
}
RagdollState3D PhysicsScene3D::ragdollState(RagdollHandle3D h) const { const auto* r = m_impl->ragdoll(h); return r ? r->state : RagdollState3D{}; }
const RagdollState3D* PhysicsScene3D::ragdollStateView(RagdollHandle3D h) const {
    const auto* r = m_impl->ragdoll(h);
    return r ? &r->state : nullptr;
}
RagdollDynamics3D PhysicsScene3D::ragdollDynamics(RagdollHandle3D h) const {
    RagdollDynamics3D result;
    const auto* r = m_impl->ragdoll(h);
    if (!r) return result;
    result.valid = true;
    result.generalizedDofCount = r->indexing.generalizedDofCount;
    result.jointGeneralizedDof = r->indexing.jointGeneralizedDof;
    for (std::size_t i = 0; i < r->links.size(); ++i)
        result.centerOfMass += (r->state.links[i].position + r->state.links[i].orientation.rotate(r->profile.links[i].centerOfMassLocal)) * r->profile.links[i].massFraction;
    return result;
}
RagdollEffectiveMass3D PhysicsScene3D::ragdollEffectiveMass(RagdollHandle3D h) const {
    RagdollEffectiveMass3D result;
    const auto* r = m_impl->ragdoll(h);
    if (!r) return result;

    result.links.reserve(r->links.size());
    Vec3 weightedCenter;
    for (std::size_t i = 0; i < r->links.size(); ++i) {
        JPH::BodyLockRead lock(m_impl->system->GetBodyLockInterface(), r->links[i]);
        if (!lock.Succeeded()) return {};
        const JPH::Body& body = lock.GetBody();
        if (!body.IsDynamic()) return {};
        const JPH::MotionProperties* motion = body.GetMotionProperties();
        if (!motion) return {};

        const float inverseMass = motion->GetInverseMass();
        const JPH::Vec3 inverseInertia = motion->GetInverseInertiaDiagonal();
        if (!(inverseMass > 0.0f) || !(inverseInertia.GetX() > 0.0f)
            || !(inverseInertia.GetY() > 0.0f) || !(inverseInertia.GetZ() > 0.0f)) {
            return {};
        }

        RagdollLinkEffectiveMass3D link;
        link.id = r->profile.links[i].id;
        link.massKg = 1.0f / inverseMass;
        link.centerOfMassLocal = fromJolt(body.GetShape()->GetCenterOfMass());
        if (i >= r->unconditionedMassProperties.size()) return {};
        JPH::Mat44 unconditionedAxes;
        JPH::Vec3 unconditionedInertia;
        if (!r->unconditionedMassProperties[i].DecomposePrincipalMomentsOfInertia(
                unconditionedAxes, unconditionedInertia)) return {};
        link.unconditionedPrincipalInertiaKgSquareMeters = fromJolt(unconditionedInertia);
        link.unconditionedPrincipalAxesLocal = fromJolt(unconditionedAxes.GetQuaternion());
        link.principalInertiaKgSquareMeters = {
            1.0f / inverseInertia.GetX(),
            1.0f / inverseInertia.GetY(),
            1.0f / inverseInertia.GetZ()
        };
        link.principalAxesLocal = fromJolt(motion->GetInertiaRotation());
        std::array<float, 3> before {
            link.unconditionedPrincipalInertiaKgSquareMeters.x,
            link.unconditionedPrincipalInertiaKgSquareMeters.y,
            link.unconditionedPrincipalInertiaKgSquareMeters.z
        };
        std::array<float, 3> after {
            link.principalInertiaKgSquareMeters.x,
            link.principalInertiaKgSquareMeters.y,
            link.principalInertiaKgSquareMeters.z
        };
        std::sort(before.begin(), before.end());
        std::sort(after.begin(), after.end());
        link.maximumInertiaConditioningRatio = 1.0f;
        for (std::size_t axis = 0; axis < before.size(); ++axis) {
            if (!(before[axis] > 0.0f)) return {};
            link.maximumInertiaConditioningRatio = std::max(
                link.maximumInertiaConditioningRatio, after[axis] / before[axis]);
        }

        result.totalMassKg += link.massKg;
        weightedCenter += fromJolt(body.GetCenterOfMassPosition()) * link.massKg;
        result.links.push_back(std::move(link));
    }
    if (!(result.totalMassKg > 0.0f)) return {};
    result.centerOfMassWorld = weightedCenter / result.totalMassKg;
    result.valid = true;
    return result;
}

} // namespace MatterEngine
