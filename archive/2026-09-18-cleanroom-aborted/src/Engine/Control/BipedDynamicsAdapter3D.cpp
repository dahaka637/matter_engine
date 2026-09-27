// Matter Engine clean-room physical biped controller.
// THEORY: [R04] Virtual Model Control; [R09/R11] task-space control; [R13] PhysX Jacobians.
// Core invariant: task wrenches become joint torques through J^T; floating-root columns are never actuated.

#include "Engine/Control/BipedDynamicsAdapter3D.hpp"

#include "Engine/Control/BipedMath3D.hpp"

#include <algorithm>
#include <cmath>

namespace MatterEngine {
namespace {

Vec3 linearJacobianColumn(const RagdollDynamics3D& dynamics,
    std::size_t link, std::size_t column) {
    if (link >= dynamics.linkJacobianRow.size()
        || column >= dynamics.jacobianColumnCount) return {};
    const std::uint32_t row = dynamics.linkJacobianRow[link];
    if (row == RagdollDynamics3D::InvalidIndex
        || row+5u >= dynamics.jacobianRowCount) return {};
    const std::size_t stride = dynamics.jacobianColumnCount;
    return {
        dynamics.denseJacobian[(row+0u)*stride+column],
        dynamics.denseJacobian[(row+1u)*stride+column],
        dynamics.denseJacobian[(row+2u)*stride+column]
    };
}

Vec3 angularJacobianColumn(const RagdollDynamics3D& dynamics,
    std::size_t link, std::size_t column) {
    if (link >= dynamics.linkJacobianRow.size()
        || column >= dynamics.jacobianColumnCount) return {};
    const std::uint32_t row = dynamics.linkJacobianRow[link];
    if (row == RagdollDynamics3D::InvalidIndex
        || row+5u >= dynamics.jacobianRowCount) return {};
    const std::size_t stride = dynamics.jacobianColumnCount;
    return {
        dynamics.denseJacobian[(row+3u)*stride+column],
        dynamics.denseJacobian[(row+4u)*stride+column],
        dynamics.denseJacobian[(row+5u)*stride+column]
    };
}

Vec3 pointJacobianColumn(const RagdollState3D& state,
    const RagdollDynamics3D& dynamics, std::size_t link,
    Vec3 pointWorld, std::size_t column) {
    if (link >= state.links.size()) return {};
    const Vec3 r = pointWorld-state.links[link].position;
    const Vec3 linear =
        linearJacobianColumn(dynamics, link, column);
    const Vec3 angular =
        angularJacobianColumn(dynamics, link, column);
    return linear+cross(angular, r);
}

std::vector<Vec3> buildComJacobian(
    const RagdollProfile3D& profile,
    const RagdollDynamics3D& dynamics) {
    std::vector<Vec3> result(dynamics.generalizedDofCount);
    float total = 0.0f;
    for (std::size_t link = 0; link < profile.links.size(); ++link) {
        const float mass = std::max(
            0.0f, profile.totalMassKg*profile.links[link].massFraction);
        total += mass;
        for (std::size_t column = 0;
                column < dynamics.generalizedDofCount; ++column) {
            result[column] +=
                linearJacobianColumn(dynamics, link, column)*mass;
        }
    }
    if (total > 1.0e-6f) {
        for (Vec3& column : result) column *= 1.0f/total;
    }
    return result;
}

}

void BipedJointTorqueField3D::reset(std::size_t linkCount) {
    torqueNewtonMeters.assign(linkCount, {});
}

void BipedJointTorqueField3D::clear() {
    for (auto& value : torqueNewtonMeters) value = {};
}

bool BipedDynamicsAdapter3D::validFor(
    const RagdollProfile3D& profile,
    const RagdollDynamics3D& dynamics) const {
    return dynamics.valid
        && dynamics.generalizedDofCount >= 6u
        && dynamics.jacobianColumnCount
            == dynamics.generalizedDofCount
        && dynamics.linkJacobianRow.size() == profile.links.size()
        && dynamics.jointGeneralizedDof.size() == profile.links.size()
        && dynamics.denseJacobian.size()
            >= static_cast<std::size_t>(dynamics.jacobianRowCount)
                *dynamics.jacobianColumnCount;
}

void BipedDynamicsAdapter3D::mapGeneralizedTorqueToJoints(
    const RagdollProfile3D& profile,
    const RagdollDynamics3D& dynamics,
    std::span<const float> generalizedTorque,
    BipedJointTorqueField3D& field,
    std::span<const std::uint8_t> enabledLinks) const {
    if (field.torqueNewtonMeters.size() != profile.links.size())
        field.reset(profile.links.size());
    for (std::size_t link = 1; link < profile.links.size(); ++link) {
        if (!enabledLinks.empty()
            && (link >= enabledLinks.size() || !enabledLinks[link])) {
            continue;
        }
        for (std::size_t axis = 0; axis < 3; ++axis) {
            if (!profile.links[link].inboundJoint.axes[axis].enabled)
                continue;
            const std::uint32_t gdof =
                dynamics.jointGeneralizedDof[link][axis];
            if (gdof == RagdollDynamics3D::InvalidIndex
                || gdof >= generalizedTorque.size()
                || !std::isfinite(generalizedTorque[gdof])) {
                continue;
            }
            field.torqueNewtonMeters[link][axis] +=
                generalizedTorque[gdof];
        }
    }
}

void BipedDynamicsAdapter3D::addComForceThroughSupport(
    const RagdollProfile3D& profile,
    const RagdollState3D& state,
    const RagdollDynamics3D& dynamics,
    std::size_t supportLink,
    Vec3 supportPointWorld,
    Vec3 forceWorld,
    BipedJointTorqueField3D& field,
    std::span<const std::uint8_t> enabledLinks) const {
    if (!validFor(profile, dynamics)
        || supportLink >= profile.links.size()
        || supportLink >= state.links.size()
        || !finiteBiped(forceWorld)) {
        return;
    }
    const auto jCom = buildComJacobian(profile, dynamics);
    std::vector<float> tau(dynamics.generalizedDofCount, 0.0f);
    for (std::size_t column = 0;
            column < dynamics.generalizedDofCount; ++column) {
        const Vec3 jSupport = pointJacobianColumn(
            state, dynamics, supportLink, supportPointWorld, column);
        const Vec3 relative = jCom[column]-jSupport;
        tau[column] = dot(relative, forceWorld);
    }
    // Floating base generalized forces are never mapped to actuators.
    for (std::size_t i = 0; i < std::min<std::size_t>(6, tau.size()); ++i)
        tau[i] = 0.0f;
    mapGeneralizedTorqueToJoints(
        profile, dynamics, tau, field, enabledLinks);
}

void BipedDynamicsAdapter3D::addComForceThroughContacts(
    const RagdollProfile3D& profile,
    const RagdollState3D& state,
    const RagdollDynamics3D& dynamics,
    std::span<const BipedSupportContact3D> contacts,
    Vec3 forceWorld,
    BipedJointTorqueField3D& field,
    std::span<const std::uint8_t> enabledLinks) const {
    float totalLoad = 0.0f;
    for (const auto& contact : contacts) {
        if (contact.linkIndex < profile.links.size())
            totalLoad += std::max(0.0f, contact.normalLoadNewtons);
    }
    if (contacts.empty()) return;
    for (const auto& contact : contacts) {
        if (contact.linkIndex >= profile.links.size()) continue;
        const float share = totalLoad > 1.0e-5f
            ? std::max(0.0f, contact.normalLoadNewtons)/totalLoad
            : 1.0f/static_cast<float>(contacts.size());
        addComForceThroughSupport(profile, state, dynamics,
            contact.linkIndex, contact.positionWorld,
            forceWorld*share, field, enabledLinks);
    }
}

void BipedDynamicsAdapter3D::addAngularTaskThroughSupport(
    const RagdollProfile3D& profile,
    const RagdollState3D&,
    const RagdollDynamics3D& dynamics,
    std::size_t taskLink,
    std::size_t supportLink,
    Vec3 torqueWorld,
    BipedJointTorqueField3D& field,
    std::span<const std::uint8_t> enabledLinks) const {
    if (!validFor(profile, dynamics)
        || taskLink >= profile.links.size()
        || supportLink >= profile.links.size()
        || !finiteBiped(torqueWorld)) {
        return;
    }
    std::vector<float> tau(dynamics.generalizedDofCount, 0.0f);
    for (std::size_t column = 0;
            column < dynamics.generalizedDofCount; ++column) {
        const Vec3 task =
            angularJacobianColumn(dynamics, taskLink, column);
        const Vec3 support =
            angularJacobianColumn(dynamics, supportLink, column);
        tau[column] = dot(task-support, torqueWorld);
    }
    for (std::size_t i = 0; i < std::min<std::size_t>(6, tau.size()); ++i)
        tau[i] = 0.0f;
    mapGeneralizedTorqueToJoints(
        profile, dynamics, tau, field, enabledLinks);
}

void BipedDynamicsAdapter3D::addAngularTaskThroughContacts(
    const RagdollProfile3D& profile,
    const RagdollState3D& state,
    const RagdollDynamics3D& dynamics,
    std::size_t taskLink,
    std::span<const BipedSupportContact3D> contacts,
    Vec3 torqueWorld,
    BipedJointTorqueField3D& field,
    std::span<const std::uint8_t> enabledLinks) const {
    float totalLoad = 0.0f;
    for (const auto& contact : contacts) {
        if (contact.linkIndex < profile.links.size())
            totalLoad += std::max(0.0f, contact.normalLoadNewtons);
    }
    if (contacts.empty()) return;
    for (const auto& contact : contacts) {
        if (contact.linkIndex >= profile.links.size()) continue;
        const float share = totalLoad > 1.0e-5f
            ? std::max(0.0f, contact.normalLoadNewtons)/totalLoad
            : 1.0f/static_cast<float>(contacts.size());
        addAngularTaskThroughSupport(profile, state, dynamics,
            taskLink, contact.linkIndex, torqueWorld*share,
            field, enabledLinks);
    }
}

} // namespace MatterEngine
