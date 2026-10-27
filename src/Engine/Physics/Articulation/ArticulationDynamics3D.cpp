#include "Engine/Physics/Articulation/ArticulationDynamics3D.hpp"

#include <Eigen/Dense>

#include <cmath>

namespace MatterEngine {
namespace {

Eigen::Matrix3d skew(const Eigen::Vector3d& v) {
    Eigen::Matrix3d m;
    m << 0.0, -v.z(), v.y(), v.z(), 0.0, -v.x(), -v.y(), v.x(), 0.0;
    return m;
}

Eigen::Vector3d jointRates(const ArticulationLink3D& link, const Eigen::VectorXd& velocity) {
    Eigen::Vector3d rates = Eigen::Vector3d::Zero();
    for (int a = 0; a < 3; ++a) {
        const int dof = link.dof[static_cast<std::size_t>(a)];
        if (dof >= 0) rates[a] = velocity[dof];
    }
    return rates;
}

} // namespace

ArticulationKinematics3D computeArticulationKinematics3D(
    const ArticulationModel3D& model, const ArticulationState3D& state) {
    const std::size_t count = model.links.size();
    const int n = model.dofCount;
    ArticulationKinematics3D k;
    k.rotation.resize(count);
    k.origin.resize(count);
    k.centerOfMass.resize(count);
    k.angularVelocity.resize(count);
    k.originVelocity.resize(count);
    k.angularBias.resize(count);
    k.originBias.resize(count);
    k.angularJacobian.assign(count, ArticulationJacobian3D::Zero(3, n));
    k.originJacobian.assign(count, ArticulationJacobian3D::Zero(3, n));
    const Eigen::VectorXd& v = state.velocity;
    for (std::size_t i = 0; i < count; ++i) {
        const ArticulationLink3D& link = model.links[i];
        if (link.parent < 0) {
            k.rotation[i] = state.rootOrientation.toRotationMatrix();
            k.origin[i] = state.rootPosition;
            k.angularVelocity[i] = v.segment<3>(3);
            k.originVelocity[i] = v.segment<3>(0);
            k.angularBias[i].setZero();
            k.originBias[i].setZero();
            k.angularJacobian[i].block<3, 3>(0, 3).setIdentity();
            k.originJacobian[i].block<3, 3>(0, 0).setIdentity();
        } else {
            const std::size_t p = static_cast<std::size_t>(link.parent);
            const Eigen::Vector3d& q = state.jointCoordinates[i];
            const Eigen::Matrix3d jointWorld = k.rotation[p] * link.jointInParent;
            const Eigen::Matrix3d left = articulationJointSubspace3D(link, q);
            k.rotation[i] = jointWorld * articulationJointRotation3D(link, q) * link.jointInChild.transpose();
            const Eigen::Vector3d anchor = k.origin[p] + k.rotation[p] * link.anchorInParent;
            k.origin[i] = anchor - k.rotation[i] * link.anchorInChild;
            const Eigen::Vector3d rates = jointRates(link, v);
            const Eigen::Vector3d relative = jointWorld * (left * rates);
            const Eigen::Vector3d& wp = k.angularVelocity[p];
            k.angularVelocity[i] = wp + relative;
            k.originVelocity[i] = k.originVelocity[p] + wp.cross(k.origin[i] - k.origin[p])
                + relative.cross(k.origin[i] - anchor);
            k.angularJacobian[i] = k.angularJacobian[p];
            for (int a = 0; a < 3; ++a) {
                const int dof = link.dof[static_cast<std::size_t>(a)];
                if (dof >= 0) k.angularJacobian[i].col(dof) += jointWorld * left.col(a);
            }
            k.originJacobian[i] = k.originJacobian[p] - skew(k.origin[i] - k.origin[p]) * k.angularJacobian[p]
                - skew(k.origin[i] - anchor) * (k.angularJacobian[i] - k.angularJacobian[p]);
            const Eigen::Vector3d leftRate = articulationJointSubspaceRate3D(link, q, rates);
            k.angularBias[i] = k.angularBias[p] + wp.cross(relative) + jointWorld * leftRate;
            const Eigen::Vector3d toAnchor = anchor - k.origin[p];
            const Eigen::Vector3d anchorBias = k.originBias[p] + k.angularBias[p].cross(toAnchor)
                + wp.cross(wp.cross(toAnchor));
            const Eigen::Vector3d fromAnchor = k.origin[i] - anchor;
            const Eigen::Vector3d& wi = k.angularVelocity[i];
            k.originBias[i] = anchorBias + k.angularBias[i].cross(fromAnchor) + wi.cross(wi.cross(fromAnchor));
        }
        k.centerOfMass[i] = k.origin[i] + k.rotation[i] * link.centerOfMassLocal;
    }
    return k;
}

ArticulationJacobian3D articulationPointJacobian3D(
    const ArticulationKinematics3D& kinematics, int link, const Eigen::Vector3d& pointWorld) {
    const std::size_t i = static_cast<std::size_t>(link);
    return kinematics.originJacobian[i] - skew(pointWorld - kinematics.origin[i]) * kinematics.angularJacobian[i];
}

Eigen::Vector3d articulationPointBias3D(
    const ArticulationKinematics3D& kinematics, int link, const Eigen::Vector3d& pointWorld) {
    const std::size_t i = static_cast<std::size_t>(link);
    const Eigen::Vector3d r = pointWorld - kinematics.origin[i];
    const Eigen::Vector3d& w = kinematics.angularVelocity[i];
    return kinematics.originBias[i] + kinematics.angularBias[i].cross(r) + w.cross(w.cross(r));
}

ArticulationDynamics3D computeArticulationDynamics3D(
    const ArticulationModel3D& model, const ArticulationKinematics3D& k) {
    const int n = model.dofCount;
    ArticulationDynamics3D d;
    d.massMatrix = Eigen::MatrixXd::Zero(n, n);
    d.bias = Eigen::VectorXd::Zero(n);
    d.centroidalMatrix = Eigen::Matrix<double, 6, Eigen::Dynamic>::Zero(6, n);
    for (std::size_t i = 0; i < model.links.size(); ++i) {
        d.centerOfMass += model.links[i].mass * k.centerOfMass[i];
    }
    d.centerOfMass /= model.totalMass;
    for (std::size_t i = 0; i < model.links.size(); ++i) {
        const ArticulationLink3D& link = model.links[i];
        const ArticulationJacobian3D comJacobian = articulationPointJacobian3D(k, static_cast<int>(i), k.centerOfMass[i]);
        const Eigen::Vector3d comBias = articulationPointBias3D(k, static_cast<int>(i), k.centerOfMass[i]);
        const Eigen::Matrix3d inertia = k.rotation[i] * link.inertiaLocal * k.rotation[i].transpose();
        const ArticulationJacobian3D& angular = k.angularJacobian[i];
        const Eigen::Vector3d& w = k.angularVelocity[i];
        d.massMatrix.noalias() += link.mass * comJacobian.transpose() * comJacobian;
        d.massMatrix.noalias() += angular.transpose() * inertia * angular;
        const Eigen::Vector3d angularBiasMoment = inertia * k.angularBias[i]
            + (model.gyroscopic ? Eigen::Vector3d(w.cross(inertia * w)) : Eigen::Vector3d::Zero());
        d.bias.noalias() += comJacobian.transpose() * (link.mass * (comBias - model.gravity));
        d.bias.noalias() += angular.transpose() * angularBiasMoment;
        const Eigen::Vector3d arm = k.centerOfMass[i] - d.centerOfMass;
        d.centroidalMatrix.topRows<3>() += link.mass * comJacobian;
        d.centroidalMatrix.bottomRows<3>() += link.mass * skew(arm) * comJacobian + inertia * angular;
        d.centroidalBias.head<3>() += link.mass * comBias;
        d.centroidalBias.tail<3>() += arm.cross(link.mass * comBias) + angularBiasMoment;
    }
    return d;
}

ArticulationState3D integrateArticulationState3D(
    const ArticulationModel3D& model, const ArticulationState3D& state, const Eigen::VectorXd& velocity, double dt) {
    ArticulationState3D next = state;
    next.velocity = velocity;
    next.rootPosition += velocity.segment<3>(0) * dt;
    const Eigen::Vector3d spin = velocity.segment<3>(3) * dt;
    next.rootOrientation = (Eigen::Quaterniond(exponentialRotation3D(spin)) * state.rootOrientation).normalized();
    for (std::size_t i = 1; i < model.links.size(); ++i) {
        next.jointCoordinates[i] += jointRates(model.links[i], velocity) * dt;
    }
    return next;
}

double articulationKineticEnergy3D(const ArticulationModel3D& model, const ArticulationKinematics3D& k) {
    double energy = 0.0;
    for (std::size_t i = 0; i < model.links.size(); ++i) {
        const ArticulationLink3D& link = model.links[i];
        const Eigen::Vector3d r = k.centerOfMass[i] - k.origin[i];
        const Eigen::Vector3d velocity = k.originVelocity[i] + k.angularVelocity[i].cross(r);
        const Eigen::Matrix3d inertia = k.rotation[i] * link.inertiaLocal * k.rotation[i].transpose();
        energy += 0.5 * link.mass * velocity.squaredNorm()
            + 0.5 * k.angularVelocity[i].dot(inertia * k.angularVelocity[i]);
    }
    return energy;
}

} // namespace MatterEngine
