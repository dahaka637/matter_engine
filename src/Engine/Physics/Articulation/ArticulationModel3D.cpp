#include "Engine/Physics/Articulation/ArticulationModel3D.hpp"

#include "Engine/Physics/Articulation/ArticulationIndexing3D.hpp"

#include <Eigen/Dense>

#include <cmath>
#include <stdexcept>

namespace MatterEngine {
namespace {

Eigen::Vector3d toEigen(Vec3 v) { return { v.x, v.y, v.z }; }

Eigen::Quaterniond toEigen(Quaternion q) {
    return Eigen::Quaterniond(q.w, q.x, q.y, q.z).normalized();
}

Eigen::Matrix3d skew(const Eigen::Vector3d& v) {
    Eigen::Matrix3d m;
    m << 0.0, -v.z(), v.y(), v.z(), 0.0, -v.x(), -v.y(), v.x(), 0.0;
    return m;
}

Eigen::Vector3d rotationVector(const Eigen::Matrix3d& rotation) {
    Eigen::AngleAxisd axisAngle(rotation);
    double angle = axisAngle.angle();
    Eigen::Vector3d axis = axisAngle.axis();
    if (angle > M_PI) {
        angle = 2.0 * M_PI - angle;
        axis = -axis;
    }
    return axis * angle;
}

} // namespace

Eigen::Matrix3d exponentialRotation3D(const Eigen::Vector3d& q) {
    const double angle = q.norm();
    if (angle < 1e-12) return Eigen::Matrix3d::Identity() + skew(q);
    return Eigen::AngleAxisd(angle, q / angle).toRotationMatrix();
}

Eigen::Matrix3d exponentialLeftJacobian3D(const Eigen::Vector3d& q) {
    const double squared = q.squaredNorm();
    const double angle = std::sqrt(squared);
    const double first = squared < 1e-8 ? 0.5 - squared / 24.0 : (1.0 - std::cos(angle)) / squared;
    const double second = squared < 1e-8 ? 1.0 / 6.0 - squared / 120.0 : (angle - std::sin(angle)) / (squared * angle);
    const Eigen::Matrix3d k = skew(q);
    return Eigen::Matrix3d::Identity() + first * k + second * k * k;
}

Eigen::Matrix3d exponentialRightJacobian3D(const Eigen::Vector3d& q) {
    return exponentialLeftJacobian3D(-q);
}

Eigen::Matrix3d articulationJointRotation3D(const ArticulationLink3D& link, const Eigen::Vector3d& c) {
    if (!link.swingAfterTwist) return exponentialRotation3D(c);
    return exponentialRotation3D(Eigen::Vector3d(0.0, c.y(), c.z())) * Eigen::AngleAxisd(c.x(), Eigen::Vector3d::UnitX()).toRotationMatrix();
}

Eigen::Matrix3d articulationJointSubspace3D(const ArticulationLink3D& link, const Eigen::Vector3d& c) {
    if (!link.swingAfterTwist) return exponentialLeftJacobian3D(c);
    // Um swing so: w = R_s e_x alfa' + e_k beta'.
    Eigen::Matrix3d basis = Eigen::Matrix3d::Identity();
    basis.col(0) = exponentialRotation3D(Eigen::Vector3d(0.0, c.y(), c.z())).col(0);
    return basis;
}

Eigen::Vector3d articulationJointSubspaceRate3D(
    const ArticulationLink3D& link, const Eigen::Vector3d& c, const Eigen::Vector3d& rates) {
    if (link.swingAfterTwist) {
        const Eigen::Vector3d swingRate(0.0, rates.y(), rates.z());
        const Eigen::Vector3d twistAxis = exponentialRotation3D(Eigen::Vector3d(0.0, c.y(), c.z())).col(0);
        return swingRate.cross(twistAxis) * rates.x();
    }
    const double speed = rates.norm();
    if (speed < 1e-12) return Eigen::Vector3d::Zero();
    const double h = 1e-6 / speed;
    return (exponentialLeftJacobian3D(c + h * rates) - exponentialLeftJacobian3D(c - h * rates)) * rates / (2.0 * h);
}

Eigen::Vector3d articulationJointCoordinates3D(const ArticulationLink3D& link, const Eigen::Matrix3d& relative) {
    Eigen::Vector3d c = Eigen::Vector3d::Zero();
    if (!link.swingAfterTwist) {
        c = rotationVector(relative);
    } else {
        Eigen::Quaterniond q(relative);
        if (q.w() < 0.0) q.coeffs() = -q.coeffs();
        const double twistNorm = std::hypot(q.w(), q.x());
        const Eigen::Quaterniond twist = twistNorm > 1e-12
            ? Eigen::Quaterniond(q.w() / twistNorm, q.x() / twistNorm, 0.0, 0.0) : Eigen::Quaterniond::Identity();
        const Eigen::Quaterniond swing = q * twist.conjugate();
        c.x() = 2.0 * std::atan2(twist.x(), twist.w());
        const Eigen::Vector3d swingVector = rotationVector(swing.toRotationMatrix());
        c.y() = swingVector.y();
        c.z() = swingVector.z();
    }
    for (int a = 0; a < 3; ++a) {
        if (!link.axisEnabled[static_cast<std::size_t>(a)]) c[a] = 0.0;
    }
    return c;
}

Eigen::VectorXd articulationGeneralizedFromFeedforward3D(const ArticulationModel3D& model,
    const ArticulationState3D& state, const std::vector<Eigen::Vector3d>& feedforward) {
    Eigen::VectorXd generalized = Eigen::VectorXd::Zero(model.dofCount);
    for (std::size_t i = 1; i < model.links.size(); ++i) {
        const ArticulationLink3D& link = model.links[i];
        const Eigen::Vector3d& c = state.jointCoordinates[i];
        // Q = S^T (R_rel f): o torque no frame da junta do pai, projetado na base.
        const Eigen::Vector3d torque = articulationJointSubspace3D(link, c).transpose()
            * (articulationJointRotation3D(link, c) * feedforward[i]);
        for (int a = 0; a < 3; ++a) {
            const int dof = link.dof[static_cast<std::size_t>(a)];
            if (dof >= 0) generalized[dof] = torque[a];
        }
    }
    return generalized;
}

std::vector<Eigen::Vector3d> articulationFeedforwardFromGeneralized3D(
    const ArticulationModel3D& model, const ArticulationState3D& state, const Eigen::VectorXd& generalized) {
    std::vector<Eigen::Vector3d> feedforward(model.links.size(), Eigen::Vector3d::Zero());
    for (std::size_t i = 1; i < model.links.size(); ++i) {
        const ArticulationLink3D& link = model.links[i];
        const Eigen::Vector3d& c = state.jointCoordinates[i];
        const Eigen::Matrix3d map = articulationJointSubspace3D(link, c).transpose() * articulationJointRotation3D(link, c);
        std::array<int, 3> axes {};
        int count = 0;
        for (int a = 0; a < 3; ++a) {
            if (link.dof[static_cast<std::size_t>(a)] >= 0) axes[static_cast<std::size_t>(count++)] = a;
        }
        if (count == 0) continue;
        Eigen::MatrixXd square(count, count);
        Eigen::VectorXd target(count);
        for (int r = 0; r < count; ++r) {
            target[r] = generalized[link.dof[static_cast<std::size_t>(axes[static_cast<std::size_t>(r)])]];
            for (int col = 0; col < count; ++col) square(r, col) = map(axes[static_cast<std::size_t>(r)], axes[static_cast<std::size_t>(col)]);
        }
        const Eigen::VectorXd solution = square.partialPivLu().solve(target);
        for (int r = 0; r < count; ++r) feedforward[i][axes[static_cast<std::size_t>(r)]] = solution[r];
    }
    return feedforward;
}

ArticulationModel3D buildArticulationModel3D(const RagdollProfile3D& profile, const RagdollEffectiveMass3D& mass) {
    if (!mass.valid || mass.links.size() != profile.links.size()) {
        throw std::invalid_argument("buildArticulationModel3D: massas efetivas ausentes ou de outro perfil");
    }
    const ArticulationIndexing3D indexing = buildArticulationIndexing3D(profile);
    ArticulationModel3D model;
    model.dofCount = static_cast<int>(indexing.generalizedDofCount);
    model.links.resize(profile.links.size());
    for (std::size_t i = 0; i < profile.links.size(); ++i) {
        const RagdollLinkDefinition3D& definition = profile.links[i];
        const RagdollLinkEffectiveMass3D& effective = mass.links[i];
        ArticulationLink3D& link = model.links[i];
        link.parent = definition.parentIndex;
        if (link.parent >= static_cast<int>(i)) {
            throw std::invalid_argument("buildArticulationModel3D: pai depois do filho");
        }
        link.mass = effective.massKg;
        link.centerOfMassLocal = toEigen(effective.centerOfMassLocal);
        const Eigen::Matrix3d axes = toEigen(effective.principalAxesLocal).toRotationMatrix();
        link.inertiaLocal = axes * toEigen(effective.principalInertiaKgSquareMeters).asDiagonal() * axes.transpose();
        model.totalMass += link.mass;
        if (link.parent < 0) continue;
        const RagdollLinkDefinition3D& parent = profile.links[static_cast<std::size_t>(link.parent)];
        const Eigen::Matrix3d parentModel = toEigen(parent.modelOrientation).toRotationMatrix();
        const Eigen::Matrix3d childModel = toEigen(definition.modelOrientation).toRotationMatrix();
        const Eigen::Matrix3d frame = toEigen(definition.inboundJoint.frameModelOrientation).toRotationMatrix();
        const Eigen::Vector3d anchor = toEigen(definition.inboundJoint.anchorModelPosition);
        link.anchorInParent = parentModel.transpose() * (anchor - toEigen(parent.modelPosition));
        link.anchorInChild = childModel.transpose() * (anchor - toEigen(definition.modelPosition));
        link.jointInParent = parentModel.transpose() * frame;
        link.jointInChild = childModel.transpose() * frame;
        for (int a = 0; a < 3; ++a) {
            link.axisEnabled[static_cast<std::size_t>(a)] = definition.inboundJoint.axes[static_cast<std::size_t>(a)].enabled;
            const std::uint32_t dof = indexing.jointGeneralizedDof[i][static_cast<std::size_t>(a)];
            link.dof[static_cast<std::size_t>(a)] = dof == ArticulationIndexing3D::InvalidIndex ? -1 : static_cast<int>(dof);
        }
        const int swings = int(link.axisEnabled[1]) + int(link.axisEnabled[2]);
        link.swingAfterTwist = link.axisEnabled[0] && swings == 1;
    }
    return model;
}

ArticulationState3D articulationStateFromRagdoll3D(const ArticulationModel3D& model, const RagdollState3D& state) {
    if (state.links.size() != model.links.size()) {
        throw std::invalid_argument("articulationStateFromRagdoll3D: estado de outro perfil");
    }
    ArticulationState3D result;
    result.jointCoordinates.assign(model.links.size(), Eigen::Vector3d::Zero());
    result.velocity = Eigen::VectorXd::Zero(model.dofCount);
    std::vector<Eigen::Matrix3d> rotation(model.links.size());
    std::vector<Eigen::Vector3d> omega(model.links.size());
    for (std::size_t i = 0; i < model.links.size(); ++i) {
        rotation[i] = toEigen(state.links[i].orientation).toRotationMatrix();
        omega[i] = toEigen(state.links[i].angularVelocity);
    }
    const PhysicsBodyState3D& root = state.links.front();
    result.rootPosition = toEigen(root.position);
    result.rootOrientation = toEigen(root.orientation);
    // O backend publica a velocidade do centro de massa; a coordenada e a da origem.
    const Eigen::Vector3d rootOffset = rotation[0] * model.links[0].centerOfMassLocal;
    result.velocity.segment<3>(0) = toEigen(root.linearVelocity) - omega[0].cross(rootOffset);
    result.velocity.segment<3>(3) = omega[0];
    for (std::size_t i = 1; i < model.links.size(); ++i) {
        const ArticulationLink3D& link = model.links[i];
        const std::size_t parent = static_cast<std::size_t>(link.parent);
        const Eigen::Matrix3d relative = link.jointInParent.transpose() * rotation[parent].transpose() * rotation[i]
            * link.jointInChild;
        const Eigen::Vector3d q = articulationJointCoordinates3D(link, relative);
        int enabledCount = 0;
        for (int a = 0; a < 3; ++a) {
            if (link.axisEnabled[static_cast<std::size_t>(a)]) ++enabledCount;
        }
        result.jointCoordinates[i] = q;
        if (enabledCount == 0) continue;
        // w_rel no frame da junta do pai = S(q) qdot.
        const Eigen::Vector3d relativeOmega = (rotation[parent] * link.jointInParent).transpose() * (omega[i] - omega[parent]);
        const Eigen::Matrix3d subspace = articulationJointSubspace3D(link, q);
        Eigen::MatrixXd basis(3, enabledCount);
        int column = 0;
        for (int a = 0; a < 3; ++a) {
            if (link.axisEnabled[static_cast<std::size_t>(a)]) basis.col(column++) = subspace.col(a);
        }
        const Eigen::VectorXd rates = basis.colPivHouseholderQr().solve(relativeOmega);
        column = 0;
        for (int a = 0; a < 3; ++a) {
            if (link.dof[static_cast<std::size_t>(a)] >= 0) result.velocity[link.dof[static_cast<std::size_t>(a)]] = rates[column++];
        }
    }
    return result;
}

} // namespace MatterEngine
