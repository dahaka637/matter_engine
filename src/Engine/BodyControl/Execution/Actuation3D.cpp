#include "Engine/BodyControl/Execution/Actuation3D.hpp"

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>

namespace MatterEngine {
namespace {

Eigen::Vector3d rotationVector(const Eigen::Matrix3d& rotation) {
    const Eigen::AngleAxisd axisAngle(rotation);
    double angle = axisAngle.angle();
    Eigen::Vector3d axis = axisAngle.axis();
    if (angle > M_PI) {
        angle = 2.0 * M_PI - angle;
        axis = -axis;
    }
    return axis * angle;
}

// No backend ativo a mola e k_perfil * escala * 5 e o amortecimento d_perfil * escala * 2.
constexpr double ActiveStiffnessMultiplier = 5.0;
constexpr double ActiveDampingMultiplier = 2.0;

} // namespace

std::vector<RagdollDriveTarget3D> actuationTargets3D(const RagdollProfile3D& profile, const ArticulationModel3D& model,
    const ArticulationState3D& state, const Eigen::VectorXd& generalizedTorque, const ActuationReference3D& reference,
    const Eigen::VectorXd& effectiveInertia, const ActuationSettings3D& settings) {
    const std::vector<Eigen::Vector3d> feedforward = articulationFeedforwardFromGeneralized3D(model, state, generalizedTorque);
    std::vector<RagdollDriveTarget3D> targets;
    targets.reserve(static_cast<std::size_t>(model.dofCount));
    const double dt = settings.timeStep;
    for (std::size_t i = 1; i < model.links.size(); ++i) {
        const ArticulationLink3D& link = model.links[i];
        const Eigen::Vector3d& c = state.jointCoordinates[i];
        const Eigen::Matrix3d relative = articulationJointRotation3D(link, c);
        // Velocidade relativa no frame da junta do filho (a do motor): R^T S qdot.
        Eigen::Vector3d rates = Eigen::Vector3d::Zero();
        for (int a = 0; a < 3; ++a) {
            const int dof = link.dof[static_cast<std::size_t>(a)];
            if (dof >= 0) rates[a] = state.velocity[dof];
        }
        const Eigen::Vector3d childOmega = relative.transpose() * (articulationJointSubspace3D(link, c) * rates);
        // O alvo da mola (B) na convencao do motor: o vetor de rotacao.
        Eigen::Vector3d targetAngles = rotationVector(relative);
        Eigen::Vector3d targetOmega = childOmega;
        if (settings.mode == ActuationMode3D::LowImpedance && i < reference.coordinates.size()) {
            const Eigen::Matrix3d targetRelative = articulationJointRotation3D(link, reference.coordinates[i]);
            targetAngles = rotationVector(targetRelative);
            targetOmega = targetRelative.transpose() * (articulationJointSubspace3D(link, reference.coordinates[i]) * reference.rates[i]);
        }
        const Eigen::Vector3d currentAngles = rotationVector(relative);
        for (int a = 0; a < 3; ++a) {
            const RagdollAxisDefinition3D& axis = profile.links[i].inboundJoint.axes[static_cast<std::size_t>(a)];
            const int dof = link.dof[static_cast<std::size_t>(a)];
            if (!axis.enabled || dof < 0) continue;
            RagdollDriveTarget3D target;
            target.linkIndex = static_cast<std::uint32_t>(i);
            target.axis = static_cast<RagdollAxis3D>(a);
            target.positionRadians = static_cast<float>(targetAngles[a]);
            target.velocityRadiansPerSecond = static_cast<float>(targetOmega[a]);
            target.maximumTorqueScale = 1.0f;
            target.gravityCompensationScale = 0.0f;
            target.stiffnessScale = 0.0f;
            target.dampingScale = 0.0f;
            double torque = feedforward[i][a];
            const double inertia = std::max(1e-4, effectiveInertia[dof]);
            if (settings.mode == ActuationMode3D::ExplicitTorqueImplicitDamping) {
                // So amortecimento, contra a variacao da velocidade dentro do passo.
                const double damping = settings.dampingPerInertia * inertia;
                target.dampingScale = static_cast<float>(std::clamp(
                    damping / (ActiveDampingMultiplier * std::max(1e-6, double(axis.damping))), 0.0, 2.5));
                target.velocityRadiansPerSecond = static_cast<float>(childOmega[a]);
            } else if (settings.mode == ActuationMode3D::LowImpedance) {
                const double stiffness = settings.stiffnessPerInertia * inertia;
                const double damping = settings.dampingPerInertia * inertia;
                const float stiffnessScale = static_cast<float>(std::clamp(
                    stiffness / (ActiveStiffnessMultiplier * std::max(1e-6, double(axis.stiffness))), 0.0, 2.0));
                const float dampingScale = static_cast<float>(std::clamp(
                    damping / (ActiveDampingMultiplier * std::max(1e-6, double(axis.damping))), 0.0, 2.5));
                target.stiffnessScale = stiffnessScale;
                target.dampingScale = dampingScale;
                const double k = ActiveStiffnessMultiplier * axis.stiffness * stiffnessScale;
                const double d = ActiveDampingMultiplier * axis.damping * dampingScale;
                // A resposta discreta prevista da mola implicita (um eixo, inercia efetiva).
                const double error = targetAngles[a] - currentAngles[a];
                const double spring = (k * error + d * (targetOmega[a] - childOmega[a])) * inertia / (inertia + d * dt + k * dt * dt);
                torque -= spring;
            }
            target.feedforwardTorqueNewtonMeters = static_cast<float>(torque);
            targets.push_back(target);
        }
    }
    return targets;
}

ActuationDelivery3D actuationDelivery3D(const RagdollProfile3D& profile, const RagdollState3D& state) {
    ActuationDelivery3D delivery;
    delivery.muscleTorque.assign(state.joints.size(), Eigen::Vector3d::Zero());
    for (std::size_t i = 1; i < state.joints.size() && i < profile.links.size(); ++i) {
        const RagdollJointState3D& joint = state.joints[i];
        for (int a = 0; a < 3; ++a) {
            if (!profile.links[i].inboundJoint.axes[static_cast<std::size_t>(a)].enabled) continue;
            const std::size_t axis = static_cast<std::size_t>(a);
            const double muscle = double(joint.motorTorqueNewtonMeters[axis]) + joint.feedforwardTorqueNewtonMeters[axis];
            delivery.muscleTorque[i][a] = muscle;
            delivery.worstBudgetExcessNewtonMeters = std::max(delivery.worstBudgetExcessNewtonMeters,
                std::abs(muscle) - double(joint.motorTorqueLimitNewtonMeters[axis]));
        }
    }
    return delivery;
}

} // namespace MatterEngine
