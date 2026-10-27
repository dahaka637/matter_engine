#include "Engine/BodyControl/Observation/ContactWrench3D.hpp"

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>

namespace MatterEngine {

ContactWrenchEstimate3D estimateContactWrench3D(const ArticulationModel3D& model, const ArticulationState3D& before,
    const ArticulationKinematics3D& k, const ArticulationDynamics3D& d, const ArticulationState3D& after,
    const Eigen::VectorXd& muscle, const std::vector<ContactSupport3D>& supports, double dt) {
    ContactWrenchEstimate3D estimate;
    const Eigen::VectorXd acceleration = (after.velocity - before.velocity) / dt;
    estimate.generalizedExternal = d.massMatrix * acceleration + d.bias - muscle;
    const Eigen::VectorXd& external = estimate.generalizedExternal;
    // Linhas da raiz: forca total (mundo) e momento em torno da origem da pelve.
    estimate.totalForce = external.segment<3>(0);
    const Eigen::Vector3d torqueAboutRoot = external.segment<3>(3);
    estimate.totalTorqueAboutCenterOfMass = torqueAboutRoot - (d.centerOfMass - k.origin[0]).cross(estimate.totalForce);

    std::vector<int> touching;
    for (std::size_t s = 0; s < supports.size(); ++s) {
        if (supports[s].touching && supports[s].link >= 0) touching.push_back(static_cast<int>(s));
    }
    estimate.supports.assign(supports.size(), {});
    for (std::size_t s = 0; s < supports.size(); ++s) {
        if (supports[s].link < 0) continue;
        const std::size_t link = static_cast<std::size_t>(supports[s].link);
        estimate.supports[s].pointWorld = k.origin[link] + k.rotation[link] * supports[s].pointLocal;
    }
    const double total = external.norm();
    if (touching.empty()) {
        estimate.residual = total;
        estimate.residualFraction = total > 1e-9 ? 1.0 : 0.0;
        return estimate;
    }
    const int n = model.dofCount;
    Eigen::MatrixXd basis(n, 6 * static_cast<int>(touching.size()));
    for (std::size_t t = 0; t < touching.size(); ++t) {
        const ContactSupport3D& support = supports[static_cast<std::size_t>(touching[t])];
        const Eigen::Vector3d point = estimate.supports[static_cast<std::size_t>(touching[t])].pointWorld;
        basis.block(0, 6 * static_cast<int>(t), n, 3) = articulationPointJacobian3D(k, support.link, point).transpose();
        basis.block(0, 6 * static_cast<int>(t) + 3, n, 3) = k.angularJacobian[static_cast<std::size_t>(support.link)].transpose();
    }
    const Eigen::VectorXd wrenches = basis.colPivHouseholderQr().solve(external);
    estimate.residual = (external - basis * wrenches).norm();
    estimate.residualFraction = estimate.residual / std::max(total, 1e-9);
    for (std::size_t t = 0; t < touching.size(); ++t) {
        SupportWrench3D& out = estimate.supports[static_cast<std::size_t>(touching[t])];
        out.force = wrenches.segment<3>(6 * static_cast<int>(t));
        out.torque = wrenches.segment<3>(6 * static_cast<int>(t) + 3);
        if (out.force.z() > 1.0) {
            // No plano horizontal do ponto: o momento horizontal some no CoP.
            out.centerOfPressure = out.pointWorld + Eigen::Vector3d(-out.torque.y() / out.force.z(), out.torque.x() / out.force.z(), 0.0);
            out.hasCenterOfPressure = true;
        }
    }
    (void)before;
    return estimate;
}

} // namespace MatterEngine
