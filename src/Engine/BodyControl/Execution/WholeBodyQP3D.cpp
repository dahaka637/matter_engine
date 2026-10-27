#include "Engine/BodyControl/Execution/WholeBodyQP3D.hpp"

#include <Eigen/Dense>
#include <proxsuite/helpers/common.hpp>
#include <proxsuite/proxqp/dense/dense.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <optional>

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

void tangentBasis(const Eigen::Vector3d& normal, Eigen::Vector3d& first, Eigen::Vector3d& second) {
    const Eigen::Vector3d helper = std::abs(normal.z()) < 0.9 ? Eigen::Vector3d::UnitZ() : Eigen::Vector3d::UnitX();
    first = normal.cross(helper).normalized();
    second = normal.cross(first);
}

} // namespace

struct WholeBodyQP3D::Impl {
    std::optional<proxsuite::proxqp::dense::QP<double>> qp;
    int dimension = -1, inequalities = -1, equalities = -1;
    int updatesSincePreconditioner = 0;
};

WholeBodyQP3D::WholeBodyQP3D() : m_impl(std::make_unique<Impl>()) {}
WholeBodyQP3D::~WholeBodyQP3D() = default;

WholeBodyQPResult3D WholeBodyQP3D::solve(const WholeBodyQPInput3D& in) {
    WholeBodyQPResult3D result;
    if (!in.model || !in.kinematics || !in.dynamics || !in.state) return result;
    const ArticulationModel3D& model = *in.model;
    const ArticulationKinematics3D& k = *in.kinematics;
    const ArticulationDynamics3D& d = *in.dynamics;
    const int n = model.dofCount;
    const int joints = n - 6;
    const int contacts = static_cast<int>(in.contacts.size());
    const int dimension = n + 3 * contacts;
    const int inequalities = joints + 5 * contacts;
    if (in.torqueLimit.size() != joints) return result;
    const Eigen::VectorXd external = in.externalGeneralized.size() == n ? in.externalGeneralized : Eigen::VectorXd::Zero(n);
    const auto start = std::chrono::steady_clock::now();

    Eigen::MatrixXd hessian = Eigen::MatrixXd::Zero(dimension, dimension);
    Eigen::VectorXd gradient = Eigen::VectorXd::Zero(dimension);
    for (const WholeBodyTask3D& task : in.tasks) {
        if (task.matrix.cols() != n || task.matrix.rows() != task.target.size() || task.weight <= 0.0) continue;
        hessian.topLeftCorner(n, n).noalias() += 2.0 * task.weight * task.matrix.transpose() * task.matrix;
        gradient.head(n).noalias() -= 2.0 * task.weight * task.matrix.transpose() * task.target;
    }
    // Os Jacobianos dos pontos de contato (3 x n cada).
    Eigen::MatrixXd contactJacobian(3 * contacts, n);
    for (int c = 0; c < contacts; ++c) {
        const WholeBodyContactPoint3D& point = in.contacts[static_cast<std::size_t>(c)];
        const ArticulationJacobian3D jacobian = articulationPointJacobian3D(k, point.link, point.pointWorld);
        contactJacobian.middleRows(3 * c, 3) = jacobian;
        const Eigen::Vector3d bias = articulationPointBias3D(k, point.link, point.pointWorld);
        if (in.contactWeight > 0.0) {
            hessian.topLeftCorner(n, n).noalias() += 2.0 * in.contactWeight * jacobian.transpose() * jacobian;
            gradient.head(n).noalias() -= 2.0 * in.contactWeight * jacobian.transpose() * (point.desiredAcceleration - bias);
        }
    }
    hessian.topLeftCorner(n, n).diagonal().array() += 2.0 * in.accelerationRegularization;
    if (contacts > 0) hessian.bottomRightCorner(3 * contacts, 3 * contacts).diagonal().array() += 2.0 * in.forceRegularization;

    // Igualdades: as 6 linhas da raiz da dinamica e a aceleracao 6D dos links apoiados.
    const int rigid = static_cast<int>(in.contactLinks.size());
    int pointRows = 0;
    for (const WholeBodyContactPoint3D& point : in.contacts) pointRows += point.constrained ? 3 : 0;
    const int equalities = 6 + 6 * rigid + pointRows;
    Eigen::MatrixXd equality = Eigen::MatrixXd::Zero(equalities, dimension);
    Eigen::VectorXd equalityTarget(equalities);
    equality.topLeftCorner(6, n) = d.massMatrix.topRows(6);
    if (contacts > 0) equality.block(0, n, 6, 3 * contacts) = -contactJacobian.leftCols(6).transpose();
    equalityTarget.head(6) = external.head(6) - d.bias.head(6);
    for (int r = 0; r < rigid; ++r) {
        const WholeBodyContactLink3D& link = in.contactLinks[static_cast<std::size_t>(r)];
        const std::size_t i = static_cast<std::size_t>(link.link);
        equality.block(6 + 6 * r, 0, 3, n) = k.angularJacobian[i];
        equality.block(9 + 6 * r, 0, 3, n) = k.originJacobian[i];
        equalityTarget.segment<3>(6 + 6 * r) = link.desiredAcceleration.head<3>() - k.angularBias[i];
        equalityTarget.segment<3>(9 + 6 * r) = link.desiredAcceleration.tail<3>() - k.originBias[i];
    }
    for (int c = 0, row = 6 + 6 * rigid; c < contacts; ++c) {
        const WholeBodyContactPoint3D& point = in.contacts[static_cast<std::size_t>(c)];
        if (!point.constrained) continue;
        equality.block(row, 0, 3, n) = contactJacobian.middleRows(3 * c, 3);
        equalityTarget.segment<3>(row) = point.desiredAcceleration - articulationPointBias3D(k, point.link, point.pointWorld);
        row += 3;
    }

    // Desigualdades: o torque de cada eixo e a piramide de atrito.
    const double infinity = proxsuite::helpers::infinite_bound<double>::value();
    Eigen::MatrixXd inequality = Eigen::MatrixXd::Zero(inequalities, dimension);
    Eigen::VectorXd lower(inequalities), upper(inequalities);
    inequality.topLeftCorner(joints, n) = d.massMatrix.bottomRows(joints);
    if (contacts > 0) inequality.block(0, n, joints, 3 * contacts) = -contactJacobian.rightCols(joints).transpose();
    const Eigen::VectorXd offset = d.bias.tail(joints) - external.tail(joints);
    lower.head(joints) = -in.torqueLimit - offset;
    upper.head(joints) = in.torqueLimit - offset;
    for (int c = 0; c < contacts; ++c) {
        const WholeBodyContactPoint3D& point = in.contacts[static_cast<std::size_t>(c)];
        Eigen::Vector3d first, second;
        tangentBasis(point.normal, first, second);
        const int row = joints + 5 * c;
        const int column = n + 3 * c;
        inequality.block<1, 3>(row, column) = point.normal.transpose();
        lower[row] = point.minimumNormalForce;
        upper[row] = point.maximumNormalForce;
        const Eigen::Vector3d sides[4] = { first, -first, second, -second };
        for (int s = 0; s < 4; ++s) {
            inequality.block<1, 3>(row + 1 + s, column) = (sides[s] - point.friction * point.normal).transpose();
            lower[row + 1 + s] = -infinity;
            upper[row + 1 + s] = 0.0;
        }
    }

    const auto built = std::chrono::steady_clock::now();
    Impl& impl = *m_impl;
    const bool reuse = impl.qp && impl.dimension == dimension && impl.inequalities == inequalities && impl.equalities == equalities;
    if (!reuse) {
        impl.qp.emplace(dimension, equalities, inequalities);
        impl.qp->settings.eps_abs = 1e-5;
        impl.qp->settings.max_iter = in.maximumIterations;
        impl.qp->settings.initial_guess = proxsuite::proxqp::InitialGuessStatus::EQUALITY_CONSTRAINED_INITIAL_GUESS;
        impl.qp->init(hessian, gradient, equality, equalityTarget, inequality, lower, upper);
        impl.dimension = dimension;
        impl.inequalities = inequalities;
        impl.equalities = equalities;
    } else {
        impl.qp->settings.initial_guess = proxsuite::proxqp::InitialGuessStatus::WARM_START_WITH_PREVIOUS_RESULT;
        // O precondicionador so e refeito de tempos em tempos (refaze-lo a cada
        // tick custava ~170 us); a falha numerica refaz tudo a frio.
        const bool refresh = (++impl.updatesSincePreconditioner >= in.preconditionerRefreshTicks);
        if (refresh) impl.updatesSincePreconditioner = 0;
        impl.qp->update(hessian, gradient, equality, equalityTarget, inequality, lower, upper, refresh);
    }
    const auto updated = std::chrono::steady_clock::now();
    impl.qp->solve();
    result.buildMicros = std::chrono::duration<double, std::micro>(built - start).count();
    result.updateMicros = std::chrono::duration<double, std::micro>(updated - built).count();
    result.innerSolveMicros = std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - updated).count();
    // Falha numerica (o problema e viavel: um LP achou o ponto; nota 24):
    // reconstruir e resolver a frio uma vez antes da politica de falha.
    const auto failed = [&] {
        const auto status = impl.qp->results.info.status;
        return !impl.qp->results.x.allFinite()
            || status == proxsuite::proxqp::QPSolverOutput::PROXQP_PRIMAL_INFEASIBLE
            || status == proxsuite::proxqp::QPSolverOutput::PROXQP_DUAL_INFEASIBLE
            || status == proxsuite::proxqp::QPSolverOutput::PROXQP_SOLVED_CLOSEST_PRIMAL_FEASIBLE;
    };
    if (reuse && failed()) {
        impl.qp.emplace(dimension, equalities, inequalities);
        impl.qp->settings.eps_abs = 1e-5;
        impl.qp->settings.max_iter = in.maximumIterations;
        impl.qp->settings.initial_guess = proxsuite::proxqp::InitialGuessStatus::EQUALITY_CONSTRAINED_INITIAL_GUESS;
        impl.qp->init(hessian, gradient, equality, equalityTarget, inequality, lower, upper);
        impl.qp->solve();
        result.coldRetry = true;
    }
    const auto& output = impl.qp->results;
    result.iterations = static_cast<int>(output.info.iter);
    switch (output.info.status) {
        case proxsuite::proxqp::QPSolverOutput::PROXQP_SOLVED: result.status = WholeBodyQPStatus3D::Solved; break;
        case proxsuite::proxqp::QPSolverOutput::PROXQP_MAX_ITER_REACHED: result.status = WholeBodyQPStatus3D::MaximumIterations; break;
        default: result.status = WholeBodyQPStatus3D::Infeasible; break;
    }
    const Eigen::VectorXd& x = output.x;
    if (!x.allFinite()) {
        result.status = WholeBodyQPStatus3D::NotFinite;
        impl.qp.reset();
        return result;
    }
    result.acceleration = x.head(n);
    result.contactForces = contacts > 0 ? Eigen::VectorXd(x.tail(3 * contacts)) : Eigen::VectorXd();
    result.torque = Eigen::VectorXd::Zero(n);
    result.torque.tail(joints) = inequality.topRows(joints) * x + offset;
    result.dynamicsResidual = (equality.topRows(6) * x - equalityTarget.head(6)).norm();
    result.equalityViolation = (equality * x - equalityTarget).cwiseAbs().maxCoeff();
    const Eigen::VectorXd product = inequality * x;
    for (int r = 0; r < inequalities; ++r)
        result.inequalityViolation = std::max({ result.inequalityViolation, lower[r] - product[r], product[r] - upper[r] });
    result.solveMicros = std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - start).count();
    return result;
}

WholeBodyTask3D wholeBodyOrientationTask3D(const ArticulationKinematics3D& k, int link, const Eigen::Matrix3d& desiredRotation,
    const Eigen::Vector3d& desiredAngularVelocity, const Eigen::Vector3d& feedforward, double stiffness, double damping,
    double weight, std::string name) {
    const std::size_t i = static_cast<std::size_t>(link);
    WholeBodyTask3D task;
    task.name = std::move(name);
    task.matrix = k.angularJacobian[i];
    task.target = feedforward + stiffness * rotationVector(desiredRotation * k.rotation[i].transpose())
        + damping * (desiredAngularVelocity - k.angularVelocity[i]) - k.angularBias[i];
    task.weight = weight;
    return task;
}

WholeBodyTask3D wholeBodyPointTask3D(const ArticulationKinematics3D& k, int link, const Eigen::Vector3d& pointWorld,
    const Eigen::Vector3d& desiredPosition, const Eigen::Vector3d& desiredVelocity, const Eigen::Vector3d& feedforward,
    double stiffness, double damping, double weight, std::string name) {
    WholeBodyTask3D task;
    task.name = std::move(name);
    task.matrix = articulationPointJacobian3D(k, link, pointWorld);
    const std::size_t i = static_cast<std::size_t>(link);
    const Eigen::Vector3d velocity = k.originVelocity[i] + k.angularVelocity[i].cross(pointWorld - k.origin[i]);
    task.target = feedforward + stiffness * (desiredPosition - pointWorld) + damping * (desiredVelocity - velocity)
        - articulationPointBias3D(k, link, pointWorld);
    task.weight = weight;
    return task;
}

WholeBodyTask3D wholeBodyCentroidalTask3D(const ArticulationModel3D& model, const ArticulationDynamics3D& d,
    const ArticulationState3D& state, const Eigen::Vector3d& linearRate, const Eigen::Vector3d& angularRate,
    double linearWeight, double angularWeight) {
    (void)model;
    (void)state;
    WholeBodyTask3D task;
    task.name = "centroidal";
    task.matrix = d.centroidalMatrix;
    Eigen::Matrix<double, 6, 1> target;
    target << linearRate, angularRate;
    task.target = target - d.centroidalBias;
    // Pesos por bloco pela escala das linhas.
    const double linearScale = std::sqrt(linearWeight), angularScale = std::sqrt(angularWeight);
    task.matrix.topRows(3) *= linearScale;
    task.matrix.bottomRows(3) *= angularScale;
    task.target.head(3) *= linearScale;
    task.target.tail(3) *= angularScale;
    task.weight = 1.0;
    return task;
}

WholeBodyTask3D wholeBodyPostureTask3D(const ArticulationModel3D& model, const ArticulationState3D& state,
    const std::vector<Eigen::Vector3d>& coordinates, const std::vector<Eigen::Vector3d>& rates,
    const std::vector<Eigen::Vector3d>& accelerations, double stiffness, double damping, double weight) {
    const int n = model.dofCount;
    WholeBodyTask3D task;
    task.name = "postura";
    task.matrix = Eigen::MatrixXd::Zero(n - 6, n);
    task.target = Eigen::VectorXd::Zero(n - 6);
    for (std::size_t i = 1; i < model.links.size(); ++i) {
        for (int a = 0; a < 3; ++a) {
            const int dof = model.links[i].dof[static_cast<std::size_t>(a)];
            if (dof < 0) continue;
            task.matrix(dof - 6, dof) = 1.0;
            task.target[dof - 6] = accelerations[i][a] + stiffness * (coordinates[i][a] - state.jointCoordinates[i][a])
                + damping * (rates[i][a] - state.velocity[dof]);
        }
    }
    task.weight = weight;
    return task;
}

} // namespace MatterEngine
