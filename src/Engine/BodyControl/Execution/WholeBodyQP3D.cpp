#include "Engine/BodyControl/Execution/WholeBodyQP3D.hpp"

#include <Eigen/Dense>
#include <proxsuite/helpers/common.hpp>
#include <proxsuite/proxqp/dense/dense.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <optional>
#include <tuple>

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
    // Uma instancia por nivel (as dimensoes diferem): 0 o de cima (ou o unico), 1 o de baixo.
    struct Slot {
        std::optional<proxsuite::proxqp::dense::QP<double>> qp;
        int dimension = -1, inequalities = -1, equalities = -1;
        int updatesSincePreconditioner = 0;
    };
    Slot slots[2];
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
    std::vector<int> guarded;
    if (in.jointLower.size() == joints && in.jointUpper.size() == joints)
        for (int j = 0; j < joints; ++j)
            if (std::isfinite(in.jointLower[j]) || std::isfinite(in.jointUpper[j])) guarded.push_back(j);
    const int inequalities = joints + 5 * contacts + static_cast<int>(guarded.size());
    if (in.torqueLimit.size() != joints) return result;
    const Eigen::VectorXd external = in.externalGeneralized.size() == n ? in.externalGeneralized : Eigen::VectorXd::Zero(n);
    const auto start = std::chrono::steady_clock::now();

    Eigen::MatrixXd hessian = Eigen::MatrixXd::Zero(dimension, dimension);
    Eigen::VectorXd gradient = Eigen::VectorXd::Zero(dimension);
    // As tarefas do nivel de cima somam a parte deles a parte; as linhas delas viram igualdades embaixo.
    Eigen::MatrixXd primaryHessian = Eigen::MatrixXd::Zero(n, n);
    Eigen::VectorXd primaryGradient = Eigen::VectorXd::Zero(n);
    int primaryRows = 0;
    for (const WholeBodyTask3D& task : in.tasks) {
        if (task.matrix.cols() != n || task.matrix.rows() != task.target.size() || task.weight <= 0.0) continue;
        hessian.topLeftCorner(n, n).noalias() += 2.0 * task.weight * task.matrix.transpose() * task.matrix;
        gradient.head(n).noalias() -= 2.0 * task.weight * task.matrix.transpose() * task.target;
        if (task.priority != 0) continue;
        primaryHessian.noalias() += 2.0 * task.weight * task.matrix.transpose() * task.matrix;
        primaryGradient.noalias() -= 2.0 * task.weight * task.matrix.transpose() * task.target;
        primaryRows += static_cast<int>(task.matrix.rows());
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
    for (const WholeBodyContactPoint3D& point : in.contacts) pointRows += point.constrained ? (point.freeDirection.isZero() ? 3 : 2) : 0;
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
        const Eigen::Vector3d target = point.desiredAcceleration - articulationPointBias3D(k, point.link, point.pointWorld);
        if (point.freeDirection.isZero()) {
            equality.block(row, 0, 3, n) = contactJacobian.middleRows(3 * c, 3);
            equalityTarget.segment<3>(row) = target;
            row += 3;
            continue;
        }
        const Eigen::Vector3d free = point.freeDirection.normalized();
        const Eigen::Vector3d first = free.unitOrthogonal(), second = free.cross(first);
        for (const Eigen::Vector3d& axis : { first, second }) {
            equality.row(row).head(n) = axis.transpose() * contactJacobian.middleRows(3 * c, 3);
            equalityTarget[row] = axis.dot(target);
            ++row;
        }
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
    // A guarda dos batentes, na aceleracao do eixo (recuperacao critica, nota 28).
    for (std::size_t g = 0; g < guarded.size(); ++g) {
        const int j = guarded[g];
        const int row = joints + 5 * contacts + static_cast<int>(g);
        const int dof = j + 6;
        double coordinate = 0.0;
        for (std::size_t l = 1; l < model.links.size(); ++l)
            for (int a = 0; a < 3; ++a)
                if (model.links[l].dof[static_cast<std::size_t>(a)] == dof) coordinate = in.state->jointCoordinates[l][a];
        const double v = in.state->velocity[dof];
        const double margin = in.jointMargin.size() == joints ? in.jointMargin[j] : in.jointGuardMargin;
        const double rate = in.jointGuardRate;
        inequality(row, dof) = 1.0;
        lower[row] = std::isfinite(in.jointLower[j]) ? -rate * rate * (coordinate - in.jointLower[j] - margin) - 2.0 * rate * v : -infinity;
        upper[row] = std::isfinite(in.jointUpper[j]) ? rate * rate * (in.jointUpper[j] - margin - coordinate) - 2.0 * rate * v : infinity;
        if (lower[row] > upper[row]) lower[row] = upper[row];
    }
    // Resolve um nivel numa instancia: reaproveita a anterior (precondicionador de
    // tempos em tempos: refaze-lo a cada tick custava ~170 us); a falha numerica
    // (o problema e viavel: um LP achou o ponto; nota 24) reconstroi a frio uma vez.
    const auto runSlot = [&](Impl::Slot& slot, const Eigen::MatrixXd& H, const Eigen::VectorXd& g, const Eigen::MatrixXd& E,
                             const Eigen::VectorXd& e) -> const proxsuite::proxqp::Results<double>& {
        const int rows = static_cast<int>(E.rows());
        const auto fresh = [&] {
            slot.qp.emplace(dimension, rows, inequalities);
            slot.qp->settings.eps_abs = 1e-5;
            slot.qp->settings.max_iter = in.maximumIterations;
            slot.qp->settings.initial_guess = proxsuite::proxqp::InitialGuessStatus::EQUALITY_CONSTRAINED_INITIAL_GUESS;
            slot.qp->init(H, g, E, e, inequality, lower, upper);
            slot.dimension = dimension;
            slot.inequalities = inequalities;
            slot.equalities = rows;
        };
        const bool reuse = slot.qp && slot.dimension == dimension && slot.inequalities == inequalities && slot.equalities == rows;
        if (!reuse) {
            fresh();
        } else {
            slot.qp->settings.initial_guess = proxsuite::proxqp::InitialGuessStatus::WARM_START_WITH_PREVIOUS_RESULT;
            const bool refresh = (++slot.updatesSincePreconditioner >= in.preconditionerRefreshTicks);
            if (refresh) slot.updatesSincePreconditioner = 0;
            slot.qp->update(H, g, E, e, inequality, lower, upper, refresh);
        }
        slot.qp->solve();
        result.iterations += static_cast<int>(slot.qp->results.info.iter);
        const auto status = slot.qp->results.info.status;
        const bool failed = !slot.qp->results.x.allFinite() || status == proxsuite::proxqp::QPSolverOutput::PROXQP_PRIMAL_INFEASIBLE
            || status == proxsuite::proxqp::QPSolverOutput::PROXQP_DUAL_INFEASIBLE
            || status == proxsuite::proxqp::QPSolverOutput::PROXQP_SOLVED_CLOSEST_PRIMAL_FEASIBLE;
        if (reuse && failed) {
            fresh();
            slot.qp->solve();
            result.iterations += static_cast<int>(slot.qp->results.info.iter);
            result.coldRetry = true;
        }
        return slot.qp->results;
    };
    // Aceita so com status bom E restricoes cumpridas (o teto de iteracoes devolvia
    // solucoes com violacao 17 nas igualdades; nota 28).
    const auto violations = [&](const Eigen::VectorXd& x, const Eigen::MatrixXd& E, const Eigen::VectorXd& e) {
        const double equalityWorst = E.rows() ? (E * x - e).cwiseAbs().maxCoeff() : 0.0;
        const Eigen::VectorXd product = inequality * x;
        double inequalityWorst = 0.0;
        for (int r = 0; r < inequalities; ++r) inequalityWorst = std::max({ inequalityWorst, lower[r] - product[r], product[r] - upper[r] });
        return std::make_pair(equalityWorst, inequalityWorst);
    };
    const auto acceptable = [&](const proxsuite::proxqp::Results<double>& r, const Eigen::MatrixXd& E, const Eigen::VectorXd& e) {
        if (!r.x.allFinite()) return false;
        if (r.info.status != proxsuite::proxqp::QPSolverOutput::PROXQP_SOLVED
            && r.info.status != proxsuite::proxqp::QPSolverOutput::PROXQP_MAX_ITER_REACHED) return false;
        const auto [equalityWorst, inequalityWorst] = violations(r.x, E, e);
        return equalityWorst <= in.acceptanceTolerance && inequalityWorst <= in.acceptanceTolerance;
    };
    Impl& impl = *m_impl;
    const auto updated = std::chrono::steady_clock::now();
    const proxsuite::proxqp::Results<double>* final = nullptr;
    // A tentativa pesada reusa o slot de cima: o resultado de cima e guardado antes (nota 29).
    proxsuite::proxqp::Results<double> topSaved;
    // As igualdades da solucao devolvida (as de baixo incluem as que guardam o nivel de cima).
    const Eigen::MatrixXd* finalEquality = &equality;
    const Eigen::VectorXd* finalTarget = &equalityTarget;
    Eigen::MatrixXd lowerEquality;
    Eigen::VectorXd lowerTarget;
    if (primaryRows > 0) {
        // Nivel de cima: so as tarefas de prioridade 0 (com as mesmas regularizacoes e restricoes).
        Eigen::MatrixXd topHessian = hessian;
        Eigen::VectorXd topGradient = gradient;
        topHessian.topLeftCorner(n, n) = primaryHessian;
        topGradient.head(n) = primaryGradient;
        topHessian.topLeftCorner(n, n).diagonal().array() += 2.0 * in.accelerationRegularization;
        for (int c = 0; c < contacts && in.contactWeight > 0.0; ++c) {
            const WholeBodyContactPoint3D& point = in.contacts[static_cast<std::size_t>(c)];
            const Eigen::MatrixXd jacobian = contactJacobian.middleRows(3 * c, 3);
            topHessian.topLeftCorner(n, n).noalias() += 2.0 * in.contactWeight * jacobian.transpose() * jacobian;
            topGradient.head(n).noalias() -= 2.0 * in.contactWeight * jacobian.transpose()
                * (point.desiredAcceleration - articulationPointBias3D(k, point.link, point.pointWorld));
        }
        const auto& top = runSlot(impl.slots[0], topHessian, topGradient, equality, equalityTarget);
        if (acceptable(top, equality, equalityTarget)) {
            // Nivel de baixo: tudo, com os valores alcancados em cima fixados.
            lowerEquality = Eigen::MatrixXd::Zero(equalities + primaryRows, dimension);
            lowerTarget.resize(equalities + primaryRows);
            lowerEquality.topRows(equalities) = equality;
            lowerTarget.head(equalities) = equalityTarget;
            int row = equalities;
            const Eigen::VectorXd topAcceleration = top.x.head(n);
            for (const WholeBodyTask3D& task : in.tasks) {
                if (task.priority != 0 || task.matrix.cols() != n || task.matrix.rows() != task.target.size() || task.weight <= 0.0) continue;
                const int r = static_cast<int>(task.matrix.rows());
                lowerEquality.block(row, 0, r, n) = task.matrix;
                lowerTarget.segment(row, r) = task.matrix * topAcceleration;
                row += r;
            }
            const auto& bottom = runSlot(impl.slots[1], hessian, gradient, lowerEquality, lowerTarget);
            if (acceptable(bottom, lowerEquality, lowerTarget)) {
                final = &bottom;
                finalEquality = &lowerEquality;
                finalTarget = &lowerTarget;
                result.path = WholeBodyQPPath3D::Hierarchical;
            } else {
                // O de baixo falha por numero (nota 28: um LP acha o ponto). Antes de
                // largar a altura e a postura, um nivel so com as de cima muito mais pesadas.
                topSaved = top;
                Eigen::MatrixXd heavyHessian = hessian;
                Eigen::VectorXd heavyGradient = gradient;
                heavyHessian.topLeftCorner(n, n) += (in.priorityWeightScale - 1.0) * primaryHessian;
                heavyGradient.head(n) += (in.priorityWeightScale - 1.0) * primaryGradient;
                const auto& heavy = runSlot(impl.slots[0], heavyHessian, heavyGradient, equality, equalityTarget);
                if (acceptable(heavy, equality, equalityTarget)) {
                    final = &heavy;
                    result.path = WholeBodyQPPath3D::WeightedPriority;
                } else {
                    final = &topSaved;
                    result.path = WholeBodyQPPath3D::TopOnly;
                }
            }
        }
    }
    if (!final) {
        final = &runSlot(impl.slots[0], hessian, gradient, equality, equalityTarget);
        if (primaryRows > 0) result.path = WholeBodyQPPath3D::FlatFallback;
    }
    result.buildMicros = std::chrono::duration<double, std::micro>(built - start).count();
    result.innerSolveMicros = std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - updated).count();
    const auto& output = *final;
    switch (output.info.status) {
        case proxsuite::proxqp::QPSolverOutput::PROXQP_SOLVED: result.status = WholeBodyQPStatus3D::Solved; break;
        case proxsuite::proxqp::QPSolverOutput::PROXQP_MAX_ITER_REACHED: result.status = WholeBodyQPStatus3D::MaximumIterations; break;
        default: result.status = WholeBodyQPStatus3D::Infeasible; break;
    }
    // A solucao fora da tolerancia nunca e usada (a politica de falha do chamador entra).
    if (result.status != WholeBodyQPStatus3D::Infeasible && output.x.allFinite() && !acceptable(output, *finalEquality, *finalTarget))
        result.status = WholeBodyQPStatus3D::Infeasible;
    const Eigen::VectorXd& x = output.x;
    if (!x.allFinite()) {
        result.status = WholeBodyQPStatus3D::NotFinite;
        for (Impl::Slot& slot : impl.slots) slot.qp.reset();
        return result;
    }
    result.acceleration = x.head(n);
    result.contactForces = contacts > 0 ? Eigen::VectorXd(x.tail(3 * contacts)) : Eigen::VectorXd();
    result.torque = Eigen::VectorXd::Zero(n);
    result.torque.tail(joints) = inequality.topRows(joints) * x + offset;
    result.dynamicsResidual = (equality.topRows(6) * x - equalityTarget.head(6)).norm();
    std::tie(result.equalityViolation, result.inequalityViolation) = violations(x, *finalEquality, *finalTarget);
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
    (void)state;
    WholeBodyTask3D task;
    task.name = "centroidal";
    task.matrix = d.centroidalMatrix;
    Eigen::Matrix<double, 6, 1> target;
    target << linearRate, angularRate;
    task.target = target - d.centroidalBias;
    // Pesos por bloco pela escala das linhas, com as unidades normalizadas pela
    // massa (linear em m/s2, angular por m * 1 m2): sem isso o custo carregava m2
    // e a tarefa centroidal dominava o tronco e a cabeca (nota 25).
    const double mass = model.totalMass;
    const double linearScale = std::sqrt(linearWeight) / mass, angularScale = std::sqrt(angularWeight) / mass;
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
