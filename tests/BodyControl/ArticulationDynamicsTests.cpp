// O modelo dinamico do corpo (G0 da locomocao fisica nova): a cinematica e os
// Jacobianos contra diferencas finitas, a matriz de massa contra a energia, o
// momento centroidal, a conversao do estado do Jolt e a aceleracao prevista
// contra a do Jolt no ar, com juntas giradas (plano, secao 6.1).
#include "Engine/Materials/MaterialLibrary.hpp"
#include "Engine/Physics/Articulation/ArticulationDynamics3D.hpp"
#include "Engine/Physics/PhysicsEngine3D.hpp"

#include <Eigen/Dense>

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <random>
#include <string>

using namespace MatterEngine;

namespace {

int failures = 0;

void check(bool condition, const std::string& message) {
    if (!condition) {
        ++failures;
        std::printf("FALHOU: %s\n", message.c_str());
    }
}

Vec3 toVec3(const Eigen::Vector3d& v) {
    return { static_cast<float>(v.x()), static_cast<float>(v.y()), static_cast<float>(v.z()) };
}

Quaternion toQuaternion(const Eigen::Matrix3d& rotation) {
    const Eigen::Quaterniond q(rotation);
    return Quaternion(static_cast<float>(q.x()), static_cast<float>(q.y()), static_cast<float>(q.z()),
        static_cast<float>(q.w()));
}

// Um estado com as juntas em `fraction` dos limites (giradas, longe do neutro).
ArticulationState3D randomState(const RagdollProfile3D& profile, const ArticulationModel3D& model, std::mt19937& rng,
    double fraction, double jointSpeed, double rootSpeed) {
    std::uniform_real_distribution<double> unit(-1.0, 1.0);
    ArticulationState3D state;
    state.rootPosition = { 0.0, 0.0, 2.0 };
    state.rootOrientation = Eigen::Quaterniond(Eigen::AngleAxisd(unit(rng), Eigen::Vector3d(unit(rng), unit(rng), 1.0).normalized()));
    state.jointCoordinates.assign(model.links.size(), Eigen::Vector3d::Zero());
    state.velocity = Eigen::VectorXd::Zero(model.dofCount);
    for (int i = 0; i < 3; ++i) {
        state.velocity[i] = rootSpeed * unit(rng);
        state.velocity[3 + i] = rootSpeed * unit(rng);
    }
    for (std::size_t i = 1; i < model.links.size(); ++i) {
        for (int a = 0; a < 3; ++a) {
            const RagdollAxisDefinition3D& axis = profile.links[i].inboundJoint.axes[static_cast<std::size_t>(a)];
            if (!axis.enabled) continue;
            const double middle = 0.5 * (axis.minimumRadians + axis.maximumRadians);
            const double half = 0.5 * (axis.maximumRadians - axis.minimumRadians);
            state.jointCoordinates[i][a] = middle + fraction * half * unit(rng);
            state.velocity[model.links[i].dof[static_cast<std::size_t>(a)]] = jointSpeed * unit(rng);
        }
    }
    return state;
}

std::vector<PhysicsBodyState3D> linkStates(const ArticulationModel3D& model, const ArticulationKinematics3D& k) {
    std::vector<PhysicsBodyState3D> links(model.links.size());
    for (std::size_t i = 0; i < links.size(); ++i) {
        const Eigen::Vector3d r = k.centerOfMass[i] - k.origin[i];
        links[i].position = toVec3(k.origin[i]);
        links[i].orientation = toQuaternion(k.rotation[i]);
        links[i].linearVelocity = toVec3(k.originVelocity[i] + k.angularVelocity[i].cross(r));
        links[i].angularVelocity = toVec3(k.angularVelocity[i]);
    }
    return links;
}

void modelOnlyChecks(const RagdollProfile3D& profile, const ArticulationModel3D& model, std::mt19937& rng) {
    double worstJacobian = 0.0, worstBias = 0.0, worstEnergy = 0.0, worstMomentum = 0.0, worstSymmetry = 0.0;
    for (int trial = 0; trial < 20; ++trial) {
        const ArticulationState3D state = randomState(profile, model, rng, 0.85, 3.0, 1.0);
        const ArticulationKinematics3D k = computeArticulationKinematics3D(model, state);
        const ArticulationDynamics3D d = computeArticulationDynamics3D(model, k);
        const double h = 1e-6;
        const ArticulationKinematics3D plus = computeArticulationKinematics3D(model,
            integrateArticulationState3D(model, state, state.velocity, h));
        const ArticulationKinematics3D minus = computeArticulationKinematics3D(model,
            integrateArticulationState3D(model, state, state.velocity, -h));
        for (std::size_t i = 0; i < model.links.size(); ++i) {
            // Um ponto preso ao link, longe da origem.
            const Eigen::Vector3d local(0.07, -0.03, 0.05);
            const Eigen::Vector3d point = k.origin[i] + k.rotation[i] * local;
            const Eigen::Vector3d pointPlus = plus.origin[i] + plus.rotation[i] * local;
            const Eigen::Vector3d pointMinus = minus.origin[i] + minus.rotation[i] * local;
            const Eigen::Vector3d numeric = (pointPlus - pointMinus) / (2.0 * h);
            const Eigen::Vector3d analytic = articulationPointJacobian3D(k, static_cast<int>(i), point) * state.velocity;
            worstJacobian = std::max(worstJacobian, (numeric - analytic).norm() / std::max(1.0, analytic.norm()));
            // A aceleracao de bias: d/dt (J v) com v fixo.
            const Eigen::Vector3d velocityPlus = articulationPointJacobian3D(plus, static_cast<int>(i), pointPlus) * state.velocity;
            const Eigen::Vector3d velocityMinus = articulationPointJacobian3D(minus, static_cast<int>(i), pointMinus) * state.velocity;
            const Eigen::Vector3d numericBias = (velocityPlus - velocityMinus) / (2.0 * h);
            const Eigen::Vector3d analyticBias = articulationPointBias3D(k, static_cast<int>(i), point);
            worstBias = std::max(worstBias, (numericBias - analyticBias).norm() / std::max(1.0, analyticBias.norm()));
        }
        const double energy = articulationKineticEnergy3D(model, k);
        const double quadratic = 0.5 * state.velocity.dot(d.massMatrix * state.velocity);
        worstEnergy = std::max(worstEnergy, std::abs(energy - quadratic) / std::max(1e-9, energy));
        worstSymmetry = std::max(worstSymmetry, (d.massMatrix - d.massMatrix.transpose()).cwiseAbs().maxCoeff());
        // Momento linear direto.
        Eigen::Vector3d linear = Eigen::Vector3d::Zero();
        for (std::size_t i = 0; i < model.links.size(); ++i) {
            const Eigen::Vector3d r = k.centerOfMass[i] - k.origin[i];
            linear += model.links[i].mass * (k.originVelocity[i] + k.angularVelocity[i].cross(r));
        }
        const Eigen::Matrix<double, 6, 1> momentum = d.centroidalMatrix * state.velocity;
        worstMomentum = std::max(worstMomentum, (momentum.head<3>() - linear).norm() / std::max(1.0, linear.norm()));
        const Eigen::LLT<Eigen::MatrixXd> cholesky(d.massMatrix);
        check(cholesky.info() == Eigen::Success, "M positiva definida");
    }
    std::printf("modelo: Jacobiano %.2e, bias %.2e, energia %.2e, simetria %.2e, momento %.2e\n", worstJacobian,
        worstBias, worstEnergy, worstSymmetry, worstMomentum);
    check(worstJacobian < 1e-4, "Jacobianos = diferencas finitas (juntas giradas)");
    check(worstBias < 1e-3, "aceleracao de bias = diferencas finitas");
    check(worstEnergy < 1e-9, "energia cinetica = v'Mv/2");
    check(worstSymmetry < 1e-9, "M simetrica");
    check(worstMomentum < 1e-9, "momento linear centroidal");
}

void joltChecks(const RagdollProfile3D& profile, std::mt19937& rng, bool gravity) {
    PhysicsEngine3D engine;
    MaterialLibrary materials;
    PhysicsSceneSettings3D settings;
    settings.gravity = gravity ? Vec3 { 0.0f, 0.0f, -9.81f } : Vec3 {};
    settings.workerThreadCount = 1;
    auto scene = engine.createScene(settings, materials);
    RagdollSpawnDefinition3D spawn;
    spawn.entityId = 1;
    spawn.pelvisPosition = { 0.0f, 0.0f, 2.0f };
    spawn.active = true;
    const RagdollHandle3D ragdoll = scene->createRagdoll(profile, spawn);
    ArticulationModel3D model = buildArticulationModel3D(profile, scene->ragdollEffectiveMass(ragdoll));
    model.gravity = gravity ? Eigen::Vector3d(0.0, 0.0, -9.81) : Eigen::Vector3d::Zero();
    std::uniform_real_distribution<double> unit(-1.0, 1.0);
    const double dt = std::getenv("ARTDT") ? std::atof(std::getenv("ARTDT")) : 1.0 / 120.0;
    double worstConversion = 0.0, worstAcceleration = 0.0, sumAcceleration = 0.0, sumRms = 0.0;
    const int trials = 40;
    const char* diagnostic = std::getenv("ARTDIAG");
    for (int trial = 0; trial < trials; ++trial) {
        const double fraction = std::getenv("ARTFRAC") ? std::atof(std::getenv("ARTFRAC")) : 0.35;
        ArticulationState3D state = randomState(profile, model, rng, fraction, 1.0, 0.5);
        if (diagnostic && std::string(diagnostic).find('z') != std::string::npos) state.velocity.setZero();
        if (diagnostic && std::string(diagnostic).find('n') != std::string::npos)
            for (auto& q : state.jointCoordinates) q.setZero();
        const ArticulationKinematics3D k = computeArticulationKinematics3D(model, state);
        const std::vector<PhysicsBodyState3D> links = linkStates(model, k);
        scene->resetRagdollState(ragdoll, links);
        // A conversao do estado publicado de volta ao generalizado.
        const ArticulationState3D read = articulationStateFromRagdoll3D(model, scene->ragdollState(ragdoll));
        worstConversion = std::max(worstConversion, (read.velocity - state.velocity).cwiseAbs().maxCoeff());
        // Torque explicito (A): k = d = 0, ff dentro do teto, sem a gravidade do backend.
        std::vector<RagdollDriveTarget3D> targets;
        std::vector<Eigen::Vector3d> feedforward(model.links.size(), Eigen::Vector3d::Zero());
        for (std::size_t i = 1; i < model.links.size(); ++i) {
            for (int a = 0; a < 3; ++a) {
                const RagdollAxisDefinition3D& axis = profile.links[i].inboundJoint.axes[static_cast<std::size_t>(a)];
                if (!axis.enabled) continue;
                RagdollDriveTarget3D target;
                target.linkIndex = static_cast<std::uint32_t>(i);
                target.axis = static_cast<RagdollAxis3D>(a);
                target.positionRadians = static_cast<float>(read.jointCoordinates[i][a]);
                target.stiffnessScale = 0.0f;
                target.dampingScale = 0.0f;
                target.maximumTorqueScale = 1.0f;
                target.gravityCompensationScale = 0.0f;
                double torque = 0.15 * axis.maximumTorque * unit(rng);
                if (diagnostic && std::string(diagnostic).find('b') != std::string::npos) torque = 0.0;
                if (diagnostic && std::string(diagnostic).find('1') != std::string::npos
                    && profile.links[i].id != "LeftForearm") torque = 0.0;
                target.feedforwardTorqueNewtonMeters = static_cast<float>(torque);
                feedforward[i][a] = static_cast<float>(torque);
                targets.push_back(target);
            }
        }
        const ArticulationKinematics3D kr = computeArticulationKinematics3D(model, read);
        const ArticulationDynamics3D d = computeArticulationDynamics3D(model, kr);
        // O Jolt corta a velocidade angular de cada link em 24 rad/s: o ensaio
        // fica no regime de uso (variacao por tick bem abaixo do teto).
        double spin = 0.0;
        {
            const Eigen::VectorXd trial = d.massMatrix.ldlt().solve(articulationGeneralizedFromFeedforward3D(model, read, feedforward) - d.bias);
            for (std::size_t i = 0; i < model.links.size(); ++i)
                spin = std::max(spin, (kr.angularJacobian[i] * trial).norm() * dt);
        }
        const double scale = spin > 4.0 ? 4.0 / spin : 1.0;
        for (std::size_t i = 0, t = 0; i < model.links.size(); ++i) {
            feedforward[i] *= scale;
            for (int a = 0; a < 3; ++a) {
                if (!profile.links[i].inboundJoint.axes[static_cast<std::size_t>(a)].enabled) continue;
                targets[t++].feedforwardTorqueNewtonMeters = static_cast<float>(feedforward[i][a]);
            }
        }
        scene->setRagdollActiveDriveTargets(ragdoll, targets, false);
        const Eigen::VectorXd force = articulationGeneralizedFromFeedforward3D(model, read, feedforward);
        const Eigen::VectorXd predicted = d.massMatrix.ldlt().solve(force - d.bias);
        scene->simulate(static_cast<float>(dt));
        const ArticulationState3D after = articulationStateFromRagdoll3D(model, scene->ragdollState(ragdoll));
        const Eigen::VectorXd measured = (after.velocity - read.velocity) / dt;
        if (diagnostic && std::string(diagnostic).find('c') != std::string::npos) {
            const RagdollState3D raw = scene->ragdollState(ragdoll);
            for (std::size_t i = 1; i < raw.joints.size(); ++i) {
                const auto& c = raw.joints[i].constraintTorqueNewtonMeters;
                const double m = std::max({ std::abs(c[0]), std::abs(c[1]), std::abs(c[2]) });
                if (m > 1.0) std::printf("  trial %d %s: restricao %.1f N.m (q %.2f %.2f %.2f)\n", trial,
                    profile.links[i].id.c_str(), m, read.jointCoordinates[i][0], read.jointCoordinates[i][1], read.jointCoordinates[i][2]);
            }
        }
        const Eigen::VectorXd error = measured - predicted;
        // A forca generalizada que o modelo nao explica, em fracao do teto de
        // cada eixo (a raiz: em mg e mg*1 m). Escala fisica com piso, nunca
        // dividida por uma aceleracao quase zero.
        const Eigen::VectorXd unexplained = d.massMatrix * error;
        double normalized = 0.0, squared = 0.0;
        int axes = 0;
        for (int j = 0; j < model.dofCount; ++j) {
            double scale = model.totalMass * 9.81;
            for (std::size_t i = 1; i < model.links.size(); ++i)
                for (int a = 0; a < 3; ++a)
                    if (model.links[i].dof[static_cast<std::size_t>(a)] == j)
                        scale = profile.links[i].inboundJoint.axes[static_cast<std::size_t>(a)].maximumTorque;
            const double ratio = std::abs(unexplained[j]) / scale;
            normalized = std::max(normalized, ratio);
            squared += ratio * ratio;
            ++axes;
        }
        sumRms += std::sqrt(squared / axes);
        worstAcceleration = std::max(worstAcceleration, normalized);
        double limitTorque = 0.0;
        {
            const RagdollState3D raw = scene->ragdollState(ragdoll);
            for (std::size_t i = 1; i < raw.joints.size(); ++i)
                for (int a = 0; a < 3; ++a)
                    if (profile.links[i].inboundJoint.axes[static_cast<std::size_t>(a)].enabled)
                        limitTorque = std::max(limitTorque, double(std::abs(raw.joints[i].constraintTorqueNewtonMeters[static_cast<std::size_t>(a)])));
        }
        if (diagnostic && std::string(diagnostic).find('v') != std::string::npos && trial == 24 && !gravity) {
            const Eigen::VectorXd jump = error * dt;
            for (std::size_t i = 0; i < model.links.size(); ++i)
                for (int a = 0; a < 3; ++a) {
                    const int j = i == 0 ? a + 3 : model.links[i].dof[static_cast<std::size_t>(a)];
                    if (j >= 0 && std::abs(jump[j]) > 2e-3)
                        std::printf("   %-14s eixo %d q(%.2f %.2f %.2f) qdot %.2f: dv nao explicado %.4f\n", profile.links[i].id.c_str(), a,
                            read.jointCoordinates[i][0], read.jointCoordinates[i][1], read.jointCoordinates[i][2], read.velocity[j], jump[j]);
                }
        }
        if (diagnostic && std::string(diagnostic).find('s') != std::string::npos)
            std::printf(" trial %2d erro %.3f limite %.1f N.m |v| %.2f\n", trial, normalized, limitTorque, read.velocity.norm());
        if (diagnostic && std::string(diagnostic).find('d') != std::string::npos && normalized > 0.15) {
            std::printf(" trial %d erro %.3f\n", trial, normalized);
            for (std::size_t i = 0; i < model.links.size(); ++i)
                for (int a = 0; a < 3; ++a) {
                    const int j = i == 0 ? a + 3 : model.links[i].dof[static_cast<std::size_t>(a)];
                    if (j < 0) continue;
                    if (std::abs(error[j]) > 0.1 * std::abs(predicted[j]) + 2.0)
                        std::printf("   %-14s eixo %d (q %.2f %.2f %.2f): medido %8.2f previsto %8.2f\n", profile.links[i].id.c_str(), a,
                            read.jointCoordinates[i][0], read.jointCoordinates[i][1], read.jointCoordinates[i][2], measured[j], predicted[j]);
                }
        }
        if (diagnostic && trial < 2 && std::string(diagnostic).find('p') != std::string::npos) {
            for (int j = 0; j < model.dofCount; ++j)
                if (std::abs(measured[j]) + std::abs(predicted[j]) > 0.05)
                    std::printf("  dof %2d: medido %9.3f previsto %9.3f\n", j, measured[j], predicted[j]);
        }
        sumAcceleration += normalized;
    }
    std::printf("jolt (%s): conversao %.2e rad/s; forca nao explicada (fracao do teto): RMS medio %.4f, pior eixo medio %.4f, pior eixo %.4f\n",
        gravity ? "com gravidade" : "sem gravidade", worstConversion, sumRms / trials, sumAcceleration / trials, worstAcceleration);
    check(worstConversion < 1e-3, "o estado do Jolt volta ao generalizado");
    check(sumRms / trials < 0.02, std::string("forca nao explicada RMS <= 2% do teto (") + (gravity ? "com" : "sem") + " gravidade)");
    check(sumAcceleration / trials < 0.05, std::string("pior eixo medio <= 5% do teto (") + (gravity ? "com" : "sem") + " gravidade)");
}

} // namespace

int main() {
    const RagdollProfile3D profile = loadRagdollProfile3D(
        std::string(MATTERENGINE_TEST_ASSETS_DIR) + "/characters/football_player/FootballPlayerV1.ragdoll.json");
    std::mt19937 rng(20261026u);
    {
        PhysicsEngine3D engine;
        MaterialLibrary materials;
        PhysicsSceneSettings3D settings;
        settings.workerThreadCount = 1;
        auto scene = engine.createScene(settings, materials);
        RagdollSpawnDefinition3D spawn;
        spawn.entityId = 1;
        const RagdollHandle3D ragdoll = scene->createRagdoll(profile, spawn);
        const ArticulationModel3D model = buildArticulationModel3D(profile, scene->ragdollEffectiveMass(ragdoll));
        std::printf("modelo: %zu links, %d graus de liberdade, %.2f kg\n", model.links.size(), model.dofCount, model.totalMass);
        check(model.dofCount == 45, "45 graus de liberdade (6 + 39)");
        {
            // O mapa K do motor bate com o conversor (nota 33).
            const ArticulationState3D state = randomState(profile, model, rng, 0.35, 1.0, 0.5);
            const Eigen::MatrixXd map = articulationMotorTorqueMap3D(model, state);
            Eigen::VectorXd tau = Eigen::VectorXd::Zero(model.dofCount);
            for (int j = 6; j < model.dofCount; ++j) tau[j] = std::sin(1.7 * j) * 50.0;
            const std::vector<Eigen::Vector3d> feedforward = articulationFeedforwardFromGeneralized3D(model, state, tau);
            const Eigen::VectorXd mapped = map * tau.tail(model.dofCount - 6);
            double worst = 0.0;
            for (std::size_t i = 1; i < model.links.size(); ++i)
                for (int a = 0; a < 3; ++a)
                    if (model.links[i].dof[static_cast<std::size_t>(a)] >= 0)
                        worst = std::max(worst, std::abs(mapped[model.links[i].dof[static_cast<std::size_t>(a)] - 6] - feedforward[i][a]));
            check(worst < 1e-9, "o mapa do motor bate com o conversor");
        }
        modelOnlyChecks(profile, model, rng);
        // O custo de cada peca (informativo; o orcamento e medido na bancada).
        const ArticulationState3D timed = randomState(profile, model, rng, 0.35, 1.0, 0.5);
        const auto micros = [](auto&& work) {
            const auto start = std::chrono::steady_clock::now();
            for (int i = 0; i < 2000; ++i) work();
            return std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - start).count() / 2000.0;
        };
        ArticulationKinematics3D k = computeArticulationKinematics3D(model, timed);
        ArticulationDynamics3D d = computeArticulationDynamics3D(model, k);
        Eigen::VectorXd solved;
        const double kinematicsMicros = micros([&] { k = computeArticulationKinematics3D(model, timed); });
        const double dynamicsMicros = micros([&] { d = computeArticulationDynamics3D(model, k); });
        const double solveMicros = micros([&] { solved = d.massMatrix.ldlt().solve(d.bias); });
        std::printf("custo: cinematica %.1f us, dinamica %.1f us, LDLT 45x45 %.1f us\n", kinematicsMicros, dynamicsMicros, solveMicros);
    }
    joltChecks(profile, rng, false);
    joltChecks(profile, rng, true);
    std::printf(failures ? "articulacao: %d falha(s)\n" : "articulacao: ok\n", failures);
    return failures ? 1 : 0;
}
