// A bancada de atuacao (G0 da locomocao fisica nova, plano 6.7): o torque
// muscular pedido por um controle de torque computado de base flutuante, com
// o modelo dinamico, entregue ao Jolt pelos atuadores A (torque explicito),
// A' (com amortecimento implicito) e B (impedancia baixa). Corpo flutuando sem
// gravidade (a dinamica interna e a mesma em queda livre), juntas seguindo
// senoides em volta da pose media. Mede o erro de rastreio, a estabilidade, o
// torque entregue contra o pedido, o orcamento por tick e o custo.
//
// uso: MatterActuationBench [--segundos 4] [--isolada <link>] [--csv arquivo] [--corpo cm,kg]
#include "Engine/BodyControl/Execution/Actuation3D.hpp"
#include "Engine/Materials/MaterialLibrary.hpp"
#include "Engine/Physics/Articulation/ArticulationDynamics3D.hpp"
#include "Engine/Physics/PhysicsEngine3D.hpp"

#include <Eigen/Dense>

#include <chrono>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <random>
#include <string>
#include <vector>

using namespace MatterEngine;

namespace {

constexpr double Dt = 1.0 / 120.0;
constexpr double Pi = 3.14159265358979323846;

Vec3 toVec3(const Eigen::Vector3d& v) {
    return { static_cast<float>(v.x()), static_cast<float>(v.y()), static_cast<float>(v.z()) };
}

Quaternion toQuaternion(const Eigen::Matrix3d& rotation) {
    const Eigen::Quaterniond q(rotation);
    return Quaternion(static_cast<float>(q.x()), static_cast<float>(q.y()), static_cast<float>(q.z()),
        static_cast<float>(q.w()));
}

struct JointWave {
    double middle = 0.0, amplitude = 0.0, frequency = 0.0, phase = 0.0;
};

struct Trajectory {
    std::vector<std::vector<JointWave>> waves; // [link][axis]
    void sample(double t, std::vector<Eigen::Vector3d>& q, std::vector<Eigen::Vector3d>& qd, std::vector<Eigen::Vector3d>& qdd) const {
        for (std::size_t i = 0; i < waves.size(); ++i) {
            for (int a = 0; a < 3; ++a) {
                const JointWave& w = waves[i][static_cast<std::size_t>(a)];
                const double angle = 2.0 * Pi * w.frequency * t + w.phase;
                const double omega = 2.0 * Pi * w.frequency;
                q[i][a] = w.middle + w.amplitude * std::sin(angle);
                qd[i][a] = w.amplitude * omega * std::cos(angle);
                qdd[i][a] = -w.amplitude * omega * omega * std::sin(angle);
            }
        }
    }
};

Trajectory makeTrajectory(const RagdollProfile3D& profile, const std::string& isolated, double amplitudeFraction, unsigned seed) {
    std::mt19937 rng(seed);
    std::uniform_real_distribution<double> unit(0.0, 1.0);
    Trajectory trajectory;
    trajectory.waves.assign(profile.links.size(), std::vector<JointWave>(3));
    for (std::size_t i = 1; i < profile.links.size(); ++i) {
        for (int a = 0; a < 3; ++a) {
            const RagdollAxisDefinition3D& axis = profile.links[i].inboundJoint.axes[static_cast<std::size_t>(a)];
            if (!axis.enabled) continue;
            JointWave& w = trajectory.waves[i][static_cast<std::size_t>(a)];
            w.middle = 0.5 * (axis.minimumRadians + axis.maximumRadians);
            const bool moving = isolated.empty() || profile.links[i].id == isolated;
            w.amplitude = moving ? amplitudeFraction * 0.5 * (axis.maximumRadians - axis.minimumRadians) : 0.0;
            w.frequency = 0.6 + 1.2 * unit(rng);
            w.phase = 2.0 * Pi * unit(rng);
        }
    }
    return trajectory;
}

struct Variant {
    std::string name;
    ActuationSettings3D settings;
};

struct Result {
    double rmsError = 0.0, maxError = 0.0, lateRms = 0.0, earlyRms = 0.0;
    double deliveredRms = 0.0, worstExcess = 0.0, controlMicros = 0.0;
    bool exploded = false;
    std::string worstJoint;
};

Result run(const RagdollProfile3D& profile, const Trajectory& trajectory, const Variant& variant, double omega,
    double seconds, std::ofstream* csv) {
    PhysicsEngine3D engine;
    MaterialLibrary materials;
    PhysicsSceneSettings3D sceneSettings;
    sceneSettings.gravity = {};
    sceneSettings.workerThreadCount = 1;
    auto scene = engine.createScene(sceneSettings, materials);
    RagdollSpawnDefinition3D spawn;
    spawn.entityId = 1;
    spawn.pelvisPosition = { 0.0f, 0.0f, 2.0f };
    spawn.active = true;
    const RagdollHandle3D ragdoll = scene->createRagdoll(profile, spawn);
    ArticulationModel3D model = buildArticulationModel3D(profile, scene->ragdollEffectiveMass(ragdoll));
    model.gravity.setZero();
    const std::size_t links = model.links.size();
    const int n = model.dofCount;

    std::vector<Eigen::Vector3d> q(links, Eigen::Vector3d::Zero()), qd = q, qdd = q;
    trajectory.sample(0.0, q, qd, qdd);
    ArticulationState3D start;
    start.rootPosition = { 0.0, 0.0, 2.0 };
    start.jointCoordinates = q;
    start.velocity = Eigen::VectorXd::Zero(n);
    for (std::size_t i = 1; i < links; ++i)
        for (int a = 0; a < 3; ++a)
            if (model.links[i].dof[static_cast<std::size_t>(a)] >= 0) start.velocity[model.links[i].dof[static_cast<std::size_t>(a)]] = qd[i][a];
    {
        const ArticulationKinematics3D k = computeArticulationKinematics3D(model, start);
        std::vector<PhysicsBodyState3D> states(links);
        for (std::size_t i = 0; i < links; ++i) {
            const Eigen::Vector3d r = k.centerOfMass[i] - k.origin[i];
            states[i] = { toVec3(k.origin[i]), toQuaternion(k.rotation[i]),
                toVec3(k.originVelocity[i] + k.angularVelocity[i].cross(r)), toVec3(k.angularVelocity[i]) };
        }
        scene->resetRagdollState(ragdoll, states);
    }

    const double kp = omega * omega, kd = 2.0 * omega;
    const int steps = static_cast<int>(seconds / Dt);
    Result result;
    double squared = 0.0, early = 0.0, late = 0.0, delivered = 0.0;
    int samples = 0, earlySamples = 0, lateSamples = 0, deliveredSamples = 0;
    Eigen::VectorXd previousRequest = Eigen::VectorXd::Zero(n);
    std::vector<double> torqueScale(static_cast<std::size_t>(n), 1.0);
    for (std::size_t i = 1; i < links; ++i)
        for (int a = 0; a < 3; ++a)
            if (model.links[i].dof[static_cast<std::size_t>(a)] >= 0)
                torqueScale[static_cast<std::size_t>(model.links[i].dof[static_cast<std::size_t>(a)])] =
                    profile.links[i].inboundJoint.axes[static_cast<std::size_t>(a)].maximumTorque;
    for (int step = 0; step < steps; ++step) {
        const double t = step * Dt;
        const RagdollState3D& raw = *scene->ragdollStateView(ragdoll);
        // O orcamento do passo anterior (motor + ff contra o teto, por eixo).
        if (step > 0) result.worstExcess = std::max(result.worstExcess, actuationDelivery3D(profile, raw).worstBudgetExcessNewtonMeters);
        const auto clockStart = std::chrono::steady_clock::now();
        const ArticulationState3D state = articulationStateFromRagdoll3D(model, raw);
        if (!state.velocity.allFinite() || state.velocity.cwiseAbs().maxCoeff() > 200.0) {
            result.exploded = true;
            break;
        }
        trajectory.sample(t, q, qd, qdd);
        const ArticulationKinematics3D k = computeArticulationKinematics3D(model, state);
        const ArticulationDynamics3D d = computeArticulationDynamics3D(model, k);
        // Aceleracao desejada das juntas e a dinamica inversa de base flutuante.
        Eigen::VectorXd jointAcceleration = Eigen::VectorXd::Zero(n - 6);
        double errorSquared = 0.0;
        int errorCount = 0;
        for (std::size_t i = 1; i < links; ++i)
            for (int a = 0; a < 3; ++a) {
                const int dof = model.links[i].dof[static_cast<std::size_t>(a)];
                if (dof < 0) continue;
                const double e = q[i][a] - state.jointCoordinates[i][a];
                const double ed = qd[i][a] - state.velocity[dof];
                jointAcceleration[dof - 6] = qdd[i][a] + kp * e + kd * ed;
                errorSquared += e * e;
                ++errorCount;
                if (std::abs(e) > result.maxError) {
                    result.maxError = std::abs(e);
                    result.worstJoint = profile.links[i].id + " eixo " + std::to_string(a) + " t=" + std::to_string(t);
                }
            }
        const Eigen::MatrixXd rootBlock = d.massMatrix.topLeftCorner(6, 6);
        const Eigen::Matrix<double, 6, 1> rootAcceleration = -rootBlock.ldlt().solve(
            d.massMatrix.topRightCorner(6, n - 6) * jointAcceleration + d.bias.head<6>());
        Eigen::VectorXd acceleration(n);
        acceleration << rootAcceleration, jointAcceleration;
        Eigen::VectorXd request = d.massMatrix * acceleration + d.bias;
        request.head<6>().setZero();
        ActuationReference3D reference { q, qd };
        const Eigen::VectorXd inertia = d.massMatrix.diagonal();
        const std::vector<RagdollDriveTarget3D> targets =
            actuationTargets3D(profile, model, state, request, reference, inertia, variant.settings);
        result.controlMicros += std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - clockStart).count();
        // Entregue (motor + ff do passo anterior) contra o pedido do passo anterior, no generalizado.
        if (step > 0) {
            const ActuationDelivery3D delivery = actuationDelivery3D(profile, raw);
            const Eigen::VectorXd deliveredGeneralized = articulationGeneralizedFromFeedforward3D(model, state, delivery.muscleTorque);
            for (int j = 6; j < n; ++j) {
                const double ratio = (deliveredGeneralized[j] - previousRequest[j]) / torqueScale[static_cast<std::size_t>(j)];
                delivered += ratio * ratio;
                ++deliveredSamples;
            }
        }
        previousRequest = request;
        const double rms = std::sqrt(errorSquared / std::max(1, errorCount));
        squared += rms * rms;
        ++samples;
        if (t < 1.0) { early += rms * rms; ++earlySamples; }
        if (t >= seconds - 1.0) { late += rms * rms; ++lateSamples; }
        if (csv) *csv << variant.name << ',' << omega << ',' << t << ',' << rms << '\n';
        scene->setRagdollActiveDriveTargets(ragdoll, targets, false);
        scene->simulate(static_cast<float>(Dt));
    }
    result.rmsError = std::sqrt(squared / std::max(1, samples));
    result.earlyRms = std::sqrt(early / std::max(1, earlySamples));
    result.lateRms = std::sqrt(late / std::max(1, lateSamples));
    result.deliveredRms = std::sqrt(delivered / std::max(1, deliveredSamples));
    result.controlMicros /= std::max(1, samples);
    return result;
}

} // namespace

int main(int argc, char** argv) {
    double seconds = 4.0;
    std::string isolated, csvPath;
    double amplitude = 0.3;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        auto next = [&]() { return i + 1 < argc ? std::string(argv[++i]) : std::string(); };
        if (arg == "--segundos") seconds = std::stod(next());
        else if (arg == "--isolada") isolated = next();
        else if (arg == "--amplitude") amplitude = std::stod(next());
        else if (arg == "--csv") csvPath = next();
        else {
            std::fprintf(stderr, "uso: MatterActuationBench [--segundos 4] [--isolada <link>] [--amplitude 0.3] [--csv arquivo]\n");
            return 2;
        }
    }
    const RagdollProfile3D profile = loadRagdollProfile3D(
        std::string(MATTERENGINE_ASSETS_DIR) + "/characters/football_player/FootballPlayerV1.ragdoll.json");
    const Trajectory trajectory = makeTrajectory(profile, isolated, amplitude, 20261027u);
    std::ofstream csvFile;
    if (!csvPath.empty()) {
        csvFile.open(csvPath);
        csvFile << "atuador,omega,t,erro_rms\n";
    }
    std::vector<Variant> variants;
    variants.push_back({ "A (explicito)", { ActuationMode3D::ExplicitTorque } });
    { ActuationSettings3D s; s.mode = ActuationMode3D::ExplicitTorqueImplicitDamping; s.dampingPerInertia = 20.0; variants.push_back({ "A' (amort. 20/I)", s }); }
    { ActuationSettings3D s; s.mode = ActuationMode3D::ExplicitTorqueImplicitDamping; s.dampingPerInertia = 60.0; variants.push_back({ "A' (amort. 60/I)", s }); }
    { ActuationSettings3D s; s.mode = ActuationMode3D::LowImpedance; s.stiffnessPerInertia = 150.0; s.dampingPerInertia = 20.0; variants.push_back({ "B (k 150/I, d 20/I)", s }); }
    std::printf("# Bancada de atuacao (%s, %.1f s, amplitude %.0f%% da faixa, sem gravidade, corpo flutuando)\n\n",
        isolated.empty() ? "todas as juntas" : ("so " + isolated).c_str(), seconds, amplitude * 100.0);
    std::printf("| atuador | omega (rad/s) | erro RMS (graus) | 1o s | ultimo s | erro max (graus) | entregue - pedido (RMS, fracao do teto) | estouro do teto (N.m) | controle (us/tick) |\n");
    std::printf("|---|---|---|---|---|---|---|---|---|\n");
    for (const Variant& variant : variants) {
        for (const double omega : { 10.0, 20.0, 40.0, 80.0 }) {
            const Result r = run(profile, trajectory, variant, omega, seconds, csvPath.empty() ? nullptr : &csvFile);
            const double deg = 180.0 / Pi;
            if (r.exploded) {
                std::printf("| %s | %.0f | EXPLODIU | | | | | | |\n", variant.name.c_str(), omega);
                continue;
            }
            std::printf("| %s | %.0f | %.3f | %.3f | %.3f | %.2f (%s) | %.4f | %.3f | %.0f |\n", variant.name.c_str(), omega,
                r.rmsError * deg, r.earlyRms * deg, r.lateRms * deg, r.maxError * deg, r.worstJoint.c_str(), r.deliveredRms, r.worstExcess, r.controlMicros);
        }
    }
    return 0;
}
