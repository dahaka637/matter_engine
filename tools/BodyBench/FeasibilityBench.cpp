// O analisador de viabilidade da referencia (G1 da locomocao fisica nova, plano
// 6.3/9): os quadros da locomocao padrao (dump da MatterLocomotionProbe, a cada
// tick) viram estados generalizados; as velocidades e aceleracoes saem por
// diferencas centrais; o QP de corpo inteiro, offline, tenta reproduzir
// exatamente essas aceleracoes com forcas so nos pes que tocam o chao (atrito e
// torque livres, para medir o que se exige). Publica, por cenario:
//   - o residuo da aceleracao da raiz (o que nenhuma forca de apoio produz);
//   - o atrito necessario (|Ft|/Fn) contra o do gramado com a sola (0,75);
//   - o torque necessario contra o teto do perfil, por regiao;
//   - a maior velocidade angular de link contra o teto do backend (24 rad/s).
//
// uso: MatterFeasibilityBench <dump> [<dump> ...]
#include "Engine/BodyControl/Execution/WholeBodyQP3D.hpp"
#include "Engine/Materials/MaterialLibrary.hpp"
#include "Engine/Physics/PhysicsEngine3D.hpp"
#include "Engine/Physics/PhysicsScene3D.hpp"

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

using namespace MatterEngine;

namespace {

constexpr double Dt = 1.0 / 120.0;

struct Frame {
    double t = 0.0;
    std::map<std::string, std::pair<Eigen::Vector3d, Eigen::Quaterniond>> colliders;
};

std::vector<std::pair<std::string, std::vector<Frame>>> readDump(const std::string& path) {
    std::vector<std::pair<std::string, std::vector<Frame>>> scenarios;
    std::ifstream file(path);
    std::string line;
    while (std::getline(file, line)) {
        std::istringstream in(line);
        std::string tag;
        in >> tag;
        if (tag == "S") {
            std::string name;
            std::getline(in, name);
            scenarios.push_back({ name.substr(name.find_first_not_of(' ')), {} });
        } else if (tag == "F" && !scenarios.empty()) {
            Frame frame;
            in >> frame.t;
            scenarios.back().second.push_back(frame);
        } else if ((tag == "C" || tag == "B") && !scenarios.empty() && !scenarios.back().second.empty()) {
            std::string id;
            double x, y, z, qx, qy, qz, qw;
            in >> id >> x >> y >> z >> qx >> qy >> qz >> qw;
            scenarios.back().second.back().colliders[id] = { Eigen::Vector3d(x, y, z), Eigen::Quaterniond(qw, qx, qy, qz).normalized() };
        }
    }
    return scenarios;
}

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

std::vector<Eigen::Vector3d> soleCorners(const RagdollProfile3D& profile, std::size_t link) {
    const RagdollCapsuleDefinition3D& collider = profile.links[link].collider;
    std::vector<Eigen::Vector3d> points;
    for (const Vec3& p : ragdollColliderSolePoints3D(collider)) {
        const Vec3 local = collider.localPosition + collider.localOrientation.rotate(p);
        points.emplace_back(local.x, local.y, local.z);
    }
    Eigen::Vector3d minimum = points.front(), maximum = points.front();
    for (const Eigen::Vector3d& p : points) {
        minimum = minimum.cwiseMin(p);
        maximum = maximum.cwiseMax(p);
    }
    const Eigen::Vector3d extent = maximum - minimum;
    int thin = 0;
    for (int a = 1; a < 3; ++a) if (extent[a] < extent[thin]) thin = a;
    const int first = (thin + 1) % 3, second = (thin + 2) % 3;
    std::vector<Eigen::Vector3d> corners;
    for (int i = 0; i < 4; ++i) {
        Eigen::Vector3d corner = 0.5 * (minimum + maximum);
        corner[thin] = std::abs(minimum[thin]) > std::abs(maximum[thin]) ? minimum[thin] : maximum[thin];
        corner[first] = (i & 1) ? maximum[first] : minimum[first];
        corner[second] = (i & 2) ? maximum[second] : minimum[second];
        corners.push_back(corner);
    }
    return corners;
}

} // namespace

int main(int argc, char** argv) {
    if (argc < 2) {
        std::fprintf(stderr, "uso: MatterFeasibilityBench <dump> [<dump> ...]\n");
        return 2;
    }
    const RagdollProfile3D profile = loadRagdollProfile3D(
        std::string(MATTERENGINE_ASSETS_DIR) + "/characters/football_player/FootballPlayerV1.ragdoll.json");
    PhysicsEngine3D engine;
    MaterialLibrary materials;
    PhysicsSceneSettings3D sceneSettings;
    sceneSettings.workerThreadCount = 1;
    auto scene = engine.createScene(sceneSettings, materials);
    RagdollSpawnDefinition3D spawn;
    spawn.entityId = 1;
    const ArticulationModel3D model = buildArticulationModel3D(profile, scene->ragdollEffectiveMass(scene->createRagdoll(profile, spawn)));
    std::size_t feet[2] {};
    std::vector<Eigen::Vector3d> corners[2];
    for (std::size_t i = 0; i < profile.links.size(); ++i) {
        if (profile.links[i].id == "LeftFoot") feet[0] = i;
        if (profile.links[i].id == "RightFoot") feet[1] = i;
    }
    for (int f = 0; f < 2; ++f) corners[f] = soleCorners(profile, feet[f]);
    Eigen::VectorXd torqueLimit = Eigen::VectorXd::Zero(model.dofCount - 6), hugeLimit = Eigen::VectorXd::Constant(model.dofCount - 6, 1e6);
    std::vector<std::string> dofRegion(static_cast<std::size_t>(model.dofCount));
    for (std::size_t i = 1; i < model.links.size(); ++i)
        for (int a = 0; a < 3; ++a) {
            const int dof = model.links[i].dof[static_cast<std::size_t>(a)];
            if (dof < 0) continue;
            torqueLimit[dof - 6] = profile.links[i].inboundJoint.axes[static_cast<std::size_t>(a)].maximumTorque;
            const std::string& id = profile.links[i].id;
            dofRegion[static_cast<std::size_t>(dof)] = id.find("Thigh") != std::string::npos ? "quadril"
                : id.find("Shin") != std::string::npos ? "joelho" : id.find("Foot") != std::string::npos ? "tornozelo"
                : (id.find("Arm") != std::string::npos || id.find("Hand") != std::string::npos) ? "braco" : "tronco";
        }
    std::printf("| cenario | quadros | residuo raiz linear p95/max (m/s2) | angular p95/max (rad/s2) | atrito necessario p95/max | torque/teto max (regiao) | omega link max (rad/s) |\n");
    std::printf("|---|---|---|---|---|---|---|\n");
    for (int arg = 1; arg < argc; ++arg) {
        for (const auto& [name, frames] : readDump(argv[arg])) {
            // Os estados generalizados (posicao) de cada quadro.
            std::vector<ArticulationState3D> states;
            for (const Frame& frame : frames) {
                ArticulationState3D state;
                state.jointCoordinates.assign(model.links.size(), Eigen::Vector3d::Zero());
                state.velocity = Eigen::VectorXd::Zero(model.dofCount);
                std::vector<Eigen::Matrix3d> rotation(model.links.size());
                std::vector<Eigen::Vector3d> origin(model.links.size());
                bool complete = true;
                for (std::size_t i = 0; i < model.links.size(); ++i) {
                    const auto found = frame.colliders.find(profile.links[i].id);
                    if (found == frame.colliders.end()) { complete = false; break; }
                    const RagdollCapsuleDefinition3D& collider = profile.links[i].collider;
                    const Eigen::Quaterniond local(collider.localOrientation.w, collider.localOrientation.x, collider.localOrientation.y, collider.localOrientation.z);
                    rotation[i] = (found->second.second * local.conjugate()).toRotationMatrix();
                    origin[i] = found->second.first - rotation[i] * Eigen::Vector3d(collider.localPosition.x, collider.localPosition.y, collider.localPosition.z);
                }
                if (!complete) continue;
                state.rootPosition = origin[0];
                state.rootOrientation = Eigen::Quaterniond(rotation[0]);
                for (std::size_t i = 1; i < model.links.size(); ++i) {
                    const ArticulationLink3D& link = model.links[i];
                    const Eigen::Matrix3d relative = link.jointInParent.transpose() * rotation[static_cast<std::size_t>(link.parent)].transpose() * rotation[i] * link.jointInChild;
                    state.jointCoordinates[i] = articulationJointCoordinates3D(link, relative);
                }
                states.push_back(state);
            }
            // Velocidades e aceleracoes por diferencas centrais.
            const std::size_t count = states.size();
            std::vector<Eigen::VectorXd> velocity(count, Eigen::VectorXd::Zero(model.dofCount));
            for (std::size_t i = 1; i + 1 < count; ++i) {
                const ArticulationState3D& a = states[i - 1];
                const ArticulationState3D& b = states[i + 1];
                velocity[i].head<3>() = (b.rootPosition - a.rootPosition) / (2.0 * Dt);
                velocity[i].segment<3>(3) = rotationVector(b.rootOrientation.toRotationMatrix() * a.rootOrientation.toRotationMatrix().transpose()) / (2.0 * Dt);
                for (std::size_t l = 1; l < model.links.size(); ++l)
                    for (int ax = 0; ax < 3; ++ax) {
                        const int dof = model.links[l].dof[static_cast<std::size_t>(ax)];
                        if (dof >= 0) velocity[i][dof] = (b.jointCoordinates[l][ax] - a.jointCoordinates[l][ax]) / (2.0 * Dt);
                    }
            }
            std::vector<double> linearResidual, angularResidual, frictionNeeded, supportedResidual, flightResidual;
            double worstTorqueRatio = 0.0, worstOmega = 0.0;
            std::string worstRegion;
            WholeBodyQP3D qp;
            for (std::size_t i = 2; i + 2 < count; ++i) {
                ArticulationState3D state = states[i];
                state.velocity = velocity[i];
                const Eigen::VectorXd acceleration = (velocity[i + 1] - velocity[i - 1]) / (2.0 * Dt);
                const ArticulationKinematics3D k = computeArticulationKinematics3D(model, state);
                const ArticulationDynamics3D d = computeArticulationDynamics3D(model, k);
                for (std::size_t l = 0; l < model.links.size(); ++l) worstOmega = std::max(worstOmega, k.angularVelocity[l].norm());
                WholeBodyQPInput3D input;
                input.model = &model;
                input.kinematics = &k;
                input.dynamics = &d;
                input.state = &state;
                input.torqueLimit = hugeLimit;
                for (int f = 0; f < 2; ++f) {
                    double lowest = 1e9;
                    for (const Eigen::Vector3d& c : corners[f]) lowest = std::min(lowest, (k.origin[feet[f]] + k.rotation[feet[f]] * c).z());
                    if (lowest > 0.015) continue;
                    for (const Eigen::Vector3d& c : corners[f]) {
                        WholeBodyContactPoint3D point;
                        point.link = static_cast<int>(feet[f]);
                        point.pointWorld = k.origin[feet[f]] + k.rotation[feet[f]] * c;
                        point.friction = 10.0;
                        point.maximumNormalForce = 1e5;
                        input.contacts.push_back(point);
                    }
                }
                WholeBodyTask3D track;
                track.name = "referencia";
                track.matrix = Eigen::MatrixXd::Identity(model.dofCount, model.dofCount);
                track.target = acceleration;
                track.weight = 1.0;
                // A raiz pesa mais: e ela que diz se as forcas de apoio bastam.
                track.matrix.topRows(6) *= 10.0;
                track.target.head(6) *= 10.0;
                input.tasks.push_back(track);
                input.accelerationRegularization = 1e-6;
                const WholeBodyQPResult3D result = qp.solve(input);
                if (!result.usable()) continue;
                const Eigen::VectorXd residual = result.acceleration - acceleration;
                linearResidual.push_back(residual.head<3>().norm());
                angularResidual.push_back(residual.segment<3>(3).norm());
                // O atrito pela forca total de cada pe (a divisao entre os cantos e arbitraria).
                double friction = 0.0;
                for (int f = 0; f < 2; ++f) {
                    Eigen::Vector3d total = Eigen::Vector3d::Zero();
                    for (std::size_t c = 0; c < input.contacts.size(); ++c)
                        if (input.contacts[c].link == static_cast<int>(feet[f])) total += result.contactForces.segment<3>(3 * static_cast<int>(c));
                    if (total.z() > 50.0) friction = std::max(friction, std::hypot(total.x(), total.y()) / total.z());
                }
                if (!input.contacts.empty()) {
                    frictionNeeded.push_back(friction);
                    supportedResidual.push_back(residual.head<3>().norm());
                } else {
                    flightResidual.push_back(residual.head<3>().norm());
                }
                for (int j = 6; j < model.dofCount; ++j) {
                    const double ratio = std::abs(result.torque[j]) / torqueLimit[j - 6];
                    if (ratio > worstTorqueRatio) { worstTorqueRatio = ratio; worstRegion = dofRegion[static_cast<std::size_t>(j)]; }
                }
            }
            const auto p95 = [](std::vector<double> v) { if (v.empty()) return 0.0; std::sort(v.begin(), v.end()); return v[static_cast<std::size_t>(0.95 * double(v.size() - 1))]; };
            const auto maxOf = [](const std::vector<double>& v) { return v.empty() ? 0.0 : *std::max_element(v.begin(), v.end()); };
            std::printf("| %s | %zu | apoiado %.2f / %.2f; no ar (%zu) %.2f | %.1f / %.1f | %.2f / %.2f | %.2f (%s) | %.1f |\n", name.c_str(), count,
                p95(supportedResidual), maxOf(supportedResidual), flightResidual.size(), p95(flightResidual), p95(angularResidual), maxOf(angularResidual),
                p95(frictionNeeded), maxOf(frictionNeeded), worstTorqueRatio, worstRegion.c_str(), worstOmega);
        }
    }
    return 0;
}
