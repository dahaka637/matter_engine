// O QP em dois niveis (nota 27): a tarefa de prioridade 0 sai exata mesmo contra
// uma de prioridade 1 muito mais pesada; o que nao conflita embaixo continua atendido;
// num nivel so, o peso maior ganha.
#include "Engine/BodyControl/Execution/WholeBodyQP3D.hpp"
#include "Engine/Materials/MaterialLibrary.hpp"
#include "Engine/Physics/PhysicsEngine3D.hpp"

#include <Eigen/Dense>

#include <cmath>
#include <cstdio>
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

WholeBodyTask3D jointTask(int n, int dof, double acceleration, double weight, int priority) {
    WholeBodyTask3D task;
    task.name = "junta";
    task.matrix = Eigen::MatrixXd::Zero(1, n);
    task.matrix(0, dof) = 1.0;
    task.target = Eigen::VectorXd::Constant(1, acceleration);
    task.weight = weight;
    task.priority = priority;
    return task;
}
} // namespace

int main() {
    const RagdollProfile3D profile = loadRagdollProfile3D(
        std::string(MATTERENGINE_TEST_ASSETS_DIR) + "/characters/football_player/FootballPlayerV1.ragdoll.json");
    PhysicsEngine3D engine;
    MaterialLibrary materials;
    PhysicsSceneSettings3D settings;
    settings.workerThreadCount = 1;
    const std::unique_ptr<PhysicsScene3D> scene = engine.createScene(settings, materials);
    RagdollSpawnDefinition3D spawn;
    spawn.entityId = 1;
    spawn.pelvisPosition = { 0.0f, 0.0f, 3.0f };
    spawn.active = true;
    const RagdollHandle3D ragdoll = scene->createRagdoll(profile, spawn);
    const ArticulationModel3D model = buildArticulationModel3D(profile, scene->ragdollEffectiveMass(ragdoll));
    const ArticulationState3D state = articulationStateFromRagdoll3D(model, *scene->ragdollStateView(ragdoll));
    const ArticulationKinematics3D k = computeArticulationKinematics3D(model, state);
    const ArticulationDynamics3D d = computeArticulationDynamics3D(model, k);
    const int n = model.dofCount;
    WholeBodyQPInput3D input;
    input.model = &model;
    input.kinematics = &k;
    input.dynamics = &d;
    input.state = &state;
    input.torqueLimit = Eigen::VectorXd::Constant(n - 6, 1000.0);
    const int first = n - 1, second = n - 2;
    // Num nivel so: o peso 1000 ganha.
    input.tasks = { jointTask(n, first, 5.0, 1.0, 1), jointTask(n, first, -5.0, 1000.0, 1), jointTask(n, second, 3.0, 1.0, 1) };
    WholeBodyQP3D flat;
    const WholeBodyQPResult3D single = flat.solve(input);
    check(single.usable() && single.path == WholeBodyQPPath3D::Single, "um nivel resolve");
    check(single.usable() && single.acceleration[first] < -4.9, "num nivel so, o peso maior ganha");
    // Em dois niveis: a de cima sai exata; a de baixo que nao conflita, tambem.
    input.tasks[0].priority = 0;
    WholeBodyQP3D layered;
    const WholeBodyQPResult3D result = layered.solve(input);
    check(result.usable() && result.path == WholeBodyQPPath3D::Hierarchical, "dois niveis resolvem");
    check(result.usable() && std::abs(result.acceleration[first] - 5.0) < 1e-3, "a prioridade 0 sai exata");
    check(result.usable() && std::abs(result.acceleration[second] - 3.0) < 1e-2, "o que nao conflita embaixo e atendido");
    // O pe numa linha (nota 29): ponto completo num extremo e livre ao longo dela no outro.
    // Gira sobre a linha, nao gira em torno de outro eixo, e os extremos nao aceleram.
    std::size_t foot = 0;
    while (foot < profile.links.size() && profile.links[foot].id != "LeftFoot") ++foot;
    check(foot < profile.links.size(), "acha o pe esquerdo");
    if (foot < profile.links.size()) {
        const int link = static_cast<int>(foot);
        const Eigen::Vector3d along = k.rotation[foot].col(1), across = k.rotation[foot].col(0);
        const Eigen::Vector3d a = k.origin[foot] - 0.04 * along, b = k.origin[foot] + 0.04 * along;
        WholeBodyQPInput3D line = input;
        line.tasks.clear();
        for (const Eigen::Vector3d& point : { a, b }) {
            WholeBodyContactPoint3D contact;
            contact.link = link;
            contact.pointWorld = point;
            contact.constrained = true;
            contact.maximumNormalForce = 0.0;
            if (point == b) contact.freeDirection = (b - a).normalized();
            line.contacts.push_back(contact);
        }
        const auto spin = [&](const Eigen::Vector3d& axis, double weight) {
            WholeBodyTask3D task;
            task.name = "giro";
            task.matrix = axis.transpose() * k.angularJacobian[foot];
            task.target = Eigen::VectorXd::Constant(1, 5.0 - axis.dot(k.angularBias[foot]));
            task.weight = weight;
            return task;
        };
        line.tasks = { spin(along, 100.0), spin(across, 100.0) };
        WholeBodyQP3D rolling;
        const WholeBodyQPResult3D r = rolling.solve(line);
        check(r.usable(), "a linha resolve");
        if (r.usable()) {
            const Eigen::Vector3d alpha = k.angularJacobian[foot] * r.acceleration + k.angularBias[foot];
            check(std::abs(along.dot(alpha) - 5.0) < 1e-2, "gira sobre a linha");
            check(std::abs(across.dot(alpha)) < 1e-2, "nao gira em torno de outro eixo");
            const auto pointAcceleration = [&](const Eigen::Vector3d& point) -> Eigen::Vector3d {
                return articulationPointJacobian3D(k, link, point) * r.acceleration + articulationPointBias3D(k, link, point);
            };
            const Eigen::Vector3d atB = pointAcceleration(b), t = (b - a).normalized();
            check(pointAcceleration(a).norm() < 1e-4, "o extremo completo nao acelera");
            check((atB - t.dot(atB) * t).norm() < 1e-4, "o outro extremo so anda ao longo da linha");
        }
    }
    std::printf(failures ? "QP em dois niveis: %d falha(s)\n" : "QP em dois niveis: ok\n", failures);
    return failures ? 1 : 0;
}
