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
    check(single.usable() && !single.hierarchical, "um nivel resolve");
    check(single.usable() && single.acceleration[first] < -4.9, "num nivel so, o peso maior ganha");
    // Em dois niveis: a de cima sai exata; a de baixo que nao conflita, tambem.
    input.tasks[0].priority = 0;
    WholeBodyQP3D layered;
    const WholeBodyQPResult3D result = layered.solve(input);
    check(result.usable() && result.hierarchical, "dois niveis resolvem");
    check(result.usable() && std::abs(result.acceleration[first] - 5.0) < 1e-3, "a prioridade 0 sai exata");
    check(result.usable() && std::abs(result.acceleration[second] - 3.0) < 1e-2, "o que nao conflita embaixo e atendido");
    std::printf(failures ? "QP em dois niveis: %d falha(s)\n" : "QP em dois niveis: ok\n", failures);
    return failures ? 1 : 0;
}
