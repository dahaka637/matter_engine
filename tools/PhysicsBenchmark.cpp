#include "Engine/Geometry/PhysicalAsset3D.hpp"
#include "Engine/Materials/MaterialLibrary.hpp"
#include "Engine/Physics/PhysicsScene3D.hpp"
#include "Engine/Physics/RagdollProfile3D.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace {

using namespace MatterEngine;

constexpr float FixedDeltaTime = 1.0f / 120.0f;

MeshData3D makeGround(float halfExtent) {
    MeshData3D mesh;
    mesh.vertices.resize(4);
    mesh.vertices[0].position = { -halfExtent, -halfExtent, 0.0f };
    mesh.vertices[1].position = { halfExtent, -halfExtent, 0.0f };
    mesh.vertices[2].position = { halfExtent, halfExtent, 0.0f };
    mesh.vertices[3].position = { -halfExtent, halfExtent, 0.0f };
    mesh.indices = { 0, 1, 2, 0, 2, 3 };
    recomputeBounds(mesh);
    return mesh;
}

float percentile(std::vector<float> values, float fraction) {
    if (values.empty()) return 0.0f;
    std::sort(values.begin(), values.end());
    const std::size_t index = std::min(values.size() - 1,
        static_cast<std::size_t>(fraction
            * static_cast<float>(values.size() - 1)));
    return values[index];
}

float mean(const std::vector<float>& values) {
    float sum = 0.0f;
    for (float value : values) sum += value;
    return values.empty() ? 0.0f : sum / static_cast<float>(values.size());
}

struct BenchmarkSamples {
    std::vector<float> total;
    std::vector<float> preSimulation;
    std::vector<float> dispatch;
    std::vector<float> wait;
    std::vector<float> callbacks;
    std::vector<float> stateSync;
    std::vector<float> statistics;
    std::size_t maximumContacts = 0;
    std::size_t maximumReportedPairs = 0;
    std::size_t maximumReportedPoints = 0;
    std::size_t maximumTasks = 0;
    std::size_t maximumActiveBodies = 0;
    std::uint32_t maximumWorkers = 0;
};

void sampleStep(PhysicsScene3D& scene, BenchmarkSamples& samples) {
    scene.simulate(FixedDeltaTime);
    const PhysicsStepDiagnostics3D& diagnostics = scene.diagnostics();
    samples.total.push_back(diagnostics.totalStepMilliseconds);
    samples.preSimulation.push_back(diagnostics.preSimulationMilliseconds);
    samples.dispatch.push_back(
        diagnostics.simulationDispatchMilliseconds);
    samples.wait.push_back(diagnostics.simulationWaitMilliseconds);
    samples.callbacks.push_back(diagnostics.contactCallbackMilliseconds);
    samples.stateSync.push_back(diagnostics.stateSyncMilliseconds);
    samples.statistics.push_back(diagnostics.statisticsMilliseconds);
    samples.maximumContacts = std::max(samples.maximumContacts,
        diagnostics.discreteContactPairs);
    samples.maximumReportedPairs = std::max(samples.maximumReportedPairs,
        diagnostics.reportedContactPairs);
    samples.maximumReportedPoints = std::max(samples.maximumReportedPoints,
        diagnostics.reportedContactPoints);
    samples.maximumTasks = std::max(samples.maximumTasks,
        diagnostics.submittedPhysicsTasks);
    samples.maximumActiveBodies = std::max(samples.maximumActiveBodies,
        diagnostics.activeDynamicBodyCount);
    samples.maximumWorkers = std::max(samples.maximumWorkers,
        diagnostics.physicsWorkerCount);
}

void printSamples(std::string_view name, const BenchmarkSamples& samples) {
    std::cout << std::left << std::setw(24) << name << std::right
        << " P50 " << std::setw(8) << percentile(samples.total, 0.50f)
        << "  P95 " << std::setw(8) << percentile(samples.total, 0.95f)
        << "  max " << std::setw(8) << percentile(samples.total, 1.0f)
        << " ms\n"
        << "  medias: pre " << std::setw(7)
        << mean(samples.preSimulation)
        << " | dispatch " << std::setw(7) << mean(samples.dispatch)
        << " | solver/wait " << std::setw(7) << mean(samples.wait)
        << " | callback " << std::setw(7) << mean(samples.callbacks)
        << " | sync " << std::setw(7) << mean(samples.stateSync)
        << " | stats " << std::setw(7) << mean(samples.statistics)
        << " ms\n"
        << "  maximos: ativos " << samples.maximumActiveBodies
        << " | contatos " << samples.maximumContacts
        << " | pares reportados " << samples.maximumReportedPairs
        << " | pontos reportados " << samples.maximumReportedPoints
        << " | tarefas " << samples.maximumTasks
        << " | workers " << samples.maximumWorkers << "\n\n";
}

std::unique_ptr<PhysicsScene3D> createSceneWithGround(
    PhysicsEngine3D& physics, const MaterialLibrary& materials,
    std::uint32_t workerCount = 0) {
    PhysicsSceneSettings3D settings;
    settings.workerThreadCount = workerCount;
    settings.solverPositionIterations = 4;
    settings.solverVelocityIterations = 1;
    settings.enableContinuousCollision = true;
    settings.enableStabilization = true;
    auto scene = physics.createScene(settings, materials);

    const auto groundMesh = physics.cookStaticTriangleMesh(makeGround(90.0f));
    PhysicsShape3D groundShape;
    groundShape.type = PhysicsShapeType3D::TriangleMesh;
    groundShape.mesh = groundMesh;
    groundShape.materialId = "concrete";
    PhysicsBodyDefinition3D groundBody;
    groundBody.motionType = PhysicsMotionType3D::Static;
    groundBody.materialId = "concrete";
    static_cast<void>(scene->createBody(groundBody,
        std::span<const PhysicsShape3D>(&groundShape, 1)));
    return scene;
}

PhysicsBodyHandle3D createBox(PhysicsScene3D& scene, Vec3 position,
    Vec3 halfExtents, float massKg = 5.0f) {
    PhysicsShape3D shape;
    shape.type = PhysicsShapeType3D::Box;
    shape.halfExtents = halfExtents;
    shape.materialId = "wood";
    PhysicsBodyDefinition3D body;
    body.position = position;
    body.massKg = massKg;
    body.materialId = "wood";
    body.aerodynamicDragEnabled = false;
    return scene.createBody(body,
        std::span<const PhysicsShape3D>(&shape, 1));
}

PhysicsBodyHandle3D createSphere(PhysicsScene3D& scene, Vec3 position,
    float radius, float massKg = 0.45f) {
    PhysicsShape3D shape;
    shape.type = PhysicsShapeType3D::Sphere;
    shape.radius = radius;
    shape.materialId = "rubber";
    PhysicsBodyDefinition3D body;
    body.position = position;
    body.massKg = massKg;
    body.materialId = "rubber";
    body.aerodynamicDragEnabled = false;
    return scene.createBody(body,
        std::span<const PhysicsShape3D>(&shape, 1));
}

template <typename Setup>
BenchmarkSamples runScenario(PhysicsEngine3D& physics,
    const MaterialLibrary& materials, int steps, Setup&& setup) {
    auto scene = createSceneWithGround(physics, materials);
    setup(*scene);
    BenchmarkSamples samples;
    samples.total.reserve(static_cast<std::size_t>(steps));
    for (int step = 0; step < steps; ++step) {
        sampleStep(*scene, samples);
    }
    return samples;
}

void spawnRagdolls(PhysicsScene3D& scene,
    const RagdollProfile3D& profile, std::size_t count,
    float spacingMeters, std::size_t columns, float rigidityPercent) {
    for (std::size_t index = 0; index < count; ++index) {
        const std::size_t column = index % columns;
        const std::size_t row = index / columns;
        RagdollSpawnDefinition3D spawn;
        spawn.entityId = 10'000 + index;
        spawn.pelvisPosition = {
            (static_cast<float>(column)
                - static_cast<float>(columns - 1) * 0.5f) * spacingMeters,
            (static_cast<float>(row)
                - static_cast<float>((count + columns - 1) / columns - 1)
                    * 0.5f) * spacingMeters,
            profile.standingRootHeightMeters + 0.04f
        };
        spawn.rigidityPercent = rigidityPercent;
        static_cast<void>(scene.createRagdoll(profile, spawn));
    }
}

} // namespace

int main() {
    try {
        std::cout << std::fixed << std::setprecision(3);
        MaterialLibrary materials;
        PhysicsEngine3D physics;
        const RagdollProfile3D ragdoll = loadRagdollProfile3D(
            std::string(MATTERENGINE_ASSETS_DIR)
                + "/physics/ragdolls/HumanAdultV1.ragdoll.json");

        constexpr int StepCount = 120;
        std::cout << "MatterEngine physics workload benchmark ("
            << StepCount << " passos @ 120 Hz, build "
#if defined(NDEBUG)
            << "Release"
#else
            << "Debug"
#endif
            << ")\n\n";

        printSamples("cena vazia", runScenario(physics, materials,
            StepCount, [](PhysicsScene3D&) {}));
        printSamples("caixa + bola", runScenario(physics, materials,
            StepCount, [](PhysicsScene3D& scene) {
                static_cast<void>(createBox(scene,
                    { 0.0f, 0.0f, 0.50f }, { 0.50f, 0.50f, 0.50f }));
                static_cast<void>(createSphere(scene,
                    { 0.0f, 0.0f, 1.35f }, 0.35f));
            }));
        printSamples("4 corpos separados", runScenario(physics, materials,
            StepCount, [](PhysicsScene3D& scene) {
                for (int index = 0; index < 4; ++index) {
                    static_cast<void>(createSphere(scene,
                        { static_cast<float>(index) * 2.0f,
                            0.0f, 1.0f + 0.2f * index }, 0.35f));
                }
            }));
        printSamples("1 ragdoll", runScenario(physics, materials,
            StepCount, [&](PhysicsScene3D& scene) {
                spawnRagdolls(scene, ragdoll, 1, 1.0f, 1, 0.0f);
            }));
        printSamples("2 ragdolls contato", runScenario(physics, materials,
            StepCount, [&](PhysicsScene3D& scene) {
                spawnRagdolls(scene, ragdoll, 2, 0.55f, 2, 0.0f);
            }));
        printSamples("3 ragdolls contato", runScenario(physics, materials,
            StepCount, [&](PhysicsScene3D& scene) {
                spawnRagdolls(scene, ragdoll, 3, 0.48f, 3, 0.0f);
            }));
        printSamples("22 ragdolls dispersos", runScenario(physics, materials,
            60, [&](PhysicsScene3D& scene) {
                spawnRagdolls(scene, ragdoll, 22, 2.25f, 11, 0.0f);
            }));
        printSamples("22 ragdolls campo", runScenario(physics, materials,
            60, [&](PhysicsScene3D& scene) {
                spawnRagdolls(scene, ragdoll, 22, 0.78f, 11, 20.0f);
            }));
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "Physics benchmark failed: " << error.what() << '\n';
        return 1;
    }
}
