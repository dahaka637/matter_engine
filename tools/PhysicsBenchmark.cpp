#include "Engine/Geometry/PhysicalAsset3D.hpp"
#include "Engine/Materials/MaterialLibrary.hpp"
#include "Engine/Physics/PhysicsScene3D.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <span>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

using namespace MatterEngine;

MeshData3D makeGround(float halfExtent) {
    MeshData3D mesh;
    mesh.vertices = {
        { { -halfExtent, -halfExtent, 0.0f } },
        { { halfExtent, -halfExtent, 0.0f } },
        { { halfExtent, halfExtent, 0.0f } },
        { { -halfExtent, halfExtent, 0.0f } }
    };
    mesh.indices = { 0, 1, 2, 0, 2, 3 };
    recomputeBounds(mesh);
    return mesh;
}

float percentile(const std::vector<float>& sorted, float fraction) {
    if (sorted.empty()) return 0.0f;
    const std::size_t index = std::min(sorted.size() - 1,
        static_cast<std::size_t>(fraction
            * static_cast<float>(sorted.size() - 1)));
    return sorted[index];
}

} // namespace

int main() {
    try {
        MaterialLibrary materials;
        PhysicsEngine3D physics;
        auto scene = physics.createScene({}, materials);

        const auto groundMesh = physics.cookStaticTriangleMesh(
            makeGround(90.0f));
        PhysicsShape3D groundShape;
        groundShape.type = PhysicsShapeType3D::TriangleMesh;
        groundShape.mesh = groundMesh;
        groundShape.materialId = "concrete";
        PhysicsBodyDefinition3D groundBody;
        groundBody.motionType = PhysicsMotionType3D::Static;
        groundBody.materialId = "concrete";
        static_cast<void>(scene->createBody(groundBody,
            std::span<const PhysicsShape3D>(&groundShape, 1)));

        constexpr std::array files {
            "wood_chair.glb", "anvil.glb", "plastic_barrel.glb",
            "soccer_ball.glb", "wood_crate.glb", "car_tire.glb"
        };
        std::vector<PhysicalAsset3D> assets;
        assets.reserve(files.size());
        for (const char* file : files) {
            assets.push_back(loadPhysicalAsset3D(
                std::string(MATTERENGINE_ASSETS_DIR)
                    + "/models/props/" + file,
                materials, physics));
        }

        constexpr std::size_t PropCount = 1000;
        std::vector<PhysicsBodyHandle3D> bodies;
        bodies.reserve(PropCount);
        for (std::size_t index = 0; index < PropCount; ++index) {
            const PhysicalAsset3D& asset = assets[index % assets.size()];
            PhysicsBodyDefinition3D body = asset.bodyTemplate;
            const std::size_t x = index % 40;
            const std::size_t y = index / 40;
            body.entityId = index + 1;
            body.position = {
                static_cast<float>(x) * 1.55f - 30.0f,
                static_cast<float>(y) * 1.55f - 18.0f,
                1.2f + static_cast<float>(index % 7) * 0.32f
            };
            bodies.push_back(scene->createBody(body,
                asset.collision.shapes));
        }

        std::vector<float> stepTimes;
        stepTimes.reserve(600);
        const auto benchmarkStart = std::chrono::steady_clock::now();
        for (int step = 0; step < 600; ++step) {
            scene->simulate(1.0f / 120.0f);
            stepTimes.push_back(scene->diagnostics().totalStepMilliseconds);
        }
        const float elapsedSeconds = std::chrono::duration<float>(
            std::chrono::steady_clock::now() - benchmarkStart).count();
        std::sort(stepTimes.begin(), stepTimes.end());
        const PhysicsStepDiagnostics3D& diagnostics = scene->diagnostics();

        std::cout << std::fixed << std::setprecision(3)
            << "MatterEngine PhysX 1000-prop benchmark\n"
            << "  600 passos: " << elapsedSeconds << " s\n"
            << "  passo P50:  " << percentile(stepTimes, 0.50f) << " ms\n"
            << "  passo P95:  " << percentile(stepTimes, 0.95f) << " ms\n"
            << "  passo P99:  " << percentile(stepTimes, 0.99f) << " ms\n"
            << "  pior passo: " << stepTimes.back() << " ms\n"
            << "  ativos:     " << diagnostics.activeDynamicBodyCount << '\n'
            << "  dormindo:   " << diagnostics.sleepingDynamicBodyCount << '\n'
            << "  workers:    " << diagnostics.physicsWorkerCount << '\n'
            << "  contatos:   " << diagnostics.discreteContactPairs << '\n'
            << "  pares CCD:  " << diagnostics.ccdPairs << '\n';
        return diagnostics.dynamicBodyCount >= PropCount ? 0 : 1;
    } catch (const std::exception& error) {
        std::cerr << "Physics benchmark failed: " << error.what() << '\n';
        return 1;
    }
}
