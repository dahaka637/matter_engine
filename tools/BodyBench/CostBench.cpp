// A matriz de custo (G0 da locomocao fisica nova): o passo de fisica com N
// atletas de pe (motores do perfil, gravidade do backend), com 1 e com 4
// subpassos de colisao. Mede o simulate() por tick (p50/p99), sem o controle.
//
// uso: MatterBodyCostBench [--ticks 600]
#include "Engine/Materials/MaterialLibrary.hpp"
#include "Engine/Physics/PhysicsEngine3D.hpp"
#include "Engine/Physics/PhysicsScene3D.hpp"
#include "Engine/Physics/RagdollProfile3D.hpp"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <string>
#include <vector>

using namespace MatterEngine;

namespace {

double percentile(std::vector<double> values, double q) {
    std::sort(values.begin(), values.end());
    return values[std::min(values.size() - 1, static_cast<std::size_t>(q * double(values.size() - 1)))];
}

} // namespace

int main(int argc, char** argv) {
    int ticks = 600;
    for (int i = 1; i < argc; ++i)
        if (std::string(argv[i]) == "--ticks" && i + 1 < argc) ticks = std::stoi(argv[++i]);
    const RagdollProfile3D profile = loadRagdollProfile3D(
        std::string(MATTERENGINE_ASSETS_DIR) + "/characters/football_player/FootballPlayerV1.ragdoll.json");
    std::printf("| atletas | subpassos | simulate p50 (ms) | p99 (ms) | por atleta p50 (us) |\n|---|---|---|---|---|\n");
    for (const int athletes : { 1, 2, 6, 22 }) {
        for (const std::uint32_t steps : { 1u, 4u }) {
            PhysicsEngine3D engine;
            MaterialLibrary materials;
            PhysicsSceneSettings3D settings;
            settings.collisionSteps = steps;
            auto scene = engine.createScene(settings, materials);
            PhysicsShape3D shape;
            shape.type = PhysicsShapeType3D::Box;
            shape.halfExtents = { 60.0f, 60.0f, 0.25f };
            shape.materialId = "grass";
            PhysicsBodyDefinition3D ground;
            ground.motionType = PhysicsMotionType3D::Static;
            ground.position = { 0.0f, 0.0f, -0.25f };
            ground.materialId = "grass";
            (void)scene->createBody(ground, std::span<const PhysicsShape3D>(&shape, 1));
            std::vector<RagdollDriveTarget3D> targets;
            for (std::size_t i = 1; i < profile.links.size(); ++i)
                for (int a = 0; a < 3; ++a) {
                    if (!profile.links[i].inboundJoint.axes[static_cast<std::size_t>(a)].enabled) continue;
                    RagdollDriveTarget3D target;
                    target.linkIndex = static_cast<std::uint32_t>(i);
                    target.axis = static_cast<RagdollAxis3D>(a);
                    target.stiffnessScale = 2.0f;
                    target.dampingScale = 2.0f;
                    target.maximumTorqueScale = 2.0f;
                    targets.push_back(target);
                }
            for (int n = 0; n < athletes; ++n) {
                RagdollSpawnDefinition3D spawn;
                spawn.entityId = static_cast<std::uint64_t>(n + 1);
                // Em pares encostados (o caso caro: ilhas grandes) e o resto espalhado.
                spawn.pelvisPosition = { float((n % 11) * 1.2), float((n / 11) * 3.0), profile.standingRootHeightMeters + 0.01f };
                const RagdollHandle3D ragdoll = scene->createRagdoll(profile, spawn);
                scene->setRagdollActiveDriveTargets(ragdoll, targets, true);
            }
            std::vector<double> times;
            for (int t = 0; t < ticks; ++t) {
                const auto start = std::chrono::steady_clock::now();
                scene->simulate(1.0f / 120.0f);
                if (t >= 60) times.push_back(std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count());
            }
            const double p50 = percentile(times, 0.5), p99 = percentile(times, 0.99);
            std::printf("| %d | %u | %.3f | %.3f | %.0f |\n", athletes, steps, p50, p99, 1000.0 * p50 / athletes);
        }
    }
    return 0;
}
