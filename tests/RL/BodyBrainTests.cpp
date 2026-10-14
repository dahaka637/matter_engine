#include "Engine/Materials/MaterialLibrary.hpp"
#include "Engine/Physics/PhysicsEngine3D.hpp"
#include "Engine/Physics/PhysicsScene3D.hpp"
#include "Engine/RL/Brain/BodyBrain3D.hpp"
#include "Engine/RL/Environment/BodyTrainingBatch3D.hpp"
#include "Engine/RL/Policy/PolicyRuntime3D.hpp"

#include <cmath>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

using namespace MatterEngine;

void require(bool condition, const std::string& message) {
    if (!condition) throw std::runtime_error(message);
}

// Uma politica pequena e aleatoria no formato do treinador (.mrlpolicy).
std::filesystem::path writeRandomPolicy(const RLVectorLayout3D& observation, std::size_t actions) {
    const std::size_t O = observation.size(), H = 32;
    std::mt19937 rng(11);
    std::normal_distribution<float> gauss(0.0f, 1.0f);
    std::vector<float> floats;
    const auto push = [&](std::size_t count, float scale, float fill) {
        const std::size_t at = floats.size();
        for (std::size_t i = 0; i < count; ++i) floats.push_back(scale > 0.0f ? gauss(rng) * scale : fill);
        return at;
    };
    const std::size_t mean = push(O, 0.0f, 0.0f);
    const std::size_t inv = push(O, 0.0f, 1.0f);
    const std::size_t w1 = push(H * O, 0.05f, 0.0f), b1 = push(H, 0.0f, 0.0f);
    const std::size_t w2 = push(actions * H, 0.2f, 0.0f), b2 = push(actions, 0.0f, 0.0f);
    std::string names;
    for (std::size_t c = 0; c < O; ++c) names += (c ? ",\"" : "\"") + observation.channels[c].name + "\"";
    const std::string header = "{\"schema\":\"matter-rl-policy-1\",\"observationSize\":" + std::to_string(O)
        + ",\"actionSize\":" + std::to_string(actions) + ",\"activation\":\"elu\",\"normalizer\":{\"mean\":"
        + std::to_string(mean) + ",\"invStd\":" + std::to_string(inv) + ",\"clip\":5.0},\"actionLimit\":10.0,\"layers\":["
        + "{\"inputs\":" + std::to_string(O) + ",\"outputs\":" + std::to_string(H) + ",\"weights\":" + std::to_string(w1)
        + ",\"bias\":" + std::to_string(b1) + "},{\"inputs\":" + std::to_string(H) + ",\"outputs\":" + std::to_string(actions)
        + ",\"weights\":" + std::to_string(w2) + ",\"bias\":" + std::to_string(b2) + "}],\"observationNames\":[" + names + "]}";
    const std::filesystem::path path = std::filesystem::temp_directory_path() / "matter_rl_teste_cerebro.mrlpolicy";
    std::ofstream out(path, std::ios::binary);
    out.write("MRLPOL01", 8);
    const auto size = static_cast<std::uint32_t>(header.size());
    out.write(reinterpret_cast<const char*>(&size), 4);
    out.write(header.data(), static_cast<std::streamsize>(header.size()));
    const std::size_t padding = (4 - (12 + header.size()) % 4) % 4;
    for (std::size_t i = 0; i < padding; ++i) out.put('\0');
    out.write(reinterpret_cast<const char*>(floats.data()), static_cast<std::streamsize>(floats.size() * sizeof(float)));
    return path;
}

// A paridade do runtime: o cerebro do jogo, numa cena so dele, com o mesmo
// estado inicial e os mesmos comandos, da a mesma entrada a cada decisao e a
// mesma trajetoria do mundo 0 do lote de treino (na avaliacao: sem ruido nem
// variacao escondida).
void testBrainMatchesBatch() {
    BodyBatchSettings3D settings;
    settings.taskPath = std::string(MATTERENGINE_TEST_ASSETS_DIR) + "/rl/tasks/corpo2.json";
    settings.environmentCount = 4;
    settings.environmentsPerScene = 4;
    settings.workerThreads = 1;
    settings.seed = 9;
    settings.evaluation = true;
    settings.drill = "deslocamento";
    BodyTrainingBatch3D batch(settings);
    const std::filesystem::path policyPath = writeRandomPolicy(batch.observationLayout(), batch.actionSize());
    const PolicyRuntime3D policy(policyPath);
    const BodyBrain3D brain(batch.body(), policyPath, batch.task().contract);
    std::filesystem::remove(policyPath);

    // A cena do cerebro: o mesmo chao da cena 0 do lote e o boneco no estado do mundo 0.
    MaterialLibrary materials;
    PhysicsEngine3D engine;
    PhysicsSceneSettings3D sceneSettings;
    sceneSettings.gravity = { 0.0f, 0.0f, -9.81f };
    sceneSettings.workerThreadCount = 1;
    sceneSettings.runJobsInline = true;
    sceneSettings.maximumBodies = 64u + 32u * 4u;
    sceneSettings.maximumBodyPairs = 2048u * 4u;
    sceneSettings.maximumContactConstraints = 1024u * 4u;
    sceneSettings.scratchBufferSizeBytes = 6u * 1024u * 1024u;
    sceneSettings.windShelterDistanceMeters = 0.0f;
    auto scene = engine.createScene(sceneSettings, materials);
    const float spacing = batch.task().worldSpacingMeters;
    PhysicsShape3D shape;
    shape.type = PhysicsShapeType3D::Box;
    shape.halfExtents = { 0.5f * spacing * 4.0f + std::max(20.0f, 0.5f * spacing), std::max(20.0f, 0.5f * spacing), 0.5f };
    shape.materialId = batch.task().groundMaterial;
    PhysicsBodyDefinition3D ground;
    ground.motionType = PhysicsMotionType3D::Static;
    ground.position = { 0.0f, 0.0f, -0.5f };
    ground.materialId = batch.task().groundMaterial;
    (void)scene->createBody(ground, std::span<const PhysicsShape3D>(&shape, 1));
    const BodyWorldView3D start = batch.view(0);
    RagdollSpawnDefinition3D spawn;
    spawn.entityId = 1;
    spawn.pelvisPosition = start.origin + Vec3 { 0.0f, 0.0f, batch.body().standingPelvisHeight + 0.05f };
    spawn.active = true;
    spawn.rigidityPercent = 0.0f;
    spawn.solverPositionIterations = batch.body().solverPositionIterations;
    spawn.solverVelocityIterations = batch.body().solverVelocityIterations;
    const RagdollHandle3D ragdoll = scene->createRagdoll(batch.body().profile, spawn);
    scene->resetRagdollState(ragdoll, start.state->links);
    scene->setRagdollFrictionScale(ragdoll, 1.0f);
    BodyBrainPlayer3D player;
    brain.reset(player, *scene->ragdollStateView(ragdoll));
    const float weight = batch.body().profile.totalMassKg * 9.81f;

    const std::size_t O = batch.observationSize(), A = batch.actionSize();
    std::vector<float> actions(batch.environmentCount() * A);
    float worstObservation = 0.0f, worstPosition = 0.0f;
    for (int decision = 0; decision < 24; ++decision) {
        const BodyWorldView3D view = batch.view(0);
        // O jogo: o mesmo comando e os mesmos dados do jogador.
        brain.decide(player, *scene->ragdollStateView(ragdoll), view.command, view.player);
        for (std::size_t k = 0; k < O; ++k)
            worstObservation = std::max(worstObservation, std::abs(player.observation[k] - batch.observations()[k]));
        for (std::size_t w = 0; w < batch.environmentCount(); ++w)
            policy.evaluate(batch.observations().subspan(w * O, O), std::span<float>(actions.data() + w * A, A));
        batch.step(actions, actions);
        require(batch.finishedEpisodes().empty(), "o mundo 0 nao pode recomecar no meio do teste");
        for (std::uint32_t sub = 0; sub < batch.task().contract.substepsPerDecision; ++sub) {
            scene->setRagdollActiveDriveTargets(ragdoll, brain.targets(player, sub, view.player), false);
            scene->simulate(1.0f / 120.0f);
            brain.afterStep(player, *scene->ragdollStateView(ragdoll), weight, 1.0f / 120.0f);
        }
        const RagdollState3D& mine = *scene->ragdollStateView(ragdoll);
        const RagdollState3D& theirs = *batch.view(0).state;
        for (std::size_t i = 0; i < mine.links.size(); ++i)
            worstPosition = std::max(worstPosition, (mine.links[i].position - theirs.links[i].position).length());
    }
    std::cout << "cerebro: a entrada difere ate " << worstObservation << " e a trajetoria ate " << worstPosition * 1000.0f
              << " mm em 24 decisoes\n";
    require(worstObservation < 1.0e-4f, "o cerebro do jogo monta a entrada do treino");
    require(worstPosition < 1.0e-3f, "o cerebro do jogo da a trajetoria do treino");
}

} // namespace

void runBodyBrainTests() {
    testBrainMatchesBatch();
    std::cout << "Body brain tests passed\n";
}
