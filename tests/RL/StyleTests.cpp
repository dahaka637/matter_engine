#include "Engine/Animation/AnimationClip3D.hpp"
#include "Engine/Materials/MaterialLibrary.hpp"
#include "Engine/Physics/PhysicsEngine3D.hpp"
#include "Engine/Physics/PhysicsScene3D.hpp"
#include "Engine/RL/Body/RLBody3D.hpp"
#include "Engine/RL/Environment/BodyTrainingBatch3D.hpp"
#include "Engine/RL/Imitation/ReferenceMotion3D.hpp"
#include "Engine/RL/Style/MotionDataset3D.hpp"
#include "Engine/RL/Style/StyleFeatures3D.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

using namespace MatterEngine;

void require(bool condition, const std::string& message) {
    if (!condition) throw std::runtime_error(message);
}

std::string assets(const std::string& path) { return std::string(MATTERENGINE_TEST_ASSETS_DIR) + "/" + path; }

RLBody3D bodyV2() { return loadRLBody3D(assets("rl/bodies/football_player_v2.json")); }

ReferenceMotion3D clip(const RLBody3D& body, const std::string& id) {
    const std::filesystem::path path = assets("animations/clips/" + id + ".matteranim.json");
    const ReferenceClip3D request { id, path, loadAnimationClip3D(path.string()).loops };
    return buildReferenceMotion3D(body, std::span<const ReferenceClip3D>(&request, 1), 1.0f / 120.0f, true);
}

std::vector<float> featuresOf(const StyleFeatures3D& style, std::span<const PhysicsBodyState3D> links) {
    std::vector<float> out(style.size());
    style.write(links, 1.0f, out.data());
    return out;
}

float worst(const std::vector<float>& a, const std::vector<float>& b, std::size_t from = 0, std::size_t to = ~std::size_t {}) {
    float result = 0.0f;
    for (std::size_t k = from; k < std::min({ a.size(), b.size(), to }); ++k) result = std::max(result, std::abs(a[k] - b[k]));
    return result;
}

// O que o juiz ve: 95 numeros, sem as pontas, nomes unicos.
void testStyleLayout() {
    const RLBody3D body = bodyV2();
    const StyleFeatures3D style(body);
    require(style.dofs().size() == body.dofs.size() - 2, "o juiz nao ve as pontas dos pes");
    require(style.size() == 1 + 3 + 3 + 3 + 2 * style.dofs().size() + 15, "o tamanho do instante");
    std::set<std::string> names;
    for (const RLChannel3D& channel : style.layout().channels) {
        require(names.insert(channel.name).second, "nome repetido: " + channel.name);
        require(channel.name.find("Toe") == std::string::npos, "nenhum canal da ponta: " + channel.name);
    }
}

// Sem posicao no campo nem rumo absoluto; e batendo com o quadro do clipe.
void testStyleFrames() {
    const RLBody3D body = bodyV2();
    const StyleFeatures3D style(body);
    const ReferenceMotion3D motion = clip(body, "trot_forward_left");
    const ReferenceSegment3D& segment = motion.segments.front();
    std::vector<PhysicsBodyState3D> a, b;
    for (std::size_t f = 0; f < segment.frames.size(); f += 7) {
        const ReferenceFrame3D& frame = segment.frames[f];
        referenceLinkStates3D(body, frame, {}, frame.heading, a);
        referenceLinkStates3D(body, frame, { 12.0f, -40.0f, 0.0f }, frame.heading + 2.1f, b);
        const std::vector<float> fa = featuresOf(style, a);
        const std::vector<float> fb = featuresOf(style, b);
        // (As velocidades da reconstrucao saem de diferencas de orientacao em
        // 1 ms: o float da ~1e-4 rad/s de ruido em outro rumo.)
        for (std::size_t k = 0; k < fa.size(); ++k) {
            const std::string& name = style.layout().channels[k].name;
            const bool rate = name.find("velocity") != std::string::npos || name.find("spin") != std::string::npos;
            require(std::abs(fa[k] - fb[k]) < (rate ? 2.0e-3f : 2.0e-4f), "a mesma medida em qualquer lugar e rumo: "
                    + name + " " + std::to_string(fa[k]) + " contra " + std::to_string(fb[k]));
        }
        // Os angulos dos eixos sao os do quadro (a mesma coordenada da junta).
        const std::size_t base = 10;
        for (std::size_t k = 0; k < style.dofs().size(); ++k)
            require(std::abs(fa[base + k] - frame.dofRadians[style.dofs()[k]]) < 2.0e-4f,
                "o angulo do eixo e o do quadro: " + body.dofs[style.dofs()[k]].name);
        // A velocidade da pelve no rumo e a do quadro.
        require(std::abs(fa[4] - frame.rootVelocity.x) < 1.0e-3f && std::abs(fa[5] - frame.rootVelocity.y) < 1.0e-3f,
            "a velocidade da pelve e a do quadro");
        // A cabeca, as maos e os pes em relacao a pelve sao os do quadro.
        const std::size_t keys = base + 2 * style.dofs().size();
        for (std::size_t k = 0; k < 5; ++k)
            for (std::size_t c = 0; c < 3; ++c) {
                const float expected = c == 0 ? frame.keyBodies[k].x : c == 1 ? frame.keyBodies[k].y : frame.keyBodies[k].z;
                require(std::abs(fa[keys + 3 * k + c] - expected) < 2.0e-4f, "as partes-chave sao as do quadro");
            }
    }
}

// O teste do acervo: o clipe posto no corpo da fisica da as mesmas medidas que
// o acervo calcula, e a velocidade de cada eixo e a que o backend publica.
void testStyleDatasetParity() {
    const RLBody3D body = bodyV2();
    const StyleFeatures3D style(body);
    MaterialLibrary materials;
    PhysicsEngine3D engine;
    PhysicsSceneSettings3D sceneSettings;
    sceneSettings.gravity = { 0.0f, 0.0f, -9.81f };
    auto scene = engine.createScene(sceneSettings, materials);
    RagdollSpawnDefinition3D spawn;
    spawn.entityId = 1;
    spawn.pelvisPosition = { 0.0f, 0.0f, 1.0f };
    spawn.active = true;
    const RagdollHandle3D ragdoll = scene->createRagdoll(body.profile, spawn);
    float worstFeature = 0.0f, worstVelocity = 0.0f;
    std::string worstWhere;
    for (const char* id : { "alert_idle", "ual_walk", "trot_forward", "sprint", "jump_forward", "stand_up_front" }) {
        const ReferenceMotion3D motion = clip(body, id);
        const ReferenceSegment3D& segment = motion.segments.front();
        std::vector<PhysicsBodyState3D> links;
        for (std::size_t f = 0; f < segment.frames.size(); f += 11) {
            referenceLinkStates3D(body, segment.frames[f], { 3.0f, 1.0f, 0.0f }, 0.7f, links);
            scene->resetRagdollState(ragdoll, links);
            const RagdollState3D* state = scene->ragdollStateView(ragdoll);
            require(state != nullptr, "o boneco existe");
            const std::vector<float> fromPhysics = featuresOf(style, state->links);
            const std::vector<float> fromDataset = featuresOf(style, links);
            for (std::size_t k = 0; k < fromPhysics.size(); ++k)
                if (std::abs(fromPhysics[k] - fromDataset[k]) > worstFeature) {
                    worstFeature = std::abs(fromPhysics[k] - fromDataset[k]);
                    worstWhere = std::string(id) + " quadro " + std::to_string(f) + " " + style.layout().channels[k].name;
                }
            const std::size_t velocities = 10 + style.dofs().size();
            for (std::size_t k = 0; k < style.dofs().size(); ++k) {
                const RLBodyDof3D& dof = body.dofs[style.dofs()[k]];
                const float published = state->joints[dof.linkIndex].velocityRadiansPerSecond[static_cast<std::size_t>(dof.axis)];
                worstVelocity = std::max(worstVelocity, std::abs(published - fromPhysics[velocities + k]));
            }
        }
    }
    require(worstFeature < 2.0e-3f,
        "o clipe no corpo da as medidas do acervo (pior " + std::to_string(worstFeature) + " em " + worstWhere + ")");
    require(worstVelocity < 1.0e-3f, "a velocidade do eixo e a do backend (pior " + std::to_string(worstVelocity) + ")");
}

// O acervo: os grupos, os reservados e os pesos.
void testMotionDataset() {
    const RLBody3D body = bodyV2();
    const StyleFeatures3D style(body);
    const MotionDataset3D dataset = loadMotionDataset3D(body, style, assets("rl/style/acervo_corpo.json"));
    require(dataset.groups.size() == 8 && dataset.clips.size() == 47, "os 8 grupos e os 47 clipes que passaram nas folhas");
    require(dataset.pairs.size() == dataset.pairCount() * 2 * dataset.featureSize, "os pares tem dois instantes");
    double total = 0.0;
    std::vector<double> perGroup(dataset.groups.size(), 0.0);
    std::size_t reserved = 0;
    for (std::size_t p = 0; p < dataset.pairCount(); ++p) {
        const MotionClip3D& clip = dataset.clips[dataset.pairClip[p]];
        if (clip.reserved) require(dataset.pairWeight[p] == 0.0f, "o reservado fica fora do treino");
        total += dataset.pairWeight[p];
        perGroup[clip.group] += dataset.pairWeight[p];
    }
    for (const MotionClip3D& clip : dataset.clips) reserved += clip.reserved ? 1u : 0u;
    require(reserved == 12, "um clipe (ou um par espelhado) reservado por grupo");
    require(std::abs(total - 1.0) < 1.0e-4, "os pesos de treino somam 1");
    double weights = 0.0;
    for (const MotionGroup3D& group : dataset.groups) weights += group.weight;
    for (std::size_t g = 0; g < dataset.groups.size(); ++g)
        require(std::abs(perGroup[g] - dataset.groups[g].weight / weights) < 1.0e-4, "o peso e por grupo, nao por duracao");
    // Um par e o instante e o de pairFrames quadros depois.
    const MotionClip3D& walk = *std::find_if(dataset.clips.begin(), dataset.clips.end(),
        [](const MotionClip3D& c) { return c.id == "ual_walk"; });
    const ReferenceMotion3D motion = clip(body, "ual_walk");
    std::vector<PhysicsBodyState3D> links;
    const std::size_t F = dataset.featureSize;
    for (const std::uint32_t f : { 0u, 17u }) {
        const float* pair = dataset.pairs.data() + static_cast<std::size_t>(walk.firstPair + f) * 2 * F;
        referenceLinkStates3D(body, motion.segments.front().frames[f], {}, motion.segments.front().frames[f].heading, links);
        const std::vector<float> now = featuresOf(style, links);
        const ReferenceFrame3D& later = motion.segments.front().frames[f + dataset.pairFrames];
        referenceLinkStates3D(body, later, {}, later.heading, links);
        const std::vector<float> next = featuresOf(style, links);
        require(worst(std::vector<float>(pair, pair + F), now) < 1.0e-6f
                && worst(std::vector<float>(pair + F, pair + 2 * F), next) < 1.0e-6f,
            "o par e o instante e o da decisao seguinte");
    }
}

// O lote entrega o par da decisao: o instante de antes e o de agora.
void testBatchStylePairs() {
    BodyBatchSettings3D settings;
    settings.taskPath = assets("rl/tasks/corpo2.json");
    settings.environmentCount = 4;
    settings.environmentsPerScene = 2;
    settings.workerThreads = 1;
    settings.evaluation = true;
    BodyTrainingBatch3D batch(settings);
    const StyleFeatures3D style(batch.body());
    const std::size_t S = batch.styleSize();
    require(S == style.size(), "o lote usa as medidas do juiz");
    std::vector<std::vector<float>> before;
    for (std::size_t w = 0; w < batch.environmentCount(); ++w) before.push_back(featuresOf(style, batch.view(w).state->links));
    std::vector<float> zero(batch.environmentCount() * batch.actionSize(), 0.0f);
    batch.step(zero, zero);
    for (std::size_t w = 0; w < batch.environmentCount(); ++w) {
        const float* pair = batch.stylePairs().data() + w * 2 * S;
        const std::vector<float> now = featuresOf(style, batch.view(w).state->links);
        require(worst(std::vector<float>(pair, pair + S), before[w]) == 0.0f, "a primeira metade e o instante de antes");
        require(worst(std::vector<float>(pair + S, pair + 2 * S), now) == 0.0f, "a segunda metade e o de agora");
    }
}

} // namespace

void runStyleTests() {
    testStyleLayout();
    testStyleFrames();
    testStyleDatasetParity();
    testMotionDataset();
    testBatchStylePairs();
    std::cout << "Style tests passed\n";
}
