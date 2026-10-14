#include "Engine/RL/Environment/BodyTrainingBatch3D.hpp"

#include "Engine/Core/TaskScheduler.hpp"
#include "Engine/Materials/MaterialLibrary.hpp"
#include "Engine/Physics/PhysicsEngine3D.hpp"
#include "Engine/RL/Commands/CommandPilot3D.hpp"
#include "Engine/RL/Commands/ControlAdapter3D.hpp"
#include "Engine/RL/Imitation/ReferenceMotion3D.hpp"
#include "Engine/RL/Style/MotionDataset3D.hpp"
#include "Engine/RL/Style/StyleFeatures3D.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <numbers>
#include <random>
#include <stdexcept>
#include <thread>

namespace MatterEngine {
namespace {

constexpr float FixedStep = 1.0f / 120.0f;
constexpr float Gravity = 9.81f;
constexpr float Pi = std::numbers::pi_v<float>;
constexpr std::size_t RewardCount = BodyRewardTermNames3D.size();
constexpr std::size_t CostCount = 9;

std::uint64_t splitmix64(std::uint64_t& state) {
    std::uint64_t z = (state += 0x9E3779B97F4A7C15ull);
    z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ull;
    z = (z ^ (z >> 27)) * 0x94D049BB133111EBull;
    return z ^ (z >> 31);
}

bool finite(Vec3 v) { return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z); }

float uniform(std::mt19937_64& rng, PilotRange3D range) {
    if (!(range.maximum > range.minimum)) return range.minimum;
    return std::uniform_real_distribution<float>(range.minimum, range.maximum)(rng);
}

float gaussian(std::mt19937_64& rng, float sigma) {
    return sigma > 0.0f ? std::normal_distribution<float>(0.0f, sigma)(rng) : 0.0f;
}

float wrap(float angle) { return std::atan2(std::sin(angle), std::cos(angle)); }

std::array<float, CostCount> costArray(const BodyCosts3D& c) {
    return { c.work, c.torque, c.slip, c.impact, c.head, c.smooth, c.limits, c.contacts, c.saturation };
}

struct World {
    std::uint32_t scene = 0;
    std::uint32_t drill = 0;
    RagdollHandle3D ragdoll;
    Vec3 origin;
    std::mt19937_64 rng;
    // A variacao escondida: a massa (a vida toda do mundo; o corpo nao e
    // recriado), o atrito, a forca e o atraso (por episodio).
    float massScale = 1.0f;
    float friction = 1.0f;
    float strength = 1.0f;
    bool delayed = false;
    float weight = 0.0f;
    BodyContractState3D contract;
    BodyContractState3D criticScratch;
    PilotState3D pilot;
    ControlState3D control;
    PlayerAttributes3D player;
    BodyCommand3D command;
    std::vector<RagdollDriveTarget3D> targets;
    // O fim da reta da ultima decisao e quanto ele mudou (a suavidade), e a
    // mudanca da acao (as reversoes).
    std::vector<float> endTargets;
    std::vector<float> endChange;
    std::vector<float> actionDelta;
    // O que o juiz viu no fim da decisao anterior.
    std::vector<float> styleLast;
    BodyCostAccumulator3D costs;
    // O relogio do episodio (que pode comecar sorteado), as decisoes jogadas e o limite.
    std::uint32_t steps = 0;
    std::uint32_t played = 0;
    std::uint32_t limit = 1;
    int firstContactLink = -1;
    Vec3 start;
    // As medidas do episodio.
    double episodeReturn = 0.0;
    double velocityScore = 0.0;
    double facingScore = 0.0;
    double velocitySquares = 0.0;
    double facingSquares = 0.0;
    std::array<double, CostCount> costSum {};
    double absAction = 0.0;
    double clipped = 0.0;
    double clamped = 0.0;
    double reversal = 0.0;
    std::uint32_t touchdowns = 0;
    Vec3 lastWanted;
    float changeAt = -1.0f;
    double reactionSum = 0.0;
    std::uint32_t reactionCount = 0;
    std::uint32_t reactionOpened = 0;
};

// Um clipe de onde os comecos pelo acervo saem: os quadros dele e os de pe.
struct StartClip {
    ReferenceSegment3D segment;
    std::vector<std::uint32_t> standing;
};

struct Scene {
    std::unique_ptr<PhysicsScene3D> physics;
    std::vector<std::uint32_t> worlds;
    std::vector<BodyEpisode3D> finished;
};

// O que a nota, o fim e o piloto leem do corpo depois de uma decisao.
struct Reading {
    Vec3 com;
    Vec3 comVelocity;
    float chestHeading = 0.0f;
    bool valid = true;
};

} // namespace

const char* bodyEndReasonName3D(BodyEndReason3D reason) {
    switch (reason) {
        case BodyEndReason3D::None: return "nenhum";
        case BodyEndReason3D::FallContact: return "queda_contato";
        case BodyEndReason3D::FallHeight: return "queda_altura";
        case BodyEndReason3D::Numeric: return "numerico";
        case BodyEndReason3D::TimeLimit: return "tempo";
        case BodyEndReason3D::Boundary: return "limite";
    }
    return "?";
}

struct BodyTrainingBatch3D::Impl {
    BodyBatchSettings3D settings;
    BodyTask3D task;
    RLBody3D body;
    std::unique_ptr<BodyContract3D> contract;
    std::unique_ptr<StyleFeatures3D> style;
    std::shared_ptr<TaskScheduler> scheduler;
    std::unique_ptr<PhysicsEngine3D> engine;
    MaterialLibrary materials;
    std::vector<Scene> scenes;
    std::vector<World> worlds;
    std::vector<std::uint32_t> worldDrill;
    RLVectorLayout3D criticLayout;
    std::vector<float> observations;
    std::vector<float> critic;
    std::vector<float> rewards;
    std::vector<float> components;
    std::vector<float> terminalCritic;
    std::vector<std::uint8_t> terminated;
    std::vector<std::uint8_t> truncated;
    std::vector<float> stylePairs;
    std::vector<BodyEpisode3D> finished;
    // Os comecos pelo acervo: os clipes de treino e o peso acumulado deles.
    std::vector<StartClip> starts;
    std::vector<double> startCumulative;
    std::uint32_t posture = 0;
    std::uint32_t workers = 1;
    std::uint64_t physicsSteps = 0;
    float lastStepMilliseconds = 0.0f;

    [[nodiscard]] bool training() const { return !settings.evaluation; }
    [[nodiscard]] std::size_t actionSize() const { return contract->actionSize(); }
    [[nodiscard]] std::size_t observationSize() const { return contract->observationSize(); }
    [[nodiscard]] std::size_t criticSize() const { return criticLayout.size(); }
    [[nodiscard]] float decision() const { return task.contract.decisionSeconds; }

    void buildCriticLayout();
    void buildStarts();
    void assignDrills();
    void createWorld();
    void resetWorld(std::uint32_t index);
    [[nodiscard]] Reading read(const RagdollState3D& state) const;
    void nextCommand(World& world, const Reading& reading, float dt);
    void writeObservation(World& world, const RagdollState3D& state, float* actor, float* criticOut);
    void finishEpisode(Scene& scene, std::uint32_t index, BodyEndReason3D reason, const RagdollState3D* state);
    void stepScene(Scene& scene, std::span<const float> actions, std::span<const float> means);
};

void BodyTrainingBatch3D::Impl::buildCriticLayout() {
    // O critico ve a entrada sem ruido (a memoria e a que o ator viu) e o que
    // o ator nao ve: a variacao do mundo, o contato de cada link, o exercicio,
    // o modo da camera e a idade do pedido.
    criticLayout = contract->observationLayout();
    criticLayout.id = "matter.rl.critic";
    RLVectorLayout3D& c = criticLayout;
    c.channels.push_back({ "world.friction_scale", "x" });
    c.channels.push_back({ "body.mass_scale", "x" });
    c.channels.push_back({ "body.strength_scale", "x" });
    c.channels.push_back({ "body.action_delayed", "0/1" });
    for (const RagdollLinkDefinition3D& link : body.profile.links)
        c.channels.push_back({ link.id + ".ground_contact", "0/1" });
    for (const BodyDrill3D& drill : task.drills) c.channels.push_back({ "drill." + drill.id, "0/1" });
    c.channels.push_back({ "pilot.camera.fixed", "0/1" });
    c.channels.push_back({ "pilot.camera.follow", "0/1" });
    c.channels.push_back({ "pilot.camera.turns", "0/1" });
    c.channels.push_back({ "pilot.command_age", "s/3" });
}

void BodyTrainingBatch3D::Impl::buildStarts() {
    // Os clipes de treino (os reservados ficam de fora), pelo peso do grupo
    // dividido entre os clipes dele; so os quadros de pe.
    starts.clear();
    startCumulative.clear();
    if (!(task.initialState.referenceFraction > 0.0f)) return;
    const MotionDataset3D catalog = loadMotionCatalog3D(task.styleDataset);
    const float minimum = task.initialState.referenceMinimumPelvis * body.postures[posture].standingPelvisHeight;
    std::vector<std::uint32_t> perGroup(catalog.groups.size(), 0);
    std::vector<std::uint32_t> groupOf;
    for (const MotionClip3D& clip : catalog.clips) {
        if (clip.reserved) continue;
        const ReferenceClip3D request { clip.id, clip.path, clip.loops };
        ReferenceMotion3D motion =
            buildReferenceMotion3D(body, std::span<const ReferenceClip3D>(&request, 1), 1.0f / 120.0f, true);
        StartClip start;
        start.segment = std::move(motion.segments.front());
        for (std::uint32_t f = 0; f < start.segment.frames.size(); ++f)
            if (start.segment.frames[f].rootHeight >= minimum) start.standing.push_back(f);
        if (start.standing.empty()) continue;
        ++perGroup[clip.group];
        groupOf.push_back(clip.group);
        starts.push_back(std::move(start));
    }
    double total = 0.0;
    for (std::size_t s = 0; s < starts.size(); ++s) {
        total += catalog.groups[groupOf[s]].weight / static_cast<double>(perGroup[groupOf[s]]);
        startCumulative.push_back(total);
    }
    if (starts.empty() || !(total > 0.0)) throw std::runtime_error("lote do Corpo: nenhum quadro de pe no acervo");
}

void BodyTrainingBatch3D::Impl::assignDrills() {
    // As cotas: cada mundo fica no seu exercicio, na fatia do peso (a fatia
    // das decisoes e a fatia dos mundos, nao importa quanto dura o episodio).
    const std::size_t count = settings.environmentCount;
    worldDrill.assign(count, 0);
    if (!settings.drill.empty()) {
        const int forced = findBodyDrill3D(task, settings.drill);
        if (forced < 0) throw std::runtime_error("lote do Corpo: a tarefa nao tem o exercicio " + settings.drill);
        std::fill(worldDrill.begin(), worldDrill.end(), static_cast<std::uint32_t>(forced));
        return;
    }
    double total = 0.0;
    for (const BodyDrill3D& drill : task.drills) total += std::max(0.0f, drill.weight);
    for (std::size_t i = 0; i < count; ++i) {
        const double at = (static_cast<double>(i) + 0.5) / static_cast<double>(count) * total;
        double cumulative = 0.0;
        std::uint32_t chosen = static_cast<std::uint32_t>(task.drills.size() - 1);
        for (std::size_t d = 0; d < task.drills.size(); ++d) {
            cumulative += std::max(0.0f, task.drills[d].weight);
            if (at < cumulative) {
                chosen = static_cast<std::uint32_t>(d);
                break;
            }
        }
        worldDrill[i] = chosen;
    }
}

void BodyTrainingBatch3D::Impl::createWorld() {
    const std::uint32_t count = settings.environmentCount;
    const std::uint32_t perScene = std::max(1u, settings.environmentsPerScene);
    const std::uint32_t sceneCount = (count + perScene - 1) / perScene;
    scenes.resize(sceneCount);
    worlds.resize(count);
    std::uint64_t seedState = settings.seed ^ 0xA5A5A5A55A5A5A5Aull;
    for (std::uint32_t s = 0; s < sceneCount; ++s) {
        Scene& scene = scenes[s];
        const std::uint32_t first = s * perScene;
        const std::uint32_t last = std::min(count, first + perScene);
        PhysicsSceneSettings3D sceneSettings;
        sceneSettings.gravity = { 0.0f, 0.0f, -Gravity };
        sceneSettings.workerThreadCount = 1;
        sceneSettings.runJobsInline = true;
        sceneSettings.maximumBodies = 64u + 32u * perScene;
        sceneSettings.maximumBodyPairs = 2048u * perScene;
        sceneSettings.maximumContactConstraints = 1024u * perScene;
        sceneSettings.scratchBufferSizeBytes = (2u + perScene) * 1024u * 1024u;
        sceneSettings.windShelterDistanceMeters = 0.0f;
        scene.physics = engine->createScene(sceneSettings, materials);

        // O chao: uma caixa estatica grande sob os mundos da cena (topo em z = 0).
        const float spacing = task.worldSpacingMeters;
        const float halfLength = 0.5f * spacing * static_cast<float>(last - first) + std::max(20.0f, 0.5f * spacing);
        PhysicsShape3D shape;
        shape.type = PhysicsShapeType3D::Box;
        shape.halfExtents = { halfLength, std::max(20.0f, 0.5f * spacing), 0.5f };
        shape.materialId = task.groundMaterial;
        PhysicsBodyDefinition3D ground;
        ground.motionType = PhysicsMotionType3D::Static;
        ground.position = { 0.0f, 0.0f, -0.5f };
        ground.materialId = task.groundMaterial;
        (void)scene.physics->createBody(ground, std::span<const PhysicsShape3D>(&shape, 1));

        for (std::uint32_t index = first; index < last; ++index) {
            World& world = worlds[index];
            world.scene = s;
            world.drill = worldDrill[index];
            world.origin = { (static_cast<float>(index - first) - 0.5f * static_cast<float>(last - first - 1)) * spacing,
                0.0f, 0.0f };
            std::uint64_t worldSeed = seedState + index * 0x9E3779B97F4A7C15ull;
            world.rng.seed(splitmix64(worldSeed));
            world.massScale = training() ? uniform(world.rng, task.randomization.mass) : 1.0f;
            RagdollProfile3D profile = body.profile;
            profile.totalMassKg *= world.massScale;
            world.weight = profile.totalMassKg * Gravity;
            RagdollSpawnDefinition3D spawn;
            spawn.entityId = index + 1;
            spawn.pelvisPosition = world.origin + Vec3 { 0.0f, 0.0f, body.standingPelvisHeight + 0.05f };
            spawn.active = true;
            spawn.rigidityPercent = 0.0f;
            spawn.solverPositionIterations = settings.solverPositionIterations >= 0
                ? static_cast<std::uint32_t>(settings.solverPositionIterations) : body.solverPositionIterations;
            spawn.solverVelocityIterations = settings.solverVelocityIterations >= 0
                ? static_cast<std::uint32_t>(settings.solverVelocityIterations) : body.solverVelocityIterations;
            world.ragdoll = scene.physics->createRagdoll(profile, spawn);
            world.endTargets.assign(actionSize(), 0.0f);
            world.endChange.assign(actionSize(), 0.0f);
            world.actionDelta.assign(actionSize(), 0.0f);
            world.styleLast.assign(style->size(), 0.0f);
            scene.worlds.push_back(index);
        }
        scene.finished.reserve(perScene);
    }
}

Reading BodyTrainingBatch3D::Impl::read(const RagdollState3D& state) const {
    Reading reading;
    const RagdollProfile3D& profile = body.profile;
    const PhysicsBodyState3D& pelvis = state.links.front();
    if (!finite(pelvis.position) || !finite(pelvis.linearVelocity) || !std::isfinite(pelvis.orientation.w)) {
        reading.valid = false;
        return reading;
    }
    float massSum = 0.0f;
    for (std::size_t i = 0; i < profile.links.size(); ++i) {
        const float m = profile.links[i].massFraction;
        reading.com += state.links[i].position * m;
        reading.comVelocity += state.links[i].linearVelocity * m;
        massSum += m;
        if (!finite(state.links[i].position) || !finite(state.links[i].linearVelocity)) reading.valid = false;
    }
    reading.com = reading.com / std::max(1.0e-6f, massSum);
    reading.comVelocity = reading.comVelocity / std::max(1.0e-6f, massSum);
    const Quaternion chest = (state.links[body.upperChest].orientation
        * profile.links[body.upperChest].modelOrientation.conjugate()).normalized();
    const Vec3 forward = chest.rotate({ 1.0f, 0.0f, 0.0f });
    reading.chestHeading = std::atan2(forward.y, forward.x);
    return reading;
}

void BodyTrainingBatch3D::Impl::nextCommand(World& world, const Reading& reading, float dt) {
    const PilotScript3D& script = task.drills[world.drill].pilot;
    const ControlInput3D input =
        CommandPilot3D::update(script, world.pilot, PilotView3D { reading.com, reading.comVelocity }, dt, world.rng);
    world.command = ControlAdapter3D::command(task.control, input, world.control, world.player, dt);
    // A reacao: uma troca do pedido de velocidade abre a janela.
    const Vec3 wanted = world.command.moveVelocity;
    if (dt > 0.0f && std::hypot(wanted.x - world.lastWanted.x, wanted.y - world.lastWanted.y) > 0.5f) {
        world.changeAt = static_cast<float>(world.played) * decision();
        ++world.reactionOpened;
    }
    world.lastWanted = wanted;
}

void BodyTrainingBatch3D::Impl::writeObservation(World& world, const RagdollState3D& state, float* actor,
    float* criticOut) {
    // O critico primeiro, numa copia do estado (escrever a entrada zera o
    // resumo dos contatos e guarda o quadro da memoria).
    world.criticScratch = world.contract;
    contract->writeObservation(state, world.command, world.player, world.criticScratch, criticOut);
    const float noise = training() ? task.randomization.observationNoise : 0.0f;
    contract->writeObservation(state, world.command, world.player, world.contract, actor, noise, &world.rng);
    float* p = criticOut + observationSize();
    *p++ = world.friction;
    *p++ = world.massScale;
    *p++ = world.strength;
    *p++ = world.delayed ? 1.0f : 0.0f;
    float* contacts = p;
    std::fill(contacts, contacts + body.profile.links.size(), 0.0f);
    for (const RagdollContactPoint3D& contact : state.contacts)
        if (contact.otherMotion == RagdollContactMotion3D::Static && contact.linkIndex < body.profile.links.size())
            contacts[contact.linkIndex] = 1.0f;
    p += body.profile.links.size();
    for (std::size_t d = 0; d < task.drills.size(); ++d) *p++ = d == world.drill ? 1.0f : 0.0f;
    *p++ = world.pilot.cameraMode == PilotCameraMode3D::Fixed ? 1.0f : 0.0f;
    *p++ = world.pilot.cameraMode == PilotCameraMode3D::Follow ? 1.0f : 0.0f;
    *p++ = world.pilot.cameraMode == PilotCameraMode3D::Turns ? 1.0f : 0.0f;
    *p++ = std::min(1.0f, (world.pilot.clock - world.pilot.lastChange) / 3.0f);
}

void BodyTrainingBatch3D::Impl::resetWorld(std::uint32_t index) {
    World& world = worlds[index];
    PhysicsScene3D& physics = *scenes[world.scene].physics;
    const BodyTask3D::InitialState& start = task.initialState;
    const RLPosture3D& pose = body.postures[posture];

    const float yaw = start.randomYaw ? std::uniform_real_distribution<float>(-Pi, Pi)(world.rng) : 0.0f;
    std::vector<PhysicsBodyState3D> links;
    // O comeco pelo acervo (so no treino): o movimento do clipe, para o pedido seguir.
    bool fromClip = false;
    Vec3 clipVelocity;
    float clipChest = 0.0f;
    if (training() && !starts.empty()
        && std::uniform_real_distribution<float>(0.0f, 1.0f)(world.rng) < start.referenceFraction) {
        const double roll = std::uniform_real_distribution<double>(0.0, startCumulative.back())(world.rng);
        const auto at = static_cast<std::size_t>(
            std::upper_bound(startCumulative.begin(), startCumulative.end(), roll) - startCumulative.begin());
        const StartClip& clip = starts[std::min(at, starts.size() - 1)];
        const std::uint32_t frameIndex =
            clip.standing[std::uniform_int_distribution<std::size_t>(0, clip.standing.size() - 1)(world.rng)];
        const ReferenceFrame3D& frame = clip.segment.frames[frameIndex];
        // O pe mais baixo 2 mm acima do chao (no voo, como o clipe).
        const float lowest = rlBodyLowestFootPoint3D(body, referencePose3D(body, frame, world.origin, yaw));
        referenceLinkStates3D(body, frame, world.origin + Vec3 { 0.0f, 0.0f, std::max(0.0f, 0.002f - lowest) }, yaw,
            links);
        clipVelocity = Quaternion::fromAxisAngle({ 0.0f, 0.0f, 1.0f }, yaw).rotate(frame.rootVelocity);
        clipChest = yaw + frame.chestHeading;
        fromClip = true;
    } else {
        // De pe na postura, com ruido, girado no rumo sorteado e apoiado no chao
        // (o ponto mais baixo dos pes 2 mm acima dele).
        std::vector<Vec3> coordinates = pose.jointCoordinates;
        std::vector<Vec3> velocities(coordinates.size());
        for (const RLBodyDof3D& dof : body.dofs) {
            Vec3& q = coordinates[dof.linkIndex];
            Vec3& qd = velocities[dof.linkIndex];
            float* value = dof.axis == RagdollAxis3D::Twist ? &q.x : dof.axis == RagdollAxis3D::Swing1 ? &q.y : &q.z;
            float* rate = dof.axis == RagdollAxis3D::Twist ? &qd.x : dof.axis == RagdollAxis3D::Swing1 ? &qd.y : &qd.z;
            *value = std::clamp(*value + gaussian(world.rng, start.jointNoiseRadians), dof.minimumRadians,
                dof.maximumRadians);
            *rate = gaussian(world.rng, start.jointVelocityNoise);
        }
        const Quaternion heading = Quaternion::fromAxisAngle({ 0.0f, 0.0f, 1.0f }, yaw);
        const Quaternion root = (heading * body.profile.links.front().modelOrientation * pose.rootRotation).normalized();
        const RLBodyPose3D probe = rlBodyPose3D(body.profile, world.origin, root, coordinates);
        const float lift = 0.002f - rlBodyLowestFootPoint3D(body, probe);
        const Vec3 rootVelocity { gaussian(world.rng, start.rootVelocityNoise),
            gaussian(world.rng, start.rootVelocityNoise), 0.0f };
        const Vec3 rootSpin { gaussian(world.rng, start.rootAngularVelocityNoise),
            gaussian(world.rng, start.rootAngularVelocityNoise), gaussian(world.rng, start.rootAngularVelocityNoise) };
        rlBodyLinkStates3D(body.profile, world.origin + Vec3 { 0.0f, 0.0f, lift }, root, rootVelocity, rootSpin,
            coordinates, velocities, links);
    }
    physics.resetRagdollState(world.ragdoll, links);

    // O mundo deste episodio.
    const BodyTask3D::Randomization& random = task.randomization;
    world.friction = training() ? uniform(world.rng, random.friction) : 1.0f;
    world.strength = training() ? uniform(world.rng, random.strength) : 1.0f;
    world.delayed = training() && std::uniform_real_distribution<float>(0.0f, 1.0f)(world.rng) < random.actionDelayProbability;
    physics.setRagdollFrictionScale(world.ragdoll, world.friction);
    world.player = task.player;
    const BodyDrill3D& drill = task.drills[world.drill];
    const float seconds = settings.evaluation ? settings.evaluationEpisodeSeconds
        : drill.episodeSeconds > 0.0f ? drill.episodeSeconds : task.episodeSeconds;
    const float stretch = training() ? std::uniform_real_distribution<float>(0.8f, 1.2f)(world.rng) : 1.0f;
    world.limit = std::max(1u, static_cast<std::uint32_t>(std::lround(seconds * stretch / decision())));
    world.steps = 0;
    world.played = 0;

    // O contrato parte do corpo de agora; os motores seguram a pose dele.
    const RagdollState3D* state = physics.ragdollStateView(world.ragdoll);
    if (!state) throw std::runtime_error("lote do Corpo: o boneco sumiu da cena");
    contract->reset(world.contract, *state);
    contract->writeTargetsAt(world.contract, 1.0f, world.player, world.targets, world.strength);
    physics.setRagdollActiveDriveTargets(world.ragdoll, world.targets, false);
    world.endTargets = world.contract.rampTo;
    std::fill(world.endChange.begin(), world.endChange.end(), 0.0f);
    std::fill(world.actionDelta.begin(), world.actionDelta.end(), 0.0f);
    world.firstContactLink = -1;
    world.start = state->links.front().position;
    style->write(state->links, world.player.heightMeters / 1.8f, world.styleLast.data());

    world.episodeReturn = 0.0;
    world.velocityScore = world.facingScore = world.velocitySquares = world.facingSquares = 0.0;
    world.costSum = {};
    world.absAction = world.clipped = world.clamped = world.reversal = 0.0;
    world.touchdowns = 0;
    world.changeAt = -1.0f;
    world.reactionSum = 0.0;
    world.reactionCount = world.reactionOpened = 0;

    // O piloto: a camera comeca no rumo do peito; no comeco pelo acervo, o
    // primeiro pedido segue o movimento do clipe.
    const Reading reading = read(*state);
    world.control = {};
    CommandPilot3D::start(drill.pilot, world.pilot, reading.com, fromClip ? clipChest : reading.chestHeading, world.rng);
    if (fromClip) CommandPilot3D::follow(drill.pilot, world.pilot, clipVelocity, world.rng);
    nextCommand(world, reading, 0.0f);
}

void BodyTrainingBatch3D::Impl::finishEpisode(Scene& scene, std::uint32_t index, BodyEndReason3D reason,
    const RagdollState3D* state) {
    const World& world = worlds[index];
    BodyEpisode3D episode;
    episode.environment = index;
    episode.drill = world.drill;
    episode.reason = reason;
    episode.firstContactLink = world.firstContactLink;
    episode.steps = world.played;
    episode.seconds = static_cast<float>(world.played) * decision();
    episode.episodeReturn = static_cast<float>(world.episodeReturn);
    const double n = std::max(1u, world.played);
    episode.velocityScore = static_cast<float>(world.velocityScore / n);
    episode.facingScore = static_cast<float>(world.facingScore / n);
    episode.velocityErrorRms = static_cast<float>(std::sqrt(world.velocitySquares / n));
    episode.facingErrorRms = static_cast<float>(std::sqrt(world.facingSquares / n));
    episode.commandChanges = world.reactionOpened;
    if (world.reactionOpened > 0)
        episode.reactionResolved = static_cast<float>(world.reactionCount) / static_cast<float>(world.reactionOpened);
    if (world.reactionCount > 0)
        episode.reactionSeconds = static_cast<float>(world.reactionSum / world.reactionCount);
    BodyCosts3D& c = episode.costs;
    float* costs[CostCount] = { &c.work, &c.torque, &c.slip, &c.impact, &c.head, &c.smooth, &c.limits, &c.contacts,
        &c.saturation };
    for (std::size_t k = 0; k < CostCount; ++k) *costs[k] = static_cast<float>(world.costSum[k] / n);
    const double actions = n * static_cast<double>(std::max<std::size_t>(1, actionSize()));
    episode.meanAbsAction = static_cast<float>(world.absAction / actions);
    episode.actionClipped = static_cast<float>(world.clipped / actions);
    episode.targetClamped = static_cast<float>(world.clamped / actions);
    episode.meanReversal = static_cast<float>(world.reversal / actions);
    episode.cadence = episode.seconds > 0.0f ? static_cast<float>(world.touchdowns) / episode.seconds : 0.0f;
    if (state && finite(state->links.front().position)) {
        const Vec3 p = state->links.front().position;
        episode.distance = std::hypot(p.x - world.start.x, p.y - world.start.y);
    }
    scene.finished.push_back(episode);
}

void BodyTrainingBatch3D::Impl::stepScene(Scene& scene, std::span<const float> actions, std::span<const float> means) {
    PhysicsScene3D& physics = *scene.physics;
    const std::size_t A = actionSize();
    scene.finished.clear();
    for (const std::uint32_t index : scene.worlds) {
        World& world = worlds[index];
        const std::span<const float> action = actions.subspan(index * A, A);
        // A telemetria da acao: presa no limite, alvo preso no curso, tamanho e reversao.
        for (std::size_t d = 0; d < A; ++d) {
            const RLBodyDof3D& dof = body.dofs[d];
            const float a = std::clamp(action[d], -RLActionLimit, RLActionLimit);
            if (std::abs(action[d]) >= RLActionLimit) world.clipped += 1.0;
            const float raw = dof.baseRadians + dof.actionScale * a;
            if (raw < dof.minimumRadians || raw > dof.maximumRadians) world.clamped += 1.0;
            world.absAction += std::abs(a);
            const float delta = a - world.contract.previousAction[d];
            world.reversal += std::max(0.0f, -delta * world.actionDelta[d]);
            world.actionDelta[d] = delta;
        }
        contract->beginDecision(world.contract, action, means.subspan(index * A, A));
        if (const RagdollState3D* state = physics.ragdollStateView(world.ragdoll)) world.costs.begin(*state, body);
        world.costs.setTargetChange(world.endTargets, world.endChange, world.contract.rampTo);
        for (std::size_t d = 0; d < A; ++d) {
            world.endChange[d] = world.contract.rampTo[d] - world.endTargets[d];
            world.endTargets[d] = world.contract.rampTo[d];
        }
    }

    const std::uint32_t substeps = task.contract.substepsPerDecision;
    for (std::uint32_t sub = 0; sub < substeps; ++sub) {
        for (const std::uint32_t index : scene.worlds) {
            World& world = worlds[index];
            // O atraso de um passo de fisica: a reta inteira um passo depois.
            const float u = static_cast<float>(world.delayed ? sub : sub + 1) / static_cast<float>(substeps);
            contract->writeTargetsAt(world.contract, u, world.player, world.targets, world.strength);
            physics.setRagdollActiveDriveTargets(world.ragdoll, world.targets, false);
        }
        physics.simulate(FixedStep);
        for (const std::uint32_t index : scene.worlds) {
            World& world = worlds[index];
            const RagdollState3D* state = physics.ragdollStateView(world.ragdoll);
            if (!state) continue;
            contract->accumulateContacts(world.contract, *state, world.weight, FixedStep);
            world.costs.addSubstep(body, *state, FixedStep, world.weight);
            // A queda por contato em qualquer passo: um link que nao e apoio no chao.
            if (!task.termination.nonSupportContact || world.firstContactLink >= 0) continue;
            for (const RagdollContactPoint3D& contact : state->contacts) {
                if (contact.otherMotion != RagdollContactMotion3D::Static) continue;
                if (contact.linkIndex < body.supportLink.size() && body.supportLink[contact.linkIndex]) continue;
                world.firstContactLink = static_cast<int>(contact.linkIndex);
                break;
            }
        }
    }

    const std::size_t O = observationSize();
    const std::size_t C = criticSize();
    const RLPosture3D& pose = body.postures[posture];
    for (const std::uint32_t index : scene.worlds) {
        World& world = worlds[index];
        ++world.steps;
        ++world.played;
        const RagdollState3D* state = physics.ragdollStateView(world.ragdoll);
        float* actor = observations.data() + index * O;
        float* criticOut = critic.data() + index * C;
        float* parts = components.data() + index * RewardCount;
        Reading reading;
        if (state) reading = read(*state);
        else reading.valid = false;

        BodyEndReason3D reason = BodyEndReason3D::None;
        if (!reading.valid) {
            reason = BodyEndReason3D::Numeric;
        } else {
            const PhysicsBodyState3D& pelvis = state->links.front();
            const bool low = state->links[body.head].position.z
                    < pose.standingHeadHeight * task.termination.minimumHeadHeightFraction
                || pelvis.position.z < pose.standingPelvisHeight * task.termination.minimumPelvisHeightFraction;
            const float dx = pelvis.position.x - world.origin.x, dy = pelvis.position.y - world.origin.y;
            if (world.firstContactLink >= 0) reason = BodyEndReason3D::FallContact;
            else if (low) reason = BodyEndReason3D::FallHeight;
            else if (world.steps >= world.limit) reason = BodyEndReason3D::TimeLimit;
            else if (dx * dx + dy * dy
                > task.termination.maximumDistanceFromStart * task.termination.maximumDistanceFromStart)
                reason = BodyEndReason3D::Boundary;
        }
        const bool fell = reason == BodyEndReason3D::FallContact || reason == BodyEndReason3D::FallHeight;
        float reward = 0.0f;
        if (reading.valid) {
            // A nota usa o comando que valeu nesta decisao.
            const BodyDrill3D& drill = task.drills[world.drill];
            const BodyCosts3D costs = world.costs.finish(decision(), world.weight);
            const Vec3 wanted = world.command.moveVelocity;
            const float velocityScore = bodyVelocityScore3D(task.reward, reading.comVelocity, wanted);
            const float facingScore = bodyFacingScore3D(task.reward, reading.chestHeading, world.command.faceHeading);
            reward = bodyReward3D(task.reward, drill.task, velocityScore, facingScore, costs, fell, decision(), parts);
            const float velocityError =
                std::hypot(reading.comVelocity.x - wanted.x, reading.comVelocity.y - wanted.y);
            const float facingError = wrap(world.command.faceHeading - reading.chestHeading);
            world.velocityScore += velocityScore;
            world.facingScore += facingScore;
            world.velocitySquares += velocityError * velocityError;
            world.facingSquares += facingError * facingError;
            const std::array<float, CostCount> values = costArray(costs);
            for (std::size_t k = 0; k < CostCount; ++k) world.costSum[k] += values[k];
            const float tolerance = task.reward.velocityToleranceAbsolute
                + task.reward.velocityToleranceRelative * std::hypot(wanted.x, wanted.y);
            if (world.changeAt >= 0.0f && velocityError <= tolerance) {
                world.reactionSum += static_cast<float>(world.played) * decision() - world.changeAt;
                ++world.reactionCount;
                world.changeAt = -1.0f;
            }
            world.touchdowns += (world.contract.touchdown[0] ? 1u : 0u) + (world.contract.touchdown[1] ? 1u : 0u);
            // O par do juiz: o instante de antes e o de agora.
            const std::size_t S = style->size();
            float* pair = stylePairs.data() + index * 2 * S;
            std::copy(world.styleLast.begin(), world.styleLast.end(), pair);
            style->write(state->links, world.player.heightMeters / 1.8f, pair + S);
            std::copy(pair + S, pair + 2 * S, world.styleLast.begin());
            // O comando da proxima decisao e a entrada.
            nextCommand(world, reading, decision());
            writeObservation(world, *state, actor, criticOut);
        } else {
            std::fill(parts, parts + RewardCount, 0.0f);
            parts[static_cast<std::size_t>(BodyRewardTerm3D::Fall)] = task.reward.fall;
            reward = -task.reward.fall;
            std::fill_n(stylePairs.data() + index * 2 * style->size(), 2 * style->size(), 0.0f);
            std::fill(actor, actor + O, 0.0f);
            std::fill(criticOut, criticOut + C, 0.0f);
        }
        rewards[index] = reward;
        world.episodeReturn += reward;
        terminated[index] = fell || reason == BodyEndReason3D::Numeric ? 1u : 0u;
        truncated[index] = reason == BodyEndReason3D::TimeLimit || reason == BodyEndReason3D::Boundary ? 1u : 0u;
        if (reason == BodyEndReason3D::None) continue;

        std::copy(criticOut, criticOut + C, terminalCritic.data() + index * C);
        finishEpisode(scene, index, reason, state);
        resetWorld(index);
        writeObservation(world, *physics.ragdollStateView(world.ragdoll), actor, criticOut);
    }
}

BodyTrainingBatch3D::BodyTrainingBatch3D(const BodyBatchSettings3D& settings) : m_impl(std::make_unique<Impl>()) {
    Impl& impl = *m_impl;
    if (settings.environmentCount == 0) throw std::invalid_argument("lote do Corpo sem mundos");
    impl.settings = settings;
    impl.task = loadBodyTask3D(settings.taskPath);
    impl.body = loadRLBody3D(impl.task.bodyPath);
    impl.contract = std::make_unique<BodyContract3D>(impl.body, impl.task.contract);
    impl.style = std::make_unique<StyleFeatures3D>(impl.body);
    const int posture = findRLPosture3D(impl.body, impl.task.initialState.posture);
    if (posture < 0) throw std::runtime_error("lote do Corpo: o corpo nao tem a postura " + impl.task.initialState.posture);
    impl.posture = static_cast<std::uint32_t>(posture);
    impl.buildStarts();
    impl.buildCriticLayout();
    impl.assignDrills();

    const std::uint32_t hardware = std::max(1u, std::thread::hardware_concurrency());
    const std::uint32_t threads = settings.workerThreads > 0 ? settings.workerThreads : std::max(1u, hardware - 2);
    impl.workers = std::max(1u, threads);
    // A thread que chama step() tambem trabalha no parallelFor.
    impl.scheduler = std::make_shared<TaskScheduler>(TaskSchedulerSettings { std::max(1u, impl.workers - 1) });
    impl.engine = std::make_unique<PhysicsEngine3D>(impl.scheduler);

    const std::size_t N = settings.environmentCount;
    impl.observations.assign(N * impl.observationSize(), 0.0f);
    impl.critic.assign(N * impl.criticSize(), 0.0f);
    impl.terminalCritic.assign(N * impl.criticSize(), 0.0f);
    impl.rewards.assign(N, 0.0f);
    impl.components.assign(N * RewardCount, 0.0f);
    impl.terminated.assign(N, 0);
    impl.truncated.assign(N, 0);
    impl.stylePairs.assign(N * 2 * impl.style->size(), 0.0f);
    impl.createWorld();
    reset(settings.seed);
}

BodyTrainingBatch3D::~BodyTrainingBatch3D() = default;

std::size_t BodyTrainingBatch3D::environmentCount() const { return m_impl->worlds.size(); }
std::size_t BodyTrainingBatch3D::observationSize() const { return m_impl->observationSize(); }
std::size_t BodyTrainingBatch3D::criticObservationSize() const { return m_impl->criticSize(); }
std::size_t BodyTrainingBatch3D::actionSize() const { return m_impl->actionSize(); }
std::size_t BodyTrainingBatch3D::rewardComponentCount() const { return RewardCount; }
std::uint32_t BodyTrainingBatch3D::workerThreads() const { return m_impl->workers; }
float BodyTrainingBatch3D::policyStepSeconds() const { return m_impl->decision(); }
const RLVectorLayout3D& BodyTrainingBatch3D::observationLayout() const { return m_impl->contract->observationLayout(); }
const RLVectorLayout3D& BodyTrainingBatch3D::criticObservationLayout() const { return m_impl->criticLayout; }
const RLVectorLayout3D& BodyTrainingBatch3D::actionLayout() const { return m_impl->contract->actionLayout(); }
const RLBody3D& BodyTrainingBatch3D::body() const { return m_impl->body; }
const BodyTask3D& BodyTrainingBatch3D::task() const { return m_impl->task; }
const BodyContract3D& BodyTrainingBatch3D::contract() const { return *m_impl->contract; }
std::span<const std::uint32_t> BodyTrainingBatch3D::worldDrills() const { return m_impl->worldDrill; }
std::size_t BodyTrainingBatch3D::styleSize() const { return m_impl->style->size(); }
const RLVectorLayout3D& BodyTrainingBatch3D::styleLayout() const { return m_impl->style->layout(); }
std::span<const float> BodyTrainingBatch3D::stylePairs() const { return m_impl->stylePairs; }

void BodyTrainingBatch3D::updateTask(const BodyTask3D& task) {
    Impl& impl = *m_impl;
    // Confere antes de mudar: uma tarefa recusada deixa o lote como estava.
    if (task.bodyPath != impl.task.bodyPath) throw std::runtime_error("lote do Corpo: o corpo nao muda com o treino andando");
    const BodyContractSettings3D& a = task.contract;
    const BodyContractSettings3D& b = impl.task.contract;
    if (a.substepsPerDecision != b.substepsPerDecision || a.historyDecisions != b.historyDecisions
        || a.velocityTarget != b.velocityTarget || a.velocityFraction != b.velocityFraction
        || a.maximumTargetVelocity != b.maximumTargetVelocity)
        throw std::runtime_error("lote do Corpo: o contrato nao muda com o treino andando");
    if (task.initialState.posture != impl.task.initialState.posture)
        throw std::runtime_error("lote do Corpo: a postura do comeco nao muda com o treino andando");
    bool same = task.drills.size() == impl.task.drills.size();
    for (std::size_t d = 0; same && d < task.drills.size(); ++d) same = task.drills[d].id == impl.task.drills[d].id;
    if (!same) throw std::runtime_error("lote do Corpo: os exercicios nao mudam de lista com o treino andando");
    impl.task.reward = task.reward;
    impl.task.termination = task.termination;
    impl.task.randomization = task.randomization;
    impl.task.initialState = task.initialState;
    impl.task.control = task.control;
    impl.task.episodeSeconds = task.episodeSeconds;
    for (std::size_t d = 0; d < task.drills.size(); ++d) {
        BodyDrill3D& drill = impl.task.drills[d];
        drill.pilot = task.drills[d].pilot;
        drill.task = task.drills[d].task;
        drill.episodeSeconds = task.drills[d].episodeSeconds;
        drill.displayName = task.drills[d].displayName;
    }
    impl.task.hash = task.hash;
}

void BodyTrainingBatch3D::reset(std::uint64_t seed) {
    Impl& impl = *m_impl;
    std::uint64_t state = seed;
    for (World& world : impl.worlds) world.rng.seed(splitmix64(state));
    const std::size_t O = impl.observationSize(), C = impl.criticSize();
    for (std::uint32_t i = 0; i < impl.worlds.size(); ++i) {
        impl.resetWorld(i);
        World& world = impl.worlds[i];
        // No treino, o relogio de cada episodio comeca sorteado: sem isso os
        // mundos que aguentam ate o fim terminam todos no mesmo passo.
        if (impl.training()) world.steps = std::uniform_int_distribution<std::uint32_t>(0, world.limit - 1)(world.rng);
        const RagdollState3D* view = impl.scenes[world.scene].physics->ragdollStateView(world.ragdoll);
        impl.writeObservation(world, *view, impl.observations.data() + i * O, impl.critic.data() + i * C);
    }
    std::fill(impl.rewards.begin(), impl.rewards.end(), 0.0f);
    std::fill(impl.components.begin(), impl.components.end(), 0.0f);
    std::fill(impl.stylePairs.begin(), impl.stylePairs.end(), 0.0f);
    std::fill(impl.terminated.begin(), impl.terminated.end(), 0);
    std::fill(impl.truncated.begin(), impl.truncated.end(), 0);
    impl.finished.clear();
}

void BodyTrainingBatch3D::step(std::span<const float> actions, std::span<const float> means) {
    Impl& impl = *m_impl;
    const std::size_t expected = impl.worlds.size() * impl.actionSize();
    if (actions.size() != expected || means.size() != expected)
        throw std::invalid_argument("lote do Corpo: dimensao das acoes incompativel");
    for (std::size_t i = 0; i < expected; ++i)
        if (!std::isfinite(actions[i]) || !std::isfinite(means[i]))
            throw std::invalid_argument("lote do Corpo: acao nao finita");
    const auto begin = std::chrono::steady_clock::now();
    struct Context {
        Impl* impl;
        std::span<const float> actions;
        std::span<const float> means;
    } context { &impl, actions, means };
    impl.scheduler->parallelFor(impl.scenes.size(), 1,
        [](std::size_t first, std::size_t last, void* raw) noexcept {
            auto* ctx = static_cast<Context*>(raw);
            for (std::size_t s = first; s < last; ++s)
                ctx->impl->stepScene(ctx->impl->scenes[s], ctx->actions, ctx->means);
        },
        &context);
    impl.finished.clear();
    for (const Scene& scene : impl.scenes)
        impl.finished.insert(impl.finished.end(), scene.finished.begin(), scene.finished.end());
    impl.physicsSteps += impl.worlds.size() * impl.task.contract.substepsPerDecision;
    impl.lastStepMilliseconds =
        std::chrono::duration<float, std::milli>(std::chrono::steady_clock::now() - begin).count();
}

std::span<const float> BodyTrainingBatch3D::observations() const { return m_impl->observations; }
std::span<const float> BodyTrainingBatch3D::criticObservations() const { return m_impl->critic; }
std::span<const float> BodyTrainingBatch3D::rewards() const { return m_impl->rewards; }
std::span<const float> BodyTrainingBatch3D::rewardComponents() const { return m_impl->components; }
std::span<const std::uint8_t> BodyTrainingBatch3D::terminated() const { return m_impl->terminated; }
std::span<const std::uint8_t> BodyTrainingBatch3D::truncated() const { return m_impl->truncated; }
std::span<const float> BodyTrainingBatch3D::terminalCriticObservations() const { return m_impl->terminalCritic; }
std::span<const BodyEpisode3D> BodyTrainingBatch3D::finishedEpisodes() const { return m_impl->finished; }

BodyWorldView3D BodyTrainingBatch3D::view(std::size_t environment) const {
    const Impl& impl = *m_impl;
    BodyWorldView3D view;
    if (environment >= impl.worlds.size()) return view;
    const World& world = impl.worlds[environment];
    view.state = impl.scenes[world.scene].physics->ragdollStateView(world.ragdoll);
    view.origin = world.origin;
    view.drill = world.drill;
    view.steps = world.played;
    view.command = world.command;
    view.player = world.player;
    view.hasTarget = world.pilot.hasTarget;
    view.target = world.pilot.target;
    view.targetStop = world.pilot.stopAtTarget;
    if (view.state) {
        const Reading reading = impl.read(*view.state);
        view.velocity = reading.comVelocity;
        view.chestHeading = reading.chestHeading;
    }
    return view;
}

std::vector<float> BodyTrainingBatch3D::actionRadiansPerUnit() const {
    std::vector<float> result;
    for (const RLBodyDof3D& dof : m_impl->body.dofs) result.push_back(dof.actionScale);
    return result;
}

float BodyTrainingBatch3D::lastStepMilliseconds() const { return m_impl->lastStepMilliseconds; }
std::uint64_t BodyTrainingBatch3D::physicsStepCount() const { return m_impl->physicsSteps; }

} // namespace MatterEngine
