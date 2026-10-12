#include "Engine/RL/Environment/TrainingBatch3D.hpp"

#include "Engine/Core/TaskScheduler.hpp"
#include "Engine/Materials/MaterialLibrary.hpp"
#include "Engine/Physics/PhysicsEngine3D.hpp"

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
// Distancia entre os mundos de uma cena: longe o bastante para nunca se
// tocarem dentro do raio em que o episodio vale.
constexpr float EnvironmentSpacing = 16.0f;
constexpr std::size_t KeyBodyCount = 5;
constexpr std::size_t RewardCount = TrainingRewardComponentNames3D.size();
// Os canais do critico alem da observacao: atrito, massa, forca, atraso, o
// empurrao (3) e um contato com o chao por link.
constexpr std::size_t CriticWorldChannels = 7;

std::uint64_t splitmix64(std::uint64_t& state) {
    std::uint64_t z = (state += 0x9E3779B97F4A7C15ull);
    z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ull;
    z = (z ^ (z >> 27)) * 0x94D049BB133111EBull;
    return z ^ (z >> 31);
}

bool finite(Vec3 v) { return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z); }

float uniform(std::mt19937_64& rng, TrainingRange3D range) {
    if (!(range.maximum > range.minimum)) return range.minimum;
    return std::uniform_real_distribution<float>(range.minimum, range.maximum)(rng);
}

float gaussian(std::mt19937_64& rng, float sigma) {
    if (!(sigma > 0.0f)) return 0.0f;
    return std::normal_distribution<float>(0.0f, sigma)(rng);
}

struct Environment {
    std::uint32_t scene = 0;
    RagdollHandle3D ragdoll;
    Vec3 origin;
    std::mt19937_64 rng;
    float massScale = 1.0f;
    // O episodio.
    std::uint32_t steps = 0;
    std::vector<float> previousAction;
    std::vector<float> action;
    std::vector<RagdollDriveTarget3D> targets;
    std::vector<RagdollDriveTarget3D> previousTargets;
    std::array<Vec3, 2> footStart {};
    float friction = 1.0f;
    float strength = 1.0f;
    bool delayed = false;
    // A queda por contato vista em qualquer subpasso (o estado publicado so
    // tem os contatos do ultimo).
    int firstContactLink = -1;
    // Empurrao (H2): tempo ate o proximo, quanto falta do atual, a forca.
    float pushTimer = 0.0f;
    float pushRemaining = 0.0f;
    Vec3 pushForce;
    std::uint32_t pushLink = 0;
    // Acumuladores do resumo.
    double episodeReturn = 0.0;
    double comSum[2] {};
    double comSquares[2] {};
    float footDriftMax = 0.0f;
    double actionRateSum = 0.0;
    double effortSum = 0.0;
    double jointSpeedSum = 0.0;
    double uprightSum = 0.0;
};

struct Scene {
    std::unique_ptr<PhysicsScene3D> physics;
    std::vector<std::uint32_t> environments;
    std::vector<TrainingEpisode3D> finished;
};

// O que a observacao e a recompensa leem de um corpo, uma vez por passo.
struct BodyReading {
    Quaternion pelvisFrame;
    Quaternion headingInverse;
    Vec3 com;
    Vec3 comVelocity;
    std::array<Vec3, 2> footCenter {};
    std::array<bool, 2> footContact {};
    std::array<float, 2> footForce {};
    std::uint32_t groundMask = 0;
    bool valid = true;
};

} // namespace

const char* trainingEndReasonName3D(TrainingEndReason3D reason) {
    switch (reason) {
        case TrainingEndReason3D::None: return "nenhum";
        case TrainingEndReason3D::FallContact: return "queda_contato";
        case TrainingEndReason3D::FallHeight: return "queda_altura";
        case TrainingEndReason3D::Numeric: return "numerico";
        case TrainingEndReason3D::TimeLimit: return "tempo";
        case TrainingEndReason3D::Boundary: return "limite";
    }
    return "?";
}

struct TrainingBatch3D::Impl {
    TrainingBatchSettings3D settings;
    RLBody3D body;
    TrainingTask3D task;
    std::shared_ptr<TaskScheduler> scheduler;
    std::unique_ptr<PhysicsEngine3D> engine;
    MaterialLibrary materials;
    std::vector<Scene> scenes;
    std::vector<Environment> environments;
    RLVectorLayout3D observationLayout;
    RLVectorLayout3D criticLayout;
    RLVectorLayout3D actionLayout;
    std::vector<float> observations;
    std::vector<float> critic;
    std::vector<float> rewards;
    std::vector<float> components;
    std::vector<float> terminalCritic;
    std::vector<std::uint8_t> terminated;
    std::vector<std::uint8_t> truncated;
    std::vector<TrainingEpisode3D> finished;
    std::vector<std::uint32_t> pushLinks;
    std::array<std::uint32_t, KeyBodyCount> keyBodies {};
    std::uint32_t workers = 1;
    std::uint32_t maximumSteps = 600;
    float lastStepMilliseconds = 0.0f;
    std::uint64_t physicsSteps = 0;

    [[nodiscard]] std::size_t actionSize() const { return body.dofs.size(); }
    [[nodiscard]] std::size_t observationSize() const { return observationLayout.size(); }
    [[nodiscard]] std::size_t criticSize() const { return criticLayout.size(); }
    [[nodiscard]] bool training() const { return !settings.evaluation; }

    void buildLayouts();
    void createWorld();
    void configureEpisodeLength();
    void resetEnvironment(std::uint32_t index);
    void writeTargets(Environment& env, std::span<const float> action);
    [[nodiscard]] BodyReading read(const RagdollState3D& state) const;
    void writeObservation(const Environment& env, const RagdollState3D& state, const BodyReading& reading,
        float* actor, float* criticOut, bool noise);
    float reward(Environment& env, const RagdollState3D& state, const BodyReading& reading, bool fell,
        float* componentsOut);
    void finishEpisode(Scene& scene, std::uint32_t index, TrainingEndReason3D reason, const BodyReading& reading);
    void stepScene(Scene& scene, std::span<const float> actions);
};

void TrainingBatch3D::Impl::buildLayouts() {
    const auto vec3 = [](RLVectorLayout3D& layout, const std::string& name, const char* unit) {
        for (const char* axis : { ".x", ".y", ".z" }) layout.channels.push_back({ name + axis, unit });
    };
    RLVectorLayout3D& o = observationLayout;
    o.id = "matter.rl.observation";
    o.version = 1;
    o.channels.push_back({ "pelvis.height_error", "m/0.3" });
    vec3(o, "pelvis.gravity_local", "unit");
    vec3(o, "pelvis.linear_velocity_heading", "m/s/2");
    vec3(o, "pelvis.angular_velocity_local", "rad/s/4");
    for (const RLBodyDof3D& dof : body.dofs) o.channels.push_back({ dof.name + ".position_from_base", "rad" });
    for (const RLBodyDof3D& dof : body.dofs) o.channels.push_back({ dof.name + ".velocity", "rad/s/10" });
    for (const std::uint32_t link : keyBodies) vec3(o, body.profile.links[link].id + ".position_heading", "m");
    vec3(o, "LeftFoot.gravity_local", "unit");
    vec3(o, "RightFoot.gravity_local", "unit");
    vec3(o, "com.position_heading", "m");
    vec3(o, "com.velocity_heading", "m/s/2");
    o.channels.push_back({ "com.support_offset.x", "m/0.2" });
    o.channels.push_back({ "com.support_offset.y", "m/0.2" });
    o.channels.push_back({ "LeftFoot.contact", "0/1" });
    o.channels.push_back({ "RightFoot.contact", "0/1" });
    o.channels.push_back({ "LeftFoot.normal_force", "peso" });
    o.channels.push_back({ "RightFoot.normal_force", "peso" });
    for (const RLBodyDof3D& dof : body.dofs) o.channels.push_back({ dof.name + ".previous_action", "acao" });

    RLVectorLayout3D& c = criticLayout;
    c = o;
    c.id = "matter.rl.critic";
    c.channels.push_back({ "world.friction_scale", "x" });
    c.channels.push_back({ "body.mass_scale", "x" });
    c.channels.push_back({ "body.strength_scale", "x" });
    c.channels.push_back({ "body.action_delayed", "0/1" });
    vec3(c, "push.force_heading", "N/200");
    for (const RagdollLinkDefinition3D& link : body.profile.links)
        c.channels.push_back({ link.id + ".ground_contact", "0/1" });

    RLVectorLayout3D& a = actionLayout;
    a.id = "matter.rl.action";
    a.version = 1;
    for (const RLBodyDof3D& dof : body.dofs) a.channels.push_back({ dof.name + ".target", "acao" });
}

void TrainingBatch3D::Impl::configureEpisodeLength() {
    const float seconds = settings.evaluation ? settings.evaluationEpisodeSeconds : task.episodeSeconds;
    const float stepSeconds = static_cast<float>(task.actionRepeat) * FixedStep;
    maximumSteps = std::max(1u, static_cast<std::uint32_t>(std::lround(seconds / stepSeconds)));
}

void TrainingBatch3D::Impl::createWorld() {
    const std::uint32_t count = settings.environmentCount;
    const std::uint32_t perScene = std::max(1u, settings.environmentsPerScene);
    const std::uint32_t sceneCount = (count + perScene - 1) / perScene;
    scenes.resize(sceneCount);
    environments.resize(count);

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
        const float halfLength = 0.5f * EnvironmentSpacing * static_cast<float>(last - first) + 20.0f;
        PhysicsShape3D shape;
        shape.type = PhysicsShapeType3D::Box;
        shape.halfExtents = { halfLength, 20.0f, 0.5f };
        shape.materialId = task.groundMaterial;
        PhysicsBodyDefinition3D ground;
        ground.motionType = PhysicsMotionType3D::Static;
        ground.position = { 0.0f, 0.0f, -0.5f };
        ground.materialId = task.groundMaterial;
        (void)scene.physics->createBody(ground, std::span<const PhysicsShape3D>(&shape, 1));

        for (std::uint32_t index = first; index < last; ++index) {
            Environment& env = environments[index];
            env.scene = s;
            env.origin = { (static_cast<float>(index - first) - 0.5f * static_cast<float>(last - first - 1))
                    * EnvironmentSpacing, 0.0f, 0.0f };
            std::uint64_t envSeed = seedState + index * 0x9E3779B97F4A7C15ull;
            env.rng.seed(splitmix64(envSeed));
            // A massa varia por mundo (fica a vida toda dele): o corpo nao e
            // recriado entre episodios.
            env.massScale = training() ? uniform(env.rng, task.randomization.mass) : 1.0f;
            RagdollProfile3D profile = body.profile;
            profile.totalMassKg *= env.massScale;
            RagdollSpawnDefinition3D spawn;
            spawn.entityId = index + 1;
            spawn.pelvisPosition = env.origin + Vec3 { 0.0f, 0.0f, body.standingPelvisHeight + 0.05f };
            spawn.active = true;
            spawn.rigidityPercent = 0.0f;
            spawn.solverPositionIterations = body.solverPositionIterations;
            spawn.solverVelocityIterations = body.solverVelocityIterations;
            if (settings.solverPositionIterations >= 0)
                spawn.solverPositionIterations = static_cast<std::uint32_t>(settings.solverPositionIterations);
            if (settings.solverVelocityIterations >= 0)
                spawn.solverVelocityIterations = static_cast<std::uint32_t>(settings.solverVelocityIterations);
            env.ragdoll = scene.physics->createRagdoll(profile, spawn);
            env.previousAction.assign(actionSize(), 0.0f);
            env.action.assign(actionSize(), 0.0f);
            env.targets.resize(actionSize());
            env.previousTargets.resize(actionSize());
            scene.environments.push_back(index);
        }
        scene.finished.reserve(perScene);
    }
}

void TrainingBatch3D::Impl::writeTargets(Environment& env, std::span<const float> action) {
    for (std::size_t d = 0; d < body.dofs.size(); ++d) {
        const RLBodyDof3D& dof = body.dofs[d];
        RagdollDriveTarget3D& target = env.targets[d];
        target.linkIndex = dof.linkIndex;
        target.axis = dof.axis;
        target.positionRadians = dof.target(action.empty() ? 0.0f : action[d]);
        target.velocityRadiansPerSecond = 0.0f;
        target.feedforwardTorqueNewtonMeters = 0.0f;
        target.stiffnessScale = dof.stiffnessScale;
        target.dampingScale = dof.dampingScale;
        target.maximumTorqueScale = dof.maximumTorqueScale * env.strength;
        target.gravityCompensationScale = 0.0f;
    }
}

void TrainingBatch3D::Impl::resetEnvironment(std::uint32_t index) {
    Environment& env = environments[index];
    Scene& scene = scenes[env.scene];
    const TrainingTask3D::InitialState& start = task.initialState;
    const float noise = 1.0f;

    // A pose base com variacao pequena, girada no rumo sorteado e apoiada no
    // chao (o ponto mais baixo dos pes 2 mm acima dele).
    std::vector<Vec3> coordinates = body.baseJointCoordinates;
    std::vector<Vec3> velocities(coordinates.size());
    for (const RLBodyDof3D& dof : body.dofs) {
        Vec3& q = coordinates[dof.linkIndex];
        Vec3& qd = velocities[dof.linkIndex];
        const float dq = gaussian(env.rng, start.jointNoiseRadians * noise);
        const float dqd = gaussian(env.rng, start.jointVelocityNoise * noise);
        float* value = dof.axis == RagdollAxis3D::Twist ? &q.x : dof.axis == RagdollAxis3D::Swing1 ? &q.y : &q.z;
        float* rate = dof.axis == RagdollAxis3D::Twist ? &qd.x : dof.axis == RagdollAxis3D::Swing1 ? &qd.y : &qd.z;
        *value = std::clamp(*value + dq, dof.minimumRadians, dof.maximumRadians);
        *rate = dqd;
    }
    const float yaw = start.randomYaw
        ? std::uniform_real_distribution<float>(-std::numbers::pi_v<float>, std::numbers::pi_v<float>)(env.rng) : 0.0f;
    const Quaternion heading = Quaternion::fromAxisAngle({ 0.0f, 0.0f, 1.0f }, yaw);
    const Quaternion root = (heading * body.profile.links.front().modelOrientation * body.baseRootRotation).normalized();
    const RLBodyPose3D probe = rlBodyPose3D(body.profile, env.origin, root, coordinates);
    const float lift = 0.002f - rlBodyLowestFootPoint3D(body, probe);
    const Vec3 rootPosition = env.origin + Vec3 { 0.0f, 0.0f, lift };
    const Vec3 rootVelocity { gaussian(env.rng, start.rootVelocityNoise * noise),
        gaussian(env.rng, start.rootVelocityNoise * noise), 0.0f };
    const Vec3 rootSpin { gaussian(env.rng, start.rootAngularVelocityNoise * noise),
        gaussian(env.rng, start.rootAngularVelocityNoise * noise),
        gaussian(env.rng, start.rootAngularVelocityNoise * noise) };
    std::vector<PhysicsBodyState3D> links;
    rlBodyLinkStates3D(body.profile, rootPosition, root, rootVelocity, rootSpin, coordinates, velocities, links);
    scene.physics->resetRagdollState(env.ragdoll, links);

    // O mundo e o corpo deste episodio.
    const TrainingTask3D::Randomization& random = task.randomization;
    env.friction = training() ? uniform(env.rng, random.friction) : 1.0f;
    env.strength = training() ? uniform(env.rng, random.strength) : 1.0f;
    env.delayed = training() && std::uniform_real_distribution<float>(0.0f, 1.0f)(env.rng)
            < random.actionDelayProbability;
    scene.physics->setRagdollFrictionScale(env.ragdoll, env.friction);
    env.steps = 0;
    std::fill(env.previousAction.begin(), env.previousAction.end(), 0.0f);
    std::fill(env.action.begin(), env.action.end(), 0.0f);
    writeTargets(env, {});
    env.previousTargets = env.targets;
    scene.physics->setRagdollActiveDriveTargets(env.ragdoll, env.targets, false);
    env.firstContactLink = -1;
    env.pushTimer = uniform(env.rng, task.pushes.intervalSeconds);
    env.pushRemaining = 0.0f;
    env.pushForce = {};
    const RagdollState3D* state = scene.physics->ragdollStateView(env.ragdoll);
    for (std::size_t f = 0; f < 2; ++f) {
        const std::uint32_t foot = f == 0 ? body.leftFoot : body.rightFoot;
        env.footStart[f] = state ? state->links[foot].position : Vec3 {};
    }
    env.episodeReturn = 0.0;
    env.comSum[0] = env.comSum[1] = 0.0;
    env.comSquares[0] = env.comSquares[1] = 0.0;
    env.footDriftMax = 0.0f;
    env.actionRateSum = env.effortSum = env.jointSpeedSum = env.uprightSum = 0.0;
}

BodyReading TrainingBatch3D::Impl::read(const RagdollState3D& state) const {
    BodyReading reading;
    const RagdollProfile3D& profile = body.profile;
    const PhysicsBodyState3D& pelvis = state.links.front();
    if (!finite(pelvis.position) || !finite(pelvis.linearVelocity) || !std::isfinite(pelvis.orientation.w)) {
        reading.valid = false;
        return reading;
    }
    reading.pelvisFrame = (pelvis.orientation * profile.links.front().modelOrientation.conjugate()).normalized();
    const Vec3 forward = reading.pelvisFrame.rotate({ 1.0f, 0.0f, 0.0f });
    const float yaw = std::atan2(forward.y, forward.x);
    reading.headingInverse = Quaternion::fromAxisAngle({ 0.0f, 0.0f, 1.0f }, -yaw);
    float massSum = 0.0f;
    for (std::size_t i = 0; i < profile.links.size(); ++i) {
        const float m = profile.links[i].massFraction;
        reading.com += state.links[i].position * m;
        reading.comVelocity += state.links[i].linearVelocity * m;
        massSum += m;
        if (!finite(state.links[i].position)) reading.valid = false;
    }
    reading.com = reading.com / massSum;
    reading.comVelocity = reading.comVelocity / massSum;
    for (std::size_t f = 0; f < 2; ++f) {
        const std::uint32_t foot = f == 0 ? body.leftFoot : body.rightFoot;
        const PhysicsBodyState3D& link = state.links[foot];
        reading.footCenter[f] = link.position + link.orientation.rotate(profile.links[foot].collider.localPosition);
    }
    for (const RagdollContactPoint3D& contact : state.contacts) {
        if (contact.otherMotion != RagdollContactMotion3D::Static || contact.linkIndex >= 32) continue;
        reading.groundMask |= 1u << contact.linkIndex;
        for (std::size_t f = 0; f < 2; ++f) {
            const std::uint32_t foot = f == 0 ? body.leftFoot : body.rightFoot;
            if (contact.linkIndex != foot) continue;
            reading.footContact[f] = true;
            reading.footForce[f] += contact.normalImpulseNewtonSeconds / FixedStep;
        }
    }
    return reading;
}

void TrainingBatch3D::Impl::writeObservation(const Environment& env, const RagdollState3D& state,
    const BodyReading& reading, float* actor, float* criticOut, bool noise) {
    const RagdollProfile3D& profile = body.profile;
    const PhysicsBodyState3D& pelvis = state.links.front();
    std::mt19937_64& rng = const_cast<Environment&>(env).rng;
    const float k = noise ? task.randomization.observationNoise : 0.0f;
    const auto n = [&](float sigma) { return k > 0.0f ? gaussian(rng, sigma * k) : 0.0f; };
    const auto put3 = [](float*& out, Vec3 v) {
        *out++ = v.x;
        *out++ = v.y;
        *out++ = v.z;
    };
    const auto noisy3 = [&](Vec3 v, float sigma) { return Vec3 { v.x + n(sigma), v.y + n(sigma), v.z + n(sigma) }; };

    float* out = actor;
    *out++ = (pelvis.position.z - body.standingPelvisHeight) / 0.3f + n(0.005f);
    put3(out, noisy3(reading.pelvisFrame.conjugate().rotate({ 0.0f, 0.0f, -1.0f }), 0.02f));
    put3(out, noisy3(reading.headingInverse.rotate(pelvis.linearVelocity) * 0.5f, 0.02f));
    put3(out, noisy3(reading.pelvisFrame.conjugate().rotate(pelvis.angularVelocity) * 0.25f, 0.02f));
    for (const RLBodyDof3D& dof : body.dofs) {
        const RagdollJointState3D& joint = state.joints[dof.linkIndex];
        *out++ = joint.positionRadians[static_cast<std::size_t>(dof.axis)] - dof.baseRadians + n(0.01f);
    }
    for (const RLBodyDof3D& dof : body.dofs) {
        const RagdollJointState3D& joint = state.joints[dof.linkIndex];
        *out++ = joint.velocityRadiansPerSecond[static_cast<std::size_t>(dof.axis)] * 0.1f + n(0.015f);
    }
    for (const std::uint32_t link : keyBodies)
        put3(out, noisy3(reading.headingInverse.rotate(state.links[link].position - pelvis.position), 0.005f));
    for (const std::uint32_t foot : { body.leftFoot, body.rightFoot }) {
        const Quaternion frame = state.links[foot].orientation * profile.links[foot].modelOrientation.conjugate();
        put3(out, noisy3(frame.conjugate().rotate({ 0.0f, 0.0f, -1.0f }), 0.02f));
    }
    put3(out, noisy3(reading.headingInverse.rotate(reading.com - pelvis.position), 0.005f));
    put3(out, noisy3(reading.headingInverse.rotate(reading.comVelocity) * 0.5f, 0.02f));
    const Vec3 support = (reading.footCenter[0] + reading.footCenter[1]) * 0.5f;
    const Vec3 offset = reading.headingInverse.rotate(Vec3 { reading.com.x - support.x, reading.com.y - support.y, 0.0f });
    *out++ = offset.x * 5.0f + n(0.02f);
    *out++ = offset.y * 5.0f + n(0.02f);
    const float weight = profile.totalMassKg * env.massScale * Gravity;
    for (std::size_t f = 0; f < 2; ++f) *out++ = reading.footContact[f] ? 1.0f : 0.0f;
    for (std::size_t f = 0; f < 2; ++f) *out++ = std::min(3.0f, reading.footForce[f] / weight);
    for (const float a : env.action) *out++ = a;

    if (criticOut == nullptr) return;
    // O critico: a observacao sem ruido mais o mundo que a politica nao ve.
    if (k > 0.0f) {
        writeObservation(env, state, reading, criticOut, nullptr, false);
    } else {
        std::copy(actor, actor + observationSize(), criticOut);
    }
    float* extra = criticOut + observationSize();
    *extra++ = env.friction;
    *extra++ = env.massScale;
    *extra++ = env.strength;
    *extra++ = env.delayed ? 1.0f : 0.0f;
    const Vec3 push = env.pushRemaining > 0.0f ? reading.headingInverse.rotate(env.pushForce) / 200.0f : Vec3 {};
    put3(extra, push);
    for (std::size_t i = 0; i < profile.links.size(); ++i)
        *extra++ = (reading.groundMask >> i) & 1u ? 1.0f : 0.0f;
}

float TrainingBatch3D::Impl::reward(Environment& env, const RagdollState3D& state, const BodyReading& reading,
    bool fell, float* componentsOut) {
    const TrainingRewardWeights3D& w = task.reward;
    const RagdollProfile3D& profile = body.profile;

    // Tronco e pelve na vertical (cada um vale metade).
    const auto tilt = [&](std::uint32_t link) {
        const Quaternion frame = state.links[link].orientation * profile.links[link].modelOrientation.conjugate();
        const float up = std::clamp(frame.rotate({ 0.0f, 0.0f, 1.0f }).z, -1.0f, 1.0f);
        return std::acos(up);
    };
    const float pelvisTilt = tilt(0);
    const float chestTilt = tilt(body.upperChest);
    const float upright = 0.5f * std::exp(-pelvisTilt * pelvisTilt / 0.06f) + 0.5f * std::exp(-chestTilt * chestTilt / 0.06f);
    const float headError = (state.links[body.head].position.z - body.standingHeadHeight) / 0.08f;
    const float headHeight = std::exp(-headError * headError);
    const Vec3 support = (reading.footCenter[0] + reading.footCenter[1]) * 0.5f;
    const float dx = reading.com.x - support.x, dy = reading.com.y - support.y;
    const float comOverSupport = std::exp(-(dx * dx + dy * dy) / (0.06f * 0.06f));
    const float comSpeed2 = reading.comVelocity.lengthSquared();
    const float still = std::exp(-comSpeed2 / 0.04f);

    float slip = 0.0f, drift = 0.0f;
    for (std::size_t f = 0; f < 2; ++f) {
        const std::uint32_t foot = f == 0 ? body.leftFoot : body.rightFoot;
        const PhysicsBodyState3D& link = state.links[foot];
        if (reading.footContact[f])
            slip += std::sqrt(link.linearVelocity.x * link.linearVelocity.x + link.linearVelocity.y * link.linearVelocity.y);
        const float mx = link.position.x - env.footStart[f].x, my = link.position.y - env.footStart[f].y;
        const float moved = std::sqrt(mx * mx + my * my);
        drift += moved;
        env.footDriftMax = std::max(env.footDriftMax, moved);
    }

    float effort = 0.0f, actionRate = 0.0f, limit = 0.0f, pose = 0.0f, speed = 0.0f;
    for (std::size_t d = 0; d < body.dofs.size(); ++d) {
        const RLBodyDof3D& dof = body.dofs[d];
        const RagdollJointState3D& joint = state.joints[dof.linkIndex];
        const std::size_t axis = static_cast<std::size_t>(dof.axis);
        const float cap = std::max(1.0f, dof.maximumTorque * env.strength);
        const float torque = joint.motorTorqueNewtonMeters[axis] / cap;
        effort += torque * torque;
        const float da = env.action[d] - env.previousAction[d];
        actionRate += da * da;
        const float range = std::max(0.01f, dof.maximumRadians - dof.minimumRadians);
        const float q = joint.positionRadians[axis];
        const float x = (q - dof.minimumRadians) / range;
        limit += std::max(0.0f, 0.05f - x) / 0.05f + std::max(0.0f, x - 0.95f) / 0.05f;
        const float fromBase = 2.0f * (q - dof.baseRadians) / range;
        pose += fromBase * fromBase;
        speed += std::abs(joint.velocityRadiansPerSecond[axis]);
    }
    const float dofCount = static_cast<float>(std::max<std::size_t>(1, body.dofs.size()));
    effort /= dofCount;
    actionRate /= dofCount;
    limit /= dofCount;
    pose /= dofCount;
    speed /= dofCount;

    const std::array<float, RewardCount> values {
        w.alive,
        w.upright * upright,
        w.headHeight * headHeight,
        w.comOverSupport * comOverSupport,
        w.still * still,
        -w.footSlip * slip,
        -w.footDrift * drift,
        -w.effort * effort,
        -w.actionRate * actionRate,
        -w.jointLimit * limit,
        -w.posePrior * pose,
        fell ? -w.fall : 0.0f,
    };
    float total = 0.0f;
    for (std::size_t i = 0; i < RewardCount; ++i) {
        componentsOut[i] = values[i];
        total += values[i];
    }

    // O resumo do episodio.
    env.comSum[0] += reading.com.x;
    env.comSum[1] += reading.com.y;
    env.comSquares[0] += static_cast<double>(reading.com.x) * reading.com.x;
    env.comSquares[1] += static_cast<double>(reading.com.y) * reading.com.y;
    env.actionRateSum += actionRate;
    env.effortSum += effort;
    env.jointSpeedSum += speed;
    env.uprightSum += upright;
    return total;
}

void TrainingBatch3D::Impl::finishEpisode(Scene& scene, std::uint32_t index, TrainingEndReason3D reason,
    const BodyReading& reading) {
    const Environment& env = environments[index];
    TrainingEpisode3D episode;
    episode.environment = index;
    episode.reason = reason;
    episode.firstContactLink = env.firstContactLink;
    episode.steps = env.steps;
    episode.seconds = static_cast<float>(env.steps) * static_cast<float>(task.actionRepeat) * FixedStep;
    episode.episodeReturn = static_cast<float>(env.episodeReturn);
    const double n = std::max(1u, env.steps);
    double variance = 0.0;
    for (std::size_t a = 0; a < 2; ++a) {
        const double mean = env.comSum[a] / n;
        variance += std::max(0.0, env.comSquares[a] / n - mean * mean);
    }
    episode.comSwayRms = static_cast<float>(std::sqrt(variance));
    episode.footDriftMax = env.footDriftMax;
    episode.meanActionRate = static_cast<float>(env.actionRateSum / n);
    episode.meanEffort = static_cast<float>(env.effortSum / n);
    episode.meanJointSpeed = static_cast<float>(env.jointSpeedSum / n);
    episode.meanUpright = static_cast<float>(env.uprightSum / n);
    if (reason == TrainingEndReason3D::FallContact || reason == TrainingEndReason3D::FallHeight) {
        // O lado da queda: para onde o centro de massa ia, no referencial do corpo.
        const Vec3 v = reading.valid ? reading.headingInverse.rotate(reading.comVelocity) : Vec3 {};
        episode.fallDirection = std::abs(v.x) >= std::abs(v.y) ? (v.x >= 0.0f ? 0 : 1) : (v.y >= 0.0f ? 2 : 3);
    }
    scene.finished.push_back(episode);
}

void TrainingBatch3D::Impl::stepScene(Scene& scene, std::span<const float> actions) {
    PhysicsScene3D& physics = *scene.physics;
    const std::size_t A = actionSize();
    scene.finished.clear();
    for (const std::uint32_t index : scene.environments) {
        Environment& env = environments[index];
        std::swap(env.previousAction, env.action);
        for (std::size_t d = 0; d < A; ++d)
            env.action[d] = std::clamp(actions[index * A + d], -RLActionLimit, RLActionLimit);
        std::swap(env.previousTargets, env.targets);
        writeTargets(env, env.action);
        physics.setRagdollActiveDriveTargets(env.ragdoll, env.delayed ? env.previousTargets : env.targets, false);
    }

    const bool pushes = task.pushes.enabled && !pushLinks.empty();
    for (std::uint32_t sub = 0; sub < task.actionRepeat; ++sub) {
        for (const std::uint32_t index : scene.environments) {
            Environment& env = environments[index];
            // Atraso de um passo fisico: o comando novo entra no segundo.
            if (sub == 1 && env.delayed) physics.setRagdollActiveDriveTargets(env.ragdoll, env.targets, false);
            if (env.pushRemaining > 0.0f) {
                physics.applyRagdollLinkForce(env.ragdoll, env.pushLink, env.pushForce, {});
                env.pushRemaining -= FixedStep;
                continue;
            }
            if (!pushes) continue;
            if ((env.pushTimer -= FixedStep) <= 0.0f) {
                const float angle =
                    std::uniform_real_distribution<float>(-std::numbers::pi_v<float>, std::numbers::pi_v<float>)(env.rng);
                const float force = uniform(env.rng, task.pushes.forceNewtons);
                env.pushForce = { std::cos(angle) * force, std::sin(angle) * force, 0.0f };
                env.pushLink = pushLinks[std::uniform_int_distribution<std::size_t>(0, pushLinks.size() - 1)(env.rng)];
                env.pushRemaining = uniform(env.rng, task.pushes.durationSeconds);
                env.pushTimer = uniform(env.rng, task.pushes.intervalSeconds);
            }
        }
        physics.simulate(FixedStep);
        // Queda por contato em qualquer subpasso: um link que nao e apoio no chao.
        if (!task.termination.nonSupportContact) continue;
        for (const std::uint32_t index : scene.environments) {
            Environment& env = environments[index];
            if (env.firstContactLink >= 0) continue;
            const RagdollState3D* state = physics.ragdollStateView(env.ragdoll);
            if (!state) continue;
            for (const RagdollContactPoint3D& contact : state->contacts) {
                if (contact.otherMotion != RagdollContactMotion3D::Static) continue;
                if (contact.linkIndex < body.supportLink.size() && body.supportLink[contact.linkIndex]) continue;
                env.firstContactLink = static_cast<int>(contact.linkIndex);
                break;
            }
        }
    }

    const std::size_t O = observationSize();
    const std::size_t C = criticSize();
    for (const std::uint32_t index : scene.environments) {
        Environment& env = environments[index];
        ++env.steps;
        const RagdollState3D* state = physics.ragdollStateView(env.ragdoll);
        float* actor = observations.data() + index * O;
        float* criticOut = critic.data() + index * C;
        float* parts = components.data() + index * RewardCount;
        BodyReading reading;
        if (state) reading = read(*state);
        else reading.valid = false;

        TrainingEndReason3D reason = TrainingEndReason3D::None;
        if (!reading.valid) {
            reason = TrainingEndReason3D::Numeric;
        } else {
            const PhysicsBodyState3D& pelvis = state->links.front();
            const float head = state->links[body.head].position.z;
            if (env.firstContactLink >= 0) reason = TrainingEndReason3D::FallContact;
            else if (head < body.standingHeadHeight * task.termination.minimumHeadHeightFraction
                || pelvis.position.z < body.standingPelvisHeight * task.termination.minimumPelvisHeightFraction)
                reason = TrainingEndReason3D::FallHeight;
            else if (env.steps >= maximumSteps) reason = TrainingEndReason3D::TimeLimit;
            else {
                const float dx = pelvis.position.x - env.origin.x, dy = pelvis.position.y - env.origin.y;
                if (dx * dx + dy * dy > task.termination.maximumDistanceFromStart * task.termination.maximumDistanceFromStart)
                    reason = TrainingEndReason3D::Boundary;
            }
        }
        const bool fell = reason == TrainingEndReason3D::FallContact || reason == TrainingEndReason3D::FallHeight;
        if (reading.valid) {
            rewards[index] = reward(env, *state, reading, fell, parts);
            writeObservation(env, *state, reading, actor, criticOut, training());
        } else {
            std::fill(parts, parts + RewardCount, 0.0f);
            parts[RewardCount - 1] = -task.reward.fall;
            rewards[index] = -task.reward.fall;
            std::fill(actor, actor + O, 0.0f);
            std::fill(criticOut, criticOut + C, 0.0f);
        }
        env.episodeReturn += rewards[index];
        terminated[index] = fell || reason == TrainingEndReason3D::Numeric ? 1u : 0u;
        truncated[index] =
            reason == TrainingEndReason3D::TimeLimit || reason == TrainingEndReason3D::Boundary ? 1u : 0u;
        if (reason == TrainingEndReason3D::None) continue;

        std::copy(criticOut, criticOut + C, terminalCritic.data() + index * C);
        finishEpisode(scene, index, reason, reading);
        resetEnvironment(index);
        const RagdollState3D* fresh = physics.ragdollStateView(env.ragdoll);
        const BodyReading start = read(*fresh);
        writeObservation(env, *fresh, start, actor, criticOut, training());
    }
}

TrainingBatch3D::TrainingBatch3D(const TrainingBatchSettings3D& settings) : m_impl(std::make_unique<Impl>()) {
    Impl& impl = *m_impl;
    if (settings.environmentCount == 0) throw std::invalid_argument("lote de RL sem mundos");
    impl.settings = settings;
    impl.task = loadTrainingTask3D(settings.taskPath);
    impl.body = loadRLBody3D(impl.task.bodyPath);
    impl.keyBodies = { impl.body.head, impl.body.leftHand, impl.body.rightHand, impl.body.leftFoot,
        impl.body.rightFoot };
    for (const std::string& id : impl.task.pushes.links)
        for (std::size_t i = 0; i < impl.body.profile.links.size(); ++i)
            if (impl.body.profile.links[i].id == id) impl.pushLinks.push_back(static_cast<std::uint32_t>(i));
    impl.configureEpisodeLength();
    impl.buildLayouts();

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
    impl.createWorld();
    reset(settings.seed);
}

TrainingBatch3D::~TrainingBatch3D() = default;

std::size_t TrainingBatch3D::environmentCount() const { return m_impl->environments.size(); }
std::size_t TrainingBatch3D::observationSize() const { return m_impl->observationSize(); }
std::size_t TrainingBatch3D::criticObservationSize() const { return m_impl->criticSize(); }
std::size_t TrainingBatch3D::actionSize() const { return m_impl->actionSize(); }
std::size_t TrainingBatch3D::rewardComponentCount() const { return RewardCount; }
std::uint32_t TrainingBatch3D::workerThreads() const { return m_impl->workers; }
float TrainingBatch3D::policyStepSeconds() const { return static_cast<float>(m_impl->task.actionRepeat) * FixedStep; }
const RLVectorLayout3D& TrainingBatch3D::observationLayout() const { return m_impl->observationLayout; }
const RLVectorLayout3D& TrainingBatch3D::criticObservationLayout() const { return m_impl->criticLayout; }
const RLVectorLayout3D& TrainingBatch3D::actionLayout() const { return m_impl->actionLayout; }
const RLBody3D& TrainingBatch3D::body() const { return m_impl->body; }
const TrainingTask3D& TrainingBatch3D::task() const { return m_impl->task; }

void TrainingBatch3D::updateTask(const TrainingTask3D& task) {
    Impl& impl = *m_impl;
    impl.task.reward = task.reward;
    impl.task.termination = task.termination;
    impl.task.initialState = task.initialState;
    impl.task.randomization = task.randomization;
    impl.task.pushes = task.pushes;
    impl.pushLinks.clear();
    for (const std::string& id : impl.task.pushes.links)
        for (std::size_t i = 0; i < impl.body.profile.links.size(); ++i)
            if (impl.body.profile.links[i].id == id) impl.pushLinks.push_back(static_cast<std::uint32_t>(i));
}

void TrainingBatch3D::reset(std::uint64_t seed) {
    Impl& impl = *m_impl;
    std::uint64_t state = seed;
    for (Environment& env : impl.environments) env.rng.seed(splitmix64(state));
    const std::size_t O = impl.observationSize(), C = impl.criticSize();
    for (std::uint32_t i = 0; i < impl.environments.size(); ++i) {
        impl.resetEnvironment(i);
        const Environment& env = impl.environments[i];
        const RagdollState3D* view = impl.scenes[env.scene].physics->ragdollStateView(env.ragdoll);
        const BodyReading reading = impl.read(*view);
        impl.writeObservation(env, *view, reading, impl.observations.data() + i * O, impl.critic.data() + i * C,
            impl.training());
    }
    std::fill(impl.rewards.begin(), impl.rewards.end(), 0.0f);
    std::fill(impl.terminated.begin(), impl.terminated.end(), 0);
    std::fill(impl.truncated.begin(), impl.truncated.end(), 0);
    impl.finished.clear();
}

void TrainingBatch3D::step(std::span<const float> actions) {
    Impl& impl = *m_impl;
    if (actions.size() != impl.environments.size() * impl.actionSize())
        throw std::invalid_argument("lote de RL: dimensao das acoes incompativel");
    for (const float value : actions)
        if (!std::isfinite(value)) throw std::invalid_argument("lote de RL: acao nao finita");
    const auto begin = std::chrono::steady_clock::now();
    struct Context {
        Impl* impl;
        std::span<const float> actions;
    } context { &impl, actions };
    impl.scheduler->parallelFor(impl.scenes.size(), 1,
        [](std::size_t first, std::size_t last, void* raw) noexcept {
            auto* ctx = static_cast<Context*>(raw);
            for (std::size_t s = first; s < last; ++s) ctx->impl->stepScene(ctx->impl->scenes[s], ctx->actions);
        },
        &context);
    impl.finished.clear();
    for (const Scene& scene : impl.scenes)
        impl.finished.insert(impl.finished.end(), scene.finished.begin(), scene.finished.end());
    impl.physicsSteps += impl.environments.size() * impl.task.actionRepeat;
    impl.lastStepMilliseconds =
        std::chrono::duration<float, std::milli>(std::chrono::steady_clock::now() - begin).count();
}

std::span<const float> TrainingBatch3D::observations() const { return m_impl->observations; }
std::span<const float> TrainingBatch3D::criticObservations() const { return m_impl->critic; }
std::span<const float> TrainingBatch3D::rewards() const { return m_impl->rewards; }
std::span<const float> TrainingBatch3D::rewardComponents() const { return m_impl->components; }
std::span<const std::uint8_t> TrainingBatch3D::terminated() const { return m_impl->terminated; }
std::span<const std::uint8_t> TrainingBatch3D::truncated() const { return m_impl->truncated; }
std::span<const float> TrainingBatch3D::terminalCriticObservations() const { return m_impl->terminalCritic; }
std::span<const TrainingEpisode3D> TrainingBatch3D::finishedEpisodes() const { return m_impl->finished; }

TrainingEnvironmentView3D TrainingBatch3D::view(std::size_t environment) const {
    const Impl& impl = *m_impl;
    TrainingEnvironmentView3D view;
    if (environment >= impl.environments.size()) return view;
    const Environment& env = impl.environments[environment];
    view.state = impl.scenes[env.scene].physics->ragdollStateView(env.ragdoll);
    view.origin = env.origin;
    view.steps = env.steps;
    if (env.pushRemaining > 0.0f) {
        view.pushForce = env.pushForce;
        view.pushLink = static_cast<int>(env.pushLink);
    }
    return view;
}

void TrainingBatch3D::push(std::size_t environment, std::uint32_t link, Vec3 forceNewtons, float seconds) {
    Impl& impl = *m_impl;
    if (environment >= impl.environments.size() || link >= impl.body.profile.links.size() || !finite(forceNewtons)
        || !(seconds > 0.0f))
        return;
    Environment& env = impl.environments[environment];
    env.pushLink = link;
    env.pushForce = forceNewtons;
    env.pushRemaining = seconds;
}

std::vector<float> TrainingBatch3D::actionRadiansPerUnit() const {
    std::vector<float> result;
    for (const RLBodyDof3D& dof : m_impl->body.dofs) result.push_back(dof.actionScale);
    return result;
}

float TrainingBatch3D::lastStepMilliseconds() const { return m_impl->lastStepMilliseconds; }
std::uint64_t TrainingBatch3D::physicsStepCount() const { return m_impl->physicsSteps; }

} // namespace MatterEngine
