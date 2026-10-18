#include "Workbench/ReinforcementLearning/LocomotionViewer3D.hpp"

#include "Engine/Athlete/AthleteTelemetry3D.hpp"
#include "Engine/Core/TaskScheduler.hpp"
#include "Engine/RL/Commands/PilotScriptJson3D.hpp"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <span>
#include <stdexcept>

namespace MatterEngine {

namespace {

constexpr float TickSeconds = 1.0f / 120.0f;
constexpr int TicksPerDecision = 4;
constexpr const char* Schema = "matter-rl-locomocao-1";

nlohmann::json readTask(const std::filesystem::path& path) {
    std::ifstream file(path);
    if (!file) return {};
    nlohmann::json document = nlohmann::json::parse(file, nullptr, false);
    return document.is_discarded() ? nlohmann::json {} : document;
}

Vec3 planar(Vec3 v) { return { v.x, v.y, 0.0f }; }

} // namespace

struct LocomotionViewer3D::Impl {
    std::shared_ptr<const AthleteBundle3D> bundle;
    std::shared_ptr<TaskScheduler> scheduler;
    std::unique_ptr<PhysicsEngine3D> engine;
    MaterialLibrary materials;
    std::unique_ptr<PhysicsScene3D> scene;
    RagdollHandle3D handle;
    RagdollState3D body;
    AthleteState3D athlete;
    AthleteRuntime3D runtime;
    AthleteRuntimeSettings3D settings;
    std::shared_ptr<LocomotionPolicy3D> policy;
    PilotScript3D pilot;
    PilotState3D pilotState;
    std::mt19937_64 rng;
    std::uint32_t drillIndex = 0;
    std::optional<float> manual;
    AthleteCommand3D command;
    Vec3 origin;
    std::uint32_t steps = 0;
    float fallenSeconds = 0.0f;
    Vec3 pushForce;
    float pushSeconds = 0.0f;
    std::size_t chest = 0;
    float groundHalfExtent = 60.0f;

    void build(std::uint64_t seed) {
        scene.reset();
        // A cena de um mundo do treino: uma thread, o Jolt inline.
        PhysicsSceneSettings3D sceneSettings;
        sceneSettings.workerThreadCount = 1;
        sceneSettings.runJobsInline = true;
        sceneSettings.maximumBodies = 128u;
        sceneSettings.maximumBodyPairs = 4096u;
        sceneSettings.maximumContactConstraints = 2048u;
        sceneSettings.scratchBufferSizeBytes = 4u * 1024u * 1024u;
        scene = engine->createScene(sceneSettings, materials);
        PhysicsShape3D ground;
        ground.type = PhysicsShapeType3D::Box;
        ground.halfExtents = { groundHalfExtent, groundHalfExtent, 0.1f };
        ground.materialId = "concrete";
        PhysicsBodyDefinition3D floor;
        floor.motionType = PhysicsMotionType3D::Static;
        floor.position = { 0.0f, 0.0f, -0.1f };
        floor.materialId = "concrete";
        (void)scene->createBody(floor, std::span<const PhysicsShape3D>(&ground, 1));
        const RagdollProfile3D& profile = bundle->profile();
        RagdollSpawnDefinition3D spawn;
        spawn.entityId = 1;
        spawn.active = true;
        spawn.pelvisPosition = { 0.0f, 0.0f, profile.standingRootHeightMeters };
        handle = scene->createRagdoll(profile, spawn);
        body = scene->ragdollState(handle);
        athlete = AthleteState3D {};
        runtime = AthleteRuntime3D {};
        runtime.setLocomotionPolicy(policy);
        athleteStart3D(*scene, handle, profile, body, athlete, AthleteStartMode3D::Physical);
        rng.seed(seed);
        origin = body.links.empty() ? Vec3 {} : planar(body.links.front().position);
        pilotState = PilotState3D {};
        CommandPilot3D::start(pilot, pilotState, origin, 0.0f, rng);
        steps = 0;
        fallenSeconds = 0.0f;
        pushSeconds = 0.0f;
    }

    [[nodiscard]] Vec3 centerOfMassVelocity() const {
        const RagdollProfile3D& profile = bundle->profile();
        Vec3 sum;
        float weight = 0.0f;
        for (std::size_t i = 0; i < profile.links.size() && i < body.links.size(); ++i) {
            sum += body.links[i].linearVelocity * profile.links[i].massFraction;
            weight += profile.links[i].massFraction;
        }
        return weight > 0.0f ? sum * (1.0f / weight) : Vec3 {};
    }

    // O comando do piloto pela conta do analogico do jogo (a mesma do treino).
    AthleteCommand3D pilotCommand() {
        PilotView3D view;
        if (!body.links.empty()) view.position = body.links.front().position;
        view.velocity = planar(centerOfMassVelocity());
        ControlInput3D input = CommandPilot3D::update(pilot, pilotState, view, policyStepSecondsValue(), rng);
        if (manual) {
            input.stickForward = *manual > 0.0f ? 1.0f : 0.0f;
            input.stickLeft = 0.0f;
            input.speed = *manual;
        }
        const float yaw = input.cameraHeading;
        Vec3 direction { std::cos(yaw) * input.stickForward - std::sin(yaw) * input.stickLeft,
            std::sin(yaw) * input.stickForward + std::cos(yaw) * input.stickLeft, 0.0f };
        const float magnitude = std::hypot(direction.x, direction.y);
        AthleteStickInput3D stick;
        stick.cameraYawRadians = yaw;
        stick.lookPitchRadians = -0.2f;
        if (magnitude > 1.0e-4f) {
            direction = direction * (1.0f / magnitude);
            stick.moveWorld = direction;
            const auto tableSpeed = [&]() {
                const Vec3 table = athleteStickCommand3D(stick, settings, bundle->gaitSpeeds(), 1.0e9f)
                                       .intent.requestedVelocityWorld;
                return std::hypot(table.x, table.y);
            };
            const float wanted = std::max(0.0f, input.speed) * std::min(1.0f, magnitude);
            float table = tableSpeed();
            // Acima do trote da tabela, o sprint (como o jogador segurando a tecla).
            if (wanted > table + 1.0e-3f) {
                stick.sprint = true;
                table = tableSpeed();
            }
            stick.moveWorld = direction * (table > 1.0e-3f ? std::min(1.0f, wanted / table) : 0.0f);
        }
        return athleteStickCommand3D(stick, settings, bundle->gaitSpeeds(), athleteTelemetry3D(athlete).terrainSpeedLimit);
    }

    static float policyStepSecondsValue() { return TickSeconds * static_cast<float>(TicksPerDecision); }

    void tick() {
        AthleteSlot3D slot { 1, handle, &body, true, false, &athlete };
        slot.command = &command;
        AthleteControlInput3D control;
        control.controlledEntityId = 0;
        AthleteRuntimeOutput3D output;
        settings.locomotionPolicy = policy != nullptr;
        // (A rede que treinou com o professor da virada brusca: ele entra.)
        settings.dynamicTurnMove = policy != nullptr && policy->usesTurnMove();
        if (settings.dynamicTurnMove) settings.dynamicTurnMoveSettings.amount = policy->turnMoveAmount();
        runtime.update(*scene, bundle->assets(), std::span<AthleteSlot3D>(&slot, 1), {}, settings, control,
            TickSeconds, output);
        if (pushSeconds > 0.0f && chest < body.links.size()) {
            scene->applyRagdollLinkForce(handle, static_cast<std::uint32_t>(chest), pushForce, {});
            pushSeconds -= TickSeconds;
        }
        scene->simulate(TickSeconds);
        body = scene->ragdollState(handle);
    }
};

LocomotionViewer3D::LocomotionViewer3D(const std::filesystem::path& taskPath,
    const std::filesystem::path& assetsDirectory, const std::string& drill, std::uint64_t seed)
    : m_impl(std::make_unique<Impl>()) {
    Impl& impl = *m_impl;
    const nlohmann::json task = readTask(taskPath);
    if (task.value("schema", std::string()) != Schema)
        throw std::runtime_error("visualizador da locomocao: a tarefa nao e da locomocao: " + taskPath.string());
    const nlohmann::json& drills = task.at("drills");
    for (std::size_t i = 0; i < drills.size(); ++i)
        if (drills[i].value("id", std::string()) == drill) impl.drillIndex = static_cast<std::uint32_t>(i);
    const nlohmann::json& chosen = drills.at(impl.drillIndex);
    impl.pilot = pilotScriptFromJson3D(chosen.value("pilot", nlohmann::json::object()), chosen.value("id", std::string()));
    // As ajudas da tarefa e o fisico total com a referencia a sombra (como no treino).
    impl.settings.defaultLocomotion = false;
    impl.settings.physicalReference = true;
    impl.settings.e5StandingAssistPercent = impl.settings.e5WalkingAssistPercent = 0.0f;
    impl.settings.physicalFeetAssistPercent = impl.settings.getUpAssistPercent = 0.0f;
    if (task.contains("assist")) {
        const nlohmann::json& a = task["assist"];
        impl.settings.e5StandingAssistPercent = a.value("standing", 0.0f);
        impl.settings.e5WalkingAssistPercent = a.value("walking", 0.0f);
        impl.settings.physicalFeetAssistPercent = a.value("feet", 0.0f);
        impl.settings.getUpAssistPercent = a.value("getUp", 0.0f);
    }
    // A tabela de velocidades e o chao da tarefa (como no treino).
    if (task.contains("speeds")) {
        const nlohmann::json& v = task["speeds"];
        impl.settings.defaultTrotSpeed = v.value("trot", impl.settings.defaultTrotSpeed);
        impl.settings.defaultSprintForwardSpeed = v.value("sprintForward", impl.settings.defaultSprintForwardSpeed);
        impl.settings.defaultSprintSideSpeed = v.value("sprintSide", impl.settings.defaultSprintSideSpeed);
        impl.settings.defaultSprintBackwardSpeed = v.value("sprintBackward", impl.settings.defaultSprintBackwardSpeed);
    }
    if (task.contains("termination"))
        impl.groundHalfExtent = task["termination"].value("groundHalfExtentMeters", impl.groundHalfExtent);
    impl.bundle = AthleteBundle3D::shared(assetsDirectory.string());
    impl.scheduler = std::make_shared<TaskScheduler>(TaskSchedulerSettings { 1u });
    impl.engine = std::make_unique<PhysicsEngine3D>(impl.scheduler);
    const RagdollProfile3D& profile = impl.bundle->profile();
    for (std::size_t i = 0; i < profile.links.size(); ++i)
        if (profile.links[i].id == "Chest") impl.chest = i;
    impl.build(seed);
}

LocomotionViewer3D::~LocomotionViewer3D() {
    if (m_impl) m_impl->scene.reset();
}

std::vector<LocomotionViewer3D::Drill> LocomotionViewer3D::drillsOf(const std::filesystem::path& taskPath) {
    std::vector<Drill> out;
    const nlohmann::json task = readTask(taskPath);
    if (task.value("schema", std::string()) != Schema || !task.contains("drills") || !task["drills"].is_array())
        return out;
    for (const nlohmann::json& d : task["drills"]) {
        Drill drill;
        drill.id = d.value("id", std::string());
        drill.name = d.value("displayName", drill.id);
        drill.phase = d.value("phase", 1);
        if (!drill.id.empty()) out.push_back(std::move(drill));
    }
    return out;
}

bool LocomotionViewer3D::isLocomotionTask(const std::filesystem::path& taskPath) {
    return readTask(taskPath).value("schema", std::string()) == Schema;
}

void LocomotionViewer3D::setPolicy(const std::filesystem::path& file, std::string& label) {
    Impl& impl = *m_impl;
    try {
        impl.policy = std::make_shared<LocomotionPolicy3D>(impl.bundle->profile(), file);
        impl.runtime.setLocomotionPolicy(impl.policy);
        label = "rodada " + std::to_string(impl.policy->iteration());
    } catch (const std::exception& error) {
        impl.policy.reset();
        impl.runtime.setLocomotionPolicy(nullptr);
        label = error.what();
    }
}

void LocomotionViewer3D::clearPolicy() {
    m_impl->policy.reset();
    m_impl->runtime.setLocomotionPolicy(nullptr);
}

bool LocomotionViewer3D::hasPolicy() const { return m_impl->policy != nullptr; }

void LocomotionViewer3D::step() {
    Impl& impl = *m_impl;
    impl.command = impl.pilotCommand();
    for (int k = 0; k < TicksPerDecision; ++k) impl.tick();
    ++impl.steps;
    // A queda como no treino (a pelve a menos de 0,45 m do chao ou o atleta
    // "caido"): o episodio acaba. Fica meio segundo no chao para se ver a
    // queda e recomeca - o levantar do classico (com a ajuda) nao esconde a queda.
    const bool down = impl.body.links.empty() || impl.body.links.front().position.z < 0.45f
        || athleteTelemetry3D(impl.athlete).fallen;
    impl.fallenSeconds = down ? impl.fallenSeconds + impl.policyStepSecondsValue() : 0.0f;
    if (impl.fallenSeconds > 0.5f) impl.build(impl.rng());
}

void LocomotionViewer3D::reset(std::uint64_t seed) { m_impl->build(seed); }

void LocomotionViewer3D::setManualSpeed(std::optional<float> metersPerSecond) { m_impl->manual = metersPerSecond; }

void LocomotionViewer3D::push(Vec3 force, float seconds) {
    m_impl->pushForce = force;
    m_impl->pushSeconds = seconds;
}

BodyWorldView3D LocomotionViewer3D::view() const {
    const Impl& impl = *m_impl;
    BodyWorldView3D view;
    view.state = &impl.body;
    view.origin = impl.origin;
    view.drill = impl.drillIndex;
    view.steps = impl.steps;
    view.command.moveVelocity = planar(impl.command.intent.requestedVelocityWorld);
    view.command.faceHeading = impl.command.facingYawRadians;
    view.hasTarget = impl.pilotState.hasTarget;
    view.target = impl.pilotState.target;
    view.targetStop = impl.pilotState.stopAtTarget;
    view.velocity = planar(impl.centerOfMassVelocity());
    if (impl.chest < impl.body.links.size() && impl.chest < impl.bundle->profile().links.size()) {
        const Quaternion r = (impl.body.links[impl.chest].orientation
            * impl.bundle->profile().links[impl.chest].modelOrientation.conjugate()).normalized();
        const Vec3 forward = r.rotate({ 1.0f, 0.0f, 0.0f });
        view.chestHeading = std::atan2(forward.y, forward.x);
    }
    if (impl.pushSeconds > 0.0f) {
        view.pushLink = static_cast<int>(impl.chest);
        view.pushForce = impl.pushForce;
    }
    return view;
}

const RagdollProfile3D& LocomotionViewer3D::profile() const { return m_impl->bundle->profile(); }

} // namespace MatterEngine
