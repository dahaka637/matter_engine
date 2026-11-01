#include "Workbench/ReinforcementLearning/LocomotionViewer3D.hpp"

#include "Engine/Athlete/AthleteTelemetry3D.hpp"
#include "Engine/Athlete/Physical/PhysicalCorrections3D.hpp"
#include "Engine/Physique/PlayerPhysique3D.hpp"
#include "Engine/Tracking/TrackingGetUpPolicy3D.hpp"
#include "Workbench/ReinforcementLearning/TrackingViewer3D.hpp"
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
    std::optional<Vec3> manualTarget;
    float manualTargetSpeed = 0.0f;
    AthleteCommand3D command;
    Vec3 origin;
    std::uint32_t steps = 0;
    float fallenSeconds = 0.0f;
    Vec3 pushForce;
    float pushSeconds = 0.0f;
    // As pancadas do exercicio (como no treino: pelve, abdomen e peito) e o
    // levantar (a pancada que derruba no comeco; recomeca depois de ficar de pe).
    float drillPushRate = 0.0f;
    // O empurrao continuo do exercicio (como no treino).
    float holdRate = 0.0f, holdLow = 150.0f, holdHigh = 350.0f, holdSecondsLow = 1.0f, holdSecondsHigh = 3.0f;
    float drillTotal = 0.0f;
    float drillPushLow = 150.0f, drillPushHigh = 600.0f;
    float drillPushSecondsLow = 0.10f, drillPushSecondsHigh = 0.20f, drillPushStart = 1.0f;
    // O empurrao unico (como no treino): quando vem e quando recomeca (s; < 0: nada marcado).
    bool singlePush = false;
    float pushWindow = 1.5f, settleSeconds = 3.0f, pushAt = -1.0f, settleEnd = -1.0f;
    bool getUp = false;
    float knockdownLow = 1600.0f, knockdownHigh = 2400.0f;
    Vec3 drillForce;
    float drillSeconds = 0.0f;
    float standingSeconds = 0.0f;
    // O ultimo empurrao (o desenho): a forca, o link e ha quanto acabou (s).
    Vec3 shownForce;
    int shownLink = -1;
    float shownAfter = 99.0f;
    std::size_t chest = 0;
    float groundHalfExtent = 60.0f;
    // A interligacao das redes nas quedas (01/11, o "getUpHandoff" da tarefa; como o treino): na queda, a rede do
    // levantar exportada por ultimo dirige o corpo como no treino dela ate ficar de pe e firme, e devolve a esta - os
    // alvos do classico saindo dos dela - com o exercicio recomecando dali; sem levantar em 8 s, o recomeco de sempre.
    bool handoff = false;
    std::shared_ptr<const GetUpPolicy3D> getUpPolicy;
    std::filesystem::file_time_type getUpWrite {};
    bool recovering = false;
    std::uint32_t getUpTicks = 0;
    float firmSeconds = 0.0f, recoverSeconds = 0.0f;
    std::vector<RagdollDriveTarget3D> getUpDrives;
    std::uint32_t recoveryCount = 0;
    bool recoveryTouched = false;
    // O pedido de deslocamento da queda (o do exercicio, como no treino) e se ja foi ao chao: a troca andando.
    Vec3 getUpVelocity;
    bool getUpDown = false;
    // A queda como o fim do episodio do treino, tick a tick (01/11: a cada decisao e sem os contatos, a troca saia
    // depois da do treino): a pelve abaixo de fallPelvisHeight (na escala do corpo) ou, com forbiddenContact, algo que
    // nao os pes no chao; e o classico caido no fim da decisao.
    float fallPelvisHeight = 0.45f;
    bool forbiddenContact = false;
    std::vector<bool> footLink;
    std::vector<bool> trunkLink;
    // A inclinacao do tronco na decisao anterior (a queda iminente pelo tronco, GetUpPolicy3D).
    float trunkTilt = 0.0f;
    bool trunkTiltValid = false;

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
        drillSeconds = 0.0f;
        standingSeconds = 0.0f;
        recovering = false;
        trunkTiltValid = false;
        shownLink = -1;
        shownAfter = 99.0f;
        if (getUp) {
            std::uniform_real_distribution<float> unit(0.0f, 1.0f);
            const float angle = unit(rng) * 6.28318530718f;
            const float newtons = knockdownLow + (knockdownHigh - knockdownLow) * unit(rng);
            drillForce = { std::cos(angle) * newtons, std::sin(angle) * newtons, 0.0f };
            drillSeconds = 0.25f;
        }
        settleEnd = -1.0f;
        pushAt = singlePush ? drillPushStart + pushWindow * std::uniform_real_distribution<float>(0.0f, 1.0f)(rng) : -1.0f;
    }

    // A rede do levantar exportada por ultimo (de novo quando o treino dela exporta outra).
    void loadGetUpPolicy() {
        std::error_code error;
        const auto write = std::filesystem::last_write_time(TrackingViewer3D::getUpExport(), error);
        if (error || (getUpPolicy && write == getUpWrite)) return;
        try {
            getUpPolicy = std::make_shared<TrackingGetUpPolicy3D>(TrackingViewer3D::getUpExport());
            getUpWrite = write;
        } catch (const std::exception&) {
        }
    }

    bool startRecovery() {
        if (!handoff || getUp) return false;
        loadGetUpPolicy();
        if (!getUpPolicy) return false;
        recovering = true;
        trunkTiltValid = false;
        getUpTicks = 0;
        firmSeconds = recoverSeconds = 0.0f;
        getUpVelocity = planar(command.intent.requestedVelocityWorld);
        getUpDown = false;
        command = {};
        ++recoveryCount;
        recoveryTouched = false;
        return true;
    }

    [[nodiscard]] bool fallTick() const {
        if (body.links.empty()) return true;
        if (body.links.front().position.z - athlete.groundHeight < fallPelvisHeight * bodyScale3D(bundle->body())) return true;
        if (!forbiddenContact) return false;
        for (const RagdollInteraction3D& contact : body.interactions)
            if (contact.linkIndex < footLink.size() && !footLink[contact.linkIndex]
                && contact.otherMotion == RagdollContactMotion3D::Static && contact.normalWorld.z > 0.5f)
                return true;
        return false;
    }

    // A volta a esta rede: o atleta do zero no corpo de agora, os alvos do classico saindo dos dados e o exercicio
    // recomecando dali (o comeco ao vivo do treino; depois da troca andando, o primeiro pedido segue follow).
    void handBack(std::span<const RagdollDriveTarget3D> drives, Vec3 follow = {}) {
        const RagdollProfile3D& profile = bundle->profile();
        const PhysicsBodyState3D& pelvis = body.links.front();
        athlete = AthleteState3D {};
        runtime = AthleteRuntime3D {};
        runtime.setLocomotionPolicy(policy);
        athleteStart3D(*scene, handle, profile, body, athlete, AthleteStartMode3D::Physical);
        athlete.getUpHandoff = { drives.empty() ? 0.0f : GetUpPolicy3D::BlendSeconds,
            std::vector<RagdollDriveTarget3D>(drives.begin(), drives.end()) };
        origin = planar(pelvis.position);
        pilotState = PilotState3D {};
        CommandPilot3D::start(pilot, pilotState, origin, locomotionFrameYaw3D(profile, pelvis.orientation), rng);
        CommandPilot3D::follow(pilot, pilotState, follow, rng);
        steps = 0;
        fallenSeconds = drillSeconds = pushSeconds = 0.0f;
        settleEnd = -1.0f;
        pushAt = singlePush ? drillPushStart + pushWindow * std::uniform_real_distribution<float>(0.0f, 1.0f)(rng) : -1.0f;
        recovering = false;
        trunkTiltValid = false;
    }

    // Uma decisao da rede do levantar e, de pe e firme (o criterio do treino; com o pedido, depois do chao, de pe por
    // GetUpPolicy3D::MovingHandoffSeconds), a troca.
    void recover() {
        const RagdollProfile3D& profile = bundle->profile();
        const bool moving = getUpDown && getUpVelocity.length() > GetUpPolicy3D::MovingRequestSpeed;
        for (int k = 0; k < TicksPerDecision; ++k) {
            if (getUpTicks++ % GetUpPolicy3D::TicksPerDecision == 0)
                getUpPolicy->decide(profile, body, getUpDrives, moving ? getUpVelocity : Vec3 {}, athlete.groundHeight);
            scene->setRagdollActiveDriveTargets(handle, getUpDrives, false);
            scene->setRagdollFrictionScale(handle, 1.0f);
            scene->simulate(TickSeconds);
            body = scene->ragdollState(handle);
            shownAfter += TickSeconds;
            for (const RagdollInteraction3D& contact : body.interactions)
                if (contact.linkIndex < trunkLink.size() && trunkLink[contact.linkIndex]
                    && contact.otherMotion == RagdollContactMotion3D::Static)
                    recoveryTouched = true;
        }
        ++steps;
        const float decision = policyStepSecondsValue();
        recoverSeconds += decision;
        const PhysicsBodyState3D& pelvis = body.links.front();
        const float upright =
            (pelvis.orientation * profile.links.front().modelOrientation.conjugate()).rotate({ 0.0f, 0.0f, 1.0f }).z;
        getUpDown = getUpDown
            || pelvis.position.z - athlete.groundHeight < GetUpPolicy3D::DownRootFraction * profile.standingRootHeightMeters;
        const bool standing = pelvis.position.z > 0.85f * profile.standingRootHeightMeters && upright > 0.85f
            && (moving || planar(centerOfMassVelocity()).length() < 0.5f);
        firmSeconds = standing ? firmSeconds + decision : 0.0f;
        if (firmSeconds < (moving ? GetUpPolicy3D::MovingHandoffSeconds : GetUpPolicy3D::FirmSeconds)) {
            if (recoverSeconds > 8.0f) build(rng());
            return;
        }
        handBack(getUpDrives, moving ? getUpVelocity : Vec3 {});
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
        // (Ate o ponto dado: a camera para ele e a frente do analogico; perto dele, parado.)
        if (manualTarget && !body.links.empty()) {
            const Vec3 to = planar(*manualTarget - body.links.front().position);
            const float distance = std::hypot(to.x, to.y);
            input.cameraHeading = distance > 1.0e-3f ? std::atan2(to.y, to.x) : input.cameraHeading;
            input.stickForward = distance > 0.6f ? 1.0f : 0.0f;
            input.stickLeft = 0.0f;
            input.speed = distance > 0.6f ? manualTargetSpeed : 0.0f;
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
        if (pushSeconds > 0.0f) {
            shownForce = pushForce;
            shownLink = static_cast<int>(chest);
            shownAfter = 0.0f;
        } else if (drillSeconds > 0.0f) {
            shownForce = drillForce;
            shownLink = 2;
            shownAfter = 0.0f;
        } else {
            shownAfter += TickSeconds;
        }
        if (drillSeconds > 0.0f && body.links.size() > 3) {
            // (O empurrao longo entra e sai em 0,2 s, como no treino.)
            const float scale = drillTotal > 0.6f
                ? std::min({ 1.0f, (drillTotal - drillSeconds + TickSeconds) / 0.2f, drillSeconds / 0.2f }) : 1.0f;
            scene->applyRagdollLinkForce(handle, 0u, drillForce * (0.20f * scale), {});
            scene->applyRagdollLinkForce(handle, 2u, drillForce * (0.32f * scale), {});
            scene->applyRagdollLinkForce(handle, 3u, drillForce * (0.48f * scale), {});
            drillSeconds -= TickSeconds;
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
    // As pancadas e o levantar do exercicio (como no treino).
    if (task.contains("pushes")) {
        const nlohmann::json& p = task["pushes"];
        impl.drillPushLow = p.value("minimumNewtons", impl.drillPushLow);
        impl.drillPushHigh = p.value("maximumNewtons", impl.drillPushHigh);
        impl.drillPushSecondsLow = p.value("minimumSeconds", impl.drillPushSecondsLow);
        impl.drillPushSecondsHigh = p.value("maximumSeconds", impl.drillPushSecondsHigh);
        impl.drillPushStart = p.value("startSeconds", impl.drillPushStart);
        impl.pushWindow = p.value("windowSeconds", impl.pushWindow);
    }
    impl.drillPushRate = chosen.value("pushRate", 0.0f);
    impl.holdRate = chosen.value("holdRate", 0.0f);
    impl.holdLow = chosen.value("holdMinimumNewtons", impl.holdLow);
    impl.holdHigh = chosen.value("holdMaximumNewtons", impl.holdHigh);
    impl.holdSecondsLow = chosen.value("holdMinimumSeconds", impl.holdSecondsLow);
    impl.holdSecondsHigh = chosen.value("holdMaximumSeconds", impl.holdSecondsHigh);
    if (chosen.value("pushMinimumNewtons", -1.0f) >= 0.0f) impl.drillPushLow = chosen.value("pushMinimumNewtons", 0.0f);
    if (chosen.value("pushMaximumNewtons", -1.0f) >= 0.0f) impl.drillPushHigh = chosen.value("pushMaximumNewtons", 0.0f);
    impl.getUp = chosen.value("getUp", false);
    impl.singlePush = chosen.value("singlePush", false);
    impl.settleSeconds = chosen.value("settleSeconds", impl.settleSeconds);
    impl.knockdownLow = chosen.value("knockdownMinimumNewtons", impl.knockdownLow);
    impl.knockdownHigh = chosen.value("knockdownMaximumNewtons", impl.knockdownHigh);
    impl.handoff = task.contains("getUpHandoff") && task["getUpHandoff"].value("enabled", false);
    if (task.contains("termination")) impl.fallPelvisHeight = task["termination"].value("pelvisHeight", impl.fallPelvisHeight);
    if (task.contains("reward")) impl.forbiddenContact = task["reward"].value("forbiddenContact", false);
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
        impl.settings.getUpStiffnessPercent = a.value("getUpStiffness", 55.0f);
        impl.settings.getUpStrengthPercent = a.value("getUpStrength", 70.0f);
    }
    // A tabela de velocidades e o chao da tarefa (como no treino).
    if (task.contains("speeds")) {
        const nlohmann::json& v = task["speeds"];
        impl.settings.defaultTrotSpeed = v.value("trot", impl.settings.defaultTrotSpeed);
        impl.settings.defaultSprintForwardSpeed = v.value("sprintForward", impl.settings.defaultSprintForwardSpeed);
        impl.settings.defaultSprintSideSpeed = v.value("sprintSide", impl.settings.defaultSprintSideSpeed);
        impl.settings.defaultSprintBackwardSpeed = v.value("sprintBackward", impl.settings.defaultSprintBackwardSpeed);
    }
    // A regua de ajuda da tarefa (o zero antigo, se historica).
    impl.settings.historicalAssistRuler = task.value("assistRuler", std::string("historical")) != "linear";
    // As viradas da tarefa: a no tempo do corpo e a da corrida (como no treino).
    if (task.contains("movingHipTurn")) impl.settings.movingHipTurn = task["movingHipTurn"].value("enabled", false);
    if (task.contains("runningTurn")) impl.settings.runningTurnClip = task["runningTurn"].value("enabled", false);
    if (task.contains("termination"))
        impl.groundHalfExtent = task["termination"].value("groundHalfExtentMeters", impl.groundHalfExtent);
    impl.bundle = AthleteBundle3D::shared(assetsDirectory.string());
    impl.scheduler = std::make_shared<TaskScheduler>(TaskSchedulerSettings { 1u });
    impl.engine = std::make_unique<PhysicsEngine3D>(impl.scheduler);
    const RagdollProfile3D& profile = impl.bundle->profile();
    for (std::size_t i = 0; i < profile.links.size(); ++i) {
        const std::string& id = profile.links[i].id;
        if (id == "Chest") impl.chest = i;
        impl.footLink.push_back(id.find("Foot") != std::string::npos || id.find("Toe") != std::string::npos);
        impl.trunkLink.push_back(id == "Pelvis" || id == "Abdomen" || id == "Chest" || id == "UpperChest" || id == "Neck"
            || id == "Head");
    }
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
        drill.category = d.value("category", std::string());
        drill.weight = d.value("weight", 1.0f);
        drill.getUp = d.value("getUp", false);
        if (d.contains("pilot") && d["pilot"].is_object() && d["pilot"].value("kind", std::string()) != "hold"
            && d["pilot"].contains("bands") && d["pilot"]["bands"].is_array())
            for (const nlohmann::json& band : d["pilot"]["bands"])
                if (band.contains("speed") && band["speed"].is_array() && band["speed"].size() == 2)
                    drill.goalSpeed = std::max(drill.goalSpeed, band["speed"][1].get<float>());
        if (!drill.id.empty()) out.push_back(std::move(drill));
    }
    return out;
}

bool LocomotionViewer3D::isLocomotionTask(const std::filesystem::path& taskPath) {
    const auto task = readTask(taskPath);
    return task.is_object() && task.value("schema", std::string()) == Schema;
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

bool LocomotionViewer3D::gettingUp() const { return m_impl->recovering; }
std::uint32_t LocomotionViewer3D::recoveries() const { return m_impl->recoveryCount; }
bool LocomotionViewer3D::lastRecoveryAvoided() const { return !m_impl->recoveryTouched; }

void LocomotionViewer3D::resumeFrom(std::span<const PhysicsBodyState3D> links, std::span<const RagdollDriveTarget3D> drives) {
    Impl& impl = *m_impl;
    if (links.size() != impl.body.links.size()) return;
    impl.scene->resetRagdollState(impl.handle, links);
    impl.body = impl.scene->ragdollState(impl.handle);
    impl.shownLink = -1;
    impl.shownAfter = 99.0f;
    impl.handBack(drives);
}

void LocomotionViewer3D::step() {
    Impl& impl = *m_impl;
    if (impl.recovering) {
        impl.recover();
        return;
    }
    impl.command = impl.pilotCommand();
    const float decision = impl.policyStepSecondsValue();
    const float now = static_cast<float>(impl.steps) * decision;
    std::uniform_real_distribution<float> unit(0.0f, 1.0f);
    // 0: nenhum, 1: o curto, 2: o continuo (o sorteio do treino).
    int kind = 0;
    if (impl.singlePush) {
        if (impl.pushAt >= 0.0f && now >= impl.pushAt) {
            impl.pushAt = -1.0f;
            const float rates = impl.holdRate + impl.drillPushRate;
            kind = rates > 0.0f && unit(impl.rng) * rates < impl.holdRate ? 2 : 1;
        }
    } else if ((impl.drillPushRate > 0.0f || impl.holdRate > 0.0f) && impl.drillSeconds <= 0.0f && now >= impl.drillPushStart) {
        const float hold = 1.0f - std::exp(-impl.holdRate * decision);
        const float impulse = 1.0f - std::exp(-impl.drillPushRate * decision);
        const float u = unit(impl.rng);
        if (u < hold + impulse) kind = u < hold ? 2 : 1;
    }
    if (kind != 0) {
        const bool continuous = kind == 2;
        const float angle = unit(impl.rng) * 6.28318530718f;
        const float newtons = continuous ? impl.holdLow + (impl.holdHigh - impl.holdLow) * unit(impl.rng)
                                         : impl.drillPushLow + (impl.drillPushHigh - impl.drillPushLow) * unit(impl.rng);
        impl.drillForce = { std::cos(angle) * newtons, std::sin(angle) * newtons, 0.0f };
        impl.drillSeconds = continuous
            ? impl.holdSecondsLow + (impl.holdSecondsHigh - impl.holdSecondsLow) * unit(impl.rng)
            : impl.drillPushSecondsLow + (impl.drillPushSecondsHigh - impl.drillPushSecondsLow) * unit(impl.rng);
        impl.drillTotal = impl.drillSeconds;
        if (impl.singlePush) impl.settleEnd = now + impl.drillSeconds + impl.settleSeconds;
    }
    bool fell = false;
    for (int k = 0; k < TicksPerDecision && !fell; ++k) {
        impl.tick();
        fell = !impl.getUp && impl.fallTick();
    }
    ++impl.steps;
    if (impl.getUp) {
        // O levantar: recomeca 2 s depois de ficar de pe (ou em 16 s).
        const bool standing = !impl.body.links.empty() && impl.body.links.front().position.z > 0.8f
            && !athleteTelemetry3D(impl.athlete).fallen;
        impl.standingSeconds = standing && impl.steps * decision > 1.0f ? impl.standingSeconds + decision : 0.0f;
        if (impl.standingSeconds > 2.0f || static_cast<float>(impl.steps) * decision > 16.0f) impl.build(impl.rng());
        return;
    }
    // A queda como no treino (a pelve a menos de 0,45 m do chao ou o atleta
    // "caido"): o episodio acaba. Fica meio segundo no chao para se ver a
    // queda e recomeca - o levantar do classico (com a ajuda) nao esconde a queda.
    bool imminent = false;
    if (impl.handoff && !impl.getUp && !impl.body.links.empty()) {
        const float tilt = GetUpPolicy3D::trunkTilt(impl.bundle->profile(), impl.body.links);
        imminent = impl.trunkTiltValid && tilt > GetUpPolicy3D::ImminentTiltDegrees
            && (tilt - impl.trunkTilt) / decision > GetUpPolicy3D::ImminentTiltRate;
        impl.trunkTilt = tilt;
        impl.trunkTiltValid = true;
    }
    const bool down = fell || athleteTelemetry3D(impl.athlete).fallen;
    if ((down || imminent) && !impl.body.links.empty() && impl.startRecovery()) return;
    impl.fallenSeconds = down ? impl.fallenSeconds + impl.policyStepSecondsValue() : 0.0f;
    // (O empurrao unico aguentado: recomeca settleSeconds depois dele, como o fim do episodio no treino.)
    const bool settled = !down && impl.settleEnd >= 0.0f && static_cast<float>(impl.steps) * decision >= impl.settleEnd - 1.0e-4f;
    if (impl.fallenSeconds > 0.5f || settled) impl.build(impl.rng());
}

void LocomotionViewer3D::reset(std::uint64_t seed) { m_impl->build(seed); }

void LocomotionViewer3D::setAssist(float percent) {
    const float p = std::clamp(percent, 0.0f, 100.0f);
    m_impl->settings.e5StandingAssistPercent = m_impl->settings.e5WalkingAssistPercent = p;
    m_impl->settings.physicalFeetAssistPercent = p;
    m_impl->settings.getUpAssistPercent = p;
}

float LocomotionViewer3D::assist() const { return m_impl->settings.e5WalkingAssistPercent; }

void LocomotionViewer3D::setManualSpeed(std::optional<float> metersPerSecond) { m_impl->manual = metersPerSecond; }

void LocomotionViewer3D::setManualTarget(std::optional<Vec3> target, float metersPerSecond) {
    m_impl->manualTarget = target;
    m_impl->manualTargetSpeed = metersPerSecond;
}

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
    view.hasTarget = impl.pilotState.hasTarget || impl.manualTarget.has_value();
    view.target = impl.manualTarget ? *impl.manualTarget : impl.pilotState.target;
    view.targetStop = impl.manualTarget ? true : impl.pilotState.stopAtTarget;
    view.velocity = planar(impl.centerOfMassVelocity());
    if (impl.chest < impl.body.links.size() && impl.chest < impl.bundle->profile().links.size()) {
        const Quaternion r = (impl.body.links[impl.chest].orientation
            * impl.bundle->profile().links[impl.chest].modelOrientation.conjugate()).normalized();
        const Vec3 forward = r.rotate({ 1.0f, 0.0f, 0.0f });
        view.chestHeading = std::atan2(forward.y, forward.x);
    }
    // O empurrao de agora (ou o ultimo, sumindo em 0,8 s).
    if (impl.shownLink >= 0 && static_cast<std::size_t>(impl.shownLink) < impl.body.links.size()
        && impl.shownAfter < 0.8f) {
        view.pushLink = impl.shownLink;
        view.pushForce = impl.shownForce;
        view.pushFade = 1.0f - impl.shownAfter / 0.8f;
    }
    return view;
}

const RagdollProfile3D& LocomotionViewer3D::profile() const { return m_impl->bundle->profile(); }

} // namespace MatterEngine
