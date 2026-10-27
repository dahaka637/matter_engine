#include "Engine/Athlete/Training/LocomotionWorld3D.hpp"

#include "Engine/Materials/MaterialLibrary.hpp"

#include <algorithm>
#include <chrono>
#include <span>
#include <stdexcept>
#include <utility>
#include <vector>

namespace MatterEngine {

void createLocomotionGround3D(PhysicsScene3D& scene, float halfExtentMeters) {
    PhysicsShape3D shape;
    shape.type = PhysicsShapeType3D::Box;
    shape.halfExtents = { halfExtentMeters, halfExtentMeters, 0.1f };
    shape.materialId = "concrete";
    PhysicsBodyDefinition3D ground;
    ground.motionType = PhysicsMotionType3D::Static;
    ground.position = { 0.0f, 0.0f, -0.1f };
    ground.materialId = "concrete";
    (void)scene.createBody(ground, std::span<const PhysicsShape3D>(&shape, 1));
}

void createLocomotionStaticBox3D(PhysicsScene3D& scene, const LocomotionStaticBox3D& box) {
    PhysicsShape3D shape;
    shape.type = PhysicsShapeType3D::Box;
    shape.halfExtents = box.halfExtents;
    shape.materialId = "concrete";
    PhysicsBodyDefinition3D body;
    body.motionType = PhysicsMotionType3D::Static;
    body.position = box.center;
    body.orientation = box.orientation;
    body.materialId = "concrete";
    (void)scene.createBody(body, std::span<const PhysicsShape3D>(&shape, 1));
}

void createLocomotionThrow3D(PhysicsScene3D& scene, const LocomotionThrow3D& item) {
    PhysicsShape3D shape;
    shape.type = PhysicsShapeType3D::Box;
    shape.halfExtents = item.halfExtents;
    shape.materialId = "wood";
    PhysicsBodyDefinition3D box;
    box.position = item.position;
    box.linearVelocity = item.velocity;
    box.massKg = item.massKg;
    box.materialId = "wood";
    box.characteristicSizeMeters = 2.0f * std::max({ item.halfExtents.x, item.halfExtents.y, item.halfExtents.z });
    (void)scene.createBody(box, std::span<const PhysicsShape3D>(&shape, 1));
}

RagdollHandle3D spawnLocomotionAthlete3D(PhysicsScene3D& scene, const RagdollProfile3D& profile,
    std::uint64_t entityId) {
    RagdollSpawnDefinition3D spawn;
    spawn.entityId = entityId;
    spawn.active = true;
    spawn.pelvisPosition = { 0.0f, 0.0f, profile.standingRootHeightMeters };
    return scene.createRagdoll(profile, spawn);
}

struct LocomotionWorld3D::Impl {
    PhysicsEngine3D& engine;
    const MaterialLibrary& materials;
    AthleteRuntimeAssets3D assets;
    // Os do proximo comeco (setAssets).
    AthleteRuntimeAssets3D nextAssets;
    LocomotionWorldSettings3D settings;
    std::unique_ptr<PhysicsScene3D> scene;
    RagdollHandle3D handle;
    RagdollState3D body;
    AthleteState3D athlete;
    AthleteRuntime3D runtime;
    std::vector<std::pair<std::uint32_t, Vec3>> pushes;
    std::vector<LocomotionThrow3D> throws;
    std::uint64_t ticks = 0;
    double athleteSeconds = 0.0;
    double physicsSeconds = 0.0;

    Impl(PhysicsEngine3D& engineIn, const MaterialLibrary& materialsIn, const AthleteRuntimeAssets3D& assetsIn,
        const LocomotionWorldSettings3D& settingsIn)
        : engine(engineIn), materials(materialsIn), assets(assetsIn), nextAssets(assetsIn), settings(settingsIn) {}

    // A cena nova: o chao e o corpo (a mesma ordem da sonda: os ids dos
    // corpos e a ordem do solver iguais).
    void build() {
        assets = nextAssets;
        scene.reset();
        scene = engine.createScene(settings.scene, materials);
        createLocomotionGround3D(*scene, settings.groundHalfExtentMeters);
        for (const LocomotionStaticBox3D& box : settings.statics) createLocomotionStaticBox3D(*scene, box);
        handle = spawnLocomotionAthlete3D(*scene, *assets.profile);
        body = scene->ragdollState(handle);
        athlete = AthleteState3D {};
        runtime = AthleteRuntime3D {};
        pushes.clear();
        throws.clear();
        ticks = 0;
        athleteSeconds = physicsSeconds = 0.0;
    }
};

LocomotionWorld3D::LocomotionWorld3D(PhysicsEngine3D& engine, const MaterialLibrary& materials,
    const AthleteRuntimeAssets3D& assets, const LocomotionWorldSettings3D& settings)
    : m_impl(std::make_unique<Impl>(engine, materials, assets, settings)) {
    if (assets.profile == nullptr || assets.clips == nullptr || assets.defaultClips == nullptr)
        throw std::invalid_argument("LocomotionWorld3D: faltam o perfil ou os clipes");
    reset(AthleteStartMode3D::Physical);
}

LocomotionWorld3D::~LocomotionWorld3D() = default;

void LocomotionWorld3D::reset(AthleteStartMode3D mode) {
    Impl& w = *m_impl;
    w.build();
    athleteStart3D(*w.scene, w.handle, *w.assets.profile, w.body, w.athlete, mode);
}

void LocomotionWorld3D::restore(const AthleteSnapshot3D& snapshot) {
    Impl& w = *m_impl;
    w.build();
    athleteRestore3D(*w.scene, w.handle, snapshot, w.athlete, w.body);
}

AthleteSnapshot3D LocomotionWorld3D::snapshot() const { return athleteSnapshot3D(m_impl->athlete, m_impl->body); }

void LocomotionWorld3D::push(std::uint32_t link, Vec3 force) { m_impl->pushes.push_back({ link, force }); }
void LocomotionWorld3D::throwBox(const LocomotionThrow3D& item) { m_impl->throws.push_back(item); }
void LocomotionWorld3D::setStatics(std::vector<LocomotionStaticBox3D> statics) {
    m_impl->settings.statics = std::move(statics);
}
void LocomotionWorld3D::setAssets(const AthleteRuntimeAssets3D& assets) {
    if (assets.profile == nullptr || assets.clips == nullptr || assets.defaultClips == nullptr)
        throw std::invalid_argument("LocomotionWorld3D: faltam o perfil ou os clipes");
    m_impl->nextAssets = assets;
}

void LocomotionWorld3D::step(const AthleteCommand3D& command) {
    Impl& w = *m_impl;
    using Clock = std::chrono::steady_clock;
    const auto start = Clock::now();
    AthleteSlot3D slot { 1, w.handle, &w.body, true, false, &w.athlete };
    slot.command = &command;
    // Ninguem e o controlado (sem a capsula do jogador).
    AthleteControlInput3D control;
    control.controlledEntityId = 0;
    AthleteRuntimeOutput3D output;
    w.runtime.update(*w.scene, w.assets, std::span<AthleteSlot3D>(&slot, 1), {}, w.settings.runtime, control,
        w.settings.tickSeconds, output);
    for (const LocomotionThrow3D& item : w.throws) createLocomotionThrow3D(*w.scene, item);
    w.throws.clear();
    for (const auto& [link, force] : w.pushes) w.scene->applyRagdollLinkForce(w.handle, link, force, {});
    w.pushes.clear();
    const auto stepped = Clock::now();
    w.scene->simulate(w.settings.tickSeconds);
    w.body = w.scene->ragdollState(w.handle);
    const auto end = Clock::now();
    w.athleteSeconds = std::chrono::duration<double>(stepped - start).count();
    w.physicsSeconds = std::chrono::duration<double>(end - stepped).count();
    ++w.ticks;
}

AthleteState3D& LocomotionWorld3D::athlete() { return m_impl->athlete; }
const AthleteState3D& LocomotionWorld3D::athlete() const { return m_impl->athlete; }
const RagdollState3D& LocomotionWorld3D::body() const { return m_impl->body; }
AthleteTelemetry3D LocomotionWorld3D::telemetry() const { return athleteTelemetry3D(m_impl->athlete); }
PhysicsScene3D& LocomotionWorld3D::scene() { return *m_impl->scene; }
RagdollHandle3D LocomotionWorld3D::handle() const { return m_impl->handle; }
const LocomotionWorldSettings3D& LocomotionWorld3D::settings() const { return m_impl->settings; }
AthleteRuntimeSettings3D& LocomotionWorld3D::runtimeSettings() { return m_impl->settings.runtime; }
std::uint64_t LocomotionWorld3D::ticks() const { return m_impl->ticks; }
double LocomotionWorld3D::lastAthleteSeconds() const { return m_impl->athleteSeconds; }
double LocomotionWorld3D::lastPhysicsSeconds() const { return m_impl->physicsSeconds; }

} // namespace MatterEngine
