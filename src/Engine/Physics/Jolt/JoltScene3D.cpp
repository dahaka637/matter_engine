#include "Engine/Physics/Jolt/JoltInternals3D.hpp"
#include "Engine/Materials/MaterialLibrary.hpp"
#include <Jolt/Physics/Body/BodyLock.h>
#include <Jolt/Physics/Body/BodyManager.h>
#include <chrono>
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace MatterEngine {
PhysicsScene3D::PhysicsScene3D(PhysicsEngine3D& engine,
    const PhysicsSceneSettings3D& settings, const MaterialLibrary& materials)
    : m_impl(std::make_unique<Impl>()) {
    auto& impl = *m_impl;
    impl.engine = engine.m_impl;
    impl.settings = settings;
    impl.jobSystem = std::make_unique<JoltJobSystem>(impl.engine->scheduler, 2048, 8);
    impl.tempAllocator = std::make_unique<JPH::TempAllocatorImpl>(
        settings.scratchBufferSizeBytes ? settings.scratchBufferSizeBytes : 10 * 1024 * 1024);
    impl.system = std::make_unique<JPH::PhysicsSystem>();
    impl.system->Init(settings.maximumBodies, settings.bodyMutexCount,
        settings.maximumBodyPairs, settings.maximumContactConstraints,
        impl.broadPhaseLayers, impl.objectVsBroadPhaseFilter, impl.objectLayerPairFilter);
    impl.system->SetGravity(toJolt(settings.gravity));
    auto tuning = impl.system->GetPhysicsSettings();
    if (settings.solverPositionIterations) tuning.mNumPositionSteps = settings.solverPositionIterations;
    if (settings.solverVelocityIterations) tuning.mNumVelocitySteps = settings.solverVelocityIterations;
    tuning.mBaumgarte = 0.2f;
    tuning.mPenetrationSlop = 0.001f;
    tuning.mMaxPenetrationDistance = 0.0025f;
    impl.system->SetPhysicsSettings(tuning);
    for (const auto& [id, material] : materials.all()) {
        impl.materials.emplace(id, Impl::SurfaceContact { material.contact.staticFriction,
            material.contact.dynamicFriction, material.contact.restitution });
    }
    impl.system->SetContactListener(&impl);
    impl.system->AddStepListener(&impl);
    impl.contactImpacts.reserve(1024);
    impl.contactSlides.reserve(1024);
}
PhysicsScene3D::~PhysicsScene3D() { clear(); }
PhysicsScene3D::Impl::~Impl() {
    if (engine) engine->scheduler->waitIdle();
    if (system) {
        system->SetContactListener(nullptr);
        system->RemoveStepListener(this);
    }
}
void PhysicsScene3D::clear() {
    if (!m_impl->system) return;
    endGrab();
    destroyCharacter();
    for (auto& slot : m_impl->ragdollSlots)
        if (slot.record) destroyRagdoll(slot.record->handle);
    for (auto& slot : m_impl->bodySlots)
        if (slot.record) destroyBody(slot.record->handle);
    m_impl->pendingRagdollCommands.clear();
    m_impl->pendingBodyWrenches.clear();
    m_impl->contactImpacts.clear();
    m_impl->contactSlides.clear();
    m_impl->activeBodyStateUpdates.clear();
}
const PhysicsScene3D::Impl::SurfaceContact& PhysicsScene3D::Impl::contactMaterial(
    const std::string& id) const {
    auto it = materials.find(id);
    if (it != materials.end()) return it->second;
    static const SurfaceContact fallback;
    return fallback;
}
void PhysicsScene3D::simulate(float dt) {
    if (!std::isfinite(dt) || dt <= 0) return;
    auto& impl = *m_impl;
    using Clock = std::chrono::steady_clock;
    const auto begin = Clock::now();
    impl.stepDeltaTime = dt;
    // Um collision step por tick. O controlador roda a 120 Hz fixos, e o
    // passo do Jolt ja e pequeno nessa frequencia: subpassos refaziam deteccao
    // de colisao, construcao de ilhas e integracao sem mudar nada que o
    // controlador veja. A versao anterior usava 4 sempre que havia ragdoll,
    // e com 22 bonecos ativos em contato isso levava o passo a P50 7,1 ms /
    // P95 8,7 ms - acima do orcamento de 8,33 ms por tick, o lag visto no
    // aplicativo. Coesao das juntas e resolvida por iteracoes (ver
    // JoltRagdoll3D), que custam bem menos que refazer o passo inteiro.
    impl.collisionSteps = 1;
    impl.contactImpacts.clear();
    impl.contactSlides.clear();
    for (auto& slot : impl.ragdollSlots) if (slot.record) slot.record->stepContacts.clear();
    impl.reportedContactPairs = 0;
    impl.reportedContactPoints = 0;
    impl.allContactPairs = 0;
    impl.callbackFailed = false;
    impl.jobSystem->resetStepCounters();
    impl.applyPendingRagdollCommands();
    impl.applyRagdollDrives();
    // Wake commanded bodies before Update: its step listener is skipped for an empty active set.
    auto& bodies = impl.system->GetBodyInterface();
    for (const auto& wrench : impl.pendingBodyWrenches) bodies.ActivateBody(wrench.body);
    impl.system->GetActiveBodies(JPH::EBodyType::RigidBody, impl.previouslyActiveBodies);
    const auto dispatch = Clock::now();
    const auto error = impl.system->Update(dt, impl.collisionSteps, impl.tempAllocator.get(), impl.jobSystem.get());
    // Barrier completion can precede release of the scheduler's last Job reference.
    impl.engine->scheduler->waitIdle();
    if (error != JPH::EPhysicsUpdateError::None || impl.callbackFailed)
        throw std::runtime_error("Jolt: simulation capacity exceeded or callback failed");
    const auto sync = Clock::now();
    impl.consolidateContacts();
    // Precisa dos contatos do passo (interferencia externa) e precisa vir
    // antes da publicacao, para que o estado lido pelo controlador no proximo
    // tick ja esteja sobre o guia - a mesma ordem do backend PhysX.
    impl.resolveRagdollGuides();
    impl.publishStepResults();
    const auto stats = impl.system->GetBodyStats();
    auto& d = impl.diagnostics;
    d.staticBodyCount = stats.mNumBodiesStatic;
    d.dynamicBodyCount = stats.mNumBodiesDynamic;
    d.activeDynamicBodyCount = stats.mNumActiveBodiesDynamic;
    d.sleepingDynamicBodyCount = d.dynamicBodyCount - d.activeDynamicBodyCount;
    d.discreteContactPairs = impl.allContactPairs;
    d.reportedContactPairs = impl.reportedContactPairs;
    d.reportedContactPoints = impl.reportedContactPoints;
    d.submittedPhysicsTasks = impl.jobSystem->submittedJobCount();
    d.physicsWorkerCount = impl.engine->scheduler->workerCount();
    const auto ms = [](auto a, auto b) { return std::chrono::duration<float, std::milli>(b-a).count(); };
    d.preSimulationMilliseconds = ms(begin, dispatch);
    d.simulationDispatchMilliseconds = ms(dispatch, sync);
    d.stateSyncMilliseconds = ms(sync, Clock::now());
    d.totalStepMilliseconds = ms(begin, Clock::now());
}
void PhysicsScene3D::Impl::publishStepResults() {
    activeBodyStateUpdates.clear();
    system->GetActiveBodies(JPH::EBodyType::RigidBody, activeBodies);
    activeBodies.insert(activeBodies.end(), previouslyActiveBodies.begin(), previouslyActiveBodies.end());
    std::sort(activeBodies.begin(), activeBodies.end());
    activeBodies.erase(std::unique(activeBodies.begin(), activeBodies.end()), activeBodies.end());
    for (auto id : activeBodies) {
        auto* record = bodyFromId(id);
        if (!record) continue;
        auto& bi = system->GetBodyInterface();
        PhysicsBodyState3D state;
        state.position = fromJolt(bi.GetPosition(id));
        state.orientation = fromJolt(bi.GetRotation(id));
        state.linearVelocity = fromJolt(bi.GetLinearVelocity(id));
        state.angularVelocity = fromJolt(bi.GetAngularVelocity(id));
        state.frozen = record->frozen;
        state.sleeping = !bi.IsActive(id);
        activeBodyStateUpdates.push_back({record->handle, state});
    }
    for (auto& slot : ragdollSlots) if (slot.record) publishRagdoll(*slot.record);
}
void PhysicsScene3D::setAirVelocity(Vec3 value) { m_impl->settings.airVelocity = value; }
void PhysicsScene3D::setOcean(const OceanVolume3D& value) { m_impl->ocean = value; }
void PhysicsScene3D::clearOcean() { m_impl->ocean.reset(); }
void PhysicsScene3D::setOceanTimeSeconds(float value) { m_impl->oceanTimeSeconds = value; }
std::span<const ContactImpactEvent3D> PhysicsScene3D::contactImpacts() const { return m_impl->contactImpacts; }
std::span<const ContactSlideEvent3D> PhysicsScene3D::contactSlides() const { return m_impl->contactSlides; }
std::span<const PhysicsBodyStateUpdate3D> PhysicsScene3D::activeBodyStates() const { return m_impl->activeBodyStateUpdates; }
const PhysicsStepDiagnostics3D& PhysicsScene3D::diagnostics() const { return m_impl->diagnostics; }
} // namespace MatterEngine
