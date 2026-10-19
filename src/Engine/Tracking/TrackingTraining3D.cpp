#include "Engine/Tracking/TrackingTraining3D.hpp"

#include "Engine/Athlete/AthleteAssets3D.hpp"
#include "Engine/Core/TaskScheduler.hpp"
#include "Engine/Materials/MaterialLibrary.hpp"
#include "Engine/Physics/PhysicsEngine3D.hpp"
#include "Engine/RL/Body/RLBody3D.hpp"
#include "Engine/Tracking/TrackingContract3D.hpp"
#include "Engine/Tracking/TrackingReward3D.hpp"
#include "Engine/Tracking/TrackingScene3D.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <mutex>
#include <random>
#include <sstream>
#include <stdexcept>
#include <thread>

namespace MatterEngine {
namespace {

constexpr float Pi = 3.14159265358979f;
// DReCon 6.7: a chance de uma troca do jogador virtual a cada quadro (60 Hz).
constexpr float EventChancePerFrame = 0.001f;
// O lider volta para perto da origem (num comeco de episodio) passando disto (m).
constexpr float RecenterDistance = 300.0f;
constexpr float GroundHalfExtent = 400.0f;

float uniform(std::mt19937& rng, float lo, float hi) { return std::uniform_real_distribution<float>(lo, hi)(rng); }
bool chance(std::mt19937& rng, float p) { return uniform(rng, 0.0f, 1.0f) < p; }

} // namespace

struct TrackingTraining3D::Impl {
    TrackingTrainingSettings3D settings;
    std::shared_ptr<const AthleteBundle3D> bundle;
    const RagdollProfile3D* profile = nullptr;
    AthleteRuntimeSettings3D runtime;
    std::unique_ptr<TrackingContract3D> contract;
    std::uint32_t workers = 1;
    std::shared_ptr<TaskScheduler> scheduler;
    std::unique_ptr<PhysicsEngine3D> engine;
    MaterialLibrary materials;

    struct World {
        std::unique_ptr<PhysicsScene3D> scene;
        RagdollHandle3D handle;
        std::unique_ptr<TrackingLeader3D> leader;
        std::unique_ptr<TrackingReward3D> reward;
        std::mt19937 rng;
        // O jogador virtual: o rumo em relacao ao olhar, a velocidade, o giro do
        // olhar e o estilo.
        TrackingCommand3D command;
        float relativeDirection = 0.0f;
        float speed = 0.0f;
        float turnRate = 0.0f;
        bool sprint = false;
        std::uint64_t ticks = 0;
        std::array<float, TrackingContract3D::ActionSize> filtered {};
        std::vector<RagdollDriveTarget3D> drives;
        const RagdollState3D* body = nullptr;
        std::vector<float> observation;
        std::vector<float> finalObservation;
        // O projetil do DReCon (7.1): o cubo da vez e o relogio dele.
        PhysicsBodyHandle3D projectile;
        bool hasProjectile = false;
        float projectileClock = 0.0f;
        // Um empurrao (o visualizador): o link, a forca e o tempo que falta.
        std::uint32_t pushLink = 0;
        Vec3 pushForce;
        float pushSeconds = 0.0f;
        // O episodio.
        float seconds = 0.0f;
        double rewardSum = 0.0, comErrorSum = 0.0, speedSum = 0.0, requestedSum = 0.0, headSum = 0.0;
        std::uint32_t decisions = 0;
    };
    std::vector<World> worlds;
    std::vector<TrackingEpisode3D> episodes;
    std::mutex episodeMutex;

    // Relanca o cubo (DReCon 7.1): o de antes sai, um novo nasce a 2 m e voa a
    // 5 m/s (na horizontal) ate um ponto de -0,5 a 0,5 m na vertical do centro de massa.
    void launchProjectile(World& w) const {
        if (w.hasProjectile) w.scene->destroyBody(w.projectile);
        const Vec3 com = rlBodyCenterOfMass3D(*profile, w.body->links);
        const float angle = uniform(w.rng, -Pi, Pi);
        const Vec3 target = com + Vec3 { 0.0f, 0.0f, uniform(w.rng, -0.5f, 0.5f) };
        const Vec3 start = target + Vec3 { 2.0f * std::cos(angle), 2.0f * std::sin(angle), 0.0f };
        const float flight = 2.0f / 5.0f;
        PhysicsShape3D shape;
        shape.type = PhysicsShapeType3D::Box;
        shape.halfExtents = { 0.1f, 0.1f, 0.1f };
        shape.materialId = "wood";
        PhysicsBodyDefinition3D box;
        box.position = start;
        // (A gravidade no voo: sobe g t / 2 para chegar no ponto.)
        box.linearVelocity = (target - start) * (1.0f / flight) + Vec3 { 0.0f, 0.0f, 0.5f * 9.81f * flight };
        box.massKg = uniform(w.rng, settings.projectileMinimumKg, settings.projectileMaximumKg);
        box.materialId = "wood";
        box.characteristicSizeMeters = 0.2f;
        w.projectile = w.scene->createBody(box, std::span<const PhysicsShape3D>(&shape, 1));
        w.hasProjectile = true;
    }

    // O pedido do jogador virtual de agora em velocidade no mundo.
    void composeCommand(World& w) const {
        const float yaw = w.command.facingYawRadians + w.relativeDirection;
        w.command.requestedVelocityWorld = { w.speed * std::cos(yaw), w.speed * std::sin(yaw), 0.0f };
    }

    // Uma troca do jogador virtual (DReCon 6.7: rumo, olhar, giro, velocidade, estilo).
    void changeCommand(World& w) const {
        if (settings.evaluation) return;
        if (settings.scenario == TrackingScenario3D::Forward) {
            w.relativeDirection = 0.0f;
            w.turnRate = 0.0f;
            w.sprint = false;
            w.speed = uniform(w.rng, 0.0f, settings.forwardMaximumSpeed);
        } else {
            w.relativeDirection = uniform(w.rng, -Pi, Pi);
            if (chance(w.rng, 0.5f)) w.command.facingYawRadians = uniform(w.rng, -Pi, Pi);
            w.turnRate = chance(w.rng, 0.3f) ? uniform(w.rng, -1.5f, 1.5f) : 0.0f;
            w.sprint = settings.allowSprint && chance(w.rng, 0.5f);
            const float top = athleteDefaultTravelSpeed3D(runtime, w.relativeDirection, w.sprint);
            w.speed = chance(w.rng, 0.15f) ? 0.0f : uniform(w.rng, 0.0f, top);
        }
        composeCommand(w);
    }

    // Um tick do lider com o jogador virtual (a troca a 60 Hz, o giro do olhar).
    void leaderTick(World& w) const {
        if (w.ticks % 2 == 0 && chance(w.rng, EventChancePerFrame)) changeCommand(w);
        if (settings.evaluation && settings.evaluationMode == TrackingEvaluation3D::Switches && w.ticks > 0) {
            // O protocolo do DReCon (7.1): um rumo novo a cada switchSeconds (o corpo vira e segue de frente).
            const auto every = static_cast<std::uint64_t>(std::max(1.0f, settings.switchSeconds / TickSeconds));
            if (w.ticks % every == 0) {
                w.command.facingYawRadians = uniform(w.rng, -Pi, Pi);
                composeCommand(w);
            }
        }
        if (w.turnRate != 0.0f) {
            w.command.facingYawRadians += w.turnRate * TickSeconds;
            composeCommand(w);
        }
        w.leader->step(w.command, TickSeconds);
        ++w.ticks;
    }

    // O lider recomeca parado perto da origem e aquece pelo jogador virtual.
    void restartLeader(World& w, float warmSeconds) const {
        w.leader->reset({}, w.command.facingYawRadians);
        const int warm = std::max(4, static_cast<int>(warmSeconds / TickSeconds));
        for (int k = 0; k < warm; ++k) leaderTick(w);
        if (!w.leader->state().valid) throw std::runtime_error("rastreador: o lider nao deu pose");
    }

    // O corpo comeca sobreposto ao lider: a pose e a velocidade dele (o RSI).
    void seedBody(World& w) const {
        const TrackingLeaderState3D& s = w.leader->state();
        w.scene->resetRagdollState(w.handle, s.links);
        w.body = w.scene->ragdollStateView(w.handle);
        w.filtered.fill(0.0f);
        w.seconds = 0.0f;
        w.rewardSum = w.comErrorSum = w.speedSum = w.requestedSum = w.headSum = 0.0;
        w.decisions = 0;
    }

    void observe(World& w) const {
        const RagdollState3D& body = *w.body;
        const Vec3 com = rlBodyCenterOfMass3D(*profile, body.links);
        const Vec3 comVelocity = rlBodyCenterOfMassVelocity3D(*profile, body.links);
        contract->observe(w.leader->state(), body.links, com, comVelocity, w.filtered, w.observation);
    }

    void startWorld(World& w, std::size_t index, std::uint64_t seed) const {
        w.rng.seed(static_cast<std::uint32_t>(seed * 2654435761ull + index * 40503ull + 12345ull));
        w.ticks = 0;
        w.command.facingYawRadians = uniform(w.rng, -Pi, Pi);
        if (settings.evaluation) {
            w.relativeDirection = settings.evaluationMode == TrackingEvaluation3D::Headings
                ? static_cast<float>(index % 8) * (Pi / 4.0f) : 0.0f;
            w.turnRate = 0.0f;
            w.sprint = false;
            w.speed = settings.evaluationSpeed;
            composeCommand(w);
        } else {
            changeCommand(w);
        }
        // (Aquecimento sorteado: o comeco cai em fases diferentes da passada.)
        restartLeader(w, settings.evaluation ? 2.0f : uniform(w.rng, 1.0f, 3.0f));
        seedBody(w);
        observe(w);
    }

    void decide(World& w, std::size_t index, std::span<const float> action, float& reward, std::uint8_t& terminated,
        std::uint8_t& truncated) {
        const float beta = settings.actionFilterBeta;
        for (std::size_t a = 0; a < w.filtered.size(); ++a)
            w.filtered[a] = beta * (std::isfinite(action[a]) ? action[a] : 0.0f) + (1.0f - beta) * w.filtered[a];
        for (int k = 0; k < TicksPerDecision; ++k) {
            leaderTick(w);
            contract->driveTargets(w.leader->state(), *w.body, w.filtered, w.drives);
            w.scene->setRagdollActiveDriveTargets(w.handle, w.drives, false);
            if (w.pushSeconds > 0.0f) {
                w.scene->applyRagdollLinkForce(w.handle, w.pushLink, w.pushForce, {});
                w.pushSeconds -= TickSeconds;
            }
            if (settings.projectiles && (w.projectileClock += TickSeconds) >= 1.0f) {
                w.projectileClock = 0.0f;
                launchProjectile(w);
            }
            w.scene->simulate(TickSeconds);
            w.body = w.scene->ragdollStateView(w.handle);
        }
        const RagdollState3D& body = *w.body;
        const Vec3 com = rlBodyCenterOfMass3D(*profile, body.links);
        const Vec3 comVelocity = rlBodyCenterOfMassVelocity3D(*profile, body.links);
        const TrackingLeaderState3D& leader = w.leader->state();
        const TrackingRewardTerms3D terms = w.reward->evaluate(leader, body.links, com, comVelocity);
        reward = terms.total;
        w.seconds += TicksPerDecision * TickSeconds;
        ++w.decisions;
        w.rewardSum += terms.total;
        const Vec3 error = comVelocity - leader.centerOfMassVelocity;
        w.comErrorSum += std::hypot(error.x, error.y);
        w.speedSum += std::hypot(comVelocity.x, comVelocity.y);
        w.requestedSum += std::hypot(leader.requestedVelocityWorld.x, leader.requestedVelocityWorld.y);
        w.headSum += terms.headDistance;
        terminated = terms.terminate ? 1 : 0;
        truncated = !terms.terminate && w.seconds >= settings.episodeSeconds - 1e-4f ? 1 : 0;
        if (terminated || truncated) {
            observe(w);
            w.finalObservation = w.observation;
            TrackingEpisode3D episode;
            episode.world = static_cast<std::uint32_t>(index);
            episode.seconds = w.seconds;
            episode.fell = terminated != 0;
            {
                const std::size_t head = w.reward->headLink();
                episode.endHeadHeight =
                    head < body.links.size() ? rlLinkCenterOfMass3D(profile->links[head], body.links[head]).z : 0.0f;
                episode.fellDown = episode.fell && episode.endHeadHeight < 1.1f;
            }
            const double n = std::max<std::uint32_t>(1, w.decisions);
            episode.meanReward = static_cast<float>(w.rewardSum / n);
            episode.centerOfMassSpeedError = static_cast<float>(w.comErrorSum / n);
            episode.bodySpeed = static_cast<float>(w.speedSum / n);
            episode.requestedSpeed = static_cast<float>(w.requestedSum / n);
            episode.meanHeadDistance = static_cast<float>(w.headSum / n);
            episode.relativeDirection = w.relativeDirection;
            {
                std::lock_guard lock(episodeMutex);
                episodes.push_back(episode);
            }
            // O lider segue (DReCon: so o corpo recomeca); longe da origem, volta
            // para perto dela aquecendo de novo.
            const Vec3 at = w.leader->capsulePosition();
            if (std::hypot(at.x, at.y) > RecenterDistance) restartLeader(w, 2.0f);
            seedBody(w);
        }
        observe(w);
    }
};

TrackingTraining3D::TrackingTraining3D(const TrackingTrainingSettings3D& settings) : m_impl(std::make_unique<Impl>()) {
    Impl& impl = *m_impl;
    impl.settings = settings;
    impl.bundle = AthleteBundle3D::shared(settings.assetsDirectory);
    impl.profile = &impl.bundle->profile();
    const AthleteRuntimeAssets3D& assets = impl.bundle->assets();
    if (assets.defaultClips == nullptr) throw std::runtime_error("rastreador: faltam os clipes do padrao");
    impl.contract = std::make_unique<TrackingContract3D>(*impl.profile);
    const std::uint32_t hardware = std::max(1u, std::thread::hardware_concurrency());
    impl.workers = settings.threads > 0 ? settings.threads : std::max(1u, hardware > 2 ? hardware - 2 : 1u);
    impl.scheduler = std::make_shared<TaskScheduler>(TaskSchedulerSettings { std::max(1u, impl.workers - 1) });
    impl.engine = std::make_unique<PhysicsEngine3D>(impl.scheduler);
    impl.worlds.resize(std::max(1u, settings.environments));
    for (Impl::World& w : impl.worlds) {
        w.scene = impl.engine->createScene(trackingSceneSettings3D(), impl.materials);
        createTrackingGround3D(*w.scene, GroundHalfExtent);
        RagdollSpawnDefinition3D spawn;
        spawn.entityId = 1;
        spawn.active = true;
        spawn.pelvisPosition = { 0.0f, 0.0f, impl.profile->standingRootHeightMeters };
        w.handle = w.scene->createRagdoll(*impl.profile, spawn);
        w.leader = std::make_unique<TrackingLeader3D>(*impl.profile, *assets.defaultClips, impl.runtime);
        w.reward = std::make_unique<TrackingReward3D>(*impl.profile);
        w.observation.assign(TrackingContract3D::ObservationSize, 0.0f);
        w.finalObservation.assign(TrackingContract3D::ObservationSize, 0.0f);
        w.body = w.scene->ragdollStateView(w.handle);
    }
    reset(settings.seed);
}

TrackingTraining3D::~TrackingTraining3D() {
    // (Os mundos antes do motor e do agendador.)
    if (m_impl) m_impl->worlds.clear();
}

std::size_t TrackingTraining3D::environmentCount() const { return m_impl->worlds.size(); }
std::size_t TrackingTraining3D::observationSize() const { return TrackingContract3D::ObservationSize; }
std::size_t TrackingTraining3D::actionSize() const { return TrackingContract3D::ActionSize; }
std::uint32_t TrackingTraining3D::workerThreads() const { return m_impl->workers; }
const std::vector<std::string>& TrackingTraining3D::observationNames() const { return m_impl->contract->observationNames(); }
const std::vector<std::string>& TrackingTraining3D::actionNames() const { return m_impl->contract->actionNames(); }
const RagdollProfile3D& TrackingTraining3D::profile() const { return *m_impl->profile; }

void TrackingTraining3D::reset(std::uint64_t seed) {
    Impl& impl = *m_impl;
    trackingPullWorlds3D(*impl.scheduler, impl.workers, impl.worlds.size(),
        [&impl, seed](std::size_t i) { impl.startWorld(impl.worlds[i], i, seed); });
    impl.episodes.clear();
}

void TrackingTraining3D::observations(std::span<float> out) const {
    const std::size_t size = TrackingContract3D::ObservationSize;
    if (out.size() < m_impl->worlds.size() * size) throw std::invalid_argument("rastreador: observacao pequena");
    for (std::size_t i = 0; i < m_impl->worlds.size(); ++i)
        std::copy(m_impl->worlds[i].observation.begin(), m_impl->worlds[i].observation.end(), out.begin() + i * size);
}

void TrackingTraining3D::finalObservations(std::span<float> out) const {
    const std::size_t size = TrackingContract3D::ObservationSize;
    if (out.size() < m_impl->worlds.size() * size) throw std::invalid_argument("rastreador: observacao pequena");
    for (std::size_t i = 0; i < m_impl->worlds.size(); ++i)
        std::copy(m_impl->worlds[i].finalObservation.begin(), m_impl->worlds[i].finalObservation.end(),
            out.begin() + i * size);
}

void TrackingTraining3D::step(std::span<const float> actions, std::span<float> rewards,
    std::span<std::uint8_t> terminated, std::span<std::uint8_t> truncated) {
    Impl& impl = *m_impl;
    const std::size_t n = impl.worlds.size();
    const std::size_t a = TrackingContract3D::ActionSize;
    if (actions.size() < n * a || rewards.size() < n || terminated.size() < n || truncated.size() < n)
        throw std::invalid_argument("rastreador: buffers pequenos no step");
    trackingPullWorlds3D(*impl.scheduler, impl.workers, n, [&](std::size_t i) {
        impl.decide(impl.worlds[i], i, actions.subspan(i * a, a), rewards[i], terminated[i], truncated[i]);
    });
}

std::vector<TrackingEpisode3D> TrackingTraining3D::takeEpisodes() {
    std::lock_guard lock(m_impl->episodeMutex);
    std::vector<TrackingEpisode3D> out;
    out.swap(m_impl->episodes);
    return out;
}

const TrackingLeaderState3D& TrackingTraining3D::leader(std::size_t world) const {
    return m_impl->worlds.at(world).leader->state();
}

const RagdollState3D& TrackingTraining3D::body(std::size_t world) const { return *m_impl->worlds.at(world).body; }

void TrackingTraining3D::setCommand(std::size_t world, float speed, float relativeDirectionRadians) {
    Impl::World& w = m_impl->worlds.at(world);
    w.speed = std::max(0.0f, speed);
    w.relativeDirection = relativeDirectionRadians;
    w.turnRate = 0.0f;
    m_impl->composeCommand(w);
}

void TrackingTraining3D::push(std::size_t world, std::uint32_t link, Vec3 force, float seconds) {
    Impl::World& w = m_impl->worlds.at(world);
    w.pushLink = link;
    w.pushForce = force;
    w.pushSeconds = std::max(0.0f, seconds);
}

std::string TrackingTraining3D::frameLines(std::size_t world, bool leader) const {
    const Impl& impl = *m_impl;
    const Impl::World& w = impl.worlds.at(world);
    const RagdollProfile3D& profile = *impl.profile;
    const std::vector<PhysicsBodyState3D>& links = leader ? w.leader->state().links : w.body->links;
    std::ostringstream out;
    for (std::size_t i = 0; i < profile.links.size() && i < links.size(); ++i) {
        const RagdollLinkDefinition3D& link = profile.links[i];
        const PhysicsBodyState3D& state = links[i];
        const Vec3 center = state.position + state.orientation.rotate(link.collider.localPosition);
        const Quaternion orientation = (state.orientation * link.collider.localOrientation).normalized();
        out << (link.collider.shape == RagdollColliderShape3D::Box ? "B " : "C ") << link.id << ' ' << center.x << ' '
            << center.y << ' ' << center.z << ' ' << orientation.x << ' ' << orientation.y << ' ' << orientation.z
            << ' ' << orientation.w << '\n';
    }
    if (!leader)
        for (const RagdollContactPoint3D& contact : w.body->contacts)
            out << "P " << contact.position.x << ' ' << contact.position.y << ' ' << contact.position.z << ' '
                << contact.linkIndex << ' ' << contact.normal.x << ' ' << contact.normal.y << ' ' << contact.normal.z
                << ' ' << contact.normalImpulseNewtonSeconds << '\n';
    const Vec3 wanted = w.command.requestedVelocityWorld;
    const float speed = std::hypot(wanted.x, wanted.y);
    if (speed > 1.0e-4f) out << "W " << wanted.x / speed << ' ' << wanted.y / speed << " 0\n";
    const Vec3 com = leader ? w.leader->state().centerOfMassVelocity : rlBodyCenterOfMassVelocity3D(profile, links);
    out << "U " << speed << ' ' << std::hypot(com.x, com.y) << '\n';
    out << "V " << com.x << ' ' << com.y << ' ' << com.z << '\n';
    return out.str();
}

} // namespace MatterEngine
