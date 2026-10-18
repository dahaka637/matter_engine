#include "Engine/Tracking/TrackingGetUpTraining3D.hpp"

#include "Engine/Athlete/AthleteAssets3D.hpp"
#include "Engine/Materials/MaterialLibrary.hpp"
#include "Engine/Physics/PhysicsEngine3D.hpp"
#include "Engine/RL/Body/RLBody3D.hpp"
#include "Engine/Tracking/TrackingClipReference3D.hpp"
#include "Engine/Tracking/TrackingGetUp3D.hpp"
#include "Engine/Tracking/TrackingScene3D.hpp"

#include <algorithm>
#include <cmath>
#include <mutex>
#include <random>
#include <sstream>
#include <stdexcept>
#include <thread>

namespace MatterEngine {
namespace {

constexpr float Pi = 3.14159265358979f;

} // namespace

struct TrackingGetUpTraining3D::Impl {
    TrackingGetUpSettings3D settings;
    std::shared_ptr<const AthleteBundle3D> bundle;
    const RagdollProfile3D* profile = nullptr;
    AnimationClip3D clip;
    std::unique_ptr<TrackingClipReference3D> reference;
    std::unique_ptr<TrackingGetUpContract3D> contract;
    std::uint32_t workers = 1;
    std::shared_ptr<TaskScheduler> scheduler;
    std::unique_ptr<PhysicsEngine3D> engine;
    MaterialLibrary materials;

    struct World {
        std::unique_ptr<PhysicsScene3D> scene;
        RagdollHandle3D handle;
        std::mt19937 rng;
        float time = 0.0f;
        float start = 0.0f;
        float yaw = 0.0f;
        TrackingClipFrame3D frame;
        std::vector<RagdollDriveTarget3D> drives;
        const RagdollState3D* body = nullptr;
        std::vector<float> observation;
        std::vector<float> finalObservation;
        double rewardSum = 0.0;
        std::uint32_t decisions = 0;
    };
    std::vector<World> worlds;
    std::vector<TrackingGetUpEpisode3D> episodes;
    std::mutex episodeMutex;

    // O comeco (RSI do DeepMimic): um instante sorteado do clipe (na avaliacao, o
    // inicio), o rumo sorteado, o corpo na pose e na velocidade da referencia.
    void begin(World& w) const {
        const float last = std::max(0.0f, clip.durationSeconds - 2.0f / 30.0f);
        w.start = settings.evaluation ? 0.0f : std::uniform_real_distribution<float>(0.0f, last)(w.rng);
        w.time = w.start;
        w.yaw = std::uniform_real_distribution<float>(-Pi, Pi)(w.rng);
        reference->frame(w.time, {}, w.yaw, w.frame);
        w.scene->resetRagdollState(w.handle, w.frame.links);
        w.body = w.scene->ragdollStateView(w.handle);
        w.rewardSum = 0.0;
        w.decisions = 0;
    }

    void observe(World& w) const {
        contract->observe(w.body->links, std::clamp(w.time / clip.durationSeconds, 0.0f, 1.0f), w.observation);
    }

    void decide(World& w, std::size_t index, std::span<const float> action, float& reward, std::uint8_t& terminated,
        std::uint8_t& truncated) {
        contract->driveTargets(action, w.drives);
        for (int k = 0; k < TicksPerDecision; ++k) {
            w.scene->setRagdollActiveDriveTargets(w.handle, w.drives, false);
            w.scene->simulate(TickSeconds);
            w.body = w.scene->ragdollStateView(w.handle);
        }
        w.time += TicksPerDecision * TickSeconds;
        reference->frame(w.time, {}, w.yaw, w.frame);
        const TrackingGetUpContract3D::Terms terms = contract->reward(w.body->links, w.frame);
        reward = terms.total;
        w.rewardSum += terms.total;
        ++w.decisions;
        terminated = 0;
        truncated = w.time >= clip.durationSeconds - 1e-4f ? 1 : 0;
        if (truncated) {
            observe(w);
            w.finalObservation = w.observation;
            TrackingGetUpEpisode3D episode;
            episode.world = static_cast<std::uint32_t>(index);
            episode.startSeconds = w.start;
            episode.seconds = w.time - w.start;
            episode.meanReward = static_cast<float>(w.rewardSum / std::max<std::uint32_t>(1, w.decisions));
            const auto& links = w.body->links;
            episode.headHeight = rlLinkCenterOfMass3D(profile->links[contract->headLink()], links[contract->headLink()]).z;
            episode.pelvisHeight = links[contract->rootLink()].position.z;
            episode.standing = episode.headHeight > 1.4f && episode.pelvisHeight > 0.75f;
            {
                std::lock_guard lock(episodeMutex);
                episodes.push_back(episode);
            }
            begin(w);
        }
        observe(w);
    }
};

TrackingGetUpTraining3D::TrackingGetUpTraining3D(const TrackingGetUpSettings3D& settings)
    : m_impl(std::make_unique<Impl>()) {
    Impl& impl = *m_impl;
    impl.settings = settings;
    impl.bundle = AthleteBundle3D::shared(settings.assetsDirectory);
    impl.profile = &impl.bundle->profile();
    const std::string name = settings.clip == "frente" ? "stand_up_front_kimodo"
        : settings.clip == "costas" ? "stand_up_back_kimodo" : "";
    if (name.empty()) throw std::invalid_argument("levantar: o clipe e 'frente' ou 'costas'");
    impl.clip = loadAnimationClip3D(settings.assetsDirectory + "/animations/clips/" + name + ".matteranim.json");
    impl.reference = std::make_unique<TrackingClipReference3D>(*impl.profile, impl.clip);
    impl.contract = std::make_unique<TrackingGetUpContract3D>(*impl.profile);
    const std::uint32_t hardware = std::max(1u, std::thread::hardware_concurrency());
    impl.workers = settings.threads > 0 ? settings.threads : std::max(1u, hardware > 2 ? hardware - 2 : 1u);
    impl.scheduler = std::make_shared<TaskScheduler>(TaskSchedulerSettings { std::max(1u, impl.workers - 1) });
    impl.engine = std::make_unique<PhysicsEngine3D>(impl.scheduler);
    impl.worlds.resize(std::max(1u, settings.environments));
    for (Impl::World& w : impl.worlds) {
        w.scene = impl.engine->createScene(trackingSceneSettings3D(), impl.materials);
        createTrackingGround3D(*w.scene, 20.0f);
        RagdollSpawnDefinition3D spawn;
        spawn.entityId = 1;
        spawn.active = true;
        spawn.pelvisPosition = { 0.0f, 0.0f, impl.profile->standingRootHeightMeters };
        w.handle = w.scene->createRagdoll(*impl.profile, spawn);
        w.observation.assign(impl.contract->observationSize(), 0.0f);
        w.finalObservation.assign(impl.contract->observationSize(), 0.0f);
    }
    reset(settings.seed);
}

TrackingGetUpTraining3D::~TrackingGetUpTraining3D() {
    if (m_impl) m_impl->worlds.clear();
}

std::size_t TrackingGetUpTraining3D::environmentCount() const { return m_impl->worlds.size(); }
std::size_t TrackingGetUpTraining3D::observationSize() const { return m_impl->contract->observationSize(); }
std::size_t TrackingGetUpTraining3D::actionSize() const { return m_impl->contract->actionSize(); }
const std::vector<std::string>& TrackingGetUpTraining3D::observationNames() const {
    return m_impl->contract->observationNames();
}
const std::vector<std::string>& TrackingGetUpTraining3D::actionNames() const { return m_impl->contract->actionNames(); }
float TrackingGetUpTraining3D::clipSeconds() const { return m_impl->clip.durationSeconds; }

void TrackingGetUpTraining3D::reset(std::uint64_t seed) {
    Impl& impl = *m_impl;
    trackingPullWorlds3D(*impl.scheduler, impl.workers, impl.worlds.size(), [&impl, seed](std::size_t i) {
        Impl::World& w = impl.worlds[i];
        w.rng.seed(static_cast<std::uint32_t>(seed * 2654435761ull + i * 40503ull + 777ull));
        impl.begin(w);
        impl.observe(w);
    });
    impl.episodes.clear();
}

void TrackingGetUpTraining3D::observations(std::span<float> out) const {
    const std::size_t size = observationSize();
    for (std::size_t i = 0; i < m_impl->worlds.size(); ++i)
        std::copy(m_impl->worlds[i].observation.begin(), m_impl->worlds[i].observation.end(), out.begin() + i * size);
}

void TrackingGetUpTraining3D::finalObservations(std::span<float> out) const {
    const std::size_t size = observationSize();
    for (std::size_t i = 0; i < m_impl->worlds.size(); ++i)
        std::copy(m_impl->worlds[i].finalObservation.begin(), m_impl->worlds[i].finalObservation.end(),
            out.begin() + i * size);
}

void TrackingGetUpTraining3D::step(std::span<const float> actions, std::span<float> rewards,
    std::span<std::uint8_t> terminated, std::span<std::uint8_t> truncated) {
    Impl& impl = *m_impl;
    const std::size_t n = impl.worlds.size();
    const std::size_t a = actionSize();
    if (actions.size() < n * a || rewards.size() < n || terminated.size() < n || truncated.size() < n)
        throw std::invalid_argument("levantar: buffers pequenos no step");
    trackingPullWorlds3D(*impl.scheduler, impl.workers, n, [&](std::size_t i) {
        impl.decide(impl.worlds[i], i, actions.subspan(i * a, a), rewards[i], terminated[i], truncated[i]);
    });
}

std::vector<TrackingGetUpEpisode3D> TrackingGetUpTraining3D::takeEpisodes() {
    std::lock_guard lock(m_impl->episodeMutex);
    std::vector<TrackingGetUpEpisode3D> out;
    out.swap(m_impl->episodes);
    return out;
}

void TrackingGetUpTraining3D::referenceActions(std::span<float> out) const {
    const std::size_t a = actionSize();
    for (std::size_t i = 0; i < m_impl->worlds.size(); ++i) {
        // A pose da referencia do proximo instante (onde o corpo deve chegar).
        const Impl::World& w = m_impl->worlds[i];
        TrackingClipFrame3D next;
        m_impl->reference->frame(w.time + TicksPerDecision * TickSeconds, {}, w.yaw, next);
        m_impl->contract->actionOf(next.coordinates, out.subspan(i * a, a));
    }
}

std::string TrackingGetUpTraining3D::frameLines(std::size_t world, bool reference) const {
    const Impl& impl = *m_impl;
    const Impl::World& w = impl.worlds.at(world);
    const RagdollProfile3D& profile = *impl.profile;
    const std::vector<PhysicsBodyState3D>& links = reference ? w.frame.links : w.body->links;
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
    if (!reference)
        for (const RagdollContactPoint3D& contact : w.body->contacts)
            out << "P " << contact.position.x << ' ' << contact.position.y << ' ' << contact.position.z << ' '
                << contact.linkIndex << ' ' << contact.normal.x << ' ' << contact.normal.y << ' ' << contact.normal.z
                << ' ' << contact.normalImpulseNewtonSeconds << '\n';
    return out.str();
}

} // namespace MatterEngine
