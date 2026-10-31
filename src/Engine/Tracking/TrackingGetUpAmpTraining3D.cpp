#include "Engine/Tracking/TrackingGetUpAmpTraining3D.hpp"

#include "Engine/Animation/AnimationClip3D.hpp"
#include "Engine/Athlete/AthleteAssets3D.hpp"
#include "Engine/Materials/MaterialLibrary.hpp"
#include "Engine/Physics/PhysicsEngine3D.hpp"
#include "Engine/RL/Body/RLBody3D.hpp"
#include "Engine/Tracking/TrackingClipReference3D.hpp"
#include "Engine/Tracking/TrackingGetUp3D.hpp"
#include "Engine/Tracking/TrackingGetUpAmp3D.hpp"
#include "Engine/Tracking/TrackingScene3D.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <mutex>
#include <random>
#include <sstream>
#include <stdexcept>
#include <thread>

namespace MatterEngine {
namespace {

constexpr float Pi = 3.14159265358979f;
// A queda: os musculos a esta fracao da rigidez do rastreador, segurando a pose sorteada com este ruido (rad), de
// 0,9 a 1,5 s; a pose comeca com o ponto mais baixo de 0,35 a 0,85 m do chao.
constexpr float FallStiffness = 0.15f;
constexpr float FallPoseNoise = 0.35f;

// Uma rotacao uniforme (Shoemake, "Uniform random rotations", 1992).
Quaternion uniformRotation(std::mt19937& rng) {
    std::uniform_real_distribution<float> u(0.0f, 1.0f);
    const float u1 = u(rng), u2 = 2.0f * Pi * u(rng), u3 = 2.0f * Pi * u(rng);
    const float a = std::sqrt(1.0f - u1), b = std::sqrt(u1);
    return Quaternion { a * std::sin(u2), a * std::cos(u2), b * std::sin(u3), b * std::cos(u3) }.normalized();
}

} // namespace

struct TrackingGetUpAmpTraining3D::Impl {
    TrackingGetUpAmpSettings3D settings;
    std::shared_ptr<const AthleteBundle3D> bundle;
    const RagdollProfile3D* profile = nullptr;
    // Os clipes do estilo (os de levantar primeiro, depois o parado) e as referencias deles.
    std::vector<std::unique_ptr<AnimationClip3D>> clips;
    std::vector<std::unique_ptr<TrackingClipReference3D>> references;
    std::unique_ptr<TrackingGetUpContract3D> contract;
    std::unique_ptr<TrackingAmpFeatures3D> features;
    std::unique_ptr<TrackingGetUpPostureProbe3D> posture;
    std::vector<std::string> observationNames;
    std::uint32_t workers = 1;
    std::shared_ptr<TaskScheduler> scheduler;
    std::unique_ptr<PhysicsEngine3D> engine;
    MaterialLibrary materials;

    struct World {
        std::unique_ptr<PhysicsScene3D> scene;
        RagdollHandle3D handle;
        std::mt19937 rng;
        float time = 0.0f;
        TrackingClipFrame3D frame;
        std::vector<float> pose;
        std::vector<RagdollDriveTarget3D> drives;
        const RagdollState3D* body = nullptr;
        std::vector<float> observation;
        std::vector<float> finalObservation;
        std::vector<float> ampPair;
        std::uint32_t startKind = 0;
        std::uint32_t startSide = 0;
        float startHead = 0.0f;
        float stoodAt = -1.0f;
        double rewardSum = 0.0;
        std::uint32_t decisions = 0;
        // O empurrao de pe (o jogo de pes de autoequilibrio): a forca, os passos de fisica que faltam e o proximo.
        Vec3 pushForce;
        int pushTicks = 0;
        float nextPushAt = -1.0f;
    };
    std::vector<World> worlds;
    std::vector<TrackingGetUpAmpEpisode3D> episodes;
    std::mutex episodeMutex;

    void settle(World& w, int ticks) {
        for (int k = 0; k < ticks; ++k) {
            w.scene->setRagdollActiveDriveTargets(w.handle, w.drives, false);
            if (w.pushTicks > 0) {
                // (Repartido como o do B19: pelve, peito e peito alto.)
                w.scene->applyRagdollLinkForce(w.handle, 0, w.pushForce * 0.20f, {});
                w.scene->applyRagdollLinkForce(w.handle, 2, w.pushForce * 0.32f, {});
                w.scene->applyRagdollLinkForce(w.handle, 3, w.pushForce * 0.48f, {});
                --w.pushTicks;
            }
            w.scene->simulate(TickSeconds);
        }
        w.body = w.scene->ragdollStateView(w.handle);
    }

    // A queda: uma pose de um clipe (qualquer um do estilo) num instante sorteado, o corpo inteiro girado num sentido
    // sorteado em torno do centro de massa, erguido, com giro e deriva sorteados, e solto com os musculos fracos
    // segurando a pose com ruido ate assentar. Se nao caiu (a cabeca acima de 1 m), outra.
    void beginFall(World& w) {
        std::uniform_real_distribution<float> u(0.0f, 1.0f);
        std::normal_distribution<float> gauss(0.0f, 1.0f);
        for (int attempt = 0; attempt < 4; ++attempt) {
            const auto& reference = *references[std::uniform_int_distribution<std::size_t>(0, references.size() - 1)(w.rng)];
            reference.frame(u(w.rng) * reference.durationSeconds(), {}, 0.0f, w.frame);
            const Quaternion turn = uniformRotation(w.rng);
            const Vec3 center = w.frame.centerOfMass;
            float lowest = std::numeric_limits<float>::max();
            for (std::size_t i = 0; i < w.frame.links.size(); ++i) {
                PhysicsBodyState3D& s = w.frame.links[i];
                s.position = center + turn.rotate(s.position - center);
                s.orientation = (turn * s.orientation).normalized();
                lowest = std::min(lowest, rlLinkCenterOfMass3D(profile->links[i], s).z);
            }
            const Vec3 lifted = center + Vec3 { 0.0f, 0.0f, 0.35f + 0.5f * u(w.rng) - lowest };
            const Vec3 spin = Vec3 { gauss(w.rng), gauss(w.rng), gauss(w.rng) } * 1.5f;
            const Vec3 drift { 2.0f * u(w.rng) - 1.0f, 2.0f * u(w.rng) - 1.0f, 0.0f };
            for (PhysicsBodyState3D& s : w.frame.links) {
                s.position = s.position + (lifted - center);
                s.linearVelocity = drift + cross(spin, s.position - lifted);
                s.angularVelocity = spin;
            }
            w.scene->resetRagdollState(w.handle, w.frame.links);
            contract->actionOf(w.frame.coordinates, w.pose);
            for (float& value : w.pose) value += FallPoseNoise * (2.0f * u(w.rng) - 1.0f);
            contract->driveTargets(w.pose, w.drives);
            for (RagdollDriveTarget3D& drive : w.drives) drive.stiffnessScale *= FallStiffness;
            settle(w, static_cast<int>((0.9f + 0.6f * u(w.rng)) / TickSeconds));
            if (posture->measure(w.body->links).headHeight < 1.0f) return;
        }
    }

    // Um instante sorteado de um clipe de levantar (o RSI do DeepMimic) ou, de pe, do parado, num rumo sorteado.
    void beginFromClip(World& w, bool standing) {
        std::uniform_real_distribution<float> u(0.0f, 1.0f);
        const auto& reference = standing ? *references.back()
            : *references[std::uniform_int_distribution<std::size_t>(0, std::size(GetUpClips) - 1)(w.rng)];
        const float last = std::max(0.0f, reference.durationSeconds() - 2.0f / 30.0f);
        reference.frame(u(w.rng) * last, {}, (2.0f * u(w.rng) - 1.0f) * Pi, w.frame);
        w.scene->resetRagdollState(w.handle, w.frame.links);
        w.body = w.scene->ragdollStateView(w.handle);
    }

    void begin(World& w) {
        const float draw = std::uniform_real_distribution<float>(0.0f, 1.0f)(w.rng);
        const bool standing = !settings.evaluation && draw < settings.standingStartFraction;
        const bool fall = settings.evaluation || draw >= settings.standingStartFraction + settings.clipStartFraction;
        if (fall) beginFall(w);
        else beginFromClip(w, standing);
        const TrackingGetUpPosture3D start = posture->measure(w.body->links);
        w.startKind = fall ? 0u : standing ? 2u : 1u;
        w.startSide = start.chestFacingUp < -0.5f ? 0u : start.chestFacingUp > 0.5f ? 1u : 2u;
        w.startHead = start.headHeight;
        w.time = 0.0f;
        w.stoodAt = -1.0f;
        w.rewardSum = 0.0;
        w.decisions = 0;
        w.pushTicks = 0;
        w.nextPushAt = -1.0f;
    }

    // O empurrao de pe (31/10 09h, ele: depois de levantar falta o "jogo de pes de auto-equilibrio"): so no treino,
    // depois de ja ter ficado de pe e enquanto esta de pe, um empurrao curto a cada 1,5-3 s (o primeiro 1-2,5 s depois
    // de ficar de pe), 250-650 N por 0,1-0,2 s (ate ~130 N.s; 150-450 ate 31/10 12h35, quando ele ja ficava de pe em 98-99% com eles), num
    // rumo sorteado. Se cair, levanta de novo.
    void maybePush(World& w, const TrackingGetUpPosture3D& posture) {
        if (settings.evaluation || w.stoodAt < 0.0f || !posture.standing || w.pushTicks > 0) return;
        std::uniform_real_distribution<float> u(0.0f, 1.0f);
        if (w.nextPushAt < 0.0f) {
            w.nextPushAt = w.time + 1.0f + 1.5f * u(w.rng);
            return;
        }
        if (w.time < w.nextPushAt) return;
        const float angle = 2.0f * Pi * u(w.rng);
        const float newtons = 250.0f + 400.0f * u(w.rng);
        w.pushForce = { std::cos(angle) * newtons, std::sin(angle) * newtons, 0.0f };
        w.pushTicks = std::max(1, static_cast<int>(std::lround((0.1f + 0.1f * u(w.rng)) / TickSeconds)));
        w.nextPushAt = w.time + 1.5f + 1.5f * u(w.rng);
    }

    void observe(World& w) const { contract->observeBody(w.body->links, w.observation); }

    void decide(World& w, std::size_t index, std::span<const float> action, float& reward, std::uint8_t& terminated,
        std::uint8_t& truncated) {
        const std::size_t a = features->size();
        features->compute(w.body->links, std::span<float>(w.ampPair).first(a));
        const TrackingGetUpPosture3D was = posture->measure(w.body->links);
        const float before = posture->risePotential(was) + posture->turnPotential(was);
        maybePush(w, was);
        contract->driveTargets(action, w.drives);
        settle(w, TicksPerDecision);
        w.time += TicksPerDecision * TickSeconds;
        features->compute(w.body->links, std::span<float>(w.ampPair).subspan(a, a));
        const TrackingGetUpPosture3D now = posture->measure(w.body->links);
        // (O gamma do treinador, 0,99, no potencial.)
        reward = posture->standingReward(now, w.body->links)
            + 0.99f * (posture->risePotential(now) + posture->turnPotential(now)) - before;
        w.rewardSum += reward;
        ++w.decisions;
        if (w.stoodAt < 0.0f && now.standing) w.stoodAt = w.time;
        terminated = 0;
        truncated = w.time >= settings.episodeSeconds - 1e-4f ? 1 : 0;
        if (truncated) {
            observe(w);
            w.finalObservation = w.observation;
            TrackingGetUpAmpEpisode3D episode;
            episode.world = static_cast<std::uint32_t>(index);
            episode.startKind = w.startKind;
            episode.startSide = w.startSide;
            episode.startHeadHeight = w.startHead;
            episode.seconds = w.time;
            episode.firstStandSeconds = w.stoodAt;
            episode.meanTaskReward = static_cast<float>(w.rewardSum / std::max<std::uint32_t>(1, w.decisions));
            episode.headHeight = now.headHeight;
            episode.pelvisHeight = now.pelvisHeight;
            episode.standing = now.standing;
            {
                std::lock_guard lock(episodeMutex);
                episodes.push_back(episode);
            }
            begin(w);
        }
        observe(w);
    }
};

TrackingGetUpAmpTraining3D::TrackingGetUpAmpTraining3D(const TrackingGetUpAmpSettings3D& settings)
    : m_impl(std::make_unique<Impl>()) {
    Impl& impl = *m_impl;
    impl.settings = settings;
    impl.bundle = AthleteBundle3D::shared(settings.assetsDirectory);
    impl.profile = &impl.bundle->profile();
    std::vector<std::string> names(std::begin(GetUpClips), std::end(GetUpClips));
    names.emplace_back(StandingClip);
    for (const std::string& name : names) {
        impl.clips.push_back(std::make_unique<AnimationClip3D>(
            loadAnimationClip3D(settings.assetsDirectory + "/animations/clips/" + name + ".matteranim.json")));
        impl.references.push_back(std::make_unique<TrackingClipReference3D>(*impl.profile, *impl.clips.back()));
    }
    impl.contract = std::make_unique<TrackingGetUpContract3D>(*impl.profile);
    impl.features = std::make_unique<TrackingAmpFeatures3D>(*impl.profile);
    impl.posture = std::make_unique<TrackingGetUpPostureProbe3D>(*impl.profile);
    {
        TrackingClipFrame3D idle;
        impl.references.back()->frame(0.0f, {}, 0.0f, idle);
        impl.posture->setIdlePose(idle.links);
    }
    // (Os nomes do corpo: os do levantar do DeepMimic sem a fase, a ultima.)
    impl.observationNames.assign(impl.contract->observationNames().begin(), impl.contract->observationNames().end() - 1);
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
        w.observation.assign(impl.contract->bodyObservationSize(), 0.0f);
        w.finalObservation.assign(impl.contract->bodyObservationSize(), 0.0f);
        w.ampPair.assign(2 * impl.features->size(), 0.0f);
        w.pose.assign(impl.contract->actionSize(), 0.0f);
    }
    reset(settings.seed);
}

TrackingGetUpAmpTraining3D::~TrackingGetUpAmpTraining3D() {
    if (m_impl) m_impl->worlds.clear();
}

std::size_t TrackingGetUpAmpTraining3D::environmentCount() const { return m_impl->worlds.size(); }
std::size_t TrackingGetUpAmpTraining3D::observationSize() const { return m_impl->contract->bodyObservationSize(); }
std::size_t TrackingGetUpAmpTraining3D::actionSize() const { return m_impl->contract->actionSize(); }
std::size_t TrackingGetUpAmpTraining3D::ampSize() const { return m_impl->features->size(); }
const std::vector<std::string>& TrackingGetUpAmpTraining3D::observationNames() const {
    return m_impl->observationNames;
}
const std::vector<std::string>& TrackingGetUpAmpTraining3D::actionNames() const {
    return m_impl->contract->actionNames();
}
float TrackingGetUpAmpTraining3D::episodeSeconds() const { return m_impl->settings.episodeSeconds; }
std::vector<float> TrackingGetUpAmpTraining3D::actionRanges() const { return m_impl->contract->actionRanges(); }

void TrackingGetUpAmpTraining3D::reset(std::uint64_t seed) {
    Impl& impl = *m_impl;
    trackingPullWorlds3D(*impl.scheduler, impl.workers, impl.worlds.size(), [&impl, seed](std::size_t i) {
        Impl::World& w = impl.worlds[i];
        w.rng.seed(static_cast<std::uint32_t>(seed * 2654435761ull + i * 40503ull + 911ull));
        impl.begin(w);
        impl.observe(w);
    });
    impl.episodes.clear();
}

void TrackingGetUpAmpTraining3D::observations(std::span<float> out) const {
    const std::size_t size = observationSize();
    for (std::size_t i = 0; i < m_impl->worlds.size(); ++i)
        std::copy(m_impl->worlds[i].observation.begin(), m_impl->worlds[i].observation.end(), out.begin() + i * size);
}

void TrackingGetUpAmpTraining3D::finalObservations(std::span<float> out) const {
    const std::size_t size = observationSize();
    for (std::size_t i = 0; i < m_impl->worlds.size(); ++i)
        std::copy(m_impl->worlds[i].finalObservation.begin(), m_impl->worlds[i].finalObservation.end(),
            out.begin() + i * size);
}

void TrackingGetUpAmpTraining3D::step(std::span<const float> actions, std::span<float> rewards,
    std::span<std::uint8_t> terminated, std::span<std::uint8_t> truncated) {
    Impl& impl = *m_impl;
    const std::size_t n = impl.worlds.size();
    const std::size_t a = actionSize();
    if (actions.size() < n * a || rewards.size() < n || terminated.size() < n || truncated.size() < n)
        throw std::invalid_argument("levantar livre: buffers pequenos no step");
    trackingPullWorlds3D(*impl.scheduler, impl.workers, n, [&](std::size_t i) {
        impl.decide(impl.worlds[i], i, actions.subspan(i * a, a), rewards[i], terminated[i], truncated[i]);
    });
}

void TrackingGetUpAmpTraining3D::ampPairs(std::span<float> out) const {
    const std::size_t size = 2 * ampSize();
    for (std::size_t i = 0; i < m_impl->worlds.size(); ++i)
        std::copy(m_impl->worlds[i].ampPair.begin(), m_impl->worlds[i].ampPair.end(), out.begin() + i * size);
}

std::vector<float> TrackingGetUpAmpTraining3D::referencePairs() const {
    const Impl& impl = *m_impl;
    const std::size_t a = ampSize();
    const float step = TicksPerDecision * TickSeconds;
    std::vector<float> out;
    std::vector<float> now(a), next(a);
    TrackingClipFrame3D first, second;
    for (const auto& reference : impl.references)
        for (float t = 0.0f; t + step <= reference->durationSeconds() + 1e-5f; t += 1.0f / 120.0f) {
            reference->frame(t, {}, 0.0f, first);
            reference->frame(t + step, {}, 0.0f, second);
            impl.features->compute(first.links, now);
            impl.features->compute(second.links, next);
            out.insert(out.end(), now.begin(), now.end());
            out.insert(out.end(), next.begin(), next.end());
        }
    return out;
}

std::vector<TrackingGetUpAmpEpisode3D> TrackingGetUpAmpTraining3D::takeEpisodes() {
    std::lock_guard lock(m_impl->episodeMutex);
    std::vector<TrackingGetUpAmpEpisode3D> out;
    out.swap(m_impl->episodes);
    return out;
}

const RagdollState3D& TrackingGetUpAmpTraining3D::body(std::size_t world) const {
    return *m_impl->worlds.at(world).body;
}

const RagdollProfile3D& TrackingGetUpAmpTraining3D::profile() const { return *m_impl->profile; }

std::string TrackingGetUpAmpTraining3D::frameLines(std::size_t world) const {
    const Impl& impl = *m_impl;
    const RagdollState3D& body = *impl.worlds.at(world).body;
    std::ostringstream out;
    for (std::size_t i = 0; i < impl.profile->links.size() && i < body.links.size(); ++i) {
        const RagdollLinkDefinition3D& link = impl.profile->links[i];
        const PhysicsBodyState3D& state = body.links[i];
        const Vec3 center = state.position + state.orientation.rotate(link.collider.localPosition);
        const Quaternion orientation = (state.orientation * link.collider.localOrientation).normalized();
        out << (link.collider.shape == RagdollColliderShape3D::Box ? "B " : "C ") << link.id << ' ' << center.x << ' '
            << center.y << ' ' << center.z << ' ' << orientation.x << ' ' << orientation.y << ' ' << orientation.z
            << ' ' << orientation.w << '\n';
    }
    for (const RagdollContactPoint3D& contact : body.contacts)
        out << "P " << contact.position.x << ' ' << contact.position.y << ' ' << contact.position.z << ' '
            << contact.linkIndex << ' ' << contact.normal.x << ' ' << contact.normal.y << ' ' << contact.normal.z << ' '
            << contact.normalImpulseNewtonSeconds << '\n';
    return out.str();
}

} // namespace MatterEngine
