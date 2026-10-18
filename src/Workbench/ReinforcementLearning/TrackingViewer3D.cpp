#include "Workbench/ReinforcementLearning/TrackingViewer3D.hpp"

#include "Engine/RL/Body/RLBody3D.hpp"
#include "Engine/RL/Policy/PolicyRuntime3D.hpp"
#include "Engine/Tracking/TrackingTraining3D.hpp"

#include <nlohmann/json.hpp>

#include <cmath>
#include <fstream>
#include <vector>

namespace MatterEngine {
namespace {

// A velocidade de quando a tela nao pede outra (a do prototipo, G1).
constexpr float DefaultSpeed = 3.0f;

} // namespace

struct TrackingViewer3D::Impl {
    std::unique_ptr<TrackingTraining3D> batch;
    std::unique_ptr<PolicyRuntime3D> policy;
    std::vector<float> observation;
    std::vector<float> action;
    std::vector<float> rewards { 0.0f };
    std::vector<std::uint8_t> terminated { 0 }, truncated { 0 };
    float speed = DefaultSpeed;
    std::uint32_t steps = 0;
    std::uint32_t chest = 0;
    float pushSeconds = 0.0f;
    Vec3 pushForce;
};

TrackingViewer3D::TrackingViewer3D(const std::filesystem::path& assetsDirectory, std::uint64_t seed)
    : m_impl(std::make_unique<Impl>()) {
    Impl& impl = *m_impl;
    TrackingTrainingSettings3D settings;
    settings.assetsDirectory = assetsDirectory.string();
    settings.environments = 1;
    settings.threads = 1;
    settings.seed = seed;
    settings.evaluation = true;
    settings.evaluationSpeed = DefaultSpeed;
    // O visualizador nao recomeca pelo tempo (so pela queda ou o afastamento).
    settings.episodeSeconds = 1.0e6f;
    impl.batch = std::make_unique<TrackingTraining3D>(settings);
    impl.observation.assign(impl.batch->observationSize(), 0.0f);
    impl.action.assign(impl.batch->actionSize(), 0.0f);
    const RagdollProfile3D& p = impl.batch->profile();
    for (std::size_t i = 0; i < p.links.size(); ++i)
        if (p.links[i].id == "UpperChest") impl.chest = static_cast<std::uint32_t>(i);
}

TrackingViewer3D::~TrackingViewer3D() = default;

bool TrackingViewer3D::isTrackingTask(const std::filesystem::path& taskPath) {
    std::ifstream file(taskPath);
    if (!file) return false;
    const nlohmann::json task = nlohmann::json::parse(file, nullptr, false);
    return !task.is_discarded() && task.is_object() && task.value("schema", std::string()) == "matter-tracking-1";
}

void TrackingViewer3D::setPolicy(const std::filesystem::path& file, std::string& label) {
    Impl& impl = *m_impl;
    try {
        auto policy = std::make_unique<PolicyRuntime3D>(file);
        if (policy->observationSize() != impl.batch->observationSize() || policy->actionSize() != impl.batch->actionSize()
            || policy->info().observationLayout != "rastreador-v1") {
            impl.policy.reset();
            label = "politica de outro contrato";
            return;
        }
        impl.policy = std::move(policy);
        label = "rodada " + std::to_string(impl.policy->info().iteration);
    } catch (const std::exception& error) {
        impl.policy.reset();
        label = error.what();
    }
}

void TrackingViewer3D::clearPolicy() { m_impl->policy.reset(); }
bool TrackingViewer3D::hasPolicy() const { return m_impl->policy != nullptr; }

void TrackingViewer3D::step() {
    Impl& impl = *m_impl;
    impl.batch->observations(impl.observation);
    // Sem rede, o lider puro (a correcao zero): o corpo cai em ~1 s.
    if (impl.policy) impl.policy->evaluate(impl.observation, impl.action);
    else std::fill(impl.action.begin(), impl.action.end(), 0.0f);
    impl.batch->step(impl.action, impl.rewards, impl.terminated, impl.truncated);
    (void)impl.batch->takeEpisodes();
    impl.pushSeconds = std::max(0.0f, impl.pushSeconds - policyStepSeconds());
    ++impl.steps;
}

void TrackingViewer3D::reset(std::uint64_t seed) {
    Impl& impl = *m_impl;
    impl.batch->reset(seed);
    impl.batch->setCommand(0, impl.speed, 0.0f);
    impl.steps = 0;
}

void TrackingViewer3D::setManualSpeed(std::optional<float> metersPerSecond) {
    Impl& impl = *m_impl;
    impl.speed = metersPerSecond.value_or(DefaultSpeed);
    impl.batch->setCommand(0, impl.speed, 0.0f);
}

void TrackingViewer3D::push(Vec3 force, float seconds) {
    Impl& impl = *m_impl;
    impl.batch->push(0, impl.chest, force, seconds);
    impl.pushForce = force;
    impl.pushSeconds = seconds;
}

BodyWorldView3D TrackingViewer3D::view() const {
    const Impl& impl = *m_impl;
    const RagdollState3D& body = impl.batch->body(0);
    const TrackingLeaderState3D& leader = impl.batch->leader(0);
    const RagdollProfile3D& profile = impl.batch->profile();
    BodyWorldView3D view;
    view.state = &body;
    view.steps = impl.steps;
    view.command.moveVelocity = { leader.requestedVelocityWorld.x, leader.requestedVelocityWorld.y, 0.0f };
    view.command.faceHeading = leader.headingYawRadians;
    const Vec3 velocity = rlBodyCenterOfMassVelocity3D(profile, body.links);
    view.velocity = { velocity.x, velocity.y, 0.0f };
    if (impl.chest < body.links.size()) {
        const Quaternion r =
            (body.links[impl.chest].orientation * profile.links[impl.chest].modelOrientation.conjugate()).normalized();
        const Vec3 forward = r.rotate({ 1.0f, 0.0f, 0.0f });
        view.chestHeading = std::atan2(forward.y, forward.x);
    }
    if (impl.pushSeconds > 0.0f) {
        view.pushLink = static_cast<int>(impl.chest);
        view.pushForce = impl.pushForce;
    }
    return view;
}

const RagdollProfile3D& TrackingViewer3D::profile() const { return m_impl->batch->profile(); }

} // namespace MatterEngine
