#include "Engine/Tracking/TrackingGetUpPolicy3D.hpp"

#include "Engine/RL/Policy/PolicyRuntime3D.hpp"
#include "Engine/Tracking/TrackingGetUp3D.hpp"

#include <stdexcept>

namespace MatterEngine {
namespace {

// As entradas do comando depois das do corpo (TrackingGetUpAmpTraining3D::CommandSize); sem pedido de deslocamento.
constexpr float NoCommand[6] { 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f };

} // namespace

TrackingGetUpPolicy3D::TrackingGetUpPolicy3D(const std::filesystem::path& file) : m_policy(std::make_unique<PolicyRuntime3D>(file)) {
    if (m_policy->info().observationLayout != "levantar-v2")
        throw std::runtime_error("rede do levantar: layout " + m_policy->info().observationLayout + " (esperado levantar-v2)");
}

TrackingGetUpPolicy3D::~TrackingGetUpPolicy3D() = default;

std::uint64_t TrackingGetUpPolicy3D::iteration() const { return m_policy->info().iteration; }

void TrackingGetUpPolicy3D::decide(const RagdollProfile3D& profile, std::span<const PhysicsBodyState3D> links,
    std::vector<RagdollDriveTarget3D>& drives) const {
    if (m_profile != &profile || !m_contract) {
        m_contract = std::make_unique<TrackingGetUpContract3D>(profile);
        m_profile = &profile;
        const std::size_t body = m_contract->bodyObservationSize();
        if (body + std::size(NoCommand) != m_policy->observationSize()
            || m_contract->actionSize() != m_policy->actionSize())
            throw std::runtime_error("rede do levantar: o corpo nao bate com a rede");
        m_observation.assign(m_policy->observationSize(), 0.0f);
        m_action.assign(m_policy->actionSize(), 0.0f);
    }
    const std::size_t body = m_contract->bodyObservationSize();
    m_contract->observeBody(links, std::span<float>(m_observation).first(body));
    std::copy(std::begin(NoCommand), std::end(NoCommand), m_observation.begin() + static_cast<std::ptrdiff_t>(body));
    m_policy->evaluate(m_observation, m_action);
    m_contract->driveTargets(m_action, drives);
}

} // namespace MatterEngine
