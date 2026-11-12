#include "Engine/Tracking/TrackingGetUpPolicy3D.hpp"

#include "Engine/RL/Policy/PolicyRuntime3D.hpp"
#include "Engine/Tracking/TrackingGetUp3D.hpp"

#include <stdexcept>

namespace MatterEngine {
namespace {

// As entradas do comando depois das do corpo (TrackingGetUpAmpTraining3D::CommandSize); sem pedido de deslocamento.
constexpr float NoCommand[6] { 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f };
// Com o pedido, as do levantar em movimento do treino (TrackingGetUpAmpTraining3D::observe): o alvo no meio da faixa
// dele (4-8 m, em 10 m) e a velocidade na faixa dele (2,5-5 m/s, em 5 m/s), de frente para o rumo.
constexpr float CommandTargetDistance = 0.6f;
constexpr float CommandMinimumSpeed = 2.5f;
constexpr float CommandMaximumSpeed = 5.0f;

} // namespace

TrackingGetUpPolicy3D::TrackingGetUpPolicy3D(const std::filesystem::path& file) : m_policy(std::make_unique<PolicyRuntime3D>(file)) {
    const std::string& layout = m_policy->info().observationLayout;
    if (layout != "levantar-v2" && layout != "levantar-v3" && layout != "levantar-v4" && layout != "levantar-v5" && layout != "levantar-v6"
        && layout != "levantar-v7")
        throw std::runtime_error("rede do levantar: layout " + layout + " (esperado levantar-v2 a v7)");
    const bool v7 = layout == "levantar-v7";
    m_ground = layout != "levantar-v2";
    // shortcut: no jogo a fase do clipe vai em zero (sem o relogio do passo); quando soltar a imitacao, gerar a fase.
    m_phase = layout == "levantar-v4" || layout == "levantar-v5" || layout == "levantar-v6" || v7 ? 2 : 0;
    m_environment = layout == "levantar-v5" || layout == "levantar-v6" || v7 ? TrackingGetUpContract3D::EnvironmentObservationSize : 0;
    // (04/11 15h, o v7: o equilibrio do contrato e o estado do gerador de passos do treino - no jogo a recuperacao age sem o
    // gerador, que e so do idle da locomocao: o estado vai em zero, como no treino fora dele.)
    m_balance = v7 ? TrackingGetUpContract3D::BalanceObservationSize : 0;
    m_stepState = v7 ? StepStateSize : 0;
    m_skillMode = layout == "levantar-v6" || v7 ? 1 : 0;
}

TrackingGetUpPolicy3D::~TrackingGetUpPolicy3D() = default;

std::uint64_t TrackingGetUpPolicy3D::iteration() const { return m_policy->info().iteration; }

const TrackingGetUpContract3D& TrackingGetUpPolicy3D::contractOf(const RagdollProfile3D& profile) const {
    std::lock_guard lock(m_mutex);
    for (const auto& [known, contract] : m_contracts)
        if (known == &profile) return *contract;
    auto contract = std::make_unique<TrackingGetUpContract3D>(profile);
    const std::size_t ground = m_ground ? TrackingGetUpContract3D::GroundObservationSize : 0;
    if (contract->bodyObservationSize() + std::size(NoCommand) + ground + m_phase + m_environment + m_balance + m_stepState
                + m_skillMode
            != m_policy->observationSize()
        || (contract->actionSize() != m_policy->actionSize() && contract->actionSize() + StepChannels != m_policy->actionSize()))
        throw std::runtime_error("rede do levantar: o corpo nao bate com a rede");
    m_contracts.emplace_back(&profile, std::move(contract));
    return *m_contracts.back().second;
}

void TrackingGetUpPolicy3D::decide(const RagdollProfile3D& profile, const RagdollState3D& state,
    std::vector<RagdollDriveTarget3D>& drives, Vec3 velocity, float groundHeight, const PhysicsScene3D* scene,
    RagdollHandle3D self) const {
    const TrackingGetUpContract3D& contract = contractOf(profile);
    const std::span<const PhysicsBodyState3D> links(state.links);
    // (Por thread: os mundos do lote decidem em paralelo; a rede ja tem o rascunho dela por thread.)
    thread_local std::vector<float> observation, action;
    observation.assign(m_policy->observationSize(), 0.0f);
    action.assign(m_policy->actionSize(), 0.0f);
    const std::size_t body = contract.bodyObservationSize();
    contract.observeBody(links, std::span<float>(observation).first(body));
    if (m_ground)
        contract.observeGround(links, state.interactions, groundHeight,
            std::span<float>(observation).subspan(body + std::size(NoCommand), TrackingGetUpContract3D::GroundObservationSize));
    if (m_environment > 0)
        contract.observeEnvironment(scene, self, links, groundHeight,
            std::span<float>(observation).subspan(body + std::size(NoCommand) + TrackingGetUpContract3D::GroundObservationSize
                + m_phase, m_environment));
    if (m_balance > 0)
        contract.observeBalance(links, state.interactions, groundHeight,
            std::span<float>(observation).subspan(body + std::size(NoCommand) + TrackingGetUpContract3D::GroundObservationSize
                + m_phase + m_environment, m_balance));
    float* command = observation.data() + body;
    const float speed = std::hypot(velocity.x, velocity.y);
    if (speed > GetUpPolicy3D::MovingRequestSpeed && !links.empty()) {
        const float move = std::atan2(velocity.y, velocity.x) - trackingBodyYaw3D(profile.links.front(), links.front());
        command[0] = command[4] = std::cos(move);
        command[1] = command[5] = std::sin(move);
        command[2] = CommandTargetDistance;
        command[3] = std::clamp(speed, CommandMinimumSpeed, CommandMaximumSpeed) / CommandMaximumSpeed;
    } else {
        std::copy(std::begin(NoCommand), std::end(NoCommand), command);
    }
    m_policy->evaluate(observation, action);
    // (Os canais do passo, se houver, ficam de fora: so os eixos.)
    contract.driveTargets(std::span<const float>(action).first(contract.actionSize()), drives);
}

} // namespace MatterEngine
