#include "Engine/RL/Brain/BodyBrain3D.hpp"

#include <algorithm>
#include <stdexcept>
#include <string>

namespace MatterEngine {

BodyBrain3D::BodyBrain3D(const RLBody3D& body, const std::filesystem::path& policyPath, BodyContractSettings3D settings)
    : m_contract(body, settings), m_policy(policyPath) {
    if (m_policy.observationSize() != m_contract.observationSize() || m_policy.actionSize() != m_contract.actionSize())
        throw std::runtime_error("cerebro do Corpo: a politica tem outros tamanhos (" + std::to_string(m_policy.observationSize())
            + " entradas, " + std::to_string(m_policy.actionSize()) + " eixos)");
    const std::vector<std::string>& names = m_policy.info().observationNames;
    if (!names.empty()) {
        const auto& channels = m_contract.observationLayout().channels;
        for (std::size_t c = 0; c < channels.size(); ++c)
            if (names[c] != channels[c].name)
                throw std::runtime_error("cerebro do Corpo: a politica e de outro contrato (entrada " + names[c] + ")");
    }
}

void BodyBrain3D::reset(BodyBrainPlayer3D& player, const RagdollState3D& state) const {
    m_contract.reset(player.contract, state);
    player.observation.assign(m_contract.observationSize(), 0.0f);
    player.action.assign(m_contract.actionSize(), 0.0f);
}

void BodyBrain3D::afterStep(BodyBrainPlayer3D& player, const RagdollState3D& state, float bodyWeight,
    float fixedStep) const {
    m_contract.accumulateContacts(player.contract, state, bodyWeight, fixedStep);
}

void BodyBrain3D::decide(BodyBrainPlayer3D& player, const RagdollState3D& state, const BodyCommand3D& command,
    const PlayerAttributes3D& attributes) const {
    m_contract.writeObservation(state, command, attributes, player.contract, player.observation.data());
    m_policy.evaluate(player.observation, player.action);
    // No jogo a acao e a media: a reta da media e a da acao.
    m_contract.beginDecision(player.contract, player.action, player.action);
}

void BodyBrain3D::decideMany(std::span<BodyBrainPlayer3D* const> players, std::span<const RagdollState3D* const> states,
    std::span<const BodyCommand3D> commands, std::span<const PlayerAttributes3D> attributes) const {
    const std::size_t count = players.size();
    if (states.size() != count || commands.size() != count || attributes.size() != count)
        throw std::invalid_argument("cerebro do Corpo: listas de tamanhos diferentes");
    const std::size_t O = m_contract.observationSize(), A = m_contract.actionSize();
    m_observations.resize(count * O);
    m_actions.resize(count * A);
    for (std::size_t i = 0; i < count; ++i) {
        BodyBrainPlayer3D& player = *players[i];
        m_contract.writeObservation(*states[i], commands[i], attributes[i], player.contract, player.observation.data());
        std::copy(player.observation.begin(), player.observation.end(), m_observations.begin() + static_cast<std::ptrdiff_t>(i * O));
    }
    m_policy.evaluateBatch(m_observations, m_actions, count);
    for (std::size_t i = 0; i < count; ++i) {
        BodyBrainPlayer3D& player = *players[i];
        std::copy_n(m_actions.begin() + static_cast<std::ptrdiff_t>(i * A), A, player.action.begin());
        m_contract.beginDecision(player.contract, player.action, player.action);
    }
}

const std::vector<RagdollDriveTarget3D>& BodyBrain3D::targets(BodyBrainPlayer3D& player, std::uint32_t substep,
    const PlayerAttributes3D& attributes) const {
    m_contract.writeTargets(player.contract, substep, attributes, player.targets);
    return player.targets;
}

} // namespace MatterEngine
