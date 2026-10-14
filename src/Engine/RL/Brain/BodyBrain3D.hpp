#pragma once

#include "Engine/Physics/PhysicsScene3D.hpp"
#include "Engine/RL/Body/RLBody3D.hpp"
#include "Engine/RL/Contract/BodyCommand3D.hpp"
#include "Engine/RL/Contract/BodyContract3D.hpp"
#include "Engine/RL/Contract/PlayerAttributes3D.hpp"
#include "Engine/RL/Policy/PolicyRuntime3D.hpp"

#include <cstdint>
#include <filesystem>
#include <span>
#include <vector>

namespace MatterEngine {

// Um jogador conduzido pelo cerebro: a memoria do contrato, a ultima entrada,
// a acao e os alvos dos motores.
struct BodyBrainPlayer3D {
    BodyContractState3D contract;
    std::vector<float> observation;
    std::vector<float> action;
    std::vector<RagdollDriveTarget3D> targets;
};

// O cerebro do Corpo no jogo (plano_claude_rl.md secoes 15.1 e 16): o contrato
// v2 e a politica exportada, a mesma conta do treino. O jogo chama, a cada
// decisao, decide (a entrada e a acao media; ou decideMany com os 22 numa
// conta so) e, a cada passo de fisica, targets antes (os alvos da reta) e
// afterStep depois (o contato resumido).
class BodyBrain3D {
public:
    // Lanca se a politica nao for deste contrato (os tamanhos ou os nomes da entrada).
    BodyBrain3D(const RLBody3D& body, const std::filesystem::path& policyPath, BodyContractSettings3D settings = {});

    [[nodiscard]] const BodyContract3D& contract() const { return m_contract; }
    [[nodiscard]] const PolicyRuntime3D& policy() const { return m_policy; }

    // Um jogador novo ou um recomeco: a memoria e a reta partem do corpo de agora.
    void reset(BodyBrainPlayer3D& player, const RagdollState3D& state) const;
    // Depois de cada passo de fisica (bodyWeight em N).
    void afterStep(BodyBrainPlayer3D& player, const RagdollState3D& state, float bodyWeight, float fixedStep) const;
    // A decisao de um jogador.
    void decide(BodyBrainPlayer3D& player, const RagdollState3D& state, const BodyCommand3D& command,
        const PlayerAttributes3D& attributes) const;
    // A decisao de varios jogadores numa conta so (o i-esimo de cada lista).
    void decideMany(std::span<BodyBrainPlayer3D* const> players, std::span<const RagdollState3D* const> states,
        std::span<const BodyCommand3D> commands, std::span<const PlayerAttributes3D> attributes) const;
    // Os alvos do passo de fisica substep (0 a substepsPerDecision-1) da decisao.
    const std::vector<RagdollDriveTarget3D>& targets(BodyBrainPlayer3D& player, std::uint32_t substep,
        const PlayerAttributes3D& attributes) const;

private:
    BodyContract3D m_contract;
    PolicyRuntime3D m_policy;
    mutable std::vector<float> m_observations;
    mutable std::vector<float> m_actions;
};

} // namespace MatterEngine
