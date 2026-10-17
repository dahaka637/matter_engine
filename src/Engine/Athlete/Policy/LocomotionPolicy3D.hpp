#pragma once

#include "Engine/Athlete/AthleteRuntime3D.hpp"
#include "Engine/Athlete/Contract/LocomotionContract3D.hpp"
#include "Engine/Physics/RagdollProfile3D.hpp"

#include <cstdint>
#include <filesystem>
#include <memory>
#include <string>

namespace MatterEngine {

// A rede da locomocao no jogo (o RL dentro dos modulos, contrato v2; vault
// 04_locomocao/plano_fases_1_2.md): a politica que o treinador exporta
// (tools/rl/matter_rl/locomotion_trainer.py, exports/policy_latest.mrlpolicy)
// decidindo como no treino - a cada TicksPerDecision ticks a entrada do
// atleta e a acao media da rede; a cada tick a rampa, o teto, a autoridade e o
// limite de taxa do contrato, nas correcoes do atleta. A autoridade e os tetos
// sao os com que ela treinou (do arquivo).
class LocomotionPolicy3D final {
public:
    // Lanca std::runtime_error se o arquivo nao serve (outro contrato, outro
    // corpo, as entradas diferentes).
    LocomotionPolicy3D(const RagdollProfile3D& profile, const std::filesystem::path& file);
    ~LocomotionPolicy3D();
    LocomotionPolicy3D(const LocomotionPolicy3D&) = delete;
    LocomotionPolicy3D& operator=(const LocomotionPolicy3D&) = delete;

    // O treino e a rodada de onde ela veio (para mostrar).
    [[nodiscard]] const std::string& run() const;
    [[nodiscard]] std::uint64_t iteration() const;
    [[nodiscard]] float authority() const;
    [[nodiscard]] const std::filesystem::path& file() const;

    // Antes do passo do atleta (o corpo publicado no ultimo passo e o comando
    // deste tick): a decisao quando cabe e o tick da rampa.
    void before(AthleteState3D& athlete, const RagdollState3D& body, const AthleteCommand3D& command, float deltaTime);

private:
    struct Impl;
    std::unique_ptr<Impl> m_impl;
};

} // namespace MatterEngine
