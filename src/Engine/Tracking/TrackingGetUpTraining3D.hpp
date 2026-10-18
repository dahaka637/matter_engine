#pragma once

#include "Engine/Physics/PhysicsScene3D.hpp"

#include <cstdint>
#include <memory>
#include <span>
#include <string>
#include <vector>

namespace MatterEngine {

struct TrackingGetUpSettings3D {
    std::string assetsDirectory;
    // O clipe: "frente" (de brucos, stand_up_front_kimodo) ou "costas" (stand_up_back_kimodo).
    std::string clip = "frente";
    std::uint32_t environments = 512;
    std::uint32_t threads = 0;
    std::uint64_t seed = 1;
    // A avaliacao: comeca no inicio do clipe (deitado), sem sorteio.
    bool evaluation = false;
};

// Um episodio de levantar que acabou (no fim do clipe).
struct TrackingGetUpEpisode3D {
    std::uint32_t world = 0;
    float startSeconds = 0.0f;
    float seconds = 0.0f;
    float meanReward = 0.0f;
    // No fim: a altura da cabeca e da pelve (m) e se esta de pe (cabeca acima
    // de 1,4 m e pelve acima de 0,75 m).
    float headHeight = 0.0f;
    float pelvisHeight = 0.0f;
    bool standing = false;
};

// O lote do levantar sem ajuda (DeepMimic; vault
// .../pesquisas/12_especificacao_rastreador.md, secao 8): o ragdoll do jogo a 0%
// imitando o clipe de levantar (RSI: um instante sorteado do clipe; o horizonte
// e o fim do clipe, T_cycle; sem termino por contato - o comeco e deitado).
class TrackingGetUpTraining3D final {
public:
    static constexpr int TicksPerDecision = 4;
    static constexpr float TickSeconds = 1.0f / 120.0f;

    explicit TrackingGetUpTraining3D(const TrackingGetUpSettings3D& settings);
    ~TrackingGetUpTraining3D();
    TrackingGetUpTraining3D(const TrackingGetUpTraining3D&) = delete;
    TrackingGetUpTraining3D& operator=(const TrackingGetUpTraining3D&) = delete;

    [[nodiscard]] std::size_t environmentCount() const;
    [[nodiscard]] std::size_t observationSize() const;
    [[nodiscard]] std::size_t actionSize() const;
    [[nodiscard]] const std::vector<std::string>& observationNames() const;
    [[nodiscard]] const std::vector<std::string>& actionNames() const;
    [[nodiscard]] float clipSeconds() const;

    void reset(std::uint64_t seed);
    void observations(std::span<float> out) const;
    // A acao: os alvos absolutos (radianos). truncated = o fim do clipe.
    void step(std::span<const float> actions, std::span<float> rewards, std::span<std::uint8_t> terminated,
        std::span<std::uint8_t> truncated);
    void finalObservations(std::span<float> out) const;
    [[nodiscard]] std::vector<TrackingGetUpEpisode3D> takeEpisodes();
    // A acao que reproduz a referencia de agora (para conferir: com ela o corpo
    // segue o clipe em malha aberta).
    void referenceActions(std::span<float> out) const;
    [[nodiscard]] std::string frameLines(std::size_t world, bool reference) const;

private:
    struct Impl;
    std::unique_ptr<Impl> m_impl;
};

} // namespace MatterEngine
