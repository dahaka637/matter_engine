#pragma once

#include "Engine/Physics/PhysicsScene3D.hpp"
#include "Engine/Physics/RagdollProfile3D.hpp"

#include <cstdint>
#include <memory>
#include <span>
#include <string>
#include <vector>

namespace MatterEngine {

struct TrackingGetUpAmpSettings3D {
    std::string assetsDirectory;
    std::uint32_t environments = 512;
    std::uint32_t threads = 0;
    std::uint64_t seed = 1;
    // A avaliacao: so as quedas (sem os comecos nos clipes).
    bool evaluation = false;
    // O episodio (s) e, no treino, a fatia que comeca num instante sorteado de um clipe de levantar.
    float episodeSeconds = 6.0f;
    float clipStartFraction = 0.25f;
};

// Um episodio do levantar livre que acabou (no fim do tempo).
struct TrackingGetUpAmpEpisode3D {
    std::uint32_t world = 0;
    // 0: uma queda; 1: um instante de um clipe de levantar.
    std::uint32_t startKind = 0;
    // Como comecou: 0 de brucos, 1 de costas, 2 de lado (ou sentado, ajoelhado).
    std::uint32_t startSide = 0;
    float startHeadHeight = 0.0f;
    float seconds = 0.0f;
    // A primeira vez de pe (s desde o comeco; < 0: nao ficou).
    float firstStandSeconds = -1.0f;
    float meanTaskReward = 0.0f;
    float headHeight = 0.0f;
    float pelvisHeight = 0.0f;
    bool standing = false;
};

// O lote do levantar livre (vault 04_locomocao/plano_compostos_31_10.md, "O levantar livre"): o ragdoll do jogo a 0%
// comecando de quedas sorteadas (uma pose de um clipe, o corpo inteiro num sentido sorteado, solto com os musculos
// fracos ate assentar no chao) e, no treino, tambem de instantes dos clipes de levantar. A acao e a do levantar do
// DeepMimic (o alvo dos 39 eixos); a observacao, so o corpo; a nota daqui e a da tarefa (TrackingGetUpPostureProbe3D),
// e a do estilo vem do discriminador do treinador, com os pares do AMP de cada passo e os dos clipes.
class TrackingGetUpAmpTraining3D final {
public:
    static constexpr int TicksPerDecision = 4;
    static constexpr float TickSeconds = 1.0f / 120.0f;
    // Os clipes do estilo: os de levantar (e os espelhados) tambem dao os comecos; o parado so o estilo.
    static constexpr const char* GetUpClips[4] { "stand_up_front_kimodo", "stand_up_back_kimodo",
        "stand_up_front_kimodo_espelho", "stand_up_back_kimodo_espelho" };
    static constexpr const char* StandingClip = "natural_idle";

    explicit TrackingGetUpAmpTraining3D(const TrackingGetUpAmpSettings3D& settings);
    ~TrackingGetUpAmpTraining3D();
    TrackingGetUpAmpTraining3D(const TrackingGetUpAmpTraining3D&) = delete;
    TrackingGetUpAmpTraining3D& operator=(const TrackingGetUpAmpTraining3D&) = delete;

    [[nodiscard]] std::size_t environmentCount() const;
    [[nodiscard]] std::size_t observationSize() const;
    [[nodiscard]] std::size_t actionSize() const;
    [[nodiscard]] std::size_t ampSize() const;
    [[nodiscard]] const std::vector<std::string>& observationNames() const;
    [[nodiscard]] const std::vector<std::string>& actionNames() const;
    [[nodiscard]] float episodeSeconds() const;

    void reset(std::uint64_t seed);
    void observations(std::span<float> out) const;
    // A observacao do fim dos episodios que acabaram no ultimo passo (para o valor do corte).
    void finalObservations(std::span<float> out) const;
    // rewards: a nota da tarefa.
    void step(std::span<const float> actions, std::span<float> rewards, std::span<std::uint8_t> terminated,
        std::span<std::uint8_t> truncated);
    // O par do AMP do ultimo passo de cada mundo (antes e depois, 2 x ampSize), antes de um recomeco.
    void ampPairs(std::span<float> out) const;
    // Os pares dos clipes do estilo, de 1/120 em 1/120 s (2 x ampSize cada; o passo e o de uma decisao).
    [[nodiscard]] std::vector<float> referencePairs() const;
    [[nodiscard]] std::vector<TrackingGetUpAmpEpisode3D> takeEpisodes();

    [[nodiscard]] const RagdollState3D& body(std::size_t world) const;
    [[nodiscard]] const RagdollProfile3D& profile() const;
    [[nodiscard]] std::string frameLines(std::size_t world) const;

private:
    struct Impl;
    std::unique_ptr<Impl> m_impl;
};

} // namespace MatterEngine
