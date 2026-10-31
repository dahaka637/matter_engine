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
    // A fatia que comeca de pe, num instante do clipe parado (31/10: ele nao se mantinha de pe depois de levantar).
    float standingStartFraction = 0.15f;
    // O levantar v2 (31/10 17h, ele): a fatia que comeca caida e levanta ja indo para um alvo (em arrancada, virado
    // para ele) e a do evitar a queda (de pe, um empurrao que o derrubaria: evitar o chao e se recompor rapido).
    float movingFraction = 0.0f;
    float avoidFraction = 0.0f;
    // A fatia que comeca na arrancada agachada (o clipe crouched_to_sprint, virado para um alvo, com o pedido de ir a
    // ele; 31/10, ele: "nao e apenas um levantar e entao ir, mas sim um levantar preparado ja indo").
    float sprintStartFraction = 0.0f;
    // O alvo do levantar em movimento: a distancia (m) e a velocidade pedida (m/s).
    float targetMinimumMeters = 4.0f;
    float targetMaximumMeters = 8.0f;
    float moveMinimumSpeed = 2.5f;
    float moveMaximumSpeed = 5.0f;
    // O empurrao do evitar a queda: o curto (N, 0,1-0,2 s) e, numa fatia, o continuo (N, s).
    float avoidMinimumNewtons = 1200.0f;
    float avoidMaximumNewtons = 1800.0f;
    float avoidHoldFraction = 0.3f;
    float avoidHoldMinimumNewtons = 350.0f;
    float avoidHoldMaximumNewtons = 550.0f;
    float avoidHoldMinimumSeconds = 0.5f;
    float avoidHoldMaximumSeconds = 1.2f;
    // O empurrao de pe do jogo de pes (N, 0,1-0,2 s).
    float pushMinimumNewtons = 400.0f;
    float pushMaximumNewtons = 900.0f;
    // Na avaliacao, o que se mede: 0 as quedas (o levantar), 1 o levantar em movimento, 2 o evitar a queda.
    std::uint32_t evaluationKind = 0;
};

// Um episodio do levantar livre que acabou (no fim do tempo).
struct TrackingGetUpAmpEpisode3D {
    std::uint32_t world = 0;
    // 0: uma queda; 1: um instante de um clipe de levantar; 2: de pe (o parado); 3: uma queda com alvo (o levantar em
    // movimento); 4: de pe com o empurrao forte (o evitar a queda); 5: a arrancada agachada com alvo.
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
    // O levantar v2: a velocidade pedida no comeco (0: ficar), quando chegou ao alvo (s; < 0: nao chegou), a
    // velocidade media rumo ao alvo enquanto de pe com o pedido (m/s), se caiu depois de ja ter ficado de pe e, no
    // evitar a queda, quando se recompos (s desde o fim do empurrao; < 0: nao se recompos).
    float commandSpeed = 0.0f;
    float reachedSeconds = -1.0f;
    float meanTowardSpeed = 0.0f;
    bool fellAfterStanding = false;
    float recomposedSeconds = -1.0f;
};

// O que o visualizador do laboratorio desenha de um mundo: como comecou, o alvo (o levantar em movimento, enquanto o
// pedido dura) e o empurrao de agora.
struct TrackingGetUpAmpWorldView3D {
    std::uint32_t startKind = 0;
    bool hasTarget = false;
    Vec3 target;
    float commandSpeed = 0.0f;
    bool pushing = false;
    Vec3 pushForce;
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
    // Os clipes do estilo do levantar em movimento (a arrancada e a corrida; no lugar, com a velocidade nominal somada
    // aos pares do discriminador): so o estilo e os comecos das quedas.
    static constexpr const char* MovingClips[5] { "idle_to_sprint", "crouched_to_sprint", "mixamo_running",
        "run_forward_medium", "sprint_forward" };
    // As entradas do comando, depois das do corpo: o rumo do alvo em relacao ao corpo (cos, sen), a distancia
    // (/10 m, ate 1) e a velocidade pedida (/5 m/s; 0: ficar onde esta).
    static constexpr std::size_t CommandSize = 4;

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
    // O minimo e o maximo (rad) de cada eixo da acao (2 por eixo).
    [[nodiscard]] std::vector<float> actionRanges() const;

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
    [[nodiscard]] TrackingGetUpAmpWorldView3D worldView(std::size_t world) const;

private:
    struct Impl;
    std::unique_ptr<Impl> m_impl;
};

} // namespace MatterEngine
