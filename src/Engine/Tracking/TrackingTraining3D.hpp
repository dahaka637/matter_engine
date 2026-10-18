#pragma once

#include "Engine/Physics/PhysicsScene3D.hpp"
#include "Engine/Tracking/TrackingLeader3D.hpp"

#include <cstdint>
#include <memory>
#include <span>
#include <string>
#include <vector>

namespace MatterEngine {

// O que o jogador virtual pede (DReCon 6.7): o prototipo do G1 (so de frente,
// de 0 ao teto do prototipo) ou o jogo inteiro (rumo, olhar, giro, velocidade
// e estilo pela tabela do padrao).
enum class TrackingScenario3D : std::uint8_t { Forward = 0, Full = 1 };

struct TrackingTrainingSettings3D {
    std::string assetsDirectory;
    std::uint32_t environments = 512;
    // As threads (0: as do hardware menos duas); quem chama step() trabalha tambem.
    std::uint32_t threads = 0;
    std::uint64_t seed = 1;
    TrackingScenario3D scenario = TrackingScenario3D::Forward;
    // O teto da velocidade no prototipo (m/s).
    float forwardMaximumSpeed = 3.0f;
    // A avaliacao (os portoes): de frente na velocidade dada, sem trocas; o
    // episodio acaba na queda ou no tempo.
    bool evaluation = false;
    float evaluationSpeed = 3.0f;
    // DReCon: 20 s por episodio, o filtro y = beta a + (1 - beta) y.
    float episodeSeconds = 20.0f;
    float actionFilterBeta = 0.2f;
};

// Um episodio que acabou.
struct TrackingEpisode3D {
    std::uint32_t world = 0;
    float seconds = 0.0f;
    // Acabou pela cabeca a mais de 1 m da do lider (a queda do DReCon).
    bool fell = false;
    // Desses, quais foram queda de verdade (a cabeca abaixo de 1,1 m no fim) e
    // quais o corpo de pe que ficou para tras (ou a frente) do lider.
    bool fellDown = false;
    float endHeadHeight = 0.0f;
    float meanReward = 0.0f;
    // A media por decisao do erro da velocidade do centro de massa no plano
    // (corpo contra lider, m/s), da velocidade do corpo e da pedida.
    float centerOfMassSpeedError = 0.0f;
    float bodySpeed = 0.0f;
    float requestedSpeed = 0.0f;
    float meanHeadDistance = 0.0f;
};

// O lote de treino do rastreador (vault
// 04_locomocao/referencia/pesquisas/12_especificacao_rastreador.md): muitos
// mundos em paralelo, cada um com a sua cena, o ragdoll do jogo a 0% de ajuda e
// o lider (o modo padrao do jogo). Uma decisao sao 4 ticks de 1/120 s (30 Hz).
class TrackingTraining3D final {
public:
    static constexpr int TicksPerDecision = 4;
    static constexpr float TickSeconds = 1.0f / 120.0f;

    explicit TrackingTraining3D(const TrackingTrainingSettings3D& settings);
    ~TrackingTraining3D();
    TrackingTraining3D(const TrackingTraining3D&) = delete;
    TrackingTraining3D& operator=(const TrackingTraining3D&) = delete;

    [[nodiscard]] std::size_t environmentCount() const;
    [[nodiscard]] std::size_t observationSize() const;
    [[nodiscard]] std::size_t actionSize() const;
    [[nodiscard]] std::uint32_t workerThreads() const;
    [[nodiscard]] const std::vector<std::string>& observationNames() const;
    [[nodiscard]] const std::vector<std::string>& actionNames() const;
    [[nodiscard]] const RagdollProfile3D& profile() const;

    // Recomeca todos os mundos (lider aquecido, corpo sobre ele).
    void reset(std::uint64_t seed);
    // A observacao de cada mundo (environments x observationSize).
    void observations(std::span<float> out) const;
    // Uma decisao: a acao de cada mundo (environments x actionSize, radianos);
    // a nota, se acabou pela queda (terminated) ou pelo tempo (truncated). Quem
    // acabou ja recomecou: a observacao nova e a do comeco seguinte, e a do fim
    // fica em finalObservations (para o valor de quem acabou pelo tempo).
    void step(std::span<const float> actions, std::span<float> rewards, std::span<std::uint8_t> terminated,
        std::span<std::uint8_t> truncated);
    void finalObservations(std::span<float> out) const;
    // Os episodios que acabaram desde a ultima chamada.
    [[nodiscard]] std::vector<TrackingEpisode3D> takeEpisodes();

    // Para as previas: o lider e o corpo de um mundo.
    [[nodiscard]] const TrackingLeaderState3D& leader(std::size_t world) const;
    [[nodiscard]] const RagdollState3D& body(std::size_t world) const;
    // As linhas de um quadro para as folhas (MatterModelInspector quadros: os
    // colisores no mundo, os contatos, o pedido e a velocidade), do corpo ou do
    // lider.
    [[nodiscard]] std::string frameLines(std::size_t world, bool leader) const;

private:
    struct Impl;
    std::unique_ptr<Impl> m_impl;
};

} // namespace MatterEngine
