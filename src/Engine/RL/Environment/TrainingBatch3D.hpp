#pragma once

#include "Engine/Physics/PhysicsScene3D.hpp"
#include "Engine/RL/Body/RLBody3D.hpp"
#include "Engine/RL/Environment/TrainingTask3D.hpp"

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <span>
#include <string>
#include <vector>

namespace MatterEngine {

// Um canal nomeado de um vetor do contrato (observacao, critico, acao).
struct RLChannel3D {
    std::string name;
    std::string unit;
};

struct RLVectorLayout3D {
    std::string id;
    std::uint32_t version = 0;
    std::vector<RLChannel3D> channels;

    [[nodiscard]] std::size_t size() const { return channels.size(); }
};

struct TrainingBatchSettings3D {
    std::filesystem::path taskPath;
    std::uint32_t environmentCount = 256;
    // Bonecos por cena: varios dividem o custo fixo do passo da cena (medido:
    // 4 por cena rende ~45% mais que 1).
    std::uint32_t environmentsPerScene = 4;
    // Threads de fisica; 0 = as do hardware menos duas.
    std::uint32_t workerThreads = 0;
    std::uint64_t seed = 1;
    // Avaliacao: sem variacao do mundo, sem ruido, episodio mais longo.
    bool evaluation = false;
    float evaluationEpisodeSeconds = 60.0f;
    // Iteracoes do solver das juntas; negativo = as do corpo de RL. A
    // avaliacao confere a politica tambem com as 20/20 do jogo.
    int solverPositionIterations = -1;
    int solverVelocityIterations = -1;
};

enum class TrainingEndReason3D : std::uint8_t {
    None = 0,
    FallContact,
    FallHeight,
    Numeric,
    TimeLimit,
    Boundary
};

[[nodiscard]] const char* trainingEndReasonName3D(TrainingEndReason3D reason);

// O resumo de um episodio que acabou no ultimo step().
struct TrainingEpisode3D {
    std::uint32_t environment = 0;
    TrainingEndReason3D reason = TrainingEndReason3D::None;
    // Primeiro link que nao e apoio a tocar o chao (-1 sem contato) e o lado
    // da queda no referencial do corpo (0 frente, 1 tras, 2 esquerda, 3 direita).
    int firstContactLink = -1;
    int fallDirection = -1;
    std::uint32_t steps = 0;
    float seconds = 0.0f;
    float episodeReturn = 0.0f;
    // Medidas que a recompensa nao paga diretamente (a bateria usa estas).
    float comSwayRms = 0.0f;
    float footDriftMax = 0.0f;
    float meanActionRate = 0.0f;
    float meanEffort = 0.0f;
    float meanJointSpeed = 0.0f;
    float meanUpright = 0.0f;
};

// Um mundo de treino visto por fora (o visualizador, a gravacao).
struct TrainingEnvironmentView3D {
    const RagdollState3D* state = nullptr;
    Vec3 origin;
    std::uint32_t steps = 0;
    Vec3 pushForce;
    int pushLink = -1;
};

// Centenas de mundos de treino sem janela, avancados em paralelo.
//
// Cada mundo e um boneco do corpo de RL sobre um chao plano. Varios mundos
// dividem uma cena (longe uns dos outros) e cada cena roda numa thread so,
// com os jobs do Jolt na propria thread. step() recebe as acoes de todos,
// escreve os alvos das juntas (nada mais: nenhuma forca na pelve, guia ou
// transform), avanca actionRepeat passos fixos de 120 Hz e publica
// observacoes, recompensas e fins de episodio em buffers contiguos, sem
// alocar. Um mundo que termina recomeca no mesmo step (o critico do estado
// final fica em terminalCriticObservations).
class TrainingBatch3D final {
public:
    explicit TrainingBatch3D(const TrainingBatchSettings3D& settings);
    ~TrainingBatch3D();
    TrainingBatch3D(const TrainingBatch3D&) = delete;
    TrainingBatch3D& operator=(const TrainingBatch3D&) = delete;

    [[nodiscard]] std::size_t environmentCount() const;
    [[nodiscard]] std::size_t observationSize() const;
    [[nodiscard]] std::size_t criticObservationSize() const;
    [[nodiscard]] std::size_t actionSize() const;
    [[nodiscard]] std::size_t rewardComponentCount() const;
    [[nodiscard]] std::uint32_t workerThreads() const;
    [[nodiscard]] float policyStepSeconds() const;
    [[nodiscard]] const RLVectorLayout3D& observationLayout() const;
    [[nodiscard]] const RLVectorLayout3D& criticObservationLayout() const;
    [[nodiscard]] const RLVectorLayout3D& actionLayout() const;
    [[nodiscard]] const RLBody3D& body() const;
    [[nodiscard]] const TrainingTask3D& task() const;
    // Muda o que pode mudar com o treino andando: recompensa, termino,
    // inicio, variacao e empurroes (vale do proximo passo/episodio em
    // diante). Corpo e actionRepeat nao mudam.
    void updateTask(const TrainingTask3D& task);

    // Todos os mundos recomecam (sementes derivadas de seed).
    void reset(std::uint64_t seed);
    // actions: environmentCount x actionSize, linha por mundo.
    void step(std::span<const float> actions);

    [[nodiscard]] std::span<const float> observations() const;
    [[nodiscard]] std::span<const float> criticObservations() const;
    [[nodiscard]] std::span<const float> rewards() const;
    [[nodiscard]] std::span<const float> rewardComponents() const;
    [[nodiscard]] std::span<const std::uint8_t> terminated() const;
    [[nodiscard]] std::span<const std::uint8_t> truncated() const;
    [[nodiscard]] std::span<const float> terminalCriticObservations() const;
    [[nodiscard]] std::span<const TrainingEpisode3D> finishedEpisodes() const;
    [[nodiscard]] TrainingEnvironmentView3D view(std::size_t environment) const;
    // Um empurrao de fora (a tela: testar a politica), no proximo step: a
    // forca no link durante os segundos dados. Registrado como os do treino.
    void push(std::size_t environment, std::uint32_t link, Vec3 forceNewtons, float seconds);
    // Radianos por unidade de acao de cada canal (o ruido inicial da politica).
    [[nodiscard]] std::vector<float> actionRadiansPerUnit() const;
    // Milissegundos do ultimo step() (fisica + observacao) e o total de passos
    // fisicos feitos desde a criacao.
    [[nodiscard]] float lastStepMilliseconds() const;
    [[nodiscard]] std::uint64_t physicsStepCount() const;

private:
    struct Impl;
    std::unique_ptr<Impl> m_impl;
};

} // namespace MatterEngine
