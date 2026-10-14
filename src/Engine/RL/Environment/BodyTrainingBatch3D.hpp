#pragma once

#include "Engine/Physics/PhysicsScene3D.hpp"
#include "Engine/RL/Body/RLBody3D.hpp"
#include "Engine/RL/Contract/BodyContract3D.hpp"
#include "Engine/RL/Contract/VectorLayout3D.hpp"
#include "Engine/RL/Environment/BodyTask3D.hpp"
#include "Engine/RL/Rewards/BodyReward3D.hpp"

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <span>
#include <string>
#include <vector>

namespace MatterEngine {

struct BodyBatchSettings3D {
    std::filesystem::path taskPath;
    std::uint32_t environmentCount = 512;
    // Bonecos por cena (dividem o custo fixo do passo da cena).
    std::uint32_t environmentsPerScene = 4;
    // Threads de fisica; 0 = as do hardware menos duas.
    std::uint32_t workerThreads = 0;
    std::uint64_t seed = 1;
    // Avaliacao: sem a variacao escondida do mundo, sem ruido de observacao e
    // com a duracao fixa.
    bool evaluation = false;
    float evaluationEpisodeSeconds = 20.0f;
    // Iteracoes do solver; negativo = as do corpo de RL (o jogo usa 20/20).
    int solverPositionIterations = -1;
    int solverVelocityIterations = -1;
    // Todos os mundos num exercicio (o id; a bateria, a previa). Vazio = as
    // cotas da tarefa: cada mundo fica no seu exercicio pelo peso.
    std::string drill;
};

enum class BodyEndReason3D : std::uint8_t { None = 0, FallContact, FallHeight, Numeric, TimeLimit, Boundary };

[[nodiscard]] const char* bodyEndReasonName3D(BodyEndReason3D reason);

// O resumo de um episodio que acabou no ultimo step().
struct BodyEpisode3D {
    std::uint32_t environment = 0;
    std::uint32_t drill = 0;
    BodyEndReason3D reason = BodyEndReason3D::None;
    // O primeiro link que nao e apoio a tocar o chao (-1 sem).
    int firstContactLink = -1;
    std::uint32_t steps = 0;
    float seconds = 0.0f;
    float episodeReturn = 0.0f;
    // A tarefa: as notas medias, o erro do vetor de velocidade (m/s) e o do
    // olhar no peito (rad), em RMS.
    float velocityScore = 0.0f;
    float facingScore = 0.0f;
    float velocityErrorRms = 0.0f;
    float facingErrorRms = 0.0f;
    // A reacao: o tempo medio de uma troca do pedido de velocidade (mais de
    // 0,5 m/s) ate o erro entrar na tolerancia (s; -1 sem troca resolvida) e a
    // fatia das trocas resolvidas (-1 sem troca).
    float reactionSeconds = -1.0f;
    float reactionResolved = -1.0f;
    std::uint32_t commandChanges = 0;
    // Os custos medios por decisao (antes dos pesos) e o tempo no teto.
    BodyCosts3D costs;
    // A atuacao: o tamanho medio da acao, a fatia das acoes presas em
    // +-RLActionLimit, a dos alvos presos no curso da junta e a reversao media
    // da acao de uma decisao para a outra (o indice de tremor).
    float meanAbsAction = 0.0f;
    float actionClipped = 0.0f;
    float targetClamped = 0.0f;
    float meanReversal = 0.0f;
    // As pisadas por segundo (os dois pes) e a distancia do comeco (m).
    float cadence = 0.0f;
    float distance = 0.0f;
};

// Um mundo visto por fora (a previa, as folhas de quadros).
struct BodyWorldView3D {
    const RagdollState3D* state = nullptr;
    Vec3 origin;
    std::uint32_t drill = 0;
    std::uint32_t steps = 0;
    // O comando de agora (o que entrou na ultima observacao) e a camera.
    BodyCommand3D command;
    PlayerAttributes3D player;
    // O ponto do piloto (a estrela) e se e de parar.
    bool hasTarget = false;
    Vec3 target;
    bool targetStop = false;
    // A velocidade do centro de massa e o rumo do peito (rad).
    Vec3 velocity;
    float chestHeading = 0.0f;
};

// Os mundos de treino do Corpo v2: centenas de bonecos sem janela, avancados
// em paralelo, so orquestrando as pecas (o contrato, o piloto pelo adaptador
// do controle e a nota). Cada cena roda numa thread so. step() recebe a acao
// executada e a media da politica de cada mundo (a reta da media vira a
// velocidade pedida aos motores), avanca os passos de fisica da decisao e
// publica entrada, critico, nota e fins em buffers contiguos. Um mundo que
// termina recomeca no mesmo step (o critico do estado final fica em
// terminalCriticObservations).
class BodyTrainingBatch3D final {
public:
    explicit BodyTrainingBatch3D(const BodyBatchSettings3D& settings);
    ~BodyTrainingBatch3D();
    BodyTrainingBatch3D(const BodyTrainingBatch3D&) = delete;
    BodyTrainingBatch3D& operator=(const BodyTrainingBatch3D&) = delete;

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
    [[nodiscard]] const BodyTask3D& task() const;
    [[nodiscard]] const BodyContract3D& contract() const;
    // O exercicio de cada mundo (o indice em task().drills).
    [[nodiscard]] std::span<const std::uint32_t> worldDrills() const;
    // O estilo: o que o juiz ve de um instante (StyleFeatures3D) e, de cada
    // mundo, o par da decisao que acabou no ultimo step() (o instante de antes
    // e o de agora, antes de um recomeco): environmentCount x (2 x styleSize).
    [[nodiscard]] std::size_t styleSize() const;
    [[nodiscard]] const RLVectorLayout3D& styleLayout() const;
    [[nodiscard]] std::span<const float> stylePairs() const;

    // Muda o que pode mudar com o treino andando (vale do proximo passo ou
    // episodio em diante): a nota, o fim, a variacao do mundo, o controle e,
    // em cada exercicio, o roteiro, os pesos da tarefa e a duracao. O corpo, o
    // contrato, o comeco e a lista dos exercicios (e as cotas) nao mudam.
    void updateTask(const BodyTask3D& task);
    // Todos os mundos recomecam (sementes derivadas de seed).
    void reset(std::uint64_t seed);
    // actions e means: environmentCount x actionSize, linha por mundo.
    void step(std::span<const float> actions, std::span<const float> means);

    [[nodiscard]] std::span<const float> observations() const;
    [[nodiscard]] std::span<const float> criticObservations() const;
    [[nodiscard]] std::span<const float> rewards() const;
    [[nodiscard]] std::span<const float> rewardComponents() const;
    [[nodiscard]] std::span<const std::uint8_t> terminated() const;
    [[nodiscard]] std::span<const std::uint8_t> truncated() const;
    [[nodiscard]] std::span<const float> terminalCriticObservations() const;
    [[nodiscard]] std::span<const BodyEpisode3D> finishedEpisodes() const;
    [[nodiscard]] BodyWorldView3D view(std::size_t environment) const;
    // Radianos por unidade de acao de cada canal (o ruido inicial da politica).
    [[nodiscard]] std::vector<float> actionRadiansPerUnit() const;
    [[nodiscard]] float lastStepMilliseconds() const;
    [[nodiscard]] std::uint64_t physicsStepCount() const;

private:
    struct Impl;
    std::unique_ptr<Impl> m_impl;
};

} // namespace MatterEngine
