#pragma once

#include <array>

#include "Engine/Physics/PhysicsScene3D.hpp"
#include "Engine/RL/Body/RLBody3D.hpp"
#include "Engine/RL/Commands/ControlAdapter3D.hpp"
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

// De onde um episodio comecou: de pe na postura, num quadro de clipe (a muleta
// do acervo) ou num estado guardado da propria experiencia.
enum class BodyStart3D : std::uint8_t { Standing = 0, Clip, Saved };

[[nodiscard]] const char* bodyEndReasonName3D(BodyEndReason3D reason);

// As fases de um passo do lote (a medida da vazao): preparar a decisao, os
// alvos dos motores, a fisica, a leitura com contatos e custos de cada passo de
// fisica, o fim da decisao (nota e fim), o par do juiz, a observacao e os
// reinicios.
// As tres ultimas dividem a fisica (a medida da cena: antes do Jolt, o Update
// do Jolt e depois) e nao entram no total.
enum class BodyStepPhase3D : std::size_t {
    Prepare = 0, Targets, Simulate, Substep, Finish, Style, Observe, Reset, JoltBefore, JoltUpdate, JoltAfter, Count
};
inline constexpr std::size_t BodyStepPhaseCount3D = static_cast<std::size_t>(BodyStepPhase3D::Count);
inline constexpr std::array<const char*, BodyStepPhaseCount3D> BodyStepPhaseNames3D {
    "preparar", "alvos", "fisica", "leitura", "fim", "estilo", "observacao", "reinicio",
    "fisica_antes", "fisica_jolt", "fisica_depois"
};

// O resumo de um episodio que acabou no ultimo step().
struct BodyEpisode3D {
    std::uint32_t environment = 0;
    std::uint32_t drill = 0;
    BodyStart3D start = BodyStart3D::Standing;
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
    // O passo util (o plano de recuperacao, 5.9): o toque de um pe que ficou
    // ao menos 0,12 s no ar e andou ao menos 10 cm no plano durante o voo, fora
    // dos primeiros 0,3 s. Por segundo do episodio (depois dos 0,3 s), o
    // tempo ate o primeiro (s; -1 sem passo), a maior sequencia de passos
    // (cada um a ate 1 s do anterior), o pedido medio de velocidade (m/s) e a
    // porta de pe media.
    float usefulStepsPerSecond = 0.0f;
    float firstStepSeconds = -1.0f;
    float longestStepRun = 0.0f;
    float meanRequested = 0.0f;
    float meanStandGate = 1.0f;
    // A fatia das decisoes com os dois pes fora do chao (o saltitar; a nota de
    // aquisicao nao o cobra: o custo de trabalho so volta na F1).
    float bothFeetAirFraction = 0.0f;
    // A estrela: os pontos alcancados por minuto e a deriva media (m) na espera
    // parado num ponto (-1 sem espera).
    float targetsPerMinute = 0.0f;
    float holdDrift = -1.0f;
    // As curvas: o afastamento medio da linha do arco (m; -1 sem curva).
    float arcError = -1.0f;
    // A base: a largura media entre os pes nas pisadas (m, de lado na pelve;
    // -1 sem pisada) e a fatia das pisadas com os pes cruzados.
    float stanceWidth = -1.0f;
    float crossFraction = 0.0f;
    // O pivo: o giro medio dos pes apoiados em torno da vertical (rad/s).
    float footPivot = 0.0f;
    // A postura funcional (graus, no rumo do peito; NaN sem amostra): em ritmo
    // constante, as medias do tronco inteiro e da parte baixa e as fatias alem
    // de -5 / -8 graus; o tronco inteiro freando e acelerando; nas curvas, a
    // fatia com o tronco inclinado para o lado da aceleracao.
    float cruiseFull = 0.0f;
    float cruiseLow = 0.0f;
    float cruiseFullBack = 0.0f;
    float cruiseLowBack = 0.0f;
    float brakingFull = 0.0f;
    float acceleratingFull = 0.0f;
    float inwardFraction = 0.0f;
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
    // Os pontos da estrela (o centro e as 8 pontas) no chao, quando o exercicio e a estrela.
    std::vector<Vec3> starPoints;
    // A velocidade do centro de massa e o rumo do peito (rad).
    Vec3 velocity;
    float chestHeading = 0.0f;
    // O empurrao de agora (push; -1 sem).
    int pushLink = -1;
    Vec3 pushForce;
    // O quanto o desenho dele aparece (1 durante; some depois que acaba).
    float pushFade = 1.0f;
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
    // Quantos estados guardados ha (somando as cenas).
    [[nodiscard]] std::size_t savedStates() const;

    // Muda o que pode mudar com o treino andando (vale do proximo passo ou
    // episodio em diante): a nota, o fim, a variacao do mundo, o controle e,
    // em cada exercicio, o roteiro, os pesos da tarefa, a duracao e o peso (as
    // cotas: os mundos se redistribuem e trocam de exercicio no proximo
    // episodio). O corpo, o contrato, o comeco e a lista dos exercicios nao mudam.
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
    // O visualizador do laboratorio (o treino nao usa): o controle de um mundo
    // na mao, no lugar do piloto (nullptr devolve ao piloto; a camera fica a
    // do piloto), e um empurrao num link por alguns segundos.
    void setManualControl(std::size_t environment, const ControlInput3D* input);
    void push(std::size_t environment, std::uint32_t link, Vec3 forceNewtons, float seconds);
    // Radianos por unidade de acao de cada canal (o ruido inicial da politica).
    [[nodiscard]] std::vector<float> actionRadiansPerUnit() const;
    [[nodiscard]] float lastStepMilliseconds() const;
    // O tempo de cada fase dos passos desde a ultima chamada (s de thread,
    // somado nas cenas; BodyStepPhaseNames3D), e zera.
    [[nodiscard]] std::array<double, BodyStepPhaseCount3D> takePhaseSeconds();
    [[nodiscard]] std::uint64_t physicsStepCount() const;

private:
    struct Impl;
    std::unique_ptr<Impl> m_impl;
};

} // namespace MatterEngine
