#pragma once

#include "Engine/Physics/PhysicsScene3D.hpp"
#include "Engine/RL/Body/RLBody3D.hpp"
#include "Engine/RL/Contract/VectorLayout3D.hpp"
#include "Engine/RL/Environment/TrainingTask3D.hpp"
#include "Engine/RL/Environment/TrainingTeacher3D.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <span>
#include <string>
#include <vector>

namespace MatterEngine {

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
    // O visualizador: na avaliacao, cada episodio novo de um mundo segue o
    // roteiro do mundo seguinte da bateria (o mundo mais os episodios ja
    // comecados vezes os mundos), entao um mundo so passa por todas as idas.
    // A bateria deixa desligado (cada mundo, o seu roteiro).
    bool evaluationCycle = false;
    // Iteracoes do solver das juntas; negativo = as do corpo de RL. A
    // avaliacao confere a politica tambem com as 20/20 do jogo.
    int solverPositionIterations = -1;
    int solverVelocityIterations = -1;
    // A postura de todos os mundos (o id no corpo): a bateria de uma
    // habilidade e o visualizador. Vazio = a mistura da tarefa no treino (a
    // primeira postura dela na avaliacao).
    std::string posture;
    // O roteiro do comando de velocidade numa bateria de andar (trechos por
    // tempo do episodio). Vazio na avaliacao = parado.
    std::vector<TrainingCommandStep3D> commandSchedule;
    // O comando de olhar na avaliacao (com target.facing): -1 = o da tarefa
    // (todos os mundos com ele), 0 = livre, 1 = com ele, 2 = com ele girando
    // (velocidade e sentido sorteados a cada 2-4 s). Uma bateria mede cada um.
    int facing = -1;
    // O exercicio de todos os mundos (o id em task.drills): a bateria de um
    // exercicio, o visualizador. Vazio = sorteado pelo peso no treino, o
    // primeiro na avaliacao.
    std::string drill;
    // O professor (opcional): ele conduz todos os mundos no lugar da politica,
    // um boneco por cena, e teacherActions() devolve a acao dele em unidades
    // do RL. O comando e a fase do relogio de passada passam a ser os dele, e
    // o mundo fica sem variacao (massa, atrito, forca, atraso).
    std::shared_ptr<TrainingTeacher3D> teacher;
};

enum class TrainingEndReason3D : std::uint8_t {
    None = 0,
    FallContact,
    FallHeight,
    Numeric,
    TimeLimit,
    Boundary,
    // Passou do alvo ou foi para longe dele (target.missMeters alem do mais
    // perto que chegou): termina, sem o custo da queda.
    TargetMissed
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
    // O giro do tronco: a velocidade angular da pelve e do peito (RMS, rad/s).
    float trunkTurnRms = 0.0f;
    // O maior giro de um pe no chao desde o inicio (rad, em torno da vertical).
    float footTurnMax = 0.0f;
    // O quanto as juntas ficam longe da postura pedida (RMS, rad).
    float poseErrorRms = 0.0f;
    // Andar: o erro da velocidade no plano (m/s) e do giro (rad/s), em RMS; a
    // nota media do acompanhamento nas decisoes com comando de andar (-1 sem
    // nenhuma: o curriculo usa); a distancia do comeco e o desvio para o lado
    // do rumo inicial (m).
    float velocityErrorRms = 0.0f;
    float yawRateErrorRms = 0.0f;
    float velocityTracking = -1.0f;
    float distance = 0.0f;
    float lateralDrift = 0.0f;
    // O alvo: quantos ele alcancou, quando chegou ao primeiro (s, -1 sem
    // chegar) e a velocidade media dos trechos ate cada alvo alcancado (m/s).
    std::uint32_t targetsReached = 0;
    float firstTargetSeconds = -1.0f;
    float meanTargetSpeed = 0.0f;
    // A imitacao: o quanto as juntas ficam longe da referencia (RMS por eixo,
    // rad; -1 sem imitacao).
    float imitationErrorRms = -1.0f;
    // O comando de olhar: o quanto o link virado para ele ficou fora (RMS,
    // rad; -1 sem o comando no episodio).
    float facingErrorRms = -1.0f;
    // O exercicio do episodio (o indice em task.drills; 0 sem exercicios).
    std::uint32_t drill = 0;
    // O comando de velocidade: o quanto a velocidade em direcao ao alvo ficou
    // longe da pedida de agora (RMS, m/s; -1 sem o comando).
    float speedErrorRms = -1.0f;
    // A sola escorregando: a velocidade media de cada pe no chao (o centro mais
    // o giro em torno da vertical, como na nota footSlip; m/s; -1 sem contato).
    float footSlipMean = -1.0f;
    // Pare aqui: o quanto passou do disco depois de entrar, na media dos
    // alvos de parar alcancados (m; -1 sem nenhum).
    float stopOvershootMean = -1.0f;
    // As curvas: o quanto o centro de massa saiu da linha dos alvos (RMS, m;
    // -1 fora delas).
    float lineErrorRms = -1.0f;
    // Os passos: pisadas por segundo de cada pe e passinhos (pe menos de
    // 0,1 s no ar) por segundo.
    float cadence = 0.0f;
    float chatterRate = 0.0f;
    // A reversao media por decisao (a mudanca da acao contra a da decisao
    // anterior, quando tem sinais opostos): o indice de tremor.
    float meanReversal = 0.0f;
    // Os cortes: o tempo medio de cada um (da chegada num alvo com virada de
    // 45 graus ou mais ate andar a 80% da pedida rumo ao seguinte; s; -1 sem).
    float cutSeconds = -1.0f;
    // Os pulos: a fatia das zonas vencidas correndo e paradas (-1 sem), a
    // altura alcancada menos a pedida (m, media), a queda na aterrissagem (1,
    // 0, ou -1 sem pulo) e a volta a 80% da velocidade depois do correndo (s).
    float runJumpPassed = -1.0f;
    float standJumpPassed = -1.0f;
    float jumpRiseError = 0.0f;
    float landingFall = -1.0f;
    float jumpRecoverySeconds = -1.0f;
    // As forcas: o tempo medio ate voltar, em pe e na velocidade pedida,
    // depois de cada uma (s; -1 sem).
    float pushRecoverySeconds = -1.0f;
    // O levantar: o tempo ate ficar de pe (s do comeco do controle; -1 nunca),
    // se levantou em 5 s (1/0; -1 fora do levantar), o tempo ate o alvo (-1 sem
    // alvo, -2 sem chegar) e as quedas de novo depois de levantar.
    float getUpSeconds = -1.0f;
    float getUpStood5 = -1.0f;
    float getUpTargetSeconds = -1.0f;
    float getUpRefalls = -1.0f;
};

// Um mundo de treino visto por fora (o visualizador, a gravacao).
struct TrainingEnvironmentView3D {
    const RagdollState3D* state = nullptr;
    Vec3 origin;
    std::uint32_t steps = 0;
    // A postura do episodio (o indice em RLBody3D::postures) e o comando de
    // velocidade de agora (m/s para frente e para a esquerda, giro em rad/s).
    std::uint32_t posture = 0;
    Vec3 command;
    // O rumo no comeco do episodio (rad, do eixo x do mundo).
    float startHeading = 0.0f;
    Vec3 pushForce;
    int pushLink = -1;
    // O alvo de agora (no mundo, no chao), o raio de chegada e quantos ele ja
    // alcancou no episodio.
    bool hasTarget = false;
    Vec3 target;
    float targetRadius = 0.0f;
    std::uint32_t targetsReached = 0;
    // O exercicio do episodio (o indice em task.drills).
    std::uint32_t drill = 0;
    // O comando de velocidade: a pedida do trecho e a de agora (m/s).
    bool hasSpeedCommand = false;
    float speedCommand = 0.0f;
    float speedNow = 0.0f;
    // O desenho do exercicio no chao (a estrela: o centro e as direcoes).
    std::uint32_t layoutCount = 0;
    std::array<Vec3, 17> layout {};
    // Os proximos alvos ja no mapa (target.lookahead), em ordem.
    std::uint32_t upcomingCount = 0;
    std::array<Vec3, 3> upcoming {};
    // Frear e parar: o alvo de agora e os proximos sao de "pare aqui".
    bool targetStop = false;
    // Os pulos: a zona do trecho de agora (ligada, parada), o centro, o rumo do
    // caminho, a altura da barreira (a pelve de pe mais a subida pedida) e, no
    // pulo parado, a direcao pedida no mundo (pelo rumo do corpo agora; sem
    // ela, para cima).
    // O levantar: caindo (o corpo solto) e levantando (o clipe de levantar).
    bool knockdown = false;
    bool gettingUp = false;
    bool hasJump = false;
    bool jumpStanding = false;
    Vec3 jumpCenter;
    float jumpPath = 0.0f;
    float jumpBarrier = 0.0f;
    bool jumpUp = true;
    float jumpDirection = 0.0f;
    std::array<bool, 3> upcomingStop {};
    // Os ultimos alcancados no episodio (o mais novo primeiro).
    std::uint32_t reachedCount = 0;
    std::array<Vec3, 3> reached {};
    // Para onde o cerebro quer ir (a direcao em que o clipe de agora anda, no
    // mundo, unitaria) e para onde ele vai (a velocidade do centro de massa).
    bool hasIntent = false;
    Vec3 intent;
    Vec3 velocity;
    // Para onde a cabeca olha de verdade (a frente dela no plano, unitaria) e,
    // com o comando de olhar (target.facing), o rumo mandado (rad, no mundo).
    Vec3 gaze;
    bool hasFacing = false;
    float facingCommand = 0.0f;
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
    // Forca a postura de todos os mundos (o id no corpo; vazio volta a mistura
    // da tarefa), do proximo recomeco em diante.
    void setPosture(const std::string& id);
    // As posturas que a tarefa pede, na ordem dela.
    [[nodiscard]] std::vector<std::string> taskPostureIds() const;
    // Um comando de velocidade fixo num mundo (o visualizador), ate soltar.
    void setCommand(std::size_t environment, Vec3 command);
    void clearCommand(std::size_t environment);

    // Todos os mundos recomecam (sementes derivadas de seed).
    void reset(std::uint64_t seed);
    // actions: environmentCount x actionSize, linha por mundo.
    void step(std::span<const float> actions);

    [[nodiscard]] std::span<const float> observations() const;
    [[nodiscard]] std::span<const float> criticObservations() const;
    [[nodiscard]] std::span<const float> rewards() const;
    [[nodiscard]] std::span<const float> rewardComponents() const;
    // Com professor: a acao dele no ultimo step() (mundos x eixos), o alvo
    // de cada junta que, com a rigidez e o amortecimento do RL, daria em
    // media o torque que ele aplicou nos passos de fisica da decisao.
    [[nodiscard]] std::span<const float> teacherActions() const;
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
