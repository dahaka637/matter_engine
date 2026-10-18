#pragma once

#include "Engine/Athlete/Contract/LocomotionContract3D.hpp"
#include "Engine/Athlete/Training/LocomotionReward3D.hpp"
#include "Engine/Athlete/Training/LocomotionTask3D.hpp"
#include "Engine/RL/Contract/VectorLayout3D.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <span>
#include <string>
#include <vector>

namespace MatterEngine {

class AthleteBundle3D;

struct LocomotionTrainingSettings3D {
    std::filesystem::path taskPath;
    // A pasta assets do repositorio (o personagem e os clipes).
    std::string assetsDirectory;
    std::uint32_t environments = 256;
    // As threads (0: as do hardware menos duas).
    std::uint32_t threads = 0;
    std::uint64_t seed = 1;
    // Avaliacao: a duracao fixa (sem o fim pelo tempo da tarefa).
    bool evaluation = false;
    float evaluationSeconds = 20.0f;
    // Todos os mundos num exercicio (o id; a avaliacao, a previa). Vazio: a
    // mistura da tarefa.
    std::string drill;
};

enum class LocomotionEnd3D : std::uint8_t { None = 0, Fall, Numeric, TimeLimit, Boundary, Failure };
[[nodiscard]] const char* locomotionEndName3D(LocomotionEnd3D end);

// O resumo de um episodio que acabou no ultimo step().
struct LocomotionEpisode3D {
    std::uint32_t environment = 0;
    std::uint32_t drill = 0;
    LocomotionEnd3D reason = LocomotionEnd3D::None;
    std::uint32_t steps = 0;
    float seconds = 0.0f;
    float episodeReturn = 0.0f;
    // As medidas (medias por decisao; RMS nos erros).
    float velocityError = 0.0f;
    float facingError = 0.0f;
    float styleError = -1.0f;
    float contactPerTouchdown = -1.0f;
    float touchdownsPerSecond = 0.0f;
    float slip = 0.0f;
    float power = 0.0f;
    float assist = 0.0f;
    float posture = 0.0f;
    float residual = 0.0f;
    float jerk = 0.0f;
    float meanRequested = 0.0f;
    float meanSpeed = 0.0f;
    float distance = 0.0f;
    // O piloto: os pontos por minuto, a deriva na espera (m; -1 sem espera) e
    // o afastamento da curva (m; -1 sem curva).
    float targetsPerMinute = 0.0f;
    float holdDrift = -1.0f;
    // O tremor medio do corpo nos instantes em que o comando pedia quietude (a
    // media pesada pela quietude; -1 sem nenhum): a medida da quietude, mesmo com
    // o peso dela em zero.
    float tremor = -1.0f;
    float arcError = -1.0f;
    // O diagnostico do erro de velocidade (fora da nota): filtrado em ~um passo, e so no cruzeiro.
    float velocityErrorWindow = -1.0f;
    float velocityErrorCruise = -1.0f;
    // As parcelas da nota somadas no episodio.
    std::array<float, LocomotionRewardTermCount3D> terms {};
};

// As colunas de um episodio (a ordem de locomotionEpisodeRow3D).
[[nodiscard]] const std::vector<std::string>& locomotionEpisodeColumns3D();
[[nodiscard]] std::vector<float> locomotionEpisodeRow3D(const LocomotionEpisode3D& episode);

// O treino da locomocao (a etapa B do plano, vault 04_locomocao/
// plano_fases_1_2.md): os mundos do lote (o atleta do jogo, comandado, no
// fisico, com a ajuda do episodio), o piloto do Corpo v2 pelo controle do
// jogo, o contrato v2 (a entrada, a acao, a rampa e o limite de taxa), a nota
// e os fins. step() recebe a acao de cada mundo, segue a rampa a cada tick
// (4 ticks: a decisao a 30 Hz), mede, da a nota e publica a entrada, o
// critico, as parcelas e os fins. Um mundo que termina recomeca no mesmo step
// (o critico do estado final fica em terminalCriticObservations).
class LocomotionTraining3D final {
public:
    explicit LocomotionTraining3D(const LocomotionTrainingSettings3D& settings);
    ~LocomotionTraining3D();
    LocomotionTraining3D(const LocomotionTraining3D&) = delete;
    LocomotionTraining3D& operator=(const LocomotionTraining3D&) = delete;

    [[nodiscard]] std::size_t environmentCount() const;
    [[nodiscard]] std::size_t observationSize() const;
    [[nodiscard]] std::size_t criticObservationSize() const;
    [[nodiscard]] std::size_t actionSize() const;
    [[nodiscard]] std::uint32_t workerThreads() const;
    [[nodiscard]] float policyStepSeconds() const;
    [[nodiscard]] const LocomotionContract3D& contract() const;
    [[nodiscard]] const RLVectorLayout3D& criticLayout() const;
    // O espelho da entrada, do critico e da acao.
    struct Mirror {
        LocomotionMirrorMap3D observation;
        LocomotionMirrorMap3D critic;
        LocomotionMirrorMap3D action;
    };
    [[nodiscard]] Mirror mirror() const;
    [[nodiscard]] const LocomotionTask3D& task() const;
    [[nodiscard]] const AthleteBundle3D& athlete() const;
    // As escalas da saida: o teto da fase de cada canal sobre a escala maxima
    // (o desvio inicial da exploracao parte dele).
    [[nodiscard]] std::vector<float> actionCapFractions() const;

    // A tarefa nova (pesos da nota, tetos, mistura); os exercicios precisam
    // ser os mesmos, na mesma ordem.
    void updateTask(LocomotionTask3D task);
    // As autoridades das cabecas (a rampa de entrada do treinador).
    void setAuthority(float footwork, float balance, float pose, float gain);
    // A virada brusca nos episodios novos (a da tarefa, quando ligada): 0 a da
    // tarefa (o sorteio do dropout no treino; sempre ligada na avaliacao), 1
    // desligada, 2 ligada. Vale a partir do proximo recomeco (reset() inclusive).
    void setTurnMoveMode(int mode);
    // Se a virada brusca esta ligada no episodio de agora de um mundo.
    [[nodiscard]] bool turnMoveActive(std::size_t world) const;
    // As fatias dos exercicios nos proximos comecos (uma por exercicio).
    void setDrillWeights(std::span<const float> weights);
    [[nodiscard]] std::span<const std::uint32_t> worldDrills() const;

    void reset(std::uint64_t seed);
    void step(std::span<const float> actions);

    [[nodiscard]] std::span<const float> observations() const;
    [[nodiscard]] std::span<const float> criticObservations() const;
    [[nodiscard]] std::span<const float> terminalCriticObservations() const;
    [[nodiscard]] std::span<const float> rewardTerms() const;
    [[nodiscard]] std::span<const float> rewards() const;
    [[nodiscard]] std::span<const std::uint8_t> terminated() const;
    [[nodiscard]] std::span<const std::uint8_t> truncated() const;
    [[nodiscard]] const std::vector<LocomotionEpisode3D>& finishedEpisodes() const;
    // O corpo de um mundo para as folhas de quadros (o formato da previa do
    // Corpo v2: os coliders, os contatos, o alvo, o pedido e a velocidade).
    [[nodiscard]] std::string frameLines(std::size_t environment) const;
    [[nodiscard]] double lastStepMilliseconds() const;

private:
    struct Impl;
    std::unique_ptr<Impl> m_impl;
};

} // namespace MatterEngine
