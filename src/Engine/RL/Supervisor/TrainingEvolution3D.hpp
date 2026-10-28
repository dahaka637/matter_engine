#pragma once

#include "Engine/RL/Supervisor/TrainingHistory3D.hpp"

#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

namespace MatterEngine {

// A evolucao do treino e os graficos de um exercicio (a tela de RL; pedido do
// usuario, 27/10): um indice de 0 a 100 que junta, por exercicio, o sem cair,
// o seguir o pedido, a estabilidade e a velocidade alcancada, e os combina pelos
// pesos da tarefa. Sai das medidas que o treinador grava por exercicio em cada
// rodada (o treino, com o ruido da exploracao).

// O que a conta precisa saber de cada exercicio da tarefa.
struct TrainingDrillInfo3D {
    std::string id;
    // O peso no treino (0: fora do treino e fora do indice do treino).
    double weight = 0.0;
    // A velocidade-meta (m/s): a maior das faixas do exercicio. 0 nos que nao
    // andam (parado, giros, empurroes, levantar): sem a velocidade alcancada e
    // sem o grafico de velocidade.
    double goalSpeed = 0.0;
    // O levantar: conta o "levantou-se" no lugar do "sem cair".
    bool getUp = false;
};

// As partes do indice de um exercicio (0 a 1; NaN sem a medida).
struct TrainingScoreParts3D {
    static constexpr double None = std::numeric_limits<double>::quiet_NaN();
    double standing = None;
    double tracking = None;
    double stability = None;
    double reach = None;
    // As partes da estabilidade: o rumo do peito, o pe firme e o tronco.
    double facing = None;
    double footing = None;
    double trunk = None;
    // Os pesos de cada parte no indice (somam 1; a que falta sai e as outras crescem).
    static constexpr double StandingWeight = 0.40;
    static constexpr double TrackingWeight = 0.25;
    static constexpr double StabilityWeight = 0.20;
    static constexpr double ReachWeight = 0.15;
    // O indice (0 a 100; NaN sem nenhuma parte).
    [[nodiscard]] double index() const;
};

// As medidas de um exercicio juntas (a media pelos episodios), das quais saem as partes.
struct TrainingDrillMeasure3D {
    double episodes = 0.0;
    double falls = TrainingScoreParts3D::None;
    double stood = TrainingScoreParts3D::None;
    double velocityError = TrainingScoreParts3D::None;
    double requestedSpeed = TrainingScoreParts3D::None;
    double bodySpeed = TrainingScoreParts3D::None;
    double facingError = TrainingScoreParts3D::None;
    double slip = TrainingScoreParts3D::None;
    double postureError = TrainingScoreParts3D::None;
    // A recompensa por segundo (o retorno sobre a duracao dos episodios).
    double rewardPerSecond = TrainingScoreParts3D::None;
};

// As partes pelas medidas:
// - sem cair: 1 - a fatia que caiu (no levantar, a que ficou de pe);
// - seguir o pedido: 1 - o erro de velocidade sobre o pedido (o pedido com piso
//   de 1 m/s: parado, o erro e o quanto ele anda);
// - velocidade alcancada: a do corpo sobre a meta do exercicio (so nos que andam);
// - estabilidade: a media de exp(-rumo/0,5 rad), exp(-escorregar/0,5 m/s) e
//   exp(-postura/0,15 rad).
[[nodiscard]] TrainingScoreParts3D trainingScoreParts3D(const TrainingDrillMeasure3D& measure,
    const TrainingDrillInfo3D& info);

// O que os graficos desenham, todos na mesma grade: as rodadas do periodo em
// fatias iguais (o treino parado nao vira um buraco), cada ponto o fim de uma
// fatia. 0 a 100 nas partes e no indice; m/s nas velocidades; NaN sem dados.
struct TrainingCharts3D {
    std::vector<double> times;
    std::vector<std::uint64_t> iterations;
    // O treino todo: os exercicios com peso, pelos pesos.
    std::vector<float> evolution, standing, tracking, stability, reach;
    // O exercicio escolhido.
    std::vector<float> drillEvolution, drillStanding, drillTest, drillSpeed, drillRequested, drillStability,
        drillFacing, drillFooting, drillTrunk, drillReward;
    // As decisoes por segundo do treino.
    std::vector<float> decisions;
};

// Cada fatia junta as anteriores (ate MaximumTrail) ate ter MinimumEpisodes do
// exercicio: uma rodada tem poucos episodios de cada um.
inline constexpr double TrainingChartMinimumEpisodes = 30.0;
inline constexpr std::size_t TrainingChartMaximumTrail = 8;
// Menos episodios que isso (mesmo juntando): sem ponto. No comeco do treino so
// terminam os episodios curtos (os que caem), e o primeiro ponto mentia.
inline constexpr double TrainingChartMinimumPlotted = 8.0;
// O treino so vira ponto com esta fracao do peso da tarefa medida na fatia.
inline constexpr double TrainingChartMinimumCoverage = 0.5;

[[nodiscard]] TrainingCharts3D trainingCharts3D(const TrainingHistory3D& history,
    const std::vector<TrainingDrillInfo3D>& task, const std::string& drill, std::size_t slices);

// A evolucao real (28/10, pedido do usuario: o indice do treino "e confuso e nao
// reflete a evolucao real"): a avaliacao da rede sem ruido que o treinador roda
// de tempos em tempos (a faixa inteira de velocidade de cada exercicio com peso),
// um ponto por avaliacao, 0 a 100 (mais alto e melhor). Uma avaliacao com menos
// da metade dos exercicios medidos nao vira ponto.
struct TrainingEvaluationCharts3D {
    std::vector<double> times;
    std::vector<std::uint64_t> iterations;
    // Sem cair: a media dos exercicios e o pior deles (e qual).
    std::vector<float> standing, worstStanding;
    std::vector<std::string> worstDrill;
    // A velocidade atendida: a media de feita sobre pedida (ate 1) nos que andam.
    std::vector<float> reach;
};

[[nodiscard]] TrainingEvaluationCharts3D trainingEvaluationCharts3D(const TrainingHistory3D& history,
    const std::vector<TrainingDrillInfo3D>& task);

} // namespace MatterEngine
