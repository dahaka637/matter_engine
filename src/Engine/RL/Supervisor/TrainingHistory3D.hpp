#pragma once

#include <cstdint>
#include <limits>
#include <filesystem>
#include <string>
#include <vector>

namespace MatterEngine {

// Uma rodada do treino (o que os graficos da tela usam).
struct TrainingIterationPoint3D {
    std::uint64_t iteration = 0;
    double time = 0.0;
    double steps = 0.0;
    double reward = 0.0;
    double episodeSeconds = 0.0;
    double fallFraction = 0.0;
    double sps = 0.0;
};

struct TrainingEvaluationPoint3D {
    std::uint64_t iteration = 0;
    double time = 0.0;
    double steps = 0.0;
    // O exercicio da tarefa e o solver ("jogo", "treino").
    std::string drill;
    std::string solver;
    // A fatia dos mundos que cairam, a mediana do erro de velocidade (m/s) e
    // os passos uteis por segundo (< 0 sem a medida).
    double falls = 0.0;
    double velocityError = -1.0;
    double usefulSteps = -1.0;
    // O retorno medio dos episodios do exercicio na avaliacao (NaN sem a medida).
    double episodeReturn = std::numeric_limits<double>::quiet_NaN();
};

// O que um exercicio mediu numa rodada do treino (da coluna details das rodadas):
// os episodios que terminaram nele, a fatia que caiu e o retorno medio.
struct TrainingDrillPoint3D {
    std::uint64_t iteration = 0;
    std::string drill;
    double episodes = 0.0;
    double falls = 0.0;
    double seconds = 0.0;
    double episodeReturn = std::numeric_limits<double>::quiet_NaN();
};

struct TrainingEventPoint3D {
    double time = 0.0;
    std::string kind;
    std::string author;
    std::string message;
};

struct TrainingHistory3D {
    std::vector<TrainingIterationPoint3D> iterations;
    std::vector<TrainingEvaluationPoint3D> evaluations;
    // Por rodada e por exercicio (as avaliacoes sao raras; estas saem de toda rodada).
    std::vector<TrainingDrillPoint3D> drills;
    std::vector<TrainingEventPoint3D> events;
};

// Le o historico de um treino no banco (rl.db), so leitura. As rodadas vem
// amostradas para no maximo maximumPoints (os graficos nao carregam milhoes).
[[nodiscard]] TrainingHistory3D readTrainingHistory3D(const std::filesystem::path& database, const std::string& run,
    double sinceSeconds = 0.0, std::size_t maximumPoints = 600);

} // namespace MatterEngine
