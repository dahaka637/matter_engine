#pragma once

#include <cstdint>
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
    // A bateria (uma por habilidade: equilibrio_v1, postura_alerta_v1...) e o solver.
    std::string battery;
    std::string solver;
    double standFraction = 0.0;
    double meanSeconds = 0.0;
    double sway = 0.0;
    double score = 0.0;
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
    std::vector<TrainingEventPoint3D> events;
};

// Le o historico de um treino no banco (rl.db), so leitura. As rodadas vem
// amostradas para no maximo maximumPoints (os graficos nao carregam milhoes).
[[nodiscard]] TrainingHistory3D readTrainingHistory3D(const std::filesystem::path& database, const std::string& run,
    double sinceSeconds = 0.0, std::size_t maximumPoints = 600);

} // namespace MatterEngine
