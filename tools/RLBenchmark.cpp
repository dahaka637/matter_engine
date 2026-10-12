#include "Engine/RL/Environment/TrainingBatch3D.hpp"

#include <chrono>
#include <cstdio>
#include <exception>
#include <iostream>
#include <random>
#include <string>
#include <string_view>
#include <vector>

// A velocidade dos mundos de treino num arranjo: decisoes por segundo
// (cada uma sao actionRepeat passos fisicos) com acoes aleatorias moderadas,
// como as de uma politica no comeco do treino.
//   MatterRLBenchmark [--envs 256] [--por-cena 4] [--threads 0] [--decisoes 300]
//                     [--solver P/V] [--tarefa <json>] [--amplitude 0.3]
int main(int argc, char** argv) {
    try {
        MatterEngine::TrainingBatchSettings3D settings;
        settings.taskPath = std::string(MATTERENGINE_ASSETS_DIR) + "/rl/tasks/h1_ficar_em_pe.json";
        int decisions = 300;
        float amplitude = 0.3f;
        for (int i = 1; i < argc; ++i) {
            const std::string_view arg = argv[i];
            const auto next = [&]() -> std::string {
                if (i + 1 >= argc) throw std::invalid_argument(std::string(arg) + " exige um valor");
                return argv[++i];
            };
            if (arg == "--envs") settings.environmentCount = static_cast<std::uint32_t>(std::stoul(next()));
            else if (arg == "--por-cena") settings.environmentsPerScene = static_cast<std::uint32_t>(std::stoul(next()));
            else if (arg == "--threads") settings.workerThreads = static_cast<std::uint32_t>(std::stoul(next()));
            else if (arg == "--decisoes") decisions = std::stoi(next());
            else if (arg == "--tarefa") settings.taskPath = next();
            else if (arg == "--amplitude") amplitude = std::stof(next());
            else if (arg == "--solver") {
                const std::string value = next();
                const auto slash = value.find('/');
                settings.solverPositionIterations = std::stoi(value.substr(0, slash));
                settings.solverVelocityIterations = std::stoi(value.substr(slash + 1));
            } else throw std::invalid_argument("argumento desconhecido: " + std::string(arg));
        }
        const auto setupBegin = std::chrono::steady_clock::now();
        MatterEngine::TrainingBatch3D batch(settings);
        const double setup =
            std::chrono::duration<double>(std::chrono::steady_clock::now() - setupBegin).count();
        std::mt19937 rng(1);
        std::normal_distribution<float> noise(0.0f, amplitude);
        std::vector<float> actions(batch.environmentCount() * batch.actionSize());
        std::size_t episodes = 0;
        double episodeSeconds = 0.0, sway = 0.0;
        const auto begin = std::chrono::steady_clock::now();
        for (int d = 0; d < decisions; ++d) {
            if (amplitude > 0.0f)
                for (float& a : actions) a = noise(rng);
            batch.step(actions);
            for (const MatterEngine::TrainingEpisode3D& episode : batch.finishedEpisodes()) {
                ++episodes;
                episodeSeconds += episode.seconds;
                sway += episode.comSwayRms;
            }
        }
        const double seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - begin).count();
        const double steps = static_cast<double>(decisions) * static_cast<double>(batch.environmentCount());
        std::printf("mundos %zu | por cena %u | threads %u | solver %d/%d | criacao %.2f s | %.0f decisoes/s | "
                    "%.0f passos fisicos/s | %.2f ms por decisao do lote | episodios %zu (media %.2f s, oscilacao %.3f m)\n",
            batch.environmentCount(), settings.environmentsPerScene, batch.workerThreads(),
            settings.solverPositionIterations, settings.solverVelocityIterations, setup, steps / seconds,
            steps / seconds * batch.task().actionRepeat, 1000.0 * seconds / decisions, episodes,
            episodes ? episodeSeconds / static_cast<double>(episodes) : 0.0,
            episodes ? sway / static_cast<double>(episodes) : 0.0);
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "MatterRLBenchmark: " << error.what() << '\n';
        return 1;
    }
}
