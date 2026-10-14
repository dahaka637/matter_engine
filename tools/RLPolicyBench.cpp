// O Experimento A da reformulacao do RL (vault: Aprendizado_por_Reforco/
// novo_plano/plano_claude_rl.md, secao 14.1): quanto custa rodar a politica
// no jogo, 22 jogadores a 30 decisoes por segundo, com o runtime de hoje
// (PolicyRuntime3D, um jogador por vez). Imprime Markdown.
//
// Uso: MatterRLPolicyBench politica.mrlpolicy [mais.mrlpolicy ...]
#include "Engine/RL/Policy/PolicyRuntime3D.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <random>
#include <vector>

int main(int argc, char** argv) {
    using namespace MatterEngine;
    if (argc < 2) {
        std::fprintf(stderr, "uso: MatterRLPolicyBench politica.mrlpolicy [...]\n");
        return 1;
    }
    constexpr int Players = 22;
    constexpr int Ticks = 900;  // 30 s de jogo a 30 decisoes por segundo
    std::printf("| Politica | Entradas | Eixos | Um jogador (us) | 22, um por um (us por tique) | Fatia de um nucleo | "
                "22 numa conta so (us por tique) | Fatia de um nucleo | Diferenca entre os dois |\n");
    std::printf("|---|---:|---:|---:|---:|---:|---:|---:|---:|\n");
    for (int a = 1; a < argc; ++a) {
        try {
            const PolicyRuntime3D policy(argv[a]);
            std::mt19937 rng(3);
            std::normal_distribution<float> gauss(0.0f, 1.0f);
            std::vector<std::vector<float>> observations(Players, std::vector<float>(policy.observationSize()));
            for (auto& o : observations)
                for (float& v : o) v = gauss(rng);
            std::vector<float> action(policy.actionSize());
            for (int warm = 0; warm < 50; ++warm) policy.evaluate(observations[0], action);
            const auto start = std::chrono::steady_clock::now();
            float sink = 0.0f;
            for (int tick = 0; tick < Ticks; ++tick) {
                for (int p = 0; p < Players; ++p) {
                    observations[static_cast<std::size_t>(p)][0] = static_cast<float>(tick) * 1e-3f;
                    policy.evaluate(observations[static_cast<std::size_t>(p)], action);
                    sink += action[0];
                }
            }
            const double seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
            const double perTick = seconds / Ticks * 1e6;
            const double perDecision = perTick / Players;
            // Os 22 numa conta so.
            std::vector<float> batch(Players * policy.observationSize()), batchActions(Players * policy.actionSize());
            for (int p = 0; p < Players; ++p)
                std::copy(observations[static_cast<std::size_t>(p)].begin(), observations[static_cast<std::size_t>(p)].end(),
                    batch.begin() + static_cast<std::ptrdiff_t>(p) * static_cast<std::ptrdiff_t>(policy.observationSize()));
            policy.evaluateBatch(batch, batchActions, Players);
            // A mesma conta que um por um (a ordem das somas muda pouco).
            float worst = 0.0f;
            for (int p = 0; p < Players; ++p) {
                policy.evaluate(observations[static_cast<std::size_t>(p)], action);
                for (std::size_t k = 0; k < action.size(); ++k)
                    worst = std::max(worst, std::abs(action[k] - batchActions[static_cast<std::size_t>(p) * action.size() + k]));
            }
            const auto batchStart = std::chrono::steady_clock::now();
            for (int tick = 0; tick < Ticks; ++tick) {
                batch[0] = static_cast<float>(tick) * 1e-3f;
                policy.evaluateBatch(batch, batchActions, Players);
                sink += batchActions[0];
            }
            const double batchTick =
                std::chrono::duration<double>(std::chrono::steady_clock::now() - batchStart).count() / Ticks * 1e6;
            // As multiplicacoes: a soma de entradas x saidas das camadas.
            std::printf("| %s | %zu | %zu | %.1f | %.0f | %.1f%% | %.0f | %.1f%% | %.1e |\n", argv[a], policy.observationSize(),
                policy.actionSize(), perDecision, perTick, 100.0 * perTick * 30.0 / 1e6, batchTick,
                100.0 * batchTick * 30.0 / 1e6, static_cast<double>(worst));
            if (sink == 12345.0f) std::printf(" ");
        } catch (const std::exception& error) {
            std::fprintf(stderr, "%s: %s\n", argv[a], error.what());
            return 1;
        }
    }
    return 0;
}
