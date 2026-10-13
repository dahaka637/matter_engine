// O professor de locomocao nos mundos de treino: a locomocao fisica do jogo
// conduzindo o corpo do RL. Ele anda pelo roteiro sem cair, o rotulo e finito,
// a fase da passada corre andando e o comando e o que ele tenta.
#include "Engine/RL/Environment/TrainingBatch3D.hpp"
#include "Engine/RL/Teacher/LocomotionTeacher3D.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

using namespace MatterEngine;

namespace {

void require(bool condition, const std::string& message) {
    if (!condition) throw std::runtime_error(message);
}

} // namespace

int main() {
    try {
        const std::filesystem::path assets = MATTERENGINE_TEST_ASSETS_DIR;
        // O andar por comando (a H3 v6), com a secao do professor.
        const std::filesystem::path walking = std::filesystem::path(MATTERENGINE_TEST_DATA_DIR) / "andar_comando.json";
        // So andando para a frente, sem empurroes: o caso que ele precisa fazer bem.
        LocomotionTeacherSettings3D teacherSettings = loadLocomotionTeacherSettings3D(walking);
        teacherSettings.standingWeight = teacherSettings.turnWeight = 0.0f;
        teacherSettings.backwardWeight = teacherSettings.sideWeight = teacherSettings.diagonalWeight = 0.0f;
        teacherSettings.forwardWeight = 1.0f;
        teacherSettings.curveFraction = 0.0f;
        teacherSettings.speedFraction = { 0.5f, 0.5f };
        teacherSettings.pushes = false;
        TrainingBatchSettings3D settings;
        settings.taskPath = walking;
        settings.environmentCount = 4;
        settings.workerThreads = 2;
        settings.seed = 5;
        auto teacher = std::make_shared<LocomotionTeacher3D>(assets, teacherSettings);
        settings.teacher = teacher;
        TrainingBatch3D batch(settings);
        const std::size_t N = batch.environmentCount(), A = batch.actionSize(), O = batch.observationSize();
        std::vector<float> ignored(N * A, 0.0f);
        std::vector<Vec3> start(N);
        std::vector<float> farthest(N, 0.0f);
        for (std::size_t e = 0; e < N; ++e) start[e] = batch.view(e).state->links.front().position;
        int falls = 0;
        float largest = 0.0f, fastest = 0.0f;
        bool phaseRuns = false, labelsFinite = true;
        const auto begin = std::chrono::steady_clock::now();
        const int decisions = static_cast<int>(10.0f / batch.policyStepSeconds());
        for (int i = 0; i < decisions; ++i) {
            batch.step(ignored);
            for (std::size_t e = 0; e < N; ++e) {
                // (O primeiro episodio comeca com o relogio sorteado: o fim por
                // tempo recomeca a conta, a queda conta como queda.)
                if (batch.terminated()[e] != 0 || batch.truncated()[e] != 0) {
                    falls += batch.terminated()[e] != 0 ? 1 : 0;
                    start[e] = batch.view(e).state->links.front().position;
                    continue;
                }
                const Vec3 now = batch.view(e).state->links.front().position;
                farthest[e] = std::max(farthest[e], std::hypot(now.x - start[e].x, now.y - start[e].y));
                for (std::size_t d = 0; d < A; ++d) {
                    const float label = batch.teacherActions()[e * A + d];
                    labelsFinite = labelsFinite && std::isfinite(label);
                    largest = std::max(largest, std::abs(label));
                }
                fastest = std::max(fastest, batch.view(e).command.x);
                const float cosine = batch.observations()[e * O + O - 1];
                phaseRuns = phaseRuns || (cosine < 0.9f && cosine > -0.9f);
            }
        }
        const double seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - begin).count();
        const float walked = *std::min_element(farthest.begin(), farthest.end());
        std::printf("professor: %d quedas em %zu mundos, o que menos andou %.1f m num episodio, comando ate %.2f m/s, "
                    "rotulo ate %.2f, %.0f decisoes/s\n",
            falls, N, walked, fastest, largest, static_cast<double>(decisions * N) / seconds);
        require(labelsFinite, "o rotulo do professor e finito");
        require(falls == 0, "andando para a frente, o professor nao cai");
        require(walked > 3.0f, "o professor anda");
        require(fastest > 0.5f, "o comando e o que ele tenta (a capsula andando)");
        require(phaseRuns, "a fase da passada corre andando");
        std::cout << "RL teacher tests passed\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "RL teacher tests failed: " << error.what() << '\n';
        return 1;
    }
}
