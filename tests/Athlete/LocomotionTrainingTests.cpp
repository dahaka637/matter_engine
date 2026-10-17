// O treino da locomocao (a etapa B do plano do RL): a nota, a postura
// funcional, a tarefa e o lote - curto, com a acao zero (sem rede).
#include "Engine/Athlete/Training/LocomotionTraining3D.hpp"

#include <cmath>
#include <cstdio>
#include <stdexcept>
#include <string>
#include <vector>

using namespace MatterEngine;

namespace {

int checks = 0;

void expect(bool condition, const std::string& what) {
    ++checks;
    if (!condition) throw std::runtime_error(what);
}

const std::string Assets = MATTERENGINE_ASSETS_DIR;

void rewardTests() {
    const LocomotionRewardWeights3D weights;
    LocomotionStepMeasure3D perfect;
    perfect.styleValid = true;
    const auto best = locomotionRewardTerms3D(perfect, weights, 1.0f / 30.0f);
    expect(std::abs(locomotionReward3D(best) - (0.40f + 0.10f + 0.20f) / 30.0f) < 1e-6f,
        "sem erro e sem custo: a velocidade, o rumo e o estilo inteiros, por segundo");
    LocomotionStepMeasure3D fell = perfect;
    fell.fell = true;
    expect(std::abs(locomotionReward3D(locomotionRewardTerms3D(fell, weights, 1.0f / 30.0f)) - (locomotionReward3D(best) - 1.0f))
            < 1e-6f,
        "a queda tira 1 de uma vez");
    LocomotionStepMeasure3D touchdown = perfect;
    touchdown.contactScore = 2.0f;
    expect(std::abs(locomotionRewardTerms3D(touchdown, weights, 1.0f / 30.0f)[static_cast<std::size_t>(
                        LocomotionRewardTerm3D::Contact)] - 0.2f) < 1e-6f,
        "as metas de contato contam por pouso, nao por segundo");
    LocomotionStepMeasure3D costly = perfect;
    costly.slip = 0.4f;
    costly.power = 30.0f;
    costly.assist = 0.34f;
    expect(locomotionReward3D(locomotionRewardTerms3D(costly, weights, 1.0f / 30.0f)) < locomotionReward3D(best),
        "os custos tiram");

    LocomotionPostureInput3D upright;
    expect(locomotionPostureExcess3D(upright) == 0.0f, "em pe e reto: dentro do envelope");
    const float lean = 30.0f * 3.14159265f / 180.0f;
    LocomotionPostureInput3D forward;
    forward.chestUp = { std::sin(lean), 0.0f, std::cos(lean) };
    const float still = locomotionPostureExcess3D(forward);
    expect(std::abs(still - 15.0f * 3.14159265f / 180.0f) < 1e-3f, "parado, 30 graus para a frente: 15 graus fora");
    forward.commandAcceleration = { 5.0f, 0.0f, 0.0f };
    expect(locomotionPostureExcess3D(forward) < still - 0.2f, "arrancando, o envelope abre para a frente");
    LocomotionPostureInput3D braking = forward;
    braking.chestUp = { -std::sin(lean), 0.0f, std::cos(lean) };
    braking.commandAcceleration = {};
    braking.measuredAcceleration = { -6.0f, 0.0f, 0.0f };
    expect(locomotionPostureExcess3D(braking) < 0.15f, "freando forte, o envelope abre para tras");
}

void taskTests() {
    const LocomotionTask3D task = loadLocomotionTask3D(Assets + "/rl/tasks/locomocao_f1f2.json");
    expect(task.drills.size() == 5 && task.drills[0].id == "parado" && task.drills[4].id == "curva",
        "a tarefa das fases 1 e 2: cinco exercicios");
    expect(task.standingAssistPercent == 0.0f && task.walkingAssistPercent == 0.0f && task.feetAssistPercent == 0.0f
            && task.getUpAssistPercent == 0.0f,
        "a ajuda em 0% (os quatro controles)");
    expect(std::abs(task.caps.touchdownMeters - 0.06f) < 1e-6f && std::abs(task.caps.jointRadians - 0.10f) < 1e-6f,
        "os tetos do consenso da etapa A");
}

void batchTests() {
    LocomotionTrainingSettings3D settings;
    settings.taskPath = Assets + "/rl/tasks/locomocao_f1f2.json";
    settings.assetsDirectory = Assets;
    settings.environments = 6;
    settings.threads = 3;
    settings.seed = 7;
    settings.evaluation = true;
    settings.evaluationSeconds = 1.0f;
    LocomotionTraining3D batch(settings);
    batch.setAuthority(1.0f, 1.0f, 1.0f, 1.0f);
    const std::size_t worlds = batch.environmentCount(), obs = batch.observationSize(), actions = batch.actionSize();
    expect(obs == batch.contract().observationSize() && actions == batch.contract().actionSize(),
        "a entrada e a acao sao as do contrato v2");
    expect(batch.criticObservationSize() == obs + 5 + 7, "o critico: a entrada, o exercicio e o piloto");
    const LocomotionTraining3D::Mirror mirror = batch.mirror();
    expect(mirror.observation.source.size() == obs && mirror.critic.source.size() == batch.criticObservationSize()
            && mirror.action.source.size() == actions,
        "o espelho cobre a entrada, o critico e a acao");
    const std::vector<float> fractions = batch.actionCapFractions();
    expect(fractions.size() == actions && fractions[0] > 0.19f && fractions[0] < 0.21f,
        "o teto do pouso e 0,06 de 0,30 m");
    const std::vector<float> zero(worlds * actions, 0.0f);
    std::size_t episodes = 0;
    bool finite = true;
    for (int step = 0; step < 31; ++step) {
        batch.step(zero);
        for (float v : batch.observations()) finite &= std::isfinite(v);
        for (float v : batch.criticObservations()) finite &= std::isfinite(v);
        for (float v : batch.rewards()) finite &= std::isfinite(v);
        for (const LocomotionEpisode3D& e : batch.finishedEpisodes()) {
            ++episodes;
            expect(locomotionEpisodeRow3D(e).size() == locomotionEpisodeColumns3D().size(), "a linha do episodio e a das colunas");
            expect(e.reason == LocomotionEnd3D::TimeLimit, std::string("um segundo de pe, a acao zero: ")
                    + locomotionEndName3D(e.reason));
        }
    }
    expect(finite, "a entrada, o critico e a nota sempre finitos");
    expect(episodes == worlds, "cada mundo terminou um episodio de 1 s (30 decisoes)");
    expect(!batch.frameLines(0).empty(), "as linhas da folha de quadros");

    // O mesmo comeco e a mesma acao: a mesma nota (determinismo por mundo).
    settings.environments = 2;
    settings.threads = 2;
    LocomotionTraining3D a(settings), b(settings);
    a.setAuthority(1.0f, 1.0f, 1.0f, 1.0f);
    b.setAuthority(1.0f, 1.0f, 1.0f, 1.0f);
    std::vector<float> wiggle(2 * actions);
    bool same = true;
    for (int step = 0; step < 20; ++step) {
        for (std::size_t i = 0; i < wiggle.size(); ++i) wiggle[i] = 0.6f * std::sin(0.3f * static_cast<float>(step + i));
        a.step(wiggle);
        b.step(wiggle);
        for (std::size_t i = 0; i < 2; ++i) same &= a.rewards()[i] == b.rewards()[i];
    }
    expect(same, "dois lotes com a mesma semente e a mesma acao dao a mesma nota");
}

} // namespace

int main() {
    try {
        rewardTests();
        taskTests();
        batchTests();
    } catch (const std::exception& error) {
        std::fprintf(stderr, "treino da locomocao: FALHOU: %s (%d checagens antes)\n", error.what(), checks);
        return 1;
    }
    std::printf("treino da locomocao: %d checagens, tudo certo\n", checks);
    return 0;
}
