#include "Engine/RL/Environment/BalanceEnvironmentBatch3D.hpp"

#include <cmath>
#include <cstdint>
#include <exception>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

using namespace MatterEngine;

void require(bool condition, const char *message) {
  if (!condition)
    throw std::runtime_error(message);
}

void requireNear(float actual, float expected, float tolerance,
                 const char *message) {
  if (std::abs(actual - expected) > tolerance)
    throw std::runtime_error(message);
}

RagdollProfile3D loadFootballPlayer() {
  return loadRagdollProfile3D(
      std::string(MATTERENGINE_TEST_ASSETS_DIR) +
      "/characters/football_player/FootballPlayerV1.ragdoll.json");
}

void testRowMajorStepAndPartialReset() {
  BalanceEnvironmentSettings3D settings;
  settings.maximumEpisodePolicySteps = 2;
  BalanceEnvironmentBatch3D batch(loadFootballPlayer(), 3, settings);
  require(batch.environmentCount() == 3, "Quantidade de ambientes incorreta");
  require(batch.actionSize() == 39 && batch.observationSize() == 138,
          "Schemas do lote divergiram do ambiente individual");

  const BalanceBatchResetResult3D reset = batch.resetAll(100);
  require(reset.environmentIndices == std::vector<std::uint32_t>({0, 1, 2}),
          "Reset total retornou indices incorretos");
  require(reset.seeds == std::vector<std::uint64_t>({100, 101, 102}),
          "Reset total nao derivou as sementes por ambiente");
  require(reset.observations.size() == 3 * batch.observationSize(),
          "Reset total retornou layout incorreto");
  for (std::size_t channel = 0; channel < batch.observationSize(); ++channel) {
    require(
        reset.observations[channel] ==
            reset.observations[batch.observationSize() + channel],
        "Sem randomizacao, resets identicos deveriam ter a mesma observacao");
  }

  std::vector<float> actions(batch.environmentCount() * batch.actionSize(),
                             0.0f);
  actions[batch.actionSize()] = 0.25f;
  const BalanceBatchStepResult3D first = batch.step(actions);
  require(first.observations.size() ==
              batch.environmentCount() * batch.observationSize(),
          "Step retornou observacoes com layout incorreto");
  require(first.rewards.size() == 3 && first.terminated.size() == 3 &&
              first.truncated.size() == 3,
          "Telemetria por ambiente incompleta");
  require(first.episodePhysicsSteps == std::vector<std::uint64_t>({4, 4, 4}),
          "Ambientes nao executaram quatro passos fisicos");
  const std::size_t previousActionOffset =
      batch.observationSize() - batch.actionSize();
  require(std::abs(first.observations[previousActionOffset]) < 0.00001f,
          "Linha zero recebeu action de outro ambiente");
  require(
      std::abs(
          first.observations[batch.observationSize() + previousActionOffset] -
          0.25f) < 0.00001f,
      "Layout row-major perdeu a action da linha um");

  const BalanceBatchStepResult3D second = batch.step(actions);
  require(second.episodePolicySteps == std::vector<std::uint32_t>({2, 2, 2}),
          "Contadores de politica do lote incorretos");
  require(second.truncated == std::vector<std::uint8_t>({1, 1, 1}),
          "Time limit nao truncou todas as linhas");

  const std::vector<std::uint32_t> firstAndLast{0, 2};
  const std::vector<std::uint64_t> firstAndLastSeeds{200, 202};
  const BalanceBatchResetResult3D partial =
      batch.reset(firstAndLast, firstAndLastSeeds);
  require(partial.observations.size() == 2 * batch.observationSize(),
          "Reset parcial retornou quantidade errada de linhas");

  bool unfinishedResetRejected = false;
  try {
    (void)batch.step(actions);
  } catch (const std::logic_error &) {
    unfinishedResetRejected = true;
  }
  require(unfinishedResetRejected,
          "Lote avancou com um ambiente terminado ainda sem reset");

  const std::vector<std::uint32_t> middle{1};
  const std::vector<std::uint64_t> middleSeed{201};
  (void)batch.reset(middle, middleSeed);
  const BalanceBatchStepResult3D afterReset = batch.step(actions);
  require(afterReset.episodePolicySteps ==
              std::vector<std::uint32_t>({1, 1, 1}),
          "Reset parcial nao reiniciou os contadores independentemente");
}

void testBatchRejectsAmbiguousInputsBeforeStepping() {
  BalanceEnvironmentBatch3D batch(loadFootballPlayer(), 2);
  (void)batch.resetAll(1);

  bool wrongActionsRejected = false;
  try {
    const std::vector<float> wrong(batch.actionSize(), 0.0f);
    (void)batch.step(wrong);
  } catch (const std::invalid_argument &) {
    wrongActionsRejected = true;
  }
  require(wrongActionsRejected,
          "Lote de actions com dimensao errada foi aceito");

  bool duplicateResetRejected = false;
  try {
    const std::vector<std::uint32_t> duplicate{0, 0};
    const std::vector<std::uint64_t> seeds{5, 6};
    (void)batch.reset(duplicate, seeds);
  } catch (const std::invalid_argument &) {
    duplicateResetRejected = true;
  }
  require(duplicateResetRejected, "Reset parcial duplicado foi aceito");

  const BalanceBatchStepResult3D step = batch.step(
      std::vector<float>(batch.environmentCount() * batch.actionSize(), 0.0f));
  require(step.episodePolicySteps == std::vector<std::uint32_t>({1, 1}),
          "Entrada rejeitada alterou parcialmente o lote");
}

void testParallelExecutorPreservesRowSemantics() {
  BalanceEnvironmentSettings3D environmentSettings;
  environmentSettings.maximumEpisodePolicySteps = 3;
  BalanceBatchExecutionSettings3D parallelSettings;
  parallelSettings.mode = BalanceBatchExecutionMode3D::Parallel;
  parallelSettings.workerThreadCount = 2;

  BalanceEnvironmentBatch3D sequential(loadFootballPlayer(), 4,
                                       environmentSettings);
  BalanceEnvironmentBatch3D parallel(loadFootballPlayer(), 4,
                                     environmentSettings, parallelSettings);
  require(parallel.executionMode() == BalanceBatchExecutionMode3D::Parallel,
          "Lote nao publicou o executor paralelo");
  require(parallel.workerThreadCount() == 2,
          "Lote nao preservou a quantidade de workers explicita");

  const BalanceBatchResetResult3D sequentialReset = sequential.resetAll(500);
  const BalanceBatchResetResult3D parallelReset = parallel.resetAll(500);
  require(sequentialReset.seeds == parallelReset.seeds,
          "Executor paralelo alterou as sementes");
  require(sequentialReset.observations.size() ==
              parallelReset.observations.size(),
          "Executor paralelo alterou o layout do reset");
  for (std::size_t index = 0; index < sequentialReset.observations.size();
       ++index) {
    requireNear(parallelReset.observations[index],
                sequentialReset.observations[index], 0.000001f,
                "Reset paralelo divergiu do sequencial");
  }

  std::vector<float> actions(
      sequential.environmentCount() * sequential.actionSize(), 0.0f);
  actions[0] = 0.15f;
  actions[sequential.actionSize() + 1] = -0.20f;
  actions[2 * sequential.actionSize() + 2] = 0.35f;
  actions[3 * sequential.actionSize() + 3] = -0.40f;
  for (std::uint32_t decision = 0; decision < 3; ++decision) {
    const BalanceBatchStepResult3D expected = sequential.step(actions);
    const BalanceBatchStepResult3D actual = parallel.step(actions);
    require(expected.episodePolicySteps == actual.episodePolicySteps &&
                expected.episodePhysicsSteps == actual.episodePhysicsSteps &&
                expected.terminated == actual.terminated &&
                expected.truncated == actual.truncated &&
                expected.actionClipped == actual.actionClipped &&
                expected.terminationReasons == actual.terminationReasons,
            "Executor paralelo alterou a telemetria por linha");
    require(expected.observations.size() == actual.observations.size(),
            "Executor paralelo alterou o tamanho da observacao");
    for (std::size_t index = 0; index < expected.observations.size(); ++index) {
      requireNear(actual.observations[index], expected.observations[index],
                  0.00001f,
                  "Executor paralelo alterou uma observacao deterministica");
    }
    for (std::size_t index = 0; index < expected.rewards.size(); ++index) {
      requireNear(actual.rewards[index].total, expected.rewards[index].total,
                  0.00001f,
                  "Executor paralelo alterou o reward deterministico");
    }
  }
}

void testParallelConfigurationValidation() {
  BalanceBatchExecutionSettings3D tooMany;
  tooMany.mode = BalanceBatchExecutionMode3D::Parallel;
  tooMany.workerThreadCount = 3;
  bool tooManyRejected = false;
  try {
    BalanceEnvironmentBatch3D batch(loadFootballPlayer(), 2, {}, tooMany);
  } catch (const std::invalid_argument &) {
    tooManyRejected = true;
  }
  require(tooManyRejected,
          "Lote aceitou mais workers que ambientes independentes");

  BalanceBatchExecutionSettings3D ambiguous;
  ambiguous.workerThreadCount = 1;
  bool sequentialWorkersRejected = false;
  try {
    BalanceEnvironmentBatch3D batch(loadFootballPlayer(), 2, {}, ambiguous);
  } catch (const std::invalid_argument &) {
    sequentialWorkersRejected = true;
  }
  require(sequentialWorkersRejected,
          "Lote sequencial aceitou configuracao de workers sem efeito");
}

} // namespace

int main() {
  try {
    testRowMajorStepAndPartialReset();
    testBatchRejectsAmbiguousInputsBeforeStepping();
    testParallelExecutorPreservesRowSemantics();
    testParallelConfigurationValidation();
    std::cout << "Balance environment batch tests passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << "Balance environment batch tests failed: " << error.what()
              << '\n';
    return 1;
  }
}
