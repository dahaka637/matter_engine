#include "Engine/RL/Environment/BalanceEnvironment3D.hpp"

#include <cmath>
#include <exception>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

using namespace MatterEngine;

void require(bool condition, const char *message) {
  if (!condition)
    throw std::runtime_error(message);
}

RagdollProfile3D loadFootballPlayer() {
  return loadRagdollProfile3D(
      std::string(MATTERENGINE_TEST_ASSETS_DIR) +
      "/characters/football_player/FootballPlayerV1.ragdoll.json");
}

bool finiteVector(const std::vector<float> &values) {
  for (const float value : values) {
    if (!std::isfinite(value))
      return false;
  }
  return true;
}

void requireObservationInSchema(const BalanceEnvironment3D &environment,
                                const std::vector<float> &observation) {
  const RLVectorSchema3D &schema = environment.observationSchema();
  require(observation.size() == schema.size(), "Observacao divergiu do schema");
  require(finiteVector(observation), "Observacao contem valor nao finito");
  for (std::size_t index = 0; index < observation.size(); ++index) {
    require(observation[index] >= schema.channels[index].minimum - 0.00001f &&
                observation[index] <= schema.channels[index].maximum + 0.00001f,
            "Observacao saiu dos limites publicados");
  }
}

void testSchemasAndFixedStepContract() {
  BalanceEnvironmentSettings3D settings;
  settings.maximumEpisodePolicySteps = 2;
  BalanceEnvironment3D environment(loadFootballPlayer(), settings);
  require(environment.actionSchema().id == "matter.balance.action",
          "Schema de action errado");
  require(environment.actionSchema().version == 1, "Versao de action errada");
  require(environment.actionSchema().size() == 39,
          "Action deveria cobrir os 39 DOFs");
  require(environment.observationSchema().id == "matter.balance.observation",
          "Schema de observacao errado");
  require(environment.observationSchema().version == 1,
          "Versao de observacao errada");
  require(environment.observationSchema().size() == 138,
          "Layout da observacao balance.v1 mudou");
  require(environment.rewardSchemaVersion() == 1, "Versao de reward errada");
  require(environment.actionRepeat() == 4,
          "Action repeat deixou de ser quatro");
  require(std::abs(environment.policyStepSeconds() - 1.0f / 30.0f) < 0.000001f,
          "Politica deixou de operar a 30 Hz");

  const BalanceResetResult3D reset = environment.reset(1234);
  require(reset.seed == 1234, "Reset nao preservou a semente");
  requireObservationInSchema(environment, reset.observation);

  std::vector<float> action(environment.actionSchema().size(), 0.0f);
  const BalanceStepResult3D first = environment.step(action);
  require(first.episodePolicyStep == 1,
          "Contador de passos de politica incorreto");
  require(first.episodePhysicsStep == 4,
          "Step nao executou quatro passos fisicos");
  require(!first.truncated, "Episodio truncou cedo demais");
  require(std::isfinite(first.reward.total), "Reward nao finito");
  requireObservationInSchema(environment, first.observation);

  const BalanceStepResult3D second = environment.step(action);
  require(second.episodePolicyStep == 2 && second.episodePhysicsStep == 8,
          "Contadores do segundo step incorretos");
  if (!second.terminated) {
    require(second.truncated, "Time limit nao truncou o episodio");
    require(second.terminationReason == "time_limit",
            "Motivo de truncamento incorreto");
  }
}

void testActionValidationAndClipping() {
  BalanceEnvironment3D environment(loadFootballPlayer());
  (void)environment.reset(7);

  bool wrongSizeRejected = false;
  try {
    const std::vector<float> wrong(environment.actionSchema().size() - 1, 0.0f);
    (void)environment.step(wrong);
  } catch (const std::invalid_argument &) {
    wrongSizeRejected = true;
  }
  require(wrongSizeRejected, "Action com dimensao errada nao foi rejeitada");

  bool nonFiniteRejected = false;
  try {
    std::vector<float> invalid(environment.actionSchema().size(), 0.0f);
    invalid.front() = std::numeric_limits<float>::quiet_NaN();
    (void)environment.step(invalid);
  } catch (const std::invalid_argument &) {
    nonFiniteRejected = true;
  }
  require(nonFiniteRejected, "Action nao finita nao foi rejeitada");

  std::vector<float> clipped(environment.actionSchema().size(), 0.0f);
  clipped.front() = 2.0f;
  const BalanceStepResult3D result = environment.step(clipped);
  require(result.actionClipped, "Clamp de action nao foi publicado");
  requireObservationInSchema(environment, result.observation);
}

void testResetIsReproducibleForSameSeedAndActions() {
  BalanceEnvironment3D first(loadFootballPlayer());
  BalanceEnvironment3D second(loadFootballPlayer());
  const BalanceResetResult3D firstReset = first.reset(99);
  const BalanceResetResult3D secondReset = second.reset(99);
  require(firstReset.observation == secondReset.observation,
          "Reset igual produziu observacoes iniciais diferentes");

  std::vector<float> action(first.actionSchema().size(), 0.0f);
  action[0] = 0.1f;
  action[12] = -0.2f;
  const BalanceStepResult3D firstStep = first.step(action);
  const BalanceStepResult3D secondStep = second.step(action);
  require(firstStep.observation.size() == secondStep.observation.size(),
          "Ambientes iguais divergiram em tamanho");
  for (std::size_t index = 0; index < firstStep.observation.size(); ++index) {
    require(std::abs(firstStep.observation[index] -
                     secondStep.observation[index]) < 0.00001f,
            "Ambientes iguais divergiram no primeiro step");
  }
}

} // namespace

int main() {
  try {
    testSchemasAndFixedStepContract();
    testActionValidationAndClipping();
    testResetIsReproducibleForSameSeedAndActions();
    std::cout << "Balance environment tests passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << "Balance environment tests failed: " << error.what() << '\n';
    return 1;
  }
}
