#include "Engine/RL/Environment/BalanceEnvironment3D.hpp"

#include <cmath>
#include <cstdint>
#include <exception>
#include <iomanip>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace {

void printUsage() {
  std::cout << "Uso: MatterRLBalanceProbe [--profile <arquivo>] [--steps <n>] "
               "[--seed <n>] [--dump-schema]\n"
            << "\n"
            << "Executa action=0 no primeiro ambiente headless de equilibrio. "
               "Nao treina uma politica e nao escreve arquivos.\n";
}

std::uint64_t parseUnsigned(std::string_view text, std::string_view option) {
  std::size_t consumed = 0;
  const std::string value(text);
  const unsigned long long parsed = std::stoull(value, &consumed);
  if (consumed != value.size())
    throw std::invalid_argument(std::string(option) +
                                " exige um inteiro sem sinal");
  return static_cast<std::uint64_t>(parsed);
}

void printSchema(const char *title,
                 const MatterEngine::RLVectorSchema3D &schema) {
  std::cout << "\n## " << title << " `" << schema.id << ".v" << schema.version
            << "`\n\n"
            << "| # | Canal | Unidade | Intervalo |\n"
            << "|---:|---|---|---:|\n";
  for (std::size_t index = 0; index < schema.channels.size(); ++index) {
    const MatterEngine::RLVectorChannel3D &channel = schema.channels[index];
    std::cout << "| " << index << " | `" << channel.name << "` | "
              << channel.unit << " | [" << channel.minimum << ", "
              << channel.maximum << "] |\n";
  }
}

} // namespace

int main(int argc, char **argv) {
  try {
    std::string profilePath =
        std::string(MATTERENGINE_ASSETS_DIR) +
        "/characters/football_player/FootballPlayerV1.ragdoll.json";
    std::uint32_t requestedSteps = 300;
    std::uint64_t seed = 1;
    bool dumpSchema = false;
    for (int index = 1; index < argc; ++index) {
      const std::string_view argument = argv[index];
      if (argument == "--help" || argument == "-h") {
        printUsage();
        return 0;
      }
      if (argument == "--dump-schema") {
        dumpSchema = true;
        continue;
      }
      if (argument == "--profile") {
        if (++index >= argc)
          throw std::invalid_argument(
              "--profile exige o caminho de um arquivo");
        profilePath = argv[index];
        continue;
      }
      if (argument == "--steps") {
        if (++index >= argc)
          throw std::invalid_argument("--steps exige um valor");
        const std::uint64_t value = parseUnsigned(argv[index], "--steps");
        if (value == 0 || value > std::numeric_limits<std::uint32_t>::max())
          throw std::invalid_argument(
              "--steps precisa estar no intervalo uint32 positivo");
        requestedSteps = static_cast<std::uint32_t>(value);
        continue;
      }
      if (argument == "--seed") {
        if (++index >= argc)
          throw std::invalid_argument("--seed exige um valor");
        seed = parseUnsigned(argv[index], "--seed");
        continue;
      }
      throw std::invalid_argument("argumento desconhecido: " +
                                  std::string(argument));
    }

    MatterEngine::BalanceEnvironmentSettings3D settings;
    settings.maximumEpisodePolicySteps = requestedSteps;
    MatterEngine::BalanceEnvironment3D environment(
        MatterEngine::loadRagdollProfile3D(profilePath), settings);
    const MatterEngine::BalanceResetResult3D reset = environment.reset(seed);
    std::vector<float> action(environment.actionSchema().size(), 0.0f);
    MatterEngine::BalanceStepResult3D result;
    for (std::uint32_t step = 0; step < requestedSteps; ++step) {
      result = environment.step(action);
      if (result.terminated || result.truncated)
        break;
    }

    const MatterEngine::RagdollState3D body = environment.state();
    const float pelvisHeight =
        body.links.empty() ? 0.0f : body.links.front().position.z;
    std::cout << std::fixed << std::setprecision(4)
              << "# Sonda do ambiente de equilibrio\n\n"
              << "> Rollout deterministico com action=0. E um baseline do "
                 "contrato, nao uma "
                 "politica treinada.\n\n"
              << "- Action schema: `" << environment.actionSchema().id << ".v"
              << environment.actionSchema().version << "` ("
              << environment.actionSchema().size() << " canais)\n"
              << "- Observation schema: `" << environment.observationSchema().id
              << ".v" << environment.observationSchema().version << "` ("
              << environment.observationSchema().size() << " canais)\n"
              << "- Reward schema: v" << environment.rewardSchemaVersion()
              << "\n"
              << "- Passo da politica: "
              << environment.policyStepSeconds() * 1000.0f << " ms; "
              << environment.actionRepeat() << " subpassos de "
              << MatterEngine::ReinforcementLearningFixedStepSeconds * 1000.0f
              << " ms\n"
              << "- Semente: " << reset.seed << "\n"
              << "- Passos de politica/fisica: " << result.episodePolicyStep
              << " / " << result.episodePhysicsStep << "\n"
              << "- Resultado: "
              << (result.terminated  ? "terminated"
                  : result.truncated ? "truncated"
                                     : "running")
              << " (`" << result.terminationReason << "`)\n"
              << "- Altura final da pelve: " << pelvisHeight << " m\n"
              << "- Reward final: " << result.reward.total << " (upright "
              << result.reward.upright << ", height " << result.reward.height
              << ", stillness " << result.reward.stillness
              << ", action penalty " << result.reward.actionPenalty << ")\n";
    if (!std::isfinite(result.reward.total))
      return 2;
    if (dumpSchema) {
      printSchema("Acoes", environment.actionSchema());
      printSchema("Observacoes", environment.observationSchema());
    }
    return 0;
  } catch (const std::exception &error) {
    std::cerr << "MatterRLBalanceProbe: " << error.what() << '\n';
    return 1;
  }
}
