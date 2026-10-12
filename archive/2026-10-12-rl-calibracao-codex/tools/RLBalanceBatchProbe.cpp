#include "Engine/RL/Environment/BalanceEnvironmentBatch3D.hpp"

#include <algorithm>
#include <chrono>
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

enum class ProbeMode { Sequential, Parallel, Both };

struct Measurement {
  MatterEngine::BalanceBatchExecutionMode3D mode =
      MatterEngine::BalanceBatchExecutionMode3D::Sequential;
  std::size_t environments = 0;
  std::uint32_t workers = 1;
  std::uint32_t vectorSteps = 0;
  std::uint64_t environmentSteps = 0;
  std::uint64_t physicsSteps = 0;
  std::uint64_t terminations = 0;
  std::uint64_t partialResets = 0;
  double setupMilliseconds = 0.0;
  double rolloutMilliseconds = 0.0;
};

void printUsage() {
  std::cout << "Uso: MatterRLBalanceBatchProbe [--profile <arquivo>] "
               "[--counts <csv>] [--steps <n>] [--seed <n>] "
               "[--mode sequential|parallel|both] [--workers <n>]\n"
            << "\n"
            << "Compara os executores headless sequencial e paralelo. "
               "Padrao: ambos, 1,2,4,6 ambientes e 60 decisoes por medicao. "
               "Nao e gate de desempenho.\n";
}

std::uint64_t parseUnsigned(std::string_view text, std::string_view option) {
  std::size_t consumed = 0;
  const std::string value(text);
  const unsigned long long parsed = std::stoull(value, &consumed);
  if (consumed != value.size())
    throw std::invalid_argument(std::string(option) +
                                " exige inteiro sem sinal");
  return static_cast<std::uint64_t>(parsed);
}

std::vector<std::size_t> parseCounts(std::string_view csv) {
  std::vector<std::size_t> counts;
  std::size_t begin = 0;
  while (begin <= csv.size()) {
    const std::size_t comma = csv.find(',', begin);
    const std::size_t end =
        comma == std::string_view::npos ? csv.size() : comma;
    if (end == begin)
      throw std::invalid_argument("--counts nao aceita itens vazios");
    const std::uint64_t value =
        parseUnsigned(csv.substr(begin, end - begin), "--counts");
    if (value == 0 || value > 64)
      throw std::invalid_argument("cada --counts precisa estar entre 1 e 64");
    counts.push_back(static_cast<std::size_t>(value));
    if (comma == std::string_view::npos)
      break;
    begin = comma + 1;
  }
  return counts;
}

Measurement measure(const MatterEngine::RagdollProfile3D &profile,
                    std::size_t count, std::uint32_t vectorSteps,
                    std::uint64_t baseSeed,
                    MatterEngine::BalanceBatchExecutionMode3D mode,
                    std::uint32_t requestedWorkers) {
  const auto setupBegin = std::chrono::steady_clock::now();
  MatterEngine::BalanceEnvironmentSettings3D settings;
  settings.maximumEpisodePolicySteps = 600;
  MatterEngine::BalanceBatchExecutionSettings3D execution;
  execution.mode = mode;
  if (mode == MatterEngine::BalanceBatchExecutionMode3D::Parallel &&
      requestedWorkers > 0) {
    execution.workerThreadCount = static_cast<std::uint32_t>(
        std::min<std::size_t>(requestedWorkers, count));
  }
  MatterEngine::BalanceEnvironmentBatch3D batch(profile, count, settings,
                                                execution);
  (void)batch.resetAll(baseSeed);
  std::vector<float> actions(batch.environmentCount() * batch.actionSize(),
                             0.0f);
  const auto setupEnd = std::chrono::steady_clock::now();

  Measurement result;
  result.mode = mode;
  result.environments = count;
  result.workers = batch.workerThreadCount();
  result.vectorSteps = vectorSteps;
  result.setupMilliseconds =
      std::chrono::duration<double, std::milli>(setupEnd - setupBegin).count();
  const auto rolloutBegin = std::chrono::steady_clock::now();
  std::uint64_t resetGeneration = 1;
  for (std::uint32_t vectorStep = 0; vectorStep < vectorSteps; ++vectorStep) {
    const MatterEngine::BalanceBatchStepResult3D step = batch.step(actions);
    result.environmentSteps += count;
    result.physicsSteps += count * batch.actionRepeat();

    std::vector<std::uint32_t> resetIndices;
    std::vector<std::uint64_t> resetSeeds;
    for (std::size_t index = 0; index < count; ++index) {
      if (step.terminated[index] == 0u && step.truncated[index] == 0u)
        continue;
      ++result.terminations;
      if (vectorStep + 1 >= vectorSteps)
        continue;
      resetIndices.push_back(static_cast<std::uint32_t>(index));
      resetSeeds.push_back(baseSeed + resetGeneration * count + index);
    }
    if (!resetIndices.empty()) {
      (void)batch.reset(resetIndices, resetSeeds);
      result.partialResets += resetIndices.size();
      ++resetGeneration;
    }
  }
  const auto rolloutEnd = std::chrono::steady_clock::now();
  result.rolloutMilliseconds =
      std::chrono::duration<double, std::milli>(rolloutEnd - rolloutBegin)
          .count();
  return result;
}

} // namespace

int main(int argc, char **argv) {
  try {
    std::string profilePath =
        std::string(MATTERENGINE_ASSETS_DIR) +
        "/characters/football_player/FootballPlayerV1.ragdoll.json";
    std::vector<std::size_t> counts{1, 2, 4, 6};
    std::uint32_t vectorSteps = 60;
    std::uint64_t seed = 1;
    ProbeMode mode = ProbeMode::Both;
    std::uint32_t workers = 0;
    for (int index = 1; index < argc; ++index) {
      const std::string_view argument = argv[index];
      if (argument == "--help" || argument == "-h") {
        printUsage();
        return 0;
      }
      if (argument == "--profile") {
        if (++index >= argc)
          throw std::invalid_argument("--profile exige um caminho");
        profilePath = argv[index];
        continue;
      }
      if (argument == "--counts") {
        if (++index >= argc)
          throw std::invalid_argument("--counts exige CSV");
        counts = parseCounts(argv[index]);
        continue;
      }
      if (argument == "--steps") {
        if (++index >= argc)
          throw std::invalid_argument("--steps exige um valor");
        const std::uint64_t parsed = parseUnsigned(argv[index], "--steps");
        if (parsed == 0 || parsed > 100000)
          throw std::invalid_argument("--steps precisa estar entre 1 e 100000");
        vectorSteps = static_cast<std::uint32_t>(parsed);
        continue;
      }
      if (argument == "--seed") {
        if (++index >= argc)
          throw std::invalid_argument("--seed exige um valor");
        seed = parseUnsigned(argv[index], "--seed");
        continue;
      }
      if (argument == "--mode") {
        if (++index >= argc)
          throw std::invalid_argument("--mode exige um valor");
        const std::string_view value = argv[index];
        if (value == "sequential")
          mode = ProbeMode::Sequential;
        else if (value == "parallel")
          mode = ProbeMode::Parallel;
        else if (value == "both")
          mode = ProbeMode::Both;
        else
          throw std::invalid_argument(
              "--mode exige sequential, parallel ou both");
        continue;
      }
      if (argument == "--workers") {
        if (++index >= argc)
          throw std::invalid_argument("--workers exige um valor");
        const std::uint64_t parsed = parseUnsigned(argv[index], "--workers");
        if (parsed == 0 || parsed > 64)
          throw std::invalid_argument("--workers precisa estar entre 1 e 64");
        workers = static_cast<std::uint32_t>(parsed);
        continue;
      }
      throw std::invalid_argument("argumento desconhecido: " +
                                  std::string(argument));
    }

    const MatterEngine::RagdollProfile3D profile =
        MatterEngine::loadRagdollProfile3D(profilePath);
    std::vector<Measurement> measurements;
    measurements.reserve(counts.size() * (mode == ProbeMode::Both ? 2u : 1u));
    for (const std::size_t count : counts) {
      if (mode != ProbeMode::Parallel) {
        measurements.push_back(
            measure(profile, count, vectorSteps, seed,
                    MatterEngine::BalanceBatchExecutionMode3D::Sequential, 0));
      }
      if (mode != ProbeMode::Sequential) {
        measurements.push_back(measure(
            profile, count, vectorSteps, seed,
            MatterEngine::BalanceBatchExecutionMode3D::Parallel, workers));
      }
    }

    std::cout
        << "# Medicao do lote de equilibrio\n\n"
        << "> Debug/headless, action=0. No modo paralelo cada solver usa uma "
           "faixa e as cenas usam o pool central. Os numeros caracterizam "
           "esta maquina e nao sao gate de "
           "desempenho.\n\n"
        << "| Modo | Ambientes | Workers | Setup | Decisoes vetoriais | Env "
           "steps/s | "
           "Physics steps/s | ms/decisao vetorial | Terminacoes/resets |\n"
        << "|:---|---:|---:|---:|---:|---:|---:|---:|---:|\n";
    for (const Measurement &measurement : measurements) {
      const double seconds = measurement.rolloutMilliseconds / 1000.0;
      const double environmentStepsPerSecond =
          static_cast<double>(measurement.environmentSteps) / seconds;
      const double physicsStepsPerSecond =
          static_cast<double>(measurement.physicsSteps) / seconds;
      const double millisecondsPerVectorStep =
          measurement.rolloutMilliseconds / measurement.vectorSteps;
      const char *modeName =
          measurement.mode ==
                  MatterEngine::BalanceBatchExecutionMode3D::Parallel
              ? "parallel"
              : "sequential";
      std::cout << std::fixed << std::setprecision(2) << "| " << modeName
                << " | " << measurement.environments << " | "
                << measurement.workers << " | " << measurement.setupMilliseconds
                << " ms | " << measurement.vectorSteps << " | "
                << environmentStepsPerSecond << " | " << physicsStepsPerSecond
                << " | " << millisecondsPerVectorStep << " | "
                << measurement.terminations << "/" << measurement.partialResets
                << " |\n";
    }
    return 0;
  } catch (const std::exception &error) {
    std::cerr << "MatterRLBalanceBatchProbe: " << error.what() << '\n';
    return 1;
  }
}
