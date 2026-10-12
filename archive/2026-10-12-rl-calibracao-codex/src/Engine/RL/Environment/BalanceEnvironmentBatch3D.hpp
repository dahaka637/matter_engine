#pragma once

#include "Engine/RL/Environment/BalanceEnvironment3D.hpp"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <span>
#include <string>
#include <vector>

namespace MatterEngine {

enum class BalanceBatchExecutionMode3D : std::uint8_t { Sequential, Parallel };

struct BalanceBatchExecutionSettings3D {
  BalanceBatchExecutionMode3D mode = BalanceBatchExecutionMode3D::Sequential;
  // Quantidade de workers do pool central usados para distribuir cenas.
  // Zero escolhe automaticamente de acordo com o hardware e o tamanho do
  // lote. Cada solver permanece com concorrencia interna igual a um.
  std::uint32_t workerThreadCount = 0;
};

struct BalanceBatchResetResult3D {
  // Observacoes row-major somente para os indices retornados, na mesma
  // ordem de environmentIndices.
  std::vector<std::uint32_t> environmentIndices;
  std::vector<std::uint64_t> seeds;
  std::vector<float> observations;
};

struct BalanceBatchStepResult3D {
  // Todos os vetores possuem environmentCount linhas. Observations usa
  // layout row-major [ambiente][canal].
  std::vector<float> observations;
  std::vector<BalanceReward3D> rewards;
  std::vector<std::uint32_t> episodePolicySteps;
  std::vector<std::uint64_t> episodePhysicsSteps;
  std::vector<std::uint8_t> terminated;
  std::vector<std::uint8_t> truncated;
  std::vector<std::uint8_t> actionClipped;
  std::vector<std::string> terminationReasons;
};

// Contrato vetorizado. As cenas compartilham a mesma instancia da engine
// fisica e podem ser avancadas sequencialmente ou em paralelo pelo pool central
// da engine. No modo paralelo cada solver usa uma unica faixa; o paralelismo
// fica entre ambientes, sem pools adicionais nem concorrencia aninhada.
class BalanceEnvironmentBatch3D final {
public:
  BalanceEnvironmentBatch3D(RagdollProfile3D profile,
                            std::size_t environmentCount,
                            BalanceEnvironmentSettings3D settings = {},
                            BalanceBatchExecutionSettings3D execution = {});
  ~BalanceEnvironmentBatch3D();

  BalanceEnvironmentBatch3D(const BalanceEnvironmentBatch3D &) = delete;
  BalanceEnvironmentBatch3D &
  operator=(const BalanceEnvironmentBatch3D &) = delete;

  [[nodiscard]] std::size_t environmentCount() const;
  [[nodiscard]] std::size_t actionSize() const;
  [[nodiscard]] std::size_t observationSize() const;
  [[nodiscard]] std::uint32_t actionRepeat() const;
  [[nodiscard]] BalanceBatchExecutionMode3D executionMode() const;
  [[nodiscard]] std::uint32_t workerThreadCount() const;
  [[nodiscard]] const RLVectorSchema3D &actionSchema() const;
  [[nodiscard]] const RLVectorSchema3D &observationSchema() const;

  [[nodiscard]] BalanceBatchResetResult3D resetAll(std::uint64_t baseSeed);
  [[nodiscard]] BalanceBatchResetResult3D
  reset(std::span<const std::uint32_t> environmentIndices,
        std::span<const std::uint64_t> seeds);
  [[nodiscard]] BalanceBatchStepResult3D
  step(std::span<const float> rowMajorActions);
  [[nodiscard]] RagdollState3D state(std::size_t environmentIndex) const;

private:
  struct Impl;
  std::unique_ptr<Impl> m_impl;
};

} // namespace MatterEngine
