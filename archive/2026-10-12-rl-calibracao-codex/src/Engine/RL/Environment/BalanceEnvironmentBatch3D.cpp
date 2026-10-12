#include "Engine/RL/Environment/BalanceEnvironmentBatch3D.hpp"

#include "Engine/Core/TaskScheduler.hpp"
#include "Engine/Physics/PhysicsEngine3D.hpp"

#include <algorithm>
#include <cmath>
#include <exception>
#include <limits>
#include <numeric>
#include <stdexcept>
#include <unordered_set>
#include <utility>

namespace MatterEngine {

struct BalanceEnvironmentBatch3D::Impl {
  std::shared_ptr<TaskScheduler> scheduler;
  std::shared_ptr<PhysicsEngine3D> engine;
  std::vector<std::unique_ptr<BalanceEnvironment3D>> environments;
  std::vector<bool> done;
  BalanceBatchExecutionMode3D executionMode =
      BalanceBatchExecutionMode3D::Sequential;
  std::uint32_t workerThreadCount = 1;
};

BalanceEnvironmentBatch3D::BalanceEnvironmentBatch3D(
    RagdollProfile3D profile, std::size_t environmentCount,
    BalanceEnvironmentSettings3D settings,
    BalanceBatchExecutionSettings3D execution)
    : m_impl(std::make_unique<Impl>()) {
  if (environmentCount == 0)
    throw std::invalid_argument(
        "O lote de equilibrio exige ao menos um ambiente");
  if (environmentCount >
      static_cast<std::size_t>(std::numeric_limits<std::uint32_t>::max()))
    throw std::invalid_argument(
        "O lote excede o espaco de indices de ambiente de 32 bits");
  if (execution.workerThreadCount > environmentCount)
    throw std::invalid_argument(
        "Workers do lote nao podem exceder a quantidade de ambientes");

  m_impl->executionMode = execution.mode;
  if (execution.mode == BalanceBatchExecutionMode3D::Parallel) {
    const std::uint32_t usefulWorkers = static_cast<std::uint32_t>(
        environmentCount > 1 ? environmentCount - 1 : 1);
    m_impl->workerThreadCount =
        execution.workerThreadCount > 0
            ? execution.workerThreadCount
            : std::max(1u, std::min(TaskScheduler::recommendedWorkerCount(),
                                    usefulWorkers));
  } else {
    if (execution.workerThreadCount != 0)
      throw std::invalid_argument(
          "workerThreadCount so e valido no modo paralelo");
    m_impl->workerThreadCount = 1;
  }
  m_impl->scheduler = std::make_shared<TaskScheduler>(
      TaskSchedulerSettings{m_impl->workerThreadCount});
  m_impl->engine = std::make_shared<PhysicsEngine3D>(m_impl->scheduler);
  m_impl->done.assign(environmentCount, true);
  m_impl->environments.reserve(environmentCount);
  for (std::size_t index = 0; index < environmentCount; ++index) {
    m_impl->environments.push_back(std::unique_ptr<BalanceEnvironment3D>(
        new BalanceEnvironment3D(profile, settings, m_impl->engine)));
  }
}

BalanceEnvironmentBatch3D::~BalanceEnvironmentBatch3D() = default;

std::size_t BalanceEnvironmentBatch3D::environmentCount() const {
  return m_impl->environments.size();
}

std::size_t BalanceEnvironmentBatch3D::actionSize() const {
  return actionSchema().size();
}

std::size_t BalanceEnvironmentBatch3D::observationSize() const {
  return observationSchema().size();
}

std::uint32_t BalanceEnvironmentBatch3D::actionRepeat() const {
  return m_impl->environments.front()->actionRepeat();
}

BalanceBatchExecutionMode3D BalanceEnvironmentBatch3D::executionMode() const {
  return m_impl->executionMode;
}

std::uint32_t BalanceEnvironmentBatch3D::workerThreadCount() const {
  return m_impl->workerThreadCount;
}

const RLVectorSchema3D &BalanceEnvironmentBatch3D::actionSchema() const {
  return m_impl->environments.front()->actionSchema();
}

const RLVectorSchema3D &BalanceEnvironmentBatch3D::observationSchema() const {
  return m_impl->environments.front()->observationSchema();
}

BalanceBatchResetResult3D
BalanceEnvironmentBatch3D::resetAll(std::uint64_t baseSeed) {
  std::vector<std::uint32_t> indices(environmentCount());
  std::iota(indices.begin(), indices.end(), 0u);
  std::vector<std::uint64_t> seeds(environmentCount());
  for (std::size_t index = 0; index < seeds.size(); ++index)
    seeds[index] = baseSeed + static_cast<std::uint64_t>(index);
  return reset(indices, seeds);
}

BalanceBatchResetResult3D BalanceEnvironmentBatch3D::reset(
    std::span<const std::uint32_t> environmentIndices,
    std::span<const std::uint64_t> seeds) {
  if (environmentIndices.size() != seeds.size())
    throw std::invalid_argument(
        "Indices e seeds do reset parcial precisam ter o mesmo tamanho");

  std::unordered_set<std::uint32_t> unique;
  unique.reserve(environmentIndices.size());
  for (const std::uint32_t index : environmentIndices) {
    if (index >= environmentCount())
      throw std::out_of_range("Indice de ambiente fora do lote");
    if (!unique.insert(index).second)
      throw std::invalid_argument(
          "Reset parcial nao aceita indices duplicados");
  }

  std::vector<BalanceResetResult3D> rows(environmentIndices.size());
  std::vector<std::exception_ptr> errors(environmentIndices.size());
  const auto executeRows = [&](std::size_t begin, std::size_t end) noexcept {
    for (std::size_t row = begin; row < end; ++row) {
      try {
        rows[row] =
            m_impl->environments[environmentIndices[row]]->reset(seeds[row]);
      } catch (...) {
        errors[row] = std::current_exception();
      }
    }
  };
  if (m_impl->executionMode == BalanceBatchExecutionMode3D::Parallel &&
      environmentIndices.size() > 1) {
    struct ResetContext {
      const decltype(executeRows) *function = nullptr;
    } context{&executeRows};
    m_impl->scheduler->parallelFor(
        environmentIndices.size(), 1,
        [](std::size_t begin, std::size_t end, void *raw) noexcept {
          const auto *context = static_cast<const ResetContext *>(raw);
          (*context->function)(begin, end);
        },
        &context);
  } else {
    executeRows(0, environmentIndices.size());
  }

  for (std::size_t row = 0; row < environmentIndices.size(); ++row)
    m_impl->done[environmentIndices[row]] = errors[row] != nullptr;
  for (const std::exception_ptr &error : errors) {
    if (error)
      std::rethrow_exception(error);
  }

  BalanceBatchResetResult3D result;
  result.environmentIndices.assign(environmentIndices.begin(),
                                   environmentIndices.end());
  result.seeds.assign(seeds.begin(), seeds.end());
  result.observations.reserve(environmentIndices.size() * observationSize());
  for (const BalanceResetResult3D &row : rows) {
    result.observations.insert(result.observations.end(),
                               row.observation.begin(), row.observation.end());
  }
  return result;
}

BalanceBatchStepResult3D
BalanceEnvironmentBatch3D::step(std::span<const float> rowMajorActions) {
  const std::size_t expected = environmentCount() * actionSize();
  if (rowMajorActions.size() != expected)
    throw std::invalid_argument("Dimensao do lote de actions incompativel");
  if (std::any_of(m_impl->done.begin(), m_impl->done.end(),
                  [](bool done) { return done; }))
    throw std::logic_error(
        "Ambientes encerrados precisam de reset parcial antes do proximo step");
  for (const float value : rowMajorActions) {
    if (!std::isfinite(value))
      throw std::invalid_argument("Lote de actions contem valor nao finito");
  }

  std::vector<BalanceStepResult3D> rows(environmentCount());
  std::vector<std::exception_ptr> errors(environmentCount());
  const auto executeRows = [&](std::size_t begin, std::size_t end) noexcept {
    for (std::size_t index = begin; index < end; ++index) {
      try {
        const std::span<const float> action =
            rowMajorActions.subspan(index * actionSize(), actionSize());
        rows[index] = m_impl->environments[index]->step(action);
      } catch (...) {
        errors[index] = std::current_exception();
      }
    }
  };
  if (m_impl->executionMode == BalanceBatchExecutionMode3D::Parallel &&
      environmentCount() > 1) {
    struct StepContext {
      const decltype(executeRows) *function = nullptr;
    } context{&executeRows};
    m_impl->scheduler->parallelFor(
        environmentCount(), 1,
        [](std::size_t begin, std::size_t end, void *raw) noexcept {
          const auto *context = static_cast<const StepContext *>(raw);
          (*context->function)(begin, end);
        },
        &context);
  } else {
    executeRows(0, environmentCount());
  }

  for (std::size_t index = 0; index < environmentCount(); ++index) {
    if (errors[index]) {
      std::fill(m_impl->done.begin(), m_impl->done.end(), true);
      std::rethrow_exception(errors[index]);
    }
  }

  BalanceBatchStepResult3D result;
  result.observations.reserve(environmentCount() * observationSize());
  result.rewards.reserve(environmentCount());
  result.episodePolicySteps.reserve(environmentCount());
  result.episodePhysicsSteps.reserve(environmentCount());
  result.terminated.reserve(environmentCount());
  result.truncated.reserve(environmentCount());
  result.actionClipped.reserve(environmentCount());
  result.terminationReasons.reserve(environmentCount());
  for (std::size_t index = 0; index < environmentCount(); ++index) {
    const BalanceStepResult3D &step = rows[index];
    result.observations.insert(result.observations.end(),
                               step.observation.begin(),
                               step.observation.end());
    result.rewards.push_back(step.reward);
    result.episodePolicySteps.push_back(step.episodePolicyStep);
    result.episodePhysicsSteps.push_back(step.episodePhysicsStep);
    result.terminated.push_back(step.terminated ? 1u : 0u);
    result.truncated.push_back(step.truncated ? 1u : 0u);
    result.actionClipped.push_back(step.actionClipped ? 1u : 0u);
    result.terminationReasons.push_back(step.terminationReason);
    m_impl->done[index] = step.terminated || step.truncated;
  }
  return result;
}

RagdollState3D
BalanceEnvironmentBatch3D::state(std::size_t environmentIndex) const {
  if (environmentIndex >= environmentCount())
    throw std::out_of_range("Indice de ambiente fora do lote");
  return m_impl->environments[environmentIndex]->state();
}

} // namespace MatterEngine
