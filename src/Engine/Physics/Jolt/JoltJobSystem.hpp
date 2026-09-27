#pragma once

#include "Engine/Core/TaskScheduler.hpp"

#include <Jolt/Jolt.h>

#include <Jolt/Core/FixedSizeFreeList.h>
#include <Jolt/Core/JobSystemWithBarrier.h>

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <memory>

namespace MatterEngine {

// JPH::JobSystem implementado sobre o TaskScheduler da engine.
//
// Existe para honrar a regra de arquitetura do projeto: o pool de workers e
// unico no processo e middlewares nao criam pools proprios. A alternativa
// pronta, JPH::JobSystemThreadPool, sobe threads dela - duas hierarquias de
// worker competindo pelos mesmos nucleos, que e exatamente o que o
// TaskScheduler foi escrito para evitar (o backend PhysX seguia a mesma regra
// implementando PxCpuDispatcher).
//
// JobSystemWithBarrier cuida das barreiras, incluindo o detalhe importante de
// que a thread que espera AJUDA a executar os jobs daquela barreira em vez de
// bloquear ociosa - mesma filosofia de TaskScheduler::wait.
//
// Restam quatro responsabilidades, e sao todas elas:
//   GetMaxConcurrency, CreateJob, QueueJob/QueueJobs e FreeJob.
class JoltJobSystem final : public JPH::JobSystemWithBarrier {
public:
    JoltJobSystem(std::shared_ptr<TaskScheduler> scheduler,
        std::uint32_t maximumJobs, std::uint32_t maximumBarriers);
    ~JoltJobSystem() override;

    [[nodiscard]] int GetMaxConcurrency() const override {
        return m_maximumConcurrency;
    }

    JPH::JobHandle CreateJob(const char* name, JPH::ColorArg color,
        const JobFunction& jobFunction,
        JPH::uint32 dependencyCount = 0) override;

    // Diagnostico por passo, para alimentar PhysicsStepDiagnostics3D.
    void resetStepCounters() {
        m_submittedJobs.store(0, std::memory_order_relaxed);
    }
    [[nodiscard]] std::size_t submittedJobCount() const {
        return m_submittedJobs.load(std::memory_order_relaxed);
    }

protected:
    void QueueJob(Job* job) override;
    void QueueJobs(Job** jobs, JPH::uint32 jobCount) override;
    void FreeJob(Job* job) override;

private:
    static void executeJob(void* context) noexcept;

    std::shared_ptr<TaskScheduler> m_scheduler;
    JPH::FixedSizeFreeList<Job> m_jobs;
    int m_maximumConcurrency = 1;
    std::atomic<std::size_t> m_submittedJobs { 0 };
};

} // namespace MatterEngine
