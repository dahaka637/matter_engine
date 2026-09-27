#include "Engine/Physics/Jolt/JoltJobSystem.hpp"

#include <algorithm>
#include <stdexcept>
#include <thread>

namespace MatterEngine {

JoltJobSystem::JoltJobSystem(std::shared_ptr<TaskScheduler> scheduler,
    std::uint32_t maximumJobs, std::uint32_t maximumBarriers)
    : m_scheduler(std::move(scheduler)) {
    if (!m_scheduler) {
        throw std::invalid_argument(
            "JoltJobSystem exige um TaskScheduler");
    }
    JPH::JobSystemWithBarrier::Init(maximumBarriers);
    m_jobs.Init(maximumJobs, maximumJobs);
    // Os workers do pool mais a thread que chama Update: numa barreira ela
    // executa jobs em vez de esperar parada, portanto conta como capacidade
    // real de execucao. O Jolt usa este numero para dimensionar estruturas
    // internas e para decidir em quantas partes dividir o trabalho do passo.
    m_maximumConcurrency =
        static_cast<int>(std::max(1u, m_scheduler->workerCount())) + 1;
}

JoltJobSystem::~JoltJobSystem() = default;

JPH::JobHandle JoltJobSystem::CreateJob(const char* name,
    JPH::ColorArg color, const JobFunction& jobFunction,
    JPH::uint32 dependencyCount) {
    JPH::uint32 index = JPH::FixedSizeFreeList<Job>::cInvalidObjectIndex;
    for (;;) {
        index = m_jobs.ConstructObject(name, color, this, jobFunction,
            dependencyCount);
        if (index != JPH::FixedSizeFreeList<Job>::cInvalidObjectIndex) {
            break;
        }
        // Estourar a lista livre significa que o orcamento de jobs do passo
        // ficou pequeno para a cena. Ceder o processador deixa os jobs em voo
        // terminarem e devolverem slots; e a mesma estrategia da
        // implementacao oficial, e o caminho tem de continuar sem alocar.
        std::this_thread::yield();
    }

    Job* job = &m_jobs.Get(index);
    // O handle segura uma referencia antes do enfileiramento: sem isso o job
    // pode completar e se destruir antes de o handle existir.
    JPH::JobHandle handle(job);
    if (dependencyCount == 0) {
        QueueJob(job);
    }
    return handle;
}

void JoltJobSystem::QueueJob(Job* job) {
    // A referencia compensa a que executeJob libera no fim. Entre as duas, o
    // job pertence ao TaskScheduler.
    job->AddRef();
    m_submittedJobs.fetch_add(1, std::memory_order_relaxed);
    m_scheduler->submit({ &executeJob, job });
}

void JoltJobSystem::QueueJobs(Job** jobs, JPH::uint32 jobCount) {
    for (JPH::uint32 index = 0; index < jobCount; ++index) {
        QueueJob(jobs[index]);
    }
}

void JoltJobSystem::FreeJob(Job* job) {
    m_jobs.DestructObject(job);
}

void JoltJobSystem::executeJob(void* context) noexcept {
    auto* job = static_cast<Job*>(context);
    job->Execute();
    job->Release();
}

} // namespace MatterEngine
