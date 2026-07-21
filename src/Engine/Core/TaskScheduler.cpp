#include "Engine/Core/TaskScheduler.hpp"

#include <algorithm>
#include <atomic>
#include <condition_variable>
#include <deque>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <vector>

namespace MatterEngine {

struct TaskGroup::Impl {
    std::atomic<std::size_t> remaining { 0 };
    std::mutex mutex;
    std::condition_variable completed;
};

namespace {

struct ScheduledTask {
    Task task;
    TaskGroup::Impl* group = nullptr;
};

struct ParallelRangeTask {
    TaskScheduler::ParallelForFunction function = nullptr;
    void* context = nullptr;
    std::size_t begin = 0;
    std::size_t end = 0;
};

void executeParallelRange(void* context) noexcept {
    const auto* range = static_cast<const ParallelRangeTask*>(context);
    range->function(range->begin, range->end, range->context);
}

// A identidade e local a cada thread e inclui o pool. Assim, dois schedulers
// existentes em testes ou ferramentas jamais confundem suas filas locais.
thread_local const void* CurrentScheduler = nullptr;
thread_local std::uint32_t CurrentWorker = 0;

} // namespace

struct TaskScheduler::Impl {
    struct Worker {
        std::mutex mutex;
        std::deque<ScheduledTask> tasks;
        std::thread thread;
    };

    explicit Impl(std::uint32_t count) {
        workers.reserve(count);
        for (std::uint32_t index = 0; index < count; ++index) {
            workers.push_back(std::make_unique<Worker>());
        }
        for (std::uint32_t index = 0; index < count; ++index) {
            workers[index]->thread = std::thread([this, index] {
                workerLoop(index);
            });
        }
    }

    ~Impl() {
        waitIdle();
        stopping.store(true, std::memory_order_release);
        available.notify_all();
        for (const std::unique_ptr<Worker>& worker : workers) {
            if (worker->thread.joinable()) worker->thread.join();
        }
    }

    void submit(ScheduledTask scheduled) {
        if (scheduled.task.execute == nullptr) {
            throw std::invalid_argument("Tarefa sem funcao de execucao");
        }
        if (stopping.load(std::memory_order_acquire)) {
            throw std::runtime_error("Agendador de tarefas esta encerrando");
        }
        outstanding.fetch_add(1, std::memory_order_relaxed);
        if (scheduled.group != nullptr) {
            scheduled.group->remaining.fetch_add(1,
                std::memory_order_relaxed);
        }
        // O contador precisa anunciar o trabalho antes que a tarefa se torne
        // visivel em uma fila. Caso contrario, outro worker poderia rouba-la
        // entre o push e o incremento e causar underflow em queued.
        queued.fetch_add(1, std::memory_order_release);

        try {
            if (CurrentScheduler == this) {
                Worker& local = *workers[CurrentWorker];
                std::lock_guard lock(local.mutex);
                local.tasks.push_front(scheduled);
            } else {
                std::lock_guard lock(injectedMutex);
                injected.push_back(scheduled);
            }
        } catch (...) {
            if (scheduled.group != nullptr) {
                scheduled.group->remaining.fetch_sub(1,
                    std::memory_order_relaxed);
            }
            queued.fetch_sub(1, std::memory_order_relaxed);
            outstanding.fetch_sub(1, std::memory_order_relaxed);
            throw;
        }
        available.notify_one();
    }

    [[nodiscard]] bool popLocal(std::uint32_t workerIndex,
        ScheduledTask& output) {
        Worker& worker = *workers[workerIndex];
        std::lock_guard lock(worker.mutex);
        if (worker.tasks.empty()) return false;
        output = worker.tasks.front();
        worker.tasks.pop_front();
        return true;
    }

    [[nodiscard]] bool popInjected(ScheduledTask& output) {
        std::lock_guard lock(injectedMutex);
        if (injected.empty()) return false;
        output = injected.front();
        injected.pop_front();
        return true;
    }

    [[nodiscard]] bool steal(std::uint32_t thief,
        ScheduledTask& output) {
        const std::size_t count = workers.size();
        for (std::size_t offset = 1; offset < count; ++offset) {
            const std::size_t victimIndex = (thief + offset) % count;
            Worker& victim = *workers[victimIndex];
            std::unique_lock lock(victim.mutex, std::try_to_lock);
            if (!lock || victim.tasks.empty()) continue;
            output = victim.tasks.back();
            victim.tasks.pop_back();
            return true;
        }
        return false;
    }

    [[nodiscard]] bool acquire(std::uint32_t workerIndex,
        ScheduledTask& output) {
        const bool found = popLocal(workerIndex, output)
            || popInjected(output)
            || steal(workerIndex, output);
        if (found) queued.fetch_sub(1, std::memory_order_acq_rel);
        return found;
    }

    void execute(const ScheduledTask& scheduled) noexcept {
        scheduled.task.execute(scheduled.task.context);
        if (scheduled.group != nullptr
            && scheduled.group->remaining.fetch_sub(1,
                std::memory_order_acq_rel) == 1) {
            scheduled.group->completed.notify_all();
        }
        if (outstanding.fetch_sub(1, std::memory_order_acq_rel) == 1) {
            idle.notify_all();
        }
    }

    void workerLoop(std::uint32_t workerIndex) {
        CurrentScheduler = this;
        CurrentWorker = workerIndex;
        while (!stopping.load(std::memory_order_acquire)) {
            ScheduledTask scheduled;
            if (acquire(workerIndex, scheduled)) {
                execute(scheduled);
                continue;
            }
            std::unique_lock lock(availableMutex);
            available.wait(lock, [this] {
                return stopping.load(std::memory_order_acquire)
                    || queued.load(std::memory_order_acquire) > 0;
            });
        }
        CurrentScheduler = nullptr;
    }

    // A thread que aguarda ajuda a drenar a fila injetada. Isso reduz a
    // latencia em barreiras pequenas e evita deixar um nucleo ocioso apenas
    // porque ele coordena o frame.
    [[nodiscard]] bool helpOnce() {
        ScheduledTask scheduled;
        const bool found = CurrentScheduler == this
            ? acquire(CurrentWorker, scheduled)
            : popInjected(scheduled);
        if (!found) return false;
        if (CurrentScheduler != this) {
            queued.fetch_sub(1, std::memory_order_acq_rel);
        }
        execute(scheduled);
        return true;
    }

    void wait(TaskGroup::Impl& group) {
        while (group.remaining.load(std::memory_order_acquire) != 0) {
            if (helpOnce()) continue;
            std::unique_lock lock(group.mutex);
            group.completed.wait(lock, [&group] {
                return group.remaining.load(std::memory_order_acquire) == 0;
            });
        }
    }

    void waitIdle() {
        while (outstanding.load(std::memory_order_acquire) != 0) {
            if (helpOnce()) continue;
            std::unique_lock lock(idleMutex);
            idle.wait(lock, [this] {
                return outstanding.load(std::memory_order_acquire) == 0;
            });
        }
    }

    std::vector<std::unique_ptr<Worker>> workers;
    std::mutex injectedMutex;
    std::deque<ScheduledTask> injected;
    std::atomic<std::size_t> outstanding { 0 };
    std::atomic<std::size_t> queued { 0 };
    std::atomic<bool> stopping { false };
    std::mutex availableMutex;
    std::condition_variable available;
    std::mutex idleMutex;
    std::condition_variable idle;
};

TaskGroup::TaskGroup() : m_impl(std::make_unique<Impl>()) {}
TaskGroup::~TaskGroup() = default;

std::uint32_t TaskScheduler::recommendedWorkerCount() {
    const std::uint32_t hardware = std::max(1u,
        std::thread::hardware_concurrency());
    // Uma thread fica para coordenacao/render e outra para audio/SO. Em CPUs
    // pequenas ainda mantemos um worker, permitindo a mesma arquitetura sem
    // oversubscription grosseira.
    return hardware > 2 ? hardware - 2 : 1u;
}

TaskScheduler::TaskScheduler(const TaskSchedulerSettings& settings)
    : m_impl(std::make_unique<Impl>(settings.workerThreadCount > 0
        ? settings.workerThreadCount : recommendedWorkerCount())) {
}

TaskScheduler::~TaskScheduler() = default;

void TaskScheduler::submit(Task task, TaskGroup* group) {
    m_impl->submit({ task, group != nullptr ? group->m_impl.get() : nullptr });
}

void TaskScheduler::wait(TaskGroup& group) { m_impl->wait(*group.m_impl); }
void TaskScheduler::waitIdle() { m_impl->waitIdle(); }

void TaskScheduler::parallelFor(std::size_t itemCount,
    std::size_t minimumGrain, ParallelForFunction function, void* context) {
    if (itemCount == 0) return;
    if (function == nullptr) {
        throw std::invalid_argument("parallelFor sem funcao de execucao");
    }
    minimumGrain = std::max<std::size_t>(1, minimumGrain);
    const std::size_t desiredRanges =
        static_cast<std::size_t>(workerCount()) * 4;
    const std::size_t grain = std::max(minimumGrain,
        (itemCount + desiredRanges - 1) / desiredRanges);
    const std::size_t rangeCount = (itemCount + grain - 1) / grain;
    if (rangeCount <= 1) {
        function(0, itemCount, context);
        return;
    }

    std::vector<ParallelRangeTask> ranges(rangeCount);
    TaskGroup group;
    for (std::size_t index = 0; index < rangeCount; ++index) {
        ranges[index] = { function, context, index * grain,
            std::min(itemCount, (index + 1) * grain) };
        submit({ &executeParallelRange, &ranges[index] }, &group);
    }
    wait(group);
}

std::uint32_t TaskScheduler::workerCount() const {
    return static_cast<std::uint32_t>(m_impl->workers.size());
}

} // namespace MatterEngine
