#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>

namespace MatterEngine {

// Unidade minima de trabalho do agendador. O chamador conserva a memoria
// apontada por context ate execute terminar; essa representacao sem
// std::function evita uma alocacao por tarefa nos caminhos quentes da engine.
struct Task {
    using ExecuteFunction = void (*)(void*) noexcept;

    ExecuteFunction execute = nullptr;
    void* context = nullptr;
};

// Contador de conclusao reutilizavel para um grupo de tarefas independentes.
// Sua implementacao permanece privada para que mutexes e primitivas do
// sistema operacional nao vazem pela API publica do modulo Core.
class TaskGroup final {
public:
    struct Impl;

    TaskGroup();
    ~TaskGroup();
    TaskGroup(const TaskGroup&) = delete;
    TaskGroup& operator=(const TaskGroup&) = delete;

private:
    std::unique_ptr<Impl> m_impl;

    friend class TaskScheduler;
};

struct TaskSchedulerSettings {
    // Zero escolhe automaticamente uma quantidade coerente com o hardware,
    // preservando capacidade para a thread principal e servicos do sistema.
    std::uint32_t workerThreadCount = 0;
};

// Pool central de workers persistentes da MatterEngine. As filas locais usam
// ordem LIFO para conservar cache; workers sem trabalho roubam tarefas antigas
// do fundo das demais filas, equilibrando cargas irregulares sem criar threads
// durante um frame.
class TaskScheduler final {
public:
    struct Impl;

    explicit TaskScheduler(const TaskSchedulerSettings& settings = {});
    ~TaskScheduler();
    TaskScheduler(const TaskScheduler&) = delete;
    TaskScheduler& operator=(const TaskScheduler&) = delete;

    void submit(Task task, TaskGroup* group = nullptr);
    void wait(TaskGroup& group);
    void waitIdle();

    using ParallelForFunction = void (*)(std::size_t begin,
        std::size_t end, void* context) noexcept;
    void parallelFor(std::size_t itemCount, std::size_t minimumGrain,
        ParallelForFunction function, void* context);

    [[nodiscard]] std::uint32_t workerCount() const;
    [[nodiscard]] static std::uint32_t recommendedWorkerCount();

private:
    std::unique_ptr<Impl> m_impl;
};

} // namespace MatterEngine
