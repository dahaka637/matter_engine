#pragma once

#include "Engine/Core/TaskScheduler.hpp"

#include <cstddef>
#include <utility>

// O paralelismo da modelagem: o agendador de tarefas da engine (um so, criado
// no primeiro uso, com os workers vivos entre as chamadas) repartindo um
// intervalo em pedacos. A funcao recebe [begin, end) e nao pode lancar
// excecao. Pode ser aninhado: quem espera ajuda a correr as tarefas da fila.
namespace MatterEngine::Modeling {

[[nodiscard]] TaskScheduler& modelingScheduler();

template <typename Function>
void parallelFor(std::size_t count, std::size_t grain, Function&& function) {
    struct Context {
        Function* function;
    } context { &function };
    modelingScheduler().parallelFor(
        count, grain, [](std::size_t begin, std::size_t end, void* data) noexcept { (*static_cast<Context*>(data)->function)(begin, end); },
        &context);
}

} // namespace MatterEngine::Modeling
