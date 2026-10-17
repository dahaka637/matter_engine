#include "Engine/Athlete/Training/LocomotionBatch3D.hpp"

#include "Engine/Core/TaskScheduler.hpp"
#include "Engine/Materials/MaterialLibrary.hpp"

#include <algorithm>
#include <atomic>
#include <thread>
#include <vector>

namespace MatterEngine {

PhysicsSceneSettings3D locomotionBatchScene3D() {
    PhysicsSceneSettings3D scene;
    scene.workerThreadCount = 1;
    scene.runJobsInline = true;
    // (Um ragdoll de ~20 links, o chao e folga para as caixas e empurroes
    // das perturbacoes.)
    scene.maximumBodies = 128u;
    scene.maximumBodyPairs = 4096u;
    scene.maximumContactConstraints = 2048u;
    scene.scratchBufferSizeBytes = 4u * 1024u * 1024u;
    return scene;
}

struct LocomotionBatch3D::Impl {
    std::uint32_t workers = 1;
    std::shared_ptr<TaskScheduler> scheduler;
    std::unique_ptr<PhysicsEngine3D> engine;
    MaterialLibrary materials;
    std::vector<std::unique_ptr<LocomotionWorld3D>> worlds;
    std::vector<AthleteCommand3D> commands;
    std::vector<std::uint8_t> failed;
};

LocomotionBatch3D::LocomotionBatch3D(const AthleteRuntimeAssets3D& assets, const LocomotionBatchSettings3D& settings)
    : m_impl(std::make_unique<Impl>()) {
    Impl& impl = *m_impl;
    const std::uint32_t hardware = std::max(1u, std::thread::hardware_concurrency());
    impl.workers = settings.threads > 0 ? settings.threads : std::max(1u, hardware > 2 ? hardware - 2 : 1u);
    // (A thread que chama step() tambem trabalha no parallelFor.)
    impl.scheduler = std::make_shared<TaskScheduler>(TaskSchedulerSettings { std::max(1u, impl.workers - 1) });
    impl.engine = std::make_unique<PhysicsEngine3D>(impl.scheduler);
    LocomotionWorldSettings3D world = settings.world;
    world.scene = locomotionBatchScene3D();
    impl.worlds.reserve(settings.worlds);
    for (std::uint32_t i = 0; i < settings.worlds; ++i)
        impl.worlds.push_back(std::make_unique<LocomotionWorld3D>(*impl.engine, impl.materials, assets, world));
    impl.commands.assign(settings.worlds, AthleteCommand3D {});
    impl.failed.assign(settings.worlds, 0);
}

LocomotionBatch3D::~LocomotionBatch3D() {
    // (Os mundos antes do motor e do agendador.)
    m_impl->worlds.clear();
}

std::size_t LocomotionBatch3D::size() const { return m_impl->worlds.size(); }
std::uint32_t LocomotionBatch3D::threads() const { return m_impl->workers; }
LocomotionWorld3D& LocomotionBatch3D::world(std::size_t index) { return *m_impl->worlds.at(index); }
std::span<AthleteCommand3D> LocomotionBatch3D::commands() { return m_impl->commands; }
bool LocomotionBatch3D::failed(std::size_t index) const { return m_impl->failed.at(index) != 0; }
void LocomotionBatch3D::clearFailure(std::size_t index) { m_impl->failed.at(index) = 0; }

namespace {

// Puxada dinamica: uma tarefa por participante (os workers e o chamador), e
// cada uma pega o proximo mundo livre ate acabar. Faixas fixas deixavam
// threads paradas na cauda de cada passo (mundos com recomeco, preempcao). Cada
// mundo e independente: a ordem em que rodam nao muda nenhum bit.
template <typename Body>
void pullWorlds(TaskScheduler& scheduler, std::uint32_t participants, std::size_t count, Body body) {
    struct Context {
        std::atomic<std::size_t> next { 0 };
        std::size_t count = 0;
        Body* body = nullptr;
    } context;
    context.count = count;
    context.body = &body;
    scheduler.parallelFor(std::max<std::size_t>(1, std::min<std::size_t>(participants, count)), 1,
        [](std::size_t, std::size_t, void* raw) noexcept {
            auto* ctx = static_cast<Context*>(raw);
            for (std::size_t i = ctx->next.fetch_add(1, std::memory_order_relaxed); i < ctx->count;
                    i = ctx->next.fetch_add(1, std::memory_order_relaxed))
                (*ctx->body)(i);
        },
        &context);
}

} // namespace

void LocomotionBatch3D::step(int ticks) {
    Impl& impl = *m_impl;
    pullWorlds(*impl.scheduler, impl.workers, impl.worlds.size(), [&impl, ticks](std::size_t i) {
        if (impl.failed[i] != 0) return;
        try {
            for (int k = 0; k < ticks; ++k) impl.worlds[i]->step(impl.commands[i]);
        } catch (...) {
            impl.failed[i] = 1;
        }
    });
}

void LocomotionBatch3D::parallel(const std::function<void(std::size_t)>& task) {
    Impl& impl = *m_impl;
    pullWorlds(*impl.scheduler, impl.workers, impl.worlds.size(), [&impl, &task](std::size_t i) {
        if (impl.failed[i] != 0) return;
        try {
            task(i);
        } catch (...) {
            impl.failed[i] = 1;
        }
    });
}

} // namespace MatterEngine
