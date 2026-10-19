#pragma once

#include "Engine/Core/TaskScheduler.hpp"
#include "Engine/Physics/PhysicsScene3D.hpp"

#include <algorithm>
#include <atomic>
#include <cstddef>
#include <cstdint>

namespace MatterEngine {

// Os musculos do rastreador: o modo ativo do backend multiplica a rigidez por 5
// e o amortecimento por 2 (JoltRagdollDrives3D); estas escalas devolvem os do
// perfil, como no corpo v2. O teto de forca e o do perfil (os de um atleta).
inline constexpr float TrackingStiffnessScale = 0.2f;
inline constexpr float TrackingDampingScale = 0.5f;

// A cena de um mundo dos lotes do rastreador: inline, uma thread, com as
// reservas de um ragdoll, do chao e dos projeteis.
[[nodiscard]] PhysicsSceneSettings3D trackingSceneSettings3D();
// O chao: uma laje de concreto com o topo em z = 0.
void createTrackingGround3D(PhysicsScene3D& scene, float halfExtentMeters);

// Puxada dinamica: cada participante pega o proximo mundo livre; cada mundo e
// independente, a ordem nao muda nenhum bit.
template <typename Body>
void trackingPullWorlds3D(TaskScheduler& scheduler, std::uint32_t participants, std::size_t count, Body body) {
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

} // namespace MatterEngine
