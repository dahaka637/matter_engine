#pragma once

#include "Engine/Physics/PhysicsScene3D.hpp"
#include <algorithm>
#include <tuple>

namespace MatterEngine {

// Storage is allocated when creating the ragdoll, never in contact callbacks.
// The caller serializes accumulation if its backend invokes callbacks in parallel.
inline void accumulateRagdollInteraction3D(std::span<RagdollInteraction3D> slots,
    std::uint32_t link, RagdollContactMotion3D motion, Vec3 position, Vec3 normal,
    Vec3 relativeVelocity, float impulse, bool estimated, bool beforeSolve) noexcept {
    const std::size_t index = static_cast<std::size_t>(link) * 6
        + static_cast<std::size_t>(motion) * 2 + (normal.z >= 0.62f ? 1 : 0);
    if (index >= slots.size()) return;
    auto& out = slots[index];
    impulse = std::max(0.0f, impulse);
    // Deterministic representative selection even when callbacks race to the mutex.
    const auto key = std::tuple { impulse, position.x, position.y, position.z,
        normal.x, normal.y, normal.z };
    const auto previous = std::tuple { out.strongestNormalImpulseNewtonSeconds,
        out.positionWorld.x, out.positionWorld.y, out.positionWorld.z,
        out.normalWorld.x, out.normalWorld.y, out.normalWorld.z };
    if (!out.pointCount || key > previous) {
        out.positionWorld = position;
        out.normalWorld = normal;
        out.relativeVelocityWorld = relativeVelocity;
        out.strongestNormalImpulseNewtonSeconds = impulse;
    }
    out.linkIndex = link;
    out.otherMotion = motion;
    ++out.pointCount;
    out.normalImpulseNewtonSeconds += impulse;
    out.normalImpulseWorld += normal * impulse;
    out.impulseEstimated = estimated;
    out.velocityBeforeSolve = beforeSolve;
}

inline void publishRagdollInteractions3D(std::span<const RagdollInteraction3D> slots,
    std::vector<RagdollInteraction3D>& output) {
    output.clear();
    for (const auto& entry : slots) if (entry.pointCount) output.push_back(entry);
}
} // namespace MatterEngine
