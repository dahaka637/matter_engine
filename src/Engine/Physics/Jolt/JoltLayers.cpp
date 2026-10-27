#include "Engine/Physics/Jolt/JoltLayers.hpp"

namespace MatterEngine {

JPH::BroadPhaseLayer JoltBroadPhaseLayerInterface::GetBroadPhaseLayer(
    JPH::ObjectLayer layer) const {
    if (layer == JoltObjectLayers::NonMoving || layer == JoltObjectLayers::PlayerBarrier
        || layer == JoltObjectLayers::CapsuleBarrier) {
        return JoltBroadPhaseLayers::NonMoving;
    }
    // Os links de ragdoll numa arvore so deles (as sondas de chao nao os veem).
    if (layer == JoltObjectLayers::Ragdoll) return JoltBroadPhaseLayers::Ragdoll;
    // Props e o personagem se movem: a arvore dinamica, reconstruida por passo.
    return JoltBroadPhaseLayers::Moving;
}

#if defined(JPH_EXTERNAL_PROFILE) || defined(JPH_PROFILE_ENABLED)
const char* JoltBroadPhaseLayerInterface::GetBroadPhaseLayerName(
    JPH::BroadPhaseLayer layer) const {
    if (layer == JoltBroadPhaseLayers::NonMoving) return "NonMoving";
    if (layer == JoltBroadPhaseLayers::Moving) return "Moving";
    if (layer == JoltBroadPhaseLayers::Ragdoll) return "Ragdoll";
    return "Unknown";
}
#endif

bool JoltObjectVsBroadPhaseLayerFilter::ShouldCollide(
    JPH::ObjectLayer layer, JPH::BroadPhaseLayer broadPhaseLayer) const {
    // Estatico contra estatico nunca produz contato, e testar a arvore estatica
    // contra ela mesma e o desperdicio classico de broad phase.
    if (layer == JoltObjectLayers::NonMoving) {
        return broadPhaseLayer == JoltBroadPhaseLayers::Moving || broadPhaseLayer == JoltBroadPhaseLayers::Ragdoll;
    }
    if (layer == JoltObjectLayers::PlayerBarrier) return broadPhaseLayer == JoltBroadPhaseLayers::Ragdoll;
    if (layer == JoltObjectLayers::CapsuleBarrier) return false;
    return true;
}

bool JoltObjectLayerPairFilter::ShouldCollide(JPH::ObjectLayer first,
    JPH::ObjectLayer second) const {
    if (first == JoltObjectLayers::NonMoving
        && second == JoltObjectLayers::NonMoving) {
        return false;
    }
    // A barreira dos jogadores so com os links, a da capsula com nada (a
    // capsula as ve pelo filtro da varredura dela).
    if (first == JoltObjectLayers::CapsuleBarrier || second == JoltObjectLayers::CapsuleBarrier) return false;
    if (first == JoltObjectLayers::PlayerBarrier || second == JoltObjectLayers::PlayerBarrier)
        return first == JoltObjectLayers::Ragdoll || second == JoltObjectLayers::Ragdoll;
    // Todo o resto colide. O personagem contra ragdoll tambem: a excecao de
    // ignoreRagdolls e por CHAMADA, num filtro passado ao ExtendedUpdate, e nao
    // uma regra global - o personagem controlado precisa ignorar a propria
    // articulacao, nao todas as outras.
    return true;
}

} // namespace MatterEngine
