#include "Engine/Physics/Jolt/JoltLayers.hpp"

namespace MatterEngine {

JPH::BroadPhaseLayer JoltBroadPhaseLayerInterface::GetBroadPhaseLayer(
    JPH::ObjectLayer layer) const {
    if (layer == JoltObjectLayers::NonMoving) {
        return JoltBroadPhaseLayers::NonMoving;
    }
    // Props, links de ragdoll e o personagem se movem: compartilham a arvore
    // dinamica, que e reconstruida por passo.
    return JoltBroadPhaseLayers::Moving;
}

#if defined(JPH_EXTERNAL_PROFILE) || defined(JPH_PROFILE_ENABLED)
const char* JoltBroadPhaseLayerInterface::GetBroadPhaseLayerName(
    JPH::BroadPhaseLayer layer) const {
    if (layer == JoltBroadPhaseLayers::NonMoving) return "NonMoving";
    if (layer == JoltBroadPhaseLayers::Moving) return "Moving";
    return "Unknown";
}
#endif

bool JoltObjectVsBroadPhaseLayerFilter::ShouldCollide(
    JPH::ObjectLayer layer, JPH::BroadPhaseLayer broadPhaseLayer) const {
    // Estatico contra estatico nunca produz contato, e testar a arvore estatica
    // contra ela mesma e o desperdicio classico de broad phase.
    if (layer == JoltObjectLayers::NonMoving) {
        return broadPhaseLayer == JoltBroadPhaseLayers::Moving;
    }
    return true;
}

bool JoltObjectLayerPairFilter::ShouldCollide(JPH::ObjectLayer first,
    JPH::ObjectLayer second) const {
    if (first == JoltObjectLayers::NonMoving
        && second == JoltObjectLayers::NonMoving) {
        return false;
    }
    // Todo o resto colide. O personagem contra ragdoll tambem: a excecao de
    // ignoreRagdolls e por CHAMADA, num filtro passado ao ExtendedUpdate, e nao
    // uma regra global - o personagem controlado precisa ignorar a propria
    // articulacao, nao todas as outras.
    return true;
}

} // namespace MatterEngine
