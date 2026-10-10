#pragma once

#include "Engine/Athlete/AthleteRuntime3D.hpp"

#include <cstdint>
#include <span>
#include <utility>

namespace MatterEngine {

// Um tick do caminho de hoje (legado) para um atleta: monta o comando da
// locomocao, atualiza as ajudas por impacto, decide o modo do corpo (padrao,
// fisico, volta), roda o manejo de bola do controlado e envia os alvos para a
// fisica. Porte fiel de WorkbenchApp::updateActiveRagdolls (10/10/2026).
// control.ballRequest e consumido (vira None) quando o controlado passa.
void stepLegacyAthlete3D(PhysicsScene3D& scene, const AthleteRuntimeAssets3D& assets,
    const AthleteRuntimeSettings3D& settings, AthleteControlInput3D& control, AthleteSlot3D& slot,
    std::span<AthleteBall3D> balls, std::span<const std::pair<std::uint64_t, Vec3>> tacklerLegs, float deltaTime,
    AthleteRuntimeOutput3D& output);

} // namespace MatterEngine
