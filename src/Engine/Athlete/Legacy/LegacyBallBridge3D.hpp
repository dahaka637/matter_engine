#pragma once

#include "Engine/Athlete/Actions/BallHandling3D.hpp"
#include "Engine/Athlete/Dribble/DribbleTypes3D.hpp"
#include "Engine/Character/BallGesturePose3D.hpp"
#include "Engine/Physics/PhysicsScene3D.hpp"

#include <cstdint>

namespace MatterEngine {

// A ponte do manejo novo (Engine/Athlete/Actions) com o modo padrao do legado:
// os alvos do corpo viram o gesto que o DefaultLocomotion3D aplica.
[[nodiscard]] BallGesturePose3D legacyBallGesture3D(const BallBodyTargets3D& targets);
// Plano v5: as duas pernas (o toque em cada balanco) e o corpo do manejo
// refeito.
[[nodiscard]] BallGesturePose3D legacyDribbleGesture3D(const DribbleOutput3D& output);

// A chave de uma bola (o handle empacotado: indice e geracao).
[[nodiscard]] std::uint64_t athleteBallKey3D(InflatedBallHandle3D handle);

} // namespace MatterEngine
