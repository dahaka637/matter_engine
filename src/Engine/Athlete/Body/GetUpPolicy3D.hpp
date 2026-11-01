#pragma once

#include "Engine/Physics/PhysicsScene3D.hpp"
#include "Engine/Physics/RagdollProfile3D.hpp"

#include <cstdint>
#include <span>
#include <vector>

namespace MatterEngine {

// A rede do levantar no jogo, como o atleta a ve (01/11, ele: "o levantar agora ocorra com esse RL do levantar"): os
// alvos de junta para o corpo de agora, a cada TicksPerDecision passos de 1/120 s, aplicados sem a compensacao da
// gravidade e sem forcas de fora (como no treino). A rede de verdade e a do treino levantar_v2 sobre o contrato do
// levantar (Tracking/TrackingGetUpPolicy3D): ela mora no rastreador, que ja depende do atleta.
class GetUpPolicy3D {
public:
    static constexpr int TicksPerDecision = 4;

    virtual ~GetUpPolicy3D() = default;
    [[nodiscard]] virtual std::uint64_t iteration() const = 0;
    virtual void decide(const RagdollProfile3D& profile, std::span<const PhysicsBodyState3D> links,
        std::vector<RagdollDriveTarget3D>& drives) const = 0;
};

} // namespace MatterEngine
