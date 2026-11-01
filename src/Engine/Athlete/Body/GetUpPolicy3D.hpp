#pragma once

#include "Engine/Physics/PhysicsScene3D.hpp"
#include "Engine/Physics/RagdollProfile3D.hpp"

#include <cstdint>
#include <span>
#include <vector>

namespace MatterEngine {

// A rede do levantar como o atleta a ve (01/11, a interligacao das redes nas quedas do treino da locomocao): os alvos de
// junta para o corpo de agora, a cada TicksPerDecision passos de 1/120 s, aplicados sem a compensacao da gravidade e
// sem forcas de fora (como no treino dela). A rede de verdade e a do treino levantar_v2 sobre o contrato do levantar
// (Tracking/TrackingGetUpPolicy3D): ela mora no rastreador, que ja depende do atleta.
class GetUpPolicy3D {
public:
    static constexpr int TicksPerDecision = 4;
    // A troca para o classico: de pe e firme por FirmSeconds (s; o mesmo do fim do episodio no treino dela) e os alvos
    // dele saindo dos dela em BlendSeconds (s).
    static constexpr float FirmSeconds = 0.5f;
    static constexpr float BlendSeconds = 0.3f;

    virtual ~GetUpPolicy3D() = default;
    [[nodiscard]] virtual std::uint64_t iteration() const = 0;
    virtual void decide(const RagdollProfile3D& profile, std::span<const PhysicsBodyState3D> links,
        std::vector<RagdollDriveTarget3D>& drives) const = 0;
};

} // namespace MatterEngine
