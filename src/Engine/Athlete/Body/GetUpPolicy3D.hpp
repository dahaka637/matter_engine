#pragma once

#include "Engine/Physics/PhysicsScene3D.hpp"
#include "Engine/Physics/RagdollProfile3D.hpp"

#include <algorithm>
#include <cmath>
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
    // A troca andando (01/11, ele: "trocar assim que ele comeca a ficar em pe" para correr ao alvo; o levantar em
    // movimento do treino dela): com o pedido de deslocamento acima de MovingRequestSpeed (m/s) e depois do chao (a pelve
    // abaixo de DownRootFraction da altura de pe), de pe por MovingHandoffSeconds (s), mesmo se mexendo.
    static constexpr float MovingRequestSpeed = 0.5f;
    static constexpr float MovingHandoffSeconds = 0.2f;
    static constexpr float DownRootFraction = 0.5f;
    // A queda iminente pelo tronco, alem do fim do episodio da rede da locomocao (01/11, ele: "o evitar queda precisa
    // saber operar em conjunto com a outra rede nos momentos de queda iminente"; medido com a rede do B21 em 32 mundos e
    // 38 quedas): inclinado alem de ImminentTiltDegrees e tombando mais rapido que ImminentTiltRate (graus/s) de uma
    // decisao para a outra - pega 87% das quedas, em media 0,31 s antes, com um disparo a toa a cada ~72 s por mundo
    // (correndo inclinado o tronco nao tomba). So a pelve baixa ou a mao no chao avisavam tarde demais para evitar.
    static constexpr float ImminentTiltDegrees = 25.0f;
    static constexpr float ImminentTiltRate = 90.0f;

    // A inclinacao do tronco (graus): o centro da pelve ao do peito alto em relacao a vertical.
    [[nodiscard]] static float trunkTilt(const RagdollProfile3D& profile, std::span<const PhysicsBodyState3D> links) {
        std::size_t chest = 0;
        for (std::size_t i = 0; i < profile.links.size(); ++i)
            if (profile.links[i].id == "UpperChest") chest = i;
        if (chest == 0 || chest >= links.size() || links.empty()) return 0.0f;
        const auto center = [&](std::size_t i) {
            return links[i].position + links[i].orientation.rotate(profile.links[i].collider.localPosition);
        };
        const Vec3 trunk = center(chest) - center(0);
        const float length = trunk.length();
        return length > 1.0e-6f ? std::acos(std::clamp(trunk.z / length, -1.0f, 1.0f)) * 57.2957795f : 0.0f;
    }

    virtual ~GetUpPolicy3D() = default;
    [[nodiscard]] virtual std::uint64_t iteration() const = 0;
    // body: os links e os contatos de agora; velocity: o pedido de deslocamento no chao (m/s, no mundo; zero, de pe no
    // lugar): ela levanta virando para la; groundHeight: a altura do chao embaixo dele (m).
    virtual void decide(const RagdollProfile3D& profile, const RagdollState3D& body,
        std::vector<RagdollDriveTarget3D>& drives, Vec3 velocity = {}, float groundHeight = 0.0f) const = 0;
};

} // namespace MatterEngine
