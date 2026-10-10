#pragma once

#include "Engine/Athlete/Sensing/BodyObservation3D.hpp"

#include <cstdint>
#include <vector>

namespace MatterEngine {

// Forca e torque num link, no mundo (o torque em volta do centro de massa
// dele). E o que vai para applyRagdollControlLinkForce.
struct LinkWrench3D {
    std::uint32_t link = 0;
    Vec3 force;
    Vec3 torque;
};

struct SupportWrenchSettings3D {
    // Atrito da sola com o chao (a grama/o concreto do laboratorio).
    float friction = 0.9f;
    // O quanto o centro de pressao pode sair do centro da sola (m): para a
    // frente/tras e para os lados; e o raio da torcao.
    float soleHalfLength = 0.12f;
    float soleHalfWidth = 0.045f;
    float torsionRadius = 0.04f;
    // Carga minima de um pe apoiado (fracao do peso) quando o contato medido
    // pisca: um pe que toca sustenta pelo menos isto na conta.
    float minimumLoadShare = 0.15f;
};

// O que o controle quer do corpo: a forca no centro de massa de um link (a
// pelve) e o torque nele, para levar o corpo ao plano.
struct SupportWrenchRequest3D {
    std::uint32_t bodyLink = 0;
    Vec3 bodyPoint;
    Vec3 force;
    Vec3 torque;
    float weightNewtons = 0.0f;
};

struct SupportWrenchResult3D {
    // Os esforcos a aplicar: no link do corpo e, com a reacao igual e
    // contraria (mais o binario que fecha o momento), nos pes apoiados. A
    // soma e zero: quem empurra o corpo e o chao, pelo contato.
    std::vector<LinkWrench3D> wrenches;
    Vec3 deliveredForce;
    Vec3 deliveredTorque;
    // O que o chao nao consegue dar (atrito, sola, pe no ar): fica de fora.
    Vec3 droppedForce;
    Vec3 droppedTorque;
};

// Reparte o pedido entre os pes apoiados pela carga de cada um e corta o que
// passa do atrito (a forca de lado ate mu vezes a normal), do chao (nao puxa)
// e da sola (o centro de pressao dentro dela). Sem pe apoiado, nada passa: no
// ar e caido a ajuda e zero.
[[nodiscard]] SupportWrenchResult3D routeSupportWrench3D(const SupportWrenchRequest3D& request,
    const BodyObservation3D& observation, const std::vector<Vec3>& linkCenters,
    const SupportWrenchSettings3D& settings);

} // namespace MatterEngine
