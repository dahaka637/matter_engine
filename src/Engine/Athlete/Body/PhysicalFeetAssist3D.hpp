#pragma once

#include "Engine/Athlete/Body/SupportWrench3D.hpp"
#include "Engine/Athlete/Sensing/BodyObservation3D.hpp"

#include <vector>

namespace MatterEngine {

// A ajuda nos pes do Fisico total (pedido do usuario, 10/10/2026 noite: "o
// modo fisico com uma pitada de ajuda auxiliar nos pes, tipo foi feito com o
// intermediario"). A forca e o torque que levam a pelve a referencia da
// locomocao fisica passam pelos pes apoiados, como no Corpo Ativo
// (SupportWrench3D): o pe empurra o chao, ate o atrito e a sola, e a soma das
// forcas e zero - quem leva o corpo e o chao. No ar e caido nada passa.
// Soma com a ajuda que ja existe na pelve sem dobrar: so cobre, no rumo do
// pedido, o que a pelve nao deu.
struct PhysicalFeetAssistSettings3D {
    // O servo da raiz (a pelve na referencia): frequencia (rad/s), razao de
    // amortecimento e aceleracao maxima pedida (m/s2) - os do Ativo.
    float rootFrequency = 7.0f;
    float rootDamping = 1.0f;
    float maximumRootAcceleration = 25.0f;
    // O servo da orientacao da pelve (o binario com os pes apoiados).
    float orientationFrequency = 10.0f;
    float orientationDamping = 1.0f;
    float bodyInertiaHorizontal = 11.0f;
    float bodyInertiaVertical = 2.5f;
    SupportWrenchSettings3D support;
};

struct PhysicalFeetAssistInput3D {
    // 0..1: a fracao da ajuda (o slider "Ajuda nos pes").
    float share = 0.0f;
    // A referencia da locomocao fisica (a raiz do guia).
    Vec3 referencePosition;
    Quaternion referenceOrientation;
    Vec3 referenceVelocity;
    Vec3 referenceAngularVelocity;
    // O que a ajuda da pelve ja aplica neste tick (forca e torque na pelve).
    Vec3 pelvisForce;
    Vec3 pelvisTorque;
};

struct PhysicalFeetAssistOutput3D {
    // Os esforcos a aplicar (applyRagdollControlLinkForce): na pelve e, com a
    // reacao, nos pes apoiados.
    std::vector<LinkWrench3D> wrenches;
    // Telemetria: o servo cheio, o que coube aos pes e o que o chao deu.
    Vec3 requestedForce;
    Vec3 routedForce;
    Vec3 deliveredForce;
    int supportCount = 0;
};

[[nodiscard]] PhysicalFeetAssistOutput3D physicalFeetAssist3D(const RagdollProfile3D& profile,
    const RagdollState3D& state, const BodyObservation3D& observation, const PhysicalFeetAssistInput3D& input,
    const PhysicalFeetAssistSettings3D& settings);

} // namespace MatterEngine
