#pragma once

#include "Engine/Math/Vec3.hpp"
#include "Engine/Physics/PhysicsScene3D.hpp"

#include <span>

namespace MatterEngine {

// A rede da locomocao como parceira da rede do levantar no treino dela (01/11, ele: "ambas as redes aprendendo em
// conjunto em todos os treinos... o evitar queda precisa saber operar em conjunto com a outra rede nos momentos de queda
// iminente"): ela dirige o corpo de uma cena a partir do estado de agora - o atleta do zero no corpo, parado no lugar
// (o pedido de ficar, olhando para onde o corpo olha) -, com um empurrao opcional, ate o fim do tempo ou a queda
// iminente (o fim do episodio do treino dela: a pelve baixa, algo que nao os pes no chao ou o classico caido). A rede de
// verdade e a exportada pelo treino dela (Athlete/Training/LocomotionPartnerRunner3D); o rastreador so ve esta.
class LocomotionPartner3D {
public:
    // O empurrao (N), repartido como nos treinos: pelve 20%, peito 32%, peito alto 48%.
    struct Push {
        float startSeconds = 0.0f;
        float seconds = 0.0f;
        Vec3 force;
    };
    struct Result {
        // Quanto ela dirigiu (s), se acabou na queda iminente e o que faltava do empurrao (s).
        float seconds = 0.0f;
        bool fell = false;
        float pushLeft = 0.0f;
    };

    virtual ~LocomotionPartner3D() = default;
    // handoff: os alvos da rede do levantar na troca (os do classico saem deles em GetUpPolicy3D::BlendSeconds; vazio,
    // sem a passagem). velocity: o pedido de deslocamento no chao (m/s; zero, parada no lugar - 01/11, o levantar em
    // movimento: ela corre ate o alvo). Varias threads podem chamar ao mesmo tempo, cada uma com a sua cena.
    virtual Result drive(PhysicsScene3D& scene, RagdollHandle3D handle, std::span<const RagdollDriveTarget3D> handoff,
        float maximumSeconds, const Push* push = nullptr, Vec3 velocity = {}) const = 0;
};

} // namespace MatterEngine
