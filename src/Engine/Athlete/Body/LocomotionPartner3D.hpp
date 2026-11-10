#pragma once

#include "Engine/Math/Vec3.hpp"
#include "Engine/Physics/PhysicsScene3D.hpp"

#include <memory>
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

    // Tick a tick, como a professora da rl_final (03/11 18h27, ele: "gravar os movimentos que o b21 faz para se
    // locomover nas 8 direcoes... FORCAR absolutamente o nosso rl_final a fazer EXATOS mesmos movimentos, e aprender com
    // eles"): numa sessao aberta no corpo de agora (o atleta fisico do zero, com o pedido pelo analogico do jogo - a vista,
    // para onde o peito e o olhar apontam, em viewYaw, e o deslocamento no chao em velocity, m/s), cada tick de 1/120 s le
    // o corpo e ENCENA na cena os alvos dos motores dela, sem simular: quem treina simula com eles (ela no volante) ou os
    // sobrescreve com os da aluna (ela em sombra, so o rotulo). Varias threads podem abrir sessoes, cada uma na sua cena.
    struct Session {
        virtual ~Session() = default;
    };
    struct Tick {
        // Fisica e de pe: o tick vale como rotulo (senao a sessao acabou).
        bool valid = false;
        // A ajuda de fora do tick (N e N.m; tem de ser 0 - ela treinou a 0%).
        float assistNewtons = 0.0f;
    };
    virtual std::unique_ptr<Session> begin(PhysicsScene3D& scene, RagdollHandle3D handle, Vec3 velocity,
        float viewYaw) const = 0;
    virtual Tick tick(Session& session, PhysicsScene3D& scene, RagdollHandle3D handle) const = 0;
    // O pedido novo da sessao (03/11, o giro parado: a vista girando a cada decisao).
    virtual void command(Session& session, Vec3 velocity, float viewYaw) const = 0;
};

} // namespace MatterEngine
