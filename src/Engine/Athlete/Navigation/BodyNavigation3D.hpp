#pragma once

#include "Engine/Physics/PhysicsScene3D.hpp"
#include "Engine/Physics/RagdollProfile3D.hpp"

namespace MatterEngine {

struct LegacyAthleteState3D;

// (16/11, ele: "refazer completamente o sistema de capsulas... puxando o boneco, teleportando ele, arrastando") A navegacao de um corpo e
// onde a locomocao padrao poe a raiz dele: a capsula do jogador (na cena), a virtual de um boneco (de dados) e a ancora do fisico. Uma
// regra so, para todos os corpos:
//  - o padrao conduz (animado, voltando): ela anda pelo pedido e o corpo guiado vai junto;
//  - o corpo conduz (o cerebro do RL, caido, levantando, o comeco de toda volta ao padrao): ela fica NO corpo - a pelve sobre o chao de
//    verdade e a velocidade do centro de massa -, posta a cada tick, sem folga e sem atraso. A do jogador e posta, nunca movida: nao
//    empurra a bola nem os props.
// Toda volta ao padrao comeca, assim, com a raiz do padrao onde o corpo esta: nada e puxado de volta nem salta. (Medido na sonda: com a
// ancora do boneco parada onde ele caiu enquanto o cerebro dirigia, a volta o arrastava 4,8 m - os pes a 47 m/s, a pelve a 1800 m/s2.)
struct BodyNavigationPoint3D {
    // O chao sob a pelve.
    Vec3 feet;
    // A do centro de massa, no plano.
    Vec3 velocity;
};

[[nodiscard]] BodyNavigationPoint3D bodyNavigationPoint3D(const PhysicsScene3D& scene, const RagdollProfile3D& profile,
    const RagdollState3D& body);

// A navegacao do atleta no ponto: a capsula da cena (o controlado) e a virtual; a ancora do fisico recomeca dele (no proximo tick do
// fisico, na pelve). avatar: a capsula do corpo (a virtual fica com o centro a meia altura de pe, como a do jogador).
void placeBodyNavigation3D(PhysicsScene3D& scene, const CharacterMotorSettings3D& avatar, bool controlled,
    LegacyAthleteState3D& legacy, const BodyNavigationPoint3D& point);

} // namespace MatterEngine
