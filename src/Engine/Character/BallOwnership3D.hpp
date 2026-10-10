#pragma once

#include "Engine/Math/Vec3.hpp"
#include "Engine/Physics/InflatedBall3D.hpp"

#include <cstdint>

namespace MatterEngine {

// Dono de uma bola (pedido do usuario, 05/10 noite: facilitar o manejo). O
// boneco que interagiu com ela (pedido de toque ou chute, manejo em
// andamento) ou que ela tocou passa a ser o dono, e o dono nao colide com ela
// (PhysicsScene3D::setInflatedBallOwner: o quadril e as pernas a atravessam;
// tronco, cabeca e bracos so a repelem se ela entrar neles) - manejando, ele
// nao a chuta nem a mexe sem querer. Um boneco por vez: outro boneco tocando
// a bola ou interagindo com ela vira o dono, e o anterior volta a colidir.
// Ficando longe do dono - pelo menos releaseDistanceMeters por
// releaseSeconds seguidos (pedido do usuario, 05/10 noite; 3 s: tocando a
// bola longe para a frente para busca-la, ela nao volta a colidir com ele no
// caminho) -, ela volta a colidir com todos.
struct BallOwnershipSettings3D {
    float releaseDistanceMeters = 5.0f;
    float releaseSeconds = 3.0f;
};

class BallOwnership3D {
public:
    // Um boneco interagiu com a bola: passa a ser o dono.
    void interact(std::uint64_t entityId);
    // Por tick, com o estado da bola do ultimo passo: um boneco que nao e o
    // dono tocando-a passa a ser o dono (o que mais a empurra); senao, o dono
    // longe dela o tempo todo - ou sumido (ownerPelvis nulo) - a solta.
    void update(const InflatedBallState3D& ball, const Vec3* ownerPelvis, float deltaTime);
    void release() { m_owner = 0; m_farSeconds = 0.0f; }
    [[nodiscard]] std::uint64_t owner() const { return m_owner; }
    [[nodiscard]] BallOwnershipSettings3D& settings() { return m_settings; }

private:
    BallOwnershipSettings3D m_settings;
    std::uint64_t m_owner = 0;
    // Ha quanto tempo a bola esta longe do dono (s).
    float m_farSeconds = 0.0f;
};

} // namespace MatterEngine
