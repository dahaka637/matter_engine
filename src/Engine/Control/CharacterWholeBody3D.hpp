#pragma once

#include "Engine/Physics/PhysicsScene3D.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>

namespace MatterEngine {

// Esforco que o ambiente (o chao) deve exercer sobre um link, no mundo:
// forca aplicada num ponto (o centro de pressao do contato) e um torque puro
// (o que o centro de pressao fora do centro da sola produz, a torcao).
struct ContactWrench3D {
    std::uint32_t linkIndex = RagdollDynamics3D::InvalidIndex;
    Vec3 pointWorld;
    Vec3 forceWorld;
    Vec3 torqueWorld;
};

// Controle do corpo inteiro pelas juntas (etapa E4 do plano Astra): converte
// o esforco de contato desejado nos apoios em torque de cada junta da cadeia
// entre o apoio e a raiz - trabalho virtual (tau = -J^T f), pela mesma
// passada reversa do compensador de gravidade, so com as cargas de contato.
//
// A raiz (pelve) nao e atuada: o que a sustenta e o que a perna transmite pela
// junta do quadril. A gravidade dos membros continua com o compensador do
// backend (ArticulationGravity3D) - aqui entra so o termo do contato, para
// nao contar o peso duas vezes.
//
// Convencao dos torques: por link, nos tres eixos do frame articular da junta
// de entrada (X Twist, Y Swing1, Z Swing2) - o que o backend aplica como
// torque de antecipacao, +no filho, -no pai.
class CharacterWholeBody3D final {
public:
    void prepare(const RagdollProfile3D& profile);
    [[nodiscard]] bool ready() const { return !m_links.empty(); }

    // Soma em `torques` (um por link, mesmo indice do perfil) o torque que
    // cada junta precisa fazer para transmitir os esforcos de contato ate a
    // raiz, na pose medida. Nao aloca se `torques` ja tiver o tamanho certo.
    void addContactTorques(const RagdollState3D& state,
        std::span<const ContactWrench3D> wrenches,
        std::vector<std::array<float, 3>>& torques) const;

private:
    struct LinkConstants {
        int parentIndex = -1;
        Vec3 anchorLocal;
        Quaternion jointFrameLocal;
        std::array<bool, 3> axisEnabled {};
    };
    std::vector<LinkConstants> m_links;
};

} // namespace MatterEngine
