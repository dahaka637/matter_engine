#pragma once

#include "Engine/Animation/RagdollCharacter3D.hpp"
#include "Engine/Math/Quaternion.hpp"
#include "Engine/Math/Vec3.hpp"

#include <array>
#include <cstdint>
#include <span>
#include <vector>

namespace MatterEngine {

// A pose dos ossos visuais (07/10, pedidos do usuario):
//  - os dedos do pe ("+1 osso no pe para, quando ele pisar no chao, dar aquela
//    inclinacao de amortecimento"): a base dos dedos dobra para a ponta ficar
//    no chao quando o pe inclina com ela apoiada (o calcanhar subindo) - na
//    hora; soltando, volta suave;
//  - os dedos da mao ("funcionais, nao fixos - o goleiro e as palmadas, no
//    futuro"): a pose pedida (relaxada, aberta, fechada) com transicao; a
//    automatica abre a mao com o braco esticado (o equilibrio, a queda) e
//    relaxa com ele dobrado.
// So o visual: a fisica fica como esta (a capsula da mao ja cobre a mao
// aberta; o pe e o bloco dele).
enum class RagdollHandPose3D : std::uint8_t { Automatic, Relaxed, Open, Fist };

struct RagdollVisualPoseInput3D {
    // Os links no mundo (na ordem do perfil fisico).
    std::span<const Vec3> linkPositions;
    std::span<const Quaternion> linkOrientations;
    // A altura do chao embaixo de cada pe (esquerdo, direito).
    std::array<float, 2> groundHeight {};
    std::array<RagdollHandPose3D, 2> handPose { RagdollHandPose3D::Automatic, RagdollHandPose3D::Automatic };
    bool toesFollowGround = true;
    float deltaTime = 1.0f / 60.0f;
};

class RagdollVisualPose3D {
public:
    void reset();
    // As rotacoes dos ossos visuais (na ordem do personagem) nesta pose.
    const std::vector<Quaternion>& update(const RagdollCharacter3D& character, const RagdollVisualPoseInput3D& input);
    [[nodiscard]] const std::vector<Quaternion>& rotations() const { return m_rotations; }
    // Telemetria: a mao (0..1 aberta e fechada) e a dobra dos dedos do pe (rad).
    [[nodiscard]] float handOpen(int side) const { return m_open[side == 0 ? 0 : 1]; }
    [[nodiscard]] float handFist(int side) const { return m_fist[side == 0 ? 0 : 1]; }
    [[nodiscard]] float toeBend(int side) const { return m_toeBend[side == 0 ? 0 : 1]; }

private:
    std::vector<Quaternion> m_rotations;
    std::array<float, 2> m_toeBend {};
    std::array<float, 2> m_open {};
    std::array<float, 2> m_fist {};
    bool m_started = false;
};

} // namespace MatterEngine
