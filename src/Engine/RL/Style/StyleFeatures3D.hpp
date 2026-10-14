#pragma once

#include "Engine/Physics/PhysicsTypes3D.hpp"
#include "Engine/RL/Body/RLBody3D.hpp"
#include "Engine/RL/Contract/VectorLayout3D.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>

namespace MatterEngine {

// O que o juiz de estilo ve de um instante (plano_claude_rl.md secao 6.3), no
// referencial do rumo da pelve, sem posicao no campo nem rumo absoluto: a
// altura e a inclinacao da pelve, a velocidade e o giro dela, os eixos (o
// angulo e a velocidade) e a cabeca, as maos e os pes em relacao a pelve.
//
// Tudo sai so do estado dos links (posicao, orientacao e velocidades), pela
// mesma conta no corpo e no acervo: os dois lados nunca diferem por convencao
// numerica, so pelo movimento. Os angulos sao os da fisica (a coordenada da
// junta) e a velocidade de um eixo e o giro do filho menos o do pai no
// referencial da junta (a conta do backend).
//
// Fora: os eixos das pontas dos pes (os clipes nao tem a junta dos dedos; o
// juiz ensinaria a nunca usa-la) e os travados (nao sao eixos).
//
// Proporcional ao corpo: as posicoes divididas pela escala da altura e as
// velocidades na escala de Froude (o tamanho nao vira atalho do juiz).
class StyleFeatures3D {
public:
    explicit StyleFeatures3D(const RLBody3D& body);

    [[nodiscard]] std::size_t size() const { return m_layout.size(); }
    [[nodiscard]] const RLVectorLayout3D& layout() const { return m_layout; }
    // Os eixos que o juiz ve (indices em RLBody3D::dofs).
    [[nodiscard]] std::span<const std::uint32_t> dofs() const { return m_dofs; }

    // links: o estado de cada link no mundo, com o chao em z = 0.
    // heightScale: a altura do jogador sobre a nominal (1,80 m).
    void write(std::span<const PhysicsBodyState3D> links, float heightScale, float* out) const;

private:
    RagdollProfile3D m_profile;
    std::vector<RLBodyDof3D> m_allDofs;
    std::vector<std::uint32_t> m_dofs;
    std::array<std::uint32_t, 5> m_keyBodies {};
    RLVectorLayout3D m_layout;
};

} // namespace MatterEngine
