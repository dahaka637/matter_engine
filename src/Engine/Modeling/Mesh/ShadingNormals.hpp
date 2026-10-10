#pragma once

#include "Engine/Modeling/Mesh/Mesh.hpp"

#include <cstdint>
#include <span>
#include <vector>

// As normais do sombreamento liso: so a luz muda (a malha, os vertices e o
// contorno ficam como sao). Em cada canto de triangulo, a media das faces em
// volta do vertice que se ligam a ele sem atravessar uma quina - a aresta
// entre duas faces com mais de "creaseDegrees" entre as normais (a ponta do
// nariz, a boca, a barra dobrada para dentro da roupa). A divisao em grupos
// sai das posicoes da construcao (a pose de ligacao) e e a mesma em qualquer
// pose (nenhuma quina acende e apaga na animacao); as normais saem das
// posicoes de cada pose, com o peso do angulo de cada canto. "facet" mistura
// com a normal da propria face: 0 liso, 1 facetado.
namespace MatterEngine::Modeling {

class ShadingNormals {
public:
    ShadingNormals() = default;
    ShadingNormals(const Mesh& mesh, float creaseDegrees);

    [[nodiscard]] bool empty() const { return m_cornerGroup.empty(); }
    [[nodiscard]] std::size_t triangleCount() const { return m_cornerGroup.size() / 3; }
    [[nodiscard]] std::uint32_t groupCount() const { return m_groupCount; }
    [[nodiscard]] float creaseDegrees() const { return m_crease; }
    // O grupo de cada canto (3 por triangulo): os cantos do mesmo grupo tem a
    // mesma normal lisa (o vertice do jogo e um por grupo).
    [[nodiscard]] std::uint32_t group(std::size_t triangle, int corner) const { return m_cornerGroup[triangle * 3 + static_cast<std::size_t>(corner)]; }

    // As normais dos cantos (3 por triangulo, na ordem dos triangulos) nas
    // posicoes dadas (as da malha ou as de uma pose).
    void compute(std::span<const Vec3> positions, const std::vector<Triangle>& triangles, float facet, std::vector<Vec3>& corners) const;

private:
    std::vector<std::uint32_t> m_cornerGroup;
    std::uint32_t m_groupCount = 0;
    float m_crease = 0.0f;
};

} // namespace MatterEngine::Modeling
