#pragma once

#include "Engine/Modeling/Mesh/Mesh.hpp"

#include <cstdint>
#include <span>
#include <unordered_map>
#include <vector>

// Consultas de vizinhanca da malha (sem mudar nada): quem usa cada aresta,
// as voltas de borda e os pedacos ligados. A base das costuras e da
// checagem de conexao.
namespace MatterEngine::Modeling {

[[nodiscard]] std::uint64_t meshEdgeKey(std::uint32_t a, std::uint32_t b);

// As arestas de um conjunto de triangulos (vazio = a malha toda).
class MeshEdges {
public:
    struct Use {
        // Os triangulos que usam a aresta (os dois primeiros) e quantos sao.
        std::array<std::uint32_t, 2> triangles { UINT32_MAX, UINT32_MAX };
        int count = 0;
    };

    explicit MeshEdges(const Mesh& mesh, std::span<const std::uint32_t> triangles = {});

    [[nodiscard]] const Use* find(std::uint32_t a, std::uint32_t b) const;
    [[nodiscard]] const std::unordered_map<std::uint64_t, Use>& uses() const { return m_uses; }
    // O outro triangulo da aresta (ou UINT32_MAX).
    [[nodiscard]] std::uint32_t across(std::uint32_t triangle, std::uint32_t a, std::uint32_t b) const;

private:
    std::unordered_map<std::uint64_t, Use> m_uses;
};

// As voltas de borda do conjunto (vazio = a malha toda), no sentido dos
// triangulos: andando nela, a superficie fica a esquerda (vista de fora).
// Num vertice com duas bordas, cada volta segue a sua.
[[nodiscard]] std::vector<std::vector<std::uint32_t>> boundaryLoops(const Mesh& mesh,
    std::span<const std::uint32_t> triangles = {});

// Pedacos ligados por arestas: o pedaco de cada triangulo (0..count-1).
[[nodiscard]] std::vector<int> edgeComponents(const Mesh& mesh, int* count = nullptr);

// Vertices onde a superficie se toca so pelo vertice (leques separados).
[[nodiscard]] std::vector<std::uint32_t> pinchedVertices(const Mesh& mesh);

} // namespace MatterEngine::Modeling
