#pragma once

#include "Engine/Modeling/Mesh/Mesh.hpp"

#include <cstdint>
#include <functional>
#include <optional>
#include <span>
#include <vector>

namespace MatterEngine::Modeling {

// Hierarquia de caixas sobre os triangulos: pares que se tocam, ponto mais
// proximo e raio, sem testar tudo contra tudo.
class TriangleBvh {
public:
    TriangleBvh(std::span<const Vec3> positions, std::span<const Triangle> triangles);

    struct Nearest {
        std::uint32_t triangle = 0;
        Vec3 point;
        float distance = 0.0f;
    };
    struct Hit {
        std::uint32_t triangle = 0;
        float distance = 0.0f;
    };

    [[nodiscard]] bool empty() const { return m_nodes.empty(); }
    // Pares (a < b) de triangulos com caixas que se cruzam (candidatos).
    void overlappingPairs(const std::function<void(std::uint32_t, std::uint32_t)>& visit) const;
    // O ponto da malha mais perto (ate maxDistance).
    [[nodiscard]] std::optional<Nearest> nearest(Vec3 point, float maxDistance) const;
    // O primeiro triangulo no raio.
    [[nodiscard]] std::optional<Hit> raycast(Vec3 origin, Vec3 direction, float maxDistance) const;
    // Quantos triangulos o raio cruza (dentro/fora de malhas fechadas).
    [[nodiscard]] int countCrossings(Vec3 origin, Vec3 direction, float maxDistance) const;

private:
    struct Node {
        Box3 box;
        std::uint32_t first = 0;  // folha: primeiro em m_order; interno: filho esquerdo
        std::uint32_t count = 0;  // 0 = interno
        std::uint32_t right = 0;
    };
    std::uint32_t build(std::uint32_t begin, std::uint32_t end);
    void pairs(std::uint32_t a, std::uint32_t b,
        const std::function<void(std::uint32_t, std::uint32_t)>& visit) const;
    [[nodiscard]] Box3 triangleBox(std::uint32_t triangle) const;

    std::span<const Vec3> m_positions;
    std::span<const Triangle> m_triangles;
    std::vector<Node> m_nodes;
    std::vector<std::uint32_t> m_order;
    std::vector<Vec3> m_centers;
};

// Raio contra triangulo (Moller-Trumbore): a distancia, se acerta.
[[nodiscard]] std::optional<float> rayTriangle(Vec3 origin, Vec3 direction, Vec3 a, Vec3 b, Vec3 c);

} // namespace MatterEngine::Modeling
