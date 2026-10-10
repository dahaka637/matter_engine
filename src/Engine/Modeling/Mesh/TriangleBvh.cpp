#include "Engine/Modeling/Mesh/TriangleBvh.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace MatterEngine::Modeling {

namespace {

constexpr std::uint32_t LeafSize = 4;

float axisValue(Vec3 v, int axis) {
    return axis == 0 ? v.x : axis == 1 ? v.y : v.z;
}

bool rayBox(Vec3 origin, Vec3 inverse, const Box3& box, float maxDistance) {
    float tMin = 0.0f, tMax = maxDistance;
    for (int axis = 0; axis < 3; ++axis) {
        const float o = axisValue(origin, axis), inv = axisValue(inverse, axis);
        float t0 = (axisValue(box.minimum, axis) - o) * inv;
        float t1 = (axisValue(box.maximum, axis) - o) * inv;
        if (t0 > t1) std::swap(t0, t1);
        tMin = std::max(tMin, t0);
        tMax = std::min(tMax, t1);
        if (tMax < tMin) return false;
    }
    return true;
}

float boxDistance(const Box3& box, Vec3 p) {
    const Vec3 clamped { std::clamp(p.x, box.minimum.x, box.maximum.x), std::clamp(p.y, box.minimum.y, box.maximum.y),
        std::clamp(p.z, box.minimum.z, box.maximum.z) };
    return (clamped - p).length();
}

} // namespace

std::optional<float> rayTriangle(Vec3 origin, Vec3 direction, Vec3 a, Vec3 b, Vec3 c) {
    const Vec3 edge1 = b - a, edge2 = c - a;
    const Vec3 p = cross(direction, edge2);
    const float determinant = dot(edge1, p);
    if (std::abs(determinant) < 1e-12f) return std::nullopt;
    const float inverse = 1.0f / determinant;
    const Vec3 s = origin - a;
    const float u = dot(s, p) * inverse;
    if (u < 0.0f || u > 1.0f) return std::nullopt;
    const Vec3 q = cross(s, edge1);
    const float v = dot(direction, q) * inverse;
    if (v < 0.0f || u + v > 1.0f) return std::nullopt;
    const float t = dot(edge2, q) * inverse;
    if (t <= 1e-7f) return std::nullopt;
    return t;
}

TriangleBvh::TriangleBvh(std::span<const Vec3> positions, std::span<const Triangle> triangles)
    : m_positions(positions), m_triangles(triangles) {
    if (triangles.empty()) return;
    m_order.resize(triangles.size());
    m_centers.resize(triangles.size());
    for (std::uint32_t i = 0; i < triangles.size(); ++i) {
        m_order[i] = i;
        const Triangle& t = triangles[i];
        m_centers[i] = (positions[t[0]] + positions[t[1]] + positions[t[2]]) / 3.0f;
    }
    m_nodes.reserve(triangles.size() * 2 / LeafSize + 2);
    build(0, static_cast<std::uint32_t>(triangles.size()));
}

Box3 TriangleBvh::triangleBox(std::uint32_t triangle) const {
    Box3 box;
    for (const std::uint32_t vertex : m_triangles[triangle]) box.include(m_positions[vertex]);
    return box;
}

std::uint32_t TriangleBvh::build(std::uint32_t begin, std::uint32_t end) {
    const std::uint32_t index = static_cast<std::uint32_t>(m_nodes.size());
    m_nodes.emplace_back();
    Box3 box, centers;
    for (std::uint32_t i = begin; i < end; ++i) {
        box.include(triangleBox(m_order[i]));
        centers.include(m_centers[m_order[i]]);
    }
    m_nodes[index].box = box;
    if (end - begin <= LeafSize) {
        m_nodes[index].first = begin;
        m_nodes[index].count = end - begin;
        return index;
    }
    const Vec3 extent = centers.size();
    const int axis = extent.x >= extent.y && extent.x >= extent.z ? 0 : extent.y >= extent.z ? 1 : 2;
    const std::uint32_t middle = begin + (end - begin) / 2;
    std::nth_element(m_order.begin() + begin, m_order.begin() + middle, m_order.begin() + end,
        [&](std::uint32_t a, std::uint32_t b) { return axisValue(m_centers[a], axis) < axisValue(m_centers[b], axis); });
    const std::uint32_t left = build(begin, middle);
    const std::uint32_t right = build(middle, end);
    m_nodes[index].first = left;
    m_nodes[index].right = right;
    m_nodes[index].count = 0;
    return index;
}

void TriangleBvh::overlappingPairs(const std::function<void(std::uint32_t, std::uint32_t)>& visit) const {
    if (!m_nodes.empty()) pairs(0, 0, visit);
}

void TriangleBvh::pairs(std::uint32_t a, std::uint32_t b,
    const std::function<void(std::uint32_t, std::uint32_t)>& visit) const {
    const Node& nodeA = m_nodes[a];
    const Node& nodeB = m_nodes[b];
    if (!nodeA.box.overlaps(nodeB.box)) return;
    const bool leafA = nodeA.count > 0, leafB = nodeB.count > 0;
    if (leafA && leafB) {
        for (std::uint32_t i = 0; i < nodeA.count; ++i) {
            const std::uint32_t first = m_order[nodeA.first + i];
            const Box3 firstBox = triangleBox(first);
            for (std::uint32_t j = (a == b ? i + 1 : 0); j < nodeB.count; ++j) {
                const std::uint32_t second = m_order[nodeB.first + j];
                if (first == second || !firstBox.overlaps(triangleBox(second))) continue;
                visit(std::min(first, second), std::max(first, second));
            }
        }
        return;
    }
    if (a == b) {
        pairs(nodeA.first, nodeA.first, visit);
        pairs(nodeA.right, nodeA.right, visit);
        pairs(nodeA.first, nodeA.right, visit);
        return;
    }
    if (leafA || (!leafB && nodeB.box.radius() > nodeA.box.radius())) {
        pairs(a, nodeB.first, visit);
        pairs(a, nodeB.right, visit);
    } else {
        pairs(nodeA.first, b, visit);
        pairs(nodeA.right, b, visit);
    }
}

std::optional<TriangleBvh::Nearest> TriangleBvh::nearest(Vec3 point, float maxDistance) const {
    if (m_nodes.empty()) return std::nullopt;
    std::optional<Nearest> best;
    float bestDistance = maxDistance;
    std::vector<std::uint32_t> stack { 0 };
    while (!stack.empty()) {
        const std::uint32_t index = stack.back();
        stack.pop_back();
        const Node& node = m_nodes[index];
        if (boxDistance(node.box, point) > bestDistance) continue;
        if (node.count > 0) {
            for (std::uint32_t i = 0; i < node.count; ++i) {
                const std::uint32_t triangle = m_order[node.first + i];
                const Triangle& t = m_triangles[triangle];
                const Vec3 closest = closestPointOnTriangle(point, m_positions[t[0]], m_positions[t[1]],
                    m_positions[t[2]]);
                const float d = (closest - point).length();
                if (d <= bestDistance) {
                    bestDistance = d;
                    best = Nearest { triangle, closest, d };
                }
            }
        } else {
            const float left = boxDistance(m_nodes[node.first].box, point);
            const float right = boxDistance(m_nodes[node.right].box, point);
            // O mais perto por ultimo na pilha: visitado primeiro.
            if (left < right) {
                stack.push_back(node.right);
                stack.push_back(node.first);
            } else {
                stack.push_back(node.first);
                stack.push_back(node.right);
            }
        }
    }
    return best;
}

std::optional<TriangleBvh::Hit> TriangleBvh::raycast(Vec3 origin, Vec3 direction, float maxDistance) const {
    if (m_nodes.empty()) return std::nullopt;
    const Vec3 inverse { 1.0f / (std::abs(direction.x) > 1e-12f ? direction.x : 1e-12f),
        1.0f / (std::abs(direction.y) > 1e-12f ? direction.y : 1e-12f),
        1.0f / (std::abs(direction.z) > 1e-12f ? direction.z : 1e-12f) };
    std::optional<Hit> best;
    float bestDistance = maxDistance;
    std::vector<std::uint32_t> stack { 0 };
    while (!stack.empty()) {
        const Node& node = m_nodes[stack.back()];
        stack.pop_back();
        if (!rayBox(origin, inverse, node.box, bestDistance)) continue;
        if (node.count > 0) {
            for (std::uint32_t i = 0; i < node.count; ++i) {
                const std::uint32_t triangle = m_order[node.first + i];
                const Triangle& t = m_triangles[triangle];
                const auto hit = rayTriangle(origin, direction, m_positions[t[0]], m_positions[t[1]],
                    m_positions[t[2]]);
                if (hit && *hit < bestDistance) {
                    bestDistance = *hit;
                    best = Hit { triangle, *hit };
                }
            }
        } else {
            stack.push_back(node.first);
            stack.push_back(node.right);
        }
    }
    return best;
}

int TriangleBvh::countCrossings(Vec3 origin, Vec3 direction, float maxDistance) const {
    if (m_nodes.empty()) return 0;
    const Vec3 inverse { 1.0f / (std::abs(direction.x) > 1e-12f ? direction.x : 1e-12f),
        1.0f / (std::abs(direction.y) > 1e-12f ? direction.y : 1e-12f),
        1.0f / (std::abs(direction.z) > 1e-12f ? direction.z : 1e-12f) };
    int crossings = 0;
    std::vector<std::uint32_t> stack { 0 };
    while (!stack.empty()) {
        const Node& node = m_nodes[stack.back()];
        stack.pop_back();
        if (!rayBox(origin, inverse, node.box, maxDistance)) continue;
        if (node.count > 0) {
            for (std::uint32_t i = 0; i < node.count; ++i) {
                const Triangle& t = m_triangles[m_order[node.first + i]];
                const auto hit = rayTriangle(origin, direction, m_positions[t[0]], m_positions[t[1]],
                    m_positions[t[2]]);
                if (hit && *hit < maxDistance) ++crossings;
            }
        } else {
            stack.push_back(node.first);
            stack.push_back(node.right);
        }
    }
    return crossings;
}

} // namespace MatterEngine::Modeling
