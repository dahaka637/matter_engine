#include "Engine/Modeling/Mesh/MeshTopology.hpp"

#include <algorithm>
#include <numeric>
#include <unordered_set>

namespace MatterEngine::Modeling {

namespace {

struct UnionFind {
    std::vector<std::uint32_t> parent;
    explicit UnionFind(std::size_t count) : parent(count) { std::iota(parent.begin(), parent.end(), 0u); }
    std::uint32_t find(std::uint32_t x) {
        while (parent[x] != x) {
            parent[x] = parent[parent[x]];
            x = parent[x];
        }
        return x;
    }
    void unite(std::uint32_t a, std::uint32_t b) { parent[find(a)] = find(b); }
};

std::uint64_t directedKey(std::uint32_t from, std::uint32_t to) {
    return (static_cast<std::uint64_t>(from) << 32) | to;
}

// O vertice do triangulo depois de "vertex" (no sentido dele).
std::uint32_t nextIn(const Triangle& tri, std::uint32_t vertex) {
    for (std::size_t i = 0; i < 3; ++i)
        if (tri[i] == vertex) return tri[(i + 1) % 3];
    return UINT32_MAX;
}

} // namespace

std::uint64_t meshEdgeKey(std::uint32_t a, std::uint32_t b) {
    return (static_cast<std::uint64_t>(std::min(a, b)) << 32) | std::max(a, b);
}

MeshEdges::MeshEdges(const Mesh& mesh, std::span<const std::uint32_t> triangles) {
    const auto visit = [&](std::uint32_t t) {
        const Triangle& tri = mesh.triangles[t];
        for (std::size_t i = 0; i < 3; ++i) {
            Use& use = m_uses[meshEdgeKey(tri[i], tri[(i + 1) % 3])];
            if (use.count < 2) use.triangles[static_cast<std::size_t>(use.count)] = t;
            ++use.count;
        }
    };
    if (triangles.empty()) {
        m_uses.reserve(mesh.triangles.size() * 2);
        for (std::uint32_t t = 0; t < mesh.triangles.size(); ++t) visit(t);
    } else {
        m_uses.reserve(triangles.size() * 2);
        for (const std::uint32_t t : triangles) visit(t);
    }
}

const MeshEdges::Use* MeshEdges::find(std::uint32_t a, std::uint32_t b) const {
    const auto found = m_uses.find(meshEdgeKey(a, b));
    return found == m_uses.end() ? nullptr : &found->second;
}

std::uint32_t MeshEdges::across(std::uint32_t triangle, std::uint32_t a, std::uint32_t b) const {
    const Use* use = find(a, b);
    if (use == nullptr || use->count != 2) return UINT32_MAX;
    return use->triangles[0] == triangle ? use->triangles[1] : use->triangles[0];
}

std::vector<std::vector<std::uint32_t>> boundaryLoops(const Mesh& mesh, std::span<const std::uint32_t> triangles) {
    const MeshEdges edges(mesh, triangles);
    // As arestas de borda no sentido do triangulo dono.
    std::unordered_map<std::uint64_t, std::uint32_t> owner;
    std::vector<std::uint64_t> order;
    const auto visit = [&](std::uint32_t t) {
        const Triangle& tri = mesh.triangles[t];
        for (std::size_t i = 0; i < 3; ++i) {
            const std::uint32_t a = tri[i], b = tri[(i + 1) % 3];
            const MeshEdges::Use* use = edges.find(a, b);
            if (use == nullptr || use->count != 1) continue;
            owner[directedKey(a, b)] = t;
            order.push_back(directedKey(a, b));
        }
    };
    if (triangles.empty())
        for (std::uint32_t t = 0; t < mesh.triangles.size(); ++t) visit(t);
    else
        for (const std::uint32_t t : triangles) visit(t);

    // A proxima aresta de borda saindo de "at", girando pelo leque do
    // triangulo da aresta que chegou (assim um vertice com duas bordas nao
    // mistura as voltas).
    const auto nextEdge = [&](std::uint32_t from, std::uint32_t at) -> std::uint64_t {
        std::uint32_t triangle = owner.at(directedKey(from, at));
        for (std::size_t guard = 0; guard < 4096; ++guard) {
            const std::uint32_t out = nextIn(mesh.triangles[triangle], at);
            const auto boundary = owner.find(directedKey(at, out));
            if (boundary != owner.end() && boundary->second == triangle) return boundary->first;
            const std::uint32_t other = edges.across(triangle, at, out);
            if (other == UINT32_MAX) return UINT64_MAX;
            triangle = other;
        }
        return UINT64_MAX;
    };

    std::unordered_set<std::uint64_t> used;
    std::vector<std::vector<std::uint32_t>> loops;
    for (const std::uint64_t start : order) {
        if (used.count(start) != 0) continue;
        std::vector<std::uint32_t> loop;
        std::uint64_t edge = start;
        while (edge != UINT64_MAX && used.insert(edge).second) {
            const auto from = static_cast<std::uint32_t>(edge >> 32), to = static_cast<std::uint32_t>(edge & 0xffffffffu);
            loop.push_back(from);
            edge = nextEdge(from, to);
        }
        loops.push_back(std::move(loop));
    }
    return loops;
}

std::vector<int> edgeComponents(const Mesh& mesh, int* count) {
    const std::size_t triangles = mesh.triangles.size();
    UnionFind groups(triangles);
    std::unordered_map<std::uint64_t, std::uint32_t> first;
    first.reserve(triangles * 2);
    for (std::uint32_t t = 0; t < triangles; ++t) {
        const Triangle& tri = mesh.triangles[t];
        for (std::size_t i = 0; i < 3; ++i) {
            const auto [it, inserted] = first.try_emplace(meshEdgeKey(tri[i], tri[(i + 1) % 3]), t);
            if (!inserted) groups.unite(t, it->second);
        }
    }
    std::vector<int> component(triangles, -1);
    std::unordered_map<std::uint32_t, int> index;
    for (std::uint32_t t = 0; t < triangles; ++t) {
        const auto [it, inserted] = index.try_emplace(groups.find(t), static_cast<int>(index.size()));
        component[t] = it->second;
    }
    if (count != nullptr) *count = static_cast<int>(index.size());
    return component;
}

std::vector<std::uint32_t> pinchedVertices(const Mesh& mesh) {
    // Os triangulos de cada vertice; num leque so, eles se ligam pelas
    // arestas que saem do vertice.
    std::vector<std::vector<std::uint32_t>> around(mesh.positions.size());
    for (std::uint32_t t = 0; t < mesh.triangles.size(); ++t)
        for (const std::uint32_t v : mesh.triangles[t]) around[v].push_back(t);
    std::vector<std::uint32_t> pinched;
    for (std::uint32_t v = 0; v < around.size(); ++v) {
        const std::vector<std::uint32_t>& fan = around[v];
        if (fan.size() < 2) continue;
        UnionFind groups(fan.size());
        std::unordered_map<std::uint32_t, std::uint32_t> byOther;
        for (std::uint32_t i = 0; i < fan.size(); ++i)
            for (const std::uint32_t w : mesh.triangles[fan[i]]) {
                if (w == v) continue;
                const auto [it, inserted] = byOther.try_emplace(w, i);
                if (!inserted) groups.unite(i, it->second);
            }
        const std::uint32_t root = groups.find(0);
        for (std::uint32_t i = 1; i < fan.size(); ++i)
            if (groups.find(i) != root) {
                pinched.push_back(v);
                break;
            }
    }
    return pinched;
}

} // namespace MatterEngine::Modeling
