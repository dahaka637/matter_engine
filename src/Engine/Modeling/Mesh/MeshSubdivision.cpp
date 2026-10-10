#include "Engine/Modeling/Mesh/MeshSubdivision.hpp"

#include "Engine/Modeling/Mesh/MeshImprint.hpp"

#include <algorithm>
#include <array>
#include <map>
#include <utility>

namespace MatterEngine::Modeling {

namespace {

using Edge = std::pair<std::uint32_t, std::uint32_t>;

Edge edgeKey(std::uint32_t a, std::uint32_t b) { return { std::min(a, b), std::max(a, b) }; }

Uv middleUv(Uv a, Uv b) { return { 0.5f * (a.u + b.u), 0.5f * (a.v + b.v) }; }

// "whole": os triangulos que podem virar quatro (null: todos). Devolve, em
// "children", quais dos novos sairam de um que virou quatro.
std::size_t subdivideOnce(Mesh& mesh, std::span<CutBorder* const> borders, const std::function<bool(std::size_t)>& refine,
    const std::vector<char>* whole, std::vector<char>& children) {
    const bool skinned = mesh.hasSkin(), mapped = mesh.hasUv();
    const std::size_t count = mesh.triangles.size();
    // Os que viram quatro: os pedidos e, ate parar, os que ficariam com duas
    // ou tres arestas partidas pelos vizinhos (o resto parte em dois). So os
    // que podem ("whole"); um que nao pode e fica com duas parte em tres.
    std::vector<char> red(count, 1);
    if (refine)
        for (std::size_t t = 0; t < count; ++t) red[t] = refine(t) ? 1 : 0;
    if (whole)
        for (std::size_t t = 0; t < count; ++t) red[t] = red[t] && (*whole)[t];
    std::map<Edge, int> splitUses;
    const auto markEdges = [&](std::size_t t) {
        const Triangle& tri = mesh.triangles[t];
        for (int i = 0; i < 3; ++i) ++splitUses[edgeKey(tri[static_cast<std::size_t>(i)], tri[static_cast<std::size_t>((i + 1) % 3)])];
    };
    for (std::size_t t = 0; t < count; ++t)
        if (red[t]) markEdges(t);
    const auto splitEdges = [&](std::size_t t) {
        const Triangle& tri = mesh.triangles[t];
        int split = 0;
        for (int i = 0; i < 3; ++i)
            split += splitUses.count(edgeKey(tri[static_cast<std::size_t>(i)], tri[static_cast<std::size_t>((i + 1) % 3)])) ? 1 : 0;
        return split;
    };
    for (bool changed = true; changed;) {
        changed = false;
        for (std::size_t t = 0; t < count; ++t)
            if (!red[t] && splitEdges(t) >= (whole && !(*whole)[t] ? 3 : 2)) {
                red[t] = 1;
                markEdges(t);
                changed = true;
            }
    }
    std::map<Edge, std::uint32_t> middle;
    const auto middleOf = [&](std::uint32_t a, std::uint32_t b) {
        const auto [at, inserted] = middle.try_emplace(edgeKey(a, b), static_cast<std::uint32_t>(mesh.positions.size()));
        if (inserted) {
            mesh.positions.push_back((mesh.positions[a] + mesh.positions[b]) * 0.5f);
            if (skinned) {
                const std::array<SkinWeights, 2> ends { mesh.skin[a], mesh.skin[b] };
                const std::array<float, 2> half { 0.5f, 0.5f };
                mesh.skin.push_back(blendSkin(ends, half));
            }
        }
        return at->second;
    };
    const std::size_t before = mesh.positions.size();
    std::vector<Triangle> triangles;
    std::vector<std::uint16_t> parts, materials;
    std::vector<std::array<Uv, 3>> uvs;
    triangles.reserve(count * 4);
    children.clear();
    const auto emit = [&](std::size_t from, const Triangle& tri, const std::array<Uv, 3>& uv) {
        children.push_back(red[from]);
        triangles.push_back(tri);
        parts.push_back(from < mesh.trianglePart.size() ? mesh.trianglePart[from] : 0);
        materials.push_back(from < mesh.triangleMaterial.size() ? mesh.triangleMaterial[from] : 0);
        if (mapped) uvs.push_back(uv);
    };
    for (std::size_t t = 0; t < count; ++t) {
        const Triangle tri = mesh.triangles[t];
        const std::array<Uv, 3> c = mapped ? mesh.triangleUv[t] : std::array<Uv, 3> {};
        if (red[t]) {
            // Os tres dos cantos e o do meio, no mesmo sentido.
            const std::uint32_t ab = middleOf(tri[0], tri[1]), bc = middleOf(tri[1], tri[2]), ca = middleOf(tri[2], tri[0]);
            const Uv uab = middleUv(c[0], c[1]), ubc = middleUv(c[1], c[2]), uca = middleUv(c[2], c[0]);
            emit(t, { tri[0], ab, ca }, { c[0], uab, uca });
            emit(t, { ab, tri[1], bc }, { uab, c[1], ubc });
            emit(t, { ca, bc, tri[2] }, { uca, ubc, c[2] });
            emit(t, { ab, bc, ca }, { uab, ubc, uca });
            continue;
        }
        std::array<bool, 3> split {};
        int splits = 0;
        for (int i = 0; i < 3; ++i) {
            split[static_cast<std::size_t>(i)] =
                splitUses.count(edgeKey(tri[static_cast<std::size_t>(i)], tri[static_cast<std::size_t>((i + 1) % 3)])) != 0;
            splits += split[static_cast<std::size_t>(i)] ? 1 : 0;
        }
        if (splits == 0) {
            emit(t, tri, c);
            continue;
        }
        if (splits == 1) {
            // Uma aresta partida (a do vizinho): dois, do meio dela ao canto
            // em frente.
            const auto i0 = static_cast<std::size_t>(split[0] ? 0 : split[1] ? 1 : 2);
            const auto i1 = (i0 + 1) % 3, i2 = (i0 + 2) % 3;
            const std::uint32_t m = middleOf(tri[i0], tri[i1]);
            const Uv um = middleUv(c[i0], c[i1]);
            emit(t, { tri[i0], m, tri[i2] }, { c[i0], um, c[i2] });
            emit(t, { m, tri[i1], tri[i2] }, { um, c[i1], c[i2] });
            continue;
        }
        // Duas (o que nao pode virar quatro): o canto entre elas e o resto em
        // dois pela diagonal mais curta - a terceira aresta fica inteira.
        const auto free = static_cast<std::size_t>(!split[0] ? 0 : !split[1] ? 1 : 2);
        const auto a = (free + 1) % 3, b = (free + 2) % 3, o = free;
        // Arestas partidas: a->b e b->o; a inteira: o->a.
        const std::uint32_t mab = middleOf(tri[a], tri[b]), mbo = middleOf(tri[b], tri[o]);
        const Uv uab = middleUv(c[a], c[b]), ubo = middleUv(c[b], c[o]);
        emit(t, { mab, tri[b], mbo }, { uab, c[b], ubo });
        if ((mesh.positions[mab] - mesh.positions[tri[o]]).lengthSquared() < (mesh.positions[mbo] - mesh.positions[tri[a]]).lengthSquared()) {
            emit(t, { tri[a], mab, tri[o] }, { c[a], uab, c[o] });
            emit(t, { mab, mbo, tri[o] }, { uab, ubo, c[o] });
        } else {
            emit(t, { tri[a], mab, mbo }, { c[a], uab, ubo });
            emit(t, { tri[a], mbo, tri[o] }, { c[a], ubo, c[o] });
        }
    }
    mesh.triangles = std::move(triangles);
    mesh.trianglePart = std::move(parts);
    mesh.triangleMaterial = std::move(materials);
    if (mapped) mesh.triangleUv = std::move(uvs);
    // As bordas: o meio de cada aresta partida entre dois vertices seguidos.
    for (CutBorder* border : borders) {
        if (border == nullptr || border->vertices.size() < 2) continue;
        const std::vector<std::uint32_t>& old = border->vertices;
        std::vector<std::uint32_t> denser;
        denser.reserve(old.size() * 2);
        for (std::size_t i = 0; i < old.size(); ++i) {
            denser.push_back(old[i]);
            if (i + 1 == old.size() && !border->closed) break;
            const auto found = middle.find(edgeKey(old[i], old[(i + 1) % old.size()]));
            if (found != middle.end()) denser.push_back(found->second);
        }
        border->vertices = std::move(denser);
    }
    return mesh.positions.size() - before;
}

} // namespace

std::size_t subdivideTriangles(Mesh& mesh, int levels, std::span<CutBorder* const> borders, const std::function<bool(std::size_t)>& refine) {
    std::size_t added = 0;
    std::vector<char> children;
    for (int level = 0; level < levels; ++level) {
        if (level == 0) {
            added += subdivideOnce(mesh, borders, refine, nullptr, children);
            continue;
        }
        // Do segundo nivel em diante so viram quatro os filhos de um que
        // virou quatro com todos os vizinhos tambem assim: o que partiu em
        // dois no nivel anterior nunca parte de novo (a metade de uma metade
        // vira agulha comprida, que o skin vira do avesso).
        std::map<Edge, std::vector<std::uint32_t>> users;
        for (std::size_t t = 0; t < mesh.triangles.size(); ++t)
            for (int i = 0; i < 3; ++i)
                users[edgeKey(mesh.triangles[t][static_cast<std::size_t>(i)], mesh.triangles[t][static_cast<std::size_t>((i + 1) % 3)])]
                    .push_back(static_cast<std::uint32_t>(t));
        std::vector<char> whole = children;
        for (const auto& [edge, list] : users) {
            bool all = true;
            for (const std::uint32_t t : list) all = all && children[t];
            if (!all)
                for (const std::uint32_t t : list) whole[t] = 0;
        }
        added += subdivideOnce(mesh, borders, refine, &whole, children);
    }
    return added;
}

} // namespace MatterEngine::Modeling
