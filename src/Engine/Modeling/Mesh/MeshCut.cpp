#include "Engine/Modeling/Mesh/MeshCut.hpp"

#include "Engine/Modeling/Mesh/MeshImprint.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <map>
#include <set>
#include <unordered_map>

namespace MatterEngine::Modeling {

namespace {

// As vizinhas de cada vertice pela borda que a malha ja tem (arestas de um
// triangulo so).
std::vector<std::vector<std::uint32_t>> rimNeighbors(const Mesh& mesh) {
    std::map<std::pair<std::uint32_t, std::uint32_t>, int> uses;
    for (const Triangle& triangle : mesh.triangles)
        for (int i = 0; i < 3; ++i) {
            const std::uint32_t a = triangle[static_cast<std::size_t>(i)], b = triangle[static_cast<std::size_t>((i + 1) % 3)];
            ++uses[{ std::min(a, b), std::max(a, b) }];
        }
    std::vector<std::vector<std::uint32_t>> rim(mesh.positions.size());
    for (const auto& [edge, count] : uses)
        if (count == 1) {
            rim[edge.first].push_back(edge.second);
            rim[edge.second].push_back(edge.first);
        }
    return rim;
}

// Leva ao corte o vertice a "d" dele ("gradient": para onde d cresce, um por
// metro num plano): pelo gradiente; o da borda da malha anda ao longo dela (o
// meio de uma metade continua no meio), e so se ela cruza o corte de frente
// (correndo quase nele, o passo seria longo: fica).
bool snapToCut(Mesh& mesh, std::uint32_t v, float d, Vec3 gradient, const std::vector<std::vector<std::uint32_t>>& rim, float snap) {
    const float strength = gradient.length();
    if (strength < 1e-6f) return false;
    if (rim[v].size() != 2) {
        mesh.positions[v] = mesh.positions[v] - gradient * (d / (strength * strength));
        return true;
    }
    const Vec3 along = safeNormalized(mesh.positions[rim[v][1]] - mesh.positions[rim[v][0]], {});
    const float rate = dot(along, gradient);
    if (std::abs(rate) < 0.3f * strength || std::abs(d / rate) > 2.0f * snap) return false;
    mesh.positions[v] = mesh.positions[v] - along * (d / rate);
    return true;
}

} // namespace

CutResult cutByPlane(Mesh& mesh, const CutPlane& plane, const CutSettings& settings) {
    CutResult result;
    const Vec3 normal = safeNormalized(plane.normal, { 0.0f, 0.0f, 1.0f });
    const bool skinned = mesh.hasSkin();
    // A distancia de cada vertice; os de perto vao para o plano.
    std::vector<float> distance(mesh.positions.size());
    std::vector<bool> onPlane(mesh.positions.size(), false);
    const std::vector<std::vector<std::uint32_t>> rim = rimNeighbors(mesh);
    for (std::size_t v = 0; v < mesh.positions.size(); ++v) {
        float d = dot(mesh.positions[v] - plane.point, normal);
        if (std::abs(d) < settings.snap && snapToCut(mesh, static_cast<std::uint32_t>(v), d, normal, rim, settings.snap)) {
            d = 0.0f;
            onPlane[v] = true;
        }
        distance[v] = d;
    }
    // O vertice novo de cada aresta cortada (o mesmo dos dois lados).
    std::map<std::pair<std::uint32_t, std::uint32_t>, std::uint32_t> crossing;
    const auto split = [&](std::uint32_t a, std::uint32_t b) {
        const auto key = std::make_pair(std::min(a, b), std::max(a, b));
        const auto found = crossing.find(key);
        if (found != crossing.end()) return found->second;
        const float da = distance[a], db = distance[b];
        const float s = da / (da - db);
        const std::uint32_t v = static_cast<std::uint32_t>(mesh.positions.size());
        Vec3 p = mesh.positions[a] + (mesh.positions[b] - mesh.positions[a]) * s;
        p = p - normal * dot(p - plane.point, normal);
        mesh.positions.push_back(p);
        distance.push_back(0.0f);
        onPlane.push_back(true);
        if (skinned) {
            const std::array<SkinWeights, 2> weights { mesh.skin[a], mesh.skin[b] };
            const std::array<float, 2> factors { 1.0f - s, s };
            mesh.skin.push_back(blendSkin(weights, factors));
        }
        crossing.emplace(key, v);
        return v;
    };

    std::vector<Triangle> triangles;
    std::vector<std::uint16_t> parts, materials;
    const auto emit = [&](Triangle triangle, std::size_t source) {
        triangles.push_back(triangle);
        parts.push_back(source < mesh.trianglePart.size() ? mesh.trianglePart[source] : 0);
        materials.push_back(source < mesh.triangleMaterial.size() ? mesh.triangleMaterial[source] : 0);
    };
    for (std::size_t t = 0; t < mesh.triangles.size(); ++t) {
        const Triangle triangle = mesh.triangles[t];
        const std::array<float, 3> d { distance[triangle[0]], distance[triangle[1]], distance[triangle[2]] };
        // Deitado no plano (os tres vertices foram para ele): sai.
        if (d[0] == 0.0f && d[1] == 0.0f && d[2] == 0.0f) continue;
        if (d[0] >= 0.0f && d[1] >= 0.0f && d[2] >= 0.0f) {
            emit(triangle, t);
            continue;
        }
        if (d[0] <= 0.0f && d[1] <= 0.0f && d[2] <= 0.0f) continue;
        // Sutherland-Hodgman: os vertices da frente (ou no plano) e os cortes.
        std::array<std::uint32_t, 4> polygon {};
        int count = 0;
        for (int i = 0; i < 3; ++i) {
            const std::uint32_t a = triangle[static_cast<std::size_t>(i)];
            const std::uint32_t b = triangle[static_cast<std::size_t>((i + 1) % 3)];
            const float da = d[static_cast<std::size_t>(i)], db = d[static_cast<std::size_t>((i + 1) % 3)];
            if (da >= 0.0f && count < 4) polygon[static_cast<std::size_t>(count++)] = a;
            if (((da > 0.0f && db < 0.0f) || (da < 0.0f && db > 0.0f)) && count < 4)
                polygon[static_cast<std::size_t>(count++)] = split(a, b);
        }
        for (int k = 1; k + 1 < count; ++k)
            emit({ polygon[0], polygon[static_cast<std::size_t>(k)], polygon[static_cast<std::size_t>(k + 1)] }, t);
    }
    mesh.triangles = std::move(triangles);
    mesh.trianglePart = std::move(parts);
    mesh.triangleMaterial = std::move(materials);

    // A borda nova: arestas de um triangulo so, com as duas pontas no plano.
    std::map<std::pair<std::uint32_t, std::uint32_t>, int> directed;
    for (const Triangle& triangle : mesh.triangles)
        for (int i = 0; i < 3; ++i)
            ++directed[{ triangle[static_cast<std::size_t>(i)], triangle[static_cast<std::size_t>((i + 1) % 3)] }];
    std::unordered_map<std::uint32_t, std::uint32_t> next;
    std::unordered_map<std::uint32_t, int> incoming;
    for (const auto& [edge, count] : directed) {
        const auto [a, b] = edge;
        if (!onPlane[a] || !onPlane[b] || directed.count({ b, a }) != 0) continue;
        if (next.count(a) != 0) {
            result.error = "corte: a borda nova se toca num vertice (o plano passa por um vertice pincado)";
            return result;
        }
        next[a] = b;
        ++incoming[b];
    }
    // Primeiro os trechos abertos (comecam onde nada chega), depois as voltas.
    const auto walk = [&](std::uint32_t start, bool open) {
        CutBorder border;
        border.closed = !open;
        std::uint32_t at = start;
        while (true) {
            border.vertices.push_back(at);
            const auto found = next.find(at);
            if (found == next.end()) break;
            const std::uint32_t following = found->second;
            next.erase(found);
            at = following;
            if (at == start) break;
        }
        result.borders.push_back(std::move(border));
    };
    std::vector<std::uint32_t> starts;
    for (const auto& [a, b] : next)
        if (incoming.count(a) == 0) starts.push_back(a);
    for (const std::uint32_t start : starts) walk(start, true);
    while (!next.empty()) walk(next.begin()->first, false);
    result.ok = true;
    return result;
}

CutField planeField(const CutPlane& plane) {
    const Vec3 normal = safeNormalized(plane.normal, { 0.0f, 0.0f, 1.0f });
    const Vec3 point = plane.point;
    return [normal, point](Vec3 p) { return dot(p - point, normal); };
}

CutResult splitNear(Mesh& mesh, const CutPlane& plane, Vec3 center, float radius, const CutSettings& settings) {
    return splitNear(mesh, planeField(plane), center, radius, settings);
}

CutResult splitNear(Mesh& mesh, const CutField& field, Vec3 center, float radius, const CutSettings& settings) {
    CutResult result;
    const bool skinned = mesh.hasSkin();
    std::vector<float> distance(mesh.positions.size());
    for (std::size_t v = 0; v < mesh.positions.size(); ++v) distance[v] = field(mesh.positions[v]);
    // Para onde a distancia cresce (diferencas pequenas).
    const auto gradient = [&](Vec3 p) {
        const float h = 1e-4f;
        return Vec3 { field(p + Vec3 { h, 0.0f, 0.0f }) - field(p - Vec3 { h, 0.0f, 0.0f }),
                   field(p + Vec3 { 0.0f, h, 0.0f }) - field(p - Vec3 { 0.0f, h, 0.0f }),
                   field(p + Vec3 { 0.0f, 0.0f, h }) - field(p - Vec3 { 0.0f, 0.0f, h }) }
            / (2.0f * h);
    };
    // Os triangulos que cruzam ou encostam no corte, ligados pelas arestas:
    // o pedaco que passa a menos de "radius" do centro e seguido inteiro (a
    // volta sai completa, sem aresta dividida de um lado so).
    const auto inSlab = [&](const Triangle& triangle) {
        float low = std::numeric_limits<float>::max(), high = std::numeric_limits<float>::lowest();
        for (const std::uint32_t v : triangle) {
            low = std::min(low, distance[v]);
            high = std::max(high, distance[v]);
        }
        return low < settings.snap && high > -settings.snap;
    };
    std::map<std::pair<std::uint32_t, std::uint32_t>, std::vector<std::size_t>> edgeTriangles;
    for (std::size_t t = 0; t < mesh.triangles.size(); ++t)
        for (int i = 0; i < 3; ++i) {
            const std::uint32_t a = mesh.triangles[t][static_cast<std::size_t>(i)];
            const std::uint32_t b = mesh.triangles[t][static_cast<std::size_t>((i + 1) % 3)];
            edgeTriangles[{ std::min(a, b), std::max(a, b) }].push_back(t);
        }
    std::vector<bool> selected(mesh.triangles.size(), false);
    std::vector<std::size_t> stack;
    for (std::size_t t = 0; t < mesh.triangles.size(); ++t) {
        if (!inSlab(mesh.triangles[t])) continue;
        for (const std::uint32_t v : mesh.triangles[t])
            if ((mesh.positions[v] - center).length() < radius) {
                selected[t] = true;
                stack.push_back(t);
                break;
            }
    }
    while (!stack.empty()) {
        const std::size_t t = stack.back();
        stack.pop_back();
        for (int i = 0; i < 3; ++i) {
            const std::uint32_t a = mesh.triangles[t][static_cast<std::size_t>(i)];
            const std::uint32_t b = mesh.triangles[t][static_cast<std::size_t>((i + 1) % 3)];
            for (const std::size_t other : edgeTriangles[{ std::min(a, b), std::max(a, b) }])
                if (!selected[other] && inSlab(mesh.triangles[other])) {
                    selected[other] = true;
                    stack.push_back(other);
                }
        }
    }
    // Os vertices desse pedaco perto do corte vao para ele, do mais perto ao
    // mais longe, menos o que deitaria um triangulo inteiro no corte (ele nao
    // seria de nenhum dos lados e a volta quebraria ali).
    std::vector<bool> near(mesh.positions.size(), false), onPlane(mesh.positions.size(), false);
    std::vector<std::vector<std::size_t>> incident(mesh.positions.size());
    for (std::size_t t = 0; t < mesh.triangles.size(); ++t)
        for (const std::uint32_t v : mesh.triangles[t]) {
            incident[v].push_back(t);
            if (selected[t]) near[v] = true;
        }
    std::vector<std::uint32_t> order;
    for (std::size_t v = 0; v < mesh.positions.size(); ++v)
        if (near[v] && std::abs(distance[v]) < settings.snap) order.push_back(static_cast<std::uint32_t>(v));
    std::sort(order.begin(), order.end(), [&](std::uint32_t a, std::uint32_t b) { return std::abs(distance[a]) < std::abs(distance[b]); });
    const std::vector<std::vector<std::uint32_t>> rim = rimNeighbors(mesh);
    for (const std::uint32_t v : order) {
        bool flattens = false;
        for (const std::size_t t : incident[v]) {
            int others = 0;
            for (const std::uint32_t u : mesh.triangles[t])
                if (u != v && onPlane[u]) ++others;
            flattens = flattens || others == 2;
        }
        if (flattens || !snapToCut(mesh, v, distance[v], gradient(mesh.positions[v]), rim, settings.snap)) continue;
        distance[v] = 0.0f;
        onPlane[v] = true;
    }
    std::map<std::pair<std::uint32_t, std::uint32_t>, std::uint32_t> crossing;
    const auto split = [&](std::uint32_t a, std::uint32_t b) {
        const auto key = std::make_pair(std::min(a, b), std::max(a, b));
        const auto found = crossing.find(key);
        if (found != crossing.end()) return found->second;
        // O zero na aresta (falsa posicao: exato num plano, perto disso numa
        // superficie curva); o vertice novo fica na aresta.
        const Vec3 pa = mesh.positions[a], pb = mesh.positions[b];
        float low = 0.0f, high = 1.0f, fLow = distance[a], fHigh = distance[b], s = fLow / (fLow - fHigh);
        for (int iteration = 0; iteration < 8; ++iteration) {
            s = low + (high - low) * fLow / (fLow - fHigh);
            const float f = field(pa + (pb - pa) * s);
            if (std::abs(f) < 1e-7f) break;
            if ((f < 0.0f) == (fLow < 0.0f)) {
                low = s;
                fLow = f;
            } else {
                high = s;
                fHigh = f;
            }
        }
        const std::uint32_t v = static_cast<std::uint32_t>(mesh.positions.size());
        mesh.positions.push_back(pa + (pb - pa) * s);
        distance.push_back(0.0f);
        onPlane.push_back(true);
        near.push_back(true);
        if (skinned) {
            const std::array<SkinWeights, 2> weights { mesh.skin[a], mesh.skin[b] };
            const std::array<float, 2> factors { 1.0f - s, s };
            mesh.skin.push_back(blendSkin(weights, factors));
        }
        crossing.emplace(key, v);
        return v;
    };
    // Os triangulos perto que cruzam o plano viram os dois lados (os
    // vertices novos no plano, os mesmos para os vizinhos).
    std::vector<Triangle> triangles;
    std::vector<std::uint16_t> parts, materials;
    std::vector<int> side;
    const auto emit = [&](Triangle triangle, std::size_t source, int which) {
        triangles.push_back(triangle);
        parts.push_back(source < mesh.trianglePart.size() ? mesh.trianglePart[source] : 0);
        materials.push_back(source < mesh.triangleMaterial.size() ? mesh.triangleMaterial[source] : 0);
        side.push_back(which);
    };
    for (std::size_t t = 0; t < mesh.triangles.size(); ++t) {
        const Triangle triangle = mesh.triangles[t];
        const std::array<float, 3> d { distance[triangle[0]], distance[triangle[1]], distance[triangle[2]] };
        const bool local = selected[t];
        const bool front = d[0] >= 0.0f && d[1] >= 0.0f && d[2] >= 0.0f;
        const bool back = d[0] <= 0.0f && d[1] <= 0.0f && d[2] <= 0.0f;
        if (!local || front || back) {
            emit(triangle, t, !local ? 0 : front && !back ? 1 : back && !front ? -1 : 0);
            continue;
        }
        for (const int which : { 1, -1 }) {
            std::array<std::uint32_t, 4> polygon {};
            int count = 0;
            for (int i = 0; i < 3; ++i) {
                const std::uint32_t a = triangle[static_cast<std::size_t>(i)];
                const std::uint32_t b = triangle[static_cast<std::size_t>((i + 1) % 3)];
                const float da = which * d[static_cast<std::size_t>(i)], db = which * d[static_cast<std::size_t>((i + 1) % 3)];
                if (da >= 0.0f && count < 4) polygon[static_cast<std::size_t>(count++)] = a;
                if (((da > 0.0f && db < 0.0f) || (da < 0.0f && db > 0.0f)) && count < 4) polygon[static_cast<std::size_t>(count++)] = split(a, b);
            }
            for (int k = 1; k + 1 < count; ++k)
                emit({ polygon[0], polygon[static_cast<std::size_t>(k)], polygon[static_cast<std::size_t>(k + 1)] }, t, which);
        }
    }
    mesh.triangles = std::move(triangles);
    mesh.trianglePart = std::move(parts);
    mesh.triangleMaterial = std::move(materials);
    // A volta: as arestas no plano entre um triangulo da frente e um de tras
    // (o sentido do da frente).
    std::map<std::pair<std::uint32_t, std::uint32_t>, int> owner;
    for (std::size_t t = 0; t < mesh.triangles.size(); ++t)
        for (int i = 0; i < 3; ++i) {
            const std::uint32_t a = mesh.triangles[t][static_cast<std::size_t>(i)];
            const std::uint32_t b = mesh.triangles[t][static_cast<std::size_t>((i + 1) % 3)];
            if (onPlane[a] && onPlane[b]) owner[{ a, b }] = side[t];
        }
    std::unordered_map<std::uint32_t, std::uint32_t> next;
    std::unordered_map<std::uint32_t, int> incoming;
    for (const auto& [edge, which] : owner) {
        if (which != 1) continue;
        const auto twin = owner.find({ edge.second, edge.first });
        if (twin == owner.end() || twin->second != -1) continue;
        if (next.count(edge.first) != 0) {
            result.error = "corte local: a volta se toca num vertice";
            return result;
        }
        next[edge.first] = edge.second;
        ++incoming[edge.second];
    }
    // Os trechos abertos comecam onde nada chega; depois as voltas.
    const auto walk = [&](std::uint32_t start, bool open) {
        CutBorder border;
        border.closed = !open;
        std::uint32_t at = start;
        while (true) {
            border.vertices.push_back(at);
            const auto found = next.find(at);
            if (found == next.end()) break;
            const std::uint32_t following = found->second;
            next.erase(found);
            at = following;
            if (at == start) break;
        }
        result.borders.push_back(std::move(border));
    };
    std::vector<std::uint32_t> starts;
    for (const auto& [a, b] : next)
        if (incoming.count(a) == 0) starts.push_back(a);
    for (const std::uint32_t start : starts) walk(start, true);
    while (!next.empty()) walk(next.begin()->first, false);
    // Um trecho aberto so pode comecar e acabar numa borda que a malha ja
    // tinha (o meio de uma metade); no meio da pele seria um corte que nao
    // fechou.
    const std::vector<std::vector<std::uint32_t>> after = rimNeighbors(mesh);
    for (const CutBorder& border : result.borders)
        if (!border.closed && (after[border.vertices.front()].empty() || after[border.vertices.back()].empty())) {
            result.error = "corte local: um trecho acaba no meio da malha (o corte nao fechou)";
            return result;
        }
    result.ok = true;
    return result;
}

std::size_t removeRegion(Mesh& mesh, Vec3 seed, std::span<const CutBorder> barriers) {
    if (mesh.triangles.empty()) return 0;
    std::set<std::pair<std::uint32_t, std::uint32_t>> wall;
    for (const CutBorder& border : barriers) {
        const std::size_t n = border.vertices.size();
        for (std::size_t i = 0; i + (border.closed ? 0 : 1) < n; ++i) {
            const std::uint32_t a = border.vertices[i], b = border.vertices[(i + 1) % n];
            wall.insert({ std::min(a, b), std::max(a, b) });
        }
    }
    std::map<std::pair<std::uint32_t, std::uint32_t>, std::vector<std::uint32_t>> edges;
    for (std::size_t t = 0; t < mesh.triangles.size(); ++t)
        for (int i = 0; i < 3; ++i) {
            const std::uint32_t a = mesh.triangles[t][static_cast<std::size_t>(i)];
            const std::uint32_t b = mesh.triangles[t][static_cast<std::size_t>((i + 1) % 3)];
            edges[{ std::min(a, b), std::max(a, b) }].push_back(static_cast<std::uint32_t>(t));
        }
    std::size_t start = 0;
    float best = std::numeric_limits<float>::max();
    for (std::size_t t = 0; t < mesh.triangles.size(); ++t) {
        const float d = (mesh.center(t) - seed).length();
        if (d < best) {
            best = d;
            start = t;
        }
    }
    std::vector<bool> removed(mesh.triangles.size(), false);
    std::vector<std::size_t> stack { start };
    removed[start] = true;
    while (!stack.empty()) {
        const std::size_t t = stack.back();
        stack.pop_back();
        for (int i = 0; i < 3; ++i) {
            const std::uint32_t a = mesh.triangles[t][static_cast<std::size_t>(i)];
            const std::uint32_t b = mesh.triangles[t][static_cast<std::size_t>((i + 1) % 3)];
            const auto key = std::make_pair(std::min(a, b), std::max(a, b));
            if (wall.count(key) != 0) continue;
            for (const std::uint32_t other : edges[key])
                if (!removed[other]) {
                    removed[other] = true;
                    stack.push_back(other);
                }
        }
    }
    std::size_t kept = 0, count = 0;
    for (std::size_t t = 0; t < mesh.triangles.size(); ++t) {
        if (removed[t]) {
            ++count;
            continue;
        }
        mesh.triangles[kept] = mesh.triangles[t];
        mesh.trianglePart[kept] = mesh.trianglePart[t];
        mesh.triangleMaterial[kept] = mesh.triangleMaterial[t];
        ++kept;
    }
    mesh.triangles.resize(kept);
    mesh.trianglePart.resize(kept);
    mesh.triangleMaterial.resize(kept);
    return count;
}

float loopWinding(const Mesh& mesh, std::span<const std::uint32_t> loop, Vec3 axis) {
    const Vec3 center = loopCenter(mesh, loop);
    Vec3 area {};
    for (std::size_t i = 0; i < loop.size(); ++i)
        area += cross(mesh.positions[loop[i]] - center, mesh.positions[loop[(i + 1) % loop.size()]] - center);
    return dot(area, axis);
}

Vec3 loopCenter(const Mesh& mesh, std::span<const std::uint32_t> loop) {
    Vec3 center {};
    for (const std::uint32_t v : loop) center += mesh.positions[v];
    return loop.empty() ? center : center / static_cast<float>(loop.size());
}

} // namespace MatterEngine::Modeling
