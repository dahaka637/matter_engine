#include "Engine/Modeling/Assembly/NeckJoin.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>

namespace MatterEngine::Modeling {

namespace {

float smooth(float edge0, float edge1, float x) {
    const float t = std::clamp((x - edge0) / (edge1 - edge0), 0.0f, 1.0f);
    return t * t * (3.0f - 2.0f * t);
}

// Do meio de tras para o meio da frente.
std::vector<std::uint32_t> backToFront(const Mesh& mesh, std::vector<std::uint32_t> chain) {
    if (mesh.positions[chain.front()].x > mesh.positions[chain.back()].x) std::reverse(chain.begin(), chain.end());
    return chain;
}

// Uma meia secao (lado direito) vista de cima, em volta de um centro no meio
// (x = centerX, y = 0): o angulo 0 e a frente, pi atras (passando pelo -y).
struct HalfSection {
    float centerX = 0.0f;
    std::vector<Vec3> points;

    [[nodiscard]] float angleOf(Vec3 p) const { return std::atan2(-std::min(0.0f, p.y), p.x - centerX); }
    [[nodiscard]] Vec3 direction(float angle) const { return { std::cos(angle), -std::sin(angle), 0.0f }; }
    // Onde o raio do centro no angulo dado encontra o contorno (a distancia).
    [[nodiscard]] float radius(float angle) const {
        const Vec3 d = direction(angle);
        float best = -1.0f;
        for (std::size_t i = 0; i + 1 < points.size(); ++i) {
            const float px = points[i].x - centerX, py = points[i].y;
            const float ex = points[i + 1].x - points[i].x, ey = points[i + 1].y - points[i].y;
            const float denominator = d.x * ey - d.y * ex;
            if (std::abs(denominator) < 1e-12f) continue;
            const float t = (px * ey - py * ex) / denominator;
            const float s = (px * d.y - py * d.x) / denominator;
            if (s < -1e-5f || s > 1.0f + 1e-5f || t <= 0.0f) continue;
            if (best < 0.0f || t < best) best = t;
        }
        if (best > 0.0f) return best;
        // Fora do contorno (nao acontece numa secao estrelada): o ponto mais perto.
        float nearest = std::numeric_limits<float>::max();
        for (const Vec3& p : points) nearest = std::min(nearest, std::hypot(p.x - centerX, p.y));
        return nearest;
    }
};

HalfSection sectionOf(const Mesh& mesh, const std::vector<std::uint32_t>& chain) {
    HalfSection section;
    section.centerX = 0.5f * (mesh.positions[chain.front()].x + mesh.positions[chain.back()].x);
    for (const std::uint32_t v : chain) section.points.push_back(mesh.positions[v]);
    return section;
}

// A altura do plano em (x, y).
float planeHeight(const CutPlane& plane, float x, float y) {
    const Vec3 n = plane.normal;
    return plane.point.z - (n.x * (x - plane.point.x) + n.y * (y - plane.point.y)) / n.z;
}

// O comprimento ao longo de um contorno (aberto) ate o ponto mais perto
// dele.
float arcPosition(const std::vector<Vec3>& polyline, Vec3 p) {
    float best = std::numeric_limits<float>::max(), position = 0.0f, walked = 0.0f;
    for (std::size_t i = 0; i + 1 < polyline.size(); ++i) {
        const Vec3 a = polyline[i], e = polyline[i + 1] - polyline[i];
        const float length = e.length();
        const float t = length > 1e-9f ? std::clamp(dot(p - a, e) / (length * length), 0.0f, 1.0f) : 0.0f;
        const float d = (a + e * t - p).length();
        if (d < best) {
            best = d;
            position = walked + t * length;
        }
        walked += length;
    }
    return position;
}

// Divide o triangulo (o unico) que tem a aresta u-v, com os vertices dados
// entre u e v (em ordem de u para v).
void splitBoundaryEdge(Mesh& mesh, std::uint32_t u, std::uint32_t v, const std::vector<std::uint32_t>& between) {
    if (between.empty()) return;
    std::size_t found = mesh.triangles.size();
    int matches = 0;
    for (std::size_t t = 0; t < mesh.triangles.size(); ++t) {
        const Triangle& tri = mesh.triangles[t];
        const bool hasU = tri[0] == u || tri[1] == u || tri[2] == u;
        const bool hasV = tri[0] == v || tri[1] == v || tri[2] == v;
        if (hasU && hasV) {
            found = t;
            ++matches;
        }
    }
    if (matches != 1) return;
    const Triangle tri = mesh.triangles[found];
    // A aresta no sentido do triangulo e o vertice de fora.
    int k = 0;
    while (!((tri[static_cast<std::size_t>(k)] == u && tri[static_cast<std::size_t>((k + 1) % 3)] == v)
        || (tri[static_cast<std::size_t>(k)] == v && tri[static_cast<std::size_t>((k + 1) % 3)] == u)))
        ++k;
    const std::uint32_t from = tri[static_cast<std::size_t>(k)], to = tri[static_cast<std::size_t>((k + 1) % 3)];
    const std::uint32_t apex = tri[static_cast<std::size_t>((k + 2) % 3)];
    std::vector<std::uint32_t> path { from };
    if (from == u) path.insert(path.end(), between.begin(), between.end());
    else path.insert(path.end(), between.rbegin(), between.rend());
    path.push_back(to);
    const std::uint16_t part = mesh.trianglePart[found], material = mesh.triangleMaterial[found];
    mesh.triangles[found] = { path[0], path[1], apex };
    for (std::size_t i = 1; i + 1 < path.size(); ++i) {
        mesh.triangles.push_back({ path[i], path[i + 1], apex });
        mesh.trianglePart.push_back(part);
        mesh.triangleMaterial.push_back(material);
    }
}

} // namespace

bool joinNeck(Mesh& mesh, const NeckJoin& join, std::string* error) {
    if (join.lower.size() < 2 || join.upper.size() < 2) {
        if (error != nullptr) *error = "pescoco: falta uma das bordas do corte";
        return false;
    }
    for (const std::vector<std::uint32_t>* chain : { &join.lower, &join.upper })
        if (std::abs(mesh.positions[chain->front()].y) > 1e-5f || std::abs(mesh.positions[chain->back()].y) > 1e-5f) {
            if (error != nullptr) *error = "pescoco: as bordas tem que ir do meio da frente ao meio de tras";
            return false;
        }
    const std::vector<std::uint32_t> lower = backToFront(mesh, join.lower);
    std::vector<std::uint32_t> upper = backToFront(mesh, join.upper);

    // ---- O ajuste: a borda de cima cai na de baixo; acima, some aos poucos ----
    const HalfSection target = sectionOf(mesh, lower);
    const HalfSection source = sectionOf(mesh, upper);
    const Vec3 normal = safeNormalized(join.plane.normal, { 0.0f, 0.0f, 1.0f });
    for (const std::uint32_t v : join.upperVertices) {
        Vec3& p = mesh.positions[v];
        const float above = std::max(0.0f, dot(p - join.plane.point, normal));
        const float weight = 1.0f - smooth(0.0f, join.blend, above);
        if (weight <= 0.0f) continue;
        const float angle = source.angleOf(p);
        const float r = std::hypot(p.x - source.centerX, std::min(0.0f, p.y));
        const float from = source.radius(angle);
        const float ratio = from > 1e-6f ? target.radius(angle) / from : 1.0f;
        const Vec3 d = target.direction(angle);
        const float x = target.centerX + d.x * r * ratio, y = d.y * r * ratio;
        const float oldPlane = planeHeight(join.plane, p.x, p.y);
        p.x += (x - p.x) * weight;
        if (p.y < 0.0f) p.y += (y - p.y) * weight;
        // Andando no x, o plano inclinado muda de altura: acompanha.
        p.z += (planeHeight(join.plane, p.x, p.y) - oldPlane) * weight;
    }

    // ---- O encaixe: a posicao de cada vertice ao longo da borda de baixo ----
    std::vector<Vec3> polyline;
    for (const std::uint32_t v : lower) polyline.push_back(mesh.positions[v]);
    std::vector<float> lowerAt, upperAt;
    for (const std::uint32_t v : lower) lowerAt.push_back(arcPosition(polyline, mesh.positions[v]));
    for (const std::uint32_t v : upper) upperAt.push_back(arcPosition(polyline, mesh.positions[v]));
    // Os de cima quase em cima de um de baixo viram ele (as pontas sempre).
    std::vector<std::uint32_t> replaced(mesh.positions.size());
    for (std::size_t v = 0; v < replaced.size(); ++v) replaced[v] = static_cast<std::uint32_t>(v);
    for (std::size_t j = 0; j < upper.size(); ++j) {
        std::size_t nearest = 0;
        for (std::size_t i = 1; i < lower.size(); ++i)
            if (std::abs(lowerAt[i] - upperAt[j]) < std::abs(lowerAt[nearest] - upperAt[j])) nearest = i;
        const bool end = j == 0 || j + 1 == upper.size();
        if (end) nearest = j == 0 ? 0 : lower.size() - 1;
        if (end || std::abs(lowerAt[nearest] - upperAt[j]) < join.weld) replaced[upper[j]] = lower[nearest];
    }
    for (Triangle& triangle : mesh.triangles)
        for (std::uint32_t& v : triangle) v = replaced[v];
    for (std::uint32_t& v : upper) v = replaced[v];
    // O triangulo de cima com dois vertices no mesmo de baixo fechou numa
    // linha: sai (os vizinhos dele ja dividem a aresta que sobrou).
    std::size_t kept = 0;
    for (std::size_t t = 0; t < mesh.triangles.size(); ++t) {
        const Triangle triangle = mesh.triangles[t];
        if (triangle[0] == triangle[1] || triangle[1] == triangle[2] || triangle[0] == triangle[2]) continue;
        mesh.triangles[kept] = triangle;
        mesh.trianglePart[kept] = mesh.trianglePart[t];
        mesh.triangleMaterial[kept] = mesh.triangleMaterial[t];
        ++kept;
    }
    mesh.triangles.resize(kept);
    mesh.trianglePart.resize(kept);
    mesh.triangleMaterial.resize(kept);
    // A borda de cima sem repetir (dois de cima no mesmo de baixo).
    std::vector<std::uint32_t> upperChain;
    std::vector<float> upperChainAt;
    for (std::size_t j = 0; j < upper.size(); ++j) {
        if (!upperChain.empty() && upperChain.back() == upper[j]) continue;
        upperChain.push_back(upper[j]);
        upperChainAt.push_back(upperAt[j]);
    }
    // Cada lado ganha, nas arestas da borda, os vertices do outro.
    const auto isLower = [&](std::uint32_t v) { return std::find(lower.begin(), lower.end(), v) != lower.end(); };
    for (std::size_t i = 0; i + 1 < lower.size(); ++i) {
        std::vector<std::pair<float, std::uint32_t>> between;
        for (std::size_t j = 0; j < upperChain.size(); ++j)
            if (!isLower(upperChain[j]) && upperChainAt[j] > lowerAt[i] && upperChainAt[j] < lowerAt[i + 1])
                between.push_back({ upperChainAt[j], upperChain[j] });
        std::sort(between.begin(), between.end());
        std::vector<std::uint32_t> vertices;
        for (const auto& [at, v] : between) vertices.push_back(v);
        splitBoundaryEdge(mesh, lower[i], lower[i + 1], vertices);
    }
    for (std::size_t j = 0; j + 1 < upperChain.size(); ++j) {
        std::vector<std::pair<float, std::uint32_t>> between;
        for (std::size_t i = 0; i < lower.size(); ++i) {
            if (lower[i] == upperChain[j] || lower[i] == upperChain[j + 1]) continue;
            if (lowerAt[i] > upperChainAt[j] && lowerAt[i] < upperChainAt[j + 1]) between.push_back({ lowerAt[i], lower[i] });
        }
        std::sort(between.begin(), between.end());
        std::vector<std::uint32_t> vertices;
        for (const auto& [at, v] : between) vertices.push_back(v);
        splitBoundaryEdge(mesh, upperChain[j], upperChain[j + 1], vertices);
    }
    return true;
}

} // namespace MatterEngine::Modeling
