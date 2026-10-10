#include "Engine/Modeling/Assembly/Garment.hpp"

#include "Engine/Modeling/Geometry/ModelGeometry.hpp"
#include "Engine/Modeling/Mesh/MeshBuilder.hpp"
#include "Engine/Modeling/Mesh/MeshStitch.hpp"
#include "Engine/Modeling/Mesh/TriangleBvh.hpp"

#include <algorithm>
#include <cmath>
#include <map>
#include <set>

namespace MatterEngine::Modeling {

namespace {

// As bordas de todos os cortes, para servir de barreira.
RegionResult splitAll(Mesh& mesh, std::span<const RegionCut> cuts, const CutSettings& settings) {
    RegionResult result;
    for (const RegionCut& cut : cuts) {
        const CutResult split = splitNear(mesh, cut.field, cut.center, cut.radius, settings);
        if (!split.ok) {
            result.error = split.error;
            return result;
        }
        if (split.borders.size() != 1) {
            result.error = "o corte em (" + std::to_string(cut.center.x) + ", " + std::to_string(cut.center.y) + ", "
                + std::to_string(cut.center.z) + ") deu " + std::to_string(split.borders.size()) + " bordas (esperada uma)";
            return result;
        }
        result.borders.push_back(split.borders.front());
    }
    result.ok = true;
    return result;
}

} // namespace

RegionResult keepRegion(Mesh& mesh, std::span<const RegionCut> cuts, const CutSettings& settings) {
    RegionResult result = splitAll(mesh, cuts, settings);
    if (!result.ok) return result;
    for (const RegionCut& cut : cuts)
        if (removeRegion(mesh, cut.beyond, result.borders) == 0) {
            result.ok = false;
            result.error = "nada saiu alem de um corte";
            return result;
        }
    return result;
}

RegionResult removeCovered(Mesh& mesh, std::span<const RegionCut> cuts, Vec3 covered, const CutSettings& settings) {
    RegionResult result = splitAll(mesh, cuts, settings);
    if (!result.ok) return result;
    if (removeRegion(mesh, covered, result.borders) == 0) {
        result.ok = false;
        result.error = "nada coberto saiu";
    }
    return result;
}

void inflateShell(Mesh& cloth, const Mesh& body, const std::function<float(std::uint32_t)>& loose, const ClothShape& shape) {
    const std::size_t count = cloth.positions.size();
    std::vector<bool> used(count, false);
    std::map<std::pair<std::uint32_t, std::uint32_t>, int> edges;
    std::vector<std::set<std::uint32_t>> neighbors(count);
    for (const Triangle& triangle : cloth.triangles)
        for (int i = 0; i < 3; ++i) {
            const std::uint32_t a = triangle[static_cast<std::size_t>(i)], b = triangle[static_cast<std::size_t>((i + 1) % 3)];
            used[a] = true;
            ++edges[{ std::min(a, b), std::max(a, b) }];
            neighbors[a].insert(b);
            neighbors[b].insert(a);
        }
    std::vector<bool> middle(count, false);
    std::vector<float> target(count, 0.0f);
    for (std::size_t v = 0; v < count; ++v) {
        if (!used[v]) continue;
        middle[v] = std::abs(cloth.positions[v].y) < 1e-6f;
        target[v] = loose(static_cast<std::uint32_t>(v));
    }
    // Quem olha para o meio (a metade direita: a normal com y > 0) infla no
    // maximo metade do espaco ate ele: o lado de dentro da perna do calcao
    // nao chega ao limite do meio (la ele viraria uma parede chapada e o
    // gancho, lascas).
    {
        std::vector<Vec3> facing(count, Vec3 {});
        for (const Triangle& triangle : cloth.triangles) {
            const Vec3 a = cloth.positions[triangle[0]], b = cloth.positions[triangle[1]], c = cloth.positions[triangle[2]];
            const Vec3 n = cross(b - a, c - a);
            for (const std::uint32_t v : triangle) facing[v] += n;
        }
        for (std::size_t v = 0; v < count; ++v) {
            if (!used[v] || middle[v]) continue;
            const Vec3 n = safeNormalized(facing[v], {});
            if (n.y <= 0.2f) continue;
            const float room = std::max(0.0f, -cloth.positions[v].y - shape.midlineGap) / n.y;
            target[v] = std::min(target[v], 0.5f * room);
        }
    }
    // A borda (aresta de um triangulo so, fora a que corre no meio: as
    // barras, inclusive a ponta delas no meio) nao alisa.
    std::vector<bool> border(count, false);
    for (const auto& [edge, uses] : edges)
        if (uses == 1 && !(middle[edge.first] && middle[edge.second])) border[edge.first] = border[edge.second] = true;
    // As normais pelos triangulos em volta (as do meio no plano do meio).
    const auto normals = [&] {
        std::vector<Vec3> result(count, Vec3 {});
        for (const Triangle& triangle : cloth.triangles) {
            const Vec3 a = cloth.positions[triangle[0]], b = cloth.positions[triangle[1]], c = cloth.positions[triangle[2]];
            const Vec3 n = cross(b - a, c - a);
            for (const std::uint32_t v : triangle) result[v] += n;
        }
        for (std::size_t v = 0; v < count; ++v) {
            if (middle[v]) result[v].y = 0.0f;
            result[v] = safeNormalized(result[v], { 1.0f, 0.0f, 0.0f });
        }
        return result;
    };
    // Fora do corpo: a "distance" do ponto mais perto dele, pelo lado de fora.
    const TriangleBvh bvh(body.positions, body.triangles);
    const auto keepOut = [&](std::uint32_t v, float distance) {
        const Vec3 p = cloth.positions[v];
        const std::optional<TriangleBvh::Nearest> nearest = bvh.nearest(p, distance + 0.05f);
        if (!nearest) return;
        const Triangle& triangle = body.triangles[nearest->triangle];
        const Vec3 a = body.positions[triangle[0]], b = body.positions[triangle[1]], c = body.positions[triangle[2]];
        const Vec3 face = safeNormalized(cross(b - a, c - a), { 1.0f, 0.0f, 0.0f });
        const Vec3 away = p - nearest->point;
        const float length = away.length();
        const bool outside = dot(away, face) > 0.0f;
        if (outside && length >= distance) return;
        Vec3 moved = nearest->point + (outside && length > 1e-6f ? away / length : face) * distance;
        if (middle[v]) moved.y = 0.0f;
        cloth.positions[v] = moved;
    };
    // Antes de inflar, Taubin (um passo para o meio dos vizinhos e um um pouco
    // maior de volta: nao encolhe) arredonda os vincos fundos (a axila).
    std::vector<float> relaxWeight(count, 1.0f);
    if (shape.relaxWeight)
        for (std::size_t v = 0; v < count; ++v)
            if (used[v]) relaxWeight[v] = std::clamp(shape.relaxWeight(static_cast<std::uint32_t>(v)), 0.0f, 1.0f);
    for (int iteration = 0; iteration < shape.relax; ++iteration)
        for (const float factor : { 0.5f, -0.53f }) {
            std::vector<Vec3> moved = cloth.positions;
            for (std::size_t v = 0; v < count; ++v) {
                if (!used[v] || neighbors[v].empty() || border[v] || relaxWeight[v] <= 0.0f) continue;
                Vec3 average {};
                for (const std::uint32_t u : neighbors[v]) average += cloth.positions[u];
                average = average / static_cast<float>(neighbors[v].size());
                Vec3 step = (average - cloth.positions[v]) * (factor * relaxWeight[v]);
                if (middle[v]) step.y = 0.0f;
                moved[v] = cloth.positions[v] + step;
            }
            cloth.positions = std::move(moved);
        }
    const int steps = std::max(1, shape.steps);
    for (int step = 1; step <= steps; ++step) {
        std::vector<Vec3> n = normals();
        for (std::size_t v = 0; v < count; ++v)
            if (used[v]) cloth.positions[v] += n[v] * (target[v] / static_cast<float>(steps));
        // Alisa so para fora, ao longo da normal: o vinco sobe ate os
        // vizinhos (o pano passa por cima dele); o que ja e convexo fica (nada
        // encolhe para dentro do corpo); ninguem escorrega de lado.
        for (int pass = 0; pass < shape.smoothing; ++pass) {
            n = normals();
            std::vector<Vec3> moved = cloth.positions;
            for (std::size_t v = 0; v < count; ++v) {
                if (!used[v] || neighbors[v].empty() || border[v]) continue;
                Vec3 average {};
                for (const std::uint32_t u : neighbors[v]) average += cloth.positions[u];
                average = average / static_cast<float>(neighbors[v].size());
                moved[v] = cloth.positions[v] + n[v] * (0.5f * std::max(0.0f, dot(average - cloth.positions[v], n[v])));
            }
            cloth.positions = std::move(moved);
        }
        const float fraction = static_cast<float>(step) / static_cast<float>(steps);
        for (std::size_t v = 0; v < count; ++v) {
            if (!used[v]) continue;
            keepOut(static_cast<std::uint32_t>(v), target[v] * fraction);
            Vec3& p = cloth.positions[v];
            if (middle[v]) p.y = 0.0f;
            else p.y = std::min(p.y, -shape.midlineGap);
        }
    }
    // A queda: em volta de cada eixo, o raio de um vertice nao fica menor
    // que o de um de cima (no mesmo angulo) menos a inclinacao pela descida.
    for (const ClothHang& hang : shape.hangs) {
        const Vec3 up = safeNormalized(hang.axisUp, { 0.0f, 0.0f, 1.0f });
        const Vec3 side = safeNormalized(cross(up, std::abs(up.x) < 0.9f ? Vec3 { 1.0f, 0.0f, 0.0f } : Vec3 { 0.0f, 1.0f, 0.0f }), {});
        const Vec3 front = cross(side, up);
        struct Polar {
            std::uint32_t vertex;
            float height, radius, angle;
        };
        std::vector<Polar> polar;
        for (std::size_t v = 0; v < count; ++v) {
            if (!used[v] || (hang.joins && !hang.joins(static_cast<std::uint32_t>(v)))) continue;
            const Vec3 d = cloth.positions[v] - hang.axisPoint;
            const float height = dot(d, up);
            const Vec3 flat = d - up * height;
            polar.push_back({ static_cast<std::uint32_t>(v), height, flat.length(), std::atan2(dot(flat, side), dot(flat, front)) });
        }
        const float window = hang.angle * 3.14159265f / 180.0f;
        std::vector<float> radius(polar.size());
        for (std::size_t i = 0; i < polar.size(); ++i) {
            radius[i] = polar[i].radius;
            for (const Polar& above : polar) {
                const float drop = above.height - polar[i].height;
                if (drop <= 0.0f || drop > hang.reach) continue;
                float turn = std::abs(above.angle - polar[i].angle);
                turn = std::min(turn, 2.0f * 3.14159265f - turn);
                if (turn > window) continue;
                // Mais longe no angulo segura menos (some na borda da abertura).
                const float hold = 1.0f - turn / window;
                radius[i] = std::max(radius[i], polar[i].radius + (above.radius - hang.slope * drop - polar[i].radius) * hold);
            }
        }
        for (std::size_t i = 0; i < polar.size(); ++i) {
            if (radius[i] <= polar[i].radius + 1e-6f) continue;
            const std::uint32_t v = polar[i].vertex;
            const Vec3 d = cloth.positions[v] - hang.axisPoint;
            const Vec3 flat = d - up * dot(d, up);
            // O lado de dentro (para o meio) nao cai: ali nao ha para onde.
            if (!middle[v] && safeNormalized(flat, {}).y > 0.3f) continue;
            Vec3 moved = cloth.positions[v] + safeNormalized(flat, {}) * (radius[i] - polar[i].radius);
            if (middle[v]) moved.y = 0.0f;
            else moved.y = std::min(moved.y, -shape.midlineGap);
            cloth.positions[v] = moved;
        }
    }
    // Onde o pano ainda se cruza (um vinco fundo demais para a folga), os
    // vertices dos triangulos que se cruzam e os vizinhos deles vao para o
    // meio dos vizinhos (a borda fica), ate soltar.
    for (int round = 0; round < shape.untangle; ++round) {
        std::vector<bool> tangled(count, false);
        bool any = false;
        const TriangleBvh clothBvh(cloth.positions, cloth.triangles);
        clothBvh.overlappingPairs([&](std::uint32_t a, std::uint32_t b) {
            const Triangle& ta = cloth.triangles[a];
            const Triangle& tb = cloth.triangles[b];
            for (const std::uint32_t va : ta)
                for (const std::uint32_t vb : tb)
                    if (va == vb) return;
            const std::vector<Vec3>& p = cloth.positions;
            if (!trianglesIntersect(p[ta[0]], p[ta[1]], p[ta[2]], p[tb[0]], p[tb[1]], p[tb[2]])) return;
            any = true;
            for (const std::uint32_t v : ta) tangled[v] = true;
            for (const std::uint32_t v : tb) tangled[v] = true;
        });
        if (!any) break;
        std::vector<bool> around = tangled;
        for (std::size_t v = 0; v < count; ++v)
            if (tangled[v])
                for (const std::uint32_t u : neighbors[v]) around[u] = true;
        std::vector<Vec3> moved = cloth.positions;
        for (std::size_t v = 0; v < count; ++v) {
            if (!around[v] || neighbors[v].empty() || border[v]) continue;
            Vec3 average {};
            for (const std::uint32_t u : neighbors[v]) average += cloth.positions[u];
            average = average / static_cast<float>(neighbors[v].size());
            moved[v] = cloth.positions[v] + (average - cloth.positions[v]) * 0.5f;
            if (middle[v]) moved[v].y = 0.0f;
        }
        cloth.positions = std::move(moved);
        for (std::size_t v = 0; v < count; ++v) {
            if (!around[v]) continue;
            keepOut(static_cast<std::uint32_t>(v), target[v]);
            Vec3& p = cloth.positions[v];
            if (middle[v]) p.y = 0.0f;
            else p.y = std::min(p.y, -shape.midlineGap);
        }
    }
}

bool stitchRecess(Mesh& mesh, const CutBorder& garmentEdge, const CutBorder& inner, std::uint16_t part, std::uint16_t material,
    std::string* error) {
    if (garmentEdge.closed != inner.closed || garmentEdge.vertices.size() < 2 || inner.vertices.size() < 2) {
        if (error != nullptr) *error = "recuo: as bordas da peca e de dentro nao combinam (uma volta e um trecho?)";
        return false;
    }
    // A faixa usa as arestas das bordas no sentido contrario ao dos
    // triangulos que ja estao nelas: a borda da peca ao contrario, a de
    // dentro como veio.
    std::vector<std::uint32_t> from(garmentEdge.vertices.rbegin(), garmentEdge.vertices.rend());
    const std::vector<std::uint32_t>& to = inner.vertices;
    const std::size_t firstTriangle = mesh.triangles.size();
    MeshBuilder builder(mesh);
    if (part < mesh.parts.size()) builder.part(mesh.parts[part].name);
    if (material < mesh.materials.size()) builder.useMaterial(mesh.materials[material].name);
    if (garmentEdge.closed) {
        stitchLoops(builder, from, to);
    } else {
        // Os trechos tem que comecar na mesma ponta (o meio da frente ou o de
        // tras).
        if ((mesh.positions[from.front()] - mesh.positions[to.front()]).length()
            > (mesh.positions[from.front()] - mesh.positions[to.back()]).length()) {
            if (error != nullptr) *error = "recuo: os trechos andam em sentidos opostos";
            return false;
        }
        stitchStrips(builder, from, to);
    }
    for (std::size_t t = firstTriangle; t < mesh.triangles.size(); ++t) {
        mesh.trianglePart[t] = part;
        mesh.triangleMaterial[t] = material;
    }
    return true;
}

} // namespace MatterEngine::Modeling
