#include "Engine/Modeling/Mesh/MeshImprint.hpp"

#include "Engine/Modeling/Geometry/Triangulation2D.hpp"
#include "Engine/Modeling/Mesh/MeshTopology.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <map>
#include <set>
#include <unordered_map>
#include <unordered_set>

namespace MatterEngine::Modeling {

namespace {

float cross2(Point2 a, Point2 b, Point2 c) {
    return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
}

// Distancia do ponto ao segmento e o parametro do mais perto.
float pointSegment2(Point2 p, Point2 a, Point2 b, float* parameter = nullptr) {
    const float dx = b.x - a.x, dy = b.y - a.y;
    const float lengthSquared = dx * dx + dy * dy;
    float t = lengthSquared > 0.0f ? ((p.x - a.x) * dx + (p.y - a.y) * dy) / lengthSquared : 0.0f;
    t = std::clamp(t, 0.0f, 1.0f);
    if (parameter != nullptr) *parameter = t;
    return std::hypot(p.x - (a.x + dx * t), p.y - (a.y + dy * t));
}

bool segmentsCross2(Point2 a, Point2 b, Point2 c, Point2 d) {
    const float d1 = cross2(a, b, c), d2 = cross2(a, b, d), d3 = cross2(c, d, a), d4 = cross2(c, d, b);
    return ((d1 > 0.0f) != (d2 > 0.0f)) && ((d3 > 0.0f) != (d4 > 0.0f));
}

std::array<float, 3> barycentric(Point2 p, Point2 a, Point2 b, Point2 c) {
    const float area = cross2(a, b, c);
    if (std::abs(area) < 1e-24f) return { 1.0f / 3.0f, 1.0f / 3.0f, 1.0f / 3.0f };
    const float wa = cross2(p, b, c) / area, wb = cross2(a, p, c) / area;
    return { wa, wb, 1.0f - wa - wb };
}

float pointTriangle2(Point2 p, Point2 a, Point2 b, Point2 c) {
    const auto w = barycentric(p, a, b, c);
    if (w[0] >= 0.0f && w[1] >= 0.0f && w[2] >= 0.0f) return 0.0f;
    return std::min({ pointSegment2(p, a, b), pointSegment2(p, b, c), pointSegment2(p, c, a) });
}

struct Frame2 {
    Vec3 x, y, z;
    [[nodiscard]] Point2 project(Vec3 p) const { return { dot(p, x), dot(p, y) }; }
};

Frame2 frameFor(Vec3 viewDirection) {
    const Vec3 z = safeNormalized(viewDirection, { 0.0f, 0.0f, 1.0f });
    const Vec3 x = anyPerpendicular(z);
    return { x, cross(z, x), z };
}

// Um ponto do arranjo: um vertice da superficie ou um novo (ponto de uma
// volta, ou o corte de uma aresta da volta com uma aresta de baixo).
struct ArrangementPoint {
    Point2 p2;
    std::uint32_t vertex = UINT32_MAX;
    Vec3 position;
    SkinWeights skin;
};

} // namespace

SkinWeights blendSkin(std::span<const SkinWeights> weights, std::span<const float> factors) {
    std::map<std::uint16_t, float> sum;
    for (std::size_t i = 0; i < weights.size() && i < factors.size(); ++i)
        for (std::size_t k = 0; k < 4; ++k)
            if (weights[i].weights[k] > 0.0f) sum[weights[i].joints[k]] += weights[i].weights[k] * std::max(0.0f, factors[i]);
    std::vector<std::pair<float, std::uint16_t>> ranked;
    for (const auto& [joint, weight] : sum)
        if (weight > 0.0f) ranked.push_back({ weight, joint });
    std::sort(ranked.begin(), ranked.end(), [](const auto& a, const auto& b) { return a.first > b.first; });
    SkinWeights result;
    float total = 0.0f;
    for (std::size_t k = 0; k < 4 && k < ranked.size(); ++k) total += ranked[k].first;
    for (std::size_t k = 0; k < 4 && k < ranked.size(); ++k) {
        result.joints[k] = ranked[k].second;
        result.weights[k] = total > 0.0f ? ranked[k].first / total : 0.0f;
    }
    return result;
}

std::optional<Vec3> projectOntoTriangles(const Mesh& mesh, std::span<const std::uint32_t> triangles, Vec3 point,
    Vec3 direction) {
    const Frame2 frame = frameFor(direction);
    const Point2 p = frame.project(point);
    std::optional<Vec3> best;
    float bestDistance = std::numeric_limits<float>::max();
    for (const std::uint32_t t : triangles) {
        const Triangle& tri = mesh.triangles[t];
        const Vec3 a = mesh.positions[tri[0]], b = mesh.positions[tri[1]], c = mesh.positions[tri[2]];
        const auto w = barycentric(p, frame.project(a), frame.project(b), frame.project(c));
        if (w[0] < -1e-4f || w[1] < -1e-4f || w[2] < -1e-4f) continue;
        const Vec3 hit = a * w[0] + b * w[1] + c * w[2];
        const float d = std::abs(dot(hit - point, frame.z));
        if (d < bestDistance) {
            bestDistance = d;
            best = point + frame.z * (dot(hit, frame.z) - dot(point, frame.z));
        }
    }
    return best;
}

ImprintResult imprintLoops(Mesh& mesh, std::span<const std::uint32_t> host, std::span<const ImprintLoop> loops,
    const ImprintSettings& settings) {
    ImprintResult result;
    result.loops.resize(loops.size());
    result.inputVertices.resize(loops.size());
    if (loops.empty()) {
        result.ok = true;
        return result;
    }
    const Frame2 frame = frameFor(settings.viewDirection);
    const float snap = std::max(settings.snap, 1e-7f);
    const bool skinned = mesh.hasSkin();

    const auto triangle2 = [&](std::uint32_t t) {
        const Triangle& tri = mesh.triangles[t];
        return std::array<Point2, 3> { frame.project(mesh.positions[tri[0]]), frame.project(mesh.positions[tri[1]]),
            frame.project(mesh.positions[tri[2]]) };
    };
    // Os de frente, sem dobra na vista.
    std::vector<std::uint32_t> facing;
    for (const std::uint32_t t : host) {
        if (t >= mesh.triangles.size() || dot(mesh.normal(t), frame.z) <= 0.02f) continue;
        const auto tri = triangle2(t);
        if (cross2(tri[0], tri[1], tri[2]) > 0.0f) facing.push_back(t);
    }
    std::vector<std::vector<Point2>> polygons(loops.size());
    for (std::size_t l = 0; l < loops.size(); ++l) {
        if (loops[l].points.size() < 3) {
            result.error = "volta com menos de 3 pontos";
            return result;
        }
        for (const Vec3& p : loops[l].points) polygons[l].push_back(frame.project(p));
    }

    // O remendo: os triangulos que as voltas tocam (por dentro, por fora ou
    // por uma aresta, com a folga do "snap").
    const auto touches = [&](std::uint32_t t, const std::vector<Point2>& polygon) {
        const auto tri = triangle2(t);
        for (const Point2& p : tri)
            if (pointInPolygon2D(p, polygon)) return true;
        for (const Point2& p : polygon)
            if (pointTriangle2(p, tri[0], tri[1], tri[2]) <= snap) return true;
        for (std::size_t i = 0; i < 3; ++i)
            for (std::size_t j = 0; j < polygon.size(); ++j)
                if (segmentsCross2(tri[i], tri[(i + 1) % 3], polygon[j], polygon[(j + 1) % polygon.size()])) return true;
        return false;
    };
    std::vector<std::uint32_t> patch;
    for (const std::uint32_t t : facing)
        for (const auto& polygon : polygons)
            if (touches(t, polygon)) {
                patch.push_back(t);
                break;
            }
    if (patch.empty()) {
        result.error = "as voltas nao caem sobre a superficie de frente";
        return result;
    }
    std::sort(patch.begin(), patch.end());

    // Pedacos ligados do remendo (as travas longe umas das outras).
    const MeshEdges patchEdges(mesh, patch);
    std::unordered_map<std::uint32_t, int> component;
    std::vector<std::vector<std::uint32_t>> components;
    for (const std::uint32_t seed : patch) {
        if (component.count(seed) != 0) continue;
        const int index = static_cast<int>(components.size());
        components.emplace_back();
        std::vector<std::uint32_t> stack { seed };
        component[seed] = index;
        while (!stack.empty()) {
            const std::uint32_t t = stack.back();
            stack.pop_back();
            components.back().push_back(t);
            const Triangle& tri = mesh.triangles[t];
            for (std::size_t i = 0; i < 3; ++i) {
                const std::uint32_t other = patchEdges.across(t, tri[i], tri[(i + 1) % 3]);
                if (other != UINT32_MAX && component.count(other) == 0) {
                    component[other] = index;
                    stack.push_back(other);
                }
            }
        }
    }

    struct Created {
        std::array<std::size_t, 3> points;
        std::uint16_t part;
        std::uint16_t material;
    };
    std::vector<ArrangementPoint> allPoints;
    std::vector<Created> created;
    std::vector<std::vector<std::size_t>> loopSequence(loops.size()), loopInputs(loops.size());
    std::vector<bool> loopDone(loops.size(), false);

    for (const std::vector<std::uint32_t>& triangles : components) {
        // O ponto dentro de qual triangulo do pedaco (ou nenhum).
        const auto locate = [&](Point2 p, std::array<float, 3>* weights) -> std::uint32_t {
            std::uint32_t best = UINT32_MAX;
            float bestDistance = std::numeric_limits<float>::max();
            for (const std::uint32_t t : triangles) {
                const auto tri = triangle2(t);
                const float d = pointTriangle2(p, tri[0], tri[1], tri[2]);
                if (d < bestDistance) {
                    bestDistance = d;
                    best = t;
                    if (weights != nullptr) *weights = barycentric(p, tri[0], tri[1], tri[2]);
                }
                if (d == 0.0f) break;
            }
            return bestDistance <= snap * 2.0f ? best : UINT32_MAX;
        };
        std::vector<std::size_t> mine;
        for (std::size_t l = 0; l < loops.size(); ++l) {
            if (loopDone[l]) continue;
            if (locate(polygons[l].front(), nullptr) != UINT32_MAX) mine.push_back(l);
        }
        if (mine.empty()) continue;

        // Os pontos: os vertices do pedaco primeiro.
        const std::size_t base = allPoints.size();
        std::unordered_map<std::uint32_t, std::size_t> hostPoint;
        for (const std::uint32_t t : triangles)
            for (const std::uint32_t v : mesh.triangles[t])
                if (hostPoint.try_emplace(v, allPoints.size()).second) {
                    ArrangementPoint point;
                    point.vertex = v;
                    point.p2 = frame.project(mesh.positions[v]);
                    point.position = mesh.positions[v];
                    allPoints.push_back(point);
                }
        // As arestas do pedaco (uma vez cada).
        struct HostEdge {
            std::size_t a, b;
            std::vector<std::pair<float, std::size_t>> splits;
        };
        std::vector<HostEdge> hostEdges;
        {
            std::unordered_set<std::uint64_t> seen;
            for (const std::uint32_t t : triangles) {
                const Triangle& tri = mesh.triangles[t];
                for (std::size_t i = 0; i < 3; ++i)
                    if (seen.insert(meshEdgeKey(tri[i], tri[(i + 1) % 3])).second)
                        hostEdges.push_back({ hostPoint.at(tri[i]), hostPoint.at(tri[(i + 1) % 3]), {} });
            }
        }

        // O quanto um corte encosta num vertice: nunca mais que um quinto da
        // menor aresta dele (paredes de meio milimetro nao podem ser puladas).
        std::unordered_map<std::size_t, float> shortest;
        for (const HostEdge& edge : hostEdges) {
            const Point2 a = allPoints[edge.a].p2, b = allPoints[edge.b].p2;
            const float length = std::hypot(b.x - a.x, b.y - a.y);
            for (const std::size_t id : { edge.a, edge.b }) {
                const auto [it, inserted] = shortest.try_emplace(id, length);
                if (!inserted) it->second = std::min(it->second, length);
            }
        }
        // (Abaixo de 20 micrometros e o mesmo ponto: solda sempre.)
        constexpr float Weld = 2e-5f;
        const auto snapAt = [&](std::size_t id) {
            const auto found = shortest.find(id);
            return std::max(Weld, found == shortest.end() ? snap : std::min(snap, 0.2f * found->second));
        };
        const auto snapAlong = [&](const HostEdge& edge) {
            const Point2 a = allPoints[edge.a].p2, b = allPoints[edge.b].p2;
            return std::max(Weld, std::min(snap, 0.2f * std::hypot(b.x - a.x, b.y - a.y)));
        };

        // Os pontos das voltas, ja na superficie (ou o vertice ao lado).
        for (const std::size_t l : mine) {
            for (const Vec3& point : loops[l].points) {
                const Point2 p2 = frame.project(point);
                std::array<float, 3> w {};
                const std::uint32_t under = locate(p2, &w);
                if (under == UINT32_MAX) {
                    result.error = "um ponto da volta fica fora da superficie de frente";
                    return result;
                }
                const Triangle& tri = mesh.triangles[under];
                std::size_t id = SIZE_MAX;
                for (std::size_t k = 0; k < 3 && id == SIZE_MAX; ++k) {
                    const std::size_t corner = hostPoint.at(tri[k]);
                    if (std::hypot(allPoints[corner].p2.x - p2.x, allPoints[corner].p2.y - p2.y) <= snapAt(corner)) id = corner;
                }
                if (id == SIZE_MAX) {
                    for (float& weight : w) weight = std::max(0.0f, weight);
                    const float sum = std::max(1e-12f, w[0] + w[1] + w[2]);
                    for (float& weight : w) weight /= sum;
                    ArrangementPoint created;
                    created.p2 = p2;
                    const Vec3 hit = mesh.positions[tri[0]] * w[0] + mesh.positions[tri[1]] * w[1] + mesh.positions[tri[2]] * w[2];
                    created.position = point + frame.z * (dot(hit, frame.z) - dot(point, frame.z));
                    if (skinned) {
                        const std::array<SkinWeights, 3> source { mesh.skin[tri[0]], mesh.skin[tri[1]], mesh.skin[tri[2]] };
                        created.skin = blendSkin(source, w);
                    }
                    id = allPoints.size();
                    allPoints.push_back(created);
                }
                loopInputs[l].push_back(id);
            }
        }

        // Cortes: cada aresta de volta com cada aresta de baixo; vertices
        // de baixo em cima de uma aresta de volta e pontos de volta em cima de
        // uma aresta de baixo dividem a aresta.
        struct LoopEdge {
            std::size_t l, i;
            std::size_t a, b;
            std::vector<std::pair<float, std::size_t>> splits;
        };
        std::vector<LoopEdge> loopEdges;
        for (const std::size_t l : mine) {
            const std::vector<std::size_t>& ids = loopInputs[l];
            for (std::size_t i = 0; i < ids.size(); ++i) {
                const std::size_t a = ids[i], b = ids[(i + 1) % ids.size()];
                if (a != b) loopEdges.push_back({ l, i, a, b, {} });
            }
        }
        for (LoopEdge& edge : loopEdges) {
            const Point2 p = allPoints[edge.a].p2, q = allPoints[edge.b].p2;
            const float length = std::hypot(q.x - p.x, q.y - p.y);
            for (const auto& [vertex, id] : hostPoint) {
                if (id == edge.a || id == edge.b) continue;
                float u = 0.0f;
                const float tolerance = snapAt(id);
                if (pointSegment2(allPoints[id].p2, p, q, &u) <= tolerance * 0.5f && u * length > tolerance
                    && (1.0f - u) * length > tolerance)
                    edge.splits.push_back({ u, id });
            }
        }
        for (HostEdge& edge : hostEdges) {
            const Point2 a = allPoints[edge.a].p2, b = allPoints[edge.b].p2;
            const float length = std::hypot(b.x - a.x, b.y - a.y);
            for (const LoopEdge& loopEdge : loopEdges)
                for (const std::size_t id : { loopEdge.a, loopEdge.b }) {
                    if (id == edge.a || id == edge.b || allPoints[id].vertex != UINT32_MAX) continue;
                    float s = 0.0f;
                    const float tolerance = snapAlong(edge);
                    if (pointSegment2(allPoints[id].p2, a, b, &s) <= tolerance * 0.5f && s * length > tolerance * 0.5f
                        && (1.0f - s) * length > tolerance * 0.5f) {
                        const bool already = std::any_of(edge.splits.begin(), edge.splits.end(),
                            [&](const auto& split) { return split.second == id; });
                        if (!already) edge.splits.push_back({ s, id });
                    }
                }
        }
        const auto addSplit = [](std::vector<std::pair<float, std::size_t>>& splits, float parameter, std::size_t id) {
            for (const auto& split : splits)
                if (split.second == id) return;
            splits.push_back({ parameter, id });
        };
        for (LoopEdge& loopEdge : loopEdges) {
            const Point2 p = allPoints[loopEdge.a].p2, q = allPoints[loopEdge.b].p2;
            for (HostEdge& hostEdge : hostEdges) {
                if (hostEdge.a == loopEdge.a || hostEdge.a == loopEdge.b || hostEdge.b == loopEdge.a || hostEdge.b == loopEdge.b)
                    continue;
                const Point2 a = allPoints[hostEdge.a].p2, b = allPoints[hostEdge.b].p2;
                const float d1 = cross2(p, q, a), d2 = cross2(p, q, b), d3 = cross2(a, b, p), d4 = cross2(a, b, q);
                if (!(((d1 > 0.0f) != (d2 > 0.0f)) && ((d3 > 0.0f) != (d4 > 0.0f)))) continue;
                const float s = d1 / (d1 - d2), u = d3 / (d3 - d4);
                const Point2 x { a.x + (b.x - a.x) * s, a.y + (b.y - a.y) * s };
                const auto distance = [&](Point2 point) { return std::hypot(point.x - x.x, point.y - x.y); };
                // Perto de uma ponta: o corte e a propria ponta (a aresta da
                // volta passa pelo vertice de baixo, ou o ponto da volta fica
                // na aresta de baixo).
                const bool nearA = distance(a) <= snapAt(hostEdge.a), nearB = distance(b) <= snapAt(hostEdge.b);
                if (nearA || nearB) {
                    const std::size_t id = nearA ? hostEdge.a : hostEdge.b;
                    float along = 0.0f;
                    pointSegment2(allPoints[id].p2, p, q, &along);
                    addSplit(loopEdge.splits, along, id);
                    continue;
                }
                const float onEdge = snapAlong(hostEdge);
                if (distance(p) <= onEdge || distance(q) <= onEdge) {
                    const std::size_t id = distance(p) <= onEdge ? loopEdge.a : loopEdge.b;
                    if (allPoints[id].vertex == UINT32_MAX) {
                        float along = 0.0f;
                        pointSegment2(allPoints[id].p2, a, b, &along);
                        addSplit(hostEdge.splits, along, id);
                        continue;
                    }
                }
                ArrangementPoint cut;
                cut.p2 = x;
                cut.position = lerp(allPoints[hostEdge.a].position, allPoints[hostEdge.b].position, s);
                if (skinned) {
                    const std::array<SkinWeights, 2> source { mesh.skin[allPoints[hostEdge.a].vertex],
                        mesh.skin[allPoints[hostEdge.b].vertex] };
                    const std::array<float, 2> factors { 1.0f - s, s };
                    cut.skin = blendSkin(source, factors);
                }
                const std::size_t id = allPoints.size();
                allPoints.push_back(cut);
                hostEdge.splits.push_back({ s, id });
                loopEdge.splits.push_back({ u, id });
            }
        }

        // As voltas inteiras (com os cortes) e as arestas obrigatorias.
        Triangulation2DInput input;
        for (std::size_t i = base; i < allPoints.size(); ++i) input.points.push_back(allPoints[i].p2);
        const auto local = [base](std::size_t id) { return static_cast<int>(id - base); };
        std::vector<std::array<int, 2>> constraints;
        const auto chain = [&](std::size_t from, std::size_t to, std::vector<std::pair<float, std::size_t>> splits,
                               std::vector<std::size_t>* sequence) {
            std::sort(splits.begin(), splits.end());
            std::size_t previous = from;
            for (const auto& [parameter, id] : splits) {
                if (id == previous) continue;
                constraints.push_back({ local(previous), local(id) });
                if (sequence != nullptr) sequence->push_back(previous);
                previous = id;
            }
            if (previous != to) {
                constraints.push_back({ local(previous), local(to) });
                if (sequence != nullptr) sequence->push_back(previous);
            }
        };
        for (const HostEdge& edge : hostEdges) chain(edge.a, edge.b, edge.splits, nullptr);
        for (const std::size_t l : mine) {
            std::vector<std::size_t> sequence;
            for (const LoopEdge& edge : loopEdges)
                if (edge.l == l) chain(edge.a, edge.b, edge.splits, &sequence);
            loopSequence[l] = sequence;
            loopDone[l] = true;
        }
        // O dominio: as bordas do pedaco (a maior e a de fora, as outras sao
        // ilhas), menos os furos.
        const std::vector<std::vector<std::uint32_t>> borders = boundaryLoops(mesh, triangles);
        std::size_t outer = 0;
        double outerArea = -std::numeric_limits<double>::max();
        std::vector<std::vector<int>> borderLoops;
        for (std::size_t b = 0; b < borders.size(); ++b) {
            std::vector<int> indices;
            std::vector<Point2> polygon;
            for (const std::uint32_t v : borders[b]) {
                indices.push_back(local(hostPoint.at(v)));
                polygon.push_back(allPoints[hostPoint.at(v)].p2);
            }
            const double area = std::abs(polygonArea2D(polygon));
            if (area > outerArea) {
                outerArea = area;
                outer = b;
            }
            borderLoops.push_back(std::move(indices));
        }
        input.loops.push_back(borderLoops[outer]);
        for (std::size_t b = 0; b < borderLoops.size(); ++b)
            if (b != outer) input.loops.push_back(borderLoops[b]);
        for (const std::size_t l : mine) {
            if (!loops[l].hole) continue;
            std::vector<int> indices;
            for (const std::size_t id : loopSequence[l]) indices.push_back(local(id));
            input.loops.push_back(std::move(indices));
        }
        input.segments = std::move(constraints);
        const Triangulation2DResult triangulation = triangulateDomain(input);
        if (!triangulation.ok) {
            result.error = "recorte: " + triangulation.error;
            if (triangulation.crossing[0] >= 0) {
                // Onde: as duas arestas (pontos no espaco do modelo; v = vertice
                // da superficie, n = ponto novo).
                const auto describe = [&](int index) {
                    const ArrangementPoint& point = allPoints[base + static_cast<std::size_t>(index)];
                    const Vec3 p = point.position;
                    return std::string(point.vertex != UINT32_MAX ? "v" : "n") + "(" + std::to_string(p.x) + ", "
                        + std::to_string(p.y) + ", " + std::to_string(p.z) + ")";
                };
                result.error += " [" + describe(triangulation.crossing[0]) + "-" + describe(triangulation.crossing[1]) + " x "
                    + describe(triangulation.crossing[2]) + "-" + describe(triangulation.crossing[3]) + "]";
            }
            return result;
        }
        for (const auto& tri : triangulation.triangles) {
            const std::array<std::size_t, 3> ids { base + static_cast<std::size_t>(tri[0]), base + static_cast<std::size_t>(tri[1]),
                base + static_cast<std::size_t>(tri[2]) };
            const Point2 a = allPoints[ids[0]].p2, b = allPoints[ids[1]].p2, c = allPoints[ids[2]].p2;
            const Point2 centroid { (a.x + b.x + c.x) / 3.0f, (a.y + b.y + c.y) / 3.0f };
            const std::uint32_t under = locate(centroid, nullptr);
            const std::uint32_t source = under != UINT32_MAX ? under : triangles.front();
            std::uint16_t part = mesh.trianglePart[source], material = mesh.triangleMaterial[source];
            double smallest = std::numeric_limits<double>::max();
            for (const std::size_t l : mine) {
                if (loops[l].hole || !pointInPolygon2D(centroid, polygons[l])) continue;
                const double area = std::abs(polygonArea2D(polygons[l]));
                if (area >= smallest) continue;
                smallest = area;
                part = loops[l].part >= 0 ? static_cast<std::uint16_t>(loops[l].part) : mesh.trianglePart[source];
                material = loops[l].material >= 0 ? static_cast<std::uint16_t>(loops[l].material) : mesh.triangleMaterial[source];
            }
            created.push_back({ ids, part, material });
        }
    }
    for (std::size_t l = 0; l < loops.size(); ++l)
        if (!loopDone[l]) {
            result.error = "uma volta nao caiu em nenhum pedaco da superficie";
            return result;
        }

    // Os vertices novos e a troca dos triangulos.
    for (ArrangementPoint& point : allPoints) {
        if (point.vertex != UINT32_MAX) continue;
        point.vertex = static_cast<std::uint32_t>(mesh.positions.size());
        mesh.positions.push_back(point.position);
        if (skinned) mesh.skin.push_back(point.skin);
    }
    std::vector<char> removed(mesh.triangles.size(), 0);
    for (const std::uint32_t t : patch) removed[t] = 1;
    std::size_t write = 0;
    for (std::size_t i = 0; i < mesh.triangles.size(); ++i) {
        if (removed[i] != 0) continue;
        mesh.triangles[write] = mesh.triangles[i];
        mesh.trianglePart[write] = mesh.trianglePart[i];
        mesh.triangleMaterial[write] = mesh.triangleMaterial[i];
        ++write;
    }
    mesh.triangles.resize(write);
    mesh.trianglePart.resize(write);
    mesh.triangleMaterial.resize(write);
    for (const Created& tri : created) {
        mesh.triangles.push_back({ allPoints[tri.points[0]].vertex, allPoints[tri.points[1]].vertex, allPoints[tri.points[2]].vertex });
        mesh.trianglePart.push_back(tri.part);
        mesh.triangleMaterial.push_back(tri.material);
    }
    for (std::size_t l = 0; l < loops.size(); ++l) {
        for (const std::size_t id : loopSequence[l]) result.loops[l].push_back(allPoints[id].vertex);
        for (const std::size_t id : loopInputs[l]) result.inputVertices[l].push_back(allPoints[id].vertex);
    }
    result.ok = true;
    return result;
}

} // namespace MatterEngine::Modeling
