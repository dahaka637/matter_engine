#include "Engine/Modeling/Texture/UvAtlas.hpp"

#include "Engine/Modeling/Texture/IslandPacking.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <map>

namespace MatterEngine::Modeling {

namespace {

struct ChartPoint {
    float x = 0.0f;
    float y = 0.0f;
};

struct Chart {
    std::vector<std::size_t> triangles;
    float density = 1.0f;
    // Os cantos de cada triangulo no plano da ilha (metros), ja girados para
    // a menor caixa e com o canto da caixa em (0, 0).
    std::vector<std::array<ChartPoint, 3>> corners;
    float width = 0.0f;
    float height = 0.0f;
};

float cross2(ChartPoint o, ChartPoint a, ChartPoint b) {
    return (a.x - o.x) * (b.y - o.y) - (a.y - o.y) * (b.x - o.x);
}

// O fecho convexo (cadeia monotona).
std::vector<ChartPoint> convexHull(std::vector<ChartPoint> points) {
    std::sort(points.begin(), points.end(), [](ChartPoint a, ChartPoint b) { return a.x < b.x || (a.x == b.x && a.y < b.y); });
    if (points.size() < 3) return points;
    std::vector<ChartPoint> hull(points.size() * 2);
    std::size_t k = 0;
    for (const ChartPoint p : points) {
        while (k >= 2 && cross2(hull[k - 2], hull[k - 1], p) <= 0.0f) --k;
        hull[k++] = p;
    }
    for (std::size_t i = points.size() - 1, lower = k + 1; i-- > 0;) {
        const ChartPoint p = points[i];
        while (k >= lower && cross2(hull[k - 2], hull[k - 1], p) <= 0.0f) --k;
        hull[k++] = p;
    }
    hull.resize(k - 1);
    return hull;
}

// Gira a ilha para a menor caixa (uma das arestas do fecho fica deitada) e
// leva o canto da caixa para a origem.
void fitBox(Chart& chart) {
    std::vector<ChartPoint> points;
    for (const auto& corners : chart.corners)
        for (const ChartPoint p : corners) points.push_back(p);
    const std::vector<ChartPoint> hull = convexHull(points);
    float bestArea = std::numeric_limits<float>::max(), bestCos = 1.0f, bestSin = 0.0f;
    for (std::size_t i = 0; i < hull.size(); ++i) {
        const ChartPoint a = hull[i], b = hull[(i + 1) % hull.size()];
        const float length = std::hypot(b.x - a.x, b.y - a.y);
        if (length < 1e-9f) continue;
        const float c = (b.x - a.x) / length, s = (b.y - a.y) / length;
        float minX = std::numeric_limits<float>::max(), maxX = -minX, minY = minX, maxY = -minX;
        for (const ChartPoint p : hull) {
            const float x = p.x * c + p.y * s, y = -p.x * s + p.y * c;
            minX = std::min(minX, x);
            maxX = std::max(maxX, x);
            minY = std::min(minY, y);
            maxY = std::max(maxY, y);
        }
        const float area = (maxX - minX) * (maxY - minY);
        if (area < bestArea) {
            bestArea = area;
            bestCos = c;
            bestSin = s;
        }
    }
    float minX = std::numeric_limits<float>::max(), maxX = -minX, minY = minX, maxY = -minX;
    for (auto& corners : chart.corners)
        for (ChartPoint& p : corners) {
            p = { p.x * bestCos + p.y * bestSin, -p.x * bestSin + p.y * bestCos };
            minX = std::min(minX, p.x);
            maxX = std::max(maxX, p.x);
            minY = std::min(minY, p.y);
            maxY = std::max(maxY, p.y);
        }
    // Deitada: a caixa mais larga que alta (as prateleiras ficam baixas).
    const bool turn = maxY - minY > maxX - minX;
    for (auto& corners : chart.corners)
        for (ChartPoint& p : corners) p = turn ? ChartPoint { p.y - minY, maxX - p.x } : ChartPoint { p.x - minX, p.y - minY };
    chart.width = turn ? maxY - minY : maxX - minX;
    chart.height = turn ? maxX - minX : maxY - minY;
}

} // namespace

UvAtlasResult unwrapAtlas(Mesh& mesh, const UvAtlasSettings& settings) {
    UvAtlasResult result;
    const std::size_t count = mesh.triangles.size();
    if (count == 0 || settings.size < 64) {
        result.error = "atlas: malha vazia ou atlas pequeno demais";
        return result;
    }
    std::vector<Vec3> normals(count);
    std::vector<float> areas(count);
    for (std::size_t t = 0; t < count; ++t) {
        normals[t] = mesh.normal(t);
        areas[t] = mesh.area(t);
    }
    // Os vizinhos pela aresta, na mesma parte.
    std::map<std::pair<std::uint32_t, std::uint32_t>, std::vector<std::size_t>> edges;
    for (std::size_t t = 0; t < count; ++t)
        for (int i = 0; i < 3; ++i) {
            const std::uint32_t a = mesh.triangles[t][static_cast<std::size_t>(i)];
            const std::uint32_t b = mesh.triangles[t][static_cast<std::size_t>((i + 1) % 3)];
            edges[{ std::min(a, b), std::max(a, b) }].push_back(t);
        }
    std::vector<std::vector<std::size_t>> neighbors(count);
    for (const auto& [edge, triangles] : edges)
        for (const std::size_t a : triangles)
            for (const std::size_t b : triangles)
                if (a != b && mesh.partOf(a) == mesh.partOf(b)) neighbors[a].push_back(b);

    std::map<int, float> partDensity;
    for (const auto& [name, factor] : settings.partDensity) {
        const int part = mesh.findPart(name);
        if (part >= 0) partDensity[part] = std::max(0.1f, factor);
    }

    // ---- As ilhas: cresce de um triangulo grande pelos vizinhos que olham
    // para o mesmo lado da ilha ----
    const float limit = std::cos(settings.chartAngle * 3.14159265f / 180.0f);
    std::vector<std::size_t> seeds(count);
    for (std::size_t t = 0; t < count; ++t) seeds[t] = t;
    std::sort(seeds.begin(), seeds.end(), [&](std::size_t a, std::size_t b) { return areas[a] > areas[b]; });
    std::vector<int> chartOf(count, -1);
    std::vector<Chart> charts;
    bool pending = true;
    while (pending) {
        pending = false;
        for (const std::size_t seed : seeds) {
            if (chartOf[seed] >= 0) continue;
            const int index = static_cast<int>(charts.size());
            Chart chart;
            Vec3 sum = normals[seed] * std::max(areas[seed], 1e-12f);
            chart.triangles.push_back(seed);
            chartOf[seed] = index;
            for (std::size_t next = 0; next < chart.triangles.size(); ++next)
                for (const std::size_t other : neighbors[chart.triangles[next]]) {
                    if (chartOf[other] >= 0) continue;
                    if (dot(normals[other], safeNormalized(sum, normals[seed])) < limit) continue;
                    chartOf[other] = index;
                    chart.triangles.push_back(other);
                    sum += normals[other] * std::max(areas[other], 1e-12f);
                }
            // O plano da ilha e a media no fim: quem ficou de lado demais para
            // ele sai e comeca outra ilha.
            const Vec3 normal = safeNormalized(sum, normals[seed]);
            std::vector<std::size_t> kept;
            for (const std::size_t t : chart.triangles) {
                if (t != seed && dot(normals[t], normal) < 0.25f) {
                    chartOf[t] = -1;
                    pending = true;
                    continue;
                }
                kept.push_back(t);
            }
            chart.triangles = std::move(kept);
            const Vec3 across = anyPerpendicular(normal);
            const Vec3 up = cross(normal, across);
            for (const std::size_t t : chart.triangles) {
                std::array<ChartPoint, 3> corners {};
                for (int i = 0; i < 3; ++i) {
                    const Vec3 p = mesh.positions[mesh.triangles[t][static_cast<std::size_t>(i)]];
                    corners[static_cast<std::size_t>(i)] = { dot(p, across), dot(p, up) };
                }
                chart.corners.push_back(corners);
            }
            const auto found = partDensity.find(mesh.partOf(seed));
            chart.density = found != partDensity.end() ? found->second : 1.0f;
            fitBox(chart);
            charts.push_back(std::move(chart));
        }
    }

    // ---- A densidade: a maior que cabe ----
    float weighted = 0.0f;
    for (const Chart& chart : charts) {
        float area = 0.0f;
        for (const std::size_t t : chart.triangles) area += areas[t];
        weighted += area * chart.density * chart.density;
    }
    const float side = static_cast<float>(settings.size);
    // As ilhas em texels numa escala (IslandPacking arruma pelo formato).
    const auto islandsAt = [&](float texelsPerMeter) {
        std::vector<PackIsland> islands;
        islands.reserve(charts.size());
        for (const Chart& chart : charts) {
            const float scale = texelsPerMeter * chart.density;
            PackIsland island;
            island.width = chart.width * scale;
            island.height = chart.height * scale;
            island.triangles.reserve(chart.corners.size());
            for (const auto& corners : chart.corners)
                island.triangles.push_back({ PackPoint { corners[0].x * scale, corners[0].y * scale }, PackPoint { corners[1].x * scale, corners[1].y * scale },
                    PackPoint { corners[2].x * scale, corners[2].y * scale } });
            islands.push_back(std::move(island));
        }
        return islands;
    };
    PackSettings packing;
    packing.size = settings.size;
    packing.padding = settings.padding;
    // A busca: a escala de 100% de cobertura nao cabe; a de 85% dela (72%
    // de cobertura) quase sempre - dali, a maior que cabe em poucos passos
    // (meio por cento da escala).
    float high = std::sqrt(side * side / std::max(weighted, 1e-9f)), low = 0.0f;
    float texelsPerMeter = 0.85f * high;
    std::optional<std::vector<PackPlacement>> placements;
    for (int attempt = 0; attempt < 40 && !placements; ++attempt) {
        placements = packIslands(islandsAt(texelsPerMeter), packing);
        if (!placements) {
            high = texelsPerMeter;
            texelsPerMeter *= 0.85f;
        }
    }
    if (!placements) {
        result.error = "atlas: as ilhas nao couberam";
        return result;
    }
    low = texelsPerMeter;
    while (high - low > 0.005f * high) {
        const float middle = 0.5f * (low + high);
        if (auto tried = packIslands(islandsAt(middle), packing)) {
            low = middle;
            placements = std::move(tried);
        } else {
            high = middle;
        }
    }
    texelsPerMeter = low;

    // ---- As coordenadas ----
    mesh.triangleUv.assign(count, {});
    float covered = 0.0f;
    for (std::size_t c = 0; c < charts.size(); ++c) {
        const Chart& chart = charts[c];
        const PackPlacement& placement = (*placements)[c];
        const float scale = texelsPerMeter * chart.density;
        for (std::size_t i = 0; i < chart.triangles.size(); ++i) {
            std::array<Uv, 3>& uv = mesh.triangleUv[chart.triangles[i]];
            for (int k = 0; k < 3; ++k) {
                const ChartPoint p = chart.corners[i][static_cast<std::size_t>(k)];
                const PackPoint q = placement.at({ p.x * scale, p.y * scale });
                uv[static_cast<std::size_t>(k)] = { q.x / side, q.y / side };
            }
            covered += areas[chart.triangles[i]] * scale * scale;
        }
    }
    result.ok = true;
    result.charts = charts.size();
    result.texelsPerMeter = texelsPerMeter;
    result.coverage = covered / (side * side);
    return result;
}

} // namespace MatterEngine::Modeling
