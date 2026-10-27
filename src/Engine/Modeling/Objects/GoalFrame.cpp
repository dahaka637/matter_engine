#include "Engine/Modeling/Objects/GoalFrame.hpp"

#include "Engine/Modeling/Objects/ObjectMesh.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <map>
#include <tuple>

namespace MatterEngine::Modeling {

namespace {

// O cabo tensor do alto da rede: 6 mm de PP (~20 g/m), bem mais rigido que o
// fio e esticado (a receita diz quanto: o cabo mais curto que o vao). Frouxo,
// ele carregava o peso do fundo e afundava ~30 cm no meio dos 7,4 m.
constexpr float CordStiffness = 30000.0f;
constexpr float CordDensity = 0.020f;
constexpr int TubeSides = 24;

float distanceOf(Vec3 a, Vec3 b) {
    return std::sqrt((b - a).lengthSquared());
}

Vec3 unitOf(Vec3 v) {
    const float length = std::sqrt(v.lengthSquared());
    return length > 1e-12f ? v * (1.0f / length) : Vec3 {};
}

// A rede montada pano a pano: os nos soldados pela posicao (o canto que dois
// panos dividem e um no so; o cabo do teto e o do fundo tambem).
struct NetBuilder {
    ObjectNet& net;
    float slack = 0.05f;
    float stiffness = 10000.0f;
    float density = 0.007f;
    // O cabo: o alongamento dele esticado entre as pontas (o comprimento solto
    // e o vao menos isto).
    float cordStretch = 0.015f;
    std::map<std::tuple<long, long, long>, std::uint32_t> welded;
    std::map<std::pair<std::uint32_t, std::uint32_t>, std::size_t> linked;

    std::uint32_t knot(Vec3 p) {
        const auto key = std::make_tuple(std::lround(p.x * 1e4f), std::lround(p.y * 1e4f), std::lround(p.z * 1e4f));
        const auto found = welded.find(key);
        if (found != welded.end()) return found->second;
        net.knots.push_back(p);
        net.pinned.push_back(0);
        const auto index = static_cast<std::uint32_t>(net.knots.size() - 1);
        welded.emplace(key, index);
        return index;
    }

    // Um fio (ou o cabo: o mesmo par ja ligado pela malha vira cabo).
    void link(std::uint32_t a, std::uint32_t b, bool cord = false) {
        if (a == b) return;
        const float span = distanceOf(net.knots[a], net.knots[b]);
        if (span < 1e-5f) return;
        const ObjectNetSegment segment { a, b, span * (cord ? 1.0f - cordStretch : 1.0f + slack), cord ? CordStiffness : stiffness,
            cord ? CordDensity : density, cord };
        const auto [found, fresh] = linked.emplace(std::make_pair(std::min(a, b), std::max(a, b)), net.segments.size());
        if (fresh) net.segments.push_back(segment);
        else if (cord) net.segments[found->second] = segment;
    }
};

struct PanelPoint {
    float u = 0.0f;
    float v = 0.0f;
};

struct PanelEdge {
    // Presa na estrutura (a presilha a cada clipEvery nos) ou o cabo.
    bool pinned = true;
    bool cord = false;
};

// Um pano plano e convexo: os cantos em (u, v), anti-horario, no plano
// origin + u * uAxis + v * vAxis, e como fica cada borda.
struct Panel {
    Vec3 origin;
    Vec3 uAxis;
    Vec3 vAxis;
    std::vector<PanelPoint> corners;
    std::vector<PanelEdge> edges;
};

// A grade de um pano (colunas x linhas no retangulo que o contem) e os nos
// dele com o (u, v) de cada um.
struct PanelGrid {
    float uMin = 0.0f;
    float vMin = 0.0f;
    float du = 1.0f;
    float dv = 1.0f;
    int columns = 1;
    int rows = 1;
    std::vector<std::pair<PanelPoint, std::uint32_t>> knots;
};

// O lado de p em relacao a borda a -> b (m; positivo a esquerda: dentro).
float sideOf(PanelPoint a, PanelPoint b, PanelPoint p) {
    const float ex = b.u - a.u, ey = b.v - a.v;
    const float length = std::sqrt(ex * ex + ey * ey);
    return length > 1e-9f ? (ex * (p.v - a.v) - ey * (p.u - a.u)) / length : 0.0f;
}

// A caixa do pano em (u, v).
void panelBox(const Panel& panel, float& uMin, float& uMax, float& vMin, float& vMax) {
    uMin = vMin = 1e9f;
    uMax = vMax = -1e9f;
    for (const PanelPoint& p : panel.corners) {
        uMin = std::min(uMin, p.u);
        uMax = std::max(uMax, p.u);
        vMin = std::min(vMin, p.v);
        vMax = std::max(vMax, p.v);
    }
}

// As celulas do tamanho `cell` que cabem na caixa do pano: as colunas em
// numero par (o meio do pano numa linha da malha, cada celula com a sua
// espelhada do outro lado) e as linhas.
std::pair<int, int> panelCells(const Panel& panel, float cell) {
    float uMin = 0.0f, uMax = 0.0f, vMin = 0.0f, vMax = 0.0f;
    panelBox(panel, uMin, uMax, vMin, vMax);
    return { 2 * std::max(1, static_cast<int>(std::lround((uMax - uMin) / (2.0f * cell)))),
        std::max(1, static_cast<int>(std::lround((vMax - vMin) / cell))) };
}

// A malha alinhada com a caixa do pano (os fios em u e em v), em colunas x
// linhas. Onde a linha da malha cruza a borda nasce um no, e a borda leva a
// corda (o debrum) de no a no, com a presilha a cada clipEvery nos.
PanelGrid addPanel(NetBuilder& builder, const Panel& panel, int columns, int rows, int clipEvery) {
    constexpr float Epsilon = 1e-5f;
    const std::vector<PanelPoint>& corners = panel.corners;
    float uMin = 0.0f, uMax = 0.0f, vMin = 0.0f, vMax = 0.0f;
    panelBox(panel, uMin, uMax, vMin, vMax);
    PanelGrid grid;
    grid.columns = columns;
    grid.rows = rows;
    grid.uMin = uMin;
    grid.vMin = vMin;
    grid.du = (uMax - uMin) / static_cast<float>(grid.columns);
    grid.dv = (vMax - vMin) / static_cast<float>(grid.rows);
    const float du = grid.du, dv = grid.dv;
    const auto at = [&](float u, float v) { return panel.origin + panel.uAxis * u + panel.vAxis * v; };
    const auto knotAt = [&](PanelPoint p) {
        const std::uint32_t knot = builder.knot(at(p.u, p.v));
        grid.knots.push_back({ p, knot });
        return knot;
    };
    const std::size_t count = corners.size();
    // O trecho de uma reta (p + d * s) dentro do pano: [enter, leave].
    const auto clip = [&](PanelPoint p, PanelPoint d, float& enter, float& leave) {
        enter = -1e9f;
        leave = 1e9f;
        for (std::size_t e = 0; e < count; ++e) {
            const PanelPoint a = corners[e], b = corners[(e + 1) % count];
            // Dentro: a esquerda da borda (anti-horario).
            const float ex = b.u - a.u, ey = b.v - a.v;
            const float side = ex * (p.v - a.v) - ey * (p.u - a.u);
            const float rate = ex * d.v - ey * d.u;
            if (std::abs(rate) < 1e-12f) {
                if (side < -Epsilon) return false;
                continue;
            }
            const float s = -side / rate;
            if (rate > 0.0f) enter = std::max(enter, s);
            else leave = std::min(leave, s);
        }
        return leave >= enter - Epsilon;
    };
    // Os nos da borda de cada lado (para o debrum e as presilhas).
    std::vector<std::vector<std::pair<float, std::uint32_t>>> onEdge(count);
    const auto noteBoundary = [&](PanelPoint p, std::uint32_t knot) {
        for (std::size_t e = 0; e < count; ++e) {
            const PanelPoint a = corners[e], b = corners[(e + 1) % count];
            const float ex = b.u - a.u, ey = b.v - a.v;
            const float length = std::sqrt(ex * ex + ey * ey);
            if (length < 1e-9f) continue;
            const float off = std::abs(ex * (p.v - a.v) - ey * (p.u - a.u)) / length;
            const float t = ((p.u - a.u) * ex + (p.v - a.v) * ey) / (length * length);
            if (off < 1e-4f && t > -1e-4f && t < 1.0f + 1e-4f) onEdge[e].push_back({ t, knot });
        }
    };
    // Uma linha da malha: os nos de dentro, os da borda nas pontas, e os fios.
    const auto line = [&](PanelPoint start, PanelPoint direction, int steps, float step) {
        float enter = 0.0f, leave = 0.0f;
        if (!clip(start, direction, enter, leave)) return;
        std::vector<std::pair<float, PanelPoint>> points;
        points.push_back({ enter, { start.u + direction.u * enter, start.v + direction.v * enter } });
        for (int k = 0; k <= steps; ++k) {
            const float s = step * static_cast<float>(k);
            if (s > enter + 1e-4f && s < leave - 1e-4f) points.push_back({ s, { start.u + direction.u * s, start.v + direction.v * s } });
        }
        if (leave > enter + 1e-4f) points.push_back({ leave, { start.u + direction.u * leave, start.v + direction.v * leave } });
        std::uint32_t previous = 0;
        for (std::size_t k = 0; k < points.size(); ++k) {
            const PanelPoint p = points[k].second;
            const std::uint32_t knot = knotAt(p);
            if (k == 0 || k + 1 == points.size()) noteBoundary(p, knot);
            if (k > 0) builder.link(previous, knot);
            previous = knot;
        }
    };
    for (int r = 0; r <= rows; ++r) line({ uMin, vMin + dv * static_cast<float>(r) }, { 1.0f, 0.0f }, columns, du);
    for (int c = 0; c <= columns; ++c) line({ uMin + du * static_cast<float>(c), vMin }, { 0.0f, 1.0f }, rows, dv);
    for (std::size_t e = 0; e < count; ++e) {
        onEdge[e].push_back({ 0.0f, knotAt(corners[e]) });
        onEdge[e].push_back({ 1.0f, knotAt(corners[(e + 1) % count]) });
    }
    // O debrum (de no a no pela borda) e as presilhas; os cantos sempre presos.
    for (std::size_t e = 0; e < count; ++e) {
        auto& list = onEdge[e];
        std::sort(list.begin(), list.end());
        list.erase(std::unique(list.begin(), list.end(), [](const auto& x, const auto& y) { return x.second == y.second; }), list.end());
        for (std::size_t k = 1; k < list.size(); ++k) builder.link(list[k - 1].second, list[k].second, panel.edges[e].cord);
        if (!panel.edges[e].pinned) continue;
        for (std::size_t k = 0; k < list.size(); ++k)
            if (k == 0 || k + 1 == list.size() || static_cast<int>(k) % std::max(1, clipEvery) == 0)
                builder.net.pinned[list[k].second] = 1;
    }
    // A ponta do cabo esta presa na estrutura (as presilhas dos lados).
    for (std::size_t e = 0; e < count; ++e)
        if (panel.edges[e].cord && !onEdge[e].empty()) {
            builder.net.pinned[onEdge[e].front().second] = 1;
            builder.net.pinned[onEdge[e].back().second] = 1;
        }
    return grid;
}

// A celula (o retangulo) recortada pelo pano convexo, sem cantos repetidos.
std::vector<PanelPoint> clipCell(const Panel& panel, float u0, float v0, float u1, float v1) {
    constexpr float Epsilon = 1e-6f;
    std::vector<PanelPoint> polygon { { u0, v0 }, { u1, v0 }, { u1, v1 }, { u0, v1 } };
    const std::size_t count = panel.corners.size();
    for (std::size_t e = 0; e < count && polygon.size() >= 3; ++e) {
        const PanelPoint a = panel.corners[e], b = panel.corners[(e + 1) % count];
        std::vector<PanelPoint> kept;
        for (std::size_t k = 0; k < polygon.size(); ++k) {
            const PanelPoint p = polygon[k], q = polygon[(k + 1) % polygon.size()];
            const float sp = sideOf(a, b, p), sq = sideOf(a, b, q);
            if (sp >= -Epsilon) kept.push_back(p);
            if ((sp >= -Epsilon) != (sq >= -Epsilon)) {
                const float t = sp / (sp - sq);
                kept.push_back({ p.u + (q.u - p.u) * t, p.v + (q.v - p.v) * t });
            }
        }
        polygon = std::move(kept);
    }
    const auto same = [](PanelPoint x, PanelPoint y) { return std::abs(x.u - y.u) + std::abs(x.v - y.v) <= 1e-5f; };
    std::vector<PanelPoint> unique;
    for (const PanelPoint& p : polygon)
        if (unique.empty() || !same(p, unique.back())) unique.push_back(p);
    while (unique.size() > 1 && same(unique.front(), unique.back())) unique.pop_back();
    return unique;
}

// Uma celula da malha da fisica dentro do pano: os triangulos dela com o
// (u, v) de cada canto.
struct PanelCell {
    std::vector<std::array<std::uint32_t, 3>> triangles;
    std::vector<std::array<PanelPoint, 3>> shapes;
};

// As celulas da malha da fisica: cada celula da grade recortada pelo pano,
// os cantos achados entre os nos dele; as inteiras em dois triangulos, as
// cortadas em leque. A diagonal alterna linha a linha e espelha no meio do
// pano: a rede igual dos dois lados (com a coluna do meio sem par, o chute
// no meio saia ate 1 m para um lado).
bool meshPanel(ObjectNet& net, const Panel& panel, const PanelGrid& grid, std::vector<PanelCell>& cells, std::string& error) {
    cells.assign(static_cast<std::size_t>(grid.columns * grid.rows), {});
    const auto knotAt = [&](PanelPoint p, std::uint32_t& knot) {
        float best = 1e-8f;
        bool found = false;
        for (const auto& [q, index] : grid.knots) {
            const float distance = (q.u - p.u) * (q.u - p.u) + (q.v - p.v) * (q.v - p.v);
            if (distance < best) {
                best = distance;
                knot = index;
                found = true;
            }
        }
        return found;
    };
    const float cellArea = grid.du * grid.dv;
    for (int j = 0; j < grid.rows; ++j)
        for (int i = 0; i < grid.columns; ++i) {
            const float u0 = grid.uMin + grid.du * static_cast<float>(i), v0 = grid.vMin + grid.dv * static_cast<float>(j);
            const std::vector<PanelPoint> polygon = clipCell(panel, u0, v0, u0 + grid.du, v0 + grid.dv);
            if (polygon.size() < 3) continue;
            std::vector<std::uint32_t> knots(polygon.size());
            float area = 0.0f;
            for (std::size_t k = 0; k < polygon.size(); ++k) {
                if (!knotAt(polygon[k], knots[k])) {
                    error = "a rede: um canto de celula sem no";
                    return false;
                }
                const PanelPoint a = polygon[k], b = polygon[(k + 1) % polygon.size()];
                area += 0.5f * (a.u * b.v - b.u * a.v);
            }
            PanelCell& cell = cells[static_cast<std::size_t>(j * grid.columns + i)];
            // (Inteira, o recorte nao mexe na ordem dos cantos do retangulo.)
            const bool whole = polygon.size() == 4 && std::abs(area - cellArea) < 1e-4f * cellArea;
            std::vector<std::array<std::size_t, 3>> fan;
            if (whole) {
                if ((2 * i + 1 < grid.columns) == (j % 2 == 0)) fan = { { 0, 1, 2 }, { 0, 2, 3 } };
                else fan = { { 0, 1, 3 }, { 1, 2, 3 } };
            } else {
                for (std::size_t k = 1; k + 1 < polygon.size(); ++k) fan.push_back({ 0, k, k + 1 });
            }
            for (const auto& t : fan) {
                const PanelPoint a = polygon[t[0]], b = polygon[t[1]], c = polygon[t[2]];
                const float twice = (b.u - a.u) * (c.v - a.v) - (b.v - a.v) * (c.u - a.u);
                if (std::abs(twice) < 2e-4f * cellArea) continue;
                cell.triangles.push_back({ knots[t[0]], knots[t[1]], knots[t[2]] });
                cell.shapes.push_back({ a, b, c });
                net.faces.push_back(cell.triangles.back());
            }
        }
    return true;
}

std::array<float, 3> barycentric(PanelPoint p, const std::array<PanelPoint, 3>& t) {
    const PanelPoint a = t[0], b = t[1], c = t[2];
    const float twice = (b.u - a.u) * (c.v - a.v) - (b.v - a.v) * (c.u - a.u);
    const float wb = ((p.u - a.u) * (c.v - a.v) - (p.v - a.v) * (c.u - a.u)) / twice;
    const float wc = ((b.u - a.u) * (p.v - a.v) - (b.v - a.v) * (p.u - a.u)) / twice;
    return { 1.0f - wb - wc, wb, wc };
}

// Os pesos dos nos desenhados do pano nos nos da fisica (o no da costura
// entre dois panos fica com o primeiro): baricentrico no triangulo da
// celula que o contem - os mesmos triangulos em que a bola e os membros
// batem, entao o fio desenhado fica sempre fora deles (na bilinear, o meio
// da celula afundava na bola).
void weighPanel(const PanelGrid& drawn, const PanelGrid& grid, const std::vector<PanelCell>& cells, std::size_t knotCount,
    std::vector<ObjectNetWeight>& weights, std::vector<std::uint8_t>& weighed) {
    weights.resize(knotCount);
    weighed.resize(knotCount, 0);
    for (const auto& [p, knot] : drawn.knots) {
        if (weighed[knot] != 0) continue;
        weighed[knot] = 1;
        const float fu = (p.u - grid.uMin) / grid.du, fv = (p.v - grid.vMin) / grid.dv;
        const int i = std::clamp(static_cast<int>(std::floor(fu)), 0, grid.columns - 1);
        const int j = std::clamp(static_cast<int>(std::floor(fv)), 0, grid.rows - 1);
        const PanelCell& own = cells[static_cast<std::size_t>(j * grid.columns + i)];
        ObjectNetWeight& w = weights[knot];
        // O triangulo com o menor peso mais alto (dentro, todos >= 0; na
        // borda da celula pela conta, quase 0): o da celula, senao o das
        // vizinhas.
        float best = -1e9f;
        const auto search = [&](const PanelCell& cell) {
            for (std::size_t k = 0; k < cell.triangles.size(); ++k) {
                std::array<float, 3> b = barycentric(p, cell.shapes[k]);
                const float lowest = std::min({ b[0], b[1], b[2] });
                if (lowest <= best) continue;
                best = lowest;
                float sum = 0.0f;
                for (float& x : b) sum += (x = std::max(0.0f, x));
                w.knots = { cell.triangles[k][0], cell.triangles[k][1], cell.triangles[k][2], cell.triangles[k][0] };
                w.weights = { b[0] / sum, b[1] / sum, b[2] / sum, 0.0f };
            }
        };
        search(own);
        for (int dj = -1; dj <= 1 && best < -1e-4f; ++dj)
            for (int di = -1; di <= 1; ++di) {
                const int ci = i + di, cj = j + dj;
                if (ci >= 0 && cj >= 0 && ci < grid.columns && cj < grid.rows) search(cells[static_cast<std::size_t>(cj * grid.columns + ci)]);
            }
    }
}

// Um tubo e os cilindros de colisao dele (um por trecho do eixo, com as dobras).
bool addTube(ProceduralObject& object, const std::vector<Vec3>& path, float radius, const std::vector<float>& bends,
    const char* part, std::string& error) {
    std::vector<Vec3> axis;
    if (!buildTube(object.mesh, path, radius, TubeSides, bends, part, "metal", error, &axis)) return false;
    for (std::size_t k = 1; k < axis.size(); ++k) {
        const Vec3 a = axis[k - 1], b = axis[k];
        const float length = distanceOf(a, b);
        if (length < 1e-4f) continue;
        ObjectCollider collider;
        collider.shape = ObjectColliderShape::Cylinder;
        collider.center = (a + b) * 0.5f;
        // O z local ao longo do tubo.
        const Vec3 direction = unitOf(b - a);
        const Vec3 z { 0.0f, 0.0f, 1.0f };
        const float c = std::clamp(dot(z, direction), -1.0f, 1.0f);
        const Vec3 pivot = cross(z, direction);
        const float pivotLength = std::sqrt(pivot.lengthSquared());
        collider.orientation = pivotLength > 1e-6f ? Quaternion::fromAxisAngle(pivot * (1.0f / pivotLength), std::acos(c))
            : (c > 0.0f ? Quaternion {} : Quaternion::fromAxisAngle({ 1.0f, 0.0f, 0.0f }, 3.14159265f));
        collider.radius = radius;
        collider.halfHeight = 0.5f * length;
        object.physics.colliders.push_back(collider);
    }
    return true;
}

bool generateGoal(const ObjectRecipe& recipe, const ObjectType& type, ProceduralObject& object, std::string& error) {
    const float width = objectNumber(recipe, type, "largura");
    const float height = objectNumber(recipe, type, "altura");
    const float r = 0.5f * objectNumber(recipe, type, "diametro");
    const float rb = 0.5f * objectNumber(recipe, type, "diametro_fundo");
    const float depthTop = objectNumber(recipe, type, "fundo_alto");
    const float depthBottom = std::max(objectNumber(recipe, type, "fundo_chao"), depthTop + 0.2f);
    const float bend = objectNumber(recipe, type, "raio_dobra");
    const float corner = objectNumber(recipe, type, "raio_canto");
    const float mesh = 0.01f * objectNumber(recipe, type, "malha");
    const float twine = 0.001f * objectNumber(recipe, type, "fio");
    // A folga de cada pano (26/10, o pedido dele): o fundo 6%, o teto 2,5% (esticado entre o travessao e
    // o cabo) e as laterais 8%.
    const float slack = 0.01f * objectNumber(recipe, type, "folga");
    const float roofSlack = 0.01f * objectNumber(recipe, type, "folga_teto");
    const float sideSlack = 0.01f * objectNumber(recipe, type, "folga_lados");
    const int clipEvery = objectInteger(recipe, type, "presilhas");
    const int split = std::max(1, objectInteger(recipe, type, "malha_fisica"));

    // No espaco da trave: a origem no meio da linha do gol, no chao.
    const float halfWidth = 0.5f * width;
    const float frameY = halfWidth + r;
    const float top = height + r;
    object.mesh = {};
    object.physics = {};
    object.physics.material = recipe.material;
    object.physics.massKg = objectNumber(recipe, type, "massa");
    object.physics.fixed = true;
    // O quadro da frente (os postes no chao, o vao de dentro exato).
    if (!addTube(object, { { 0.0f, -frameY, 0.0f }, { 0.0f, -frameY, top }, { 0.0f, frameY, top }, { 0.0f, frameY, 0.0f } }, r,
            { corner, corner }, "quadro_frente", error))
        return false;
    // O quadro de tras de cada lado: a barra de cima, a dobra, a diagonal ate o
    // chao e a barra do chao de volta ao pe do poste (as pontas dentro dele).
    for (const float side : { -1.0f, 1.0f }) {
        const float y = side * frameY;
        if (!addTube(object, { { 0.0f, y, top }, { -depthTop, y, top }, { -depthBottom, y, rb }, { 0.0f, y, rb } }, rb, { bend, 0.0f },
                side < 0.0f ? "quadro_direito" : "quadro_esquerdo", error))
            return false;
    }
    if (!addTube(object, { { -depthBottom, -frameY, rb }, { -depthBottom, frameY, rb } }, rb, {}, "barra_de_tras", error)) return false;
    object.mesh.materials.front().color = objectColor(recipe, type, "cor_trave");
    // A ponta do canto de tras (a diagonal encontra a barra do chao a 115
    // graus: a meia-esquadria e comprida) descia ~2 cm abaixo do chao: fica
    // rente a ele, como a solda apoiada na grama.
    for (Vec3& p : object.mesh.positions) p.z = std::max(p.z, 0.0f);

    // A rede: um pouco para dentro dos tubos (o fio encostado neles). A da
    // fisica com a celula de `split` malhas (cada fio dela vale pelos split
    // fios paralelos: a rigidez somada; as presilhas no mesmo espacamento em
    // metros), e a de verdade dentro dela.
    ObjectNet net;
    ObjectNet real;
    net.twineDiameter = twine;
    net.color = objectColor(recipe, type, "cor_rede");
    const float stiffness = 1000.0f * objectNumber(recipe, type, "rigidez_fio");
    const float density = 0.001f * objectNumber(recipe, type, "massa_fio");
    const float cordStretch = 0.01f * objectNumber(recipe, type, "tensao_cabo");
    NetBuilder physics { net, slack, stiffness * static_cast<float>(split), density, cordStretch, {}, {} };
    NetBuilder drawn { real, slack, stiffness, density, cordStretch, {}, {} };
    const int physicsClips = std::max(1, static_cast<int>(std::lround(static_cast<float>(clipEvery) / static_cast<float>(split))));
    std::vector<ObjectNetWeight> weights;
    std::vector<std::uint8_t> weighed;
    const auto add = [&](const Panel& panel, float panelSlack) {
        physics.slack = drawn.slack = panelSlack;
        const auto [columns, rows] = panelCells(panel, mesh * static_cast<float>(split));
        const PanelGrid grid = addPanel(physics, panel, columns, rows, physicsClips);
        const PanelGrid fine = addPanel(drawn, panel, columns * split, rows * split, clipEvery);
        std::vector<PanelCell> cells;
        if (!meshPanel(net, panel, grid, cells, error)) return false;
        weighPanel(fine, grid, cells, real.knots.size(), weights, weighed);
        return true;
    };
    const float inset = rb + 0.5f * twine;
    const float sideY = frameY - inset;
    const float roofZ = top - inset;
    const float crossbarBack = -std::sqrt(std::max(0.0f, r * r - inset * inset));
    const float postBack = crossbarBack;
    const Vec3 rearBar { -depthBottom + inset, 0.0f, rb };
    // O teto: do travessao ate o cabo, entre as barras de cima.
    const float roofLength = depthTop + crossbarBack;
    if (!add({ { crossbarBack, -sideY, roofZ }, { 0.0f, 1.0f, 0.0f }, { -1.0f, 0.0f, 0.0f },
            { { 0.0f, 0.0f }, { 2.0f * sideY, 0.0f }, { 2.0f * sideY, roofLength }, { 0.0f, roofLength } },
            { { true, false }, { true, false }, { false, true }, { true, false } } }, roofSlack))
        return false;
    // O fundo: do cabo, pela diagonal, ate a barra de tras.
    const Vec3 cordStart { -depthTop, -sideY, roofZ };
    const Vec3 down = unitOf(Vec3 { rearBar.x - cordStart.x, 0.0f, rearBar.z - cordStart.z });
    const float backLength = distanceOf(Vec3 { cordStart.x, 0.0f, cordStart.z }, Vec3 { rearBar.x, 0.0f, rearBar.z });
    if (!add({ cordStart, { 0.0f, 1.0f, 0.0f }, down,
            { { 0.0f, 0.0f }, { 2.0f * sideY, 0.0f }, { 2.0f * sideY, backLength }, { 0.0f, backLength } },
            { { false, true }, { true, false }, { true, false }, { true, false } } }, slack))
        return false;
    // As laterais: do poste, pela barra de cima e pela diagonal, ate a barra
    // do chao.
    for (const float side : { -1.0f, 1.0f }) {
        if (!add({ { postBack, side * sideY, roofZ }, { -1.0f, 0.0f, 0.0f }, { 0.0f, 0.0f, -1.0f },
                { { 0.0f, 0.0f }, { depthTop + postBack, 0.0f }, { -rearBar.x + postBack, roofZ - rearBar.z }, { 0.0f, roofZ - rearBar.z } },
                { { true, false }, { true, false }, { true, false }, { true, false } } }, sideSlack))
            return false;
    }
    if (std::find(weighed.begin(), weighed.end(), std::uint8_t { 0 }) != weighed.end()) {
        error = "a rede: um no desenhado fora das celulas";
        return false;
    }
    // O peso e o fio de cada no da fisica: os da malha de verdade (a metade de
    // cada fio dela em cada ponta), repartidos pelos pesos.
    net.masses.assign(net.knots.size(), 0.0f);
    net.twineLengths.assign(net.knots.size(), 0.0f);
    std::vector<float> fineMass(real.knots.size(), 0.0f), fineTwine(real.knots.size(), 0.0f);
    for (const ObjectNetSegment& segment : real.segments) {
        const float half = 0.5f * segment.restLength;
        for (const std::uint32_t knot : { segment.a, segment.b }) {
            fineMass[knot] += half * segment.linearDensity;
            fineTwine[knot] += half;
        }
        net.drawnSegments.push_back({ segment.a, segment.b });
    }
    for (std::size_t i = 0; i < real.knots.size(); ++i)
        for (int k = 0; k < 4; ++k) {
            net.masses[weights[i].knots[k]] += weights[i].weights[k] * fineMass[i];
            net.twineLengths[weights[i].knots[k]] += weights[i].weights[k] * fineTwine[i];
        }
    net.drawnKnots = std::move(real.knots);
    net.drawnWeights = std::move(weights);
    // A escala de massa no contato (ver NetSettings3D): os nos da malha grossa
    // ja sao mais pesados. Com 4 na de 2 malhas, o chute no meio estufa 0,75 m
    // e a grade de 27 chutes 0,71 m na media, como na malha de verdade com 6;
    // com 6, a malha de 2 ficava dura no impacto (0,67 m, o alto do fundo e o
    // teto andando ~40% menos nos chutes altos).
    constexpr float ContactScales[] { 6.0f, 4.0f, 3.0f };
    net.contactMassScale = ContactScales[std::clamp(split, 1, 3) - 1];

    // A barreira dos jogadores: uma placa por pano (o teto, o fundo e os
    // lados), de 3 cm para dentro da rede a 12 cm para fora - a mao rapida (10
    // m/s: 8 cm por tick) entra menos que a metade e volta pelo lado de onde
    // veio -, ate abaixo do chao e passando das quinas. A boca fica aberta. O
    // fundo inclinado fica por cima do chao de dentro: so a capsula bate nele
    // (o boneco caido embaixo levantava com a cabeca na placa e caia de novo).
    // O corpo para numa parede reta na barra de tras, da altura da trave: por
    // dentro ele ainda afunda o alto da rede, por tras para onde ela toca o
    // chao (com um meio-fio de 35 cm, o corpo fisico empurrando passava o pe
    // por cima e saia por baixo da rede; por tras tropecava nele).
    constexpr float BarrierInside = 0.03f, BarrierOutside = 0.12f, BarrierBelow = 0.05f, BarrierOver = 0.1f;
    const auto slab = [&](std::initializer_list<Vec3> outline, Vec3 outward, bool capsuleOnly = false) {
        std::vector<Vec3>& hull = (capsuleOnly ? net.capsuleBarrier : net.barrier).emplace_back();
        for (const Vec3& p : outline) {
            hull.push_back(p - outward * BarrierInside);
            hull.push_back(p + outward * BarrierOutside);
        }
    };
    const float wideY = sideY + BarrierOutside;
    const Vec3 backTop = Vec3 { cordStart.x, 0.0f, cordStart.z } - down * BarrierOver;
    const Vec3 backFoot = rearBar + down * ((rearBar.z + BarrierBelow) / -down.z);
    slab({ { 0.0f, -wideY, roofZ }, { 0.0f, wideY, roofZ }, { -depthTop - BarrierOver, wideY, roofZ },
             { -depthTop - BarrierOver, -wideY, roofZ } },
        { 0.0f, 0.0f, 1.0f });
    slab({ backTop - Vec3 { 0.0f, wideY, 0.0f }, backTop + Vec3 { 0.0f, wideY, 0.0f }, backFoot + Vec3 { 0.0f, wideY, 0.0f },
             backFoot - Vec3 { 0.0f, wideY, 0.0f } },
        { down.z, 0.0f, -down.x }, true);
    slab({ { rearBar.x, -wideY, -BarrierBelow }, { rearBar.x, wideY, -BarrierBelow }, { rearBar.x, wideY, roofZ },
             { rearBar.x, -wideY, roofZ } },
        { -1.0f, 0.0f, 0.0f });
    // Os lados inteiros, do poste ate a parede de tras e do chao ao teto: so
    // ate a diagonal, perto do canto de tras a placa tinha ~40 cm (a capsula
    // subia nela) e por cima dela o corpo entrava no gol pelo lado.
    for (const float side : { -1.0f, 1.0f }) {
        const float y = side * sideY;
        const float back = rearBar.x - BarrierOutside;
        slab({ { 0.0f, y, -BarrierBelow }, { 0.0f, y, roofZ }, { back, y, roofZ }, { back, y, -BarrierBelow } }, { 0.0f, side, 0.0f });
    }

    // Para o espaco do objeto: a origem no meio da caixa que o contem.
    const Box3 box = object.mesh.bounds();
    const Vec3 center = box.center();
    for (Vec3& p : object.mesh.positions) p = p - center;
    for (ObjectCollider& collider : object.physics.colliders) collider.center = collider.center - center;
    for (Vec3& p : net.knots) p = p - center;
    for (Vec3& p : net.drawnKnots) p = p - center;
    for (std::vector<Vec3>& hull : net.barrier)
        for (Vec3& p : hull) p = p - center;
    for (std::vector<Vec3>& hull : net.capsuleBarrier)
        for (Vec3& p : hull) p = p - center;
    object.size = box.size();
    object.net = std::move(net);
    return true;
}

} // namespace

ObjectType goalFrameObjectType() {
    ObjectType type;
    type.id = "trave";
    type.label = "Trave";
    const auto number = [&](const char* key, const char* label, const char* group, float value, float minimum, float maximum,
                            const char* unit = "") {
        type.parameters.push_back({ key, label, group, ObjectParameterKind::Number, value, minimum, maximum, unit, {} });
    };
    const auto integer = [&](const char* key, const char* label, const char* group, float value, float minimum, float maximum,
                             const char* unit = "") {
        type.parameters.push_back({ key, label, group, ObjectParameterKind::Integer, value, minimum, maximum, unit, {} });
    };
    const auto color = [&](const char* key, const char* label, Color value) {
        type.parameters.push_back({ key, label, "Cores", ObjectParameterKind::Color, 0.0f, 0.0f, 1.0f, "", value });
    };
    number("largura", "Vão (largura)", "Forma", 7.32f, 3.0f, 8.0f, "m");
    number("altura", "Vão (altura)", "Forma", 2.44f, 1.5f, 2.6f, "m");
    number("diametro", "Tubo da frente", "Forma", 0.12f, 0.06f, 0.12f, "m");
    number("raio_canto", "Canto da frente", "Forma", 0.0f, 0.0f, 0.2f, "m");
    number("fundo_alto", "Fundo em cima", "Forma", 0.80f, 0.3f, 2.0f, "m");
    number("fundo_chao", "Fundo no chão", "Forma", 2.00f, 0.8f, 3.0f, "m");
    number("diametro_fundo", "Tubo de trás", "Forma", 0.05f, 0.03f, 0.08f, "m");
    number("raio_dobra", "Dobra de trás", "Forma", 0.10f, 0.0f, 0.3f, "m");
    number("malha", "Malha", "Rede", 12.0f, 7.0f, 15.0f, "cm");
    number("fio", "Fio", "Rede", 6.0f, 2.0f, 8.0f, "mm");
    number("folga", "Folga do fundo", "Rede", 6.0f, 0.0f, 15.0f, "%");
    number("folga_teto", "Folga do teto", "Rede", 2.5f, 0.0f, 15.0f, "%");
    number("folga_lados", "Folga dos lados", "Rede", 8.0f, 0.0f, 15.0f, "%");
    number("rigidez_fio", "Rigidez do fio", "Rede", 10.0f, 2.0f, 30.0f, "kN");
    number("massa_fio", "Peso do fio", "Rede", 7.0f, 3.0f, 15.0f, "g/m");
    // A presilha a cada 2 malhas (24 cm, o espacamento das redes de verdade) e
    // o cabo a ~150-225 N: com uma presilha por no e o cabo a 450 N o teto quase nao
    // se mexia no chute (o no que mais andava, 2-4 cm; assim, 11-14 cm).
    integer("presilhas", "Presilha a cada", "Rede", 2.0f, 1.0f, 4.0f, "malhas");
    // A celula da fisica (a malha grossa que anda; a de verdade e desenhada
    // por cima dela): com 2, a quarta parte dos nos.
    integer("malha_fisica", "Física a cada", "Rede", 2.0f, 1.0f, 3.0f, "malhas");
    // (0,75% desde 26/10, o pedido dele: o cabo entre o teto e o caimento um pouco menos dobrado.)
    number("tensao_cabo", "Cabo do alto esticado", "Rede", 0.75f, 0.1f, 3.0f, "%");
    number("massa", "Massa", "Física", 95.0f, 20.0f, 200.0f, "kg");
    color("cor_trave", "Trave", rgb(242, 242, 238));
    color("cor_rede", "Rede", rgb(236, 236, 232));
    type.generate = generateGoal;
    return type;
}

} // namespace MatterEngine::Modeling
