#include "Engine/Modeling/Parts/SurfaceDrape.hpp"

#include "Engine/Modeling/Mesh/MeshBuilder.hpp"
#include "Engine/Modeling/Mesh/MeshStitch.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <unordered_map>

namespace MatterEngine::Modeling {

namespace {

// De onde os raios descem (bem acima de qualquer peca do personagem).
constexpr float RayStart = 10.0f;
// Pontos de cada lado a lado da tira (as bordas e o meio).
constexpr int AcrossSamples = 5;

// A altura do plano do triangulo em (x, y) e se o ponto cai dentro dele
// (vista de cima); false para um triangulo de pe (sem area vista de cima).
bool planeHeight(const std::array<Vec3, 3>& t, float x, float y, float& z, bool& inside) {
    const float denominator = (t[1].y - t[2].y) * (t[0].x - t[2].x) + (t[2].x - t[1].x) * (t[0].y - t[2].y);
    if (std::abs(denominator) < 1e-12f) return false;
    const float l0 = ((t[1].y - t[2].y) * (x - t[2].x) + (t[2].x - t[1].x) * (y - t[2].y)) / denominator;
    const float l1 = ((t[2].y - t[0].y) * (x - t[2].x) + (t[0].x - t[2].x) * (y - t[2].y)) / denominator;
    const float l2 = 1.0f - l0 - l1;
    constexpr float Tolerance = 1e-5f;
    inside = l0 >= -Tolerance && l1 >= -Tolerance && l2 >= -Tolerance;
    z = l0 * t[0].z + l1 * t[1].z + l2 * t[2].z;
    return true;
}

// Quanto o triangulo de baixo da tira (strap) precisa subir para ficar a
// "clearance" acima do triangulo do campo onde os dois se sobrepoem vistos de
// cima. Os dois sao planos: o pior ponto e um vertice de um dentro do outro ou
// um cruzamento de arestas.
float deficitOver(const std::array<Vec3, 3>& strap, const std::array<Vec3, 3>& below, float clearance) {
    float worst = 0.0f;
    float z = 0.0f;
    bool inside = false;
    if (!planeHeight(below, strap[0].x, strap[0].y, z, inside)) return 0.0f;
    const auto consider = [&](float x, float y) {
        float strapZ = 0.0f, belowZ = 0.0f;
        bool unused = false;
        if (!planeHeight(strap, x, y, strapZ, unused) || !planeHeight(below, x, y, belowZ, unused)) return;
        worst = std::max(worst, belowZ + clearance - strapZ);
    };
    for (const Vec3& p : strap)
        if (planeHeight(below, p.x, p.y, z, inside) && inside) consider(p.x, p.y);
    for (const Vec3& p : below)
        if (planeHeight(strap, p.x, p.y, z, inside) && inside) consider(p.x, p.y);
    for (std::size_t i = 0; i < 3; ++i) {
        const Vec3 a = strap[i], b = strap[(i + 1) % 3];
        for (std::size_t j = 0; j < 3; ++j) {
            const Vec3 c = below[j], d = below[(j + 1) % 3];
            const float rx = b.x - a.x, ry = b.y - a.y, sx = d.x - c.x, sy = d.y - c.y;
            const float denominator = rx * sy - ry * sx;
            if (std::abs(denominator) < 1e-14f) continue;
            const float qx = c.x - a.x, qy = c.y - a.y;
            const float t = (qx * sy - qy * sx) / denominator, u = (qx * ry - qy * rx) / denominator;
            if (t < 0.0f || t > 1.0f || u < 0.0f || u > 1.0f) continue;
            consider(a.x + rx * t, a.y + ry * t);
        }
    }
    return worst;
}

// A descida da ponta que mergulha: 0 na ponta, 1 onde o mergulho comeca
// (suave nos dois lados).
float bootDiveCurve(float f) {
    const float t = std::clamp(f, 0.0f, 1.0f);
    return t * t * (3.0f - 2.0f * t);
}

// O envelope convexo de cima dos pontos (s, valor), avaliado em cada s: a
// linha de um fio esticado por cima deles.
std::vector<float> upperEnvelope(std::span<const float> along, std::span<const float> values) {
    std::vector<std::size_t> hull;
    for (std::size_t i = 0; i < along.size(); ++i) {
        while (hull.size() >= 2) {
            const std::size_t a = hull[hull.size() - 2], b = hull.back();
            const float turn = (along[b] - along[a]) * (values[i] - values[a]) - (values[b] - values[a]) * (along[i] - along[a]);
            if (turn < 0.0f) break;
            hull.pop_back();
        }
        hull.push_back(i);
    }
    std::vector<float> result(values.begin(), values.end());
    for (std::size_t h = 0; h + 1 < hull.size(); ++h) {
        const std::size_t a = hull[h], b = hull[h + 1];
        for (std::size_t i = a; i <= b; ++i) {
            const float span = along[b] - along[a];
            const float f = span > 1e-9f ? (along[i] - along[a]) / span : 0.0f;
            result[i] = std::max(values[i], values[a] + (values[b] - values[a]) * f);
        }
    }
    return result;
}

// A tira esticada lisa: a parabola que melhor acompanha o envelope (minimos
// quadrados), erguida ate passar por cima dele; sem dobra onde o envelope
// quebra. Poucos pontos: o proprio envelope.
std::vector<float> smoothArc(std::span<const float> along, std::span<const float> hull) {
    const std::size_t count = along.size();
    std::vector<float> result(hull.begin(), hull.end());
    if (count < 3) return result;
    // Normal equations da parabola a + b s + c s^2 (s centrado e em escala).
    const float center = 0.5f * (along.front() + along.back());
    const float scale = std::max(1e-6f, 0.5f * (along.back() - along.front()));
    double m[3][4] {};
    for (std::size_t i = 0; i < count; ++i) {
        const double s = (along[i] - center) / scale;
        const double powers[3] { 1.0, s, s * s };
        for (int r = 0; r < 3; ++r) {
            for (int k = 0; k < 3; ++k) m[r][k] += powers[r] * powers[k];
            m[r][3] += powers[r] * hull[i];
        }
    }
    for (int col = 0; col < 3; ++col) {
        int pivot = col;
        for (int r = col + 1; r < 3; ++r)
            if (std::abs(m[r][col]) > std::abs(m[pivot][col])) pivot = r;
        if (std::abs(m[pivot][col]) < 1e-12) return result;
        for (int k = 0; k < 4; ++k) std::swap(m[col][k], m[pivot][k]);
        for (int r = 0; r < 3; ++r) {
            if (r == col) continue;
            const double f = m[r][col] / m[col][col];
            for (int k = col; k < 4; ++k) m[r][k] -= f * m[col][k];
        }
    }
    const double a = m[0][3] / m[0][0], b = m[1][3] / m[1][1], c = m[2][3] / m[2][2];
    std::vector<float> arc(count);
    float raise = 0.0f;
    for (std::size_t i = 0; i < count; ++i) {
        const double s = (along[i] - center) / scale;
        arc[i] = static_cast<float>(a + b * s + c * s * s);
        raise = std::max(raise, hull[i] - arc[i]);
    }
    for (std::size_t i = 0; i < count; ++i) result[i] = arc[i] + raise;
    return result;
}

} // namespace

std::uint32_t LocalMesh::vertex(Vec3 position) {
    positions.push_back(position);
    return static_cast<std::uint32_t>(positions.size() - 1);
}

void LocalMesh::quad(std::uint32_t a, std::uint32_t b, std::uint32_t c, std::uint32_t d) {
    triangles.push_back({ a, b, c });
    triangles.push_back({ a, c, d });
}

void LocalMesh::append(const LocalMesh& other) {
    const std::uint32_t offset = static_cast<std::uint32_t>(positions.size());
    positions.insert(positions.end(), other.positions.begin(), other.positions.end());
    for (const Triangle& triangle : other.triangles)
        triangles.push_back({ triangle[0] + offset, triangle[1] + offset, triangle[2] + offset });
}

LocalMesh localMeshFrom(const Mesh& mesh, std::size_t firstTriangle, const Transform& toLocal) {
    LocalMesh local;
    std::unordered_map<std::uint32_t, std::uint32_t> remap;
    for (std::size_t i = firstTriangle; i < mesh.triangles.size(); ++i) {
        Triangle triangle {};
        for (std::size_t c = 0; c < 3; ++c) {
            const std::uint32_t original = mesh.triangles[i][c];
            const auto [found, inserted] = remap.emplace(original, static_cast<std::uint32_t>(local.positions.size()));
            if (inserted) local.positions.push_back(toLocal.apply(mesh.positions[original]));
            triangle[c] = found->second;
        }
        local.triangles.push_back(triangle);
    }
    return local;
}

LocalMesh localMeshOf(const Mesh& mesh, std::span<const std::uint32_t> triangles, const Transform& toLocal) {
    LocalMesh local;
    std::unordered_map<std::uint32_t, std::uint32_t> remap;
    for (const std::uint32_t t : triangles) {
        Triangle triangle {};
        for (std::size_t k = 0; k < 3; ++k) {
            const std::uint32_t v = mesh.triangles[t][k];
            const auto [it, inserted] = remap.try_emplace(v, static_cast<std::uint32_t>(local.positions.size()));
            if (inserted) local.positions.push_back(toLocal.apply(mesh.positions[v]));
            triangle[k] = it->second;
        }
        local.triangles.push_back(triangle);
    }
    return local;
}

void emitLocalMesh(MeshBuilder& builder, const Transform& frame, const LocalMesh& mesh) {
    std::vector<std::uint32_t> indices;
    indices.reserve(mesh.positions.size());
    for (const Vec3& position : mesh.positions) indices.push_back(builder.vertex(frame.apply(position)));
    for (const Triangle& triangle : mesh.triangles) builder.triangle(indices[triangle[0]], indices[triangle[1]], indices[triangle[2]]);
}

void DrapeField::addLayer(LocalMesh mesh, bool host) {
    auto layer = std::make_unique<Layer>();
    layer->mesh = std::move(mesh);
    layer->host = host;
    layer->bvh = std::make_unique<TriangleBvh>(layer->mesh.positions, layer->mesh.triangles);
    for (const Triangle& triangle : layer->mesh.triangles) {
        std::array<Point2, 2> box { { { std::numeric_limits<float>::max(), std::numeric_limits<float>::max() },
            { -std::numeric_limits<float>::max(), -std::numeric_limits<float>::max() } } };
        for (const std::uint32_t index : triangle) {
            const Vec3 p = layer->mesh.positions[index];
            box[0] = { std::min(box[0].x, p.x), std::min(box[0].y, p.y) };
            box[1] = { std::max(box[1].x, p.x), std::max(box[1].y, p.y) };
        }
        layer->bounds.push_back(box);
    }
    m_layers.push_back(std::move(layer));
}

void DrapeField::forEachTriangleNear(Point2 minimum, Point2 maximum,
    const std::function<void(const std::array<Vec3, 3>&, bool host)>& visit) const {
    for (const std::unique_ptr<Layer>& layer : m_layers)
        for (std::size_t t = 0; t < layer->mesh.triangles.size(); ++t) {
            const std::array<Point2, 2>& box = layer->bounds[t];
            if (box[1].x < minimum.x || box[0].x > maximum.x || box[1].y < minimum.y || box[0].y > maximum.y) continue;
            const Triangle& triangle = layer->mesh.triangles[t];
            visit({ layer->mesh.positions[triangle[0]], layer->mesh.positions[triangle[1]], layer->mesh.positions[triangle[2]] },
                layer->host);
        }
}

std::optional<float> DrapeField::layerHeight(const Layer& layer, float x, float y) {
    const std::optional<TriangleBvh::Hit> hit = layer.bvh->raycast({ x, y, RayStart }, { 0.0f, 0.0f, -1.0f }, 2.0f * RayStart);
    if (!hit) return std::nullopt;
    return RayStart - hit->distance;
}

std::optional<float> DrapeField::height(float x, float y) const {
    std::optional<float> best;
    for (const std::unique_ptr<Layer>& layer : m_layers) {
        const std::optional<float> h = layerHeight(*layer, x, y);
        if (h && (!best || *h > *best)) best = h;
    }
    return best;
}

std::optional<float> DrapeField::surfaceHeight(float x, float y) const {
    if (m_layers.empty()) return std::nullopt;
    return layerHeight(*m_layers.front(), x, y);
}

std::optional<float> DrapeField::hostHeight(float x, float y) const {
    std::optional<float> best;
    for (const std::unique_ptr<Layer>& layer : m_layers) {
        if (!layer->host) continue;
        const std::optional<float> h = layerHeight(*layer, x, y);
        if (h && (!best || *h > *best)) best = h;
    }
    return best;
}

std::optional<float> DrapeField::requirement(float x, float y, float hostClearance, float clearance) const {
    std::optional<float> best;
    for (const std::unique_ptr<Layer>& layer : m_layers) {
        const std::optional<float> h = layerHeight(*layer, x, y);
        if (!h) continue;
        const float need = *h + (layer->host ? hostClearance : clearance);
        if (!best || need > *best) best = need;
    }
    return best;
}

std::optional<float> heightFromBelow(const LocalMesh& mesh, float x, float y) {
    const TriangleBvh bvh(mesh.positions, mesh.triangles);
    const std::optional<TriangleBvh::Hit> hit = bvh.raycast({ x, y, -RayStart }, { 0.0f, 0.0f, 1.0f }, 2.0f * RayStart);
    if (!hit) return std::nullopt;
    return hit->distance - RayStart;
}

std::vector<Point2> sampleLine(Point2 a, Point2 b, float step) {
    const float length = std::hypot(b.x - a.x, b.y - a.y);
    const int count = std::max(2, static_cast<int>(std::ceil(length / std::max(1e-4f, step))));
    std::vector<Point2> points;
    for (int i = 0; i <= count; ++i) {
        const float f = static_cast<float>(i) / static_cast<float>(count);
        points.push_back({ a.x + (b.x - a.x) * f, a.y + (b.y - a.y) * f });
    }
    return points;
}

std::vector<Point2> sampleLine(Point2 a, Point2 b, float step, float endStep, float endLength) {
    const float length = std::hypot(b.x - a.x, b.y - a.y);
    if (length <= 2.0f * endLength || endStep <= 0.0f) return sampleLine(a, b, std::min(step, endStep));
    const auto at = [&](float s) { return Point2 { a.x + (b.x - a.x) * s / length, a.y + (b.y - a.y) * s / length }; };
    std::vector<Point2> points;
    const int ends = std::max(1, static_cast<int>(std::ceil(endLength / endStep)));
    for (int i = 0; i < ends; ++i) points.push_back(at(endLength * static_cast<float>(i) / static_cast<float>(ends)));
    const float middle = length - 2.0f * endLength;
    const int inner = std::max(1, static_cast<int>(std::ceil(middle / step)));
    for (int i = 0; i < inner; ++i) points.push_back(at(endLength + middle * static_cast<float>(i) / static_cast<float>(inner)));
    for (int i = 0; i <= ends; ++i) points.push_back(at(length - endLength + endLength * static_cast<float>(i) / static_cast<float>(ends)));
    return points;
}

float surfaceClearanceDeficit(const std::array<Vec3, 3>& face, const DrapeField& field, float clearance) {
    const Point2 minimum { std::min({ face[0].x, face[1].x, face[2].x }), std::min({ face[0].y, face[1].y, face[2].y }) };
    const Point2 maximum { std::max({ face[0].x, face[1].x, face[2].x }), std::max({ face[0].y, face[1].y, face[2].y }) };
    float lift = 0.0f;
    field.forEachTriangleNear(minimum, maximum,
        [&](const std::array<Vec3, 3>& below, bool) { lift = std::max(lift, deficitOver(face, below, clearance)); });
    return lift;
}

float liftAboveSurface(LocalMesh& mesh, const DrapeField& field, float clearance) {
    float lift = 0.0f;
    for (const Triangle& t : mesh.triangles) {
        const std::array<Vec3, 3> face { mesh.positions[t[0]], mesh.positions[t[1]], mesh.positions[t[2]] };
        if (cross(face[1] - face[0], face[2] - face[0]).z >= -1e-10f) continue;
        lift = std::max(lift, surfaceClearanceDeficit(face, field, clearance));
    }
    if (lift > 0.0f)
        for (Vec3& position : mesh.positions) position.z += lift + 1e-5f;
    return lift;
}

namespace {

// A tira antes de virar malha: o centro e o lado (com a inclinacao) de cada
// anel, o comprimento ate ele e as medidas da secao.
struct StrapFrames {
    std::vector<Vec3> center;
    std::vector<Vec3> side;
    std::vector<float> along;
    // A meia largura de cada anel (afina no mergulho, StrapSpec::endWidth).
    std::vector<float> halfWidth;
    float thick = 0.0f;

    // Os cantos do anel i: 0 baixo-esquerda, 1 cima-esquerda, 2 cima-direita,
    // 3 baixo-direita (esquerda de quem anda pelo caminho).
    [[nodiscard]] Vec3 corner(std::size_t i, int k) const {
        const Vec3 up { 0.0f, 0.0f, thick };
        const Vec3 across = side[i] * halfWidth[i];
        switch (k) {
        case 0: return center[i] + across - up;
        case 1: return center[i] + across + up;
        case 2: return center[i] - across + up;
        default: return center[i] - across - up;
        }
    }
};

StrapFrames strapFrames(const DrapeField& field, std::span<const Point2> path, const StrapSpec& spec) {
    StrapFrames result;
    const std::size_t count = path.size();
    if (count < 2) return result;
    const float half = spec.width * 0.5f;
    std::vector<float> center(count), slope(count), along(count);
    std::vector<Point2> sides(count);
    for (std::size_t i = 0; i < count; ++i) {
        along[i] = i == 0 ? 0.0f : along[i - 1] + std::hypot(path[i].x - path[i - 1].x, path[i].y - path[i - 1].y);
        const Point2 a = path[i > 0 ? i - 1 : i], b = path[i + 1 < count ? i + 1 : i];
        const float run = std::max(1e-9f, std::hypot(b.x - a.x, b.y - a.y));
        sides[i] = { -(b.y - a.y) / run, (b.x - a.x) / run };
        const auto at = [&](float s) { return Point2 { path[i].x + sides[i].x * s, path[i].y + sides[i].y * s }; };
        // A inclinacao de lado vem da superficie, nas bordas da tira.
        const Point2 left = at(half), right = at(-half);
        const std::optional<float> leftHeight = field.surfaceHeight(left.x, left.y);
        const std::optional<float> rightHeight = field.surfaceHeight(right.x, right.y);
        slope[i] = leftHeight && rightHeight ? (*leftHeight - *rightHeight) / spec.width : 0.0f;
        // A base: o plano inclinado que passa por cima de tudo embaixo da tira.
        float base = -std::numeric_limits<float>::max();
        for (int j = 0; j < AcrossSamples; ++j) {
            const float s = -half + spec.width * static_cast<float>(j) / static_cast<float>(AcrossSamples - 1);
            const Point2 q = at(s);
            if (const std::optional<float> h = field.requirement(q.x, q.y, spec.hostClearance, spec.clearance))
                base = std::max(base, *h - slope[i] * s);
        }
        if (base == -std::numeric_limits<float>::max()) base = 0.0f;
        center[i] = base + spec.thickness * 0.5f;
    }
    // O apoio de cada anel (deitado em tudo o que tem embaixo): para onde as
    // pontas descem.
    std::vector<float> rest = center;
    if (spec.taut) {
        // Uma tira esticada tem a mesma inclinacao transversal de ponta a
        // ponta. A mediana descarta o salto das bordas da lingua/ilhoses.
        std::vector<float> sorted = slope;
        std::sort(sorted.begin(), sorted.end());
        const float tilt = sorted[sorted.size() / 2];
        std::fill(slope.begin(), slope.end(), tilt);
        for (std::size_t i = 0; i < count; ++i) {
            float base = -std::numeric_limits<float>::max();
            for (int j = 0; j < AcrossSamples; ++j) {
                const float s = -half + spec.width * static_cast<float>(j) / static_cast<float>(AcrossSamples - 1);
                if (const auto h = field.requirement(path[i].x + sides[i].x * s, path[i].y + sides[i].y * s,
                        spec.hostClearance, spec.clearance))
                    base = std::max(base, *h - tilt * s);
            }
            center[i] = (base == -std::numeric_limits<float>::max() ? 0.0f : base) + spec.thickness * 0.5f;
        }
        rest = center;
        // As pontas que mergulham ficam fora do arco (ele passa por cima do
        // meio); depois elas descem ate a base.
        std::vector<float> arcAlong, arcCenter;
        const float total = along.back();
        for (std::size_t i = 0; i < count; ++i)
            if (spec.dive <= 0.0f || (along[i] >= spec.dive && along[i] <= total - spec.dive)) {
                arcAlong.push_back(along[i]);
                arcCenter.push_back(center[i]);
            }
        if (arcAlong.size() >= 2) {
            const std::vector<float> arc = smoothArc(arcAlong, upperEnvelope(arcAlong, arcCenter));
            // O arco vale no meio e continua reto ate as pontas.
            for (std::size_t i = 0; i < count; ++i) {
                const float s = std::clamp(along[i], arcAlong.front(), arcAlong.back());
                std::size_t j = 0;
                while (j + 2 < arcAlong.size() && arcAlong[j + 1] < s) ++j;
                const float span = std::max(1e-9f, arcAlong[j + 1] - arcAlong[j]);
                const float f = std::clamp((s - arcAlong[j]) / span, 0.0f, 1.0f);
                center[i] = std::max(center[i], arc[j] + (arc[j + 1] - arc[j]) * f);
            }
        }
    }
    // As pontas descem ate o apoio (menos diveDepth): suave no comeco e cada
    // vez mais inclinada.
    if (spec.dive > 0.0f)
        for (std::size_t i = 0; i < count; ++i) {
            const float distance = std::min(along[i], along.back() - along[i]);
            if (distance >= spec.dive) continue;
            const float f = (1.0f - distance / spec.dive) * (1.0f - distance / spec.dive);
            center[i] = center[i] + (rest[i] - spec.diveDepth - center[i]) * f;
        }

    // Os aneis: x na tangente, y no lado (com a inclinacao), z na normal.
    const float thick = spec.thickness * 0.5f;
    struct RingFrame {
        Vec3 center;
        Vec3 side;
        Vec3 normal;
    };
    const auto frames = [&] {
        std::vector<RingFrame> result(count);
        for (std::size_t i = 0; i < count; ++i) {
            // Parametrizacao por altura: a projecao XY nunca muda quando a
            // folga levanta um anel. Extrudar pela normal inclinada deslocava
            // os cantos em XY e dobrava a propria tira nas bordas da lingua.
            result[i] = { { path[i].x, path[i].y, center[i] },
                { sides[i].x, sides[i].y, slope[i] }, { 0.0f, 0.0f, 1.0f } };
        }
        return result;
    };
    // A conferencia exata: cada triangulo da face de baixo contra os
    // triangulos do campo embaixo dele (quinas e degraus entre as amostras
    // levantam a tira ate a folga).
    std::vector<RingFrame> ringFrames = frames();
    const auto bottom = [&](std::size_t i, float across) {
        const RingFrame& f = ringFrames[i];
        return f.center + f.side * (across * half) - f.normal * thick;
    };
    for (int pass = 0; pass < 6; ++pass) {
        std::vector<float> lift(count, 0.0f);
        bool lifted = false;
        // As pontas que mergulham entram na base de proposito: os aneis que
        // ja afundaram (o fundo abaixo da base) nao sobem; os de cima, sim.
        std::vector<bool> plunged(count, false);
        if (spec.dive > 0.0f)
            for (std::size_t i = 0; i < count; ++i) {
                if (std::min(along[i], along.back() - along[i]) >= spec.dive) continue;
                for (const float across : { -1.0f, 1.0f }) {
                    const Vec3 b = bottom(i, across);
                    const std::optional<float> host = field.hostHeight(b.x, b.y);
                    if (host && b.z <= *host) plunged[i] = true;
                }
            }
        for (std::size_t i = 0; i + 1 < count; ++i) {
            if (plunged[i] || plunged[i + 1]) continue;
            // Os dois triangulos como a malha corta a face (a, b, c) e (a, c, d).
            const Vec3 a = bottom(i, -1.0f), b = bottom(i, 1.0f), c = bottom(i + 1, 1.0f), d = bottom(i + 1, -1.0f);
            for (const std::array<Vec3, 3>& strap : { std::array<Vec3, 3> { a, b, c }, std::array<Vec3, 3> { a, c, d } }) {
                const Point2 minimum { std::min({ strap[0].x, strap[1].x, strap[2].x }), std::min({ strap[0].y, strap[1].y, strap[2].y }) };
                const Point2 maximum { std::max({ strap[0].x, strap[1].x, strap[2].x }), std::max({ strap[0].y, strap[1].y, strap[2].y }) };
                float deficit = 0.0f;
                field.forEachTriangleNear(minimum, maximum, [&](const std::array<Vec3, 3>& below, bool host) {
                    deficit = std::max(deficit, deficitOver(strap, below, host ? spec.hostClearance : spec.clearance));
                });
                if (deficit <= 0.0f) continue;
                lift[i] = std::max(lift[i], deficit + 1e-5f);
                lift[i + 1] = std::max(lift[i + 1], deficit + 1e-5f);
                lifted = true;
            }
        }
        if (!lifted) break;
        for (std::size_t i = 0; i < count; ++i) center[i] += lift[i];
        ringFrames = frames();
    }

    result.thick = thick;
    result.along = along;
    for (std::size_t i = 0; i < count; ++i) {
        result.center.push_back(ringFrames[i].center);
        result.side.push_back(ringFrames[i].side);
        // A ponta afina ao longo do mergulho (so o desenho: a altura ja foi
        // decidida com a largura cheia, por cima).
        float width = spec.width;
        if (spec.endWidth > 0.0f && spec.dive > 0.0f) {
            const float distance = std::min(along[i], along.back() - along[i]);
            width += (spec.endWidth - spec.width) * (1.0f - std::clamp(distance / spec.dive, 0.0f, 1.0f));
        }
        result.halfWidth.push_back(width * 0.5f);
    }
    return result;
}

} // namespace

LocalMesh drapeStrap(const DrapeField& field, std::span<const Point2> path, const StrapSpec& spec) {
    LocalMesh mesh;
    const StrapFrames frames = strapFrames(field, path, spec);
    if (frames.center.size() < 2) return mesh;
    // A secao anti-horaria vista da frente (as faces para fora).
    std::vector<std::array<std::uint32_t, 4>> rings;
    for (std::size_t i = 0; i < frames.center.size(); ++i)
        rings.push_back({ mesh.vertex(frames.corner(i, 0)), mesh.vertex(frames.corner(i, 1)), mesh.vertex(frames.corner(i, 2)),
            mesh.vertex(frames.corner(i, 3)) });
    for (std::size_t i = 0; i + 1 < rings.size(); ++i)
        for (std::size_t k = 0; k < 4; ++k) {
            const std::size_t next = (k + 1) % 4;
            // Mesma diagonal XY em cima e embaixo. Diagonais opostas em uma
            // sela podiam se atravessar, apesar da espessura nos vertices.
            if (k == 1) mesh.quad(rings[i][next], rings[i + 1][next], rings[i + 1][k], rings[i][k]);
            else mesh.quad(rings[i][k], rings[i][next], rings[i + 1][next], rings[i + 1][k]);
        }
    const auto& first = rings.front();
    const auto& last = rings.back();
    mesh.quad(first[3], first[2], first[1], first[0]);
    mesh.quad(last[0], last[1], last[2], last[3]);
    return mesh;
}

WeldedStrap drapeWeldedStrap(const DrapeField& field, std::span<const Point2> path, const StrapSpec& spec,
    float endLength, float endLift) {
    WeldedStrap strap;
    // As pontas descem ate "endLift" acima do apoio (a parede de baixo do
    // furo tem altura para subir sem raspar na superficie).
    StrapSpec lifted = spec;
    lifted.diveDepth = spec.diveDepth - endLift;
    const StrapFrames frames = strapFrames(field, path, lifted);
    const std::size_t count = frames.center.size();
    if (count < 2) return strap;
    const auto onSurface = [&](Vec3 p) {
        const std::optional<float> h = field.hostHeight(p.x, p.y);
        if (h) p.z = *h;
        return p;
    };
    // A ponta: o fundo do ultimo anel desce ate a base um pouco adiante
    // (um quarto do furo) e o topo inclina ate ela "endLength" adiante: o
    // furo e esse retangulo.
    const auto hole = [&](std::size_t ring, std::size_t inner) {
        Vec3 out = frames.center[ring] - frames.center[inner];
        out = safeNormalized({ out.x, out.y, 0.0f }, {});
        std::array<Vec3, 4> corners {};
        corners[0] = onSurface(frames.corner(ring, 0) + out * (endLength * 0.25f));
        corners[1] = onSurface(frames.corner(ring, 1) + out * endLength);
        corners[2] = onSurface(frames.corner(ring, 2) + out * endLength);
        corners[3] = onSurface(frames.corner(ring, 3) + out * (endLength * 0.25f));
        return corners;
    };
    strap.startHole = hole(0, 1);
    strap.endHole = hole(count - 1, count - 2);
    for (std::size_t i = 0; i < count; ++i)
        strap.rings.push_back({ frames.corner(i, 0), frames.corner(i, 1), frames.corner(i, 2), frames.corner(i, 3) });
    // O fundo das pontas precisa estar acima da base (a parede de baixo do
    // furo sobe dela).
    for (const std::size_t ring : { std::size_t { 0 }, count - 1 })
        for (const int k : { 0, 3 }) {
            const Vec3 c = frames.corner(ring, k);
            if (c.z <= onSurface(c).z) return strap;
        }
    strap.ok = true;
    return strap;
}

LocalMesh weldedStrapMesh(const WeldedStrap& strap) {
    LocalMesh mesh;
    if (!strap.ok) return mesh;
    std::vector<std::array<std::uint32_t, 4>> rings;
    const auto add = [&](const std::array<Vec3, 4>& corners) {
        rings.push_back({ mesh.vertex(corners[0]), mesh.vertex(corners[1]), mesh.vertex(corners[2]), mesh.vertex(corners[3]) });
    };
    add(strap.startHole);
    for (const auto& ring : strap.rings) add(ring);
    add(strap.endHole);
    for (std::size_t i = 0; i + 1 < rings.size(); ++i)
        strapSegment(rings[i], rings[i + 1],
            [&](std::uint32_t a, std::uint32_t b, std::uint32_t c) { mesh.triangles.push_back({ a, b, c }); });
    return mesh;
}

namespace {

// Os trechos de uma volta recortada entre os 4 cantos dados (cada um do
// canto k ao k+1, com os cortes do meio).
std::array<std::vector<std::uint32_t>, 4> cornerSegments(std::span<const std::uint32_t> loop,
    std::span<const std::uint32_t> corners) {
    std::array<std::size_t, 4> at {};
    std::size_t search = 0;
    for (std::size_t k = 0; k < 4; ++k) {
        for (std::size_t step = 0; step < loop.size(); ++step) {
            const std::size_t index = (search + step) % loop.size();
            if (loop[index] == corners[k]) {
                search = index;
                break;
            }
        }
        at[k] = search;
    }
    std::array<std::vector<std::uint32_t>, 4> segments;
    for (std::size_t k = 0; k < 4; ++k) {
        segments[k].push_back(loop[at[k]]);
        for (std::size_t index = at[k]; index != at[(k + 1) % 4];) {
            index = (index + 1) % loop.size();
            segments[k].push_back(loop[index]);
        }
    }
    return segments;
}

// A aresta de baixo do anel (do canto 3 ao 0) dividida acima de cada corte
// do fundo do furo (o trecho do canto 3 ao 0 da volta).
std::vector<std::uint32_t> splitBottomEdge(MeshBuilder& builder, const std::array<std::uint32_t, 4>& ring,
    const std::vector<std::uint32_t>& bottom) {
    const Mesh& mesh = builder.mesh();
    const Vec3 a = mesh.positions[ring[3]], b = mesh.positions[ring[0]];
    const Vec3 edge = b - a;
    const float length = std::max(1e-12f, edge.lengthSquared());
    std::vector<std::uint32_t> result { ring[3] };
    float last = 0.0f;
    for (std::size_t i = 1; i + 1 < bottom.size(); ++i) {
        const float t = std::clamp(dot(builder.mesh().positions[bottom[i]] - a, edge) / length, 0.0f, 1.0f);
        if (t <= last + 1e-3f || t >= 1.0f - 1e-3f) continue;
        last = t;
        result.push_back(builder.vertex(a + edge * t));
    }
    result.push_back(ring[0]);
    return result;
}

} // namespace

void emitWeldedStrap(MeshBuilder& builder, const Transform& frame, const WeldedStrap& strap,
    std::span<const std::uint32_t> startLoop, std::span<const std::uint32_t> startCorners,
    std::span<const std::uint32_t> endLoop, std::span<const std::uint32_t> endCorners) {
    if (!strap.ok || strap.rings.empty()) return;
    std::vector<std::array<std::uint32_t, 4>> rings;
    for (const auto& ring : strap.rings)
        rings.push_back({ builder.vertex(frame.apply(ring[0])), builder.vertex(frame.apply(ring[1])),
            builder.vertex(frame.apply(ring[2])), builder.vertex(frame.apply(ring[3])) });
    const auto start = cornerSegments(startLoop, startCorners);
    const auto end = cornerSegments(endLoop, endCorners);
    const std::array<std::uint32_t, 4>& first = rings.front();
    const std::array<std::uint32_t, 4>& last = rings.back();
    // O furo de onde a tira sai (antes do primeiro anel).
    stitchStrips(builder, start[0], std::vector<std::uint32_t> { first[0], first[1] }, StripOrder::ToFirst);
    stitchStrips(builder, start[1], std::vector<std::uint32_t> { first[1], first[2] }, StripOrder::Shortest);
    stitchStrips(builder, start[2], std::vector<std::uint32_t> { first[2], first[3] }, StripOrder::FromFirst);
    const std::vector<std::uint32_t> startBottom = splitBottomEdge(builder, first, start[3]);
    stitchStrips(builder, start[3], startBottom);
    // O furo onde ela entra (depois do ultimo anel).
    stitchStrips(builder, std::vector<std::uint32_t> { last[0], last[1] }, end[0], StripOrder::FromFirst);
    stitchStrips(builder, std::vector<std::uint32_t> { last[1], last[2] }, end[1], StripOrder::Shortest);
    stitchStrips(builder, std::vector<std::uint32_t> { last[2], last[3] }, end[2], StripOrder::ToFirst);
    const std::vector<std::uint32_t> endBottom = splitBottomEdge(builder, last, end[3]);
    stitchStrips(builder, endBottom, end[3]);
    // O corpo: o fundo dos trechos das pontas vem das arestas divididas.
    for (std::size_t i = 0; i + 1 < rings.size(); ++i) {
        const auto& a = rings[i];
        const auto& b = rings[i + 1];
        bool bottomDone = false;
        if (i == 0 || i + 2 == rings.size()) {
            const std::vector<std::uint32_t> from = i == 0 ? startBottom : std::vector<std::uint32_t> { a[3], a[0] };
            const std::vector<std::uint32_t> to = i + 2 == rings.size() ? endBottom : std::vector<std::uint32_t> { b[3], b[0] };
            stitchStrips(builder, from, to);
            bottomDone = true;
        }
        strapSegment(a, b, [&](std::uint32_t x, std::uint32_t y, std::uint32_t z) {
            // A face de baixo (cantos 3 e 0) ja saiu dividida.
            const bool bottomFace = (x == a[3] || x == a[0] || x == b[3] || x == b[0]) && (y == a[3] || y == a[0] || y == b[3] || y == b[0])
                && (z == a[3] || z == a[0] || z == b[3] || z == b[0]);
            if (bottomDone && bottomFace) return;
            builder.triangle(x, y, z);
        });
    }
    if (rings.size() == 1) stitchStrips(builder, startBottom, endBottom);
}

ReliefPiece transformedRelief(const ReliefPiece& piece, const Transform& frame) {
    ReliefPiece result = piece;
    for (Vec3& p : result.footprint) p = frame.apply(p);
    for (Vec3& p : result.topPositions) p = frame.apply(p);
    return result;
}

} // namespace MatterEngine::Modeling
