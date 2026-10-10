#include "Engine/Modeling/Paint/PaintSpace.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <unordered_map>

namespace MatterEngine::Modeling {

namespace {

Vec3 mirrored(Vec3 v) { return { v.x, -v.y, v.z }; }

std::vector<bool> partMask(const Mesh& mesh, const std::vector<int>& parts) {
    std::vector<bool> mask(mesh.parts.size(), false);
    for (const int part : parts)
        if (part >= 0 && static_cast<std::size_t>(part) < mask.size()) mask[static_cast<std::size_t>(part)] = true;
    return mask;
}

int sideOf(Vec3 position) { return position.y > 0.0f ? 1 : -1; }

// O arco levado para dentro de meia volta da costura (sem volta, igual).
float wrapped(float s, float period) {
    if (period <= 0.0f) return s;
    while (s > 0.5f * period) s -= period;
    while (s < -0.5f * period) s += period;
    return s;
}

// Uma metade da volta desenrolada: as fatias (cada trecho com o triangulo que
// o gerou e o arco ate ele na corrente dele) e a costura.
class HalfUnrolled {
public:
    HalfUnrolled(const Mesh& mesh, const UnrolledSpaceSettings& settings, int side);
    [[nodiscard]] std::optional<Point2> locate(const SurfaceSample& sample) const;
    [[nodiscard]] float axisMinimum() const { return m_t0; }
    [[nodiscard]] float axisMaximum() const { return m_t0 + m_step * static_cast<float>(std::max<std::size_t>(1, m_levels.size()) - 1); }

private:
    struct Segment {
        std::uint32_t triangle = 0;
        std::uint32_t chain = 0;
        Point2 a;
        Point2 b;
        float startArc = 0.0f;
        float length = 0.0f;
    };
    struct Chain {
        float seamArc = 0.0f;
        float direction = 1.0f;
        float length = 0.0f;
        bool closed = false;
        // Os trechos dela (seguidos em level.segments, na ordem do arco).
        std::uint32_t first = 0;
        std::uint32_t count = 0;
        // A volta (o arco da costura da de volta nela; 0 sem volta): a
        // corrente fechada, ou aberta numa brecha pequena (a fatia na barra
        // pega o recorte dela) - ai a volta e a corrente mais a brecha.
        float period = 0.0f;
        // A corrente cruza a costura (sem cruzar - um pedaco partido pela
        // cava da manga -, nao tem coordenada).
        bool seam = false;
    };
    struct Level {
        std::vector<Segment> segments;
        std::vector<Chain> chains;
        std::unordered_map<std::uint32_t, std::uint32_t> byTriangle;
    };

    const Mesh* m_mesh = nullptr;
    Vec3 m_origin, m_axis, m_front, m_outward;
    float m_t0 = 0.0f;
    float m_step = 0.002f;
    std::vector<Level> m_levels;
    std::vector<bool> m_triangles;

    [[nodiscard]] Point2 project(Vec3 p) const { return { dot(p - m_origin, m_front), dot(p - m_origin, m_outward) }; }
    // O arco num nivel e a volta da corrente dele (0 sem volta).
    struct Arc {
        float s = 0.0f;
        float period = 0.0f;
    };
    // "own": so o trecho do proprio triangulo da amostra.
    [[nodiscard]] std::optional<Arc> arcAt(std::size_t level, std::uint32_t triangle, Point2 q, bool own) const;
};

HalfUnrolled::HalfUnrolled(const Mesh& mesh, const UnrolledSpaceSettings& settings, int side) : m_mesh(&mesh) {
    const bool left = side > 0;
    m_origin = left ? mirrored(settings.origin) : settings.origin;
    m_axis = safeNormalized(left ? mirrored(settings.axis) : settings.axis, { 0.0f, 0.0f, 1.0f });
    // A frente e o lado de fora no plano da fatia.
    const Vec3 front = left ? mirrored(settings.front) : settings.front, outward = left ? mirrored(settings.outward) : settings.outward;
    m_front = safeNormalized(front - m_axis * dot(front, m_axis), { 1.0f, 0.0f, 0.0f });
    m_outward = safeNormalized(outward - m_axis * dot(outward, m_axis) - m_front * dot(outward, m_front), { 0.0f, -1.0f, 0.0f });
    m_step = std::max(1e-4f, settings.step);
    const std::vector<bool> parts = partMask(mesh, settings.parts);
    m_triangles.assign(mesh.triangles.size(), false);
    float low = std::numeric_limits<float>::max(), high = std::numeric_limits<float>::lowest();
    for (std::size_t t = 0; t < mesh.triangles.size(); ++t) {
        const int part = mesh.partOf(t);
        if (part < 0 || static_cast<std::size_t>(part) >= parts.size() || !parts[static_cast<std::size_t>(part)]) continue;
        const Vec3 c = mesh.center(t);
        if (sideOf(c) != side || (settings.include && !settings.include(left ? mirrored(c) : c))) continue;
        const Vec3 normal = mesh.normal(t);
        if (std::abs(dot(normal, m_axis)) > settings.maxAxisFacing) continue;
        // (So a superficie de fora: a normal para longe do eixo.)
        const Vec3 radial = c - m_origin - m_axis * dot(c - m_origin, m_axis);
        if (settings.outerOnly && dot(normal - m_axis * dot(normal, m_axis), radial) < 0.0f) continue;
        m_triangles[t] = true;
        for (const std::uint32_t v : mesh.triangles[t]) {
            const float along = dot(mesh.positions[v] - m_origin, m_axis);
            low = std::min(low, along);
            high = std::max(high, along);
        }
    }
    if (low > high) return;
    m_t0 = low;
    const std::size_t count = static_cast<std::size_t>(std::ceil((high - low) / m_step)) + 1;
    m_levels.resize(count);
    // A distancia de cada vertice ao plano de cada nivel sai do valor no eixo.
    std::vector<float> along(mesh.positions.size(), 0.0f);
    for (std::size_t v = 0; v < mesh.positions.size(); ++v) along[v] = dot(mesh.positions[v] - m_origin, m_axis);
    std::vector<float> seamT, seamX;
    for (std::size_t k = 0; k < count; ++k) {
        const float t = m_t0 + m_step * static_cast<float>(k);
        Level& level = m_levels[k];
        // Os trechos: cada triangulo que cruza o plano, entre as duas arestas
        // cruzadas (a chave de cada ponto e a aresta).
        struct Piece {
            std::uint64_t keyA, keyB;
            Point2 a, b;
            std::uint32_t triangle;
        };
        std::vector<Piece> pieces;
        for (std::size_t tri = 0; tri < mesh.triangles.size(); ++tri) {
            if (!m_triangles[tri]) continue;
            const Triangle& T = mesh.triangles[tri];
            float d[3];
            for (int i = 0; i < 3; ++i) {
                d[i] = along[T[static_cast<std::size_t>(i)]] - t;
                if (std::abs(d[i]) < 1e-9f) d[i] = 1e-9f;
            }
            std::uint64_t keys[2];
            Point2 points[2];
            int found = 0;
            for (int i = 0; i < 3 && found < 2; ++i) {
                const int j = (i + 1) % 3;
                if ((d[i] > 0.0f) == (d[j] > 0.0f)) continue;
                std::uint32_t va = T[static_cast<std::size_t>(i)], vb = T[static_cast<std::size_t>(j)];
                float da = d[i], db = d[j];
                if (va > vb) {
                    std::swap(va, vb);
                    std::swap(da, db);
                }
                const Vec3 p = mesh.positions[va] + (mesh.positions[vb] - mesh.positions[va]) * (da / (da - db));
                keys[found] = (static_cast<std::uint64_t>(va) << 32) | vb;
                points[found] = project(p);
                ++found;
            }
            if (found == 2) pieces.push_back({ keys[0], keys[1], points[0], points[1], static_cast<std::uint32_t>(tri) });
        }
        // As correntes: os trechos ligados pelas arestas.
        std::unordered_map<std::uint64_t, std::vector<std::uint32_t>> byKey;
        for (std::uint32_t i = 0; i < pieces.size(); ++i) {
            byKey[pieces[i].keyA].push_back(i);
            byKey[pieces[i].keyB].push_back(i);
        }
        std::vector<bool> used(pieces.size(), false);
        const auto walk = [&](std::uint32_t first, std::uint64_t startKey) {
            Chain chain;
            chain.first = static_cast<std::uint32_t>(level.segments.size());
            const std::uint32_t chainIndex = static_cast<std::uint32_t>(level.chains.size());
            std::uint32_t current = first;
            std::uint64_t entry = startKey;
            float arc = 0.0f;
            while (true) {
                used[current] = true;
                const Piece& piece = pieces[current];
                const bool forward = piece.keyA == entry;
                const Point2 a = forward ? piece.a : piece.b, b = forward ? piece.b : piece.a;
                const std::uint64_t exit = forward ? piece.keyB : piece.keyA;
                const float length = std::hypot(b.x - a.x, b.y - a.y);
                level.segments.push_back({ piece.triangle, chainIndex, a, b, arc, length });
                arc += length;
                std::uint32_t next = current;
                for (const std::uint32_t candidate : byKey[exit])
                    if (!used[candidate]) next = candidate;
                if (next == current) {
                    chain.closed = exit == startKey;
                    break;
                }
                current = next;
                entry = exit;
            }
            chain.length = arc;
            chain.count = static_cast<std::uint32_t>(level.segments.size()) - chain.first;
            const Point2 head = level.segments[chain.first].a, tail = level.segments.back().b;
            const float gap = std::hypot(tail.x - head.x, tail.y - head.y);
            if (chain.closed) chain.period = arc;
            else if (gap < 0.15f * arc) chain.period = arc + gap;
            level.chains.push_back(chain);
        };
        for (std::uint32_t i = 0; i < pieces.size(); ++i)
            if (!used[i]) {
                const bool openA = byKey[pieces[i].keyA].size() == 1, openB = byKey[pieces[i].keyB].size() == 1;
                if (openA) walk(i, pieces[i].keyA);
                else if (openB) walk(i, pieces[i].keyB);
            }
        for (std::uint32_t i = 0; i < pieces.size(); ++i)
            if (!used[i]) walk(i, pieces[i].keyA);
        // A costura bruta: o ponto mais para fora da corrente mais comprida.
        if (t >= settings.seamFrom && t <= settings.seamTo && !level.chains.empty()) {
            std::uint32_t longest = 0;
            for (std::uint32_t c = 1; c < level.chains.size(); ++c)
                if (level.chains[c].length > level.chains[longest].length) longest = c;
            // (Empate - uma face de frente para fora -: o meio dela.)
            float best = std::numeric_limits<float>::lowest();
            for (const Segment& s : level.segments)
                if (s.chain == longest) best = std::max({ best, s.a.y, s.b.y });
            float x = 0.0f, n = 0.0f;
            for (const Segment& s : level.segments)
                if (s.chain == longest)
                    for (const Point2 p : { s.a, s.b })
                        if (p.y > best - 1e-5f) {
                            x += p.x;
                            n += 1.0f;
                        }
            seamT.push_back(t);
            seamX.push_back(x / std::max(1.0f, n));
        }
    }
    // A costura: a reta (frente por nivel) que passa mais perto.
    float base = 0.0f, slope = 0.0f;
    if (!seamT.empty()) {
        double st = 0.0, sx = 0.0, stt = 0.0, stx = 0.0;
        const double n = static_cast<double>(seamT.size());
        for (std::size_t i = 0; i < seamT.size(); ++i) {
            st += seamT[i];
            sx += seamX[i];
            stt += static_cast<double>(seamT[i]) * seamT[i];
            stx += static_cast<double>(seamT[i]) * seamX[i];
        }
        const double denominator = n * stt - st * st;
        slope = std::abs(denominator) > 1e-12 ? static_cast<float>((n * stx - st * sx) / denominator) : 0.0f;
        base = static_cast<float>((sx - slope * st) / n);
    }
    // Em cada corrente, o ponto da costura e o sentido do arco para a frente.
    // A reta da costura pode cruzar o lado de fora mais de uma vez (o pano
    // ondula): a costura e seguida de nivel em nivel - em cada um, o
    // cruzamento mais perto do escolhido no vizinho, comecando no meio do
    // trecho ajustado pelo mais para fora. Sem cruzar, a corrente fica sem
    // coordenada.
    struct Crossing {
        Point2 at;
        float arc = 0.0f;
        float direction = 1.0f;
    };
    std::vector<std::vector<std::vector<Crossing>>> crossings(count);
    for (std::size_t k = 0; k < count; ++k) {
        Level& level = m_levels[k];
        const float seam = base + slope * (m_t0 + m_step * static_cast<float>(k));
        crossings[k].assign(level.chains.size(), {});
        for (const Segment& s : level.segments) {
            const float dx = s.b.x - s.a.x;
            if ((s.a.x - seam) * (s.b.x - seam) > 0.0f || std::abs(dx) < 1e-12f) continue;
            const float u = std::clamp((seam - s.a.x) / dx, 0.0f, 1.0f);
            crossings[k][s.chain].push_back({ { seam, s.a.y + (s.b.y - s.a.y) * u }, s.startArc + s.length * u, dx > 0.0f ? 1.0f : -1.0f });
        }
        for (std::uint32_t i = 0; i < level.segments.size(); ++i) level.byTriangle.emplace(level.segments[i].triangle, i);
    }
    // O nivel de partida: o do meio do trecho ajustado (com cruzamento).
    const float middle = 0.5f * (std::max(settings.seamFrom, m_t0) + std::min(settings.seamTo, m_t0 + m_step * static_cast<float>(count - 1)));
    std::size_t start = std::min(count - 1, static_cast<std::size_t>(std::max(0.0f, (middle - m_t0) / m_step)));
    // O ponto da costura de cada nivel (o da corrente mais comprida), seguido.
    std::vector<std::optional<Point2>> track(count);
    const auto choose = [&](std::size_t k, const std::optional<Point2>& near) {
        Level& level = m_levels[k];
        std::optional<Point2> mainPoint;
        float mainLength = -1.0f;
        for (std::uint32_t c = 0; c < level.chains.size(); ++c) {
            const std::vector<Crossing>& list = crossings[k][c];
            level.chains[c].seam = !list.empty();
            if (list.empty()) continue;
            const Crossing* best = &list.front();
            for (const Crossing& candidate : list) {
                const bool better = near ? std::hypot(candidate.at.x - near->x, candidate.at.y - near->y) < std::hypot(best->at.x - near->x, best->at.y - near->y)
                                         : candidate.at.y > best->at.y;
                if (better) best = &candidate;
            }
            // Longe da costura seguida (um pedaco partido que so cruza a reta
            // do lado de dentro): sem coordenada.
            if (near && std::hypot(best->at.x - near->x, best->at.y - near->y) > 0.03f) {
                level.chains[c].seam = false;
                continue;
            }
            level.chains[c].seamArc = best->arc;
            level.chains[c].direction = best->direction;
            if (level.chains[c].length > mainLength) {
                mainLength = level.chains[c].length;
                mainPoint = best->at;
            }
        }
        return mainPoint ? mainPoint : near;
    };
    if (count > 0) {
        track[start] = choose(start, std::nullopt);
        for (std::size_t k = start + 1; k < count; ++k) track[k] = choose(k, track[k - 1]);
        for (std::size_t k = start; k-- > 0;) track[k] = choose(k, track[k + 1]);
    }
}

std::optional<HalfUnrolled::Arc> HalfUnrolled::arcAt(std::size_t index, std::uint32_t triangle, Point2 q, bool own) const {
    const Level& level = m_levels[index];
    if (level.segments.empty()) return std::nullopt;
    // O ponto do trecho mais perto da amostra (u de 0 a 1, ou de lo a hi).
    const auto nearest = [&](const Segment& s, float lo, float hi, float& u) {
        const float dx = s.b.x - s.a.x, dy = s.b.y - s.a.y;
        u = std::clamp(((q.x - s.a.x) * dx + (q.y - s.a.y) * dy) / std::max(1e-12f, dx * dx + dy * dy), lo, hi);
        return std::hypot(q.x - (s.a.x + dx * u), q.y - (s.a.y + dy * u));
    };
    // O trecho de partida: o do proprio triangulo (a amostra esta nele);
    // senao, o mais perto entre os dos vizinhos dele (um triangulo fino entre
    // dois niveis), se estiver perto. Nunca o de outra corrente qualquer:
    // perto da barra a fatia se parte, e a ponta do outro pedaco esta na
    // costura.
    std::uint32_t start = 0;
    if (const auto found = level.byTriangle.find(triangle); found != level.byTriangle.end()) {
        start = found->second;
    } else if (own) {
        return std::nullopt;
    } else {
        const Triangle& mine = m_mesh->triangles[triangle];
        const auto neighbor = [&](std::uint32_t other) {
            for (const std::uint32_t a : m_mesh->triangles[other])
                for (const std::uint32_t b : mine)
                    if (a == b) return true;
            return false;
        };
        float best = 5.0f * m_step;
        bool found = false;
        for (std::uint32_t i = 0; i < level.segments.size(); ++i) {
            if (!neighbor(level.segments[i].triangle)) continue;
            float u = 0.0f;
            const float d = nearest(level.segments[i], 0.0f, 1.0f, u);
            if (d < best) {
                best = d;
                start = i;
                found = true;
            }
        }
        if (!found) return std::nullopt;
    }
    const Chain& chain = level.chains[level.segments[start].chain];
    if (!chain.seam || chain.count == 0) return std::nullopt;
    // O ponto da corrente mais perto da amostra em volta do trecho de partida:
    // entre dois niveis a amostra cai alem da ponta de um trecho curtinho (perto
    // de um vertice), ja na altura do trecho seguinte. Alem da ponta de uma
    // corrente aberta, a reta do trecho da ponta.
    const int count = static_cast<int>(chain.count);
    const int at = static_cast<int>(start - chain.first);
    constexpr int window = 8;
    float best = std::numeric_limits<float>::max(), arc = 0.0f;
    for (int offset = -window; offset <= window; ++offset) {
        int p = at + offset;
        if (chain.closed) p = ((p % count) + count) % count;
        else if (p < 0 || p >= count) continue;
        const Segment& s = level.segments[chain.first + static_cast<std::uint32_t>(p)];
        const float lo = !chain.closed && p == 0 ? -1e6f : 0.0f, hi = !chain.closed && p == count - 1 ? 1e6f : 1.0f;
        float u = 0.0f;
        const float d = nearest(s, lo, hi, u);
        if (d < best) {
            best = d;
            arc = s.startArc + s.length * u;
        }
    }
    return Arc { wrapped((arc - chain.seamArc) * chain.direction, chain.period), chain.period };
}

std::optional<Point2> HalfUnrolled::locate(const SurfaceSample& sample) const {
    if (sample.triangle >= m_triangles.size() || !m_triangles[sample.triangle] || m_levels.empty()) return std::nullopt;
    const float t = dot(sample.position - m_origin, m_axis);
    const float f = std::clamp((t - m_t0) / m_step, 0.0f, static_cast<float>(m_levels.size() - 1));
    const std::size_t k = static_cast<std::size_t>(f);
    const std::size_t k1 = std::min(m_levels.size() - 1, k + 1);
    const Point2 q = project(sample.position);
    const auto triangle = static_cast<std::uint32_t>(sample.triangle);
    // Os dois niveis em volta pelo trecho do proprio triangulo; se so um o
    // corta (a ponta do triangulo entre os niveis), so ele - o vizinho do
    // outro nivel pode estar longe (na barra a fatia de baixo se parte). Os
    // vizinhos so quando nenhum dos dois corta o triangulo.
    std::optional<Arc> a = arcAt(k, triangle, q, true);
    std::optional<Arc> b = arcAt(k1, triangle, q, true);
    if (!a && !b) {
        a = arcAt(k, triangle, q, false);
        b = arcAt(k1, triangle, q, false);
    }
    if (!a || !b) {
        if (!a && !b) return std::nullopt;
        return Point2 { a ? a->s : b->s, t };
    }
    // Entre os dois niveis pelo caminho curto da volta; do outro lado do
    // corte da volta (a costura de dentro; na barra o corte muda de lugar),
    // o nivel mais perto - a media passaria por todo o arco (pela faixa).
    const float w = f - static_cast<float>(k);
    const float period = std::max(a->period, b->period);
    float to = b->s;
    if (period > 0.0f && std::abs(to - a->s) > 0.5f * period) to += to < a->s ? period : -period;
    if (std::abs(to - a->s) > 0.02f) return Point2 { w < 0.5f ? a->s : b->s, t };
    return Point2 { wrapped(a->s + (to - a->s) * w, period), t };
}

} // namespace

PaintSpace planarSpace(std::vector<int> parts, Vec3 origin, Vec3 u, Vec3 v, Vec3 facing, float minFacing) {
    PaintSpace space;
    const Vec3 face = safeNormalized(facing, { 1.0f, 0.0f, 0.0f });
    space.locate = [parts = std::move(parts), origin, u, v, face, minFacing](const SurfaceSample& sample) -> std::optional<SpacePoint> {
        if (std::find(parts.begin(), parts.end(), sample.part) == parts.end()) return std::nullopt;
        if (dot(sample.normal, face) < minFacing) return std::nullopt;
        const Vec3 d = sample.position - origin;
        return SpacePoint { { dot(d, u), dot(d, v) }, sideOf(sample.position), sample.footprint };
    };
    return space;
}

PaintSpace unrolledSpace(const Mesh& mesh, const UnrolledSpaceSettings& settings) {
    PaintSpace space;
    const auto right = std::make_shared<HalfUnrolled>(mesh, settings, -1);
    const auto left = std::make_shared<HalfUnrolled>(mesh, settings, 1);
    space.locate = [right, left](const SurfaceSample& sample) -> std::optional<SpacePoint> {
        // A metade do triangulo da amostra (no meio, o y do ponto engana).
        int side = -1;
        std::optional<Point2> point = right->locate(sample);
        if (!point) {
            side = 1;
            point = left->locate(sample);
        }
        if (!point) return std::nullopt;
        return SpacePoint { *point, side, sample.footprint };
    };
    space.anchors["inicio"] = right->axisMinimum();
    space.anchors["fim"] = right->axisMaximum();
    return space;
}

} // namespace MatterEngine::Modeling
