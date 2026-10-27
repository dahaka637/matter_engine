#include "Engine/Physics/Net/NetSolver3D.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <queue>

namespace MatterEngine {

namespace {

constexpr std::uint32_t NoAnchor = std::numeric_limits<std::uint32_t>::max();
// O atrito do fio no tubo da trave (aco pintado).
constexpr float FrameFriction = 0.3f;
// Folga das listas do tick: o quanto um no ou um fio pode andar ate um corpo
// num tick sem estar na lista (m).
constexpr float TickMargin = 0.12f;
// O fio esticado acima disto: o no dele nao cede mais a um corpo no contato
// final (a caixa de 14 kg contra os nos de 2 g esticava fios ate 51%).
constexpr float StiffStrain = 0.05f;
// O quanto um no pode andar ate a lista dos nos perto dos tubos ser refeita
// (m).
constexpr float FrameSlack = 0.05f;
// A velocidade de separacao que um contato pode dar alem de parar o corpo
// (m/s): a posicao sai inteira, a velocidade nao. Um ponto que nao cede
// (preso na trave, ou de fio esticado) achado fundo na caixa que girou no
// tick a lancava a mais de 10 m/s; a bola espremendo a rede contra o tubo do
// lado, de perto a 30-45 m/s, saia a 40-60 m/s pela rede.
constexpr float MaximumSeparation = 0.5f;
// Dormindo, o corpo devagar a menos disto de uma celula (alem do que anda no
// tick) esta apoiado: a rede parada o segura (m).
constexpr float RestReach = 0.01f;

inline float lengthOf(Vec3 v) {
    return std::sqrt(v.lengthSquared());
}

inline Vec3 closestOnSegment(Vec3 point, Vec3 a, Vec3 b, float& t) {
    const Vec3 ab = b - a;
    const float lengthSquared = ab.lengthSquared();
    t = lengthSquared > 1e-12f ? std::clamp(dot(point - a, ab) / lengthSquared, 0.0f, 1.0f) : 0.0f;
    return a + ab * t;
}

inline Vec3 samplePosition(const NetSample3D& sample, const std::vector<Vec3>& positions) {
    return positions[sample.knots[0]] * sample.weights[0] + positions[sample.knots[1]] * sample.weights[1]
        + positions[sample.knots[2]] * sample.weights[2] + positions[sample.knots[3]] * sample.weights[3];
}

// O ponto do triangulo abc mais perto de p e os pesos dele nos cantos
// (Ericson, "Real-Time Collision Detection", 5.1.5).
inline Vec3 closestOnTriangle(Vec3 p, Vec3 a, Vec3 b, Vec3 c, std::array<float, 3>& weights) {
    const Vec3 ab = b - a, ac = c - a, ap = p - a;
    const float d1 = dot(ab, ap), d2 = dot(ac, ap);
    if (d1 <= 0.0f && d2 <= 0.0f) {
        weights = { 1.0f, 0.0f, 0.0f };
        return a;
    }
    const Vec3 bp = p - b;
    const float d3 = dot(ab, bp), d4 = dot(ac, bp);
    if (d3 >= 0.0f && d4 <= d3) {
        weights = { 0.0f, 1.0f, 0.0f };
        return b;
    }
    const float vc = d1 * d4 - d3 * d2;
    if (vc <= 0.0f && d1 >= 0.0f && d3 <= 0.0f) {
        const float v = d1 / (d1 - d3);
        weights = { 1.0f - v, v, 0.0f };
        return a + ab * v;
    }
    const Vec3 cp = p - c;
    const float d5 = dot(ab, cp), d6 = dot(ac, cp);
    if (d6 >= 0.0f && d5 <= d6) {
        weights = { 0.0f, 0.0f, 1.0f };
        return c;
    }
    const float vb = d5 * d2 - d1 * d6;
    if (vb <= 0.0f && d2 >= 0.0f && d6 <= 0.0f) {
        const float w = d2 / (d2 - d6);
        weights = { 1.0f - w, 0.0f, w };
        return a + ac * w;
    }
    const float va = d3 * d6 - d5 * d4;
    if (va <= 0.0f && d4 - d3 >= 0.0f && d5 - d6 >= 0.0f) {
        const float w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        weights = { 0.0f, 1.0f - w, w };
        return b + (c - b) * w;
    }
    const float sum = va + vb + vc;
    // (A celula amassada, os tres cantos em linha: pela borda ab.)
    if (sum <= 1e-30f) {
        float t = 0.0f;
        const Vec3 closest = closestOnSegment(p, a, b, t);
        weights = { 1.0f - t, t, 0.0f };
        return closest;
    }
    const float v = vb / sum, w = vc / sum;
    weights = { 1.0f - v - w, v, w };
    return a + ab * v + ac * w;
}

// Os pontos mais perto entre os segmentos p1q1 e p2q2 (Ericson, 5.1.9): s e t
// ao longo de cada um; devolve a distancia ao quadrado.
inline float closestSegmentSegment(Vec3 p1, Vec3 q1, Vec3 p2, Vec3 q2, float& s, float& t) {
    constexpr float Epsilon = 1e-12f;
    const Vec3 d1 = q1 - p1, d2 = q2 - p2, r = p1 - p2;
    const float a = d1.lengthSquared(), e = d2.lengthSquared(), f = dot(d2, r);
    if (a <= Epsilon && e <= Epsilon) {
        s = t = 0.0f;
    } else if (a <= Epsilon) {
        s = 0.0f;
        t = std::clamp(f / e, 0.0f, 1.0f);
    } else {
        const float c = dot(d1, r);
        if (e <= Epsilon) {
            t = 0.0f;
            s = std::clamp(-c / a, 0.0f, 1.0f);
        } else {
            const float b = dot(d1, d2), denominator = a * e - b * b;
            s = denominator > Epsilon ? std::clamp((b * f - c * e) / denominator, 0.0f, 1.0f) : 0.0f;
            t = (b * s + f) / e;
            if (t < 0.0f) {
                t = 0.0f;
                s = std::clamp(-c / a, 0.0f, 1.0f);
            } else if (t > 1.0f) {
                t = 1.0f;
                s = std::clamp((b - c) / a, 0.0f, 1.0f);
            }
        }
    }
    return ((p1 + d1 * s) - (p2 + d2 * t)).lengthSquared();
}

// O ponto do segmento pq mais perto do triangulo abc, o do triangulo e os pesos
// dele. O segmento que fura o triangulo devolve o furo (distancia 0).
inline void closestSegmentTriangle(Vec3 p, Vec3 q, Vec3 a, Vec3 b, Vec3 c, Vec3& onSegment, Vec3& onTriangle,
    std::array<float, 3>& weights) {
    const Vec3 normal = cross(b - a, c - a);
    const float dp = dot(p - a, normal), dq = dot(q - a, normal);
    if ((dp > 0.0f) != (dq > 0.0f) && std::abs(dp - dq) > 1e-20f) {
        const Vec3 x = p + (q - p) * (dp / (dp - dq));
        std::array<float, 3> w {};
        const Vec3 on = closestOnTriangle(x, a, b, c, w);
        if ((on - x).lengthSquared() < 1e-12f) {
            onSegment = x;
            onTriangle = on;
            weights = w;
            return;
        }
    }
    // As pontas contra o triangulo e o segmento contra cada borda.
    onSegment = p;
    onTriangle = closestOnTriangle(p, a, b, c, weights);
    float best = (onSegment - onTriangle).lengthSquared();
    std::array<float, 3> w {};
    const Vec3 fromQ = closestOnTriangle(q, a, b, c, w);
    if ((q - fromQ).lengthSquared() < best) {
        best = (q - fromQ).lengthSquared();
        onSegment = q;
        onTriangle = fromQ;
        weights = w;
    }
    const Vec3 corners[3] { a, b, c };
    for (int e = 0; e < 3; ++e) {
        const Vec3 u = corners[e], v = corners[(e + 1) % 3];
        float s = 0.0f, t = 0.0f;
        const float distance = closestSegmentSegment(p, q, u, v, s, t);
        if (distance >= best) continue;
        best = distance;
        onSegment = p + (q - p) * s;
        onTriangle = u + (v - u) * t;
        weights = { 0.0f, 0.0f, 0.0f };
        weights[static_cast<std::size_t>(e)] = 1.0f - t;
        weights[static_cast<std::size_t>((e + 1) % 3)] = t;
    }
}

struct Box {
    Vec3 minimum { std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max() };
    Vec3 maximum { -std::numeric_limits<float>::max(), -std::numeric_limits<float>::max(), -std::numeric_limits<float>::max() };

    void include(Vec3 p) {
        minimum = { std::min(minimum.x, p.x), std::min(minimum.y, p.y), std::min(minimum.z, p.z) };
        maximum = { std::max(maximum.x, p.x), std::max(maximum.y, p.y), std::max(maximum.z, p.z) };
    }
    void expand(float amount) {
        minimum = minimum - Vec3 { amount, amount, amount };
        maximum = maximum + Vec3 { amount, amount, amount };
    }
    [[nodiscard]] bool overlaps(Vec3 otherMinimum, Vec3 otherMaximum) const {
        return minimum.x <= otherMaximum.x && maximum.x >= otherMinimum.x && minimum.y <= otherMaximum.y
            && maximum.y >= otherMinimum.y && minimum.z <= otherMaximum.z && maximum.z >= otherMinimum.z;
    }
    [[nodiscard]] bool contains(Vec3 p) const {
        return p.x >= minimum.x && p.x <= maximum.x && p.y >= minimum.y && p.y <= maximum.y && p.z >= minimum.z
            && p.z <= maximum.z;
    }
};

// A caixa (no mundo) que contem o proxy agora e onde ele estara no fim do tick.
Box sweptProxyBox(const NetBody3D& body, const NetProxy3D& proxy, float deltaTime, float margin) {
    const Vec3 center = body.position + body.orientation.rotate(proxy.localPosition);
    float reach = proxy.radius;
    if (proxy.shape == NetProxyShape3D::Capsule) reach = proxy.radius + proxy.halfHeight;
    if (proxy.shape == NetProxyShape3D::Box) reach = lengthOf(proxy.halfExtents);
    Box box;
    box.include(center);
    box.include(center + body.velocity * deltaTime);
    box.expand(reach + margin);
    return box;
}

} // namespace

NetSolver3D::NetSolver3D(const NetDescription3D& description)
    : m_settings(description.settings), m_positions(description.positions), m_previous(description.positions), m_start(description.positions),
      m_velocities(description.positions.size()), m_pinned(description.pinned), m_segments(description.segments),
      m_faces(description.faces), m_colliders(description.colliders) {
    const std::size_t count = m_positions.size();
    m_pinned.resize(count, 0);
    m_masses.assign(count, 0.0f);
    m_inverseMasses.assign(count, 0.0f);
    for (std::size_t i = 0; i < count; ++i) {
        const float mass = i < description.masses.size() ? description.masses[i] : 0.0f;
        m_masses[i] = std::max(mass, 1e-5f);
        m_totalMass += m_masses[i];
        m_inverseMasses[i] = m_pinned[i] != 0 ? 0.0f : 1.0f / m_masses[i];
        if (m_pinned[i] == 0) m_free.push_back(static_cast<std::uint32_t>(i));
    }
    // O fio que cada no carrega (a metade de cada fio dele) contra o ar.
    std::vector<float> carried(count, 0.0f);
    m_compliances.reserve(m_segments.size());
    for (const NetSegment3D& segment : m_segments) {
        carried[segment.a] += 0.5f * segment.restLength;
        carried[segment.b] += 0.5f * segment.restLength;
        m_compliances.push_back(segment.restLength / std::max(segment.axialStiffness, 1.0f));
    }
    m_samples = description.samples;
    if (m_samples.empty())
        for (std::uint32_t i = 0; i < count; ++i) m_samples.push_back({ { i, i, i, i }, { 1.0f, 0.0f, 0.0f, 0.0f } });
    // O arrasto de cada no por velocidade e por segundo (1/m): 1/2 rho Cd A / m.
    m_dragFactors.resize(count);
    for (std::size_t i = 0; i < count; ++i) {
        const float area = description.dragAreas.size() == count ? description.dragAreas[i] : 2.0f * m_settings.twineRadius * carried[i];
        m_dragFactors[i] = 0.5f * m_settings.airDensity * m_settings.dragCoefficient * area * m_inverseMasses[i];
    }
    // Os fios do laco quente, com o peso de cada ponta (o fio preso nas duas
    // pontas fica de fora), em cada sentido da passada.
    std::vector<Link> links, ropes;
    for (std::size_t s = 0; s < m_segments.size(); ++s) {
        const NetSegment3D& segment = m_segments[s];
        Link link { segment.a, segment.b, segment.restLength, m_compliances[s], m_inverseMasses[segment.a], m_inverseMasses[segment.b] };
        if (link.wa + link.wb == 0.0f) continue;
        link.inverseWeights = 1.0f / (link.wa + link.wb);
        link.dampingRate = m_settings.stretchDampingTime * segment.axialStiffness / std::max(segment.restLength, 1e-6f) * (link.wa + link.wb);
        link.slot = static_cast<std::uint32_t>(s);
        links.push_back(link);
        if (segment.rope) ropes.push_back(link);
    }
    // O tamanho maximo de uma celula: dois fios dela esticados ate o limite
    // (a celula pode se fechar como um losango; com folga).
    float longest = 0.0f;
    for (const Link& link : links) longest = std::max(longest, link.rest);
    m_cellReach = 2.0f * (1.0f + m_settings.maximumStrain) * longest * 1.1f;
    m_links = scheduleLinks(links, count);
    std::reverse(links.begin(), links.end());
    m_linksBackward = scheduleLinks(links, count);
    m_ropeLinks = scheduleLinks(ropes, count);
    std::reverse(ropes.begin(), ropes.end());
    m_ropeLinksBackward = scheduleLinks(ropes, count);
    m_linkLambda.assign(m_segments.size(), 0.0f);
    m_stiff.assign(count, 0);
    m_onFrame.assign(count, Vec3 {});
    // O alongamento medido em pelo menos meio fio comum (a mediana): os
    // pedacos de milimetros junto a borda (o cruzamento da malha rente a um no
    // da borda) marcavam 40% com 4 mm.
    std::vector<float> lengths;
    for (const NetSegment3D& segment : m_segments) lengths.push_back(segment.restLength);
    if (!lengths.empty()) {
        std::nth_element(lengths.begin(), lengths.begin() + static_cast<std::ptrdiff_t>(lengths.size() / 2), lengths.end());
        m_strainLength = 0.5f * lengths[lengths.size() / 2];
    }
    computeTethers();
    computeBounds();
    // Os pares (no, tubo) que podem se tocar: o no nao passa do caminho dos
    // fios ate a amarra esticado ate o limite (mais a folga).
    for (std::uint32_t i = 0; i < count; ++i) {
        if (m_inverseMasses[i] == 0.0f) continue;
        const std::uint32_t anchor = m_tetherAnchor[i];
        const float reach = anchor == NoAnchor ? std::numeric_limits<float>::max()
                                               : m_tetherLength[i] * (1.0f + m_settings.maximumStrain) * 1.1f + 0.05f;
        for (std::uint32_t c = 0; c < m_colliders.size(); ++c) {
            const NetCapsule3D& capsule = m_colliders[c];
            float t = 0.0f;
            const Vec3 from = anchor == NoAnchor ? m_positions[i] : m_positions[anchor];
            const float distance = lengthOf(from - closestOnSegment(from, capsule.a, capsule.b, t));
            if (distance < reach + capsule.radius + m_settings.twineRadius) m_framePairs.emplace_back(i, c);
        }
    }
}

std::vector<NetSolver3D::Link> NetSolver3D::scheduleLinks(const std::vector<Link>& order, std::size_t knotCount) {
    // A mesma passada de Gauss-Seidel em outra ordem: cada fio no nivel logo
    // depois do ultimo fio que mexeu num no dele (o no preso nao conta: nao
    // anda), e os niveis em sequencia. Cada no ve os seus fios na mesma ordem
    // (o resultado e o mesmo, bit a bit), e os fios de um nivel nao dividem
    // nos: o processador adianta varios ao mesmo tempo em vez de esperar o
    // fio anterior (o custo era a espera, nao a conta).
    std::vector<std::uint32_t> lastLevel(knotCount, 0);
    std::vector<std::pair<std::uint32_t, std::uint32_t>> levels;
    for (std::uint32_t k = 0; k < order.size(); ++k) {
        const Link& link = order[k];
        const std::uint32_t level = 1 + std::max(link.wa > 0.0f ? lastLevel[link.a] : 0u, link.wb > 0.0f ? lastLevel[link.b] : 0u);
        if (link.wa > 0.0f) lastLevel[link.a] = level;
        if (link.wb > 0.0f) lastLevel[link.b] = level;
        levels.emplace_back(level, k);
    }
    std::stable_sort(levels.begin(), levels.end(), [](const auto& x, const auto& y) { return x.first < y.first; });
    std::vector<Link> scheduled;
    scheduled.reserve(order.size());
    for (const auto& [level, k] : levels) scheduled.push_back(order[k]);
    return scheduled;
}

void NetSolver3D::computeTethers() {
    // O caminho mais curto pelos fios ate um no preso (Dijkstra de todas as
    // amarras de uma vez), e a rigidez do fio desse caminho.
    const std::size_t count = m_positions.size();
    std::vector<std::uint32_t> offsets(count + 1, 0);
    for (const NetSegment3D& segment : m_segments) {
        ++offsets[segment.a + 1];
        ++offsets[segment.b + 1];
    }
    for (std::size_t i = 0; i < count; ++i) offsets[i + 1] += offsets[i];
    struct Edge {
        std::uint32_t other = 0;
        float length = 0.0f;
        float compliance = 0.0f;
    };
    std::vector<Edge> edges(offsets[count]);
    std::vector<std::uint32_t> fill(offsets.begin(), offsets.end() - 1);
    for (std::size_t s = 0; s < m_segments.size(); ++s) {
        const NetSegment3D& segment = m_segments[s];
        edges[fill[segment.a]++] = { segment.b, segment.restLength, m_compliances[s] };
        edges[fill[segment.b]++] = { segment.a, segment.restLength, m_compliances[s] };
    }
    m_tetherAnchor.assign(count, NoAnchor);
    m_tetherLength.assign(count, std::numeric_limits<float>::max());
    m_tetherCompliance.assign(count, 0.0f);
    using Item = std::pair<float, std::uint32_t>;
    std::priority_queue<Item, std::vector<Item>, std::greater<Item>> queue;
    for (std::uint32_t i = 0; i < count; ++i)
        if (m_pinned[i] != 0) {
            m_tetherAnchor[i] = i;
            m_tetherLength[i] = 0.0f;
            queue.push({ 0.0f, i });
        }
    while (!queue.empty()) {
        const auto [distance, node] = queue.top();
        queue.pop();
        if (distance > m_tetherLength[node]) continue;
        for (std::uint32_t e = offsets[node]; e < offsets[node + 1]; ++e) {
            const Edge& edge = edges[e];
            const float candidate = distance + edge.length;
            if (candidate < m_tetherLength[edge.other]) {
                m_tetherLength[edge.other] = candidate;
                m_tetherAnchor[edge.other] = m_tetherAnchor[node];
                // Os fios do caminho em serie: as complacencias somam.
                m_tetherCompliance[edge.other] = m_tetherCompliance[node] + edge.compliance;
                queue.push({ candidate, edge.other });
            }
        }
    }
    for (float& length : m_tetherLength)
        if (length < std::numeric_limits<float>::max()) length *= 1.0f + m_settings.tetherStretch;
    // Os cabos: de cada ponta presa, pelo cabo, o comprimento e a
    // complacencia ate cada no; o no entre duas pontas ganha a restricao do
    // vao (as duas somadas).
    std::vector<std::vector<std::pair<std::uint32_t, std::uint32_t>>> ropeLinks(count);
    for (std::uint32_t s = 0; s < m_segments.size(); ++s)
        if (m_segments[s].rope) {
            ropeLinks[m_segments[s].a].push_back({ m_segments[s].b, s });
            ropeLinks[m_segments[s].b].push_back({ m_segments[s].a, s });
        }
    struct Reach {
        std::uint32_t anchor = NoAnchor;
        float length = 0.0f;
        float compliance = 0.0f;
    };
    std::vector<std::vector<Reach>> reached(count);
    for (std::uint32_t anchor = 0; anchor < count; ++anchor) {
        if (m_pinned[anchor] == 0 || ropeLinks[anchor].empty()) continue;
        std::vector<std::uint8_t> seen(count, 0);
        seen[anchor] = 1;
        std::vector<std::pair<std::uint32_t, Reach>> stack { { anchor, { anchor, 0.0f, 0.0f } } };
        while (!stack.empty()) {
            const auto [knot, reach] = stack.back();
            stack.pop_back();
            for (const auto& [next, s] : ropeLinks[knot]) {
                if (seen[next] != 0 || m_pinned[next] != 0) continue;
                seen[next] = 1;
                const Reach further { anchor, reach.length + m_segments[s].restLength, reach.compliance + m_compliances[s] };
                reached[next].push_back(further);
                stack.push_back({ next, further });
            }
        }
    }
    for (std::uint32_t knot = 0; knot < count; ++knot)
        if (reached[knot].size() >= 2) {
            const Reach& a = reached[knot][0];
            const Reach& b = reached[knot][1];
            // A tracao: o cabo inteiro (a + b de comprimento solto, a + b de
            // complacencia) esticado no vao entre as pontas (no minimo 0,5%).
            const float span = lengthOf(m_positions[b.anchor] - m_positions[a.anchor]);
            const float rest = a.length + b.length;
            const float tension = std::max(0.005f, span / std::max(rest, 1e-6f) - 1.0f) * rest / std::max(a.compliance + b.compliance, 1e-9f);
            const float toFirst = std::max(0.01f, lengthOf(m_positions[knot] - m_positions[a.anchor]));
            const float toSecond = std::max(0.01f, lengthOf(m_positions[knot] - m_positions[b.anchor]));
            m_ropeSpans.push_back({ knot, a.anchor, b.anchor, 1.0f / (tension * (1.0f / toFirst + 1.0f / toSecond)) });
        }
}

void NetSolver3D::computeBounds() {
    Box box;
    for (const Vec3& p : m_positions) box.include(p);
    if (m_positions.empty()) box = {};
    m_boundsMinimum = box.minimum;
    m_boundsMaximum = box.maximum;
}

void NetSolver3D::wake() {
    m_sleeping = false;
    m_frameStale = true;
    m_stillSeconds = 0.0f;
    m_stillReference.clear();
}

void NetSolver3D::beginTick(float deltaTime, std::span<const NetBody3D> bodies, std::span<const NetProxy3D> proxies) {
    m_stats = {};
    m_tickTime = deltaTime;
    m_touchSpeed = 0.0f;
    Box net;
    net.minimum = m_boundsMinimum;
    net.maximum = m_boundsMaximum;
    net.expand(m_settings.twineRadius);
    // Dormindo, so contam os corpos que a rede parada segura (com massa) e os
    // que chegam rapido (acordam): o cinematico devagar (o boneco guiado parado
    // dentro do gol, o goleiro) passa por ela parada sem nada, e contava como
    // apoiado: a lista do tick e os subpassos a cada tick, 0,06 ms.
    const auto counts = [&](const NetProxy3D& proxy) {
        const NetBody3D& body = bodies[proxy.body];
        return !m_sleeping || body.inverseMass > 0.0f || lengthOf(body.velocity) > m_settings.wakeSpeed;
    };
    bool nearby = false;
    for (const NetProxy3D& proxy : proxies)
        if (proxy.body < bodies.size() && counts(proxy)
            && sweptProxyBox(bodies[proxy.body], proxy, deltaTime, m_settings.twineRadius).overlaps(net.minimum, net.maximum))
            nearby = true;
    m_touching = false;
    // Os candidatos de cada proxy: celulas para a esfera, nos virtuais para
    // os outros.
    m_proxyRanges.assign(proxies.size(), { 0u, 0u });
    m_candidates.clear();
    // O centro de cada proxy no comeco do tick: o lado de onde ele veio.
    m_proxyStarts.resize(proxies.size());
    for (std::uint32_t p = 0; p < proxies.size(); ++p)
        if (proxies[p].body < bodies.size())
            m_proxyStarts[p] = bodies[proxies[p].body].position + bodies[proxies[p].body].orientation.rotate(proxies[p].localPosition);
    if (m_sleeping && !nearby) return;
    const bool awake = !m_sleeping;
    // Uma passada so pelas celulas (e pelos nos virtuais) contra a caixa que
    // junta os proxies perto da rede: a lista curta, em ordem, que cada proxy
    // testa (o boneco tem ~20; cada um passava por todos os nos e celulas).
    // Dormindo, so os que contam, e o devagar so com o que ele alcanca no tick
    // (acordando, a lista inteira e refeita).
    const auto marginOf = [&](const NetProxy3D& proxy) {
        const float speed = lengthOf(bodies[proxy.body].velocity);
        return m_settings.twineRadius + (m_sleeping && speed <= m_settings.wakeSpeed ? RestReach + speed * deltaTime : TickMargin);
    };
    const auto gather = [&] {
        Box gathered;
        bool round = false, square = false;
        for (std::uint32_t p = 0; p < proxies.size(); ++p) {
            const NetProxy3D& proxy = proxies[p];
            if (proxy.body >= bodies.size() || !counts(proxy)) continue;
            const Box box = sweptProxyBox(bodies[proxy.body], proxy, deltaTime, marginOf(proxy));
            if (!box.overlaps(net.minimum, net.maximum)) continue;
            gathered.include(box.minimum);
            gathered.include(box.maximum);
            (proxy.shape == NetProxyShape3D::Box ? square : round) = true;
        }
        m_localFaces.clear();
        m_localSamples.clear();
        // Os nos perto da caixa de todos (alargada pelo tamanho maximo de uma
        // celula esticada; a celula ou o no virtual que cai nela tem todos os
        // nos na alargada), uma vez por tick.
        bool marked = false;
        const auto markNear = [&] {
            if (marked) return;
            marked = true;
            Box wide = gathered;
            wide.expand(m_cellReach);
            m_near.resize(m_positions.size());
            for (std::size_t i = 0; i < m_positions.size(); ++i) m_near[i] = wide.contains(m_positions[i]) ? 1 : 0;
        };
        if (round) {
            // (A caixa de cada celula: dormindo, todas uma vez so, ate a rede
            // andar; acordada, a cada tick so as que tem um no perto.)
            m_faceLow.resize(m_faces.size());
            m_faceHigh.resize(m_faces.size());
            const auto bound = [&](std::uint32_t f) {
                const Vec3 a = m_positions[m_faces[f][0]], b = m_positions[m_faces[f][1]], c = m_positions[m_faces[f][2]];
                m_faceLow[f] = { std::min({ a.x, b.x, c.x }), std::min({ a.y, b.y, c.y }), std::min({ a.z, b.z, c.z }) };
                m_faceHigh[f] = { std::max({ a.x, b.x, c.x }), std::max({ a.y, b.y, c.y }), std::max({ a.z, b.z, c.z }) };
            };
            if (m_faceBoundsStale && m_sleeping) {
                m_faceBoundsStale = false;
                for (std::uint32_t f = 0; f < m_faces.size(); ++f) bound(f);
            }
            if (!m_faceBoundsStale) {
                for (std::uint32_t f = 0; f < m_faces.size(); ++f)
                    if (gathered.overlaps(m_faceLow[f], m_faceHigh[f])) m_localFaces.push_back(f);
            } else {
                markNear();
                for (std::uint32_t f = 0; f < m_faces.size(); ++f) {
                    if ((m_near[m_faces[f][0]] | m_near[m_faces[f][1]] | m_near[m_faces[f][2]]) == 0) continue;
                    bound(f);
                    if (gathered.overlaps(m_faceLow[f], m_faceHigh[f])) m_localFaces.push_back(f);
                }
            }
        }
        if (square) {
            markNear();
            for (std::uint32_t i = 0; i < m_samples.size(); ++i) {
                const NetSample3D& sample = m_samples[i];
                if ((m_near[sample.knots[0]] | m_near[sample.knots[1]] | m_near[sample.knots[2]] | m_near[sample.knots[3]]) == 0) continue;
                if (gathered.contains(samplePosition(sample, m_positions))) m_localSamples.push_back(i);
            }
        }
        m_candidates.clear();
        for (std::uint32_t p = 0; p < proxies.size(); ++p) {
            const NetProxy3D& proxy = proxies[p];
            m_proxyRanges[p] = { 0u, 0u };
            if (proxy.body >= bodies.size() || !counts(proxy)) continue;
            const Box box = sweptProxyBox(bodies[proxy.body], proxy, deltaTime, marginOf(proxy));
            const auto first = static_cast<std::uint32_t>(m_candidates.size());
            if (box.overlaps(net.minimum, net.maximum)) {
                // (Os nos perto: a caixa alargada pelo tamanho maximo de uma
                // celula esticada; a celula ou o no virtual que cai na caixa tem
                // todos os nos nela.)
                Box wide = box;
                wide.expand(m_cellReach);
                if (proxy.shape != NetProxyShape3D::Box) {
                    for (const std::uint32_t f : m_localFaces) {
                        const std::array<std::uint32_t, 3>& face = m_faces[f];
                        if (!wide.contains(m_positions[face[0]]) && !wide.contains(m_positions[face[1]]) && !wide.contains(m_positions[face[2]])) continue;
                        if (box.overlaps(m_faceLow[f], m_faceHigh[f])) m_candidates.push_back(f);
                    }
                } else {
                    for (const std::uint32_t i : m_localSamples) {
                        const NetSample3D& sample = m_samples[i];
                        if (!wide.contains(m_positions[sample.knots[0]]) && !wide.contains(m_positions[sample.knots[1]])
                            && !wide.contains(m_positions[sample.knots[2]]) && !wide.contains(m_positions[sample.knots[3]]))
                            continue;
                        if (box.contains(samplePosition(sample, m_positions))) m_candidates.push_back(i);
                    }
                }
            }
            m_proxyRanges[p] = { first, static_cast<std::uint32_t>(m_candidates.size()) };
        }
    };
    gather();
    // Dormindo: acorda com um corpo chegando rapido a uma celula (o eixo dele a
    // menos do raio mais o que anda no tick), nao so a caixa da rede (o
    // jogador dentro do gol ou encostado na barreira, longe dos fios, a
    // acordava sem tocar nela: 0,1-0,25 ms por tick); segura o corpo devagar
    // que esta a menos de RestReach de uma celula.
    if (m_sleeping) {
        for (std::uint32_t p = 0; p < proxies.size() && m_sleeping; ++p) {
            const NetProxy3D& proxy = proxies[p];
            if (proxy.body >= bodies.size() || !counts(proxy)) continue;
            const NetBody3D& body = bodies[proxy.body];
            const float speed = lengthOf(body.velocity);
            const bool fast = speed > m_settings.wakeSpeed;
            const float slack = fast ? 0.0f : RestReach;
            const Box tight = sweptProxyBox(body, proxy, deltaTime, m_settings.twineRadius + slack);
            const Vec3 center = body.position + body.orientation.rotate(proxy.localPosition);
            const Vec3 axis = proxy.shape == NetProxyShape3D::Capsule
                ? (body.orientation * proxy.localOrientation).rotate({ 0.0f, 0.0f, proxy.halfHeight }) : Vec3 {};
            const float reach = proxy.radius + m_settings.twineRadius + speed * deltaTime + slack;
            for (std::uint32_t k = m_proxyRanges[p].first; k < m_proxyRanges[p].second && m_sleeping; ++k) {
                bool close = false;
                if (proxy.shape == NetProxyShape3D::Box) {
                    close = tight.contains(samplePosition(m_samples[m_candidates[k]], m_positions));
                } else {
                    const std::array<std::uint32_t, 3>& face = m_faces[m_candidates[k]];
                    std::array<float, 3> weights {};
                    const Vec3 a = m_positions[face[0]], b = m_positions[face[1]], c = m_positions[face[2]];
                    Vec3 point = center, closest;
                    if (proxy.shape == NetProxyShape3D::Capsule) closestSegmentTriangle(center - axis, center + axis, a, b, c, point, closest, weights);
                    else closest = closestOnTriangle(center, a, b, c, weights);
                    close = (point - closest).lengthSquared() < reach * reach;
                }
                if (!close) continue;
                if (fast) wake();
                else m_touching = true;
            }
        }
    }
    // Acordou: a lista inteira (todos os corpos, com a folga do tick).
    if (!m_sleeping && !awake) gather();
    if (!m_sleeping || m_touching) m_tickStart.assign(m_positions.begin(), m_positions.end());
    // Os nos que podem tocar um tubo ate a lista ser refeita: refeita quando
    // o no mais rapido ja pode ter andado a folga desde a ultima (a cada tick
    // no impacto; na rede quase parada, a cada dezenas de ticks).
    m_frameTravel += m_fastestKnot * deltaTime;
    if (!m_sleeping && (m_frameTravel > FrameSlack || m_frameStale)) {
        m_frameTravel = 0.0f;
        m_frameStale = false;
        m_frameNear.clear();
        const float margin = m_fastestKnot * deltaTime + FrameSlack + 0.02f;
        for (const auto& [i, c] : m_framePairs) {
            const NetCapsule3D& capsule = m_colliders[c];
            const float reach = capsule.radius + m_settings.twineRadius + margin;
            float t = 0.0f;
            if ((m_positions[i] - closestOnSegment(m_positions[i], capsule.a, capsule.b, t)).lengthSquared() < reach * reach)
                m_frameNear.emplace_back(i, c);
        }
    }
}

void NetSolver3D::substep(float h, std::span<NetBody3D> bodies, std::span<const NetProxy3D> proxies) {
    if (h <= 0.0f) return;
    ++m_stats.substeps;
    m_sphereContacts.clear();
    for (NetBody3D& body : bodies) {
        if (!body.advance) continue;
        body.position = body.position + body.velocity * h;
        const float spin = lengthOf(body.angularVelocity);
        if (spin > 1e-6f)
            body.orientation = (Quaternion::fromAxisAngle(body.angularVelocity * (1.0f / spin), spin * h) * body.orientation).normalized();
    }
    if (m_sleeping) {
        // A rede parada segura o que esta apoiado nela (sem se mexer).
        if (m_touching) {
            collideBodies(h, bodies, proxies, ContactPass::StaticNet);
            applyFriction(h, bodies, proxies, true);
        }
        return;
    }
    m_faceBoundsStale = true;
    const Vec3 gravityStep = m_settings.gravity * h;
    const float linear = m_settings.linearDamping * h;
    const float rest = m_settings.restFriction * h;
    for (const std::uint32_t i : m_free) {
        m_previous[i] = m_positions[i];
        m_start[i] = m_positions[i];
        Vec3 v = m_velocities[i] + gravityStep;
        // O arrasto e o atrito interno implicitos (nunca invertem a velocidade).
        const float speed = lengthOf(v);
        v = v * (std::max(0.0f, speed - rest) / std::max(speed * (1.0f + linear + m_dragFactors[i] * h * speed), 1e-12f));
        m_velocities[i] = v;
        m_positions[i] = m_positions[i] + v * h;
    }

    // 1. O corpo empurra os nos e os nos o freiam (o no conta como
    // contactMassScale vezes mais pesado no contato).
    m_contactImpulse.assign(bodies.size(), 0.0f);
    m_contactNormal.assign(bodies.size(), Vec3 {});
    collideBodies(h, bodies, proxies, ContactPass::Share);
    // 2. Os fios: so puxam (o frouxo fica frouxo). A ordem alterna a cada
    // subpasso: numa ordem so, os fios quase deitados empurravam os nos sempre
    // para o mesmo lado e a rede parada nao parava de escorregar. O fio
    // esticado tambem amortece (viscoso, ver stretchDampingTime): tira a fracao
    // dele do quanto as pontas ainda se afastam (ou se aproximam) ao longo
    // dele neste subpasso.
    const float inverseSquare = 1.0f / (h * h);
    for (const std::uint32_t knot : m_stiffKnots) m_stiff[knot] = 0;
    m_stiffKnots.clear();
    // (1/(w + alfa/h^2) e a fracao do amortecimento de cada fio, refeitos
    // quando o passo muda; o pedaco curtinho da borda amortece no maximo
    // metade.)
    if (h != m_linkStep) {
        m_linkStep = h;
        for (std::vector<Link>* list : { &m_links, &m_linksBackward, &m_ropeLinks, &m_ropeLinksBackward })
            for (Link& link : *list) {
                link.inverseSoft = 1.0f / (link.wa + link.wb + link.compliance * inverseSquare);
                link.damping = std::min(0.5f, link.dampingRate * h);
            }
    }
    const float strainLimit = m_settings.maximumStrain;
    std::fill(m_linkLambda.begin(), m_linkLambda.end(), 0.0f);
    const auto solveLinks = [&](const std::vector<Link>& links, bool main) {
        for (const Link& link : links) {
            const Vec3 delta = m_positions[link.b] - m_positions[link.a];
            const float lengthSquared = delta.lengthSquared();
            float& pulled = m_linkLambda[link.slot];
            if (lengthSquared <= link.rest * link.rest && pulled <= 0.0f) continue;
            const float length = std::sqrt(lengthSquared);
            if (length < 1e-9f) continue;
            const Vec3 direction = delta * (1.0f / length);
            // O que passa do alongamento maximo sai inteiro (o fio nao cede
            // mais); o resto, com a complacencia do fio, pelo XPBD: menos a
            // forca que ja entrou neste subpasso, e o total nunca empurra (o
            // fio que afrouxou devolve so o que puxou).
            const float excess = length - link.rest;
            const float rigid = std::max(0.0f, excess - link.rest * strainLimit);
            const float soft = std::max(-pulled, (excess - rigid - link.compliance * inverseSquare * pulled) * link.inverseSoft);
            pulled += soft;
            float lambda = rigid * link.inverseWeights + soft;
            if (main) {
                const float moved = dot((m_positions[link.b] - m_previous[link.b]) - (m_positions[link.a] - m_previous[link.a]), direction);
                lambda += link.damping * (moved * link.inverseWeights - lambda);
                if (excess > link.rest * StiffStrain) {
                    for (const std::uint32_t knot : { link.a, link.b })
                        if (m_stiff[knot] == 0) {
                            m_stiff[knot] = 1;
                            m_stiffKnots.push_back(knot);
                        }
                }
            }
            if (link.wa > 0.0f) m_positions[link.a] = m_positions[link.a] + direction * (lambda * link.wa);
            if (link.wb > 0.0f) m_positions[link.b] = m_positions[link.b] - direction * (lambda * link.wb);
        }
    };
    solveLinks((m_stats.substeps & 1u) == 0 ? m_linksBackward : m_links, true);
    // Os cabos: passadas a mais, ida e volta.
    for (std::uint32_t pass = 0; pass < (m_ropeLinks.empty() ? 0u : m_settings.ropeIterations); ++pass)
        solveLinks((pass & 1u) != 0 ? m_ropeLinksBackward : m_ropeLinks, false);
    // Os nos dos cabos puxados para a reta entre as pontas pela tracao.
    for (const RopeSpan& span : m_ropeSpans) {
        float t = 0.0f;
        const Vec3 onLine = closestOnSegment(m_positions[span.knot], m_positions[span.first], m_positions[span.second], t);
        const Vec3 away = m_positions[span.knot] - onLine;
        const float sag = lengthOf(away);
        if (sag < 1e-7f) continue;
        const float w = m_inverseMasses[span.knot];
        const float lambda = sag / (w + span.compliance * inverseSquare);
        m_positions[span.knot] = m_positions[span.knot] - away * (w * lambda / sag);
    }
    // 3. As amarras: a mola do caminho dos fios ate a trave (so puxa).
    for (const std::uint32_t i : m_free) {
        const std::uint32_t anchor = m_tetherAnchor[i];
        if (anchor == NoAnchor) continue;
        const Vec3 delta = m_positions[i] - m_positions[anchor];
        const float lengthSquared = delta.lengthSquared();
        const float limit = m_tetherLength[i];
        if (lengthSquared <= limit * limit) continue;
        const float w = m_inverseMasses[i];
        const float length = std::sqrt(lengthSquared);
        const float lambda = (length - limit) / (w + m_tetherCompliance[i] * inverseSquare);
        m_positions[i] = m_positions[i] - delta * (w * lambda / length);
    }
    const float inverseH = 1.0f / h;
    // 4. O mundo parado (os tubos, o chao) e os nos que sobraram dentro dos
    // corpos. (Os apoiados nos tubos: o primeiro contato usou os do subpasso
    // anterior; a partir daqui, os deste.)
    for (const std::uint32_t knot : m_onFrameKnots) m_onFrame[knot] = Vec3 {};
    m_onFrameKnots.clear();
    collideFrame();
    collideGround();
    // O que os fios puxaram de volta para dentro do corpo: de novo pela massa
    // com a escala, e o no de fio esticado demais nao cede (o fio esticado
    // freia o corpo; nada fica dentro dele).
    collideBodies(h, bodies, proxies, ContactPass::Final);
    // (A bola espremendo a rede contra um tubo passava o no para o outro lado
    // dele, e ele ficava preso la: os tubos de novo, depois dos corpos.)
    collideFrame();
    // Com a bola encostando: rodadas do acoplamento fio-bola. Com as massas de verdade a bola empurra
    // os nos leves mais rapido que a tensao corre pela malha numa passada so; nas rodadas a tensao
    // chega a moldura e o apoio aparece sozinho (de perto no canto do poste a 45 m/s, sem elas o fio
    // esticava 42%; com 1, 14%; com 2, 8-11%; com 4, 5-7%).
    for (std::uint32_t round = 0; round < m_settings.ballContactRounds && !m_sphereContacts.empty(); ++round) {
        solveLinks(((m_stats.substeps + static_cast<std::uint32_t>(round)) & 1u) != 0 ? m_linksBackward : m_links, false);
        collideFrame();
        collideBodies(h, bodies, proxies, ContactPass::Final, true);
        collideFrame();
    }
    // A perda no contato dos outros corpos (a caixa, a capsula), ao longo da normal media (uma vez
    // por corpo). A bola nao entra: a freada dela e a da rede (as massas de verdade e o atrito).
    for (std::size_t b = 0; b < bodies.size() && b < m_contactNormal.size(); ++b) {
        NetBody3D& body = bodies[b];
        const float length = lengthOf(m_contactNormal[b]);
        if (length <= 1e-9f || body.inverseMass <= 0.0f) continue;
        const Vec3 n = m_contactNormal[b] * (1.0f / length);
        const float along = dot(body.velocity, n);
        const float brake = std::min(std::abs(along), m_settings.contactBrake * h);
        const Vec3 loss = n * (along >= 0.0f ? brake : -brake);
        body.velocity = body.velocity - loss;
        body.impulse = body.impulse - loss * (1.0f / body.inverseMass);
    }
    for (const std::uint32_t i : m_free) m_velocities[i] = (m_positions[i] - m_previous[i]) * inverseH;
    // O atrito da bola com a rede, nas velocidades (depois de todas as correcoes de posicao, como
    // no XPBD de corpos): o deslizamento do ponto da bola (o giro incluso) contra o ponto material
    // da rede, ate mu x o impulso normal, com a reacao nos nos. O giro muda so pelo torque disso.
    applyFriction(h, bodies, proxies, false);
}

void NetSolver3D::collideFrame() {
    const float twine = m_settings.twineRadius;
    for (const auto& [i, c] : m_frameNear) {
        const NetCapsule3D& capsule = m_colliders[c];
        const float reach = capsule.radius + twine;
        float t = 0.0f;
        const Vec3 closest = closestOnSegment(m_positions[i], capsule.a, capsule.b, t);
        const Vec3 offset = m_positions[i] - closest;
        const float distanceSquared = offset.lengthSquared();
        // De onde o no veio: num passo grande ele pode atravessar o tubo fino
        // (2,7 cm) e sair pelo outro lado; o caminho do subpasso que passa
        // pelo tubo tambem e colisao, e o no volta para o lado de onde veio.
        float previousT = 0.0f;
        const Vec3 previousClosest = closestOnSegment(m_start[i], capsule.a, capsule.b, previousT);
        const Vec3 previousOffset = m_start[i] - previousClosest;
        const float previousDistance = lengthOf(previousOffset);
        bool hit = distanceSquared < reach * reach;
        if (!hit && previousDistance >= reach) {
            // O caminho cruzou o eixo do tubo? (o ponto do caminho mais perto
            // do eixo dentro do raio).
            const Vec3 motion = m_positions[i] - m_start[i];
            const float motionSquared = motion.lengthSquared();
            if (motionSquared > (reach * reach)) {
                float best = 1e9f;
                for (int k = 1; k < 8; ++k) {
                    const Vec3 sample = m_start[i] + motion * (static_cast<float>(k) / 8.0f);
                    float sampleT = 0.0f;
                    best = std::min(best, (sample - closestOnSegment(sample, capsule.a, capsule.b, sampleT)).lengthSquared());
                }
                hit = best < reach * reach;
            }
        }
        if (!hit) continue;
        Vec3 n;
        if (previousDistance >= reach * 0.5f && previousDistance > 1e-7f) {
            n = previousOffset * (1.0f / previousDistance);
        } else {
            const float distance = std::sqrt(distanceSquared);
            n = distance > 1e-7f ? offset * (1.0f / distance) : Vec3 { 0.0f, 0.0f, 1.0f };
        }
        // (Para o contato da bola: este no esta apoiado no tubo, do lado de n.)
        if (m_onFrame[i].lengthSquared() == 0.0f) m_onFrameKnots.push_back(i);
        m_onFrame[i] = n;
        // Na superficie do tubo, do lado de n (na altura do tubo de agora).
        const Vec3 surface = closest + n * reach;
        const float depth = lengthOf(surface - m_positions[i]);
        m_positions[i] = surface;
        // Atrito no tubo: o deslizamento deste subpasso, ate mu x a correcao.
        Vec3 slide = m_positions[i] - m_start[i];
        slide = slide - n * dot(slide, n);
        const float slideLength = lengthOf(slide);
        if (slideLength > 1e-9f) m_positions[i] = m_positions[i] - slide * std::min(1.0f, FrameFriction * depth / slideLength);
    }
}

void NetSolver3D::collideGround() {
    if (!m_settings.ground) return;
    const float floor = m_settings.groundHeight + m_settings.twineRadius;
    const float mu = m_settings.groundFriction;
    for (const std::uint32_t i : m_free) {
        if (m_positions[i].z >= floor) continue;
        const float depth = floor - m_positions[i].z;
        m_positions[i].z = floor;
        const float dx = m_positions[i].x - m_start[i].x, dy = m_positions[i].y - m_start[i].y;
        const float slide = std::sqrt(dx * dx + dy * dy);
        if (slide <= 1e-9f) continue;
        const float keep = std::max(0.0f, 1.0f - mu * depth / slide);
        m_positions[i].x = m_start[i].x + dx * keep;
        m_positions[i].y = m_start[i].y + dy * keep;
    }
}

void NetSolver3D::collideBodies(float h, std::span<NetBody3D> bodies, std::span<const NetProxy3D> proxies, ContactPass pass, bool ballsOnly) {
    const float twine = m_settings.twineRadius;
    const float inverseH = 1.0f / h;
    // No contato o no conta como mais pesado (ver contactMassScale).
    const float knotScale = pass == ContactPass::StaticNet ? 0.0f : 1.0f / std::max(1.0f, m_settings.contactMassScale);
    // A ordem dos contatos alterna como a dos fios: numa ordem so, o ultimo
    // contato da celula de cada lado decidia, e o chute no meio saia para um
    // lado.
    const bool reverse = (m_stats.substeps & 1u) == 0;
    for (std::uint32_t p = 0; p < proxies.size() && p < m_proxyRanges.size(); ++p) {
        const NetProxy3D& proxy = proxies[p];
        if (proxy.body >= bodies.size()) continue;
        const auto [first, last] = m_proxyRanges[p];
        if (first == last) continue;
        NetBody3D& body = bodies[proxy.body];
        const float wb = body.inverseMass;
        if (ballsOnly && (proxy.shape != NetProxyShape3D::Sphere || body.guided || wb <= 0.0f)) continue;
        // O guiado nao cede: na conta dos nos ele e imovel; o impulso relatado e
        // o que ele levaria solto (lambda com a massa dele). O cinematico (sem
        // massa) tambem passa por cima do no de fio esticado.
        const bool guided = body.guided || wb <= 0.0f;
        const auto pushBody = [&](Vec3 n, float lambda, Vec3 point) {
            ++body.contacts;
            ++m_stats.contacts;
            m_touchSpeed = std::max(m_touchSpeed, lengthOf(body.velocity));
            if (wb <= 0.0f) return;
            float gain = wb * lambda * inverseH;
            gain = std::min(gain, std::max(0.0f, -dot(body.velocity, n)) + MaximumSeparation);
            // (O guiado nao para de verdade e volta a entrar a cada subpasso:
            // conta a media do tick, ~ o impulso de para-lo uma vez.)
            const float magnitude = gain / wb * (guided ? h / std::max(m_tickTime, h) : 1.0f);
            body.impulse = body.impulse + n * magnitude;
            body.impulsePointSum = body.impulsePointSum + point * magnitude;
            body.impulseWeight += magnitude;
            if (guided) {
                // No maximo o de para-lo (e um pouco de volta), somando as
                // celulas: varias celulas da rede esticada davam 6x isso.
                const float cap = (lengthOf(body.velocity) + MaximumSeparation) / wb;
                const float total = lengthOf(body.impulse);
                if (total > cap) body.impulse = body.impulse * (cap / total);
                return;
            }
            if (proxy.body < m_contactImpulse.size()) {
                m_contactImpulse[proxy.body] += magnitude;
                m_contactNormal[proxy.body] = m_contactNormal[proxy.body] + n * magnitude;
            }
            body.position = body.position + n * (wb * lambda);
            body.shift = body.shift + n * (wb * lambda);
            body.velocity = body.velocity + n * gain;
        };
        if (proxy.shape != NetProxyShape3D::Box) {
            // Esfera (o segmento de um ponto) e capsula (o eixo) contra as
            // celulas: o ponto do eixo mais perto da celula faz o contato de
            // esfera. Se o caminho do centro desde o comeco do tick passa pela
            // celula (um passo grande, ou a celula passou por ele), ele volta
            // para o lado de onde veio: a rede nao e atravessada.
            const float reach = proxy.radius + twine;
            const bool sphere = proxy.shape == NetProxyShape3D::Sphere;
            const Vec3 start = m_proxyStarts[p];
            // A caixa do proxy agora (o eixo e o raio) e de onde o centro veio
            // no tick: a celula fora dela nem encosta nem foi cruzada (o teste
            // barato antes da conta do eixo contra o triangulo).
            const Vec3 nowCenter = body.position + body.orientation.rotate(proxy.localPosition);
            const Vec3 nowAxis = sphere ? Vec3 {} : (body.orientation * proxy.localOrientation).rotate({ 0.0f, 0.0f, proxy.halfHeight });
            Box reachBox;
            reachBox.include(nowCenter - nowAxis);
            reachBox.include(nowCenter + nowAxis);
            reachBox.include(start);
            reachBox.expand(reach);
            for (std::uint32_t j = first; j < last; ++j) {
                const std::uint32_t k = reverse ? first + last - 1 - j : j;
                const std::uint32_t f = m_candidates[k];
                const std::array<std::uint32_t, 3>& face = m_faces[f];
                const Vec3 a = m_positions[face[0]], b = m_positions[face[1]], c = m_positions[face[2]];
                if (!reachBox.overlaps({ std::min({ a.x, b.x, c.x }), std::min({ a.y, b.y, c.y }), std::min({ a.z, b.z, c.z }) },
                        { std::max({ a.x, b.x, c.x }), std::max({ a.y, b.y, c.y }), std::max({ a.z, b.z, c.z }) }))
                    continue;
                const Vec3 center = body.position + body.orientation.rotate(proxy.localPosition);
                const Vec3 axis = nowAxis;
                std::array<float, 3> weights {};
                Vec3 point, closest;
                const Vec3 faceNormal = cross(b - a, c - a);
                const float faceArea = lengthOf(faceNormal);
                Vec3 n;
                float depth = 0.0f;
                // (O lado de antes na celula de antes: a rede voltando do
                // estufado mais rapido que a bola passava por cima dela, e a
                // bola, ja do lado de fora, saia empurrada pela rede.)
                const Vec3 a0 = m_tickStart[face[0]], b0 = m_tickStart[face[1]], c0 = m_tickStart[face[2]];
                const Vec3 startNormal = cross(b0 - a0, c0 - a0);
                const float startArea = lengthOf(startNormal);
                const float startHeight = startArea > 1e-12f ? dot(start - a0, startNormal) / startArea : 0.0f;
                const float nowHeight = faceArea > 1e-12f ? dot(center - a, faceNormal) / faceArea : 0.0f;
                bool crossed = false;
                if (startHeight * nowHeight < 0.0f) {
                    // O furo do caminho no plano da celula, dentro dela?
                    const Vec3 hole = start + (center - start) * (startHeight / (startHeight - nowHeight));
                    std::array<float, 3> holeWeights {};
                    const Vec3 onFace = closestOnTriangle(hole, a, b, c, holeWeights);
                    if ((onFace - hole).lengthSquared() < reach * reach) {
                        crossed = true;
                        const float side = startHeight > 0.0f ? 1.0f : -1.0f;
                        n = faceNormal * (side / faceArea);
                        // Ate a parte mais funda do outro lado (a ponta do eixo).
                        const float far = std::max(-side * dot(center - axis - a, faceNormal), -side * dot(center + axis - a, faceNormal)) / faceArea;
                        depth = reach + std::max(0.0f, far);
                        std::array<float, 3> projected {};
                        const Vec3 onPlane = center - faceNormal * (nowHeight / faceArea);
                        closest = closestOnTriangle(onPlane, a, b, c, projected);
                        weights = projected;
                    }
                }
                if (!crossed) {
                    // (O eixo inteiro de um lado do plano da celula, mais longe
                    // que o alcance: nem encosta; a conta exata so para o resto.)
                    if (!sphere && faceArea > 1e-12f) {
                        const float low = dot(center - axis - a, faceNormal) / faceArea, high = dot(center + axis - a, faceNormal) / faceArea;
                        if (low * high > 0.0f && std::min(std::abs(low), std::abs(high)) > reach * 1.001f + 1e-6f) continue;
                    }
                    if (sphere) {
                        point = center;
                        closest = closestOnTriangle(center, a, b, c, weights);
                    } else {
                        closestSegmentTriangle(center - axis, center + axis, a, b, c, point, closest, weights);
                    }
                    const Vec3 offset = point - closest;
                    const float distanceSquared = offset.lengthSquared();
                    if (distanceSquared >= reach * reach) continue;
                    const float distance = std::sqrt(distanceSquared);
                    // (O eixo furando a celula: para o lado de onde o centro veio.)
                    if (distance > 1e-7f) n = offset * (1.0f / distance);
                    else if (faceArea > 1e-12f) n = faceNormal * ((startHeight >= 0.0f ? 1.0f : -1.0f) / faceArea);
                    else n = Vec3 { 0.0f, 0.0f, 1.0f };
                    depth = reach - distance;
                }
                // (Na esfera o fio esticado continua cedendo pela massa: o fio
                // acima do equador da bola, na bolsa da rede, a empurraria para
                // baixo inteira, atravessando a rede. Na capsula, como na caixa,
                // o no de fio esticado demais nao cede no contato final.)
                // A bola (a esfera solta): com as massas de verdade, o impulso nela e nos nos e o
                // mesmo. A escala de massa de contato tirava 3/4 da reacao dos nos (a bola perdia
                // metade da velocidade nos primeiros centimetros, antes de a rede esticar).
                const bool ball = sphere && !guided && wb > 0.0f;
                const float scale = ball ? (pass == ContactPass::StaticNet ? 0.0f : 1.0f) : knotScale;
                std::array<float, 3> shares {};
                std::array<float, 3> moves {};
                float total = wb;
                float knotsOnly = 0.0f;
                for (int corner = 0; corner < 3; ++corner) {
                    const std::size_t c3 = static_cast<std::size_t>(corner);
                    const std::uint32_t knot = face[c3];
                    // Na bola, o no apoiado num tubo nao cede na direcao do tubo: a bola bate na moldura
                    // atraves da rede.
                    const bool backed = ball && dot(m_onFrame[knot], n) > 0.0f;
                    const bool held = (!sphere && pass == ContactPass::Final && m_stiff[knot] != 0) || backed;
                    shares[c3] = weights[c3] * m_inverseMasses[knot] * (held ? 0.0f : scale);
                    total += weights[c3] * shares[c3];
                    // (O guiado empurra ate o no de fio esticado.)
                    moves[c3] = guided ? weights[c3] * m_inverseMasses[knot] * knotScale : shares[c3];
                    knotsOnly += weights[c3] * moves[c3];
                }
                if (total <= 0.0f && knotsOnly <= 0.0f) continue;
                const float lambda = total > 0.0f ? depth / total : 0.0f;
                if (ball) {
                    // So a correcao que a aproximacao deste subpasso cria (relativa ao ponto material
                    // da rede) vira velocidade, na bola e nos nos; o resto e a recuperacao de uma
                    // penetracao (o cruzamento, o passo grande, o que sobrou): so posicao, nos dois
                    // lados (sem isso, 1 cm num subpasso de 0,5 ms viravam 19 m/s).
                    // (A velocidade da superficie: o deslocamento dos nos neste subpasso ate aqui, sem a
                    // recuperacao, coerente com as correcoes que ja entraram.)
                    // (Dormindo, a rede esta parada: o deslocamento guardado e de antes do sono.)
                    Vec3 surface;
                    for (int corner = 0; corner < 3 && pass != ContactPass::StaticNet; ++corner) {
                        const std::uint32_t knot = face[static_cast<std::size_t>(corner)];
                        surface = surface + (m_positions[knot] - m_previous[knot]) * (weights[static_cast<std::size_t>(corner)] * inverseH);
                    }
                    const float approach = std::max(0.0f, -dot(body.velocity - surface, n));
                    const float physical = std::min(depth, approach * h) / total;
                    ++body.contacts;
                    ++m_stats.contacts;
                    m_touchSpeed = std::max(m_touchSpeed, lengthOf(body.velocity));
                    const float magnitude = physical * inverseH;
                    body.impulse = body.impulse + n * magnitude;
                    body.impulsePointSum = body.impulsePointSum + closest * magnitude;
                    body.impulseWeight += magnitude;
                    body.position = body.position + n * (wb * lambda);
                    body.shift = body.shift + n * (wb * lambda);
                    body.velocity = body.velocity + n * (wb * magnitude);
                    for (int corner = 0; corner < 3; ++corner) {
                        const std::uint32_t knot = face[static_cast<std::size_t>(corner)];
                        const Vec3 move = n * shares[static_cast<std::size_t>(corner)];
                        m_positions[knot] = m_positions[knot] - move * lambda;
                        m_previous[knot] = m_previous[knot] - move * (lambda - physical);
                    }
                    m_sphereContacts.push_back({ p, f, weights, n, physical });
                    continue;
                }
                if (total > 0.0f) pushBody(n, lambda, closest);
                const float knotLambda = guided ? (knotsOnly > 0.0f ? depth / knotsOnly : 0.0f) : lambda;
                for (int corner = 0; corner < 3; ++corner)
                    m_positions[face[static_cast<std::size_t>(corner)]] = m_positions[face[static_cast<std::size_t>(corner)]] - n * (moves[static_cast<std::size_t>(corner)] * knotLambda);

            }
            continue;
        }
        const Quaternion orientation = (body.orientation * proxy.localOrientation).normalized();
        const Quaternion inverse = orientation.conjugate();
        for (std::uint32_t j = first; j < last; ++j) {
            const std::uint32_t k = reverse ? first + last - 1 - j : j;
            const NetSample3D& sample = m_samples[m_candidates[k]];
            const Vec3 center = body.position + body.orientation.rotate(proxy.localPosition);
            const Vec3 x = samplePosition(sample, m_positions);
            // Caixa: o no virtual sai pela face mais perto.
            const Vec3 local = inverse.rotate(x - center);
            const Vec3 half = proxy.halfExtents + Vec3 { twine, twine, twine };
            const float px = half.x - std::abs(local.x), py = half.y - std::abs(local.y), pz = half.z - std::abs(local.z);
            if (px <= 0.0f || py <= 0.0f || pz <= 0.0f) continue;
            float depth = 0.0f;
            Vec3 face;
            if (px <= py && px <= pz) {
                depth = px;
                face = { local.x >= 0.0f ? 1.0f : -1.0f, 0.0f, 0.0f };
            } else if (py <= pz) {
                depth = py;
                face = { 0.0f, local.y >= 0.0f ? 1.0f : -1.0f, 0.0f };
            } else {
                depth = pz;
                face = { 0.0f, 0.0f, local.z >= 0.0f ? 1.0f : -1.0f };
            }
            const Vec3 n = orientation.rotate(face);
            // O no de fio esticado demais nao cede no contato final (menos ao
            // guiado, que passa por cima).
            std::array<float, 4> shares {};
            std::array<float, 4> moves {};
            float wk = 0.0f, knotsOnly = 0.0f;
            for (int c = 0; c < 4; ++c) {
                const std::uint32_t knot = sample.knots[c];
                shares[c] = sample.weights[c] * m_inverseMasses[knot] * (pass == ContactPass::Final && m_stiff[knot] != 0 ? 0.0f : knotScale);
                wk += sample.weights[c] * shares[c];
                moves[c] = guided ? sample.weights[c] * m_inverseMasses[knot] * knotScale : shares[c];
                knotsOnly += sample.weights[c] * moves[c];
            }
            const float total = wb + wk;
            if (total <= 0.0f && knotsOnly <= 0.0f) continue;
            const float lambda = total > 0.0f ? depth / total : 0.0f;
            const float knotLambda = guided ? (knotsOnly > 0.0f ? depth / knotsOnly : 0.0f) : lambda;
            for (int c = 0; c < 4; ++c)
                if (moves[c] > 0.0f) m_positions[sample.knots[c]] = m_positions[sample.knots[c]] + n * (moves[c] * knotLambda);
            if (total > 0.0f) pushBody(-n, lambda, x);
        }
    }
}

void NetSolver3D::applyFriction(float h, std::span<NetBody3D> bodies, std::span<const NetProxy3D> proxies, bool staticNet) {
    const float inverseH = 1.0f / h;
    for (const SphereContact& contact : m_sphereContacts) {
        const NetProxy3D& proxy = proxies[contact.proxy];
        NetBody3D& body = bodies[proxy.body];
        const std::array<std::uint32_t, 3>& face = m_faces[contact.face];
        const std::array<float, 3>& weights = contact.weights;
        const Vec3 arm = contact.normal * -proxy.radius;
        const Vec3 bodyVelocity = body.velocity + cross(body.angularVelocity, arm);
        const Vec3 twineVelocity = m_velocities[face[0]] * weights[0] + m_velocities[face[1]] * weights[1] + m_velocities[face[2]] * weights[2];
        const Vec3 relative = bodyVelocity - twineVelocity;
        const Vec3 tangential = relative - contact.normal * dot(relative, contact.normal);
        const float slip = lengthOf(tangential);
        if (slip < 1e-6f) continue;
        const Vec3 direction = tangential * (1.0f / slip);
        float effective = body.inverseMass + body.inverseInertia * proxy.radius * proxy.radius;
        if (!staticNet)
            for (int c = 0; c < 3; ++c) effective += m_inverseMasses[face[c]] * weights[c] * weights[c];
        if (effective <= 0.0f) continue;
        const float impulse = std::min(slip / effective, proxy.friction * contact.lambda * inverseH);
        if (impulse <= 0.0f) continue;
        body.velocity = body.velocity - direction * (body.inverseMass * impulse);
        body.angularVelocity = body.angularVelocity - cross(arm, direction) * (body.inverseInertia * impulse);
        body.impulse = body.impulse - direction * impulse;
        if (!staticNet)
            for (int c = 0; c < 3; ++c) m_velocities[face[c]] = m_velocities[face[c]] + direction * (weights[c] * m_inverseMasses[face[c]] * impulse);
    }
}

void NetSolver3D::endTick(float deltaTime, std::span<const NetBody3D> bodies) {
    static_cast<void>(bodies);
    if (m_sleeping) return;
    // Uma passada nos nos: a velocidade maior, a caixa e se ninguem saiu de
    // perto de onde estava desde a referencia (parada).
    bool still = m_touchSpeed < m_settings.sleepSpeed && m_stillReference.size() == m_positions.size();
    const float limit = m_settings.sleepDistance * m_settings.sleepDistance;
    float fastest = 0.0f;
    Box box;
    for (std::size_t i = 0; i < m_positions.size(); ++i) {
        const Vec3 p = m_positions[i];
        box.include(p);
        fastest = std::max(fastest, m_velocities[i].lengthSquared());
        still = still && (p - m_stillReference[i]).lengthSquared() <= limit;
    }
    m_fastestKnot = std::sqrt(fastest);
    m_stats.maximumSpeed = m_fastestKnot;
    m_boundsMinimum = box.minimum;
    m_boundsMaximum = box.maximum;
    float strain = 0.0f;
    for (const Link& link : m_links) {
        const float lengthSquared = (m_positions[link.b] - m_positions[link.a]).lengthSquared();
        if (lengthSquared <= link.rest * link.rest) continue;
        strain = std::max(strain, (std::sqrt(lengthSquared) - link.rest) / std::max(link.rest, m_strainLength));
    }
    m_stats.maximumStrain = strain;
    if (!still) {
        m_stillReference = m_positions;
        m_stillSeconds = 0.0f;
        return;
    }
    m_stillSeconds += deltaTime;
    if (m_stillSeconds >= m_settings.sleepSeconds) {
        m_sleeping = true;
        m_fastestKnot = 0.0f;
        std::fill(m_velocities.begin(), m_velocities.end(), Vec3 {});
    }
}

float NetSolver3D::settle(float maximumSeconds, float tick, std::uint32_t substeps) {
    float elapsed = 0.0f;
    const std::uint32_t count = std::max<std::uint32_t>(1u, substeps);
    while (elapsed < maximumSeconds && !m_sleeping) {
        beginTick(tick, {}, {});
        for (std::uint32_t k = 0; k < count; ++k) substep(tick / static_cast<float>(count), {}, {});
        endTick(tick, {});
        elapsed += tick;
    }
    return elapsed;
}

} // namespace MatterEngine
