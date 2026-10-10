#include "Engine/Modeling/Parts/ShirtGenerator.hpp"

#include "Engine/Modeling/Mesh/MeshBuilder.hpp"
#include "Engine/Modeling/Mesh/MeshStitch.hpp"
#include "Engine/Modeling/Mesh/TriangleBvh.hpp"
#include "Engine/Modeling/Parts/FacetNoise.hpp"
#include "Engine/Modeling/Parts/LimbFrame.hpp"
#include "Engine/Modeling/Parts/NumberGlyphs.hpp"
#include "Engine/Modeling/Parts/SuperSection.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <map>
#include <optional>

namespace MatterEngine::Modeling {

namespace {

float smooth(float edge0, float edge1, float x) {
    const float t = std::clamp((x - edge0) / (edge1 - edge0), 0.0f, 1.0f);
    return t * t * (3.0f - 2.0f * t);
}

using ArmFrame = LimbFrame;

ArmFrame armFrame(const ShirtPlacement& placement) {
    return limbFrame(placement.shoulder, placement.elbow);
}

// A secao do tronco numa altura (a mesma superelipse do campo).
using TorsoSection = SuperSection;

TorsoSection torsoSection(const ShirtRecipe& recipe, float height, float ground) {
    return SectionCurves { recipe.torsoCenterX, recipe.torsoFront, recipe.torsoBack, recipe.torsoSide, recipe.torsoExponent,
        &recipe.torsoSideExponent }
        .at(height, ground);
}

// O angulo em (low, high) onde value(angulo) vale target (monotono ali).
float solveAngle(const std::function<float(float)>& value, float low, float high, float target) {
    const bool increasing = value(high) > value(low);
    for (int i = 0; i < 48; ++i) {
        const float middle = 0.5f * (low + high);
        if ((value(middle) < target) == increasing) low = middle;
        else high = middle;
    }
    return 0.5f * (low + high);
}

// O comprimento ao longo da secao entre dois angulos (tabela para achar o
// angulo de uma fracao do caminho).
struct ArcTable {
    std::vector<float> angles;
    std::vector<float> lengths;

    [[nodiscard]] float total() const { return lengths.back(); }
    [[nodiscard]] float angleAt(float fraction) const {
        const float target = std::clamp(fraction, 0.0f, 1.0f) * total();
        const auto it = std::lower_bound(lengths.begin(), lengths.end(), target);
        const std::size_t i = std::clamp<std::size_t>(static_cast<std::size_t>(it - lengths.begin()), 1, lengths.size() - 1);
        const float span = lengths[i] - lengths[i - 1];
        const float t = span > 1e-9f ? (target - lengths[i - 1]) / span : 0.0f;
        return angles[i - 1] + (angles[i] - angles[i - 1]) * t;
    }
};

ArcTable arcTable(const TorsoSection& section, float from, float to, int samples = 256) {
    ArcTable table;
    Vec3 previous = section.point(from);
    float length = 0.0f;
    for (int i = 0; i <= samples; ++i) {
        const float angle = from + (to - from) * static_cast<float>(i) / static_cast<float>(samples);
        const Vec3 p = section.point(angle);
        length += (p - previous).length();
        previous = p;
        table.angles.push_back(angle);
        table.lengths.push_back(length);
    }
    return table;
}

// Uma polilinha densa refeita em "steps" passos iguais (as pontas ficam).
std::vector<Vec3> resample(const std::vector<Vec3>& line, int steps) {
    std::vector<float> lengths { 0.0f };
    for (std::size_t i = 1; i < line.size(); ++i) lengths.push_back(lengths.back() + (line[i] - line[i - 1]).length());
    std::vector<Vec3> result;
    for (int k = 0; k <= steps; ++k) {
        const float target = lengths.back() * static_cast<float>(k) / static_cast<float>(steps);
        std::size_t i = 1;
        while (i + 1 < lengths.size() && lengths[i] < target) ++i;
        const float span = lengths[i] - lengths[i - 1];
        const float t = span > 1e-9f ? std::clamp((target - lengths[i - 1]) / span, 0.0f, 1.0f) : 0.0f;
        result.push_back(lerp(line[i - 1], line[i], t));
    }
    result.front() = line.front();
    result.back() = line.back();
    return result;
}

// Os pesos de um vertice: juntas e pesos (soma 1), ate quatro.
class WeightSet {
public:
    void add(int joint, float weight) {
        if (joint < 0 || weight <= 1e-5f) return;
        m_weights[joint] += weight;
    }
    [[nodiscard]] SkinWeights finish() const {
        std::vector<std::pair<float, int>> sorted;
        for (const auto& [joint, weight] : m_weights) sorted.push_back({ weight, joint });
        std::sort(sorted.begin(), sorted.end(), [](const auto& a, const auto& b) { return a.first > b.first; });
        if (sorted.size() > 4) sorted.resize(4);
        float total = 0.0f;
        for (const auto& [weight, joint] : sorted) total += weight;
        SkinWeights skin;
        for (std::size_t i = 0; i < sorted.size(); ++i) {
            skin.joints[i] = static_cast<std::uint16_t>(sorted[i].second);
            skin.weights[i] = total > 0.0f ? sorted[i].first / total : 0.0f;
        }
        return skin;
    }

private:
    std::map<int, float> m_weights;
};

} // namespace

ShapeField buildShirtField(const ShirtRecipe& recipe, const ShirtPlacement& placement) {
    ShapeField field;
    field.add(SectionStack { recipe.torsoCenterX, recipe.torsoFront, recipe.torsoBack, recipe.torsoSide, recipe.torsoExponent,
        placement.ground, recipe.torsoSideExponent });
    // A manga: uma esfera em cada ponto do raio (o casco entre elas), unida
    // ao tronco com mistura (a axila e o alto do ombro).
    const ArmFrame arm = armFrame(placement);
    ConeChain sleeve;
    for (const auto& [s, radius] : recipe.sleeveRadius.points()) {
        sleeve.centers.push_back(arm.at(s, recipe.sleeveForward(s), recipe.sleeveLift(s)));
        sleeve.radii.push_back(radius);
    }
    field.add(std::move(sleeve), recipe.sleeveBlend);
    return field;
}

ShirtSeams buildShirtHalf(Mesh& mesh, const ShirtRecipe& recipe, const ShirtPlacement& placement) {
    ShirtSeams seams;
    seams.part = placement.part;
    seams.sleevePart = "manga_" + placement.side;
    seams.collarPart = "gola";
    if (placement.tuck.size() < 2) {
        seams.error = "camisa: falta o alto do calcao para a dobra da barra";
        return seams;
    }
    const ShapeField field = buildShirtField(recipe, placement);
    const ArmFrame arm = armFrame(placement);
    const float ground = placement.ground;
    MeshBuilder builder(mesh);
    const std::size_t firstVertex = mesh.positions.size();
    builder.material("camisa", recipe.shirtColor);
    builder.material("camisa_faixa", recipe.trimColor);
    builder.material("camisa_dentro", recipe.insideColor);

    // Na superficie (pelo gradiente) e no plano do meio (y = 0 fica).
    const auto onSurface = [&](Vec3 p) { return field.project(p); };
    const auto onMiddle = [&](Vec3 p) {
        p.y = 0.0f;
        for (int i = 0; i < 24; ++i) {
            const float v = field.value(p);
            if (std::abs(v) < 1e-6f) break;
            Vec3 g = field.gradient(p);
            g.y = 0.0f;
            const float g2 = dot(g, g);
            if (g2 < 1e-12f) break;
            Vec3 step = g * (v / g2);
            const float length = step.length();
            if (length > 0.02f) step = step * (0.02f / length);
            p = p - step;
            p.y = 0.0f;
        }
        return p;
    };
    // Onde um raio de fora encontra a superficie (de "from" no rumo dado).
    const auto hit = [&](Vec3 from, Vec3 direction) -> std::optional<Vec3> {
        return field.cross(from, direction, 1.2f, 0.002f);
    };

    // ---- 1. Os aneis do tronco, da barra ate a axila ----
    // Colunas da metade (do meio de tras ao meio da frente): as de tras, o
    // canto de tras da cava (kb), a borda de tras da faixa, a costura, a
    // borda da frente, o canto da frente da cava (kf) e as da frente.
    const int B = recipe.backColumns, F = recipe.frontColumns;
    const int N = B + F + 5;
    const int kb = B, sb = B + 1, sm = B + 2, sf = B + 3, kf = B + 4, cf = N - 1;
    const auto edgeAngles = [&](const TorsoSection& section, float h) {
        const float half = recipe.stripeHalfWidth(h);
        const auto xAt = [&](float a) { return section.point(a).x; };
        return std::array<float, 2> { solveAngle(xAt, Pi + 0.01f, 1.5f * Pi, section.centerX - half),
            solveAngle(xAt, 1.5f * Pi, 2.0f * Pi - 0.01f, section.centerX + half) };
    };
    // Os cantos da cava no anel da axila: onde ele cruza o plano da cava.
    const TorsoSection armpitSection = torsoSection(recipe, recipe.armpitHeight, ground);
    const auto yAt = [&](float a) { return armpitSection.point(a).y; };
    if (armpitSection.side <= recipe.armholeDepth + 0.005f) {
        seams.error = "camisa: o tronco na axila nao chega ao plano da cava (armholeDepth)";
        return seams;
    }
    const float kbAngle = solveAngle(yAt, Pi + 0.01f, 1.5f * Pi, -recipe.armholeDepth);
    const float kfAngle = solveAngle(yAt, 1.5f * Pi, 2.0f * Pi - 0.01f, -recipe.armholeDepth);
    const std::array<float, 2> armpitEdges = edgeAngles(armpitSection, recipe.armpitHeight);
    if (kbAngle >= armpitEdges[0] - 0.02f || kfAngle <= armpitEdges[1] + 0.02f) {
        seams.error = "camisa: a cava e mais estreita que a faixa do lado na axila";
        return seams;
    }
    const float kbFraction = arcTable(armpitSection, Pi, kbAngle).total() / arcTable(armpitSection, Pi, armpitEdges[0]).total();
    const float kfFraction =
        arcTable(armpitSection, armpitEdges[1], kfAngle).total() / arcTable(armpitSection, armpitEdges[1], 2.0f * Pi).total();
    const auto ringAngles = [&](const TorsoSection& section, float h) {
        const std::array<float, 2> edges = edgeAngles(section, h);
        const ArcTable backArc = arcTable(section, Pi, edges[0]);
        const ArcTable frontArc = arcTable(section, edges[1], 2.0f * Pi);
        std::vector<float> angles(static_cast<std::size_t>(N));
        for (int j = 0; j <= B; ++j)
            angles[static_cast<std::size_t>(j)] = backArc.angleAt(kbFraction * static_cast<float>(j) / static_cast<float>(B));
        angles[static_cast<std::size_t>(sb)] = edges[0];
        angles[static_cast<std::size_t>(sm)] = 1.5f * Pi;
        angles[static_cast<std::size_t>(sf)] = edges[1];
        for (int j = 0; j <= F; ++j)
            angles[static_cast<std::size_t>(kf + j)] =
                frontArc.angleAt(kfFraction + (1.0f - kfFraction) * static_cast<float>(j) / static_cast<float>(F));
        angles.front() = Pi;
        angles.back() = 2.0f * Pi;
        return angles;
    };
    std::vector<float> ringHeights { recipe.hemHeight };
    ringHeights.insert(ringHeights.end(), recipe.rings.begin(), recipe.rings.end());
    ringHeights.push_back(recipe.armpitHeight);
    std::vector<std::vector<std::uint32_t>> rings;
    for (std::size_t r = 0; r < ringHeights.size(); ++r) {
        const float h = ringHeights[r];
        const TorsoSection section = torsoSection(recipe, h, ground);
        const std::vector<float> angles = ringAngles(section, h);
        const bool clean = r == 0 || r + 1 == ringHeights.size();
        const float spacing = clean ? 0.0f : std::min(h - ringHeights[r - 1], ringHeights[r + 1] - h);
        std::vector<std::uint32_t> ring;
        for (int k = 0; k < N; ++k) {
            const bool middle = k == 0 || k == cf;
            // O facetado irregular: as colunas comuns andam um pouco ao longo
            // do anel (o meio, a costura e as bordas da faixa ficam).
            float angle = angles[static_cast<std::size_t>(k)];
            if (!clean && !middle && k != sb && k != sm && k != sf) {
                const float step = 0.5f * (angles[static_cast<std::size_t>(k + 1)] - angles[static_cast<std::size_t>(k - 1)]);
                angle += recipe.ringSlide * step * facetNoise(recipe.seed, static_cast<int>(r), k, 2);
            }
            Vec3 p = middle ? onMiddle(section.point(angle)) : onSurface(section.point(angle));
            // O facetado (nunca nas bordas da faixa, no meio ou nos aneis da
            // barra e da axila): para dentro/fora e um pouco para cima/baixo.
            if (!clean) {
                const bool edge = k == sb || k == sf;
                if (!middle && !edge) p = p + field.normal(p) * (recipe.jitter * facetNoise(recipe.seed, static_cast<int>(r), k, 0));
                p.z += std::min(recipe.slideJitter, 0.35f * spacing) * facetNoise(recipe.seed, static_cast<int>(r), k, 1);
            }
            ring.push_back(builder.vertex(p));
        }
        rings.push_back(std::move(ring));
    }
    // A barra nunca entra no calcao: ate a altura da dobra, cada ponto fica
    // ao menos hemClearance por fora dele (no rumo do centro do tronco).
    if (const int shortsPart = mesh.findPart(placement.tuckPart); shortsPart >= 0) {
        std::vector<Triangle> shorts;
        for (const std::uint32_t t : mesh.partTriangles(shortsPart)) shorts.push_back(mesh.triangles[t]);
        const TriangleBvh bvh(mesh.positions, shorts);
        for (std::size_t r = 0; r < rings.size() && ringHeights[r] < recipe.tuckHeight + 0.02f; ++r) {
            const TorsoSection section = torsoSection(recipe, ringHeights[r], ground);
            for (const std::uint32_t v : rings[r]) {
                Vec3& p = mesh.positions[v];
                const Vec3 center { section.centerX, 0.0f, p.z };
                const Vec3 out = safeNormalized({ p.x - center.x, p.y, 0.0f }, { 1.0f, 0.0f, 0.0f });
                // A altura do calcao conferida: a da barra e, acima da dobra, a dela.
                const float probeZ = std::min(p.z, ground + recipe.tuckHeight - 0.002f);
                const Vec3 from = Vec3 { center.x, 0.0f, probeZ } + out * 0.6f;
                const std::optional<TriangleBvh::Hit> hitShorts = bvh.raycast(from, -out, 0.6f);
                if (!hitShorts) continue;
                const float shortsRadius = 0.6f - hitShorts->distance;
                const float radius = std::hypot(p.x - center.x, p.y);
                if (radius < shortsRadius + recipe.hemClearance) {
                    const float z = p.z;
                    p = center + out * (shortsRadius + recipe.hemClearance);
                    p.z = z;
                    if (std::abs(out.y) < 1e-6f) p.y = 0.0f;
                }
            }
        }
    }
    const std::vector<std::uint32_t>& armpit = rings.back();
    const auto position = [&](std::uint32_t v) { return mesh.positions[v]; };

    // ---- 2. A cava: o contorno do corpo no plano dela ----
    // O centro e o eixo da manga nesse plano; cada ponto sai do centro no
    // plano ate deixar o corpo.
    const float planeS = std::abs(arm.along.y) > 1e-4f ? (-recipe.armholeDepth - arm.origin.y) / arm.along.y : 0.0f;
    const Vec3 armholeCenter = arm.at(planeS, recipe.sleeveForward(planeS), recipe.sleeveLift(planeS));
    if (field.value(armholeCenter) >= 0.0f) {
        seams.error = "camisa: o centro da cava fica fora do corpo";
        return seams;
    }
    const auto contour = [&](float angle) -> std::optional<Vec3> {
        return field.cross(armholeCenter, arm.direction(angle), 0.4f, 0.0015f);
    };
    const int rows = recipe.armholeRows;
    const float shoulderAngle = 0.5f * Pi + recipe.shoulderPointAngle * DegreesToRadians;
    float frontStart = arm.angleOf(position(armpit[static_cast<std::size_t>(kf)]), armholeCenter);
    float backStart = arm.angleOf(position(armpit[static_cast<std::size_t>(kb)]), armholeCenter);
    while (backStart < shoulderAngle) backStart += 2.0f * Pi;
    while (frontStart > shoulderAngle) frontStart -= 2.0f * Pi;
    const auto armholeEdge = [&](float from, std::uint32_t corner, std::vector<Vec3>& out) {
        std::vector<Vec3> dense { position(corner) };
        constexpr int Samples = 96;
        for (int i = 1; i <= Samples; ++i) {
            const float angle = from + (shoulderAngle - from) * static_cast<float>(i) / static_cast<float>(Samples);
            const std::optional<Vec3> p = contour(angle);
            if (!p) return false;
            dense.push_back(*p);
        }
        out = resample(dense, rows);
        return true;
    };
    std::vector<Vec3> frontEdge, backEdge;
    if (!armholeEdge(frontStart, armpit[static_cast<std::size_t>(kf)], frontEdge)
        || !armholeEdge(backStart, armpit[static_cast<std::size_t>(kb)], backEdge)) {
        seams.error = "camisa: o contorno da cava nao fecha (o centro dela sai do corpo)";
        return seams;
    }
    const Vec3 shoulderPoint = frontEdge.back();
    backEdge.back() = shoulderPoint;

    // ---- 3. O ponto do pescoco e a costura do ombro ----
    const auto fromAbove = [&](float x, float y) { return hit({ x, y, ground + 2.4f }, { 0.0f, 0.0f, -1.0f }); };
    const std::optional<Vec3> neckPointHit = fromAbove(recipe.neckPointX, -recipe.neckPointSide);
    if (!neckPointHit) {
        seams.error = "camisa: o ponto do pescoco (neckPointX/neckPointSide) fica fora do corpo";
        return seams;
    }
    const Vec3 neckPoint = *neckPointHit;
    const int nb = B + 1;
    std::vector<Vec3> seamLine { neckPoint };
    for (int j = 1; j + 1 < nb; ++j) {
        const float t = static_cast<float>(j) / static_cast<float>(nb - 1);
        const std::optional<Vec3> p = fromAbove(neckPoint.x + (shoulderPoint.x - neckPoint.x) * t,
            neckPoint.y + (shoulderPoint.y - neckPoint.y) * t);
        if (!p) {
            seams.error = "camisa: a costura do ombro sai do corpo";
            return seams;
        }
        seamLine.push_back(*p);
    }
    seamLine.push_back(shoulderPoint);

    // ---- 4. A borda da gola (o V na frente, a volta de tras) ----
    // Na frente, cada ponto (y, altura do V) vem de um raio de frente; atras,
    // de um raio de tras; o fim sobe por cima do ombro ate o ponto do
    // pescoco. Depois tudo refeito em passos iguais.
    const auto neckline = [&](const ProfileCurve& profile, float sign, int steps, std::vector<Vec3>& out) {
        const auto& keys = profile.points();
        const float last = keys.back().first;
        std::vector<Vec3> dense;
        constexpr int Samples = 48;
        for (int i = 0; i <= Samples; ++i) {
            const float side = last * static_cast<float>(i) / static_cast<float>(Samples);
            const std::optional<Vec3> p = hit({ sign * 0.6f, -side, ground + profile(side) }, { -sign, 0.0f, 0.0f });
            if (!p) return false;
            dense.push_back(i == 0 ? onMiddle(*p) : *p);
        }
        // Por cima do ombro: do ultimo ponto ate o do pescoco, na superficie.
        const Vec3 from = dense.back();
        for (int i = 1; i <= 16; ++i) dense.push_back(onSurface(lerp(from, neckPoint, static_cast<float>(i) / 16.0f)));
        dense.back() = neckPoint;
        out = resample(dense, steps);
        out.front().y = 0.0f;
        return true;
    };
    std::vector<Vec3> vEdge, backNeck;
    if (!neckline(recipe.vNeckHeight, 1.0f, recipe.vSteps, vEdge)
        || !neckline(recipe.backNeckHeight, -1.0f, recipe.backNeckSteps, backNeck)) {
        seams.error = "camisa: a borda da gola sai do corpo";
        return seams;
    }
    // O meio da frente da axila ate o fundo do V, e o de tras ate a gola.
    const auto middleLine = [&](float sign, std::uint32_t start, float topHeight, int steps) {
        std::vector<Vec3> line { position(start) };
        const float from = position(start).z;
        for (int i = 1; i < steps; ++i) {
            const float z = from + (topHeight - from) * static_cast<float>(i) / static_cast<float>(steps);
            const std::optional<Vec3> p = hit({ sign * 0.6f, 0.0f, z }, { -sign, 0.0f, 0.0f });
            line.push_back(p ? onMiddle(*p) : onMiddle(lerp(position(start), Vec3 { 0, 0, topHeight }, 0.5f)));
        }
        return line;
    };
    const int frontMiddleSteps = rows - recipe.vSteps, backMiddleSteps = rows - recipe.backNeckSteps;
    if (frontMiddleSteps < 1 || backMiddleSteps < 1) {
        seams.error = "camisa: armholeRows precisa ser maior que vSteps e backNeckSteps";
        return seams;
    }
    std::vector<Vec3> frontMiddle = middleLine(1.0f, armpit[static_cast<std::size_t>(cf)], vEdge.front().z, frontMiddleSteps);
    std::vector<Vec3> backMiddle = middleLine(-1.0f, armpit.front(), backNeck.front().z, backMiddleSteps);

    // Os vertices das bordas dos paineis (os cantos dividos entre eles).
    const std::uint32_t shoulderVertex = builder.vertex(shoulderPoint);
    const std::uint32_t neckVertex = builder.vertex(neckPoint);
    std::vector<std::uint32_t> seamVertices { neckVertex };
    for (std::size_t j = 1; j + 1 < seamLine.size(); ++j) seamVertices.push_back(builder.vertex(seamLine[j]));
    seamVertices.push_back(shoulderVertex);
    const auto edgeVertices = [&](const std::vector<Vec3>& edge, std::uint32_t start) {
        std::vector<std::uint32_t> vertices { start };
        for (std::size_t i = 1; i + 1 < edge.size(); ++i) vertices.push_back(builder.vertex(edge[i]));
        vertices.push_back(shoulderVertex);
        return vertices;
    };
    const std::vector<std::uint32_t> frontArmhole = edgeVertices(frontEdge, armpit[static_cast<std::size_t>(kf)]);
    const std::vector<std::uint32_t> backArmhole = edgeVertices(backEdge, armpit[static_cast<std::size_t>(kb)]);
    // O lado do meio de cada painel: o meio (da axila ate a gola) e a borda
    // da gola ate o ponto do pescoco.
    const auto middleSide = [&](const std::vector<Vec3>& middle, std::uint32_t start, const std::vector<Vec3>& edge) {
        std::vector<std::uint32_t> vertices { start };
        for (std::size_t i = 1; i < middle.size(); ++i) vertices.push_back(builder.vertex(middle[i]));
        for (std::size_t i = 0; i + 1 < edge.size(); ++i) vertices.push_back(builder.vertex(edge[i]));
        vertices.push_back(neckVertex);
        return vertices;
    };
    const std::vector<std::uint32_t> frontSide = middleSide(frontMiddle, armpit[static_cast<std::size_t>(cf)], vEdge);
    const std::vector<std::uint32_t> backSide = middleSide(backMiddle, armpit.front(), backNeck);

    // ---- 5. Os paineis da frente e de tras (Coons na superficie) ----
    // Linhas da axila (i = 0) ate a costura do ombro (i = rows); colunas do
    // meio (j = 0) ate a cava (j = nb - 1).
    using Grid = std::vector<std::vector<std::uint32_t>>;
    const auto buildPanel = [&](const std::vector<std::uint32_t>& bottom, const std::vector<std::uint32_t>& middle,
                                const std::vector<std::uint32_t>& armhole, int panel) {
        std::vector<std::vector<Vec3>> p(static_cast<std::size_t>(rows + 1), std::vector<Vec3>(static_cast<std::size_t>(nb)));
        Grid grid(static_cast<std::size_t>(rows + 1), std::vector<std::uint32_t>(static_cast<std::size_t>(nb), 0));
        for (int j = 0; j < nb; ++j) {
            grid[0][static_cast<std::size_t>(j)] = bottom[static_cast<std::size_t>(j)];
            grid[static_cast<std::size_t>(rows)][static_cast<std::size_t>(j)] = seamVertices[static_cast<std::size_t>(j)];
        }
        for (int i = 0; i <= rows; ++i) {
            grid[static_cast<std::size_t>(i)][0] = middle[static_cast<std::size_t>(i)];
            grid[static_cast<std::size_t>(i)][static_cast<std::size_t>(nb - 1)] = armhole[static_cast<std::size_t>(i)];
        }
        const Vec3 p00 = position(grid[0][0]), p10 = position(grid[0][static_cast<std::size_t>(nb - 1)]);
        const Vec3 p01 = position(grid[static_cast<std::size_t>(rows)][0]);
        const Vec3 p11 = position(grid[static_cast<std::size_t>(rows)][static_cast<std::size_t>(nb - 1)]);
        for (int i = 0; i <= rows; ++i)
            for (int j = 0; j < nb; ++j) {
                const float u = static_cast<float>(j) / static_cast<float>(nb - 1);
                const float v = static_cast<float>(i) / static_cast<float>(rows);
                const bool border = i == 0 || i == rows || j == 0 || j == nb - 1;
                if (border) {
                    p[static_cast<std::size_t>(i)][static_cast<std::size_t>(j)] = position(grid[static_cast<std::size_t>(i)][static_cast<std::size_t>(j)]);
                    continue;
                }
                const Vec3 d = position(grid[0][static_cast<std::size_t>(j)]);
                const Vec3 b = position(grid[static_cast<std::size_t>(rows)][static_cast<std::size_t>(j)]);
                const Vec3 a = position(grid[static_cast<std::size_t>(i)][0]);
                const Vec3 c = position(grid[static_cast<std::size_t>(i)][static_cast<std::size_t>(nb - 1)]);
                const Vec3 coons = d * (1.0f - v) + b * v + a * (1.0f - u) + c * u
                    - (p00 * ((1.0f - u) * (1.0f - v)) + p10 * (u * (1.0f - v)) + p01 * ((1.0f - u) * v) + p11 * (u * v));
                p[static_cast<std::size_t>(i)][static_cast<std::size_t>(j)] = onSurface(coons);
            }
        // Alisamento: cada vertice de dentro vai para a media dos vizinhos e
        // volta para a superficie (o espacamento fica uniforme).
        for (int pass = 0; pass < recipe.smoothing; ++pass) {
            std::vector<std::vector<Vec3>> next = p;
            for (int i = 1; i < rows; ++i)
                for (int j = 1; j + 1 < nb; ++j) {
                    const Vec3 average = (p[static_cast<std::size_t>(i - 1)][static_cast<std::size_t>(j)]
                                             + p[static_cast<std::size_t>(i + 1)][static_cast<std::size_t>(j)]
                                             + p[static_cast<std::size_t>(i)][static_cast<std::size_t>(j - 1)]
                                             + p[static_cast<std::size_t>(i)][static_cast<std::size_t>(j + 1)])
                        * 0.25f;
                    next[static_cast<std::size_t>(i)][static_cast<std::size_t>(j)] =
                        onSurface(lerp(p[static_cast<std::size_t>(i)][static_cast<std::size_t>(j)], average, 0.5f));
                }
            p = std::move(next);
        }
        for (int i = 1; i < rows; ++i)
            for (int j = 1; j + 1 < nb; ++j) {
                const auto at = [&](int a, int b) { return p[static_cast<std::size_t>(a)][static_cast<std::size_t>(b)]; };
                Vec3 q = at(i, j);
                // Ao longo da linha e da coluna (uma fracao do passo) e para fora.
                const Vec3 alongRow = (at(i, j + 1) - at(i, j - 1)) * 0.5f;
                const Vec3 alongColumn = (at(i + 1, j) - at(i - 1, j)) * 0.5f;
                q = onSurface(q + alongRow * (recipe.ringSlide * facetNoise(recipe.seed, 300 + panel * 50 + i, j, 2))
                    + alongColumn * (recipe.ringSlide * facetNoise(recipe.seed, 300 + panel * 50 + i, j, 3)));
                q = q + field.normal(q) * (recipe.jitter * facetNoise(recipe.seed, 300 + panel * 50 + i, j, 0));
                grid[static_cast<std::size_t>(i)][static_cast<std::size_t>(j)] = builder.vertex(q);
            }
        return grid;
    };
    std::vector<std::uint32_t> frontBottom, backBottom;
    for (int j = 0; j < nb; ++j) {
        frontBottom.push_back(armpit[static_cast<std::size_t>(cf - j)]);
        backBottom.push_back(armpit[static_cast<std::size_t>(j)]);
    }
    const Grid frontPanel = buildPanel(frontBottom, frontSide, frontArmhole, 0);
    const Grid backPanel = buildPanel(backBottom, backSide, backArmhole, 1);

    // ---- 6. A manga: da cava ate o punho ----
    // A cava (anti-horaria vista de fora do braco): o canto da frente, a
    // frente ate o ombro, as costas ate o canto de tras e o fundo (a axila).
    std::vector<std::uint32_t> armhole;
    for (int i = 0; i <= rows; ++i) armhole.push_back(frontArmhole[static_cast<std::size_t>(i)]);
    for (int i = rows - 1; i >= 0; --i) armhole.push_back(backArmhole[static_cast<std::size_t>(i)]);
    armhole.push_back(armpit[static_cast<std::size_t>(sb)]);
    armhole.push_back(armpit[static_cast<std::size_t>(sm)]);
    armhole.push_back(armpit[static_cast<std::size_t>(sf)]);
    const std::size_t sleeveCount = armhole.size();
    // Os angulos em volta do eixo (a partir do canto da frente, crescentes);
    // os aneis seguintes vao para angulos iguais aos poucos.
    std::vector<float> armholeAngles;
    for (const std::uint32_t v : armhole) {
        const Vec3 p = position(v);
        const float s = dot(p - arm.origin, arm.along);
        float a = arm.angleOf(p, arm.at(s, recipe.sleeveForward(s), recipe.sleeveLift(s)));
        while (!armholeAngles.empty() && a <= armholeAngles.back()) a += 2.0f * Pi;
        armholeAngles.push_back(a);
    }
    const float tilt = std::tan(recipe.sleeveEndTilt);
    // Um anel da manga na estacao s (no eixo; a ponta inclinada sobe "tilt"
    // por metro de altura), com os angulos misturados "even" para iguais.
    const auto sleeveRing = [&](float station, float even, bool tilted, int ringIndex, bool jitter) {
        std::vector<std::uint32_t> ring;
        for (std::size_t k = 0; k < sleeveCount; ++k) {
            const float uniform = armholeAngles.front() + 2.0f * Pi * static_cast<float>(k) / static_cast<float>(sleeveCount);
            const float angle = armholeAngles[k] + (uniform - armholeAngles[k]) * even;
            const Vec3 direction = arm.direction(angle);
            float s = station;
            if (tilted) s += tilt * recipe.sleeveRadius(station) * std::sin(angle);
            const Vec3 center = arm.at(s, recipe.sleeveForward(s), recipe.sleeveLift(s));
            const std::optional<Vec3> p = field.cross(center, direction, 0.3f, 0.0015f);
            Vec3 q = p ? *p : center + direction * recipe.sleeveRadius(s);
            if (jitter) q = q + field.normal(q) * (recipe.jitter * facetNoise(recipe.seed, 500 + ringIndex, static_cast<int>(k), 0));
            ring.push_back(builder.vertex(q));
        }
        return ring;
    };
    std::vector<std::vector<std::uint32_t>> sleeve { armhole };
    for (std::size_t r = 0; r < recipe.sleeveRings.size(); ++r) {
        const float even = std::min(1.0f, static_cast<float>(r + 1) / 2.0f);
        sleeve.push_back(sleeveRing(recipe.sleeveRings[r], even, false, static_cast<int>(r), true));
    }
    const std::size_t whiteRings = sleeve.size();
    sleeve.push_back(sleeveRing(recipe.sleeveLength - recipe.cuffWidth, 1.0f, true, 90, false));
    sleeve.push_back(sleeveRing(recipe.sleeveLength, 1.0f, true, 91, false));
    // A abertura do punho: no plano da ponta, em volta do eixo do braco.
    std::vector<std::uint32_t> cuff;
    for (std::size_t k = 0; k < sleeveCount; ++k) {
        const float angle = armholeAngles.front() + 2.0f * Pi * static_cast<float>(k) / static_cast<float>(sleeveCount);
        const Vec3 direction = arm.direction(angle);
        const float s = recipe.sleeveLength + tilt * recipe.cuffOpening * std::sin(angle);
        cuff.push_back(builder.vertex(arm.at(s, recipe.cuffOpeningForward, 0.0f) + direction * recipe.cuffOpening));
    }

    // ---- 7. A faixa da gola ----
    // A borda de fora (do fundo do V ao meio de tras, por cima do ombro), a
    // mesma borda levantada, a de dentro levantada (a largura da faixa para
    // o pescoco) e a de dentro descida ate a pele.
    std::vector<std::uint32_t> collarOuter;
    for (int i = frontMiddleSteps; i <= rows; ++i) collarOuter.push_back(frontSide[static_cast<std::size_t>(i)]);
    for (int i = rows - 1; i >= backMiddleSteps; --i) collarOuter.push_back(backSide[static_cast<std::size_t>(i)]);
    const std::size_t collarCount = collarOuter.size();
    std::vector<std::uint32_t> collarRaised, collarInnerRaised, collarInner;
    for (std::size_t i = 0; i < collarCount; ++i) {
        const Vec3 p = position(collarOuter[i]);
        const Vec3 n = field.normal(p);
        Vec3 d;
        if (i == 0 || i + 1 == collarCount) {
            // No meio (o fundo do V, o meio de tras): subindo pela superficie
            // no plano do meio; no V a ponta e mais longa (o chanfro).
            const Vec3 next = position(collarOuter[i == 0 ? 1 : collarCount - 2]);
            const Vec3 tangent = safeNormalized(next - p, { 0.0f, -1.0f, 0.0f });
            const Vec3 across = safeNormalized(cross(tangent, n), { 0.0f, 0.0f, 1.0f }) * (i == 0 ? 1.0f : -1.0f);
            Vec3 mirrored = across;
            mirrored.y = -mirrored.y;
            d = safeNormalized(across + mirrored, { 0.0f, 0.0f, 1.0f });
            const float miter = std::max(0.35f, dot(d, across));
            d = d * (1.0f / miter);
        } else {
            const Vec3 tangent = safeNormalized(position(collarOuter[i + 1]) - position(collarOuter[i - 1]), { 0.0f, -1.0f, 0.0f });
            d = safeNormalized(cross(tangent, n), { 0.0f, 0.0f, 1.0f });
        }
        // Para o pescoco: sempre subindo (a borda de dentro fica acima).
        if (d.z < 0.0f) d = -d;
        Vec3 inner = onSurface(p + d * recipe.collarWidth);
        if (i == 0 || i + 1 == collarCount) inner = onMiddle(inner);
        const Vec3 innerNormal = field.normal(inner);
        // A borda de fora sobe num chanfro (a faixa salta da camisa sem uma
        // parede fina): levantada e um pouco para dentro da faixa.
        Vec3 raised = onSurface(p + d * (2.0f * recipe.collarRaise)) + n * recipe.collarRaise;
        Vec3 innerRaised = inner + innerNormal * recipe.collarRaise;
        Vec3 innerLow = inner - innerNormal * recipe.collarDepth;
        if (i == 0 || i + 1 == collarCount) raised.y = innerRaised.y = innerLow.y = 0.0f;
        collarRaised.push_back(builder.vertex(raised));
        collarInnerRaised.push_back(builder.vertex(innerRaised));
        collarInner.push_back(builder.vertex(innerLow));
    }

    // ---- 8. As faces ----
    builder.part(placement.part, placement.region).connectTo(placement.tuckPart);
    // A dobra da barra: do alto do calcao (por dentro) ate a barra.
    builder.useMaterial("camisa_dentro");
    stitchStrips(builder, placement.tuck, rings.front());
    // O tronco: a faixa do lado entre as bordas.
    for (std::size_t r = 0; r + 1 < rings.size(); ++r)
        for (int k = 0; k + 1 < N; ++k) {
            builder.useMaterial(k == sb || k == sm ? "camisa_faixa" : "camisa");
            const std::size_t a = static_cast<std::size_t>(k), b = static_cast<std::size_t>(k + 1);
            builder.quad(rings[r][a], rings[r][b], rings[r + 1][b], rings[r + 1][a]);
        }
    // Os paineis: a frente anda do meio para o lado (ao contrario dos
    // aneis), as costas no mesmo sentido deles.
    builder.useMaterial("camisa");
    for (int i = 0; i < rows; ++i)
        for (int j = 0; j + 1 < nb; ++j) {
            const std::size_t a = static_cast<std::size_t>(i), b = static_cast<std::size_t>(j);
            builder.quad(frontPanel[a][b + 1], frontPanel[a][b], frontPanel[a + 1][b], frontPanel[a + 1][b + 1]);
            builder.quad(backPanel[a][b], backPanel[a][b + 1], backPanel[a + 1][b + 1], backPanel[a + 1][b]);
        }
    // A manga e o punho (a ponta e o fundo dela pretos).
    builder.part(seams.sleevePart, "braco_" + std::string(placement.side == "direita" ? "direito" : "esquerdo"));
    builder.connectTo(placement.part);
    for (std::size_t r = 0; r + 1 < sleeve.size(); ++r) {
        builder.useMaterial(r + 1 >= whiteRings + 1 ? "camisa_faixa" : "camisa");
        builder.bridge(sleeve[r], sleeve[r + 1]);
    }
    builder.useMaterial("camisa_faixa");
    builder.bridge(sleeve.back(), cuff);
    if (placement.cuffOpening) builder.opening("punho");
    // A gola: a borda de fora sobe ate a faixa, a faixa vai para o pescoco
    // e a borda de dentro desce ate a pele.
    builder.part(seams.collarPart, "pescoco").connectTo(placement.part);
    builder.useMaterial("camisa_faixa");
    for (std::size_t i = 0; i + 1 < collarCount; ++i) {
        builder.quad(collarOuter[i + 1], collarOuter[i], collarRaised[i], collarRaised[i + 1]);
        builder.quad(collarRaised[i + 1], collarRaised[i], collarInnerRaised[i], collarInnerRaised[i + 1]);
        builder.quad(collarInnerRaised[i + 1], collarInnerRaised[i], collarInner[i], collarInner[i + 1]);
    }
    if (placement.collarOpening) builder.opening("pescoco");
    seams.cuff = cuff;
    seams.collar = collarInner;

    // ---- 9. Pesos ----
    // O tronco pela altura na coluna (pelve, abdomen, peito, alto do peito);
    // perto da cava a auxiliar do ombro (metade do giro do braco) e, na
    // manga, o braco.
    if (mesh.skin.size() < mesh.positions.size()) mesh.skin.resize(mesh.positions.size());
    const std::vector<std::pair<int, float>> chain {
        { placement.pelvisJoint, placement.pelvisHeight }, { placement.abdomenJoint, placement.abdomenHeight },
        { placement.chestJoint, placement.chestHeight }, { placement.upperChestJoint, placement.upperChestHeight } };
    // Abaixo do abdomen a cintura passa da pelve para o abdomen numa faixa
    // em volta da barra (a mesma do calcao embaixo: os dois andam juntos
    // quando a coluna dobra); acima, a coluna junta a junta.
    const float waistFrom = recipe.hemHeight - 0.08f, waistTo = recipe.hemHeight + 0.05f;
    const auto spineWeights = [&](WeightSet& set, float h, float share) {
        if (share <= 0.0f) return;
        if (h < chain[1].second) {
            const float abdomen = smooth(waistFrom, waistTo, h);
            set.add(chain[0].first, share * (1.0f - abdomen));
            set.add(chain[1].first, share * abdomen);
            return;
        }
        // Linear entre as juntas (a rampa mais suave: a frente do peito nao
        // amassa tanto quando a coluna enrola).
        for (std::size_t i = 1; i + 1 < chain.size(); ++i) {
            const float a = chain[i].second, b = chain[i + 1].second;
            if (h <= b) {
                const float t = std::clamp((h - a) / (b - a), 0.0f, 1.0f);
                set.add(chain[i].first, share * (1.0f - t));
                set.add(chain[i + 1].first, share * t);
                return;
            }
        }
        set.add(chain.back().first, share);
    };
    for (std::size_t v = firstVertex; v < mesh.positions.size(); ++v) {
        const Vec3 p = mesh.positions[v];
        const float h = p.z - ground;
        const float s = dot(p - arm.origin, arm.along);
        const Vec3 radial = p - arm.at(s, 0.0f, 0.0f);
        const float distanceFromArm = radial.length();
        // Perto do braco (o ombro e a manga, ate uns 12 cm do eixo, sumindo
        // ate 20): fora disso (o lado do tronco la embaixo) nada do braco.
        const float nearArm = 1.0f - smooth(0.12f, 0.2f, distanceFromArm);
        const float u = s - planeS;
        // Embaixo do eixo (a axila): quando o braco desce, o que fica sob o
        // ombro gira para dentro do corpo e amassaria a axila; ali a auxiliar
        // pesa menos (a pele so fecha a dobra).
        const float down = std::max(0.0f, -dot(safeNormalized(radial, arm.up), arm.up));
        // A manga: da cava ao braco inteiro (uns 10 cm; embaixo, na axila,
        // uns 20: a dobra quando o braco desce ou vai para tras se espalha e
        // a manga nao entra na camisa).
        const float ramp = 0.1f + 0.1f * down;
        const float upperArm = placement.upperArmJoint >= 0 ? nearArm * smooth(0.0f, ramp, u) : 0.0f;
        // A auxiliar (a metade do giro do braco): o ombro em volta da cava,
        // entrando uns 14 cm no tronco (o peitoral e as costas acompanham o
        // braco aos poucos: com a rampa curta a frente da cava se cruza quando
        // o braco vai a frente); a manga passa dela para o braco.
        const float nearShoulder = nearArm * smooth(-0.14f, 0.0f, u);
        const float peak = 0.9f - 0.55f * down * down;
        const float helper = placement.shoulderHelperJoint >= 0 ? (1.0f - upperArm) * peak * nearShoulder : 0.0f;
        WeightSet set;
        set.add(placement.upperArmJoint, upperArm);
        set.add(placement.shoulderHelperJoint, helper);
        // A gola e da camisa (nao gira com o pescoco: a pele dele dobra
        // acima dela).
        spineWeights(set, h, 1.0f - upperArm - helper);
        mesh.skin[v] = set.finish();
    }
    seams.ok = true;
    return seams;
}

bool imprintShirtNumbers(Mesh& mesh, const ShirtRecipe& recipe, float ground, const std::string& part, std::string* error) {
    if (recipe.number.empty()) return true;
    const int partIndex = mesh.findPart(part);
    if (partIndex < 0) {
        if (error != nullptr) *error = "numero: a camisa nao existe";
        return false;
    }
    MeshBuilder builder(mesh);
    builder.material("camisa_numero", recipe.numberColor);
    const int numberMaterial = mesh.findMaterial("camisa_numero");
    const int shirtMaterial = mesh.findMaterial("camisa");
    // Na frente quem olha tem a esquerda do jogador a direita (+y); nas
    // costas, a direita dele (-y). Desenhados num plano e projetados.
    struct Side {
        float x;
        float direction;
        float height;
        float center;
    };
    for (const Side& side : { Side { 0.4f, 1.0f, recipe.frontNumberHeight, recipe.frontNumberCenter },
             Side { -0.5f, -1.0f, recipe.backNumberHeight, recipe.backNumberCenter } }) {
        NumberLayout layout;
        layout.center = { side.x, 0.0f, ground + side.center };
        layout.right = { 0.0f, side.direction, 0.0f };
        layout.height = side.height;
        layout.stroke = recipe.numberStroke;
        const std::vector<ImprintLoop> loops = numberLoops(recipe.number, layout, numberMaterial, shirtMaterial);
        ImprintSettings settings;
        settings.viewDirection = { side.direction, 0.0f, 0.0f };
        const ImprintResult cut = imprintLoops(mesh, mesh.partTriangles(partIndex), loops, settings);
        if (!cut.ok) {
            if (error != nullptr) *error = std::string("numero da camisa (") + (side.direction > 0.0f ? "frente" : "costas") + "): " + cut.error;
            return false;
        }
    }
    return true;
}

} // namespace MatterEngine::Modeling
