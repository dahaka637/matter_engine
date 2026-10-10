#include "Engine/Modeling/Parts/HandGenerator.hpp"

#include "Engine/Modeling/Mesh/MeshBuilder.hpp"
#include "Engine/Modeling/Mesh/MeshStitch.hpp"
#include "Engine/Modeling/Parts/FacetNoise.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <span>
#include <utility>

namespace MatterEngine::Modeling {

namespace {

constexpr std::uint32_t NoVertex = std::numeric_limits<std::uint32_t>::max();
constexpr std::size_t DigitPoints = 8;
constexpr std::size_t FingerCount = 4;

float smooth(float edge0, float edge1, float x) {
    const float t = std::clamp((x - edge0) / (edge1 - edge0), 0.0f, 1.0f);
    return t * t * (3.0f - 2.0f * t);
}

float bump(float x) {
    x = std::abs(x);
    return x >= 1.0f ? 0.0f : (1.0f - x * x) * (1.0f - x * x);
}

// O angulo em [0, 2pi).
float positiveAngle(float angle) {
    return angle - 2.0f * Pi * std::floor(angle / (2.0f * Pi));
}

// Os oito pontos em volta de um dedo: o octogono com uma face em cima (a
// unha) e outra embaixo. 0 = o lado do polegar, pi/2 = o dorso.
float digitAngle(std::size_t k) {
    return (22.5f + 45.0f * static_cast<float>(k % DigitPoints)) * DegreesToRadians;
}

// O referencial da mao: ao longo (do pulso ao medio), o lado do polegar e o
// dorso. Os aneis vao no sentido anti-horario vistos dos dedos.
struct HandFrame {
    Vec3 origin;
    Vec3 along;
    Vec3 radial;
    Vec3 dorsal;
    [[nodiscard]] Vec3 at(float s, Point2 section) const {
        return origin + along * s + radial * section.x + dorsal * section.y;
    }
    [[nodiscard]] float station(Vec3 p) const { return dot(p - origin, along); }
    [[nodiscard]] Point2 section(Vec3 p) const { return { dot(p - origin, radial), dot(p - origin, dorsal) }; }
};

// A secao da palma numa estacao: superelipse de quatro quadrantes.
struct PalmSection {
    float radial = 0.0f;
    float ulnar = 0.0f;
    float back = 0.0f;
    float palm = 0.0f;
    float exponent = 2.0f;
    [[nodiscard]] Point2 point(float angle) const {
        const float c = std::cos(angle), sn = std::sin(angle);
        const float sum = std::pow(std::abs(c) / (c >= 0.0f ? radial : ulnar), exponent)
            + std::pow(std::abs(sn) / (sn >= 0.0f ? back : palm), exponent);
        const float radius = std::pow(std::max(sum, 1e-20f), -1.0f / exponent);
        return { c * radius, sn * radius };
    }
};

PalmSection palmSection(const HandRecipe& recipe, float s) {
    return { recipe.radial(s), recipe.ulnar(s), recipe.back(s), recipe.palm(s), recipe.exponent(s) };
}

float length2(Point2 a, Point2 b) {
    return std::hypot(a.x - b.x, a.y - b.y);
}

// Os pontos do contorno da secao nas fracoes do comprimento dadas, a partir
// do angulo "start" e no sentido anti-horario.
std::vector<Point2> sectionAtFractions(const PalmSection& section, float start, std::span<const float> fractions) {
    constexpr int Samples = 720;
    std::vector<Point2> points(Samples + 1);
    std::vector<float> cumulative(Samples + 1, 0.0f);
    for (int i = 0; i <= Samples; ++i) {
        points[static_cast<std::size_t>(i)] = section.point(start + 2.0f * Pi * static_cast<float>(i) / Samples);
        if (i > 0)
            cumulative[static_cast<std::size_t>(i)] = cumulative[static_cast<std::size_t>(i - 1)]
                + length2(points[static_cast<std::size_t>(i)], points[static_cast<std::size_t>(i - 1)]);
    }
    std::vector<Point2> result;
    for (const float fraction : fractions) {
        const float target = std::clamp(fraction, 0.0f, 1.0f) * cumulative.back();
        const auto upper = std::upper_bound(cumulative.begin(), cumulative.end(), target);
        const std::size_t i = std::min<std::size_t>(static_cast<std::size_t>(std::max<std::ptrdiff_t>(upper - cumulative.begin() - 1, 0)),
            Samples - 1);
        const float span = cumulative[i + 1] - cumulative[i];
        const float t = span > 1e-9f ? (target - cumulative[i]) / span : 0.0f;
        result.push_back({ points[i].x + (points[i + 1].x - points[i].x) * t, points[i].y + (points[i + 1].y - points[i].y) * t });
    }
    return result;
}

// O referencial de um anel de dedo: o centro, o eixo, o lado (angulo 0) e o
// dorso (pi/2).
struct DigitFrame {
    Vec3 center;
    Vec3 along;
    Vec3 side;
    Vec3 dorsal;
    [[nodiscard]] Vec3 direction(float angle) const { return side * std::cos(angle) + dorsal * std::sin(angle); }
    [[nodiscard]] float angleOf(Vec3 p) const {
        const Vec3 d = p - center;
        return std::atan2(dot(d, dorsal), dot(d, side));
    }
};

DigitFrame digitFrame(Vec3 center, Vec3 along, Vec3 dorsalHint, float roll = 0.0f) {
    DigitFrame frame;
    frame.center = center;
    frame.along = safeNormalized(along, { 0.0f, -1.0f, 0.0f });
    Vec3 dorsal = dorsalHint - frame.along * dot(dorsalHint, frame.along);
    dorsal = safeNormalized(dorsal, anyPerpendicular(frame.along));
    if (roll != 0.0f) dorsal = Quaternion::fromAxisAngle(frame.along, roll).rotate(dorsal);
    frame.dorsal = dorsal;
    frame.side = cross(frame.dorsal, frame.along);
    return frame;
}

// O raio de um dedo no angulo: superelipse com a meia largura e a meia
// espessura (quase um quadrado arredondado).
float digitRadius(float angle, float width, float thickness) {
    constexpr float Exponent = 2.4f;
    const float sum = std::pow(std::abs(std::cos(angle)) / width, Exponent)
        + std::pow(std::abs(std::sin(angle)) / thickness, Exponent);
    return std::pow(std::max(sum, 1e-20f), -1.0f / Exponent);
}

// A medida ao longo do dedo: 0 na base, 1 na junta do meio, 2 na da ponta,
// 3 perto da ponta.
float sizeAt(const std::array<float, 4>& values, float position) {
    const float clamped = std::clamp(position, 0.0f, 3.0f);
    const std::size_t i = std::min<std::size_t>(static_cast<std::size_t>(clamped), 2);
    return values[i] + (values[i + 1] - values[i]) * (clamped - static_cast<float>(i));
}

// O skin de um vertice: as juntas somadas, as quatro mais fortes,
// normalizadas (sem juntas validas, fica vazio).
using Influences = std::vector<std::pair<int, float>>;

void writeSkin(Mesh& mesh, std::uint32_t vertex, const Influences& influences) {
    Influences merged;
    for (const auto& [joint, weight] : influences) {
        if (joint < 0 || weight <= 1e-4f) continue;
        const auto same = std::find_if(merged.begin(), merged.end(), [&](const auto& entry) { return entry.first == joint; });
        if (same != merged.end()) same->second += weight;
        else merged.emplace_back(joint, weight);
    }
    std::stable_sort(merged.begin(), merged.end(), [](const auto& a, const auto& b) { return a.second > b.second; });
    if (merged.size() > 4) merged.resize(4);
    float total = 0.0f;
    for (const auto& entry : merged) total += entry.second;
    if (mesh.skin.size() < mesh.positions.size()) mesh.skin.resize(mesh.positions.size());
    SkinWeights& weights = mesh.skin[vertex];
    weights = {};
    for (std::size_t i = 0; i < merged.size(); ++i) {
        weights.joints[i] = static_cast<std::uint16_t>(merged[i].first);
        weights.weights[i] = total > 0.0f ? merged[i].second / total : 0.0f;
    }
}

// Um ponto planejado (posicao e skin) antes de virar vertice.
struct Planned {
    Vec3 position;
    Influences influences;
};

} // namespace

HandSeams buildHand(Mesh& mesh, const HandRecipe& recipe, const HandPlacement& placement) {
    HandSeams seams;
    seams.part = placement.part;
    if (placement.wristRing.size() < 3) {
        seams.error = "mao: falta o anel do pulso do antebraco";
        return seams;
    }
    const auto& fingers = placement.fingers;
    const DigitPlacement& thumb = placement.thumb;

    // ---- 1. O referencial da mao ----
    HandFrame frame;
    frame.origin = placement.wrist;
    frame.along = safeNormalized(fingers[1].points[0] - placement.wrist, { 0.0f, -1.0f, 0.0f });
    const Vec3 across = fingers[0].points[0] - fingers[3].points[0];
    frame.radial = safeNormalized(across - frame.along * dot(across, frame.along), { 1.0f, 0.0f, 0.0f });
    frame.dorsal = cross(frame.along, frame.radial);
    if (dot(frame.dorsal, fingers[1].dorsal[0]) <= 0.0f) {
        seams.error = "mao: o dorso dos dedos nao bate com o lado do polegar (a mao e a direita)";
        return seams;
    }
    std::array<float, FingerCount> fs {}, length {};
    std::array<Point2, FingerCount> fp {};
    for (std::size_t i = 0; i < FingerCount; ++i) {
        fs[i] = frame.station(fingers[i].points[0]);
        fp[i] = frame.section(fingers[i].points[0]);
        length[i] = distance(fingers[i].points[1], fingers[i].points[0]);
        if (i > 0 && fp[i].x > fp[i - 1].x - 0.008f) {
            seams.error = "mao: os dedos fora de ordem (do indicador ao minimo, a partir do polegar)";
            return seams;
        }
    }
    const int hand = placement.handJoint;
    const auto base = [&](std::size_t i) { return fingers[i].joints[0]; };
    // A prega da base de cada dedo, na palma (um pouco antes da forquilha).
    const auto crease = [&](std::size_t i) { return fs[i] + 0.8f * recipe.webDepth * length[i]; };
    const auto handPoint = [&](float s, float u, float h) { return frame.at(s, { u, h }); };

    // ---- 2. A linha dos nos: o ultimo anel da palma ----
    // Cada dedo deixa nela o alto (os nos, com os lados) e a palma (na prega
    // da base); o indicador e o minimo, tambem o lado de baixo da borda.
    // Entre os dedos, os vales do dorso e da palma. Ordem (anti-horaria
    // vista dos dedos): a borda do polegar, o dorso do indicador ao minimo, a
    // borda do minimo, a palma do minimo ao indicador.
    std::array<std::array<Vec3, DigitPoints>, FingerCount> knuckle {};
    std::array<float, FingerCount> reachRadial {}, reachUlnar {};
    for (std::size_t i = 0; i < FingerCount; ++i) {
        const float wk = recipe.knuckleWidth * recipe.fingers[i].width[0];
        const float radialGap = i > 0 ? 0.5f * (fp[i - 1].x - fp[i].x) : 1.0f;
        const float ulnarGap = i + 1 < FingerCount ? 0.5f * (fp[i].x - fp[i + 1].x) : 1.0f;
        reachRadial[i] = std::min(0.92f * wk, 0.85f * radialGap) + (i == 0 ? recipe.palmEdge : 0.0f);
        reachUlnar[i] = std::min(0.92f * wk, 0.85f * ulnarGap) + (i + 1 == FingerCount ? recipe.palmEdge : 0.0f);
        const float s = fs[i], back = recipe.back(s), u = fp[i].x, h = fp[i].y;
        knuckle[i][0] = handPoint(s, u + reachRadial[i], h + 0.72f * back);
        knuckle[i][1] = handPoint(s, u + 0.4f * wk, h + back + recipe.knuckleBump);
        knuckle[i][2] = handPoint(s, u - 0.4f * wk, h + back + recipe.knuckleBump);
        knuckle[i][3] = handPoint(s, u - reachUlnar[i], h + 0.72f * back);
        // A prega da base do dedo: a palma desce ate perto da espessura do
        // dedo (sem degrau).
        const float sc = crease(i), w = recipe.fingers[i].width[0];
        const float depth = recipe.palm(sc) + 0.35f * (recipe.fingers[i].thickness[0] - recipe.palm(sc));
        knuckle[i][5] = handPoint(sc, u - 0.42f * w, h - depth);
        knuckle[i][6] = handPoint(sc, u + 0.42f * w, h - depth);
    }
    {
        const float s0 = 0.5f * (fs[0] + crease(0)), s3 = 0.5f * (fs[3] + crease(3));
        knuckle[0][7] = handPoint(s0, fp[0].x + reachRadial[0], fp[0].y - 0.45f * recipe.palm(s0));
        knuckle[3][4] = handPoint(s3, fp[3].x - reachUlnar[3], fp[3].y - 0.45f * recipe.palm(s3));
    }
    std::vector<Planned> line;
    // O ponto de cada dedo no anel (o indice em "line"; -1 = fora dele: os
    // da membrana).
    std::array<std::array<int, DigitPoints>, FingerCount> onLine {};
    for (auto& fingerSlots : onLine) fingerSlots.fill(-1);
    const auto pushFinger = [&](std::size_t i, std::size_t k, float handWeight) {
        onLine[i][k] = static_cast<int>(line.size());
        line.push_back({ knuckle[i][k], { { hand, handWeight }, { base(i), 1.0f - handWeight } } });
    };
    pushFinger(0, 7, 0.5f);
    for (std::size_t i = 0; i < FingerCount; ++i) {
        for (std::size_t k = 0; k < 4; ++k) pushFinger(i, k, 0.55f);
        if (i + 1 < FingerCount) {
            const Vec3 middle = (knuckle[i][3] + knuckle[i + 1][0]) * 0.5f;
            line.push_back({ middle - frame.dorsal * (0.14f * recipe.back(frame.station(middle))),
                { { hand, 0.7f }, { base(i), 0.15f }, { base(i + 1), 0.15f } } });
        }
    }
    pushFinger(3, 4, 0.5f);
    for (std::size_t n = FingerCount; n-- > 0;) {
        pushFinger(n, 5, 0.4f);
        pushFinger(n, 6, 0.4f);
        if (n > 0) {
            const Vec3 middle = (knuckle[n][6] + knuckle[n - 1][5]) * 0.5f;
            line.push_back({ middle + frame.dorsal * (0.08f * recipe.palm(frame.station(middle))),
                { { hand, 0.6f }, { base(n), 0.2f }, { base(n - 1), 0.2f } } });
        }
    }
    const std::size_t slots = line.size();

    // ---- 3. Os aneis da palma ----
    // Cada ponto do anel dos nos tem a sua fracao do contorno (a partir da
    // borda do polegar); os aneis da palma poem os pontos nas mesmas
    // fracoes do contorno da secao deles e, chegando perto dos nos, vao
    // tomando a forma da linha dos nos (o arco dos nos, a prega da palma).
    std::vector<float> fractions(slots, 0.0f);
    float perimeter = 0.0f;
    for (std::size_t k = 0; k < slots; ++k) {
        fractions[k] = perimeter;
        perimeter += length2(frame.section(line[k].position), frame.section(line[(k + 1) % slots].position));
    }
    for (float& fraction : fractions) fraction /= perimeter;
    const Point2 startPoint = frame.section(line[0].position);
    const float start = std::atan2(startPoint.y, startPoint.x);
    float lineStation = 0.0f;
    for (const Planned& point : line) lineStation += frame.station(point.position) / static_cast<float>(slots);
    const std::vector<Point2> lineBase = sectionAtFractions(palmSection(recipe, lineStation), start, fractions);
    const std::size_t ringCount = recipe.palmRings.size();
    std::vector<std::vector<Vec3>> palm(ringCount);
    for (std::size_t r = 0; r < ringCount; ++r) {
        const float s = recipe.palmRings[r];
        const float toward = std::clamp((s - recipe.palmRings.front()) / (lineStation - recipe.palmRings.front()), 0.0f, 1.0f);
        const float follow = toward * toward;
        const std::vector<Point2> section = sectionAtFractions(palmSection(recipe, s), start, fractions);
        for (std::size_t k = 0; k < slots; ++k) {
            const Point2 target = frame.section(line[k].position);
            const Point2 point { section[k].x + follow * (target.x - lineBase[k].x),
                section[k].y + follow * (target.y - lineBase[k].y) };
            palm[r].push_back(frame.at(s + follow * (frame.station(line[k].position) - lineStation), point));
        }
    }

    // ---- 4. A janela do polegar ----
    // Tres aneis seguidos (a, b, c) e tres pontos em volta (j0, j1, j2): o
    // do meio (b, j1) e onde o eixo do polegar cruza a palma; sem os quatro
    // quads em volta dele, a borda da janela tem oito pontos.
    const std::size_t ra = static_cast<std::size_t>(recipe.thumbFromRing), rb = ra + 1, rc = ra + 2;
    const Vec3 thumbBase = thumb.points[0], thumbKnuckle = thumb.points[1];
    const auto thumbAxisAt = [&](float s) {
        const float s0 = frame.station(thumbBase), s1 = frame.station(thumbKnuckle);
        const float t = std::abs(s1 - s0) > 1e-5f ? std::clamp((s - s0) / (s1 - s0), 0.0f, 1.0f) : 0.0f;
        return thumbBase + (thumbKnuckle - thumbBase) * t;
    };
    std::size_t j1 = 0;
    {
        const Vec3 crossing = thumbAxisAt(recipe.palmRings[rb]);
        float best = std::numeric_limits<float>::max();
        for (std::size_t k = 0; k < slots; ++k)
            if (const float d = distance(palm[rb][k], crossing); d < best) {
                best = d;
                j1 = k;
            }
    }
    const std::size_t j0 = (j1 + slots - 1) % slots, j2 = (j1 + 1) % slots;
    const auto thumbSegmentDistance = [&](Vec3 p) {
        const Vec3 axis = thumbKnuckle - thumbBase;
        const float t = std::clamp(dot(p - thumbBase, axis) / std::max(dot(axis, axis), 1e-9f), 0.0f, 1.0f);
        return distance(p, thumbBase + axis * t);
    };

    // ---- 5. Os vertices da palma e da linha dos nos ----
    MeshBuilder builder(mesh);
    builder.material("unha", recipe.nailColor);
    builder.material("pele", recipe.skinColor);
    builder.part(placement.part, placement.region).connectTo(placement.forearmPart);
    builder.useMaterial("pele");
    const auto vertex = [&](Vec3 p, const Influences& influences) {
        const std::uint32_t v = builder.vertex(p);
        writeSkin(mesh, v, influences);
        return v;
    };
    const auto outward = [&](Vec3 p) {
        const Point2 q = frame.section(p);
        return safeNormalized(frame.radial * q.x + frame.dorsal * q.y, frame.dorsal);
    };
    // A eminencia tenar: a palma salta em volta do metacarpo do polegar (ate
    // uns 3 cm dele).
    constexpr float ThenarReach = 0.03f;
    std::vector<std::vector<std::uint32_t>> rings(ringCount);
    for (std::size_t r = 0; r < ringCount; ++r) {
        for (std::size_t k = 0; k < slots; ++k) {
            if (r == rb && k == j1) {
                rings[r].push_back(NoVertex);
                continue;
            }
            Vec3 p = palm[r][k];
            const float d = thumbSegmentDistance(p);
            p = p + outward(p) * (recipe.thenarBulge * bump(d / ThenarReach)
                    + recipe.jitter * facetNoise(recipe.seed, static_cast<int>(r), static_cast<int>(k), 0));
            const float s = frame.station(p);
            const float wrist = 1.0f - smooth(0.0f, recipe.palmRings[std::min<std::size_t>(1, ringCount - 1)], s);
            // A eminencia tenar anda com o metacarpo do polegar (a janela, uns
            // 60%), sumindo uns 3,5 cm em volta dele.
            const float thenar = 0.75f * bump(d / (1.15f * ThenarReach));
            rings[r].push_back(vertex(p, { { placement.wristHelperJoint, wrist * (1.0f - thenar) },
                                             { hand, (1.0f - wrist) * (1.0f - thenar) }, { thumb.joints[0], thenar } }));
        }
    }
    std::vector<std::uint32_t> lineRing;
    for (std::size_t k = 0; k < slots; ++k) {
        const Vec3 p = line[k].position
            + outward(line[k].position) * (0.5f * recipe.jitter * facetNoise(recipe.seed, 40, static_cast<int>(k), 0));
        lineRing.push_back(vertex(p, line[k].influences));
    }

    // ---- 6. A palma: do pulso aos nos, sem a janela ----
    stitchLoops(builder, placement.wristRing, rings.front());
    for (std::size_t r = 0; r + 1 < ringCount; ++r)
        for (std::size_t k = 0; k < slots; ++k) {
            const std::size_t next = (k + 1) % slots;
            if ((r == ra || r == rb) && (k == j0 || k == j1)) continue;
            builder.quad(rings[r][k], rings[r][next], rings[r + 1][next], rings[r + 1][k]);
        }
    builder.bridge(rings.back(), lineRing);

    // ---- 7. Os dedos ----
    // A volta da base de cada dedo: os pontos da linha dos nos e, entre os
    // dedos, os da membrana (na forquilha, embaixo dos lados).
    std::array<std::array<std::uint32_t, DigitPoints>, FingerCount> loops {};
    for (std::size_t i = 0; i < FingerCount; ++i) {
        const DigitPlacement& finger = fingers[i];
        const DigitFrame root = digitFrame(finger.points[0], finger.points[1] - finger.points[0], finger.dorsal[0]);
        for (std::size_t k = 0; k < DigitPoints; ++k) {
            if (onLine[i][k] >= 0) {
                loops[i][k] = lineRing[static_cast<std::size_t>(onLine[i][k])];
                continue;
            }
            const float angle = digitAngle(k);
            const float width = recipe.fingers[i].width[0], thickness = recipe.fingers[i].thickness[0];
            const Vec3 p = root.center + root.along * (recipe.webDepth * length[i])
                + root.direction(angle) * digitRadius(angle, width, thickness);
            loops[i][k] = vertex(p, { { hand, 0.35f }, { base(i), 0.65f } });
        }
    }
    // As membranas: entre o dedo A (do lado do polegar) e o B, o vale do
    // dorso e o da palma descem ate a forquilha (a aresta entre o lado de
    // baixo de A e o de B).
    for (std::size_t i = 0; i + 1 < FingerCount; ++i) {
        const auto& a = loops[i];
        const auto& b = loops[i + 1];
        const std::uint32_t dorsalValley = lineRing[static_cast<std::size_t>(onLine[i][3] + 1)];
        const std::uint32_t palmarValley = lineRing[static_cast<std::size_t>(onLine[i + 1][6] + 1)];
        builder.triangle(a[3], dorsalValley, a[4]);
        builder.triangle(dorsalValley, b[0], b[7]);
        builder.triangle(dorsalValley, b[7], a[4]);
        builder.triangle(b[6], palmarValley, b[7]);
        builder.triangle(palmarValley, a[5], a[4]);
        builder.triangle(palmarValley, a[4], b[7]);
    }
    // Os tubos: o meio da primeira falange, as juntas (com o no em cima e a
    // prega embaixo), o meio das outras e a ponta arredondada.
    const auto digitRing = [&](const DigitFrame& ring, std::span<const float> angles, float width, float thickness,
                               float knot, int salt, const Influences& influences) {
        std::vector<std::uint32_t> result;
        for (std::size_t k = 0; k < angles.size(); ++k) {
            const float angle = angles[k];
            const float lift = std::sin(angle);
            const float extra = lift > 0.5f ? knot : lift < -0.5f ? -0.4f * knot : 0.0f;
            const Vec3 direction = ring.direction(angle);
            const float radius = digitRadius(angle, width, thickness) + extra
                + 0.5f * recipe.jitter * facetNoise(recipe.seed, salt, static_cast<int>(k), 1);
            result.push_back(vertex(ring.center + direction * radius, influences));
        }
        return result;
    };
    std::array<float, DigitPoints> uniform {};
    for (std::size_t k = 0; k < DigitPoints; ++k) uniform[k] = digitAngle(k);
    const auto finishDigit = [&](const DigitPlacement& digit, const FingerSize& size, std::vector<std::uint32_t> start,
                                 std::span<const float> startAngles, int salt, bool isThumb) {
        const Vec3 p0 = digit.points[0], p1 = digit.points[1], p2 = digit.points[2], tip = digit.points[3];
        const float roll = isThumb ? recipe.thumbRoll : 0.0f;
        const int bone0 = digit.joints[0], bone1 = digit.joints[1], bone2 = digit.joints[2];
        // Os aneis: o centro, o eixo, o dorso, a posicao ao longo das medidas
        // (0 base, 1 meio, 2 ponta, 3 perto da ponta), a escala, o no em
        // cima e o skin.
        struct Station {
            Vec3 center;
            Vec3 along;
            Vec3 dorsal;
            float size;
            float scale;
            float knot;
            Influences influences;
        };
        std::vector<Station> stations;
        const Vec3 a0 = p1 - p0, a1 = p2 - p1, a2 = tip - p2;
        const Vec3 bend1 = safeNormalized(a0, a1) + safeNormalized(a1, a0), bend2 = safeNormalized(a1, a2) + safeNormalized(a2, a1);
        if (isThumb) {
            // O polegar sai da janela: o MCP, o meio da falange, o IP...
            stations.push_back({ p1, bend1, digit.dorsal[0] + digit.dorsal[1], 0.0f, 1.0f, recipe.jointBump,
                { { bone0, 0.5f }, { bone1, 0.5f } } });
            stations.push_back({ p1 + a1 * 0.5f, a1, digit.dorsal[1], 0.5f, 1.0f, 0.0f, { { bone1, 1.0f } } });
            stations.push_back({ p2, bend2, digit.dorsal[1] + digit.dorsal[2], 1.0f, 1.0f, 0.7f * recipe.jointBump,
                { { bone1, 0.5f }, { bone2, 0.5f } } });
            stations.push_back({ p2 + a2 * 0.45f, a2, digit.dorsal[2], 2.0f, 1.0f, 0.0f, { { bone2, 1.0f } } });
        } else {
            stations.push_back({ p0 + a0 * 0.55f, a0, digit.dorsal[0], 0.55f, 1.0f, 0.0f, { { bone0, 1.0f } } });
            stations.push_back({ p1, bend1, digit.dorsal[0] + digit.dorsal[1], 1.0f, 1.0f, recipe.jointBump,
                { { bone0, 0.5f }, { bone1, 0.5f } } });
            stations.push_back({ p1 + a1 * 0.5f, a1, digit.dorsal[1], 1.5f, 1.0f, 0.0f, { { bone1, 1.0f } } });
            stations.push_back({ p2, bend2, digit.dorsal[1] + digit.dorsal[2], 2.0f, 1.0f, 0.6f * recipe.jointBump,
                { { bone1, 0.5f }, { bone2, 0.5f } } });
            stations.push_back({ p2 + a2 * 0.45f, a2, digit.dorsal[2], 2.0f + 0.45f / recipe.tipStart, 1.0f, 0.0f,
                { { bone2, 1.0f } } });
        }
        stations.push_back({ p2 + a2 * recipe.tipStart, a2, digit.dorsal[2], 3.0f, 1.0f, 0.0f, { { bone2, 1.0f } } });
        stations.push_back({ p2 + a2 * (recipe.tipStart + 0.6f * (1.0f - recipe.tipStart)), a2, digit.dorsal[2], 3.0f,
            recipe.tipScale, 0.0f, { { bone2, 1.0f } } });
        // Os angulos: do comeco (a volta da base) ate o octogono com a face
        // da unha em cima, alinhado com ela.
        std::size_t shift = 0;
        {
            float best = std::numeric_limits<float>::max();
            for (std::size_t c = 0; c < DigitPoints; ++c) {
                float cost = 0.0f;
                for (std::size_t k = 0; k < DigitPoints; ++k)
                    cost += std::abs(std::remainder(startAngles[k] - uniform[(k + c) % DigitPoints], 2.0f * Pi));
                if (cost < best) {
                    best = cost;
                    shift = c;
                }
            }
        }
        std::vector<std::vector<std::uint32_t>> digitRings { std::move(start) };
        for (std::size_t n = 0; n < stations.size(); ++n) {
            const Station& station = stations[n];
            const DigitFrame ring = digitFrame(station.center, station.along, station.dorsal, roll);
            // O polegar sai da janela com os angulos dela e chega ao
            // octogono no IP.
            const float follow = isThumb ? std::clamp(1.0f - static_cast<float>(n) / 2.0f, 0.0f, 1.0f) : 0.0f;
            std::array<float, DigitPoints> angles {};
            for (std::size_t k = 0; k < DigitPoints; ++k) {
                const float target = uniform[(k + shift) % DigitPoints];
                angles[k] = target + follow * std::remainder(startAngles[k] - target, 2.0f * Pi);
            }
            const float width = sizeAt(size.width, station.size) * station.scale;
            const float thickness = sizeAt(size.thickness, station.size) * station.scale;
            digitRings.push_back(digitRing(ring, angles, width, thickness, station.knot, salt * 10 + static_cast<int>(n),
                station.influences));
        }
        // As faixas; nas duas ultimas, a face de cima (a do octogono, entre
        // 67,5 e 112,5 graus) e a unha.
        for (std::size_t n = 0; n + 1 < digitRings.size(); ++n) {
            const bool nailBand = n + 3 >= digitRings.size();
            const std::vector<std::uint32_t>& from = digitRings[n];
            const std::vector<std::uint32_t>& to = digitRings[n + 1];
            for (std::size_t k = 0; k < DigitPoints; ++k) {
                const std::size_t next = (k + 1) % DigitPoints;
                builder.useMaterial(nailBand && (k + shift) % DigitPoints == 1 ? "unha" : "pele");
                builder.quad(from[k], from[next], to[next], to[k]);
            }
        }
        builder.useMaterial("pele");
        const DigitFrame last = digitFrame(tip, a2, digit.dorsal[2], roll);
        const float pad = 0.25f * size.thickness[3];
        const std::uint32_t apex = vertex(tip - last.dorsal * pad, { { bone2, 1.0f } });
        const std::vector<std::uint32_t>& end = digitRings.back();
        for (std::size_t k = 0; k < end.size(); ++k) builder.triangle(apex, end[k], end[(k + 1) % end.size()]);
    };
    for (std::size_t i = 0; i < FingerCount; ++i) {
        // A volta da base ja esta no octogono do dedo.
        const std::vector<std::uint32_t> start(loops[i].begin(), loops[i].end());
        finishDigit(fingers[i], recipe.fingers[i], start, uniform, static_cast<int>(i) + 1, false);
    }

    // ---- 8. O polegar: da janela ate a ponta ----
    const std::vector<std::uint32_t> window { rings[ra][j0], rings[ra][j1], rings[ra][j2], rings[rb][j2], rings[rc][j2],
        rings[rc][j1], rings[rc][j0], rings[rb][j0] };
    {
        const Vec3 a0 = thumb.points[1] - thumb.points[0], a1 = thumb.points[2] - thumb.points[1];
        const DigitFrame knuckleFrame = digitFrame(thumb.points[1], safeNormalized(a0, a1) + safeNormalized(a1, a0),
            thumb.dorsal[0] + thumb.dorsal[1], recipe.thumbRoll);
        std::array<float, DigitPoints> angles {};
        float turn = 0.0f;
        for (std::size_t k = 0; k < DigitPoints; ++k) angles[k] = knuckleFrame.angleOf(mesh.positions[window[k]]);
        for (std::size_t k = 0; k < DigitPoints; ++k) turn += positiveAngle(angles[(k + 1) % DigitPoints] - angles[k]);
        if (std::abs(turn - 2.0f * Pi) > 0.3f) {
            seams.error = "mao: a janela do polegar nao da a volta no eixo dele (thumbFromRing ou o polegar do esqueleto)";
            return seams;
        }
        finishDigit(thumb, recipe.thumb, window, angles, 9, true);
    }

    seams.ok = true;
    return seams;
}

} // namespace MatterEngine::Modeling
