#include "Engine/Modeling/Parts/HeadGenerator.hpp"

#include "Engine/Modeling/Geometry/Triangulation2D.hpp"
#include "Engine/Modeling/Mesh/MeshBuilder.hpp"
#include "Engine/Modeling/Mesh/MeshStitch.hpp"
#include "Engine/Modeling/Parts/FacetNoise.hpp"
#include "Engine/Modeling/Parts/HeadBrow.hpp"
#include "Engine/Modeling/Parts/HeadEar.hpp"
#include "Engine/Modeling/Parts/HeadEye.hpp"
#include "Engine/Modeling/Parts/HeadFace.hpp"
#include "Engine/Modeling/Parts/SuperSection.hpp"

#include <algorithm>
#include <cmath>

namespace MatterEngine::Modeling {

namespace {

float smooth(float edge0, float edge1, float x) {
    const float t = std::clamp((x - edge0) / (edge1 - edge0), 0.0f, 1.0f);
    return t * t * (3.0f - 2.0f * t);
}

float distanceToSegment(Point2 p, Point2 a, Point2 b) {
    const float dx = b.x - a.x, dy = b.y - a.y;
    const float length = dx * dx + dy * dy;
    const float t = length > 1e-12f ? std::clamp(((p.x - a.x) * dx + (p.y - a.y) * dy) / length, 0.0f, 1.0f) : 0.0f;
    return std::hypot(p.x - (a.x + t * dx), p.y - (a.y + t * dy));
}

} // namespace

HeadSeams buildHeadHalf(Mesh& mesh, const HeadRecipe& recipe, const HeadPlacement& placement) {
    HeadSeams seams;
    seams.part = placement.part;
    if (placement.neckRing.size() < 3) {
        seams.error = "cabeca: falta a volta de cima do pescoco";
        return seams;
    }
    const float ground = placement.ground;
    const SectionCurves curves { recipe.centerX, recipe.front, recipe.back, recipe.side, recipe.exponent, &recipe.sideExponent,
        &recipe.frontExponent, &recipe.frontSideExponent };
    HeadShape shape { recipe, curves, ground };
    shape.neckFront = mesh.positions[placement.neckRing.front()].z - ground;
    shape.neckBack = mesh.positions[placement.neckRing.back()].z - ground;

    // ---- 1. O dominio: (angulo em volta x raio medio, altura) ----
    // A pele e desenrolada em volta: um passo no dominio e mais ou menos o
    // mesmo passo na pele. Os pontos: o anel do pescoco (a borda de baixo),
    // o meio de tras, a fileira do alto (o cranio fecha em leque acima
    // dela), os tracos do rosto (o meio da frente e a borda da esquerda) e o
    // preenchimento.
    struct Point {
        float angle = 0.0f;
        float height = 0.0f;
        Vec3 position;
        std::uint32_t vertex = UINT32_MAX;
    };
    std::vector<Point> points;
    // Na vertical, o comprimento da pele ao longo do meridiano (no alto, onde
    // a pele deita, um centimetro de altura e varios de pele): a tabela da
    // altura para ele e a volta.
    std::vector<float> tableHeight, tableArc;
    {
        const float step = 0.0005f;
        float arc = 0.0f;
        for (float height = 1.55f; height <= recipe.crownHeight + 1e-5f; height += step) {
            const float d = 0.5f * step;
            const auto slope = [&](const ProfileCurve& curve) { return std::abs(curve(height + d) - curve(height - d)) / step; };
            const float lean = 0.25f * (slope(recipe.front) + 2.0f * slope(recipe.side) + slope(recipe.back));
            if (!tableHeight.empty()) arc += step * std::sqrt(1.0f + lean * lean);
            tableHeight.push_back(height);
            tableArc.push_back(arc);
        }
    }
    const auto lookup = [](const std::vector<float>& from, const std::vector<float>& to, float value) {
        const auto it = std::lower_bound(from.begin(), from.end(), value);
        if (it == from.begin()) return to.front() + (value - from.front());
        if (it == from.end()) return to.back() + (value - from.back());
        const std::size_t i = static_cast<std::size_t>(it - from.begin());
        const float t = (value - from[i - 1]) / std::max(1e-9f, from[i] - from[i - 1]);
        return to[i - 1] + (to[i] - to[i - 1]) * t;
    };
    const auto vertical = [&](float height) { return lookup(tableHeight, tableArc, height); };
    const auto heightAt = [&](float arc) { return lookup(tableArc, tableHeight, arc); };
    const auto domain = [&](float angle, float height) { return Point2 { angle * shape.meanRadius(height), vertical(height) }; };
    const auto addPoint = [&](float angle, float height, Vec3 position, std::uint32_t vertex = UINT32_MAX) {
        points.push_back({ angle, height, position, vertex });
        return static_cast<int>(points.size() - 1);
    };
    // O anel do pescoco (da frente para tras).
    std::vector<int> neck;
    for (std::size_t k = 0; k < placement.neckRing.size(); ++k) {
        const Vec3 p = mesh.positions[placement.neckRing[k]];
        const float height = p.z - ground;
        float angle = std::atan2(-p.y, p.x - shape.at(height).centerX);
        if (k == 0) angle = 0.0f;
        if (k + 1 == placement.neckRing.size()) angle = Pi;
        neck.push_back(addPoint(std::clamp(angle, 0.0f, Pi), height, p, placement.neckRing[k]));
    }
    // A fileira do alto e o meio de tras.
    const int capCount = std::max(4, recipe.capColumns);
    std::vector<int> cap;
    for (int k = 0; k < capCount; ++k) {
        const float angle = Pi * static_cast<float>(k) / static_cast<float>(capCount - 1);
        cap.push_back(addPoint(angle, recipe.capHeight, shape.surface(angle, recipe.capHeight)));
    }
    std::vector<int> back;
    {
        const float from = vertical(points[static_cast<std::size_t>(neck.back())].height), to = vertical(recipe.capHeight);
        const int steps = std::max(1, static_cast<int>(std::round((to - from) / recipe.skullSpacing)));
        for (int k = 1; k < steps; ++k) {
            const float height = heightAt(from + (to - from) * static_cast<float>(k) / static_cast<float>(steps));
            back.push_back(addPoint(Pi, height, shape.surface(Pi, height)));
        }
    }
    // Os tracos do rosto.
    const FaceLayout layout = faceLayout(shape);
    const int faceBase = static_cast<int>(points.size());
    for (const FaceVertex& vertex : layout.vertices) addPoint(vertex.angle, vertex.height, vertex.position);
    const auto face = [&](int index) { return faceBase + index; };

    // A borda (anti-horaria no dominio): o pescoco, o meio de tras subindo,
    // o alto de tras para a frente e o meio da frente descendo. Os furos: o
    // olho, a pegada da sobrancelha e a raiz da orelha.
    std::vector<int> outline(neck.begin(), neck.end());
    outline.insert(outline.end(), back.begin(), back.end());
    outline.insert(outline.end(), cap.rbegin(), cap.rend());
    for (auto it = layout.middle.rbegin(); it != layout.middle.rend(); ++it) outline.push_back(face(*it));
    std::vector<int> eyeHole, browHole, earHole;
    for (const int v : layout.eye) eyeHole.push_back(face(v));
    for (const int v : layout.browLoop()) browHole.push_back(face(v));
    for (const int v : layout.earRoot) earHole.push_back(face(v));
    std::vector<std::array<int, 2>> segments;
    for (const auto& [a, b] : layout.segments) segments.push_back({ face(a), face(b) });

    // ---- 2. O preenchimento: denso nos tracos, medio no rosto, esparso no
    // cranio ----
    const auto spacingAt = [&](float angle, float height) {
        const float side = smooth(50.0f * DegreesToRadians, 100.0f * DegreesToRadians, angle);
        const float top = smooth(1.83f, 1.87f, height);
        const float spacing = recipe.faceSpacing + (recipe.skullSpacing - recipe.faceSpacing) * std::max(side, top);
        const SuperSection section = shape.at(height);
        const float lateral = section.radius(angle) * std::sin(angle);
        const float x = section.centerX + section.radius(angle) * std::cos(angle);
        // Em volta da raiz da orelha o passo encolhe (a raiz e miuda).
        const float ear = 1.0f - smooth(0.02f, 0.04f, std::hypot(x - recipe.earX, height - recipe.earHeight));
        const float detail = std::max(faceDetail(recipe, lateral, height) * (1.0f - side), 0.7f * ear);
        return spacing * (1.0f - (1.0f - recipe.detailSpacing) * detail);
    };
    {
        std::vector<Point2> outlinePolygon, eyePolygon, browPolygon, earPolygon;
        const auto at = [&](int v) { return domain(points[static_cast<std::size_t>(v)].angle, points[static_cast<std::size_t>(v)].height); };
        for (const int v : outline) outlinePolygon.push_back(at(v));
        for (const int v : eyeHole) eyePolygon.push_back(at(v));
        for (const int v : browHole) browPolygon.push_back(at(v));
        for (const int v : earHole) earPolygon.push_back(at(v));
        std::vector<std::pair<Point2, Point2>> lines;
        for (const auto& [a, b] : segments) lines.push_back({ at(a), at(b) });
        for (const std::vector<int>* loop : { &outline, &eyeHole, &browHole, &earHole })
            for (std::size_t k = 0; k < loop->size(); ++k) lines.push_back({ at((*loop)[k]), at((*loop)[(k + 1) % loop->size()]) });
        // Candidatos numa grade fina (hexagonal, tremida), na ordem do ruido.
        struct Candidate {
            float angle;
            float height;
            float order;
        };
        std::vector<Candidate> candidates;
        const float step = 0.4f * recipe.detailSpacing * recipe.faceSpacing;
        int row = 0;
        const float top = vertical(recipe.capHeight);
        for (float arc = vertical(1.655f); arc < top; arc += 0.87f * step, ++row) {
            int column = 0;
            const float rowHeight = heightAt(arc);
            const float radius = shape.meanRadius(rowHeight);
            for (float s = static_cast<float>(row % 2) * 0.5f * step; s < Pi * radius; s += step, ++column) {
                const float jitterS = 0.35f * step * facetNoise(recipe.seed, row, column, 3);
                const float jitterV = 0.35f * step * facetNoise(recipe.seed, row, column, 4);
                const float height = heightAt(arc + jitterV);
                const float angle = (s + jitterS) / shape.meanRadius(height);
                if (angle <= 0.0f || angle >= Pi) continue;
                candidates.push_back({ angle, height, facetNoise(recipe.seed, row, column, 5) });
            }
        }
        std::sort(candidates.begin(), candidates.end(), [](const Candidate& a, const Candidate& b) { return a.order < b.order; });
        std::vector<Point2> accepted;
        std::vector<float> keep;
        for (std::size_t k = 0; k < points.size(); ++k) {
            accepted.push_back(domain(points[k].angle, points[k].height));
            keep.push_back(0.65f * spacingAt(points[k].angle, points[k].height));
        }
        for (const Candidate& candidate : candidates) {
            const Point2 p = domain(candidate.angle, candidate.height);
            if (!pointInPolygon2D(p, outlinePolygon) || pointInPolygon2D(p, eyePolygon) || pointInPolygon2D(p, browPolygon)
                || pointInPolygon2D(p, earPolygon))
                continue;
            const float spacing = spacingAt(candidate.angle, candidate.height);
            bool free = true;
            for (const auto& [a, b] : lines)
                if (distanceToSegment(p, a, b) < 0.42f * spacing) {
                    free = false;
                    break;
                }
            for (std::size_t k = 0; free && k < accepted.size(); ++k)
                if (std::hypot(p.x - accepted[k].x, p.y - accepted[k].y) < std::max(0.85f * spacing, keep[k])) free = false;
            if (!free) continue;
            accepted.push_back(p);
            keep.push_back(0.85f * spacing);
            Vec3 position = shape.surface(candidate.angle, candidate.height);
            const SuperSection section = shape.at(candidate.height);
            const Vec3 out = safeNormalized({ position.x - section.centerX, position.y, 0.0f }, { 1.0f, 0.0f, 0.0f });
            position = position + out * (recipe.jitter * facetNoise(recipe.seed, static_cast<int>(points.size()), 0, 6));
            points.push_back({ candidate.angle, candidate.height, position, UINT32_MAX });
        }
    }

    // ---- 3. A triangulacao com os tracos e os furos ----
    Triangulation2DInput input;
    for (const Point& point : points) input.points.push_back(domain(point.angle, point.height));
    input.loops = { outline, eyeHole, browHole, earHole };
    input.segments = segments;
    const Triangulation2DResult triangulation = triangulateDomain(input);
    if (!triangulation.ok) {
        seams.error = "cabeca: a triangulacao do rosto falhou (" + triangulation.error + ")";
        return seams;
    }
    MeshBuilder builder(mesh);
    builder.material("pele", recipe.skinColor);
    builder.material("narina", recipe.nostrilColor);
    builder.part(placement.part, placement.region).connectTo(placement.neckPart);
    const std::size_t firstVertex = mesh.positions.size();
    for (Point& point : points)
        if (point.vertex == UINT32_MAX) point.vertex = builder.vertex(point.position);
    // As regioes de outro material (as narinas), no dominio.
    std::vector<std::pair<std::vector<Point2>, std::string>> regions;
    for (const FaceRegion& region : layout.regions) {
        std::vector<Point2> polygon;
        for (const int v : region.loop)
            polygon.push_back(domain(points[static_cast<std::size_t>(face(v))].angle, points[static_cast<std::size_t>(face(v))].height));
        regions.push_back({ std::move(polygon), region.material });
    }
    // Anti-horario no dominio e horario visto de fora (o angulo cresce para
    // tras): a ordem trocada.
    for (const auto& triangle : triangulation.triangles) {
        Point2 center {};
        for (const int v : triangle) {
            const Point2 p = input.points[static_cast<std::size_t>(v)];
            center = { center.x + p.x / 3.0f, center.y + p.y / 3.0f };
        }
        std::string_view material = "pele";
        for (const auto& [polygon, name] : regions)
            if (pointInPolygon2D(center, polygon)) material = name;
        builder.useMaterial(material);
        builder.triangle(points[static_cast<std::size_t>(triangle[0])].vertex, points[static_cast<std::size_t>(triangle[2])].vertex,
            points[static_cast<std::size_t>(triangle[1])].vertex);
    }
    builder.useMaterial("pele");

    // ---- 4. O alto: da fileira de cima, aneis menores (no meio do caminho
    // de pele que falta) e o ponto do alto ----
    {
        std::vector<std::uint32_t> row;
        for (const int v : cap) row.push_back(points[static_cast<std::size_t>(v)].vertex);
        const float from = vertical(recipe.capHeight), to = vertical(recipe.crownHeight);
        const int rings = std::max(1, static_cast<int>(std::round((to - from) / recipe.skullSpacing)) - 1);
        for (int ring = 1; ring <= rings; ++ring) {
            const float height = heightAt(from + (to - from) * static_cast<float>(ring) / static_cast<float>(rings + 1));
            const int count = std::max(3, static_cast<int>(std::round(Pi * shape.meanRadius(height) / recipe.skullSpacing)) + 1);
            std::vector<std::uint32_t> next;
            for (int k = 0; k < count; ++k) {
                const float angle = Pi * static_cast<float>(k) / static_cast<float>(count - 1);
                next.push_back(builder.vertex(shape.surface(angle, height)));
            }
            const std::vector<std::uint32_t> rowReversed(row.rbegin(), row.rend());
            const std::vector<std::uint32_t> nextReversed(next.rbegin(), next.rend());
            stitchStrips(builder, rowReversed, nextReversed);
            row = std::move(next);
        }
        const std::uint32_t crown = builder.vertex({ shape.at(recipe.crownHeight).centerX, 0.0f, ground + recipe.crownHeight });
        for (std::size_t k = 0; k + 1 < row.size(); ++k) builder.triangle(row[k + 1], row[k], crown);
    }

    // ---- 5. O olho, a sobrancelha e a orelha ----
    {
        EyePlacement eye;
        for (std::size_t k = 0; k < 8; ++k) eye.opening[k] = points[static_cast<std::size_t>(eyeHole[k])].vertex;
        const Vec3 outer = mesh.positions[eye.opening[0]], inner = mesh.positions[eye.opening[4]];
        eye.center = layout.eyeCenter;
        // O olhar: a normal do plano da abertura (anti-horaria vista de fora),
        // virada para o lado como o rosto ali.
        Vec3 facing {};
        for (std::size_t k = 0; k < 8; ++k)
            facing += cross(mesh.positions[eye.opening[k]] - eye.center, mesh.positions[eye.opening[(k + 1) % 8]] - eye.center);
        eye.forward = safeNormalized(facing, { 1.0f, 0.0f, 0.0f });
        eye.across = outer - inner;
        eye.headJoint = placement.headJoint;
        buildEye(builder, recipe, eye);
        builder.part(placement.part, placement.region);

        BrowPlacement brow;
        for (const int v : browHole) {
            const Point& point = points[static_cast<std::size_t>(v)];
            brow.footprint.push_back(point.vertex);
            brow.normals.push_back(shape.normal(point.angle, point.height));
        }
        brow.stations = layout.browBottom.size();
        brow.headJoint = placement.headJoint;
        buildBrow(builder, recipe, brow);
        builder.part(placement.part, placement.region);

        // A orelha: o plano dela e o da pele no centro da concha.
        EarPlacement ear;
        for (const int v : earHole) ear.root.push_back(points[static_cast<std::size_t>(v)].vertex);
        const float earAngle = shape.angleForX(recipe.earHeight, recipe.earX);
        ear.center = shape.surface(earAngle, recipe.earHeight);
        ear.out = shape.normal(earAngle, recipe.earHeight);
        ear.up = safeNormalized(Vec3 { 0.0f, 0.0f, 1.0f } - ear.out * ear.out.z, { 0.0f, 0.0f, 1.0f });
        ear.forward = cross(ear.up, ear.out);
        ear.headJoint = placement.headJoint;
        buildEar(builder, recipe, ear);
        builder.part(placement.part, placement.region);
    }

    // ---- 6. Pesos: a cabeca; perto do pescoco, passando dele ----
    if (mesh.skin.size() < mesh.positions.size()) mesh.skin.resize(mesh.positions.size());
    const float neckFront = shape.neckFront, neckBack = shape.neckBack;
    for (std::size_t v = firstVertex; v < mesh.positions.size(); ++v) {
        const Vec3 p = mesh.positions[v];
        const float angle = std::abs(std::atan2(p.y, p.x - shape.at(p.z - ground).centerX));
        const float base = neckFront + (neckBack - neckFront) * (0.5f - 0.5f * std::cos(angle));
        const float head = placement.headJoint >= 0 ? 0.45f + 0.55f * smooth(0.0f, 0.04f, p.z - ground - base) : 0.0f;
        SkinWeights& weights = mesh.skin[v];
        weights = {};
        std::size_t slot = 0;
        const auto add = [&](int joint, float weight) {
            if (joint < 0 || weight <= 1e-4f || slot >= 4) return;
            weights.joints[slot] = static_cast<std::uint16_t>(joint);
            weights.weights[slot] = weight;
            ++slot;
        };
        add(placement.headJoint, head);
        add(placement.neckJoint, 1.0f - head);
        float total = 0.0f;
        for (const float w : weights.weights) total += w;
        if (total > 0.0f)
            for (float& w : weights.weights) w /= total;
    }
    seams.ok = true;
    return seams;
}

} // namespace MatterEngine::Modeling
