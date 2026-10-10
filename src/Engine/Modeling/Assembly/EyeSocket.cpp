#include "Engine/Modeling/Assembly/EyeSocket.hpp"

#include "Engine/Modeling/Geometry/Triangulation2D.hpp"
#include "Engine/Modeling/Mesh/MeshBuilder.hpp"
#include "Engine/Modeling/Mesh/TriangleBvh.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <map>
#include <unordered_map>

namespace MatterEngine::Modeling {

namespace {

constexpr std::size_t Points = 12;
// Os cantos na volta: o de fora e o de dentro.
constexpr std::size_t OuterCorner = 0;
constexpr std::size_t InnerCorner = 6;

// O referencial do olho: u para a direita de quem olha de frente (o nariz,
// no olho direito), v para cima, w para fora (o olhar).
struct EyeFrame {
    Vec3 center;
    Vec3 u;
    Vec3 v;
    Vec3 w;

    [[nodiscard]] Vec3 at(Point2 p, float depth) const { return center + u * p.x + v * p.y + w * depth; }
    [[nodiscard]] Point2 project(Vec3 p) const { return { dot(p - center, u), dot(p - center, v) }; }
    [[nodiscard]] float depth(Vec3 p) const { return dot(p - center, w); }
};

float cross2(Point2 a, Point2 b, Point2 c) { return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x); }

bool segmentsCross(Point2 a, Point2 b, Point2 c, Point2 d) {
    const float d1 = cross2(a, b, c), d2 = cross2(a, b, d), d3 = cross2(c, d, a), d4 = cross2(c, d, b);
    return ((d1 > 0.0f) != (d2 > 0.0f)) && ((d3 > 0.0f) != (d4 > 0.0f));
}

bool insideTriangle(Point2 p, Point2 a, Point2 b, Point2 c) {
    const float d1 = cross2(a, b, p), d2 = cross2(b, c, p), d3 = cross2(c, a, p);
    return (d1 >= 0.0f && d2 >= 0.0f && d3 >= 0.0f) || (d1 <= 0.0f && d2 <= 0.0f && d3 <= 0.0f);
}

Point2 operator+(Point2 a, Point2 b) { return { a.x + b.x, a.y + b.y }; }
Point2 operator*(Point2 a, float s) { return { a.x * s, a.y * s }; }

} // namespace

EyeSocketResult buildEyeSocket(Mesh& mesh, const EyeShape& eye, int part) {
    EyeSocketResult result;
    // ---- O referencial: o centro na pele (de frente), o olhar girado ----
    std::vector<std::uint32_t> partTriangles;
    std::vector<Triangle> partList;
    for (std::size_t t = 0; t < mesh.triangles.size(); ++t)
        if (mesh.partOf(t) == part) {
            partTriangles.push_back(static_cast<std::uint32_t>(t));
            partList.push_back(mesh.triangles[t]);
        }
    const TriangleBvh bvh(mesh.positions, partList);
    const auto surface = [&](Vec3 origin, Vec3 direction) -> std::optional<Vec3> {
        const std::optional<TriangleBvh::Hit> hit = bvh.raycast(origin, direction, 1.0f);
        if (!hit) return std::nullopt;
        return origin + direction * hit->distance;
    };
    const std::optional<Vec3> center = surface({ 0.5f, eye.centerY, eye.centerZ }, { -1.0f, 0.0f, 0.0f });
    if (!center) {
        result.error = "olho: o centro nao cai na pele da cabeca";
        return result;
    }
    const float yaw = eye.yaw * DegreesToRadians, pitch = eye.pitch * DegreesToRadians;
    EyeFrame frame;
    frame.center = *center;
    frame.w = { std::cos(yaw) * std::cos(pitch), -std::sin(yaw) * std::cos(pitch), -std::sin(pitch) };
    frame.u = safeNormalized(cross({ 0.0f, 0.0f, 1.0f }, frame.w), { 0.0f, 1.0f, 0.0f });
    frame.v = cross(frame.w, frame.u);

    // ---- A amendoa (anti-horaria vista de frente): o canto de fora, a
    // palpebra de baixo, o canto de dentro e a de cima ----
    const float width = eye.width, height = eye.height, tilt = eye.tilt;
    // (O alto da de cima e o fundo da de baixo um pouco para fora do meio; o
    // canto de fora sobe e o de dentro desce "tilt". Os pontos 2 e 4 da de
    // baixo e 8 e 10 da de cima ficam nos cantos da placa do olho.)
    const std::array<Point2, Points> opening { { { -0.5f * width, tilt * height }, { -0.32f * width, -0.3f * height },
        { -0.1f * width, -0.47f * height }, { 0.08f * width, -0.48f * height }, { 0.25f * width, -0.4f * height },
        { 0.4f * width, -0.26f * height }, { 0.5f * width, -tilt * height }, { 0.4f * width, 0.3f * height },
        { 0.28f * width, 0.44f * height }, { 0.09f * width, 0.52f * height }, { -0.1f * width, 0.5f * height },
        { -0.32f * width, 0.36f * height } } };
    // A placa do olho (como na referencia): um hexagono com o alto e a base
    // retos em volta da abertura - o alto "marginUpper" acima dela, a base
    // "marginLower" abaixo, os cantos "cornerMargin" alem dos cantos. Cada
    // ponto da abertura tem o seu na placa logo acima ou abaixo dele.
    const float top = 0.52f * height + eye.marginUpper, bottom = -0.48f * height - eye.marginLower;
    const float outerU = -0.5f * width - eye.cornerMargin, innerU = 0.5f * width + eye.cornerMargin;
    const float upperOuter = -0.1f * width, upperInner = 0.28f * width, lowerOuter = -0.1f * width, lowerInner = 0.25f * width;
    const auto plate = [&](float u, bool upper) {
        const float cornerV = u < 0.0f ? opening[OuterCorner].y : opening[InnerCorner].y;
        const float flat = upper ? top : bottom;
        const float from = upper ? upperOuter : lowerOuter, to = upper ? upperInner : lowerInner;
        if (u >= from && u <= to) return flat;
        const float t = u < from ? (u - outerU) / (from - outerU) : (innerU - u) / (innerU - to);
        return cornerV + (flat - cornerV) * std::clamp(t, 0.0f, 1.0f);
    };
    std::array<Point2, Points> rim {};
    for (std::size_t k = 0; k < Points; ++k) {
        if (k == OuterCorner) rim[k] = { outerU, opening[k].y };
        else if (k == InnerCorner) rim[k] = { innerU, opening[k].y };
        else rim[k] = { opening[k].x, plate(opening[k].x, k > InnerCorner) };
    }
    // Para fora (da amendoa) em cada ponto e a crista de cada palpebra.
    std::array<Point2, Points> outward {};
    std::array<float, Points> crest {};
    for (std::size_t k = 0; k < Points; ++k) {
        const Point2 previous = opening[(k + Points - 1) % Points], next = opening[(k + 1) % Points];
        const float dx = next.x - previous.x, dy = next.y - previous.y, length = std::max(1e-9f, std::hypot(dx, dy));
        outward[k] = { dy / length, -dx / length };
        const bool corner = k == OuterCorner || k == InnerCorner;
        crest[k] = corner ? 0.8f * eye.lidLower : k > InnerCorner ? eye.lidUpper : eye.lidLower;
    }
    // A placa na pele (ao longo do olhar).
    std::array<Vec3, Points> rimPosition {};
    for (std::size_t k = 0; k < Points; ++k) {
        const std::optional<Vec3> on = surface(frame.at(rim[k], 0.05f), -frame.w);
        if (!on) {
            result.error = "olho: a placa do olho sai da pele da cabeca";
            return result;
        }
        rimPosition[k] = *on;
    }

    // ---- Os triangulos por baixo da borda: saem e a orbita e refeita ----
    std::vector<bool> host(mesh.triangles.size(), false);
    for (const std::uint32_t t : partTriangles) {
        const Triangle& tri = mesh.triangles[t];
        if (dot(mesh.normal(t), frame.w) < 0.15f) continue;
        if ((mesh.center(t) - frame.center).length() > 0.05f) continue;
        const std::array<Point2, 3> p { frame.project(mesh.positions[tri[0]]), frame.project(mesh.positions[tri[1]]),
            frame.project(mesh.positions[tri[2]]) };
        bool overlap = false;
        for (std::size_t k = 0; k < Points && !overlap; ++k) {
            if (insideTriangle(rim[k], p[0], p[1], p[2])) overlap = true;
            for (int e = 0; e < 3 && !overlap; ++e)
                if (segmentsCross(rim[k], rim[(k + 1) % Points], p[static_cast<std::size_t>(e)], p[static_cast<std::size_t>((e + 1) % 3)]))
                    overlap = true;
        }
        if (!overlap) {
            // Um vertice do triangulo dentro da borda (o triangulo todo dentro).
            std::vector<Point2> polygon(rim.begin(), rim.end());
            overlap = pointInPolygon2D(p[0], polygon);
        }
        host[t] = overlap;
    }
    // A volta dos triangulos que saem (a borda de fora da orbita refeita).
    std::map<std::pair<std::uint32_t, std::uint32_t>, int> directed;
    for (std::size_t t = 0; t < host.size(); ++t)
        if (host[t])
            for (int i = 0; i < 3; ++i)
                ++directed[{ mesh.triangles[t][static_cast<std::size_t>(i)], mesh.triangles[t][static_cast<std::size_t>((i + 1) % 3)] }];
    std::unordered_map<std::uint32_t, std::uint32_t> next;
    for (const auto& [edge, count] : directed)
        if (directed.count({ edge.second, edge.first }) == 0) {
            if (next.count(edge.first) != 0) {
                result.error = "olho: a orbita refeita nao e um disco (a volta se toca)";
                return result;
            }
            next[edge.first] = edge.second;
        }
    if (next.empty()) {
        result.error = "olho: nenhum triangulo da orbita debaixo da abertura";
        return result;
    }
    std::vector<std::uint32_t> outer;
    {
        const std::uint32_t start = next.begin()->first;
        std::uint32_t at = start;
        do {
            outer.push_back(at);
            const auto found = next.find(at);
            if (found == next.end()) {
                result.error = "olho: a volta da orbita nao fecha";
                return result;
            }
            at = found->second;
        } while (at != start && outer.size() <= next.size());
        if (outer.size() != next.size()) {
            result.error = "olho: a orbita refeita tem mais de uma volta";
            return result;
        }
    }

    MeshBuilder builder(mesh);
    const int skinMaterial = mesh.findMaterial("pele");
    builder.material("olho", rgb(226, 216, 200));
    builder.part(mesh.parts[static_cast<std::size_t>(part)].name);
    if (skinMaterial >= 0) builder.useMaterial("pele");
    else builder.material("pele", rgb(212, 152, 106));
    // A orbita: a volta de fora e a borda (o furo), Delaunay no plano do olho.
    std::vector<std::uint32_t> rimVertex(Points);
    for (std::size_t k = 0; k < Points; ++k) rimVertex[k] = builder.vertex(rimPosition[k]);
    Triangulation2DInput input;
    std::vector<std::uint32_t> vertexOf;
    std::vector<int> outerLoop, holeLoop;
    for (const std::uint32_t v : outer) {
        outerLoop.push_back(static_cast<int>(input.points.size()));
        input.points.push_back(frame.project(mesh.positions[v]));
        vertexOf.push_back(v);
    }
    for (std::size_t k = 0; k < Points; ++k) {
        holeLoop.push_back(static_cast<int>(input.points.size()));
        input.points.push_back(rim[k]);
        vertexOf.push_back(rimVertex[k]);
    }
    input.loops = { outerLoop, holeLoop };
    const Triangulation2DResult socket = triangulateDomain(input);
    if (!socket.ok) {
        result.error = "olho: a orbita nao triangulou (" + socket.error + ")";
        return result;
    }
    for (std::size_t t = 0; t < host.size(); ++t)
        if (host[t]) mesh.triangles[t] = { 0, 0, 0 };
    for (const auto& triangle : socket.triangles)
        builder.triangle(vertexOf[static_cast<std::size_t>(triangle[0])], vertexOf[static_cast<std::size_t>(triangle[1])],
            vertexOf[static_cast<std::size_t>(triangle[2])]);

    // ---- As palpebras e o globo: aneis da borda para dentro ----
    std::array<std::uint32_t, Points> crestRing {}, edgeRing {}, contactRing {}, outerDome {}, innerDome {};
    std::array<float, Points> edgeDepth {};
    for (std::size_t k = 0; k < Points; ++k) {
        const float crestDepth = frame.depth(rimPosition[k]) + crest[k];
        const bool upper = k > InnerCorner;
        const float band = k == OuterCorner || k == InnerCorner ? 0.5f * (eye.lidBandUpper + eye.lidBandLower)
            : upper ? eye.lidBandUpper : eye.lidBandLower;
        crestRing[k] = builder.vertex(frame.at(opening[k] + outward[k] * band, crestDepth));
        edgeDepth[k] = crestDepth - 0.0005f;
        edgeRing[k] = builder.vertex(frame.at(opening[k], edgeDepth[k]));
    }
    // O globo: a frente de um elipsoide (largo para os cantos; o raio na
    // profundidade da a curva: o meio sobe uns "bulge" acima da borda), o
    // mais a frente possivel com a borda das palpebras "recess" a frente
    // dele; o contato, os aneis e o meio sobre ele.
    const float radiusU = 0.5f * width * 1.25f, radiusV = 0.5f * height * 1.6f, radiusW = 4.0f * eye.bulge;
    const auto dome = [&](Point2 at) {
        const float a = at.x / radiusU, b = at.y / radiusV;
        return radiusW * std::sqrt(std::max(0.0f, 1.0f - a * a - b * b));
    };
    float back = 1e9f;
    for (std::size_t k = 0; k < Points; ++k) back = std::min(back, edgeDepth[k] - eye.recess - dome(opening[k] * 0.94f));
    const auto onBall = [&](Point2 at) { return frame.at(at, back + dome(at)); };
    for (std::size_t k = 0; k < Points; ++k) {
        contactRing[k] = builder.vertex(onBall(opening[k] * 0.94f));
        outerDome[k] = builder.vertex(onBall(opening[k] * 0.66f));
        innerDome[k] = builder.vertex(onBall(opening[k] * 0.33f));
    }
    const Vec3 apexPosition = onBall({ 0.0f, 0.0f });
    const std::uint32_t apex = builder.vertex(apexPosition);
    for (std::size_t k = 0; k < Points; ++k) {
        const std::size_t n = (k + 1) % Points;
        builder.useMaterial("pele");
        builder.quad(rimVertex[k], rimVertex[n], crestRing[n], crestRing[k]);
        builder.quad(crestRing[k], crestRing[n], edgeRing[n], edgeRing[k]);
        builder.quad(edgeRing[k], edgeRing[n], contactRing[n], contactRing[k]);
        builder.useMaterial("olho");
        builder.quad(contactRing[k], contactRing[n], outerDome[n], outerDome[k]);
        builder.quad(outerDome[k], outerDome[n], innerDome[n], innerDome[k]);
        builder.triangle(innerDome[k], innerDome[n], apex);
    }
    builder.useMaterial("pele");
    // Os triangulos que sairam (marcados) somem.
    std::size_t kept = 0;
    for (std::size_t t = 0; t < mesh.triangles.size(); ++t) {
        const Triangle& tri = mesh.triangles[t];
        if (tri[0] == tri[1] && tri[1] == tri[2]) continue;
        mesh.triangles[kept] = tri;
        mesh.trianglePart[kept] = mesh.trianglePart[t];
        mesh.triangleMaterial[kept] = mesh.triangleMaterial[t];
        ++kept;
    }
    mesh.triangles.resize(kept);
    mesh.trianglePart.resize(kept);
    mesh.triangleMaterial.resize(kept);
    result.ok = true;
    return result;
}

} // namespace MatterEngine::Modeling
