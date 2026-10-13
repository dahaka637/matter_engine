#include "Engine/Geometry/ConvexHull3D.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <stdexcept>
#include <utility>

namespace MatterEngine {
namespace {

struct Face {
    std::array<std::uint32_t, 3> vertex {};
    Vec3 normal;
    float offset = 0.0f;
    bool alive = true;
    // Os pontos ainda fora desta face (os candidatos a vertice).
    std::vector<std::uint32_t> outside;
};

// Uma face pelos tres vertices, virada para fora (o lado oposto ao do ponto
// de dentro); a ordem dos vertices segue a normal (anti-horario visto de fora).
Face makeFace(std::span<const Vec3> points, std::uint32_t a, std::uint32_t b, std::uint32_t c, Vec3 inside) {
    Face face;
    face.vertex = { a, b, c };
    Vec3 normal = cross(points[b] - points[a], points[c] - points[a]);
    const float length = normal.length();
    normal = length > 0.0f ? normal * (1.0f / length) : Vec3 {};
    if (dot(normal, points[a] - inside) < 0.0f) {
        std::swap(face.vertex[1], face.vertex[2]);
        normal = normal * -1.0f;
    }
    face.normal = normal;
    face.offset = dot(normal, points[a]);
    return face;
}

float distanceTo(const Face& face, Vec3 point) { return dot(face.normal, point) - face.offset; }

float distanceToLine(Vec3 point, Vec3 a, Vec3 b) {
    const Vec3 axis = b - a;
    const float length = axis.length();
    if (length <= 0.0f) return (point - a).length();
    return cross(point - a, axis).length() / length;
}

} // namespace

ConvexHull3D buildConvexHull3D(std::span<const Vec3> points, float tolerance, std::size_t maxVertices) {
    const std::size_t count = points.size();
    if (count < 4) throw std::invalid_argument("casca convexa: menos de 4 pontos");
    if (count >= std::numeric_limits<std::uint32_t>::max()) throw std::invalid_argument("casca convexa: pontos demais");
    Vec3 low = points[0], high = points[0];
    for (const Vec3& p : points) {
        low = { std::min(low.x, p.x), std::min(low.y, p.y), std::min(low.z, p.z) };
        high = { std::max(high.x, p.x), std::max(high.y, p.y), std::max(high.z, p.z) };
    }
    const Vec3 extent = high - low;
    const float scale = std::max({ extent.x, extent.y, extent.z });
    const float epsilon = std::max(1.0e-7f, scale * 1.0e-6f);

    // O tetraedro inicial: as pontas do eixo mais longo, o mais longe da reta
    // delas e o mais longe do plano dos tres.
    const int axis = extent.x >= extent.y && extent.x >= extent.z ? 0 : extent.y >= extent.z ? 1 : 2;
    const auto component = [axis](Vec3 p) { return axis == 0 ? p.x : axis == 1 ? p.y : p.z; };
    std::uint32_t i0 = 0, i1 = 0;
    for (std::uint32_t i = 0; i < count; ++i) {
        if (component(points[i]) < component(points[i0])) i0 = i;
        if (component(points[i]) > component(points[i1])) i1 = i;
    }
    std::uint32_t i2 = i0;
    float far = -1.0f;
    for (std::uint32_t i = 0; i < count; ++i) {
        const float d = distanceToLine(points[i], points[i0], points[i1]);
        if (d > far) {
            far = d;
            i2 = i;
        }
    }
    if (far <= epsilon) throw std::invalid_argument("casca convexa: os pontos estao numa reta");
    const Vec3 baseNormal = cross(points[i1] - points[i0], points[i2] - points[i0]).normalized();
    std::uint32_t i3 = i0;
    far = -1.0f;
    for (std::uint32_t i = 0; i < count; ++i) {
        const float d = std::abs(dot(baseNormal, points[i] - points[i0]));
        if (d > far) {
            far = d;
            i3 = i;
        }
    }
    if (far <= epsilon) throw std::invalid_argument("casca convexa: os pontos estao num plano");
    const Vec3 inside = (points[i0] + points[i1] + points[i2] + points[i3]) * 0.25f;

    std::vector<Face> faces;
    faces.push_back(makeFace(points, i0, i1, i2, inside));
    faces.push_back(makeFace(points, i0, i1, i3, inside));
    faces.push_back(makeFace(points, i0, i2, i3, inside));
    faces.push_back(makeFace(points, i1, i2, i3, inside));
    // Cada ponto de fora vai para a face de que ele fica mais longe: assim a
    // parada pela tolerancia vale para a casca toda (na primeira face de que
    // ele ficasse fora, ele podia ficar mais longe de outra).
    const auto assign = [&](std::uint32_t i, std::size_t first) {
        float best = epsilon;
        std::size_t chosen = faces.size();
        for (std::size_t f = first; f < faces.size(); ++f) {
            if (!faces[f].alive) continue;
            const float d = distanceTo(faces[f], points[i]);
            if (d > best) {
                best = d;
                chosen = f;
            }
        }
        if (chosen < faces.size()) faces[chosen].outside.push_back(i);
    };
    for (std::uint32_t i = 0; i < count; ++i) {
        if (i == i0 || i == i1 || i == i2 || i == i3) continue;
        assign(i, 0);
    }

    std::size_t vertices = 4;
    const float stop = std::max(tolerance, epsilon);
    while (vertices < maxVertices) {
        // O ponto mais longe de fora (de todas as faces).
        float farthest = stop;
        std::uint32_t eye = 0;
        bool found = false;
        for (const Face& face : faces) {
            if (!face.alive) continue;
            for (const std::uint32_t i : face.outside) {
                const float d = distanceTo(face, points[i]);
                if (d > farthest) {
                    farthest = d;
                    eye = i;
                    found = true;
                }
            }
        }
        if (!found) break;
        // As faces que ele ve saem; a borda delas (aresta de uma face vista
        // so) liga ao ponto novo.
        std::vector<std::size_t> visible;
        for (std::size_t f = 0; f < faces.size(); ++f)
            if (faces[f].alive && distanceTo(faces[f], points[eye]) > epsilon) visible.push_back(f);
        std::map<std::pair<std::uint32_t, std::uint32_t>, int> edges;
        std::vector<std::uint32_t> orphans;
        for (const std::size_t f : visible) {
            Face& face = faces[f];
            for (int k = 0; k < 3; ++k) {
                const std::uint32_t a = face.vertex[static_cast<std::size_t>(k)];
                const std::uint32_t b = face.vertex[static_cast<std::size_t>((k + 1) % 3)];
                ++edges[{ std::min(a, b), std::max(a, b) }];
            }
            orphans.insert(orphans.end(), face.outside.begin(), face.outside.end());
            face.outside.clear();
            face.alive = false;
        }
        const std::size_t first = faces.size();
        for (const auto& [edge, uses] : edges)
            if (uses == 1) faces.push_back(makeFace(points, edge.first, edge.second, eye, inside));
        for (const std::uint32_t i : orphans)
            if (i != eye) assign(i, first);
        ++vertices;
    }

    ConvexHull3D hull;
    std::map<std::uint32_t, std::uint16_t> remap;
    for (const Face& face : faces) {
        if (!face.alive) continue;
        std::array<std::uint16_t, 3> triangle {};
        for (std::size_t k = 0; k < 3; ++k) {
            const std::uint32_t v = face.vertex[k];
            auto it = remap.find(v);
            if (it == remap.end()) {
                if (hull.points.size() >= std::numeric_limits<std::uint16_t>::max())
                    throw std::invalid_argument("casca convexa: vertices demais");
                it = remap.emplace(v, static_cast<std::uint16_t>(hull.points.size())).first;
                hull.points.push_back(points[v]);
            }
            triangle[k] = it->second;
        }
        hull.faces.push_back(triangle);
    }
    return hull;
}

float convexHullOutside3D(const ConvexHull3D& hull, Vec3 point) {
    float outside = -std::numeric_limits<float>::max();
    Vec3 centroid {};
    for (const Vec3& p : hull.points) centroid = centroid + p;
    centroid = centroid * (1.0f / static_cast<float>(std::max<std::size_t>(1, hull.points.size())));
    for (const auto& face : hull.faces) {
        const Vec3 a = hull.points[face[0]], b = hull.points[face[1]], c = hull.points[face[2]];
        Vec3 normal = cross(b - a, c - a);
        const float length = normal.length();
        if (length <= 0.0f) continue;
        normal = normal * (1.0f / length);
        if (dot(normal, a - centroid) < 0.0f) normal = normal * -1.0f;
        outside = std::max(outside, dot(normal, point - a));
    }
    return outside;
}

} // namespace MatterEngine
