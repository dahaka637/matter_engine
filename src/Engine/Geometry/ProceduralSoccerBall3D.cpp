#include "Engine/Geometry/ProceduralSoccerBall3D.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <map>
#include <stdexcept>
#include <utility>

namespace MatterEngine {
namespace {

constexpr float Phi = 1.61803398875f;
using Triangle = std::array<std::uint32_t, 3>;

std::vector<Vec3> icosahedronVertices() {
    return {
        { -1.0f, Phi, 0.0f }, { 1.0f, Phi, 0.0f }, { -1.0f, -Phi, 0.0f }, { 1.0f, -Phi, 0.0f },
        { 0.0f, -1.0f, Phi }, { 0.0f, 1.0f, Phi }, { 0.0f, -1.0f, -Phi }, { 0.0f, 1.0f, -Phi },
        { Phi, 0.0f, -1.0f }, { Phi, 0.0f, 1.0f }, { -Phi, 0.0f, -1.0f }, { -Phi, 0.0f, 1.0f },
    };
}

// Faces com a ordem anti-horaria vista de fora (conferida ao construir).
std::vector<Triangle> icosahedronFaces(const std::vector<Vec3>& vertices) {
    std::vector<Triangle> faces {
        { 0, 11, 5 }, { 0, 5, 1 }, { 0, 1, 7 }, { 0, 7, 10 }, { 0, 10, 11 },
        { 1, 5, 9 }, { 5, 11, 4 }, { 11, 10, 2 }, { 10, 7, 6 }, { 7, 1, 8 },
        { 3, 9, 4 }, { 3, 4, 2 }, { 3, 2, 6 }, { 3, 6, 8 }, { 3, 8, 9 },
        { 4, 9, 5 }, { 2, 4, 11 }, { 6, 2, 10 }, { 8, 6, 7 }, { 9, 8, 1 },
    };
    for (Triangle& face : faces) {
        const Vec3 a = vertices[face[0]], b = vertices[face[1]], c = vertices[face[2]];
        if (dot(cross(b - a, c - a), a + b + c) < 0.0f) std::swap(face[1], face[2]);
    }
    return faces;
}

struct Icosphere {
    std::vector<Vec3> directions;
    std::vector<Triangle> triangles;
};

Icosphere buildIcosphere(std::uint32_t subdivisions) {
    Icosphere sphere;
    for (const Vec3& vertex : icosahedronVertices()) sphere.directions.push_back(vertex.normalized());
    sphere.triangles = icosahedronFaces(icosahedronVertices());
    for (std::uint32_t level = 0; level < subdivisions; ++level) {
        std::map<std::pair<std::uint32_t, std::uint32_t>, std::uint32_t> midpoints;
        const auto midpoint = [&](std::uint32_t a, std::uint32_t b) {
            const auto key = std::minmax(a, b);
            const auto found = midpoints.find(key);
            if (found != midpoints.end()) return found->second;
            const auto index = static_cast<std::uint32_t>(sphere.directions.size());
            sphere.directions.push_back((sphere.directions[a] + sphere.directions[b]).normalized());
            midpoints.emplace(key, index);
            return index;
        };
        std::vector<Triangle> refined;
        refined.reserve(sphere.triangles.size() * 4);
        for (const Triangle& t : sphere.triangles) {
            const std::uint32_t ab = midpoint(t[0], t[1]);
            const std::uint32_t bc = midpoint(t[1], t[2]);
            const std::uint32_t ca = midpoint(t[2], t[0]);
            refined.push_back({ t[0], ab, ca });
            refined.push_back({ t[1], bc, ab });
            refined.push_back({ t[2], ca, bc });
            refined.push_back({ ab, bc, ca });
        }
        sphere.triangles = std::move(refined);
    }
    return sphere;
}

// Um gomo: poligono plano do icosaedro truncado, em ordem anti-horaria.
struct Panel {
    std::vector<Vec3> corners;
    Vec3 center;
    bool black = false;
};

std::vector<Panel> truncatedIcosahedronPanels() {
    const std::vector<Vec3> vertices = icosahedronVertices();
    const std::vector<Triangle> faces = icosahedronFaces(vertices);
    std::vector<Panel> panels;
    // Pentagonos: um por vertice do icosaedro, cortado a um terco das arestas.
    for (std::size_t a = 0; a < vertices.size(); ++a) {
        const Vec3 axis = vertices[a].normalized();
        std::vector<Vec3> neighbours;
        for (std::size_t b = 0; b < vertices.size(); ++b) {
            if (b != a && (vertices[b] - vertices[a]).length() < 2.01f) neighbours.push_back(vertices[b]);
        }
        const Vec3 tangent = (neighbours.front() - axis * dot(neighbours.front(), axis)).normalized();
        const Vec3 bitangent = cross(axis, tangent);
        std::sort(neighbours.begin(), neighbours.end(), [&](Vec3 p, Vec3 q) {
            return std::atan2(dot(p, bitangent), dot(p, tangent))
                < std::atan2(dot(q, bitangent), dot(q, tangent));
        });
        Panel panel;
        panel.black = true;
        for (const Vec3& neighbour : neighbours) {
            panel.corners.push_back(vertices[a] + (neighbour - vertices[a]) * (1.0f / 3.0f));
        }
        panels.push_back(std::move(panel));
    }
    // Hexagonos: um por face, os dois tercos de cada aresta.
    for (const Triangle& face : faces) {
        Panel panel;
        for (std::size_t k = 0; k < 3; ++k) {
            const Vec3 from = vertices[face[k]];
            const Vec3 to = vertices[face[(k + 1) % 3]];
            panel.corners.push_back(from + (to - from) * (1.0f / 3.0f));
            panel.corners.push_back(from + (to - from) * (2.0f / 3.0f));
        }
        panels.push_back(std::move(panel));
    }
    for (Panel& panel : panels) {
        Vec3 sum {};
        for (const Vec3& corner : panel.corners) sum += corner;
        panel.center = sum * (1.0f / static_cast<float>(panel.corners.size()));
        const Vec3 normal = cross(panel.corners[1] - panel.corners[0], panel.corners[2] - panel.corners[0]);
        if (dot(normal, panel.center) < 0.0f) std::reverse(panel.corners.begin(), panel.corners.end());
    }
    return panels;
}

float smoothstep(float edge0, float edge1, float x) {
    const float t = std::clamp((x - edge0) / (edge1 - edge0), 0.0f, 1.0f);
    return t * t * (3.0f - 2.0f * t);
}

// Perfil radial do gomo pela distancia e (m) ate a costura: o meio estufa
// um pouco e a borda rola para dentro do sulco. Devolve o deslocamento e a
// derivada em e.
struct Profile {
    float offset = 0.0f;
    float slope = 0.0f;
};

Profile panelProfile(const SoccerBallStyle3D& style, float e) {
    e = std::max(0.0f, e);
    Profile profile;
    // Sulco: (1 - e/w)^p, quina viva no fundo (os dois gomos se encontram em
    // V) e tangente horizontal em w.
    constexpr float GrooveExponent = 2.5f;
    const float w = std::max(1e-5f, style.seamHalfWidthMeters);
    if (e < w) {
        const float x = 1.0f - e / w;
        profile.offset -= style.seamDepthMeters * std::pow(x, GrooveExponent);
        profile.slope += style.seamDepthMeters * GrooveExponent * std::pow(x, GrooveExponent - 1.0f) / w;
    }
    const float reach = std::max(1e-5f, style.panelPuffReachMeters);
    const float t = std::min(e / reach, 1.0f);
    profile.offset += style.panelPuffMeters * (1.0f - (1.0f - t) * (1.0f - t)) - style.panelPuffMeters;
    if (e < reach) profile.slope += 2.0f * style.panelPuffMeters * (1.0f - t) / reach;
    return profile;
}

struct RingLevel {
    float edgeDistanceMeters = 0.0f;
    // Bit l: o anel entra no nivel de detalhe l.
    std::uint32_t lodMask = 0;
};

std::vector<RingLevel> ringLevels(const SoccerBallStyle3D& style, float apothemMeters) {
    const float w = style.seamHalfWidthMeters;
    std::vector<RingLevel> levels {
        { 0.0f, 7u }, { 0.12f * w, 1u }, { 0.3f * w, 3u }, { 0.55f * w, 1u }, { 0.8f * w, 1u },
        { w, 7u },
    };
    const std::array<RingLevel, 8> interior { {
        { 0.0028f, 1u }, { 0.0043f, 3u }, { 0.0062f, 1u }, { 0.0085f, 7u },
        { 0.012f, 1u }, { 0.017f, 3u }, { 0.023f, 1u }, { 0.030f, 1u },
    } };
    for (const RingLevel& level : interior) {
        if (level.edgeDistanceMeters > w * 1.4f && level.edgeDistanceMeters < apothemMeters - 0.004f) {
            levels.push_back(level);
        }
    }
    // Do centro para fora.
    std::reverse(levels.begin(), levels.end());
    return levels;
}

} // namespace

SoccerBallMesh3D buildProceduralSoccerBall3D(const SoccerBallStyle3D& style) {
    const std::uint32_t segments = std::max(4u, style.edgeSegments / 4u * 4u);
    const float radius = style.radiusMeters;
    SoccerBallMesh3D result;
    result.radiusMeters = radius;
    result.lods.resize(2);
    MeshData3D& mesh = result.mesh;

    constexpr float SoftMinimumMeters = 0.0002f;
    for (const Panel& panel : truncatedIcosahedronPanels()) {
        const std::size_t sides = panel.corners.size();
        // Normais dos grandes circulos das arestas, para dentro do gomo.
        std::vector<Vec3> edgeNormals(sides);
        for (std::size_t k = 0; k < sides; ++k) {
            edgeNormals[k] = cross(panel.corners[k], panel.corners[(k + 1) % sides]).normalized();
        }
        const Vec3 centerDirection = panel.center.normalized();
        const float apothemMeters = radius * std::asin(std::clamp(dot(centerDirection, edgeNormals[0]), -1.0f, 1.0f));
        const std::vector<RingLevel> levels = ringLevels(style, apothemMeters);

        std::vector<Vec3> boundary;
        boundary.reserve(sides * segments);
        for (std::size_t k = 0; k < sides; ++k) {
            const Vec3 from = panel.corners[k];
            const Vec3 to = panel.corners[(k + 1) % sides];
            for (std::uint32_t i = 0; i < segments; ++i) {
                boundary.push_back(from + (to - from) * (static_cast<float>(i) / static_cast<float>(segments)));
            }
        }
        const auto loop = static_cast<std::uint32_t>(boundary.size());

        const auto emit = [&](Vec3 planar) {
            const Vec3 u = planar.normalized();
            // Distancia suave ate a costura mais proxima (arredonda os cantos).
            float sum = 0.0f;
            std::array<float, 6> distances {};
            float nearest = 1e9f;
            for (std::size_t k = 0; k < sides; ++k) {
                distances[k] = radius * std::asin(std::clamp(dot(u, edgeNormals[k]), -1.0f, 1.0f));
                nearest = std::min(nearest, distances[k]);
            }
            Vec3 gradient {};
            for (std::size_t k = 0; k < sides; ++k) {
                const float weight = std::exp(-(distances[k] - nearest) / SoftMinimumMeters);
                sum += weight;
                const float along = dot(u, edgeNormals[k]);
                const Vec3 tangential = edgeNormals[k] - u * along;
                gradient += tangential * (weight * radius / std::sqrt(std::max(1e-8f, 1.0f - along * along)));
            }
            gradient *= 1.0f / sum;
            const float e = nearest - SoftMinimumMeters * std::log(sum);
            const Profile profile = panelProfile(style, e);
            const float surface = radius + profile.offset;
            MeshVertex3D vertex;
            vertex.position = u * surface;
            vertex.normal = (u - gradient * (profile.slope / surface)).normalized();
            const float seam = 1.0f - style.seamShadow * (1.0f - smoothstep(0.0f, style.seamHalfWidthMeters * 1.2f, e));
            vertex.color = panel.black ? style.blackColor : style.whiteColor * seam;
            mesh.vertices.push_back(vertex);
        };

        const auto base = static_cast<std::uint32_t>(mesh.vertices.size());
        emit(panel.center);
        for (const RingLevel& level : levels) {
            const float t = std::clamp(1.0f - level.edgeDistanceMeters / apothemMeters, 0.0f, 1.0f);
            for (const Vec3& point : boundary) emit(panel.center + (point - panel.center) * t);
        }
        const auto ringVertex = [&](std::size_t r, std::uint32_t m) {
            return base + 1u + static_cast<std::uint32_t>(r) * loop + (m % loop);
        };

        for (std::uint32_t lod = 0; lod < 3; ++lod) {
            std::vector<std::uint32_t>& indices = lod == 0 ? mesh.indices : result.lods[lod - 1];
            const std::uint32_t step = 1u << lod;
            bool fromCenter = true;
            std::size_t inner = 0;
            for (std::size_t r = 0; r < levels.size(); ++r) {
                if ((levels[r].lodMask & (1u << lod)) == 0) continue;
                for (std::uint32_t m = 0; m < loop; m += step) {
                    if (fromCenter) {
                        indices.insert(indices.end(), { base, ringVertex(r, m), ringVertex(r, m + step) });
                        continue;
                    }
                    const std::uint32_t a = ringVertex(inner, m), b = ringVertex(inner, m + step);
                    const std::uint32_t c = ringVertex(r, m + step), d = ringVertex(r, m);
                    indices.insert(indices.end(), { a, d, c, a, c, b });
                }
                fromCenter = false;
                inner = r;
            }
        }
    }

    // Gaiola: cada vertice preso aos tres pontos do triangulo que o contem.
    const Icosphere cage = buildIcosphere(style.cageSubdivisions);
    result.cageDirections = cage.directions;
    std::vector<std::vector<std::uint32_t>> trianglesOfPoint(cage.directions.size());
    std::vector<std::vector<std::uint32_t>> neighboursOfPoint(cage.directions.size());
    for (std::uint32_t t = 0; t < cage.triangles.size(); ++t) {
        for (std::size_t k = 0; k < 3; ++k) {
            const std::uint32_t point = cage.triangles[t][k];
            trianglesOfPoint[point].push_back(t);
            neighboursOfPoint[point].push_back(cage.triangles[t][(k + 1) % 3]);
        }
    }
    const auto determinant = [](Vec3 a, Vec3 b, Vec3 c) { return dot(a, cross(b, c)); };
    // Ponto mais proximo por caminhada na gaiola, partindo do ultimo achado:
    // vertices vizinhos na malha sao vizinhos na esfera.
    std::uint32_t nearest = 0;
    for (MeshVertex3D& vertex : mesh.vertices) {
        const Vec3 u = vertex.position.normalized();
        float best = dot(u, cage.directions[nearest]);
        for (bool improved = true; improved;) {
            improved = false;
            for (const std::uint32_t neighbour : neighboursOfPoint[nearest]) {
                const float alignment = dot(u, cage.directions[neighbour]);
                if (alignment > best) {
                    best = alignment;
                    nearest = neighbour;
                    improved = true;
                }
            }
        }
        float bestMinimum = -1e9f;
        const auto tryTriangle = [&](std::uint32_t t) {
            const Triangle& triangle = cage.triangles[t];
            const Vec3 a = cage.directions[triangle[0]], b = cage.directions[triangle[1]];
            const Vec3 c = cage.directions[triangle[2]];
            std::array<float, 3> weights { determinant(u, b, c), determinant(a, u, c), determinant(a, b, u) };
            const float total = weights[0] + weights[1] + weights[2];
            if (total <= 0.0f) return;
            for (float& weight : weights) weight /= total;
            const float minimum = std::min({ weights[0], weights[1], weights[2] });
            if (minimum <= bestMinimum) return;
            bestMinimum = minimum;
            for (std::size_t k = 0; k < 3; ++k) {
                vertex.joints[k] = triangle[k];
                vertex.weights[k] = std::max(0.0f, weights[k]);
            }
            vertex.joints[3] = 0;
            vertex.weights[3] = 0.0f;
        };
        for (const std::uint32_t t : trianglesOfPoint[nearest]) tryTriangle(t);
        if (bestMinimum < -1e-4f) {
            for (std::uint32_t t = 0; t < cage.triangles.size(); ++t) tryTriangle(t);
        }
        const float total = vertex.weights[0] + vertex.weights[1] + vertex.weights[2];
        if (total <= 0.0f) throw std::runtime_error("Bola procedural: vertice fora da gaiola");
        for (std::size_t k = 0; k < 3; ++k) vertex.weights[k] /= total;
    }
    recomputeBounds(mesh);
    return result;
}

float soccerBallPlaneCompression3D(Vec3 center, float radiusMeters,
    const SoccerBallContactPlane3D& plane) {
    return radiusMeters - dot(center - plane.pointWorld, plane.normalWorld);
}

void buildSoccerBallSkin3D(std::span<const Vec3> cageDirections, float radiusMeters,
    Vec3 center, Quaternion orientation,
    std::span<const SoccerBallContactPlane3D> planes, std::vector<Mat4>& outMatrices) {
    struct Active {
        Vec3 normal;
        float compression = 0.0f;
        float rim = 0.0f;
    };
    std::array<Active, 8> active {};
    std::size_t activeCount = 0;
    for (const SoccerBallContactPlane3D& plane : planes) {
        if (activeCount == active.size()) break;
        const float compression = std::min(0.75f * radiusMeters,
            soccerBallPlaneCompression3D(center, radiusMeters, plane));
        if (compression <= SoccerBallVisibleCompressionMeters) continue;
        // Largura da curva da borda: a casca dobra num raio que cresce com o
        // achatamento (uma quina viva seria uma casca sem rigidez nenhuma).
        active[activeCount++] = { plane.normalWorld.normalized(), compression,
            std::clamp(0.6f * compression, 0.0008f, 0.014f) };
    }

    orientation = orientation.normalized();
    outMatrices.resize(cageDirections.size());
    for (std::size_t j = 0; j < cageDirections.size(); ++j) {
        const Vec3 direction = orientation.rotate(cageDirections[j]);
        Vec3 offset {};
        Vec3 normal = direction;
        Quaternion bend {};
        for (std::size_t p = 0; p < activeCount; ++p) {
            const Active& plane = active[p];
            // Altura do ponto de repouso sobre o plano. Max suave com zero: a
            // mancha fica plana, a borda curva e sobe um pouco (k/4 no maximo).
            const float height = radiusMeters * dot(direction, plane.normal) + radiusMeters - plane.compression;
            const float k = plane.rim;
            if (height >= k) continue;
            float flattened = 0.0f;
            float slope = 0.0f;
            if (height > -k) {
                const float x = 1.0f - std::abs(height) / k;
                flattened = std::max(height, 0.0f) + 0.25f * k * x * x;
                slope = height > 0.0f ? 1.0f - 0.5f * (1.0f - height / k) : 0.5f * (1.0f + height / k);
            }
            offset += plane.normal * (flattened - height);
            // A superficie achatada e o grafico f(altura) sobre o plano: a
            // parte tangencial da normal encolhe pela derivada.
            const float along = dot(normal, plane.normal);
            const Vec3 bent = ((normal - plane.normal * along) * slope + plane.normal * along).normalized();
            const Vec3 axis = cross(normal, bent);
            const float cosine = dot(normal, bent);
            if (cosine > -0.999f) {
                bend = (Quaternion { axis.x, axis.y, axis.z, 1.0f + cosine }.normalized() * bend).normalized();
            }
            normal = bent;
        }
        const Quaternion rotation = (bend * orientation).normalized();
        const Vec3 rest = cageDirections[j] * radiusMeters;
        const Vec3 translation = center + offset + direction * radiusMeters - rotation.rotate(rest);
        Mat4 matrix = Mat4::rotation(rotation);
        matrix.at(0, 3) = translation.x;
        matrix.at(1, 3) = translation.y;
        matrix.at(2, 3) = translation.z;
        outMatrices[j] = matrix;
    }
}

} // namespace MatterEngine
