#include "Engine/Modeling/Mesh/MeshBuilder.hpp"

#include "Engine/Modeling/Skeleton/Skeleton.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <map>
#include <unordered_map>

namespace MatterEngine::Modeling {

MeshBuilder::MeshBuilder(Mesh& mesh) : m_mesh(mesh) {}

std::uint16_t MeshBuilder::ensurePart(std::string_view name, std::string_view region) {
    const int existing = m_mesh.findPart(name);
    if (existing >= 0) {
        if (!region.empty()) m_mesh.parts[static_cast<std::size_t>(existing)].region = std::string(region);
        return static_cast<std::uint16_t>(existing);
    }
    MeshPart part;
    part.name = std::string(name);
    part.region = std::string(region);
    m_mesh.parts.push_back(std::move(part));
    return static_cast<std::uint16_t>(m_mesh.parts.size() - 1);
}

MeshBuilder& MeshBuilder::part(std::string_view name, std::string_view region) {
    m_part = ensurePart(name, region);
    return *this;
}

MeshBuilder& MeshBuilder::connectTo(std::string_view other) {
    if (m_part < 0) part("sem_parte");
    std::vector<std::string>& list = m_mesh.parts[static_cast<std::size_t>(m_part)].connectedTo;
    if (std::find(list.begin(), list.end(), other) == list.end()) list.emplace_back(other);
    return *this;
}

MeshBuilder& MeshBuilder::opening(std::string_view name) {
    if (m_part < 0) part("sem_parte");
    m_mesh.parts[static_cast<std::size_t>(m_part)].openings.emplace_back(name);
    return *this;
}

MeshBuilder& MeshBuilder::root(bool value) {
    if (m_part < 0) part("sem_parte");
    m_mesh.parts[static_cast<std::size_t>(m_part)].root = value;
    return *this;
}

MeshBuilder& MeshBuilder::material(std::string_view name, Color color) {
    const int existing = m_mesh.findMaterial(name);
    if (existing >= 0) {
        m_mesh.materials[static_cast<std::size_t>(existing)].color = color;
        m_material = existing;
    } else {
        m_mesh.materials.push_back({ std::string(name), color });
        m_material = static_cast<int>(m_mesh.materials.size() - 1);
    }
    return *this;
}

MeshBuilder& MeshBuilder::useMaterial(std::string_view name) {
    const int existing = m_mesh.findMaterial(name);
    if (existing >= 0) m_material = existing;
    else material(name, Palette::Surface);
    return *this;
}

std::uint32_t MeshBuilder::vertex(Vec3 position) {
    m_mesh.positions.push_back(position);
    if (!m_mesh.skin.empty()) m_mesh.skin.emplace_back();
    return static_cast<std::uint32_t>(m_mesh.positions.size() - 1);
}

void MeshBuilder::triangle(std::uint32_t a, std::uint32_t b, std::uint32_t c) {
    if (a == b || b == c || a == c) return;
    if (m_part < 0) part("sem_parte");
    if (m_material < 0) material("padrao", Palette::Surface);
    m_mesh.triangles.push_back({ a, b, c });
    m_mesh.trianglePart.push_back(static_cast<std::uint16_t>(m_part));
    m_mesh.triangleMaterial.push_back(static_cast<std::uint16_t>(m_material));
    if (!m_mesh.triangleUv.empty()) m_mesh.triangleUv.emplace_back();
}

void MeshBuilder::quad(std::uint32_t a, std::uint32_t b, std::uint32_t c, std::uint32_t d) {
    const std::vector<Vec3>& p = m_mesh.positions;
    if ((p[a] - p[c]).lengthSquared() <= (p[b] - p[d]).lengthSquared()) {
        triangle(a, b, c);
        triangle(a, c, d);
    } else {
        triangle(a, b, d);
        triangle(b, c, d);
    }
}

void MeshBuilder::polygon(std::span<const std::uint32_t> loop) {
    for (std::size_t i = 1; i + 1 < loop.size(); ++i) triangle(loop[0], loop[i], loop[i + 1]);
}

std::vector<std::uint32_t> MeshBuilder::ring(const Transform& frame, std::span<const Point2> profile) {
    std::vector<std::uint32_t> indices;
    indices.reserve(profile.size());
    for (const Point2& point : profile) indices.push_back(vertex(frame.apply({ 0.0f, point.x, point.y })));
    return indices;
}

std::vector<std::uint32_t> MeshBuilder::ellipseRing(const Transform& frame, float radiusY, float radiusZ, int count,
    float phaseRadians) {
    std::vector<Point2> profile;
    count = std::max(3, count);
    for (int k = 0; k < count; ++k) {
        const float angle = phaseRadians + 2.0f * Pi * static_cast<float>(k) / static_cast<float>(count);
        profile.push_back({ radiusY * std::cos(angle), radiusZ * std::sin(angle) });
    }
    return ring(frame, profile);
}

void MeshBuilder::bridge(std::span<const std::uint32_t> from, std::span<const std::uint32_t> to) {
    const std::size_t count = std::min(from.size(), to.size());
    for (std::size_t k = 0; k < count; ++k) {
        const std::size_t next = (k + 1) % count;
        quad(from[k], from[next], to[next], to[k]);
    }
}

void MeshBuilder::cap(std::span<const std::uint32_t> ringIndices, bool atStart, Cap kind, Vec3 tip) {
    if (kind == Cap::Open || ringIndices.size() < 3) return;
    Vec3 center {};
    for (const std::uint32_t index : ringIndices) center += m_mesh.positions[index];
    center = center / static_cast<float>(ringIndices.size());
    const std::uint32_t hub = vertex(kind == Cap::Point ? tip : center);
    const std::size_t count = ringIndices.size();
    for (std::size_t k = 0; k < count; ++k) {
        const std::uint32_t current = ringIndices[k], next = ringIndices[(k + 1) % count];
        if (atStart) triangle(hub, next, current);
        else triangle(hub, current, next);
    }
}

void MeshBuilder::loft(std::span<const std::vector<std::uint32_t>> rings, Cap startCap, Cap endCap, Vec3 startTip,
    Vec3 endTip) {
    for (std::size_t i = 0; i + 1 < rings.size(); ++i) bridge(rings[i], rings[i + 1]);
    if (!rings.empty()) {
        cap(rings.front(), true, startCap, startTip);
        cap(rings.back(), false, endCap, endTip);
    }
}

void MeshBuilder::box(const Transform& frame, Vec3 h) {
    std::array<std::uint32_t, 8> c {};
    for (int i = 0; i < 8; ++i)
        c[static_cast<std::size_t>(i)] = vertex(frame.apply({ i & 1 ? h.x : -h.x, i & 2 ? h.y : -h.y,
            i & 4 ? h.z : -h.z }));
    quad(c[1], c[3], c[7], c[5]); // +x
    quad(c[0], c[4], c[6], c[2]); // -x
    quad(c[2], c[6], c[7], c[3]); // +y
    quad(c[0], c[1], c[5], c[4]); // -y
    quad(c[4], c[5], c[7], c[6]); // +z
    quad(c[0], c[2], c[3], c[1]); // -z
}

void MeshBuilder::ellipsoid(const Transform& frame, Vec3 radii, int segments, int rings) {
    rings = std::max(2, rings);
    std::vector<std::vector<std::uint32_t>> loops;
    for (int i = 1; i < rings; ++i) {
        const float phi = Pi * static_cast<float>(i) / static_cast<float>(rings);
        const Transform ringFrame { frame.apply({ -radii.x * std::cos(phi), 0.0f, 0.0f }), frame.orientation };
        loops.push_back(ellipseRing(ringFrame, radii.y * std::sin(phi), radii.z * std::sin(phi), segments));
    }
    loft(loops, Cap::Point, Cap::Point, frame.apply({ -radii.x, 0.0f, 0.0f }), frame.apply({ radii.x, 0.0f, 0.0f }));
}

void MeshBuilder::tube(Vec3 from, Vec3 to, float radiusFrom, float radiusTo, int segments, Cap caps) {
    const Vec3 axis = safeNormalized(to - from, { 1.0f, 0.0f, 0.0f });
    const Quaternion orientation = frameAlong(axis, { 0.0f, 0.0f, 1.0f });
    std::vector<std::vector<std::uint32_t>> loops;
    loops.push_back(ellipseRing({ from, orientation }, radiusFrom, radiusFrom, segments));
    loops.push_back(ellipseRing({ to, orientation }, radiusTo, radiusTo, segments));
    loft(loops, caps, caps, from - axis * radiusFrom, to + axis * radiusTo);
}

void MeshBuilder::mirrorParts(std::span<const std::string> names, std::span<const int> jointMirror) {
    // Primeiro sai o espelho antigo (removePart renumera os vertices).
    for (const std::string& name : names) {
        const std::string targetName = mirroredName(name);
        if (targetName != name && m_mesh.findPart(name) >= 0) removePart(targetName);
    }
    // Um mapa so para todas as partes: o vertice dividido entre duas partes
    // continua dividido no espelho; os do plano y = 0 sao os mesmos.
    std::unordered_map<std::uint32_t, std::uint32_t> copied;
    const auto copyVertex = [&](std::uint32_t original) {
        const auto found = copied.find(original);
        if (found != copied.end()) return found->second;
        const Vec3 p = m_mesh.positions[original];
        if (std::abs(p.y) <= 1e-6f) {
            copied.emplace(original, original);
            // No meio os pesos sao dos dois lados: a media do lado direito com
            // o espelho dele (o gancho segue as duas coxas, nao so uma).
            if (!m_mesh.skin.empty() && original < m_mesh.skin.size()) {
                std::map<std::uint16_t, float> both;
                const SkinWeights& weights = m_mesh.skin[original];
                for (std::size_t i = 0; i < 4; ++i) {
                    if (weights.weights[i] <= 0.0f) continue;
                    const std::uint16_t joint = weights.joints[i];
                    const std::uint16_t mirrored = joint < jointMirror.size() && jointMirror[joint] >= 0
                        ? static_cast<std::uint16_t>(jointMirror[joint]) : joint;
                    both[joint] += 0.5f * weights.weights[i];
                    both[mirrored] += 0.5f * weights.weights[i];
                }
                std::vector<std::pair<float, std::uint16_t>> ranked;
                for (const auto& [joint, weight] : both) ranked.push_back({ weight, joint });
                std::sort(ranked.begin(), ranked.end(), [](const auto& a, const auto& b) { return a.first > b.first; });
                if (ranked.size() > 4) ranked.resize(4);
                float total = 0.0f;
                for (const auto& [weight, joint] : ranked) total += weight;
                if (total > 0.0f) {
                    SkinWeights symmetric {};
                    for (std::size_t i = 0; i < ranked.size(); ++i) {
                        symmetric.joints[i] = ranked[i].second;
                        symmetric.weights[i] = ranked[i].first / total;
                    }
                    m_mesh.skin[original] = symmetric;
                }
            }
            return original;
        }
        m_mesh.positions.push_back({ p.x, -p.y, p.z });
        if (!m_mesh.skin.empty()) {
            SkinWeights weights = m_mesh.skin[original];
            for (std::uint16_t& joint : weights.joints)
                if (joint < jointMirror.size() && jointMirror[joint] >= 0)
                    joint = static_cast<std::uint16_t>(jointMirror[joint]);
            m_mesh.skin.push_back(weights);
        }
        const std::uint32_t index = static_cast<std::uint32_t>(m_mesh.positions.size() - 1);
        copied.emplace(original, index);
        return index;
    };
    const std::size_t triangleCount = m_mesh.triangles.size();
    for (const std::string& name : names) {
        const int sourceIndex = m_mesh.findPart(name);
        if (sourceIndex < 0) continue;
        const std::string targetName = mirroredName(name);
        std::uint16_t target = static_cast<std::uint16_t>(sourceIndex);
        if (targetName != name) {
            const MeshPart sourcePart = m_mesh.parts[static_cast<std::size_t>(sourceIndex)];
            target = ensurePart(targetName, mirroredName(sourcePart.region));
            MeshPart& mirrored = m_mesh.parts[target];
            mirrored.connectedTo.clear();
            for (const std::string& other : sourcePart.connectedTo) mirrored.connectedTo.push_back(mirroredName(other));
            mirrored.openings = sourcePart.openings;
            mirrored.root = sourcePart.root;
        }
        for (std::size_t i = 0; i < triangleCount; ++i) {
            if (m_mesh.trianglePart[i] != sourceIndex) continue;
            const Triangle t = m_mesh.triangles[i];
            const std::uint32_t a = copyVertex(t[0]), b = copyVertex(t[1]), c = copyVertex(t[2]);
            // O espelho inverte a orientacao: troca dois vertices.
            m_mesh.triangles.push_back({ a, c, b });
            m_mesh.trianglePart.push_back(target);
            m_mesh.triangleMaterial.push_back(m_mesh.triangleMaterial[i]);
            if (!m_mesh.triangleUv.empty()) {
                const std::array<Uv, 3> uv = m_mesh.triangleUv[i];
                m_mesh.triangleUv.push_back({ uv[0], uv[2], uv[1] });
            }
        }
    }
}

std::size_t MeshBuilder::weld(float tolerance, bool samePart) {
    const std::size_t count = m_mesh.positions.size();
    if (count == 0) return 0;
    std::vector<int> vertexPart(count, -1);
    for (std::size_t i = 0; i < m_mesh.triangles.size(); ++i)
        for (const std::uint32_t vertex : m_mesh.triangles[i])
            if (vertexPart[vertex] < 0) vertexPart[vertex] = m_mesh.trianglePart[i];

    const float cell = std::max(tolerance, 1e-6f);
    const auto key = [cell](Vec3 p) {
        return std::array<long long, 3> { static_cast<long long>(std::floor(p.x / cell)),
            static_cast<long long>(std::floor(p.y / cell)), static_cast<long long>(std::floor(p.z / cell)) };
    };
    std::map<std::array<long long, 3>, std::vector<std::uint32_t>> grid;
    std::vector<std::uint32_t> remap(count);
    std::size_t merged = 0;
    for (std::uint32_t v = 0; v < count; ++v) {
        const Vec3 p = m_mesh.positions[v];
        const auto base = key(p);
        std::uint32_t found = v;
        for (long long dx = -1; dx <= 1 && found == v; ++dx)
            for (long long dy = -1; dy <= 1 && found == v; ++dy)
                for (long long dz = -1; dz <= 1 && found == v; ++dz) {
                    const auto cellIt = grid.find({ base[0] + dx, base[1] + dy, base[2] + dz });
                    if (cellIt == grid.end()) continue;
                    for (const std::uint32_t other : cellIt->second) {
                        if (samePart && vertexPart[other] != vertexPart[v]) continue;
                        if ((m_mesh.positions[other] - p).length() <= tolerance) {
                            found = other;
                            break;
                        }
                    }
                }
        remap[v] = found;
        if (found == v) grid[base].push_back(v);
        else ++merged;
    }
    for (Triangle& t : m_mesh.triangles)
        for (std::uint32_t& vertex : t) vertex = remap[vertex];
    compact();
    return merged;
}

void MeshBuilder::transformPart(int part, const Transform& transform) {
    for (const std::uint32_t vertex : m_mesh.partVertices(part))
        m_mesh.positions[vertex] = transform.apply(m_mesh.positions[vertex]);
}

void MeshBuilder::flipPart(int part) {
    for (std::size_t i = 0; i < m_mesh.triangles.size(); ++i)
        if (m_mesh.trianglePart[i] == part) std::swap(m_mesh.triangles[i][1], m_mesh.triangles[i][2]);
}

void MeshBuilder::removePart(std::string_view name) {
    const int part = m_mesh.findPart(name);
    if (part < 0) return;
    std::size_t write = 0;
    for (std::size_t i = 0; i < m_mesh.triangles.size(); ++i) {
        if (m_mesh.trianglePart[i] == part) continue;
        m_mesh.triangles[write] = m_mesh.triangles[i];
        m_mesh.trianglePart[write] = m_mesh.trianglePart[i] > part
            ? static_cast<std::uint16_t>(m_mesh.trianglePart[i] - 1) : m_mesh.trianglePart[i];
        m_mesh.triangleMaterial[write] = m_mesh.triangleMaterial[i];
        if (!m_mesh.triangleUv.empty()) m_mesh.triangleUv[write] = m_mesh.triangleUv[i];
        ++write;
    }
    m_mesh.triangles.resize(write);
    m_mesh.trianglePart.resize(write);
    m_mesh.triangleMaterial.resize(write);
    if (!m_mesh.triangleUv.empty()) m_mesh.triangleUv.resize(write);
    m_mesh.parts.erase(m_mesh.parts.begin() + part);
    if (m_part == part) m_part = -1;
    else if (m_part > part) --m_part;
    compact();
}

void MeshBuilder::compact() {
    // Sem triangulos degenerados e sem vertices soltos.
    std::size_t write = 0;
    for (std::size_t i = 0; i < m_mesh.triangles.size(); ++i) {
        const Triangle& t = m_mesh.triangles[i];
        if (t[0] == t[1] || t[1] == t[2] || t[0] == t[2]) continue;
        m_mesh.triangles[write] = t;
        m_mesh.trianglePart[write] = m_mesh.trianglePart[i];
        m_mesh.triangleMaterial[write] = m_mesh.triangleMaterial[i];
        if (!m_mesh.triangleUv.empty()) m_mesh.triangleUv[write] = m_mesh.triangleUv[i];
        ++write;
    }
    m_mesh.triangles.resize(write);
    m_mesh.trianglePart.resize(write);
    m_mesh.triangleMaterial.resize(write);
    if (!m_mesh.triangleUv.empty()) m_mesh.triangleUv.resize(write);

    std::vector<std::uint32_t> newIndex(m_mesh.positions.size(), UINT32_MAX);
    std::vector<Vec3> positions;
    std::vector<SkinWeights> skin;
    for (Triangle& t : m_mesh.triangles) {
        for (std::uint32_t& vertex : t) {
            if (newIndex[vertex] == UINT32_MAX) {
                newIndex[vertex] = static_cast<std::uint32_t>(positions.size());
                positions.push_back(m_mesh.positions[vertex]);
                if (m_mesh.hasSkin()) skin.push_back(m_mesh.skin[vertex]);
            }
            vertex = newIndex[vertex];
        }
    }
    m_mesh.positions = std::move(positions);
    m_mesh.skin = std::move(skin);
}

} // namespace MatterEngine::Modeling
