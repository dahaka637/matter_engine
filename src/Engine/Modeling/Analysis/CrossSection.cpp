#include "Engine/Modeling/Analysis/CrossSection.hpp"

#include <algorithm>
#include <cmath>
#include <set>
#include <unordered_map>

namespace MatterEngine::Modeling {

namespace {

std::uint64_t edgeKey(std::uint32_t a, std::uint32_t b) {
    return (static_cast<std::uint64_t>(std::min(a, b)) << 32) | std::max(a, b);
}

struct Segment {
    std::uint64_t keyA = 0;
    Vec3 a;
    std::uint64_t keyB = 0;
    Vec3 b;
};

void finishLoop(SectionLoop& loop) {
    loop.length = 0.0f;
    Vec3 sum {};
    for (std::size_t i = 0; i < loop.points.size(); ++i) {
        sum += loop.points[i];
        if (i + 1 < loop.points.size()) loop.length += distance(loop.points[i], loop.points[i + 1]);
    }
    if (loop.closed && loop.points.size() > 1) loop.length += distance(loop.points.back(), loop.points.front());
    loop.centroid = loop.points.empty() ? Vec3 {} : sum / static_cast<float>(loop.points.size());
}

} // namespace

float MeshSection::totalLength() const {
    float total = 0.0f;
    for (const SectionLoop& loop : loops) total += loop.length;
    return total;
}

MeshSection sliceMesh(const Mesh& mesh, std::span<const Vec3> positions, const Plane& plane,
    std::span<const int> partFilter) {
    MeshSection section;
    section.plane = plane;
    if (positions.size() != mesh.positions.size()) positions = mesh.positions;
    const std::set<int> filter(partFilter.begin(), partFilter.end());
    std::vector<Segment> segments;
    for (std::size_t t = 0; t < mesh.triangles.size(); ++t) {
        if (!filter.empty() && filter.count(mesh.partOf(t)) == 0) continue;
        const Triangle& tri = mesh.triangles[t];
        float d[3];
        for (int i = 0; i < 3; ++i) {
            d[i] = plane.signedDistance(positions[tri[static_cast<std::size_t>(i)]]);
            if (std::abs(d[i]) < 1e-9f) d[i] = 1e-9f;
        }
        Segment segment;
        int found = 0;
        for (int i = 0; i < 3 && found < 2; ++i) {
            const int j = (i + 1) % 3;
            if (d[i] * d[j] >= 0.0f) continue;
            const std::uint32_t vi = tri[static_cast<std::size_t>(i)], vj = tri[static_cast<std::size_t>(j)];
            // O mesmo ponto pelos dois triangulos da aresta: do menor indice.
            const bool forward = vi < vj;
            const Vec3 from = positions[forward ? vi : vj], to = positions[forward ? vj : vi];
            const float dFrom = forward ? d[i] : d[j], dTo = forward ? d[j] : d[i];
            const Vec3 point = lerp(from, to, dFrom / (dFrom - dTo));
            if (found == 0) {
                segment.keyA = edgeKey(vi, vj);
                segment.a = point;
            } else {
                segment.keyB = edgeKey(vi, vj);
                segment.b = point;
            }
            ++found;
        }
        if (found == 2) segments.push_back(segment);
    }

    std::unordered_map<std::uint64_t, std::vector<std::size_t>> byKey;
    for (std::size_t i = 0; i < segments.size(); ++i) {
        byKey[segments[i].keyA].push_back(i);
        byKey[segments[i].keyB].push_back(i);
    }
    std::vector<bool> used(segments.size(), false);
    const auto nextSegment = [&](std::uint64_t key) -> long long {
        const auto found = byKey.find(key);
        if (found == byKey.end()) return -1;
        for (const std::size_t index : found->second)
            if (!used[index]) return static_cast<long long>(index);
        return -1;
    };
    for (std::size_t start = 0; start < segments.size(); ++start) {
        if (used[start]) continue;
        used[start] = true;
        SectionLoop loop;
        std::vector<Vec3> forwardPoints { segments[start].a, segments[start].b };
        const std::uint64_t startKey = segments[start].keyA;
        std::uint64_t key = segments[start].keyB;
        while (true) {
            if (key == startKey) {
                loop.closed = true;
                forwardPoints.pop_back();
                break;
            }
            const long long next = nextSegment(key);
            if (next < 0) break;
            used[static_cast<std::size_t>(next)] = true;
            const Segment& segment = segments[static_cast<std::size_t>(next)];
            const bool fromA = segment.keyA == key;
            forwardPoints.push_back(fromA ? segment.b : segment.a);
            key = fromA ? segment.keyB : segment.keyA;
        }
        std::vector<Vec3> backwardPoints;
        if (!loop.closed) {
            key = startKey;
            while (true) {
                const long long next = nextSegment(key);
                if (next < 0) break;
                used[static_cast<std::size_t>(next)] = true;
                const Segment& segment = segments[static_cast<std::size_t>(next)];
                const bool fromA = segment.keyA == key;
                backwardPoints.push_back(fromA ? segment.b : segment.a);
                key = fromA ? segment.keyB : segment.keyA;
            }
        }
        loop.points.assign(backwardPoints.rbegin(), backwardPoints.rend());
        loop.points.insert(loop.points.end(), forwardPoints.begin(), forwardPoints.end());
        finishLoop(loop);
        section.loops.push_back(std::move(loop));
    }
    return section;
}

bool loopContains(const SectionLoop& loop, const Plane& plane, Vec3 point) {
    if (loop.points.size() < 3) return false;
    const Vec3 u = anyPerpendicular(plane.normal);
    const Vec3 v = cross(plane.normal, u);
    const float px = dot(point, u), py = dot(point, v);
    bool inside = false;
    const std::size_t count = loop.points.size();
    for (std::size_t i = 0, j = count - 1; i < count; j = i++) {
        const float xi = dot(loop.points[i], u), yi = dot(loop.points[i], v);
        const float xj = dot(loop.points[j], u), yj = dot(loop.points[j], v);
        if ((yi > py) != (yj > py) && px < (xj - xi) * (py - yi) / (yj - yi) + xi) inside = !inside;
    }
    return inside;
}

int limbChild(const Skeleton& skeleton, int joint) {
    if (joint < 0 || joint >= static_cast<int>(skeleton.joints.size())) return -1;
    const std::vector<int> children = skeleton.children(joint);
    if (children.empty()) return -1;
    for (const int child : children)
        if (skeleton.joints[static_cast<std::size_t>(child)].name.find("Middle1") != std::string::npos) return child;
    const Side side = skeleton.joints[static_cast<std::size_t>(joint)].side;
    for (const int child : children) {
        const Joint& candidate = skeleton.joints[static_cast<std::size_t>(child)];
        if (candidate.kind != JointKind::Tip && candidate.kind != JointKind::Helper && candidate.side == side) return child;
    }
    return children.front();
}

BoneSection sectionAcrossBone(const Mesh& mesh, std::span<const Vec3> positions, const Skeleton& skeleton, int joint,
    float t, std::span<const int> partFilter) {
    BoneSection result;
    const int child = limbChild(skeleton, joint);
    if (child < 0) return result;
    const Joint& from = skeleton.joints[static_cast<std::size_t>(joint)];
    const Vec3 a = from.bind.position, b = skeleton.joints[static_cast<std::size_t>(child)].bind.position;
    const Vec3 axis = safeNormalized(b - a, axisX(from.bind.orientation));
    result.point = lerp(a, b, t);
    const Plane plane = Plane::fromPointNormal(result.point, axis);
    const MeshSection section = sliceMesh(mesh, positions, plane, partFilter);
    if (section.loops.empty()) return result;
    const SectionLoop* chosen = nullptr;
    for (const SectionLoop& loop : section.loops)
        if (loop.closed && loopContains(loop, plane, result.point) && (chosen == nullptr || loop.length < chosen->length))
            chosen = &loop;
    result.inside = chosen != nullptr;
    if (chosen == nullptr)
        for (const SectionLoop& loop : section.loops)
            if (chosen == nullptr || distance(loop.centroid, result.point) < distance(chosen->centroid, result.point))
                chosen = &loop;
    result.found = true;
    result.loop = *chosen;
    result.circumference = chosen->length;
    result.offset = distance(chosen->centroid, result.point);
    // Largura e profundidade nos eixos da junta (perpendiculares ao osso).
    Vec3 u = axisY(from.bind.orientation);
    u = safeNormalized(u - axis * dot(u, axis), anyPerpendicular(axis));
    const Vec3 v = cross(axis, u);
    float minU = 1e9f, maxU = -1e9f, minV = 1e9f, maxV = -1e9f;
    for (const Vec3& point : chosen->points) {
        minU = std::min(minU, dot(point, u));
        maxU = std::max(maxU, dot(point, u));
        minV = std::min(minV, dot(point, v));
        maxV = std::max(maxV, dot(point, v));
    }
    result.width = maxU - minU;
    result.depth = maxV - minV;
    return result;
}

void checkJointCentering(const Mesh& mesh, const Skeleton& skeleton, const BodyRegions* regions,
    std::span<const int> jointRegion, Report& report) {
    if (mesh.triangles.empty() || regions == nullptr) return;
    std::vector<bool> regionHasMesh(regions->regions.size(), false);
    for (const MeshPart& part : mesh.parts) {
        const int region = regions->find(part.region);
        if (region >= 0) regionHasMesh[static_cast<std::size_t>(region)] = true;
    }
    std::size_t checked = 0, outside = 0;
    for (std::size_t j = 0; j < skeleton.joints.size(); ++j) {
        const Joint& joint = skeleton.joints[j];
        if (joint.kind == JointKind::Tip || joint.kind == JointKind::Helper) continue;
        const int region = j < jointRegion.size() ? jointRegion[j] : -1;
        if (region < 0 || !regionHasMesh[static_cast<std::size_t>(region)]) continue;
        const BoneSection section = sectionAcrossBone(mesh, mesh.positions, skeleton, static_cast<int>(j), 0.02f);
        if (!section.found) continue;
        ++checked;
        const std::string regionId = regions->regions[static_cast<std::size_t>(region)].id;
        if (!section.inside) {
            ++outside;
            Finding finding;
            finding.severity = Severity::Warning;
            finding.code = "junta_fora_da_malha";
            finding.message = "a junta " + joint.name + " fica fora da malha (o osso sai do corpo)";
            finding.region = regionId;
            finding.location = section.point;
            finding.hasLocation = true;
            finding.points.push_back(section.point);
            report.add(std::move(finding));
            continue;
        }
        const float radius = section.circumference / (2.0f * Pi);
        if (radius > 1e-4f && section.offset > 0.55f * radius) {
            Finding finding;
            finding.severity = Severity::Info;
            finding.code = "osso_fora_do_centro";
            finding.message = "o osso de " + joint.name + " passa a " + formatNumber(section.offset / radius * 100.0f, 0)
                + "% do raio do centro do membro";
            finding.region = regionId;
            finding.location = section.point;
            finding.hasLocation = true;
            finding.points = { section.point, section.loop.centroid };
            for (std::size_t i = 0; i + 1 < section.loop.points.size(); ++i)
                finding.segments.push_back({ section.loop.points[i], section.loop.points[i + 1] });
            report.add(std::move(finding));
        }
    }
    report.statistic("juntas dentro da malha", std::to_string(checked - outside) + " de " + std::to_string(checked));
}

} // namespace MatterEngine::Modeling
