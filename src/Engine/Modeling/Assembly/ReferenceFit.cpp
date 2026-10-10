#include "Engine/Modeling/Assembly/ReferenceFit.hpp"

#include "Engine/Modeling/Assembly/Uniform.hpp"
#include "Engine/Modeling/Import/GltfImport.hpp"
#include "Engine/Modeling/Mesh/SurfaceFit.hpp"
#include "Engine/Modeling/Mesh/TriangleBvh.hpp"

#include <algorithm>
#include <cmath>

namespace MatterEngine::Modeling {

namespace {

// A altura da linha [x, altura] em x (reta entre os pontos; fora, a ponta).
float lineHeight(std::vector<std::array<float, 2>> points, float x) {
    std::sort(points.begin(), points.end(), [](const auto& a, const auto& b) { return a[0] < b[0]; });
    if (x <= points.front()[0]) return points.front()[1];
    for (std::size_t i = 1; i < points.size(); ++i)
        if (x <= points[i][0]) {
            const float span = std::max(1e-9f, points[i][0] - points[i - 1][0]);
            return points[i - 1][1] + (points[i][1] - points[i - 1][1]) * (x - points[i - 1][0]) / span;
        }
    return points.back()[1];
}

float smooth01(float t) {
    t = std::clamp(t, 0.0f, 1.0f);
    return t * t * (3.0f - 2.0f * t);
}

} // namespace

bool fitToReference(Mesh& mesh, const ReferenceFitShape& shape, const std::filesystem::path& directory, const UniformLines& lines,
    int headPart, ReferenceFitReport& report, std::string& error) {
    if (shape.file.empty()) return true;
    if (shape.jawLine.size() < 2) {
        error = "molde: a linha da mandibula precisa de dois pontos";
        return false;
    }
    GltfImportSettings settings;
    settings.part = "molde";
    settings.weldTolerance = 1e-5f;
    const GltfImportResult reference = importGltfMesh(directory / shape.file, settings);
    if (!reference.ok) {
        error = "molde: " + reference.error;
        return false;
    }
    const float ground = lines.ground;

    // ---- O alinhamento: a cabeca acima da boca, sem as orelhas, nos dois
    // lados; a partida pelo alto e pela ponta do nariz ----
    std::vector<bool> isHead(mesh.positions.size(), false);
    for (std::size_t t = 0; t < mesh.triangles.size(); ++t)
        if (mesh.partOf(t) == headPart)
            for (const std::uint32_t v : mesh.triangles[t]) isHead[v] = true;
    std::vector<Vec3> points;
    Vec3 crown { 0.0f, 0.0f, -1e9f }, nose { -1e9f, 0.0f, 0.0f };
    for (std::size_t v = 0; v < mesh.positions.size(); ++v) {
        if (!isHead[v]) continue;
        const Vec3 p = mesh.positions[v];
        if (p.z > crown.z) crown = p;
        if (p.x > nose.x) nose = p;
        if (p.z - ground < shape.alignAbove || std::abs(p.y) > shape.alignMaxSide) continue;
        points.push_back(p);
        if (std::abs(p.y) > 1e-6f) points.push_back({ p.x, -p.y, p.z });
    }
    Vec3 referenceCrown { 0.0f, 0.0f, -1e9f }, referenceNose { -1e9f, 0.0f, 0.0f };
    for (const Vec3& p : reference.mesh.positions) {
        if (p.z > referenceCrown.z) referenceCrown = p;
        if (p.x > referenceNose.x) referenceNose = p;
    }
    const float heightSpan = referenceCrown.z - referenceNose.z, depthSpan = referenceNose.x - referenceCrown.x;
    if (points.size() < 12 || heightSpan <= 1e-6f || depthSpan <= 1e-6f) {
        error = "molde: poucos pontos para alinhar ou referencia sem alto e nariz";
        return false;
    }
    Similarity initial;
    initial.scale = 0.5f * ((crown.z - nose.z) / heightSpan + (nose.x - crown.x) / depthSpan);
    initial.translation = Vec3 { crown.x, 0.0f, crown.z } - Vec3 { referenceCrown.x, 0.0f, referenceCrown.z } * initial.scale;
    const AlignResult aligned = alignReference(reference.mesh.positions, reference.mesh.triangles, points, initial);
    if (!aligned.ok) {
        error = "molde: o alinhamento nao convergiu";
        return false;
    }
    const Vec3 forward = aligned.transform.rotation.rotate({ 1.0f, 0.0f, 0.0f });
    report.alignmentRms = aligned.rms;
    report.transform = aligned.transform;
    report.scale = aligned.transform.scale;
    report.pitchDegrees = std::atan2(-forward.z, forward.x) * 57.29578f;
    report.alignmentPoints = points.size();
    std::vector<Vec3> placed;
    placed.reserve(reference.mesh.positions.size());
    for (const Vec3& p : reference.mesh.positions) placed.push_back(aligned.transform.apply(p));
    const TriangleBvh surface(placed, reference.mesh.triangles);

    // ---- A projecao: abaixo da linha da mandibula (crescendo em topBlend) e
    // acima da gola (em linha reta da gola, zero, ate a linha de cima do
    // pescoco, inteira), ao longo da normal ----
    const std::vector<Vec3> normals = vertexNormals(mesh);
    const std::vector<Vec3> before = mesh.positions;
    // So os vertices da superficie (os cortes deixam soltos os que sairam, ate
    // compactar).
    std::vector<bool> used(before.size(), false);
    for (const Triangle& t : mesh.triangles)
        for (const std::uint32_t v : t) used[v] = true;
    for (std::size_t v = 0; v < before.size(); ++v) {
        if (!used[v]) continue;
        const Vec3 p = before[v];
        const float h = p.z - ground;
        const float below = smooth01((lineHeight(shape.jawLine, p.x) - h) / std::max(1e-4f, shape.topBlend));
        const float collar = lines.collarHeight(p) + shape.collarClearance, neckTop = ground + lines.neck.top(p.x);
        const float ramp = (neckTop - collar) * std::clamp(shape.neckRamp, 0.05f, 1.0f);
        const float aboveCollar = ramp > 1e-3f ? std::clamp((p.z - collar) / ramp, 0.0f, 1.0f) : 0.0f;
        const float weight = below * aboveCollar;
        if (weight <= 0.0f) continue;
        Vec3 direction = normals[v];
        const bool midline = std::abs(p.y) < 1e-6f;
        if (midline) direction = safeNormalized({ direction.x, 0.0f, direction.z }, { 1.0f, 0.0f, 0.0f });
        const std::optional<Vec3> target = projectOntoSurface(surface, p, direction, shape.maxDistance);
        if (!target) continue;
        Vec3 moved = p + (*target - p) * weight;
        if (midline) moved.y = p.y;
        mesh.positions[v] = moved;
        report.moves.push_back({ p, moved, weight });
        ++report.moved;
        report.largestMove = std::max(report.largestMove, (moved - p).length());
    }
    return true;
}

} // namespace MatterEngine::Modeling
