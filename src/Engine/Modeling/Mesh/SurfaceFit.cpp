#include "Engine/Modeling/Mesh/SurfaceFit.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

namespace MatterEngine::Modeling {

namespace {

using Matrix4 = std::array<std::array<double, 4>, 4>;

// O autovetor do maior autovalor de uma matriz simetrica 4x4 (Jacobi).
std::array<double, 4> largestEigenvector(Matrix4 a) {
    Matrix4 v {};
    for (int i = 0; i < 4; ++i) v[i][i] = 1.0;
    for (int sweep = 0; sweep < 64; ++sweep) {
        double off = 0.0;
        for (int p = 0; p < 4; ++p)
            for (int q = p + 1; q < 4; ++q) off += a[p][q] * a[p][q];
        if (off < 1e-30) break;
        for (int p = 0; p < 4; ++p)
            for (int q = p + 1; q < 4; ++q) {
                if (std::abs(a[p][q]) < 1e-300) continue;
                const double theta = (a[q][q] - a[p][p]) / (2.0 * a[p][q]);
                const double t = (theta >= 0.0 ? 1.0 : -1.0) / (std::abs(theta) + std::sqrt(theta * theta + 1.0));
                const double c = 1.0 / std::sqrt(t * t + 1.0), s = t * c;
                for (int k = 0; k < 4; ++k) {
                    const double akp = a[k][p], akq = a[k][q];
                    a[k][p] = c * akp - s * akq;
                    a[k][q] = s * akp + c * akq;
                }
                for (int k = 0; k < 4; ++k) {
                    const double apk = a[p][k], aqk = a[q][k];
                    a[p][k] = c * apk - s * aqk;
                    a[q][k] = s * apk + c * aqk;
                }
                for (int k = 0; k < 4; ++k) {
                    const double vkp = v[k][p], vkq = v[k][q];
                    v[k][p] = c * vkp - s * vkq;
                    v[k][q] = s * vkp + c * vkq;
                }
            }
    }
    int best = 0;
    for (int i = 1; i < 4; ++i)
        if (a[i][i] > a[best][best]) best = i;
    return { v[0][best], v[1][best], v[2][best], v[3][best] };
}

// So o giro em volta do y (o mesmo angulo que o giro leva o x no plano x-z) e
// a translacao sem y: a semelhanca de duas malhas simetricas.
Similarity symmetricPart(const Similarity& s) {
    const Vec3 forward = s.rotation.rotate({ 1.0f, 0.0f, 0.0f });
    const float pitch = std::atan2(-forward.z, forward.x);
    Similarity out = s;
    out.rotation = Quaternion::fromAxisAngle({ 0.0f, 1.0f, 0.0f }, pitch);
    out.translation.y = 0.0f;
    return out;
}

} // namespace

Similarity Similarity::inverse() const {
    Similarity out;
    out.scale = scale > 1e-12f ? 1.0f / scale : 1.0f;
    out.rotation = rotation.conjugate();
    out.translation = out.rotation.rotate(translation) * -out.scale;
    return out;
}

std::optional<Similarity> fitSimilarity(std::span<const Vec3> source, std::span<const Vec3> target, bool allowScale) {
    const std::size_t n = std::min(source.size(), target.size());
    if (n < 3) return std::nullopt;
    double ca[3] = {}, cb[3] = {};
    for (std::size_t i = 0; i < n; ++i) {
        ca[0] += source[i].x, ca[1] += source[i].y, ca[2] += source[i].z;
        cb[0] += target[i].x, cb[1] += target[i].y, cb[2] += target[i].z;
    }
    for (int k = 0; k < 3; ++k) ca[k] /= static_cast<double>(n), cb[k] /= static_cast<double>(n);
    // S[r][c] = soma de a_r b_c (os dois em volta dos centros).
    double S[3][3] = {}, spread = 0.0;
    for (std::size_t i = 0; i < n; ++i) {
        const double a[3] = { source[i].x - ca[0], source[i].y - ca[1], source[i].z - ca[2] };
        const double b[3] = { target[i].x - cb[0], target[i].y - cb[1], target[i].z - cb[2] };
        for (int r = 0; r < 3; ++r)
            for (int c = 0; c < 3; ++c) S[r][c] += a[r] * b[c];
        spread += a[0] * a[0] + a[1] * a[1] + a[2] * a[2];
    }
    if (spread < 1e-18) return std::nullopt;
    const double Sxx = S[0][0], Sxy = S[0][1], Sxz = S[0][2], Syx = S[1][0], Syy = S[1][1], Syz = S[1][2];
    const double Szx = S[2][0], Szy = S[2][1], Szz = S[2][2];
    const Matrix4 N { { { Sxx + Syy + Szz, Syz - Szy, Szx - Sxz, Sxy - Syx },
        { Syz - Szy, Sxx - Syy - Szz, Sxy + Syx, Szx + Sxz },
        { Szx - Sxz, Sxy + Syx, -Sxx + Syy - Szz, Syz + Szy },
        { Sxy - Syx, Szx + Sxz, Syz + Szy, -Sxx - Syy + Szz } } };
    const std::array<double, 4> q = largestEigenvector(N);
    Similarity out;
    out.rotation = Quaternion { static_cast<float>(q[1]), static_cast<float>(q[2]), static_cast<float>(q[3]), static_cast<float>(q[0]) }
                       .normalized();
    double along = 0.0;
    for (std::size_t i = 0; i < n; ++i) {
        const Vec3 a = out.rotation.rotate({ static_cast<float>(source[i].x - ca[0]), static_cast<float>(source[i].y - ca[1]),
            static_cast<float>(source[i].z - ca[2]) });
        along += a.x * (target[i].x - cb[0]) + a.y * (target[i].y - cb[1]) + a.z * (target[i].z - cb[2]);
    }
    out.scale = allowScale ? static_cast<float>(along / spread) : 1.0f;
    if (!(out.scale > 1e-9f)) return std::nullopt;
    const Vec3 centerA { static_cast<float>(ca[0]), static_cast<float>(ca[1]), static_cast<float>(ca[2]) };
    const Vec3 centerB { static_cast<float>(cb[0]), static_cast<float>(cb[1]), static_cast<float>(cb[2]) };
    out.translation = centerB - out.rotation.rotate(centerA) * out.scale;
    return out;
}

AlignResult alignReference(std::span<const Vec3> referencePositions, std::span<const Triangle> referenceTriangles,
    std::span<const Vec3> points, const Similarity& initial, const AlignSettings& settings) {
    AlignResult result;
    if (points.size() < 3 || referenceTriangles.empty()) return result;
    const TriangleBvh bvh(referencePositions, referenceTriangles);
    // Os nossos pontos vao para a referencia (que fica parada); no fim, o
    // inverso leva a referencia ate eles.
    Similarity toReference = initial.inverse();
    std::vector<Vec3> sources, targets;
    std::vector<std::pair<float, std::size_t>> ranked;
    std::vector<Vec3> nearest(points.size());
    const float far = std::numeric_limits<float>::max();
    float previous = far;
    for (int iteration = 0; iteration < settings.iterations; ++iteration) {
        ranked.clear();
        for (std::size_t i = 0; i < points.size(); ++i) {
            const std::optional<TriangleBvh::Nearest> hit = bvh.nearest(toReference.apply(points[i]), far);
            if (!hit) continue;
            nearest[i] = hit->point;
            ranked.push_back({ hit->distance, i });
        }
        if (ranked.size() < 3) return result;
        std::sort(ranked.begin(), ranked.end());
        const std::size_t keep = std::max<std::size_t>(3, static_cast<std::size_t>(std::ceil(ranked.size() * (1.0f - settings.trim))));
        sources.clear();
        targets.clear();
        double squares = 0.0;
        for (std::size_t k = 0; k < std::min(keep, ranked.size()); ++k) {
            sources.push_back(points[ranked[k].second]);
            targets.push_back(nearest[ranked[k].second]);
            squares += static_cast<double>(ranked[k].first) * ranked[k].first;
        }
        const std::optional<Similarity> step = fitSimilarity(sources, targets, settings.allowScale);
        if (!step) return result;
        toReference = settings.symmetric ? symmetricPart(*step) : *step;
        result.iterations = iteration + 1;
        // A distancia na escala dos pontos (a referencia pode ser de outro tamanho).
        const float rms = static_cast<float>(std::sqrt(squares / static_cast<double>(sources.size()))) / toReference.scale;
        result.rms = rms;
        if (std::abs(previous - rms) < 1e-7f) break;
        previous = rms;
    }
    result.transform = toReference.inverse();
    result.ok = true;
    return result;
}

std::vector<Vec3> vertexNormals(const Mesh& mesh) {
    std::vector<Vec3> normals(mesh.positions.size(), Vec3 {});
    for (const Triangle& t : mesh.triangles) {
        const Vec3 n = cross(mesh.positions[t[1]] - mesh.positions[t[0]], mesh.positions[t[2]] - mesh.positions[t[0]]);
        for (const std::uint32_t v : t) normals[v] += n;
    }
    for (Vec3& n : normals) n = safeNormalized(n, { 0.0f, 0.0f, 1.0f });
    return normals;
}

std::optional<Vec3> projectOntoSurface(const TriangleBvh& surface, Vec3 point, Vec3 direction, float maxDistance) {
    const Vec3 d = safeNormalized(direction, { 0.0f, 0.0f, 1.0f });
    std::optional<Vec3> best;
    float bestDistance = maxDistance;
    for (const float sign : { 1.0f, -1.0f }) {
        const std::optional<TriangleBvh::Hit> hit = surface.raycast(point, d * sign, maxDistance);
        if (hit && hit->distance < bestDistance) {
            bestDistance = hit->distance;
            best = point + d * (sign * hit->distance);
        }
    }
    if (best) return best;
    if (const std::optional<TriangleBvh::Nearest> nearest = surface.nearest(point, maxDistance)) return nearest->point;
    return std::nullopt;
}

} // namespace MatterEngine::Modeling
