#include "Engine/Modeling/Skin/BoneHeatSkin.hpp"

#include "Engine/Modeling/Geometry/Parallel.hpp"
#include "Engine/Modeling/Mesh/TriangleBvh.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <limits>
#include <map>
#include <optional>

namespace MatterEngine::Modeling {

namespace {

struct Bone {
    int joint = -1;
    Vec3 from;
    Vec3 to;
    // A junta de onde o osso sai e ponta de outro osso (o do pai): o que fica
    // antes dela e desse outro osso.
    bool continues = false;
};

Vec3 closestOnSegment(Vec3 p, Vec3 a, Vec3 b) {
    const Vec3 ab = b - a;
    const float length = dot(ab, ab);
    const float t = length > 1e-12f ? std::clamp(dot(p - a, ab) / length, 0.0f, 1.0f) : 0.0f;
    return a + ab * t;
}

} // namespace

BoneHeatResult boneHeatSkin(Mesh& mesh, const Skeleton& skeleton, std::span<const int> parts, const BoneHeatSettings& settings) {
    BoneHeatResult result;
    // ---- Os ossos: de cada junta (menos pontas e auxiliares) a cada filha ----
    std::vector<Bone> bones;
    for (std::size_t j = 0; j < skeleton.joints.size(); ++j) {
        const Joint& joint = skeleton.joints[j];
        if (joint.kind == JointKind::Tip || joint.kind == JointKind::Helper) continue;
        if (settings.rightHalf && joint.side == Side::Left) continue;
        for (const int child : skeleton.children(static_cast<int>(j)))
            if (skeleton.joints[static_cast<std::size_t>(child)].kind != JointKind::Helper)
                bones.push_back({ static_cast<int>(j), joint.bind.position, skeleton.joints[static_cast<std::size_t>(child)].bind.position,
                    joint.parent >= 0 });
    }
    if (bones.empty()) {
        result.error = "skin: o esqueleto nao tem ossos";
        return result;
    }
    // As juntas que pesam (uma coluna de pesos por junta, nao por osso).
    std::vector<int> jointColumn(skeleton.joints.size(), -1);
    std::vector<int> columnJoint;
    for (const Bone& bone : bones)
        if (jointColumn[static_cast<std::size_t>(bone.joint)] < 0) {
            jointColumn[static_cast<std::size_t>(bone.joint)] = static_cast<int>(columnJoint.size());
            columnJoint.push_back(bone.joint);
        }

    // ---- Os vertices a pesar e os vizinhos (a superficie). O vertice que
    // tambem e de outra parte ja pesada (a borda da chuteira costurada na
    // perna) fica com os pesos dele e segura o calor dos vizinhos ----
    const std::size_t vertexCount = mesh.positions.size();
    std::vector<bool> selected(vertexCount, false), locked(vertexCount, false);
    for (std::size_t t = 0; t < mesh.triangles.size(); ++t)
        if (parts.empty() || std::find(parts.begin(), parts.end(), mesh.partOf(t)) != parts.end())
            for (const std::uint32_t v : mesh.triangles[t]) selected[v] = true;
    if (mesh.hasSkin() && !parts.empty())
        for (std::size_t t = 0; t < mesh.triangles.size(); ++t)
            if (std::find(parts.begin(), parts.end(), mesh.partOf(t)) == parts.end())
                for (const std::uint32_t v : mesh.triangles[t]) {
                    float total = 0.0f;
                    for (const float w : mesh.skin[v].weights) total += w;
                    if (selected[v] && total > 0.0f) locked[v] = true;
                }
    std::vector<std::map<std::uint32_t, float>> neighbors(vertexCount);
    for (const Triangle& triangle : mesh.triangles)
        for (int i = 0; i < 3; ++i) {
            const std::uint32_t a = triangle[static_cast<std::size_t>(i)], b = triangle[static_cast<std::size_t>((i + 1) % 3)];
            if (!selected[a] || !selected[b]) continue;
            const float length = std::max(1e-4f, (mesh.positions[a] - mesh.positions[b]).length());
            neighbors[a][b] = neighbors[b][a] = 1.0f / (length * length);
        }

    // ---- O osso mais perto a vista de cada vertice (o caminho ate ele nao
    // atravessa a malha) ----
    const TriangleBvh bvh(mesh.positions, mesh.triangles);
    const std::size_t columns = columnJoint.size();
    std::vector<float> target(vertexCount * columns, 0.0f);
    std::vector<float> holding(vertexCount, 0.0f);
    std::vector<std::optional<SkinWeights>> requested(vertexCount);
    // (Cada vertice e independente: em paralelo; o pino da receita so le.)
    std::atomic<int> hidden { 0 };
    parallelFor(vertexCount, 64, [&](std::size_t first, std::size_t last) {
        for (std::size_t v = first; v < last; ++v) {
            if (!selected[v]) continue;
            // Preso: o peso que ja tem (a borda de outra parte) ou o pedido.
            if (!locked[v] && settings.pin) requested[v] = settings.pin(static_cast<std::uint32_t>(v));
            if (locked[v] || requested[v]) {
                const SkinWeights& pinned = requested[v] ? *requested[v] : mesh.skin[v];
                for (std::size_t i = 0; i < 4; ++i)
                    if (pinned.weights[i] > 0.0f && pinned.joints[i] < jointColumn.size() && jointColumn[pinned.joints[i]] >= 0)
                        target[v * columns + static_cast<std::size_t>(jointColumn[pinned.joints[i]])] += pinned.weights[i];
                holding[v] = 1e9f;
                continue;
            }
            const Vec3 p = mesh.positions[v];
            // A distancia a cada osso e se ele esta a vista (o caminho ate ele nao
            // atravessa a malha). O que fica antes do comeco de um osso (a cintura
            // acima da junta da coxa, o ombro antes do braco) e do osso do pai,
            // que chega ate a mesma junta: a coxa nao puxa a cintura.
            std::vector<std::pair<float, std::size_t>> visible;
            float best = std::numeric_limits<float>::max();
            int nearest = -1;
            for (std::size_t b = 0; b < bones.size(); ++b) {
                const Vec3 axis = bones[b].to - bones[b].from;
                const float along = dot(p - bones[b].from, axis) / std::max(1e-12f, dot(axis, axis));
                const Vec3 q = closestOnSegment(p, bones[b].from, bones[b].to);
                const float distance = (q - p).length();
                if (distance < best) {
                    best = distance;
                    nearest = static_cast<int>(b);
                }
                if (bones[b].continues && along < -settings.behindStart) continue;
                const Vec3 direction = safeNormalized(q - p, {});
                const std::optional<TriangleBvh::Hit> hit = bvh.raycast(p + direction * 1e-4f, direction, distance);
                if (hit && hit->distance < distance - 2e-4f) continue;
                visible.push_back({ distance, b });
            }
            if (visible.empty()) {
                ++hidden;
                visible.push_back({ best, static_cast<std::size_t>(nearest) });
            }
            std::sort(visible.begin(), visible.end());
            // Os ossos quase a mesma distancia dividem o vertice (o anel em cima
            // de uma junta: metade de cada osso dela; sem isso o primeiro da lista
            // levava o anel inteiro).
            const float closest = visible.front().first;
            const float band = settings.tieBand * closest + 0.0015f;
            float total = 0.0f;
            for (const auto& [distance, b] : visible) {
                const float share = 1.0f - (distance - closest) / band;
                if (share <= 0.0f) break;
                target[v * columns + static_cast<std::size_t>(jointColumn[static_cast<std::size_t>(bones[b].joint)])] += share;
                total += share;
            }
            for (std::size_t c = 0; c < columns; ++c) target[v * columns + c] /= total;
            holding[v] = settings.heat / std::max(1e-6f, closest * closest);
        }
    });
    result.hidden += hidden.load();

    // ---- O calor: cada coluna resolve (L + H) w = H alvo (Gauss-Seidel),
    // uma coluna por vez em cada thread ----
    std::vector<float> weight(vertexCount * columns, 0.0f);
    parallelFor(columns, 1, [&](std::size_t firstColumn, std::size_t lastColumn) {
        for (std::size_t c = firstColumn; c < lastColumn; ++c) {
            std::vector<float> w(vertexCount, 0.0f);
            for (std::size_t v = 0; v < vertexCount; ++v) w[v] = target[v * columns + c];
            for (int iteration = 0; iteration < settings.iterations; ++iteration)
                for (std::size_t v = 0; v < vertexCount; ++v) {
                    if (!selected[v]) continue;
                    float sum = holding[v] * target[v * columns + c], total = holding[v];
                    for (const auto& [u, k] : neighbors[v]) {
                        sum += k * w[u];
                        total += k;
                    }
                    if (total > 0.0f) w[v] = sum / total;
                }
            for (std::size_t v = 0; v < vertexCount; ++v) weight[v * columns + c] = w[v];
        }
    });

    // ---- As quatro maiores, sem as pequenas, somando 1 ----
    if (mesh.skin.size() < vertexCount) mesh.skin.resize(vertexCount);
    for (std::size_t v = 0; v < vertexCount; ++v) {
        if (!selected[v] || locked[v]) continue;
        if (requested[v]) {
            mesh.skin[v] = *requested[v];
            continue;
        }
        std::vector<std::pair<float, int>> ranked;
        for (std::size_t c = 0; c < columns; ++c)
            if (weight[v * columns + c] > 0.0f) ranked.push_back({ weight[v * columns + c], columnJoint[c] });
        std::sort(ranked.begin(), ranked.end(), [](const auto& a, const auto& b) { return a.first > b.first; });
        if (ranked.size() > 4) ranked.resize(4);
        float total = 0.0f;
        for (const auto& [w, joint] : ranked) total += w;
        SkinWeights& skin = mesh.skin[v];
        skin = {};
        std::size_t slot = 0;
        for (const auto& [w, joint] : ranked) {
            if (total <= 0.0f || w / total < settings.minimumWeight) continue;
            skin.joints[slot] = static_cast<std::uint16_t>(joint);
            skin.weights[slot] = w;
            ++slot;
        }
        float kept = 0.0f;
        for (std::size_t i = 0; i < slot; ++i) kept += skin.weights[i];
        if (kept <= 0.0f && !ranked.empty()) {
            skin.joints[0] = static_cast<std::uint16_t>(ranked.front().second);
            skin.weights[0] = 1.0f;
            continue;
        }
        for (std::size_t i = 0; i < slot; ++i) skin.weights[i] /= kept;
    }
    result.ok = true;
    return result;
}

} // namespace MatterEngine::Modeling
