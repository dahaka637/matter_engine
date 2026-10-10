#include "Engine/Modeling/Mesh/MeshSkinning.hpp"

#include <algorithm>
#include <cmath>
#include <set>
#include <utility>

namespace MatterEngine::Modeling {

namespace {

// Os ossos que uma regiao pode usar: as juntas dela, o pai de cada raiz e
// as juntas das regioes filhas que nascem dela.
std::vector<bool> allowedJoints(const Skeleton& skeleton, std::span<const int> jointRegion, int region) {
    std::vector<bool> allowed(skeleton.joints.size(), region < 0);
    if (region < 0) return allowed;
    for (std::size_t j = 0; j < skeleton.joints.size(); ++j) {
        const int owner = j < jointRegion.size() ? jointRegion[j] : -1;
        const int parent = skeleton.joints[j].parent;
        const int parentOwner = parent >= 0 && static_cast<std::size_t>(parent) < jointRegion.size()
            ? jointRegion[static_cast<std::size_t>(parent)] : -1;
        if (owner == region) {
            allowed[j] = true;
            if (parent >= 0 && parentOwner != region) allowed[static_cast<std::size_t>(parent)] = true;
        } else if (parentOwner == region) {
            allowed[j] = true;
        }
    }
    return allowed;
}

} // namespace

float weightFor(const SkinWeights& weights, int joint) {
    float total = 0.0f;
    for (std::size_t i = 0; i < 4; ++i)
        if (weights.joints[i] == joint) total += weights.weights[i];
    return total;
}

std::vector<int> dominantJoints(const Mesh& mesh) {
    std::vector<int> result(mesh.positions.size(), -1);
    if (!mesh.hasSkin()) return result;
    for (std::size_t v = 0; v < mesh.positions.size(); ++v) {
        float best = 0.0f;
        for (std::size_t i = 0; i < 4; ++i)
            if (mesh.skin[v].weights[i] > best) {
                best = mesh.skin[v].weights[i];
                result[v] = mesh.skin[v].joints[i];
            }
    }
    return result;
}

void computeAutoSkin(Mesh& mesh, const Skeleton& skeleton, const BodyRegions* regions,
    std::span<const int> jointRegion, const AutoSkinSettings& settings) {
    const std::size_t vertexCount = mesh.positions.size();
    const std::size_t jointCount = skeleton.joints.size();
    mesh.skin.assign(vertexCount, {});
    if (vertexCount == 0 || jointCount == 0) return;

    // A regiao de cada vertice (a da parte do primeiro triangulo que o usa).
    std::vector<int> vertexRegion(vertexCount, -1);
    for (std::size_t t = 0; t < mesh.triangles.size(); ++t) {
        const int part = mesh.partOf(t);
        int region = -1;
        if (regions != nullptr && part >= 0)
            region = regions->find(mesh.parts[static_cast<std::size_t>(part)].region);
        for (const std::uint32_t vertex : mesh.triangles[t])
            if (vertexRegion[vertex] < 0) vertexRegion[vertex] = region;
    }

    // Segmentos de cada junta ate as filhas (a junta sem filhas usa o ponto).
    std::vector<std::vector<std::pair<Vec3, Vec3>>> segments(jointCount);
    for (std::size_t j = 0; j < jointCount; ++j) {
        if (skeleton.joints[j].kind == JointKind::Tip || skeleton.joints[j].kind == JointKind::Helper) continue;
        for (std::size_t child = 0; child < jointCount; ++child)
            if (skeleton.joints[child].parent == static_cast<int>(j))
                segments[j].push_back({ skeleton.joints[j].bind.position, skeleton.joints[child].bind.position });
        if (segments[j].empty())
            segments[j].push_back({ skeleton.joints[j].bind.position, skeleton.joints[j].bind.position });
    }

    std::vector<std::vector<float>> weights(vertexCount, std::vector<float>(jointCount, 0.0f));
    std::vector<std::vector<bool>> allowedCache;
    std::vector<int> cachedRegion;
    for (std::size_t v = 0; v < vertexCount; ++v) {
        const int region = vertexRegion[v];
        std::size_t cacheIndex = 0;
        while (cacheIndex < cachedRegion.size() && cachedRegion[cacheIndex] != region) ++cacheIndex;
        if (cacheIndex == cachedRegion.size()) {
            cachedRegion.push_back(region);
            allowedCache.push_back(allowedJoints(skeleton, jointRegion, regions != nullptr ? region : -1));
        }
        const std::vector<bool>& allowed = allowedCache[cacheIndex];
        float total = 0.0f;
        for (std::size_t j = 0; j < jointCount; ++j) {
            if (!allowed[j] || segments[j].empty()) continue;
            float nearest = 1e9f;
            for (const auto& [a, b] : segments[j])
                nearest = std::min(nearest, pointSegmentDistance(mesh.positions[v], a, b));
            const float weight = 1.0f / (std::pow(std::max(nearest, 1e-4f), settings.falloffPower));
            weights[v][j] = weight;
            total += weight;
        }
        if (total > 0.0f)
            for (float& weight : weights[v]) weight /= total;
    }

    // Suavizacao pelos vizinhos (arestas dos triangulos).
    std::vector<std::set<std::uint32_t>> neighbors(vertexCount);
    for (const Triangle& t : mesh.triangles)
        for (int i = 0; i < 3; ++i) {
            neighbors[t[static_cast<std::size_t>(i)]].insert(t[static_cast<std::size_t>((i + 1) % 3)]);
            neighbors[t[static_cast<std::size_t>((i + 1) % 3)]].insert(t[static_cast<std::size_t>(i)]);
        }
    for (int pass = 0; pass < settings.smoothIterations; ++pass) {
        std::vector<std::vector<float>> next = weights;
        for (std::size_t v = 0; v < vertexCount; ++v) {
            if (neighbors[v].empty()) continue;
            for (std::size_t j = 0; j < jointCount; ++j) {
                float average = 0.0f;
                for (const std::uint32_t n : neighbors[v]) average += weights[n][j];
                average /= static_cast<float>(neighbors[v].size());
                next[v][j] = weights[v][j] + (average - weights[v][j]) * settings.smoothStrength;
            }
        }
        weights = std::move(next);
    }

    // As quatro maiores, renormalizadas.
    for (std::size_t v = 0; v < vertexCount; ++v) {
        std::vector<std::pair<float, std::size_t>> ranked;
        for (std::size_t j = 0; j < jointCount; ++j)
            if (weights[v][j] > 0.0f) ranked.push_back({ weights[v][j], j });
        std::sort(ranked.begin(), ranked.end(), [](const auto& a, const auto& b) { return a.first > b.first; });
        if (ranked.size() > 4) ranked.resize(4);
        float total = 0.0f;
        for (const auto& [weight, joint] : ranked)
            if (weight >= settings.minimumWeight || joint == ranked.front().second) total += weight;
        SkinWeights& skin = mesh.skin[v];
        std::size_t slot = 0;
        for (const auto& [weight, joint] : ranked) {
            if (weight < settings.minimumWeight && joint != ranked.front().second) continue;
            skin.joints[slot] = static_cast<std::uint16_t>(joint);
            skin.weights[slot] = total > 0.0f ? weight / total : 0.0f;
            ++slot;
        }
    }
}

} // namespace MatterEngine::Modeling
