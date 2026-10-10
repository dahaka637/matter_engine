#include "Engine/Modeling/Skin/SkinSmoothing.hpp"

#include <algorithm>
#include <limits>
#include <map>
#include <set>

namespace MatterEngine::Modeling {

namespace {

// As quatro maiores, somando 1.
SkinWeights strongest(const std::map<std::uint16_t, float>& weights, const SkinWeights& fallback) {
    std::vector<std::pair<float, std::uint16_t>> ranked;
    for (const auto& [joint, weight] : weights)
        if (weight > 0.0f) ranked.push_back({ weight, joint });
    std::sort(ranked.begin(), ranked.end(), [](const auto& a, const auto& b) { return a.first > b.first; });
    if (ranked.size() > 4) ranked.resize(4);
    float total = 0.0f;
    for (const auto& [weight, joint] : ranked) total += weight;
    if (total <= 0.0f) return fallback;
    SkinWeights skin {};
    for (std::size_t i = 0; i < ranked.size(); ++i) {
        skin.joints[i] = ranked[i].second;
        skin.weights[i] = ranked[i].first / total;
    }
    return skin;
}

} // namespace

void smoothSkinWeights(Mesh& mesh, int iterations, float amount) {
    smoothSkinWeights(mesh, iterations, amount, {});
}

void blendMidlineWeights(Mesh& mesh, std::span<const int> jointMirror, float distance) {
    if (!mesh.hasSkin() || distance <= 0.0f) return;
    for (std::size_t v = 0; v < mesh.positions.size(); ++v) {
        const float y = std::abs(mesh.positions[v].y);
        if (y >= distance) continue;
        // No meio metade de cada lado, saindo macio ate a distancia.
        const float t = y / distance;
        const float share = 0.5f * (1.0f - t * t * (3.0f - 2.0f * t));
        std::map<std::uint16_t, float> mixed;
        const SkinWeights& weights = mesh.skin[v];
        for (std::size_t i = 0; i < 4; ++i) {
            if (weights.weights[i] <= 0.0f) continue;
            const std::uint16_t joint = weights.joints[i];
            const std::uint16_t other = joint < jointMirror.size() && jointMirror[joint] >= 0
                ? static_cast<std::uint16_t>(jointMirror[joint]) : joint;
            mixed[joint] += (1.0f - share) * weights.weights[i];
            mixed[other] += share * weights.weights[i];
        }
        mesh.skin[v] = strongest(mixed, weights);
    }
}

void smoothSkinWeights(Mesh& mesh, int iterations, float amount, const std::function<bool(std::uint32_t)>& moves) {
    if (!mesh.hasSkin() || iterations <= 0) return;
    const std::size_t count = mesh.positions.size();
    std::vector<std::set<std::uint32_t>> neighbors(count);
    for (const Triangle& triangle : mesh.triangles)
        for (int i = 0; i < 3; ++i) {
            const std::uint32_t a = triangle[static_cast<std::size_t>(i)], b = triangle[static_cast<std::size_t>((i + 1) % 3)];
            neighbors[a].insert(b);
            neighbors[b].insert(a);
        }
    std::vector<std::map<std::uint16_t, float>> weights(count);
    for (std::size_t v = 0; v < count; ++v)
        for (std::size_t i = 0; i < 4; ++i)
            if (mesh.skin[v].weights[i] > 0.0f) weights[v][mesh.skin[v].joints[i]] += mesh.skin[v].weights[i];
    for (int iteration = 0; iteration < iterations; ++iteration) {
        std::vector<std::map<std::uint16_t, float>> next = weights;
        for (std::size_t v = 0; v < count; ++v) {
            if (neighbors[v].empty() || (moves && !moves(static_cast<std::uint32_t>(v)))) continue;
            std::map<std::uint16_t, float> average;
            for (const std::uint32_t u : neighbors[v])
                for (const auto& [joint, weight] : weights[u]) average[joint] += weight / static_cast<float>(neighbors[v].size());
            std::map<std::uint16_t, float>& mixed = next[v];
            for (auto& [joint, weight] : mixed) weight *= 1.0f - amount;
            for (const auto& [joint, weight] : average) mixed[joint] += weight * amount;
        }
        weights = std::move(next);
    }
    for (std::size_t v = 0; v < count; ++v)
        if (!moves || moves(static_cast<std::uint32_t>(v))) mesh.skin[v] = strongest(weights[v], mesh.skin[v]);
}

void pinWeights(Mesh& mesh, std::span<const Vec3> anchors, float radius, int joint, float amount) {
    if (!mesh.hasSkin() || joint < 0 || radius <= 0.0f || anchors.empty()) return;
    for (std::size_t v = 0; v < mesh.positions.size(); ++v) {
        float nearest = std::numeric_limits<float>::max();
        for (const Vec3& anchor : anchors) nearest = std::min(nearest, (mesh.positions[v] - anchor).length());
        const float d = nearest / radius;
        if (d >= 1.0f) continue;
        const float k = amount * (1.0f - d * d * (3.0f - 2.0f * d));
        std::map<std::uint16_t, float> weights;
        float moved = 0.0f;
        for (std::size_t i = 0; i < 4; ++i) {
            const float w = mesh.skin[v].weights[i];
            if (w <= 0.0f) continue;
            if (mesh.skin[v].joints[i] == joint) {
                weights[mesh.skin[v].joints[i]] += w;
                continue;
            }
            weights[mesh.skin[v].joints[i]] += w * (1.0f - k);
            moved += w * k;
        }
        weights[static_cast<std::uint16_t>(joint)] += moved;
        mesh.skin[v] = strongest(weights, mesh.skin[v]);
    }
}

void moveWeights(Mesh& mesh, std::uint32_t vertex, std::span<const int> from, int to, float fraction) {
    if (!mesh.hasSkin() || vertex >= mesh.skin.size() || to < 0 || fraction <= 0.0f) return;
    std::map<std::uint16_t, float> weights;
    float moved = 0.0f;
    for (std::size_t i = 0; i < 4; ++i) {
        const float w = mesh.skin[vertex].weights[i];
        if (w <= 0.0f) continue;
        const std::uint16_t joint = mesh.skin[vertex].joints[i];
        if (std::find(from.begin(), from.end(), static_cast<int>(joint)) != from.end()) {
            weights[joint] += w * (1.0f - fraction);
            moved += w * fraction;
        } else {
            weights[joint] += w;
        }
    }
    if (moved <= 0.0f) return;
    weights[static_cast<std::uint16_t>(to)] += moved;
    mesh.skin[vertex] = strongest(weights, mesh.skin[vertex]);
}

void chainWeights(Mesh& mesh, std::uint32_t vertex, std::span<const int> chain, float t, float amount, float straight) {
    if (!mesh.hasSkin() || vertex >= mesh.skin.size() || chain.size() < 2 || amount <= 0.0f) return;
    for (const int joint : chain)
        if (joint < 0) return;
    amount = std::min(amount, 1.0f);
    std::map<std::uint16_t, float> weights;
    float share = 0.0f;
    for (std::size_t i = 0; i < 4; ++i) {
        const float w = mesh.skin[vertex].weights[i];
        if (w <= 0.0f) continue;
        const std::uint16_t joint = mesh.skin[vertex].joints[i];
        if (std::find(chain.begin(), chain.end(), static_cast<int>(joint)) != chain.end()) {
            share += w;
            // O que ele tinha na cadeia fica na parte que nao muda.
            weights[joint] += w * (1.0f - amount);
        } else {
            weights[joint] += w;
        }
    }
    if (share <= 0.0f) return;
    // O ponto na cadeia: entre a junta k e a k+1 (os aneis); o reto, nas
    // pontas.
    t = std::clamp(t, 0.0f, 1.0f);
    straight = std::clamp(straight, 0.0f, 1.0f);
    const float at = t * static_cast<float>(chain.size() - 1);
    const std::size_t k = std::min(chain.size() - 2, static_cast<std::size_t>(at));
    const float u = at - static_cast<float>(k);
    const float rings = amount * share * (1.0f - straight), direct = amount * share * straight;
    weights[static_cast<std::uint16_t>(chain[k])] += rings * (1.0f - u);
    weights[static_cast<std::uint16_t>(chain[k + 1])] += rings * u;
    weights[static_cast<std::uint16_t>(chain.front())] += direct * (1.0f - t);
    weights[static_cast<std::uint16_t>(chain.back())] += direct * t;
    mesh.skin[vertex] = strongest(weights, mesh.skin[vertex]);
}

void relaxWeights(Mesh& mesh, std::span<const WeightRelaxPoint> points, int iterations) {
    if (points.empty() || iterations <= 0) return;
    // A parte de cada vertice: -1 se ele esta em triangulos de mais de uma.
    std::vector<int> only(mesh.positions.size(), -2);
    for (std::size_t t = 0; t < mesh.triangles.size(); ++t)
        for (const std::uint32_t v : mesh.triangles[t]) {
            const int part = mesh.partOf(t);
            only[v] = only[v] == -2 || only[v] == part ? part : -1;
        }
    std::vector<int> parts;
    for (const WeightRelaxPoint& point : points) parts.push_back(point.part.empty() ? -2 : mesh.findPart(point.part));
    smoothSkinWeights(mesh, iterations, 0.5f, [&](std::uint32_t v) {
        for (std::size_t i = 0; i < points.size(); ++i)
            if ((mesh.positions[v] - points[i].at).length() < points[i].radius && (parts[i] == -2 || only[v] == parts[i])) return true;
        return false;
    });
}

} // namespace MatterEngine::Modeling
