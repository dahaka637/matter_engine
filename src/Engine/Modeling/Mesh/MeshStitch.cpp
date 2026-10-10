#include "Engine/Modeling/Mesh/MeshStitch.hpp"

#include "Engine/Modeling/Geometry/Triangulation2D.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace MatterEngine::Modeling {

namespace {

// O casamento de menor custo entre duas sequencias (programacao dinamica):
// cada passo avanca em "from" (triangulo from[i], from[i+1], to[j]) ou em
// "to" (from[i], to[j+1], to[j]). F(i) e T(j) dao os vertices.
template <typename FromAt, typename ToAt>
float bestMatch(const Mesh& mesh, std::size_t m, std::size_t n, FromAt fromAt, ToAt toAt, std::vector<char>* steps) {
    const float infinity = std::numeric_limits<float>::max();
    std::vector<float> cost((m + 1) * (n + 1), infinity);
    std::vector<char> choice((m + 1) * (n + 1), 0);
    const auto at = [n](std::size_t i, std::size_t j) { return i * (n + 1) + j; };
    const auto length = [&](std::uint32_t a, std::uint32_t b) { return (mesh.positions[a] - mesh.positions[b]).length(); };
    cost[at(0, 0)] = 0.0f;
    for (std::size_t i = 0; i <= m; ++i)
        for (std::size_t j = 0; j <= n; ++j) {
            const float here = cost[at(i, j)];
            if (here == infinity) continue;
            if (i < m) {
                const float next = here + length(fromAt(i + 1), toAt(j));
                if (next < cost[at(i + 1, j)]) {
                    cost[at(i + 1, j)] = next;
                    choice[at(i + 1, j)] = 'f';
                }
            }
            if (j < n) {
                const float next = here + length(fromAt(i), toAt(j + 1));
                if (next < cost[at(i, j + 1)]) {
                    cost[at(i, j + 1)] = next;
                    choice[at(i, j + 1)] = 't';
                }
            }
        }
    if (steps != nullptr) {
        steps->clear();
        std::size_t i = m, j = n;
        while (i > 0 || j > 0) {
            const char c = choice[at(i, j)];
            steps->push_back(c);
            if (c == 'f') --i;
            else --j;
        }
        std::reverse(steps->begin(), steps->end());
    }
    return cost[at(m, n)];
}

template <typename FromAt, typename ToAt>
void emitSteps(MeshBuilder& builder, const std::vector<char>& steps, FromAt fromAt, ToAt toAt) {
    std::size_t i = 0, j = 0;
    for (const char step : steps) {
        if (step == 'f') {
            builder.triangle(fromAt(i), fromAt(i + 1), toAt(j));
            ++i;
        } else {
            builder.triangle(fromAt(i), toAt(j + 1), toAt(j));
            ++j;
        }
    }
}

} // namespace

void stitchLoops(MeshBuilder& builder, std::span<const std::uint32_t> from, std::span<const std::uint32_t> to) {
    const Mesh& mesh = builder.mesh();
    const std::size_t m = from.size(), n = to.size();
    if (m < 3 || n < 3) return;
    // Comeca pelo vertice de "to" mais perto de from[0] (e os vizinhos dele).
    std::size_t nearest = 0;
    float best = std::numeric_limits<float>::max();
    for (std::size_t j = 0; j < n; ++j) {
        const float d = (mesh.positions[to[j]] - mesh.positions[from[0]]).length();
        if (d < best) {
            best = d;
            nearest = j;
        }
    }
    const auto fromAt = [&](std::size_t i) { return from[i % m]; };
    float bestCost = std::numeric_limits<float>::max();
    std::size_t bestStart = nearest;
    std::vector<char> bestSteps;
    for (const std::size_t start : { (nearest + n - 1) % n, nearest, (nearest + 1) % n }) {
        const auto toAt = [&](std::size_t j) { return to[(start + j) % n]; };
        std::vector<char> steps;
        const float cost = bestMatch(mesh, m, n, fromAt, toAt, &steps) + (mesh.positions[toAt(0)] - mesh.positions[from[0]]).length();
        if (cost < bestCost) {
            bestCost = cost;
            bestStart = start;
            bestSteps = std::move(steps);
        }
    }
    const auto toAt = [&](std::size_t j) { return to[(bestStart + j) % n]; };
    emitSteps(builder, bestSteps, fromAt, toAt);
}

void stitchStrips(MeshBuilder& builder, std::span<const std::uint32_t> from, std::span<const std::uint32_t> to) {
    if (from.size() < 2 || to.size() < 2) return;
    const auto fromAt = [&](std::size_t i) { return from[i]; };
    const auto toAt = [&](std::size_t j) { return to[j]; };
    std::vector<char> steps;
    bestMatch(builder.mesh(), from.size() - 1, to.size() - 1, fromAt, toAt, &steps);
    emitSteps(builder, steps, fromAt, toAt);
}

void stitchStrips(MeshBuilder& builder, std::span<const std::uint32_t> from, std::span<const std::uint32_t> to,
    StripOrder order) {
    if (order == StripOrder::Shortest || from.size() < 2 || to.size() < 2) {
        stitchStrips(builder, from, to);
        return;
    }
    const auto fromAt = [&](std::size_t i) { return from[i]; };
    const auto toAt = [&](std::size_t j) { return to[j]; };
    std::vector<char> steps;
    const char first = order == StripOrder::FromFirst ? 'f' : 't', second = first == 'f' ? 't' : 'f';
    steps.insert(steps.end(), first == 'f' ? from.size() - 1 : to.size() - 1, first);
    steps.insert(steps.end(), second == 'f' ? from.size() - 1 : to.size() - 1, second);
    emitSteps(builder, steps, fromAt, toAt);
}

void stitchCorresponding(MeshBuilder& builder, std::span<const std::uint32_t> loop,
    std::span<const std::uint32_t> inputs, std::span<const std::uint32_t> to, std::span<const std::uint32_t> map,
    bool loopFirst, std::span<const StripOrder> orders) {
    const std::size_t count = inputs.size();
    if (count < 2 || loop.empty() || to.empty()) return;
    // Onde cada ponto dado esta na volta (em ordem; repetidos ficam no mesmo).
    std::vector<std::size_t> at(count, 0);
    std::size_t search = 0;
    for (std::size_t i = 0; i < count; ++i) {
        for (std::size_t step = 0; step < loop.size(); ++step) {
            const std::size_t k = (search + step) % loop.size();
            if (loop[k] == inputs[i]) {
                search = k;
                break;
            }
        }
        at[i] = search;
    }
    const auto rimAt = [&](std::size_t i) { return map.empty() ? i % to.size() : static_cast<std::size_t>(map[i]) % to.size(); };
    for (std::size_t i = 0; i < count; ++i) {
        const std::size_t next = (i + 1) % count;
        std::vector<std::uint32_t> bottom { loop[at[i]] };
        for (std::size_t k = at[i]; k != at[next];) {
            k = (k + 1) % loop.size();
            bottom.push_back(loop[k]);
        }
        std::vector<std::uint32_t> top { to[rimAt(i)] };
        for (std::size_t k = rimAt(i); k != rimAt(next);) {
            k = (k + 1) % to.size();
            top.push_back(to[k]);
        }
        // "first" vem antes no eixo da peca (a faixa vai dele para "second").
        const std::vector<std::uint32_t>& first = loopFirst ? bottom : top;
        const std::vector<std::uint32_t>& second = loopFirst ? top : bottom;
        if (first.size() >= 2 && second.size() >= 2) {
            stitchStrips(builder, first, second, i < orders.size() ? orders[i] : StripOrder::Shortest);
        } else if (first.size() == 1) {
            for (std::size_t j = 0; j + 1 < second.size(); ++j) builder.triangle(first[0], second[j + 1], second[j]);
        } else if (second.size() == 1) {
            for (std::size_t j = 0; j + 1 < first.size(); ++j) builder.triangle(first[j], first[j + 1], second[0]);
        }
    }
}

bool fillLoop(MeshBuilder& builder, std::span<const std::uint32_t> loop) {
    const Mesh& mesh = builder.mesh();
    if (loop.size() < 3) return false;
    // A normal de Newell da volta aponta para dentro da superficie (a volta
    // e a borda de um furo, a superficie a esquerda): a tampa olha ao contrario.
    Vec3 newell {}, center {};
    for (std::size_t i = 0; i < loop.size(); ++i) {
        const Vec3 a = mesh.positions[loop[i]], b = mesh.positions[loop[(i + 1) % loop.size()]];
        newell.x += (a.y - b.y) * (a.z + b.z);
        newell.y += (a.z - b.z) * (a.x + b.x);
        newell.z += (a.x - b.x) * (a.y + b.y);
        center += a;
    }
    center = center / static_cast<float>(loop.size());
    const Vec3 outward = safeNormalized(newell * -1.0f, {});
    if (outward.lengthSquared() < 0.5f) return false;
    const Vec3 axisU = anyPerpendicular(outward);
    const Vec3 axisV = cross(outward, axisU);
    Triangulation2DInput input;
    std::vector<int> outer;
    for (std::size_t i = 0; i < loop.size(); ++i) {
        const Vec3 d = mesh.positions[loop[i]] - center;
        input.points.push_back({ dot(d, axisU), dot(d, axisV) });
        outer.push_back(static_cast<int>(i));
    }
    input.loops.push_back(std::move(outer));
    const Triangulation2DResult result = triangulateDomain(input);
    if (!result.ok) return false;
    for (const auto& tri : result.triangles)
        builder.triangle(loop[static_cast<std::size_t>(tri[0])], loop[static_cast<std::size_t>(tri[1])],
            loop[static_cast<std::size_t>(tri[2])]);
    return true;
}

std::vector<std::uint32_t> reversedLoop(std::span<const std::uint32_t> loop) {
    std::vector<std::uint32_t> reversed;
    reversed.reserve(loop.size());
    if (loop.empty()) return reversed;
    reversed.push_back(loop[0]);
    for (std::size_t i = loop.size() - 1; i > 0; --i) reversed.push_back(loop[i]);
    return reversed;
}

} // namespace MatterEngine::Modeling
