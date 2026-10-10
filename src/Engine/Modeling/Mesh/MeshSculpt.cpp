#include "Engine/Modeling/Mesh/MeshSculpt.hpp"

#include <algorithm>
#include <cmath>

namespace MatterEngine::Modeling {

void sculptMesh(Mesh& mesh, std::span<const SculptPull> pulls, const std::function<bool(std::uint32_t)>& affects, bool midline) {
    if (pulls.empty()) return;
    // Os puxoes leem as posicoes de antes (a ordem deles nao importa).
    const std::vector<Vec3> before = mesh.positions;
    for (std::size_t v = 0; v < mesh.positions.size(); ++v) {
        if (affects && !affects(static_cast<std::uint32_t>(v))) continue;
        Vec3 offset {};
        for (const SculptPull& pull : pulls) {
            const float d = (before[v] - pull.at).length();
            if (d >= pull.radius) continue;
            const float t = std::clamp((d - 0.4f * pull.radius) / (0.6f * pull.radius), 0.0f, 1.0f);
            offset += pull.move * (1.0f - t * t * (3.0f - 2.0f * t));
        }
        if (midline && std::abs(before[v].y) < 1e-6f) offset.y = 0.0f;
        mesh.positions[v] = before[v] + offset;
    }
}

bool moveVertices(Mesh& mesh, std::span<const VertexMove> moves, float tolerance, std::string& error,
    const std::function<bool(std::uint32_t)>& affects) {
    const std::vector<Vec3> before = mesh.positions;
    for (const VertexMove& move : moves) {
        std::size_t nearest = before.size();
        float best = tolerance;
        for (std::size_t v = 0; v < before.size(); ++v) {
            if (affects && !affects(static_cast<std::uint32_t>(v))) continue;
            const float d = (before[v] - move.at).length();
            if (d < best) {
                best = d;
                nearest = v;
            }
        }
        if (nearest == before.size()) {
            error = "nenhum vertice perto de (" + std::to_string(move.at.x) + ", " + std::to_string(move.at.y) + ", "
                + std::to_string(move.at.z) + ")";
            return false;
        }
        Vec3 target = move.to;
        if (std::abs(before[nearest].y) < 1e-6f) target.y = before[nearest].y;
        mesh.positions[nearest] = target;
    }
    return true;
}

} // namespace MatterEngine::Modeling
