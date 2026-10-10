#include "Engine/Modeling/Parts/HeadBrow.hpp"

#include <algorithm>
#include <cmath>

namespace MatterEngine::Modeling {

void buildBrow(MeshBuilder& builder, const HeadRecipe& recipe, const BrowPlacement& brow) {
    Mesh& mesh = builder.mesh();
    const std::size_t count = brow.footprint.size();
    const std::size_t stations = brow.stations;
    if (stations < 2 || count != 1 + 2 * stations || brow.normals.size() != count) return;
    builder.material("sobrancelha", recipe.browColor);
    builder.useMaterial("sobrancelha");
    const auto vertex = [&](Vec3 p) {
        const std::uint32_t v = builder.vertex(p);
        if (mesh.skin.size() < mesh.positions.size()) mesh.skin.resize(mesh.positions.size());
        mesh.skin[v] = {};
        if (brow.headJoint >= 0) {
            mesh.skin[v].joints[0] = static_cast<std::uint16_t>(brow.headJoint);
            mesh.skin[v].weights[0] = 1.0f;
        }
        return v;
    };
    std::vector<Vec3> foot(count);
    for (std::size_t i = 0; i < count; ++i) foot[i] = mesh.positions[brow.footprint[i]];
    const auto bottom = [&](std::size_t k) { return 1 + k; };
    const auto top = [&](std::size_t k) { return count - 1 - k; };

    // A borda do topo: cada ponto da pegada sobe um pouco e entra para o meio
    // da sobrancelha (o chanfro); a ponta entra para a primeira estacao.
    std::vector<std::uint32_t> edge(count);
    for (std::size_t i = 0; i < count; ++i) {
        Vec3 inward;
        if (i == 0) {
            inward = (foot[bottom(0)] + foot[top(0)]) * 0.5f - foot[0];
        } else {
            const std::size_t k = i <= stations ? i - 1 : count - 1 - i;
            inward = i <= stations ? foot[top(k)] - foot[i] : foot[bottom(k)] - foot[i];
        }
        const Vec3 n = safeNormalized(brow.normals[i], { 1.0f, 0.0f, 0.0f });
        inward = safeNormalized(inward - n * dot(inward, n), {});
        edge[i] = vertex(foot[i] + n * recipe.browEdge + inward * (0.6f * recipe.browEdge));
    }
    // A quina ao longo da sobrancelha (pouco abaixo do meio): o mais alto do
    // bloco; a face de cima sobe ate ela e a da frente desce dela.
    std::vector<std::uint32_t> ridge(stations);
    for (std::size_t k = 0; k < stations; ++k) {
        const Vec3 n = safeNormalized(brow.normals[bottom(k)] + brow.normals[top(k)], { 1.0f, 0.0f, 0.0f });
        ridge[k] = vertex(foot[bottom(k)] + (foot[top(k)] - foot[bottom(k)]) * 0.45f + n * recipe.browThickness);
    }

    // A parede em volta (da pegada a borda do topo), as faces da frente e de
    // cima e o fim de dentro - todas anti-horarias vistas de fora.
    builder.bridge(brow.footprint, edge);
    builder.triangle(edge[0], edge[bottom(0)], ridge[0]);
    builder.triangle(edge[0], ridge[0], edge[top(0)]);
    for (std::size_t k = 0; k + 1 < stations; ++k) {
        builder.quad(edge[bottom(k)], edge[bottom(k + 1)], ridge[k + 1], ridge[k]);
        builder.quad(ridge[k], ridge[k + 1], edge[top(k + 1)], edge[top(k)]);
    }
    builder.triangle(ridge[stations - 1], edge[bottom(stations - 1)], edge[top(stations - 1)]);
    builder.useMaterial("pele");
}

} // namespace MatterEngine::Modeling
