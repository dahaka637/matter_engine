#include "Engine/Modeling/Parts/NeckGenerator.hpp"

#include "Engine/Modeling/Mesh/MeshBuilder.hpp"
#include "Engine/Modeling/Parts/FacetNoise.hpp"
#include "Engine/Modeling/Parts/SuperSection.hpp"

#include <algorithm>
#include <cmath>

namespace MatterEngine::Modeling {

namespace {

float smooth(float edge0, float edge1, float x) {
    const float t = std::clamp((x - edge0) / (edge1 - edge0), 0.0f, 1.0f);
    return t * t * (3.0f - 2.0f * t);
}

} // namespace

NeckSeams buildNeckHalf(Mesh& mesh, const NeckRecipe& recipe, const NeckPlacement& placement) {
    NeckSeams seams;
    seams.part = placement.part;
    if (placement.collar.size() < 3) {
        seams.error = "pescoco: falta a volta de dentro da gola";
        return seams;
    }
    MeshBuilder builder(mesh);
    builder.material("pele", recipe.skinColor);
    const SectionCurves curves { recipe.centerX, recipe.front, recipe.back, recipe.side, recipe.exponent };
    const float ground = placement.ground;
    // Um ponto levado ate a pele na altura dele (pela direcao em volta do
    // centro da secao); os do meio ficam no meio.
    const auto onSkin = [&](Vec3 p, bool middle) {
        const SuperSection section = curves.at(p.z - ground, ground);
        float angle = std::atan2(p.y, p.x - section.centerX);
        if (middle) angle = p.x >= section.centerX ? 0.0f : Pi;
        Vec3 q = section.point(angle);
        q.z = p.z;
        if (middle) q.y = 0.0f;
        return q;
    };
    // A abertura da cabeca: os mesmos angulos da gola, no plano inclinado.
    const std::size_t count = placement.collar.size();
    std::vector<Vec3> bottom, top;
    for (std::size_t k = 0; k < count; ++k) {
        const Vec3 p = mesh.positions[placement.collar[k]];
        bottom.push_back(p);
        const bool middle = k == 0 || k + 1 == count;
        const SuperSection probe = curves.at(p.z - ground, ground);
        float angle = std::atan2(p.y, p.x - probe.centerX);
        if (middle) angle = k == 0 ? 0.0f : -Pi;
        const float backness = 0.5f - 0.5f * std::cos(angle);
        const float height = recipe.headFrontHeight + (recipe.headBackHeight - recipe.headFrontHeight) * backness;
        Vec3 q = curves.at(height, ground).point(angle);
        if (middle) q.y = 0.0f;
        top.push_back(q);
    }
    std::vector<std::vector<std::uint32_t>> rings { placement.collar };
    for (int j = 1; j <= recipe.rings + 1; ++j) {
        const float t = static_cast<float>(j) / static_cast<float>(recipe.rings + 1);
        const bool last = j == recipe.rings + 1;
        std::vector<std::uint32_t> ring;
        for (std::size_t k = 0; k < count; ++k) {
            const bool middle = k == 0 || k + 1 == count;
            Vec3 p = last ? top[k] : onSkin(lerp(bottom[k], top[k], t), middle);
            if (!last) {
                const SuperSection section = curves.at(p.z - ground, ground);
                const Vec3 out = safeNormalized({ p.x - section.centerX, p.y, 0.0f }, { 1.0f, 0.0f, 0.0f });
                p = p + out * (recipe.jitter * facetNoise(recipe.seed, j, static_cast<int>(k), 0));
                if (middle) p.y = 0.0f;
            }
            ring.push_back(builder.vertex(p));
        }
        rings.push_back(std::move(ring));
    }
    // A volta anda da frente para tras pelo lado direito (horaria vista de
    // cima): os quads ao contrario dos aneis do tronco.
    builder.part(placement.part, placement.region).connectTo(placement.collarPart);
    builder.useMaterial("pele");
    for (std::size_t r = 0; r + 1 < rings.size(); ++r)
        for (std::size_t k = 0; k + 1 < count; ++k)
            builder.quad(rings[r][k + 1], rings[r][k], rings[r + 1][k], rings[r + 1][k + 1]);
    if (placement.headOpening) builder.opening("cabeca");
    seams.top = rings.back();

    // Pesos pela volta (0 na gola, 1 na abertura da cabeca), nao pela
    // altura: a gola e da camisa (o alto do peito), a pele passa para o
    // pescoco subindo dela (no V o peito, atras logo acima da gola) e a
    // cabeca puxa a abertura de cima.
    if (mesh.skin.size() < mesh.positions.size()) mesh.skin.resize(mesh.positions.size());
    for (std::size_t r = 1; r < rings.size(); ++r) {
        const float t = static_cast<float>(r) / static_cast<float>(rings.size() - 1);
        const float neck = placement.neckJoint >= 0 ? smooth(0.0f, 0.8f, t) : 0.0f;
        const float head = placement.headJoint >= 0 ? 0.35f * smooth(0.6f, 1.0f, t) : 0.0f;
        for (const std::uint32_t v : rings[r]) {
            SkinWeights& weights = mesh.skin[v];
            weights = {};
            std::size_t slot = 0;
            const auto add = [&](int joint, float weight) {
                if (joint < 0 || weight <= 1e-4f || slot >= 4) return;
                weights.joints[slot] = static_cast<std::uint16_t>(joint);
                weights.weights[slot] = weight;
                ++slot;
            };
            add(placement.upperChestJoint, (1.0f - neck) * (1.0f - head));
            add(placement.neckJoint, neck * (1.0f - head));
            add(placement.headJoint, head);
            float total = 0.0f;
            for (const float w : weights.weights) total += w;
            if (total > 0.0f)
                for (float& w : weights.weights) w /= total;
        }
    }
    seams.ok = true;
    return seams;
}

} // namespace MatterEngine::Modeling
