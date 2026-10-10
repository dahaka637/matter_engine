#include "Engine/Modeling/Parts/HeadEar.hpp"

#include <algorithm>
#include <cmath>

namespace MatterEngine::Modeling {

void buildEar(MeshBuilder& builder, const HeadRecipe& recipe, const EarPlacement& ear) {
    Mesh& mesh = builder.mesh();
    const auto& outline = recipe.earOutline;
    const std::size_t count = outline.size();
    if (count < 6 || ear.root.size() != count) return;
    builder.material("pele", recipe.skinColor);
    builder.material("orelha_canal", recipe.canalColor);
    builder.useMaterial("pele");
    const auto vertex = [&](Vec3 p) {
        const std::uint32_t v = builder.vertex(p);
        if (mesh.skin.size() < mesh.positions.size()) mesh.skin.resize(mesh.positions.size());
        mesh.skin[v] = {};
        if (ear.headJoint >= 0) {
            mesh.skin[v].joints[0] = static_cast<std::uint16_t>(ear.headJoint);
            mesh.skin[v].weights[0] = 1.0f;
        }
        return v;
    };
    // Um ponto da orelha: (para a frente, para cima) no plano dela e o quanto
    // se afasta da cabeca.
    const auto at = [&](float u, float v, float height) { return ear.center + ear.forward * u + ear.up * v + ear.out * height; };
    // A borda da concha em volta do centro, na direcao de cada ponto do
    // contorno; os aneis da frente ficam entre ela e o contorno.
    std::vector<Point2> concha(count);
    for (std::size_t k = 0; k < count; ++k) {
        const float angle = std::atan2(outline[k][1], outline[k][0]);
        concha[k] = { recipe.earConchaWidth * std::cos(angle), recipe.earConchaHeight * std::sin(angle) };
    }
    // O relevo (a aba, o sulco e a anti-helice) some no lobulo: embaixo da
    // concha a orelha e uma aba lisa e cheia.
    std::vector<float> relief(count);
    for (std::size_t k = 0; k < count; ++k)
        relief[k] = 0.15f + 0.85f * (1.0f - std::clamp((-outline[k][1] - recipe.earConchaHeight) / 0.008f, 0.0f, 1.0f));
    // Um anel a "fraction" do caminho da concha (0) ao contorno (1), na
    // altura height(afastamento do contorno, relevo).
    const auto ring = [&](float fraction, auto height) {
        std::vector<std::uint32_t> ids;
        for (std::size_t k = 0; k < count; ++k) {
            const float u = concha[k].x + (outline[k][0] - concha[k].x) * fraction;
            const float v = concha[k].y + (outline[k][1] - concha[k].y) * fraction;
            ids.push_back(vertex(at(u, v, height(outline[k][2], relief[k]))));
        }
        return ids;
    };
    const float rim = recipe.earRim;
    std::vector<std::vector<std::uint32_t>> rings;
    rings.push_back(ear.root);
    {
        std::vector<std::uint32_t> base;
        for (const std::uint32_t v : ear.root) base.push_back(vertex(mesh.positions[v] + ear.out * 0.002f));
        rings.push_back(base);
    }
    // As costas (da raiz a borda), a parede da borda, a aba da helice, o
    // sulco, a anti-helice e a borda da concha.
    // (As costas sobem sempre: na frente, onde a orelha quase encosta, a
    // borda de tras nunca desce abaixo da base.)
    rings.push_back(ring(1.0f, [&](float d, float) { return std::max(d - rim, 0.0025f); }));
    rings.push_back(ring(1.0f, [&](float d, float) { return d; }));
    rings.push_back(ring(0.85f, [&](float d, float k) { return d + 0.5f * rim * k; }));
    rings.push_back(ring(0.6f, [&](float d, float k) { return d * (0.92f - 0.22f * k); }));
    rings.push_back(ring(0.35f, [&](float d, float k) { return d * (0.8f - 0.08f * k) + 0.5f * rim * k; }));
    rings.push_back(ring(0.0f, [&](float d, float) { return 0.45f * d + 0.001f; }));
    // O fundo da concha (na sombra) e o canal, quase na pele.
    std::vector<std::uint32_t> floor;
    for (std::size_t k = 0; k < count; ++k) floor.push_back(vertex(at(0.45f * concha[k].x, 0.45f * concha[k].y, 0.0025f)));
    rings.push_back(floor);
    for (std::size_t i = 0; i + 1 < rings.size(); ++i) {
        if (i + 2 == rings.size()) builder.useMaterial("orelha_canal");
        builder.bridge(rings[i], rings[i + 1]);
    }
    builder.cap(floor, false, Cap::Point, at(0.0f, 0.0f, 0.0012f));
    if (mesh.skin.size() < mesh.positions.size()) mesh.skin.resize(mesh.positions.size());
    mesh.skin.back() = {};
    if (ear.headJoint >= 0) {
        mesh.skin.back().joints[0] = static_cast<std::uint16_t>(ear.headJoint);
        mesh.skin.back().weights[0] = 1.0f;
    }
    builder.useMaterial("pele");
}

} // namespace MatterEngine::Modeling
