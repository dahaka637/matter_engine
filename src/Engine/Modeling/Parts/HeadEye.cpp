#include "Engine/Modeling/Parts/HeadEye.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

namespace MatterEngine::Modeling {

void buildEye(MeshBuilder& builder, const HeadRecipe& recipe, const EyePlacement& eye) {
    Mesh& mesh = builder.mesh();
    builder.material("olho_branco", recipe.scleraColor);
    builder.material("olho_iris", recipe.irisColor);
    builder.material("olho_cilios", recipe.lashColor);
    builder.material("pele", recipe.skinColor);
    const Vec3 forward = safeNormalized(eye.forward, { 1.0f, 0.0f, 0.0f });
    const Vec3 across = safeNormalized(eye.across - forward * dot(eye.across, forward), { 0.0f, -1.0f, 0.0f });
    const Vec3 up = cross(across, forward);
    const auto weigh = [&](std::uint32_t v) {
        if (mesh.skin.size() < mesh.positions.size()) mesh.skin.resize(mesh.positions.size());
        mesh.skin[v] = {};
        if (eye.headJoint >= 0) {
            mesh.skin[v].joints[0] = static_cast<std::uint16_t>(eye.headJoint);
            mesh.skin[v].weights[0] = 1.0f;
        }
    };
    const auto vertex = [&](Vec3 p) {
        const std::uint32_t v = builder.vertex(p);
        weigh(v);
        return v;
    };
    // A direcao para o centro no plano da abertura.
    const auto center2D = [&](Vec3 p) {
        const Vec3 d = eye.center - p;
        return d - forward * dot(d, forward);
    };
    const std::size_t count = eye.opening.size();
    std::vector<Vec3> rim(count);
    for (std::size_t k = 0; k < count; ++k) rim[k] = mesh.positions[eye.opening[k]];

    // A borda da palpebra: a volta entra (para o meio, no plano da
    // abertura) e recua um pouco.
    std::vector<std::uint32_t> margin, contact, iris;
    std::vector<Vec3> marginPoints;
    for (std::size_t k = 0; k < count; ++k) {
        const Vec3 toCenter = center2D(rim[k]);
        const Vec3 p = rim[k] + safeNormalized(toCenter, {}) * std::min(recipe.lidThickness, 0.45f * toCenter.length())
            - forward * (0.8f * recipe.lidThickness);
        marginPoints.push_back(p);
        margin.push_back(vertex(p));
    }
    // O globo acompanha a borda: o contato com a palpebra logo atras dela (e
    // um fio para dentro, a linha dos cilios vista de frente), a iris num
    // domo raso (mais alto no meio). Visto de frente cada faixa fica dentro
    // da anterior: a palpebra cobre o globo e nada se cruza.
    // Os cilios so na palpebra de cima: embaixo o globo encosta direto na
    // borda (o mesmo vertice).
    const auto depth = [&](Vec3 p) { return dot(p - eye.center, forward); };
    std::vector<bool> lashed(count, false);
    for (std::size_t k = 0; k < count; ++k) {
        const std::size_t next = (k + 1) % count;
        if (dot(marginPoints[k] - eye.center, up) > 0.0f && dot(marginPoints[next] - eye.center, up) > 0.0f)
            lashed[k] = lashed[next] = true;
    }
    float meanDepth = 0.0f;
    for (std::size_t k = 0; k < count; ++k) {
        const Vec3 inward = center2D(marginPoints[k]);
        const Vec3 p = lashed[k]
            ? marginPoints[k] - forward * recipe.eyeGap + safeNormalized(inward, {}) * std::min(0.0003f, 0.3f * inward.length())
            : marginPoints[k];
        contact.push_back(lashed[k] ? vertex(p) : margin[k]);
        meanDepth += depth(p) / static_cast<float>(count);
    }
    const float centerDepth = meanDepth + recipe.eyeBulge;
    // A borda das palpebras no plano do olho (u para o canto de fora, v para
    // cima): a de cima do canto de dentro (4) ao de fora (0), a de baixo do
    // de fora ao de dentro.
    std::vector<Point2> lid(count);
    for (std::size_t k = 0; k < count; ++k)
        lid[k] = { dot(marginPoints[k] - eye.center, across), dot(marginPoints[k] - eye.center, up) };
    const auto lidAt = [&](float u, bool upper) {
        const std::size_t chain[5] = { 4, 5, 6, 7, 0 };
        const std::size_t lower[5] = { 4, 3, 2, 1, 0 };
        const std::size_t* path = upper ? chain : lower;
        if (u <= lid[path[0]].x) return lid[path[0]].y;
        for (int i = 0; i + 1 < 5; ++i) {
            const Point2 a = lid[path[i]], b = lid[path[i + 1]];
            if (u <= b.x) return a.y + (b.y - a.y) * (u - a.x) / std::max(1e-6f, b.x - a.x);
        }
        return lid[path[4]].y;
    };
    // A iris escura (rosto.png): os lados quase retos, encostando nas
    // palpebras em cima e embaixo; um vertice para cada ponto da borda.
    const float radius = recipe.irisRadius;
    for (std::size_t k = 0; k < count; ++k) {
        const Point2 m = lid[k];
        const float side = m.x >= 0.0f ? 1.0f : -1.0f;
        float iu = 0.0f;
        bool upper = m.y > 0.0f, middle = false;
        if (k == 0 || k == 4) {
            iu = side * radius;
            middle = true;
        } else if (k == 2 || k == 6) {
            iu = std::clamp(m.x, -0.35f * radius, 0.35f * radius);
        } else {
            iu = side * 0.82f * radius;
        }
        const float top = lidAt(iu, true), bottom = lidAt(iu, false);
        const float mid = 0.5f * (top + bottom);
        const float iv = middle ? mid : mid + ((upper ? top : bottom) - mid) * 0.94f;
        const float spoke = std::hypot(iu, iv) / std::max(1e-6f, std::hypot(m.x, m.y));
        const float at = centerDepth + (depth(mesh.positions[contact[k]]) - centerDepth) * std::min(1.0f, spoke * spoke);
        iris.push_back(vertex(eye.center + across * iu + up * iv + forward * (at + 0.0002f)));
    }
    const Vec3 pupil = eye.center + forward * (centerDepth + 0.0004f);

    // As faixas: a borda da palpebra (pele), a linha dos cilios (escura em
    // cima), o branco e a iris.
    const std::vector<std::uint32_t> opening(eye.opening.begin(), eye.opening.end());
    builder.useMaterial("pele");
    builder.bridge(opening, margin);
    for (std::size_t k = 0; k < count; ++k) {
        const std::size_t next = (k + 1) % count;
        const bool upper = dot(marginPoints[k] - eye.center, up) > 0.0f && dot(marginPoints[next] - eye.center, up) > 0.0f;
        builder.useMaterial(upper ? "olho_cilios" : "pele");
        if (contact[k] != margin[k] && contact[next] != margin[next]) builder.quad(margin[k], margin[next], contact[next], contact[k]);
        else if (contact[k] != margin[k]) builder.triangle(margin[k], margin[next], contact[k]);
        else if (contact[next] != margin[next]) builder.triangle(margin[k], margin[next], contact[next]);
    }
    builder.useMaterial("olho_branco");
    builder.bridge(contact, iris);
    builder.useMaterial("olho_iris");
    builder.cap(iris, false, Cap::Point, pupil);
    weigh(static_cast<std::uint32_t>(mesh.positions.size() - 1));
    builder.useMaterial("pele");
}

} // namespace MatterEngine::Modeling
