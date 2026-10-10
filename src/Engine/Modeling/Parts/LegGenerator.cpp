#include "Engine/Modeling/Parts/LegGenerator.hpp"

#include "Engine/Modeling/Mesh/MeshBuilder.hpp"
#include "Engine/Modeling/Mesh/MeshStitch.hpp"
#include "Engine/Modeling/Parts/FacetNoise.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>

namespace MatterEngine::Modeling {

namespace {

float smooth(float edge0, float edge1, float x) {
    const float t = std::clamp((x - edge0) / (edge1 - edge0), 0.0f, 1.0f);
    return t * t * (3.0f - 2.0f * t);
}

// O ponto da volta (fechada) na direcao "angle" a partir do eixo (vista de
// cima): onde o raio do eixo cruza a volta, com a altura no lugar.
std::optional<Vec3> loopPointAt(const Mesh& mesh, const std::vector<std::uint32_t>& loop, Point2 axis, float angle) {
    const float dx = std::cos(angle), dy = std::sin(angle);
    std::optional<Vec3> best;
    float bestDistance = std::numeric_limits<float>::max();
    for (std::size_t i = 0; i < loop.size(); ++i) {
        const Vec3 a = mesh.positions[loop[i]], b = mesh.positions[loop[(i + 1) % loop.size()]];
        // Raio p = axis + s * d; aresta q = a + u * (b - a).
        const float ex = b.x - a.x, ey = b.y - a.y;
        const float denominator = dx * ey - dy * ex;
        if (std::abs(denominator) < 1e-12f) continue;
        const float wx = a.x - axis.x, wy = a.y - axis.y;
        const float s = (wx * ey - wy * ex) / denominator;
        const float u = (wx * dy - wy * dx) / denominator;
        if (s <= 0.0f || u < 0.0f || u > 1.0f) continue;
        if (s < bestDistance) {
            bestDistance = s;
            best = lerp(a, b, u);
        }
    }
    return best;
}

} // namespace

LegSeams buildLeg(Mesh& mesh, const LegRecipe& recipe, const LegPlacement& placement) {
    LegSeams seams;
    MeshBuilder builder(mesh);
    const std::size_t firstVertex = mesh.positions.size();
    const int points = recipe.ringPoints;
    const float step = 2.0f * Pi / static_cast<float>(points);
    builder.material("meiao", recipe.sockColor);
    builder.material("meiao_faixa", recipe.bandColor);
    builder.material("pele", recipe.skinColor);
    seams.sockPart = "meiao_" + placement.side;
    seams.skinPart = "perna_" + placement.side;

    // O joelho em volta da secao: a patela e a tuberosidade saltam na frente
    // e o oco atras afunda (o quanto o ponto anda para fora do eixo).
    const auto kneeRelief = [&](float h, float angle) {
        const float dz = h - placement.kneeHeight;
        const auto bump = [](float x) { return x >= 1.0f ? 0.0f : (1.0f - x * x) * (1.0f - x * x); };
        float a = std::remainder(angle, 2.0f * Pi);
        const float front = bump(std::abs(a) / recipe.patellaHalfAngle);
        float relief = recipe.patellaProtrusion * front * bump(std::abs(dz - recipe.patellaCenter) / recipe.patellaHalfHeight);
        relief += recipe.tuberosityProtrusion * bump(std::abs(a) / (recipe.patellaHalfAngle * 0.5f))
            * bump(std::abs(dz - recipe.tuberosityCenter) / recipe.tuberosityHalfHeight);
        const float behind = bump(std::abs(std::remainder(a - Pi, 2.0f * Pi)) / 0.9f);
        relief -= recipe.poplitealDepth * behind * bump(std::abs(dz - 0.01f) / 0.03f);
        return relief;
    };
    // A secao em h na direcao angle (0 = frente, pi/2 = +y): a superelipse
    // de quatro quadrantes em volta do eixo, com o relevo do joelho.
    const auto sectionPoint = [&](float h, float angle) {
        const float c = std::cos(angle), s = std::sin(angle);
        const float e = 2.0f / recipe.exponent(h);
        const float along = c >= 0.0f ? recipe.front(h) : recipe.back(h);
        const float across = s * placement.medialSign >= 0.0f ? recipe.medial(h) : recipe.lateral(h);
        const float x = (c >= 0.0f ? 1.0f : -1.0f) * along * std::pow(std::abs(c), e);
        const float y = (s >= 0.0f ? 1.0f : -1.0f) * across * std::pow(std::abs(s), e);
        const Vec3 out = safeNormalized({ x, y, 0.0f }, { 1.0f, 0.0f, 0.0f });
        return Vec3 { placement.axis.x + x, placement.axis.y + y, placement.ground + h } + out * kneeRelief(h, angle);
    };
    // O facetado: o vertice anda para dentro/fora do eixo (a altura fica:
    // as bordas da faixa continuam retas).
    const auto jittered = [&](Vec3 p, int ring, int k) {
        const Vec3 out = safeNormalized({ p.x - placement.axis.x, p.y - placement.axis.y, 0.0f }, {});
        return p + out * (recipe.jitter * facetNoise(recipe.seed, ring, k, 0));
    };

    // Os aneis de saida da gola: a forma da borda da gola (na direcao de
    // cada ponto do anel) levantada collarLift, virando a do primeiro anel.
    const float firstHeight = recipe.rings.front();
    std::vector<Vec3> collarShape, firstShape;
    for (int k = 0; k < points; ++k) {
        const float angle = step * static_cast<float>(k);
        firstShape.push_back(sectionPoint(firstHeight, angle));
        const std::optional<Vec3> rim = loopPointAt(mesh, placement.collar, placement.axis, angle);
        if (!rim) {
            seams.error = "perna " + placement.side + ": a gola nao envolve o eixo da perna";
            return seams;
        }
        collarShape.push_back(*rim + Vec3 { 0.0f, 0.0f, recipe.collarLift });
    }
    std::vector<std::vector<std::uint32_t>> rings;
    std::vector<float> ringHeights;
    for (int j = 0; j < recipe.collarRings; ++j) {
        const float f = static_cast<float>(j) / static_cast<float>(recipe.collarRings);
        std::vector<std::uint32_t> ring;
        float height = 0.0f;
        for (int k = 0; k < points; ++k) {
            const Vec3 p = lerp(collarShape[static_cast<std::size_t>(k)], firstShape[static_cast<std::size_t>(k)], f);
            height += p.z;
            ring.push_back(builder.vertex(j == 0 ? p : jittered(p, -1 - j, k)));
        }
        rings.push_back(std::move(ring));
        ringHeights.push_back(height / static_cast<float>(points) - placement.ground);
    }
    // Os aneis da perna; os alternados desencontrados meio passo (triangulos
    // quase equilateros, o facetado da referencia). O ultimo fica liso (a
    // costura da barra).
    for (std::size_t i = 0; i < recipe.rings.size(); ++i) {
        const float h = recipe.rings[i];
        const float phase = i % 2 == 1 ? 0.5f : 0.0f;
        const bool last = i + 1 == recipe.rings.size();
        // Os vertices sobem/descem um pouco (o facetado irregular), menos nas
        // bordas da faixa (ficam retas) e nunca alem de um quarto do passo.
        const bool edge = std::abs(h - recipe.sockTop) < 1e-4f || std::abs(h - recipe.bandTop) < 1e-4f;
        const float below = i > 0 ? h - recipe.rings[i - 1] : 0.0f, above = last ? 0.0f : recipe.rings[i + 1] - h;
        const float lift = edge || i == 0 || last ? 0.0f : std::min(recipe.verticalJitter, 0.25f * std::min(below, above));
        std::vector<std::uint32_t> ring;
        for (int k = 0; k < points; ++k) {
            Vec3 p = sectionPoint(h, step * (static_cast<float>(k) + phase));
            if (!last) p = jittered(p, static_cast<int>(i), k);
            p.z += lift * facetNoise(recipe.seed, static_cast<int>(i), k, 1);
            ring.push_back(builder.vertex(p));
        }
        rings.push_back(std::move(ring));
        ringHeights.push_back(h);
    }

    // As faces: o meiao sai da gola; a faixa e a pele por altura.
    builder.part(seams.sockPart, placement.shinRegion).connectTo(placement.bootPart).connectTo(seams.skinPart);
    builder.useMaterial("meiao");
    stitchLoops(builder, placement.collar, rings.front());
    for (std::size_t i = 0; i + 1 < rings.size(); ++i) {
        const float middle = 0.5f * (ringHeights[i] + ringHeights[i + 1]);
        if (middle < recipe.sockTop) {
            builder.part(seams.sockPart).useMaterial("meiao");
        } else if (middle < recipe.bandTop) {
            builder.part(seams.sockPart).useMaterial("meiao_faixa");
        } else {
            builder.part(seams.skinPart, middle < placement.kneeHeight + 0.06f ? placement.shinRegion : placement.thighRegion);
            builder.useMaterial("pele");
        }
        builder.bridge(rings[i], rings[i + 1]);
    }
    if (placement.topOpening) builder.part(seams.skinPart).opening("barra");
    seams.top = rings.back();

    // Pesos: o pe na gola, a canela no meio, a coxa acima do joelho; as
    // transicoes no tornozelo (o meiao saindo da chuteira) e no joelho. Na
    // frente do joelho a auxiliar (que gira a metade) segura a quina: a
    // canela, a quina a 45 graus (na dobra de 90) e a coxa, sem perder o
    // volume; atras, a dobra entre a coxa e a canela.
    if (mesh.skin.size() < mesh.positions.size()) mesh.skin.resize(mesh.positions.size());
    for (std::size_t v = firstVertex; v < mesh.positions.size(); ++v) {
        const Vec3 p = mesh.positions[v];
        const float h = p.z - placement.ground;
        const float dz = h - placement.kneeHeight;
        const float frontness = 0.5f + 0.5f * std::cos(std::atan2(p.y - placement.axis.y, p.x - placement.axis.x));
        // As transicoes da frente cobrem o arco da quina (o raio da frente
        // vezes 45 graus): a pele estica aos poucos, nao numa faixa so.
        const float helperFront = smooth(-0.045f, 0.0f, dz) * (1.0f - smooth(0.02f, 0.075f, dz));
        const float helperBack = 0.3f * smooth(-0.035f, 0.0f, dz) * (1.0f - smooth(0.0f, 0.04f, dz));
        const float helper = placement.kneeHelperJoint >= 0 ? frontness * helperFront + (1.0f - frontness) * helperBack : 0.0f;
        const float upper = placement.thighJoint >= 0 ? smooth(-0.035f, 0.045f, dz) : 0.0f;
        const float thigh = (1.0f - helper) * upper;
        const float below = (1.0f - helper) * (1.0f - upper);
        const float shin = placement.shinJoint >= 0 ? smooth(placement.ankleHeight - 0.005f, placement.ankleHeight + 0.075f, h) : 1.0f;
        // No meio da transicao do tornozelo a auxiliar (a metade do giro do
        // pe): o meiao dobra sem amassar nem virar.
        const float ankle = placement.ankleHelperJoint >= 0 ? 2.0f * std::min(shin, 1.0f - shin) : 0.0f;
        SkinWeights& weights = mesh.skin[v];
        weights = {};
        std::size_t slot = 0;
        const auto add = [&](int joint, float weight) {
            if (joint < 0 || weight <= 1e-4f || slot >= 4) return;
            weights.joints[slot] = static_cast<std::uint16_t>(joint);
            weights.weights[slot] = weight;
            ++slot;
        };
        add(placement.footJoint, below * (1.0f - shin) * (1.0f - ankle));
        add(placement.ankleHelperJoint, below * ankle);
        add(placement.shinJoint, below * shin * (1.0f - ankle));
        add(placement.kneeHelperJoint, helper);
        add(placement.thighJoint, thigh);
        float total = 0.0f;
        for (const float w : weights.weights) total += w;
        if (total > 0.0f)
            for (float& w : weights.weights) w /= total;
    }
    seams.ok = true;
    return seams;
}

} // namespace MatterEngine::Modeling
