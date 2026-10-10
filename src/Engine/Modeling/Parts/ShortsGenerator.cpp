#include "Engine/Modeling/Parts/ShortsGenerator.hpp"

#include "Engine/Modeling/Mesh/MeshBuilder.hpp"
#include "Engine/Modeling/Mesh/MeshImprint.hpp"
#include "Engine/Modeling/Mesh/MeshStitch.hpp"
#include "Engine/Modeling/Parts/FacetNoise.hpp"
#include "Engine/Modeling/Parts/NumberGlyphs.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <functional>

namespace MatterEngine::Modeling {

namespace {

float smooth(float edge0, float edge1, float x) {
    const float t = std::clamp((x - edge0) / (edge1 - edge0), 0.0f, 1.0f);
    return t * t * (3.0f - 2.0f * t);
}

// A superelipse de quatro quadrantes em volta de "center": ate front/back
// no x e ate plusY/minusY no y; angle 0 = +x, pi/2 = +y.
Vec3 superellipse(Point2 center, float front, float back, float plusY, float minusY, float exponent, float angle, float z) {
    const float c = std::cos(angle), s = std::sin(angle);
    const float e = 2.0f / exponent;
    const float x = (c >= 0.0f ? front : -back) * std::pow(std::abs(c), e);
    const float y = (s >= 0.0f ? plusY : -minusY) * std::pow(std::abs(s), e);
    return { center.x + x, center.y + y, z };
}

// O angulo em (low, high) onde x(angle) vale "target" (x monotono ali).
float angleForX(const std::function<float(float)>& x, float low, float high, float target) {
    const bool increasing = x(high) > x(low);
    for (int i = 0; i < 48; ++i) {
        const float middle = 0.5f * (low + high);
        if ((x(middle) < target) == increasing) low = middle;
        else high = middle;
    }
    return 0.5f * (low + high);
}

} // namespace

ShortsSeams buildShortsHalf(Mesh& mesh, const ShortsRecipe& recipe, const ShortsPlacement& placement) {
    ShortsSeams seams;
    seams.part = placement.part;
    if (placement.legTop.size() < 3) {
        seams.error = "calcao: falta a barra da perna para costurar";
        return seams;
    }
    MeshBuilder builder(mesh);
    const std::size_t firstVertex = mesh.positions.size();
    builder.material("calcao", recipe.shortsColor);
    builder.material("calcao_faixa", recipe.stripeColor);
    builder.material("calcao_dentro", recipe.insideColor);
    const bool covered = placement.coverHeight > 0.0f;
    builder.part(placement.part, placement.region).connectTo(placement.legPart);
    if (!covered) builder.opening("cintura");
    const Point2 body { placement.bodyX, 0.0f };
    const float ground = placement.ground;
    const float seat = recipe.seatHeight;
    const float half = recipe.stripeWidth * 0.5f;

    const auto hipPoint = [&](float h, float phi) {
        const float side = recipe.hipSide(h);
        return superellipse(body, recipe.hipFront(h), recipe.hipBack(h), side, side, recipe.hipExponent(h), phi, ground + h);
    };
    // A perna direita: o lado de dentro (medial) e +y.
    const auto legPoint = [&](float h, float angle) {
        return superellipse(placement.legAxis, recipe.legFront(h), recipe.legBack(h), recipe.legMedial(h), recipe.legLateral(h),
            2.2f, angle, ground + h);
    };

    // A metade do quadril (de tras pela direita ate a frente): as costas
    // (pi), os pontos de tras, as duas bordas da faixa (x = centro -/+ meia
    // faixa), os da frente e a frente (2 pi).
    const int count = recipe.hipPoints;
    const int interior = count - 2;
    const int backCount = (interior - 2) / 2, frontCount = interior - 2 - backCount;
    const int stripeSlot = 1 + backCount;
    const auto hipSlots = [&](float h) {
        const auto xAt = [&](float phi) { return hipPoint(h, phi).x; };
        const float phiBack = angleForX(xAt, Pi + 0.01f, 1.5f * Pi, placement.bodyX - half);
        const float phiFront = angleForX(xAt, 1.5f * Pi, 2.0f * Pi - 0.01f, placement.bodyX + half);
        std::vector<float> phi { Pi };
        for (int j = 1; j <= backCount; ++j) phi.push_back(Pi + (phiBack - Pi) * static_cast<float>(j) / static_cast<float>(backCount + 1));
        phi.push_back(phiBack);
        phi.push_back(phiFront);
        for (int j = 1; j <= frontCount; ++j)
            phi.push_back(phiFront + (2.0f * Pi - phiFront) * static_cast<float>(j) / static_cast<float>(frontCount + 1));
        phi.push_back(2.0f * Pi);
        return phi;
    };
    // O facetado: o ponto anda para dentro/fora do centro (nunca os do meio,
    // as bordas da faixa ou as costuras).
    const auto jittered = [&](Vec3 p, Point2 center, int ring, int k) {
        const Vec3 out = safeNormalized({ p.x - center.x, p.y - center.y, 0.0f }, {});
        return p + out * (recipe.jitter * facetNoise(recipe.seed, ring, k, 0));
    };
    const auto onMiddle = [](Vec3 p) {
        p.y = 0.0f;
        return p;
    };

    // Os aneis do quadril: o assento, os do meio, o alto do corpo e o cos
    // (saltado para fora: a costura dele).
    struct HipRing {
        float height;
        float step;
        bool clean;
    };
    std::vector<HipRing> hipHeights { { seat, 0.0f, false } };
    const float bandBottom = recipe.waistHeight - recipe.waistbandHeight;
    if (covered) {
        // Por baixo da camisa: os aneis ate perto da costura e o anel dela
        // (liso), sem o cos.
        for (const float h : recipe.hipRings)
            if (h < placement.coverHeight - 0.012f) hipHeights.push_back({ h, 0.0f, false });
        hipHeights.push_back({ placement.coverHeight, 0.0f, true });
    } else {
        for (const float h : recipe.hipRings) hipHeights.push_back({ h, 0.0f, false });
        hipHeights.push_back({ bandBottom, 0.0f, true });
        hipHeights.push_back({ bandBottom, recipe.waistbandStep, true });
        hipHeights.push_back({ recipe.waistHeight, recipe.waistbandStep, true });
    }
    const float topHeight = covered ? placement.coverHeight : recipe.waistHeight;
    std::vector<std::vector<std::uint32_t>> hip;
    for (std::size_t r = 0; r < hipHeights.size(); ++r) {
        const auto [h, step, clean] = hipHeights[r];
        const std::vector<float> slots = hipSlots(h);
        std::vector<std::uint32_t> ring;
        for (std::size_t k = 0; k < slots.size(); ++k) {
            Vec3 p = hipPoint(h, slots[k]);
            const bool middle = k == 0 || k + 1 == slots.size();
            if (middle) p = onMiddle(p);
            if (step > 0.0f) p = p + safeNormalized({ p.x - body.x, p.y - body.y, 0.0f }, {}) * step;
            const bool edge = static_cast<int>(k) == stripeSlot || static_cast<int>(k) == stripeSlot + 1;
            if (!clean && !middle && !edge) p = jittered(p, body, 100 + static_cast<int>(r), static_cast<int>(k));
            // Sobe/desce um pouco (o facetado irregular), menos no assento (a
            // costura das pernas) e no cos.
            if (!clean && r > 0 && r + 1 < hipHeights.size()) {
                const float spacing = std::min(h - hipHeights[r - 1].height, hipHeights[r + 1].height - h);
                p.z += std::min(recipe.verticalJitter, 0.25f * spacing) * facetNoise(recipe.seed, 100 + static_cast<int>(r), static_cast<int>(k), 1);
            }
            ring.push_back(builder.vertex(p));
        }
        hip.push_back(std::move(ring));
    }

    // A virilha: a costura no meio (y = 0) da frente as costas por baixo,
    // descendo do assento ate o ponto mais baixo.
    const std::vector<std::uint32_t>& seatRing = hip.front();
    const Vec3 front = mesh.positions[seatRing.back()], back = mesh.positions[seatRing.front()];
    std::vector<std::uint32_t> crotch;
    for (int j = 1; j <= recipe.crotchPoints; ++j) {
        const float s = static_cast<float>(j) / static_cast<float>(recipe.crotchPoints + 1);
        const float dip = std::pow(std::sin(Pi * s), 0.6f);
        crotch.push_back(builder.vertex({ front.x + (back.x - front.x) * s, 0.0f,
            ground + seat - (seat - recipe.crotchHeight) * dip }));
    }
    // O alto da perna direita (anti-horario visto de cima): a frente, a
    // virilha ate as costas e o arco de fora (o assento) de volta a frente.
    std::vector<std::uint32_t> top { seatRing.back() };
    top.insert(top.end(), crotch.begin(), crotch.end());
    top.push_back(seatRing.front());
    for (std::size_t k = 1; k + 1 < seatRing.size(); ++k) top.push_back(seatRing[k]);
    const std::size_t slots = top.size();
    const int legStripe = recipe.crotchPoints + 1 + stripeSlot;
    // O angulo de cada ponto em volta do eixo da perna (crescente).
    std::vector<float> angles;
    for (const std::uint32_t v : top) {
        const Vec3 p = mesh.positions[v];
        float a = std::atan2(p.y - placement.legAxis.y, p.x - placement.legAxis.x);
        while (!angles.empty() && a <= angles.back()) a += 2.0f * Pi;
        angles.push_back(a);
    }
    // Os aneis da perna do calcao: a secao em volta da coxa virando o alto
    // (a virilha e o assento) perto do topo; as bordas da faixa no lugar.
    std::vector<float> legHeights { recipe.hemHeight };
    legHeights.insert(legHeights.end(), recipe.legRings.begin(), recipe.legRings.end());
    std::vector<std::vector<std::uint32_t>> leg;
    for (std::size_t r = 0; r < legHeights.size(); ++r) {
        const float h = legHeights[r];
        const float blend = smooth(recipe.hemHeight + 0.04f, seat + 0.01f, h);
        const auto xAt = [&](float a) { return legPoint(h, a).x; };
        std::vector<float> ringAngles = angles;
        // As bordas da faixa no lado de fora (x = centro -/+ meia faixa).
        const float outside = 1.5f * Pi;
        ringAngles[static_cast<std::size_t>(legStripe)] = angleForX(xAt, Pi + 0.05f, outside, placement.bodyX - half);
        ringAngles[static_cast<std::size_t>(legStripe + 1)] = angleForX(xAt, outside, 2.0f * Pi - 0.05f, placement.bodyX + half);
        std::vector<std::uint32_t> ring;
        for (std::size_t s = 0; s < slots; ++s) {
            const Vec3 target = mesh.positions[top[s]];
            Vec3 p = legPoint(h, ringAngles[s]);
            p = lerp(p, target, blend);
            const bool edge = static_cast<int>(s) == legStripe || static_cast<int>(s) == legStripe + 1;
            if (r > 0 && !edge) p = jittered(p, placement.legAxis, 200 + static_cast<int>(r), static_cast<int>(s));
            if (r > 0) {
                const float next = r + 1 < legHeights.size() ? legHeights[r + 1] : seat;
                const float spacing = std::min(h - legHeights[r - 1], next - h);
                p.z += std::min(recipe.verticalJitter, 0.25f * spacing) * facetNoise(recipe.seed, 200 + static_cast<int>(r), static_cast<int>(s), 1);
            }
            ring.push_back(builder.vertex(p));
        }
        leg.push_back(std::move(ring));
    }
    leg.push_back(top);

    // As faces. A barra por baixo (o lado de dentro do calcao aparece ali).
    builder.useMaterial("calcao_dentro");
    stitchLoops(builder, placement.legTop, leg.front());
    // A perna do calcao (a faixa entre as duas bordas).
    for (std::size_t r = 0; r + 1 < leg.size(); ++r)
        for (std::size_t s = 0; s < slots; ++s) {
            builder.useMaterial(static_cast<int>(s) == legStripe ? "calcao_faixa" : "calcao");
            const std::size_t next = (s + 1) % slots;
            builder.quad(leg[r][s], leg[r][next], leg[r + 1][next], leg[r + 1][s]);
        }
    // O quadril (metade: sem a volta pelo meio); a faixa ate o cos.
    for (std::size_t r = 0; r + 1 < hip.size(); ++r) {
        const bool band = !covered && hipHeights[r].height >= bandBottom - 1e-5f;
        for (std::size_t k = 0; k + 1 < hip[r].size(); ++k) {
            builder.useMaterial(!band && static_cast<int>(k) == stripeSlot ? "calcao_faixa" : "calcao");
            builder.quad(hip[r][k], hip[r][k + 1], hip[r + 1][k + 1], hip[r + 1][k]);
        }
    }

    // Pesos: a perna do calcao segue a coxa, o quadril a pelve, o cos ja
    // puxa o abdomen; os do meio dividem as duas coxas por igual.
    if (mesh.skin.size() < mesh.positions.size()) mesh.skin.resize(mesh.positions.size());
    for (std::size_t v = firstVertex; v < mesh.positions.size(); ++v) {
        const Vec3 p = mesh.positions[v];
        const float h = p.z - ground;
        float thigh = h <= seat ? 1.0f - 0.6f * smooth(recipe.hemHeight + 0.03f, seat, h) : 0.4f * (1.0f - smooth(seat, seat + 0.06f, h));
        const bool middle = std::abs(p.y) < 1e-6f;
        // O meio da virilha fica quase todo nas duas auxiliares (com as
        // pernas para lados opostos, o carrinho, ele fica entre as duas). Os
        // vizinhos do alto da perna colados nele (a menos de 1,6 cm, na altura
        // da virilha) vao a meio caminho: o triangulo entre eles nao dobra
        // quando uma coxa sobe muito (agachando, no carrinho).
        const float nearMiddle = middle ? 1.0f
            : 0.5f * (1.0f - smooth(0.006f, 0.016f, std::abs(p.y))) * smooth(recipe.crotchHeight - 0.008f, recipe.crotchHeight - 0.003f, h)
                * (1.0f - smooth(seat, seat + 0.02f, h));
        thigh += (std::min(thigh, h < seat - 1e-4f ? 0.65f : 0.5f) - thigh) * nearMiddle;
        const bool waistBand = placement.abdomenTo > placement.abdomenFrom;
        const float abdomen = placement.abdomenJoint < 0 ? 0.0f
            : waistBand ? smooth(placement.abdomenFrom, placement.abdomenTo, h)
                        : 0.25f * smooth(topHeight - 0.05f, topHeight, h);
        // No meio da passagem da coxa para a pelve, a auxiliar da virilha (a
        // metade do giro da coxa): a dobra da frente e o gluteo sem amassar.
        const bool helpers = placement.rightHipHelperJoint >= 0 && placement.leftHipHelperJoint >= 0;
        const float helper = helpers ? 1.6f * std::min(thigh, 1.0f - thigh - abdomen) : 0.0f;
        const float thighShare = thigh - 0.5f * helper;
        const float pelvis = 1.0f - thigh - abdomen - 0.5f * helper;
        SkinWeights& weights = mesh.skin[v];
        weights = {};
        std::size_t slot = 0;
        const auto add = [&](int joint, float weight) {
            if (joint < 0 || weight <= 1e-4f || slot >= 4) return;
            weights.joints[slot] = static_cast<std::uint16_t>(joint);
            weights.weights[slot] = weight;
            ++slot;
        };
        add(placement.pelvisJoint, pelvis);
        if (helpers && nearMiddle > 0.0f) {
            // O meio: as duas virilhas por igual (simetrico; cabe em 4 juntas);
            // os vizinhos colados, a mistura do lado com o meio.
            const float shared = 0.5f * (helper + thighShare) * nearMiddle;
            add(placement.rightThighJoint, thighShare * (1.0f - nearMiddle));
            add(placement.rightHipHelperJoint, helper * (1.0f - nearMiddle) + shared);
            add(placement.leftHipHelperJoint, shared);
        } else if (middle) {
            add(placement.rightThighJoint, thigh * 0.5f);
            add(placement.leftThighJoint, thigh * 0.5f);
        } else {
            add(placement.rightThighJoint, thighShare);
            add(placement.rightHipHelperJoint, helper);
        }
        add(placement.abdomenJoint, abdomen);
        float total = 0.0f;
        for (const float w : weights.weights) total += w;
        if (total > 0.0f)
            for (float& w : weights.weights) w /= total;
    }
    seams.top = hip.back();
    seams.ok = true;
    return seams;
}

bool imprintShortsNumber(Mesh& mesh, const ShortsRecipe& recipe, Point2 leftLegAxis, float ground, const std::string& part,
    std::string* error) {
    if (recipe.number.empty()) return true;
    const int partIndex = mesh.findPart(part);
    if (partIndex < 0) {
        if (error != nullptr) *error = "numero: o calcao nao existe";
        return false;
    }
    MeshBuilder builder(mesh);
    builder.material("calcao_numero", recipe.numberColor);
    const int numberMaterial = mesh.findMaterial("calcao_numero");
    const int shortsMaterial = mesh.findMaterial("calcao");
    // A perna esquerda: o lado de fora e +y (o angulo da receita e da direita).
    const float angle = -recipe.numberAngle;
    const Vec3 out { std::cos(angle), std::sin(angle), 0.0f };
    // Desenhado num plano tangente a perna e projetado nela pela frente; visto
    // de fora, "across" aponta para a direita de quem olha.
    NumberLayout layout;
    layout.center = { leftLegAxis.x + out.x * 0.2f, leftLegAxis.y + out.y * 0.2f,
        ground + recipe.hemHeight + recipe.numberAboveHem + recipe.numberHeight * 0.5f };
    layout.right = { -out.y, out.x, 0.0f };
    layout.height = recipe.numberHeight;
    layout.stroke = recipe.numberStroke / recipe.numberHeight;
    const std::vector<ImprintLoop> loops = numberLoops(recipe.number, layout, numberMaterial, shortsMaterial);
    ImprintSettings settings;
    settings.viewDirection = out;
    const ImprintResult cut = imprintLoops(mesh, mesh.partTriangles(partIndex), loops, settings);
    if (!cut.ok) {
        if (error != nullptr) *error = "numero do calcao: " + cut.error;
        return false;
    }
    return true;
}

} // namespace MatterEngine::Modeling
