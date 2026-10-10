#include "Engine/Modeling/Parts/ArmGenerator.hpp"

#include "Engine/Modeling/Mesh/MeshBuilder.hpp"
#include "Engine/Modeling/Parts/FacetNoise.hpp"
#include "Engine/Modeling/Parts/LimbFrame.hpp"

#include <algorithm>
#include <cmath>

namespace MatterEngine::Modeling {

namespace {

float smooth(float edge0, float edge1, float x) {
    const float t = std::clamp((x - edge0) / (edge1 - edge0), 0.0f, 1.0f);
    return t * t * (3.0f - 2.0f * t);
}

} // namespace

ArmSeams buildArm(Mesh& mesh, const ArmRecipe& recipe, const ArmPlacement& placement) {
    ArmSeams seams;
    seams.part = placement.part;
    if (placement.cuff.size() < 3) {
        seams.error = "braco: falta a abertura do punho da manga";
        return seams;
    }
    MeshBuilder builder(mesh);
    const std::size_t firstVertex = mesh.positions.size();
    builder.material("pele", recipe.skinColor);
    const LimbFrame frame = limbFrame(placement.shoulder, placement.elbow);

    // O angulo de cada ponto do punho em volta do eixo: os aneis seguem os
    // mesmos (sem torcer).
    std::vector<float> angles;
    for (const std::uint32_t v : placement.cuff) {
        const Vec3 p = mesh.positions[v];
        const float s = frame.station(p);
        angles.push_back(frame.angleOf(p, frame.at(s, recipe.forward(s))));
    }
    const float elbowStation = frame.station(placement.elbow);
    // A secao: a superelipse de quatro quadrantes em volta do centro, com o
    // olecrano saltando atras do cotovelo.
    const auto bump = [](float x) { return x >= 1.0f ? 0.0f : (1.0f - x * x) * (1.0f - x * x); };
    const auto sectionPoint = [&](float s, float angle) {
        const float c = std::cos(angle), sn = std::sin(angle);
        const float e = recipe.exponent(s);
        const float across = c >= 0.0f ? recipe.front(s) : recipe.back(s);
        const float vertical = sn >= 0.0f ? recipe.top(s) : recipe.bottom(s);
        const float sum = std::pow(std::abs(c) / across, e) + std::pow(std::abs(sn) / vertical, e);
        float radius = std::pow(std::max(sum, 1e-20f), -1.0f / e);
        const float fromBack = std::abs(std::remainder(angle - Pi, 2.0f * Pi));
        radius += recipe.olecranonProtrusion * bump(fromBack / recipe.olecranonHalfAngle)
            * bump(std::abs(s - elbowStation) / recipe.olecranonHalfLength);
        return frame.at(s, recipe.forward(s)) + frame.direction(angle) * radius;
    };
    std::vector<std::vector<std::uint32_t>> rings { placement.cuff };
    for (std::size_t r = 0; r < recipe.rings.size(); ++r) {
        const float s = recipe.rings[r];
        const bool last = r + 1 == recipe.rings.size();
        std::vector<std::uint32_t> ring;
        for (std::size_t k = 0; k < angles.size(); ++k) {
            Vec3 p = sectionPoint(s, angles[k]);
            // O facetado (o anel do pulso fica liso: a costura da mao).
            if (!last) {
                const Vec3 out = safeNormalized(p - frame.at(s, recipe.forward(s)), frame.up);
                p = p + out * (recipe.jitter * facetNoise(recipe.seed, static_cast<int>(r), static_cast<int>(k), 0));
            }
            ring.push_back(builder.vertex(p));
        }
        rings.push_back(std::move(ring));
    }
    // O braco ate forearmFrom; dali o antebraco (outra parte do menu).
    seams.forearmPart = placement.forearmPart;
    builder.part(placement.part, placement.region).connectTo(placement.sleevePart);
    builder.useMaterial("pele");
    for (std::size_t r = 0; r + 1 < rings.size(); ++r) {
        const bool forearm = r > 0 && recipe.rings[r - 1] >= recipe.forearmFrom - 1e-5f;
        if (forearm) builder.part(placement.forearmPart, placement.forearmRegion).connectTo(placement.part);
        builder.bridge(rings[r], rings[r + 1]);
    }
    if (placement.wristOpening) builder.part(placement.forearmPart).opening("pulso");
    seams.wrist = rings.back();

    // Pesos: o braco, o antebraco depois do cotovelo e, na dobra, a
    // auxiliar (a metade do giro do antebraco). Atras (o olecrano, o lado de
    // fora da dobra) ela segura a quina numa faixa (que some antes do punho,
    // a uns 3 cm); na frente a pele dobra para dentro: braco e antebraco se
    // dividem numa faixa larga (de 4,5 cm acima a 7 cm abaixo da junta), a
    // dobra se espalha e nada vira do avesso ate uns 90 graus (a corrida).
    if (mesh.skin.size() < mesh.positions.size()) mesh.skin.resize(mesh.positions.size());
    for (std::size_t v = firstVertex; v < mesh.positions.size(); ++v) {
        const Vec3 p = mesh.positions[v];
        const float s = frame.station(p);
        const float t = elbowStation - s;
        const float outer = 0.5f - 0.5f * std::cos(frame.angleOf(p, frame.at(s, recipe.forward(s))));
        const float helperOuter = smooth(-0.03f, 0.0f, t) * (1.0f - smooth(0.006f, 0.026f, t));
        const float helper = placement.elbowHelperJoint >= 0 ? outer * helperOuter : 0.0f;
        const float crease = smooth(-0.07f, 0.045f, t), corner = smooth(-0.026f, 0.024f, t);
        const float upper = placement.forearmJoint >= 0 ? crease + (corner - crease) * outer : 1.0f;
        // O pulso: a auxiliar dele (a metade do giro da mao) no anel do
        // pulso, sumindo uns 3,5 cm antes.
        const float wrist = placement.wristHelperJoint >= 0
            ? smooth(placement.wristStation - 0.035f, placement.wristStation, s) : 0.0f;
        SkinWeights& weights = mesh.skin[v];
        weights = {};
        std::size_t slot = 0;
        const auto add = [&](int joint, float weight) {
            if (joint < 0 || weight <= 1e-4f || slot >= 4) return;
            weights.joints[slot] = static_cast<std::uint16_t>(joint);
            weights.weights[slot] = weight;
            ++slot;
        };
        add(placement.upperArmJoint, (1.0f - helper) * upper);
        add(placement.elbowHelperJoint, helper);
        add(placement.forearmJoint, (1.0f - helper) * (1.0f - upper) * (1.0f - wrist));
        add(placement.wristHelperJoint, (1.0f - helper) * (1.0f - upper) * wrist);
        float total = 0.0f;
        for (const float w : weights.weights) total += w;
        if (total > 0.0f)
            for (float& w : weights.weights) w /= total;
    }
    seams.ok = true;
    return seams;
}

} // namespace MatterEngine::Modeling
