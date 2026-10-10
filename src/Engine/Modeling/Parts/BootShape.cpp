#include "Engine/Modeling/Parts/BootShape.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace MatterEngine::Modeling {

namespace {

// Angulos do anel fechado (peito do pe e bico): mais pontos no topo.
constexpr std::array<float, BootRingSize> ClosedAngles { 0.0f, 0.18f, 0.40f, 0.64f, 0.88f, 1.10f, 1.34f, 1.5708f,
    1.80f, 2.04f, 2.26f, 2.50f, 2.74f, 2.96f, 3.14159f };

// Ponta arredondada (superelipse): s de 0 (onde o arredondamento comeca) a 1
// (a ponta); 1 longe dela, 0 nela.
float roundedEnd(float s, float exponent) {
    s = std::clamp(s, 0.0f, 1.0f);
    return std::pow(std::max(0.0f, 1.0f - std::pow(s, exponent)), 1.0f / exponent);
}

// Superelipse: theta 0 = base de dentro (v+), pi/2 = topo, pi = base de fora.
Point2 sectionPoint(const BootSection& section, float theta) {
    const float c = std::cos(theta), s = std::sin(theta);
    const float e = 2.0f / section.exponent;
    const float half = c >= 0.0f ? section.medial : section.lateral;
    const float height = std::pow(std::abs(s), e);
    // A barriga do lado: maxima a um terco da altura, nula na base e no topo.
    const float belly = std::sin(Pi * std::pow(height, 0.6f));
    return { (c >= 0.0f ? 1.0f : -1.0f) * half * std::pow(std::abs(c), e) * (1.0f + section.bulge * belly),
        section.base + section.height * height };
}

// O angulo do ponto da secao que fica no lado v (o inverso do de cima).
float sectionAngle(const BootSection& section, float v) {
    const float half = v >= 0.0f ? section.medial : section.lateral;
    const float ratio = std::clamp(std::abs(v) / half, 0.0f, 1.0f);
    const float angle = std::acos(std::pow(ratio, section.exponent * 0.5f));
    return v >= 0.0f ? angle : Pi - angle;
}

Point2 mix(Point2 a, Point2 b, float f) {
    return { a.x + (b.x - a.x) * f, a.y + (b.y - a.y) * f };
}

} // namespace

float bootSmoothstep(float edge0, float edge1, float x) {
    if (edge1 <= edge0) return x >= edge1 ? 1.0f : 0.0f;
    const float t = std::clamp((x - edge0) / (edge1 - edge0), 0.0f, 1.0f);
    return t * t * (3.0f - 2.0f * t);
}

float bootOutlineScale(const BootRecipe& recipe, float t) {
    if (recipe.heelRound > 0.0f && t < recipe.heelRound)
        return std::max(recipe.heelEndScale, roundedEnd((recipe.heelRound - t) / recipe.heelRound, recipe.heelRoundExponent));
    if (recipe.toeRound > 0.0f && t > 1.0f - recipe.toeRound)
        return std::max(recipe.toeEndScale, roundedEnd((t - 1.0f + recipe.toeRound) / recipe.toeRound, recipe.toeRoundExponent));
    return 1.0f;
}

float bootNoseScale(const BootRecipe& recipe, float t) {
    if (recipe.noseRound <= 0.0f) return 1.0f;
    return std::max(recipe.toeEndScale, roundedEnd((t - 1.0f + recipe.noseRound) / recipe.noseRound, recipe.noseExponent));
}

float bootLateralCollarDrop(const BootRecipe& recipe, float t) {
    return recipe.collarLateralDrop * bootSmoothstep(recipe.openingFull, recipe.openingFull + 0.08f, t)
        * (1.0f - bootSmoothstep(recipe.tongueStart - 0.08f, recipe.tongueStart, t));
}

float bootSoleTop(const BootRecipe& recipe, float t) {
    // A sola afina na ponta do bico (a frente nao vira um bloco alto).
    return recipe.soleBottom(t) + recipe.soleThickness * (1.0f - recipe.soleToeThinning * bootSmoothstep(0.9f, 1.0f, t));
}

BootHalfWidths bootBaseHalfWidths(const BootRecipe& recipe, float t) {
    const float scale = bootOutlineScale(recipe, t);
    return { std::max(0.002f, recipe.medialHalfWidth(t) * scale), std::max(0.002f, recipe.lateralHalfWidth(t) * scale) };
}

BootHalfWidths bootSoleHalfWidths(const BootRecipe& recipe, float t) {
    const BootHalfWidths base = bootBaseHalfWidths(recipe, t);
    return { base.medial / recipe.sideBulge + recipe.soleOverhang, base.lateral / recipe.sideBulge + recipe.soleOverhang };
}

BootSection bootSection(const BootRecipe& recipe, float t) {
    BootSection section;
    const BootHalfWidths half = bootBaseHalfWidths(recipe, t);
    section.medial = half.medial;
    section.lateral = half.lateral;
    section.base = bootSoleTop(recipe, t);
    section.height = std::max(0.004f, (recipe.topHeight(t) - section.base) * bootNoseScale(recipe, t));
    section.exponent = std::max(1.2f, recipe.sectionExponent);
    section.bulge = recipe.heelSideBulge * (1.0f - bootSmoothstep(0.05f, 0.17f, t));
    return section;
}

float bootOpening(const BootRecipe& recipe, float t) {
    return std::min(bootSmoothstep(recipe.openingStart, recipe.openingFull, t),
        1.0f - bootSmoothstep(recipe.tongueStart, recipe.tongueTop, t));
}

float bootOpeningAt(const BootRecipe& recipe, float t, int k) {
    static constexpr std::array<float, BootRingSize> Weight { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.15f, 0.65f, 1.0f, 0.65f,
        0.15f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };
    if (k < 0 || k >= BootRingSize) return bootOpening(recipe, t);
    const bool heelSide = t < 0.5f * (recipe.openingFull + recipe.tongueStart);
    if (!heelSide) {
        // Na lingua as paredes (0..4, 10..14) passam para o perfil fechado
        // numa janela mais longa que o meio: o lado nao vinca.
        if (Weight[static_cast<std::size_t>(k)] > 0.0f) return bootOpening(recipe, t);
        return 1.0f - bootSmoothstep(recipe.tongueStart - recipe.tongueWallBlend, recipe.tongueTop + 0.6f * recipe.tongueWallBlend, t);
    }
    const float delta = recipe.openingRound * Weight[static_cast<std::size_t>(k)];
    return bootSmoothstep(recipe.openingStart - delta, recipe.openingFull - delta, t);
}

float bootLacing(const BootRecipe& recipe, float t) {
    return bootSmoothstep(recipe.laceStart - 0.045f, recipe.laceStart - 0.005f, t)
        * (1.0f - bootSmoothstep(recipe.laceEnd + 0.005f, recipe.laceEnd + 0.05f, t));
}

float bootLaceHalfSpacing(const BootRecipe& recipe, float t) {
    const float f = std::clamp((t - recipe.laceStart) / std::max(0.01f, recipe.laceEnd - recipe.laceStart), 0.0f, 1.0f);
    return recipe.laceHalfSpacingStart + (recipe.laceHalfSpacingEnd - recipe.laceHalfSpacingStart) * f;
}

float bootLaceChannel(const BootRecipe& recipe, float t, float v) {
    const float half = bootLaceHalfSpacing(recipe, t);
    const float across = 1.0f - bootSmoothstep(half * 0.55f, half + 0.004f, std::abs(v));
    return recipe.laceChannel * bootLacing(recipe, t) * across;
}

BootRing bootRing(const BootRecipe& recipe, float t, float phase) {
    const BootSection section = bootSection(recipe, t);
    const float lacing = bootLacing(recipe, t);
    BootRing ring {};
    for (int i = 0; i < BootRingSize; ++i) {
        const std::size_t k = static_cast<std::size_t>(i);
        // No cadarco o topo nao desencontra: as abas e o canal ficam alinhados.
        const float shift = phase * (i >= 4 && i <= 10 ? 1.0f - lacing : 1.0f);
        const float angle = i == 0 || i == BootRingSize - 1
            ? ClosedAngles[k] : ClosedAngles[k] + shift * (ClosedAngles[k + 1] - ClosedAngles[k]);
        ring[k] = sectionPoint(section, angle);
    }
    // No cadarco: 5 e 9 nas abas (logo fora dos ilhoses), 6 e 8 na borda do
    // fundo do canal, 7 no meio.
    if (lacing > 0.0f) {
        const float half = bootLaceHalfSpacing(recipe, t);
        const std::array<std::pair<std::size_t, float>, 4> targets { { { 5, half + 0.003f }, { 6, half * 0.55f },
            { 8, -half * 0.55f }, { 9, -(half + 0.003f) } } };
        for (const auto& [k, v] : targets) ring[k] = mix(ring[k], sectionPoint(section, sectionAngle(section, v)), lacing);
    }
    const float open = bootOpening(recipe, t);
    // Do lado do calcanhar a parte fechada e a aba: as paredes da abertura com
    // o topo reto na altura da borda (a gola continua em volta, sem orelhas).
    const bool heelSide = t < 0.5f * (recipe.openingFull + recipe.tongueStart);
    if (open > 0.0f || heelSide || bootOpeningAt(recipe, t, 0) > 0.0f) {
        // A borda de dentro e a de fora (mais baixa no meio da abertura).
        const float collar = recipe.collarHeight(t);
        const float lateralCollar = collar - bootLateralCollarDrop(recipe, t);
        // Paredes ate a borda: a altura virtual acima dela diz o quanto elas
        // inclinam para dentro no alto (o contraforte abraca o tornozelo).
        const auto wallSection = [&](float rim) {
            BootSection walls = section;
            const float lean = recipe.collarLeanProfile.empty() ? recipe.collarLean : recipe.collarLeanProfile(t);
            walls.height = rim + lean - section.base;
            return walls;
        };
        const auto rimAngle = [&](const BootSection& walls, float rim) {
            const float ratio = std::clamp((rim - section.base) / walls.height, 0.0f, 0.999f);
            return std::asin(std::pow(ratio, walls.exponent * 0.5f));
        };
        const BootSection medialWalls = wallSection(collar), lateralWalls = wallSection(lateralCollar);
        const float medialRim = rimAngle(medialWalls, collar), lateralRim = rimAngle(lateralWalls, lateralCollar);
        BootRing opened {};
        for (int k = 0; k <= 4; ++k) {
            const float shift = k == 0 || k == 4 ? 0.0f : phase;
            opened[static_cast<std::size_t>(k)] = sectionPoint(medialWalls, medialRim * (static_cast<float>(k) + shift) / 4.0f);
            opened[static_cast<std::size_t>(14 - k)] =
                sectionPoint(lateralWalls, Pi - lateralRim * (static_cast<float>(k) - shift) / 4.0f);
        }
        // As paredes primeiro (cada ponto com a sua abertura); a borda grossa e o
        // interior saem da parede ja misturada, sem dobra.
        BootRing closedRing = ring;
        if (heelSide)
            for (std::size_t k : { 0u, 1u, 2u, 3u, 4u, 10u, 11u, 12u, 13u, 14u }) closedRing[k] = opened[k];
        for (std::size_t k : { 0u, 1u, 2u, 3u, 4u, 10u, 11u, 12u, 13u, 14u })
            opened[k] = mix(closedRing[k], opened[k], bootOpeningAt(recipe, t, static_cast<int>(k)));
        const float floor = section.base + recipe.insoleAboveSole;
        const float lowRim = std::min(opened[4].y, opened[10].y);
        const float wall = std::min(floor + 0.022f, lowRim - 0.008f);
        opened[5] = { opened[4].x - recipe.rimThickness, opened[4].y - 0.0015f };
        opened[9] = { opened[10].x + recipe.rimThickness, opened[10].y - 0.0015f };
        opened[6] = { opened[5].x - 0.003f, wall };
        opened[8] = { opened[9].x + 0.003f, wall };
        opened[7] = { (opened[4].x + opened[10].x) * 0.5f, floor };
        if (heelSide) {
            constexpr std::array<float, 5> Across { 0.2f, 0.38f, 0.5f, 0.62f, 0.8f };
            const float span = opened[4].x - opened[10].x;
            for (std::size_t k = 5; k <= 9; ++k) {
                const float f = Across[k - 5];
                const float rim = collar + (lateralCollar - collar) * f;
                ring[k] = { opened[4].x - span * f, rim + 0.0015f * std::sin(Pi * f) };
            }
        }
        for (std::size_t k : { 0u, 1u, 2u, 3u, 4u, 10u, 11u, 12u, 13u, 14u }) ring[k] = opened[k];
        for (std::size_t k = 5; k <= 9; ++k) ring[k] = mix(ring[k], opened[k], bootOpeningAt(recipe, t, static_cast<int>(k)));
    }
    // O canal do cadarco: o meio do topo (a lingua) fica abaixo das abas.
    for (std::size_t k = 5; k <= 9; ++k) ring[k].y -= bootLaceChannel(recipe, t, ring[k].x) * (1.0f - open);
    // A base entra um pouco: o lado estufa acima da sola.
    const BootHalfWidths base = bootBaseHalfWidths(recipe, t);
    ring[0].x = base.medial / recipe.sideBulge;
    ring[14].x = -base.lateral / recipe.sideBulge;
    return ring;
}

float bootHeelShift(const BootRecipe& recipe, float t, float heightFraction) {
    const float q = std::clamp(heightFraction, 0.0f, 1.0f);
    const float reach = 1.0f - bootSmoothstep(0.0f, recipe.heelSlantReach, t);
    return reach * (recipe.heelSlant * q * q - recipe.heelBulge * 4.0f * q * (1.0f - q));
}

} // namespace MatterEngine::Modeling
