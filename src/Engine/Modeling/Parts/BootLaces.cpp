#include "Engine/Modeling/Parts/BootLaces.hpp"

#include "Engine/Modeling/Parts/BootShape.hpp"

#include <algorithm>
#include <array>
#include <limits>

namespace MatterEngine::Modeling {

namespace {

// O passo das amostras ao longo das tiras (e no mergulho das pontas).
constexpr float StrapStep = 0.0025f;
constexpr float DiveStep = 0.001f;
// O comprimento do furo de cada ponta (o topo inclinando para dentro), em
// espessuras da tira, e a altura do fundo da ponta acima do apoio.
constexpr float EndSlope = 0.9f;
constexpr float EndLift = 0.0005f;
// A fenda do ilhos passa da ponta da tira isto (ao longo do pe e do lado).
constexpr float EyeletMargin = 0.0008f;

} // namespace

BootLacePieces buildBootLaces(const BootRecipe& recipe, DrapeField& field, std::string* error) {
    BootLacePieces result;
    const int rows = std::max(2, recipe.laceRows);
    const auto rowT = [&](int k) {
        return recipe.laceStart + (recipe.laceEnd - recipe.laceStart) * static_cast<float>(k) / static_cast<float>(rows - 1);
    };
    const auto at = [&](float t, float v) { return Point2 { t * recipe.length, v }; };
    const auto shifted = [](Point2 point, float dx) { return Point2 { point.x + dx, point.y }; };
    const auto failed = [&](const char* what, int k) {
        if (error != nullptr) *error = std::string(what) + " " + std::to_string(k) + ": a ponta ficou abaixo do cabedal";
        return BootLacePieces {};
    };

    // O cruzado: primeiro as diagonais de baixo (de dentro na fileira k para
    // fora na k+1), depois as de cima, que passam por cima delas. Deitam no
    // cabedal (com a folga) e as pontas entram nele debaixo das tiras.
    StrapSpec cross { recipe.crossWidth, recipe.crossThickness, recipe.laceClearance };
    cross.hostClearance = recipe.laceClearance;
    cross.dive = recipe.crossDive;
    // As pontas de duas diagonais ficam lado a lado debaixo da tira: afinam.
    cross.endWidth = recipe.crossWidth * 0.5f;
    for (const float sign : { 1.0f, -1.0f }) {
        LocalMesh layer;
        for (int k = 0; k + 1 < rows; ++k) {
            const float ta = rowT(k), tb = rowT(k + 1);
            const Point2 from = shifted(at(ta, sign * bootLaceHalfSpacing(recipe, ta) * recipe.crossInset), recipe.crossEndShift);
            const Point2 to = shifted(at(tb, -sign * bootLaceHalfSpacing(recipe, tb) * recipe.crossInset), -recipe.crossEndShift);
            const WeldedStrap strap = drapeWeldedStrap(field, sampleLine(from, to, StrapStep, DiveStep, recipe.crossDive), cross,
                recipe.crossThickness * EndSlope, EndLift);
            if (!strap.ok) return failed("diagonal", k);
            layer.append(weldedStrapMesh(strap));
            result.crosses.push_back(strap);
        }
        field.addLayer(layer);
    }

    // As tiras de cima: em arco por cima do cruzado, as pontas descendo e
    // entrando no meio da fenda do ilhos.
    StrapSpec bar { recipe.laceWidth, recipe.laceThickness, recipe.laceClearance };
    bar.taut = true;
    bar.hostClearance = recipe.laceClearance;
    bar.dive = recipe.laceDive;
    LocalMesh bars;
    for (int k = 0; k < rows; ++k) {
        const float t = rowT(k);
        const float reach = bootLaceHalfSpacing(recipe, t) + recipe.laceOverhang;
        const WeldedStrap strap = drapeWeldedStrap(field, sampleLine(at(t, -reach), at(t, reach), StrapStep, DiveStep, recipe.laceDive), bar,
            recipe.laceThickness * EndSlope, EndLift);
        if (!strap.ok) return failed("tira", k);
        bars.append(weldedStrapMesh(strap));
        result.bars.push_back(strap);
        // A fenda: o furo da ponta com a margem, desenhada no cabedal.
        for (const auto* hole : { &strap.startHole, &strap.endHole }) {
            float minX = std::numeric_limits<float>::max(), maxX = -minX, minY = minX, maxY = -minX;
            for (const Vec3& c : *hole) {
                minX = std::min(minX, c.x);
                maxX = std::max(maxX, c.x);
                minY = std::min(minY, c.y);
                maxY = std::max(maxY, c.y);
            }
            minX -= EyeletMargin;
            maxX += EyeletMargin;
            // Ao longo da tira (o lado) a fenda tem o comprimento da receita.
            const float centerY = (minY + maxY) * 0.5f;
            const float alongHalf = std::max((maxY - minY) * 0.5f + EyeletMargin, recipe.slotLength * 0.5f);
            minY = centerY - alongHalf;
            maxY = centerY + alongHalf;
            std::vector<Vec3> outline;
            for (const Point2 corner : { Point2 { minX, minY }, Point2 { maxX, minY }, Point2 { maxX, maxY }, Point2 { minX, maxY } })
                outline.push_back({ corner.x, corner.y, field.hostHeight(corner.x, corner.y).value_or(0.0f) });
            result.eyelets.push_back(std::move(outline));
        }
    }
    field.addLayer(bars);
    return result;
}

} // namespace MatterEngine::Modeling
