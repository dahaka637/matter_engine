#include "Engine/Modeling/Parts/BootSole.hpp"

#include "Engine/Modeling/Mesh/MeshImprint.hpp"
#include "Engine/Modeling/Mesh/MeshStitch.hpp"
#include "Engine/Modeling/Parts/BootShape.hpp"

#include <algorithm>
#include <array>
#include <cmath>

namespace MatterEngine::Modeling {
namespace {

using Row = std::vector<std::pair<float, std::uint32_t>>;

void bridgeRows(MeshBuilder& builder, const Row& a, const Row& b) {
    std::size_t i = 0, j = 0;
    while (i + 1 < a.size() || j + 1 < b.size()) {
        if (i + 1 < a.size() && j + 1 < b.size() && std::abs(a[i + 1].first - b[j + 1].first) < 1e-6f) {
            builder.quad(a[i].second, b[j].second, b[j + 1].second, a[i + 1].second);
            ++i;
            ++j;
        } else if (j + 1 == b.size() || (i + 1 < a.size() && a[i + 1].first < b[j + 1].first)) {
            builder.triangle(a[i].second, b[j].second, a[i + 1].second);
            ++i;
        } else {
            builder.triangle(a[i].second, b[j].second, b[j + 1].second);
            ++j;
        }
    }
}

} // namespace

void buildBootSole(MeshBuilder& builder, const Transform& frame, const BootRecipe& recipe,
    std::span<const std::uint32_t> medialBase, std::span<const std::uint32_t> lateralBase) {
    // As fileiras ao longo do pe: 0 quina de baixo de dentro, 1 meio da
    // lateral de dentro, 2 borda de cima de dentro, 3..5 o mesmo de fora, 6..8
    // as nervuras de baixo. A face de cima (entre 2 e 3) e o cabedal.
    std::array<Row, 9> rows;
    for (std::size_t lane = 0; lane < rows.size(); ++lane) {
        // As bordas acompanham o cabedal; as nervuras usam paineis grandes.
        std::vector<float> stations = lane < 6 ? recipe.stations : recipe.solePanelStations;
        if (stations.empty()) stations = recipe.stations;
        stations.front() = recipe.stations.front();
        stations.back() = recipe.stations.back();
        for (std::size_t i = 0; i < stations.size(); ++i) {
            float t = stations[i];
            if (lane == 7 && i > 0 && i + 1 < stations.size()) t += 0.32f * (stations[i + 1] - t);
            const BootHalfWidths half = bootSoleHalfWidths(recipe, t);
            const float bottom = recipe.soleBottom(t), top = bootSoleTop(recipe, t);
            const float relief = recipe.soleRelief(t);
            // A espessura do lugar (a sola afina no bico).
            const float middle = bottom + 0.45f * (top - bottom);
            const std::array<Point2, 9> profile {{
                { half.medial - 0.0012f, bottom },
                { half.medial, middle },
                { half.medial - 0.0016f, top },
                { -half.lateral + 0.0016f, top },
                { -half.lateral, middle },
                { -half.lateral + 0.0012f, bottom },
                { -half.lateral * 0.43f, bottom - relief * 0.14f },
                { (half.medial - half.lateral) * 0.18f, bottom - relief },
                { half.medial * 0.43f, bottom - relief * 0.14f }
            }};
            // Na frente a sola acompanha o bico: passa so um pouco.
            const float x = t * recipe.length + (i + 1 == stations.size() ? recipe.soleToeOverhang : 0.0f);
            rows[lane].emplace_back(t, builder.vertex(frame.apply({ x, profile[lane].x, profile[lane].y })));
        }
    }
    Row medial, lateral;
    for (std::size_t i = 0; i < recipe.stations.size() && i < medialBase.size() && i < lateralBase.size(); ++i) {
        medial.emplace_back(recipe.stations[i], medialBase[i]);
        lateral.emplace_back(recipe.stations[i], lateralBase[i]);
    }
    // A volta da secao (anti-horaria vista do bico): de baixo de dentro,
    // subindo, a costura com o cabedal, descendo por fora e as nervuras.
    const std::array<const Row*, 11> loop { &rows[0], &rows[1], &rows[2], &medial, &lateral, &rows[3], &rows[4], &rows[5],
        &rows[6], &rows[7], &rows[8] };
    for (std::size_t k = 0; k < loop.size(); ++k) {
        // Entre as duas fileiras do cabedal fica o proprio cabedal.
        if (loop[k] == &medial) continue;
        bridgeRows(builder, *loop[k], *loop[(k + 1) % loop.size()]);
    }
    std::vector<std::uint32_t> first, last;
    for (const Row* row : loop) {
        first.push_back(row->front().second);
        last.push_back(row->back().second);
    }
    builder.cap(first, true, Cap::Flat);
    builder.cap(last, false, Cap::Flat);
}

bool buildBootStuds(MeshBuilder& builder, const Transform& frame, const BootRecipe& recipe,
    std::span<const std::uint32_t> soleTriangles, std::string* error) {
    const int sides = recipe.studSides;
    // A pegada de cada trava (horaria vista de cima = anti-horaria vista de
    // baixo, o eixo da trava) e os aneis do chanfro e da ponta.
    const auto ring = [&](const BootRecipe::Stud& stud, float radius, float z) {
        std::vector<Vec3> points;
        const float rotation = (stud.v > 0.0f ? 1.0f : -1.0f) * 0.35f;
        for (int i = 0; i < sides; ++i) {
            const float angle = rotation - 2.0f * Pi * static_cast<float>(i) / static_cast<float>(sides);
            points.push_back(frame.apply({ stud.t * recipe.length + std::cos(angle) * radius,
                stud.v + std::sin(angle) * radius, z }));
        }
        return points;
    };
    std::vector<ImprintLoop> footprints;
    for (const BootRecipe::Stud& stud : recipe.studs) {
        ImprintLoop loop;
        loop.points = ring(stud, stud.baseRadius, recipe.soleBottom(stud.t));
        footprints.push_back(std::move(loop));
    }
    ImprintSettings settings;
    settings.viewDirection = frame.applyVector({ 0.0f, 0.0f, -1.0f });
    const ImprintResult cut = imprintLoops(builder.mesh(), soleTriangles, footprints, settings);
    if (!cut.ok) {
        if (error != nullptr) *error = "travas: " + cut.error;
        return false;
    }
    for (std::size_t s = 0; s < recipe.studs.size(); ++s) {
        const BootRecipe::Stud& stud = recipe.studs[s];
        std::vector<std::uint32_t> bevel, tip;
        for (const Vec3& p : ring(stud, stud.tipRadius, recipe.studBevel)) bevel.push_back(builder.vertex(p));
        for (const Vec3& p : ring(stud, stud.tipRadius * 0.64f, 0.0f)) tip.push_back(builder.vertex(p));
        stitchCorresponding(builder, cut.loops[s], cut.inputVertices[s], bevel);
        builder.bridge(bevel, tip);
        builder.polygon(tip);
    }
    return true;
}

} // namespace MatterEngine::Modeling
