#include "Engine/Modeling/Parts/BootGenerator.hpp"

#include "Engine/Modeling/Mesh/MeshBuilder.hpp"
#include "Engine/Modeling/Mesh/MeshImprint.hpp"
#include "Engine/Modeling/Mesh/MeshRelief.hpp"
#include "Engine/Modeling/Mesh/MeshStitch.hpp"
#include "Engine/Modeling/Parts/BootLaces.hpp"
#include "Engine/Modeling/Parts/BootShape.hpp"
#include "Engine/Modeling/Parts/BootSole.hpp"
#include "Engine/Modeling/Parts/BootTongue.hpp"
#include "Engine/Modeling/Parts/FacetNoise.hpp"
#include "Engine/Modeling/Parts/SurfaceDrape.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <vector>

namespace MatterEngine::Modeling {

namespace {

struct LocalBuilder {
    MeshBuilder& builder;
    const Mesh& mesh;
    const Transform& frame;

    std::uint32_t vertex(Vec3 local) { return builder.vertex(frame.apply(local)); }
};

// Fecha a ponta de um loft. Anel mais alto que largo (o calcanhar):
// costurando os dois lados (k com n-1-k), a face em faixas, sem o leque de
// triangulos finos de um polo. Mais largo que alto (o bico): um cone curto
// para fora ("outward", metros ao longo do pe), que continua a cupula sem
// deixar uma tampa plana aparecendo. No inicio a face olha para tras, no fim
// para a frente.
void endCap(MeshBuilder& builder, const Mesh& mesh, const std::vector<std::uint32_t>& ring, bool atStart, Vec3 outward) {
    Box3 bounds;
    Vec3 center {};
    for (const std::uint32_t index : ring) {
        bounds.include(mesh.positions[index]);
        center += mesh.positions[index];
    }
    center = center / static_cast<float>(ring.size());
    const Vec3 size = bounds.size();
    if (std::hypot(size.x, size.y) > size.z) {
        builder.cap(ring, atStart, Cap::Point, center + outward);
        return;
    }
    const std::size_t n = ring.size();
    for (std::size_t k = 0; k + 1 < n / 2; ++k) {
        const std::uint32_t a = ring[k], b = ring[k + 1], c = ring[n - 2 - k], d = ring[n - 1 - k];
        if (atStart) builder.quad(d, c, b, a);
        else builder.quad(a, b, c, d);
    }
    if (n % 2 == 1) {
        const std::size_t m = n / 2;
        if (atStart) builder.triangle(ring[m + 1], ring[m], ring[m - 1]);
        else builder.triangle(ring[m - 1], ring[m], ring[m + 1]);
    }
}

// O cabedal montado: os aneis (15 vertices por estacao) e a borda de dentro
// da gola (anti-horaria vista de cima).
struct UpperRings {
    std::vector<std::vector<std::uint32_t>> rings;
    std::vector<std::uint32_t> collar;
};

// Cabedal: aneis ao longo do comprimento com o facetado aleatorio; as pontas
// arredondadas fecham costuradas. Sem fundo (a sola costura na base: k = 0 e
// 14 de cada anel) e sem o interior da abertura: a borda de dentro da gola
// (5 e 9) fica livre para a meia, do alto da aba do calcanhar ao alto da
// lingua.
UpperRings buildUpper(LocalBuilder& local, const BootRecipe& recipe) {
    MeshBuilder& builder = local.builder;
    const Mesh& mesh = local.mesh;
    const std::vector<float>& stations = recipe.stations;
    const float length = recipe.length;
    std::vector<std::vector<std::uint32_t>> rings;
    for (std::size_t i = 0; i < stations.size(); ++i) {
        const float t = stations[i];
        const float phase = i % 2 == 1 && i + 1 < stations.size() ? 0.5f : 0.0f;
        const BootRing profile = bootRing(recipe, t, phase);
        const BootSection section = bootSection(recipe, t);
        // O passo ate as estacoes vizinhas (o empurrao ao longo do comprimento).
        const float spacing = (i + 1 < stations.size() ? stations[i + 1] - t : t - stations[i - 1]) * length;
        const bool lacing = bootLacing(recipe, t) > 0.0f;
        // Nas pontas os aneis encolhem: o facetado acompanha (sem amassar).
        const float endFade = std::min(bootOutlineScale(recipe, t), bootNoseScale(recipe, t));
        // Estacoes densas (a lingua, as pontas): o tremor do tamanho do passo
        // vira uma faixa de triangulos finos marcada no lado; ele acompanha o
        // passo ate as estacoes vizinhas.
        const float before = i > 0 ? t - stations[i - 1] : 1.0f, after = i + 1 < stations.size() ? stations[i + 1] - t : 1.0f;
        const float density = std::clamp(std::min(before, after) * length / recipe.jitterSpacing, 0.2f, 1.0f);
        // A altura de referencia do contraforte (o topo, ou a borda na abertura).
        const float reach = std::max(0.004f, std::max(section.height, recipe.collarHeight(t) - section.base));
        // A lingua inclina para tras no topo (sobre a abertura).
        const float tongue = bootSmoothstep(recipe.tongueStart, recipe.tongueTop, t)
            * (1.0f - bootSmoothstep(recipe.tongueTop, recipe.tongueTop + 0.03f, t));
        std::vector<std::uint32_t> ring;
        for (int k = 0; k < BootRingSize; ++k) {
            const float open = bootOpeningAt(recipe, t, k);
            const Point2 point = profile[static_cast<std::size_t>(k)];
            Vec3 p { t * length, point.x, point.y };
            if (k >= 5 && k <= 9) p.x -= recipe.tongueLean * tongue;
            p.x += bootHeelShift(recipe, t, (p.z - section.base) / reach);
            const bool bottom = k == 0 || k == BootRingSize - 1;
            const bool rim = open > 0.5f && (k == 4 || k == 5 || k == 9 || k == 10);
            const bool top = k >= 4 && k <= 10 && open <= 0.5f;
            // Lisos: a base, a borda da abertura, o topo da aba do calcanhar e o
            // do cadarco (onde as tiras deitam).
            const bool heelTab = top && t < recipe.openingFull;
            float amplitude = bottom || rim || heelTab ? 0.0f : top ? (lacing ? 0.0f : recipe.topJitter) : recipe.jitter;
            // Os aneis das pontas ficam lisos (a tampa fecha plana, sem dobra) e
            // os ja encolhidos tremem bem menos (aneis pequenos amassariam).
            amplitude *= i == 0 || i + 1 == stations.size() ? 0.0f : endFade * endFade * endFade;
            // A biqueira e mais lisa que o resto (o couro esticado no bico).
            amplitude *= (1.0f + (recipe.toeJitter - 1.0f) * bootSmoothstep(0.76f, 0.92f, t)) * density;
            const int station = static_cast<int>(i);
            p.x += (amplitude > 0.0f ? recipe.lengthJitter * spacing * endFade : 0.0f) * facetNoise(recipe.seed, station, k, 0);
            p.y += amplitude * facetNoise(recipe.seed, station, k, 1);
            p.z += amplitude * facetNoise(recipe.seed, station, k, 2);
            ring.push_back(local.vertex(p));
        }
        rings.push_back(std::move(ring));
    }
    // A gola: da ultima estacao fechada atras (o alto da aba do calcanhar) a
    // primeira fechada na frente (o alto da lingua).
    std::size_t back = 0, front = stations.size() - 1;
    for (std::size_t i = 0; i < stations.size(); ++i)
        if (stations[i] < recipe.openingStart) back = i;
    for (std::size_t i = stations.size(); i-- > 0;)
        if (stations[i] >= recipe.tongueTop) front = i;
    builder.useMaterial("chuteira");
    for (std::size_t i = 0; i + 1 < rings.size(); ++i) {
        // k = 14 -> 0 seria o fundo: e a sola.
        for (int k = 0; k + 1 < BootRingSize; ++k) {
            if (i >= back && i < front && k >= 5 && k <= 8) continue;
            const std::size_t a = static_cast<std::size_t>(k), b = a + 1;
            builder.quad(rings[i][a], rings[i][b], rings[i + 1][b], rings[i + 1][a]);
        }
    }
    const Vec3 forward = axisX(local.frame.orientation);
    endCap(builder, mesh, rings.front(), true, forward * -0.001f);
    endCap(builder, mesh, rings.back(), false, forward * recipe.toeTipReach);
    UpperRings result;
    // Por fora pra frente, a lingua de fora para dentro, por dentro pra tras
    // e a aba de dentro para fora.
    for (std::size_t i = back; i <= front; ++i) result.collar.push_back(rings[i][9]);
    for (std::size_t k = 8; k >= 6; --k) result.collar.push_back(rings[front][k]);
    for (std::size_t i = front + 1; i-- > back;) result.collar.push_back(rings[i][5]);
    for (std::size_t k = 6; k <= 8; ++k) result.collar.push_back(rings[back][k]);
    result.rings = std::move(rings);
    return result;
}

// Pesos: tudo no pe; a frente passa para os dedos na base deles.
void applyBootWeights(Mesh& mesh, const BootPlacement& placement, const Transform& frame, std::size_t firstVertex) {
    if (placement.footJoint < 0) return;
    if (mesh.skin.size() < mesh.positions.size()) mesh.skin.resize(mesh.positions.size());
    const Vec3 forward = axisX(frame.orientation);
    for (std::size_t v = firstVertex; v < mesh.positions.size(); ++v) {
        const float u = dot(mesh.positions[v] - frame.position, forward);
        const float toe = placement.toeJoint >= 0
            ? bootSmoothstep(placement.toeBendAt - placement.toeBendWidth * 0.5f, placement.toeBendAt + placement.toeBendWidth * 0.5f, u)
            : 0.0f;
        SkinWeights& weights = mesh.skin[v];
        weights = {};
        weights.joints[0] = static_cast<std::uint16_t>(placement.footJoint);
        weights.weights[0] = 1.0f - toe;
        if (toe > 0.0f) {
            weights.joints[1] = static_cast<std::uint16_t>(placement.toeJoint);
            weights.weights[1] = toe;
        }
    }
}

} // namespace

BootSeams buildBoot(Mesh& mesh, const BootRecipe& recipe, const BootPlacement& placement) {
    BootSeams seams;
    MeshBuilder builder(mesh);
    const std::size_t firstVertex = mesh.positions.size();
    const Transform& frame = placement.frame;
    const Transform toLocal = frame.inverse();
    LocalBuilder local { builder, mesh, frame };
    const Vec3 up = frame.applyVector({ 0.0f, 0.0f, 1.0f });
    const auto fail = [&](const std::string& message) {
        seams.error = "chuteira " + placement.side + ": " + message;
        return seams;
    };

    builder.material("chuteira", recipe.upperColor);
    builder.material("chuteira_sola", recipe.soleColor);
    builder.material("chuteira_travas", recipe.studColor);
    if (placement.laceGeometry) {
        builder.material("chuteira_cadarco", recipe.laceColor);
        builder.material("chuteira_cruzado", scaled(recipe.laceColor, 0.82f));
        builder.material("chuteira_ilhos", recipe.eyeletColor);
    }

    // As partes e as costuras: o cabedal na sola (pela base), as travas na
    // sola, a lingua, os ilhoses e o cadarco no cabedal (o cadarco tambem nos
    // ilhoses, onde as tiras entram).
    seams.upperPart = "chuteira_cabedal_" + placement.side;
    seams.solePart = "chuteira_sola_" + placement.side;
    const std::string& upperPart = seams.upperPart;
    const std::string& solePart = seams.solePart;
    const std::string tonguePart = "chuteira_lingua_" + placement.side;
    const std::string eyeletPart = "chuteira_ilhos_" + placement.side;
    const std::string lacePart = "chuteira_cadarco_" + placement.side;
    const std::string studPart = "chuteira_travas_" + placement.side;

    builder.part(upperPart, placement.region).connectTo(solePart);
    if (placement.collarOpening) builder.opening("gola");
    const UpperRings upper = buildUpper(local, recipe);
    seams.collar = upper.collar;

    builder.part(solePart, placement.region);
    builder.useMaterial("chuteira_sola");
    std::vector<std::uint32_t> medialBase, lateralBase;
    for (const auto& ring : upper.rings) {
        medialBase.push_back(ring.front());
        lateralBase.push_back(ring.back());
    }
    buildBootSole(builder, frame, recipe, medialBase, lateralBase);

    builder.part(studPart, placement.region).connectTo(solePart);
    builder.useMaterial("chuteira_travas");
    std::string error;
    if (!buildBootStuds(builder, frame, recipe, mesh.partTriangles(mesh.findPart(solePart)), &error)) return fail(error);

    // A lingua em relevo sobre a superficie de verdade do cabedal.
    const auto partsTriangles = [&](std::initializer_list<const std::string*> names) {
        std::vector<std::uint32_t> list;
        for (const std::string* name : names) {
            const std::vector<std::uint32_t> part = mesh.partTriangles(mesh.findPart(*name));
            list.insert(list.end(), part.begin(), part.end());
        }
        return list;
    };
    ImprintSettings fromAbove;
    fromAbove.viewDirection = up;
    const bool tongue = !recipe.tongueHalfWidth.empty();
    if (tongue) {
        DrapeField upperField;
        upperField.addLayer(localMeshOf(mesh, partsTriangles({ &upperPart }), toLocal), true);
        const ReliefPiece pad = transformedRelief(buildBootTongue(recipe, upperField), frame);
        builder.part(tonguePart, placement.region).connectTo(upperPart);
        builder.useMaterial("chuteira");
        const std::vector<ReliefPiece> pieces { pad };
        if (!addReliefs(builder, partsTriangles({ &upperPart }), pieces, fromAbove, &error)) return fail("lingua: " + error);
    }

    // O cadarco: o campo e a superficie visivel (cabedal e lingua).
    if (!placement.laceGeometry) {
        applyBootWeights(mesh, placement, frame, firstVertex);
        seams.ok = true;
        return seams;
    }
    DrapeField field;
    field.addLayer(localMeshOf(mesh, tongue ? partsTriangles({ &upperPart, &tonguePart }) : partsTriangles({ &upperPart }), toLocal),
        true);
    const BootLacePieces laces = buildBootLaces(recipe, field, &error);
    if (laces.bars.empty()) return fail("cadarco: " + error);
    // Os ilhoses desenhados no cabedal (so a cor).
    builder.part(eyeletPart, placement.region).connectTo(upperPart);
    builder.useMaterial("chuteira_ilhos");
    {
        std::vector<ImprintLoop> eyelets;
        for (const std::vector<Vec3>& outline : laces.eyelets) {
            ImprintLoop loop;
            for (const Vec3& p : outline) loop.points.push_back(frame.apply(p));
            loop.hole = false;
            loop.material = builder.currentMaterial();
            loop.part = builder.currentPart();
            eyelets.push_back(std::move(loop));
        }
        const ImprintResult cut = imprintLoops(mesh, partsTriangles({ &upperPart }), eyelets, fromAbove);
        if (!cut.ok) return fail("ilhoses: " + cut.error);
    }
    // As pontas: os furos no cabedal (as diagonais, debaixo das tiras) e nas
    // fendas dos ilhoses (as tiras); cada cordao vai de um furo ao outro.
    builder.part(lacePart, placement.region).connectTo(upperPart).connectTo(eyeletPart);
    std::vector<const WeldedStrap*> straps;
    for (const WeldedStrap& strap : laces.crosses) straps.push_back(&strap);
    for (const WeldedStrap& strap : laces.bars) straps.push_back(&strap);
    std::vector<ImprintLoop> holes;
    for (const WeldedStrap* strap : straps)
        for (const auto* corners : { &strap->startHole, &strap->endHole }) {
            ImprintLoop loop;
            for (const Vec3& p : *corners) loop.points.push_back(frame.apply(p));
            holes.push_back(std::move(loop));
        }
    const ImprintResult cut = imprintLoops(mesh, partsTriangles({ &upperPart, &eyeletPart }), holes, fromAbove);
    if (!cut.ok) return fail("pontas do cadarco: " + cut.error);
    for (std::size_t s = 0; s < straps.size(); ++s) {
        builder.useMaterial(s < laces.crosses.size() ? "chuteira_cruzado" : "chuteira_cadarco");
        emitWeldedStrap(builder, frame, *straps[s], cut.loops[2 * s], cut.inputVertices[2 * s], cut.loops[2 * s + 1],
            cut.inputVertices[2 * s + 1]);
    }

    applyBootWeights(mesh, placement, frame, firstVertex);
    seams.ok = true;
    return seams;
}

} // namespace MatterEngine::Modeling
