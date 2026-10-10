#include "Engine/Modeling/Assembly/PlayerPaint.hpp"

#include "Engine/Modeling/Assembly/FacePaint.hpp"
#include "Engine/Modeling/Assembly/Uniform.hpp"
#include "Engine/Modeling/Mesh/TriangleBvh.hpp"
#include "Engine/Modeling/Parts/BootShape.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <optional>
#include <memory>
#include <string_view>

namespace MatterEngine::Modeling {

namespace {

// O cadarco desenhado no referencial da chuteira (x para o bico a partir do
// calcanhar, y para dentro do pe, metros sem a escala): o mesmo desenho do
// cadarco em malha (BootLaces) - as tiras por cima do cruzado, as pontas das
// tiras entrando nas fendas -, com a borda escura e a sombra no cabedal no
// lugar do relevo.
enum class LaceMark { None, Shadow, Edge, Cross, Bar, Eyelet };

float segmentDistance2(float x, float y, float ax, float ay, float bx, float by) {
    const float dx = bx - ax, dy = by - ay;
    const float t = std::clamp(((x - ax) * dx + (y - ay) * dy) / std::max(1e-12f, dx * dx + dy * dy), 0.0f, 1.0f);
    return std::hypot(x - (ax + dx * t), y - (ay + dy * t));
}

LaceMark laceMark(const BootRecipe& r, float x, float y) {
    const int rows = std::max(2, r.laceRows);
    const auto rowT = [&](int k) { return r.laceStart + (r.laceEnd - r.laceStart) * static_cast<float>(k) / static_cast<float>(rows - 1); };
    const float edge = 0.0007f, shadow = 0.0012f, slot = 0.003f;
    bool nearSomething = false;
    for (int k = 0; k < rows; ++k) {
        const float cx = rowT(k) * r.length, reach = bootLaceHalfSpacing(r, rowT(k)) + r.laceOverhang;
        const float dx = std::abs(x - cx), dy = std::abs(y);
        if (dx < 0.5f * r.laceWidth && dy < reach) {
            if (dy > reach - slot) return LaceMark::Eyelet;
            return dx > 0.5f * r.laceWidth - edge ? LaceMark::Edge : LaceMark::Bar;
        }
        nearSomething = nearSomething || (dx < 0.5f * r.laceWidth + shadow && dy < reach + shadow);
    }
    for (int k = 0; k + 1 < rows; ++k)
        for (const float sign : { 1.0f, -1.0f }) {
            const float ta = rowT(k), tb = rowT(k + 1);
            const float d = segmentDistance2(x, y, ta * r.length + r.crossEndShift, sign * bootLaceHalfSpacing(r, ta) * r.crossInset,
                tb * r.length - r.crossEndShift, -sign * bootLaceHalfSpacing(r, tb) * r.crossInset);
            if (d < 0.5f * r.crossWidth) return d > 0.5f * r.crossWidth - edge ? LaceMark::Edge : LaceMark::Cross;
            nearSomething = nearSomething || d < 0.5f * r.crossWidth + shadow;
        }
    return nearSomething ? LaceMark::Shadow : LaceMark::None;
}

Color scaledColor(Color c, float k) {
    return { std::clamp(c.r * k, 0.0f, 1.0f), std::clamp(c.g * k, 0.0f, 1.0f), std::clamp(c.b * k, 0.0f, 1.0f), c.a };
}

} // namespace

SurfacePaint playerPaint(const Mesh& mesh, const AssemblyRecipe& recipe, const Skeleton& skeleton, const PlayerPaintDesign& design,
    const BootPaint* boot) {
    const float ground = skeleton.groundHeight;
    std::vector<Color> materialColor;
    for (const MeshMaterial& material : mesh.materials) materialColor.push_back(material.color);
    // A cor de base de cada parte: a do desenho, senao a do material.
    std::vector<std::optional<Color>> partBase(mesh.parts.size());
    if (design.design != nullptr)
        for (std::size_t i = 0; i < mesh.parts.size(); ++i)
            if (const auto found = design.design->partBase.find(mesh.parts[i].name); found != design.design->partBase.end())
                partBase[i] = found->second;
    // As partes da chuteira com cadarco por cima (o cabedal e a lingua dos dois pes).
    std::vector<bool> laced(mesh.parts.size(), false), booted(mesh.parts.size(), false);
    for (std::size_t i = 0; i < mesh.parts.size(); ++i) {
        laced[i] = mesh.parts[i].name.rfind("chuteira_cabedal", 0) == 0 || mesh.parts[i].name.rfind("chuteira_lingua", 0) == 0;
        booted[i] = mesh.parts[i].name.rfind("chuteira_", 0) == 0;
    }
    const std::optional<BootPaint> laces = boot != nullptr ? std::optional<BootPaint>(*boot) : std::nullopt;
    const Transform toBoot = laces ? laces->frame.inverse() : Transform {};
    // O rosto: a superficie do olho medida na cabeca (um raio de frente no
    // centro dele).
    const int head = mesh.findPart("cabeca");
    FaceLayout face;
    face.shape = recipe.face;
    face.ground = ground;
    {
        std::vector<Triangle> headTriangles;
        for (std::size_t t = 0; t < mesh.triangles.size(); ++t)
            if (mesh.partOf(t) == head) headTriangles.push_back(mesh.triangles[t]);
        const TriangleBvh bvh(mesh.positions, headTriangles);
        const std::optional<TriangleBvh::Hit> hit = bvh.raycast({ 1.0f, recipe.face.eyeY, ground + recipe.face.eyeHeight }, { -1.0f, 0.0f, 0.0f }, 2.0f);
        face.eyeSurfaceX = hit ? 1.0f - hit->distance : 0.08f;
    }
    // As camadas na ordem do desenho, cada uma com o espaco dela.
    struct Layer {
        const PaintLayer* layer;
        const PaintSpace* space;
    };
    std::vector<Layer> layers;
    if (design.design != nullptr && design.spaces != nullptr)
        for (const PaintLayer& layer : design.design->layers)
            if (const auto space = design.spaces->find(layer.space); space != design.spaces->end()) layers.push_back({ &layer, &space->second });

    // (O desenho e os espacos ficam vivos junto com a pintura.)
    return [=, keep = design](const SurfaceSample& sample) -> Color {
        (void)keep;
        const Vec3 p = sample.position;
        // O lado esquerdo e o espelho do direito (os desenhos simetricos).
        const Vec3 q { p.x, -std::abs(p.y), p.z };
        // A normal no lado direito (o espelho troca o y).
        const Vec3 n { sample.normal.x, p.y > 0.0f ? -sample.normal.y : sample.normal.y, sample.normal.z };
        const Color material = sample.material >= 0 && static_cast<std::size_t>(sample.material) < materialColor.size()
            ? materialColor[static_cast<std::size_t>(sample.material)] : Color { 0.8f, 0.8f, 0.8f, 1.0f };
        Color color = sample.part >= 0 && static_cast<std::size_t>(sample.part) < partBase.size() && partBase[static_cast<std::size_t>(sample.part)]
            ? *partBase[static_cast<std::size_t>(sample.part)] : material;
        if (sample.part == head)
            if (const std::optional<Color> painted = paintFace(face, p, sample.normal, color)) color = *painted;
        if (sample.part >= 0 && static_cast<std::size_t>(sample.part) < booted.size() && booted[static_cast<std::size_t>(sample.part)]) {
            if (!(laces && laced[static_cast<std::size_t>(sample.part)])) return scaledColor(material, recipe.bootShade);
            // No referencial da chuteira direita (sem a escala sobre o calcanhar).
            const Vec3 heel = laces->frame.position;
            const Vec3 local = toBoot.apply(heel + (q - heel) / laces->scale);
            const Vec3 up = toBoot.applyVector(n);
            const float shade = recipe.bootShade;
            if (up.z > 0.35f) switch (laceMark(laces->recipe, local.x, local.y)) {
                case LaceMark::Bar: return scaledColor(laces->recipe.laceColor, 1.32f * shade);
                case LaceMark::Cross: return scaledColor(laces->recipe.laceColor, 1.18f * shade);
                case LaceMark::Edge: return scaledColor(laces->recipe.laceColor, 0.72f * shade);
                case LaceMark::Eyelet: return laces->recipe.eyeletColor;
                case LaceMark::Shadow: return scaledColor(material, 0.6f * shade);
                case LaceMark::None: break;
                }
            return scaledColor(material, shade);
        }
        // As camadas do desenho (o uniforme, o meiao, a orelha).
        for (const Layer& entry : layers) {
            const std::optional<SpacePoint> at = entry.space->locate(sample);
            if (!at || !entry.layer->appliesTo(at->side)) continue;
            color = paintOver(color, *entry.layer, at->point, at->footprint);
        }
        return color;
    };
}

} // namespace MatterEngine::Modeling
