#include "Engine/Modeling/Inspection/InspectionRenderer.hpp"

#include "Engine/Modeling/Geometry/Parallel.hpp"
#include "Engine/Modeling/Mesh/MeshSkinning.hpp"
#include "Engine/Modeling/Raster/Rasterizer.hpp"
#include "Engine/Modeling/Raster/TextRenderer.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <unordered_set>

namespace MatterEngine::Modeling {

namespace {

struct ModeInfo {
    SurfaceMode mode;
    std::string_view id;
    std::string_view label;
};

constexpr std::array<ModeInfo, 9> ModeInfos { {
    { SurfaceMode::Hidden, "ossos", "Só ossos" },
    { SurfaceMode::Solid, "solido", "Sólido" },
    { SurfaceMode::SolidWire, "solido_arame", "Sólido + arame" },
    { SurfaceMode::Wire, "arame", "Arame" },
    { SurfaceMode::XRay, "raiox", "Raio-x" },
    { SurfaceMode::Normals, "normais", "Normais" },
    { SurfaceMode::Parts, "partes", "Partes" },
    { SurfaceMode::Weights, "pesos", "Pesos de skin" },
    { SurfaceMode::Clay, "argila", "Argila" },
} };

constexpr std::array<SurfaceMode, 9> Modes { SurfaceMode::Hidden, SurfaceMode::Solid, SurfaceMode::SolidWire,
    SurfaceMode::Wire, SurfaceMode::XRay, SurfaceMode::Normals, SurfaceMode::Parts, SurfaceMode::Weights,
    SurfaceMode::Clay };

Color sideColor(Side side) {
    switch (side) {
    case Side::Left: return Palette::Left;
    case Side::Right: return Palette::Right;
    case Side::Center: break;
    }
    return Palette::Center;
}

// Luzes presas a camera (como num estudio): a forma fica legivel de
// qualquer angulo.
struct Lighting {
    PixelLight light;

    Lighting(const Projector& projector, bool frontal) {
        const Vec3 f = projector.forward(), r = projector.right(), u = projector.up();
        light.key = frontal ? safeNormalized(-f * 0.85f + u * 0.45f - r * 0.12f, { 0.0f, 0.0f, 1.0f })
                            : safeNormalized(-f * 0.55f + u * 0.75f - r * 0.45f, { 0.0f, 0.0f, 1.0f });
        light.fill = safeNormalized(-f * 0.7f + r * 0.65f - u * 0.15f, { 0.0f, 0.0f, 1.0f });
        light.back = safeNormalized(f * 0.6f + u * 0.5f, { 0.0f, 0.0f, 1.0f });
    }

    // A luz vista de "view" (da camera para a superficie).
    [[nodiscard]] PixelLight at(Vec3 view) const {
        PixelLight seen = light;
        seen.view = view;
        return seen;
    }

    [[nodiscard]] float intensity(Vec3 normal, Vec3 view) const { return at(view).intensity(normal); }

    [[nodiscard]] Color shade(Color base, Vec3 normal, Vec3 view) const {
        const float k = intensity(normal, view);
        return { std::clamp(base.r * k, 0.0f, 1.0f), std::clamp(base.g * k, 0.0f, 1.0f),
            std::clamp(base.b * k, 0.0f, 1.0f), base.a };
    }
};

// A regua: um comprimento "redondo" com pelo menos 60 px.
float niceLength(float pixelsPerMeter) {
    constexpr std::array<float, 11> Steps { 0.001f, 0.002f, 0.005f, 0.01f, 0.02f, 0.05f, 0.1f, 0.2f, 0.5f, 1.0f, 2.0f };
    for (const float step : Steps)
        if (step * pixelsPerMeter >= 60.0f) return step;
    return Steps.back();
}

struct Box2 {
    float x0, y0, x1, y1;
    [[nodiscard]] bool overlaps(const Box2& other) const {
        return x0 < other.x1 && x1 > other.x0 && y0 < other.y1 && y1 > other.y0;
    }
};

} // namespace

std::string_view surfaceModeId(SurfaceMode mode) {
    for (const ModeInfo& info : ModeInfos)
        if (info.mode == mode) return info.id;
    return "solido";
}

std::string_view surfaceModeLabel(SurfaceMode mode) {
    for (const ModeInfo& info : ModeInfos)
        if (info.mode == mode) return info.label;
    return "Sólido";
}

std::optional<SurfaceMode> parseSurfaceMode(std::string_view id) {
    for (const ModeInfo& info : ModeInfos)
        if (info.id == id) return info.mode;
    return std::nullopt;
}

std::span<const SurfaceMode> allSurfaceModes() {
    return Modes;
}

Highlight highlightFor(const Finding& finding) {
    Highlight highlight;
    highlight.triangles = finding.triangles;
    highlight.segments = finding.segments;
    highlight.points = finding.points;
    highlight.color = finding.severity == Severity::Error ? Palette::Error
        : finding.severity == Severity::Warning          ? Palette::Warning
                                                         : Palette::Info;
    highlight.label = finding.code;
    highlight.labelAt = finding.location;
    highlight.hasLabel = finding.hasLocation;
    return highlight;
}

std::vector<Vec3> sceneJointPositions(const InspectionScene& scene) {
    std::vector<Vec3> positions;
    if (scene.skeleton == nullptr) return positions;
    const std::size_t count = scene.skeleton->joints.size();
    positions.reserve(count);
    const bool posed = scene.pose != nullptr && scene.pose->joints.size() == count;
    for (std::size_t j = 0; j < count; ++j)
        positions.push_back(posed ? scene.pose->joints[j].position : scene.skeleton->joints[j].bind.position);
    return positions;
}

int partRegion(const InspectionScene& scene, int part) {
    if (scene.mesh == nullptr || scene.regions == nullptr || part < 0
        || static_cast<std::size_t>(part) >= scene.mesh->parts.size())
        return -1;
    return scene.regions->find(scene.mesh->parts[static_cast<std::size_t>(part)].region);
}

namespace {

struct Visibility {
    const InspectionScene& scene;
    const InspectionStyle& style;
    bool focusing = false;

    Visibility(const InspectionScene& sceneValue, const InspectionStyle& styleValue)
        : scene(sceneValue), style(styleValue) {
        focusing = style.focusRegion >= 0 && scene.regions != nullptr
            && static_cast<std::size_t>(style.focusRegion) < scene.regions->regions.size()
            && !scene.regions->regions[static_cast<std::size_t>(style.focusRegion)].overview;
    }
    [[nodiscard]] int jointRegion(int joint) const {
        return joint >= 0 && static_cast<std::size_t>(joint) < scene.jointRegion.size()
            ? scene.jointRegion[static_cast<std::size_t>(joint)] : -1;
    }
    [[nodiscard]] bool regionShown(int region) const {
        return !style.isolate || !focusing || region == style.focusRegion;
    }
};

// "corners": as normais lisas dos cantos (3 por triangulo), ou nulo
// (facetado).
void drawSurfaces(const InspectionScene& scene, const InspectionStyle& style, const Visibility& visibility,
    std::span<const Vec3> positions, Rasterizer& raster, const Projector& projector, float pixel,
    std::vector<float>& xrayBuffer, const std::vector<Vec3>* corners) {
    const Mesh& mesh = *scene.mesh;
    const SurfaceMode mode = style.surface;
    std::vector<bool> partShown(mesh.parts.size(), true);
    for (std::size_t p = 0; p < mesh.parts.size(); ++p)
        partShown[p] = visibility.regionShown(partRegion(scene, static_cast<int>(p)));
    const auto shown = [&](std::size_t t) {
        const int part = mesh.partOf(t);
        return part < 0 || static_cast<std::size_t>(part) >= partShown.size() || partShown[static_cast<std::size_t>(part)];
    };
    const Lighting lights(projector, style.frontLight);

    if (mode == SurfaceMode::XRay) {
        RasterTarget& target = raster.target();
        std::vector<float>& buffer = xrayBuffer;
        for (std::size_t t = 0; t < mesh.triangles.size(); ++t) {
            if (!shown(t)) continue;
            const Triangle& tri = mesh.triangles[t];
            const Vec3 a = positions[tri[0]], b = positions[tri[1]], c = positions[tri[2]];
            const Vec3 view = projector.viewDirection((a + b + c) / 3.0f);
            const float facing = std::abs(dot(triangleNormal(a, b, c), view));
            raster.accumulate(a, b, c, 0.035f + 0.30f * (1.0f - facing) * (1.0f - facing), buffer);
        }
        for (int y = raster.rowBegin(); y < raster.rowEnd(); ++y)
            for (int x = 0; x < target.width(); ++x) {
                const float amount = buffer[static_cast<std::size_t>(y) * target.width() + x];
                if (amount > 0.0f)
                    target.color.blendPixel(x, y, Palette::XRay, std::min(0.92f, 1.0f - std::exp(-amount * 1.4f)));
            }
        return;
    }

    std::vector<float> vertexWeight;
    if (mode == SurfaceMode::Weights && mesh.hasSkin() && style.weightJoint >= 0) {
        vertexWeight.resize(mesh.positions.size());
        for (std::size_t v = 0; v < mesh.positions.size(); ++v) vertexWeight[v] = weightFor(mesh.skin[v], style.weightJoint);
    }
    for (std::size_t t = 0; t < mesh.triangles.size(); ++t) {
        if (!shown(t)) continue;
        const Triangle& tri = mesh.triangles[t];
        const Vec3 a = positions[tri[0]], b = positions[tri[1]], c = positions[tri[2]];
        const Vec3 normal = triangleNormal(a, b, c);
        const Vec3 view = projector.viewDirection((a + b + c) / 3.0f);
        const bool front = dot(normal, view) < 0.0f;
        const Vec3 litNormal = front ? normal : -normal;
        if (mode == SurfaceMode::Wire) {
            raster.depthOnly(a, b, c);
            continue;
        }
        if (mode == SurfaceMode::Normals) {
            raster.triangle(a, b, c,
                front ? Color { normal.x * 0.5f + 0.5f, normal.y * 0.5f + 0.5f, normal.z * 0.5f + 0.5f, 1.0f }
                      : Palette::Backface);
            continue;
        }
        if (mode == SurfaceMode::Weights && !vertexWeight.empty()) {
            const float k = lights.intensity(litNormal, view) * 0.85f + 0.15f;
            raster.triangleGradient(a, b, c, scaled(heatColor(vertexWeight[tri[0]]), k),
                scaled(heatColor(vertexWeight[tri[1]]), k), scaled(heatColor(vertexWeight[tri[2]]), k));
            continue;
        }
        Color base = Palette::Surface;
        const int part = mesh.partOf(t);
        const MeshMaterial* material = t < mesh.triangleMaterial.size() && mesh.triangleMaterial[t] < mesh.materials.size()
            ? &mesh.materials[mesh.triangleMaterial[t]] : nullptr;
        if (mode == SurfaceMode::Parts) base = categoricalColor(part);
        else if (mode == SurfaceMode::Clay || mode == SurfaceMode::Weights) base = rgb(178, 180, 186);
        else if (material != nullptr) base = material->color;
        if (style.backfaces && !front) base = Palette::Backface;
        // Liso: as normais dos cantos (viradas junto com a face vista por
        // tras); o avesso marcado fica chapado.
        const bool smooth = corners != nullptr && (mode == SurfaceMode::Solid || mode == SurfaceMode::SolidWire) && (!style.backfaces || front);
        std::array<Vec3, 3> normals {};
        if (smooth)
            for (std::size_t k = 0; k < 3; ++k) normals[k] = front ? (*corners)[t * 3 + k] : -(*corners)[t * 3 + k];
        // A textura do material (solido): a pintura do atlas sob a mesma luz.
        if ((mode == SurfaceMode::Solid || mode == SurfaceMode::SolidWire) && (!style.backfaces || front) && material != nullptr
            && material->texture >= 0 && static_cast<std::size_t>(material->texture) < mesh.textures.size() && mesh.hasUv()) {
            const std::array<Uv, 3>& uv = mesh.triangleUv[t];
            const Image& texture = mesh.textures[static_cast<std::size_t>(material->texture)].image;
            if (smooth) raster.triangleTexturedLit(a, b, c, uv[0], uv[1], uv[2], normals, texture, lights.at(view));
            else raster.triangleTextured(a, b, c, uv[0], uv[1], uv[2], texture, lights.intensity(litNormal, view));
            continue;
        }
        if (smooth) raster.triangleLit(a, b, c, normals, withAlpha(base, 1.0f), lights.at(view));
        else raster.triangle(a, b, c, lights.shade(withAlpha(base, 1.0f), litNormal, view));
    }

    if (mode == SurfaceMode::SolidWire || mode == SurfaceMode::Wire) {
        std::unordered_set<std::uint64_t> drawn;
        const Color color = mode == SurfaceMode::Wire ? rgb(150, 205, 255, 0.95f) : Palette::Wire;
        const float thickness = (mode == SurfaceMode::Wire ? 1.25f : 1.0f) * pixel;
        for (std::size_t t = 0; t < mesh.triangles.size(); ++t) {
            if (!shown(t)) continue;
            const Triangle& tri = mesh.triangles[t];
            for (int i = 0; i < 3; ++i) {
                const std::uint32_t va = tri[static_cast<std::size_t>(i)], vb = tri[static_cast<std::size_t>((i + 1) % 3)];
                const std::uint64_t key = (static_cast<std::uint64_t>(std::min(va, vb)) << 32) | std::max(va, vb);
                if (!drawn.insert(key).second) continue;
                raster.line(positions[va], positions[vb], thickness, color, DepthMode::Test, 0.0f, 0.0015f);
            }
        }
    }
}

void drawGrid(const InspectionScene& scene, const InspectionStyle& style, const OrbitCamera& camera, Rasterizer& raster,
    float pixel) {
    const float ground = scene.skeleton->groundHeight;
    // Mais fina de perto: 1 cm, 5 cm ou 10 cm (a linha forte a cada 5).
    const float extent = style.gridExtent > 0.0f ? style.gridExtent : std::clamp(camera.distance * 0.9f, 0.3f, 2.5f);
    const float step = style.gridStep > 0.0f ? style.gridStep
        : camera.distance < 0.6f ? 0.01f : camera.distance < 1.4f ? 0.05f : 0.1f;
    const int count = static_cast<int>(std::ceil(extent / step));
    const float cx = std::round(camera.target.x / step) * step;
    const float cy = std::round(camera.target.y / step) * step;
    for (int i = -count; i <= count; ++i) {
        const float x = cx + static_cast<float>(i) * step;
        const float y = cy + static_cast<float>(i) * step;
        const bool majorX = std::lround(x / step) % 5 == 0;
        const bool majorY = std::lround(y / step) % 5 == 0;
        raster.line({ x, cy - extent, ground }, { x, cy + extent, ground }, (majorX ? 1.3f : 1.0f) * pixel,
            majorX ? Palette::GridMajor : Palette::Grid, DepthMode::Test, 0.0f, 0.0f);
        raster.line({ cx - extent, y, ground }, { cx + extent, y, ground }, (majorY ? 1.3f : 1.0f) * pixel,
            majorY ? Palette::GridMajor : Palette::Grid, DepthMode::Test, 0.0f, 0.0f);
    }
}

// O disco no chao (um pouco acima, misturado), o brilho em faixas que sobem do
// aro cada vez mais fracas e o aro por cima. Atras do corpo ele some.
void drawMarkers(const InspectionScene& scene, Rasterizer& raster, float pixel) {
    constexpr int Segments = 64;
    constexpr int Bands = 12;
    constexpr float Tau = 6.2831853f;
    for (const InspectionMarker& marker : scene.markers) {
        std::array<Vec3, Segments> rim {};
        for (int i = 0; i < Segments; ++i) {
            const float angle = Tau * static_cast<float>(i) / static_cast<float>(Segments);
            rim[static_cast<std::size_t>(i)] =
                marker.center + Vec3 { std::cos(angle) * marker.radius, std::sin(angle) * marker.radius, 0.0f };
        }
        const Vec3 lift { 0.0f, 0.0f, 0.003f };
        const auto at = [&](int i) { return rim[static_cast<std::size_t>((i + Segments) % Segments)]; };
        for (int i = 0; i < Segments; ++i)
            raster.overlayTriangle(marker.center + lift, at(i) + lift, at(i + 1) + lift, withAlpha(marker.color, 0.42f),
                0.01f);
        const Color glow = mix(marker.color, rgb(255, 214, 200), 0.35f);
        for (int band = 0; band < Bands; ++band) {
            const Vec3 low { 0.0f, 0.0f, marker.glowHeight * static_cast<float>(band) / static_cast<float>(Bands) };
            const Vec3 high { 0.0f, 0.0f, marker.glowHeight * static_cast<float>(band + 1) / static_cast<float>(Bands) };
            const float fade = 1.0f - (static_cast<float>(band) + 0.5f) / static_cast<float>(Bands);
            const Color color = withAlpha(glow, 0.26f * fade * fade);
            for (int i = 0; i < Segments; ++i) {
                raster.overlayTriangle(at(i) + low, at(i + 1) + low, at(i + 1) + high, color, 0.01f);
                raster.overlayTriangle(at(i) + low, at(i + 1) + high, at(i) + high, color, 0.01f);
            }
        }
        for (int i = 0; i < Segments; ++i)
            raster.line(at(i) + lift, at(i + 1) + lift, 2.4f * pixel, withAlpha(marker.color, 0.95f), DepthMode::Test,
                0.0f, 0.004f);
    }
}

// A haste e a ponta como faixas chapadas na horizontal (misturadas, atras do
// corpo somem), com a borda escura para ler sobre a grade.
void drawArrows(const InspectionScene& scene, Rasterizer& raster) {
    for (const InspectionArrow& arrow : scene.arrows) {
        Vec3 along = arrow.to - arrow.from;
        along.z = 0.0f;
        const float length = along.length();
        if (length < 1.0e-3f) continue;
        along = along * (1.0f / length);
        const Vec3 side { -along.y, along.x, 0.0f };
        const float head = std::min(length * 0.45f, arrow.width * 3.5f);
        const Vec3 neck = arrow.from + along * (length - head);
        const Vec3 tip = arrow.from + along * length;
        const Vec3 up { 0.0f, 0.0f, arrow.to.z - arrow.from.z };
        const auto lift = [&](Vec3 p, float t) { return p + up * t; };
        const float half = 0.5f * arrow.width;
        const Color fill = withAlpha(arrow.color, 0.92f);
        const Vec3 a = arrow.from + side * half, b = arrow.from - side * half;
        const Vec3 c = neck - side * half, d = neck + side * half;
        const float neckT = (length - head) / length;
        raster.overlayTriangle(a, b, lift(c, neckT), fill, 0.02f);
        raster.overlayTriangle(a, lift(c, neckT), lift(d, neckT), fill, 0.02f);
        raster.overlayTriangle(lift(neck + side * (half * 2.6f), neckT), lift(neck - side * (half * 2.6f), neckT),
            lift(tip, 1.0f), fill, 0.02f);
    }
}

void drawSkeleton(const InspectionScene& scene, const InspectionStyle& style, const Visibility& visibility,
    std::span<const Vec3> joints, Rasterizer& raster, DepthMode depthMode, float pixel) {
    const Skeleton& skeleton = *scene.skeleton;
    const int focus = visibility.focusing ? style.focusRegion : -1;
    for (std::size_t j = 0; j < skeleton.joints.size(); ++j) {
        const Joint& child = skeleton.joints[j];
        // As auxiliares do skin nao tem osso proprio (ficam no joelho).
        if (child.parent < 0 || child.kind == JointKind::Helper) continue;
        // O osso pertence a regiao da junta de onde ele sai.
        const int region = visibility.jointRegion(child.parent);
        if (!visibility.regionShown(region)) continue;
        const Joint& parent = skeleton.joints[static_cast<std::size_t>(child.parent)];
        const bool thin = child.kind == JointKind::Finger || child.kind == JointKind::Toe
            || (child.kind == JointKind::Tip && parent.kind != JointKind::Body);
        float thickness = (thin ? 1.7f : 3.2f) * pixel;
        Color color = sideColor(child.kind == JointKind::Tip ? parent.side : child.side);
        if (focus >= 0) {
            if (region == focus) thickness *= 1.35f;
            else color = withAlpha(color, 0.28f);
        }
        raster.line(joints[static_cast<std::size_t>(child.parent)], joints[j], thickness, color, depthMode, 0.3f, 0.0f);
    }
    if (!style.joints) return;
    for (std::size_t j = 0; j < skeleton.joints.size(); ++j) {
        const Joint& joint = skeleton.joints[j];
        const int region = visibility.jointRegion(static_cast<int>(j));
        if (!visibility.regionShown(region) || joint.kind == JointKind::Helper) continue;
        const float radius = (joint.kind == JointKind::Tip ? 2.0f
                                 : joint.kind == JointKind::Body ? 4.6f
                                                                 : 2.6f) * pixel;
        Color fill = focus >= 0 && region == focus ? Palette::Focus : mix(sideColor(joint.side), rgb(255, 255, 255), 0.35f);
        if (focus >= 0 && region != focus) fill = withAlpha(fill, 0.3f);
        raster.disc(joints[j], radius + 1.2f * pixel, rgb(10, 12, 16, fill.a), depthMode, 0.3f, 0.0f);
        raster.disc(joints[j], radius, fill, depthMode, 0.3f, 0.0f);
    }
}

void drawJointLabels(Image& image, const InspectionScene& scene, const Visibility& visibility, const InspectionStyle& style,
    std::span<const Vec3> joints, const Projector& projector, float size) {
    const Skeleton& skeleton = *scene.skeleton;
    const TextRenderer& text = TextRenderer::standard();
    std::vector<int> order;
    for (std::size_t j = 0; j < skeleton.joints.size(); ++j)
        if (skeleton.joints[j].kind != JointKind::Tip && skeleton.joints[j].kind != JointKind::Helper)
            order.push_back(static_cast<int>(j));
    std::stable_sort(order.begin(), order.end(), [&](int a, int b) {
        return (skeleton.joints[static_cast<std::size_t>(a)].kind == JointKind::Body)
            > (skeleton.joints[static_cast<std::size_t>(b)].kind == JointKind::Body);
    });
    std::vector<Box2> placed;
    for (const int j : order) {
        const int region = visibility.jointRegion(j);
        if (!visibility.regionShown(region)) continue;
        if (visibility.focusing && region != style.focusRegion) continue;
        const ScreenPoint p = projector.project(joints[static_cast<std::size_t>(j)]);
        if (!p.visible || p.x < 0 || p.y < 0 || p.x > image.width() || p.y > image.height()) continue;
        const std::string& name = skeleton.joints[static_cast<std::size_t>(j)].name;
        const TextSize measured = text.measure(name, size);
        const float w = measured.width + 8.0f, h = size * 1.18f + 6.0f;
        const std::array<std::array<float, 2>, 4> offsets { { { 8.0f, -h - 3.0f }, { -w - 8.0f, -h - 3.0f },
            { 8.0f, 4.0f }, { -w - 8.0f, 4.0f } } };
        for (const auto& offset : offsets) {
            const Box2 box { p.x + offset[0], p.y + offset[1], p.x + offset[0] + w, p.y + offset[1] + h };
            if (box.x0 < 0 || box.y0 < 0 || box.x1 > image.width() || box.y1 > image.height()) continue;
            const bool free = std::none_of(placed.begin(), placed.end(), [&](const Box2& other) { return box.overlaps(other); });
            if (!free) continue;
            placed.push_back(box);
            drawLine2D(image, p.x, p.y, box.x0 < p.x ? box.x1 : box.x0, box.y0 < p.y ? box.y1 : box.y0, 1.0f,
                withAlpha(Palette::TextMuted, 0.7f));
            text.drawLabel(image, box.x0, box.y0, name, size, Palette::Text, Palette::LabelBackground, 3.0f);
            break;
        }
    }
}

void drawGizmo(Image& image, const Projector& projector, float size, bool legend) {
    const TextRenderer& text = TextRenderer::standard();
    const float cx = 46.0f, cy = static_cast<float>(image.height()) - 50.0f, length = 30.0f;
    fillDisc2D(image, cx, cy, 40.0f, withAlpha(Palette::LabelBackground, 0.6f));
    const std::array<std::pair<Vec3, Color>, 3> axes { { { { 1, 0, 0 }, rgb(240, 80, 80) },
        { { 0, 1, 0 }, rgb(110, 220, 110) }, { { 0, 0, 1 }, rgb(90, 150, 255) } } };
    const char* letters[] = { "X", "Y", "Z" };
    for (std::size_t i = 0; i < axes.size(); ++i) {
        const Vec3 axis = axes[i].first;
        const float sx = dot(axis, projector.right()), sy = -dot(axis, projector.up());
        drawLine2D(image, cx, cy, cx + sx * length, cy + sy * length, 2.2f, axes[i].second);
        text.drawOutlined(image, cx + sx * (length + 6.0f) - size * 0.3f, cy + sy * (length + 6.0f) - size * 0.6f,
            letters[i], size, axes[i].second, rgb(0, 0, 0, 0.8f));
    }
    if (legend)
        text.drawOutlined(image, cx + 46.0f, cy + 14.0f, "X frente · Y esquerda · Z cima", size * 0.85f,
            Palette::TextMuted, rgb(0, 0, 0, 0.7f));
}

void drawScaleBar(Image& image, const Projector& projector, const OrbitCamera& camera, float size) {
    const float z = projector.toCamera(camera.target).z;
    const float ppm = projector.pixelsPerMeter(z);
    if (ppm <= 0.0f) return;
    const float meters = niceLength(ppm);
    const float length = meters * ppm;
    const float x1 = static_cast<float>(image.width()) - 24.0f, x0 = x1 - length;
    const float y = static_cast<float>(image.height()) - 24.0f;
    drawLine2D(image, x0, y, x1, y, 2.0f, Palette::Text);
    drawLine2D(image, x0, y - 5.0f, x0, y + 5.0f, 2.0f, Palette::Text);
    drawLine2D(image, x1, y - 5.0f, x1, y + 5.0f, 2.0f, Palette::Text);
    const std::string label = formatLength(meters) + " (no alvo)";
    const TextRenderer& text = TextRenderer::standard();
    const TextSize measured = text.measure(label, size);
    text.drawOutlined(image, x1 - measured.width, y - size - 10.0f, label, size, Palette::Text, rgb(0, 0, 0, 0.8f));
}

void drawHud(Image& image, const InspectionStyle& style, float titleSize, float noteSize) {
    if (style.title.empty() && style.notes.empty()) return;
    const TextRenderer& text = TextRenderer::standard();
    float width = text.measure(style.title, titleSize).width;
    for (const std::string& note : style.notes) width = std::max(width, text.measure(note, noteSize).width);
    const float height = (style.title.empty() ? 0.0f : titleSize * 1.35f)
        + static_cast<float>(style.notes.size()) * noteSize * 1.3f + 16.0f;
    image.fillRect(10, 10, static_cast<int>(width + 34.0f), static_cast<int>(10.0f + height), Palette::LabelBackground);
    float y = 18.0f;
    if (!style.title.empty()) {
        text.draw(image, 20.0f, y, style.title, titleSize, Palette::Text);
        y += titleSize * 1.35f;
    }
    for (const std::string& note : style.notes) {
        text.draw(image, 20.0f, y, note, noteSize, Palette::TextMuted);
        y += noteSize * 1.3f;
    }
}

} // namespace

Image renderInspection(const InspectionScene& scene, const InspectionStyle& style, const OrbitCamera& camera, int width,
    int height) {
    width = std::max(16, width);
    height = std::max(16, height);
    int supersample = std::clamp(style.supersample, 1, 4);
    while (supersample > 1 && static_cast<long long>(width) * height * supersample * supersample > 36'000'000LL)
        --supersample;
    const float pixel = static_cast<float>(supersample);

    RasterTarget target(width * supersample, height * supersample);
    target.color.fillVerticalGradient(Palette::BackgroundTop, Palette::BackgroundBottom);
    if (scene.backdrop != nullptr && !scene.backdrop->empty())
        target.color.drawImage(scene.backdrop->resized(target.width(), target.height()), 0, 0, scene.backdropOpacity);
    const Projector projector(camera, target.width(), target.height());
    Rasterizer raster(target, projector);
    const Visibility visibility(scene, style);
    const std::vector<Vec3> joints = sceneJointPositions(scene);

    const Mesh* mesh = scene.mesh;
    std::span<const Vec3> positions = scene.meshPositions;
    if (mesh != nullptr && positions.empty()) positions = mesh->positions;
    const bool hasMesh = mesh != nullptr && !mesh->empty() && positions.size() == mesh->positions.size();
    std::function<void(Rasterizer&)> surfaces;
    // (Ficam vivos ate as faixas desenharem.)
    std::vector<float> xrayBuffer;
    std::vector<Vec3> cornerNormals;
    if (hasMesh && style.surface != SurfaceMode::Hidden) {
        if (style.surface == SurfaceMode::XRay)
            xrayBuffer.assign(static_cast<std::size_t>(target.width()) * target.height(), 0.0f);
        // O sombreamento liso: as normais dos cantos na pose, uma vez so (os
        // grupos da cena, ou feitos aqui na quina do estilo).
        const bool smooth = style.facet < 0.999f && (style.surface == SurfaceMode::Solid || style.surface == SurfaceMode::SolidWire);
        if (smooth) {
            std::optional<ShadingNormals> local;
            const ShadingNormals* shading = scene.shading;
            if (shading == nullptr || shading->triangleCount() != mesh->triangles.size()
                || std::abs(shading->creaseDegrees() - style.creaseAngle) > 1e-3f)
                shading = &local.emplace(*mesh, style.creaseAngle);
            shading->compute(positions, mesh->triangles, style.facet, cornerNormals);
        }
        const std::vector<Vec3>* corners = smooth ? &cornerNormals : nullptr;
        surfaces = [&, corners](Rasterizer& rows) {
            drawSurfaces(scene, style, visibility, positions, rows, projector, pixel, xrayBuffer, corners);
        };
    }
    // As superficies e a grade (escondida atras delas) em faixas de linhas,
    // uma por vez em cada thread: cada faixa so escreve nas linhas dela, e a
    // grade de uma faixa so depende das superficies da mesma faixa.
    const bool grid = style.grid && scene.skeleton != nullptr;
    const int bands = (surfaces || grid) ? std::clamp(target.height() / 64, 1, 32) : 0;
    parallelFor(static_cast<std::size_t>(bands), 1, [&](std::size_t first, std::size_t last) {
        for (std::size_t band = first; band < last; ++band) {
            Rasterizer rows(target, projector);
            rows.restrictRows(target.height() * static_cast<int>(band) / bands, target.height() * static_cast<int>(band + 1) / bands);
            if (surfaces) surfaces(rows);
            if (grid) drawGrid(scene, style, camera, rows, pixel);
        }
    });

    drawMarkers(scene, raster, pixel);
    drawArrows(scene, raster);
    for (const Highlight& highlight : scene.highlights) {
        if (hasMesh)
            for (const std::uint32_t t : highlight.triangles) {
                if (t >= mesh->triangles.size()) continue;
                const Triangle& tri = mesh->triangles[t];
                raster.overlayTriangle(positions[tri[0]], positions[tri[1]], positions[tri[2]],
                    withAlpha(highlight.color, 0.55f));
            }
        for (const auto& [a, b] : highlight.segments)
            raster.line(a, b, 2.4f * pixel, highlight.color, DepthMode::Ghost, 0.35f, 0.003f);
        for (const Vec3& point : highlight.points)
            raster.disc(point, 4.0f * pixel, highlight.color, DepthMode::Ghost, 0.4f);
    }

    if (style.skeleton && scene.skeleton != nullptr) {
        const bool opaque = hasMesh && style.surface != SurfaceMode::Hidden && style.surface != SurfaceMode::XRay;
        drawSkeleton(scene, style, visibility, joints, raster, opaque ? DepthMode::Ghost : DepthMode::Ignore, pixel);
    }

    Image image = target.color.downsampled(supersample);
    const Projector overlay(camera, width, height);
    const float labelSize = std::clamp(static_cast<float>(height) / 58.0f, 11.0f, 17.0f);
    const float titleSize = std::clamp(static_cast<float>(height) / 36.0f, 14.0f, 26.0f);
    if (style.jointLabels && scene.skeleton != nullptr)
        drawJointLabels(image, scene, visibility, style, joints, overlay, labelSize);
    const TextRenderer& text = TextRenderer::standard();
    for (const Highlight& highlight : scene.highlights) {
        if (!highlight.hasLabel) continue;
        const ScreenPoint p = overlay.project(highlight.labelAt);
        if (!p.visible) continue;
        // (Perto da borda direita o rotulo vai para a esquerda do ponto.)
        const float labelWidth = text.measure(highlight.label, labelSize).width + 6.0f;
        const float right = p.x + 8.0f;
        const float x = right + labelWidth < static_cast<float>(image.width()) - 4.0f
            ? right : std::max(4.0f, p.x - labelWidth - 8.0f);
        text.drawLabel(image, x, p.y - labelSize - 8.0f, highlight.label, labelSize, highlight.color,
            Palette::LabelBackground, 3.0f);
    }
    if (style.gizmo) drawGizmo(image, overlay, labelSize, style.gizmoLegend);
    if (style.scaleBar) drawScaleBar(image, overlay, camera, labelSize);
    if (style.hud) drawHud(image, style, titleSize, labelSize);
    return image;
}

int pickJoint(const InspectionScene& scene, const InspectionStyle& style, const OrbitCamera& camera, int width,
    int height, float x, float y, float radiusPixels) {
    if (scene.skeleton == nullptr) return -1;
    const Projector projector(camera, width, height);
    const Visibility visibility(scene, style);
    const std::vector<Vec3> joints = sceneJointPositions(scene);
    int best = -1;
    float bestDistance = radiusPixels;
    for (std::size_t j = 0; j < joints.size(); ++j) {
        if (!visibility.regionShown(visibility.jointRegion(static_cast<int>(j)))) continue;
        const ScreenPoint p = projector.project(joints[j]);
        if (!p.visible) continue;
        const float d = std::hypot(p.x - x, p.y - y);
        if (d < bestDistance) {
            bestDistance = d;
            best = static_cast<int>(j);
        }
    }
    return best;
}

} // namespace MatterEngine::Modeling
