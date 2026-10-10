#include "Commands.hpp"

#include "Engine/Modeling/Inspection/ViewFraming.hpp"
#include "Engine/Modeling/Raster/ImageIO.hpp"

#include <algorithm>
#include <cstdlib>
#include <iostream>

namespace MatterEngine::ModelInspector {

using namespace Modeling;

std::optional<ViewRequest> viewFromOptions(const ModelContext& context, const CommandLine& line, int defaultWidth,
    int defaultHeight) {
    ViewRequest request;
    request.width = std::clamp(line.integer("largura", defaultWidth), 64, 4096);
    request.height = std::clamp(line.integer("altura", defaultHeight), 64, 4096);
    const InspectionModel& model = context.model();
    request.region = context.region(line.text("regiao", "visao_geral"));
    if (request.region < 0) return std::nullopt;

    InspectionStyle& style = request.style;
    const std::string modeId = line.text("modo", model.mesh.empty() ? "ossos" : "solido");
    const std::optional<SurfaceMode> mode = parseSurfaceMode(modeId);
    if (!mode) {
        std::cerr << "modo desconhecido: " << modeId << " (ossos, solido, solido_arame, arame, raiox, normais, partes, "
                  << "pesos, argila)\n";
        return std::nullopt;
    }
    style.surface = *mode;
    style.focusRegion = request.region;
    style.isolate = line.has("isolar");
    style.jointLabels = line.has("rotulos");
    style.grid = !line.has("sem-grade");
    style.hud = !line.has("sem-hud");
    style.skeleton = !line.has("sem-ossos");
    style.backfaces = !line.has("sem-avesso");
    style.supersample = std::clamp(line.integer("ssaa", 3), 1, 4);
    style.frontLight = line.has("luz-frontal");
    style.creaseAngle = std::clamp(line.number("quinas", style.creaseAngle), 0.0f, 180.0f);
    style.facet = std::clamp(line.number("facetado", style.facet), 0.0f, 1.0f);
    if (line.has("junta")) {
        style.weightJoint = model.skeleton.find(line.text("junta"));
        if (style.weightJoint < 0) {
            std::cerr << "junta desconhecida: " << line.text("junta") << '\n';
            return std::nullopt;
        }
    }

    const InspectionScene scene = context.scene();
    const float aspect = static_cast<float>(request.width) / static_cast<float>(request.height);
    const bool orthographic = line.has("orto");
    const ViewPreset* preset = nullptr;
    if (line.has("vista")) {
        preset = findViewPreset(line.text("vista"));
        if (preset == nullptr) {
            std::cerr << "vista desconhecida: " << line.text("vista")
                      << " (frente, costas, direita, esquerda, cima, baixo, 34, 34e, 34c)\n";
            return std::nullopt;
        }
        request.viewId = std::string(preset->id);
    }
    request.camera = frameRegion(scene, request.region, aspect, preset, orthographic);
    if (line.has("yaw") || line.has("pitch")) {
        const float yaw = line.number("yaw", request.camera.yaw * RadiansToDegrees);
        const float pitch = line.number("pitch", request.camera.pitch * RadiansToDegrees);
        request.camera = frameBounds(regionBounds(scene, request.region), yaw, pitch, aspect,
            model.regions.regions[static_cast<std::size_t>(request.region)].margin, orthographic);
        request.viewId = "yaw" + std::to_string(static_cast<int>(yaw)) + "_pitch" + std::to_string(static_cast<int>(pitch));
    }
    // --alvo x,y,z [--raio m]: enquadra um ponto (um detalhe de perto), no
    // angulo pedido (ou o da regiao).
    if (line.has("alvo")) {
        const std::vector<std::string> parts = line.list("alvo");
        if (parts.size() != 3) {
            std::cerr << "--alvo espera x,y,z (metros, espaco do modelo)\n";
            return std::nullopt;
        }
        Vec3 target {};
        float* axes[] = { &target.x, &target.y, &target.z };
        for (std::size_t i = 0; i < 3; ++i) {
            std::string value = parts[i];
            std::replace(value.begin(), value.end(), ';', '.');
            *axes[i] = std::strtof(value.c_str(), nullptr);
        }
        const float radius = std::max(0.001f, line.number("raio", 0.02f));
        Box3 box;
        box.include(target, radius);
        const float yaw = line.number("yaw", request.camera.yaw * RadiansToDegrees);
        const float pitch = line.number("pitch", request.camera.pitch * RadiansToDegrees);
        request.camera = frameBounds(box, yaw, pitch, aspect, 1.0f, orthographic);
        request.viewId = "alvo";
    }
    if (request.viewId.empty()) request.viewId = "padrao";
    // --fov: o angulo vertical da lente (graus); reenquadra no mesmo angulo.
    if (line.has("fov")) {
        const float yaw = request.camera.yaw, pitch = request.camera.pitch;
        request.camera.verticalFov = std::clamp(line.number("fov", 28.0f), 5.0f, 100.0f) * DegreesToRadians;
        request.camera.yaw = yaw;
        request.camera.pitch = pitch;
        framePoints(request.camera, regionPoints(scene, request.region), aspect,
            model.regions.regions[static_cast<std::size_t>(request.region)].margin);
    }
    const float zoom = std::max(0.05f, line.number("zoom", 1.0f));
    request.camera.distance /= zoom;
    request.camera.orthoHalfHeight /= zoom;

    const BodyRegion& region = model.regions.regions[static_cast<std::size_t>(request.region)];
    style.title = region.label + " · " + (preset != nullptr ? std::string(preset->label) : std::string("ângulo da região"))
        + " · " + std::string(surfaceModeLabel(style.surface));
    style.notes.push_back(model.source);
    if (!context.poseLabel.empty()) style.notes.push_back("pose: " + context.poseLabel);
    if (orthographic) style.notes.push_back("projeção ortográfica");
    return request;
}

bool saveImage(const Image& image, const std::filesystem::path& path) {
    std::string error;
    if (!writePng(image, path.string(), &error)) {
        std::cerr << error << '\n';
        return false;
    }
    std::cout << "imagem: " << std::filesystem::absolute(path).string() << " (" << image.width() << "x"
              << image.height() << ")\n";
    return true;
}

} // namespace MatterEngine::ModelInspector
