#include "Commands.hpp"

#include "Engine/Modeling/Analysis/CrossSection.hpp"
#include "Engine/Modeling/Inspection/ContactSheet.hpp"
#include "Engine/Modeling/Inspection/ViewFraming.hpp"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <iterator>
#include <span>
#include <string>
#include <vector>

namespace MatterEngine::ModelInspector {

using namespace Modeling;

namespace {

// Camera olhando no rumo dado (o plano do corte de frente).
OrbitCamera lookingAlong(Vec3 direction) {
    OrbitCamera camera;
    const Vec3 d = safeNormalized(direction, { -1.0f, 0.0f, 0.0f });
    camera.pitch = std::asin(std::clamp(-d.z, -0.999f, 0.999f));
    camera.yaw = std::atan2(-d.y, -d.x);
    return camera;
}

} // namespace

// O corte da malha por um plano: o contorno de frente (2D) e no corpo (3D).
//   --junta <nome> [--t 0.5]   plano perpendicular ao osso
//   --altura <m>               plano horizontal a essa altura do chao
//   --plano frontal|sagital [--deslocamento <m>]   plano vertical
//   --partes camisa,calcao     um contorno por parte, cada um numa cor
//   --alvo x,y,z [--raio m]    a vista de frente enquadrada nesse ponto
// Com --pose, a malha e cortada na pose (o plano fica no espaco do modelo).
int runSection(const ModelContext& context, const CommandLine& line) {
    const InspectionModel& model = context.model();
    if (model.mesh.empty()) {
        std::cerr << "sem malha para cortar (so ossos)\n";
        return 2;
    }
    Plane plane;
    std::string label;
    Vec3 marker;
    bool hasMarker = false;
    if (line.has("junta")) {
        const int joint = model.skeleton.find(line.text("junta"));
        const int child = limbChild(model.skeleton, joint);
        if (joint < 0 || child < 0) {
            std::cerr << "junta desconhecida ou sem filha: " << line.text("junta") << '\n';
            return 2;
        }
        const float t = line.number("t", 0.5f);
        const Vec3 a = model.skeleton.joints[static_cast<std::size_t>(joint)].bind.position;
        const Vec3 b = model.skeleton.joints[static_cast<std::size_t>(child)].bind.position;
        marker = lerp(a, b, t);
        hasMarker = true;
        plane = Plane::fromPointNormal(marker, b - a);
        label = "osso " + line.text("junta") + " em t=" + formatNumber(t, 2);
    } else if (line.has("altura")) {
        const float z = model.skeleton.groundHeight + line.number("altura", 1.0f);
        plane = Plane::fromPointNormal({ 0.0f, 0.0f, z }, { 0.0f, 0.0f, 1.0f });
        label = "altura " + formatLength(line.number("altura", 1.0f)) + " do chao";
    } else {
        const bool sagittal = line.text("plano", "sagital") == "sagital";
        const float offset = line.number("deslocamento", 0.0f);
        plane = sagittal ? Plane::fromPointNormal({ 0.0f, offset, 0.0f }, { 0.0f, 1.0f, 0.0f })
                         : Plane::fromPointNormal({ offset, 0.0f, 0.0f }, { 1.0f, 0.0f, 0.0f });
        label = std::string(sagittal ? "plano sagital" : "plano frontal") + " em " + formatLength(offset);
    }
    // Na pose, a malha deformada.
    const std::span<const Vec3> positions = context.posedPositions.empty() ? std::span<const Vec3>(model.mesh.positions)
                                                                           : std::span<const Vec3>(context.posedPositions);
    // Uma passada por parte pedida (cada uma na cor dela), ou todas juntas.
    struct Pass {
        std::string name;
        std::vector<int> parts;
        Color color;
    };
    std::vector<Pass> passes;
    if (line.has("partes")) {
        const Color colors[] { rgb(80, 255, 230), rgb(255, 200, 60), rgb(255, 90, 200), rgb(140, 255, 90) };
        for (const std::string& name : line.list("partes")) {
            const int part = model.mesh.findPart(name);
            if (part < 0) {
                std::cerr << "parte desconhecida: " << name << '\n';
                return 2;
            }
            passes.push_back({ name, { part }, colors[passes.size() % std::size(colors)] });
        }
    } else {
        passes.push_back({ "", {}, rgb(80, 255, 230) });
    }
    Box3 bounds;
    std::vector<Highlight> highlights;
    std::size_t loops = 0;
    float total = 0.0f;
    for (const Pass& pass : passes) {
        const MeshSection section = sliceMesh(model.mesh, positions, plane, pass.parts);
        if (!pass.name.empty()) std::cout << "Parte " << pass.name << ":\n";
        Highlight contour;
        contour.color = pass.color;
        for (std::size_t i = 0; i < section.loops.size(); ++i) {
            const SectionLoop& loop = section.loops[i];
            Box3 loopBox;
            for (const Vec3& p : loop.points) loopBox.include(p);
            bounds.include(loopBox);
            std::printf("  [%zu] %s, %zu pontos, perimetro %s, caixa %s x %s x %s\n", i, loop.closed ? "fechado" : "aberto",
                loop.points.size(), formatLength(loop.length).c_str(), formatLength(loopBox.size().x).c_str(),
                formatLength(loopBox.size().y).c_str(), formatLength(loopBox.size().z).c_str());
            // Os limites no espaco do modelo (para comparar com as referencias).
            std::printf("       x [%.3f, %.3f]  y [%.3f, %.3f]  z [%.3f, %.3f]\n", loopBox.minimum.x, loopBox.maximum.x,
                loopBox.minimum.y, loopBox.maximum.y, loopBox.minimum.z, loopBox.maximum.z);
            for (std::size_t p = 0; p + 1 < loop.points.size(); ++p) contour.segments.push_back({ loop.points[p], loop.points[p + 1] });
            if (loop.closed && loop.points.size() > 1) contour.segments.push_back({ loop.points.back(), loop.points.front() });
        }
        loops += section.loops.size();
        total += section.totalLength();
        highlights.push_back(std::move(contour));
    }
    std::cout << "Corte: " << label << (context.poseLabel.empty() ? "" : " na pose " + context.poseLabel) << " - " << loops
              << " contorno(s), total " << formatLength(total) << '\n';
    if (!bounds.valid()) return 0;
    if (hasMarker) highlights.front().points.push_back(marker);
    // --alvo: a vista de frente so em volta do ponto.
    Box3 framed = bounds.expanded(0.02f);
    if (line.has("alvo")) {
        const std::vector<std::string> axes = line.list("alvo");
        if (axes.size() != 3) {
            std::cerr << "--alvo espera x,y,z (metros, espaco do modelo)\n";
            return 2;
        }
        Vec3 target {};
        float* values[] = { &target.x, &target.y, &target.z };
        for (std::size_t i = 0; i < 3; ++i) *values[i] = std::strtof(axes[i].c_str(), nullptr);
        framed = Box3 {};
        framed.include(target, std::max(0.005f, line.number("raio", 0.1f)));
    }
    InspectionScene scene = context.scene();
    scene.highlights = highlights;
    const int width = line.integer("largura", 900), height = line.integer("altura-imagem", 760);

    // De frente para o plano: o contorno sobre a malha em raio-x.
    OrbitCamera flat = lookingAlong(-plane.normal);
    flat.orthographic = true;
    frameBox(flat, framed, static_cast<float>(width) / static_cast<float>(height), 1.25f);
    InspectionStyle flatStyle;
    flatStyle.surface = SurfaceMode::XRay;
    flatStyle.grid = false;
    flatStyle.supersample = 3;
    flatStyle.title = "Corte · " + label;
    flatStyle.notes.push_back(std::to_string(loops) + " contorno(s), " + formatLength(total));
    if (!context.poseLabel.empty()) flatStyle.notes.push_back("pose: " + context.poseLabel);
    const Image front = renderInspection(scene, flatStyle, flat, width, height);

    // No corpo, em 3/4.
    OrbitCamera body = frameBounds(bounds.expanded(0.15f), 30.0f, 20.0f, static_cast<float>(width) / static_cast<float>(height));
    InspectionStyle bodyStyle;
    bodyStyle.surface = SurfaceMode::Solid;
    bodyStyle.supersample = 2;
    bodyStyle.title = "Corte no corpo · " + label;
    const Image placed = renderInspection(scene, bodyStyle, body, width, height);

    const std::vector<Image> images { front, placed };
    const std::vector<std::string> captions { "de frente para o plano (raio-x)", "no corpo" };
    return saveImage(composeSheet(images, captions, 2, "Corte · " + label), context.output(line, "corte.png")) ? 0 : 1;
}

} // namespace MatterEngine::ModelInspector
