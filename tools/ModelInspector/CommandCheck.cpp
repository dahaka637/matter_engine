#include "Commands.hpp"

#include "Engine/Modeling/Analysis/ModelValidator.hpp"
#include "Engine/Modeling/Inspection/ContactSheet.hpp"
#include "Engine/Modeling/Inspection/ViewFraming.hpp"

#include <fstream>
#include <iostream>

namespace MatterEngine::ModelInspector {

using namespace Modeling;

namespace {

// Close-up de um achado: pelo lado onde ele esta, com ele destacado.
Image renderFinding(const ModelContext& context, const Finding& finding, std::size_t index, int width, int height) {
    InspectionScene scene = context.scene();
    const std::vector<Highlight> highlights { highlightFor(finding) };
    scene.highlights = highlights;
    Box3 box = finding.bounds();
    for (const std::uint32_t t : finding.triangles)
        if (t < context.model().mesh.triangles.size())
            for (const std::uint32_t v : context.model().mesh.triangles[t]) box.include(context.model().mesh.positions[v]);
    if (!box.valid()) box = regionBounds(scene, 0);
    box = box.expanded(std::max(0.03f, box.radius() * 0.35f));
    const float side = box.center().y;
    const float yaw = side > 0.05f ? 60.0f : side < -0.05f ? -60.0f : 20.0f;
    const OrbitCamera camera = frameBounds(box, yaw, 15.0f, static_cast<float>(width) / static_cast<float>(height), 1.25f);
    InspectionStyle style;
    style.surface = SurfaceMode::SolidWire;
    style.supersample = 2;
    style.title = "[" + std::to_string(index) + "] " + severityName(finding.severity) + " · " + finding.code;
    style.notes.push_back(finding.message);
    if (!finding.part.empty()) style.notes.push_back("parte: " + finding.part);
    return renderInspection(scene, style, camera, width, height);
}

} // namespace

int runCheck(const ModelContext& context, const CommandLine& line) {
    const InspectionModel& model = context.model();
    for (const std::string& problem : model.skeleton.validate()) std::cout << "ESQUELETO: " << problem << '\n';
    for (const std::string& problem : model.regions.validate(model.skeleton)) std::cout << "REGIOES: " << problem << '\n';

    ValidationOptions options;
    options.intersections = !line.has("sem-intersecoes");
    options.symmetry = !line.has("sem-simetria");
    options.symmetrySettings.tolerance = line.number("tolerancia-simetria", options.symmetrySettings.tolerance);
    std::vector<PoseSample> poses;
    for (const std::string& spec : line.list("poses")) {
        std::string error;
        const std::optional<SkeletonPose> pose = poseFromSpec(context.loaded, spec, &error);
        if (!pose) {
            std::cerr << error << '\n';
            return 2;
        }
        poses.push_back({ spec, *pose });
    }
    options.poses = poses;
    const Report report = validateModel(model.mesh, &model.skeleton, &model.regions, model.jointRegion, options);
    std::cout << report.text(static_cast<std::size_t>(std::max(1, line.integer("max", 200))));

    if (line.has("json")) {
        std::ofstream file(line.text("json"));
        file << report.json() << '\n';
        std::cout << "json: " << line.text("json") << '\n';
    }
    const int imageCount = line.integer("imagens", 0);
    if (imageCount > 0 && !model.mesh.empty()) {
        std::vector<Image> images;
        std::vector<std::string> captions;
        for (std::size_t i = 0; i < report.findings().size() && static_cast<int>(images.size()) < imageCount; ++i) {
            const Finding& finding = report.findings()[i];
            if (!finding.hasLocation && finding.triangles.empty() && finding.points.empty()) continue;
            images.push_back(renderFinding(context, finding, i, line.integer("largura", 640), line.integer("altura", 520)));
            captions.push_back("[" + std::to_string(i) + "] " + finding.code + (finding.part.empty() ? "" : " · " + finding.part));
            saveImage(images.back(), context.outputDirectory / ("achado_" + std::to_string(i) + "_" + finding.code + ".png"));
        }
        if (!images.empty())
            saveImage(composeSheet(images, captions, std::min<int>(3, static_cast<int>(images.size())), "Achados da analise"),
                context.outputDirectory / "achados.png");
    }
    return report.count(Severity::Error) > 0 ? 1 : 0;
}

} // namespace MatterEngine::ModelInspector
