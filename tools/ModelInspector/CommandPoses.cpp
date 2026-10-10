#include "Commands.hpp"

#include "Engine/Modeling/Analysis/DeformationCheck.hpp"
#include "Engine/Modeling/Inspection/ContactSheet.hpp"
#include "Engine/Modeling/Inspection/ViewFraming.hpp"

#include <iostream>

namespace MatterEngine::ModelInspector {

using namespace Modeling;

// O modelo em poses das animacoes do jogo (folha) e a deformacao da malha
// nelas (estica, amassa, vira, cruza).
//   --poses repouso,jog@0.25,sprint@0.3   (padrao: um conjunto variado)
int runPoses(const ModelContext& context, const CommandLine& line) {
    std::vector<std::string> specs = line.list("poses");
    if (specs.empty())
        specs = { "repouso", "jog@0.25", "sprint@0.3", "jump_standing@0.45", "tackle_slide@0.5",
            "stand_up_front_kimodo@1.2" };
    const InspectionModel& model = context.model();
    const int width = line.integer("largura", 520), height = line.integer("altura", 620);
    const std::string modeId = line.text("modo", model.mesh.empty() ? "ossos" : "solido");
    const std::optional<SurfaceMode> mode = parseSurfaceMode(modeId);
    if (!mode) {
        std::cerr << "modo desconhecido: " << modeId << '\n';
        return 2;
    }
    const ViewPreset* view = findViewPreset(line.text("vista", "34"));
    std::vector<PoseSample> samples;
    std::vector<Image> images;
    std::vector<std::string> captions;
    for (const std::string& spec : specs) {
        std::string error;
        const std::optional<SkeletonPose> pose = poseFromSpec(context.loaded, spec, &error);
        if (!pose) {
            std::cerr << error << '\n';
            return 2;
        }
        samples.push_back({ spec, *pose });
        std::vector<Vec3> posed;
        if (model.mesh.hasSkin()) posed = model.mesh.posedPositions(skinningTransforms(model.skeleton, *pose));
        InspectionScene scene = context.scene();
        scene.pose = &samples.back().pose;
        scene.meshPositions = posed;
        InspectionStyle style;
        style.surface = *mode;
        style.supersample = 2;
        style.hud = false;
        const OrbitCamera camera = frameRegion(scene, 0, static_cast<float>(width) / static_cast<float>(height), view);
        images.push_back(renderInspection(scene, style, camera, width, height));
        captions.push_back(spec);
    }
    if (!model.mesh.empty()) {
        Report report;
        checkDeformation(model.mesh, model.skeleton, samples, report);
        report.sort();
        std::cout << report.text(static_cast<std::size_t>(std::max(1, line.integer("max", 60))));
    }
    return saveImage(composeSheet(images, captions, std::min<int>(3, static_cast<int>(images.size())),
                         "Poses · " + model.source),
               context.output(line, "poses.png"))
        ? 0
        : 1;
}

} // namespace MatterEngine::ModelInspector
