#include "Commands.hpp"

#include "Engine/Modeling/Analysis/Report.hpp"

#include <cstdio>
#include <iostream>

namespace MatterEngine::ModelInspector {

using namespace Modeling;

int runInfo(const ModelContext& context, const CommandLine& line) {
    const InspectionModel& model = context.model();
    const Skeleton& skeleton = model.skeleton;
    std::cout << "Modelo: " << model.source << '\n';
    std::cout << "Manifesto: " << (context.loaded.manifest.directory / "model.json").string() << '\n';
    std::cout << "Altura do esqueleto (chao ao alto da cabeca): " << formatLength(skeleton.height())
              << "  chao em z = " << formatNumber(skeleton.groundHeight, 4) << " m (espaco do modelo: z cima, x frente,"
              << " y esquerda)\n";

    std::cout << "\nJuntas (" << skeleton.joints.size() << "):\n";
    std::printf("  %-3s %-16s %-14s %-11s %-9s %-20s %-26s %s\n", "#", "nome", "pai", "tipo", "lado", "regiao",
        "posicao (x; y; z) m", "osso");
    for (std::size_t j = 0; j < skeleton.joints.size(); ++j) {
        const Joint& joint = skeleton.joints[j];
        if (!line.has("dedos") && (joint.kind == JointKind::Finger || (joint.kind == JointKind::Tip
                && joint.parent >= 0 && skeleton.joints[static_cast<std::size_t>(joint.parent)].kind == JointKind::Finger)))
            continue;
        const int region = j < model.jointRegion.size() ? model.jointRegion[j] : -1;
        const std::string regionId = region >= 0 ? model.regions.regions[static_cast<std::size_t>(region)].id : "-";
        const std::string parent = joint.parent >= 0 ? skeleton.joints[static_cast<std::size_t>(joint.parent)].name : "-";
        char position[64];
        std::snprintf(position, sizeof(position), "%+.3f; %+.3f; %+.3f", static_cast<double>(joint.bind.position.x),
            static_cast<double>(joint.bind.position.y), static_cast<double>(joint.bind.position.z));
        std::printf("  %-3zu %-16s %-14s %-11s %-9s %-20s %-26s %s\n", j, joint.name.c_str(), parent.c_str(),
            std::string(jointKindName(joint.kind)).c_str(), std::string(sideName(joint.side)).c_str(), regionId.c_str(),
            position, formatLength(skeleton.boneLength(static_cast<int>(j))).c_str());
    }
    if (!line.has("dedos")) std::cout << "  (dedos das maos ocultos: --dedos mostra os 30 e as pontas)\n";

    std::cout << "\nRegioes (" << model.regions.regions.size() << "):\n";
    for (std::size_t r = 0; r < model.regions.regions.size(); ++r) {
        const BodyRegion& region = model.regions.regions[r];
        std::size_t joints = 0;
        for (const int owner : model.jointRegion) joints += owner == static_cast<int>(r) ? 1 : 0;
        std::printf("  %-20s %-22s juntas %-3zu angulo %+.0f/%+.0f\n", region.id.c_str(), region.label.c_str(),
            region.overview ? skeleton.joints.size() : joints, static_cast<double>(region.yawDegrees),
            static_cast<double>(region.pitchDegrees));
    }
    for (const std::string& problem : model.regions.validate(skeleton)) std::cout << "  PROBLEMA: " << problem << '\n';

    std::cout << "\nMalha: ";
    if (model.mesh.empty()) {
        std::cout << "nenhuma (so ossos)\n";
    } else {
        std::cout << model.mesh.positions.size() << " vertices, " << model.mesh.triangles.size() << " triangulos, "
                  << model.mesh.parts.size() << " partes, " << model.mesh.materials.size() << " materiais"
                  << (model.mesh.hasSkin() ? ", com skin" : ", sem skin") << '\n';
        for (std::size_t p = 0; p < model.mesh.parts.size(); ++p)
            std::cout << "  " << model.mesh.parts[p].name << " (regiao " << model.mesh.parts[p].region << "): "
                      << model.mesh.partTriangleCount(static_cast<int>(p)) << " triangulos\n";
    }

    if (!context.loaded.timings.empty()) {
        double total = 0.0;
        for (const auto& [stage, ms] : context.loaded.timings) total += ms;
        std::printf("\nCarga: %.0f ms\n", total);
        for (const auto& [stage, ms] : context.loaded.timings) std::printf("  %-34s %7.1f ms\n", stage.c_str(), ms);
    }

    std::cout << "\nReferencias:\n";
    for (const ReferenceImage& reference : context.loaded.manifest.references)
        std::cout << "  [" << reference.view << "] " << reference.label << ": " << reference.path.string() << '\n';

    if (line.has("animacoes")) {
        std::cout << "\nAnimacoes (--pose <id>@<s>):\n ";
        for (const std::string& id : availableClipIds()) std::cout << ' ' << id;
        std::cout << '\n';
    } else {
        std::cout << "\n(--animacoes lista as animacoes para --pose)\n";
    }
    return 0;
}

} // namespace MatterEngine::ModelInspector
