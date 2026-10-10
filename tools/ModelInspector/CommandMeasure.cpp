#include "Commands.hpp"

#include "Engine/Modeling/Analysis/CrossSection.hpp"
#include "Engine/Modeling/Inspection/ViewFraming.hpp"

#include <cmath>
#include <cstdio>
#include <iostream>

namespace MatterEngine::ModelInspector {

using namespace Modeling;

namespace {

void printDistance(const Skeleton& skeleton, const char* label, const char* from, const char* to) {
    const int a = skeleton.find(from), b = skeleton.find(to);
    if (a < 0 || b < 0) return;
    std::printf("  %-34s %s\n", label,
        formatLength(distance(skeleton.joints[static_cast<std::size_t>(a)].bind.position,
                         skeleton.joints[static_cast<std::size_t>(b)].bind.position))
            .c_str());
}

void printHeight(const Skeleton& skeleton, const char* label, const char* joint) {
    const int j = skeleton.find(joint);
    if (j < 0) return;
    std::printf("  %-34s %s\n", label,
        formatLength(skeleton.joints[static_cast<std::size_t>(j)].bind.position.z - skeleton.groundHeight).c_str());
}

void printSection(const Mesh& mesh, const Skeleton& skeleton, int joint, float t) {
    const BoneSection section = sectionAcrossBone(mesh, mesh.positions, skeleton, joint, t);
    const std::string& name = skeleton.joints[static_cast<std::size_t>(joint)].name;
    if (!section.found) {
        std::printf("  %-16s t=%.2f  sem corte\n", name.c_str(), static_cast<double>(t));
        return;
    }
    std::printf("  %-16s t=%.2f  circunferencia %-9s largura %-9s profundidade %-9s osso a %-8s do centro%s\n",
        name.c_str(), static_cast<double>(t), formatLength(section.circumference).c_str(),
        formatLength(section.width).c_str(), formatLength(section.depth).c_str(), formatLength(section.offset).c_str(),
        section.inside ? "" : "  (OSSO FORA DA MALHA)");
}

} // namespace

// Medidas do esqueleto (sempre) e da malha (cortes ao longo dos ossos).
//   --junta <nome> --t <0..1>  mede so um corte
int runMeasure(const ModelContext& context, const CommandLine& line) {
    const InspectionModel& model = context.model();
    const Skeleton& skeleton = model.skeleton;
    if (line.has("junta")) {
        const int joint = skeleton.find(line.text("junta"));
        if (joint < 0 || model.mesh.empty()) {
            std::cerr << (joint < 0 ? "junta desconhecida" : "sem malha para cortar") << '\n';
            return 2;
        }
        printSection(model.mesh, skeleton, joint, line.number("t", 0.5f));
        return 0;
    }
    std::cout << "Esqueleto (pose de ligacao):\n";
    std::printf("  %-34s %s\n", "altura (chao ao alto da cabeca)", formatLength(skeleton.height()).c_str());
    printHeight(skeleton, "altura dos ombros", "LeftUpperArm");
    printHeight(skeleton, "altura do quadril (junta da coxa)", "LeftThigh");
    printHeight(skeleton, "altura do joelho", "LeftShin");
    printHeight(skeleton, "altura do tornozelo", "LeftFoot");
    printHeight(skeleton, "altura da base do pescoco", "Neck");
    printDistance(skeleton, "largura dos ombros (juntas)", "LeftUpperArm", "RightUpperArm");
    printDistance(skeleton, "largura do quadril (juntas)", "LeftThigh", "RightThigh");
    printDistance(skeleton, "envergadura (ponta a ponta)", "LeftMiddleTip", "RightMiddleTip");
    printDistance(skeleton, "braco (ombro ao cotovelo)", "LeftUpperArm", "LeftForearm");
    printDistance(skeleton, "antebraco (cotovelo ao pulso)", "LeftForearm", "LeftHand");
    printDistance(skeleton, "mao (pulso a ponta do medio)", "LeftHand", "LeftMiddleTip");
    printDistance(skeleton, "coxa (quadril ao joelho)", "LeftThigh", "LeftShin");
    printDistance(skeleton, "canela (joelho ao tornozelo)", "LeftShin", "LeftFoot");
    printDistance(skeleton, "pe (tornozelo a ponta)", "LeftFoot", "LeftToeTip");
    printDistance(skeleton, "pescoco (base a cabeca)", "Neck", "Head");
    printDistance(skeleton, "cabeca (junta ao alto)", "Head", "HeadTop");

    if (model.mesh.empty()) {
        std::cout << "\nMalha: nenhuma (as medidas de corte aparecem quando houver malha)\n";
        return 0;
    }
    const Box3 bounds = model.mesh.bounds();
    std::cout << "\nMalha:\n";
    std::printf("  %-34s %s\n", "altura (chao ao topo)", formatLength(bounds.maximum.z - skeleton.groundHeight).c_str());
    std::printf("  %-34s %s x %s x %s\n", "caixa (frente x lado x altura)", formatLength(bounds.size().x).c_str(),
        formatLength(bounds.size().y).c_str(), formatLength(bounds.size().z).c_str());
    std::printf("  %-34s %s\n", "area da superficie", (formatNumber(model.mesh.surfaceArea(), 3) + " m2").c_str());
    std::cout << "\nCortes perpendiculares aos ossos (t: 0 na junta, 1 na filha):\n";
    const char* joints[] = { "Pelvis", "Abdomen", "Chest", "UpperChest", "Neck", "Head", "LeftUpperArm", "LeftForearm",
        "LeftHand", "LeftThigh", "LeftShin", "LeftFoot", "RightUpperArm", "RightForearm", "RightHand", "RightThigh",
        "RightShin", "RightFoot" };
    for (const char* name : joints) {
        const int joint = skeleton.find(name);
        if (joint < 0) continue;
        for (const float t : { 0.25f, 0.5f, 0.75f }) printSection(model.mesh, skeleton, joint, t);
    }
    return 0;
}

} // namespace MatterEngine::ModelInspector
