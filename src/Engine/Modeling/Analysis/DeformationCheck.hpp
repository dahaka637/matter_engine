#pragma once

#include "Engine/Modeling/Analysis/Report.hpp"
#include "Engine/Modeling/Mesh/Mesh.hpp"
#include "Engine/Modeling/Skeleton/BodyRegions.hpp"

#include <span>
#include <string>

namespace MatterEngine::Modeling {

struct PoseSample {
    std::string name;
    SkeletonPose pose;
};

struct DeformationSettings {
    // Triangulo que cresce mais que isso (area) esticou demais; menor que
    // compressLimit amassou (o "embrulho de bala" nas juntas).
    float stretchLimit = 1.8f;
    float compressLimit = 0.35f;
    // Partes que passam a se cruzar na pose.
    bool intersections = true;
};

// A malha deformada pelo skin em cada pose: estiramento, amassado,
// triangulos virados e partes que se atravessam. Virado a vista e erro
// ("virou_pose"); virado escondido dentro de uma prega (nenhum olhar de fora
// chega nele) e observacao ("prega_pose"). Virado: a normal contra a que o
// skin pede, ou dobrada contra a de um vizinho com o verso aparecendo de fora
// (a mistura dos giros vira a normal esperada junto; o verso que ja aparece
// na ligacao, numa borda aberta, nao conta).
void checkDeformation(const Mesh& mesh, const Skeleton& skeleton, std::span<const PoseSample> poses, Report& report,
    const DeformationSettings& settings = {});

} // namespace MatterEngine::Modeling
