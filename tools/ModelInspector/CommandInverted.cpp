#include "Commands.hpp"

#include "Engine/Modeling/Analysis/DeformationCheck.hpp"
#include "Engine/Modeling/Assembly/AssemblyRecipe.hpp"
#include "Engine/Modeling/Geometry/Parallel.hpp"
#include "Engine/Modeling/Rig/GameRig.hpp"
#include "Engine/Modeling/Rig/PlayerModelAsset.hpp"

#include <algorithm>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <map>
#include <memory>
#include <sstream>

namespace MatterEngine::ModelInspector {

using namespace Modeling;

// O avesso a vista nas poses do jogo: todas as animacoes ("--instantes" de
// cada, a mao solta e o punho) pela checagem de deformacao. Cada triangulo
// virado a vista, em quantas poses, a primeira e onde ele fica na ligacao; e
// os pontos do weightRelax da montagem (o centro dele levado a metade
// direita, so na parte dele; os da receita junto, os perto um do outro -
// menos que "--raio" - num so).
int runInverted(const ModelContext& context, const CommandLine& line) {
    const InspectionModel& model = context.model();
    if (model.mesh.empty() || !model.mesh.hasSkin() || !context.loaded.character || !context.loaded.rig) {
        std::cerr << "precisa da malha com skin e do jogo (personagem e esqueleto)\n";
        return 1;
    }
    const int instants = std::max(1, line.integer("instantes", 6));
    const float radius = line.number("raio", 0.02f);
    struct Hit {
        std::size_t poses = 0;
        std::string first;
    };
    std::map<std::uint32_t, Hit> hits;
    DeformationSettings settings;
    settings.intersections = false;
    // As poses (cada clipe em "instants" instantes, com a mao solta e o
    // punho), amostradas e checadas em paralelo; os achados juntados na ordem.
    struct Job {
        std::shared_ptr<const AnimationClip3D> clip;
        std::string id;
        float time = 0.0f;
        RigHandPose hand = RigHandPose::Relaxed;
    };
    std::vector<Job> jobs;
    for (const std::string& id : availableClipIds()) {
        std::string error;
        std::optional<AnimationClip3D> clip = loadCharacterClip(id, &error);
        if (!clip) continue;
        const auto shared = std::make_shared<const AnimationClip3D>(std::move(*clip));
        for (int k = 0; k < instants; ++k)
            for (const RigHandPose hand : { RigHandPose::Relaxed, RigHandPose::Fist })
                jobs.push_back({ shared, id, shared->durationSeconds * static_cast<float>(k) / static_cast<float>(instants), hand });
    }
    std::vector<std::string> names(jobs.size());
    std::vector<Report> reports(jobs.size());
    parallelFor(jobs.size(), 1, [&](std::size_t first, std::size_t last) {
        for (std::size_t i = first; i < last; ++i) {
            const Job& job = jobs[i];
            char label[160];
            std::snprintf(label, sizeof label, "%s@%.2f%s", job.id.c_str(), job.time, job.hand == RigHandPose::Fist ? "#punho" : "");
            names[i] = label;
            const PoseSample sample { label, sampleGameRigPose(*context.loaded.character, *context.loaded.rig, job.clip.get(), job.time, job.hand, true) };
            checkDeformation(model.mesh, model.skeleton, std::span<const PoseSample>(&sample, 1), reports[i], settings);
        }
    });
    for (std::size_t i = 0; i < jobs.size(); ++i)
        for (const Finding& finding : reports[i].findings())
            if (finding.code == "virou_pose")
                for (const std::uint32_t t : finding.triangles) {
                    Hit& hit = hits[t];
                    if (hit.poses++ == 0) hit.first = names[i];
                }
    std::printf("Poses do jogo: %zu; triangulos do avesso a vista: %zu\n", jobs.size(), hits.size());
    // Os pontos da receita e os novos (na ligacao, na metade direita).
    std::vector<WeightRelaxPoint> points;
    int iterations = 2;
    if (!context.loaded.manifest.assemblyPath.empty()) {
        std::ifstream file(context.loaded.manifest.assemblyPath);
        std::stringstream text;
        text << file.rdbuf();
        std::string error;
        if (const std::optional<AssemblyRecipe> recipe = parseAssemblyRecipe(text.str(), &error)) {
            points = recipe->weightRelax;
            iterations = recipe->weightRelaxIterations;
        }
    }
    const std::size_t known = points.size();
    for (const auto& [t, hit] : hits) {
        const Triangle& tri = model.mesh.triangles[t];
        const Vec3 center = (model.mesh.positions[tri[0]] + model.mesh.positions[tri[1]] + model.mesh.positions[tri[2]]) / 3.0f;
        const Vec3 right { center.x, -std::abs(center.y), center.z };
        std::printf("  T%-5u %-26s %3zu poses (%s) na ligacao (%+.4f %+.4f %+.4f)\n", t,
            model.mesh.parts[static_cast<std::size_t>(model.mesh.partOf(t))].name.c_str(), hit.poses, hit.first.c_str(), center.x, center.y,
            center.z);
        const bool near = std::any_of(points.begin(), points.end(), [&](const WeightRelaxPoint& p) { return (p.at - right).length() < radius; });
        if (!near) points.push_back({ right, radius, model.mesh.parts[static_cast<std::size_t>(model.mesh.partOf(t))].name });
    }
    if (points.size() == known) {
        std::printf("Nada novo para o weightRelax (%zu pontos na receita).\n", known);
        return hits.empty() ? 0 : 1;
    }
    std::printf("\nweightRelax (%zu da receita, %zu novos):\n  \"weightRelax\": { \"iterations\": %d, \"points\": [\n", known, points.size() - known,
        iterations);
    for (std::size_t i = 0; i < points.size(); ++i)
        std::printf("    { \"at\": [%.4f, %.4f, %.4f], \"radius\": %.3f%s%s%s }%s\n", points[i].at.x, points[i].at.y, points[i].at.z,
            points[i].radius, points[i].part.empty() ? "" : ", \"part\": \"", points[i].part.c_str(), points[i].part.empty() ? "" : "\"",
            i + 1 < points.size() ? "," : "");
    std::printf("  ] }\n");
    return 1;
}

} // namespace MatterEngine::ModelInspector
