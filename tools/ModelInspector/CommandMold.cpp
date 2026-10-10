#include "Commands.hpp"

#include "Engine/Modeling/Assembly/AssemblyRecipe.hpp"
#include "Engine/Modeling/Assembly/PlayerAssembly.hpp"
#include "Engine/Modeling/Import/GltfImport.hpp"
#include "Engine/Modeling/Mesh/TriangleBvh.hpp"

#include <algorithm>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <sstream>

namespace MatterEngine::ModelInspector {

using namespace Modeling;

// O molde da cabeca de referencia (referenceFit da montagem): o alinhamento
// pelo cranio e o rosto e o quanto o queixo, a mandibula e o pescoco ficaram
// iguais a ela - a distancia de cada vertice movido a superficie da
// referencia (os inteiros, em cima dela; os da transicao, perto da gola e da
// linha da mandibula, no caminho).
int runMold(const ModelContext& context, const CommandLine& line) {
    (void)line;
    const PlayerModelManifest& manifest = context.loaded.manifest;
    if (manifest.assemblyPath.empty()) {
        std::cerr << "o modelo nao e montado (sem montagem.json)\n";
        return 1;
    }
    std::ifstream file(manifest.assemblyPath);
    std::stringstream text;
    text << file.rdbuf();
    std::string error;
    const std::optional<AssemblyRecipe> recipe = parseAssemblyRecipe(text.str(), &error);
    if (!recipe) {
        std::cerr << "receita: " << error << '\n';
        return 1;
    }
    if (recipe->referenceFit.file.empty()) {
        std::cout << "a receita nao usa molde (referenceFit)\n";
        return 0;
    }
    const std::filesystem::path directory = manifest.assemblyPath.parent_path();
    const AssemblyResult assembled = assemblePlayer(*recipe, directory, context.loaded.rig->skeleton);
    if (!assembled.ok) {
        std::cerr << "montagem: " << assembled.error << '\n';
        return 1;
    }
    const ReferenceFitReport& fit = assembled.referenceFit;
    GltfImportSettings settings;
    settings.weldTolerance = 1e-5f;
    const GltfImportResult reference = importGltfMesh(directory / recipe->referenceFit.file, settings);
    if (!reference.ok) {
        std::cerr << "referencia: " << reference.error << '\n';
        return 1;
    }
    std::vector<Vec3> placed;
    placed.reserve(reference.mesh.positions.size());
    for (const Vec3& p : reference.mesh.positions) placed.push_back(fit.transform.apply(p));
    const TriangleBvh surface(placed, reference.mesh.triangles);
    std::size_t whole = 0, partial = 0;
    float wholeWorst = 0.0f, partialWorst = 0.0f;
    for (const ReferenceFitReport::Move& move : fit.moves) {
        const std::optional<TriangleBvh::Nearest> nearest = surface.nearest(move.to, 1.0f);
        const float gap = nearest ? nearest->distance : 1.0f;
        if (move.weight >= 0.999f) {
            ++whole;
            wholeWorst = std::max(wholeWorst, gap);
        } else {
            ++partial;
            partialWorst = std::max(partialWorst, gap);
        }
    }
    std::printf("Molde: %s (%zu vertices, %zu triangulos)\n", recipe->referenceFit.file.c_str(), reference.mesh.positions.size(),
        reference.mesh.triangles.size());
    std::printf("Alinhamento pelo cranio e o rosto: %zu pontos, erro medio %.2f mm, escala %.5f, inclinacao %+.2f graus\n",
        fit.alignmentPoints, fit.alignmentRms * 1000.0f, fit.scale, fit.pitchDegrees);
    std::printf("Cabeca recuada sobre o pescoco: %+.1f mm (headShift)\n", recipe->headShift * 1000.0f);
    std::printf("Vertices no molde (metade direita): %zu inteiros (o mais longe da referencia a %.2f mm), %zu na transicao "
                "(ate %.1f mm), o maior passo %.1f mm\n",
        whole, wholeWorst * 1000.0f, partial, partialWorst * 1000.0f, fit.largestMove * 1000.0f);
    return 0;
}

} // namespace MatterEngine::ModelInspector
