#include "Commands.hpp"

#include <algorithm>
#include <cstdio>
#include <iostream>
#include <set>
#include <sstream>

namespace MatterEngine::ModelInspector {

using namespace Modeling;

// Os pesos do skin dos vertices perto de um ponto (espaco do modelo, na
// ligacao): o indice, a posicao, as partes dos triangulos dele e cada junta
// com o peso. Para achar a causa de um avesso no lugar que o "avesso" mostra.
//   --perto x,y,z  --raio 0.02  --parte calcao (so os vertices dela)
//   --triangulos T2174,T2265 (os vertices desses triangulos, no lugar do --perto)
int runWeights(const ModelContext& context, const CommandLine& line) {
    const Mesh& mesh = context.model().mesh;
    const Skeleton& skeleton = context.model().skeleton;
    if (!mesh.hasSkin()) {
        std::cerr << "a malha nao tem skin\n";
        return 1;
    }
    Vec3 center {};
    {
        std::string text = line.text("perto", "0,0,0");
        std::replace(text.begin(), text.end(), ',', ' ');
        std::istringstream values(text);
        values >> center.x >> center.y >> center.z;
    }
    const float radius = line.number("raio", 0.02f);
    const int onlyPart = line.has("parte") ? mesh.findPart(line.text("parte")) : -1;
    std::vector<std::set<int>> parts(mesh.positions.size());
    for (std::size_t t = 0; t < mesh.triangles.size(); ++t)
        for (const std::uint32_t v : mesh.triangles[t]) parts[v].insert(mesh.partOf(t));
    // --triangulos 2174,2265: os vertices desses triangulos (os do "avesso").
    std::set<std::uint32_t> picked;
    for (const std::string& id : line.list("triangulos")) {
        const unsigned long t = std::stoul(id[0] == 'T' ? id.substr(1) : id);
        if (t < mesh.triangles.size())
            for (const std::uint32_t v : mesh.triangles[t]) picked.insert(v);
    }
    int shown = 0;
    for (std::uint32_t v = 0; v < mesh.positions.size(); ++v) {
        const Vec3 p = mesh.positions[v];
        if (picked.empty() ? (p - center).length() > radius : picked.count(v) == 0) continue;
        if (onlyPart >= 0 && parts[v].count(onlyPart) == 0) continue;
        std::printf("v%-6u (%+.4f %+.4f %+.4f)  [", v, p.x, p.y, p.z);
        bool first = true;
        for (const int part : parts[v]) {
            std::printf("%s%s", first ? "" : ",", part >= 0 && static_cast<std::size_t>(part) < mesh.parts.size() ? mesh.parts[static_cast<std::size_t>(part)].name.c_str() : "?");
            first = false;
        }
        std::printf("]  ");
        for (std::size_t i = 0; i < 4; ++i) {
            const float w = mesh.skin[v].weights[i];
            if (w <= 0.0f) continue;
            const std::uint16_t joint = mesh.skin[v].joints[i];
            std::printf("%s %.2f  ", joint < skeleton.joints.size() ? skeleton.joints[joint].name.c_str() : "?", w);
        }
        std::printf("\n");
        ++shown;
    }
    if (picked.empty()) std::printf("%d vertices a menos de %.3f m de (%+.4f %+.4f %+.4f)\n", shown, radius, center.x, center.y, center.z);
    return 0;
}

} // namespace MatterEngine::ModelInspector
