#include "Commands.hpp"

#include "Engine/Modeling/Raster/Rasterizer.hpp"

#include <iostream>
#include <unordered_set>

namespace MatterEngine::ModelInspector {

using namespace Modeling;

// O atlas de textura da malha num PNG: --ilhas desenha as arestas do mapa por
// cima (o layout); --fundo pinta o que esta fora das ilhas de cinza.
int runTexture(const ModelContext& context, const CommandLine& line) {
    const Mesh& mesh = context.model().mesh;
    if (mesh.textures.empty() || !mesh.hasUv()) {
        std::cerr << "a malha nao tem textura (montagem com \"textured\": false?)\n";
        return 2;
    }
    Image image = mesh.textures.front().image;
    if (line.has("fundo")) {
        Image backed(image.width(), image.height(), rgb(70, 72, 78));
        backed.drawImage(image, 0, 0);
        image = std::move(backed);
    }
    if (line.has("ilhas")) {
        const float w = static_cast<float>(image.width()), h = static_cast<float>(image.height());
        std::unordered_set<std::uint64_t> drawn;
        for (std::size_t t = 0; t < mesh.triangles.size(); ++t) {
            const std::array<Uv, 3>& uv = mesh.triangleUv[t];
            for (int i = 0; i < 3; ++i) {
                const Uv a = uv[static_cast<std::size_t>(i)], b = uv[static_cast<std::size_t>((i + 1) % 3)];
                drawLine2D(image, a.u * w, a.v * h, b.u * w, b.v * h, 1.0f, rgb(40, 120, 255, 0.85f));
            }
        }
    }
    std::cout << "atlas: " << mesh.textures.front().image.width() << " texels, " << mesh.triangleUv.size() << " triangulos mapeados\n";
    return saveImage(image, context.output(line, "textura.png")) ? 0 : 1;
}

} // namespace MatterEngine::ModelInspector
