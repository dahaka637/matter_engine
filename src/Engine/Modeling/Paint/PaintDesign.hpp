#pragma once

#include "Engine/Modeling/Paint/PaintLayer.hpp"
#include "Engine/Modeling/Paint/PaintSpace.hpp"

#include <filesystem>
#include <map>
#include <optional>
#include <string>

// O desenho da pintura em dados (pintura.json): as cores com nome, a cor de
// base de cada parte e as camadas, de baixo para cima. Cada camada e uma
// forma num espaco (PaintSpace) com a tinta dela:
//
//   "layers": [ { "name": "faixa do lado", "space": "tronco",
//                 "shape": { "type": "rectangle", "from": [-0.04, "barra"], "to": [0.04, "axila"] },
//                 "color": "preto" } ]
//
// Formas: "polygon" (points), "rectangle" (from, to, chamfer, round),
// "ellipse" (center, radius), "line" (points, width, closed) e "text" (text,
// font: "block" ou um .ttf, height, at, align: center/left/right, vertical:
// middle/baseline, spacing, slant em graus, stretch) e "path" (um caminho de
// SVG: "d", ou "file" um .svg e "id" do <path>; o quadro - "viewBox" ou o do
// arquivo, senao a caixa do caminho - centrado em "at" com a altura "height",
// stretch). Todas aceitam "rotate" (graus, em volta de "pivot" ou do centro).
// Tinta: "color" ([r, g, b(, a)] 0-255, "#rrggbb" ou um nome de "colors"),
// "gradient" (linear: from, to; radial: center, radius, angle; e stops
// [[0, cor], [1, cor]]) ou "image" (file: um PNG; "from" e "to", ou "at" e
// "height" com a largura pela proporcao, stretch; rotate - sem "shape", a
// forma e a caixa da imagem). Mais:
// "opacity", "blend" (normal/multiply), "outline" {width, color} e "sides"
// (both/right/left). Uma coordenada e um numero ou uma ancora do espaco com
// deslocamento ("axila", "barra-0.01").
namespace MatterEngine::Modeling {

struct PaintDesign {
    std::map<std::string, Color> partBase;
    std::vector<PaintLayer> layers;
    // Os arquivos lidos (o desenho, as fontes, os SVG e as imagens): quem
    // mostra o modelo recarrega quando um deles muda.
    std::vector<std::filesystem::path> files;
};

// "spaces": os espacos que o desenho pode usar (as ancoras deles);
// "directory": onde ficam as fontes TrueType, os SVG e as imagens citadas.
[[nodiscard]] std::optional<PaintDesign> parsePaintDesign(const std::string& text, const std::map<std::string, PaintSpace>& spaces,
    const std::filesystem::path& directory, std::string* error);
[[nodiscard]] std::optional<PaintDesign> loadPaintDesign(const std::filesystem::path& file, const std::map<std::string, PaintSpace>& spaces,
    std::string* error);

} // namespace MatterEngine::Modeling
