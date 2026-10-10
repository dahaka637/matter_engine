#pragma once

#include "Engine/Modeling/Paint/Shape2D.hpp"

#include <array>
#include <filesystem>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

// Os caminhos de SVG na pintura: um escudo ou um logo desenhado num editor
// vetorial (Inkscape, Illustrator) entra como forma, nitido em qualquer
// tamanho. O "d" de um <path> (M, L, H, V, C, S, Q, T, A, Z, absolutos e
// relativos) vira contornos: as curvas e os arcos em segmentos curtos (o
// desvio ate a curva fica abaixo de um milesimo do tamanho do desenho), cada
// subcaminho fechado. As coordenadas ficam as do SVG (y para baixo); o
// preenchimento e o do SVG padrao (a regra das voltas, "nonzero").
namespace MatterEngine::Modeling {

[[nodiscard]] std::optional<Shape2D> parseSvgPath(std::string_view d, std::string* error = nullptr);

// Um arquivo SVG simples: o viewBox do <svg> e o "d" de cada <path> (com o
// id dele, ou vazio). As transformacoes (transform=) nao sao lidas: no
// editor, "aplicar as transformacoes" antes de salvar.
struct SvgDocument {
    std::optional<std::array<float, 4>> viewBox;
    std::vector<std::pair<std::string, std::string>> paths;
};
[[nodiscard]] std::optional<SvgDocument> parseSvgDocument(std::string_view text, std::string* error = nullptr);
[[nodiscard]] std::optional<SvgDocument> readSvgDocument(const std::filesystem::path& file, std::string* error = nullptr);

} // namespace MatterEngine::Modeling
