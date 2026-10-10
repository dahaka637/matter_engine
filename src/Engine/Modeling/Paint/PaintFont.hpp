#pragma once

#include "Engine/Modeling/Paint/Shape2D.hpp"

#include <filesystem>
#include <map>
#include <optional>
#include <string>
#include <string_view>

// As letras da pintura como formas vetoriais (Shape2D): a fonte de bloco das
// camisas (os algarismos com chanfros de 45 graus; o "1" e o "0" medidos no
// "10" das costas da referencia) e qualquer fonte TrueType (os contornos das
// letras, as curvas em segmentos curtos) para nomes e textos. Altura das
// maiusculas e dos algarismos = 1, linha de base em y = 0, y para cima.
namespace MatterEngine::Modeling {

class PaintFont {
public:
    struct Glyph {
        Shape2D shape;
        float advance = 0.0f;
    };

    // A fonte de bloco das camisas (os algarismos 0 a 9).
    [[nodiscard]] static const PaintFont& block();
    // Uma fonte TrueType (.ttf/.otf): as letras de 32 a 255.
    [[nodiscard]] static std::optional<PaintFont> loadTrueType(const std::filesystem::path& path, std::string* error);

    [[nodiscard]] const Glyph* glyph(char32_t codepoint) const;
    // O texto numa linha: comeca em x = 0; "spacing" e o espaco extra entre
    // as letras (fracao da altura). A largura total em "width".
    [[nodiscard]] Shape2D layout(std::string_view utf8, float spacing, float* width = nullptr) const;

private:
    std::map<char32_t, Glyph> m_glyphs;
    // O ajuste de cada par (kerning), quando a fonte tem.
    std::map<std::pair<char32_t, char32_t>, float> m_kerning;
};

} // namespace MatterEngine::Modeling
