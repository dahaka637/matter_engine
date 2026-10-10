#pragma once

#include "Engine/Modeling/Raster/Image.hpp"

#include <memory>
#include <string>
#include <string_view>

namespace MatterEngine::Modeling {

struct TextSize {
    float width = 0.0f;
    float height = 0.0f;
};

// Texto UTF-8 (com acentos) rasterizado de uma fonte TrueType, para os
// rotulos das imagens. Sem fonte carregada nada e desenhado.
class TextRenderer {
public:
    TextRenderer();
    ~TextRenderer();
    TextRenderer(const TextRenderer&) = delete;
    TextRenderer& operator=(const TextRenderer&) = delete;

    bool load(const std::string& fontPath);
    [[nodiscard]] bool loaded() const;

    [[nodiscard]] TextSize measure(std::string_view text, float pixelHeight) const;
    // (x, y) e o canto de cima a esquerda do texto.
    void draw(Image& image, float x, float y, std::string_view text, float pixelHeight, Color color) const;
    // Com contorno: legivel sobre qualquer fundo.
    void drawOutlined(Image& image, float x, float y, std::string_view text, float pixelHeight, Color color,
        Color outline) const;
    // Caixa de fundo com o texto dentro; devolve o tamanho da caixa.
    TextSize drawLabel(Image& image, float x, float y, std::string_view text, float pixelHeight, Color color,
        Color background, float padding) const;

    // A fonte padrao das ferramentas (a Roboto dos assets), carregada uma vez.
    [[nodiscard]] static const TextRenderer& standard();

private:
    struct Impl;
    std::unique_ptr<Impl> m_impl;
};

} // namespace MatterEngine::Modeling
