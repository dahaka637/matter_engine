#pragma once

#include "Engine/Render/Renderer.hpp"
#include "Workbench/PlayerInspector/CanvasTarget.hpp"

#include <cstdint>

namespace MatterEngine::Workbench {

// A textura de UI que recebe as imagens: criada no primeiro envio, com os
// pixels trocados quando o tamanho e o mesmo e recriada quando muda. Quem usa
// chama release() ao sair da tela (com o renderer vivo); no fim do programa
// o proprio renderer libera o que sobrou.
class UiCanvasTexture final : public CanvasTarget {
public:
    explicit UiCanvasTexture(Renderer& renderer) : m_renderer(renderer) {}
    ~UiCanvasTexture() override = default;
    UiCanvasTexture(const UiCanvasTexture&) = delete;
    UiCanvasTexture& operator=(const UiCanvasTexture&) = delete;

    std::uint64_t present(const Modeling::Image& image, bool changed) override;
    void release() override;

private:
    Renderer& m_renderer;
    UiTexture m_texture;
};

} // namespace MatterEngine::Workbench
