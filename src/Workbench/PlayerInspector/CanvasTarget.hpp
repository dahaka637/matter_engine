#pragma once

#include "Engine/Modeling/Raster/Image.hpp"

#include <cstdint>

namespace MatterEngine::Workbench {

// O que a tela de inspecao precisa da GPU: mostrar uma imagem gerada na CPU.
// A tela so conhece esta interface (a textura de verdade e a UiCanvasTexture;
// nos testes sem janela, uma falsa).
class CanvasTarget {
public:
    virtual ~CanvasTarget() = default;
    // Envia a imagem quando ela mudou e devolve o id da textura no ImGui.
    virtual std::uint64_t present(const Modeling::Image& image, bool changed) = 0;
    virtual void release() = 0;
};

} // namespace MatterEngine::Workbench
