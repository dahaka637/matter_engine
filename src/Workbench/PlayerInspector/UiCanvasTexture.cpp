#include "Workbench/PlayerInspector/UiCanvasTexture.hpp"

#include <span>

namespace MatterEngine::Workbench {

std::uint64_t UiCanvasTexture::present(const Modeling::Image& image, bool changed) {
    if (image.empty()) return m_texture.id;
    const auto bytes = std::as_bytes(image.pixels());
    if (m_texture && (m_texture.width != image.width() || m_texture.height != image.height())) release();
    if (!m_texture) {
        m_texture = m_renderer.createUiTexture(image.width(), image.height(), bytes);
        return m_texture.id;
    }
    if (changed) m_renderer.updateUiTexture(m_texture, bytes);
    return m_texture.id;
}

void UiCanvasTexture::release() {
    if (m_texture) m_renderer.destroyUiTexture(m_texture);
}

} // namespace MatterEngine::Workbench
