#pragma once

#include "Engine/RHI/CharacterMaterial.hpp"
#include "Engine/RHI/RHICommandList.hpp"
#include <array>
#include <cstddef>
#include <span>

namespace MatterEngine::RHI {

class Device {
public:
    virtual ~Device() = default;

    virtual BufferHandle createBuffer(const BufferDesc& desc) = 0;
    virtual ShaderHandle createShader(const ShaderDesc& desc) = 0;
    virtual PipelineHandle createGraphicsPipeline(const GraphicsPipelineDesc& desc) = 0;
    virtual TextureHandle createRenderTarget(const TextureDesc& desc) = 0;
    // A real sampled asset texture (as opposed to createRenderTarget's
    // empty, write-only pixel-art target): uploads RGBA8 pixel data through
    // a staging buffer and leaves it shader-read-only, ready for a material
    // to sample. Used by GltfLoader and anything else bringing in authored
    // textures - not the game's per-frame render path.
    virtual TextureHandle createTexture2D(Extent2D extent, std::span<const std::byte> rgbaPixels) = 0;
    // O material do personagem (CharacterMaterial.hpp): os arrays com os niveis
    // prontos e o sampler proprio, o buffer por vertice e os parametros. A troca
    // de uma camada (um uniforme novo) e a dos vertices (outro corpo) respeitam
    // os quadros em voo, sem esperar o dispositivo inteiro.
    virtual CharacterMaterialHandle createCharacterMaterial(const CharacterMaterialDesc& desc) = 0;
    virtual void updateCharacterKitLayer(CharacterMaterialHandle handle, std::uint32_t layer, std::span<const std::byte> levels,
        std::span<const float> regionAverages) = 0;
    virtual void updateCharacterVertices(CharacterMaterialHandle handle, std::span<const CharacterKitVertexGpu> vertices) = 0;
    virtual void destroyCharacterMaterial(CharacterMaterialHandle handle) = 0;

    virtual void destroyBuffer(BufferHandle handle) = 0;
    virtual void destroyShader(ShaderHandle handle) = 0;
    virtual void destroyPipeline(PipelineHandle handle) = 0;
    virtual void destroyTexture(TextureHandle handle) = 0;
    virtual void writeBuffer(BufferHandle handle, std::size_t offset, std::span<const std::byte> data) = 0;

    virtual FrameStatus beginFrame() = 0;
    virtual CommandList& commandList() = 0;

    // Renders into `target` (an offscreen texture from createRenderTarget)
    // instead of the swapchain image - the game world draws here, at
    // whatever low, fixed resolution `target` was created with.
    virtual void beginRenderTargetPass(TextureHandle target, ClearColor clearColor) = 0;
    virtual void endRenderTargetPass() = 0;

    // Begins the swapchain's own rendering pass and immediately draws `source`
    // full-screen with nearest-neighbor sampling (the pixel-art upscale) -
    // the swapchain pass is left open afterward so further draws (ImGui, at
    // native resolution) can follow before endFrame().
    virtual void blitToSwapchain(TextureHandle source, ClearColor clearColor) = 0;

    // Draws a pre-baked static texture (e.g. the grass field) stretched
    // across 4 screen-space corners (top-left, top-right, bottom-right,
    // bottom-left) into the currently active render target pass - must be
    // called between beginRenderTargetPass/endRenderTargetPass. Exists so
    // static, expensive-to-generate world content only needs to be drawn
    // once (baked into a texture) and then reused every frame as a cheap
    // textured quad, instead of regenerating thousands of primitives per
    // frame.
    virtual void drawWorldSprite(TextureHandle source, const std::array<float, 8>& corners,
        const std::array<float, 4>& perspectiveDepths) = 0;

    virtual void endFrame() = 0;
    virtual void requestSwapchainRebuild() = 0;
    virtual void waitIdle() = 0;

    [[nodiscard]] virtual Extent2D drawableExtent() const = 0;
    [[nodiscard]] virtual std::uint32_t currentFrameSlot() const = 0;
    [[nodiscard]] virtual const char* backendName() const = 0;
    [[nodiscard]] virtual FramePerformanceMetrics
        framePerformanceMetrics() const = 0;
};

} // namespace MatterEngine::RHI
