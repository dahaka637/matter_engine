#pragma once

#include "Engine/RHI/RHIDevice.hpp"
#include "Engine/Render/Scene3D.hpp"
#include <array>
#include <memory>
#include <string>

struct ImDrawData;
struct SDL_Window;

namespace MatterEngine::RHI::Vulkan {

class VulkanDevice final : public Device {
public:
    VulkanDevice();
    ~VulkanDevice() override;

    VulkanDevice(const VulkanDevice&) = delete;
    VulkanDevice& operator=(const VulkanDevice&) = delete;

    void initialize(SDL_Window* window, bool vsync, const std::string& applicationName);
    void shutdown();

    BufferHandle createBuffer(const BufferDesc& desc) override;
    ShaderHandle createShader(const ShaderDesc& desc) override;
    PipelineHandle createGraphicsPipeline(const GraphicsPipelineDesc& desc) override;
    TextureHandle createRenderTarget(const TextureDesc& desc) override;
    TextureHandle createTexture2D(Extent2D extent, std::span<const std::byte> rgbaPixels) override;
    void destroyBuffer(BufferHandle handle) override;
    void destroyShader(ShaderHandle handle) override;
    void destroyPipeline(PipelineHandle handle) override;
    void destroyTexture(TextureHandle handle) override;
    void writeBuffer(BufferHandle handle, std::size_t offset, std::span<const std::byte> data) override;

    FrameStatus beginFrame() override;
    CommandList& commandList() override;
    void beginRenderTargetPass(TextureHandle target, ClearColor clearColor) override;
    void endRenderTargetPass() override;
    void blitToSwapchain(TextureHandle source, ClearColor clearColor) override;
    void drawWorldSprite(TextureHandle source, const std::array<float, 8>& corners,
        const std::array<float, 4>& perspectiveDepths) override;
    void endFrame() override;
    void requestSwapchainRebuild() override;
    void waitIdle() override;
    [[nodiscard]] Extent2D drawableExtent() const override;
    [[nodiscard]] std::uint32_t currentFrameSlot() const override;
    [[nodiscard]] const char* backendName() const override;
    [[nodiscard]] FramePerformanceMetrics
        framePerformanceMetrics() const override;

    // Backend-specific implementation used only by Engine/Render/ImGuiLayer.
    void initializeImGui(SDL_Window* window);
    void shutdownImGui();
    void beginImGuiFrame();
    void renderImGui(ImDrawData* drawData);
    [[nodiscard]] std::uint64_t createImGuiTexture(Extent2D extent, std::span<const std::byte> rgbaPixels);

    // Records a depth-only light pass followed by a depth-tested color pass.
    // Vulkan resources stay private to this backend; Renderer exposes only
    // the resulting backend-neutral UiTexture to the game/editor layer.
    [[nodiscard]] std::uint64_t renderScene3D(const Scene3DFrame& scene, Extent2D extent);
    // Records the 3D color pass directly into the acquired swapchain image
    // and leaves that pass open so Dear ImGui can be overlaid afterward.
    void renderScene3DToSwapchain(const Scene3DFrame& scene);

private:
    class Impl;
    std::unique_ptr<Impl> m_impl;
};

} // namespace MatterEngine::RHI::Vulkan
