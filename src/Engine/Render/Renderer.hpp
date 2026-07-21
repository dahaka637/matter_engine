#pragma once

#include "Engine/Platform/DisplayMode.hpp"
#include "Engine/RHI/RHITypes.hpp"
#include "Engine/Render/Render2D.hpp"
#include "Engine/Render/Scene3D.hpp"
#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <span>
#include <string>
#include <utility>
#include <vector>

struct SDL_Window;
union SDL_Event;

namespace MatterEngine::RHI::Vulkan {
class VulkanDevice;
}

namespace MatterEngine {

struct Color {
    std::uint8_t r = 0;
    std::uint8_t g = 0;
    std::uint8_t b = 0;
    std::uint8_t a = 255;
};

struct UiTexture {
    std::uint64_t id = 0;
    int width = 0;
    int height = 0;

    [[nodiscard]] bool valid() const { return id != 0 && width > 0 && height > 0; }
    explicit operator bool() const { return valid(); }
};

class Renderer {
public:
    Renderer();
    ~Renderer();

    Renderer(const Renderer&) = delete;
    Renderer& operator=(const Renderer&) = delete;

    void initialize(const std::string& title, int width, int height, bool vsync, DisplayMode mode = DisplayMode::Windowed);
    void shutdown();

    void processEvent(const SDL_Event& event);
    void beginFrame(Color clearColor);
    void beginGuiFrame();
    void endGuiFrame();
    void endFrame();

    // Switches window mode and/or size live, without recreating the window
    // or GL context. `width`/`height` are ignored in borderless fullscreen
    // (it always tracks the desktop resolution).
    void applyVideoSettings(int width, int height, DisplayMode mode);

    [[nodiscard]] SDL_Window* window() const;
    [[nodiscard]] int width() const;
    [[nodiscard]] int height() const;
    // The low, fixed-height virtual resolution the game world actually
    // renders at (pixel-art look + the performance headroom of shading far
    // fewer pixels than the window's real resolution) - width adapts to
    // match the window's aspect ratio so there's never any letterboxing.
    // World/camera/mouse math should use this, NOT width()/height(), which
    // stay physical window pixels for ImGui UI layout only.
    [[nodiscard]] int renderWidth() const;
    [[nodiscard]] int renderHeight() const;
    [[nodiscard]] DisplayMode displayMode() const;
    [[nodiscard]] Render2D& render2D();
    [[nodiscard]] const char* backendName() const;
    [[nodiscard]] RHI::FramePerformanceMetrics
        framePerformanceMetrics() const;
    [[nodiscard]] std::vector<std::pair<int, int>> availableFullscreenResolutions() const;
    [[nodiscard]] UiTexture loadUiTexture(const std::string& path);
    [[nodiscard]] UiTexture renderScene3D(const Scene3DFrame& scene, int width, int height);
    // Retorna true somente quando os comandos da cena foram realmente
    // gravados. Estado temporal do chamador nao deve avancar em frames
    // pulados por resize, minimizacao ou indisponibilidade da swapchain.
    [[nodiscard]] bool renderScene3DToScreen(const Scene3DFrame& scene);

    // Thin passthroughs to the RHI device, exposed so callers can build the
    // vertex/index buffers a MeshRender3D references (see MeshData3D).
    // Renderer stays the only game-facing surface onto GPU resources - Game
    // code never touches RHI::Device or the Vulkan backend directly.
    [[nodiscard]] RHI::BufferHandle createBuffer(const RHI::BufferDesc& desc);
    void writeBuffer(RHI::BufferHandle handle, std::size_t offset, std::span<const std::byte> data);
    void destroyBuffer(RHI::BufferHandle handle);
    // A real sampled asset texture (glTF materials, UI art, etc.), as
    // opposed to createBakedTexture's empty, write-only render target.
    [[nodiscard]] RHI::TextureHandle createTexture2D(RHI::Extent2D extent,
        std::span<const std::byte> rgbaPixels);
    void destroyTexture(RHI::TextureHandle handle);
    void setMouseCaptured(bool captured);
    [[nodiscard]] bool mouseCaptured() const { return m_mouseCaptured; }

    // Bakes static world content (e.g. the grass field) once into a texture
    // instead of regenerating thousands of primitives every frame: create a
    // texture, draw into it via render2D() between begin/endBakePass, then
    // every subsequent frame just drawWorldSprite() it - cheap regardless of
    // how much detail went into the one-time bake. drawWorldSprite must be
    // called between the main beginFrame/beginGuiFrame pair (i.e. while the
    // world render target pass is active).
    [[nodiscard]] RHI::TextureHandle createBakedTexture(int width, int height);
    void beginBakePass(RHI::TextureHandle target, int width, int height, Color clearColor);
    void endBakePass();
    void drawWorldSprite(RHI::TextureHandle source, const std::array<float, 8>& screenCorners,
        const std::array<float, 4>& perspectiveDepths = { 1.0f, 1.0f, 1.0f, 1.0f });

    // A one-time bake (see above) can't happen while the main world pass is
    // already open - dynamic rendering passes can't nest, and Render2D only
    // supports one active recording context at a time. Call pauseWorldPass()
    // before the bake, run it (createBakedTexture/beginBakePass/.../
    // endBakePass), then resumeWorldPass() to reopen the main pass exactly
    // as beginFrame() left it, so the rest of that frame's rendering
    // continues normally.
    void pauseWorldPass();
    void resumeWorldPass();

private:
    void ensureRenderTarget();

    SDL_Window* m_window = nullptr;
    std::unique_ptr<RHI::Vulkan::VulkanDevice> m_vulkan;
    Render2D m_render2D;
    int m_width = 0;
    int m_height = 0;
    int m_renderWidth = 0;
    int m_renderHeight = 0;
    RHI::TextureHandle m_renderTarget;
    RHI::ClearColor m_clearColor;
    DisplayMode m_displayMode = DisplayMode::Windowed;
    bool m_frameReady = false;
    bool m_render2DActive = false;
    bool m_worldPassActive = false;
    bool m_directSceneRendered = false;
    bool m_mouseCaptured = false;
};

}
