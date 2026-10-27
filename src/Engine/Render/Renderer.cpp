#include "Engine/Render/Renderer.hpp"

#include "Engine/Core/Log.hpp"
#include "Engine/RHI/Vulkan/VulkanDevice.hpp"
#include "Engine/UI/ImGuiFontLibrary.hpp"
#include "Engine/UI/ImGuiTheme.hpp"
#include "imgui.h"
#include "imgui_impl_sdl3.h"
#include <SDL3/SDL.h>
#include <algorithm>
#include <cstddef>
#include <cmath>
#include <cstring>
#include <stdexcept>
#include <vector>

namespace MatterEngine {

namespace {

// Fixed virtual height for the pixel-art world render target; width adapts
// to the window's aspect ratio (computed in ensureRenderTarget) so the
// upscale blit always covers the swapchain exactly, with no letterboxing.
// The Workbench camera bounds are scaled proportionally to this value -
// raising it here without
// scaling those too would just show more of the pitch at the same per-object
// pixel density, not fix small-object legibility.
constexpr int VirtualRenderHeight = 360;

} // namespace

Renderer::Renderer() = default;

Renderer::~Renderer() {
    shutdown();
}

void Renderer::initialize(const std::string& title, int width, int height, bool vsync, DisplayMode mode) {
    m_window = SDL_CreateWindow(title.c_str(), width, height,
        SDL_WINDOW_VULKAN | SDL_WINDOW_RESIZABLE | SDL_WINDOW_HIGH_PIXEL_DENSITY);
    if (m_window == nullptr) {
        throw std::runtime_error(std::string("SDL_CreateWindow failed: ") + SDL_GetError());
    }
    SDL_SetWindowMinimumSize(m_window, 960, 540);
    SDL_SetWindowPosition(m_window, SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED);

    m_vulkan = std::make_unique<RHI::Vulkan::VulkanDevice>();
    m_vulkan->initialize(m_window, vsync, title);

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
    io.IniFilename = nullptr;
    UI::configureImGuiFonts();
    UI::applyMatterEngineStyle();

    m_vulkan->initializeImGui(m_window);
    m_render2D.initialize(*m_vulkan);
    applyVideoSettings(width, height, mode);
    if (mode == DisplayMode::Windowed) {
        SDL_MaximizeWindow(m_window);
        SDL_SyncWindow(m_window);
    }
    SDL_GetWindowSizeInPixels(m_window, &m_width, &m_height);
    Log::info(std::string("SDL3, ") + m_vulkan->backendName() + " and Dear ImGui initialized.");
}

void Renderer::shutdown() {
    if (m_vulkan) {
        m_vulkan->waitIdle();
        m_render2D.shutdown();
        m_vulkan->shutdownImGui();
    }
    if (ImGui::GetCurrentContext() != nullptr) {
        ImGui::DestroyContext();
    }
    if (m_vulkan) {
        m_vulkan->shutdown();
        m_vulkan.reset();
    }
    if (m_window != nullptr) {
        SDL_DestroyWindow(m_window);
        m_window = nullptr;
    }
    m_width = 0;
    m_height = 0;
    m_renderWidth = 0;
    m_renderHeight = 0;
    m_renderTarget = {};
    m_frameReady = false;
}

void Renderer::processEvent(const SDL_Event& event) {
    if (ImGui::GetCurrentContext() != nullptr) {
        ImGui_ImplSDL3_ProcessEvent(&event);
    }
    if (m_vulkan && (event.type == SDL_EVENT_WINDOW_PIXEL_SIZE_CHANGED
        || event.type == SDL_EVENT_WINDOW_RESIZED
        || event.type == SDL_EVENT_WINDOW_ENTER_FULLSCREEN
        || event.type == SDL_EVENT_WINDOW_LEAVE_FULLSCREEN)) {
        m_vulkan->requestSwapchainRebuild();
    }
}

void Renderer::ensureRenderTarget() {
    if (m_width <= 0 || m_height <= 0) {
        return;
    }
    const int desiredHeight = VirtualRenderHeight;
    const int desiredWidth = std::max(1, static_cast<int>(std::lround(
        static_cast<double>(desiredHeight) * m_width / m_height)));
    if (m_renderTarget && desiredWidth == m_renderWidth && desiredHeight == m_renderHeight) {
        return;
    }
    if (m_renderTarget) {
        m_vulkan->destroyTexture(m_renderTarget);
        m_renderTarget = {};
    }
    RHI::TextureDesc desc;
    desc.extent = { static_cast<std::uint32_t>(desiredWidth), static_cast<std::uint32_t>(desiredHeight) };
    desc.debugName = "World render target";
    m_renderTarget = m_vulkan->createRenderTarget(desc);
    m_renderWidth = desiredWidth;
    m_renderHeight = desiredHeight;
}

void Renderer::beginFrame(Color clearColor) {
    SDL_GetWindowSizeInPixels(m_window, &m_width, &m_height);
    m_clearColor = {
        static_cast<float>(clearColor.r) / 255.0f,
        static_cast<float>(clearColor.g) / 255.0f,
        static_cast<float>(clearColor.b) / 255.0f,
        static_cast<float>(clearColor.a) / 255.0f,
    };
    m_frameReady = m_vulkan->beginFrame() == RHI::FrameStatus::Ready;
    m_directSceneRendered = false;
    m_worldPassActive = false;
    RHI::Extent2D extent { static_cast<std::uint32_t>(std::max(0, m_width)),
        static_cast<std::uint32_t>(std::max(0, m_height)) };
    if (m_frameReady) {
        ensureRenderTarget();
        extent = { static_cast<std::uint32_t>(m_renderWidth), static_cast<std::uint32_t>(m_renderHeight) };
        m_vulkan->beginRenderTargetPass(m_renderTarget, m_clearColor);
        m_worldPassActive = true;
    }
    m_render2D.begin(extent);
    m_render2DActive = true;
}

void Renderer::beginGuiFrame() {
    if (m_render2DActive) {
        if (m_frameReady) {
            m_render2D.flush();
        }
        m_render2D.end();
        m_render2DActive = false;
    }
    if (m_frameReady && m_worldPassActive) {
        m_vulkan->endRenderTargetPass();
        m_worldPassActive = false;
    }
    if (m_frameReady && !m_directSceneRendered) {
        m_vulkan->blitToSwapchain(m_renderTarget, m_clearColor);
    }
    m_vulkan->beginImGuiFrame();
    ImGui::NewFrame();
}

void Renderer::endGuiFrame() {
    ImGui::Render();
    if (m_frameReady) {
        m_vulkan->renderImGui(ImGui::GetDrawData());
    }
}

void Renderer::endFrame() {
    if (m_frameReady) {
        m_vulkan->endFrame();
    }
    m_frameReady = false;
    m_render2DActive = false;
    m_worldPassActive = false;
    m_directSceneRendered = false;
}

void Renderer::applyVideoSettings(int width, int height, DisplayMode mode) {
    if (m_window == nullptr) {
        return;
    }
    if (mode == DisplayMode::Fullscreen) {
        SDL_DisplayMode closest {};
        const SDL_DisplayID display = SDL_GetDisplayForWindow(m_window);
        if (SDL_GetClosestFullscreenDisplayMode(display, width, height, 0.0f, true, &closest)) {
            SDL_SetWindowFullscreenMode(m_window, &closest);
        }
        SDL_SetWindowFullscreen(m_window, true);
    } else if (mode == DisplayMode::BorderlessFullscreen) {
        SDL_SetWindowFullscreenMode(m_window, nullptr);
        SDL_SetWindowFullscreen(m_window, true);
    } else {
        SDL_SetWindowFullscreen(m_window, false);
        SDL_SetWindowSize(m_window, width, height);
        SDL_SetWindowPosition(m_window, SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED);
    }
    m_displayMode = mode;
    SDL_SyncWindow(m_window);
    SDL_GetWindowSizeInPixels(m_window, &m_width, &m_height);
    if (m_vulkan) {
        m_vulkan->requestSwapchainRebuild();
    }
}

SDL_Window* Renderer::window() const { return m_window; }
int Renderer::width() const { return m_width; }
int Renderer::height() const { return m_height; }
int Renderer::renderWidth() const { return m_renderWidth; }
int Renderer::renderHeight() const { return m_renderHeight; }
DisplayMode Renderer::displayMode() const { return m_displayMode; }
Render2D& Renderer::render2D() { return m_render2D; }
const char* Renderer::backendName() const { return m_vulkan ? m_vulkan->backendName() : "None"; }
RHI::FramePerformanceMetrics Renderer::framePerformanceMetrics() const {
    return m_vulkan ? m_vulkan->framePerformanceMetrics()
        : RHI::FramePerformanceMetrics {};
}

UiTexture Renderer::loadUiTexture(const std::string& path) {
    if (!m_vulkan) {
        throw std::runtime_error("Cannot load a UI texture before Renderer initialization");
    }

    SDL_Surface* decoded = SDL_LoadPNG(path.c_str());
    if (decoded == nullptr) {
        throw std::runtime_error("Failed to load UI texture '" + path + "': " + SDL_GetError());
    }

    SDL_Surface* rgba = SDL_ConvertSurface(decoded, SDL_PIXELFORMAT_RGBA32);
    SDL_DestroySurface(decoded);
    if (rgba == nullptr) {
        throw std::runtime_error("Failed to convert UI texture '" + path + "' to RGBA: " + SDL_GetError());
    }

    if (!SDL_LockSurface(rgba)) {
        const std::string error = SDL_GetError();
        SDL_DestroySurface(rgba);
        throw std::runtime_error("Failed to lock UI texture '" + path + "': " + error);
    }

    const std::size_t rowBytes = static_cast<std::size_t>(rgba->w) * 4;
    std::vector<std::byte> pixels(rowBytes * static_cast<std::size_t>(rgba->h));
    const auto* source = static_cast<const std::byte*>(rgba->pixels);
    for (int row = 0; row < rgba->h; ++row) {
        std::memcpy(pixels.data() + static_cast<std::size_t>(row) * rowBytes,
            source + static_cast<std::size_t>(row) * static_cast<std::size_t>(rgba->pitch), rowBytes);
    }
    SDL_UnlockSurface(rgba);

    const int textureWidth = rgba->w;
    const int textureHeight = rgba->h;
    SDL_DestroySurface(rgba);

    const std::uint64_t textureId = m_vulkan->createImGuiTexture(
        { static_cast<std::uint32_t>(textureWidth), static_cast<std::uint32_t>(textureHeight) }, pixels);
    return { textureId, textureWidth, textureHeight };
}

UiTexture Renderer::createUiTexture(int width, int height, std::span<const std::byte> rgbaPixels) {
    if (!m_vulkan) throw std::runtime_error("Cannot create a UI texture before Renderer initialization");
    const std::uint64_t textureId = m_vulkan->createImGuiTexture(
        { static_cast<std::uint32_t>(width), static_cast<std::uint32_t>(height) }, rgbaPixels);
    return { textureId, width, height };
}

void Renderer::updateUiTexture(const UiTexture& texture, std::span<const std::byte> rgbaPixels) {
    if (!m_vulkan || !texture) return;
    m_vulkan->updateImGuiTexture(texture.id, rgbaPixels);
}

void Renderer::destroyUiTexture(UiTexture& texture) {
    if (m_vulkan && texture) m_vulkan->destroyImGuiTexture(texture.id);
    texture = {};
}

UiTexture Renderer::renderScene3D(const Scene3DFrame& scene, int width, int height) {
    if (!m_vulkan || !m_frameReady) {
        return {};
    }
    const int requestedWidth = std::max(1, width);
    const int requestedHeight = std::max(1, height);
    const float scale = std::min({ 1.0f, 1280.0f / static_cast<float>(requestedWidth),
        900.0f / static_cast<float>(requestedHeight) });
    const int targetWidth = std::max(1, static_cast<int>(std::lround(requestedWidth * scale)));
    const int targetHeight = std::max(1, static_cast<int>(std::lround(requestedHeight * scale)));
    pauseWorldPass();
    const std::uint64_t textureId = m_vulkan->renderScene3D(scene,
        { static_cast<std::uint32_t>(targetWidth), static_cast<std::uint32_t>(targetHeight) });
    resumeWorldPass();
    return { textureId, targetWidth, targetHeight };
}

RHI::BufferHandle Renderer::createBuffer(const RHI::BufferDesc& desc) {
    return m_vulkan->createBuffer(desc);
}

void Renderer::writeBuffer(RHI::BufferHandle handle, std::size_t offset, std::span<const std::byte> data) {
    m_vulkan->writeBuffer(handle, offset, data);
}

void Renderer::destroyBuffer(RHI::BufferHandle handle) {
    m_vulkan->destroyBuffer(handle);
}

RHI::TextureHandle Renderer::createTexture2D(RHI::Extent2D extent, std::span<const std::byte> rgbaPixels) {
    return m_vulkan->createTexture2D(extent, rgbaPixels);
}

RHI::CharacterMaterialHandle Renderer::createCharacterMaterial(const RHI::CharacterMaterialDesc& desc) {
    return m_vulkan->createCharacterMaterial(desc);
}

void Renderer::updateCharacterKitLayer(RHI::CharacterMaterialHandle handle, std::uint32_t layer, std::span<const std::byte> levels,
    std::span<const float> regionAverages) {
    m_vulkan->updateCharacterKitLayer(handle, layer, levels, regionAverages);
}

void Renderer::updateCharacterVertices(RHI::CharacterMaterialHandle handle, std::span<const RHI::CharacterKitVertexGpu> vertices) {
    m_vulkan->updateCharacterVertices(handle, vertices);
}

void Renderer::destroyCharacterMaterial(RHI::CharacterMaterialHandle handle) {
    m_vulkan->destroyCharacterMaterial(handle);
}

void Renderer::destroyTexture(RHI::TextureHandle handle) {
    m_vulkan->destroyTexture(handle);
}

bool Renderer::renderScene3DToScreen(const Scene3DFrame& scene) {
    if (!m_vulkan || !m_frameReady || m_directSceneRendered) {
        return false;
    }
    pauseWorldPass();
    m_vulkan->renderScene3DToSwapchain(scene);
    m_directSceneRendered = true;
    return true;
}

void Renderer::setMouseCaptured(bool captured) {
    if (m_window == nullptr || m_mouseCaptured == captured) {
        return;
    }
    if (!SDL_SetWindowRelativeMouseMode(m_window, captured)) {
        Log::warn(std::string("SDL_SetWindowRelativeMouseMode failed: ") + SDL_GetError());
        return;
    }
    m_mouseCaptured = captured;
}

std::vector<std::pair<int, int>> Renderer::availableFullscreenResolutions() const {
    std::vector<std::pair<int, int>> resolutions;
    const SDL_DisplayID display = m_window != nullptr
        ? SDL_GetDisplayForWindow(m_window) : SDL_GetPrimaryDisplay();
    int count = 0;
    SDL_DisplayMode** modes = SDL_GetFullscreenDisplayModes(display, &count);
    for (int i = 0; modes != nullptr && i < count; ++i) {
        const SDL_DisplayMode* mode = modes[i];
        if (mode == nullptr || mode->w <= 0 || mode->h <= 0) {
            continue;
        }
        const std::pair resolution { mode->w, mode->h };
        if (std::find(resolutions.begin(), resolutions.end(), resolution) == resolutions.end()) {
            resolutions.push_back(resolution);
        }
    }
    SDL_free(modes);
    std::sort(resolutions.begin(), resolutions.end(), [](const auto& a, const auto& b) {
        return a.first * a.second > b.first * b.second;
    });
    return resolutions;
}

RHI::TextureHandle Renderer::createBakedTexture(int width, int height) {
    RHI::TextureDesc desc;
    desc.extent = { static_cast<std::uint32_t>(std::max(1, width)), static_cast<std::uint32_t>(std::max(1, height)) };
    desc.debugName = "Baked world texture";
    return m_vulkan->createRenderTarget(desc);
}

void Renderer::beginBakePass(RHI::TextureHandle target, int width, int height, Color clearColor) {
    const RHI::ClearColor normalized {
        static_cast<float>(clearColor.r) / 255.0f,
        static_cast<float>(clearColor.g) / 255.0f,
        static_cast<float>(clearColor.b) / 255.0f,
        static_cast<float>(clearColor.a) / 255.0f,
    };
    m_vulkan->beginRenderTargetPass(target, normalized);
    // A bake pass borrows m_render2D just like the main world pass does -
    // Render2D::active() is thread-local and only one recording context is
    // ever open at a time, so this is safe as long as callers don't nest it
    // inside the regular per-frame world render's own begin()/end() pair
    // (bakes happen once, at setup time, never mid-frame).
    m_render2D.begin({ static_cast<std::uint32_t>(std::max(1, width)), static_cast<std::uint32_t>(std::max(1, height)) });
}

void Renderer::endBakePass() {
    m_render2D.flush();
    m_render2D.end();
    m_vulkan->endRenderTargetPass();
}

void Renderer::drawWorldSprite(RHI::TextureHandle source, const std::array<float, 8>& screenCorners,
    const std::array<float, 4>& perspectiveDepths) {
    if (m_frameReady) {
        m_vulkan->drawWorldSprite(source, screenCorners, perspectiveDepths);
    }
}

void Renderer::pauseWorldPass() {
    if (!m_frameReady || !m_worldPassActive) {
        return;
    }
    if (m_render2DActive) {
        m_render2D.flush();
        m_render2D.end();
        m_render2DActive = false;
    }
    m_vulkan->endRenderTargetPass();
    m_worldPassActive = false;
}

void Renderer::resumeWorldPass() {
    if (!m_frameReady) {
        return;
    }
    m_vulkan->beginRenderTargetPass(m_renderTarget, m_clearColor);
    m_render2D.begin({ static_cast<std::uint32_t>(m_renderWidth), static_cast<std::uint32_t>(m_renderHeight) });
    m_worldPassActive = true;
    m_render2DActive = true;
}

} // namespace MatterEngine
