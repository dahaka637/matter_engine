#include "Engine/Render/Renderer.hpp"

#include "Engine/Core/Log.hpp"
#include "Engine/RHI/Vulkan/VulkanDevice.hpp"
#include "Engine/UI/ImGuiFontLibrary.hpp"
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

void applyMatterEngineStyle() {
    ImGuiStyle& style = ImGui::GetStyle();
    style.WindowRounding = 4.0f;
    style.ChildRounding = 4.0f;
    style.FrameRounding = 3.0f;
    style.PopupRounding = 4.0f;
    style.ScrollbarRounding = 2.0f;
    style.GrabRounding = 2.0f;
    style.TabRounding = 2.0f;
    style.WindowBorderSize = 1.0f;
    style.FrameBorderSize = 1.0f;
    style.ChildBorderSize = 1.0f;
    style.WindowPadding = ImVec2(12.0f, 12.0f);
    style.FramePadding = ImVec2(10.0f, 6.0f);
    style.ItemSpacing = ImVec2(8.0f, 7.0f);
    style.ItemInnerSpacing = ImVec2(7.0f, 6.0f);
    style.WindowTitleAlign = ImVec2(0.03f, 0.5f);

    ImVec4* colors = style.Colors;
    colors[ImGuiCol_Text] = ImVec4(0.82f, 0.87f, 0.91f, 1.00f);
    colors[ImGuiCol_TextDisabled] = ImVec4(0.43f, 0.51f, 0.58f, 1.00f);
    colors[ImGuiCol_WindowBg] = ImVec4(0.031f, 0.052f, 0.071f, 0.98f);
    colors[ImGuiCol_ChildBg] = ImVec4(0.042f, 0.073f, 0.102f, 0.97f);
    colors[ImGuiCol_PopupBg] = ImVec4(0.047f, 0.078f, 0.108f, 0.99f);
    colors[ImGuiCol_Border] = ImVec4(0.10f, 0.17f, 0.23f, 1.00f);
    colors[ImGuiCol_FrameBg] = ImVec4(0.035f, 0.063f, 0.086f, 1.00f);
    colors[ImGuiCol_FrameBgHovered] = ImVec4(0.055f, 0.13f, 0.20f, 1.00f);
    colors[ImGuiCol_FrameBgActive] = ImVec4(0.06f, 0.20f, 0.33f, 1.00f);
    colors[ImGuiCol_TitleBg] = ImVec4(0.027f, 0.047f, 0.063f, 1.00f);
    colors[ImGuiCol_TitleBgActive] = ImVec4(0.035f, 0.075f, 0.11f, 1.00f);
    colors[ImGuiCol_Button] = ImVec4(0.05f, 0.18f, 0.30f, 1.00f);
    colors[ImGuiCol_ButtonHovered] = ImVec4(0.07f, 0.34f, 0.58f, 1.00f);
    colors[ImGuiCol_ButtonActive] = ImVec4(0.08f, 0.46f, 0.82f, 1.00f);
    colors[ImGuiCol_Header] = ImVec4(0.055f, 0.18f, 0.29f, 1.00f);
    colors[ImGuiCol_HeaderHovered] = ImVec4(0.07f, 0.31f, 0.52f, 1.00f);
    colors[ImGuiCol_HeaderActive] = ImVec4(0.08f, 0.42f, 0.74f, 1.00f);
    colors[ImGuiCol_CheckMark] = ImVec4(0.16f, 0.57f, 1.00f, 1.00f);
    colors[ImGuiCol_SliderGrab] = ImVec4(0.12f, 0.49f, 0.89f, 1.00f);
    colors[ImGuiCol_SliderGrabActive] = ImVec4(0.22f, 0.67f, 1.00f, 1.00f);
    colors[ImGuiCol_Separator] = ImVec4(0.10f, 0.17f, 0.23f, 1.00f);
    colors[ImGuiCol_Tab] = ImVec4(0.035f, 0.09f, 0.14f, 1.00f);
    colors[ImGuiCol_TabHovered] = ImVec4(0.06f, 0.28f, 0.47f, 1.00f);
    colors[ImGuiCol_TabSelected] = ImVec4(0.06f, 0.24f, 0.42f, 1.00f);
}

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
    applyMatterEngineStyle();

    m_vulkan->initializeImGui(m_window);
    m_render2D.initialize(*m_vulkan);
    applyVideoSettings(width, height, mode);
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
