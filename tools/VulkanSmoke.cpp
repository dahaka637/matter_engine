#include "Engine/Render/Renderer.hpp"

#include <SDL3/SDL.h>

#include <chrono>
#include <iostream>
#include <stdexcept>
#include <thread>

namespace {

void drawFrames(MatterEngine::Renderer& renderer, int count) {
    for (int frame = 0; frame < count; ++frame) {
        SDL_Event event;
        while (SDL_PollEvent(&event)) {
            renderer.processEvent(event);
        }
        renderer.beginFrame({ 7, 12, 19, 255 });
        renderer.beginGuiFrame();
        renderer.endGuiFrame();
        renderer.endFrame();
        std::this_thread::sleep_for(std::chrono::milliseconds(16));
    }
}

void pumpEvents(MatterEngine::Renderer& renderer, int iterations) {
    for (int iteration = 0; iteration < iterations; ++iteration) {
        SDL_Event event;
        while (SDL_PollEvent(&event)) {
            renderer.processEvent(event);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(16));
    }
}

void benchmarkUncappedFrame() {
    MatterEngine::Renderer renderer;
    renderer.initialize("MatterEngine Vulkan Benchmark", 1280, 720, false,
        MatterEngine::DisplayMode::Windowed);
    for (int frame = 0; frame < 60; ++frame) {
        renderer.beginFrame({ 7, 12, 19, 255 });
        renderer.beginGuiFrame();
        renderer.endGuiFrame();
        renderer.endFrame();
    }
    constexpr int MeasuredFrames = 1200;
    const auto start = std::chrono::steady_clock::now();
    for (int frame = 0; frame < MeasuredFrames; ++frame) {
        SDL_Event event;
        while (SDL_PollEvent(&event)) renderer.processEvent(event);
        renderer.beginFrame({ 7, 12, 19, 255 });
        renderer.beginGuiFrame();
        renderer.endGuiFrame();
        renderer.endFrame();
    }
    const float seconds = std::chrono::duration<float>(
        std::chrono::steady_clock::now() - start).count();
    const MatterEngine::RHI::FramePerformanceMetrics timings =
        renderer.framePerformanceMetrics();
    std::cout << "[BENCH] clear+ImGui: "
              << static_cast<float>(MeasuredFrames) / seconds << " FPS, "
              << seconds * 1000.0f / static_cast<float>(MeasuredFrames)
              << " ms/frame; GPU=" << timings.gpuFrameMilliseconds
              << " ms, fence=" << timings.cpuFenceWaitMilliseconds
              << " ms, acquire=" << timings.cpuAcquireMilliseconds
              << " ms, present=" << timings.cpuPresentMilliseconds
              << " ms" << std::endl;
    renderer.shutdown();
}

void require(bool condition, const char* message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

} // namespace

int main() {
    MatterEngine::Renderer renderer;
    try {
        renderer.initialize("MatterEngine Vulkan Smoke", 960, 540, true,
            MatterEngine::DisplayMode::Windowed);
        std::cout << "[SMOKE] startup" << std::endl;
        require(renderer.window() != nullptr, "window creation failed");
        drawFrames(renderer, 6);

        renderer.applyVideoSettings(1180, 720,
            MatterEngine::DisplayMode::Windowed);
        std::cout << "[SMOKE] resize" << std::endl;
        drawFrames(renderer, 6);

        require(SDL_MinimizeWindow(renderer.window()), "minimize failed");
        std::cout << "[SMOKE] minimize" << std::endl;
        pumpEvents(renderer, 6);
        require(SDL_RestoreWindow(renderer.window()), "restore failed");
        std::cout << "[SMOKE] restore" << std::endl;
        pumpEvents(renderer, 6);
        drawFrames(renderer, 6);

        renderer.applyVideoSettings(1280, 720,
            MatterEngine::DisplayMode::Fullscreen);
        std::cout << "[SMOKE] fullscreen" << std::endl;
        drawFrames(renderer, 8);
        renderer.applyVideoSettings(1280, 720,
            MatterEngine::DisplayMode::BorderlessFullscreen);
        std::cout << "[SMOKE] borderless" << std::endl;
        drawFrames(renderer, 8);
        renderer.applyVideoSettings(1100, 680,
            MatterEngine::DisplayMode::Windowed);
        std::cout << "[SMOKE] windowed" << std::endl;
        drawFrames(renderer, 6);

        renderer.shutdown();
        std::cout << "[SMOKE] graceful shutdown" << std::endl;
        benchmarkUncappedFrame();
        std::cout << "MatterEngine Vulkan smoke test passed.\n";
        return 0;
    } catch (const std::exception& error) {
        try {
            renderer.shutdown();
        } catch (...) {
        }
        std::cerr << "MatterEngine Vulkan smoke test failed: "
                  << error.what() << '\n';
        return 1;
    }
}
