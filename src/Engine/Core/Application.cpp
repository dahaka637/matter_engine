#include "Engine/Core/Application.hpp"
#include "Engine/Platform/SDL3/SDLPlatform.hpp"
#include "Engine/Platform/SDL3/SDLInputAdapter.hpp"
#include <SDL3/SDL.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <thread>
#include <utility>

namespace MatterEngine {

namespace {

// Swapchain FIFO present is supposed to pace the loop to the display's
// refresh rate on its own, but under Windows DWM composition (no exclusive
// fullscreen) some driver/compositor combinations let the CPU submit frames
// faster than they're actually shown - observed in practice as an uncapped,
// multi-thousand-FPS loop even with vsync requested. A backstop limiter here
// is standard practice in shipped engines for exactly this reason, not a
// workaround for one specific machine.
float queryRefreshRateHz(SDL_Window* window) {
    constexpr float FallbackHz = 60.0f;
    if (window == nullptr) {
        return FallbackHz;
    }
    const SDL_DisplayID display = SDL_GetDisplayForWindow(window);
    if (display == 0) {
        return FallbackHz;
    }
    const SDL_DisplayMode* mode = SDL_GetCurrentDisplayMode(display);
    return (mode != nullptr && mode->refresh_rate > 0.0f) ? mode->refresh_rate : FallbackHz;
}

}

Application::Application(ApplicationConfig config)
    : m_config(std::move(config)),
      m_taskScheduler(std::make_shared<TaskScheduler>(
          TaskSchedulerSettings { m_config.workerThreadCount })) {
}

void Application::run() {
    m_running = true;
    Platform::SDLPlatform platform;
    platform.initialize();
    bool lifecycleStarted = false;
    try {
        m_renderer = std::make_unique<Renderer>();
        m_renderer->initialize(m_config.title, m_config.width, m_config.height,
            m_config.vsync, m_config.displayMode);
        lifecycleStarted = true;

        onStart();

        using Clock = std::chrono::steady_clock;
        auto previous = Clock::now();
        const float fixedDelta = 1.0f / std::max(1.0f, m_config.fixedUpdateHz);
        float accumulator = 0.0f;
        const float targetFrameSeconds = m_config.vsync ? 1.0f / queryRefreshRateHz(m_renderer->window()) : 0.0f;

        while (m_running) {
            const auto frameStart = Clock::now();
            SDL_Event event;
            while (SDL_PollEvent(&event)) {
                m_renderer->processEvent(event);
                if (const auto translated = Platform::SDLInputAdapter::translate(event, m_renderer->window())) {
                    m_input.setMousePosition(translated->mousePosition);
                    if (translated->type == EventType::KeyDown) m_input.setKey(translated->key, true);
                    if (translated->type == EventType::KeyUp) m_input.setKey(translated->key, false);
                    onEvent(*translated);
                    if (translated->type == EventType::Quit || translated->type == EventType::WindowClose) {
                        requestQuit();
                    }
                }
            }

            Platform::SDLInputAdapter::synchronize(m_input, m_renderer->window());
            const auto eventsEnd = Clock::now();

            const auto current = Clock::now();
            const float deltaTime = std::min(std::chrono::duration<float>(current - previous).count(), 0.25f);
            previous = current;

            accumulator += deltaTime;
            int fixedSteps = 0;
            while (accumulator >= fixedDelta && fixedSteps < std::max(1, m_config.maxFixedStepsPerFrame)) {
                onUpdate(fixedDelta);
                accumulator -= fixedDelta;
                ++fixedSteps;
            }
            if (fixedSteps == std::max(1, m_config.maxFixedStepsPerFrame) && accumulator >= fixedDelta) {
                // Drop excessive backlog after a long stall instead of entering a
                // spiral where simulation can never catch up with rendering.
                accumulator = std::fmod(accumulator, fixedDelta);
            }
            const auto updateEnd = Clock::now();
            m_renderer->beginFrame(Color { 19, 18, 16, 255 });
            onRender(*m_renderer);
            const auto renderEnd = Clock::now();
            m_renderer->beginGuiFrame();
            onGui(*m_renderer);
            m_renderer->endGuiFrame();
            m_renderer->endFrame();
            const auto guiEnd = Clock::now();

            m_frameMetrics.eventsMilliseconds =
                std::chrono::duration<float, std::milli>(
                    eventsEnd - frameStart).count();
            m_frameMetrics.fixedUpdateMilliseconds =
                std::chrono::duration<float, std::milli>(
                    updateEnd - eventsEnd).count();
            m_frameMetrics.renderMilliseconds =
                std::chrono::duration<float, std::milli>(
                    renderEnd - updateEnd).count();
            m_frameMetrics.guiMilliseconds =
                std::chrono::duration<float, std::milli>(
                    guiEnd - renderEnd).count();
            m_frameMetrics.totalMilliseconds =
                std::chrono::duration<float, std::milli>(
                    guiEnd - frameStart).count();
            m_frameMetrics.fixedStepCount = fixedSteps;

            if (targetFrameSeconds > 0.0f) {
                // Measured: std::this_thread::sleep_for on this machine (even
                // after requesting 1ms timer resolution via timeBeginPeriod, and
                // even for a request as small as ~4-5ms) routinely overshot to a
                // ~10ms floor regardless of how small a sleep was actually asked
                // for - turning an intended ~7ms vsync-paced frame into ~10-11ms
                // actually elapsed. That oversleep, not game logic or rendering,
                // was the dominant cost in every frame - a pure spin-wait (yield,
                // not sleep) has no OS timer-granularity floor to overshoot past.
                while (std::chrono::duration<float>(Clock::now() - current).count() < targetFrameSeconds) {
                    std::this_thread::yield();
                }
            }
        }
    } catch (...) {
        // The game's resources (notably SDL audio streams) must be released
        // before SDLPlatform tears down its subsystems during stack unwinding.
        // Otherwise the original rendering error is hidden by a second fault
        // while a stale SDL resource is destroyed.
        if (lifecycleStarted) {
            try {
                onStop();
            } catch (...) {
                // Preserve the original exception.
            }
        }
        if (m_renderer) {
            try {
                m_renderer->shutdown();
            } catch (...) {
                // Preserve the original exception.
            }
            m_renderer.reset();
        }
        platform.shutdown();
        throw;
    }

    onStop();
    m_renderer->shutdown();
    m_renderer.reset();
    platform.shutdown();
}

void Application::requestQuit() {
    m_running = false;
}

}
