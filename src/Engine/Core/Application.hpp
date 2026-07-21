#pragma once

#include "Engine/Input/Input.hpp"
#include "Engine/Render/Renderer.hpp"
#include "Engine/Core/TaskScheduler.hpp"
#include <memory>
#include <string>

namespace MatterEngine {

struct ApplicationConfig {
    std::string title = "MatterEngine";
    int width = 1280;
    int height = 720;
    bool vsync = true;
    DisplayMode displayMode = DisplayMode::Windowed;
    float fixedUpdateHz = 120.0f;
    int maxFixedStepsPerFrame = 8;
    std::uint32_t workerThreadCount = 0;
};

struct ApplicationFrameMetrics {
    float eventsMilliseconds = 0.0f;
    float fixedUpdateMilliseconds = 0.0f;
    float renderMilliseconds = 0.0f;
    float guiMilliseconds = 0.0f;
    float totalMilliseconds = 0.0f;
    int fixedStepCount = 0;
};

class Application {
public:
    explicit Application(ApplicationConfig config = {});
    virtual ~Application() = default;

    void run();
    void requestQuit();

protected:
    virtual void onStart() {}
    virtual void onEvent(const Event&) {}
    virtual void onUpdate(float) {}
    virtual void onRender(Renderer&) {}
    virtual void onGui(Renderer&) {}
    virtual void onStop() {}

    [[nodiscard]] const InputState& input() const { return m_input; }
    [[nodiscard]] Renderer& renderer() { return *m_renderer; }
    [[nodiscard]] std::shared_ptr<TaskScheduler> taskScheduler() const {
        return m_taskScheduler;
    }
    [[nodiscard]] const ApplicationFrameMetrics&
        applicationFrameMetrics() const { return m_frameMetrics; }

private:
    ApplicationConfig m_config;
    std::shared_ptr<TaskScheduler> m_taskScheduler;
    std::unique_ptr<Renderer> m_renderer;
    InputState m_input;
    ApplicationFrameMetrics m_frameMetrics;
    bool m_running = true;
};

}
