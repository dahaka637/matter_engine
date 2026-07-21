#pragma once

#include "Engine/Math/Vec2.hpp"
#include "Engine/RHI/RHIDevice.hpp"
#include <array>
#include <cstdint>
#include <span>
#include <vector>

namespace MatterEngine {

struct DrawColor {
    float r = 1.0f;
    float g = 1.0f;
    float b = 1.0f;
    float a = 1.0f;
};

// CPU-batched 2D renderer. It intentionally exposes shape operations instead
// of graphics-API topology/state, so game presentation never depends on Vulkan.
class Render2D final {
public:
    Render2D() = default;
    ~Render2D();

    Render2D(const Render2D&) = delete;
    Render2D& operator=(const Render2D&) = delete;

    void initialize(RHI::Device& device);
    void shutdown();

    void begin(RHI::Extent2D extent);
    void flush();
    void end();

    void setColor(DrawColor color) { m_color = color; }
    void setLineWidth(float width) { m_lineWidth = width; }
    [[nodiscard]] DrawColor color() const { return m_color; }
    [[nodiscard]] float lineWidth() const { return m_lineWidth; }

    void line(Vec2 a, Vec2 b);
    void polyline(std::span<const Vec2> points, bool closed = false);
    void fillConvex(std::span<const Vec2> points);
    void rect(Vec2 min, Vec2 max);
    void rectOutline(Vec2 min, Vec2 max);

    // Transitional access for the existing procedural MatterEngine painter.
    // Valid only between Renderer::beginFrame() and endFrame(), on the render thread.
    [[nodiscard]] static Render2D& active();

private:
    struct Vertex {
        Vec2 position;
        DrawColor color;
    };

    struct FrameBuffers {
        RHI::BufferHandle vertices;
        RHI::BufferHandle indices;
    };

    void reserveGpuBuffers(std::size_t vertexCount, std::size_t indexCount);
    std::uint32_t addVertex(Vec2 position);

    RHI::Device* m_device = nullptr;
    RHI::Extent2D m_extent;
    RHI::ShaderHandle m_vertexShader;
    RHI::ShaderHandle m_fragmentShader;
    RHI::PipelineHandle m_pipeline;
    std::array<FrameBuffers, 2> m_frameBuffers {};
    std::size_t m_vertexCapacity = 0;
    std::size_t m_indexCapacity = 0;
    std::vector<Vertex> m_vertices;
    std::vector<std::uint32_t> m_indices;
    DrawColor m_color;
    float m_lineWidth = 1.0f;
    bool m_recording = false;

    static thread_local Render2D* s_active;
};

} // namespace MatterEngine
