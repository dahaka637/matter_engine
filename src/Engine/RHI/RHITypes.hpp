#pragma once

#include "Engine/RHI/RHIHandles.hpp"
#include <cstddef>
#include <cstdint>
#include <vector>

namespace MatterEngine::RHI {

struct ClearColor {
    float r = 0.0f;
    float g = 0.0f;
    float b = 0.0f;
    float a = 1.0f;
};

struct Extent2D {
    std::uint32_t width = 0;
    std::uint32_t height = 0;
};

// Medidas do frame concluido no mesmo slot de frames-in-flight. Tempos CPU
// separam sincronizacao/apresentacao; gpuFrameMilliseconds vem de timestamps
// Vulkan e nao inclui espera do compositor.
struct FramePerformanceMetrics {
    float cpuFenceWaitMilliseconds = 0.0f;
    float cpuAcquireMilliseconds = 0.0f;
    float cpuPresentMilliseconds = 0.0f;
    float gpuFrameMilliseconds = 0.0f;
    bool gpuTimingValid = false;
};

enum class BufferUsage {
    Vertex,
    Index,
    Uniform,
    Transfer
};

struct BufferDesc {
    std::size_t size = 0;
    BufferUsage usage = BufferUsage::Vertex;
    bool cpuVisible = false;
    const char* debugName = nullptr;
};

enum class ShaderStage : std::uint8_t {
    Vertex = 1,
    Fragment = 2,
    Compute = 4
};

constexpr ShaderStage operator|(ShaderStage lhs, ShaderStage rhs) {
    return static_cast<ShaderStage>(static_cast<std::uint8_t>(lhs) | static_cast<std::uint8_t>(rhs));
}

struct ShaderDesc {
    ShaderStage stage = ShaderStage::Vertex;
    std::vector<std::uint32_t> spirv;
    const char* entryPoint = "main";
    const char* debugName = nullptr;
};

enum class VertexFormat {
    Float2,
    Float3,
    Float4,
    UNorm8x4
};

struct VertexAttribute {
    std::uint32_t location = 0;
    VertexFormat format = VertexFormat::Float2;
    std::uint32_t offset = 0;
};

struct GraphicsPipelineDesc {
    ShaderHandle vertexShader;
    ShaderHandle fragmentShader;
    std::uint32_t vertexStride = 0;
    std::vector<VertexAttribute> attributes;
    std::uint32_t pushConstantSize = 0;
    ShaderStage pushConstantStages = ShaderStage::Vertex;
    bool alphaBlend = false;
    bool depthTest = false;
    const char* debugName = nullptr;
};

enum class IndexType {
    UInt16,
    UInt32
};

enum class FrameStatus {
    Ready,
    Skipped
};

// A single-purpose offscreen color target: rendered into like the swapchain,
// then sampled (nearest-neighbor) exactly once by Device::blitToSwapchain.
// Deliberately minimal - this is not a general asset/material texture
// pipeline (no mips, no formats, no CPU upload); it exists solely to let the
// game world render at a small, fixed low resolution for the pixel-art look
// and the performance headroom that comes with shading far fewer pixels,
// decoupled from the window's real resolution.
struct TextureDesc {
    Extent2D extent;
    const char* debugName = nullptr;
};

} // namespace MatterEngine::RHI
