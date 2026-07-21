#include "Engine/Render/Render2D.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <fstream>
#include <stdexcept>
#include <string>

namespace MatterEngine {

thread_local Render2D* Render2D::s_active = nullptr;

namespace {

std::vector<std::uint32_t> readSpirv(const std::string& path) {
    std::ifstream file(path, std::ios::binary | std::ios::ate);
    if (!file) {
        throw std::runtime_error("Failed to open SPIR-V shader: " + path);
    }
    const std::streamsize size = file.tellg();
    if (size <= 0 || size % static_cast<std::streamsize>(sizeof(std::uint32_t)) != 0) {
        throw std::runtime_error("Invalid SPIR-V shader size: " + path);
    }
    file.seekg(0, std::ios::beg);
    std::vector<std::uint32_t> words(static_cast<std::size_t>(size) / sizeof(std::uint32_t));
    if (!file.read(reinterpret_cast<char*>(words.data()), size)) {
        throw std::runtime_error("Failed to read SPIR-V shader: " + path);
    }
    return words;
}

std::size_t growCapacity(std::size_t required, std::size_t current, std::size_t minimum) {
    std::size_t result = std::max(current, minimum);
    while (result < required) {
        result *= 2;
    }
    return result;
}

} // namespace

Render2D::~Render2D() {
    shutdown();
}

void Render2D::initialize(RHI::Device& device) {
    if (m_device != nullptr) {
        return;
    }
    m_device = &device;

    RHI::ShaderDesc vertexDesc;
    vertexDesc.stage = RHI::ShaderStage::Vertex;
    vertexDesc.spirv = readSpirv(std::string(MATTERENGINE_SHADER_DIR) + "/render2d.vert.spv");
    vertexDesc.debugName = "Render2D vertex shader";
    m_vertexShader = m_device->createShader(vertexDesc);

    RHI::ShaderDesc fragmentDesc;
    fragmentDesc.stage = RHI::ShaderStage::Fragment;
    fragmentDesc.spirv = readSpirv(std::string(MATTERENGINE_SHADER_DIR) + "/render2d.frag.spv");
    fragmentDesc.debugName = "Render2D fragment shader";
    m_fragmentShader = m_device->createShader(fragmentDesc);

    RHI::GraphicsPipelineDesc pipelineDesc;
    pipelineDesc.vertexShader = m_vertexShader;
    pipelineDesc.fragmentShader = m_fragmentShader;
    pipelineDesc.vertexStride = sizeof(Vertex);
    pipelineDesc.attributes = {
        { 0, RHI::VertexFormat::Float2, static_cast<std::uint32_t>(offsetof(Vertex, position)) },
        { 1, RHI::VertexFormat::Float4, static_cast<std::uint32_t>(offsetof(Vertex, color)) },
    };
    pipelineDesc.pushConstantSize = sizeof(float) * 2;
    pipelineDesc.pushConstantStages = RHI::ShaderStage::Vertex;
    pipelineDesc.alphaBlend = true;
    pipelineDesc.debugName = "Render2D pipeline";
    m_pipeline = m_device->createGraphicsPipeline(pipelineDesc);

    reserveGpuBuffers(65536, 131072);
    m_vertices.reserve(m_vertexCapacity);
    m_indices.reserve(m_indexCapacity);
}

void Render2D::shutdown() {
    if (m_device == nullptr) {
        return;
    }
    m_device->waitIdle();
    for (FrameBuffers& buffers : m_frameBuffers) {
        if (buffers.vertices) m_device->destroyBuffer(buffers.vertices);
        if (buffers.indices) m_device->destroyBuffer(buffers.indices);
        buffers = {};
    }
    if (m_pipeline) m_device->destroyPipeline(m_pipeline);
    if (m_fragmentShader) m_device->destroyShader(m_fragmentShader);
    if (m_vertexShader) m_device->destroyShader(m_vertexShader);
    m_pipeline = {};
    m_fragmentShader = {};
    m_vertexShader = {};
    m_device = nullptr;
    m_vertices.clear();
    m_indices.clear();
    m_vertexCapacity = 0;
    m_indexCapacity = 0;
}

void Render2D::begin(RHI::Extent2D extent) {
    if (m_device == nullptr) {
        throw std::runtime_error("Render2D was not initialized");
    }
    m_extent = extent;
    m_vertices.clear();
    m_indices.clear();
    m_color = {};
    m_lineWidth = 1.0f;
    m_recording = true;
    s_active = this;
}

void Render2D::flush() {
    if (!m_recording || m_vertices.empty() || m_indices.empty()) {
        return;
    }
    reserveGpuBuffers(m_vertices.size(), m_indices.size());
    const std::uint32_t frameSlot = m_device->currentFrameSlot() % static_cast<std::uint32_t>(m_frameBuffers.size());
    FrameBuffers& buffers = m_frameBuffers[frameSlot];

    m_device->writeBuffer(buffers.vertices, 0, std::as_bytes(std::span(m_vertices)));
    m_device->writeBuffer(buffers.indices, 0, std::as_bytes(std::span(m_indices)));

    RHI::CommandList& commands = m_device->commandList();
    commands.setViewport(m_extent);
    commands.setScissor(m_extent);
    commands.bindPipeline(m_pipeline);
    commands.bindVertexBuffer(buffers.vertices);
    commands.bindIndexBuffer(buffers.indices, RHI::IndexType::UInt32);
    const std::array<float, 2> viewportSize {
        static_cast<float>(m_extent.width), static_cast<float>(m_extent.height)
    };
    commands.pushConstants(RHI::ShaderStage::Vertex, std::as_bytes(std::span(viewportSize)));
    commands.drawIndexed(static_cast<std::uint32_t>(m_indices.size()));
}

void Render2D::end() {
    if (s_active == this) {
        s_active = nullptr;
    }
    m_recording = false;
}

void Render2D::line(Vec2 a, Vec2 b) {
    const Vec2 delta = b - a;
    const float length = delta.length();
    if (length <= 0.0001f) {
        return;
    }
    const float halfWidth = std::max(0.5f, m_lineWidth * 0.5f);
    const Vec2 normal { -delta.y / length * halfWidth, delta.x / length * halfWidth };
    const std::uint32_t base = static_cast<std::uint32_t>(m_vertices.size());
    addVertex(a + normal);
    addVertex(b + normal);
    addVertex(b - normal);
    addVertex(a - normal);
    m_indices.insert(m_indices.end(), { base, base + 1, base + 2, base, base + 2, base + 3 });
}

void Render2D::polyline(std::span<const Vec2> points, bool closed) {
    if (points.size() < 2) {
        return;
    }
    for (std::size_t i = 1; i < points.size(); ++i) {
        line(points[i - 1], points[i]);
    }
    if (closed) {
        line(points.back(), points.front());
    }
}

void Render2D::fillConvex(std::span<const Vec2> points) {
    if (points.size() < 3) {
        return;
    }
    const std::uint32_t base = static_cast<std::uint32_t>(m_vertices.size());
    for (Vec2 point : points) {
        addVertex(point);
    }
    for (std::uint32_t i = 1; i + 1 < points.size(); ++i) {
        m_indices.insert(m_indices.end(), { base, base + i, base + i + 1 });
    }
}

void Render2D::rect(Vec2 min, Vec2 max) {
    const std::array<Vec2, 4> points {
        Vec2 { min.x, min.y }, Vec2 { max.x, min.y },
        Vec2 { max.x, max.y }, Vec2 { min.x, max.y }
    };
    fillConvex(points);
}

void Render2D::rectOutline(Vec2 min, Vec2 max) {
    const std::array<Vec2, 4> points {
        Vec2 { min.x, min.y }, Vec2 { max.x, min.y },
        Vec2 { max.x, max.y }, Vec2 { min.x, max.y }
    };
    polyline(points, true);
}

Render2D& Render2D::active() {
    if (s_active == nullptr) {
        throw std::runtime_error("No active Render2D recording context");
    }
    return *s_active;
}

void Render2D::reserveGpuBuffers(std::size_t vertexCount, std::size_t indexCount) {
    if (vertexCount <= m_vertexCapacity && indexCount <= m_indexCapacity) {
        return;
    }
    const std::size_t newVertexCapacity = growCapacity(vertexCount, m_vertexCapacity, 4096);
    const std::size_t newIndexCapacity = growCapacity(indexCount, m_indexCapacity, 8192);
    m_device->waitIdle();
    for (FrameBuffers& buffers : m_frameBuffers) {
        if (buffers.vertices) m_device->destroyBuffer(buffers.vertices);
        if (buffers.indices) m_device->destroyBuffer(buffers.indices);
        buffers.vertices = m_device->createBuffer({ newVertexCapacity * sizeof(Vertex),
            RHI::BufferUsage::Vertex, true, "Render2D vertices" });
        buffers.indices = m_device->createBuffer({ newIndexCapacity * sizeof(std::uint32_t),
            RHI::BufferUsage::Index, true, "Render2D indices" });
    }
    m_vertexCapacity = newVertexCapacity;
    m_indexCapacity = newIndexCapacity;
}

std::uint32_t Render2D::addVertex(Vec2 position) {
    m_vertices.push_back({ position, m_color });
    return static_cast<std::uint32_t>(m_vertices.size() - 1);
}

} // namespace MatterEngine
