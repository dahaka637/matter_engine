#pragma once

#include "Engine/RHI/RHITypes.hpp"
#include <cstddef>
#include <cstdint>
#include <span>

namespace MatterEngine::RHI {

class CommandList {
public:
    virtual ~CommandList() = default;

    virtual void setViewport(Extent2D extent) = 0;
    virtual void setScissor(Extent2D extent) = 0;
    virtual void bindPipeline(PipelineHandle pipeline) = 0;
    virtual void bindVertexBuffer(BufferHandle buffer, std::size_t offset = 0) = 0;
    virtual void bindIndexBuffer(BufferHandle buffer, IndexType type, std::size_t offset = 0) = 0;
    virtual void pushConstants(ShaderStage stages, std::span<const std::byte> data) = 0;
    virtual void drawIndexed(std::uint32_t indexCount, std::uint32_t firstIndex = 0,
        std::int32_t vertexOffset = 0) = 0;
};

} // namespace MatterEngine::RHI
