#pragma once

#include <cstdint>
#include <limits>

namespace MatterEngine::RHI {

template <typename Tag>
struct Handle {
    static constexpr std::uint32_t InvalidIndex = std::numeric_limits<std::uint32_t>::max();

    std::uint32_t index = InvalidIndex;
    std::uint32_t generation = 0;

    [[nodiscard]] constexpr bool valid() const { return index != InvalidIndex; }
    explicit constexpr operator bool() const { return valid(); }

    friend constexpr bool operator==(Handle, Handle) = default;
};

struct BufferTag;
struct ShaderTag;
struct PipelineTag;
struct TextureTag;

using BufferHandle = Handle<BufferTag>;
using ShaderHandle = Handle<ShaderTag>;
using PipelineHandle = Handle<PipelineTag>;
using TextureHandle = Handle<TextureTag>;

} // namespace MatterEngine::RHI
