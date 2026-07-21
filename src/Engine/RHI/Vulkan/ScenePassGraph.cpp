#include "Engine/RHI/Vulkan/ScenePassGraph.hpp"

namespace MatterEngine::RHI::Vulkan {

void transitionSceneAttachment(VkCommandBuffer commandBuffer, VkImage image,
    VkImageAspectFlags aspect, SceneAttachmentState3D& state,
    VkImageLayout newLayout, VkPipelineStageFlags2 newStage,
    VkAccessFlags2 newAccess) {
    const bool firstUse = state.layout == VK_IMAGE_LAYOUT_UNDEFINED;

    VkImageMemoryBarrier2 barrier { VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER_2 };
    barrier.srcStageMask = firstUse ? VK_PIPELINE_STAGE_2_NONE : state.stage;
    barrier.srcAccessMask = firstUse ? VK_ACCESS_2_NONE : state.access;
    barrier.dstStageMask = newStage;
    barrier.dstAccessMask = newAccess;
    barrier.oldLayout = state.layout;
    barrier.newLayout = newLayout;
    barrier.image = image;
    barrier.subresourceRange.aspectMask = aspect;
    barrier.subresourceRange.levelCount = 1;
    barrier.subresourceRange.layerCount = 1;

    VkDependencyInfo dependency { VK_STRUCTURE_TYPE_DEPENDENCY_INFO };
    dependency.imageMemoryBarrierCount = 1;
    dependency.pImageMemoryBarriers = &barrier;
    vkCmdPipelineBarrier2(commandBuffer, &dependency);

    state.layout = newLayout;
    state.stage = newStage;
    state.access = newAccess;
}

} // namespace MatterEngine::RHI::Vulkan
