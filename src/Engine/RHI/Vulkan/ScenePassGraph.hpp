#pragma once

#include <volk.h>

namespace MatterEngine::RHI::Vulkan {

// Estado de sincronizacao (layout atual + estagio/acesso da ultima operacao)
// de uma imagem usada como attachment que persiste entre chamadas de
// renderScene3D*/renderScene3DToSwapchain - o shadow map, os alvos de cor e
// profundidade offscreen, e o depth buffer do caminho direto ao swapchain.
// Cada um desses recursos guarda a propria instancia deste struct (ver
// VulkanDevice::Impl), exatamente como antes um `VkImageLayout` solto era
// guardado por recurso; a diferenca e que agora tambem guarda estagio/acesso,
// entao quem chama transitionSceneAttachment nunca precisa decidir na mao
// "essa e a primeira vez que uso essa imagem?" nem "o que aconteceu com ela
// da ultima vez?" - o proprio estado responde as duas perguntas.
struct SceneAttachmentState3D {
    VkImageLayout layout = VK_IMAGE_LAYOUT_UNDEFINED;
    VkPipelineStageFlags2 stage = VK_PIPELINE_STAGE_2_NONE;
    VkAccessFlags2 access = VK_ACCESS_2_NONE;
};

// Emite a unica VkImageMemoryBarrier2 necessaria para mover `image` de
// `state` (seu estado guardado da ultima transicao) para
// newLayout/newStage/newAccess, e atualiza `state` para refletir o novo
// estado. Quando `state.layout` ainda e VK_IMAGE_LAYOUT_UNDEFINED (a imagem
// nunca foi usada, ou acabou de ser recriada - ver destroySceneShadowTarget/
// destroyScene3DTarget/destroySceneDirectDepth, que resetam o estado para
// {} exatamente por isso), a origem da barreira colapsa automaticamente
// para NONE/NONE: nao ha uso anterior para sincronizar contra.
void transitionSceneAttachment(VkCommandBuffer commandBuffer, VkImage image,
    VkImageAspectFlags aspect, SceneAttachmentState3D& state,
    VkImageLayout newLayout, VkPipelineStageFlags2 newStage,
    VkAccessFlags2 newAccess);

} // namespace MatterEngine::RHI::Vulkan
