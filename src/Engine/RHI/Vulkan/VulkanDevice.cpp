#include "Engine/RHI/Vulkan/VulkanDevice.hpp"

#include "Engine/Core/Log.hpp"
#include "Engine/Core/Version.hpp"
#include "Engine/RHI/Vulkan/ScenePassGraph.hpp"
#include "Engine/Render/SceneLightPacking.hpp"
#include "imgui.h"
#include "imgui_impl_sdl3.h"
#include "imgui_impl_vulkan.h"
#include <SDL3/SDL.h>
#include <SDL3/SDL_vulkan.h>
#include <volk.h>

#ifdef _MSC_VER
#pragma warning(push, 0)
#endif
#define VMA_IMPLEMENTATION
#include <vk_mem_alloc.h>
#ifdef _MSC_VER
#pragma warning(pop)
#endif

#include <algorithm>
#include <array>
#include <bit>
#include <chrono>
#include <cmath>
#include <cstring>
#include <fstream>
#include <limits>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

namespace MatterEngine::RHI::Vulkan {

namespace {

constexpr std::uint32_t TargetVulkanVersion = VK_API_VERSION_1_4;
constexpr std::uint32_t FramesInFlight = 2;
constexpr VkFormat SceneDepthFormat = VK_FORMAT_D32_SFLOAT;
constexpr std::uint32_t SceneShadowMapSize = 2048;
// Alvo intermediario do passe opaco (ceu+mesh) - meia precisao de ponto
// flutuante por canal, para guardar luminancia acima de 1.0 (destaques de sol,
// especular de metal) sem estourar, ate o passe de tonemap (ver
// createScene3DResources / tonemap.frag) comprimir de volta para o destino
// final em UNORM.
constexpr VkFormat SceneHdrColorFormat = VK_FORMAT_R16G16B16A16_SFLOAT;
// Vetores de movimento por pixel (ver scene3d_mesh.frag) - deslocamento em
// UV entre este quadro e o passado, tipicamente uma fracao pequena de 1.0;
// 2 canais de meia precisao de ponto flutuante sobram de folga sem o custo
// de banda de um formato de 4 canais como o HDR.
constexpr VkFormat SceneMotionVectorFormat = VK_FORMAT_R16G16_SFLOAT;

struct alignas(16) SceneUniformGpu {
    // Jitterada quando a cena usa TAA (Fase 6, ver Scene3DFrame::
    // cameraViewProjectionJittered) - usada para rasterizar geometria.
    std::array<float, 16> cameraViewProjection {};
    // VP SEM jitter do quadro atual. A rasterizacao nao usa esta matriz;
    // ela define a grade estavel em que o historico temporal e armazenado e
    // serve ao calculo dos vetores de movimento.
    std::array<float, 16> cameraViewProjectionUnjittered {};
    // Inversa da VP SEM jitter. O ceu procedural nao entra no resolve
    // temporal e precisa permanecer estavel na grade final de pixels.
    std::array<float, 16> inverseCameraViewProjection {};
    // Cascaded Shadow Maps (ver Engine/Math/ShadowCascade.hpp e
    // Scene3DFrame::cascadeViewProjections/cascadeSplits) - mat4 e sempre
    // multiplo de 16 bytes por coluna, entao um array de mat4 nao sofre do
    // "each array element padded to vec4" que um array de float/vec2/vec3
    // sofreria em std140; cascadeSplits already fits in one vec4 (4 floats,
    // ShadowCascadeCount==4) por isso continua um unico campo, nao um array
    // GLSL de escalares.
    std::array<std::array<float, 16>, ShadowCascadeCount>
        cascadeViewProjections {};
    std::array<float, 4> cascadeSplits {};
    // Tamanho (metros) de um texel do mapa de sombra em cada cascata (ver
    // Scene3DFrame::cascadeTexelWorldSizes) - usado pro bias de
    // normal-offset em shadowVisibility (scene3d_mesh.frag). Mesmo
    // raciocinio de cascadeSplits acima: cabe num unico vec4, nao um array
    // GLSL de escalares (evitaria a penalidade de padding do std140).
    std::array<float, 4> cascadeTexelWorldSizes {};
    // Extensao linear near/far em metros por cascata, usada pelo PCSS para
    // converter profundidade normalizada em distancia fisica.
    std::array<float, 4> cascadeDepthRanges {};
    // VP SEM jitter do quadro anterior. Usar a matriz jitterada aqui inclui
    // a propria sequencia Halton no vetor de movimento e faz o historico
    // oscilar entre texels mesmo com camera e objeto perfeitamente parados.
    std::array<float, 16> previousCameraViewProjection {};
    std::array<float, 4> cameraPosition {};
    // x=sombras ligadas, y=quantidade de luzes no SSBO SceneLights,
    // z=historico temporal valido neste quadro, w=luz ambiente.
    std::array<float, 4> settings {};
    // x=mostrar ceu, y=tempo do ceu, z=cobertura de nuvens, w reservado (era
    // a intensidade fixa do sol - agora vem de lights[0] no SSBO).
    std::array<float, 4> skySettings {};
    // x=densidade (taxa exponencial por metro), y=acoplamento altura-distancia,
    // z=opacidade maxima, w reservado - ver FogSettings3D/scene3d_mesh.frag.
    std::array<float, 4> fogSettings {};
    // rgb=cor da neblina, a reservado.
    std::array<float, 4> fogColor {};
    // xy=deslocamento acumulado do vento nas nuvens (unidades de ruido do
    // shader do ceu, ja integrado no tempo pelo lado CPU - ver
    // Scene3DFrame::cloudWindOffset/WindSystem), zw reservado.
    std::array<float, 4> windOffset {};
};

// Transform e parametros variaveis por instancia. Geometria e texturas iguais
// sao agrupadas em um unico vkCmdDrawIndexed, reduzindo milhares de chamadas e
// trocas de estado a poucos batches quando muitos props repetem um asset.
struct alignas(16) SceneMeshInstanceGpu {
    std::array<float, 4> positionScale {};
    std::array<float, 4> orientationX {};
    std::array<float, 4> orientationY {};
    std::array<float, 4> orientationZ {};
    std::array<float, 4> materialAndFlags {};
    // Transformacao do MESMO objeto no quadro anterior - usada so pelo
    // vertex shader da mesh (scene3d_mesh.vert) para computar o vetor de
    // movimento por pixel que o resolve de TAA consome (ver
    // MeshRender3D::previousPosition/previousOrientation e
    // taa_resolve.frag). Objetos parados replicam os mesmos valores acima.
    std::array<float, 4> previousPositionScale {};
    std::array<float, 4> previousOrientationX {};
    std::array<float, 4> previousOrientationY {};
    std::array<float, 4> previousOrientationZ {};
};

static_assert(sizeof(SceneUniformGpu) == 656);
static_assert(sizeof(SceneMeshInstanceGpu) == 144);

// Espelha o bloco "push_constant" de tonemap.frag campo a campo - ver
// ToneMappingSettings3D (Scene3D.hpp) para o que cada campo faz.
struct TonemapPushConstantsGpu {
    float exposure = 1.0f;
    float brightness = 0.0f;
    float contrast = 1.0f;
    float saturation = 1.0f;
};

static_assert(sizeof(TonemapPushConstantsGpu) == 16);

void check(VkResult result, const char* operation) {
    if (result != VK_SUCCESS) {
        throw std::runtime_error(std::string(operation) + " failed with VkResult " + std::to_string(result));
    }
}

VKAPI_ATTR VkBool32 VKAPI_CALL debugCallback(VkDebugUtilsMessageSeverityFlagBitsEXT severity,
    VkDebugUtilsMessageTypeFlagsEXT, const VkDebugUtilsMessengerCallbackDataEXT* callbackData, void*) {
    const std::string message = callbackData != nullptr && callbackData->pMessage != nullptr
        ? callbackData->pMessage
        : "Unknown Vulkan validation message";
    if ((severity & VK_DEBUG_UTILS_MESSAGE_SEVERITY_ERROR_BIT_EXT) != 0) {
        Log::error("Vulkan: " + message);
    } else if ((severity & VK_DEBUG_UTILS_MESSAGE_SEVERITY_WARNING_BIT_EXT) != 0) {
        Log::warn("Vulkan: " + message);
    } else {
        Log::info("Vulkan: " + message);
    }
    return VK_FALSE;
}

bool hasInstanceLayer(const char* requested) {
    std::uint32_t count = 0;
    vkEnumerateInstanceLayerProperties(&count, nullptr);
    std::vector<VkLayerProperties> layers(count);
    vkEnumerateInstanceLayerProperties(&count, layers.data());
    return std::any_of(layers.begin(), layers.end(), [requested](const VkLayerProperties& layer) {
        return std::strcmp(layer.layerName, requested) == 0;
    });
}

bool hasInstanceExtension(const char* requested) {
    std::uint32_t count = 0;
    vkEnumerateInstanceExtensionProperties(nullptr, &count, nullptr);
    std::vector<VkExtensionProperties> extensions(count);
    vkEnumerateInstanceExtensionProperties(nullptr, &count, extensions.data());
    return std::any_of(extensions.begin(), extensions.end(), [requested](const VkExtensionProperties& extension) {
        return std::strcmp(extension.extensionName, requested) == 0;
    });
}

VkBufferUsageFlags bufferUsage(BufferUsage usage) {
    switch (usage) {
    case BufferUsage::Vertex:
        return VK_BUFFER_USAGE_VERTEX_BUFFER_BIT;
    case BufferUsage::Index:
        return VK_BUFFER_USAGE_INDEX_BUFFER_BIT;
    case BufferUsage::Uniform:
        return VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT;
    case BufferUsage::Transfer:
        return VK_BUFFER_USAGE_TRANSFER_SRC_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT;
    }
    return 0;
}

VkShaderStageFlags shaderStages(ShaderStage stages) {
    const auto bits = static_cast<std::uint8_t>(stages);
    VkShaderStageFlags result = 0;
    if ((bits & static_cast<std::uint8_t>(ShaderStage::Vertex)) != 0) {
        result |= VK_SHADER_STAGE_VERTEX_BIT;
    }
    if ((bits & static_cast<std::uint8_t>(ShaderStage::Fragment)) != 0) {
        result |= VK_SHADER_STAGE_FRAGMENT_BIT;
    }
    if ((bits & static_cast<std::uint8_t>(ShaderStage::Compute)) != 0) {
        result |= VK_SHADER_STAGE_COMPUTE_BIT;
    }
    return result;
}

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

VkFormat vertexFormat(VertexFormat format) {
    switch (format) {
    case VertexFormat::Float2:
        return VK_FORMAT_R32G32_SFLOAT;
    case VertexFormat::Float3:
        return VK_FORMAT_R32G32B32_SFLOAT;
    case VertexFormat::Float4:
        return VK_FORMAT_R32G32B32A32_SFLOAT;
    case VertexFormat::UNorm8x4:
        return VK_FORMAT_R8G8B8A8_UNORM;
    }
    return VK_FORMAT_UNDEFINED;
}

template <typename Resource>
std::uint32_t acquireSlot(std::vector<Resource>& resources) {
    for (std::uint32_t i = 0; i < resources.size(); ++i) {
        if (!resources[i].alive) {
            resources[i].alive = true;
            return i;
        }
    }
    resources.emplace_back();
    resources.back().alive = true;
    return static_cast<std::uint32_t>(resources.size() - 1);
}

template <typename Resource, typename HandleType>
Resource& checkedResource(std::vector<Resource>& resources, HandleType handle, const char* kind) {
    if (!handle.valid() || handle.index >= resources.size()) {
        throw std::runtime_error(std::string("Invalid ") + kind + " handle");
    }
    Resource& resource = resources[handle.index];
    if (!resource.alive || resource.generation != handle.generation) {
        throw std::runtime_error(std::string("Stale ") + kind + " handle");
    }
    return resource;
}

} // namespace

class VulkanDevice::Impl final : public CommandList {
public:
    struct QueueFamilies {
        std::uint32_t graphics = std::numeric_limits<std::uint32_t>::max();
        std::uint32_t present = std::numeric_limits<std::uint32_t>::max();

        [[nodiscard]] bool complete() const {
            return graphics != std::numeric_limits<std::uint32_t>::max()
                && present != std::numeric_limits<std::uint32_t>::max();
        }
    };

    struct Frame {
        VkCommandPool commandPool = VK_NULL_HANDLE;
        VkCommandBuffer commandBuffer = VK_NULL_HANDLE;
        VkSemaphore imageAvailable = VK_NULL_HANDLE;
        VkSemaphore renderFinished = VK_NULL_HANDLE;
        VkFence fence = VK_NULL_HANDLE;
        bool timestampWritten = false;
        FramePerformanceMetrics performance;
    };

    struct BufferResource {
        VkBuffer buffer = VK_NULL_HANDLE;
        VmaAllocation allocation = VK_NULL_HANDLE;
        void* mapped = nullptr;
        std::size_t size = 0;
        std::uint32_t generation = 1;
        bool alive = false;
    };

    struct ShaderResource {
        VkShaderModule module = VK_NULL_HANDLE;
        ShaderStage stage = ShaderStage::Vertex;
        std::string entryPoint = "main";
        std::uint32_t generation = 1;
        bool alive = false;
    };

    struct PipelineResource {
        VkPipeline pipeline = VK_NULL_HANDLE;
        VkPipelineLayout layout = VK_NULL_HANDLE;
        std::uint32_t generation = 1;
        bool alive = false;
    };

    struct TextureResource {
        VkImage image = VK_NULL_HANDLE;
        VmaAllocation allocation = VK_NULL_HANDLE;
        VkImageView view = VK_NULL_HANDLE;
        VkExtent2D extent {};
        VkImageLayout layout = VK_IMAGE_LAYOUT_UNDEFINED;
        VkDescriptorSet imguiDescriptor = VK_NULL_HANDLE;
        // Only set for textures created via createTexture2D: a set=1
        // material descriptor (single combined image sampler) the mesh
        // pipeline binds to sample this texture as an albedo map.
        VkDescriptorSet materialDescriptor = VK_NULL_HANDLE;
        std::uint32_t generation = 1;
        bool alive = false;
    };

    void initialize(SDL_Window* newWindow, bool enableVsync, const std::string& applicationName) {
        window = newWindow;
        vsync = enableVsync;

        check(volkInitialize(), "volkInitialize");
        createInstance(applicationName);
        volkLoadInstance(instance);
        createDebugMessenger();

        if (!SDL_Vulkan_CreateSurface(window, instance, nullptr, &surface)) {
            throw std::runtime_error(std::string("SDL_Vulkan_CreateSurface failed: ") + SDL_GetError());
        }

        selectPhysicalDevice();
        createLogicalDevice();
        volkLoadDevice(device);
        createAllocator();
        createFrames();
        createSwapchain();
        createSpriteResources();
        createScene3DResources();

        VkPhysicalDeviceProperties properties {};
        vkGetPhysicalDeviceProperties(physicalDevice, &properties);
        timestampPeriodNanoseconds = properties.limits.timestampPeriod;
        Log::info(std::string("Vulkan 1.4 initialized on ") + properties.deviceName + ".");
    }

    void shutdown() {
        if (device != VK_NULL_HANDLE) {
            vkDeviceWaitIdle(device);
        }

        destroyScene3DResources();
        shutdownImGui();

        if (spritePipeline != VK_NULL_HANDLE) vkDestroyPipeline(device, spritePipeline, nullptr);
        if (spritePipelineLayout != VK_NULL_HANDLE) vkDestroyPipelineLayout(device, spritePipelineLayout, nullptr);
        if (spriteDescriptorPool != VK_NULL_HANDLE) vkDestroyDescriptorPool(device, spriteDescriptorPool, nullptr);
        if (spriteDescriptorSetLayout != VK_NULL_HANDLE) vkDestroyDescriptorSetLayout(device, spriteDescriptorSetLayout, nullptr);
        if (spriteVertexModule != VK_NULL_HANDLE) vkDestroyShaderModule(device, spriteVertexModule, nullptr);
        if (spriteFragmentModule != VK_NULL_HANDLE) vkDestroyShaderModule(device, spriteFragmentModule, nullptr);
        if (nearestSampler != VK_NULL_HANDLE) vkDestroySampler(device, nearestSampler, nullptr);
        spritePipeline = VK_NULL_HANDLE;
        spritePipelineLayout = VK_NULL_HANDLE;
        spriteDescriptorPool = VK_NULL_HANDLE;
        spriteDescriptorSetLayout = VK_NULL_HANDLE;
        spriteVertexModule = VK_NULL_HANDLE;
        spriteFragmentModule = VK_NULL_HANDLE;
        nearestSampler = VK_NULL_HANDLE;

        for (TextureResource& resource : textures) {
            if (resource.alive) {
                vkDestroyImageView(device, resource.view, nullptr);
                vmaDestroyImage(allocator, resource.image, resource.allocation);
            }
        }
        textures.clear();

        for (PipelineResource& resource : pipelines) {
            if (resource.alive) {
                vkDestroyPipeline(device, resource.pipeline, nullptr);
                vkDestroyPipelineLayout(device, resource.layout, nullptr);
            }
        }
        pipelines.clear();
        for (ShaderResource& resource : shaders) {
            if (resource.alive) {
                vkDestroyShaderModule(device, resource.module, nullptr);
            }
        }
        shaders.clear();
        for (BufferResource& resource : buffers) {
            if (resource.alive) {
                vmaDestroyBuffer(allocator, resource.buffer, resource.allocation);
            }
        }
        buffers.clear();

        destroySwapchain();
        for (Frame& frame : frames) {
            if (frame.fence != VK_NULL_HANDLE) vkDestroyFence(device, frame.fence, nullptr);
            if (frame.imageAvailable != VK_NULL_HANDLE) vkDestroySemaphore(device, frame.imageAvailable, nullptr);
            if (frame.renderFinished != VK_NULL_HANDLE) vkDestroySemaphore(device, frame.renderFinished, nullptr);
            if (frame.commandPool != VK_NULL_HANDLE) vkDestroyCommandPool(device, frame.commandPool, nullptr);
        }
        frames = {};
        if (frameTimestampQueryPool != VK_NULL_HANDLE) {
            vkDestroyQueryPool(device, frameTimestampQueryPool, nullptr);
            frameTimestampQueryPool = VK_NULL_HANDLE;
        }

        if (allocator != VK_NULL_HANDLE) {
            vmaDestroyAllocator(allocator);
            allocator = VK_NULL_HANDLE;
        }
        if (device != VK_NULL_HANDLE) {
            vkDestroyDevice(device, nullptr);
            device = VK_NULL_HANDLE;
        }
        if (surface != VK_NULL_HANDLE) {
            SDL_Vulkan_DestroySurface(instance, surface, nullptr);
            surface = VK_NULL_HANDLE;
        }
        if (debugMessenger != VK_NULL_HANDLE && vkDestroyDebugUtilsMessengerEXT != nullptr) {
            vkDestroyDebugUtilsMessengerEXT(instance, debugMessenger, nullptr);
            debugMessenger = VK_NULL_HANDLE;
        }
        if (instance != VK_NULL_HANDLE) {
            vkDestroyInstance(instance, nullptr);
            instance = VK_NULL_HANDLE;
        }
        window = nullptr;
    }

    BufferHandle createBuffer(const BufferDesc& desc) {
        if (desc.size == 0) {
            throw std::runtime_error("Cannot create an empty RHI buffer");
        }

        VkBufferCreateInfo bufferInfo { VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO };
        bufferInfo.size = desc.size;
        bufferInfo.usage = bufferUsage(desc.usage);
        bufferInfo.sharingMode = VK_SHARING_MODE_EXCLUSIVE;

        VmaAllocationCreateInfo allocationInfo {};
        allocationInfo.usage = desc.cpuVisible ? VMA_MEMORY_USAGE_AUTO_PREFER_HOST : VMA_MEMORY_USAGE_AUTO_PREFER_DEVICE;
        if (desc.cpuVisible) {
            allocationInfo.flags = VMA_ALLOCATION_CREATE_HOST_ACCESS_SEQUENTIAL_WRITE_BIT
                | VMA_ALLOCATION_CREATE_MAPPED_BIT;
        }

        VmaAllocationInfo createdAllocation {};
        VkBuffer buffer = VK_NULL_HANDLE;
        VmaAllocation allocation = VK_NULL_HANDLE;
        check(vmaCreateBuffer(allocator, &bufferInfo, &allocationInfo, &buffer, &allocation, &createdAllocation),
            "vmaCreateBuffer");

        const std::uint32_t index = acquireSlot(buffers);
        BufferResource& resource = buffers[index];
        resource.buffer = buffer;
        resource.allocation = allocation;
        resource.mapped = createdAllocation.pMappedData;
        resource.size = desc.size;
        return { index, resource.generation };
    }

    ShaderHandle createShader(const ShaderDesc& desc) {
        if (desc.spirv.empty()) {
            throw std::runtime_error("Cannot create an empty shader module");
        }
        VkShaderModuleCreateInfo createInfo { VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO };
        createInfo.codeSize = desc.spirv.size() * sizeof(std::uint32_t);
        createInfo.pCode = desc.spirv.data();

        VkShaderModule module = VK_NULL_HANDLE;
        check(vkCreateShaderModule(device, &createInfo, nullptr, &module), "vkCreateShaderModule");

        const std::uint32_t index = acquireSlot(shaders);
        ShaderResource& resource = shaders[index];
        resource.module = module;
        resource.stage = desc.stage;
        resource.entryPoint = desc.entryPoint != nullptr ? desc.entryPoint : "main";
        return { index, resource.generation };
    }

    PipelineHandle createGraphicsPipeline(const GraphicsPipelineDesc& desc) {
        ShaderResource& vertex = checkedResource(shaders, desc.vertexShader, "vertex shader");
        ShaderResource& fragment = checkedResource(shaders, desc.fragmentShader, "fragment shader");

        std::array<VkPipelineShaderStageCreateInfo, 2> stages {};
        stages[0].sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
        stages[0].stage = VK_SHADER_STAGE_VERTEX_BIT;
        stages[0].module = vertex.module;
        stages[0].pName = vertex.entryPoint.c_str();
        stages[1].sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
        stages[1].stage = VK_SHADER_STAGE_FRAGMENT_BIT;
        stages[1].module = fragment.module;
        stages[1].pName = fragment.entryPoint.c_str();

        VkVertexInputBindingDescription binding {};
        binding.binding = 0;
        binding.stride = desc.vertexStride;
        binding.inputRate = VK_VERTEX_INPUT_RATE_VERTEX;

        std::vector<VkVertexInputAttributeDescription> attributes;
        attributes.reserve(desc.attributes.size());
        for (const VertexAttribute& attribute : desc.attributes) {
            attributes.push_back({ attribute.location, 0, vertexFormat(attribute.format), attribute.offset });
        }

        VkPipelineVertexInputStateCreateInfo vertexInput { VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO };
        if (desc.vertexStride > 0) {
            vertexInput.vertexBindingDescriptionCount = 1;
            vertexInput.pVertexBindingDescriptions = &binding;
        }
        vertexInput.vertexAttributeDescriptionCount = static_cast<std::uint32_t>(attributes.size());
        vertexInput.pVertexAttributeDescriptions = attributes.data();

        VkPipelineInputAssemblyStateCreateInfo inputAssembly { VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO };
        inputAssembly.topology = VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST;

        VkPipelineViewportStateCreateInfo viewport { VK_STRUCTURE_TYPE_PIPELINE_VIEWPORT_STATE_CREATE_INFO };
        viewport.viewportCount = 1;
        viewport.scissorCount = 1;

        VkPipelineRasterizationStateCreateInfo rasterizer { VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO };
        rasterizer.polygonMode = VK_POLYGON_MODE_FILL;
        rasterizer.cullMode = VK_CULL_MODE_NONE;
        rasterizer.frontFace = VK_FRONT_FACE_COUNTER_CLOCKWISE;
        rasterizer.lineWidth = 1.0f;

        VkPipelineMultisampleStateCreateInfo multisample { VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO };
        multisample.rasterizationSamples = VK_SAMPLE_COUNT_1_BIT;

        VkPipelineDepthStencilStateCreateInfo depth { VK_STRUCTURE_TYPE_PIPELINE_DEPTH_STENCIL_STATE_CREATE_INFO };
        depth.depthTestEnable = desc.depthTest ? VK_TRUE : VK_FALSE;
        depth.depthWriteEnable = desc.depthTest ? VK_TRUE : VK_FALSE;
        depth.depthCompareOp = VK_COMPARE_OP_GREATER_OR_EQUAL;

        VkPipelineColorBlendAttachmentState blendAttachment {};
        blendAttachment.colorWriteMask = VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT
            | VK_COLOR_COMPONENT_B_BIT | VK_COLOR_COMPONENT_A_BIT;
        blendAttachment.blendEnable = desc.alphaBlend ? VK_TRUE : VK_FALSE;
        blendAttachment.srcColorBlendFactor = VK_BLEND_FACTOR_SRC_ALPHA;
        blendAttachment.dstColorBlendFactor = VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA;
        blendAttachment.colorBlendOp = VK_BLEND_OP_ADD;
        blendAttachment.srcAlphaBlendFactor = VK_BLEND_FACTOR_ONE;
        blendAttachment.dstAlphaBlendFactor = VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA;
        blendAttachment.alphaBlendOp = VK_BLEND_OP_ADD;

        VkPipelineColorBlendStateCreateInfo blend { VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO };
        blend.attachmentCount = 1;
        blend.pAttachments = &blendAttachment;

        const std::array<VkDynamicState, 2> dynamics { VK_DYNAMIC_STATE_VIEWPORT, VK_DYNAMIC_STATE_SCISSOR };
        VkPipelineDynamicStateCreateInfo dynamic { VK_STRUCTURE_TYPE_PIPELINE_DYNAMIC_STATE_CREATE_INFO };
        dynamic.dynamicStateCount = static_cast<std::uint32_t>(dynamics.size());
        dynamic.pDynamicStates = dynamics.data();

        VkPushConstantRange pushRange {};
        pushRange.stageFlags = shaderStages(desc.pushConstantStages);
        pushRange.size = desc.pushConstantSize;

        VkPipelineLayoutCreateInfo layoutInfo { VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO };
        if (desc.pushConstantSize > 0) {
            layoutInfo.pushConstantRangeCount = 1;
            layoutInfo.pPushConstantRanges = &pushRange;
        }

        VkPipelineLayout layout = VK_NULL_HANDLE;
        check(vkCreatePipelineLayout(device, &layoutInfo, nullptr, &layout), "vkCreatePipelineLayout");

        VkPipelineRenderingCreateInfo renderingInfo { VK_STRUCTURE_TYPE_PIPELINE_RENDERING_CREATE_INFO };
        renderingInfo.colorAttachmentCount = 1;
        renderingInfo.pColorAttachmentFormats = &swapchainFormat;

        VkGraphicsPipelineCreateInfo pipelineInfo { VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO };
        pipelineInfo.pNext = &renderingInfo;
        pipelineInfo.stageCount = static_cast<std::uint32_t>(stages.size());
        pipelineInfo.pStages = stages.data();
        pipelineInfo.pVertexInputState = &vertexInput;
        pipelineInfo.pInputAssemblyState = &inputAssembly;
        pipelineInfo.pViewportState = &viewport;
        pipelineInfo.pRasterizationState = &rasterizer;
        pipelineInfo.pMultisampleState = &multisample;
        pipelineInfo.pDepthStencilState = &depth;
        pipelineInfo.pColorBlendState = &blend;
        pipelineInfo.pDynamicState = &dynamic;
        pipelineInfo.layout = layout;

        VkPipeline pipeline = VK_NULL_HANDLE;
        const VkResult result = vkCreateGraphicsPipelines(device, VK_NULL_HANDLE, 1, &pipelineInfo, nullptr, &pipeline);
        if (result != VK_SUCCESS) {
            vkDestroyPipelineLayout(device, layout, nullptr);
            check(result, "vkCreateGraphicsPipelines");
        }

        const std::uint32_t index = acquireSlot(pipelines);
        PipelineResource& resource = pipelines[index];
        resource.pipeline = pipeline;
        resource.layout = layout;
        return { index, resource.generation };
    }

    TextureHandle createRenderTarget(const TextureDesc& desc) {
        if (desc.extent.width == 0 || desc.extent.height == 0) {
            throw std::runtime_error("Cannot create a zero-sized render target");
        }

        VkImageCreateInfo imageInfo { VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO };
        imageInfo.imageType = VK_IMAGE_TYPE_2D;
        imageInfo.format = swapchainFormat;
        imageInfo.extent = { desc.extent.width, desc.extent.height, 1 };
        imageInfo.mipLevels = 1;
        imageInfo.arrayLayers = 1;
        imageInfo.samples = VK_SAMPLE_COUNT_1_BIT;
        imageInfo.tiling = VK_IMAGE_TILING_OPTIMAL;
        imageInfo.usage = VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT;
        imageInfo.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
        imageInfo.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;

        VmaAllocationCreateInfo allocationInfo {};
        allocationInfo.usage = VMA_MEMORY_USAGE_AUTO_PREFER_DEVICE;

        VkImage image = VK_NULL_HANDLE;
        VmaAllocation allocation = VK_NULL_HANDLE;
        check(vmaCreateImage(allocator, &imageInfo, &allocationInfo, &image, &allocation, nullptr), "vmaCreateImage");

        VkImageViewCreateInfo viewInfo { VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO };
        viewInfo.image = image;
        viewInfo.viewType = VK_IMAGE_VIEW_TYPE_2D;
        viewInfo.format = swapchainFormat;
        viewInfo.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
        viewInfo.subresourceRange.levelCount = 1;
        viewInfo.subresourceRange.layerCount = 1;
        VkImageView view = VK_NULL_HANDLE;
        const VkResult viewResult = vkCreateImageView(device, &viewInfo, nullptr, &view);
        if (viewResult != VK_SUCCESS) {
            vmaDestroyImage(allocator, image, allocation);
            check(viewResult, "vkCreateImageView(renderTarget)");
        }

        const std::uint32_t index = acquireSlot(textures);
        TextureResource& resource = textures[index];
        resource.image = image;
        resource.allocation = allocation;
        resource.view = view;
        resource.extent = { desc.extent.width, desc.extent.height };
        resource.layout = VK_IMAGE_LAYOUT_UNDEFINED;
        return { index, resource.generation };
    }

    std::uint64_t createImGuiTexture(Extent2D extent, std::span<const std::byte> rgbaPixels) {
        if (!imguiInitialized) {
            throw std::runtime_error("Cannot create an ImGui texture before ImGui initialization");
        }
        if (extent.width == 0 || extent.height == 0) {
            throw std::runtime_error("Cannot create a zero-sized ImGui texture");
        }
        const std::size_t expectedSize = static_cast<std::size_t>(extent.width)
            * static_cast<std::size_t>(extent.height) * 4;
        if (rgbaPixels.size() != expectedSize) {
            throw std::runtime_error("ImGui texture data does not match its RGBA extent");
        }

        VkBuffer stagingBuffer = VK_NULL_HANDLE;
        VmaAllocation stagingAllocation = VK_NULL_HANDLE;
        VkImage image = VK_NULL_HANDLE;
        VmaAllocation imageAllocation = VK_NULL_HANDLE;
        VkImageView view = VK_NULL_HANDLE;
        VkDescriptorSet imguiDescriptor = VK_NULL_HANDLE;
        VkCommandPool commandPool = VK_NULL_HANDLE;
        VkFence uploadFence = VK_NULL_HANDLE;

        const auto cleanupTemporary = [&] {
            if (uploadFence != VK_NULL_HANDLE) vkDestroyFence(device, uploadFence, nullptr);
            if (commandPool != VK_NULL_HANDLE) vkDestroyCommandPool(device, commandPool, nullptr);
            if (stagingBuffer != VK_NULL_HANDLE) vmaDestroyBuffer(allocator, stagingBuffer, stagingAllocation);
        };
        const auto cleanupImage = [&] {
            if (imguiDescriptor != VK_NULL_HANDLE && imguiInitialized) {
                ImGui_ImplVulkan_RemoveTexture(imguiDescriptor);
            }
            if (view != VK_NULL_HANDLE) vkDestroyImageView(device, view, nullptr);
            if (image != VK_NULL_HANDLE) vmaDestroyImage(allocator, image, imageAllocation);
        };

        try {
            VkBufferCreateInfo stagingInfo { VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO };
            stagingInfo.size = expectedSize;
            stagingInfo.usage = VK_BUFFER_USAGE_TRANSFER_SRC_BIT;
            stagingInfo.sharingMode = VK_SHARING_MODE_EXCLUSIVE;

            VmaAllocationCreateInfo stagingAllocationInfo {};
            stagingAllocationInfo.usage = VMA_MEMORY_USAGE_AUTO_PREFER_HOST;
            stagingAllocationInfo.flags = VMA_ALLOCATION_CREATE_HOST_ACCESS_SEQUENTIAL_WRITE_BIT
                | VMA_ALLOCATION_CREATE_MAPPED_BIT;
            VmaAllocationInfo mappedInfo {};
            check(vmaCreateBuffer(allocator, &stagingInfo, &stagingAllocationInfo,
                &stagingBuffer, &stagingAllocation, &mappedInfo), "vmaCreateBuffer(UI staging)");
            std::memcpy(mappedInfo.pMappedData, rgbaPixels.data(), expectedSize);
            check(vmaFlushAllocation(allocator, stagingAllocation, 0, expectedSize),
                "vmaFlushAllocation(UI staging)");

            VkImageCreateInfo imageInfo { VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO };
            imageInfo.imageType = VK_IMAGE_TYPE_2D;
            imageInfo.format = VK_FORMAT_R8G8B8A8_UNORM;
            imageInfo.extent = { extent.width, extent.height, 1 };
            imageInfo.mipLevels = 1;
            imageInfo.arrayLayers = 1;
            imageInfo.samples = VK_SAMPLE_COUNT_1_BIT;
            imageInfo.tiling = VK_IMAGE_TILING_OPTIMAL;
            imageInfo.usage = VK_IMAGE_USAGE_TRANSFER_DST_BIT | VK_IMAGE_USAGE_SAMPLED_BIT;
            imageInfo.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
            imageInfo.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;

            VmaAllocationCreateInfo imageAllocationInfo {};
            imageAllocationInfo.usage = VMA_MEMORY_USAGE_AUTO_PREFER_DEVICE;
            check(vmaCreateImage(allocator, &imageInfo, &imageAllocationInfo,
                &image, &imageAllocation, nullptr), "vmaCreateImage(UI texture)");

            VkImageViewCreateInfo viewInfo { VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO };
            viewInfo.image = image;
            viewInfo.viewType = VK_IMAGE_VIEW_TYPE_2D;
            viewInfo.format = imageInfo.format;
            viewInfo.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
            viewInfo.subresourceRange.levelCount = 1;
            viewInfo.subresourceRange.layerCount = 1;
            check(vkCreateImageView(device, &viewInfo, nullptr, &view), "vkCreateImageView(UI texture)");

            VkCommandPoolCreateInfo poolInfo { VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO };
            poolInfo.flags = VK_COMMAND_POOL_CREATE_TRANSIENT_BIT;
            poolInfo.queueFamilyIndex = queueFamilies.graphics;
            check(vkCreateCommandPool(device, &poolInfo, nullptr, &commandPool), "vkCreateCommandPool(UI upload)");

            VkCommandBuffer commandBuffer = VK_NULL_HANDLE;
            VkCommandBufferAllocateInfo commandInfo { VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO };
            commandInfo.commandPool = commandPool;
            commandInfo.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
            commandInfo.commandBufferCount = 1;
            check(vkAllocateCommandBuffers(device, &commandInfo, &commandBuffer),
                "vkAllocateCommandBuffers(UI upload)");

            VkCommandBufferBeginInfo beginInfo { VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO };
            beginInfo.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
            check(vkBeginCommandBuffer(commandBuffer, &beginInfo), "vkBeginCommandBuffer(UI upload)");

            VkImageMemoryBarrier2 toTransfer { VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER_2 };
            toTransfer.srcStageMask = VK_PIPELINE_STAGE_2_NONE;
            toTransfer.dstStageMask = VK_PIPELINE_STAGE_2_COPY_BIT;
            toTransfer.dstAccessMask = VK_ACCESS_2_TRANSFER_WRITE_BIT;
            toTransfer.oldLayout = VK_IMAGE_LAYOUT_UNDEFINED;
            toTransfer.newLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
            toTransfer.image = image;
            toTransfer.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
            toTransfer.subresourceRange.levelCount = 1;
            toTransfer.subresourceRange.layerCount = 1;
            VkDependencyInfo dependency { VK_STRUCTURE_TYPE_DEPENDENCY_INFO };
            dependency.imageMemoryBarrierCount = 1;
            dependency.pImageMemoryBarriers = &toTransfer;
            vkCmdPipelineBarrier2(commandBuffer, &dependency);

            VkBufferImageCopy copy {};
            copy.imageSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
            copy.imageSubresource.layerCount = 1;
            copy.imageExtent = { extent.width, extent.height, 1 };
            vkCmdCopyBufferToImage(commandBuffer, stagingBuffer, image,
                VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, 1, &copy);

            VkImageMemoryBarrier2 toShaderRead { VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER_2 };
            toShaderRead.srcStageMask = VK_PIPELINE_STAGE_2_COPY_BIT;
            toShaderRead.srcAccessMask = VK_ACCESS_2_TRANSFER_WRITE_BIT;
            toShaderRead.dstStageMask = VK_PIPELINE_STAGE_2_FRAGMENT_SHADER_BIT;
            toShaderRead.dstAccessMask = VK_ACCESS_2_SHADER_SAMPLED_READ_BIT;
            toShaderRead.oldLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
            toShaderRead.newLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
            toShaderRead.image = image;
            toShaderRead.subresourceRange = toTransfer.subresourceRange;
            dependency.pImageMemoryBarriers = &toShaderRead;
            vkCmdPipelineBarrier2(commandBuffer, &dependency);
            check(vkEndCommandBuffer(commandBuffer), "vkEndCommandBuffer(UI upload)");

            VkFenceCreateInfo fenceInfo { VK_STRUCTURE_TYPE_FENCE_CREATE_INFO };
            check(vkCreateFence(device, &fenceInfo, nullptr, &uploadFence), "vkCreateFence(UI upload)");
            VkCommandBufferSubmitInfo commandSubmit { VK_STRUCTURE_TYPE_COMMAND_BUFFER_SUBMIT_INFO };
            commandSubmit.commandBuffer = commandBuffer;
            VkSubmitInfo2 submit { VK_STRUCTURE_TYPE_SUBMIT_INFO_2 };
            submit.commandBufferInfoCount = 1;
            submit.pCommandBufferInfos = &commandSubmit;
            check(vkQueueSubmit2(graphicsQueue, 1, &submit, uploadFence), "vkQueueSubmit2(UI upload)");
            check(vkWaitForFences(device, 1, &uploadFence, VK_TRUE, UINT64_MAX),
                "vkWaitForFences(UI upload)");

            imguiDescriptor = ImGui_ImplVulkan_AddTexture(
                nearestSampler, view, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
            if (imguiDescriptor == VK_NULL_HANDLE) {
                throw std::runtime_error("ImGui failed to allocate a descriptor for a UI texture");
            }

            const std::uint32_t index = acquireSlot(textures);
            TextureResource& resource = textures[index];
            resource.image = image;
            resource.allocation = imageAllocation;
            resource.view = view;
            resource.extent = { extent.width, extent.height };
            resource.layout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
            resource.imguiDescriptor = imguiDescriptor;

            image = VK_NULL_HANDLE;
            imageAllocation = VK_NULL_HANDLE;
            view = VK_NULL_HANDLE;
            const std::uint64_t textureId = static_cast<std::uint64_t>(
                reinterpret_cast<std::uintptr_t>(imguiDescriptor));
            imguiDescriptor = VK_NULL_HANDLE;
            cleanupTemporary();
            uploadFence = VK_NULL_HANDLE;
            commandPool = VK_NULL_HANDLE;
            stagingBuffer = VK_NULL_HANDLE;

            return textureId;
        } catch (...) {
            cleanupTemporary();
            cleanupImage();
            throw;
        }
    }

    // A real sampled asset texture: staging-buffer upload identical in
    // shape to createImGuiTexture above, but registers a set=1 material
    // descriptor (see createScene3DResources) instead of an ImGui one, so
    // the mesh pipeline can sample it as an albedo map.
    TextureHandle createTexture2D(Extent2D extent, std::span<const std::byte> rgbaPixels) {
        if (extent.width == 0 || extent.height == 0) {
            throw std::runtime_error("Cannot create a zero-sized texture");
        }
        const std::size_t expectedSize = static_cast<std::size_t>(extent.width)
            * static_cast<std::size_t>(extent.height) * 4;
        if (rgbaPixels.size() != expectedSize) {
            throw std::runtime_error("Texture2D data does not match its RGBA extent");
        }

        VkBuffer stagingBuffer = VK_NULL_HANDLE;
        VmaAllocation stagingAllocation = VK_NULL_HANDLE;
        VkImage image = VK_NULL_HANDLE;
        VmaAllocation imageAllocation = VK_NULL_HANDLE;
        VkImageView view = VK_NULL_HANDLE;
        VkCommandPool commandPool = VK_NULL_HANDLE;
        VkFence uploadFence = VK_NULL_HANDLE;

        const auto cleanupTemporary = [&] {
            if (uploadFence != VK_NULL_HANDLE) vkDestroyFence(device, uploadFence, nullptr);
            if (commandPool != VK_NULL_HANDLE) vkDestroyCommandPool(device, commandPool, nullptr);
            if (stagingBuffer != VK_NULL_HANDLE) vmaDestroyBuffer(allocator, stagingBuffer, stagingAllocation);
        };
        const auto cleanupImage = [&] {
            if (view != VK_NULL_HANDLE) vkDestroyImageView(device, view, nullptr);
            if (image != VK_NULL_HANDLE) vmaDestroyImage(allocator, image, imageAllocation);
        };

        try {
            VkBufferCreateInfo stagingInfo { VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO };
            stagingInfo.size = expectedSize;
            stagingInfo.usage = VK_BUFFER_USAGE_TRANSFER_SRC_BIT;
            stagingInfo.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
            VmaAllocationCreateInfo stagingAllocationInfo {};
            stagingAllocationInfo.usage = VMA_MEMORY_USAGE_AUTO_PREFER_HOST;
            stagingAllocationInfo.flags = VMA_ALLOCATION_CREATE_HOST_ACCESS_SEQUENTIAL_WRITE_BIT
                | VMA_ALLOCATION_CREATE_MAPPED_BIT;
            VmaAllocationInfo mappedInfo {};
            check(vmaCreateBuffer(allocator, &stagingInfo, &stagingAllocationInfo,
                &stagingBuffer, &stagingAllocation, &mappedInfo), "vmaCreateBuffer(texture staging)");
            std::memcpy(mappedInfo.pMappedData, rgbaPixels.data(), expectedSize);
            check(vmaFlushAllocation(allocator, stagingAllocation, 0, expectedSize),
                "vmaFlushAllocation(texture staging)");

            // Cadeia completa de mip (nao so o nivel 0) - sem isso, amostrar
            // esta textura de longe/em angulo raso pula direto pra
            // minificacao severa: cada texel da tela deveria misturar
            // dezenas de texels da fonte, mas so existe o nivel de
            // resolucao total pra escolher, entao o resultado e ruido de
            // alta frequencia que muda a cada quadro (o chao do laboratorio
            // gera um padrao ligeiramente diferente conforme o jitter
            // sub-pixel do TAA desloca a amostra). Essa e a causa raiz do
            // serrilhado/moire original que motivou toda a Fase 6, e de boa
            // parte do "tremer" residual: o clamp de vizinhanca do resolve
            // de TAA (ver taa_resolve.frag) tenta absorver essa variancia
            // enorme quadro a quadro, mas nao existe reprojecao que arrume
            // um sinal que ja nasceu sem filtragem correta. std::bit_width
            // de uma dimensao equivale a floor(log2(dimensao))+1 pra
            // qualquer inteiro positivo - a contagem padrao de niveis de
            // mip (1x1 no topo da cadeia).
            const std::uint32_t mipLevels = std::bit_width(
                std::max(extent.width, extent.height));

            VkImageCreateInfo imageInfo { VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO };
            imageInfo.imageType = VK_IMAGE_TYPE_2D;
            imageInfo.format = VK_FORMAT_R8G8B8A8_UNORM;
            imageInfo.extent = { extent.width, extent.height, 1 };
            imageInfo.mipLevels = mipLevels;
            imageInfo.arrayLayers = 1;
            imageInfo.samples = VK_SAMPLE_COUNT_1_BIT;
            imageInfo.tiling = VK_IMAGE_TILING_OPTIMAL;
            // TRANSFER_SRC_BIT alem de DST/SAMPLED: cada nivel da cadeia
            // serve de fonte pro blit que gera o nivel seguinte (ver o loop
            // de geracao de mip abaixo).
            imageInfo.usage = VK_IMAGE_USAGE_TRANSFER_DST_BIT
                | VK_IMAGE_USAGE_TRANSFER_SRC_BIT | VK_IMAGE_USAGE_SAMPLED_BIT;
            imageInfo.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
            imageInfo.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
            VmaAllocationCreateInfo imageAllocationInfo {};
            imageAllocationInfo.usage = VMA_MEMORY_USAGE_AUTO_PREFER_DEVICE;
            check(vmaCreateImage(allocator, &imageInfo, &imageAllocationInfo,
                &image, &imageAllocation, nullptr), "vmaCreateImage(texture2D)");

            VkImageViewCreateInfo viewInfo { VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO };
            viewInfo.image = image;
            viewInfo.viewType = VK_IMAGE_VIEW_TYPE_2D;
            viewInfo.format = imageInfo.format;
            viewInfo.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
            viewInfo.subresourceRange.levelCount = mipLevels;
            viewInfo.subresourceRange.layerCount = 1;
            check(vkCreateImageView(device, &viewInfo, nullptr, &view), "vkCreateImageView(texture2D)");

            VkCommandPoolCreateInfo poolInfo { VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO };
            poolInfo.flags = VK_COMMAND_POOL_CREATE_TRANSIENT_BIT;
            poolInfo.queueFamilyIndex = queueFamilies.graphics;
            check(vkCreateCommandPool(device, &poolInfo, nullptr, &commandPool),
                "vkCreateCommandPool(texture upload)");

            VkCommandBuffer commandBuffer = VK_NULL_HANDLE;
            VkCommandBufferAllocateInfo commandInfo { VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO };
            commandInfo.commandPool = commandPool;
            commandInfo.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
            commandInfo.commandBufferCount = 1;
            check(vkAllocateCommandBuffers(device, &commandInfo, &commandBuffer),
                "vkAllocateCommandBuffers(texture upload)");

            VkCommandBufferBeginInfo beginInfo { VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO };
            beginInfo.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
            check(vkBeginCommandBuffer(commandBuffer, &beginInfo), "vkBeginCommandBuffer(texture upload)");

            VkImageMemoryBarrier2 toTransfer { VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER_2 };
            toTransfer.srcStageMask = VK_PIPELINE_STAGE_2_NONE;
            toTransfer.dstStageMask = VK_PIPELINE_STAGE_2_COPY_BIT;
            toTransfer.dstAccessMask = VK_ACCESS_2_TRANSFER_WRITE_BIT;
            toTransfer.oldLayout = VK_IMAGE_LAYOUT_UNDEFINED;
            toTransfer.newLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
            toTransfer.image = image;
            toTransfer.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
            toTransfer.subresourceRange.levelCount = 1;
            toTransfer.subresourceRange.layerCount = 1;
            VkDependencyInfo dependency { VK_STRUCTURE_TYPE_DEPENDENCY_INFO };
            dependency.imageMemoryBarrierCount = 1;
            dependency.pImageMemoryBarriers = &toTransfer;
            vkCmdPipelineBarrier2(commandBuffer, &dependency);

            VkBufferImageCopy copy {};
            copy.imageSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
            copy.imageSubresource.layerCount = 1;
            copy.imageExtent = { extent.width, extent.height, 1 };
            vkCmdCopyBufferToImage(commandBuffer, stagingBuffer, image,
                VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, 1, &copy);

            // Transiciona um unico nivel de mip, emitindo sua propria
            // VkDependencyInfo (nao reaproveita `dependency`/`toTransfer` de
            // cima porque o loop abaixo precisa de niveis/estagios/layouts
            // diferentes por chamada).
            const auto transitionMipLevel = [&](std::uint32_t level,
                    VkImageLayout oldLayout, VkImageLayout newLayout,
                    VkPipelineStageFlags2 srcStage, VkAccessFlags2 srcAccess,
                    VkPipelineStageFlags2 dstStage, VkAccessFlags2 dstAccess) {
                VkImageMemoryBarrier2 barrier { VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER_2 };
                barrier.image = image;
                barrier.subresourceRange = { VK_IMAGE_ASPECT_COLOR_BIT, level, 1, 0, 1 };
                barrier.oldLayout = oldLayout;
                barrier.newLayout = newLayout;
                barrier.srcStageMask = srcStage;
                barrier.srcAccessMask = srcAccess;
                barrier.dstStageMask = dstStage;
                barrier.dstAccessMask = dstAccess;
                VkDependencyInfo levelDependency { VK_STRUCTURE_TYPE_DEPENDENCY_INFO };
                levelDependency.imageMemoryBarrierCount = 1;
                levelDependency.pImageMemoryBarriers = &barrier;
                vkCmdPipelineBarrier2(commandBuffer, &levelDependency);
            };

            if (mipLevels == 1) {
                // Sem cadeia pra gerar (ex.: a textura branca 1x1 default) -
                // so libera o unico nivel pra leitura, como antes desta
                // mudanca.
                transitionMipLevel(0, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
                    VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
                    VK_PIPELINE_STAGE_2_COPY_BIT, VK_ACCESS_2_TRANSFER_WRITE_BIT,
                    VK_PIPELINE_STAGE_2_FRAGMENT_SHADER_BIT, VK_ACCESS_2_SHADER_SAMPLED_READ_BIT);
            } else {
                // Gera a cadeia de mip via blit progressivo nivel a nivel
                // (tecnica padrao Vulkan - ver "Generating Mipmaps" no
                // tutorial oficial). R8G8B8A8_UNORM com filtro linear em
                // blit e suporte obrigatorio no conjunto minimo de qualquer
                // GPU Vulkan (tabela de formatos obrigatorios da spec),
                // entao nao precisa de checagem de VkFormatProperties em
                // runtime aqui.
                int32_t mipWidth = static_cast<int32_t>(extent.width);
                int32_t mipHeight = static_cast<int32_t>(extent.height);
                transitionMipLevel(0, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
                    VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,
                    VK_PIPELINE_STAGE_2_COPY_BIT, VK_ACCESS_2_TRANSFER_WRITE_BIT,
                    VK_PIPELINE_STAGE_2_BLIT_BIT, VK_ACCESS_2_TRANSFER_READ_BIT);

                for (std::uint32_t level = 1; level < mipLevels; ++level) {
                    const int32_t nextWidth = mipWidth > 1 ? mipWidth / 2 : 1;
                    const int32_t nextHeight = mipHeight > 1 ? mipHeight / 2 : 1;
                    const bool isLastLevel = (level + 1 == mipLevels);

                    transitionMipLevel(level, VK_IMAGE_LAYOUT_UNDEFINED,
                        VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
                        VK_PIPELINE_STAGE_2_NONE, VK_ACCESS_2_NONE,
                        VK_PIPELINE_STAGE_2_BLIT_BIT, VK_ACCESS_2_TRANSFER_WRITE_BIT);

                    VkImageBlit blit {};
                    blit.srcSubresource = { VK_IMAGE_ASPECT_COLOR_BIT, level - 1, 0, 1 };
                    blit.srcOffsets[1] = { mipWidth, mipHeight, 1 };
                    blit.dstSubresource = { VK_IMAGE_ASPECT_COLOR_BIT, level, 0, 1 };
                    blit.dstOffsets[1] = { nextWidth, nextHeight, 1 };
                    vkCmdBlitImage(commandBuffer, image, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,
                        image, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, 1, &blit, VK_FILTER_LINEAR);

                    // O nivel anterior ja serviu de fonte deste blit - libera
                    // pra leitura no shader.
                    transitionMipLevel(level - 1, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,
                        VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
                        VK_PIPELINE_STAGE_2_BLIT_BIT, VK_ACCESS_2_TRANSFER_READ_BIT,
                        VK_PIPELINE_STAGE_2_FRAGMENT_SHADER_BIT, VK_ACCESS_2_SHADER_SAMPLED_READ_BIT);
                    // Este nivel vira fonte da proxima iteracao - a nao ser
                    // que seja o ultimo, caso em que vai direto pra leitura.
                    transitionMipLevel(level, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
                        isLastLevel ? VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL
                            : VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,
                        VK_PIPELINE_STAGE_2_BLIT_BIT, VK_ACCESS_2_TRANSFER_WRITE_BIT,
                        isLastLevel ? VK_PIPELINE_STAGE_2_FRAGMENT_SHADER_BIT
                            : VK_PIPELINE_STAGE_2_BLIT_BIT,
                        isLastLevel ? VK_ACCESS_2_SHADER_SAMPLED_READ_BIT
                            : VK_ACCESS_2_TRANSFER_READ_BIT);

                    mipWidth = nextWidth;
                    mipHeight = nextHeight;
                }
            }
            check(vkEndCommandBuffer(commandBuffer), "vkEndCommandBuffer(texture upload)");

            VkFenceCreateInfo fenceInfo { VK_STRUCTURE_TYPE_FENCE_CREATE_INFO };
            check(vkCreateFence(device, &fenceInfo, nullptr, &uploadFence), "vkCreateFence(texture upload)");
            VkCommandBufferSubmitInfo commandSubmit { VK_STRUCTURE_TYPE_COMMAND_BUFFER_SUBMIT_INFO };
            commandSubmit.commandBuffer = commandBuffer;
            VkSubmitInfo2 submit { VK_STRUCTURE_TYPE_SUBMIT_INFO_2 };
            submit.commandBufferInfoCount = 1;
            submit.pCommandBufferInfos = &commandSubmit;
            check(vkQueueSubmit2(graphicsQueue, 1, &submit, uploadFence), "vkQueueSubmit2(texture upload)");
            check(vkWaitForFences(device, 1, &uploadFence, VK_TRUE, UINT64_MAX),
                "vkWaitForFences(texture upload)");

            VkDescriptorSetAllocateInfo materialAllocInfo {
                VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO
            };
            materialAllocInfo.descriptorPool = materialDescriptorPool;
            materialAllocInfo.descriptorSetCount = 1;
            materialAllocInfo.pSetLayouts = &materialDescriptorSetLayout;
            VkDescriptorSet materialSet = VK_NULL_HANDLE;
            check(vkAllocateDescriptorSets(device, &materialAllocInfo, &materialSet),
                "vkAllocateDescriptorSets(material)");
            VkDescriptorImageInfo materialImageInfo {};
            materialImageInfo.sampler = materialSampler;
            materialImageInfo.imageView = view;
            materialImageInfo.imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
            VkWriteDescriptorSet materialWrite { VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET };
            materialWrite.dstSet = materialSet;
            materialWrite.dstBinding = 0;
            materialWrite.descriptorCount = 1;
            materialWrite.descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
            materialWrite.pImageInfo = &materialImageInfo;
            vkUpdateDescriptorSets(device, 1, &materialWrite, 0, nullptr);

            const std::uint32_t index = acquireSlot(textures);
            TextureResource& resource = textures[index];
            resource.image = image;
            resource.allocation = imageAllocation;
            resource.view = view;
            resource.extent = { extent.width, extent.height };
            resource.layout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
            resource.materialDescriptor = materialSet;
            const TextureHandle handle { index, resource.generation };

            image = VK_NULL_HANDLE;
            imageAllocation = VK_NULL_HANDLE;
            view = VK_NULL_HANDLE;
            cleanupTemporary();
            uploadFence = VK_NULL_HANDLE;
            commandPool = VK_NULL_HANDLE;
            stagingBuffer = VK_NULL_HANDLE;

            return handle;
        } catch (...) {
            cleanupTemporary();
            cleanupImage();
            throw;
        }
    }

    void destroyBuffer(BufferHandle handle) {
        BufferResource& resource = checkedResource(buffers, handle, "buffer");
        vkDeviceWaitIdle(device);
        vmaDestroyBuffer(allocator, resource.buffer, resource.allocation);
        resource.buffer = VK_NULL_HANDLE;
        resource.allocation = VK_NULL_HANDLE;
        resource.mapped = nullptr;
        resource.alive = false;
        ++resource.generation;
    }

    void destroyShader(ShaderHandle handle) {
        ShaderResource& resource = checkedResource(shaders, handle, "shader");
        vkDeviceWaitIdle(device);
        vkDestroyShaderModule(device, resource.module, nullptr);
        resource.module = VK_NULL_HANDLE;
        resource.alive = false;
        ++resource.generation;
    }

    void destroyPipeline(PipelineHandle handle) {
        PipelineResource& resource = checkedResource(pipelines, handle, "pipeline");
        vkDeviceWaitIdle(device);
        vkDestroyPipeline(device, resource.pipeline, nullptr);
        vkDestroyPipelineLayout(device, resource.layout, nullptr);
        resource.pipeline = VK_NULL_HANDLE;
        resource.layout = VK_NULL_HANDLE;
        resource.alive = false;
        ++resource.generation;
    }

    void destroyTexture(TextureHandle handle) {
        TextureResource& resource = checkedResource(textures, handle, "texture");
        vkDeviceWaitIdle(device);
        if (resource.imguiDescriptor != VK_NULL_HANDLE && imguiInitialized) {
            ImGui_ImplVulkan_RemoveTexture(resource.imguiDescriptor);
            resource.imguiDescriptor = VK_NULL_HANDLE;
        }
        vkDestroyImageView(device, resource.view, nullptr);
        vmaDestroyImage(allocator, resource.image, resource.allocation);
        resource.image = VK_NULL_HANDLE;
        resource.allocation = VK_NULL_HANDLE;
        resource.view = VK_NULL_HANDLE;
        // materialDescriptor is left allocated in materialDescriptorPool
        // (that pool never frees individual sets, only on shutdown) but no
        // longer reachable through this handle once alive flips false.
        resource.materialDescriptor = VK_NULL_HANDLE;
        resource.alive = false;
        ++resource.generation;
    }

    void writeBuffer(BufferHandle handle, std::size_t offset, std::span<const std::byte> data) {
        BufferResource& resource = checkedResource(buffers, handle, "buffer");
        if (resource.mapped == nullptr) {
            throw std::runtime_error("writeBuffer requires a CPU-visible buffer");
        }
        if (offset + data.size() > resource.size) {
            throw std::runtime_error("writeBuffer exceeds buffer capacity");
        }
        std::memcpy(static_cast<std::byte*>(resource.mapped) + offset, data.data(), data.size());
        check(vmaFlushAllocation(allocator, resource.allocation, offset, data.size()), "vmaFlushAllocation");
    }

    FrameStatus beginFrame() {
        if (frameActive) {
            throw std::runtime_error("RHI frame already active");
        }

        int pixelWidth = 0;
        int pixelHeight = 0;
        SDL_GetWindowSizeInPixels(window, &pixelWidth, &pixelHeight);
        if ((SDL_GetWindowFlags(window) & SDL_WINDOW_MINIMIZED) != 0
            || pixelWidth <= 0 || pixelHeight <= 0) {
            swapchainDirty = true;
            return FrameStatus::Skipped;
        }
        if (swapchainDirty || swapchainExtent.width != static_cast<std::uint32_t>(pixelWidth)
            || swapchainExtent.height != static_cast<std::uint32_t>(pixelHeight)) {
            recreateSwapchain();
        }
        if (swapchain == VK_NULL_HANDLE || swapchainExtent.width == 0
            || swapchainExtent.height == 0) {
            swapchainDirty = true;
            return FrameStatus::Skipped;
        }

        Frame& frame = frames[currentFrame];
        using Clock = std::chrono::steady_clock;
        const auto fenceWaitStart = Clock::now();
        check(vkWaitForFences(device, 1, &frame.fence, VK_TRUE, UINT64_MAX), "vkWaitForFences");
        const float fenceWaitMilliseconds =
            std::chrono::duration<float, std::milli>(
                Clock::now() - fenceWaitStart).count();

        // O fence confirma que as metricas e os timestamps guardados neste
        // slot pertencem a um frame integralmente concluido. Publicamos esse
        // snapshot e so entao reutilizamos o slot para o frame atual; assim a
        // UI nunca observa um present ainda zerado ou medidas pela metade.
        frameMetrics = frame.performance;
        if (frame.timestampWritten) {
            std::array<std::uint64_t, 2> timestamps {};
            const std::uint32_t firstQuery = currentFrame * 2;
            const VkResult timingResult = vkGetQueryPoolResults(device,
                frameTimestampQueryPool, firstQuery, 2,
                sizeof(timestamps), timestamps.data(),
                sizeof(std::uint64_t), VK_QUERY_RESULT_64_BIT);
            if (timingResult == VK_SUCCESS
                && timestamps[1] >= timestamps[0]) {
                frameMetrics.gpuFrameMilliseconds =
                    static_cast<float>(timestamps[1] - timestamps[0])
                    * timestampPeriodNanoseconds / 1'000'000.0f;
                frameMetrics.gpuTimingValid = true;
            }
        }
        frame.performance = {};
        frame.performance.cpuFenceWaitMilliseconds = fenceWaitMilliseconds;

        const auto acquireStart = Clock::now();
        const VkResult acquired = vkAcquireNextImageKHR(device, swapchain, UINT64_MAX,
            frame.imageAvailable, VK_NULL_HANDLE, &currentImage);
        if (acquired == VK_ERROR_OUT_OF_DATE_KHR) {
            swapchainDirty = true;
            recreateSwapchain();
            return FrameStatus::Skipped;
        }
        if (acquired != VK_SUCCESS && acquired != VK_SUBOPTIMAL_KHR) {
            check(acquired, "vkAcquireNextImageKHR");
        }
        if (acquired == VK_SUBOPTIMAL_KHR) {
            swapchainDirty = true;
        }

        if (imageFences[currentImage] != VK_NULL_HANDLE) {
            check(vkWaitForFences(device, 1, &imageFences[currentImage], VK_TRUE, UINT64_MAX),
                "vkWaitForFences(image)");
        }
        frame.performance.cpuAcquireMilliseconds =
            std::chrono::duration<float, std::milli>(
                Clock::now() - acquireStart).count();
        imageFences[currentImage] = frame.fence;

        check(vkResetFences(device, 1, &frame.fence), "vkResetFences");
        check(vkResetCommandPool(device, frame.commandPool, 0), "vkResetCommandPool");

        VkCommandBufferBeginInfo beginInfo { VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO };
        beginInfo.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
        check(vkBeginCommandBuffer(frame.commandBuffer, &beginInfo), "vkBeginCommandBuffer");
        const std::uint32_t firstQuery = currentFrame * 2;
        vkCmdResetQueryPool(frame.commandBuffer, frameTimestampQueryPool,
            firstQuery, 2);
        vkCmdWriteTimestamp2(frame.commandBuffer,
            VK_PIPELINE_STAGE_2_TOP_OF_PIPE_BIT,
            frameTimestampQueryPool, firstQuery);

        boundPipeline = {};
        worldSpriteDrawCount = 0;
        frameActive = true;
        swapchainPassActive = false;
        return FrameStatus::Ready;
    }

    void beginRenderTargetPass(TextureHandle target, ClearColor clearColor) {
        ensureFrame();
        TextureResource& resource = checkedResource(textures, target, "render target");
        Frame& frame = frames[currentFrame];

        VkImageMemoryBarrier2 toAttachment { VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER_2 };
        toAttachment.srcStageMask = resource.layout == VK_IMAGE_LAYOUT_UNDEFINED
            ? VK_PIPELINE_STAGE_2_NONE : VK_PIPELINE_STAGE_2_FRAGMENT_SHADER_BIT;
        toAttachment.srcAccessMask = resource.layout == VK_IMAGE_LAYOUT_UNDEFINED
            ? VK_ACCESS_2_NONE : VK_ACCESS_2_SHADER_READ_BIT;
        toAttachment.dstStageMask = VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT;
        toAttachment.dstAccessMask = VK_ACCESS_2_COLOR_ATTACHMENT_WRITE_BIT;
        toAttachment.oldLayout = resource.layout;
        toAttachment.newLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
        toAttachment.image = resource.image;
        toAttachment.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
        toAttachment.subresourceRange.levelCount = 1;
        toAttachment.subresourceRange.layerCount = 1;

        VkDependencyInfo dependency { VK_STRUCTURE_TYPE_DEPENDENCY_INFO };
        dependency.imageMemoryBarrierCount = 1;
        dependency.pImageMemoryBarriers = &toAttachment;
        vkCmdPipelineBarrier2(frame.commandBuffer, &dependency);
        resource.layout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;

        VkClearValue clear {};
        clear.color.float32[0] = clearColor.r;
        clear.color.float32[1] = clearColor.g;
        clear.color.float32[2] = clearColor.b;
        clear.color.float32[3] = clearColor.a;

        VkRenderingAttachmentInfo colorAttachment { VK_STRUCTURE_TYPE_RENDERING_ATTACHMENT_INFO };
        colorAttachment.imageView = resource.view;
        colorAttachment.imageLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
        colorAttachment.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
        colorAttachment.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
        colorAttachment.clearValue = clear;

        VkRenderingInfo rendering { VK_STRUCTURE_TYPE_RENDERING_INFO };
        rendering.renderArea.extent = resource.extent;
        rendering.layerCount = 1;
        rendering.colorAttachmentCount = 1;
        rendering.pColorAttachments = &colorAttachment;
        vkCmdBeginRendering(frame.commandBuffer, &rendering);

        boundPipeline = {};
        activeRenderTarget = target;
        renderTargetPassActive = true;
    }

    void endRenderTargetPass() {
        if (!renderTargetPassActive) {
            return;
        }
        Frame& frame = frames[currentFrame];
        vkCmdEndRendering(frame.commandBuffer);

        TextureResource& resource = checkedResource(textures, activeRenderTarget, "render target");
        VkImageMemoryBarrier2 toShaderRead { VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER_2 };
        toShaderRead.srcStageMask = VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT;
        toShaderRead.srcAccessMask = VK_ACCESS_2_COLOR_ATTACHMENT_WRITE_BIT;
        toShaderRead.dstStageMask = VK_PIPELINE_STAGE_2_FRAGMENT_SHADER_BIT;
        toShaderRead.dstAccessMask = VK_ACCESS_2_SHADER_READ_BIT;
        toShaderRead.oldLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
        toShaderRead.newLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
        toShaderRead.image = resource.image;
        toShaderRead.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
        toShaderRead.subresourceRange.levelCount = 1;
        toShaderRead.subresourceRange.layerCount = 1;

        VkDependencyInfo dependency { VK_STRUCTURE_TYPE_DEPENDENCY_INFO };
        dependency.imageMemoryBarrierCount = 1;
        dependency.pImageMemoryBarriers = &toShaderRead;
        vkCmdPipelineBarrier2(frame.commandBuffer, &dependency);
        resource.layout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;

        renderTargetPassActive = false;
        activeRenderTarget = {};
    }

    // Shared by blitToSwapchain (upscaling the low-res world target onto the
    // swapchain) and drawWorldSprite (drawing a pre-baked static texture,
    // like the grass field, into the world render target) - both are "draw
    // this whole texture stretched across 4 screen-space corners," just
    // with different corners/extent/descriptor slot. descriptorSlot must be
    // stable per call-site (never shared between two call-sites that use
    // different textures within the same frame) since updating a
    // descriptor set's binding is only safe between GPU uses of that set,
    // not between two record-time updates in the same not-yet-submitted
    // command buffer.
    void emitSpriteDraw(TextureHandle source, const std::array<float, 8>& corners,
        const std::array<float, 4>& perspectiveDepths, VkExtent2D viewportExtent,
        std::size_t descriptorSlot) {
        TextureResource& resource = checkedResource(textures, source, "sprite source");
        Frame& frame = frames[currentFrame];
        VkDescriptorSet descriptorSet = spriteDescriptorSets[descriptorSlot];

        VkDescriptorImageInfo imageInfo {};
        imageInfo.sampler = nearestSampler;
        imageInfo.imageView = resource.view;
        imageInfo.imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;

        VkWriteDescriptorSet write { VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET };
        write.dstSet = descriptorSet;
        write.dstBinding = 0;
        write.descriptorCount = 1;
        write.descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
        write.pImageInfo = &imageInfo;
        vkUpdateDescriptorSets(device, 1, &write, 0, nullptr);

        VkViewport viewport {};
        viewport.width = static_cast<float>(viewportExtent.width);
        viewport.height = static_cast<float>(viewportExtent.height);
        viewport.minDepth = 0.0f;
        viewport.maxDepth = 1.0f;
        vkCmdSetViewport(frame.commandBuffer, 0, 1, &viewport);
        VkRect2D scissor {};
        scissor.extent = viewportExtent;
        vkCmdSetScissor(frame.commandBuffer, 0, 1, &scissor);

        std::array<float, 14> pushData {};
        std::copy(corners.begin(), corners.end(), pushData.begin());
        std::copy(perspectiveDepths.begin(), perspectiveDepths.end(), pushData.begin() + 8);
        pushData[12] = static_cast<float>(viewportExtent.width);
        pushData[13] = static_cast<float>(viewportExtent.height);

        vkCmdBindPipeline(frame.commandBuffer, VK_PIPELINE_BIND_POINT_GRAPHICS, spritePipeline);
        vkCmdPushConstants(frame.commandBuffer, spritePipelineLayout, VK_SHADER_STAGE_VERTEX_BIT,
            0, static_cast<std::uint32_t>(pushData.size() * sizeof(float)), pushData.data());
        vkCmdBindDescriptorSets(frame.commandBuffer, VK_PIPELINE_BIND_POINT_GRAPHICS, spritePipelineLayout,
            0, 1, &descriptorSet, 0, nullptr);
        vkCmdDraw(frame.commandBuffer, 6, 1, 0, 0);

        boundPipeline = {};
    }

    void blitToSwapchain(TextureHandle source, ClearColor clearColor) {
        ensureFrame();
        Frame& frame = frames[currentFrame];

        VkImageMemoryBarrier2 toAttachment { VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER_2 };
        toAttachment.srcStageMask = swapchainInitialized[currentImage]
            ? VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT : VK_PIPELINE_STAGE_2_NONE;
        toAttachment.srcAccessMask = VK_ACCESS_2_NONE;
        toAttachment.dstStageMask = VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT;
        toAttachment.dstAccessMask = VK_ACCESS_2_COLOR_ATTACHMENT_WRITE_BIT;
        toAttachment.oldLayout = swapchainInitialized[currentImage]
            ? VK_IMAGE_LAYOUT_PRESENT_SRC_KHR : VK_IMAGE_LAYOUT_UNDEFINED;
        toAttachment.newLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
        toAttachment.image = swapchainImages[currentImage];
        toAttachment.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
        toAttachment.subresourceRange.levelCount = 1;
        toAttachment.subresourceRange.layerCount = 1;

        VkDependencyInfo dependency { VK_STRUCTURE_TYPE_DEPENDENCY_INFO };
        dependency.imageMemoryBarrierCount = 1;
        dependency.pImageMemoryBarriers = &toAttachment;
        vkCmdPipelineBarrier2(frame.commandBuffer, &dependency);

        VkClearValue clear {};
        clear.color.float32[0] = clearColor.r;
        clear.color.float32[1] = clearColor.g;
        clear.color.float32[2] = clearColor.b;
        clear.color.float32[3] = clearColor.a;

        VkRenderingAttachmentInfo colorAttachment { VK_STRUCTURE_TYPE_RENDERING_ATTACHMENT_INFO };
        colorAttachment.imageView = swapchainImageViews[currentImage];
        colorAttachment.imageLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
        colorAttachment.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
        colorAttachment.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
        colorAttachment.clearValue = clear;

        VkRenderingInfo rendering { VK_STRUCTURE_TYPE_RENDERING_INFO };
        rendering.renderArea.extent = swapchainExtent;
        rendering.layerCount = 1;
        rendering.colorAttachmentCount = 1;
        rendering.pColorAttachments = &colorAttachment;
        vkCmdBeginRendering(frame.commandBuffer, &rendering);
        swapchainPassActive = true;

        const std::array<float, 8> corners {
            0.0f, 0.0f,
            static_cast<float>(swapchainExtent.width), 0.0f,
            static_cast<float>(swapchainExtent.width), static_cast<float>(swapchainExtent.height),
            0.0f, static_cast<float>(swapchainExtent.height)
        };
        constexpr std::array<float, 4> FlatDepths { 1.0f, 1.0f, 1.0f, 1.0f };
        emitSpriteDraw(source, corners, FlatDepths, swapchainExtent, 0);
    }

    void drawWorldSprite(TextureHandle source, const std::array<float, 8>& corners,
        const std::array<float, 4>& perspectiveDepths) {
        ensureFrame();
        if (!renderTargetPassActive) {
            throw std::runtime_error("drawWorldSprite requires an active render target pass");
        }
        TextureResource& target = checkedResource(textures, activeRenderTarget, "active render target");
        const std::size_t descriptorSlot = 1 + worldSpriteDrawCount;
        if (descriptorSlot >= spriteDescriptorSets.size()) {
            throw std::runtime_error("Too many world sprites in one frame");
        }
        ++worldSpriteDrawCount;
        emitSpriteDraw(source, corners, perspectiveDepths, target.extent, descriptorSlot);
    }

    std::uint64_t renderScene3DInternal(const Scene3DFrame& scene, Extent2D extent,
        bool directToSwapchain) {
        ensureFrame();
        if (renderTargetPassActive || swapchainPassActive) {
            throw std::runtime_error("renderScene3D requires no active rendering pass");
        }
        if (extent.width == 0 || extent.height == 0) {
            throw std::runtime_error("renderScene3D requires a non-zero viewport");
        }
        if (directToSwapchain) {
            ensureSceneShadowTarget();
            ensureSceneDirectDepth(extent);
        } else {
            ensureScene3DTarget(extent);
        }
        Frame& frame = frames[currentFrame];
        bool& temporalHistoryValid = directToSwapchain
            ? sceneDirectTaaHistoryValid : sceneTaaHistoryValid;
        const bool useTemporalHistory =
            scene.temporalAntiAliasingEnabled
            && temporalHistoryValid
            && !scene.resetTemporalHistory;

        SceneUniformGpu uniform;
        // A GPU sempre recebe a variante jitterada (ver comentario em
        // SceneUniformGpu::cameraViewProjection) - identica a
        // scene.cameraViewProjection quando a cena nao usa TAA de verdade.
        uniform.cameraViewProjection = scene.cameraViewProjectionJittered.values;
        uniform.cameraViewProjectionUnjittered =
            scene.cameraViewProjection.values;
        // O ceu nao participa da acumulacao temporal; reconstruir seu raio
        // com a inversa jitterada deslocaria horizonte/nuvens a cada amostra
        // Halton. A geometria continua usando a VP jitterada acima.
        uniform.inverseCameraViewProjection =
            scene.cameraViewProjection.inverse().values;
        for (std::uint32_t cascade = 0; cascade < ShadowCascadeCount; ++cascade) {
            uniform.cascadeViewProjections[cascade] =
                scene.cascadeViewProjections[cascade].values;
        }
        uniform.cascadeSplits = scene.cascadeSplits;
        uniform.cascadeTexelWorldSizes = scene.cascadeTexelWorldSizes;
        uniform.cascadeDepthRanges = scene.cascadeDepthRanges;
        uniform.previousCameraViewProjection =
            scene.previousCameraViewProjection.values;
        uniform.cameraPosition = {
            scene.cameraPosition.x, scene.cameraPosition.y, scene.cameraPosition.z, 1.0f
        };
        uniform.settings = {
            scene.showShadows ? 1.0f : 0.0f,
            static_cast<float>(scene.lights.size()),
            useTemporalHistory ? 1.0f : 0.0f,
            scene.ambientLight
        };
        uniform.skySettings = {
            scene.showSky ? 1.0f : 0.0f,
            scene.skyTime,
            scene.cloudCoverage,
            0.0f
        };
        uniform.fogSettings = {
            scene.fog.density, scene.fog.heightFalloff, scene.fog.maxOpacity, 0.0f
        };
        uniform.fogColor = {
            scene.fog.color.x, scene.fog.color.y, scene.fog.color.z, 0.0f
        };
        uniform.windOffset = {
            scene.cloudWindOffset.x, scene.cloudWindOffset.y, 0.0f, 0.0f
        };
        std::memcpy(sceneUniformMapped[currentFrame], &uniform, sizeof(uniform));
        check(vmaFlushAllocation(allocator, sceneUniformAllocations[currentFrame],
            0, sizeof(uniform)), "vmaFlushAllocation(scene uniform)");

        // O SSBO de luzes precisa de um buffer valido no descriptor set 0
        // (binding 2) antes de qualquer bind de pipeline que o referencie -
        // diferente do buffer de instancia de mesh (um vertex buffer, so
        // relevante quando ha meshes), este e um binding de descriptor, e
        // Vulkan exige que ele aponte pra algo valido mesmo quando a cena nao
        // tem luz nenhuma. Ver ensureSceneLightCapacity.
        ensureSceneLightCapacity(scene.lights.size());
        const std::vector<GpuLightData3D> packedLights = packSceneLights(scene.lights);
        if (!packedLights.empty()) {
            const VkDeviceSize lightByteCount =
                packedLights.size() * sizeof(GpuLightData3D);
            std::memcpy(sceneLightMapped[currentFrame], packedLights.data(),
                static_cast<std::size_t>(lightByteCount));
            check(vmaFlushAllocation(allocator, sceneLightAllocations[currentFrame],
                0, lightByteCount), "vmaFlushAllocation(scene lights)");
        }

        const auto setViewportAndScissor = [&](VkExtent2D targetExtent) {
            VkViewport viewport {};
            viewport.width = static_cast<float>(targetExtent.width);
            viewport.height = static_cast<float>(targetExtent.height);
            viewport.minDepth = 0.0f;
            viewport.maxDepth = 1.0f;
            vkCmdSetViewport(frame.commandBuffer, 0, 1, &viewport);
            VkRect2D scissor {};
            scissor.extent = targetExtent;
            vkCmdSetScissor(frame.commandBuffer, 0, 1, &scissor);
        };
        struct MeshBatch {
            const MeshRender3D* prototype = nullptr;
            std::uint32_t firstInstance = 0;
            std::uint32_t instanceCount = 0;
        };
        const auto meshKey = [](const MeshRender3D* mesh) {
            return std::tuple {
                mesh->vertexBuffer.index, mesh->vertexBuffer.generation,
                mesh->indexBuffer.index, mesh->indexBuffer.generation,
                mesh->indexCount,
                mesh->albedoTexture.index, mesh->albedoTexture.generation,
                mesh->metallicRoughnessTexture.index,
                mesh->metallicRoughnessTexture.generation,
                mesh->castsShadow, mesh->visibleInCamera,
                mesh->shadowCascadeMask
            };
        };
        std::vector<const MeshRender3D*> orderedMeshes;
        orderedMeshes.reserve(scene.meshes.size());
        for (const MeshRender3D& mesh : scene.meshes) {
            orderedMeshes.push_back(&mesh);
        }
        std::sort(orderedMeshes.begin(), orderedMeshes.end(),
            [&](const MeshRender3D* left, const MeshRender3D* right) {
                return meshKey(left) < meshKey(right);
            });
        std::vector<SceneMeshInstanceGpu> meshInstances;
        std::vector<MeshBatch> meshBatches;
        meshInstances.reserve(orderedMeshes.size());
        meshBatches.reserve(orderedMeshes.size());
        for (const MeshRender3D* mesh : orderedMeshes) {
            if (meshBatches.empty()
                || meshKey(meshBatches.back().prototype) != meshKey(mesh)) {
                meshBatches.push_back({ mesh,
                    static_cast<std::uint32_t>(meshInstances.size()), 0 });
            }
            ++meshBatches.back().instanceCount;
            const Vec3 orientationX = mesh->orientation.rotate(
                { 1.0f, 0.0f, 0.0f });
            const Vec3 orientationY = mesh->orientation.rotate(
                { 0.0f, 1.0f, 0.0f });
            const Vec3 orientationZ = mesh->orientation.rotate(
                { 0.0f, 0.0f, 1.0f });
            const Vec3 previousOrientationX = mesh->previousOrientation.rotate(
                { 1.0f, 0.0f, 0.0f });
            const Vec3 previousOrientationY = mesh->previousOrientation.rotate(
                { 0.0f, 1.0f, 0.0f });
            const Vec3 previousOrientationZ = mesh->previousOrientation.rotate(
                { 0.0f, 0.0f, 1.0f });
            SceneMeshInstanceGpu gpuInstance;
            gpuInstance.positionScale = { mesh->position.x, mesh->position.y,
                mesh->position.z, mesh->scale };
            gpuInstance.orientationX = { orientationX.x, orientationX.y,
                orientationX.z, 0.0f };
            gpuInstance.orientationY = { orientationY.x, orientationY.y,
                orientationY.z, 0.0f };
            gpuInstance.orientationZ = { orientationZ.x, orientationZ.y,
                orientationZ.z, 0.0f };
            gpuInstance.materialAndFlags = { mesh->metallic, mesh->roughness,
                mesh->selected ? 1.0f : 0.0f,
                mesh->outlineGlow ? 1.0f : 0.0f };
            // Mesma escala para o passado - nada nesta engine anima escala
            // ao longo do tempo hoje, entao rastrear uma escala anterior
            // separada seria estado sem nenhum consumidor real.
            gpuInstance.previousPositionScale = { mesh->previousPosition.x,
                mesh->previousPosition.y, mesh->previousPosition.z,
                mesh->scale };
            gpuInstance.previousOrientationX = { previousOrientationX.x,
                previousOrientationX.y, previousOrientationX.z, 0.0f };
            gpuInstance.previousOrientationY = { previousOrientationY.x,
                previousOrientationY.y, previousOrientationY.z, 0.0f };
            gpuInstance.previousOrientationZ = { previousOrientationZ.x,
                previousOrientationZ.y, previousOrientationZ.z, 0.0f };
            meshInstances.push_back(gpuInstance);
        }
        if (!meshInstances.empty()) {
            ensureSceneMeshInstanceCapacity(meshInstances.size());
            const VkDeviceSize byteCount = meshInstances.size()
                * sizeof(SceneMeshInstanceGpu);
            std::memcpy(sceneMeshInstanceMapped[currentFrame],
                meshInstances.data(), static_cast<std::size_t>(byteCount));
            check(vmaFlushAllocation(allocator,
                sceneMeshInstanceAllocations[currentFrame], 0, byteCount),
                "vmaFlushAllocation(scene mesh instances)");
        }

        const auto pushMeshBatch = [&](const MeshBatch& batch,
            bool bindMaterial) {
            const MeshRender3D& meshObject = *batch.prototype;
            BufferResource& vertexBuffer =
                checkedResource(buffers, meshObject.vertexBuffer, "mesh vertex buffer");
            BufferResource& indexBuffer =
                checkedResource(buffers, meshObject.indexBuffer, "mesh index buffer");
            if (bindMaterial) {
                const TextureResource& materialTexture = meshObject.albedoTexture.valid()
                    ? checkedResource(textures, meshObject.albedoTexture, "mesh albedo texture")
                    : checkedResource(textures, defaultMaterialTexture, "default material texture");
                vkCmdBindDescriptorSets(frame.commandBuffer, VK_PIPELINE_BIND_POINT_GRAPHICS,
                    sceneMeshPipelineLayout, 1, 1, &materialTexture.materialDescriptor, 0, nullptr);
                const TextureResource& metallicRoughnessTexture =
                    meshObject.metallicRoughnessTexture.valid()
                    ? checkedResource(textures,
                        meshObject.metallicRoughnessTexture,
                        "mesh metallic roughness texture")
                    : checkedResource(textures, defaultMaterialTexture,
                        "default metallic roughness texture");
                vkCmdBindDescriptorSets(frame.commandBuffer,
                    VK_PIPELINE_BIND_POINT_GRAPHICS,
                    sceneMeshPipelineLayout, 2, 1,
                    &metallicRoughnessTexture.materialDescriptor,
                    0, nullptr);
            }
            const std::array<VkBuffer, 2> vertexBuffers {
                vertexBuffer.buffer,
                sceneMeshInstanceBuffers[currentFrame]
            };
            const std::array<VkDeviceSize, 2> vertexOffsets {
                0,
                static_cast<VkDeviceSize>(batch.firstInstance)
                    * sizeof(SceneMeshInstanceGpu)
            };
            vkCmdBindVertexBuffers(frame.commandBuffer, 0,
                static_cast<std::uint32_t>(vertexBuffers.size()),
                vertexBuffers.data(), vertexOffsets.data());
            vkCmdBindIndexBuffer(frame.commandBuffer, indexBuffer.buffer, 0, VK_INDEX_TYPE_UINT32);
            vkCmdDrawIndexed(frame.commandBuffer, meshObject.indexCount,
                batch.instanceCount, 0, 0, 0);
        };

        // O primeiro uso inicializa o recurso para manter o descriptor valido.
        // Depois disso, previews/cenas sem sombras pulam integralmente o passe
        // de 2048x2048, incluindo clear, barreiras e draws.
        const bool firstShadowUse =
            sceneShadowStates[0].layout == VK_IMAGE_LAYOUT_UNDEFINED;
        if (scene.showShadows || firstShadowUse) {
        // Uma passada por cascata (ver ShadowCascadeCount) - mesmo bloco de
        // antes, agora dentro de um laco, escrevendo num mapa e usando uma
        // matriz view-projection diferente por iteracao (empurrada via push
        // constant pro vertex shader de sombra escolher
        // scene.cascadeViewProjections[cascadeIndex]).
        for (std::uint32_t cascade = 0; cascade < ShadowCascadeCount; ++cascade) {
            transitionSceneAttachment(frame.commandBuffer, sceneShadowImages[cascade],
                VK_IMAGE_ASPECT_DEPTH_BIT, sceneShadowStates[cascade],
                VK_IMAGE_LAYOUT_DEPTH_ATTACHMENT_OPTIMAL,
                VK_PIPELINE_STAGE_2_EARLY_FRAGMENT_TESTS_BIT
                    | VK_PIPELINE_STAGE_2_LATE_FRAGMENT_TESTS_BIT,
                VK_ACCESS_2_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT);

            VkClearValue shadowClear {};
            shadowClear.depthStencil.depth = 1.0f;
            VkRenderingAttachmentInfo shadowDepthAttachment { VK_STRUCTURE_TYPE_RENDERING_ATTACHMENT_INFO };
            shadowDepthAttachment.imageView = sceneShadowViews[cascade];
            shadowDepthAttachment.imageLayout = VK_IMAGE_LAYOUT_DEPTH_ATTACHMENT_OPTIMAL;
            shadowDepthAttachment.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
            shadowDepthAttachment.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
            shadowDepthAttachment.clearValue = shadowClear;
            VkRenderingInfo shadowRendering { VK_STRUCTURE_TYPE_RENDERING_INFO };
            shadowRendering.renderArea.extent = { SceneShadowMapSize, SceneShadowMapSize };
            shadowRendering.layerCount = 1;
            shadowRendering.pDepthAttachment = &shadowDepthAttachment;
            vkCmdBeginRendering(frame.commandBuffer, &shadowRendering);
            setViewportAndScissor({ SceneShadowMapSize, SceneShadowMapSize });
            if (!scene.meshes.empty()) {
                vkCmdBindPipeline(frame.commandBuffer, VK_PIPELINE_BIND_POINT_GRAPHICS,
                    sceneMeshShadowPipeline);
                vkCmdBindDescriptorSets(frame.commandBuffer, VK_PIPELINE_BIND_POINT_GRAPHICS,
                    scenePipelineLayout, 0, 1, &sceneDescriptorSets[currentFrame], 0, nullptr);
                vkCmdPushConstants(frame.commandBuffer, scenePipelineLayout,
                    VK_SHADER_STAGE_VERTEX_BIT, 0, sizeof(std::uint32_t), &cascade);
                for (const MeshBatch& batch : meshBatches) {
                    if (batch.prototype->castsShadow
                        && (batch.prototype->shadowCascadeMask
                            & (1u << cascade)) != 0u) {
                        pushMeshBatch(batch, false);
                    }
                }
            }
            vkCmdEndRendering(frame.commandBuffer);

            transitionSceneAttachment(frame.commandBuffer, sceneShadowImages[cascade],
                VK_IMAGE_ASPECT_DEPTH_BIT, sceneShadowStates[cascade],
                VK_IMAGE_LAYOUT_DEPTH_STENCIL_READ_ONLY_OPTIMAL,
                VK_PIPELINE_STAGE_2_FRAGMENT_SHADER_BIT,
                VK_ACCESS_2_SHADER_SAMPLED_READ_BIT);
        }
        }

        // Passe opaco (ceu+mesh): escreve no alvo HDR, nao mais direto no
        // destino final - ver o passe de tonemap logo abaixo, que resolve
        // HDR->LDR depois de encerrar este.
        VkImage hdrColorImage = directToSwapchain ? sceneDirectHdrColorImage : sceneHdrColorImage;
        VkImageView hdrColorView = directToSwapchain ? sceneDirectHdrColorView : sceneHdrColorView;
        SceneAttachmentState3D& hdrColorState = directToSwapchain
            ? sceneDirectHdrColorState : sceneHdrColorState;
        transitionSceneAttachment(frame.commandBuffer, hdrColorImage,
            VK_IMAGE_ASPECT_COLOR_BIT, hdrColorState,
            VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL,
            VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT,
            VK_ACCESS_2_COLOR_ATTACHMENT_WRITE_BIT);

        // Segundo attachment de cor do MESMO passe opaco (MRT) - ver
        // scene3d_mesh.vert/frag e o comentario em sceneMotionVectorImage.
        VkImage motionVectorImage = directToSwapchain
            ? sceneDirectMotionVectorImage : sceneMotionVectorImage;
        VkImageView motionVectorView = directToSwapchain
            ? sceneDirectMotionVectorView : sceneMotionVectorView;
        SceneAttachmentState3D& motionVectorState = directToSwapchain
            ? sceneDirectMotionVectorState : sceneMotionVectorState;
        transitionSceneAttachment(frame.commandBuffer, motionVectorImage,
            VK_IMAGE_ASPECT_COLOR_BIT, motionVectorState,
            VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL,
            VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT,
            VK_ACCESS_2_COLOR_ATTACHMENT_WRITE_BIT);

        VkImage depthImage = directToSwapchain ? sceneDirectDepthImage : sceneDepthImage;
        VkImageView depthView = directToSwapchain ? sceneDirectDepthView : sceneDepthView;
        SceneAttachmentState3D& depthState = directToSwapchain
            ? sceneDirectDepthState : sceneDepthState;
        const VkExtent2D renderExtent { extent.width, extent.height };
        transitionSceneAttachment(frame.commandBuffer, depthImage,
            VK_IMAGE_ASPECT_DEPTH_BIT, depthState,
            VK_IMAGE_LAYOUT_DEPTH_ATTACHMENT_OPTIMAL,
            VK_PIPELINE_STAGE_2_EARLY_FRAGMENT_TESTS_BIT
                | VK_PIPELINE_STAGE_2_LATE_FRAGMENT_TESTS_BIT,
            VK_ACCESS_2_DEPTH_STENCIL_ATTACHMENT_READ_BIT
                | VK_ACCESS_2_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT);

        // Pre-pass de profundidade: resolve o buffer de profundidade inteiro
        // ANTES do passe de cor opaco, para que este possa so testar (EQUAL)
        // sem escrever de novo (ver sceneMeshPipeline em
        // createScene3DResources) - fragmentos ja sabidamente ocluidos nunca
        // chegam a rodar o fragment shader completo (BRDF + shadow lookup)
        // do passe de cor.
        // 0.0f, nao 1.0f: a camera principal usa profundidade INVERTIDA
        // (Mat4::perspective produz perto=1, longe=0) - o valor de clear
        // precisa representar "o mais distante possivel", que agora e 0.
        VkClearValue depthPrepassClear {};
        depthPrepassClear.depthStencil.depth = 0.0f;
        VkRenderingAttachmentInfo depthPrepassAttachment {
            VK_STRUCTURE_TYPE_RENDERING_ATTACHMENT_INFO
        };
        depthPrepassAttachment.imageView = depthView;
        depthPrepassAttachment.imageLayout = VK_IMAGE_LAYOUT_DEPTH_ATTACHMENT_OPTIMAL;
        depthPrepassAttachment.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
        depthPrepassAttachment.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
        depthPrepassAttachment.clearValue = depthPrepassClear;
        VkRenderingInfo depthPrepassRendering { VK_STRUCTURE_TYPE_RENDERING_INFO };
        depthPrepassRendering.renderArea.extent = renderExtent;
        depthPrepassRendering.layerCount = 1;
        depthPrepassRendering.pDepthAttachment = &depthPrepassAttachment;
        vkCmdBeginRendering(frame.commandBuffer, &depthPrepassRendering);
        setViewportAndScissor(renderExtent);
        if (!scene.meshes.empty()) {
            vkCmdBindPipeline(frame.commandBuffer, VK_PIPELINE_BIND_POINT_GRAPHICS,
                sceneMeshDepthPrepassPipeline);
            vkCmdBindDescriptorSets(frame.commandBuffer, VK_PIPELINE_BIND_POINT_GRAPHICS,
                scenePipelineLayout, 0, 1, &sceneDescriptorSets[currentFrame], 0, nullptr);
            for (const MeshBatch& batch : meshBatches) {
                if (batch.prototype->visibleInCamera) {
                    pushMeshBatch(batch, false);
                }
            }
        }
        vkCmdEndRendering(frame.commandBuffer);

        // Barreira entre os dois passes: mesmo layout dos dois lados
        // (DEPTH_ATTACHMENT_OPTIMAL), mas vkCmdBeginRendering nao insere
        // dependencia automatica entre instancias de rendering separadas -
        // sem isso, o passe de cor poderia testar profundidade antes do
        // pre-pass realmente terminar de escrever.
        transitionSceneAttachment(frame.commandBuffer, depthImage,
            VK_IMAGE_ASPECT_DEPTH_BIT, depthState,
            VK_IMAGE_LAYOUT_DEPTH_ATTACHMENT_OPTIMAL,
            VK_PIPELINE_STAGE_2_EARLY_FRAGMENT_TESTS_BIT
                | VK_PIPELINE_STAGE_2_LATE_FRAGMENT_TESTS_BIT,
            VK_ACCESS_2_DEPTH_STENCIL_ATTACHMENT_READ_BIT
                | VK_ACCESS_2_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT);

        VkClearValue colorClear {};
        colorClear.color.float32[0] = 0.025f;
        colorClear.color.float32[1] = 0.040f;
        colorClear.color.float32[2] = 0.055f;
        colorClear.color.float32[3] = 1.0f;
        VkRenderingAttachmentInfo colorAttachment { VK_STRUCTURE_TYPE_RENDERING_ATTACHMENT_INFO };
        colorAttachment.imageView = hdrColorView;
        colorAttachment.imageLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
        colorAttachment.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
        colorAttachment.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
        colorAttachment.clearValue = colorClear;
        // Vetores de movimento (ver scene3d_mesh.vert/frag) - limpo pra
        // zero: pixels nunca cobertos por nenhuma geometria neste quadro
        // (nao deveria acontecer com o ceu ligado, mas por seguranca) ficam
        // sem movimento algum em vez de lixo nao inicializado.
        VkClearValue motionVectorClear {};
        VkRenderingAttachmentInfo motionVectorAttachment {
            VK_STRUCTURE_TYPE_RENDERING_ATTACHMENT_INFO
        };
        motionVectorAttachment.imageView = motionVectorView;
        motionVectorAttachment.imageLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
        motionVectorAttachment.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
        motionVectorAttachment.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
        motionVectorAttachment.clearValue = motionVectorClear;
        VkRenderingAttachmentInfo depthAttachment { VK_STRUCTURE_TYPE_RENDERING_ATTACHMENT_INFO };
        depthAttachment.imageView = depthView;
        depthAttachment.imageLayout = VK_IMAGE_LAYOUT_DEPTH_ATTACHMENT_OPTIMAL;
        // LOAD (nao CLEAR): a profundidade ja foi resolvida pelo pre-pass
        // acima - limpar de novo aqui apagaria o resultado que o teste
        // EQUAL do passe de cor depende.
        depthAttachment.loadOp = VK_ATTACHMENT_LOAD_OP_LOAD;
        depthAttachment.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
        const std::array<VkRenderingAttachmentInfo, 2> opaqueColorAttachments {
            colorAttachment, motionVectorAttachment
        };
        VkRenderingInfo rendering { VK_STRUCTURE_TYPE_RENDERING_INFO };
        rendering.renderArea.extent = renderExtent;
        rendering.layerCount = 1;
        rendering.colorAttachmentCount =
            static_cast<std::uint32_t>(opaqueColorAttachments.size());
        rendering.pColorAttachments = opaqueColorAttachments.data();
        rendering.pDepthAttachment = &depthAttachment;
        vkCmdBeginRendering(frame.commandBuffer, &rendering);
        setViewportAndScissor(renderExtent);
        if (scene.showSky) {
            vkCmdBindPipeline(frame.commandBuffer, VK_PIPELINE_BIND_POINT_GRAPHICS,
                sceneSkyPipeline);
            vkCmdBindDescriptorSets(frame.commandBuffer, VK_PIPELINE_BIND_POINT_GRAPHICS,
                scenePipelineLayout, 0, 1, &sceneDescriptorSets[currentFrame], 0, nullptr);
            vkCmdDraw(frame.commandBuffer, 3, 1, 0, 0);
        }
        if (!scene.meshes.empty()) {
            vkCmdBindPipeline(frame.commandBuffer, VK_PIPELINE_BIND_POINT_GRAPHICS,
                sceneMeshPipeline);
            // Antes, este bind vinha "de carona" do desenho do chao/caixas/
            // esferas analiticos que precedia este bloco (ambos os layouts
            // compartilham o set 0). Removido esse desenho, o bind precisa
            // ser explicito aqui para nao depender de estado deixado por um
            // pipeline diferente.
            vkCmdBindDescriptorSets(frame.commandBuffer, VK_PIPELINE_BIND_POINT_GRAPHICS,
                sceneMeshPipelineLayout, 0, 1, &sceneDescriptorSets[currentFrame], 0, nullptr);
            for (const MeshBatch& batch : meshBatches) {
                if (batch.prototype->visibleInCamera) {
                    pushMeshBatch(batch, true);
                }
            }
        }
        vkCmdEndRendering(frame.commandBuffer);

        // Fase 6 (TAA): resolve temporal - le a cor HDR recem-preenchida
        // acima, a profundidade do pre-pass (Fase 5) e o historico resolvido
        // do quadro passado, e escreve o resultado num dos 2 slots de
        // historico (indexado por currentFrame, ver comentario em
        // sceneTaaHistoryImage). O tonemap, logo depois, passa a ler esse
        // resultado em vez do HDR cru.
        transitionSceneAttachment(frame.commandBuffer, hdrColorImage,
            VK_IMAGE_ASPECT_COLOR_BIT, hdrColorState,
            VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
            VK_PIPELINE_STAGE_2_FRAGMENT_SHADER_BIT,
            VK_ACCESS_2_SHADER_SAMPLED_READ_BIT);
        transitionSceneAttachment(frame.commandBuffer, depthImage,
            VK_IMAGE_ASPECT_DEPTH_BIT, depthState,
            VK_IMAGE_LAYOUT_DEPTH_STENCIL_READ_ONLY_OPTIMAL,
            VK_PIPELINE_STAGE_2_FRAGMENT_SHADER_BIT,
            VK_ACCESS_2_SHADER_SAMPLED_READ_BIT);
        transitionSceneAttachment(frame.commandBuffer, motionVectorImage,
            VK_IMAGE_ASPECT_COLOR_BIT, motionVectorState,
            VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
            VK_PIPELINE_STAGE_2_FRAGMENT_SHADER_BIT,
            VK_ACCESS_2_SHADER_SAMPLED_READ_BIT);

        std::array<VkImage, FramesInFlight>& historyImages = directToSwapchain
            ? sceneDirectTaaHistoryImage : sceneTaaHistoryImage;
        std::array<VkImageView, FramesInFlight>& historyViews = directToSwapchain
            ? sceneDirectTaaHistoryView : sceneTaaHistoryView;
        std::array<SceneAttachmentState3D, FramesInFlight>& historyStates =
            directToSwapchain ? sceneDirectTaaHistoryState : sceneTaaHistoryState;
        const std::uint32_t historyReadIndex =
            (currentFrame + FramesInFlight - 1) % FramesInFlight;

        // Redundante na maioria dos quadros (o slot de leitura ja ficou
        // nesse layout desde que o tonemap do quadro passado o leu), mas
        // necessario tambem no primeiro uso de cada slot (estado UNDEFINED
        // colapsando pra NONE, ver transitionSceneAttachment) - sem isso o
        // binding abaixo apontaria pra uma imagem em layout invalido na
        // primeira vez que este caminho renderiza.
        transitionSceneAttachment(frame.commandBuffer, historyImages[historyReadIndex],
            VK_IMAGE_ASPECT_COLOR_BIT, historyStates[historyReadIndex],
            VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
            VK_PIPELINE_STAGE_2_FRAGMENT_SHADER_BIT,
            VK_ACCESS_2_SHADER_SAMPLED_READ_BIT);

        const VkDescriptorSet taaResolveDescriptorSet = directToSwapchain
            ? sceneDirectTaaResolveDescriptorSets[currentFrame]
            : sceneTaaResolveDescriptorSets[currentFrame];

        transitionSceneAttachment(frame.commandBuffer, historyImages[currentFrame],
            VK_IMAGE_ASPECT_COLOR_BIT, historyStates[currentFrame],
            VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL,
            VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT,
            VK_ACCESS_2_COLOR_ATTACHMENT_WRITE_BIT);

        VkRenderingAttachmentInfo taaResolveAttachment { VK_STRUCTURE_TYPE_RENDERING_ATTACHMENT_INFO };
        taaResolveAttachment.imageView = historyViews[currentFrame];
        taaResolveAttachment.imageLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
        taaResolveAttachment.loadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
        taaResolveAttachment.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
        VkRenderingInfo taaResolveRenderingInfo { VK_STRUCTURE_TYPE_RENDERING_INFO };
        taaResolveRenderingInfo.renderArea.extent = renderExtent;
        taaResolveRenderingInfo.layerCount = 1;
        taaResolveRenderingInfo.colorAttachmentCount = 1;
        taaResolveRenderingInfo.pColorAttachments = &taaResolveAttachment;
        vkCmdBeginRendering(frame.commandBuffer, &taaResolveRenderingInfo);
        setViewportAndScissor(renderExtent);
        vkCmdBindPipeline(frame.commandBuffer, VK_PIPELINE_BIND_POINT_GRAPHICS,
            sceneTaaResolvePipeline);
        const std::array<VkDescriptorSet, 2> taaResolveSets {
            sceneDescriptorSets[currentFrame], taaResolveDescriptorSet
        };
        vkCmdBindDescriptorSets(frame.commandBuffer, VK_PIPELINE_BIND_POINT_GRAPHICS,
            sceneTaaResolvePipelineLayout, 0,
            static_cast<std::uint32_t>(taaResolveSets.size()), taaResolveSets.data(),
            0, nullptr);
        vkCmdDraw(frame.commandBuffer, 3, 1, 0, 0);
        vkCmdEndRendering(frame.commandBuffer);
        // A imagem escrita neste quadro passa a ser uma semente valida para
        // o proximo. Cenas sem TAA (previews) nunca deixam historico ativo.
        temporalHistoryValid = scene.temporalAntiAliasingEnabled;

        // Passe de tonemap: le o resultado ja resolvido pelo TAA acima (nao
        // mais o HDR cru) e escreve o resultado LDR (exposicao + curva
        // filmica + sRGB, ver tonemap.frag) no destino final de verdade - a
        // imagem do swapchain no caminho direto, ou sceneColorImage
        // (amostrada pelo ImGui) no caminho de preview offscreen.
        transitionSceneAttachment(frame.commandBuffer, historyImages[currentFrame],
            VK_IMAGE_ASPECT_COLOR_BIT, historyStates[currentFrame],
            VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
            VK_PIPELINE_STAGE_2_FRAGMENT_SHADER_BIT,
            VK_ACCESS_2_SHADER_SAMPLED_READ_BIT);

        VkImage finalColorImage = directToSwapchain
            ? swapchainImages[currentImage] : sceneColorImage;
        VkImageView finalColorView = directToSwapchain
            ? swapchainImageViews[currentImage] : sceneColorView;
        // A imagem do swapchain e um pool rotativo (identidade de VkImage
        // diferente a cada frame conforme o indice adquirido) - nao ha um
        // SceneAttachmentState3D persistente fazendo sentido para ela; em
        // vez disso, reconstruimos o estado esperado a cada frame a partir
        // de swapchainInitialized (apos apresentar, a imagem sempre fica em
        // PRESENT_SRC_KHR, tendo sido escrita por ultimo como color
        // attachment).
        SceneAttachmentState3D swapchainColorState {
            swapchainInitialized[currentImage]
                ? VK_IMAGE_LAYOUT_PRESENT_SRC_KHR : VK_IMAGE_LAYOUT_UNDEFINED,
            VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT,
            VK_ACCESS_2_NONE
        };
        SceneAttachmentState3D& finalColorState = directToSwapchain
            ? swapchainColorState : sceneColorState;
        transitionSceneAttachment(frame.commandBuffer, finalColorImage,
            VK_IMAGE_ASPECT_COLOR_BIT, finalColorState,
            VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL,
            VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT,
            VK_ACCESS_2_COLOR_ATTACHMENT_WRITE_BIT);

        // DONT_CARE (nao CLEAR): o triangulo cheio de tela do tonemap cobre
        // 100% da area de render, entao nenhum pixel do destino sobrevive do
        // conteudo anterior - limpar antes seria trabalho descartado.
        VkRenderingAttachmentInfo tonemapAttachment { VK_STRUCTURE_TYPE_RENDERING_ATTACHMENT_INFO };
        tonemapAttachment.imageView = finalColorView;
        tonemapAttachment.imageLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
        tonemapAttachment.loadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
        tonemapAttachment.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
        VkRenderingInfo tonemapRenderingInfo { VK_STRUCTURE_TYPE_RENDERING_INFO };
        tonemapRenderingInfo.renderArea.extent = renderExtent;
        tonemapRenderingInfo.layerCount = 1;
        tonemapRenderingInfo.colorAttachmentCount = 1;
        tonemapRenderingInfo.pColorAttachments = &tonemapAttachment;
        vkCmdBeginRendering(frame.commandBuffer, &tonemapRenderingInfo);
        setViewportAndScissor(renderExtent);
        vkCmdBindPipeline(frame.commandBuffer, VK_PIPELINE_BIND_POINT_GRAPHICS, sceneTonemapPipeline);
        const VkDescriptorSet tonemapDescriptorSet = directToSwapchain
            ? sceneDirectTonemapDescriptorSets[currentFrame]
            : sceneTonemapDescriptorSets[currentFrame];
        vkCmdBindDescriptorSets(frame.commandBuffer, VK_PIPELINE_BIND_POINT_GRAPHICS,
            sceneTonemapPipelineLayout, 0, 1, &tonemapDescriptorSet, 0, nullptr);
        const TonemapPushConstantsGpu tonemapPushData {
            scene.toneMapping.exposure, scene.toneMapping.brightness,
            scene.toneMapping.contrast, scene.toneMapping.saturation
        };
        vkCmdPushConstants(frame.commandBuffer, sceneTonemapPipelineLayout,
            VK_SHADER_STAGE_FRAGMENT_BIT, 0, sizeof(TonemapPushConstantsGpu),
            &tonemapPushData);
        vkCmdDraw(frame.commandBuffer, 3, 1, 0, 0);

        if (directToSwapchain) {
            // Este passe de tonemap fica aberto de proposito - o ImGui
            // desenha a UI por cima dele antes do vkCmdEndRendering em
            // endFrame() (ver renderImGui/endFrame). Formato de cor e
            // ausencia de depth attachment aqui batem exatamente com o que
            // o pipeline do proprio ImGui declara (ver initialize()).
            swapchainPassActive = true;
            boundPipeline = {};
            return 0;
        }

        vkCmdEndRendering(frame.commandBuffer);
        transitionSceneAttachment(frame.commandBuffer, sceneColorImage,
            VK_IMAGE_ASPECT_COLOR_BIT, sceneColorState,
            VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
            VK_PIPELINE_STAGE_2_FRAGMENT_SHADER_BIT,
            VK_ACCESS_2_SHADER_SAMPLED_READ_BIT);
        boundPipeline = {};
        return static_cast<std::uint64_t>(
            reinterpret_cast<std::uintptr_t>(sceneColorImGuiDescriptor));
    }

    std::uint64_t renderScene3D(const Scene3DFrame& scene, Extent2D extent) {
        return renderScene3DInternal(scene, extent, false);
    }

    void renderScene3DToSwapchain(const Scene3DFrame& scene) {
        renderScene3DInternal(scene,
            { swapchainExtent.width, swapchainExtent.height }, true);
    }

    void endFrame() {
        if (!frameActive) {
            return;
        }
        if (!swapchainPassActive) {
            throw std::runtime_error("endFrame called without blitToSwapchain opening the swapchain pass");
        }
        Frame& frame = frames[currentFrame];
        vkCmdEndRendering(frame.commandBuffer);
        swapchainPassActive = false;

        VkImageMemoryBarrier2 toPresent { VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER_2 };
        toPresent.srcStageMask = VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT;
        toPresent.srcAccessMask = VK_ACCESS_2_COLOR_ATTACHMENT_WRITE_BIT;
        toPresent.dstStageMask = VK_PIPELINE_STAGE_2_NONE;
        toPresent.dstAccessMask = VK_ACCESS_2_NONE;
        toPresent.oldLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
        toPresent.newLayout = VK_IMAGE_LAYOUT_PRESENT_SRC_KHR;
        toPresent.image = swapchainImages[currentImage];
        toPresent.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
        toPresent.subresourceRange.levelCount = 1;
        toPresent.subresourceRange.layerCount = 1;

        VkDependencyInfo dependency { VK_STRUCTURE_TYPE_DEPENDENCY_INFO };
        dependency.imageMemoryBarrierCount = 1;
        dependency.pImageMemoryBarriers = &toPresent;
        vkCmdPipelineBarrier2(frame.commandBuffer, &dependency);
        vkCmdWriteTimestamp2(frame.commandBuffer,
            VK_PIPELINE_STAGE_2_ALL_COMMANDS_BIT,
            frameTimestampQueryPool, currentFrame * 2 + 1);
        check(vkEndCommandBuffer(frame.commandBuffer), "vkEndCommandBuffer");

        VkSemaphoreSubmitInfo waitInfo { VK_STRUCTURE_TYPE_SEMAPHORE_SUBMIT_INFO };
        waitInfo.semaphore = frame.imageAvailable;
        waitInfo.stageMask = VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT;
        VkCommandBufferSubmitInfo commandInfo { VK_STRUCTURE_TYPE_COMMAND_BUFFER_SUBMIT_INFO };
        commandInfo.commandBuffer = frame.commandBuffer;
        VkSemaphoreSubmitInfo signalInfo { VK_STRUCTURE_TYPE_SEMAPHORE_SUBMIT_INFO };
        signalInfo.semaphore = frame.renderFinished;
        signalInfo.stageMask = VK_PIPELINE_STAGE_2_ALL_GRAPHICS_BIT;

        VkSubmitInfo2 submit { VK_STRUCTURE_TYPE_SUBMIT_INFO_2 };
        submit.waitSemaphoreInfoCount = 1;
        submit.pWaitSemaphoreInfos = &waitInfo;
        submit.commandBufferInfoCount = 1;
        submit.pCommandBufferInfos = &commandInfo;
        submit.signalSemaphoreInfoCount = 1;
        submit.pSignalSemaphoreInfos = &signalInfo;
        check(vkQueueSubmit2(graphicsQueue, 1, &submit, frame.fence), "vkQueueSubmit2");

        VkPresentInfoKHR present { VK_STRUCTURE_TYPE_PRESENT_INFO_KHR };
        present.waitSemaphoreCount = 1;
        present.pWaitSemaphores = &frame.renderFinished;
        present.swapchainCount = 1;
        present.pSwapchains = &swapchain;
        present.pImageIndices = &currentImage;
        const auto presentStart = std::chrono::steady_clock::now();
        const VkResult presented = vkQueuePresentKHR(presentQueue, &present);
        frame.performance.cpuPresentMilliseconds =
            std::chrono::duration<float, std::milli>(
                std::chrono::steady_clock::now() - presentStart).count();
        if (presented == VK_ERROR_OUT_OF_DATE_KHR || presented == VK_SUBOPTIMAL_KHR) {
            swapchainDirty = true;
        } else if (presented != VK_SUCCESS) {
            check(presented, "vkQueuePresentKHR");
        }

        swapchainInitialized[currentImage] = true;
        frame.timestampWritten = true;
        currentFrame = (currentFrame + 1) % FramesInFlight;
        frameActive = false;
    }

    void setViewport(Extent2D extent) override {
        ensureFrame();
        VkViewport viewport {};
        viewport.width = static_cast<float>(extent.width);
        viewport.height = static_cast<float>(extent.height);
        viewport.minDepth = 0.0f;
        viewport.maxDepth = 1.0f;
        vkCmdSetViewport(frames[currentFrame].commandBuffer, 0, 1, &viewport);
    }

    void setScissor(Extent2D extent) override {
        ensureFrame();
        VkRect2D scissor {};
        scissor.extent = { extent.width, extent.height };
        vkCmdSetScissor(frames[currentFrame].commandBuffer, 0, 1, &scissor);
    }

    void bindPipeline(PipelineHandle pipeline) override {
        ensureFrame();
        PipelineResource& resource = checkedResource(pipelines, pipeline, "pipeline");
        vkCmdBindPipeline(frames[currentFrame].commandBuffer, VK_PIPELINE_BIND_POINT_GRAPHICS, resource.pipeline);
        boundPipeline = pipeline;
    }

    void bindVertexBuffer(BufferHandle buffer, std::size_t offset) override {
        ensureFrame();
        BufferResource& resource = checkedResource(buffers, buffer, "vertex buffer");
        const VkDeviceSize vkOffset = offset;
        vkCmdBindVertexBuffers(frames[currentFrame].commandBuffer, 0, 1, &resource.buffer, &vkOffset);
    }

    void bindIndexBuffer(BufferHandle buffer, IndexType type, std::size_t offset) override {
        ensureFrame();
        BufferResource& resource = checkedResource(buffers, buffer, "index buffer");
        vkCmdBindIndexBuffer(frames[currentFrame].commandBuffer, resource.buffer, offset,
            type == IndexType::UInt16 ? VK_INDEX_TYPE_UINT16 : VK_INDEX_TYPE_UINT32);
    }

    void pushConstants(ShaderStage stages, std::span<const std::byte> data) override {
        ensureFrame();
        PipelineResource& pipeline = checkedResource(pipelines, boundPipeline, "bound pipeline");
        vkCmdPushConstants(frames[currentFrame].commandBuffer, pipeline.layout,
            shaderStages(stages), 0, static_cast<std::uint32_t>(data.size()), data.data());
    }

    void drawIndexed(std::uint32_t indexCount, std::uint32_t firstIndex, std::int32_t vertexOffset) override {
        ensureFrame();
        vkCmdDrawIndexed(frames[currentFrame].commandBuffer, indexCount, 1, firstIndex, vertexOffset, 0);
    }

    void initializeImGui(SDL_Window* sdlWindow) {
        if (imguiInitialized) {
            return;
        }
        if (!ImGui_ImplSDL3_InitForVulkan(sdlWindow)) {
            throw std::runtime_error("ImGui SDL3 Vulkan backend initialization failed");
        }

        ImGui_ImplVulkan_InitInfo init {};
        init.ApiVersion = TargetVulkanVersion;
        init.Instance = instance;
        init.PhysicalDevice = physicalDevice;
        init.Device = device;
        init.QueueFamily = queueFamilies.graphics;
        init.Queue = graphicsQueue;
        init.DescriptorPoolSize = 256;
        init.MinImageCount = std::max(2u, minImageCount);
        init.ImageCount = static_cast<std::uint32_t>(swapchainImages.size());
        init.PipelineInfoMain.MSAASamples = VK_SAMPLE_COUNT_1_BIT;
        init.UseDynamicRendering = true;
        init.PipelineInfoMain.PipelineRenderingCreateInfo.sType =
            VK_STRUCTURE_TYPE_PIPELINE_RENDERING_CREATE_INFO_KHR;
        init.PipelineInfoMain.PipelineRenderingCreateInfo.colorAttachmentCount = 1;
        init.PipelineInfoMain.PipelineRenderingCreateInfo.pColorAttachmentFormats =
            &swapchainFormat;
        init.CheckVkResultFn = [](VkResult result) {
            if (result != VK_SUCCESS) {
                Log::error("Dear ImGui Vulkan backend returned VkResult " + std::to_string(result));
            }
        };
        if (!ImGui_ImplVulkan_Init(&init)) {
            ImGui_ImplSDL3_Shutdown();
            throw std::runtime_error("ImGui Vulkan renderer backend initialization failed");
        }
        imguiInitialized = true;
    }

    void shutdownImGui() {
        if (!imguiInitialized) {
            return;
        }
        if (device != VK_NULL_HANDLE) {
            vkDeviceWaitIdle(device);
        }
        for (TextureResource& resource : textures) {
            if (resource.alive && resource.imguiDescriptor != VK_NULL_HANDLE) {
                ImGui_ImplVulkan_RemoveTexture(resource.imguiDescriptor);
                resource.imguiDescriptor = VK_NULL_HANDLE;
            }
        }
        ImGui_ImplVulkan_Shutdown();
        ImGui_ImplSDL3_Shutdown();
        imguiInitialized = false;
    }

    void beginImGuiFrame() {
        ImGui_ImplVulkan_NewFrame();
        ImGui_ImplSDL3_NewFrame();
    }

    void renderImGui(ImDrawData* drawData) {
        if (frameActive && drawData != nullptr) {
            ImGui_ImplVulkan_RenderDrawData(drawData, frames[currentFrame].commandBuffer);
        }
    }

    void createInstance(const std::string& applicationName) {
        VkApplicationInfo application { VK_STRUCTURE_TYPE_APPLICATION_INFO };
        application.pApplicationName = applicationName.c_str();
        application.applicationVersion = VK_MAKE_API_VERSION(0, 0, 1, 0);
        application.pEngineName = "MatterEngine";
        application.engineVersion = VK_MAKE_API_VERSION(0,
            MatterEngine::VersionMajor, MatterEngine::VersionMinor, MatterEngine::VersionPatch);
        application.apiVersion = TargetVulkanVersion;

        Uint32 sdlExtensionCount = 0;
        const char* const* sdlExtensions = SDL_Vulkan_GetInstanceExtensions(&sdlExtensionCount);
        if (sdlExtensions == nullptr) {
            throw std::runtime_error(std::string("SDL_Vulkan_GetInstanceExtensions failed: ") + SDL_GetError());
        }
        std::vector<const char*> extensions(sdlExtensions, sdlExtensions + sdlExtensionCount);

        validationEnabled = hasInstanceLayer("VK_LAYER_KHRONOS_validation");
        if (validationEnabled && hasInstanceExtension(VK_EXT_DEBUG_UTILS_EXTENSION_NAME)) {
            extensions.push_back(VK_EXT_DEBUG_UTILS_EXTENSION_NAME);
            debugUtilsEnabled = true;
        } else if (!validationEnabled) {
            Log::warn("VK_LAYER_KHRONOS_validation is not installed; continuing without validation.");
        }

        VkInstanceCreateFlags flags = 0;
#ifdef VK_KHR_PORTABILITY_ENUMERATION_EXTENSION_NAME
        if (hasInstanceExtension(VK_KHR_PORTABILITY_ENUMERATION_EXTENSION_NAME)) {
            extensions.push_back(VK_KHR_PORTABILITY_ENUMERATION_EXTENSION_NAME);
            flags |= VK_INSTANCE_CREATE_ENUMERATE_PORTABILITY_BIT_KHR;
        }
#endif

        const char* validationLayer = "VK_LAYER_KHRONOS_validation";
        VkDebugUtilsMessengerCreateInfoEXT debugInfo { VK_STRUCTURE_TYPE_DEBUG_UTILS_MESSENGER_CREATE_INFO_EXT };
        debugInfo.messageSeverity = VK_DEBUG_UTILS_MESSAGE_SEVERITY_WARNING_BIT_EXT
            | VK_DEBUG_UTILS_MESSAGE_SEVERITY_ERROR_BIT_EXT;
        debugInfo.messageType = VK_DEBUG_UTILS_MESSAGE_TYPE_GENERAL_BIT_EXT
            | VK_DEBUG_UTILS_MESSAGE_TYPE_VALIDATION_BIT_EXT
            | VK_DEBUG_UTILS_MESSAGE_TYPE_PERFORMANCE_BIT_EXT;
        debugInfo.pfnUserCallback = debugCallback;

        VkInstanceCreateInfo createInfo { VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO };
        createInfo.flags = flags;
        createInfo.pApplicationInfo = &application;
        createInfo.enabledExtensionCount = static_cast<std::uint32_t>(extensions.size());
        createInfo.ppEnabledExtensionNames = extensions.data();
        if (validationEnabled) {
            createInfo.enabledLayerCount = 1;
            createInfo.ppEnabledLayerNames = &validationLayer;
            if (debugUtilsEnabled) {
                createInfo.pNext = &debugInfo;
            }
        }
        check(vkCreateInstance(&createInfo, nullptr, &instance), "vkCreateInstance");
    }

    void createDebugMessenger() {
        if (!debugUtilsEnabled || vkCreateDebugUtilsMessengerEXT == nullptr) {
            return;
        }
        VkDebugUtilsMessengerCreateInfoEXT createInfo { VK_STRUCTURE_TYPE_DEBUG_UTILS_MESSENGER_CREATE_INFO_EXT };
        createInfo.messageSeverity = VK_DEBUG_UTILS_MESSAGE_SEVERITY_VERBOSE_BIT_EXT
            | VK_DEBUG_UTILS_MESSAGE_SEVERITY_WARNING_BIT_EXT
            | VK_DEBUG_UTILS_MESSAGE_SEVERITY_ERROR_BIT_EXT;
        createInfo.messageType = VK_DEBUG_UTILS_MESSAGE_TYPE_GENERAL_BIT_EXT
            | VK_DEBUG_UTILS_MESSAGE_TYPE_VALIDATION_BIT_EXT
            | VK_DEBUG_UTILS_MESSAGE_TYPE_PERFORMANCE_BIT_EXT;
        createInfo.pfnUserCallback = debugCallback;
        check(vkCreateDebugUtilsMessengerEXT(instance, &createInfo, nullptr, &debugMessenger),
            "vkCreateDebugUtilsMessengerEXT");
    }

    QueueFamilies findQueueFamilies(VkPhysicalDevice candidate) const {
        QueueFamilies result;
        std::uint32_t count = 0;
        vkGetPhysicalDeviceQueueFamilyProperties(candidate, &count, nullptr);
        std::vector<VkQueueFamilyProperties> families(count);
        vkGetPhysicalDeviceQueueFamilyProperties(candidate, &count, families.data());
        for (std::uint32_t i = 0; i < count; ++i) {
            if ((families[i].queueFlags & VK_QUEUE_GRAPHICS_BIT) != 0) {
                result.graphics = i;
            }
            VkBool32 presentSupport = VK_FALSE;
            vkGetPhysicalDeviceSurfaceSupportKHR(candidate, i, surface, &presentSupport);
            if (presentSupport == VK_TRUE) {
                result.present = i;
            }
            if (result.complete()) {
                break;
            }
        }
        return result;
    }

    bool deviceSupportsSwapchain(VkPhysicalDevice candidate) const {
        std::uint32_t extensionCount = 0;
        vkEnumerateDeviceExtensionProperties(candidate, nullptr, &extensionCount, nullptr);
        std::vector<VkExtensionProperties> extensions(extensionCount);
        vkEnumerateDeviceExtensionProperties(candidate, nullptr, &extensionCount, extensions.data());
        const bool hasSwapchain = std::any_of(extensions.begin(), extensions.end(), [](const auto& extension) {
            return std::strcmp(extension.extensionName, VK_KHR_SWAPCHAIN_EXTENSION_NAME) == 0;
        });
        if (!hasSwapchain) {
            return false;
        }
        std::uint32_t formatCount = 0;
        std::uint32_t presentCount = 0;
        vkGetPhysicalDeviceSurfaceFormatsKHR(candidate, surface, &formatCount, nullptr);
        vkGetPhysicalDeviceSurfacePresentModesKHR(candidate, surface, &presentCount, nullptr);
        return formatCount > 0 && presentCount > 0;
    }

    void selectPhysicalDevice() {
        std::uint32_t count = 0;
        check(vkEnumeratePhysicalDevices(instance, &count, nullptr), "vkEnumeratePhysicalDevices");
        if (count == 0) {
            throw std::runtime_error("No Vulkan-capable GPU was found");
        }
        std::vector<VkPhysicalDevice> candidates(count);
        check(vkEnumeratePhysicalDevices(instance, &count, candidates.data()), "vkEnumeratePhysicalDevices");

        int bestScore = -1;
        for (VkPhysicalDevice candidate : candidates) {
            VkPhysicalDeviceProperties properties {};
            vkGetPhysicalDeviceProperties(candidate, &properties);
            if (properties.apiVersion < TargetVulkanVersion) {
                continue;
            }
            const QueueFamilies families = findQueueFamilies(candidate);
            if (!families.complete() || !deviceSupportsSwapchain(candidate)) {
                continue;
            }

            VkPhysicalDeviceVulkan13Features features13 { VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_VULKAN_1_3_FEATURES };
            VkPhysicalDeviceFeatures2 features { VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_FEATURES_2 };
            features.pNext = &features13;
            vkGetPhysicalDeviceFeatures2(candidate, &features);
            if (features13.dynamicRendering != VK_TRUE || features13.synchronization2 != VK_TRUE) {
                continue;
            }
            // Filtragem anisotropica pras texturas de material (ver
            // materialSamplerInfo em createScene3DResources) - parte do
            // conjunto minimo obrigatorio do Vulkan pra qualquer GPU real,
            // mas checado explicitamente aqui em vez de assumido, no mesmo
            // espirito das outras features acima.
            if (features.features.samplerAnisotropy != VK_TRUE) {
                continue;
            }

            int score = static_cast<int>(properties.limits.maxImageDimension2D);
            if (properties.deviceType == VK_PHYSICAL_DEVICE_TYPE_DISCRETE_GPU) score += 100000;
            if (properties.deviceType == VK_PHYSICAL_DEVICE_TYPE_INTEGRATED_GPU) score += 50000;
            if (score > bestScore) {
                bestScore = score;
                physicalDevice = candidate;
                queueFamilies = families;
            }
        }
        if (physicalDevice == VK_NULL_HANDLE) {
            throw std::runtime_error("No suitable Vulkan 1.4 GPU with dynamic rendering was found");
        }
    }

    void createLogicalDevice() {
        const std::set<std::uint32_t> uniqueFamilies { queueFamilies.graphics, queueFamilies.present };
        const float priority = 1.0f;
        std::vector<VkDeviceQueueCreateInfo> queues;
        for (std::uint32_t family : uniqueFamilies) {
            VkDeviceQueueCreateInfo queue { VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO };
            queue.queueFamilyIndex = family;
            queue.queueCount = 1;
            queue.pQueuePriorities = &priority;
            queues.push_back(queue);
        }

        VkPhysicalDeviceVulkan13Features features13 { VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_VULKAN_1_3_FEATURES };
        features13.dynamicRendering = VK_TRUE;
        features13.synchronization2 = VK_TRUE;

        // samplerAnisotropy (checado em selectPhysicalDevice) precisa ser
        // pedido explicitamente aqui pra virar utilizavel - sem isso,
        // VkSamplerCreateInfo::anisotropyEnable=VK_TRUE seria invalido.
        // pEnabledFeatures (base) e pNext=VkPhysicalDeviceFeatures2 sao
        // mutuamente exclusivos pela spec - como o pNext desta struct so
        // encadeia VkPhysicalDeviceVulkan13Features (features "por versao",
        // nao a struct base), usar pEnabledFeatures aqui continua valido.
        VkPhysicalDeviceFeatures enabledFeatures {};
        enabledFeatures.samplerAnisotropy = VK_TRUE;

        const char* extension = VK_KHR_SWAPCHAIN_EXTENSION_NAME;
        VkDeviceCreateInfo createInfo { VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO };
        createInfo.pNext = &features13;
        createInfo.queueCreateInfoCount = static_cast<std::uint32_t>(queues.size());
        createInfo.pQueueCreateInfos = queues.data();
        createInfo.enabledExtensionCount = 1;
        createInfo.ppEnabledExtensionNames = &extension;
        createInfo.pEnabledFeatures = &enabledFeatures;
        check(vkCreateDevice(physicalDevice, &createInfo, nullptr, &device), "vkCreateDevice");
        vkGetDeviceQueue(device, queueFamilies.graphics, 0, &graphicsQueue);
        vkGetDeviceQueue(device, queueFamilies.present, 0, &presentQueue);
    }

    void createAllocator() {
        VmaVulkanFunctions functions {};
        functions.vkGetInstanceProcAddr = vkGetInstanceProcAddr;
        functions.vkGetDeviceProcAddr = vkGetDeviceProcAddr;

        VmaAllocatorCreateInfo createInfo {};
        createInfo.instance = instance;
        createInfo.physicalDevice = physicalDevice;
        createInfo.device = device;
        createInfo.vulkanApiVersion = TargetVulkanVersion;
        createInfo.pVulkanFunctions = &functions;
        check(vmaCreateAllocator(&createInfo, &allocator), "vmaCreateAllocator");
    }

    void createFrames() {
        for (Frame& frame : frames) {
            VkCommandPoolCreateInfo poolInfo { VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO };
            poolInfo.flags = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;
            poolInfo.queueFamilyIndex = queueFamilies.graphics;
            check(vkCreateCommandPool(device, &poolInfo, nullptr, &frame.commandPool), "vkCreateCommandPool");

            VkCommandBufferAllocateInfo commandInfo { VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO };
            commandInfo.commandPool = frame.commandPool;
            commandInfo.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
            commandInfo.commandBufferCount = 1;
            check(vkAllocateCommandBuffers(device, &commandInfo, &frame.commandBuffer), "vkAllocateCommandBuffers");

            VkSemaphoreCreateInfo semaphoreInfo { VK_STRUCTURE_TYPE_SEMAPHORE_CREATE_INFO };
            check(vkCreateSemaphore(device, &semaphoreInfo, nullptr, &frame.imageAvailable), "vkCreateSemaphore");
            check(vkCreateSemaphore(device, &semaphoreInfo, nullptr, &frame.renderFinished), "vkCreateSemaphore");

            VkFenceCreateInfo fenceInfo { VK_STRUCTURE_TYPE_FENCE_CREATE_INFO };
            fenceInfo.flags = VK_FENCE_CREATE_SIGNALED_BIT;
            check(vkCreateFence(device, &fenceInfo, nullptr, &frame.fence), "vkCreateFence");
        }
        VkQueryPoolCreateInfo queryInfo {
            VK_STRUCTURE_TYPE_QUERY_POOL_CREATE_INFO
        };
        queryInfo.queryType = VK_QUERY_TYPE_TIMESTAMP;
        queryInfo.queryCount = FramesInFlight * 2;
        check(vkCreateQueryPool(device, &queryInfo, nullptr,
            &frameTimestampQueryPool),
            "vkCreateQueryPool(frame timestamps)");
    }

    void createSwapchain() {
        int pixelWidth = 0;
        int pixelHeight = 0;
        SDL_GetWindowSizeInPixels(window, &pixelWidth, &pixelHeight);
        if (pixelWidth <= 0 || pixelHeight <= 0) {
            swapchainDirty = true;
            return;
        }

        VkSurfaceCapabilitiesKHR capabilities {};
        check(vkGetPhysicalDeviceSurfaceCapabilitiesKHR(physicalDevice, surface, &capabilities),
            "vkGetPhysicalDeviceSurfaceCapabilitiesKHR");

        std::uint32_t formatCount = 0;
        vkGetPhysicalDeviceSurfaceFormatsKHR(physicalDevice, surface, &formatCount, nullptr);
        std::vector<VkSurfaceFormatKHR> formats(formatCount);
        vkGetPhysicalDeviceSurfaceFormatsKHR(physicalDevice, surface, &formatCount, formats.data());
        // UNORM, not SRGB: Render2D and ImGui both write display-ready bytes
        // straight through (no linear-light pipeline, no shader gamma pass) -
        // an _SRGB swapchain format makes the driver re-encode those bytes on
        // write, washing everything out (dark near-black backgrounds turn
        // grey, saturated reds turn salmon). UNORM stores exactly the bytes
        // the shader outputs, matching how the original OpenGL renderer
        // displayed these same color constants.
        VkSurfaceFormatKHR selectedFormat = formats.front();
        for (const VkSurfaceFormatKHR& format : formats) {
            if (format.format == VK_FORMAT_B8G8R8A8_UNORM
                && format.colorSpace == VK_COLOR_SPACE_SRGB_NONLINEAR_KHR) {
                selectedFormat = format;
                break;
            }
        }

        std::uint32_t presentModeCount = 0;
        vkGetPhysicalDeviceSurfacePresentModesKHR(physicalDevice, surface, &presentModeCount, nullptr);
        std::vector<VkPresentModeKHR> presentModes(presentModeCount);
        vkGetPhysicalDeviceSurfacePresentModesKHR(physicalDevice, surface, &presentModeCount, presentModes.data());
        VkPresentModeKHR selectedPresent = VK_PRESENT_MODE_FIFO_KHR;
        if (!vsync) {
            // IMMEDIATE não espera o blanking do monitor e, portanto, fornece
            // uma medição uncapped do custo real do frame. MAILBOX é o fallback
            // sem VSync quando o driver não expõe apresentação imediata.
            if (std::find(presentModes.begin(), presentModes.end(), VK_PRESENT_MODE_IMMEDIATE_KHR) != presentModes.end()) {
                selectedPresent = VK_PRESENT_MODE_IMMEDIATE_KHR;
            } else if (std::find(presentModes.begin(), presentModes.end(), VK_PRESENT_MODE_MAILBOX_KHR) != presentModes.end()) {
                selectedPresent = VK_PRESENT_MODE_MAILBOX_KHR;
            }
        }

        VkExtent2D extent {};
        if (capabilities.currentExtent.width != std::numeric_limits<std::uint32_t>::max()) {
            extent = capabilities.currentExtent;
        } else {
            extent.width = std::clamp(static_cast<std::uint32_t>(pixelWidth),
                capabilities.minImageExtent.width, capabilities.maxImageExtent.width);
            extent.height = std::clamp(static_cast<std::uint32_t>(pixelHeight),
                capabilities.minImageExtent.height, capabilities.maxImageExtent.height);
        }
        // Windows can report the previous SDL pixel size for a short window
        // while the Vulkan surface has already become 0x0 during minimize.
        // A zero-size swapchain is neither renderable nor valid to feed into
        // the direct 3D path; defer recreation until the window is restored.
        if (extent.width == 0 || extent.height == 0) {
            swapchainDirty = true;
            return;
        }

        minImageCount = capabilities.minImageCount;
        std::uint32_t requestedImages = std::max(capabilities.minImageCount + 1, FramesInFlight);
        if (capabilities.maxImageCount > 0) {
            requestedImages = std::min(requestedImages, capabilities.maxImageCount);
        }

        const std::array<std::uint32_t, 2> families { queueFamilies.graphics, queueFamilies.present };
        VkSwapchainCreateInfoKHR createInfo { VK_STRUCTURE_TYPE_SWAPCHAIN_CREATE_INFO_KHR };
        createInfo.surface = surface;
        createInfo.minImageCount = requestedImages;
        createInfo.imageFormat = selectedFormat.format;
        createInfo.imageColorSpace = selectedFormat.colorSpace;
        createInfo.imageExtent = extent;
        createInfo.imageArrayLayers = 1;
        createInfo.imageUsage = VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT;
        if (queueFamilies.graphics != queueFamilies.present) {
            createInfo.imageSharingMode = VK_SHARING_MODE_CONCURRENT;
            createInfo.queueFamilyIndexCount = 2;
            createInfo.pQueueFamilyIndices = families.data();
        } else {
            createInfo.imageSharingMode = VK_SHARING_MODE_EXCLUSIVE;
        }
        createInfo.preTransform = capabilities.currentTransform;
        createInfo.compositeAlpha = VK_COMPOSITE_ALPHA_OPAQUE_BIT_KHR;
        createInfo.presentMode = selectedPresent;
        createInfo.clipped = VK_TRUE;
        check(vkCreateSwapchainKHR(device, &createInfo, nullptr, &swapchain), "vkCreateSwapchainKHR");

        std::uint32_t imageCount = 0;
        vkGetSwapchainImagesKHR(device, swapchain, &imageCount, nullptr);
        swapchainImages.resize(imageCount);
        vkGetSwapchainImagesKHR(device, swapchain, &imageCount, swapchainImages.data());
        swapchainImageViews.resize(imageCount);
        for (std::size_t i = 0; i < swapchainImages.size(); ++i) {
            VkImageViewCreateInfo viewInfo { VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO };
            viewInfo.image = swapchainImages[i];
            viewInfo.viewType = VK_IMAGE_VIEW_TYPE_2D;
            viewInfo.format = selectedFormat.format;
            viewInfo.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
            viewInfo.subresourceRange.levelCount = 1;
            viewInfo.subresourceRange.layerCount = 1;
            check(vkCreateImageView(device, &viewInfo, nullptr, &swapchainImageViews[i]), "vkCreateImageView");
        }
        swapchainFormat = selectedFormat.format;
        swapchainExtent = extent;
        swapchainInitialized.assign(imageCount, false);
        imageFences.assign(imageCount, VK_NULL_HANDLE);
        swapchainDirty = false;
    }

    void destroySwapchain() {
        for (VkImageView view : swapchainImageViews) {
            vkDestroyImageView(device, view, nullptr);
        }
        swapchainImageViews.clear();
        swapchainImages.clear();
        swapchainInitialized.clear();
        imageFences.clear();
        if (swapchain != VK_NULL_HANDLE) {
            vkDestroySwapchainKHR(device, swapchain, nullptr);
            swapchain = VK_NULL_HANDLE;
        }
        swapchainExtent = {};
    }

    void destroyScene3DTarget() {
        if (sceneColorImGuiDescriptor != VK_NULL_HANDLE && imguiInitialized) {
            ImGui_ImplVulkan_RemoveTexture(sceneColorImGuiDescriptor);
        }
        sceneColorImGuiDescriptor = VK_NULL_HANDLE;
        if (sceneColorView != VK_NULL_HANDLE) vkDestroyImageView(device, sceneColorView, nullptr);
        if (sceneDepthView != VK_NULL_HANDLE) vkDestroyImageView(device, sceneDepthView, nullptr);
        if (sceneHdrColorView != VK_NULL_HANDLE) vkDestroyImageView(device, sceneHdrColorView, nullptr);
        if (sceneMotionVectorView != VK_NULL_HANDLE) vkDestroyImageView(device, sceneMotionVectorView, nullptr);
        if (sceneColorImage != VK_NULL_HANDLE) vmaDestroyImage(allocator, sceneColorImage, sceneColorAllocation);
        if (sceneDepthImage != VK_NULL_HANDLE) vmaDestroyImage(allocator, sceneDepthImage, sceneDepthAllocation);
        if (sceneHdrColorImage != VK_NULL_HANDLE) vmaDestroyImage(allocator, sceneHdrColorImage, sceneHdrColorAllocation);
        if (sceneMotionVectorImage != VK_NULL_HANDLE) vmaDestroyImage(allocator, sceneMotionVectorImage, sceneMotionVectorAllocation);
        sceneColorImage = VK_NULL_HANDLE;
        sceneDepthImage = VK_NULL_HANDLE;
        sceneHdrColorImage = VK_NULL_HANDLE;
        sceneMotionVectorImage = VK_NULL_HANDLE;
        sceneColorAllocation = VK_NULL_HANDLE;
        sceneDepthAllocation = VK_NULL_HANDLE;
        sceneHdrColorAllocation = VK_NULL_HANDLE;
        sceneMotionVectorAllocation = VK_NULL_HANDLE;
        sceneColorView = VK_NULL_HANDLE;
        sceneDepthView = VK_NULL_HANDLE;
        sceneHdrColorView = VK_NULL_HANDLE;
        sceneMotionVectorView = VK_NULL_HANDLE;
        sceneColorState = {};
        sceneDepthState = {};
        sceneHdrColorState = {};
        sceneMotionVectorState = {};
        for (std::size_t index = 0; index < sceneTaaHistoryImage.size(); ++index) {
            if (sceneTaaHistoryView[index] != VK_NULL_HANDLE) {
                vkDestroyImageView(device, sceneTaaHistoryView[index], nullptr);
            }
            if (sceneTaaHistoryImage[index] != VK_NULL_HANDLE) {
                vmaDestroyImage(allocator, sceneTaaHistoryImage[index],
                    sceneTaaHistoryAllocation[index]);
            }
            sceneTaaHistoryImage[index] = VK_NULL_HANDLE;
            sceneTaaHistoryAllocation[index] = VK_NULL_HANDLE;
            sceneTaaHistoryView[index] = VK_NULL_HANDLE;
            sceneTaaHistoryState[index] = {};
        }
        sceneTaaHistoryValid = false;
        sceneTargetExtent = {};
    }

    void destroySceneShadowTarget() {
        for (std::uint32_t cascade = 0; cascade < ShadowCascadeCount; ++cascade) {
            if (sceneShadowViews[cascade] != VK_NULL_HANDLE) {
                vkDestroyImageView(device, sceneShadowViews[cascade], nullptr);
            }
            if (sceneShadowImages[cascade] != VK_NULL_HANDLE) {
                vmaDestroyImage(allocator, sceneShadowImages[cascade],
                    sceneShadowAllocations[cascade]);
            }
            sceneShadowImages[cascade] = VK_NULL_HANDLE;
            sceneShadowAllocations[cascade] = VK_NULL_HANDLE;
            sceneShadowViews[cascade] = VK_NULL_HANDLE;
            sceneShadowStates[cascade] = {};
        }
    }

    void destroySceneDirectDepth() {
        if (sceneDirectDepthView != VK_NULL_HANDLE) {
            vkDestroyImageView(device, sceneDirectDepthView, nullptr);
        }
        if (sceneDirectHdrColorView != VK_NULL_HANDLE) {
            vkDestroyImageView(device, sceneDirectHdrColorView, nullptr);
        }
        if (sceneDirectMotionVectorView != VK_NULL_HANDLE) {
            vkDestroyImageView(device, sceneDirectMotionVectorView, nullptr);
        }
        if (sceneDirectDepthImage != VK_NULL_HANDLE) {
            vmaDestroyImage(allocator, sceneDirectDepthImage, sceneDirectDepthAllocation);
        }
        if (sceneDirectHdrColorImage != VK_NULL_HANDLE) {
            vmaDestroyImage(allocator, sceneDirectHdrColorImage, sceneDirectHdrColorAllocation);
        }
        if (sceneDirectMotionVectorImage != VK_NULL_HANDLE) {
            vmaDestroyImage(allocator, sceneDirectMotionVectorImage, sceneDirectMotionVectorAllocation);
        }
        sceneDirectDepthImage = VK_NULL_HANDLE;
        sceneDirectHdrColorImage = VK_NULL_HANDLE;
        sceneDirectMotionVectorImage = VK_NULL_HANDLE;
        sceneDirectDepthAllocation = VK_NULL_HANDLE;
        sceneDirectHdrColorAllocation = VK_NULL_HANDLE;
        sceneDirectMotionVectorAllocation = VK_NULL_HANDLE;
        sceneDirectDepthView = VK_NULL_HANDLE;
        sceneDirectHdrColorView = VK_NULL_HANDLE;
        sceneDirectMotionVectorView = VK_NULL_HANDLE;
        sceneDirectDepthState = {};
        sceneDirectHdrColorState = {};
        sceneDirectMotionVectorState = {};
        for (std::size_t index = 0; index < sceneDirectTaaHistoryImage.size(); ++index) {
            if (sceneDirectTaaHistoryView[index] != VK_NULL_HANDLE) {
                vkDestroyImageView(device, sceneDirectTaaHistoryView[index], nullptr);
            }
            if (sceneDirectTaaHistoryImage[index] != VK_NULL_HANDLE) {
                vmaDestroyImage(allocator, sceneDirectTaaHistoryImage[index],
                    sceneDirectTaaHistoryAllocation[index]);
            }
            sceneDirectTaaHistoryImage[index] = VK_NULL_HANDLE;
            sceneDirectTaaHistoryAllocation[index] = VK_NULL_HANDLE;
            sceneDirectTaaHistoryView[index] = VK_NULL_HANDLE;
            sceneDirectTaaHistoryState[index] = {};
        }
        sceneDirectTaaHistoryValid = false;
        sceneDirectExtent = {};
    }

    void destroyScene3DResources() {
        if (device == VK_NULL_HANDLE) {
            return;
        }
        destroyScene3DTarget();
        destroySceneDirectDepth();
        destroySceneShadowTarget();
        if (sceneSkyPipeline != VK_NULL_HANDLE) vkDestroyPipeline(device, sceneSkyPipeline, nullptr);
        if (sceneMeshPipeline != VK_NULL_HANDLE) vkDestroyPipeline(device, sceneMeshPipeline, nullptr);
        if (sceneMeshShadowPipeline != VK_NULL_HANDLE) vkDestroyPipeline(device, sceneMeshShadowPipeline, nullptr);
        if (sceneMeshDepthPrepassPipeline != VK_NULL_HANDLE) vkDestroyPipeline(device, sceneMeshDepthPrepassPipeline, nullptr);
        if (sceneTonemapPipeline != VK_NULL_HANDLE) vkDestroyPipeline(device, sceneTonemapPipeline, nullptr);
        if (sceneTaaResolvePipeline != VK_NULL_HANDLE) vkDestroyPipeline(device, sceneTaaResolvePipeline, nullptr);
        if (sceneMeshPipelineLayout != VK_NULL_HANDLE) vkDestroyPipelineLayout(device, sceneMeshPipelineLayout, nullptr);
        if (sceneTonemapPipelineLayout != VK_NULL_HANDLE) vkDestroyPipelineLayout(device, sceneTonemapPipelineLayout, nullptr);
        if (sceneTaaResolvePipelineLayout != VK_NULL_HANDLE) vkDestroyPipelineLayout(device, sceneTaaResolvePipelineLayout, nullptr);
        if (materialDescriptorPool != VK_NULL_HANDLE) vkDestroyDescriptorPool(device, materialDescriptorPool, nullptr);
        if (materialDescriptorSetLayout != VK_NULL_HANDLE) vkDestroyDescriptorSetLayout(device, materialDescriptorSetLayout, nullptr);
        if (materialSampler != VK_NULL_HANDLE) vkDestroySampler(device, materialSampler, nullptr);
        if (scenePipelineLayout != VK_NULL_HANDLE) vkDestroyPipelineLayout(device, scenePipelineLayout, nullptr);
        if (sceneDescriptorPool != VK_NULL_HANDLE) vkDestroyDescriptorPool(device, sceneDescriptorPool, nullptr);
        if (sceneDescriptorSetLayout != VK_NULL_HANDLE) vkDestroyDescriptorSetLayout(device, sceneDescriptorSetLayout, nullptr);
        if (sceneTonemapDescriptorPool != VK_NULL_HANDLE) vkDestroyDescriptorPool(device, sceneTonemapDescriptorPool, nullptr);
        if (sceneTonemapDescriptorSetLayout != VK_NULL_HANDLE) vkDestroyDescriptorSetLayout(device, sceneTonemapDescriptorSetLayout, nullptr);
        if (sceneTonemapSampler != VK_NULL_HANDLE) vkDestroySampler(device, sceneTonemapSampler, nullptr);
        if (sceneTaaResolveDescriptorPool != VK_NULL_HANDLE) vkDestroyDescriptorPool(device, sceneTaaResolveDescriptorPool, nullptr);
        if (sceneTaaResolveDescriptorSetLayout != VK_NULL_HANDLE) vkDestroyDescriptorSetLayout(device, sceneTaaResolveDescriptorSetLayout, nullptr);
        if (sceneDepthSampleSampler != VK_NULL_HANDLE) vkDestroySampler(device, sceneDepthSampleSampler, nullptr);
        if (sceneTaaHistorySampler != VK_NULL_HANDLE) vkDestroySampler(device, sceneTaaHistorySampler, nullptr);
        if (sceneSkyVertexModule != VK_NULL_HANDLE) vkDestroyShaderModule(device, sceneSkyVertexModule, nullptr);
        if (sceneSkyFragmentModule != VK_NULL_HANDLE) vkDestroyShaderModule(device, sceneSkyFragmentModule, nullptr);
        if (sceneMeshVertexModule != VK_NULL_HANDLE) vkDestroyShaderModule(device, sceneMeshVertexModule, nullptr);
        if (sceneMeshFragmentModule != VK_NULL_HANDLE) vkDestroyShaderModule(device, sceneMeshFragmentModule, nullptr);
        if (sceneMeshShadowVertexModule != VK_NULL_HANDLE) vkDestroyShaderModule(device, sceneMeshShadowVertexModule, nullptr);
        if (sceneMeshDepthVertexModule != VK_NULL_HANDLE) vkDestroyShaderModule(device, sceneMeshDepthVertexModule, nullptr);
        if (sceneTonemapVertexModule != VK_NULL_HANDLE) vkDestroyShaderModule(device, sceneTonemapVertexModule, nullptr);
        if (sceneTonemapFragmentModule != VK_NULL_HANDLE) vkDestroyShaderModule(device, sceneTonemapFragmentModule, nullptr);
        if (sceneTaaResolveFragmentModule != VK_NULL_HANDLE) vkDestroyShaderModule(device, sceneTaaResolveFragmentModule, nullptr);
        if (sceneShadowSampler != VK_NULL_HANDLE) vkDestroySampler(device, sceneShadowSampler, nullptr);
        if (sceneShadowRawSampler != VK_NULL_HANDLE) vkDestroySampler(device, sceneShadowRawSampler, nullptr);
        for (std::size_t index = 0; index < sceneUniformBuffers.size(); ++index) {
            if (sceneUniformBuffers[index] != VK_NULL_HANDLE) {
                vmaDestroyBuffer(allocator, sceneUniformBuffers[index], sceneUniformAllocations[index]);
            }
            if (sceneMeshInstanceBuffers[index] != VK_NULL_HANDLE) {
                vmaDestroyBuffer(allocator, sceneMeshInstanceBuffers[index],
                    sceneMeshInstanceAllocations[index]);
            }
            if (sceneLightBuffers[index] != VK_NULL_HANDLE) {
                vmaDestroyBuffer(allocator, sceneLightBuffers[index],
                    sceneLightAllocations[index]);
            }
        }
        sceneSkyPipeline = VK_NULL_HANDLE;
        sceneMeshPipeline = VK_NULL_HANDLE;
        sceneMeshShadowPipeline = VK_NULL_HANDLE;
        sceneMeshDepthPrepassPipeline = VK_NULL_HANDLE;
        sceneTonemapPipeline = VK_NULL_HANDLE;
        sceneTaaResolvePipeline = VK_NULL_HANDLE;
        sceneMeshPipelineLayout = VK_NULL_HANDLE;
        sceneTonemapPipelineLayout = VK_NULL_HANDLE;
        sceneTaaResolvePipelineLayout = VK_NULL_HANDLE;
        materialDescriptorPool = VK_NULL_HANDLE;
        materialDescriptorSetLayout = VK_NULL_HANDLE;
        materialSampler = VK_NULL_HANDLE;
        defaultMaterialTexture = {};
        scenePipelineLayout = VK_NULL_HANDLE;
        sceneDescriptorPool = VK_NULL_HANDLE;
        sceneDescriptorSetLayout = VK_NULL_HANDLE;
        sceneTonemapDescriptorPool = VK_NULL_HANDLE;
        sceneTonemapDescriptorSetLayout = VK_NULL_HANDLE;
        sceneTonemapDescriptorSets = {};
        sceneDirectTonemapDescriptorSets = {};
        sceneTonemapSampler = VK_NULL_HANDLE;
        sceneTaaResolveDescriptorPool = VK_NULL_HANDLE;
        sceneTaaResolveDescriptorSetLayout = VK_NULL_HANDLE;
        sceneTaaResolveDescriptorSets = {};
        sceneDirectTaaResolveDescriptorSets = {};
        sceneDepthSampleSampler = VK_NULL_HANDLE;
        sceneTaaHistorySampler = VK_NULL_HANDLE;
        sceneSkyVertexModule = VK_NULL_HANDLE;
        sceneSkyFragmentModule = VK_NULL_HANDLE;
        sceneMeshVertexModule = VK_NULL_HANDLE;
        sceneMeshFragmentModule = VK_NULL_HANDLE;
        sceneMeshShadowVertexModule = VK_NULL_HANDLE;
        sceneMeshDepthVertexModule = VK_NULL_HANDLE;
        sceneTonemapVertexModule = VK_NULL_HANDLE;
        sceneTonemapFragmentModule = VK_NULL_HANDLE;
        sceneTaaResolveFragmentModule = VK_NULL_HANDLE;
        sceneShadowSampler = VK_NULL_HANDLE;
        sceneShadowRawSampler = VK_NULL_HANDLE;
        sceneUniformBuffers = {};
        sceneUniformAllocations = {};
        sceneUniformMapped = {};
        sceneMeshInstanceBuffers = {};
        sceneMeshInstanceAllocations = {};
        sceneMeshInstanceMapped = {};
        sceneMeshInstanceCapacities = {};
        sceneLightBuffers = {};
        sceneLightAllocations = {};
        sceneLightMapped = {};
        sceneLightCapacities = {};
        sceneDescriptorSets = {};
    }

    void ensureSceneMeshInstanceCapacity(std::size_t required) {
        if (required <= sceneMeshInstanceCapacities[currentFrame]) return;
        // Crescimento geometrico evita realocar durante rajadas de spawn. O
        // fence do slot atual ja foi aguardado em beginFrame(), portanto seu
        // buffer nao esta mais sendo lido pela GPU.
        const std::size_t capacity = std::max<std::size_t>(4096,
            std::bit_ceil(required));
        if (sceneMeshInstanceBuffers[currentFrame] != VK_NULL_HANDLE) {
            vmaDestroyBuffer(allocator,
                sceneMeshInstanceBuffers[currentFrame],
                sceneMeshInstanceAllocations[currentFrame]);
        }
        VkBufferCreateInfo bufferInfo { VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO };
        bufferInfo.size = capacity * sizeof(SceneMeshInstanceGpu);
        bufferInfo.usage = VK_BUFFER_USAGE_VERTEX_BUFFER_BIT;
        bufferInfo.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
        VmaAllocationCreateInfo allocationInfo {};
        allocationInfo.usage = VMA_MEMORY_USAGE_AUTO_PREFER_HOST;
        allocationInfo.flags =
            VMA_ALLOCATION_CREATE_HOST_ACCESS_SEQUENTIAL_WRITE_BIT
            | VMA_ALLOCATION_CREATE_MAPPED_BIT;
        VmaAllocationInfo mappedInfo {};
        check(vmaCreateBuffer(allocator, &bufferInfo, &allocationInfo,
            &sceneMeshInstanceBuffers[currentFrame],
            &sceneMeshInstanceAllocations[currentFrame], &mappedInfo),
            "vmaCreateBuffer(scene mesh instances)");
        sceneMeshInstanceMapped[currentFrame] = mappedInfo.pMappedData;
        sceneMeshInstanceCapacities[currentFrame] = capacity;
    }

    void ensureSceneLightCapacity(std::size_t required) {
        // Nunca aloca zero: o binding 2 do descriptor set precisa apontar
        // pra um buffer valido mesmo com a cena sem nenhuma luz (o shader so
        // deixa de ler o array via lightCount=0, mas o descriptor em si tem
        // que existir).
        const std::size_t minimumRequired = std::max<std::size_t>(1, required);
        if (minimumRequired <= sceneLightCapacities[currentFrame]) return;
        const std::size_t capacity = std::max<std::size_t>(8,
            std::bit_ceil(minimumRequired));
        if (sceneLightBuffers[currentFrame] != VK_NULL_HANDLE) {
            vmaDestroyBuffer(allocator, sceneLightBuffers[currentFrame],
                sceneLightAllocations[currentFrame]);
        }
        VkBufferCreateInfo bufferInfo { VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO };
        bufferInfo.size = capacity * sizeof(GpuLightData3D);
        bufferInfo.usage = VK_BUFFER_USAGE_STORAGE_BUFFER_BIT;
        bufferInfo.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
        VmaAllocationCreateInfo allocationInfo {};
        allocationInfo.usage = VMA_MEMORY_USAGE_AUTO_PREFER_HOST;
        allocationInfo.flags =
            VMA_ALLOCATION_CREATE_HOST_ACCESS_SEQUENTIAL_WRITE_BIT
            | VMA_ALLOCATION_CREATE_MAPPED_BIT;
        VmaAllocationInfo mappedInfo {};
        check(vmaCreateBuffer(allocator, &bufferInfo, &allocationInfo,
            &sceneLightBuffers[currentFrame],
            &sceneLightAllocations[currentFrame], &mappedInfo),
            "vmaCreateBuffer(scene lights)");
        sceneLightMapped[currentFrame] = mappedInfo.pMappedData;
        sceneLightCapacities[currentFrame] = capacity;

        // O buffer trocou de identidade (VkBuffer novo) - o descriptor set
        // precisa ser reescrito, ou continuaria apontando pro buffer antigo
        // ja destruido. Seguro sem esperar a GPU: beginFrame() ja aguardou o
        // fence deste slot antes de renderScene3DInternal chegar aqui.
        VkDescriptorBufferInfo descriptorBuffer {};
        descriptorBuffer.buffer = sceneLightBuffers[currentFrame];
        descriptorBuffer.range = VK_WHOLE_SIZE;
        VkWriteDescriptorSet write { VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET };
        write.dstSet = sceneDescriptorSets[currentFrame];
        write.dstBinding = 2;
        write.descriptorCount = 1;
        write.descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
        write.pBufferInfo = &descriptorBuffer;
        vkUpdateDescriptorSets(device, 1, &write, 0, nullptr);
    }

    void createScene3DResources() {
        VkSamplerCreateInfo samplerInfo { VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO };
        samplerInfo.magFilter = VK_FILTER_LINEAR;
        samplerInfo.minFilter = VK_FILTER_LINEAR;
        samplerInfo.mipmapMode = VK_SAMPLER_MIPMAP_MODE_NEAREST;
        samplerInfo.addressModeU = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_BORDER;
        samplerInfo.addressModeV = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_BORDER;
        samplerInfo.addressModeW = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_BORDER;
        samplerInfo.borderColor = VK_BORDER_COLOR_FLOAT_OPAQUE_WHITE;
        samplerInfo.compareEnable = VK_TRUE;
        samplerInfo.compareOp = VK_COMPARE_OP_LESS_OR_EQUAL;
        samplerInfo.minLod = 0.0f;
        samplerInfo.maxLod = 0.0f;
        check(vkCreateSampler(device, &samplerInfo, nullptr, &sceneShadowSampler),
            "vkCreateSampler(scene shadow)");

        // Mesma imagem, SEM comparacao - PCSS (ver shadowVisibility em
        // scene3d_mesh.frag) precisa ler profundidade crua pra busca de
        // bloqueadores (media de profundidade na vizinhanca), algo que um
        // sampler2DShadow nao expoe (ele so devolve o resultado 0/1 do
        // teste, nunca o valor de profundidade em si).
        VkSamplerCreateInfo rawSamplerInfo { VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO };
        // A busca de bloqueadores do PCSS precisa da profundidade realmente
        // gravada por um caster. LINEAR interpolava caster e fundo, criando
        // bloqueadores inexistentes e penumbras borradas ao redor da silhueta.
        rawSamplerInfo.magFilter = VK_FILTER_NEAREST;
        rawSamplerInfo.minFilter = VK_FILTER_NEAREST;
        rawSamplerInfo.mipmapMode = VK_SAMPLER_MIPMAP_MODE_NEAREST;
        rawSamplerInfo.addressModeU = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_BORDER;
        rawSamplerInfo.addressModeV = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_BORDER;
        rawSamplerInfo.addressModeW = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_BORDER;
        // Borda = profundidade 1.0 (longe, ver Mat4::orthographic - luz usa
        // Z padrao, nao invertido) - fora do mapa conta como "sem
        // bloqueador", mesmo comportamento de borda do sampler de
        // comparacao acima.
        rawSamplerInfo.borderColor = VK_BORDER_COLOR_FLOAT_OPAQUE_WHITE;
        rawSamplerInfo.minLod = 0.0f;
        rawSamplerInfo.maxLod = 0.0f;
        check(vkCreateSampler(device, &rawSamplerInfo, nullptr, &sceneShadowRawSampler),
            "vkCreateSampler(scene shadow raw)");

        std::array<VkDescriptorSetLayoutBinding, 4> bindings {};
        bindings[0].binding = 0;
        bindings[0].descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
        bindings[0].descriptorCount = 1;
        bindings[0].stageFlags = VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT;
        bindings[1].binding = 1;
        bindings[1].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
        // Um mapa por cascata (ver ShadowCascadeCount) - array de
        // sampler2DShadow no shader, nao mais um unico sampler.
        bindings[1].descriptorCount = ShadowCascadeCount;
        bindings[1].stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;
        // Binding 2: SSBO de luzes genericas (ver GpuLightData3D/
        // ensureSceneLightCapacity) - usado pelo ceu (direcao do sol) e pela
        // mesh (loop de iluminacao), so no estagio de fragmento.
        bindings[2].binding = 2;
        bindings[2].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
        bindings[2].descriptorCount = 1;
        bindings[2].stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;
        // Binding 3: as MESMAS N imagens de sombra do binding 1, mas com o
        // sampler sem comparacao (ver sceneShadowRawSampler acima) - so pra
        // a busca de bloqueadores do PCSS.
        bindings[3].binding = 3;
        bindings[3].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
        bindings[3].descriptorCount = ShadowCascadeCount;
        bindings[3].stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;
        VkDescriptorSetLayoutCreateInfo descriptorLayoutInfo {
            VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO
        };
        descriptorLayoutInfo.bindingCount = static_cast<std::uint32_t>(bindings.size());
        descriptorLayoutInfo.pBindings = bindings.data();
        check(vkCreateDescriptorSetLayout(device, &descriptorLayoutInfo, nullptr,
            &sceneDescriptorSetLayout), "vkCreateDescriptorSetLayout(scene)");

        const std::array<VkDescriptorPoolSize, 3> poolSizes { {
            { VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, FramesInFlight },
            // ShadowCascadeCount descritores por set em CADA um dos dois
            // bindings de imagem de sombra (1 = comparacao, 3 = cru pro
            // PCSS, ver bindings[1]/bindings[3] acima) - daí o *2.
            { VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER,
                FramesInFlight * ShadowCascadeCount * 2 },
            { VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, FramesInFlight }
        } };
        VkDescriptorPoolCreateInfo poolInfo { VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO };
        poolInfo.maxSets = FramesInFlight;
        poolInfo.poolSizeCount = static_cast<std::uint32_t>(poolSizes.size());
        poolInfo.pPoolSizes = poolSizes.data();
        check(vkCreateDescriptorPool(device, &poolInfo, nullptr, &sceneDescriptorPool),
            "vkCreateDescriptorPool(scene)");
        const std::array<VkDescriptorSetLayout, FramesInFlight> layouts {
            sceneDescriptorSetLayout, sceneDescriptorSetLayout
        };
        VkDescriptorSetAllocateInfo allocateInfo { VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO };
        allocateInfo.descriptorPool = sceneDescriptorPool;
        allocateInfo.descriptorSetCount = FramesInFlight;
        allocateInfo.pSetLayouts = layouts.data();
        check(vkAllocateDescriptorSets(device, &allocateInfo, sceneDescriptorSets.data()),
            "vkAllocateDescriptorSets(scene)");

        for (std::size_t index = 0; index < sceneUniformBuffers.size(); ++index) {
            VkBufferCreateInfo bufferInfo { VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO };
            bufferInfo.size = sizeof(SceneUniformGpu);
            bufferInfo.usage = VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT;
            bufferInfo.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
            VmaAllocationCreateInfo allocationInfo {};
            allocationInfo.usage = VMA_MEMORY_USAGE_AUTO_PREFER_HOST;
            allocationInfo.flags = VMA_ALLOCATION_CREATE_HOST_ACCESS_SEQUENTIAL_WRITE_BIT
                | VMA_ALLOCATION_CREATE_MAPPED_BIT;
            VmaAllocationInfo mappedInfo {};
            check(vmaCreateBuffer(allocator, &bufferInfo, &allocationInfo,
                &sceneUniformBuffers[index], &sceneUniformAllocations[index], &mappedInfo),
                "vmaCreateBuffer(scene uniform)");
            sceneUniformMapped[index] = mappedInfo.pMappedData;

            VkDescriptorBufferInfo descriptorBuffer {};
            descriptorBuffer.buffer = sceneUniformBuffers[index];
            descriptorBuffer.range = sizeof(SceneUniformGpu);
            VkWriteDescriptorSet write { VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET };
            write.dstSet = sceneDescriptorSets[index];
            write.dstBinding = 0;
            write.descriptorCount = 1;
            write.descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
            write.pBufferInfo = &descriptorBuffer;
            vkUpdateDescriptorSets(device, 1, &write, 0, nullptr);
        }

        const std::string shaderDir = MATTERENGINE_SHADER_DIR;
        const auto createModule = [&](const std::string& filename, VkShaderModule& output) {
            const std::vector<std::uint32_t> spirv = readSpirv(shaderDir + "/" + filename);
            VkShaderModuleCreateInfo moduleInfo { VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO };
            moduleInfo.codeSize = spirv.size() * sizeof(std::uint32_t);
            moduleInfo.pCode = spirv.data();
            check(vkCreateShaderModule(device, &moduleInfo, nullptr, &output),
                "vkCreateShaderModule(scene)");
        };
        createModule("scene3d_sky.vert.spv", sceneSkyVertexModule);
        createModule("scene3d_sky.frag.spv", sceneSkyFragmentModule);
        createModule("scene3d_mesh.vert.spv", sceneMeshVertexModule);
        createModule("scene3d_mesh.frag.spv", sceneMeshFragmentModule);
        createModule("scene3d_mesh_shadow.vert.spv", sceneMeshShadowVertexModule);
        createModule("scene3d_mesh_depth.vert.spv", sceneMeshDepthVertexModule);
        createModule("tonemap.vert.spv", sceneTonemapVertexModule);
        createModule("tonemap.frag.spv", sceneTonemapFragmentModule);
        createModule("taa_resolve.frag.spv", sceneTaaResolveFragmentModule);

        // Dados de instancia real vao por vertex buffer (SceneMeshInstanceGpu),
        // nao push constant - a UNICA excecao e este indice de cascata (4
        // bytes), que so o vertex shader de sombra (scene3d_mesh_shadow.vert)
        // le de fato pra escolher scene.cascadeViewProjections[cascadeIndex].
        // Pipelines deste layout que nao usam push constant (ceu, pre-pass de
        // profundidade da camera) simplesmente nao declaram o bloco
        // correspondente no shader - Vulkan permite um layout declarar uma
        // faixa que nem todo pipeline que o usa consome.
        VkPushConstantRange shadowCascadePushRange {};
        shadowCascadePushRange.stageFlags = VK_SHADER_STAGE_VERTEX_BIT;
        shadowCascadePushRange.offset = 0;
        shadowCascadePushRange.size = sizeof(std::uint32_t);

        VkPipelineLayoutCreateInfo pipelineLayoutInfo { VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO };
        pipelineLayoutInfo.setLayoutCount = 1;
        pipelineLayoutInfo.pSetLayouts = &sceneDescriptorSetLayout;
        pipelineLayoutInfo.pushConstantRangeCount = 1;
        pipelineLayoutInfo.pPushConstantRanges = &shadowCascadePushRange;
        check(vkCreatePipelineLayout(device, &pipelineLayoutInfo, nullptr, &scenePipelineLayout),
            "vkCreatePipelineLayout(scene)");

        // Descriptor set dedicado do passe de tonemap: um unico binding
        // (o alvo HDR de entrada), deliberadamente separado do
        // sceneDescriptorSetLayout (UBO + shadow map) porque o tonemap nao
        // precisa de nenhum dos dois. Dois sets alocados aqui, um para cada
        // alvo HDR (preview offscreen e caminho direto ao swapchain) - so a
        // binding de imagem e escrita depois, quando cada alvo HDR e (re)criado
        // em ensureScene3DTarget/ensureSceneDirectDepth (mesmo padrao ja usado
        // pelo shadow map em ensureSceneShadowTarget).
        VkSamplerCreateInfo tonemapSamplerInfo { VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO };
        tonemapSamplerInfo.magFilter = VK_FILTER_NEAREST;
        tonemapSamplerInfo.minFilter = VK_FILTER_NEAREST;
        tonemapSamplerInfo.mipmapMode = VK_SAMPLER_MIPMAP_MODE_NEAREST;
        tonemapSamplerInfo.addressModeU = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
        tonemapSamplerInfo.addressModeV = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
        tonemapSamplerInfo.addressModeW = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
        tonemapSamplerInfo.maxLod = 0.0f;
        check(vkCreateSampler(device, &tonemapSamplerInfo, nullptr, &sceneTonemapSampler),
            "vkCreateSampler(tonemap)");

        // Sampler de profundidade "cru" (sem compare) pro resolve de TAA
        // reconstruir posicao no mundo - diferente de sceneShadowSampler,
        // que faz PCF via compareEnable, aqui e so uma leitura direta do
        // valor de profundidade armazenado.
        VkSamplerCreateInfo depthSampleSamplerInfo { VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO };
        depthSampleSamplerInfo.magFilter = VK_FILTER_NEAREST;
        depthSampleSamplerInfo.minFilter = VK_FILTER_NEAREST;
        depthSampleSamplerInfo.mipmapMode = VK_SAMPLER_MIPMAP_MODE_NEAREST;
        depthSampleSamplerInfo.addressModeU = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
        depthSampleSamplerInfo.addressModeV = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
        depthSampleSamplerInfo.addressModeW = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
        depthSampleSamplerInfo.maxLod = 0.0f;
        check(vkCreateSampler(device, &depthSampleSamplerInfo, nullptr, &sceneDepthSampleSampler),
            "vkCreateSampler(depth sample)");

        // Sampler LINEAR pro historico do TAA - ver comentario junto do
        // campo sceneTaaHistorySampler.
        VkSamplerCreateInfo taaHistorySamplerInfo { VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO };
        taaHistorySamplerInfo.magFilter = VK_FILTER_LINEAR;
        taaHistorySamplerInfo.minFilter = VK_FILTER_LINEAR;
        taaHistorySamplerInfo.mipmapMode = VK_SAMPLER_MIPMAP_MODE_NEAREST;
        taaHistorySamplerInfo.addressModeU = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
        taaHistorySamplerInfo.addressModeV = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
        taaHistorySamplerInfo.addressModeW = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
        taaHistorySamplerInfo.maxLod = 0.0f;
        check(vkCreateSampler(device, &taaHistorySamplerInfo, nullptr, &sceneTaaHistorySampler),
            "vkCreateSampler(taa history)");

        VkDescriptorSetLayoutBinding tonemapBinding {};
        tonemapBinding.binding = 0;
        tonemapBinding.descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
        tonemapBinding.descriptorCount = 1;
        tonemapBinding.stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;
        VkDescriptorSetLayoutCreateInfo tonemapLayoutInfo {
            VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO
        };
        tonemapLayoutInfo.bindingCount = 1;
        tonemapLayoutInfo.pBindings = &tonemapBinding;
        check(vkCreateDescriptorSetLayout(device, &tonemapLayoutInfo, nullptr,
            &sceneTonemapDescriptorSetLayout), "vkCreateDescriptorSetLayout(tonemap)");

        constexpr std::uint32_t TonemapDescriptorSetCount =
            FramesInFlight * 2;
        const VkDescriptorPoolSize tonemapPoolSize {
            VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, TonemapDescriptorSetCount
        };
        VkDescriptorPoolCreateInfo tonemapPoolInfo { VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO };
        tonemapPoolInfo.maxSets = TonemapDescriptorSetCount;
        tonemapPoolInfo.poolSizeCount = 1;
        tonemapPoolInfo.pPoolSizes = &tonemapPoolSize;
        check(vkCreateDescriptorPool(device, &tonemapPoolInfo, nullptr,
            &sceneTonemapDescriptorPool), "vkCreateDescriptorPool(tonemap)");

        std::array<VkDescriptorSetLayout, TonemapDescriptorSetCount>
            tonemapSetLayouts {};
        tonemapSetLayouts.fill(sceneTonemapDescriptorSetLayout);
        std::array<VkDescriptorSet, TonemapDescriptorSetCount> tonemapDescriptorSets {};
        VkDescriptorSetAllocateInfo tonemapAllocateInfo {
            VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO
        };
        tonemapAllocateInfo.descriptorPool = sceneTonemapDescriptorPool;
        tonemapAllocateInfo.descriptorSetCount = TonemapDescriptorSetCount;
        tonemapAllocateInfo.pSetLayouts = tonemapSetLayouts.data();
        check(vkAllocateDescriptorSets(device, &tonemapAllocateInfo, tonemapDescriptorSets.data()),
            "vkAllocateDescriptorSets(tonemap)");
        for (std::size_t index = 0; index < FramesInFlight; ++index) {
            sceneTonemapDescriptorSets[index] = tonemapDescriptorSets[index];
            sceneDirectTonemapDescriptorSets[index] =
                tonemapDescriptorSets[FramesInFlight + index];
        }

        // Exposicao/brilho/contraste/saturacao chegam por push constant (nao
        // pelo SceneUniform) - o tonemap e o unico consumidor e nao ha
        // motivo para inflar o UBO espelhado em 7 arquivos de shader por
        // isso.
        VkPushConstantRange tonemapPushConstant {};
        tonemapPushConstant.stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;
        tonemapPushConstant.offset = 0;
        tonemapPushConstant.size = sizeof(TonemapPushConstantsGpu);
        VkPipelineLayoutCreateInfo tonemapPipelineLayoutInfo {
            VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO
        };
        tonemapPipelineLayoutInfo.setLayoutCount = 1;
        tonemapPipelineLayoutInfo.pSetLayouts = &sceneTonemapDescriptorSetLayout;
        tonemapPipelineLayoutInfo.pushConstantRangeCount = 1;
        tonemapPipelineLayoutInfo.pPushConstantRanges = &tonemapPushConstant;
        check(vkCreatePipelineLayout(device, &tonemapPipelineLayoutInfo, nullptr,
            &sceneTonemapPipelineLayout), "vkCreatePipelineLayout(tonemap)");

        // Descriptor set do resolve de TAA: 4 bindings, todas amostradas so
        // no fragmento - cor HDR atual, profundidade, vetores de movimento,
        // historico (ver comentario em sceneTaaResolveDescriptorSetLayout).
        // Um set por frame em voo e por caminho. Todos os bindings sao
        // escritos apenas quando os attachments sao criados/recriados com o
        // device ocioso; nenhum descriptor em uso e mutado durante o frame.
        std::array<VkDescriptorSetLayoutBinding, 4> taaResolveBindings {};
        for (std::uint32_t index = 0; index < taaResolveBindings.size(); ++index) {
            taaResolveBindings[index].binding = index;
            taaResolveBindings[index].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
            taaResolveBindings[index].descriptorCount = 1;
            taaResolveBindings[index].stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;
        }
        VkDescriptorSetLayoutCreateInfo taaResolveLayoutInfo {
            VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO
        };
        taaResolveLayoutInfo.bindingCount = static_cast<std::uint32_t>(taaResolveBindings.size());
        taaResolveLayoutInfo.pBindings = taaResolveBindings.data();
        check(vkCreateDescriptorSetLayout(device, &taaResolveLayoutInfo, nullptr,
            &sceneTaaResolveDescriptorSetLayout), "vkCreateDescriptorSetLayout(taa resolve)");

        constexpr std::uint32_t TaaResolveDescriptorSetCount =
            FramesInFlight * 2;
        const VkDescriptorPoolSize taaResolvePoolSize {
            VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER,
            TaaResolveDescriptorSetCount * static_cast<std::uint32_t>(taaResolveBindings.size())
        };
        VkDescriptorPoolCreateInfo taaResolvePoolInfo { VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO };
        taaResolvePoolInfo.maxSets = TaaResolveDescriptorSetCount;
        taaResolvePoolInfo.poolSizeCount = 1;
        taaResolvePoolInfo.pPoolSizes = &taaResolvePoolSize;
        check(vkCreateDescriptorPool(device, &taaResolvePoolInfo, nullptr,
            &sceneTaaResolveDescriptorPool), "vkCreateDescriptorPool(taa resolve)");

        std::array<VkDescriptorSetLayout, TaaResolveDescriptorSetCount>
            taaResolveSetLayouts {};
        taaResolveSetLayouts.fill(sceneTaaResolveDescriptorSetLayout);
        std::array<VkDescriptorSet, TaaResolveDescriptorSetCount> taaResolveDescriptorSets {};
        VkDescriptorSetAllocateInfo taaResolveAllocateInfo {
            VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO
        };
        taaResolveAllocateInfo.descriptorPool = sceneTaaResolveDescriptorPool;
        taaResolveAllocateInfo.descriptorSetCount = TaaResolveDescriptorSetCount;
        taaResolveAllocateInfo.pSetLayouts = taaResolveSetLayouts.data();
        check(vkAllocateDescriptorSets(device, &taaResolveAllocateInfo, taaResolveDescriptorSets.data()),
            "vkAllocateDescriptorSets(taa resolve)");
        for (std::size_t index = 0; index < FramesInFlight; ++index) {
            sceneTaaResolveDescriptorSets[index] =
                taaResolveDescriptorSets[index];
            sceneDirectTaaResolveDescriptorSets[index] =
                taaResolveDescriptorSets[FramesInFlight + index];
        }

        // Layout do resolve: set 0 = UBO da cena (cameraViewProjection
        // jitterada/inversa/previousCameraViewProjection), set 1 = as 3
        // texturas acima. Sem push constant - o peso do historico (90%) e
        // uma constante no proprio shader (ver taa_resolve.frag).
        const std::array<VkDescriptorSetLayout, 2> taaResolveSetLayoutsForPipeline {
            sceneDescriptorSetLayout, sceneTaaResolveDescriptorSetLayout
        };
        VkPipelineLayoutCreateInfo taaResolvePipelineLayoutInfo {
            VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO
        };
        taaResolvePipelineLayoutInfo.setLayoutCount =
            static_cast<std::uint32_t>(taaResolveSetLayoutsForPipeline.size());
        taaResolvePipelineLayoutInfo.pSetLayouts = taaResolveSetLayoutsForPipeline.data();
        check(vkCreatePipelineLayout(device, &taaResolvePipelineLayoutInfo, nullptr,
            &sceneTaaResolvePipelineLayout), "vkCreatePipelineLayout(taa resolve)");

        VkPipelineVertexInputStateCreateInfo vertexInput { VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO };
        VkPipelineInputAssemblyStateCreateInfo inputAssembly { VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO };
        inputAssembly.topology = VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST;
        VkPipelineViewportStateCreateInfo viewport { VK_STRUCTURE_TYPE_PIPELINE_VIEWPORT_STATE_CREATE_INFO };
        viewport.viewportCount = 1;
        viewport.scissorCount = 1;
        VkPipelineMultisampleStateCreateInfo multisample { VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO };
        multisample.rasterizationSamples = VK_SAMPLE_COUNT_1_BIT;
        VkPipelineDepthStencilStateCreateInfo depth { VK_STRUCTURE_TYPE_PIPELINE_DEPTH_STENCIL_STATE_CREATE_INFO };
        depth.depthTestEnable = VK_TRUE;
        depth.depthWriteEnable = VK_TRUE;
        depth.depthCompareOp = VK_COMPARE_OP_LESS_OR_EQUAL;
        const std::array<VkDynamicState, 2> dynamicStates {
            VK_DYNAMIC_STATE_VIEWPORT, VK_DYNAMIC_STATE_SCISSOR
        };
        VkPipelineDynamicStateCreateInfo dynamic { VK_STRUCTURE_TYPE_PIPELINE_DYNAMIC_STATE_CREATE_INFO };
        dynamic.dynamicStateCount = static_cast<std::uint32_t>(dynamicStates.size());
        dynamic.pDynamicStates = dynamicStates.data();

        // Modelo de pipeline depth-only (sem attachment de cor): hoje serve
        // so a sombra de mesh, mas fica separado do modelo de cor abaixo
        // porque os dois tem bias/attachments diferentes por natureza (uma
        // sombra precisa de depth bias para evitar acne; a cena principal
        // nao).
        VkPipelineRasterizationStateCreateInfo shadowRasterizer { VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO };
        shadowRasterizer.polygonMode = VK_POLYGON_MODE_FILL;
        shadowRasterizer.cullMode = VK_CULL_MODE_NONE;
        shadowRasterizer.frontFace = VK_FRONT_FACE_COUNTER_CLOCKWISE;
        shadowRasterizer.lineWidth = 1.0f;
        shadowRasterizer.depthBiasEnable = VK_TRUE;
        // Primeiro estagio contra acne, aplicado durante a rasterizacao. O
        // shader complementa com bias em metros proporcional ao texel; estes
        // valores deliberadamente moderados evitam separar sombra e objeto.
        shadowRasterizer.depthBiasConstantFactor = 0.75f;
        shadowRasterizer.depthBiasSlopeFactor = 1.25f;
        VkPipelineColorBlendStateCreateInfo noColorBlend { VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO };
        VkPipelineRenderingCreateInfo shadowRendering { VK_STRUCTURE_TYPE_PIPELINE_RENDERING_CREATE_INFO };
        shadowRendering.depthAttachmentFormat = SceneDepthFormat;
        // stageCount/pStages ficam em zero aqui de proposito: este struct e
        // so um modelo, nunca cria um pipeline por si so - meshShadowPipelineInfo
        // (mais abaixo) sempre sobrescreve os dois antes de usar.
        VkGraphicsPipelineCreateInfo shadowPipelineInfo { VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO };
        shadowPipelineInfo.pNext = &shadowRendering;
        shadowPipelineInfo.pVertexInputState = &vertexInput;
        shadowPipelineInfo.pInputAssemblyState = &inputAssembly;
        shadowPipelineInfo.pViewportState = &viewport;
        shadowPipelineInfo.pRasterizationState = &shadowRasterizer;
        shadowPipelineInfo.pMultisampleState = &multisample;
        shadowPipelineInfo.pDepthStencilState = &depth;
        shadowPipelineInfo.pColorBlendState = &noColorBlend;
        shadowPipelineInfo.pDynamicState = &dynamic;
        shadowPipelineInfo.layout = scenePipelineLayout;

        // Modelo de pipeline de cor (com depth): compartilhado pelo ceu e
        // pela mesh principal, nenhum dos dois cria um pipeline a partir
        // deste struct diretamente - cada um sobrescreve estagios/stages
        // antes de usar. stageCount/pStages tambem ficam em zero aqui pelo
        // mesmo motivo do modelo depth-only acima.
        VkPipelineRasterizationStateCreateInfo sceneRasterizer { VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO };
        sceneRasterizer.polygonMode = VK_POLYGON_MODE_FILL;
        sceneRasterizer.cullMode = VK_CULL_MODE_NONE;
        sceneRasterizer.frontFace = VK_FRONT_FACE_COUNTER_CLOCKWISE;
        sceneRasterizer.lineWidth = 1.0f;
        // 2 attachments (MRT): cor HDR + vetores de movimento (ver
        // scene3d_mesh.vert/frag) - toda pipeline que desenha neste passe
        // (ceu e mesh) escreve nos dois, mesmo o ceu so preenchendo o
        // segundo com zero (ver scene3d_sky.frag).
        VkPipelineColorBlendAttachmentState blendAttachment {};
        blendAttachment.colorWriteMask = VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT
            | VK_COLOR_COMPONENT_B_BIT | VK_COLOR_COMPONENT_A_BIT;
        VkPipelineColorBlendAttachmentState motionVectorBlendAttachment {};
        motionVectorBlendAttachment.colorWriteMask = VK_COLOR_COMPONENT_R_BIT
            | VK_COLOR_COMPONENT_G_BIT;
        const std::array<VkPipelineColorBlendAttachmentState, 2> sceneBlendAttachments {
            blendAttachment, motionVectorBlendAttachment
        };
        VkPipelineColorBlendStateCreateInfo colorBlend { VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO };
        colorBlend.attachmentCount =
            static_cast<std::uint32_t>(sceneBlendAttachments.size());
        colorBlend.pAttachments = sceneBlendAttachments.data();
        // SceneHdrColorFormat, nao swapchainFormat: o ceu e a mesh escrevem
        // no alvo HDR intermediario (ver renderScene3DInternal), nunca
        // direto na imagem final - o formato declarado aqui precisa bater
        // exatamente com o attachment de verdade usado em vkCmdBeginRendering
        // (bug pre-existente descoberto e corrigido junto desta mudanca:
        // o formato declarado antes era o do swapchain, nunca o realmente
        // usado por este pipeline).
        const std::array<VkFormat, 2> sceneColorFormats {
            SceneHdrColorFormat, SceneMotionVectorFormat
        };
        VkPipelineRenderingCreateInfo sceneRendering { VK_STRUCTURE_TYPE_PIPELINE_RENDERING_CREATE_INFO };
        sceneRendering.colorAttachmentCount =
            static_cast<std::uint32_t>(sceneColorFormats.size());
        sceneRendering.pColorAttachmentFormats = sceneColorFormats.data();
        sceneRendering.depthAttachmentFormat = SceneDepthFormat;
        VkGraphicsPipelineCreateInfo scenePipelineInfo = shadowPipelineInfo;
        scenePipelineInfo.pNext = &sceneRendering;
        scenePipelineInfo.pRasterizationState = &sceneRasterizer;
        scenePipelineInfo.pColorBlendState = &colorBlend;

        std::array<VkPipelineShaderStageCreateInfo, 2> skyStages {};
        skyStages[0].sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
        skyStages[0].stage = VK_SHADER_STAGE_VERTEX_BIT;
        skyStages[0].pName = "main";
        skyStages[1].sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
        skyStages[1].stage = VK_SHADER_STAGE_FRAGMENT_BIT;
        skyStages[1].pName = "main";
        skyStages[0].module = sceneSkyVertexModule;
        skyStages[1].module = sceneSkyFragmentModule;
        VkPipelineDepthStencilStateCreateInfo skyDepth {
            VK_STRUCTURE_TYPE_PIPELINE_DEPTH_STENCIL_STATE_CREATE_INFO
        };
        skyDepth.depthTestEnable = VK_FALSE;
        skyDepth.depthWriteEnable = VK_FALSE;
        VkGraphicsPipelineCreateInfo skyPipelineInfo = scenePipelineInfo;
        skyPipelineInfo.stageCount = static_cast<std::uint32_t>(skyStages.size());
        skyPipelineInfo.pStages = skyStages.data();
        skyPipelineInfo.pDepthStencilState = &skyDepth;
        check(vkCreateGraphicsPipelines(device, VK_NULL_HANDLE, 1, &skyPipelineInfo,
            nullptr, &sceneSkyPipeline), "vkCreateGraphicsPipelines(scene sky)");

        // Pipeline de tonemap: mesmo triangulo cheio de tela do ceu, mas sem
        // attachment de depth algum (nao e so depthTestEnable=false com um
        // depth attachment presente como o ceu faz - aqui o proprio passe
        // nao declara depth attachment, ver renderScene3DInternal), porque
        // este e o passe que fica aberto para o ImGui desenhar em cima no
        // caminho direto ao swapchain, e o pipeline do proprio ImGui (ver
        // initialize()) tambem nao declara nenhum formato de depth - as duas
        // declaracoes precisam bater exatamente.
        VkPipelineRenderingCreateInfo tonemapRendering { VK_STRUCTURE_TYPE_PIPELINE_RENDERING_CREATE_INFO };
        tonemapRendering.colorAttachmentCount = 1;
        tonemapRendering.pColorAttachmentFormats = &swapchainFormat;
        // 1 attachment so, ao contrario de colorBlend (2, ver
        // sceneBlendAttachments acima) - tonemap e o resolve de TAA (mais
        // abaixo) precisam da propria VkPipelineColorBlendStateCreateInfo
        // em vez de herdar a de scenePipelineInfo, senao attachmentCount
        // (2) divergiria do colorAttachmentCount (1) de cada um deles,
        // configuracao invalida no Vulkan.
        VkPipelineColorBlendStateCreateInfo singleColorBlend {
            VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO
        };
        singleColorBlend.attachmentCount = 1;
        singleColorBlend.pAttachments = &blendAttachment;
        VkPipelineDepthStencilStateCreateInfo tonemapDepth {
            VK_STRUCTURE_TYPE_PIPELINE_DEPTH_STENCIL_STATE_CREATE_INFO
        };
        tonemapDepth.depthTestEnable = VK_FALSE;
        tonemapDepth.depthWriteEnable = VK_FALSE;
        std::array<VkPipelineShaderStageCreateInfo, 2> tonemapStages {};
        tonemapStages[0].sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
        tonemapStages[0].stage = VK_SHADER_STAGE_VERTEX_BIT;
        tonemapStages[0].module = sceneTonemapVertexModule;
        tonemapStages[0].pName = "main";
        tonemapStages[1].sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
        tonemapStages[1].stage = VK_SHADER_STAGE_FRAGMENT_BIT;
        tonemapStages[1].module = sceneTonemapFragmentModule;
        tonemapStages[1].pName = "main";
        VkGraphicsPipelineCreateInfo tonemapPipelineInfo = scenePipelineInfo;
        tonemapPipelineInfo.pNext = &tonemapRendering;
        tonemapPipelineInfo.pDepthStencilState = &tonemapDepth;
        tonemapPipelineInfo.pColorBlendState = &singleColorBlend;
        tonemapPipelineInfo.stageCount = static_cast<std::uint32_t>(tonemapStages.size());
        tonemapPipelineInfo.pStages = tonemapStages.data();
        tonemapPipelineInfo.layout = sceneTonemapPipelineLayout;
        check(vkCreateGraphicsPipelines(device, VK_NULL_HANDLE, 1, &tonemapPipelineInfo,
            nullptr, &sceneTonemapPipeline), "vkCreateGraphicsPipelines(scene tonemap)");

        // Pipeline de resolve de TAA: mesmo triangulo cheio de tela
        // (reaproveita sceneTonemapVertexModule), sem depth attachment como
        // o tonemap, mas escrevendo no FORMATO HDR (SceneHdrColorFormat) do
        // historico, nao no formato final LDR do swapchain/preview.
        VkPipelineRenderingCreateInfo taaResolveRendering { VK_STRUCTURE_TYPE_PIPELINE_RENDERING_CREATE_INFO };
        taaResolveRendering.colorAttachmentCount = 1;
        taaResolveRendering.pColorAttachmentFormats = &SceneHdrColorFormat;
        VkPipelineDepthStencilStateCreateInfo taaResolveDepth {
            VK_STRUCTURE_TYPE_PIPELINE_DEPTH_STENCIL_STATE_CREATE_INFO
        };
        taaResolveDepth.depthTestEnable = VK_FALSE;
        taaResolveDepth.depthWriteEnable = VK_FALSE;
        std::array<VkPipelineShaderStageCreateInfo, 2> taaResolveStages {};
        taaResolveStages[0].sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
        taaResolveStages[0].stage = VK_SHADER_STAGE_VERTEX_BIT;
        taaResolveStages[0].module = sceneTonemapVertexModule;
        taaResolveStages[0].pName = "main";
        taaResolveStages[1].sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
        taaResolveStages[1].stage = VK_SHADER_STAGE_FRAGMENT_BIT;
        taaResolveStages[1].module = sceneTaaResolveFragmentModule;
        taaResolveStages[1].pName = "main";
        VkGraphicsPipelineCreateInfo taaResolvePipelineInfo = scenePipelineInfo;
        taaResolvePipelineInfo.pNext = &taaResolveRendering;
        taaResolvePipelineInfo.pDepthStencilState = &taaResolveDepth;
        taaResolvePipelineInfo.pColorBlendState = &singleColorBlend;
        taaResolvePipelineInfo.stageCount = static_cast<std::uint32_t>(taaResolveStages.size());
        taaResolvePipelineInfo.pStages = taaResolveStages.data();
        taaResolvePipelineInfo.layout = sceneTaaResolvePipelineLayout;
        check(vkCreateGraphicsPipelines(device, VK_NULL_HANDLE, 1, &taaResolvePipelineInfo,
            nullptr, &sceneTaaResolvePipeline), "vkCreateGraphicsPipelines(scene taa resolve)");

        // Material sampler + descriptor set (set=1) sampled by the mesh
        // fragment shader for its albedo map. Each texture gets its own
        // descriptor set allocated on demand in createTexture2D rather than
        // a bindless array - the object counts this engine deals with today
        // (a prop library, not thousands of unique materials per frame)
        // don't justify that complexity yet (see ROADMAP.md Marco 5).
        // Trilinear + anisotropico, e maxLod liberado pra cadeia inteira -
        // createTexture2D agora gera mipmaps de verdade (ver o loop de blit
        // la dentro), entao o sampler precisa efetivamente poder usa-los.
        // Antes desta mudanca (mipmapMode=NEAREST + maxLod=0.0, travando a
        // amostragem sempre no nivel de resolucao total), texturas
        // minificadas na tela - o chao do laboratorio visto de longe/em
        // angulo raso e o exemplo que motivou essa mudanca - viravam ruido
        // de alta frequencia por falta de filtragem, nao por falta de
        // antisserrilhado geometrico.
        VkPhysicalDeviceProperties deviceProperties {};
        vkGetPhysicalDeviceProperties(physicalDevice, &deviceProperties);

        VkSamplerCreateInfo materialSamplerInfo { VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO };
        materialSamplerInfo.magFilter = VK_FILTER_LINEAR;
        materialSamplerInfo.minFilter = VK_FILTER_LINEAR;
        materialSamplerInfo.mipmapMode = VK_SAMPLER_MIPMAP_MODE_LINEAR;
        materialSamplerInfo.addressModeU = VK_SAMPLER_ADDRESS_MODE_REPEAT;
        materialSamplerInfo.addressModeV = VK_SAMPLER_ADDRESS_MODE_REPEAT;
        materialSamplerInfo.addressModeW = VK_SAMPLER_ADDRESS_MODE_REPEAT;
        materialSamplerInfo.anisotropyEnable = VK_TRUE;
        materialSamplerInfo.maxAnisotropy = deviceProperties.limits.maxSamplerAnisotropy;
        materialSamplerInfo.maxLod = VK_LOD_CLAMP_NONE;
        check(vkCreateSampler(device, &materialSamplerInfo, nullptr, &materialSampler),
            "vkCreateSampler(material)");

        VkDescriptorSetLayoutBinding materialBinding {};
        materialBinding.binding = 0;
        materialBinding.descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
        materialBinding.descriptorCount = 1;
        materialBinding.stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;
        VkDescriptorSetLayoutCreateInfo materialLayoutInfo {
            VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO
        };
        materialLayoutInfo.bindingCount = 1;
        materialLayoutInfo.pBindings = &materialBinding;
        check(vkCreateDescriptorSetLayout(device, &materialLayoutInfo, nullptr,
            &materialDescriptorSetLayout), "vkCreateDescriptorSetLayout(material)");

        constexpr std::uint32_t MaxMaterialTextures = 256;
        const VkDescriptorPoolSize materialPoolSize {
            VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, MaxMaterialTextures
        };
        VkDescriptorPoolCreateInfo materialPoolInfo {
            VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO
        };
        materialPoolInfo.maxSets = MaxMaterialTextures;
        materialPoolInfo.poolSizeCount = 1;
        materialPoolInfo.pPoolSizes = &materialPoolSize;
        check(vkCreateDescriptorPool(device, &materialPoolInfo, nullptr,
            &materialDescriptorPool), "vkCreateDescriptorPool(material)");

        // Nenhum push constant aqui: dados por instancia (posicao/orientacao/
        // metallic/roughness/flags) ja chegam pelo vertex buffer de instancia
        // (binding 1, ver meshBinding abaixo).
        const std::array<VkDescriptorSetLayout, 3> meshSetLayouts {
            sceneDescriptorSetLayout, materialDescriptorSetLayout,
            materialDescriptorSetLayout
        };
        VkPipelineLayoutCreateInfo meshPipelineLayoutInfo {
            VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO
        };
        meshPipelineLayoutInfo.setLayoutCount =
            static_cast<std::uint32_t>(meshSetLayouts.size());
        meshPipelineLayoutInfo.pSetLayouts = meshSetLayouts.data();
        check(vkCreatePipelineLayout(device, &meshPipelineLayoutInfo, nullptr,
            &sceneMeshPipelineLayout), "vkCreatePipelineLayout(scene mesh)");

        // Real mesh vertex input: position/normal/uv/color, interleaved.
        // Both the shadow and main mesh pipelines bind the same
        // buffer/stride - the shadow vertex shader simply only reads
        // location 0.
        std::array<VkVertexInputBindingDescription, 2> meshBinding {};
        meshBinding[0].binding = 0;
        meshBinding[0].stride = sizeof(float) * 11;
        meshBinding[0].inputRate = VK_VERTEX_INPUT_RATE_VERTEX;
        meshBinding[1].binding = 1;
        meshBinding[1].stride = sizeof(SceneMeshInstanceGpu);
        meshBinding[1].inputRate = VK_VERTEX_INPUT_RATE_INSTANCE;
        // Localizacoes 9-12 (transformacao do quadro anterior) so sao lidas
        // pelo vertex shader do passe de COR (scene3d_mesh.vert, pro vetor
        // de movimento do TAA) - o pre-pass de profundidade compartilha este
        // mesmo layout de vertice (meshDepthPrepassPipelineInfo abaixo) mas
        // seu shader simplesmente nao declara essas localizacoes, o que e
        // valido no Vulkan (nem todo atributo descrito precisa ser
        // consumido por todo shader que usa o mesmo layout).
        std::array<VkVertexInputAttributeDescription, 13> meshAttributes { {
            { 0, 0, VK_FORMAT_R32G32B32_SFLOAT, 0 },
            { 1, 0, VK_FORMAT_R32G32B32_SFLOAT, sizeof(float) * 3 },
            { 2, 0, VK_FORMAT_R32G32_SFLOAT, sizeof(float) * 6 },
            { 3, 0, VK_FORMAT_R32G32B32_SFLOAT, sizeof(float) * 8 },
            { 4, 1, VK_FORMAT_R32G32B32A32_SFLOAT,
                offsetof(SceneMeshInstanceGpu, positionScale) },
            { 5, 1, VK_FORMAT_R32G32B32A32_SFLOAT,
                offsetof(SceneMeshInstanceGpu, orientationX) },
            { 6, 1, VK_FORMAT_R32G32B32A32_SFLOAT,
                offsetof(SceneMeshInstanceGpu, orientationY) },
            { 7, 1, VK_FORMAT_R32G32B32A32_SFLOAT,
                offsetof(SceneMeshInstanceGpu, orientationZ) },
            { 8, 1, VK_FORMAT_R32G32B32A32_SFLOAT,
                offsetof(SceneMeshInstanceGpu, materialAndFlags) },
            { 9, 1, VK_FORMAT_R32G32B32A32_SFLOAT,
                offsetof(SceneMeshInstanceGpu, previousPositionScale) },
            { 10, 1, VK_FORMAT_R32G32B32A32_SFLOAT,
                offsetof(SceneMeshInstanceGpu, previousOrientationX) },
            { 11, 1, VK_FORMAT_R32G32B32A32_SFLOAT,
                offsetof(SceneMeshInstanceGpu, previousOrientationY) },
            { 12, 1, VK_FORMAT_R32G32B32A32_SFLOAT,
                offsetof(SceneMeshInstanceGpu, previousOrientationZ) }
        } };
        VkPipelineVertexInputStateCreateInfo meshVertexInput {
            VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO
        };
        meshVertexInput.vertexBindingDescriptionCount =
            static_cast<std::uint32_t>(meshBinding.size());
        meshVertexInput.pVertexBindingDescriptions = meshBinding.data();
        meshVertexInput.vertexAttributeDescriptionCount =
            static_cast<std::uint32_t>(meshAttributes.size());
        meshVertexInput.pVertexAttributeDescriptions = meshAttributes.data();

        VkPipelineShaderStageCreateInfo meshVertexStage {
            VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO
        };
        meshVertexStage.stage = VK_SHADER_STAGE_VERTEX_BIT;
        meshVertexStage.module = sceneMeshVertexModule;
        meshVertexStage.pName = "main";
        VkPipelineShaderStageCreateInfo meshFragmentStage {
            VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO
        };
        meshFragmentStage.stage = VK_SHADER_STAGE_FRAGMENT_BIT;
        meshFragmentStage.module = sceneMeshFragmentModule;
        meshFragmentStage.pName = "main";
        const std::array<VkPipelineShaderStageCreateInfo, 2> meshStages {
            meshVertexStage, meshFragmentStage
        };
        // O passe de cor agora roda depois do pre-pass de profundidade
        // (sceneMeshDepthPrepassPipeline, logo abaixo) - so testa (EQUAL)
        // sem escrever de novo, pra aproveitar o early-Z ja resolvido pelo
        // pre-pass e nao pagar o fragment shader completo (BRDF + shadow
        // lookup) em fragmentos que vao ser sobrescritos por outro objeto
        // mais proximo. Seguro porque os dois passes usam exatamente a
        // mesma transformacao de vertice (ver scene3d_mesh_depth.vert),
        // produzindo gl_Position.z identico bit a bit.
        VkPipelineDepthStencilStateCreateInfo meshColorDepth {
            VK_STRUCTURE_TYPE_PIPELINE_DEPTH_STENCIL_STATE_CREATE_INFO
        };
        meshColorDepth.depthTestEnable = VK_TRUE;
        meshColorDepth.depthWriteEnable = VK_FALSE;
        meshColorDepth.depthCompareOp = VK_COMPARE_OP_EQUAL;
        VkGraphicsPipelineCreateInfo meshPipelineInfo = scenePipelineInfo;
        meshPipelineInfo.pVertexInputState = &meshVertexInput;
        meshPipelineInfo.pDepthStencilState = &meshColorDepth;
        meshPipelineInfo.stageCount = static_cast<std::uint32_t>(meshStages.size());
        meshPipelineInfo.pStages = meshStages.data();
        meshPipelineInfo.layout = sceneMeshPipelineLayout;
        check(vkCreateGraphicsPipelines(device, VK_NULL_HANDLE, 1, &meshPipelineInfo,
            nullptr, &sceneMeshPipeline), "vkCreateGraphicsPipelines(scene mesh)");

        // Depth-only shadow variant: no color attachment, so the pipeline
        // can legally omit the fragment stage entirely. It never samples a
        // material texture, so it keeps the plain (set=0 only) scene layout.
        VkPipelineShaderStageCreateInfo meshShadowVertexStage {
            VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO
        };
        meshShadowVertexStage.stage = VK_SHADER_STAGE_VERTEX_BIT;
        meshShadowVertexStage.module = sceneMeshShadowVertexModule;
        meshShadowVertexStage.pName = "main";
        VkGraphicsPipelineCreateInfo meshShadowPipelineInfo = shadowPipelineInfo;
        meshShadowPipelineInfo.pVertexInputState = &meshVertexInput;
        meshShadowPipelineInfo.stageCount = 1;
        meshShadowPipelineInfo.pStages = &meshShadowVertexStage;
        check(vkCreateGraphicsPipelines(device, VK_NULL_HANDLE, 1, &meshShadowPipelineInfo,
            nullptr, &sceneMeshShadowPipeline), "vkCreateGraphicsPipelines(scene mesh shadow)");

        // Pre-pass de profundidade da cena principal (nao da sombra): mesma
        // ideia do shadow acima (so vertice, sem estagio de fragmento,
        // layout compartilhado scenePipelineLayout), mas usando
        // cameraViewProjection em vez de lightViewProjection, e sem bias de
        // profundidade (shadowRasterizer tem bias so pra evitar acne de
        // sombra - usar esse bias aqui deslocaria a profundidade e quebraria
        // o teste EQUAL do passe de cor acima). Roda ANTES do passe de cor
        // em renderScene3DInternal.
        VkPipelineShaderStageCreateInfo meshDepthPrepassVertexStage {
            VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO
        };
        meshDepthPrepassVertexStage.stage = VK_SHADER_STAGE_VERTEX_BIT;
        meshDepthPrepassVertexStage.module = sceneMeshDepthVertexModule;
        meshDepthPrepassVertexStage.pName = "main";
        // GREATER_OR_EQUAL, nao o LESS_OR_EQUAL de "depth" (struct
        // compartilhada com o pipeline de sombra, que usa a camera da luz -
        // essa continua em profundidade padrao). A camera principal usa
        // Mat4::perspective(), que agora produz profundidade INVERTIDA
        // (perto=1, longe=0, ver Mat4.cpp) - por isso este pre-pass precisa
        // do proprio struct de depth-stencil em vez de herdar o de
        // shadowPipelineInfo.
        VkPipelineDepthStencilStateCreateInfo depthPrepassDepthStencil {
            VK_STRUCTURE_TYPE_PIPELINE_DEPTH_STENCIL_STATE_CREATE_INFO
        };
        depthPrepassDepthStencil.depthTestEnable = VK_TRUE;
        depthPrepassDepthStencil.depthWriteEnable = VK_TRUE;
        depthPrepassDepthStencil.depthCompareOp = VK_COMPARE_OP_GREATER_OR_EQUAL;
        VkGraphicsPipelineCreateInfo meshDepthPrepassPipelineInfo = shadowPipelineInfo;
        meshDepthPrepassPipelineInfo.pRasterizationState = &sceneRasterizer;
        meshDepthPrepassPipelineInfo.pVertexInputState = &meshVertexInput;
        meshDepthPrepassPipelineInfo.pDepthStencilState = &depthPrepassDepthStencil;
        meshDepthPrepassPipelineInfo.stageCount = 1;
        meshDepthPrepassPipelineInfo.pStages = &meshDepthPrepassVertexStage;
        check(vkCreateGraphicsPipelines(device, VK_NULL_HANDLE, 1, &meshDepthPrepassPipelineInfo,
            nullptr, &sceneMeshDepthPrepassPipeline),
            "vkCreateGraphicsPipelines(scene mesh depth prepass)");

        const std::array<std::byte, 4> whitePixel {
            std::byte { 255 }, std::byte { 255 }, std::byte { 255 }, std::byte { 255 }
        };
        defaultMaterialTexture = createTexture2D({ 1, 1 }, whitePixel);
    }

    void createSceneAttachment(VkExtent2D imageExtent, VkFormat format,
        VkImageUsageFlags usage, VkImageAspectFlags aspect,
        VkImage& image, VmaAllocation& allocation, VkImageView& view) {
        VkImageCreateInfo imageInfo { VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO };
        imageInfo.imageType = VK_IMAGE_TYPE_2D;
        imageInfo.format = format;
        imageInfo.extent = { imageExtent.width, imageExtent.height, 1 };
        imageInfo.mipLevels = 1;
        imageInfo.arrayLayers = 1;
        imageInfo.samples = VK_SAMPLE_COUNT_1_BIT;
        imageInfo.tiling = VK_IMAGE_TILING_OPTIMAL;
        imageInfo.usage = usage;
        imageInfo.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
        imageInfo.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
        VmaAllocationCreateInfo allocationInfo {};
        allocationInfo.usage = VMA_MEMORY_USAGE_AUTO_PREFER_DEVICE;
        check(vmaCreateImage(allocator, &imageInfo, &allocationInfo,
            &image, &allocation, nullptr), "vmaCreateImage(scene attachment)");
        VkImageViewCreateInfo viewInfo { VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO };
        viewInfo.image = image;
        viewInfo.viewType = VK_IMAGE_VIEW_TYPE_2D;
        viewInfo.format = format;
        viewInfo.subresourceRange.aspectMask = aspect;
        viewInfo.subresourceRange.levelCount = 1;
        viewInfo.subresourceRange.layerCount = 1;
        check(vkCreateImageView(device, &viewInfo, nullptr, &view),
            "vkCreateImageView(scene attachment)");
    }

    void ensureSceneShadowTarget() {
        if (sceneShadowImages[0] != VK_NULL_HANDLE) {
            return;
        }
        const VkExtent2D shadowExtent { SceneShadowMapSize, SceneShadowMapSize };
        std::array<VkDescriptorImageInfo, ShadowCascadeCount> shadowInfos {};
        // Mesmas N imagens do array acima, mas com o sampler SEM comparacao
        // (binding 3, ver bindings[3] em createScene3DResources) - so pra
        // busca de bloqueadores do PCSS (ver shadowVisibility em
        // scene3d_mesh.frag).
        std::array<VkDescriptorImageInfo, ShadowCascadeCount> shadowRawInfos {};
        for (std::uint32_t cascade = 0; cascade < ShadowCascadeCount; ++cascade) {
            createSceneAttachment(shadowExtent, SceneDepthFormat,
                VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT,
                VK_IMAGE_ASPECT_DEPTH_BIT, sceneShadowImages[cascade],
                sceneShadowAllocations[cascade], sceneShadowViews[cascade]);
            sceneShadowStates[cascade] = {};
            shadowInfos[cascade].sampler = sceneShadowSampler;
            shadowInfos[cascade].imageView = sceneShadowViews[cascade];
            shadowInfos[cascade].imageLayout =
                VK_IMAGE_LAYOUT_DEPTH_STENCIL_READ_ONLY_OPTIMAL;
            shadowRawInfos[cascade].sampler = sceneShadowRawSampler;
            shadowRawInfos[cascade].imageView = sceneShadowViews[cascade];
            shadowRawInfos[cascade].imageLayout =
                VK_IMAGE_LAYOUT_DEPTH_STENCIL_READ_ONLY_OPTIMAL;
        }

        // Dois WRITEs por descriptor set (bindings 1 e 3), cada um com
        // descriptorCount=ShadowCascadeCount - arrays de sampler no shader
        // (ver scene3d_mesh.frag), nao mais samplers unicos.
        std::array<VkWriteDescriptorSet, FramesInFlight * 2> writes {};
        for (std::size_t index = 0; index < FramesInFlight; ++index) {
            writes[index * 2].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
            writes[index * 2].dstSet = sceneDescriptorSets[index];
            writes[index * 2].dstBinding = 1;
            writes[index * 2].descriptorCount = ShadowCascadeCount;
            writes[index * 2].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
            writes[index * 2].pImageInfo = shadowInfos.data();

            writes[index * 2 + 1].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
            writes[index * 2 + 1].dstSet = sceneDescriptorSets[index];
            writes[index * 2 + 1].dstBinding = 3;
            writes[index * 2 + 1].descriptorCount = ShadowCascadeCount;
            writes[index * 2 + 1].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
            writes[index * 2 + 1].pImageInfo = shadowRawInfos.data();
        }
        vkUpdateDescriptorSets(device, static_cast<std::uint32_t>(writes.size()),
            writes.data(), 0, nullptr);
    }

    void updateTemporalDescriptorSets(VkImageView hdrColorView,
        VkImageView depthView, VkImageView motionVectorView,
        const std::array<VkImageView, FramesInFlight>& historyViews,
        const std::array<VkDescriptorSet, FramesInFlight>& resolveSets,
        const std::array<VkDescriptorSet, FramesInFlight>& tonemapSets) {
        // Cada slot referencia de forma imutavel os attachments que usara:
        // resolve[i] le history[i-1] e escreve history[i]; tonemap[i] le o
        // history[i] recem-resolvido. Como os sets nao mudam durante frames
        // em voo, a GPU nunca observa uma troca de binding pela metade.
        for (std::size_t frameIndex = 0; frameIndex < FramesInFlight;
            ++frameIndex) {
            const std::size_t historyReadIndex =
                (frameIndex + FramesInFlight - 1) % FramesInFlight;
            std::array<VkDescriptorImageInfo, 5> imageInfos {};
            imageInfos[0] = { sceneTonemapSampler, hdrColorView,
                VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL };
            imageInfos[1] = { sceneDepthSampleSampler, depthView,
                VK_IMAGE_LAYOUT_DEPTH_STENCIL_READ_ONLY_OPTIMAL };
            imageInfos[2] = { sceneTonemapSampler, motionVectorView,
                VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL };
            imageInfos[3] = { sceneTaaHistorySampler,
                historyViews[historyReadIndex],
                VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL };
            imageInfos[4] = { sceneTonemapSampler, historyViews[frameIndex],
                VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL };

            std::array<VkWriteDescriptorSet, 5> writes {};
            for (std::size_t binding = 0; binding < 4; ++binding) {
                writes[binding].sType =
                    VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
                writes[binding].dstSet = resolveSets[frameIndex];
                writes[binding].dstBinding =
                    static_cast<std::uint32_t>(binding);
                writes[binding].descriptorCount = 1;
                writes[binding].descriptorType =
                    VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
                writes[binding].pImageInfo = &imageInfos[binding];
            }
            writes[4].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
            writes[4].dstSet = tonemapSets[frameIndex];
            writes[4].dstBinding = 0;
            writes[4].descriptorCount = 1;
            writes[4].descriptorType =
                VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
            writes[4].pImageInfo = &imageInfos[4];
            vkUpdateDescriptorSets(device,
                static_cast<std::uint32_t>(writes.size()), writes.data(),
                0, nullptr);
        }
    }

    void ensureSceneDirectDepth(Extent2D requestedExtent) {
        const VkExtent2D extent { requestedExtent.width, requestedExtent.height };
        if (sceneDirectDepthImage != VK_NULL_HANDLE
            && sceneDirectExtent.width == extent.width
            && sceneDirectExtent.height == extent.height) {
            return;
        }
        check(vkDeviceWaitIdle(device), "vkDeviceWaitIdle(scene direct resize)");
        destroySceneDirectDepth();
        createSceneAttachment(extent, SceneDepthFormat,
            VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT,
            VK_IMAGE_ASPECT_DEPTH_BIT, sceneDirectDepthImage,
            sceneDirectDepthAllocation, sceneDirectDepthView);
        createSceneAttachment(extent, SceneHdrColorFormat,
            VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT,
            VK_IMAGE_ASPECT_COLOR_BIT, sceneDirectHdrColorImage,
            sceneDirectHdrColorAllocation, sceneDirectHdrColorView);
        createSceneAttachment(extent, SceneMotionVectorFormat,
            VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT,
            VK_IMAGE_ASPECT_COLOR_BIT, sceneDirectMotionVectorImage,
            sceneDirectMotionVectorAllocation, sceneDirectMotionVectorView);
        for (std::size_t index = 0; index < sceneDirectTaaHistoryImage.size(); ++index) {
            createSceneAttachment(extent, SceneHdrColorFormat,
                VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT,
                VK_IMAGE_ASPECT_COLOR_BIT, sceneDirectTaaHistoryImage[index],
                sceneDirectTaaHistoryAllocation[index], sceneDirectTaaHistoryView[index]);
        }
        sceneDirectExtent = extent;
        updateTemporalDescriptorSets(sceneDirectHdrColorView,
            sceneDirectDepthView, sceneDirectMotionVectorView,
            sceneDirectTaaHistoryView, sceneDirectTaaResolveDescriptorSets,
            sceneDirectTonemapDescriptorSets);
    }

    void ensureScene3DTarget(Extent2D requestedExtent) {
        ensureSceneShadowTarget();
        const VkExtent2D extent { requestedExtent.width, requestedExtent.height };
        if (sceneColorImage != VK_NULL_HANDLE
            && sceneTargetExtent.width == extent.width
            && sceneTargetExtent.height == extent.height) {
            return;
        }
        if (!imguiInitialized) {
            throw std::runtime_error("Scene3D viewport requires initialized ImGui");
        }
        check(vkDeviceWaitIdle(device), "vkDeviceWaitIdle(scene target resize)");
        destroyScene3DTarget();

        const auto createImage = [&](VkExtent2D imageExtent, VkFormat format,
            VkImageUsageFlags usage, VkImageAspectFlags aspect,
            VkImage& image, VmaAllocation& allocation, VkImageView& view) {
            VkImageCreateInfo imageInfo { VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO };
            imageInfo.imageType = VK_IMAGE_TYPE_2D;
            imageInfo.format = format;
            imageInfo.extent = { imageExtent.width, imageExtent.height, 1 };
            imageInfo.mipLevels = 1;
            imageInfo.arrayLayers = 1;
            imageInfo.samples = VK_SAMPLE_COUNT_1_BIT;
            imageInfo.tiling = VK_IMAGE_TILING_OPTIMAL;
            imageInfo.usage = usage;
            imageInfo.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
            imageInfo.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
            VmaAllocationCreateInfo allocationInfo {};
            allocationInfo.usage = VMA_MEMORY_USAGE_AUTO_PREFER_DEVICE;
            check(vmaCreateImage(allocator, &imageInfo, &allocationInfo,
                &image, &allocation, nullptr), "vmaCreateImage(scene target)");
            VkImageViewCreateInfo viewInfo { VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO };
            viewInfo.image = image;
            viewInfo.viewType = VK_IMAGE_VIEW_TYPE_2D;
            viewInfo.format = format;
            viewInfo.subresourceRange.aspectMask = aspect;
            viewInfo.subresourceRange.levelCount = 1;
            viewInfo.subresourceRange.layerCount = 1;
            check(vkCreateImageView(device, &viewInfo, nullptr, &view),
                "vkCreateImageView(scene target)");
        };

        createImage(extent, swapchainFormat,
            VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT,
            VK_IMAGE_ASPECT_COLOR_BIT, sceneColorImage, sceneColorAllocation, sceneColorView);
        createImage(extent, SceneDepthFormat,
            VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT,
            VK_IMAGE_ASPECT_DEPTH_BIT, sceneDepthImage, sceneDepthAllocation, sceneDepthView);
        createImage(extent, SceneHdrColorFormat,
            VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT,
            VK_IMAGE_ASPECT_COLOR_BIT, sceneHdrColorImage, sceneHdrColorAllocation, sceneHdrColorView);
        createImage(extent, SceneMotionVectorFormat,
            VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT,
            VK_IMAGE_ASPECT_COLOR_BIT, sceneMotionVectorImage,
            sceneMotionVectorAllocation, sceneMotionVectorView);
        for (std::size_t index = 0; index < sceneTaaHistoryImage.size(); ++index) {
            createImage(extent, SceneHdrColorFormat,
                VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT,
                VK_IMAGE_ASPECT_COLOR_BIT, sceneTaaHistoryImage[index],
                sceneTaaHistoryAllocation[index], sceneTaaHistoryView[index]);
        }
        sceneTargetExtent = extent;
        sceneColorImGuiDescriptor = ImGui_ImplVulkan_AddTexture(
            nearestSampler, sceneColorView, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
        if (sceneColorImGuiDescriptor == VK_NULL_HANDLE) {
            throw std::runtime_error("ImGui failed to allocate the Scene3D viewport descriptor");
        }

        updateTemporalDescriptorSets(sceneHdrColorView, sceneDepthView,
            sceneMotionVectorView, sceneTaaHistoryView,
            sceneTaaResolveDescriptorSets, sceneTonemapDescriptorSets);
    }

    void createSpriteResources() {
        VkSamplerCreateInfo samplerInfo { VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO };
        samplerInfo.magFilter = VK_FILTER_NEAREST;
        samplerInfo.minFilter = VK_FILTER_NEAREST;
        samplerInfo.mipmapMode = VK_SAMPLER_MIPMAP_MODE_NEAREST;
        samplerInfo.addressModeU = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
        samplerInfo.addressModeV = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
        samplerInfo.addressModeW = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
        samplerInfo.maxLod = 0.0f;
        check(vkCreateSampler(device, &samplerInfo, nullptr, &nearestSampler), "vkCreateSampler");

        VkDescriptorSetLayoutBinding binding {};
        binding.binding = 0;
        binding.descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
        binding.descriptorCount = 1;
        binding.stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;

        VkDescriptorSetLayoutCreateInfo layoutInfo { VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO };
        layoutInfo.bindingCount = 1;
        layoutInfo.pBindings = &binding;
        check(vkCreateDescriptorSetLayout(device, &layoutInfo, nullptr, &spriteDescriptorSetLayout),
            "vkCreateDescriptorSetLayout");

        const auto setCount = static_cast<std::uint32_t>(spriteDescriptorSets.size());
        VkDescriptorPoolSize poolSize {};
        poolSize.type = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
        poolSize.descriptorCount = setCount;
        VkDescriptorPoolCreateInfo poolInfo { VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO };
        poolInfo.maxSets = setCount;
        poolInfo.poolSizeCount = 1;
        poolInfo.pPoolSizes = &poolSize;
        check(vkCreateDescriptorPool(device, &poolInfo, nullptr, &spriteDescriptorPool), "vkCreateDescriptorPool");

        const std::vector<VkDescriptorSetLayout> setLayouts(setCount, spriteDescriptorSetLayout);
        VkDescriptorSetAllocateInfo allocInfo { VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO };
        allocInfo.descriptorPool = spriteDescriptorPool;
        allocInfo.descriptorSetCount = setCount;
        allocInfo.pSetLayouts = setLayouts.data();
        check(vkAllocateDescriptorSets(device, &allocInfo, spriteDescriptorSets.data()), "vkAllocateDescriptorSets");

        const std::string shaderDir = MATTERENGINE_SHADER_DIR;
        const std::vector<std::uint32_t> vertexSpirv = readSpirv(shaderDir + "/sprite.vert.spv");
        const std::vector<std::uint32_t> fragmentSpirv = readSpirv(shaderDir + "/sprite.frag.spv");

        VkShaderModuleCreateInfo vertexModuleInfo { VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO };
        vertexModuleInfo.codeSize = vertexSpirv.size() * sizeof(std::uint32_t);
        vertexModuleInfo.pCode = vertexSpirv.data();
        check(vkCreateShaderModule(device, &vertexModuleInfo, nullptr, &spriteVertexModule), "vkCreateShaderModule(sprite.vert)");

        VkShaderModuleCreateInfo fragmentModuleInfo { VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO };
        fragmentModuleInfo.codeSize = fragmentSpirv.size() * sizeof(std::uint32_t);
        fragmentModuleInfo.pCode = fragmentSpirv.data();
        check(vkCreateShaderModule(device, &fragmentModuleInfo, nullptr, &spriteFragmentModule), "vkCreateShaderModule(sprite.frag)");

        std::array<VkPipelineShaderStageCreateInfo, 2> stages {};
        stages[0].sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
        stages[0].stage = VK_SHADER_STAGE_VERTEX_BIT;
        stages[0].module = spriteVertexModule;
        stages[0].pName = "main";
        stages[1].sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
        stages[1].stage = VK_SHADER_STAGE_FRAGMENT_BIT;
        stages[1].module = spriteFragmentModule;
        stages[1].pName = "main";

        // No vertex buffer at all - sprite.vert derives the quad's 4 corners
        // from push constants, picked per-vertex via gl_VertexIndex.
        VkPipelineVertexInputStateCreateInfo vertexInput { VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO };

        VkPipelineInputAssemblyStateCreateInfo inputAssembly { VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO };
        inputAssembly.topology = VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST;

        VkPipelineViewportStateCreateInfo viewportState { VK_STRUCTURE_TYPE_PIPELINE_VIEWPORT_STATE_CREATE_INFO };
        viewportState.viewportCount = 1;
        viewportState.scissorCount = 1;

        VkPipelineRasterizationStateCreateInfo rasterizer { VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO };
        rasterizer.polygonMode = VK_POLYGON_MODE_FILL;
        rasterizer.cullMode = VK_CULL_MODE_NONE;
        rasterizer.frontFace = VK_FRONT_FACE_COUNTER_CLOCKWISE;
        rasterizer.lineWidth = 1.0f;

        VkPipelineMultisampleStateCreateInfo multisample { VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO };
        multisample.rasterizationSamples = VK_SAMPLE_COUNT_1_BIT;

        VkPipelineDepthStencilStateCreateInfo depth { VK_STRUCTURE_TYPE_PIPELINE_DEPTH_STENCIL_STATE_CREATE_INFO };

        VkPipelineColorBlendAttachmentState blendAttachment {};
        blendAttachment.colorWriteMask = VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT
            | VK_COLOR_COMPONENT_B_BIT | VK_COLOR_COMPONENT_A_BIT;
        // Enabled so baked sprites with transparent regions (e.g. a square
        // ball texture with a circle inscribed in it) composite correctly
        // over whatever's underneath - a no-op for the fully-opaque
        // grass/blit use cases (alpha 1 everywhere), so one pipeline still
        // serves all three.
        blendAttachment.blendEnable = VK_TRUE;
        blendAttachment.srcColorBlendFactor = VK_BLEND_FACTOR_SRC_ALPHA;
        blendAttachment.dstColorBlendFactor = VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA;
        blendAttachment.colorBlendOp = VK_BLEND_OP_ADD;
        blendAttachment.srcAlphaBlendFactor = VK_BLEND_FACTOR_ONE;
        blendAttachment.dstAlphaBlendFactor = VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA;
        blendAttachment.alphaBlendOp = VK_BLEND_OP_ADD;

        VkPipelineColorBlendStateCreateInfo blend { VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO };
        blend.attachmentCount = 1;
        blend.pAttachments = &blendAttachment;

        const std::array<VkDynamicState, 2> dynamics { VK_DYNAMIC_STATE_VIEWPORT, VK_DYNAMIC_STATE_SCISSOR };
        VkPipelineDynamicStateCreateInfo dynamic { VK_STRUCTURE_TYPE_PIPELINE_DYNAMIC_STATE_CREATE_INFO };
        dynamic.dynamicStateCount = static_cast<std::uint32_t>(dynamics.size());
        dynamic.pDynamicStates = dynamics.data();

        VkPushConstantRange pushRange {};
        pushRange.stageFlags = VK_SHADER_STAGE_VERTEX_BIT;
        pushRange.size = 14 * sizeof(float); // 4 corners + 4 perspective depths + viewport size

        VkPipelineLayoutCreateInfo layoutCreateInfo { VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO };
        layoutCreateInfo.setLayoutCount = 1;
        layoutCreateInfo.pSetLayouts = &spriteDescriptorSetLayout;
        layoutCreateInfo.pushConstantRangeCount = 1;
        layoutCreateInfo.pPushConstantRanges = &pushRange;
        check(vkCreatePipelineLayout(device, &layoutCreateInfo, nullptr, &spritePipelineLayout), "vkCreatePipelineLayout(sprite)");

        VkPipelineRenderingCreateInfo renderingInfo { VK_STRUCTURE_TYPE_PIPELINE_RENDERING_CREATE_INFO };
        renderingInfo.colorAttachmentCount = 1;
        renderingInfo.pColorAttachmentFormats = &swapchainFormat;

        VkGraphicsPipelineCreateInfo pipelineInfo { VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO };
        pipelineInfo.pNext = &renderingInfo;
        pipelineInfo.stageCount = static_cast<std::uint32_t>(stages.size());
        pipelineInfo.pStages = stages.data();
        pipelineInfo.pVertexInputState = &vertexInput;
        pipelineInfo.pInputAssemblyState = &inputAssembly;
        pipelineInfo.pViewportState = &viewportState;
        pipelineInfo.pRasterizationState = &rasterizer;
        pipelineInfo.pMultisampleState = &multisample;
        pipelineInfo.pDepthStencilState = &depth;
        pipelineInfo.pColorBlendState = &blend;
        pipelineInfo.pDynamicState = &dynamic;
        pipelineInfo.layout = spritePipelineLayout;
        check(vkCreateGraphicsPipelines(device, VK_NULL_HANDLE, 1, &pipelineInfo, nullptr, &spritePipeline),
            "vkCreateGraphicsPipelines(sprite)");
    }

    void recreateSwapchain() {
        int pixelWidth = 0;
        int pixelHeight = 0;
        SDL_GetWindowSizeInPixels(window, &pixelWidth, &pixelHeight);
        if (pixelWidth <= 0 || pixelHeight <= 0) {
            swapchainDirty = true;
            return;
        }
        check(vkDeviceWaitIdle(device), "vkDeviceWaitIdle(swapchain)");
        destroySwapchain();
        createSwapchain();
    }

    void ensureFrame() const {
        if (!frameActive) {
            throw std::runtime_error("No active RHI frame");
        }
    }

    SDL_Window* window = nullptr;
    bool vsync = true;
    bool validationEnabled = false;
    bool debugUtilsEnabled = false;
    bool imguiInitialized = false;
    bool swapchainDirty = false;
    bool frameActive = false;

    VkInstance instance = VK_NULL_HANDLE;
    VkDebugUtilsMessengerEXT debugMessenger = VK_NULL_HANDLE;
    VkSurfaceKHR surface = VK_NULL_HANDLE;
    VkPhysicalDevice physicalDevice = VK_NULL_HANDLE;
    VkDevice device = VK_NULL_HANDLE;
    QueueFamilies queueFamilies;
    VkQueue graphicsQueue = VK_NULL_HANDLE;
    VkQueue presentQueue = VK_NULL_HANDLE;
    VmaAllocator allocator = VK_NULL_HANDLE;

    VkSwapchainKHR swapchain = VK_NULL_HANDLE;
    VkFormat swapchainFormat = VK_FORMAT_UNDEFINED;
    VkExtent2D swapchainExtent {};
    std::uint32_t minImageCount = 2;
    std::vector<VkImage> swapchainImages;
    std::vector<VkImageView> swapchainImageViews;
    std::vector<bool> swapchainInitialized;
    std::vector<VkFence> imageFences;

    std::array<Frame, FramesInFlight> frames {};
    VkQueryPool frameTimestampQueryPool = VK_NULL_HANDLE;
    float timestampPeriodNanoseconds = 1.0f;
    FramePerformanceMetrics frameMetrics;
    std::uint32_t currentFrame = 0;
    std::uint32_t currentImage = 0;
    PipelineHandle boundPipeline;

    std::vector<BufferResource> buffers;
    std::vector<ShaderResource> shaders;
    std::vector<PipelineResource> pipelines;
    std::vector<TextureResource> textures;

    // Draws a whole texture stretched across 4 arbitrary screen-space
    // corners (via push constants, no vertex buffer) - used both for the
    // low-res pixel-art upscale onto the swapchain and for drawing
    // pre-baked static world textures like the grass field. Self-managed
    // internally (like ImGui manages its own pipeline), not exposed through
    // the generic createGraphicsPipeline path, since it needs a descriptor
    // set that no other RHI consumer needs. Slot 0 is reserved for the
    // swapchain blit; each submitted world layer gets another fixed slot.
    // This keeps two different grass textures from rewriting a descriptor
    // set that an earlier draw in the same command buffer still references.
    VkSampler nearestSampler = VK_NULL_HANDLE;
    VkDescriptorSetLayout spriteDescriptorSetLayout = VK_NULL_HANDLE;
    VkDescriptorPool spriteDescriptorPool = VK_NULL_HANDLE;
    std::array<VkDescriptorSet, 4> spriteDescriptorSets {};
    std::size_t worldSpriteDrawCount = 0;
    VkPipelineLayout spritePipelineLayout = VK_NULL_HANDLE;
    VkPipeline spritePipeline = VK_NULL_HANDLE;
    VkShaderModule spriteVertexModule = VK_NULL_HANDLE;
    VkShaderModule spriteFragmentModule = VK_NULL_HANDLE;

    // Real-time 3D preview path: a GPU shadow map, a camera depth buffer and
    // a color target sampled directly by ImGui. No per-pixel visibility work
    // or shadow tessellation is performed on the CPU.
    VkSampler sceneShadowSampler = VK_NULL_HANDLE;
    // Mesmas imagens de sceneShadowImages, sampler SEM comparacao - so pra
    // busca de bloqueadores do PCSS (binding 3, ver createScene3DResources).
    VkSampler sceneShadowRawSampler = VK_NULL_HANDLE;
    VkDescriptorSetLayout sceneDescriptorSetLayout = VK_NULL_HANDLE;
    VkDescriptorPool sceneDescriptorPool = VK_NULL_HANDLE;
    std::array<VkDescriptorSet, FramesInFlight> sceneDescriptorSets {};
    std::array<VkBuffer, FramesInFlight> sceneUniformBuffers {};
    std::array<VmaAllocation, FramesInFlight> sceneUniformAllocations {};
    std::array<void*, FramesInFlight> sceneUniformMapped {};
    std::array<VkBuffer, FramesInFlight> sceneMeshInstanceBuffers {};
    std::array<VmaAllocation, FramesInFlight>
        sceneMeshInstanceAllocations {};
    std::array<void*, FramesInFlight> sceneMeshInstanceMapped {};
    std::array<std::size_t, FramesInFlight>
        sceneMeshInstanceCapacities {};
    // SSBO de luzes (binding 2 de sceneDescriptorSetLayout) - cresce sob
    // demanda como o buffer de instancia de mesh acima, mas ao contrario
    // daquele (um vertex buffer, so referenciado pelo vkCmdBindVertexBuffers
    // de cada draw) este e um binding de descriptor set: toda realocacao
    // exige reemitir vkUpdateDescriptorSets, ver ensureSceneLightCapacity.
    std::array<VkBuffer, FramesInFlight> sceneLightBuffers {};
    std::array<VmaAllocation, FramesInFlight> sceneLightAllocations {};
    std::array<void*, FramesInFlight> sceneLightMapped {};
    std::array<std::size_t, FramesInFlight> sceneLightCapacities {};
    // Layout so com o UBO de cena (set=0), sem push constants - usado pelos
    // passes que nao precisam de material/textura: sombra de mesh (depth-only)
    // e ceu procedural.
    VkPipelineLayout scenePipelineLayout = VK_NULL_HANDLE;
    VkPipeline sceneSkyPipeline = VK_NULL_HANDLE;
    VkShaderModule sceneSkyVertexModule = VK_NULL_HANDLE;
    VkShaderModule sceneSkyFragmentModule = VK_NULL_HANDLE;
    // Real GPU mesh path (MeshData3D uploads via createBuffer/writeBuffer),
    // com layout proprio (sceneMeshPipelineLayout) por precisar dos sets de
    // textura de material (1 e 2) alem do UBO de cena (0).
    VkPipeline sceneMeshPipeline = VK_NULL_HANDLE;
    VkPipeline sceneMeshShadowPipeline = VK_NULL_HANDLE;
    // Pre-pass de profundidade da cena principal (Fase 5) - mesmo layout
    // depth-only da sombra (scenePipelineLayout), so vertice, mas usando
    // cameraViewProjection. sceneMeshPipeline passa a so testar (EQUAL) o
    // que este pre-pass ja escreveu, ver createScene3DResources.
    VkPipeline sceneMeshDepthPrepassPipeline = VK_NULL_HANDLE;
    VkShaderModule sceneMeshVertexModule = VK_NULL_HANDLE;
    VkShaderModule sceneMeshFragmentModule = VK_NULL_HANDLE;
    VkShaderModule sceneMeshShadowVertexModule = VK_NULL_HANDLE;
    VkShaderModule sceneMeshDepthVertexModule = VK_NULL_HANDLE;
    // Per-material albedo texture binding (set=1) for sceneMeshPipeline -
    // sceneMeshShadowPipeline never needs it, it stays on scenePipelineLayout.
    VkPipelineLayout sceneMeshPipelineLayout = VK_NULL_HANDLE;
    VkSampler materialSampler = VK_NULL_HANDLE;
    VkDescriptorSetLayout materialDescriptorSetLayout = VK_NULL_HANDLE;
    VkDescriptorPool materialDescriptorPool = VK_NULL_HANDLE;
    // A 1x1 white texture bound whenever a mesh has no real albedo map, so
    // the shader can unconditionally sample set=1 without branching.
    TextureHandle defaultMaterialTexture;
    VkImage sceneColorImage = VK_NULL_HANDLE;
    VmaAllocation sceneColorAllocation = VK_NULL_HANDLE;
    VkImageView sceneColorView = VK_NULL_HANDLE;
    SceneAttachmentState3D sceneColorState;
    VkDescriptorSet sceneColorImGuiDescriptor = VK_NULL_HANDLE;
    VkImage sceneDepthImage = VK_NULL_HANDLE;
    VmaAllocation sceneDepthAllocation = VK_NULL_HANDLE;
    VkImageView sceneDepthView = VK_NULL_HANDLE;
    SceneAttachmentState3D sceneDepthState;
    // Um mapa por cascata (ver ShadowCascadeCount) - todos do mesmo tamanho
    // (SceneShadowMapSize), compartilhados entre o caminho offscreen
    // (preview) e o direto-pro-swapchain (Laboratorio), igual o unico mapa
    // fazia antes das cascatas.
    std::array<VkImage, ShadowCascadeCount> sceneShadowImages {};
    std::array<VmaAllocation, ShadowCascadeCount> sceneShadowAllocations {};
    std::array<VkImageView, ShadowCascadeCount> sceneShadowViews {};
    std::array<SceneAttachmentState3D, ShadowCascadeCount> sceneShadowStates {};
    VkExtent2D sceneTargetExtent {};
    VkImage sceneDirectDepthImage = VK_NULL_HANDLE;
    VmaAllocation sceneDirectDepthAllocation = VK_NULL_HANDLE;
    VkImageView sceneDirectDepthView = VK_NULL_HANDLE;
    SceneAttachmentState3D sceneDirectDepthState;
    VkExtent2D sceneDirectExtent {};

    // Alvos HDR do passe opaco (ver renderScene3DInternal): um pareado com
    // sceneColorImage (preview offscreen, recriado em ensureScene3DTarget) e
    // outro pareado com sceneDirectDepthImage (caminho direto ao swapchain,
    // recriado em ensureSceneDirectDepth). O passe de tonemap le um destes
    // via o descriptor set correspondente e escreve o resultado LDR no
    // destino final (sceneColorImage ou a propria imagem do swapchain).
    VkImage sceneHdrColorImage = VK_NULL_HANDLE;
    VmaAllocation sceneHdrColorAllocation = VK_NULL_HANDLE;
    VkImageView sceneHdrColorView = VK_NULL_HANDLE;
    SceneAttachmentState3D sceneHdrColorState;
    VkImage sceneDirectHdrColorImage = VK_NULL_HANDLE;
    VmaAllocation sceneDirectHdrColorAllocation = VK_NULL_HANDLE;
    VkImageView sceneDirectHdrColorView = VK_NULL_HANDLE;
    SceneAttachmentState3D sceneDirectHdrColorState;

    // Vetores de movimento por pixel (ver scene3d_mesh.vert/frag) - segundo
    // attachment de cor do MESMO passe opaco que escreve sceneHdrColorImage/
    // sceneDirectHdrColorImage (MRT - Multiple Render Targets), nao um passe
    // separado. So precisa de 1 imagem por caminho (nao FramesInFlight como
    // o historico do TAA): e escrito e lido dentro do MESMO quadro, nunca
    // precisa sobreviver pro proximo.
    VkImage sceneMotionVectorImage = VK_NULL_HANDLE;
    VmaAllocation sceneMotionVectorAllocation = VK_NULL_HANDLE;
    VkImageView sceneMotionVectorView = VK_NULL_HANDLE;
    SceneAttachmentState3D sceneMotionVectorState;
    VkImage sceneDirectMotionVectorImage = VK_NULL_HANDLE;
    VmaAllocation sceneDirectMotionVectorAllocation = VK_NULL_HANDLE;
    VkImageView sceneDirectMotionVectorView = VK_NULL_HANDLE;
    SceneAttachmentState3D sceneDirectMotionVectorState;

    // Um descriptor por frame em voo e por caminho. Descriptor sets podem
    // ser lidos pela GPU depois do vkQueueSubmit; reescrever um unico set no
    // quadro seguinte, como fazia a implementacao antiga, era comportamento
    // indefinido e podia alternar os slots do historico durante a execucao.
    VkSampler sceneTonemapSampler = VK_NULL_HANDLE;
    VkDescriptorSetLayout sceneTonemapDescriptorSetLayout = VK_NULL_HANDLE;
    VkDescriptorPool sceneTonemapDescriptorPool = VK_NULL_HANDLE;
    std::array<VkDescriptorSet, FramesInFlight> sceneTonemapDescriptorSets {};
    std::array<VkDescriptorSet, FramesInFlight>
        sceneDirectTonemapDescriptorSets {};
    VkPipelineLayout sceneTonemapPipelineLayout = VK_NULL_HANDLE;
    VkPipeline sceneTonemapPipeline = VK_NULL_HANDLE;
    VkShaderModule sceneTonemapVertexModule = VK_NULL_HANDLE;
    VkShaderModule sceneTonemapFragmentModule = VK_NULL_HANDLE;

    // Historico de TAA (Fase 6) - 2 slots por caminho, indexados por
    // currentFrame (nao um ping-pong separado: currentFrame ja alterna em
    // lockstep com a submissao de cada quadro real, ver renderScene3DInternal).
    // history[currentFrame] e escrito agora pelo resolve; history[1-currentFrame]
    // ja tem o resultado resolvido do quadro passado, usado como entrada de
    // historico. O mesmo buffer tambem vira a entrada do tonemap (que le o
    // resultado ja resolvido, nao mais o HDR cru).
    std::array<VkImage, FramesInFlight> sceneTaaHistoryImage {};
    std::array<VmaAllocation, FramesInFlight> sceneTaaHistoryAllocation {};
    std::array<VkImageView, FramesInFlight> sceneTaaHistoryView {};
    std::array<SceneAttachmentState3D, FramesInFlight> sceneTaaHistoryState {};
    std::array<VkImage, FramesInFlight> sceneDirectTaaHistoryImage {};
    std::array<VmaAllocation, FramesInFlight> sceneDirectTaaHistoryAllocation {};
    std::array<VkImageView, FramesInFlight> sceneDirectTaaHistoryView {};
    std::array<SceneAttachmentState3D, FramesInFlight> sceneDirectTaaHistoryState {};
    bool sceneTaaHistoryValid = false;
    bool sceneDirectTaaHistoryValid = false;

    // Pipeline de resolve de TAA: mesmo triangulo cheio de tela (reaproveita
    // sceneTonemapVertexModule - nenhum binding/push constant no vertex, ver
    // tonemap.vert), mas com 2 sets: set 0 = sceneDescriptorSetLayout (UBO da
    // cena) e set 1 = cor HDR atual, profundidade, vetor de movimento e
    // historico. Existe um set 1 imutavel para cada frame em voo; todos eles
    // sao escritos somente ao criar/recriar os attachments, depois de
    // vkDeviceWaitIdle. Assim nenhum descriptor usado por uma submissao
    // pendente e alterado pela CPU.
    VkSampler sceneDepthSampleSampler = VK_NULL_HANDLE;
    // Filtro LINEAR dedicado ao historico do TAA (ver
    // updateTemporalDescriptorSets) - diferente de sceneTonemapSampler (NEAREST),
    // que serve leituras em coordenadas exatamente alinhadas ao pixel
    // (cor atual, profundidade). O historico e amostrado em
    // `previousTexCoord`, resultado de uma reprojecao por matriz que quase
    // nunca cai exatamente num texel - amostrar isso com NEAREST faz a
    // leitura "saltar" pro texel mais proximo a cada quadro em vez de
    // interpolar suavemente, produzindo um tremor/cintilacao visivel em
    // qualquer geometria (nao so texturas de alto contraste), mesmo com a
    // camera parada.
    VkSampler sceneTaaHistorySampler = VK_NULL_HANDLE;
    VkDescriptorSetLayout sceneTaaResolveDescriptorSetLayout = VK_NULL_HANDLE;
    VkDescriptorPool sceneTaaResolveDescriptorPool = VK_NULL_HANDLE;
    std::array<VkDescriptorSet, FramesInFlight>
        sceneTaaResolveDescriptorSets {};
    std::array<VkDescriptorSet, FramesInFlight>
        sceneDirectTaaResolveDescriptorSets {};
    VkPipelineLayout sceneTaaResolvePipelineLayout = VK_NULL_HANDLE;
    VkPipeline sceneTaaResolvePipeline = VK_NULL_HANDLE;
    VkShaderModule sceneTaaResolveFragmentModule = VK_NULL_HANDLE;

    bool renderTargetPassActive = false;
    bool swapchainPassActive = false;
    TextureHandle activeRenderTarget;
};

VulkanDevice::VulkanDevice()
    : m_impl(std::make_unique<Impl>()) {
}

VulkanDevice::~VulkanDevice() {
    shutdown();
}

void VulkanDevice::initialize(SDL_Window* window, bool vsync, const std::string& applicationName) {
    m_impl->initialize(window, vsync, applicationName);
}

void VulkanDevice::shutdown() {
    if (m_impl) m_impl->shutdown();
}

BufferHandle VulkanDevice::createBuffer(const BufferDesc& desc) { return m_impl->createBuffer(desc); }
ShaderHandle VulkanDevice::createShader(const ShaderDesc& desc) { return m_impl->createShader(desc); }
PipelineHandle VulkanDevice::createGraphicsPipeline(const GraphicsPipelineDesc& desc) { return m_impl->createGraphicsPipeline(desc); }
TextureHandle VulkanDevice::createRenderTarget(const TextureDesc& desc) { return m_impl->createRenderTarget(desc); }
TextureHandle VulkanDevice::createTexture2D(Extent2D extent, std::span<const std::byte> rgbaPixels) {
    return m_impl->createTexture2D(extent, rgbaPixels);
}
void VulkanDevice::destroyBuffer(BufferHandle handle) { m_impl->destroyBuffer(handle); }
void VulkanDevice::destroyShader(ShaderHandle handle) { m_impl->destroyShader(handle); }
void VulkanDevice::destroyPipeline(PipelineHandle handle) { m_impl->destroyPipeline(handle); }
void VulkanDevice::destroyTexture(TextureHandle handle) { m_impl->destroyTexture(handle); }
void VulkanDevice::writeBuffer(BufferHandle handle, std::size_t offset, std::span<const std::byte> data) { m_impl->writeBuffer(handle, offset, data); }
FrameStatus VulkanDevice::beginFrame() { return m_impl->beginFrame(); }
CommandList& VulkanDevice::commandList() { return *m_impl; }
void VulkanDevice::beginRenderTargetPass(TextureHandle target, ClearColor clearColor) { m_impl->beginRenderTargetPass(target, clearColor); }
void VulkanDevice::endRenderTargetPass() { m_impl->endRenderTargetPass(); }
void VulkanDevice::blitToSwapchain(TextureHandle source, ClearColor clearColor) { m_impl->blitToSwapchain(source, clearColor); }
void VulkanDevice::drawWorldSprite(TextureHandle source, const std::array<float, 8>& corners,
    const std::array<float, 4>& perspectiveDepths) {
    m_impl->drawWorldSprite(source, corners, perspectiveDepths);
}
void VulkanDevice::endFrame() { m_impl->endFrame(); }
void VulkanDevice::requestSwapchainRebuild() { m_impl->swapchainDirty = true; }
void VulkanDevice::waitIdle() { if (m_impl->device != VK_NULL_HANDLE) check(vkDeviceWaitIdle(m_impl->device), "vkDeviceWaitIdle"); }
Extent2D VulkanDevice::drawableExtent() const { return { m_impl->swapchainExtent.width, m_impl->swapchainExtent.height }; }
std::uint32_t VulkanDevice::currentFrameSlot() const { return m_impl->currentFrame; }
const char* VulkanDevice::backendName() const { return "Vulkan 1.4"; }
FramePerformanceMetrics VulkanDevice::framePerformanceMetrics() const {
    return m_impl->frameMetrics;
}
void VulkanDevice::initializeImGui(SDL_Window* window) { m_impl->initializeImGui(window); }
void VulkanDevice::shutdownImGui() { m_impl->shutdownImGui(); }
void VulkanDevice::beginImGuiFrame() { m_impl->beginImGuiFrame(); }
void VulkanDevice::renderImGui(ImDrawData* drawData) { m_impl->renderImGui(drawData); }
std::uint64_t VulkanDevice::createImGuiTexture(Extent2D extent, std::span<const std::byte> rgbaPixels) {
    return m_impl->createImGuiTexture(extent, rgbaPixels);
}
std::uint64_t VulkanDevice::renderScene3D(const Scene3DFrame& scene, Extent2D extent) {
    return m_impl->renderScene3D(scene, extent);
}
void VulkanDevice::renderScene3DToSwapchain(const Scene3DFrame& scene) {
    m_impl->renderScene3DToSwapchain(scene);
}

} // namespace MatterEngine::RHI::Vulkan
