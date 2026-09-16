#include "Engine/RHI/Vulkan/VulkanDevice.hpp"

#include "Engine/Core/Log.hpp"
#include "Engine/Core/Version.hpp"
#include "Engine/Geometry/MeshData3D.hpp"
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

constexpr std::uint32_t FrameTimestampCount = 10;

constexpr std::uint32_t TargetVulkanVersion = VK_API_VERSION_1_4;
constexpr std::uint32_t FramesInFlight = 3;
constexpr VkFormat SceneDepthFormat = VK_FORMAT_D32_SFLOAT;
// Alvo intermediario do passe opaco (ceu+mesh) - meia precisao de ponto
// flutuante por canal, para guardar luminancia acima de 1.0 (destaques de sol,
// especular de metal) sem estourar, ate o passe de tonemap (ver
// createScene3DResources / tonemap.frag) comprimir de volta para o destino
// final em UNORM.
constexpr VkFormat SceneHdrColorFormat = VK_FORMAT_R16G16B16A16_SFLOAT;
// Um único texel guarda o multiplicador de exposição adaptado. Reusar o
// formato HDR já validado pelo device evita depender de suporte opcional a
// R16_SFLOAT como color attachment; o custo extra dos canais num alvo 1x1 é
// irrelevante.
constexpr VkFormat SceneAutoExposureFormat = SceneHdrColorFormat;
// Vetores de movimento por pixel (ver scene3d_mesh.frag) - deslocamento em
// UV entre este quadro e o passado, tipicamente uma fracao pequena de 1.0;
// 2 canais de meia precisao de ponto flutuante sobram de folga sem o custo
// de banda de um formato de 4 canais como o HDR.
constexpr VkFormat SceneMotionVectorFormat = VK_FORMAT_R16G16_SFLOAT;
// R = visibilidade da oclusão, G = distância linear à câmera para o
// upsample bilateral no TAA. Meia resolução, portanto 4 bytes por pixel
// resultam em apenas 1 byte por pixel de tela.
constexpr VkFormat SceneAmbientOcclusionFormat =
    VK_FORMAT_R16G16_SFLOAT;
// Glare solar e ghosts permanecem em HDR até o tonemap. O alvo roda a 1/8
// de cada dimensão (1/64 dos pixels); não há bloom geral neste caminho.
constexpr VkFormat SceneBloomGlareFormat = SceneHdrColorFormat;
// Normal (codificada em octaedro) + rugosidade + metalico por pixel, ver
// scene3d_mesh.frag/scene3d_sky.frag - terceiro attachment do MESMO passe
// opaco MRT, fundacao para um futuro passe de SSR (ainda sem nenhum
// consumidor). UNORM8 x4 (4 bytes) em vez de um formato de meia precisao de
// ponto flutuante (8+ bytes): a normal nao precisa de mais que isso para
// orientar um raio de tela, e a banda de memoria importa no hardware minimo
// do jogo (GTX 750 e equivalentes).
constexpr VkFormat SceneNormalRoughnessFormat = VK_FORMAT_R8G8B8A8_UNORM;
// Peso de reflectancia ambiente por pixel (ver outReflectance em
// scene3d_mesh.frag/scene3d_sky.frag) - quarto attachment do MESMO passe
// opaco MRT. UNORM8 pelo mesmo motivo do buffer de normal/rugosidade acima:
// e so um peso 0..1, sem precisao extra que valha o dobro (ou mais) de
// banda de um formato de ponto flutuante.
constexpr VkFormat SceneReflectanceFormat = VK_FORMAT_R8G8B8A8_UNORM;
// Metais comuns usam reflexão ambiente analítica no próprio mesh shader.
// O caminho SSR permanece compilável para experimentos futuros, mas não
// grava command buffers nem consome GPU no runtime atual.
constexpr bool SceneSsrEnabled = false;
// Historico de SSR (ver ssr_trace_resolve.frag) - meia resolucao linear
// (1/4 dos pixels, ver ensureScene3DTarget), entao mesmo em ponto flutuante
// de meia precisao custa menos banda por quadro que o buffer de vetores de
// movimento em resolucao cheia acima. Precisa ser float (nao UNORM): a
// radiancia refletida pode superar 1.0 (brilho do sol refletido).
constexpr VkFormat SceneSsrHistoryFormat = VK_FORMAT_R16G16B16A16_SFLOAT;

using Matrix4Values = std::array<float, 16>;

[[nodiscard]] float& matrixElement(
    Matrix4Values& matrix, std::size_t row, std::size_t column) {
    return matrix[column * 4 + row];
}

[[nodiscard]] float matrixElement(
    const Matrix4Values& matrix, std::size_t row, std::size_t column) {
    return matrix[column * 4 + row];
}

[[nodiscard]] Matrix4Values identityMatrix() {
    Matrix4Values result {};
    for (std::size_t index = 0; index < 4; ++index) {
        matrixElement(result, index, index) = 1.0f;
    }
    return result;
}

[[nodiscard]] Matrix4Values multiplyMatrices(
    const Matrix4Values& left, const Matrix4Values& right) {
    Matrix4Values result {};
    for (std::size_t row = 0; row < 4; ++row) {
        for (std::size_t column = 0; column < 4; ++column) {
            for (std::size_t inner = 0; inner < 4; ++inner) {
                matrixElement(result, row, column) +=
                    matrixElement(left, row, inner)
                    * matrixElement(right, inner, column);
            }
        }
    }
    return result;
}

[[nodiscard]] Matrix4Values inverseMatrix(const Matrix4Values& matrix) {
    float augmented[4][8] {};
    for (std::size_t row = 0; row < 4; ++row) {
        for (std::size_t column = 0; column < 4; ++column) {
            augmented[row][column] =
                matrixElement(matrix, row, column);
        }
        augmented[row][row + 4] = 1.0f;
    }

    for (std::size_t column = 0; column < 4; ++column) {
        std::size_t pivotRow = column;
        for (std::size_t row = column + 1; row < 4; ++row) {
            if (std::abs(augmented[row][column])
                > std::abs(augmented[pivotRow][column])) {
                pivotRow = row;
            }
        }
        if (std::abs(augmented[pivotRow][column]) <= 0.0000001f) {
            throw std::invalid_argument(
                "Cannot invert a singular scene matrix");
        }
        if (pivotRow != column) {
            for (std::size_t entry = 0; entry < 8; ++entry) {
                std::swap(augmented[column][entry],
                    augmented[pivotRow][entry]);
            }
        }

        const float pivot = augmented[column][column];
        for (float& entry : augmented[column]) {
            entry /= pivot;
        }
        for (std::size_t row = 0; row < 4; ++row) {
            if (row == column) {
                continue;
            }
            const float factor = augmented[row][column];
            for (std::size_t entry = 0; entry < 8; ++entry) {
                augmented[row][entry] -=
                    factor * augmented[column][entry];
            }
        }
    }

    Matrix4Values result {};
    for (std::size_t row = 0; row < 4; ++row) {
        for (std::size_t column = 0; column < 4; ++column) {
            matrixElement(result, row, column) =
                augmented[row][column + 4];
        }
    }
    return result;
}

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
    // x=amostras PCSS (zero desliga sombras),
    // y=quantidade de luzes no SSBO SceneLights,
    // z=historico temporal valido neste quadro, w=luz ambiente.
    std::array<float, 4> settings {};
    // x=mostrar ceu, y=tempo do ceu, z=cobertura de nuvens,
    // w=transmitância solar local através das nuvens.
    std::array<float, 4> skySettings {};
    // x=densidade (taxa exponencial por metro), y=acoplamento altura-distancia,
    // z=opacidade maxima, w=distancia inicial.
    std::array<float, 4> fogSettings {};
    // rgb=cor dinâmica da neblina, a=distancia de ocultação total.
    std::array<float, 4> fogColor {};
    // xy=deslocamento acumulado do vento nas nuvens (unidades de ruido do
    // shader do ceu, ja integrado no tempo pelo lado CPU - ver
    // EnvironmentLightingState3D::cloudWindOffset/WindSystem), zw reservado.
    std::array<float, 4> windOffset {};
    // x=visibilidade indireta mínima em regiões totalmente ocluídas,
    // y=força do primeiro rebote solar difuso, z=precipitação,
    // w=umidade acumulada das superfícies.
    std::array<float, 4> environmentSettings {};
    // x=modo Pixel Art, y=tamanho base do pixel artístico,
    // z=TAA completo opcional, w reservado.
    std::array<float, 4> renderSettings {};
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

static_assert(sizeof(SceneUniformGpu) == 688);
static_assert(sizeof(SceneMeshInstanceGpu) == 144);

// Espelha o bloco "push_constant" de tonemap.frag campo a campo - ver
// ToneMappingSettings3D (Scene3D.hpp) para o que cada campo faz.
struct TonemapPushConstantsGpu {
    float exposure = 1.0f;
    float brightness = 0.0f;
    float contrast = 1.0f;
    float saturation = 1.0f;
    float oceanSubmersion = 0.0f;
    float animationTimeSeconds = 0.0f;
    float pixelArtEnabled = 0.0f;
    float luminanceLevelCount = 12.0f;
    float chromaLevelCount = 18.0f;
    float ditherStrength = 0.025f;
    // Mantém os offsets do push constant estáveis; o antigo contorno
    // artificial de silhueta foi removido do Pixel Art.
    float reservedPixelArt0 = 0.0f;
    float pixelGridHeight = 540.0f;
    float worldPixelation = 1.0f;
    float physicalPropPixelation = 1.0f;
    float distanceLodStrength = 0.65f;
    float distanceLodStartMeters = 12.0f;
    float distanceLodEndMeters = 110.0f;
    float flatteningStrength = 1.0f;
};

static_assert(sizeof(TonemapPushConstantsGpu) == 72);

struct AutoExposurePushConstantsGpu {
    float minimumExposure = 0.75f;
    float maximumExposure = 2.50f;
    float meteringKey = 0.35f;
    float deltaTime = 1.0f / 60.0f;
    float brightAdaptationSpeed = 7.0f;
    float darkAdaptationSpeed = 2.4f;
    float historyValid = 0.0f;
    float enabled = 0.0f;
};

static_assert(sizeof(AutoExposurePushConstantsGpu) == 32);

struct GtaoPushConstantsGpu {
    float radiusMeters = 1.15f;
    float strength = 0.72f;
    float maxDarkening = 0.24f;
    float normalBias = 0.055f;
    std::uint32_t sampleDirectionCount = 6;
    std::uint32_t sampleStepCount = 2;
    std::array<std::uint32_t, 2> reserved {};
};

static_assert(sizeof(GtaoPushConstantsGpu) == 32);

struct BloomGlarePushConstantsGpu {
    float glareStrength = 0.24f;
    float flareStrength = 0.055f;
    float reserved0 = 0.0f;
    float reserved1 = 0.0f;
    std::array<float, 2> sunUv { 0.5f, 0.5f };
    float sunOnScreen = 0.0f;
    float enabled = 0.0f;
};

static_assert(sizeof(BloomGlarePushConstantsGpu) == 32);

struct OceanPushConstantsGpu {
    std::array<float, 2> meshOrigin { 0.0f, 0.0f };
    float meanSeaLevel = 0.0f;
    float timeSeconds = 0.0f;
    std::array<float, 2> worldCenter { 0.0f, 0.0f };
    float worldHalfExtent = 1500.0f;
    float reserved = 0.0f;
};

static_assert(sizeof(OceanPushConstantsGpu) == 32);

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
        VkFence fence = VK_NULL_HANDLE;
        bool timestampWritten = false;
        bool sceneTimestampsWritten = false;
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
            std::array<std::uint64_t, FrameTimestampCount> timestamps {};
            const std::uint32_t firstQuery =
                currentFrame * FrameTimestampCount;
            const VkResult timingResult = vkGetQueryPoolResults(device,
                frameTimestampQueryPool, firstQuery, FrameTimestampCount,
                sizeof(timestamps), timestamps.data(),
                sizeof(std::uint64_t), VK_QUERY_RESULT_64_BIT);
            if (timingResult == VK_SUCCESS
                && std::is_sorted(timestamps.begin(), timestamps.end())) {
                const auto milliseconds = [&](std::size_t begin,
                    std::size_t end) {
                    return static_cast<float>(
                        timestamps[end] - timestamps[begin])
                        * timestampPeriodNanoseconds / 1'000'000.0f;
                };
                frameMetrics.gpuFrameMilliseconds = milliseconds(0, 9);
                frameMetrics.gpuShadowMilliseconds = milliseconds(0, 1);
                frameMetrics.gpuDepthPrepassMilliseconds =
                    milliseconds(1, 2);
                frameMetrics.gpuOpaqueMilliseconds = milliseconds(2, 3);
                frameMetrics.gpuOceanMilliseconds = milliseconds(3, 4);
                frameMetrics.gpuTemporalMilliseconds = milliseconds(4, 5);
                frameMetrics.gpuBloomGlareMilliseconds =
                    milliseconds(5, 6);
                frameMetrics.gpuExposureMilliseconds =
                    milliseconds(6, 7);
                frameMetrics.gpuTonemapMilliseconds =
                    milliseconds(7, 8);
                frameMetrics.gpuPostProcessMilliseconds =
                    milliseconds(5, 8);
                frameMetrics.gpuUiMilliseconds = milliseconds(8, 9);
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
        const std::uint32_t firstQuery =
            currentFrame * FrameTimestampCount;
        vkCmdResetQueryPool(frame.commandBuffer, frameTimestampQueryPool,
            firstQuery, FrameTimestampCount);
        vkCmdWriteTimestamp2(frame.commandBuffer,
            VK_PIPELINE_STAGE_2_TOP_OF_PIPE_BIT,
            frameTimestampQueryPool, firstQuery);
        frame.sceneTimestampsWritten = false;

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
        VkDescriptorSet descriptorSet = spriteDescriptorSets[currentFrame * 4 + descriptorSlot];

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
        if (descriptorSlot >= 4) {
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
        const auto writeSceneTimestamp = [&](std::uint32_t index) {
            if (!directToSwapchain) return;
            vkCmdWriteTimestamp2(frame.commandBuffer,
                VK_PIPELINE_STAGE_2_ALL_GRAPHICS_BIT,
                frameTimestampQueryPool,
                currentFrame * FrameTimestampCount + index);
        };
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
            inverseMatrix(scene.cameraViewProjection.values);
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
            scene.showShadows
                ? static_cast<float>(std::clamp(
                    scene.shadowFilterSampleCount, 4u, 16u))
                : 0.0f,
            static_cast<float>(scene.lights.size()),
            useTemporalHistory ? 1.0f : 0.0f,
            scene.environment.skyIrradiance
        };
        uniform.skySettings = {
            scene.showSky ? 1.0f : 0.0f,
            scene.environment.skyAnimationTime,
            scene.environment.cloudCoverage,
            scene.environment.cloudSunTransmittance
        };
        uniform.fogSettings = {
            scene.fog.density, scene.fog.heightFalloff, scene.fog.maxOpacity,
            scene.fog.startDistanceMeters
        };
        uniform.fogColor = {
            scene.fog.color.x, scene.fog.color.y, scene.fog.color.z,
            std::max(scene.fog.endDistanceMeters,
                scene.fog.startDistanceMeters + 1.0f)
        };
        uniform.windOffset = {
            scene.environment.cloudWindOffset.x,
            scene.environment.cloudWindOffset.y,
            scene.ambientOcclusion.enabled ? 1.0f : 0.0f, 0.0f
        };
        uniform.environmentSettings = {
            scene.environment.minimumIndirectVisibility,
            scene.environment.sunDiffuseBounce,
            scene.environment.precipitation,
            scene.environment.surfaceWetness
        };
        const float artisticPixelSize =
            scene.renderMode == SceneRenderMode3D::PixelArt
            ? std::max(std::round(static_cast<float>(extent.height)
                / std::max(scene.pixelArt.pixelGridHeight, 1.0f)), 1.0f)
            : 1.0f;
        uniform.renderSettings = {
            scene.renderMode == SceneRenderMode3D::PixelArt ? 1.0f : 0.0f,
            artisticPixelSize,
            scene.pixelArt.temporalAntiAliasingEnabled ? 1.0f : 0.0f,
            0.0f
        };
        std::memcpy(sceneUniformMapped[currentFrame], &uniform, sizeof(uniform));
        check(vmaFlushAllocation(allocator, sceneUniformAllocations[currentFrame],
            0, sizeof(uniform)), "vmaFlushAllocation(scene uniform)");

        SceneUniformGpu planarUniform = uniform;
        if (scene.planarReflectionEnabled
            && scene.planarReflectionNormal.lengthSquared() > 0.000001f) {
            const Vec3 normal =
                scene.planarReflectionNormal.normalized();
            const float planeDistance =
                dot(normal, scene.planarReflectionPoint);
            const std::array<float, 3> n {
                normal.x, normal.y, normal.z
            };
            Matrix4Values reflection = identityMatrix();
            for (std::size_t row = 0; row < 3; ++row) {
                for (std::size_t column = 0; column < 3; ++column) {
                    matrixElement(reflection, row, column) -=
                        2.0f * n[row] * n[column];
                }
                matrixElement(reflection, row, 3) =
                    2.0f * planeDistance * n[row];
            }
            const Matrix4Values reflectedViewProjection =
                multiplyMatrices(scene.cameraViewProjection.values,
                    reflection);
            const Vec3 reflectedCamera = scene.cameraPosition
                - normal * (2.0f
                    * (dot(normal, scene.cameraPosition)
                        - planeDistance));
            // Sem jitter e sem histórico: o resultado planar já passa pelo
            // TAA da câmera principal quando é amostrado pelo espelho.
            planarUniform.cameraViewProjection =
                reflectedViewProjection;
            planarUniform.cameraViewProjectionUnjittered =
                reflectedViewProjection;
            planarUniform.inverseCameraViewProjection =
                inverseMatrix(reflectedViewProjection);
            planarUniform.previousCameraViewProjection =
                reflectedViewProjection;
            planarUniform.cameraPosition = {
                reflectedCamera.x, reflectedCamera.y,
                reflectedCamera.z, 1.0f
            };
            // Os mapas de sombra foram calculados para a câmera principal;
            // reutilizá-los daqui produziria sombras em cascatas erradas.
            planarUniform.settings[0] = 0.0f;
            planarUniform.settings[2] = 0.0f;
        }
        std::memcpy(scenePlanarCameraUniformMapped[currentFrame],
            &planarUniform, sizeof(planarUniform));
        check(vmaFlushAllocation(allocator,
            scenePlanarCameraUniformAllocations[currentFrame], 0,
            sizeof(planarUniform)),
            "vmaFlushAllocation(scene planar camera uniform)");

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
                mesh->shadowIndexBuffer.index,
                mesh->shadowIndexBuffer.generation,
                mesh->shadowIndexCount,
                mesh->albedoTexture.index, mesh->albedoTexture.generation,
                mesh->metallicRoughnessTexture.index,
                mesh->metallicRoughnessTexture.generation,
                mesh->castsShadow, mesh->visibleInCamera,
                mesh->planarReflection,
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
        std::vector<Mat4> skinMatrices { Mat4::identity() };
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
            if (!mesh->skinMatrices.empty()) {
                gpuInstance.orientationX[3] = static_cast<float>(skinMatrices.size());
                skinMatrices.insert(skinMatrices.end(),
                    mesh->skinMatrices.begin(), mesh->skinMatrices.end());
                gpuInstance.orientationY[3] = static_cast<float>(skinMatrices.size());
                const auto previous = mesh->previousSkinMatrices.size()
                    == mesh->skinMatrices.size()
                    ? mesh->previousSkinMatrices : mesh->skinMatrices;
                skinMatrices.insert(skinMatrices.end(), previous.begin(), previous.end());
            }
            const float materialFlags =
                (mesh->outlineGlow ? 1.0f : 0.0f)
                + (mesh->planarReflection ? 2.0f : 0.0f)
                + (mesh->pixelArtHighDetail ? 4.0f : 0.0f)
                + (mesh->matteSurface ? 8.0f : 0.0f)
                + (mesh->flatShaded ? 16.0f : 0.0f);
            gpuInstance.materialAndFlags = { mesh->metallic, mesh->roughness,
                mesh->selected ? 1.0f : 0.0f, materialFlags };
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
        ensureSceneSkinCapacity(skinMatrices.size());
        const auto skinBytes = skinMatrices.size() * sizeof(Mat4);
        std::memcpy(sceneSkinMapped[currentFrame], skinMatrices.data(), skinBytes);
        check(vmaFlushAllocation(allocator, sceneSkinAllocations[currentFrame],
            0, skinBytes), "vmaFlushAllocation(scene skin)");
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
            bool bindMaterial, bool shadowGeometry = false) {
            const MeshRender3D& meshObject = *batch.prototype;
            BufferResource& vertexBuffer =
                checkedResource(buffers, meshObject.vertexBuffer, "mesh vertex buffer");
            const RHI::BufferHandle selectedIndexHandle =
                shadowGeometry && meshObject.shadowIndexBuffer.valid()
                    ? meshObject.shadowIndexBuffer : meshObject.indexBuffer;
            const std::uint32_t selectedIndexCount =
                shadowGeometry && meshObject.shadowIndexBuffer.valid()
                    ? meshObject.shadowIndexCount : meshObject.indexCount;
            BufferResource& indexBuffer =
                checkedResource(buffers, selectedIndexHandle, "mesh index buffer");
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
            vkCmdDrawIndexed(frame.commandBuffer, selectedIndexCount,
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
            const std::uint32_t shadowMapSize =
                ShadowCascadeMapSizes[cascade];
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
            shadowRendering.renderArea.extent = {
                shadowMapSize, shadowMapSize
            };
            shadowRendering.layerCount = 1;
            shadowRendering.pDepthAttachment = &shadowDepthAttachment;
            vkCmdBeginRendering(frame.commandBuffer, &shadowRendering);
            setViewportAndScissor({ shadowMapSize, shadowMapSize });
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
                        pushMeshBatch(batch, false, true);
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
        writeSceneTimestamp(1);
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

        // Recursos legados do experimento de SSR. Não entram mais no MRT
        // ativo; ficam disponíveis somente no bloco morto SceneSsrEnabled.
        VkImage normalRoughnessImage = directToSwapchain
            ? sceneDirectNormalRoughnessImage : sceneNormalRoughnessImage;
        VkImageView normalRoughnessView = directToSwapchain
            ? sceneDirectNormalRoughnessView : sceneNormalRoughnessView;
        SceneAttachmentState3D& normalRoughnessState = directToSwapchain
            ? sceneDirectNormalRoughnessState : sceneNormalRoughnessState;
        // Recursos legados mantidos apenas para que o experimento de SSR
        // continue compilável. Com SceneSsrEnabled=false eles não entram no
        // MRT nem recebem transições/comandos durante o frame.
        VkImage reflectanceImage = directToSwapchain
            ? sceneDirectReflectanceImage : sceneReflectanceImage;
        VkImageView reflectanceView = directToSwapchain
            ? sceneDirectReflectanceView : sceneReflectanceView;
        SceneAttachmentState3D& reflectanceState = directToSwapchain
            ? sceneDirectReflectanceState : sceneReflectanceState;
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

        VkImage planarReflectionImage = directToSwapchain
            ? sceneDirectPlanarReflectionImage
            : scenePlanarReflectionImage;
        VkImageView planarReflectionView = directToSwapchain
            ? sceneDirectPlanarReflectionView
            : scenePlanarReflectionView;
        SceneAttachmentState3D& planarReflectionState =
            directToSwapchain ? sceneDirectPlanarReflectionState
                : scenePlanarReflectionState;
        const VkExtent2D planarExtent {
            std::max(1u, renderExtent.width / 2),
            std::max(1u, renderExtent.height / 2)
        };

        if (scene.planarReflectionEnabled) {
            transitionSceneAttachment(frame.commandBuffer,
                planarReflectionImage, VK_IMAGE_ASPECT_COLOR_BIT,
                planarReflectionState,
                VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL,
                VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT,
                VK_ACCESS_2_COLOR_ATTACHMENT_WRITE_BIT);

            VkClearValue planarDepthClear {};
            planarDepthClear.depthStencil.depth = 0.0f;
            VkRenderingAttachmentInfo planarDepthAttachment {
                VK_STRUCTURE_TYPE_RENDERING_ATTACHMENT_INFO
            };
            planarDepthAttachment.imageView = depthView;
            planarDepthAttachment.imageLayout =
                VK_IMAGE_LAYOUT_DEPTH_ATTACHMENT_OPTIMAL;
            planarDepthAttachment.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
            planarDepthAttachment.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
            planarDepthAttachment.clearValue = planarDepthClear;
            VkRenderingInfo planarDepthRendering {
                VK_STRUCTURE_TYPE_RENDERING_INFO
            };
            planarDepthRendering.renderArea.extent = planarExtent;
            planarDepthRendering.layerCount = 1;
            planarDepthRendering.pDepthAttachment =
                &planarDepthAttachment;
            vkCmdBeginRendering(frame.commandBuffer,
                &planarDepthRendering);
            setViewportAndScissor(planarExtent);
            if (!scene.meshes.empty()) {
                vkCmdBindPipeline(frame.commandBuffer,
                    VK_PIPELINE_BIND_POINT_GRAPHICS,
                    sceneMeshDepthPrepassPipeline);
                vkCmdBindDescriptorSets(frame.commandBuffer,
                    VK_PIPELINE_BIND_POINT_GRAPHICS,
                    scenePipelineLayout, 0, 1,
                    &scenePlanarCameraDescriptorSets[currentFrame],
                    0, nullptr);
                for (const MeshBatch& batch : meshBatches) {
                    if (!batch.prototype->planarReflection) {
                        pushMeshBatch(batch, false);
                    }
                }
            }
            vkCmdEndRendering(frame.commandBuffer);

            transitionSceneAttachment(frame.commandBuffer, depthImage,
                VK_IMAGE_ASPECT_DEPTH_BIT, depthState,
                VK_IMAGE_LAYOUT_DEPTH_ATTACHMENT_OPTIMAL,
                VK_PIPELINE_STAGE_2_EARLY_FRAGMENT_TESTS_BIT
                    | VK_PIPELINE_STAGE_2_LATE_FRAGMENT_TESTS_BIT,
                VK_ACCESS_2_DEPTH_STENCIL_ATTACHMENT_READ_BIT
                    | VK_ACCESS_2_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT);

            VkClearValue planarColorClear {};
            planarColorClear.color.float32[0] = 0.025f;
            planarColorClear.color.float32[1] = 0.040f;
            planarColorClear.color.float32[2] = 0.055f;
            planarColorClear.color.float32[3] = 1.0f;
            VkClearValue planarAuxClear {};
            std::array<VkRenderingAttachmentInfo, 2>
                planarColorAttachments {};
            for (VkRenderingAttachmentInfo& attachment :
                    planarColorAttachments) {
                attachment.sType =
                    VK_STRUCTURE_TYPE_RENDERING_ATTACHMENT_INFO;
                attachment.imageLayout =
                    VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
                attachment.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
                attachment.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
            }
            planarColorAttachments[0].imageView =
                planarReflectionView;
            planarColorAttachments[0].clearValue =
                planarColorClear;
            planarColorAttachments[1].imageView = motionVectorView;
            planarColorAttachments[1].clearValue = planarAuxClear;

            VkRenderingAttachmentInfo planarColorDepth {
                VK_STRUCTURE_TYPE_RENDERING_ATTACHMENT_INFO
            };
            planarColorDepth.imageView = depthView;
            planarColorDepth.imageLayout =
                VK_IMAGE_LAYOUT_DEPTH_ATTACHMENT_OPTIMAL;
            planarColorDepth.loadOp = VK_ATTACHMENT_LOAD_OP_LOAD;
            planarColorDepth.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
            VkRenderingInfo planarRendering {
                VK_STRUCTURE_TYPE_RENDERING_INFO
            };
            planarRendering.renderArea.extent = planarExtent;
            planarRendering.layerCount = 1;
            planarRendering.colorAttachmentCount =
                static_cast<std::uint32_t>(
                    planarColorAttachments.size());
            planarRendering.pColorAttachments =
                planarColorAttachments.data();
            planarRendering.pDepthAttachment = &planarColorDepth;
            vkCmdBeginRendering(frame.commandBuffer, &planarRendering);
            setViewportAndScissor(planarExtent);
            if (scene.showSky) {
                vkCmdBindPipeline(frame.commandBuffer,
                    VK_PIPELINE_BIND_POINT_GRAPHICS,
                    sceneSkyPipeline);
                vkCmdBindDescriptorSets(frame.commandBuffer,
                    VK_PIPELINE_BIND_POINT_GRAPHICS,
                    scenePipelineLayout, 0, 1,
                    &scenePlanarCameraDescriptorSets[currentFrame],
                    0, nullptr);
                vkCmdDraw(frame.commandBuffer, 3, 1, 0, 0);
            }
            if (!scene.meshes.empty()) {
                vkCmdBindPipeline(frame.commandBuffer,
                    VK_PIPELINE_BIND_POINT_GRAPHICS,
                    sceneMeshPipeline);
                vkCmdBindDescriptorSets(frame.commandBuffer,
                    VK_PIPELINE_BIND_POINT_GRAPHICS,
                    sceneMeshPipelineLayout, 0, 1,
                    &scenePlanarCameraDescriptorSets[currentFrame],
                    0, nullptr);
                const TextureResource& fallbackReflection =
                    checkedResource(textures, defaultMaterialTexture,
                        "default planar reflection texture");
                vkCmdBindDescriptorSets(frame.commandBuffer,
                    VK_PIPELINE_BIND_POINT_GRAPHICS,
                    sceneMeshPipelineLayout, 3, 1,
                    &fallbackReflection.materialDescriptor,
                    0, nullptr);
                for (const MeshBatch& batch : meshBatches) {
                    if (!batch.prototype->planarReflection) {
                        pushMeshBatch(batch, true);
                    }
                }
            }
            vkCmdEndRendering(frame.commandBuffer);

            transitionSceneAttachment(frame.commandBuffer,
                planarReflectionImage, VK_IMAGE_ASPECT_COLOR_BIT,
                planarReflectionState,
                VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
                VK_PIPELINE_STAGE_2_FRAGMENT_SHADER_BIT,
                VK_ACCESS_2_SHADER_SAMPLED_READ_BIT);

            // Os attachments auxiliares e o depth serão sobrescritos pela
            // câmera principal. Barreiras explícitas separam os dois usos,
            // mesmo mantendo o mesmo layout.
            transitionSceneAttachment(frame.commandBuffer, depthImage,
                VK_IMAGE_ASPECT_DEPTH_BIT, depthState,
                VK_IMAGE_LAYOUT_DEPTH_ATTACHMENT_OPTIMAL,
                VK_PIPELINE_STAGE_2_EARLY_FRAGMENT_TESTS_BIT
                    | VK_PIPELINE_STAGE_2_LATE_FRAGMENT_TESTS_BIT,
                VK_ACCESS_2_DEPTH_STENCIL_ATTACHMENT_READ_BIT
                    | VK_ACCESS_2_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT);
            transitionSceneAttachment(frame.commandBuffer,
                motionVectorImage, VK_IMAGE_ASPECT_COLOR_BIT,
                motionVectorState,
                VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL,
                VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT,
                VK_ACCESS_2_COLOR_ATTACHMENT_WRITE_BIT);
        } else {
            transitionSceneAttachment(frame.commandBuffer,
                planarReflectionImage, VK_IMAGE_ASPECT_COLOR_BIT,
                planarReflectionState,
                VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
                VK_PIPELINE_STAGE_2_FRAGMENT_SHADER_BIT,
                VK_ACCESS_2_SHADER_SAMPLED_READ_BIT);
        }

        // Pre-pass de profundidade: resolve o buffer inteiro antes do passe
        // opaco. O fragment shader principal (BRDF + sombras) roda apenas
        // para a superficie que realmente ficou visivel.
        VkClearValue depthPrepassClear {};
        depthPrepassClear.depthStencil.depth = 0.0f;
        VkRenderingAttachmentInfo depthPrepassAttachment {
            VK_STRUCTURE_TYPE_RENDERING_ATTACHMENT_INFO
        };
        depthPrepassAttachment.imageView = depthView;
        depthPrepassAttachment.imageLayout =
            VK_IMAGE_LAYOUT_DEPTH_ATTACHMENT_OPTIMAL;
        depthPrepassAttachment.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
        depthPrepassAttachment.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
        depthPrepassAttachment.clearValue = depthPrepassClear;
        VkRenderingInfo depthPrepassRendering {
            VK_STRUCTURE_TYPE_RENDERING_INFO
        };
        depthPrepassRendering.renderArea.extent = renderExtent;
        depthPrepassRendering.layerCount = 1;
        depthPrepassRendering.pDepthAttachment = &depthPrepassAttachment;
        vkCmdBeginRendering(frame.commandBuffer, &depthPrepassRendering);
        setViewportAndScissor(renderExtent);
        if (!scene.meshes.empty()) {
            vkCmdBindPipeline(frame.commandBuffer,
                VK_PIPELINE_BIND_POINT_GRAPHICS,
                sceneMeshDepthPrepassPipeline);
            vkCmdBindDescriptorSets(frame.commandBuffer,
                VK_PIPELINE_BIND_POINT_GRAPHICS,
                scenePipelineLayout, 0, 1,
                &sceneDescriptorSets[currentFrame], 0, nullptr);
            for (const MeshBatch& batch : meshBatches) {
                if (batch.prototype->visibleInCamera) {
                    pushMeshBatch(batch, false);
                }
            }
        }
        vkCmdEndRendering(frame.commandBuffer);

        // Dependencia explicita entre a escrita do pre-pass e os testes do
        // passe de cor seguinte.
        transitionSceneAttachment(frame.commandBuffer, depthImage,
            VK_IMAGE_ASPECT_DEPTH_BIT, depthState,
            VK_IMAGE_LAYOUT_DEPTH_ATTACHMENT_OPTIMAL,
            VK_PIPELINE_STAGE_2_EARLY_FRAGMENT_TESTS_BIT
                | VK_PIPELINE_STAGE_2_LATE_FRAGMENT_TESTS_BIT,
            VK_ACCESS_2_DEPTH_STENCIL_ATTACHMENT_READ_BIT
                | VK_ACCESS_2_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT);
        writeSceneTimestamp(2);

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
        // A profundidade ja foi resolvida pelo pre-pass acima.
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
            const VkDescriptorSet planarDescriptorSet =
                directToSwapchain
                    ? sceneDirectPlanarReflectionDescriptorSets[
                        currentFrame]
                    : scenePlanarReflectionDescriptorSets[currentFrame];
            vkCmdBindDescriptorSets(frame.commandBuffer,
                VK_PIPELINE_BIND_POINT_GRAPHICS,
                sceneMeshPipelineLayout, 3, 1,
                &planarDescriptorSet, 0, nullptr);
            for (const MeshBatch& batch : meshBatches) {
                if (batch.prototype->visibleInCamera) {
                    pushMeshBatch(batch, true);
                }
            }
        }
        vkCmdEndRendering(frame.commandBuffer);
        writeSceneTimestamp(3);

        // Fase 6 (TAA): resolve temporal - le a cor HDR recem-preenchida
        // acima, a profundidade do pre-pass e o historico resolvido
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

        // Superfície oceânica procedural: um passe transparente antes do TAA.
        // A profundidade opaca recorta a costa e fornece espessura óptica.
        if (scene.oceanEnabled && scene.ocean.vertexBuffer.valid()
                && scene.ocean.indexBuffer.valid()
                && scene.ocean.indexCount > 0) {
            BufferResource& oceanVertexBuffer = checkedResource(buffers,
                scene.ocean.vertexBuffer, "ocean clipmap vertex buffer");
            BufferResource& oceanIndexBuffer = checkedResource(buffers,
                scene.ocean.indexBuffer, "ocean clipmap index buffer");

            transitionSceneAttachment(frame.commandBuffer, hdrColorImage,
                VK_IMAGE_ASPECT_COLOR_BIT, hdrColorState,
                VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL,
                VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT,
                VK_ACCESS_2_COLOR_ATTACHMENT_WRITE_BIT);
            transitionSceneAttachment(frame.commandBuffer,
                motionVectorImage, VK_IMAGE_ASPECT_COLOR_BIT,
                motionVectorState,
                VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL,
                VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT,
                VK_ACCESS_2_COLOR_ATTACHMENT_WRITE_BIT);

            VkRenderingAttachmentInfo oceanColorAttachment {
                VK_STRUCTURE_TYPE_RENDERING_ATTACHMENT_INFO
            };
            oceanColorAttachment.imageView = hdrColorView;
            oceanColorAttachment.imageLayout =
                VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
            oceanColorAttachment.loadOp = VK_ATTACHMENT_LOAD_OP_LOAD;
            oceanColorAttachment.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
            VkRenderingAttachmentInfo oceanMotionAttachment {
                VK_STRUCTURE_TYPE_RENDERING_ATTACHMENT_INFO
            };
            oceanMotionAttachment.imageView = motionVectorView;
            oceanMotionAttachment.imageLayout =
                VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
            oceanMotionAttachment.loadOp = VK_ATTACHMENT_LOAD_OP_LOAD;
            oceanMotionAttachment.storeOp = VK_ATTACHMENT_STORE_OP_STORE;

            VkRenderingAttachmentInfo oceanDepthAttachment {
                VK_STRUCTURE_TYPE_RENDERING_ATTACHMENT_INFO
            };
            oceanDepthAttachment.imageView = depthView;
            oceanDepthAttachment.imageLayout =
                VK_IMAGE_LAYOUT_DEPTH_STENCIL_READ_ONLY_OPTIMAL;
            oceanDepthAttachment.loadOp = VK_ATTACHMENT_LOAD_OP_LOAD;
            oceanDepthAttachment.storeOp = VK_ATTACHMENT_STORE_OP_NONE;

            VkRenderingInfo oceanRenderingInfo {
                VK_STRUCTURE_TYPE_RENDERING_INFO
            };
            oceanRenderingInfo.renderArea.extent = renderExtent;
            oceanRenderingInfo.layerCount = 1;
            const std::array<VkRenderingAttachmentInfo, 2>
                oceanColorAttachments {
                    oceanColorAttachment, oceanMotionAttachment
                };
            oceanRenderingInfo.colorAttachmentCount =
                static_cast<std::uint32_t>(oceanColorAttachments.size());
            oceanRenderingInfo.pColorAttachments =
                oceanColorAttachments.data();
            oceanRenderingInfo.pDepthAttachment = &oceanDepthAttachment;
            vkCmdBeginRendering(frame.commandBuffer, &oceanRenderingInfo);
            setViewportAndScissor(renderExtent);
            vkCmdBindPipeline(frame.commandBuffer,
                VK_PIPELINE_BIND_POINT_GRAPHICS, sceneOceanPipeline);
            const VkDescriptorSet oceanDescriptorSet = directToSwapchain
                ? sceneDirectOceanDescriptorSets[currentFrame]
                : sceneOceanDescriptorSets[currentFrame];
            const std::array<VkDescriptorSet, 2> oceanSets {
                sceneDescriptorSets[currentFrame], oceanDescriptorSet
            };
            vkCmdBindDescriptorSets(frame.commandBuffer,
                VK_PIPELINE_BIND_POINT_GRAPHICS,
                sceneOceanPipelineLayout, 0,
                static_cast<std::uint32_t>(oceanSets.size()),
                oceanSets.data(), 0, nullptr);

            constexpr float FinestOceanCellMeters = 4.0f;
            OceanPushConstantsGpu oceanPush;
            oceanPush.meshOrigin = {
                std::floor(scene.cameraPosition.x / FinestOceanCellMeters)
                    * FinestOceanCellMeters,
                std::floor(scene.cameraPosition.y / FinestOceanCellMeters)
                    * FinestOceanCellMeters
            };
            oceanPush.meanSeaLevel = scene.ocean.meanSeaLevelMeters;
            oceanPush.timeSeconds = scene.environment.skyAnimationTime;
            oceanPush.worldCenter = {
                scene.ocean.center.x, scene.ocean.center.y
            };
            oceanPush.worldHalfExtent = scene.ocean.halfExtentMeters;
            vkCmdPushConstants(frame.commandBuffer, sceneOceanPipelineLayout,
                VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT, 0,
                sizeof(OceanPushConstantsGpu), &oceanPush);

            const VkDeviceSize oceanVertexOffset = 0;
            vkCmdBindVertexBuffers(frame.commandBuffer, 0, 1,
                &oceanVertexBuffer.buffer, &oceanVertexOffset);
            vkCmdBindIndexBuffer(frame.commandBuffer,
                oceanIndexBuffer.buffer, 0,
                VK_INDEX_TYPE_UINT32);
            vkCmdDrawIndexed(frame.commandBuffer,
                scene.ocean.indexCount, 1, 0, 0, 0);
            vkCmdEndRendering(frame.commandBuffer);

            transitionSceneAttachment(frame.commandBuffer, hdrColorImage,
                VK_IMAGE_ASPECT_COLOR_BIT, hdrColorState,
                VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
                VK_PIPELINE_STAGE_2_FRAGMENT_SHADER_BIT,
                VK_ACCESS_2_SHADER_SAMPLED_READ_BIT);
            transitionSceneAttachment(frame.commandBuffer,
                motionVectorImage, VK_IMAGE_ASPECT_COLOR_BIT,
                motionVectorState,
                VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
                VK_PIPELINE_STAGE_2_FRAGMENT_SHADER_BIT,
                VK_ACCESS_2_SHADER_SAMPLED_READ_BIT);
        }
        writeSceneTimestamp(4);

        // SSR (reflexos em espaco de tela, ver ssr_trace_resolve.frag) -
        // traça+acumula em MEIA resolucao, depois soma o resultado de volta
        // no HDR em resolucao cheia (ssr_composite.frag). Roda entre o passe
        // opaco e o resolve de TAA: precisa do HDR/profundidade/normal/
        // reflectancia ja prontos (acima) E precisa terminar de somar no HDR
        // ANTES do TAA resolver, senao o historico temporal nunca veria o
        // especular refletido.
        if (SceneSsrEnabled) {
        transitionSceneAttachment(frame.commandBuffer,
            normalRoughnessImage, VK_IMAGE_ASPECT_COLOR_BIT,
            normalRoughnessState,
            VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
            VK_PIPELINE_STAGE_2_FRAGMENT_SHADER_BIT,
            VK_ACCESS_2_SHADER_SAMPLED_READ_BIT);
        transitionSceneAttachment(frame.commandBuffer, reflectanceImage,
            VK_IMAGE_ASPECT_COLOR_BIT, reflectanceState,
            VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
            VK_PIPELINE_STAGE_2_FRAGMENT_SHADER_BIT,
            VK_ACCESS_2_SHADER_SAMPLED_READ_BIT);
        const VkExtent2D ssrExtent {
            std::max(1u, renderExtent.width / 2),
            std::max(1u, renderExtent.height / 2)
        };
        std::array<VkImage, FramesInFlight>& ssrHistoryImages = directToSwapchain
            ? sceneDirectSsrHistoryImage : sceneSsrHistoryImage;
        std::array<VkImageView, FramesInFlight>& ssrHistoryViews = directToSwapchain
            ? sceneDirectSsrHistoryView : sceneSsrHistoryView;
        std::array<SceneAttachmentState3D, FramesInFlight>& ssrHistoryStates =
            directToSwapchain ? sceneDirectSsrHistoryState : sceneSsrHistoryState;
        const std::uint32_t ssrHistoryReadIndex =
            (currentFrame + FramesInFlight - 1) % FramesInFlight;

        // Mesmo raciocinio do historico de TAA acima: redundante na maioria
        // dos quadros, necessario no primeiro uso de cada slot.
        transitionSceneAttachment(frame.commandBuffer,
            ssrHistoryImages[ssrHistoryReadIndex], VK_IMAGE_ASPECT_COLOR_BIT,
            ssrHistoryStates[ssrHistoryReadIndex],
            VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
            VK_PIPELINE_STAGE_2_FRAGMENT_SHADER_BIT,
            VK_ACCESS_2_SHADER_SAMPLED_READ_BIT);

        const VkDescriptorSet ssrTraceDescriptorSet = directToSwapchain
            ? sceneDirectSsrDescriptorSets[currentFrame]
            : sceneSsrDescriptorSets[currentFrame];

        transitionSceneAttachment(frame.commandBuffer,
            ssrHistoryImages[currentFrame], VK_IMAGE_ASPECT_COLOR_BIT,
            ssrHistoryStates[currentFrame],
            VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL,
            VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT,
            VK_ACCESS_2_COLOR_ATTACHMENT_WRITE_BIT);

        VkRenderingAttachmentInfo ssrTraceAttachment { VK_STRUCTURE_TYPE_RENDERING_ATTACHMENT_INFO };
        ssrTraceAttachment.imageView = ssrHistoryViews[currentFrame];
        ssrTraceAttachment.imageLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
        ssrTraceAttachment.loadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
        ssrTraceAttachment.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
        VkRenderingInfo ssrTraceRenderingInfo { VK_STRUCTURE_TYPE_RENDERING_INFO };
        ssrTraceRenderingInfo.renderArea.extent = ssrExtent;
        ssrTraceRenderingInfo.layerCount = 1;
        ssrTraceRenderingInfo.colorAttachmentCount = 1;
        ssrTraceRenderingInfo.pColorAttachments = &ssrTraceAttachment;
        vkCmdBeginRendering(frame.commandBuffer, &ssrTraceRenderingInfo);
        setViewportAndScissor(ssrExtent);
        vkCmdBindPipeline(frame.commandBuffer, VK_PIPELINE_BIND_POINT_GRAPHICS,
            sceneSsrPipeline);
        const std::array<VkDescriptorSet, 2> ssrTraceSets {
            sceneDescriptorSets[currentFrame], ssrTraceDescriptorSet
        };
        vkCmdBindDescriptorSets(frame.commandBuffer, VK_PIPELINE_BIND_POINT_GRAPHICS,
            sceneSsrPipelineLayout, 0,
            static_cast<std::uint32_t>(ssrTraceSets.size()), ssrTraceSets.data(),
            0, nullptr);
        vkCmdDraw(frame.commandBuffer, 3, 1, 0, 0);
        vkCmdEndRendering(frame.commandBuffer);

        transitionSceneAttachment(frame.commandBuffer,
            ssrHistoryImages[currentFrame], VK_IMAGE_ASPECT_COLOR_BIT,
            ssrHistoryStates[currentFrame],
            VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
            VK_PIPELINE_STAGE_2_FRAGMENT_SHADER_BIT,
            VK_ACCESS_2_SHADER_SAMPLED_READ_BIT);

        // Composicao: soma o resultado acima de volta no HDR em resolucao
        // cheia (blend ADITIVO, ver sceneSsrCompositePipeline). Precisa
        // transicionar o HDR de volta pra COLOR_ATTACHMENT_OPTIMAL - ele
        // acabou de ser lido como fonte pelo traçado acima - e devolve-lo
        // pra SHADER_READ_ONLY_OPTIMAL depois, porque o resolve de TAA logo
        // abaixo o le atraves do proprio descriptor set (ja vinculado
        // naquele layout).
        const VkDescriptorSet ssrCompositeDescriptorSet = directToSwapchain
            ? sceneDirectSsrCompositeDescriptorSets[currentFrame]
            : sceneSsrCompositeDescriptorSets[currentFrame];
        transitionSceneAttachment(frame.commandBuffer, hdrColorImage,
            VK_IMAGE_ASPECT_COLOR_BIT, hdrColorState,
            VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL,
            VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT,
            VK_ACCESS_2_COLOR_ATTACHMENT_WRITE_BIT);

        VkRenderingAttachmentInfo ssrCompositeAttachment { VK_STRUCTURE_TYPE_RENDERING_ATTACHMENT_INFO };
        ssrCompositeAttachment.imageView = hdrColorView;
        ssrCompositeAttachment.imageLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
        // LOAD (nao CLEAR): soma sobre o que o passe opaco ja escreveu
        // (difusa+direta, sem especular ambiente - ver scene3d_mesh.frag).
        ssrCompositeAttachment.loadOp = VK_ATTACHMENT_LOAD_OP_LOAD;
        ssrCompositeAttachment.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
        VkRenderingInfo ssrCompositeRenderingInfo { VK_STRUCTURE_TYPE_RENDERING_INFO };
        ssrCompositeRenderingInfo.renderArea.extent = renderExtent;
        ssrCompositeRenderingInfo.layerCount = 1;
        ssrCompositeRenderingInfo.colorAttachmentCount = 1;
        ssrCompositeRenderingInfo.pColorAttachments = &ssrCompositeAttachment;
        vkCmdBeginRendering(frame.commandBuffer, &ssrCompositeRenderingInfo);
        setViewportAndScissor(renderExtent);
        vkCmdBindPipeline(frame.commandBuffer, VK_PIPELINE_BIND_POINT_GRAPHICS,
            sceneSsrCompositePipeline);
        vkCmdBindDescriptorSets(frame.commandBuffer, VK_PIPELINE_BIND_POINT_GRAPHICS,
            sceneSsrCompositePipelineLayout, 0, 1, &ssrCompositeDescriptorSet,
            0, nullptr);
        vkCmdDraw(frame.commandBuffer, 3, 1, 0, 0);
        vkCmdEndRendering(frame.commandBuffer);

        transitionSceneAttachment(frame.commandBuffer, hdrColorImage,
            VK_IMAGE_ASPECT_COLOR_BIT, hdrColorState,
            VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
            VK_PIPELINE_STAGE_2_FRAGMENT_SHADER_BIT,
            VK_ACCESS_2_SHADER_SAMPLED_READ_BIT);
        }

        std::array<VkImage, FramesInFlight>& historyImages = directToSwapchain
            ? sceneDirectTaaHistoryImage : sceneTaaHistoryImage;
        std::array<VkImageView, FramesInFlight>& historyViews = directToSwapchain
            ? sceneDirectTaaHistoryView : sceneTaaHistoryView;
        std::array<SceneAttachmentState3D, FramesInFlight>& historyStates =
            directToSwapchain ? sceneDirectTaaHistoryState : sceneTaaHistoryState;
        const std::uint32_t historyReadIndex =
            (currentFrame + FramesInFlight - 1) % FramesInFlight;

        if (scene.temporalAntiAliasingEnabled) {
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
        temporalHistoryValid = true;
        } else {
            // O HDR opaco/oceano já está em SHADER_READ_ONLY. No perfil de
            // desempenho o tonemap o consome diretamente: sem cópia
            // fullscreen, histórico, motion-vector lookup ou barreira
            // adicional. Invalidar aqui garante uma semente limpa se o
            // usuário voltar a um perfil com TAA.
            temporalHistoryValid = false;
        }
        writeSceneTimestamp(5);

        // Passe de tonemap: le o resultado ja resolvido pelo TAA acima (nao
        // mais o HDR cru) e escreve o resultado LDR (exposicao + curva
        // filmica + sRGB, ver tonemap.frag) no destino final de verdade - a
        // imagem do swapchain no caminho direto, ou sceneColorImage
        // (amostrada pelo ImGui) no caminho de preview offscreen.
        if (scene.temporalAntiAliasingEnabled) {
            transitionSceneAttachment(frame.commandBuffer,
                historyImages[currentFrame],
                VK_IMAGE_ASPECT_COLOR_BIT, historyStates[currentFrame],
                VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
                VK_PIPELINE_STAGE_2_FRAGMENT_SHADER_BIT,
                VK_ACCESS_2_SHADER_SAMPLED_READ_BIT);
        }

        // Glare e ghosts solares em um único passe HDR a 1/8 de cada
        // dimensão. Sem bloom geral: são apenas 2 leituras de textura por
        // fragmento (HDR+depth na posição do Sol), em 1/64 dos pixels.
        const VkExtent2D bloomGlareExtent {
            std::max(1u, renderExtent.width / 8),
            std::max(1u, renderExtent.height / 8)
        };
        std::array<VkImage, FramesInFlight>& bloomGlareImages =
            directToSwapchain ? sceneDirectBloomGlareImage
                              : sceneBloomGlareImage;
        std::array<VkImageView, FramesInFlight>& bloomGlareViews =
            directToSwapchain ? sceneDirectBloomGlareView
                              : sceneBloomGlareView;
        std::array<SceneAttachmentState3D, FramesInFlight>&
            bloomGlareStates = directToSwapchain
                ? sceneDirectBloomGlareState : sceneBloomGlareState;
        const VkDescriptorSet bloomGlareDescriptorSet =
            directToSwapchain
                ? sceneDirectBloomGlareDescriptorSets[currentFrame]
                : sceneBloomGlareDescriptorSets[currentFrame];

        std::array<float, 2> sunUv { 0.5f, 0.5f };
        float sunOnScreen = 0.0f;
        if (scene.showSky && !scene.lights.empty()) {
            const Vec3 sunDirection =
                scene.lights.front().direction.normalized();
            const Vec3 sunPoint =
                scene.cameraPosition + sunDirection * 1000.0f;
            const Mat4& viewProjection = scene.cameraViewProjection;
            const float clipX = viewProjection.at(0, 0) * sunPoint.x
                + viewProjection.at(0, 1) * sunPoint.y
                + viewProjection.at(0, 2) * sunPoint.z
                + viewProjection.at(0, 3);
            const float clipY = viewProjection.at(1, 0) * sunPoint.x
                + viewProjection.at(1, 1) * sunPoint.y
                + viewProjection.at(1, 2) * sunPoint.z
                + viewProjection.at(1, 3);
            const float clipW = viewProjection.at(3, 0) * sunPoint.x
                + viewProjection.at(3, 1) * sunPoint.y
                + viewProjection.at(3, 2) * sunPoint.z
                + viewProjection.at(3, 3);
            if (clipW > 0.0001f) {
                sunUv[0] = clipX / clipW * 0.5f + 0.5f;
                // A viewport Vulkan inverte Y em relação ao clip calculado
                // pela matriz usada nos shaders da cena.
                sunUv[1] = -clipY / clipW * 0.5f + 0.5f;
                sunOnScreen = sunUv[0] >= 0.0f && sunUv[0] <= 1.0f
                        && sunUv[1] >= 0.0f && sunUv[1] <= 1.0f
                    ? 1.0f : 0.0f;
            }
        }

        transitionSceneAttachment(frame.commandBuffer,
            bloomGlareImages[currentFrame], VK_IMAGE_ASPECT_COLOR_BIT,
            bloomGlareStates[currentFrame],
            VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL,
            VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT,
            VK_ACCESS_2_COLOR_ATTACHMENT_WRITE_BIT);
        VkRenderingAttachmentInfo bloomGlareAttachment {
            VK_STRUCTURE_TYPE_RENDERING_ATTACHMENT_INFO
        };
        bloomGlareAttachment.imageView = bloomGlareViews[currentFrame];
        bloomGlareAttachment.imageLayout =
            VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
        bloomGlareAttachment.loadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
        bloomGlareAttachment.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
        VkRenderingInfo bloomGlareRenderingInfo {
            VK_STRUCTURE_TYPE_RENDERING_INFO
        };
        bloomGlareRenderingInfo.renderArea.extent = bloomGlareExtent;
        bloomGlareRenderingInfo.layerCount = 1;
        bloomGlareRenderingInfo.colorAttachmentCount = 1;
        bloomGlareRenderingInfo.pColorAttachments =
            &bloomGlareAttachment;
        vkCmdBeginRendering(frame.commandBuffer,
            &bloomGlareRenderingInfo);
        setViewportAndScissor(bloomGlareExtent);
        vkCmdBindPipeline(frame.commandBuffer,
            VK_PIPELINE_BIND_POINT_GRAPHICS, sceneBloomGlarePipeline);
        vkCmdBindDescriptorSets(frame.commandBuffer,
            VK_PIPELINE_BIND_POINT_GRAPHICS,
            sceneBloomGlarePipelineLayout, 0, 1,
            &bloomGlareDescriptorSet, 0, nullptr);
        const BloomGlarePushConstantsGpu bloomGlarePushData {
            std::max(scene.opticalEffects.sunGlareStrength, 0.0f),
            std::max(scene.opticalEffects.lensFlareStrength, 0.0f),
            0.0f,
            0.0f,
            sunUv,
            sunOnScreen,
            scene.opticalEffects.enabled ? 1.0f : 0.0f
        };
        vkCmdPushConstants(frame.commandBuffer,
            sceneBloomGlarePipelineLayout,
            VK_SHADER_STAGE_FRAGMENT_BIT, 0,
            sizeof(BloomGlarePushConstantsGpu),
            &bloomGlarePushData);
        vkCmdDraw(frame.commandBuffer, 3, 1, 0, 0);
        vkCmdEndRendering(frame.commandBuffer);
        transitionSceneAttachment(frame.commandBuffer,
            bloomGlareImages[currentFrame], VK_IMAGE_ASPECT_COLOR_BIT,
            bloomGlareStates[currentFrame],
            VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
            VK_PIPELINE_STAGE_2_FRAGMENT_SHADER_BIT,
            VK_ACCESS_2_SHADER_SAMPLED_READ_BIT);
        writeSceneTimestamp(6);

        // Exposição automática temporal. O trabalho inteiro acontece em um
        // único fragmento: ele mede 24 pontos do HDR, lê o multiplicador do
        // quadro anterior e grava o novo valor num attachment 1x1.
        std::array<VkImage, FramesInFlight>& exposureImages =
            directToSwapchain ? sceneDirectAutoExposureImage
                              : sceneAutoExposureImage;
        std::array<VkImageView, FramesInFlight>& exposureViews =
            directToSwapchain ? sceneDirectAutoExposureView
                              : sceneAutoExposureView;
        std::array<SceneAttachmentState3D, FramesInFlight>& exposureStates =
            directToSwapchain ? sceneDirectAutoExposureState
                              : sceneAutoExposureState;
        auto& exposureInitialized=directToSwapchain
            ? sceneDirectAutoExposureInitialized : sceneAutoExposureInitialized;
        bool& exposureHistoryValid = directToSwapchain
            ? sceneDirectAutoExposureHistoryValid
            : sceneAutoExposureHistoryValid;
        bool& exposureClockValid = directToSwapchain
            ? sceneDirectAutoExposureClockValid
            : sceneAutoExposureClockValid;
        std::chrono::steady_clock::time_point& exposureLastTime =
            directToSwapchain ? sceneDirectAutoExposureLastTime
                              : sceneAutoExposureLastTime;

        const auto exposureNow = std::chrono::steady_clock::now();
        float exposureDeltaTime = 1.0f / 60.0f;
        if (exposureClockValid) {
            exposureDeltaTime = std::clamp(
                std::chrono::duration<float>(
                    exposureNow - exposureLastTime).count(),
                0.0f, 0.1f);
        }
        exposureLastTime = exposureNow;
        exposureClockValid = scene.toneMapping.automaticExposureEnabled;
        const bool useExposureHistory =
            scene.toneMapping.automaticExposureEnabled
            && exposureHistoryValid
            && !scene.resetTemporalHistory;

        const VkDescriptorSet tonemapDescriptorSet =
            scene.temporalAntiAliasingEnabled
                ? (directToSwapchain
                    ? sceneDirectTonemapDescriptorSets[currentFrame]
                    : sceneTonemapDescriptorSets[currentFrame])
                : (directToSwapchain
                    ? sceneDirectTonemapWithoutTaaDescriptorSets[
                        currentFrame]
                    : sceneTonemapWithoutTaaDescriptorSets[currentFrame]);
        const bool initializeNeutralExposure =
            !exposureInitialized[currentFrame];
        if (scene.toneMapping.automaticExposureEnabled
            || initializeNeutralExposure) {
            // Quando desabilitado, o shader grava 1.0 somente na primeira
            // utilização de cada slot de frames-in-flight. Reexecutar este
            // passe 1x1 todo frame criava uma dependência de pipeline que
            // custava muito mais que o fragmento em si.
            {
                // A statically referenced descriptor must have its declared
                // layout even when the exposure shader skips that branch.
                transitionSceneAttachment(frame.commandBuffer,
                    exposureImages[historyReadIndex],
                    VK_IMAGE_ASPECT_COLOR_BIT,
                    exposureStates[historyReadIndex],
                    VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
                    VK_PIPELINE_STAGE_2_FRAGMENT_SHADER_BIT,
                    VK_ACCESS_2_SHADER_SAMPLED_READ_BIT);
            }
            transitionSceneAttachment(frame.commandBuffer,
                exposureImages[currentFrame], VK_IMAGE_ASPECT_COLOR_BIT,
                exposureStates[currentFrame],
                VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL,
                VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT,
                VK_ACCESS_2_COLOR_ATTACHMENT_WRITE_BIT);

            VkRenderingAttachmentInfo exposureAttachment {
                VK_STRUCTURE_TYPE_RENDERING_ATTACHMENT_INFO
            };
            exposureAttachment.imageView = exposureViews[currentFrame];
            exposureAttachment.imageLayout =
                VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
            exposureAttachment.loadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
            exposureAttachment.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
            VkRenderingInfo exposureRenderingInfo {
                VK_STRUCTURE_TYPE_RENDERING_INFO
            };
            exposureRenderingInfo.renderArea.extent = { 1, 1 };
            exposureRenderingInfo.layerCount = 1;
            exposureRenderingInfo.colorAttachmentCount = 1;
            exposureRenderingInfo.pColorAttachments =
                &exposureAttachment;
            vkCmdBeginRendering(frame.commandBuffer,
                &exposureRenderingInfo);
            setViewportAndScissor({ 1, 1 });
            vkCmdBindPipeline(frame.commandBuffer,
                VK_PIPELINE_BIND_POINT_GRAPHICS,
                sceneAutoExposurePipeline);
            vkCmdBindDescriptorSets(frame.commandBuffer,
                VK_PIPELINE_BIND_POINT_GRAPHICS,
                sceneAutoExposurePipelineLayout, 0, 1,
                &tonemapDescriptorSet, 0, nullptr);
            const float minimumExposure = std::max(
                scene.toneMapping.automaticExposureMinimum, 0.01f);
            const AutoExposurePushConstantsGpu exposurePushData {
                minimumExposure,
                std::max(scene.toneMapping.automaticExposureMaximum,
                    minimumExposure),
                std::max(scene.toneMapping.automaticExposureMeteringKey,
                    0.001f),
                exposureDeltaTime,
                std::max(scene.toneMapping.brightAdaptationSpeed, 0.0f),
                std::max(scene.toneMapping.darkAdaptationSpeed, 0.0f),
                useExposureHistory ? 1.0f : 0.0f,
                scene.toneMapping.automaticExposureEnabled ? 1.0f : 0.0f
            };
            vkCmdPushConstants(frame.commandBuffer,
                sceneAutoExposurePipelineLayout,
                VK_SHADER_STAGE_FRAGMENT_BIT, 0,
                sizeof(AutoExposurePushConstantsGpu),
                &exposurePushData);
            vkCmdDraw(frame.commandBuffer, 3, 1, 0, 0);
            vkCmdEndRendering(frame.commandBuffer);
            transitionSceneAttachment(frame.commandBuffer,
                exposureImages[currentFrame], VK_IMAGE_ASPECT_COLOR_BIT,
                exposureStates[currentFrame],
                VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
                VK_PIPELINE_STAGE_2_FRAGMENT_SHADER_BIT,
                VK_ACCESS_2_SHADER_SAMPLED_READ_BIT);
        }
        exposureHistoryValid =
            scene.toneMapping.automaticExposureEnabled;
        exposureInitialized[currentFrame]=true;
        writeSceneTimestamp(7);

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
        // O Pixel Art pode renderizar o mundo abaixo da resolução nativa
        // para economizar fill-rate. Só o tonemap cobre o swapchain inteiro:
        // a UI vem depois, já nativa, e o sampler nearest mantém os blocos
        // deliberados sem introduzir blur.
        const VkExtent2D presentationExtent = directToSwapchain
            ? swapchainExtent : renderExtent;
        VkRenderingInfo tonemapRenderingInfo { VK_STRUCTURE_TYPE_RENDERING_INFO };
        tonemapRenderingInfo.renderArea.extent = presentationExtent;
        tonemapRenderingInfo.layerCount = 1;
        tonemapRenderingInfo.colorAttachmentCount = 1;
        tonemapRenderingInfo.pColorAttachments = &tonemapAttachment;
        vkCmdBeginRendering(frame.commandBuffer, &tonemapRenderingInfo);
        setViewportAndScissor(presentationExtent);
        vkCmdBindPipeline(frame.commandBuffer, VK_PIPELINE_BIND_POINT_GRAPHICS, sceneTonemapPipeline);
        vkCmdBindDescriptorSets(frame.commandBuffer, VK_PIPELINE_BIND_POINT_GRAPHICS,
            sceneTonemapPipelineLayout, 0, 1, &tonemapDescriptorSet, 0, nullptr);
        const TonemapPushConstantsGpu tonemapPushData {
            scene.toneMapping.exposure, scene.toneMapping.brightness,
            scene.toneMapping.contrast, scene.toneMapping.saturation,
            scene.oceanSubmersion,
            scene.environment.skyAnimationTime,
            scene.renderMode == SceneRenderMode3D::PixelArt ? 1.0f : 0.0f,
            scene.pixelArt.luminanceLevelCount,
            scene.pixelArt.chromaLevelCount,
            scene.pixelArt.ditherStrength,
            0.0f,
            scene.pixelArt.pixelGridHeight,
            scene.pixelArt.worldPixelation,
            scene.pixelArt.physicalPropPixelation,
            scene.pixelArt.distanceLodStrength,
            scene.pixelArt.distanceLodStartMeters,
            scene.pixelArt.distanceLodEndMeters,
            scene.pixelArt.flatteningStrength
        };
        vkCmdPushConstants(frame.commandBuffer, sceneTonemapPipelineLayout,
            VK_SHADER_STAGE_FRAGMENT_BIT, 0, sizeof(TonemapPushConstantsGpu),
            &tonemapPushData);
        vkCmdDraw(frame.commandBuffer, 3, 1, 0, 0);
        writeSceneTimestamp(8);

        if (directToSwapchain) {
            // Este passe de tonemap fica aberto de proposito - o ImGui
            // desenha a UI por cima dele antes do vkCmdEndRendering em
            // endFrame() (ver renderImGui/endFrame). Formato de cor e
            // ausencia de depth attachment aqui batem exatamente com o que
            // o pipeline do proprio ImGui declara (ver initialize()).
            swapchainPassActive = true;
            frame.sceneTimestampsWritten = true;
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
        float renderScale = 1.0f;
        if (scene.renderMode == SceneRenderMode3D::PixelArt) {
            renderScale = std::clamp(
                scene.pixelArt.renderScale, 0.25f, 1.0f);
        }
        const Extent2D renderExtent {
            std::max(1u, static_cast<std::uint32_t>(std::lround(
                static_cast<float>(swapchainExtent.width) * renderScale))),
            std::max(1u, static_cast<std::uint32_t>(std::lround(
                static_cast<float>(swapchainExtent.height) * renderScale)))
        };
        renderScene3DInternal(scene, renderExtent, true);
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
        const std::uint32_t firstTimestamp =
            currentFrame * FrameTimestampCount;
        if (!frame.sceneTimestampsWritten) {
            // Menus e previews não passam pelo frame-graph 3D. Inicializa os
            // marcos intermediários no fim para manter a consulta inteira
            // válida; nesses frames só o total e UI têm significado.
            for (std::uint32_t index = 1;
                    index + 1 < FrameTimestampCount; ++index) {
                vkCmdWriteTimestamp2(frame.commandBuffer,
                    VK_PIPELINE_STAGE_2_ALL_COMMANDS_BIT,
                    frameTimestampQueryPool, firstTimestamp + index);
            }
        }
        vkCmdWriteTimestamp2(frame.commandBuffer,
            VK_PIPELINE_STAGE_2_ALL_COMMANDS_BIT,
            frameTimestampQueryPool,
            firstTimestamp + FrameTimestampCount - 1);
        check(vkEndCommandBuffer(frame.commandBuffer), "vkEndCommandBuffer");

        VkSemaphoreSubmitInfo waitInfo { VK_STRUCTURE_TYPE_SEMAPHORE_SUBMIT_INFO };
        waitInfo.semaphore = frame.imageAvailable;
        waitInfo.stageMask = VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT;
        VkCommandBufferSubmitInfo commandInfo { VK_STRUCTURE_TYPE_COMMAND_BUFFER_SUBMIT_INFO };
        commandInfo.commandBuffer = frame.commandBuffer;
        VkSemaphoreSubmitInfo signalInfo { VK_STRUCTURE_TYPE_SEMAPHORE_SUBMIT_INFO };
        signalInfo.semaphore = swapchainPresentSemaphores[currentImage];
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
        present.pWaitSemaphores = &swapchainPresentSemaphores[currentImage];
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
            if (features13.dynamicRendering != VK_TRUE || features13.synchronization2 != VK_TRUE
                || features13.shaderDemoteToHelperInvocation != VK_TRUE
                || features.features.independentBlend != VK_TRUE) {
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
        features13.shaderDemoteToHelperInvocation = VK_TRUE;

        // samplerAnisotropy (checado em selectPhysicalDevice) precisa ser
        // pedido explicitamente aqui pra virar utilizavel - sem isso,
        // VkSamplerCreateInfo::anisotropyEnable=VK_TRUE seria invalido.
        // pEnabledFeatures (base) e pNext=VkPhysicalDeviceFeatures2 sao
        // mutuamente exclusivos pela spec - como o pNext desta struct so
        // encadeia VkPhysicalDeviceVulkan13Features (features "por versao",
        // nao a struct base), usar pEnabledFeatures aqui continua valido.
        VkPhysicalDeviceFeatures enabledFeatures {};
        enabledFeatures.samplerAnisotropy = VK_TRUE;
        enabledFeatures.independentBlend = VK_TRUE;

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

            VkFenceCreateInfo fenceInfo { VK_STRUCTURE_TYPE_FENCE_CREATE_INFO };
            fenceInfo.flags = VK_FENCE_CREATE_SIGNALED_BIT;
            check(vkCreateFence(device, &fenceInfo, nullptr, &frame.fence), "vkCreateFence");
        }
        VkQueryPoolCreateInfo queryInfo {
            VK_STRUCTURE_TYPE_QUERY_POOL_CREATE_INFO
        };
        queryInfo.queryType = VK_QUERY_TYPE_TIMESTAMP;
        queryInfo.queryCount = FramesInFlight * FrameTimestampCount;
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
        swapchainPresentSemaphores.resize(imageCount);
        for (std::size_t i = 0; i < swapchainImages.size(); ++i) {
            VkSemaphoreCreateInfo semaphoreInfo { VK_STRUCTURE_TYPE_SEMAPHORE_CREATE_INFO };
            check(vkCreateSemaphore(device, &semaphoreInfo, nullptr,
                &swapchainPresentSemaphores[i]), "vkCreateSemaphore(present image)");
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
        for (VkSemaphore semaphore : swapchainPresentSemaphores)
            vkDestroySemaphore(device, semaphore, nullptr);
        swapchainPresentSemaphores.clear();
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
        if (sceneNormalRoughnessView != VK_NULL_HANDLE) vkDestroyImageView(device, sceneNormalRoughnessView, nullptr);
        if (sceneReflectanceView != VK_NULL_HANDLE) vkDestroyImageView(device, sceneReflectanceView, nullptr);
        if (scenePlanarReflectionView != VK_NULL_HANDLE) vkDestroyImageView(device, scenePlanarReflectionView, nullptr);
        if (sceneColorImage != VK_NULL_HANDLE) vmaDestroyImage(allocator, sceneColorImage, sceneColorAllocation);
        if (sceneDepthImage != VK_NULL_HANDLE) vmaDestroyImage(allocator, sceneDepthImage, sceneDepthAllocation);
        if (sceneHdrColorImage != VK_NULL_HANDLE) vmaDestroyImage(allocator, sceneHdrColorImage, sceneHdrColorAllocation);
        if (sceneMotionVectorImage != VK_NULL_HANDLE) vmaDestroyImage(allocator, sceneMotionVectorImage, sceneMotionVectorAllocation);
        if (sceneNormalRoughnessImage != VK_NULL_HANDLE) vmaDestroyImage(allocator, sceneNormalRoughnessImage, sceneNormalRoughnessAllocation);
        if (sceneReflectanceImage != VK_NULL_HANDLE) vmaDestroyImage(allocator, sceneReflectanceImage, sceneReflectanceAllocation);
        if (scenePlanarReflectionImage != VK_NULL_HANDLE) vmaDestroyImage(allocator, scenePlanarReflectionImage, scenePlanarReflectionAllocation);
        sceneColorImage = VK_NULL_HANDLE;
        sceneDepthImage = VK_NULL_HANDLE;
        sceneHdrColorImage = VK_NULL_HANDLE;
        sceneMotionVectorImage = VK_NULL_HANDLE;
        sceneNormalRoughnessImage = VK_NULL_HANDLE;
        sceneReflectanceImage = VK_NULL_HANDLE;
        scenePlanarReflectionImage = VK_NULL_HANDLE;
        sceneColorAllocation = VK_NULL_HANDLE;
        sceneDepthAllocation = VK_NULL_HANDLE;
        sceneHdrColorAllocation = VK_NULL_HANDLE;
        sceneMotionVectorAllocation = VK_NULL_HANDLE;
        sceneNormalRoughnessAllocation = VK_NULL_HANDLE;
        sceneReflectanceAllocation = VK_NULL_HANDLE;
        scenePlanarReflectionAllocation = VK_NULL_HANDLE;
        sceneColorView = VK_NULL_HANDLE;
        sceneDepthView = VK_NULL_HANDLE;
        sceneHdrColorView = VK_NULL_HANDLE;
        sceneMotionVectorView = VK_NULL_HANDLE;
        sceneNormalRoughnessView = VK_NULL_HANDLE;
        sceneReflectanceView = VK_NULL_HANDLE;
        scenePlanarReflectionView = VK_NULL_HANDLE;
        sceneColorState = {};
        sceneDepthState = {};
        sceneHdrColorState = {};
        sceneMotionVectorState = {};
        sceneNormalRoughnessState = {};
        sceneReflectanceState = {};
        scenePlanarReflectionState = {};
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
        for (std::size_t index = 0;
            index < sceneAutoExposureImage.size(); ++index) {
            if (sceneAutoExposureView[index] != VK_NULL_HANDLE) {
                vkDestroyImageView(device, sceneAutoExposureView[index],
                    nullptr);
            }
            if (sceneAutoExposureImage[index] != VK_NULL_HANDLE) {
                vmaDestroyImage(allocator, sceneAutoExposureImage[index],
                    sceneAutoExposureAllocation[index]);
            }
            sceneAutoExposureImage[index] = VK_NULL_HANDLE;
            sceneAutoExposureAllocation[index] = VK_NULL_HANDLE;
            sceneAutoExposureView[index] = VK_NULL_HANDLE;
            sceneAutoExposureState[index] = {};
            sceneAutoExposureInitialized[index]=false;
        }
        sceneAutoExposureHistoryValid = false;
        sceneAutoExposureClockValid = false;
        for (std::size_t index = 0;
            index < sceneGtaoImage.size(); ++index) {
            if (sceneGtaoView[index] != VK_NULL_HANDLE) {
                vkDestroyImageView(device, sceneGtaoView[index], nullptr);
            }
            if (sceneGtaoImage[index] != VK_NULL_HANDLE) {
                vmaDestroyImage(allocator, sceneGtaoImage[index],
                    sceneGtaoAllocation[index]);
            }
            sceneGtaoImage[index] = VK_NULL_HANDLE;
            sceneGtaoAllocation[index] = VK_NULL_HANDLE;
            sceneGtaoView[index] = VK_NULL_HANDLE;
            sceneGtaoState[index] = {};
        }
        for (std::size_t index = 0;
            index < sceneBloomGlareImage.size(); ++index) {
            if (sceneBloomGlareView[index] != VK_NULL_HANDLE) {
                vkDestroyImageView(device, sceneBloomGlareView[index],
                    nullptr);
            }
            if (sceneBloomGlareImage[index] != VK_NULL_HANDLE) {
                vmaDestroyImage(allocator, sceneBloomGlareImage[index],
                    sceneBloomGlareAllocation[index]);
            }
            sceneBloomGlareImage[index] = VK_NULL_HANDLE;
            sceneBloomGlareAllocation[index] = VK_NULL_HANDLE;
            sceneBloomGlareView[index] = VK_NULL_HANDLE;
            sceneBloomGlareState[index] = {};
        }
        for (std::size_t index = 0; index < sceneSsrHistoryImage.size(); ++index) {
            if (sceneSsrHistoryView[index] != VK_NULL_HANDLE) {
                vkDestroyImageView(device, sceneSsrHistoryView[index], nullptr);
            }
            if (sceneSsrHistoryImage[index] != VK_NULL_HANDLE) {
                vmaDestroyImage(allocator, sceneSsrHistoryImage[index],
                    sceneSsrHistoryAllocation[index]);
            }
            sceneSsrHistoryImage[index] = VK_NULL_HANDLE;
            sceneSsrHistoryAllocation[index] = VK_NULL_HANDLE;
            sceneSsrHistoryView[index] = VK_NULL_HANDLE;
            sceneSsrHistoryState[index] = {};
        }
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
        if (sceneDirectNormalRoughnessView != VK_NULL_HANDLE) {
            vkDestroyImageView(device, sceneDirectNormalRoughnessView, nullptr);
        }
        if (sceneDirectReflectanceView != VK_NULL_HANDLE) {
            vkDestroyImageView(device, sceneDirectReflectanceView, nullptr);
        }
        if (sceneDirectPlanarReflectionView != VK_NULL_HANDLE) {
            vkDestroyImageView(device, sceneDirectPlanarReflectionView, nullptr);
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
        if (sceneDirectNormalRoughnessImage != VK_NULL_HANDLE) {
            vmaDestroyImage(allocator, sceneDirectNormalRoughnessImage, sceneDirectNormalRoughnessAllocation);
        }
        if (sceneDirectReflectanceImage != VK_NULL_HANDLE) {
            vmaDestroyImage(allocator, sceneDirectReflectanceImage, sceneDirectReflectanceAllocation);
        }
        if (sceneDirectPlanarReflectionImage != VK_NULL_HANDLE) {
            vmaDestroyImage(allocator, sceneDirectPlanarReflectionImage,
                sceneDirectPlanarReflectionAllocation);
        }
        sceneDirectDepthImage = VK_NULL_HANDLE;
        sceneDirectHdrColorImage = VK_NULL_HANDLE;
        sceneDirectMotionVectorImage = VK_NULL_HANDLE;
        sceneDirectNormalRoughnessImage = VK_NULL_HANDLE;
        sceneDirectReflectanceImage = VK_NULL_HANDLE;
        sceneDirectPlanarReflectionImage = VK_NULL_HANDLE;
        sceneDirectDepthAllocation = VK_NULL_HANDLE;
        sceneDirectHdrColorAllocation = VK_NULL_HANDLE;
        sceneDirectMotionVectorAllocation = VK_NULL_HANDLE;
        sceneDirectNormalRoughnessAllocation = VK_NULL_HANDLE;
        sceneDirectReflectanceAllocation = VK_NULL_HANDLE;
        sceneDirectPlanarReflectionAllocation = VK_NULL_HANDLE;
        sceneDirectDepthView = VK_NULL_HANDLE;
        sceneDirectHdrColorView = VK_NULL_HANDLE;
        sceneDirectMotionVectorView = VK_NULL_HANDLE;
        sceneDirectNormalRoughnessView = VK_NULL_HANDLE;
        sceneDirectReflectanceView = VK_NULL_HANDLE;
        sceneDirectPlanarReflectionView = VK_NULL_HANDLE;
        sceneDirectDepthState = {};
        sceneDirectHdrColorState = {};
        sceneDirectMotionVectorState = {};
        sceneDirectNormalRoughnessState = {};
        sceneDirectReflectanceState = {};
        sceneDirectPlanarReflectionState = {};
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
        for (std::size_t index = 0;
            index < sceneDirectAutoExposureImage.size(); ++index) {
            if (sceneDirectAutoExposureView[index] != VK_NULL_HANDLE) {
                vkDestroyImageView(device,
                    sceneDirectAutoExposureView[index], nullptr);
            }
            if (sceneDirectAutoExposureImage[index] != VK_NULL_HANDLE) {
                vmaDestroyImage(allocator,
                    sceneDirectAutoExposureImage[index],
                    sceneDirectAutoExposureAllocation[index]);
            }
            sceneDirectAutoExposureImage[index] = VK_NULL_HANDLE;
            sceneDirectAutoExposureAllocation[index] = VK_NULL_HANDLE;
            sceneDirectAutoExposureView[index] = VK_NULL_HANDLE;
            sceneDirectAutoExposureState[index] = {};
            sceneDirectAutoExposureInitialized[index]=false;
        }
        sceneDirectAutoExposureHistoryValid = false;
        sceneDirectAutoExposureClockValid = false;
        for (std::size_t index = 0;
            index < sceneDirectGtaoImage.size(); ++index) {
            if (sceneDirectGtaoView[index] != VK_NULL_HANDLE) {
                vkDestroyImageView(device, sceneDirectGtaoView[index],
                    nullptr);
            }
            if (sceneDirectGtaoImage[index] != VK_NULL_HANDLE) {
                vmaDestroyImage(allocator, sceneDirectGtaoImage[index],
                    sceneDirectGtaoAllocation[index]);
            }
            sceneDirectGtaoImage[index] = VK_NULL_HANDLE;
            sceneDirectGtaoAllocation[index] = VK_NULL_HANDLE;
            sceneDirectGtaoView[index] = VK_NULL_HANDLE;
            sceneDirectGtaoState[index] = {};
        }
        for (std::size_t index = 0;
            index < sceneDirectBloomGlareImage.size(); ++index) {
            if (sceneDirectBloomGlareView[index] != VK_NULL_HANDLE) {
                vkDestroyImageView(device,
                    sceneDirectBloomGlareView[index], nullptr);
            }
            if (sceneDirectBloomGlareImage[index] != VK_NULL_HANDLE) {
                vmaDestroyImage(allocator,
                    sceneDirectBloomGlareImage[index],
                    sceneDirectBloomGlareAllocation[index]);
            }
            sceneDirectBloomGlareImage[index] = VK_NULL_HANDLE;
            sceneDirectBloomGlareAllocation[index] = VK_NULL_HANDLE;
            sceneDirectBloomGlareView[index] = VK_NULL_HANDLE;
            sceneDirectBloomGlareState[index] = {};
        }
        for (std::size_t index = 0; index < sceneDirectSsrHistoryImage.size(); ++index) {
            if (sceneDirectSsrHistoryView[index] != VK_NULL_HANDLE) {
                vkDestroyImageView(device, sceneDirectSsrHistoryView[index], nullptr);
            }
            if (sceneDirectSsrHistoryImage[index] != VK_NULL_HANDLE) {
                vmaDestroyImage(allocator, sceneDirectSsrHistoryImage[index],
                    sceneDirectSsrHistoryAllocation[index]);
            }
            sceneDirectSsrHistoryImage[index] = VK_NULL_HANDLE;
            sceneDirectSsrHistoryAllocation[index] = VK_NULL_HANDLE;
            sceneDirectSsrHistoryView[index] = VK_NULL_HANDLE;
            sceneDirectSsrHistoryState[index] = {};
        }
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
        if (sceneAutoExposurePipeline != VK_NULL_HANDLE) vkDestroyPipeline(device, sceneAutoExposurePipeline, nullptr);
        if (sceneGtaoPipeline != VK_NULL_HANDLE) vkDestroyPipeline(device, sceneGtaoPipeline, nullptr);
        if (sceneBloomGlarePipeline != VK_NULL_HANDLE) vkDestroyPipeline(device, sceneBloomGlarePipeline, nullptr);
        if (sceneTaaResolvePipeline != VK_NULL_HANDLE) vkDestroyPipeline(device, sceneTaaResolvePipeline, nullptr);
        if (sceneSsrPipeline != VK_NULL_HANDLE) vkDestroyPipeline(device, sceneSsrPipeline, nullptr);
        if (sceneSsrCompositePipeline != VK_NULL_HANDLE) vkDestroyPipeline(device, sceneSsrCompositePipeline, nullptr);
        if (sceneOceanPipeline != VK_NULL_HANDLE) vkDestroyPipeline(device, sceneOceanPipeline, nullptr);
        if (sceneMeshPipelineLayout != VK_NULL_HANDLE) vkDestroyPipelineLayout(device, sceneMeshPipelineLayout, nullptr);
        if (sceneTonemapPipelineLayout != VK_NULL_HANDLE) vkDestroyPipelineLayout(device, sceneTonemapPipelineLayout, nullptr);
        if (sceneAutoExposurePipelineLayout != VK_NULL_HANDLE) vkDestroyPipelineLayout(device, sceneAutoExposurePipelineLayout, nullptr);
        if (sceneGtaoPipelineLayout != VK_NULL_HANDLE) vkDestroyPipelineLayout(device, sceneGtaoPipelineLayout, nullptr);
        if (sceneBloomGlarePipelineLayout != VK_NULL_HANDLE) vkDestroyPipelineLayout(device, sceneBloomGlarePipelineLayout, nullptr);
        if (sceneTaaResolvePipelineLayout != VK_NULL_HANDLE) vkDestroyPipelineLayout(device, sceneTaaResolvePipelineLayout, nullptr);
        if (sceneSsrPipelineLayout != VK_NULL_HANDLE) vkDestroyPipelineLayout(device, sceneSsrPipelineLayout, nullptr);
        if (sceneSsrCompositePipelineLayout != VK_NULL_HANDLE) vkDestroyPipelineLayout(device, sceneSsrCompositePipelineLayout, nullptr);
        if (sceneOceanPipelineLayout != VK_NULL_HANDLE) vkDestroyPipelineLayout(device, sceneOceanPipelineLayout, nullptr);
        if (materialDescriptorPool != VK_NULL_HANDLE) vkDestroyDescriptorPool(device, materialDescriptorPool, nullptr);
        if (scenePlanarReflectionDescriptorPool != VK_NULL_HANDLE) vkDestroyDescriptorPool(device, scenePlanarReflectionDescriptorPool, nullptr);
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
        if (sceneGtaoDescriptorPool != VK_NULL_HANDLE) vkDestroyDescriptorPool(device, sceneGtaoDescriptorPool, nullptr);
        if (sceneGtaoDescriptorSetLayout != VK_NULL_HANDLE) vkDestroyDescriptorSetLayout(device, sceneGtaoDescriptorSetLayout, nullptr);
        if (sceneBloomGlareDescriptorPool != VK_NULL_HANDLE) vkDestroyDescriptorPool(device, sceneBloomGlareDescriptorPool, nullptr);
        if (sceneBloomGlareDescriptorSetLayout != VK_NULL_HANDLE) vkDestroyDescriptorSetLayout(device, sceneBloomGlareDescriptorSetLayout, nullptr);
        if (sceneSsrDescriptorPool != VK_NULL_HANDLE) vkDestroyDescriptorPool(device, sceneSsrDescriptorPool, nullptr);
        if (sceneSsrDescriptorSetLayout != VK_NULL_HANDLE) vkDestroyDescriptorSetLayout(device, sceneSsrDescriptorSetLayout, nullptr);
        if (sceneSsrCompositeDescriptorPool != VK_NULL_HANDLE) vkDestroyDescriptorPool(device, sceneSsrCompositeDescriptorPool, nullptr);
        if (sceneSsrCompositeDescriptorSetLayout != VK_NULL_HANDLE) vkDestroyDescriptorSetLayout(device, sceneSsrCompositeDescriptorSetLayout, nullptr);
        if (sceneOceanDescriptorPool != VK_NULL_HANDLE) vkDestroyDescriptorPool(device, sceneOceanDescriptorPool, nullptr);
        if (sceneOceanDescriptorSetLayout != VK_NULL_HANDLE) vkDestroyDescriptorSetLayout(device, sceneOceanDescriptorSetLayout, nullptr);
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
        if (sceneAutoExposureFragmentModule != VK_NULL_HANDLE) vkDestroyShaderModule(device, sceneAutoExposureFragmentModule, nullptr);
        if (sceneGtaoFragmentModule != VK_NULL_HANDLE) vkDestroyShaderModule(device, sceneGtaoFragmentModule, nullptr);
        if (sceneBloomGlareFragmentModule != VK_NULL_HANDLE) vkDestroyShaderModule(device, sceneBloomGlareFragmentModule, nullptr);
        if (sceneTaaResolveFragmentModule != VK_NULL_HANDLE) vkDestroyShaderModule(device, sceneTaaResolveFragmentModule, nullptr);
        if (sceneSsrFragmentModule != VK_NULL_HANDLE) vkDestroyShaderModule(device, sceneSsrFragmentModule, nullptr);
        if (sceneSsrCompositeFragmentModule != VK_NULL_HANDLE) vkDestroyShaderModule(device, sceneSsrCompositeFragmentModule, nullptr);
        if (sceneOceanVertexModule != VK_NULL_HANDLE) vkDestroyShaderModule(device, sceneOceanVertexModule, nullptr);
        if (sceneOceanFragmentModule != VK_NULL_HANDLE) vkDestroyShaderModule(device, sceneOceanFragmentModule, nullptr);
        if (sceneShadowSampler != VK_NULL_HANDLE) vkDestroySampler(device, sceneShadowSampler, nullptr);
        if (sceneShadowRawSampler != VK_NULL_HANDLE) vkDestroySampler(device, sceneShadowRawSampler, nullptr);
        for (std::size_t index = 0; index < sceneUniformBuffers.size(); ++index) {
            if (sceneUniformBuffers[index] != VK_NULL_HANDLE) {
                vmaDestroyBuffer(allocator, sceneUniformBuffers[index], sceneUniformAllocations[index]);
            }
            if (scenePlanarCameraUniformBuffers[index] != VK_NULL_HANDLE) {
                vmaDestroyBuffer(allocator,
                    scenePlanarCameraUniformBuffers[index],
                    scenePlanarCameraUniformAllocations[index]);
            }
            if (sceneMeshInstanceBuffers[index] != VK_NULL_HANDLE) {
                vmaDestroyBuffer(allocator, sceneMeshInstanceBuffers[index],
                    sceneMeshInstanceAllocations[index]);
            }
            if (sceneLightBuffers[index] != VK_NULL_HANDLE) {
                vmaDestroyBuffer(allocator, sceneLightBuffers[index],
                    sceneLightAllocations[index]);
            }
            if (sceneSkinBuffers[index] != VK_NULL_HANDLE) {
                vmaDestroyBuffer(allocator, sceneSkinBuffers[index],
                    sceneSkinAllocations[index]);
            }
        }
        sceneSkyPipeline = VK_NULL_HANDLE;
        sceneMeshPipeline = VK_NULL_HANDLE;
        sceneMeshShadowPipeline = VK_NULL_HANDLE;
        sceneMeshDepthPrepassPipeline = VK_NULL_HANDLE;
        sceneTonemapPipeline = VK_NULL_HANDLE;
        sceneAutoExposurePipeline = VK_NULL_HANDLE;
        sceneGtaoPipeline = VK_NULL_HANDLE;
        sceneBloomGlarePipeline = VK_NULL_HANDLE;
        sceneTaaResolvePipeline = VK_NULL_HANDLE;
        sceneSsrPipeline = VK_NULL_HANDLE;
        sceneSsrCompositePipeline = VK_NULL_HANDLE;
        sceneOceanPipeline = VK_NULL_HANDLE;
        sceneMeshPipelineLayout = VK_NULL_HANDLE;
        sceneTonemapPipelineLayout = VK_NULL_HANDLE;
        sceneAutoExposurePipelineLayout = VK_NULL_HANDLE;
        sceneGtaoPipelineLayout = VK_NULL_HANDLE;
        sceneBloomGlarePipelineLayout = VK_NULL_HANDLE;
        sceneTaaResolvePipelineLayout = VK_NULL_HANDLE;
        sceneSsrPipelineLayout = VK_NULL_HANDLE;
        sceneSsrCompositePipelineLayout = VK_NULL_HANDLE;
        sceneOceanPipelineLayout = VK_NULL_HANDLE;
        materialDescriptorPool = VK_NULL_HANDLE;
        scenePlanarReflectionDescriptorPool = VK_NULL_HANDLE;
        scenePlanarReflectionDescriptorSets = {};
        sceneDirectPlanarReflectionDescriptorSets = {};
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
        sceneTonemapWithoutTaaDescriptorSets = {};
        sceneDirectTonemapWithoutTaaDescriptorSets = {};
        sceneTonemapSampler = VK_NULL_HANDLE;
        sceneTaaResolveDescriptorPool = VK_NULL_HANDLE;
        sceneTaaResolveDescriptorSetLayout = VK_NULL_HANDLE;
        sceneTaaResolveDescriptorSets = {};
        sceneDirectTaaResolveDescriptorSets = {};
        sceneGtaoDescriptorPool = VK_NULL_HANDLE;
        sceneGtaoDescriptorSetLayout = VK_NULL_HANDLE;
        sceneGtaoDescriptorSets = {};
        sceneDirectGtaoDescriptorSets = {};
        sceneBloomGlareDescriptorPool = VK_NULL_HANDLE;
        sceneBloomGlareDescriptorSetLayout = VK_NULL_HANDLE;
        sceneBloomGlareDescriptorSets = {};
        sceneDirectBloomGlareDescriptorSets = {};
        sceneSsrDescriptorPool = VK_NULL_HANDLE;
        sceneSsrDescriptorSetLayout = VK_NULL_HANDLE;
        sceneSsrDescriptorSets = {};
        sceneDirectSsrDescriptorSets = {};
        sceneSsrCompositeDescriptorPool = VK_NULL_HANDLE;
        sceneSsrCompositeDescriptorSetLayout = VK_NULL_HANDLE;
        sceneSsrCompositeDescriptorSets = {};
        sceneDirectSsrCompositeDescriptorSets = {};
        sceneOceanDescriptorPool = VK_NULL_HANDLE;
        sceneOceanDescriptorSetLayout = VK_NULL_HANDLE;
        sceneOceanDescriptorSets = {};
        sceneDirectOceanDescriptorSets = {};
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
        sceneAutoExposureFragmentModule = VK_NULL_HANDLE;
        sceneGtaoFragmentModule = VK_NULL_HANDLE;
        sceneBloomGlareFragmentModule = VK_NULL_HANDLE;
        sceneTaaResolveFragmentModule = VK_NULL_HANDLE;
        sceneSsrFragmentModule = VK_NULL_HANDLE;
        sceneSsrCompositeFragmentModule = VK_NULL_HANDLE;
        sceneOceanVertexModule = VK_NULL_HANDLE;
        sceneOceanFragmentModule = VK_NULL_HANDLE;
        sceneShadowSampler = VK_NULL_HANDLE;
        sceneShadowRawSampler = VK_NULL_HANDLE;
        sceneUniformBuffers = {};
        sceneUniformAllocations = {};
        sceneUniformMapped = {};
        scenePlanarCameraUniformBuffers = {};
        scenePlanarCameraUniformAllocations = {};
        scenePlanarCameraUniformMapped = {};
        sceneMeshInstanceBuffers = {};
        sceneMeshInstanceAllocations = {};
        sceneMeshInstanceMapped = {};
        sceneMeshInstanceCapacities = {};
        sceneLightBuffers = {};
        sceneLightAllocations = {};
        sceneLightMapped = {};
        sceneLightCapacities = {};
        sceneSkinBuffers = {};
        sceneSkinAllocations = {};
        sceneSkinMapped = {};
        sceneSkinCapacities = {};
        sceneDescriptorSets = {};
        scenePlanarCameraDescriptorSets = {};
    }

    void ensureSceneSkinCapacity(std::size_t required) {
        if (required <= sceneSkinCapacities[currentFrame]) return;
        const auto capacity = std::max<std::size_t>(1024, std::bit_ceil(required));
        if (sceneSkinBuffers[currentFrame] != VK_NULL_HANDLE) {
            vmaDestroyBuffer(allocator, sceneSkinBuffers[currentFrame],
                sceneSkinAllocations[currentFrame]);
        }
        VkBufferCreateInfo info { VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO };
        info.size = capacity * sizeof(Mat4);
        info.usage = VK_BUFFER_USAGE_STORAGE_BUFFER_BIT;
        info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
        VmaAllocationCreateInfo allocation {};
        allocation.usage = VMA_MEMORY_USAGE_AUTO_PREFER_HOST;
        allocation.flags = VMA_ALLOCATION_CREATE_HOST_ACCESS_SEQUENTIAL_WRITE_BIT
            | VMA_ALLOCATION_CREATE_MAPPED_BIT;
        VmaAllocationInfo mapped {};
        check(vmaCreateBuffer(allocator, &info, &allocation,
            &sceneSkinBuffers[currentFrame], &sceneSkinAllocations[currentFrame],
            &mapped), "vmaCreateBuffer(scene skin)");
        sceneSkinMapped[currentFrame] = mapped.pMappedData;
        sceneSkinCapacities[currentFrame] = capacity;
        VkDescriptorBufferInfo buffer {};
        buffer.buffer = sceneSkinBuffers[currentFrame];
        buffer.range = VK_WHOLE_SIZE;
        std::array<VkWriteDescriptorSet, 2> writes {};
        writes[0].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
        writes[0].dstSet = sceneDescriptorSets[currentFrame];
        writes[0].dstBinding = 4;
        writes[0].descriptorCount = 1;
        writes[0].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
        writes[0].pBufferInfo = &buffer;
        writes[1] = writes[0];
        writes[1].dstSet = scenePlanarCameraDescriptorSets[currentFrame];
        vkUpdateDescriptorSets(device, 2, writes.data(), 0, nullptr);
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
        std::array<VkWriteDescriptorSet, 2> writes {};
        writes[0].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
        writes[0].dstSet = sceneDescriptorSets[currentFrame];
        writes[0].dstBinding = 2;
        writes[0].descriptorCount = 1;
        writes[0].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
        writes[0].pBufferInfo = &descriptorBuffer;
        writes[1] = writes[0];
        writes[1].dstSet =
            scenePlanarCameraDescriptorSets[currentFrame];
        vkUpdateDescriptorSets(device,
            static_cast<std::uint32_t>(writes.size()), writes.data(),
            0, nullptr);
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

        std::array<VkDescriptorSetLayoutBinding, 5> bindings {};
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
        bindings[4].binding = 4;
        bindings[4].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
        bindings[4].descriptorCount = 1;
        bindings[4].stageFlags = VK_SHADER_STAGE_VERTEX_BIT;
        VkDescriptorSetLayoutCreateInfo descriptorLayoutInfo {
            VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO
        };
        descriptorLayoutInfo.bindingCount = static_cast<std::uint32_t>(bindings.size());
        descriptorLayoutInfo.pBindings = bindings.data();
        check(vkCreateDescriptorSetLayout(device, &descriptorLayoutInfo, nullptr,
            &sceneDescriptorSetLayout), "vkCreateDescriptorSetLayout(scene)");

        constexpr std::uint32_t SceneDescriptorSetCount =
            FramesInFlight * 2;
        const std::array<VkDescriptorPoolSize, 3> poolSizes { {
            { VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, SceneDescriptorSetCount },
            // ShadowCascadeCount descritores por set em CADA um dos dois
            // bindings de imagem de sombra (1 = comparacao, 3 = cru pro
            // PCSS, ver bindings[1]/bindings[3] acima) - daí o *2.
            { VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER,
                SceneDescriptorSetCount * ShadowCascadeCount * 2 },
            { VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, SceneDescriptorSetCount * 2 }
        } };
        VkDescriptorPoolCreateInfo poolInfo { VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO };
        poolInfo.maxSets = SceneDescriptorSetCount;
        poolInfo.poolSizeCount = static_cast<std::uint32_t>(poolSizes.size());
        poolInfo.pPoolSizes = poolSizes.data();
        check(vkCreateDescriptorPool(device, &poolInfo, nullptr, &sceneDescriptorPool),
            "vkCreateDescriptorPool(scene)");
        std::array<VkDescriptorSetLayout, SceneDescriptorSetCount> layouts {};
        layouts.fill(sceneDescriptorSetLayout);
        std::array<VkDescriptorSet, SceneDescriptorSetCount>
            allocatedSceneSets {};
        VkDescriptorSetAllocateInfo allocateInfo { VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO };
        allocateInfo.descriptorPool = sceneDescriptorPool;
        allocateInfo.descriptorSetCount = SceneDescriptorSetCount;
        allocateInfo.pSetLayouts = layouts.data();
        check(vkAllocateDescriptorSets(device, &allocateInfo,
            allocatedSceneSets.data()),
            "vkAllocateDescriptorSets(scene)");
        for (std::size_t index = 0; index < FramesInFlight; ++index) {
            sceneDescriptorSets[index] = allocatedSceneSets[index];
            scenePlanarCameraDescriptorSets[index] =
                allocatedSceneSets[FramesInFlight + index];
        }

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

            VmaAllocationInfo planarMappedInfo {};
            check(vmaCreateBuffer(allocator, &bufferInfo, &allocationInfo,
                &scenePlanarCameraUniformBuffers[index],
                &scenePlanarCameraUniformAllocations[index],
                &planarMappedInfo),
                "vmaCreateBuffer(scene planar camera uniform)");
            scenePlanarCameraUniformMapped[index] =
                planarMappedInfo.pMappedData;

            const std::array<VkDescriptorBufferInfo, 2> descriptorBuffers { {
                { sceneUniformBuffers[index], 0, sizeof(SceneUniformGpu) },
                { scenePlanarCameraUniformBuffers[index], 0,
                    sizeof(SceneUniformGpu) }
            } };
            std::array<VkWriteDescriptorSet, 2> writes {};
            writes[0].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
            writes[0].dstSet = sceneDescriptorSets[index];
            writes[0].dstBinding = 0;
            writes[0].descriptorCount = 1;
            writes[0].descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
            writes[0].pBufferInfo = &descriptorBuffers[0];
            writes[1] = writes[0];
            writes[1].dstSet = scenePlanarCameraDescriptorSets[index];
            writes[1].pBufferInfo = &descriptorBuffers[1];
            vkUpdateDescriptorSets(device,
                static_cast<std::uint32_t>(writes.size()), writes.data(),
                0, nullptr);
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
        createModule("auto_exposure.frag.spv",
            sceneAutoExposureFragmentModule);
        createModule("gtao.frag.spv", sceneGtaoFragmentModule);
        createModule("bloom_glare.frag.spv",
            sceneBloomGlareFragmentModule);
        createModule("taa_resolve.frag.spv", sceneTaaResolveFragmentModule);
        createModule("ssr_trace_resolve.frag.spv", sceneSsrFragmentModule);
        createModule("ssr_composite.frag.spv", sceneSsrCompositeFragmentModule);
        createModule("ocean_surface.vert.spv", sceneOceanVertexModule);
        createModule("ocean_surface.frag.spv", sceneOceanFragmentModule);

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

        // binding 0: HDR resolvido; binding 1: exposição do quadro anterior
        // (passe 1x1); binding 2: exposição atual (tonemap); binding 3:
        // bloom/glare HDR reduzido; binding 4: profundidade usada somente
        // pelo contorno seletivo do Matter Mosaic; binding 5: motion vectors
        // com a classe de detalhe dos props físicos codificada em X. Um set
        // por slot mantém os descriptors imutáveis enquanto a GPU os usa.
        std::array<VkDescriptorSetLayoutBinding, 6> tonemapBindings {};
        for (std::uint32_t binding = 0;
            binding < tonemapBindings.size(); ++binding) {
            tonemapBindings[binding].binding = binding;
            tonemapBindings[binding].descriptorType =
                VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
            tonemapBindings[binding].descriptorCount = 1;
            tonemapBindings[binding].stageFlags =
                VK_SHADER_STAGE_FRAGMENT_BIT;
        }
        VkDescriptorSetLayoutCreateInfo tonemapLayoutInfo {
            VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO
        };
        tonemapLayoutInfo.bindingCount =
            static_cast<std::uint32_t>(tonemapBindings.size());
        tonemapLayoutInfo.pBindings = tonemapBindings.data();
        check(vkCreateDescriptorSetLayout(device, &tonemapLayoutInfo, nullptr,
            &sceneTonemapDescriptorSetLayout), "vkCreateDescriptorSetLayout(tonemap)");

        constexpr std::uint32_t TonemapDescriptorSetCount =
            FramesInFlight * 4;
        const VkDescriptorPoolSize tonemapPoolSize {
            VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER,
            TonemapDescriptorSetCount
                * static_cast<std::uint32_t>(tonemapBindings.size())
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
            sceneTonemapWithoutTaaDescriptorSets[index] =
                tonemapDescriptorSets[FramesInFlight * 2 + index];
            sceneDirectTonemapWithoutTaaDescriptorSets[index] =
                tonemapDescriptorSets[FramesInFlight * 3 + index];
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

        VkPushConstantRange autoExposurePushConstant {};
        autoExposurePushConstant.stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;
        autoExposurePushConstant.offset = 0;
        autoExposurePushConstant.size =
            sizeof(AutoExposurePushConstantsGpu);
        VkPipelineLayoutCreateInfo autoExposurePipelineLayoutInfo {
            VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO
        };
        autoExposurePipelineLayoutInfo.setLayoutCount = 1;
        autoExposurePipelineLayoutInfo.pSetLayouts =
            &sceneTonemapDescriptorSetLayout;
        autoExposurePipelineLayoutInfo.pushConstantRangeCount = 1;
        autoExposurePipelineLayoutInfo.pPushConstantRanges =
            &autoExposurePushConstant;
        check(vkCreatePipelineLayout(device,
            &autoExposurePipelineLayoutInfo, nullptr,
            &sceneAutoExposurePipelineLayout),
            "vkCreatePipelineLayout(auto exposure)");

        // Descriptor set do resolve de TAA: 5 bindings, todas amostradas so
        // no fragmento - cor HDR atual, profundidade, vetores de movimento,
        // historico e GTAO (ver sceneTaaResolveDescriptorSetLayout).
        // Um set por frame em voo e por caminho. Todos os bindings sao
        // escritos apenas quando os attachments sao criados/recriados com o
        // device ocioso; nenhum descriptor em uso e mutado durante o frame.
        std::array<VkDescriptorSetLayoutBinding, 5> taaResolveBindings {};
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

        // GTAO lê profundidade+normal já finalizadas pelo passe opaco e
        // escreve um alvo em meia resolução. Há um set imutável por frame
        // em voo e por caminho, como nos demais passes temporais.
        std::array<VkDescriptorSetLayoutBinding, 2> gtaoBindings {};
        for (std::uint32_t binding = 0;
            binding < gtaoBindings.size(); ++binding) {
            gtaoBindings[binding].binding = binding;
            gtaoBindings[binding].descriptorType =
                VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
            gtaoBindings[binding].descriptorCount = 1;
            gtaoBindings[binding].stageFlags =
                VK_SHADER_STAGE_FRAGMENT_BIT;
        }
        VkDescriptorSetLayoutCreateInfo gtaoLayoutInfo {
            VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO
        };
        gtaoLayoutInfo.bindingCount =
            static_cast<std::uint32_t>(gtaoBindings.size());
        gtaoLayoutInfo.pBindings = gtaoBindings.data();
        check(vkCreateDescriptorSetLayout(device, &gtaoLayoutInfo, nullptr,
            &sceneGtaoDescriptorSetLayout),
            "vkCreateDescriptorSetLayout(gtao)");

        constexpr std::uint32_t GtaoDescriptorSetCount =
            FramesInFlight * 2;
        const VkDescriptorPoolSize gtaoPoolSize {
            VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER,
            GtaoDescriptorSetCount
                * static_cast<std::uint32_t>(gtaoBindings.size())
        };
        VkDescriptorPoolCreateInfo gtaoPoolInfo {
            VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO
        };
        gtaoPoolInfo.maxSets = GtaoDescriptorSetCount;
        gtaoPoolInfo.poolSizeCount = 1;
        gtaoPoolInfo.pPoolSizes = &gtaoPoolSize;
        check(vkCreateDescriptorPool(device, &gtaoPoolInfo, nullptr,
            &sceneGtaoDescriptorPool), "vkCreateDescriptorPool(gtao)");
        std::array<VkDescriptorSetLayout, GtaoDescriptorSetCount>
            gtaoSetLayouts {};
        gtaoSetLayouts.fill(sceneGtaoDescriptorSetLayout);
        std::array<VkDescriptorSet, GtaoDescriptorSetCount>
            gtaoDescriptorSets {};
        VkDescriptorSetAllocateInfo gtaoAllocateInfo {
            VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO
        };
        gtaoAllocateInfo.descriptorPool = sceneGtaoDescriptorPool;
        gtaoAllocateInfo.descriptorSetCount = GtaoDescriptorSetCount;
        gtaoAllocateInfo.pSetLayouts = gtaoSetLayouts.data();
        check(vkAllocateDescriptorSets(device, &gtaoAllocateInfo,
            gtaoDescriptorSets.data()),
            "vkAllocateDescriptorSets(gtao)");
        for (std::size_t index = 0; index < FramesInFlight; ++index) {
            sceneGtaoDescriptorSets[index] = gtaoDescriptorSets[index];
            sceneDirectGtaoDescriptorSets[index] =
                gtaoDescriptorSets[FramesInFlight + index];
        }

        const std::array<VkDescriptorSetLayout, 2>
            gtaoPipelineSetLayouts {
                sceneDescriptorSetLayout, sceneGtaoDescriptorSetLayout
            };
        VkPushConstantRange gtaoPushConstant {};
        gtaoPushConstant.stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;
        gtaoPushConstant.size = sizeof(GtaoPushConstantsGpu);
        VkPipelineLayoutCreateInfo gtaoPipelineLayoutInfo {
            VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO
        };
        gtaoPipelineLayoutInfo.setLayoutCount =
            static_cast<std::uint32_t>(gtaoPipelineSetLayouts.size());
        gtaoPipelineLayoutInfo.pSetLayouts =
            gtaoPipelineSetLayouts.data();
        gtaoPipelineLayoutInfo.pushConstantRangeCount = 1;
        gtaoPipelineLayoutInfo.pPushConstantRanges = &gtaoPushConstant;
        check(vkCreatePipelineLayout(device, &gtaoPipelineLayoutInfo,
            nullptr, &sceneGtaoPipelineLayout),
            "vkCreatePipelineLayout(gtao)");

        // Bloom/glare lê o HDR resolvido pelo TAA e a profundidade da cena.
        // O mesmo passe reduz, filtra realces e desenha os efeitos do Sol.
        std::array<VkDescriptorSetLayoutBinding, 2>
            bloomGlareBindings {};
        for (std::uint32_t binding = 0;
            binding < bloomGlareBindings.size(); ++binding) {
            bloomGlareBindings[binding].binding = binding;
            bloomGlareBindings[binding].descriptorType =
                VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
            bloomGlareBindings[binding].descriptorCount = 1;
            bloomGlareBindings[binding].stageFlags =
                VK_SHADER_STAGE_FRAGMENT_BIT;
        }
        VkDescriptorSetLayoutCreateInfo bloomGlareLayoutInfo {
            VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO
        };
        bloomGlareLayoutInfo.bindingCount =
            static_cast<std::uint32_t>(bloomGlareBindings.size());
        bloomGlareLayoutInfo.pBindings = bloomGlareBindings.data();
        check(vkCreateDescriptorSetLayout(device, &bloomGlareLayoutInfo,
            nullptr, &sceneBloomGlareDescriptorSetLayout),
            "vkCreateDescriptorSetLayout(bloom glare)");

        constexpr std::uint32_t BloomGlareDescriptorSetCount =
            FramesInFlight * 2;
        const VkDescriptorPoolSize bloomGlarePoolSize {
            VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER,
            BloomGlareDescriptorSetCount
                * static_cast<std::uint32_t>(bloomGlareBindings.size())
        };
        VkDescriptorPoolCreateInfo bloomGlarePoolInfo {
            VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO
        };
        bloomGlarePoolInfo.maxSets = BloomGlareDescriptorSetCount;
        bloomGlarePoolInfo.poolSizeCount = 1;
        bloomGlarePoolInfo.pPoolSizes = &bloomGlarePoolSize;
        check(vkCreateDescriptorPool(device, &bloomGlarePoolInfo, nullptr,
            &sceneBloomGlareDescriptorPool),
            "vkCreateDescriptorPool(bloom glare)");
        std::array<VkDescriptorSetLayout, BloomGlareDescriptorSetCount>
            bloomGlareSetLayouts {};
        bloomGlareSetLayouts.fill(sceneBloomGlareDescriptorSetLayout);
        std::array<VkDescriptorSet, BloomGlareDescriptorSetCount>
            bloomGlareDescriptorSets {};
        VkDescriptorSetAllocateInfo bloomGlareAllocateInfo {
            VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO
        };
        bloomGlareAllocateInfo.descriptorPool =
            sceneBloomGlareDescriptorPool;
        bloomGlareAllocateInfo.descriptorSetCount =
            BloomGlareDescriptorSetCount;
        bloomGlareAllocateInfo.pSetLayouts =
            bloomGlareSetLayouts.data();
        check(vkAllocateDescriptorSets(device, &bloomGlareAllocateInfo,
            bloomGlareDescriptorSets.data()),
            "vkAllocateDescriptorSets(bloom glare)");
        for (std::size_t index = 0; index < FramesInFlight; ++index) {
            sceneBloomGlareDescriptorSets[index] =
                bloomGlareDescriptorSets[index];
            sceneDirectBloomGlareDescriptorSets[index] =
                bloomGlareDescriptorSets[FramesInFlight + index];
        }

        VkPushConstantRange bloomGlarePushConstant {};
        bloomGlarePushConstant.stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;
        bloomGlarePushConstant.size =
            sizeof(BloomGlarePushConstantsGpu);
        VkPipelineLayoutCreateInfo bloomGlarePipelineLayoutInfo {
            VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO
        };
        bloomGlarePipelineLayoutInfo.setLayoutCount = 1;
        bloomGlarePipelineLayoutInfo.pSetLayouts =
            &sceneBloomGlareDescriptorSetLayout;
        bloomGlarePipelineLayoutInfo.pushConstantRangeCount = 1;
        bloomGlarePipelineLayoutInfo.pPushConstantRanges =
            &bloomGlarePushConstant;
        check(vkCreatePipelineLayout(device,
            &bloomGlarePipelineLayoutInfo, nullptr,
            &sceneBloomGlarePipelineLayout),
            "vkCreatePipelineLayout(bloom glare)");

        // Descriptor set do traçado+resolve de SSR: 6 bindings, todas
        // amostradas so no fragmento (ver ssr_trace_resolve.frag) - mesmo
        // padrao do resolve de TAA acima (um set por frame em voo e por
        // caminho, escrito so ao criar/recriar attachments).
        std::array<VkDescriptorSetLayoutBinding, 6> ssrBindings {};
        for (std::uint32_t index = 0; index < ssrBindings.size(); ++index) {
            ssrBindings[index].binding = index;
            ssrBindings[index].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
            ssrBindings[index].descriptorCount = 1;
            ssrBindings[index].stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;
        }
        VkDescriptorSetLayoutCreateInfo ssrLayoutInfo {
            VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO
        };
        ssrLayoutInfo.bindingCount = static_cast<std::uint32_t>(ssrBindings.size());
        ssrLayoutInfo.pBindings = ssrBindings.data();
        check(vkCreateDescriptorSetLayout(device, &ssrLayoutInfo, nullptr,
            &sceneSsrDescriptorSetLayout), "vkCreateDescriptorSetLayout(ssr)");

        constexpr std::uint32_t SsrDescriptorSetCount = FramesInFlight * 2;
        const VkDescriptorPoolSize ssrPoolSize {
            VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER,
            SsrDescriptorSetCount * static_cast<std::uint32_t>(ssrBindings.size())
        };
        VkDescriptorPoolCreateInfo ssrPoolInfo { VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO };
        ssrPoolInfo.maxSets = SsrDescriptorSetCount;
        ssrPoolInfo.poolSizeCount = 1;
        ssrPoolInfo.pPoolSizes = &ssrPoolSize;
        check(vkCreateDescriptorPool(device, &ssrPoolInfo, nullptr,
            &sceneSsrDescriptorPool), "vkCreateDescriptorPool(ssr)");

        std::array<VkDescriptorSetLayout, SsrDescriptorSetCount> ssrSetLayouts {};
        ssrSetLayouts.fill(sceneSsrDescriptorSetLayout);
        std::array<VkDescriptorSet, SsrDescriptorSetCount> ssrDescriptorSets {};
        VkDescriptorSetAllocateInfo ssrAllocateInfo {
            VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO
        };
        ssrAllocateInfo.descriptorPool = sceneSsrDescriptorPool;
        ssrAllocateInfo.descriptorSetCount = SsrDescriptorSetCount;
        ssrAllocateInfo.pSetLayouts = ssrSetLayouts.data();
        check(vkAllocateDescriptorSets(device, &ssrAllocateInfo, ssrDescriptorSets.data()),
            "vkAllocateDescriptorSets(ssr)");
        for (std::size_t index = 0; index < FramesInFlight; ++index) {
            sceneSsrDescriptorSets[index] = ssrDescriptorSets[index];
            sceneDirectSsrDescriptorSets[index] =
                ssrDescriptorSets[FramesInFlight + index];
        }

        // Layout do traçado+resolve: set 0 = UBO da cena + luzes (direcao do
        // sol, ver reflectionEnvironmentColor em ssr_trace_resolve.frag),
        // set 1 = as 6 texturas acima.
        const std::array<VkDescriptorSetLayout, 2> ssrSetLayoutsForPipeline {
            sceneDescriptorSetLayout, sceneSsrDescriptorSetLayout
        };
        VkPipelineLayoutCreateInfo ssrPipelineLayoutInfo {
            VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO
        };
        ssrPipelineLayoutInfo.setLayoutCount =
            static_cast<std::uint32_t>(ssrSetLayoutsForPipeline.size());
        ssrPipelineLayoutInfo.pSetLayouts = ssrSetLayoutsForPipeline.data();
        check(vkCreatePipelineLayout(device, &ssrPipelineLayoutInfo, nullptr,
            &sceneSsrPipelineLayout), "vkCreatePipelineLayout(ssr)");

        // Descriptor set da composicao de SSR: 2 bindings (reflectancia +
        // historico de SSR ja resolvido) - um unico set dedicado, sem UBO da
        // cena (mesmo padrao de sceneTonemapDescriptorSetLayout, que tambem
        // nao precisa de camera/luzes).
        std::array<VkDescriptorSetLayoutBinding, 2> ssrCompositeBindings {};
        for (std::uint32_t index = 0; index < ssrCompositeBindings.size(); ++index) {
            ssrCompositeBindings[index].binding = index;
            ssrCompositeBindings[index].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
            ssrCompositeBindings[index].descriptorCount = 1;
            ssrCompositeBindings[index].stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;
        }
        VkDescriptorSetLayoutCreateInfo ssrCompositeLayoutInfo {
            VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO
        };
        ssrCompositeLayoutInfo.bindingCount =
            static_cast<std::uint32_t>(ssrCompositeBindings.size());
        ssrCompositeLayoutInfo.pBindings = ssrCompositeBindings.data();
        check(vkCreateDescriptorSetLayout(device, &ssrCompositeLayoutInfo, nullptr,
            &sceneSsrCompositeDescriptorSetLayout), "vkCreateDescriptorSetLayout(ssr composite)");

        constexpr std::uint32_t SsrCompositeDescriptorSetCount = FramesInFlight * 2;
        const VkDescriptorPoolSize ssrCompositePoolSize {
            VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER,
            SsrCompositeDescriptorSetCount
                * static_cast<std::uint32_t>(ssrCompositeBindings.size())
        };
        VkDescriptorPoolCreateInfo ssrCompositePoolInfo {
            VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO
        };
        ssrCompositePoolInfo.maxSets = SsrCompositeDescriptorSetCount;
        ssrCompositePoolInfo.poolSizeCount = 1;
        ssrCompositePoolInfo.pPoolSizes = &ssrCompositePoolSize;
        check(vkCreateDescriptorPool(device, &ssrCompositePoolInfo, nullptr,
            &sceneSsrCompositeDescriptorPool), "vkCreateDescriptorPool(ssr composite)");

        std::array<VkDescriptorSetLayout, SsrCompositeDescriptorSetCount>
            ssrCompositeSetLayouts {};
        ssrCompositeSetLayouts.fill(sceneSsrCompositeDescriptorSetLayout);
        std::array<VkDescriptorSet, SsrCompositeDescriptorSetCount>
            ssrCompositeDescriptorSets {};
        VkDescriptorSetAllocateInfo ssrCompositeAllocateInfo {
            VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO
        };
        ssrCompositeAllocateInfo.descriptorPool = sceneSsrCompositeDescriptorPool;
        ssrCompositeAllocateInfo.descriptorSetCount = SsrCompositeDescriptorSetCount;
        ssrCompositeAllocateInfo.pSetLayouts = ssrCompositeSetLayouts.data();
        check(vkAllocateDescriptorSets(device, &ssrCompositeAllocateInfo,
            ssrCompositeDescriptorSets.data()), "vkAllocateDescriptorSets(ssr composite)");
        for (std::size_t index = 0; index < FramesInFlight; ++index) {
            sceneSsrCompositeDescriptorSets[index] = ssrCompositeDescriptorSets[index];
            sceneDirectSsrCompositeDescriptorSets[index] =
                ssrCompositeDescriptorSets[FramesInFlight + index];
        }
        VkPipelineLayoutCreateInfo ssrCompositePipelineLayoutInfo {
            VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO
        };
        ssrCompositePipelineLayoutInfo.setLayoutCount = 1;
        ssrCompositePipelineLayoutInfo.pSetLayouts = &sceneSsrCompositeDescriptorSetLayout;
        check(vkCreatePipelineLayout(device, &ssrCompositePipelineLayoutInfo, nullptr,
            &sceneSsrCompositePipelineLayout), "vkCreatePipelineLayout(ssr composite)");

        std::array<VkDescriptorSetLayoutBinding, 1> oceanBindings {};
        oceanBindings[0].binding = 0;
        oceanBindings[0].descriptorType =
            VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
        oceanBindings[0].descriptorCount = 1;
        oceanBindings[0].stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;
        VkDescriptorSetLayoutCreateInfo oceanLayoutInfo {
            VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO
        };
        oceanLayoutInfo.bindingCount =
            static_cast<std::uint32_t>(oceanBindings.size());
        oceanLayoutInfo.pBindings = oceanBindings.data();
        check(vkCreateDescriptorSetLayout(device, &oceanLayoutInfo, nullptr,
            &sceneOceanDescriptorSetLayout),
            "vkCreateDescriptorSetLayout(ocean)");

        constexpr std::uint32_t OceanDescriptorSetCount = FramesInFlight * 2;
        const VkDescriptorPoolSize oceanPoolSize {
            VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER,
            OceanDescriptorSetCount
                * static_cast<std::uint32_t>(oceanBindings.size())
        };
        VkDescriptorPoolCreateInfo oceanPoolInfo {
            VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO
        };
        oceanPoolInfo.maxSets = OceanDescriptorSetCount;
        oceanPoolInfo.poolSizeCount = 1;
        oceanPoolInfo.pPoolSizes = &oceanPoolSize;
        check(vkCreateDescriptorPool(device, &oceanPoolInfo, nullptr,
            &sceneOceanDescriptorPool), "vkCreateDescriptorPool(ocean)");

        std::array<VkDescriptorSetLayout, OceanDescriptorSetCount>
            oceanSetLayouts {};
        oceanSetLayouts.fill(sceneOceanDescriptorSetLayout);
        std::array<VkDescriptorSet, OceanDescriptorSetCount>
            oceanDescriptorSets {};
        VkDescriptorSetAllocateInfo oceanAllocateInfo {
            VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO
        };
        oceanAllocateInfo.descriptorPool = sceneOceanDescriptorPool;
        oceanAllocateInfo.descriptorSetCount = OceanDescriptorSetCount;
        oceanAllocateInfo.pSetLayouts = oceanSetLayouts.data();
        check(vkAllocateDescriptorSets(device, &oceanAllocateInfo,
            oceanDescriptorSets.data()), "vkAllocateDescriptorSets(ocean)");
        for (std::size_t index = 0; index < FramesInFlight; ++index) {
            sceneOceanDescriptorSets[index] = oceanDescriptorSets[index];
            sceneDirectOceanDescriptorSets[index] =
                oceanDescriptorSets[FramesInFlight + index];
        }

        const std::array<VkDescriptorSetLayout, 2>
            oceanSetLayoutsForPipeline {
            sceneDescriptorSetLayout, sceneOceanDescriptorSetLayout
        };
        VkPushConstantRange oceanPushConstantRange {};
        oceanPushConstantRange.stageFlags = VK_SHADER_STAGE_VERTEX_BIT
            | VK_SHADER_STAGE_FRAGMENT_BIT;
        oceanPushConstantRange.offset = 0;
        oceanPushConstantRange.size = sizeof(OceanPushConstantsGpu);
        VkPipelineLayoutCreateInfo oceanPipelineLayoutInfo {
            VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO
        };
        oceanPipelineLayoutInfo.setLayoutCount =
            static_cast<std::uint32_t>(oceanSetLayoutsForPipeline.size());
        oceanPipelineLayoutInfo.pSetLayouts =
            oceanSetLayoutsForPipeline.data();
        oceanPipelineLayoutInfo.pushConstantRangeCount = 1;
        oceanPipelineLayoutInfo.pPushConstantRanges =
            &oceanPushConstantRange;
        check(vkCreatePipelineLayout(device, &oceanPipelineLayoutInfo,
            nullptr, &sceneOceanPipelineLayout),
            "vkCreatePipelineLayout(ocean)");

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
        // 2 attachments (MRT): cor HDR + vetores de movimento. Normal,
        // rugosidade e reflectância pertenciam ao SSR/GTAO já retirados do
        // frame ativo e desperdiçavam banda em todos os pixels.
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
        // O prepass ja marcou todos os pixels cobertos por geometria. Como o
        // fullscreen triangle do ceu fica no clear depth (zero no reversed-Z),
        // EQUAL evita executar nuvens/estrelas atras do mundo sem alterar a
        // imagem final.
        skyDepth.depthTestEnable = VK_TRUE;
        skyDepth.depthWriteEnable = VK_FALSE;
        skyDepth.depthCompareOp = VK_COMPARE_OP_EQUAL;
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

        // Medição e adaptação de exposição: o mesmo triângulo fullscreen,
        // mas num attachment 1x1 HDR. Somente 24 amostras são executadas por
        // quadro, independentemente da resolução da cena.
        VkPipelineRenderingCreateInfo autoExposureRendering {
            VK_STRUCTURE_TYPE_PIPELINE_RENDERING_CREATE_INFO
        };
        autoExposureRendering.colorAttachmentCount = 1;
        autoExposureRendering.pColorAttachmentFormats =
            &SceneAutoExposureFormat;
        std::array<VkPipelineShaderStageCreateInfo, 2> autoExposureStages {};
        autoExposureStages[0] = tonemapStages[0];
        autoExposureStages[1].sType =
            VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
        autoExposureStages[1].stage = VK_SHADER_STAGE_FRAGMENT_BIT;
        autoExposureStages[1].module = sceneAutoExposureFragmentModule;
        autoExposureStages[1].pName = "main";
        VkGraphicsPipelineCreateInfo autoExposurePipelineInfo =
            tonemapPipelineInfo;
        autoExposurePipelineInfo.pNext = &autoExposureRendering;
        autoExposurePipelineInfo.stageCount =
            static_cast<std::uint32_t>(autoExposureStages.size());
        autoExposurePipelineInfo.pStages = autoExposureStages.data();
        autoExposurePipelineInfo.layout = sceneAutoExposurePipelineLayout;
        check(vkCreateGraphicsPipelines(device, VK_NULL_HANDLE, 1,
            &autoExposurePipelineInfo, nullptr, &sceneAutoExposurePipeline),
            "vkCreateGraphicsPipelines(scene auto exposure)");

        VkPipelineRenderingCreateInfo gtaoRendering {
            VK_STRUCTURE_TYPE_PIPELINE_RENDERING_CREATE_INFO
        };
        gtaoRendering.colorAttachmentCount = 1;
        gtaoRendering.pColorAttachmentFormats =
            &SceneAmbientOcclusionFormat;
        std::array<VkPipelineShaderStageCreateInfo, 2> gtaoStages {};
        gtaoStages[0] = tonemapStages[0];
        gtaoStages[1].sType =
            VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
        gtaoStages[1].stage = VK_SHADER_STAGE_FRAGMENT_BIT;
        gtaoStages[1].module = sceneGtaoFragmentModule;
        gtaoStages[1].pName = "main";
        VkGraphicsPipelineCreateInfo gtaoPipelineInfo =
            tonemapPipelineInfo;
        gtaoPipelineInfo.pNext = &gtaoRendering;
        gtaoPipelineInfo.stageCount =
            static_cast<std::uint32_t>(gtaoStages.size());
        gtaoPipelineInfo.pStages = gtaoStages.data();
        gtaoPipelineInfo.layout = sceneGtaoPipelineLayout;
        check(vkCreateGraphicsPipelines(device, VK_NULL_HANDLE, 1,
            &gtaoPipelineInfo, nullptr, &sceneGtaoPipeline),
            "vkCreateGraphicsPipelines(scene gtao)");

        VkPipelineRenderingCreateInfo bloomGlareRendering {
            VK_STRUCTURE_TYPE_PIPELINE_RENDERING_CREATE_INFO
        };
        bloomGlareRendering.colorAttachmentCount = 1;
        bloomGlareRendering.pColorAttachmentFormats =
            &SceneBloomGlareFormat;
        std::array<VkPipelineShaderStageCreateInfo, 2>
            bloomGlareStages {};
        bloomGlareStages[0] = tonemapStages[0];
        bloomGlareStages[1].sType =
            VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
        bloomGlareStages[1].stage = VK_SHADER_STAGE_FRAGMENT_BIT;
        bloomGlareStages[1].module = sceneBloomGlareFragmentModule;
        bloomGlareStages[1].pName = "main";
        VkGraphicsPipelineCreateInfo bloomGlarePipelineInfo =
            tonemapPipelineInfo;
        bloomGlarePipelineInfo.pNext = &bloomGlareRendering;
        bloomGlarePipelineInfo.stageCount =
            static_cast<std::uint32_t>(bloomGlareStages.size());
        bloomGlarePipelineInfo.pStages = bloomGlareStages.data();
        bloomGlarePipelineInfo.layout = sceneBloomGlarePipelineLayout;
        check(vkCreateGraphicsPipelines(device, VK_NULL_HANDLE, 1,
            &bloomGlarePipelineInfo, nullptr, &sceneBloomGlarePipeline),
            "vkCreateGraphicsPipelines(scene bloom glare)");

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

        // Pipeline de traçado+resolve de SSR: mesmo triangulo cheio de tela
        // (reaproveita sceneTonemapVertexModule), mas escrevendo no formato
        // do historico de SSR (SceneSsrHistoryFormat), nao no HDR - a
        // viewport/scissor dinamica deste pipeline usa a MEIA resolucao no
        // render loop, nao o formato em si (que so declara o layout).
        VkPipelineRenderingCreateInfo ssrRendering { VK_STRUCTURE_TYPE_PIPELINE_RENDERING_CREATE_INFO };
        ssrRendering.colorAttachmentCount = 1;
        ssrRendering.pColorAttachmentFormats = &SceneSsrHistoryFormat;
        VkPipelineDepthStencilStateCreateInfo ssrDepth {
            VK_STRUCTURE_TYPE_PIPELINE_DEPTH_STENCIL_STATE_CREATE_INFO
        };
        ssrDepth.depthTestEnable = VK_FALSE;
        ssrDepth.depthWriteEnable = VK_FALSE;
        std::array<VkPipelineShaderStageCreateInfo, 2> ssrStages {};
        ssrStages[0].sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
        ssrStages[0].stage = VK_SHADER_STAGE_VERTEX_BIT;
        ssrStages[0].module = sceneTonemapVertexModule;
        ssrStages[0].pName = "main";
        ssrStages[1].sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
        ssrStages[1].stage = VK_SHADER_STAGE_FRAGMENT_BIT;
        ssrStages[1].module = sceneSsrFragmentModule;
        ssrStages[1].pName = "main";
        VkGraphicsPipelineCreateInfo ssrPipelineInfo = scenePipelineInfo;
        ssrPipelineInfo.pNext = &ssrRendering;
        ssrPipelineInfo.pDepthStencilState = &ssrDepth;
        ssrPipelineInfo.pColorBlendState = &singleColorBlend;
        ssrPipelineInfo.stageCount = static_cast<std::uint32_t>(ssrStages.size());
        ssrPipelineInfo.pStages = ssrStages.data();
        ssrPipelineInfo.layout = sceneSsrPipelineLayout;
        check(vkCreateGraphicsPipelines(device, VK_NULL_HANDLE, 1, &ssrPipelineInfo,
            nullptr, &sceneSsrPipeline), "vkCreateGraphicsPipelines(scene ssr)");

        // Pipeline de composicao de SSR: soma (nao substitui) no HDR em
        // resolucao cheia - blend ADITIVO de verdade (srcFactor=dstFactor=UM),
        // ao contrario de todo outro pipeline de tela cheia deste arquivo
        // (todos REPLACE). Ver ssr_composite.frag.
        VkPipelineColorBlendAttachmentState ssrCompositeBlendAttachment {};
        ssrCompositeBlendAttachment.blendEnable = VK_TRUE;
        ssrCompositeBlendAttachment.srcColorBlendFactor = VK_BLEND_FACTOR_ONE;
        ssrCompositeBlendAttachment.dstColorBlendFactor = VK_BLEND_FACTOR_ONE;
        ssrCompositeBlendAttachment.colorBlendOp = VK_BLEND_OP_ADD;
        ssrCompositeBlendAttachment.srcAlphaBlendFactor = VK_BLEND_FACTOR_ONE;
        ssrCompositeBlendAttachment.dstAlphaBlendFactor = VK_BLEND_FACTOR_ONE;
        ssrCompositeBlendAttachment.alphaBlendOp = VK_BLEND_OP_ADD;
        ssrCompositeBlendAttachment.colorWriteMask = VK_COLOR_COMPONENT_R_BIT
            | VK_COLOR_COMPONENT_G_BIT | VK_COLOR_COMPONENT_B_BIT
            | VK_COLOR_COMPONENT_A_BIT;
        VkPipelineColorBlendStateCreateInfo ssrCompositeColorBlend {
            VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO
        };
        ssrCompositeColorBlend.attachmentCount = 1;
        ssrCompositeColorBlend.pAttachments = &ssrCompositeBlendAttachment;
        VkPipelineRenderingCreateInfo ssrCompositeRendering {
            VK_STRUCTURE_TYPE_PIPELINE_RENDERING_CREATE_INFO
        };
        ssrCompositeRendering.colorAttachmentCount = 1;
        ssrCompositeRendering.pColorAttachmentFormats = &SceneHdrColorFormat;
        VkPipelineDepthStencilStateCreateInfo ssrCompositeDepth {
            VK_STRUCTURE_TYPE_PIPELINE_DEPTH_STENCIL_STATE_CREATE_INFO
        };
        ssrCompositeDepth.depthTestEnable = VK_FALSE;
        ssrCompositeDepth.depthWriteEnable = VK_FALSE;
        std::array<VkPipelineShaderStageCreateInfo, 2> ssrCompositeStages {};
        ssrCompositeStages[0].sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
        ssrCompositeStages[0].stage = VK_SHADER_STAGE_VERTEX_BIT;
        ssrCompositeStages[0].module = sceneTonemapVertexModule;
        ssrCompositeStages[0].pName = "main";
        ssrCompositeStages[1].sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
        ssrCompositeStages[1].stage = VK_SHADER_STAGE_FRAGMENT_BIT;
        ssrCompositeStages[1].module = sceneSsrCompositeFragmentModule;
        ssrCompositeStages[1].pName = "main";
        VkGraphicsPipelineCreateInfo ssrCompositePipelineInfo = scenePipelineInfo;
        ssrCompositePipelineInfo.pNext = &ssrCompositeRendering;
        ssrCompositePipelineInfo.pDepthStencilState = &ssrCompositeDepth;
        ssrCompositePipelineInfo.pColorBlendState = &ssrCompositeColorBlend;
        ssrCompositePipelineInfo.stageCount =
            static_cast<std::uint32_t>(ssrCompositeStages.size());
        ssrCompositePipelineInfo.pStages = ssrCompositeStages.data();
        ssrCompositePipelineInfo.layout = sceneSsrCompositePipelineLayout;
        check(vkCreateGraphicsPipelines(device, VK_NULL_HANDLE, 1, &ssrCompositePipelineInfo,
            nullptr, &sceneSsrCompositePipeline), "vkCreateGraphicsPipelines(scene ssr composite)");

        VkVertexInputBindingDescription oceanVertexBinding {};
        oceanVertexBinding.binding = 0;
        oceanVertexBinding.stride = sizeof(float) * 2;
        oceanVertexBinding.inputRate = VK_VERTEX_INPUT_RATE_VERTEX;
        VkVertexInputAttributeDescription oceanVertexAttribute {};
        oceanVertexAttribute.location = 0;
        oceanVertexAttribute.binding = 0;
        oceanVertexAttribute.format = VK_FORMAT_R32G32_SFLOAT;
        oceanVertexAttribute.offset = 0;
        VkPipelineVertexInputStateCreateInfo oceanVertexInput {
            VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO
        };
        oceanVertexInput.vertexBindingDescriptionCount = 1;
        oceanVertexInput.pVertexBindingDescriptions = &oceanVertexBinding;
        oceanVertexInput.vertexAttributeDescriptionCount = 1;
        oceanVertexInput.pVertexAttributeDescriptions =
            &oceanVertexAttribute;

        VkPipelineColorBlendAttachmentState oceanBlendAttachment {};
        oceanBlendAttachment.colorWriteMask = VK_COLOR_COMPONENT_R_BIT
            | VK_COLOR_COMPONENT_G_BIT | VK_COLOR_COMPONENT_B_BIT
            | VK_COLOR_COMPONENT_A_BIT;
        oceanBlendAttachment.blendEnable = VK_TRUE;
        oceanBlendAttachment.srcColorBlendFactor =
            VK_BLEND_FACTOR_SRC_ALPHA;
        oceanBlendAttachment.dstColorBlendFactor =
            VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA;
        oceanBlendAttachment.colorBlendOp = VK_BLEND_OP_ADD;
        oceanBlendAttachment.srcAlphaBlendFactor = VK_BLEND_FACTOR_ONE;
        oceanBlendAttachment.dstAlphaBlendFactor =
            VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA;
        oceanBlendAttachment.alphaBlendOp = VK_BLEND_OP_ADD;
        VkPipelineColorBlendAttachmentState oceanMotionBlendAttachment {};
        oceanMotionBlendAttachment.colorWriteMask =
            VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT;
        const std::array<VkPipelineColorBlendAttachmentState, 2>
            oceanBlendAttachments {
                oceanBlendAttachment, oceanMotionBlendAttachment
            };
        VkPipelineColorBlendStateCreateInfo oceanColorBlend {
            VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO
        };
        oceanColorBlend.attachmentCount =
            static_cast<std::uint32_t>(oceanBlendAttachments.size());
        oceanColorBlend.pAttachments = oceanBlendAttachments.data();

        VkPipelineRenderingCreateInfo oceanRendering {
            VK_STRUCTURE_TYPE_PIPELINE_RENDERING_CREATE_INFO
        };
        const std::array<VkFormat, 2> oceanColorFormats {
            SceneHdrColorFormat, SceneMotionVectorFormat
        };
        oceanRendering.colorAttachmentCount =
            static_cast<std::uint32_t>(oceanColorFormats.size());
        oceanRendering.pColorAttachmentFormats = oceanColorFormats.data();
        oceanRendering.depthAttachmentFormat = SceneDepthFormat;

        VkPipelineDepthStencilStateCreateInfo oceanDepth {
            VK_STRUCTURE_TYPE_PIPELINE_DEPTH_STENCIL_STATE_CREATE_INFO
        };
        oceanDepth.depthTestEnable = VK_TRUE;
        oceanDepth.depthWriteEnable = VK_FALSE;
        oceanDepth.depthCompareOp = VK_COMPARE_OP_GREATER_OR_EQUAL;

        std::array<VkPipelineShaderStageCreateInfo, 2> oceanStages {};
        oceanStages[0].sType =
            VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
        oceanStages[0].stage = VK_SHADER_STAGE_VERTEX_BIT;
        oceanStages[0].module = sceneOceanVertexModule;
        oceanStages[0].pName = "main";
        oceanStages[1].sType =
            VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
        oceanStages[1].stage = VK_SHADER_STAGE_FRAGMENT_BIT;
        oceanStages[1].module = sceneOceanFragmentModule;
        oceanStages[1].pName = "main";

        VkPipelineRasterizationStateCreateInfo oceanRasterizer =
            sceneRasterizer;
        oceanRasterizer.cullMode = VK_CULL_MODE_NONE;
        VkGraphicsPipelineCreateInfo oceanPipelineInfo = scenePipelineInfo;
        oceanPipelineInfo.pNext = &oceanRendering;
        oceanPipelineInfo.pVertexInputState = &oceanVertexInput;
        oceanPipelineInfo.pRasterizationState = &oceanRasterizer;
        oceanPipelineInfo.pDepthStencilState = &oceanDepth;
        oceanPipelineInfo.pColorBlendState = &oceanColorBlend;
        oceanPipelineInfo.stageCount =
            static_cast<std::uint32_t>(oceanStages.size());
        oceanPipelineInfo.pStages = oceanStages.data();
        oceanPipelineInfo.layout = sceneOceanPipelineLayout;
        check(vkCreateGraphicsPipelines(device, VK_NULL_HANDLE, 1,
            &oceanPipelineInfo, nullptr, &sceneOceanPipeline),
            "vkCreateGraphicsPipelines(scene ocean)");

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
        // 16x preserva frequências que já são menores que um pixel em
        // superfícies muito inclinadas e pode realçar moiré. 8x com um
        // pequeno bias positivo conserva nitidez próxima e estabiliza chão/
        // paredes distantes.
        materialSamplerInfo.maxAnisotropy = std::min(
            8.0f, deviceProperties.limits.maxSamplerAnisotropy);
        materialSamplerInfo.mipLodBias = 0.40f;
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

        constexpr std::uint32_t PlanarDescriptorSetCount =
            FramesInFlight * 2;
        const VkDescriptorPoolSize planarPoolSize {
            VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER,
            PlanarDescriptorSetCount
        };
        VkDescriptorPoolCreateInfo planarPoolInfo {
            VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO
        };
        planarPoolInfo.maxSets = PlanarDescriptorSetCount;
        planarPoolInfo.poolSizeCount = 1;
        planarPoolInfo.pPoolSizes = &planarPoolSize;
        check(vkCreateDescriptorPool(device, &planarPoolInfo, nullptr,
            &scenePlanarReflectionDescriptorPool),
            "vkCreateDescriptorPool(planar reflection)");
        std::array<VkDescriptorSetLayout, PlanarDescriptorSetCount>
            planarSetLayouts {};
        planarSetLayouts.fill(materialDescriptorSetLayout);
        std::array<VkDescriptorSet, PlanarDescriptorSetCount>
            planarDescriptorSets {};
        VkDescriptorSetAllocateInfo planarAllocateInfo {
            VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO
        };
        planarAllocateInfo.descriptorPool =
            scenePlanarReflectionDescriptorPool;
        planarAllocateInfo.descriptorSetCount = PlanarDescriptorSetCount;
        planarAllocateInfo.pSetLayouts = planarSetLayouts.data();
        check(vkAllocateDescriptorSets(device, &planarAllocateInfo,
            planarDescriptorSets.data()),
            "vkAllocateDescriptorSets(planar reflection)");
        for (std::size_t index = 0; index < FramesInFlight; ++index) {
            scenePlanarReflectionDescriptorSets[index] =
                planarDescriptorSets[index];
            sceneDirectPlanarReflectionDescriptorSets[index] =
                planarDescriptorSets[FramesInFlight + index];
        }

        // Nenhum push constant aqui: dados por instancia (posicao/orientacao/
        // metallic/roughness/flags) ja chegam pelo vertex buffer de instancia
        // (binding 1, ver meshBinding abaixo).
        const std::array<VkDescriptorSetLayout, 4> meshSetLayouts {
            sceneDescriptorSetLayout, materialDescriptorSetLayout,
            materialDescriptorSetLayout, materialDescriptorSetLayout
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
        meshBinding[0].stride = sizeof(MeshVertex3D);
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
        std::array<VkVertexInputAttributeDescription, 15> meshAttributes { {
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
                offsetof(SceneMeshInstanceGpu, previousOrientationZ) },
            { 13, 0, VK_FORMAT_R32G32B32A32_UINT, offsetof(MeshVertex3D, joints) },
            { 14, 0, VK_FORMAT_R32G32B32A32_SFLOAT, offsetof(MeshVertex3D, weights) }
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
        // O passe de cor roda depois do pre-pass de profundidade
        // (sceneMeshDepthPrepassPipeline, logo abaixo) - so testa (EQUAL)
        // sem escrever de novo, pra aproveitar o early-Z ja resolvido pelo
        // pre-pass e nao pagar o fragment shader completo (BRDF + shadow
        // lookup) em fragmentos que vao ser sobrescritos por outro objeto.
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
        std::array<VkDescriptorImageInfo, ShadowCascadeCount> shadowInfos {};
        // Mesmas N imagens do array acima, mas com o sampler SEM comparacao
        // (binding 3, ver bindings[3] em createScene3DResources) - so pra
        // busca de bloqueadores do PCSS (ver shadowVisibility em
        // scene3d_mesh.frag).
        std::array<VkDescriptorImageInfo, ShadowCascadeCount> shadowRawInfos {};
        for (std::uint32_t cascade = 0; cascade < ShadowCascadeCount; ++cascade) {
            const VkExtent2D shadowExtent {
                ShadowCascadeMapSizes[cascade],
                ShadowCascadeMapSizes[cascade]
            };
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
        std::array<VkWriteDescriptorSet, FramesInFlight * 4> writes {};
        for (std::size_t index = 0; index < FramesInFlight; ++index) {
            const std::size_t base = index * 4;
            writes[base].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
            writes[base].dstSet = sceneDescriptorSets[index];
            writes[base].dstBinding = 1;
            writes[base].descriptorCount = ShadowCascadeCount;
            writes[base].descriptorType =
                VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
            writes[base].pImageInfo = shadowInfos.data();

            writes[base + 1] = writes[base];
            writes[base + 1].dstBinding = 3;
            writes[base + 1].pImageInfo = shadowRawInfos.data();

            writes[base + 2] = writes[base];
            writes[base + 2].dstSet =
                scenePlanarCameraDescriptorSets[index];
            writes[base + 3] = writes[base + 1];
            writes[base + 3].dstSet =
                scenePlanarCameraDescriptorSets[index];
        }
        vkUpdateDescriptorSets(device, static_cast<std::uint32_t>(writes.size()),
            writes.data(), 0, nullptr);
    }

    void updateTemporalDescriptorSets(VkImageView hdrColorView,
        VkImageView depthView, VkImageView motionVectorView,
        const std::array<VkImageView, FramesInFlight>& historyViews,
        const std::array<VkImageView, FramesInFlight>& exposureViews,
        const std::array<VkImageView, FramesInFlight>& gtaoViews,
        const std::array<VkImageView, FramesInFlight>& bloomGlareViews,
        const std::array<VkDescriptorSet, FramesInFlight>& resolveSets,
        const std::array<VkDescriptorSet, FramesInFlight>& tonemapSets,
        const std::array<VkDescriptorSet, FramesInFlight>&
            tonemapWithoutTaaSets) {
        // Cada slot referencia de forma imutavel os attachments que usara:
        // resolve[i] le history[i-1] e escreve history[i]; tonemap[i] le o
        // history[i] recem-resolvido. Como os sets nao mudam durante frames
        // em voo, a GPU nunca observa uma troca de binding pela metade.
        for (std::size_t frameIndex = 0; frameIndex < FramesInFlight;
            ++frameIndex) {
            const std::size_t historyReadIndex =
                (frameIndex + FramesInFlight - 1) % FramesInFlight;
            std::array<VkDescriptorImageInfo, 9> imageInfos {};
            imageInfos[0] = { sceneTonemapSampler, hdrColorView,
                VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL };
            imageInfos[1] = { sceneDepthSampleSampler, depthView,
                VK_IMAGE_LAYOUT_DEPTH_STENCIL_READ_ONLY_OPTIMAL };
            imageInfos[2] = { sceneTonemapSampler, motionVectorView,
                VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL };
            imageInfos[3] = { sceneTaaHistorySampler,
                historyViews[historyReadIndex],
                VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL };
            imageInfos[4] = { sceneTaaHistorySampler,
                gtaoViews[frameIndex],
                VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL };
            imageInfos[5] = { sceneTonemapSampler, historyViews[frameIndex],
                VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL };
            imageInfos[6] = { sceneTonemapSampler,
                exposureViews[historyReadIndex],
                VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL };
            imageInfos[7] = { sceneTonemapSampler,
                exposureViews[frameIndex],
                VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL };
            imageInfos[8] = { sceneTaaHistorySampler,
                bloomGlareViews[frameIndex],
                VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL };

            std::array<VkWriteDescriptorSet, 17> writes {};
            for (std::size_t binding = 0; binding < 5; ++binding) {
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
            writes[5].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
            writes[5].dstSet = tonemapSets[frameIndex];
            writes[5].dstBinding = 0;
            writes[5].descriptorCount = 1;
            writes[5].descriptorType =
                VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
            writes[5].pImageInfo = &imageInfos[5];
            for (std::size_t binding = 1; binding < 3; ++binding) {
                writes[5 + binding].sType =
                    VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
                writes[5 + binding].dstSet = tonemapSets[frameIndex];
                writes[5 + binding].dstBinding =
                    static_cast<std::uint32_t>(binding);
                writes[5 + binding].descriptorCount = 1;
                writes[5 + binding].descriptorType =
                    VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
                writes[5 + binding].pImageInfo =
                    &imageInfos[5 + binding];
            }
            writes[8].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
            writes[8].dstSet = tonemapSets[frameIndex];
            writes[8].dstBinding = 3;
            writes[8].descriptorCount = 1;
            writes[8].descriptorType =
                VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
            writes[8].pImageInfo = &imageInfos[8];
            writes[9].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
            writes[9].dstSet = tonemapSets[frameIndex];
            writes[9].dstBinding = 4;
            writes[9].descriptorCount = 1;
            writes[9].descriptorType =
                VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
            writes[9].pImageInfo = &imageInfos[1];
            writes[10].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
            writes[10].dstSet = tonemapSets[frameIndex];
            writes[10].dstBinding = 5;
            writes[10].descriptorCount = 1;
            writes[10].descriptorType =
                VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
            writes[10].pImageInfo = &imageInfos[2];

            // Variante do mesmo set que lê diretamente o HDR atual. Ela
            // permite ao perfil Desempenho remover o resolve temporal
            // inteiro sem reescrever descriptors ainda em uso pela GPU.
            // Os outros cinco bindings permanecem idênticos.
            writes[11] = writes[5];
            writes[11].dstSet = tonemapWithoutTaaSets[frameIndex];
            writes[11].pImageInfo = &imageInfos[0];
            for (std::size_t binding = 1; binding < 6; ++binding) {
                writes[11 + binding] = writes[5 + binding];
                writes[11 + binding].dstSet =
                    tonemapWithoutTaaSets[frameIndex];
            }
            vkUpdateDescriptorSets(device,
                static_cast<std::uint32_t>(writes.size()), writes.data(),
                0, nullptr);
        }
    }

    void updateBloomGlareDescriptorSets(VkImageView depthView,
        VkImageView hdrColorView,
        const std::array<VkDescriptorSet, FramesInFlight>& descriptorSets) {
        for (std::size_t frameIndex = 0; frameIndex < FramesInFlight;
            ++frameIndex) {
            const std::array<VkDescriptorImageInfo, 2> imageInfos {{
                { sceneTaaHistorySampler, hdrColorView,
                    VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL },
                { sceneDepthSampleSampler, depthView,
                    VK_IMAGE_LAYOUT_DEPTH_STENCIL_READ_ONLY_OPTIMAL }
            }};
            std::array<VkWriteDescriptorSet, 2> writes {};
            for (std::size_t binding = 0; binding < writes.size();
                ++binding) {
                writes[binding].sType =
                    VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
                writes[binding].dstSet = descriptorSets[frameIndex];
                writes[binding].dstBinding =
                    static_cast<std::uint32_t>(binding);
                writes[binding].descriptorCount = 1;
                writes[binding].descriptorType =
                    VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
                writes[binding].pImageInfo = &imageInfos[binding];
            }
            vkUpdateDescriptorSets(device,
                static_cast<std::uint32_t>(writes.size()), writes.data(),
                0, nullptr);
        }
    }

    void updateGtaoDescriptorSets(VkImageView depthView,
        VkImageView normalRoughnessView,
        const std::array<VkDescriptorSet, FramesInFlight>& gtaoSets) {
        const std::array<VkDescriptorImageInfo, 2> imageInfos { {
            { sceneDepthSampleSampler, depthView,
                VK_IMAGE_LAYOUT_DEPTH_STENCIL_READ_ONLY_OPTIMAL },
            { sceneTonemapSampler, normalRoughnessView,
                VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL }
        } };
        for (VkDescriptorSet set : gtaoSets) {
            std::array<VkWriteDescriptorSet, 2> writes {};
            for (std::uint32_t binding = 0;
                binding < writes.size(); ++binding) {
                writes[binding].sType =
                    VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
                writes[binding].dstSet = set;
                writes[binding].dstBinding = binding;
                writes[binding].descriptorCount = 1;
                writes[binding].descriptorType =
                    VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
                writes[binding].pImageInfo = &imageInfos[binding];
            }
            vkUpdateDescriptorSets(device,
                static_cast<std::uint32_t>(writes.size()),
                writes.data(), 0, nullptr);
        }
    }

    // Mesmo raciocinio de updateTemporalDescriptorSets acima, para os dois
    // passes de SSR (ver ssr_trace_resolve.frag/ssr_composite.frag): sets
    // pre-vinculados uma vez por slot em voo, nunca reescritos durante um
    // frame - traceSets[i] le o historico meia-resolucao do slot OPOSTO
    // (reprojetado) e escreve no slot i; compositeSets[i], executado depois
    // no MESMO frame real, le o slot i recem-escrito.
    void updateSsrDescriptorSets(VkImageView hdrColorView, VkImageView depthView,
        VkImageView normalRoughnessView, VkImageView reflectanceView,
        VkImageView motionVectorView,
        const std::array<VkImageView, FramesInFlight>& ssrHistoryViews,
        const std::array<VkDescriptorSet, FramesInFlight>& ssrTraceSets,
        const std::array<VkDescriptorSet, FramesInFlight>& ssrCompositeSets) {
        for (std::size_t frameIndex = 0; frameIndex < FramesInFlight;
            ++frameIndex) {
            const std::size_t historyReadIndex =
                (frameIndex + FramesInFlight - 1) % FramesInFlight;
            std::array<VkDescriptorImageInfo, 6> traceImageInfos {};
            traceImageInfos[0] = { sceneTonemapSampler, hdrColorView,
                VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL };
            traceImageInfos[1] = { sceneDepthSampleSampler, depthView,
                VK_IMAGE_LAYOUT_DEPTH_STENCIL_READ_ONLY_OPTIMAL };
            traceImageInfos[2] = { sceneTonemapSampler, normalRoughnessView,
                VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL };
            traceImageInfos[3] = { sceneTonemapSampler, reflectanceView,
                VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL };
            traceImageInfos[4] = { sceneTonemapSampler, motionVectorView,
                VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL };
            // Sampler LINEAR (nao sceneTonemapSampler/NEAREST): o historico
            // e lido em previousTexCoord, resultado de reprojecao por vetor
            // de movimento, que quase nunca cai exatamente num texel - o
            // mesmo raciocinio do historico de TAA (ver
            // sceneTaaHistorySampler acima).
            traceImageInfos[5] = { sceneTaaHistorySampler,
                ssrHistoryViews[historyReadIndex],
                VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL };
            std::array<VkWriteDescriptorSet, 6> traceWrites {};
            for (std::size_t binding = 0; binding < traceWrites.size(); ++binding) {
                traceWrites[binding].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
                traceWrites[binding].dstSet = ssrTraceSets[frameIndex];
                traceWrites[binding].dstBinding = static_cast<std::uint32_t>(binding);
                traceWrites[binding].descriptorCount = 1;
                traceWrites[binding].descriptorType =
                    VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
                traceWrites[binding].pImageInfo = &traceImageInfos[binding];
            }
            vkUpdateDescriptorSets(device,
                static_cast<std::uint32_t>(traceWrites.size()), traceWrites.data(),
                0, nullptr);

            std::array<VkDescriptorImageInfo, 2> compositeImageInfos {};
            compositeImageInfos[0] = { sceneTonemapSampler, reflectanceView,
                VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL };
            // Sampler LINEAR (nao sceneTonemapSampler/NEAREST): este binding
            // e o unico neste passe que faz upsample de verdade (MEIA
            // resolucao para CHEIA, ver sceneSsrHistoryImage) - NEAREST
            // alargaria cada texel do traçado em blocos 2x2 visiveis no
            // resultado final, a causa do aspecto "quadriculado/quebrado"
            // reportado no espelho.
            compositeImageInfos[1] = { sceneTaaHistorySampler,
                ssrHistoryViews[frameIndex],
                VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL };
            std::array<VkWriteDescriptorSet, 2> compositeWrites {};
            for (std::size_t binding = 0; binding < compositeWrites.size(); ++binding) {
                compositeWrites[binding].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
                compositeWrites[binding].dstSet = ssrCompositeSets[frameIndex];
                compositeWrites[binding].dstBinding = static_cast<std::uint32_t>(binding);
                compositeWrites[binding].descriptorCount = 1;
                compositeWrites[binding].descriptorType =
                    VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
                compositeWrites[binding].pImageInfo = &compositeImageInfos[binding];
            }
            vkUpdateDescriptorSets(device,
                static_cast<std::uint32_t>(compositeWrites.size()), compositeWrites.data(),
                0, nullptr);
        }
    }

    void updatePlanarReflectionDescriptorSets(VkImageView reflectionView,
        const std::array<VkDescriptorSet, FramesInFlight>& descriptorSets) {
        VkDescriptorImageInfo imageInfo {
            sceneTaaHistorySampler, reflectionView,
            VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL
        };
        std::array<VkWriteDescriptorSet, FramesInFlight> writes {};
        for (std::size_t index = 0; index < FramesInFlight; ++index) {
            writes[index].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
            writes[index].dstSet = descriptorSets[index];
            writes[index].dstBinding = 0;
            writes[index].descriptorCount = 1;
            writes[index].descriptorType =
                VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
            writes[index].pImageInfo = &imageInfo;
        }
        vkUpdateDescriptorSets(device,
            static_cast<std::uint32_t>(writes.size()), writes.data(),
            0, nullptr);
    }

    void updateOceanDescriptorSets(VkImageView depthView,
        const std::array<VkDescriptorSet, FramesInFlight>& descriptorSets) {
        VkDescriptorImageInfo imageInfo {
            sceneDepthSampleSampler, depthView,
            VK_IMAGE_LAYOUT_DEPTH_STENCIL_READ_ONLY_OPTIMAL
        };
        std::array<VkWriteDescriptorSet, FramesInFlight> writes {};
        for (std::size_t index = 0; index < FramesInFlight; ++index) {
            writes[index].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
            writes[index].dstSet = descriptorSets[index];
            writes[index].dstBinding = 0;
            writes[index].descriptorCount = 1;
            writes[index].descriptorType =
                VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
            writes[index].pImageInfo = &imageInfo;
        }
        vkUpdateDescriptorSets(device,
            static_cast<std::uint32_t>(writes.size()), writes.data(),
            0, nullptr);
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
        createSceneAttachment(extent, SceneNormalRoughnessFormat,
            VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT,
            VK_IMAGE_ASPECT_COLOR_BIT, sceneDirectNormalRoughnessImage,
            sceneDirectNormalRoughnessAllocation, sceneDirectNormalRoughnessView);
        createSceneAttachment(extent, SceneReflectanceFormat,
            VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT,
            VK_IMAGE_ASPECT_COLOR_BIT, sceneDirectReflectanceImage,
            sceneDirectReflectanceAllocation, sceneDirectReflectanceView);
        const VkExtent2D planarExtent {
            std::max(1u, extent.width / 2),
            std::max(1u, extent.height / 2)
        };
        createSceneAttachment(planarExtent, SceneHdrColorFormat,
            VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT,
            VK_IMAGE_ASPECT_COLOR_BIT, sceneDirectPlanarReflectionImage,
            sceneDirectPlanarReflectionAllocation,
            sceneDirectPlanarReflectionView);
        for (std::size_t index = 0; index < sceneDirectTaaHistoryImage.size(); ++index) {
            createSceneAttachment(extent, SceneHdrColorFormat,
                VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT,
                VK_IMAGE_ASPECT_COLOR_BIT, sceneDirectTaaHistoryImage[index],
                sceneDirectTaaHistoryAllocation[index], sceneDirectTaaHistoryView[index]);
        }
        for (std::size_t index = 0;
            index < sceneDirectAutoExposureImage.size(); ++index) {
            createSceneAttachment({ 1, 1 }, SceneAutoExposureFormat,
                VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT
                    | VK_IMAGE_USAGE_SAMPLED_BIT,
                VK_IMAGE_ASPECT_COLOR_BIT,
                sceneDirectAutoExposureImage[index],
                sceneDirectAutoExposureAllocation[index],
                sceneDirectAutoExposureView[index]);
        }
        const VkExtent2D gtaoExtent {
            std::max(1u, extent.width / 2),
            std::max(1u, extent.height / 2)
        };
        for (std::size_t index = 0;
            index < sceneDirectGtaoImage.size(); ++index) {
            createSceneAttachment(gtaoExtent,
                SceneAmbientOcclusionFormat,
                VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT
                    | VK_IMAGE_USAGE_SAMPLED_BIT,
                VK_IMAGE_ASPECT_COLOR_BIT,
                sceneDirectGtaoImage[index],
                sceneDirectGtaoAllocation[index],
                sceneDirectGtaoView[index]);
        }
        const VkExtent2D bloomGlareExtent {
            std::max(1u, extent.width / 8),
            std::max(1u, extent.height / 8)
        };
        for (std::size_t index = 0;
            index < sceneDirectBloomGlareImage.size(); ++index) {
            createSceneAttachment(bloomGlareExtent,
                SceneBloomGlareFormat,
                VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT
                    | VK_IMAGE_USAGE_SAMPLED_BIT,
                VK_IMAGE_ASPECT_COLOR_BIT,
                sceneDirectBloomGlareImage[index],
                sceneDirectBloomGlareAllocation[index],
                sceneDirectBloomGlareView[index]);
        }
        // SSR roda em MEIA resolucao linear (1/4 dos pixels, ver
        // SceneSsrHistoryFormat) - minimo 1x1 pra extents degenerados.
        const VkExtent2D ssrExtent {
            std::max(1u, extent.width / 2), std::max(1u, extent.height / 2)
        };
        for (std::size_t index = 0; index < sceneDirectSsrHistoryImage.size(); ++index) {
            createSceneAttachment(ssrExtent, SceneSsrHistoryFormat,
                VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT,
                VK_IMAGE_ASPECT_COLOR_BIT, sceneDirectSsrHistoryImage[index],
                sceneDirectSsrHistoryAllocation[index], sceneDirectSsrHistoryView[index]);
        }
        sceneDirectExtent = extent;
        updateTemporalDescriptorSets(sceneDirectHdrColorView,
            sceneDirectDepthView, sceneDirectMotionVectorView,
            sceneDirectTaaHistoryView, sceneDirectAutoExposureView,
            sceneDirectGtaoView, sceneDirectBloomGlareView,
            sceneDirectTaaResolveDescriptorSets,
            sceneDirectTonemapDescriptorSets,
            sceneDirectTonemapWithoutTaaDescriptorSets);
        updateBloomGlareDescriptorSets(sceneDirectDepthView,
            sceneDirectHdrColorView,
            sceneDirectBloomGlareDescriptorSets);
        updateGtaoDescriptorSets(sceneDirectDepthView,
            sceneDirectNormalRoughnessView,
            sceneDirectGtaoDescriptorSets);
        updateSsrDescriptorSets(sceneDirectHdrColorView, sceneDirectDepthView,
            sceneDirectNormalRoughnessView, sceneDirectReflectanceView,
            sceneDirectMotionVectorView, sceneDirectSsrHistoryView,
            sceneDirectSsrDescriptorSets, sceneDirectSsrCompositeDescriptorSets);
        updatePlanarReflectionDescriptorSets(
            sceneDirectPlanarReflectionView,
            sceneDirectPlanarReflectionDescriptorSets);
        updateOceanDescriptorSets(sceneDirectDepthView,
            sceneDirectOceanDescriptorSets);
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
        createImage(extent, SceneNormalRoughnessFormat,
            VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT,
            VK_IMAGE_ASPECT_COLOR_BIT, sceneNormalRoughnessImage,
            sceneNormalRoughnessAllocation, sceneNormalRoughnessView);
        createImage(extent, SceneReflectanceFormat,
            VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT,
            VK_IMAGE_ASPECT_COLOR_BIT, sceneReflectanceImage,
            sceneReflectanceAllocation, sceneReflectanceView);
        const VkExtent2D planarExtent {
            std::max(1u, extent.width / 2),
            std::max(1u, extent.height / 2)
        };
        createImage(planarExtent, SceneHdrColorFormat,
            VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT,
            VK_IMAGE_ASPECT_COLOR_BIT, scenePlanarReflectionImage,
            scenePlanarReflectionAllocation, scenePlanarReflectionView);
        for (std::size_t index = 0; index < sceneTaaHistoryImage.size(); ++index) {
            createImage(extent, SceneHdrColorFormat,
                VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT,
                VK_IMAGE_ASPECT_COLOR_BIT, sceneTaaHistoryImage[index],
                sceneTaaHistoryAllocation[index], sceneTaaHistoryView[index]);
        }
        for (std::size_t index = 0;
            index < sceneAutoExposureImage.size(); ++index) {
            createImage({ 1, 1 }, SceneAutoExposureFormat,
                VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT
                    | VK_IMAGE_USAGE_SAMPLED_BIT,
                VK_IMAGE_ASPECT_COLOR_BIT, sceneAutoExposureImage[index],
                sceneAutoExposureAllocation[index],
                sceneAutoExposureView[index]);
        }
        const VkExtent2D gtaoExtent {
            std::max(1u, extent.width / 2),
            std::max(1u, extent.height / 2)
        };
        for (std::size_t index = 0;
            index < sceneGtaoImage.size(); ++index) {
            createImage(gtaoExtent, SceneAmbientOcclusionFormat,
                VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT
                    | VK_IMAGE_USAGE_SAMPLED_BIT,
                VK_IMAGE_ASPECT_COLOR_BIT, sceneGtaoImage[index],
                sceneGtaoAllocation[index], sceneGtaoView[index]);
        }
        const VkExtent2D bloomGlareExtent {
            std::max(1u, extent.width / 8),
            std::max(1u, extent.height / 8)
        };
        for (std::size_t index = 0;
            index < sceneBloomGlareImage.size(); ++index) {
            createImage(bloomGlareExtent, SceneBloomGlareFormat,
                VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT
                    | VK_IMAGE_USAGE_SAMPLED_BIT,
                VK_IMAGE_ASPECT_COLOR_BIT,
                sceneBloomGlareImage[index],
                sceneBloomGlareAllocation[index],
                sceneBloomGlareView[index]);
        }
        // SSR roda em MEIA resolucao linear (ver comentario equivalente em
        // ensureSceneDirectDepth).
        const VkExtent2D ssrExtent {
            std::max(1u, extent.width / 2), std::max(1u, extent.height / 2)
        };
        for (std::size_t index = 0; index < sceneSsrHistoryImage.size(); ++index) {
            createImage(ssrExtent, SceneSsrHistoryFormat,
                VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT,
                VK_IMAGE_ASPECT_COLOR_BIT, sceneSsrHistoryImage[index],
                sceneSsrHistoryAllocation[index], sceneSsrHistoryView[index]);
        }
        sceneTargetExtent = extent;
        sceneColorImGuiDescriptor = ImGui_ImplVulkan_AddTexture(
            nearestSampler, sceneColorView, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
        if (sceneColorImGuiDescriptor == VK_NULL_HANDLE) {
            throw std::runtime_error("ImGui failed to allocate the Scene3D viewport descriptor");
        }

        updateTemporalDescriptorSets(sceneHdrColorView, sceneDepthView,
            sceneMotionVectorView, sceneTaaHistoryView,
            sceneAutoExposureView,
            sceneGtaoView, sceneBloomGlareView,
            sceneTaaResolveDescriptorSets, sceneTonemapDescriptorSets,
            sceneTonemapWithoutTaaDescriptorSets);
        updateBloomGlareDescriptorSets(sceneDepthView,
            sceneHdrColorView, sceneBloomGlareDescriptorSets);
        updateGtaoDescriptorSets(sceneDepthView,
            sceneNormalRoughnessView, sceneGtaoDescriptorSets);
        updateSsrDescriptorSets(sceneHdrColorView, sceneDepthView,
            sceneNormalRoughnessView, sceneReflectanceView,
            sceneMotionVectorView, sceneSsrHistoryView,
            sceneSsrDescriptorSets, sceneSsrCompositeDescriptorSets);
        updatePlanarReflectionDescriptorSets(scenePlanarReflectionView,
            scenePlanarReflectionDescriptorSets);
        updateOceanDescriptorSets(sceneDepthView, sceneOceanDescriptorSets);
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
    // Reacquiring the same image guarantees its previous present wait has
    // finished; a frame fence alone does not guarantee that.
    std::vector<VkSemaphore> swapchainPresentSemaphores;
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
    std::array<VkDescriptorSet, 4 * FramesInFlight> spriteDescriptorSets {};
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
    // A camera planar usa outro UBO/set no mesmo quadro: sobrescrever o UBO
    // principal faria todos os comandos já gravados enxergarem o último
    // valor no momento da submissão.
    std::array<VkDescriptorSet, FramesInFlight>
        scenePlanarCameraDescriptorSets {};
    std::array<VkBuffer, FramesInFlight> scenePlanarCameraUniformBuffers {};
    std::array<VmaAllocation, FramesInFlight>
        scenePlanarCameraUniformAllocations {};
    std::array<void*, FramesInFlight> scenePlanarCameraUniformMapped {};
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
    std::array<VkBuffer, FramesInFlight> sceneSkinBuffers {};
    std::array<VmaAllocation, FramesInFlight> sceneSkinAllocations {};
    std::array<void*, FramesInFlight> sceneSkinMapped {};
    std::array<std::size_t, FramesInFlight> sceneSkinCapacities {};
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
    VkDescriptorPool scenePlanarReflectionDescriptorPool = VK_NULL_HANDLE;
    std::array<VkDescriptorSet, FramesInFlight>
        scenePlanarReflectionDescriptorSets {};
    std::array<VkDescriptorSet, FramesInFlight>
        sceneDirectPlanarReflectionDescriptorSets {};
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
    // Um mapa por cascata, com resolução progressiva definida em
    // ShadowCascadeMapSizes e compartilhado pelos caminhos offscreen e
    // direto-pro-swapchain.
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

    // Cor HDR da camera espelhada, em meia resolução. Profundidade e os
    // outros MRTs são temporariamente reutilizados do caminho principal e
    // limpos de novo antes da cena normal.
    VkImage scenePlanarReflectionImage = VK_NULL_HANDLE;
    VmaAllocation scenePlanarReflectionAllocation = VK_NULL_HANDLE;
    VkImageView scenePlanarReflectionView = VK_NULL_HANDLE;
    SceneAttachmentState3D scenePlanarReflectionState;
    VkImage sceneDirectPlanarReflectionImage = VK_NULL_HANDLE;
    VmaAllocation sceneDirectPlanarReflectionAllocation = VK_NULL_HANDLE;
    VkImageView sceneDirectPlanarReflectionView = VK_NULL_HANDLE;
    SceneAttachmentState3D sceneDirectPlanarReflectionState;

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

    // Normal+rugosidade+metalico por pixel (ver SceneNormalRoughnessFormat)
    // - terceiro attachment de cor do MESMO passe opaco MRT. Mesmo
    // raciocinio do vetor de movimento acima: 1 imagem por caminho, nao
    // FramesInFlight, escrita e (futuramente) lida dentro do MESMO quadro.
    VkImage sceneNormalRoughnessImage = VK_NULL_HANDLE;
    VmaAllocation sceneNormalRoughnessAllocation = VK_NULL_HANDLE;
    VkImageView sceneNormalRoughnessView = VK_NULL_HANDLE;
    SceneAttachmentState3D sceneNormalRoughnessState;
    VkImage sceneDirectNormalRoughnessImage = VK_NULL_HANDLE;
    VmaAllocation sceneDirectNormalRoughnessAllocation = VK_NULL_HANDLE;
    VkImageView sceneDirectNormalRoughnessView = VK_NULL_HANDLE;
    SceneAttachmentState3D sceneDirectNormalRoughnessState;

    // Peso de reflectancia ambiente por pixel (ver SceneReflectanceFormat) -
    // quarto attachment de cor do MESMO passe opaco MRT. Mesmo raciocinio
    // dos dois attachments acima: 1 imagem por caminho, escrita pelo passe
    // opaco e lida pelo passe de composicao de SSR dentro do MESMO quadro
    // (ver ssr_composite.frag).
    VkImage sceneReflectanceImage = VK_NULL_HANDLE;
    VmaAllocation sceneReflectanceAllocation = VK_NULL_HANDLE;
    VkImageView sceneReflectanceView = VK_NULL_HANDLE;
    SceneAttachmentState3D sceneReflectanceState;
    VkImage sceneDirectReflectanceImage = VK_NULL_HANDLE;
    VmaAllocation sceneDirectReflectanceAllocation = VK_NULL_HANDLE;
    VkImageView sceneDirectReflectanceView = VK_NULL_HANDLE;
    SceneAttachmentState3D sceneDirectReflectanceState;

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
    std::array<VkDescriptorSet, FramesInFlight>
        sceneTonemapWithoutTaaDescriptorSets {};
    std::array<VkDescriptorSet, FramesInFlight>
        sceneDirectTonemapWithoutTaaDescriptorSets {};
    VkPipelineLayout sceneTonemapPipelineLayout = VK_NULL_HANDLE;
    VkPipeline sceneTonemapPipeline = VK_NULL_HANDLE;
    VkShaderModule sceneTonemapVertexModule = VK_NULL_HANDLE;
    VkShaderModule sceneTonemapFragmentModule = VK_NULL_HANDLE;
    VkPipelineLayout sceneAutoExposurePipelineLayout = VK_NULL_HANDLE;
    VkPipeline sceneAutoExposurePipeline = VK_NULL_HANDLE;
    VkShaderModule sceneAutoExposureFragmentModule = VK_NULL_HANDLE;

    // Dois texels 1x1 por caminho, alternados junto de currentFrame. O passe
    // atual lê o slot anterior, suaviza em função do tempo e grava no slot
    // atual; o tonemap lê esse resultado imediatamente no mesmo quadro.
    std::array<VkImage, FramesInFlight> sceneAutoExposureImage {};
    std::array<VmaAllocation, FramesInFlight>
        sceneAutoExposureAllocation {};
    std::array<VkImageView, FramesInFlight> sceneAutoExposureView {};
    std::array<SceneAttachmentState3D, FramesInFlight>
        sceneAutoExposureState {};
    std::array<VkImage, FramesInFlight> sceneDirectAutoExposureImage {};
    std::array<VmaAllocation, FramesInFlight>
        sceneDirectAutoExposureAllocation {};
    std::array<VkImageView, FramesInFlight>
        sceneDirectAutoExposureView {};
    std::array<SceneAttachmentState3D, FramesInFlight>
        sceneDirectAutoExposureState {};
    bool sceneAutoExposureHistoryValid = false;
    std::array<bool, FramesInFlight> sceneAutoExposureInitialized {};
    std::array<bool, FramesInFlight> sceneDirectAutoExposureInitialized {};
    bool sceneDirectAutoExposureHistoryValid = false;
    std::chrono::steady_clock::time_point sceneAutoExposureLastTime {};
    std::chrono::steady_clock::time_point
        sceneDirectAutoExposureLastTime {};
    bool sceneAutoExposureClockValid = false;
    bool sceneDirectAutoExposureClockValid = false;

    // GTAO em meia resolução. Cada frame em voo tem o próprio alvo para a
    // GPU nunca ler (no TAA) o mesmo VkImage que outro command buffer já
    // começou a sobrescrever.
    std::array<VkImage, FramesInFlight> sceneGtaoImage {};
    std::array<VmaAllocation, FramesInFlight> sceneGtaoAllocation {};
    std::array<VkImageView, FramesInFlight> sceneGtaoView {};
    std::array<SceneAttachmentState3D, FramesInFlight> sceneGtaoState {};
    std::array<VkImage, FramesInFlight> sceneDirectGtaoImage {};
    std::array<VmaAllocation, FramesInFlight>
        sceneDirectGtaoAllocation {};
    std::array<VkImageView, FramesInFlight> sceneDirectGtaoView {};
    std::array<SceneAttachmentState3D, FramesInFlight>
        sceneDirectGtaoState {};
    VkDescriptorSetLayout sceneGtaoDescriptorSetLayout = VK_NULL_HANDLE;
    VkDescriptorPool sceneGtaoDescriptorPool = VK_NULL_HANDLE;
    std::array<VkDescriptorSet, FramesInFlight> sceneGtaoDescriptorSets {};
    std::array<VkDescriptorSet, FramesInFlight>
        sceneDirectGtaoDescriptorSets {};
    VkPipelineLayout sceneGtaoPipelineLayout = VK_NULL_HANDLE;
    VkPipeline sceneGtaoPipeline = VK_NULL_HANDLE;
    VkShaderModule sceneGtaoFragmentModule = VK_NULL_HANDLE;

    // Glare/flare HDR em 1/8 de cada dimensão. Há um alvo e um descriptor por
    // frame em voo e por caminho para nunca sobrescrever uma imagem que o
    // tonemap de uma submissão anterior ainda possa estar lendo.
    std::array<VkImage, FramesInFlight> sceneBloomGlareImage {};
    std::array<VmaAllocation, FramesInFlight>
        sceneBloomGlareAllocation {};
    std::array<VkImageView, FramesInFlight> sceneBloomGlareView {};
    std::array<SceneAttachmentState3D, FramesInFlight>
        sceneBloomGlareState {};
    std::array<VkImage, FramesInFlight> sceneDirectBloomGlareImage {};
    std::array<VmaAllocation, FramesInFlight>
        sceneDirectBloomGlareAllocation {};
    std::array<VkImageView, FramesInFlight>
        sceneDirectBloomGlareView {};
    std::array<SceneAttachmentState3D, FramesInFlight>
        sceneDirectBloomGlareState {};
    VkDescriptorSetLayout sceneBloomGlareDescriptorSetLayout =
        VK_NULL_HANDLE;
    VkDescriptorPool sceneBloomGlareDescriptorPool = VK_NULL_HANDLE;
    std::array<VkDescriptorSet, FramesInFlight>
        sceneBloomGlareDescriptorSets {};
    std::array<VkDescriptorSet, FramesInFlight>
        sceneDirectBloomGlareDescriptorSets {};
    VkPipelineLayout sceneBloomGlarePipelineLayout = VK_NULL_HANDLE;
    VkPipeline sceneBloomGlarePipeline = VK_NULL_HANDLE;
    VkShaderModule sceneBloomGlareFragmentModule = VK_NULL_HANDLE;

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

    // Historico de SSR (ver ssr_trace_resolve.frag) - MEIA resolucao (ver
    // ensureScene3DTarget), 2 slots por caminho indexados por currentFrame,
    // exatamente como sceneTaaHistoryImage acima (mesmo raciocinio: nenhum
    // ping-pong dedicado, currentFrame ja alterna em lockstep com cada
    // quadro real). history[currentFrame] e escrito agora pelo traçado/
    // resolve; history[1-currentFrame] tem o resultado do quadro passado,
    // usado como entrada de historico E como fonte do passe de composicao
    // (ver sceneSsrCompositeDescriptorSets, que lê o slot currentFrame
    // apos este passe escreve-lo).
    std::array<VkImage, FramesInFlight> sceneSsrHistoryImage {};
    std::array<VmaAllocation, FramesInFlight> sceneSsrHistoryAllocation {};
    std::array<VkImageView, FramesInFlight> sceneSsrHistoryView {};
    std::array<SceneAttachmentState3D, FramesInFlight> sceneSsrHistoryState {};
    std::array<VkImage, FramesInFlight> sceneDirectSsrHistoryImage {};
    std::array<VmaAllocation, FramesInFlight> sceneDirectSsrHistoryAllocation {};
    std::array<VkImageView, FramesInFlight> sceneDirectSsrHistoryView {};
    std::array<SceneAttachmentState3D, FramesInFlight> sceneDirectSsrHistoryState {};

    // Pipeline de traçado+resolve de SSR: mesmo triangulo cheio de tela
    // (reaproveita sceneTonemapVertexModule), 2 sets como o resolve de TAA:
    // set 0 = sceneDescriptorSetLayout (UBO da cena + luzes), set 1 = cor
    // HDR atual, profundidade, normal/rugosidade, reflectancia, vetor de
    // movimento e historico (6 bindings, ver ssr_trace_resolve.frag). Roda
    // em MEIA resolucao - viewport/scissor proprios no render loop, nao os
    // mesmos de sceneTargetExtent/sceneDirectExtent.
    VkDescriptorSetLayout sceneSsrDescriptorSetLayout = VK_NULL_HANDLE;
    VkDescriptorPool sceneSsrDescriptorPool = VK_NULL_HANDLE;
    std::array<VkDescriptorSet, FramesInFlight> sceneSsrDescriptorSets {};
    std::array<VkDescriptorSet, FramesInFlight> sceneDirectSsrDescriptorSets {};
    VkPipelineLayout sceneSsrPipelineLayout = VK_NULL_HANDLE;
    VkPipeline sceneSsrPipeline = VK_NULL_HANDLE;
    VkShaderModule sceneSsrFragmentModule = VK_NULL_HANDLE;

    // Pipeline de composicao de SSR: soma (blend ADITIVO, ver
    // createScene3DResources) o historico de SSR recem-resolvido de volta
    // no HDR em resolucao CHEIA, pesado pela reflectancia por pixel. Um set
    // dedicado so seu (2 bindings: reflectancia, historico de SSR do slot
    // currentFrame) - nao precisa do UBO da cena, mesmo padrao de
    // sceneTonemapDescriptorSetLayout (um unico set, sem set 0
    // compartilhado).
    VkDescriptorSetLayout sceneSsrCompositeDescriptorSetLayout = VK_NULL_HANDLE;
    VkDescriptorPool sceneSsrCompositeDescriptorPool = VK_NULL_HANDLE;
    std::array<VkDescriptorSet, FramesInFlight> sceneSsrCompositeDescriptorSets {};
    std::array<VkDescriptorSet, FramesInFlight> sceneDirectSsrCompositeDescriptorSets {};
    VkPipelineLayout sceneSsrCompositePipelineLayout = VK_NULL_HANDLE;
    VkPipeline sceneSsrCompositePipeline = VK_NULL_HANDLE;
    VkShaderModule sceneSsrCompositeFragmentModule = VK_NULL_HANDLE;

    VkDescriptorSetLayout sceneOceanDescriptorSetLayout = VK_NULL_HANDLE;
    VkDescriptorPool sceneOceanDescriptorPool = VK_NULL_HANDLE;
    std::array<VkDescriptorSet, FramesInFlight> sceneOceanDescriptorSets {};
    std::array<VkDescriptorSet, FramesInFlight>
        sceneDirectOceanDescriptorSets {};
    VkPipelineLayout sceneOceanPipelineLayout = VK_NULL_HANDLE;
    VkPipeline sceneOceanPipeline = VK_NULL_HANDLE;
    VkShaderModule sceneOceanVertexModule = VK_NULL_HANDLE;
    VkShaderModule sceneOceanFragmentModule = VK_NULL_HANDLE;

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
