#include "Workbench/Props/PropCatalog.hpp"

#include "Engine/Core/Log.hpp"
#include "Engine/Geometry/GltfLoader.hpp"
#include "Engine/Geometry/MeshData3D.hpp"
#include "Engine/Geometry/PhysicalAsset3D.hpp"
#include "Engine/RHI/RHITypes.hpp"

#include <meshoptimizer.h>

#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <limits>
#include <span>
#include <stdexcept>
#include <utility>

namespace MatterEngine::Workbench {
namespace {

struct CatalogEntry {
    std::string_view id;
    std::string_view displayName;
    std::string_view fileName;
};

constexpr std::array CatalogEntries {
    CatalogEntry { "wood_chair", "Cadeira de madeira", "wood_chair.glb" },
    CatalogEntry { "steel_anvil", "Bigorna de aço", "anvil.glb" },
    CatalogEntry { "plastic_barrel", "Barril de plástico", "plastic_barrel.glb" },
    CatalogEntry { "soccer_ball", "Bola de futebol", "soccer_ball.glb" },
    CatalogEntry { "wood_crate", "Caixa de madeira", "wood_crate.glb" },
    CatalogEntry { "car_tire", "Pneu de carro", "car_tire.glb" }
};

struct Bounds3D {
    Vec3 minimum {
        std::numeric_limits<float>::max(),
        std::numeric_limits<float>::max(),
        std::numeric_limits<float>::max()
    };
    Vec3 maximum {
        std::numeric_limits<float>::lowest(),
        std::numeric_limits<float>::lowest(),
        std::numeric_limits<float>::lowest()
    };
};

Bounds3D modelBounds(const LoadedGltfModel& model) {
    Bounds3D bounds;
    for (const LoadedGltfPart& part : model.parts) {
        bounds.minimum.x = std::min(bounds.minimum.x, part.mesh.boundsMin.x);
        bounds.minimum.y = std::min(bounds.minimum.y, part.mesh.boundsMin.y);
        bounds.minimum.z = std::min(bounds.minimum.z, part.mesh.boundsMin.z);
        bounds.maximum.x = std::max(bounds.maximum.x, part.mesh.boundsMax.x);
        bounds.maximum.y = std::max(bounds.maximum.y, part.mesh.boundsMax.y);
        bounds.maximum.z = std::max(bounds.maximum.z, part.mesh.boundsMax.z);
    }
    return bounds;
}

// Gera os LODs uma unica vez, durante o carregamento. O vertex buffer
// permanece compartilhado e cada nivel adiciona somente um index buffer.
// Isso evita duplicar atributos/texturas e, principalmente, impede que um
// asset visual exageradamente denso seja redesenhado integralmente no
// depth-prepass e nas quatro cascatas de sombra.
std::vector<std::vector<std::uint32_t>> buildMeshLods(MeshData3D& mesh) {
    constexpr std::size_t MinimumSourceIndexCount = 1'800;
    if (mesh.indices.size() < MinimumSourceIndexCount
        || mesh.vertices.size() < 200) {
        return {};
    }

    meshopt_optimizeVertexCache(mesh.indices.data(), mesh.indices.data(),
        mesh.indices.size(), mesh.vertices.size());

    constexpr std::array<float, 2> TargetRatios { 0.20f, 0.04f };
    constexpr std::array<float, 2> TargetErrors { 0.006f, 0.025f };
    // normal.xyz + uv.xy sao contiguos em MeshVertex3D. Preservar ambos
    // evita que a simplificacao transforme uma bola texturizada em um
    // poliedro visualmente instavel nas distancias de troca.
    constexpr std::array<float, 5> AttributeWeights {
        0.45f, 0.45f, 0.45f, 0.70f, 0.70f
    };

    std::vector<std::vector<std::uint32_t>> lods;
    lods.reserve(TargetRatios.size());
    const std::size_t originalIndexCount = mesh.indices.size();
    const std::vector<std::uint32_t>* source = &mesh.indices;
    for (std::size_t level = 0; level < TargetRatios.size(); ++level) {
        std::size_t targetIndexCount = static_cast<std::size_t>(
            static_cast<double>(originalIndexCount) * TargetRatios[level]);
        targetIndexCount = std::max<std::size_t>(96,
            targetIndexCount - targetIndexCount % 3);
        if (targetIndexCount >= source->size()) continue;

        std::vector<std::uint32_t> simplified(source->size());
        float resultError = 0.0f;
        const std::size_t simplifiedIndexCount =
            meshopt_simplifyWithAttributes(simplified.data(),
                source->data(), source->size(),
                &mesh.vertices.front().position.x, mesh.vertices.size(),
                sizeof(MeshVertex3D),
                &mesh.vertices.front().normal.x, sizeof(MeshVertex3D),
                AttributeWeights.data(), AttributeWeights.size(), nullptr,
                targetIndexCount, TargetErrors[level],
                meshopt_SimplifyPermissive, &resultError);
        if (simplifiedIndexCount >= source->size()
            || simplifiedIndexCount < 3) {
            continue;
        }
        simplified.resize(simplifiedIndexCount);
        meshopt_optimizeVertexCache(simplified.data(), simplified.data(),
            simplified.size(), mesh.vertices.size());
        lods.push_back(std::move(simplified));
        source = &lods.back();
    }
    return lods;
}

GpuModel3D uploadModel(Renderer& renderer, LoadedGltfModel& model,
    bool recenter) {
    if (model.parts.empty()) {
        throw std::runtime_error("Modelo sem partes renderizáveis");
    }
    const Bounds3D bounds = modelBounds(model);
    const Vec3 center = recenter
        ? (bounds.minimum + bounds.maximum) * 0.5f : Vec3 {};

    GpuModel3D result;
    result.dimensionsMeters = bounds.maximum - bounds.minimum;
    result.textures.resize(model.images.size());
    try {
        for (std::size_t index = 0; index < model.images.size(); ++index) {
            const LoadedGltfImage& image = model.images[index];
            if (image.width <= 0 || image.height <= 0
                || image.rgbaPixels.empty()) {
                continue;
            }
            result.textures[index] = renderer.createTexture2D(
                { static_cast<std::uint32_t>(image.width),
                    static_cast<std::uint32_t>(image.height) },
                std::as_bytes(std::span(image.rgbaPixels)));
        }

        result.parts.reserve(model.parts.size());
        for (LoadedGltfPart& part : model.parts) {
            if (part.mesh.vertices.empty() || part.mesh.indices.empty()) {
                continue;
            }
            if (recenter) {
                for (MeshVertex3D& vertex : part.mesh.vertices) {
                    vertex.position -= center;
                }
            }
            std::vector<std::vector<std::uint32_t>> lodIndices =
                buildMeshLods(part.mesh);

            GpuModelPart3D gpuPart;
            gpuPart.vertexBuffer = renderer.createBuffer({
                part.mesh.vertices.size() * sizeof(MeshVertex3D),
                RHI::BufferUsage::Vertex, true, "Prop vertices" });
            gpuPart.indexBuffer = renderer.createBuffer({
                part.mesh.indices.size() * sizeof(std::uint32_t),
                RHI::BufferUsage::Index, true, "Prop indices" });
            renderer.writeBuffer(gpuPart.vertexBuffer, 0,
                std::as_bytes(std::span(part.mesh.vertices)));
            renderer.writeBuffer(gpuPart.indexBuffer, 0,
                std::as_bytes(std::span(part.mesh.indices)));
            gpuPart.indexCount = static_cast<std::uint32_t>(
                part.mesh.indices.size());
            gpuPart.lods.reserve(lodIndices.size());
            for (const std::vector<std::uint32_t>& indices : lodIndices) {
                GpuMeshLod3D lod;
                lod.indexBuffer = renderer.createBuffer({
                    indices.size() * sizeof(std::uint32_t),
                    RHI::BufferUsage::Index, true, "Prop LOD indices" });
                renderer.writeBuffer(lod.indexBuffer, 0,
                    std::as_bytes(std::span(indices)));
                lod.indexCount = static_cast<std::uint32_t>(indices.size());
                gpuPart.lods.push_back(lod);
            }

            if (part.materialIndex >= 0
                && static_cast<std::size_t>(part.materialIndex)
                    < model.materials.size()) {
                const LoadedGltfMaterial& material = model.materials[
                    static_cast<std::size_t>(part.materialIndex)];
                gpuPart.metallic = material.metallicFactor;
                gpuPart.roughness = material.roughnessFactor;
                if (material.imageIndex >= 0
                    && static_cast<std::size_t>(material.imageIndex)
                        < result.textures.size()) {
                    gpuPart.albedoTexture = result.textures[
                        static_cast<std::size_t>(material.imageIndex)];
                }
                if (material.metallicRoughnessImageIndex >= 0
                    && static_cast<std::size_t>(
                        material.metallicRoughnessImageIndex)
                        < result.textures.size()) {
                    gpuPart.metallicRoughnessTexture = result.textures[
                        static_cast<std::size_t>(
                            material.metallicRoughnessImageIndex)];
                }
            }
            result.parts.push_back(std::move(gpuPart));
        }
    } catch (...) {
        releaseGpuModel3D(renderer, result);
        throw;
    }
    return result;
}

} // namespace

void PropCatalog::load(Renderer& renderer, const MaterialLibrary& materials,
    PhysicsEngine3D& physics, std::string_view assetsDirectory) {
    if (m_loaded) return;
    std::vector<PropDefinition3D> loaded;
    loaded.reserve(CatalogEntries.size());
    try {
        for (const CatalogEntry& entry : CatalogEntries) {
            const std::string path = std::string(assetsDirectory)
                + "/models/props/" + std::string(entry.fileName);
            PhysicalAsset3D physical = loadPhysicalAsset3D(path, materials,
                physics);

            PropDefinition3D definition;
            definition.id = entry.id;
            definition.displayName = entry.displayName;
            definition.materialId = physical.metadata.materialId;
            definition.dimensionsMeters = physical.dimensionsMeters;
            definition.massKg = physical.bodyTemplate.massKg;
            definition.bodyTemplate = std::move(physical.bodyTemplate);
            definition.collisionShapes =
                std::move(physical.collision.shapes);
            // O importador físico já recentrou a malha no centro do collider.
            definition.visual = uploadModel(renderer, physical.model, false);
            std::uint64_t visualTriangles = 0;
            std::uint64_t shadowTriangles = 0;
            for (const GpuModelPart3D& part : definition.visual.parts) {
                visualTriangles += part.indexCount / 3u;
                shadowTriangles += (part.lods.empty()
                    ? part.indexCount : part.lods.back().indexCount) / 3u;
            }
            Log::info("Prop " + definition.id + ": material="
                + definition.materialId + ", massa="
                + std::to_string(definition.massKg) + " kg, dimensões="
                + std::to_string(definition.dimensionsMeters.x) + " x "
                + std::to_string(definition.dimensionsMeters.y) + " x "
                + std::to_string(definition.dimensionsMeters.z)
                + " m, colisores="
                + std::to_string(definition.collisionShapes.size())
                + ", visual=" + std::to_string(visualTriangles)
                + " tri, proxy de sombra="
                + std::to_string(shadowTriangles) + " tri.");
            loaded.push_back(std::move(definition));
        }
    } catch (...) {
        for (PropDefinition3D& definition : loaded) {
            releaseGpuModel3D(renderer, definition.visual);
        }
        throw;
    }
    m_definitions = std::move(loaded);
    m_loaded = true;
    Log::info("Catálogo de props carregado com "
        + std::to_string(m_definitions.size()) + " objetos.");
}

void PropCatalog::release(Renderer& renderer) {
    for (PropDefinition3D& definition : m_definitions) {
        releaseGpuModel3D(renderer, definition.visual);
    }
    m_definitions.clear();
    m_loaded = false;
}

const PropDefinition3D* PropCatalog::find(std::string_view id) const {
    const auto found = std::find_if(m_definitions.begin(),
        m_definitions.end(), [id](const PropDefinition3D& definition) {
            return definition.id == id;
        });
    return found != m_definitions.end() ? &*found : nullptr;
}

GpuModel3D loadGpuModel3D(Renderer& renderer, const std::string& path,
    bool recenter) {
    LoadedGltfModel model = loadGltfModel(path);
    return uploadModel(renderer, model, recenter);
}

void releaseGpuModel3D(Renderer& renderer, GpuModel3D& model) {
    for (GpuModelPart3D& part : model.parts) {
        if (part.vertexBuffer) renderer.destroyBuffer(part.vertexBuffer);
        if (part.indexBuffer) renderer.destroyBuffer(part.indexBuffer);
        for (GpuMeshLod3D& lod : part.lods) {
            if (lod.indexBuffer) renderer.destroyBuffer(lod.indexBuffer);
            lod.indexBuffer = {};
            lod.indexCount = 0;
        }
        part.lods.clear();
        part.vertexBuffer = {};
        part.indexBuffer = {};
    }
    for (RHI::TextureHandle& texture : model.textures) {
        if (texture) renderer.destroyTexture(texture);
        texture = {};
    }
    model.parts.clear();
    model.textures.clear();
    model.dimensionsMeters = {};
}

void appendGpuModelRenderables(const GpuModel3D& model, Vec3 position,
    Quaternion orientation, Vec3 previousPosition,
    Quaternion previousOrientation, float scale, bool selected,
    bool outlineGlow, bool castsShadow,
    std::vector<MeshRender3D>& destination, float cameraDistanceMeters) {
    const std::size_t first = destination.size();
    destination.resize(first + model.parts.size());
    writeGpuModelRenderables(model, position, orientation, previousPosition,
        previousOrientation, scale, selected, outlineGlow, castsShadow,
        std::span<MeshRender3D>(destination).subspan(first),
        cameraDistanceMeters);
}

void writeGpuModelRenderables(const GpuModel3D& model, Vec3 position,
    Quaternion orientation, Vec3 previousPosition,
    Quaternion previousOrientation, float scale, bool selected,
    bool outlineGlow, bool castsShadow,
    std::span<MeshRender3D> destination, float cameraDistanceMeters) noexcept {
    assert(destination.size() >= model.parts.size());
    const float modelRadius = std::max(0.001f,
        model.dimensionsMeters.length() * 0.5f * std::abs(scale));
    const float relativeDistance = cameraDistanceMeters > 0.0f
        ? cameraDistanceMeters / modelRadius : 0.0f;
    for (std::size_t index = 0; index < model.parts.size(); ++index) {
        const GpuModelPart3D& part = model.parts[index];
        MeshRender3D renderable;
        renderable.vertexBuffer = part.vertexBuffer;
        renderable.indexBuffer = part.indexBuffer;
        renderable.indexCount = part.indexCount;
        if (!part.lods.empty()) {
            // relativeDistance equivale aproximadamente ao inverso do
            // tamanho projetado. A bola, por exemplo, tem 81.920 triângulos:
            // manter o original quando já ocupa poucas centenas de pixels
            // aumenta custo sem acrescentar silhueta visível.
            if (relativeDistance > 35.0f) {
                renderable.indexBuffer = part.lods.back().indexBuffer;
                renderable.indexCount = part.lods.back().indexCount;
            } else if (relativeDistance > 12.0f) {
                renderable.indexBuffer = part.lods.front().indexBuffer;
                renderable.indexCount = part.lods.front().indexCount;
            }
            // Sombras precisam apenas preservar a silhueta. O nivel mais
            // compacto e usado em todas as cascatas, inclusive quando o
            // objeto esta perto da camera.
            renderable.shadowIndexBuffer = part.lods.back().indexBuffer;
            renderable.shadowIndexCount = part.lods.back().indexCount;
        }
        renderable.position = position;
        renderable.orientation = orientation;
        renderable.previousPosition = previousPosition;
        renderable.previousOrientation = previousOrientation;
        renderable.scale = scale;
        renderable.albedoTexture = part.albedoTexture;
        renderable.metallicRoughnessTexture =
            part.metallicRoughnessTexture;
        renderable.metallic = part.metallic;
        renderable.roughness = part.roughness;
        renderable.selected = selected;
        renderable.outlineGlow = outlineGlow;
        renderable.castsShadow = castsShadow;
        destination[index] = renderable;
    }
}

} // namespace MatterEngine::Workbench
