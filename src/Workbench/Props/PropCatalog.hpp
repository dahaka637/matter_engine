#pragma once

#include "Engine/Materials/MaterialLibrary.hpp"
#include "Engine/Physics/PhysicsEngine3D.hpp"
#include "Engine/Render/Renderer.hpp"

#include <cstddef>
#include <cstdint>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace MatterEngine::Workbench {

struct GpuModelPart3D {
    RHI::BufferHandle vertexBuffer;
    RHI::BufferHandle indexBuffer;
    RHI::TextureHandle albedoTexture;
    RHI::TextureHandle metallicRoughnessTexture;
    std::uint32_t indexCount = 0;
    float metallic = 0.0f;
    float roughness = 1.0f;
};

// Modelo compartilhado por todas as instâncias de um prop. Buffers e
// texturas são carregados uma única vez; cada entidade fornece apenas pose.
struct GpuModel3D {
    std::vector<GpuModelPart3D> parts;
    std::vector<RHI::TextureHandle> textures;
    Vec3 dimensionsMeters;
};

struct PropDefinition3D {
    std::string id;
    std::string displayName;
    std::string materialId;
    Vec3 dimensionsMeters;
    float massKg = 0.0f;
    PhysicsBodyDefinition3D bodyTemplate;
    std::vector<PhysicsShape3D> collisionShapes;
    GpuModel3D visual;
};

// Catálogo da aplicação de desenvolvimento. Ele conhece quais assets devem
// aparecer nas ferramentas, enquanto importação glTF e regras físicas ficam
// na engine e continuam reutilizáveis por futuros jogos.
class PropCatalog final {
public:
    void load(Renderer& renderer, const MaterialLibrary& materials,
        PhysicsEngine3D& physics, std::string_view assetsDirectory);
    void release(Renderer& renderer);

    [[nodiscard]] bool loaded() const { return m_loaded; }
    [[nodiscard]] const std::vector<PropDefinition3D>& definitions() const {
        return m_definitions;
    }
    [[nodiscard]] const PropDefinition3D* find(std::string_view id) const;

private:
    std::vector<PropDefinition3D> m_definitions;
    bool m_loaded = false;
};

[[nodiscard]] GpuModel3D loadGpuModel3D(Renderer& renderer,
    const std::string& path, bool recenter = true);
void releaseGpuModel3D(Renderer& renderer, GpuModel3D& model);

// Acrescenta todas as partes de um modelo à lista de renderização sem copiar
// recursos. O span final só permanece válido durante o frame atual.
// previousPosition/previousOrientation alimentam o vetor de movimento por
// pixel do TAA (ver MeshRender3D) - para um objeto parado, basta repetir
// position/orientation aqui.
void appendGpuModelRenderables(const GpuModel3D& model, Vec3 position,
    Quaternion orientation, Vec3 previousPosition,
    Quaternion previousOrientation, float scale, bool selected,
    bool outlineGlow, bool castsShadow,
    std::vector<MeshRender3D>& destination);
void writeGpuModelRenderables(const GpuModel3D& model, Vec3 position,
    Quaternion orientation, Vec3 previousPosition,
    Quaternion previousOrientation, float scale, bool selected,
    bool outlineGlow, bool castsShadow,
    std::span<MeshRender3D> destination) noexcept;

} // namespace MatterEngine::Workbench
