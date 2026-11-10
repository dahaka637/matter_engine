#include "Engine/Core/Log.hpp"
#include "Engine/Geometry/AuthoredMap3D.hpp"
#include "Engine/Geometry/GltfAcousticZone3D.hpp"
#include "Engine/Geometry/GltfPhysicsMetadata3D.hpp"
#include "Workbench/WorkbenchApp.hpp"

#include <cmath>
#include <span>
#include <stdexcept>

namespace MatterEngine::Workbench {

void WorkbenchApp::selectLaboratoryMap(bool indoor) {
    if (m_laboratoryMapLoaded || m_laboratoryIndoor != indoor || m_laboratoryMapLoadFailed) {
        releaseLaboratoryAssets(*m_renderer);
        m_laboratoryIndoor = indoor;
        m_laboratoryMapLoadFailed = false;
        m_laboratoryInitialized = false;
        m_controlledCharacterEntityId = 0;
        m_propPreviewAtlas = {};
        m_laboratoryMapLights.clear();
        m_laboratoryClockRunning = false;
        m_laboratoryToneMapping = ToneMappingSettings3D{};
        if (indoor) {
            applyLaboratoryGraphicsPreset(1);
            m_laboratoryToneMapping.exposure = .08f;
            m_laboratoryToneMapping.contrast = 1.10f;
            m_laboratoryToneMapping.saturation = 1.05f;
        }
    }
    enterLaboratory();
}

void WorkbenchApp::loadAuthoredLaboratoryMap(Renderer &renderer) {
    try {
        const auto map = loadGltfModel(std::string(MATTERENGINE_ASSETS_DIR) + "/maps/compiled/galpao_01.glb");
        const auto markers = parseAuthoredMap3D(map);
        if (markers.fields.size() != 1)
            throw std::runtime_error("Galpao 01 requer um matter_field");
        std::vector<bool> colorImages(map.images.size(), false);
        for (const auto &material : map.materials) {
            if (material.imageIndex >= 0)
                colorImages.at(material.imageIndex) = true;
            if (material.emissiveImageIndex >= 0)
                colorImages.at(material.emissiveImageIndex) = true;
        }
        for (std::size_t imageIndex = 0; imageIndex < map.images.size(); ++imageIndex) {
            const auto &image = map.images[imageIndex];
            if (image.rgbaPixels.empty())
                throw std::runtime_error("Textura do mapa ausente");
            m_laboratoryMapTextures.push_back(renderer.createTexture2D(
                {static_cast<std::uint32_t>(image.width), static_cast<std::uint32_t>(image.height)}, image.rgbaPixels,
                colorImages[imageIndex]));
        }
        const auto texture = [&](int index) {
            return index < 0 ? RHI::TextureHandle{} : m_laboratoryMapTextures.at(static_cast<std::size_t>(index));
        };
        const auto upload = [&](const MeshData3D &mesh) -> LaboratoryMapPart & {
            auto &part = m_laboratoryMapParts.emplace_back();
            part.vertexBuffer = renderer.createBuffer(
                {mesh.vertices.size() * sizeof(MeshVertex3D), RHI::BufferUsage::Vertex, true, "Mapa: vertices"});
            part.indexBuffer = renderer.createBuffer(
                {mesh.indices.size() * sizeof(std::uint32_t), RHI::BufferUsage::Index, true, "Mapa: indices"});
            renderer.writeBuffer(part.vertexBuffer, 0, std::as_bytes(std::span(mesh.vertices)));
            renderer.writeBuffer(part.indexBuffer, 0, std::as_bytes(std::span(mesh.indices)));
            part.indexCount = static_cast<std::uint32_t>(mesh.indices.size());
            part.boundsCenter = (mesh.boundsMin + mesh.boundsMax) * .5f;
            part.boundsRadius = (mesh.boundsMax - mesh.boundsMin).length() * .5f;
            return part;
        };
        for (const auto &source : map.parts) {
            const auto metadata = parseGltfPhysicsMetadata(source.extras);
            if (!m_materialLibrary.find(metadata.materialId))
                throw std::runtime_error("Material fisico desconhecido: " + metadata.materialId);
            if (metadata.generateCollision) {
                if (metadata.collisionMode != ImportedCollisionMode3D::StaticTriangleMesh)
                    throw std::runtime_error("O cenario autorado requer collision_mode=static_mesh");
                PhysicsShape3D shape;
                shape.type = PhysicsShapeType3D::TriangleMesh;
                shape.mesh = m_physicsEngine.cookStaticTriangleMesh(source.mesh, metadata.materialId);
                shape.materialId = metadata.materialId;
                PhysicsBodyDefinition3D body;
                body.motionType = PhysicsMotionType3D::Static;
                body.materialId = metadata.materialId;
                body.entityId = m_nextEntityId++;
                m_laboratoryMapBodies.push_back(m_physicsScene->createBody(body, std::span(&shape, 1)));
            }
            auto &part = upload(source.mesh);
            part.name = source.nodeName;
            part.materialId = metadata.materialId;
            part.matteSurface = metadata.materialId == "grass" || metadata.materialId == "rubber";
            const auto &material = map.materials.at(static_cast<std::size_t>(source.materialIndex));
            part.albedoTexture = texture(material.imageIndex);
            part.metallicRoughnessTexture = texture(material.metallicRoughnessImageIndex);
            part.normalTexture = texture(material.normalImageIndex);
            part.normalScale = material.normalScale;
            part.emissiveTexture = texture(material.emissiveImageIndex);
            part.emissiveFactor = material.emissiveFactor;
            part.metallic = material.metallicFactor;
            part.roughness = material.roughnessFactor;
            if (const auto *value = material.extras.find("parallax_depth")) {
                std::size_t consumed = 0;
                part.parallaxDepth = std::stof(*value, &consumed);
                if (consumed != value->size() || !std::isfinite(part.parallaxDepth) || part.parallaxDepth < 0 ||
                    part.parallaxDepth > .02f)
                    throw std::runtime_error("parallax_depth invalido");
            }
        }
        for (const auto &field : markers.fields) {
            auto &paint = upload(buildFutsalMarkings3D(field));
            paint.name = "Demarcacoes procedurais";
            paint.matteSurface = true;
            const auto orientation = Quaternion::fromAxisAngle({0, 0, 1}, field.yaw);
            for (float side : {-1.0f, 1.0f}) {
                const Vec3 center = field.center + orientation.rotate({side * field.length * .5f, 0, 0});
                if (!createLaboratoryGoal(renderer, center, field.yaw + (side > 0 ? 3.14159265359f : 0.0f), true))
                    throw std::runtime_error("Falha ao criar trave do campo");
            }
        }
        m_laboratoryMapLights = map.lights;
        m_worldAudio.setAcousticZones(parseAcousticZones(map.entities));
        m_laboratorySpawnPosition = markers.spawns.front().position;
        m_laboratorySpawnYaw = markers.spawns.front().yaw;
        m_laboratoryMapLoaded = true;
        resetLaboratoryCharacter();
        Log::info("Galpao 01: " + std::to_string(map.parts.size()) + " partes, " + std::to_string(map.lights.size()) +
                  " luzes, " + std::to_string(markers.spawns.size()) +
                  " nascimentos, 1 campo e 2 traves com redes fisicas.");
    } catch (const std::exception &error) {
        releaseLaboratoryAssets(renderer);
        m_laboratoryMapLoadFailed = true;
        m_laboratoryStatus = std::string("Falha ao carregar Galpão 01: ") + error.what();
        Log::error(m_laboratoryStatus);
    }
}
} // namespace MatterEngine::Workbench
