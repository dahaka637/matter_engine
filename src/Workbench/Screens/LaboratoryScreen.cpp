#include "Workbench/WorkbenchApp.hpp"
#include "Workbench/UiKit.hpp"

#include "Engine/Core/Log.hpp"
#include "Engine/Geometry/GltfAcousticZone3D.hpp"
#include "Engine/Geometry/GltfLoader.hpp"
#include "Engine/Geometry/GltfPhysicsMetadata3D.hpp"
#include "Engine/Math/Mat4.hpp"
#include "Engine/Math/Frustum3D.hpp"
#include "Engine/Math/JitterSequence.hpp"
#include "Engine/Math/ShadowCascade.hpp"
#include "Engine/RHI/RHITypes.hpp"
#include "Engine/Render/Scene3D.hpp"
#include "imgui.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <span>
#include <string>
#include <vector>

namespace MatterEngine::Workbench {
namespace {

constexpr float Pi = 3.14159265358979323846f;
constexpr float LaboratoryFarPlane = 2000.0f;

Vec3 cameraForward(float yaw, float pitch) {
    const float planar = std::cos(pitch);
    return {
        planar * std::cos(yaw),
        planar * std::sin(yaw),
        std::sin(pitch)
    };
}

bool projectToScreen(const Mat4& viewProjection, Vec3 point,
    ImVec2 displaySize, ImVec2& screenPoint) {
    const float clipX = viewProjection.at(0, 0) * point.x
        + viewProjection.at(0, 1) * point.y
        + viewProjection.at(0, 2) * point.z
        + viewProjection.at(0, 3);
    const float clipY = viewProjection.at(1, 0) * point.x
        + viewProjection.at(1, 1) * point.y
        + viewProjection.at(1, 2) * point.z
        + viewProjection.at(1, 3);
    const float clipW = viewProjection.at(3, 0) * point.x
        + viewProjection.at(3, 1) * point.y
        + viewProjection.at(3, 2) * point.z
        + viewProjection.at(3, 3);
    if (clipW <= 0.001f) return false;
    const float inverseW = 1.0f / clipW;
    screenPoint = {
        (clipX * inverseW * 0.5f + 0.5f) * displaySize.x,
        (0.5f - clipY * inverseW * 0.5f) * displaySize.y
    };
    return true;
}

// Abre um dos paineis flutuantes de depuracao (Physgun/Gráficos/Performance)
// com posicao/tamanho padrao e o mesmo teto de altura pra nenhum deles
// nascer maior que a tela - fatorado porque os 3 paineis precisam
// exatamente do mesmo comportamento, so o conteudo interno muda.
// `open` tambem vira o X de fechar da propria janela ImGui, entao fechar
// pelo X ou clicando o botao da barra de novo do exatamente no mesmo lugar.
bool beginDebugPanel(const char* name, bool* open, ImVec2 defaultPosition,
    ImVec2 defaultSize, ImVec2 displaySize, float uiScale) {
    ImGui::SetNextWindowPos(defaultPosition, ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(defaultSize, ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSizeConstraints(
        { 300.0f * uiScale, 160.0f * uiScale },
        { std::min(760.0f * uiScale, displaySize.x - 32.0f * uiScale),
            displaySize.y - defaultPosition.y - 24.0f * uiScale });
    return ImGui::Begin(name, open, ImGuiWindowFlags_NoCollapse);
}

// Cresce a janela atual pra caber o conteudo que acabou de ser desenhado
// (mas nunca encolhe sozinha nem ultrapassa a tela) - so deve ser chamada
// enquanto o Begin correspondente retornou true (conteudo de verdade foi
// desenhado neste quadro).
void growPanelToFitContent(ImVec2 displaySize, float uiScale) {
    const float requiredHeight = ImGui::GetCursorPosY()
        + ImGui::GetStyle().WindowPadding.y;
    const ImVec2 currentSize = ImGui::GetWindowSize();
    const float maximumHeight = displaySize.y - ImGui::GetWindowPos().y
        - 12.0f * uiScale;
    if (requiredHeight > currentSize.y) {
        ImGui::SetWindowSize({ currentSize.x,
            std::min(requiredHeight, maximumHeight) });
    }
}

} // namespace

void WorkbenchApp::enterLaboratory(bool resetCamera) {
    if (!m_laboratoryInitialized || resetCamera) {
        if (m_laboratoryMapLoaded) {
            resetLaboratoryCharacter();
        } else {
            m_laboratoryCameraPosition = { 0.0f, -90.0f, 4.6f };
            m_laboratoryCameraYaw = 1.57079632679f;
        }
        m_laboratoryCameraPitch = -0.08f;
        m_laboratoryInitialized = true;
    }

    m_screen = Screen::Laboratory;
    m_laboratoryPaused = false;
    m_laboratoryConfirmQuit = false;
    m_laboratoryDebugVisible = false;
    m_spawnMenuOpen = false;
    m_laboratoryJumpRequested = false;
    m_laboratoryFlightToggleRequested = false;
    m_laboratoryTaaHistoryValid = false;
    m_taaFrameIndex = 0;
    m_discardNextMouseDelta = true;
    syncLaboratoryMouseCapture();
}

void WorkbenchApp::syncLaboratoryMouseCapture() {
    const bool capture = m_screen == Screen::Laboratory
        && m_laboratoryWindowFocused && !m_laboratoryPaused
        && !m_laboratoryDebugVisible && !m_spawnMenuOpen;
    renderer().setMouseCaptured(capture);
    if (capture) {
        m_discardNextMouseDelta = true;
    }
}

void WorkbenchApp::updateLaboratory(float deltaTime) {
    m_laboratoryElapsedTime += deltaTime;
    if (!m_laboratoryCharacterInitialized || !m_laboratoryMapLoaded
        || !m_physicsScene || !m_physicsScene->hasCharacter()) {
        return;
    }

    const PhysicsCharacterState3D stateBefore =
        m_physicsScene->characterState();

    CharacterMotorCommand3D command;
    const bool acceptsMovement = !m_laboratoryDebugVisible && !m_spawnMenuOpen;
    if (acceptsMovement) {
        const Vec3 planarForward = cameraForward(m_laboratoryCameraYaw, 0.0f);
        const Vec3 viewForward = cameraForward(m_laboratoryCameraYaw,
            m_laboratoryCameraPitch);
        // Em coordenadas Z-up, direita é forward × up. No voo, W/S usa
        // viewForward completo para acompanhar exatamente o ponto observado.
        const Vec3 forward = stateBefore.flying
            ? viewForward : planarForward;
        const Vec3 right { planarForward.y, -planarForward.x, 0.0f };
        if (input().keyDown(Key::W)) command.moveDirection += forward;
        if (input().keyDown(Key::S)) command.moveDirection -= forward;
        if (input().keyDown(Key::D)) command.moveDirection += right;
        if (input().keyDown(Key::A)) command.moveDirection -= right;
        command.sprint = input().keyDown(Key::LeftShift)
            || input().keyDown(Key::RightShift);
        const bool control = input().keyDown(Key::LeftControl)
            || input().keyDown(Key::RightControl);
        if (stateBefore.flying) {
            if (input().keyDown(Key::Space)) command.moveDirection.z += 1.0f;
            command.crouch = control;
        } else {
            command.crouch = control;
        }
        command.jumpPressed = m_laboratoryJumpRequested;
        command.toggleFlight = m_laboratoryFlightToggleRequested;
    } else {
        // Um painel aberto bloqueia novos comandos, mas não congela gravidade
        // nem força o personagem agachado a tentar levantar.
        command.crouch = stateBefore.crouched;
    }
    m_laboratoryJumpRequested = false;
    m_laboratoryFlightToggleRequested = false;

    m_physicsScene->moveCharacter(command, m_characterSettings, deltaTime);
    const PhysicsCharacterState3D& character =
        m_physicsScene->characterState();
    if (character.flightExitBlocked) {
        m_laboratoryStatus = "Saia da geometria para desativar o voo";
    } else if (m_laboratoryStatus
            == "Saia da geometria para desativar o voo") {
        m_laboratoryStatus.clear();
    }

    // A câmera lê a cápsula depois da física. Somente a altura dos olhos é
    // interpolada, evitando um salto visual ao agachar sem atrasar colisões.
    const float bodyHeight = character.crouched
        ? m_characterSettings.crouchedHeight
        : m_characterSettings.standingHeight;
    const float targetEyeHeight = character.crouched
        ? 1.02f : 1.68f;
    const float blend = 1.0f - std::exp(-14.0f * deltaTime);
    m_laboratoryEyeHeight +=
        (targetEyeHeight - m_laboratoryEyeHeight) * blend;
    const float feetZ = character.position.z - bodyHeight * 0.5f;
    m_laboratoryCameraPosition = {
        character.position.x,
        character.position.y,
        feetZ + m_laboratoryEyeHeight
    };
    updateDynamicProps(deltaTime);
}

void WorkbenchApp::ensureLaboratoryMapLoaded(Renderer& activeRenderer) {
    if (m_laboratoryMapLoaded || m_laboratoryMapLoadFailed) {
        return;
    }

    LoadedGltfModel model;
    try {
        model = loadGltfModel(std::string(MATTERENGINE_ASSETS_DIR)
            + "/models/lab_map.glb");
    } catch (const std::exception& error) {
        Log::error(std::string("Falha ao carregar lab_map.glb: ")
            + error.what());
        m_laboratoryMapLoadFailed = true;
        return;
    }

    m_laboratoryMapTextures.resize(model.images.size());
    for (std::size_t index = 0; index < model.images.size(); ++index) {
        const LoadedGltfImage& image = model.images[index];
        if (image.width <= 0 || image.height <= 0
            || image.rgbaPixels.empty()) {
            continue;
        }
        m_laboratoryMapTextures[index] = activeRenderer.createTexture2D(
            { static_cast<std::uint32_t>(image.width),
                static_cast<std::uint32_t>(image.height) },
            std::as_bytes(std::span(image.rgbaPixels)));
    }

    m_laboratoryMapParts.reserve(model.parts.size());
    for (const LoadedGltfPart& part : model.parts) {
        if (part.mesh.vertices.empty() || part.mesh.indices.empty()) {
            continue;
        }

        GltfPhysicsMetadata3D physicsMetadata;
        try {
            physicsMetadata = parseGltfPhysicsMetadata(part.extras);
        } catch (const std::exception& error) {
            Log::error("Metadados fisicos invalidos no node "
                + part.nodeName + ": " + error.what());
            m_laboratoryMapLoadFailed = true;
            return;
        }
        const std::string& materialId = physicsMetadata.materialId;
        if (m_materialLibrary.find(materialId) == nullptr) {
            Log::error("Material fisico nao registrado no laboratorio: "
                + materialId + " (node " + part.nodeName + ")");
            m_laboratoryMapLoadFailed = true;
            return;
        }
        try {
            const auto triangleMesh =
                m_physicsEngine.cookStaticTriangleMesh(part.mesh);
            PhysicsShape3D collisionShape;
            collisionShape.type = PhysicsShapeType3D::TriangleMesh;
            collisionShape.mesh = triangleMesh;
            collisionShape.materialId = materialId;
            PhysicsBodyDefinition3D staticBody;
            staticBody.motionType = PhysicsMotionType3D::Static;
            staticBody.materialId = materialId;
            staticBody.entityId = m_nextEntityId++;
            m_laboratoryMapBodies.push_back(
                m_physicsScene->createBody(staticBody,
                    std::span<const PhysicsShape3D>(&collisionShape, 1)));
        } catch (const std::exception& error) {
            Log::error("Falha no cooking PhysX do node " + part.nodeName
                + ": " + error.what());
            m_laboratoryMapLoadFailed = true;
            return;
        }

        LaboratoryMapPart gpuPart;
        const std::size_t vertexBytes = part.mesh.vertices.size()
            * sizeof(MeshVertex3D);
        const std::size_t indexBytes = part.mesh.indices.size()
            * sizeof(std::uint32_t);
        gpuPart.vertexBuffer = activeRenderer.createBuffer({ vertexBytes,
            RHI::BufferUsage::Vertex, true, "Laboratory map vertices" });
        gpuPart.indexBuffer = activeRenderer.createBuffer({ indexBytes,
            RHI::BufferUsage::Index, true, "Laboratory map indices" });
        activeRenderer.writeBuffer(gpuPart.vertexBuffer, 0,
            std::as_bytes(std::span(part.mesh.vertices)));
        activeRenderer.writeBuffer(gpuPart.indexBuffer, 0,
            std::as_bytes(std::span(part.mesh.indices)));
        gpuPart.indexCount = static_cast<std::uint32_t>(part.mesh.indices.size());
        gpuPart.name = part.nodeName;

        gpuPart.materialId = materialId;
        if (part.materialIndex >= 0
            && static_cast<std::size_t>(part.materialIndex) < model.materials.size()) {
            const LoadedGltfMaterial& material =
                model.materials[static_cast<std::size_t>(part.materialIndex)];
            gpuPart.metallic = material.metallicFactor;
            gpuPart.roughness = material.roughnessFactor;
            if (material.imageIndex >= 0
                && static_cast<std::size_t>(material.imageIndex)
                    < m_laboratoryMapTextures.size()) {
                gpuPart.albedoTexture = m_laboratoryMapTextures[
                    static_cast<std::size_t>(material.imageIndex)];
            }
        }
        m_laboratoryMapParts.push_back(std::move(gpuPart));
    }
    for (const LoadedGltfEntity& entity : model.entities) {
        const std::string* type = entity.extras.find("entity_type");
        if (type == nullptr || *type != "spawnpoint") {
            continue;
        }
        m_laboratorySpawnPosition = entity.position;
        if (entity.hasOrientation) {
            const Vec3 forward = entity.orientation.rotate({ 1.0f, 0.0f, 0.0f });
            m_laboratorySpawnYaw = std::atan2(forward.y, forward.x);
        }
        // Sem break: o mapa tambem pode conter zonas acusticas
        // (entity_type=acoustic_zone), coletadas logo abaixo.
    }
    m_worldAudio.setAcousticZones(parseAcousticZones(model.entities));

    // O mapa só pode publicar o estado "carregado" depois de todos os
    // recursos terem sido criados. Assim, uma exceção nunca deixa uma cena
    // parcialmente utilizável sendo tratada como pronta.
    m_laboratoryMapLoaded = true;
    resetLaboratoryCharacter();
    std::size_t triangleCount = 0;
    for (const LoadedGltfPart& part : model.parts) {
        triangleCount += part.mesh.indices.size() / 3;
    }
    Log::info("Mapa do laboratorio carregado no PhysX com "
        + std::to_string(triangleCount)
        + " triangulos estaticos em BVH34.");
}

void WorkbenchApp::resetLaboratoryCharacter() {
    if (!m_physicsScene) return;
    m_physicsScene->placeCharacter(m_laboratorySpawnPosition,
        m_characterSettings);
    m_laboratoryEyeHeight = 1.68f;
    m_laboratoryCameraPosition = m_laboratorySpawnPosition
        + Vec3 { 0.0f, 0.0f, m_laboratoryEyeHeight };
    m_laboratoryCameraYaw = m_laboratorySpawnYaw;
    m_laboratoryCameraPitch = -0.08f;
    m_laboratoryCharacterInitialized = true;
    m_laboratoryJumpRequested = false;
    m_laboratoryFlightToggleRequested = false;
    // Teleporte/camera cut nao possui correspondencia valida no quadro
    // anterior. O proximo render semeia novamente o historico temporal.
    m_laboratoryTaaHistoryValid = false;
}

void WorkbenchApp::releaseLaboratoryAssets(Renderer& activeRenderer) {
    endPhysGunGrab();
    if (m_physicsScene) {
        for (const SpawnedPropInstance& instance : m_spawnedProps) {
            m_physicsScene->destroyBody(instance.physicsBody);
        }
        for (const PhysicsBodyHandle3D body : m_laboratoryMapBodies) {
            m_physicsScene->destroyBody(body);
        }
        m_physicsScene->destroyCharacter();
    }
    m_spawnedProps.clear();
    m_spawnedPropByBodyIndex.clear();
    m_laboratoryMapBodies.clear();
    m_impactAcousticResolver.reset();
    m_propCatalog.release(activeRenderer);
    releaseGpuModel3D(activeRenderer, m_physGunVisual);
    m_physGunVisualLoaded = false;
    releaseGpuModel3D(activeRenderer, m_objectViewerFloor);
    m_objectViewerFloorLoaded = false;
    m_propAssetsLoadFailed = false;
    for (LaboratoryMapPart& part : m_laboratoryMapParts) {
        if (part.vertexBuffer) {
            activeRenderer.destroyBuffer(part.vertexBuffer);
            part.vertexBuffer = {};
        }
        if (part.indexBuffer) {
            activeRenderer.destroyBuffer(part.indexBuffer);
            part.indexBuffer = {};
        }
    }
    for (RHI::TextureHandle& texture : m_laboratoryMapTextures) {
        if (texture) {
            activeRenderer.destroyTexture(texture);
            texture = {};
        }
    }
    m_laboratoryMapParts.clear();
    m_laboratoryMapTextures.clear();
    m_laboratoryMapLoaded = false;
    m_laboratoryCharacterInitialized = false;
}

void WorkbenchApp::renderLaboratory3D(Renderer& activeRenderer) {
    ensureLaboratoryMapLoaded(activeRenderer);
    const bool temporalHistoryValid = m_laboratoryTaaHistoryValid;

    const float aspect = static_cast<float>(std::max(1, activeRenderer.width()))
        / static_cast<float>(std::max(1, activeRenderer.height()));
    const Mat4 view = Mat4::lookAt(m_laboratoryCameraPosition,
        m_laboratoryCameraPosition + cameraForward(
            m_laboratoryCameraYaw, m_laboratoryCameraPitch),
        { 0.0f, 0.0f, 1.0f });
    const Mat4 projection = Mat4::perspective(
        67.0f * Pi / 180.0f, aspect, 0.08f, LaboratoryFarPlane);
    m_laboratoryViewProjection = projection * view;

    // Fase 6 (TAA): amostra de Halton (base 2/3) convertida de [0,1) pra um
    // deslocamento sub-pixel em espaco NDC (2 unidades de NDC cobrem a tela
    // inteira em cada eixo, entao 1 pixel = 2/extent unidades de NDC) -
    // ver JitterSequence.hpp e Mat4::perspectiveJittered.
    const Vec2 haltonSample = haltonJitter(m_taaFrameIndex);
    const Vec2 ndcJitter {
        (haltonSample.x - 0.5f) * 2.0f
            / static_cast<float>(std::max(1, activeRenderer.width())),
        (haltonSample.y - 0.5f) * 2.0f
            / static_cast<float>(std::max(1, activeRenderer.height()))
    };
    const Mat4 jitteredProjection = Mat4::perspectiveJittered(
        67.0f * Pi / 180.0f, aspect, 0.08f, LaboratoryFarPlane, ndcJitter);
    const Mat4 cameraViewProjectionJittered = jitteredProjection * view;

    // Distancia maxima que a sombra precisa cobrir - deliberadamente bem
    // menor que LaboratoryFarPlane (2000m): nevoa/distancia ja escondem
    // qualquer sombra la longe (ver scene.fogSettings em scene3d_mesh.frag),
    // entao gastar cascatas ate o plano distante da camera desperdicaria a
    // cascata mais proxima (a que mais importa) numa faixa gigante demais.
    // 450m cobre todo o espaco jogavel atual e ainda permite uma transicao
    // final gradual antes da neblina dominar, sem gastar cascatas nos 2000m
    // do plano distante usados apenas pelo ceu/horizonte.
    constexpr float ShadowMaxDistanceMeters = 450.0f;
    // Precisa bater com SceneShadowMapSize em VulkanDevice.cpp - sem header
    // compartilhado entre este arquivo e o backend Vulkan (mesmo padrao ja
    // usado pelo bloco SceneUniform espelhado nos shaders).
    constexpr float ShadowMapResolutionTexels = 2048.0f;
    // Prioriza fortemente resolucao proxima. Lambda 0.5 produzia a primeira
    // fronteira perto de 38m (com far=300), tornando pernas/objetos pequenos
    // borrados mesmo ao lado do jogador.
    constexpr float CascadeSplitLambda = 0.85f;
    constexpr float CascadeBlendFraction = 0.12f;

    const Vec3& sunDirection = m_laboratorySunDirection;
    const Vec3 cameraForwardVector =
        cameraForward(m_laboratoryCameraYaw, m_laboratoryCameraPitch)
            .normalized();
    const CameraFrustumParameters3D cameraFrustumParameters {
        m_laboratoryCameraPosition, cameraForwardVector,
        { 0.0f, 0.0f, 1.0f }, 67.0f * Pi / 180.0f, aspect
    };
    const std::vector<float> cascadeFarDistances = computeCascadeSplits(
        0.08f, ShadowMaxDistanceMeters, ShadowCascadeCount, CascadeSplitLambda);

    std::array<Mat4, ShadowCascadeCount> cascadeViewProjections {};
    std::array<float, ShadowCascadeCount> cascadeSplits {};
    std::array<float, ShadowCascadeCount> cascadeTexelWorldSizes {};
    std::array<float, ShadowCascadeCount> cascadeDepthRanges {};
    std::array<Frustum3D, ShadowCascadeCount> lightFrustums {};
    float previousCascadeNear = 0.08f;
    for (std::uint32_t cascade = 0; cascade < ShadowCascadeCount; ++cascade) {
        const float cascadeFar = cascadeFarDistances.size() > cascade
            ? cascadeFarDistances[cascade] : ShadowMaxDistanceMeters;
        // Cascatas adjacentes se sobrepoem na faixa em que o shader mistura
        // as duas. Sem essa geometria extra, a segunda cascata seria lida
        // antes do seu near real e a transicao mostraria regioes vazias.
        const float nominalLength = cascadeFar - previousCascadeNear;
        const float fittedNear = cascade == 0 ? previousCascadeNear
            : std::max(0.08f, previousCascadeNear
                - nominalLength * CascadeBlendFraction);
        const FittedShadowCascade fitted = fitCascadeFrustumToCamera(
            cameraFrustumParameters, fittedNear, cascadeFar,
            sunDirection, ShadowMapResolutionTexels);
        cascadeViewProjections[cascade] = fitted.viewProjection;
        cascadeSplits[cascade] = cascadeFar;
        cascadeTexelWorldSizes[cascade] = fitted.texelWorldSizeMeters;
        cascadeDepthRanges[cascade] = fitted.depthRangeMeters;
        lightFrustums[cascade] = Frustum3D::fromViewProjection(
            cascadeViewProjections[cascade], /*reversedDepth=*/false);
        previousCascadeNear = cascadeFar;
    }

    const Frustum3D cameraFrustum =
        Frustum3D::fromViewProjection(m_laboratoryViewProjection,
            /*reversedDepth=*/true);

    std::vector<MeshRender3D> meshes;
    const auto& definitions = m_propCatalog.definitions();
    std::vector<std::uint8_t> propVisibility(m_spawnedProps.size(), 0);
    struct VisibilityContext {
        const std::vector<SpawnedPropInstance>* instances = nullptr;
        const std::vector<PropDefinition3D>* definitions = nullptr;
        const Frustum3D* camera = nullptr;
        // Uma por cascata - um prop conta como "projeta sombra" se estiver
        // dentro de QUALQUER uma delas (as fatias sao adjacentes ao longo da
        // distancia de camera, nao aninhadas, entao a uniao das 4 e o teste
        // correto, nao so a mais distante).
        const std::array<Frustum3D, ShadowCascadeCount>* lightCascades = nullptr;
        std::vector<std::uint8_t>* output = nullptr;
    } visibilityContext {
        &m_spawnedProps, &definitions, &cameraFrustum, &lightFrustums,
        &propVisibility
    };
    taskScheduler()->parallelFor(m_spawnedProps.size(), 64,
        [](std::size_t begin, std::size_t end, void* rawContext) noexcept {
            auto& context = *static_cast<VisibilityContext*>(rawContext);
            for (std::size_t index = begin; index < end; ++index) {
                const SpawnedPropInstance& instance =
                    (*context.instances)[index];
                if (instance.definitionIndex
                    >= context.definitions->size()) {
                    continue;
                }
                const Vec3 dimensions = (*context.definitions)[
                    instance.definitionIndex].dimensionsMeters;
                const float radius = dimensions.length() * 0.5f;
                const Vec3 center = instance.physicsState.position;
                std::uint8_t visibility = 0;
                if (context.camera->intersectsSphere(center, radius)) {
                    visibility |= 1u;
                }
                for (std::size_t cascade = 0;
                        cascade < context.lightCascades->size(); ++cascade) {
                    if ((*context.lightCascades)[cascade].intersectsSphere(
                            center, radius)) {
                        // Bit 0 pertence a camera; bits 1..4 representam as
                        // quatro cascatas individualmente.
                        visibility |= static_cast<std::uint8_t>(
                            1u << (cascade + 1u));
                    }
                }
                (*context.output)[index] = visibility;
            }
        }, &visibilityContext);
    std::vector<std::size_t> dynamicOffsets(m_spawnedProps.size() + 1, 0);
    for (std::size_t index = 0; index < m_spawnedProps.size(); ++index) {
        const SpawnedPropInstance& instance = m_spawnedProps[index];
        dynamicOffsets[index + 1] = dynamicOffsets[index];
        if (propVisibility[index] != 0
            && instance.definitionIndex < definitions.size()) {
            dynamicOffsets[index + 1] += definitions[instance.definitionIndex]
                .visual.parts.size();
        }
    }
    meshes.reserve(m_laboratoryMapParts.size() + dynamicOffsets.back()
        + m_physGunVisual.parts.size());
    for (const LaboratoryMapPart& part : m_laboratoryMapParts) {
        MeshRender3D mesh;
        mesh.vertexBuffer = part.vertexBuffer;
        mesh.indexBuffer = part.indexBuffer;
        mesh.indexCount = part.indexCount;
        mesh.albedoTexture = part.albedoTexture;
        mesh.metallic = part.metallic;
        mesh.roughness = part.roughness;
        meshes.push_back(mesh);
    }

    const std::size_t dynamicBase = meshes.size();
    meshes.resize(dynamicBase + dynamicOffsets.back());
    struct RenderPreparationContext {
        const std::vector<SpawnedPropInstance>* instances = nullptr;
        const std::vector<PropDefinition3D>* definitions = nullptr;
        const std::vector<std::size_t>* offsets = nullptr;
        const std::vector<std::uint8_t>* visibility = nullptr;
        std::vector<MeshRender3D>* output = nullptr;
        std::size_t outputBase = 0;
        std::uint64_t heldEntityId = 0;
        bool temporalHistoryValid = false;
    } preparation {
        &m_spawnedProps, &definitions, &dynamicOffsets, &propVisibility,
        &meshes,
        dynamicBase, m_physGunGrabbedEntityId, temporalHistoryValid
    };
    taskScheduler()->parallelFor(m_spawnedProps.size(), 64,
        [](std::size_t begin, std::size_t end, void* rawContext) noexcept {
            auto& context = *static_cast<RenderPreparationContext*>(
                rawContext);
            for (std::size_t index = begin; index < end; ++index) {
                const SpawnedPropInstance& instance =
                    (*context.instances)[index];
                if (instance.definitionIndex
                        >= context.definitions->size()
                    || (*context.visibility)[index] == 0) {
                    continue;
                }
                const PropDefinition3D& definition =
                    (*context.definitions)[instance.definitionIndex];
                const bool highlighted = instance.entityId
                        == context.heldEntityId
                    || instance.freezeFlashSeconds > 0.0f;
                const std::size_t first = context.outputBase
                    + (*context.offsets)[index];
                const bool visibleInCamera =
                    ((*context.visibility)[index] & 1u) != 0;
                const bool castsShadow =
                    ((*context.visibility)[index] & 0x1Eu) != 0;
                const std::uint8_t shadowCascadeMask =
                    static_cast<std::uint8_t>(
                        (*context.visibility)[index] >> 1u);
                std::span<MeshRender3D> destination =
                    std::span<MeshRender3D>(*context.output).subspan(
                        first, definition.visual.parts.size());
                writeGpuModelRenderables(definition.visual,
                    instance.physicsState.position,
                    instance.physicsState.orientation,
                    context.temporalHistoryValid
                        ? instance.previousPhysicsState.position
                        : instance.physicsState.position,
                    context.temporalHistoryValid
                        ? instance.previousPhysicsState.orientation
                        : instance.physicsState.orientation,
                    1.0f,
                    highlighted, highlighted, castsShadow, destination);
                for (MeshRender3D& mesh : destination) {
                    mesh.visibleInCamera = visibleInCamera;
                    mesh.shadowCascadeMask = shadowCascadeMask;
                }
            }
        }, &preparation);

    Vec3 renderedWeaponPosition;
    Quaternion renderedWeaponOrientation;
    bool renderedWeapon = false;
    if (m_physGunVisualLoaded && !m_physGunVisual.parts.empty()) {
        const Vec3 forward = cameraForward(m_laboratoryCameraYaw,
            m_laboratoryCameraPitch).normalized();
        const Vec3 planarForward = cameraForward(
            m_laboratoryCameraYaw, 0.0f);
        const Vec3 right { planarForward.y, -planarForward.x, 0.0f };
        const Vec3 cameraUp = cross(right, forward).normalized();
        const Vec3 dimensions = m_physGunVisual.dimensionsMeters;
        const float largest = std::max({ dimensions.x,
            dimensions.y, dimensions.z, 0.001f });
        const Quaternion viewOrientation =
            Quaternion::fromAxisAngle({ 0.0f, 0.0f, 1.0f },
                m_laboratoryCameraYaw)
            * Quaternion::fromAxisAngle({ 0.0f, 1.0f, 0.0f },
                -m_laboratoryCameraPitch);
        const Vec3 rotationRadians =
            m_physGunViewCalibration.rotationDegrees * (Pi / 180.0f);
        const Quaternion localCalibration =
            Quaternion::fromAxisAngle({ 1.0f, 0.0f, 0.0f },
                rotationRadians.x)
            * Quaternion::fromAxisAngle({ 0.0f, 1.0f, 0.0f },
                rotationRadians.y)
            * Quaternion::fromAxisAngle({ 0.0f, 0.0f, 1.0f },
                rotationRadians.z);
        Vec3 localOffset = m_physGunViewCalibration.offsetCameraLocal;
        localOffset.y -= m_physGunRecoilOffset;
        const Vec3 weaponPosition = m_laboratoryCameraPosition
            + right * localOffset.x + forward * localOffset.y
            + cameraUp * localOffset.z;
        const Quaternion weaponOrientation = viewOrientation * localCalibration;
        appendGpuModelRenderables(m_physGunVisual, weaponPosition,
            weaponOrientation,
            temporalHistoryValid ? m_previousPhysGunWeaponPosition
                : weaponPosition,
            temporalHistoryValid ? m_previousPhysGunWeaponOrientation
                : weaponOrientation,
            m_physGunViewCalibration.sizeMeters / largest,
            false, false, false,
            meshes);
        renderedWeaponPosition = weaponPosition;
        renderedWeaponOrientation = weaponOrientation;
        renderedWeapon = true;

        const Vec3 beamStart = m_physGunBeamStartCameraLocal;
        m_physGunBeamStartWorldPosition = m_laboratoryCameraPosition
            + right * beamStart.x
            + forward * (beamStart.y - m_physGunRecoilOffset)
            + cameraUp * beamStart.z;

    }

    std::array<LightRender3D, 2> lights;
    lights[0].type = LightType3D::Directional;
    lights[0].direction = sunDirection;
    lights[0].intensity = 1.08f;
    lights[0].castsShadow = true;
    lights[1].type = LightType3D::Spot;
    lights[1].position = m_physGunBeamStartWorldPosition;
    lights[1].direction = cameraForward(m_laboratoryCameraYaw,
        m_laboratoryCameraPitch).normalized();
    lights[1].color = m_physGunFlashlight.color;
    lights[1].range = m_physGunFlashlight.enabled
        ? m_physGunFlashlight.rangeMeters : 0.0f;
    lights[1].intensity = m_physGunFlashlight.enabled
        ? m_physGunFlashlight.intensity : 0.0f;
    lights[1].coneOuterDegrees = m_physGunFlashlight.coneDegrees;

    Scene3DFrame scene;
    scene.cameraViewProjection = m_laboratoryViewProjection;
    scene.cameraViewProjectionJittered = cameraViewProjectionJittered;
    scene.previousCameraViewProjection = temporalHistoryValid
        ? m_previousLaboratoryViewProjection
        : m_laboratoryViewProjection;
    scene.taaFrameIndex = m_taaFrameIndex;
    scene.temporalAntiAliasingEnabled = true;
    scene.resetTemporalHistory = !temporalHistoryValid;
    scene.cascadeViewProjections = cascadeViewProjections;
    scene.cascadeSplits = cascadeSplits;
    scene.cascadeTexelWorldSizes = cascadeTexelWorldSizes;
    scene.cascadeDepthRanges = cascadeDepthRanges;
    scene.cameraPosition = m_laboratoryCameraPosition;
    scene.lights = lights;
    scene.meshes = meshes;
    scene.ambientLight = 0.31f;
    scene.skyTime = m_laboratoryElapsedTime;
    scene.cloudCoverage = 0.48f;
    scene.showShadows = true;
    scene.showSky = true;
    scene.toneMapping = m_laboratoryToneMapping;
    scene.fog = m_laboratoryFog;
    scene.cloudWindOffset = m_cloudWindOffset;
    if (activeRenderer.renderScene3DToScreen(scene)) {
        // Estado temporal avanca na cadencia de RENDER, nao na cadencia fixa
        // da fisica. Em FPS maior que 120, repetir o mesmo delta de transform
        // em varios quadros geraria vetores de movimento falsos.
        for (SpawnedPropInstance& instance : m_spawnedProps) {
            instance.previousPhysicsState = instance.physicsState;
        }
        if (renderedWeapon) {
            m_previousPhysGunWeaponPosition = renderedWeaponPosition;
            m_previousPhysGunWeaponOrientation = renderedWeaponOrientation;
        }
        m_previousLaboratoryViewProjection = m_laboratoryViewProjection;
        m_laboratoryTaaHistoryValid = true;
        ++m_taaFrameIndex;
    }
}

void WorkbenchApp::drawSpawnMenu() {
    const ImGuiIO& io = ImGui::GetIO();
    // O escurecimento pertence ao fundo da interface: a cena fica atenuada,
    // enquanto a janela do menu permanece nítida e totalmente legível.
    ImDrawList* background = ImGui::GetBackgroundDrawList();
    background->AddRectFilled({ 0.0f, 0.0f }, io.DisplaySize,
        IM_COL32(2, 7, 13, 160));

    const float width = std::min(ui(870.0f), io.DisplaySize.x - ui(36.0f));
    const float height = std::min(ui(600.0f),
        io.DisplaySize.y - ui(44.0f));
    ImGui::SetNextWindowPos({ (io.DisplaySize.x - width) * 0.5f,
        (io.DisplaySize.y - height) * 0.5f }, ImGuiCond_Always);
    ImGui::SetNextWindowSize({ width, height }, ImGuiCond_Always);
    ImGui::Begin("SpawnMenu", nullptr,
        ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove
            | ImGuiWindowFlags_NoSavedSettings
            | ImGuiWindowFlags_NoScrollbar);
    ImGui::SetWindowFontScale(1.18f);
    ImGui::TextUnformatted("OBJETOS");
    ImGui::SetWindowFontScale(1.0f);
    subtleSeparator();
    const auto& definitions = m_propCatalog.definitions();
    if (definitions.empty()) {
        ImGui::Dummy({ 1.0f, ui(28.0f) });
        centeredTextColored({ 0.55f, 0.68f, 0.78f, 1.0f },
            m_propAssetsLoadFailed
                ? "Falha ao carregar objetos" : "Carregando objetos");
        ImGui::End();
        return;
    }

    ImGui::Dummy({ 1.0f, ui(10.0f) });
    constexpr int Columns = 3;
    const float gap = ui(10.0f);
    const float contentWidth = ImGui::GetContentRegionAvail().x;
    const float cardWidth = (contentWidth
        - gap * static_cast<float>(Columns - 1))
        / static_cast<float>(Columns);
    const float cardHeight = ui(238.0f);
    ImDrawList* drawList = ImGui::GetWindowDrawList();
    for (std::size_t index = 0; index < definitions.size(); ++index) {
        if (index % Columns != 0) ImGui::SameLine(0.0f, gap);
        const PropDefinition3D& definition = definitions[index];
        ImGui::PushID(static_cast<int>(index));
        const ImVec2 cardMin = ImGui::GetCursorScreenPos();
        ImGui::InvisibleButton("PropCard", { cardWidth, cardHeight });
        const bool hovered = ImGui::IsItemHovered();
        const bool clicked = ImGui::IsItemClicked(ImGuiMouseButton_Left);
        const ImVec2 cardMax { cardMin.x + cardWidth,
            cardMin.y + cardHeight };
        drawList->AddRectFilled(cardMin, cardMax,
            hovered ? IM_COL32(15, 34, 49, 255)
                    : IM_COL32(10, 23, 34, 255), ui(7.0f));
        drawList->AddRect(cardMin, cardMax,
            hovered ? UiAccent : UiBorder, ui(7.0f), 0,
            hovered ? ui(1.5f) : ui(1.0f));

        const ImVec2 previewMin { cardMin.x + ui(5.0f),
            cardMin.y + ui(5.0f) };
        const ImVec2 previewMax { cardMax.x - ui(5.0f),
            cardMax.y - ui(39.0f) };
        if (m_propPreviewAtlas) {
            const float column = static_cast<float>(index % 3);
            const float row = static_cast<float>(index / 3);
            drawList->AddImage(
                static_cast<ImTextureID>(m_propPreviewAtlas.id),
                previewMin, previewMax,
                { column / 3.0f, row / 2.0f },
                { (column + 1.0f) / 3.0f,
                    (row + 1.0f) / 2.0f });
        }
        const ImVec2 nameSize = ImGui::CalcTextSize(
            definition.displayName.c_str());
        drawList->AddText({ cardMin.x
                + (cardWidth - nameSize.x) * 0.5f,
                cardMax.y - ui(27.0f) }, UiText,
            definition.displayName.c_str());

        if (hovered) {
            ImGui::BeginTooltip();
            ImGui::TextUnformatted(definition.displayName.c_str());
            subtleSeparator();
            ImGui::Text("Material: %s", definition.materialId.c_str());
            ImGui::Text("Massa: %.2f kg", definition.massKg);
            ImGui::Text("Dimensões: %.2f x %.2f x %.2f m",
                definition.dimensionsMeters.x,
                definition.dimensionsMeters.y,
                definition.dimensionsMeters.z);
            ImGui::EndTooltip();
        }
        if (clicked) spawnProp(index);
        ImGui::PopID();
    }
    ImGui::End();
}

void WorkbenchApp::drawLaboratoryPauseMenu() {
    const ImGuiIO& io = ImGui::GetIO();
    // Assim como no menu de objetos, o overlay não pode cobrir o próprio
    // diálogo. A camada de fundo preserva a hierarquia visual correta.
    ImDrawList* background = ImGui::GetBackgroundDrawList();
    background->AddRectFilled({ 0.0f, 0.0f }, io.DisplaySize,
        IM_COL32(1, 5, 10, 205));

    const float width = ui(340.0f);
    ImGui::SetNextWindowPos({ (io.DisplaySize.x - width) * 0.5f,
        io.DisplaySize.y * 0.22f }, ImGuiCond_Always);
    ImGui::SetNextWindowSize({ width, 0.0f }, ImGuiCond_Always);
    ImGui::Begin("LaboratoryPause", nullptr,
        ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove
            | ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_AlwaysAutoResize);
    ImGui::SetWindowFontScale(1.30f);
    ImGui::TextUnformatted("LABORATÓRIO");
    ImGui::SetWindowFontScale(1.0f);
    ImGui::Dummy({ 1.0f, ui(12.0f) });
    const ImVec2 buttonSize { -1.0f, ui(42.0f) };
    if (ImGui::Button("Continuar", buttonSize)) {
        m_laboratoryPaused = false;
        m_laboratoryConfirmQuit = false;
        syncLaboratoryMouseCapture();
    }
    if (ImGui::Button("Configurações", buttonSize)) {
        m_settingsReturnScreen = Screen::Laboratory;
        m_screen = Screen::Settings;
    }
    if (ImGui::Button("Menu principal", buttonSize)) {
        m_laboratoryPaused = false;
        m_laboratoryConfirmQuit = false;
        m_screen = Screen::MainMenu;
        renderer().setMouseCaptured(false);
    }
    if (!m_laboratoryConfirmQuit) {
        if (ImGui::Button("Sair", buttonSize)) {
            m_laboratoryConfirmQuit = true;
        }
    } else {
        ImGui::TextColored({ 1.0f, 0.55f, 0.48f, 1.0f },
            "Encerrar o MatterEngine?");
        if (ImGui::Button("Confirmar", { ui(145.0f), ui(38.0f) })) {
            requestQuit();
        }
        ImGui::SameLine();
        if (ImGui::Button("Cancelar", { ui(145.0f), ui(38.0f) })) {
            m_laboratoryConfirmQuit = false;
        }
    }
    ImGui::End();
}

void WorkbenchApp::drawLaboratoryUi() {
    const ImGuiIO& io = ImGui::GetIO();
    ImDrawList* foreground = ImGui::GetForegroundDrawList();

    char headerStatus[32] {};
    if (m_showPerformanceOverlay) {
        std::snprintf(headerStatus, sizeof(headerStatus), "%.0f FPS",
            io.Framerate);
    }
    // Desenhado no background draw list (nao no foreground usado pelo resto
    // desta funcao) de proposito: o foreground sempre renderiza por cima de
    // TODAS as janelas ImGui, entao o retangulo opaco do cabecalho cobriria
    // os botoes da barra de depuracao (ver mais abaixo) mesmo que eles
    // sejam desenhados depois - o unico jeito da barra ficar por cima do
    // cabecalho e o cabecalho nao estar no foreground. Isso nao muda nada
    // visualmente pro resto da cena 3D (ja renderizada via Vulkan antes de
    // qualquer coisa do ImGui, background inclusive).
    drawWorkbenchHeader(ImGui::GetBackgroundDrawList(), io.DisplaySize,
        m_uiScale, "LABORATÓRIO", headerStatus);

    const ImVec2 center { io.DisplaySize.x * 0.5f, io.DisplaySize.y * 0.5f };
    foreground->AddLine({ center.x - ui(5.0f), center.y },
        { center.x + ui(5.0f), center.y }, IM_COL32(190, 222, 245, 205));
    foreground->AddLine({ center.x, center.y - ui(5.0f) },
        { center.x, center.y + ui(5.0f) }, IM_COL32(190, 222, 245, 205));

    if (m_physGunTriggerHeld) {
        ImVec2 beamStartScreen;
        ImVec2 beamEndScreen = center;
        if (projectToScreen(m_laboratoryViewProjection,
                m_physGunBeamStartWorldPosition, io.DisplaySize,
                beamStartScreen)) {
            if (m_physGunGrabbedEntityId != 0) {
                ImVec2 heldPointScreen;
                if (projectToScreen(m_laboratoryViewProjection,
                        m_physGunBeamTargetWorldPosition,
                        io.DisplaySize, heldPointScreen)) {
                    beamEndScreen = heldPointScreen;
                }
            }
            // Overlay deliberado: o feixe pertence à apresentação da arma e
            // permanece legível independentemente da profundidade da cena.
            foreground->AddLine(beamStartScreen, beamEndScreen,
                IM_COL32(0, 150, 255, 36), ui(8.5f));
            foreground->AddLine(beamStartScreen, beamEndScreen,
                IM_COL32(0, 206, 255, 112), ui(3.8f));
            foreground->AddLine(beamStartScreen, beamEndScreen,
                IM_COL32(184, 249, 255, 250), ui(1.3f));
            foreground->AddCircleFilled(beamStartScreen, ui(4.0f),
                IM_COL32(38, 211, 255, 190));
            foreground->AddCircleFilled(beamEndScreen, ui(2.8f),
                IM_COL32(195, 250, 255, 225));
        }
    }

    if (!m_laboratoryStatus.empty()) {
        const ImVec2 size = ImGui::CalcTextSize(m_laboratoryStatus.c_str());
        foreground->AddText({ (io.DisplaySize.x - size.x) * 0.5f,
            io.DisplaySize.y - ui(44.0f) }, UiMuted,
            m_laboratoryStatus.c_str());
    }

    if (m_notification.secondsRemaining > 0.0f
        && !m_notification.text.empty()) {
        const float elapsed = m_notification.durationSeconds
            - m_notification.secondsRemaining;
        const float alpha = std::clamp(std::min(
            elapsed / 0.16f,
            m_notification.secondsRemaining / 0.42f), 0.0f, 1.0f);
        const ImVec2 textSize = ImGui::CalcTextSize(
            m_notification.text.c_str());
        const ImVec2 minimum { ui(18.0f),
            io.DisplaySize.y - ui(36.0f) - textSize.y };
        const ImVec2 maximum { minimum.x + textSize.x + ui(38.0f),
            minimum.y + textSize.y + ui(20.0f) };
        foreground->AddRectFilled(minimum, maximum,
            IM_COL32(5, 16, 25, static_cast<int>(218.0f * alpha)),
            ui(6.0f));
        foreground->AddRectFilled(minimum,
            { minimum.x + ui(3.0f), maximum.y },
            IM_COL32(17, 166, 255, static_cast<int>(240.0f * alpha)),
            ui(6.0f));
        foreground->AddText({ minimum.x + ui(18.0f),
            minimum.y + ui(10.0f) },
            IM_COL32(205, 229, 244, static_cast<int>(255.0f * alpha)),
            m_notification.text.c_str());
    }

    if (m_laboratoryDebugVisible) {
        // Botoes na MESMA linha do cabecalho "MATTERENGINE | LABORATÓRIO"
        // (nao uma segunda barra abaixo dele) - cada um abre/fecha seu
        // proprio painel flutuante e independente (ver beginDebugPanel), em
        // vez da unica janela com abas de antes. A posicao X replica a
        // mesma matematica que drawWorkbenchHeader (UiKit.cpp) usa pra
        // desenhar "LABORATÓRIO", entao os botoes sempre nascem logo depois
        // desse texto, nunca por cima dele - sem precisar alterar aquele
        // componente, que tambem e usado por outras telas.
        const ImVec2 brandSize = ImGui::CalcTextSize("MATTERENGINE");
        const ImVec2 contextSize = ImGui::CalcTextSize("LABORATÓRIO");
        const float toolbarStartX = ui(76.0f) + brandSize.x + contextSize.x
            + ui(28.0f);
        const float toolbarButtonHeight = ui(30.0f);
        const float toolbarY =
            (headerHeight(m_uiScale) - toolbarButtonHeight) * 0.5f;
        // ImGuiWindowFlags_NoDecoration NAO remove o WindowPadding padrao
        // do ImGui (8,8 nao escalado) - sem empurrar isso pra zero, a
        // janela reservava altura so pro botao (toolbarButtonHeight+4) e o
        // padding de cima/baixo cortava a base de cada botao. Mesmo
        // mecanismo ja usado em SettingsScreen.cpp (drawVideoConfirmPopup).
        const ImVec2 toolbarPadding { ui(4.0f), ui(2.0f) };
        ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, toolbarPadding);

        ImGui::SetNextWindowPos({ toolbarStartX, toolbarY }, ImGuiCond_Always);
        ImGui::SetNextWindowSize(
            { io.DisplaySize.x - toolbarStartX - ui(90.0f),
                toolbarButtonHeight + toolbarPadding.y * 2.0f },
            ImGuiCond_Always);
        ImGui::Begin("LaboratoryDebugToolbar", nullptr,
            ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove
                | ImGuiWindowFlags_NoSavedSettings
                | ImGuiWindowFlags_NoFocusOnAppearing
                | ImGuiWindowFlags_NoBackground);
        const ImVec2 toolbarButtonSize { ui(130.0f), toolbarButtonHeight };
        if (tabButton("Physgun", m_showPhysgunPanel, toolbarButtonSize)) {
            m_showPhysgunPanel = !m_showPhysgunPanel;
        }
        ImGui::SameLine();
        if (tabButton("Gráficos", m_showGraphicsPanel, toolbarButtonSize)) {
            m_showGraphicsPanel = !m_showGraphicsPanel;
        }
        ImGui::SameLine();
        if (tabButton("Performance", m_showPerformancePanel,
                toolbarButtonSize)) {
            m_showPerformancePanel = !m_showPerformancePanel;
        }
        ImGui::End();
        ImGui::PopStyleVar();

        const float panelWidth = ui(390.0f);
        const float panelTop = headerHeight(m_uiScale) + ui(12.0f);
        if (m_showPhysgunPanel && beginDebugPanel("Physgun",
                &m_showPhysgunPanel,
                { io.DisplaySize.x - panelWidth - ui(12.0f), panelTop },
                { panelWidth, ui(430.0f) }, io.DisplaySize, m_uiScale)) {
            {
                    panelHeader("FORÇA DE MANIPULAÇÃO");
                    ImGui::Dummy({ 1.0f, ui(8.0f) });
                    ImGui::SetNextItemWidth(-1.0f);
                    ImGui::SliderFloat("##PhysGunMaximumForce",
                        &m_physGunSettings.maximumForce,
                        250.0f, 500000.0f, "%.0f N",
                        ImGuiSliderFlags_Logarithmic
                            | ImGuiSliderFlags_AlwaysClamp);
                    ImGui::Dummy({ 1.0f, ui(12.0f) });
                    panelHeader("COMPORTAMENTO");
                    int holdMode = m_physGunHoldMode
                            == PhysGunHoldMode::GrabPoint
                        ? 0 : 1;
                    ImGui::SetNextItemWidth(-1.0f);
                    if (ImGui::Combo("##PhysGunHoldMode", &holdMode,
                            "Ponto de grab\0Pose fixa\0")) {
                        m_physGunHoldMode = holdMode == 0
                            ? PhysGunHoldMode::GrabPoint
                            : PhysGunHoldMode::FixedPose;
                    }
                    ImGui::Dummy({ 1.0f, ui(7.0f) });
                    ImGui::TextColored({ 0.50f, 0.62f, 0.72f, 1.0f },
                        "Suavidade da rotação");
                    ImGui::SetNextItemWidth(-1.0f);
                    ImGui::SliderFloat("##PhysGunRotationSmoothness",
                        &m_physGunRotationSmoothness,
                        0.0f, 1.0f, "%.2f",
                        ImGuiSliderFlags_AlwaysClamp);
                    ImGui::Dummy({ 1.0f, ui(7.0f) });
                    ImGui::TextColored({ 0.50f, 0.62f, 0.72f, 1.0f },
                        "Passo angular do Shift + E");
                    ImGui::SetNextItemWidth(-1.0f);
                    ImGui::SliderFloat("##PhysGunRotationSnap",
                        &m_physGunRotationSnapDegrees,
                        1.0f, 90.0f, "%.0f°",
                        ImGuiSliderFlags_AlwaysClamp);

                    ImGui::Dummy({ 1.0f, ui(12.0f) });
                    if (ImGui::CollapsingHeader("Ajuste visual")) {
                        ImGui::TextColored({ 0.50f, 0.62f, 0.72f, 1.0f },
                            "Eixos locais da câmera");
                        ImGui::SetNextItemWidth(-1.0f);
                        ImGui::DragFloat3("Posição X/Y/Z",
                            &m_physGunViewCalibration.offsetCameraLocal.x,
                            0.005f, -2.0f, 2.0f, "%.3f m");
                        ImGui::SetNextItemWidth(-1.0f);
                        ImGui::DragFloat3("Rotação X/Y/Z",
                            &m_physGunViewCalibration.rotationDegrees.x,
                            0.25f, -180.0f, 180.0f, "%.2f°");
                        ImGui::SetNextItemWidth(-1.0f);
                        ImGui::DragFloat("Tamanho",
                            &m_physGunViewCalibration.sizeMeters,
                            0.005f, 0.05f, 2.5f, "%.3f m");
                        if (ImGui::Button("Copiar valores",
                                { -1.0f, ui(36.0f) })) {
                            char calibration[384] {};
                            std::snprintf(calibration, sizeof(calibration),
                                "Physgun: posição={%.3ff, %.3ff, %.3ff}; "
                                "rotação={%.2ff, %.2ff, %.2ff} graus; "
                                "tamanho=%.3ff",
                                m_physGunViewCalibration.offsetCameraLocal.x,
                                m_physGunViewCalibration.offsetCameraLocal.y,
                                m_physGunViewCalibration.offsetCameraLocal.z,
                                m_physGunViewCalibration.rotationDegrees.x,
                                m_physGunViewCalibration.rotationDegrees.y,
                                m_physGunViewCalibration.rotationDegrees.z,
                                m_physGunViewCalibration.sizeMeters);
                            ImGui::SetClipboardText(calibration);
                            m_physGunCalibrationCopyStatus =
                                "Valores copiados";
                        }
                        if (!m_physGunCalibrationCopyStatus.empty()) {
                            ImGui::TextColored(
                                { 0.38f, 0.82f, 0.62f, 1.0f },
                                "%s",
                                m_physGunCalibrationCopyStatus.c_str());
                        }
                    }

                    if (ImGui::CollapsingHeader("Origem do feixe")) {
                        ImGui::TextColored({ 0.50f, 0.62f, 0.72f, 1.0f },
                            "Eixos locais da câmera");
                        ImGui::SetNextItemWidth(-1.0f);
                        ImGui::DragFloat3("Origem X/Y/Z",
                            &m_physGunBeamStartCameraLocal.x,
                            0.0025f, -2.0f, 3.0f, "%.3f m");
                        if (ImGui::Button("Copiar origem do feixe",
                                { -1.0f, ui(36.0f) })) {
                            char calibration[256] {};
                            std::snprintf(calibration, sizeof(calibration),
                                "Feixe da Physgun: origem={%.3ff, %.3ff, %.3ff}",
                                m_physGunBeamStartCameraLocal.x,
                                m_physGunBeamStartCameraLocal.y,
                                m_physGunBeamStartCameraLocal.z);
                            ImGui::SetClipboardText(calibration);
                            m_physGunBeamCalibrationCopyStatus =
                                "Origem copiada";
                        }
                        if (!m_physGunBeamCalibrationCopyStatus.empty()) {
                            ImGui::TextColored(
                                { 0.38f, 0.82f, 0.62f, 1.0f },
                                "%s",
                                m_physGunBeamCalibrationCopyStatus.c_str());
                        }
                    }

                    if (ImGui::CollapsingHeader("Lanterna")) {
                        ImGui::Checkbox("Ativa (F)",
                            &m_physGunFlashlight.enabled);
                        ImGui::SetNextItemWidth(-1.0f);
                        ImGui::ColorEdit3("Cor",
                            &m_physGunFlashlight.color.x,
                            ImGuiColorEditFlags_NoInputs);
                        ImGui::SetNextItemWidth(-1.0f);
                        ImGui::SliderFloat("Abertura do cone",
                            &m_physGunFlashlight.coneDegrees,
                            6.0f, 120.0f, "%.0f°");
                        ImGui::SetNextItemWidth(-1.0f);
                        ImGui::SliderFloat("Distância",
                            &m_physGunFlashlight.rangeMeters,
                            2.0f, 100.0f, "%.1f m",
                            ImGuiSliderFlags_Logarithmic);
                        ImGui::SetNextItemWidth(-1.0f);
                        ImGui::SliderFloat("Brilho",
                            &m_physGunFlashlight.intensity,
                            0.1f, 20.0f, "%.2f",
                            ImGuiSliderFlags_Logarithmic);
                    }
            }
            growPanelToFitContent(io.DisplaySize, m_uiScale);
        }
        if (m_showPhysgunPanel) ImGui::End();

        if (m_showGraphicsPanel && beginDebugPanel("Gráficos",
                &m_showGraphicsPanel,
                { io.DisplaySize.x - panelWidth - ui(12.0f),
                    panelTop + ui(24.0f) },
                { panelWidth, ui(430.0f) }, io.DisplaySize, m_uiScale)) {
            {
                    panelHeader("TONEMAP (HDR → LDR)");
                    ImGui::TextColored({ 0.50f, 0.62f, 0.72f, 1.0f },
                        "Exposição");
                    ImGui::SetNextItemWidth(-1.0f);
                    ImGui::SliderFloat("##ToneMappingExposure",
                        &m_laboratoryToneMapping.exposure,
                        0.05f, 2.0f, "%.3f",
                        ImGuiSliderFlags_Logarithmic
                            | ImGuiSliderFlags_AlwaysClamp);
                    ImGui::Dummy({ 1.0f, ui(7.0f) });
                    ImGui::TextColored({ 0.50f, 0.62f, 0.72f, 1.0f },
                        "Brilho");
                    ImGui::SetNextItemWidth(-1.0f);
                    ImGui::SliderFloat("##ToneMappingBrightness",
                        &m_laboratoryToneMapping.brightness,
                        -0.5f, 0.5f, "%.3f",
                        ImGuiSliderFlags_AlwaysClamp);
                    ImGui::Dummy({ 1.0f, ui(7.0f) });
                    ImGui::TextColored({ 0.50f, 0.62f, 0.72f, 1.0f },
                        "Contraste");
                    ImGui::SetNextItemWidth(-1.0f);
                    ImGui::SliderFloat("##ToneMappingContrast",
                        &m_laboratoryToneMapping.contrast,
                        0.0f, 2.0f, "%.3f",
                        ImGuiSliderFlags_AlwaysClamp);
                    ImGui::Dummy({ 1.0f, ui(7.0f) });
                    ImGui::TextColored({ 0.50f, 0.62f, 0.72f, 1.0f },
                        "Saturação");
                    ImGui::SetNextItemWidth(-1.0f);
                    ImGui::SliderFloat("##ToneMappingSaturation",
                        &m_laboratoryToneMapping.saturation,
                        0.0f, 2.0f, "%.3f",
                        ImGuiSliderFlags_AlwaysClamp);

                    ImGui::Dummy({ 1.0f, ui(12.0f) });
                    panelHeader("NEBLINA");
                    ImGui::TextColored({ 0.50f, 0.62f, 0.72f, 1.0f },
                        "Densidade (queda por metro)");
                    ImGui::SetNextItemWidth(-1.0f);
                    ImGui::SliderFloat("##FogDensity",
                        &m_laboratoryFog.density,
                        0.0f, 0.05f, "%.4f",
                        ImGuiSliderFlags_AlwaysClamp);
                    ImGui::Dummy({ 1.0f, ui(7.0f) });
                    ImGui::TextColored({ 0.50f, 0.62f, 0.72f, 1.0f },
                        "Acoplamento altura-distância");
                    ImGui::SetNextItemWidth(-1.0f);
                    ImGui::SliderFloat("##FogHeightFalloff",
                        &m_laboratoryFog.heightFalloff,
                        0.0f, 0.10f, "%.4f",
                        ImGuiSliderFlags_AlwaysClamp);
                    ImGui::Dummy({ 1.0f, ui(7.0f) });
                    ImGui::TextColored({ 0.50f, 0.62f, 0.72f, 1.0f },
                        "Opacidade máxima");
                    ImGui::SetNextItemWidth(-1.0f);
                    ImGui::SliderFloat("##FogMaxOpacity",
                        &m_laboratoryFog.maxOpacity,
                        0.0f, 1.0f, "%.3f",
                        ImGuiSliderFlags_AlwaysClamp);
                    ImGui::Dummy({ 1.0f, ui(7.0f) });
                    ImGui::TextColored({ 0.50f, 0.62f, 0.72f, 1.0f },
                        "Cor");
                    ImGui::SetNextItemWidth(-1.0f);
                    ImGui::ColorEdit3("##FogColor",
                        &m_laboratoryFog.color.x,
                        ImGuiColorEditFlags_NoInputs);

                    ImGui::Dummy({ 1.0f, ui(12.0f) });
                    if (ImGui::Button("Copiar valores calibrados",
                            { -1.0f, ui(36.0f) })) {
                        char calibration[384] {};
                        std::snprintf(calibration, sizeof(calibration),
                            "ToneMapping: exposure=%.4ff; brightness=%.4ff; "
                            "contrast=%.4ff; saturation=%.4ff\n"
                            "Fog: density=%.4ff; heightFalloff=%.4ff; "
                            "maxOpacity=%.4ff; color={%.3ff, %.3ff, %.3ff}",
                            m_laboratoryToneMapping.exposure,
                            m_laboratoryToneMapping.brightness,
                            m_laboratoryToneMapping.contrast,
                            m_laboratoryToneMapping.saturation,
                            m_laboratoryFog.density,
                            m_laboratoryFog.heightFalloff,
                            m_laboratoryFog.maxOpacity,
                            m_laboratoryFog.color.x,
                            m_laboratoryFog.color.y,
                            m_laboratoryFog.color.z);
                        ImGui::SetClipboardText(calibration);
                        m_toneMappingCopyStatus = "Valores copiados";
                    }
                    if (!m_toneMappingCopyStatus.empty()) {
                        ImGui::TextColored({ 0.38f, 0.82f, 0.62f, 1.0f },
                            "%s", m_toneMappingCopyStatus.c_str());
                    }
            }
            growPanelToFitContent(io.DisplaySize, m_uiScale);
        }
        if (m_showGraphicsPanel) ImGui::End();

        if (m_showPerformancePanel && beginDebugPanel("Performance",
                &m_showPerformancePanel,
                { io.DisplaySize.x - panelWidth - ui(12.0f),
                    panelTop + ui(48.0f) },
                { panelWidth, ui(300.0f) }, io.DisplaySize, m_uiScale)) {
            {
                    const ApplicationFrameMetrics& application =
                        applicationFrameMetrics();
                    const RHI::FramePerformanceMetrics graphics =
                        renderer().framePerformanceMetrics();
                    panelHeader("FRAME");
                    ImGui::Text("Total     %.3f ms",
                        application.totalMilliseconds);
                    ImGui::Text("Update    %.3f ms",
                        application.fixedUpdateMilliseconds);
                    ImGui::Text("Render    %.3f ms",
                        application.renderMilliseconds);
                    ImGui::Text("UI        %.3f ms",
                        application.guiMilliseconds);
                    ImGui::Text("GPU       %.3f ms",
                        graphics.gpuTimingValid
                            ? graphics.gpuFrameMilliseconds : 0.0f);
                    ImGui::Text("Fence     %.3f ms",
                        graphics.cpuFenceWaitMilliseconds);
                    ImGui::Text("Acquire   %.3f ms",
                        graphics.cpuAcquireMilliseconds);
                    ImGui::Text("Present   %.3f ms",
                        graphics.cpuPresentMilliseconds);
                    if (m_physicsScene) {
                        const PhysicsStepDiagnostics3D& physics =
                            m_physicsScene->diagnostics();
                        ImGui::Dummy({ 1.0f, ui(10.0f) });
                        panelHeader("PHYSX 120 HZ");
                        ImGui::Text("Passo     %.3f ms",
                            physics.totalStepMilliseconds);
                        ImGui::Text("Espera    %.3f ms",
                            physics.simulationWaitMilliseconds);
                        ImGui::Text("Workers   %u",
                            physics.physicsWorkerCount);
                        ImGui::Text("Ativos    %zu",
                            physics.activeDynamicBodyCount);
                        ImGui::Text("Dormindo  %zu",
                            physics.sleepingDynamicBodyCount);
                        ImGui::Text("Contatos  %zu",
                            physics.discreteContactPairs);
                        ImGui::Text("CCD       %zu", physics.ccdPairs);
                    }
            }
            growPanelToFitContent(io.DisplaySize, m_uiScale);
        }
        if (m_showPerformancePanel) ImGui::End();
    }

    if (m_spawnMenuOpen) {
        drawSpawnMenu();
    }
    if (m_laboratoryPaused) {
        drawLaboratoryPauseMenu();
    }
}

} // namespace MatterEngine::Workbench
