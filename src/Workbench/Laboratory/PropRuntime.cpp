#include "Workbench/WorkbenchApp.hpp"

#include "Engine/Core/Log.hpp"
#include "Engine/Geometry/MeshData3D.hpp"
#include "Engine/Math/Mat4.hpp"
#include "Engine/RHI/RHITypes.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <span>
#include <string>
#include <vector>

namespace MatterEngine::Workbench {
namespace {

constexpr float Pi = 3.14159265358979323846f;
constexpr float MaximumSpawnDistanceMeters = 18.0f;
constexpr float EmptySpaceSpawnDistanceMeters = 7.0f;
constexpr float MaximumPhysGunQueryDistanceMeters = 1000.0f;
// Pequena margem antes da superficie estatica atingida pelo raio central.
// Impede a assistencia esferica de selecionar atraves de uma parede sem
// prejudicar props apoiados no piso, cuja superficie fica antes dele.
constexpr float PhysGunOcclusionMarginMeters = 0.005f;
// Props dinamicos usam decomposicao convexa (V-HACD); o casco de colisao
// pode ficar visivelmente menor que a malha visual em partes finas (perna de
// cadeira, ripa, banda de pneu). Um raio infinitamente fino erra nessas
// bordas mesmo mirando no centro visual do objeto. Esse raio "gordo" e o
// fallback: so entra em jogo quando o raio exato ja errou.
constexpr float PhysGunAssistSweepRadiusMeters = 0.06f;
constexpr std::size_t MaximumSpawnedProps = 2048;

Vec3 viewForward(float yaw, float pitch) {
    const float planar = std::cos(pitch);
    return { planar * std::cos(yaw), planar * std::sin(yaw),
        std::sin(pitch) };
}

Quaternion previewOrientation() {
    return Quaternion::fromAxisAngle({ 0.0f, 0.0f, 1.0f }, -0.38f);
}

Quaternion viewOrientation(float yaw, float pitch) {
    return (Quaternion::fromAxisAngle({ 0.0f, 0.0f, 1.0f }, yaw)
        * Quaternion::fromAxisAngle({ 0.0f, 1.0f, 0.0f }, -pitch))
        .normalized();
}

float supportDistance(Vec3 dimensions, Vec3 direction) {
    const Vec3 halfExtents = dimensions * 0.5f;
    direction = direction.normalized();
    return std::abs(direction.x) * halfExtents.x
        + std::abs(direction.y) * halfExtents.y
        + std::abs(direction.z) * halfExtents.z;
}

// Piso do Object Viewer: 10x10 m, celulas de 1 m, mesmas cores do antigo
// chao procedural analitico (scene3d.frag) - agora assadas numa textura em
// vez de recalculadas por pixel a cada quadro. Assar em vez de manter o
// desenho analitico converge 100% do conteudo visual real para o pipeline
// de mesh/material (relevante para a reforma de PBR planejada a seguir).
constexpr float ObjectViewerFloorHalfExtentMeters = 5.0f;
constexpr int ObjectViewerFloorTileCount = 10;
constexpr int ObjectViewerFloorTexelsPerTile = 8;

std::vector<std::byte> buildFloorCheckerboardPixels() {
    constexpr int size = ObjectViewerFloorTileCount * ObjectViewerFloorTexelsPerTile;
    constexpr std::array<std::uint8_t, 3> lightColor { 194, 201, 207 };
    constexpr std::array<std::uint8_t, 3> darkColor { 102, 110, 115 };
    std::vector<std::byte> pixels(
        static_cast<std::size_t>(size) * static_cast<std::size_t>(size) * 4);
    for (int y = 0; y < size; ++y) {
        const int tileY = y / ObjectViewerFloorTexelsPerTile;
        for (int x = 0; x < size; ++x) {
            const int tileX = x / ObjectViewerFloorTexelsPerTile;
            const bool light = ((tileX + tileY) & 1) == 0;
            const std::array<std::uint8_t, 3>& color =
                light ? lightColor : darkColor;
            const std::size_t offset =
                (static_cast<std::size_t>(y) * static_cast<std::size_t>(size)
                    + static_cast<std::size_t>(x)) * 4;
            pixels[offset + 0] = static_cast<std::byte>(color[0]);
            pixels[offset + 1] = static_cast<std::byte>(color[1]);
            pixels[offset + 2] = static_cast<std::byte>(color[2]);
            pixels[offset + 3] = static_cast<std::byte>(255);
        }
    }
    return pixels;
}

// Mesma ordem de vertices/indices do antigo quad procedural (groundPositions
// em scene3d.vert), so que como uma malha real com UV 0..1 cobrindo o
// quadrado inteiro em vez de calculado a partir de worldPosition.
MeshData3D buildFloorQuadMesh() {
    const float half = ObjectViewerFloorHalfExtentMeters;
    const Vec3 up { 0.0f, 0.0f, 1.0f };
    MeshData3D mesh;
    mesh.vertices = {
        { { -half, -half, 0.0f }, up, { 0.0f, 0.0f } },
        { { half, -half, 0.0f }, up, { 1.0f, 0.0f } },
        { { half, half, 0.0f }, up, { 1.0f, 1.0f } },
        { { -half, half, 0.0f }, up, { 0.0f, 1.0f } }
    };
    mesh.indices = { 0, 2, 3, 0, 1, 2 };
    recomputeBounds(mesh);
    return mesh;
}

} // namespace

void WorkbenchApp::ensurePropAssetsLoaded(Renderer& activeRenderer) {
    if (m_propCatalog.loaded() || m_propAssetsLoadFailed
        || !m_physicsScene) return;
    try {
        m_propCatalog.load(activeRenderer, m_materialLibrary,
            m_physicsEngine, MATTERENGINE_ASSETS_DIR);
        m_physGunVisual = loadGpuModel3D(activeRenderer,
            std::string(MATTERENGINE_ASSETS_DIR)
                + "/models/weapons/phys_gun.glb");
        m_physGunVisualLoaded = true;
    } catch (const std::exception& error) {
        Log::error(std::string("Falha ao carregar catalogo de props: ")
            + error.what());
        m_propCatalog.release(activeRenderer);
        releaseGpuModel3D(activeRenderer, m_physGunVisual);
        m_physGunVisualLoaded = false;
        m_propAssetsLoadFailed = true;
        m_laboratoryStatus = "Falha ao carregar objetos";
    }
}

void WorkbenchApp::spawnProp(std::size_t definitionIndex) {
    const auto& definitions = m_propCatalog.definitions();
    if (!m_physicsScene || definitionIndex >= definitions.size()
        || !m_laboratoryMapLoaded) {
        m_laboratoryStatus = "Objeto indisponivel";
        return;
    }
    if (m_spawnedProps.size() >= MaximumSpawnedProps) {
        m_laboratoryStatus = "Limite de objetos atingido";
        return;
    }

    const PropDefinition3D& definition = definitions[definitionIndex];
    PhysicsBodyDefinition3D body = definition.bodyTemplate;
    body.entityId = m_nextEntityId++;
    body.orientation = {};
    body.linearVelocity = {};
    body.angularVelocity = {};
    body.startAwake = true;

    const Vec3 direction = viewForward(m_laboratoryCameraYaw,
        m_laboratoryCameraPitch).normalized();
    const Ray3D ray { m_laboratoryCameraPosition, direction };
    PhysicsRayHit3D hit;
    if (m_physicsScene->raycastStatic(ray,
            MaximumSpawnDistanceMeters, hit)) {
        body.position = hit.position + hit.normal
            * (supportDistance(definition.dimensionsMeters, hit.normal)
                + 0.035f);
    } else {
        body.position = ray.origin
            + ray.direction * EmptySpaceSpawnDistanceMeters;
    }

    const PhysicsBodyHandle3D physicsBody = m_physicsScene->createBody(
        body, definition.collisionShapes);
    PhysicsBodyState3D initialState;
    initialState.position = body.position;
    initialState.orientation = body.orientation;
    initialState.linearVelocity = body.linearVelocity;
    initialState.angularVelocity = body.angularVelocity;
    m_spawnedProps.push_back({ body.entityId, definitionIndex,
        physicsBody, initialState, initialState, 0.0f });
    m_spawnedPropByBodyIndex.insert_or_assign(physicsBody.index,
        m_spawnedProps.size() - 1);
    m_laboratoryStatus = definition.displayName + " criado";
}

void WorkbenchApp::removeLatestSpawnedEntity() {
    if (m_spawnedProps.empty()) {
        m_laboratoryStatus.clear();
        m_notification = { "Nenhum objeto para remover", 1.8f, 1.8f };
        return;
    }
    const SpawnedPropInstance removed = m_spawnedProps.back();
    if (m_physGunGrabbedEntityId == removed.entityId) endPhysGunGrab();
    if (m_physicsScene) m_physicsScene->destroyBody(removed.physicsBody);
    m_spawnedPropByBodyIndex.erase(removed.physicsBody.index);
    m_spawnedProps.pop_back();
    m_laboratoryStatus.clear();
    m_notification = { "Objeto removido", 2.35f, 2.35f };
}

void WorkbenchApp::beginPhysGunGrab() {
    if (!m_physicsScene || m_spawnedProps.empty()) return;
    const Ray3D ray { m_laboratoryCameraPosition,
        viewForward(m_laboratoryCameraYaw,
            m_laboratoryCameraPitch).normalized() };
    // Primeiro determina ate onde a mira possui linha de visao. A selecao em
    // si consulta apenas corpos dinamicos; assim o piso nao vence a sweep de
    // assistencia, mas uma parede continua bloqueando props atras dela.
    PhysicsRayHit3D staticBlocker;
    float visibleDistance = MaximumPhysGunQueryDistanceMeters;
    if (m_physicsScene->raycastStatic(ray,
            MaximumPhysGunQueryDistanceMeters, staticBlocker)) {
        visibleDistance = std::max(0.0f,
            staticBlocker.distance - PhysGunOcclusionMarginMeters);
    }

    PhysicsRayHit3D hit;
    const bool exactHit = visibleDistance > 0.0f
        && m_physicsScene->raycastDynamic(ray, visibleDistance, hit)
        && hit.body;
    if (!exactHit && (visibleDistance <= 0.0f
        || !m_physicsScene->sweepSphereDynamic(ray,
            PhysGunAssistSweepRadiusMeters, visibleDistance, hit)
        || !hit.body)) {
        return;
    }
    const auto instance = std::find_if(m_spawnedProps.begin(),
        m_spawnedProps.end(), [&](const SpawnedPropInstance& candidate) {
            return candidate.physicsBody == hit.body;
        });
    if (instance == m_spawnedProps.end()) return;
    if (m_physicsScene->bodyFrozen(hit.body)) {
        m_physicsScene->setBodyFrozen(hit.body, false);
    }

    const PhysicsBodyState3D state = m_physicsScene->bodyState(hit.body);
    const Vec3 localHit = state.orientation.conjugate().rotate(
        hit.position - state.position);
    const bool fixedPose = m_physGunHoldMode == PhysGunHoldMode::FixedPose;
    m_physGunLocalGrabPoint = fixedPose ? Vec3 {} : localHit;
    m_physGunTargetOrientation = state.orientation;
    m_physGunRotationStartOrientation = state.orientation;
    const Quaternion cameraOrientation = viewOrientation(
        m_laboratoryCameraYaw, m_laboratoryCameraPitch);
    m_physGunRelativeOrientation = (cameraOrientation.conjugate()
        * state.orientation).normalized();
    m_physGunFixedCenterCameraLocal = cameraOrientation.conjugate().rotate(
        state.position - m_laboratoryCameraPosition);
    m_physGunRotationInputDegrees = {};
    m_physGunOrientationLocked = fixedPose;
    m_physGunHoldDistance = std::max(0.65f, hit.distance);

    PhysicsGrabTarget3D target;
    target.position = fixedPose ? state.position : hit.position;
    target.orientation = state.orientation;
    target.lockOrientation = fixedPose;
    if (!m_physicsScene->beginGrab(hit.body, m_physGunLocalGrabPoint,
            target, m_physGunSettings)) {
        return;
    }
    m_physGunBeamTargetWorldPosition = hit.position;
    m_physGunGrabbedEntityId = instance->entityId;
}

void WorkbenchApp::endPhysGunGrab() {
    if (m_physicsScene) m_physicsScene->endGrab();
    m_physGunGrabbedEntityId = 0;
    m_physGunLocalGrabPoint = {};
    m_physGunOrientationLocked = false;
    m_physGunTriggerHeld = false;
}

void WorkbenchApp::freezePhysGunObject() {
    if (!m_physicsScene || !m_physicsScene->grabbing()) {
        endPhysGunGrab();
        return;
    }
    const PhysicsBodyHandle3D body = m_physicsScene->grabbedBody();
    const auto instance = std::find_if(m_spawnedProps.begin(),
        m_spawnedProps.end(), [&](const SpawnedPropInstance& candidate) {
            return candidate.physicsBody == body;
        });
    if (instance == m_spawnedProps.end()) {
        endPhysGunGrab();
        return;
    }
    m_physicsScene->endGrab();
    m_physicsScene->setBodyFrozen(body, true);
    instance->freezeFlashSeconds = 0.34f;
    m_notification = { "Objeto congelado", 1.55f, 1.55f };
    endPhysGunGrab();
}

void WorkbenchApp::unfreezeLookedAtObject() {
    if (!m_physicsScene) return;
    const Ray3D ray { m_laboratoryCameraPosition,
        viewForward(m_laboratoryCameraYaw,
            m_laboratoryCameraPitch).normalized() };
    PhysicsRayHit3D hit;
    if (!m_physicsScene->raycast(ray,
            MaximumPhysGunQueryDistanceMeters, hit)
        || !hit.body || !m_physicsScene->bodyFrozen(hit.body)) {
        return;
    }
    m_physicsScene->setBodyFrozen(hit.body, false);
    m_notification = { "Objeto descongelado", 1.55f, 1.55f };
}

void WorkbenchApp::unfreezeAllObjects() {
    if (!m_physicsScene) return;
    std::size_t count = 0;
    for (const SpawnedPropInstance& instance : m_spawnedProps) {
        if (!m_physicsScene->bodyFrozen(instance.physicsBody)) continue;
        m_physicsScene->setBodyFrozen(instance.physicsBody, false);
        ++count;
    }
    if (count > 0) {
        m_notification = { "Objetos descongelados", 1.7f, 1.7f };
    }
}

void WorkbenchApp::updateDynamicProps(float deltaTime) {
    if (!m_physicsScene) return;
    for (SpawnedPropInstance& instance : m_spawnedProps) {
        instance.freezeFlashSeconds = std::max(0.0f,
            instance.freezeFlashSeconds - deltaTime);
    }
    const Vec3 forward = viewForward(m_laboratoryCameraYaw,
        m_laboratoryCameraPitch).normalized();
    if (m_physGunGrabbedEntityId == 0 && m_physGunTriggerHeld) {
        beginPhysGunGrab();
    }
    if (m_physGunGrabbedEntityId != 0) {
        const auto instance = std::find_if(m_spawnedProps.begin(),
            m_spawnedProps.end(), [&](const SpawnedPropInstance& candidate) {
                return candidate.entityId == m_physGunGrabbedEntityId;
            });
        if (instance == m_spawnedProps.end()
            || !m_physicsScene->contains(instance->physicsBody)) {
            endPhysGunGrab();
        } else {
            const Quaternion cameraOrientation = viewOrientation(
                m_laboratoryCameraYaw, m_laboratoryCameraPitch);
            const bool fixedPose =
                m_physGunHoldMode == PhysGunHoldMode::FixedPose;
            PhysicsGrabTarget3D target;
            target.position = fixedPose
                ? m_laboratoryCameraPosition
                    + cameraOrientation.rotate(m_physGunFixedCenterCameraLocal)
                : m_laboratoryCameraPosition
                    + forward * m_physGunHoldDistance;
            if (fixedPose) {
                m_physGunTargetOrientation = (cameraOrientation
                    * m_physGunRelativeOrientation).normalized();
            }
            target.orientation = m_physGunTargetOrientation;
            target.lockOrientation = fixedPose
                || m_physGunOrientationLocked || input().keyDown(Key::E);

            PhysicsHandleSettings3D settings = m_physGunSettings;
            if (target.lockOrientation) {
                const float response = std::clamp(
                    m_physGunRotationSmoothness, 0.0f, 1.0f);
                settings.angularStiffness =
                    80.0f + response * response * 760.0f;
                settings.angularDampingRatio = 0.72f + response * 0.38f;
            }
            m_physicsScene->updateGrabTarget(target, settings);
            const PhysicsBodyState3D state =
                m_physicsScene->bodyState(instance->physicsBody);
            m_physGunBeamTargetWorldPosition = state.position
                + state.orientation.rotate(m_physGunLocalGrabPoint);
        }
    }

    // Vento (ver WindSystem): um unico avanco de relogio por quadro, depois
    // amostrado em duas alturas diferentes - a do jogador (fisica/audio) e
    // uma de referencia bem mais alta (nuvens, ver mais abaixo) - mesma
    // direcao/rajada em ambas, so a magnitude muda com a altura.
    m_windSystem.advance(deltaTime);
    const float playerHeightMeters = m_physicsScene->hasCharacter()
        ? m_physicsScene->characterState().position.z
        : m_laboratoryCameraPosition.z;
    const Vec3 windAtPlayerHeight =
        m_windSystem.velocityAtHeight(playerHeightMeters);
    // So altera o vetor de vento da cena; a formula de arrasto quadratico
    // em si ja existia (ver PhysXScene3D::simulate) e nunca precisou mudar.
    m_physicsScene->setAirVelocity(windAtPlayerHeight);

    // Altitude tipica de base de nuvens cumulus - bem mais alta que
    // qualquer ponto alcancavel no Laboratorio, entao as nuvens sempre leem
    // o vento "de cima", acima do marco de forca plena (ver
    // WindSettings3D::referenceHeightMeters).
    constexpr float CloudReferenceAltitudeMeters = 600.0f;
    const Vec3 windAtCloudAltitude =
        m_windSystem.velocityAtHeight(CloudReferenceAltitudeMeters);
    m_cloudWindOffset += Vec2 { windAtCloudAltitude.x, windAtCloudAltitude.y }
        * m_cloudWindVisualScale * deltaTime;

    m_physicsScene->simulate(deltaTime);
    for (const PhysicsBodyStateUpdate3D& update :
        m_physicsScene->activeBodyStates()) {
        const auto found = m_spawnedPropByBodyIndex.find(update.body.index);
        if (found == m_spawnedPropByBodyIndex.end()
            || found->second >= m_spawnedProps.size()) {
            continue;
        }
        SpawnedPropInstance& instance = m_spawnedProps[found->second];
        if (instance.physicsBody == update.body) {
            // previousPhysicsState pertence ao ultimo frame efetivamente
            // renderizado e e avancado em renderLaboratory3D. O fixed update
            // pode executar zero, uma ou varias vezes antes de um render;
            // altera-lo aqui faria o motion vector representar apenas o
            // ultimo subpasso fisico, nao todo o deslocamento visivel.
            instance.physicsState = update.state;
        }
    }
    m_impactAcousticResolver.resolve(m_physicsScene->contactImpacts(),
        m_materialLibrary, deltaTime);
    m_worldAudio.submitImpacts(m_impactAcousticResolver.commands());

    // Pitch completo (nao so o yaw planar) e o que da ao HRTF a pista de
    // elevacao: e o motivo pratico de ligar HRTF, entao a pose enviada ao
    // AudioDevice3D precisa refletir para onde a camera realmente aponta.
    AudioListenerPose3D listenerPose;
    listenerPose.position = m_laboratoryCameraPosition;
    listenerPose.forward = viewForward(m_laboratoryCameraYaw,
        m_laboratoryCameraPitch).normalized();
    m_worldAudio.update(listenerPose, m_physicsScene.get(), deltaTime);

    const Vec3 characterVelocityForAudio = m_physicsScene->hasCharacter()
        ? m_physicsScene->characterState().velocity : Vec3 {};
    m_worldAudio.updateWindAmbience(windAtPlayerHeight,
        characterVelocityForAudio, m_windAudioVolume, deltaTime);
}

UiTexture WorkbenchApp::renderPropPreviewAtlas(Renderer& activeRenderer) {
    const auto& definitions = m_propCatalog.definitions();
    if (definitions.empty()) return {};
    std::vector<MeshRender3D> meshes;
    for (std::size_t index = 0; index < definitions.size(); ++index) {
        const std::size_t column = index % 3;
        const std::size_t row = index / 3;
        const Vec3 dimensions = definitions[index].dimensionsMeters;
        const float largest = std::max({ dimensions.x, dimensions.y,
            dimensions.z, 0.001f });
        const Vec3 position {
            (static_cast<float>(column) - 1.0f) * 2.25f, 0.0f,
            (0.5f - static_cast<float>(row)) * 2.15f
        };
        // Camera estatica, objeto parado - previousPosition/Orientation
        // repetem os mesmos valores (motion vector zero de proposito).
        appendGpuModelRenderables(definitions[index].visual, position,
            previewOrientation(), position, previewOrientation(),
            1.28f / largest, false, false, true, meshes);
    }
    const Mat4 view = Mat4::lookAt({ 0.0f, -8.6f, 0.15f }, {},
        { 0.0f, 0.0f, 1.0f });
    const Mat4 projection = Mat4::perspective(
        35.0f * Pi / 180.0f, 1.5f, 0.05f, 40.0f);
    std::array<LightRender3D, 1> lights;
    lights[0].type = LightType3D::Point;
    lights[0].position = { -3.0f, -4.5f, 7.0f };
    lights[0].intensity = 1.10f;
    // castsShadow so importa de verdade com showShadows=true (nao e o caso
    // aqui), mantido true so pra bater com o comportamento antigo (o unico
    // chamador de shadowVisibility nao verificava tipo/flag nenhum).
    lights[0].castsShadow = true;

    Scene3DFrame scene;
    scene.cameraViewProjection = projection * view;
    // Sem TAA nesta preview (camera estatica, sem jitter) - a matriz
    // "jitterada" que a GPU usa e so a mesma, e a reprojecao vira uma
    // identidade (quadro anterior == atual).
    scene.cameraViewProjectionJittered = scene.cameraViewProjection;
    scene.previousCameraViewProjection = scene.cameraViewProjection;
    scene.cameraPosition = { 0.0f, -8.6f, 0.15f };
    scene.lights = lights;
    scene.meshes = meshes;
    scene.ambientLight = 0.38f;
    scene.showSky = false;
    scene.showShadows = false;
    return activeRenderer.renderScene3D(scene, 768, 512);
}

void WorkbenchApp::ensureObjectViewerFloorLoaded(Renderer& activeRenderer) {
    if (m_objectViewerFloorLoaded) return;
    const std::vector<std::byte> pixels = buildFloorCheckerboardPixels();
    const MeshData3D mesh = buildFloorQuadMesh();
    constexpr std::uint32_t textureSize = static_cast<std::uint32_t>(
        ObjectViewerFloorTileCount * ObjectViewerFloorTexelsPerTile);

    GpuModelPart3D floorPart;
    floorPart.albedoTexture = activeRenderer.createTexture2D(
        { textureSize, textureSize }, pixels);
    floorPart.vertexBuffer = activeRenderer.createBuffer({
        mesh.vertices.size() * sizeof(MeshVertex3D),
        RHI::BufferUsage::Vertex, true, "Object viewer floor vertices" });
    floorPart.indexBuffer = activeRenderer.createBuffer({
        mesh.indices.size() * sizeof(std::uint32_t),
        RHI::BufferUsage::Index, true, "Object viewer floor indices" });
    activeRenderer.writeBuffer(floorPart.vertexBuffer, 0,
        std::as_bytes(std::span(mesh.vertices)));
    activeRenderer.writeBuffer(floorPart.indexBuffer, 0,
        std::as_bytes(std::span(mesh.indices)));
    floorPart.indexCount = static_cast<std::uint32_t>(mesh.indices.size());

    m_objectViewerFloor.parts = { floorPart };
    m_objectViewerFloor.textures = { floorPart.albedoTexture };
    m_objectViewerFloor.dimensionsMeters = {
        ObjectViewerFloorHalfExtentMeters * 2.0f,
        ObjectViewerFloorHalfExtentMeters * 2.0f, 0.0f
    };
    m_objectViewerFloorLoaded = true;
}

UiTexture WorkbenchApp::renderObjectViewerPreview(Renderer& activeRenderer,
    std::size_t definitionIndex) {
    const auto& definitions = m_propCatalog.definitions();
    if (definitionIndex >= definitions.size()) return {};
    ensureObjectViewerFloorLoaded(activeRenderer);
    const PropDefinition3D& definition = definitions[definitionIndex];
    const float largest = std::max({ definition.dimensionsMeters.x,
        definition.dimensionsMeters.y, definition.dimensionsMeters.z,
        0.001f });
    std::vector<MeshRender3D> meshes;
    // Camera estatica, objetos parados - previousPosition/Orientation
    // repetem os mesmos valores (motion vector zero de proposito).
    appendGpuModelRenderables(definition.visual, {}, previewOrientation(),
        {}, previewOrientation(), 2.4f / largest, false, false, true,
        meshes);
    // Chao real (ver ensureObjectViewerFloorLoaded) na mesma altura do
    // antigo plano procedural (groundPlaneZ=-1.5).
    appendGpuModelRenderables(m_objectViewerFloor,
        { 0.0f, 0.0f, -1.5f }, {}, { 0.0f, 0.0f, -1.5f }, {}, 1.0f, false,
        false, true, meshes);
    const Mat4 view = Mat4::lookAt({ 0.0f, -6.3f, 1.2f }, {},
        { 0.0f, 0.0f, 1.0f });
    const Mat4 projection = Mat4::perspective(
        39.0f * Pi / 180.0f, 1.30f, 0.05f, 30.0f);
    std::array<LightRender3D, 1> lights;
    lights[0].type = LightType3D::Point;
    lights[0].position = { -3.5f, -3.8f, 6.5f };
    lights[0].intensity = 1.14f;
    lights[0].castsShadow = true;

    Scene3DFrame scene;
    scene.cameraViewProjection = projection * view;
    // Sem TAA nesta preview - ver comentario equivalente em
    // renderPropPreviewAtlas.
    scene.cameraViewProjectionJittered = scene.cameraViewProjection;
    scene.previousCameraViewProjection = scene.cameraViewProjection;
    scene.cameraPosition = { 0.0f, -6.3f, 1.2f };
    scene.lights = lights;
    scene.meshes = meshes;
    scene.ambientLight = 0.35f;
    scene.showSky = false;
    scene.showShadows = true;
    return activeRenderer.renderScene3D(scene, 900, 700);
}

} // namespace MatterEngine::Workbench
