#include "Workbench/WorkbenchApp.hpp"

#include "Engine/Core/Log.hpp"
#include "Engine/Geometry/MeshData3D.hpp"
#include "Engine/Math/Mat4.hpp"
#include "Engine/Physics/WindShelter3D.hpp"
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
constexpr float ViewerFloorHalfExtentMeters = 5.0f;
constexpr int ViewerFloorTileCount = 10;
constexpr int ViewerFloorTexelsPerTile = 8;

std::vector<std::byte> buildFloorCheckerboardPixels() {
    constexpr int size = ViewerFloorTileCount * ViewerFloorTexelsPerTile;
    constexpr std::array<std::uint8_t, 3> lightColor { 194, 201, 207 };
    constexpr std::array<std::uint8_t, 3> darkColor { 102, 110, 115 };
    std::vector<std::byte> pixels(
        static_cast<std::size_t>(size) * static_cast<std::size_t>(size) * 4);
    for (int y = 0; y < size; ++y) {
        const int tileY = y / ViewerFloorTexelsPerTile;
        for (int x = 0; x < size; ++x) {
            const int tileX = x / ViewerFloorTexelsPerTile;
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
    const float half = ViewerFloorHalfExtentMeters;
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
        m_propCatalog.load(activeRenderer, m_materialLibrary, MATTERENGINE_ASSETS_DIR);
        m_physGunVisual = loadGpuModel3D(activeRenderer,
            std::string(MATTERENGINE_ASSETS_DIR)
                + "/models/weapons/phys_gun.glb");
        m_physGunVisualLoaded = true;
        ensureSoccerBallVisual(activeRenderer);
    } catch (const std::exception& error) {
        Log::error(std::string("Falha ao carregar catalogo de props: ")
            + error.what());
        m_propCatalog.release(activeRenderer);
        releaseGpuModel3D(activeRenderer, m_physGunVisual);
        releaseGpuModel3D(activeRenderer, m_soccerBallVisual);
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
    // A bola de futebol do menu e a bola inflada (fisica e malha proprias).
    const bool inflatedBall = definition.inflatedBall;
    // A inflada nasce no tamanho escolhido no menu da bola, nao no do GLB
    // (maior, nascia na altura da oficial e a checagem dela via o chao).
    const Vec3 dimensions = inflatedBall
        ? Vec3 { 1.0f, 1.0f, 1.0f } * (2.0f * m_soccerBallProfile.radiusMeters)
        : definition.dimensionsMeters;

    const Vec3 direction = viewForward(m_laboratoryCameraYaw,
        m_laboratoryCameraPitch).normalized();
    const Ray3D ray { m_laboratoryCameraPosition, direction };
    PhysicsRayHit3D hit;
    Vec3 requestedPosition;
    if (m_physicsScene->raycastStatic(ray,
            MaximumSpawnDistanceMeters, hit)) {
        requestedPosition = hit.position + hit.normal
            * (supportDistance(dimensions, hit.normal)
                + 0.035f);
    } else {
        requestedPosition = ray.origin
            + ray.direction * EmptySpaceSpawnDistanceMeters;
    }

    const Quaternion orientation {};
    const std::optional<Vec3> safePosition = findSafeSpawnPosition(
        requestedPosition, dimensions * 0.5f,
        orientation, 4.0f);
    const bool spawned = safePosition
        && (inflatedBall ? spawnInflatedBallAt(*safePosition)
                         : spawnPropAt(definitionIndex, *safePosition,
                               orientation, false));
    if (!spawned) {
        m_laboratoryStatus = "Sem espaco seguro para criar o objeto";
        m_notification = {
            "Spawn bloqueado: procure uma area livre", 2.2f, 2.2f
        };
    }
}

bool WorkbenchApp::spawnPropAt(std::size_t definitionIndex,
    Vec3 position, Quaternion orientation, bool benchmarkEntity) {
    const auto& definitions = m_propCatalog.definitions();
    if (!m_physicsScene || definitionIndex >= definitions.size()
        || !m_laboratoryMapLoaded
        || m_spawnedProps.size() >= MaximumSpawnedProps) {
        return false;
    }
    const PropDefinition3D& definition = definitions[definitionIndex];
    // (A bola inflada tem a fisica dela.)
    if (definition.inflatedBall) return spawnInflatedBallAt(position);
    // Última barreira imediatamente antes de publicar o ator. O benchmark
    // também passa por aqui; assim uma posição que ficou ocupada entre a
    // escolha e a criação nunca produz duas entidades interpenetradas.
    constexpr float SpawnClearanceMeters = 0.025f;
    if (m_physicsScene->overlapsBox(position,
            definition.dimensionsMeters * 0.5f
                + Vec3 { SpawnClearanceMeters, SpawnClearanceMeters,
                    SpawnClearanceMeters },
            orientation)) {
        return false;
    }

    PhysicsBodyDefinition3D body = definition.bodyTemplate;
    body.entityId = m_nextEntityId++;
    body.position = position;
    body.orientation = orientation.normalized();
    body.linearVelocity = {};
    body.angularVelocity = {};
    body.startAwake = true;
    const PhysicsBodyHandle3D physicsBody = m_physicsScene->createBody(
        body, definition.collisionShapes);
    PhysicsBodyState3D initialState;
    initialState.position = body.position;
    initialState.orientation = body.orientation;
    initialState.linearVelocity = body.linearVelocity;
    initialState.angularVelocity = body.angularVelocity;
    // Campos nomeados: a inicializacao posicional quebrava em silencio a
    // cada campo novo no meio do struct.
    m_spawnedProps.push_back({
        .entityId = body.entityId,
        .definitionIndex = definitionIndex,
        .physicsBody = physicsBody,
        .physicsState = initialState,
        .simulationPreviousState = initialState,
        .renderedState = initialState,
        .previousPhysicsState = initialState,
    });
    m_spawnedPropByBodyIndex.insert_or_assign(physicsBody.index,
        m_spawnedProps.size() - 1);
    if (benchmarkEntity) {
        m_physicsBenchmarkPropEntityIds.push_back(body.entityId);
    }
    m_laboratoryStatus = definition.displayName + " criado";
    return true;
}

std::optional<Vec3> WorkbenchApp::findSafeSpawnPosition(
    Vec3 requestedPosition, Vec3 halfExtents,
    Quaternion orientation, float maximumSearchRadiusMeters) const {
    if (!m_physicsScene) return std::nullopt;
    constexpr float ClearanceMeters = 0.025f;
    const Vec3 queryHalfExtents = halfExtents
        + Vec3 { ClearanceMeters, ClearanceMeters, ClearanceMeters };
    if (!m_physicsScene->overlapsBox(requestedPosition,
            queryHalfExtents, orientation)) {
        return requestedPosition;
    }

    // Busca determinística em anéis de Fibonacci. Ela preenche cada raio
    // uniformemente, testa primeiro o mesmo nível e só então pequenos
    // degraus verticais. Não há "teleporte aleatório" imprevisível no menu.
    constexpr float GoldenAngle = 2.39996322973f;
    constexpr float RadialStepMeters = 0.32f;
    const int samples = std::max(1, static_cast<int>(std::ceil(
        maximumSearchRadiusMeters / RadialStepMeters)));
    for (int ring = 1; ring <= samples; ++ring) {
        const float radius = std::min(maximumSearchRadiusMeters,
            static_cast<float>(ring) * RadialStepMeters);
        const int points = std::max(8,
            static_cast<int>(std::ceil(2.0f * Pi * radius
                / RadialStepMeters)));
        for (int point = 0; point < points; ++point) {
            const float angle =
                static_cast<float>(point) * GoldenAngle;
            for (int level = 0; level < 3; ++level) {
                const Vec3 candidate = requestedPosition + Vec3 {
                    std::cos(angle) * radius,
                    std::sin(angle) * radius,
                    static_cast<float>(level) * 0.35f
                };
                if (!m_physicsScene->overlapsBox(candidate,
                        queryHalfExtents, orientation)) {
                    return candidate;
                }
            }
        }
    }
    return std::nullopt;
}

void WorkbenchApp::removeLatestSpawnedEntity() {
    if (m_spawnedProps.empty() && m_spawnedRagdolls.empty()
        && m_inflatedBalls.empty()) {
        m_laboratoryStatus.clear();
        m_notification = { "Nenhum objeto para remover", 1.8f, 1.8f };
        return;
    }
    const std::uint64_t latestProp =
        m_spawnedProps.empty() ? 0 : m_spawnedProps.back().entityId;
    const std::uint64_t latestRagdoll =
        m_spawnedRagdolls.empty() ? 0 : m_spawnedRagdolls.back().entityId;
    const std::uint64_t latestBall =
        m_inflatedBalls.empty() ? 0 : m_inflatedBalls.back().entityId;
    if (latestBall > latestProp && latestBall > latestRagdoll) {
        if (m_physGunGrabbedEntityId == latestBall) endPhysGunGrab();
        if (m_physicsScene) {
            m_physicsScene->destroyInflatedBall(
                m_inflatedBalls.back().physicsBall);
        }
        m_inflatedBalls.pop_back();
        m_laboratoryStatus.clear();
        m_notification = { "Bola removida", 2.35f, 2.35f };
        return;
    }
    const bool removeRagdoll = latestRagdoll > latestProp;
    if (removeRagdoll) {
        const std::uint64_t removedEntityId =
            m_spawnedRagdolls.back().entityId;
        const RagdollHandle3D removedHandle =
            m_spawnedRagdolls.back().physicsRagdoll;
        if (m_physGunGrabbedEntityId == removedEntityId) endPhysGunGrab();
        if (m_controlledCharacterEntityId == removedEntityId) {
            releaseControlledCharacter();
        }
        if (m_physicsScene) {
            m_physicsScene->destroyRagdoll(removedHandle);
        }
        m_spawnedRagdolls.pop_back();
        const auto benchmark = std::find(
            m_physicsBenchmarkRagdollEntityIds.begin(),
            m_physicsBenchmarkRagdollEntityIds.end(), removedEntityId);
        if (benchmark != m_physicsBenchmarkRagdollEntityIds.end()) {
            m_physicsBenchmarkRagdollEntityIds.erase(benchmark);
            m_physicsBenchmarkSpawnedRagdolls = static_cast<std::uint32_t>(
                m_physicsBenchmarkRagdollEntityIds.size());
        }
        m_laboratoryStatus.clear();
        m_notification = { "Ragdoll removido", 2.35f, 2.35f };
        return;
    }
    const SpawnedPropInstance removed = m_spawnedProps.back();
    if (m_physGunGrabbedEntityId == removed.entityId) endPhysGunGrab();
    if (m_physicsScene) m_physicsScene->destroyBody(removed.physicsBody);
    m_spawnedPropByBodyIndex.erase(removed.physicsBody.index);
    m_spawnedProps.pop_back();
    const auto benchmark = std::find(
        m_physicsBenchmarkPropEntityIds.begin(),
        m_physicsBenchmarkPropEntityIds.end(), removed.entityId);
    if (benchmark != m_physicsBenchmarkPropEntityIds.end()) {
        m_physicsBenchmarkPropEntityIds.erase(benchmark);
        m_physicsBenchmarkSpawnedProps = static_cast<std::uint32_t>(
            m_physicsBenchmarkPropEntityIds.size());
    }
    m_laboratoryStatus.clear();
    m_notification = { "Objeto removido", 2.35f, 2.35f };
}

void WorkbenchApp::beginPhysGunGrab() {
    if (!m_physicsScene
        || (m_spawnedProps.empty() && m_spawnedRagdolls.empty()
            && m_inflatedBalls.empty())) {
        return;
    }
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

    // As bolas infladas nao estao no broad phase: o raio e testado contra a
    // esfera de cada uma (com a mesma folga da assistencia, se preciso).
    float ballDistance = visibleDistance;
    std::size_t ballIndex = m_inflatedBalls.size();
    const auto findBall = [&](float extraRadius) {
        for (std::size_t index = 0; index < m_inflatedBalls.size(); ++index) {
            const Vec3 center = m_inflatedBalls[index].physicsState.position;
            const float radius = m_soccerBallRadius + extraRadius;
            const Vec3 toCenter = center - ray.origin;
            const float along = dot(toCenter, ray.direction);
            const float closestSquared = toCenter.lengthSquared() - along * along;
            if (closestSquared > radius * radius) continue;
            const float distance = along
                - std::sqrt(radius * radius - closestSquared);
            if (distance < 0.0f || distance >= ballDistance) continue;
            ballDistance = distance;
            ballIndex = index;
        }
    };
    if (visibleDistance > 0.0f) findBall(0.0f);

    PhysicsRayHit3D propHit;
    PhysicsRagdollRayHit3D ragdollHit;
    bool hasPropHit = visibleDistance > 0.0f
        && m_physicsScene->raycastDynamic(
            ray, visibleDistance, propHit)
        && propHit.body;
    bool hasRagdollHit = visibleDistance > 0.0f
        && m_physicsScene->raycastRagdoll(
            ray, visibleDistance, ragdollHit)
        && ragdollHit.ragdoll;
    if (!hasPropHit && !hasRagdollHit && ballIndex == m_inflatedBalls.size()
        && visibleDistance > 0.0f) {
        hasPropHit = m_physicsScene->sweepSphereDynamic(ray,
                PhysGunAssistSweepRadiusMeters, visibleDistance, propHit)
            && propHit.body;
        hasRagdollHit = m_physicsScene->sweepSphereRagdoll(ray,
                PhysGunAssistSweepRadiusMeters,
                visibleDistance, ragdollHit)
            && ragdollHit.ragdoll;
        findBall(PhysGunAssistSweepRadiusMeters);
    }
    const bool hasBallHit = ballIndex < m_inflatedBalls.size()
        && (!hasPropHit || ballDistance < propHit.distance)
        && (!hasRagdollHit || ballDistance < ragdollHit.distance);
    if (hasBallHit) {
        // A bola e segurada pelo centro: girar a mira nao a faz pender.
        SpawnedInflatedBallInstance& ball = m_inflatedBalls[ballIndex];
        const InflatedBallState3D ballState =
            m_physicsScene->inflatedBallState(ball.physicsBall);
        if (ballState.frozen) {
            m_physicsScene->setInflatedBallFrozen(ball.physicsBall, false);
        }
        const Quaternion cameraOrientation = viewOrientation(
            m_laboratoryCameraYaw, m_laboratoryCameraPitch);
        m_physGunLocalGrabPoint = {};
        m_physGunTargetOrientation = ballState.orientation;
        m_physGunRotationStartOrientation = ballState.orientation;
        m_physGunRelativeOrientation = (cameraOrientation.conjugate()
            * ballState.orientation).normalized();
        m_physGunFixedCenterCameraLocal = cameraOrientation.conjugate().rotate(
            ballState.position - m_laboratoryCameraPosition);
        m_physGunRotationInputDegrees = {};
        m_physGunOrientationLocked =
            m_physGunHoldMode == PhysGunHoldMode::FixedPose;
        m_physGunHoldDistance = std::max(0.65f,
            (ballState.position - m_laboratoryCameraPosition).length());
        m_physGunBeamTargetWorldPosition = ballState.position;
        m_physGunGrabbedEntityId = ball.entityId;
        m_physGunGrabbedBall = ball.physicsBall;
        return;
    }
    if (!hasPropHit && !hasRagdollHit) {
        return;
    }
    const bool grabRagdoll = hasRagdollHit
        && (!hasPropHit || ragdollHit.distance < propHit.distance);
    PhysicsBodyState3D state;
    Vec3 hitPosition;
    float hitDistance = 0.0f;
    std::uint64_t entityId = 0;
    PhysicsBodyHandle3D propBody;
    RagdollHandle3D ragdoll;
    std::uint32_t ragdollLink = 0;
    if (grabRagdoll) {
        const auto instance = std::find_if(m_spawnedRagdolls.begin(),
            m_spawnedRagdolls.end(),
            [&](const SpawnedRagdollInstance& candidate) {
                return candidate.physicsRagdoll == ragdollHit.ragdoll;
            });
        if (instance == m_spawnedRagdolls.end()) return;
        const RagdollState3D snapshot =
            m_physicsScene->ragdollState(ragdollHit.ragdoll);
        if (ragdollHit.linkIndex >= snapshot.links.size()) return;
        state = snapshot.links[ragdollHit.linkIndex];
        hitPosition = ragdollHit.position;
        hitDistance = ragdollHit.distance;
        entityId = instance->entityId;
        ragdoll = ragdollHit.ragdoll;
        ragdollLink = ragdollHit.linkIndex;
    } else {
        const auto instance = std::find_if(m_spawnedProps.begin(),
            m_spawnedProps.end(),
            [&](const SpawnedPropInstance& candidate) {
                return candidate.physicsBody == propHit.body;
            });
        if (instance == m_spawnedProps.end()) return;
        if (m_physicsScene->bodyFrozen(propHit.body)) {
            m_physicsScene->setBodyFrozen(propHit.body, false);
        }
        state = m_physicsScene->bodyState(propHit.body);
        hitPosition = propHit.position;
        hitDistance = propHit.distance;
        entityId = instance->entityId;
        propBody = propHit.body;
    }

    const Vec3 localHit = state.orientation.conjugate().rotate(
        hitPosition - state.position);
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
    m_physGunHoldDistance = std::max(0.65f, hitDistance);

    PhysicsGrabTarget3D target;
    target.position = fixedPose ? state.position : hitPosition;
    target.orientation = state.orientation;
    target.lockOrientation = fixedPose;
    const bool began = grabRagdoll
        ? m_physicsScene->beginRagdollGrab(ragdoll, ragdollLink,
            m_physGunLocalGrabPoint, target, m_physGunSettings)
        : m_physicsScene->beginGrab(propBody, m_physGunLocalGrabPoint,
            target, m_physGunSettings);
    if (!began) {
        return;
    }
    m_physGunBeamTargetWorldPosition = hitPosition;
    m_physGunGrabbedEntityId = entityId;
}

void WorkbenchApp::endPhysGunGrab() {
    if (m_physicsScene) m_physicsScene->endGrab();
    m_physGunGrabbedEntityId = 0;
    m_physGunGrabbedBall = {};
    m_physGunLocalGrabPoint = {};
    m_physGunOrientationLocked = false;
    m_physGunTriggerHeld = false;
}

void WorkbenchApp::freezePhysGunObject() {
    if (m_physicsScene && m_physGunGrabbedBall) {
        m_physicsScene->setInflatedBallFrozen(m_physGunGrabbedBall, true);
        for (SpawnedInflatedBallInstance& ball : m_inflatedBalls) {
            if (ball.physicsBall == m_physGunGrabbedBall) {
                ball.freezeFlashSeconds = 0.34f;
            }
        }
        m_notification = { "Objeto congelado", 1.55f, 1.55f };
        endPhysGunGrab();
        return;
    }
    if (!m_physicsScene || !m_physicsScene->grabbing()) {
        endPhysGunGrab();
        return;
    }
    if (m_physicsScene->grabbedRagdoll()) {
        m_notification = {
            "Use o painel RAGDOLL para controlar a rigidez",
            2.2f, 2.2f
        };
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
    const bool bodyHit = m_physicsScene->raycast(ray,
        MaximumPhysGunQueryDistanceMeters, hit);
    InflatedBallHandle3D ball;
    float ballDistance = 0.0f;
    if (m_physicsScene->raycastInflatedBall(ray,
            bodyHit ? hit.distance : MaximumPhysGunQueryDistanceMeters,
            ball, ballDistance)) {
        if (!m_physicsScene->inflatedBallState(ball).frozen) return;
        m_physicsScene->setInflatedBallFrozen(ball, false);
        m_notification = { "Objeto descongelado", 1.55f, 1.55f };
        return;
    }
    if (!bodyHit || !hit.body || !m_physicsScene->bodyFrozen(hit.body)) {
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
    for (const SpawnedInflatedBallInstance& ball : m_inflatedBalls) {
        if (!m_physicsScene->inflatedBallState(ball.physicsBall).frozen) continue;
        m_physicsScene->setInflatedBallFrozen(ball.physicsBall, false);
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
    for (SpawnedInflatedBallInstance& ball : m_inflatedBalls) {
        ball.freezeFlashSeconds = std::max(0.0f,
            ball.freezeFlashSeconds - deltaTime);
    }
    const Vec3 forward = viewForward(m_laboratoryCameraYaw,
        m_laboratoryCameraPitch).normalized();
    if (m_physGunGrabbedEntityId == 0 && m_physGunTriggerHeld) {
        beginPhysGunGrab();
    }
    if (m_physGunGrabbedEntityId != 0 && m_physGunGrabbedBall) {
        updateInflatedBallGrab(deltaTime);
    } else if (m_physGunGrabbedEntityId != 0) {
        const auto propInstance = std::find_if(m_spawnedProps.begin(),
            m_spawnedProps.end(), [&](const SpawnedPropInstance& candidate) {
                return candidate.entityId == m_physGunGrabbedEntityId;
            });
        const auto ragdollInstance = std::find_if(
            m_spawnedRagdolls.begin(), m_spawnedRagdolls.end(),
            [&](const SpawnedRagdollInstance& candidate) {
                return candidate.entityId == m_physGunGrabbedEntityId;
            });
        const bool validProp = propInstance != m_spawnedProps.end()
            && m_physicsScene->contains(propInstance->physicsBody);
        const bool validRagdoll =
            ragdollInstance != m_spawnedRagdolls.end()
            && m_physicsScene->contains(ragdollInstance->physicsRagdoll);
        if (!validProp && !validRagdoll) {
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
            if (validProp) {
                const PhysicsBodyState3D state =
                    m_physicsScene->bodyState(propInstance->physicsBody);
                m_physGunBeamTargetWorldPosition = state.position
                    + state.orientation.rotate(m_physGunLocalGrabPoint);
            } else {
                const RagdollState3D state =
                    m_physicsScene->ragdollState(
                        ragdollInstance->physicsRagdoll);
                const std::uint32_t linkIndex =
                    m_physicsScene->grabbedRagdollLink();
                if (linkIndex >= state.links.size()) {
                    endPhysGunGrab();
                } else {
                    const PhysicsBodyState3D& link =
                        state.links[linkIndex];
                    m_physGunBeamTargetWorldPosition = link.position
                        + link.orientation.rotate(
                            m_physGunLocalGrabPoint);
                }
            }
        }
    }

    // Vento (ver WindSystem): um unico avanco de relogio por quadro, depois
    // amostrado em duas alturas diferentes - a do jogador (fisica/audio) e
    // uma de referencia bem mais alta (nuvens, ver mais abaixo) - mesma
    // direcao/rajada em ambas, so a magnitude muda com a altura.
    m_windSystem.advance(deltaTime);
    const Vec3 playerPosition = m_physicsScene->hasCharacter()
        ? m_physicsScene->characterState().position
        : m_laboratoryCameraPosition;
    const Vec3 windAtPlayerHeight =
        m_windSystem.velocityAtHeight(playerPosition.z);
    // So altera o vetor de vento da cena; a formula de arrasto quadratico
    // em si ja existia (ver PhysXScene3D::simulate) e nunca precisou mudar.
    // Sem abrigo aplicado aqui: cada prop dinamico ja recalcula seu proprio
    // abrigo por posicao dentro de PhysXScene3D::simulate() (mais preciso
    // que reusar uma unica checagem na posicao do jogador para props
    // espalhados pelo cenario). O abrigo do UIVO AMBIENTE (audio) e
    // calculado separadamente mais abaixo, ver windShelterExposure3D.
    m_physicsScene->setAirVelocity(windAtPlayerHeight);
    m_physicsScene->setOceanTimeSeconds(m_laboratoryElapsedTime);

    // Altitude tipica de base de nuvens cumulus - bem mais alta que
    // qualquer ponto alcancavel no Laboratorio, entao as nuvens sempre leem
    // o vento "de cima", acima do marco de forca plena (ver
    // WindSettings3D::referenceHeightMeters).
    constexpr float CloudReferenceAltitudeMeters = 600.0f;
    const Vec3 windAtCloudAltitude =
        m_windSystem.velocityAtHeight(CloudReferenceAltitudeMeters);
    m_cloudWindOffset += Vec2 { windAtCloudAltitude.x, windAtCloudAltitude.y }
        * m_cloudWindVisualScale * deltaTime;

    updatePhysicsBenchmark(deltaTime);
    // O controlador observa exclusivamente o snapshot do passo anterior e
    // enfileira alvos/torques antes do safe point da próxima simulação.
    updateActiveRagdolls(deltaTime);
    m_physicsScene->simulate(deltaTime);
    ++m_laboratoryPhysicsStep;
    readInflatedBallStates(deltaTime);
    captureLaboratoryGoalNet();
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
            instance.simulationPreviousState = instance.physicsState;
            instance.physicsState = update.state;
            instance.updatedAtPhysicsStep = m_laboratoryPhysicsStep;
        }
    }
    for (SpawnedRagdollInstance& instance : m_spawnedRagdolls) {
        if (m_physicsScene->contains(instance.physicsRagdoll)) {
            // Keep the step we are leaving so the renderer can interpolate
            // across it (see SpawnedRagdollInstance::simulationPreviousState).
            instance.simulationPreviousState = instance.physicsState;
            instance.physicsState =
                m_physicsScene->ragdollState(instance.physicsRagdoll);
        }
    }
    updatePhysicsBenchmarkAnalytics(deltaTime);

    // Pitch completo (nao so o yaw planar) e o que da ao HRTF a pista de
    // elevacao: e o motivo pratico de ligar HRTF, entao a pose enviada ao
    // AudioDevice3D precisa refletir para onde a camera realmente aponta.
    AudioListenerPose3D listenerPose;
    listenerPose.position = m_laboratoryCameraPosition;
    listenerPose.forward = viewForward(m_laboratoryCameraYaw,
        m_laboratoryCameraPitch).normalized();
    m_worldAudio.update(listenerPose);

    const Vec3 characterVelocityForAudio = m_physicsScene->hasCharacter()
        ? m_physicsScene->characterState().velocity : Vec3 {};
    // Oclusao do uivo ambiente por geometria (ver WindShelter3D.hpp): o
    // jogador nao deveria ouvir o vento como se estivesse ao ar livre estando
    // dentro de um comodo fechado. Escopo deliberadamente so audio - nao ha
    // forca de vento aplicada ao personagem hoje (so props dinamicos sentem
    // arrasto, ja abrigados individualmente acima).
    const float windAudioExposure = windShelterExposure3D(*m_physicsScene,
        playerPosition, windAtPlayerHeight,
        m_windSystem.settings().audioShelterDistanceMeters);
    m_worldAudio.updateWindAmbience(windAtPlayerHeight * windAudioExposure,
        characterVelocityForAudio, m_windAudioVolume);
}

// Physgun na bola: mola criticamente amortecida no centro, com aceleracao
// limitada. A bola segue a mira, achata no que encontrar pelo caminho e,
// solta, sai com a velocidade que tinha (o arremesso).
void WorkbenchApp::updateInflatedBallGrab(float deltaTime) {
    if (!m_physicsScene->contains(m_physGunGrabbedBall)) {
        endPhysGunGrab();
        return;
    }
    const Quaternion cameraOrientation = viewOrientation(
        m_laboratoryCameraYaw, m_laboratoryCameraPitch);
    const Vec3 forward = viewForward(m_laboratoryCameraYaw,
        m_laboratoryCameraPitch).normalized();
    const Vec3 target = m_physGunHoldMode == PhysGunHoldMode::FixedPose
        ? m_laboratoryCameraPosition
            + cameraOrientation.rotate(m_physGunFixedCenterCameraLocal)
        : m_laboratoryCameraPosition + forward * m_physGunHoldDistance;
    const InflatedBallState3D state =
        m_physicsScene->inflatedBallState(m_physGunGrabbedBall);
    constexpr float AngularFrequency = 2.0f * Pi * 5.0f;
    constexpr float MaximumAcceleration = 250.0f;
    Vec3 acceleration = (target - state.position)
            * (AngularFrequency * AngularFrequency)
        - state.linearVelocity * (2.0f * AngularFrequency)
        + Vec3 { 0.0f, 0.0f, 9.81f };
    const float magnitude = acceleration.length();
    if (magnitude > MaximumAcceleration) {
        acceleration *= MaximumAcceleration / magnitude;
    }
    m_physicsScene->setInflatedBallMotion(m_physGunGrabbedBall,
        state.position, state.linearVelocity + acceleration * deltaTime,
        state.angularVelocity * std::exp(-4.0f * deltaTime));
    m_physGunBeamTargetWorldPosition = state.position;
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
        // A bola do menu e a procedural (a mesma que o card cria).
        const GpuModel3D& visual = definitions[index].inflatedBall && !m_soccerBallVisual.parts.empty()
            ? m_soccerBallVisual : definitions[index].visual;
        // Camera estatica, objeto parado - previousPosition/Orientation
        // repetem os mesmos valores (motion vector zero de proposito).
        appendGpuModelRenderables(visual, position,
            previewOrientation(), position, previewOrientation(),
            1.28f / largest, false, false, true, meshes, 100'000.0f);
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
    scene.environment.skyIrradiance = 0.38f;
    scene.showSky = false;
    scene.showShadows = false;
    return activeRenderer.renderScene3D(scene, 768, 512);
}

void WorkbenchApp::ensureViewerFloorLoaded(Renderer& activeRenderer) {
    if (m_viewerFloorLoaded) return;
    const std::vector<std::byte> pixels = buildFloorCheckerboardPixels();
    const MeshData3D mesh = buildFloorQuadMesh();
    constexpr std::uint32_t textureSize = static_cast<std::uint32_t>(
        ViewerFloorTileCount * ViewerFloorTexelsPerTile);

    GpuModelPart3D floorPart;
    floorPart.albedoTexture = activeRenderer.createTexture2D(
        { textureSize, textureSize }, pixels);
    floorPart.vertexBuffer = activeRenderer.createBuffer({
        mesh.vertices.size() * sizeof(MeshVertex3D),
        RHI::BufferUsage::Vertex, true, "Viewer floor vertices" });
    floorPart.indexBuffer = activeRenderer.createBuffer({
        mesh.indices.size() * sizeof(std::uint32_t),
        RHI::BufferUsage::Index, true, "Viewer floor indices" });
    activeRenderer.writeBuffer(floorPart.vertexBuffer, 0,
        std::as_bytes(std::span(mesh.vertices)));
    activeRenderer.writeBuffer(floorPart.indexBuffer, 0,
        std::as_bytes(std::span(mesh.indices)));
    floorPart.indexCount = static_cast<std::uint32_t>(mesh.indices.size());

    m_viewerFloor.parts = { floorPart };
    m_viewerFloor.textures = { floorPart.albedoTexture };
    m_viewerFloor.dimensionsMeters = {
        ViewerFloorHalfExtentMeters * 2.0f,
        ViewerFloorHalfExtentMeters * 2.0f, 0.0f
    };
    m_viewerFloorLoaded = true;
}

} // namespace MatterEngine::Workbench
