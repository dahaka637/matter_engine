#include "Workbench/WorkbenchApp.hpp"

#include "Engine/Character/CharacterControlApplication3D.hpp"
#include "Engine/Core/Log.hpp"
#include "Engine/Geometry/MeshData3D.hpp"
#include "Engine/Geometry/GltfLoader.hpp"
#include "Engine/RHI/RHITypes.hpp"

#include <algorithm>
#include <cmath>
#include <span>
#include <stdexcept>
#include <string>
#include <vector>

namespace MatterEngine::Workbench {
namespace {

constexpr float Pi = 3.14159265358979323846f;
constexpr float MaximumRagdollSpawnDistanceMeters = 18.0f;
constexpr float EmptySpaceRagdollSpawnDistanceMeters = 7.0f;
constexpr std::size_t MaximumSpawnedRagdolls = 256;
const Vec3 RagdollVisualColor { 0.67f, 0.76f, 0.82f };

Vec3 ragdollViewForward(float yaw, float pitch) {
    const float planar = std::cos(pitch);
    return { planar * std::cos(yaw), planar * std::sin(yaw),
        std::sin(pitch) };
}


MeshData3D buildCapsuleMesh(
    const RagdollCapsuleDefinition3D& capsule) {
    // A cápsula nativa do PhysX é alinhada ao eixo X. A mesma geometria é
    // assada aqui já no espaço local do link, incluindo pose do collider:
    // render e colisão permanecem rigorosamente sobrepostos.
    constexpr int RadialSegments = 12;
    constexpr int HemisphereRings = 4;
    // Cônica: tampa -X de raio radiusMeters, +X de raio do outro lado; o
    // tronco liga os dois equadores (a mesma forma do TaperedCapsule).
    const float negativeRadius = capsule.radiusMeters;
    const float positiveRadius = ragdollCapsuleRadiusAtPositiveX3D(capsule);
    const float halfSegment = ragdollCapsuleHalfSegment3D(capsule);

    struct Ring {
        float x = 0.0f;
        float radial = 0.0f;
        float normalX = 0.0f;
        float normalRadial = 1.0f;
    };
    std::vector<Ring> rings;
    rings.reserve(static_cast<std::size_t>(HemisphereRings * 2 + 3));
    rings.push_back(
        { -halfSegment - negativeRadius, 0.0f, -1.0f, 0.0f });
    for (int index = 1; index <= HemisphereRings; ++index) {
        const float angle = -Pi * 0.5f
            + Pi * 0.5f
                * static_cast<float>(index)
                / static_cast<float>(HemisphereRings);
        rings.push_back({ -halfSegment + negativeRadius * std::sin(angle),
            negativeRadius * std::cos(angle), std::sin(angle),
            std::cos(angle) });
    }
    if (halfSegment > 0.00001f) {
        rings.push_back(
            { halfSegment, positiveRadius, 0.0f, 1.0f });
    }
    for (int index = 1; index <= HemisphereRings; ++index) {
        const float angle = Pi * 0.5f
            * static_cast<float>(index)
            / static_cast<float>(HemisphereRings);
        rings.push_back({ halfSegment + positiveRadius * std::sin(angle),
            positiveRadius * std::cos(angle), std::sin(angle),
            std::cos(angle) });
    }

    MeshData3D mesh;
    mesh.vertices.reserve(
        rings.size() * static_cast<std::size_t>(RadialSegments));
    for (std::size_t ringIndex = 0;
            ringIndex < rings.size(); ++ringIndex) {
        const Ring& ring = rings[ringIndex];
        for (int segment = 0; segment < RadialSegments; ++segment) {
            const float angle = 2.0f * Pi
                * static_cast<float>(segment)
                / static_cast<float>(RadialSegments);
            const float cosine = std::cos(angle);
            const float sine = std::sin(angle);
            const Vec3 positionLocal {
                ring.x, ring.radial * cosine, ring.radial * sine
            };
            const Vec3 normalLocal {
                ring.normalX,
                ring.normalRadial * cosine,
                ring.normalRadial * sine
            };
            MeshVertex3D vertex;
            vertex.position = capsule.localPosition
                + capsule.localOrientation.rotate(positionLocal);
            vertex.normal =
                capsule.localOrientation.rotate(normalLocal).normalized();
            vertex.uv = {
                static_cast<float>(segment)
                    / static_cast<float>(RadialSegments),
                static_cast<float>(ringIndex)
                    / static_cast<float>(rings.size() - 1)
            };
            vertex.color = RagdollVisualColor;
            mesh.vertices.push_back(vertex);
        }
    }

    for (std::size_t ring = 0; ring + 1 < rings.size(); ++ring) {
        for (int segment = 0; segment < RadialSegments; ++segment) {
            const std::uint32_t nextSegment = static_cast<std::uint32_t>(
                (segment + 1) % RadialSegments);
            const std::uint32_t current = static_cast<std::uint32_t>(
                ring * RadialSegments + static_cast<std::size_t>(segment));
            const std::uint32_t next = static_cast<std::uint32_t>(
                (ring + 1) * RadialSegments
                    + static_cast<std::size_t>(segment));
            const std::uint32_t currentAround = static_cast<std::uint32_t>(
                ring * RadialSegments + nextSegment);
            const std::uint32_t nextAround = static_cast<std::uint32_t>(
                (ring + 1) * RadialSegments + nextSegment);
            mesh.indices.insert(mesh.indices.end(),
                { current, next, nextAround,
                    current, nextAround, currentAround });
        }
    }
    recomputeBounds(mesh);
    return mesh;
}

MeshData3D buildBoxMesh(Vec3 center, Vec3 halfExtents,
    Quaternion orientation) {
    MeshData3D mesh;
    const auto appendFace = [&](Vec3 normal, Vec3 tangent, Vec3 bitangent) {
        const std::uint32_t first =
            static_cast<std::uint32_t>(mesh.vertices.size());
        const Vec3 faceCenter {
            normal.x * halfExtents.x,
            normal.y * halfExtents.y,
            normal.z * halfExtents.z
        };
        const float tangentExtent = std::abs(tangent.x) * halfExtents.x
            + std::abs(tangent.y) * halfExtents.y
            + std::abs(tangent.z) * halfExtents.z;
        const float bitangentExtent =
            std::abs(bitangent.x) * halfExtents.x
            + std::abs(bitangent.y) * halfExtents.y
            + std::abs(bitangent.z) * halfExtents.z;
        const Vec2 uvs[] {
            { 0.0f, 0.0f }, { 1.0f, 0.0f },
            { 1.0f, 1.0f }, { 0.0f, 1.0f }
        };
        const Vec3 corners[] {
            faceCenter - tangent * tangentExtent
                - bitangent * bitangentExtent,
            faceCenter + tangent * tangentExtent
                - bitangent * bitangentExtent,
            faceCenter + tangent * tangentExtent
                + bitangent * bitangentExtent,
            faceCenter - tangent * tangentExtent
                + bitangent * bitangentExtent
        };
        const Vec3 transformedNormal =
            orientation.rotate(normal).normalized();
        for (int index = 0; index < 4; ++index) {
            MeshVertex3D vertex;
            vertex.position = center + orientation.rotate(corners[index]);
            vertex.normal = transformedNormal;
            vertex.uv = uvs[index];
            vertex.color = RagdollVisualColor;
            mesh.vertices.push_back(vertex);
        }
        mesh.indices.insert(mesh.indices.end(),
            { first, first + 1, first + 2,
                first, first + 2, first + 3 });
    };
    appendFace({ 1.0f, 0.0f, 0.0f },
        { 0.0f, 1.0f, 0.0f }, { 0.0f, 0.0f, 1.0f });
    appendFace({ -1.0f, 0.0f, 0.0f },
        { 0.0f, -1.0f, 0.0f }, { 0.0f, 0.0f, 1.0f });
    appendFace({ 0.0f, 1.0f, 0.0f },
        { -1.0f, 0.0f, 0.0f }, { 0.0f, 0.0f, 1.0f });
    appendFace({ 0.0f, -1.0f, 0.0f },
        { 1.0f, 0.0f, 0.0f }, { 0.0f, 0.0f, 1.0f });
    appendFace({ 0.0f, 0.0f, 1.0f },
        { 1.0f, 0.0f, 0.0f }, { 0.0f, 1.0f, 0.0f });
    appendFace({ 0.0f, 0.0f, -1.0f },
        { 1.0f, 0.0f, 0.0f }, { 0.0f, -1.0f, 0.0f });
    recomputeBounds(mesh);
    return mesh;
}

MeshData3D buildJointSphereMesh() {
    RagdollCapsuleDefinition3D sphere;
    sphere.radiusMeters = 1.0f;
    sphere.lengthMeters = 2.001f;
    return buildCapsuleMesh(sphere);
}

MeshData3D buildRagdollLinkVisual(
    const RagdollLinkDefinition3D& link) {
    if (link.collider.shape == RagdollColliderShape3D::Box) {
        return buildBoxMesh(link.collider.localPosition,
            link.collider.boxHalfExtents, link.collider.localOrientation);
    }
    return buildCapsuleMesh(link.collider);
}

GpuModel3D uploadRagdollMesh(
    Renderer& renderer, const MeshData3D& mesh) {
    if (mesh.vertices.empty() || mesh.indices.empty()) {
        throw std::runtime_error("Malha procedural de osso vazia");
    }

    GpuModel3D model;
    GpuModelPart3D part;
    part.vertexBuffer = renderer.createBuffer({
        mesh.vertices.size() * sizeof(MeshVertex3D),
        RHI::BufferUsage::Vertex, true, "Ragdoll link vertices" });
    part.indexBuffer = renderer.createBuffer({
        mesh.indices.size() * sizeof(std::uint32_t),
        RHI::BufferUsage::Index, true, "Ragdoll link indices" });
    renderer.writeBuffer(part.vertexBuffer, 0,
        std::as_bytes(std::span(mesh.vertices)));
    renderer.writeBuffer(part.indexBuffer, 0,
        std::as_bytes(std::span(mesh.indices)));
    part.indexCount = static_cast<std::uint32_t>(mesh.indices.size());
    part.metallic = 0.0f;
    part.roughness = 0.78f;
    model.parts.push_back(part);
    model.dimensionsMeters = mesh.boundsMax - mesh.boundsMin;
    return model;
}

} // namespace

void WorkbenchApp::ensureRagdollAssetsLoaded(Renderer& activeRenderer) {
    if ((m_humanRagdollProfile && !m_characterVisual.parts.empty())
        || m_ragdollAssetsLoadFailed
        || !m_physicsScene) {
        return;
    }
    try {
        if (!m_ragdollCharacter) {
            m_ragdollCharacter = loadRagdollCharacter3D(
                std::string(MATTERENGINE_ASSETS_DIR) + "/" + m_characterAssetPath);
        }
        m_humanRagdollProfile = m_ragdollCharacter->profile;
        Vec3 minimum=m_ragdollCharacter->mesh.boundsMin;
        Vec3 maximum=m_ragdollCharacter->mesh.boundsMax;
        const auto absolute=[](Vec3 value) {return Vec3{std::abs(value.x),std::abs(value.y),std::abs(value.z)};};
        for (const auto& link : m_humanRagdollProfile->links) {
            const auto rotation=link.modelOrientation*link.collider.localOrientation;
            Vec3 extents;
            if (link.collider.shape==RagdollColliderShape3D::Box) {
                const auto half=link.collider.boxHalfExtents;
                extents=absolute(rotation.rotate({half.x,0,0}))
                    +absolute(rotation.rotate({0,half.y,0}))+absolute(rotation.rotate({0,0,half.z}));
            } else {
                const auto radius=link.collider.radiusMeters;
                extents=absolute(rotation.rotate({std::max(0.0f,link.collider.lengthMeters*0.5f-radius),0,0}))
                    +Vec3{radius,radius,radius};
            }
            const auto center=link.modelPosition+link.modelOrientation.rotate(link.collider.localPosition);
            const auto lo=center-extents, hi=center+extents;
            minimum={std::min(minimum.x,lo.x),std::min(minimum.y,lo.y),std::min(minimum.z,lo.z)};
            maximum={std::max(maximum.x,hi.x),std::max(maximum.y,hi.y),std::max(maximum.z,hi.z)};
        }
        m_characterSpawnHalfExtents=(maximum-minimum)*0.5f;
        m_characterSpawnCenter=(maximum+minimum)*0.5f;
        m_characterVisual = uploadRagdollMesh(activeRenderer, m_ragdollCharacter->mesh);
        if (!m_ragdollCharacter->albedoPath.empty()) {
            const auto image=loadImageRgba3D(m_ragdollCharacter->albedoPath);
            const auto texture=activeRenderer.createTexture2D(
                {static_cast<std::uint32_t>(image.width),static_cast<std::uint32_t>(image.height)},
                image.rgbaPixels);
            m_characterVisual.textures.push_back(texture);
            m_characterVisual.parts.front().albedoTexture=texture;
        }
        if (!m_characterThumbnail && !m_ragdollCharacter->thumbnailPath.empty())
            m_characterThumbnail = activeRenderer.loadUiTexture(m_ragdollCharacter->thumbnailPath);
        for (const auto& link : m_humanRagdollProfile->links) {
            m_ragdollLinkVisuals.push_back(uploadRagdollMesh(activeRenderer,
                buildRagdollLinkVisual(link)));
            m_ragdollJointLocalPositions.push_back(link.modelOrientation.conjugate().rotate(
                link.inboundJoint.anchorModelPosition-link.modelPosition));
        }
        Log::info("Personagem carregado: " + m_ragdollCharacter->displayName
            + " | rig " + m_humanRagdollProfile->id + " | skin GPU");
        m_ragdollJointVisual = uploadRagdollMesh(
            activeRenderer, buildJointSphereMesh());
    } catch (const std::exception& error) {
        Log::error(std::string("Falha ao carregar articulation: ")
            + error.what());
        m_humanRagdollProfile.reset();
        releaseGpuModel3D(activeRenderer, m_characterVisual);
        for (GpuModel3D& visual : m_ragdollLinkVisuals) {
            releaseGpuModel3D(activeRenderer, visual);
        }
        m_ragdollLinkVisuals.clear();
        releaseGpuModel3D(activeRenderer, m_ragdollJointVisual);
        m_ragdollJointLocalPositions.clear();
        m_ragdollAssetsLoadFailed = true;
        m_laboratoryStatus = "Falha ao carregar articulation";
    }
}

void WorkbenchApp::spawnHumanRagdoll() {
    if (!m_physicsScene || !m_humanRagdollProfile
        || !m_laboratoryMapLoaded) {
        m_laboratoryStatus = "Ragdoll indisponivel";
        return;
    }
    if (m_spawnedRagdolls.size() >= MaximumSpawnedRagdolls) {
        m_laboratoryStatus = "Limite de ragdolls atingido";
        return;
    }

    const Vec3 direction = ragdollViewForward(
        m_laboratoryCameraYaw, m_laboratoryCameraPitch).normalized();
    const Ray3D ray { m_laboratoryCameraPosition, direction };
    PhysicsRayHit3D hit;
    Vec3 pelvisPosition =
        ray.origin + ray.direction * EmptySpaceRagdollSpawnDistanceMeters;
    if (m_physicsScene->raycastStatic(ray,
            MaximumRagdollSpawnDistanceMeters, hit)) {
        if (hit.normal.z > 0.55f) {
            pelvisPosition = hit.position
                + Vec3 { 0.0f, 0.0f,
                    m_characterSpawnHalfExtents.z - m_characterSpawnCenter.z
                        + 0.055f };
        } else {
            // Em paredes, mantém o esqueleto à frente da superfície em vez
            // de enterrar a T-pose nela.
            pelvisPosition = hit.position - ray.direction * 0.75f;
        }
    }

    const Quaternion orientation = Quaternion::fromAxisAngle(
        { 0.0f, 0.0f, 1.0f }, m_laboratoryCameraYaw);
    const std::optional<Vec3> safePosition = findSafeSpawnPosition(
        pelvisPosition + orientation.rotate(m_characterSpawnCenter),
        // The generic search adds 25 mm; match the 35 mm final guard below.
        m_characterSpawnHalfExtents + Vec3 { 0.01f, 0.01f, 0.01f },
        orientation, 5.0f);
    if (!safePosition
        || !spawnHumanRagdollAt(
            *safePosition - orientation.rotate(m_characterSpawnCenter),
            orientation, false)) {
        m_laboratoryStatus = "Sem espaco seguro para criar o ragdoll";
        m_notification = {
            "Spawn bloqueado: procure uma area livre", 2.2f, 2.2f
        };
    }
}

bool WorkbenchApp::spawnHumanRagdollAt(Vec3 pelvisPosition,
    Quaternion orientation, bool benchmarkEntity) {
    if (!m_physicsScene || !m_humanRagdollProfile
        || !m_laboratoryMapLoaded
        || m_spawnedRagdolls.size() >= MaximumSpawnedRagdolls) {
        return false;
    }
    constexpr float SpawnClearanceMeters = 0.035f;
    if (m_physicsScene->overlapsBox(
            pelvisPosition + orientation.rotate(m_characterSpawnCenter),
            m_characterSpawnHalfExtents
                + Vec3 { SpawnClearanceMeters, SpawnClearanceMeters,
                    SpawnClearanceMeters },
            orientation)) {
        return false;
    }

    RagdollSpawnDefinition3D spawn;
    spawn.entityId = m_nextEntityId++;
    spawn.pelvisPosition = pelvisPosition;
    spawn.orientation = orientation.normalized();
    spawn.rigidityPercent = m_ragdollRigidityPercent;
    spawn.active = m_ragdollActive;
    try {
        const RagdollHandle3D handle =
            m_physicsScene->createRagdoll(*m_humanRagdollProfile, spawn);
        const RagdollState3D initial =
            m_physicsScene->ragdollState(handle);
        SpawnedRagdollInstance instance;
        instance.entityId = spawn.entityId;
        instance.physicsRagdoll = handle;
        instance.physicsState = initial;
        instance.previousPhysicsState = initial;
        const Vec3 spawnForward = orientation.rotate({ 1.0f, 0.0f, 0.0f });
        instance.guideFacingYawRadians = std::atan2(
            spawnForward.y, spawnForward.x);
        instance.kind = ArticulatedRigKind::Human;
        instance.active = spawn.active;
        instance.locomotion.reset(*m_humanRagdollProfile, initial);
        instance.adaptivePhysics.reset(*m_humanRagdollProfile);
        instance.biomechanics.reset(*m_humanRagdollProfile, initial,
            m_physicsScene->ragdollDynamics(handle));
        m_spawnedRagdolls.push_back(std::move(instance));
        if (benchmarkEntity) {
            m_physicsBenchmarkRagdollEntityIds.push_back(spawn.entityId);
        }
        Log::info("Ragdoll " + m_humanRagdollProfile->id + " criado: 18 segmentos, 41 DOFs, "
            + std::string(spawn.active ? "controle ativo."
                : "passivo, rigidez ")
            + (spawn.active ? std::string {}
                : std::to_string(m_ragdollRigidityPercent) + "%."));
        m_laboratoryStatus = spawn.active
            ? "Ragdoll ativo criado" : "Ragdoll passivo criado";
        return true;
    } catch (const std::exception& error) {
        Log::error(std::string("Falha ao criar ragdoll humano: ")
            + error.what());
        m_laboratoryStatus = "Falha ao criar ragdoll";
        return false;
    }
}

CharacterLocomotionAnimations3D
WorkbenchApp::characterLocomotionAnimations() const {
    CharacterLocomotionAnimations3D clips;
    if(!m_ragdollCharacter)return clips;
    for(const auto& clip:m_animationClips) {
        if(clip.id==m_ragdollCharacter->idleClipId)clips.idle=&clip;
        if(clip.id==m_ragdollCharacter->walkClipId)clips.walk=&clip;
        if(clip.id==m_ragdollCharacter->idleToSprintClipId)clips.idleToSprint=&clip;
        if(clip.id==m_ragdollCharacter->crouchToSprintClipId)clips.crouchToSprint=&clip;
        if(clip.id==m_ragdollCharacter->runForwardArcLeftClipId)clips.runForwardArcLeft=&clip;
        if(clip.id==m_ragdollCharacter->runForwardArcRightClipId)clips.runForwardArcRight=&clip;
        if(clip.id==m_ragdollCharacter->runBackwardArcRightClipId)clips.runBackwardArcRight=&clip;
        if(clip.id==m_ragdollCharacter->walkBackwardClipId)clips.walkBackward=&clip;
        if(clip.id==m_ragdollCharacter->sprintClipId)clips.sprint=&clip;
        if(clip.id==m_ragdollCharacter->sprintBackwardClipId)clips.sprintBackward=&clip;
        if(clip.id==m_ragdollCharacter->strafeLeftClipId)clips.strafeLeft=&clip;
        if(clip.id==m_ragdollCharacter->strafeRightClipId)clips.strafeRight=&clip;
        if(clip.id==m_ragdollCharacter->sprintStrafeLeftClipId)clips.sprintStrafeLeft=&clip;
        if(clip.id==m_ragdollCharacter->sprintStrafeRightClipId)clips.sprintStrafeRight=&clip;
        if(clip.id==m_ragdollCharacter->jumpStandingClipId)clips.jumpStanding=&clip;
        if(clip.id==m_ragdollCharacter->jumpForwardClipId)clips.jumpForward=&clip;
        if(clip.id==m_ragdollCharacter->jumpBackwardClipId)clips.jumpBackward=&clip;
        if(clip.id==m_ragdollCharacter->jumpLeftClipId)clips.jumpLeft=&clip;
        if(clip.id==m_ragdollCharacter->jumpRightClipId)clips.jumpRight=&clip;
        if(clip.id==m_ragdollCharacter->jumpPrepClipId)clips.jumpPrep=&clip;
        if(clip.id==m_ragdollCharacter->standUpBackClipId)clips.standUpBack=&clip;
        if(clip.id==m_ragdollCharacter->standUpFrontClipId)clips.standUpFront=&clip;
        if(clip.id==m_ragdollCharacter->turnLeftClipId)clips.turnLeft=&clip;
        if(clip.id==m_ragdollCharacter->turnRightClipId)clips.turnRight=&clip;
        for(std::size_t k=0;k<m_ragdollCharacter->tackleClipIds.size()&&k<clips.tackles.size();++k)
            if(clip.id==m_ragdollCharacter->tackleClipIds[k])clips.tackles[k]=&clip;
    }
    // O levantar de bruços escolhido no painel (para comparar).
    {
        const char* ids[2]={"stand_up_front_kimodo","stand_up_front"};
        const int choice=std::clamp(m_getUpFrontClip,0,1);
        for(const auto& clip:m_animationClips)
            if(clip.id==ids[choice])clips.standUpFront=&clip;
    }
    // E o de costas.
    {
        const char* ids[2]={"stand_up_back_kimodo","stand_up_back"};
        const int choice=std::clamp(m_getUpBackClip,0,1);
        for(const auto& clip:m_animationClips)
            if(clip.id==ids[choice])clips.standUpBack=&clip;
    }
    return clips;
}

float WorkbenchApp::ragdollGroundHeight(Vec3 position) const {
    if(!m_physicsScene)return 0;
    PhysicsRayHit3D hit;
    if(m_physicsScene->raycastStatic({position+Vec3{0,0,0.3f},{0,0,-1}},2000,hit)
        &&hit.normal.z>0.7f)return hit.position.z;
    return position.z-(m_humanRagdollProfile?m_humanRagdollProfile->standingRootHeightMeters:1.0f);
}

void WorkbenchApp::takeControlOfLatestCharacter() {
    if (!m_physicsScene || !m_humanRagdollProfile
        || (!m_biomechanicalExperiment
            && !m_animationLocomotionCompatible)) {
        m_laboratoryStatus = "Locomoção do personagem indisponível";
        return;
    }
    const auto candidate = std::find_if(m_spawnedRagdolls.rbegin(),
        m_spawnedRagdolls.rend(), [](const SpawnedRagdollInstance& instance) {
            return instance.kind == ArticulatedRigKind::Human
                && instance.active && !instance.physicsState.links.empty();
        });
    if (candidate == m_spawnedRagdolls.rend()) {
        m_laboratoryStatus = "Crie um personagem ativo antes de assumir";
        return;
    }
    endPhysGunGrab();
    const Vec3 root = candidate->physicsState.links.front().position;
    const float floor = ragdollGroundHeight(root);
    m_physicsScene->placeCharacter({ root.x, root.y, floor },
        m_avatarCharacterSettings);
    m_controlledCharacterEntityId = candidate->entityId;
    candidate->locomotion.reset(*m_humanRagdollProfile,
        candidate->physicsState);
    candidate->adaptivePhysics.reset(*m_humanRagdollProfile);
    m_characterFollowVelocity = {};
    candidate->biomechanics.reset(*m_humanRagdollProfile,
        candidate->physicsState,
        m_physicsScene->ragdollDynamics(candidate->physicsRagdoll));
    const Vec3 forward = candidate->physicsState.links.front()
        .orientation.rotate({ 1.0f, 0.0f, 0.0f });
    m_characterDesiredFacingYaw = std::atan2(forward.y, forward.x);
    m_laboratoryCameraYaw = m_characterDesiredFacingYaw;
    m_laboratoryCameraPitch = -0.20f;
    m_characterProxyVelocityWorld = {};
    m_characterSprinting = false;
    m_hidePhysGunPresentation = true;
    m_laboratoryDebugVisible = false;
    m_showCharacterPanel = false;
    m_laboratoryStatus = m_biomechanicalExperiment
        ? "Experimento biomecânico ativo"
        : "Controle do personagem ativo";
    m_notification = { m_biomechanicalExperiment
            ? "WASD: passada física | Alt: câmera livre | U: congelar"
            : "WASD mover | Alt: câmera livre | U: congelar",
        4.0f, 4.0f };
    syncLaboratoryMouseCapture();
}

void WorkbenchApp::releaseControlledCharacter() {
    if (m_controlledCharacterEntityId == 0) return;
    m_controlledCharacterEntityId = 0;
    m_characterProxyVelocityWorld = {};
    m_characterSprinting = false;
    // O padrão do físgun é oculto; voltar do controle do personagem não
    // deve reexibi-lo sozinho — quem quiser vê-lo usa o checkbox do painel.
    m_hidePhysGunPresentation = true;
    m_laboratoryStatus = "Controle do personagem liberado";
}

void WorkbenchApp::resetControlledCharacterPose() {
    if (!m_physicsScene || !m_humanRagdollProfile
        || m_controlledCharacterEntityId == 0) {
        return;
    }
    const auto controlled = std::find_if(m_spawnedRagdolls.begin(),
        m_spawnedRagdolls.end(), [&](const SpawnedRagdollInstance& item) {
            return item.entityId == m_controlledCharacterEntityId
                && item.active && !item.physicsState.links.empty();
        });
    if (controlled == m_spawnedRagdolls.end()) return;

    endPhysGunGrab();
    const Vec3 oldRoot = controlled->physicsState.links.front().position;
    const GroundProbeResult3D ground = m_physicsScene->probeGround(
        oldRoot + Vec3 { 0.0f, 0.0f, 1.5f },
        0.08f, 0.0f, 4.0f, 60.0f);
    const float floorHeight = ground.hasSurface
        ? ground.pointWorld.z
        : oldRoot.z - m_humanRagdollProfile->standingRootHeightMeters;
    const Quaternion orientation = Quaternion::fromAxisAngle(
        { 0.0f, 0.0f, 1.0f }, m_characterDesiredFacingYaw);

    const RagdollHandle3D previousHandle = controlled->physicsRagdoll;
    m_physicsScene->destroyRagdoll(previousHandle);
    try {
        RagdollSpawnDefinition3D spawn;
        spawn.entityId = controlled->entityId;
        spawn.pelvisPosition = {
            oldRoot.x, oldRoot.y,
            floorHeight + m_humanRagdollProfile->standingRootHeightMeters
        };
        spawn.orientation = orientation;
        spawn.rigidityPercent = m_ragdollRigidityPercent;
        spawn.active = true;
        controlled->physicsRagdoll = m_physicsScene->createRagdoll(
            *m_humanRagdollProfile, spawn);
        const RagdollState3D initial = m_physicsScene->ragdollState(
            controlled->physicsRagdoll);
        controlled->physicsState = initial;
        controlled->previousPhysicsState = initial;
        controlled->frozen = false;
        controlled->skinMatrices.clear();
        controlled->previousSkinMatrices.clear();
        controlled->locomotion.reset(*m_humanRagdollProfile, initial);
        controlled->adaptivePhysics.reset(*m_humanRagdollProfile);
        controlled->navigationAnchorValid = false;
        // O modo do corpo tambem recomeca: o ragdoll novo nasce dinamico, e o
        // estado antigo ("cinematico", no padrao) o deixava sem pose animada e
        // sem motores - 100% ragdoll. Com a locomocao padrao, ja volta nela.
        using BodyMode = SpawnedRagdollInstance::BodyMode;
        controlled->defaultLocomotion.reset(m_characterDesiredFacingYaw);
        controlled->bodyModeSeconds = 0.0f;
        controlled->calmSeconds = 0.0f;
        controlled->airSeconds = 0.0f;
        controlled->physicalFromJump = false;
        controlled->returnBoost = 0.0f;
        controlled->crouchHeld = false;
        controlled->defaultTackling = false;
        controlled->guided = m_defaultLocomotion;
        controlled->guidedExitLeft = 0.0f;
        controlled->bodyMode = m_defaultLocomotion ? BodyMode::Animated : BodyMode::Physical;
        if (controlled->guided)
            m_physicsScene->setRagdollGuided(controlled->physicsRagdoll, true);
        m_characterFollowVelocity = {};
        controlled->biomechanics.reset(*m_humanRagdollProfile, initial,
            m_physicsScene->ragdollDynamics(controlled->physicsRagdoll));
        m_physicsScene->placeCharacter(
            { oldRoot.x, oldRoot.y, floorHeight },
            m_avatarCharacterSettings);
        m_characterProxyVelocityWorld = {};
        m_characterSprinting = false;
        m_laboratoryTaaHistoryValid = false;
        m_laboratoryStatus = "Pose inicial restaurada";
        m_notification = {
            "Pose física e controlador procedural reiniciados",
            2.4f, 2.4f
        };
    } catch (const std::exception& error) {
        controlled->active = false;
        Log::error(std::string("Falha ao restaurar pose do personagem: ")
            + error.what());
        releaseControlledCharacter();
        m_laboratoryStatus = "Falha ao restaurar pose inicial";
    }
}

void WorkbenchApp::toggleControlledCharacterFrozen() {
    if (!m_physicsScene || m_controlledCharacterEntityId == 0) return;
    const auto controlled = std::find_if(m_spawnedRagdolls.begin(),
        m_spawnedRagdolls.end(), [&](const SpawnedRagdollInstance& item) {
            return item.entityId == m_controlledCharacterEntityId
                && item.active;
        });
    if (controlled == m_spawnedRagdolls.end()
        || !m_physicsScene->contains(controlled->physicsRagdoll)) {
        return;
    }

    const bool frozen = !controlled->frozen;
    controlled->frozen = frozen;
    controlled->previousPhysicsState = controlled->physicsState;
    m_physicsScene->setRagdollFrozen(controlled->physicsRagdoll, frozen);
    m_characterProxyVelocityWorld = {};
    m_characterSprinting = false;
    m_laboratoryTaaHistoryValid = false;
    m_laboratoryStatus = frozen
        ? "Ragdoll congelado para inspeção"
        : "Ragdoll descongelado";
    m_notification = {
        frozen ? "RAGDOLL CONGELADO | U para descongelar"
               : "RAGDOLL DESCONGELADO",
        2.4f, 2.4f
    };
}

namespace {
// Levantar com pressa entregue ao padrao: a meia-vida e a duracao da inercia
// (s) - o corpo agachado vai a pose de andar enquanto ja anda.
constexpr float GetUpHandoffHalfLife = 0.16f;
constexpr float GetUpHandoffSeconds = 0.65f;
}

void WorkbenchApp::updateActiveRagdolls(float deltaTime) {
    if(!m_physicsScene||!m_humanRagdollProfile)return;
    const auto clips=characterLocomotionAnimations();
    // O modo padrao com os clipes dele (o trote e o sprint das referencias e
    // as diagonais) por cima dos da locomocao fisica.
    const auto defaultClips=m_ragdollCharacter
        ?defaultLocomotionAnimations3D(clips,m_ragdollCharacter->defaultLocomotionClipIds,m_animationClips):clips;
    // Quem esta dando carrinho (pedido do usuario, 05/10 noite): as pernas
    // dele neste tick. Um boneco com pancada de algo que se move perto delas
    // levou o carrinho - a ajuda dele cai (ImpactAssist3D, ate 20%) e volta.
    // Dono de cada bola (pedido do usuario, 05/10 noite; BallOwnership3D):
    // quem ela tocou por ultimo (fora o dono, que nao a toca) passa a ser o
    // dono; a 5 m dele por 3 s (ou ele sumido), ela volta a colidir com
    // todos. A interacao do jogador o torna dono no manejo, abaixo.
    for (auto& ball : m_inflatedBalls) {
        if (!m_physicsScene->contains(ball.physicsBall)) continue;
        const SpawnedRagdollInstance* owner = nullptr;
        for (const auto& instance : m_spawnedRagdolls)
            if (instance.entityId == ball.ownership.owner() && instance.active && !instance.physicsState.links.empty()
                && m_physicsScene->contains(instance.physicsRagdoll))
                owner = &instance;
        const Vec3 pelvis = owner != nullptr ? owner->physicsState.links.front().position : Vec3 {};
        ball.ownership.update(ball.physicsState, owner != nullptr ? &pelvis : nullptr, deltaTime);
        RagdollHandle3D handle {};
        for (const auto& instance : m_spawnedRagdolls)
            if (instance.entityId == ball.ownership.owner() && instance.active) handle = instance.physicsRagdoll;
        m_physicsScene->setInflatedBallOwner(ball.physicsBall, handle);
    }
    std::vector<std::pair<std::uint64_t, Vec3>> tacklerLegs;
    for (const auto& other : m_spawnedRagdolls) {
        if (!other.active || other.physicsState.links.empty()) continue;
        const bool tackling = other.defaultTackling
            || other.locomotion.telemetry().state == CharacterLocomotionState3D::Tackling;
        if (!tackling) continue;
        for (std::size_t i = 0; i < m_humanRagdollProfile->links.size() && i < other.physicsState.links.size(); ++i) {
            const std::string& id = m_humanRagdollProfile->links[i].id;
            if (id.find("Shin") != std::string::npos || id.find("Foot") != std::string::npos
                || id.find("Thigh") != std::string::npos)
                tacklerLegs.push_back({ other.entityId, other.physicsState.links[i].position });
        }
    }
    for(auto& instance:m_spawnedRagdolls) {
        if(!instance.active||instance.frozen||instance.physicsState.frozen
            ||!m_physicsScene->contains(instance.physicsRagdoll)
            ||instance.physicsState.links.empty())continue;
        {
            // Levou carrinho: pancada de algo que se move e uma perna de quem
            // da o carrinho a menos de 45 cm de um link dele.
            float impulse = 0.0f;
            for (const RagdollInteraction3D& contact : instance.physicsState.interactions)
                if (contact.otherMotion != RagdollContactMotion3D::Static) impulse += contact.normalImpulseNewtonSeconds;
            bool tackled = false;
            if (impulse > 2.0f)
                for (const auto& [entity, leg] : tacklerLegs) {
                    if (entity == instance.entityId) continue;
                    for (const PhysicsBodyState3D& link : instance.physicsState.links)
                        if ((link.position - leg).lengthSquared() < 0.45f * 0.45f) { tackled = true; break; }
                    if (tackled) break;
                }
            if (tackled && m_tackledAssistLow) {
                if (instance.tackledRecovery >= 1.0f) Log::info("corpo: levou carrinho (ajuda baixa)");
                instance.tackledRecovery = 0.0f;
            } else {
                instance.tackledRecovery = std::min(1.0f, instance.tackledRecovery
                    + deltaTime / std::max(0.1f, m_tackledRecoverSeconds));
            }
            // A ajuda depois do impacto (pedido do usuario, 06/10): cai pela
            // forca do choque - ate 50%, ou ate 20% num carrinho levado - e
            // volta aos poucos (no padrao tambem: o choque que o leva ao
            // fisico ja conta).
            instance.impactAssist.update(impactImpulse3D(instance.physicsState), tackled && m_tackledAssistLow,
                impulse, deltaTime);
        }
        const bool grabbed=m_physicsScene->grabbing()
            &&m_physicsScene->grabbedRagdoll()==instance.physicsRagdoll;
        const auto& root=instance.physicsState.links.front();
        std::array<GroundProbeResult3D,2> footGround;
        for(std::size_t i=0;i<m_humanRagdollProfile->links.size();++i) {
            const auto& id=m_humanRagdollProfile->links[i].id;
            const int foot=id=="LeftFoot"?0:id=="RightFoot"?1:-1;
            if(foot<0||i>=instance.physicsState.links.size())continue;
            // O terreno dos pes ve tambem os corpos dinamicos (caixas, props).
            footGround[foot]=m_physicsScene->probeTerrain(
                instance.physicsState.links[i].position+Vec3{0,0,0.10f},
                0.05f,0.0f,0.40f,48.0f);
        }
        CharacterLocomotionInput3D command;
        command.rootPositionWorld = {
            root.position.x,
            root.position.y,
            ragdollGroundHeight(root.position)
                + m_humanRagdollProfile->standingRootHeightMeters
        };
        command.rootVelocityWorld = root.linearVelocity;
        command.facingYawRadians = instance.guideFacingYawRadians;
        command.grounded = true;
        command.controlled = instance.entityId
            == m_controlledCharacterEntityId;
        if (command.controlled && m_physicsScene->hasCharacter()) {
            const PhysicsCharacterState3D& character =
                m_physicsScene->characterState();
            const float bodyHeight = character.crouched
                ? m_avatarCharacterSettings.crouchedHeight
                : m_avatarCharacterSettings.standingHeight;
            const float feetHeight = character.position.z
                - bodyHeight * 0.5f;
            command.rootPositionWorld = {
                character.position.x,
                character.position.y,
                feetHeight + m_humanRagdollProfile->standingRootHeightMeters
            };
            command.rootVelocityWorld = character.velocity;
            command.proxyVelocityWorld = m_characterProxyVelocityWorld;
            command.intent = m_characterIntent;
            command.facingYawRadians = m_characterDesiredFacingYaw;
            instance.guideFacingYawRadians = m_characterDesiredFacingYaw;
            command.lookPitchRadians = m_laboratoryCameraPitch;
            command.grounded = character.grounded;
            command.crouched = character.crouched;
            command.sprinting = m_characterSprinting;
            // Chao em qualquer ponto: os pes consultam onde estao e onde vao
            // pisar (so o personagem controlado; os bonecos soltos usam a
            // sonda embaixo de cada pe).
            PhysicsScene3D* scene = m_physicsScene.get();
            command.groundAt = [scene](Vec3 point) {
                return scene->probeTerrain(point + Vec3 { 0.0f, 0.0f, 0.55f },
                    0.03f, 0.0f, 1.5f, 60.0f);
            };
        }
        command.manipulated = grabbed;
        if (!command.controlled && m_adaptivePhysics) {
            // Capsula virtual dos bonecos nao controlados (modo fisico).
            if (!instance.navigationAnchorValid) {
                instance.navigationAnchor = root.position;
                instance.navigationAnchorVelocity = {};
                instance.navigationAnchorValid = true;
            }
            command.rootPositionWorld.x = instance.navigationAnchor.x;
            command.rootPositionWorld.y = instance.navigationAnchor.y;
            command.rootVelocityWorld = {};
        } else {
            instance.navigationAnchorValid = false;
            instance.navigationAnchorVelocity = {};
        }
        // Todos os bonecos humanos ativos: o controlado segue a capsula; os
        // outros tem a referencia no proprio corpo (empurraveis, dao passo).
        const bool adaptive = m_adaptivePhysics
            && instance.kind == ArticulatedRigKind::Human;
        command.forceDrivenRoot = adaptive;
        if (adaptive) {
            // As juntas perto de uma pancada cedem (do tick anterior).
            command.jointStrength = instance.adaptivePhysics.output().jointStrength;
            command.externalPerturbation = instance.adaptivePhysics.output().perturbation;
            if (command.controlled) command.jumpCrouch = m_laboratoryJumpCrouch;
        }
        command.grabbedLink = grabbed
            ? m_physicsScene->grabbedRagdollLink()
            : RagdollDynamics3D::InvalidIndex;
        command.poseAuthority = m_ragdollForceControlOverride
            ? m_ragdollAuxiliaryPercent * 0.01f : 1.0f;
        // (A rigidez muscular geral do menu do personagem; o painel Ragdoll
        // ainda pode forcar outra.)
        command.muscleAuthority = m_ragdollForceControlOverride
            ? m_ragdollMusclePercent * 0.01f : std::clamp(m_characterMusclePercent * 0.01f, 0.0f, 1.0f);
        command.footGround = footGround;
        // Only the controlled character pays for the mass matrix/Jacobian;
        // both the biomechanical path and the hybrid balance assist below
        // need it, everyone else in the scene stays with an invalid one.
        const RagdollDynamics3D dynamics = command.controlled
            ? m_physicsScene->ragdollDynamics(instance.physicsRagdoll)
            : RagdollDynamics3D {};
        if (command.controlled && m_biomechanicalExperiment) {
            BiomechanicalBipedInput3D physicalInput;
            physicalInput.desiredVelocityWorld =
                m_biomechanicalRequestedVelocity;
            physicalInput.desiredFacingYawRadians =
                m_characterDesiredFacingYaw;
            physicalInput.muscleAuthority = command.muscleAuthority;
            physicalInput.balanceAssistance =
                m_biomechanicalBalanceAssistPercent * 0.01f;
            // Reference poses come only from clips already vetted against the
            // profile (see loadAnimationCatalog): the character's own idle
            // and walk.
            for (const AnimationClip3D& candidate : m_animationClips) {
                if (m_ragdollCharacter
                    && candidate.id == m_ragdollCharacter->idleClipId) {
                    physicalInput.idleReferenceClip = &candidate;
                } else if (m_ragdollCharacter
                    && candidate.id == m_ragdollCharacter->walkClipId) {
                    physicalInput.locomotionReferenceClip = &candidate;
                }
            }
            instance.biomechanics.update(*m_humanRagdollProfile,
                instance.physicsState, dynamics, physicalInput, deltaTime);
            const auto& physicalOutput = instance.biomechanics.output();
            m_physicsScene->setRagdollActiveDriveTargets(
                instance.physicsRagdoll, physicalOutput.driveTargets,
                physicalOutput.gravityCompensationEnabled);
            m_physicsScene->setRagdollAnimationConstraint(
                instance.physicsRagdoll, physicalOutput.guide);
            if (physicalOutput.balanceForceWorld.lengthSquared() > 0.000001f
                || physicalOutput.balanceTorqueWorld.lengthSquared()
                    > 0.000001f) {
                m_physicsScene->applyRagdollControlRootForce(
                    instance.physicsRagdoll,
                    physicalOutput.balanceForceWorld,
                    physicalOutput.balanceTorqueWorld);
            }
            continue;
        }
        // E4 (parado pelas pernas) e E5 hibrida (a animacao anda com os pes
        // conscientes e um envelope unico de ajuda; parado, a E4 com a
        // parcela escolhida - harness XE5ANIM=1).
        const bool e5 = m_legsWalkingE5;
        const bool legsStanding = m_legsOnlyStanding || e5;
        instance.locomotion.footworkSettings().recoveryCaptureTargeting = legsStanding;
        instance.locomotion.footworkSettings().locomotionStepping = false;
        instance.locomotion.footworkSettings().locomotionPlacementPredictor = false;
        instance.adaptivePhysics.settings().jointSupport = legsStanding;
        instance.adaptivePhysics.settings().jointBalance = legsStanding;
        // Na E5 a postura fica na pelve (o par pelve x pes) e o peso com a pelve
        // e os motores: pelos tornozelos/juntas, somados a ajuda, o corpo
        // tremia parado (oscilacao vertical, pes perdendo o chao). As pernas
        // fazem o equilibrio (centro de pressao, passos de captura).
        instance.adaptivePhysics.settings().jointPosture = m_legsOnlyStanding
            && m_legsPostureStanding;
        instance.adaptivePhysics.settings().legsWeightFraction = e5 ? 0.0f : 1.0f;
        // A 10% de ajuda, a marcha precisa produzir momento pelo apoio real:
        // as pernas fazem a aceleracao planar limitada pelo centro de
        // pressao; a pelve conserva apenas o envelope auxiliar escolhido.
        instance.adaptivePhysics.settings().jointWhileWalking = e5;
        instance.adaptivePhysics.settings().v2 = m_physicalLocomotionV2;
        command.physicalV2 = m_physicalLocomotionV2;
        instance.adaptivePhysics.settings().legsAssistRetained = e5
            ? std::clamp(m_e5StandingAssistPercent * 0.01f, 0.0f, 1.0f) : 0.0f;
        instance.adaptivePhysics.settings().walkingAssistScale = e5
            ? std::clamp(m_e5WalkingAssistPercent * 0.01f, 0.0f, 1.0f) : 1.0f;
        // Voltando do fisico ao padrao: a ajuda sobe gradativamente ate a
        // cheia so na transicao (pedido do usuario, 05/10) - o corpo fisico
        // vai a pose da animacao antes de ela assumir.
        {
            // Depois de um impacto (e do carrinho levado): a ajuda escolhida
            // vezes o quanto ja voltou.
            const float level = instance.impactAssist.level();
            auto& physical = instance.adaptivePhysics.settings();
            physical.legsAssistRetained *= level;
            physical.walkingAssistScale *= level;
        }
        if (instance.returnBoost > 0.0f) {
            const float boost = instance.returnBoost * instance.returnBoost * (3.0f - 2.0f * instance.returnBoost);
            auto& physical = instance.adaptivePhysics.settings();
            physical.legsAssistRetained += (1.0f - physical.legsAssistRetained) * boost;
            physical.walkingAssistScale += (1.0f - physical.walkingAssistScale) * boost;
        }
        // Cambaleio (do tick anterior): a ajuda cai com o tonus e volta com
        // ele; amolecido, ele nao segue o comando de andar (volta junto).
        command.staggerRecovery = m_staggerRecovery;
        {
            // (So no fisico: no padrao a locomocao fisica nao roda e o valor
            // ficaria parado no ultimo.)
            const float stagger = instance.bodyMode == SpawnedRagdollInstance::BodyMode::Physical
                ? instance.locomotion.telemetry().stagger : 0.0f;
            // (A ajuda nao cai mais pelo cambaleio: cai pelo impacto, acima.)
            if (stagger > 0.0f)
                command.intent.requestedVelocityWorld = command.intent.requestedVelocityWorld * (1.0f - stagger);
            // O chute correndo freia um pouco (a escala do manejo, do tick
            // anterior; rework de 06/10).
            if (command.controlled)
                command.intent.requestedVelocityWorld = command.intent.requestedVelocityWorld * instance.ballSpeedScale;
        }
        command.balanceFootPlacement = e5 && m_e5FootPlacement;
        command.walkingAssist = instance.adaptivePhysics.settings().walkingAssistScale;
        command.standingAssist = instance.adaptivePhysics.settings().legsAssistRetained;
        command.getUpAssist = std::clamp(m_getUpAssistPercent * 0.01f, 0.0f, 2.0f);
        {
            // Pancada enquanto levanta (pedido do usuario, 05/10 noite): a
            // ajuda do levantar cai a m_getUpHitAssistPercent e volta em
            // m_getUpHitRecoverSeconds - ele pode cair de novo.
            const CharacterLocomotionState3D now = instance.locomotion.telemetry().state;
            const bool rising = now == CharacterLocomotionState3D::GettingUp || now == CharacterLocomotionState3D::Fallen;
            float impulse = 0.0f;
            for (const RagdollInteraction3D& contact : instance.physicsState.interactions)
                if (contact.otherMotion != RagdollContactMotion3D::Static) impulse += contact.normalImpulseNewtonSeconds;
            if (rising && impulse > 3.0f && m_getUpHitWeakens) {
                if (instance.getUpHitRecovery >= 1.0f) Log::info("corpo: pancada no levantar (ajuda baixa)");
                instance.getUpHitRecovery = 0.0f;
            } else {
                instance.getUpHitRecovery = std::min(1.0f, instance.getUpHitRecovery
                    + deltaTime / std::max(0.1f, m_getUpHitRecoverSeconds));
            }
            if (instance.getUpHitRecovery < 1.0f) {
                const float x = instance.getUpHitRecovery;
                const float low = std::clamp(m_getUpHitAssistPercent * 0.01f, 0.0f, 1.0f);
                command.getUpAssist *= low + (1.0f - low) * x * x * (3.0f - 2.0f * x);
            }
        }
        command.getUpStiffness = std::clamp(m_getUpStiffnessPercent * 0.01f, 0.2f, 1.5f);
        command.getUpStrength = std::clamp(m_getUpStrengthPercent * 0.01f, 0.2f, 1.5f);
        // (O espaco segurado tambem e pressa para levantar.)
        command.getUpHurry = command.controlled && m_characterSpaceHeld;
        command.tackleEnabled = m_tackleEnabled;
        // A voadora tambem no fisico (06/10), pela mesma opcao do padrao.
        command.airTackleEnabled = m_defaultAirTackle;
        command.motionVariety = m_motionVariety ? m_motionVarietyPercent * 0.01f : 0.0f;
        command.footSpring = m_footSpring;
        command.ankleBalance = m_ankleBalance;
        command.varietySeed = static_cast<std::uint32_t>(instance.entityId * 0x9E3779B97F4A7C15ull >> 32);
        // Locomocao padrao: o modo do corpo do personagem controlado (ver
        // SpawnedRagdollInstance::BodyMode).
        using BodyMode = SpawnedRagdollInstance::BodyMode;
        const bool defaultMode = m_defaultLocomotion && command.controlled && adaptive
            && m_physicsScene->hasCharacter();
        {
            const auto toPhysical = [&](const char* why) {
                if (instance.bodyMode != BodyMode::Physical)
                    Log::info(std::string("corpo: modo fisico (") + why + ")");
                if (instance.guided) {
                    // O corpo guiado ja e o fisico: so as juntas, a gravidade
                    // e as colisoes voltam. Sem contato (pulo fisico, padrao
                    // desligado), a pose da animacao segue segurando-o por
                    // m_defaultGuidedExitSeconds com a autoridade descendo
                    // (o padrao roda junto).
                    m_physicsScene->setRagdollGuided(instance.physicsRagdoll, false);
                    m_physicsScene->removeRagdollVerticalSpin(instance.physicsRagdoll);
                    instance.guided = false;
                    // (So sem contato - o pulo fisico, o padrao desligado: num
                    // choque, a pose seguraria o embalo e empurraria o corpo
                    // contra o que ele acertou.)
                    const std::string reason(why);
                    instance.guidedExitSeconds = (reason == "pulo" || reason == "padrao desligado")
                        && !instance.defaultLocomotion.output().tackling ? m_defaultGuidedExitSeconds : 0.0f;
                    instance.guidedExitLeft = instance.guidedExitSeconds;
                }
                // O controle fisico comeca do corpo como esta (com o embalo
                // da animacao: os links saem com a velocidade que tinham).
                if (instance.bodyMode != BodyMode::Physical) {
                    instance.locomotion.reset(*m_humanRagdollProfile, instance.physicsState);
                    instance.adaptivePhysics.reset(*m_humanRagdollProfile);
                    // (Do pulo, ou de um esbarrao no ar: o pouso e fisico e a
                    // volta vem ~1 s depois dele.)
                    instance.physicalFromJump = std::string(why) == "pulo"
                        || !m_physicsScene->characterState().grounded;
                    if (m_defaultSeedFromBody) {
                        // O fisico continua a marcha de onde o padrao estava
                        // (fase pelos pes, sem a arrancada).
                        CharacterLocomotionSeed3D seed;
                        const Vec3 body = m_physicsScene->characterState().velocity;
                        seed.moving = std::hypot(body.x, body.y) > 0.3f
                            || command.intent.requestedVelocityWorld.lengthSquared() > 0.01f;
                        // No meio do carrinho (a pancada nele mesmo, ao
                        // acertar alguem): o fisico continua o carrinho, com a
                        // ajuda normal (pedido do usuario, 05/10 noite).
                        const DefaultLocomotion3D::Tackle tackle = instance.defaultLocomotion.tackle();
                        if (tackle.clip != nullptr) {
                            seed.tackleClip = tackle.clip;
                            seed.tackleTime = tackle.time;
                            seed.tackleRate = tackle.rate;
                            seed.tackleScale = tackle.scale;
                            seed.tackleYaw = tackle.yaw;
                            seed.tackleEntrySpeed = tackle.entrySpeed;
                            seed.tackleStart = tackle.start;
                        }
                        instance.locomotion.seedMotion(*m_humanRagdollProfile, instance.physicsState, seed);
                    }
                }
                instance.returnBoost = 0.0f;
                instance.landedSeconds = 0.0f;
                instance.bodyMode = BodyMode::Physical;
                instance.bodyModeSeconds = 0.0f;
                instance.calmSeconds = 0.0f;
            };
            if (!defaultMode) {
                toPhysical("padrao desligado");
            } else {
                const PhysicsCharacterState3D& capsule = m_physicsScene->characterState();
                // Pancada: contato com algo que se move (outro boneco, objeto)
                // acima de DefaultHitImpulse, ou agarrado.
                float hitImpulse = 0.0f;
                for (const RagdollInteraction3D& contact : instance.physicsState.interactions)
                    if (contact.otherMotion != RagdollContactMotion3D::Static)
                        hitImpulse += contact.normalImpulseNewtonSeconds;
                constexpr float DefaultHitImpulse = 4.0f;
                const bool hit = hitImpulse > DefaultHitImpulse || grabbed;
                // No carrinho do padrao (o do tick anterior), qualquer impacto
                // leva ao fisico - um boneco ou uma parede (pedido do usuario,
                // 05/10 noite). A voadora tambem: animada ate encostar em algo
                // (06/10: "so alternar para o fisico no contato", mais
                // fluido); com m_defaultVoadoraContactOnly desligado, ela vai
                // ao fisico ja no comeco, como antes.
                const DefaultLocomotionOutput3D& defaultBefore = instance.defaultLocomotion.output();
                const bool defaultTackling = instance.bodyMode != BodyMode::Physical && defaultBefore.tackling;
                constexpr float TackleHitImpulse = 0.4f;
                const bool tackleHit = defaultTackling && hitImpulse > TackleHitImpulse;
                const bool airTackle = defaultTackling && defaultBefore.airTackle && !m_defaultVoadoraContactOnly;
                // Parede, meio-fio, frente de mureta: o corpo guiado bate de
                // verdade (contato com o que nao se move e nao e chao) e, acima
                // de WallHitImpulse, vai ao fisico na hora com o choque (pedido
                // do usuario, 05/10 noite: como no modo fisico).
                float wallImpulse = 0.0f;
                for (const RagdollInteraction3D& contact : instance.physicsState.interactions)
                    if (contact.otherMotion != RagdollContactMotion3D::Dynamic && contact.normalWorld.z < 0.6f)
                        wallImpulse += contact.normalImpulseNewtonSeconds;
                constexpr float WallHitImpulse = 2.5f;
                const bool wallHit = instance.bodyMode != BodyMode::Physical
                    && wallImpulse > (defaultTackling ? TackleHitImpulse : WallHitImpulse);
                instance.bodyModeSeconds += deltaTime;
                if (instance.bodyMode == BodyMode::Physical) {
                    // De volta ao padrao com o corpo assentado: de pe, no chao,
                    // sem pancada, por um tempo.
                    const CharacterLocomotionState3D now = instance.locomotion.telemetry().state;
                    const bool standingState = now == CharacterLocomotionState3D::Idle
                        || now == CharacterLocomotionState3D::Turning || now == CharacterLocomotionState3D::Walking
                        || now == CharacterLocomotionState3D::Running
                        || (instance.physicalFromJump && now == CharacterLocomotionState3D::Landing);
                    // (Encostado numa parede - contato com o estatico que nao e
                    // chao - ele fica no fisico: voltando, os membros entravam
                    // nela de novo.)
                    bool againstWall = false;
                    for (const RagdollInteraction3D& contact : instance.physicsState.interactions)
                        if (contact.otherMotion == RagdollContactMotion3D::Static && contact.normalWorld.z < 0.6f)
                            againstWall = true;
                    const PhysicsBodyState3D& pelvis = instance.physicsState.links.front();
                    const Vec3 pelvisUp = (pelvis.orientation
                        * m_humanRagdollProfile->links.front().modelOrientation.conjugate()).rotate({ 0.0f, 0.0f, 1.0f });
                    const bool upright = pelvisUp.z > 0.85f;
                    const bool calm = standingState && upright && capsule.grounded && !hit && !againstWall
                        && instance.locomotion.telemetry().stagger < 0.1f
                        && instance.adaptivePhysics.output().perturbation < (instance.physicalFromJump ? 0.6f : 0.25f);
                    instance.calmSeconds = calm ? instance.calmSeconds + deltaTime : 0.0f;
                    // (Do pulo: ~1 s de fisica depois do pouso - pedido do
                    // usuario, 05/10 -, contado do toque, nao da decolagem.)
                    instance.landedSeconds = capsule.grounded ? instance.landedSeconds + deltaTime : 0.0f;
                    // (Levantou com pressa - as teclas: volta logo, ele sai
                    // andando/correndo no padrao.)
                    const bool hurried = instance.locomotion.telemetry().getUpHurried;
                    const float minimumSeconds = instance.physicalFromJump || hurried ? 0.0f : 0.8f;
                    const float calmSeconds = hurried ? 0.08f : instance.physicalFromJump ? 0.15f : 0.4f;
                    const bool landedLongEnough = !instance.physicalFromJump || instance.landedSeconds > 1.0f;
                    // Pronto para voltar: a ajuda sobe ate a cheia em
                    // ReturnBoostSeconds e so entao ele volta (se deixar de
                    // estar assentado no meio, a ajuda desce de novo).
                    const float ReturnBoostSeconds = hurried ? 0.15f : 0.4f;
                    // (E com a ajuda de volta inteira depois do impacto - 06/10.)
                    const bool ready = instance.bodyModeSeconds > minimumSeconds
                        && instance.calmSeconds > calmSeconds && landedLongEnough
                        && !instance.impactAssist.recovering();
                    instance.returnBoost = ready
                        ? std::min(1.0f, instance.returnBoost + deltaTime / ReturnBoostSeconds)
                        : std::max(0.0f, instance.returnBoost - deltaTime / 0.2f);
                    // Levantando com pressa (as teclas), ja sobre os pes: o
                    // padrao assume ja, andando para onde elas pedem - o
                    // resto do levantar sai pela inercia, mais longa (pedido
                    // do usuario, 05/10 noite: "comecar a levantar e ja ir
                    // indo para a direcao que quero").
                    // (So com Shift - correndo; so com as teclas, o levantar
                    // rapido e a volta logo ao padrao. O Shift pela tecla: o
                    // m_characterSprinting so liga com a capsula andando, e
                    // caido ela esta parada embaixo do corpo.)
                    const bool shiftHeld = command.intent.effort > 0.5f;
                    const bool handoff = m_defaultGetUpHandoff && m_defaultSeedFromBody && !hit
                        && shiftHeld && instance.locomotion.telemetry().getUpHandoffReady;
                    if (handoff || (ready && instance.returnBoost >= 1.0f)) {
                        instance.returnBoost = 0.0f;
                        // A volta: a pose fisica de agora (raiz em relacao a base
                        // da capsula, juntas) misturada ate a animada.
                        const Vec3 feet = command.rootPositionWorld
                            - Vec3 { 0.0f, 0.0f, m_humanRagdollProfile->standingRootHeightMeters };
                        instance.returnRootOffset = pelvis.position - feet;
                        instance.returnRootOrientation = pelvis.orientation;
                        instance.returnCoordinates.assign(m_humanRagdollProfile->links.size(), {});
                        for (std::size_t i = 1; i < instance.returnCoordinates.size()
                                && i < instance.physicsState.joints.size(); ++i) {
                            const auto& joint = instance.physicsState.joints[i].positionRadians;
                            instance.returnCoordinates[i] = { joint[0], joint[1], joint[2] };
                        }
                        instance.defaultLocomotion.reset(m_characterDesiredFacingYaw);
                        if (m_defaultSeedFromBody) {
                            // O padrao comeca de onde o corpo esta: pelve no
                            // rumo dela, na velocidade e na fase dos pes.
                            DefaultLocomotionSeed3D seed;
                            const Vec3 forward = (pelvis.orientation
                                * m_humanRagdollProfile->links.front().modelOrientation.conjugate()).rotate({ 1.0f, 0.0f, 0.0f });
                            seed.pelvisYawRadians = std::atan2(forward.y, forward.x);
                            seed.velocityWorld = capsule.velocity;
                            seed.pelvisWorld = pelvis.position;
                            std::size_t feet[2] = { 0, 0 };
                            for (std::size_t i = 0; i < m_humanRagdollProfile->links.size(); ++i) {
                                if (m_humanRagdollProfile->links[i].id == "LeftFoot") feet[0] = i;
                                if (m_humanRagdollProfile->links[i].id == "RightFoot") feet[1] = i;
                            }
                            seed.feetValid = feet[0] != 0 && feet[1] != 0
                                && feet[0] < instance.physicsState.links.size() && feet[1] < instance.physicsState.links.size();
                            seed.pelvisVelocityWorld = pelvis.linearVelocity;
                            if (seed.feetValid)
                                for (std::size_t side = 0; side < 2; ++side) {
                                    seed.feetWorld[side] = instance.physicsState.links[feet[side]].position;
                                    seed.feetVelocityWorld[side] = instance.physicsState.links[feet[side]].linearVelocity;
                                }
                            instance.defaultLocomotion.seed(seed);
                            // E a pose sai pela inercia: a diferenca para a
                            // animacao, com a velocidade que o corpo tinha.
                            std::vector<Vec3> velocities(instance.returnCoordinates.size());
                            for (std::size_t i = 1; i < velocities.size() && i < instance.physicsState.joints.size(); ++i) {
                                const auto& joint = instance.physicsState.joints[i].velocityRadiansPerSecond;
                                velocities[i] = { joint[0], joint[1], joint[2] };
                            }
                            instance.returnInertia.start(instance.returnCoordinates, velocities, pelvis.position,
                                pelvis.orientation, pelvis.linearVelocity, pelvis.angularVelocity,
                                handoff ? GetUpHandoffHalfLife : 0.10f, handoff ? GetUpHandoffSeconds : 0.45f);
                        } else {
                            instance.returnInertia.stop();
                        }
                        m_physicsScene->setRagdollGuided(instance.physicsRagdoll, true);
                        instance.guided = true;
                        instance.guidedExitLeft = 0.0f;
                        instance.bodyMode = BodyMode::Returning;
                        instance.bodyModeSeconds = 0.0f;
                        instance.airSeconds = 0.0f;
                        if (handoff) {
                            // Do agachado, a arrancada (Crouched To Sprinting)
                            // para onde as teclas pedem.
                            const Vec3 keys = command.intent.requestedVelocityWorld;
                            const float travelYaw = keys.lengthSquared() > 1e-4f ? std::atan2(keys.y, keys.x)
                                : m_characterDesiredFacingYaw;
                            instance.defaultLocomotion.startCrouchSprint(travelYaw, true);
                        }
                        Log::info(handoff ? "corpo: levantando - arrancada agachada (padrao)" : "corpo: voltando ao padrao");
                    }
                } else {
                    // Animado: a fisica entra quando ele bate em algo fisico
                    // (pedidos do usuario, 04-05/10) - outro boneco ou objeto
                    // (pancada), a capsula batendo forte (parede), um membro
                    // entrando no mundo estatico (parede, meio-fio: o pe que
                    // tropeca) - e no pulo (o voo e o pouso com fisica; volta
                    // logo depois do pouso).
                    constexpr float WallImpact = 1.5f;
                    const bool wall = m_controlledWallImpact > WallImpact;
                    bool penetrating = false;
                    {
                        std::vector<Vec3> positions(instance.physicsState.links.size());
                        std::vector<Quaternion> orientations(instance.physicsState.links.size());
                        for (std::size_t i = 0; i < positions.size(); ++i) {
                            positions[i] = instance.physicsState.links[i].position;
                            orientations[i] = instance.physicsState.links[i].orientation;
                        }
                        // (No carrinho, qualquer encostada: 5 mm.)
                        const float PenetrationDepth = defaultTackling ? 0.005f : 0.015f;
                        for (const RagdollWorldContact3D& contact : m_physicsScene->ragdollPoseWorldContacts(
                                instance.physicsRagdoll, positions, orientations, PenetrationDepth))
                            // (O chao - normal para cima - nao conta.)
                            if (contact.normalWorld.z < 0.6f) penetrating = true;
                    }
                    // Pulo: o voo e o pouso no fisico, como no modo fisico (pedido
                    // do usuario, 05/10: "la e perfeito"). A decolagem fica
                    // animada por JumpTakeoffSeconds: trocando no primeiro tick
                    // no ar, o corpo ainda estava parado no chao, nao levava o
                    // impulso e a capsula voltava presa a ele - o boneco nao
                    // pulava. Alguns ticks depois o corpo cinematico ja sobe com
                    // a capsula e, dinamico, segue com esse embalo. Caindo sem
                    // pulo (de um degrau), o voo animado e a fisica no pouso.
                    // Pulo no padrao de novo (pedido do usuario, 05/10 noite:
                    // "para melhor estabilidade"): voo e pouso animados; so um
                    // esbarrao no ar (as pancadas acima) o leva ao fisico, e
                    // ai o pouso tambem e fisico. "Pulo físico" no
                    // laboratorio volta ao de antes.
                    constexpr float JumpTakeoffSeconds = 0.05f;
                    instance.airSeconds = capsule.grounded ? instance.airSeconds : instance.airSeconds + deltaTime;
                    const bool jumped = m_defaultPhysicalJump
                        && ((!capsule.grounded && capsule.velocity.z > 1.0f
                            && instance.airSeconds >= JumpTakeoffSeconds)
                        || (capsule.grounded && instance.airSeconds > 0.25f));
                    if (capsule.grounded) instance.airSeconds = 0.0f;
                    if (hit || tackleHit || airTackle || wall || wallHit || penetrating || jumped)
                        toPhysical(grabbed ? "agarrado" : airTackle ? "voadora" : hit || tackleHit ? "pancada"
                            : wall || wallHit ? "parede" : penetrating ? "membro no mundo" : jumped ? "pulo" : "pancada");
                }
            }
        }
        const bool animated = defaultMode && instance.bodyMode != BodyMode::Physical;
        if (command.controlled) m_controlledAnimated = animated;
        // Manejo de bola (o personagem controlado): a bola mais perto, ate
        // 3 m. O pedido (mouse) sai uma vez; o drible vai para as teclas (sem
        // tecla, para onde olha), o chute para onde a camera olha.
        if (command.controlled && !instance.physicsState.links.empty()) {
            const Vec3 pelvis = instance.physicsState.links.front().position;
            SpawnedInflatedBallInstance* nearest = nullptr;
            float best = 3.0f;
            for (auto& ball : m_inflatedBalls) {
                if (!m_physicsScene->contains(ball.physicsBall)) continue;
                const float distance = (ball.physicsState.position - pelvis).length();
                if (distance < best) { best = distance; nearest = &ball; }
            }
            BallControlInput3D ballInput;
            ballInput.request = m_ballRequest;
            ballInput.power = m_ballRequestPower;
            m_ballRequest = BallActionKind3D::None;
            const float look = command.facingYawRadians;
            const Vec3 keys = command.intent.requestedDirectionWorld;
            ballInput.direction = keys.lengthSquared() > 1e-4f ? keys : Vec3 { std::cos(look), std::sin(look), 0.0f };
            ballInput.lookYawRadians = look;
            if (ballInput.request == BallActionKind3D::Shot) {
                ballInput.lookYawRadians = m_ballRequestAimYaw;
                ballInput.shotLift = m_ballRequestLift;
                ballInput.shotCurve = m_ballRequestCurve;
            }
            // Interagindo (barra carregando, pedido, toque em andamento): ele
            // vira o dono da bola - sem colisao com ela ja, mesmo sem encostar.
            if (nearest != nullptr
                && (m_ballShotHeld || m_ballDribbleHeld || m_ballRequest != BallActionKind3D::None
                    || ballInput.request != BallActionKind3D::None
                    || instance.ballControl.phase() != BallControlPhase3D::Idle)) {
                nearest->ownership.interact(instance.entityId);
                m_physicsScene->setInflatedBallOwner(nearest->physicsBall, instance.physicsRagdoll);
            }
            if (nearest != nullptr) {
                const InflatedBallProfile3D ballProfile = m_physicsScene->inflatedBallProfile(nearest->physicsBall);
                ballInput.ballValid = true;
                ballInput.ballPosition = nearest->physicsState.position;
                ballInput.ballVelocity = nearest->physicsState.linearVelocity;
                ballInput.ballRadius = ballProfile.radiusMeters;
                ballInput.ballMass = ballProfile.massKg;
            }
            ballInput.profile = &*m_humanRagdollProfile;
            ballInput.state = &instance.physicsState;
            ballInput.facingYawRadians = look;
            ballInput.groundHeight = ragdollGroundHeight(pelvis);
            ballInput.requestedVelocity = command.intent.requestedVelocityWorld;
            // A velocidade do corpo: no padrao, a da capsula (a da pelve tem o
            // balanco da passada e o proprio gesto); no fisico, a da pelve (a
            // capsula vai a frente do corpo que atrasa, e o gesto nao mexe
            // nela).
            ballInput.bodyVelocity = { command.rootVelocityWorld.x, command.rootVelocityWorld.y, 0.0f };
            ballInput.bodyVelocityValid = animated;
            ballInput.contactTimingFactor = std::clamp(m_ballContactTiming, 0.0f, 1.0f);
            ballInput.hitboxContact = m_ballHitboxContact;
            ballInput.force = &m_ballForce;
            ballInput.forceOnly = m_ballForceOnly;
            ballInput.bodyYawValid = true;
            ballInput.animatedBody = animated;
            if (animated) {
                // Os pes pela animacao do modo padrao.
                const DefaultLocomotionOutput3D& feet = instance.defaultLocomotion.output();
                ballInput.bodyYawRadians = feet.facingYawRadians;
                for (std::size_t side = 0; side < 2; ++side) {
                    ballInput.swingProgress[side] = feet.swingProgress[side];
                    ballInput.footPlanted[side] = feet.footPlanted[side];
                    ballInput.secondsToTouchdown[side] = feet.secondsToTouchdown[side];
                }
                const Vec3 body = m_physicsScene->characterState().velocity;
                ballInput.standing = std::hypot(body.x, body.y) < 0.3f
                    && command.intent.requestedVelocityWorld.lengthSquared() < 0.01f;
            } else {
                ballInput.bodyYawRadians = instance.locomotion.telemetry().facingYawRadians;
                for (std::size_t side = 0; side < 2; ++side) {
                    ballInput.swingProgress[side] = instance.locomotion.telemetry().footSwingProgress[side];
                    ballInput.footPlanted[side] = instance.locomotion.telemetry().footPlanted[side];
                    ballInput.footLanding[side] = instance.locomotion.telemetry().footLandingWorld[side];
                    ballInput.secondsToTouchdown[side] = instance.locomotion.telemetry().footSecondsToTouchdown[side];
                }
                const CharacterLocomotionState3D now = instance.locomotion.telemetry().state;
                ballInput.standing = (now == CharacterLocomotionState3D::Idle
                    || now == CharacterLocomotionState3D::Turning)
                    && command.intent.requestedVelocityWorld.lengthSquared() < 0.01f;
            }
            ballInput.deltaTime = deltaTime;
            BallControlOutput3D ballOutput;
            instance.ballControl.update(ballInput, ballOutput);
            instance.ballSpeedScale = ballOutput.speedScale;
            instance.ballLastOutput = ballOutput;
            // O reforco de estabilidade do gesto (modo fisico; o controle o
            // corta com contato de fora - aqui, tambem na volta de um impacto).
            instance.ballStability = m_ballStabilityAssist && !instance.impactAssist.recovering()
                ? ballOutput.stability : 0.0f;
            command.ballGesture = ballOutput.gesture;
            command.ballFoot = ballOutput.foot;
            command.ballScript = ballOutput.script;
            command.ballBody = ballOutput.body;
            command.ballRepel = ballOutput.repel;
            command.ballAvoid = ballOutput.avoid;
            command.ballPlant = ballOutput.plant;
            command.ballGaitRate = ballOutput.gaitRate;
            command.ballKick = ballOutput.kick;
            if (ballOutput.setMotion && nearest != nullptr)
                m_physicsScene->setInflatedBallMotion(nearest->physicsBall, nearest->physicsState.position,
                    ballOutput.ballVelocity, ballOutput.ballSpin);
        }
        // A entrada do modo padrao (capsula, teclas, manejo) e os ajustes do
        // laboratorio; tambem na saida do corpo guiado, com o padrao rodando
        // junto do fisico.
        const auto defaultLocomotionInput = [&]() {
            const PhysicsCharacterState3D& capsule = m_physicsScene->characterState();
            DefaultLocomotionInput3D input;
            input.feetPositionWorld = command.rootPositionWorld
                - Vec3 { 0.0f, 0.0f, m_humanRagdollProfile->standingRootHeightMeters };
            input.velocityWorld = capsule.velocity;
            input.grounded = capsule.grounded;
            input.requestedVelocityWorld = command.intent.requestedVelocityWorld;
            input.facingYawRadians = m_characterDesiredFacingYaw;
            input.lookPitchRadians = command.lookPitchRadians;
            input.groundAt = command.groundAt;
            input.deltaTime = deltaTime;
            input.jumpCrouch = command.jumpCrouch;
            input.tackleRequest = command.intent.crouch && !instance.crouchHeld;
            input.tackleEnabled = m_tackleEnabled;
            // A voadora: Ctrl com Shift + W no ar (W: para a frente do olhar).
            input.tackleHeld = command.intent.crouch;
            {
                const Vec3 keys = command.intent.requestedDirectionWorld;
                const float forward = keys.x * std::cos(m_characterDesiredFacingYaw)
                    + keys.y * std::sin(m_characterDesiredFacingYaw);
                input.airTackleCombo = command.intent.effort > 0.5f && forward > 0.7f;
            }
            instance.crouchHeld = command.intent.crouch;
            input.ballGesture = command.ballGesture;
            input.ballFoot = command.ballFoot;
            input.ballScript = command.ballScript;
            input.ballKick = command.ballKick;
            input.motionVariety = command.motionVariety;
            input.varietySeed = command.varietySeed;
            instance.defaultLocomotion.settings().adaptiveTransitions = m_defaultAdaptiveTransitions;
            instance.defaultLocomotion.settings().toeRoll = m_toeRoll;
            instance.defaultLocomotion.settings().smoothClipSampling = m_defaultAdaptiveTransitions;
            instance.defaultLocomotion.settings().trunkMotion = m_defaultTrunkMotion;
            instance.defaultLocomotion.settings().landingAbsorb = m_defaultLandingAbsorb;
            instance.defaultLocomotion.settings().curveLeanDeadZone = m_defaultCurveLeanDeadZone;
            instance.defaultLocomotion.settings().airTackle = m_defaultAirTackle;
            instance.defaultLocomotion.settings().airTacklePitchDegrees = m_defaultAirTacklePitch;
            {
                // As velocidades da aba "Velocidades" (0, 45, 90, 135 e 180 graus).
                const float t = m_defaultTrotSpeed;
                instance.defaultLocomotion.settings().trotSpeeds = { t, t, t, t, t };
                instance.defaultLocomotion.settings().sprintSpeeds = { m_defaultSprintForwardSpeed,
                    m_defaultSprintForwardSpeed, m_defaultSprintSideSpeed, m_defaultSprintBackwardSpeed,
                    m_defaultSprintBackwardSpeed };
            }
            instance.defaultLocomotion.settings().trunkMotionAmount = m_defaultTrunkMotionPercent * 0.01f;
            return input;
        };
        if (!animated && instance.guidedExitLeft > 0.0f) {
            // Saida do corpo guiado: a pose do padrao com a autoridade
            // descendo enquanto os motores do fisico assumem.
            const DefaultLocomotionInput3D input = defaultLocomotionInput();
            instance.defaultLocomotion.update(*m_humanRagdollProfile, defaultClips, input);
            const DefaultLocomotionOutput3D& out = instance.defaultLocomotion.output();
            const float x = std::clamp(instance.guidedExitLeft / std::max(1e-3f, instance.guidedExitSeconds), 0.0f, 1.0f);
            const std::vector<float> authority(m_humanRagdollProfile->links.size(), x * x * (3.0f - 2.0f * x));
            if (out.valid)
                m_physicsScene->setRagdollGuidePose(instance.physicsRagdoll, out.pose.linkPositions,
                    out.pose.linkOrientations, authority);
            instance.guidedExitLeft -= deltaTime;
            // (No fim, de novo sem o giro em volta da vertical: a pose ainda
            // o dava - no ar, o voo girava ate 60 graus.)
            if (instance.guidedExitLeft <= 0.0f) m_physicsScene->removeRagdollVerticalSpin(instance.physicsRagdoll);
        }
        if (animated) {
            // Modo padrao: a pose da animacao sobre a capsula (o corpo guiado
            // chega nela no fim do passo). Na volta do fisico, a pose fisica
            // do comeco (levada com a capsula) misturada ate ela.
            if (!instance.guided) {
                m_physicsScene->setRagdollGuided(instance.physicsRagdoll, true);
                instance.guided = true;
            }
            const DefaultLocomotionInput3D input = defaultLocomotionInput();
            instance.defaultLocomotion.update(*m_humanRagdollProfile, defaultClips, input);
            const DefaultLocomotionOutput3D& out = instance.defaultLocomotion.output();
            // Carrinho: a capsula vai embaixo da pelve do clipe; no fim sai
            // com o embalo dela.
            if (out.rootScripted) {
                m_physicsScene->placeCharacter({ out.tackleRootWorld.x, out.tackleRootWorld.y,
                    ragdollGroundHeight(out.tackleRootWorld) }, m_avatarCharacterSettings);
            }
            if (out.tackleEnded)
                m_physicsScene->setCharacterVelocity({ out.tackleVelocity.x, out.tackleVelocity.y, 0.0f });
            if (out.tackling && !instance.defaultTackling) Log::info("corpo: carrinho (padrao)");
            instance.defaultTackling = out.tackling;
            if (out.valid) {
                RagdollAnimationPose3D pose = out.pose;
                if (instance.bodyMode == BodyMode::Returning && instance.returnInertia.active()) {
                    std::vector<Vec3> coordinates = out.coordinates;
                    Vec3 rootPosition = out.rootPosition;
                    Quaternion rootOrientation = out.rootOrientation;
                    instance.returnInertia.apply(coordinates, rootPosition, rootOrientation, deltaTime);
                    defaultLocomotionPose3D(*m_humanRagdollProfile, rootPosition, rootOrientation, coordinates, pose);
                    if (!instance.returnInertia.active()) {
                        instance.bodyMode = BodyMode::Animated;
                        instance.bodyModeSeconds = 0.0f;
                    }
                } else if (instance.bodyMode == BodyMode::Returning) {
                    constexpr float ReturnSeconds = 0.35f;
                    const float x = std::clamp(instance.bodyModeSeconds / ReturnSeconds, 0.0f, 1.0f);
                    const float share = x * x * (3.0f - 2.0f * x);
                    std::vector<Vec3> coordinates = out.coordinates;
                    for (std::size_t i = 0; i < coordinates.size() && i < instance.returnCoordinates.size(); ++i)
                        coordinates[i] = instance.returnCoordinates[i]
                            + (coordinates[i] - instance.returnCoordinates[i]) * share;
                    const Vec3 from = input.feetPositionWorld + instance.returnRootOffset;
                    Quaternion fromOrientation = instance.returnRootOrientation;
                    const Quaternion to = out.rootOrientation;
                    if (fromOrientation.x * to.x + fromOrientation.y * to.y + fromOrientation.z * to.z
                            + fromOrientation.w * to.w < 0.0f)
                        fromOrientation = { -fromOrientation.x, -fromOrientation.y, -fromOrientation.z, -fromOrientation.w };
                    const Quaternion rootOrientation = Quaternion {
                        fromOrientation.x + (to.x - fromOrientation.x) * share,
                        fromOrientation.y + (to.y - fromOrientation.y) * share,
                        fromOrientation.z + (to.z - fromOrientation.z) * share,
                        fromOrientation.w + (to.w - fromOrientation.w) * share }.normalized();
                    defaultLocomotionPose3D(*m_humanRagdollProfile, from + (out.rootPosition - from) * share,
                        rootOrientation, coordinates, pose);
                    if (x >= 1.0f) {
                        instance.bodyMode = BodyMode::Animated;
                        instance.bodyModeSeconds = 0.0f;
                    }
                }
                m_physicsScene->setRagdollGuidePose(instance.physicsRagdoll, pose.linkPositions,
                    pose.linkOrientations);
                instance.previousAnimationPose = instance.animationPose.linkPositions.empty()
                    ? pose : instance.animationPose;
                instance.animationPose = pose;
                instance.physicsBlend = 0.0f;
            }
            // A capsula manda: nada de ir atras do corpo.
            m_characterFollowVelocity = {};
            continue;
        }
        instance.locomotion.update(*m_humanRagdollProfile, clips,
            instance.physicsState, dynamics, command, deltaTime);
        const auto& output=instance.locomotion.output();
        // Fisica adaptativa: com a opcao ligada, a forca limitada da pelve e
        // aplicada no lugar do transporte pela capsula; desligada, so mede.
        const CharacterLocomotionState3D bodyState =
            instance.locomotion.telemetry().state;
        // (No carrinho tambem: o corpo vai pelo clipe e pela propria ajuda.)
        const bool down = bodyState == CharacterLocomotionState3D::Fallen
            || bodyState == CharacterLocomotionState3D::GettingUp
            || bodyState == CharacterLocomotionState3D::Tackling;
        AdaptivePhysicalIntent3D physicalIntent;
        physicalIntent.proxyVelocityWorld = command.proxyVelocityWorld;
        // Sem a velocidade pedida, o controlador interpretava toda marcha
        // como estado parado: legsWalking ficava falso e walkingAssist era
        // forçado a 1, ignorando completamente o slider "Ajuda andando".
        physicalIntent.requestedVelocityWorld =
            command.intent.requestedVelocityWorld;
        // Freada correndo: a velocidade do perfil e a parcela da ajuda de
        // quem corre (da locomocao).
        physicalIntent.brakeRunVelocityWorld =
            instance.locomotion.telemetry().brakeRunVelocityWorld;
        physicalIntent.brakeRunShare = instance.locomotion.telemetry().brakeRunShare;
        physicalIntent.navigationRootWorld = command.rootPositionWorld;
        physicalIntent.airborne = !command.grounded;
        physicalIntent.manipulated = command.manipulated;
        physicalIntent.suspended = down;
        physicalIntent.postureLost = instance.locomotion.telemetry().postureLost;
        physicalIntent.braceRecovering = instance.locomotion.telemetry().braceRecovering;
        physicalIntent.jumpCharge = command.controlled ? m_laboratoryJumpCharge : 0.0f;
        physicalIntent.footGround = command.footGround;
        // O que a capsula (ou a ancora virtual) andou para ir atras do corpo
        // neste tick nao e intencao do jogador.
        physicalIntent.followVelocityWorld = command.controlled
            ? m_characterFollowVelocity : instance.navigationAnchorVelocity;
        physicalIntent.physicalState = &instance.locomotion.physicalState();
        physicalIntent.contactPlan = &instance.locomotion.contactPlan();
        physicalIntent.targetPose = &instance.locomotion.output().targetPose;
        physicalIntent.footPlantedValid = true;
        physicalIntent.footPlanted = instance.locomotion.telemetry().footPlanted;
        physicalIntent.actionStability = command.controlled ? instance.ballStability : 0.0f;
        instance.adaptivePhysics.update(*m_humanRagdollProfile,
            instance.physicsState, output.guide, physicalIntent, deltaTime);
        const float followSlack = 0.10f * (1.0f - std::clamp(
            instance.adaptivePhysics.output().perturbation, 0.0f, 1.0f));
        if (adaptive && !command.controlled) {
            // A capsula virtual vai atras do corpo; caido, ela fica nele.
            // Comparada com ELA MESMA: no levantar a referencia da locomocao
            // fica presa ao ponto onde o corpo deitou, e comparando com ela a
            // ancora acumulava metros de erro - na saida, o boneco era
            // arremessado atras dela.
            instance.navigationAnchorVelocity = navigationFollowVelocity3D(
                instance.physicsState.links.front().position,
                instance.navigationAnchor, down ? 0.0f : followSlack);
            instance.navigationAnchor +=
                instance.navigationAnchorVelocity * deltaTime;
        }
        if (command.controlled) {
            // A capsula vai atras do corpo quando ele se afasta da pelve de
            // referencia mais de 10 cm (empurrao, tropeco, parede): o corpo
            // e a verdade; depois de um contato externo, sem folga. Com a
            // opcao desligada o corpo e carregado e isso nunca acontece.
            m_characterFollowVelocity = adaptive && !down
                ? navigationFollowVelocity3D(
                    instance.physicsState.links.front().position,
                    output.navigationAnchorWorld, followSlack)
                : Vec3 {};
        }
        instance.previousAnimationPose =
            instance.animationPose.linkPositions.empty()
                ? output.targetPose : instance.animationPose;
        instance.animationPose = output.targetPose;
        instance.physicsBlend = output.physicsBlend;
        // While the body is down, the capsule goes where the body is. It is
        // what the camera follows, and leaving it standing where the fall
        // started is what made the character snap back across the floor the
        // moment a recovery finished. placeCharacter is a real teleport
        // (PxController::setFootPosition underneath) - the earlier note in
        // the devlog claiming no such API existed was simply wrong.
        if (command.controlled && m_physicsScene->hasCharacter()) {
            const CharacterLocomotionState3D locomotionState =
                instance.locomotion.telemetry().state;
            if (locomotionState == CharacterLocomotionState3D::Fallen
                || locomotionState == CharacterLocomotionState3D::GettingUp
                || locomotionState == CharacterLocomotionState3D::Tackling) {
                const Vec3 root = instance.physicsState.links.front().position;
                m_physicsScene->placeCharacter(
                    { root.x, root.y, ragdollGroundHeight(root) },
                    m_avatarCharacterSettings);
                // No carrinho a capsula vai com o embalo do corpo: ao sair
                // dele (correndo), ela continua com essa velocidade.
                if (locomotionState == CharacterLocomotionState3D::Tackling) {
                    const Vec3 velocity = instance.physicsState.links.front().linearVelocity;
                    m_physicsScene->setCharacterVelocity({ velocity.x, velocity.y, 0.0f });
                }
            }
        }
        applyCharacterControl3D(*m_physicsScene, instance.physicsRagdoll,
            output, adaptive ? &instance.adaptivePhysics.output() : nullptr,
            down);
    }
    updateRagdollImpactLab(deltaTime);
}

void WorkbenchApp::updateRagdollImpactLab(float deltaTime) {
    if (!m_physicsScene) return;
    m_ragdollImpactLab.update(deltaTime);
    const RagdollImpactTestOutput3D& impact =
        m_ragdollImpactLab.output();
    if (!impact.applying || impact.forceNewtons <= 0.0f) return;

    const float radians = impact.directionDegrees * Pi / 180.0f;
    for (const SpawnedRagdollInstance& instance : m_spawnedRagdolls) {
        if (instance.kind != ArticulatedRigKind::Human
            || !instance.active
            || instance.physicsState.links.size() <= 3
            || !m_physicsScene->contains(instance.physicsRagdoll)) {
            continue;
        }

        // A direcao acompanha o corpo, nao a camera nem os eixos do mapa.
        // Assim 90 graus continua sendo o lado esquerdo mesmo depois que o
        // ragdoll gira durante a recuperacao.
        Vec3 forward = instance.physicsState.links.front()
            .orientation.rotate({ 1.0f, 0.0f, 0.0f });
        forward.z = 0.0f;
        if (forward.lengthSquared() < 0.000001f) {
            forward = { 1.0f, 0.0f, 0.0f };
        } else {
            forward = forward.normalized();
        }
        const Vec3 left { -forward.y, forward.x, 0.0f };
        const Vec3 direction = forward * std::cos(radians)
            + left * std::sin(radians);
        const Vec3 totalForce = direction * impact.forceNewtons;

        // Rajada distribuida pela area frontal do corpo. A soma e exatamente
        // a forca escolhida; aplicar em tres links permite que a articulation
        // produza inclinacao e momento reais em vez de empurrar a raiz como
        // uma peca cinematica unica.
        m_physicsScene->applyRagdollLinkForce(
            instance.physicsRagdoll, 0, totalForce * 0.20f, {});
        m_physicsScene->applyRagdollLinkForce(
            instance.physicsRagdoll, 2, totalForce * 0.32f, {});
        m_physicsScene->applyRagdollLinkForce(
            instance.physicsRagdoll, 3, totalForce * 0.48f, {});
    }
}

} // namespace MatterEngine::Workbench
