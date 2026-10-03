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
        if(clip.id==m_ragdollCharacter->standUpBackClipId)clips.standUpBack=&clip;
        if(clip.id==m_ragdollCharacter->standUpFrontClipId)clips.standUpFront=&clip;
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

void WorkbenchApp::updateActiveRagdolls(float deltaTime) {
    if(!m_physicsScene||!m_humanRagdollProfile)return;
    const auto clips=characterLocomotionAnimations();
    for(auto& instance:m_spawnedRagdolls) {
        if(!instance.active||instance.frozen||instance.physicsState.frozen
            ||!m_physicsScene->contains(instance.physicsRagdoll)
            ||instance.physicsState.links.empty())continue;
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
        command.muscleAuthority = m_ragdollForceControlOverride
            ? m_ragdollMusclePercent * 0.01f : 1.0f;
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
            // Reference poses come only from clips already vetted in the
            // procedural workspace (see loadAnimationCatalog): the character's
            // own idle and walk.
            for (const AnimationClip3D& candidate : m_proceduralAnimationClips) {
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
        instance.adaptivePhysics.settings().legsAssistRetained = e5
            ? std::clamp(m_e5StandingAssistPercent * 0.01f, 0.0f, 1.0f) : 0.0f;
        instance.adaptivePhysics.settings().walkingAssistScale = e5
            ? std::clamp(m_e5WalkingAssistPercent * 0.01f, 0.0f, 1.0f) : 1.0f;
        command.balanceFootPlacement = e5 && m_e5FootPlacement;
        command.motionVariety = m_motionVariety ? m_motionVarietyPercent * 0.01f : 0.0f;
        command.footSpring = m_footSpring;
        command.ankleBalance = m_ankleBalance;
        command.varietySeed = static_cast<std::uint32_t>(instance.entityId * 0x9E3779B97F4A7C15ull >> 32);
        instance.locomotion.update(*m_humanRagdollProfile, clips,
            instance.physicsState, dynamics, command, deltaTime);
        const auto& output=instance.locomotion.output();
        // Fisica adaptativa: com a opcao ligada, a forca limitada da pelve e
        // aplicada no lugar do transporte pela capsula; desligada, so mede.
        const CharacterLocomotionState3D bodyState =
            instance.locomotion.telemetry().state;
        const bool down = bodyState == CharacterLocomotionState3D::Fallen
            || bodyState == CharacterLocomotionState3D::GettingUp;
        AdaptivePhysicalIntent3D physicalIntent;
        physicalIntent.proxyVelocityWorld = command.proxyVelocityWorld;
        // Sem a velocidade pedida, o controlador interpretava toda marcha
        // como estado parado: legsWalking ficava falso e walkingAssist era
        // forçado a 1, ignorando completamente o slider "Ajuda andando".
        physicalIntent.requestedVelocityWorld =
            command.intent.requestedVelocityWorld;
        physicalIntent.navigationRootWorld = command.rootPositionWorld;
        physicalIntent.airborne = !command.grounded;
        physicalIntent.manipulated = command.manipulated;
        physicalIntent.suspended = down;
        physicalIntent.jumpCharge = command.controlled ? m_laboratoryJumpCharge : 0.0f;
        physicalIntent.terrainDemand = instance.locomotion.telemetry().terrainDemand;
        physicalIntent.footGround = command.footGround;
        // O que a capsula (ou a ancora virtual) andou para ir atras do corpo
        // neste tick nao e intencao do jogador.
        physicalIntent.followVelocityWorld = command.controlled
            ? m_characterFollowVelocity : instance.navigationAnchorVelocity;
        physicalIntent.physicalState = &instance.locomotion.physicalState();
        physicalIntent.contactPlan = &instance.locomotion.contactPlan();
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
                    output.guide.rootPositionWorld, followSlack)
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
                || locomotionState == CharacterLocomotionState3D::GettingUp) {
                const Vec3 root = instance.physicsState.links.front().position;
                m_physicsScene->placeCharacter(
                    { root.x, root.y, ragdollGroundHeight(root) },
                    m_avatarCharacterSettings);
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
