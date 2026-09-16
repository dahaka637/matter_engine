#include "Workbench/WorkbenchApp.hpp"

#include "Engine/Core/Log.hpp"
#include "Engine/Geometry/MeshData3D.hpp"
#include "Engine/Geometry/GltfLoader.hpp"
#include "Engine/RHI/RHITypes.hpp"
#include "Workbench/Laboratory/StaticFootworkTerrainProbe.hpp"

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

float ragdollYaw(Quaternion orientation) {
    const Vec3 forward = orientation.rotate({ 1.0f, 0.0f, 0.0f });
    return std::atan2(forward.y, forward.x);
}

MeshData3D buildCapsuleMesh(
    const RagdollCapsuleDefinition3D& capsule) {
    // A cápsula nativa do PhysX é alinhada ao eixo X. A mesma geometria é
    // assada aqui já no espaço local do link, incluindo pose do collider:
    // render e colisão permanecem rigorosamente sobrepostos.
    constexpr int RadialSegments = 12;
    constexpr int HemisphereRings = 4;
    const float radius = capsule.radiusMeters;
    const float cylinderHalfLength =
        std::max(0.0f, capsule.lengthMeters * 0.5f - radius);

    struct Ring {
        float x = 0.0f;
        float radial = 0.0f;
        float normalX = 0.0f;
        float normalRadial = 1.0f;
    };
    std::vector<Ring> rings;
    rings.reserve(static_cast<std::size_t>(HemisphereRings * 2 + 3));
    rings.push_back(
        { -cylinderHalfLength - radius, 0.0f, -1.0f, 0.0f });
    for (int index = 1; index <= HemisphereRings; ++index) {
        const float angle = -Pi * 0.5f
            + Pi * 0.5f
                * static_cast<float>(index)
                / static_cast<float>(HemisphereRings);
        rings.push_back({ -cylinderHalfLength + radius * std::sin(angle),
            radius * std::cos(angle), std::sin(angle), std::cos(angle) });
    }
    if (cylinderHalfLength > 0.00001f) {
        rings.push_back(
            { cylinderHalfLength, radius, 0.0f, 1.0f });
    }
    for (int index = 1; index <= HemisphereRings; ++index) {
        const float angle = Pi * 0.5f
            * static_cast<float>(index)
            / static_cast<float>(HemisphereRings);
        rings.push_back({ cylinderHalfLength + radius * std::sin(angle),
            radius * std::cos(angle), std::sin(angle), std::cos(angle) });
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
    if ((m_humanRagdollProfile && m_footworkRagdollProfile && !m_characterVisual.parts.empty())
        || m_ragdollAssetsLoadFailed
        || !m_physicsScene) {
        return;
    }
    try {
        const auto loadVisualProfile = [&](const char* fileName,
                std::vector<GpuModel3D>& destinationVisuals,
                std::vector<Vec3>& destinationJointPositions) {
            RagdollProfile3D profile = loadRagdollProfile3D(
                std::string(MATTERENGINE_ASSETS_DIR)
                    + "/physics/ragdolls/" + fileName);
            validateRagdollProfileOrThrow3D(profile);
            std::vector<GpuModel3D> visuals;
            visuals.reserve(profile.links.size());
            std::vector<Vec3> jointLocalPositions(profile.links.size());
            for (std::size_t index = 0;
                    index < profile.links.size(); ++index) {
                const RagdollLinkDefinition3D& link = profile.links[index];
                visuals.push_back(uploadRagdollMesh(
                    activeRenderer, buildRagdollLinkVisual(link)));
                if (link.parentIndex >= 0) {
                    jointLocalPositions[index] =
                        link.modelOrientation.conjugate().rotate(
                            link.inboundJoint.anchorModelPosition
                                - link.modelPosition);
                }
            }
            destinationVisuals = std::move(visuals);
            destinationJointPositions = std::move(jointLocalPositions);
            return profile;
        };
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
        m_footworkRagdollProfile = loadVisualProfile(
            "FootworkLowerBodyV1.ragdoll.json",
            m_footworkRagdollLinkVisuals,
            m_footworkRagdollJointLocalPositions);
        m_ragdollJointVisual = uploadRagdollMesh(
            activeRenderer, buildJointSphereMesh());
    } catch (const std::exception& error) {
        Log::error(std::string("Falha ao carregar articulation: ")
            + error.what());
        m_humanRagdollProfile.reset();
        releaseGpuModel3D(activeRenderer, m_characterVisual);
        m_footworkRagdollProfile.reset();
        for (GpuModel3D& visual : m_ragdollLinkVisuals) {
            releaseGpuModel3D(activeRenderer, visual);
        }
        m_ragdollLinkVisuals.clear();
        for (GpuModel3D& visual : m_footworkRagdollLinkVisuals) {
            releaseGpuModel3D(activeRenderer, visual);
        }
        m_footworkRagdollLinkVisuals.clear();
        releaseGpuModel3D(activeRenderer, m_ragdollJointVisual);
        m_ragdollJointLocalPositions.clear();
        m_footworkRagdollJointLocalPositions.clear();
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
        instance.kind = ArticulatedRigKind::Human;
        instance.active = spawn.active;
        instance.activeController.reset(
            *m_humanRagdollProfile, spawn.orientation);
        instance.activeController.config().useMagicPelvisStabilization =
            m_ragdollUseMagicMode;
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

void WorkbenchApp::spawnFootworkRagdoll() {
    if (!m_physicsScene || !m_footworkRagdollProfile
        || !m_laboratoryMapLoaded) {
        m_laboratoryStatus = "Meio-esqueleto Footwork indisponivel";
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
        pelvisPosition = hit.normal.z > 0.55f
            ? hit.position + Vec3 { 0.0f, 0.0f,
                m_footworkRagdollProfile->standingRootHeightMeters + 0.025f }
            : hit.position - ray.direction * 0.55f;
    }
    const Quaternion orientation = Quaternion::fromAxisAngle(
        { 0.0f, 0.0f, 1.0f }, m_laboratoryCameraYaw);
    const Vec3 halfExtents { 0.31f, 0.22f, 0.54f };
    const Vec3 centerOffset { 0.0f, 0.0f, -0.44f };
    const std::optional<Vec3> safe = findSafeSpawnPosition(
        pelvisPosition + orientation.rotate(centerOffset),
        halfExtents, orientation, 4.0f);
    if (!safe || !spawnFootworkRagdollAt(
            *safe - orientation.rotate(centerOffset),
            orientation, false)) {
        m_laboratoryStatus =
            "Sem espaco seguro para criar o meio-esqueleto";
    }
}

bool WorkbenchApp::spawnFootworkRagdollAt(Vec3 pelvisPosition,
    Quaternion orientation, bool controlledTest) {
    if (!m_physicsScene || !m_footworkRagdollProfile
        || !m_laboratoryMapLoaded
        || m_spawnedRagdolls.size() >= MaximumSpawnedRagdolls) {
        return false;
    }
    RagdollSpawnDefinition3D spawn;
    spawn.entityId = m_nextEntityId++;
    spawn.pelvisPosition = pelvisPosition;
    spawn.orientation = orientation.normalized();
    spawn.rigidityPercent = 100.0f;
    spawn.active = true;
    try {
        const RagdollHandle3D handle = m_physicsScene->createRagdoll(
            *m_footworkRagdollProfile, spawn);
        const RagdollState3D initial =
            m_physicsScene->ragdollState(handle);
        StaticFootworkTerrainProbe terrain(*m_physicsScene);
        SpawnedRagdollInstance instance;
        instance.entityId = spawn.entityId;
        instance.physicsRagdoll = handle;
        instance.physicsState = initial;
        instance.previousPhysicsState = initial;
        instance.kind = ArticulatedRigKind::FootworkLowerBody;
        instance.controlledFootworkTest = controlledTest;
        instance.active = true;
        const PhysicalFootworkMode3D mode = controlledTest
            ? PhysicalFootworkMode3D::Manual
            : PhysicalFootworkMode3D::AutonomousFollow;
        if (!instance.footworkController.reset(
                *m_footworkRagdollProfile, initial,
                ragdollYaw(spawn.orientation), terrain, mode)) {
            m_physicsScene->destroyRagdoll(handle);
            return false;
        }
        m_spawnedRagdolls.push_back(std::move(instance));
        if (controlledTest) m_controlledFootworkEntityId = spawn.entityId;
        m_laboratoryStatus = controlledTest
            ? "Footwork fisico: WASD move, Shift corre"
            : "Meio-esqueleto Footwork criado";
        return true;
    } catch (const std::exception& error) {
        Log::error(std::string("Falha ao criar FootworkLowerBodyV1: ")
            + error.what());
        return false;
    }
}

void WorkbenchApp::updateActiveRagdolls(float deltaTime) {
    if (!m_physicsScene) return;
    const StaticFootworkTerrainProbe terrain(*m_physicsScene);
    for (SpawnedRagdollInstance& instance : m_spawnedRagdolls) {
        if (!instance.active
            || !m_physicsScene->contains(instance.physicsRagdoll)) {
            continue;
        }
        if (instance.kind == ArticulatedRigKind::FootworkLowerBody) {
            if (!m_footworkRagdollProfile) continue;
            instance.footworkController.update(
                *m_footworkRagdollProfile, instance.physicsState,
                deltaTime, terrain);
            const PhysicalFootworkOutput3D& output =
                instance.footworkController.output();
            if (output.applyRootForce) {
                m_physicsScene->applyRagdollRootForce(
                    instance.physicsRagdoll,
                    output.rootForceNewtons,
                    output.rootTorqueNewtonMeters);
            }
            m_physicsScene->setRagdollActiveDriveTargets(
                instance.physicsRagdoll, output.driveTargets,
                output.gravityCompensationEnabled);
            continue;
        }
        if (!m_humanRagdollProfile) continue;
        const RagdollDynamics3D dynamics =
            m_physicsScene->ragdollDynamics(instance.physicsRagdoll);
        instance.activeController.setExternalManipulationActive(
            m_physicsScene->grabbing()
                && m_physicsScene->grabbedRagdoll()
                    == instance.physicsRagdoll,
            m_physicsScene->grabbedRagdollLink());
        instance.activeController.update(*m_humanRagdollProfile,
            instance.physicsState, deltaTime, &terrain, &dynamics);
        const ActiveRagdollControlOutput3D& output =
            instance.activeController.output();
        if (output.applyRootForce) {
            m_physicsScene->applyRagdollRootForce(instance.physicsRagdoll,
                output.rootForceNewtons, output.rootTorqueNewtonMeters);
        }
        if (output.applySpineForce) {
            m_physicsScene->applyRagdollLinkForce(
                instance.physicsRagdoll, output.spineLinkIndex,
                output.spineForceNewtons,
                output.spineTorqueNewtonMeters);
        }
        m_physicsScene->setRagdollActiveDriveTargets(
            instance.physicsRagdoll, output.driveTargets,
            output.gravityCompensationEnabled);
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
