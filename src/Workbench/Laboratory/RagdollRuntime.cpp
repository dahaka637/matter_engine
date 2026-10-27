#include "Workbench/WorkbenchApp.hpp"

#include "Engine/Appearance/Compose/KitCompositor.hpp"

#include "Engine/Athlete/AthleteAssets3D.hpp"
#include "Engine/Athlete/Policy/LocomotionPolicy3D.hpp"
#include "Engine/Character/CharacterControlApplication3D.hpp"
#include "Engine/Core/Log.hpp"
#include "Engine/Geometry/MeshData3D.hpp"
#include "Engine/Geometry/GltfLoader.hpp"
#include "Engine/RHI/RHITypes.hpp"

#include <algorithm>
#include <cstdio>
#include <memory>
#include <filesystem>
#include <cstdlib>
#include <cmath>
#include <span>
#include <stdexcept>
#include <string>
#include <utility>
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

// A casca convexa (a chuteira): cada face com a normal dela (sombreado
// facetado, como a fisica a ve).
MeshData3D buildHullMesh(const RagdollCapsuleDefinition3D& hull) {
    MeshData3D mesh;
    for (std::size_t f = 0; f < hull.hullFaces.size() && f < hull.hullPlanes.size(); ++f) {
        const auto& face = hull.hullFaces[f];
        const Vec3 normal { hull.hullPlanes[f][0], hull.hullPlanes[f][1], hull.hullPlanes[f][2] };
        Vec3 a = hull.hullPoints[face[0]], b = hull.hullPoints[face[1]], c = hull.hullPoints[face[2]];
        if (dot(cross(b - a, c - a), normal) < 0.0f) std::swap(b, c);
        const std::uint32_t first = static_cast<std::uint32_t>(mesh.vertices.size());
        for (const Vec3 corner : { a, b, c }) {
            MeshVertex3D vertex;
            vertex.position = hull.localPosition + hull.localOrientation.rotate(corner);
            vertex.normal = hull.localOrientation.rotate(normal);
            vertex.color = RagdollVisualColor;
            mesh.vertices.push_back(vertex);
        }
        mesh.indices.insert(mesh.indices.end(), { first, first + 1, first + 2 });
    }
    recomputeBounds(mesh);
    return mesh;
}

MeshData3D buildRagdollLinkVisual(
    const RagdollLinkDefinition3D& link) {
    if (link.collider.shape == RagdollColliderShape3D::ConvexHull) return buildHullMesh(link.collider);
    if (ragdollColliderHasBox3D(link.collider)) {
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

// O uniforme do sistema de aparencia no personagem (RHI/CharacterMaterial.hpp):
// o kit composto sobre o template da pele (as costuras pelos vertices de
// origem), os arrays com os niveis prontos, a entrada por vertice e os
// parametros. Sem template ou se a composicao falhar, nada (o atlas antigo).
RHI::CharacterMaterialHandle uploadCharacterKit(Renderer& renderer, const RagdollCharacter3D& character, RHI::TextureHandle body,
    const std::string& kitId) {
    if (!character.surface || character.surfaceVertices.size() != character.mesh.vertices.size() || !body.valid()) return {};
    const Appearance::SurfaceTemplate& surface = *character.surface;
    std::vector<std::array<std::uint32_t, 3>> triangles(character.mesh.indices.size() / 3);
    for (std::size_t i = 0; i < character.mesh.indices.size(); ++i)
        triangles[i / 3][i % 3] = character.surfaceVertices[character.mesh.indices[i]].source;
    std::string error;
    const std::optional<Appearance::LoadedKit> kit =
        Appearance::loadKit(std::string(MATTERENGINE_ASSETS_DIR) + "/appearance", kitId, surface, &error);
    std::optional<Appearance::ComposedKit> composed;
    if (kit) composed = Appearance::composeKit(surface, triangles, *kit->design, kit->kit, {}, &error);
    if (!composed) {
        Log::error("Uniforme " + kitId + " nao composto (fica o atlas antigo): " + error);
        return {};
    }
    const auto packed = [](const std::vector<Appearance::KitTexture>& levels) {
        std::vector<std::byte> bytes;
        for (const Appearance::KitTexture& level : levels)
            bytes.insert(bytes.end(), reinterpret_cast<const std::byte*>(level.rgba.data()),
                reinterpret_cast<const std::byte*>(level.rgba.data() + level.rgba.size()));
        return bytes;
    };
    const std::vector<std::byte> kitPixels = packed(composed->kit), identityPixels = packed(composed->identity);
    std::vector<RHI::CharacterKitVertexGpu> vertices(character.surfaceVertices.size());
    for (std::size_t v = 0; v < vertices.size(); ++v) {
        const RagdollCharacter3D::SurfaceVertex& source = character.surfaceVertices[v];
        vertices[v].kitUv[0] = source.kitUv.x;
        vertices[v].kitUv[1] = source.kitUv.y;
        vertices[v].identityUv[0] = source.identityUv.x;
        vertices[v].identityUv[1] = source.identityUv.y;
        vertices[v].region = source.region >= 0 ? static_cast<std::uint32_t>(source.region) : RHI::CharacterKitNone;
        vertices[v].zone = source.identity >= 0 ? static_cast<std::uint32_t>(source.identity) : RHI::CharacterKitNone;
    }
    RHI::CharacterMaterialDesc desc;
    desc.kit = { { static_cast<std::uint32_t>(composed->kit.front().width), static_cast<std::uint32_t>(composed->kit.front().height) }, 1,
        static_cast<std::uint32_t>(composed->kit.size()), static_cast<float>(composed->kitSafeLevel), kitPixels };
    desc.identity = { { static_cast<std::uint32_t>(composed->identity.front().width), static_cast<std::uint32_t>(composed->identity.front().height) },
        1, static_cast<std::uint32_t>(composed->identity.size()), static_cast<float>(composed->identitySafeLevel), identityPixels };
    desc.body = body;
    desc.vertices = vertices;
    desc.parameters.levels[0] = static_cast<float>(composed->kitSafeLevel);
    desc.parameters.levels[1] = static_cast<float>(composed->identitySafeLevel);
    for (std::size_t r = 0; r < std::min(composed->regionAverage.size(), RHI::CharacterKitMaxRegions); ++r)
        for (std::size_t k = 0; k < 4; ++k) desc.parameters.regionAverage[r][k] = composed->regionAverage[r][k];
    const float width = static_cast<float>(surface.identityLayout.width), height = static_cast<float>(surface.identityLayout.height);
    for (std::size_t z = 0; z < std::min(surface.zones.size(), RHI::CharacterKitMaxZones); ++z) {
        const Appearance::SurfaceZone& zone = surface.zones[z];
        desc.parameters.zoneRect[z][0] = static_cast<float>(zone.block[0] + zone.inset) / width;
        desc.parameters.zoneRect[z][1] = static_cast<float>(zone.block[1] + zone.inset) / height;
        desc.parameters.zoneRect[z][2] = static_cast<float>(zone.block[0] + zone.block[2] - zone.inset) / width;
        desc.parameters.zoneRect[z][3] = static_cast<float>(zone.block[1] + zone.block[3] - zone.inset) / height;
    }
    try {
        const RHI::CharacterMaterialHandle handle = renderer.createCharacterMaterial(desc);
        Log::info("Uniforme " + kitId + " composto em " + std::to_string(static_cast<int>(composed->milliseconds.total)) + " ms"
            + (handle.valid() ? "" : " (sem o pipeline do personagem: fica o atlas antigo)"));
        return handle;
    } catch (const std::exception& exception) {
        Log::error("Uniforme " + kitId + " sem material na GPU (fica o atlas antigo): " + exception.what());
        return {};
    }
}

} // namespace

void WorkbenchApp::ensureRagdollAssetsLoaded(Renderer& activeRenderer) {
    // O corpo mudou (applyLaboratoryBody): a malha dele no lugar da de antes;
    // os indices e a textura sao os mesmos.
    if (m_characterVisualDirty && m_ragdollCharacter && !m_characterVisual.parts.empty()) {
        GpuModel3D visual = uploadRagdollMesh(activeRenderer, laboratoryCharacter().mesh);
        visual.parts.front().albedoTexture = m_characterVisual.parts.front().albedoTexture;
        visual.textures = std::exchange(m_characterVisual.textures, {});
        releaseGpuModel3D(activeRenderer, m_characterVisual);
        m_characterVisual = std::move(visual);
    }
    m_characterVisualDirty = false;
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
        m_humanRagdollProfile = &laboratoryCharacter().profile;
        updateCharacterSpawnBox();
        m_characterVisual = uploadRagdollMesh(activeRenderer, laboratoryCharacter().mesh);
        if (!m_ragdollCharacter->albedoPath.empty()) {
            const auto image=loadImageRgba3D(m_ragdollCharacter->albedoPath);
            const auto texture=activeRenderer.createTexture2D(
                {static_cast<std::uint32_t>(image.width),static_cast<std::uint32_t>(image.height)},
                image.rgbaPixels);
            m_characterVisual.textures.push_back(texture);
            m_characterVisual.parts.front().albedoTexture=texture;
            // O uniforme novo por cima (o atlas antigo fica como o corpo).
            m_characterKit = uploadCharacterKit(activeRenderer, *m_ragdollCharacter, texture, "branco_10");
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
        m_humanRagdollProfile = nullptr;
        if (m_characterKit.valid()) activeRenderer.destroyCharacterMaterial(std::exchange(m_characterKit, {}));
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

void WorkbenchApp::updateCharacterSpawnBox() {
    Vec3 minimum=laboratoryCharacter().mesh.boundsMin;
    Vec3 maximum=laboratoryCharacter().mesh.boundsMax;
    const auto absolute=[](Vec3 value) {return Vec3{std::abs(value.x),std::abs(value.y),std::abs(value.z)};};
    for (const auto& link : m_humanRagdollProfile->links) {
        const auto rotation=link.modelOrientation*link.collider.localOrientation;
        Vec3 extents;
        if (ragdollColliderHasBox3D(link.collider)) {
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
        instance.athlete.legacy.guideFacingYawRadians = std::atan2(
            spawnForward.y, spawnForward.x);
        instance.kind = ArticulatedRigKind::Human;
        instance.active = spawn.active;
        instance.athlete.legacy.locomotion.reset(*m_humanRagdollProfile, initial);
        instance.athlete.legacy.adaptivePhysics.reset(*m_humanRagdollProfile);
        instance.athlete.legacy.biomechanics.reset(*m_humanRagdollProfile, initial,
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
    if(!m_ragdollCharacter)return {};
    // O levantar de brucos e o de costas escolhidos no painel (para comparar).
    return athleteLocomotionClips3D(laboratoryCharacter(),laboratoryClips(),m_getUpFrontClip,m_getUpBackClip);
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
    candidate->athlete.legacy.locomotion.reset(*m_humanRagdollProfile,
        candidate->physicsState);
    candidate->athlete.legacy.adaptivePhysics.reset(*m_humanRagdollProfile);
    m_characterFollowVelocity = {};
    candidate->athlete.legacy.biomechanics.reset(*m_humanRagdollProfile,
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
            ? "WASD: passada física | Alt: câmera livre | mouse 4: congelar"
            : "WASD mover | Alt: câmera livre | mouse 4: congelar",
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
        LegacyAthleteState3D& legacy = controlled->athlete.legacy;
        legacy.locomotion.reset(*m_humanRagdollProfile, initial);
        legacy.adaptivePhysics.reset(*m_humanRagdollProfile);
        legacy.navigationAnchorValid = false;
        // O modo do corpo tambem recomeca: o ragdoll novo nasce dinamico, e o
        // estado antigo ("cinematico", no padrao) o deixava sem pose animada e
        // sem motores - 100% ragdoll. Com a locomocao padrao, ja volta nela.
        legacy.defaultLocomotion.reset(m_characterDesiredFacingYaw);
        legacy.bodyModeSeconds = 0.0f;
        legacy.calmSeconds = 0.0f;
        legacy.airSeconds = 0.0f;
        legacy.physicalFromJump = false;
        legacy.returnBoost = 0.0f;
        legacy.crouchHeld = false;
        legacy.defaultTackling = false;
        legacy.guided = m_defaultLocomotion;
        legacy.guidedExitLeft = 0.0f;
        legacy.bodyMode = m_defaultLocomotion ? LegacyBodyMode3D::Animated : LegacyBodyMode3D::Physical;
        if (legacy.guided)
            m_physicsScene->setRagdollGuided(controlled->physicsRagdoll, true);
        m_characterFollowVelocity = {};
        legacy.biomechanics.reset(*m_humanRagdollProfile, initial,
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

void WorkbenchApp::applyLaboratoryBody() {
    m_bodyApplyRequested = false;
    if (!m_ragdollCharacter) return;
    const LaboratoryBody* body = m_laboratoryBodies.select(*m_ragdollCharacter, m_animationClips,
        m_bodyHeightCm, m_bodyMassKg);
    if (body == m_laboratoryBody) return;
    m_laboratoryBody = body;
    // A capsula do corpo, com o andar e o sprint dos clipes dele (como o
    // AthleteBundle3D).
    m_avatarGaitSpeeds = characterGaitSpeeds3D(characterLocomotionAnimations());
    m_avatarCharacterSettings = athleteAvatarMotorSettings3D(laboratoryBody());
    if (m_avatarGaitSpeeds.walk > 0.0f) {
        m_avatarCharacterSettings.walkSpeed = m_avatarGaitSpeeds.walk;
        m_avatarCharacterSettings.sprintSpeed = m_avatarGaitSpeeds.sprint;
    }
    // (Sem os recursos carregados, ensureRagdollAssetsLoaded ja usa o corpo.)
    if (!m_humanRagdollProfile || !m_physicsScene) return;
    endPhysGunGrab();
    m_humanRagdollProfile = &laboratoryCharacter().profile;
    updateCharacterSpawnBox();
    m_characterVisualDirty = true;
    m_laboratoryTaaHistoryValid = false;
    // Os bonecos humanos de novo no perfil do corpo, de pe onde estavam, com o
    // atleta do zero: a locomocao padrao guarda o esqueleto dos gestos do
    // primeiro perfil que viu (o reset nao o zera). O controlado volta pela
    // pose inicial (a capsula nova).
    for (SpawnedRagdollInstance& instance : m_spawnedRagdolls) {
        if (instance.physicsState.links.empty()) continue;
        const PhysicsBodyState3D root = instance.physicsState.links.front();
        const Vec3 forward = root.orientation.rotate({ 1.0f, 0.0f, 0.0f });
        const bool controlled = instance.entityId == m_controlledCharacterEntityId;
        const float yaw = controlled ? m_characterDesiredFacingYaw : std::atan2(forward.y, forward.x);
        instance.athlete = AthleteState3D {};
        instance.athlete.legacy.guideFacingYawRadians = yaw;
        instance.skinMatrices.clear();
        instance.previousSkinMatrices.clear();
        instance.visualPose = {};
        instance.visualRotations.clear();
        instance.previousVisualRotations.clear();
        instance.visualPoseTime = -1.0;
        instance.simulationPreviousState = {};
        instance.renderedPositions.clear();
        instance.renderedOrientations.clear();
        if (controlled) continue;
        RagdollSpawnDefinition3D spawn;
        spawn.entityId = instance.entityId;
        spawn.pelvisPosition = { root.position.x, root.position.y,
            ragdollGroundHeight(root.position) + m_humanRagdollProfile->standingRootHeightMeters };
        spawn.orientation = Quaternion::fromAxisAngle({ 0.0f, 0.0f, 1.0f }, yaw);
        spawn.rigidityPercent = m_ragdollRigidityPercent;
        spawn.active = instance.active;
        m_physicsScene->destroyRagdoll(instance.physicsRagdoll);
        try {
            instance.physicsRagdoll = m_physicsScene->createRagdoll(*m_humanRagdollProfile, spawn);
        } catch (const std::exception& error) {
            Log::error(std::string("Falha ao recriar ragdoll no corpo novo: ") + error.what());
            instance.entityId = 0;
            continue;
        }
        const RagdollState3D initial = m_physicsScene->ragdollState(instance.physicsRagdoll);
        instance.physicsState = initial;
        instance.previousPhysicsState = initial;
        instance.frozen = false;
        LegacyAthleteState3D& legacy = instance.athlete.legacy;
        legacy.locomotion.reset(*m_humanRagdollProfile, initial);
        legacy.adaptivePhysics.reset(*m_humanRagdollProfile);
        legacy.biomechanics.reset(*m_humanRagdollProfile, initial,
            m_physicsScene->ragdollDynamics(instance.physicsRagdoll));
    }
    // (Os que nao renasceram saem; o id 0 nunca e o controlado.)
    std::erase_if(m_spawnedRagdolls, [](const SpawnedRagdollInstance& instance) { return instance.entityId == 0; });
    resetControlledCharacterPose();
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
    // A bola mais perto congela junto (e a mesma descongela depois).
    if (frozen) {
        m_frozenSceneBallEntityId = 0;
        float best = std::numeric_limits<float>::max();
        const Vec3 at = controlled->physicsState.links.empty() ? Vec3 {} : controlled->physicsState.links.front().position;
        for (const SpawnedInflatedBallInstance& ball : m_inflatedBalls) {
            if (!m_physicsScene->contains(ball.physicsBall)) continue;
            const Vec3 d = m_physicsScene->inflatedBallState(ball.physicsBall).position - at;
            const float distance = d.x * d.x + d.y * d.y + d.z * d.z;
            if (distance < best) {
                best = distance;
                m_frozenSceneBallEntityId = ball.entityId;
            }
        }
    }
    for (const SpawnedInflatedBallInstance& ball : m_inflatedBalls)
        if (ball.entityId == m_frozenSceneBallEntityId && m_physicsScene->contains(ball.physicsBall))
            m_physicsScene->setInflatedBallFrozen(ball.physicsBall, frozen);
    if (!frozen) m_frozenSceneBallEntityId = 0;
    m_characterProxyVelocityWorld = {};
    m_characterSprinting = false;
    m_laboratoryTaaHistoryValid = false;
    m_laboratoryStatus = frozen
        ? "Ragdoll congelado para inspeção"
        : "Ragdoll descongelado";
    m_notification = {
        frozen ? "CENA CONGELADA | mouse 4 para descongelar"
               : "RAGDOLL DESCONGELADO",
        2.4f, 2.4f
    };
}

void WorkbenchApp::reloadLocomotionPolicy() {
    namespace fs = std::filesystem;
    fs::path file;
    if (const char* given = std::getenv("MATTER_LOCOMOTION_POLICY"); given != nullptr && *given != '\0') {
        file = given;
    } else {
        fs::path data;
        if (const char* root = std::getenv("MATTER_RL_DATA"); root != nullptr && *root != '\0') data = root;
        else if (const char* home = std::getenv("HOME"); home != nullptr) data = fs::path(home) / ".local/share/matter-engine/rl";
        file = data / "runs" / "loco_f1f2" / "exports" / "policy_latest.mrlpolicy";
    }
    if (!m_humanRagdollProfile) {
        m_locomotionPolicyLabel = "sem o corpo do jogador";
        return;
    }
    try {
        auto policy = std::make_shared<LocomotionPolicy3D>(*m_humanRagdollProfile, file);
        char label[96];
        std::snprintf(label, sizeof label, "%s, rodada %llu", policy->run().c_str(),
            static_cast<unsigned long long>(policy->iteration()));
        m_locomotionPolicyLabel = label;
        Log::info("rede da locomocao: " + m_locomotionPolicyLabel + " (" + file.string() + ")");
        m_athleteRuntime.setLocomotionPolicy(std::move(policy));
    } catch (const std::exception& error) {
        m_locomotionPolicyLabel = "nao carregou";
        Log::warn(std::string("rede da locomocao: ") + error.what());
    }
}

AthleteRuntimeSettings3D WorkbenchApp::athleteRuntimeSettings() const {
    AthleteRuntimeSettings3D settings;
    settings.defaultLocomotion = m_defaultLocomotion;
    settings.adaptivePhysics = m_adaptivePhysics;
    settings.ragdollForceControlOverride = m_ragdollForceControlOverride;
    settings.ragdollAuxiliaryPercent = m_ragdollAuxiliaryPercent;
    settings.ragdollMusclePercent = m_ragdollMusclePercent;
    settings.characterMusclePercent = m_characterMusclePercent;
    settings.legsWalkingE5 = m_legsWalkingE5;
    settings.legsOnlyStanding = m_legsOnlyStanding;
    settings.legsPostureStanding = m_legsPostureStanding;
    settings.physicalLocomotionV2 = m_physicalLocomotionV2;
    settings.e5StandingAssistPercent = m_e5StandingAssistPercent;
    settings.e5WalkingAssistPercent = m_e5WalkingAssistPercent;
    settings.e5FootPlacement = m_e5FootPlacement;
    settings.staggerRecovery = m_staggerRecovery;
    settings.biomechanicalExperiment = m_biomechanicalExperiment;
    settings.biomechanicalBalanceAssistPercent = m_biomechanicalBalanceAssistPercent;
    settings.getUpAssistPercent = m_getUpAssistPercent;
    settings.getUpHitWeakens = m_getUpHitWeakens;
    settings.getUpHitAssistPercent = m_getUpHitAssistPercent;
    settings.getUpHitRecoverSeconds = m_getUpHitRecoverSeconds;
    settings.getUpStiffnessPercent = m_getUpStiffnessPercent;
    settings.getUpStrengthPercent = m_getUpStrengthPercent;
    settings.tackleEnabled = m_tackleEnabled;
    settings.tackledAssistLow = m_tackledAssistLow;
    settings.tackledRecoverSeconds = m_tackledRecoverSeconds;
    settings.defaultAirTackle = m_defaultAirTackle;
    settings.defaultAirTacklePitch = m_defaultAirTacklePitch;
    settings.defaultVoadoraContactOnly = m_defaultVoadoraContactOnly;
    settings.motionVariety = m_motionVariety;
    settings.motionVarietyPercent = m_motionVarietyPercent;
    settings.footSpring = m_footSpring;
    settings.ankleBalance = m_ankleBalance;
    settings.toeRoll = m_toeRoll;
    settings.defaultGuidedExitSeconds = m_defaultGuidedExitSeconds;
    settings.defaultSeedFromBody = m_defaultSeedFromBody;
    settings.defaultPhysicalJump = m_defaultPhysicalJump;
    settings.defaultGetUpHandoff = m_defaultGetUpHandoff;
    settings.defaultAdaptiveTransitions = m_defaultAdaptiveTransitions;
    settings.defaultTrunkMotion = m_defaultTrunkMotion;
    settings.defaultTrunkMotionPercent = m_defaultTrunkMotionPercent;
    settings.defaultStartGesturePercent = m_defaultStartGesturePercent;
    settings.defaultLandingAbsorb = m_defaultLandingAbsorb;
    settings.defaultCurveLeanDeadZone = m_defaultCurveLeanDeadZone;
    settings.defaultTrotSpeed = m_defaultTrotSpeed;
    settings.defaultSprintForwardSpeed = m_defaultSprintForwardSpeed;
    settings.defaultSprintSideSpeed = m_defaultSprintSideSpeed;
    settings.defaultSprintBackwardSpeed = m_defaultSprintBackwardSpeed;
    settings.defaultMaxSprintSpeed = m_defaultMaxSprintSpeed;
    settings.ballForce = m_ballForce;
    settings.touchGestureSeconds = m_touchGestureMilliseconds / 1000.0f;
    settings.shotGestureSeconds = m_shotGestureMilliseconds / 1000.0f;
    settings.activeBody = m_activeBody;
    settings.physicalBridge = m_physicalBridge;
    settings.modeTable = m_modeTable;
    settings.physicalUsesDefaultSpeeds = m_physicalDefaultSpeeds;
    settings.physicalFeetAssistPercent = m_physicalFeetAssistPercent;
    settings.locomotionPolicy = m_locomotionPolicyOn && m_athleteRuntime.locomotionPolicy() != nullptr;
    // (Com a rede, sem guiagem de ossos: como no treino.)
    settings.physicalBoneGuidePercent = m_locomotionPolicyOn ? 0.0f : m_physicalBoneGuidePercent;
    // A virada brusca (experimento): so no Fisico total.
    settings.dynamicTurnMove = m_dynamicTurnMove && !m_defaultLocomotion && !m_locomotionPolicyOn;
    settings.dynamicTurnMoveSettings.amount = std::clamp(m_dynamicTurnMovePercent * 0.01f, 0.0f, 1.5f);
    // Com a rede que treinou com o professor (Fisico com RL), ele entra, como
    // no treino.
    if (settings.locomotionPolicy && m_athleteRuntime.locomotionPolicy() != nullptr
        && m_athleteRuntime.locomotionPolicy()->usesTurnMove()) {
        settings.dynamicTurnMove = true;
        settings.dynamicTurnMoveSettings.amount = m_athleteRuntime.locomotionPolicy()->turnMoveAmount();
    }
    settings.fullPhysicalImpulse = m_fullPhysicalImpulse;
    settings.activeBodySettings.gameplayAssistAcceleration = m_activeAssistAcceleration;
    settings.capsuleResponsePercent = m_capsuleResponsePercent * LabResponseScale;
    settings.capsuleReversalPercent = m_capsuleReversalPercent;
    return settings;
}

void WorkbenchApp::updateActiveRagdolls(float deltaTime) {
    if(!m_physicsScene||!m_humanRagdollProfile)return;
    const auto clips=characterLocomotionAnimations();
    // O modo padrao com os clipes dele (o trote e o sprint das referencias e
    // as diagonais) por cima dos da locomocao fisica.
    const auto defaultClips=m_ragdollCharacter
        ?defaultLocomotionAnimations3D(clips,m_ragdollCharacter->defaultLocomotionClipIds,laboratoryClips()):clips;
    AthleteRuntimeAssets3D assets;
    assets.profile = &*m_humanRagdollProfile;
    assets.clips = &clips;
    assets.defaultClips = &defaultClips;
    // As poses de referencia do experimento biomecanico: o parado e o andar
    // do proprio personagem (clipes ja validados contra o perfil).
    for (const AnimationClip3D& candidate : laboratoryClips()) {
        if (m_ragdollCharacter && candidate.id == m_ragdollCharacter->idleClipId)
            assets.idleReferenceClip = &candidate;
        else if (m_ragdollCharacter && candidate.id == m_ragdollCharacter->walkClipId)
            assets.walkReferenceClip = &candidate;
    }
    assets.avatarSettings = m_avatarCharacterSettings;
    assets.body = laboratoryBody();
    std::vector<AthleteSlot3D> athletes;
    athletes.reserve(m_spawnedRagdolls.size());
    for (SpawnedRagdollInstance& instance : m_spawnedRagdolls)
        athletes.push_back({ instance.entityId, instance.physicsRagdoll, &instance.physicsState, instance.active,
            instance.frozen, &instance.athlete });
    std::vector<AthleteBall3D> balls;
    balls.reserve(m_inflatedBalls.size());
    for (SpawnedInflatedBallInstance& ball : m_inflatedBalls)
        balls.push_back({ ball.physicsBall, &ball.physicsState, &ball.ownership });
    AthleteControlInput3D control;
    control.controlledEntityId = m_controlledCharacterEntityId;
    control.intent = m_characterIntent;
    control.facingYawRadians = m_characterDesiredFacingYaw;
    control.lookPitchRadians = m_laboratoryCameraPitch;
    control.sprinting = m_characterSprinting;
    control.proxyVelocityWorld = m_characterProxyVelocityWorld;
    control.capsuleWallImpact = m_controlledWallImpact;
    control.capsuleFollowVelocityWorld = m_characterFollowVelocity;
    control.spaceHeld = m_characterSpaceHeld;
    control.jumpCrouch = m_laboratoryJumpCrouch;
    control.jumpCharge = m_laboratoryJumpCharge;
    control.ballRequest = m_ballRequest;
    control.ballRequestPower = m_ballRequestPower;
    control.ballRequestAimYaw = m_ballRequestAimYaw;
    control.ballRequestLift = m_ballRequestLift;
    control.ballRequestCurve = m_ballRequestCurve;
    control.ballShotHeld = m_ballShotHeld;
    control.ballDribbleHeld = m_ballDribbleHeld;
    control.ballRaiseHeld = m_ballRaiseHeld;
    control.biomechanicalRequestedVelocity = m_biomechanicalRequestedVelocity;
    AthleteRuntimeOutput3D output;
    m_athleteRuntime.update(*m_physicsScene, assets, athletes, balls, athleteRuntimeSettings(), control, deltaTime,
        output);
    if (output.followVelocitySet) m_characterFollowVelocity = output.followVelocityWorld;
    if (output.controlledAnimatedSet) m_controlledAnimated = output.controlledAnimated;
    m_controlledSpeedScale = output.controlledSpeedScale;
    if (output.ballRequestConsumed) m_ballRequest = BallActionKind3D::None;
    m_reachValid = output.reachValid;
    if (output.reachValid) {
        m_reachOrigin = output.reachOrigin;
        m_reachYaw = output.reachYaw;
        m_reachPelvisHeight = output.reachPelvisHeight;
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
