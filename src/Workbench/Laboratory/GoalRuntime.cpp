#include "Workbench/WorkbenchApp.hpp"

#include "Engine/Core/Log.hpp"
#include "Engine/Modeling/Objects/ObjectRecipe.hpp"
#include "Engine/Objects/ObjectAsset3D.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <span>
#include <string>

// A trave com rede do mapa do laboratorio: a estrutura fixa no chao (o corpo
// estatico com os tubos), a rede na cena de fisica do laboratorio (anda nos
// subpassos da bola, segura bola e objetos), a barreira invisivel no lugar
// dela (so os jogadores batem) e o desenho - a trave
// como objeto procedural e os fios como a pele dos nos da fisica, cada fio no
// triangulo de colisao (fora da bola e dos membros).
namespace MatterEngine::Workbench {

namespace {

// O fundo da rede estufa ate ~0,8 m para fora da trave: a esfera do recorte
// leva essa folga.
constexpr float NetBulgeMargin = 1.0f;

} // namespace

void WorkbenchApp::createLaboratoryGoal(Renderer& renderer, Vec3 goalLineCenter, float facingYaw) {
    releaseLaboratoryGoal(renderer);
    if (!m_physicsScene) return;
    std::string error;
    const auto recipe = Modeling::readObjectRecipe(std::filesystem::path(MATTERENGINE_ASSETS_DIR) / "objects" / "trave.json", &error);
    const auto object = recipe ? Modeling::generateObject(*recipe, &error) : std::nullopt;
    if (!object || !object->net) {
        Log::error("Trave do laboratorio: " + (error.empty() ? std::string("sem rede") : error));
        return;
    }
    LaboratoryGoal& goal = m_laboratoryGoal;
    try {
        goal.object = std::make_unique<Modeling::ProceduralObject>(*object);
        // A boca para facingYaw; a linha do gol (a frente dos postes) no
        // ponto dado, os pes no chao (z = 0).
        goal.orientation = Quaternion::fromAxisAngle({ 0.0f, 0.0f, 1.0f }, facingYaw);
        goal.position = goalLineCenter - goal.orientation.rotate({ 0.5f * object->size.x, 0.0f, 0.0f })
            + Vec3 { 0.0f, 0.0f, 0.5f * object->size.z };
        ObjectBodyData3D body = objectBodyData3D(*object, m_materialLibrary);
        body.body.motionType = PhysicsMotionType3D::Static;
        body.body.position = goal.position;
        body.body.orientation = goal.orientation;
        body.body.entityId = m_nextEntityId++;
        goal.body = m_physicsScene->createBody(body.body, body.shapes);
        goal.net = m_physicsScene->createNet(objectNetDescription3D(*object, goal.position, goal.orientation, 0.0f));
        m_physicsScene->settleNet(goal.net, 8.0f);
        // A barreira invisivel dos jogadores, na pose da trave.
        for (ObjectBodyData3D& barrier : objectNetBarrier3D(*object)) {
            barrier.body.position = goal.position;
            barrier.body.orientation = goal.orientation;
            goal.barriers.push_back(m_physicsScene->createBody(barrier.body, barrier.shapes));
        }
        const std::span<const Vec3> knots = m_physicsScene->netPositions(goal.net);
        goal.knots.assign(knots.begin(), knots.end());
        goal.previousKnots = goal.knots;
        goal.frame = uploadProceduralObject(renderer, objectRenderData3D(*object));
        const ObjectNetRenderData3D strands = objectNetRenderData3D(*object, goal.position, goal.orientation);
        goal.restKnots = strands.restKnots;
        goal.strandVertices = renderer.createBuffer({ strands.mesh.vertices.size() * sizeof(MeshVertex3D), RHI::BufferUsage::Vertex,
            true, "Trave: fios" });
        goal.strandIndices = renderer.createBuffer({ strands.mesh.indices.size() * sizeof(std::uint32_t), RHI::BufferUsage::Index, true,
            "Trave: fios (indices)" });
        renderer.writeBuffer(goal.strandVertices, 0, std::as_bytes(std::span(strands.mesh.vertices)));
        renderer.writeBuffer(goal.strandIndices, 0, std::as_bytes(std::span(strands.mesh.indices)));
        goal.strandIndexCount = static_cast<std::uint32_t>(strands.mesh.indices.size());
        goal.boundsCenter = goal.position;
        goal.boundsRadius = 0.5f * object->size.length() + NetBulgeMargin;
        goal.previousSkinValid = false;
    } catch (const std::exception& exception) {
        Log::error(std::string("Trave do laboratorio: ") + exception.what());
        releaseLaboratoryGoal(renderer);
    }
}

void WorkbenchApp::releaseLaboratoryGoal(Renderer& renderer) {
    LaboratoryGoal& goal = m_laboratoryGoal;
    if (m_physicsScene) {
        if (goal.net.valid() && m_physicsScene->contains(goal.net)) m_physicsScene->destroyNet(goal.net);
        if (goal.body.valid() && m_physicsScene->contains(goal.body)) m_physicsScene->destroyBody(goal.body);
        for (const PhysicsBodyHandle3D barrier : goal.barriers)
            if (m_physicsScene->contains(barrier)) m_physicsScene->destroyBody(barrier);
    }
    releaseGpuModel3D(renderer, goal.frame);
    if (goal.strandVertices) renderer.destroyBuffer(goal.strandVertices);
    if (goal.strandIndices) renderer.destroyBuffer(goal.strandIndices);
    goal = LaboratoryGoal {};
}

void WorkbenchApp::captureLaboratoryGoalNet() {
    LaboratoryGoal& goal = m_laboratoryGoal;
    if (!goal.object || !m_physicsScene || !m_physicsScene->contains(goal.net)) return;
    goal.previousKnots.swap(goal.knots);
    const std::span<const Vec3> knots = m_physicsScene->netPositions(goal.net);
    goal.knots.assign(knots.begin(), knots.end());
    if (goal.previousKnots.size() != goal.knots.size()) goal.previousKnots = goal.knots;
}

void WorkbenchApp::appendLaboratoryGoalRenderables(std::vector<MeshRender3D>& meshes, const Frustum3D& cameraFrustum,
    std::span<const Frustum3D> lightFrustums, bool temporalHistoryValid) {
    LaboratoryGoal& goal = m_laboratoryGoal;
    goal.renderedThisFrame = false;
    if (!goal.object) return;
    const bool visible = cameraFrustum.intersectsSphere(goal.boundsCenter, goal.boundsRadius);
    std::uint8_t shadowMask = 0;
    for (std::size_t cascade = 0; cascade < lightFrustums.size(); ++cascade)
        if (lightFrustums[cascade].intersectsSphere(goal.boundsCenter, goal.boundsRadius))
            shadowMask |= static_cast<std::uint8_t>(1u << cascade);
    if (!visible && shadowMask == 0) return;
    // A estrutura (parada).
    const std::size_t first = meshes.size();
    appendGpuModelRenderables(goal.frame, goal.position, goal.orientation, goal.position, goal.orientation, 1.0f, false, false,
        shadowMask != 0, meshes, (goal.position - m_laboratoryCameraPosition).length());
    for (std::size_t index = first; index < meshes.size(); ++index) {
        meshes[index].visibleInCamera = visible;
        meshes[index].shadowCascadeMask = shadowMask;
    }
    if (goal.knots.empty() || goal.restKnots.size() != goal.knots.size()) return;
    // Os fios: os nos entre os dois ultimos passos (o mesmo alfa dos props).
    const float alpha = std::clamp(applicationFrameMetrics().fixedStepAlpha, 0.0f, 1.0f);
    goal.renderedKnots.resize(goal.knots.size());
    for (std::size_t i = 0; i < goal.knots.size(); ++i)
        goal.renderedKnots[i] = goal.previousKnots[i] + (goal.knots[i] - goal.previousKnots[i]) * alpha;
    objectNetSkin3D(goal.renderedKnots, goal.restKnots, goal.skin);
    if (!goal.previousSkinValid || !temporalHistoryValid || goal.previousSkin.size() != goal.skin.size()) goal.previousSkin = goal.skin;
    MeshRender3D strands;
    strands.vertexBuffer = goal.strandVertices;
    strands.indexBuffer = goal.strandIndices;
    strands.indexCount = goal.strandIndexCount;
    strands.position = goal.position;
    strands.orientation = goal.orientation;
    strands.previousPosition = goal.position;
    strands.previousOrientation = goal.orientation;
    strands.skinMatrices = goal.skin;
    strands.previousSkinMatrices = goal.previousSkin;
    strands.roughness = 0.9f;
    strands.matteSurface = true;
    strands.pixelArtHighDetail = true;
    // A sombra dos fios: o passe de sombra engrossa o fio fino ate ~0,7 texel
    // da cascata (sem isso, o fio de milimetros virava pontilhado piscando).
    strands.castsShadow = shadowMask != 0;
    strands.shadowCascadeMask = shadowMask;
    strands.thinShadowRadius = 0.5f * goal.object->net->twineDiameter;
    strands.visibleInCamera = visible;
    meshes.push_back(strands);
    goal.renderedThisFrame = true;
}

void WorkbenchApp::finishLaboratoryGoalFrame() {
    LaboratoryGoal& goal = m_laboratoryGoal;
    // O desenhado vira o "anterior" do TAA (na cadencia do desenho).
    if (goal.renderedThisFrame) goal.previousSkin.swap(goal.skin);
    goal.previousSkinValid = goal.renderedThisFrame;
}

} // namespace MatterEngine::Workbench
