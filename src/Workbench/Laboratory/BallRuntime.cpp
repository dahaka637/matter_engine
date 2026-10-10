#include "Workbench/WorkbenchApp.hpp"
#include "Workbench/UiKit.hpp"

#include "Engine/Core/Log.hpp"
#include "Engine/Geometry/ProceduralSoccerBall3D.hpp"
#include "Engine/RHI/RHITypes.hpp"
#include "imgui.h"

#include <algorithm>
#include <cmath>
#include <span>
#include <vector>

namespace MatterEngine::Workbench {
namespace {

constexpr std::size_t MaximumInflatedBalls = 256;
// O pico do painel fica visivel por este tempo depois do chute/impacto.
constexpr float PeakHoldSeconds = 2.5f;

Quaternion blendOrientation(Quaternion from, Quaternion to, float amount) {
    if (from.x * to.x + from.y * to.y + from.z * to.z + from.w * to.w < 0.0f) {
        to = { -to.x, -to.y, -to.z, -to.w };
    }
    return Quaternion {
        from.x + (to.x - from.x) * amount, from.y + (to.y - from.y) * amount,
        from.z + (to.z - from.z) * amount, from.w + (to.w - from.w) * amount
    }.normalized();
}

// Plano em que a esfera de repouso estava achatada neste passo: ele passa a
// (raio - compressao) do centro, contra a normal.
void appendContactPlanes(const InflatedBallState3D& state, float radius,
    std::vector<SoccerBallContactPlane3D>& planes) {
    const std::uint32_t count = std::min(state.contactCount,
        InflatedBallState3D::MaximumContacts);
    for (std::uint32_t index = 0; index < count; ++index) {
        const InflatedBallContact3D& contact = state.contacts[index];
        planes.push_back({ state.position
                - contact.normalWorld * (radius - contact.compressionMeters),
            contact.normalWorld });
    }
}

} // namespace

void WorkbenchApp::ensureSoccerBallVisual(Renderer& activeRenderer) {
    if (!m_soccerBallVisual.parts.empty()) return;
    SoccerBallStyle3D style;
    style.radiusMeters = m_soccerBallProfile.radiusMeters;
    const SoccerBallMesh3D ball = buildProceduralSoccerBall3D(style);
    GpuModelPart3D part;
    part.vertexBuffer = activeRenderer.createBuffer({
        ball.mesh.vertices.size() * sizeof(MeshVertex3D),
        RHI::BufferUsage::Vertex, true, "Soccer ball vertices" });
    activeRenderer.writeBuffer(part.vertexBuffer, 0,
        std::as_bytes(std::span(ball.mesh.vertices)));
    part.indexBuffer = activeRenderer.createBuffer({
        ball.mesh.indices.size() * sizeof(std::uint32_t),
        RHI::BufferUsage::Index, true, "Soccer ball indices" });
    activeRenderer.writeBuffer(part.indexBuffer, 0,
        std::as_bytes(std::span(ball.mesh.indices)));
    part.indexCount = static_cast<std::uint32_t>(ball.mesh.indices.size());
    for (const std::vector<std::uint32_t>& indices : ball.lods) {
        GpuMeshLod3D lod;
        lod.indexBuffer = activeRenderer.createBuffer({
            indices.size() * sizeof(std::uint32_t),
            RHI::BufferUsage::Index, true, "Soccer ball LOD indices" });
        activeRenderer.writeBuffer(lod.indexBuffer, 0,
            std::as_bytes(std::span(indices)));
        lod.indexCount = static_cast<std::uint32_t>(indices.size());
        part.lods.push_back(lod);
    }
    // Couro sintetico (PU) semi-brilhante.
    part.metallic = 0.0f;
    part.roughness = 0.42f;
    m_soccerBallVisual.parts.push_back(part);
    m_soccerBallVisual.dimensionsMeters = ball.mesh.boundsMax - ball.mesh.boundsMin;
    m_soccerBallCage = ball.cageDirections;
    m_soccerBallRadius = ball.radiusMeters;
}

bool WorkbenchApp::spawnInflatedBallAt(Vec3 position) {
    if (!m_physicsScene || !m_laboratoryMapLoaded
        || m_inflatedBalls.size() >= MaximumInflatedBalls) {
        return false;
    }
    const float radius = m_soccerBallProfile.radiusMeters;
    constexpr float ClearanceMeters = 0.025f;
    // As bolas nao estao no broad phase: a busca de espaco livre nao as ve.
    // Uma bola nova em cima de outra nasce acima dela e cai.
    for (int attempt = 0; attempt < 8; ++attempt) {
        const bool overlapsBall = std::any_of(m_inflatedBalls.begin(),
            m_inflatedBalls.end(), [&](const SpawnedInflatedBallInstance& other) {
                return (other.physicsState.position - position).length()
                    < 2.0f * radius + ClearanceMeters;
            });
        if (!overlapsBall) break;
        position.z += 2.0f * radius + ClearanceMeters;
    }
    const Vec3 halfExtents { radius + ClearanceMeters, radius + ClearanceMeters,
        radius + ClearanceMeters };
    if (m_physicsScene->overlapsBox(position, halfExtents, {})) return false;

    InflatedBallDefinition3D definition;
    definition.profile = m_soccerBallProfile;
    definition.position = position;
    definition.entityId = m_nextEntityId++;
    // A bola colide com os bonecos, menos o dono dela (quem a tocou ou
    // interagiu com ela por ultimo) - ver updateActiveRagdolls.
    definition.collidesWithRagdolls = true;
    const InflatedBallHandle3D handle =
        m_physicsScene->createInflatedBall(definition);
    if (!handle) return false;
    SpawnedInflatedBallInstance instance;
    instance.entityId = definition.entityId;
    instance.physicsBall = handle;
    instance.physicsState = m_physicsScene->inflatedBallState(handle);
    instance.simulationPreviousState = instance.physicsState;
    instance.renderedPosition = position;
    instance.renderedOrientation = instance.physicsState.orientation;
    instance.previousRenderedPosition = position;
    instance.previousRenderedOrientation = instance.physicsState.orientation;
    m_inflatedBalls.push_back(std::move(instance));
    m_laboratoryStatus = "Bola de futebol criada";
    return true;
}

void WorkbenchApp::readInflatedBallStates(float deltaTime) {
    if (!m_physicsScene) return;
    for (SpawnedInflatedBallInstance& ball : m_inflatedBalls) {
        if (!m_physicsScene->contains(ball.physicsBall)) continue;
        ball.simulationPreviousState = ball.physicsState;
        ball.physicsState = m_physicsScene->inflatedBallState(ball.physicsBall);
        // Um impacto de verdade (mais de 1 mm) abre uma janela nova; nela
        // ficam o maior achatamento e a maior velocidade (a de saida do chute).
        const float compression = ball.physicsState.compressionMeters;
        ball.recentPeakAgeSeconds += deltaTime;
        if (compression > 0.001f && ball.recentPeakAgeSeconds > PeakHoldSeconds) {
            ball.recentPeakCompressionMeters = 0.0f;
            ball.recentPeakSpeed = 0.0f;
            ball.recentPeakAgeSeconds = 0.0f;
        }
        if (ball.recentPeakAgeSeconds <= PeakHoldSeconds) {
            ball.recentPeakCompressionMeters =
                std::max(ball.recentPeakCompressionMeters, compression);
            ball.recentPeakSpeed = std::max(ball.recentPeakSpeed,
                ball.physicsState.linearVelocity.length());
        }
    }
}

void WorkbenchApp::appendInflatedBallRenderables(
    std::vector<MeshRender3D>& meshes, const Frustum3D& cameraFrustum,
    std::span<const Frustum3D> lightFrustums, bool temporalHistoryValid) {
    if (m_inflatedBalls.empty() || m_soccerBallVisual.parts.empty()) return;
    const float alpha = std::clamp(
        applicationFrameMetrics().fixedStepAlpha, 0.0f, 1.0f);
    const float radius = m_soccerBallRadius;
    std::vector<SoccerBallContactPlane3D> previousPlanes, currentPlanes, planes;
    for (SpawnedInflatedBallInstance& ball : m_inflatedBalls) {
        const InflatedBallState3D& from = ball.simulationPreviousState;
        const InflatedBallState3D& to = ball.physicsState;
        ball.renderedPosition = from.position
            + (to.position - from.position) * alpha;
        ball.renderedOrientation = blendOrientation(from.orientation,
            to.orientation, alpha);
        if (!temporalHistoryValid) {
            ball.previousRenderedPosition = ball.renderedPosition;
            ball.previousRenderedOrientation = ball.renderedOrientation;
            ball.previousSkinned = false;
        }
        if (m_physGunGrabbedEntityId != 0
            && ball.entityId == m_physGunGrabbedEntityId) {
            m_physGunBeamTargetWorldPosition = ball.renderedPosition;
        }
        ball.skinMatrices.clear();

        const float cullRadius = radius + 0.01f;
        const bool visible = cameraFrustum.intersectsSphere(
            ball.renderedPosition, cullRadius);
        std::uint8_t shadowMask = 0;
        for (std::size_t cascade = 0; cascade < lightFrustums.size(); ++cascade) {
            if (lightFrustums[cascade].intersectsSphere(
                    ball.renderedPosition, cullRadius)) {
                shadowMask |= static_cast<std::uint8_t>(1u << cascade);
            }
        }
        if (!visible && shadowMask == 0) continue;

        // Achatamento no instante desenhado: os planos de contato dos dois
        // passos (o mesmo contato e interpolado; um que comecou ou acabou
        // entra como esta) contra o centro interpolado. A bola subindo do
        // chao desamassa junto com o movimento, sem degrau a 120 Hz.
        previousPlanes.clear();
        currentPlanes.clear();
        planes.clear();
        appendContactPlanes(from, radius, previousPlanes);
        appendContactPlanes(to, radius, currentPlanes);
        std::vector<bool> matched(previousPlanes.size(), false);
        for (const SoccerBallContactPlane3D& current : currentPlanes) {
            bool merged = false;
            for (std::size_t index = 0; index < previousPlanes.size(); ++index) {
                const SoccerBallContactPlane3D& previous = previousPlanes[index];
                if (matched[index]
                    || dot(previous.normalWorld, current.normalWorld) < 0.95f
                    || std::abs(dot(current.pointWorld - previous.pointWorld,
                        current.normalWorld)) > 0.03f) {
                    continue;
                }
                matched[index] = true;
                merged = true;
                planes.push_back({ previous.pointWorld
                        + (current.pointWorld - previous.pointWorld) * alpha,
                    (previous.normalWorld
                        + (current.normalWorld - previous.normalWorld) * alpha)
                        .normalized() });
                break;
            }
            if (!merged) planes.push_back(current);
        }
        for (std::size_t index = 0; index < previousPlanes.size(); ++index) {
            if (!matched[index]) planes.push_back(previousPlanes[index]);
        }
        const bool deformed = std::any_of(planes.begin(), planes.end(),
            [&](const SoccerBallContactPlane3D& plane) {
                return soccerBallPlaneCompression3D(ball.renderedPosition,
                    radius, plane) > SoccerBallVisibleCompressionMeters;
            });
        if (deformed) {
            buildSoccerBallSkin3D(m_soccerBallCage, radius,
                ball.renderedPosition, ball.renderedOrientation, planes,
                ball.skinMatrices);
            if (!ball.previousSkinned
                || ball.previousSkinMatrices.size() != ball.skinMatrices.size()) {
                if (temporalHistoryValid) {
                    buildSoccerBallSkin3D(m_soccerBallCage, radius,
                        ball.previousRenderedPosition,
                        ball.previousRenderedOrientation, {},
                        ball.previousSkinMatrices);
                } else {
                    ball.previousSkinMatrices = ball.skinMatrices;
                }
            }
        }

        const bool highlighted = ball.entityId == m_physGunGrabbedEntityId
            || ball.freezeFlashSeconds > 0.0f;
        const std::size_t first = meshes.size();
        appendGpuModelRenderables(m_soccerBallVisual, ball.renderedPosition,
            ball.renderedOrientation, ball.previousRenderedPosition,
            ball.previousRenderedOrientation, 1.0f, highlighted, highlighted,
            shadowMask != 0, meshes,
            (ball.renderedPosition - m_laboratoryCameraPosition).length());
        for (std::size_t index = first; index < meshes.size(); ++index) {
            MeshRender3D& mesh = meshes[index];
            mesh.visibleInCamera = visible;
            mesh.shadowCascadeMask = shadowMask;
            mesh.pixelArtHighDetail = true;
            if (deformed) {
                mesh.skinMatrices = ball.skinMatrices;
                mesh.previousSkinMatrices = ball.previousSkinMatrices;
            }
        }
    }
}

void WorkbenchApp::finishInflatedBallFrame() {
    for (SpawnedInflatedBallInstance& ball : m_inflatedBalls) {
        ball.previousRenderedPosition = ball.renderedPosition;
        ball.previousRenderedOrientation = ball.renderedOrientation;
        ball.previousSkinned = !ball.skinMatrices.empty();
        if (ball.previousSkinned) ball.previousSkinMatrices.swap(ball.skinMatrices);
    }
}

namespace {

// O raio da bola no tamanho 1 (m): 28 cm de diametro.
constexpr float SoccerBallLargestRadius = 0.14f;

// O perfil da bola no tamanho dado (0..1): o oficial (na pressao do menu)
// levado ao raio dele com o mesmo jeito (InflatedBallModel3D::scaledProfile).
InflatedBallProfile3D soccerBallProfileForSize(float size, float baseGaugeBar) {
    InflatedBallProfile3D base;
    base.gaugePressurePascals = baseGaugeBar * 100000.0f;
    const float radius = base.radiusMeters
        + (SoccerBallLargestRadius - base.radiusMeters) * std::clamp(size, 0.0f, 1.0f);
    return InflatedBallModel3D::scaledProfile(base, radius);
}

} // namespace

void WorkbenchApp::applySoccerBallSize() {
    const InflatedBallProfile3D sized = soccerBallProfileForSize(m_soccerBallSize, m_soccerBallBaseGaugeBar);
    const float previousRadius = m_soccerBallProfile.radiusMeters;
    m_soccerBallProfile.radiusMeters = sized.radiusMeters;
    m_soccerBallProfile.massKg = sized.massKg;
    m_soccerBallProfile.gaugePressurePascals = sized.gaugePressurePascals;
    m_soccerBallProfile.dampingReferencePressurePascals = sized.dampingReferencePressurePascals;
    m_soccerBallProfile.rollingResistanceMeters = sized.rollingResistanceMeters;
    if (m_physicsScene) {
        for (const SpawnedInflatedBallInstance& ball : m_inflatedBalls) {
            InflatedBallProfile3D profile = m_physicsScene->inflatedBallProfile(ball.physicsBall);
            const float grow = sized.radiusMeters - profile.radiusMeters;
            profile.radiusMeters = sized.radiusMeters;
            profile.massKg = sized.massKg;
            profile.gaugePressurePascals = sized.gaugePressurePascals;
            profile.dampingReferencePressurePascals = sized.dampingReferencePressurePascals;
            profile.rollingResistanceMeters = sized.rollingResistanceMeters;
            m_physicsScene->setInflatedBallProfile(ball.physicsBall, profile);
            // (Crescendo no chao, sobe a diferenca: nasceria enterrada e saltava.)
            if (grow > 0.0f) {
                const InflatedBallState3D state = m_physicsScene->inflatedBallState(ball.physicsBall);
                m_physicsScene->setInflatedBallMotion(ball.physicsBall, state.position + Vec3 { 0.0f, 0.0f, grow },
                    state.linearVelocity, state.angularVelocity);
            }
        }
    }
    // A malha da bola e refeita no proximo quadro (no raio novo).
    if (std::abs(sized.radiusMeters - previousRadius) > 1e-5f) m_soccerBallVisualDirty = true;
}

void WorkbenchApp::drawInflatedBallPanel() {
    panelHeader("CAMERA LENTA");
    float scale = timeScale();
    ImGui::TextColored({ 0.50f, 0.62f, 0.72f, 1.0f },
        "Velocidade do tempo (a fisica e a mesma, so mais devagar)");
    ImGui::SetNextItemWidth(-1.0f);
    if (ImGui::SliderFloat("##TimeScale", &scale, 0.05f, 1.0f, "%.2fx",
            ImGuiSliderFlags_AlwaysClamp)) {
        setTimeScale(scale);
    }
    if (ImGui::Button("Tempo real")) setTimeScale(1.0f);
    ImGui::SameLine();
    if (ImGui::Button("0,25x")) setTimeScale(0.25f);
    ImGui::SameLine();
    if (ImGui::Button("0,1x")) setTimeScale(0.1f);
    ImGui::Dummy({ 1.0f, ui(10.0f) });
    panelHeader("BOLA DE FUTEBOL");
    // IFAB: 0,6 a 1,1 atm acima da atmosfera; a FIFA testa a 0,8 bar. (A da
    // bola oficial; maior, a efetiva cai com o tamanho - applySoccerBallSize.)
    ImGui::TextColored({ 0.50f, 0.62f, 0.72f, 1.0f },
        "Pressão (bar acima da atmosfera; regra 0,6-1,1)");
    ImGui::SetNextItemWidth(-1.0f);
    if (ImGui::SliderFloat("##SoccerBallPressure", &m_soccerBallBaseGaugeBar, 0.2f, 1.3f,
            "%.2f bar", ImGuiSliderFlags_AlwaysClamp)) {
        applySoccerBallSize();
    }
    // Tamanho (07/10): 0 a oficial, 1 com 28 cm; o peso, a pressao e o rolar
    // acompanham. Aplica ao soltar (a malha e refeita no raio novo).
    ImGui::TextColored({ 0.50f, 0.62f, 0.72f, 1.0f },
        "Tamanho (0: oficial, 22 cm; 1: 28 cm)");
    ImGui::SetNextItemWidth(-1.0f);
    ImGui::SliderFloat("##SoccerBallSize", &m_soccerBallSize, 0.0f, 1.0f, "%.2f",
        ImGuiSliderFlags_AlwaysClamp);
    if (ImGui::IsItemDeactivatedAfterEdit()) applySoccerBallSize();
    if (ImGui::IsItemHovered()) {
        ImGui::BeginTooltip();
        ImGui::PushTextWrapPos(ImGui::GetFontSize() * 24.0f);
        ImGui::TextUnformatted("O tamanho da bola de futebol (todas no mapa e as novas). O peso cresce com a área do couro, a pressão cai com o tamanho (no mesmo impacto ela achata o mesmo em relação ao tamanho) e o rolar acompanha: ela quica, rola e voa como a oficial, só maior. Vale ao soltar o controle.");
        ImGui::PopTextWrapPos();
        ImGui::EndTooltip();
    }
    {
        const InflatedBallProfile3D preview = soccerBallProfileForSize(m_soccerBallSize, m_soccerBallBaseGaugeBar);
        ImGui::TextColored({ 0.45f, 0.55f, 0.65f, 1.0f }, "%.1f cm, %.0f g, %.2f bar",
            preview.radiusMeters * 200.0f, preview.massKg * 1000.0f, preview.gaugePressurePascals / 100000.0f);
    }
    if (m_inflatedBalls.empty()) {
        ImGui::TextColored({ 0.45f, 0.55f, 0.65f, 1.0f },
            "Nenhuma bola nova no mapa.");
        return;
    }
    // A segurada pela Physgun; senao a ultima criada.
    const SpawnedInflatedBallInstance* shown = &m_inflatedBalls.back();
    for (const SpawnedInflatedBallInstance& ball : m_inflatedBalls) {
        if (ball.entityId == m_physGunGrabbedEntityId) shown = &ball;
    }
    const InflatedBallState3D& state = shown->physicsState;
    const float speed = state.linearVelocity.length();
    ImGui::Text("Velocidade: %.1f m/s (%.0f km/h)", speed, speed * 3.6f);
    ImGui::Text("Giro: %.1f rot/s", state.angularVelocity.length()
        / (2.0f * 3.14159265f));
    ImGui::Text("Pressão interna: %.3f bar", state.gaugePressurePascals / 100000.0f);
    ImGui::Text("Achatamento: %.1f mm (contatos %u)",
        state.compressionMeters * 1000.0f, state.contactCount);
    ImGui::Text("Último impacto: %.1f mm, %.1f m/s (%.0f km/h)",
        shown->recentPeakCompressionMeters * 1000.0f, shown->recentPeakSpeed,
        shown->recentPeakSpeed * 3.6f);
    if (state.frozen) {
        ImGui::TextColored({ 0.55f, 0.80f, 1.0f, 1.0f }, "Congelada");
    }
    ImGui::Text("Bolas novas no mapa: %zu", m_inflatedBalls.size());
}

} // namespace MatterEngine::Workbench
