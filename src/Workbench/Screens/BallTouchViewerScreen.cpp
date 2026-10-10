#include "Workbench/WorkbenchApp.hpp"
#include "Workbench/UiKit.hpp"

#include "Engine/Core/Log.hpp"
#include "Engine/Math/Mat4.hpp"
#include "Engine/UI/FontAwesome.hpp"
#include "imgui.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

// Visualizador de toque na bola (pedido do usuario, 04/10): a marcha (o
// clipe tocando), o quadro do clipe em que sai o pedido, a bola em relacao
// ao corpo (X, Y, Z), o tipo (manejo ou chute), a direcao e a forca. A
// simulacao e a do jogo (BallTouchPreview3D; desde 06/10 na locomocao padrao,
// o corpo pela animacao, para ver o movimento limpo) e o resultado toca em
// loop ou pelo scrubber, com os rastros dos pes, da bola e do alvo que o
// manejo deu ao pe.

namespace MatterEngine::Workbench {
namespace {

// O chute vai para a frente: a direcao dele no visualizador ate isto (graus)
// para cada lado.
constexpr float ShotDirectionLimitDegrees = 90.0f;


constexpr float Pi = 3.14159265358979323846f;
constexpr int PreviewWidth = 1280;
constexpr int PreviewHeight = 752;
constexpr ImU32 LeftColor = IM_COL32(86, 150, 255, 255);
constexpr ImU32 RightColor = IM_COL32(255, 150, 70, 255);
constexpr ImU32 BallColor = IM_COL32(240, 240, 240, 255);
constexpr ImU32 GoalColor = IM_COL32(255, 60, 60, 255);
constexpr ImU32 PlantColor = IM_COL32(70, 220, 120, 255);
constexpr ImU32 DirectionColor = IM_COL32(200, 90, 255, 255);

bool projectPoint(const Mat4& viewProjection, Vec3 point, ImVec2 origin, ImVec2 size, ImVec2& out) {
    const float clipX = viewProjection.at(0, 0) * point.x + viewProjection.at(0, 1) * point.y
        + viewProjection.at(0, 2) * point.z + viewProjection.at(0, 3);
    const float clipY = viewProjection.at(1, 0) * point.x + viewProjection.at(1, 1) * point.y
        + viewProjection.at(1, 2) * point.z + viewProjection.at(1, 3);
    const float clipW = viewProjection.at(3, 0) * point.x + viewProjection.at(3, 1) * point.y
        + viewProjection.at(3, 2) * point.z + viewProjection.at(3, 3);
    if (clipW <= 0.001f) return false;
    out = { origin.x + (clipX / clipW * 0.5f + 0.5f) * size.x,
        origin.y + (0.5f - clipY / clipW * 0.5f) * size.y };
    return true;
}

const char* surfaceLabel(BallTouchSurface3D surface) {
    switch (surface) {
    case BallTouchSurface3D::Laces: return "peito do pé";
    case BallTouchSurface3D::Inside: return "parte de dentro";
    case BallTouchSurface3D::Outside: return "parte de fora";
    case BallTouchSurface3D::Sole: return "sola";
    case BallTouchSurface3D::Heel: return "calcanhar";
    }
    return "?";
}

const char* phaseLabel(BallControlPhase3D phase) {
    switch (phase) {
    case BallControlPhase3D::Idle: return "livre";
    case BallControlPhase3D::Waiting: return "esperando o pé";
    case BallControlPhase3D::Windup: return "armando";
    case BallControlPhase3D::Reaching: return "indo à bola";
    case BallControlPhase3D::FollowThrough: return "acompanhando";
    }
    return "?";
}

const char* missLabel(int reason) {
    switch (reason) {
    case 1: return "bola longe";
    case 2: return "o pé não chegou a tempo";
    case 3: return "o pedido acabou sem toque";
    default: return "";
    }
}

std::size_t frameAt(const BallTouchPreviewResult3D& result, float seconds) {
    const auto& frames = result.frames;
    if (frames.empty()) return 0;
    const auto it = std::lower_bound(frames.begin(), frames.end(), seconds,
        [](const BallTouchPreviewFrame3D& frame, float t) { return frame.timeSeconds < t; });
    if (it == frames.end()) return frames.size() - 1;
    return static_cast<std::size_t>(it - frames.begin());
}

int clipFrameCount(const BallTouchPreviewResult3D& result, const AnimationClip3D* clip) {
    if (clip == nullptr) return 0;
    const float rate = clip->sourceSampleRateHz > 0.0f ? clip->sourceSampleRateHz : 30.0f;
    static_cast<void>(result);
    return std::max(1, static_cast<int>(std::lround(clip->durationSeconds * rate)));
}

bool sameSettings(const BallTouchPreviewSettings3D& a, const BallTouchPreviewSettings3D& b) {
    return a.gait == b.gait && a.requestPhase == b.requestPhase && a.ballOffset.x == b.ballOffset.x
        && a.ballOffset.y == b.ballOffset.y && a.ballOffset.z == b.ballOffset.z && a.ballRolling == b.ballRolling
        && a.kind == b.kind && a.directionDegrees == b.directionDegrees && a.power == b.power
        && a.shotLift == b.shotLift && a.shotCurve == b.shotCurve && a.areaTouch == b.areaTouch
        && a.contactTimingFactor == b.contactTimingFactor
        && a.motionVariety == b.motionVariety
        && a.recordBeforeSeconds == b.recordBeforeSeconds
        && a.recordAfterSeconds == b.recordAfterSeconds;
}

void drawArrow(ImDrawList* list, ImVec2 from, ImVec2 to, ImU32 color, float thickness) {
    list->AddLine(from, to, color, thickness);
    const float dx = to.x - from.x, dy = to.y - from.y;
    const float length = std::sqrt(dx * dx + dy * dy);
    if (length < 1.0f) return;
    const float ux = dx / length, uy = dy / length;
    const float head = std::min(14.0f, length * 0.4f) * thickness / 2.5f;
    list->AddTriangleFilled(to, { to.x - ux * head - uy * head * 0.55f, to.y - uy * head + ux * head * 0.55f },
        { to.x - ux * head + uy * head * 0.55f, to.y - uy * head - ux * head * 0.55f }, color);
}

} // namespace

void WorkbenchApp::stopBallTouchSimulation() {
    if (!m_ballViewerJob.valid()) return;
    m_ballViewerCancel.store(true);
    m_ballViewerJob.wait();
    m_ballViewerJob = {};
}

void WorkbenchApp::updateBallTouchViewer(float deltaTime) {
    if (!m_ballViewer && m_ragdollCharacter && !m_animationClips.empty() && m_ballViewerError.empty()) {
        try {
            m_ballViewer = std::make_unique<BallTouchPreview3D>(*m_ragdollCharacter, m_animationClips);
        } catch (const std::exception& error) {
            m_ballViewerError = std::string("Falha ao preparar a simulação: ") + error.what();
            Log::error(m_ballViewerError);
        }
    }
    // A simulacao terminou: o resultado entra mantendo o instante (relativo
    // ao pedido) que estava na tela - mexer num parametro com o scrubber
    // parado compara o mesmo momento.
    if (m_ballViewerJob.valid()
        && m_ballViewerJob.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
        BallTouchPreviewResult3D result = m_ballViewerJob.get();
        m_ballViewerJob = {};
        if (result.valid) {
            const bool first = !m_ballViewerResult.valid;
            m_ballViewerResult = std::move(result);
            m_ballViewerError.clear();
            m_ballViewerVisualRotations.clear();
            if (m_ragdollCharacter) {
                RagdollVisualPose3D visualPose;
                const auto& frames = m_ballViewerResult.frames;
                m_ballViewerVisualRotations.reserve(frames.size());
                for (std::size_t index = 0; index < frames.size(); ++index) {
                    RagdollVisualPoseInput3D visualInput;
                    visualInput.linkPositions = frames[index].linkPositions;
                    visualInput.linkOrientations = frames[index].linkOrientations;
                    visualInput.deltaTime = index > 0
                        ? frames[index].timeSeconds - frames[index - 1].timeSeconds : 0.0f;
                    m_ballViewerVisualRotations.push_back(visualPose.update(*m_ragdollCharacter, visualInput));
                }
            }
            if (first && m_ballViewerPlaying)
                m_ballViewerPlaybackSeconds = m_ballViewerResult.frames.front().timeSeconds;
        } else if (result.error != "cancelado") {
            m_ballViewerError = result.error;
        }
    }
    // Pede outra quando os parametros mudaram (sem cancelar a que esta
    // rodando: arrastando um slider, os resultados vao chegando).
    if (m_ballViewerDirty) m_ballViewerDirtySeconds += deltaTime;
    if (m_ballViewerDirty && !m_ballViewerJob.valid() && m_ballViewer && m_ballViewerDirtySeconds >= 0.03f) {
        m_ballViewerDirty = false;
        m_ballViewerCancel.store(false);
        m_ballViewerProgress.store(0.0f);
        const BallTouchPreviewSettings3D settings = m_ballViewerSettings;
        BallTouchPreview3D* preview = m_ballViewer.get();
        std::atomic<bool>* cancel = &m_ballViewerCancel;
        std::atomic<float>* progress = &m_ballViewerProgress;
        m_ballViewerJob = std::async(std::launch::async, [preview, settings, cancel, progress] {
            return preview->run(settings, cancel, progress);
        });
    }
    if (m_ballViewerResult.valid && m_ballViewerPlaying && !m_ballViewerResult.frames.empty()) {
        const float start = m_ballViewerResult.frames.front().timeSeconds;
        const float end = m_ballViewerResult.frames.back().timeSeconds;
        m_ballViewerPlaybackSeconds += deltaTime * m_ballViewerPlaybackSpeed;
        if (m_ballViewerPlaybackSeconds > end) {
            if (m_ballViewerLoop) {
                m_ballViewerPlaybackSeconds = start;
            } else {
                m_ballViewerPlaybackSeconds = end;
                m_ballViewerPlaying = false;
            }
        }
    }
}

UiTexture WorkbenchApp::renderBallTouchViewerPreview(Renderer& activeRenderer) {
    if (!m_ragdollCharacter || m_characterVisual.parts.empty() || !m_ballViewerResult.valid
        || m_ballViewerResult.frames.empty()) {
        return {};
    }
    const RagdollProfile3D& profile = m_ragdollCharacter->profile;
    const BallTouchPreviewResult3D& result = m_ballViewerResult;
    const std::size_t frameIndex = frameAt(result, m_ballViewerPlaybackSeconds);
    const BallTouchPreviewFrame3D& frame = result.frames[frameIndex];
    if (frame.linkPositions.size() != profile.links.size()) return {};

    std::vector<MeshRender3D> meshes;
    // A paleta vive ate o render (MeshRender3D so guarda um span dela).
    const std::vector<Mat4> palette = buildRagdollSkinMatrices3D(*m_ragdollCharacter,
        frame.linkPositions, frame.linkOrientations,
        frameIndex < m_ballViewerVisualRotations.size()
            ? std::span<const Quaternion>(m_ballViewerVisualRotations[frameIndex]) : std::span<const Quaternion> {});
    if (!m_ballViewerShowColliders) {
        appendGpuModelRenderables(m_characterVisual, {}, {}, {}, {}, 1.0f, false, false, true, meshes);
        for (auto& mesh : meshes) {
            mesh.skinMatrices = palette;
            mesh.previousSkinMatrices = palette;
            mesh.flatShaded = m_ragdollCharacter->flatShaded;
            mesh.facet = m_ragdollCharacter->facet;
        }
    } else {
        for (std::size_t index = 0; index < profile.links.size() && index < m_ragdollLinkVisuals.size(); ++index) {
            appendGpuModelRenderables(m_ragdollLinkVisuals[index],
                frame.linkPositions[index], frame.linkOrientations[index],
                frame.linkPositions[index], frame.linkOrientations[index],
                1.0f, false, false, true, meshes);
        }
    }
    if (frame.ballPresent && !m_soccerBallVisual.parts.empty()) {
        // (A malha e a do tamanho escolhido no laboratorio; aqui a bola e a
        // oficial.)
        const float scale = InflatedBallProfile3D {}.radiusMeters / std::max(0.01f, m_soccerBallRadius);
        appendGpuModelRenderables(m_soccerBallVisual, frame.ballPosition, frame.ballOrientation,
            frame.ballPosition, frame.ballOrientation, scale, false, false, true, meshes);
    }

    // A camera segue a pelve (ou fica na do pedido).
    const Vec3 pelvis = m_ballViewerFollowCamera ? frame.linkPositions.front() : result.pelvisAtRequest;
    const Vec3 cameraTarget { pelvis.x, pelvis.y, 0.72f };
    // Chao: o quadriculado de 10 m do visualizador de objetos, em 3x3
    // ladrilhos em volta da camera (o jogador anda varios metros).
    ensureViewerFloorLoaded(activeRenderer);
    const float tileX = std::round(cameraTarget.x / 10.0f) * 10.0f;
    const float tileY = std::round(cameraTarget.y / 10.0f) * 10.0f;
    for (int i = -1; i <= 1; ++i) {
        for (int j = -1; j <= 1; ++j) {
            const Vec3 at { tileX + 10.0f * static_cast<float>(i), tileY + 10.0f * static_cast<float>(j), 0.0f };
            appendGpuModelRenderables(m_viewerFloor, at, {}, at, {}, 1.0f, false, false, true, meshes);
        }
    }

    const float planarDistance = std::cos(m_ballViewerCameraPitch) * m_ballViewerCameraDistance;
    const Vec3 cameraPosition = cameraTarget + Vec3 {
        std::cos(m_ballViewerCameraYaw) * planarDistance,
        std::sin(m_ballViewerCameraYaw) * planarDistance,
        std::sin(m_ballViewerCameraPitch) * m_ballViewerCameraDistance };
    const Mat4 view = Mat4::lookAt(cameraPosition, cameraTarget, { 0.0f, 0.0f, 1.0f });
    const Mat4 projection = Mat4::perspective(38.0f * Pi / 180.0f,
        static_cast<float>(PreviewWidth) / static_cast<float>(PreviewHeight), 0.05f, 60.0f);
    m_ballViewerViewProjection = projection * view;

    std::array<LightRender3D, 2> lights;
    lights[0].type = LightType3D::Point;
    lights[0].position = cameraTarget + Vec3 { -2.6f, -3.4f, 5.6f };
    lights[0].color = { 0.82f, 0.92f, 1.0f };
    lights[0].intensity = 1.25f;
    lights[0].range = 14.0f;
    lights[0].castsShadow = true;
    lights[1].type = LightType3D::Point;
    lights[1].position = cameraTarget + Vec3 { 3.8f, 2.4f, 2.3f };
    lights[1].color = { 0.34f, 0.60f, 1.0f };
    lights[1].intensity = 0.58f;
    lights[1].range = 10.0f;

    Scene3DFrame scene;
    scene.cameraViewProjection = m_ballViewerViewProjection;
    scene.cameraViewProjectionJittered = scene.cameraViewProjection;
    scene.previousCameraViewProjection = scene.cameraViewProjection;
    scene.cameraPosition = cameraPosition;
    scene.lights = lights;
    scene.meshes = meshes;
    scene.environment.skyIrradiance = 0.31f;
    scene.showSky = false;
    scene.showShadows = true;
    return activeRenderer.renderScene3D(scene, PreviewWidth, PreviewHeight);
}

void WorkbenchApp::drawBallTouchViewer() {
    beginFullscreenPanel("MatterEngineBallTouchViewer");
    const ImGuiIO& io = ImGui::GetIO();
    ImDrawList* drawList = ImGui::GetWindowDrawList();
    drawWorkbenchBackdrop(drawList, io.DisplaySize, m_uiScale);
    drawWorkbenchHeader(drawList, io.DisplaySize, m_uiScale, "TOQUE NA BOLA", "PREDIÇÃO DE MOVIMENTO");

    const float margin = ui(12.0f);
    const float top = headerHeight(m_uiScale) + margin;
    const float height = io.DisplaySize.y - top - margin;
    const float panelWidth = std::max(sidebarWidth(m_uiScale), ui(360.0f));
    BallTouchPreviewSettings3D& settings = m_ballViewerSettings;
    const BallTouchPreviewSettings3D before = settings;

    // ---- Painel de parametros ------------------------------------------
    ImGui::SetCursorPos({ margin, top });
    ImGui::BeginChild("BallTouchControls", { panelWidth, height }, true);
    panelHeader("CENÁRIO DO TOQUE");
    ImGui::Dummy({ 1.0f, ui(4.0f) });

    sectionLabel("ANIMAÇÃO TOCANDO (MARCHA)");
    const float gaitWidth = (ImGui::GetContentRegionAvail().x - ImGui::GetStyle().ItemSpacing.x) * 0.5f;
    for (std::size_t i = 0; i < static_cast<std::size_t>(BallPreviewGait3D::Count); ++i) {
        const auto gait = static_cast<BallPreviewGait3D>(i);
        ImGui::PushID(static_cast<int>(i));
        const float width = gaitWidth;
        if (navigationButton(ballPreviewGaitName(gait), settings.gait == gait, { width, ui(30.0f) }))
            settings.gait = gait;
        if (i % 2 == 0) ImGui::SameLine();
        ImGui::PopID();
    }

    const AnimationClip3D* gaitClip = m_ballViewer ? m_ballViewer->gaitClip(settings.gait) : nullptr;
    if (settings.gait != BallPreviewGait3D::Idle && gaitClip != nullptr) {
        ImGui::Dummy({ 1.0f, ui(4.0f) });
        sectionLabel("QUADRO DO CLIPE NO PEDIDO");
        const int frames = clipFrameCount(m_ballViewerResult, gaitClip);
        // O quadro sai da fase (trocando de marcha, a mesma fase em outro
        // clipe).
        m_ballViewerClipFrame = static_cast<int>(std::lround(settings.requestPhase * static_cast<float>(frames)))
            % frames;
        ImGui::SetNextItemWidth(-1.0f);
        if (ImGui::SliderInt("##ClipFrame", &m_ballViewerClipFrame, 0, frames - 1, "quadro %d"))
            settings.requestPhase = static_cast<float>(m_ballViewerClipFrame) / static_cast<float>(frames);
        if (ImGui::Button("-##ClipFrameDown", { ui(30.0f), 0.0f })) {
            m_ballViewerClipFrame = (m_ballViewerClipFrame + frames - 1) % frames;
            settings.requestPhase = static_cast<float>(m_ballViewerClipFrame) / static_cast<float>(frames);
        }
        ImGui::SameLine();
        if (ImGui::Button("+##ClipFrameUp", { ui(30.0f), 0.0f })) {
            m_ballViewerClipFrame = (m_ballViewerClipFrame + 1) % frames;
            settings.requestPhase = static_cast<float>(m_ballViewerClipFrame) / static_cast<float>(frames);
        }
        ImGui::SameLine();
        ImGui::TextColored({ 0.43f, 0.62f, 0.76f, 1.0f }, "%s  |  %d quadros  |  fase %.2f",
            gaitClip->id.c_str(), frames, settings.requestPhase);
    }

    ImGui::Dummy({ 1.0f, ui(4.0f) });
    sectionLabel("AÇÃO");
    {
        const float width = (ImGui::GetContentRegionAvail().x - ImGui::GetStyle().ItemSpacing.x) * 0.5f;
        if (navigationButton("Manejo (drible)", settings.kind == BallActionKind3D::Dribble, { width, ui(32.0f) }))
            settings.kind = BallActionKind3D::Dribble;
        ImGui::SameLine();
        if (navigationButton("Chute", settings.kind == BallActionKind3D::Shot, { width, ui(32.0f) }))
            settings.kind = BallActionKind3D::Shot;
    }
    // O chute e sempre para a frente (pedido do usuario, 04/10): a direcao
    // dele ate 90 graus para cada lado; o drible, para qualquer lado.
    const float directionLimit = settings.kind == BallActionKind3D::Shot ? ShotDirectionLimitDegrees : 180.0f;
    settings.directionDegrees = std::clamp(settings.directionDegrees, -directionLimit, directionLimit);
    ImGui::SetNextItemWidth(-ui(96.0f));
    ImGui::SliderFloat("Direção", &settings.directionDegrees, -directionLimit, directionLimit, "%.0f°");
    ImGui::SetNextItemWidth(-ui(96.0f));
    ImGui::SliderFloat("Força", &settings.power, 0.0f, 1.0f, "%.2f");
    if (settings.kind == BallActionKind3D::Shot) {
        ImGui::SetNextItemWidth(-ui(96.0f));
        ImGui::SliderFloat("Altura", &settings.shotLift, -1.0f, 1.0f, "%.2f");
        ImGui::SetNextItemWidth(-ui(96.0f));
        ImGui::SliderFloat("Efeito", &settings.shotCurve, -1.0f, 1.0f, "%.2f");
    }

    ImGui::Dummy({ 1.0f, ui(4.0f) });
    sectionLabel("BOLA NO PEDIDO (EM RELAÇÃO AO CORPO)");
    ImGui::SetNextItemWidth(-ui(96.0f));
    ImGui::SliderFloat("X frente", &settings.ballOffset.x, -1.0f, 2.5f, "%.2f m");
    ImGui::SetNextItemWidth(-ui(96.0f));
    ImGui::SliderFloat("Y esquerda", &settings.ballOffset.y, -1.5f, 1.5f, "%.2f m");
    ImGui::SetNextItemWidth(-ui(96.0f));
    ImGui::SliderFloat("Z altura", &settings.ballOffset.z, 0.11f, 1.6f, "%.2f m");
    ImGui::Checkbox("Bola rolando junto com o jogador", &settings.ballRolling);

    // Mapa de cima (o corpo no pedido, rumo para cima): esquerdo posiciona a
    // bola, direito aponta a direcao.
    {
        const float side = std::min(ImGui::GetContentRegionAvail().x, ui(290.0f));
        const ImVec2 origin = ImGui::GetCursorScreenPos();
        ImGui::InvisibleButton("##BallMap", { side, side },
            ImGuiButtonFlags_MouseButtonLeft | ImGuiButtonFlags_MouseButtonRight);
        ImDrawList* map = ImGui::GetWindowDrawList();
        map->AddRectFilled(origin, { origin.x + side, origin.y + side }, IM_COL32(9, 17, 25, 255), ui(4.0f));
        map->AddRect(origin, { origin.x + side, origin.y + side }, UiBorder, ui(4.0f));
        // 3,2 m de lado; a pelve um pouco abaixo do centro.
        const float span = 3.2f;
        const float scale = side / span;
        const ImVec2 center { origin.x + side * 0.5f, origin.y + side * 0.66f };
        auto toMap = [&](float forward, float leftward) {
            return ImVec2 { center.x - leftward * scale, center.y - forward * scale };
        };
        map->PushClipRect(origin, { origin.x + side, origin.y + side }, true);
        // Grade de 25 cm (forte a cada metro).
        for (int k = -12; k <= 12; ++k) {
            const float v = static_cast<float>(k) * 0.25f;
            const ImU32 color = k % 4 == 0 ? IM_COL32(40, 60, 78, 255) : IM_COL32(22, 36, 48, 255);
            const float y = toMap(v, 0.0f).y;
            const float x = toMap(0.0f, v).x;
            map->AddLine({ origin.x, y }, { origin.x + side, y }, color);
            map->AddLine({ x, origin.y }, { x, origin.y + side }, color);
        }
        // O corpo: a pelve, os pes no pedido (do ultimo resultado) e o rumo
        // do movimento.
        map->AddCircle(center, ui(9.0f), IM_COL32(170, 180, 190, 255), 20, 2.0f);
        if (m_ballViewerResult.valid && !m_ballViewerResult.frames.empty()) {
            const auto& request = m_ballViewerResult.frames[std::min(m_ballViewerResult.requestFrame,
                m_ballViewerResult.frames.size() - 1)];
            const float yaw = m_ballViewerResult.facingYawRadians;
            const float c = std::cos(-yaw), s = std::sin(-yaw);
            for (int foot = 0; foot < 2; ++foot) {
                const Vec3 d = request.feet[static_cast<std::size_t>(foot)] - m_ballViewerResult.pelvisAtRequest;
                const ImVec2 p = toMap(d.x * c - d.y * s, d.x * s + d.y * c);
                const ImU32 color = foot == 0 ? LeftColor : RightColor;
                if (request.planted[static_cast<std::size_t>(foot)]) {
                    map->AddRectFilled({ p.x - ui(4.0f), p.y - ui(9.0f) }, { p.x + ui(4.0f), p.y + ui(9.0f) }, color);
                } else {
                    map->AddRect({ p.x - ui(4.0f), p.y - ui(9.0f) }, { p.x + ui(4.0f), p.y + ui(9.0f) }, color, 0.0f, 0,
                        2.0f);
                }
            }
        }
        static constexpr std::array<float, static_cast<std::size_t>(BallPreviewGait3D::Count)> Travel {
            0.0f, 0.0f, 0.0f, 90.0f, -90.0f, 180.0f, 90.0f, -90.0f };
        if (settings.gait != BallPreviewGait3D::Idle) {
            const float travel = Travel[static_cast<std::size_t>(settings.gait)] * Pi / 180.0f;
            drawArrow(map, center, toMap(std::cos(travel) * 0.7f, std::sin(travel) * 0.7f),
                IM_COL32(120, 135, 150, 255), 2.0f);
        }
        const ImVec2 ball = toMap(settings.ballOffset.x, settings.ballOffset.y);
        const float direction = settings.directionDegrees * Pi / 180.0f;
        const ImVec2 tip = toMap(settings.ballOffset.x + std::cos(direction) * 0.8f,
            settings.ballOffset.y + std::sin(direction) * 0.8f);
        const float ballRadius = std::max(ui(4.0f), 0.11f * scale);
        // A seta pega pelo mouse: perto da ponta ou da haste (fora da bola).
        const ImVec2 mouse = io.MousePos;
        const auto overArrow = [&]() {
            const float tx = tip.x - ball.x, ty = tip.y - ball.y;
            const float length2 = std::max(1.0f, tx * tx + ty * ty);
            const float along = std::clamp(((mouse.x - ball.x) * tx + (mouse.y - ball.y) * ty) / length2, 0.0f, 1.0f);
            const float px = ball.x + tx * along - mouse.x, py = ball.y + ty * along - mouse.y;
            const float bx = mouse.x - ball.x, by = mouse.y - ball.y;
            const float tipX = mouse.x - tip.x, tipY = mouse.y - tip.y;
            return tipX * tipX + tipY * tipY < ui(14.0f) * ui(14.0f)
                || (px * px + py * py < ui(8.0f) * ui(8.0f) && bx * bx + by * by > ballRadius * ballRadius * 1.6f);
        };
        if (ImGui::IsItemActivated() && ImGui::IsMouseClicked(ImGuiMouseButton_Left))
            m_ballViewerMapDrag = overArrow() ? 2 : 1;
        if (!ImGui::IsItemActive()) m_ballViewerMapDrag = 0;
        const bool arrowHot = m_ballViewerMapDrag == 2 || (m_ballViewerMapDrag == 0 && ImGui::IsItemHovered() && overArrow());
        if (arrowHot) ImGui::SetMouseCursor(ImGuiMouseCursor_Hand);
        drawArrow(map, ball, tip, arrowHot ? IM_COL32(225, 150, 255, 255) : DirectionColor, arrowHot ? 3.5f : 2.5f);
        if (arrowHot) map->AddCircle(tip, ui(9.0f), IM_COL32(225, 150, 255, 200), 16, 1.5f);
        map->AddCircleFilled(ball, ballRadius, BallColor, 20);
        map->PopClipRect();
        ImGui::SetItemTooltip("Arraste a bola ou a seta; o botão direito aponta a direção.");
        if (ImGui::IsItemActive() || ImGui::IsItemHovered()) {
            const float forward = (center.y - mouse.y) / scale;
            const float leftward = (center.x - mouse.x) / scale;
            const auto aim = [&]() {
                const float df = forward - settings.ballOffset.x, dl = leftward - settings.ballOffset.y;
                if (df * df + dl * dl > 0.0025f) {
                    const float limit = settings.kind == BallActionKind3D::Shot ? ShotDirectionLimitDegrees : 180.0f;
                    settings.directionDegrees = std::clamp(std::round(std::atan2(dl, df) * 180.0f / Pi), -limit, limit);
                }
            };
            if (ImGui::IsItemActive() && ImGui::IsMouseDown(ImGuiMouseButton_Left)) {
                if (m_ballViewerMapDrag == 2) {
                    aim();
                } else {
                    settings.ballOffset.x = std::clamp(std::round(forward * 100.0f) / 100.0f, -1.0f, 2.5f);
                    settings.ballOffset.y = std::clamp(std::round(leftward * 100.0f) / 100.0f, -1.5f, 1.5f);
                }
            }
            if (ImGui::IsItemHovered() && ImGui::IsMouseDown(ImGuiMouseButton_Right)) aim();
        }
    }

    ImGui::Dummy({ 1.0f, ui(4.0f) });
    sectionLabel("TOQUE");
    // (V3, 06/10 noite: sem hitbox - a forca sai numa fracao do tempo que o
    // pe leva ate a bola, com ela dentro da area, como no laboratorio.)
    ImGui::SetNextItemWidth(-ui(96.0f));
    ImGui::SliderFloat("Tempo da força", &settings.contactTimingFactor, 0.0f, 1.0f, "%.2f");
    ImGui::SetItemTooltip("0: na hora do pedido; 1: no contato visual do pé.");
    ImGui::Checkbox("Variação de movimento", &settings.motionVariety);

    ImGui::Dummy({ 1.0f, ui(4.0f) });
    if (m_ballViewerJob.valid()) {
        ImGui::ProgressBar(m_ballViewerProgress.load(), { -1.0f, ui(18.0f) }, "simulando...");
    } else if (!m_ballViewerError.empty()) {
        ImGui::PushTextWrapPos(0.0f);
        ImGui::TextColored({ 1.0f, 0.45f, 0.4f, 1.0f }, "%s", m_ballViewerError.c_str());
        ImGui::PopTextWrapPos();
    } else if (m_ballViewerResult.valid) {
        ImGui::TextColored({ 0.42f, 0.51f, 0.59f, 1.0f }, "%.1f s de jogo simulados em %.2f s",
            m_ballViewerResult.simulatedSeconds, m_ballViewerResult.wallSeconds);
    }
    ImGui::Dummy({ 1.0f, ui(6.0f) });
    const std::string back = UI::FontAwesome::label(UI::FontAwesome::ArrowLeft, "Menu principal");
    if (navigationButton(back.c_str(), false, { -1.0f, ui(38.0f) })) {
        m_screen = Screen::MainMenu;
    }
    ImGui::EndChild();

    if (!sameSettings(before, settings)) {
        m_ballViewerDirty = true;
        m_ballViewerDirtySeconds = 0.0f;
    }

    // ---- Area de visualizacao --------------------------------------------
    const float viewportLeft = margin * 2.0f + panelWidth;
    const float viewportWidth = io.DisplaySize.x - viewportLeft - margin;
    ImGui::SetCursorPos({ viewportLeft, top });
    ImGui::BeginChild("BallTouchWorkspace", { viewportWidth, height }, true, ImGuiWindowFlags_NoScrollbar);
    panelHeader("MOVIMENTO PREVISTO");

    const BallTouchPreviewResult3D& result = m_ballViewerResult;
    const bool hasResult = result.valid && !result.frames.empty();
    const std::string playLabel = UI::FontAwesome::label(
        m_ballViewerPlaying ? UI::FontAwesome::Pause : UI::FontAwesome::Play,
        m_ballViewerPlaying ? "Pausar" : "Reproduzir");
    ImGui::BeginDisabled(!hasResult);
    if (ImGui::Button(playLabel.c_str(), { ui(120.0f), ui(32.0f) })) m_ballViewerPlaying = !m_ballViewerPlaying;
    ImGui::SameLine();
    if (ImGui::Button("<##FrameBack", { ui(32.0f), ui(32.0f) })) {
        m_ballViewerPlaying = false;
        const std::size_t index = frameAt(result, m_ballViewerPlaybackSeconds);
        m_ballViewerPlaybackSeconds = result.frames[index > 0 ? index - 1 : 0].timeSeconds;
    }
    ImGui::SameLine();
    if (ImGui::Button(">##FrameForward", { ui(32.0f), ui(32.0f) })) {
        m_ballViewerPlaying = false;
        const std::size_t index = frameAt(result, m_ballViewerPlaybackSeconds);
        m_ballViewerPlaybackSeconds = result.frames[std::min(index + 1, result.frames.size() - 1)].timeSeconds;
    }
    ImGui::SameLine();
    if (ImGui::Button("Pedido", { ui(70.0f), ui(32.0f) })) {
        m_ballViewerPlaying = false;
        m_ballViewerPlaybackSeconds = result.frames[std::min(result.requestFrame, result.frames.size() - 1)].timeSeconds;
    }
    ImGui::SameLine();
    ImGui::BeginDisabled(result.touchFrame < 0);
    if (ImGui::Button("Toque", { ui(70.0f), ui(32.0f) })) {
        m_ballViewerPlaying = false;
        m_ballViewerPlaybackSeconds = result.frames[static_cast<std::size_t>(std::max(0, result.touchFrame))].timeSeconds;
    }
    ImGui::EndDisabled();
    ImGui::SameLine(0.0f, ui(14.0f));
    ImGui::Checkbox("Loop", &m_ballViewerLoop);
    ImGui::SameLine(0.0f, ui(14.0f));
    ImGui::SetNextItemWidth(ui(110.0f));
    ImGui::SliderFloat("Velocidade", &m_ballViewerPlaybackSpeed, 0.05f, 2.0f, "%.2fx");
    ImGui::EndDisabled();
    ImGui::SameLine(0.0f, ui(14.0f));
    ImGui::Checkbox("Rastros", &m_ballViewerShowTrails);
    ImGui::SameLine();
    ImGui::Checkbox("Esqueleto físico", &m_ballViewerShowColliders);
    ImGui::SameLine();
    ImGui::Checkbox("Câmera segue", &m_ballViewerFollowCamera);
    // Vistas prontas.
    const struct { const char* name; float yaw; float pitch; } views[] = {
        { "Lado dir.", -Pi * 0.5f, 0.12f }, { "Lado esq.", Pi * 0.5f, 0.12f }, { "Trás", Pi, 0.30f },
        { "Frente", 0.0f, 0.22f }, { "Cima", -Pi * 0.5f, 1.25f }, { "3/4", -1.15f, 0.32f } };
    for (const auto& v : views) {
        if (ImGui::Button(v.name, { ui(76.0f), ui(26.0f) })) {
            m_ballViewerCameraYaw = v.yaw;
            m_ballViewerCameraPitch = v.pitch;
        }
        ImGui::SameLine();
    }
    ImGui::NewLine();

    const float detailsHeight = ui(118.0f);
    const ImVec2 available = ImGui::GetContentRegionAvail();
    const float previewHeight = std::max(ui(180.0f), available.y - detailsHeight);
    const ImVec2 previewStart = ImGui::GetCursorScreenPos();
    const std::size_t current = hasResult ? frameAt(result, m_ballViewerPlaybackSeconds) : 0;
    if (m_ballViewerPreview && hasResult) {
        const float aspect = static_cast<float>(m_ballViewerPreview.width)
            / static_cast<float>(m_ballViewerPreview.height);
        const float imageWidth = std::min(available.x, previewHeight * aspect);
        const float imageHeight = std::min(previewHeight, imageWidth / aspect);
        ImGui::SetCursorPosX(ImGui::GetCursorPosX() + (available.x - imageWidth) * 0.5f);
        const ImVec2 imageStart = ImGui::GetCursorScreenPos();
        ImGui::Image(static_cast<ImTextureID>(m_ballViewerPreview.id), { imageWidth, imageHeight });
        if (ImGui::IsItemHovered()) {
            ImGui::SetMouseCursor(ImGuiMouseCursor_ResizeAll);
            if (ImGui::IsMouseDragging(ImGuiMouseButton_Left)) {
                m_ballViewerCameraYaw -= io.MouseDelta.x * 0.008f;
                m_ballViewerCameraPitch = std::clamp(m_ballViewerCameraPitch + io.MouseDelta.y * 0.006f, -0.2f, 1.45f);
            }
            if (std::abs(io.MouseWheel) > 0.001f)
                m_ballViewerCameraDistance = std::clamp(m_ballViewerCameraDistance - io.MouseWheel * 0.35f, 1.4f, 14.0f);
        }

        // Rastros e marcas por cima da imagem.
        ImDrawList* overlay = ImGui::GetWindowDrawList();
        overlay->PushClipRect(imageStart, { imageStart.x + imageWidth, imageStart.y + imageHeight }, true);
        const ImVec2 size { imageWidth, imageHeight };
        const Mat4& vp = m_ballViewerViewProjection;
        auto polyline = [&](auto&& pointOf, ImU32 color, float thickness) {
            std::vector<ImVec2> points;
            points.reserve(result.frames.size());
            for (std::size_t i = 0; i < result.frames.size(); ++i) {
                Vec3 p;
                if (!pointOf(result.frames[i], p)) {
                    if (points.size() > 1) overlay->AddPolyline(points.data(), static_cast<int>(points.size()), color, 0, thickness);
                    points.clear();
                    continue;
                }
                ImVec2 s;
                if (projectPoint(vp, p, imageStart, size, s)) points.push_back(s);
            }
            if (points.size() > 1) overlay->AddPolyline(points.data(), static_cast<int>(points.size()), color, 0, thickness);
        };
        if (m_ballViewerShowTrails) {
            for (int foot = 0; foot < 2; ++foot) {
                const ImU32 color = foot == 0 ? LeftColor : RightColor;
                polyline([foot](const BallTouchPreviewFrame3D& f, Vec3& p) {
                    p = f.feet[static_cast<std::size_t>(foot)];
                    return true;
                }, (color & 0x00FFFFFFu) | 0xB0000000u, 2.0f);
                // Pousos: pontos cheios onde o pe esta plantado.
                for (std::size_t i = 0; i < result.frames.size(); i += 4) {
                    const auto& f = result.frames[i];
                    ImVec2 s;
                    if (f.planted[static_cast<std::size_t>(foot)]
                        && projectPoint(vp, f.feet[static_cast<std::size_t>(foot)], imageStart, size, s))
                        overlay->AddCircleFilled(s, 2.5f, color);
                }
            }
            polyline([](const BallTouchPreviewFrame3D& f, Vec3& p) {
                p = f.ballPosition;
                return f.ballPresent;
            }, IM_COL32(255, 255, 255, 200), 2.0f);
            polyline([](const BallTouchPreviewFrame3D& f, Vec3& p) {
                p = f.footGoal.position;
                return f.footGoal.active;
            }, IM_COL32(255, 70, 70, 170), 1.5f);
        }
        // O pedido: a bola onde estava e a direcao pedida.
        ImVec2 ballStart, ballTip;
        if (projectPoint(vp, result.ballAtRequest, imageStart, size, ballStart)
            && projectPoint(vp, result.ballAtRequest + result.directionWorld * 0.9f, imageStart, size, ballTip)) {
            overlay->AddCircle(ballStart, 7.0f, IM_COL32(255, 255, 255, 150), 16, 1.5f);
            drawArrow(overlay, ballStart, ballTip, DirectionColor, 3.0f);
        }
        // O toque.
        if (result.touchFrame >= 0) {
            const auto& touch = result.frames[static_cast<std::size_t>(result.touchFrame)];
            ImVec2 s;
            if (touch.ballPresent && projectPoint(vp, touch.ballPosition, imageStart, size, s)) {
                overlay->AddCircle(s, 12.0f, GoalColor, 20, 2.5f);
                overlay->AddText({ s.x + 14.0f, s.y - 8.0f }, GoalColor, "TOQUE");
            }
        }
        // O quadro atual: o alvo do pe e o pe de apoio.
        const auto& frame = result.frames[current];
        if (frame.footGoal.active) {
            ImVec2 s;
            if (projectPoint(vp, frame.footGoal.position, imageStart, size, s)) {
                const float r = 6.0f + 6.0f * frame.footGoal.weight;
                overlay->AddLine({ s.x - r, s.y - r }, { s.x + r, s.y + r }, GoalColor, 3.0f);
                overlay->AddLine({ s.x - r, s.y + r }, { s.x + r, s.y - r }, GoalColor, 3.0f);
            }
        }
        if (frame.plant.active) {
            ImVec2 s;
            if (projectPoint(vp, frame.plant.position, imageStart, size, s))
                overlay->AddRect({ s.x - 8.0f, s.y - 8.0f }, { s.x + 8.0f, s.y + 8.0f }, PlantColor, 0.0f, 0, 2.5f);
        }
        overlay->PopClipRect();

        // Leitura do quadro.
        overlay->AddRectFilled({ imageStart.x + ui(12.0f), imageStart.y + ui(12.0f) },
            { imageStart.x + ui(330.0f), imageStart.y + ui(108.0f) }, IM_COL32(7, 14, 21, 222), ui(4.0f));
        char line[160];
        std::snprintf(line, sizeof(line), "t %+.3f s  (%s)", frame.timeSeconds,
            frame.timeSeconds < 0.0f ? "antes do pedido" : "depois do pedido");
        overlay->AddText({ imageStart.x + ui(22.0f), imageStart.y + ui(20.0f) }, UiAccent, line);
        std::snprintf(line, sizeof(line), "manejo: %s%s", phaseLabel(frame.phase),
            frame.side >= 0 ? (frame.side == 0 ? "  |  pé esquerdo" : "  |  pé direito") : "");
        overlay->AddText({ imageStart.x + ui(22.0f), imageStart.y + ui(40.0f) }, UiText, line);
        std::snprintf(line, sizeof(line), "pé-bola %.3f m  |  peso do alvo %.2f  |  ritmo %.2f",
            frame.footGap, frame.footGoal.weight, frame.gaitRate);
        overlay->AddText({ imageStart.x + ui(22.0f), imageStart.y + ui(60.0f) }, UiMuted, line);
        std::snprintf(line, sizeof(line), "plantados: E %s  D %s  |  fase da marcha %.2f",
            frame.planted[0] ? "sim" : "não", frame.planted[1] ? "sim" : "não", frame.cyclePhase);
        overlay->AddText({ imageStart.x + ui(22.0f), imageStart.y + ui(80.0f) }, UiMuted, line);
        // Legenda.
        const char* legend = "azul pé E  |  laranja pé D  |  branco bola  |  vermelho alvo do pé  |  "
                             "verde apoio do chute  |  roxo direção";
        const ImVec2 legendSize = ImGui::CalcTextSize(legend);
        overlay->AddRectFilled({ imageStart.x + ui(12.0f), imageStart.y + imageHeight - legendSize.y - ui(24.0f) },
            { imageStart.x + legendSize.x + ui(28.0f), imageStart.y + imageHeight - ui(12.0f) },
            IM_COL32(7, 14, 21, 205), ui(4.0f));
        overlay->AddText({ imageStart.x + ui(20.0f), imageStart.y + imageHeight - legendSize.y - ui(18.0f) },
            UiMuted, legend);
        if (imageHeight < previewHeight) ImGui::Dummy({ 1.0f, previewHeight - imageHeight });
    } else {
        ImGui::Dummy({ available.x, previewHeight });
        const char* loading = !m_ballViewerError.empty() ? "SEM RESULTADO"
            : m_ragdollAssetsLoadFailed ? "NÃO FOI POSSÍVEL PREPARAR O RIG" : "SIMULANDO...";
        const ImVec2 labelSize = ImGui::CalcTextSize(loading);
        ImGui::GetWindowDrawList()->AddText({ previewStart.x + (available.x - labelSize.x) * 0.5f,
            previewStart.y + (previewHeight - labelSize.y) * 0.5f }, UiMuted, loading);
    }

    subtleSeparator();
    if (hasResult) {
        // Resultado do toque.
        if (result.touchFrame >= 0) {
            const Vec3 v = result.ballVelocityAfter;
            ImGui::TextColored({ 0.34f, 0.78f, 0.57f, 1.0f },
                "TOQUE %.2f s depois do pedido  |  pé %s, %s  |  bola %.1f m/s, rumo %+.0f°, elevação %.0f°%s",
                result.touchDelaySeconds, result.side == 0 ? "esquerdo" : "direito", surfaceLabel(result.surface),
                v.length(), std::atan2(v.y, v.x) * 180.0f / Pi,
                std::atan2(v.z, std::hypot(v.x, v.y)) * 180.0f / Pi, result.fell ? "  |  CAIU" : "");
        } else {
            ImGui::TextColored({ 1.0f, 0.55f, 0.4f, 1.0f }, "SEM TOQUE: %s%s", missLabel(result.missReason),
                result.fell ? "  |  CAIU" : "");
        }
        // Linha do tempo com o pedido (roxo) e o toque (vermelho).
        const float start = result.frames.front().timeSeconds;
        const float end = result.frames.back().timeSeconds;
        float timeline = m_ballViewerPlaybackSeconds;
        ImGui::SetNextItemWidth(-1.0f);
        const ImVec2 sliderStart = ImGui::GetCursorScreenPos();
        if (ImGui::SliderFloat("##BallTimeline", &timeline, start, end, "")) {
            m_ballViewerPlaybackSeconds = timeline;
            m_ballViewerPlaying = false;
        }
        const ImVec2 sliderEnd { sliderStart.x + ImGui::GetItemRectSize().x, sliderStart.y + ImGui::GetItemRectSize().y };
        auto mark = [&](float t, ImU32 color) {
            const float x = sliderStart.x + (t - start) / std::max(1e-3f, end - start) * (sliderEnd.x - sliderStart.x);
            ImGui::GetWindowDrawList()->AddLine({ x, sliderStart.y - ui(3.0f) }, { x, sliderEnd.y + ui(3.0f) }, color, 2.0f);
        };
        mark(0.0f, DirectionColor);
        if (result.touchFrame >= 0) mark(result.frames[static_cast<std::size_t>(result.touchFrame)].timeSeconds, GoalColor);
        ImGui::TextColored({ 0.48f, 0.59f, 0.68f, 1.0f },
            "%+.3f s  |  quadro %zu de %zu  |  pedido na fase %.2f da marcha (%s)  |  roxo = pedido, vermelho = toque",
            result.frames[current].timeSeconds, current + 1, result.frames.size(), result.cyclePhaseAtRequest,
            result.gaitClipId.empty() ? "-" : result.gaitClipId.c_str());
    }
    ImGui::EndChild();
    ImGui::End();
}

} // namespace MatterEngine::Workbench
