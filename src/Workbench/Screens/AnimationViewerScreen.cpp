#include "Workbench/WorkbenchApp.hpp"
#include "Workbench/UiKit.hpp"

#include "Engine/Math/Mat4.hpp"
#include "Engine/UI/FontAwesome.hpp"
#include "imgui.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

namespace MatterEngine::Workbench {
namespace {

constexpr float Pi = 3.14159265358979323846f;

std::string playbackTimeLabel(float seconds) {
    const int wholeSeconds = std::max(0, static_cast<int>(seconds));
    const int centiseconds = std::max(0,
        static_cast<int>((seconds - static_cast<float>(wholeSeconds)) * 100.0f));
    char buffer[24] {};
    std::snprintf(buffer, sizeof(buffer), "%02d:%02d.%02d",
        wholeSeconds / 60, wholeSeconds % 60, centiseconds);
    return buffer;
}

} // namespace

UiTexture WorkbenchApp::renderAnimationViewerPreview(
    Renderer& activeRenderer) {
    if (!m_humanRagdollProfile
        || !m_ragdollCharacter || m_characterVisual.parts.empty()) {
        return {};
    }

    const RagdollProfile3D& profile = *m_humanRagdollProfile;
    const AnimationClip3D* clip = nullptr;
    if (!m_animationClips.empty()) {
        m_animationViewerSelectedIndex = std::min(
            m_animationViewerSelectedIndex, m_animationClips.size() - 1);
        clip = &m_animationClips[m_animationViewerSelectedIndex];
    }

    const RagdollAnimationPose3D pose = sampleRagdollAnimationPose3D(
        profile, clip, m_animationViewerPlaybackSeconds,
        m_animationViewerLoop);
    const std::vector<Vec3>& linkPositions = pose.linkPositions;
    const std::vector<Quaternion>& linkOrientations = pose.linkOrientations;

    std::vector<MeshRender3D> meshes;
    const auto palette = buildRagdollSkinMatrices3D(*m_ragdollCharacter,
        linkPositions, linkOrientations);
    if (!m_animationViewerShowColliders) {
        appendGpuModelRenderables(m_characterVisual, {}, {}, {}, {},
            1.0f, false, false, true, meshes);
        for (auto& mesh : meshes) {
            mesh.skinMatrices = palette;
            mesh.previousSkinMatrices = palette;
            mesh.flatShaded = m_ragdollCharacter->flatShaded;
        }
    }
    meshes.reserve(profile.links.size() * 2 + 1);
    for (std::size_t index = 0; m_animationViewerShowColliders && index < profile.links.size(); ++index) {
        appendGpuModelRenderables(m_ragdollLinkVisuals[index],
            linkPositions[index], linkOrientations[index],
            linkPositions[index], linkOrientations[index],
            1.0f, false, false, true, meshes);
        if (index > 0 && index < m_ragdollJointLocalPositions.size()
            && !m_ragdollJointVisual.parts.empty()) {
            const Vec3 jointPosition = linkPositions[index]
                + linkOrientations[index].rotate(
                    m_ragdollJointLocalPositions[index]);
            appendGpuModelRenderables(m_ragdollJointVisual,
                jointPosition, linkOrientations[index],
                jointPosition, linkOrientations[index],
                0.063f, false, false, true, meshes);
        }
    }

    ensureObjectViewerFloorLoaded(activeRenderer);
    const float floorHeight = -profile.standingRootHeightMeters;
    appendGpuModelRenderables(m_objectViewerFloor,
        { 0.0f, 0.0f, floorHeight }, {},
        { 0.0f, 0.0f, floorHeight }, {},
        0.52f, false, false, true, meshes);

    const Vec3 cameraTarget { 0.0f, 0.0f, -0.02f };
    const float planarDistance =
        std::cos(m_animationViewerCameraPitch)
        * m_animationViewerCameraDistance;
    const Vec3 cameraPosition = cameraTarget + Vec3 {
        std::cos(m_animationViewerCameraYaw) * planarDistance,
        std::sin(m_animationViewerCameraYaw) * planarDistance,
        std::sin(m_animationViewerCameraPitch)
            * m_animationViewerCameraDistance
    };
    const Mat4 view = Mat4::lookAt(cameraPosition,
        cameraTarget, { 0.0f, 0.0f, 1.0f });
    const Mat4 projection = Mat4::perspective(
        34.0f * Pi / 180.0f, 1.36f, 0.05f, 30.0f);
    std::array<LightRender3D, 2> lights;
    lights[0].type = LightType3D::Point;
    lights[0].position = { -2.6f, -3.4f, 5.6f };
    lights[0].color = { 0.78f, 0.90f, 1.0f };
    lights[0].intensity = 1.18f;
    lights[0].range = 12.0f;
    lights[0].castsShadow = true;
    lights[1].type = LightType3D::Point;
    lights[1].position = { 3.8f, 2.4f, 2.3f };
    lights[1].color = { 0.34f, 0.60f, 1.0f };
    lights[1].intensity = 0.58f;
    lights[1].range = 10.0f;

    Scene3DFrame scene;
    scene.cameraViewProjection = projection * view;
    scene.cameraViewProjectionJittered = scene.cameraViewProjection;
    scene.previousCameraViewProjection = scene.cameraViewProjection;
    scene.cameraPosition = cameraPosition;
    scene.lights = lights;
    scene.meshes = meshes;
    scene.environment.skyIrradiance = 0.31f;
    scene.showSky = false;
    scene.showShadows = true;
    return activeRenderer.renderScene3D(scene, 1024, 752);
}

void WorkbenchApp::drawAnimationViewer() {
    beginFullscreenPanel("MatterEngineAnimationViewer");
    const ImGuiIO& io = ImGui::GetIO();
    ImDrawList* drawList = ImGui::GetWindowDrawList();
    drawWorkbenchBackdrop(drawList, io.DisplaySize, m_uiScale);

    const std::string status = std::to_string(m_animationClips.size())
        + (m_animationClips.size() == 1 ? " CLIPE" : " CLIPES");
    drawWorkbenchHeader(drawList, io.DisplaySize, m_uiScale,
        "ANIMAÇÕES", status.c_str());

    const float margin = ui(12.0f);
    const float top = headerHeight(m_uiScale) + margin;
    const float height = io.DisplaySize.y - top - margin;
    const float libraryWidth = std::max(sidebarWidth(m_uiScale), ui(250.0f));

    ImGui::SetCursorPos({ margin, top });
    ImGui::BeginChild("AnimationLibrary", { libraryWidth, height }, true,
        ImGuiWindowFlags_NoScrollbar);
    panelHeader("BIBLIOTECA DE ANIMAÇÕES");
    ImGui::Dummy({ 1.0f, ui(10.0f) });

    if (m_animationClips.empty()) {
        const float cardWidth = ImGui::GetContentRegionAvail().x;
        const ImVec2 cardStart = ImGui::GetCursorScreenPos();
        const float cardHeight = ui(148.0f);
        ImGui::GetWindowDrawList()->AddRectFilled(cardStart,
            { cardStart.x + cardWidth, cardStart.y + cardHeight },
            UiPanelElevated, ui(5.0f));
        ImGui::GetWindowDrawList()->AddRect(cardStart,
            { cardStart.x + cardWidth, cardStart.y + cardHeight },
            UiBorder, ui(5.0f));
        ImGui::SetCursorScreenPos(
            { cardStart.x + ui(14.0f), cardStart.y + ui(17.0f) });
        ImGui::TextColored({ 0.38f, 0.69f, 1.0f, 1.0f },
            "BIBLIOTECA VAZIA");
        ImGui::SetCursorScreenPos(
            { cardStart.x + ui(14.0f), cardStart.y + ui(48.0f) });
        ImGui::PushTextWrapPos(cardStart.x + cardWidth - ui(14.0f));
        ImGui::TextColored({ 0.52f, 0.61f, 0.69f, 1.0f },
            "Os clipes importados aparecerão aqui, organizados para "
            "seleção e reprodução.");
        ImGui::PopTextWrapPos();
        ImGui::SetCursorScreenPos(
            { cardStart.x + ui(14.0f), cardStart.y + ui(112.0f) });
        ImGui::TextColored({ 0.29f, 0.48f, 0.63f, 1.0f },
            "PRONTA PARA RECEBER CLIPES");
        ImGui::SetCursorScreenPos(
            { cardStart.x, cardStart.y + cardHeight + ui(12.0f) });
    } else {
        m_animationViewerSelectedIndex = std::min(
            m_animationViewerSelectedIndex, m_animationClips.size() - 1);
        for (std::size_t index = 0;
                index < m_animationClips.size(); ++index) {
            if (navigationButton(
                    m_animationClips[index].displayName.c_str(),
                    index == m_animationViewerSelectedIndex,
                    { -1.0f, ui(42.0f) })) {
                m_animationViewerSelectedIndex = index;
                m_animationViewerPlaybackSeconds = 0.0f;
                m_animationViewerPlaying = true;
                m_animationViewerLoop = m_animationClips[index].loops;
            }
        }
    }

    ImGui::Dummy({ 1.0f, ui(10.0f) });
    sectionLabel("RIG ALVO");
    ImGui::TextColored({ 0.72f, 0.80f, 0.87f, 1.0f }, "%s",
        m_ragdollCharacter ? m_ragdollCharacter->displayName.c_str() : "Carregando...");
    ImGui::TextColored({ 0.42f, 0.51f, 0.59f, 1.0f },
        "18 links  /  41 DOF");
    ImGui::Checkbox("Inspecionar esqueleto físico", &m_animationViewerShowColliders);

    const std::string back = UI::FontAwesome::label(
        UI::FontAwesome::ArrowLeft, "Menu principal");
    ImGui::SetCursorPosY(std::max(ImGui::GetCursorPosY(),
        height - ui(58.0f)));
    if (navigationButton(back.c_str(), false, { -1.0f, ui(40.0f) })) {
        m_screen = Screen::MainMenu;
    }
    ImGui::EndChild();

    const float viewportLeft = margin * 2.0f + libraryWidth;
    const float viewportWidth = io.DisplaySize.x - viewportLeft - margin;
    ImGui::SetCursorPos({ viewportLeft, top });
    ImGui::BeginChild("AnimationWorkspace", { viewportWidth, height }, true,
        ImGuiWindowFlags_NoScrollbar);
    panelHeader("VISUALIZADOR");

    const bool hasClip = !m_animationClips.empty();
    const std::string playLabel = UI::FontAwesome::label(
        m_animationViewerPlaying ? UI::FontAwesome::Pause
                                 : UI::FontAwesome::Play,
        m_animationViewerPlaying ? "Pausar" : "Reproduzir");
    ImGui::BeginDisabled(!hasClip);
    if (ImGui::Button(playLabel.c_str(), { ui(126.0f), ui(34.0f) })) {
        m_animationViewerPlaying = !m_animationViewerPlaying;
    }
    ImGui::SameLine();
    if (ImGui::Button("Reiniciar", { ui(94.0f), ui(34.0f) })) {
        m_animationViewerPlaybackSeconds = 0.0f;
    }
    ImGui::SameLine(0.0f, ui(18.0f));
    ImGui::Checkbox("Loop", &m_animationViewerLoop);
    ImGui::SameLine(0.0f, ui(18.0f));
    ImGui::SetNextItemWidth(ui(112.0f));
    ImGui::SliderFloat("Velocidade", &m_animationViewerPlaybackSpeed,
        0.25f, 2.0f, "%.2fx");
    ImGui::EndDisabled();
    ImGui::SameLine(0.0f, ui(18.0f));
    if (ImGui::Button("Redefinir câmera", { ui(126.0f), ui(34.0f) })) {
        m_animationViewerCameraYaw = -0.975f;
        m_animationViewerCameraPitch = 0.071f;
        m_animationViewerCameraDistance = 4.5f;
    }
    ImGui::Dummy({ 1.0f, ui(7.0f) });

    const float detailsHeight = ui(112.0f);
    const ImVec2 available = ImGui::GetContentRegionAvail();
    const float previewHeight = std::max(ui(180.0f),
        available.y - detailsHeight);
    const ImVec2 previewStart = ImGui::GetCursorScreenPos();
    if (m_animationViewerPreview) {
        const float sourceAspect = static_cast<float>(
            m_animationViewerPreview.width)
            / static_cast<float>(m_animationViewerPreview.height);
        const float imageWidth = std::min(available.x,
            previewHeight * sourceAspect);
        const float imageHeight = std::min(previewHeight,
            imageWidth / sourceAspect);
        ImGui::SetCursorPosX(ImGui::GetCursorPosX()
            + (available.x - imageWidth) * 0.5f);
        const ImVec2 imageStart = ImGui::GetCursorScreenPos();
        ImGui::Image(static_cast<ImTextureID>(m_animationViewerPreview.id),
            { imageWidth, imageHeight });
        if (ImGui::IsItemHovered()) {
            ImGui::SetMouseCursor(ImGuiMouseCursor_ResizeAll);
            if (ImGui::IsMouseDragging(ImGuiMouseButton_Left)) {
                m_animationViewerCameraYaw -= io.MouseDelta.x * 0.008f;
                m_animationViewerCameraPitch = std::clamp(
                    m_animationViewerCameraPitch + io.MouseDelta.y * 0.006f,
                    -1.22f, 1.22f);
            }
            if (std::abs(io.MouseWheel) > 0.001f) {
                m_animationViewerCameraDistance = std::clamp(
                    m_animationViewerCameraDistance - io.MouseWheel * 0.42f,
                    2.8f, 9.0f);
            }
        }
        ImDrawList* viewportDrawList = ImGui::GetWindowDrawList();
        viewportDrawList->AddRectFilled(
            { imageStart.x + ui(14.0f), imageStart.y + ui(14.0f) },
            { imageStart.x + ui(191.0f), imageStart.y + ui(68.0f) },
            IM_COL32(7, 14, 21, 218), ui(4.0f));
        viewportDrawList->AddText(
            { imageStart.x + ui(26.0f), imageStart.y + ui(25.0f) },
            UiAccent, hasClip ? "CLIPE EM REPRODUÇÃO" : "POSE DE REFERÊNCIA");
        viewportDrawList->AddText(
            { imageStart.x + ui(26.0f), imageStart.y + ui(46.0f) },
            UiMuted, m_ragdollCharacter ? m_ragdollCharacter->profile.id.c_str() : "RIG FÍSICO");
        const char* cameraHint = "ARRASTE PARA GIRAR  |  RODA PARA ZOOM";
        const ImVec2 cameraHintSize = ImGui::CalcTextSize(cameraHint);
        const ImVec2 hintMin {
            imageStart.x + imageWidth - cameraHintSize.x - ui(30.0f),
            imageStart.y + imageHeight - ui(45.0f)
        };
        viewportDrawList->AddRectFilled(hintMin,
            { imageStart.x + imageWidth - ui(14.0f),
                imageStart.y + imageHeight - ui(14.0f) },
            IM_COL32(7, 14, 21, 205), ui(4.0f));
        viewportDrawList->AddText(
            { hintMin.x + ui(8.0f), hintMin.y + ui(8.0f) },
            UiMuted, cameraHint);
        if (imageHeight < previewHeight) {
            ImGui::Dummy({ 1.0f, previewHeight - imageHeight });
        }
    } else {
        ImGui::Dummy({ available.x, previewHeight });
        const char* loading = m_ragdollAssetsLoadFailed
            ? "NÃO FOI POSSÍVEL PREPARAR O RIG"
            : "PREPARANDO RIG...";
        const ImVec2 labelSize = ImGui::CalcTextSize(loading);
        ImGui::GetWindowDrawList()->AddText(
            { previewStart.x + (available.x - labelSize.x) * 0.5f,
                previewStart.y + (previewHeight - labelSize.y) * 0.5f },
            UiMuted, loading);
    }

    subtleSeparator();
    if (hasClip) {
        const AnimationClip3D& clip =
            m_animationClips[m_animationViewerSelectedIndex];
        ImGui::TextUnformatted(clip.displayName.c_str());
        ImGui::SameLine(0.0f, ui(18.0f));
        ImGui::TextColored({ 0.43f, 0.62f, 0.76f, 1.0f },
            "%zu canais  |  %.2f s  |  %.1f fps",
            clip.tracks.size(), clip.durationSeconds,
            clip.sourceSampleRateHz);
        if (clip.retargetReport.available) {
            ImGui::SameLine(0.0f, ui(18.0f));
            ImGui::TextColored({ 0.34f, 0.78f, 0.57f, 1.0f },
                "RETARGET VALIDADO  |  erro RMS %.1f°  |  membros máx %.1f°",
                clip.retargetReport.directionRmsDegrees,
                clip.retargetReport.limbDirectionMaxDegrees);
        }
        float timeline = m_animationViewerPlaybackSeconds;
        ImGui::SetNextItemWidth(-1.0f);
        if (ImGui::SliderFloat("##AnimationTimeline", &timeline,
                0.0f, clip.durationSeconds, "")) {
            m_animationViewerPlaybackSeconds = timeline;
        }
        ImGui::TextColored({ 0.48f, 0.59f, 0.68f, 1.0f }, "%s / %s",
            playbackTimeLabel(m_animationViewerPlaybackSeconds).c_str(),
            playbackTimeLabel(clip.durationSeconds).c_str());
    } else {
        ImGui::TextColored({ 0.78f, 0.86f, 0.92f, 1.0f },
            "Nenhuma animação importada");
        ImGui::TextColored({ 0.43f, 0.53f, 0.61f, 1.0f },
            "O rig permanece na pose neutra. Ao inserir o primeiro clipe, "
            "a timeline e os controles de loop serão habilitados.");
        ImGui::BeginDisabled();
        float emptyTimeline = 0.0f;
        ImGui::SetNextItemWidth(-1.0f);
        ImGui::SliderFloat("##EmptyAnimationTimeline", &emptyTimeline,
            0.0f, 1.0f, "");
        ImGui::EndDisabled();
    }

    ImGui::EndChild();
    ImGui::End();
}

} // namespace MatterEngine::Workbench
