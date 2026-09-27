#include "Workbench/WorkbenchApp.hpp"

#include "Engine/Core/Log.hpp"
#include "Engine/Platform/PlatformServices.hpp"
#include "imgui.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <string>

namespace MatterEngine::Workbench {
namespace {

constexpr float Pi = 3.14159265358979323846f;

Vec3 laboratoryViewForward(float yaw, float pitch) {
    const float planar = std::cos(pitch);
    return { planar * std::cos(yaw), planar * std::sin(yaw),
        std::sin(pitch) };
}

Quaternion laboratoryViewOrientation(float yaw, float pitch) {
    return (Quaternion::fromAxisAngle({ 0.0f, 0.0f, 1.0f }, yaw)
        * Quaternion::fromAxisAngle({ 0.0f, 1.0f, 0.0f }, -pitch))
        .normalized();
}

std::string settingsDatabasePath() {
    return Platform::executableBasePath() + "matterengine.db";
}

std::string environmentValue(const char* name) {
#if defined(_MSC_VER)
    char* value = nullptr;
    std::size_t length = 0;
    const int error = _dupenv_s(&value, &length, name);
    if (error != 0 || value == nullptr) {
        std::free(value);
        return {};
    }
    std::string result(value);
    std::free(value);
    return result;
#else
    const char* value = std::getenv(name);
    return value != nullptr ? std::string(value) : std::string {};
#endif
}

int environmentInt(const char* name, int fallback,
    int minimum, int maximum) {
    const std::string text = environmentValue(name);
    if (text.empty()) return fallback;
    char* end = nullptr;
    const long parsed = std::strtol(text.c_str(), &end, 10);
    if (end == text.c_str() || *end != '\0') return fallback;
    return std::clamp(static_cast<int>(parsed), minimum, maximum);
}

float environmentFloat(const char* name, float fallback,
    float minimum, float maximum) {
    const std::string text = environmentValue(name);
    if (text.empty()) return fallback;
    char* end = nullptr;
    const float parsed = std::strtof(text.c_str(), &end);
    if (end == text.c_str() || *end != '\0' || !std::isfinite(parsed)) {
        return fallback;
    }
    return std::clamp(parsed, minimum, maximum);
}

ApplicationConfig initialConfiguration() {
    const Data::VideoSettings video =
        Data::SettingsRepository(settingsDatabasePath()).loadVideoSettings();
    ApplicationConfig config;
    config.title = "MatterEngine";
    config.width = std::max(960, video.width);
    config.height = std::max(540, video.height);
    // V-sync ligado: sem limite a GPU renderiza a taxa que conseguir, o que
    // nesta máquina faz a placa apitar. A simulação permanece independente
    // e determinística nos 120 Hz definidos abaixo, então isso não muda o
    // comportamento do jogo — só limita a taxa de apresentação ao monitor.
    config.vsync = true;
    config.displayMode = video.mode;
    config.fixedUpdateHz = 120.0f;
    config.maxFixedStepsPerFrame = 8;
    return config;
}

} // namespace

WorkbenchApp::WorkbenchApp()
    : Application(initialConfiguration()),
      m_physicsEngine(taskScheduler()) {
    // A exposição do Laboratório é fixa: mudar a direção da câmera não deve
    // clarear ou escurecer a cena com base no conteúdo visível.
    m_laboratoryToneMapping.automaticExposureEnabled = false;
    applyLaboratoryGraphicsPreset(0);
}

void WorkbenchApp::applyLaboratoryGraphicsPreset(int preset) {
    m_laboratoryGraphicsPreset = std::clamp(preset, 0, 2);
    // GTAO saiu do frame ativo na varredura de desempenho. O preenchimento
    // de contato agora vem das sombras e da luz indireta analítica.
    m_laboratoryAmbientOcclusion.enabled = false;
    m_laboratoryOpticalEffects.enabled = true;
}

void WorkbenchApp::onStart() {
    Log::info("MatterEngine Workbench iniciado.");
    PhysicsSceneSettings3D physicsSettings;
    // Iteracoes do solver ficam no default do backend: cada motor tem a sua
    // propria escala e fixar numeros aqui carregaria a calibragem de um deles
    // para o outro (ver PhysicsSceneSettings3D).
    physicsSettings.enableContinuousCollision = true;
    physicsSettings.enableStabilization = true;
    m_physicsScene = m_physicsEngine.createScene(physicsSettings,
        m_materialLibrary);
    m_menuLogo = renderer().loadUiTexture(
        std::string(MATTERENGINE_ASSETS_DIR) + "/ui/matter-engine-logo.png");
    loadAnimationCatalog();
    m_hrtfEnabled = Data::SettingsRepository(settingsDatabasePath())
        .getInt("audio.hrtf_enabled").value_or(1) != 0;
    static_cast<void>(m_worldAudio.initialize(
        MATTERENGINE_ASSETS_DIR, m_hrtfEnabled));
    applyAutostartIfRequested();
}

void WorkbenchApp::loadAnimationCatalog() {
    namespace fs = std::filesystem;
    m_animationClips.clear();
    m_proceduralAnimationClips.clear();
    m_animationLocomotionCompatible = false;
    RagdollProfile3D targetProfile;
    try {
        const std::string configuredCharacter = environmentValue("MATTERENGINE_CHARACTER");
        if (!configuredCharacter.empty()) m_characterAssetPath = configuredCharacter;
        m_ragdollCharacter = loadRagdollCharacter3D(
            (fs::path(MATTERENGINE_ASSETS_DIR) / m_characterAssetPath).string());
        targetProfile = m_ragdollCharacter->profile;
    } catch (const std::exception& exception) {
        Log::error("Biblioteca de animações sem perfil alvo: "
            + std::string(exception.what()));
        return;
    }
    const fs::path clipDirectory =
        fs::path(MATTERENGINE_ASSETS_DIR) / "animations/clips";
    std::error_code error;
    if (!fs::is_directory(clipDirectory, error)) {
        Log::warn("Pasta de clipes de animação não encontrada: "
            + clipDirectory.string());
        return;
    }

    std::vector<fs::path> files;
    for (fs::directory_iterator iterator(clipDirectory, error), end;
            !error && iterator != end; iterator.increment(error)) {
        if (iterator->is_regular_file()
            && iterator->path().extension() == ".json"
            && iterator->path().filename().string().ends_with(
                ".matteranim.json")) {
            files.push_back(iterator->path());
        }
    }
    std::sort(files.begin(), files.end());
    for (const fs::path& file : files) {
        try {
            AnimationClip3D clip = loadAnimationClip3D(file.string());
            if (clip.targetRigId != targetProfile.id) continue;
            const auto rigIssues =
                validateAnimationClipForRagdoll3D(clip, targetProfile);
            if (!rigIssues.empty()) {
                Log::error("Clipe rejeitado pelo perfil físico: "
                    + file.string() + " (" + rigIssues.front().path + ": "
                    + rigIssues.front().message + ")");
                continue;
            }
            m_animationClips.push_back(std::move(clip));
        } catch (const std::exception& exception) {
            Log::error("Falha ao carregar clipe " + file.string()
                + ": " + exception.what());
        }
    }
    if (error) {
        Log::error("Falha ao enumerar clipes de animação: "
            + error.message());
    }
    m_animationLocomotionCompatible =
        characterLocomotionAnimations().compatible(targetProfile);
    // O visualizador mostra os ciclos do próprio personagem, na ordem em
    // que o jogo os usa: parado em alerta, passada, recuo, sprint, strafe e
    // pulos.
    const std::vector<std::string> characterClipIds {
        m_ragdollCharacter->idleClipId,
        m_ragdollCharacter->walkClipId,
        m_ragdollCharacter->walkBackwardClipId,
        m_ragdollCharacter->sprintClipId,
        m_ragdollCharacter->sprintBackwardClipId,
        m_ragdollCharacter->strafeLeftClipId,
        m_ragdollCharacter->strafeRightClipId,
        m_ragdollCharacter->sprintStrafeLeftClipId,
        m_ragdollCharacter->sprintStrafeRightClipId,
        m_ragdollCharacter->jumpStandingClipId,
        m_ragdollCharacter->jumpForwardClipId,
        m_ragdollCharacter->jumpBackwardClipId,
        m_ragdollCharacter->jumpLeftClipId,
        m_ragdollCharacter->jumpRightClipId,
    };
    for (const std::string& clipId : characterClipIds) {
        if (clipId.empty()) continue;
        const auto match = std::find_if(m_animationClips.begin(),
            m_animationClips.end(), [&](const AnimationClip3D& clip) {
                return clip.id == clipId;
            });
        if (match != m_animationClips.end()) {
            m_proceduralAnimationClips.push_back(*match);
        } else {
            Log::warn("Clipe do personagem não encontrado: " + clipId);
        }
    }
    // A cápsula anda na velocidade que cada ciclo foi feito para andar; os
    // números saem dos clipes, não de constantes que se desencontram deles.
    m_avatarGaitSpeeds = characterGaitSpeeds3D(characterLocomotionAnimations());
    if (m_avatarGaitSpeeds.walk > 0.0f) {
        m_avatarCharacterSettings.walkSpeed = m_avatarGaitSpeeds.walk;
        m_avatarCharacterSettings.sprintSpeed = m_avatarGaitSpeeds.sprint;
    }
    Log::info("Biblioteca interna de locomoção: "
        + std::to_string(m_animationClips.size()) + " clipe(s); workspace "
        "procedural: " + std::to_string(m_proceduralAnimationClips.size())
        + " movimento(s).");
}

void WorkbenchApp::applyAutostartIfRequested() {
    const std::string mode = environmentValue("MATTERENGINE_AUTOSTART");
    if (mode == "laboratory" || mode == "laboratory-debug" || mode == "laboratory-pose"
        || mode == "laboratory-biomechanics"
        || mode == "laboratory-character"
        || mode == "laboratory-camera-orbit-smoke"
        || mode == "laboratory-prop-motion-smoke"
        || mode == "laboratory-smoke"
        || mode == "laboratory-pixel"
        || mode == "laboratory-pixel-smoke"
        || mode == "laboratory-pixel-scaled-smoke"
        || mode == "laboratory-pixel-taa-smoke"
        || mode == "laboratory-pixel-spawn-smoke"
        || mode == "laboratory-spawn-smoke"
        || mode == "laboratory-ragdoll-smoke"
        || mode == "laboratory-animation" || mode == "laboratory-animation-smoke"
        || mode == "laboratory-walk" || mode == "laboratory-walk-smoke"
        || mode == "laboratory-prop-smoke"
        || mode == "laboratory-prop-suite-smoke"
        || mode == "laboratory-benchmark") {
        enterLaboratory(true);
        m_laboratoryDebugVisible = mode == "laboratory-debug";
        if (mode == "laboratory-biomechanics") {
            m_biomechanicalExperiment = true;
            m_laboratoryDebugVisible = true;
            m_showCharacterPanel = true;
        }
        if(mode=="laboratory-pose") {
            m_laboratoryDebugVisible=true;
            m_showRagdollPanel=true;
            m_manualRagdollInspection=true;
            m_hidePhysGunPresentation=true;
        }
        m_spawnMenuOpen = mode == "laboratory-spawn-smoke"
            || mode == "laboratory-pixel-spawn-smoke";
        m_spawnRagdollWhenReady = mode == "laboratory-ragdoll-smoke";
        m_takeCharacterControlWhenReady = mode == "laboratory-character"
            || mode == "laboratory-biomechanics"
            || mode == "laboratory-camera-orbit-smoke";
        if (mode == "laboratory-camera-orbit-smoke"
            || mode == "laboratory-prop-motion-smoke") {
            m_renderMotionSmoke.mode = mode == "laboratory-camera-orbit-smoke"
                ? RenderMotionSmoke::Mode::CameraOrbit
                : RenderMotionSmoke::Mode::PropMotion;
            // Teto de seguranca; o proprio smoke encerra ao fim da medicao.
            m_automaticQuitSeconds = 40.0f;
        }
        m_animationWalkTest=mode=="laboratory-walk"||mode=="laboratory-walk-smoke";
        m_animationRunTest=mode=="laboratory-animation"||mode=="laboratory-animation-smoke"||m_animationWalkTest;
        if(m_animationRunTest) {
            m_laboratoryDebugVisible=true;m_showRagdollPanel=true;
            m_hidePhysGunPresentation=true;
            if(mode.ends_with("-smoke"))m_automaticQuitSeconds=m_animationWalkTest?22.0f:11.0f;
        }
        m_spawnAllPropsWhenReady =
            mode == "laboratory-prop-suite-smoke";
        if (mode == "laboratory-benchmark") {
            m_physicsBenchmarkRequestedProps = environmentInt(
                "MATTERENGINE_BENCHMARK_PROPS", 100, 10, 500);
            m_physicsBenchmarkRequestedRagdolls = environmentInt(
                "MATTERENGINE_BENCHMARK_RAGDOLLS", 30, 1, 100);
            m_physicsBenchmarkSpawnRatePerSecond = environmentFloat(
                "MATTERENGINE_BENCHMARK_RATE", 30.0f, 1.0f, 30.0f);
            m_automaticBenchmarkDurationSeconds = environmentFloat(
                "MATTERENGINE_BENCHMARK_SECONDS", 25.0f, 5.0f, 600.0f);
            m_automaticBenchmarkWarmupSeconds = environmentFloat(
                "MATTERENGINE_BENCHMARK_WARMUP", 2.0f, 0.0f, 30.0f);
            m_physicsBenchmarkShowAnalytics = true;
            m_startPhysicsBenchmarkWhenReady = true;
        }
        if (mode == "laboratory-prop-smoke") {
            m_spawnPropWhenReadyId =
                environmentValue("MATTERENGINE_AUTOSPAWN_PROP");
            if (m_spawnPropWhenReadyId.empty()) {
                m_spawnPropWhenReadyId = "soccer_ball";
            }
        }
        if (mode == "laboratory-pixel"
            || mode == "laboratory-pixel-smoke"
            || mode == "laboratory-pixel-scaled-smoke"
            || mode == "laboratory-pixel-taa-smoke"
            || mode == "laboratory-pixel-spawn-smoke") {
            m_laboratoryRenderMode = SceneRenderMode3D::PixelArt;
            // O smoke com atlas do menu também cobre o caminho de upscale
            // interno, sem alterar o default interativo de 100%.
            if (mode == "laboratory-pixel-scaled-smoke"
                || mode == "laboratory-pixel-spawn-smoke") {
                m_laboratoryPixelArt.renderScale = 0.75f;
            }
            if (mode == "laboratory-pixel-taa-smoke") {
                m_laboratoryPixelArt.temporalAntiAliasingEnabled = true;
            }
            m_laboratoryTaaHistoryValid = false;
        }
        if (mode == "laboratory-smoke"
            || mode == "laboratory-pixel-smoke"
            || mode == "laboratory-pixel-scaled-smoke"
            || mode == "laboratory-pixel-taa-smoke"
            || mode == "laboratory-pixel-spawn-smoke"
            || mode == "laboratory-spawn-smoke"
            || mode == "laboratory-ragdoll-smoke"
            || mode == "laboratory-prop-smoke"
            || mode == "laboratory-prop-suite-smoke") {
            m_automaticQuitSeconds =
                mode == "laboratory-ragdoll-smoke"
                    || mode == "laboratory-prop-smoke"
                    || mode == "laboratory-prop-suite-smoke"
                    ? 10.0f : 6.0f;
        }
        syncLaboratoryMouseCapture();
    } else if (mode == "object-viewer" || mode == "object-viewer-smoke") {
        m_screen = Screen::ObjectViewer;
        if (mode == "object-viewer-smoke") m_automaticQuitSeconds = 6.0f;
    } else if (mode == "animation-viewer"
        || mode == "animation-viewer-smoke") {
        m_screen = Screen::AnimationViewer;
        m_animationViewerCameraYaw = environmentFloat("MATTERENGINE_ANIMATION_YAW",
            m_animationViewerCameraYaw,-6.3f,6.3f);
        m_animationViewerCameraPitch = environmentFloat("MATTERENGINE_ANIMATION_PITCH",
            m_animationViewerCameraPitch,-1.22f,1.22f);
        m_animationViewerCameraDistance = environmentFloat("MATTERENGINE_ANIMATION_DISTANCE",
            m_animationViewerCameraDistance,2.8f,9.0f);
        const auto selected=environmentValue("MATTERENGINE_ANIMATION_CLIP");
        for(std::size_t i=0;i<m_proceduralAnimationClips.size();++i)if(m_proceduralAnimationClips[i].id==selected)m_animationViewerSelectedIndex=i;
        if(!m_proceduralAnimationClips.empty())m_animationViewerLoop=m_proceduralAnimationClips[m_animationViewerSelectedIndex].loops;
        if (!environmentValue("MATTERENGINE_ANIMATION_PHASE").empty() && !m_proceduralAnimationClips.empty()) {
            m_animationViewerPlaybackSeconds = m_proceduralAnimationClips[m_animationViewerSelectedIndex].durationSeconds
                * environmentFloat("MATTERENGINE_ANIMATION_PHASE",0.0f,0.0f,1.0f);
            m_animationViewerPlaying=false;
        }
        if (mode == "animation-viewer-smoke") {
            m_automaticQuitSeconds = 6.0f;
        }
    } else if (mode == "settings") {
        m_settingsReturnScreen = Screen::MainMenu;
        m_screen = Screen::Settings;
    }
    if (mode.ends_with("-smoke"))
        m_automaticQuitSeconds=environmentFloat("MATTERENGINE_SMOKE_SECONDS",
            m_automaticQuitSeconds,1.0f,120.0f);
}

void WorkbenchApp::onEvent(const Event& event) {
    if (m_screen == Screen::Laboratory) {
        if (event.type == EventType::WindowFocusLost) {
            m_laboratoryWindowFocused = false;
            endPhysGunGrab();
            m_spawnMenuOpen = false;
            m_laboratoryJumpRequested = false;
            m_laboratoryFlightToggleRequested = false;
            renderer().setMouseCaptured(false);
            return;
        }
        if (event.type == EventType::WindowFocusGained) {
            m_laboratoryWindowFocused = true;
            m_discardNextMouseDelta = true;
            syncLaboratoryMouseCapture();
            return;
        }
        if (event.type == EventType::KeyDown && !event.repeat) {
            if (event.key == Key::Escape) {
                endPhysGunGrab();
                if (m_laboratoryConfirmQuit) {
                    m_laboratoryConfirmQuit = false;
                } else {
                    m_laboratoryPaused = !m_laboratoryPaused;
                }
                m_spawnMenuOpen = false;
                syncLaboratoryMouseCapture();
                return;
            }
            if (event.key == Key::Quote && !m_laboratoryPaused) {
                m_laboratoryDebugVisible = !m_laboratoryDebugVisible;
                syncLaboratoryMouseCapture();
                return;
            }
            if (event.key == Key::T && !m_laboratoryPaused
                && !m_spawnMenuOpen
                && m_controlledCharacterEntityId != 0) {
                resetControlledCharacterPose();
                return;
            }
            if (event.key == Key::U && !m_laboratoryPaused
                && !m_spawnMenuOpen
                && m_controlledCharacterEntityId != 0) {
                toggleControlledCharacterFrozen();
                return;
            }
            if (event.key == Key::Space && !m_laboratoryPaused
                && !m_laboratoryDebugVisible && !m_spawnMenuOpen) {
                m_laboratoryJumpRequested = true;
                return;
            }
            if (event.key == Key::V && !m_laboratoryPaused
                && !m_laboratoryDebugVisible && !m_spawnMenuOpen) {
                m_laboratoryFlightToggleRequested = true;
                return;
            }
            if (event.key == Key::Q && !m_laboratoryPaused
                && !m_laboratoryDebugVisible) {
                endPhysGunGrab();
                m_spawnMenuOpen = true;
                syncLaboratoryMouseCapture();
                return;
            }
            if (event.key == Key::Z && !m_laboratoryPaused
                && !m_laboratoryDebugVisible) {
                removeLatestSpawnedEntity();
                return;
            }
            if (event.key == Key::F && !m_laboratoryPaused
                && !m_laboratoryDebugVisible && !m_spawnMenuOpen) {
                m_physGunFlashlight.enabled =
                    !m_physGunFlashlight.enabled;
                return;
            }
            if (event.key == Key::R && !m_laboratoryPaused
                && !m_laboratoryDebugVisible && !m_spawnMenuOpen) {
                constexpr float DoublePressWindowSeconds = 0.32f;
                if (m_laboratoryElapsedTime - m_lastUnfreezePressSeconds
                    <= DoublePressWindowSeconds) {
                    unfreezeAllObjects();
                    m_lastUnfreezePressSeconds = -10.0f;
                } else {
                    unfreezeLookedAtObject();
                    m_lastUnfreezePressSeconds = m_laboratoryElapsedTime;
                }
                return;
            }
            if (event.key == Key::E && m_physGunGrabbedEntityId != 0
                && !m_laboratoryPaused && !m_laboratoryDebugVisible
                && !m_spawnMenuOpen) {
                m_physGunRotationStartOrientation =
                    m_physGunTargetOrientation;
                m_physGunRotationInputDegrees = {};
                return;
            }
        }
        if (event.type == EventType::MouseButtonDown
            && event.button == MouseButton::Left
            && renderer().mouseCaptured() && !m_laboratoryPaused
            && !m_laboratoryDebugVisible && !m_spawnMenuOpen
            && m_controlledCharacterEntityId == 0) {
            m_physGunTriggerHeld = true;
            // O impulso alimenta um oscilador amortecido atualizado no passo
            // fixo. O viewmodel recua sem afetar a camera nem a simulacao.
            m_physGunRecoilVelocity += 6.5f;
            beginPhysGunGrab();
            return;
        }
        if (event.type == EventType::MouseButtonDown
            && event.button == MouseButton::Right
            && m_physGunGrabbedEntityId != 0
            && renderer().mouseCaptured() && !m_laboratoryPaused
            && !m_laboratoryDebugVisible && !m_spawnMenuOpen) {
            freezePhysGunObject();
            return;
        }
        if (event.type == EventType::MouseButtonUp
            && event.button == MouseButton::Left) {
            m_physGunTriggerHeld = false;
            endPhysGunGrab();
            return;
        }
        if (event.type == EventType::MouseWheel
            && m_physGunGrabbedEntityId != 0) {
            const float previousDistance = m_physGunHoldDistance;
            m_physGunHoldDistance = std::clamp(
                m_physGunHoldDistance + event.wheelY * 0.45f,
                0.65f, 80.0f);
            if (m_physGunHoldMode == PhysGunHoldMode::FixedPose) {
                // Na base da orientação da câmera, +X é o eixo de visão.
                m_physGunFixedCenterCameraLocal.x +=
                    m_physGunHoldDistance - previousDistance;
            }
            return;
        }
        if (event.type == EventType::KeyUp && event.key == Key::Q) {
            m_spawnMenuOpen = false;
            syncLaboratoryMouseCapture();
            return;
        }
        if (event.type == EventType::MouseMove && renderer().mouseCaptured()
            && !m_laboratoryPaused && !m_laboratoryDebugVisible
            && !m_spawnMenuOpen) {
            if (m_discardNextMouseDelta) {
                m_discardNextMouseDelta = false;
                return;
            }
            if (m_physGunGrabbedEntityId != 0
                && input().keyDown(Key::E)) {
                constexpr float RotationDegreesPerPixel = 0.22f;
                m_physGunRotationInputDegrees.x +=
                    event.mouseDelta.x * RotationDegreesPerPixel;
                m_physGunRotationInputDegrees.y +=
                    event.mouseDelta.y * RotationDegreesPerPixel;

                Vec2 applied = m_physGunRotationInputDegrees;
                const bool snap = input().keyDown(Key::LeftShift)
                    || input().keyDown(Key::RightShift);
                if (snap) {
                    const float snapDegrees = std::max(1.0f,
                        m_physGunRotationSnapDegrees);
                    applied.x = std::round(applied.x / snapDegrees)
                        * snapDegrees;
                    applied.y = std::round(applied.y / snapDegrees)
                        * snapDegrees;
                }
                const Vec3 forward = laboratoryViewForward(
                    m_laboratoryCameraYaw,
                    m_laboratoryCameraPitch).normalized();
                const Vec3 planarForward = laboratoryViewForward(
                    m_laboratoryCameraYaw, 0.0f);
                const Vec3 right { planarForward.y,
                    -planarForward.x, 0.0f };
                const Vec3 up = cross(right, forward).normalized();
                const Quaternion horizontal = Quaternion::fromAxisAngle(
                    up, -applied.x * Pi / 180.0f);
                const Quaternion vertical = Quaternion::fromAxisAngle(
                    right, -applied.y * Pi / 180.0f);
                m_physGunTargetOrientation = (horizontal * vertical
                    * m_physGunRotationStartOrientation).normalized();
                m_physGunOrientationLocked = true;
                const Quaternion viewOrientation =
                    laboratoryViewOrientation(m_laboratoryCameraYaw,
                        m_laboratoryCameraPitch);
                m_physGunRelativeOrientation =
                    (viewOrientation.conjugate()
                        * m_physGunTargetOrientation).normalized();
                return;
            }
            constexpr float Sensitivity = 0.0022f;
            m_laboratoryCameraYaw -= event.mouseDelta.x * Sensitivity;
            m_laboratoryCameraPitch = std::clamp(
                m_laboratoryCameraPitch - event.mouseDelta.y * Sensitivity,
                -1.48f, 1.48f);
        }
        return;
    }

    if (event.type == EventType::KeyDown && !event.repeat
        && event.key == Key::Escape) {
        if (m_screen == Screen::Settings) {
            m_screen = m_settingsReturnScreen;
        } else if (m_screen == Screen::ObjectViewer
            || m_screen == Screen::AnimationViewer) {
            m_screen = Screen::MainMenu;
        }
    }
}

void WorkbenchApp::onUpdate(float deltaTime) {
    if (m_automaticQuitSeconds > 0.0f) {
        m_automaticQuitSeconds -= deltaTime;
        if (m_automaticQuitSeconds <= 0.0f) {
            const ApplicationFrameMetrics& application =
                applicationFrameMetrics();
            const RHI::FramePerformanceMetrics graphics =
                renderer().framePerformanceMetrics();
            reportRenderMotionSmoke();
            Log::info("Smoke performance: "
                + std::to_string(ImGui::GetIO().Framerate)
                + " FPS; GPU "
                + std::to_string(graphics.gpuFrameMilliseconds)
                + " ms; frame "
                + std::to_string(application.totalMilliseconds)
                + " ms (update "
                + std::to_string(application.fixedUpdateMilliseconds)
                + ", render "
                + std::to_string(application.renderMilliseconds)
                + ", UI "
                + std::to_string(application.guiMilliseconds)
                + "); sync "
                + std::to_string(graphics.cpuFenceWaitMilliseconds)
                + "/"
                + std::to_string(graphics.cpuAcquireMilliseconds)
                + "/"
                + std::to_string(graphics.cpuPresentMilliseconds)
                + " ms.");
            if (graphics.gpuTimingValid) {
                Log::info("Smoke GPU passes: shadow "
                    + std::to_string(graphics.gpuShadowMilliseconds)
                    + " ms; depth "
                    + std::to_string(
                        graphics.gpuDepthPrepassMilliseconds)
                    + "; opaque "
                    + std::to_string(graphics.gpuOpaqueMilliseconds)
                    + "; ocean "
                    + std::to_string(graphics.gpuOceanMilliseconds)
                    + "; temporal "
                    + std::to_string(graphics.gpuTemporalMilliseconds)
                    + "; glare "
                    + std::to_string(graphics.gpuBloomGlareMilliseconds)
                    + "; exposure "
                    + std::to_string(graphics.gpuExposureMilliseconds)
                    + "; tonemap "
                    + std::to_string(graphics.gpuTonemapMilliseconds)
                    + "; UI "
                    + std::to_string(graphics.gpuUiMilliseconds)
                    + " ms.");
            }
            if (m_physicsBenchmarkOverallFrameSamples > 0) {
                const float benchmarkAverage =
                    m_physicsBenchmarkOverallFpsSum
                    / static_cast<float>(
                        m_physicsBenchmarkOverallFrameSamples);
                Log::info("Benchmark completo: baseline "
                    + std::to_string(m_physicsBenchmarkBaselineFps)
                    + " FPS; media "
                    + std::to_string(benchmarkAverage)
                    + "; minimo "
                    + std::to_string(
                        m_physicsBenchmarkOverallMinimumFps)
                    + "; props "
                    + std::to_string(
                        m_physicsBenchmarkPropEntityIds.size())
                    + "; ragdolls "
                    + std::to_string(
                        m_physicsBenchmarkRagdollEntityIds.size())
                    + ".");
            }
            if (m_physicsScene) {
                const PhysicsStepDiagnostics3D& physics =
                    m_physicsScene->diagnostics();
                Log::info("Smoke fisica: passo "
                    + std::to_string(physics.totalStepMilliseconds)
                    + " ms; contatos "
                    + std::to_string(physics.discreteContactPairs)
                    + "; reportados "
                    + std::to_string(physics.reportedContactPairs)
                    + "; workers "
                    + std::to_string(physics.physicsWorkerCount) + ".");
            }
            requestQuit();
        }
    }
    if (m_videoConfirm.active) {
        m_videoConfirm.secondsLeft -= deltaTime;
        if (m_videoConfirm.secondsLeft <= 0.0f) {
            revertVideoSettings();
        }
    }
    m_notification.secondsRemaining = std::max(0.0f,
        m_notification.secondsRemaining - deltaTime);
    if (m_startPhysicsBenchmarkWhenReady && m_laboratoryMapLoaded
        && m_propCatalog.loaded() && m_humanRagdollProfile) {
        m_automaticBenchmarkWarmupSeconds -= deltaTime;
        if (m_automaticBenchmarkWarmupSeconds <= 0.0f) {
            m_startPhysicsBenchmarkWhenReady = false;
            beginPhysicsBenchmark();
            if (m_physicsBenchmarkRunning) {
                m_automaticQuitSeconds =
                    m_automaticBenchmarkDurationSeconds;
                Log::info("Benchmark automatico: "
                    + std::to_string(m_physicsBenchmarkRequestedProps)
                    + " props, "
                    + std::to_string(
                        m_physicsBenchmarkRequestedRagdolls)
                    + " ragdolls, "
                    + std::to_string(
                        m_physicsBenchmarkSpawnRatePerSecond)
                    + " entidades/s.");
            }
        }
    }
    if (m_screen == Screen::Laboratory) {
        // Mola curta e subamortecida: resposta rapida, um unico retorno suave
        // e nenhum deslocamento permanente do viewmodel.
        constexpr float RecoilAngularFrequency = 42.0f;
        constexpr float RecoilDampingRatio = 0.78f;
        const float acceleration =
            -RecoilAngularFrequency * RecoilAngularFrequency
                * m_physGunRecoilOffset
            -2.0f * RecoilDampingRatio * RecoilAngularFrequency
                * m_physGunRecoilVelocity;
        m_physGunRecoilVelocity += acceleration * deltaTime;
        m_physGunRecoilOffset = std::clamp(
            m_physGunRecoilOffset + m_physGunRecoilVelocity * deltaTime,
            -0.018f, 0.145f);
        if (std::abs(m_physGunRecoilOffset) < 0.00005f
            && std::abs(m_physGunRecoilVelocity) < 0.002f) {
            m_physGunRecoilOffset = 0.0f;
            m_physGunRecoilVelocity = 0.0f;
        }
    }
    if (m_screen == Screen::Laboratory && !m_laboratoryPaused) {
        updateLaboratory(deltaTime);
    }
    if (m_screen == Screen::AnimationViewer && m_animationViewerPlaying
        && !m_proceduralAnimationClips.empty()) {
        m_animationViewerSelectedIndex = std::min(
            m_animationViewerSelectedIndex,
            m_proceduralAnimationClips.size() - 1);
        const AnimationClip3D& clip =
            m_proceduralAnimationClips[m_animationViewerSelectedIndex];
        m_animationViewerPlaybackSeconds +=
            deltaTime * m_animationViewerPlaybackSpeed;
        if (clip.durationSeconds > 0.0f
            && m_animationViewerPlaybackSeconds > clip.durationSeconds) {
            if (m_animationViewerLoop) {
                m_animationViewerPlaybackSeconds = std::fmod(
                    m_animationViewerPlaybackSeconds, clip.durationSeconds);
            } else {
                m_animationViewerPlaybackSeconds = clip.durationSeconds;
                m_animationViewerPlaying = false;
            }
        }
    }
}

void WorkbenchApp::onRender(Renderer& activeRenderer) {
    // Fora do laboratorio nao existe uma sequencia continua de imagens da
    // mesma camera. Ao retornar, o primeiro quadro precisa semear um novo
    // historico em vez de reprojetar a ultima imagem vista antes do menu.
    if (m_screen != Screen::Laboratory) {
        m_laboratoryTaaHistoryValid = false;
    }
    if (m_screen == Screen::Laboratory) {
        ensurePropAssetsLoaded(activeRenderer);
        ensureRagdollAssetsLoaded(activeRenderer);
        if (!m_propPreviewAtlas && m_propCatalog.loaded()) {
            // O renderer 3D usa um snapshot uniforme por frame. O atlas é
            // preparado em um frame próprio e preservado como textura; assim
            // a câmera do laboratório não sobrescreve a câmera dos previews
            // antes de a GPU executar os comandos.
            m_propPreviewAtlas = renderPropPreviewAtlas(activeRenderer);
            m_objectViewerPreview = {};
            return;
        }
        renderLaboratory3D(activeRenderer);
    } else if (m_screen == Screen::ObjectViewer) {
        ensurePropAssetsLoaded(activeRenderer);
        if (m_propCatalog.loaded()
            && m_objectViewerSelectedIndex
                < m_propCatalog.definitions().size()) {
            m_objectViewerPreview = renderObjectViewerPreview(
                activeRenderer, m_objectViewerSelectedIndex);
            // renderScene3D reutiliza o alvo offscreen; o atlas será
            // reconstruído ao retornar ao laboratório.
            m_propPreviewAtlas = {};
        }
    } else if (m_screen == Screen::AnimationViewer) {
        ensureRagdollAssetsLoaded(activeRenderer);
        if (m_humanRagdollProfile && !m_characterVisual.parts.empty()) {
            m_animationViewerPreview =
                renderAnimationViewerPreview(activeRenderer);
            m_propPreviewAtlas = {};
            m_objectViewerPreview = {};
        }
    }
}

void WorkbenchApp::updateUiScale() {
    static const ImGuiStyle baseStyle = ImGui::GetStyle();
    constexpr float ReferenceWidth = 1920.0f;
    constexpr float ReferenceHeight = 1080.0f;
    const ImGuiIO& io = ImGui::GetIO();
    m_uiScale = std::clamp(std::min(io.DisplaySize.x / ReferenceWidth,
                               io.DisplaySize.y / ReferenceHeight),
        0.92f, 2.25f);
    ImGui::GetIO().FontGlobalScale = m_uiScale;
    ImGui::GetStyle() = baseStyle;
    ImGui::GetStyle().ScaleAllSizes(m_uiScale);
}

float WorkbenchApp::ui(float value) const {
    return value * m_uiScale;
}

void WorkbenchApp::onGui(Renderer& activeRenderer) {
    m_renderer = &activeRenderer;
    updateUiScale();

    switch (m_screen) {
    case Screen::MainMenu:
        drawMainMenu();
        break;
    case Screen::Laboratory:
        drawLaboratoryUi();
        break;
    case Screen::ObjectViewer:
        drawObjectViewer();
        break;
    case Screen::AnimationViewer:
        drawAnimationViewer();
        break;
    case Screen::Settings:
        drawSettings();
        break;
    }

    if (m_videoConfirm.active) {
        drawVideoConfirmPopup();
    }
}

void WorkbenchApp::onStop() {
    renderer().setMouseCaptured(false);
    m_worldAudio.shutdown();
    releaseLaboratoryAssets(renderer());
    m_physicsScene.reset();
    Log::info("MatterEngine Workbench finalizado.");
}

} // namespace MatterEngine::Workbench
