#include "Workbench/WorkbenchApp.hpp"

#include "Engine/Core/Log.hpp"
#include "Engine/Platform/PlatformServices.hpp"
#include "imgui.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
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

ApplicationConfig initialConfiguration() {
    const Data::VideoSettings video =
        Data::SettingsRepository(settingsDatabasePath()).loadVideoSettings();
    ApplicationConfig config;
    config.title = "MatterEngine";
    config.width = std::max(960, video.width);
    config.height = std::max(540, video.height);
    // Durante o desenvolvimento, renderizar sem limite torna regressões de
    // desempenho imediatamente visíveis no contador. A simulação permanece
    // independente e determinística nos 120 Hz definidos abaixo.
    config.vsync = false;
    config.displayMode = video.mode;
    config.fixedUpdateHz = 120.0f;
    config.maxFixedStepsPerFrame = 8;
    return config;
}

} // namespace

WorkbenchApp::WorkbenchApp()
    : Application(initialConfiguration()),
      m_physicsEngine(taskScheduler()) {
}

void WorkbenchApp::onStart() {
    Log::info("MatterEngine Workbench iniciado.");
    PhysicsSceneSettings3D physicsSettings;
    physicsSettings.solverPositionIterations = 8;
    physicsSettings.solverVelocityIterations = 2;
    physicsSettings.enableContinuousCollision = true;
    physicsSettings.enableStabilization = true;
    m_physicsScene = m_physicsEngine.createScene(physicsSettings,
        m_materialLibrary);
    m_menuLogo = renderer().loadUiTexture(
        std::string(MATTERENGINE_ASSETS_DIR) + "/ui/matter-engine-logo.png");
    m_hrtfEnabled = Data::SettingsRepository(settingsDatabasePath())
        .getInt("audio.hrtf_enabled").value_or(1) != 0;
    static_cast<void>(m_worldAudio.initialize(
        MATTERENGINE_ASSETS_DIR, m_hrtfEnabled));
    applyAutostartIfRequested();
}

void WorkbenchApp::applyAutostartIfRequested() {
    const std::string mode = environmentValue("MATTERENGINE_AUTOSTART");
    if (mode == "laboratory" || mode == "laboratory-debug"
        || mode == "laboratory-smoke"
        || mode == "laboratory-spawn-smoke") {
        enterLaboratory(true);
        m_laboratoryDebugVisible = mode == "laboratory-debug";
        m_spawnMenuOpen = mode == "laboratory-spawn-smoke";
        if (mode == "laboratory-smoke"
            || mode == "laboratory-spawn-smoke") {
            m_automaticQuitSeconds = 6.0f;
        }
        syncLaboratoryMouseCapture();
    } else if (mode == "object-viewer" || mode == "object-viewer-smoke") {
        m_screen = Screen::ObjectViewer;
        if (mode == "object-viewer-smoke") m_automaticQuitSeconds = 6.0f;
    } else if (mode == "settings") {
        m_settingsReturnScreen = Screen::MainMenu;
        m_screen = Screen::Settings;
    }
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
            && !m_laboratoryDebugVisible && !m_spawnMenuOpen) {
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
        } else if (m_screen == Screen::ObjectViewer) {
            m_screen = Screen::MainMenu;
        }
    }
}

void WorkbenchApp::onUpdate(float deltaTime) {
    if (m_automaticQuitSeconds > 0.0f) {
        m_automaticQuitSeconds -= deltaTime;
        if (m_automaticQuitSeconds <= 0.0f) {
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
