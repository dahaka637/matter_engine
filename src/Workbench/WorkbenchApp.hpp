#pragma once

#include "Engine/Core/Application.hpp"
#include "Engine/Audio/ImpactAcoustics.hpp"
#include "Engine/Data/SettingsRepository.hpp"
#include "Engine/Environment/WindSystem.hpp"
#include "Engine/Materials/MaterialLibrary.hpp"
#include "Engine/Math/Quaternion.hpp"
#include "Engine/Physics/PhysicsScene3D.hpp"
#include "Engine/RHI/RHIHandles.hpp"
#include "Engine/Render/Scene3D.hpp"
#include "Workbench/Audio/WorldAudioController.hpp"
#include "Workbench/Props/PropCatalog.hpp"

#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace MatterEngine::Workbench {

// Composition root do ambiente de desenvolvimento. A classe coordena telas e
// ciclo de vida; física, importação, renderização e persistência continuam em
// módulos da engine e não dependem da UI.
class WorkbenchApp final : public Application {
public:
    WorkbenchApp();

protected:
    void onStart() override;
    void onEvent(const Event& event) override;
    void onUpdate(float deltaTime) override;
    void onRender(Renderer& renderer) override;
    void onGui(Renderer& renderer) override;
    void onStop() override;

private:
    enum class Screen {
        MainMenu,
        Laboratory,
        ObjectViewer,
        Settings
    };

    struct VideoConfirm {
        bool active = false;
        float secondsLeft = 0.0f;
        Data::VideoSettings previous;
        Data::VideoSettings applied;
    };

    struct LaboratoryMapPart {
        RHI::BufferHandle vertexBuffer;
        RHI::BufferHandle indexBuffer;
        RHI::TextureHandle albedoTexture;
        std::uint32_t indexCount = 0;
        float metallic = 0.0f;
        float roughness = 1.0f;
        std::string name;
        std::string materialId;
    };

    struct SpawnedPropInstance {
        std::uint64_t entityId = 0;
        std::size_t definitionIndex = 0;
        PhysicsBodyHandle3D physicsBody;
        PhysicsBodyState3D physicsState;
        // Pose observada no ultimo frame realmente gravado pela GPU. Nao e a
        // pose do fixed step anterior: podem existir varios passos fisicos
        // ou nenhum entre dois renders. Ver renderLaboratory3D.
        PhysicsBodyState3D previousPhysicsState;
        float freezeFlashSeconds = 0.0f;
    };

    // Ajuste persistente do viewmodel. A seção recolhível do depurador permite
    // recalibrar assets futuros sem contaminar a lógica física da Physgun.
    struct PhysGunViewCalibration {
        Vec3 offsetCameraLocal { 0.300f, 0.665f, -0.200f };
        Vec3 rotationDegrees { -1.50f, -0.50f, -97.50f };
        float sizeMeters = 0.950f;
    };

    enum class PhysGunHoldMode {
        GrabPoint,
        FixedPose
    };

    struct TransientNotification {
        std::string text;
        float secondsRemaining = 0.0f;
        float durationSeconds = 2.35f;
    };

    struct PhysGunFlashlightSettings {
        Vec3 color { 1.0f, 1.0f, 1.0f };
        float coneDegrees = 34.0f;
        float rangeMeters = 28.0f;
        float intensity = 3.0f;
        bool enabled = false;
    };

    void updateUiScale();
    [[nodiscard]] float ui(float value) const;
    void applyAutostartIfRequested();

    void drawMainMenu();
    void drawObjectViewer();
    void drawSettings();
    void drawSettingsVideoTab();
    void drawSettingsGeneralTab();
    void populateAvailableResolutions();
    void applyAndSaveVideoSettings();
    void confirmVideoSettings();
    void revertVideoSettings();
    void drawVideoConfirmPopup();
    void setHrtfEnabled(bool enabled);

    void enterLaboratory(bool resetCamera = false);
    void updateLaboratory(float deltaTime);
    void renderLaboratory3D(Renderer& renderer);
    void drawLaboratoryUi();
    void drawLaboratoryPauseMenu();
    void drawSpawnMenu();
    void syncLaboratoryMouseCapture();
    void ensureLaboratoryMapLoaded(Renderer& renderer);
    void ensurePropAssetsLoaded(Renderer& renderer);
    void releaseLaboratoryAssets(Renderer& renderer);
    void resetLaboratoryCharacter();
    void removeLatestSpawnedEntity();
    void spawnProp(std::size_t definitionIndex);
    void beginPhysGunGrab();
    void endPhysGunGrab();
    void freezePhysGunObject();
    void unfreezeLookedAtObject();
    void unfreezeAllObjects();
    void updateDynamicProps(float deltaTime);
    [[nodiscard]] UiTexture renderPropPreviewAtlas(Renderer& renderer);
    [[nodiscard]] UiTexture renderObjectViewerPreview(Renderer& renderer,
        std::size_t definitionIndex);
    void ensureObjectViewerFloorLoaded(Renderer& renderer);

    Screen m_screen = Screen::MainMenu;
    Screen m_settingsReturnScreen = Screen::MainMenu;
    Renderer* m_renderer = nullptr;
    UiTexture m_menuLogo;
    UiTexture m_propPreviewAtlas;
    UiTexture m_objectViewerPreview;
    float m_uiScale = 1.0f;

    int m_settingsTab = 0;
    bool m_resolutionsPopulated = false;
    std::vector<std::pair<int, int>> m_availableResolutions;
    int m_pendingResolutionIndex = 0;
    int m_pendingDisplayMode = 0;
    std::string m_videoApplyStatus;
    VideoConfirm m_videoConfirm;
    // O contador é útil durante todo o desenvolvimento e aparece por padrão.
    // Ainda pode ser ocultado pela opção correspondente nas configurações.
    bool m_showPerformanceOverlay = true;
    // Lido de SettingsRepository em onStart() e persistido quando o
    // checkbox correspondente muda; ver applyAndSaveAudioSettings().
    bool m_hrtfEnabled = true;
    float m_automaticQuitSeconds = -1.0f;

    Vec3 m_laboratoryCameraPosition { 0.0f, -90.0f, 4.6f };
    float m_laboratoryEyeHeight = 1.68f;
    float m_laboratoryCameraYaw = 1.57079632679f;
    float m_laboratoryCameraPitch = -0.08f;
    float m_laboratoryElapsedTime = 0.0f;
    bool m_laboratoryInitialized = false;
    bool m_laboratoryPaused = false;
    bool m_laboratoryConfirmQuit = false;
    bool m_laboratoryDebugVisible = false;
    // Visibilidade independente de cada painel de depuracao flutuante,
    // acionada pelos botoes na barra logo abaixo do cabecalho "MATTERENGINE
    // | LABORATÓRIO" (ver drawLaboratoryUi) - cada um abre/fecha por conta
    // propria, nao e mais uma unica janela com abas. So tem efeito visivel
    // enquanto m_laboratoryDebugVisible tambem esta ligado (o `'` continua
    // sendo o unico controle de captura do mouse).
    bool m_showPhysgunPanel = false;
    bool m_showGraphicsPanel = false;
    bool m_showPerformancePanel = false;
    // Controles de tonemap/correcao de cor (ver Scene3DFrame::toneMapping e
    // tonemap.frag) - ajustaveis ao vivo pelos sliders na aba "Gráficos" do
    // painel de debug, para calibrar a olho contra o conjunto de cores desta
    // engine em vez de adivinhar valores fixos.
    ToneMappingSettings3D m_laboratoryToneMapping;
    // Neblina atmosferica (ver Scene3DFrame::fog e scene3d_mesh.frag) -
    // mesmo padrao acima: ajustavel ao vivo na aba "Gráficos".
    FogSettings3D m_laboratoryFog;
    // Direcao do sol (de onde ele vem, pra onde a luz viaja - mesma
    // convencao de LightRender3D::direction). Campo nomeado e persistente
    // de proposito, em vez de um Vec3 literal solto dentro de
    // renderLaboratory3D como antes das Cascaded Shadow Maps - hoje nada
    // ainda muda este valor quadro a quadro (o sol e fixo), mas um futuro
    // ciclo dia/noite so precisa passar a escrever aqui, num lugar so, em
    // vez de caçar o literal.
    Vec3 m_laboratorySunDirection = Vec3 { -0.44f, -0.31f, 0.84f }.normalized();
    // Fonte unica de verdade do vento (ver WindSystem) - alimenta a rolagem
    // de nuvens do ceu, o arrasto aerodinamico da fisica e o assobio de
    // vento no ouvido do jogador, todos a partir do mesmo estado.
    WindSystem m_windSystem;
    // Deslocamento acumulado (integrado quadro a quadro) enviado ao shader
    // do ceu como Scene3DFrame::cloudWindOffset - ver o comentario la sobre
    // por que precisa ser uma integral e nao velocidade*tempo_total.
    Vec2 m_cloudWindOffset;
    // Conversao entre metros reais percorridos pelo vento e unidades de
    // ruido do shader do ceu - uma nuvem de verdade e enorme (centenas de
    // metros), entao 1 unidade de ruido corresponde a uma distancia real
    // bem maior que 1 metro. Puramente visual (nao afeta fisica/audio, que
    // trabalham em m/s reais). Fixo no valor ja calibrado - o painel de
    // depuracao que ajustava isto ao vivo ("Áudio Procedural") foi removido.
    float m_cloudWindVisualScale = 0.0026f;
    // Ganho mestre do assobio de vento (ver WorldAudioController::
    // updateWindAmbience). Baixo de proposito: e um toque de realismo
    // sutil, nao um efeito sonoro chamativo. Fixo no valor ja calibrado,
    // mesmo motivo do campo acima.
    float m_windAudioVolume = 0.30f;
    bool m_spawnMenuOpen = false;
    bool m_laboratoryWindowFocused = true;
    bool m_discardNextMouseDelta = true;
    bool m_laboratoryJumpRequested = false;
    bool m_laboratoryFlightToggleRequested = false;
    bool m_laboratoryCharacterInitialized = false;
    std::string m_laboratoryStatus;

    PhysicsEngine3D m_physicsEngine;
    MaterialLibrary m_materialLibrary;
    std::unique_ptr<PhysicsScene3D> m_physicsScene;
    PhysicsHandleSettings3D m_physGunSettings;
    PropCatalog m_propCatalog;
    GpuModel3D m_physGunVisual;
    bool m_propAssetsLoadFailed = false;
    bool m_physGunVisualLoaded = false;
    // Piso do Object Viewer: uma unica mesh real (quad + textura de
    // xadrez assada), carregada uma vez sob demanda. Substitui o antigo
    // plano procedural analitico (ver PropRuntime.cpp) para que 100% do
    // conteudo visual passe pelo mesmo pipeline de mesh/material.
    GpuModel3D m_objectViewerFloor;
    bool m_objectViewerFloorLoaded = false;
    std::vector<SpawnedPropInstance> m_spawnedProps;
    std::unordered_map<std::uint32_t, std::size_t> m_spawnedPropByBodyIndex;
    std::uint64_t m_nextEntityId = 1;
    std::uint64_t m_physGunGrabbedEntityId = 0;
    Vec3 m_physGunLocalGrabPoint;
    Quaternion m_physGunTargetOrientation;
    Quaternion m_physGunRelativeOrientation;
    Quaternion m_physGunRotationStartOrientation;
    Vec3 m_physGunFixedCenterCameraLocal;
    Vec2 m_physGunRotationInputDegrees;
    float m_physGunHoldDistance = 3.0f;
    float m_physGunRotationSmoothness = 0.72f;
    float m_physGunRotationSnapDegrees = 15.0f;
    float m_physGunRecoilOffset = 0.0f;
    float m_physGunRecoilVelocity = 0.0f;
    // Posicao/orientacao da arma no quadro anterior - alimenta o vetor de
    // movimento por pixel do TAA (ver MeshRender3D::previousPosition), ja
    // que a arma nao vem da fisica como os props (recalculada toda vez a
    // partir da pose da camera, sem estado persistente proprio ate agora).
    // Atualizados logo apos appendGpuModelRenderables em renderLaboratory3D.
    Vec3 m_previousPhysGunWeaponPosition;
    Quaternion m_previousPhysGunWeaponOrientation;
    PhysGunHoldMode m_physGunHoldMode = PhysGunHoldMode::GrabPoint;
    bool m_physGunTriggerHeld = false;
    bool m_physGunOrientationLocked = false;
    PhysGunViewCalibration m_physGunViewCalibration;
    std::string m_physGunCalibrationCopyStatus;
    std::string m_toneMappingCopyStatus;
    Vec3 m_physGunBeamStartCameraLocal { 0.204f, 0.605f, -0.105f };
    Vec3 m_physGunBeamStartWorldPosition;
    std::string m_physGunBeamCalibrationCopyStatus;
    Vec3 m_physGunBeamTargetWorldPosition;
    Mat4 m_laboratoryViewProjection;
    // Fase 6 (TAA): VP jitterada do Laboratorio no quadro anterior, pro
    // resolve reprojetar o historico - espelha m_laboratoryViewProjection
    // acima, mas com o deslocamento sub-pixel (ver renderLaboratory3D).
    // Matriz estavel (sem jitter) do ultimo frame realmente renderizado.
    // Vetores de movimento nao podem incorporar a sequencia sub-pixel.
    Mat4 m_previousLaboratoryViewProjection;
    // So fica verdadeiro depois que uma cena do laboratorio foi realmente
    // gravada. Impede usar matriz/historico antigos ao entrar ou retomar a
    // tela, e tambem controla o primeiro motion vector da Physgun/props.
    bool m_laboratoryTaaHistoryValid = false;
    // Contador de quadro monotonico usado pra gerar o jitter de Halton -
    // incrementado uma vez por onRender() (ver WorkbenchApp.cpp). Nao e o
    // currentFrame do backend Vulkan, que so alterna 0/1 pro duplo buffer e
    // nao serve de chave de sequencia de jitter.
    std::uint32_t m_taaFrameIndex = 0;
    PhysGunFlashlightSettings m_physGunFlashlight;
    float m_lastUnfreezePressSeconds = -10.0f;
    TransientNotification m_notification;
    std::size_t m_objectViewerSelectedIndex = 0;

    ImpactAcousticResolver m_impactAcousticResolver;
    WorldAudioController m_worldAudio;

    CharacterMotorSettings3D m_characterSettings;
    std::vector<LaboratoryMapPart> m_laboratoryMapParts;
    std::vector<PhysicsBodyHandle3D> m_laboratoryMapBodies;
    std::vector<RHI::TextureHandle> m_laboratoryMapTextures;
    bool m_laboratoryMapLoaded = false;
    bool m_laboratoryMapLoadFailed = false;
    Vec3 m_laboratorySpawnPosition;
    float m_laboratorySpawnYaw = 1.57079632679f;
};

} // namespace MatterEngine::Workbench
