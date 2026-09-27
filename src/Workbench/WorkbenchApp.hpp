#pragma once

#include "Engine/Animation/AnimationClip3D.hpp"
#include "Engine/Animation/RagdollCharacter3D.hpp"
#include "Engine/Core/Application.hpp"
#include "Engine/Audio/DragAcoustics.hpp"
#include "Engine/Audio/ImpactAcoustics.hpp"
#include "Engine/Character/BiomechanicalBipedExperiment3D.hpp"
#include "Engine/Character/CharacterLocomotion3D.hpp"
#include "Engine/Control/RagdollImpactTest3D.hpp"
#include "Engine/Data/SettingsRepository.hpp"
#include "Engine/Environment/WindSystem.hpp"
#include "Engine/Materials/MaterialLibrary.hpp"
#include "Engine/Math/Quaternion.hpp"
#include "Engine/Physics/PhysicsScene3D.hpp"
#include "Engine/RHI/RHIHandles.hpp"
#include "Engine/Render/Scene3D.hpp"
#include "Workbench/Audio/WorldAudioController.hpp"
#include "Workbench/Props/PropCatalog.hpp"

#include <array>
#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
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
        AnimationViewer,
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
        std::vector<GpuMeshLod3D> lods;
        float metallic = 0.0f;
        float roughness = 1.0f;
        bool matteSurface = false;
        Vec3 boundsCenter;
        float boundsRadius = 0.0f;
        std::string name;
        std::string materialId;
    };

    struct SpawnedPropInstance {
        std::uint64_t entityId = 0;
        std::size_t definitionIndex = 0;
        PhysicsBodyHandle3D physicsBody;
        PhysicsBodyState3D physicsState;
        // Pose antes do ultimo passo que moveu este prop, e o indice desse
        // passo. So interpola quem foi atualizado no passo mais recente.
        // Defensivo: o contrato de activeBodyStates() promete apenas os
        // corpos que mudaram. O backend Jolt continua publicando um corpo ate
        // ele desativar, e ate la as duas poses ja convergiram: medido, um
        // prop dormindo fica em 0 px mesmo sem esta guarda; um congelado pela
        // Physgun vira cinematico parado e segue publicado (lido em
        // setBodyFrozen, nao medido). Um
        // backend que parasse de publicar no instante em que o corpo para
        // deixaria o prop interpolando para sempre entre duas poses velhas.
        PhysicsBodyState3D simulationPreviousState;
        std::uint64_t updatedAtPhysicsStep = 0;
        // Pose efetivamente desenhada neste quadro (interpolada).
        PhysicsBodyState3D renderedState;
        // Pose desenhada no quadro anterior, base dos motion vectors. Nao e a
        // pose do passo fixo anterior: podem existir varios passos fisicos ou
        // nenhum entre dois renders. Ver renderLaboratory3D.
        PhysicsBodyState3D previousPhysicsState;
        float freezeFlashSeconds = 0.0f;
    };

    enum class ArticulatedRigKind {
        Human
    };

    struct SpawnedRagdollInstance {
        std::uint64_t entityId = 0;
        RagdollHandle3D physicsRagdoll;
        RagdollState3D physicsState;
        // Snapshot da última imagem enviada à GPU, equivalente ao histórico
        // dos props e independente da frequência fixa da simulação.
        RagdollState3D previousPhysicsState;
        // Rumo da guia, separado da rotacao fisica da pelve. O idle em
        // alerta abre a pelve em relacao ao olhar; realimentar essa abertura
        // como comando acumularia giro em todo tick.
        float guideFacingYawRadians = 0.0f;
        std::vector<Mat4> skinMatrices;
        std::vector<Mat4> previousSkinMatrices;
        CharacterLocomotion3D locomotion;
        BiomechanicalBipedExperiment3D biomechanics;
        // What the player actually sees. The authored pose is authoritative
        // and the simulated one is blended in by physicsBlend, so at zero the
        // character renders exactly as animated - bodies still collide, they
        // just do not get to decide how it looks. Same split Unreal makes
        // with PhysicsBlendWeight.
        // The state at the PREVIOUS fixed step, kept so the renderer can
        // interpolate: physics advances at 120 Hz while frames follow the
        // display, so drawing the last simulated state directly makes the
        // character move in steps of its own against a smoothly moving
        // camera. Distinct from previousPhysicsState, which advances at
        // render cadence and exists for motion vectors.
        RagdollState3D simulationPreviousState;
        std::vector<Vec3> renderedPositions;
        std::vector<Quaternion> renderedOrientations;
        RagdollAnimationPose3D animationPose;
        RagdollAnimationPose3D previousAnimationPose;
        float physicsBlend = 0.0f;
        ArticulatedRigKind kind = ArticulatedRigKind::Human;
        bool active = true;
        bool frozen = false;
        // Per-instance color multiply, neutral by default - see
        // MeshRender3D::tintColor for how it reaches the GPU.
        Vec3 tintColor { 1.0f, 1.0f, 1.0f };
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

    struct PhysicsBenchmarkSample {
        float averageFps = 0.0f;
        float minimumFps = 0.0f;
        float averagePhysicsMilliseconds = 0.0f;
        float maximumPhysicsMilliseconds = 0.0f;
        std::uint32_t propCount = 0;
        std::uint32_t ragdollCount = 0;
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
    void loadAnimationCatalog();

    void drawMainMenu();
    void drawObjectViewer();
    void drawAnimationViewer();
    void drawSettings();
    void drawSettingsVideoTab();
    void drawSettingsGeneralTab();
    void populateAvailableResolutions();
    void applyAndSaveVideoSettings();
    void confirmVideoSettings();
    void revertVideoSettings();
    void drawVideoConfirmPopup();
    void setHrtfEnabled(bool enabled);
    void applyLaboratoryGraphicsPreset(int preset);

    void enterLaboratory(bool resetCamera = false);
    void updateLaboratory(float deltaTime);
    void renderLaboratory3D(Renderer& renderer);
    // Camera do laboratorio em duas metades: o passo fixo decide o FOCO,
    // cada quadro monta a posicao. Ver updateLaboratoryCamera.
    enum class LaboratoryCameraMode : std::uint8_t {
        None,
        FirstPerson,
        ThirdPerson
    };
    void setLaboratoryCameraFocus(Vec3 focus, LaboratoryCameraMode mode);
    void updateLaboratoryCamera(float fixedStepAlpha);
    // Smokes de suavidade de movimento; ver Laboratory/RenderMotionSmoke.cpp.
    void sampleRenderMotionSmoke(float viewportWidth, float viewportHeight);
    void advanceRenderMotionSmoke();
    void reportRenderMotionSmoke() const;
    void drawLaboratoryUi();
    void drawLaboratoryPauseMenu();
    void drawSpawnMenu();
    void syncLaboratoryMouseCapture();
    void ensureLaboratoryMapLoaded(Renderer& renderer);
    void ensureOceanClipmap(Renderer& renderer);
    void ensurePropAssetsLoaded(Renderer& renderer);
    void ensureRagdollAssetsLoaded(Renderer& renderer);
    void releaseLaboratoryAssets(Renderer& renderer);
    void resetLaboratoryCharacter();
    void removeLatestSpawnedEntity();
    void spawnProp(std::size_t definitionIndex);
    void spawnHumanRagdoll();
    [[nodiscard]] bool spawnPropAt(std::size_t definitionIndex,
        Vec3 position, Quaternion orientation, bool benchmarkEntity);
    [[nodiscard]] bool spawnHumanRagdollAt(Vec3 pelvisPosition,
        Quaternion orientation, bool benchmarkEntity);
    void updateActiveRagdolls(float deltaTime);
    [[nodiscard]] CharacterLocomotionAnimations3D
        characterLocomotionAnimations() const;
    [[nodiscard]] float ragdollGroundHeight(Vec3 position) const;
    void takeControlOfLatestCharacter();
    void releaseControlledCharacter();
    void resetControlledCharacterPose();
    void toggleControlledCharacterFrozen();
    void updateRagdollImpactLab(float deltaTime);
    [[nodiscard]] std::optional<Vec3> findSafeSpawnPosition(
        Vec3 requestedPosition, Vec3 halfExtents,
        Quaternion orientation, float maximumSearchRadiusMeters) const;
    void beginPhysicsBenchmark();
    void pausePhysicsBenchmark();
    void clearPhysicsBenchmark();
    void updatePhysicsBenchmark(float deltaTime);
    void updatePhysicsBenchmarkAnalytics(float deltaTime);
    void beginPhysGunGrab();
    void endPhysGunGrab();
    void freezePhysGunObject();
    void unfreezeLookedAtObject();
    void unfreezeAllObjects();
    void updateDynamicProps(float deltaTime);
    [[nodiscard]] UiTexture renderPropPreviewAtlas(Renderer& renderer);
    [[nodiscard]] UiTexture renderObjectViewerPreview(Renderer& renderer,
        std::size_t definitionIndex);
    [[nodiscard]] UiTexture renderAnimationViewerPreview(Renderer& renderer);
    void ensureObjectViewerFloorLoaded(Renderer& renderer);

    Screen m_screen = Screen::MainMenu;
    Screen m_settingsReturnScreen = Screen::MainMenu;
    Renderer* m_renderer = nullptr;
    UiTexture m_menuLogo;
    UiTexture m_propPreviewAtlas;
    UiTexture m_objectViewerPreview;
    UiTexture m_animationViewerPreview;
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
    float m_automaticBenchmarkDurationSeconds = -1.0f;
    float m_automaticBenchmarkWarmupSeconds = 0.0f;

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
    bool m_showRenderPanel = false;
    bool m_showGraphicsPanel = false;
    bool m_showPerformancePanel = false;
    bool m_showWindPanel = false;
    bool m_showTimePanel = false;
    bool m_showRagdollPanel = false;
    bool m_showCharacterPanel = false;
    bool m_showPhysicsBenchmarkPanel = false;
    SceneRenderMode3D m_laboratoryRenderMode =
        SceneRenderMode3D::Standard;
    PixelArtSettings3D m_laboratoryPixelArt;
    // Controles de tonemap/correcao de cor (ver Scene3DFrame::toneMapping e
    // tonemap.frag) - ajustaveis ao vivo pelos sliders na aba "Gráficos" do
    // painel de debug, para calibrar a olho contra o conjunto de cores desta
    // engine em vez de adivinhar valores fixos.
    ToneMappingSettings3D m_laboratoryToneMapping;
    AmbientOcclusionSettings3D m_laboratoryAmbientOcclusion;
    OpticalEffectsSettings3D m_laboratoryOpticalEffects;
    // 0=Desempenho, 1=Equilibrado, 2=Alto, 3=Personalizado.
    int m_laboratoryGraphicsPreset = 0;
    // Fonte única dos parâmetros ambientais contínuos. O futuro relógio e
    // sistema meteorológico escreverão aqui; hoje o painel Gráficos permite
    // validar a fundação de iluminação sem estados discretos de clima.
    EnvironmentLightingState3D m_laboratoryEnvironment;
    // O relógio pode ser manipulado manualmente ou avançar em escala de
    // tempo de jogo. 60x significa um minuto do mundo por segundo real.
    bool m_laboratoryClockRunning = false;
    float m_laboratoryClockTimeScale = 60.0f;
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
    // do ceu como EnvironmentLightingState3D::cloudWindOffset - ver o
    // comentario la sobre
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
    std::optional<RagdollProfile3D> m_humanRagdollProfile;
    std::optional<RagdollCharacter3D> m_ragdollCharacter;
    GpuModel3D m_characterVisual;
    UiTexture m_characterThumbnail;
    Vec3 m_characterSpawnHalfExtents;
    Vec3 m_characterSpawnCenter;
    std::string m_characterAssetPath = "characters/football_player/character.json";
    bool m_animationViewerShowColliders = false;
    std::vector<GpuModel3D> m_ragdollLinkVisuals;
    GpuModel3D m_ragdollJointVisual;
    std::vector<Vec3> m_ragdollJointLocalPositions;
    std::vector<SpawnedRagdollInstance> m_spawnedRagdolls;
    bool m_ragdollAssetsLoadFailed = false;
    bool m_ragdollActive = true;
    float m_ragdollRigidityPercent = 0.0f;
    // Explicit bounded physical assistance; the previous balance/WBC is archived.
    bool m_manualRagdollInspection = false;
    bool m_ragdollForceControlOverride = false;
    float m_ragdollAuxiliaryPercent = 100.0f;
    float m_ragdollMusclePercent = 100.0f;
    RagdollImpactTest3D m_ragdollImpactLab;
    std::uint64_t m_controlledCharacterEntityId = 0;
    Vec3 m_characterDesiredVelocityWorld;
    float m_characterDesiredFacingYaw = 0.0f;
    bool m_characterSprinting = false;
    // Laboratório de base flutuante: a cápsula deixa de transportar o corpo
    // e nenhuma autoridade pós-solver ou wrench de raiz participa. Mantido
    // como experimento isolado, desligado por padrão: o caminho padrão do
    // personagem voltou a ser o modelo híbrido (cápsula + pose autoral de
    // CharacterLocomotion3D), com o toggle no painel PERSONAGEM ainda
    // disponível para comparação.
    bool m_biomechanicalExperiment = false;
    float m_biomechanicalBalanceAssistPercent = 100.0f;
    // Oculta apenas a apresentação em primeira pessoa; controles, física,
    // lanterna e manipulação continuam ativos para captura de imagens.
    // Oculto por padrão.
    bool m_hidePhysGunPresentation = true;
    // Piso do Object Viewer: uma unica mesh real (quad + textura de
    // xadrez assada), carregada uma vez sob demanda. Substitui o antigo
    // plano procedural analitico (ver PropRuntime.cpp) para que 100% do
    // conteudo visual passe pelo mesmo pipeline de mesh/material.
    GpuModel3D m_objectViewerFloor;
    bool m_objectViewerFloorLoaded = false;
    std::vector<SpawnedPropInstance> m_spawnedProps;
    std::unordered_map<std::uint32_t, std::size_t> m_spawnedPropByBodyIndex;
    std::uint64_t m_nextEntityId = 1;
    int m_physicsBenchmarkRequestedProps = 100;
    int m_physicsBenchmarkRequestedRagdolls = 10;
    float m_physicsBenchmarkSpawnRatePerSecond = 30.0f;
    bool m_physicsBenchmarkRunning = false;
    bool m_physicsBenchmarkPaused = false;
    bool m_physicsBenchmarkShowAnalytics = true;
    float m_physicsBenchmarkSpawnAccumulator = 0.0f;
    std::uint32_t m_physicsBenchmarkSpawnedProps = 0;
    std::uint32_t m_physicsBenchmarkSpawnedRagdolls = 0;
    std::uint32_t m_physicsBenchmarkRandomState = 0x4D415454u;
    std::vector<std::uint64_t> m_physicsBenchmarkPropEntityIds;
    std::vector<std::uint64_t> m_physicsBenchmarkRagdollEntityIds;
    std::vector<PhysicsBenchmarkSample> m_physicsBenchmarkSamples;
    float m_physicsBenchmarkSampleSeconds = 0.0f;
    float m_physicsBenchmarkFpsSum = 0.0f;
    float m_physicsBenchmarkMinimumFps = 0.0f;
    float m_physicsBenchmarkPhysicsMillisecondsSum = 0.0f;
    float m_physicsBenchmarkMaximumPhysicsMilliseconds = 0.0f;
    std::uint32_t m_physicsBenchmarkFrameSamples = 0;
    float m_physicsBenchmarkBaselineFps = 0.0f;
    float m_physicsBenchmarkOverallFpsSum = 0.0f;
    float m_physicsBenchmarkOverallMinimumFps = 0.0f;
    std::uint64_t m_physicsBenchmarkOverallFrameSamples = 0;
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
    PhysGunHoldMode m_physGunHoldMode = PhysGunHoldMode::FixedPose;
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
    // Biblioteca preenchida no início com clipes canônicos já retargeteados.
    std::vector<AnimationClip3D> m_animationClips;
    // Catálogo independente exibido no workspace de animações. Começa apenas
    // com a referência parada natural e receberá movimentos procedurais
    // gerados em memória, sem expor a antiga biblioteca de locomoção.
    std::vector<AnimationClip3D> m_proceduralAnimationClips;
    // Recomputed only when the validated catalog/profile changes, not in UI frames.
    bool m_animationLocomotionCompatible = false;
    std::size_t m_animationViewerSelectedIndex = 0;
    float m_animationViewerPlaybackSeconds = 0.0f;
    float m_animationViewerPlaybackSpeed = 1.0f;
    bool m_animationViewerPlaying = true;
    bool m_animationViewerLoop = true;
    float m_animationViewerCameraYaw = -0.975f;
    float m_animationViewerCameraPitch = 0.071f;
    float m_animationViewerCameraDistance = 4.5f;

    ImpactAcousticResolver m_impactAcousticResolver;
    DragAcousticResolver m_dragAcousticResolver;
    WorldAudioController m_worldAudio;

    CharacterMotorSettings3D m_characterSettings;
    // Velocidade de cada ciclo do personagem controlado (lidas dos clipes).
    CharacterGaitSpeeds3D m_avatarGaitSpeeds;
    CharacterMotorSettings3D m_avatarCharacterSettings {
        .radius = 0.34f,
        .standingHeight = 1.83f,
        .crouchedHeight = 1.22f,
        // Substituídos em loadAnimationCatalog pelas velocidades dos clipes
        // (corrida leve 3,0 m/s, sprint 7,5 m/s); ficam só como valor
        // antes do catálogo carregar.
        .walkSpeed = 3.0f,
        .sprintSpeed = 7.5f,
        .crouchedSpeed = 0.95f,
        .groundAcceleration = 14.0f,
        .groundDeceleration = 18.0f,
        .airAcceleration = 4.5f,
        .jumpSpeed = 4.6f,
        .gravityScale = 1.0f,
        .maximumFallSpeed = 48.0f,
        .maximumSlopeDegrees = 50.0f,
        // Sobe os degraus da escada do laboratorio (18 cm) e ainda deixa os
        // meios-fios de 20/30 cm como obstaculo de verdade. Com 12 cm a
        // escada nao subia andando.
        .maximumStepHeight = 0.19f,
        .skinWidth = 0.018f,
        .coyoteTime = 0.10f,
        .jumpBufferTime = 0.12f,
        .flightSpeed = 12.0f,
        .fastFlightSpeed = 28.0f,
        .swimSpeed = 3.4f,
        .fastSwimSpeed = 5.0f,
        .swimAcceleration = 9.0f,
        .maximumPushSpeedMetersPerSecond = 2.2f,
        .pushSaturationPenetrationMeters = 0.25f
    };
    std::vector<LaboratoryMapPart> m_laboratoryMapParts;
    std::vector<PhysicsBodyHandle3D> m_laboratoryMapBodies;
    std::vector<RHI::TextureHandle> m_laboratoryMapTextures;
    OceanVolume3D m_laboratoryOcean;
    bool m_laboratoryOceanEnabled = false;
    RHI::BufferHandle m_laboratoryOceanVertexBuffer;
    RHI::BufferHandle m_laboratoryOceanIndexBuffer;
    std::uint32_t m_laboratoryOceanIndexCount = 0;
    bool m_laboratoryMapLoaded = false;
    bool m_laboratoryMapLoadFailed = false;
    bool m_spawnRagdollWhenReady = false;
    bool m_takeCharacterControlWhenReady = false;
    // Foco da camera nos dois ultimos passos fixos, para interpolar com o
    // mesmo alfa usado pelo render do boneco.
    LaboratoryCameraMode m_laboratoryCameraMode = LaboratoryCameraMode::None;
    // Passos de simulacao executados, para saber quem se moveu no ultimo.
    std::uint64_t m_laboratoryPhysicsStep = 0;
    Vec3 m_laboratoryCameraFocusPrevious;
    Vec3 m_laboratoryCameraFocus;
    struct RenderMotionSmoke {
        enum class Mode : std::uint8_t { Disabled, CameraOrbit, PropMotion };
        Mode mode = Mode::Disabled;
        bool started = false;
        bool finished = false;
        std::chrono::steady_clock::time_point startTime;
        bool probePlaced = false;
        Vec3 probeWorld;
        std::uint64_t probeEntityId = 0;
        bool hasPreviousSample = false;
        std::chrono::steady_clock::time_point previousSampleTime;
        float previousScreenX = 0.0f;
        float previousScreenY = 0.0f;
        bool hasPreviousVelocity = false;
        float previousVelocityX = 0.0f;
        float previousVelocityY = 0.0f;
        bool hasAdvanced = false;
        std::chrono::steady_clock::time_point previousAdvanceTime;
        std::vector<float> movingErrors;
        std::vector<float> restingErrors;
    } m_renderMotionSmoke;
    bool m_animationRunTest = false;
    bool m_animationWalkTest = false;
    bool m_animationRunTestStarted = false;
    float m_animationRunTestSeconds = 0;
    bool m_spawnAllPropsWhenReady = false;
    bool m_startPhysicsBenchmarkWhenReady = false;
    // Automação de benchmark/smoke. Vazio no uso interativo; evita depender
    // de eventos sintéticos do menu Q para perfilar um prop específico.
    std::string m_spawnPropWhenReadyId;
    Vec3 m_laboratorySpawnPosition;
    float m_laboratorySpawnYaw = 1.57079632679f;
};

} // namespace MatterEngine::Workbench
