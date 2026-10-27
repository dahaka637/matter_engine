#pragma once

#include "Engine/Animation/AnimationClip3D.hpp"
#include "Engine/Animation/RagdollCharacter3D.hpp"
#include "Engine/Animation/RagdollVisualPose3D.hpp"
#include "Engine/Core/Application.hpp"
#include "Engine/Athlete/AthleteAssets3D.hpp"
#include "Engine/Athlete/AthleteRuntime3D.hpp"
#include "Engine/Athlete/AthleteState3D.hpp"
#include "Engine/Character/BallOwnership3D.hpp"
#include "Engine/Character/BiomechanicalBipedExperiment3D.hpp"
#include "Engine/Character/CharacterLocomotion3D.hpp"
#include "Engine/Character/DefaultLocomotion3D.hpp"
#include "Engine/Control/RagdollImpactTest3D.hpp"
#include "Engine/Control/AdaptivePhysicalCharacter3D.hpp"
#include "Engine/Control/ImpactAssist3D.hpp"
#include "Engine/Data/SettingsRepository.hpp"
#include "Engine/Environment/WindSystem.hpp"
#include "Engine/Materials/MaterialLibrary.hpp"
#include "Engine/Math/Frustum3D.hpp"
#include "Engine/Math/Mat4.hpp"
#include "Engine/Modeling/Objects/ProceduralObject.hpp"
#include "Engine/Math/Quaternion.hpp"
#include "Engine/Physics/PhysicsScene3D.hpp"
#include "Engine/RHI/RHIHandles.hpp"
#include "Engine/Render/Scene3D.hpp"
#include "Workbench/Audio/WorldAudioController.hpp"
#include "Workbench/Laboratory/LaboratoryBody.hpp"
#include "Workbench/ObjectInspector/ObjectInspectorScreen.hpp"
#include "Workbench/PlayerInspector/PlayerInspectorScreen.hpp"
#include "Workbench/PlayerInspector/UiCanvasTexture.hpp"
#include "Workbench/ReinforcementLearning/ReinforcementLearningScreen.hpp"
#include "Workbench/Props/PropCatalog.hpp"

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <future>
#include <memory>
#include <optional>
#include <span>
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
        ObjectInspector,
        ReinforcementLearning,
        PlayerInspector,
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

    // A trave com rede do mapa (GoalRuntime.cpp): o objeto, o corpo fixo da
    // estrutura, a rede na cena, a malha da trave, a dos fios (a pele dos nos
    // da fisica, sem sombra: o fio de 4 mm tremia no mapa de sombra) e os nos
    // nos dois ultimos passos (o desenho interpola) com as matrizes do quadro
    // e do anterior (o TAA).
    struct LaboratoryGoal {
        std::unique_ptr<Modeling::ProceduralObject> object;
        Vec3 position;
        Quaternion orientation;
        PhysicsBodyHandle3D body;
        std::vector<PhysicsBodyHandle3D> barriers;
        NetHandle3D net;
        GpuModel3D frame;
        RHI::BufferHandle strandVertices;
        RHI::BufferHandle strandIndices;
        std::uint32_t strandIndexCount = 0;
        Vec3 color { 1.0f, 1.0f, 1.0f };
        std::vector<Vec3> restKnots;
        std::vector<Vec3> previousKnots;
        std::vector<Vec3> knots;
        std::vector<Vec3> renderedKnots;
        std::vector<Mat4> skin;
        std::vector<Mat4> previousSkin;
        bool previousSkinValid = false;
        bool renderedThisFrame = false;
        Vec3 boundsCenter;
        float boundsRadius = 0.0f;
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

    // A bola de futebol (bola inflada, InflatedBall3D.hpp): fisica propria,
    // malha procedural e deformacao por skinning (ProceduralSoccerBall3D).
    struct SpawnedInflatedBallInstance {
        std::uint64_t entityId = 0;
        InflatedBallHandle3D physicsBall;
        // O boneco que a maneja (nao colide com ela; ver BallOwnership3D).
        BallOwnership3D ownership;
        // Estado do ultimo passo fixo e o do passo antes dele (interpolacao).
        InflatedBallState3D physicsState;
        InflatedBallState3D simulationPreviousState;
        // Pose desenhada neste quadro e no anterior (motion vectors).
        Vec3 renderedPosition;
        Quaternion renderedOrientation;
        Vec3 previousRenderedPosition;
        Quaternion previousRenderedOrientation;
        // Paleta da gaiola quando achatada; vazia = desenhada rigida.
        std::vector<Mat4> skinMatrices;
        std::vector<Mat4> previousSkinMatrices;
        bool previousSkinned = false;
        float freezeFlashSeconds = 0.0f;
        // O contorno da bola escondida atras do proprio corpo e o ponto no centro (0..1, entram e saem suaves).
        float hiddenOutline = 0.0f;
        float centerDot = 0.0f;
        // Leitura do painel: maior achatamento e maior velocidade recentes
        // (um chute dura um ou dois passos; o pico fica visivel um tempo).
        float recentPeakCompressionMeters = 0.0f;
        float recentPeakSpeed = 0.0f;
        float recentPeakAgeSeconds = 0.0f;
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
        std::vector<Mat4> skinMatrices;
        std::vector<Mat4> previousSkinMatrices;
        // Os ossos visuais (07/10): os dedos do pe no chao e a pose das maos;
        // as rotacoes deste quadro e as do anterior (os vetores de movimento).
        RagdollVisualPose3D visualPose;
        std::vector<Quaternion> visualRotations;
        std::vector<Quaternion> previousVisualRotations;
        double visualPoseTime = -1.0;
        // O controle do atleta (engine: Engine/Athlete): os controladores, o
        // modo do corpo e o manejo de bola. O runtime dos atletas
        // (m_athleteRuntime) e quem escreve nele.
        AthleteState3D athlete;
        // The state at the PREVIOUS fixed step, kept so the renderer can
        // interpolate: physics advances at 120 Hz while frames follow the
        // display, so drawing the last simulated state directly makes the
        // character move in steps of its own against a smoothly moving
        // camera. Distinct from previousPhysicsState, which advances at
        // render cadence and exists for motion vectors.
        RagdollState3D simulationPreviousState;
        std::vector<Vec3> renderedPositions;
        std::vector<Quaternion> renderedOrientations;
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
    // A tela "Objetos" (Workbench/ObjectInspector).
    void drawObjectInspector(Renderer& activeRenderer);
    // A tela "Aprendizado por Reforco" (Workbench/ReinforcementLearning).
    void drawReinforcementLearning(Renderer& activeRenderer);
    // A tela "Inspecao de jogador" (Workbench/PlayerInspector).
    void drawPlayerInspector(Renderer& activeRenderer);
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
    bool updateControlledPhysicalCameraFocus();
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
    // A caixa de spawn dos bonecos humanos (a malha e os colisores do corpo).
    void updateCharacterSpawnBox();
    void releaseLaboratoryAssets(Renderer& renderer);
    void resetLaboratoryCharacter();
    void removeLatestSpawnedEntity();
    // R (25/10, pedido do usuario): a ultima bola criada para a frente do jogador.
    void teleportLatestBallToFront();
    void spawnProp(std::size_t definitionIndex);
    void spawnHumanRagdoll();
    [[nodiscard]] bool spawnPropAt(std::size_t definitionIndex,
        Vec3 position, Quaternion orientation, bool benchmarkEntity);
    void ensureSoccerBallVisual(Renderer& renderer);
    [[nodiscard]] bool spawnInflatedBallAt(Vec3 position);
    void updateInflatedBallGrab(float deltaTime);
    void readInflatedBallStates(float deltaTime);
    void appendInflatedBallRenderables(std::vector<MeshRender3D>& meshes,
        const Frustum3D& cameraFrustum,
        std::span<const Frustum3D> lightFrustums, bool temporalHistoryValid);
    void finishInflatedBallFrame();
    // A trave com rede do mapa (GoalRuntime.cpp).
    void createLaboratoryGoal(Renderer& renderer, Vec3 goalLineCenter, float facingYaw);
    void releaseLaboratoryGoal(Renderer& renderer);
    void captureLaboratoryGoalNet();
    void appendLaboratoryGoalRenderables(std::vector<MeshRender3D>& meshes, const Frustum3D& cameraFrustum,
        std::span<const Frustum3D> lightFrustums, bool temporalHistoryValid);
    void finishLaboratoryGoalFrame();
    void drawInflatedBallPanel();
    void applySoccerBallSize();
    [[nodiscard]] bool spawnHumanRagdollAt(Vec3 pelvisPosition,
        Quaternion orientation, bool benchmarkEntity);
    void updateActiveRagdolls(float deltaTime);
    // Os ajustes do laboratorio (os m_*) para o runtime dos atletas.
    [[nodiscard]] AthleteRuntimeSettings3D athleteRuntimeSettings() const;
    [[nodiscard]] CharacterLocomotionAnimations3D
        characterLocomotionAnimations() const;
    // O corpo do laboratorio (26/10): o personagem e a biblioteca de clipes do
    // corpo aplicado (no 180/80, os originais) e o pedido do painel.
    [[nodiscard]] const RagdollCharacter3D& laboratoryCharacter() const {
        return m_laboratoryBody ? m_laboratoryBody->character : *m_ragdollCharacter;
    }
    [[nodiscard]] const std::vector<AnimationClip3D>& laboratoryClips() const {
        return m_laboratoryBody ? m_laboratoryBody->clips : m_animationClips;
    }
    [[nodiscard]] PlayerBody3D laboratoryBody() const {
        return m_laboratoryBody ? m_laboratoryBody->body : PlayerBody3D {};
    }
    void applyLaboratoryBody();
    [[nodiscard]] float ragdollGroundHeight(Vec3 position) const;
    void takeControlOfLatestCharacter();
    void releaseControlledCharacter();
    // A rede da locomocao (o RL, contrato v2): carrega a ultima politica
    // exportada pelo treino (MATTER_LOCOMOTION_POLICY, ou a do treino
    // loco_f1f2 em MATTER_RL_DATA / ~/.local/share/matter-engine/rl).
    void reloadLocomotionPolicy();
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
    void ensureViewerFloorLoaded(Renderer& renderer);

    Screen m_screen = Screen::MainMenu;
    Screen m_settingsReturnScreen = Screen::MainMenu;
    Renderer* m_renderer = nullptr;
    UiTexture m_menuLogo;
    UiTexture m_propPreviewAtlas;
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
    // O perfil dos bonecos humanos: o do personagem do corpo
    // (laboratoryCharacter); nulo com os recursos descarregados.
    const RagdollProfile3D* m_humanRagdollProfile = nullptr;
    std::optional<RagdollCharacter3D> m_ragdollCharacter;
    // O corpo (painel Personagem, aba Corpo; sem persistir): a altura (cm) e
    // o peso (kg) pedidos, aplicados ao soltar (applyLaboratoryBody, no
    // updateLaboratory); o corpo aplicado (nulo: o de hoje, os originais) e os
    // ja feitos. A malha da GPU e refeita no render (m_characterVisualDirty).
    float m_bodyHeightCm = 180.0f;
    float m_bodyMassKg = 80.0f;
    bool m_bodyApplyRequested = false;
    LaboratoryBodyCache m_laboratoryBodies;
    const LaboratoryBody* m_laboratoryBody = nullptr;
    bool m_characterVisualDirty = false;
    GpuModel3D m_characterVisual;
    // O uniforme do sistema de aparencia sobre o personagem (o material dele).
    RHI::CharacterMaterialHandle m_characterKit;
    UiTexture m_characterThumbnail;
    Vec3 m_characterSpawnHalfExtents;
    Vec3 m_characterSpawnCenter;
    std::string m_characterAssetPath = "characters/football_player/character.json";
    std::vector<GpuModel3D> m_ragdollLinkVisuals;
    GpuModel3D m_ragdollJointVisual;
    std::vector<Vec3> m_ragdollJointLocalPositions;
    std::vector<SpawnedRagdollInstance> m_spawnedRagdolls;
    // O runtime dos atletas (engine): o que antes era updateActiveRagdolls.
    AthleteRuntime3D m_athleteRuntime;
    // A rede da locomocao: ligada no menu do personagem, e o que foi
    // carregado (a rodada do treino, ou o erro).
    bool m_locomotionPolicyOn = false;
    // As ajudas: a regua unica ou as quatro (Avancado, desligado por padrao).
    bool m_assistAdvanced = false;
    // A virada brusca (experimento, so no Fisico total): desligada por padrao.
    bool m_dynamicTurnMove = false;
    float m_dynamicTurnMovePercent = 100.0f;
    std::string m_locomotionPolicyLabel;
    bool m_ragdollAssetsLoadFailed = false;
    bool m_ragdollActive = true;
    float m_ragdollRigidityPercent = 0.0f;
    // Explicit bounded physical assistance; the previous balance/WBC is archived.
    bool m_manualRagdollInspection = false;
    bool m_ragdollForceControlOverride = false;
    float m_ragdollAuxiliaryPercent = 100.0f;
    float m_ragdollMusclePercent = 100.0f;
    // Rigidez muscular geral (menu do personagem): rigidez e forca de todos
    // os musculos no modo fisico (100% e a calibrada).
    float m_characterMusclePercent = 100.0f;
    // Cambaleio (05/10 noite): depois de um choque forte o corpo amolece mas
    // tenta se manter em pe (CharacterLocomotionInput3D::staggerRecovery).
    // Sempre ligado (06/10: a caixa saiu).
    bool m_staggerRecovery = true;
    // Locomocao fisica v2 (06/10/2026): equilibrio e locomocao do modo fisico
    // (CharacterLocomotionInput3D::physicalV2, AdaptivePhysicalSettings3D::v2
    // e o limite de aceleracao da capsula). Sempre ligada (a caixa saiu).
    bool m_physicalLocomotionV2 = true;
    // (v2: o rumo pedido no tick anterior e a taxa de giro dele, filtrada -
    // physicalNavigationTurnScale3D.)
    Vec3 m_v2RequestDirection {};
    bool m_v2RequestValid = false;
    float m_v2RequestTurnRate = 0.0f;
    RagdollImpactTest3D m_ragdollImpactLab;
    std::uint64_t m_controlledCharacterEntityId = 0;
    // Velocidade da capsula (proxy de navegacao) neste tick - nao o pedido.
    Vec3 m_characterProxyVelocityWorld;
    // O pedido do jogador, puro: coletado inclusive caido/levantando.
    CharacterIntent3D m_characterIntent;
    // Experimento biomecanico: a velocidade pedida na faixa lenta dele.
    Vec3 m_biomechanicalRequestedVelocity;
    float m_characterDesiredFacingYaw = 0.0f;
    bool m_characterSprinting = false;
    // Mantido internamente para compatibilidade com os modos de diagnóstico
    // antigos; não é mais exposto no painel do personagem.
    bool m_biomechanicalExperiment = false;
    float m_biomechanicalBalanceAssistPercent = 100.0f;
    // Fisica adaptativa (habilitada por padrao): a pelve do
    // personagem controlado deixa de ser carregada pela capsula e vai a
    // pose por forca limitada, que so existe com apoio real dos pes; a
    // capsula segue o corpo quando ele e empurrado, tropeca ou cai. Ver
    // IA_Brain .../Atualizacao_de_Locomocao_e_Fisica_do_Ragdoll.md.
    bool m_adaptivePhysics = true;
    // Etapa E4 (experimental): parado, o corpo e sustentado e equilibrado so
    // pelas pernas (torque de contato nas juntas, forca zero na pelve), e o
    // passo de recuperacao mira o ponto de captura. Andando, nada muda.
    bool m_legsOnlyStanding = true;
    bool m_legsPostureStanding = true;
    // Etapa E5 (decisao de 30/09, noite): a animacao anda e corre, os pes
    // corrigem o pouso pelo equilibrio, a ajuda da pelve fica presente e
    // discreta; parado, a E4 pelas pernas com parte da ajuda. (A marcha so
    // pelas pernas, pelo planejador, foi abandonada.) Harness: XE5ANIM=1.
    bool m_legsWalkingE5 = true;
    // Locomocao padrao (04/10, pedido do usuario): o boneco controlado pela
    // capsula, sem fisica, como na maioria dos jogos; o modo fisico so em
    // momentos especificos (pancada, pouso, agarrado). Desligada: sempre o
    // fisico (comparacao). m_controlledAnimated: o controlado esta no padrao
    // neste tick (a capsula usa a aceleracao dele).
    bool m_defaultLocomotion = true;
    // Corpo Ativo (10/10, rework Etapa 2): o estado intermediario - os
    // encostoes, a parede e a volta do fisico passam por ele. A pancada (N.s)
    // que vai direto ao fisico total e a ajuda de jogo dele (m/s2).
    bool m_activeBody = false;
    bool m_physicalBridge = true;
    // A reescrita (R3, 16/10): as trocas de modo pela tabela e as classes de
    // contato (AthleteRuntimeSettings3D::modeTable). Desligado: as de antes.
    bool m_modeTable = true;
    // A mesma intencao nos dois modos (R5, 16/10): no fisico a capsula pede a
    // tabela de velocidades do padrao (AthleteRuntimeSettings3D).
    bool m_physicalDefaultSpeeds = true;
    // A ajuda nos pes do corpo fisico (%; aprovada em 11/10): a forca que leva
    // o corpo passa pelos pes apoiados, no que a ajuda da pelve nao deu. E a
    // guiagem de ossos do Fisico total (%; experimento de 11/10).
    float m_physicalFeetAssistPercent = 100.0f;
    float m_physicalBoneGuidePercent = 0.0f;
    float m_fullPhysicalImpulse = 25.0f;
    float m_activeAssistAcceleration = 4.0f;
    // O tempo do gesto de bola (24/10, pedido do usuario: a forca sai no
    // comando; so o tempo do gesto se configura), o do manejo e o do chute.
    float m_touchGestureMilliseconds = 450.0f;
    float m_shotGestureMilliseconds = 580.0f;
    // A regua "Resposta" da capsula no modo padrao (%; 100 = 16 / 22 / 30 m/s2
    // e 10 rad/s, os aprovados).
    // Os padroes calibrados do modo padrao no laboratorio (17/10, escolha do
    // usuario): o trote, a corrida para a frente, de lado e de costas (m/s), a
    // resposta (%) e a troca de direcao viva (%).
    static constexpr float LabDefaultTrotSpeed = 4.5f;
    static constexpr float LabDefaultSprintForwardSpeed = 8.0f;
    static constexpr float LabDefaultSprintSideSpeed = 6.0f;
    static constexpr float LabDefaultSprintBackwardSpeed = 6.0f;
    // O sprint maximo (24/10, pedido do usuario: o Shift duas vezes rapido e segurado, so para a frente e nas
    // diagonais da frente) e a fracao da sensibilidade da camera que fica nele.
    static constexpr float LabDefaultMaxSprintSpeed = 10.0f;
    static constexpr float LabDefaultMaxSprintCameraSensitivity = 0.4f;
    // A "Resposta" do laboratorio e mostrada na escala do usuario (17/10): 100% e o
    // que o motor chama de 150% (24 / 33 / 45 m/s2 e 15 rad/s); o motor recebe o
    // valor vezes LabResponseScale.
    static constexpr float LabResponseScale = 1.5f;
    static constexpr float LabDefaultCapsuleResponsePercent = 100.0f;
    static constexpr float LabDefaultCapsuleReversalPercent = 100.0f;
    float m_capsuleResponsePercent = LabDefaultCapsuleResponsePercent;
    float m_capsuleReversalPercent = LabDefaultCapsuleReversalPercent;
    // Transicoes adaptativas da locomocao padrao (05/10): giro da pelve com
    // aceleracao limitada, clipes misturados pelo movimento real e amostragem
    // suave. Desligada: o modo de antes (comparacao).
    bool m_defaultAdaptiveTransitions = true;
    // Volta do fisico ao padrao comecando de onde o corpo esta (05/10).
    bool m_defaultSeedFromBody = true;
    // Pulo no fisico (voo e pouso; 05/10). Desligado: o pulo e animado e so
    // um esbarrao no ar o leva ao fisico.
    bool m_defaultPhysicalJump = false;
    // Aterrissagem amortecida pela altura (05/10, Falling To Landing).
    bool m_defaultLandingAbsorb = true;
    // A partir de quanto (m/s2 de aceleracao lateral) ele inclina na curva.
    float m_defaultCurveLeanDeadZone = 3.5f;
    // O corpo do modo padrao e o guiado (aprovado pelo usuario, 05/10 noite;
    // o cinematico saiu do laboratorio): dinamico, levado a pose da animacao
    // pela velocidade de cada link - livre e o mesmo movimento; nas
    // trombadas tem massa; a ida ao fisico nao troca de corpo.
    // A saida dele para o fisico sem contato (pulo fisico, padrao
    // desligado): a autoridade da pose desce nesse tempo (s) enquanto os
    // motores assumem. Num choque (pancada, parede, carrinho) e seca:
    // segurando a pose, o corpo seguia empurrando o que ele acertou.
    float m_defaultGuidedExitSeconds = 0.12f;
    // Carrinho no ar (a voadora; 05/10 noite) e o quanto ele deita para tras
    // no voo (graus).
    bool m_defaultAirTackle = true;
    float m_defaultAirTacklePitch = 30.0f;
    // A voadora so vai ao fisico no contato - um boneco, um objeto, uma
    // parede (pedido do usuario, 06/10: mais fluido). Desligada, ela vai ao
    // fisico ja no comeco (o de 05/10 noite).
    bool m_defaultVoadoraContactOnly = true;
    // Velocidades do modo padrao (m/s; aba "Velocidades" do menu do
    // personagem, 06/10): o trote em todos os rumos; a corrida para a frente
    // (vale para as diagonais da frente), de lado e de costas (vale para as
    // diagonais de costas). Os padroes sao os de DefaultLocomotionSettings3D.
    float m_defaultTrotSpeed = LabDefaultTrotSpeed;
    float m_defaultSprintForwardSpeed = LabDefaultSprintForwardSpeed;
    float m_defaultSprintSideSpeed = LabDefaultSprintSideSpeed;
    float m_defaultSprintBackwardSpeed = LabDefaultSprintBackwardSpeed;
    float m_defaultMaxSprintSpeed = LabDefaultMaxSprintSpeed;
    float m_maxSprintCameraSensitivity = LabDefaultMaxSprintCameraSensitivity;
    // Armado pelo segundo Shift rapido (soltar desarma) e a rampa da camera dele (0..1).
    bool m_maxSprintArmed = false;
    // Esteve ativo no tick anterior: sair do cone (sem W, ou com S) o corta.
    bool m_maxSprintActive = false;
    float m_lastShiftPressSeconds = -10.0f;
    float m_maxSprintCameraSlow = 0.0f;
    // Quando os parametros foram copiados (ImGui::GetTime; o aviso some em 2 s).
    double m_defaultSpeedsCopiedTime = -10.0;
    // Levantar com pressa ja andando no padrao (05/10).
    bool m_defaultGetUpHandoff = true;
    // Quem leva carrinho fica com a ajuda baixa (05/10; desde 06/10 pelo
    // ImpactAssist3D: ate 20%, voltando num tempo pela forca do carrinho).
    bool m_tackledAssistLow = true;
    float m_tackledRecoverSeconds = 2.5f;
    // Pancada enquanto levanta (05/10): a ajuda do levantar cai a isto (%)
    // e volta neste tempo (s).
    bool m_getUpHitWeakens = true;
    float m_getUpHitAssistPercent = 25.0f;
    float m_getUpHitRecoverSeconds = 1.5f;
    // Vida do tronco e da cabeca no padrao (05/10): sobe-e-desce, inclinacao e
    // balanco calibrados pelas referencias; a intensidade escala tudo.
    bool m_defaultTrunkMotion = true;
    float m_defaultTrunkMotionPercent = 100.0f;
    // A arrancada do padrao (0 desliga): o gesto do idle_to_sprint e a
    // inclinacao do corpo com a aceleracao (e com a freada). 100% no laboratorio.
    static constexpr float LabDefaultStartGesturePercent = 100.0f;
    float m_defaultStartGesturePercent = LabDefaultStartGesturePercent;
    bool m_controlledAnimated = false;
    // A escala da capsula pedida pelo runtime (o chute correndo freia).
    float m_controlledSpeedScale = 1.0f;
    // A capsula do controlado bateu em algo neste tick: quanto perdeu de
    // velocidade (m/s, no plano) alem do que a frenagem dela tira - parede,
    // degrau alto. No padrao, a batida forte passa ao modo fisico.
    float m_controlledWallImpact = 0.0f;
    // A velocidade no plano que a capsula andou de fato no tick anterior.
    Vec3 m_capsuleMovedVelocity;
    float m_capsuleRose = 0.0f;
    // A forca na bola (aba "Forca na bola", 07/10: o toque e o chute para o
    // usuario ajustar tudo) e o tempo para encher a barra (s) de cada um.
    BallOutcomeSettings3D m_ballForce = defaultBallOutcomeSettings3D();
    float m_ballTouchChargeSeconds = 0.6f;
    // Maos e pes (07/10): a pose dos dedos das maos (so o visual) e os dedos do
    // pe dobrando no chao (o pe gira na base deles no fim do apoio).
    RagdollHandPose3D m_handPose = RagdollHandPose3D::Automatic;
    bool m_toeRoll = true;
    float m_ballShotChargeSeconds = 1.0f;
    // Levantar a bola (o botao do meio): o tempo para encher a barra amarela.
    float m_ballRaiseChargeSeconds = 0.6f;
    double m_ballForceCopiedTime = -10.0;
    // Carregando o chute, a camera gira menos (09/10, pedido do usuario:
    // mirar enquanto ajusta a altura e o efeito, "sem exagero"): a fracao da
    // sensibilidade.
    float m_shotCameraSensitivity = 0.6f;
    // A sensibilidade do gesto nos efeitos (23/10, pedido do usuario; nao a da camera): o puxao vertical (altura,
    // giro para tras/por cima) e o horizontal (efeito de lado) valem esta fracao (1: como antes).
    float m_shotLiftSensitivity = 1.0f;
    float m_shotCurveSensitivity = 1.0f;
    // A altura e o efeito do gesto como o indicador mostra neste quadro (-1..1, com as sensibilidades): o chute
    // usa exatamente estes (23/10, pedido do usuario: soltando, o efeito pulava para mais forte que o indicador).
    float m_shotLiftShown = 0.0f;
    float m_shotCurveShown = 0.0f;
    float m_shotCameraSlow = 0.0f;
    // Quanto da ajuda da pelve fica: parado, com as pernas carregando (%), e
    // andando pela passada (correcao de posicao e postura, %). Pe consciente
    // (o pouso da passada corrigido pelo equilibrio).
    // (100% por padrao desde 06/10, pedido do usuario: no fisico a ajuda
    // comeca inteira e cai pelo impacto - ImpactAssist3D.)
    float m_e5StandingAssistPercent = 100.0f;
    // Enquanto a marcha hibrida e polida, o jogo abre na referencia estavel.
    // Reduzir este valor continua disponivel como diagnostico no painel, mas
    // 10% ainda nao sustenta a passada atual e nao pode ser apresentado como
    // experiencia padrao ao jogador.
    float m_e5WalkingAssistPercent = 100.0f;
    // Ajuda para levantar do chao (%): a subida na pelve e no peito e o torque
    // de endireitar do levantar. 100% e o levantar atual; 0%, so os musculos.
    float m_getUpAssistPercent = 100.0f;
    // Rigidez dos musculos ao levantar (%): 100% e a de antes (rigida);
    // menos, mais relaxado (a forca maxima fica).
    float m_getUpStiffnessPercent = 55.0f;
    // Forca dos musculos ao levantar (05/10 noite).
    float m_getUpStrengthPercent = 70.0f;
    // Levantar de bruços: 0 o do Kimodo (padrao do personagem), 1 o antigo
    // (stand_up_front) - para comparar.
    int m_getUpFrontClip = 0;
    // Levantar de costas: 0 o do Kimodo, 1 o antigo (stand_up_back).
    int m_getUpBackClip = 0;
    // Carrinho: correndo, Ctrl da o carrinho (desligado, Ctrl so agacha).
    bool m_tackleEnabled = true;
    bool m_e5FootPlacement = true;
    // Variacao de movimento (vida): cada boneco com o seu jeito.
    bool m_motionVariety = true;
    float m_motionVarietyPercent = 100.0f;
    // Pe com mola e tornozelo contra a queda (modo fisico).
    bool m_footSpring = true;
    bool m_ankleBalance = true;
    // Pulo com altura controlada: sai ao soltar o espaco; o tempo segurado
    // decide a altura. Carga do ultimo pulo (0..1) para o corpo, e o
    // agachar de preparacao enquanto segura.
    bool m_laboratoryJumpCharging = false;
    // O pulo instantaneo (24/10, pedido do usuario): sai ao apertar o espaco,
    // com a altura media; desligado, o de antes (carrega e sai ao soltar).
    bool m_instantJump = true;
    // O espaco apertado no modo instantaneo (o laboratorio pede no quadro).
    // (Um agachar curto antes de sair do chao, 80 a 100 ms, dava 2.000 a 3.800
    // m/s2 nas pernas: o agachar ainda descia na decolagem.)
    bool m_instantJumpPressed = false;
    // O espaco segurado agora (pressa para levantar).
    bool m_characterSpaceHeld = false;
    float m_laboratoryJumpChargeSeconds = 0.0f;
    // Manejo de bola (personagem controlado): mouse esquerdo segurado carrega
    // o chute, o direito o drible; soltar faz o pedido com a carga (0..1). A
    // barra (centro, embaixo) mostra a carga e fica um instante depois.
    bool m_ballShotHeld = false;
    bool m_ballDribbleHeld = false;
    bool m_ballRaiseHeld = false;
    float m_ballCharge = 0.0f;
    BallActionKind3D m_ballRequest = BallActionKind3D::None;
    // Estresse da bola no laboratorio (autostart laboratory-ball-stress, 04/10:
    // achar o crash ao chutar): corre em rumos aleatorios, repoe a bola perto e
    // pede toques e chutes sem parar.
    bool m_ballStress = false;
    float m_ballStressTime = 0.0f;
    std::uint32_t m_ballStressState = 2463534242u;
    float m_ballStressNextRequest = 1.5f;
    float m_ballStressNextPlace = 1.0f;
    float m_ballStressSegmentEnd = 0.0f;
    Vec3 m_ballStressMove;
    bool m_ballStressSprint = false;
    // (Ctrl curto no sprint: o carrinho.)
    float m_ballStressCrouchSeconds = 0.0f;
    float m_ballStressCrouchDelay = 0.0f;
    std::uint32_t m_ballStressRequests = 0;
    float m_ballRequestPower = 0.0f;
    float m_ballBarSeconds = 0.0f;
    // A barra de quem (vermelha o chute, azul o drible, amarela o levantar).
    BallActionKind3D m_ballBarKind = BallActionKind3D::Dribble;
    // As areas de alcance do jogador (o menu do jogador): o referencial da
    // escolha do gesto, como o atleta o devolveu no ultimo tick.
    bool m_showReachAreas = false;
    bool m_reachValid = false;
    Vec3 m_reachOrigin;
    float m_reachYaw = 0.0f;
    float m_reachPelvisHeight = 0.0f;
    // Gesto do chute: o puxao do mouse logo antes de soltar (pixels, esquece
    // em ~0,15 s). Para cima/baixo a altura, para os lados o efeito; a mira e
    // o olhar de antes do puxao (ele tambem gira a camera).
    Vec2 m_ballFlick;
    // Pixels do puxao -> -1..1 (abaixo de ~25 px e so mira; cheio em ~180).
    static float ballFlickShape(float pixels);
    float m_ballRequestLift = 0.0f;
    float m_ballRequestCurve = 0.0f;
    float m_ballRequestAimYaw = 0.0f;
    float m_laboratoryJumpCharge = 0.0f;
    float m_laboratoryJumpCrouch = 0.0f;
    Vec3 m_characterFollowVelocity;
    // Oculta apenas a apresentação em primeira pessoa; controles, física,
    // lanterna e manipulação continuam ativos para captura de imagens.
    // Oculto por padrão.
    bool m_hidePhysGunPresentation = true;
    // Piso das previas na GPU (o visualizador de toque na bola): uma mesh
    // real (quad + textura de xadrez assada), carregada uma vez sob demanda.
    GpuModel3D m_viewerFloor;
    bool m_viewerFloorLoaded = false;
    std::vector<SpawnedPropInstance> m_spawnedProps;
    std::unordered_map<std::uint32_t, std::size_t> m_spawnedPropByBodyIndex;
    std::vector<SpawnedInflatedBallInstance> m_inflatedBalls;
    // A bola congelada junto com o corpo (mouse 4); 0 sem nenhuma.
    std::uint64_t m_frozenSceneBallEntityId = 0;
    GpuModel3D m_soccerBallVisual;
    std::vector<Vec3> m_soccerBallCage;
    float m_soccerBallRadius = 0.1098f;
    // Propriedades das bolas novas e das existentes (painel Physgun).
    InflatedBallProfile3D m_soccerBallProfile;
    // Tamanho da bola de futebol (07/10, pedido do usuario): 0 a oficial
    // (22 cm), 1 com 28 cm (o topo um pouco acima do comeco da canela). O peso,
    // a pressao e o rolar acompanham (applySoccerBallSize); a pressao do menu
    // e a da bola oficial, a efetiva cai com o tamanho.
    // (25/10, pedido do usuario: 0,35 por padrao - 24,1 cm.)
    float m_soccerBallSize = 0.35f;
    // A bola escondida atras do proprio corpo (25/10, pedidos do usuario; painel
    // Physgun): o contorno so na parte coberta (senao o circulo inteiro, com o
    // centro dela coberto) e a opacidade (0,10, a que ele gostou); e o ponto no
    // centro, sempre visivel quando ligado (bem pequeno de perto, cresce com a
    // distancia, no maximo o tamanho dela na tela), com a opacidade dele.
    bool m_hiddenBallCoveredOnly = true;
    float m_hiddenBallOpacity = 0.10f;
    bool m_ballDot = false;
    float m_ballDotOpacity = 0.5f;
    float m_soccerBallBaseGaugeBar = 0.8f;
    bool m_soccerBallVisualDirty = false;
    InflatedBallHandle3D m_physGunGrabbedBall;
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
    // Biblioteca preenchida no início com clipes canônicos já retargeteados.
    std::vector<AnimationClip3D> m_animationClips;
    // Recomputed only when the validated catalog/profile changes, not in UI frames.
    bool m_animationLocomotionCompatible = false;

    // Inspecao de jogador (07/10): a tela e a textura da imagem dela.
    std::unique_ptr<PlayerInspectorScreen> m_playerInspector;
    std::unique_ptr<UiCanvasTexture> m_playerInspectorCanvas;
    // Objetos procedurais (09/10): a tela e a textura da imagem dela.
    std::unique_ptr<ObjectInspectorScreen> m_objectInspector;
    std::unique_ptr<UiCanvasTexture> m_objectInspectorCanvas;
    // Aprendizado por Reforco (12/10): a tela e a textura do visualizador.
    std::unique_ptr<ReinforcementLearningScreen> m_reinforcementLearning;
    std::unique_ptr<UiCanvasTexture> m_reinforcementLearningCanvas;

    WorldAudioController m_worldAudio;

    CharacterMotorSettings3D m_characterSettings;
    // Velocidade de cada ciclo do personagem controlado (lidas dos clipes).
    CharacterGaitSpeeds3D m_avatarGaitSpeeds;
    // A capsula do jogador: a do atleta (athleteAvatarMotorSettings3D, a do
    // corpo em applyLaboratoryBody); o andar e o sprint saem dos clipes em
    // loadAnimationCatalog.
    CharacterMotorSettings3D m_avatarCharacterSettings = athleteAvatarMotorSettings3D();
    std::vector<LaboratoryMapPart> m_laboratoryMapParts;
    std::vector<PhysicsBodyHandle3D> m_laboratoryMapBodies;
    LaboratoryGoal m_laboratoryGoal;
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
