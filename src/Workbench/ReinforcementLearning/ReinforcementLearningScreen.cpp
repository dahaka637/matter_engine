#include "Workbench/ReinforcementLearning/ReinforcementLearningScreen.hpp"

#include "Engine/Modeling/Inspection/InspectionRenderer.hpp"
#include "Engine/Modeling/Raster/ImageIO.hpp"
#include "Engine/Modeling/Skeleton/Skeleton.hpp"
#include "Engine/UI/FontAwesome.hpp"
#include "Workbench/UiKit.hpp"
#include "imgui.h"

#include <algorithm>
#include <array>
#include <cfloat>
#include <chrono>
#include <cmath>
#include <ctime>
#include <utility>

namespace MatterEngine::Workbench {

using namespace Modeling;

namespace {

constexpr ImVec4 MutedText { 0.47f, 0.54f, 0.60f, 1.0f };
constexpr ImVec4 InfoText { 0.62f, 0.72f, 0.80f, 1.0f };
constexpr ImVec4 ValueText { 0.84f, 0.89f, 0.94f, 1.0f };
constexpr ImVec4 AccentText { 0.38f, 0.69f, 1.0f, 1.0f };
constexpr ImVec4 WarningText { 1.0f, 0.62f, 0.42f, 1.0f };
constexpr ImVec4 SuccessText { 0.36f, 0.85f, 0.55f, 1.0f };
constexpr ImVec4 ErrorText { 1.0f, 0.38f, 0.45f, 1.0f };
// Um passo da politica no visualizador; mais que isso por quadro so atrasa.
constexpr int MaximumStepsPerFrame = 4;

struct StateLabel {
    const char* key;
    const char* label;
    ImVec4 color;
};

constexpr std::array<StateLabel, 9> StateLabels { {
    { "treinando", "Treinando", SuccessText },
    { "pausado", "Pausado", WarningText },
    { "avaliando", "Avaliando", AccentText },
    { "salvando", "Salvando", AccentText },
    { "abrindo", "Abrindo", AccentText },
    { "parando", "Parando", WarningText },
    { "parado", "Parado", MutedText },
    { "novo", "Sem treino", MutedText },
    { "erro", "Erro", ErrorText },
} };

StateLabel stateLabel(const std::string& state) {
    for (const StateLabel& label : StateLabels)
        if (state == label.key) return label;
    return { "", state.empty() ? "Sem treino" : state.c_str(), MutedText };
}

struct ModeButton {
    const char* key;
    const char* label;
    const char* tooltip;
};

constexpr std::array<ModeButton, 3> ModeButtons { {
    { "maximo", "Máximo", "10 threads de física com prioridade baixa: o treino mais rápido, e o PC continua respondendo." },
    { "equilibrado", "Equilibrado", "8 threads com prioridade ociosa: o treino só usa o que o resto do PC deixa." },
    { "leve", "Leve", "4 threads com prioridade ociosa: para usar o PC (jogos, por exemplo) com o treino ligado." },
} };

std::string duration(double seconds) {
    const long total = static_cast<long>(std::max(0.0, seconds));
    const long hours = total / 3600, minutes = (total % 3600) / 60;
    char text[48];
    if (hours > 0) std::snprintf(text, sizeof text, "%ld h %02ld min", hours, minutes);
    else std::snprintf(text, sizeof text, "%ld min", minutes);
    return text;
}

std::string decimal(double value, int digits) {
    char text[48];
    std::snprintf(text, sizeof text, "%.*f", digits, value);
    std::string result = text;
    std::replace(result.begin(), result.end(), '.', ',');
    return result;
}

std::string timestamp() {
    const std::time_t now = std::time(nullptr);
    char text[32];
    std::strftime(text, sizeof text, "%Y%m%d_%H%M%S", std::localtime(&now));
    return text;
}

void tooltip(const char* text) {
    if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled)) ImGui::SetTooltip("%s", text);
}

void infoRow(const char* label, const std::string& value) {
    ImGui::TextColored(MutedText, "%s", label);
    ImGui::SameLine(ImGui::GetContentRegionAvail().x * 0.42f);
    ImGui::TextColored(ValueText, "%s", value.c_str());
}

Highlight cross(Vec3 at, float radius, Color color) {
    Highlight marker;
    marker.color = color;
    marker.segments.push_back({ at - Vec3 { radius, 0, 0 }, at + Vec3 { radius, 0, 0 } });
    marker.segments.push_back({ at - Vec3 { 0, radius, 0 }, at + Vec3 { 0, radius, 0 } });
    marker.segments.push_back({ at - Vec3 { 0, 0, radius }, at + Vec3 { 0, 0, radius } });
    return marker;
}

} // namespace

ReinforcementLearningScreen::ReinforcementLearningScreen(Options options, std::function<void()> onBack)
    : m_options(std::move(options)), m_onBack(std::move(onBack)) {
    m_link = std::make_unique<TrainerLink3D>(defaultTrainerPaths3D(m_options.sourceRoot), m_options.run);
    m_pending = std::async(std::launch::async, [] {
        ModelOutcome outcome;
        outcome.loaded = loadPlayerModel(defaultPlayerModelManifest(), {}, &outcome.error);
        return outcome;
    });
    frame(35.0f, 10.0f);
    refreshHistory();
}

ReinforcementLearningScreen::~ReinforcementLearningScreen() = default;

TrainerStatus3D ReinforcementLearningScreen::status() const { return m_link->status(); }

std::filesystem::path ReinforcementLearningScreen::taskPath() const {
    // A tarefa do treino (com os pesos de agora) ou a do repositorio.
    const std::filesystem::path live = m_link->paths().run(m_options.run) / "tarefa.json";
    if (std::filesystem::exists(live)) return live;
    return m_options.sourceRoot / "assets" / "rl" / "tasks" / (m_options.task + ".json");
}

std::filesystem::path ReinforcementLearningScreen::policyPath() const {
    return m_link->paths().run(m_options.run) / "exports"
        / (m_useBest ? "policy_best.mrlpolicy" : "policy_latest.mrlpolicy");
}

void ReinforcementLearningScreen::notify(std::string text) {
    m_notice = std::move(text);
    m_noticeSeconds = 4.0f;
}

void ReinforcementLearningScreen::frame(float yawDegrees, float pitchDegrees) {
    m_camera.yaw = yawDegrees * DegreesToRadians;
    m_camera.pitch = pitchDegrees * DegreesToRadians;
    m_camera.distance = 4.4f;
    m_dirty = true;
}

void ReinforcementLearningScreen::ensureViewer() {
    if (m_viewer || !m_viewerError.empty()) return;
    try {
        TrainingBatchSettings3D settings;
        settings.taskPath = taskPath();
        settings.environmentCount = 1;
        settings.environmentsPerScene = 1;
        settings.workerThreads = 1;
        settings.seed = 7;
        settings.evaluation = true;
        settings.evaluationEpisodeSeconds = 600.0f;
        // O visualizador mostra a fisica do jogo.
        settings.solverPositionIterations = 20;
        settings.solverVelocityIterations = 20;
        m_viewer = std::make_unique<TrainingBatch3D>(settings);
        m_action.assign(m_viewer->actionSize(), 0.0f);
        m_colliders = buildRagdollColliderWireframe3D(m_viewer->body().profile);
        reloadPolicy(true);
    } catch (const std::exception& error) {
        m_viewerError = error.what();
    }
}

void ReinforcementLearningScreen::reloadPolicy(bool force) {
    if (!m_viewer) return;
    const std::filesystem::path path = policyPath();
    std::error_code code;
    if (!std::filesystem::exists(path, code)) {
        m_policy.reset();
        m_policyText = "sem política ainda";
        return;
    }
    const auto write = std::filesystem::last_write_time(path, code);
    if (!force && write == m_policyWrite) return;
    m_policyWrite = write;
    try {
        auto policy = std::make_unique<PolicyRuntime3D>(path);
        if (policy->observationSize() != m_viewer->observationSize() || policy->actionSize() != m_viewer->actionSize()) {
            m_policy.reset();
            m_policyText = "política de outro corpo";
            return;
        }
        m_policy = std::move(policy);
        m_policyText = "política da rodada " + std::to_string(m_policy->info().iteration);
    } catch (const std::exception& error) {
        m_policy.reset();
        m_policyText = error.what();
    }
}

void ReinforcementLearningScreen::stepOnce() {
    const std::span<const float> observation = m_viewer->observations();
    if (m_policy) m_policy->evaluate(observation, m_action);
    else std::fill(m_action.begin(), m_action.end(), 0.0f);
    m_viewer->step(m_action);
}

void ReinforcementLearningScreen::stepViewer(float deltaSeconds) {
    if (!m_viewer || m_paused) return;
    const float step = m_viewer->policyStepSeconds();
    m_accumulator += deltaSeconds;
    int steps = 0;
    while (m_accumulator >= step && steps < MaximumStepsPerFrame) {
        stepOnce();
        m_accumulator -= step;
        ++steps;
    }
    if (m_accumulator > step * MaximumStepsPerFrame) m_accumulator = 0.0f;
    if (steps > 0) m_dirty = true;
}

void ReinforcementLearningScreen::advanceViewer(int decisions) {
    ensureViewer();
    if (!m_viewer) return;
    for (int i = 0; i < decisions; ++i) stepOnce();
    m_dirty = true;
}

void ReinforcementLearningScreen::push(int direction, float newtons) {
    if (!m_viewer) return;
    const TrainingEnvironmentView3D view = m_viewer->view(0);
    if (!view.state) return;
    const RagdollProfile3D& profile = m_viewer->body().profile;
    const Quaternion frame = view.state->links.front().orientation * profile.links.front().modelOrientation.conjugate();
    Vec3 forward = frame.rotate({ 1.0f, 0.0f, 0.0f });
    forward = Vec3 { forward.x, forward.y, 0.0f }.normalized();
    const Vec3 left { -forward.y, forward.x, 0.0f };
    const std::array<Vec3, 4> directions { forward, forward * -1.0f, left, left * -1.0f };
    const Vec3 force = directions[static_cast<std::size_t>(std::clamp(direction, 0, 3))] * newtons;
    m_viewer->push(0, m_viewer->body().upperChest, force, 0.2f);
}

void ReinforcementLearningScreen::pollModel() {
    if (!m_pending || m_pending->wait_for(std::chrono::seconds(0)) != std::future_status::ready) return;
    ModelOutcome outcome = m_pending->get();
    m_pending.reset();
    if (!outcome.loaded) {
        m_modelError = outcome.error.empty() ? "o jogador nao carregou" : outcome.error;
        return;
    }
    m_model = std::move(outcome.loaded);
    m_skeleton = std::make_unique<RagdollSkeletonView>(m_model->character, m_model->rig);
    m_dirty = true;
}

void ReinforcementLearningScreen::waitUntilLoaded() {
    if (m_pending) m_pending->wait();
    pollModel();
    ensureViewer();
    if (m_historyPending) {
        m_historyPending->wait();
        refreshHistory();
    }
}

void ReinforcementLearningScreen::refreshHistory() {
    if (m_historyPending) {
        if (m_historyPending->wait_for(std::chrono::seconds(0)) != std::future_status::ready) return;
        m_history = m_historyPending->get();
        m_historyPending.reset();
        return;
    }
    m_historyPending = std::async(std::launch::async,
        [database = m_link->paths().database, run = m_options.run] { return readTrainingHistory3D(database, run, 0.0, 400); });
}

void ReinforcementLearningScreen::update(float deltaSeconds) {
    pollModel();
    ensureViewer();
    stepViewer(deltaSeconds);
    if ((m_policyWatch -= deltaSeconds) <= 0.0f) {
        m_policyWatch = 2.0f;
        reloadPolicy(false);
    }
    // (A leitura pedida chega no quadro seguinte em que ficar pronta.)
    if (m_historyPending || (m_historyWatch -= deltaSeconds) <= 0.0f) {
        if (!m_historyPending) m_historyWatch = 5.0f;
        refreshHistory();
    }
    if (m_noticeSeconds > 0.0f) m_noticeSeconds -= deltaSeconds;
}

bool ReinforcementLearningScreen::render(int width, int height) {
    if (!m_viewer || !m_model || !m_skeleton) return false;
    if (!m_dirty && !m_image.empty() && m_image.width() == width && m_image.height() == height) return false;
    const TrainingEnvironmentView3D view = m_viewer->view(0);
    if (!view.state) return false;
    const RagdollState3D& state = *view.state;
    const RagdollProfile3D& profile = m_viewer->body().profile;
    std::vector<Vec3> positions(state.links.size());
    std::vector<Quaternion> orientations(state.links.size());
    for (std::size_t i = 0; i < state.links.size(); ++i) {
        positions[i] = state.links[i].position;
        orientations[i] = state.links[i].orientation;
    }
    const Vec3 anchor = positions.front();
    const SkeletonPose& pose = m_skeleton->update(positions, orientations, anchor, 0.0f, m_viewer->policyStepSeconds());
    const InspectionModel& model = *m_model->model;
    std::vector<Vec3> posed;
    if (model.mesh.hasSkin()) posed = model.mesh.posedPositions(skinningTransforms(model.skeleton, pose));

    std::vector<Highlight> highlights;
    if (m_showColliders && m_colliders) {
        const RagdollAnimationPose3D& links = m_skeleton->gameLinks();
        const RagdollColliderWireframe3D wireframe =
            poseRagdollColliderWireframe3D(profile, *m_colliders, links.linkPositions, links.linkOrientations, false);
        std::array<Highlight, 3> sides;
        sides[0].color = Palette::Center;
        sides[1].color = Palette::Left;
        sides[2].color = Palette::Right;
        for (const RagdollColliderWireframeLink3D& link : wireframe.links) {
            const std::size_t side = link.side == RagdollColliderWireframeSide3D::Left ? 1
                : link.side == RagdollColliderWireframeSide3D::Right ? 2 : 0;
            for (const auto& [a, b] : link.segments)
                sides[side].segments.push_back({ m_skeleton->gameToSkeleton(a), m_skeleton->gameToSkeleton(b) });
        }
        for (Highlight& side : sides) highlights.push_back(std::move(side));
    }
    if (m_showCenterOfMass) {
        Vec3 center;
        for (std::size_t i = 0; i < state.links.size(); ++i) center += positions[i] * profile.links[i].massFraction;
        Highlight com = cross(m_skeleton->toSkeleton(center), 0.05f, Palette::Focus);
        // A projecao no chao: onde o peso cai em relacao aos pes.
        com.segments.push_back({ m_skeleton->toSkeleton(center), m_skeleton->toSkeleton({ center.x, center.y, 0.0f }) });
        highlights.push_back(std::move(com));
        highlights.push_back(cross(m_skeleton->toSkeleton({ center.x, center.y, 0.0f }), 0.04f, Palette::Focus));
    }
    if (m_showContacts)
        for (const RagdollContactPoint3D& contact : state.contacts)
            highlights.push_back(cross(m_skeleton->toSkeleton(contact.position), 0.025f, Palette::Error));
    if (view.pushLink >= 0) {
        const Vec3 from = positions[static_cast<std::size_t>(view.pushLink)];
        Highlight arrow;
        arrow.color = Palette::Warning;
        arrow.segments.push_back({ m_skeleton->toSkeleton(from - view.pushForce * 0.002f), m_skeleton->toSkeleton(from) });
        highlights.push_back(std::move(arrow));
    }

    InspectionScene scene;
    scene.skeleton = &model.skeleton;
    scene.mesh = &model.mesh;
    scene.regions = &model.regions;
    scene.jointRegion = model.jointRegion;
    scene.pose = &pose;
    scene.meshPositions = posed;
    scene.highlights = highlights;
    InspectionStyle style;
    style.surface = m_showMesh ? SurfaceMode::Solid : SurfaceMode::Hidden;
    style.skeleton = !m_showMesh;
    style.joints = !m_showMesh;
    style.hud = false;
    style.gizmo = false;
    style.scaleBar = false;
    style.gridExtent = 4.0f;
    style.supersample = 1;
    OrbitCamera camera = m_camera;
    const Vec3 pelvis = m_skeleton->toSkeleton(anchor);
    camera.target = { pelvis.x, pelvis.y, pelvis.z - 0.04f };
    m_image = renderInspection(scene, style, camera, width, height);
    m_dirty = false;
    return true;
}

void ReinforcementLearningScreen::draw(CanvasTarget& canvas, float uiScale) {
    m_uiScale = uiScale;
    beginFullscreenPanel("MatterEngineReinforcementLearning");
    const ImGuiIO& io = ImGui::GetIO();
    ImDrawList* drawList = ImGui::GetWindowDrawList();
    drawWorkbenchBackdrop(drawList, io.DisplaySize, uiScale);
    const TrainerStatus3D status = m_link->status();
    drawWorkbenchHeader(drawList, io.DisplaySize, uiScale, "APRENDIZADO POR REFORÇO", stateLabel(status.state).label);

    const float margin = ui(12.0f);
    const float top = headerHeight(uiScale) + margin;
    const float height = io.DisplaySize.y - top - margin;
    const float sidebar = std::max(sidebarWidth(uiScale), ui(320.0f));
    ImGui::SetCursorPos({ margin, top });
    ImGui::BeginChild("RLSidebar", { sidebar, height }, true, ImGuiWindowFlags_NoScrollbar);
    drawSidebar(height);
    ImGui::EndChild();

    const float left = margin * 2.0f + sidebar;
    ImGui::SetCursorPos({ left, top });
    ImGui::BeginChild("RLWorkspace", { io.DisplaySize.x - left - margin, height }, true, ImGuiWindowFlags_NoScrollbar);
    panelHeader("VISUALIZADOR");
    drawToolbar();
    const float charts = ui(150.0f);
    drawViewport(canvas, charts + ui(78.0f));
    drawPushes();
    drawCharts(charts);
    drawFooter();
    ImGui::EndChild();
    ImGui::End();
}

void ReinforcementLearningScreen::drawSidebar(float height) {
    drawTrainer();
    drawSkills();
    drawEvaluation(m_link->status());
    const std::string back = UI::FontAwesome::label(UI::FontAwesome::ArrowLeft, "Menu principal");
    ImGui::SetCursorPosY(std::max(ImGui::GetCursorPosY(), height - ui(58.0f)));
    if (navigationButton(back.c_str(), false, { -1.0f, ui(40.0f) }) && m_onBack) m_onBack();
}

void ReinforcementLearningScreen::drawTrainer() {
    const TrainerStatus3D status = m_link->status();
    panelHeader("TREINO");
    ImGui::Dummy({ 1.0f, ui(2.0f) });
    const StateLabel state = stateLabel(status.state);
    ImGui::TextColored(state.color, "%s", state.label);
    if (status.alive && !status.device.empty()) {
        ImGui::SameLine();
        ImGui::TextColored(MutedText, "· %s", status.device == "cuda" ? "GPU" : "CPU");
    }
    ImGui::Dummy({ 1.0f, ui(2.0f) });
    if (status.known) {
        infoRow("Etapa", status.taskName.empty() ? status.task : status.taskName);
        infoRow("Rodada", std::to_string(status.iteration));
        infoRow("Decisões", decimal(static_cast<double>(status.steps) / 1e6, 1) + " M");
        if (status.alive) infoRow("Velocidade", decimal(status.sps, 0) + " /s");
        infoRow("Treinado", duration(status.trainSeconds));
        if (status.alive && status.lastFallFraction >= 0.0)
            infoRow("Episódio", decimal(status.lastEpisodeSeconds, 1) + " s");
        if (status.alive && status.cpuTemperature > 0.0) infoRow("CPU", decimal(status.cpuTemperature, 0) + " °C");
    }
    if (!status.message.empty()) ImGui::TextColored(WarningText, "%s", status.message.c_str());
    ImGui::Dummy({ 1.0f, ui(4.0f) });
    drawControls(status);
    drawModes(status);
}

void ReinforcementLearningScreen::drawControls(const TrainerStatus3D& status) {
    const float spacing = ImGui::GetStyle().ItemSpacing.x;
    const float half = (ImGui::GetContentRegionAvail().x - spacing) * 0.5f;
    const bool alive = status.alive;
    const bool opening = status.state == "abrindo";
    if (!alive) {
        ImGui::BeginDisabled(opening);
        const std::string label = UI::FontAwesome::label(UI::FontAwesome::Play,
            status.known ? "Continuar treino" : "Iniciar treino");
        if (ImGui::Button(label.c_str(), { -1.0f, ui(36.0f) })) {
            std::string error;
            if (!m_link->start(m_options.task, m_mode, &error)) notify(error);
        }
        ImGui::EndDisabled();
        tooltip("Abre o treino em segundo plano, do último checkpoint. Ele continua com o Matter Engine fechado.");
        return;
    }
    const bool paused = status.state == "pausado";
    const std::string toggle = paused ? UI::FontAwesome::label(UI::FontAwesome::Play, "Retomar")
                                      : UI::FontAwesome::label(UI::FontAwesome::Pause, "Pausar");
    if (ImGui::Button(toggle.c_str(), { half, ui(34.0f) })) m_link->send(paused ? "retomar" : "pausar");
    tooltip(paused ? "Continua de onde parou." : "Para a física e salva um checkpoint; o treinador fica aberto.");
    ImGui::SameLine();
    if (ImGui::Button(UI::FontAwesome::label(UI::FontAwesome::Stop, "Parar").c_str(), { half, ui(34.0f) }))
        m_link->send("parar");
    tooltip("Salva e fecha o treinador. \"Continuar treino\" volta do mesmo ponto.");
    if (ImGui::Button(UI::FontAwesome::label(UI::FontAwesome::FloppyDisk, "Salvar").c_str(), { half, ui(30.0f) })) {
        m_link->send("salvar");
        notify("Checkpoint pedido");
    }
    tooltip("Grava um checkpoint agora (o treino também salva sozinho a cada 10 min).");
    ImGui::SameLine();
    if (ImGui::Button(UI::FontAwesome::label(UI::FontAwesome::Check, "Avaliar").c_str(), { half, ui(30.0f) })) {
        m_link->send("avaliar");
        notify("Avaliação pedida");
    }
    tooltip("Roda a bateria de avaliação agora (ela também roda sozinha a cada 10 min).");
}

void ReinforcementLearningScreen::drawModes(const TrainerStatus3D& status) {
    ImGui::Dummy({ 1.0f, ui(4.0f) });
    sectionLabel("DESEMPENHO");
    const std::string current = status.alive && !status.mode.empty() ? status.mode : m_mode;
    const float spacing = ImGui::GetStyle().ItemSpacing.x;
    const float third = (ImGui::GetContentRegionAvail().x - 2.0f * spacing) / 3.0f;
    for (std::size_t i = 0; i < ModeButtons.size(); ++i) {
        const ModeButton& mode = ModeButtons[i];
        if (i > 0) ImGui::SameLine();
        if (tabButton(mode.label, current == mode.key, { third, ui(30.0f) }) && current != mode.key) {
            m_mode = mode.key;
            if (status.alive) {
                m_link->send("modo", std::string("{\"modo\":\"") + mode.key + "\"}");
                notify(std::string("Modo ") + mode.label + ": o treinador reabre em alguns segundos");
            }
        }
        tooltip(mode.tooltip);
    }
    ImGui::Dummy({ 1.0f, ui(2.0f) });
    bool boot = m_link->resumesAtBoot();
    if (ImGui::Checkbox("Retomar ao ligar o PC", &boot)) {
        std::string error;
        if (!m_link->runLauncher({ "servico", boot ? "instalar" : "remover", "--run", m_options.run }, &error))
            notify(error);
        else notify(boot ? "O treino volta sozinho ao ligar o PC" : "O treino não volta mais sozinho");
    }
    tooltip("Se o PC desligar com o treino aberto, ele volta sozinho no próximo login. Um \"Parar\" seu não volta.");
}

void ReinforcementLearningScreen::drawSkills() {
    ImGui::Dummy({ 1.0f, ui(8.0f) });
    panelHeader("HABILIDADES");
    ImGui::Dummy({ 1.0f, ui(2.0f) });
    const std::string label = UI::FontAwesome::label(UI::FontAwesome::Brain, "Ficar em pé");
    navigationButton(label.c_str(), true, { -1.0f, ui(32.0f) });
    tooltip("H1: ficar em pé, estável, sem ajuda nenhuma (só os músculos das juntas). Pronto quando fica de pé "
            "60 s em todas as sementes da bateria, sem escorregar nem tremer.");
}

void ReinforcementLearningScreen::drawEvaluation(const TrainerStatus3D& status) {
    ImGui::Dummy({ 1.0f, ui(8.0f) });
    panelHeader("AVALIAÇÃO");
    ImGui::Dummy({ 1.0f, ui(2.0f) });
    if (!status.evaluated) {
        ImGui::TextColored(MutedText, "—");
        return;
    }
    for (const auto& [solver, stand] : status.evaluationStand) {
        const std::string name = solver == "jogo" ? "De pé (jogo)" : "De pé (treino)";
        infoRow(name.c_str(), decimal(stand * 100.0, 0) + "%");
    }
    tooltip("Quantos dos 32 mundos da bateria ficaram de pé os 60 s, sem ruído e sem variação do mundo. "
            "\"Jogo\" é a mesma bateria com a física do jogo (20/20).");
    infoRow("Nota", decimal(status.evaluationScore, 2) + " (rodada " + std::to_string(status.evaluationIteration) + ")");
    if (status.bestIteration >= 0)
        infoRow("Melhor", decimal(status.bestScore, 2) + " (rodada " + std::to_string(status.bestIteration) + ")");
}

void ReinforcementLearningScreen::drawToolbar() {
    struct View {
        const char* label;
        float yaw;
        float pitch;
    };
    constexpr std::array<View, 3> Views { { { "Frente", 0.0f, 6.0f }, { "Lado", 90.0f, 6.0f }, { "3/4", 35.0f, 10.0f } } };
    for (const View& view : Views) {
        if (ImGui::Button(view.label, { ui(70.0f), ui(30.0f) })) frame(view.yaw, view.pitch);
        ImGui::SameLine();
    }
    if (ImGui::Checkbox("Malha", &m_showMesh)) m_dirty = true;
    ImGui::SameLine();
    if (ImGui::Checkbox("Colisores", &m_showColliders)) m_dirty = true;
    ImGui::SameLine();
    if (ImGui::Checkbox("Centro de massa", &m_showCenterOfMass)) m_dirty = true;
    tooltip("O centro de massa (amarelo) e a projeção dele no chão: dentro dos pés, ele está equilibrado.");
    ImGui::SameLine();
    if (ImGui::Checkbox("Contatos", &m_showContacts)) m_dirty = true;
    ImGui::SameLine();
    ImGui::SetNextItemWidth(ui(150.0f));
    const char* source = m_useBest ? "Política melhor" : "Política recente";
    if (ImGui::BeginCombo("##RLPolicySource", source)) {
        if (ImGui::Selectable("Política recente", !m_useBest)) {
            m_useBest = false;
            reloadPolicy(true);
        }
        if (ImGui::Selectable("Política melhor", m_useBest)) {
            m_useBest = true;
            reloadPolicy(true);
        }
        ImGui::EndCombo();
    }
    tooltip("Recente: o último checkpoint do treino (recarrega sozinha). Melhor: a de maior nota na avaliação.");
    ImGui::SameLine();
    const std::string pause = m_paused ? UI::FontAwesome::label(UI::FontAwesome::Play, "Continuar")
                                       : UI::FontAwesome::label(UI::FontAwesome::Pause, "Congelar");
    if (ImGui::Button(pause.c_str(), { ui(110.0f), ui(30.0f) })) m_paused = !m_paused;
    tooltip("Congela só o visualizador (o treino segue).");
    ImGui::SameLine();
    if (ImGui::Button(UI::FontAwesome::label(UI::FontAwesome::RotateLeft, "Recomeçar").c_str(), { ui(120.0f), ui(30.0f) })
        && m_viewer) {
        m_viewer->reset(static_cast<std::uint64_t>(std::time(nullptr)));
        m_dirty = true;
    }
    ImGui::SameLine();
    if (ImGui::Button(UI::FontAwesome::label(UI::FontAwesome::Camera, "Imagem").c_str(), { ui(100.0f), ui(30.0f) }))
        saveCapture();
    ImGui::Dummy({ 1.0f, ui(2.0f) });
}

void ReinforcementLearningScreen::drawViewport(CanvasTarget& canvas, float bottomHeight) {
    const ImGuiIO& io = ImGui::GetIO();
    const ImVec2 available = ImGui::GetContentRegionAvail();
    const ImVec2 size { std::max(ui(160.0f), available.x), std::max(ui(160.0f), available.y - bottomHeight) };
    const ImVec2 origin = ImGui::GetCursorScreenPos();
    ImGui::InvisibleButton("##RLViewport", size, ImGuiButtonFlags_MouseButtonLeft | ImGuiButtonFlags_MouseButtonRight);
    const bool hovered = ImGui::IsItemHovered();
    const bool active = ImGui::IsItemActive();
    ImDrawList* drawList = ImGui::GetWindowDrawList();
    const ImVec2 end { origin.x + size.x, origin.y + size.y };
    drawList->AddRectFilled(origin, end, IM_COL32(6, 8, 11, 255), ui(4.0f));
    const float scaleX = io.DisplayFramebufferScale.x > 0.0f ? io.DisplayFramebufferScale.x : 1.0f;
    const float scaleY = io.DisplayFramebufferScale.y > 0.0f ? io.DisplayFramebufferScale.y : 1.0f;
    const int width = std::max(16, static_cast<int>(std::lround(size.x * scaleX)));
    const int height = std::max(16, static_cast<int>(std::lround(size.y * scaleY)));
    const char* waiting = !m_viewerError.empty() ? m_viewerError.c_str()
        : !m_modelError.empty()                  ? m_modelError.c_str()
        : !m_model || !m_viewer                  ? "CARREGANDO"
                                                 : nullptr;
    if (waiting != nullptr) {
        const ImVec2 textSize = ImGui::CalcTextSize(waiting);
        drawList->AddText({ origin.x + (size.x - textSize.x) * 0.5f, origin.y + (size.y - textSize.y) * 0.5f },
            m_viewerError.empty() && m_modelError.empty() ? UiMuted : IM_COL32(255, 120, 120, 255), waiting);
        return;
    }
    if (active && (io.MouseDelta.x != 0.0f || io.MouseDelta.y != 0.0f) && ImGui::IsMouseDragging(ImGuiMouseButton_Left, 2.0f)) {
        m_camera.yaw -= io.MouseDelta.x * 0.008f;
        m_camera.pitch = std::clamp(m_camera.pitch + io.MouseDelta.y * 0.006f, -0.2f, 1.45f);
        m_dirty = true;
    }
    if (hovered && std::abs(io.MouseWheel) > 0.0f) {
        m_camera.distance = std::clamp(m_camera.distance * std::pow(0.88f, io.MouseWheel), 1.2f, 14.0f);
        m_dirty = true;
    }
    if (hovered && ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left)) frame(35.0f, 10.0f);
    const bool rendered = render(width, height);
    const std::uint64_t id = canvas.present(m_image, rendered);
    drawList->AddImage(static_cast<ImTextureID>(id), origin, end);
    // A politica que esta no corpo agora (canto de cima).
    const std::string label = m_policyText;
    drawList->AddText({ origin.x + ui(10.0f), origin.y + ui(8.0f) }, UiMuted, label.c_str());
}

void ReinforcementLearningScreen::drawPushes() {
    ImGui::Dummy({ 1.0f, ui(2.0f) });
    ImGui::AlignTextToFramePadding();
    ImGui::TextColored(InfoText, "%s", UI::FontAwesome::label(UI::FontAwesome::Hand, "Empurrão").c_str());
    tooltip("Um empurrão no peito do jogador do visualizador, por 0,2 s, para testar a política (o treino não muda).");
    ImGui::SameLine();
    ImGui::SetNextItemWidth(ui(180.0f));
    ImGui::SliderFloat("##RLPushForce", &m_pushNewtons, 50.0f, 800.0f, "%.0f N");
    const std::array<const char*, 4> labels { "Frente", "Trás", "Esquerda", "Direita" };
    for (int i = 0; i < 4; ++i) {
        ImGui::SameLine();
        ImGui::BeginDisabled(!m_viewer);
        if (ImGui::Button(labels[static_cast<std::size_t>(i)], { ui(84.0f), ui(28.0f) })) push(i, m_pushNewtons);
        ImGui::EndDisabled();
    }
}

void ReinforcementLearningScreen::drawCharts(float height) {
    ImGui::Dummy({ 1.0f, ui(4.0f) });
    std::vector<float> stand, seconds, reward, speed;
    for (const TrainingEvaluationPoint3D& point : m_history.evaluations)
        if (point.solver == "jogo") stand.push_back(static_cast<float>(point.standFraction * 100.0));
    for (const TrainingIterationPoint3D& point : m_history.iterations) {
        seconds.push_back(static_cast<float>(point.episodeSeconds));
        reward.push_back(static_cast<float>(point.reward));
        speed.push_back(static_cast<float>(point.sps));
    }
    struct Chart {
        const char* title;
        const std::vector<float>* values;
        float minimum;
        float maximum;
        const char* tooltip;
    };
    const auto top = [](const std::vector<float>& values, float floor) {
        float result = floor;
        for (const float v : values) result = std::max(result, v);
        return result * 1.05f;
    };
    const std::array<Chart, 4> charts { {
        { "De pé 60 s (%)", &stand, 0.0f, 100.0f, "A bateria de avaliação com a física do jogo, a cada 10 min." },
        { "Episódio (s)", &seconds, 0.0f, top(seconds, 1.0f), "Quanto dura um episódio do treino (o máximo é 20 s)." },
        { "Recompensa / passo", &reward, FLT_MAX, FLT_MAX, "A recompensa média por decisão em cada rodada do treino." },
        { "Decisões / s", &speed, 0.0f, top(speed, 1000.0f), "A velocidade do treino (física + rede)." },
    } };
    const float spacing = ImGui::GetStyle().ItemSpacing.x;
    const float width = (ImGui::GetContentRegionAvail().x - 3.0f * spacing) / 4.0f;
    const float rowTop = ImGui::GetCursorPosY();
    for (std::size_t i = 0; i < charts.size(); ++i) {
        if (i > 0) ImGui::SameLine();
        ImGui::SetCursorPosY(rowTop);
        const Chart& chart = charts[i];
        ImGui::BeginGroup();
        ImGui::TextColored(MutedText, "%s", chart.title);
        char overlay[48] = "";
        if (!chart.values->empty()) std::snprintf(overlay, sizeof overlay, "%.2f", chart.values->back());
        ImGui::PushID(static_cast<int>(i));
        ImGui::PlotLines("##chart", chart.values->empty() ? nullptr : chart.values->data(),
            static_cast<int>(chart.values->size()), 0, overlay, chart.minimum, chart.maximum,
            { width, height - ImGui::GetTextLineHeightWithSpacing() });
        ImGui::PopID();
        ImGui::EndGroup();
        tooltip(chart.tooltip);
    }
}

void ReinforcementLearningScreen::drawFooter() {
    if (m_noticeSeconds > 0.0f && !m_notice.empty()) {
        ImGui::TextColored(AccentText, "%s", m_notice.c_str());
        return;
    }
    if (!m_history.events.empty()) {
        const TrainingEventPoint3D& event = m_history.events.front();
        const std::time_t when = static_cast<std::time_t>(event.time);
        char text[16];
        std::strftime(text, sizeof text, "%d/%m %H:%M", std::localtime(&when));
        ImGui::TextColored(MutedText, "%s · %s: %s", text, event.author.c_str(), event.message.c_str());
    }
}

void ReinforcementLearningScreen::saveCapture() {
    if (m_image.empty()) return;
    std::error_code code;
    std::filesystem::create_directories(m_options.capturesDirectory, code);
    const std::filesystem::path path = m_options.capturesDirectory / ("rl_" + timestamp() + ".png");
    std::string error;
    if (writePng(m_image, path.string(), &error)) notify("Imagem salva: " + path.string());
    else notify("Falha ao salvar: " + error);
}

} // namespace MatterEngine::Workbench
