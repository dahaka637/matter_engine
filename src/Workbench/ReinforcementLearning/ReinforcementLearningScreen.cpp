#include "Workbench/ReinforcementLearning/ReinforcementLearningScreen.hpp"

#include "Engine/Modeling/Inspection/InspectionRenderer.hpp"
#include "Engine/Modeling/Raster/ImageIO.hpp"
#include "Engine/Modeling/Skeleton/Skeleton.hpp"
#include "Engine/RL/Teacher/LocomotionTeacher3D.hpp"
#include "Engine/UI/FontAwesome.hpp"
#include "Workbench/UiKit.hpp"
#include "imgui.h"

#include <algorithm>
#include <cctype>
#include <array>
#include <cfloat>
#include <chrono>
#include <cmath>
#include <numbers>
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

constexpr std::array<StateLabel, 11> StateLabels { {
    { "trocando", "Trocando de habilidade", AccentText },
    { "professor", "Aprendendo com a locomoção física", AccentText },
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
    loadSkills();
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

void ReinforcementLearningScreen::loadSkills() {
    std::vector<std::filesystem::path> files;
    std::error_code code;
    for (const auto& entry : std::filesystem::directory_iterator(m_options.sourceRoot / "assets" / "rl" / "tasks", code))
        if (entry.path().extension() == ".json") files.push_back(entry.path());
    std::sort(files.begin(), files.end());
    for (const std::filesystem::path& file : files) {
        try {
            const TrainingTask3D task = loadTrainingTask3D(file);
            if (!task.listed && !m_options.showUnlistedSkills) continue;
            Skill skill;
            skill.task = task.id;
            skill.name = task.displayName;
            float weight = -1.0f;
            for (const TrainingPostureWeight3D& posture : task.postures)
                if (posture.weight > weight) {
                    weight = posture.weight;
                    skill.posture = posture.id;
                }
            skill.battery = skill.posture == "natural" ? "equilibrio_v1" : "postura_" + skill.posture + "_v1";
            skill.maxForward = task.commands.forward;
            skill.teacher = task.hasTeacher;
            skill.target = task.target.enabled;
            if (skill.maxForward > 0.0f) {
                skill.battery = skill.walkBattery = "andar_frente_v1";
            }
            // A bateria do alvo e a primeira da tarefa (corrida, caminhada...).
            if (skill.target && !task.evaluationBatteries.empty()) skill.battery = task.evaluationBatteries.front();
            for (const TrainingTask3D::Drill& drill : task.drills) {
                skill.drills.push_back(drill.id);
                skill.drillNames.push_back(drill.name);
                // Os parados tem um peito so: o fixo, ou o girando.
                const TrainingTask3D::Target& target = drill.task->target;
                skill.drillChests.push_back(target.pattern != "hold" ? -1 : target.facingDriftFraction >= 1.0f ? 2 : 1);
            }
            skill.facing = task.target.facing;
            m_skills.push_back(std::move(skill));
        } catch (const std::exception&) {
            // Uma tarefa que nao abre fica fora da lista.
        }
    }
    for (std::size_t i = 0; i < m_skills.size(); ++i)
        if (m_skills[i].task == m_options.task) m_skill = i;
}

std::vector<std::string> ReinforcementLearningScreen::skillTasks() const {
    std::vector<std::string> tasks;
    for (const Skill& skill : m_skills) tasks.push_back(skill.task);
    return tasks;
}

std::string ReinforcementLearningScreen::skillName(const std::string& battery) const {
    for (const Skill& skill : m_skills)
        if (skill.battery == battery) return skill.name;
    return battery;
}

void ReinforcementLearningScreen::setWalkSpeed(float metersPerSecond) {
    const float maximum = m_skill < m_skills.size() ? m_skills[m_skill].maxForward : 0.0f;
    m_walkOverride = true;
    m_walkSpeed = std::clamp(metersPerSecond, 0.0f, maximum);
    if (m_viewer) m_viewer->setCommand(0, { m_walkSpeed, 0.0f, 0.0f });
}

void ReinforcementLearningScreen::selectSkill(std::size_t index) {
    if (index >= m_skills.size()) return;
    const bool wasRacing = m_skill < m_skills.size() && m_skills[m_skill].target;
    m_skill = index;
    m_viewerDrill.clear();
    m_viewerChest = 0;
    // Entrando ou saindo da corrida, a camera se ajusta (longe com o alvo).
    if (m_skills[index].target != wasRacing) frame(35.0f, 10.0f);
    m_skillChosen = true;
    // (O professor so na habilidade que o tem ligado.)
    if (!m_skills[index].teacher) m_showTeacher = false;
    m_walkSpeed = 0.0f;
    m_demoSeconds = 0.0f;
    m_walkOverride = false;
    // Outra tarefa (de ficar de pe para andar): o mundo do visualizador e refeito.
    const std::filesystem::path task = m_options.sourceRoot / "assets" / "rl" / "tasks" / (m_skills[index].task + ".json");
    if (m_viewer && task != m_viewerTask) {
        m_viewer.reset();
        m_viewerError.clear();
        ensureViewer();
        m_dirty = true;
        return;
    }
    if (m_viewer) {
        try {
            m_viewer->setCommand(0, {});
            m_viewer->setPosture(m_skills[index].posture);
            m_viewer->reset(static_cast<std::uint64_t>(std::time(nullptr)));
        } catch (const std::exception& error) {
            notify(error.what());
        }
    }
    m_dirty = true;
}

void ReinforcementLearningScreen::trainSelectedSkill() {
    const TrainerStatus3D status = m_link->status();
    if (m_skill >= m_skills.size() || !status.alive || status.task == m_skills[m_skill].task || !m_switchTask.empty())
        return;
    // O treino salva, fecha e reabre nela: vale para qualquer versao do
    // treinador aberto, e o treinador e o supervisor voltam atualizados.
    m_switchTask = m_skills[m_skill].task;
    m_switchMode = status.mode.empty() ? m_mode : status.mode;
    m_switchRun = status.run;
    m_switchSeconds = 0.0f;
    m_link->send("parar");
    notify("Trocando para " + m_skills[m_skill].name);
}

std::uint32_t ReinforcementLearningScreen::viewerPosture() const {
    return m_viewer ? m_viewer->view(0).posture : 0;
}

std::string ReinforcementLearningScreen::activeRun() const {
    const TrainerStatus3D status = m_link->status();
    return status.alive && !status.run.empty() ? status.run : m_options.run;
}

std::filesystem::path ReinforcementLearningScreen::taskPath() const {
    // A tarefa do treino (com os pesos de agora) ou a do repositorio.
    const std::filesystem::path live = m_link->paths().run(activeRun()) / "tarefa.json";
    if (std::filesystem::exists(live)) return live;
    return m_options.sourceRoot / "assets" / "rl" / "tasks" / (m_options.task + ".json");
}

std::filesystem::path ReinforcementLearningScreen::policyPath() const {
    return m_link->paths().run(activeRun()) / "exports"
        / (m_useBest ? "policy_best.mrlpolicy" : "policy_latest.mrlpolicy");
}

void ReinforcementLearningScreen::notify(std::string text) {
    m_notice = std::move(text);
    m_noticeSeconds = 4.0f;
}

void ReinforcementLearningScreen::frame(float yawDegrees, float pitchDegrees) {
    // Na corrida, a 3a pessoa: atras dele e um pouco acima, perto o bastante
    // para ver o passo e baixa o bastante para ver o alvo la na frente.
    const bool racing = m_skill < m_skills.size() && m_skills[m_skill].target;
    m_camera.yaw = yawDegrees * DegreesToRadians;
    m_camera.pitch = (racing ? std::max(pitchDegrees, 18.0f) : pitchDegrees) * DegreesToRadians;
    m_camera.distance = racing ? 7.0f : 4.4f;
    m_dirty = true;
}

void ReinforcementLearningScreen::ensureViewer() {
    if (m_viewer || !m_viewerError.empty()) return;
    try {
        TrainingBatchSettings3D settings;
        settings.taskPath = m_skill < m_skills.size()
            ? m_options.sourceRoot / "assets" / "rl" / "tasks" / (m_skills[m_skill].task + ".json")
            : taskPath();
        m_viewerTask = settings.taskPath;
        settings.environmentCount = 1;
        settings.environmentsPerScene = 1;
        settings.workerThreads = 1;
        settings.seed = 7;
        settings.evaluation = true;
        settings.evaluationEpisodeSeconds = 600.0f;
        // Cada recomeco (uma queda) segue o roteiro de outro mundo da bateria:
        // na estrela, outra direcao e outra velocidade.
        settings.evaluationCycle = true;
        // O visualizador mostra a fisica do jogo.
        settings.solverPositionIterations = 20;
        settings.solverVelocityIterations = 20;
        if (m_skill < m_skills.size()) settings.posture = m_skills[m_skill].posture;
        // O exercicio e o peito que o visualizador mostra (o treino sorteia).
        if (m_skill < m_skills.size() && !m_skills[m_skill].drills.empty()) {
            const auto& drills = m_skills[m_skill].drills;
            settings.drill = std::find(drills.begin(), drills.end(), m_viewerDrill) != drills.end() ? m_viewerDrill : drills.front();
        }
        if (m_skill < m_skills.size() && m_skills[m_skill].facing) settings.facing = m_viewerChest;
        if (m_showTeacher && m_skill < m_skills.size() && m_skills[m_skill].teacher)
            settings.teacher = std::make_shared<LocomotionTeacher3D>(
                m_options.sourceRoot / "assets", loadLocomotionTeacherSettings3D(settings.taskPath));
        m_viewer = std::make_unique<TrainingBatch3D>(settings);
        m_raceCamera.valid = false;
        m_viewer->setCommand(0, { m_walkSpeed, 0.0f, 0.0f });
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
    if (!force && write == m_policyWrite && path == m_policyPath) return;
    m_policyWrite = write;
    m_policyPath = path;
    try {
        auto policy = std::make_unique<PolicyRuntime3D>(path);
        // Com os nomes das entradas, a observacao da politica e montada por eles
        // (o que falta entra com zero, como na migracao do treino): qualquer
        // cerebro do corpo serve a qualquer tarefa dele.
        m_policyMap.clear();
        const std::vector<std::string>& names = policy->info().observationNames;
        if (names.size() == policy->observationSize() && policy->actionSize() == m_viewer->actionSize()) {
            const auto& channels = m_viewer->observationLayout().channels;
            for (const std::string& name : names) {
                int found = -1;
                for (std::size_t c = 0; c < channels.size(); ++c)
                    if (channels[c].name == name) found = static_cast<int>(c);
                m_policyMap.push_back(found);
            }
            m_policyInput.assign(names.size(), 0.0f);
            m_policy = std::move(policy);
            m_policyInputs = names.size();
            m_policyText = "política da rodada " + std::to_string(m_policy->info().iteration);
            return;
        }
        // Sem os nomes: os comandos fecham a observacao, na ordem em que
        // entraram; uma politica de antes de parte deles le o comeco.
        std::size_t commands = 0;
        for (const RLChannel3D& channel : m_viewer->observationLayout().channels)
            if (channel.name.rfind("command.", 0) == 0) ++commands;
        const std::size_t inputs = policy->observationSize(), full = m_viewer->observationSize();
        if (inputs > full || full - inputs > commands || policy->actionSize() != m_viewer->actionSize()) {
            m_policy.reset();
            m_policyText = "política de outro corpo";
            return;
        }
        m_policy = std::move(policy);
        m_policyInputs = inputs;
        m_policyText = "política da rodada " + std::to_string(m_policy->info().iteration);
    } catch (const std::exception& error) {
        m_policy.reset();
        m_policyText = error.what();
    }
}

void ReinforcementLearningScreen::setViewerDrill(const std::string& drill, int chest) {
    if (drill == m_viewerDrill && chest == m_viewerChest && m_viewer) return;
    m_viewerDrill = drill;
    m_viewerChest = chest;
    m_viewer.reset();
    m_viewerError.clear();
    ensureViewer();
    m_dirty = true;
}

void ReinforcementLearningScreen::showTeacher(bool show) {
    // (Sem professor ligado na habilidade, o pedido nao vale.)
    if (show && (m_skill >= m_skills.size() || !m_skills[m_skill].teacher)) show = false;
    if (show == m_showTeacher) return;
    m_showTeacher = show;
    // O mundo do visualizador e refeito (com ou sem o professor).
    m_viewer.reset();
    m_viewerError.clear();
    m_demoSeconds = 0.0f;
    ensureViewer();
    m_dirty = true;
}

void ReinforcementLearningScreen::stepOnce() {
    // O professor conduz o corpo: as acoes nao valem.
    if (m_viewer && !m_viewer->teacherActions().empty()) {
        std::fill(m_action.begin(), m_action.end(), 0.0f);
        m_viewer->step(m_action);
        return;
    }
    // Andar: o ciclo de demonstracao (parado 3 s, metade do limite 5 s, o
    // limite 5 s, parado 4 s), com o limite que o treino ja pede.
    if (m_skill < m_skills.size() && m_skills[m_skill].maxForward > 0.0f && !m_walkOverride) {
        m_demoSeconds = std::fmod(m_demoSeconds + m_viewer->policyStepSeconds(), 17.0f);
        m_walkSpeed = m_demoSeconds < 3.0f ? 0.0f
            : m_demoSeconds < 8.0f        ? 0.5f * m_demoLimit
            : m_demoSeconds < 13.0f       ? m_demoLimit
                                          : 0.0f;
        m_viewer->setCommand(0, { m_walkSpeed, 0.0f, 0.0f });
    }
    const std::span<const float> observation = m_viewer->observations();
    if (m_policy && !m_policyMap.empty()) {
        for (std::size_t i = 0; i < m_policyMap.size(); ++i)
            m_policyInput[i] = m_policyMap[i] >= 0 ? observation[static_cast<std::size_t>(m_policyMap[i])] : 0.0f;
        m_policy->evaluate(m_policyInput, m_action);
    } else if (m_policy) {
        m_policy->evaluate(observation.first(m_policyInputs), m_action);
    } else {
        std::fill(m_action.begin(), m_action.end(), 0.0f);
    }
    m_viewer->step(m_action);
    updateRaceCamera(m_viewer->policyStepSeconds(), false);
}

void ReinforcementLearningScreen::updateRaceCamera(float seconds, bool snap) {
    const TrainingEnvironmentView3D view = m_viewer ? m_viewer->view(0) : TrainingEnvironmentView3D {};
    if (!view.hasTarget || !view.state || view.state->links.empty()) {
        m_raceCamera.valid = false;
        return;
    }
    const Vec3 pelvis = view.state->links.front().position;
    const Vec3 to { view.target.x - pelvis.x, view.target.y - pelvis.y, 0.0f };
    // O rumo e o do mapa (dele ate o alvo), nao o do corpo, que balanca a cada
    // passo e gira com o peito fixo; perto do alvo o rumo fica (senao a camera
    // gira em volta dele na chegada).
    float yaw = m_raceCamera.yaw;
    if (std::hypot(to.x, to.y) > 1.5f || !m_raceCamera.valid) yaw = std::atan2(to.y, to.x);
    // Comeco, outro episodio ou outro mundo: a camera vai direto.
    if (snap || !m_raceCamera.valid || view.steps < m_raceCamera.steps) {
        m_raceCamera = { true, yaw, view.steps };
        return;
    }
    const float turn = 1.0f - std::exp(-seconds / 1.0f);
    m_raceCamera.yaw += std::remainder(yaw - m_raceCamera.yaw, 2.0f * std::numbers::pi_v<float>) * turn;
    m_raceCamera.steps = view.steps;
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
        [database = m_link->paths().database, run = activeRun()] { return readTrainingHistory3D(database, run, 0.0, 400); });
}

void ReinforcementLearningScreen::update(float deltaSeconds) {
    pollModel();
    ensureViewer();
    // A troca de habilidade: depois que o treino fecha, ele reabre na nova.
    if (!m_switchTask.empty()) {
        m_switchSeconds += deltaSeconds;
        const TrainerStatus3D status = m_link->status();
        if (!status.alive && status.state != "abrindo" && status.state != "parando") {
            std::string error;
            if (!m_link->start(m_switchTask, m_switchMode, &error, m_switchRun)) notify(error);
            m_switchTask.clear();
        } else if (m_switchSeconds > 90.0f) {
            notify("O treino não fechou para trocar de habilidade");
            m_switchTask.clear();
        }
    }
    // O limite do ciclo de andar do visualizador: o que o treino ja pede (o
    // curriculo), ou o comeco dele sem um treino de andar.
    {
        const TrainerStatus3D status = m_link->status();
        const float maximum = m_skill < m_skills.size() ? m_skills[m_skill].maxForward : 0.0f;
        m_demoLimit = status.forwardMax > 0.0 && status.forwardLimit > 0.0
            ? std::min(maximum, static_cast<float>(status.forwardLimit)) : std::min(1.0f, maximum);
    }
    // Ao abrir, a habilidade escolhida e a do treino (aberto ou o ultimo).
    if (!m_skillChosen) {
        const TrainerStatus3D status = m_link->status();
        if (status.known && !status.task.empty()) {
            for (std::size_t i = 0; i < m_skills.size(); ++i)
                if (m_skills[i].task == status.task && i != m_skill) selectSkill(i);
            m_skillChosen = true;
        }
    }
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
    // O mundo fica preso na origem do episodio (e nao na pelve): andando, o chao
    // corre sob os pes e a camera segue o jogador.
    const Vec3 anchor = view.origin;
    const SkeletonPose& pose = m_skeleton->update(positions, orientations, anchor, 0.0f, m_viewer->policyStepSeconds());
    const InspectionModel& model = *m_model->model;
    std::vector<Vec3> posed;
    if (model.mesh.hasSkin()) posed = model.mesh.posedPositions(skinningTransforms(model.skeleton, pose));

    std::vector<Highlight> highlights;
    if (m_display == Display::Colliders && m_colliders) {
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

    // O alvo: o disco vermelho no chao com o brilho subindo das bordas; os
    // proximos (os varios pontos) mais fracos, ligados em ordem; os ja
    // alcancados verdes.
    std::vector<InspectionMarker> markers;
    std::vector<InspectionArrow> arrows;
    if (view.hasTarget) {
        std::vector<Vec3> sequence { m_skeleton->toSkeleton({ view.target.x, view.target.y, 0.0f }) };
        for (std::uint32_t i = 0; i < view.upcomingCount; ++i)
            sequence.push_back(m_skeleton->toSkeleton({ view.upcoming[i].x, view.upcoming[i].y, 0.0f }));
        std::vector<Vec3> reached;
        for (std::uint32_t i = 0; i < view.reachedCount; ++i)
            reached.push_back(m_skeleton->toSkeleton({ view.reached[i].x, view.reached[i].y, 0.0f }));
        appendTargetSequence(sequence, reached, view.targetRadius, markers, arrows);
        // Os de "pare aqui" (frear e parar) em laranja.
        const std::size_t first = markers.size() - sequence.size();
        for (std::size_t i = 0; i < sequence.size(); ++i)
            if (i == 0 ? view.targetStop : view.upcomingStop[i - 1]) markers[first + i].color = rgb(245, 160, 40);
        // O desenho do exercicio (a estrela): os pontos fracos, fora os que ja
        // sao alvo agora.
        for (std::uint32_t i = 0; i < view.layoutCount; ++i) {
            const Vec3 point = m_skeleton->toSkeleton({ view.layout[i].x, view.layout[i].y, 0.0f });
            bool taken = false;
            for (const InspectionMarker& marker : markers)
                taken = taken || std::hypot(marker.center.x - point.x, marker.center.y - point.y) < view.targetRadius;
            if (taken) continue;
            InspectionMarker spot;
            spot.center = point;
            spot.radius = view.targetRadius * 0.7f;
            spot.color = rgb(170, 182, 200);
            spot.strength = 0.35f;
            spot.glowHeight = 0.0f;
            markers.push_back(spot);
        }
    }

    // A seta de para onde ele vai: a velocidade do centro de massa no chao (0,35 s
    // a frente), um traco fino.
    if (std::hypot(view.velocity.x, view.velocity.y) > 0.2f) {
        Vec3 center;
        for (std::size_t i = 0; i < state.links.size(); ++i) center += positions[i] * profile.links[i].massFraction;
        InspectionArrow going;
        going.from = m_skeleton->toSkeleton({ center.x, center.y, 0.03f });
        going.to = m_skeleton->toSkeleton(
            { center.x + view.velocity.x * 0.35f, center.y + view.velocity.y * 0.35f, 0.03f });
        going.color = rgb(245, 214, 60, 0.7f);
        going.lineWidthPixels = 2.0f;
        arrows.push_back(going);
    }
    // Para onde ele olha de verdade: um traco azul saindo da cabeca.
    std::vector<InspectionEye> eyes;
    if (view.gaze.lengthSquared() > 0.5f) {
        const Vec3 head = positions[m_viewer->body().head];
        InspectionArrow looking;
        looking.from = m_skeleton->toSkeleton(head);
        looking.to = m_skeleton->toSkeleton(head + view.gaze * 0.9f);
        looking.color = rgb(110, 190, 255, 0.8f);
        looking.lineWidthPixels = 2.0f;
        arrows.push_back(looking);
    }
    // E, com o comando de olhar, o olho no horizonte na direcao mandada.
    if (view.hasFacing) {
        const Vec3 pelvis = positions.front();
        InspectionEye eye;
        eye.position = m_skeleton->toSkeleton({ pelvis.x + std::cos(view.facingCommand) * 28.0f,
            pelvis.y + std::sin(view.facingCommand) * 28.0f, 1.7f });
        eye.size = 3.2f;
        eyes.push_back(eye);
    }

    InspectionScene scene;
    scene.skeleton = &model.skeleton;
    scene.mesh = &model.mesh;
    scene.regions = &model.regions;
    scene.jointRegion = model.jointRegion;
    scene.pose = &pose;
    scene.meshPositions = posed;
    scene.highlights = highlights;
    scene.markers = markers;
    scene.arrows = arrows;
    scene.eyes = eyes;
    InspectionStyle style;
    style.surface = m_display == Display::Full ? SurfaceMode::Solid : SurfaceMode::Hidden;
    style.skeleton = m_display == Display::Skeleton;
    style.joints = m_display == Display::Skeleton;
    style.hud = false;
    style.gizmo = false;
    style.scaleBar = false;
    // O chao: 70 m de cada lado com passo de 0,5 m na corrida (a camera de
    // tras ve longe, ate perto do horizonte); 12 m com a grade fina (10 cm) parado.
    const bool racing = view.hasTarget;
    style.gridExtent = racing ? 70.0f : 12.0f;
    style.gridStep = racing ? 0.5f : 0.0f;
    style.supersample = 1;
    OrbitCamera camera = m_camera;
    const Vec3 pelvis = m_skeleton->toSkeleton(positions.front());
    camera.target = { pelvis.x, pelvis.y, pelvis.z - 0.04f };
    // Na corrida, a 3a pessoa presa ao mapa: a camera mira a pelve (sem
    // atraso; so o rumo anda suave), atras dele no rumo do alvo e um pouco de
    // lado, para ver as pernas (o giro do mouse soma em cima disso). Por
    // tras, o caminho ate o alvo ja fica na frente dele no quadro.
    if (view.hasTarget) {
        if (!m_raceCamera.valid) updateRaceCamera(0.0f, true);
        const Vec3 base { positions.front().x, positions.front().y, 0.0f };
        const Vec3 aim = m_skeleton->toSkeleton(base);
        const Vec3 ahead = m_skeleton->toSkeleton(base + Vec3 { std::cos(m_raceCamera.yaw), std::sin(m_raceCamera.yaw), 0.0f });
        camera.target = { aim.x, aim.y, 0.85f * pelvis.z };
        const float run = std::atan2(ahead.y - aim.y, ahead.x - aim.x);
        camera.yaw = run - std::numbers::pi_v<float> + 0.45f + (m_camera.yaw - 35.0f * DegreesToRadians);
    }
    // Onde ele aparece (os testes conferem que a camera nao o corta).
    const ScreenPoint shown = Projector(camera, width, height).project(pelvis);
    m_pelvisOnScreen = shown.visible
        ? std::array<float, 2> { shown.x / static_cast<float>(width), shown.y / static_cast<float>(height) }
        : std::array<float, 2> { -1.0f, -1.0f };
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
    drawSkills(m_link->status());
    drawEvaluation(m_link->status());
    const std::string back = UI::FontAwesome::label(UI::FontAwesome::ArrowLeft, "Menu principal");
    ImGui::SetCursorPosY(std::max(ImGui::GetCursorPosY(), height - ui(58.0f)));
    if (navigationButton(back.c_str(), false, { -1.0f, ui(40.0f) }) && m_onBack) m_onBack();
}

void ReinforcementLearningScreen::drawTrainer() {
    const TrainerStatus3D status = m_link->status();
    panelHeader("TREINO");
    ImGui::Dummy({ 1.0f, ui(2.0f) });
    const StateLabel state = stateLabel(m_switchTask.empty() ? status.state : std::string("trocando"));
    ImGui::TextColored(state.color, "%s", state.label);
    if (status.alive && !status.device.empty()) {
        ImGui::SameLine();
        ImGui::TextColored(MutedText, "· %s", status.device == "cuda" ? "GPU" : "CPU");
    }
    ImGui::Dummy({ 1.0f, ui(2.0f) });
    if (status.known) {
        if (status.alive && status.run != m_options.run) infoRow("Treino", status.run);
        infoRow("Habilidade", status.taskName.empty() ? status.task : status.taskName);
        tooltip("A habilidade que o treino está treinando (a marcada com o raio em HABILIDADES).");
        if (status.forwardMax > 0.0) {
            infoRow("Velocidade", decimal(status.forwardLimit, 2) + " de " + decimal(status.forwardMax, 1) + " m/s");
            tooltip("O currículo: a velocidade máxima que o treino pede agora, que sobe conforme ele acompanha, "
                    "até o alvo.");
        }
        infoRow("Rodada", std::to_string(status.iteration));
        infoRow("Decisões", decimal(static_cast<double>(status.steps) / 1e6, 1) + " M");
        if (status.alive) infoRow("Velocidade", decimal(status.sps, 0) + " /s");
        infoRow("Treinado", duration(status.trainSeconds));
        if (status.alive && status.lastFallFraction >= 0.0)
            infoRow("Episódio", decimal(status.lastEpisodeSeconds, 1) + " s");
        if (status.alive && status.cpuTemperature > 0.0) infoRow("CPU", decimal(status.cpuTemperature, 0) + " °C");
        if (status.alive && status.imitation.active) {
            infoRow("Professor", decimal(status.imitation.elapsedSeconds / 60.0, 1) + " de "
                    + decimal(status.imitation.minutes, 0) + " min");
            tooltip("O aprendizado guiado: a locomoção física do jogo conduz o corpo nos mundos de treino e a política "
                    "imita o que ela faz. No fim, o treino normal segue sozinho.");
            if (status.imitation.loss >= 0.0) {
                infoRow("Erro da imitação", decimal(status.imitation.loss, 3));
                tooltip("O quanto a política ainda erra a ação do professor (quadrado médio, em unidades da ação). "
                        "Cai conforme ela aprende.");
            }
            infoRow("O professor", decimal(status.imitation.teacherSpeed, 2) + " m/s · caiu em "
                    + decimal(status.imitation.teacherFalls * 100.0, 0) + "%");
            tooltip("A velocidade média que o professor anda no roteiro (com os trechos parado e girando) e a fração "
                    "dos episódios em que ele caiu.");
        } else if (status.alive && status.kickstart.active) {
            const bool critic = status.kickstart.since < status.kickstart.criticIterations;
            infoRow("Depois do professor", critic ? "crítico " + std::to_string(status.kickstart.since) + " de "
                                                        + std::to_string(status.kickstart.criticIterations)
                                                  : "imitação a " + decimal(status.kickstart.weight, 2));
            tooltip(critic ? "Primeiro só o crítico aprende (os valores da política nova), com o ator parado."
                           : "O treino normal, com um resto da imitação do professor que some aos poucos.");
        }
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
        ImGui::BeginDisabled(opening || !m_switchTask.empty());
        const std::string& task = m_skill < m_skills.size() ? m_skills[m_skill].task : m_options.task;
        // Com outra habilidade escolhida, o botao diz qual vai ser treinada.
        const std::string text = !status.known ? std::string("Iniciar treino")
            : status.task == task || m_skill >= m_skills.size() ? std::string("Continuar treino")
                                                                 : "Continuar em: " + m_skills[m_skill].name;
        const std::string label = UI::FontAwesome::label(UI::FontAwesome::Play, text.c_str());
        if (ImGui::Button(label.c_str(), { -1.0f, ui(36.0f) })) {
            std::string error;
            if (!m_link->start(task, m_mode, &error)) notify(error);
        }
        ImGui::EndDisabled();
        tooltip("Abre o treino em segundo plano, do último checkpoint, na habilidade escolhida em HABILIDADES. "
                "Ele continua com o Matter Engine fechado.");
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
        if (!m_link->runLauncher({ "servico", boot ? "instalar" : "remover", "--run", activeRun() }, &error))
            notify(error);
        else notify(boot ? "O treino volta sozinho ao ligar o PC" : "O treino não volta mais sozinho");
    }
    tooltip("Se o PC desligar com o treino aberto, ele volta sozinho no próximo login. Um \"Parar\" seu não volta.");
}

void ReinforcementLearningScreen::drawSkills(const TrainerStatus3D& status) {
    ImGui::Dummy({ 1.0f, ui(8.0f) });
    panelHeader("HABILIDADES");
    ImGui::Dummy({ 1.0f, ui(2.0f) });
    for (std::size_t i = 0; i < m_skills.size(); ++i) {
        const Skill& skill = m_skills[i];
        const bool trained = status.known && status.task == skill.task;
        const std::string name = trained ? skill.name + " · em treino" : skill.name;
        const std::string label
            = UI::FontAwesome::label(trained ? UI::FontAwesome::Bolt : UI::FontAwesome::Brain, name.c_str()) + "##" + skill.task;
        if (navigationButton(label.c_str(), i == m_skill, { -1.0f, ui(32.0f) }) && i != m_skill) selectSkill(i);
        tooltip(trained ? "A habilidade do treino (o raio). Escolher uma habilidade mostra a postura dela no mundo."
                        : "Escolher mostra a postura desta habilidade no mundo.");
        drawDrillTree(i);
    }
    // O aprendizado guiado: a habilidade em treino, com professor.
    if (m_skill < m_skills.size() && status.alive && status.task == m_skills[m_skill].task && m_skills[m_skill].teacher) {
        ImGui::Dummy({ 1.0f, ui(2.0f) });
        ImGui::BeginDisabled(status.state == "professor" || !m_switchTask.empty());
        const std::string label = UI::FontAwesome::label(UI::FontAwesome::Brain, "Aprender com a locomoção física");
        if (ImGui::Button(label.c_str(), { -1.0f, ui(32.0f) })) {
            m_link->send("professor");
            notify("O professor assume os mundos de treino");
        }
        ImGui::EndDisabled();
        tooltip("A locomoção física do jogo (sem força auxiliar) conduz o boneco nos mundos de treino: parado, girando, "
                "andando para todos os lados e levando empurrões. A política imita por um tempo e depois o treino "
                "normal segue com o que ela extraiu. Para ver o professor, escolha \"Professor\" acima do mundo.");
    }
    if (m_skill < m_skills.size() && status.alive && status.task != m_skills[m_skill].task) {
        ImGui::Dummy({ 1.0f, ui(2.0f) });
        ImGui::BeginDisabled(!m_switchTask.empty());
        const std::string label = UI::FontAwesome::label(UI::FontAwesome::Play, "Treinar esta habilidade");
        if (ImGui::Button(label.c_str(), { -1.0f, ui(32.0f) })) trainSelectedSkill();
        ImGui::EndDisabled();
        tooltip("O treino passa para esta habilidade, no mesmo cérebro: ele salva, fecha e reabre nela, e o que já "
                "sabe continua (as habilidades anteriores seguem numa parte dos episódios).");
    }
}

// Os exercicios da habilidade como ramos dela e, com o comando de olhar, os
// modos do peito como ramos de cada exercicio: escolher um mostra aquilo no
// visualizador (o treino sorteia todos).
void ReinforcementLearningScreen::drawDrillTree(std::size_t index) {
    const Skill& skill = m_skills[index];
    if (skill.drills.empty()) return;
    const bool here = index == m_skill;
    const std::string current = here && std::find(skill.drills.begin(), skill.drills.end(), m_viewerDrill) != skill.drills.end()
        ? m_viewerDrill : skill.drills.front();
    static constexpr const char* Chests[] = { "Peito livre", "Peito fixo", "Peito girando" };
    for (std::size_t d = 0; d < skill.drills.size(); ++d) {
        const std::string& drill = skill.drills[d];
        const bool open = here && drill == current;
        ImGui::Indent(ui(14.0f));
        const std::string label = UI::FontAwesome::label(UI::FontAwesome::Crosshairs, skill.drillNames[d].c_str())
            + "##drill" + skill.task + drill;
        const int only = skill.drillChests[d];
        const bool branches = skill.facing && only < 0;
        if (navigationButton(label.c_str(), open && !branches, { -1.0f, ui(28.0f) })) {
            if (!here) selectSkill(index);
            setViewerDrill(drill, only >= 0 ? only : m_viewerChest);
        }
        if (branches) {
            ImGui::Indent(ui(14.0f));
            for (int chest = 0; chest < 3; ++chest) {
                const std::string item = std::string(Chests[chest]) + "##chest" + skill.task + drill + std::to_string(chest);
                if (navigationButton(item.c_str(), open && chest == m_viewerChest, { -1.0f, ui(26.0f) })) {
                    if (!here) selectSkill(index);
                    setViewerDrill(drill, chest);
                }
            }
            ImGui::Unindent(ui(14.0f));
        }
        ImGui::Unindent(ui(14.0f));
    }
}

void ReinforcementLearningScreen::drawEvaluation(const TrainerStatus3D& status) {
    ImGui::Dummy({ 1.0f, ui(8.0f) });
    panelHeader("AVALIAÇÃO");
    ImGui::Dummy({ 1.0f, ui(2.0f) });
    if (!status.evaluated) {
        ImGui::TextColored(MutedText, "—");
        return;
    }
    // Uma bateria por habilidade: de pe no jogo e no treino, e o giro do tronco.
    for (const TrainerStatus3D::Battery& battery : status.evaluationBatteries) {
        if (status.evaluationBatteries.size() > 1) ImGui::TextColored(InfoText, "%s", skillName(battery.battery).c_str());
        if (battery.kind != "de_pe") {
            // Andar (o roteiro de velocidades) e ir ate o ponto: sem cair e o quanto acerta.
            for (const auto& [solver, stand] : battery.stand)
                if (solver == "jogo") infoRow("Sem cair (jogo)", decimal(stand * 100.0, 0) + "%");
            if (battery.kind == "andar" && battery.velocityError >= 0.0) {
                infoRow("Erro da velocidade", decimal(battery.velocityError, 2) + " m/s");
                tooltip("A diferença média entre a velocidade pedida e a dele no roteiro (parado, 1,5, 3 e 4,5 m/s, "
                        "parar), na física do jogo.");
            }
            if (battery.kind == "andar" && battery.lateralDrift >= 0.0)
                infoRow("Desvio lateral", decimal(battery.lateralDrift, 2) + " m");
            if (battery.kind == "alvo" && battery.goalError >= 0.0) {
                infoRow("Erro no ponto", decimal(battery.goalError, 2) + " m");
                tooltip("O quanto ele para longe do ponto a 10 m na frente (indo a até 2 m/s e freando perto).");
            }
            continue;
        }
        for (const auto& [solver, stand] : battery.stand) {
            const std::string name = solver == "jogo" ? "De pé (jogo)" : "De pé (treino)";
            infoRow(name.c_str(), decimal(stand * 100.0, 0) + "%");
        }
        tooltip("Quantos dos 32 mundos da bateria ficaram de pé os 60 s, sem ruído e sem variação do mundo. "
                "\"Jogo\" é a mesma bateria com a física do jogo (20/20).");
        if (battery.turn >= 0.0) {
            infoRow("Giro do tronco", decimal(battery.turn * 180.0 / 3.14159265358979, 1) + "°/s");
            tooltip("A velocidade angular da pelve e do peito na bateria do jogo (média dos mundos). Parado de "
                    "verdade fica perto de zero; o tronco rodando de um lado para o outro sobe.");
        }
        if (battery.poseError >= 0.0) {
            infoRow("Postura", decimal(battery.poseError * 180.0 / 3.14159265358979, 1) + "°");
            tooltip("O quanto as juntas ficam longe da postura pedida (a média, nos mundos da bateria do jogo). "
                    "Perto de zero, ele segura a postura da habilidade.");
        }
    }
    infoRow("Nota", decimal(status.evaluationScore, 2) + " (rodada " + std::to_string(status.evaluationIteration) + ")");
    tooltip("A nota da pior bateria: uma habilidade nova não esconde a antiga.");
    // (Um melhor de outra conta da nota guarda so a rodada, ate a proxima avaliacao.)
    if (status.bestIteration >= 0 && status.bestScore > -1.0e8)
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
    if (ImGui::Checkbox("Centro de massa", &m_showCenterOfMass)) m_dirty = true;
    tooltip("O centro de massa (amarelo) e a projeção dele no chão: dentro dos pés, ele está equilibrado.");
    ImGui::SameLine();
    if (ImGui::Checkbox("Contatos", &m_showContacts)) m_dirty = true;
    ImGui::SameLine();
    ImGui::SetNextItemWidth(ui(150.0f));
    const bool teacherAvailable = m_skill < m_skills.size() && m_skills[m_skill].teacher;
    const char* source = m_showTeacher ? "Professor" : m_useBest ? "Política melhor" : "Política recente";
    if (ImGui::BeginCombo("##RLPolicySource", source)) {
        if (ImGui::Selectable("Política recente", !m_showTeacher && !m_useBest)) {
            m_useBest = false;
            showTeacher(false);
            reloadPolicy(true);
        }
        if (ImGui::Selectable("Política melhor", !m_showTeacher && m_useBest)) {
            m_useBest = true;
            showTeacher(false);
            reloadPolicy(true);
        }
        if (teacherAvailable && ImGui::Selectable("Professor", m_showTeacher)) showTeacher(true);
        ImGui::EndCombo();
    }
    tooltip("Recente: o último checkpoint do treino (recarrega sozinha). Melhor: a de maior nota na avaliação. "
            "Professor: a locomoção física do jogo conduzindo o boneco do RL, no roteiro do aprendizado guiado.");
    ImGui::SameLine();
    struct DisplayOption {
        Display display;
        const char* label;
    };
    constexpr std::array<DisplayOption, 3> Displays { { { Display::Full, "Render completo" },
        { Display::Skeleton, "Só esqueleto" }, { Display::Colliders, "Só colisores" } } };
    ImGui::SetNextItemWidth(ui(150.0f));
    if (ImGui::BeginCombo("##RLDisplay", Displays[static_cast<std::size_t>(m_display)].label)) {
        for (const DisplayOption& option : Displays)
            if (ImGui::Selectable(option.label, m_display == option.display)) {
                m_display = option.display;
                m_dirty = true;
            }
        ImGui::EndCombo();
    }
    tooltip("O que aparece no mundo: o jogador inteiro, só o esqueleto ou só as caixas de colisão da física.");
    ImGui::SameLine();
    const std::string pause = m_paused ? UI::FontAwesome::label(UI::FontAwesome::Play, "Continuar")
                                       : UI::FontAwesome::label(UI::FontAwesome::Pause, "Congelar");
    if (ImGui::Button(pause.c_str(), { ui(110.0f), ui(30.0f) })) m_paused = !m_paused;
    tooltip("Congela só o visualizador (o treino segue).");
    ImGui::SameLine();
    if (ImGui::Button(UI::FontAwesome::label(UI::FontAwesome::RotateLeft, "Recomeçar").c_str(), { ui(120.0f), ui(30.0f) })
        && m_viewer) {
        m_viewer->reset(static_cast<std::uint64_t>(std::time(nullptr)));
        m_demoSeconds = 0.0f;
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
    // A politica que esta no corpo agora (canto de cima) e, andando, o pedido
    // do ciclo e a velocidade dele.
    std::string label = m_policyText;
    if (m_viewer && m_skill < m_skills.size() && m_skills[m_skill].maxForward > 0.0f) {
        const TrainingEnvironmentView3D view = m_viewer->view(0);
        const Vec3 v = view.state ? view.state->links.front().linearVelocity : Vec3 {};
        label += "\npedido " + decimal(m_walkSpeed, 1) + " m/s · ele " + decimal(std::hypot(v.x, v.y), 1) + " m/s";
    }
    if (m_viewer && m_skill < m_skills.size() && m_skills[m_skill].target) {
        const TrainingEnvironmentView3D view = m_viewer->view(0);
        const Vec3 v = view.state ? view.state->links.front().linearVelocity : Vec3 {};
        label += "\n" + decimal(std::hypot(v.x, v.y), 1) + " m/s"
            + (view.hasSpeedCommand ? " · pede " + decimal(view.speedNow > 0.0f ? view.speedNow : view.speedCommand, 1) : std::string())
            + " · " + std::to_string(view.targetsReached) + (view.targetsReached == 1 ? " alvo" : " alvos");
    }
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
    // (Os rotulos repetem os das vistas, "Frente": IDs proprios.)
    ImGui::PushID("RLPush");
    for (int i = 0; i < 4; ++i) {
        ImGui::SameLine();
        ImGui::BeginDisabled(!m_viewer);
        if (ImGui::Button(labels[static_cast<std::size_t>(i)], { ui(84.0f), ui(28.0f) })) push(i, m_pushNewtons);
        ImGui::EndDisabled();
    }
    ImGui::PopID();
}

void ReinforcementLearningScreen::drawCharts(float height) {
    ImGui::Dummy({ 1.0f, ui(4.0f) });
    std::vector<float> stand, seconds, reward, speed;
    // De pe: a bateria da habilidade escolhida, no jogo.
    const std::string battery = m_skill < m_skills.size() ? m_skills[m_skill].battery : std::string("equilibrio_v1");
    for (const TrainingEvaluationPoint3D& point : m_history.evaluations)
        if (point.solver == "jogo" && (point.battery == battery || (point.battery.empty() && battery == "equilibrio_v1")))
            stand.push_back(static_cast<float>(point.standFraction * 100.0));
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
    const bool walking = m_skill < m_skills.size() && m_skills[m_skill].maxForward > 0.0f;
    const bool racing = m_skill < m_skills.size() && m_skills[m_skill].target;
    const std::array<Chart, 4> charts { {
        { racing ? "Sem cair (%)" : walking ? "Anda sem cair (%)" : "De pé 60 s (%)", &stand, 0.0f, 100.0f,
            "A bateria da habilidade escolhida, com a física do jogo, a cada 10 min." },
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
