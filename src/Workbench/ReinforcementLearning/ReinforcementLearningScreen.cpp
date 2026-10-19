#include "Workbench/ReinforcementLearning/ReinforcementLearningScreen.hpp"

#include "Engine/Modeling/Inspection/InspectionRenderer.hpp"
#include "Engine/Modeling/Raster/ImageIO.hpp"
#include "Engine/Modeling/Skeleton/Skeleton.hpp"
#include "Engine/UI/FontAwesome.hpp"
#include "Workbench/UiKit.hpp"
#include "imgui.h"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <array>
#include <cfloat>
#include <chrono>
#include <cmath>
#include <ctime>
#include <fstream>
#include <thread>
#include <string_view>
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
// A gravacao do visualizador: 30 s a 30 decisoes por segundo.
constexpr std::size_t ReplayFrames = 900;
constexpr std::array<float, 5> PlaybackSpeeds { 0.05f, 0.1f, 0.25f, 0.5f, 1.0f };

Quaternion blendQuaternion(Quaternion a, Quaternion b, float t) {
    // (O caminho curto; entre duas decisoes o giro e pequeno, a media normalizada basta.)
    if (a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w < 0.0f) b = { -b.x, -b.y, -b.z, -b.w };
    return Quaternion { a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t, a.z + (b.z - a.z) * t, a.w + (b.w - a.w) * t }
        .normalized();
}
// O teto do comando na mao (m/s): o sprint do jogador.
constexpr float MaximumManualSpeed = 6.5f;

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

// A cor de uma fase (1: verde, 2: ambar, 3: violeta, 4: rosa, 5: ciano; 0 ou alem: sem cor).
bool phaseColor(int phase, ImVec4& color) {
    switch (phase) {
    case 1: color = { 0.25f, 0.78f, 0.52f, 1.0f }; return true;
    case 2: color = { 0.96f, 0.66f, 0.22f, 1.0f }; return true;
    case 3: color = { 0.66f, 0.50f, 0.96f, 1.0f }; return true;
    case 4: color = { 0.94f, 0.44f, 0.66f, 1.0f }; return true;
    case 5: color = { 0.30f, 0.80f, 0.92f, 1.0f }; return true;
    default: return false;
    }
}

// O botao de um exercicio, na cor da fase dele: o fundo e o texto na cor (mais forte
// ativo) e uma barra cheia na borda esquerda. Sem fase, o botao de sempre.
bool phaseButton(const char* label, bool active, ImVec2 size, int phase) {
    ImVec4 color;
    if (!phaseColor(phase, color)) return navigationButton(label, active, size);
    const auto mix = [&](float amount) {
        return ImVec4(0.055f + (color.x - 0.055f) * amount, 0.095f + (color.y - 0.095f) * amount,
            0.14f + (color.z - 0.14f) * amount, 1.0f);
    };
    ImGui::PushStyleVar(ImGuiStyleVar_ButtonTextAlign, { 0.08f, 0.5f });
    ImGui::PushStyleColor(ImGuiCol_Button, active ? mix(0.55f) : mix(0.14f));
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, active ? mix(0.68f) : mix(0.26f));
    ImGui::PushStyleColor(ImGuiCol_ButtonActive, mix(0.80f));
    ImGui::PushStyleColor(ImGuiCol_Text, active ? ImVec4(0.97f, 0.98f, 1.0f, 1.0f) : mix(0.78f));
    const bool clicked = ImGui::Button(label, size);
    ImGui::PopStyleColor(4);
    ImGui::PopStyleVar();
    const ImVec2 low = ImGui::GetItemRectMin(), high = ImGui::GetItemRectMax();
    const float rounding = ImGui::GetStyle().FrameRounding;
    ImGui::GetWindowDrawList()->AddRectFilled(low, { low.x + rounding + 4.0f, high.y },
        ImGui::ColorConvertFloat4ToU32(color), rounding, ImDrawFlags_RoundCornersLeft);
    return clicked;
}

// Um treino: o run.json do Corpo (contrato 2) ou o da locomocao.
bool isTrainingRun(const std::filesystem::path& directory) {
    std::ifstream file(directory / "run.json");
    if (!file) return false;
    const nlohmann::json config = nlohmann::json::parse(file, nullptr, false);
    if (config.is_discarded() || !config.is_object()) return false;
    if (config.value("kind", std::string()) == "locomocao" || config.value("kind", std::string()) == "rastreador") return true;
    const auto contract = config.find("contract");
    return contract != config.end() && contract->is_number() && contract->get<int>() == 2;
}

// As janelas de tempo dos graficos expandidos.
constexpr std::array<std::pair<const char*, double>, 4> ChartWindows { { { "60 s", 60.0 }, { "60 min", 3600.0 },
    { "5 h", 5.0 * 3600.0 }, { "24 h", 24.0 * 3600.0 } } };

std::string clockText(double unixSeconds) {
    const std::time_t when = static_cast<std::time_t>(unixSeconds);
    char text[24];
    std::strftime(text, sizeof text, "%d/%m %H:%M:%S", std::localtime(&when));
    return text;
}

// Uma serie de um grafico: os valores, a cor e se e a de referencia (mais apagada, sem preenchimento).
struct ChartSeries {
    const std::vector<float>* values;
    ImVec4 color;
    bool reference = false;
    std::string label;
};

// Um grafico de linha: fundo, grade, a serie principal com a area, a de referencia por tras, o valor de
// agora grande no canto e a dica com o valor sob o mouse.
void lineChart(const char* id, const std::vector<ChartSeries>& series, float minimum, float maximum, ImVec2 size,
    const char* valueText, const char* unit, const std::vector<double>* times = nullptr) {
    ImGui::InvisibleButton(id, size);
    const bool hovered = ImGui::IsItemHovered();
    const ImVec2 low = ImGui::GetItemRectMin(), high = ImGui::GetItemRectMax();
    ImDrawList* draw = ImGui::GetWindowDrawList();
    draw->AddRectFilled(low, high, IM_COL32(10, 17, 25, 255), 5.0f);
    draw->AddRect(low, high, IM_COL32(40, 62, 84, 255), 5.0f);
    const ImVec2 inner0 { low.x + 8.0f, low.y + 8.0f }, inner1 { high.x - 8.0f, high.y - 8.0f };
    const float span = std::max(maximum - minimum, 1e-6f);
    const auto yOf = [&](float v) {
        return inner1.y - (std::clamp(v, minimum, maximum) - minimum) / span * (inner1.y - inner0.y);
    };
    for (int i = 1; i <= 3; ++i) {
        const float y = inner0.y + (inner1.y - inner0.y) * static_cast<float>(i) / 4.0f;
        draw->AddLine({ low.x + 1.0f, y }, { high.x - 1.0f, y }, IM_COL32(32, 48, 64, 140));
    }
    int hoverIndex = -1;
    std::size_t hoverCount = 0;
    for (const ChartSeries& item : series) hoverCount = std::max(hoverCount, item.values->size());
    if (hovered && hoverCount > 1) {
        const float t = (ImGui::GetIO().MousePos.x - inner0.x) / std::max(inner1.x - inner0.x, 1.0f);
        hoverIndex = std::clamp(static_cast<int>(std::lround(t * static_cast<float>(hoverCount - 1))), 0,
            static_cast<int>(hoverCount) - 1);
    }
    for (int pass = 0; pass < 2; ++pass) {
        for (const ChartSeries& item : series) {
            if ((pass == 0) != item.reference) continue;
            const std::vector<float>& v = *item.values;
            if (v.size() < 2) continue;
            const auto point = [&](std::size_t i) {
                return ImVec2 { inner0.x + (inner1.x - inner0.x) * static_cast<float>(i) / static_cast<float>(v.size() - 1),
                    yOf(v[i]) };
            };
            ImVec4 line = item.color;
            if (item.reference) line.w = 0.45f;
            const ImU32 lineColor = ImGui::ColorConvertFloat4ToU32(line);
            if (!item.reference) {
                ImVec4 fill = item.color;
                fill.w = 0.16f;
                const ImU32 fillColor = ImGui::ColorConvertFloat4ToU32(fill);
                for (std::size_t i = 0; i + 1 < v.size(); ++i) {
                    const ImVec2 a = point(i), b = point(i + 1);
                    draw->AddQuadFilled(a, b, { b.x, inner1.y }, { a.x, inner1.y }, fillColor);
                }
            }
            for (std::size_t i = 0; i + 1 < v.size(); ++i)
                draw->AddLine(point(i), point(i + 1), lineColor, item.reference ? 1.2f : 2.0f);
            if (!item.reference) draw->AddCircleFilled(point(v.size() - 1), 3.2f, lineColor);
        }
    }
    if (hoverIndex >= 0) {
        const float x = inner0.x + (inner1.x - inner0.x) * static_cast<float>(hoverIndex)
            / static_cast<float>(hoverCount - 1);
        draw->AddLine({ x, inner0.y }, { x, inner1.y }, IM_COL32(120, 150, 180, 120));
        ImGui::BeginTooltip();
        if (times && static_cast<std::size_t>(hoverIndex) < times->size())
            ImGui::TextDisabled("%s", clockText((*times)[static_cast<std::size_t>(hoverIndex)]).c_str());
        for (const ChartSeries& item : series)
            if (static_cast<std::size_t>(hoverIndex) < item.values->size())
                ImGui::TextColored(item.color, "%s%.2f %s",
                    !item.label.empty() ? (item.label + "  ").c_str() : item.reference ? "clássico  " : "",
                    (*item.values)[static_cast<std::size_t>(hoverIndex)], unit);
        ImGui::EndTooltip();
    }
    if (valueText[0]) {
        const ImVec2 extent = ImGui::CalcTextSize(valueText);
        draw->AddText({ high.x - extent.x - 10.0f, low.y + 6.0f }, ImGui::ColorConvertFloat4ToU32(series.front().color),
            valueText);
    }
    char range[32];
    std::snprintf(range, sizeof range, "%.3g", maximum);
    draw->AddText({ low.x + 10.0f, low.y + 6.0f }, IM_COL32(100, 120, 140, 200), range);
    std::snprintf(range, sizeof range, "%.3g", minimum);
    draw->AddText({ low.x + 10.0f, high.y - ImGui::GetTextLineHeight() - 6.0f }, IM_COL32(100, 120, 140, 200), range);
    if (times && times->size() > 1) {
        // O comeco e o fim do trecho, embaixo nos cantos.
        const std::string first = clockText(times->front()), last = clockText(times->back());
        const float y = high.y - ImGui::GetTextLineHeight() - 6.0f;
        draw->AddText({ low.x + 60.0f, y }, IM_COL32(100, 120, 140, 200), first.c_str());
        draw->AddText({ high.x - ImGui::CalcTextSize(last.c_str()).x - 10.0f, y }, IM_COL32(100, 120, 140, 200),
            last.c_str());
    }
    if (std::all_of(series.begin(), series.end(), [](const ChartSeries& s) { return s.values->empty(); })) {
        const ImVec2 extent = ImGui::CalcTextSize("sem dados");
        draw->AddText({ (low.x + high.x - extent.x) * 0.5f, (low.y + high.y - extent.y) * 0.5f },
            IM_COL32(100, 120, 140, 200), "sem dados");
    }
}

} // namespace

ReinforcementLearningScreen::ReinforcementLearningScreen(Options options, std::function<void()> onBack)
    : m_options(std::move(options)), m_onBack(std::move(onBack)), m_paths(defaultTrainerPaths3D(m_options.sourceRoot)) {
    scanRuns();
    m_run = !m_options.run.empty() ? m_options.run : m_runs.empty() ? std::string() : m_runs.front();
    m_link = std::make_unique<TrainerLink3D>(m_paths, m_run);
    m_pending = std::async(std::launch::async, [] {
        ModelOutcome outcome;
        outcome.loaded = loadPlayerModel(defaultPlayerModelManifest(), {}, &outcome.error);
        return outcome;
    });
    frame(90.0f, 6.0f);
}

ReinforcementLearningScreen::~ReinforcementLearningScreen() = default;

TrainerStatus3D ReinforcementLearningScreen::status() const { return m_link->status(); }

void ReinforcementLearningScreen::scanRuns() {
    // Os treinos, do mais recente (o ultimo checkpoint gravado) ao mais antigo.
    std::vector<std::pair<std::filesystem::file_time_type, std::string>> found;
    std::error_code code;
    for (const auto& entry : std::filesystem::directory_iterator(m_paths.data / "runs", code)) {
        if (!entry.is_directory() || !isTrainingRun(entry.path())) continue;
        std::error_code timeCode;
        std::filesystem::path stamp = entry.path() / "run.json";
        for (const char* name : { "status.json", "latest.json" })
            if (std::filesystem::exists(entry.path() / name, timeCode)) stamp = entry.path() / name;
        found.emplace_back(std::filesystem::last_write_time(stamp, timeCode), entry.path().filename().string());
    }
    std::sort(found.begin(), found.end(), [](const auto& a, const auto& b) { return a.first > b.first; });
    m_runs.clear();
    for (auto& [time, name] : found) m_runs.push_back(std::move(name));
}

void ReinforcementLearningScreen::selectRun(const std::string& run) {
    if (run == m_run) return;
    m_run = run;
    m_link = std::make_unique<TrainerLink3D>(m_paths, m_run);
    m_historyRun.clear();
}

std::string ReinforcementLearningScreen::activeRun() const {
    const TrainerStatus3D status = m_link->status();
    return status.alive && !status.run.empty() ? status.run : m_run;
}

std::filesystem::path ReinforcementLearningScreen::taskPath() const {
    // A tarefa do treino (com os pesos e o curriculo de agora) ou a do repositorio.
    const std::filesystem::path live = m_paths.run(activeRun()) / "tarefa.json";
    if (std::filesystem::exists(live)) return live;
    const TrainerStatus3D status = m_link->status();
    if (status.task.empty()) return {};
    return m_options.sourceRoot / "assets" / "rl" / "tasks" / (status.task + ".json");
}

std::filesystem::path ReinforcementLearningScreen::policyPath() const {
    return m_paths.run(activeRun()) / "exports" / "policy_latest.mrlpolicy";
}

void ReinforcementLearningScreen::loadDrills() {
    m_drillIds.clear();
    m_drillNames.clear();
    m_drillPhases.clear();
    m_futureNames.clear();
    m_futurePhases.clear();
    const std::filesystem::path path = taskPath();
    if (path.empty() || !std::filesystem::exists(path)) return;
    if (TrackingViewer3D::isTrackingTask(path)) {
        // O rastreador: o lider na velocidade da tela, num dos 8 rumos (os do portao).
        for (std::size_t i = 0; i < TrackingViewer3D::HeadingNames.size(); ++i) {
            m_drillIds.push_back("rumo" + std::to_string(i));
            m_drillNames.push_back(TrackingViewer3D::HeadingNames[i]);
            m_drillPhases.push_back(i == 0 ? 1 : 2);
        }
    } else if (LocomotionViewer3D::isLocomotionTask(path)) {
        for (const LocomotionViewer3D::Drill& drill : LocomotionViewer3D::drillsOf(path)) {
            m_drillIds.push_back(drill.id);
            m_drillNames.push_back(drill.name);
            m_drillPhases.push_back(drill.phase);
        }
        // A escada completa (a ultima tarefa): o que ainda nao esta no treino fica apagado.
        const std::filesystem::path ladder = m_options.sourceRoot / "assets" / "rl" / "tasks" / "locomocao_f3f.json";
        if (std::filesystem::exists(ladder)) {
            for (const LocomotionViewer3D::Drill& drill : LocomotionViewer3D::drillsOf(ladder)) {
                if (std::find(m_drillIds.begin(), m_drillIds.end(), drill.id) != m_drillIds.end()) continue;
                m_futureNames.push_back(drill.name);
                m_futurePhases.push_back(drill.phase);
            }
        }
    } else try {
        const BodyTask3D task = loadBodyTask3D(path);
        for (const BodyDrill3D& drill : task.drills) {
            m_drillIds.push_back(drill.id);
            m_drillNames.push_back(drill.displayName.empty() ? drill.id : drill.displayName);
            m_drillPhases.push_back(0);
        }
    } catch (const std::exception& error) {
        m_viewerError = error.what();
    }
    if (std::find(m_drillIds.begin(), m_drillIds.end(), m_viewerDrill) == m_drillIds.end())
        m_viewerDrill = m_drillIds.empty() ? std::string() : m_drillIds.front();
}

std::vector<std::string> ReinforcementLearningScreen::drills() const { return m_drillIds; }

void ReinforcementLearningScreen::showDrill(const std::string& drill) {
    if (drill == m_viewerDrill && hasViewer()) return;
    m_viewerDrill = drill;
    resetViewer();
}

void ReinforcementLearningScreen::resetViewer() {
    clearReplay();
    m_viewer.reset();
    m_locomotion.reset();
    m_policy.reset();
    m_policyPath.clear();
    m_viewerError.clear();
    m_dirty = true;
}

void ReinforcementLearningScreen::ensureViewer() {
    const std::string run = activeRun();
    if (run != m_viewerRun) {
        // Outro treino: a tarefa, os exercicios e o mundo dele.
        m_viewerRun = run;
        m_viewerDrill.clear();
        resetViewer();
        loadDrills();
    }
    if (hasViewer() || !m_viewerError.empty() || run.empty()) return;
    const std::filesystem::path task = taskPath();
    const bool tracking = !task.empty() && TrackingViewer3D::isTrackingTask(task);
    if (tracking || (!task.empty() && LocomotionViewer3D::isLocomotionTask(task))) {
        try {
            m_viewerTask = task;
            const auto seed = static_cast<std::uint64_t>(std::time(nullptr));
            if (tracking) m_locomotion = std::make_unique<TrackingViewer3D>(m_options.sourceRoot / "assets", seed,
                    TrackingViewer3D::headingOf(m_viewerDrill));
            else
                m_locomotion = std::make_unique<LocomotionViewer3D>(task, m_options.sourceRoot / "assets", m_viewerDrill,
                    seed);
            m_colliders = buildRagdollColliderWireframe3D(m_locomotion->profile());
            applyManual();
            reloadPolicy(true);
        } catch (const std::exception& error) {
            m_locomotion.reset();
            m_viewerError = error.what();
        }
        return;
    }
    try {
        BodyBatchSettings3D settings;
        settings.taskPath = taskPath();
        if (settings.taskPath.empty()) return;
        m_viewerTask = settings.taskPath;
        settings.environmentCount = 1;
        settings.environmentsPerScene = 1;
        settings.workerThreads = 1;
        settings.seed = static_cast<std::uint64_t>(std::time(nullptr));
        settings.evaluation = true;
        settings.evaluationEpisodeSeconds = 600.0f;
        // O visualizador mostra a fisica do jogo.
        settings.solverPositionIterations = 20;
        settings.solverVelocityIterations = 20;
        settings.drill = m_viewerDrill;
        m_viewer = std::make_unique<BodyTrainingBatch3D>(settings);
        m_action.assign(m_viewer->actionSize(), 0.0f);
        m_colliders = buildRagdollColliderWireframe3D(m_viewer->body().profile);
        applyManual();
        reloadPolicy(true);
    } catch (const std::exception& error) {
        m_viewer.reset();
        m_viewerError = error.what();
    }
}

void ReinforcementLearningScreen::reloadPolicy(bool force) {
    if (!hasViewer()) return;
    const std::filesystem::path path = policyPath();
    std::error_code code;
    if (!std::filesystem::exists(path, code)) {
        m_policy.reset();
        if (m_locomotion) m_locomotion->clearPolicy();
        m_policyText = "sem política ainda";
        return;
    }
    const auto write = std::filesystem::last_write_time(path, code);
    if (!force && write == m_policyWrite && path == m_policyPath) return;
    m_policyWrite = write;
    m_policyPath = path;
    if (m_locomotion) {
        m_locomotion->setPolicy(path, m_policyText);
        return;
    }
    try {
        auto policy = std::make_unique<PolicyRuntime3D>(path);
        // A politica do Corpo le a entrada do contrato inteira, na mesma ordem.
        const auto& channels = m_viewer->observationLayout().channels;
        const std::vector<std::string>& names = policy->info().observationNames;
        bool same = policy->observationSize() == channels.size() && policy->actionSize() == m_viewer->actionSize();
        for (std::size_t i = 0; same && i < names.size(); ++i) same = names[i] == channels[i].name;
        if (!same) {
            m_policy.reset();
            m_policyText = "política de outro contrato";
            return;
        }
        m_policy = std::move(policy);
        m_policyText = "rodada " + std::to_string(m_policy->info().iteration);
    } catch (const std::exception& error) {
        m_policy.reset();
        m_policyText = error.what();
    }
}

void ReinforcementLearningScreen::setManualSpeed(std::optional<float> metersPerSecond) {
    m_manual = metersPerSecond.has_value();
    if (metersPerSecond) m_manualSpeed = std::clamp(*metersPerSecond, 0.0f, MaximumManualSpeed);
    applyManual();
}

void ReinforcementLearningScreen::applyManual() {
    if (m_locomotion) {
        m_locomotion->setManualSpeed(m_manual ? std::optional<float>(m_manualSpeed) : std::nullopt);
        return;
    }
    if (!m_viewer) return;
    if (!m_manual) {
        m_viewer->setManualControl(0, nullptr);
        return;
    }
    ControlInput3D input;
    input.stickForward = m_manualSpeed > 0.0f ? 1.0f : 0.0f;
    input.speed = m_manualSpeed;
    m_viewer->setManualControl(0, &input);
}

void ReinforcementLearningScreen::stepOnce() {
    if (m_locomotion) {
        m_locomotion->step();
        return;
    }
    const std::span<const float> observation = m_viewer->observations();
    if (m_policy) m_policy->evaluate(observation, m_action);
    else std::fill(m_action.begin(), m_action.end(), 0.0f);
    // A acao executada e a media (sem ruido, como no jogo).
    m_viewer->step(m_action, m_action);
}

void ReinforcementLearningScreen::stepViewer(float deltaSeconds) {
    if (!hasViewer()) return;
    if (m_replay.empty()) recordFrame();
    if (!m_paused) {
        const float step = viewerStepSeconds();
        if (m_playhead >= 0.0f) {
            // Tocando a gravacao (depois de voltar): no fim dela, segue ao vivo.
            m_playhead += deltaSeconds * m_playbackSpeed / step;
            if (m_playhead >= static_cast<float>(m_replay.size() - 1)) {
                m_playhead = -1.0f;
                m_accumulator = 0.0f;
            }
        } else {
            m_accumulator += deltaSeconds * m_playbackSpeed;
            int steps = 0;
            while (m_accumulator >= step && steps < MaximumStepsPerFrame) {
                stepOnce();
                recordFrame();
                m_accumulator -= step;
                ++steps;
            }
            if (m_accumulator > step * MaximumStepsPerFrame) m_accumulator = 0.0f;
        }
    }
    updateShown();
}

void ReinforcementLearningScreen::advanceViewer(int decisions) {
    ensureViewer();
    if (!hasViewer()) return;
    if (m_replay.empty()) recordFrame();
    for (int i = 0; i < decisions; ++i) {
        stepOnce();
        recordFrame();
    }
    m_playhead = -1.0f;
    m_accumulator = 0.0f;
    updateShown();
    m_dirty = true;
}

void ReinforcementLearningScreen::recordFrame() {
    const BodyWorldView3D view = liveView();
    if (!view.state) return;
    m_replay.push_back({ *view.state, view });
    m_replay.back().view.state = nullptr;
    // (Com a gravacao cheia o indice do ultimo quadro nao muda: o conteudo sim, e o mostrado tem de ser refeito.)
    m_shownHead = -1.0f;
    if (m_replay.size() > ReplayFrames) {
        m_replay.pop_front();
        if (m_playhead >= 0.0f) m_playhead = std::max(0.0f, m_playhead - 1.0f);
    }
}

void ReinforcementLearningScreen::clearReplay() {
    m_replay.clear();
    m_playhead = -1.0f;
    m_shownHead = -1.0f;
    m_shownValid = false;
}

float ReinforcementLearningScreen::liveHead() const {
    // Ao vivo em camera lenta, entre os dois ultimos quadros (uma decisao atras, sem pulos); em 1x, o ultimo.
    const float last = static_cast<float>(m_replay.size()) - 1.0f;
    if (m_playbackSpeed >= 1.0f || m_replay.size() < 2) return last;
    return last - 1.0f + std::clamp(m_accumulator / viewerStepSeconds(), 0.0f, 1.0f);
}

void ReinforcementLearningScreen::updateShown() {
    if (m_replay.empty()) {
        m_shownValid = false;
        return;
    }
    const float head = std::clamp(m_playhead >= 0.0f ? m_playhead : liveHead(), 0.0f,
        static_cast<float>(m_replay.size() - 1));
    if (m_shownValid && head == m_shownHead) return;
    const std::size_t i = static_cast<std::size_t>(head);
    const ReplayFrame& a = m_replay[i];
    const ReplayFrame& b = m_replay[std::min(i + 1, m_replay.size() - 1)];
    float t = head - static_cast<float>(i);
    // Um recomeco do episodio entre os dois: sem misturar as poses.
    if (!a.state.links.empty() && !b.state.links.empty()
        && (a.state.links[0].position - b.state.links[0].position).length() > 0.5f)
        t = std::round(t);
    m_shownState = t < 0.5f ? a.state : b.state;
    m_shownView = t < 0.5f ? a.view : b.view;
    for (std::size_t k = 0; k < m_shownState.links.size() && k < a.state.links.size() && k < b.state.links.size(); ++k) {
        m_shownState.links[k].position = a.state.links[k].position + (b.state.links[k].position - a.state.links[k].position) * t;
        m_shownState.links[k].orientation = blendQuaternion(a.state.links[k].orientation, b.state.links[k].orientation, t);
    }
    m_shownView.origin = a.view.origin + (b.view.origin - a.view.origin) * t;
    m_shownHead = head;
    m_shownValid = true;
    m_dirty = true;
}

float ReinforcementLearningScreen::viewerRequestedSpeed() const {
    if (!hasViewer()) return 0.0f;
    const Vec3 wanted = viewerView().command.moveVelocity;
    return std::hypot(wanted.x, wanted.y);
}

float ReinforcementLearningScreen::viewerSpeed() const {
    if (!hasViewer()) return 0.0f;
    const Vec3 velocity = viewerView().velocity;
    return std::hypot(velocity.x, velocity.y);
}

void ReinforcementLearningScreen::push(int direction, float newtons) {
    if (!hasViewer()) return;
    const BodyWorldView3D view = viewerView();
    if (!view.state) return;
    // No referencial do peito: frente, tras, esquerda, direita.
    const Vec3 forward { std::cos(view.chestHeading), std::sin(view.chestHeading), 0.0f };
    const Vec3 left { -forward.y, forward.x, 0.0f };
    const std::array<Vec3, 4> directions { forward, forward * -1.0f, left, left * -1.0f };
    const Vec3 force = directions[static_cast<std::size_t>(std::clamp(direction, 0, 3))] * newtons;
    if (m_locomotion) m_locomotion->push(force, 0.2f);
    else m_viewer->push(0, m_viewer->body().upperChest, force, 0.2f);
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
    // O status do treino chega pela thread da ponte.
    for (int i = 0; i < 40 && !m_link->status().known; ++i) std::this_thread::sleep_for(std::chrono::milliseconds(50));
    ensureViewer();
    refreshHistory();
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
    m_historyRun = activeRun();
    m_historyPending = std::async(std::launch::async,
        [database = m_paths.database, directory = m_paths.run(m_historyRun), run = m_historyRun,
            since = m_chartsExpanded ? ChartWindows[static_cast<std::size_t>(m_chartWindow)].second : 0.0,
            points = std::size_t { m_chartsExpanded ? 1200u : 400u }] {
            // O rastreador grava os seus arquivos, nao o rl.db.
            if (TrackingViewer3D::isTrackingTask(directory / "tarefa.json"))
                return TrackingViewer3D::readHistory(directory, points, since);
            TrainingHistory3D history = readTrainingHistory3D(database, run, since, points);
            // (Um treino parado ha mais que a janela: o fim dele.)
            if (since > 0.0 && history.iterations.empty()) history = readTrainingHistory3D(database, run, 0.0, points);
            return history;
        });
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
    if (m_historyPending || m_historyRun != activeRun() || (m_historyWatch -= deltaSeconds) <= 0.0f) {
        if (!m_historyPending) m_historyWatch = 5.0f;
        refreshHistory();
    }
    if (m_noticeSeconds > 0.0f) m_noticeSeconds -= deltaSeconds;
}

bool ReinforcementLearningScreen::render(int width, int height) {
    if (!hasViewer() || !m_model || !m_skeleton) return false;
    if (!m_dirty && !m_image.empty() && m_image.width() == width && m_image.height() == height) return false;
    const BodyWorldView3D view = viewerView();
    if (!view.state) return false;
    const RagdollState3D& state = *view.state;
    const RagdollProfile3D& profile = viewerProfile();
    std::vector<Vec3> positions(state.links.size());
    std::vector<Quaternion> orientations(state.links.size());
    for (std::size_t i = 0; i < state.links.size(); ++i) {
        positions[i] = state.links[i].position;
        orientations[i] = state.links[i].orientation;
    }
    // O mundo fica preso na origem do episodio (e nao na pelve): andando, o chao
    // corre sob os pes e a camera segue o jogador.
    const SkeletonPose& pose = m_skeleton->update(positions, orientations, view.origin, 0.0f, viewerStepSeconds());
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
    Vec3 center;
    for (std::size_t i = 0; i < state.links.size(); ++i) center += positions[i] * profile.links[i].massFraction;
    if (m_showCenterOfMass) {
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

    std::vector<InspectionMarker> markers;
    std::vector<InspectionArrow> arrows;
    // O ponto do piloto (a estrela): laranja quando e para parar nele.
    if (view.hasTarget) {
        InspectionMarker target;
        target.center = m_skeleton->toSkeleton({ view.target.x, view.target.y, 0.0f });
        target.radius = 0.35f;
        if (view.targetStop) target.color = rgb(245, 160, 40);
        markers.push_back(target);
    }
    // O pedido (azul) e para onde ele vai de verdade (amarelo): as velocidades
    // no chao, 0,35 s a frente.
    const Vec3 wanted = view.command.moveVelocity;
    if (std::hypot(wanted.x, wanted.y) > 0.05f) {
        InspectionArrow asked;
        asked.from = m_skeleton->toSkeleton({ center.x, center.y, 0.02f });
        asked.to = m_skeleton->toSkeleton({ center.x + wanted.x * 0.35f, center.y + wanted.y * 0.35f, 0.02f });
        asked.color = rgb(110, 190, 255, 0.8f);
        asked.lineWidthPixels = 2.0f;
        arrows.push_back(asked);
    }
    if (std::hypot(view.velocity.x, view.velocity.y) > 0.2f) {
        InspectionArrow going;
        going.from = m_skeleton->toSkeleton({ center.x, center.y, 0.03f });
        going.to = m_skeleton->toSkeleton(
            { center.x + view.velocity.x * 0.35f, center.y + view.velocity.y * 0.35f, 0.03f });
        going.color = rgb(245, 214, 60, 0.7f);
        going.lineWidthPixels = 2.0f;
        arrows.push_back(going);
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
    InspectionStyle style;
    style.surface = m_display == Display::Full ? SurfaceMode::Solid : SurfaceMode::Hidden;
    style.skeleton = m_display == Display::Skeleton;
    style.joints = m_display == Display::Skeleton;
    style.hud = false;
    style.gizmo = false;
    style.scaleBar = false;
    // O chao largo com passo de 0,5 m: andando, ele sai da grade fina logo. Ate
    // 24 m em volta (a grade segue a camera): a de 70 m tinha 560 linhas de
    // 140 m e custava 26 dos 40 ms do quadro (o custo cresce com o quadrado do
    // alcance); alem de 24 m as linhas ja se juntam no horizonte.
    style.gridExtent = 24.0f;
    style.gridStep = 0.5f;
    style.supersample = 1;
    OrbitCamera camera = m_camera;
    const Vec3 pelvis = m_skeleton->toSkeleton(positions.front());
    camera.target = { pelvis.x, pelvis.y, pelvis.z - 0.04f };
    // As vistas giram com o rumo pedido (a camera do piloto, que nao balanca
    // com o passo): "Frente" o ve de frente andando, "Lado" pela direita.
    {
        const float heading = view.command.faceHeading;
        const Vec3 base { positions.front().x, positions.front().y, 0.0f };
        const Vec3 aim = m_skeleton->toSkeleton(base);
        const Vec3 ahead = m_skeleton->toSkeleton(base + Vec3 { std::cos(heading), std::sin(heading), 0.0f });
        camera.yaw = std::atan2(ahead.y - aim.y, ahead.x - aim.x) + m_camera.yaw;
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

void ReinforcementLearningScreen::frame(float yawDegrees, float pitchDegrees) {
    m_camera.yaw = yawDegrees * DegreesToRadians;
    m_camera.pitch = pitchDegrees * DegreesToRadians;
    m_camera.distance = 4.4f;
    m_dirty = true;
}

void ReinforcementLearningScreen::notify(std::string text) {
    m_notice = std::move(text);
    m_noticeSeconds = 4.0f;
}

void ReinforcementLearningScreen::draw(CanvasTarget& canvas, float uiScale) {
    m_uiScale = uiScale;
    beginFullscreenPanel("MatterEngineReinforcementLearning");
    const ImGuiIO& io = ImGui::GetIO();
    ImDrawList* drawList = ImGui::GetWindowDrawList();
    drawWorkbenchBackdrop(drawList, io.DisplaySize, uiScale);
    const TrainerStatus3D status = m_link->status();
    drawWorkbenchHeader(drawList, io.DisplaySize, uiScale, "TREINO RL", stateLabel(status.state).label);

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
    if (m_chartsExpanded) {
        drawCommands();
        drawCharts(ImGui::GetContentRegionAvail().y - ui(30.0f));
    } else {
        drawViewport(canvas, charts + ui(78.0f));
        drawCommands();
        drawCharts(charts);
    }
    drawFooter();
    ImGui::EndChild();
    ImGui::End();
}

void ReinforcementLearningScreen::drawSidebar(float height) {
    drawTrainer();
    drawDrills();
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
    // O treino mostrado: com um treinador aberto, o dele.
    const std::string shown = activeRun();
    ImGui::BeginDisabled(status.alive);
    ImGui::SetNextItemWidth(-1.0f);
    if (ImGui::BeginCombo("##RLRun", shown.empty() ? "—" : shown.c_str())) {
        scanRuns();
        for (const std::string& run : m_runs)
            if (ImGui::Selectable(run.c_str(), run == shown)) selectRun(run);
        ImGui::EndCombo();
    }
    ImGui::EndDisabled();
    tooltip(status.alive ? "O treino aberto no PC (só um por vez)." : "O treino mostrado e que \"Continuar treino\" abre.");
    if (status.known) {
        infoRow("Tarefa", status.taskName.empty() ? status.task : status.taskName);
        infoRow("Rodada", std::to_string(status.iteration));
        infoRow("Decisões", decimal(static_cast<double>(status.steps) / 1e6, 1) + " M");
        if (status.alive) infoRow("Velocidade", decimal(status.sps, 0) + " /s");
        infoRow("Treinado", duration(status.trainSeconds));
        if (status.lastFallFraction >= 0.0) infoRow("Quedas", decimal(status.lastFallFraction * 100.0, 0) + "%");
        tooltip("A fração dos episódios da última rodada do treino que acabaram em queda.");
        if (status.styleBeta >= 0.0) {
            infoRow("Estilo", "peso " + decimal(status.styleBeta, 2) + " · " + decimal(status.lastStyle, 3));
            tooltip("O peso do juiz de estilo (beta) e a nota média dele por decisão na última rodada.");
        }
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
    if (!status.alive) {
        ImGui::BeginDisabled(status.state == "abrindo" || !status.known || status.task.empty());
        const std::string label = UI::FontAwesome::label(UI::FontAwesome::Play, "Continuar treino");
        if (ImGui::Button(label.c_str(), { -1.0f, ui(36.0f) })) {
            std::string error;
            if (!m_link->start(status.task, m_mode, &error)) notify(error);
        }
        ImGui::EndDisabled();
        tooltip("Abre o treino em segundo plano, do último checkpoint. Ele continua com o Matter Engine fechado.");
        return;
    }
    if (status.pid > 0) {
        // O rastreador (processo proprio): parar salva e fecha; nao tem pausa nem avaliacao pelo canal.
        if (ImGui::Button(UI::FontAwesome::label(UI::FontAwesome::Stop, "Parar").c_str(), { half, ui(34.0f) }))
            m_link->send("parar");
        tooltip("Termina a rodada, salva e fecha o treinador. \"Continuar treino\" volta do mesmo ponto.");
        ImGui::SameLine();
        if (ImGui::Button(UI::FontAwesome::label(UI::FontAwesome::FloppyDisk, "Salvar").c_str(), { half, ui(34.0f) })) {
            m_link->send("salvar");
            notify("Checkpoint pedido");
        }
        tooltip("Grava um checkpoint no fim da rodada.");
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
    tooltip("Grava um checkpoint agora.");
    ImGui::SameLine();
    if (ImGui::Button(UI::FontAwesome::label(UI::FontAwesome::Check, "Avaliar").c_str(), { half, ui(30.0f) })) {
        m_link->send("avaliar");
        notify("Avaliação pedida");
    }
    tooltip("Roda a avaliação dos exercícios agora (nos dois solvers).");
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

void ReinforcementLearningScreen::drawDrills() {
    if (m_drillIds.empty()) return;
    ImGui::Dummy({ 1.0f, ui(8.0f) });
    panelHeader("EXERCÍCIOS");
    ImGui::Dummy({ 1.0f, ui(2.0f) });
    for (std::size_t i = 0; i < m_drillIds.size(); ++i) {
        const std::string label = UI::FontAwesome::label(UI::FontAwesome::Crosshairs, m_drillNames[i].c_str())
            + "##drill" + m_drillIds[i];
        const int phase = i < m_drillPhases.size() ? m_drillPhases[i] : 0;
        if (phaseButton(label.c_str(), m_drillIds[i] == m_viewerDrill, { -1.0f, ui(30.0f) }, phase))
            showDrill(m_drillIds[i]);
        const std::string hint = (phase > 0 ? "Fase " + std::to_string(phase) + ": m" : std::string("M"))
            + "ostra este exercício no mundo (o treino sorteia todos pelos pesos).";
        tooltip(hint.c_str());
    }
    for (std::size_t i = 0; i < m_futureNames.size(); ++i) {
        const std::string label = UI::FontAwesome::label(UI::FontAwesome::Crosshairs, m_futureNames[i].c_str())
            + "##future" + std::to_string(i);
        ImGui::BeginDisabled();
        phaseButton(label.c_str(), false, { -1.0f, ui(30.0f) }, m_futurePhases[i]);
        ImGui::EndDisabled();
        tooltip("Ainda não está no treino: entra quando a fase dele abrir.");
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
    infoRow("Rodada", std::to_string(status.evaluationIteration));
    std::size_t shown = 0;
    for (const TrainerStatus3D::DrillEvaluation& entry : status.evaluation) {
        if (entry.solver != "jogo") continue;
        if (++shown > 1 || m_drillIds.size() > 1) {
            const auto it = std::find(m_drillIds.begin(), m_drillIds.end(), entry.drill);
            const std::string name = it == m_drillIds.end() ? entry.drill
                : m_drillNames[static_cast<std::size_t>(it - m_drillIds.begin())];
            ImGui::TextColored(InfoText, "%s", name.c_str());
        }
        infoRow("Sem cair", decimal((1.0 - entry.falls) * 100.0, 0) + "%");
        tooltip("Os mundos da avaliação que não caíram em 20 s, sem ruído e com a física do jogo (20/20).");
        if (entry.velocityError >= 0.0) {
            infoRow("Erro vel.", decimal(entry.velocityError, 2) + " m/s");
            tooltip("A mediana da diferença entre a velocidade pedida e a dele.");
        }
        if (entry.usefulSteps >= 0.0) {
            infoRow("Passos úteis", decimal(entry.usefulSteps, 2) + " /s");
            tooltip("Os toques de um pé que ficou ao menos 0,12 s no ar e andou ao menos 10 cm.");
        }
        if (entry.slip >= 0.0) {
            infoRow("Escorregar", decimal(entry.slip, 3) + " m/s");
            tooltip("A velocidade média dos pés no chão (patinar sobe).");
        }
    }
}

void ReinforcementLearningScreen::drawToolbar() {
    struct View {
        const char* label;
        float yaw;
        float pitch;
    };
    constexpr std::array<View, 4> Views { { { "Frente", 0.0f, 6.0f }, { "Lado", 90.0f, 6.0f }, { "Costas", 180.0f, 6.0f },
        { "3/4", 35.0f, 10.0f } } };
    for (const View& view : Views) {
        if (ImGui::Button(view.label, { ui(70.0f), ui(30.0f) })) frame(view.yaw, view.pitch);
        ImGui::SameLine();
    }
    if (ImGui::Checkbox("Centro de massa", &m_showCenterOfMass)) m_dirty = true;
    tooltip("O centro de massa (amarelo) e a projeção dele no chão: dentro dos pés, ele está equilibrado.");
    ImGui::SameLine();
    if (ImGui::Checkbox("Contatos", &m_showContacts)) m_dirty = true;
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
    if (ImGui::Button(UI::FontAwesome::label(UI::FontAwesome::RotateLeft, "Recomeçar").c_str(), { ui(120.0f), ui(30.0f) })
        && hasViewer()) {
        const auto seed = static_cast<std::uint64_t>(std::time(nullptr));
        if (m_locomotion) m_locomotion->reset(seed);
        else m_viewer->reset(seed);
        applyManual();
        clearReplay();
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
    const int width = std::max(16, static_cast<int>(std::lround(size.x * scaleX * m_renderScale)));
    const int height = std::max(16, static_cast<int>(std::lround(size.y * scaleY * m_renderScale)));
    const char* waiting = !m_viewerError.empty() ? m_viewerError.c_str()
        : !m_modelError.empty()                  ? m_modelError.c_str()
        : activeRun().empty()                    ? "SEM TREINO"
        : !m_model || !hasViewer()              ? "CARREGANDO"
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
    if (hovered && ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left)) frame(90.0f, 6.0f);
    const auto renderBegin = std::chrono::steady_clock::now();
    const bool rendered = render(width, height);
    if (rendered) {
        const float milliseconds = std::chrono::duration<float, std::milli>(std::chrono::steady_clock::now() - renderBegin).count();
        m_renderMilliseconds = m_renderMilliseconds <= 0.0f ? milliseconds : 0.85f * m_renderMilliseconds + 0.15f * milliseconds;
        // (Degraus de escala, no maximo um a cada ~0,5 s: nao pisca.)
        constexpr std::array<float, 4> Steps { 1.0f, 0.85f, 0.7f, 0.55f };
        if (++m_renderScaleHold > 15) {
            std::size_t at = 0;
            for (std::size_t i = 0; i < Steps.size(); ++i)
                if (std::abs(Steps[i] - m_renderScale) < 0.01f) at = i;
            if (m_renderMilliseconds > 28.0f && at + 1 < Steps.size()) {
                m_renderScale = Steps[at + 1];
                m_renderScaleHold = 0;
                m_renderMilliseconds = 0.0f;
                m_dirty = true;
            } else if (m_renderMilliseconds < 14.0f && at > 0) {
                m_renderScale = Steps[at - 1];
                m_renderScaleHold = 0;
                m_renderMilliseconds = 0.0f;
                m_dirty = true;
            }
        }
    }
    const std::uint64_t id = canvas.present(m_image, rendered);
    drawList->AddImage(static_cast<ImTextureID>(id), origin, end);
    // A politica que esta no corpo (canto de cima), o pedido e a velocidade dele.
    const std::string label = m_policyText + "\npedido " + decimal(viewerRequestedSpeed(), 1) + " m/s · ele "
        + decimal(viewerSpeed(), 1) + " m/s";
    drawList->AddText({ origin.x + ui(10.0f), origin.y + ui(8.0f) }, UiMuted, label.c_str());
}

void ReinforcementLearningScreen::drawCommands() {
    ImGui::Dummy({ 1.0f, ui(2.0f) });
    // A reproducao do visualizador: pausar, a camera lenta, quadro a quadro e voltar na gravacao (o treino segue).
    ImGui::BeginDisabled(!hasViewer());
    const std::string pause = m_paused ? UI::FontAwesome::label(UI::FontAwesome::Play, "Continuar")
                                       : UI::FontAwesome::label(UI::FontAwesome::Pause, "Pausar");
    if (ImGui::Button(pause.c_str(), { ui(120.0f), ui(28.0f) })) {
        // Pausando ao vivo, a gravacao fica no quadro mostrado.
        if (!m_paused && m_playhead < 0.0f && !m_replay.empty()) m_playhead = liveHead();
        m_paused = !m_paused;
    }
    tooltip("Pausa só o visualizador (o treino segue).");
    ImGui::SameLine(0.0f, ui(16.0f));
    ImGui::PushID("RLPlaybackSpeed");
    for (std::size_t i = 0; i < PlaybackSpeeds.size(); ++i) {
        if (i > 0) ImGui::SameLine(0.0f, ui(4.0f));
        const bool on = std::abs(PlaybackSpeeds[i] - m_playbackSpeed) < 1e-4f;
        char label[16];
        std::snprintf(label, sizeof label, "%gx", static_cast<double>(PlaybackSpeeds[i]));
        if (on) ImGui::PushStyleColor(ImGuiCol_Button, ImGui::GetStyleColorVec4(ImGuiCol_ButtonActive));
        if (ImGui::Button(label, { ui(56.0f), ui(28.0f) })) {
            m_playbackSpeed = PlaybackSpeeds[i];
            m_shownValid = false;
        }
        if (on) ImGui::PopStyleColor();
    }
    ImGui::PopID();
    tooltip("A velocidade do visualizador (câmera lenta).");
    ImGui::SameLine(0.0f, ui(16.0f));
    const float last = m_replay.empty() ? 0.0f : static_cast<float>(m_replay.size() - 1);
    const float head = m_playhead >= 0.0f ? m_playhead : (m_replay.empty() ? 0.0f : liveHead());
    const auto stepTo = [&](float target) {
        m_paused = true;
        m_playhead = std::clamp(target, 0.0f, last);
    };
    if (ImGui::Button("<##RLFrameBack", { ui(32.0f), ui(28.0f) })) stepTo(std::ceil(head) - 1.0f);
    tooltip("Um quadro para trás.");
    ImGui::SameLine(0.0f, ui(4.0f));
    if (ImGui::Button(">##RLFrameForward", { ui(32.0f), ui(28.0f) })) {
        if (std::floor(head) + 1.0f <= last) stepTo(std::floor(head) + 1.0f);
        else {
            // No fim da gravacao: simula a proxima decisao.
            stepOnce();
            recordFrame();
            m_paused = true;
            m_playhead = static_cast<float>(m_replay.size() - 1);
        }
    }
    tooltip("Um quadro para frente.");
    ImGui::SameLine(0.0f, ui(8.0f));
    ImGui::SetNextItemWidth(ui(420.0f));
    float scrub = head;
    char behind[48];
    // (O primeiro quadro do app chega antes do visualizador.)
    const float step = hasViewer() ? viewerStepSeconds() : 1.0f / 30.0f;
    std::snprintf(behind, sizeof behind, "-%.2f s", static_cast<double>((last - head) * step));
    if (ImGui::SliderFloat("##RLTimeline", &scrub, 0.0f, std::max(last, 1e-3f), behind)) {
        m_paused = true;
        m_playhead = std::round(scrub);
    }
    tooltip("A gravação dos últimos 30 s do visualizador: arraste para o momento exato (o tempo é o quanto antes de "
            "agora).");
    ImGui::EndDisabled();
    ImGui::SameLine(0.0f, ui(24.0f));
    if (ImGui::Button(UI::FontAwesome::label(UI::FontAwesome::ChartLine, m_chartsExpanded ? "Recolher" : "Expandir").c_str(),
            { ui(120.0f), ui(28.0f) })) {
        m_chartsExpanded = !m_chartsExpanded;
        m_historyWatch = 0.0f;
    }
    tooltip(m_chartsExpanded ? "Volta o visualizador." : "Os gráficos no lugar do visualizador, maiores e com a janela de tempo.");
    if (!m_chartsExpanded) return;
    ImGui::SameLine(0.0f, ui(24.0f));
    ImGui::PushID("RLChartWindow");
    for (int i = 0; i < static_cast<int>(ChartWindows.size()); ++i) {
        if (i > 0) ImGui::SameLine();
        const bool on = i == m_chartWindow;
        if (on) ImGui::PushStyleColor(ImGuiCol_Button, ImGui::GetStyleColorVec4(ImGuiCol_ButtonActive));
        if (ImGui::Button(ChartWindows[static_cast<std::size_t>(i)].first, { ui(70.0f), ui(28.0f) }) && !on) {
            m_chartWindow = i;
            m_historyWatch = 0.0f;
        }
        if (on) ImGui::PopStyleColor();
    }
    ImGui::PopID();
    tooltip("O trecho mais recente do treino nos gráficos (sem tanto treino, desde o começo).");
}

void ReinforcementLearningScreen::drawCharts(float height) {
    ImGui::Dummy({ 1.0f, ui(4.0f) });
    // A avaliacao do exercicio mostrado: a da rede (ou a do jogo, no Corpo) e, de referencia, a do classico.
    bool hasGame = false;
    for (const TrainingEvaluationPoint3D& point : m_history.evaluations) hasGame = hasGame || point.solver == "jogo";
    const std::string primary = hasGame ? "jogo" : "rede";
    std::vector<float> stand, standClassic, seconds, speed;
    for (const TrainingEvaluationPoint3D& point : m_history.evaluations) {
        if (!m_viewerDrill.empty() && point.drill != m_viewerDrill) continue;
        if (point.solver == "classico") standClassic.push_back(static_cast<float>((1.0 - point.falls) * 100.0));
    }
    for (const TrainingIterationPoint3D& point : m_history.iterations) {
        seconds.push_back(static_cast<float>(point.episodeSeconds));
        speed.push_back(static_cast<float>(point.sps));
    }
    // O exercicio mostrado, rodada a rodada, com os episodios das ultimas 12 rodadas juntos (uma rodada tem poucos
    // episodios por exercicio, sozinha ela so oscila): o "sem cair" e a recompensa por segundo (o retorno ja traz a
    // queda como perda, entao cair derruba a curva).
    constexpr std::size_t Window = 12;
    std::vector<float> rate;
    const bool tracking = TrackingViewer3D::isTrackingTask(m_viewerTask);
    if (tracking) {
        // O rastreador: o sem cair do treino rodada a rodada (os episodios que chegaram ao fim sem cair nem ficar
        // para tras; o ultimo portao do rumo mostrado vai no canto) e a nota por decisao (todos os rumos juntos).
        for (const TrainingIterationPoint3D& point : m_history.iterations) {
            stand.push_back(static_cast<float>((1.0 - point.fallFraction) * 100.0));
            rate.push_back(static_cast<float>(point.reward));
        }
    } else {
        struct Entry {
            double episodes, fell, reward, seconds;
        };
        std::vector<Entry> recent;
        for (const TrainingDrillPoint3D& point : m_history.drills) {
            if (point.drill != m_viewerDrill) continue;
            recent.push_back({ point.episodes, point.falls * point.episodes,
                std::isnan(point.episodeReturn) ? 0.0 : point.episodeReturn * point.episodes,
                point.seconds * point.episodes });
            if (recent.size() > Window) recent.erase(recent.begin());
            Entry sum {};
            for (const Entry& entry : recent) {
                sum.episodes += entry.episodes;
                sum.fell += entry.fell;
                sum.reward += entry.reward;
                sum.seconds += entry.seconds;
            }
            if (sum.episodes <= 0.0) continue;
            stand.push_back(static_cast<float>((1.0 - sum.fell / sum.episodes) * 100.0));
            if (sum.seconds > 0.0) rate.push_back(static_cast<float>(sum.reward / sum.seconds));
        }
    }
    const auto range = [](std::initializer_list<const std::vector<float>*> lists, float floorTop, float floorBottom) {
        float lowest = floorBottom, highest = floorTop;
        for (const std::vector<float>* list : lists)
            for (const float v : *list) {
                lowest = std::min(lowest, v);
                highest = std::max(highest, v);
            }
        const float pad = std::max((highest - lowest) * 0.08f, 1e-3f);
        return std::pair<float, float> { lowest - (lowest < floorBottom + 1e-6f && floorBottom == 0.0f ? 0.0f : pad), highest + pad };
    };
    const std::string drillName = [&] {
        const auto it = std::find(m_drillIds.begin(), m_drillIds.end(), m_viewerDrill);
        return it == m_drillIds.end() ? std::string() : m_drillNames[static_cast<std::size_t>(it - m_drillIds.begin())];
    }();
    struct Chart {
        std::string title;
        std::vector<ChartSeries> series;
        float minimum;
        float maximum;
        std::string value;
        const char* unit;
        const char* tooltip;
        const std::vector<double>* times = nullptr;
        // Se cair e melhorar (as quedas, o erro): a cor da variacao no periodo.
        bool lowerIsBetter = false;
    };
    const auto text = [](float v, const char* format) {
        char buffer[32];
        std::snprintf(buffer, sizeof buffer, format, v);
        return std::string(buffer);
    };
    const ImVec4 green { 0.30f, 0.82f, 0.56f, 1.0f }, amber { 0.96f, 0.70f, 0.28f, 1.0f },
        cyan { 0.35f, 0.78f, 0.94f, 1.0f }, violet { 0.70f, 0.56f, 0.96f, 1.0f };
    // A evolucao: a recompensa por segundo do exercicio e, apagado, o melhor nivel que ela ja alcancou no treino.
    const float best = rate.empty() ? 0.0f : *std::max_element(rate.begin(), rate.end());
    const std::vector<float> bestLine(2, best);
    std::string rateValue;
    if (!rate.empty()) {
        rateValue = text(rate.back(), tracking ? "%.3f" : "%.2f/s");
        rateValue += text(rate.back() - rate[rate.size() > 30 ? rate.size() - 31 : 0], tracking ? "  (%+.3f)" : "  (%+.2f)");
    }
    const auto [rateLow, rateHigh] = range({ &rate, &bestLine }, 0.0f, 0.0f);
    // Os horarios de cada ponto (as rodadas; no rastreador, os portoes do sem cair).
    std::vector<double> iterationTimes;
    std::vector<float> falls, bodySpeed, requestedSpeed, velocityError;
    for (const TrainingIterationPoint3D& point : m_history.iterations) {
        iterationTimes.push_back(point.time);
        falls.push_back(static_cast<float>(point.fallFraction * 100.0));
        if (!std::isnan(point.bodySpeed)) bodySpeed.push_back(static_cast<float>(point.bodySpeed));
        if (!std::isnan(point.requestedSpeed)) requestedSpeed.push_back(static_cast<float>(point.requestedSpeed));
        if (!std::isnan(point.velocityError)) velocityError.push_back(static_cast<float>(point.velocityError));
    }
    // O ultimo portao do rumo mostrado (no rastreador, no canto do sem cair).
    std::string gate;
    if (tracking)
        for (const TrainingEvaluationPoint3D& point : m_history.evaluations)
            if (point.drill == m_viewerDrill) gate = drillName + " " + text(static_cast<float>((1.0 - point.falls) * 100.0), "%.0f%%");
    std::string standValue = stand.empty() ? "" : text(stand.back(), tracking ? "treino %.0f%%" : "%.0f%%");
    if (!gate.empty()) standValue += " · teste " + gate;
    std::vector<Chart> charts { {
        { tracking ? std::string("Sem cair (%) · treino")
                   : "Sem cair (%)" + (drillName.empty() ? std::string() : " · " + drillName),
            { { &standClassic, green, true }, { &stand, green, false } }, 0.0f, 100.0f, standValue, "%",
            tracking ? "Os episódios do treino que chegaram ao fim sem cair nem ficar para trás, rodada a rodada. No canto, "
                       "o último teste das 8 direções no rumo escolhido (só as quedas de verdade)."
                     : "A avaliação do exercício mostrado (a rede; o traço apagado é o clássico, sem a rede).",
            tracking ? &iterationTimes : nullptr },
        { tracking ? std::string("Evolução · nota por decisão (todos os rumos)")
                   : "Evolução · recompensa por segundo" + (drillName.empty() ? std::string() : " · " + drillName),
            { { &bestLine, amber, true, "melhor" }, { &rate, amber, false } }, rateLow, rateHigh, rateValue,
            tracking ? "" : "/s",
            "Quanto o exercício mostrado é recompensado por segundo (as últimas 12 rodadas juntas; cair conta como perda, "
            "então falhar derruba a curva). Entre parênteses, a variação em 30 rodadas; o traço apagado é o melhor nível "
            "já alcançado neste treino.",
            tracking ? &iterationTimes : nullptr },
        { "Episódio (s)", { { &seconds, cyan, false } }, 0.0f, range({ &seconds }, 1.0f, 0.0f).second,
            seconds.empty() ? "" : text(seconds.back(), "%.1f s"), "s", "Quanto dura um episódio do treino.",
            &iterationTimes },
        { "Decisões / s", { { &speed, violet, false } }, 0.0f, range({ &speed }, 1000.0f, 0.0f).second,
            speed.empty() ? "" : text(speed.back(), "%.0f"), "", "A velocidade do treino (física + rede).",
            &iterationTimes },
    } };
    if (m_chartsExpanded) {
        // Expandido: tambem as quedas do treino (no rastreador o sem cair ja e o avesso delas) e, no rastreador, a
        // velocidade e o erro dela.
        if (!tracking)
            charts.insert(charts.begin() + 3, { "Quedas no treino (%)", { { &falls, amber, false } }, 0.0f, 100.0f,
                falls.empty() ? "" : text(falls.back(), "%.1f%%"), "%",
                "A fatia dos episódios do treino que acabou em queda (todos os exercícios).", &iterationTimes, true });
        if (!bodySpeed.empty())
            charts.insert(charts.end() - 1, { "Velocidade (m/s) · ele e, apagado, o pedido",
                { { &requestedSpeed, cyan, true, "pedido" }, { &bodySpeed, cyan, false, "ele" } }, 0.0f,
                range({ &bodySpeed, &requestedSpeed }, 1.0f, 0.0f).second, text(bodySpeed.back(), "%.2f m/s"), "m/s",
                "A velocidade média do corpo nos episódios do treino e a pedida pelo líder.", &iterationTimes });
        if (!velocityError.empty())
            charts.insert(charts.end() - 1, { "Erro de velocidade (m/s)", { { &velocityError, green, false } }, 0.0f,
                range({ &velocityError }, 0.5f, 0.0f).second, text(velocityError.back(), "%.3f m/s"), "m/s",
                "O erro médio entre a velocidade do centro de massa e a do líder (menor é melhor).", &iterationTimes,
                true });
    }
    const float spacing = ImGui::GetStyle().ItemSpacing.x;
    if (m_chartsExpanded) {
        // Um embaixo do outro, na largura toda, dividindo a altura.
        const float each = std::max(ui(80.0f), height / static_cast<float>(charts.size()) - ImGui::GetStyle().ItemSpacing.y);
        for (std::size_t i = 0; i < charts.size(); ++i) {
            const Chart& chart = charts[i];
            ImGui::BeginGroup();
            ImGui::TextColored(InfoText, "%s", chart.title.c_str());
            // A variacao no periodo: a media dos primeiros e dos ultimos 5% dos pontos (o ruido nao engana).
            const auto main = std::find_if(chart.series.begin(), chart.series.end(),
                [](const ChartSeries& item) { return !item.reference; });
            if (main != chart.series.end() && main->values->size() >= 2) {
                const std::vector<float>& v = *main->values;
                const std::size_t k = std::max<std::size_t>(1, v.size() / 20);
                double head = 0.0, tail = 0.0;
                for (std::size_t j = 0; j < k; ++j) {
                    head += v[j];
                    tail += v[v.size() - 1 - j];
                }
                head /= static_cast<double>(k);
                tail /= static_cast<double>(k);
                const double delta = tail - head;
                char line[128];
                // (Num grafico em %, a variacao em pontos; a relativa de quase zero nao diz nada.)
                if (std::abs(head) > 1e-9 && std::string_view(chart.unit) != "%")
                    std::snprintf(line, sizeof line, "no período: %.3g → %.3g %s  (%+.3g, %+.1f%%)", head, tail, chart.unit,
                        delta, 100.0 * delta / std::abs(head));
                else
                    std::snprintf(line, sizeof line, "no período: %.3g → %.3g %s  (%+.3g)", head, tail, chart.unit, delta);
                const bool flat = std::abs(delta) <= 0.01 * std::max(std::abs(head), 1e-6);
                const bool better = chart.lowerIsBetter ? delta < 0.0 : delta > 0.0;
                ImGui::SameLine(0.0f, ui(18.0f));
                ImGui::TextColored(flat ? ImVec4 { 0.55f, 0.62f, 0.70f, 1.0f }
                        : better         ? ImVec4 { 0.30f, 0.82f, 0.56f, 1.0f }
                                         : ImVec4 { 0.95f, 0.42f, 0.42f, 1.0f },
                    "%s", line);
            }
            ImGui::PushID(static_cast<int>(i));
            lineChart("##chart", chart.series, chart.minimum, chart.maximum,
                { ImGui::GetContentRegionAvail().x, each - ImGui::GetTextLineHeightWithSpacing() }, chart.value.c_str(),
                chart.unit, chart.times);
            ImGui::PopID();
            ImGui::EndGroup();
            tooltip(chart.tooltip);
        }
        return;
    }
    const float width = (ImGui::GetContentRegionAvail().x - 3.0f * spacing) / 4.0f;
    const float rowTop = ImGui::GetCursorPosY();
    for (std::size_t i = 0; i < charts.size(); ++i) {
        if (i > 0) ImGui::SameLine();
        ImGui::SetCursorPosY(rowTop);
        const Chart& chart = charts[i];
        ImGui::BeginGroup();
        ImGui::TextColored(InfoText, "%s", chart.title.c_str());
        ImGui::PushID(static_cast<int>(i));
        lineChart("##chart", chart.series, chart.minimum, chart.maximum,
            { width, std::max(height - ImGui::GetTextLineHeightWithSpacing(), ui(40.0f)) }, chart.value.c_str(),
            chart.unit);
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
