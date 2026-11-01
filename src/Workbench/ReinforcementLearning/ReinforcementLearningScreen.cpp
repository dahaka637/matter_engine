#include "Workbench/ReinforcementLearning/MotorViewer3D.hpp"
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
#include <limits>
#include <optional>
#include <ctime>
#include <fstream>
#include <sstream>
#include <thread>
#include <string_view>
#include <utility>

namespace MatterEngine::Workbench {

using namespace Modeling;

namespace {

// O exercicio do levantar do chao: o grafico dele mostra quantos levantaram.
constexpr const char* GetUpDrill = "levantar";
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

// As chaves do log da locomocao na rede do levantar, na ordem de TrackingViewer3D::LocomotionDrills: o primeiro grafico
// (no treino e no teste; em %, menos o erro do rumo do giro, em graus) e o sem cair (no treino e no teste). O parado
// treina de pe com empurroes e testa sem.
struct LocomotionLogKeys3D {
    const char* main;
    const char* mainTest;
    const char* up;
    const char* upTest;
    bool percent;
};
constexpr std::array<LocomotionLogKeys3D, 12> LocomotionLogKeys { {
    { "de_pe_pose", "parado_pose", "de_pe_sem_cair", "parado_sem_cair", true },
    { "giro_rumo", "giro_rumo", "giro_sem_cair", "giro_sem_cair", false },
    { "loco_frente_razao", "loco_frente_razao", "loco_frente_sem_cair", "loco_frente_sem_cair", true },
    { "loco_tras_razao", "loco_tras_razao", "loco_tras_sem_cair", "loco_tras_sem_cair", true },
    { "loco_direita_razao", "loco_direita_razao", "loco_direita_sem_cair", "loco_direita_sem_cair", true },
    { "loco_esquerda_razao", "loco_esquerda_razao", "loco_esquerda_sem_cair", "loco_esquerda_sem_cair", true },
    { "loco_diagonal_frente_razao", "loco_diagonal_frente_razao", "loco_diagonal_frente_sem_cair",
        "loco_diagonal_frente_sem_cair", true },
    { "loco_diagonal_costas_razao", "loco_diagonal_costas_razao", "loco_diagonal_costas_sem_cair",
        "loco_diagonal_costas_sem_cair", true },
    // (01/11, a rl_final: as faixas - no treino, os episodios que comecaram na faixa; no teste, para a frente.)
    { "loco_andar_razao", "andar_frente_razao", "loco_andar_sem_cair", "andar_frente_sem_cair", true },
    { "loco_trote_razao", "trote_frente_razao", "loco_trote_sem_cair", "trote_frente_sem_cair", true },
    { "loco_corrida_razao", "corrida_frente_razao", "loco_corrida_sem_cair", "corrida_frente_sem_cair", true },
    { "loco_sprint_razao", "sprint_frente_razao", "loco_sprint_sem_cair", "sprint_frente_sem_cair", true } } };
constexpr std::array<const char*, 12> LocomotionNames { "Parado", "Parado girando suave", "Frente (trote)",
    "Trás (trote)", "Direita (trote)", "Esquerda (trote)", "Diagonal frente (trote)", "Diagonal costas (trote)", "Andar",
    "Trote", "Corrida", "Sprint" };
// A velocidade do teste de cada faixa (m/s): a do primeiro clipe dela, para a frente (TrackingGetUpAmpTraining3D).
constexpr std::array<double, 4> BandTestSpeeds { 1.11, 2.62, 4.33, 6.0 };

// Os exercicios do levantar v2 depois do "Levantar do chao", na ordem da lista. (01/11, ele: o levantar volta a ser so
// levantar e evitar quedas - "dele tiramos o de locomocao": os da locomocao nele sairam da lista, o codigo deles fica; e
// o equilibrio parado tambem, de pe quem fica e a rede da locomocao, a do levantar so assume na queda iminente.)
std::vector<std::array<const char*, 2>> getUpDrillList() {
    std::vector<std::array<const char*, 2>> list { { TrackingViewer3D::MovingGetUpDrill, "Levantar em movimento" },
        { TrackingViewer3D::AvoidFallDrill, "Evitar a queda" } };
    // (01/11, a rl_final: a locomocao volta - as faixas primeiro, depois o parado, o giro e os rumos no trote. As quedas
    // da locomocao eram as do B21, fora de uso; ele: "parece meio cagado... acho dispensavel": so fora da rl_final.)
    if (!TrackingViewer3D::getUpLocomotion()) list.push_back({ TrackingViewer3D::LocomotionFallDrill, "Quedas da locomoção" });
    if (TrackingViewer3D::getUpLocomotion()) {
        // (Sem o andar: o jogo so usa trote, corrida e sprint - ele, 01/11 19h.)
        constexpr std::size_t Order[11] { 9, 10, 11, 0, 1, 2, 3, 4, 5, 6, 7 };
        for (std::size_t k : Order) list.push_back({ TrackingViewer3D::LocomotionDrills[k], LocomotionNames[k] });
    }
    return list;
}

// A velocidade-alvo de um exercicio do levantar: o em movimento 5, os rumos 2,5 (o teste), os outros parados.
double getUpGoalSpeed(const std::string& drill) {
    if (drill == TrackingViewer3D::MovingGetUpDrill) return 5.0;
    if (const int band = TrackingViewer3D::locomotionBandOf(drill); band >= 0) return BandTestSpeeds[static_cast<std::size_t>(band)];
    return TrackingViewer3D::locomotionHeadingOf(drill) >= 0 ? 2.5 : 0.0;
}

std::size_t locomotionIndex(const std::string& drill) {
    const auto& drills = TrackingViewer3D::LocomotionDrills;
    return static_cast<std::size_t>(std::find(drills.begin(), drills.end(), drill) - drills.begin());
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
    const std::string kind = config.value("kind", std::string());
    if (kind == "motor" || kind == "locomocao" || kind == "rastreador" || kind == "levantar") return true;
    const auto contract = config.find("contract");
    return contract != config.end() && contract->is_number() && contract->get<int>() == 2;
}

// O periodo dos graficos (recolhidos e expandidos; 0 = desde o inicio do treino),
// contado da rodada mais recente do treino.
constexpr std::array<std::pair<const char*, double>, 6> ChartWindows { { { "1 hora", 3600.0 },
    { "3 horas", 3.0 * 3600.0 }, { "6 horas", 6.0 * 3600.0 }, { "12 horas", 12.0 * 3600.0 },
    { "24 horas", 24.0 * 3600.0 }, { "Desde o início", 0.0 } } };

std::string clockText(double unixSeconds) {
    const std::time_t when = static_cast<std::time_t>(unixSeconds);
    char text[24];
    std::strftime(text, sizeof text, "%d/%m %H:%M:%S", std::localtime(&when));
    return text;
}

// Uma serie de um grafico: os valores (NaN: sem dado naquele ponto), a cor, se e a de referencia (mais
// apagada, sem preenchimento) e se so aparece na dica (as partes de um indice).
struct ChartSeries {
    const std::vector<float>* values;
    ImVec4 color;
    bool reference = false;
    std::string label;
    bool hidden = false;
};

bool anyFinite(const std::vector<float>& values) {
    return std::any_of(values.begin(), values.end(), [](float v) { return std::isfinite(v); });
}

// O ultimo valor com dado (NaN sem nenhum).
float lastFinite(const std::vector<float>& values) {
    for (auto it = values.rbegin(); it != values.rend(); ++it)
        if (std::isfinite(*it)) return *it;
    return std::numeric_limits<float>::quiet_NaN();
}

// A variacao no periodo: a media dos primeiros e dos ultimos 5% dos pontos com dado (o ruido nao engana).
std::optional<std::pair<double, double>> periodChange(const std::vector<float>& values) {
    std::vector<float> v;
    for (const float x : values)
        if (std::isfinite(x)) v.push_back(x);
    if (v.size() < 2) return std::nullopt;
    const std::size_t k = std::max<std::size_t>(1, v.size() / 20);
    double head = 0.0, tail = 0.0;
    for (std::size_t j = 0; j < k; ++j) {
        head += v[j];
        tail += v[v.size() - 1 - j];
    }
    return std::pair<double, double> { head / static_cast<double>(k), tail / static_cast<double>(k) };
}

// Um numero de grafico: inteiro de 1000 para cima (7450, nao 7.45e+03), uma casa de 10 a 1000, tres algarismos abaixo.
std::string chartNumber(double v) {
    char text[32];
    const bool whole = std::abs(v - std::round(v)) < 1e-6;
    std::snprintf(text, sizeof text, std::abs(v) >= 1000.0 || whole ? "%.0f" : std::abs(v) >= 10.0 ? "%.1f" : "%.3g", v);
    return text;
}

// A escala de um indice ou de uma fatia (0 a 100): em volta dos dados, de 10 em 10, nunca menos de 20 pontos
// (a tendencia aparece; o ruido nao vira montanha).
std::pair<float, float> indexRange(std::initializer_list<const std::vector<float>*> lists) {
    float lowest = FLT_MAX, highest = -FLT_MAX;
    for (const std::vector<float>* list : lists)
        for (const float v : *list) {
            if (!std::isfinite(v)) continue;
            lowest = std::min(lowest, v);
            highest = std::max(highest, v);
        }
    if (lowest > highest) return { 0.0f, 100.0f };
    float low = std::max(0.0f, std::floor((lowest - 3.0f) / 10.0f) * 10.0f);
    float high = std::min(100.0f, std::ceil((highest + 3.0f) / 10.0f) * 10.0f);
    while (high - low < 20.0f) {
        if (low > 0.0f) low -= 10.0f;
        if (high - low < 20.0f && high < 100.0f) high += 10.0f;
    }
    return { std::max(0.0f, low), std::min(100.0f, high) };
}

// A escala de um grafico: as bordas arredondadas para fora num numero redondo (1; 1,2; 1,5; 2; 2,5; 3; 4; 5; 6;
// 8 x 10^k), com folga acima do maximo: a linha nunca encosta no topo. Ignora os pontos sem dado.
std::pair<float, float> niceRange(std::initializer_list<const std::vector<float>*> lists, float floorTop,
    float floorBottom) {
    float lowest = floorBottom, highest = floorTop;
    for (const std::vector<float>* list : lists)
        for (const float v : *list) {
            if (!std::isfinite(v)) continue;
            lowest = std::min(lowest, v);
            highest = std::max(highest, v);
        }
    const auto nice = [](float v) {
        if (!(v > 0.0f)) return 0.0f;
        const float power = std::pow(10.0f, std::floor(std::log10(v)));
        for (const float step : { 1.0f, 1.2f, 1.5f, 2.0f, 2.5f, 3.0f, 4.0f, 5.0f, 6.0f, 8.0f, 10.0f })
            if (step * power >= v * 0.9999f) return step * power;
        return 10.0f * power;
    };
    const float pad = std::max((highest - lowest) * 0.10f, 1e-3f);
    const float top = highest > 0.0f ? nice(highest + pad) : (highest < 0.0f ? 0.0f : nice(pad));
    const float bottom = lowest >= 0.0f ? (floorBottom == 0.0f && lowest < 1e-6f ? 0.0f : std::max(0.0f, lowest - pad))
                                        : -nice(-lowest + pad);
    return { bottom, std::max(top, bottom + 1e-3f) };
}

// Um grafico de linha: fundo, grade, a serie principal com a area, a de referencia por tras, o valor de
// agora grande no canto e a dica com o valor sob o mouse.
void lineChart(const char* id, const std::vector<ChartSeries>& series, float minimum, float maximum, ImVec2 size,
    const char* valueText, const char* unit, const std::vector<double>* times = nullptr, bool timeLabels = false,
    const char* emptyText = "sem dados") {
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
    // O mouse em fracao da largura: cada serie acha o proprio ponto (elas tem tamanhos diferentes).
    float hoverAt = -1.0f;
    std::size_t hoverCount = 0;
    for (const ChartSeries& item : series) hoverCount = std::max(hoverCount, item.values->size());
    const auto drawn = [](const ChartSeries& item) { return !item.hidden && anyFinite(*item.values); };
    if (hovered && hoverCount > 1)
        hoverAt = std::clamp((ImGui::GetIO().MousePos.x - inner0.x) / std::max(inner1.x - inner0.x, 1.0f), 0.0f, 1.0f);
    const auto indexAt = [hoverAt](std::size_t size) {
        return static_cast<std::size_t>(std::lround(hoverAt * static_cast<float>(size - 1)));
    };
    // As areas, as referencias e as linhas, nessa ordem: nenhuma linha fica por baixo da area de outra.
    for (int pass = 0; pass < 3; ++pass) {
        for (const ChartSeries& item : series) {
            if (item.hidden || (pass == 1) != item.reference) continue;
            const std::vector<float>& v = *item.values;
            if (v.empty()) continue;
            const auto has = [&v](std::size_t i) { return std::isfinite(v[i]); };
            // (Um ponto so - a primeira avaliacao - fica na ponta direita.)
            const auto point = [&](std::size_t i) {
                const float at = v.size() < 2 ? 1.0f : static_cast<float>(i) / static_cast<float>(v.size() - 1);
                return ImVec2 { inner0.x + (inner1.x - inner0.x) * at, yOf(v[i]) };
            };
            if (pass == 0) {
                ImVec4 fill = item.color;
                fill.w = 0.16f;
                const ImU32 fillColor = ImGui::ColorConvertFloat4ToU32(fill);
                for (std::size_t i = 0; i + 1 < v.size(); ++i) {
                    if (!has(i) || !has(i + 1)) continue;
                    const ImVec2 a = point(i), b = point(i + 1);
                    draw->AddQuadFilled(a, b, { b.x, inner1.y }, { a.x, inner1.y }, fillColor);
                }
                continue;
            }
            ImVec4 line = item.color;
            if (item.reference) line.w = 0.8f;
            const ImU32 lineColor = ImGui::ColorConvertFloat4ToU32(line);
            for (std::size_t i = 0; i + 1 < v.size(); ++i)
                if (has(i) && has(i + 1)) draw->AddLine(point(i), point(i + 1), lineColor, item.reference ? 1.2f : 2.0f);
            // (Um ponto sozinho entre buracos vira um pingo; o ultimo com dado, o ponto de agora.)
            for (std::size_t i = 0; i < v.size(); ++i)
                if (has(i) && (i == 0 || !has(i - 1)) && (i + 1 == v.size() || !has(i + 1)))
                    draw->AddCircleFilled(point(i), 1.8f, lineColor);
            if (!item.reference)
                for (std::size_t i = v.size(); i-- > 0;)
                    if (has(i)) {
                        draw->AddCircleFilled(point(i), 3.2f, lineColor);
                        break;
                    }
        }
    }
    if (hoverAt >= 0.0f) {
        const float x = inner0.x + (inner1.x - inner0.x) * hoverAt;
        draw->AddLine({ x, inner0.y }, { x, inner1.y }, IM_COL32(120, 150, 180, 120));
        ImGui::BeginTooltip();
        if (times && !times->empty()) ImGui::TextDisabled("%s", clockText((*times)[indexAt(times->size())]).c_str());
        for (const ChartSeries& item : series) {
            if (item.values->empty() || !anyFinite(*item.values)) continue;
            const float v = (*item.values)[indexAt(item.values->size())];
            const std::string name = !item.label.empty() ? item.label + "  " : item.reference ? "clássico  " : "";
            if (!std::isfinite(v)) {
                ImGui::TextColored(item.color, "%s—", name.c_str());
                continue;
            }
            // As casas pelo tamanho do numero (8862, 13,2, 0,265).
            const char* format = std::abs(v) >= 100.0f ? "%s%.0f %s" : std::abs(v) >= 10.0f ? "%s%.1f %s" : "%s%.3g %s";
            ImGui::TextColored(item.color, format, name.c_str(), v, unit);
        }
        ImGui::EndTooltip();
    }
    if (valueText[0]) {
        const ImVec2 extent = ImGui::CalcTextSize(valueText);
        draw->AddText({ high.x - extent.x - 10.0f, low.y + 6.0f }, ImGui::ColorConvertFloat4ToU32(series.front().color),
            valueText);
    }
    const bool empty = std::none_of(series.begin(), series.end(), drawn);
    if (!empty) {
        // A escala so com dados (vazio, os numeros dos eixos nao dizem nada).
        draw->AddText({ low.x + 10.0f, low.y + 6.0f }, IM_COL32(100, 120, 140, 200), chartNumber(maximum).c_str());
        draw->AddText({ low.x + 10.0f, high.y - ImGui::GetTextLineHeight() - 6.0f }, IM_COL32(100, 120, 140, 200),
            chartNumber(minimum).c_str());
    }
    if (timeLabels && times && times->size() > 1) {
        // O comeco e o fim do trecho, embaixo nos cantos.
        const std::string first = clockText(times->front()), last = clockText(times->back());
        const float y = high.y - ImGui::GetTextLineHeight() - 6.0f;
        draw->AddText({ low.x + 60.0f, y }, IM_COL32(100, 120, 140, 200), first.c_str());
        draw->AddText({ high.x - ImGui::CalcTextSize(last.c_str()).x - 10.0f, y }, IM_COL32(100, 120, 140, 200),
            last.c_str());
    }
    if (empty) {
        const ImVec2 extent = ImGui::CalcTextSize(emptyText);
        draw->AddText({ (low.x + high.x - extent.x) * 0.5f, (low.y + high.y - extent.y) * 0.5f },
            IM_COL32(100, 120, 140, 200), emptyText);
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
    // (O treino principal primeiro - o "principal" do run.json; 31/10 23h, ele: a rede do levantar "passa a ser a nossa
    // rede PRINCIPAL de treinamento".)
    std::stable_partition(found.begin(), found.end(), [&](const auto& item) {
        std::ifstream file(m_paths.run(item.second) / "run.json");
        const nlohmann::json config = nlohmann::json::parse(file, nullptr, false);
        return !config.is_discarded() && config.is_object() && config.value("principal", false);
    });
    const bool hasMotor = std::any_of(found.begin(), found.end(), [&](const auto& item) {
        return MotorViewer3D::isMotorTask(m_paths.run(item.second) / "tarefa.json");
    });
    if (hasMotor) std::erase_if(found, [&](const auto& item) {
        return !MotorViewer3D::isMotorTask(m_paths.run(item.second) / "tarefa.json");
    });
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
    if (status.task == "motor_basico")
        return m_options.sourceRoot / "assets" / "rl" / "motor" / "basico.json";
    return m_options.sourceRoot / "assets" / "rl" / "tasks" / (status.task + ".json");
}

std::filesystem::path ReinforcementLearningScreen::policyPath() const {
    return m_paths.run(activeRun()) / "exports" / "policy_latest.mrlpolicy";
}

void ReinforcementLearningScreen::loadDrills() {
    m_drillIds.clear();
    m_drillNames.clear();
    m_drillPhases.clear();
    m_drillCategories.clear();
    m_drillWeights.clear();
    m_drillInfo.clear();
    m_locomotionTask = false;
    m_chartDataVersion = ~std::uint64_t { 0 };
    m_futureNames.clear();
    m_futurePhases.clear();
    const std::filesystem::path path = taskPath();
    if (path.empty() || !std::filesystem::exists(path)) return;
    if (TrackingViewer3D::isGetUpTask(path)) {
        // A rede do levantar, a principal: os exercicios dela, ativos (em roxo) enquanto ela treina.
        std::vector<std::array<const char*, 2>> all { { TrackingViewer3D::GetUpDrill, "Levantar do chão" } };
        const auto rest = getUpDrillList();
        all.insert(all.end(), rest.begin(), rest.end());
        for (const auto& [id, name] : all) {
            m_drillIds.push_back(id);
            m_drillNames.push_back(name);
            m_drillPhases.push_back(3);
            m_drillCategories.push_back("levantar");
            m_drillWeights.push_back(0.0f);
            m_drillInfo.push_back({ id, 0.0, getUpGoalSpeed(id), true });
        }
    } else if (MotorViewer3D::isMotorTask(path)) {
        const auto& ids = MotorTraining3D::drillIds();
        const auto& names = MotorTraining3D::drillNames();
        m_drillIds.assign(ids.begin(),ids.end());
        m_drillNames.assign(names.begin(),names.end());
        m_drillPhases.assign(ids.size(),1);
    } else if (const auto exercises = TrackingViewer3D::isTrackingTask(path)
                   ? TrackingViewer3D::locomotionTaskOf(path, m_options.sourceRoot / "assets")
                   : std::filesystem::path {};
               !exercises.empty()) {
        // O rastreador com os exercicios de uma tarefa da locomocao (29/10): a lista dela,
        // com as categorias e os pesos (o resto da tela do classico fica de fora).
        for (const LocomotionViewer3D::Drill& drill : LocomotionViewer3D::drillsOf(exercises)) {
            m_drillIds.push_back(drill.id);
            m_drillNames.push_back(drill.name);
            m_drillPhases.push_back(drill.phase);
            m_drillCategories.push_back(drill.category);
            m_drillWeights.push_back(drill.weight);
            m_drillInfo.push_back({ drill.id, drill.weight, drill.goalSpeed, drill.getUp });
        }
    } else if (TrackingViewer3D::isTrackingTask(path)) {
        // O rastreador: o lider na velocidade da tela, num dos 8 rumos (os do portao).
        for (std::size_t i = 0; i < TrackingViewer3D::HeadingNames.size(); ++i) {
            m_drillIds.push_back("rumo" + std::to_string(i));
            m_drillNames.push_back(TrackingViewer3D::HeadingNames[i]);
            m_drillPhases.push_back(i == 0 ? 1 : 2);
        }
        for (const auto& [id, name] : TrackingViewer3D::StillDrills) {
            m_drillIds.push_back(id);
            m_drillNames.push_back(name);
            m_drillPhases.push_back(2);
        }
    } else if (LocomotionViewer3D::isLocomotionTask(path)) {
        m_locomotionTask = true;
        for (const LocomotionViewer3D::Drill& drill : LocomotionViewer3D::drillsOf(path)) {
            m_drillIds.push_back(drill.id);
            m_drillNames.push_back(drill.name);
            m_drillPhases.push_back(drill.phase);
            m_drillCategories.push_back(drill.category);
            m_drillWeights.push_back(drill.weight);
            m_drillInfo.push_back({ drill.id, drill.weight, drill.goalSpeed, drill.getUp });
        }
        // A escada completa (a ultima tarefa): o que ainda nao esta no treino fica
        // apagado (so nas tarefas sem categorias).
        const bool categorized = std::any_of(m_drillCategories.begin(), m_drillCategories.end(),
            [](const std::string& c) { return !c.empty(); });
        const std::filesystem::path ladder = m_options.sourceRoot / "assets" / "rl" / "tasks" / "locomocao_f3f.json";
        if (!categorized && std::filesystem::exists(ladder)) {
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
    // O levantar v2 (31/10 17h): o levantar em movimento e o evitar a queda, da mesma rede do levantar (o treino a
    // parte levantar_v2), logo depois do "Levantar do chao" (fora da tarefa da locomocao: ela nao muda).
    if (const auto at = std::find(m_drillIds.begin(), m_drillIds.end(), std::string(TrackingViewer3D::GetUpDrill));
        at != m_drillIds.end() && TrackingViewer3D::getUpVersion2() && !TrackingViewer3D::isGetUpTask(path)) {
        const std::size_t i = static_cast<std::size_t>(at - m_drillIds.begin());
        const std::string category = i < m_drillCategories.size() ? m_drillCategories[i] : std::string();
        const int phase = i < m_drillPhases.size() ? m_drillPhases[i] : 1;
        // (A locomocao na rede do levantar, 31/10 22h35: os basicos do B21, depois dos outros.)
        const std::vector<std::array<const char*, 2>> all = getUpDrillList();
        for (std::size_t k = 0; k < all.size(); ++k) {
            const std::size_t where = i + 1 + k;
            m_drillIds.insert(m_drillIds.begin() + static_cast<std::ptrdiff_t>(where), all[k][0]);
            m_drillNames.insert(m_drillNames.begin() + static_cast<std::ptrdiff_t>(where), all[k][1]);
            if (m_drillPhases.size() >= where) m_drillPhases.insert(m_drillPhases.begin() + static_cast<std::ptrdiff_t>(where), phase);
            if (m_drillCategories.size() >= where)
                m_drillCategories.insert(m_drillCategories.begin() + static_cast<std::ptrdiff_t>(where), category);
            if (m_drillWeights.size() >= where) m_drillWeights.insert(m_drillWeights.begin() + static_cast<std::ptrdiff_t>(where), 0.0f);
            if (m_drillInfo.size() >= where)
                m_drillInfo.insert(m_drillInfo.begin() + static_cast<std::ptrdiff_t>(where),
                    { all[k][0], 0.0, getUpGoalSpeed(all[k][0]), true });
        }
    }
    if (std::find(m_drillIds.begin(), m_drillIds.end(), m_viewerDrill) == m_drillIds.end())
        m_viewerDrill = m_drillIds.empty() ? std::string() : m_drillIds.front();
}

std::vector<std::string> ReinforcementLearningScreen::drills() const { return m_drillIds; }

void ReinforcementLearningScreen::showDrill(const std::string& drill) {
    if (drill == m_viewerDrill && hasViewer()) return;
    m_viewerDrill = drill;
    resetViewer();
    // (O levantar: os graficos dele lidos na hora, sem esperar o ciclo.)
    if (TrackingViewer3D::isGetUpDrill(drill)) refreshGetUpCurve();
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
        m_assistChosen = false;
        resetViewer();
        loadDrills();
    }
    if (hasViewer() || !m_viewerError.empty() || run.empty()) return;
    const std::filesystem::path task = taskPath();
    const bool motor = !task.empty() && MotorViewer3D::isMotorTask(task);
    const bool tracking = !task.empty() && TrackingViewer3D::isTrackingTask(task);
    const bool getUpTask = !task.empty() && TrackingViewer3D::isGetUpTask(task);
    if (motor || tracking || getUpTask || (!task.empty() && LocomotionViewer3D::isLocomotionTask(task))) {
        try {
            m_viewerTask = task;
            const auto seed = static_cast<std::uint64_t>(std::time(nullptr));
            if (motor) m_locomotion = std::make_unique<MotorViewer3D>(m_options.sourceRoot / "assets", task,
                    m_viewerDrill, seed);
            else if (tracking) m_locomotion = std::make_unique<TrackingViewer3D>(m_options.sourceRoot / "assets", seed,
                    m_viewerDrill, TrackingViewer3D::locomotionTaskOf(task, m_options.sourceRoot / "assets"),
                    TrackingViewer3D::speedScaleOf(task), TrackingViewer3D::filterBetaOf(task),
                    TrackingViewer3D::conditionsOf(task));
            else if (getUpTask
                || (TrackingViewer3D::isGetUpDrill(m_viewerDrill)
                    && (TrackingViewer3D::getUpReady() || TrackingViewer3D::getUpTraining())))
                // (O levantar do chao e a rede propria dele, a do treino a parte, nao a deste.)
                m_locomotion = std::make_unique<TrackingViewer3D>(m_options.sourceRoot / "assets", seed, m_viewerDrill);
            else
                m_locomotion = std::make_unique<LocomotionViewer3D>(task, m_options.sourceRoot / "assets", m_viewerDrill,
                    seed);
            m_colliders = buildRagdollColliderWireframe3D(m_locomotion->profile());
            // (28/10: o 0% fixo da regua mostrava a rede numa ajuda em que ela nao treina.)
            if (auto* locomotion = dynamic_cast<LocomotionViewer3D*>(m_locomotion.get())) {
                if (m_assistChosen) locomotion->setAssist(m_locomotionAssist);
                else m_locomotionAssist = locomotion->assist();
            }
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
        ++m_historyVersion;
        return;
    }
    m_historyRun = activeRun();
    m_historyPending = std::async(std::launch::async,
        [database = m_paths.database, directory = m_paths.run(m_historyRun), run = m_historyRun,
            ancestors = m_ancestorHistories,
            since = ChartWindows[static_cast<std::size_t>(m_chartWindow)].second,
            points = std::size_t { m_chartsExpanded ? 1200u : 400u }] {
            // O rastreador grava os seus arquivos, nao o rl.db.
            if (MotorViewer3D::isMotorTask(directory / "tarefa.json"))
                return MotorViewer3D::readHistory(directory, points, since);
            if (TrackingViewer3D::isTrackingTask(directory / "tarefa.json"))
                return TrackingViewer3D::readHistory(directory, points, since);
            TrainingHistory3D history = readTrainingHistory3D(database, run, since, points);
            // (Um treino parado ha mais que a janela: o fim dele.)
            if (since > 0.0 && history.iterations.empty()) history = readTrainingHistory3D(database, run, 0.0, points);
            // A linhagem (28/10): os treinos de onde este ramo saiu, ate a rodada de cada ramo - o grafico nao
            // recomeca a cada ramo. Na janela de tempo, contada da rodada mais recente deste.
            double newest = 0.0;
            for (const TrainingIterationPoint3D& point : history.iterations) newest = std::max(newest, point.time);
            const double from = since > 0.0 && newest > 0.0 ? newest - since : 0.0;
            std::filesystem::path at = directory;
            for (int depth = 0; depth < 16; ++depth) {
                const std::optional<TrainingLineageLink3D> link = readTrainingLineage3D(at);
                if (!link) break;
                // (Os ramos anteriores nao mudam mais: lidos uma vez so enquanto a tela esta aberta.)
                const std::string key = link->run + "@" + std::to_string(points);
                auto cached = ancestors->find(key);
                if (cached == ancestors->end())
                    cached = ancestors->emplace(key, readTrainingHistory3D(database, link->run, 0.0, points)).first;
                prependTrainingAncestor3D(history, cached->second, link->iteration, from);
                at = directory.parent_path() / link->run;
            }
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
        m_getUpTraining = TrackingViewer3D::getUpTraining();
        if (TrackingViewer3D::isGetUpDrill(m_viewerDrill)) refreshGetUpCurve();
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
    std::vector<InspectionMarker> markers;
    std::vector<InspectionArrow> arrows;
    // O empurrao (pedido do usuario, 21/10): a seta vem de onde ele chega e
    // aponta para o corpo; comprimento e espessura crescem com a forca, a cor
    // diz a faixa (verde ate 300 N, amarelo ate 700, laranja ate 1400, vermelho
    // acima), os newtons ficam escritos e a seta some em 0,8 s depois do fim.
    m_pushLabel = {};
    Vec3 pushLabelPoint;
    if (view.pushLink >= 0 && static_cast<std::size_t>(view.pushLink) < positions.size()) {
        const Vec3 force = view.pushForce;
        const float newtons = std::hypot(force.x, force.y);
        if (newtons > 1.0f) {
            const Vec3 direction { force.x / newtons, force.y / newtons, 0.0f };
            const Vec3 at = positions[static_cast<std::size_t>(view.pushLink)];
            const float length = 0.35f + std::min(newtons, 2500.0f) / 1500.0f;
            const Vec3 tip = at - direction * 0.12f;
            const Vec3 tail = tip - direction * length;
            const float fade = std::clamp(view.pushFade, 0.0f, 1.0f);
            InspectionArrow push;
            push.from = m_skeleton->toSkeleton(tail);
            push.to = m_skeleton->toSkeleton(tip);
            push.color = newtons < 300.0f ? rgb(120, 220, 90, fade) : newtons < 700.0f ? rgb(250, 210, 60, fade)
                : newtons < 1400.0f ? rgb(250, 140, 40, fade) : rgb(235, 60, 50, fade);
            push.width = 0.05f + std::min(newtons, 2500.0f) / 25000.0f;
            push.lineWidthPixels = 3.0f + std::min(newtons, 2500.0f) / 300.0f;
            arrows.push_back(push);
            // (O rotulo e projetado depois, com a camera do quadro.)
            pushLabelPoint = m_skeleton->toSkeleton(tail + Vec3 { 0.0f, 0.0f, 0.12f });
            m_pushLabel.newtons = newtons;
            m_pushLabel.fade = fade;
        }
    }
    // Os pontos da estrela no chao (azuis); o de agora vai por cima, em verde (laranja: ele espera la).
    for (const Vec3& point : view.starPoints) {
        InspectionMarker mark;
        mark.center = m_skeleton->toSkeleton({ point.x, point.y, 0.0f });
        mark.radius = 0.3f;
        mark.color = rgb(70, 140, 230);
        mark.strength = 0.7f;
        markers.push_back(mark);
    }
    // O ponto do piloto (a estrela): verde, ou laranja quando e para parar nele.
    if (view.hasTarget) {
        InspectionMarker target;
        target.center = m_skeleton->toSkeleton({ view.target.x, view.target.y, 0.0f });
        target.radius = 0.35f;
        if (!view.starPoints.empty()) target.color = rgb(60, 210, 120);
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
    // O uniforme novo (o sistema de aparencia), como no inspetor e no laboratorio:
    // sem eles a imagem caia na textura antiga.
    scene.surface = model.surface.get();
    scene.kit = model.kit.get();
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
        // (01/11, ele: "a camera do laboratorio de RL nao resetar a posicao a cada troca de treino ou reinicio": o rumo
        // pedido pula no recomeco, na troca de exercicio e na troca de rede - a camera compensa o pulo e fica onde ele a
        // deixou; os giros do pedido ela segue.)
        const float headingTerm = std::atan2(ahead.y - aim.y, ahead.x - aim.x);
        if (m_headingValid) {
            const float jump = std::remainder(headingTerm - m_headingTerm, 6.28318531f);
            if (std::abs(jump) > 0.6f || view.steps < m_headingSteps || m_viewerDrill != m_headingDrill) m_camera.yaw -= jump;
        }
        m_headingTerm = headingTerm;
        m_headingSteps = view.steps;
        m_headingDrill = m_viewerDrill;
        m_headingValid = true;
        camera.yaw = headingTerm + m_camera.yaw;
    }
    if (m_pushLabel.newtons > 0.0f) {
        const ScreenPoint label = Projector(camera, width, height).project(pushLabelPoint);
        if (label.visible) {
            m_pushLabel.x = label.x / static_cast<float>(width);
            m_pushLabel.y = label.y / static_cast<float>(height);
        } else {
            m_pushLabel = {};
        }
    }
    // Onde ele aparece (os testes conferem que a camera nao o corta).
    const ScreenPoint shown = Projector(camera, width, height).project(pelvis);
    m_pelvisOnScreen = shown.visible
        ? std::array<float, 2> { shown.x / static_cast<float>(width), shown.y / static_cast<float>(height) }
        : std::array<float, 2> { -1.0f, -1.0f };
    {
        // O olho (um icone na tela): para onde o comando manda olhar, a 2 m dele na altura dos olhos.
        const float look = view.command.faceHeading;
        const Vec3 point = Vec3 { positions.front().x, positions.front().y, 1.65f }
            + Vec3 { std::cos(look), std::sin(look), 0.0f } * 2.0f;
        const ScreenPoint eye = Projector(camera, width, height).project(m_skeleton->toSkeleton(point));
        m_lookOnScreen = eye.visible
            ? std::array<float, 2> { eye.x / static_cast<float>(width), eye.y / static_cast<float>(height) }
            : std::array<float, 2> { -1.0f, -1.0f };
    }
    m_image = renderInspection(scene, style, camera, width, height);
    m_dirty = false;
    return true;
}

void ReinforcementLearningScreen::frame(float yawDegrees, float pitchDegrees, float distance) {
    m_camera.yaw = yawDegrees * DegreesToRadians;
    m_camera.pitch = pitchDegrees * DegreesToRadians;
    m_camera.distance = distance;
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
    const float charts = ui(150.0f);
    if (m_chartsExpanded) {
        // Expandido: so os graficos. Na linha do titulo, no canto, o recolher e o periodo (o mesmo do recolhido).
        sectionLabel("GRÁFICOS");
        const float back = ui(124.0f), window = ui(150.0f), gap = ui(8.0f);
        ImGui::SameLine(std::max(ImGui::GetCursorPosX(), ImGui::GetWindowContentRegionMax().x - back - gap - window));
        if (ImGui::Button(UI::FontAwesome::label(UI::FontAwesome::ArrowLeft, "Recolher").c_str(), { back, ui(28.0f) })) {
            m_chartsExpanded = false;
            m_historyWatch = 0.0f;
        }
        tooltip("Volta o visualizador.");
        ImGui::SameLine(0.0f, gap);
        drawChartWindow(window);
        subtleSeparator();
        // Sem rodape: a altura toda e dos graficos (rolando so se nao couberem).
        ImGui::BeginChild("RLChartsScroll", { 0.0f, std::max(ui(100.0f), ImGui::GetContentRegionAvail().y) }, false);
        drawCharts(ImGui::GetContentRegionAvail().y);
        ImGui::EndChild();
    } else {
        panelHeader("VISUALIZADOR");
        drawToolbar();
        drawViewport(canvas, charts + ui(106.0f));
        drawCommands();
        drawCharts(charts);
    }
    if (!m_chartsExpanded) drawFooter();
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
        const double decisions = static_cast<double>(status.steps);
        infoRow("Decisões", decisions >= 1e9 ? decimal(decisions / 1e9, 2) + " bi" : decimal(decisions / 1e6, 1) + " M");
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
    if (status.pid > 0 && status.task != "motor_basico") {
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
    tooltip(status.task == "motor_basico" ? "Avalia os dez exercícios por até 30 s cada, com a física do jogo."
        : "Roda a avaliação dos exercícios agora (nos dois solvers).");
}

void ReinforcementLearningScreen::drawModes(const TrainerStatus3D& status) {
    // (A rede do levantar: as threads e a volta depois dos cortes de temperatura sao do guarda dela.)
    if (TrackingViewer3D::isGetUpTask(taskPath())) return;
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
                notify(std::string("Modo ") + mode.label + (status.task == "motor_basico"
                    ? ": aplicado entre rodadas" : ": o treinador reabre em alguns segundos"));
            }
        }
        tooltip(status.task == "motor_basico"
            ? (i == 0 ? "12 threads de física: prioriza a velocidade do treino."
                : i == 1 ? "8 threads de física: reduz o uso do processador."
                         : "4 threads de física: deixa mais processador para outros programas.")
            : mode.tooltip);
    }
    ImGui::Dummy({ 1.0f, ui(2.0f) });
    if (status.task == "motor_basico") return;
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
    const bool motor = MotorViewer3D::isMotorTask(m_viewerTask);
    const bool categorized = std::any_of(m_drillCategories.begin(), m_drillCategories.end(),
        [](const std::string& c) { return !c.empty(); });
    if (categorized) {
        // Os ativos (no treino agora) juntos e os inativos a parte (pedido do usuario, 31/10; eram as
        // categorias Basico e Compostos).
        const bool getUpMain = TrackingViewer3D::isGetUpTask(m_viewerTask);
        for (const bool active : { true, false }) {
            if (!ImGui::CollapsingHeader(active ? "Ativos" : "Inativos", active ? ImGuiTreeNodeFlags_DefaultOpen : 0)) continue;
            for (std::size_t i = 0; i < m_drillIds.size(); ++i) {
                // (O levantar e outra rede, a do treino a parte levantar_v2: ativo enquanto aquele treino roda, em roxo -
                // 31/10, ele: "os exercicios que fazem parte da outra rede neural... cor roxa ao inves de verde".)
                const bool getUp = !motor && i < m_drillInfo.size() && m_drillInfo[i].getUp;
                const bool training = getUp ? m_getUpTraining : (i >= m_drillWeights.size() || m_drillWeights[i] > 0.0f);
                if (training != active) continue;
                const std::string label = UI::FontAwesome::label(UI::FontAwesome::Crosshairs, m_drillNames[i].c_str())
                    + "##drill" + m_drillIds[i];
                const bool apart = getUp && !m_getUpTraining && !TrackingViewer3D::getUpReady();
                ImGui::BeginDisabled(apart);
                if (phaseButton(label.c_str(), m_drillIds[i] == m_viewerDrill, { -1.0f, ui(30.0f) },
                        training ? (getUp ? 3 : 1) : 0))
                    showDrill(m_drillIds[i]);
                ImGui::EndDisabled();
                tooltip(apart ? "Desligado: a rede do levantar treina à parte e ainda não tem rede exportada."
                    : getUp && getUpMain ? "Mostra este exercício no mundo, com a rede do levantar (a principal)."
                    : getUp ? "Outra rede (roxo): a do levantar, que treina à parte. Mostra este exercício com ela."
                    : training ? "Mostra este exercício no mundo (o treino sorteia todos pelos pesos)."
                               : "Pronto, mas ainda fora do treino. Mostra este exercício no mundo.");
            }
        }
        return;
    }
    const bool basicOpen = !motor || ImGui::CollapsingHeader("Básico", ImGuiTreeNodeFlags_DefaultOpen);
    if (basicOpen) for (std::size_t i = 0; i < m_drillIds.size(); ++i) {
        const std::string label = UI::FontAwesome::label(UI::FontAwesome::Crosshairs, m_drillNames[i].c_str())
            + "##drill" + m_drillIds[i];
        const int phase = i < m_drillPhases.size() ? m_drillPhases[i] : 0;
        if (phaseButton(label.c_str(), m_drillIds[i] == m_viewerDrill, { -1.0f, ui(30.0f) }, phase))
            showDrill(m_drillIds[i]);
        const std::string hint = (phase > 0 ? "Fase " + std::to_string(phase) + ": m" : std::string("M"))
            + "ostra este exercício no mundo (o treino sorteia todos pelos pesos).";
        tooltip(motor
            ? "Mostra este exercício. Os exercícios do Básico treinam juntos na mesma política."
            : hint.c_str());
    }
    if (motor && ImGui::CollapsingHeader("Compostos")) ImGui::TextDisabled("Nenhum exercício por enquanto.");
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
        if (status.task == "motor_basico" && entry.drill != m_viewerDrill) continue;
        if (++shown > 1 || m_drillIds.size() > 1) {
            const auto it = std::find(m_drillIds.begin(), m_drillIds.end(), entry.drill);
            const std::string name = it == m_drillIds.end() ? entry.drill
                : m_drillNames[static_cast<std::size_t>(it - m_drillIds.begin())];
            ImGui::TextColored(InfoText, "%s", name.c_str());
        }
        infoRow("Sem cair", decimal((1.0 - entry.falls) * 100.0, 0) + "%");
        tooltip(status.task == "motor_basico"
            ? "Mundos que completaram 30 s sem cair, sem ruído e com a física do jogo (20/20)."
            : "Os mundos da avaliação que não caíram em 20 s, sem ruído e com a física do jogo (20/20).");
        if (entry.velocityError >= 0.0) {
            infoRow("Erro vel.", decimal(entry.velocityError, 2) + " m/s");
            tooltip("A mediana da diferença entre a velocidade pedida e a dele.");
        }
        if (entry.chestErrorDegrees >= 0.0) infoRow("Erro peito", decimal(entry.chestErrorDegrees, 1) + "°");
        if (entry.drift >= 0.0) infoRow("Deriva P90", decimal(entry.drift, 2) + " m");
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
    if (m_lookOnScreen[0] >= 0.0f) {
        const std::string eye = UI::FontAwesome::icon(UI::FontAwesome::Eye);
        const float iconSize = ImGui::GetFontSize() * 1.1f;
        const ImVec2 extent = ImGui::GetFont()->CalcTextSizeA(iconSize, FLT_MAX, 0.0f, eye.c_str());
        const ImVec2 at { origin.x + m_lookOnScreen[0] * size.x - extent.x * 0.5f,
            origin.y + m_lookOnScreen[1] * size.y - extent.y * 0.5f };
        drawList->AddText(ImGui::GetFont(), iconSize, at, IM_COL32(110, 205, 255, 235), eye.c_str());
    }
    if (m_pushLabel.newtons > 0.0f) {
        const std::string text = std::to_string(static_cast<int>(std::lround(m_pushLabel.newtons))) + " N";
        const float n = m_pushLabel.newtons;
        const int alpha = static_cast<int>(255.0f * std::clamp(m_pushLabel.fade, 0.0f, 1.0f));
        const ImU32 color = n < 300.0f ? IM_COL32(120, 220, 90, alpha) : n < 700.0f ? IM_COL32(250, 210, 60, alpha)
            : n < 1400.0f ? IM_COL32(250, 140, 40, alpha) : IM_COL32(235, 60, 50, alpha);
        const float textSize = ImGui::GetFontSize() * 1.15f;
        const ImVec2 extent = ImGui::GetFont()->CalcTextSizeA(textSize, FLT_MAX, 0.0f, text.c_str());
        const ImVec2 at { origin.x + m_pushLabel.x * size.x - extent.x * 0.5f, origin.y + m_pushLabel.y * size.y - extent.y };
        drawList->AddText(ImGui::GetFont(), textSize, { at.x + 1.0f, at.y + 1.0f }, IM_COL32(0, 0, 0, alpha), text.c_str());
        drawList->AddText(ImGui::GetFont(), textSize, at, color, text.c_str());
    }
    // A politica que esta no corpo (canto de cima), o pedido e a velocidade dele; no levantar, as quedas levantadas.
    std::string label = m_policyText + "\npedido " + decimal(viewerRequestedSpeed(), 1) + " m/s · ele "
        + decimal(viewerSpeed(), 1) + " m/s";
    if (const auto* tracking = dynamic_cast<const TrackingViewer3D*>(m_locomotion.get());
        tracking != nullptr && TrackingViewer3D::isGetUpDrill(m_viewerDrill)) {
        const auto [good, total] = tracking->getUpTally();
        const std::string count = std::to_string(good) + " de " + std::to_string(total);
        if (m_viewerDrill == TrackingViewer3D::MovingGetUpDrill)
            label = m_policyText + "\nchegou ao alvo " + count + " · pedido " + decimal(viewerRequestedSpeed(), 1)
                + " m/s · ele " + decimal(viewerSpeed(), 1) + " m/s";
        else if (m_viewerDrill == TrackingViewer3D::AvoidFallDrill)
            label = m_policyText + "\nevitou a queda " + count + " empurrões";
        else if (m_viewerDrill == TrackingViewer3D::BalanceDrill)
            label = m_policyText + "\nnão caiu " + count + " empurrões";
        else if (TrackingViewer3D::locomotionHeadingOf(m_viewerDrill) >= 0)
            label = m_policyText + "\npedido " + decimal(viewerRequestedSpeed(), 1) + " m/s · ele "
                + decimal(viewerSpeed(), 1) + " m/s · não caiu " + count;
        else if (TrackingViewer3D::isLocomotionDrill(m_viewerDrill))
            label = m_policyText + "\nnão caiu " + count;
        else
            label = m_policyText + "\nlevantou " + count + " quedas";
        // (A troca, 01/11: quem dirige agora - a rede do levantar ou, de pe e firme, a da locomocao.)
        const std::size_t cut = label.find('\n');
        const std::string who = tracking->drivingLocomotion() ? "rede da locomoção · " + tracking->partnerText()
                                                              : "rede do levantar · " + m_policyText;
        label = who + (cut == std::string::npos ? std::string() : label.substr(cut));
    }
    // (A interligacao das redes, 01/11: na queda do B21, quem dirige ate a troca.)
    if (const auto* locomotion = dynamic_cast<const LocomotionViewer3D*>(m_locomotion.get());
        locomotion != nullptr && locomotion->gettingUp())
        label = "rede do levantar · na queda\n" + m_policyText;
    drawList->AddText({ origin.x + ui(10.0f), origin.y + ui(8.0f) }, UiMuted, label.c_str());
}

void ReinforcementLearningScreen::drawCommands() {
    if (auto* locomotion = dynamic_cast<LocomotionViewer3D*>(m_locomotion.get())) {
        ImGui::SetNextItemWidth(ui(220.0f));
        if (ImGui::SliderFloat("Ajuda auxiliar", &m_locomotionAssist, 0.0f, 100.0f, "%.0f%%")) {
            locomotion->setAssist(m_locomotionAssist);
            m_assistChosen = true;
            m_dirty = true;
        }
        tooltip("Começa na do treino. No jogo ela cai com as pancadas e volta.");
    }
    if (auto* motor = dynamic_cast<MotorViewer3D*>(m_locomotion.get())) {
        ImGui::SetNextItemWidth(ui(220.0f));
        if (ImGui::SliderAngle("Alvo no horizonte", &m_motorLook, -180.0f, 180.0f)) {
            motor->setLook(m_motorLook);
            m_dirty = true;
        }
        ImGui::SameLine();
        if (ImGui::Button("Alvo automático")) {
            motor->reset(static_cast<std::uint64_t>(std::time(nullptr)));
            clearReplay();
            m_dirty = true;
        }
    }
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
    if (ImGui::Button(UI::FontAwesome::label(UI::FontAwesome::ChartLine, "Expandir").c_str(), { ui(120.0f), ui(28.0f) })) {
        m_chartsExpanded = true;
        m_historyWatch = 0.0f;
    }
    tooltip("Os gráficos detalhados no lugar do visualizador.");
    ImGui::SameLine(0.0f, ui(8.0f));
    drawChartWindow(ui(150.0f));
}

void ReinforcementLearningScreen::drawChartWindow(float width) {
    // (Da altura dos botoes da linha: 28 px na escala.)
    ImGui::PushStyleVar(ImGuiStyleVar_FramePadding,
        { ImGui::GetStyle().FramePadding.x, std::max(0.0f, (ui(28.0f) - ImGui::GetFontSize()) * 0.5f) });
    ImGui::SetNextItemWidth(width);
    if (ImGui::BeginCombo("##RLChartWindow", ChartWindows[static_cast<std::size_t>(m_chartWindow)].first)) {
        for (int i = 0; i < static_cast<int>(ChartWindows.size()); ++i) {
            const bool on = i == m_chartWindow;
            if (ImGui::Selectable(ChartWindows[static_cast<std::size_t>(i)].first, on) && !on) {
                m_chartWindow = i;
                m_historyWatch = 0.0f;
            }
            if (on) ImGui::SetItemDefaultFocus();
        }
        ImGui::EndCombo();
    }
    ImGui::PopStyleVar();
    tooltip("O período dos gráficos.");
}

const TrainingCharts3D& ReinforcementLearningScreen::charts(std::size_t slices) {
    if (m_chartDataVersion != m_historyVersion || m_chartDataDrill != m_viewerDrill || m_chartDataSlices != slices) {
        m_chartData = trainingCharts3D(m_history, m_drillInfo, m_viewerDrill, slices);
        m_chartDataVersion = m_historyVersion;
        m_chartDataDrill = m_viewerDrill;
        m_chartDataSlices = slices;
    }
    return m_chartData;
}

const TrainingEvaluationCharts3D& ReinforcementLearningScreen::evaluationCharts() {
    if (m_evaluationDataVersion != m_historyVersion) {
        m_evaluationData = trainingEvaluationCharts3D(m_history, m_drillInfo);
        m_evaluationDataVersion = m_historyVersion;
    }
    return m_evaluationData;
}

void ReinforcementLearningScreen::drawCharts(float height) {
    if (TrackingViewer3D::isGetUpDrill(m_viewerDrill)) {
        drawGetUpCharts(height);
        return;
    }
    if (m_locomotionTask && !MotorViewer3D::isMotorTask(m_viewerTask) && !TrackingViewer3D::isTrackingTask(m_viewerTask))
        drawLocomotionCharts(height);
    else
        drawLegacyCharts(height);
}

void ReinforcementLearningScreen::refreshGetUpCurve() {
    std::error_code error;
    const auto write = std::filesystem::last_write_time(TrackingViewer3D::getUpLog(), error);
    if (error || (write == m_getUpCurve.write && m_getUpCurve.window == m_chartWindow)) return;
    GetUpCurve curve;
    curve.write = write;
    curve.window = m_chartWindow;
    // (01/11, ele: "nao consigo filtrar por temporalidade, nem mostra a hora do dado": a janela do periodo, contada da
    // linha mais nova, e no maximo maxPoints pontos - um a cada passo; as avaliacoes de antes da janela seguem valendo.)
    std::vector<std::string> lines;
    {
        std::ifstream all(TrackingViewer3D::getUpLog());
        for (std::string line; std::getline(all, line);) lines.push_back(std::move(line));
    }
    double newest = 0.0;
    std::vector<double> hours(lines.size(), -1.0);
    for (std::size_t i = 0; i < lines.size(); ++i) {
        if (lines[i].find("\"tarefa\"") == std::string::npos) continue;
        const nlohmann::json row = nlohmann::json::parse(lines[i], nullptr, false);
        if (!row.is_discarded() && row.is_object() && row.contains("hora") && row["hora"].is_number())
            newest = std::max(newest, hours[i] = row["hora"].get<double>());
    }
    const double span = ChartWindows[static_cast<std::size_t>(std::clamp(m_chartWindow, 0, 5))].second;
    const double from = span > 0.0 && newest > 0.0 ? newest - span : -1.0;
    std::size_t inside = 0;
    for (double hour : hours) inside += hour >= 0.0 && hour >= from ? 1 : 0;
    const std::size_t maxPoints = m_chartsExpanded ? 1200 : 400;
    const std::size_t stride = std::max<std::size_t>(1, (inside + maxPoints - 1) / maxPoints);
    std::size_t seen = 0;
    std::istringstream file([&] {
        std::string joined;
        for (const std::string& line : lines) joined += line + '\n';
        return joined;
    }());
    const float none = std::numeric_limits<float>::quiet_NaN();
    const auto percent = [none](const nlohmann::json& row, const char* key) {
        const auto it = row.find(key);
        return it == row.end() || !it->is_number() ? none : 100.0f * it->get<float>();
    };
    const auto plain = [none](const nlohmann::json& row, const char* key) {
        const auto it = row.find(key);
        return it == row.end() || !it->is_number() ? none : it->get<float>();
    };
    float test = none, seconds = none, movingTest = none, movingSpeedTest = none, avoidTest = none, recomposeTest = none,
          balanceTest = none, poseTest = none;
    float spineTest = none, locoFallTest = none, locoFallSecondsTest = none;
    std::array<float, LocomotionLogKeys.size()> locoMainTest, locoUpTest;
    locoMainTest.fill(none);
    locoUpTest.fill(none);
    for (std::string line; std::getline(file, line);) {
        const nlohmann::json row = nlohmann::json::parse(line, nullptr, false);
        if (row.is_discarded() || !row.is_object() || !row.contains("atualizacao")) continue;
        if (row.value("aval", false)) {
            test = percent(row, "de_pe");
            seconds = percent(row, "tempo") / 100.0f;
            movingTest = percent(row, "mov_chegou");
            movingSpeedTest = plain(row, "mov_vel_rumo");
            avoidTest = percent(row, "evitar_evitou");
            recomposeTest = plain(row, "evitar_recompos_s");
            balanceTest = percent(row, "equil_sem_cair");
            poseTest = percent(row, "pose");
            spineTest = percent(row, "parado_coluna");
            locoFallTest = percent(row, "quedas_loco_levantou");
            locoFallSecondsTest = plain(row, "quedas_loco_tempo");
            for (std::size_t k = 0; k < LocomotionLogKeys.size(); ++k) {
                const LocomotionLogKeys3D& keys = LocomotionLogKeys[k];
                locoMainTest[k] = keys.percent ? percent(row, keys.mainTest) : plain(row, keys.mainTest);
                locoUpTest[k] = percent(row, keys.upTest);
            }
            continue;
        }
        if (!row.contains("tarefa")) continue;
        {
            const double hour = row.contains("hora") && row["hora"].is_number() ? row["hora"].get<double>() : -1.0;
            if (hour < from) continue;
            const bool lastInside = ++seen == inside;
            if ((seen - 1) % stride != 0 && !lastInside) continue;
            curve.times.push_back(hour);
        }
        curve.standing.push_back(percent(row, "queda_de_pe"));
        curve.kept.push_back(percent(row, "de_pe_de_pe"));
        curve.prone.push_back(percent(row, "queda_brucos"));
        curve.supine.push_back(percent(row, "queda_costas"));
        curve.side.push_back(percent(row, "queda_lado"));
        curve.test.push_back(test);
        curve.seconds.push_back(seconds);
        // (O levantar v2: o em movimento e o evitar a queda, no treino e, tracejado, nas avaliacoes fixas.)
        curve.reached.push_back(percent(row, "mov_chegou"));
        curve.reachedTest.push_back(movingTest);
        curve.movingStood.push_back(percent(row, "mov_de_pe"));
        curve.towardSpeed.push_back(plain(row, "mov_vel_rumo"));
        curve.towardSpeedTest.push_back(movingSpeedTest);
        curve.avoided.push_back(percent(row, "evitar_evitou"));
        curve.avoidedTest.push_back(avoidTest);
        curve.recomposed.push_back(percent(row, "evitar_recompos"));
        curve.recomposeSeconds.push_back(plain(row, "evitar_recompos_s"));
        curve.recomposeSecondsTest.push_back(recomposeTest);
        // (O equilibrio parado e a pose de pe perto do parado em alerta.)
        curve.balance.push_back(percent(row, "de_pe_sem_cair"));
        curve.balanceTest.push_back(balanceTest);
        curve.pose.push_back(percent(row, "queda_pose"));
        curve.poseStanding.push_back(percent(row, "de_pe_pose"));
        curve.poseTest.push_back(poseTest);
        curve.spine.push_back(percent(row, "de_pe_coluna"));
        curve.spineTest.push_back(spineTest);
        curve.locoFall.push_back(percent(row, "quedas_loco_levantou"));
        curve.locoFallTest.push_back(locoFallTest);
        curve.locoFallSeconds.push_back(plain(row, "quedas_loco_tempo"));
        curve.locoFallSecondsTest.push_back(locoFallSecondsTest);
        curve.handoffSeconds.push_back(plain(row, "quedas_loco_troca_s"));
        for (std::size_t k = 0; k < LocomotionLogKeys.size(); ++k) {
            const LocomotionLogKeys3D& keys = LocomotionLogKeys[k];
            curve.locoMain[k].push_back(keys.percent ? percent(row, keys.main) : plain(row, keys.main));
            curve.locoMainTest[k].push_back(locoMainTest[k]);
            curve.locoUp[k].push_back(percent(row, keys.up));
            curve.locoUpTest[k].push_back(locoUpTest[k]);
        }
    }
    m_getUpCurve = std::move(curve);
}

void ReinforcementLearningScreen::drawGetUpCharts(float height) {
    ImGui::Dummy({ 1.0f, ui(4.0f) });
    const GetUpCurve& c = m_getUpCurve;
    const ImVec4 green { 0.30f, 0.82f, 0.56f, 1.0f }, amber { 0.96f, 0.70f, 0.28f, 1.0f },
        violet { 0.70f, 0.56f, 0.96f, 1.0f }, blue { 0.47f, 0.62f, 0.98f, 1.0f }, cyan { 0.35f, 0.78f, 0.94f, 1.0f };
    const auto last = [](const std::vector<float>& values, const char* format) {
        const float v = lastFinite(values);
        if (!std::isfinite(v)) return std::string();
        char buffer[48];
        std::snprintf(buffer, sizeof buffer, format, v);
        return std::string(buffer);
    };
    const float spacing = ImGui::GetStyle().ItemSpacing.x;
    const ImVec2 size { (ImGui::GetContentRegionAvail().x - spacing) * 0.5f,
        std::max(height - ImGui::GetTextLineHeightWithSpacing(), ui(40.0f)) };
    const float rowTop = ImGui::GetCursorPosY();
    // Os graficos com o valor da avaliacao fixa na frente (o numero grande) e o do treino depois.
    const auto both = [&](const std::vector<float>& test, const std::vector<float>& train, const char* format) {
        const std::string t = last(test, format), r = last(train, format);
        return t.empty() ? r : "teste " + t + (r.empty() ? std::string() : " · treino " + r);
    };
    if (m_viewerDrill == TrackingViewer3D::MovingGetUpDrill) {
        ImGui::BeginGroup();
        ImGui::TextColored(InfoText, "%s", "Chegou ao alvo (%) · Levantar em movimento");
        tooltip("Cai e levanta já indo para um alvo a 4–8 m, em arrancada, virado para ele: quantos chegam ao alvo nos "
                "6 s. Tracejado, o teste da rede sem ruído em 32 quedas fixas com alvo; em laranja, quantos ficam de pé.");
        lineChart("##chegou", { { &c.reached, green, false, "treino" }, { &c.reachedTest, green, true, "teste sem ruído" },
                                  { &c.movingStood, amber, false, "de pé" } },
            0.0f, 100.0f, size, both(c.reachedTest, c.reached, "%.0f%%").c_str(), "%", &c.times, true, "sem dados ainda");
        ImGui::EndGroup();
        ImGui::SameLine();
        ImGui::SetCursorPosY(rowTop);
        ImGui::BeginGroup();
        ImGui::TextColored(InfoText, "%s", "Velocidade rumo ao alvo (m/s) · Levantar em movimento");
        tooltip("A velocidade média rumo ao alvo enquanto de pé e com o pedido (2,5–5 m/s): o treino e, tracejado, o teste.");
        lineChart("##rumo", { { &c.towardSpeed, cyan, false, "treino" }, { &c.towardSpeedTest, cyan, true, "teste sem ruído" } },
            0.0f, 6.0f, size, both(c.towardSpeedTest, c.towardSpeed, "%.2f m/s").c_str(), "m/s", &c.times, true,
            "sem dados ainda");
        ImGui::EndGroup();
        return;
    }
    if (const std::size_t k = locomotionIndex(m_viewerDrill); k < LocomotionLogKeys.size()) {
        const std::string name = LocomotionNames[k];
        const bool still = k == 0, turn = k == 1;
        ImGui::BeginGroup();
        const std::string title = still ? "Pose do parado em alerta (%) · " + name
            : turn                      ? "Erro do rumo (graus) · " + name
                                        : "Velocidade no rumo (% da pedida) · " + name;
        ImGui::TextColored(InfoText, "%s", title.c_str());
        tooltip(still ? "De pé, no fim: a semelhança da pose com o parado em alerta e, em violeta, a da coluna (a pelve, o "
                        "tronco, o pescoço e a cabeça como no clipe, reto de lado). O treino (de pé, com empurrões) e, "
                        "tracejado, o teste sem ruído em 8 começos fixos de pé, sem empurrão."
                : turn ? "O rumo pedido do corpo gira (0,4–0,8 rad/s, giros de 30–450° com pausas de 1–3 s): o erro médio "
                         "entre o rumo do corpo e o pedido, de pé. Tracejado, o teste sem ruído: 0,6 rad/s o episódio todo, 4 "
                         "para cada lado. Os empurrões em movimento entram quando ele estiver se locomovendo bem."
                : k >= 8
                ? "A velocidade média no rumo pedido sobre a pedida, de pé: no treino, os episódios que começaram nesta "
                  "faixa (parte já no clipe, na velocidade dele); tracejado, o teste sem ruído para a frente, de pé no "
                  "começo, na velocidade do clipe da faixa."
                       : "A velocidade média no rumo pedido sobre a pedida, de pé, mantendo o rumo do corpo: o treino (andar "
                         "0,8–1,4 ou trotar 2,0–3,0 m/s) e, tracejado, o teste sem ruído a 2,5 m/s. Os empurrões em "
                         "movimento entram quando ele estiver se locomovendo bem.");
        std::vector<ChartSeries> series { { &c.locoMain[k], turn ? amber : cyan, false, "treino" },
            { &c.locoMainTest[k], turn ? amber : cyan, true, "teste sem ruído" } };
        if (still) {
            series.push_back({ &c.spine, violet, false, "coluna, treino" });
            series.push_back({ &c.spineTest, violet, true, "coluna, teste" });
        }
        lineChart("##loco_principal", series,
            0.0f, turn ? 90.0f : still ? 100.0f : 150.0f, size,
            both(c.locoMainTest[k], c.locoMain[k], turn ? "%.0f°" : "%.0f%%").c_str(), turn ? "°" : "%", &c.times, true,
            "sem dados ainda");
        ImGui::EndGroup();
        ImGui::SameLine();
        ImGui::SetCursorPosY(rowTop);
        ImGui::BeginGroup();
        ImGui::TextColored(InfoText, "%s", ("Sem cair (%) · " + name).c_str());
        tooltip("Quantos episódios terminam sem cair (o chão tocado pelo tronco, o pescoço ou a cabeça).");
        lineChart("##loco_cair", { { &c.locoUp[k], green, false, "treino" }, { &c.locoUpTest[k], green, true, "teste sem ruído" } },
            0.0f, 100.0f, size, both(c.locoUpTest[k], c.locoUp[k], "%.0f%%").c_str(), "%", &c.times, true,
            "sem dados ainda");
        ImGui::EndGroup();
        return;
    }
    if (m_viewerDrill == TrackingViewer3D::BalanceDrill) {
        ImGui::BeginGroup();
        ImGui::TextColored(InfoText, "%s", "Sem cair com empurrões (%) · Equilíbrio parado");
        tooltip("De pé, com empurrões curtos e contínuos: quantos não caem (o chão tocado pelo tronco, o pescoço ou a "
                "cabeça). Tracejado, o teste sem ruído em 32 empurrões fixos com a régua do parado com empurrão forte do "
                "B21 (curto de 900–1200 N ou contínuo de 205–275 N por 1–3 s): a comparação com a outra rede.");
        lineChart("##equilibrio", { { &c.balance, green, false, "treino" }, { &c.balanceTest, green, true, "teste sem ruído" } },
            0.0f, 100.0f, size, both(c.balanceTest, c.balance, "%.0f%%").c_str(), "%", &c.times, true, "sem dados ainda");
        ImGui::EndGroup();
        ImGui::SameLine();
        ImGui::SetCursorPosY(rowTop);
        ImGui::BeginGroup();
        ImGui::TextColored(InfoText, "%s", "Pose de pé (%) · perto do parado em alerta");
        tooltip("A semelhança com o parado em alerta (ou o espelho) dos que terminam de pé, de 0 a 100: começando de pé "
                "com empurrões e, em laranja, depois de levantar de uma queda.");
        lineChart("##pose_equilibrio", { { &c.poseStanding, violet, false, "começando de pé" }, { &c.pose, amber, false, "depois de levantar" } },
            0.0f, 100.0f, size, last(c.poseStanding, "%.0f%%").c_str(), "%", &c.times, true, "sem dados ainda");
        ImGui::EndGroup();
        return;
    }
    if (m_viewerDrill == TrackingViewer3D::LocomotionFallDrill) {
        ImGui::BeginGroup();
        ImGui::TextColored(InfoText, "%s", "Levantou (%) · Quedas da locomoção");
        tooltip("Começando do instante em que a rede da locomoção (B21) caiu no treino dela: quantos ficam de pé. "
                "Tracejado, o teste sem ruído nas 32 primeiras quedas que chegaram (fixas).");
        lineChart("##quedas_loco", { { &c.locoFall, green, false, "treino" }, { &c.locoFallTest, green, true, "teste sem ruído" } },
            0.0f, 100.0f, size, both(c.locoFallTest, c.locoFall, "%.0f%%").c_str(), "%", &c.times, true, "sem dados ainda");
        ImGui::EndGroup();
        ImGui::SameLine();
        ImGui::SetCursorPosY(rowTop);
        ImGui::BeginGroup();
        ImGui::TextColored(InfoText, "%s", "Tempo para levantar (s) · Quedas da locomoção");
        tooltip("Da queda até ficar de pé: o treino e, tracejado, o teste. Em laranja, até a troca de rede (de pé e "
                "firme por 0,5 s; dali a rede da locomoção assume).");
        lineChart("##quedas_loco_tempo", { { &c.locoFallSeconds, violet, false, "treino" },
                                             { &c.locoFallSecondsTest, violet, true, "teste sem ruído" },
                                             { &c.handoffSeconds, amber, false, "troca" } },
            0.0f, 6.0f, size, both(c.locoFallSecondsTest, c.locoFallSeconds, "%.2f s").c_str(), "s", &c.times, true,
            "sem dados ainda");
        ImGui::EndGroup();
        return;
    }
    if (m_viewerDrill == TrackingViewer3D::AvoidFallDrill) {
        ImGui::BeginGroup();
        ImGui::TextColored(InfoText, "%s", "Evitou a queda (%) · Evitar a queda");
        tooltip("De pé, um empurrão forte (o curto ou o contínuo) que derrubava a rede de antes: quantos não caem "
                "(cabeça abaixo de 0,9 m ou o chão tocado por algo que não os pés, as mãos e os antebraços). Tracejado, o "
                "teste sem ruído em 32 empurrões fixos; em laranja, quantos se recompõem (de pé, na pose do parado e quase parado).");
        lineChart("##evitou", { { &c.avoided, green, false, "treino" }, { &c.avoidedTest, green, true, "teste sem ruído" },
                                  { &c.recomposed, amber, false, "recompôs-se" } },
            0.0f, 100.0f, size, both(c.avoidedTest, c.avoided, "%.0f%%").c_str(), "%", &c.times, true, "sem dados ainda");
        ImGui::EndGroup();
        ImGui::SameLine();
        ImGui::SetCursorPosY(rowTop);
        ImGui::BeginGroup();
        ImGui::TextColored(InfoText, "%s", "Tempo para se recompor (s) · Evitar a queda");
        tooltip("Do fim do empurrão até estar de pé, na pose do parado e quase parado: o treino e, tracejado, o teste.");
        lineChart("##recompos", { { &c.recomposeSeconds, violet, false, "treino" },
                                    { &c.recomposeSecondsTest, violet, true, "teste sem ruído" } },
            0.0f, 4.0f, size, both(c.recomposeSecondsTest, c.recomposeSeconds, "%.2f s").c_str(), "s", &c.times, true,
            "sem dados ainda");
        ImGui::EndGroup();
        return;
    }
    ImGui::BeginGroup();
    ImGui::TextColored(InfoText, "%s", "Levantou da queda (%) · Levantar do chão");
    tooltip("De pé no fim do episódio (6 s), começando de uma queda sorteada: o treino (com ruído) e, tracejado, o "
            "teste da rede sem ruído nas mesmas 64 quedas. Em laranja, os episódios que já começam de pé: quantos "
            "continuam de pé até o fim (o equilíbrio).");
    lineChart("##levantou", { { &c.standing, green, false, "treino" }, { &c.test, green, true, "teste sem ruído" },
                                { &c.kept, amber, false, "começando de pé" } },
        0.0f, 100.0f, size, last(c.standing, "%.0f%%").c_str(), "%", &c.times, true, "sem dados ainda");
    ImGui::EndGroup();
    ImGui::SameLine();
    ImGui::SetCursorPosY(rowTop);
    ImGui::BeginGroup();
    ImGui::TextColored(InfoText, "%s", "Pose de pé (%) · Levantar do chão");
    tooltip("Depois de levantar, a semelhança com o parado em alerta (ou o espelho), de 0 a 100: o treino e, tracejado, o "
            "teste nas 64 quedas fixas.");
    lineChart("##pose_levantar", { { &c.pose, violet, false, "treino" }, { &c.poseTest, violet, true, "teste sem ruído" } },
        0.0f, 100.0f, size, both(c.poseTest, c.pose, "%.0f%%").c_str(), "%", &c.times, true, "sem dados ainda");
    (void)blue;
    ImGui::EndGroup();
}

void ReinforcementLearningScreen::drawLocomotionCharts(float height) {
    ImGui::Dummy({ 1.0f, ui(4.0f) });
    const TrainingCharts3D& data = charts(m_chartsExpanded ? 240 : 160);
    const auto shown = std::find(m_drillIds.begin(), m_drillIds.end(), m_viewerDrill);
    const std::string drillName
        = shown == m_drillIds.end() ? std::string() : m_drillNames[static_cast<std::size_t>(shown - m_drillIds.begin())];
    TrainingDrillInfo3D info;
    for (const TrainingDrillInfo3D& candidate : m_drillInfo)
        if (candidate.id == m_viewerDrill) info = candidate;
    const auto text = [](float v, const char* format) {
        if (!std::isfinite(v)) return std::string();
        char buffer[48];
        std::snprintf(buffer, sizeof buffer, format, v);
        return std::string(buffer);
    };
    const ImVec4 green { 0.30f, 0.82f, 0.56f, 1.0f }, amber { 0.96f, 0.70f, 0.28f, 1.0f },
        cyan { 0.35f, 0.78f, 0.94f, 1.0f }, violet { 0.70f, 0.56f, 0.96f, 1.0f }, blue { 0.47f, 0.62f, 0.98f, 1.0f };

    if (!m_chartsExpanded) {
        // Recolhido (31/10, pedido dele: os graficos sao do exercicio que esta sendo visto, nao a media da tarefa):
        // o sem cair dele (o treino e, tracejado, o teste sem ruido) e, ao lado, a velocidade feita e a pedida (nos
        // parados, a estabilidade do movimento).
        const auto lastText = [&](const std::vector<float>& values, const char* format) {
            std::string value = text(lastFinite(values), format);
            if (const auto change = periodChange(values))
                value += text(static_cast<float>(change->second - change->first), "  (%+.1f)");
            return value;
        };
        const std::string inDrill = drillName.empty() ? std::string() : " · " + drillName;
        const char* empty = m_historyPending && m_history.iterations.empty() ? "carregando o histórico..."
                                                                             : "sem dados no período";
        const float spacing = ImGui::GetStyle().ItemSpacing.x;
        const ImVec2 size { (ImGui::GetContentRegionAvail().x - spacing) * 0.5f,
            std::max(height - ImGui::GetTextLineHeightWithSpacing(), ui(40.0f)) };
        const float rowTop = ImGui::GetCursorPosY();
        ImGui::BeginGroup();
        ImGui::TextColored(InfoText, "%s", ("Sem cair (%)" + inDrill).c_str());
        tooltip("Neste exercício: tracejado, o teste da rede sem ruído na velocidade alvo (a faixa inteira, como aqui "
                "no visualizador e no jogo); cheio, os episódios do treino, que pedem só até o teto do currículo.");
        std::vector<ChartSeries> standing { { &data.drillStanding, green, false, "treino (até o teto)" } };
        if (anyFinite(data.drillTest)) standing.push_back({ &data.drillTest, green, true, "teste na velocidade alvo" });
        // (31/10, ele: o numero do treino - pedido so ate o teto - parecia bom e na corrida de verdade ele caia. O numero
        // grande e o do teste na velocidade alvo; o do treino vem depois.)
        const std::string standingText = anyFinite(data.drillTest)
            ? "alvo " + text(lastFinite(data.drillTest), "%.0f%%") + " · treino " + text(lastFinite(data.drillStanding), "%.0f%%")
            : lastText(data.drillStanding, "%.0f%%");
        lineChart("##semcair", standing, 0.0f, 100.0f, size, standingText.c_str(), "%", &data.times, false, empty);
        ImGui::EndGroup();
        ImGui::SameLine();
        ImGui::SetCursorPosY(rowTop);
        ImGui::BeginGroup();
        if (info.goalSpeed > 0.0) {
            ImGui::TextColored(InfoText, "%s", ("Velocidade (m/s)" + inDrill).c_str());
            tooltip("Neste exercício: a velocidade do corpo e, tracejado, a pedida.");
            float top = 1.0f;
            for (const std::vector<float>* series : { &data.drillSpeed, &data.drillRequested })
                for (const float v : *series)
                    if (std::isfinite(v)) top = std::max(top, v);
            lineChart("##velocidade", { { &data.drillSpeed, cyan, false, "ele" }, { &data.drillRequested, cyan, true, "pedido" } },
                0.0f, std::ceil(top), size, lastText(data.drillSpeed, "%.2f m/s").c_str(), "m/s", &data.times, false,
                empty);
        } else {
            ImGui::TextColored(InfoText, "%s", ("Estabilidade (0 a 100)" + inDrill).c_str());
            tooltip("Neste exercício (parado): a estabilidade do movimento, de 0 a 100.");
            lineChart("##estabilidade", { { &data.drillStability, violet, false, "estabilidade" } }, 0.0f, 100.0f, size,
                lastText(data.drillStability, "%.0f").c_str(), "", &data.times, false, empty);
        }
        ImGui::EndGroup();
        return;
    }

    // Expandido: um grafico para cada coisa, do exercicio mostrado (as decisoes por segundo, do treino).
    struct Chart {
        std::string title;
        std::vector<ChartSeries> series;
        float minimum;
        float maximum;
        std::string value;
        const char* unit;
    };
    const std::string inDrill = drillName.empty() ? std::string() : " · " + drillName;
    float best = -FLT_MAX;
    for (const float v : data.drillReward)
        if (std::isfinite(v)) best = std::max(best, v);
    const std::vector<float> bestLine(best > -FLT_MAX ? 2 : 0, best);
    std::vector<Chart> list;
    {
        const auto [low, high] = indexRange({ &data.drillStanding, &data.drillTest });
        Chart chart { std::string(info.getUp ? "Levantou-se (%)" : "Sem cair (%)") + inDrill,
            { { &data.drillStanding, green, false, "treino (até o teto)" } }, low, high,
            anyFinite(data.drillTest)
                ? "alvo " + text(lastFinite(data.drillTest), "%.0f%%") + " · treino " + text(lastFinite(data.drillStanding), "%.0f%%")
                : text(lastFinite(data.drillStanding), "%.0f%%"),
            "%" };
        if (anyFinite(data.drillTest)) chart.series.push_back({ &data.drillTest, green, true, "teste na velocidade alvo" });
        list.push_back(std::move(chart));
    }
    // (A velocidade so nos exercicios que andam: parado, girando ou levando empurrao, ela nao diz nada.)
    if (info.goalSpeed > 0.0)
        list.push_back({ "Velocidade (m/s) · ele e, apagado, o pedido" + inDrill,
            { { &data.drillSpeed, cyan, false, "ele" }, { &data.drillRequested, cyan, true, "pedido" } }, 0.0f,
            niceRange({ &data.drillSpeed, &data.drillRequested }, 1.0f, 0.0f).second,
            text(lastFinite(data.drillSpeed), "%.2f m/s"), "m/s" });
    list.push_back({ "Estabilidade do movimento (0 a 100)" + inDrill,
        { { &data.drillStability, violet, false, "estabilidade" }, { &data.drillFacing, violet, false, "rumo do peito", true },
            { &data.drillFooting, violet, false, "pé firme", true }, { &data.drillTrunk, violet, false, "tronco", true } },
        indexRange({ &data.drillStability }).first, indexRange({ &data.drillStability }).second,
        text(lastFinite(data.drillStability), "%.0f"), "" });
    {
        const auto [low, high] = niceRange({ &data.drillReward, &bestLine }, 0.0f, 0.0f);
        list.push_back({ "Recompensa por segundo" + inDrill,
            { { &data.drillReward, amber, false, "" }, { &bestLine, amber, true, "melhor" } }, low, high,
            text(lastFinite(data.drillReward), "%.2f/s"), "/s" });
    }
    list.push_back({ "Decisões / s · o treino", { { &data.decisions, blue, false, "" } }, 0.0f,
        niceRange({ &data.decisions }, 1000.0f, 0.0f).second, text(lastFinite(data.decisions), "%.0f"), "" });

    // Um embaixo do outro, na largura toda, dividindo a altura (rolando so abaixo de 150 px cada).
    const float spacing = ImGui::GetStyle().ItemSpacing.y;
    const float each = std::max(ui(150.0f), (height + spacing) / static_cast<float>(list.size()) - spacing);
    for (std::size_t i = 0; i < list.size(); ++i) {
        const Chart& chart = list[i];
        ImGui::BeginGroup();
        ImGui::TextColored(InfoText, "%s", chart.title.c_str());
        if (i == 2) tooltip("A média do rumo do peito, do pé apoiado firme (sem escorregar) e do tronco dentro da postura.");
        const auto main = std::find_if(chart.series.begin(), chart.series.end(),
            [](const ChartSeries& item) { return !item.reference && !item.hidden; });
        if (main != chart.series.end())
            if (const auto change = periodChange(*main->values)) {
                const auto [head, tail] = *change;
                const double delta = tail - head;
                const std::string_view unit = chart.unit;
                std::string line = "no período: " + chartNumber(head) + " → " + chartNumber(tail)
                    + (unit.empty() ? "" : " " + std::string(unit)) + "  (" + (delta >= 0.0 ? "+" : "") + chartNumber(delta);
                // (Em %, a variacao em pontos; a relativa de quase zero nao diz nada.)
                if (std::abs(head) > 1e-9 && unit != "%" && !unit.empty()) {
                    char relative[32];
                    std::snprintf(relative, sizeof relative, ", %+.1f%%", 100.0 * delta / std::abs(head));
                    line += relative;
                }
                line += ")";
                const bool flat = std::abs(delta) <= 0.01 * std::max(std::abs(head), 1e-6);
                ImGui::SameLine(0.0f, ui(18.0f));
                ImGui::TextColored(flat           ? ImVec4 { 0.55f, 0.62f, 0.70f, 1.0f }
                        : delta > 0.0 ? ImVec4 { 0.30f, 0.82f, 0.56f, 1.0f }
                                      : ImVec4 { 0.95f, 0.42f, 0.42f, 1.0f },
                    "%s", line.c_str());
            }
        ImGui::PushID(static_cast<int>(i));
        lineChart("##chart", chart.series, chart.minimum, chart.maximum,
            { ImGui::GetContentRegionAvail().x, each - ImGui::GetTextLineHeightWithSpacing() }, chart.value.c_str(),
            chart.unit, &data.times, true, "sem dados no período");
        ImGui::PopID();
        ImGui::EndGroup();
    }
}

void ReinforcementLearningScreen::drawLegacyCharts(float height) {
    ImGui::Dummy({ 1.0f, ui(4.0f) });
    // A avaliacao do exercicio mostrado: a da rede (ou a do jogo, no Corpo) e, de referencia, a do classico.
    bool hasGame = false;
    for (const TrainingEvaluationPoint3D& point : m_history.evaluations) hasGame = hasGame || point.solver == "jogo";
    const std::string primary = hasGame ? "jogo" : "rede";
    std::vector<float> stand, standClassic, seconds, speed;
    for (const TrainingEvaluationPoint3D& point : m_history.evaluations) {
        if (!m_viewerDrill.empty() && point.drill != m_viewerDrill) continue;
        // (No levantar do chao o que conta e o quanto levantou, nao o quanto nao caiu.)
        if (point.solver == "classico")
            standClassic.push_back(static_cast<float>((std::isnan(point.stood) || m_viewerDrill != GetUpDrill
                                                           ? 1.0 - point.falls : point.stood) * 100.0));
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
    const bool motor = MotorViewer3D::isMotorTask(m_viewerTask);
    const bool tracking = motor || TrackingViewer3D::isTrackingTask(m_viewerTask);
    // O rastreador: tudo (menos decisoes/s) dos episodios do treino no rumo mostrado, rodada a rodada, com as
    // ultimas 12 rodadas juntas (pesadas pelos episodios); o sem cair conta os que chegaram ao fim sem cair nem
    // ficar para tras. O ultimo portao do rumo vai no canto.
    std::vector<float> directionSeconds, directionError, directionBody, directionRequested;
    std::vector<double> directionTimes;
    if (tracking) {
        struct Entry {
            double episodes, stood, reward, seconds, error, body, requested;
        };
        std::vector<Entry> recent;
        for (const TrainingDrillPoint3D& point : m_history.drills) {
            if (point.drill != m_viewerDrill || point.episodes <= 0.0) continue;
            const double n = point.episodes;
            const auto weighted = [n](double v) { return std::isnan(v) ? 0.0 : v * n; };
            recent.push_back({ n, (1.0 - point.falls) * n, weighted(point.reward), point.seconds * n,
                weighted(point.velocityError), weighted(point.bodySpeed), weighted(point.requestedSpeed) });
            if (recent.size() > Window) recent.erase(recent.begin());
            Entry sum {};
            for (const Entry& entry : recent) {
                sum.episodes += entry.episodes;
                sum.stood += entry.stood;
                sum.reward += entry.reward;
                sum.seconds += entry.seconds;
                sum.error += entry.error;
                sum.body += entry.body;
                sum.requested += entry.requested;
            }
            const auto mean = [&sum](double v) { return static_cast<float>(v / sum.episodes); };
            stand.push_back(mean(sum.stood) * 100.0f);
            rate.push_back(mean(sum.reward));
            directionSeconds.push_back(mean(sum.seconds));
            directionError.push_back(mean(sum.error));
            directionBody.push_back(mean(sum.body));
            directionRequested.push_back(mean(sum.requested));
            directionTimes.push_back(point.time);
        }
    } else {
        struct Entry {
            double episodes, fell, reward, seconds;
        };
        std::vector<Entry> recent;
        for (const TrainingDrillPoint3D& point : m_history.drills) {
            if (point.drill != m_viewerDrill) continue;
            const double notStood = std::isnan(point.stood) || m_viewerDrill != GetUpDrill ? point.falls : 1.0 - point.stood;
            recent.push_back({ point.episodes, notStood * point.episodes,
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
        // As bordas arredondadas para fora num numero redondo (1; 1,2; 1,5; 2; 2,5; 3; 4; 5; 6; 8 x 10^k), com
        // folga acima do maximo historico: a linha nunca encosta no topo e a escala da a ordem de grandeza.
        const auto nice = [](float v) {
            if (!(v > 0.0f)) return 0.0f;
            const float power = std::pow(10.0f, std::floor(std::log10(v)));
            for (const float step : { 1.0f, 1.2f, 1.5f, 2.0f, 2.5f, 3.0f, 4.0f, 5.0f, 6.0f, 8.0f, 10.0f })
                if (step * power >= v * 0.9999f) return step * power;
            return 10.0f * power;
        };
        const float pad = std::max((highest - lowest) * 0.10f, 1e-3f);
        const float top = highest > 0.0f ? nice(highest + pad) : (highest < 0.0f ? 0.0f : nice(pad));
        const float bottom = lowest >= 0.0f ? (floorBottom == 0.0f && lowest < 1e-6f ? 0.0f : std::max(0.0f, lowest - pad))
                                            : -nice(-lowest + pad);
        return std::pair<float, float> { bottom, std::max(top, bottom + 1e-3f) };
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
    const std::vector<float> bestLine(rate.empty() ? 0 : 2, best);
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
    if (tracking) {
        seconds.swap(directionSeconds);
        velocityError.swap(directionError);
        bodySpeed.swap(directionBody);
        requestedSpeed.swap(directionRequested);
    }
    // Os horarios das series do exercicio mostrado (no rastreador, as do rumo).
    const std::vector<double>& seriesTimes = tracking ? directionTimes : iterationTimes;
    const std::string inDrill = drillName.empty() ? std::string() : " · " + drillName;
    // O ultimo portao do rumo mostrado (no rastreador, no canto do sem cair).
    std::string gate;
    if (tracking)
        for (const TrainingEvaluationPoint3D& point : m_history.evaluations)
            if (point.drill == m_viewerDrill)
                gate = "teste r" + std::to_string(point.iteration) + ": "
                    + text(static_cast<float>((1.0 - point.falls) * 100.0), "%.0f%%");
    std::string standValue = stand.empty() ? "" : text(stand.back(), tracking ? "treino %.0f%%" : "%.0f%%");
    if (!gate.empty()) standValue += standValue.empty() ? gate : " · " + gate;
    // Sem episodios do rumo no treino: o degrau nao treina essa direcao (o G1 so a frente).
    const char* empty = motor ? "aguardando episódios deste exercício" : tracking ? "não treinado neste degrau" : "sem dados";
    std::vector<Chart> charts { {
        { std::string(m_viewerDrill == GetUpDrill ? "Levantou (%)" : "Sem cair (%)") + inDrill, { { &standClassic, green, true }, { &stand, green, false } }, 0.0f, 100.0f,
            standValue, "%", tracking ? &seriesTimes : nullptr },
        { (tracking && !motor ? "Evolução · nota por decisão" : "Evolução · recompensa por segundo") + inDrill,
            { { &bestLine, amber, true, "melhor" }, { &rate, amber, false } }, rateLow, rateHigh, rateValue,
            tracking && !motor ? "" : "/s", tracking ? &seriesTimes : nullptr },
        { std::string("Episódio (s)") + (tracking ? inDrill : std::string()), { { &seconds, cyan, false } }, 0.0f,
            range({ &seconds }, 1.0f, 0.0f).second, seconds.empty() ? "" : text(seconds.back(), "%.1f s"), "s",
            &seriesTimes },
        { "Decisões / s", { { &speed, violet, false } }, 0.0f, range({ &speed }, 1000.0f, 0.0f).second,
            speed.empty() ? "" : text(speed.back(), "%.0f"), "", &iterationTimes },
    } };
    std::vector<float> motorChest, motorDrift, motorImitation, motorPosture;
    std::vector<double> imitationTimes;
    std::vector<double> motorTimes;
    if (motor) {
        for (const auto& p : m_history.drills) {
            if (p.drill != m_viewerDrill || !std::isfinite(p.chestErrorDegrees) || !std::isfinite(p.drift)) continue;
            if (std::isfinite(p.imitation) && std::isfinite(p.posture)) {
                motorImitation.push_back(static_cast<float>(p.imitation * 100));
                motorPosture.push_back(static_cast<float>(p.posture * 100));
                imitationTimes.push_back(p.time);
            }
            motorChest.push_back(static_cast<float>(p.chestErrorDegrees));
            motorDrift.push_back(static_cast<float>(p.drift));
            motorTimes.push_back(p.time);
        }
        charts.insert(charts.end()-1, { "Erro do peito (graus)" + inDrill, { { &motorChest, cyan, false } }, 0.0f,
            range({ &motorChest }, 10.0f, 0.0f).second, motorChest.empty() ? "" : text(motorChest.back(), "%.1f°"),
            "°", &motorTimes, true });
        charts.insert(charts.begin()+3, { "Postura e semelhança com o clipe (%)" + inDrill,
            { { &motorPosture, green, false, "postura" }, { &motorImitation, violet, false, "clipe" } }, 0.f, 100.f,
            motorPosture.empty() ? "" : text(motorPosture.back(), "postura %.0f%%") + " · " + text(motorImitation.back(), "clipe %.0f%%"),
            "%", &imitationTimes });
        if (m_chartsExpanded) charts.insert(charts.end()-1, { (m_viewerDrill.starts_with("trote_") ? "Desvio lateral (m)" : "Deriva (m)") + inDrill, { { &motorDrift, amber, false } }, 0.0f,
            range({ &motorDrift }, 0.3f, 0.0f).second, motorDrift.empty() ? "" : text(motorDrift.back(), "%.2f m"),
            "m", &motorTimes, true });
    }
    if (motor && m_viewerDrill.starts_with("trote_") && !m_chartsExpanded) {
        charts[2] = { "Velocidade (m/s) · real e pedida" + inDrill,
            { { &requestedSpeed, cyan, true, "pedido" }, { &bodySpeed, cyan, false, "ele" } }, 0.0f,
            range({ &bodySpeed, &requestedSpeed }, 4.5f, 0.0f).second,
            bodySpeed.empty() ? "" : text(bodySpeed.back(), "%.2f m/s"), "m/s", &seriesTimes };
    }
    if (m_chartsExpanded) {
        // Expandido: tambem as quedas do treino (no rastreador o sem cair ja e o avesso delas) e, no rastreador, a
        // velocidade e o erro dela.
        if (!tracking)
            charts.insert(charts.begin() + 3, { "Quedas no treino (%)", { { &falls, amber, false } }, 0.0f, 100.0f,
                falls.empty() ? "" : text(falls.back(), "%.1f%%"), "%",
                &iterationTimes, true });
        if (!bodySpeed.empty())
            charts.insert(charts.end() - 1, { "Velocidade (m/s) · ele e, apagado, o pedido" + inDrill,
                { { &requestedSpeed, cyan, true, "pedido" }, { &bodySpeed, cyan, false, "ele" } }, 0.0f,
                range({ &bodySpeed, &requestedSpeed }, 1.0f, 0.0f).second, text(bodySpeed.back(), "%.2f m/s"), "m/s",
                &seriesTimes });
        if (!velocityError.empty())
            charts.insert(charts.end() - 1, { "Erro de velocidade (m/s)" + inDrill, { { &velocityError, green, false } }, 0.0f,
                range({ &velocityError }, 0.5f, 0.0f).second, text(velocityError.back(), "%.3f m/s"), "m/s",
                &seriesTimes, true });
    }
    const float spacing = ImGui::GetStyle().ItemSpacing.x;
    if (m_chartsExpanded) {
        // Um embaixo do outro, na largura toda, dividindo a altura (rolando so abaixo de 150 px cada).
        const float each = std::max(ui(150.0f), (height + ImGui::GetStyle().ItemSpacing.y) / static_cast<float>(charts.size())
            - ImGui::GetStyle().ItemSpacing.y);
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
                chart.unit, chart.times, true, chart.times == &iterationTimes ? "sem dados" : empty);
            ImGui::PopID();
            ImGui::EndGroup();
        }
        return;
    }
    // Recolhido: so "Sem cair" e "Evolucao", lado a lado, sem nada cortado.
    if (charts.size() > 2) charts.resize(2);
    const float width = (ImGui::GetContentRegionAvail().x - spacing) / static_cast<float>(charts.size());
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
            chart.unit, chart.times, false, chart.times == &iterationTimes ? "sem dados" : empty);
        ImGui::PopID();
        ImGui::EndGroup();
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
