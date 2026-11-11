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
std::vector<std::array<const char*, 2>> getUpDrillList(const std::filesystem::path& runDirectory = {}) {
    // (07/11, ele: "vamos treinar o levantar de costas e de brucos e... o de levantar correndo": com o v2.deitado, o levantar
    // em movimento comeca deitado e vira o "Levantar correndo"; o deitado de costas e de brucos, cada um pela avaliacao dele.)
    std::vector<std::array<const char*, 2>> list { { TrackingViewer3D::MovingGetUpDrill, "Levantar correndo" },
        { TrackingViewer3D::LyingBackDrill, "Levantar de costas" }, { TrackingViewer3D::LyingFrontDrill, "Levantar de bruços" },
        { TrackingViewer3D::AvoidFallDrill, "Evitar queda" }, { TrackingViewer3D::BalanceDrill, "Parado" },
        { TrackingViewer3D::EquilibriumDrill, "Equilíbrio (empurrões)" },
        { TrackingViewer3D::SceneryDrill, "Bench: levantar" },
        { TrackingViewer3D::AvoidSceneryDrill, "Bench: evitar" } };
    constexpr const char* getUpNames[6] { "Lev: pilares", "Lev: corredor", "Lev: rampa", "Lev: corpo no tronco", "Lev: corpos em cima", "Lev: corredor largo" };
    constexpr const char* avoidNames[5] { "Evit: paredes", "Evit: corredor", "Evit: rampa", "Evit: corpos", "Evit: pilares" };
    for (std::size_t k = 0; k < TrackingViewer3D::ControlledGetUpDrills.size(); ++k) list.push_back({ TrackingViewer3D::ControlledGetUpDrills[k], getUpNames[k] });
    for (std::size_t k = 0; k < 5; ++k) list.push_back({ TrackingViewer3D::ControlledAvoidDrills[k], avoidNames[k] });
    // (01/11, a rl_final: a locomocao volta - as faixas primeiro, depois o parado, o giro e os rumos no trote. As quedas
    // da locomocao - as do B21 - sairam; ele, 23h30: "nao vejo distincao para ele existir".)
    std::ifstream curriculumFile(runDirectory / "run.json");
    const auto curriculum = nlohmann::json::parse(curriculumFile, nullptr, false);
    // (03/11, o rastreio dos clipes: os 8 rumos do trote - as diagonais com o clipe da frente ou de costas, como o modo
    // padrao - e o giro parado.)
    if (!curriculum.is_discarded() && curriculum.contains("v2") && curriculum["v2"].value("rastreio", false)) {
        constexpr const char* ids[8] { "lev_trote_0", "lev_trote_1", "lev_trote_2", "lev_trote_3", "lev_trote_4", "lev_trote_5", "lev_trote_6", "lev_trote_7" };
        constexpr const char* names[8] { "Trote: frente", "Trote: diag. fr. esq.", "Trote: esq.", "Trote: diag. cost. esq.",
            "Trote: costas", "Trote: diag. cost. dir.", "Trote: dir.", "Trote: diag. fr. dir." };
        for (std::size_t k = 0; k < 8; ++k) list.push_back({ ids[k], names[k] });
        // (04/11, ele: "adicionar os treinos de CORRIDA nas 8 direcoes".)
        if (curriculum["v2"].value("corrida", 0.0) > 0.0) {
            constexpr const char* runIds[8] { "lev_corrida_0", "lev_corrida_1", "lev_corrida_2", "lev_corrida_3", "lev_corrida_4",
                "lev_corrida_5", "lev_corrida_6", "lev_corrida_7" };
            constexpr const char* runNames[8] { "Corrida: frente", "Corrida: diag. fr. esq.", "Corrida: esq.",
                "Corrida: diag. cost. esq.", "Corrida: costas", "Corrida: diag. cost. dir.", "Corrida: dir.", "Corrida: diag. fr. dir." };
            for (std::size_t k = 0; k < 8; ++k) list.push_back({ runIds[k], runNames[k] });
        }
        // (05/11, ele: "vamos fazer um treino de SPRINT com a velocidade meta de 8 m/s".)
        if (curriculum["v2"].value("sprint", 0.0) > 0.0) list.push_back({ "lev_sprint", "Sprint: frente" });
        // (04/11, ele: "adaptar os treinos de TROCAS de direcoes e curvas do b21" - no trote.)
        if (curriculum["v2"].value("manobras", 0.0) > 0.0) {
            constexpr const char* moveIds[7] { "lev_troca_suave", "lev_troca_media", "lev_troca_forte", "lev_troca_livre",
                "lev_curva_suave", "lev_curva_s", "lev_curva_fechada" };
            constexpr const char* moveNames[7] { "Troca suave", "Troca média", "Troca forte", "Troca livre", "Curva suave", "Curva em S", "Curva fechada" };
            for (std::size_t k = 0; k < 7; ++k) list.push_back({ moveIds[k], moveNames[k] });
            // (04/11 11h50, ele: "adicionar os exercicios das curvas agora em corridas".)
            if (curriculum["v2"].value("corrida", 0.0) > 0.0) {
                list.push_back({ "lev_curva_suave_corrida", "Corrida: curva suave" });
                list.push_back({ "lev_curva_s_corrida", "Corrida: curva em S" });
                list.push_back({ "lev_curva_fechada_corrida", "Corrida: curva fechada" });
            }
        }
        list.push_back({ TrackingViewer3D::LocomotionDrills[1], "Giro parado" });
        // (08/11, ele: "6 exercicios novos, da estrela de oito pontas... bem elaborados e visualmente organizados no laboratorio":
        // a estrela no treino - "estrela" no v2 do run.json.)
        // (08/11 14h40: o comando - o jogador emulado.)
        if (curriculum["v2"].value("comando", 0.0) > 0.0) list.push_back({ "lev_comando", "Comando (jogador emulado)" });
        if (curriculum["v2"].value("estrela", 0.0) > 0.0) {
            constexpr const char* starNames[6] { "Estrela trote: peito fixo", "Estrela trote: peito livre",
                "Estrela trote: peito girando", "Estrela corrida: peito fixo", "Estrela corrida: peito livre",
                "Estrela corrida: peito girando" };
            for (std::size_t k = 0; k < TrackingViewer3D::StarDrills.size(); ++k)
                list.push_back({ TrackingViewer3D::StarDrills[k], starNames[k] });
        }
        // (07/11, ele: os compostos - a estrela de 8 pontas. 07/11 06h20, ele: "pode ocultar os treinos da estrela... eles nao
        // entram agora": so com os compostos no treino - "compostos" no v2 do run.json.)
        if (curriculum["v2"].value("compostos", 0.0) > 0.0) {
            constexpr const char* compositeNames[4] { "Estrela: trote, peito fixo", "Estrela: trote, peito livre",
                "Estrela: corrida, peito fixo", "Estrela: corrida, peito livre" };
            for (std::size_t k = 0; k < TrackingViewer3D::CompositeDrills.size(); ++k)
                list.push_back({ TrackingViewer3D::CompositeDrills[k], compositeNames[k] });
        }
    } else if (TrackingViewer3D::getUpLocomotion(runDirectory)) {
        // (Sem o andar: o jogo so usa trote, corrida e sprint - ele, 01/11 19h.)
        // (03/11 17h50, ele: "por que botou trote corrida sprint como treino? ja temos os trote nas 8 direcoes" - so os
        // 8 rumos do trote, o parado e o giro.)
        constexpr const char* ids[8] { "lev_trote_0", "lev_trote_1", "lev_trote_2", "lev_trote_3", "lev_trote_4", "lev_trote_5", "lev_trote_6", "lev_trote_7" };
        constexpr const char* names[8] { "Trote: frente", "Trote: diagonal frente esquerda", "Trote: esquerda", "Trote: diagonal costas esquerda",
            "Trote: costas", "Trote: diagonal costas direita", "Trote: direita", "Trote: diagonal frente direita" };
        for (std::size_t k = 0; k < 8; ++k) list.push_back({ ids[k], names[k] });
        list.push_back({ TrackingViewer3D::LocomotionDrills[0], LocomotionNames[0] });
        list.push_back({ TrackingViewer3D::LocomotionDrills[1], "Giro parado" });
    }
    // (04/11 07h05, ele: "deixar ativo apenas o levantar e o evitar queda, ambos sem ambiente": com os cenarios e o levantar
    // andando fora do treino - o v2 do run.json -, os botoes deles saem.)
    if (!curriculum.is_discarded() && curriculum.contains("v2") && curriculum["v2"].is_object()) {
        const auto& v2 = curriculum["v2"];
        bool scenery = v2.value("evitar_ambientes", 0.0) > 0.0;
        if (v2.contains("ambientes") && v2["ambientes"].is_array())
            for (const auto& share : v2["ambientes"]) scenery = scenery || (share.is_number() && share.get<double>() > 0.0);
        const bool moving = v2.value("movimento", 0.0) > 0.0;
        std::erase_if(list, [&](const std::array<const char*, 2>& item) {
            const std::string id = item[0];
            const bool sceneryDrill = TrackingViewer3D::sceneryPresetOf(id) >= 0 || TrackingViewer3D::isAvoidSceneryDrill(id)
                || id == TrackingViewer3D::SceneryDrill || id == TrackingViewer3D::AvoidSceneryDrill;
            const bool lyingDrill = id == TrackingViewer3D::LyingBackDrill || id == TrackingViewer3D::LyingFrontDrill;
            return (sceneryDrill && !scenery) || (id == TrackingViewer3D::MovingGetUpDrill && !moving)
                || (lyingDrill && !v2.value("deitado", false))
                // (07/11, ele: "pode tirar o equilibrio empurroes... sera so no futuro... e sera na locomocao": so com a fatia.)
                || (id == TrackingViewer3D::EquilibriumDrill && (!v2.value("equilibrio", false) || v2.value("equilibrio_fatia", 0.0) <= 0.0))
                // (05/11 04h55, ele: "o treino do evitar queda agora sai, fica inativo; vou idealizar melhor ele futuramente".)
                || (id == TrackingViewer3D::AvoidFallDrill && v2.value("evitar", 0.0) <= 0.0);
        });
    }
    return list;
}

// Os rumos do trote e da corrida (os graficos por rumo).
bool headingDrill(const std::string& drill) {
    return drill.starts_with("lev_trote_") || drill.starts_with("lev_corrida_") || TrackingViewer3D::maneuverOf(drill) > 0
        || drill == "lev_giro" || drill == TrackingViewer3D::BalanceDrill || drill == "lev_sprint";
}
// A chave do exercicio nas estatisticas por exercicio do log (o parado em alerta e "lev_parado" la).
const char* scenarioKey(const std::string& drill) {
    // (05/11: o levantar do chao, com o levantar pelo clipe, tem a forca dele - lev_levantar.)
    if (drill == TrackingViewer3D::GetUpDrill) return "lev_levantar";
    if (drill == TrackingViewer3D::LyingBackDrill) return "lev_levantar_costas";
    if (drill == TrackingViewer3D::LyingFrontDrill) return "lev_levantar_brucos";
    if (drill == TrackingViewer3D::MovingGetUpDrill) return "lev_levantar_correndo";
    return drill == TrackingViewer3D::BalanceDrill ? "lev_parado" : drill.c_str();
}

// A velocidade-alvo de um exercicio do levantar: o em movimento 5, os rumos 2,5 (o teste), os outros parados.
double getUpGoalSpeed(const std::string& drill) {
    if (drill == TrackingViewer3D::MovingGetUpDrill) return 5.0;
    // (07/11, ele: as velocidades pedidas "os originais [dos clipes da Unreal] para nao contrapor com o juiz": o trote 4,5 na
    // frente e nas diagonais da frente, 3,9 de lado e 3,35 de costas; a corrida 7,7 (o sprint deles) na frente e nas
    // diagonais da frente e, sem sprint de lado e de costas, a corrida deles - 3,9 e 3,35.)
    if (drill == "lev_sprint") return 8.0;
    if (drill.starts_with("lev_trote_") || drill.starts_with("lev_corrida_")) {
        const char h = drill.back();
        const bool front = h == '0' || h == '1' || h == '7';
        // (07/11, ele: "a corrida a 6 m/s, sendo 5,5 para lateral e costas".)
        if (drill.starts_with("lev_corrida_")) return front ? 6.0 : 5.5;
        return front ? 4.5 : h == '2' || h == '6' ? 3.9 : 3.35;
    }
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
    // (04/11, ele: "os botoes dos exercicios com cores diferentes para cada categoria" - as da rl_final, ver drillCategory.)
    case 10: color = { 0.66f, 0.50f, 0.96f, 1.0f }; return true; // levantar
    case 11: color = { 0.95f, 0.38f, 0.35f, 1.0f }; return true; // evitar a queda
    case 12: color = { 0.30f, 0.80f, 0.80f, 1.0f }; return true; // parado
    case 13: color = { 0.25f, 0.78f, 0.45f, 1.0f }; return true; // trote
    case 14: color = { 0.96f, 0.62f, 0.20f, 1.0f }; return true; // corrida
    case 15: color = { 0.94f, 0.44f, 0.70f, 1.0f }; return true; // trocas de direcao
    case 16: color = { 0.40f, 0.58f, 0.98f, 1.0f }; return true; // curvas
    case 17: color = { 0.92f, 0.84f, 0.30f, 1.0f }; return true; // giro
    case 18: color = { 0.95f, 0.27f, 0.27f, 1.0f }; return true; // sprint (05/11)
    case 19: color = { 0.30f, 0.86f, 0.70f, 1.0f }; return true; // compostos (07/11)
    default: return false;
    }
}

// A categoria de um exercicio da rl_final (a cor do botao e o nome dela).
int drillCategory(const std::string& id) {
    if (id.starts_with("comp_")) return 19;
    if (id.starts_with("lev_trote_")) return 13;
    if (id == "lev_sprint") return 18;
    if (id.starts_with("lev_corrida_")) return 14;
    if (id.starts_with("lev_troca_")) return 15;
    if (id.starts_with("lev_curva_")) return 16;
    if (id.find("giro") != std::string::npos) return 17;
    if (id.find("parado") != std::string::npos) return 12;
    if (id.find("evitar") != std::string::npos) return 11;
    return 10;
}
const char* categoryName(int phase) {
    switch (phase) {
    case 10: return "Levantar";
    case 11: return "Evitar a queda";
    case 12: return "Parado";
    case 13: return "Trote";
    case 14: return "Corrida";
    case 15: return "Troca de direção";
    case 16: return "Curva";
    case 17: return "Giro";
    case 18: return "Sprint";
    case 19: return "Compostos";
    default: return nullptr;
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
    // (06/11 05h35, ele: "os graficos precisam ter LIMITES de quanto eles expandem, para nao virarem essas aberracoes": no
    // maximo 16 linhas de texto de altura e 55% da largura - o espaco que sobra fica vazio embaixo.)
    size.y = std::min(size.y, std::min(16.0f * ImGui::GetFontSize(), 0.55f * size.x));
    ImGui::InvisibleButton(id, size);
    const bool hovered = ImGui::IsItemHovered();
    const ImVec2 low = ImGui::GetItemRectMin(), high = ImGui::GetItemRectMax();
    ImDrawList* draw = ImGui::GetWindowDrawList();
    draw->AddRectFilled(low, high, IM_COL32(10, 17, 25, 255), 5.0f);
    draw->AddRect(low, high, IM_COL32(40, 62, 84, 255), 5.0f);
    // (05/11 20h: a faixa de cima com o valor e a de baixo com as horas ficam fora da area das linhas - a 99% a linha passava
    // por cima do texto, e perto de zero por cima das horas.)
    const float band = ImGui::GetTextLineHeight() + 8.0f;
    const ImVec2 inner0 { low.x + 8.0f, low.y + 8.0f + (valueText[0] ? band : 0.0f) },
        inner1 { high.x - 8.0f, high.y - 8.0f - (timeLabels && times && times->size() > 1 ? band : 0.0f) };
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
    // (06/11 04h, ele: "quando nao ha dados... eles precisam ficar bonitinhos e nao cortados nunca": o trecho do grafico vai
    // do primeiro ponto com dado de qualquer serie desenhada ate o agora - antes o eixo comecava na primeira linha do log e,
    // com o exercicio sem dado no comeco, a linha nascia no meio do grafico. Em fracao do historico, 0 = a primeira linha.)
    // (Pelas series principais: a linha vermelha do exigido tem dado desde o comeco e nao manda no recorte.)
    const auto firstAt = [&](bool references) {
        float at = 1.0f;
        for (const ChartSeries& item : series) {
            if (!drawn(item) || item.reference != references || item.values->size() < 2) continue;
            const std::vector<float>& v = *item.values;
            for (std::size_t i = 0; i < v.size(); ++i)
                if (std::isfinite(v[i])) {
                    at = std::min(at, static_cast<float>(i) / static_cast<float>(v.size() - 1));
                    break;
                }
        }
        return at;
    };
    float start = firstAt(false);
    if (start > 0.999f) start = firstAt(true);
    if (start > 0.999f) start = 0.0f;
    const auto frameAt = [start](float at) { return std::clamp((at - start) / (1.0f - start), 0.0f, 1.0f); };
    if (hovered && hoverCount > 1)
        hoverAt = std::clamp((ImGui::GetIO().MousePos.x - inner0.x) / std::max(inner1.x - inner0.x, 1.0f), 0.0f, 1.0f);
    const auto indexAt = [hoverAt, start](std::size_t size) {
        return static_cast<std::size_t>(std::lround((start + hoverAt * (1.0f - start)) * static_cast<float>(size - 1)));
    };
    // As areas, as referencias e as linhas, nessa ordem: nenhuma linha fica por baixo da area de outra.
    // (05/11 20h, ele: "corrija esses malditos graficos do levantar quando expandido... isso e horrivel": com duas ou mais
    // series cheias as areas se somavam num bloco solido - a area so com uma serie cheia.)
    const std::size_t filled = static_cast<std::size_t>(std::count_if(series.begin(), series.end(),
        [&](const ChartSeries& item) { return !item.reference && drawn(item); }));
    for (int pass = 0; pass < 3; ++pass) {
        if (pass == 0 && filled > 1) continue;
        for (const ChartSeries& item : series) {
            if (item.hidden || (pass == 1) != item.reference) continue;
            const std::vector<float>& v = *item.values;
            if (v.empty()) continue;
            const auto has = [&v](std::size_t i) { return std::isfinite(v[i]); };
            // (Um ponto so - a primeira avaliacao - fica na ponta direita.)
            const auto point = [&](std::size_t i) {
                const float at = v.size() < 2 ? 1.0f : frameAt(static_cast<float>(i) / static_cast<float>(v.size() - 1));
                return ImVec2 { inner0.x + (inner1.x - inner0.x) * at, yOf(v[i]) };
            };
            // (04/11, ele: "quando ha lacunas de dados, interligar com o mais proximo, para nao ficar feio" - os pontos com
            // dado ligados por cima dos buracos; depois do ultimo, o valor dele segue reto ate o agora (a borda direita);
            // antes do primeiro, nada.)
            std::vector<ImVec2> joined;
            const auto inside = [&](std::size_t i) {
                return v.size() < 2 || static_cast<float>(i) / static_cast<float>(v.size() - 1) >= start - 1e-6f;
            };
            for (std::size_t i = 0; i < v.size(); ++i)
                if (has(i) && inside(i)) joined.push_back(point(i));
            if (joined.empty()) continue;
            std::size_t lastFinite = v.size();
            while (lastFinite-- > 0 && !has(lastFinite)) {}
            if (lastFinite + 1 < v.size() && v.size() > 1) joined.push_back({ inner1.x, joined.back().y });
            // (06/11: um ponto so vira uma linha reta na largura toda, no valor dele - nao um pontinho solto no canto.)
            if (joined.size() == 1 && !item.reference) joined.insert(joined.begin(), ImVec2 { inner0.x, joined.front().y });
            if (pass == 0) {
                // (05/11 20h: a area num poligono so - a linha e o fundo -; em quadrilateros, as emendas faziam faixas
                // verticais.)
                if (joined.size() < 2) continue;
                ImVec4 fill = item.color;
                fill.w = 0.16f;
                std::vector<ImVec2> area = joined;
                area.push_back({ joined.back().x, inner1.y });
                area.push_back({ joined.front().x, inner1.y });
                draw->AddConcavePolyFilled(area.data(), static_cast<int>(area.size()), ImGui::ColorConvertFloat4ToU32(fill));
                continue;
            }
            ImVec4 line = item.color;
            if (item.reference) line.w = 0.8f;
            const ImU32 lineColor = ImGui::ColorConvertFloat4ToU32(line);
            for (std::size_t k = 0; k + 1 < joined.size(); ++k)
                draw->AddLine(joined[k], joined[k + 1], lineColor, item.reference ? 1.2f : 2.0f);
            if (joined.size() == 1) draw->AddCircleFilled(joined.front(), 1.8f, lineColor);
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
            // (Numa lacuna, o ponto com dado mais proximo.)
            const std::vector<float>& values = *item.values;
            const std::size_t at = indexAt(values.size());
            float v = values[at];
            // (06/11 08h15, ele: a dica mostrava a capacidade antes da primeira medicao - a linha de referencia (uma medicao que vale
            // ate a seguinte) so olha para tras.)
            for (std::size_t d = 1; !std::isfinite(v) && d < values.size(); ++d) {
                if (at >= d && std::isfinite(values[at - d])) v = values[at - d];
                else if (!item.reference && at + d < values.size() && std::isfinite(values[at + d])) v = values[at + d];
            }
            const std::string name = !item.label.empty() ? item.label + "  " : item.reference ? "clássico  " : "";
            if (!std::isfinite(v)) {
                // (Uma referencia sem medicao ainda - a capacidade antes da primeira - nao aparece na dica.)
                if (item.reference) continue;
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
        const std::size_t firstIndex = static_cast<std::size_t>(std::lround(start * static_cast<float>(times->size() - 1)));
        const std::string first = clockText((*times)[firstIndex]), last = clockText(times->back());
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
    std::stable_partition(found.begin(), found.end(), [&](const auto& item) {
        std::ifstream file(m_paths.run(item.second) / "status.json");
        const auto status = nlohmann::json::parse(file, nullptr, false);
        return !status.is_discarded() && status.is_object()
            && trainerProcessAlive3D(status.value("pid", 0), item.second);
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
    m_getUpCurve = {};
    m_getUpLive = {};
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

std::filesystem::path ReinforcementLearningScreen::getUpRunDirectory() const {
    const auto task = taskPath();
    return TrackingViewer3D::isGetUpTask(task) ? task.parent_path() : TrackingViewer3D::getUpLog().parent_path();
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
        std::vector<std::array<const char*, 2>> all { { TrackingViewer3D::GetUpDrill, "Levantar" } };
        // (07/11, ele: "vai ter 3 exercicios de levantar so: de costas, de brucos, correndo - apenas esses": com os deitados,
        // o levantar das quedas sorteadas sai da lista.)
        {
            std::ifstream runFile(path.parent_path() / "run.json");
            const auto runJson = nlohmann::json::parse(runFile, nullptr, false);
            if (!runJson.is_discarded() && runJson.contains("v2") && runJson["v2"].is_object() && runJson["v2"].value("deitado", false))
                all.clear();
        }
        const auto rest = getUpDrillList(path.parent_path());
        all.insert(all.end(), rest.begin(), rest.end());
        for (const auto& [id, name] : all) {
            m_drillIds.push_back(id);
            m_drillNames.push_back(name);
            m_drillPhases.push_back(drillCategory(id));
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
        at != m_drillIds.end() && TrackingViewer3D::getUpVersion2(getUpRunDirectory()) && !TrackingViewer3D::isGetUpTask(path)) {
        const std::size_t i = static_cast<std::size_t>(at - m_drillIds.begin());
        const std::string category = i < m_drillCategories.size() ? m_drillCategories[i] : std::string();
        // (A locomocao na rede do levantar, 31/10 22h35: os basicos do B21, depois dos outros.)
        const std::vector<std::array<const char*, 2>> all = getUpDrillList();
        for (std::size_t k = 0; k < all.size(); ++k) {
            const std::size_t where = i + 1 + k;
            m_drillIds.insert(m_drillIds.begin() + static_cast<std::ptrdiff_t>(where), all[k][0]);
            m_drillNames.insert(m_drillNames.begin() + static_cast<std::ptrdiff_t>(where), all[k][1]);
            if (m_drillPhases.size() >= where)
                m_drillPhases.insert(m_drillPhases.begin() + static_cast<std::ptrdiff_t>(where), drillCategory(all[k][0]));
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
                    && (TrackingViewer3D::getUpReady(getUpRunDirectory()) || TrackingViewer3D::getUpTraining(getUpRunDirectory()))))
                // (O levantar do chao e a rede propria dele, a do treino a parte, nao a deste.)
                m_locomotion = std::make_unique<TrackingViewer3D>(m_options.sourceRoot / "assets", seed, m_viewerDrill,
                    std::filesystem::path {}, 1.0f, 0.2f, TrackingViewerConditions3D {}, getUpRunDirectory());
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
            if (TrackingViewer3D::isTrackingTask(directory / "tarefa.json")
                || std::filesystem::exists(directory / "log.jsonl"))
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
        // (03/11, ele: "o laboratorio SEMPRE deve mostrar o atual que esta sendo treinado": o treino vivo - a lista poe os
        // vivos primeiro - assume a tela, mesmo que ela tenha aberto com o treino parado ou com outro.)
        if (m_options.run.empty()) {
            scanRuns();
            if (!m_runs.empty() && m_runs.front() != m_run) {
                std::ifstream file(m_paths.run(m_runs.front()) / "status.json");
                const auto status = nlohmann::json::parse(file, nullptr, false);
                if (!status.is_discarded() && status.is_object()
                    && trainerProcessAlive3D(status.value("pid", 0), m_runs.front()))
                    selectRun(m_runs.front());
            }
        }
        reloadPolicy(false);
        m_getUpTraining = TrackingViewer3D::getUpTraining(getUpRunDirectory());
        refreshGetUpLive();
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
    if (!view.scenery.empty()) {
        // (O cenario do levantar nos ambientes: as caixas e os corpos soltos em arame.)
        Highlight scenery;
        scenery.color = Palette::Info;
        for (const auto& [a, b] : view.scenery)
            scenery.segments.push_back({ m_skeleton->toSkeleton(a), m_skeleton->toSkeleton(b) });
        highlights.push_back(std::move(scenery));
    }
    if (m_colliders) {
        // (02/11, ele: os corpos que nascem em cima ou embaixo "terem suas caixas e colisoes humanas igual o nosso": o
        // mesmo perfil, desenhado pelos colisores, nas cores dos lados.)
        std::array<Highlight, 3> sides;
        sides[0].color = Palette::Center;
        sides[1].color = Palette::Left;
        sides[2].color = Palette::Right;
        for (const RagdollState3D* other : view.otherBodies) {
            if (other == nullptr || other->links.size() != profile.links.size()) continue;
            std::vector<Vec3> at(other->links.size());
            std::vector<Quaternion> turn(other->links.size());
            for (std::size_t i = 0; i < other->links.size(); ++i) {
                at[i] = other->links[i].position;
                turn[i] = other->links[i].orientation;
            }
            const RagdollColliderWireframe3D wireframe = poseRagdollColliderWireframe3D(profile, *m_colliders, at, turn, false);
            for (const RagdollColliderWireframeLink3D& link : wireframe.links) {
                const std::size_t side = link.side == RagdollColliderWireframeSide3D::Left ? 1
                    : link.side == RagdollColliderWireframeSide3D::Right ? 2 : 0;
                for (const auto& [a, b] : link.segments)
                    sides[side].segments.push_back({ m_skeleton->toSkeleton(a), m_skeleton->toSkeleton(b) });
            }
        }
        for (Highlight& side : sides)
            if (!side.segments.empty()) highlights.push_back(std::move(side));
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
    // (08/11, a estrela de 8 pontas no treino) As linhas do centro as pontas (azul claro), a perna de agora (verde, com o texto
    // dela em cima do ponto), o ponto seguinte (verde fraco) e o rastro da pelve pela velocidade (azul trote, laranja corrida,
    // vermelho acima de 6,5 m/s), que recomeca no episodio novo.
    if (!view.starLines.empty()) {
        Highlight lines;
        lines.color = rgb(120, 170, 235, 0.55f);
        for (const auto& [a, b] : view.starLines)
            lines.segments.push_back({ m_skeleton->toSkeleton({ a.x, a.y, 0.01f }), m_skeleton->toSkeleton({ b.x, b.y, 0.01f }) });
        highlights.push_back(std::move(lines));
    }
    if (view.hasStarLeg) {
        Highlight leg;
        leg.color = rgb(70, 220, 120);
        const auto& [a, b] = view.starLeg;
        leg.segments.push_back({ m_skeleton->toSkeleton({ a.x, a.y, 0.02f }), m_skeleton->toSkeleton({ b.x, b.y, 0.02f }) });
        if (!view.starCaption.empty()) {
            leg.label = view.starCaption;
            leg.labelAt = m_skeleton->toSkeleton({ b.x, b.y, 0.9f });
            leg.hasLabel = true;
        }
        highlights.push_back(std::move(leg));
    }
    // (08/11 14h40: o comando - o texto em cima do corpo.)
    if (!view.hasStarLeg && !view.starCaption.empty() && !positions.empty()) {
        Highlight caption;
        caption.color = rgb(230, 230, 240);
        caption.label = view.starCaption;
        caption.labelAt = m_skeleton->toSkeleton({ positions[0].x, positions[0].y, positions[0].z + 1.2f });
        caption.hasLabel = true;
        caption.points.push_back(caption.labelAt);
        highlights.push_back(std::move(caption));
    }
    if (view.hasNextTarget) {
        InspectionMarker next;
        next.center = m_skeleton->toSkeleton({ view.nextTarget.x, view.nextTarget.y, 0.0f });
        next.radius = 0.3f;
        next.color = rgb(60, 210, 120);
        next.strength = 0.35f;
        markers.push_back(next);
    }
    if (!view.starPoints.empty() && !positions.empty()) {
        const Vec3 pelvis { positions[0].x, positions[0].y, 0.03f };
        const float speed = std::hypot(view.velocity.x, view.velocity.y);
        if (!m_starTrail.empty() && std::hypot(pelvis.x - m_starTrail.back().first.x, pelvis.y - m_starTrail.back().first.y) > 3.0f)
            m_starTrail.clear();
        if (m_starTrail.empty() || std::hypot(pelvis.x - m_starTrail.back().first.x, pelvis.y - m_starTrail.back().first.y) > 0.05f)
            m_starTrail.push_back({ pelvis, speed });
        if (m_starTrail.size() > 900) m_starTrail.erase(m_starTrail.begin(), m_starTrail.begin() + 100);
        Highlight bands[3];
        bands[0].color = rgb(90, 160, 255);
        bands[1].color = rgb(250, 150, 40);
        bands[2].color = rgb(235, 60, 50);
        for (std::size_t k = 1; k < m_starTrail.size(); ++k) {
            const float v = m_starTrail[k].second;
            bands[v < 4.6f ? 0 : v < 6.5f ? 1 : 2].segments.push_back(
                { m_skeleton->toSkeleton(m_starTrail[k - 1].first), m_skeleton->toSkeleton(m_starTrail[k].first) });
        }
        for (Highlight& band : bands)
            if (!band.segments.empty()) highlights.push_back(std::move(band));
    } else {
        m_starTrail.clear();
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
        // (04/11 12h40, ele: "ao lado do botao de recolher, um checkbox de mostrar exigencias" - nos exercicios com a forca
        // dinamica.)
        const bool demands = headingDrill(m_viewerDrill);
        const char* demandLabel = "Mostrar exigências";
        const float demand = demands ? ImGui::GetFrameHeight() + ImGui::GetStyle().ItemInnerSpacing.x
                + ImGui::CalcTextSize(demandLabel).x + gap : 0.0f;
        ImGui::SameLine(std::max(ImGui::GetCursorPosX(), ImGui::GetWindowContentRegionMax().x - demand - back - gap - window));
        if (demands) {
            const float rowY = ImGui::GetCursorPosY();
            ImGui::SetCursorPosY(rowY + std::max(0.0f, (ui(28.0f) - ImGui::GetFrameHeight()) * 0.5f));
            ImGui::Checkbox(demandLabel, &m_showRequirements);
            tooltip("O mínimo exigido para descer a força auxiliar, em vermelho.");
            ImGui::SameLine(0.0f, gap);
            ImGui::SetCursorPosY(rowY);
        }
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
    // Trocar a visualizacao nao altera os treinos em andamento.
    const std::string shown = activeRun();
    ImGui::TextWrapped("%s", shown.c_str());
    if (status.known) {
        infoRow("Tarefa", status.taskName.empty() ? status.task : status.taskName);
        infoRow("Rodada", std::to_string(status.iteration));
        if (m_getUpLive.run == shown && m_getUpLive.assist > 0.0)
            infoRow("Força auxiliar", assistText(m_getUpLive));
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
    drawGetUpLive();
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

// A forca auxiliar de agora num exercicio da rl_final: o trote e o giro a do rumo, a corrida e as manobras as delas (ou a do
// rumo); o resto (levantar, evitar, parado) nao tem mao.
std::string ReinforcementLearningScreen::drillAssist(const std::string& id) const {
    const GetUpLive& g = m_getUpLive;
    const int category = drillCategory(id);
    // (A forca dinamica, por exercicio, quando existe.)
    if (const auto it = g.drillAssist.find(scenarioKey(id)); it != g.drillAssist.end()) {
        const double percent = it->second * 100.0;
        return decimal(percent, std::abs(percent - std::round(percent)) > 0.05 ? 1 : 0) + "%";
    }
    const double value = category == 13 || category == 17 ? g.assist
        : category == 14 ? (g.runAssist >= 0.0 ? g.runAssist : g.assist)
        : category == 15 || category == 16 ? (g.maneuverAssist >= 0.0 ? g.maneuverAssist : g.assist) : 0.0;
    const double percent = value * 100.0;
    return decimal(percent, std::abs(percent - std::round(percent)) > 0.05 ? 1 : 0) + "%";
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
        // (04/11 03h45, ele: "duas categorias, uma para locomocao e outra para queda, e entre parenteses a porcentagem
        // destinada" - na rede principal; nos outros treinos, os ativos e os inativos como antes.)
        const double locomotionShare = std::clamp(m_getUpLive.locomotionShare, 0.0, 1.0);
        const std::string groups[2] { "Locomoção (" + decimal(locomotionShare * 100.0, 0) + "%)##grupo_locomocao",
            "Quedas (" + decimal((1.0 - locomotionShare) * 100.0, 0) + "%)##grupo_quedas" };
        for (const bool active : { true, false }) {
            const char* header = getUpMain && m_getUpLive.known ? groups[active ? 0 : 1].c_str() : active ? "Ativos" : "Inativos";
            if (!ImGui::CollapsingHeader(header, active || getUpMain ? ImGuiTreeNodeFlags_DefaultOpen : 0)) continue;
            for (std::size_t i = 0; i < m_drillIds.size(); ++i) {
                if (getUpMain && m_getUpLive.known) {
                    // (Locomocao: o parado, os rumos, a corrida, as manobras e o giro; quedas: o levantar e o evitar.)
                    const bool locomotion = drillCategory(m_drillIds[i]) >= 12;
                    if (locomotion != active) continue;
                }
                // (O levantar e outra rede, a do treino a parte levantar_v2: ativo enquanto aquele treino roda, em roxo -
                // 31/10, ele: "os exercicios que fazem parte da outra rede neural... cor roxa ao inves de verde".)
                const bool getUp = !motor && i < m_drillInfo.size() && m_drillInfo[i].getUp;
                const bool training = getUp ? m_getUpTraining : (i >= m_drillWeights.size() || m_drillWeights[i] > 0.0f);
                if (!(getUpMain && m_getUpLive.known) && training != active) continue;
                // (04/11, ele: "bota entre parenteses de cada um a forca auxiliar que esta atuando naquele exercicio".)
                // (07/11: sem mao, o "(0%)" sumiu.)
                const std::string force = getUpMain && getUp && m_getUpLive.known ? drillAssist(m_drillIds[i]) : "0%";
                const std::string shown = m_drillNames[i] + (force != "0%" ? " (" + force + ")" : std::string());
                const std::string label = UI::FontAwesome::label(UI::FontAwesome::Crosshairs, shown.c_str()) + "##drill" + m_drillIds[i];
                const bool apart = getUp && !m_getUpTraining && !TrackingViewer3D::getUpReady(getUpRunDirectory());
                ImGui::BeginDisabled(apart);
                // (04/11, ele: "ta tudo roxo os botao" - a cor da categoria, quando o exercicio tem uma.)
                const int category = i < m_drillPhases.size() && m_drillPhases[i] >= 10 ? m_drillPhases[i] : getUp ? 3 : 1;
                if (phaseButton(label.c_str(), m_drillIds[i] == m_viewerDrill, { -1.0f, ui(30.0f) },
                        training || (getUpMain && m_getUpLive.known) ? category : 0))
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
        const char* category = categoryName(phase);
        const std::string hint = (category ? std::string(category) + ": m" : phase > 0 ? "Fase " + std::to_string(phase) + ": m" : std::string("M"))
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
    // (07/11, temporario - ele: "so para eu ver na pratica como o Motion Matching estaria operando") Os exercicios de forma
    // cinematica: o corpo no quadro que o Motion Matching do juiz escolhe a cada decisao, sem fisica.
    // (07/11, ele: "vamos fazer os clipes tipo nos faziamos antes... so ver no laboratorio": a sequencia de clipes da Unreal
    // de cada exercicio, sem fisica.)
    if (auto* sequencing = dynamic_cast<TrackingViewer3D*>(m_locomotion.get()); sequencing != nullptr && TrackingViewer3D::isGetUpDrill(m_viewerDrill)) {
        bool on = sequencing->clipSequence();
        if (ImGui::Checkbox("Clipes do exercício", &on)) {
            sequencing->setClipSequence(on);
            m_dirty = true;
        }
        tooltip("A pose alvo que o treino segue a cada momento (o clipe de referência do exercício: a passada, o idle alerta "
                "depois de parar, o parado), no lugar do corpo da rede. Desmarcado: a rede, como no treino.");
        if (!sequencing->kinematicError().empty()) ImGui::TextColored(WarningText, "%s", sequencing->kinematicError().c_str());
        ImGui::SameLine();
        // (08/11 18h05, ele: "um checkbox ao lado do Clipes do exercicio, chamado empurroes".)
        bool pushes = sequencing->pushes();
        if (ImGui::Checkbox("Empurrões", &pushes)) {
            sequencing->setPushes(pushes);
            m_dirty = true;
        }
        tooltip("As pancadas e os empurrões do treino, no nível de agora de cada grupo, em todo episódio. No treino, só uma "
                "parte dos episódios tem empurrão.");
        ImGui::SameLine();
    }
    // (07/11: arquivado - ele: "esse Motion Matching esta muito problematico, entao ele fica arquivado por hora".)
    constexpr bool MotionMatchingArchived = true;
    if (auto* matching = dynamic_cast<TrackingViewer3D*>(m_locomotion.get());
        !MotionMatchingArchived && matching != nullptr && TrackingViewer3D::isGetUpDrill(m_viewerDrill)) {
        bool kinematic = matching->kinematicMotionMatching();
        if (ImGui::Checkbox("Motion Matching (cinemático)", &kinematic)) {
            matching->setKinematicMotionMatching(kinematic);
            m_dirty = true;
        }
        tooltip("Temporário: o boneco no quadro que o Motion Matching escolhe a cada decisão para o pedido do exercício "
                "(os bancos densos do Game Animation Sample), sem física e sem a rede - o que o juiz compara.");
        if (!matching->kinematicError().empty()) ImGui::TextColored(WarningText, "%s", matching->kinematicError().c_str());
        ImGui::SameLine();
    }
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
        else if (m_viewerDrill == TrackingViewer3D::AvoidFallDrill || TrackingViewer3D::isAvoidSceneryDrill(m_viewerDrill))
            label = m_policyText + "\nevitou a queda " + count + " empurrões";
        else if (m_viewerDrill == TrackingViewer3D::BalanceDrill)
            label = m_policyText + "\nnão caiu " + count + " empurrões";
        else if (m_viewerDrill == TrackingViewer3D::EquilibriumDrill)
            label = m_policyText + "\nnão caiu " + count + " episódios com empurrões";
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
        if (TrackingViewer3D::sceneryPresetOf(m_viewerDrill) >= 0 || headingDrill(m_viewerDrill)) {
            const auto at = std::find(m_drillIds.begin(), m_drillIds.end(), m_viewerDrill);
            if (at != m_drillIds.end()) label += "\n" + m_drillNames[static_cast<std::size_t>(at - m_drillIds.begin())];
        }
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

// A forca auxiliar e o que a mao faz de verdade: a sustentacao media e o pico (p90) em % do peso do corpo do treino.
std::string ReinforcementLearningScreen::assistText(const GetUpLive& g) {
    constexpr double Weight = 75.0 * 9.81;
    if (!g.drillAssist.empty()) {
        double low = 1.0, high = 0.0, sum = 0.0;
        for (const auto& [id, value] : g.drillAssist) {
            low = std::min(low, value);
            high = std::max(high, value);
            sum += value;
        }
        std::string text = "dinâmica, média " + decimal(100.0 * sum / static_cast<double>(g.drillAssist.size()), 0) + "% (de "
            + decimal(100.0 * low, 0) + " a " + decimal(100.0 * high, 0) + "%)";
        if (g.handMean >= 0.0 && g.handPeak >= 0.0)
            text += " · sustenta " + decimal(100.0 * g.handMean / (75.0 * 9.81), 0) + "% do peso (pico " + decimal(100.0 * g.handPeak / (75.0 * 9.81), 0) + "%)";
        return text;
    }
    std::string text = decimal(g.assist * 100.0, 0) + "%";
    if (g.runAssist >= 0.0) text += " · corrida " + decimal(g.runAssist * 100.0, 0) + "%";
    if (g.maneuverAssist >= 0.0) text += " · manobras " + decimal(g.maneuverAssist * 100.0, 0) + "%";
    if (g.handMean >= 0.0 && g.handPeak >= 0.0)
        text += " · sustenta " + decimal(100.0 * g.handMean / Weight, 0) + "% do peso (pico " + decimal(100.0 * g.handPeak / Weight, 0) + "%)";
    return text;
}

void ReinforcementLearningScreen::refreshGetUpLive() {
    GetUpLive live;
    const std::filesystem::path log = TrackingViewer3D::getUpLog(getUpRunDirectory());
    live.run = log.parent_path().filename().string();
    std::ifstream statusFile(log.parent_path() / "status.json");
    const nlohmann::json status = nlohmann::json::parse(statusFile, nullptr, false);
    if (!status.is_discarded() && status.is_object()) {
        live.known = true;
        live.state = status.value("state", std::string());
        live.iteration = status.value("iteration", std::uint64_t { 0 });
        live.sps = status.value("sps", 0.0);
        live.alive = trainerProcessAlive3D(status.value("pid", 0), live.run);
    }
    {
        std::ifstream runFile(log.parent_path() / "run.json");
        const nlohmann::json run = nlohmann::json::parse(runFile, nullptr, false);
        // (04/11 12h50: sem as chaves, so a primeira linha ficava no if - sem o run.json, ou lido no meio de uma gravacao do
        // treinador, o run["v2"] de um json descartado derrubava a tela.)
        if (!run.is_discarded() && run.is_object() && run.contains("v2") && run["v2"].is_object()) {
            const nlohmann::json& v2 = run["v2"];
            live.assist = v2.value("ajuda_locomocao", 0.0);
            live.runAssist = v2.value("ajuda_corrida", -1.0);
            live.maneuverAssist = v2.value("ajuda_manobra", -1.0);
            if (const auto drills = v2.find("ajuda_exercicios"); drills != v2.end() && drills->is_object())
                for (const auto& [id, value] : drills->items())
                    if (value.is_number()) live.drillAssist[id] = value.get<double>();
            // (A fatia da locomocao no treino: os rumos, o giro e o parado; o resto sao as quedas.)
            live.locomotionShare = v2.value("locomocao", 0.0) + v2.value("giro", 0.0) + v2.value("comeco_de_pe", 0.0);
        }
    }
    // (O ultimo teste: a ultima linha "aval" nos ultimos ~256 kB do log.)
    std::ifstream file(log, std::ios::binary | std::ios::ate);
    if (file) {
        const std::streamoff size = file.tellg();
        file.seekg(std::max<std::streamoff>(0, size - 262144));
        std::string line;
        nlohmann::json last;
        while (std::getline(file, line)) {
            if (line.find("\"aval\": true") != std::string::npos || line.find("\"aval\":true") != std::string::npos) {
                const nlohmann::json row = nlohmann::json::parse(line, nullptr, false);
                if (!row.is_discarded() && row.is_object()) last = row;
            } else if (line.find("rastreio_mao_sustenta_p90") != std::string::npos) {
                // (A mao de verdade: a ultima linha do treino que a mediu.)
                const nlohmann::json row = nlohmann::json::parse(line, nullptr, false);
                if (!row.is_discarded() && row.is_object()) {
                    live.handMean = row.value("rastreio_mao_sustenta_n", -1.0);
                    live.handPeak = row.value("rastreio_mao_sustenta_p90", -1.0);
                }
            }
        }
        const auto number = [&](const char* key) {
            return last.contains(key) && last[key].is_number() ? last[key].get<double>() : -1.0;
        };
        if (last.is_object()) {
            live.stood = number("levantou");
            live.seconds = number("tempo");
            live.moving = number("mov_levantou");
            live.avoided = number("evitar_evitou");
            const double hora = number("hora");
            if (hora > 0.0) {
                const std::time_t t = static_cast<std::time_t>(hora);
                char text[16];
                std::strftime(text, sizeof(text), "%H:%M", std::localtime(&t));
                live.testTime = text;
            }
        }
    }
    m_getUpLive = std::move(live);
}

std::string ReinforcementLearningScreen::getUpLiveText() const {
    const GetUpLive& g = m_getUpLive;
    if (!g.known) return "sem o treino do levantar";
    std::string scenario;
    if (TrackingViewer3D::sceneryPresetOf(m_viewerDrill) >= 0 || headingDrill(m_viewerDrill)) {
        const float rate = lastFinite(TrackingViewer3D::isAvoidSceneryDrill(m_viewerDrill)
            ? m_getUpCurve.sceneryAvoided : m_getUpCurve.sceneryStood);
        scenario = ", cenário " + (std::isfinite(rate) ? decimal(rate, 0) + "%" : std::string("sem dados"));
    }
    return g.run + " " + (g.alive ? g.state : std::string("parado")) + ", rodada " + std::to_string(g.iteration)
        + ", levantou " + decimal(g.stood * 100.0, 0) + "% em " + decimal(g.seconds, 2) + " s, em movimento "
        + decimal(g.moving * 100.0, 0) + "%, evitou " + (g.avoided >= 0.0 ? decimal(g.avoided * 100.0, 0) + "%" : "-")
        + " (teste " + g.testTime + ")" + (g.assist > 0.0 ? ", força auxiliar " + assistText(g) : std::string()) + (g.run == activeRun() ? " [o treino de cima]" : " [bloco proprio]") + scenario;
}

void ReinforcementLearningScreen::drawGetUpLive() {
    const GetUpLive& g = m_getUpLive;
    if (!g.known || g.run == activeRun()) return;
    ImGui::Dummy({ 1.0f, ui(4.0f) });
    ImGui::Separator();
    ImGui::TextColored(InfoText, "Levantar (%s)", g.run.c_str());
    tooltip("O treino da rede do levantar (levantar do chão, em movimento e evitar a queda), que roda junto do treino acima.");
    const StateLabel state = stateLabel(g.alive ? g.state : std::string("parado"));
    ImGui::SameLine();
    ImGui::TextColored(state.color, "%s", state.label);
    infoRow("Rodada", std::to_string(g.iteration));
    if (g.alive) infoRow("Velocidade", decimal(g.sps, 0) + " /s");
    if (g.stood >= 0.0) {
        infoRow("Levantou", decimal(g.stood * 100.0, 0) + "% · " + (g.seconds >= 0.0 ? decimal(g.seconds, 2) + " s" : "—"));
        tooltip("O último teste sem ruído: quantos ficaram de pé e o tempo médio até ficar de pé.");
    }
    if (g.moving >= 0.0) infoRow("Em movimento", decimal(g.moving * 100.0, 0) + "%");
    if (g.avoided >= 0.0) infoRow("Evitou a queda", decimal(g.avoided * 100.0, 0) + "%");
    if (!g.testTime.empty()) infoRow("Teste", g.testTime);
}

void ReinforcementLearningScreen::refreshGetUpCurve() {
    std::error_code error;
    const auto write = std::filesystem::last_write_time(TrackingViewer3D::getUpLog(getUpRunDirectory()), error);
    if (error) {
        m_getUpCurve = {};
        return;
    }
    if (write == m_getUpCurve.write && m_getUpCurve.window == m_chartWindow && m_getUpCurve.drill == m_viewerDrill) return;
    GetUpCurve curve;
    curve.drill = m_viewerDrill;
    curve.write = write;
    curve.window = m_chartWindow;
    // (01/11, ele: "nao consigo filtrar por temporalidade, nem mostra a hora do dado": a janela do periodo, contada da
    // linha mais nova, e no maximo maxPoints pontos - um a cada passo; as avaliacoes de antes da janela seguem valendo.)
    std::vector<std::string> lines;
    {
        std::ifstream all(TrackingViewer3D::getUpLog(getUpRunDirectory()));
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
    float movingTest = none, movingSpeedTest = none, spineTest = none, capacityZero = none, capacitySpeed = none;
    // (06/11) O levantar pela avaliacao fixa (64 quedas, a rede do levantar sozinha, sem clipe e sem mao), repetida ate a seguinte.
    float evalStood = none, evalRose = none, evalSeconds = none, evalPose = none, judgeValue = none;
    const std::string capacityKey = scenarioKey(m_viewerDrill);
    // (07/11) O treino dos levantares: de costas e de brucos separados; o correndo e o "mov_" do treino.
    const std::string trainPrefix = m_viewerDrill == TrackingViewer3D::LyingBackDrill ? "treino_costas_"
        : m_viewerDrill == TrackingViewer3D::LyingFrontDrill                       ? "treino_brucos_"
        : m_viewerDrill == TrackingViewer3D::MovingGetUpDrill                      ? "mov_"
                                                                                   : "";
    const auto capacityOf = [&](const nlohmann::json& row, const char* tag, const char* field) {
        const auto all = row.find("capacidade");
        if (all == row.end() || !all->is_object() || !all->contains(tag) || !(*all)[tag].is_object()) return none;
        const nlohmann::json& part = (*all)[tag];
        return part.contains(capacityKey) && part[capacityKey].is_object() ? percent(part[capacityKey], field) : none;
    };
    std::array<float, LocomotionLogKeys.size()> locoMainTest, locoUpTest;
    locoMainTest.fill(none);
    locoUpTest.fill(none);
    std::uint64_t plottedIteration = 0;
    for (std::string line; std::getline(file, line);) {
        const nlohmann::json row = nlohmann::json::parse(line, nullptr, false);
        if (row.is_discarded() || !row.is_object() || !row.contains("atualizacao")) continue;
        if (row.value("aval", false)) {
            movingTest = percent(row, "mov_chegou");
            movingSpeedTest = plain(row, "mov_vel_rumo");
            spineTest = percent(row, "parado_coluna");
            if (row.contains("capacidade")) {
                capacityZero = capacityOf(row, "0", "de_pe");
                capacitySpeed = capacityOf(row, "0", "vel");
            }
            // (07/11: o levantar deitado pela avaliacao dele - costas_ e brucos_; o levantar do chao pela das quedas.)
            const std::string evalPrefix = m_viewerDrill == TrackingViewer3D::LyingBackDrill ? "costas_"
                : m_viewerDrill == TrackingViewer3D::LyingFrontDrill ? "brucos_"
                : m_viewerDrill == TrackingViewer3D::MovingGetUpDrill ? "mov_" : "";
            // (A nota do juiz no exercicio, de -1 a 1 - parece clipe perto de 1 -, em %.)
            if (const auto j = row.find("juiz_exercicios");
                j != row.end() && j->is_object() && j->contains(capacityKey) && (*j)[capacityKey].is_number())
                judgeValue = 50.0f * ((*j)[capacityKey].get<float>() + 1.0f);
            if (row.contains(evalPrefix + "de_pe")) {
                evalStood = percent(row, (evalPrefix + "de_pe").c_str());
                evalRose = percent(row, (evalPrefix + "levantou").c_str());
                evalSeconds = plain(row, (evalPrefix + "tempo").c_str());
                evalPose = percent(row, (evalPrefix + "pose").c_str());
            }
            for (std::size_t k = 0; k < LocomotionLogKeys.size(); ++k) {
                const LocomotionLogKeys3D& keys = LocomotionLogKeys[k];
                locoMainTest[k] = keys.percent ? percent(row, keys.mainTest) : plain(row, keys.mainTest);
                locoUpTest[k] = percent(row, keys.upTest);
            }
            // A avaliacao vem depois da linha de treino da mesma atualizacao.
            if (!curve.times.empty() && row.value("atualizacao", std::uint64_t { 0 }) == plottedIteration) {
                curve.reachedTest.back() = movingTest;
                curve.towardSpeedTest.back() = movingSpeedTest;
                curve.spineTest.back() = spineTest;
                curve.capacityZero.back() = capacityZero;
                curve.capacitySpeed.back() = capacitySpeed;
                curve.evalStood.back() = evalStood;
                curve.judge.back() = judgeValue;
                curve.evalRose.back() = evalRose;
                curve.evalSeconds.back() = evalSeconds;
                curve.evalPose.back() = evalPose;
                for (std::size_t k = 0; k < LocomotionLogKeys.size(); ++k) {
                    curve.locoMainTest[k].back() = locoMainTest[k];
                    curve.locoUpTest[k].back() = locoUpTest[k];
                }
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
            plottedIteration = row.value("atualizacao", std::uint64_t { 0 });
        }
        {
            const nlohmann::json none = nlohmann::json::object();
            const auto scenarios = row.find("cenarios");
            const auto& pushed = scenarios != row.end() && scenarios->is_object() && scenarios->contains("lev_evitar")
                && (*scenarios)["lev_evitar"].is_object() ? (*scenarios)["lev_evitar"] : none;
            curve.avoidStill.push_back(percent(pushed, "estabilidade"));
        }
        curve.avoidStanding.push_back(percent(row, "evitar_de_pe"));
        curve.capacityZero.push_back(capacityZero);
        curve.capacitySpeed.push_back(capacitySpeed);
        curve.evalStood.push_back(evalStood);
        curve.judge.push_back(judgeValue);
        curve.trainStood.push_back(trainPrefix.empty() ? none : percent(row, (trainPrefix + "de_pe").c_str()));
        curve.trainSeconds.push_back(trainPrefix.empty() ? none : plain(row, (trainPrefix + "tempo").c_str()));
        curve.trainRose.push_back(trainPrefix.empty() ? none : percent(row, (trainPrefix + "levantou").c_str()));
        curve.evalRose.push_back(evalRose);
        curve.evalSeconds.push_back(evalSeconds);
        curve.evalPose.push_back(evalPose);
        if (TrackingViewer3D::sceneryPresetOf(m_viewerDrill) >= 0 || headingDrill(m_viewerDrill)) {
            const nlohmann::json empty = nlohmann::json::object();
            const auto scenarios = row.find("cenarios");
            const std::string key = scenarioKey(m_viewerDrill);
            const auto& selected = scenarios != row.end() && scenarios->is_object() && scenarios->contains(key)
                && (*scenarios)[key].is_object() ? (*scenarios)[key] : empty;
            // (04/11: nos rumos, a fracao que chega ao fim - o fim antes da hora sem queda, o corpo virado, nao aparecia no sem cair.)
            curve.sceneryStood.push_back(percent(selected, headingDrill(m_viewerDrill) ? "ate_o_fim" : "levantou"));
            curve.scenerySeconds.push_back(plain(selected, headingDrill(m_viewerDrill) ? "vel_rumo" : TrackingViewer3D::isAvoidSceneryDrill(m_viewerDrill) ? "recompos_s" : "tempo"));
            curve.sceneryAvoided.push_back(percent(selected, "evitou"));
            curve.sceneryImitation.push_back(percent(selected, "semelhanca"));
            curve.sceneryStability.push_back(percent(selected, "estabilidade"));
            const auto assists = row.find("ajuda_exercicios");
            curve.sceneryAssist.push_back(assists != row.end() && assists->is_object() ? percent(*assists, scenarioKey(m_viewerDrill))
                                                                                    : std::numeric_limits<float>::quiet_NaN());
            curve.sceneryRatio.push_back(percent(selected, "vel_pedido"));
            const auto demands = row.find("exigencias");
            const auto& demand = demands != row.end() && demands->is_object() && demands->contains(key)
                && (*demands)[key].is_object() ? (*demands)[key] : empty;
            curve.requiredStood.push_back(percent(demand, "fim"));
            curve.requiredRatio.push_back(percent(demand, "vel"));
            curve.requiredImitation.push_back(percent(demand, "semelhanca"));
            curve.requiredStability.push_back(percent(demand, "estabilidade"));
            curve.requiredSeconds.push_back(none);
        } else if (m_viewerDrill == TrackingViewer3D::GetUpDrill || m_viewerDrill == TrackingViewer3D::EquilibriumDrill) {
            // (06/11 05h30, ele: os graficos do levantar e do equilibrio "iguais aos outros, com as mesmas informacoes porem
            // voltadas ao caso": as cinco medidas do modulo unico. O levantar pelo dominio do treino - o que o portao usa -, com as
            // exigencias; o equilibrio pelas medidas dele, e no lugar da forca o nivel dos empurroes do curriculo.)
            const nlohmann::json empty = nlohmann::json::object();
            const auto pick = [&row, &empty](const char* section) -> const nlohmann::json& {
                const auto it = row.find(section);
                return it != row.end() && it->is_object() && it->contains("lev_levantar") && (*it)["lev_levantar"].is_object()
                    ? (*it)["lev_levantar"] : empty;
            };
            if (m_viewerDrill == TrackingViewer3D::GetUpDrill) {
                const nlohmann::json &domain = pick("dominio"), &demand = pick("exigencias");
                const auto assists = row.find("ajuda_exercicios");
                curve.sceneryStood.push_back(percent(domain, "fim"));
                curve.scenerySeconds.push_back(plain(domain, "tempo"));
                curve.sceneryImitation.push_back(percent(domain, "semelhanca"));
                curve.sceneryStability.push_back(percent(domain, "estabilidade"));
                curve.sceneryAssist.push_back(assists != row.end() && assists->is_object() ? percent(*assists, "lev_levantar") : none);
                curve.requiredStood.push_back(percent(demand, "fim"));
                curve.requiredSeconds.push_back(plain(demand, "tempo"));
                curve.requiredImitation.push_back(percent(demand, "semelhanca"));
                curve.requiredStability.push_back(percent(demand, "estabilidade"));
            } else {
                curve.sceneryStood.push_back(percent(row, "equilibrio_sem_cair"));
                curve.scenerySeconds.push_back(plain(row, "equilibrio_passos_rec"));
                curve.sceneryImitation.push_back(percent(row, "equilibrio_pose"));
                curve.sceneryStability.push_back(percent(row, "equilibrio_quietou"));
                curve.sceneryAssist.push_back(percent(row, "equilibrio_nivel"));
                for (std::vector<float>* v : { &curve.requiredStood, &curve.requiredSeconds, &curve.requiredImitation, &curve.requiredStability })
                    v->push_back(none);
            }
            curve.sceneryAvoided.push_back(none);
            curve.sceneryRatio.push_back(none);
            curve.requiredRatio.push_back(none);
        } else {
            curve.sceneryStood.push_back(percent(row, "queda_sujo_levantou"));
            curve.scenerySeconds.push_back(plain(row, "queda_sujo_tempo"));
            curve.sceneryAvoided.push_back(percent(row, "evitar_evitou_ambientes"));
            // (05/11: o levantar pelo clipe - a forca e a semelhanca dele, lev_levantar.)
            const nlohmann::json empty = nlohmann::json::object();
            const auto assists = row.find("ajuda_exercicios");
            const auto domains = row.find("dominio");
            const auto demands = row.find("exigencias");
            const bool getUp = m_viewerDrill == TrackingViewer3D::GetUpDrill;
            const auto& domain = getUp && domains != row.end() && domains->is_object() && domains->contains("lev_levantar")
                && (*domains)["lev_levantar"].is_object() ? (*domains)["lev_levantar"] : empty;
            const auto& demand = getUp && demands != row.end() && demands->is_object() && demands->contains("lev_levantar")
                && (*demands)["lev_levantar"].is_object() ? (*demands)["lev_levantar"] : empty;
            curve.sceneryImitation.push_back(percent(domain, "semelhanca"));
            curve.sceneryStability.push_back(std::numeric_limits<float>::quiet_NaN());
            curve.sceneryAssist.push_back(getUp && assists != row.end() && assists->is_object() ? percent(*assists, "lev_levantar")
                                                                                              : std::numeric_limits<float>::quiet_NaN());
            curve.requiredImitation.push_back(percent(demand, "semelhanca"));
            for (std::vector<float>* v : { &curve.sceneryRatio, &curve.requiredStood, &curve.requiredRatio, &curve.requiredStability,
                     &curve.requiredSeconds })
                v->push_back(std::numeric_limits<float>::quiet_NaN());
        }
        // (O levantar v2: o em movimento e o evitar a queda, no treino e, tracejado, nas avaliacoes fixas.)
        curve.reached.push_back(percent(row, "mov_chegou"));
        curve.reachedTest.push_back(movingTest);
        curve.movingStood.push_back(percent(row, "mov_de_pe"));
        curve.towardSpeed.push_back(plain(row, "mov_vel_rumo"));
        curve.towardSpeedTest.push_back(movingSpeedTest);
        curve.avoided.push_back(percent(row, "evitar_evitou"));
        curve.recomposeSeconds.push_back(plain(row, "evitar_recompos_s"));
        // (O equilibrio parado e a pose de pe perto do parado em alerta.)
        curve.spine.push_back(percent(row, "de_pe_coluna"));
        curve.spineTest.push_back(spineTest);
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
    // (06/11 05h30, ele: "deveria ter um so modulo de grafico para tudo, nao gambiarra assim") O modulo dos exercicios: o
    // "ate o fim" e o segundo grafico do caso lado a lado; no expandido, a semelhanca e a estabilidade e, na largura toda, a
    // forca (ou o nivel do curriculo). Cada grafico com a linha vermelha do exigido no expandido, quando ha.
    // (04/11, ele: "mais dois graficos nos expandidos"; 12h40, "mostrar exigencias"; 05h, a escala da forca pelos dados.)
    struct Panel {
        const char* title;
        const std::vector<float>* values;
        const std::vector<float>* required;
        ImVec4 color;
        const char* label;
        const char* format;
        const char* unit;
        float high; // > 0: de 0 a este; 0: pelos dados (de 0 a um numero redondo logo acima do maior)
        const char* empty;
        // (07/11) O teste sem ruido do mesmo numero, tracejado na mesma cor. (07/11 06h, ele: "porque estamos dizendo coisas de
        // professor sendo que ele saiu do jogo?... mostrasse a evolucao REAL do treino": so o treino e o teste - sem a
        // professora, a forca auxiliar e a capacidade a 15%.)
        const std::vector<float>* test = nullptr;
    };
    const ImVec4 pink { 0.95f, 0.55f, 0.80f, 1.0f };
    const auto panels = [&](const std::vector<Panel>& p) {
        // (06/11 05h40, ele: "era preferivel um abaixo do outro, com alturas padronizadas, do que lado a lado": cada grafico na
        // largura toda, todos da mesma altura - o espaco dividido entre eles, entre 5 e 11 linhas de texto.)
        // (No recolhido, embaixo do visualizador, a faixa e baixa: os dois lado a lado - empilhados ficavam achatados.)
        const int count = m_chartsExpanded ? static_cast<int>(p.size()) : std::min(2, static_cast<int>(p.size()));
        const float title = ImGui::GetTextLineHeightWithSpacing() + ImGui::GetStyle().ItemSpacing.y;
        const float font = ImGui::GetFontSize();
        const int rows = m_chartsExpanded ? count : 1;
        const ImVec2 cell { m_chartsExpanded ? ImGui::GetContentRegionAvail().x : size.x,
            std::clamp((height - static_cast<float>(rows) * title) / static_cast<float>(rows), 5.0f * font, 11.0f * font) };
        const float top = ImGui::GetCursorPosY();
        const bool demand = m_chartsExpanded && m_showRequirements;
        const ImVec4 red { 0.95f, 0.38f, 0.38f, 1.0f };
        for (int k = 0; k < count; ++k) {
            const Panel& item = p[static_cast<std::size_t>(k)];
            std::vector<ChartSeries> series { { item.values, item.color, false, item.label } };
            if (demand && item.required) series.push_back({ item.required, red, true, "exigido" });
            if (item.test && anyFinite(*item.test)) series.push_back({ item.test, item.color, true, "teste sem ruído" });
            float low = 0.0f, high = item.high;
            // (Pelos dados com o teto minimo de 1: a forca toda em 0 nao vira uma escala de 0,001.)
            if (high <= 0.0f)
                std::tie(low, high) = item.required && demand ? niceRange({ item.values, item.required }, 1.0f, 0.0f)
                                                              : niceRange({ item.values }, 1.0f, 0.0f);
            if (!m_chartsExpanded && k == 1) {
                ImGui::SameLine();
                ImGui::SetCursorPosY(top);
            }
            ImGui::BeginGroup();
            ImGui::TextColored(InfoText, "%s", item.title);
            const std::string id = "##modulo" + std::to_string(k);
            // (06/11 18h25, ele: "nao sei exatamente o que estou vendo": primeiro o teste - o que importa -, depois o treino.)
            std::string headline = last(*item.values, item.format);
            if (item.test && anyFinite(*item.test))
                headline = "teste " + last(*item.test, item.format) + (anyFinite(*item.values) ? " · treino " + headline : std::string());
            lineChart(id.c_str(), series, low, high, cell, headline.c_str(), item.unit, &c.times, true, item.empty);
            ImGui::EndGroup();
        }
    };
    if (headingDrill(m_viewerDrill)) {
        // (04/11 12h40, ele: "o grafico de velocidade dizer o quanto % ele atendeu da velocidade pedida, e nao o m/s cru" - a
        // media da fracao de cada episodio, ate 100%: a medida da regua da forca.)
        const bool still = m_viewerDrill == "lev_giro" || m_viewerDrill == TrackingViewer3D::BalanceDrill;
        // (O teste: o fixo de cada exercicio, sem ruido e sem forca - "capacidade" a 0% das avaliacoes.)
        std::vector<Panel> shown { Panel { "Até o fim neste exercício (%)", &c.sceneryStood, &c.requiredStood, green, "treino",
                                       "%.0f%%", "%", 100.0f, "sem dados deste exercício", &c.capacityZero },
            Panel { "Velocidade neste exercício (% do pedido)", &c.sceneryRatio, &c.requiredRatio, cyan, "treino", "%.0f%%",
                "%", 100.0f, still ? "sem pedido de velocidade" : "sem dados deste exercício", &c.capacitySpeed },
            Panel { "Estabilidade (%)", &c.sceneryStability, &c.requiredStability, amber, "estabilidade", "%.0f%%", "%", 100.0f,
                "sem dados deste exercício" },
            Panel { "Parecido com os clipes, para o juiz (%)", &c.judge, nullptr, pink, "juiz (teste)", "%.0f%%", "%", 100.0f,
                "sem a nota do juiz ainda (sai nas avaliações)" } };
        // (07/11 06h25, ele: "ha dois graficos, semelhanca com o clipe e parecido com os clipes para o juiz... deixe so o que
        // estamos usando de fato": andando, a nota do rastreio v14 nao tem a semelhanca - a forma e so do juiz; no parado e no
        // giro ela vale 65% da nota.)
        if (still)
            shown.insert(shown.begin() + 2, Panel { "Semelhança com o clipe (%)", &c.sceneryImitation, &c.requiredImitation,
                                                    violet, "semelhança", "%.0f%%", "%", 100.0f, "sem dados deste exercício" });
        panels(shown);
        return;
    }
    if (m_viewerDrill == TrackingViewer3D::GetUpDrill || m_viewerDrill == TrackingViewer3D::LyingBackDrill
        || m_viewerDrill == TrackingViewer3D::LyingFrontDrill) {
        // (06/11 13h41: o levantar voltou a rede livre - sem clipe e sem mao; a medida e a avaliacao fixa de 64 quedas, a mesma
        // desde 31/10 - o dominio do levantar pelo clipe deixou de existir.)
        // (07/11, ele: "os levantares nao estao mostrando os dados completos": o treino de cada um, cheio, e o teste fixo,
        // tracejado; e a nota do juiz.)
        panels({ Panel { "Levanta e fica de pé no fim (%)", &c.trainStood, nullptr, green, "treino", "%.0f%%", "%", 100.0f,
                     "sem dados deste levantar", &c.evalStood },
            Panel { "Tempo para levantar (s)", &c.trainSeconds, nullptr, cyan, "treino", "%.2f s", "s", 0.0f,
                "sem dados deste levantar", &c.evalSeconds },
            Panel { "Parecido com os clipes, para o juiz (%)", &c.judge, nullptr, pink, "juiz (teste)", "%.0f%%", "%", 100.0f,
                "sem a nota do juiz ainda (sai nas avaliações)" },
            Panel { "Pose de pé no fim, perto do parado (%)", &c.evalPose, nullptr, violet, "teste", "%.0f%%", "%",
                100.0f, "sem avaliação do levantar" },
            Panel { "Levantou alguma vez (%)", &c.trainRose, nullptr, amber, "treino", "%.0f%%", "%", 100.0f,
                "sem dados deste levantar", &c.evalRose } });
        return;
    }
    if (m_viewerDrill == TrackingViewer3D::EquilibriumDrill) {
        panels({ Panel { "Sem cair com empurrões (%)", &c.sceneryStood, nullptr, green, "sem cair", "%.0f%%", "%", 100.0f,
                     "sem dados do equilíbrio" },
            Panel { "Passos de recuperação por episódio", &c.scenerySeconds, nullptr, cyan, "passos", "%.1f", "", 0.0f,
                "sem dados do equilíbrio" },
            Panel { "Pose de pé, perto do parado (%)", &c.sceneryImitation, nullptr, violet, "pose", "%.0f%%", "%",
                100.0f, "sem dados do equilíbrio" },
            Panel { "Quietou depois do empurrão (%)", &c.sceneryStability, nullptr, amber, "quietou", "%.0f%%", "%", 100.0f,
                "sem dados do equilíbrio" },
            Panel { "Nível dos empurrões (% da faixa do treino)", &c.sceneryAssist, nullptr, blue, "nível", "%.0f%%", "%", 100.0f,
                "sem o currículo dos empurrões" } });
        return;
    }
    if (m_viewerDrill == TrackingViewer3D::SceneryDrill || TrackingViewer3D::isAvoidSceneryDrill(m_viewerDrill)
        || TrackingViewer3D::sceneryPresetOf(m_viewerDrill) >= 0) {
        const bool avoid = TrackingViewer3D::isAvoidSceneryDrill(m_viewerDrill);
        const bool individual = TrackingViewer3D::sceneryPresetOf(m_viewerDrill) >= 0;
        const auto& rate = avoid ? c.sceneryAvoided : c.sceneryStood;
        ImGui::BeginGroup();
        ImGui::TextColored(InfoText, "%s", individual ? (avoid ? "Evitou neste cenário (%)" : "Levantou neste cenário (%)") : (avoid ? "Evitou nos ambientes · agregado (%)" : "Levantou nos ambientes · agregado (%)"));
        tooltip(individual ? "Somente episodios deste layout. O historico anterior a coleta individual fica sem dados." : "Agregado dos episodios de treino ambientados.");
        lineChart("##ambientes_taxa", { { &rate, green, false, individual ? "cenário selecionado" : "treino com cenário" } }, 0.0f, 100.0f,
            size, last(rate, "%.0f%%").c_str(), "%", &c.times, true, "sem dados ainda");
        ImGui::EndGroup();
        ImGui::SameLine();
        ImGui::SetCursorPosY(rowTop);
        ImGui::BeginGroup();
        const auto& other = avoid && !individual ? c.avoided : c.scenerySeconds;
        ImGui::TextColored(InfoText, "%s", individual ? (avoid ? "Tempo para recompor neste cenário (s)" : "Tempo para levantar neste cenário (s)") : (avoid ? "Evitou em todos os episodios (%)" : "Tempo para levantar nos ambientes (s)"));
        lineChart("##ambientes_comparacao", { { &other, violet, false, individual ? "cenário selecionado" : avoid ? "todos os cenários" : "treino com cenário" } },
            0.0f, avoid && !individual ? 100.0f : 12.0f, size, last(other, avoid && !individual ? "%.0f%%" : "%.2f s").c_str(),
            avoid && !individual ? "%" : "s", &c.times, true, "sem dados ainda");
        ImGui::EndGroup();
        return;
    }
    if (m_viewerDrill == TrackingViewer3D::MovingGetUpDrill) {
        // (07/11: levanta deitado e corre ate um alvo a 4-8 m - o treino, cheio, e o teste fixo de 32 comecos, tracejado.)
        panels({ Panel { "Chegou ao alvo (%)", &c.reached, nullptr, green, "treino", "%.0f%%", "%", 100.0f, "sem dados ainda",
                     &c.reachedTest },
            Panel { "Velocidade rumo ao alvo (m/s)", &c.towardSpeed, nullptr, cyan, "treino", "%.2f m/s", "m/s", 6.0f,
                "sem dados ainda", &c.towardSpeedTest },
            Panel { "Parecido com os clipes, para o juiz (%)", &c.judge, nullptr, pink, "juiz (teste)", "%.0f%%", "%", 100.0f,
                "sem a nota do juiz ainda (sai nas avaliações)" },
            Panel { "Levanta e fica de pé no fim (%)", &c.trainStood, nullptr, violet, "treino", "%.0f%%", "%", 100.0f,
                "sem dados ainda", &c.evalStood },
            Panel { "Tempo para levantar (s)", &c.trainSeconds, nullptr, amber, "treino", "%.2f s", "s", 0.0f, "sem dados ainda",
                &c.evalSeconds } });
        return;
    }
    if (const std::size_t k = locomotionIndex(m_viewerDrill); k < LocomotionLogKeys.size()) {
        const std::string name = LocomotionNames[k];
        const bool still = k == 0, turn = k == 1;
        ImGui::BeginGroup();
        const std::string title = still ? "Pose do parado (%) · " + name
            : turn                      ? "Erro do rumo (graus) · " + name
                                        : "Velocidade no rumo (% da pedida) · " + name;
        ImGui::TextColored(InfoText, "%s", title.c_str());
        tooltip(still ? "De pé, no fim: a semelhança da pose com o parado e, em violeta, a da coluna (a pelve, o "
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
    // (04/11 23h, ele: "refaca completamente o grafico do levantar e do evitar queda (grafico horrivel atual nao entendo nada!!!
    // ele precisa mostrar quantos % ele consegue se levantar e permanecer de pe!!!! e o outro a velocidade com que faz isso, e
    // ai um terceiro grafico para a estabilidade dele)": tres graficos lado a lado, o treino cheio e o teste sem ruido tracejado.)
    const float third = (ImGui::GetContentRegionAvail().x - 2.0f * spacing) / 3.0f;
    const ImVec2 cell3 { third, m_chartsExpanded && m_viewerDrill != TrackingViewer3D::AvoidFallDrill
        ? std::max(height / 3.0f - 2.0f * ImGui::GetTextLineHeightWithSpacing(), ui(40.0f)) : size.y };
    const auto column = [&](bool first) {
        if (!first) {
            ImGui::SameLine();
            ImGui::SetCursorPosY(rowTop);
        }
        ImGui::BeginGroup();
    };
    if (m_viewerDrill == TrackingViewer3D::AvoidFallDrill) {
        column(true);
        ImGui::TextColored(InfoText, "%s", "Evita a queda e fica de pé (%)");
        // (05/11 01h40, ele: "esses graficos do levantar estao confusos... tira esse (teste), eu quero ver os dados do treino":
        // so o treino nos graficos do levantar e do evitar.)
        tooltip("De pé, leva um empurrão forte. Verde: não caiu (o tronco nunca no chão). Azul: de pé no fim do episódio "
                "(contando quem caiu e levantou). Dados do treino.");
        lineChart("##evitar_taxa", { { &c.avoided, green, false, "evitou a queda" }, { &c.avoidStanding, cyan, false, "de pé no fim" } },
            0.0f, 100.0f, cell3, ("evitou " + last(c.avoided, "%.0f%%") + " · de pé no fim " + last(c.avoidStanding, "%.0f%%")).c_str(),
            "%", &c.times, true, "sem dados ainda");
        ImGui::EndGroup();
        column(false);
        ImGui::TextColored(InfoText, "%s", "Tempo para se recompor (s)");
        tooltip("Do fim do empurrão até estar de pé, quieto e na pose: menor é melhor. Sem dados enquanto nenhum episódio se recompõe.");
        lineChart("##evitar_tempo", { { &c.recomposeSeconds, violet, false, "treino" } },
            0.0f, 4.0f, cell3, last(c.recomposeSeconds, "%.2f s").c_str(), "s", &c.times, true,
            "nenhum se recompôs ainda");
        ImGui::EndGroup();
        column(false);
        ImGui::TextColored(InfoText, "%s", "Estabilidade depois do empurrão (%)");
        tooltip("A estabilidade do episódio de pé depois do empurrão (100 = firme e quieto o tempo todo).");
        lineChart("##evitar_estab", { { &c.avoidStill, amber, false, "estabilidade" } }, 0.0f, 100.0f, cell3,
            last(c.avoidStill, "%.0f%%").c_str(), "%", &c.times, true, "sem dados ainda");
        ImGui::EndGroup();
        return;
    }
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
