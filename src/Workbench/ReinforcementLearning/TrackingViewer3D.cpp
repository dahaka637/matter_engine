#include "Workbench/ReinforcementLearning/TrackingViewer3D.hpp"

#include "Engine/Athlete/Body/GetUpPolicy3D.hpp"
#include "Engine/RL/Body/RLBody3D.hpp"
#include "Engine/RL/Policy/PolicyRuntime3D.hpp"
#include "Engine/RL/Supervisor/TrainerLink3D.hpp"
#include "Engine/Tracking/TrackingGetUpAmpTraining3D.hpp"
#include "Engine/Tracking/TrackingTraining3D.hpp"
#include "Workbench/ReinforcementLearning/LocomotionViewer3D.hpp"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <array>
#include <cctype>
#include <cstdio>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <random>
#include <string_view>
#include <vector>

namespace MatterEngine {
namespace {

// A velocidade de quando a tela nao pede outra (a do prototipo, G1).
constexpr float DefaultSpeed = 3.0f;

// O treino do levantar livre (31/10: uma rede para qualquer queda, tools/rl/levantar_amp.py).
std::filesystem::path getUpRun(const std::filesystem::path& selected = {}) {
    if (!selected.empty()) return selected;
    const auto runs = defaultTrainerPaths3D({}).data / "runs";
    // (31/10 17h: o levantar v2 - com o levantar em movimento e o evitar a queda -, quando existe; 01/11, a rl_final, a
    // rede que levanta, se protege e se locomove, quando existe.)
    if (std::filesystem::exists(runs / "rl_final")) return runs / "rl_final";
    return std::filesystem::exists(runs / "levantar_v2") ? runs / "levantar_v2" : runs / "levantar_livre";
}

// O treino da rede da locomocao ligado ao do levantar (o "getUpHandoff" na tarefa; o de tarefa mais nova).
std::filesystem::path partnerRun() {
    std::filesystem::path best;
    std::filesystem::file_time_type bestTime {};
    std::error_code code;
    for (const auto& entry : std::filesystem::directory_iterator(getUpRun().parent_path(), code)) {
        std::ifstream file(entry.path() / "tarefa.json");
        if (!file) continue;
        const nlohmann::json task = nlohmann::json::parse(file, nullptr, false);
        if (task.is_discarded() || !task.is_object() || !task.contains("getUpHandoff") || !task["getUpHandoff"].is_object()
            || !task["getUpHandoff"].value("enabled", false))
            continue;
        std::error_code timeCode;
        const auto stamp = std::filesystem::last_write_time(entry.path() / "tarefa.json", timeCode);
        if (best.empty() || stamp > bestTime) {
            best = entry.path();
            bestTime = stamp;
        }
    }
    return best;
}

// A rede da locomocao depois da troca, parada no lugar, antes de o exercicio recomecar (s).
constexpr float PartnerSeconds = 3.0f;
// O levantar em movimento: a troca de pe por este tanto (s), mesmo andando, e a corrida ate o alvo ate esta velocidade
// (m/s; as do treino do levantar).
constexpr float MovingHandoffSeconds = 0.2f;
constexpr float PartnerRunSpeed = 3.5f;

// A forca auxiliar por exercicio do run.json (v2.ajuda_exercicios, 04/11) na ordem do drillAssist do treino; -1 sem.
std::vector<float> drillAssistOf(const nlohmann::json& v2) {
    static constexpr const char* Names[37] { "lev_trote_0", "lev_trote_1", "lev_trote_2", "lev_trote_3", "lev_trote_4",
        "lev_trote_5", "lev_trote_6", "lev_trote_7", "lev_corrida_0", "lev_corrida_1", "lev_corrida_2", "lev_corrida_3",
        "lev_corrida_4", "lev_corrida_5", "lev_corrida_6", "lev_corrida_7", "lev_troca_suave", "lev_troca_media",
        "lev_troca_forte", "lev_troca_livre", "lev_curva_suave", "lev_curva_s", "lev_curva_fechada", "lev_giro", "lev_parado",
        "lev_curva_suave_corrida", "lev_curva_s_corrida", "lev_curva_fechada_corrida", "lev_sprint", "lev_levantar",
        "lev_estrela_trote_fixo", "lev_estrela_trote_livre", "lev_estrela_trote_girando", "lev_estrela_corrida_fixo",
        "lev_estrela_corrida_livre", "lev_estrela_corrida_girando", "lev_comando" };
    std::vector<float> levels(std::size(Names), -1.0f);
    const auto it = v2.find("ajuda_exercicios");
    if (it == v2.end() || !it->is_object()) return levels;
    for (std::size_t k = 0; k < std::size(Names); ++k)
        if (it->contains(Names[k]) && (*it)[Names[k]].is_number()) levels[k] = (*it)[Names[k]].get<float>();
    return levels;
}

} // namespace

struct TrackingViewer3D::Impl {
    std::unique_ptr<TrackingTraining3D> batch;
    std::unique_ptr<PolicyRuntime3D> policy;
    std::vector<float> observation;
    std::vector<float> action;
    std::vector<float> rewards { 0.0f };
    std::vector<std::uint8_t> terminated { 0 }, truncated { 0 };
    float speed = DefaultSpeed;
    float heading = 0.0f;
    TrackingDrill3D drill = TrackingDrill3D::Move;
    // Com a tarefa, quem pede e o piloto do exercicio (a velocidade da tela nao entra).
    bool task = false;

    // O pedido do mundo do visualizador: andar no rumo, ou o exercicio parado.
    void apply() {
        if (task) return;
        if (drill == TrackingDrill3D::Move) batch->setCommand(0, speed, heading);
        else batch->setDrill(0, drill);
    }
    // O levantar: o lote do levantar livre com um mundo, como a avaliacao do treino (so quedas sorteadas: o corpo cai,
    // a rede levanta e fica de pe ate o fim do episodio, e cai de novo), com a ultima rede exportada (relida quando o
    // treino exporta outra) e a conta de quantas quedas ele levantou.
    std::filesystem::path getUpDirectory;
    std::unique_ptr<TrackingGetUpAmpTraining3D> getUp;
    std::unique_ptr<PolicyRuntime3D> getUpPolicy;
    std::filesystem::file_time_type getUpPolicyWrite {};
    std::uint32_t getUpFalls = 0, getUpStood = 0;
    std::filesystem::path assets;
    std::uint64_t seed = 1;
    std::vector<float> getUpObservation, getUpAction;
    // Qual avaliacao do levantar: 0 as quedas, 1 o levantar em movimento, 2 o evitar a queda, 3 o equilibrio parado,
    // 4 um rumo (o do exercicio), 5 o parado, 6 o giro parado suave, 7 as quedas da locomocao.
    std::uint32_t getUpKind = 0;
    std::string getUpDrill;
    // A troca para a rede da locomocao (01/11, ele: "nao observo a mudanca de rede no momento apos ele estar de pe"):
    // de pe e firme por GetUpPolicy3D::FirmSeconds (o criterio do treino), o corpo vai para o mundo do treino da rede
    // da locomocao (LocomotionViewer3D, a rede exportada dele, parado no lugar) por PartnerSeconds - se ele cair, a rede
    // do levantar assume de novo na queda iminente, como no treino - e o exercicio recomeca. No evitar a queda ele comeca
    // ali, de pe, e leva o empurrao forte (ele: "o evitar queda precisa saber operar em conjunto com a outra rede nos
    // momentos de queda iminente").
    std::filesystem::path partnerDirectory;
    std::unique_ptr<LocomotionViewer3D> partner;
    std::filesystem::file_time_type partnerWrite {};
    std::string partnerLabel;
    bool partnerActive = false;
    float partnerTime = 0.0f;
    float firmSeconds = 0.0f;
    // (A troca so depois de levantar do chao, como no treino: a cabeca abaixo de 1 m em algum momento do episodio.)
    bool wasDown = false;
    std::uint32_t restarts = 0;
    bool avoidPushed = false;
    std::uint32_t avoidRecoveries = 0;
    float avoidBackAt = -1.0f;
    // O levantar em movimento (01/11, ele: "levantar-se virando para o alvo, fazer a troca do modelo de RL e correr para
    // o alvo com rapidez"): a troca de pe por MovingHandoffSeconds, mesmo andando, e a rede da locomocao correndo ate o
    // alvo; o exercicio recomeca 1 s depois de ela chegar (ou em 6 s).
    std::optional<Vec3> runTarget;
    float arrivedAt = -1.0f;
    bool ensurePartner() {
        if (partnerDirectory.empty()) return false;
        if (!partner) {
            try {
                partner = std::make_unique<LocomotionViewer3D>(partnerDirectory / "tarefa.json", assets, "parado", seed);
            } catch (const std::exception&) {
                partnerDirectory.clear();
                return false;
            }
        }
        // (A rede nova do treino dela, quando ele exporta outra.)
        const std::filesystem::path file = partnerDirectory / "exports" / "policy_latest.mrlpolicy";
        std::error_code error;
        const auto write = std::filesystem::last_write_time(file, error);
        if (!error && (!partner->hasPolicy() || write != partnerWrite)) {
            partner->setPolicy(file, partnerLabel);
            partnerWrite = write;
        }
        return partner->hasPolicy();
    }
    // O evitar a queda: a rede da locomocao de pe na origem e, em 1 s, o empurrao forte do treino.
    void startAvoid() {
        if (TrackingViewer3D::isAvoidSceneryDrill(getUpDrill)) return;
        if (!ensurePartner()) return;
        partner->reset(seed + ++restarts);
        partnerActive = true;
        partnerTime = 0.0f;
        avoidPushed = false;
        avoidBackAt = -1.0f;
    }
    // De pe e firme no mundo do levantar: a troca.
    void checkHandoff() {
        // (Na sequencia de clipes nao ha troca de rede: os clipes seguem ate o fim do episodio.)
        if (sequenceActive) return;
        if (TrackingViewer3D::sceneryPresetOf(getUpDrill) >= 0 || getUpDrill == SceneryDrill || getUpDrill == AvoidSceneryDrill) return;
        const RagdollProfile3D& profile = getUp->profile();
        const RagdollState3D& body = getUp->body(0);
        if (body.links.empty()) return;
        const PhysicsBodyState3D& pelvis = body.links.front();
        for (std::size_t i = 0; i < profile.links.size() && i < body.links.size(); ++i)
            if (profile.links[i].id == "Head" && body.links[i].position.z < 1.0f) wasDown = true;
        const float upright =
            (pelvis.orientation * profile.links.front().modelOrientation.conjugate()).rotate({ 0.0f, 0.0f, 1.0f }).z;
        const Vec3 v = rlBodyCenterOfMassVelocity3D(profile, body.links);
        // (No levantar em movimento, de pe - a cabeca acima de 1,4 m, a pelve de 0,75 m e em pe -, mesmo andando.)
        const bool moving = getUpKind == 1;
        float head = 0.0f;
        for (std::size_t i = 0; i < profile.links.size() && i < body.links.size(); ++i)
            if (profile.links[i].id == "Head") head = body.links[i].position.z;
        const bool firm = wasDown
            && (moving ? head > 1.4f && pelvis.position.z > 0.75f && upright > 0.8f
                       : pelvis.position.z > 0.85f * profile.standingRootHeightMeters && upright > 0.85f
                           && std::hypot(v.x, v.y) < 0.5f);
        firmSeconds = firm ? firmSeconds + TrackingGetUpAmpTraining3D::TicksPerDecision * TrackingGetUpAmpTraining3D::TickSeconds
                           : 0.0f;
        if (firmSeconds < (moving ? MovingHandoffSeconds : GetUpPolicy3D::FirmSeconds) || !ensurePartner()) return;
        ++getUpFalls;
        if (!moving) ++getUpStood;
        // (O alvo do levantar em movimento: a rede da locomocao corre ate ele.)
        const TrackingGetUpAmpWorldView3D world = getUp->worldView(0);
        runTarget = moving && world.hasTarget ? std::optional<Vec3>(world.target) : std::nullopt;
        arrivedAt = -1.0f;
        partner->resumeFrom(body.links, getUp->driveTargets(0));
        partner->setManualTarget(runTarget,
            runTarget ? std::min(std::max(world.commandSpeed, 2.5f), PartnerRunSpeed) : 0.0f);
        partnerActive = true;
        partnerTime = 0.0f;
        firmSeconds = 0.0f;
    }
    // Uma decisao no mundo da rede da locomocao.
    void stepPartner() {
        partner->step();
        partnerTime += partner->policyStepSeconds();
        if (getUpKind == 2) {
            if (!avoidPushed && partnerTime >= 1.0f) {
                std::mt19937 rng(static_cast<std::uint32_t>(seed * 7919u + restarts));
                std::uniform_real_distribution<float> u(0.0f, 1.0f);
                const float angle = 6.2831853f * u(rng);
                const float newtons = avoidLow + (avoidHigh - avoidLow) * u(rng);
                partner->push({ std::cos(angle) * newtons, std::sin(angle) * newtons, 0.0f }, 0.1f + 0.1f * u(rng));
                avoidPushed = true;
                avoidRecoveries = partner->recoveries();
            }
            const bool took = avoidPushed && partner->recoveries() > avoidRecoveries;
            if (took && !partner->gettingUp() && avoidBackAt < 0.0f) {
                avoidBackAt = partnerTime;
                ++getUpFalls;
                if (partner->lastRecoveryAvoided()) ++getUpStood;
            }
            // (Recomeca 2 s depois da volta, 3 s depois do empurrao sem a troca - ele aguentou sozinho - ou em 12 s.)
            if ((avoidBackAt >= 0.0f && partnerTime >= avoidBackAt + 2.0f) || (!took && partnerTime >= 4.0f)
                || partnerTime >= 12.0f)
                startAvoid();
            return;
        }
        // (O levantar em movimento: chegou ao alvo - conta - e recomeca 1 s depois, ou em 6 s.)
        if (runTarget) {
            const RagdollState3D* body = partner->view().state;
            if (arrivedAt < 0.0f && body != nullptr && !body->links.empty()) {
                const Vec3 p = body->links.front().position;
                if (std::hypot(runTarget->x - p.x, runTarget->y - p.y) < 0.8f) {
                    arrivedAt = partnerTime;
                    ++getUpStood;
                }
            }
            if ((arrivedAt >= 0.0f && partnerTime >= arrivedAt + 1.0f) || partnerTime >= 6.0f) {
                partner->setManualTarget(std::nullopt, 0.0f);
                runTarget.reset();
                partnerActive = false;
                refreshAssist();
                getUp->reset(seed + ++restarts);
                firmSeconds = 0.0f;
                wasDown = false;
            }
            return;
        }
        if (partnerTime >= PartnerSeconds && !partner->gettingUp()) {
            partnerActive = false;
            refreshAssist();
                getUp->reset(seed + ++restarts);
            firmSeconds = 0.0f;
            wasDown = false;
        }
    }
    float avoidLow = 1800.0f, avoidHigh = 2600.0f;
    // O musculo da cabeca da locomocao do treino (o json do run.json; vazio: o do rastreador).
    std::string muscleFile;
    // (04/11, ele: "eu devo ver o exercicio rodando com a forca auxiliar real que ele tem" - relida do run.json a cada
    // episodio: o treinador a afrouxa durante o treino.)
    void refreshAssist() {
        if (!getUp) return;
        std::ifstream file(getUpDirectory / "run.json");
        const nlohmann::json run = nlohmann::json::parse(file, nullptr, false);
        if (!run.is_discarded() && run.is_object() && run.contains("v2") && run["v2"].is_object()) {
            getUp->setLocomotionAssist(run["v2"].value("ajuda_locomocao", 0.0f));
            getUp->setRunAssist(run["v2"].value("ajuda_corrida", -1.0f));
            getUp->setManeuverAssist(run["v2"].value("ajuda_manobra", -1.0f));
            getUp->setDrillAssist(drillAssistOf(run["v2"]));
        }
    }
    void startGetUp() {
        TrackingGetUpAmpSettings3D s;
        s.assetsDirectory = assets.string();
        s.environments = 1;
        s.threads = 1;
        s.seed = seed;
        s.evaluation = true;
        s.evaluationKind = getUpKind;
        s.heading = locomotionHeadingOf(getUpDrill, seed);
        s.evaluationBand = locomotionBandOf(getUpDrill);
        s.evaluationManeuver = maneuverOf(getUpDrill);
        if (s.evaluationManeuver > 0)
            s.evaluationBand = getUpDrill.ends_with("_corrida") || getUpDrill.starts_with("lev_estrela_corrida_") ? 2 : 1;
        // (As forcas e as velocidades do treino de agora: o run.json do levantar v2, gravado a cada abertura.)
        std::ifstream file(getUpDirectory / "run.json");
        const nlohmann::json run = nlohmann::json::parse(file, nullptr, false);
        if (!run.is_discarded() && run.is_object() && run.contains("v2") && run["v2"].is_object()) {
            const nlohmann::json& v2 = run["v2"];
            const auto pair = [&](const char* key, float& low, float& high) {
                if (v2.contains(key) && v2[key].is_array() && v2[key].size() == 2) {
                    low = v2[key][0].get<float>();
                    high = v2[key][1].get<float>();
                }
            };
            pair("evitar_forca", s.avoidMinimumNewtons, s.avoidMaximumNewtons);
            avoidLow = s.avoidMinimumNewtons;
            avoidHigh = s.avoidMaximumNewtons;
            pair("empurrao_forca", s.pushMinimumNewtons, s.pushMaximumNewtons);
            pair("velocidade", s.moveMinimumSpeed, s.moveMaximumSpeed);
            pair("empurrao_continuo_forca", s.pushHoldMinimumNewtons, s.pushHoldMaximumNewtons);
            // (A rl_final: o tempo dos rumos e a queda encerrando, como no treino.)
            s.locomotionSeconds = v2.value("locomocao_segundos", 0.0f);
            s.locomotionTermination = v2.value("queda_encerra", false);
            // (O clipe nos motores, como no treino: 01/11 18h45.)
            s.clipBlend = v2.value("mistura_clipe", 1.0f);
            // (A queda programada, como no treino: 01/11 22h50.)
            s.protectiveFall = v2.value("queda_programada", false);
            s.protectiveFallStepwise = true;
            // (O levantar guiado pelo clipe, como no treino: 01/11 23h10.)
            s.getUpGuide = v2.value("levantar_guiado", false);
            s.getUpSeconds = v2.value("levantar_segundos", 1.7f);
            // (O levantar pelo clipe, como no treino: 05/11 04h40.)
            s.getUpTracking = v2.value("levantar_clipe", false);
            // (O rolar pelo clipe, como no treino: 02/11 01h40.)
            s.rollBlend = v2.value("rolar_clipe", 1.0f);
            // (Os ambientes nas fatias do treino, so no exercicio deles.)
            if (getUpDrill == SceneryDrill || TrackingViewer3D::isAvoidSceneryDrill(getUpDrill) || TrackingViewer3D::sceneryPresetOf(getUpDrill) >= 0) {
                s.evaluationScenery = true;
                s.obstacleFraction = v2.value("obstaculo", 0.0f);
                s.weightFraction = v2.value("peso_em_cima", 0.0f);
                if (v2.contains("ambientes") && v2["ambientes"].is_array() && v2["ambientes"].size() == 6) {
                    float* fractions[6] { &s.corridorFraction, &s.pillarFraction, &s.slopeFraction, &s.roughFraction,
                        &s.bodiesUnderFraction, &s.bodyOnTopFraction };
                    for (std::size_t k = 0; k < 6; ++k) *fractions[k] = v2["ambientes"][k].get<float>();
                }
            }
            // (Os rumos e o giro com a regua fixa da avaliacao do treino: 2,5 m/s e, se o treino tem empurroes em
            // movimento, os do comeco do B21 - os padroes daqui; sem eles no treino, sem eles aqui.)
            // (03/11, o rastreio dos clipes: as duas cabecas, a nota e o fim do episodio como no treino.)
            s.separateSkills = v2.value("habilidades_separadas", false);
            s.locomotionTracking = v2.value("rastreio", false);
            s.trackingTermination = v2.value("rastreio_encerra", 0.0f);
            // (04/11: a forca auxiliar da locomocao de agora - o treinador grava a cada mudanca.)
            s.locomotionAssist = v2.value("ajuda_locomocao", 0.0f);
            s.balanceFeet = v2.value("pe_equilibrio", false);
            s.runAssist = v2.value("ajuda_corrida", -1.0f);
            s.maneuverAssist = v2.value("ajuda_manobra", -1.0f);
            s.idleAfterStand = v2.value("idle_pos_levantar", false);
            s.idleOnLocomotion = v2.value("parado_locomocao", false);
            s.idleResidualScale = v2.value("idle_alcance", 1.0f);
            // (04/11 09h10: a entrega ao idle depois de levantar, como no treino.)
            s.idleBlendSeconds = v2.value("idle_entrega", 0.0f);
            s.riseHandoffHead = v2.value("entrega_subindo", 0.0f);
            s.lateralHandPush = !v2.value("trote_lateral_sem_puxao", false);
            s.lateralRunLegReach = v2.value("corrida_lateral_pernas", 1.0f);
            // (04/11 12h40: os pedidos fixos do trote e da corrida, com o clipe na cadencia dele.)
            for (const auto& [key, speeds] : { std::pair { "velocidades_trote", &s.trotSpeeds }, std::pair { "velocidades_corrida", &s.runSpeeds } })
                if (v2.contains(key) && v2[key].is_array() && v2[key].size() == 8)
                    for (std::size_t k = 0; k < 8; ++k) (*speeds)[k] = v2[key][k].get<float>();
            s.backStartLean = -v2.value("largada_costas", 0.0f) * 3.14159265f / 180.0f;
            s.courseLimit = v2.value("curva_fora", 0.0f);
            s.curveLean = v2.value("curva_inclinar", false);
            s.accelLean = v2.value("troca_inclinar", 0.0f) * 3.14159265f / 180.0f;
            s.sprintSpeed = v2.value("sprint_velocidade", 0.0f);
            if (v2.contains("sprint_inclinar") && v2["sprint_inclinar"].is_array() && v2["sprint_inclinar"].size() == 2) {
                s.sprintLean = v2["sprint_inclinar"][0].get<float>() * 3.14159265f / 180.0f;
                s.sprintLeanLegs = v2["sprint_inclinar"][1].get<float>();
            }
            // (06/11: a largada progressiva da corrida, como no treino.)
            if (v2.contains("corrida_largada") && v2["corrida_largada"].is_array() && v2["corrida_largada"].size() == 3) {
                s.runLaunchScale = v2["corrida_largada"][0].get<float>();
                s.runLaunchEaseSeconds = v2["corrida_largada"][1].get<float>();
                s.runLaunchAcceleration = v2["corrida_largada"][2].get<float>();
            }
            if (v2.contains("corrida_inclinar") && v2["corrida_inclinar"].is_array() && v2["corrida_inclinar"].size() == 2) {
                s.runLean = v2["corrida_inclinar"][0].get<float>() * 3.14159265f / 180.0f;
                s.runLeanLegs = v2["corrida_inclinar"][1].get<float>();
            }
            if (v2.contains("corrida_costas_inclinar") && v2["corrida_costas_inclinar"].is_array()
                && v2["corrida_costas_inclinar"].size() == 2) {
                s.runBackLean = v2["corrida_costas_inclinar"][0].get<float>() * 3.14159265f / 180.0f;
                s.runBackLeanLegs = v2["corrida_costas_inclinar"][1].get<float>();
            }
            if (v2.contains("trote_frente_inclinar") && v2["trote_frente_inclinar"].is_array() && v2["trote_frente_inclinar"].size() == 2) {
                s.frontLean = v2["trote_frente_inclinar"][0].get<float>() * 3.14159265f / 180.0f;
                s.frontStartLean = v2["trote_frente_inclinar"][1].get<float>() * 3.14159265f / 180.0f;
            }
            s.recoveryNatural = v2.value("naturalidade", 0.0f);
            s.idleBlendReach = v2.value("idle_entrega_alcance", -1.0f);
            s.trackingReward = v2.value("recompensa_rastreio", s.trackingReward);
            s.phaseSync = v2.value("sincronizar_fase", false);
            s.armClipReach = v2.value("bracos_clipe", -1.0f);
            s.headClipReach = v2.value("cabeca_clipe", -1.0f);
            // (08/11: a estrela na fase de agora do treino e a duracao dela.)
            if (v2.contains("estrela_fases") && v2["estrela_fases"].is_array() && v2["estrela_fases"].size() == 6)
                for (std::size_t k = 0; k < 6; ++k) s.starPhases[k] = std::clamp(v2["estrela_fases"][k].get<int>(), 1, 4);
            s.starSeconds = v2.value("estrela_segundos", s.starSeconds);
            // (08/11 14h40: o comando na fase de agora do treino.)
            s.commandPhase = std::clamp(v2.value("comando_fase", 1), 1, 4);
            s.commandSeconds = v2.value("comando_segundos", s.commandSeconds);
            s.upperHand = v2.value("mao_tronco", false);
            s.bodyHand = v2.value("mao_corpo", false);
            // (04/11 15h, os passos: o layout levantar-v7 - as entradas do equilibrio, os canais do passo - e o gerador, a
            // autoridade sob ameaca e a chegada do levantar como no treino; o equilibrio parado no nivel dele.)
            s.balanceObservation = s.stepChannels = s.balanceSteps = v2.value("equilibrio", false);
            // (11/11 18h05: a variante do levantar pelo clipe na entrada, como no treino - o rolando com 1.)
            s.clipVariantObservation = v2.value("levantar_unreal_variante", false);
            // (12/11 06h40: as entradas do fogo, como no treino.)
            s.fireObservation = v2.value("levantar_fogo_entrada", false);
            s.idleThroughPushes = v2.value("empurrao_idle", false);
            s.idleThreatReach = v2.value("idle_alcance_ameaca", 0.0f);
            s.avoidOnIdle = v2.value("evitar_idle", false);
            s.arrivalBonus = v2.value("chegada_premio", 0.0f);
            s.flightPenalty = v2.value("voo_multa", 0.0f);
            s.balancePushLevel = v2.value("equilibrio_nivel", 0.5f);
            {
                const std::vector<float> levels = drillAssistOf(v2);
                for (std::size_t k = 0; k < s.drillAssist.size() && k < levels.size(); ++k) s.drillAssist[k] = levels[k];
            }
            s.locomotionResidual = v2.value("residuo", 0.0f) > 0.0f;
            s.trotMinimumSpeed = s.trotMaximumSpeed = 2.5f;
            // (03/11 18h27, a professora B21: os rumos como no treino dela - de pe, no pedido do que ela faz, sem rampa
            // nem fase - e o musculo da cabeca da locomocao; aqui a aluna sozinha.)
            if (!v2.value("professor", std::string()).empty()) {
                s.taughtHeadings = true;
                if (v2.contains("professor_aluna") && v2["professor_aluna"].is_array() && v2["professor_aluna"].size() == 8)
                    for (std::size_t k = 0; k < 8; ++k) s.teacherStudentSpeeds[k] = v2["professor_aluna"][k].get<float>();
            }
            muscleFile = v2.value("musculo", std::string());
            float low = 0.0f, high = 0.0f, holdLow = 0.0f, holdHigh = 0.0f;
            pair("empurrao_mov_forca", low, high);
            pair("empurrao_mov_continuo_forca", holdLow, holdHigh);
            if (high <= 0.0f && holdHigh <= 0.0f)
                s.movingPushMaximumNewtons = s.movingPushMinimumNewtons = s.movingPushHoldMinimumNewtons =
                    s.movingPushHoldMaximumNewtons = 0.0f;
            // (08/11, a fase de fogo: as pancadas e os empurroes no nivel de agora do treino - as faixas do fire_bands do
            // levantar_amp.py -, um por episodio na locomocao e, no equilibrio, a cada 1,5-3 s.)
            if (const float level = std::clamp(v2.value("fogo_nivel", -1.0f), -1.0f, 3.0f); level >= 0.0f) {
                s.pushMinimumNewtons = s.movingPushMinimumNewtons = 350.0f + 250.0f * level;
                s.pushMaximumNewtons = s.movingPushMaximumNewtons = std::min(2400.0f, 350.0f + 1050.0f * level);
                s.pushHoldMinimumNewtons = s.movingPushHoldMinimumNewtons = 120.0f + 80.0f * level;
                s.pushHoldMaximumNewtons = s.movingPushHoldMaximumNewtons = std::min(900.0f, 120.0f + 380.0f * level);
                s.pushHoldMinimumSeconds = s.movingPushHoldMinimumSeconds = 3.0f;
                s.pushHoldMaximumSeconds = s.movingPushHoldMaximumSeconds = 5.0f;
                s.pushHoldFraction = s.movingPushHoldFraction = 0.5f;
                s.balancePushLevel = 1.0f;
            }
        }
        // (18h05) A caixa "Empurroes": marcada, os empurroes do treino (as faixas por grupo, abaixo, depois de construir); desmarcada,
        // nenhum na locomocao.
        if (pushes) {
            s.pushHoldFraction = s.movingPushHoldFraction = 0.5f;
            s.pushHoldMinimumSeconds = s.movingPushHoldMinimumSeconds = 3.0f;
            s.pushHoldMaximumSeconds = s.movingPushHoldMaximumSeconds = 5.0f;
        } else {
            s.movingPushMinimumNewtons = s.movingPushMaximumNewtons = s.movingPushHoldMinimumNewtons = s.movingPushHoldMaximumNewtons = 0.0f;
        }
        s.sceneryPreset = TrackingViewer3D::sceneryPresetOf(getUpDrill);
        if (s.sceneryPreset >= 0) {
            s.evaluationScenery = true;
            if (s.sceneryPreset == 3 || s.sceneryPreset == 4) s.episodeSeconds = 12.0f;
        }
        // (05/11 02h, ele: "faca durar mais o tempo do exercicio para eu ver se de fato ele esta caindo": o levantar no
        // visualizador com 20 s - a queda, o levantar e o de pe no idle por muito tempo.)
        if (getUpKind == 0 && s.sceneryPreset < 0) s.episodeSeconds = 20.0f;
        getUp = std::make_unique<TrackingGetUpAmpTraining3D>(s);
        // (08/11: os passos pelos ciclos na meta, como no treino.)
        {
            std::ifstream cyclesFile(getUpDirectory / "run.json");
            const nlohmann::json cyclesRun = nlohmann::json::parse(cyclesFile, nullptr, false);
            if (!cyclesRun.is_discarded() && cyclesRun.is_object() && cyclesRun.contains("v2") && cyclesRun["v2"].is_object())
            {
                getUp->setGaitCycles(cyclesRun["v2"].value("passos_ciclos", false));
                // (08/11 16h20: a parada no fim dos retos do treino - aqui em todo reto, na janela dele -, pelas paradas da Unreal
                // quando o treino as usa.)
                if (cyclesRun["v2"].value("parada", 0.0) > 0.0) {
                    getUp->setFinalStop(1.0f, 4.5f, 5.5f, 5.0f, true);
                    getUp->setStopClips(cyclesRun["v2"].value("parada_clipes", false));
                }
                // (09/11 14h: a parada livre - a locomocao dirige ate parar de pe, sem referencia que freia - tambem no comando.)
                getUp->setFreeStop(cyclesRun["v2"].value("parada_livre", false));
                getUp->setFreeTroca(cyclesRun["v2"].value("troca_livre", false));
                getUp->setFreeStandGate(cyclesRun["v2"].value("parada_pose", 0.0f));
                getUp->setCommandChestGate(cyclesRun["v2"].value("peito_comando", 0.0f));
                getUp->setFreeIdle(cyclesRun["v2"].value("parado_livre", false));
                getUp->setPoseBoost(cyclesRun["v2"].value("pose_reforco", 0.0f));
                getUp->setFreePush(cyclesRun["v2"].value("empurrao_livre", false));
                getUp->setFreeIdleMatch(cyclesRun["v2"].value("parado_semelhanca", 0.0f));
                getUp->setFreeStepHeight(cyclesRun["v2"].value("passo_baixo", 0.0f));
                getUp->setIdleBentEnd(cyclesRun["v2"].value("parado_dobrado_encerra", false));
                getUp->setUpperImitation(cyclesRun["v2"].value("imitacao_tronco", false));
                getUp->setIdleWide(cyclesRun["v2"].value("parado_largo", false));
                // (11/11: sem freio - a ordem de parar vai direto ao idle alerta -, o parado unico e a nota nova dele, como no treino.)
                getUp->setStopToIdle(cyclesRun["v2"].value("parada_direto_idle", false));
                getUp->setIdleAfterStop(cyclesRun["v2"].value("idle_depois_freada", false));
                getUp->setTurnVariety(cyclesRun["v2"].value("giro_variado", false), cyclesRun["v2"].value("giro_coluna", 0.0f));
                getUp->setStillCost(cyclesRun["v2"].value("quieto_custos", 0.0f));
                getUp->setIdleStatic(cyclesRun["v2"].value("parado_estatico", false));
                // (11/11 10h45, ele: "a coluna ta torta no parado": a coluna sem o canal livre de velocidade, como no treino.)
                getUp->setSpineVelocityFeedforward(cyclesRun["v2"].value("coluna_sem_velocidade", false));
                getUp->setIdleUnified(cyclesRun["v2"].value("parado_unico", false));
                if (cyclesRun["v2"].contains("parado_recompensa") && cyclesRun["v2"]["parado_recompensa"].is_array()
                    && cyclesRun["v2"]["parado_recompensa"].size() == 3) {
                    const nlohmann::json& r = cyclesRun["v2"]["parado_recompensa"];
                    getUp->setIdleReward(r[0].get<float>(), r[1].get<float>(), r[2].get<float>());
                }
                if (pushes) {
                    // (As faixas do fire_bands do levantar_amp.py, no nivel de cada grupo do treino agora.)
                    const nlohmann::json& v = cyclesRun["v2"];
                    std::vector<float> bands;
                    for (const char* group : { "frente", "lado", "costas", "giro", "parado", "outros" }) {
                        float level = -0.5f;
                        if (v.contains("fogo_niveis") && v["fogo_niveis"].is_object()) level = v["fogo_niveis"].value(group, -0.5f);
                        level = std::clamp(level, -0.5f, 3.0f);
                        const float hitLow = 350.0f + 250.0f * level, holdLow = 120.0f + 80.0f * level;
                        const float hitHigh = level < 0.0f ? hitLow : std::min(2400.0f, 350.0f + 1050.0f * level);
                        const float holdHigh = level < 0.0f ? holdLow : std::min(900.0f, 120.0f + 380.0f * level);
                        bands.insert(bands.end(), { hitLow, hitHigh, holdLow, holdHigh });
                    }
                    getUp->setFireGroups(bands);
                    getUp->setPushVariety(v.value("fogo_pontos", false));
                    getUp->setFireShown(true);
                }
            }
        }
        // (06/11 17h40: a velocidade-alvo na acao da rede livre, como no treino - antes de dimensionar a acao.)
        {
            std::ifstream velocityFile(getUpDirectory / "run.json");
            const nlohmann::json velocityRun = nlohmann::json::parse(velocityFile, nullptr, false);
            if (!velocityRun.is_discarded() && velocityRun.is_object() && velocityRun.contains("v2") && velocityRun["v2"].is_object())
            {
                getUp->setVelocityActions(velocityRun["v2"].value("acao_velocidade", false));
                getUp->setRecoveryVelocity(velocityRun["v2"].value("levantar_velocidade", false));
                // (11/11 14h: o levantar pelo clipe - os clipes e as opcoes do treino, LEVANTAR_UNREAL do levantar_amp.py.)
                if (velocityRun["v2"].value("levantar_unreal", 0.0) > 0.0) {
                    const nlohmann::json& v = velocityRun["v2"];
                    const std::vector<std::string> clips { "levantar/levantar_costas", "levantar/levantar_brucos",
                        "levantar/levantar_correndo_costas", "levantar/levantar_correndo_brucos", "levantar/levantar_costas_rolando" };
                    getUp->addStyleClips(clips);
                    getUp->setClipGetUp(clips, 0.0f, 0.0f, v.value("levantar_unreal_depois", 1.5f), 0.0f,
                        v.value("levantar_unreal_largura", 2.0f), v.value("levantar_unreal_vel", 1.5f));
                    getUp->setClipGetUpVariants({ 0.0f, 0.0f, 0.0f, 0.0f, 1.0f });
                    getUp->setClipGetUpParts(v.value("levantar_unreal_partes", false));
                    // (12/11 04h30, ele: "os levantares correndo estao terminando muito cedo": sem o relogio da agilidade do treino, a
                    // rede - mais rapida que o clipe - ficava longe dele e o laboratorio encerrava aos 1,0-1,3 s.) Os pontos de entrega e
                    // a agilidade do treino (levantar_unreal_entrega, levantar_unreal_agilidade: parado, correndo, rampa).
                    if (v.contains("levantar_unreal_entrega") && v["levantar_unreal_entrega"].is_array()) {
                        std::vector<float> deliver;
                        for (const auto& x : v["levantar_unreal_entrega"]) deliver.push_back(x.get<float>());
                        getUp->setClipGetUpDeliver(deliver);
                    }
                    if (v.contains("levantar_unreal_agilidade") && v["levantar_unreal_agilidade"].is_array()
                        && v["levantar_unreal_agilidade"].size() >= 2) {
                        const float stand = v["levantar_unreal_agilidade"][0].get<float>(), run = v["levantar_unreal_agilidade"][1].get<float>();
                        getUp->setClipGetUpAgility({ stand, stand, run, run, 0.0f });
                    }
                    // (12/11 04h40, a fase de fogo do levantar: o levantar adaptativo e o fogo como no treino.)
                    // (05h30, ele: "se eu marcar a checkbox Empurroes eu devo ver os treinos que ocorrem com os empurroes/pancadas (de
                    // diferentes tipos que temos)... caso o checkbox do empurroes esteja desativo eu vejo so os cenarios limpos") Marcada:
                    // so os cenarios com fogo - pancada, continuo, os dois e os de cima, nas fatias do treino - nos niveis de agora do
                    // treino (a ultima linha do log com fogo_levantar; sem ela, os de partida); desmarcada: so os limpos.
                    if (v.value("levantar_adaptativo", false)) getUp->setClipGetUpAdaptive(true);
                    if (v.contains("levantar_fogo") && v["levantar_fogo"].is_array() && v["levantar_fogo"].size() == 4
                        && v.contains("levantar_fogo_prova") && v["levantar_fogo_prova"].is_object()) {
                        std::vector<float> shares;
                        for (const auto& x : v["levantar_fogo"]) shares.push_back(x.get<float>());
                        nlohmann::json current = v["levantar_fogo_prova"];
                        {
                            std::ifstream log(getUpDirectory / "log.jsonl", std::ios::binary | std::ios::ate);
                            const std::streamoff size = log ? static_cast<std::streamoff>(log.tellg()) : 0;
                            const std::streamoff from = std::max<std::streamoff>(0, size - 600000);
                            std::string tail(static_cast<std::size_t>(size - from), '\0');
                            if (log) {
                                log.seekg(from);
                                log.read(tail.data(), static_cast<std::streamsize>(tail.size()));
                            }
                            for (std::size_t end = tail.size(); end > 0;) {
                                const std::size_t start = tail.rfind('\n', end - 1);
                                const std::size_t begin = start == std::string::npos ? 0 : start + 1;
                                const std::string_view line(tail.data() + begin, end - begin);
                                if (line.find("\"fogo_levantar\"") != std::string_view::npos) {
                                    const nlohmann::json row = nlohmann::json::parse(line, nullptr, false);
                                    if (!row.is_discarded() && row.contains("fogo_levantar") && row["fogo_levantar"].is_object()) {
                                        current = row["fogo_levantar"];
                                        break;
                                    }
                                }
                                if (start == std::string::npos) break;
                                end = start;
                            }
                        }
                        std::vector<float> levels;
                        for (const char* key : { "lev_levantar_costas", "lev_levantar_brucos", "lev_levantar_correndo_costas",
                                 "lev_levantar_correndo_brucos" }) {
                            const nlohmann::json& level = current.value(key, nlohmann::json::array());
                            for (std::size_t k = 0; k < 4; ++k) levels.push_back(k < level.size() ? level[k].get<float>() : 0.0f);
                        }
                        levels.insert(levels.end(), 4, 0.0f);
                        const float above = v.value("levantar_fogo_de_cima", 0.0f);
                        if (shares[1] + shares[2] + shares[3] > 0.0f)
                            getUp->setGetUpFire(pushes ? std::vector<float> { 0.0f, shares[1], shares[2], shares[3], above }
                                                       : std::vector<float> { 1.0f, 0.0f, 0.0f, 0.0f, 0.0f },
                                levels, false);
                    }
                    getUp->reset(seed);
                }
                // (07/11: os comecos deitados do treino - os mesmos clipes de LYING_STAND e LYING_RUN do levantar_amp.py.)
                if (velocityRun["v2"].value("deitado", false)) {
                    const std::vector<std::string> stand { "referencias_unreal/ragdoll_getup_stand_b", "referencias_unreal/ragdoll_getup_stand_f" };
                    const std::vector<std::string> run { "referencias_unreal/ragdoll_getup_run_b", "referencias_unreal/ragdoll_getup_run_f",
                        "referencias_unreal/ragdoll_getup_run_l", "referencias_unreal/ragdoll_getup_run_r" };
                    std::vector<std::string> all = stand;
                    all.insert(all.end(), run.begin(), run.end());
                    getUp->addStyleClips(all);
                    getUp->setLyingStarts(stand, run);
                    getUp->setMovingHandoff(true);
                    // (07/11, ele: "as vezes nasce em pe": o construtor ja sorteou o primeiro comeco sem os deitados.)
                    getUp->reset(seed);
                }
            }
        }
        if (!muscleFile.empty()) {
            std::ifstream muscleStream(muscleFile);
            const nlohmann::json muscle = nlohmann::json::parse(muscleStream, nullptr, false);
            if (!muscle.is_discarded() && muscle.is_object()) {
                const auto values = [&](const char* key) {
                    std::vector<float> out;
                    if (muscle.contains(key) && muscle[key].is_array())
                        for (const auto& v : muscle[key]) out.push_back(v.get<float>());
                    return out;
                };
                const std::vector<float> stiffness = values("rigidez"), damping = values("amortecimento"),
                                         torque = values("teto");
                if (stiffness.size() == getUp->actionSize() && damping.size() == stiffness.size()
                    && torque.size() == stiffness.size())
                    getUp->setStudentActuator(stiffness, damping, torque, muscle.value("gravidade", false));
            }
        }
        // (As quedas da locomocao: as 32 fixas da avaliacao do treino, que ele grava cruas - a posicao, a orientacao e
        // as velocidades de cada link; sem o arquivo, as quedas sorteadas.)
        if (getUpKind == 7) {
            std::ifstream raw(getUpDirectory / "quedas_locomocao_aval.f32", std::ios::binary);
            std::vector<float> values;
            for (float value; raw.read(reinterpret_cast<char*>(&value), sizeof value);) values.push_back(value);
            const std::size_t links = getUp->profile().links.size();
            std::vector<std::vector<PhysicsBodyState3D>> states(values.size() / (links * 13));
            for (std::size_t i = 0; i < states.size(); ++i) {
                states[i].resize(links);
                for (std::size_t k = 0; k < links; ++k) {
                    const float* d = values.data() + (i * links + k) * 13;
                    states[i][k].position = { d[0], d[1], d[2] };
                    states[i][k].orientation = Quaternion { d[3], d[4], d[5], d[6] };
                    states[i][k].linearVelocity = { d[7], d[8], d[9] };
                    states[i][k].angularVelocity = { d[10], d[11], d[12] };
                }
            }
            getUp->setFallStarts(std::move(states));
            getUp->reset(seed);
        }
        getUpObservation.assign(getUp->observationSize(), 0.0f);
        getUpAction.assign(getUp->actionSize(), 0.0f);
        getUpFalls = getUpStood = 0;
        // (So nos exercicios em que o cinematico age: os rumos e manobras, o parado e o giro - as avaliacoes 3 a 6.)
        if (kinematic && getUpKind >= 3 && getUpKind <= 6) applyKinematic();
        // (07/11, ele: "falta essa visao cinematica para os exercicios de levantar": de costas, de brucos e correndo tambem.)
        // (11/11, ele: "o clipes do exercicio deveria mostrar o que o treino segue... foda-se se e ou nao Unreal": a vista
        // mostra a pose alvo do treino - em view() -, nao mais a sequencia de clipes da Unreal do juiz; a rede segue agindo.)
    }
    // (07/11, ele: "vamos fazer os clipes tipo nos faziamos antes, sem Motion Matching... ligar as trocas adequadamente" e
    // "so ver no laboratorio") A sequencia de clipes da Unreal de cada exercicio, por regra, sem fisica - ligada por padrao.
    // (07/11, ele: "agora o clipes do exercicio desativado por default".)
    bool sequence = false;
    mutable RagdollState3D referenceBody;
    // (08/11 18h05, ele: "um checkbox ao lado do Clipes do exercicio, chamado empurroes, para eu visualizar os treinos com
    // empurroes"): marcada, os empurroes do treino - o nivel de cada grupo, os pontos variados - em todo episodio.
    bool pushes = false;
    bool sequenceActive = false;
    void applySequence() {
        kinematicError.clear();
        sequenceActive = false;
        if (!getUp) return;
        try {
            std::vector<std::string> names;
            const std::filesystem::path folder = assets / "animations" / "clips" / "referencias_unreal";
            for (const auto& entry : std::filesystem::directory_iterator(folder)) {
                const std::string file = entry.path().filename().string();
                constexpr std::string_view Suffix = ".matteranim.json";
                if (!file.ends_with(Suffix)) continue;
                const std::string id = file.substr(0, file.size() - Suffix.size());
                const bool used = id.starts_with("trote_run_start_") || id.starts_with("trote_run_loop_") || id.starts_with("trote_run_stop_")
                    || id.starts_with("trote_run_turn_") || id.starts_with("trote_run_arc_") || id.starts_with("corrida_")
                    || id.starts_with("trote_run_reface_start_") || id.starts_with("stand_turn_")
                    || id == "stand_idle_loop";
                if (used) names.push_back("referencias_unreal/" + id);
            }
            std::sort(names.begin(), names.end());
            getUp->addStyleClips(names);
            getUp->setClipSequence(true);
            getUp->setStarDrill(starModeOf(getUpDrill));
            getUp->reset(seed);
            sequenceActive = true;
        } catch (const std::exception& error) {
            kinematicError = error.what();
        }
    }
    // (07/11, ele: "temporariamente, coloca os movimentos dos exercicios de forma cinematica, so para eu ver o Motion Matching
    // operando") Os bancos do juiz (tools/rl/juiz_mm_bancos.json, o conjunto denso do Game Animation Sample) e a previsao do
    // treino no lote do laboratorio; o corpo vai ao quadro escolhido a cada decisao, sem fisica.
    // (07/11, ele: "deixa como default ativo, pois nao consigo ativar pois trava"; depois: "esse Motion Matching esta muito
    // problematico, entao ele fica arquivado por hora" - desligado e sem a chave na tela; o codigo fica.)
    bool kinematic = false;
    std::string kinematicError;
    void applyKinematic() {
        kinematicError.clear();
        if (!getUp) return;
        try {
            std::ifstream specFile(assets.parent_path() / "tools" / "rl" / "juiz_mm_bancos.json");
            const nlohmann::json spec = nlohmann::json::parse(specFile);
            std::vector<std::string> names, schemas, all;
            std::vector<std::vector<std::string>> clips;
            std::vector<std::array<float, 3>> biases;
            std::vector<int> categories;
            std::size_t main = 0;
            for (const nlohmann::json& g : spec.at("grupos")) {
                if (g.at("nome").get<std::string>() == spec.value("principal", std::string())) main = names.size();
                names.push_back(g.at("nome").get<std::string>());
                schemas.push_back(g.at("esquema").get<std::string>());
                clips.push_back(g.at("clipes").get<std::vector<std::string>>());
                all.insert(all.end(), clips.back().begin(), clips.back().end());
                const auto v = g.at("vieses");
                biases.push_back({ v.at(0).get<float>(), v.at(1).get<float>(), v.at(2).get<float>() });
                categories.push_back(g.at("categoria").get<int>());
            }
            getUp->addStyleClips(all);
            getUp->buildMotionMatching(names, schemas, clips, biases, categories, main);
            getUp->setMotionMovement(8.8f, 22.1f, 5.0f, 0.0f, -1.0f);
            getUp->setKinematicMotion(true);
            getUp->reset(seed);
        } catch (const std::exception& error) {
            kinematicError = error.what();
        }
    }
    void loadGetUpPolicy() {
        std::error_code error;
        const auto write = std::filesystem::last_write_time(getUpDirectory / "exports/policy_latest.mrlpolicy", error);
        if (error || (getUpPolicy && write == getUpPolicyWrite)) return;
        try {
            auto policy = std::make_unique<PolicyRuntime3D>(getUpDirectory / "exports/policy_latest.mrlpolicy");
            // (A rede do contrato v2, sem as entradas do chao do v3 no fim, le so o comeco da observacao.)
            const std::size_t size = policy->observationSize();
            // (As do ambiente, do levantar-v5, vem por ultimo: a rede do v4 le tudo menos elas.)
            constexpr std::size_t environment = TrackingGetUpAmpTraining3D::EnvironmentSize;
            constexpr std::size_t phase = TrackingGetUpAmpTraining3D::PhaseSize;
            const std::size_t full = getUp->observationSize() - environment - TrackingGetUpAmpTraining3D::SkillModeSize;
            if ((size == getUp->observationSize() || size == full + environment || size == full || size + phase == full
                    || size + phase + TrackingGetUpAmpTraining3D::GroundObservationSize == full)
                && policy->actionSize() == getUp->actionSize()) {
                getUpPolicy = std::move(policy);
                getUpPolicyWrite = write;
            }
        } catch (const std::exception&) {
        }
    }
    std::uint32_t steps = 0;
    std::uint32_t chest = 0;
    float pushSeconds = 0.0f;
    Vec3 pushForce;
    // A seta da tela (como no classico): o empurrao de agora, o manual ou o do
    // exercicio, e depois do fim ela some em 0,8 s.
    Vec3 shownForce;
    int shownLink = -1;
    float shownAfter = 1.0e9f;
};

TrainingHistory3D TrackingViewer3D::readHistory(const std::filesystem::path& runDirectory, std::size_t maximumPoints,
    double sinceSeconds) {
    TrainingHistory3D history;
    std::ifstream log(runDirectory / "log.jsonl");
    for (std::string line; std::getline(log, line);) {
        const nlohmann::json row = nlohmann::json::parse(line, nullptr, false);
        if (row.is_discarded() || !row.is_object()) continue;
        // O levantar (levantar_amp, a rl_final - 02/11, ele: "nao aparece para mim no laboratorio"): as linhas do
        // treino (as de "aval" sao os testes, lidos a parte) e os tres exercicios dele.
        if (row.contains("atualizacao") && !row.contains("rodada")) {
            if (row.contains("aval")) continue;
            const auto number = [&](const char* key, double fallback) {
                const auto it = row.find(key);
                return it != row.end() && it->is_number() ? it->get<double>() : fallback;
            };
            TrainingIterationPoint3D point;
            point.iteration = static_cast<std::uint64_t>(number("atualizacao", 0.0));
            point.time = number("hora", 0.0);
            point.steps = number("amostras", 0.0);
            point.sps = number("am_s", 0.0);
            point.fallFraction = 1.0 - number("queda_levantou", 1.0);
            point.episodeSeconds = number("queda_tempo", 0.0);
            history.iterations.push_back(point);
            for (const auto& [drillName, stood, seconds] : { std::tuple { "levantar", "queda_levantou", "queda_tempo" },
                     std::tuple { "levantar_movimento", "mov_levantou", "mov_tempo" },
                     std::tuple { "evitar_queda", "evitar_evitou", "evitar_recompos_s" },
                     std::tuple { "levantar_ambientes", "queda_sujo_levantou", "queda_sujo_tempo" },
                     std::tuple { "evitar_queda_ambientes", "evitar_evitou_ambientes", "evitar_recompos_s" } }) {
                if (!row.contains(stood) || !row[stood].is_number()) continue;
                TrainingDrillPoint3D drill;
                drill.iteration = point.iteration;
                drill.time = point.time;
                drill.drill = drillName;
                drill.episodes = 1.0;
                drill.stood = row[stood].get<double>();
                drill.falls = 1.0 - drill.stood;
                drill.seconds = number(seconds, 0.0);
                history.drills.push_back(drill);
            }
            continue;
        }
        TrainingIterationPoint3D point;
        point.iteration = row.value("rodada", 0ull);
        point.time = row.value("hora", 0.0);
        point.steps = row.value("decisoes", 0.0);
        point.reward = row.value("nota_passo", 0.0);
        point.episodeSeconds = row.value("ep_segundos", 0.0);
        point.fallFraction = row.value("quedas", 0.0);
        point.sps = row.value("dec_s", 0.0);
        point.velocityError = row.value("erro_vel_cm", point.velocityError);
        point.bodySpeed = row.value("vel_corpo", point.bodySpeed);
        point.requestedSpeed = row.value("vel_pedida", point.requestedSpeed);
        history.iterations.push_back(point);
        // Por direcao ("rumos" do treinador): um exercicio "rumoN" por rumo com episodios na rodada.
        if (const auto rumos = row.find("rumos"); rumos != row.end() && rumos->is_object())
            for (const auto& [key, data] : rumos->items()) {
                if (!data.is_object()) continue;
                TrainingDrillPoint3D drill;
                drill.iteration = point.iteration;
                drill.time = point.time;
                drill.drill = !key.empty() && std::isdigit(static_cast<unsigned char>(key[0])) ? "rumo" + key : key;
                drill.episodes = data.value("n", 0.0);
                drill.falls = 1.0 - data.value("sem_cair", 0.0);
                drill.seconds = data.value("ep_segundos", 0.0);
                drill.reward = data.value("nota", drill.reward);
                drill.velocityError = data.value("erro_vel_cm", drill.velocityError);
                drill.bodySpeed = data.value("vel_corpo", drill.bodySpeed);
                drill.requestedSpeed = data.value("vel_pedida", drill.requestedSpeed);
                history.drills.push_back(drill);
            }
        // Com a tarefa ("exercicios" do treinador): um ponto por exercicio com episodios na rodada.
        if (const auto drills = row.find("exercicios"); drills != row.end() && drills->is_object())
            for (const auto& [key, data] : drills->items()) {
                if (!data.is_object()) continue;
                TrainingDrillPoint3D drill;
                drill.iteration = point.iteration;
                drill.time = point.time;
                drill.drill = key;
                drill.episodes = data.value("n", 0.0);
                drill.falls = data.value("quedas", 0.0);
                drill.seconds = data.value("ep_segundos", 0.0);
                drill.velocityError = data.value("erro_vel_cm", drill.velocityError);
                drill.bodySpeed = data.value("vel_corpo", drill.bodySpeed);
                drill.requestedSpeed = data.value("vel_pedida", drill.requestedSpeed);
                history.drills.push_back(drill);
            }
    }
    if (sinceSeconds > 0.0 && !history.iterations.empty()) {
        const double from = history.iterations.back().time - sinceSeconds;
        std::erase_if(history.iterations, [from](const TrainingIterationPoint3D& p) { return p.time < from; });
        std::erase_if(history.drills, [from](const TrainingDrillPoint3D& p) { return p.time < from; });
    }
    if (maximumPoints > 0 && history.iterations.size() > maximumPoints) {
        std::vector<TrainingIterationPoint3D> sampled;
        const double stride = static_cast<double>(history.iterations.size() - 1) / static_cast<double>(maximumPoints - 1);
        for (std::size_t i = 0; i < maximumPoints; ++i)
            sampled.push_back(history.iterations[static_cast<std::size_t>(std::lround(i * stride))]);
        history.iterations.swap(sampled);
    }
    // Os portoes de rumos (a ordem dos nomes e a do portao: 45 graus para a esquerda).
    static constexpr std::array<const char*, 8> Keys { "frente", "diag_esq", "esquerda", "tras_esq", "costas",
        "tras_dir", "direita", "diag_dir" };
    std::vector<nlohmann::json> gates;
    std::ifstream gate(runDirectory / "portao.jsonl");
    for (std::string line; std::getline(gate, line);) {
        nlohmann::json row = nlohmann::json::parse(line, nullptr, false);
        if (row.is_discarded() || !row.is_object()) continue;
        const std::string mode = row.value("modo", std::string());
        if (const std::string exercise = row.value("exercicio", std::string()); !exercise.empty()) {
            // O portao de um exercicio da tarefa: a queda de verdade e o erro da velocidade.
            TrainingEvaluationPoint3D point;
            point.iteration = row.value("rodada", 0ull);
            point.time = row.value("hora", 0.0);
            point.drill = exercise;
            point.solver = "rede";
            point.falls = row.value("quedas_de_verdade", row.value("quedas", 1.0));
            point.velocityError = row.value("erro_vel_cm", -1.0);
            history.evaluations.push_back(point);
        } else if (mode == "parado" || mode == "giro" || mode == "giro_rapido" || mode == "estrela" || mode == "estrela_fixo"
            || mode == "estrela_girando") {
            // Os exercicios parados: o portao inteiro e do exercicio (chegou ao fim ou nao).
            TrainingEvaluationPoint3D point;
            point.iteration = row.value("rodada", 0ull);
            point.time = row.value("hora", 0.0);
            point.drill = mode;
            point.solver = "rede";
            point.falls = row.value("fins_antes_do_tempo", 1.0);
            history.evaluations.push_back(point);
        } else if (mode == "rumos" && row.contains("por_rumo")) {
            gates.push_back(std::move(row));
        }
    }
    std::sort(history.evaluations.begin(), history.evaluations.end(),
        [](const auto& a, const auto& b) { return a.iteration < b.iteration; });
    if (gates.empty()) return history;
    // A velocidade com mais portoes (empate: a do mais recente), para a curva comparar o mesmo teste.
    double speed = 0.0;
    std::size_t most = 0;
    for (auto it = gates.rbegin(); it != gates.rend(); ++it) {
        const double v = it->value("velocidade", 0.0);
        const auto same = static_cast<std::size_t>(std::count_if(gates.begin(), gates.end(),
            [v](const nlohmann::json& g) { return std::abs(g.value("velocidade", 0.0) - v) <= 1e-3; }));
        if (same > most) most = same, speed = v;
    }
    for (const nlohmann::json& row : gates) {
        if (std::abs(row.value("velocidade", 0.0) - speed) > 1e-3) continue;
        for (std::size_t i = 0; i < Keys.size(); ++i) {
            const nlohmann::json& one = row["por_rumo"].value(Keys[i], nlohmann::json::object());
            TrainingEvaluationPoint3D point;
            point.iteration = row.value("rodada", 0ull);
            point.time = row.value("hora", 0.0);
            point.drill = "rumo" + std::to_string(i);
            point.solver = "rede";
            // Como o sem cair do treino: chegou ao fim (sem cair nem ficar para tras do lider) ou nao.
            point.falls = one.value("quedas", 0.0);
            point.velocityError = one.value("erro_vel_cm", -1.0);
            history.evaluations.push_back(point);
        }
    }
    std::sort(history.evaluations.begin(), history.evaluations.end(),
        [](const auto& a, const auto& b) { return a.iteration < b.iteration; });
    return history;
}

float TrackingViewer3D::headingOf(const std::string& drill) {
    if (drill.size() != 5 || drill.rfind("rumo", 0) != 0 || drill[4] < '0' || drill[4] > '7') return 0.0f;
    return static_cast<float>(drill[4] - '0') * (3.14159265358979f / 4.0f);
}

std::filesystem::path TrackingViewer3D::locomotionTaskOf(const std::filesystem::path& trackingTask,
    const std::filesystem::path& assetsDirectory) {
    std::ifstream file(trackingTask);
    if (!file) return {};
    const nlohmann::json task = nlohmann::json::parse(file, nullptr, false);
    if (task.is_discarded() || !task.is_object()) return {};
    const std::filesystem::path given = task.value("tarefaLocomocao", std::string());
    if (given.empty()) return {};
    const std::filesystem::path path = given.is_absolute() ? given : assetsDirectory / given;
    return LocomotionViewer3D::isLocomotionTask(path) ? path : std::filesystem::path {};
}

float TrackingViewer3D::speedScaleOf(const std::filesystem::path& trackingTask) {
    std::ifstream file(trackingTask);
    if (!file) return 1.0f;
    const nlohmann::json task = nlohmann::json::parse(file, nullptr, false);
    return task.is_object() ? task.value("escalaVelocidade", 1.0f) : 1.0f;
}

float TrackingViewer3D::filterBetaOf(const std::filesystem::path& trackingTask) {
    std::ifstream file(trackingTask);
    if (!file) return 0.2f;
    const nlohmann::json task = nlohmann::json::parse(file, nullptr, false);
    return task.is_object() ? task.value("filtroBeta", 0.2f) : 0.2f;
}

TrackingViewer3D::Conditions TrackingViewer3D::conditionsOf(const std::filesystem::path& trackingTask) {
    Conditions out;
    std::ifstream file(trackingTask);
    if (!file) return out;
    const nlohmann::json task = nlohmann::json::parse(file, nullptr, false);
    if (!task.is_object()) return out;
    out.followSlack = task.value("seguirCorpo", out.followSlack);
    out.realFallOnly = task.value("soQuedaReal", out.realFallOnly);
    out.labLeader = task.value("liderLaboratorio", out.labLeader);
    out.stanceTwistTorque = task.value("torcaoApoio", out.stanceTwistTorque);
    out.leaderSideFromForward = task.value("ladoDeFrente", out.leaderSideFromForward);
    out.leaderResponsePercent = task.value("respostaLider", out.leaderResponsePercent);
    out.terminationDistance = task.value("distanciaFim", out.terminationDistance);
    out.fallHeadHeight = task.value("cabecaQueda", out.fallHeadHeight);
    out.captureStepSeconds = task.value("passoCaptura", out.captureStepSeconds);
    out.leaderSideAmplitude = task.value("amplitudeLado", out.leaderSideAmplitude);
    return out;
}

TrackingViewer3D::TrackingViewer3D(const std::filesystem::path& assetsDirectory, std::uint64_t seed,
    const std::string& drill, const std::filesystem::path& locomotionTask, float speedScale, float filterBeta,
    Conditions conditions, const std::filesystem::path& getUpDirectory)
    : m_impl(std::make_unique<Impl>()) {
    Impl& impl = *m_impl;
    impl.heading = headingOf(drill);
    impl.drill = drill == "parado" ? TrackingDrill3D::Idle
        : drill == "giro"          ? TrackingDrill3D::Turn
        : drill == "giro_rapido"   ? TrackingDrill3D::FastTurn
        : drill == "estrela"       ? TrackingDrill3D::Star
        : drill == "estrela_fixo"  ? TrackingDrill3D::StarFixed
        : drill == "estrela_girando" ? TrackingDrill3D::StarTurning
                                   : TrackingDrill3D::Move;
    // O modo de avaliacao do exercicio: o recomeco de cada episodio volta a ele.
    const TrackingEvaluation3D mode = impl.drill == TrackingDrill3D::Idle ? TrackingEvaluation3D::Idle
        : impl.drill == TrackingDrill3D::Turn                            ? TrackingEvaluation3D::Turn
        : impl.drill == TrackingDrill3D::FastTurn                        ? TrackingEvaluation3D::FastTurn
        : impl.drill == TrackingDrill3D::Star                            ? TrackingEvaluation3D::Star
        : impl.drill == TrackingDrill3D::StarFixed                       ? TrackingEvaluation3D::StarFixed
        : impl.drill == TrackingDrill3D::StarTurning                     ? TrackingEvaluation3D::StarTurning
                                                                         : TrackingEvaluation3D::Forward;
    TrackingTrainingSettings3D settings;
    settings.assetsDirectory = assetsDirectory.string();
    settings.environments = 1;
    settings.threads = 1;
    settings.seed = seed;
    settings.evaluation = true;
    // (As condicoes do treino: o lider do laboratorio, indo ou nao atras do corpo, e o fim.)
    settings.realFallOnly = conditions.realFallOnly;
    settings.labLeader = conditions.labLeader;
    settings.stanceTwistTorque = conditions.stanceTwistTorque;
    settings.leaderSideFromForward = conditions.leaderSideFromForward;
    settings.leaderResponsePercent = conditions.leaderResponsePercent;
    settings.terminationDistance = conditions.terminationDistance;
    settings.fallHeadHeight = conditions.fallHeadHeight;
    settings.captureStepSeconds = conditions.captureStepSeconds;
    settings.leaderSideAmplitude = conditions.leaderSideAmplitude;
    settings.leaderFollowSlack = conditions.followSlack;
    settings.evaluationSpeed = DefaultSpeed;
    settings.evaluationMode = mode;
    settings.actionFilterBeta = filterBeta;
    if (!locomotionTask.empty()) {
        // O exercicio da tarefa (vazio ou de outra lista: o primeiro com peso), na escala do treino.
        impl.task = true;
        settings.taskPath = locomotionTask.string();
        settings.speedScale = speedScale;
        const auto drills = LocomotionViewer3D::drillsOf(locomotionTask);
        const bool known = std::any_of(drills.begin(), drills.end(), [&](const auto& d) { return d.id == drill && !d.getUp; });
        for (const auto& d : drills)
            if (known ? d.id == drill : d.weight > 0.0f && !d.getUp) {
                settings.evaluationDrill = d.id;
                break;
            }
    }
    // O visualizador nao recomeca pelo tempo (so pela queda ou o afastamento).
    settings.episodeSeconds = 1.0e6f;
    impl.batch = std::make_unique<TrackingTraining3D>(settings);
    impl.observation.assign(impl.batch->observationSize(), 0.0f);
    impl.action.assign(impl.batch->actionSize(), 0.0f);
    const RagdollProfile3D& p = impl.batch->profile();
    for (std::size_t i = 0; i < p.links.size(); ++i)
        if (p.links[i].id == "UpperChest") impl.chest = static_cast<std::uint32_t>(i);
    impl.apply();
    if (isGetUpDrill(drill)) {
        impl.getUpDirectory = getUpRun(getUpDirectory);
        impl.assets = assetsDirectory;
        impl.seed = seed;
        impl.getUpDrill = drill;
        // (11/11 14h: com o levantar pelo clipe no treino, os quatro exercicios dele - as avaliacoes 11 a 14.)
        const bool clipGetUp = clipGetUpRun(impl.getUpDirectory);
        impl.getUpKind = drill == MovingGetUpDrill ? 1u : drill == LyingBackDrill ? (clipGetUp ? 11u : 9u)
            : drill == LyingFrontDrill ? (clipGetUp ? 12u : 10u) : drill == RunGetUpBackDrill ? 13u : drill == RunGetUpFrontDrill ? 14u
            : drill == RollGetUpDrill ? 15u
            : drill == AvoidFallDrill || isAvoidSceneryDrill(drill) ? 2u
            : drill == BalanceDrill                                                                     ? 3u
            : drill == EquilibriumDrill                                                                 ? 8u
            : drill == LocomotionFallDrill                                                                      ? 7u
            : drill == LocomotionDrills[0]                                                                      ? 5u
            : drill == LocomotionDrills[1]                                                                      ? 6u
            : isLocomotionDrill(drill)                                                                          ? 4u
                                                                                                                : 0u;
        impl.startGetUp();
        impl.loadGetUpPolicy();
        // (02/11 22h45, ele: "faz ele nem trocar o mundo da locomocao, faz ele ficar do comeco ao fim no RL final sem
        // troca de RL": a rl_final dirige o episodio inteiro, no mundo dela - o cenario dos ambientes fica ate o fim -,
        // sem a parceira da locomocao.)
        if (impl.getUpKind == 2) impl.startAvoid();
    }
}

int TrackingViewer3D::sceneryPresetOf(const std::string& drill) {
    for (std::size_t i = 0; i < ControlledGetUpDrills.size(); ++i)
        if (drill == ControlledGetUpDrills[i]) return static_cast<int>(i);
    for (std::size_t i = 0; i < ControlledAvoidDrills.size(); ++i)
        if (drill == ControlledAvoidDrills[i]) return static_cast<int>(i);
    return -1;
}

bool TrackingViewer3D::isAvoidSceneryDrill(const std::string& drill) {
    return drill == AvoidSceneryDrill || std::find(ControlledAvoidDrills.begin(), ControlledAvoidDrills.end(), drill) != ControlledAvoidDrills.end();
}

bool TrackingViewer3D::clipGetUpRun(const std::filesystem::path& runDirectory) {
    std::ifstream file(runDirectory / "run.json");
    const nlohmann::json run = nlohmann::json::parse(file, nullptr, false);
    return !run.is_discarded() && run.is_object() && run.contains("v2") && run["v2"].is_object()
        && run["v2"].value("levantar_unreal", 0.0) > 0.0;
}

bool TrackingViewer3D::isGetUpDrill(const std::string& drill) {
    return drill == GetUpDrill || drill == MovingGetUpDrill || drill == LyingBackDrill || drill == LyingFrontDrill
        || drill == RunGetUpBackDrill || drill == RunGetUpFrontDrill || drill == RollGetUpDrill || drill == AvoidFallDrill || drill == BalanceDrill || drill == EquilibriumDrill
        || drill == LocomotionFallDrill || drill == SceneryDrill || drill == AvoidSceneryDrill || sceneryPresetOf(drill) >= 0 || isLocomotionDrill(drill);
}

bool TrackingViewer3D::isGetUpTask(const std::filesystem::path& taskPath) {
    std::ifstream file(taskPath);
    if (!file) return false;
    const nlohmann::json task = nlohmann::json::parse(file, nullptr, false);
    return !task.is_discarded() && task.is_object() && task.value("schema", std::string()) == "matter-getup-amp-1";
}

// (04/11: os rumos da corrida, "lev_corrida_0".."lev_corrida_7", como os do trote.)
bool runHeadingDrill(const std::string& drill) {
    return drill.size() == 13 && drill.rfind("lev_corrida_", 0) == 0 && drill.back() >= '0' && drill.back() <= '7';
}

int TrackingViewer3D::maneuverOf(const std::string& drill) {
    static constexpr const char* Names[7] { "lev_troca_suave", "lev_troca_media", "lev_troca_forte", "lev_troca_livre",
        "lev_curva_suave", "lev_curva_s", "lev_curva_fechada" };
    if (drill == "lev_comando") return 12;
    // (08/11: a estrela - 8 peito fixo, 9 livre, 10 girando; a faixa pelo nome.)
    for (std::size_t k = 0; k < StarDrills.size(); ++k)
        if (drill == StarDrills[k]) return 8 + static_cast<int>(k % 3);
    for (int k = 0; k < 7; ++k)
        if (drill == Names[k]) return k + 1;
    // (04/11 11h50: as curvas na corrida.)
    for (int k = 4; k < 7; ++k)
        if (drill == std::string(Names[k]) + "_corrida") return k + 1;
    return -1;
}

int TrackingViewer3D::starModeOf(const std::string& drill) {
    if (!drill.starts_with("comp_estrela_")) return 0;
    return drill.ends_with("_livre") ? 2 : drill.ends_with("_fixo") ? 1 : 0;
}

int TrackingViewer3D::starBandOf(const std::string& drill) {
    return starModeOf(drill) == 0 ? -1 : drill.find("_corrida_") != std::string::npos ? 2 : 1;
}

bool TrackingViewer3D::isLocomotionDrill(const std::string& drill) {
    if (maneuverOf(drill) > 0) return true;
    if (starModeOf(drill) > 0) return true;
    if (drill.size() == 11 && drill.rfind("lev_trote_", 0) == 0 && drill.back() >= '0' && drill.back() <= '7') return true;
    if (runHeadingDrill(drill)) return true;
    return std::find(LocomotionDrills.begin(), LocomotionDrills.end(), drill) != LocomotionDrills.end();
}

int TrackingViewer3D::locomotionHeadingOf(const std::string& drill, std::uint64_t seed) {
    if (drill.size() == 11 && drill.rfind("lev_trote_", 0) == 0 && drill.back() >= '0' && drill.back() <= '7') return drill.back() - '0';
    if (runHeadingDrill(drill)) return drill.back() - '0';
    if (maneuverOf(drill) > 0) return 0;
    if (starModeOf(drill) > 0) return 0;
    // (05/11, ele: "no sprint faltou a velocidade pedida x atendida em tempo real, igual faz com os outros": o sprint e para a frente.)
    if (drill == "lev_sprint") return 0;
    const bool right = seed % 2 == 1;
    if (drill == "lev_frente") return 0;
    if (drill == "lev_diagonal_frente") return right ? 7 : 1;
    if (drill == "lev_esquerda") return 2;
    if (drill == "lev_diagonal_costas") return right ? 5 : 3;
    if (drill == "lev_tras") return 4;
    if (drill == "lev_direita") return 6;
    return -1;
}

bool TrackingViewer3D::getUpVersion2(const std::filesystem::path& runDirectory) {
    const auto directory = getUpRun(runDirectory);
    std::ifstream file(directory / "run.json");
    const auto run = nlohmann::json::parse(file, nullptr, false);
    return (!run.is_discarded() && run.is_object() && run.contains("v2") && run["v2"].is_object())
        || directory.filename() == "levantar_v2" || directory.filename() == "rl_final";
}

int TrackingViewer3D::locomotionBandOf(const std::string& drill) {
    if (runHeadingDrill(drill)) return 2;
    if (starModeOf(drill) > 0) return starBandOf(drill);
    for (int band = 0; band < 4; ++band)
        if (drill == LocomotionDrills[8 + static_cast<std::size_t>(band)]) return band;
    return -1;
}

bool TrackingViewer3D::getUpLocomotion(const std::filesystem::path& runDirectory) {
    std::ifstream file(getUpRun(runDirectory) / "run.json");
    const nlohmann::json run = nlohmann::json::parse(file, nullptr, false);
    return !run.is_discarded() && run.is_object() && run.contains("v2") && run["v2"].is_object()
        && run["v2"].value("locomocao", 0.0) > 0.0;
}

std::filesystem::path TrackingViewer3D::getUpExport(const std::filesystem::path& runDirectory) { return getUpRun(runDirectory) / "exports/policy_latest.mrlpolicy"; }

std::filesystem::path TrackingViewer3D::getUpLog(const std::filesystem::path& runDirectory) { return getUpRun(runDirectory) / "log.jsonl"; }

bool TrackingViewer3D::getUpReady(const std::filesystem::path& runDirectory) {
    std::error_code error;
    return std::filesystem::exists(getUpExport(runDirectory), error);
}

bool TrackingViewer3D::getUpTraining(const std::filesystem::path& runDirectory) {
    const auto directory = getUpRun(runDirectory);
    std::ifstream file(directory / "status.json");
    const auto status = nlohmann::json::parse(file, nullptr, false);
    return !status.is_discarded() && status.is_object()
        && trainerProcessAlive3D(status.value("pid", 0), directory.filename().string());
}

TrackingViewer3D::~TrackingViewer3D() = default;

bool TrackingViewer3D::isTrackingTask(const std::filesystem::path& taskPath) {
    std::ifstream file(taskPath);
    if (!file) return false;
    const nlohmann::json task = nlohmann::json::parse(file, nullptr, false);
    return !task.is_discarded() && task.is_object() && task.value("schema", std::string()) == "matter-tracking-1";
}

void TrackingViewer3D::setPolicy(const std::filesystem::path& file, std::string& label) {
    Impl& impl = *m_impl;
    if (impl.getUp) {
        // (O levantar usa a rede propria, nao a do treino mostrado.)
        impl.loadGetUpPolicy();
        label = impl.getUpPolicy
            ? impl.getUpDirectory.filename().string() + " r" + std::to_string(impl.getUpPolicy->info().iteration)
            : "sem rede";
        return;
    }
    try {
        auto policy = std::make_unique<PolicyRuntime3D>(file);
        if (policy->observationSize() != impl.batch->observationSize() || policy->actionSize() != impl.batch->actionSize()
            || policy->info().observationLayout != "rastreador-v2") {
            impl.policy.reset();
            label = "politica de outro contrato";
            return;
        }
        impl.policy = std::move(policy);
        label = "rodada " + std::to_string(impl.policy->info().iteration);
    } catch (const std::exception& error) {
        impl.policy.reset();
        label = error.what();
    }
}

void TrackingViewer3D::clearPolicy() { m_impl->policy.reset(); }
void TrackingViewer3D::setKinematicMotionMatching(bool on) {
    Impl& impl = *m_impl;
    if (on == impl.kinematic) return;
    impl.kinematic = on;
    if (impl.getUp) {
        if (on) impl.applyKinematic();
        else impl.startGetUp();
    }
}
bool TrackingViewer3D::kinematicMotionMatching() const { return m_impl->kinematic; }
void TrackingViewer3D::setClipSequence(bool on) {
    Impl& impl = *m_impl;
    if (on == impl.sequence) return;
    impl.sequence = on;
    if (impl.getUp) impl.startGetUp();
}
bool TrackingViewer3D::clipSequence() const { return m_impl->sequence; }
void TrackingViewer3D::setPushes(bool on) {
    Impl& impl = *m_impl;
    if (on == impl.pushes) return;
    impl.pushes = on;
    if (impl.getUp) impl.startGetUp();
}
bool TrackingViewer3D::pushes() const { return m_impl->pushes; }
std::string TrackingViewer3D::kinematicClip() const { return m_impl->getUp ? m_impl->getUp->kinematicClip(0) : std::string(); }
std::size_t TrackingViewer3D::kinematicJumps() const { return m_impl->getUp ? m_impl->getUp->kinematicJumps(0) : 0; }
const std::string& TrackingViewer3D::kinematicError() const { return m_impl->kinematicError; }
bool TrackingViewer3D::hasPolicy() const {
    return m_impl->getUp ? m_impl->getUpPolicy != nullptr : m_impl->policy != nullptr;
}

void TrackingViewer3D::step() {
    Impl& impl = *m_impl;
    if (impl.getUp) {
        if (impl.partnerActive) {
            impl.stepPartner();
            ++impl.steps;
            return;
        }
        // A rede nova do treino a cada ~2 s; sem rede, os alvos zerados (o corpo largado).
        if (impl.steps % 60 == 0) {
            impl.loadGetUpPolicy();
        }
        impl.getUp->observations(impl.getUpObservation);
        if (impl.getUpPolicy)
            impl.getUpPolicy->evaluate(std::span<const float>(impl.getUpObservation).first(impl.getUpPolicy->observationSize()),
                impl.getUpAction);
        else std::fill(impl.getUpAction.begin(), impl.getUpAction.end(), 0.0f);
        impl.getUp->step(impl.getUpAction, impl.rewards, impl.terminated, impl.truncated);
        for (const TrackingGetUpAmpEpisode3D& episode : impl.getUp->takeEpisodes()) {
            ++impl.getUpFalls;
            // (O levantar pelo clipe: a troca no fim do clipe e de pe com a locomocao depois.)
            const bool good = impl.getUpKind >= 11 ? episode.handoffSeconds >= 0.0f && !episode.partnerFell
                : impl.getUpKind == 1 ? episode.reachedSeconds >= 0.0f
                : impl.getUpKind >= 2             ? !episode.fellAfterStanding
                                                  : episode.firstStandSeconds >= 0.0f;
            if (good) ++impl.getUpStood;
            // (O episodio novo do mundo do levantar.)
            impl.wasDown = false;
            impl.firmSeconds = 0.0f;
        }
        impl.checkHandoff();
        ++impl.steps;
        return;
    }
    impl.batch->observations(impl.observation);
    // Sem rede, o lider puro (a correcao zero): o corpo cai em ~1 s.
    if (impl.policy) impl.policy->evaluate(impl.observation, impl.action);
    else std::fill(impl.action.begin(), impl.action.end(), 0.0f);
    impl.batch->step(impl.action, impl.rewards, impl.terminated, impl.truncated);
    (void)impl.batch->takeEpisodes();
    impl.pushSeconds = std::max(0.0f, impl.pushSeconds - policyStepSeconds());
    Vec3 force;
    std::uint32_t link = 0;
    if (impl.pushSeconds > 0.0f) {
        impl.shownForce = impl.pushForce;
        impl.shownLink = static_cast<int>(impl.chest);
        impl.shownAfter = 0.0f;
    } else if (impl.batch->taskPush(0, force, link)) {
        impl.shownForce = force;
        impl.shownLink = static_cast<int>(link);
        impl.shownAfter = 0.0f;
    } else {
        impl.shownAfter += policyStepSeconds();
    }
    ++impl.steps;
}

void TrackingViewer3D::reset(std::uint64_t seed) {
    Impl& impl = *m_impl;
    if (impl.getUp) {
        impl.seed = seed;
        // (Com o cinematico, o recomeco nao refaz o lote - os bancos levam segundos para montar.)
        if ((impl.kinematic || impl.sequenceActive) && impl.kinematicError.empty()) impl.getUp->reset(seed);
        else impl.startGetUp();
        impl.steps = 0;
        impl.partnerActive = false;
        impl.firmSeconds = 0.0f;
        impl.wasDown = false;
        if (impl.getUpKind == 2) impl.startAvoid();
        return;
    }
    impl.batch->reset(seed);
    impl.apply();
    impl.steps = 0;
    impl.shownLink = -1;
    impl.shownAfter = 1.0e9f;
}

void TrackingViewer3D::setManualSpeed(std::optional<float> metersPerSecond) {
    Impl& impl = *m_impl;
    impl.speed = metersPerSecond.value_or(DefaultSpeed);
    impl.apply();
}

void TrackingViewer3D::push(Vec3 force, float seconds) {
    Impl& impl = *m_impl;
    if (impl.getUp && impl.partnerActive) impl.partner->push(force, seconds);
    if (impl.getUp) return;
    impl.batch->push(0, impl.chest, force, seconds);
    impl.pushForce = force;
    impl.pushSeconds = seconds;
}

BodyWorldView3D TrackingViewer3D::view() const {
    const Impl& impl = *m_impl;
    if (impl.getUp && impl.partnerActive) return impl.partner->view();
    if (impl.getUp) {
        const RagdollState3D& body = impl.getUp->body(0);
        BodyWorldView3D view;
        view.state = &body;
        // (11/11) Clipes do exercicio: a pose alvo que o treino segue, no lugar do corpo da rede.
        if (impl.sequence) {
            impl.referenceBody = body;
            if (impl.getUp->referencePose(0, impl.referenceBody.links)) view.state = &impl.referenceBody;
        }
        view.steps = impl.steps;
        const Vec3 velocity = rlBodyCenterOfMassVelocity3D(impl.getUp->profile(), body.links);
        view.velocity = { velocity.x, velocity.y, 0.0f };
        // O alvo do levantar em movimento (parado quando ele chega) e o empurrao de agora (o forte do evitar).
        const TrackingGetUpAmpWorldView3D world = impl.getUp->worldView(0);
        if (world.hasTarget) {
            view.hasTarget = true;
            view.target = world.target;
            view.targetStop = world.commandSpeed <= 0.0f;
        }
        // (08/11: a estrela - as 9 marcas, as linhas do centro as pontas, a perna de agora, o ponto dela e o seguinte, o texto.)
        if (world.star && world.starPoints.size() == 9) {
            const auto point = [&](int k) { return world.starPoints[static_cast<std::size_t>(std::clamp(k + 1, 0, 8))]; };
            view.starPoints = world.starPoints;
            for (int k = 0; k < 8; ++k) view.starLines.push_back({ world.starPoints[0], point(k) });
            view.hasTarget = true;
            view.target = point(world.starGoal);
            view.targetStop = world.starArrive < 1.6f;
            view.hasStarLeg = true;
            view.starLeg = { point(world.starFrom), point(world.starGoal) };
            view.hasNextTarget = true;
            view.nextTarget = point(world.starNext);
            static constexpr const char* Chest[3] { "peito fixo", "peito livre", "peito girando" };
            char text[160];
            std::snprintf(text, sizeof text, "%s · fase %d/4 · perna %d · próxima troca %.0f° · chegada %.1f m/s",
                Chest[std::clamp(world.starChest - 8, 0, 2)], world.starPhase, world.starLegs + 1,
                std::abs(world.starTurnDegrees), world.starArrive);
            view.starCaption = text;
        }
        if (!world.commandText.empty()) view.starCaption = world.commandText;
        // (04/11 11h50: nas curvas, o caminho pedido a frente - os pontos azuis no chao; o fim dele em verde.)
        if (!world.path.empty()) {
            view.starPoints = world.path;
            view.hasTarget = true;
            view.target = world.path.back();
        }
        // (O pedido de deslocamento - rumo ao alvo no levantar em movimento, o rumo pedido na locomocao - e, na
        // locomocao, o rumo pedido do corpo: a camera e o olho seguem ele, como nos exercicios do B21.)
        view.command.moveVelocity = world.moveVelocity;
        if (world.hasFace) view.command.faceHeading = world.faceYaw;
        if (world.clipGetUp && world.fireKind > 0) {
            // (12/11 05h30) O fogo do levantar, desenhado a parte (ReinforcementLearningScreen) - sem a seta antiga do empurrao.
            GetUpFireView3D& fire = view.fire;
            fire.active = true;
            fire.kind = world.fireKind;
            fire.timing = world.fireTiming;
            fire.direction = world.fireDirection;
            fire.shoveLink = world.fireShoveLink;
            fire.abovePitch = world.fireAbovePitch;
            fire.fraction = world.fireFraction;
            fire.size = world.fireSize;
            fire.shoveSize = world.fireShoveSize;
            fire.hold = world.fireHold;
            fire.shoveForce = world.fireShoveForce;
            fire.aim = world.fireAim;
            fire.shoveAt = world.fireShoveAt;
            fire.shoveSeconds = world.fireShoveSeconds;
            fire.shoveDone = world.fireShoveDone;
            fire.time = world.time;
            fire.budget = world.fireBudget;
            fire.upAt = world.upAt;
            fire.deliveredAt = world.deliveredAt;
            fire.downAt = world.downAt;
            fire.weight = world.weight;
            fire.levelHold = world.fireLevelHold;
            fire.levelShove = world.fireLevelShove;
        } else if (world.pushing && impl.chest < body.links.size()) {
            view.pushLink = static_cast<int>(impl.chest);
            view.pushForce = world.pushForce;
            view.pushFade = 1.0f;
        }
        impl.getUp->scenerySegments(0, view.scenery);
        impl.getUp->sceneryBodies(0, view.otherBodies);
        return view;
    }
    const RagdollState3D& body = impl.batch->body(0);
    const TrackingLeaderState3D& leader = impl.batch->leader(0);
    const RagdollProfile3D& profile = impl.batch->profile();
    BodyWorldView3D view;
    view.state = &body;
    view.steps = impl.steps;
    view.command.moveVelocity = { leader.requestedVelocityWorld.x, leader.requestedVelocityWorld.y, 0.0f };
    view.command.faceHeading = leader.headingYawRadians;
    const Vec3 velocity = rlBodyCenterOfMassVelocity3D(profile, body.links);
    view.velocity = { velocity.x, velocity.y, 0.0f };
    if (impl.chest < body.links.size()) {
        const Quaternion r =
            (body.links[impl.chest].orientation * profile.links[impl.chest].modelOrientation.conjugate()).normalized();
        const Vec3 forward = r.rotate({ 1.0f, 0.0f, 0.0f });
        view.chestHeading = std::atan2(forward.y, forward.x);
    }
    Vec3 target;
    bool waiting = false;
    if (impl.batch->starView(0, view.starPoints, target, waiting)) {
        view.hasTarget = true;
        view.target = target;
        view.targetStop = waiting;
    }
    // O empurrao de agora (ou o ultimo, sumindo em 0,8 s).
    if (impl.shownLink >= 0 && static_cast<std::size_t>(impl.shownLink) < body.links.size() && impl.shownAfter < 0.8f) {
        view.pushLink = impl.shownLink;
        view.pushForce = impl.shownForce;
        view.pushFade = 1.0f - impl.shownAfter / 0.8f;
    }
    return view;
}

const RagdollProfile3D& TrackingViewer3D::profile() const {
    return m_impl->getUp ? m_impl->getUp->profile() : m_impl->batch->profile();
}

std::pair<std::uint32_t, std::uint32_t> TrackingViewer3D::getUpTally() const {
    return { m_impl->getUpStood, m_impl->getUpFalls };
}

bool TrackingViewer3D::drivingLocomotion() const {
    return m_impl->getUp && m_impl->partnerActive && !m_impl->partner->gettingUp();
}

bool TrackingViewer3D::locomotionHeadDriving() const {
    return m_impl->getUp && !m_impl->getUpObservation.empty() && m_impl->getUpObservation.back() > 0.5f;
}

const std::string& TrackingViewer3D::partnerText() const { return m_impl->partnerLabel; }

} // namespace MatterEngine
