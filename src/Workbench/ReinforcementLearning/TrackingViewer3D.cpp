#include "Workbench/ReinforcementLearning/TrackingViewer3D.hpp"

#include "Engine/RL/Body/RLBody3D.hpp"
#include "Engine/RL/Policy/PolicyRuntime3D.hpp"
#include "Engine/Tracking/TrackingGetUpAmpTraining3D.hpp"
#include "Engine/Tracking/TrackingTraining3D.hpp"
#include "Workbench/ReinforcementLearning/LocomotionViewer3D.hpp"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <array>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <vector>

namespace MatterEngine {
namespace {

// A velocidade de quando a tela nao pede outra (a do prototipo, G1).
constexpr float DefaultSpeed = 3.0f;

// O treino do levantar livre (31/10: uma rede para qualquer queda, tools/rl/levantar_amp.py).
std::filesystem::path getUpRun() {
    const char* home = std::getenv("HOME");
    const auto runs = std::filesystem::path(home != nullptr ? home : ".") / ".local/share/matter-engine/rl/runs";
    // (31/10 17h: o levantar v2 - com o levantar em movimento e o evitar a queda -, quando existe.)
    return std::filesystem::exists(runs / "levantar_v2") ? runs / "levantar_v2" : runs / "levantar_livre";
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
    std::unique_ptr<TrackingGetUpAmpTraining3D> getUp;
    std::unique_ptr<PolicyRuntime3D> getUpPolicy;
    std::filesystem::file_time_type getUpPolicyWrite {};
    std::uint32_t getUpFalls = 0, getUpStood = 0;
    std::filesystem::path assets;
    std::uint64_t seed = 1;
    std::vector<float> getUpObservation, getUpAction;
    // Qual avaliacao do levantar: 0 as quedas, 1 o levantar em movimento, 2 o evitar a queda, 3 o equilibrio parado,
    // 4 um rumo (o do exercicio), 5 o parado, 6 o giro parado suave.
    std::uint32_t getUpKind = 0;
    std::string getUpDrill;
    void startGetUp() {
        TrackingGetUpAmpSettings3D s;
        s.assetsDirectory = assets.string();
        s.environments = 1;
        s.threads = 1;
        s.seed = seed;
        s.evaluation = true;
        s.evaluationKind = getUpKind;
        s.heading = locomotionHeadingOf(getUpDrill, seed);
        // (As forcas e as velocidades do treino de agora: o run.json do levantar v2, gravado a cada abertura.)
        std::ifstream file(getUpRun() / "run.json");
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
            pair("empurrao_forca", s.pushMinimumNewtons, s.pushMaximumNewtons);
            pair("velocidade", s.moveMinimumSpeed, s.moveMaximumSpeed);
            pair("empurrao_continuo_forca", s.pushHoldMinimumNewtons, s.pushHoldMaximumNewtons);
            // (Os rumos e o giro com a regua fixa da avaliacao do treino: 2,5 m/s e os empurroes do comeco do B21.)
            s.trotMinimumSpeed = s.trotMaximumSpeed = 2.5f;
        }
        getUp = std::make_unique<TrackingGetUpAmpTraining3D>(s);
        getUpObservation.assign(getUp->observationSize(), 0.0f);
        getUpAction.assign(getUp->actionSize(), 0.0f);
        getUpFalls = getUpStood = 0;
    }
    void loadGetUpPolicy() {
        std::error_code error;
        const auto write = std::filesystem::last_write_time(getUpExport(), error);
        if (error || (getUpPolicy && write == getUpPolicyWrite)) return;
        try {
            auto policy = std::make_unique<PolicyRuntime3D>(getUpExport());
            if (policy->observationSize() == getUp->observationSize() && policy->actionSize() == getUp->actionSize()) {
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
    Conditions conditions)
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
        impl.assets = assetsDirectory;
        impl.seed = seed;
        impl.getUpDrill = drill;
        impl.getUpKind = drill == MovingGetUpDrill ? 1u : drill == AvoidFallDrill ? 2u : drill == BalanceDrill ? 3u
            : drill == LocomotionDrills[0]                                                                      ? 5u
            : drill == LocomotionDrills[1]                                                                      ? 6u
            : isLocomotionDrill(drill)                                                                          ? 4u
                                                                                                                : 0u;
        impl.startGetUp();
        impl.loadGetUpPolicy();
    }
}

bool TrackingViewer3D::isGetUpDrill(const std::string& drill) {
    return drill == GetUpDrill || drill == MovingGetUpDrill || drill == AvoidFallDrill || drill == BalanceDrill
        || isLocomotionDrill(drill);
}

bool TrackingViewer3D::isLocomotionDrill(const std::string& drill) {
    return std::find(LocomotionDrills.begin(), LocomotionDrills.end(), drill) != LocomotionDrills.end();
}

int TrackingViewer3D::locomotionHeadingOf(const std::string& drill, std::uint64_t seed) {
    const bool right = seed % 2 == 1;
    if (drill == "lev_frente") return 0;
    if (drill == "lev_diagonal_frente") return right ? 7 : 1;
    if (drill == "lev_esquerda") return 2;
    if (drill == "lev_diagonal_costas") return right ? 5 : 3;
    if (drill == "lev_tras") return 4;
    if (drill == "lev_direita") return 6;
    return -1;
}

bool TrackingViewer3D::getUpVersion2() { return getUpRun().filename() == "levantar_v2"; }

std::filesystem::path TrackingViewer3D::getUpExport() { return getUpRun() / "exports/policy_latest.mrlpolicy"; }

std::filesystem::path TrackingViewer3D::getUpLog() { return getUpRun() / "log.jsonl"; }

bool TrackingViewer3D::getUpReady() {
    std::error_code error;
    return std::filesystem::exists(getUpExport(), error);
}

bool TrackingViewer3D::getUpTraining() {
    // (O treino grava o log a cada ~2 min.)
    std::error_code error;
    const auto written = std::filesystem::last_write_time(getUpLog(), error);
    return !error && std::filesystem::file_time_type::clock::now() - written < std::chrono::minutes(10);
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
            ? std::string(getUpVersion2() ? "levantar v2 r" : "levantar livre r") + std::to_string(impl.getUpPolicy->info().iteration)
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
bool TrackingViewer3D::hasPolicy() const {
    return m_impl->getUp ? m_impl->getUpPolicy != nullptr : m_impl->policy != nullptr;
}

void TrackingViewer3D::step() {
    Impl& impl = *m_impl;
    if (impl.getUp) {
        // A rede nova do treino a cada ~2 s; sem rede, os alvos zerados (o corpo largado).
        if (impl.steps % 60 == 0) impl.loadGetUpPolicy();
        impl.getUp->observations(impl.getUpObservation);
        if (impl.getUpPolicy) impl.getUpPolicy->evaluate(impl.getUpObservation, impl.getUpAction);
        else std::fill(impl.getUpAction.begin(), impl.getUpAction.end(), 0.0f);
        impl.getUp->step(impl.getUpAction, impl.rewards, impl.terminated, impl.truncated);
        for (const TrackingGetUpAmpEpisode3D& episode : impl.getUp->takeEpisodes()) {
            ++impl.getUpFalls;
            const bool good = impl.getUpKind == 1 ? episode.reachedSeconds >= 0.0f
                : impl.getUpKind >= 2             ? !episode.fellAfterStanding
                                                  : episode.firstStandSeconds >= 0.0f;
            if (good) ++impl.getUpStood;
        }
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
        impl.startGetUp();
        impl.steps = 0;
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
    if (impl.getUp) return;
    impl.batch->push(0, impl.chest, force, seconds);
    impl.pushForce = force;
    impl.pushSeconds = seconds;
}

BodyWorldView3D TrackingViewer3D::view() const {
    const Impl& impl = *m_impl;
    if (impl.getUp) {
        const RagdollState3D& body = impl.getUp->body(0);
        BodyWorldView3D view;
        view.state = &body;
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
        // (O pedido de deslocamento - rumo ao alvo no levantar em movimento, o rumo pedido na locomocao - e, na
        // locomocao, o rumo pedido do corpo: a camera e o olho seguem ele, como nos exercicios do B21.)
        view.command.moveVelocity = world.moveVelocity;
        if (world.hasFace) view.command.faceHeading = world.faceYaw;
        if (world.pushing && impl.chest < body.links.size()) {
            view.pushLink = static_cast<int>(impl.chest);
            view.pushForce = world.pushForce;
            view.pushFade = 1.0f;
        }
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

} // namespace MatterEngine
