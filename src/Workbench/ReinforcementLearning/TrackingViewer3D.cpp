#include "Workbench/ReinforcementLearning/TrackingViewer3D.hpp"

#include "Engine/RL/Body/RLBody3D.hpp"
#include "Engine/RL/Policy/PolicyRuntime3D.hpp"
#include "Engine/Tracking/TrackingTraining3D.hpp"
#include "Workbench/ReinforcementLearning/LocomotionViewer3D.hpp"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <fstream>
#include <vector>

namespace MatterEngine {
namespace {

// A velocidade de quando a tela nao pede outra (a do prototipo, G1).
constexpr float DefaultSpeed = 3.0f;

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

TrackingViewer3D::TrackingViewer3D(const std::filesystem::path& assetsDirectory, std::uint64_t seed,
    const std::string& drill, const std::filesystem::path& locomotionTask, float speedScale, float filterBeta)
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
    settings.realFallOnly = true;
    // (As condicoes do treino de 29/10: o lider do laboratorio indo atras do corpo.)
    settings.labLeader = true;
    settings.leaderFollowSlack = 0.15f;
    settings.evaluationSpeed = DefaultSpeed;
    settings.evaluationMode = mode;
    settings.actionFilterBeta = filterBeta;
    if (!locomotionTask.empty()) {
        // O exercicio da tarefa (vazio ou de outra lista: o primeiro com peso), na escala do treino.
        impl.task = true;
        settings.taskPath = locomotionTask.string();
        settings.speedScale = speedScale;
        const auto drills = LocomotionViewer3D::drillsOf(locomotionTask);
        const bool known = std::any_of(drills.begin(), drills.end(), [&](const auto& d) { return d.id == drill; });
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
bool TrackingViewer3D::hasPolicy() const { return m_impl->policy != nullptr; }

void TrackingViewer3D::step() {
    Impl& impl = *m_impl;
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
    impl.batch->push(0, impl.chest, force, seconds);
    impl.pushForce = force;
    impl.pushSeconds = seconds;
}

BodyWorldView3D TrackingViewer3D::view() const {
    const Impl& impl = *m_impl;
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

const RagdollProfile3D& TrackingViewer3D::profile() const { return m_impl->batch->profile(); }

} // namespace MatterEngine
