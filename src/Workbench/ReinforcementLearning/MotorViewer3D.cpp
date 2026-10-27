#include "Workbench/ReinforcementLearning/MotorViewer3D.hpp"
#include "Engine/RL/Body/RLBody3D.hpp"
#include "Workbench/ReinforcementLearning/TrackingViewer3D.hpp"
#include <cmath>
#include <algorithm>
#include <fstream>
#include <nlohmann/json.hpp>

namespace MatterEngine {
bool MotorViewer3D::isMotorTask(const std::filesystem::path &path) {
    std::ifstream file(path);
    if (!file)
        return false;
    auto j = nlohmann::json::parse(file, nullptr, false);
    return j.is_object() && j.value("schema", std::string()) == "matter-motor-2";
}
TrainingHistory3D MotorViewer3D::readHistory(const std::filesystem::path &directory, std::size_t maximum,
                                             double since) {
    // O mesmo formato de linhas por exercicio, sem reutilizar a simulacao do
    // rastreador.
    auto history = TrackingViewer3D::readHistory(directory, maximum, since);
    std::ifstream log(directory / "log.jsonl");
    std::size_t cursor = 0;
    for (std::string line; std::getline(log, line);) {
        auto j = nlohmann::json::parse(line, nullptr, false);
        if (!j.is_object() || !j.contains("rumos"))
            continue;
        const auto iteration = j.value("rodada", 0ull);
        while (cursor < history.drills.size() && history.drills[cursor].iteration < iteration)
            ++cursor;
        for (auto at = cursor; at < history.drills.size() && history.drills[at].iteration == iteration; ++at) {
            auto &p = history.drills[at];
            if (j["rumos"].contains(p.drill)) {
                const auto &d = j["rumos"][p.drill];
                p.chestErrorDegrees = d.value("erro_peito_graus", p.chestErrorDegrees);
                p.drift = d.value("deriva", p.drift);
                if (d.value("imitacao_versao", 1) == 3 && d.contains("imitacao"))
                    p.imitation = d["imitacao"].value("similaridade", p.imitation);
                p.posture = d.value("postura", p.posture);
            }
        }
    }
    std::ifstream file(directory / "avaliacoes.jsonl");
    for (std::string line; std::getline(file, line);) {
        auto j = nlohmann::json::parse(line, nullptr, false);
        if (!j.is_object() || !j.contains("metrics"))
            continue;
        for (const auto &[id, m] : j["metrics"].items()) {
            TrainingEvaluationPoint3D p;
            p.iteration = j.value("rodada", 0ull);
            p.time = j.value("hora", 0.0);
            p.drill = id;
            p.solver = "jogo";
            p.falls = m.value("falls", 0.0);
            p.velocityError = m.value("velocity_error_median", 0.0);
            history.evaluations.push_back(p);
        }
    }
    return history;
}
MotorViewer3D::MotorViewer3D(const std::filesystem::path &assets, const std::filesystem::path &task,
                             const std::string &drill, std::uint64_t seed) {
    m_random.seed(seed);
    m_task = task;
    MotorSettings3D s;
    s.assetsDirectory = assets;
    s.taskPath = task;
    s.environments = 1;
    s.threads = 1;
    const auto &ids = MotorTraining3D::drillIds();
    const auto at = std::find(ids.begin(), ids.end(), drill);
    if (at == ids.end())
        throw std::invalid_argument("exercicio motor desconhecido: " + drill);
    s.drill = static_cast<int>(at - ids.begin());
    s.seed = seed;
    m_batch = std::make_unique<MotorTraining3D>(s);
    readTrotLimits();
    m_batch->reset(seed);
    m_obs.resize(m_batch->observationSize());
    m_action.resize(m_batch->actionSize());
}
void MotorViewer3D::readTrotLimits() {
    // A previa pede o trote no mesmo teto do curriculo que o treino usa agora.
    std::ifstream file(m_task.parent_path() / "curriculo.json");
    if (!file)
        return;
    const auto j = nlohmann::json::parse(file, nullptr, false);
    if (!j.is_object() || !j.contains("limits") || !j["limits"].is_array() || j["limits"].size() != 8)
        return;
    std::vector<float> limits;
    for (const auto &v : j["limits"])
        limits.push_back(v.is_number() ? v.get<float>() : 0.f);
    try {
        m_batch->setTrotLimits(limits);
    } catch (const std::invalid_argument &) {
    }
}
void MotorViewer3D::setPolicy(const std::filesystem::path &path, std::string &label) {
    try {
        m_batch->setTask(m_task);
        readTrotLimits();
        auto policy = std::make_unique<PolicyRuntime3D>(path);
        if (policy->info().observationLayout != "motor-v4" || policy->info().bodyHash != m_batch->identity() ||
            policy->info().observationNames != m_batch->observationNames() ||
            policy->actionSize() != m_batch->actionSize())
            throw std::runtime_error("politica de outro corpo/contrato motor");
        label = "rodada " + std::to_string(policy->info().iteration);
        m_policy = std::move(policy);
    } catch (const std::exception &e) {
        m_policy.reset();
        label = e.what();
    }
}
void MotorViewer3D::step() {
    m_batch->observations(m_obs);
    // A media da politica, como no jogo: o ruido de exploracao do treino aparecia na tela como tremor.
    if (m_policy)
        m_policy->evaluate(m_obs, m_action);
    else
        std::fill(m_action.begin(), m_action.end(), 0.f);
    float r;
    std::uint8_t t, u;
    m_batch->step(m_action, {&r, 1}, {&t, 1}, {&u, 1});
    (void)m_batch->takeEpisodes();
    ++m_steps;
}
void MotorViewer3D::reset(std::uint64_t seed) {
    m_batch->reset(seed);
    m_random.seed(seed);
    m_normal.reset();
    m_steps = 0;
}
void MotorViewer3D::setLook(float radians) { m_batch->setLookYaw(0, radians); }
BodyWorldView3D MotorViewer3D::view() const {
    BodyWorldView3D v;
    v.state = &m_batch->body(0);
    v.steps = m_steps;
    v.command.moveVelocity = m_batch->commandVelocity(0);
    v.command.faceHeading = m_batch->lookYaw(0);
    v.velocity = rlBodyCenterOfMassVelocity3D(profile(), v.state->links);
    for (std::size_t i = 0; i < profile().links.size(); ++i)
        if (profile().links[i].id == "UpperChest") {
            const Quaternion q =
                (v.state->links[i].orientation * profile().links[i].modelOrientation.conjugate()).normalized();
            const Vec3 f = q.rotate({1, 0, 0});
            v.chestHeading = std::atan2(f.y, f.x);
        }
    return v;
}
} // namespace MatterEngine
