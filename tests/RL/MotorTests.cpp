#include "Engine/RL/Motor/MotorTraining3D.hpp"
#include "Engine/RL/Body/RLBody3D.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <iterator>
#include <numeric>
#include <stdexcept>
#include <string>
#include <vector>
using namespace MatterEngine;
namespace {
void require(bool b, const char *why) {
    if (!b)
        throw std::runtime_error(why);
}
constexpr float Pi = 3.14159265f;
std::size_t metric(const char *name) {
    const auto &names = MotorTraining3D::episodeMetricNames();
    const auto at = std::find_if(names.begin(), names.end(), [&](const char *n) { return std::string(n) == name; });
    require(at != names.end(), "medida conhecida");
    return static_cast<std::size_t>(at - names.begin());
}
std::size_t link(const MotorTraining3D &b, const char *id) {
    const auto &links = b.profile().links;
    return static_cast<std::size_t>(
        std::find_if(links.begin(), links.end(), [&](const auto &l) { return l.id == id; }) - links.begin());
}
MotorSettings3D settingsFor(int drill, std::uint32_t worlds = 4, std::uint32_t threads = 1) {
    MotorSettings3D s;
    s.assetsDirectory = MATTERENGINE_ASSETS_DIR;
    s.taskPath = s.assetsDirectory / "rl/motor/basico.json";
    s.environments = worlds;
    s.threads = threads;
    s.evaluation = true;
    s.drill = drill;
    return s;
}
// A media da recompensa por decisao nas primeiras decisoes, com uma acao fixa.
float meanReward(MotorTraining3D &b, const std::vector<float> &action, int decisions) {
    const auto n = b.environmentCount();
    std::vector<float> a(n * b.actionSize()), r(n);
    for (std::size_t w = 0; w < n; ++w)
        std::copy(action.begin(), action.end(), a.begin() + w * b.actionSize());
    std::vector<std::uint8_t> t(n), u(n);
    double sum = 0;
    for (int k = 0; k < decisions; ++k) {
        b.step(a, r, t, u);
        sum += std::accumulate(r.begin(), r.end(), 0.0) / n;
    }
    return static_cast<float>(sum / decisions);
}
std::string recipeText() {
    std::ifstream in(std::filesystem::path(MATTERENGINE_ASSETS_DIR) / "rl/motor/basico.json");
    return {std::istreambuf_iterator<char>(in), {}};
}
std::filesystem::path writeTemporary(const std::string &text) {
    const auto path = std::filesystem::temp_directory_path() /
                      ("motor-v4-" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()) + ".json");
    std::ofstream(path) << text;
    return path;
}
} // namespace

int main() {
    // Cobertura e o zero da acao no IDLE alerta.
    MotorTraining3D one(settingsFor(1));
    require(one.actionSize() == ragdollDegreesOfFreedom3D(one.profile()), "todos os eixos");
    for (const char *name : {"Head.twist", "LeftHand.twist", "RightHand.twist", "UpperChest.twist"})
        require(std::find(one.actionNames().begin(), one.actionNames().end(), name) != one.actionNames().end(),
                "nenhum membro segue lider");
    std::vector<float> idleAction(one.actionSize());
    one.poseActions(one.idleCoordinates(), idleAction);
    require(std::all_of(idleAction.begin(), idleAction.end(), [](float v) { return std::abs(v) < 1e-3f; }),
            "a acao zero comanda o IDLE alerta");
    // Nascer: a pose do proprio idle (instante sorteado dele), perto do quadro 0.
    for (std::size_t j = 1; j < one.profile().links.size(); ++j)
        for (int a = 0; a < 3; ++a)
            require(std::abs(one.body(0).joints[j].positionRadians[a] - (&one.idleCoordinates()[j].x)[a]) < .2f,
                    "nascimento no IDLE alerta");
    std::vector<float> reference(4 * one.actionSize());
    one.referenceActions(reference);
    require(std::all_of(reference.begin(), reference.end(), [](float v) { return std::isfinite(v); }), "rotulo finito");

    // Determinismo entre threads.
    auto parallel = settingsFor(1);
    parallel.threads = 4;
    MotorTraining3D many(parallel);
    std::vector<float> a(one.actionSize() * 4), o(one.observationSize() * 4), p(o.size()), r(4), r2(4);
    std::vector<std::uint8_t> t(4), u(4), t2(4), u2(4);
    one.observations(o);
    many.observations(p);
    require(o == p, "reset deterministico");
    for (int k = 0; k < 25; ++k) {
        one.step(a, r, t, u);
        many.step(a, r2, t2, u2);
        one.observations(o);
        many.observations(p);
        require(o == p && r == r2 && t == t2 && u == u2, "paridade entre threads");
        require(std::all_of(o.begin(), o.end(), [](float v) { return std::isfinite(v); }), "observacao finita");
        require(!one.body(0).guided && !one.body(0).kinematic, "corpo dinamico sem ajuda");
    }
    const float look = one.lookYaw(0);
    one.setLookYaw(0, look + 1);
    std::vector<float> after(o.size());
    one.observations(after);
    require(after != o, "comando de olhar visivel");
    bool refused = false;
    a[0] = NAN;
    try {
        one.step(a, r, t, u);
    } catch (const std::invalid_argument &) {
        refused = true;
    }
    require(refused, "acao invalida recusada na fronteira");

    // De joelhos ou de quatro: um link que nao e pe no chao encerra o episodio (contato).
    MotorTraining3D kneel(settingsFor(0, 1));
    auto pose = kneel.idleCoordinates();
    for (const char *side : {"Left", "Right"}) {
        pose[link(kneel, (std::string(side) + "Thigh").c_str())].y = -10 * Pi / 180;
        pose[link(kneel, (std::string(side) + "Shin").c_str())].x = 110 * Pi / 180;
    }
    std::vector<float> kneelAction(kneel.actionSize()), single(1);
    std::vector<std::uint8_t> st(1), su(1);
    kneel.poseActions(pose, kneelAction);
    kneel.placePose(0, pose, {0, 0, 0, 1});
    bool ended = false;
    for (int k = 0; k < 40 && !ended; ++k) {
        kneel.step(kneelAction, single, st, su);
        ended = st[0] != 0;
    }
    const auto kneelEpisodes = kneel.takeEpisodes();
    require(ended && kneelEpisodes.size() == 1 && kneelEpisodes[0].values[metric("causa_queda")] == 1.f,
            "joelho no chao e queda por contato");

    // A ordem das recompensas: ficar de pe no IDLE paga; parado sob pedido de trote nao paga;
    // o corpo envergado rende menos que o reto.
    std::vector<float> zero(one.actionSize(), 0.f);
    MotorTraining3D stand(settingsFor(0));
    const float standing = meanReward(stand, zero, 15);
    auto trotSettings = settingsFor(2);
    trotSettings.trotSpeed = 1.f;
    MotorTraining3D trot(trotSettings);
    const float idleInTrot = meanReward(trot, zero, 30);
    MotorTraining3D bentBody(settingsFor(0));
    auto bent = bentBody.idleCoordinates();
    for (const char *id : {"Abdomen", "Chest", "UpperChest"})
        bent[link(bentBody, id)].y += 18 * Pi / 180;
    std::vector<float> bentAction(bentBody.actionSize());
    bentBody.poseActions(bent, bentAction);
    const float bentReward = meanReward(bentBody, bentAction, 15);
    std::cout << "recompensa por decisao: IDLE " << standing << ", parado no trote " << idleInTrot << ", envergado "
              << bentReward << '\n';
    require(standing > 0, "ficar de pe no IDLE paga");
    require(idleInTrot < 0, "parado sob pedido de movimento nao paga");
    require(bentReward < standing, "envergado rende menos que o reto");

    // Fim por tempo separado de queda.
    auto shortRun = settingsFor(0);
    shortRun.episodeSeconds = .1f;
    MotorTraining3D timed(shortRun);
    std::fill(a.begin(), a.end(), 0);
    bool truncated = false;
    for (int k = 0; k < 5 && !truncated; ++k) {
        timed.step(a, r, t, u);
        truncated = std::any_of(u.begin(), u.end(), [](auto v) { return v != 0; });
    }
    require(truncated && std::none_of(t.begin(), t.end(), [](auto v) { return v != 0; }), "timeout separado de queda");

    // Validacao: receita invalida recusada sem trocar a valida; tetos e cotas validados.
    std::string text = recipeText();
    const std::string alive = "\"alive\": 0.25";
    require(text.find(alive) != std::string::npos, "receita com peso de vida");
    std::string broken = text;
    broken.replace(broken.find(alive), alive.size(), "\"alive\": -1");
    const auto brokenPath = writeTemporary(broken);
    refused = false;
    try {
        stand.setTask(brokenPath);
    } catch (const std::exception &) {
        refused = true;
    }
    std::filesystem::remove(brokenPath);
    require(refused, "receita com peso negativo recusada");
    refused = false;
    try {
        const std::array<float, 8> bad{1, 1, 1, 1, 1, 1, 1, NAN};
        stand.setTrotLimits(bad);
    } catch (const std::invalid_argument &) {
        refused = true;
    }
    require(refused, "teto de trote invalido recusado");
    refused = false;
    try {
        const std::array<float, 3> bad{.5f, .5f, .5f};
        stand.setShares(bad);
    } catch (const std::invalid_argument &) {
        refused = true;
    }
    require(refused, "cotas que nao somam 1 recusadas");
    const std::array<float, 3> shares{.4f, .2f, .4f};
    stand.setShares(shares);
    require(stand.shares()[0] == .4f, "cotas trocadas");
    std::cout << "Motor v4: cobertura, IDLE como zero, quedas por contato, ordem das notas, determinismo e "
                 "validacao OK\n";
}
