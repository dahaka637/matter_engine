#include "Engine/RL/Commands/CommandPilot3D.hpp"
#include "Engine/RL/Commands/ControlAdapter3D.hpp"
#include "Engine/RL/Contract/BodyContract3D.hpp"
#include "Engine/RL/Environment/BodyTask3D.hpp"
#include "Engine/RL/Environment/BodyTrainingBatch3D.hpp"
#include "Engine/RL/Rewards/BodyReward3D.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <numbers>
#include <random>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

using namespace MatterEngine;

constexpr float Pi = std::numbers::pi_v<float>;
constexpr float Dt = 1.0f / 30.0f;

void require(bool condition, const std::string& message) {
    if (!condition) throw std::runtime_error(message);
}

bool near(float a, float b, float tolerance = 1.0e-5f) { return std::abs(a - b) <= tolerance; }

std::string taskPath() { return std::string(MATTERENGINE_TEST_ASSETS_DIR) + "/rl/tasks/corpo2.json"; }

float wrap(float angle) { return std::atan2(std::sin(angle), std::cos(angle)); }

// O adaptador do controle: as teclas no referencial da camera, os tetos e o pulo.
void testControlAdapter() {
    const ControlSettings3D settings;
    PlayerAttributes3D player;
    player.sprintCap = 6.5f;
    ControlState3D state;
    const auto diagonal = controlKeys3D(true, false, true, false);
    require(near(std::hypot(diagonal[0], diagonal[1]), 1.0f), "a diagonal das teclas e normalizada");
    const auto opposite = controlKeys3D(true, true, false, false);
    require(opposite[0] == 0.0f && opposite[1] == 0.0f, "teclas opostas se anulam");

    ControlInput3D input;
    input.cameraHeading = Pi / 2.0f;
    input.speed = 2.0f;
    input.stickForward = 1.0f;
    BodyCommand3D command = ControlAdapter3D::command(settings, input, state, player, Dt);
    require(near(command.moveVelocity.x, 0.0f) && near(command.moveVelocity.y, 2.0f), "W anda para onde a camera olha");
    require(near(command.faceHeading, Pi / 2.0f), "o olhar e a camera");
    input.stickForward = 0.0f;
    input.stickLeft = 1.0f;
    command = ControlAdapter3D::command(settings, input, state, player, Dt);
    require(near(command.moveVelocity.x, -2.0f) && near(command.moveVelocity.y, 0.0f), "A anda para a esquerda da camera");
    input.stickForward = diagonal[0];
    input.stickLeft = diagonal[1];
    command = ControlAdapter3D::command(settings, input, state, player, Dt);
    require(near(std::hypot(command.moveVelocity.x, command.moveVelocity.y), 2.0f), "a diagonal anda na mesma velocidade");

    // Os tetos pela direcao em relacao a camera.
    const std::array<std::pair<float, float>, 5> caps {
        { { 0.0f, 1.0f }, { Pi / 4.0f, 0.9f }, { Pi / 2.0f, 0.65f }, { 3.0f * Pi / 4.0f, 0.6f }, { Pi, 0.55f } }
    };
    input.speed = 20.0f;
    for (const auto& [relative, fraction] : caps)
        for (const float side : { 1.0f, -1.0f }) {
            input.stickForward = std::cos(side * relative);
            input.stickLeft = std::sin(side * relative);
            command = ControlAdapter3D::command(settings, input, state, player, Dt);
            require(near(std::hypot(command.moveVelocity.x, command.moveVelocity.y), 6.5f * fraction, 1.0e-4f),
                "o pedido fica no teto da direcao");
            require(near(player.capNow, 6.5f * fraction, 1.0e-4f), "o teto de agora vai para o jogador");
        }
    input.speed = 4.0f;
    input.stickForward = 0.5f;
    input.stickLeft = 0.0f;
    command = ControlAdapter3D::command(settings, input, state, player, Dt);
    require(near(std::hypot(command.moveVelocity.x, command.moveVelocity.y), 2.0f), "o analogico pela metade, metade da velocidade");
    input.stickForward = 0.01f;
    command = ControlAdapter3D::command(settings, input, state, player, Dt);
    require(command.moveVelocity.x == 0.0f && command.moveVelocity.y == 0.0f, "abaixo da zona morta e solto");

    // O pulo: espera a decolagem ate a janela acabar.
    input = {};
    input.jumpPressed = true;
    input.jumpRise = 0.3f;
    command = ControlAdapter3D::command(settings, input, state, player, Dt);
    require(command.jumpPending && near(command.jumpRise, 0.3f) && command.jumpAge == 0.0f, "o aperto vira pedido");
    input.jumpPressed = false;
    for (int i = 0; i < 10; ++i) command = ControlAdapter3D::command(settings, input, state, player, Dt);
    require(command.jumpPending && near(command.jumpAge, 10.0f * Dt), "o pedido envelhece");
    for (int i = 0; i < 10; ++i) command = ControlAdapter3D::command(settings, input, state, player, Dt);
    require(!command.jumpPending && command.jumpRise == 0.0f, "a janela acaba em 0,6 s");
    input.jumpPressed = true;
    command = ControlAdapter3D::command(settings, input, state, player, Dt);
    ControlAdapter3D::jumpTookOff(state);
    input.jumpPressed = false;
    command = ControlAdapter3D::command(settings, input, state, player, Dt);
    require(!command.jumpPending, "a decolagem atende o pedido");
}

// Um corpo de mentira que segue o comando (para ver o piloto andando).
struct FakeBody {
    Vec3 position;
    Vec3 velocity;
    void follow(const BodyCommand3D& command, float dt) {
        const float k = std::min(1.0f, dt / 0.25f);
        velocity += (command.moveVelocity - velocity) * k;
        position += velocity * dt;
    }
};

struct PilotRun {
    std::vector<float> trace;
    PilotState3D state;
    std::set<int> modes;
    float worstCapExcess = 0.0f;
    float fastestCamera = 0.0f;
};

PilotRun runPilot(const BodyTask3D& task, const BodyDrill3D& drill, std::uint64_t seed, int decisions) {
    PilotRun run;
    std::mt19937_64 rng(seed);
    FakeBody body;
    ControlState3D control;
    PlayerAttributes3D player = task.player;
    CommandPilot3D::start(drill.pilot, run.state, body.position, 0.3f, rng);
    run.modes.insert(static_cast<int>(run.state.cameraMode));
    for (int i = 0; i < decisions; ++i) {
        const ControlInput3D input = CommandPilot3D::update(drill.pilot, run.state, { body.position, body.velocity }, Dt, rng);
        const BodyCommand3D command = ControlAdapter3D::command(task.control, input, control, player, Dt);
        const float speed = std::hypot(command.moveVelocity.x, command.moveVelocity.y);
        if (speed > 0.0f) {
            const float relative = std::atan2(command.moveVelocity.y, command.moveVelocity.x) - command.faceHeading;
            run.worstCapExcess = std::max(run.worstCapExcess, speed - task.control.caps.cap(relative, player.sprintCap));
        }
        run.fastestCamera = std::max(run.fastestCamera, std::abs(command.faceRate));
        for (const float v : { input.stickForward, input.stickLeft, input.speed, input.cameraHeading, command.moveVelocity.x,
                 command.moveVelocity.y })
            run.trace.push_back(v);
        body.follow(command, Dt);
    }
    return run;
}

// O piloto: deterministico, dentro dos tetos, a camera nos modos e na
// velocidade do roteiro, a estrela chegando aos pontos.
void testPilot() {
    const BodyTask3D task = loadBodyTask3D(taskPath());
    for (const BodyDrill3D& drill : task.drills) {
        const PilotRun a = runPilot(task, drill, 11, 900);
        const PilotRun b = runPilot(task, drill, 11, 900);
        require(a.trace == b.trace, "o piloto repete com a mesma semente: " + drill.id);
        require(a.worstCapExcess <= 1.0e-4f, "o comando fica no teto da direcao: " + drill.id);
        const PilotCamera3D& camera = drill.pilot.camera;
        const float limit = std::max({ camera.followRate, camera.turnRate.maximum }) + 1.0e-3f;
        require(a.fastestCamera <= limit, "a camera gira no maximo o do roteiro: " + drill.id);
    }
    // Os modos da camera aparecem nas fatias do roteiro.
    const BodyDrill3D& stick = task.drills[static_cast<std::size_t>(findBodyDrill3D(task, "deslocamento"))];
    std::set<int> modes;
    for (std::uint64_t seed = 1; seed <= 40; ++seed) modes.merge(runPilot(task, stick, seed, 1).modes);
    require(modes.size() == 3, "a camera parada, acompanhando e em trechos aparecem");
    // O segurar troca o pedido e solta as teclas as vezes.
    const PilotRun moving = runPilot(task, stick, 5, 1800);
    require(moving.state.changes >= 10, "o segurar troca o pedido");
    // A estrela chega aos pontos e para nos de parar.
    const BodyDrill3D& star = task.drills[static_cast<std::size_t>(findBodyDrill3D(task, "estrela"))];
    const PilotRun stars = runPilot(task, star, 3, 1800);
    require(stars.state.reached >= 4, "a estrela chega aos pontos");
    // Parado: teclas soltas o tempo todo.
    const BodyDrill3D& hold = task.drills[static_cast<std::size_t>(findBodyDrill3D(task, "parado"))];
    const PilotRun still = runPilot(task, hold, 9, 600);
    for (std::size_t i = 0; i < still.trace.size(); i += 6)
        require(still.trace[i + 4] == 0.0f && still.trace[i + 5] == 0.0f, "parado pede velocidade zero");
}

float rewardOf(const BodyTask3D& task, const BodyTaskWeights3D& weights, Vec3 measured, Vec3 wanted, float facingError,
    const BodyCosts3D& costs, bool fell) {
    const float velocity = bodyVelocityScore3D(task.reward, measured, wanted);
    const float facing = bodyFacingScore3D(task.reward, 0.0f, facingError);
    return bodyReward3D(task.reward, weights, velocity, facing, costs, fell, Dt, nullptr);
}

// A ordem da nota (plano secao 7.4) e as trapacas (secao 7.7), com custos de
// tamanho plausivel (parado, andando, correndo) nos pesos do corpo2.
void testRewardOrder() {
    const BodyTask3D task = loadBodyTask3D(taskPath());
    const BodyTaskWeights3D move { 0.7f, 0.3f };
    const BodyTaskWeights3D stand { 0.5f, 0.5f };
    BodyCosts3D standing;
    standing.work = 0.02f;
    standing.torque = 0.01f;
    standing.slip = 0.01f;
    standing.head = 0.05f;
    standing.smooth = 0.002f;
    BodyCosts3D walking;
    walking.work = 0.3f;
    walking.torque = 0.04f;
    walking.slip = 0.03f;
    walking.head = 0.5f;
    walking.smooth = 0.01f;
    walking.limits = 0.02f;
    BodyCosts3D running;
    running.work = 1.0f;
    running.torque = 0.1f;
    running.slip = 0.05f;
    running.impact = 0.3f;
    running.head = 2.0f;
    running.smooth = 0.05f;
    running.limits = 0.05f;
    const Vec3 walk { 1.5f, 0.0f, 0.0f };

    // Andar no pedido contra correr acima dele.
    require(rewardOf(task, move, walk, walk, 0.0f, walking, false)
            > rewardOf(task, move, { 4.0f, 0.0f, 0.0f }, walk, 0.0f, running, false),
        "andar no pedido vale mais que correr acima dele");
    // Ficar parado com pedido de andar (a enrolacao) vale bem menos que andar.
    const float walked = rewardOf(task, move, walk, walk, 0.0f, walking, false);
    const float stalled = rewardOf(task, move, {}, walk, 0.0f, standing, false);
    require(walked > stalled && stalled < 0.6f * walked, "parado com pedido de andar nao compensa");
    // O passo de recuperacao contra a queda.
    require(rewardOf(task, move, { 0.8f, 0.0f, 0.0f }, walk, 0.3f, running, false)
            > rewardOf(task, move, walk, walk, 0.0f, standing, true),
        "o passo que segura vale mais que cair");
    // Parado quieto contra parado tremendo.
    BodyCosts3D trembling = standing;
    trembling.smooth = 0.06f;
    trembling.head = 0.6f;
    require(rewardOf(task, stand, {}, {}, 0.0f, standing, false) > rewardOf(task, stand, {}, {}, 0.0f, trembling, false),
        "parado quieto vale mais que tremendo");
    // Parar no ponto contra passar direto.
    require(rewardOf(task, move, {}, {}, 0.0f, standing, false)
            > rewardOf(task, move, { 2.0f, 0.0f, 0.0f }, {}, 0.0f, walking, false),
        "parar no ponto vale mais que passar direto");
    // O giro com apoio contra a sola deslizando.
    BodyCosts3D sliding = standing;
    sliding.slip = 0.6f;
    require(rewardOf(task, stand, {}, {}, 0.2f, standing, false) > rewardOf(task, stand, {}, {}, 0.2f, sliding, false),
        "girar com apoio vale mais que arrastar a sola");
    // Nenhum esforco plausivel faz a queda compensar: uma decisao com o dobro
    // dos custos de correr custa menos que cair, e andar no pedido paga.
    BodyCosts3D heavy = running;
    for (float* c : { &heavy.work, &heavy.torque, &heavy.slip, &heavy.impact, &heavy.head, &heavy.smooth, &heavy.limits })
        *c *= 2.0f;
    require(rewardOf(task, move, {}, walk, 0.0f, heavy, false) > -task.reward.fall, "o esforco nunca e pior que a queda");
    require(walked > 0.0f, "andar no pedido paga mais do que custa");
    require(rewardOf(task, move, { 4.5f, 0.0f, 0.0f }, { 4.5f, 0.0f, 0.0f }, 0.0f, running, false) > 0.0f,
        "correr no pedido paga mais do que custa");
    // Nenhuma parcela paga sem o objetivo.
    require(bodyVelocityScore3D(task.reward, { 3.0f, 0.0f, 0.0f }, {}) < 0.05f, "longe do pedido, a velocidade quase nao paga");
    require(bodyFacingScore3D(task.reward, 0.0f, Pi / 2.0f) < 0.01f, "o peito de lado nao paga o olhar");
    // A cauda longa da sinal onde o sino ja nao da.
    BodyRewardSettings3D bell = task.reward;
    bell.velocityForm = "bell";
    const Vec3 far { 1.0f, 0.0f, 0.0f };
    require(bodyVelocityScore3D(task.reward, far, {}) > 5.0f * bodyVelocityScore3D(bell, far, {}),
        "a cauda longa ainda paga longe do pedido");
    // A conta: dt x (tarefa - custos) - queda, com as parcelas.
    std::array<float, BodyRewardTermNames3D.size()> parts {};
    const float r = bodyReward3D(task.reward, move, 0.8f, 0.5f, running, true, Dt, parts.data());
    float costs = 0.0f;
    for (std::size_t k = static_cast<std::size_t>(BodyRewardTerm3D::Work); k < static_cast<std::size_t>(BodyRewardTerm3D::Fall); ++k)
        costs += parts[k];
    require(near(parts[static_cast<std::size_t>(BodyRewardTerm3D::Task)], 0.7f * 0.8f + 0.3f * 0.5f), "a tarefa pesa o exercicio");
    require(near(r, Dt * (parts[static_cast<std::size_t>(BodyRewardTerm3D::Task)] - costs) - task.reward.fall),
        "a nota e dt x (tarefa - custos) - queda");
}

// O esquema 2 recusa o que nao e dele.
void testTaskLoader() {
    const BodyTask3D task = loadBodyTask3D(taskPath());
    require(task.drills.size() == 3 && findBodyDrill3D(task, "estrela") >= 0, "o corpo2 tem os exercicios do Experimento B");
    require(task.contract.velocityTarget == BodyVelocityTarget3D::MeanSlope, "a velocidade pela reta da media");
    require(near(task.contract.decisionSeconds, 4.0f / 120.0f), "a decisao a 30 Hz");
    bool refused = false;
    try {
        (void)loadBodyTask3D(std::string(MATTERENGINE_TEST_ASSETS_DIR) + "/rl/tasks/corpo.json");
    } catch (const std::runtime_error&) {
        refused = true;
    }
    require(refused, "o esquema 1 nao abre como tarefa do Corpo v2");
}

BodyBatchSettings3D smallBatch(std::uint32_t worlds, std::uint32_t threads, bool evaluation) {
    BodyBatchSettings3D settings;
    settings.taskPath = taskPath();
    settings.environmentCount = worlds;
    settings.environmentsPerScene = 2;
    settings.workerThreads = threads;
    settings.seed = 7;
    settings.evaluation = evaluation;
    return settings;
}

bool allFinite(std::span<const float> values) {
    return std::all_of(values.begin(), values.end(), [](float v) { return std::isfinite(v); });
}

// O lote v2: os tamanhos, as cotas, a conta da nota e os recomecos.
void testBodyBatch() {
    BodyTrainingBatch3D batch(smallBatch(10, 2, false));
    const std::size_t A = batch.actionSize();
    const std::size_t L = batch.body().profile.links.size();
    const std::size_t D = batch.task().drills.size();
    require(A == 37 && batch.observationSize() == batch.contract().observationSize(), "a entrada e a do contrato");
    require(batch.criticObservationSize() == batch.observationSize() + 4 + L + D + 4, "o critico ve o mundo");
    require(batch.rewardComponentCount() == BodyRewardTermNames3D.size(), "as parcelas da nota");
    std::array<int, 3> perDrill {};
    for (const std::uint32_t d : batch.worldDrills()) ++perDrill[d];
    require(perDrill[0] == 2 && perDrill[1] == 5 && perDrill[2] == 3, "os mundos nas cotas do peso");
    require(allFinite(batch.observations()) && allFinite(batch.criticObservations()), "a entrada do comeco e finita");

    std::vector<float> zero(batch.environmentCount() * A, 0.0f);
    std::size_t episodes = 0, falls = 0;
    const std::size_t parts = batch.rewardComponentCount();
    for (int step = 0; step < 150; ++step) {
        batch.step(zero, zero);
        require(allFinite(batch.observations()) && allFinite(batch.criticObservations()) && allFinite(batch.rewards()),
            "tudo finito no passo " + std::to_string(step));
        for (std::size_t w = 0; w < batch.environmentCount(); ++w) {
            const float* c = batch.rewardComponents().data() + w * parts;
            float costs = 0.0f;
            for (std::size_t k = static_cast<std::size_t>(BodyRewardTerm3D::Work);
                 k < static_cast<std::size_t>(BodyRewardTerm3D::Fall); ++k)
                costs += c[k];
            const float expected = batch.policyStepSeconds() * (c[static_cast<std::size_t>(BodyRewardTerm3D::Task)] - costs)
                - c[static_cast<std::size_t>(BodyRewardTerm3D::Fall)];
            require(near(batch.rewards()[w], expected, 1.0e-5f), "a nota bate com as parcelas");
        }
        for (const BodyEpisode3D& episode : batch.finishedEpisodes()) {
            ++episodes;
            if (episode.reason == BodyEndReason3D::FallContact || episode.reason == BodyEndReason3D::FallHeight) ++falls;
            require(episode.reason != BodyEndReason3D::Numeric, "nenhum fim numerico");
            require(std::isfinite(episode.velocityErrorRms) && std::isfinite(episode.costs.work), "as medidas sao finitas");
        }
    }
    // Sem politica (a acao zero segura a postura natural), ele cai e recomeca.
    require(episodes > 0 && falls > 0, "sem politica ele cai e o mundo recomeca");
}

// O mesmo lote com a mesma semente e as mesmas acoes da o mesmo resultado,
// com qualquer numero de threads.
void testBodyBatchDeterminism() {
    BodyTrainingBatch3D a(smallBatch(6, 1, false));
    BodyTrainingBatch3D b(smallBatch(6, 3, false));
    std::mt19937 rng(5);
    std::normal_distribution<float> gauss(0.0f, 0.5f);
    std::vector<float> actions(a.environmentCount() * a.actionSize());
    for (int step = 0; step < 40; ++step) {
        for (float& v : actions) v = gauss(rng);
        a.step(actions, actions);
        b.step(actions, actions);
        require(std::equal(a.observations().begin(), a.observations().end(), b.observations().begin()),
            "a mesma entrada com threads diferentes");
        require(std::equal(a.rewards().begin(), a.rewards().end(), b.rewards().begin()), "a mesma nota com threads diferentes");
    }
}

bool contactChannel(const std::string& name) {
    for (const char* part : { ".contact_fraction", ".force_mean", ".force_peak", ".touchdown", ".liftoff" })
        if (name.find(part) != std::string::npos) return true;
    return false;
}

// A paridade de informacao: a entrada montada so com o que o jogo tem (o
// estado do boneco, o comando do adaptador e os dados do jogador) e a mesma do
// treino. Os canais de contato dependem dos passos de fisica (o jogo chama o
// mesmo accumulateContacts); aqui ficam de fora depois do primeiro passo.
void testBodyBatchInfoParity() {
    BodyTrainingBatch3D batch(smallBatch(2, 1, true));
    const BodyContract3D game(batch.body(), batch.task().contract);
    const std::size_t O = batch.observationSize();
    const std::size_t A = batch.actionSize();
    std::vector<BodyContractState3D> states(batch.environmentCount());
    std::vector<float> out(O);
    for (std::size_t w = 0; w < batch.environmentCount(); ++w) {
        const BodyWorldView3D view = batch.view(w);
        game.reset(states[w], *view.state);
        game.writeObservation(*view.state, view.command, view.player, states[w], out.data());
        for (std::size_t k = 0; k < O; ++k)
            require(out[k] == batch.observations()[w * O + k],
                "a entrada do comeco e a do jogo: " + batch.observationLayout().channels[k].name);
    }
    std::vector<float> actions(batch.environmentCount() * A);
    std::mt19937 rng(3);
    std::normal_distribution<float> gauss(0.0f, 0.3f);
    for (int step = 0; step < 6; ++step) {
        for (float& v : actions) v = gauss(rng);
        batch.step(actions, actions);
        require(batch.finishedEpisodes().empty(), "a paridade precisa do mesmo episodio");
        for (std::size_t w = 0; w < batch.environmentCount(); ++w) {
            const BodyWorldView3D view = batch.view(w);
            const std::span<const float> action(actions.data() + w * A, A);
            game.beginDecision(states[w], action, action);
            game.writeObservation(*view.state, view.command, view.player, states[w], out.data());
            for (std::size_t k = 0; k < O; ++k) {
                const std::string& name = batch.observationLayout().channels[k].name;
                if (contactChannel(name)) continue;
                require(near(out[k], batch.observations()[w * O + k], 1.0e-6f), "a entrada e a do jogo: " + name);
            }
        }
    }
}

} // namespace

void runBodyBatchTests() {
    testControlAdapter();
    testPilot();
    testRewardOrder();
    testTaskLoader();
    testBodyBatch();
    testBodyBatchDeterminism();
    testBodyBatchInfoParity();
    std::cout << "Body batch tests passed\n";
}
