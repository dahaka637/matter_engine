#include "Engine/RL/Environment/TrainingBatch3D.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <numbers>
#include <random>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace {

using namespace MatterEngine;

void require(bool condition, const std::string& message) {
    if (!condition) throw std::runtime_error(message);
}

std::filesystem::path taskPath() {
    return std::filesystem::path(MATTERENGINE_TEST_ASSETS_DIR) / "rl/tasks/h1_ficar_em_pe.json";
}

TrainingBatchSettings3D settings(std::uint32_t environments, std::uint32_t threads) {
    TrainingBatchSettings3D value;
    value.taskPath = taskPath();
    value.environmentCount = environments;
    value.environmentsPerScene = 4;
    value.workerThreads = threads;
    value.seed = 11;
    return value;
}

bool finiteAll(std::span<const float> values) {
    for (const float v : values)
        if (!std::isfinite(v)) return false;
    return true;
}

void testContract() {
    TrainingBatch3D batch(settings(6, 2));
    require(batch.actionSize() == 39, "a acao tem um canal por eixo atuado");
    require(batch.observationSize() == batch.observationLayout().size(), "observacao e layout divergem");
    require(batch.criticObservationSize() == batch.observationSize() + 7 + batch.body().profile.links.size(),
        "o critico e a observacao mais o mundo");
    require(batch.rewardComponentCount() == TrainingRewardComponentNames3D.size(), "parcelas da recompensa");
    require(batch.observations().size() == 6 * batch.observationSize(), "buffer de observacoes");
    require(finiteAll(batch.observations()) && finiteAll(batch.criticObservations()), "reset com valor invalido");
    // Recem-posto de pe: a pelve na altura de pe e os dois pes no chao (depois
    // de um passo para os contatos existirem).
    const std::size_t O = batch.observationSize();
    for (std::size_t e = 0; e < 6; ++e)
        require(std::abs(batch.observations()[e * O]) < 0.1f, "a pelve deveria comecar na altura de pe");
    std::vector<float> actions(6 * batch.actionSize(), 0.0f);
    batch.step(actions);
    require(finiteAll(batch.observations()) && finiteAll(batch.rewards()), "passo com valor invalido");
    const auto& layout = batch.observationLayout().channels;
    std::size_t leftContact = 0;
    for (std::size_t i = 0; i < layout.size(); ++i)
        if (layout[i].name == "LeftFoot.contact") leftContact = i;
    require(leftContact > 0, "o canal do contato do pe esquerdo existe");
    for (std::size_t e = 0; e < 6; ++e)
        require(batch.observations()[e * O + leftContact] == 1.0f, "o pe esquerdo deveria estar apoiado no inicio");
    // A recompensa e a soma das parcelas.
    const std::size_t R = batch.rewardComponentCount();
    for (std::size_t e = 0; e < 6; ++e) {
        float sum = 0.0f;
        for (std::size_t r = 0; r < R; ++r) sum += batch.rewardComponents()[e * R + r];
        require(std::abs(sum - batch.rewards()[e]) < 1e-4f, "recompensa diferente da soma das parcelas");
    }
}

// A observacao fica no referencial do rumo: sem ruido e sem variacao, o mesmo
// corpo virado para lados diferentes (o rumo sorteado de cada mundo) ve a
// mesma coisa, no inicio e depois de alguns passos.
void testHeadingInvariance() {
    constexpr std::size_t Worlds = 6;
    TrainingBatch3D batch(settings(Worlds, 2));
    TrainingTask3D task = batch.task();
    task.initialState.jointNoiseRadians = 0.0f;
    task.initialState.jointVelocityNoise = 0.0f;
    task.initialState.rootVelocityNoise = 0.0f;
    task.initialState.rootAngularVelocityNoise = 0.0f;
    task.initialState.randomYaw = true;
    task.randomization.friction = { 0.9f, 0.9f };
    task.randomization.mass = { 1.0f, 1.0f };
    task.randomization.strength = { 1.0f, 1.0f };
    task.randomization.actionDelayProbability = 0.0f;
    task.randomization.observationNoise = 0.0f;
    batch.updateTask(task);
    batch.reset(21);
    // O rumo como o lote mede (Z para cima, o X da pelve no referencial do modelo).
    const Quaternion model = batch.body().profile.links.front().modelOrientation.conjugate();
    const auto heading = [&](std::size_t e) {
        const Quaternion frame = batch.view(e).state->links.front().orientation * model;
        const Vec3 forward = frame.rotate(Vec3 { 1.0f, 0.0f, 0.0f });
        return std::atan2(forward.y, forward.x);
    };
    float spread = 0.0f;
    for (std::size_t e = 1; e < Worlds; ++e) spread = std::max(spread, std::abs(heading(e) - heading(0)));
    require(spread > 0.3f, "os mundos deveriam comecar virados para lados diferentes");
    const std::size_t O = batch.observationSize();
    const auto& layout = batch.observationLayout().channels;
    std::vector<float> actions(Worlds * batch.actionSize(), 0.0f);
    // No comeco, exato; nos primeiros passos, quase. Depois a fisica diverge
    // sozinha (a sola da chuteira e um poligono e o Jolt escolhe 4 pontos
    // dele para o contato, uma escolha que muda com a orientacao): ali nao ha
    // mais o que comparar.
    for (int step = 0; step <= 2; ++step) {
        if (step > 0) batch.step(actions);
        const auto obs = batch.observations();
        for (std::size_t e = 1; e < Worlds; ++e)
            for (std::size_t i = 0; i < O; ++i)
                require(std::abs(obs[e * O + i] - obs[i]) < (step == 0 ? 1e-4f : 1e-2f),
                    "o canal " + layout[i].name + " depende do rumo (passo " + std::to_string(step) + ", "
                        + std::to_string(obs[e * O + i]) + " x " + std::to_string(obs[i]) + ")");
    }
}

// As posturas: o comando no fim da observacao (zero na H1), a postura forcada
// (a bateria, o visualizador) partindo da pose dela e a tarefa da postura de
// alerta sorteando as duas no treino.
void testPostures() {
    {
        TrainingBatch3D batch(settings(4, 1));
        const auto& channels = batch.observationLayout().channels;
        const std::size_t O = batch.observationSize();
        require(channels[O - 6].name == "command.posture.alerta" && channels[O - 5].name == "command.velocity.forward"
                && channels[O - 3].name == "command.velocity.yaw" && channels.back().name == "command.gait.cos",
            "os comandos (postura, velocidade, relogio) fecham a observacao");
        for (std::size_t e = 0; e < 4; ++e)
            require(batch.observations()[e * O + O - 6] == 0.0f && batch.view(e).posture == 0
                    && batch.observations()[e * O + O - 1] == 0.0f,
                "a H1 pede a postura natural, sem relogio");
        require(batch.taskPostureIds() == std::vector<std::string> { "natural" }, "as posturas da H1");
    }
    {
        TrainingBatchSettings3D s = settings(4, 1);
        s.evaluation = true;
        s.posture = "alerta";
        TrainingBatch3D batch(s);
        TrainingTask3D task = batch.task();
        task.initialState.jointNoiseRadians = 0.0f;
        task.initialState.jointVelocityNoise = 0.0f;
        task.initialState.rootVelocityNoise = 0.0f;
        task.initialState.rootAngularVelocityNoise = 0.0f;
        batch.updateTask(task);
        batch.reset(5);
        const RLBody3D& body = batch.body();
        const RLPosture3D& alert = body.postures[1];
        const auto& channels = batch.observationLayout().channels;
        const std::size_t O = batch.observationSize();
        const auto obs = batch.observations();
        for (std::size_t i = 0; i < channels.size(); ++i) {
            const std::string& name = channels[i].name;
            const std::string suffix = ".position_from_base";
            if (name.size() <= suffix.size() || name.compare(name.size() - suffix.size(), suffix.size(), suffix) != 0)
                continue;
            for (std::size_t d = 0; d < body.dofs.size(); ++d)
                if (body.dofs[d].name + suffix == name)
                    require(std::abs(obs[i] - (alert.dofRadians[d] - body.dofs[d].baseRadians)) < 2e-3f,
                        "o alerta comeca na pose dele: " + name);
        }
        require(obs[O - 6] == 1.0f && batch.view(0).posture == 1, "o comando pede o alerta");
        require(obs[0] < -0.01f, "a pelve fica mais baixa que a da base");
    }
    {
        TrainingBatchSettings3D s = settings(48, 2);
        s.taskPath = std::filesystem::path(MATTERENGINE_TEST_ASSETS_DIR) / "rl/tasks/h1b_em_pe_posturado.json";
        TrainingBatch3D batch(s);
        require(batch.taskPostureIds() == std::vector<std::string> { "natural", "alerta" }, "as posturas da H1b");
        std::size_t alert = 0;
        const std::size_t O = batch.observationSize();
        for (std::size_t e = 0; e < 48; ++e) {
            const bool asked = batch.observations()[e * O + O - 6] == 1.0f;
            require(asked == (batch.view(e).posture == 1), "o comando e a postura do episodio");
            alert += asked ? 1 : 0;
        }
        require(alert > 20 && alert < 48, "o treino sorteia as duas posturas (70% alerta)");
    }
}

// O comando de velocidade (andar): no treino, sorteado ate o limite de agora
// do curriculo (para frente so, nesta etapa) com uma fatia parada; na bateria,
// o roteiro; de fora, o fixo. Os mundos ficam longe uns dos outros.
void testCommands() {
    // O andar por comando de velocidade (a H3 v6; a do repositorio virou a corrida ate o alvo).
    const std::filesystem::path walking = std::filesystem::path(MATTERENGINE_TEST_DATA_DIR) / "andar_comando.json";
    TrainingBatchSettings3D s = settings(64, 2);
    s.taskPath = walking;
    TrainingBatch3D batch(s);
    const std::size_t O = batch.observationSize();
    const auto commandOf = [&](std::size_t e) {
        const auto obs = batch.observations();
        return Vec3 { obs[e * O + O - 5] * 3.0f, obs[e * O + O - 4] * 3.0f, obs[e * O + O - 3] * 2.0f };
    };
    std::size_t standing = 0;
    for (std::size_t e = 0; e < 64; ++e) {
        const Vec3 c = commandOf(e);
        require(c.x >= 0.0f && c.x <= 4.5f + 1e-4f && c.y == 0.0f && c.z == 0.0f, "so para frente, ate 4,5 m/s");
        require(std::abs(c.x - batch.view(e).command.x) < 1e-4f, "o comando da observacao e o do mundo");
        standing += c.x == 0.0f ? 1 : 0;
    }
    require(standing == 0, "o treino de andar nao pede ficar parado");
    // Metade dos episodios comeca ja na velocidade pedida (no rumo do corpo), a
    // outra metade sai do parado.
    std::size_t velocityChannel = 0;
    for (std::size_t i = 0; i < batch.observationLayout().channels.size(); ++i)
        if (batch.observationLayout().channels[i].name == "pelvis.linear_velocity_heading.x") velocityChannel = i;
    std::size_t moving = 0, resting = 0;
    for (std::size_t e = 0; e < 64; ++e) {
        const float start = batch.observations()[e * O + velocityChannel] * 2.0f;
        if (std::abs(start - commandOf(e).x) < 0.45f && commandOf(e).x > 0.6f) ++moving;
        else if (std::abs(start) < 0.3f) ++resting;
    }
    require(moving > 12 && resting > 12, "uns comecam na velocidade pedida e outros parados");
    for (std::size_t e = 0; e < 64; ++e) require(batch.view(e).posture == 1, "o andar e todo na postura de alerta");
    require(std::abs(batch.view(1).origin.x - batch.view(0).origin.x) > 200.0f, "os mundos de andar ficam longe");
    TrainingTask3D task = batch.task();
    task.commands.limits.forward = 1.0f;
    batch.updateTask(task);
    batch.reset(9);
    for (std::size_t e = 0; e < 64; ++e) require(commandOf(e).x <= 1.0f + 1e-4f, "o limite do curriculo segura o sorteio");
    batch.setCommand(3, { 2.0f, 0.0f, 0.0f });
    std::vector<float> actions(64 * batch.actionSize(), 0.0f);
    batch.step(actions);
    require(std::abs(commandOf(3).x - 2.0f) < 1e-4f, "o comando fixo vale");

    TrainingBatchSettings3D e = settings(2, 1);
    e.taskPath = walking;
    e.evaluation = true;
    e.posture = "alerta";
    e.commandSchedule = { { 0.0f, 0.0f, 0.0f, 0.0f }, { 0.5f, 1.5f, 0.0f, 0.0f } };
    TrainingBatch3D battery(e);
    std::vector<float> still(2 * battery.actionSize(), 0.0f);
    require(battery.view(0).command.x == 0.0f, "o roteiro comeca parado");
    for (int i = 0; i < 30; ++i) battery.step(still);
    require(battery.view(0).command.x == 1.5f && battery.view(1).command.x == 1.5f, "o roteiro pede 1,5 m/s depois de 0,5 s");
    // O relogio corre andando (o cosseno da fase sai de 1) e fica em zero parado.
    const std::size_t B = battery.observationSize();
    for (int i = 0; i < 5; ++i) battery.step(still);
    require(battery.observations()[B - 1] < 0.999f && battery.observations()[B - 1] > -1.0f, "o relogio de passada corre andando");

    // A referencia de passada: so andando, no maximo o peso, e some acima da
    // faixa de fade. Uma junta que o corpo nao tem e erro.
    constexpr std::size_t R = TrainingRewardComponentNames3D.size();
    std::size_t referenceIndex = R;
    for (std::size_t i = 0; i < R; ++i)
        if (std::string_view(TrainingRewardComponentNames3D[i]) == "gaitReference") referenceIndex = i;
    require(referenceIndex < R - 1 && std::string_view(TrainingRewardComponentNames3D[R - 1]) == "fall",
        "a parcela da referencia antes da queda (a queda e a ultima)");
    TrainingTask3D guided = battery.task();
    guided.reward.gaitReference = 2.0f;
    guided.gait.referenceHipFlexion = 0.4f;
    guided.gait.referenceFade = { 2.0f, 3.5f };
    guided.gait.referenceJoints = { { "LeftThigh.swing1", 0, -1.0f }, { "LeftShin.twist", 0, 2.0f },
        { "RightThigh.swing1", 1, -1.0f }, { "RightShin.twist", 1, 2.0f } };
    battery.updateTask(guided);
    battery.setCommand(1, { 4.0f, 0.0f, 0.0f });
    float walked = 0.0f;
    for (int i = 0; i < 6; ++i) {
        battery.step(still);
        const float a = battery.rewardComponents()[0 * R + referenceIndex];
        const float b = battery.rewardComponents()[1 * R + referenceIndex];
        require(a > -0.21f && a <= 2.0f + 1e-4f, "a referencia vale no maximo o peso");
        require(b == 0.0f, "acima da faixa de fade a referencia some");
        walked += a;
    }
    require(walked != 0.0f, "andando a 1,5 m/s a referencia conta");
    guided.gait.referenceJoints.push_back({ "LeftKnee.swing9", 0, 1.0f });
    bool refused = false;
    try {
        battery.updateTask(guided);
    } catch (const std::runtime_error&) {
        refused = true;
    }
    require(refused, "uma junta que o corpo nao tem e erro");

    // Medido contra ficar parado, e o total nunca negativo: parado sob um
    // pedido de andar, a velocidade vale 0 e o passo vale 0 (nao menos).
    TrainingTask3D relative = battery.task();
    relative.reward.gaitReference = 0.0f;
    relative.gait.referenceJoints.clear();
    relative.commands.relativeToStanding = true;
    relative.reward.positiveOnly = true;
    battery.updateTask(relative);
    battery.setCommand(0, { 1.5f, 0.0f, 0.0f });
    std::size_t velocityIndex = R;
    for (std::size_t i = 0; i < R; ++i)
        if (std::string_view(TrainingRewardComponentNames3D[i]) == "velocity") velocityIndex = i;
    std::size_t yawIndex = R, gaitIndex = R;
    for (std::size_t i = 0; i < R; ++i) {
        if (std::string_view(TrainingRewardComponentNames3D[i]) == "yawRate") yawIndex = i;
        if (std::string_view(TrainingRewardComponentNames3D[i]) == "gaitContact") gaitIndex = i;
    }
    for (int i = 0; i < 4; ++i) {
        battery.step(still);
        require(battery.rewardComponents()[velocityIndex] < 0.05f * relative.reward.velocity,
            "parado sob pedido de andar, a velocidade nao paga");
        require(battery.rewardComponents()[yawIndex] < 0.05f * relative.reward.yawRate,
            "parado sob pedido de andar, o nao girar nao paga");
        require(battery.rewardComponents()[gaitIndex] <= 0.0f, "parado, o relogio de passada nunca paga");
        require(battery.rewards()[0] == 0.0f, "parado sob pedido de andar, o passo vale exatamente 0");
    }
}

// Mesma semente, mesmas acoes: o mesmo resultado, com 1 ou 3 threads.
void testDeterminism() {
    TrainingBatch3D a(settings(8, 1));
    TrainingBatch3D b(settings(8, 3));
    std::mt19937 rng(5);
    std::uniform_real_distribution<float> unit(-0.3f, 0.3f);
    std::vector<float> actions(8 * a.actionSize());
    for (int step = 0; step < 40; ++step) {
        for (float& v : actions) v = unit(rng);
        a.step(actions);
        b.step(actions);
    }
    const auto oa = a.observations(), ob = b.observations();
    for (std::size_t i = 0; i < oa.size(); ++i)
        require(oa[i] == ob[i], "o lote deveria ser deterministico entre contagens de threads");
}

// Acoes aleatorias fortes derrubam o corpo: o episodio acaba por queda, o
// resumo existe, o critico do estado final fica guardado e o mundo recomeca
// de pe no mesmo passo.
void testFallsAndRestart() {
    TrainingBatch3D batch(settings(8, 2));
    std::mt19937 rng(3);
    std::uniform_real_distribution<float> unit(-6.0f, 6.0f);
    std::vector<float> actions(8 * batch.actionSize());
    std::size_t falls = 0;
    for (int step = 0; step < 150 && falls < 3; ++step) {
        for (float& v : actions) v = unit(rng);
        batch.step(actions);
        for (const TrainingEpisode3D& episode : batch.finishedEpisodes()) {
            require(episode.reason == TrainingEndReason3D::FallContact
                    || episode.reason == TrainingEndReason3D::FallHeight,
                std::string("fim inesperado: ") + trainingEndReasonName3D(episode.reason));
            require(batch.terminated()[episode.environment] == 1, "queda deveria marcar terminated");
            require(episode.trunkTurnRms > 0.05f && std::isfinite(episode.trunkTurnRms),
                "o tronco gira numa queda (o giro do resumo)");
            require(episode.footTurnMax >= 0.0f && episode.footTurnMax <= 3.1416f, "o giro do pe e um angulo");
            // (O ruido inicial das juntas muda a altura da pelve em uns cm.)
            require(std::abs(batch.observations()[episode.environment * batch.observationSize()]) < 0.2f,
                "o mundo deveria recomecar de pe");
            ++falls;
        }
    }
    require(falls >= 3, "acoes aleatorias fortes deveriam derrubar o corpo");
}

// O limite de tempo trunca (nao termina) e guarda o critico do estado final.
void testTimeLimit() {
    const std::filesystem::path dir = std::filesystem::temp_directory_path() / "matter_rl_task_test";
    std::filesystem::create_directories(dir);
    const std::filesystem::path path = dir / "curta.json";
    {
        std::ifstream in(taskPath());
        std::string text((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
        const std::string key = "\"episodeSeconds\": 20.0";
        const auto at = text.find(key);
        require(at != std::string::npos, "a tarefa H1 deveria declarar episodeSeconds");
        text.replace(at, key.size(), "\"episodeSeconds\": 0.5");
        const std::string body = "\"body\": \"../bodies/football_player_v1.json\"";
        const auto bodyAt = text.find(body);
        text.replace(bodyAt, body.size(),
            "\"body\": \"" + (std::filesystem::path(MATTERENGINE_TEST_ASSETS_DIR) / "rl/bodies/football_player_v1.json").string() + "\"");
        std::ofstream out(path);
        out << text;
    }
    TrainingBatchSettings3D s = settings(4, 1);
    s.taskPath = path;
    TrainingBatch3D batch(s);
    std::vector<float> actions(4 * batch.actionSize(), 0.0f);
    bool truncatedSeen = false;
    for (int step = 0; step < 20; ++step) {
        batch.step(actions);
        for (const TrainingEpisode3D& episode : batch.finishedEpisodes()) {
            if (episode.reason != TrainingEndReason3D::TimeLimit) continue;
            truncatedSeen = true;
            require(batch.truncated()[episode.environment] == 1 && batch.terminated()[episode.environment] == 0,
                "o limite de tempo trunca");
            // (O primeiro episodio de cada mundo comeca com o relogio sorteado, e
            // no treino cada episodio dura a tarefa +-20%.)
            require(episode.steps >= 1 && episode.steps <= 18, "0,5 s +20% a 30 Hz sao ate 18 decisoes");
            const std::size_t C = batch.criticObservationSize();
            const auto terminal = batch.terminalCriticObservations().subspan(episode.environment * C, C);
            require(finiteAll(terminal), "o critico final deveria estar guardado");
        }
    }
    require(truncatedSeen, "o episodio curto deveria acabar pelo tempo");
    std::filesystem::remove_all(dir);
}

// Um professor que manda nas juntas como o RL, com uma acao fixa: o rotulo
// do lote (o alvo que da o torque dele) tem de devolver a acao.
class EchoTeacher final : public TrainingTeacher3D {
public:
    EchoTeacher(const RLBody3D& body, std::vector<float> action) : m_body(body), m_action(std::move(action)) {}
    void attach(const TrainingTeacherWorld3D&) override { ++attached; }
    void beginEpisode(const TrainingTeacherWorld3D&, const RagdollState3D&, std::uint64_t) override { ++episodes; }
    void control(const TrainingTeacherWorld3D& world, const RagdollState3D&, float) override {
        std::vector<RagdollDriveTarget3D> targets(m_body.dofs.size());
        for (std::size_t d = 0; d < m_body.dofs.size(); ++d) {
            const RLBodyDof3D& dof = m_body.dofs[d];
            RagdollDriveTarget3D& target = targets[d];
            target.linkIndex = dof.linkIndex;
            target.axis = dof.axis;
            target.positionRadians = dof.target(m_action[d]);
            target.stiffnessScale = dof.stiffnessScale;
            target.dampingScale = dof.dampingScale;
            target.maximumTorqueScale = dof.maximumTorqueScale;
            target.gravityCompensationScale = 0.0f;
        }
        world.scene->setRagdollActiveDriveTargets(world.ragdoll, targets, false);
    }
    [[nodiscard]] Vec3 command(std::size_t) const override { return { 1.0f, 0.0f, 0.0f }; }
    [[nodiscard]] float gaitPhase(std::size_t) const override { return 0.3f; }
    int attached = 0;
    int episodes = 0;

private:
    const RLBody3D& m_body;
    std::vector<float> m_action;
};

void testTeacher() {
    TrainingBatchSettings3D s = settings(4, 2);
    s.taskPath = std::filesystem::path(MATTERENGINE_TEST_DATA_DIR) / "andar_comando.json";
    const RLBody3D body = loadRLBody3D(loadTrainingTask3D(s.taskPath).bodyPath);
    std::mt19937 rng(3);
    std::uniform_real_distribution<float> unit(-0.6f, 0.6f);
    std::vector<float> action(body.dofs.size());
    for (float& a : action) a = unit(rng);
    auto teacher = std::make_shared<EchoTeacher>(body, action);
    s.teacher = teacher;
    TrainingBatch3D batch(s);
    require(teacher->attached == 4 && teacher->episodes >= 4, "o professor conhece cada mundo e cada episodio");
    const std::size_t A = batch.actionSize();
    std::vector<float> ignored(4 * A, 0.0f);
    // (Alguns passos para o corpo chegar perto dos alvos: perto, nada satura.)
    for (int i = 0; i < 3; ++i) batch.step(ignored);
    // (Os ombros, de tres eixos acoplados num alvo de orientacao, e o pe de
    // apoio, preso pelo chao, ficam mais longe: o rotulo e aproximado.)
    std::size_t close = 0, total = 0;
    double error = 0.0;
    for (std::size_t e = 0; e < 4; ++e)
        for (std::size_t d = 0; d < A; ++d) {
            const RLBodyDof3D& dof = body.dofs[d];
            const float target = dof.baseRadians + dof.actionScale * action[d];
            if (target <= dof.minimumRadians + 0.02f || target >= dof.maximumRadians - 0.02f) continue;
            ++total;
            const float miss = std::abs(batch.teacherActions()[e * A + d] - action[d]);
            close += miss < 0.15f ? 1 : 0;
            error += miss;
        }
    require(total > 100 && close >= total * 8 / 10 && error / static_cast<double>(total) < 0.08,
        "o rotulo devolve a acao de quem manda como o RL");
    // O comando e a fase sao os do professor (no rumo do corpo).
    require(std::hypot(batch.view(0).command.x, batch.view(0).command.y) > 0.9f, "o comando e o do professor");
    const std::size_t O = batch.observationSize();
    require(std::abs(batch.observations()[O - 2] - std::sin(2.0f * std::numbers::pi_v<float> * 0.3f)) < 1e-4f,
        "a fase do relogio e a do professor");
}


// A corrida ate o alvo (a H3 v7): a imitacao do clipe e o alvo.
std::size_t rewardIndex(std::string_view name) {
    for (std::size_t i = 0; i < TrainingRewardComponentNames3D.size(); ++i)
        if (TrainingRewardComponentNames3D[i] == name) return i;
    throw std::runtime_error("parcela desconhecida: " + std::string(name));
}

std::size_t channel(const TrainingBatch3D& batch, std::string_view name) {
    const auto& channels = batch.observationLayout().channels;
    for (std::size_t i = 0; i < channels.size(); ++i)
        if (channels[i].name == name) return i;
    throw std::runtime_error("canal ausente: " + std::string(name));
}

// Uma copia da corrida com trocas no texto (caminhos absolutos: a copia mora fora).
std::filesystem::path raceVariant(const std::string& name, const std::vector<std::pair<std::string, std::string>>& swaps) {
    const std::filesystem::path assets = MATTERENGINE_TEST_ASSETS_DIR;
    std::ifstream in(assets / "rl/tasks/h3_andar.json");
    std::string text((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    std::vector<std::pair<std::string, std::string>> all {
        { "\"body\": \"../bodies/football_player_v1.json\"",
            "\"body\": \"" + (assets / "rl/bodies/football_player_v1.json").string() + "\"" },
        { "\"clip\": \"../../animations/clips/", "\"clip\": \"" + (assets / "animations/clips").string() + "/" } };
    all.insert(all.end(), swaps.begin(), swaps.end());
    for (const auto& [from, to] : all) {
        bool found = false;
        for (std::size_t at = text.find(from); at != std::string::npos; at = text.find(from, at + to.size())) {
            text.replace(at, from.size(), to);
            found = true;
        }
        require(found, "a corrida deveria ter " + from);
    }
    const std::filesystem::path dir = std::filesystem::temp_directory_path() / "matter_rl_task_test";
    std::filesystem::create_directories(dir);
    const std::filesystem::path path = dir / name;
    std::ofstream(path) << text;
    return path;
}

// A acao que leva cada junta a pose base (com a acao corrigindo a referencia,
// a base e a referencia menos o desvio dela): o contrario de seguir o clipe.
std::vector<float> baseActions(const TrainingBatch3D& batch) {
    const std::size_t N = batch.environmentCount(), A = batch.actionSize(), O = batch.observationSize();
    const std::size_t first = channel(batch, "reference." + batch.body().dofs.front().name);
    std::vector<float> actions(N * A);
    for (std::size_t e = 0; e < N; ++e)
        for (std::size_t d = 0; d < A; ++d)
            actions[e * A + d] = -batch.observations()[e * O + first + d] / batch.body().dofs[d].actionScale;
    return actions;
}

void testRace() {
    const std::size_t pose = rewardIndex("imitationPose"), speed = rewardIndex("targetSpeed"),
                      reached = rewardIndex("targetReached");
    const std::size_t R = TrainingRewardComponentNames3D.size();
    {
        // A avaliacao sai do parado, com o alvo 10 m a frente na direcao da partida.
        TrainingBatchSettings3D s = settings(4, 2);
        s.taskPath = std::filesystem::path(MATTERENGINE_TEST_ASSETS_DIR) / "rl/tasks/h3_andar.json";
        s.evaluation = true;
        s.evaluationEpisodeSeconds = 12.0f;
        TrainingBatch3D batch(s);
        const std::size_t O = batch.observationSize();
        const std::size_t distance = channel(batch, "target.distance"), start = channel(batch, "reference.start");
        for (std::size_t e = 0; e < 4; ++e) {
            const TrainingEnvironmentView3D view = batch.view(e);
            require(view.hasTarget && view.targetsReached == 0, "o alvo existe desde o comeco");
            const Vec3 pelvis = view.state->links.front().position;
            require(std::abs(std::hypot(view.target.x - pelvis.x, view.target.y - pelvis.y) - 10.0f) < 0.05f,
                "o primeiro alvo fica a 10 m");
            require(std::abs(batch.observations()[e * O + distance] - 1.0f) < 0.01f, "a distancia do alvo na observacao");
            require(batch.observations()[e * O + start] == 1.0f, "a avaliacao sai da partida");
        }
        // Acao zero segue a referencia (a acao corrige a pose dela): a nota da
        // pose no primeiro passo e alta; levando as juntas a base, bem menor.
        batch.step(std::vector<float>(4 * batch.actionSize(), 0.0f));
        float following = 0.0f;
        for (std::size_t e = 0; e < 4; ++e) following += batch.rewardComponents()[e * R + pose] / 0.5f / 4.0f;
        require(following > 0.6f, "seguir a referencia vale a pose (" + std::to_string(following) + ")");
        batch.reset(11);
        batch.step(baseActions(batch));
        float holding = 0.0f;
        for (std::size_t e = 0; e < 4; ++e) holding += batch.rewardComponents()[e * R + pose] / 0.5f / 4.0f;
        require(holding < following, "segurar a base vale menos que seguir a referencia");
    }
    {
        // No treino, metade sai do parado e metade ja no sprint (a velocidade do clipe).
        TrainingBatchSettings3D s = settings(64, 2);
        s.taskPath = std::filesystem::path(MATTERENGINE_TEST_ASSETS_DIR) / "rl/tasks/h3_andar.json";
        TrainingBatch3D batch(s);
        const std::size_t O = batch.observationSize(), start = channel(batch, "reference.start");
        const std::size_t forward = channel(batch, "com.velocity_heading.x");
        std::size_t sprinting = 0, fast = 0;
        for (std::size_t e = 0; e < 64; ++e) {
            if (batch.observations()[e * O + start] == 0.0f) ++sprinting;
            if (batch.observations()[e * O + forward] * 2.0f > 4.0f) ++fast;
        }
        require(sprinting > 16 && sprinting < 48, "metade dos episodios no sprint (" + std::to_string(sprinting) + ")");
        require(fast + 4 >= sprinting && fast <= sprinting + 4, "quem sai no sprint ja corre (" + std::to_string(fast) + ")");
    }
    {
        // Um alvo a 1,2 m de quem sai no sprint: ele chega, paga a velocidade do
        // trecho e o proximo nasce 1,2 m adiante.
        const std::filesystem::path near = raceVariant("alvo_perto.json",
            { { "\"distance\": 10.0", "\"distance\": 1.2" }, { "\"standingStart\": 0.5", "\"standingStart\": 0.0" },
              { "\"firstAngleDegrees\": 10.0", "\"firstAngleDegrees\": 0.0" } });
        TrainingBatchSettings3D s = settings(8, 2);
        s.taskPath = near;
        TrainingBatch3D batch(s);
        std::vector<Vec3> first;
        for (std::size_t e = 0; e < 8; ++e) first.push_back(batch.view(e).target);
        float paid = 0.0f, toward = 0.0f;
        std::size_t arrived = 0;
        for (int step = 0; step < 6; ++step) {
            batch.step(std::vector<float>(8 * batch.actionSize(), 0.0f));
            for (std::size_t e = 0; e < 8; ++e) {
                paid += batch.rewardComponents()[e * R + reached];
                toward += batch.rewardComponents()[e * R + speed];
            }
        }
        for (std::size_t e = 0; e < 8; ++e) {
            const TrainingEnvironmentView3D view = batch.view(e);
            if (view.targetsReached == 0) continue;
            ++arrived;
            require(std::abs((view.target - first[e]).length() - 1.2f * static_cast<float>(view.targetsReached)) < 0.05f,
                "o proximo alvo nasce 1,2 m adiante");
        }
        require(arrived >= 6, "quem corre chega ao alvo perto (" + std::to_string(arrived) + " de 8)");
        require(paid > 0.0f && toward > 0.0f, "chegar e ir em direcao ao alvo pagam");
    }
    {
        // O alvo atras (a avaliacao o poe a 180 graus): correndo para longe, o
        // episodio acaba como alvo perdido, sem queda.
        const std::filesystem::path behind = raceVariant("alvo_atras.json",
            { { "\"evaluationAngles\": [0.0]", "\"evaluationAngles\": [180.0]" },
              { "\"missMeters\": 3.0", "\"missMeters\": 0.5" } });
        TrainingBatchSettings3D s = settings(4, 2);
        s.taskPath = behind;
        s.evaluation = true;
        s.evaluationEpisodeSeconds = 12.0f;
        TrainingBatch3D batch(s);
        bool missed = false;
        for (int step = 0; step < 45 && !missed; ++step) {
            batch.step(std::vector<float>(4 * batch.actionSize(), 0.0f));
            for (const TrainingEpisode3D& episode : batch.finishedEpisodes())
                if (episode.reason == TrainingEndReason3D::TargetMissed) {
                    missed = true;
                    require(batch.terminated()[episode.environment] == 1, "passar do alvo termina (nao trunca)");
                }
        }
        require(missed, "indo para longe do alvo, o episodio acaba como alvo perdido");
    }
}


// Todas as direcoes (H4, livre): o planejador escolhe entre as 8 direcoes do
// trote a que anda mais perto do alvo.
void testDirections() {
    TrainingBatchSettings3D s = settings(16, 2);
    s.taskPath = std::filesystem::path(MATTERENGINE_TEST_ASSETS_DIR) / "rl/tasks/h4_direcoes.json";
    s.evaluation = true;
    s.evaluationEpisodeSeconds = 15.0f;
    TrainingBatch3D batch(s);
    const std::size_t O = batch.observationSize();
    const std::vector<std::string> clips { "frente", "frente_esquerda", "esquerda", "tras_esquerda", "costas",
        "tras_direita", "direita", "frente_direita" };
    const auto clipOf = [&](std::size_t e) {
        std::size_t best = 0;
        for (std::size_t c = 1; c < clips.size(); ++c)
            if (batch.observations()[e * O + channel(batch, "reference.clip." + clips[c])]
                > batch.observations()[e * O + channel(batch, "reference.clip." + clips[best])])
                best = c;
        return clips[best];
    };
    // Os mundos ficam a 22,5 graus um do outro: os pares caem em cheio numa das 8.
    for (std::size_t e = 0; e < 16; e += 2)
        require(clipOf(e) == clips[e / 2], "o alvo a " + std::to_string(e * 22.5) + " graus pede " + clips[e / 2]
            + " (veio " + clipOf(e) + ")");
    // Livre (sem comando de olhar): o giro pedido so acerta o clipe mais perto
    // com o alvo (no maximo meio setor de 45 graus).
    const std::size_t turn = channel(batch, "target.turn_rate");
    for (std::size_t e = 0; e < 16; ++e)
        require(std::abs(batch.observations()[e * O + turn]) * 2.0f < 1.0f, "o giro pedido so corrige o clipe mais perto");
    // Seguindo o clipe (acao zero), o giro paga e nada quebra.
    const std::size_t R = TrainingRewardComponentNames3D.size(), spin = rewardIndex("targetTurn");
    float paid = 0.0f;
    for (int step = 0; step < 20; ++step) {
        batch.step(std::vector<float>(16 * batch.actionSize(), 0.0f));
        require(finiteAll(batch.observations()) && finiteAll(batch.rewards()), "a biblioteca anda sem numero ruim");
        for (std::size_t e = 0; e < 16; ++e) paid += batch.rewardComponents()[e * R + spin];
    }
    require(paid > 0.0f, "o giro paga");
}

} // namespace

void runTrainingBatchTests() {
    testContract();
    testHeadingInvariance();
    testPostures();
    testCommands();
    testDeterminism();
    testFallsAndRestart();
    testTimeLimit();
    testTeacher();
    testRace();
    testDirections();
    std::cout << "RL training batch tests passed\n";
}
