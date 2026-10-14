#include "Engine/RL/Environment/TrainingBatch3D.hpp"
#include "Engine/RL/Imitation/ReferenceMotion3D.hpp"

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
std::filesystem::path raceVariant(const std::string& name, const std::vector<std::pair<std::string, std::string>>& swaps,
    const std::string& source = "h3_andar.json") {
    const std::filesystem::path assets = MATTERENGINE_TEST_ASSETS_DIR;
    std::ifstream in(assets / "rl/tasks" / source);
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

// Os varios pontos (H5): os dois proximos alvos ja no mapa e na observacao; a
// avaliacao faz os percursos fixos, um por mundo.
void testCourses() {
    const auto planar = [](Vec3 a, Vec3 b) { return std::hypot(a.x - b.x, a.y - b.y); };
    const auto near = [](float a, float b, float tolerance) { return std::abs(a - b) <= tolerance; };
    {
        TrainingBatchSettings3D s = settings(16, 2);
        s.taskPath = std::filesystem::path(MATTERENGINE_TEST_ASSETS_DIR) / "rl/tasks/h5_pontos.json";
        s.evaluation = true;
        s.evaluationEpisodeSeconds = 20.0f;
        TrainingBatch3D batch(s);
        const std::size_t O = batch.observationSize();
        // O variado (o desenho do usuario, mundo 0), o quadrado de 6 m com um
        // ponto a cada 3 m (mundo 1) e o zigue-zague (mundo 2), pela forma.
        const TrainingEnvironmentView3D varied = batch.view(0);
        require(near(planar(varied.target, varied.origin), std::hypot(1.5f, 2.2f), 1e-3f)
                && near(planar(varied.upcoming[0], varied.target), 4.0f, 1e-3f)
                && near(planar(varied.upcoming[1], varied.upcoming[0]), 4.2f, 1e-3f),
            "o mundo 0 faz o percurso variado");
        const TrainingEnvironmentView3D square = batch.view(1);
        require(square.upcomingCount == 2, "a H5 mostra os dois proximos alvos");
        const Vec3 origin = square.origin;
        require(near(planar(square.target, origin), 3.0f, 1e-3f) && near(planar(square.upcoming[0], square.target), 3.0f, 1e-3f)
                && near(planar(square.upcoming[1], square.upcoming[0]), 3.0f, 1e-3f)
                && near(planar(square.upcoming[0], origin), 6.0f, 1e-3f)
                && near(planar(square.upcoming[1], origin), std::hypot(6.0f, 3.0f), 1e-3f),
            "o mundo 0 faz o quadrado de 6 m");
        const TrainingEnvironmentView3D zigzag = batch.view(2);
        require(near(planar(zigzag.target, zigzag.origin), std::hypot(2.5f, 2.0f), 1e-3f)
                && near(planar(zigzag.upcoming[0], zigzag.origin), 5.0f, 1e-3f)
                && near(planar(zigzag.upcoming[1], zigzag.origin), std::hypot(7.5f, 2.0f), 1e-3f),
            "o mundo 1 faz o zigue-zague (recomecando do ultimo ponto)");
        // Os proximos na observacao: a distancia ate o primeiro da fila.
        // Nenhum percurso repete ponto na janela do visualizador (3 alcancados,
        // o de agora e os 2 proximos): o alcancado nunca volta a ser vermelho.
        {
            const TrainingTask3D h7 = loadTrainingTask3D(std::filesystem::path(MATTERENGINE_TEST_ASSETS_DIR) / "rl/tasks/h7_tiro.json");
            std::vector<TrainingTask3D::Target::Course> courses = loadTrainingTask3D(s.taskPath).target.evaluationCourses;
            courses.insert(courses.end(), h7.target.evaluationCourses.begin(), h7.target.evaluationCourses.end());
            const TrainingTask3D task = loadTrainingTask3D(s.taskPath);
            for (const TrainingTask3D::Target::Course& course : courses) {
                const std::size_t n = course.points.size();
                const auto at = [&](std::size_t i) {
                    const float laps = static_cast<float>(i / n);
                    return Vec3 { course.points[i % n][0] + laps * course.points[n - 1][0],
                        course.points[i % n][1] + laps * course.points[n - 1][1], 0.0f };
                };
                for (std::size_t i = 0; i < 24; ++i)
                    for (std::size_t j = i + 1; j < i + 6; ++j)
                        require(planar(at(i), at(j)) > 2.0f * task.target.radius,
                            "o percurso " + course.name + " repete ponto na janela");
            }
        }
        const std::size_t next = channel(batch, "target.next1.distance");
        require(channel(batch, "target.next2.direction.y") > next, "os canais dos dois proximos");
        const float seen = batch.observations()[O + next] * 10.0f;
        require(std::abs(seen - planar(square.upcoming[0], origin)) < 0.5f, "o proximo alvo na observacao");
    }
    {
        // Chegando: o primeiro da fila vira o alvo e um novo entra no fim.
        const std::filesystem::path close = raceVariant("percurso_perto.json",
            { { "[[3.0, 0.0], [6.0, 0.0]", "[[1.2, 0.0], [6.0, 0.0]" } },
            "h5_pontos.json");
        TrainingBatchSettings3D s = settings(4, 2);
        s.taskPath = close;
        s.evaluation = true;
        s.evaluationEpisodeSeconds = 20.0f;
        TrainingBatch3D batch(s);
        const TrainingEnvironmentView3D before = batch.view(1);
        bool reached = false;
        for (int step = 0; step < 45 && !reached; ++step) {
            batch.step(std::vector<float>(4 * batch.actionSize(), 0.0f));
            const TrainingEnvironmentView3D now = batch.view(1);
            if (now.targetsReached == 0) continue;
            reached = true;
            require(planar(now.target, before.upcoming[0]) < 1e-3f && planar(now.upcoming[0], before.upcoming[1]) < 1e-3f,
                "a fila anda: o proximo vira o alvo");
            require(near(planar(now.upcoming[1], now.upcoming[0]), 3.0f, 1e-3f), "o novo entra no fim (o lado do quadrado)");
            require(now.reachedCount == 1 && planar(now.reached[0], before.target) < 1e-3f,
                "o alcancado fica guardado (o visualizador o pinta de verde)");
        }
        require(reached, "seguindo o clipe, ele chega no primeiro ponto a 1,2 m");
    }
    {
        // O tracado de corrida: a mira vai para o lado do proximo e muda o giro
        // pedido (o percurso variado, mundo 0: o proximo fica a direita do alvo).
        const auto turn = [&](const std::filesystem::path& task) {
            TrainingBatchSettings3D s = settings(4, 2);
            s.taskPath = task;
            s.evaluation = true;
            s.evaluationEpisodeSeconds = 20.0f;
            TrainingBatch3D batch(s);
            return batch.observations()[channel(batch, "target.turn_rate")] * 2.0f;
        };
        const float center = turn(raceVariant("tracado_centro.json", { { "\"racingLine\": 0.6", "\"racingLine\": 0.0" } },
            "h5_pontos.json"));
        const float line = turn(raceVariant("tracado_borda.json", { { "\"racingLine\": 0.6", "\"racingLine\": 0.6" } },
            "h5_pontos.json"));
        require(std::abs(line - center) > 0.05f, "o tracado de corrida muda o giro pedido (" + std::to_string(center)
            + " -> " + std::to_string(line) + ")");
    }
    {
        // No treino, cada alvo sai do anterior, entre 2 e 8 m.
        TrainingBatchSettings3D s = settings(8, 2);
        s.taskPath = std::filesystem::path(MATTERENGINE_TEST_ASSETS_DIR) / "rl/tasks/h5_pontos.json";
        TrainingBatch3D batch(s);
        for (std::size_t e = 0; e < 8; ++e) {
            const TrainingEnvironmentView3D view = batch.view(e);
            const float a = planar(view.upcoming[0], view.target), b = planar(view.upcoming[1], view.upcoming[0]);
            require(a >= 2.0f - 1e-3f && a <= 8.0f + 1e-3f && b >= 2.0f - 1e-3f && b <= 8.0f + 1e-3f,
                "no treino os proximos ficam a 2-8 m do anterior");
        }
    }
}

// O peito para a camera (H6): o comando de olhar vale no peito, numa fracao
// dos episodios de treino; a bateria escolhe com ou sem ele.
void testChestFacing() {
    const std::filesystem::path task = std::filesystem::path(MATTERENGINE_TEST_ASSETS_DIR) / "rl/tasks/h6_peito.json";
    const auto evaluation = [&](int facing) {
        TrainingBatchSettings3D s = settings(4, 2);
        s.taskPath = task;
        s.evaluation = true;
        s.evaluationEpisodeSeconds = 20.0f;
        s.facing = facing;
        return s;
    };
    TrainingBatch3D on(evaluation(1)), off(evaluation(0)), standard(evaluation(-1));
    const std::size_t O = on.observationSize(), active = channel(on, "command.facing.active");
    for (std::size_t e = 0; e < 4; ++e) {
        require(on.observations()[e * O + active] == 1.0f && standard.observations()[e * O + active] == 1.0f,
            "na avaliacao o comando vale (pedido ou o da tarefa)");
        require(off.observations()[e * O + active] == 0.0f, "a bateria livre desliga o comando");
        require(on.view(e).hasFacing && !off.view(e).hasFacing, "o olho no horizonte so com o comando");
    }
    // O percurso variado (mundo 0): o primeiro alvo fica a -56 graus. Livre, o
    // giro alinha o clipe com ele; com o comando, o peito fica para a frente e
    // so as pernas levam.
    const std::size_t turn = channel(on, "target.turn_rate");
    const float free = off.observations()[turn] * 2.0f, held = on.observations()[turn] * 2.0f;
    require(std::abs(held) < 0.1f && std::abs(free - held) > 0.1f,
        "com o comando o peito nao vira para o alvo (livre " + std::to_string(free) + ", com ele " + std::to_string(held) + ")");
    // Segurar o peito paga so com o comando; o episodio guarda o erro dele.
    {
        const std::size_t R = TrainingRewardComponentNames3D.size(), hold = rewardIndex("facingHold");
        float paidOn = 0.0f, paidOff = 0.0f;
        for (int step = 0; step < 10; ++step) {
            on.step(std::vector<float>(4 * on.actionSize(), 0.0f));
            off.step(std::vector<float>(4 * off.actionSize(), 0.0f));
            for (std::size_t e = 0; e < 4; ++e) {
                paidOn += on.rewardComponents()[e * R + hold];
                paidOff += off.rewardComponents()[e * R + hold];
            }
            for (const TrainingEpisode3D& episode : on.finishedEpisodes())
                require(episode.facingErrorRms >= 0.0f, "com o comando o episodio mede o peito");
            for (const TrainingEpisode3D& episode : off.finishedEpisodes())
                require(episode.facingErrorRms < 0.0f, "livre o episodio nao mede o peito");
        }
        require(paidOn > 0.0f && paidOff == 0.0f, "segurar o peito paga so com o comando");
    }
    // No treino, ~70% dos episodios com o comando.
    TrainingBatchSettings3D s = settings(64, 4);
    s.taskPath = task;
    TrainingBatch3D training(s);
    std::size_t count = 0;
    for (std::size_t e = 0; e < 64; ++e) count += training.observations()[e * training.observationSize() + active] == 1.0f;
    require(count >= 32 && count <= 58, "facingFraction 0,7 no treino (" + std::to_string(count) + " de 64)");
    for (int step = 0; step < 10; ++step) {
        training.step(std::vector<float>(64 * training.actionSize(), 0.0f));
        require(finiteAll(training.observations()) && finiteAll(training.rewards()), "o peito anda sem numero ruim");
    }
}

// A velocidade no planejador: com o sprint na biblioteca e a velocidade pedida
// pela distancia, longe ele escolhe o sprint e perto o trote.
void testPlannerSpeed() {
    const std::string clips = (std::filesystem::path(MATTERENGINE_TEST_ASSETS_DIR) / "animations/clips").string();
    const auto clipAt = [&](const std::string& distance) {
        const std::filesystem::path task = raceVariant("velocidade_" + distance + ".json",
            { { "trot_forward_right.matteranim.json\"",
                  "trot_forward_right.matteranim.json\" }, { \"id\": \"sprint\", \"clip\": \"" + clips
                      + "/mixamo_sprint.matteranim.json\"" },
              { "\"hysteresisDegrees\": 10.0,",
                  "\"hysteresisDegrees\": 10.0, \"speedByDistance\": [[2.0, 2.5], [10.0, 6.0]], \"speedWeight\": 1.0," },
              { "\"evaluationDistances\": [\n      4.0\n    ]", "\"evaluationDistances\": [" + distance + "]" } },
            "h4_direcoes.json");
        TrainingBatchSettings3D s = settings(1, 1);
        s.taskPath = task;
        s.evaluation = true;
        s.evaluationEpisodeSeconds = 15.0f;
        TrainingBatch3D batch(s);
        return std::pair { batch.observations()[channel(batch, "reference.clip.sprint")],
            batch.observations()[channel(batch, "reference.clip.frente")] };
    };
    const auto [farSprint, farTrot] = clipAt("15.0");
    require(farSprint == 1.0f && farTrot == 0.0f, "o alvo a 15 m a frente pede o sprint");
    const auto [nearSprint, nearTrot] = clipAt("2.0");
    require(nearSprint == 0.0f && nearTrot == 1.0f, "o alvo a 2 m a frente pede o trote");
}

// O tiro com volta (H7): no treino cada alvo seguinte fica de 12 a 24 m do
// anterior e volta (180 graus, ate 20 para cada lado).
void testShuttle() {
    TrainingBatchSettings3D s = settings(16, 4);
    s.taskPath = std::filesystem::path(MATTERENGINE_TEST_ASSETS_DIR) / "rl/tasks/h7_tiro.json";
    TrainingBatch3D batch(s);
    for (std::size_t e = 0; e < 16; ++e) {
        const TrainingEnvironmentView3D view = batch.view(e);
        const Vec3 a = view.upcoming[0] - view.target, b = view.upcoming[1] - view.upcoming[0];
        const float la = std::hypot(a.x, a.y), lb = std::hypot(b.x, b.y);
        require(la >= 12.0f - 1e-3f && la <= 24.0f + 1e-3f && lb >= 12.0f - 1e-3f && lb <= 24.0f + 1e-3f,
            "os alvos do tiro ficam de 12 a 24 m");
        require((a.x * b.x + a.y * b.y) / (la * lb) <= std::cos(160.0f * std::numbers::pi_v<float> / 180.0f) + 1e-3f,
            "cada alvo do tiro volta (160 a 200 graus)");
    }
    for (int step = 0; step < 10; ++step) {
        batch.step(std::vector<float>(16 * batch.actionSize(), 0.0f));
        require(finiteAll(batch.observations()) && finiteAll(batch.rewards()), "o tiro anda sem numero ruim");
    }
}

// Os exercicios (a metodologia das 3 areas): cada mundo sorteia o seu pelo
// peso e usa o que ele muda; a bateria pode fixar um; um exercicio nao muda o
// que a rede ve.
void testDrills() {
    const std::string drills = "\"drills\": ["
        "{ \"id\": \"perto\", \"weight\": 3.0, \"overrides\": { \"target\": { \"distanceRange\": [2.0, 4.0] } } },"
        "{ \"id\": \"longe\", \"weight\": 1.0, \"overrides\": { \"target\": { \"distanceRange\": [10.0, 14.0] },"
        " \"episodeSeconds\": 30.0 } } ], \"exploration\": {";
    const std::filesystem::path task = raceVariant("exercicios.json", { { "\"exploration\": {", drills } }, "h5_pontos.json");
    {
        TrainingBatchSettings3D s = settings(64, 4);
        s.taskPath = task;
        TrainingBatch3D batch(s);
        require(batch.task().drills.size() == 2 && batch.task().drills[1].task->episodeSeconds == 30.0f,
            "os exercicios carregam o que mudam");
        std::size_t near = 0;
        for (std::size_t e = 0; e < 64; ++e) {
            const TrainingEnvironmentView3D view = batch.view(e);
            const Vec3 a = view.upcoming[0] - view.target;
            const float length = std::hypot(a.x, a.y);
            if (view.drill == 0) {
                ++near;
                require(length >= 2.0f - 1e-3f && length <= 4.0f + 1e-3f, "o exercicio perto poe os alvos a 2-4 m");
            } else {
                require(length >= 10.0f - 1e-3f && length <= 14.0f + 1e-3f, "o exercicio longe poe os alvos a 10-14 m");
            }
        }
        require(near >= 38 && near <= 60, "os pesos 3:1 (" + std::to_string(near) + " de 64 perto)");
        for (int step = 0; step < 5; ++step) {
            batch.step(std::vector<float>(64 * batch.actionSize(), 0.0f));
            require(finiteAll(batch.observations()) && finiteAll(batch.rewards()), "os exercicios andam sem numero ruim");
        }
    }
    {
        TrainingBatchSettings3D s = settings(4, 2);
        s.taskPath = task;
        s.drill = "longe";
        TrainingBatch3D batch(s);
        for (std::size_t e = 0; e < 4; ++e) require(batch.view(e).drill == 1, "a bateria fixa o exercicio");
    }
    {
        const std::filesystem::path bad = raceVariant("exercicio_ruim.json",
            { { "\"exploration\": {", "\"drills\": [ { \"id\": \"x\", \"overrides\": { \"target\": { \"lookahead\": 1 } } } ],"
                " \"exploration\": {" } }, "h5_pontos.json");
        bool refused = false;
        try { (void)loadTrainingTask3D(bad); } catch (const std::exception&) { refused = true; }
        require(refused, "um exercicio nao muda o que a rede ve (lookahead)");
    }
    {
        // A tarefa Corpo (a area 1): a estrela, com o desenho e o comando de
        // velocidade dela.
        const TrainingTask3D corpo = loadTrainingTask3D(std::filesystem::path(MATTERENGINE_TEST_ASSETS_DIR) / "rl/tasks/corpo.json");
        require(corpo.drills.size() == 7 && corpo.drills[0].id == "estrela"
                && corpo.drills[0].task->target.pattern == "star" && corpo.target.pattern == "random"
                && corpo.drills[0].task->target.facingFraction == 0.6f && corpo.target.speedCommand.enabled
                && corpo.target.speedCommand.bands.size() == 4,
            "o Corpo carrega a estrela com o que ela muda");
        const auto& brake = corpo.drills[1].task->target.speedCommand;
        require(corpo.drills[1].id == "frear" && brake.changesPerLeg == 2 && brake.stopFraction == 0.5f
                && brake.bands.size() == 4 && corpo.drills[1].task->target.pattern == "star"
                && corpo.target.speedCommand.stopFraction == 0.0f,
            "o frear e a estrela com a pedida mudando e os discos de parar");
    }
}

// A estrela (o primeiro exercicio do Corpo): do centro a uma das 8 direcoes e de
// volta, na velocidade pedida; o planejador escolhe o passo pela velocidade e,
// com o peito fixo, a direcao pelo peito e a pedida nao passa do que os clipes
// fazem para aquele lado.
void testStar() {
    const std::filesystem::path task = std::filesystem::path(MATTERENGINE_TEST_ASSETS_DIR) / "rl/tasks/corpo.json";
    const TrainingTask3D corpo = loadTrainingTask3D(task);
    const auto& bands = corpo.target.speedCommand.bands;
    const auto planar = [](Vec3 a, Vec3 b) { return std::hypot(a.x - b.x, a.y - b.y); };
    {
        // No treino: o de agora fora, o proximo no centro, o seguinte fora de novo.
        TrainingBatchSettings3D s = settings(32, 4);
        s.taskPath = task;
        s.drill = "estrela";
        TrainingBatch3D batch(s);
        const std::size_t O = batch.observationSize(), speed = channel(batch, "command.speed"),
                          now = channel(batch, "command.speed.now");
        std::uint32_t continuous = 0, hidden = 0;
        for (std::size_t e = 0; e < 32; ++e) {
            const TrainingEnvironmentView3D view = batch.view(e);
            const float out = planar(view.target, view.origin);
            require(out >= bands.front().distance.minimum - 0.5f && out <= bands.back().distance.maximum + 0.5f,
                "o ponto de fora fica na distancia das faixas (" + std::to_string(out) + " m)");
            // Depois da ponta, o centro ou (a estrela continua) outra ponta; com
            // os proximos escondidos (a reacao), a vista nao os mostra.
            if (view.upcomingCount > 0) {
                const float next = planar(view.upcoming[0], view.origin);
                require(next < 1e-3f || (next >= bands.front().distance.minimum - 0.5f
                                            && planar(view.upcoming[0], view.target) > 1.0f),
                    "depois da ponta vem o centro ou outra ponta");
                continuous += next > 1e-3f ? 1u : 0u;
            } else {
                ++hidden;
            }
            const float wanted = batch.observations()[e * O + speed] * 6.0f, current = batch.observations()[e * O + now] * 6.0f;
            require(wanted >= bands.front().speed.minimum - 1e-3f && wanted <= bands.back().speed.maximum + 1e-3f,
                "a velocidade pedida vem das faixas");
            require(current <= wanted + 1e-3f, "a pedida de agora nunca passa da do trecho");
            require(view.hasSpeedCommand && view.layoutCount == 9, "a vista tem a velocidade e os 9 pontos da estrela");
            // As pontas desenhadas ficam na distancia da ida de agora (nao na
            // do ponto sorteado mais a frente).
            require(std::abs(planar(view.layout[1], view.layout[0]) - out) < 1e-3f,
                "a estrela desenhada tem o tamanho da ida de agora");
        }
        require(continuous > 2 && hidden > 2, "a estrela continua e a reacao aparecem no treino ("
            + std::to_string(continuous) + " continuas, " + std::to_string(hidden) + " escondidas)");
        for (int step = 0; step < 20; ++step) {
            batch.step(std::vector<float>(32 * batch.actionSize(), 0.0f));
            require(finiteAll(batch.observations()) && finiteAll(batch.rewards()), "a estrela anda sem numero ruim");
            for (const TrainingEpisode3D& episode : batch.finishedEpisodes()) {
                require(episode.speedErrorRms >= 0.0f, "o episodio mede o erro de velocidade");
                require(episode.footSlipMean >= 0.0f || episode.steps < 5, "e a sola escorregando");
            }
        }
    }
    const auto evaluation = [&](int facing) {
        TrainingBatchSettings3D s = settings(8, 4);
        s.taskPath = task;
        s.evaluation = true;
        s.evaluationEpisodeSeconds = 40.0f;
        s.facing = facing;
        return s;
    };
    {
        // Na avaliacao, o mundo 0 anda (a faixa 0) para a frente: o andar.
        TrainingBatch3D free(evaluation(0));
        require(free.observations()[channel(free, "reference.clip.andar_frente")] == 1.0f,
            "andando para a frente, o clipe e o andar");
        // O mundo 7 pede o sprint (a faixa 3) para tras e para o lado (a direcao
        // 5 de 8): a pedida continua a do sprint, mas ele vira andando antes de
        // acelerar (o sprint de costas para o alvo o levava para longe dele).
        const std::size_t O = free.observationSize();
        const float* behind = free.observations().data() + 7 * O;
        require(behind[channel(free, "command.speed")] * 6.0f > 5.4f, "a pedida para tras e a do sprint");
        require(behind[channel(free, "reference.clip.sprint")] == 0.0f
                && behind[channel(free, "reference.clip.corrida_frente")] == 0.0f
                && behind[channel(free, "reference.clip.frente")] == 0.0f,
            "de costas para o alvo, ele vira num passo lento antes do sprint");
    }
    {
        // Com o peito fixo, o mundo 4 anda para tras (a direcao 4 de 8): o andar
        // de costas. O mundo 6 corre (a faixa 2) para o lado: a pedida nao passa
        // do sprint de lado.
        TrainingBatch3D fixed(evaluation(1));
        const std::size_t O = fixed.observationSize();
        require(fixed.observations()[4 * O + channel(fixed, "reference.clip.andar_costas")] == 1.0f,
            "com o peito fixo e o alvo atras, o andar de costas");
        const float sideways = fixed.observations()[6 * O + channel(fixed, "command.speed")] * 6.0f;
        const float run = 0.5f * (bands[2].speed.minimum + bands[2].speed.maximum);
        require(sideways < run - 0.1f, "de lado a pedida fica no que o sprint de lado faz (" + std::to_string(sideways) + " m/s)");
    }
    {
        // O peito girando (facing 2): na avaliacao o rumo pedido gira, em
        // velocidade e sentido sorteados por mundo; com o fixo ele fica.
        TrainingBatch3D fixed(evaluation(1)), turning(evaluation(2));
        std::vector<float> start(8), held(8);
        for (std::size_t e = 0; e < 8; ++e) {
            start[e] = turning.view(e).facingCommand;
            held[e] = fixed.view(e).facingCommand;
        }
        // (Sem politica eles caem em meio segundo: 1/3 s, e o mundo que
        // recomecou fica de fora.)
        std::vector<bool> restarted(8, false);
        for (int step = 0; step < 10; ++step) {
            fixed.step(std::vector<float>(8 * fixed.actionSize(), 0.0f));
            turning.step(std::vector<float>(8 * turning.actionSize(), 0.0f));
            for (const TrainingEpisode3D& episode : fixed.finishedEpisodes()) restarted[episode.environment] = true;
            for (const TrainingEpisode3D& episode : turning.finishedEpisodes()) restarted[episode.environment] = true;
        }
        float moved = 0.0f, still = 0.0f;
        bool clockwise = false, counter = false;
        for (std::size_t e = 0; e < 8; ++e) {
            if (restarted[e]) continue;
            const float turned = std::remainder(turning.view(e).facingCommand - start[e], 2.0f * std::numbers::pi_v<float>);
            moved = std::max(moved, std::abs(turned));
            clockwise = clockwise || turned < -0.005f;
            counter = counter || turned > 0.005f;
            still = std::max(still, std::abs(fixed.view(e).facingCommand - held[e]));
        }
        require(std::count(restarted.begin(), restarted.end(), false) >= 4, "a maioria ainda de pe");
        require(moved > 0.03f && still < 1e-4f, "girando o rumo do peito gira; fixo ele fica (" + std::to_string(moved) + ", " + std::to_string(still) + ")");
        require(clockwise && counter, "o giro sorteia os dois sentidos");
    }
    {
        // O visualizador (um mundo, evaluationCycle): cada recomeco segue o
        // roteiro do mundo seguinte da bateria, outra direcao e outra faixa.
        TrainingBatchSettings3D s = evaluation(0);
        s.environmentCount = 1;
        s.environmentsPerScene = 1;
        s.evaluationCycle = true;
        TrainingBatch3D viewer(s);
        const TrainingEnvironmentView3D first = viewer.view(0);
        viewer.reset(7);
        const TrainingEnvironmentView3D second = viewer.view(0);
        const float a = std::atan2(first.target.y - first.origin.y, first.target.x - first.origin.x);
        const float b = std::atan2(second.target.y - second.origin.y, second.target.x - second.origin.x);
        require(std::abs(std::atan2(std::sin(a - b), std::cos(a - b))) > 0.5f, "o recomeco vai para outra direcao");
        require(std::abs(planar(second.target, second.origin) - planar(first.target, first.origin)) > 1.0f,
            "e em outra faixa (outra distancia)");
    }
}

// O frear (o 4): a estrela com a pedida mudando no caminho e os discos de
// "pare aqui" (na avaliacao, um sim e um nao, cada mundo de um lado).
void testBrake() {
    const std::filesystem::path task = std::filesystem::path(MATTERENGINE_TEST_ASSETS_DIR) / "rl/tasks/corpo.json";
    {
        TrainingBatchSettings3D s = settings(8, 4);
        s.taskPath = task;
        s.evaluation = true;
        s.evaluationEpisodeSeconds = 40.0f;
        s.facing = 0;
        s.drill = "frear";
        TrainingBatch3D batch(s);
        const TrainingEnvironmentView3D even = batch.view(0), odd = batch.view(1);
        require(!even.targetStop && even.upcomingStop[0] && !even.upcomingStop[1], "mundo par: o segundo alvo e de parar");
        require(odd.targetStop && !odd.upcomingStop[0], "mundo impar: o primeiro e de parar");
        const std::size_t O = batch.observationSize(), now = channel(batch, "command.speed.now");
        require(batch.observations()[O + now] > 0.1f, "longe do disco de parar ele ainda corre");
        for (int step = 0; step < 20; ++step) {
            batch.step(std::vector<float>(8 * batch.actionSize(), 0.0f));
            require(finiteAll(batch.observations()) && finiteAll(batch.rewards()), "o frear anda sem numero ruim");
        }
    }
    {
        TrainingBatchSettings3D s = settings(32, 4);
        s.taskPath = task;
        s.drill = "frear";
        TrainingBatch3D batch(s);
        std::uint32_t stops = 0;
        for (std::size_t e = 0; e < 32; ++e) stops += batch.view(e).targetStop ? 1u : 0u;
        require(stops > 4 && stops < 28, "no treino, parte dos discos e de parar (" + std::to_string(stops) + " de 32)");
    }
}

// As curvas (o 5): alvos a cada ~2 m num arco, a velocidade limitada pelo raio.
void testCurves() {
    const std::filesystem::path task = std::filesystem::path(MATTERENGINE_TEST_ASSETS_DIR) / "rl/tasks/corpo.json";
    const auto planar = [](Vec3 a, Vec3 b) { return std::hypot(a.x - b.x, a.y - b.y); };
    TrainingBatchSettings3D s = settings(8, 4);
    s.taskPath = task;
    s.evaluation = true;
    s.evaluationEpisodeSeconds = 40.0f;
    s.facing = 0;
    s.drill = "curvas";
    TrainingBatch3D batch(s);
    const std::size_t O = batch.observationSize(), speed = channel(batch, "command.speed");
    for (std::size_t e = 0; e < 8; ++e) {
        const TrainingEnvironmentView3D view = batch.view(e);
        const Vec3 a { view.origin.x, view.origin.y, 0.0f }, b = view.target, c = view.upcoming[0], d = view.upcoming[1];
        require(planar(a, b) > 1.4f && planar(a, b) < 2.2f && planar(b, c) > 1.4f && planar(b, c) < 2.2f,
            "os alvos da curva ficam a cada ~2 m (" + std::to_string(planar(a, b)) + ")");
        // A curva: o giro de um trecho para o outro tem o mesmo sentido.
        const float turn1 = std::remainder(std::atan2(c.y - b.y, c.x - b.x) - std::atan2(b.y - a.y, b.x - a.x), 6.2831853f);
        const float turn2 = std::remainder(std::atan2(d.y - c.y, d.x - c.x) - std::atan2(c.y - b.y, c.x - b.x), 6.2831853f);
        require(std::abs(turn1) > 0.05f && turn1 * turn2 > 0.0f, "os alvos seguem um arco");
    }
    // O mundo 2 comeca na curva fechada (~3 m) com a corrida pedida: a
    // pedida fica no que o raio deixa (sqrt(5 x 3) ~ 3,9 m/s).
    require(batch.observations()[2 * O + speed] * 6.0f < 4.0f, "na curva fechada a pedida e limitada pelo raio");
    for (int step = 0; step < 20; ++step) {
        batch.step(std::vector<float>(8 * batch.actionSize(), 0.0f));
        require(finiteAll(batch.observations()) && finiteAll(batch.rewards()), "as curvas andam sem numero ruim");
        for (const TrainingEpisode3D& episode : batch.finishedEpisodes()) {
            require(episode.lineErrorRms >= 0.0f || episode.steps < 2, "o episodio mede o afastamento da linha");
            // Com o peito livre, o peito olhando para onde vai entra no erro.
            require(episode.facingErrorRms >= 0.0f || episode.steps < 2, "o peito livre mede o olhar para o caminho");
        }
    }
}

// Os parados (o 1 e o 2): o alvo e o lugar do comeco, a pedida zero e o clipe
// parado; girando, o rumo do peito gira; nos empurroes, eles ligados.
void testStanding() {
    const std::filesystem::path task = std::filesystem::path(MATTERENGINE_TEST_ASSETS_DIR) / "rl/tasks/corpo.json";
    const TrainingTask3D corpo = loadTrainingTask3D(task);
    const auto drill = [&](const std::string& id) {
        for (const auto& d : corpo.drills)
            if (d.id == id) return d.task;
        throw std::runtime_error("sem o exercicio " + id);
    };
    require(drill("empurroes")->pushes.enabled && !drill("parado")->pushes.enabled && !corpo.pushes.enabled,
        "so os empurroes empurram");
    require(drill("girar")->target.facingDriftFraction == 1.0f && drill("parado")->target.facingDriftFraction == 0.0f,
        "o girar gira o peito, o parado nao");
    for (const char* id : { "parado", "girar", "empurroes" }) {
        TrainingBatchSettings3D s = settings(4, 2);
        s.taskPath = task;
        s.evaluation = true;
        s.evaluationEpisodeSeconds = 30.0f;
        s.facing = std::string(id) == "girar" ? 2 : 1;
        s.drill = id;
        TrainingBatch3D batch(s);
        const std::size_t O = batch.observationSize(), now = channel(batch, "command.speed.now"),
                          idle = channel(batch, "reference.clip.parado");
        for (std::size_t e = 0; e < 4; ++e) {
            const TrainingEnvironmentView3D view = batch.view(e);
            require(std::hypot(view.target.x - view.origin.x, view.target.y - view.origin.y) < 1e-3f && view.targetStop,
                std::string(id) + ": o alvo e o lugar do comeco, de parar");
            require(batch.observations()[e * O + now] == 0.0f && batch.observations()[e * O + idle] == 1.0f,
                std::string(id) + ": a pedida zero e o clipe parado");
        }
        for (int step = 0; step < 10; ++step) {
            batch.step(std::vector<float>(4 * batch.actionSize(), 0.0f));
            require(finiteAll(batch.observations()) && finiteAll(batch.rewards()), std::string(id) + " sem numero ruim");
        }
    }
}

// Os giros no lugar em ciclo (turnCycleDegrees): o giro medio de cada um no
// lado e no ritmo do clipe, e a emenda do fim no comeco sem o pulo de uma
// volta no giro da referencia.
void testTurnCycles() {
    const std::filesystem::path assets = MATTERENGINE_TEST_ASSETS_DIR;
    const TrainingTask3D corpo = loadTrainingTask3D(assets / "rl/tasks/corpo.json");
    const RLBody3D body = loadRLBody3D(corpo.bodyPath);
    const std::vector<ReferenceClip3D> clips {
        { "e90", assets / "animations/clips/turn_left.matteranim.json", true },
        { "d90", assets / "animations/clips/turn_right.matteranim.json", true },
        { "e180", assets / "animations/clips/giro_esquerda_180.matteranim.json", true },
        { "d180", assets / "animations/clips/giro_direita_180.matteranim.json", true } };
    const ReferenceMotion3D motion = buildReferenceMotion3D(body, clips, 1.0f / 120.0f, true);
    const std::array<float, 4> expected { 1.5708f / 1.0f, -1.5708f / 1.0f, 3.1416f / 2.0667f, -3.1416f / 2.0667f };
    for (std::size_t i = 0; i < 4; ++i) {
        const ReferenceSegment3D& segment = motion.segments[i];
        require(std::abs(segment.meanSpin - expected[i]) < 0.05f && segment.meanSpeed < 0.3f,
            segment.id + ": o giro medio do clipe (" + std::to_string(segment.meanSpin) + " rad/s)");
        float worst = 0.0f;
        for (const ReferenceFrame3D& frame : segment.frames) worst = std::max(worst, std::abs(frame.rootSpin.z));
        require(worst < 8.0f, segment.id + ": sem o pulo de uma volta na emenda (" + std::to_string(worst) + " rad/s)");
    }
}

// Os pulos (o 8, a secao 8 do plano): na ida do centro a ponta a zona no meio
// do caminho (a barreira na subida pedida da pelve); a volta ao centro e o
// pulo parado (o disco de parar); a velocidade so de trote ou corrida; o
// comando de pulo na observacao de todos os exercicios, ligado so aqui.
void testJumps() {
    const std::filesystem::path task = std::filesystem::path(MATTERENGINE_TEST_ASSETS_DIR) / "rl/tasks/corpo.json";
    const TrainingTask3D corpo = loadTrainingTask3D(task);
    float standing = 0.0f;  // a pelve de pe na postura dos mundos (o alerta)
    for (const RLPosture3D& posture : loadRLBody3D(corpo.bodyPath).postures)
        if (posture.id == "alerta") standing = posture.standingPelvisHeight;
    const auto planar = [](Vec3 a, Vec3 b) { return std::hypot(a.x - b.x, a.y - b.y); };
    TrainingBatchSettings3D s = settings(8, 4);
    s.taskPath = task;
    s.evaluation = true;
    s.evaluationEpisodeSeconds = 40.0f;
    s.facing = 0;
    s.drill = "pulos";
    TrainingBatch3D batch(s);
    const std::size_t O = batch.observationSize(), active = channel(batch, "jump.active"),
                      stand = channel(batch, "jump.standing"), rise = channel(batch, "jump.rise"),
                      speed = channel(batch, "command.speed");
    for (std::size_t e = 0; e < 8; ++e) {
        const TrainingEnvironmentView3D view = batch.view(e);
        const Vec3 middle { 0.5f * (view.origin.x + view.target.x), 0.5f * (view.origin.y + view.target.y), 0.0f };
        require(view.hasJump && !view.jumpStanding && planar(view.jumpCenter, middle) < 1e-3f,
            "a ida do centro a ponta tem a zona no meio do caminho");
        const float wanted = batch.observations()[e * O + rise];
        require(wanted >= 0.2f && wanted <= 0.5f && std::abs(view.jumpBarrier - (standing + wanted)) < 1e-3f,
            "a barreira fica na pelve de pe mais a subida pedida (" + std::to_string(wanted) + " m)");
        require(batch.observations()[e * O + active] == 1.0f && batch.observations()[e * O + stand] == 0.0f,
            "a rede ve a zona correndo");
        require(planar(view.upcoming[0], view.origin) < 1e-3f && view.upcomingStop[0], "a volta ao centro e o pulo parado");
        const float commanded = batch.observations()[e * O + speed] * 6.0f;
        require(commanded >= 2.0f - 1e-3f && commanded <= 5.0f + 1e-3f, "so trote ou corrida");
    }
    TrainingBatchSettings3D other = s;
    other.drill = "estrela";
    TrainingBatch3D star(other);
    require(star.observationSize() == O && star.observations()[channel(star, "jump.active")] == 0.0f && !star.view(0).hasJump,
        "a estrela ve o comando de pulo, desligado");
    for (int step = 0; step < 20; ++step) {
        batch.step(std::vector<float>(8 * batch.actionSize(), 0.0f));
        require(finiteAll(batch.observations()) && finiteAll(batch.rewards()), "os pulos andam sem numero ruim");
    }
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
    testCourses();
    testChestFacing();
    testPlannerSpeed();
    testShuttle();
    testDrills();
    testStar();
    testBrake();
    testCurves();
    testStanding();
    testTurnCycles();
    testJumps();
    std::cout << "RL training batch tests passed\n";
}
