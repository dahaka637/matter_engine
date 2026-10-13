#include "Engine/RL/Environment/TrainingBatch3D.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <random>
#include <stdexcept>
#include <string>
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
    for (int step = 0; step <= 10; ++step) {
        if (step > 0) batch.step(actions);
        const auto obs = batch.observations();
        for (std::size_t e = 1; e < Worlds; ++e)
            for (std::size_t i = 0; i < O; ++i)
                require(std::abs(obs[e * O + i] - obs[i]) < (step == 0 ? 1e-4f : 2e-2f),
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
    const std::filesystem::path walking = std::filesystem::path(MATTERENGINE_TEST_ASSETS_DIR) / "rl/tasks/h3_andar.json";
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
    require(standing > 4 && standing < 40, "uma fatia dos sorteios pede ficar parado");
    for (std::size_t e = 0; e < 64; ++e)
        if (batch.view(e).posture == 0) require(commandOf(e).x == 0.0f, "a postura natural fica so parada (o ensaio da H1)");
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

} // namespace

void runTrainingBatchTests() {
    testContract();
    testHeadingInvariance();
    testPostures();
    testCommands();
    testDeterminism();
    testFallsAndRestart();
    testTimeLimit();
    std::cout << "RL training batch tests passed\n";
}
