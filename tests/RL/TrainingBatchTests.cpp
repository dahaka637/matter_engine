#include "Engine/RL/Environment/TrainingBatch3D.hpp"

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
    std::uniform_real_distribution<float> unit(-1.0f, 1.0f);
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
            require(std::abs(batch.observations()[episode.environment * batch.observationSize()]) < 0.1f,
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
            require(episode.steps == 15, "0,5 s a 30 Hz sao 15 decisoes");
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
    testDeterminism();
    testFallsAndRestart();
    testTimeLimit();
    std::cout << "RL training batch tests passed\n";
}
