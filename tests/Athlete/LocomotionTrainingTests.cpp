// O treino da locomocao (a etapa B do plano do RL): a nota, a postura
// funcional, a tarefa e o lote - curto, com a acao zero (sem rede).
#include "Engine/Athlete/Training/LocomotionTraining3D.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <cstdio>
#include <stdexcept>
#include <string>
#include <vector>

using namespace MatterEngine;

namespace {

int checks = 0;

void expect(bool condition, const std::string& what) {
    ++checks;
    if (!condition) throw std::runtime_error(what);
}

const std::string Assets = MATTERENGINE_ASSETS_DIR;

void rewardTests() {
    const LocomotionRewardWeights3D weights;
    LocomotionStepMeasure3D perfect;
    perfect.styleValid = true;
    const auto best = locomotionRewardTerms3D(perfect, weights, 1.0f / 30.0f);
    expect(std::abs(locomotionReward3D(best) - (0.40f + 0.10f + 0.20f) / 30.0f) < 1e-6f,
        "sem erro e sem custo: a velocidade, o rumo e o estilo inteiros, por segundo");
    LocomotionStepMeasure3D fell = perfect;
    fell.fell = true;
    expect(std::abs(locomotionReward3D(locomotionRewardTerms3D(fell, weights, 1.0f / 30.0f)) - (locomotionReward3D(best) - 1.0f))
            < 1e-6f,
        "a queda tira 1 de uma vez");
    {
        // A quietude: o tremor vezes a quietude pedida, por segundo; peso 0 (o padrao): nada.
        LocomotionStepMeasure3D shaking = perfect;
        shaking.still = 0.8f;
        expect(std::abs(locomotionReward3D(locomotionRewardTerms3D(shaking, weights, 1.0f / 30.0f)) - locomotionReward3D(best)) < 1e-9f,
            "o peso da quietude em 0 (o padrao): o tremor nao custa");
        LocomotionRewardWeights3D withStill = weights;
        withStill.still = 0.25f;
        expect(std::abs(locomotionReward3D(locomotionRewardTerms3D(shaking, withStill, 1.0f / 30.0f))
                   - (locomotionReward3D(best) - 0.25f * 0.8f / 30.0f)) < 1e-7f,
            "a quietude custa peso x tremor x quietude x duracao");
    }
    LocomotionStepMeasure3D touchdown = perfect;
    touchdown.contactScore = 2.0f;
    expect(std::abs(locomotionRewardTerms3D(touchdown, weights, 1.0f / 30.0f)[static_cast<std::size_t>(
                        LocomotionRewardTerm3D::Contact)] - 0.2f) < 1e-6f,
        "as metas de contato contam por pouso, nao por segundo");
    LocomotionStepMeasure3D costly = perfect;
    costly.slip = 0.4f;
    costly.power = 30.0f;
    costly.assist = 0.34f;
    expect(locomotionReward3D(locomotionRewardTerms3D(costly, weights, 1.0f / 30.0f)) < locomotionReward3D(best),
        "os custos tiram");

    LocomotionPostureInput3D upright;
    expect(locomotionPostureExcess3D(upright) == 0.0f, "em pe e reto: dentro do envelope");
    const float lean = 30.0f * 3.14159265f / 180.0f;
    LocomotionPostureInput3D forward;
    forward.chestUp = { std::sin(lean), 0.0f, std::cos(lean) };
    const float still = locomotionPostureExcess3D(forward);
    expect(std::abs(still - 15.0f * 3.14159265f / 180.0f) < 1e-3f, "parado, 30 graus para a frente: 15 graus fora");
    forward.commandAcceleration = { 5.0f, 0.0f, 0.0f };
    expect(locomotionPostureExcess3D(forward) < still - 0.2f, "arrancando, o envelope abre para a frente");
    LocomotionPostureInput3D braking = forward;
    braking.chestUp = { -std::sin(lean), 0.0f, std::cos(lean) };
    braking.commandAcceleration = {};
    braking.measuredAcceleration = { -6.0f, 0.0f, 0.0f };
    expect(locomotionPostureExcess3D(braking) < 0.15f, "freando forte, o envelope abre para tras");
    LocomotionPostureInput3D neck;
    neck.neckBend = 8.0f * 3.14159265f / 180.0f;
    expect(locomotionPostureExcess3D(neck) == 0.0f, "o pescoco dentro da folga nao custa");
    neck.neckBend = 20.0f * 3.14159265f / 180.0f;
    expect(std::abs(locomotionPostureExcess3D(neck) - 10.0f * 3.14159265f / 180.0f) < 1e-3f, "a cabeca torta custa o que passa da folga");
}

void taskTests() {
    const LocomotionTask3D task = loadLocomotionTask3D(Assets + "/rl/tasks/locomocao_f1f2.json");
    expect(task.drills.size() == 5 && task.drills[0].id == "parado" && task.drills[4].id == "curva",
        "a tarefa das fases 1 e 2: cinco exercicios");
    expect(task.standingAssistPercent == 30.0f && task.walkingAssistPercent == 30.0f && task.feetAssistPercent == 30.0f
            && task.getUpAssistPercent == 30.0f,
        "a ajuda em 30% (os quatro controles; o usuario desce depois)");
    expect(std::abs(task.caps.touchdownMeters - 0.06f) < 1e-6f && std::abs(task.caps.jointRadians - 0.10f) < 1e-6f
            && std::abs(task.caps.leanRadians - 0.20f) < 1e-6f && std::abs(task.caps.pelvisDownMeters - 0.12f) < 1e-6f,
        "os tetos do consenso da etapa A, com a inclinacao e a pelve abertas para cobrir o professor da virada");
    {
        const std::vector<std::string>& columns = locomotionEpisodeColumns3D();
        const auto has = [&](const char* name) { return std::find(columns.begin(), columns.end(), name) != columns.end(); };
        expect(has("tremor") && has("nota_quietude"), "as colunas do episodio ganharam o tremor e a nota da quietude");
        expect(task.reward.still > 0.0f && task.reward.still <= 0.31f, "a tarefa pesa a quietude (0,20 a 0,30)");
    }
    expect(task.turnMove.enabled && std::abs(task.turnMove.dropout - 0.3f) < 1e-6f && task.turnMove.amount == 1.0f,
        "a virada brusca ligada na tarefa, com 30% de dropout");
}

void batchTests() {
    LocomotionTrainingSettings3D settings;
    settings.taskPath = Assets + "/rl/tasks/locomocao_f1f2.json";
    settings.assetsDirectory = Assets;
    settings.environments = 6;
    settings.threads = 3;
    settings.seed = 7;
    settings.evaluation = true;
    settings.evaluationSeconds = 1.0f;
    LocomotionTraining3D batch(settings);
    batch.setAuthority(1.0f, 1.0f, 1.0f, 1.0f);
    const std::size_t worlds = batch.environmentCount(), obs = batch.observationSize(), actions = batch.actionSize();
    expect(obs == batch.contract().observationSize() && actions == batch.contract().actionSize(),
        "a entrada e a acao sao as do contrato v2");
    expect(batch.criticObservationSize() == obs + 5 + 7, "o critico: a entrada, o exercicio e o piloto");
    const LocomotionTraining3D::Mirror mirror = batch.mirror();
    expect(mirror.observation.source.size() == obs && mirror.critic.source.size() == batch.criticObservationSize()
            && mirror.action.source.size() == actions,
        "o espelho cobre a entrada, o critico e a acao");
    const std::vector<float> fractions = batch.actionCapFractions();
    expect(fractions.size() == actions && fractions[0] > 0.19f && fractions[0] < 0.21f,
        "o teto do pouso e 0,06 de 0,30 m");
    const std::vector<float> zero(worlds * actions, 0.0f);
    std::size_t episodes = 0;
    bool finite = true;
    for (int step = 0; step < 31; ++step) {
        batch.step(zero);
        for (float v : batch.observations()) finite &= std::isfinite(v);
        for (float v : batch.criticObservations()) finite &= std::isfinite(v);
        for (float v : batch.rewards()) finite &= std::isfinite(v);
        for (const LocomotionEpisode3D& e : batch.finishedEpisodes()) {
            ++episodes;
            expect(locomotionEpisodeRow3D(e).size() == locomotionEpisodeColumns3D().size(), "a linha do episodio e a das colunas");
            expect(e.reason == LocomotionEnd3D::TimeLimit, std::string("um segundo de pe, a acao zero: ")
                    + locomotionEndName3D(e.reason));
        }
    }
    expect(finite, "a entrada, o critico e a nota sempre finitos");
    expect(episodes == worlds, "cada mundo terminou um episodio de 1 s (30 decisoes)");
    expect(!batch.frameLines(0).empty(), "as linhas da folha de quadros");

    // O mesmo comeco e a mesma acao: a mesma nota (determinismo por mundo).
    settings.environments = 2;
    settings.threads = 2;
    LocomotionTraining3D a(settings), b(settings);
    a.setAuthority(1.0f, 1.0f, 1.0f, 1.0f);
    b.setAuthority(1.0f, 1.0f, 1.0f, 1.0f);
    std::vector<float> wiggle(2 * actions);
    bool same = true;
    for (int step = 0; step < 20; ++step) {
        for (std::size_t i = 0; i < wiggle.size(); ++i) wiggle[i] = 0.6f * std::sin(0.3f * static_cast<float>(step + i));
        a.step(wiggle);
        b.step(wiggle);
        for (std::size_t i = 0; i < 2; ++i) same &= a.rewards()[i] == b.rewards()[i];
    }
    expect(same, "dois lotes com a mesma semente e a mesma acao dao a mesma nota");
}

void turnMoveTests() {
    // A virada brusca nos episodios: o dropout sorteia por episodio, a avaliacao
    // usa sempre ligada, o modo forca e uma tarefa desligada nao sorteia nada.
    const auto write = [](const std::string& name, const std::string& turnMove) {
        std::ifstream in(Assets + "/rl/tasks/locomocao_f1f2.json");
        std::string text((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
        const std::string key = "\"turnMove\": { \"enabled\": true, \"dropout\": 0.3, \"amount\": 1.0 },";
        const std::size_t at = text.find(key);
        if (at == std::string::npos) throw std::runtime_error("a tarefa mudou: sem a linha do turnMove");
        text.replace(at, key.size(), turnMove);
        const std::filesystem::path path = std::filesystem::temp_directory_path() / name;
        std::ofstream(path) << text;
        return path;
    };
    const auto count = [](LocomotionTraining3D& batch) {
        std::size_t on = 0;
        for (std::size_t i = 0; i < batch.environmentCount(); ++i) on += batch.turnMoveActive(i) ? 1 : 0;
        return on;
    };
    LocomotionTrainingSettings3D settings;
    settings.assetsDirectory = Assets;
    settings.environments = 40;
    settings.threads = 2;
    settings.seed = 11;
    {
        settings.taskPath = write("matter_tm_drop0.json", "\"turnMove\": { \"enabled\": true, \"dropout\": 0.0 },");
        LocomotionTraining3D batch(settings);
        expect(count(batch) == 40, "dropout 0: todos os episodios com a virada");
    }
    {
        settings.taskPath = write("matter_tm_drop1.json", "\"turnMove\": { \"enabled\": true, \"dropout\": 1.0 },");
        LocomotionTraining3D batch(settings);
        expect(count(batch) == 0, "dropout 1: nenhum episodio com a virada");
    }
    {
        settings.taskPath = write("matter_tm_drop3.json", "\"turnMove\": { \"enabled\": true, \"dropout\": 0.3 },");
        LocomotionTraining3D batch(settings);
        const std::size_t on = count(batch);
        expect(on > 15 && on < 40, "dropout 0,3: a maioria liga e alguns ficam sem (o sorteio e por episodio)");
        batch.setTurnMoveMode(1);
        batch.reset(5);
        expect(count(batch) == 0, "o modo 1 desliga em todos");
        batch.setTurnMoveMode(2);
        batch.reset(5);
        expect(count(batch) == 40, "o modo 2 liga em todos");
        LocomotionTrainingSettings3D evaluation = settings;
        evaluation.evaluation = true;
        LocomotionTraining3D judged(evaluation);
        expect(count(judged) == 40, "na avaliacao a virada fica sempre ligada (o dropout e do treino)");
    }
    {
        settings.taskPath = write("matter_tm_off.json", "\"turnMove\": { \"enabled\": false },");
        LocomotionTraining3D batch(settings);
        expect(count(batch) == 0, "tarefa sem a virada: nenhum episodio com ela");
        batch.setTurnMoveMode(2);
        batch.reset(5);
        expect(count(batch) == 0, "tarefa sem a virada: nem o modo forcado a liga");
    }
    {
        // A tarefa trocada andando (a avaliacao troca o arquivo vivo): de ligada
        // para desligada, o recomeco volta ao classico; e a intensidade nova chega.
        settings.taskPath = write("matter_tm_swap_on.json", "\"turnMove\": { \"enabled\": true, \"dropout\": 0.0 },");
        LocomotionTraining3D batch(settings);
        expect(count(batch) == 40, "antes da troca: todos ligados");
        batch.updateTask(loadLocomotionTask3D(write("matter_tm_swap_off.json", "\"turnMove\": { \"enabled\": false },").string()));
        batch.reset(6);
        expect(count(batch) == 0, "tarefa trocada para sem a virada: o recomeco desliga");
    }
}

// Uma copia da tarefa f1f2 com trocas de texto (cada par: o trecho e o novo).
std::filesystem::path editedTask(const std::string& name, const std::vector<std::pair<std::string, std::string>>& edits) {
    std::ifstream in(Assets + "/rl/tasks/locomocao_f1f2.json");
    std::string text((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    for (const auto& [from, to] : edits) {
        const std::size_t at = text.find(from);
        if (at == std::string::npos) throw std::runtime_error("a tarefa mudou: sem o trecho " + from);
        text.replace(at, from.size(), to);
    }
    const std::filesystem::path path = std::filesystem::temp_directory_path() / name;
    std::ofstream(path) << text;
    return path;
}

void diagnosisTests() {
    // O diagnostico de 18/10 (vault .../pesquisas/08_plano_consenso.md): as celulas
    // de comando, a postura separada, a cauda da nota, o curriculo de velocidade,
    // os comecos guardados e a ajuda sorteada.
    constexpr float Deg = 3.14159265f / 180.0f;
    expect(locomotionCommandCell3D(0.0f, false) == 0 && locomotionCommandCell3D(22.0f * Deg, false) == 0
            && locomotionCommandCell3D(23.0f * Deg, false) == 2 && locomotionCommandCell3D(90.0f * Deg, true) == 5
            && locomotionCommandCell3D(180.0f * Deg, false) == 8 && locomotionCommandCell3D(-45.0f * Deg, false) == 14
            && locomotionCommandCell3D(7.0f * 3.14159265f, true) == 9,
        "as celulas: 8 setores de 45 graus a partir da frente, a esquerda positiva, e o modo da camera");

    LocomotionPostureInput3D neck;
    neck.neckBend = 25.0f * Deg;
    expect(std::abs(locomotionPostureParts3D(neck).neck - 15.0f * Deg) < 1e-4f && locomotionPostureParts3D(neck).trunk == 0.0f,
        "o pescoco contra o peito: o que passa da folga de 10 graus, e o tronco a parte");
    neck.neckRelative = true;
    neck.neckSlack = 5.0f * Deg;
    neck.headUpInChest = { std::sin(20.0f * Deg), 0.0f, std::cos(20.0f * Deg) };
    neck.referenceValid = true;
    neck.referenceHeadUpInChest = { std::sin(20.0f * Deg), 0.0f, std::cos(20.0f * Deg) };
    expect(locomotionPostureParts3D(neck).neck == 0.0f, "relativo: a cabeca igual a da sombra nao custa, mesmo inclinada");
    neck.referenceHeadUpInChest = { 0.0f, 0.0f, 1.0f };
    expect(std::abs(locomotionPostureParts3D(neck).neck - 15.0f * Deg) < 1e-3f, "relativo: 20 graus da sombra, 15 alem da folga");
    neck.referenceValid = false;
    expect(std::abs(locomotionPostureParts3D(neck).neck - 20.0f * Deg) < 1e-4f, "sem a referencia, o modo de sempre");

    LocomotionRewardWeights3D tail;
    LocomotionStepMeasure3D far;
    far.velocityError = 1.6f;
    const float core = locomotionRewardTerms3D(far, tail, 1.0f)[0];
    tail.velocityTail = 0.5f;
    const float withTail = locomotionRewardTerms3D(far, tail, 1.0f)[0];
    expect(core < 1e-4f && std::abs(withTail - 0.4f * (0.5f * core + 0.5f * std::exp(-1.6f / 1.5f))) < 1e-5f,
        "a cauda devolve o gradiente longe do alvo (1,6 m/s: o nucleo vale ~0)");

    // A tarefa: as chaves novas e a validacao.
    const std::string curriculum = "\"speedCurriculum\": { \"enabled\": true, \"caps\": { \"estrela_trote\": "
        "[0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5] } },";
    {
        const LocomotionTask3D task = loadLocomotionTask3D(editedTask("matter_diag_task.json",
            { { "\"episodeSeconds\": 20.0,", "\"episodeSeconds\": 20.0, " + curriculum
                  + " \"savedStarts\": { \"capacity\": 4, \"surviveSeconds\": 0.2, \"minimumSpeed\": 0.3 },"
                  + " \"assistSchedule\": { \"enabled\": true, \"zeroFraction\": 0.5, \"minimum\": 5, \"maximum\": 30 }," },
              { "\"id\": \"estrela\",", "\"id\": \"estrela\", \"savedStartFraction\": 1.0," } }).string());
        const auto index = [&](const char* id) {
            for (std::size_t d = 0; d < task.drills.size(); ++d)
                if (task.drills[d].id == id) return d;
            throw std::runtime_error(std::string("sem o exercicio ") + id);
        };
        expect(task.speedCurriculum.enabled && task.speedCurriculum.caps.size() == task.drills.size()
                && task.speedCurriculum.caps[index("estrela_trote")][7] == 0.5f
                && task.speedCurriculum.caps[index("estrela")][0] == 0.0f,
            "o curriculo: 16 tetos por exercicio, e sem tetos o exercicio nao corta");
        expect(task.savedStarts.capacity == 4 && task.drills[index("estrela")].savedStartFraction == 1.0f
                && task.drills[index("estrela_trote")].savedStartFraction == 0.0f && task.assistSchedule.enabled,
            "os comecos guardados e a ajuda sorteada na tarefa");
    }
    bool rejected = false;
    try {
        (void)loadLocomotionTask3D(editedTask("matter_diag_bad.json",
            { { "\"episodeSeconds\": 20.0,", "\"episodeSeconds\": 20.0, \"speedCurriculum\": { \"caps\": { \"estrela\": [1, 2] } }," } })
                .string());
    } catch (const std::runtime_error&) {
        rejected = true;
    }
    expect(rejected, "um exercicio com menos de 16 tetos e recusado");
    const auto refuses = [](const std::filesystem::path& path) {
        try {
            (void)loadLocomotionTask3D(path.string());
        } catch (const std::runtime_error&) {
            return true;
        }
        return false;
    };
    expect(refuses(editedTask("matter_diag_arc.json", { { "\"id\": \"curva\",", "\"id\": \"curva\", \"savedStartFraction\": 0.5," } })),
        "o comeco guardado nao vale para o arco (o piloto do arco nao segue a velocidade do instante)");
    expect(refuses(editedTask("matter_diag_caps.json", { { "\"cadenceOctaves\": 0.2,", "\"cadenceOctaves\": -0.1," } })),
        "um teto negativo e recusado");
    {
        const std::vector<std::string>& columns = locomotionEpisodeColumns3D();
        expect(std::find(columns.begin(), columns.end(), "cel07.fronteira") != columns.end()
                && std::find(columns.begin(), columns.end(), "cel15.rastreio_fronteira") != columns.end(),
            "as colunas da fronteira do curriculo");
    }

    // O curriculo corta o pedido no treino e nao na avaliacao.
    LocomotionTrainingSettings3D settings;
    settings.assetsDirectory = Assets;
    settings.environments = 8;
    settings.threads = 2;
    settings.seed = 13;
    settings.drill = "estrela_trote";
    settings.taskPath = editedTask("matter_diag_cap.json", { { "\"episodeSeconds\": 20.0,", "\"episodeSeconds\": 2.0, " + curriculum } });
    const auto askedPerDecision = [&](LocomotionTraining3D& batch) {
        const std::vector<float> zero(batch.environmentCount() * batch.actionSize(), 0.0f);
        double asked = 0.0, decisions = 0.0;
        for (int step = 0; step < 61; ++step) {
            batch.step(zero);
            for (const LocomotionEpisode3D& e : batch.finishedEpisodes())
                for (std::size_t k = 0; k < LocomotionCommandCells3D; ++k) {
                    asked += e.cellAsked[k];
                    decisions += e.cellDecisions[k];
                }
        }
        return decisions > 0.0 ? asked / decisions : -1.0;
    };
    {
        LocomotionTraining3D batch(settings);
        batch.setAuthority(1.0f, 1.0f, 1.0f, 1.0f);
        const double asked = askedPerDecision(batch);
        expect(asked > 0.3 && asked <= 0.5001, "no treino o pedido fica no teto de 0,5 m/s da celula");
    }
    {
        LocomotionTrainingSettings3D evaluation = settings;
        evaluation.evaluation = true;
        evaluation.evaluationSeconds = 2.0f;
        LocomotionTraining3D batch(evaluation);
        batch.setAuthority(1.0f, 1.0f, 1.0f, 1.0f);
        expect(askedPerDecision(batch) > 0.6, "na avaliacao a faixa inteira (sem o teto)");
    }

    // Os comecos guardados e a ajuda sorteada.
    {
        LocomotionTrainingSettings3D saved = settings;
        // (A mistura: os mundos trocam de exercicio, e o banco tem de guardar cada instante no dele.)
        saved.drill = "";
        saved.environments = 24;
        saved.taskPath = editedTask("matter_diag_saved.json",
            { { "\"episodeSeconds\": 20.0,", "\"episodeSeconds\": 1.5, \"savedStarts\": { \"capacity\": 4, \"surviveSeconds\": 0.2, \"minimumSpeed\": 0.3 },"
                  " \"assistSchedule\": { \"enabled\": true, \"zeroFraction\": 0.5, \"minimum\": 5, \"maximum\": 30 }," },
              { "\"id\": \"estrela\",", "\"id\": \"estrela\", \"savedStartFraction\": 1.0," } });
        LocomotionTraining3D batch(saved);
        batch.setAuthority(1.0f, 1.0f, 1.0f, 1.0f);
        const std::vector<float> zero(batch.environmentCount() * batch.actionSize(), 0.0f);
        std::size_t fromSaved = 0, zeroAid = 0, someAid = 0, episodes = 0;
        bool finite = true, levelsOk = true;
        for (int step = 0; step < 240; ++step) {
            batch.step(zero);
            for (float v : batch.observations()) finite &= std::isfinite(v);
            for (const LocomotionEpisode3D& e : batch.finishedEpisodes()) {
                ++episodes;
                fromSaved += e.savedStart > 0.5f ? 1 : 0;
                if (e.assistLevel == 0.0f) ++zeroAid;
                else if (e.assistLevel >= 5.0f && e.assistLevel <= 30.0f) ++someAid;
                else levelsOk = false;
            }
        }
        expect(finite, "os comecos guardados: a entrada segue finita");
        const LocomotionTask3D& task = batch.task();
        std::size_t star = 0, trot = 0;
        for (std::size_t d = 0; d < task.drills.size(); ++d) {
            if (task.drills[d].id == "estrela") star = d;
            if (task.drills[d].id == "estrela_trote") trot = d;
        }
        std::size_t others = 0;
        for (std::size_t d = 0; d < task.drills.size(); ++d)
            if (d != star) others += batch.savedStartCount(d);
        expect(others == 0, "o banco so guarda instantes no exercicio de onde vieram (nada nos outros)");
        expect(batch.savedStartCount(star) > 0 && batch.savedStartCount(star) <= 4 && batch.savedStartCount(trot) == 0,
            "o banco junta instantes do exercicio que pede (no teto da capacidade) e so dele");
        expect(fromSaved > 0, "episodios comecam de instantes guardados");
        expect(levelsOk && zeroAid > 0 && someAid > 0, "a ajuda sorteada: parte a 0%, parte entre 5 e 30%");
        expect(episodes > 20, "episodios bastantes no teste");
    }
}

} // namespace

int main() {
    try {
        rewardTests();
        taskTests();
        batchTests();
        turnMoveTests();
        diagnosisTests();
    } catch (const std::exception& error) {
        std::fprintf(stderr, "treino da locomocao: FALHOU: %s (%d checagens antes)\n", error.what(), checks);
        return 1;
    }
    std::printf("treino da locomocao: %d checagens, tudo certo\n", checks);
    return 0;
}
