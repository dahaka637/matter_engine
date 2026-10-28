// A evolucao do treino: o indice (0 a 100) de cada exercicio e do treino, e as
// series dos graficos da tela, numa grade de fatias de rodadas.
#include "Engine/RL/Supervisor/TrainingEvolution3D.hpp"

#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>

using namespace MatterEngine;

namespace {

void require(bool condition, const std::string& message) {
    if (!condition) throw std::runtime_error(message);
}

bool near(double a, double b, double tolerance = 1.0e-6) { return std::abs(a - b) <= tolerance; }

TrainingDrillPoint3D point(std::uint64_t iteration, const std::string& drill, double episodes, double falls) {
    TrainingDrillPoint3D p;
    p.iteration = iteration;
    p.time = 1000.0 + static_cast<double>(iteration) * 3.0;
    p.drill = drill;
    p.episodes = episodes;
    p.falls = falls;
    p.seconds = 10.0;
    p.episodeReturn = 4.0;
    p.velocityError = 0.25;
    p.requestedSpeed = 1.0;
    p.bodySpeed = 0.9;
    p.facingError = 0.2;
    p.slip = 0.3;
    p.postureError = 0.05;
    return p;
}

// As partes pelas medidas, uma a uma.
void testParts() {
    TrainingDrillMeasure3D m;
    m.episodes = 10.0;
    m.falls = 0.2;
    m.velocityError = 0.5;
    m.requestedSpeed = 2.0;
    m.bodySpeed = 1.5;
    m.facingError = 0.0;
    m.slip = 0.0;
    m.postureError = 0.0;
    TrainingDrillInfo3D moving { "frente", 1.0, 4.5, false };
    const TrainingScoreParts3D parts = trainingScoreParts3D(m, moving);
    require(near(parts.standing, 0.8), "sem cair = 1 - quedas");
    require(near(parts.tracking, 0.75), "seguir o pedido = 1 - erro/pedido");
    require(near(parts.reach, 1.5 / 4.5), "velocidade alcancada = a do corpo sobre a meta");
    require(near(parts.stability, 1.0) && near(parts.facing, 1.0) && near(parts.footing, 1.0) && near(parts.trunk, 1.0),
        "sem erro de rumo, sem escorregar e o tronco no envelope: estabilidade cheia");
    const double expected = 100.0 * (0.40 * 0.8 + 0.25 * 0.75 + 0.20 * 1.0 + 0.15 * (1.5 / 4.5));
    require(near(parts.index(), expected), "o indice pelos pesos das partes");

    // Parado (sem meta): sem a velocidade alcancada, as outras partes crescem; o pedido com piso de 1 m/s.
    TrainingDrillInfo3D still { "parado", 1.0, 0.0, false };
    TrainingDrillMeasure3D standing = m;
    standing.requestedSpeed = 0.0;
    standing.velocityError = 0.1;
    const TrainingScoreParts3D quiet = trainingScoreParts3D(standing, still);
    require(std::isnan(quiet.reach), "parado nao tem velocidade alcancada");
    require(near(quiet.tracking, 0.9), "parado, o erro conta sobre 1 m/s");
    require(near(quiet.index(), 100.0 * (0.40 * 0.8 + 0.25 * 0.9 + 0.20 * 1.0) / 0.85), "sem uma parte, as outras crescem");

    // O levantar conta quem ficou de pe.
    TrainingDrillInfo3D getUp { "levantar", 1.0, 0.0, true };
    TrainingDrillMeasure3D rising = m;
    rising.stood = 0.3;
    require(near(trainingScoreParts3D(rising, getUp).standing, 0.3), "o levantar conta o levantou-se");

    // A estabilidade cai com o rumo, o escorregar e a postura.
    TrainingDrillMeasure3D shaky = m;
    shaky.facingError = 0.5;
    shaky.slip = 0.5;
    shaky.postureError = 0.15;
    require(near(trainingScoreParts3D(shaky, moving).stability, std::exp(-1.0)), "cada parte da estabilidade e exp(-medida/escala)");

    // Sem episodios, nada.
    TrainingDrillMeasure3D none;
    require(std::isnan(trainingScoreParts3D(none, moving).index()), "sem episodios nao ha indice");
}

// O treino: o sem cair subindo sobe o indice; os pesos da tarefa; o exercicio fora do treino nao entra.
void testCharts() {
    TrainingHistory3D history;
    for (std::uint64_t i = 1; i <= 100; ++i) {
        const double falls = i <= 50 ? 1.0 : 0.0;
        history.drills.push_back(point(i, "frente", 40.0, falls));
        history.drills.push_back(point(i, "parado", 40.0, 0.0));
        history.drills.push_back(point(i, "fora", 40.0, 1.0));
        TrainingIterationPoint3D round;
        round.iteration = i;
        round.time = 1000.0 + static_cast<double>(i) * 3.0;
        round.sps = 7000.0 + static_cast<double>(i);
        history.iterations.push_back(round);
    }
    TrainingEvaluationPoint3D test;
    test.iteration = 60;
    test.drill = "frente";
    test.solver = "rede";
    test.falls = 0.25;
    history.evaluations.push_back(test);
    test.solver = "classico";
    test.falls = 0.9;
    history.evaluations.push_back(test);
    const std::vector<TrainingDrillInfo3D> task { { "frente", 3.0, 4.5, false }, { "parado", 1.0, 0.0, false },
        { "fora", 0.0, 4.5, false } };
    const TrainingCharts3D charts = trainingCharts3D(history, task, "frente", 10);
    require(charts.times.size() == 10 && charts.evolution.size() == 10, "dez fatias");
    require(charts.iterations.front() == 10 && charts.iterations.back() == 100, "cada fatia termina na ultima rodada dela");
    require(near(charts.times.back(), 1000.0 + 300.0), "o horario da fatia e o da ultima rodada");
    require(charts.evolution.back() > charts.evolution.front() + 20.0f, "parar de cair sobe a evolucao");
    require(near(charts.drillStanding.front(), 0.0, 1e-4) && near(charts.drillStanding.back(), 100.0, 1e-4),
        "o sem cair do exercicio vai de 0 a 100");
    // Os pesos: o indice do treino e a media pelos pesos (o "fora" tem peso 0 e caia sempre).
    {
        TrainingDrillMeasure3D m;
        m.episodes = 40.0;
        m.falls = 0.0;
        m.velocityError = 0.25;
        m.requestedSpeed = 1.0;
        m.bodySpeed = 0.9;
        m.facingError = 0.2;
        m.slip = 0.3;
        m.postureError = 0.05;
        const double front = trainingScoreParts3D(m, task[0]).index();
        const double still = trainingScoreParts3D(m, task[1]).index();
        require(near(charts.evolution.back(), (3.0 * front + 1.0 * still) / 4.0, 1e-3),
            "a evolucao do treino e a media dos exercicios pelos pesos, sem os de peso 0");
    }
    require(near(charts.drillSpeed.back(), 0.9, 1e-5) && near(charts.drillRequested.back(), 1.0, 1e-5),
        "as velocidades do exercicio");
    require(near(charts.drillReward.back(), 0.4, 1e-5), "a recompensa por segundo e o retorno sobre a duracao");
    // O teste sem ruido (o da rede, nao o do classico) vale da rodada 60 em diante.
    require(std::isnan(charts.drillTest[4]) && near(charts.drillTest[5], 75.0, 1e-4) && near(charts.drillTest.back(), 75.0, 1e-4),
        "o teste sem ruido vale da rodada dele em diante, o do classico fica de fora");
    require(near(charts.decisions.back(), 7095.5, 1e-3), "as decisoes por segundo, pela media das rodadas da fatia");
}

// Poucos episodios numa fatia: ela junta as anteriores; um exercicio sem rodadas fica sem pontos.
void testTrail() {
    TrainingHistory3D history;
    for (std::uint64_t i = 1; i <= 20; ++i) history.drills.push_back(point(i, "frente", 2.0, i % 2 == 0 ? 1.0 : 0.0));
    const std::vector<TrainingDrillInfo3D> task { { "frente", 1.0, 4.5, false } };
    const TrainingCharts3D charts = trainingCharts3D(history, task, "frente", 20);
    // Cada fatia tem 2 episodios: ela junta as 7 anteriores (8 fatias, 16 episodios): metade caiu.
    require(near(charts.drillStanding.back(), 50.0, 1e-4), "a fatia junta as anteriores ate o limite");
    require(std::isnan(charts.drillStanding.front()), "a primeira fatia so tem 2 episodios: sem ponto");
    require(near(charts.drillStanding[3], 50.0, 1e-4), "com 4 fatias juntas (8 episodios) ja ha ponto");
    const TrainingCharts3D other = trainingCharts3D(history, task, "nenhum", 20);
    require(std::isnan(other.drillStanding.back()) && !std::isnan(other.evolution.back()),
        "um exercicio sem rodadas fica sem pontos; o treino continua");
    // Poucos episodios mesmo juntando: sem ponto (o comeco do treino).
    TrainingHistory3D sparse;
    for (std::uint64_t i = 1; i <= 4; ++i) sparse.drills.push_back(point(i, "frente", 1.0, 1.0));
    const TrainingCharts3D thin = trainingCharts3D(sparse, task, "frente", 4);
    require(std::isnan(thin.drillStanding.back()) && std::isnan(thin.evolution.back()),
        "menos de 8 episodios mesmo juntando: sem ponto");
    // As decisoes por segundo: a fatia sem rodada amostrada fica com a mais perto.
    TrainingHistory3D sampled = history;
    TrainingIterationPoint3D only;
    only.iteration = 15;
    only.sps = 6000.0;
    sampled.iterations.push_back(only);
    const TrainingCharts3D filled = trainingCharts3D(sampled, task, "frente", 20);
    require(near(filled.decisions.front(), 6000.0, 1e-3) && near(filled.decisions.back(), 6000.0, 1e-3),
        "as fatias sem rodada amostrada ficam com a medida mais perto");
    // O treino sem metade do peso medido na fatia: sem ponto (so o exercicio leve apareceu).
    {
        TrainingHistory3D early;
        for (std::uint64_t i = 1; i <= 10; ++i) early.drills.push_back(point(i, "leve", 40.0, 1.0));
        for (std::uint64_t i = 11; i <= 20; ++i) {
            early.drills.push_back(point(i, "leve", 40.0, 0.0));
            early.drills.push_back(point(i, "pesado", 40.0, 0.0));
        }
        const std::vector<TrainingDrillInfo3D> weights { { "leve", 1.0, 4.5, false }, { "pesado", 3.0, 4.5, false } };
        const TrainingCharts3D covered = trainingCharts3D(early, weights, "leve", 2);
        require(std::isnan(covered.evolution.front()) && !std::isnan(covered.evolution.back()),
            "com menos da metade do peso medido, o treino fica sem ponto");
        require(!std::isnan(covered.drillEvolution.front()), "o exercicio continua com o seu ponto");
    }
    const TrainingCharts3D empty = trainingCharts3D(TrainingHistory3D {}, task, "frente", 20);
    require(empty.times.empty() && empty.evolution.empty(), "sem rodadas, nenhum ponto");
}

TrainingEvaluationPoint3D evaluation(std::uint64_t iteration, const std::string& drill, const std::string& solver,
    double falls, double speed, double requested) {
    TrainingEvaluationPoint3D p;
    p.iteration = iteration;
    p.time = 1000.0 + static_cast<double>(iteration);
    p.drill = drill;
    p.solver = solver;
    p.falls = falls;
    p.bodySpeed = speed;
    p.requestedSpeed = requested;
    return p;
}

// A evolucao real (28/10): a avaliacao da rede, um ponto por avaliacao.
void testEvaluation() {
    const std::vector<TrainingDrillInfo3D> task { { "frente", 1.0, 4.8, false }, { "corte", 1.0, 4.8, false },
        { "parado", 1.0, 0.0, false }, { "fora", 0.0, 4.8, false } };
    TrainingHistory3D history;
    // Avaliacao 100: frente nao cai e faz 90% (pedido 3), o corte cai 25% e faz 120% (vale 1), o parado
    // cai 50% (sem velocidade); o classico e o exercicio sem peso nao contam.
    history.evaluations.push_back(evaluation(100, "corte", "rede", 0.25, 3.6, 3.0));
    history.evaluations.push_back(evaluation(100, "fora", "rede", 1.0, 0.0, 3.0));
    history.evaluations.push_back(evaluation(100, "frente", "classico", 1.0, 0.0, 3.0));
    history.evaluations.push_back(evaluation(100, "frente", "rede", 0.0, 2.7, 3.0));
    history.evaluations.push_back(evaluation(100, "parado", "rede", 0.5, 0.1, 0.0));
    // Avaliacao 200 pela metade (so 1 dos 3): nao vira ponto.
    history.evaluations.push_back(evaluation(200, "frente", "rede", 0.0, 3.0, 3.0));
    const TrainingEvaluationCharts3D charts = trainingEvaluationCharts3D(history, task);
    require(charts.standing.size() == 1 && charts.iterations.front() == 100, "uma avaliacao pela metade nao vira ponto");
    require(near(charts.standing.front(), 100.0 * (1.0 + 0.75 + 0.5) / 3.0, 1e-4), "sem cair: a media dos exercicios");
    require(near(charts.worstStanding.front(), 50.0, 1e-4) && charts.worstDrill.front() == "parado", "o pior e qual");
    require(near(charts.reach.front(), 100.0 * (0.9 + 1.0) / 2.0, 1e-4), "a velocidade: feita/pedida ate 1, so os que andam");
    require(trainingEvaluationCharts3D(TrainingHistory3D {}, task).times.empty(), "sem avaliacao, nenhum ponto");
}

// A linhagem (28/10): o historico do pai ate a rodada do ramo vai antes do do filho.
void testLineage() {
    TrainingHistory3D parent, child;
    for (std::uint64_t i : { 10u, 20u, 30u }) parent.evaluations.push_back(evaluation(i, "frente", "rede", 0.5, 1.0, 1.0));
    for (std::uint64_t i : { 10u, 20u, 30u }) parent.drills.push_back(point(i, "frente", 10.0, 0.5));
    child.evaluations.push_back(evaluation(25, "frente", "rede", 0.0, 1.0, 1.0));
    prependTrainingAncestor3D(child, parent, 20);
    require(child.evaluations.size() == 3 && child.evaluations[0].iteration == 10 && child.evaluations[1].iteration == 20
            && child.evaluations[2].iteration == 25,
        "o pai ate a rodada do ramo, antes do filho");
    require(child.drills.size() == 2, "as rodadas do pai depois do ramo ficam de fora");
    TrainingHistory3D recent;
    prependTrainingAncestor3D(recent, parent, 30, 1015.0);
    require(recent.evaluations.size() == 2 && recent.evaluations.front().iteration == 20, "so desde o horario pedido");
}

} // namespace

int main() {
    try {
        testParts();
        testCharts();
        testTrail();
        testEvaluation();
        testLineage();
    } catch (const std::exception& error) {
        std::cerr << "Training evolution tests failed: " << error.what() << '\n';
        return 1;
    }
    std::cout << "Training evolution tests passed\n";
    return 0;
}
