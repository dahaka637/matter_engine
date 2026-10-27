// A tela "Aprendizado por Reforco" sem janela, num treino de mentira do Corpo
// (uma pasta temporaria no lugar de ~/.local/share/matter-engine/rl e do
// socket): o visualizador roda o jogador de verdade com a politica exportada,
// o comando na mao e os empurroes andam, o historico do banco vira grafico, e
// o ImGui nao reclama. MATTERENGINE_UI_SNAPSHOT=<pasta> guarda as fotos da tela.
//
// MATTER_RL_TEST_REAL=<pasta>: o treino de verdade mais recente da pasta de
// dados (ou MATTER_RL_TEST_RUN), sem janela, com o comando na mao a
// MATTER_RL_TEST_SPEED m/s (1,0 por padrao): imprime o pedido e a velocidade
// e grava o visualizador de lado e de frente na pasta. Nada e clicado: o
// treino nao muda.

#include "Engine/Modeling/Raster/ImageIO.hpp"
#include "Engine/RL/Environment/BodyTrainingBatch3D.hpp"
#include "Engine/RL/Policy/PolicyRuntime3D.hpp"
#include "Engine/RL/Motor/MotorTraining3D.hpp"
#include "Workbench/ReinforcementLearning/MotorViewer3D.hpp"
#include "Engine/RL/Supervisor/TrainerLink3D.hpp"
#include "Support/HeadlessUi.hpp"
#include "Workbench/ReinforcementLearning/ReinforcementLearningScreen.hpp"

#include "imgui.h"
#include "imgui_internal.h"

#include <nlohmann/json.hpp>
#include <sqlite3.h>

#include <algorithm>
#include <chrono>
#include <thread>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

using namespace MatterEngine;
using TestSupport::FakeCanvas;
using TestSupport::snapshot;
using TestSupport::sweepIdConflicts;
using Workbench::ReinforcementLearningScreen;

namespace {

int checks = 0;
constexpr const char* Task = "aquisicao_p3a";

void require(bool condition, const std::string& message) {
    ++checks;
    if (!condition) throw std::runtime_error(message);
}

void frame(ReinforcementLearningScreen& screen, FakeCanvas& canvas, int count = 1) {
    ImGuiIO& io = ImGui::GetIO();
    for (int i = 0; i < count; ++i) {
        io.DeltaTime = 1.0f / 60.0f;
        ImGui::NewFrame();
        screen.update(io.DeltaTime);
        screen.draw(canvas, 1.0f);
        ImGui::Render();
    }
}

std::filesystem::path sourceRoot() { return std::filesystem::path(MATTERENGINE_TEST_ASSETS_DIR).parent_path(); }

// Uma politica de mentira no formato do treinador, com as entradas da tarefa
// pelos nomes: pesos zero (a acao zero segura a pose base).
void writePolicy(const std::filesystem::path& path, std::uint64_t iteration, const std::vector<std::string>& names,
    std::size_t actions, const nlohmann::json& meta = nlohmann::json::object()) {
    const std::size_t obs = names.size(), hidden = 8;
    std::vector<float> data;
    const std::size_t mean = data.size();
    data.insert(data.end(), obs, 0.0f);
    const std::size_t inverse = data.size();
    data.insert(data.end(), obs, 1.0f);
    const std::size_t w1 = data.size();
    data.insert(data.end(), obs * hidden, 0.0f);
    const std::size_t b1 = data.size();
    data.insert(data.end(), hidden, 0.0f);
    const std::size_t w2 = data.size();
    data.insert(data.end(), hidden * actions, 0.0f);
    const std::size_t b2 = data.size();
    data.insert(data.end(), actions, 0.0f);
    nlohmann::json header {
        { "schema", "matter-rl-policy-1" },
        { "observationSize", obs },
        { "actionSize", actions },
        { "activation", "elu" },
        { "iteration", iteration },
        { "observationNames", names },
        { "normalizer", { { "mean", mean }, { "invStd", inverse }, { "clip", 5.0 } } },
        { "layers", nlohmann::json::array({ { { "inputs", obs }, { "outputs", hidden }, { "weights", w1 }, { "bias", b1 } },
                        { { "inputs", hidden }, { "outputs", actions }, { "weights", w2 }, { "bias", b2 } } }) },
    };
    header.update(meta);
    const std::string text = header.dump();
    std::filesystem::create_directories(path.parent_path());
    std::ofstream out(path, std::ios::binary);
    out.write("MRLPOL01", 8);
    const std::uint32_t size = static_cast<std::uint32_t>(text.size());
    out.write(reinterpret_cast<const char*>(&size), 4);
    out.write(text.data(), static_cast<std::streamsize>(text.size()));
    const std::size_t pad = (4 - (12 + text.size()) % 4) % 4;
    for (std::size_t i = 0; i < pad; ++i) out.put('\0');
    out.write(reinterpret_cast<const char*>(data.data()), static_cast<std::streamsize>(data.size() * 4));
}

void writeJson(const std::filesystem::path& path, const nlohmann::json& value);

void checkMotorExploration(const std::filesystem::path& folder) {
    const auto assets = sourceRoot() / "assets";
    const auto task = assets / "rl/motor/basico.json";
    MotorSettings3D settings;
    settings.assetsDirectory = assets;
    settings.taskPath = task;
    settings.environments = 1;
    settings.threads = 1;
    MotorTraining3D batch(settings);
    std::filesystem::create_directories(folder);
    std::ofstream historyLog(folder / "log.jsonl");
    historyLog << R"({"rodada":1,"hora":1,"rumos":{"parado":{"n":1,"erro_peito_graus":2,"deriva":0.1,"imitacao_versao":2,"imitacao":{"similaridade":0.99}}}})" << '\n';
    historyLog << R"({"rodada":2,"hora":2,"rumos":{"parado":{"n":1,"ep_segundos":1,"nota":0.5,"erro_peito_graus":2,"deriva":0.1,"postura":0.8,"imitacao_versao":3,"imitacao":{"similaridade":0.75}}}})" << '\n';
    historyLog.close();
    const auto history = MotorViewer3D::readHistory(folder, 600, 0);
    require(history.drills.size() == 2 && std::isnan(history.drills[0].imitation) &&
            std::abs(history.drills[1].imitation - .75) < 1e-6 && std::abs(history.drills[1].posture - .8) < 1e-6,
            "grafico le a semelhanca e a postura do motor-v4, nao as notas antigas");
    const auto path = folder / "exploration.mrlpolicy";
    nlohmann::json meta {{"observationLayout","motor-v4"},{"bodyHash",batch.identity()},
                         {"explorationStd",std::vector<float>(batch.actionSize(),.4f)}};
    writePolicy(path,7,batch.observationNames(),batch.actionSize(),meta);
    PolicyRuntime3D policy(path);
    require(policy.info().explorationStd.size()==batch.actionSize(), "ruido por eixo exportado");
    std::vector<float> obs(batch.observationSize()),mean(batch.actionSize());
    policy.evaluate(obs,mean);
    require(std::all_of(mean.begin(),mean.end(),[](float v){return v==0.f;}), "runtime medio continua sem ruido");
    MotorViewer3D first(assets,task,"parado",42),same(assets,task,"parado",42),quiet(assets,task,"parado",42);
    std::string label;
    first.setPolicy(path,label);
    require(first.hasPolicy() && label.find("exploracao")!=std::string::npos && label.find("sustentacao")==std::string::npos,
            "previa identifica exploracao, sem apoio");
    same.setPolicy(path,label);
    meta.erase("explorationStd");
    writePolicy(folder/"quiet.mrlpolicy",7,batch.observationNames(),batch.actionSize(),meta);
    quiet.setPolicy(folder/"quiet.mrlpolicy",label);
    float difference=0.f;
    for(int k=0;k<12;++k) {
        first.step();same.step();quiet.step();
        const auto& a=first.view().state->links;
        const auto& b=same.view().state->links;
        const auto& c=quiet.view().state->links;
        for(std::size_t i=0;i<a.size();++i) {
            require((a[i].position-b[i].position).length()<1e-6f,"mesma semente reproduz tentativas");
            difference+=(a[i].position-c[i].position).lengthSquared();
        }
    }
    require(difference>1e-5f,"ruido altera as tentativas fisicas da previa");
    first.reset(42);same.reset(42);
    first.step();same.step();
    require((first.view().state->links[0].position-same.view().state->links[0].position).length()<1e-6f,"reset reproduz exploracao");
    for(const auto& bad : {std::vector<float>{.4f},std::vector<float>(batch.actionSize(),-1.f)}) {
        meta["explorationStd"]=bad;
        writePolicy(path,8,batch.observationNames(),batch.actionSize(),meta);
        first.setPolicy(path,label);
        require(!first.hasPolicy(),"ruido invalido recusado na fronteira");
    }
    // A previa pede o trote no teto atual do curriculo (curriculo.json ao lado da tarefa).
    const auto runTask = folder / "tarefa.json";
    std::filesystem::copy_file(task, runTask, std::filesystem::copy_options::overwrite_existing);
    writeJson(folder / "curriculo.json", nlohmann::json{{"limits", std::vector<float>(8, .4f)}, {"stage", 0}});
    MotorViewer3D trot(assets,runTask,"trote_frente",42);
    trot.setPolicy(folder/"quiet.mrlpolicy",label);
    for(int k=0;k<70;++k) trot.step();
    const float requested = trot.view().command.moveVelocity.length();
    require(requested > .25f && requested < .41f, "previa do trote usa o teto do curriculo");
}

void writeJson(const std::filesystem::path& path, const nlohmann::json& value) {
    std::filesystem::create_directories(path.parent_path());
    std::ofstream(path) << value.dump(2);
}

// Um treino do Corpo de mentira (run.json, status.json e a politica) e um
// antigo, do contrato 1, que a tela nao lista.
void writeRuns(const std::filesystem::path& data) {
    BodyBatchSettings3D settings;
    settings.taskPath = sourceRoot() / "assets" / "rl" / "tasks" / (std::string(Task) + ".json");
    settings.environmentCount = 1;
    settings.environmentsPerScene = 1;
    settings.workerThreads = 1;
    const BodyTrainingBatch3D batch(settings);
    std::vector<std::string> names;
    for (const RLChannel3D& channel : batch.observationLayout().channels) names.push_back(channel.name);
    const std::filesystem::path run = data / "runs" / "teste_v2";
    writePolicy(run / "exports" / "policy_latest.mrlpolicy", 5, names, batch.actionSize());
    writeJson(run / "run.json", { { "task", Task }, { "contract", 2 }, { "environments", 4 } });
    writeJson(run / "status.json",
        { { "run", "teste_v2" }, { "task", Task }, { "taskName", "Aquisicao P3a: andar com o juiz" }, { "contract", 2 },
            { "state", "parado" }, { "mode", "equilibrado" }, { "iteration", 50 }, { "steps", 614400 },
            { "trainSeconds", 3600.0 },
            { "last", { { "reward", 0.04 }, { "episode_seconds", 18.0 }, { "fall_fraction", 0.05 }, { "style", 0.04 } } },
            { "style", { { "enabled", true }, { "beta", 1.0 } } },
            { "evaluation",
                { { "iteration", 50 },
                    { "solvers",
                        { { "jogo", { { "andar_frente", { { "falls", 0.0 }, { "velocity_error_median", 0.41 },
                                                           { "useful_steps", 1.6 }, { "sustained", 1.0 }, { "slip", 0.05 } } } } },
                            { "treino", { { "andar_frente", { { "falls", 0.03 }, { "velocity_error_median", 0.38 } } } } } } } } } });
    writeJson(data / "runs" / "antigo" / "run.json", { { "task", "corpo" }, { "contract", 1 } });
}

void writeHistory(const std::filesystem::path& database) {
    sqlite3* db = nullptr;
    require(sqlite3_open(database.string().c_str(), &db) == SQLITE_OK, "banco de mentira");
    const char* sql = R"(
        CREATE TABLE iterations (run TEXT, iteration INTEGER, time REAL, steps INTEGER, sps REAL, reward REAL,
            episodes INTEGER, episode_return REAL, episode_seconds REAL, fall_fraction REAL);
        CREATE TABLE body_evaluations (run TEXT, iteration INTEGER, time REAL, steps INTEGER, drill TEXT, solver TEXT,
            data TEXT);
        CREATE TABLE events (id INTEGER PRIMARY KEY, run TEXT, time REAL, kind TEXT, author TEXT, message TEXT, data TEXT);
    )";
    require(sqlite3_exec(db, sql, nullptr, nullptr, nullptr) == SQLITE_OK, "tabelas do banco de mentira");
    for (int i = 1; i <= 50; ++i) {
        const std::string row = "INSERT INTO iterations(run, iteration, time, steps, sps, reward, episode_seconds, "
                                "fall_fraction) VALUES ('teste_v2', " + std::to_string(i) + ", " + std::to_string(1.0e9 + i)
            + ", " + std::to_string(i * 12288) + ", 9000, 0.04, " + std::to_string(2.0 + 0.3 * i) + ", 0.1);";
        require(sqlite3_exec(db, row.c_str(), nullptr, nullptr, nullptr) == SQLITE_OK, "rodada do banco de mentira");
    }
    for (int i = 1; i <= 5; ++i)
        for (const char* solver : { "treino", "jogo" }) {
            const std::string row = "INSERT INTO body_evaluations VALUES ('teste_v2', " + std::to_string(i * 10) + ", "
                + std::to_string(1.0e9 + i * 10) + ", " + std::to_string(i * 122880) + ", 'andar_frente', '" + solver
                + "', '{\"falls\": " + std::to_string(0.5 - 0.1 * i) + ", \"velocity_error_median\": 0.4, "
                  "\"useful_steps\": 1.5}');";
            require(sqlite3_exec(db, row.c_str(), nullptr, nullptr, nullptr) == SQLITE_OK, "avaliacao de mentira");
        }
    require(sqlite3_exec(db, "INSERT INTO events(run, time, kind, author, message) VALUES ('teste_v2', 1000000060, "
                             "'beta', 'claude', 'estilo: 0.5 -> 1.0');",
                nullptr, nullptr, nullptr)
            == SQLITE_OK,
        "evento de mentira");
    sqlite3_close(db);
}

// O treino de verdade, sem janela (a conferencia antes de entregar a tela).
int realRun(const std::filesystem::path& out) {
    TestSupport::HeadlessUi ui;
    FakeCanvas canvas;
    ReinforcementLearningScreen::Options options;
    options.sourceRoot = sourceRoot();
    options.capturesDirectory = out;
    if (const char* run = std::getenv("MATTER_RL_TEST_RUN")) options.run = run;
    ReinforcementLearningScreen screen(std::move(options), {});
    // Como o app: quadros desenhados desde o comeco, enquanto o jogador e a rede carregam.
    for (int i = 0; i < 240; ++i) {
        frame(screen, canvas);
        std::this_thread::sleep_for(std::chrono::milliseconds(8));
    }
    screen.waitUntilLoaded();
    if (const char* drill = std::getenv("MATTER_RL_TEST_DRILL")) {
        screen.showDrill(drill);
        screen.waitUntilLoaded();
    }
    std::cout << "status: " << (screen.status().alive ? "vivo" : "fechado") << ", " << screen.status().state << ", rodada "
              << screen.status().iteration << ", pid " << screen.status().pid << '\n';
    std::cout << "historico: " << screen.history().iterations.size() << " rodadas, "
              << screen.history().evaluations.size() << " avaliacoes, " << screen.drills().size() << " exercicios\n";
    if (screen.status().task == "motor_basico")
        require(screen.drills() == std::vector<std::string>(MotorTraining3D::drillIds().begin(),MotorTraining3D::drillIds().end()), "o motor mostra os dez exercicios do Basico");
    require(!screen.history().iterations.empty(), "os graficos deveriam ter as rodadas do treino");
    require(screen.viewerReady(), "o visualizador deveria abrir o treino: " + screen.viewerError());
    require(screen.policyLoaded(), "a politica do treino deveria carregar");
    const char* speedText = std::getenv("MATTER_RL_TEST_SPEED");
    const float speed = speedText ? std::strtof(speedText, nullptr) : 1.0f;
    screen.setManualSpeed(speed);
    std::filesystem::create_directories(out);
    double requested = 0.0, measured = 0.0;
    int samples = 0;
    double turned = 0.0;
    for (int second = 1; second <= 20; ++second) {
        // O giro do peito (os exercicios parados giram no lugar), decisao a decisao.
        for (int k = 0; k < 30; ++k) {
            const float before = screen.viewerChestHeading();
            screen.advanceViewer(1);
            turned += std::abs(std::remainder(screen.viewerChestHeading() - before, 6.28318531f));
        }
        if (second > 4) {
            requested += screen.viewerRequestedSpeed();
            measured += screen.viewerSpeed();
            ++samples;
        }
        if (second % 4 == 0) {
            for (const auto& [name, yaw] : { std::pair<const char*, float> { "lado", 90.0f }, { "frente", 0.0f } }) {
                const char* far = std::getenv("MATTER_RL_TEST_DISTANCE");
                screen.frameView(yaw, far ? 25.0f : 6.0f, far ? std::strtof(far, nullptr) : 4.4f);
                require(screen.renderViewport(720, 540), "o visualizador renderiza");
                std::string error;
                const std::filesystem::path file = out / (std::string("rl_") + name + "_" + std::to_string(second) + "s.png");
                require(Modeling::writePng(screen.image(), file.string(), &error), error);
            }
        }
    }
    // O custo do visualizador (a fluidez da tela): a simulacao de 1 s (30
    // decisoes) e um quadro desenhado, no tamanho da tela de verdade.
    {
        using Clock = std::chrono::steady_clock;
        const auto seconds = [](Clock::time_point a, Clock::time_point b) { return std::chrono::duration<double>(b - a).count(); };
        double simulate = 0.0, draw = 0.0;
        constexpr int Rounds = 5;
        for (int i = 0; i < Rounds; ++i) {
            const auto a = Clock::now();
            screen.advanceViewer(30);
            const auto b = Clock::now();
            screen.frameView(i % 2 == 0 ? 90.0f : 0.0f, 6.0f);
            require(screen.renderViewport(1540, 660), "o visualizador renderiza");
            draw += seconds(b, Clock::now());
            simulate += seconds(a, b);
        }
        std::cout << "custo do visualizador: simular 1 s = " << 1000.0 * simulate / Rounds << " ms, um quadro (1540x660) = "
                  << 1000.0 * draw / Rounds << " ms\n";
    }
    frame(screen, canvas, 3);
    snapshot(canvas, "rl_tela_treino_real");
    if (std::getenv("MATTER_RL_TEST_CATEGORIES")) {
        ImGuiWindow* sidebar = nullptr;
        for (auto* window : ImGui::GetCurrentContext()->Windows)
            if (std::string(window->Name).find("RLSidebar") != std::string::npos) sidebar = window;
        require(sidebar != nullptr, "a lateral de exercicios existe");
        const int expandedVertices = ImGui::GetDrawData()->TotalVtxCount;
        sidebar->StateStorage.SetInt(sidebar->GetID("Básico"),0);
        sidebar->StateStorage.SetInt(sidebar->GetID("Compostos"),1);
        frame(screen,canvas,2);
        require(ImGui::GetDrawData()->TotalVtxCount < expandedVertices, "recolher Basico esconde os exercicios");
        snapshot(canvas,"rl_categorias_recolhidas");
        sidebar->StateStorage.SetInt(sidebar->GetID("Básico"),1);
    }
    {
        // A gravacao: 1 s atras a pelve esta em outro lugar; 0 atras volta a de agora; meio quadro fica entre os dois.
        const Vec3 now = screen.viewerPelvis();
        screen.scrubViewer(30.0f);
        const Vec3 before = screen.viewerPelvis();
        screen.scrubViewer(0.5f);
        const Vec3 half = screen.viewerPelvis();
        screen.scrubViewer(0.0f);
        // (Nos exercicios parados a pelve fica no lugar: so andando ela tem de ter andado.)
        const char* shownDrill = std::getenv("MATTER_RL_TEST_DRILL");
        const bool moving = shownDrill == nullptr || std::string(shownDrill).rfind("rumo", 0) == 0;
        require(!moving || (now - before).length() > 0.05f, "a gravacao deveria mostrar o quadro de 1 s atras");
        require((screen.viewerPelvis() - now).length() < 1e-5f, "o fim da gravacao e o quadro de agora");
        require(!moving || (half - now).length() < (before - now).length(), "meio quadro atras fica perto de agora");
        std::cout << "gravacao: a pelve andou " << (now - before).length() << " m no ultimo 1 s\n";
        // Com a gravacao cheia (mais de 30 s), o quadro mostrado segue o de agora (nao congela no 899).
        screen.advanceViewer(950);
        const Vec3 full = screen.viewerPelvis();
        screen.advanceViewer(30);
        require(!moving || (screen.viewerPelvis() - full).length() > 0.05f,
            "com a gravacao cheia o visualizador deveria seguir");
    }
    if (const char* mouse = std::getenv("MATTER_RL_TEST_MOUSE")) {
        // O mouse parado num ponto (x,y): a dica do grafico sob ele.
        float x = 0.0f, y = 0.0f;
        if (std::sscanf(mouse, "%f,%f", &x, &y) == 2) {
            ImGui::GetIO().MousePos = { x, y };
            frame(screen, canvas, 3);
            snapshot(canvas, "rl_tela_mouse");
        }
    }
    if (const char* window = std::getenv("MATTER_RL_TEST_EXPAND")) {
        screen.setChartsExpanded(true, std::atoi(window));
        screen.waitUntilLoaded();
        frame(screen, canvas, 3);
        std::cout << "expandido: " << screen.history().iterations.size() << " rodadas na janela\n";
        snapshot(canvas, "rl_tela_graficos");
    }
    std::cout << "giro do peito em 20 s: " << turned * 57.2957795 << " graus\n";
    std::cout << "treino " << screen.shownRun() << ": pedido medio " << requested / samples << " m/s, ele "
              << measured / samples << " m/s (de 5 a 20 s), passos " << screen.viewerSteps() << '\n';
    require(ui.errors().empty(), "o ImGui reclamou: " + (ui.errors().empty() ? std::string() : ui.errors().front()));
    return 0;
}

// O treinador de processo proprio (o rastreador): um Python de mentira que grava
// o status.json com o pid e sai no SIGTERM. A ponte o abre pelo comando do
// run.json, o ve vivo, o para pelo sinal e o ve parado.
void processTrainer(const std::filesystem::path& data) {
    const std::filesystem::path run = data / "runs" / "rast_teste";
    std::filesystem::create_directories(run);
    const std::string script = "import json,os,signal,sys,time\n"
        "p=sys.argv[1]\n"
        "def w(st): open(p+'/status.json','w').write(json.dumps({'run':'rast_teste','task':'rastreador_frente','state':st,'pid':os.getpid(),'iteration':7}))\n"
        "signal.signal(signal.SIGTERM, lambda *a: (w('parado'), sys.exit(0)))\n"
        "w('treinando')\n"
        "while True: time.sleep(0.05)\n";
    nlohmann::json config { { "kind", "rastreador" }, { "task", "rastreador_frente" },
        { "comando", { "/usr/bin/python3", "-c", script, run.string(), "--treino", "rast_teste" } },
        { "pasta", run.string() } };
    std::ofstream(run / "run.json") << config.dump();
    TrainerPaths3D paths = defaultTrainerPaths3D(sourceRoot());
    paths.data = data;
    TrainerLink3D link(paths, "rast_teste");
    std::string error;
    require(link.start("rastreador_frente", "equilibrado", &error), "a ponte abre o treinador do run.json: " + error);
    const auto waitFor = [&](bool alive) {
        for (int i = 0; i < 100; ++i) {
            link.refresh();
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            if (link.status().alive == alive && link.status().known) return true;
        }
        return false;
    };
    require(waitFor(true), "o treinador aberto aparece vivo pelo pid do status.json");
    require(link.status().pid > 0 && link.status().iteration == 7 && link.status().state == "treinando",
        "o status do treinador de processo proprio chega a tela");
    link.send("parar");
    require(waitFor(false), "o parar fecha o treinador pelo sinal");
    require(link.status().state == "parado", "parado depois do sinal");
    std::cout << "treinador de processo proprio: abre, aparece vivo, para pelo sinal\n";
}

// O empurrao unico no visualizador (23/10, como no treino): o exercicio leve
// recomeca settleSeconds (3 s) depois do empurrao, que vem entre 1,5 e 3 s.
void checkSinglePushViewer() {
    LocomotionViewer3D viewer(sourceRoot() / "assets" / "rl" / "tasks" / "locomocao_e1.json", sourceRoot() / "assets",
        "parado_empurrao", 7);
    float first = -1.0f;
    for (int k = 0; k < 230 && first < 0.0f; ++k) {
        const std::uint32_t before = viewer.view().steps;
        viewer.step();
        if (viewer.view().steps < before) first = static_cast<float>(before + 1) * viewer.policyStepSeconds();
    }
    require(first >= 4.6f - 1.0e-3f && first <= 6.2f + 0.07f,
        "o visualizador recomeca 3 s depois do empurrao unico (recomecou em " + std::to_string(first) + " s)");
    std::cout << "visualizador: empurrao unico, recomeca em " << first << " s\n";
}

} // namespace

int main() {
    try {
        if (const char* real = std::getenv("MATTER_RL_TEST_REAL")) return realRun(real);
        const std::filesystem::path sandbox = std::filesystem::temp_directory_path() / "matter_rl_tela";
        std::filesystem::remove_all(sandbox);
        std::filesystem::create_directories(sandbox / "runtime");
        setenv("MATTER_RL_DATA", (sandbox / "dados").c_str(), 1);
        setenv("XDG_RUNTIME_DIR", (sandbox / "runtime").c_str(), 1);
        unsetenv("MATTER_RL_SOCKET");
        require(!LocomotionViewer3D::isLocomotionTask(sandbox / "ausente.json"), "tarefa ausente nao pode derrubar a tela");
        checkSinglePushViewer();
        checkMotorExploration(sandbox / "motor");
        writeRuns(sandbox / "dados");
        processTrainer(sandbox / "dados_processo");
        writeHistory(sandbox / "dados" / "rl.db");

        TestSupport::HeadlessUi ui;
        FakeCanvas canvas;
        ReinforcementLearningScreen::Options options;
        options.sourceRoot = sourceRoot();
        options.capturesDirectory = sandbox / "capturas";
        ReinforcementLearningScreen screen(std::move(options), {});
        {
            // Como o app: o primeiro quadro desenha antes de qualquer update (ainda sem visualizador).
            ImGui::GetIO().DeltaTime = 1.0f / 60.0f;
            ImGui::NewFrame();
            screen.draw(canvas, 1.0f);
            ImGui::Render();
        }
        require(screen.runs() == std::vector<std::string> { "teste_v2" }, "a tela so lista os treinos do Corpo");
        require(screen.shownRun() == "teste_v2", "abre no treino mais recente");
        screen.waitUntilLoaded();
        require(screen.modelLoaded(), "o jogador deveria carregar");
        require(screen.viewerReady(), "o visualizador deveria existir: " + screen.viewerError());
        require(screen.policyLoaded(), "a politica do treino deveria carregar");
        require(screen.drills() == std::vector<std::string> { "andar_frente" }, "os exercicios vem da tarefa do treino");
        frame(screen, canvas, 40);
        require(canvas.uploads > 0 && canvas.width > 100, "o visualizador deveria desenhar");
        const TrainerStatus3D status = screen.status();
        require(!status.alive && status.known && status.task == Task, "o status gravado do treino aparece");
        require(status.evaluated && status.evaluation.size() == 2 && status.styleBeta == 1.0, "a avaliacao e o juiz aparecem");
        require(screen.history().iterations.size() == 50 && screen.history().evaluations.size() == 10,
            "o historico do banco deveria aparecer");
        require(screen.history().evaluations.back().drill == "andar_frente"
                && std::abs(screen.history().evaluations.back().falls) < 1e-6,
            "a avaliacao do banco vem por exercicio");
        snapshot(canvas, "rl_tela");

        // Nenhum ID repetido (o ImGui so avisa com o mouse em cima): a varredura
        // acha dois botoes iguais e nao acha nada na tela.
        const auto duplicates = sweepIdConflicts([] {
            ImGui::SetNextWindowPos({ 0.0f, 0.0f });
            ImGui::Begin("teste");
            ImGui::Button("Igual", { 200.0f, 60.0f });
            ImGui::Button("Igual", { 200.0f, 60.0f });
            ImGui::End();
        });
        require(!duplicates.empty(), "a varredura deveria achar dois botoes com o mesmo ID");
        const auto conflicts = sweepIdConflicts([&] { screen.draw(canvas, 1.0f); });
        require(conflicts.empty(), "IDs repetidos na tela: " + (conflicts.empty() ? std::string() : conflicts.front()));

        // O que o mundo mostra: o jogador, so o esqueleto, so os colisores.
        screen.setDisplay(ReinforcementLearningScreen::Display::Skeleton);
        frame(screen, canvas, 2);
        snapshot(canvas, "rl_tela_esqueleto");
        screen.setDisplay(ReinforcementLearningScreen::Display::Colliders);
        frame(screen, canvas, 2);
        snapshot(canvas, "rl_tela_colisores");
        screen.setDisplay(ReinforcementLearningScreen::Display::Full);

        // O comando na mao: o pedido vira a velocidade escolhida; de volta ao
        // piloto, o roteiro do exercicio.
        screen.setManualSpeed(1.5f);
        screen.advanceViewer(3);
        require(std::abs(screen.viewerRequestedSpeed() - 1.5f) < 0.05f,
            "o comando na mao pede 1,5 m/s (pediu " + std::to_string(screen.viewerRequestedSpeed()) + ")");
        frame(screen, canvas, 2);
        snapshot(canvas, "rl_tela_na_mao");
        const auto manualConflicts = sweepIdConflicts([&] { screen.draw(canvas, 1.0f); });
        require(manualConflicts.empty(),
            "IDs repetidos com o comando na mao: " + (manualConflicts.empty() ? std::string() : manualConflicts.front()));
        screen.setManualSpeed(0.0f);
        screen.advanceViewer(2);
        require(screen.viewerRequestedSpeed() < 0.01f, "na mao a 0 m/s ele para");
        screen.setManualSpeed(std::nullopt);

        // A politica de mentira segura a pose base: depois de 1 s o corpo ainda
        // esta de pe e no quadro; um empurrao forte o tira do lugar.
        screen.advanceViewer(30);
        require(screen.renderViewport(640, 480) && !screen.image().empty(), "o visualizador renderiza sob demanda");
        const std::array<float, 2> shown = screen.viewerPelvisOnScreen();
        require(shown[0] > 0.2f && shown[0] < 0.8f && shown[1] > 0.2f && shown[1] < 0.8f, "a camera mostra o jogador");
        const std::uint32_t before = screen.viewerSteps();
        screen.push(0, 600.0f);
        screen.advanceViewer(10);
        require(screen.viewerSteps() != before, "o mundo segue depois do empurrao");
        frame(screen, canvas, 5);
        snapshot(canvas, "rl_tela_empurrao");
        require(ui.errors().empty(), "o ImGui reclamou: " + (ui.errors().empty() ? std::string() : ui.errors().front()));
        std::filesystem::remove_all(sandbox);
        std::cout << "Reinforcement learning screen tests passed (" << checks << " checks)\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "Reinforcement learning screen tests failed: " << error.what() << '\n';
        return 1;
    }
}
