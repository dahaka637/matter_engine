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
#include "Engine/Tracking/TrackingGetUpAmpTraining3D.hpp"
#include "Engine/Tracking/TrackingGetUpAmp3D.hpp"
#include "Engine/Tracking/TrackingLeader3D.hpp"
#include "Engine/Athlete/AthleteAssets3D.hpp"
#include "Workbench/ReinforcementLearning/MotorViewer3D.hpp"
#include "Workbench/ReinforcementLearning/TrackingViewer3D.hpp"
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
    if (meta.value("testRouted", false)) {
        auto routed = header["layers"];
        routed.back()["bias"] = data.size();
        data.insert(data.end(), actions, .75f);
        header["locomotionLayers"] = routed;
        header["skillModeIndex"] = obs - 1;
    }
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

void checkSkillRoutingAndIdle(const std::filesystem::path& folder) {
    const auto path = folder / "routed.mrlpolicy";
    writePolicy(path, 1, { "body", "command", "habilidade.locomocao" }, 2, { { "testRouted", true } });
    PolicyRuntime3D policy(path);
    const std::array<float, 6> observations { .1f, .2f, 0.f, .1f, .2f, 1.f };
    std::array<float, 4> actions {};
    policy.evaluateBatch(observations, actions, 2);
    require(std::abs(actions[0]) < 1e-6f && std::abs(actions[2] - .75f) < 1e-6f,
        "politica unica seleciona cabecas distintas, tambem em lote");
    std::array<float, 2> single {};
    policy.evaluate(std::span(observations).subspan(3, 3), single);
    require(single[0] == actions[2], "execucao individual e lote roteado coincidem");
    TrackingGetUpAmpSettings3D settings;
    settings.assetsDirectory = (sourceRoot() / "assets").string();
    settings.environments = settings.threads = 1;
    settings.evaluation = true;
    settings.evaluationKind = 3;
    TrackingGetUpAmpTraining3D batch(settings);
    auto idle = batch.body(0).links;
    TrackingGetUpPostureProbe3D probe(batch.profile());
    probe.setIdlePose(idle);
    const auto original = probe.measure(idle);
    auto arched = idle;
    const auto chest = std::find_if(batch.profile().links.begin(), batch.profile().links.end(),
        [](const auto& link) { return link.id == "UpperChest"; });
    const auto index = static_cast<std::size_t>(chest - batch.profile().links.begin());
    require(index < arched.size(), "peito existe na referencia IDLE");
    arched[index].orientation = Quaternion::fromAxisAngle({ 0.f, 1.f, 0.f }, .3f) * arched[index].orientation;
    const auto changed = probe.measure(arched);
    require(original.standing && changed.standing, "teste compara duas poses de pe");
    require(probe.standingReward(original, idle, true) > probe.standingReward(changed, arched, true),
        "quieto, IDLE reto rende mais que peito arqueado");
    require(std::abs(probe.standingReward(original, idle, false) - probe.standingReward(changed, arched, false)) < 1e-6f,
        "durante perturbacao nao ha cobranca de semelhança IDLE");
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
    // (Desde 26/10, 0f7ed2d, a previa do motor usa a media da politica, como no jogo: o ruido de exploracao do treino
    // aparecia na tela como tremor. O rotulo e a rodada.)
    require(first.hasPolicy() && label.find("rodada")!=std::string::npos, "previa carrega a rede do motor (a media)");
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
    require(difference<1e-9f,"a previa usa a media: o ruido de exploracao do treino nao entra");
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
    std::cout << "exercicios:";
    for (const std::string& drill : screen.drills()) std::cout << ' ' << drill;
    std::cout << '\n';
    std::cout << "levantar: " << screen.getUpLiveText() << '\n';
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
        // (04/11 12h40: as linhas do minimo para descer a forca.)
        screen.setShowRequirements(std::getenv("MATTER_RL_TEST_EXIGENCIAS") != nullptr);
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
    const auto socket = data / "controle.sock";
    const std::string script = "import json,os,signal,socket,sys,threading,time\n"
        "p=sys.argv[1]\n"
        "def w(st): open(p+'/status.json','w').write(json.dumps({'run':'rast_teste','task':'rastreador_frente','state':st,'pid':os.getpid(),'iteration':7}))\n"
        "signal.signal(signal.SIGTERM, lambda *a: (w('parado'), sys.exit(0)))\n"
        "s=socket.socket(socket.AF_UNIX); s.bind(sys.argv[2]); s.listen()\n"
        "def serve():\n"
        " while True:\n"
        "  c,_=s.accept()\n"
        "  with c:\n"
        "   row=json.loads(c.makefile().readline())\n"
        "   if row['cmd']=='salvar': open(p+'/salvo_pelo_socket','w').write('ok')\n"
        "   c.sendall((json.dumps({'ok':True,'status':{'run':'outro_treino','task':'locomocao','state':'treinando','pid':os.getpid(),'iteration':999}})+'\\n').encode())\n"
        "threading.Thread(target=serve,daemon=True).start()\n"
        "w('treinando')\n"
        "while True: time.sleep(0.05)\n";
    // (Com guarda, como o levantar: abrir apaga o arquivo de parar dele e parar o cria, para o guarda nao religar.)
    const std::filesystem::path stop = run / "parar_guarda";
    std::ofstream(stop).put('\n');
    nlohmann::json config { { "kind", "rastreador" }, { "task", "rastreador_frente" },
        { "comando", { "/usr/bin/python3", "-c", script, run.string(), socket.string(), "--treino", "rast_teste" } },
        { "pasta", run.string() }, { "arquivoParar", stop.string() } };
    std::ofstream(run / "run.json") << config.dump();
    TrainerPaths3D paths = defaultTrainerPaths3D(sourceRoot());
    paths.data = data;
    paths.socket = socket;
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
    require(!std::filesystem::exists(stop), "abrir apaga o arquivo de parar do guarda");
    require(waitFor(true), "o treinador aberto aparece vivo pelo pid do status.json");
    require(link.status().pid > 0 && link.status().iteration == 7 && link.status().state == "treinando",
        "o status do treinador de processo proprio chega a tela");
    require(link.status().run == "rast_teste", "o socket de outro treino nao substitui o selecionado");
    {
        TrainerLink3D controlled(paths, "outro_treino");
        for (int i = 0; i < 100 && !controlled.status().alive; ++i)
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        require(controlled.status().alive && controlled.status().pid == 0,
            "treino controlado pelo socket nao usa sinais de processo proprio");
        controlled.send("salvar");
        for (int i = 0; i < 100 && !std::filesystem::exists(run / "salvo_pelo_socket"); ++i)
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        require(std::filesystem::exists(run / "salvo_pelo_socket"), "salvar segue pelo socket sem matar o treinador");
    }
    link.send("parar");
    require(waitFor(false), "o parar fecha o treinador pelo sinal");
    require(link.status().state == "parado", "parado depois do sinal");
    require(std::filesystem::exists(stop), "parar cria o arquivo de parar do guarda");
    std::cout << "treinador de processo proprio: abre, aparece vivo, para pelo sinal\n";
}

void selectedGetUpRun(const std::filesystem::path& data, FakeCanvas& canvas) {
    const auto run = data / "runs" / "ab_levantar_teste";
    std::filesystem::create_directories(run);
    std::ofstream(run / "run.json") << R"({"kind":"levantar","v2":{"locomocao":0.2,"rastreio":true,"habilidades_separadas":true}})";
    std::ofstream(run / "tarefa.json") << R"({"schema":"matter-getup-amp-1"})";
    std::ofstream(run / "status.json") << R"({"run":"ab_levantar_teste","task":"levantar","state":"parado","iteration":123})";
    std::ofstream(run / "log.jsonl")
        << R"({"atualizacao":123,"hora":1000,"tarefa":0.5,"queda_de_pe":0.8,"cenarios":{"levantar_fixo_corredor":{"levantou":0.25,"tempo":2},"levantar_fixo_corredor_leve":{"levantou":0.9,"tempo":1},"evitar_fixo_pilares":{"evitou":0.6,"recompos_s":0.8},"lev_trote_0":{"sem_cair":0.95,"ate_o_fim":0.95,"vel_rumo":0.85},"lev_trote_7":{"sem_cair":0.5,"ate_o_fim":0.5,"vel_rumo":1.1}}})" << '\n'
        << R"({"atualizacao":123,"hora":1001,"aval":true,"levantou":0.75,"de_pe":0.75,"tempo":1.2})" << '\n';
    TrackingGetUpAmpSettings3D settings;
    settings.assetsDirectory = (sourceRoot() / "assets").string();
    settings.environments = 1;
    settings.threads = 1;
    TrackingGetUpAmpTraining3D batch(settings);
    writePolicy(run / "exports/policy_latest.mrlpolicy", 123, batch.observationNames(), batch.actionSize());
    require(TrackingViewer3D::getUpLog(run) == run / "log.jsonl", "o log vem do braco selecionado");
    require(TrackingViewer3D::getUpReady(run) && TrackingViewer3D::getUpVersion2(run), "o braco aceita os exercicios v2");
    require(!TrackingViewer3D::getUpTraining(run), "log recente de treino encerrado nao aparece como treinando");
    TrackingViewer3D viewer(sourceRoot() / "assets", 7, TrackingViewer3D::GetUpDrill, {}, 1.0f, 0.2f, {}, run);
    std::string label;
    viewer.setPolicy({}, label);
    require(viewer.hasPolicy() && label == "ab_levantar_teste r123", "a politica vem do braco selecionado");
    writePolicy(run / "exports/policy_latest.mrlpolicy", 124, batch.observationNames(), batch.actionSize());
    std::filesystem::last_write_time(run / "exports/policy_latest.mrlpolicy",
        std::filesystem::file_time_type::clock::now() + std::chrono::seconds(2));
    viewer.setPolicy({}, label);
    require(label == "ab_levantar_teste r124", "o visualizador recarrega o novo export do mesmo braco");
    ReinforcementLearningScreen::Options options;
    options.sourceRoot = sourceRoot();
    options.run = "ab_levantar_teste";
    ReinforcementLearningScreen screen(options, {});
    screen.waitUntilLoaded();
    frame(screen, canvas, 180);
    require(screen.shownRun() == "ab_levantar_teste" && screen.policyLoaded(), "a tela conserva o braco selecionado");
    require(screen.getUpLiveText().find("ab_levantar_teste") != std::string::npos
            && screen.getUpLiveText().find("75%") != std::string::npos,
        "o resumo usa a avaliacao mais recente do braco selecionado");
    require(std::find(screen.drills().begin(), screen.drills().end(), TrackingViewer3D::BalanceDrill) != screen.drills().end(),
        "o laboratorio oferece equilibrio parado");
    for (int heading = 0; heading < 8; ++heading) {
        const std::string drill = "lev_trote_" + std::to_string(heading);
        const auto drills = screen.drills();
        require(std::find(drills.begin(), drills.end(), drill) != drills.end()
                && TrackingViewer3D::isGetUpDrill(drill) && TrackingViewer3D::isLocomotionDrill(drill)
                && TrackingViewer3D::locomotionHeadingOf(drill) == heading,
            "oito trotes independentes no laboratorio e no visualizador");
    }
    {
        const auto drills = screen.drills();
        require(std::find(drills.begin(), drills.end(), std::string(TrackingViewer3D::LocomotionDrills[1])) != drills.end(),
            "o rastreio mostra o giro parado");
    }
    TrackingViewer3D independent(sourceRoot() / "assets", 7, "lev_trote_0", {}, 1.0f, 0.2f, {}, run);
    independent.setPolicy({}, label);
    for (int step = 0; step < 10; ++step) independent.step();
    require(independent.hasPolicy() && !independent.drivingLocomotion(), "ator unificado funciona sem arquivos do professor");
    require(std::abs(independent.view().command.moveVelocity.length() - 3.0f) < .01f,
        "o trote do rastreio pede a velocidade do clipe jog (3 m/s), nao a antiga fixa 2,5 m/s");
    {
        // (04/11 12h40: os pedidos fixos do run.json - o trote 4 m/s, a corrida 6 na frente, 4,5 de lado e 5,5 de costas.)
        const auto fixed = data / "runs" / "ab_levantar_velocidades";
        std::filesystem::create_directories(fixed / "exports");
        std::filesystem::copy_file(run / "exports/policy_latest.mrlpolicy", fixed / "exports/policy_latest.mrlpolicy",
            std::filesystem::copy_options::overwrite_existing);
        std::ofstream(fixed / "run.json") << R"({"kind":"levantar","v2":{"locomocao":0.2,"rastreio":true,"habilidades_separadas":true,)"
                                             R"("velocidades_trote":[4,4,4,4,4,4,4,4],"velocidades_corrida":[6,6,4.5,5.5,5.5,5.5,4.5,6]}})";
        for (const auto& [drill, wanted] : { std::pair { "lev_trote_2", 4.0f }, std::pair { "lev_corrida_0", 6.0f },
                 std::pair { "lev_corrida_6", 4.5f }, std::pair { "lev_corrida_4", 5.5f } }) {
            TrackingViewer3D viewer(sourceRoot() / "assets", 7, drill, {}, 1.0f, 0.2f, {}, fixed);
            std::string name;
            viewer.setPolicy({}, name);
            for (int step = 0; step < 10; ++step) viewer.step();
            require(std::abs(viewer.view().command.moveVelocity.length() - wanted) < .01f,
                std::string("o visualizador pede o pedido fixo do treino em ") + drill + ": "
                    + std::to_string(viewer.view().command.moveVelocity.length()));
        }
    }
    screen.showDrill("lev_trote_0");
    require(screen.getUpLiveText().find("cenário 95%") != std::string::npos, "trote frontal usa metricas proprias");
    screen.showDrill("lev_trote_7");
    require(screen.getUpLiveText().find("cenário 50%") != std::string::npos, "trote diagonal nao reaproveita grafico frontal");
    screen.showDrill("levantar_fixo_corredor");
    require(screen.getUpLiveText().find("cenário 25%") != std::string::npos, "grafico do corredor justo usa somente suas medidas");
    screen.showDrill("levantar_fixo_corredor_leve");
    require(screen.getUpLiveText().find("cenário 90%") != std::string::npos, "troca de exercicio invalida cache mesmo sem mudar arquivo");
    screen.showDrill("evitar_fixo_pilares");
    require(screen.getUpLiveText().find("cenário 60%") != std::string::npos, "grafico do evitar usa taxa do seu preset");
    screen.showDrill("levantar_fixo_rampa");
    require(screen.getUpLiveText().find("cenário sem dados") != std::string::npos, "sem metricas individuais nao exibe agregado de outro exercicio");
}

// O empurrao unico no visualizador (23/10, como no treino): o exercicio leve
// recomeca settleSeconds (o da tarefa; 5 s desde 24/10) depois do fim do
// empurrao, que comeca na janela da tarefa (1,5 a 3 s) e dura 0,1 a 0,2 s.
// O levantar v2 no visualizador (31/10 17h): o levantar em movimento mostra o alvo e o pedido, o evitar a queda mostra
// o empurrao forte, e o levantar do chao segue sem alvo (a rede do treino, se houver; sem ela, os alvos zerados).
void checkGetUpViewers() {
    const auto assets = sourceRoot() / "assets";
    {
        TrackingViewer3D moving(assets, 3, TrackingViewer3D::MovingGetUpDrill);
        bool target = false, asked = false;
        for (int i = 0; i < 90; ++i) {
            moving.step();
            const BodyWorldView3D view = moving.view();
            target = target || view.hasTarget;
            asked = asked || std::hypot(view.command.moveVelocity.x, view.command.moveVelocity.y) > 1.0f;
        }
        require(target && asked, "o levantar em movimento mostra o alvo e o pedido");
    }
    {
        TrackingViewer3D avoid(assets, 5, TrackingViewer3D::AvoidFallDrill);
        bool pushed = false;
        for (int i = 0; i < 60 && !pushed; ++i) {
            avoid.step();
            pushed = avoid.view().pushLink >= 0;
        }
        require(pushed, "o evitar a queda mostra o empurrao forte");
    }
    {
        TrackingViewer3D balance(assets, 9, TrackingViewer3D::BalanceDrill);
        bool pushed = false, target = false;
        for (int i = 0; i < 100 && !pushed; ++i) {
            balance.step();
            pushed = balance.view().pushLink >= 0;
            target = target || balance.view().hasTarget;
        }
        require(!pushed && !target, "o equilibrio parado pratica IDLE sem empurrao nem alvo");
    }
    {
        TrackingViewer3D still(assets, 7, TrackingViewer3D::GetUpDrill);
        for (int i = 0; i < 10; ++i) still.step();
        require(!still.view().hasTarget, "o levantar do chao nao tem alvo");
    }
    {
        // As quedas da locomocao (01/11): as do arquivo do treino ou, sem ele, as sorteadas; sem alvo.
        TrackingViewer3D falls(assets, 3, TrackingViewer3D::LocomotionFallDrill);
        for (int i = 0; i < 10; ++i) falls.step();
        require(!falls.view().hasTarget, "as quedas da locomocao nao tem alvo");
    }
    {
        // A locomocao na rede do levantar: o rumo de costas pede a velocidade do clipe jog_backward (3 m/s; 03/11, os
        // clipes do modo padrao no lugar da regua fixa de 2,5 m/s) de costas com o rumo do corpo pedido, o giro gira o
        // rumo pedido sem pedir deslocamento, o parado nao pede nada nem empurra.
        TrackingViewer3D back(assets, 4, "lev_tras");
        back.step();
        const BodyWorldView3D asked = back.view();
        const Vec3 move = asked.command.moveVelocity;
        require(std::abs(std::hypot(move.x, move.y) - 3.0f) < 0.01f
                && std::cos(std::atan2(move.y, move.x) - asked.command.faceHeading) < -0.99f,
            "o rumo para tras pede 3 m/s (jog_backward) de costas");
        TrackingViewer3D turn(assets, 5, "lev_giro");
        turn.step();
        const float start = turn.view().command.faceHeading;
        for (int i = 0; i < 30; ++i) turn.step();
        const BodyWorldView3D turned = turn.view();
        const float angle = std::abs(std::remainder(turned.command.faceHeading - start, 6.2831853f));
        require(angle > 0.55f && angle < 0.65f && std::hypot(turned.command.moveVelocity.x, turned.command.moveVelocity.y) < 1e-6f,
            "o giro gira o rumo pedido a 0,6 rad/s, sem deslocamento (girou " + std::to_string(angle) + " rad em 1 s)");
        TrackingViewer3D still(assets, 6, "lev_parado");
        bool pushed = false;
        for (int i = 0; i < 120 && !pushed; ++i) {
            still.step();
            pushed = still.view().pushLink >= 0;
        }
        const Vec3 none = still.view().command.moveVelocity;
        require(!pushed && !still.view().hasTarget && std::hypot(none.x, none.y) < 1e-6f, "o parado nao pede nem empurra");
    }
    const auto checkPillarSpace = [&](const auto& view, bool avoid) {
        std::vector<Vec3> centers;
        const Vec3 root = view.state->links.front().position;
        for (std::size_t offset = 0; offset < view.scenery.size(); offset += 12) {
            Vec3 center;
            for (std::size_t edge = offset; edge < offset + 12; ++edge)
                center += view.scenery[edge].first + view.scenery[edge].second;
            center = center / 24.0f;
            require(std::hypot(center.x - root.x, center.y - root.y) > (avoid ? 0.95f : 0.20f),
                "pilar afastado do jogador para permitir a queda");
            for (const Vec3 other : centers)
                require(std::hypot(center.x - other.x, center.y - other.y) > (avoid ? 1.0f : 0.20f),
                    "pilares separados em vez de uma gaiola junto ao jogador");
            centers.push_back(center);
        }
    };
    for (std::size_t i = 0; i < 5; ++i) {
        const char* getUp = TrackingViewer3D::ControlledGetUpDrills[i];
        const char* avoid = TrackingViewer3D::ControlledAvoidDrills[i];
        require(TrackingViewer3D::isGetUpDrill(getUp) && TrackingViewer3D::isGetUpDrill(avoid)
                && TrackingViewer3D::sceneryPresetOf(getUp) == static_cast<int>(i)
                && TrackingViewer3D::sceneryPresetOf(avoid) == static_cast<int>(i)
                && !TrackingViewer3D::isAvoidSceneryDrill(getUp) && TrackingViewer3D::isAvoidSceneryDrill(avoid),
            "os cinco layouts controlados preservam tipo e indice");
        TrackingViewer3D controlled(sourceRoot() / "assets", 42, getUp);
        const auto controlledView = controlled.view();
        const std::size_t expectedBodies = i == 3 ? 1u : i == 4 ? 4u : 0u;
        require(controlledView.otherBodies.size() == expectedBodies, "o laboratorio mostra os corpos do preset sem extras aleatorios");
        require((i < 3) == !controlledView.scenery.empty(), "parede/corredor/rampa visiveis, contato sem obstaculo extra");
        if (i == 0) checkPillarSpace(controlledView, false);
        const std::size_t getUpBoxes[5] { 6, 2, 1, 0, 0 };
        require(controlledView.scenery.size() == 12 * getUpBoxes[i], "quantidade fixa de pilares/corredor/rampa no levantar");
        if (i >= 3) {
            for (int step = 0; step < 185; ++step) controlled.step();
            require(controlled.getUpTally().second == 0 && !controlled.drivingLocomotion(),
                "o contato continua depois de seis segundos sem troca de rede");
        }
        TrackingViewer3D controlledAvoid(sourceRoot() / "assets", 42, avoid);
        require(!controlledAvoid.drivingLocomotion(), "o evitar ambientado preserva o mundo com cenario do rl_final");
        if (i == 4) checkPillarSpace(controlledAvoid.view(), true);
        if (i == 0) {
            const auto walls = controlledAvoid.view().scenery;
            const Vec3 root = controlledAvoid.view().state->links.front().position;
            for (std::size_t offset = 0; offset < walls.size(); offset += 12) {
                Vec3 center;
                for (std::size_t edge = offset; edge < offset + 12; ++edge)
                    center += walls[edge].first + walls[edge].second;
                center = center / 24.0f;
                require(std::hypot(center.x - root.x, center.y - root.y) > 1.3f,
                    "paredes do evitar afastadas para permitir a queda");
            }
        }
        if (i == 1) {
            const auto corridorGap = [](const auto& view) {
                const Vec3 root = view.state->links.front().position;
                const Vec3 axis = (view.scenery[0].second - view.scenery[0].first).normalized();
                const float half = (view.scenery[0].second - view.scenery[0].first).length() * 0.5f;
                Vec3 center;
                for (std::size_t edge = 0; edge < 12; ++edge)
                    center += view.scenery[edge].first + view.scenery[edge].second;
                center = center / 24.0f;
                float reach = 0.0f;
                for (const auto& link : view.state->links) reach = std::max(reach, dot(link.position - root, axis));
                return dot(center - root, axis) - half - reach;
            };
            const auto corridorWidth = [](const auto& view) {
                const Vec3 axis = (view.scenery[0].second - view.scenery[0].first).normalized();
                Vec3 centers[2];
                for (int box = 0; box < 2; ++box)
                    for (std::size_t edge = 12 * box; edge < 12 * (box + 1); ++edge)
                        centers[box] += (view.scenery[edge].first + view.scenery[edge].second) / 24.0f;
                const float thickness = (view.scenery[0].second - view.scenery[0].first).length();
                return std::abs(dot(centers[1] - centers[0], axis)) - thickness;
            };
            require(std::abs(corridorWidth(controlledView) - 1.30f) < 1e-4f, "largura fisica do corredor justo e1.30m");
            require(std::abs(corridorGap(controlledAvoid.view()) - 0.30f) < 1e-4f, "corredor do evitar tem folga restaurada");
            const char* looseDrill = TrackingViewer3D::ControlledGetUpDrills[5];
            require(TrackingViewer3D::isGetUpDrill(looseDrill) && TrackingViewer3D::sceneryPresetOf(looseDrill) == 5
                && !TrackingViewer3D::isAvoidSceneryDrill(looseDrill), "variante leve e exclusiva do levantar");
            TrackingViewer3D loose(sourceRoot() / "assets", 42, looseDrill);
            const auto looseView = loose.view();
            require(looseView.scenery.size() == 24 && looseView.otherBodies.empty(), "corredor leve tem somente duas paredes");
            require(std::abs(corridorWidth(looseView) - 1.36f) < 1e-4f, "largura fisica do corredor folgado e1.36m");
            for (const int seed : { 7, 99, 123 }) {
                controlled.reset(seed);
                loose.reset(seed);
                require(std::abs(corridorWidth(controlled.view()) - 1.30f) < 1e-4f
                        && std::abs(corridorWidth(loose.view()) - 1.36f) < 1e-4f,
                    "largura permanece fixa ao trocar pose e rodada");
            }
            const Vec3 tightEdge = controlledView.scenery[0].second - controlledView.scenery[0].first;
            const Vec3 looseEdge = looseView.scenery[0].second - looseView.scenery[0].first;
            require((tightEdge - looseEdge).length() < 1e-4f, "variantes compartilham espessura e orientacao das paredes");
        }
        const std::size_t avoidBoxes[5] { 4, 2, 1, 0, 6 };
        require(controlledAvoid.view().scenery.size() == 12 * avoidBoxes[i], "o evitar mostra quatro paredes ou seis pilares conforme preset");
        if (i == 2) {
            for (const auto* view : { &controlled, &controlledAvoid }) {
                const auto segments = view->view().scenery;
                const Vec3 normal = cross(segments[0].second - segments[0].first,
                    segments[1].second - segments[1].first).normalized();
                require(std::abs(normal.z - std::cos(22.0f * 3.14159265f / 180.0f)) < 1e-4f,
                    "rampa do levantar e do evitar tem 22 graus");
                Vec3 center;
                for (const auto& edge : segments) center += edge.first + edge.second;
                center = center / static_cast<float>(2 * segments.size());
                require(std::abs((center + normal).z - 0.8f) < 1e-4f, "plataforma da rampa com altura de 0,8 m");
            }
        }
        require(controlledAvoid.view().otherBodies.size() == (i == 3 ? 4u : 0u),
            "o evitar mostra quatro corpos e o preset de pilares nao tem corpos");
    }
    for (const char* drill : TrackingViewer3D::LocomotionDrills)
        require(TrackingViewer3D::isGetUpDrill(drill), std::string("o exercicio do levantar ") + drill);
    require(TrackingViewer3D::locomotionHeadingOf("lev_diagonal_frente", 0) == 1
            && TrackingViewer3D::locomotionHeadingOf("lev_diagonal_frente", 1) == 7
            && TrackingViewer3D::locomotionHeadingOf("lev_giro") < 0,
        "os rumos das diagonais pelo lado da semente");
    require(TrackingViewer3D::isGetUpDrill("levantar_movimento") && TrackingViewer3D::isGetUpDrill("evitar_queda")
            && TrackingViewer3D::isGetUpDrill("equilibrio_parado") && TrackingViewer3D::isGetUpDrill("quedas_locomocao")
            && !TrackingViewer3D::isGetUpDrill("parado"),
        "os exercicios do levantar");
}

// A folha de um exercicio do levantar com as redes de verdade (MATTER_RL_TEST_FOLHA=<pasta>, MATTER_RL_TEST_DRILL): o
// dump dos quadros do visualizador para o MatterModelInspector quadros e a linha do tempo de quem dirige.
int dumpGetUpViewer(const std::filesystem::path& out, const std::string& drill) {
    const std::filesystem::path assets = sourceRoot() / "assets";
    // (MATTER_RL_TEST_RUN=<pasta do treino>: outro treino que nao a rl_final - 03/11, o rastreio.)
    const char* run = std::getenv("MATTER_RL_TEST_RUN");
    TrackingViewer3D viewer(assets, 21, drill, {}, 1.0f, 0.2f, {}, run ? std::filesystem::path(run) : std::filesystem::path());
    std::string label;
    viewer.setPolicy({}, label);
    // (07/11: MATTER_RL_TEST_CINEMATICO=1, os exercicios pelo Motion Matching do juiz, sem fisica.)
    if (std::getenv("MATTER_RL_TEST_CINEMATICO")) {
        viewer.setKinematicMotionMatching(true);
        if (!viewer.kinematicError().empty()) std::cout << "cinematico: " << viewer.kinematicError() << '\n';
    }
    const RagdollProfile3D& profile = viewer.profile();
    std::filesystem::create_directories(out);
    std::ofstream dump(out / (drill + ".dump"));
    dump << "S " << drill << '\n';
    std::string who;
    const int decisions = std::atoi(std::getenv("MATTER_RL_TEST_DECISOES") ? std::getenv("MATTER_RL_TEST_DECISOES") : "300");
    for (int i = 0; i < decisions; ++i) {
        viewer.step();
        if (std::getenv("MATTER_RL_TEST_CLIPES") && (viewer.kinematicMotionMatching() || viewer.clipSequence()))
            std::cout << "  " << static_cast<float>(i + 1) / 30.0f << " s: " << viewer.kinematicClip() << " (trocas " << viewer.kinematicJumps() << ")\n";
        const std::string now = viewer.drivingLocomotion() ? "locomocao (parceira)"
            : viewer.locomotionHeadDriving() ? "cabeca da locomocao" : "levantar";
        if (now != who) {
            std::cout << "t " << static_cast<float>(i + 1) / 30.0f << " s: rede da " << now << '\n';
            who = now;
        }
        if (i == 60 || i == 150) std::cout << "marcas do caminho em " << static_cast<float>(i + 1) / 30.0f << " s: "
                                           << viewer.view().starPoints.size() << '\n';
        if (i % 2 != 0) continue;
        const RagdollState3D& body = *viewer.view().state;
        dump << "F " << static_cast<float>(i + 1) / 30.0f << " 0\n";
        for (std::size_t k = 0; k < profile.links.size() && k < body.links.size(); ++k) {
            const RagdollLinkDefinition3D& link = profile.links[k];
            const PhysicsBodyState3D& state = body.links[k];
            const Vec3 center = state.position + state.orientation.rotate(link.collider.localPosition);
            const Quaternion q = (state.orientation * link.collider.localOrientation).normalized();
            dump << (link.collider.shape == RagdollColliderShape3D::Box ? "B " : "C ") << link.id << ' ' << center.x << ' '
                 << center.y << ' ' << center.z << ' ' << q.x << ' ' << q.y << ' ' << q.z << ' ' << q.w << '\n';
        }
    }
    const auto [good, total] = viewer.getUpTally();
    std::cout << drill << ": " << good << " de " << total << " (" << label << ")\n";
    return 0;
}

// O giro parado do modo padrao (04/11 09h40, ele: "o giro parado deveria ser feito do IDLE alerta, e ja temos os
// movimentos de giro do footplanner, que ele DEVERIA usar como referencia"): o lider do rastreador (o modo padrao de
// verdade, DefaultLocomotion3D com o StandTurnFeet3D) parado no idle e girando a vista a MATTER_RL_TEST_GIRO_TAXA graus/s
// (+ esquerda, - direita); grava a 120 Hz a raiz no espaco do clipe (o deslocamento e o delta da rotacao, a regra de
// TrackingClipReference3D), as coordenadas de junta e os pes no mundo. tools/animation/clips/giro_idle.py corta o ciclo.
int captureStandTurn(const std::filesystem::path& out) {
    const std::filesystem::path assets = sourceRoot() / "assets";
    const auto bundle = AthleteBundle3D::shared(assets.string());
    const RagdollProfile3D& profile = bundle->profile();
    // (Os do laboratorio, como o labLeader do TrackingTraining3D.)
    AthleteRuntimeSettings3D runtime;
    runtime.physicalFeel = 1;
    runtime.defaultStartGesturePercent = 100.0f;
    runtime.capsuleResponsePercent = 150.0f;
    runtime.capsuleReversalPercent = 100.0f;
    runtime.defaultTrotSpeed = 4.5f;
    runtime.defaultSprintForwardSpeed = 8.0f;
    runtime.defaultSprintSideSpeed = 6.0f;
    runtime.defaultSprintBackwardSpeed = 6.0f;
    runtime.defaultMaxSprintSpeed = 10.0f;
    TrackingLeader3D leader(profile, *bundle->assets().defaultClips, runtime);
    leader.reset({}, 0.0f);
    const float rate = (std::getenv("MATTER_RL_TEST_GIRO_TAXA") ? std::strtof(std::getenv("MATTER_RL_TEST_GIRO_TAXA"), nullptr) : 90.0f)
        * 3.14159265f / 180.0f;
    std::size_t root = 0;
    std::array<std::size_t, 2> feet { 0, 0 };
    for (std::size_t i = 0; i < profile.links.size(); ++i) {
        if (profile.links[i].parentIndex < 0) root = i;
        if (profile.links[i].id == "LeftFoot") feet[0] = i;
        if (profile.links[i].id == "RightFoot") feet[1] = i;
    }
    const RagdollLinkDefinition3D& pelvis = profile.links[root];
    std::ofstream file(out);
    nlohmann::json head;
    head["links"] = nlohmann::json::array();
    for (const RagdollLinkDefinition3D& link : profile.links) head["links"].push_back(link.id);
    head["raiz"] = root;
    head["taxa"] = rate;
    file << head.dump() << '\n';
    const float dt = 1.0f / 120.0f;
    float facing = 0.0f;
    for (int k = 0; k < 120 * 12; ++k) {
        const float t = static_cast<float>(k) * dt;
        if (t >= 2.0f) facing += rate * dt;
        leader.step({ {}, facing }, dt);
        const TrackingLeaderState3D& s = leader.state();
        if (!s.valid) continue;
        // (A inversa de TrackingClipReference3D::pose no chao 0 e rumo 0.)
        const Vec3 offset = s.rootPosition - Vec3 { 0.0f, 0.0f, profile.standingRootHeightMeters } - pelvis.modelPosition;
        const Quaternion delta = (pelvis.modelOrientation.conjugate() * s.rootOrientation).normalized();
        nlohmann::json row;
        row["t"] = t;
        row["vista"] = facing;
        row["deslocamento"] = { offset.x, offset.y, offset.z };
        row["delta"] = { delta.x, delta.y, delta.z, delta.w };
        row["coordenadas"] = nlohmann::json::array();
        for (const Vec3& c : s.coordinates) row["coordenadas"].push_back({ c.x, c.y, c.z });
        row["pes"] = nlohmann::json::array();
        for (std::size_t f : feet) {
            const PhysicsBodyState3D& l = s.links[f];
            row["pes"].push_back({ l.position.x, l.position.y, l.position.z, l.linearVelocity.x, l.linearVelocity.y, l.linearVelocity.z });
        }
        row["links"] = nlohmann::json::array();
        for (const PhysicsBodyState3D& l : s.links) row["links"].push_back({ l.position.x, l.position.y, l.position.z });
        file << row.dump() << '\n';
    }
    std::cout << "giro capturado: " << out << '\n';
    return 0;
}

void checkGetUpHandoffViewer() {
    // A interligacao das redes no visualizador do B21 (01/11): na queda, a rede do levantar exportada assume e devolve
    // de pe e firme antes do recomeco de 8 s (sem ela exportada nesta maquina, sem o teste).
    if (!std::filesystem::exists(TrackingViewer3D::getUpExport())) {
        std::cout << "visualizador: sem a rede do levantar exportada, a interligacao fica sem teste\n";
        return;
    }
    const std::filesystem::path task = sourceRoot() / "assets" / "rl" / "tasks" / "locomocao_b21.json";
    LocomotionViewer3D viewer(task, sourceRoot() / "assets", "parado", 3);
    viewer.push({ 2500.0f, 0.0f, 0.0f }, 0.3f);
    int took = -1, back = -1;
    for (int i = 0; i < 30 * 14 && back < 0; ++i) {
        viewer.step();
        if (took < 0 && viewer.gettingUp()) took = i;
        if (took >= 0 && !viewer.gettingUp()) back = i;
    }
    require(took >= 0 && back > took && back - took < 30 * 8,
        "na queda a rede do levantar assume e devolve antes de 8 s (assumiu " + std::to_string(took) + ", devolveu "
            + std::to_string(back) + ")");
    std::cout << "visualizador: a rede do levantar assumiu na decisao " << took << " e devolveu em "
              << static_cast<float>(back - took) / 30.0f << " s\n";
}

void checkSinglePushViewer() {
    const std::filesystem::path task = sourceRoot() / "assets" / "rl" / "tasks" / "locomocao_e1.json";
    float settle = 3.0f;
    {
        std::ifstream file(task);
        const nlohmann::json json = nlohmann::json::parse(file);
        for (const nlohmann::json& drill : json["drills"])
            if (drill.value("id", std::string()) == "parado_empurrao") settle = drill.value("settleSeconds", settle);
    }
    LocomotionViewer3D viewer(task, sourceRoot() / "assets", "parado_empurrao", 7);
    float first = -1.0f;
    for (int k = 0; k < 400 && first < 0.0f; ++k) {
        const std::uint32_t before = viewer.view().steps;
        viewer.step();
        if (viewer.view().steps < before) first = static_cast<float>(before + 1) * viewer.policyStepSeconds();
    }
    require(first >= 1.5f + 0.1f + settle - 0.05f && first <= 3.0f + 0.2f + settle + 0.07f,
        "o visualizador recomeca " + std::to_string(settle) + " s depois do empurrao unico (recomecou em "
            + std::to_string(first) + " s)");
    std::cout << "visualizador: empurrao unico, recomeca em " << first << " s\n";
}

} // namespace

int main() {
    try {
        if (const char* real = std::getenv("MATTER_RL_TEST_REAL")) return realRun(real);
        if (const char* turn = std::getenv("MATTER_RL_TEST_GIRO")) return captureStandTurn(turn);
        if (const char* sheet = std::getenv("MATTER_RL_TEST_FOLHA"))
            return dumpGetUpViewer(sheet, std::getenv("MATTER_RL_TEST_DRILL") ? std::getenv("MATTER_RL_TEST_DRILL") : "levantar");
        const std::filesystem::path sandbox = std::filesystem::temp_directory_path() / "matter_rl_tela";
        std::filesystem::remove_all(sandbox);
        std::filesystem::create_directories(sandbox / "runtime");
        setenv("MATTER_RL_DATA", (sandbox / "dados").c_str(), 1);
        setenv("XDG_RUNTIME_DIR", (sandbox / "runtime").c_str(), 1);
        unsetenv("MATTER_RL_SOCKET");
        require(!LocomotionViewer3D::isLocomotionTask(sandbox / "ausente.json"), "tarefa ausente nao pode derrubar a tela");
        checkSkillRoutingAndIdle(sandbox / "skills");
        checkSinglePushViewer();
        checkGetUpHandoffViewer();
        checkGetUpViewers();
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
        selectedGetUpRun(sandbox / "dados", canvas);
        require(ui.errors().empty(), "o ImGui reclamou: " + (ui.errors().empty() ? std::string() : ui.errors().front()));
        std::filesystem::remove_all(sandbox);
        std::cout << "Reinforcement learning screen tests passed (" << checks << " checks)\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "Reinforcement learning screen tests failed: " << error.what() << '\n';
        return 1;
    }
}
