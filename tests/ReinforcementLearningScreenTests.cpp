// A tela "Aprendizado por Reforco" sem janela, num treino de mentira (uma
// pasta temporaria no lugar de ~/.local/share/matter-engine/rl e do socket):
// o visualizador roda o jogador de verdade com a politica exportada, os
// empurroes andam, o historico do banco vira grafico, e o ImGui nao reclama.
// MATTERENGINE_UI_SNAPSHOT=<pasta> guarda as fotos da tela.

#include "Engine/RL/Policy/PolicyRuntime3D.hpp"
#include "Support/HeadlessUi.hpp"
#include "Workbench/ReinforcementLearning/ReinforcementLearningScreen.hpp"

#include "imgui.h"

#include <sqlite3.h>

#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <chrono>
#include <thread>
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

// Uma politica de mentira no formato do treinador: pesos zero (a acao zero
// segura a pose base), 160 -> 8 -> 39.
void writePolicy(const std::filesystem::path& path, std::uint64_t iteration) {
    constexpr std::size_t Obs = 160, Hidden = 8, Act = 39;
    std::vector<float> data;
    const std::size_t mean = data.size();
    data.insert(data.end(), Obs, 0.0f);
    const std::size_t inverse = data.size();
    data.insert(data.end(), Obs, 1.0f);
    const std::size_t w1 = data.size();
    data.insert(data.end(), Obs * Hidden, 0.0f);
    const std::size_t b1 = data.size();
    data.insert(data.end(), Hidden, 0.0f);
    const std::size_t w2 = data.size();
    data.insert(data.end(), Hidden * Act, 0.0f);
    const std::size_t b2 = data.size();
    data.insert(data.end(), Act, 0.0f);
    const std::string header = "{\"schema\":\"matter-rl-policy-1\",\"observationSize\":160,\"actionSize\":39,"
                               "\"activation\":\"elu\",\"iteration\":" + std::to_string(iteration)
        + ",\"normalizer\":{\"mean\":" + std::to_string(mean) + ",\"invStd\":" + std::to_string(inverse)
        + ",\"clip\":5.0},\"layers\":[{\"inputs\":160,\"outputs\":8,\"weights\":" + std::to_string(w1)
        + ",\"bias\":" + std::to_string(b1) + "},{\"inputs\":8,\"outputs\":39,\"weights\":" + std::to_string(w2)
        + ",\"bias\":" + std::to_string(b2) + "}]}";
    std::filesystem::create_directories(path.parent_path());
    std::ofstream out(path, std::ios::binary);
    out.write("MRLPOL01", 8);
    const std::uint32_t size = static_cast<std::uint32_t>(header.size());
    out.write(reinterpret_cast<const char*>(&size), 4);
    out.write(header.data(), static_cast<std::streamsize>(header.size()));
    const std::size_t pad = (4 - (12 + header.size()) % 4) % 4;
    for (std::size_t i = 0; i < pad; ++i) out.put('\0');
    out.write(reinterpret_cast<const char*>(data.data()), static_cast<std::streamsize>(data.size() * 4));
}

void writeHistory(const std::filesystem::path& database) {
    sqlite3* db = nullptr;
    require(sqlite3_open(database.string().c_str(), &db) == SQLITE_OK, "banco de mentira");
    const char* sql = R"(
        CREATE TABLE iterations (run TEXT, iteration INTEGER, time REAL, steps INTEGER, sps REAL, reward REAL,
            episodes INTEGER, episode_return REAL, episode_seconds REAL, fall_fraction REAL, components TEXT, ends TEXT,
            policy_loss REAL, value_loss REAL, entropy REAL, kl REAL, clip_fraction REAL, learning_rate REAL,
            action_std REAL, grad_norm REAL, collect_seconds REAL, update_seconds REAL);
        CREATE TABLE evaluations (run TEXT, iteration INTEGER, time REAL, steps INTEGER, battery TEXT, solver TEXT,
            stand_fraction REAL, mean_seconds REAL, sway REAL, foot_drift REAL, action_rate REAL, effort REAL,
            upright REAL, score REAL, ends TEXT);
        CREATE TABLE events (id INTEGER PRIMARY KEY, run TEXT, time REAL, kind TEXT, author TEXT, message TEXT, data TEXT);
    )";
    require(sqlite3_exec(db, sql, nullptr, nullptr, nullptr) == SQLITE_OK, "tabelas do banco de mentira");
    for (int i = 1; i <= 50; ++i) {
        const std::string row = "INSERT INTO iterations(run, iteration, time, steps, sps, reward, episode_seconds, "
                                "fall_fraction) VALUES ('principal', " + std::to_string(i) + ", " + std::to_string(1.0e9 + i)
            + ", " + std::to_string(i * 12288) + ", 20000, " + std::to_string(0.5 + 0.01 * i) + ", "
            + std::to_string(2.0 + 0.3 * i) + ", 0.9);";
        require(sqlite3_exec(db, row.c_str(), nullptr, nullptr, nullptr) == SQLITE_OK, "rodada do banco de mentira");
    }
    for (int i = 1; i <= 5; ++i) {
        for (const char* solver : { "treino", "jogo" }) {
            const std::string row = "INSERT INTO evaluations VALUES ('principal', " + std::to_string(i * 10) + ", "
                + std::to_string(1.0e9 + i * 10) + ", " + std::to_string(i * 122880) + ", 'equilibrio_v1', '" + solver
                + "', " + std::to_string(0.2 * i) + ", 30, 0.02, 0.05, 0.01, 0.1, 0.9, 0.5, '{}');";
            require(sqlite3_exec(db, row.c_str(), nullptr, nullptr, nullptr) == SQLITE_OK, "avaliacao de mentira");
        }
    }
    require(sqlite3_exec(db, "INSERT INTO events(run, time, kind, author, message) VALUES ('principal', 1000000060, "
                             "'peso', 'claude', 'still: 0.3 -> 0.4');",
                nullptr, nullptr, nullptr)
            == SQLITE_OK,
        "evento de mentira");
    sqlite3_close(db);
}

// MATTER_RL_TEST_LIVE=1: a tela no treino de verdade (o treinador aberto no
// PC), so para a foto "rl_tela_ao_vivo". Nada e clicado: o treino nao muda.
int liveSnapshot() {
    TestSupport::HeadlessUi ui;
    FakeCanvas canvas;
    ReinforcementLearningScreen::Options options;
    options.sourceRoot = std::filesystem::path(MATTERENGINE_TEST_ASSETS_DIR).parent_path();
    options.capturesDirectory = std::filesystem::temp_directory_path() / "matter_rl_tela_capturas";
    ReinforcementLearningScreen screen(std::move(options), {});
    screen.waitUntilLoaded();
    for (int i = 0; i < 100 && !screen.status().alive; ++i) {
        frame(screen, canvas, 1);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    require(screen.status().alive, "nenhum treinador aberto no PC");
    screen.advanceViewer(300);
    frame(screen, canvas, 30);
    snapshot(canvas, "rl_tela_ao_vivo");
    require(ui.errors().empty(), "o ImGui reclamou: " + (ui.errors().empty() ? std::string() : ui.errors().front()));
    std::cout << "Reinforcement learning screen live snapshot (" << checks << " checks)\n";
    return 0;
}

} // namespace

int main() {
    try {
        if (std::getenv("MATTER_RL_TEST_LIVE") != nullptr) return liveSnapshot();
        const std::filesystem::path sandbox = std::filesystem::temp_directory_path() / "matter_rl_tela";
        std::filesystem::remove_all(sandbox);
        std::filesystem::create_directories(sandbox / "runtime");
        setenv("MATTER_RL_DATA", (sandbox / "dados").c_str(), 1);
        setenv("XDG_RUNTIME_DIR", (sandbox / "runtime").c_str(), 1);
        // MATTER_RL_TEST_POLICY=<arquivo>: uma politica de verdade (de um treino)
        // no lugar da de mentira, para ver o jogador treinado na tela.
        const char* realPolicy = std::getenv("MATTER_RL_TEST_POLICY");
        const std::filesystem::path policyFile = sandbox / "dados" / "runs" / "principal" / "exports" / "policy_latest.mrlpolicy";
        writePolicy(policyFile, 5);
        writeHistory(sandbox / "dados" / "rl.db");
        {
            PolicyRuntime3D runtime(policyFile);
            std::vector<float> obs(160, 1.0f), act(39, 7.0f);
            runtime.evaluate(obs, act);
            require(act[0] == 0.0f && runtime.info().iteration == 5, "a politica de mentira da acao zero");
        }
        if (realPolicy != nullptr) std::filesystem::copy_file(realPolicy, policyFile,
            std::filesystem::copy_options::overwrite_existing);

        TestSupport::HeadlessUi ui;
        FakeCanvas canvas;
        ReinforcementLearningScreen::Options options;
        options.sourceRoot = std::filesystem::path(MATTERENGINE_TEST_ASSETS_DIR).parent_path();
        options.capturesDirectory = sandbox / "capturas";
        ReinforcementLearningScreen screen(std::move(options), {});
        screen.waitUntilLoaded();
        require(screen.modelLoaded(), "o jogador deveria carregar");
        require(screen.viewerReady(), "o visualizador deveria existir");
        frame(screen, canvas, 40);
        require(canvas.uploads > 0 && canvas.width > 100, "o visualizador deveria desenhar");
        require(!screen.status().alive, "nao ha treinador nesta pasta");
        require(screen.history().iterations.size() == 50 && screen.history().evaluations.size() == 10,
            "o historico do banco deveria aparecer");
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
        frame(screen, canvas, 2);

        // A politica de mentira segura a pose base: depois de 1 s o corpo
        // ainda esta de pe; um empurrao forte o tira do lugar. Com uma politica
        // de verdade, 30 s no visualizador antes da foto.
        screen.advanceViewer(realPolicy != nullptr ? 900 : 30);
        if (realPolicy != nullptr) {
            frame(screen, canvas, 3);
            snapshot(canvas, "rl_tela_politica_30s");
        }
        require(screen.renderViewport(640, 480) && !screen.image().empty(), "o visualizador renderiza sob demanda");
        screen.push(0, 600.0f);
        screen.advanceViewer(10);
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
