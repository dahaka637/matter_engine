#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <filesystem>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace MatterEngine {

// Onde o treinador de RL vive no PC (as mesmas regras do tools/rl/matter_rl/paths.py).
struct TrainerPaths3D {
    std::filesystem::path data;      // ~/.local/share/matter-engine/rl (ou MATTER_RL_DATA)
    std::filesystem::path database;  // data/rl.db
    std::filesystem::path socket;    // $XDG_RUNTIME_DIR/matter-rl/control.sock (ou MATTER_RL_SOCKET)
    std::filesystem::path launcher;  // <repo>/tools/rl/matter-rl

    [[nodiscard]] std::filesystem::path run(const std::string& name) const { return data / "runs" / name; }
};

[[nodiscard]] TrainerPaths3D defaultTrainerPaths3D(const std::filesystem::path& sourceRoot);

// O que o treinador conta de si (o "status" do canal de controle).
struct TrainerStatus3D {
    // O treinador respondeu agora; sem ele, os campos vem do status.json do
    // treino (o ultimo estado conhecido).
    bool alive = false;
    bool known = false;
    std::string run;
    std::string task;
    std::string taskName;
    std::string state;
    std::string mode;
    std::string modeLabel;
    std::string device;
    std::string message;
    std::uint64_t iteration = 0;
    std::uint64_t steps = 0;
    std::uint64_t episodes = 0;
    double sps = 0.0;
    double trainSeconds = 0.0;
    int threads = 0;
    int environments = 0;
    double lastReward = 0.0;
    double lastEpisodeSeconds = 0.0;
    double lastFallFraction = -1.0;
    double cpuTemperature = 0.0;
    // O peso do juiz de estilo (beta; < 0 sem juiz) e a nota media dele na
    // ultima rodada.
    double styleBeta = -1.0;
    double lastStyle = 0.0;
    // A ultima avaliacao: as medidas de cada exercicio em cada solver.
    struct DrillEvaluation {
        std::string drill;
        std::string solver;
        double falls = 0.0;
        double velocityError = -1.0;
        double usefulSteps = -1.0;
        double sustained = -1.0;
        double slip = -1.0;
    };
    bool evaluated = false;
    std::uint64_t evaluationIteration = 0;
    std::vector<DrillEvaluation> evaluation;
    std::string policy;
    // Um treinador de processo proprio (o rastreador, sem o supervisor): o pid
    // dele; parar e salvar vao por sinal.
    int pid = 0;
};

// A ponte da tela com o treinador: abre o processo (que sobrevive a tela
// fechada), manda comandos e pergunta o status a cada segundo, numa thread
// propria (a interface nunca espera o socket).
class TrainerLink3D final {
public:
    TrainerLink3D(TrainerPaths3D paths, std::string run);
    ~TrainerLink3D();
    TrainerLink3D(const TrainerLink3D&) = delete;
    TrainerLink3D& operator=(const TrainerLink3D&) = delete;

    [[nodiscard]] const TrainerPaths3D& paths() const { return m_paths; }
    [[nodiscard]] const std::string& run() const { return m_run; }
    [[nodiscard]] TrainerStatus3D status() const;
    // Abre o supervisor do treino (tools/rl/matter-rl servir) em segundo
    // plano, com a saida no treinador.log do treino. run vazio = o desta ponte.
    bool start(const std::string& task, const std::string& mode, std::string* error, const std::string& run = {});
    // Roda um comando curto do treinador (matter-rl <argumentos>) sem esperar.
    bool runLauncher(const std::vector<std::string>& arguments, std::string* error);
    // O servico que retoma o treino ao ligar o PC esta ligado.
    [[nodiscard]] bool resumesAtBoot() const;
    // Um comando do canal (pausar, retomar, parar, salvar, avaliar, modo...),
    // com os argumentos em JSON; a resposta chega depois (lastReply).
    void send(const std::string& command, const std::string& argumentsJson = "{}");
    [[nodiscard]] std::string lastReply() const;
    // Pede o status ja (depois de um comando).
    void refresh();

private:
    void worker();
    [[nodiscard]] std::string request(const std::string& line, int timeoutMilliseconds) const;
    void readStatus(const std::string& json, bool alive);
    [[nodiscard]] std::string lastLogLine() const;

    TrainerPaths3D m_paths;
    std::string m_run;
    mutable std::mutex m_mutex;
    std::condition_variable m_wake;
    TrainerStatus3D m_status;
    std::deque<std::string> m_queue;
    std::string m_lastReply;
    std::vector<int> m_children;
    // Abrindo o treinador (o Python e o PyTorch levam alguns segundos).
    std::chrono::steady_clock::time_point m_openedAt {};
    bool m_opening = false;
    // Por que o ultimo treinador aberto por esta tela nao abriu.
    std::string m_failure;
    bool m_refresh = true;
    std::atomic<bool> m_stop { false };
    std::thread m_thread;
};

} // namespace MatterEngine
