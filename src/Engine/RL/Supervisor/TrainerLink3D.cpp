#include "Engine/RL/Supervisor/TrainerLink3D.hpp"

#include <nlohmann/json.hpp>

#include <cerrno>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <fstream>
#include <iterator>
#include <spawn.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <sys/un.h>
#include <sys/wait.h>
#include <unistd.h>

extern char** environ;

namespace MatterEngine {
namespace {

std::filesystem::path environmentPath(const char* name, const std::filesystem::path& fallback) {
    const char* value = std::getenv(name);
    return value != nullptr && *value != '\0' ? std::filesystem::path(value) : fallback;
}

double number(const nlohmann::json& object, const char* key, double fallback = 0.0) {
    const auto it = object.find(key);
    return it != object.end() && it->is_number() ? it->get<double>() : fallback;
}

std::string text(const nlohmann::json& object, const char* key) {
    const auto it = object.find(key);
    return it != object.end() && it->is_string() ? it->get<std::string>() : std::string();
}

} // namespace

TrainerPaths3D defaultTrainerPaths3D(const std::filesystem::path& sourceRoot) {
    TrainerPaths3D paths;
    const std::filesystem::path home = environmentPath("HOME", "/tmp");
    paths.data = environmentPath("MATTER_RL_DATA", home / ".local" / "share" / "matter-engine" / "rl");
    paths.database = paths.data / "rl.db";
    paths.socket = environmentPath("MATTER_RL_SOCKET", environmentPath("XDG_RUNTIME_DIR", "/tmp") / "matter-rl" / "control.sock");
    paths.launcher = sourceRoot / "tools" / "rl" / "matter-rl";
    return paths;
}

TrainerLink3D::TrainerLink3D(TrainerPaths3D paths, std::string run)
    : m_paths(std::move(paths)), m_run(std::move(run)) {
    m_thread = std::thread([this] { worker(); });
}

TrainerLink3D::~TrainerLink3D() {
    m_stop = true;
    m_wake.notify_all();
    if (m_thread.joinable()) m_thread.join();
}

TrainerStatus3D TrainerLink3D::status() const {
    std::lock_guard lock(m_mutex);
    return m_status;
}

std::string TrainerLink3D::lastReply() const {
    std::lock_guard lock(m_mutex);
    return m_lastReply;
}

void TrainerLink3D::send(const std::string& command, const std::string& argumentsJson) {
    nlohmann::json message;
    message["cmd"] = command;
    message["args"] = nlohmann::json::parse(argumentsJson.empty() ? "{}" : argumentsJson, nullptr, false);
    if (message["args"].is_discarded()) message["args"] = nlohmann::json::object();
    {
        std::lock_guard lock(m_mutex);
        m_queue.push_back(message.dump());
        m_refresh = true;
    }
    m_wake.notify_all();
}

void TrainerLink3D::refresh() {
    {
        std::lock_guard lock(m_mutex);
        m_refresh = true;
    }
    m_wake.notify_all();
}

bool TrainerLink3D::runLauncher(const std::vector<std::string>& arguments, std::string* error) {
    if (!std::filesystem::exists(m_paths.launcher)) {
        if (error) *error = "treinador ausente: " + m_paths.launcher.string();
        return false;
    }
    std::vector<std::string> values { m_paths.launcher.string() };
    values.insert(values.end(), arguments.begin(), arguments.end());
    std::vector<char*> argv;
    for (std::string& value : values) argv.push_back(value.data());
    argv.push_back(nullptr);
    posix_spawn_file_actions_t actions;
    posix_spawn_file_actions_init(&actions);
    posix_spawn_file_actions_addopen(&actions, 0, "/dev/null", O_RDONLY, 0);
    posix_spawn_file_actions_addopen(&actions, 1, "/dev/null", O_WRONLY, 0);
    posix_spawn_file_actions_adddup2(&actions, 1, 2);
    posix_spawn_file_actions_addclosefrom_np(&actions, 3);
    pid_t pid = 0;
    const int result = posix_spawn(&pid, values.front().c_str(), &actions, nullptr, argv.data(), environ);
    posix_spawn_file_actions_destroy(&actions);
    if (result != 0) {
        if (error) *error = std::string("nao foi possivel rodar o treinador: ") + std::strerror(result);
        return false;
    }
    std::lock_guard lock(m_mutex);
    m_children.push_back(pid);
    return true;
}

std::string TrainerLink3D::lastLogLine() const {
    std::ifstream log(m_paths.run(m_run) / "treinador.log");
    std::string line, last;
    while (std::getline(log, line))
        if (!line.empty() && line.find("Warning") == std::string::npos) last = line;
    return last.size() > 160 ? last.substr(last.size() - 160) : last;
}

bool TrainerLink3D::resumesAtBoot() const {
    const std::filesystem::path home = environmentPath("HOME", "/tmp");
    std::error_code code;
    return std::filesystem::exists(home / ".config/systemd/user/default.target.wants/matter-rl.service", code);
}

bool TrainerLink3D::start(const std::string& task, const std::string& mode, std::string* error) {
    if (status().alive) return true;
    if (!std::filesystem::exists(m_paths.launcher)) {
        if (error) *error = "treinador ausente: " + m_paths.launcher.string();
        return false;
    }
    const std::filesystem::path directory = m_paths.run(m_run);
    std::error_code code;
    std::filesystem::create_directories(directory, code);
    const std::string log = (directory / "treinador.log").string();
    const std::string launcher = m_paths.launcher.string();
    std::vector<std::string> arguments { launcher, "servir", "--run", m_run, "--tarefa", task, "--modo", mode };
    std::vector<char*> argv;
    for (std::string& argument : arguments) argv.push_back(argument.data());
    argv.push_back(nullptr);

    posix_spawn_file_actions_t actions;
    posix_spawn_file_actions_init(&actions);
    posix_spawn_file_actions_addopen(&actions, 0, "/dev/null", O_RDONLY, 0);
    posix_spawn_file_actions_addopen(&actions, 1, log.c_str(), O_WRONLY | O_CREAT | O_APPEND, 0644);
    posix_spawn_file_actions_adddup2(&actions, 1, 2);
    // O treinador nao herda o que a janela tem aberto (Vulkan, Wayland, audio).
    posix_spawn_file_actions_addclosefrom_np(&actions, 3);
    posix_spawnattr_t attributes;
    posix_spawnattr_init(&attributes);
    // Sessao propria: o treino segue com a tela (e o terminal) fechados.
    posix_spawnattr_setflags(&attributes, POSIX_SPAWN_SETSID);
    pid_t pid = 0;
    const int result = posix_spawn(&pid, launcher.c_str(), &actions, &attributes, argv.data(), environ);
    posix_spawn_file_actions_destroy(&actions);
    posix_spawnattr_destroy(&attributes);
    if (result != 0) {
        if (error) *error = std::string("nao foi possivel abrir o treinador: ") + std::strerror(result);
        return false;
    }
    {
        std::lock_guard lock(m_mutex);
        m_children.push_back(pid);
        m_status.state = "abrindo";
        m_opening = true;
        m_failure.clear();
        m_openedAt = std::chrono::steady_clock::now();
        m_refresh = true;
    }
    m_wake.notify_all();
    return true;
}

std::string TrainerLink3D::request(const std::string& line, int timeoutMilliseconds) const {
    const std::string path = m_paths.socket.string();
    if (path.size() >= sizeof(sockaddr_un::sun_path) || !std::filesystem::exists(m_paths.socket)) return {};
    const int fd = ::socket(AF_UNIX, SOCK_STREAM | SOCK_CLOEXEC, 0);
    if (fd < 0) return {};
    timeval timeout { timeoutMilliseconds / 1000, (timeoutMilliseconds % 1000) * 1000 };
    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof timeout);
    setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof timeout);
    sockaddr_un address {};
    address.sun_family = AF_UNIX;
    std::memcpy(address.sun_path, path.c_str(), path.size() + 1);
    std::string reply;
    if (::connect(fd, reinterpret_cast<const sockaddr*>(&address), sizeof address) == 0) {
        const std::string message = line + "\n";
        if (::send(fd, message.data(), message.size(), MSG_NOSIGNAL) == static_cast<ssize_t>(message.size())) {
            char buffer[8192];
            while (reply.empty() || reply.back() != '\n') {
                const ssize_t got = ::recv(fd, buffer, sizeof buffer, 0);
                if (got <= 0) break;
                reply.append(buffer, static_cast<std::size_t>(got));
            }
        }
    }
    ::close(fd);
    return reply;
}

void TrainerLink3D::readStatus(const std::string& json, bool alive) {
    const nlohmann::json s = nlohmann::json::parse(json, nullptr, false);
    if (s.is_discarded() || !s.is_object()) return;
    TrainerStatus3D status;
    status.alive = alive;
    status.known = true;
    status.run = text(s, "run");
    status.task = text(s, "task");
    status.taskName = text(s, "taskName");
    status.state = text(s, "state");
    status.mode = text(s, "mode");
    status.modeLabel = text(s, "modeLabel");
    status.device = text(s, "device");
    status.message = text(s, "message");
    status.iteration = static_cast<std::uint64_t>(number(s, "iteration"));
    status.steps = static_cast<std::uint64_t>(number(s, "steps"));
    status.episodes = static_cast<std::uint64_t>(number(s, "episodes"));
    status.sps = number(s, "sps");
    status.trainSeconds = number(s, "trainSeconds");
    status.threads = static_cast<int>(number(s, "threads"));
    status.environments = static_cast<int>(number(s, "environments"));
    status.cpuTemperature = number(s, "cpuTemperature");
    status.policy = text(s, "policy");
    if (const auto last = s.find("last"); last != s.end() && last->is_object()) {
        status.lastReward = number(*last, "reward");
        status.lastEpisodeSeconds = number(*last, "episode_seconds");
        status.lastFallFraction = number(*last, "fall_fraction", -1.0);
    }
    if (const auto evaluation = s.find("evaluation"); evaluation != s.end() && evaluation->is_object()
        && evaluation->contains("score")) {
        status.evaluated = true;
        status.evaluationScore = number(*evaluation, "score");
        status.evaluationIteration = static_cast<std::uint64_t>(number(*evaluation, "iteration"));
        if (const auto solvers = evaluation->find("solvers"); solvers != evaluation->end() && solvers->is_object())
            for (const auto& [name, metrics] : solvers->items())
                status.evaluationStand.emplace_back(name, number(metrics, "stand_fraction"));
    }
    if (const auto best = s.find("best"); best != s.end() && best->is_object()) {
        status.bestScore = number(*best, "score");
        status.bestIteration = static_cast<std::int64_t>(number(*best, "iteration", -1.0));
    }
    if (!alive && status.state != "erro") status.state = "parado";
    std::lock_guard lock(m_mutex);
    m_status = std::move(status);
}

void TrainerLink3D::worker() {
    while (!m_stop) {
        std::deque<std::string> commands;
        {
            std::unique_lock lock(m_mutex);
            m_wake.wait_for(lock, std::chrono::milliseconds(1000), [this] { return m_stop || m_refresh; });
            commands.swap(m_queue);
            m_refresh = false;
            // Os processos abertos por esta tela que ja sairam (sem zumbis). Um
            // treinador que nao abriu deixa a ultima linha do log como aviso.
            std::erase_if(m_children, [this](int pid) {
                int status = 0;
                if (::waitpid(pid, &status, WNOHANG) != pid) return false;
                if (m_opening && WIFEXITED(status) && WEXITSTATUS(status) != 0) {
                    m_opening = false;
                    m_failure = lastLogLine();
                }
                return true;
            });
        }
        if (m_stop) break;
        for (const std::string& command : commands) {
            std::string reply = request(command, 40000);
            std::lock_guard lock(m_mutex);
            m_lastReply = reply.empty() ? std::string("{\"ok\":false,\"error\":\"o treinador nao respondeu\"}") : reply;
        }
        const std::string reply = request(R"({"cmd":"status"})", 3000);
        const nlohmann::json parsed = nlohmann::json::parse(reply, nullptr, false);
        if (!parsed.is_discarded() && parsed.is_object() && parsed.value("ok", false) && parsed.contains("status")
            && parsed["status"].value("run", std::string()) == m_run) {
            readStatus(parsed["status"].dump(), true);
            std::lock_guard lock(m_mutex);
            m_opening = false;
            m_failure.clear();
            continue;
        }
        // Sem treinador (ou outro treino aberto): o ultimo estado gravado.
        std::ifstream file(m_paths.run(m_run) / "status.json");
        if (file) {
            const std::string json { std::istreambuf_iterator<char>(file), std::istreambuf_iterator<char>() };
            readStatus(json, false);
        } else {
            std::lock_guard lock(m_mutex);
            m_status = TrainerStatus3D {};
            m_status.state = "novo";
        }
        std::lock_guard lock(m_mutex);
        if (m_opening && std::chrono::steady_clock::now() - m_openedAt < std::chrono::seconds(120))
            m_status.state = "abrindo";
        else m_opening = false;
        if (!m_failure.empty()) m_status.message = "o treinador não abriu: " + m_failure;
    }
}

} // namespace MatterEngine
