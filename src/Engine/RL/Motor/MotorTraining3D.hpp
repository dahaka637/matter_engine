#pragma once

#include "Engine/Physics/PhysicsScene3D.hpp"
#include "Engine/Physics/RagdollProfile3D.hpp"

#include <array>
#include <filesystem>
#include <memory>
#include <span>
#include <string>
#include <vector>

namespace MatterEngine {

// O contrato motor absoluto (motor-v4): a rede comanda todos os eixos, sem
// lider animado; o zero da acao e a pose IDLE alerta (dado estatico do corpo).
struct MotorSettings3D {
    std::filesystem::path assetsDirectory;
    std::filesystem::path taskPath;
    std::uint32_t environments = 512, threads = 12;
    std::uint64_t seed = 1;
    bool evaluation = false;
    int drill = -1; // -1: mistura; 0: parado; 1: giro; 2..9: oito rumos de trote.
    float episodeSeconds = 20.0f;
    float trotSpeed = -1.0f; // > 0: velocidade fixa do trote; senao o teto do curriculo daquele rumo.
};

// As medidas de um episodio, na ordem de motorEpisodeMetricNames().
inline constexpr std::size_t MotorEpisodeMetricCount3D = 45;
struct MotorEpisode3D {
    std::uint32_t world = 0, drill = 0;
    std::array<float, MotorEpisodeMetricCount3D> values{};
};

class MotorTraining3D final {
  public:
    static constexpr float TickSeconds = 1.0f / 120.0f;
    static constexpr float DecisionSeconds = 4 * TickSeconds;
    explicit MotorTraining3D(const MotorSettings3D &settings);
    ~MotorTraining3D();
    MotorTraining3D(const MotorTraining3D &) = delete;
    MotorTraining3D &operator=(const MotorTraining3D &) = delete;
    [[nodiscard]] std::size_t environmentCount() const;
    [[nodiscard]] std::size_t actionSize() const;
    [[nodiscard]] std::size_t observationSize() const;
    [[nodiscard]] const std::vector<std::string> &actionNames() const;
    [[nodiscard]] const std::vector<std::string> &observationNames() const;
    [[nodiscard]] std::vector<float> actionDerivatives() const;
    // O mapa da acao: alvo = centro + raio * tanh(latente + vies), por eixo (raio, vies).
    [[nodiscard]] std::vector<std::array<float, 2>> actionMapping() const;
    [[nodiscard]] const RagdollProfile3D &profile() const;
    [[nodiscard]] const RagdollState3D &body(std::size_t world) const;
    [[nodiscard]] float lookYaw(std::size_t world) const;
    [[nodiscard]] std::string identity() const;
    [[nodiscard]] Vec3 commandVelocity(std::size_t world) const;
    [[nodiscard]] float imitationWeight() const;
    // O peso do clipe nos motores (clipDrive da receita; 0 = a rede sozinha).
    [[nodiscard]] float clipDrive() const;
    [[nodiscard]] static const std::array<std::string, 10> &drillIds();
    [[nodiscard]] static const std::array<std::string, 10> &drillNames();
    [[nodiscard]] static const std::array<const char *, MotorEpisodeMetricCount3D> &episodeMetricNames();
    // A pose IDLE de referencia (as coordenadas das juntas do quadro 0) e a acao latente de uma pose.
    [[nodiscard]] const std::vector<Vec3> &idleCoordinates() const;
    void poseActions(std::span<const Vec3> coordinates, std::span<float> actions) const;
    void reset(std::uint64_t seed);
    // Coloca o corpo de um mundo numa pose dada, o ponto mais baixo a 3 mm do chao (testes e sondas).
    void placePose(std::size_t world, std::span<const Vec3> coordinates, Quaternion rootOrientation);
    void observations(std::span<float> out, bool final = false) const;
    // A acao latente da pose de referencia na fase atual (o rotulo do professor), por mundo.
    void referenceActions(std::span<float> out) const;
    void step(std::span<const float> actions, std::span<float> rewards, std::span<std::uint8_t> terminated,
              std::span<std::uint8_t> truncated);
    [[nodiscard]] std::vector<MotorEpisode3D> takeEpisodes();
    // Troca atomica na fronteira do rollout; o contrato nao muda.
    void setTask(const std::filesystem::path &path);
    void setThreads(std::uint32_t threads);
    void setLookYaw(std::size_t world, float radians);
    // O teto do curriculo de velocidade de cada rumo de trote (m/s), lido no comeco de cada episodio.
    void setTrotLimits(std::span<const float> limits);
    [[nodiscard]] std::array<float, 8> trotLimits() const;
    // As cotas dos exercicios (parado, giro, trote; somam 1), lidas no comeco de cada episodio.
    void setShares(std::span<const float> shares);
    [[nodiscard]] std::array<float, 3> shares() const;
    [[nodiscard]] std::string frameLines(std::size_t world) const;
    // A postura dos clipes de referencia nas faixas da receita (JSON por exercicio): o clipe na velocidade
    // nominal dele e a referencia misturada que o treino pede na velocidade dada.
    [[nodiscard]] std::string referencePostureAudit(float trotSpeed) const;

  private:
    struct Impl;
    std::unique_ptr<Impl> m_impl;
};

} // namespace MatterEngine
