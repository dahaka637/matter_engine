#pragma once

#include "Engine/RL/Environment/TrainingTask3D.hpp"
#include "Engine/RL/Environment/TrainingTeacher3D.hpp"

#include <filesystem>
#include <memory>
#include <string>
#include <vector>

namespace MatterEngine {

// O professor de locomocao: a locomocao fisica do jogo (o mesmo runtime dos
// atletas do laboratorio, no modo fisico total) conduzindo o corpo dos mundos
// de treino, num roteiro sorteado de se estabilizar, girar, andar para todos
// os lados e aguentar empurroes. O RL aprende imitando a acao dele (o
// aprendizado guiado) e depois segue pelo treino normal.
struct LocomotionTeacherSettings3D {
    // As ajudas do modo fisico (%): parado, andando e nos pes. Em 0 e o "sem
    // forca auxiliar" do jogo (andando, a pelve ainda leva metade do peso:
    // a sustentacao de base da locomocao fisica v2).
    float standingAssistPercent = 0.0f;
    float walkingAssistPercent = 0.0f;
    float feetAssistPercent = 0.0f;
    // O roteiro: trechos de segmentSeconds; cada um parado, girando no lugar
    // ou andando num rumo em relacao ao olhar (frente, costas, lados e
    // diagonais), numa fracao da velocidade da passada desse rumo, as vezes
    // virando o olhar ao mesmo tempo (curva).
    TrainingRange3D segmentSeconds { 2.0f, 5.0f };
    float standingWeight = 0.2f;
    float turnWeight = 0.15f;
    float forwardWeight = 0.35f;
    float backwardWeight = 0.1f;
    float sideWeight = 0.1f;
    float diagonalWeight = 0.1f;
    TrainingRange3D speedFraction { 0.3f, 1.0f };
    // O giro do olhar (rad/s): girando no lugar e nas curvas (em curveFraction
    // dos trechos andando).
    TrainingRange3D turnRate { 0.6f, 1.8f };
    float curveFraction = 0.3f;
    TrainingRange3D curveRate { 0.2f, 0.8f };
    // Empurroes no tronco (pelve, peito e alto do peito, como o teste do
    // laboratorio): a cada pushInterval s, pushForce N por pushSeconds s.
    bool pushes = true;
    TrainingRange3D pushInterval { 3.0f, 7.0f };
    TrainingRange3D pushForce { 80.0f, 220.0f };
    TrainingRange3D pushSeconds { 0.1f, 0.2f };
};

// O "teacher" de uma tarefa (a secao opcional dela); sem ela, os padroes.
[[nodiscard]] LocomotionTeacherSettings3D loadLocomotionTeacherSettings3D(const std::filesystem::path& taskPath);

class LocomotionTeacher3D final : public TrainingTeacher3D {
public:
    // assetsDirectory: a pasta assets do jogo (o personagem e os clipes).
    LocomotionTeacher3D(const std::filesystem::path& assetsDirectory, const LocomotionTeacherSettings3D& settings);
    ~LocomotionTeacher3D() override;

    void attach(const TrainingTeacherWorld3D& world) override;
    void beginEpisode(const TrainingTeacherWorld3D& world, const RagdollState3D& state, std::uint64_t seed) override;
    void control(const TrainingTeacherWorld3D& world, const RagdollState3D& state, float deltaTime) override;
    [[nodiscard]] Vec3 command(std::size_t world) const override;
    [[nodiscard]] float gaitPhase(std::size_t world) const override;

    [[nodiscard]] const LocomotionTeacherSettings3D& settings() const;
    // O estado do professor num mundo, em texto (a locomocao, o modo do corpo
    // e as teclas do roteiro): diagnostico.
    [[nodiscard]] std::string describe(std::size_t world) const;

private:
    struct Assets;
    struct World;
    LocomotionTeacherSettings3D m_settings;
    std::unique_ptr<Assets> m_assets;
    std::vector<std::unique_ptr<World>> m_worlds;
};

} // namespace MatterEngine
