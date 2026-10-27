#pragma once

#include <Eigen/Core>

#include <array>
#include <vector>

namespace MatterEngine {

// O passo de recuperacao (G3 da locomocao fisica nova, plano 6.4): o ponto de
// captura (DCM, xi = c + c'/omega do pendulo invertido linear) diz para onde o
// corpo "cai"; quando ele sai do apoio, um pe da um passo para pegar o corpo.
// O lugar do pouso e o DCM previsto no fim do passo com o CoP no pe de apoio,
// xi(T) = p + e^(omega T) (xi0 - p), dentro do alcance e sem cruzar as pernas.
struct StepRecoverySettings3D {
    double comHeight = 0.9;
    double gravity = 9.81;
    // O DCM fora do apoio por mais que isto dispara o passo (m).
    double triggerMargin = 0.05;
    double stepDuration = 0.3;
    double minimumStepDuration = 0.22;
    // Alcance do pouso a partir do pe de apoio (m) e a largura minima entre os pes.
    double maximumReach = 0.6;
    double minimumWidth = 0.14;
    // O pouso passa um pouco do DCM previsto (m): o corpo fica capturado com folga.
    double overshoot = 0.03;
    // Tempos candidatos do passo e o teto da aceleracao de pico do pe (quintica: 5,774 d / T^2).
    std::vector<double> durations { 0.14, 0.16, 0.18, 0.20, 0.22, 0.25, 0.30 };
    double maximumSwingAcceleration = 60.0;
};

// O ponto do poligono (convexo) mais proximo, recuado `margin` para dentro.
[[nodiscard]] Eigen::Vector2d projectToPolygon3D(const std::vector<Eigen::Vector2d>& polygon, const Eigen::Vector2d& point, double margin);

struct StepRecoveryFoot3D {
    // Centro da sola no chao (mundo) e a direcao "para fora" do corpo (unitaria,
    // horizontal): para o pe esquerdo, a esquerda do corpo.
    Eigen::Vector2d center = Eigen::Vector2d::Zero();
    Eigen::Vector2d outward = Eigen::Vector2d::UnitY();
    bool supporting = true;
    // Os pontos de apoio observados deste pe (o CoP util fica neles).
    std::vector<Eigen::Vector2d> support;
};

struct StepRecoveryPlan3D {
    bool step = false;
    int swingFoot = -1;
    Eigen::Vector2d landing = Eigen::Vector2d::Zero();
    double duration = 0.0;
    // Diagnostico: o DCM e a distancia dele para fora do apoio.
    Eigen::Vector2d capturePoint = Eigen::Vector2d::Zero();
    double outside = 0.0;
    // O CoP do pe de apoio usado na previsao (o balanco o comanda).
    Eigen::Vector2d stanceCop = Eigen::Vector2d::Zero();
};

[[nodiscard]] double stepRecoveryOmega3D(const StepRecoverySettings3D& settings);

// Quanto um ponto esta fora do poligono convexo (negativo: dentro).
[[nodiscard]] double polygonOutside3D(const std::vector<Eigen::Vector2d>& polygon, const Eigen::Vector2d& point);

// Decide se e como dar o passo. `support` e o poligono do apoio atual (os
// pontos de contato no chao); `feet` os dois pes.
// O pouso replanejado durante o passo (a cada tick): o DCM de agora, o tempo
// que falta e o pe de apoio fixo.
[[nodiscard]] Eigen::Vector2d replanLanding3D(const StepRecoverySettings3D& settings, const Eigen::Vector2d& centerOfMass,
    const Eigen::Vector2d& centerOfMassVelocity, const Eigen::Vector2d& stanceCenter, const Eigen::Vector2d& swingOutward,
    double remaining);

[[nodiscard]] StepRecoveryPlan3D planRecoveryStep3D(const StepRecoverySettings3D& settings,
    const Eigen::Vector2d& centerOfMass, const Eigen::Vector2d& centerOfMassVelocity,
    const std::vector<Eigen::Vector2d>& support, const std::array<StepRecoveryFoot3D, 2>& feet);

// A trajetoria do pe no ar: posicao quintica no plano (velocidade e aceleracao
// zero nas pontas) e altura em arco, do inicio ao pouso.
struct SwingSample3D {
    Eigen::Vector3d position = Eigen::Vector3d::Zero();
    Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
    Eigen::Vector3d acceleration = Eigen::Vector3d::Zero();
};
[[nodiscard]] SwingSample3D sampleSwing3D(const Eigen::Vector3d& start, const Eigen::Vector3d& landing, double height,
    double duration, double time);

} // namespace MatterEngine
