#pragma once

#include <Eigen/Core>

#include <vector>

namespace MatterEngine {

// A marcha pelo DCM (G3 da locomocao fisica nova; Englsberger et al., T-RO 2015):
// as pegadas saem da velocidade pedida, alternando os pes com a largura do
// passo; a referencia do DCM e calculada de tras para frente pelas pegadas
// futuras (o CoP em cada pe de apoio),
//   xi_fim(i) = p(i+1) + e^(-omega T) (xi_fim(i+1) - p(i+1)),   xi_fim(ultimo) = p(ultimo),
// e dentro do passo i: xi_ref(t) = p(i) + e^(omega (t - T)) (xi_fim(i) - p(i)).
struct DcmWalkerSettings3D {
    double stepDuration = 0.5;
    double stepWidth = 0.2;
    double comHeight = 0.9;
    double gravity = 9.81;
    // Quantas pegadas a frente entram na recursao (a ultima e tratada como parada).
    int horizon = 3;
};

struct DcmWalkerPlan3D {
    // As pegadas (a primeira e o pe de apoio de agora).
    std::vector<Eigen::Vector2d> footsteps;
    // O DCM no fim de cada passo.
    std::vector<Eigen::Vector2d> endOfStep;
};

// O passo de agora: o pe de apoio, o lado dele (+1 esquerdo, -1 direito), a
// direcao "esquerda" do corpo (unitaria, horizontal) e a velocidade pedida.
[[nodiscard]] DcmWalkerPlan3D planDcmWalk3D(const DcmWalkerSettings3D& settings, const Eigen::Vector2d& stance,
    int stanceSide, const Eigen::Vector2d& left, const Eigen::Vector2d& velocity);

struct DcmReference3D {
    Eigen::Vector2d position = Eigen::Vector2d::Zero();
    Eigen::Vector2d rate = Eigen::Vector2d::Zero();
};

// A referencia do DCM no instante t (0..T) do passo de agora.
[[nodiscard]] DcmReference3D sampleDcmWalk3D(const DcmWalkerSettings3D& settings, const DcmWalkerPlan3D& plan, double time);

// O ajuste do pouso pelo DCM medido (Khadiv et al. 2016; Griffin et al. 2017):
// o DCM previsto no toque, com o CoP no pe de apoio, e
//   xi_T = p0 + e^(omega t_resto) (xi - p0),
// e o pouso que o deixa no deslocamento nominal do plano e
//   p1 = xi_T - (xi_fim(0) - p1_nominal).
// O ajuste e limitado em modulo e nao deixa o pe cruzar o de apoio.
struct DcmLandingLimits3D {
    double maximumAdjustment = 0.25;
    double minimumWidth = 0.12;
};
[[nodiscard]] Eigen::Vector2d adjustDcmLanding3D(const DcmWalkerSettings3D& settings, const DcmWalkerPlan3D& plan,
    const DcmLandingLimits3D& limits, const Eigen::Vector2d& dcm, int stanceSide, const Eigen::Vector2d& left, double timeLeft);

} // namespace MatterEngine
