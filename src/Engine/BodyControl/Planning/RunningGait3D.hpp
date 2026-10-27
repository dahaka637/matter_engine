#pragma once

#include <Eigen/Core>

namespace MatterEngine {

// As regras do trote (G4 da locomocao fisica nova; diario 31 do vault, 27/10): a marcha com o
// apoio duplo trocado pelo voo. Cada apoio absorve o pouso e empurra; o equilibrio vem de onde
// o pe pousa (o ponto de captura previsto no toque), nao de forca horizontal no apoio.

// Onde o pe que vai pousar mira (plano do chao). `remaining` e o tempo ate o toque (o resto do
// empurrao mais o voo); o CoM segue balistico no voo, sem CoP. O ponto neutro de Raibert usa o
// apoio inteiro (absorcao + empurrao); `homeError` (casa - CoM) so pesa parado (v_des = 0).
struct RunningLandingInput3D {
    Eigen::Vector2d centerOfMass = Eigen::Vector2d::Zero();
    Eigen::Vector2d velocity = Eigen::Vector2d::Zero();
    Eigen::Vector2d desiredVelocity = Eigen::Vector2d::Zero();
    Eigen::Vector2d left = Eigen::Vector2d::UnitY();
    double remaining = 0.0;
    double omega = 3.3;
    double stanceTime = 0.25;
    double stepWidth = 0.08;
    // +1 o pe esquerdo, -1 o direito.
    double side = 1.0;
    Eigen::Vector2d homeError = Eigen::Vector2d::Zero();
    double homeGain = 0.05;
    // O pe de apoio e a largura minima (2 cm; com 5 cm, 1,5 m/s caia): o pe que pousa nunca cruza o de apoio (as pernas
    // colidiriam); o que faltar de captura o proximo passo, do outro pe, completa. Sem apoio, nao limita.
    bool hasStance = false;
    Eigen::Vector2d stance = Eigen::Vector2d::Zero();
    double minimumWidth = 0.02;
};
[[nodiscard]] Eigen::Vector2d runningLandingPoint3D(const RunningLandingInput3D& input);

// A decolagem: com a perna de apoio estendida (joelho ate `kneeLimit`), quando a velocidade
// vertical chega a v0 ou atinge o pico acima de metade dela (a forca de chao ja caiu abaixo do peso).
[[nodiscard]] bool runningTakeoffReady3D(double verticalSpeed, double previousVerticalSpeed, double takeoffSpeed,
    double knee, double kneeLimit);

// A absorcao: a aceleracao vertical que leva o CoM parado a altura do empurrao (PD critico).
[[nodiscard]] double runningAbsorbAcceleration3D(double height, double verticalSpeed, double targetHeight, double stiffness);

// O andar pela velocidade pedida, com histerese: trote a partir de `threshold`, de volta a marcha
// abaixo de `threshold - hysteresis`.
[[nodiscard]] bool runningGaitSelectTrot3D(double speed, bool trotting, double threshold, double hysteresis);

} // namespace MatterEngine
