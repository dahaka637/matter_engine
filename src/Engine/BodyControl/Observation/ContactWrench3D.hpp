#pragma once

#include "Engine/Physics/Articulation/ArticulationDynamics3D.hpp"

#include <Eigen/Core>

#include <array>
#include <vector>

namespace MatterEngine {

// A conta das forcas externas pela dinamica (G0, plano 6.2): a forca
// generalizada que o movimento medido exige alem dos musculos e da gravidade,
//   tau_ext = M(q) (v+ - v) / dt + h(q, v) - tau_musculo,
// nunca pelos impulsos de contato do backend (estimativas de antes da solucao).
// As 6 linhas da raiz dao o wrench externo total do corpo (torques internos nao
// aparecem nelas); a divisao entre os apoios sai por minimos quadrados nos
// Jacobianos dos apoios, com o residuo como confianca.
struct ContactSupport3D {
    int link = -1;
    // O ponto de reducao do wrench (no frame do link): o centro da sola, a mao.
    Eigen::Vector3d pointLocal = Eigen::Vector3d::Zero();
    bool touching = false;
};

struct SupportWrench3D {
    // Forca e momento (mundo) no ponto de reducao do apoio.
    Eigen::Vector3d force = Eigen::Vector3d::Zero();
    Eigen::Vector3d torque = Eigen::Vector3d::Zero();
    Eigen::Vector3d pointWorld = Eigen::Vector3d::Zero();
    // Centro de pressao no plano horizontal do ponto (so com forca normal).
    Eigen::Vector3d centerOfPressure = Eigen::Vector3d::Zero();
    bool hasCenterOfPressure = false;
};

struct ContactWrenchEstimate3D {
    // O wrench externo total: forca (mundo) e momento em torno do centro de massa.
    Eigen::Vector3d totalForce = Eigen::Vector3d::Zero();
    Eigen::Vector3d totalTorqueAboutCenterOfMass = Eigen::Vector3d::Zero();
    std::vector<SupportWrench3D> supports;
    // O que os apoios nao explicam (norma da forca generalizada restante, nas
    // linhas da raiz e das juntas) e a fracao do total: a confianca da divisao.
    double residual = 0.0;
    double residualFraction = 0.0;
    Eigen::VectorXd generalizedExternal;
};

// `before`/`after` sao estados consecutivos (um passo dt); `muscle` e o torque
// muscular generalizado aplicado nesse passo (zero nas linhas da raiz).
[[nodiscard]] ContactWrenchEstimate3D estimateContactWrench3D(const ArticulationModel3D& model,
    const ArticulationState3D& before, const ArticulationKinematics3D& beforeKinematics,
    const ArticulationDynamics3D& beforeDynamics, const ArticulationState3D& after, const Eigen::VectorXd& muscle,
    const std::vector<ContactSupport3D>& supports, double dt);

} // namespace MatterEngine
