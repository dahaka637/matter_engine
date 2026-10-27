#pragma once

#include "Engine/Physics/Articulation/ArticulationModel3D.hpp"

#include <Eigen/Core>

#include <vector>

namespace MatterEngine {

using ArticulationJacobian3D = Eigen::Matrix<double, 3, Eigen::Dynamic>;

// Cinematica de mundo de cada link: pose, velocidades, Jacobianos (3 x n) e a
// aceleracao "de bias" (com aceleracao generalizada zero: so os produtos de
// velocidade). Tudo em coordenadas de mundo.
struct ArticulationKinematics3D {
    std::vector<Eigen::Matrix3d> rotation;
    std::vector<Eigen::Vector3d> origin;
    std::vector<Eigen::Vector3d> centerOfMass;
    std::vector<Eigen::Vector3d> angularVelocity;
    std::vector<Eigen::Vector3d> originVelocity;
    std::vector<Eigen::Vector3d> angularBias;
    std::vector<Eigen::Vector3d> originBias;
    std::vector<ArticulationJacobian3D> angularJacobian;
    std::vector<ArticulationJacobian3D> originJacobian;
};

// M(q) a + h(q, v) = tau + sum J^T f (h inclui a gravidade do modelo).
struct ArticulationDynamics3D {
    Eigen::MatrixXd massMatrix;
    Eigen::VectorXd bias;
    Eigen::Vector3d centerOfMass = Eigen::Vector3d::Zero();
    // Momento centroidal: [linear; angular em torno do centro de massa] = A v.
    Eigen::Matrix<double, 6, Eigen::Dynamic> centroidalMatrix;
    // d/dt (A v) com aceleracao zero (sem a gravidade).
    Eigen::Matrix<double, 6, 1> centroidalBias = Eigen::Matrix<double, 6, 1>::Zero();
};

[[nodiscard]] ArticulationKinematics3D computeArticulationKinematics3D(
    const ArticulationModel3D& model, const ArticulationState3D& state);

[[nodiscard]] ArticulationDynamics3D computeArticulationDynamics3D(
    const ArticulationModel3D& model, const ArticulationKinematics3D& kinematics);

// Jacobiano linear de um ponto preso ao link (mundo) e a aceleracao de bias dele.
[[nodiscard]] ArticulationJacobian3D articulationPointJacobian3D(
    const ArticulationKinematics3D& kinematics, int link, const Eigen::Vector3d& pointWorld);
[[nodiscard]] Eigen::Vector3d articulationPointBias3D(
    const ArticulationKinematics3D& kinematics, int link, const Eigen::Vector3d& pointWorld);

// Integra o estado por dt com a velocidade dada (raiz e juntas): o passo de
// referencia para as provas por diferenca finita.
[[nodiscard]] ArticulationState3D integrateArticulationState3D(
    const ArticulationModel3D& model, const ArticulationState3D& state, const Eigen::VectorXd& velocity, double dt);

// Energia cinetica (prova da matriz de massa).
[[nodiscard]] double articulationKineticEnergy3D(
    const ArticulationModel3D& model, const ArticulationKinematics3D& kinematics);

} // namespace MatterEngine
