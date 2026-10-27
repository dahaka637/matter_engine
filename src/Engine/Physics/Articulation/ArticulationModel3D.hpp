#pragma once

#include "Engine/Physics/PhysicsScene3D.hpp"
#include "Engine/Physics/RagdollProfile3D.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <array>
#include <cstdint>
#include <vector>

namespace MatterEngine {

// O modelo dinamico do corpo articulado (locomocao fisica nova, G0): uma
// arvore de base flutuante com as massas e inercias que o backend de fato usa
// (ragdollEffectiveMass: depois do Stabilize do Jolt). Lado neutro, sem SDK.
//
// Manifesto das coordenadas (04_locomocao/plano_locomocao_fisica_nova.md, 6.1):
// - posicao de cada junta: o vetor de rotacao (mapa exponencial) no frame da
//   junta, o mesmo dos clipes e dos motores; eixos travados valem 0;
// - R_filho = R_pai * A_pai * exp(q) * A_filho^T, com A = modelo^-1 * frame da junta;
// - velocidade generalizada: [v da origem da pelve (mundo), w da pelve (mundo),
//   qdot dos eixos habilitados (Twist, Swing1, Swing2)] - a ordem do
//   ArticulationIndexing3D;
// - o Jolt publica w_rel no frame da junta do FILHO = J_r(q) qdot (nao qdot):
//   a conversao fica em articulationStateFromRagdoll3D.
struct ArticulationLink3D {
    int parent = -1;
    double mass = 0.0;
    Eigen::Vector3d centerOfMassLocal = Eigen::Vector3d::Zero();
    // Tensor de inercia no centro de massa, no frame do link.
    Eigen::Matrix3d inertiaLocal = Eigen::Matrix3d::Zero();
    // A junta de entrada (links >= 1): ancora e frame da junta nos dois lados.
    Eigen::Vector3d anchorInParent = Eigen::Vector3d::Zero();
    Eigen::Vector3d anchorInChild = Eigen::Vector3d::Zero();
    Eigen::Matrix3d jointInParent = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d jointInChild = Eigen::Matrix3d::Identity();
    std::array<bool, 3> axisEnabled {};
    // Twist + um swing (maos e pes): o Jolt trava pela decomposicao
    // q = q_swing * q_twist, entao R = R_swing(beta) R_x(alfa) - nao o mapa
    // exponencial com o eixo travado em zero (outra superficie de poses).
    bool swingAfterTwist = false;
    // Indice generalizado de cada eixo (-1 travado).
    std::array<int, 3> dof { -1, -1, -1 };
};

struct ArticulationModel3D {
    std::vector<ArticulationLink3D> links;
    // As colunas nao nulas dos Jacobianos de cada link: os 6 da raiz e os eixos
    // dos ancestrais (a esparsidade que a montagem da dinamica usa).
    std::vector<std::vector<int>> chainDofs;
    int dofCount = 6;
    double totalMass = 0.0;
    Eigen::Vector3d gravity { 0.0, 0.0, -9.81 };
    // O Jolt nao aplica o torque giroscopico (w x Iw) nos corpos do ragdoll
    // (ApplyGyroscopicForce desligado): o modelo segue o corpo simulado.
    bool gyroscopic = false;
};

struct ArticulationState3D {
    Eigen::Vector3d rootPosition = Eigen::Vector3d::Zero();
    Eigen::Quaterniond rootOrientation = Eigen::Quaterniond::Identity();
    // Mesmo indice dos links; a raiz fica zero.
    std::vector<Eigen::Vector3d> jointCoordinates;
    Eigen::VectorXd velocity;
};

[[nodiscard]] ArticulationModel3D buildArticulationModel3D(
    const RagdollProfile3D& profile, const RagdollEffectiveMass3D& mass);

// Estado generalizado a partir do snapshot do backend (poses de mundo,
// velocidades de centro de massa e w_rel no frame do filho).
[[nodiscard]] ArticulationState3D articulationStateFromRagdoll3D(
    const ArticulationModel3D& model, const RagdollState3D& state);

// A junta de entrada de um link, no frame da junta do pai: a rotacao relativa
// pelas coordenadas, a base S (w_rel = S qdot; colunas pelos eixos Twist,
// Swing1, Swing2), a derivada S' qdot e as coordenadas de uma rotacao.
[[nodiscard]] Eigen::Matrix3d articulationJointRotation3D(const ArticulationLink3D& link, const Eigen::Vector3d& coordinates);
[[nodiscard]] Eigen::Matrix3d articulationJointSubspace3D(const ArticulationLink3D& link, const Eigen::Vector3d& coordinates);
[[nodiscard]] Eigen::Vector3d articulationJointSubspaceRate3D(
    const ArticulationLink3D& link, const Eigen::Vector3d& coordinates, const Eigen::Vector3d& rates);
[[nodiscard]] Eigen::Vector3d articulationJointCoordinates3D(const ArticulationLink3D& link, const Eigen::Matrix3d& relative);

// O atuador: o torque da junta e um par pai/filho com componentes no frame da
// junta do filho (o feedforward do backend), so nos eixos habilitados.
// Generalizado de um feedforward e o feedforward que entrega um generalizado.
[[nodiscard]] Eigen::VectorXd articulationGeneralizedFromFeedforward3D(const ArticulationModel3D& model,
    const ArticulationState3D& state, const std::vector<Eigen::Vector3d>& feedforward);
[[nodiscard]] std::vector<Eigen::Vector3d> articulationFeedforwardFromGeneralized3D(
    const ArticulationModel3D& model, const ArticulationState3D& state, const Eigen::VectorXd& generalized);

// Jacobianos esquerdo e direito do mapa exponencial: w = J qdot.
[[nodiscard]] Eigen::Matrix3d exponentialLeftJacobian3D(const Eigen::Vector3d& q);
[[nodiscard]] Eigen::Matrix3d exponentialRightJacobian3D(const Eigen::Vector3d& q);
[[nodiscard]] Eigen::Matrix3d exponentialRotation3D(const Eigen::Vector3d& q);

} // namespace MatterEngine
