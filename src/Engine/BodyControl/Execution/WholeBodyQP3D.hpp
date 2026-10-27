#pragma once

#include "Engine/Physics/Articulation/ArticulationDynamics3D.hpp"

#include <Eigen/Core>

#include <memory>
#include <string>
#include <vector>

namespace MatterEngine {

// O executor de corpo inteiro (plano 6.6): um QP ponderado por tick sobre as
// aceleracoes generalizadas e as forcas nos pontos de contato.
//   restricoes: M a + h = S^T tau + Jc^T lambda + tau_ext (as 6 linhas da raiz),
//               piramide de atrito em cada ponto, teto de torque de cada eixo;
//   custo: tarefas ponderadas sobre a aceleracao (|A a - b|^2), o contato sem
//          escorregar (|Jc a + Jc' v - a_c|^2, com peso alto: folga penalizada,
//          o pe nunca fica soldado) e uma regularizacao pequena.
// Unico escritor muscular: devolve o torque de cada eixo (generalizado).
struct WholeBodyContactPoint3D {
    int link = -1;
    Eigen::Vector3d pointWorld = Eigen::Vector3d::Zero();
    Eigen::Vector3d normal = Eigen::Vector3d::UnitZ();
    double friction = 0.6;
    double maximumNormalForce = 5000.0;
    // Forca normal minima: o centro de pressao nunca encosta na borda do apoio
    // (na borda, qualquer erro faz o pe tombar e o pe deixa de ser rigido).
    double minimumNormalForce = 0.0;
    // A aceleracao desejada do ponto (zero: parado; com amortecimento do escorregar).
    Eigen::Vector3d desiredAcceleration = Eigen::Vector3d::Zero();
    // Ponto de contato observado (o backend o publica tocando): a aceleracao
    // dele e uma igualdade (3 linhas). Com o pe todo apoiado ele fica rigido;
    // apoiado numa quina, gira sobre ela - o grau que o pe rigido 6D nao tem.
    bool constrained = false;
};

// Um link apoiado como corpo rigido (o pe com a sola no chao): a aceleracao
// 6D dele e uma igualdade (o contato do backend e rigido; como tarefa mole o
// QP previa o pe acelerando contra o chao e a forca real saia outra).
struct WholeBodyContactLink3D {
    int link = -1;
    // [angular; linear da origem] desejadas (zero ou o amortecimento do escorregar).
    Eigen::Matrix<double, 6, 1> desiredAcceleration = Eigen::Matrix<double, 6, 1>::Zero();
};

struct WholeBodyTask3D {
    std::string name;
    // A a ~ b (linhas x n).
    Eigen::MatrixXd matrix;
    Eigen::VectorXd target;
    double weight = 1.0;
};

struct WholeBodyQPInput3D {
    const ArticulationModel3D* model = nullptr;
    const ArticulationKinematics3D* kinematics = nullptr;
    const ArticulationDynamics3D* dynamics = nullptr;
    const ArticulationState3D* state = nullptr;
    std::vector<WholeBodyContactPoint3D> contacts;
    std::vector<WholeBodyContactLink3D> contactLinks;
    std::vector<WholeBodyTask3D> tasks;
    // Teto de torque de cada eixo (n - 6, a ordem das juntas).
    Eigen::VectorXd torqueLimit;
    // A forca externa estimada (generalizada, n), com a confianca ja aplicada.
    Eigen::VectorXd externalGeneralized;
    // Peso da tarefa mole de nao escorregar nos pontos (0 com os links rigidos).
    double contactWeight = 0.0;
    double accelerationRegularization = 1e-4;
    double forceRegularization = 1e-5;
    int maximumIterations = 200;
};

enum class WholeBodyQPStatus3D { Solved, MaximumIterations, Infeasible, NotFinite, InvalidInput };

struct WholeBodyQPResult3D {
    WholeBodyQPStatus3D status = WholeBodyQPStatus3D::InvalidInput;
    Eigen::VectorXd acceleration;
    // Generalizado (n): zero nas linhas da raiz.
    Eigen::VectorXd torque;
    // 3 por ponto de contato (mundo).
    Eigen::VectorXd contactForces;
    int iterations = 0;
    double solveMicros = 0.0;
    // Resolvido de novo a frio depois de uma falha numerica do reaproveitado.
    bool coldRetry = false;
    // O residuo das linhas da raiz com a solucao (N, N.m): a coerencia dinamica.
    double dynamicsResidual = 0.0;
    // A maior violacao das igualdades (raiz + pes rigidos) e das desigualdades.
    double equalityViolation = 0.0;
    double inequalityViolation = 0.0;
    [[nodiscard]] bool usable() const {
        return status == WholeBodyQPStatus3D::Solved || status == WholeBodyQPStatus3D::MaximumIterations;
    }
};

class WholeBodyQP3D {
public:
    WholeBodyQP3D();
    ~WholeBodyQP3D();
    WholeBodyQP3D(const WholeBodyQP3D&) = delete;
    WholeBodyQP3D& operator=(const WholeBodyQP3D&) = delete;

    [[nodiscard]] WholeBodyQPResult3D solve(const WholeBodyQPInput3D& input);

private:
    struct Impl;
    std::unique_ptr<Impl> m_impl;
};

// Tarefas prontas (aceleracao desejada por PD + bias): orientacao de um link,
// posicao de um ponto, momento centroidal e postura das juntas.
[[nodiscard]] WholeBodyTask3D wholeBodyOrientationTask3D(const ArticulationKinematics3D& kinematics, int link,
    const Eigen::Matrix3d& desiredRotation, const Eigen::Vector3d& desiredAngularVelocity,
    const Eigen::Vector3d& feedforwardAngularAcceleration, double stiffness, double damping, double weight,
    std::string name);
[[nodiscard]] WholeBodyTask3D wholeBodyCentroidalTask3D(const ArticulationModel3D& model,
    const ArticulationDynamics3D& dynamics, const ArticulationState3D& state,
    const Eigen::Vector3d& desiredLinearMomentumRate, const Eigen::Vector3d& desiredAngularMomentumRate,
    double linearWeight, double angularWeight);
[[nodiscard]] WholeBodyTask3D wholeBodyPostureTask3D(const ArticulationModel3D& model,
    const ArticulationState3D& state, const std::vector<Eigen::Vector3d>& coordinates,
    const std::vector<Eigen::Vector3d>& rates, const std::vector<Eigen::Vector3d>& accelerations,
    double stiffness, double damping, double weight);

} // namespace MatterEngine
