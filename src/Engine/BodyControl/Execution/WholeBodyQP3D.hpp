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
    // Nao zero (unitaria): a igualdade so nas 2 direcoes perpendiculares a ela. O pe
    // numa linha (ponta, calcanhar) e um ponto completo num extremo e este no outro:
    // posto 5, a rotacao sobre a linha livre (dois pontos 3D seriam redundantes; nota 29).
    Eigen::Vector3d freeDirection = Eigen::Vector3d::Zero();
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
    // 0: o nivel de cima (equilibrio, captura), resolvido primeiro; os valores
    // que ele alcanca viram igualdades do nivel 1, onde entra o resto (nota 27:
    // a postura nao pode tirar do equilibrio o que ele precisa).
    int priority = 1;
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
    // O mapa generalizado -> motor (articulationMotorTorqueMap3D; vazio = so a caixa
    // generalizada): com ele, |K tau| <= torqueLimit tambem (o recorte real do backend).
    Eigen::MatrixXd motorTorqueMap;
    // A forca externa estimada (generalizada, n), com a confianca ja aplicada.
    Eigen::VectorXd externalGeneralized;
    // Peso da tarefa mole de nao escorregar nos pontos (0 com os links rigidos).
    double contactWeight = 0.0;
    double accelerationRegularization = 1e-4;
    double forceRegularization = 1e-5;
    int maximumIterations = 200;
    // O teto do laco interno do ProxQP (o padrao dele, 1500, deixava ticks de ate 0,5 s na absorcao do salto).
    int maximumInnerIterations = 30;
    // A guarda dos batentes (notas 25 e 28 do vault), em aceleracao de recuperacao:
    // em cada eixo com limite (n - 6; infinito = sem guarda),
    //   -k^2 (q - qmin - e) - 2 k v <= a <= k^2 (qmax - e - q) - 2 k v.
    // (A forma em velocidade, v + dt a, exigia centenas de rad/s^2 depois de um
    // transiente de contato e tornava o problema inviavel longe do batente.)
    Eigen::VectorXd jointLower, jointUpper;
    // Margem por eixo (n - 6; vazio = jointGuardMargin para todos).
    Eigen::VectorXd jointMargin;
    double jointGuardRate = 10.0, jointGuardMargin = 0.015;
    // De quantos em quantos ticks o precondicionador do ProxQP e refeito.
    int preconditionerRefreshTicks = 30;
    // A maior violacao aceita nas restricoes (igualdades, inclusive as que guardam o
    // nivel de cima, e desigualdades): acima disso a solucao e recusada (nota 28).
    double acceptanceTolerance = 1e-3;
    // O peso extra das tarefas de cima no recurso de um nivel so.
    double priorityWeightScale = 1000.0;
};

enum class WholeBodyQPStatus3D { Solved, MaximumIterations, Infeasible, NotFinite, InvalidInput };

// Qual caminho deu a solucao: um nivel (sem prioridade 0), os dois niveis, um nivel
// so com as de cima priorityWeightScale vezes mais pesadas (o de baixo falhou por
// numero), so o de cima (a postura e a altura de baixo nao entram), ou o QP
// ponderado unico depois de o de cima falhar (as prioridades viram pesos).
enum class WholeBodyQPPath3D { Single, Hierarchical, WeightedPriority, TopOnly, FlatFallback };

struct WholeBodyQPResult3D {
    WholeBodyQPStatus3D status = WholeBodyQPStatus3D::InvalidInput;
    Eigen::VectorXd acceleration;
    // Generalizado (n): zero nas linhas da raiz.
    Eigen::VectorXd torque;
    // 3 por ponto de contato (mundo).
    Eigen::VectorXd contactForces;
    // Somadas sobre todas as resolucoes do tick (os dois niveis e as tentativas a frio).
    int iterations = 0;
    double solveMicros = 0.0;
    // Resolvido de novo a frio depois de uma falha numerica do reaproveitado.
    bool coldRetry = false;
    // Onde o tempo vai: montar as matrizes; atualizar e resolver os niveis (com as tentativas a frio).
    double buildMicros = 0.0, innerSolveMicros = 0.0;
    // O residuo das linhas da raiz com a solucao (N, N.m): a coerencia dinamica.
    double dynamicsResidual = 0.0;
    // A maior violacao das igualdades (raiz + pes rigidos) e das desigualdades.
    double equalityViolation = 0.0;
    double inequalityViolation = 0.0;
    WholeBodyQPPath3D path = WholeBodyQPPath3D::Single;
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
[[nodiscard]] WholeBodyTask3D wholeBodyPointTask3D(const ArticulationKinematics3D& kinematics, int link,
    const Eigen::Vector3d& pointWorld, const Eigen::Vector3d& desiredPosition, const Eigen::Vector3d& desiredVelocity,
    const Eigen::Vector3d& feedforwardAcceleration, double stiffness, double damping, double weight, std::string name);
[[nodiscard]] WholeBodyTask3D wholeBodyCentroidalTask3D(const ArticulationModel3D& model,
    const ArticulationDynamics3D& dynamics, const ArticulationState3D& state,
    const Eigen::Vector3d& desiredLinearMomentumRate, const Eigen::Vector3d& desiredAngularMomentumRate,
    double linearWeight, double angularWeight);
[[nodiscard]] WholeBodyTask3D wholeBodyPostureTask3D(const ArticulationModel3D& model,
    const ArticulationState3D& state, const std::vector<Eigen::Vector3d>& coordinates,
    const std::vector<Eigen::Vector3d>& rates, const std::vector<Eigen::Vector3d>& accelerations,
    double stiffness, double damping, double weight);

} // namespace MatterEngine
