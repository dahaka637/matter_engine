#pragma once

#include "Engine/Physics/Articulation/ArticulationModel3D.hpp"
#include "Engine/Physics/PhysicsScene3D.hpp"
#include "Engine/Physics/RagdollProfile3D.hpp"

#include <Eigen/Core>

#include <vector>

namespace MatterEngine {

// O atuador da locomocao fisica nova (G0, plano 6.7): leva o torque muscular
// pedido pelo executor (generalizado, um por eixo habilitado) aos motores do
// backend. O executor e o unico escritor; a gravidade e compensada uma vez so
// (pelo modelo), entao a compensacao propria do backend fica desligada.
enum class ActuationMode3D {
    // (A) motor desligado (k = d = 0): o torque vai inteiro como par pai/filho.
    ExplicitTorque,
    // (A') o mesmo, mais so o amortecimento implicito do Jolt.
    ExplicitTorqueImplicitDamping,
    // (B) uma mola implicita baixa em volta da referencia e o feedforward
    // que desconta a resposta prevista dela.
    LowImpedance,
};

struct ActuationSettings3D {
    ActuationMode3D mode = ActuationMode3D::ExplicitTorque;
    // Amortecimento (A') e rigidez/amortecimento (B), em N.m.s/rad e N.m/rad
    // por kg.m2 de inercia efetiva do eixo: a impedancia acompanha o corpo.
    double dampingPerInertia = 20.0;
    double stiffnessPerInertia = 150.0;
    // (B) com a rigidez em fracao da do perfil (e nao por inercia): a mola
    // implicita estabiliza o laco fechado pe-chao dentro do solver. Maior que
    // zero liga este modo; o amortecimento segue a mesma fracao.
    double profileStiffnessFraction = 0.0;
    double timeStep = 1.0 / 120.0;
};

struct ActuationReference3D {
    // Coordenadas e taxas do modelo (por link), as do alvo da mola em (B).
    std::vector<Eigen::Vector3d> coordinates;
    std::vector<Eigen::Vector3d> rates;
};

// Os alvos de motor para o torque generalizado pedido (eixos das juntas),
// no estado atual. `effectiveInertia` e a diagonal da matriz de massa (a
// inercia que o eixo enxerga), usada para a impedancia por inercia.
[[nodiscard]] std::vector<RagdollDriveTarget3D> actuationTargets3D(const RagdollProfile3D& profile,
    const ArticulationModel3D& model, const ArticulationState3D& state, const Eigen::VectorXd& generalizedTorque,
    const ActuationReference3D& reference, const Eigen::VectorXd& effectiveInertia, const ActuationSettings3D& settings);

// O torque muscular entregue no ultimo passo (motor + feedforward, no frame
// da junta do filho, so eixos habilitados) e o pior estouro do teto em N.m.
struct ActuationDelivery3D {
    std::vector<Eigen::Vector3d> muscleTorque;
    double worstBudgetExcessNewtonMeters = 0.0;
};
[[nodiscard]] ActuationDelivery3D actuationDelivery3D(const RagdollProfile3D& profile, const RagdollState3D& state);

} // namespace MatterEngine
