#include "Engine/Control/WholeBodyController3D.hpp"

#include <Eigen/Core>
#include <proxsuite/proxqp/dense/dense.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <utility>

namespace MatterEngine {
namespace {

using Matrix = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>;
using Vector = Eigen::Matrix<double, Eigen::Dynamic, 1>;

constexpr double Infinity = 1.0e20;

Vec3 orientationError(Quaternion current, Quaternion desired) {
    const Vec3 currentForward = current.rotate({ 1.0f, 0.0f, 0.0f });
    const Vec3 currentLeft = current.rotate({ 0.0f, 1.0f, 0.0f });
    const Vec3 currentUp = current.rotate({ 0.0f, 0.0f, 1.0f });
    const Vec3 desiredForward = desired.rotate({ 1.0f, 0.0f, 0.0f });
    const Vec3 desiredLeft = desired.rotate({ 0.0f, 1.0f, 0.0f });
    const Vec3 desiredUp = desired.rotate({ 0.0f, 0.0f, 1.0f });
    return (cross(currentForward, desiredForward)
        + cross(currentLeft, desiredLeft)
        + cross(currentUp, desiredUp)) * 0.5f;
}

double component(Vec3 value, std::size_t index) {
    if (index == 0) return value.x;
    if (index == 1) return value.y;
    return value.z;
}

float maximumAbs(const Vector& values, Eigen::Index begin,
    Eigen::Index count) {
    float result = 0.0f;
    for (Eigen::Index index = 0; index < count; ++index) {
        result = std::max(result,
            static_cast<float>(std::abs(values[begin + index])));
    }
    return result;
}

std::size_t findLink(
    const RagdollProfile3D& profile, const char* id) {
    for (std::size_t index = 0; index < profile.links.size(); ++index) {
        if (profile.links[index].id == id) return index;
    }
    return profile.links.size();
}

} // namespace

struct WholeBodyController3D::Impl {
    std::unique_ptr<proxsuite::proxqp::dense::QP<double>> qp;
    Eigen::Index variables = 0;
    Eigen::Index equalities = 0;
    Eigen::Index inequalities = 0;
};

WholeBodyController3D::WholeBodyController3D()
    : m_impl(std::make_unique<Impl>()) {}

WholeBodyController3D::~WholeBodyController3D() = default;
WholeBodyController3D::WholeBodyController3D(
    WholeBodyController3D&&) noexcept = default;
WholeBodyController3D& WholeBodyController3D::operator=(
    WholeBodyController3D&&) noexcept = default;

void WholeBodyController3D::reset(Quaternion facingOrientation) {
    m_facingOrientation = facingOrientation.normalized();
    m_contactAnchored.fill(false);
    m_hasPreviousSwingTarget.fill(false);
    for (std::vector<float>& jacobian : m_previousContactJacobian) {
        jacobian.clear();
    }
    m_telemetry = {};
    m_impl->qp.reset();
    m_impl->variables = 0;
    m_impl->equalities = 0;
    m_impl->inequalities = 0;
}

bool WholeBodyController3D::update(
    const RagdollProfile3D& profile, const RagdollState3D& state,
    const RagdollDynamics3D& dynamics,
    bool leftFootSupported, bool rightFootSupported,
    std::size_t leftFootLink, std::size_t rightFootLink,
    Vec3 supportCenter, const WholeBodyMotionIntent3D& intent,
    float deltaTime,
    std::vector<RagdollDriveTarget3D>& targets) {
    m_telemetry = {};
    m_telemetry.dynamicsAvailable = dynamics.valid;
    if (!dynamics.valid || state.links.empty()
        || dynamics.generalizedDofCount < 7
        || dynamics.massMatrix.size()
            != static_cast<std::size_t>(
                dynamics.generalizedDofCount)
                * dynamics.generalizedDofCount
        || dynamics.biasForce.size()
            != dynamics.generalizedDofCount
        || dynamics.jacobianColumnCount
            != dynamics.generalizedDofCount
        || dynamics.centroidalMomentumMatrix.size()
            != static_cast<std::size_t>(6u)
                * dynamics.generalizedDofCount
        || dynamics.generalizedVelocity.size()
            != dynamics.generalizedDofCount) {
        return false;
    }

    const std::array<std::size_t, 2> footLinks {
        leftFootLink, rightFootLink
    };
    const std::array<bool, 2> supported {
        leftFootSupported, rightFootSupported
    };
    std::array<int, 2> contactSlot { -1, -1 };
    std::array<std::array<double, 6>, 2> contactKinematicBias {};
    std::array<std::array<double, 6>, 2> contactVelocity {};
    std::array<std::vector<float>, 2> contactJacobian;
    int contactCount = 0;
    for (std::size_t side = 0; side < 2; ++side) {
        if (!supported[side] || footLinks[side] >= state.links.size()) {
            m_contactAnchored[side] = false;
            m_previousContactJacobian[side].clear();
            continue;
        }
        contactSlot[side] = contactCount++;
        if (!m_contactAnchored[side]) {
            m_contactOrientation[side] =
                state.links[footLinks[side]].orientation;
        }
        m_contactAnchored[side] = true;
        const std::uint32_t firstRow =
            footLinks[side] < dynamics.linkJacobianRow.size()
            ? dynamics.linkJacobianRow[footLinks[side]]
            : RagdollDynamics3D::InvalidIndex;
        const std::size_t blockSize =
            static_cast<std::size_t>(6u)
            * dynamics.generalizedDofCount;
        if (firstRow != RagdollDynamics3D::InvalidIndex
            && firstRow + 6u <= dynamics.jacobianRowCount) {
            // computeDenseJacobian publica a velocidade no centro de massa do
            // link. A reação do chão, porém, atua na sola. Usar diretamente
            // J_com e depois limitar o momento como se o wrench estivesse na
            // sola omitia r×f da dinâmica: durante apoio simples o QP podia
            // escolher uma força lateral matematicamente válida no COM, mas
            // fisicamente errada no pé. Transformamos as três linhas lineares
            // para o centro da sola:
            //
            //   v_sola = v_com + omega × (p_sola - p_com)
            //
            // As linhas angulares permanecem iguais. Assim tanto a igualdade
            // cinemática quanto Jᵀ*lambda e os limites de CoP descrevem o
            // mesmo ponto físico.
            const PhysicsBodyState3D& foot =
                state.links[footLinks[side]];
            const RagdollLinkDefinition3D& definition =
                profile.links[footLinks[side]];
            const Vec3 centerOfMass = foot.position
                + foot.orientation.rotate(definition.centerOfMassLocal);
            const Vec3 soleCenter = foot.position
                + foot.orientation.rotate(
                    definition.collider.localPosition)
                - foot.orientation.rotate({
                    0.0f, 0.0f,
                    definition.collider.boxHalfExtents.z });
            const Vec3 lever = soleCenter - centerOfMass;
            contactJacobian[side].assign(blockSize, 0.0f);
            for (std::size_t column = 0;
                    column < dynamics.generalizedDofCount; ++column) {
                const Vec3 linear {
                    dynamics.denseJacobian[
                        static_cast<std::size_t>(firstRow)
                            * dynamics.jacobianColumnCount + column],
                    dynamics.denseJacobian[
                        static_cast<std::size_t>(firstRow + 1u)
                            * dynamics.jacobianColumnCount + column],
                    dynamics.denseJacobian[
                        static_cast<std::size_t>(firstRow + 2u)
                            * dynamics.jacobianColumnCount + column]
                };
                const Vec3 angular {
                    dynamics.denseJacobian[
                        static_cast<std::size_t>(firstRow + 3u)
                            * dynamics.jacobianColumnCount + column],
                    dynamics.denseJacobian[
                        static_cast<std::size_t>(firstRow + 4u)
                            * dynamics.jacobianColumnCount + column],
                    dynamics.denseJacobian[
                        static_cast<std::size_t>(firstRow + 5u)
                            * dynamics.jacobianColumnCount + column]
                };
                const Vec3 pointLinear =
                    linear + cross(angular, lever);
                for (std::size_t axis = 0; axis < 3; ++axis) {
                    contactJacobian[side][
                        axis * dynamics.generalizedDofCount + column] =
                        static_cast<float>(component(pointLinear, axis));
                    contactJacobian[side][
                        (axis + 3u) * dynamics.generalizedDofCount
                            + column] =
                        static_cast<float>(component(angular, axis));
                }
            }
            for (std::size_t spatial = 0; spatial < 6; ++spatial) {
                for (std::size_t column = 0;
                        column < dynamics.generalizedDofCount; ++column) {
                    contactVelocity[side][spatial] +=
                        static_cast<double>(contactJacobian[side][
                            spatial * dynamics.generalizedDofCount
                                + column])
                        * dynamics.generalizedVelocity[column];
                }
            }
            if (m_previousContactJacobian[side].size() == blockSize
                && deltaTime > 1.0e-5f) {
                for (std::size_t spatial = 0; spatial < 6; ++spatial) {
                    double bias = 0.0;
                    for (std::size_t column = 0;
                            column < dynamics.generalizedDofCount;
                            ++column) {
                        const float current =
                            contactJacobian[side][
                                spatial * dynamics.generalizedDofCount
                                    + column];
                        const float previous =
                            m_previousContactJacobian[side][
                                spatial
                                    * dynamics.generalizedDofCount
                                + column];
                        bias += static_cast<double>(current - previous)
                            / deltaTime
                            * dynamics.generalizedVelocity[column];
                    }
                    contactKinematicBias[side][spatial] = bias;
                }
            }
            m_previousContactJacobian[side] = contactJacobian[side];
        }
    }
    m_telemetry.contactCount =
        static_cast<std::uint32_t>(contactCount);
    if (contactCount == 0) return false;

    const Vec3 desiredHorizontalVelocity {
        intent.desiredHorizontalVelocity.x,
        intent.desiredHorizontalVelocity.y,
        0.0f
    };
    const float locomotionSpeed =
        desiredHorizontalVelocity.length();
    const bool locomotionActive =
        intent.locomotionActive || locomotionSpeed > 0.05f;
    const Eigen::Index generalized =
        static_cast<Eigen::Index>(dynamics.generalizedDofCount);
    const Eigen::Index jointDofs =
        static_cast<Eigen::Index>(dynamics.jointDofCount);
    const Eigen::Index torqueOffset = generalized;
    const Eigen::Index contactOffset = torqueOffset + jointDofs;
    const Eigen::Index centroidalSlackOffset =
        contactOffset + contactCount * 6;
    // Durante uma passada, os tres eixos lineares do COM pertencem a mesma
    // prioridade centroidal.
    // prioridade centroidal. A versao antiga promovia somente X/Y a
    // igualdade com folga e deixava Z como custo quadratico secundario. O QP
    // podia entao encontrar uma solucao horizontalmente boa levantando muito
    // pouco peso na perna de apoio: o torax continuava vertical enquanto a
    // bacia afundava. A terceira folga preserva factibilidade sem transformar
    // sustentacao em uma forca externa -- ela ainda precisa ser produzida
    // pelos contatos e torques articulares do modelo flutuante.
    const Eigen::Index centroidalSlackCount =
        locomotionActive ? 3 : 0;
    const Eigen::Index variableCount =
        centroidalSlackOffset + centroidalSlackCount;
    // Uma sola apoiada impede translacao no ponto de contato, mas nao e uma
    // junta soldada ao mundo. Roll/pitch/yaw sao controlados como uma tarefa
    // forte e cedem quando torque, atrito ou CoP chegam ao limite. Impor os
    // seis eixos como igualdades tornava o QP matematicamente inviavel assim
    // que um impacto fazia o pe de apoio bascular alguns graus; o controlador
    // entao perdia TODA a atuacao no momento mais importante da recuperacao.
    constexpr Eigen::Index ContactTranslationRows = 3;
    const Eigen::Index centroidalEqualityOffset =
        generalized + contactCount * ContactTranslationRows;
    const Eigen::Index equalityCount =
        centroidalEqualityOffset + centroidalSlackCount;
    // torque box + 11 faces de pirâmide/centro de pressão por contato.
    const Eigen::Index centroidalSlackBoundCount =
        locomotionActive ? 2 : 0;
    const Eigen::Index inequalityCount =
        jointDofs + contactCount * 11 + centroidalSlackBoundCount;

    Matrix hessian = Matrix::Zero(variableCount, variableCount);
    Vector gradient = Vector::Zero(variableCount);
    Matrix equality = Matrix::Zero(equalityCount, variableCount);
    Vector equalityTarget = Vector::Zero(equalityCount);
    Matrix inequality = Matrix::Zero(
        inequalityCount, variableCount);
    Vector lower = Vector::Constant(inequalityCount, -Infinity);
    Vector upper = Vector::Constant(inequalityCount, Infinity);

    // Dinâmica flutuante:
    // M*qdd + h = S^T*tau + Jc^T*lambda.
    for (Eigen::Index row = 0; row < generalized; ++row) {
        for (Eigen::Index column = 0;
                column < generalized; ++column) {
            equality(row, column) =
                dynamics.massMatrix[
                    static_cast<std::size_t>(row * generalized + column)];
        }
        equalityTarget[row] =
            -dynamics.biasForce[static_cast<std::size_t>(row)];
    }
    for (Eigen::Index dof = 0; dof < jointDofs; ++dof) {
        equality(6 + dof, torqueOffset + dof) = -1.0;
    }

    for (std::size_t side = 0; side < 2; ++side) {
        if (contactSlot[side] < 0) continue;
        const std::size_t link = footLinks[side];
        if (link >= dynamics.linkJacobianRow.size()) return false;
        const std::uint32_t firstRow =
            dynamics.linkJacobianRow[link];
        if (firstRow == RagdollDynamics3D::InvalidIndex
            || firstRow + 6u > dynamics.jacobianRowCount) {
            return false;
        }
        const Eigen::Index slot = contactSlot[side];
        const Eigen::Index wrench = contactOffset + slot * 6;
        const Eigen::Index contactEquality =
            generalized + slot * ContactTranslationRows;
        for (Eigen::Index spatial = 0; spatial < 6; ++spatial) {
            for (Eigen::Index column = 0;
                    column < generalized; ++column) {
                const double coefficient =
                    contactJacobian[side][
                        static_cast<std::size_t>(spatial)
                            * dynamics.generalizedDofCount
                        + static_cast<std::size_t>(column)];
                equality(column, wrench + spatial) =
                    -coefficient;
                if (spatial < ContactTranslationRows) {
                    equality(contactEquality + spatial, column) =
                        coefficient;
                }
            }
            // Contato de superfície sem escorregamento translacional:
            // J*qdd + Jdot*qdot = -Kd*(J*qdot).
            //
            // Impor apenas aceleração zero preservava a velocidade residual
            // existente no instante do touchdown. A sola então deslizava
            // lentamente por vários segundos e o alvo de suporte fugia do
            // corpo. O termo dissipativo não inventa uma posição nem concorre
            // com a colisão do PhysX: ele leva a velocidade do MESMO ponto de
            // contato a zero. Mantemos a correção translacional um pouco mais
            // firme para assentar rapido. A orientacao nao e uma igualdade
            // soldada: logo abaixo ela vira tarefa de torque/CoP, permitindo
            // que a sola bascule e volte a assentar durante um impacto.
            const double contactDamping = spatial < 3 ? 24.0 : 16.0;
            const double rawContactAcceleration =
                -contactKinematicBias[side][
                    static_cast<std::size_t>(spatial)]
                - contactDamping * contactVelocity[side][
                    static_cast<std::size_t>(spatial)];
            const double contactAccelerationLimit =
                spatial < 3 ? 14.0 : 24.0;
            const double desiredContactAcceleration = std::clamp(
                rawContactAcceleration,
                -contactAccelerationLimit, contactAccelerationLimit);
            if (spatial < ContactTranslationRows) {
                equalityTarget[contactEquality + spatial] =
                    desiredContactAcceleration;
            } else {
                // A orientação da planta continua firme, porém é uma tarefa
                // de impedância, não uma impossibilidade geométrica. O erro
                // para a orientação registrada no heel-strike recompõe toda
                // a planta depois de uma basculada sem invalidar dinâmica,
                // torque ou a pirâmide de atrito.
                const Vec3 angularCorrection = orientationError(
                    state.links[link].orientation,
                    m_contactOrientation[side]);
                const double desiredAngularAcceleration = std::clamp(
                    static_cast<double>(component(angularCorrection,
                        static_cast<std::size_t>(spatial - 3))) * 72.0
                        + desiredContactAcceleration,
                    -90.0, 90.0);
                Eigen::RowVectorXd angularTask(generalized);
                for (Eigen::Index column = 0;
                        column < generalized; ++column) {
                    angularTask[column] = contactJacobian[side][
                        static_cast<std::size_t>(spatial)
                            * dynamics.generalizedDofCount
                        + static_cast<std::size_t>(column)];
                }
                constexpr double PlantedSoleOrientationWeight =
                    8'000'000.0;
                hessian.topLeftCorner(generalized, generalized)
                    .noalias() += PlantedSoleOrientationWeight
                        * angularTask.transpose() * angularTask;
                gradient.head(generalized).noalias() -=
                    PlantedSoleOrientationWeight
                    * angularTask.transpose()
                    * desiredAngularAcceleration;
            }
        }
    }

    // Objetivo centroidal. Estabilizar somente a raiz/bacia deixa braços e
    // tronco livres para levar o COM para fora da base. A matriz A fornecida
    // pelo PhysX transforma qdot em momento do corpo inteiro; sua derivada
    // A*qdd+b permite controlar diretamente COM e momento angular.
    const PhysicsBodyState3D& pelvis = state.links.front();
    const double totalMass = std::max(
        1.0, static_cast<double>(profile.totalMassKg));
    Matrix centroidal(6, generalized);
    Vector generalizedVelocity(generalized);
    for (Eigen::Index column = 0; column < generalized; ++column) {
        generalizedVelocity[column] =
            dynamics.generalizedVelocity[
                static_cast<std::size_t>(column)];
        for (Eigen::Index row = 0; row < 6; ++row) {
            centroidal(row, column) =
                dynamics.centroidalMomentumMatrix[
                    static_cast<std::size_t>(
                        row * generalized + column)];
        }
    }
    const Vector momentum = centroidal * generalizedVelocity;
    const Vec3 centerOfMassVelocity {
        static_cast<float>(momentum[0] / totalMass),
        static_cast<float>(momentum[1] / totalMass),
        static_cast<float>(momentum[2] / totalMass)
    };
    m_telemetry.measuredCenterOfMass = dynamics.centerOfMass;
    m_telemetry.measuredCenterOfMassVelocity = centerOfMassVelocity;
    m_telemetry.desiredHorizontalVelocity =
        desiredHorizontalVelocity;
    const float horizontalPositionGain =
        locomotionActive ? 18.0f : m_config.pelvisPositionGain;
    const float horizontalVelocityGain =
        locomotionActive ? 8.0f : m_config.pelvisVelocityGain;
    Vec3 centerOfMassTarget = supportCenter
        + desiredHorizontalVelocity
            * (locomotionActive ? 0.12f : 0.0f);
    centerOfMassTarget.z = supportCenter.z
        + m_config.nominalCenterOfMassHeightMeters;
    m_telemetry.desiredCenterOfMass = centerOfMassTarget;
    Vec3 centerOfMassAcceleration {
        (centerOfMassTarget.x - dynamics.centerOfMass.x)
                * horizontalPositionGain
            + (desiredHorizontalVelocity.x
                    - centerOfMassVelocity.x)
                * horizontalVelocityGain,
        (centerOfMassTarget.y - dynamics.centerOfMass.y)
                * horizontalPositionGain
            + (desiredHorizontalVelocity.y
                    - centerOfMassVelocity.y)
                * horizontalVelocityGain,
        (centerOfMassTarget.z - dynamics.centerOfMass.z)
                * m_config.pelvisVerticalPositionGain
            - centerOfMassVelocity.z
                * m_config.pelvisVerticalVelocityGain
    };
    centerOfMassAcceleration.x +=
        intent.desiredContactAccelerationWorld.x;
    centerOfMassAcceleration.y +=
        intent.desiredContactAccelerationWorld.y;
    if (locomotionActive) {
        // Regulador de posição/velocidade sobre o modelo do pêndulo
        // invertido:
        //
        //   c_ddot = omega² * (c - zmp)
        //
        // O PD acima expressa a intenção de transferência/marcha. Convertê-lo
        // diretamente em igualdade centroidal permitia pedir acelerações cujo
        // ZMP estaria metros fora dos pés. O QP preservava a física, mas a
        // folga dessa tarefa impossível dominava postura e pé em voo. Aqui
        // obtemos o ZMP implícito, projetamos no polígono conservador dos pés
        // realmente apoiados e só então reconstruímos uma aceleração
        // fisicamente realizável.
        const float height = std::max(
            0.45f, dynamics.centerOfMass.z - supportCenter.z);
        const float omegaSquared = 9.81f / height;
        const Vec3 requestedAcceleration = centerOfMassAcceleration;
        const Vec3 requestedZmp {
            dynamics.centerOfMass.x
                - requestedAcceleration.x / omegaSquared,
            dynamics.centerOfMass.y
                - requestedAcceleration.y / omegaSquared,
            supportCenter.z
        };
        Vec3 footForward = m_facingOrientation.rotate(
            { 1.0f, 0.0f, 0.0f });
        footForward.z = 0.0f;
        footForward = footForward.lengthSquared() > 1.0e-8f
            ? footForward.normalized() : Vec3 { 1.0f, 0.0f, 0.0f };
        const Vec3 footLateral {
            -footForward.y, footForward.x, 0.0f
        };

        float minimumForward = std::numeric_limits<float>::max();
        float maximumForward = -std::numeric_limits<float>::max();
        float minimumLateral = std::numeric_limits<float>::max();
        float maximumLateral = -std::numeric_limits<float>::max();
        for (std::size_t side = 0; side < 2; ++side) {
            if (!supported[side] || footLinks[side] >= state.links.size()
                || footLinks[side] >= profile.links.size()) {
                continue;
            }
            const PhysicsBodyState3D& foot = state.links[footLinks[side]];
            const RagdollCapsuleDefinition3D& collider =
                profile.links[footLinks[side]].collider;
            const Vec3 sole = foot.position
                + foot.orientation.rotate(collider.localPosition)
                - foot.orientation.rotate({
                    0.0f, 0.0f, collider.boxHalfExtents.z });
            Vec3 soleForward =
                foot.orientation.rotate({ 1.0f, 0.0f, 0.0f });
            soleForward.z = 0.0f;
            soleForward = soleForward.lengthSquared() > 1.0e-8f
                ? soleForward.normalized() : footForward;
            const Vec3 soleLateral {
                -soleForward.y, soleForward.x, 0.0f
            };
            for (float forwardSign : { -1.0f, 1.0f }) {
                for (float lateralSign : { -1.0f, 1.0f }) {
                    const Vec3 corner = sole
                        + soleForward
                            * (collider.boxHalfExtents.x * forwardSign)
                        + soleLateral
                            * (collider.boxHalfExtents.y * lateralSign);
                    minimumForward = std::min(
                        minimumForward, dot(corner, footForward));
                    maximumForward = std::max(
                        maximumForward, dot(corner, footForward));
                    minimumLateral = std::min(
                        minimumLateral, dot(corner, footLateral));
                    maximumLateral = std::max(
                        maximumLateral, dot(corner, footLateral));
                }
            }
        }
        // Não usamos a borda geométrica exata: a margem mantém o CoP longe
        // das quinas, onde uma caixa de pé fisicamente já estaria basculando.
        constexpr float ForwardMargin = 0.012f;
        constexpr float LateralMargin = 0.010f;
        minimumForward += ForwardMargin;
        maximumForward -= ForwardMargin;
        minimumLateral += LateralMargin;
        maximumLateral -= LateralMargin;
        const float admissibleForward = std::clamp(
            dot(requestedZmp, footForward),
            minimumForward, maximumForward);
        const float admissibleLateral = std::clamp(
            dot(requestedZmp, footLateral),
            minimumLateral, maximumLateral);
        const Vec3 admissibleZmp =
            footForward * admissibleForward
            + footLateral * admissibleLateral
            + Vec3 { 0.0f, 0.0f, supportCenter.z };
        centerOfMassAcceleration.x =
            (dynamics.centerOfMass.x - admissibleZmp.x) * omegaSquared;
        centerOfMassAcceleration.y =
            (dynamics.centerOfMass.y - admissibleZmp.y) * omegaSquared;
    }
    const Vec3 orientationCorrection =
        orientationError(pelvis.orientation, m_facingOrientation);
    constexpr double NominalCentroidalInertia = 8.0;
    const Vec3 desiredAngularMomentumRate {
        orientationCorrection.x * m_config.pelvisOrientationGain
                * static_cast<float>(NominalCentroidalInertia)
            - static_cast<float>(momentum[3])
                * m_config.pelvisAngularVelocityGain,
        orientationCorrection.y * m_config.pelvisOrientationGain
                * static_cast<float>(NominalCentroidalInertia)
            - static_cast<float>(momentum[4])
                * m_config.pelvisAngularVelocityGain,
        orientationCorrection.z * m_config.pelvisOrientationGain
                * static_cast<float>(NominalCentroidalInertia)
            - static_cast<float>(momentum[5])
                * m_config.pelvisAngularVelocityGain
    };
    m_telemetry.desiredRootLinearAcceleration =
        centerOfMassAcceleration;
    for (Eigen::Index row = 0; row < 6; ++row) {
        const bool linear = row < 3;
        const double scale = linear
            ? totalMass : NominalCentroidalInertia;
        const double desiredRate = linear
            ? totalMass * component(centerOfMassAcceleration,
                static_cast<std::size_t>(row))
            : component(desiredAngularMomentumRate,
                static_cast<std::size_t>(row - 3));
        const double bias =
            dynamics.centroidalMomentumBias[
                static_cast<std::size_t>(row)];
        const double taskLimit = linear
            ? (row == 2 ? 16.0
                        : locomotionActive ? 6.8 : 10.0)
            : 45.0;
        const double desired = std::clamp(
            (desiredRate - bias) / scale,
            -taskLimit, taskLimit);
        const Eigen::RowVectorXd task =
            centroidal.row(row) / scale;
        if (locomotionActive && row < 3) {
            // Prioridade centroidal explícita. Codificar isto como
            // 50 milhões * ||A*qdd-a*||² deixava o Hessiano mal condicionado
            // e o ProxQP declarava convergência com resíduos duais enormes.
            // A igualdade com folga mantém a mesma hierarquia sem formar
            // AᵀA em escala extrema; a folga torna a tarefa sempre factível
            // diante de limites de torque/atrito.
            const Eigen::Index taskRow =
                centroidalEqualityOffset + row;
            equality.block(taskRow, 0, 1, generalized) = task;
            equalityTarget[taskRow] = desired;
            equality(taskRow, centroidalSlackOffset + row) = 1.0;
            if (row == 2) {
                // A sustentação vertical precisa de folga para mudanças de
                // contato e limites de torque. Horizontalmente, porém, o ZMP
                // já foi projetado no polígono físico das solas: essa tarefa
                // é admissível e deve permanecer hierarquicamente acima de
                // pé em voo, coluna e cabeça. Permitir folga em X/Y fazia o
                // QP escolher acelerar no sentido da queda para satisfazer a
                // trajetória de um membro — exatamente o oposto do controle
                // de captura.
                hessian(centroidalSlackOffset + row,
                    centroidalSlackOffset + row) = 18'000'000.0;
            } else {
                hessian(centroidalSlackOffset + row,
                    centroidalSlackOffset + row) = 4'000'000.0;
            }
        } else {
            const bool horizontalAngularMomentum = row == 3 || row == 4;
            const double weight = locomotionActive
                ? (row == 2 ? 120'000.0
                    : horizontalAngularMomentum ? 2'400'000.0
                    : 95'000.0)
                : (row == 2 ? 1'600.0
                            : linear ? 1'350.0 : 900.0);
            hessian.topLeftCorner(generalized, generalized)
                .noalias() += weight * task.transpose() * task;
            gradient.head(generalized).noalias() -=
                weight * task.transpose() * desired;
        }
    }

    // A tarefa de momento angular acima estabiliza o corpo como um todo, mas
    // não impede uma distribuição ruim desse momento: pernas/cabeça podem
    // compensar enquanto a própria bacia gira para fora da base. Como a
    // bacia define quadris e postura, controlamos diretamente roll/pitch do
    // seu frame. Ainda é uma tarefa suave do QP: só torques articulares e
    // reações reais dos pés podem realizá-la.
    if (!dynamics.linkJacobianRow.empty()) {
        const std::uint32_t pelvisFirstRow =
            dynamics.linkJacobianRow.front();
        if (pelvisFirstRow != RagdollDynamics3D::InvalidIndex
            && pelvisFirstRow + 6u <= dynamics.jacobianRowCount) {
            const Vec3 pelvisAngularAcceleration =
                orientationCorrection * m_config.pelvisOrientationGain
                - pelvis.angularVelocity
                    * m_config.pelvisAngularVelocityGain;
            for (Eigen::Index axis = 0; axis < 2; ++axis) {
                Eigen::RowVectorXd task(generalized);
                for (Eigen::Index column = 0;
                        column < generalized; ++column) {
                    task[column] = dynamics.denseJacobian[
                        static_cast<std::size_t>(pelvisFirstRow + 3u + axis)
                            * dynamics.jacobianColumnCount
                        + static_cast<std::size_t>(column)];
                }
                const double desired = std::clamp(
                    component(pelvisAngularAcceleration,
                        static_cast<std::size_t>(axis)),
                    -40.0, 40.0);
                constexpr double PelvisOrientationWeight = 480'000.0;
                hessian.topLeftCorner(generalized, generalized)
                    .noalias() += PelvisOrientationWeight
                        * task.transpose() * task;
                gradient.head(generalized).noalias() -=
                    PelvisOrientationWeight * task.transpose() * desired;
            }
        }
    }

    // O momento centroidal mantém o corpo inteiro estável, mas não determina
    // sozinho como esse momento se distribui pela coluna. O elo distal da
    // cadeia axial recebe uma tarefa espacial; as articulações intermediárias
    // permanecem preferências de pose para não criar quatro tarefas
    // cartesianas redundantes sobre os mesmos poucos DOFs.
    for (const char* torsoId : { "UpperChest" }) {
        const std::size_t torsoLink = findLink(profile, torsoId);
        if (torsoLink >= state.links.size()
            || torsoLink >= dynamics.linkJacobianRow.size()) {
            continue;
        }
        const std::uint32_t firstRow =
            dynamics.linkJacobianRow[torsoLink];
        if (firstRow != RagdollDynamics3D::InvalidIndex
            && firstRow + 6u <= dynamics.jacobianRowCount) {
            const PhysicsBodyState3D& torso =
                state.links[torsoLink];
            const Vec3 torsoUp = torso.orientation.rotate(
                { 0.0f, 0.0f, 1.0f }).normalized();
            Vec3 desiredTorsoUp = intent.desiredTorsoUpWorld;
            desiredTorsoUp = desiredTorsoUp.lengthSquared() > 1.0e-8f
                ? desiredTorsoUp.normalized()
                : Vec3 { 0.0f, 0.0f, 1.0f };
            const Vec3 uprightError =
                cross(torsoUp, desiredTorsoUp);
            const float reactiveGain = 1.0f
                + std::clamp(intent.balanceRecoveryUrgency,
                    0.0f, 1.0f) * 0.45f
                + (locomotionActive ? 0.55f : 0.0f);
            const Vec3 angularAcceleration =
                uprightError * (108.0f * reactiveGain)
                - Vec3 { torso.angularVelocity.x,
                    torso.angularVelocity.y, 0.0f }
                    * (locomotionActive ? 34.0f : 20.0f);
            for (Eigen::Index axis = 0; axis < 2; ++axis) {
                Eigen::RowVectorXd task(generalized);
                for (Eigen::Index column = 0;
                        column < generalized; ++column) {
                    task[column] = dynamics.denseJacobian[
                        static_cast<std::size_t>(firstRow + 3u + axis)
                            * dynamics.jacobianColumnCount
                        + static_cast<std::size_t>(column)];
                }
                const double torsoAccelerationLimit =
                    intent.balanceRecoveryUrgency > 0.75f ? 60.0 : 35.0;
                const double desired = std::clamp(
                    component(angularAcceleration,
                        static_cast<std::size_t>(axis)),
                    -torsoAccelerationLimit, torsoAccelerationLimit);
                // A transferência do COM deve vir de tornozelos, joelhos e
                // quadris. Com peso comparável ao pé em voo, o otimizador
                // descobria um atalho barato: inclinava toda a coluna para
                // deslocar massa lateralmente. Mantemos esta tarefa abaixo
                // das igualdades de dinâmica/contato/COM, mas claramente
                // acima das preferências articulares secundárias.
                const double torsoOrientationWeight =
                    intent.balanceRecoveryUrgency > 0.75f
                    ? 4'000'000.0
                    : locomotionActive ? 2'200'000.0 : 520'000.0;
                hessian.topLeftCorner(generalized, generalized)
                    .noalias() += torsoOrientationWeight
                        * task.transpose() * task;
                gradient.head(generalized).noalias() -=
                    torsoOrientationWeight * task.transpose() * desired;
            }
        }
    }

    // Reflexo vestibulo-cervical: estabiliza a cabeça em espaço global,
    // independentemente da inclinação deliberada do tórax. Uma única tarefa
    // no elo distal deixa o QP distribuir o movimento entre pescoço, cabeça e
    // coluna, evitando dois servos cartesianos redundantes na mesma cadeia.
    const std::size_t headLink = findLink(profile, "Head");
    if (headLink < state.links.size()
        && headLink < dynamics.linkJacobianRow.size()) {
        const std::uint32_t firstRow =
            dynamics.linkJacobianRow[headLink];
        if (firstRow != RagdollDynamics3D::InvalidIndex
            && firstRow + 6u <= dynamics.jacobianRowCount) {
            const PhysicsBodyState3D& head = state.links[headLink];
            const Vec3 headUp = head.orientation.rotate(
                { 0.0f, 0.0f, 1.0f }).normalized();
            Vec3 desiredHeadUp = intent.desiredHeadUpWorld;
            desiredHeadUp = desiredHeadUp.lengthSquared() > 1.0e-8f
                ? desiredHeadUp.normalized()
                : Vec3 { 0.0f, 0.0f, 1.0f };
            const float reactiveGain = 1.0f
                + std::clamp(intent.balanceRecoveryUrgency,
                    0.0f, 1.0f) * 0.30f
                + (locomotionActive ? 0.40f : 0.0f);
            const Vec3 angularAcceleration =
                cross(headUp, desiredHeadUp)
                    * (m_config.headOrientationGain * reactiveGain)
                - Vec3 { head.angularVelocity.x,
                    head.angularVelocity.y, 0.0f }
                    * m_config.headAngularVelocityGain;
            for (Eigen::Index axis = 0; axis < 2; ++axis) {
                Eigen::RowVectorXd task(generalized);
                for (Eigen::Index column = 0;
                        column < generalized; ++column) {
                    task[column] = dynamics.denseJacobian[
                        static_cast<std::size_t>(firstRow + 3u + axis)
                            * dynamics.jacobianColumnCount
                        + static_cast<std::size_t>(column)];
                }
                const double headAccelerationLimit =
                    intent.balanceRecoveryUrgency > 0.75f ? 60.0 : 40.0;
                const double desired = std::clamp(
                    component(angularAcceleration,
                        static_cast<std::size_t>(axis)),
                    -headAccelerationLimit, headAccelerationLimit);
                const double headOrientationWeight =
                    intent.balanceRecoveryUrgency > 0.75f
                    ? 600'000.0
                    : locomotionActive ? 260'000.0 : 110'000.0;
                hessian.topLeftCorner(generalized, generalized)
                    .noalias() += headOrientationWeight
                        * task.transpose() * task;
                gradient.head(generalized).noalias() -=
                    headOrientationWeight * task.transpose() * desired;
            }
        }
    }

    // O pé em balanço é uma tarefa cartesiana do corpo inteiro. O planejador
    // fornece a sola; o WBC converte para o COM do link e distribui o esforço
    // por quadril, joelho e tornozelo através do Jacobiano, sem IK isolada.
    for (std::size_t side = 0; side < 2; ++side) {
        if (!intent.trackSwingFoot[side]
            || footLinks[side] >= state.links.size()
            || footLinks[side] >= dynamics.linkJacobianRow.size()) {
            m_hasPreviousSwingTarget[side] = false;
            continue;
        }
        const std::size_t link = footLinks[side];
        const std::uint32_t firstRow =
            dynamics.linkJacobianRow[link];
        if (firstRow == RagdollDynamics3D::InvalidIndex
            || firstRow + 6u > dynamics.jacobianRowCount) {
            continue;
        }
        FootPose3D sole = intent.desiredFootSoles[side];
        const bool urgentPlant = intent.urgentPlantFoot[side];
        if (urgentPlant) {
            const Vec3 groundNormal =
                sole.groundNormal.lengthSquared() > 1.0e-8f
                ? sole.groundNormal.normalized()
                : Vec3 { 0.0f, 0.0f, 1.0f };
            // Pequena pré-carga virtual no fim da curva para garantir que
            // o PhysX produza heel-strike, em vez de a sola permanecer
            // milímetros acima da superfície por vários frames.
            sole.position -= groundNormal * 0.014f;
        }
        const RagdollCapsuleDefinition3D& collider =
            profile.links[link].collider;
        const Vec3 desiredLinkPosition = sole.position
            + sole.orientation.rotate({
                0.0f, 0.0f, collider.boxHalfExtents.z })
            - sole.orientation.rotate(collider.localPosition);
        Vec3 desiredVelocity;
        if (m_hasPreviousSwingTarget[side] && deltaTime > 1.0e-5f) {
            desiredVelocity = (desiredLinkPosition
                - m_previousSwingTarget[side]) / deltaTime;
            const float speed = desiredVelocity.length();
            const float maximumSwingSpeed =
                intent.balanceRecoveryUrgency > 0.55f ? 5.6f : 3.4f;
            if (speed > maximumSwingSpeed) {
                desiredVelocity *= maximumSwingSpeed / speed;
            }
        }
        m_previousSwingTarget[side] = desiredLinkPosition;
        m_hasPreviousSwingTarget[side] = true;
        const PhysicsBodyState3D& foot = state.links[link];
        const Vec3 linearAcceleration =
            (desiredLinkPosition - foot.position)
                * (urgentPlant ? 260.0f
                    : locomotionActive ? 260.0f : 210.0f)
            + (desiredVelocity - foot.linearVelocity)
                * (urgentPlant ? 46.0f
                    : locomotionActive ? 46.0f : 29.0f);
        const Vec3 angularAcceleration =
            orientationError(foot.orientation, sole.orientation)
                * 135.0f
            - foot.angularVelocity * 22.0f;
        const double swingAccelerationLimit =
            intent.balanceRecoveryUrgency > 0.75f
            ? 240.0
            : static_cast<double>(m_config.generalizedAccelerationLimit);
        m_telemetry.desiredSwingFootAcceleration = linearAcceleration;
        m_telemetry.swingFootLink = static_cast<std::uint32_t>(link);
        for (Eigen::Index spatial = 0; spatial < 6; ++spatial) {
            Eigen::RowVectorXd task(generalized);
            for (Eigen::Index column = 0;
                    column < generalized; ++column) {
                task[column] = dynamics.denseJacobian[
                    static_cast<std::size_t>(firstRow + spatial)
                        * dynamics.jacobianColumnCount
                    + static_cast<std::size_t>(column)];
            }
            const double desired = std::clamp(
                spatial < 3
                    ? static_cast<double>(component(linearAcceleration,
                        static_cast<std::size_t>(spatial)))
                    : static_cast<double>(component(angularAcceleration,
                        static_cast<std::size_t>(spatial - 3))),
                -swingAccelerationLimit,
                swingAccelerationLimit);
            // Colocar o pé é tarefa primária de locomoção. Com pesos de
            // postura comuns (~10²), o objetivo centroidal podia preservar
            // o tronco sacrificando completamente o pé: a trajetória
            // terminava no planejador enquanto a sola física ainda estava
            // longe e o corpo caía antes do contato. A hierarquia numérica
            // mantém contato/dinâmica como igualdades e dá à translação do
            // pé em voo prioridade comparável à do COM.
            // A tarefa do pé não pode ganhar do equilíbrio centroidal. Na
            // versão anterior, três eixos a 210k cada superavam o objetivo do
            // COM (95k): acelerar o pé em voo para o lado arrastava o corpo
            // inteiro para fora da única sola de apoio. O pé continua sendo
            // uma tarefa primária, mas abaixo de COM/ZMP e acima da pose.
            const double baseWeight = urgentPlant
                ? (spatial < 3 ? 260'000.0 : 42'000.0)
                : locomotionActive
                // Com 12k, o planner chegava ao fim mas a sola física
                // aterrissava 25--35 cm antes do ponto de captura. Esta
                // prioridade segue abaixo da aquisição urgente e da tarefa
                // centroidal, mas permite que a perna cumpra o voo curto.
                ? (spatial < 3 ? 180'000.0 : 20'000.0)
                : (spatial < 3 ? 82'000.0 : 18'000.0);
            const double weight = baseWeight * std::clamp(
                static_cast<double>(
                    intent.swingFootTaskWeightScale[side]),
                0.10, 2.0);
            hessian.topLeftCorner(generalized, generalized)
                .noalias() += weight * task.transpose() * task;
            gradient.head(generalized).noalias() -=
                weight * task.transpose() * desired;
        }
    }

    // A pose é uma preferência secundária. Os targets vêm do perfil
    // biomecânico/planejador, mas o QP pode cedê-los para manter contatos e
    // dinâmica, em vez de criar motores concorrentes.
    for (const RagdollDriveTarget3D& target : targets) {
        if (target.linkIndex >= dynamics.jointGeneralizedDof.size()
            || target.linkIndex >= state.joints.size()) {
            continue;
        }
        const std::size_t axis =
            static_cast<std::size_t>(target.axis);
        const std::uint32_t generalizedIndex =
            dynamics.jointGeneralizedDof[target.linkIndex][axis];
        if (generalizedIndex == RagdollDynamics3D::InvalidIndex
            || generalizedIndex >= dynamics.generalizedDofCount) {
            continue;
        }
        const float positionError = target.positionRadians
            - state.joints[target.linkIndex].positionRadians[axis];
        const float desiredAcceleration = std::clamp(
            positionError * m_config.jointPositionGain
                - state.joints[target.linkIndex]
                    .velocityRadiansPerSecond[axis]
                    * m_config.jointVelocityGain,
            -m_config.generalizedAccelerationLimit,
            m_config.generalizedAccelerationLimit);
        const std::string& linkId =
            profile.links[target.linkIndex].id;
        const bool spine = linkId == "Abdomen"
            || linkId == "Chest" || linkId == "UpperChest";
        const bool headChain = linkId == "Neck" || linkId == "Head";
        const bool leg = linkId.find("Thigh") != std::string::npos
            || linkId.find("Shin") != std::string::npos
            || linkId.find("Foot") != std::string::npos;
        const bool leftSide = linkId.rfind("Left", 0) == 0;
        const bool rightSide = linkId.rfind("Right", 0) == 0;
        const bool externallyManipulatedLeg = leg
            && ((leftSide && intent.externallyManipulatedLeg[0])
                || (rightSide && intent.externallyManipulatedLeg[1]));
        if (externallyManipulatedLeg) {
            continue;
        }
        const bool unloadingLeg = leg
            && ((leftSide && intent.unloadingFoot[0])
                || (rightSide && intent.unloadingFoot[1]));
        // O planejador já avança sua curva durante a transferência de peso,
        // porém o pé físico ainda é apoio. Aplicar a pose futura nessa fase
        // fazia o motor levantar a ponta do pé contra a igualdade de contato.
        if (unloadingLeg
            && !((leftSide && intent.trackSwingFoot[0])
                || (rightSide && intent.trackSwingFoot[1]))) {
            continue;
        }
        // Com os drives paralelos corretamente desligados, esta é a única
        // preferência de pose articular. Ela precisa ter autoridade suficiente
        // para manter a cadeia da coluna e a flexão útil das pernas, mas segue
        // abaixo das tarefas primárias de COM/contato/pé em voo.
        const double weight = spine ? 64.0
            : headChain ? 48.0 : leg ? 12.0 : 4.0;
        hessian(generalizedIndex, generalizedIndex) += weight;
        gradient[generalizedIndex] -=
            weight * desiredAcceleration;

    }

    // Os reflexos internos de quadril/coluna/bracos sao uma preferencia de
    // torque do mesmo QP, nao uma segunda atuacao aplicada depois da solucao.
    // Assim o otimizador pode usa-los para redistribuir momento angular, mas
    // eles continuam sujeitos aos limites anatomicos e permanecem presentes
    // em M*qdd+h=S^T*tau+J^T*lambda. Somar esse termo depois do solver fazia
    // picos acima de 600 Nm e gerava exatamente o "chicote" tardio observado
    // apos um impacto.
    constexpr double InternalReflexTorqueWeight = 1.8e-3;
    for (const RagdollDriveTarget3D& target : targets) {
        if (target.linkIndex >= dynamics.jointGeneralizedDof.size()) continue;
        const std::size_t axis = static_cast<std::size_t>(target.axis);
        const std::uint32_t generalizedIndex =
            dynamics.jointGeneralizedDof[target.linkIndex][axis];
        if (generalizedIndex == RagdollDynamics3D::InvalidIndex) continue;
        const Eigen::Index dof =
            static_cast<Eigen::Index>(generalizedIndex - 6u);
        hessian(torqueOffset + dof, torqueOffset + dof) +=
            InternalReflexTorqueWeight;
        gradient[torqueOffset + dof] -=
            InternalReflexTorqueWeight
            * static_cast<double>(target.feedforwardTorqueNewtonMeters);
    }
    for (Eigen::Index dof = 0; dof < jointDofs; ++dof) {
        hessian(torqueOffset + dof, torqueOffset + dof) += 2.0e-4;
    }
    for (Eigen::Index index = contactOffset;
            index < centroidalSlackOffset; ++index) {
        hessian(index, index) += 1.0e-5;
    }
    hessian.diagonal().array() += 1.0e-8;

    Eigen::Index inequalityRow = 0;
    for (std::size_t link = 1;
            link < dynamics.jointGeneralizedDof.size(); ++link) {
        for (std::size_t axis = 0; axis < 3; ++axis) {
            const std::uint32_t generalizedIndex =
                dynamics.jointGeneralizedDof[link][axis];
            if (generalizedIndex == RagdollDynamics3D::InvalidIndex) continue;
            const Eigen::Index dof =
                static_cast<Eigen::Index>(generalizedIndex - 6u);
            inequality(inequalityRow, torqueOffset + dof) = 1.0;
            const std::string& linkId = profile.links[link].id;
            const bool axial = linkId == "Abdomen"
                || linkId == "Chest" || linkId == "UpperChest";
            const bool cervical = linkId == "Neck" || linkId == "Head";
            const float anatomicalAuthority = axial
                ? std::max(m_config.torqueLimitScale, 2.35f)
                : cervical
                    ? std::max(m_config.torqueLimitScale, 1.90f)
                    : m_config.torqueLimitScale;
            const float limit =
                profile.links[link].inboundJoint.axes[axis].maximumTorque
                * anatomicalAuthority;
            lower[inequalityRow] = -limit;
            upper[inequalityRow] = limit;
            ++inequalityRow;
        }
    }

    const double maximumNormal =
        profile.totalMassKg * 9.81
        * m_config.maximumContactForceWeightScale;
    const double friction = m_config.frictionCoefficient;
    const double halfWidth = 0.0575;
    const double halfLength = 0.14;
    const double torsionalFriction = 0.035;
    for (int contact = 0; contact < contactCount; ++contact) {
        const Eigen::Index wrench = contactOffset + contact * 6;
        // fz
        inequality(inequalityRow, wrench + 2) = 1.0;
        lower[inequalityRow] = 0.0;
        upper[inequalityRow] = maximumNormal;
        ++inequalityRow;
        const auto coneFace = [&](Eigen::Index componentIndex,
                double sign, double coefficient) {
            inequality(inequalityRow, wrench + componentIndex) = sign;
            inequality(inequalityRow, wrench + 2) = -coefficient;
            upper[inequalityRow] = 0.0;
            ++inequalityRow;
        };
        coneFace(0, 1.0, friction);
        coneFace(0, -1.0, friction);
        coneFace(1, 1.0, friction);
        coneFace(1, -1.0, friction);
        coneFace(3, 1.0, halfWidth);
        coneFace(3, -1.0, halfWidth);
        coneFace(4, 1.0, halfLength);
        coneFace(4, -1.0, halfLength);
        coneFace(5, 1.0, torsionalFriction);
        coneFace(5, -1.0, torsionalFriction);
    }
    if (locomotionActive) {
        // A folga horizontal cobre discrepância de modelo/limite articular,
        // mas não pode inverter a reação de captura. Um único bound bilateral
        // por eixo preserva factibilidade e a hierarquia centroidal sem os
        // problemas numéricos de um peso quadrático gigantesco.
        constexpr double MaximumHorizontalCentroidalSlack = 0.45;
        for (Eigen::Index axis = 0; axis < 2; ++axis) {
            inequality(inequalityRow,
                centroidalSlackOffset + axis) = 1.0;
            lower[inequalityRow] = -MaximumHorizontalCentroidalSlack;
            upper[inequalityRow] = MaximumHorizontalCentroidalSlack;
            ++inequalityRow;
        }
    }
    if (inequalityRow != inequalityCount) return false;

    if (m_impl->qp == nullptr || m_impl->variables != variableCount
        || m_impl->equalities != equalityCount
        || m_impl->inequalities != inequalityCount) {
        m_impl->qp =
            std::make_unique<proxsuite::proxqp::dense::QP<double>>(
                variableCount, equalityCount, inequalityCount);
        m_impl->variables = variableCount;
        m_impl->equalities = equalityCount;
        m_impl->inequalities = inequalityCount;
        m_impl->qp->settings.eps_abs = 1.0e-5;
        m_impl->qp->settings.eps_rel = 1.0e-5;
        // Trocas de apoio alteram contatos, tarefas e limites ativos no
        // mesmo frame. O warm-start normalmente converge cedo; este teto
        // maior cobre o heel-strike sem penalizar os frames simples.
        m_impl->qp->settings.max_iter = 160;
        m_impl->qp->init(hessian, gradient, equality, equalityTarget,
            inequality, lower, upper);
    } else {
        m_impl->qp->settings.initial_guess =
            proxsuite::proxqp::InitialGuessStatus::
                WARM_START_WITH_PREVIOUS_RESULT;
        m_impl->qp->update(hessian, gradient, equality, equalityTarget,
            inequality, lower, upper, true);
    }
    m_impl->qp->solve();
    const auto& solverInfo = m_impl->qp->results.info;
    m_telemetry.solverOptimal =
        solverInfo.status
        == proxsuite::proxqp::QPSolverOutput::PROXQP_SOLVED;
    m_telemetry.solverIterations =
        static_cast<std::uint32_t>(
            std::max<std::int64_t>(0, solverInfo.iter));
    m_telemetry.solverPrimalResidual =
        static_cast<float>(solverInfo.pri_res);
    m_telemetry.solverDualResidual =
        static_cast<float>(solverInfo.dua_res);
    const Vector& solution = m_impl->qp->results.x;
    if (!m_telemetry.solverOptimal
        || solution.size() != variableCount || !solution.allFinite()) {
        return false;
    }

    for (RagdollDriveTarget3D& target : targets) {
        if (target.linkIndex >= dynamics.jointGeneralizedDof.size()) continue;
        const std::size_t axis =
            static_cast<std::size_t>(target.axis);
        const std::uint32_t generalizedIndex =
            dynamics.jointGeneralizedDof[target.linkIndex][axis];
        if (generalizedIndex == RagdollDynamics3D::InvalidIndex) continue;
        const Eigen::Index dof =
            static_cast<Eigen::Index>(generalizedIndex - 6u);
        target.feedforwardTorqueNewtonMeters =
            static_cast<float>(solution[torqueOffset + dof]);
        // O QP já contém postura, amortecimento, COM e tarefa cartesiana do
        // pé. Somar aqui um drive de pose do PhysX cria um segundo controlador
        // que não aparece em M*qdd+h=S^T*tau+J^T*lambda. A aceleração prevista
        // pelo WBC então deixa de corresponder ao frame físico seguinte — em
        // transferência de peso chegamos a medir aceleração com sinal oposto.
        // Com a solução ótima, a atuação é exclusivamente o torque
        // generalizado calculado pelo QP. O reflexo interno ja entrou como
        // referencia limitada dentro do proprio problema. Os drives
        // permanecem configurados com autoridade/limites válidos, mas ganhos
        // nulos.
        // O torque do QP é o feed-forward do corpo inteiro. Uma pequena
        // impedância local na cadeia axial corrige erro de discretização,
        // contato e modelo entre ticks, como servo de baixo nível — não é
        // força na raiz nem sustentação externa. Nos demais membros o QP
        // continua sendo a única atuação durante esta etapa.
        const float requestedStiffness = target.stiffnessScale;
        const float requestedDamping = target.dampingScale;
        const std::string& linkId = profile.links[target.linkIndex].id;
        const bool spine = linkId == "Abdomen"
            || linkId == "Chest" || linkId == "UpperChest";
        const bool arm = linkId.find("Arm") != std::string::npos
            || linkId.find("Forearm") != std::string::npos
            || linkId.find("Hand") != std::string::npos;
        const bool headChain = linkId == "Neck" || linkId == "Head";
        const bool leg = linkId.find("Thigh") != std::string::npos
            || linkId.find("Shin") != std::string::npos
            || linkId.find("Foot") != std::string::npos;
        const bool leftLeg = linkId.rfind("Left", 0) == 0;
        const bool swingLeg = leg
            && ((leftLeg && intent.trackSwingFoot[0])
                || (!leftLeg && intent.trackSwingFoot[1]));
        target.stiffnessScale = spine
            ? (locomotionActive ? 1.58f : 0.58f)
            : arm
                ? std::min(requestedStiffness,
                    intent.fallProtectionActive ? 0.52f : 0.28f)
                : headChain ? std::min(requestedStiffness,
                    locomotionActive ? 1.38f : 0.82f)
                : leg ? std::min(requestedStiffness,
                    // A perna em voo usa inverse dynamics como feed-forward
                    // e a IK do footwork como impedância local. Sem este
                    // servo de baixo nível, erros de contato/modelo deixavam
                    // o pé 15--20 cm acima do touchdown apesar do QP.
                    swingLeg ? 0.92f : 0.30f) : 0.0f;
        target.dampingScale = spine
            ? (locomotionActive ? 2.00f : 1.08f)
            : arm
                ? std::min(requestedDamping,
                    intent.fallProtectionActive ? 0.95f : 0.66f)
                : headChain ? std::min(requestedDamping, 1.48f)
                : leg ? std::min(requestedDamping,
                    swingLeg ? 1.18f : 0.92f) : 0.0f;
        target.maximumTorqueScale = spine
            ? std::max(m_config.torqueLimitScale, 2.20f)
            : headChain
                ? std::max(m_config.torqueLimitScale, 1.90f)
                : m_config.torqueLimitScale;
    }

    const Vector residual =
        equality * solution - equalityTarget;
    m_telemetry.maximumTorqueNewtonMeters =
        maximumAbs(solution, torqueOffset, jointDofs);
    m_telemetry.solvedRootLinearAcceleration = {
        static_cast<float>(solution[0]),
        static_cast<float>(solution[1]),
        static_cast<float>(solution[2])
    };
    const Vector solvedMomentumRate =
        centroidal * solution.head(generalized)
        + Eigen::Map<const Eigen::Matrix<float, 6, 1>>(
            dynamics.centroidalMomentumBias.data()).cast<double>();
    m_telemetry.solvedCenterOfMassAcceleration = {
        static_cast<float>(solvedMomentumRate[0] / totalMass),
        static_cast<float>(solvedMomentumRate[1] / totalMass),
        static_cast<float>(solvedMomentumRate[2] / totalMass)
    };
    if (m_telemetry.swingFootLink != RagdollDynamics3D::InvalidIndex
        && m_telemetry.swingFootLink < dynamics.linkJacobianRow.size()) {
        const std::uint32_t firstRow =
            dynamics.linkJacobianRow[m_telemetry.swingFootLink];
        if (firstRow != RagdollDynamics3D::InvalidIndex
            && firstRow + 3u <= dynamics.jacobianRowCount) {
            for (std::size_t axis = 0; axis < 3; ++axis) {
                double acceleration = 0.0;
                for (Eigen::Index column = 0;
                        column < generalized; ++column) {
                    acceleration += dynamics.denseJacobian[
                        static_cast<std::size_t>(firstRow + axis)
                            * dynamics.jacobianColumnCount
                        + static_cast<std::size_t>(column)]
                        * solution[column];
                }
                if (axis == 0) {
                    m_telemetry.solvedSwingFootAcceleration.x =
                        static_cast<float>(acceleration);
                } else if (axis == 1) {
                    m_telemetry.solvedSwingFootAcceleration.y =
                        static_cast<float>(acceleration);
                } else {
                    m_telemetry.solvedSwingFootAcceleration.z =
                        static_cast<float>(acceleration);
                }
            }
        }
    }
    m_telemetry.dynamicsResidual =
        static_cast<float>(residual.head(generalized).norm());
    for (std::size_t side = 0; side < 2; ++side) {
        if (contactSlot[side] < 0) continue;
        const Eigen::Index wrench =
            contactOffset + contactSlot[side] * 6;
        const Vec3 force {
            static_cast<float>(solution[wrench]),
            static_cast<float>(solution[wrench + 1]),
            static_cast<float>(solution[wrench + 2])
        };
        const float normal = static_cast<float>(
            solution[wrench + 2]);
        if (side == 0) {
            m_telemetry.leftNormalForceNewtons = normal;
            m_telemetry.leftContactForceNewtons = force;
        } else {
            m_telemetry.rightNormalForceNewtons = normal;
            m_telemetry.rightContactForceNewtons = force;
        }
    }
    m_telemetry.solved = true;
    static_cast<void>(deltaTime);
    return true;
}

} // namespace MatterEngine
