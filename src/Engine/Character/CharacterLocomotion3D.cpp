#include "Engine/Character/CharacterLocomotion3D.hpp"
#include "Engine/Character/CharacterFootGeometry3D.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

namespace MatterEngine {
namespace {

constexpr float Pi = 3.14159265358979323846f;
constexpr float TwoPi = Pi * 2.0f;

// Olhar x movimento: setores de direcao em relacao a camera. A direcao
// decide o ciclo, nunca a ordem das teclas. O teclado da oito direcoes a
// 45 graus uma da outra; as fronteiras ficam no meio entre elas:
//   frente e diagonais da frente (0, +-45)  -> corrida, pelve girada
//   lados (+-90)                            -> strafe
//   costas e diagonais de tras (+-135, 180) -> recuo, pelve girada
// Fronteiras em 67,5 e 112,5 graus. Com as fronteiras em 45 e 135 (setores
// iguais de 90), cada diagonal do teclado caia em cima de uma delas e quem
// decidia era a histerese - W e depois A dava corrida; A e depois W, strafe.
// A histerese (a folga para trocar de setor depois de passar a fronteira)
// fica so para o analogico perto da fronteira; nenhuma direcao do teclado
// chega nela. Sem strafe, dois setores (frente, costas), fronteira em 90.
constexpr float StrafeFrontLimitRadians = 67.5f * Pi / 180.0f;
constexpr float StrafeBackLimitRadians = 112.5f * Pi / 180.0f;
constexpr float StrafeSectorHysteresisRadians = 10.0f * Pi / 180.0f;
constexpr float PlainSectorLimitRadians = 90.0f * Pi / 180.0f;
constexpr float PlainSectorHysteresisRadians = 15.0f * Pi / 180.0f;

float gaitSectorCenter(CharacterGaitDirection3D direction) {
    switch (direction) {
    case CharacterGaitDirection3D::Left: return Pi * 0.5f;
    case CharacterGaitDirection3D::Right: return -Pi * 0.5f;
    case CharacterGaitDirection3D::Backward: return Pi;
    default: return 0.0f;
    }
}

// Quanto a direcao (em relacao ao olhar) passa do setor; zero dentro dele.
float gaitSectorOverrun(CharacterGaitDirection3D direction,
    float travelFromView, bool strafe) {
    const float angle = std::abs(travelFromView);
    const float front = strafe ? StrafeFrontLimitRadians
        : PlainSectorLimitRadians;
    const float back = strafe ? StrafeBackLimitRadians
        : PlainSectorLimitRadians;
    switch (direction) {
    case CharacterGaitDirection3D::Forward:
        return std::max(angle - front, 0.0f);
    case CharacterGaitDirection3D::Backward:
        return std::max(back - angle, 0.0f);
    case CharacterGaitDirection3D::Left:
    case CharacterGaitDirection3D::Right: {
        // O lado certo, entre as duas fronteiras; o outro lado esta longe.
        const bool leftSide = travelFromView >= 0.0f;
        const bool wanted = direction == CharacterGaitDirection3D::Left;
        if (leftSide != wanted) return Pi;
        return std::max({front - angle, angle - back, 0.0f});
    }
    }
    return Pi;
}
// Quanto do olhar a coluna devolve e quanto a cabeca completa, em relacao a
// pelve. Coluna: 45 graus dos 67 que Abdomen, Chest e UpperChest somam - o
// resto fica de folga para a rotacao do proprio ciclo. Cabeca: 75, o limite
// de pescoco (30) mais cabeca (45). Pelve a ate 105 graus da camera (a borda
// do recuo) ainda deixa a cabeca olhando para ela.
constexpr float TorsoTwistLimitRadians = 45.0f * Pi / 180.0f;
// Correndo, o tronco fica com as pernas e quem olha e a cabeca: com a coluna
// girada 45 graus, o braco balanca num plano torto em relacao a passada e
// bate na coxa que sobe (medido: antebraco 61 graus fora do alvo num sprint
// em diagonal). Com 20, cabeca e coluna ainda cobrem 95 graus.
constexpr float SprintTorsoTwistLimitRadians = 20.0f * Pi / 180.0f;
constexpr float HeadTurnLimitRadians = 75.0f * Pi / 180.0f;
constexpr float TorsoLookShare = 0.8f;
constexpr float DefaultTransitionSeconds = 0.14f;
constexpr float GaitFlipTransitionSeconds = 0.35f;

// Fall protection keeps some tone, then yields while the body settles.
// Recovery ramps from this relaxed state over the existing 0.5 s envelope.
// Modo fisico, de pe: tonus das juntas (fora as pernas, que tem o proprio
// ajuste) em relacao a forca nominal do perfil. Era 2,0.
constexpr float UprightMuscleTone = 1.3f;
// Modo fisico: altura minima do pe no meio de um passo (m, acima do chao).
constexpr float SwingMinimumClearance = 0.13f;
constexpr float SettleRestingTone = 0.25f;
// Caido e parado, quanto tempo fica largado antes de se arrumar para levantar.
constexpr float SettleRestBeforeGather = 1.20f;
// Numa nova tentativa de levantar (a anterior falhou), quanto espera.
constexpr float RetryRestBeforeGather = 0.90f;
// Caindo: tonus dos bracos/pescoco (reflexo) e do resto (queda sem volta).
// O corpo (era 0,38) cai solto; os bracos protegem - com 0,65 a mao que ia
// ao chao cedia no tombo e a cabeca batia antes (queda de lado).
constexpr float FallReflexLimbTone = 0.85f;
constexpr float FallBodyTone = 0.25f;
// Largado (caindo sem volta, deitado): quanto da compensacao de gravidade
// resta - os membros pesam. Bracos protegendo a queda seguram mais.
constexpr float FallBodyGravity = 0.25f;
constexpr float FallLimbGravity = 0.80f;
constexpr float LyingGravity = 0.10f;
// Tonus ao bater no chao (cai dele ate SettleRestingTone em ~0,1 s).
constexpr float SettleImpactTone = 0.50f;
constexpr float RisingArmStiffness = 2.0f;
constexpr float RisingTorsoStiffness = 1.3f;
constexpr float RisingLegStiffness = 1.6f;
constexpr float RisingArmTorque = 2.0f;
constexpr float RisingTorsoTorque = 1.5f;
constexpr float RisingLegTorque = 1.8f;

bool finite(Vec3 value) {
    return std::isfinite(value.x) && std::isfinite(value.y)
        && std::isfinite(value.z);
}

float component(Vec3 value, std::size_t axis) {
    return axis == 0 ? value.x : axis == 1 ? value.y : value.z;
}

void setComponent(Vec3& value, std::size_t axis, float componentValue) {
    if (axis == 0) value.x = componentValue;
    else if (axis == 1) value.y = componentValue;
    else value.z = componentValue;
}

float smoothStep(float value) {
    value = std::clamp(value, 0.0f, 1.0f);
    return value * value * (3.0f - 2.0f * value);
}

float wrapAngle(float radians) {
    while (radians > Pi) radians -= TwoPi;
    while (radians < -Pi) radians += TwoPi;
    return radians;
}

std::size_t findLink(const RagdollProfile3D& profile, std::string_view id) {
    for (std::size_t index = 0; index < profile.links.size(); ++index) {
        if (profile.links[index].id == id) return index;
    }
    return profile.links.size();
}


Vec3 uprightOrientationError(Quaternion current, float desiredYaw,
    CharacterFallOrientation3D fallOrientation) {
    const Vec3 currentForward = current.rotate({ 1.0f, 0.0f, 0.0f });
    const Vec3 currentLeft = current.rotate({ 0.0f, 1.0f, 0.0f });
    const Vec3 currentUp = current.rotate({ 0.0f, 0.0f, 1.0f });
    const Vec3 desiredForward {
        std::cos(desiredYaw), std::sin(desiredYaw), 0.0f
    };
    const Vec3 desiredLeft {
        -std::sin(desiredYaw), std::cos(desiredYaw), 0.0f
    };
    Vec3 error = (cross(currentForward, desiredForward)
        + cross(currentLeft, desiredLeft)
        + cross(currentUp, Vec3 { 0.0f, 0.0f, 1.0f })) * 0.5f;
    if (error.lengthSquared() < 0.0025f && currentUp.z < -0.65f) {
        error = desiredLeft * (fallOrientation
                == CharacterFallOrientation3D::FaceDown ? -1.0f : 1.0f);
    }
    return error;
}

Quaternion exponentialRotation(Vec3 coordinates) {
    const float angle = coordinates.length();
    return angle > 0.000001f
        ? Quaternion::fromAxisAngle(coordinates / angle, angle)
        : Quaternion {};
}

Vec3 exponentialJacobian(Vec3 coordinates, Vec3 velocity) {
    const float squared = coordinates.lengthSquared();
    const float angle = std::sqrt(squared);
    const float first = squared < 0.0001f
        ? 0.5f - squared / 24.0f
        : (1.0f - std::cos(angle)) / squared;
    const float second = squared < 0.0001f
        ? 1.0f / 6.0f - squared / 120.0f
        : (angle - std::sin(angle)) / (squared * angle);
    return velocity + cross(coordinates, velocity) * first
        + cross(coordinates, cross(coordinates, velocity)) * second;
}

Vec3 jointAngularVelocity(Vec3 target, Vec3 coordinateVelocity,
    Vec3 measured) {
    if (!finite(target) || !finite(coordinateVelocity) || !finite(measured)) {
        return {};
    }
    return exponentialRotation(measured).conjugate().rotate(
        exponentialJacobian(target, coordinateVelocity));
}

void buildLocalPose(const RagdollProfile3D& profile,
    const std::vector<Vec3>& coordinates, RagdollAnimationPose3D& pose) {
    pose.linkPositions.assign(profile.links.size(), {});
    pose.linkOrientations.assign(profile.links.size(), {});
    for (std::size_t index = 1; index < profile.links.size(); ++index) {
        const RagdollLinkDefinition3D& link = profile.links[index];
        if (link.parentIndex < 0) continue;
        const std::size_t parent = static_cast<std::size_t>(link.parentIndex);
        const RagdollLinkDefinition3D& parentLink = profile.links[parent];
        Vec3 angles = index < coordinates.size()
            ? coordinates[index] : Vec3 {};
        for (std::size_t axis = 0; axis < 3; ++axis) {
            const RagdollAxisDefinition3D& limit =
                link.inboundJoint.axes[axis];
            setComponent(angles, axis, limit.enabled
                ? std::clamp(component(angles, axis),
                    limit.minimumRadians, limit.maximumRadians)
                : 0.0f);
        }
        const Quaternion frame = link.inboundJoint.frameModelOrientation;
        pose.linkOrientations[index] =
            (pose.linkOrientations[parent]
                * parentLink.modelOrientation.conjugate()
                * frame * exponentialRotation(angles)
                * (link.modelOrientation.conjugate() * frame).conjugate())
                    .normalized();
        const Vec3 anchor = link.inboundJoint.anchorModelPosition;
        pose.linkPositions[index] = pose.linkPositions[parent]
            + pose.linkOrientations[parent].rotate(
                parentLink.modelOrientation.conjugate().rotate(
                    anchor - parentLink.modelPosition))
            - pose.linkOrientations[index].rotate(
                link.modelOrientation.conjugate().rotate(
                    anchor - link.modelPosition));
    }
}

// Position-only damped least squares: brings a point fixed to endIndex
// (pointInEnd, in that link's frame) to a target, moving the joints of
// endIndex and its ancestors up to `depth` links, within their limits.
void solveChainPoint(const RagdollProfile3D& profile,
    std::vector<Vec3>& coordinates, std::size_t footIndex, Vec3 pointInEnd,
    int depth, Vec3 targetRootLocal, float weight) {
    if (footIndex == 0 || footIndex >= profile.links.size()
        || coordinates.size() != profile.links.size()
        || !finite(targetRootLocal) || weight <= 0.0f) {
        return;
    }
    struct DegreeOfFreedom {
        std::size_t link = 0;
        std::size_t axis = 0;
    };
    std::array<DegreeOfFreedom, 9> degrees;
    std::size_t count = 0;
    int linkIndex = static_cast<int>(footIndex);
    for (int level = 0; level < depth && linkIndex > 0; ++level) {
        const auto& joint = profile.links[static_cast<std::size_t>(linkIndex)]
            .inboundJoint;
        for (std::size_t axis = 0; axis < 3; ++axis) {
            if (joint.axes[axis].enabled && count < degrees.size()) {
                degrees[count++] = {
                    static_cast<std::size_t>(linkIndex), axis
                };
            }
        }
        linkIndex = profile.links[static_cast<std::size_t>(linkIndex)]
            .parentIndex;
    }
    if (count == 0) return;

    const std::vector<Vec3> original = coordinates;
    RagdollAnimationPose3D pose;
    for (int iteration = 0; iteration < 7; ++iteration) {
        buildLocalPose(profile, coordinates, pose);
        const Vec3 endPoint = pose.linkPositions[footIndex]
            + pose.linkOrientations[footIndex].rotate(pointInEnd);
        const Vec3 error = targetRootLocal - endPoint;
        if (error.lengthSquared() < 0.00000025f) break;

        float jacobian[3][9] {};
        for (std::size_t column = 0; column < count; ++column) {
            const DegreeOfFreedom degree = degrees[column];
            const RagdollLinkDefinition3D& link = profile.links[degree.link];
            const std::size_t parent = static_cast<std::size_t>(
                link.parentIndex);
            const RagdollLinkDefinition3D& parentLink = profile.links[parent];
            const Quaternion parentModelFrame = pose.linkOrientations[parent]
                * parentLink.modelOrientation.conjugate();
            const Vec3 basis = degree.axis == 0 ? Vec3 { 1.0f, 0.0f, 0.0f }
                : degree.axis == 1 ? Vec3 { 0.0f, 1.0f, 0.0f }
                : Vec3 { 0.0f, 0.0f, 1.0f };
            const Vec3 angular = (parentModelFrame
                * link.inboundJoint.frameModelOrientation).rotate(
                    exponentialJacobian(coordinates[degree.link], basis));
            const Vec3 anchor = pose.linkPositions[parent]
                + parentModelFrame.rotate(
                    link.inboundJoint.anchorModelPosition
                        - parentLink.modelPosition);
            const Vec3 linear = cross(angular, endPoint - anchor);
            jacobian[0][column] = linear.x;
            jacobian[1][column] = linear.y;
            jacobian[2][column] = linear.z;
        }

        float system[9][10] {};
        for (std::size_t row = 0; row < count; ++row) {
            for (std::size_t column = 0; column < count; ++column) {
                for (std::size_t dimension = 0; dimension < 3; ++dimension) {
                    system[row][column] += jacobian[dimension][row]
                        * jacobian[dimension][column];
                }
                if (row == column) system[row][column] += 0.025f;
            }
            system[row][count] = jacobian[0][row] * error.x
                + jacobian[1][row] * error.y
                + jacobian[2][row] * error.z;
        }
        for (std::size_t pivot = 0; pivot < count; ++pivot) {
            std::size_t best = pivot;
            for (std::size_t row = pivot + 1; row < count; ++row) {
                if (std::abs(system[row][pivot])
                    > std::abs(system[best][pivot])) best = row;
            }
            if (best != pivot) {
                for (std::size_t column = pivot; column <= count; ++column) {
                    std::swap(system[pivot][column], system[best][column]);
                }
            }
            const float divisor = system[pivot][pivot];
            if (std::abs(divisor) < 0.000001f) continue;
            for (std::size_t column = pivot; column <= count; ++column) {
                system[pivot][column] /= divisor;
            }
            for (std::size_t row = 0; row < count; ++row) {
                if (row == pivot) continue;
                const float factor = system[row][pivot];
                for (std::size_t column = pivot; column <= count; ++column) {
                    system[row][column] -= factor * system[pivot][column];
                }
            }
        }
        for (std::size_t degreeIndex = 0;
                degreeIndex < count; ++degreeIndex) {
            const DegreeOfFreedom degree = degrees[degreeIndex];
            const RagdollAxisDefinition3D& limit = profile.links[degree.link]
                .inboundJoint.axes[degree.axis];
            const float next = component(coordinates[degree.link], degree.axis)
                + std::clamp(system[degreeIndex][count], -0.13f, 0.13f);
            setComponent(coordinates[degree.link], degree.axis,
                std::clamp(next, limit.minimumRadians, limit.maximumRadians));
        }
    }

    weight = std::clamp(weight, 0.0f, 1.0f);
    for (std::size_t degreeIndex = 0; degreeIndex < count; ++degreeIndex) {
        const DegreeOfFreedom degree = degrees[degreeIndex];
        const float result = component(coordinates[degree.link], degree.axis);
        const float before = component(original[degree.link], degree.axis);
        setComponent(coordinates[degree.link], degree.axis,
            before + (result - before) * weight);
    }
}

// Keeps the foot link on a support point (stance lock, turn steps).
void solveFootPosition(const RagdollProfile3D& profile,
    std::vector<Vec3>& coordinates, std::size_t footIndex,
    Vec3 targetRootLocal, float weight) {
    solveChainPoint(profile, coordinates, footIndex, {}, 3, targetRootLocal,
        weight);
}

Quaternion rotationBetween(Vec3 from, Vec3 to) {
    from = from.normalized();
    to = to.normalized();
    const float cosine = std::clamp(dot(from, to), -1.0f, 1.0f);
    const Vec3 axis = cross(from, to);
    const float sine = axis.length();
    if (sine < 0.000001f) {
        if (cosine > 0.0f) return {};
        const Vec3 other = std::abs(from.x) < 0.9f
            ? Vec3 { 1.0f, 0.0f, 0.0f } : Vec3 { 0.0f, 1.0f, 0.0f };
        return Quaternion::fromAxisAngle(cross(from, other).normalized(), Pi);
    }
    return Quaternion::fromAxisAngle(axis / sine, std::atan2(sine, cosine));
}

Quaternion slerpQuaternion(Quaternion from, Quaternion to, float t) {
    t = std::clamp(t, 0.0f, 1.0f);
    float cosine = from.x * to.x + from.y * to.y + from.z * to.z
        + from.w * to.w;
    if (cosine < 0.0f) {
        to = { -to.x, -to.y, -to.z, -to.w };
        cosine = -cosine;
    }
    if (cosine > 0.9995f) {
        return Quaternion { from.x + (to.x - from.x) * t,
            from.y + (to.y - from.y) * t, from.z + (to.z - from.z) * t,
            from.w + (to.w - from.w) * t }.normalized();
    }
    const float angle = std::acos(std::clamp(cosine, -1.0f, 1.0f));
    const float sine = std::sin(angle);
    const float a = std::sin((1.0f - t) * angle) / sine;
    const float b = std::sin(t * angle) / sine;
    return Quaternion { from.x * a + to.x * b, from.y * a + to.y * b,
        from.z * a + to.z * b, from.w * a + to.w * b }.normalized();
}

// Pe inteiro: posicao e orientacao do link do pe, no espaco da raiz. A
// coxa e a canela levam o tornozelo (a ancora da junta do pe) onde o pe
// pedido precisa dele; o tornozelo, com os dois eixos que tem, orienta o pe,
// com prioridade para a sola (o vetor "para cima" do pe decide o contato
// com o chao) e a ponta em segundo.
Vec3 rotationVector(Quaternion value);

void solveFootPose(const RagdollProfile3D& profile,
    std::vector<Vec3>& coordinates, std::size_t footIndex,
    Vec3 targetPosition, Quaternion targetOrientation, float weight,
    bool holdYaw) {
    if (footIndex == 0 || footIndex >= profile.links.size()
        || coordinates.size() != profile.links.size() || weight <= 0.0f
        || !finite(targetPosition)) {
        return;
    }
    const RagdollLinkDefinition3D& foot = profile.links[footIndex];
    if (foot.parentIndex <= 0) return;
    const std::size_t shinIndex = static_cast<std::size_t>(foot.parentIndex);
    const RagdollLinkDefinition3D& shin = profile.links[shinIndex];
    const Vec3 anchor = foot.inboundJoint.anchorModelPosition;
    const Vec3 anchorInFoot = foot.modelOrientation.conjugate().rotate(
        anchor - foot.modelPosition);
    const Vec3 anchorInShin = shin.modelOrientation.conjugate().rotate(
        anchor - shin.modelPosition);
    const std::vector<Vec3> original = coordinates;
    const Vec3 ankleTarget = targetPosition
        + targetOrientation.rotate(anchorInFoot);
    solveChainPoint(profile, coordinates, shinIndex, anchorInShin, 2,
        ankleTarget, 1.0f);

    RagdollAnimationPose3D pose;
    // Rumo do pe pelo quadril. O tornozelo so tem flexao e inversao: o rumo
    // do pe em relacao a pelve vem da rotacao da coxa. Resolvendo so a
    // posicao do tornozelo, o pe apoiado girava junto com a pelve - no chao
    // (medido: pe "travado" rodando 32 graus no primeiro passo de um giro
    // parado, os mesmos da pelve). Gira a perna inteira em volta da linha
    // quadril-tornozelo (o tornozelo fica onde esta) ate o pe apontar para
    // onde deve, dentro dos limites do quadril.
    //
    // So parado (giro no lugar, acomodacao, idle). Andando, o pe segue o
    // rumo da pelve como sempre seguiu: com a abertura de 7 graus da pose
    // respeitada, o pe de 28 cm nao cabia mais no degrau de 28 cm e a
    // quina afundava (medido: descendo escada, 7 amostras a ate 11 cm).
    if (holdYaw && shin.parentIndex > 0) {
        const std::size_t thighIndex =
            static_cast<std::size_t>(shin.parentIndex);
        const RagdollLinkDefinition3D& thigh = profile.links[thighIndex];
        const std::size_t pelvisIndex =
            static_cast<std::size_t>(std::max(0, thigh.parentIndex));
        const RagdollLinkDefinition3D& pelvis = profile.links[pelvisIndex];
        buildLocalPose(profile, coordinates, pose);
        const Quaternion parentFrame = pose.linkOrientations[pelvisIndex]
            * pelvis.modelOrientation.conjugate()
            * thigh.inboundJoint.frameModelOrientation;
        const Quaternion childFrame = (thigh.modelOrientation.conjugate()
            * thigh.inboundJoint.frameModelOrientation).conjugate();
        const Vec3 hip = pose.linkPositions[pelvisIndex]
            + (pose.linkOrientations[pelvisIndex]
                * pelvis.modelOrientation.conjugate()).rotate(
                    thigh.inboundJoint.anchorModelPosition
                        - pelvis.modelPosition);
        const Vec3 ankle = pose.linkPositions[footIndex]
            + pose.linkOrientations[footIndex].rotate(anchorInFoot);
        Vec3 axis = ankle - hip;
        const Vec3 forwardLocal = foot.modelOrientation.conjugate().rotate(
            { 1.0f, 0.0f, 0.0f });
        if (axis.lengthSquared() > 0.01f) {
            axis = axis.normalized();
            Vec3 current = pose.linkOrientations[footIndex].rotate(
                forwardLocal);
            Vec3 wanted = targetOrientation.rotate(forwardLocal);
            current -= axis * dot(current, axis);
            wanted -= axis * dot(wanted, axis);
            if (current.lengthSquared() > 0.01f
                && wanted.lengthSquared() > 0.01f) {
                const float swivel = std::atan2(
                    dot(cross(current, wanted), axis), dot(current, wanted));
                const Quaternion thighWorld = pose.linkOrientations[thighIndex];
                const Quaternion turned = (Quaternion::fromAxisAngle(axis,
                    swivel) * thighWorld).normalized();
                Vec3 local = rotationVector(parentFrame.conjugate() * turned
                    * childFrame.conjugate());
                for (std::size_t index = 0; index < 3; ++index) {
                    const RagdollAxisDefinition3D& limit =
                        thigh.inboundJoint.axes[index];
                    setComponent(local, index, limit.enabled
                        ? std::clamp(component(local, index),
                            limit.minimumRadians, limit.maximumRadians)
                        : 0.0f);
                }
                coordinates[thighIndex] = local;
                // No limite do quadril o tornozelo sai do lugar: de volta.
                solveChainPoint(profile, coordinates, shinIndex, anchorInShin,
                    2, ankleTarget, 1.0f);
            }
        }
    }

    // Tornozelo: Gauss-Newton nos eixos habilitados do pe.
    buildLocalPose(profile, coordinates, pose);
    const Quaternion shinFrame = pose.linkOrientations[shinIndex]
        * shin.modelOrientation.conjugate() * foot.inboundJoint.frameModelOrientation;
    const Quaternion childFrame = (foot.modelOrientation.conjugate()
        * foot.inboundJoint.frameModelOrientation).conjugate();
    const Vec3 upLocal = foot.modelOrientation.conjugate().rotate(
        { 0.0f, 0.0f, 1.0f });
    const Vec3 forwardLocal = foot.modelOrientation.conjugate().rotate(
        { 1.0f, 0.0f, 0.0f });
    const Vec3 wantedUp = targetOrientation.rotate(upLocal);
    const Vec3 wantedForward = targetOrientation.rotate(forwardLocal);
    const auto residual = [&](Vec3 angles, float out[6]) {
        const Quaternion world = shinFrame * exponentialRotation(angles)
            * childFrame;
        const Vec3 up = world.rotate(upLocal) - wantedUp;
        const Vec3 forward = (world.rotate(forwardLocal) - wantedForward)
            * 0.25f;
        out[0] = up.x; out[1] = up.y; out[2] = up.z;
        out[3] = forward.x; out[4] = forward.y; out[5] = forward.z;
    };
    std::array<std::size_t, 3> axes {};
    std::size_t free = 0;
    for (std::size_t axis = 0; axis < 3; ++axis) {
        if (foot.inboundJoint.axes[axis].enabled) axes[free++] = axis;
    }
    Vec3 angles = coordinates[footIndex];
    for (int iteration = 0; iteration < 8 && free > 0; ++iteration) {
        float r[6];
        residual(angles, r);
        float jacobian[6][3] {};
        for (std::size_t column = 0; column < free; ++column) {
            Vec3 probe = angles;
            setComponent(probe, axes[column],
                component(probe, axes[column]) + 0.0005f);
            float shifted[6];
            residual(probe, shifted);
            for (int row = 0; row < 6; ++row) {
                jacobian[row][column] = (shifted[row] - r[row]) / 0.0005f;
            }
        }
        // (J^T J + lambda) step = -J^T r
        float normal[3][4] {};
        for (std::size_t a = 0; a < free; ++a) {
            for (std::size_t b = 0; b < free; ++b) {
                for (int row = 0; row < 6; ++row) {
                    normal[a][b] += jacobian[row][a] * jacobian[row][b];
                }
            }
            normal[a][a] += 0.001f;
            for (int row = 0; row < 6; ++row) {
                normal[a][free] -= jacobian[row][a] * r[row];
            }
        }
        float step[3] {};
        if (free == 1) {
            step[0] = normal[0][1] / normal[0][0];
        } else if (free == 2) {
            const float determinant = normal[0][0] * normal[1][1]
                - normal[0][1] * normal[1][0];
            if (std::abs(determinant) < 1e-9f) break;
            step[0] = (normal[0][2] * normal[1][1]
                - normal[0][1] * normal[1][2]) / determinant;
            step[1] = (normal[0][0] * normal[1][2]
                - normal[1][0] * normal[0][2]) / determinant;
        } else {
            break;
        }
        float size = 0.0f;
        for (std::size_t column = 0; column < free; ++column) {
            const RagdollAxisDefinition3D& limit =
                foot.inboundJoint.axes[axes[column]];
            const float next = std::clamp(component(angles, axes[column])
                    + std::clamp(step[column], -0.2f, 0.2f),
                limit.minimumRadians, limit.maximumRadians);
            size += std::abs(next - component(angles, axes[column]));
            setComponent(angles, axes[column], next);
        }
        if (size < 0.00005f) break;
    }
    coordinates[footIndex] = angles;

    weight = std::clamp(weight, 0.0f, 1.0f);
    if (weight < 1.0f) {
        for (int link : { static_cast<int>(footIndex),
                static_cast<int>(shinIndex), shin.parentIndex }) {
            if (link <= 0) continue;
            const auto index = static_cast<std::size_t>(link);
            coordinates[index] = original[index]
                + (coordinates[index] - original[index]) * weight;
        }
    }
}

bool validClip(const AnimationClip3D* clip,
    const RagdollProfile3D& profile) {
    return clip != nullptr && clip->targetRigId == profile.id
        && std::isfinite(clip->durationSeconds)
        && clip->durationSeconds > 0.0f
        && validateAnimationClipForRagdoll3D(*clip, profile).empty();
}

float authoredHorizontalSpeed(const AnimationClip3D& clip) {
    if (clip.nominalSpeedMetersPerSecond > 0.0f) {
        return clip.nominalSpeedMetersPerSecond;
    }
    const Vec3 displacement = clip.sourceRootDisplacementMeters;
    return clip.durationSeconds > 0.0f
        ? std::hypot(displacement.x, displacement.y) / clip.durationSeconds
        : 0.0f;
}

float stanceAlignedPhase(const AnimationClip3D& clip, float cyclePhase) {
    const auto& contacts = clip.leftFootContact.keyframes;
    if (contacts.empty() || clip.durationSeconds <= 0.0f) {
        return cyclePhase;
    }
    const auto strongest = std::max_element(contacts.begin(), contacts.end(),
        [](const AnimationScalarKeyframe3D& left,
            const AnimationScalarKeyframe3D& right) {
            return left.value < right.value;
        });
    const float offset = strongest->timeSeconds / clip.durationSeconds;
    return std::fmod(cyclePhase + offset, 1.0f);
}

constexpr float LocomotionAcceptanceStiffness = 0.75f;
// Pe com mola (input.footSpring): freio do tornozelo no ar, e no toque a
// mola (fracao da rigidez), o freio extra e quanto tempo dura.
constexpr float AnkleAirDamping = 0.6f;
constexpr float AnkleLandingStiffness = 0.4f;
constexpr float AnkleLandingDamping = 0.4f;
constexpr float AnkleLandingSeconds = 0.15f;
// Quanto o pe que vai pousar fica paralelo ao chao de pouso (o resto e
// a inclinacao do clipe).
constexpr float FootLandingFlatten = 0.7f;
// Tornozelo contra a queda: rad de pressao por metro de desvio do ponto de
// captura e os limites (ponta/calcanhar, borda).
constexpr float AnkleBalanceGain = 3.5f;
constexpr float AnkleBalancePitchLimit = 0.10f;
constexpr float AnkleBalanceRollLimit = 0.04f;
constexpr float AnkleBalanceDeadband = 0.012f;
// E5 (animacao + pes conscientes): fracao da diferenca de ponto de captura
// que o pouso corrige, e o limite (m).
constexpr float BalancePlacementGain = 0.6f;
constexpr float BalancePlacementLimitMeters = 0.15f;
// Janela do empurrao: a partir desta fracao do balanco do pe que vem.
constexpr float LocomotionPushWindow = 0.67f;

float footSoleOffset(const RagdollLinkDefinition3D& link) {
    return link.collider.shape == RagdollColliderShape3D::Box
        ? link.collider.boxHalfExtents.z
        : link.collider.radiusMeters;
}

// How far the collider's lowest point sits below its centre, for the
// orientation it is actually in. Treating a foot as a flat slab of
// halfExtents.z underestimates this the moment the ankle pitches - a 12
// degree pitch on a 30 cm foot puts the toe ~3 cm lower than that - and the
// grounding below then leaves the toe buried, which the floor answers with
// an enormous contact force.
float lowestColliderOffset(const RagdollLinkDefinition3D& link,
    Quaternion colliderWorldOrientation) {
    const Vec3 up = colliderWorldOrientation.conjugate().rotate(
        { 0.0f, 0.0f, 1.0f });
    if (link.collider.shape == RagdollColliderShape3D::Box) {
        const Vec3 half = link.collider.boxHalfExtents;
        return std::abs(up.x) * half.x + std::abs(up.y) * half.y
            + std::abs(up.z) * half.z;
    }
    // Capsule: the cylinder runs along local X, capped by the radius.
    return std::abs(up.x) * std::max(0.0f,
        link.collider.lengthMeters * 0.5f - link.collider.radiusMeters)
        + link.collider.radiusMeters;
}

Vec3 rotationVector(Quaternion value) {
    value = value.normalized();
    if (value.w < 0.0f) {
        value = { -value.x, -value.y, -value.z, -value.w };
    }
    const Vec3 imaginary { value.x, value.y, value.z };
    const float length = imaginary.length();
    if (length < 0.000001f) return {};
    const float angle = 2.0f * std::atan2(length, value.w);
    return imaginary * (angle / length);
}

Vec3 clampMagnitude(Vec3 value, float maximumLength) {
    const float squared = value.lengthSquared();
    if (squared <= maximumLength * maximumLength || squared < 0.000001f) {
        return value;
    }
    return value * (maximumLength / std::sqrt(squared));
}

void addJointCoordinate(const RagdollProfile3D& profile,
    std::vector<Vec3>& coordinates, std::string_view linkId,
    std::size_t axis, float amount) {
    for (std::size_t index = 1; index < profile.links.size(); ++index) {
        if (profile.links[index].id != linkId || index >= coordinates.size()) {
            continue;
        }
        const RagdollAxisDefinition3D& limit =
            profile.links[index].inboundJoint.axes[axis];
        if (!limit.enabled) return;
        setComponent(coordinates[index], axis,
            std::clamp(component(coordinates[index], axis) + amount,
                limit.minimumRadians, limit.maximumRadians));
        return;
    }
}

// Uma coordenada de junta levada para `wanted` por `amount` (0..1), dentro
// dos limites.
void blendJointCoordinate(const RagdollProfile3D& profile,
    std::vector<Vec3>& coordinates, std::size_t index, std::size_t axis,
    float wanted, float amount) {
    if (index == 0 || index >= profile.links.size()
        || index >= coordinates.size()) return;
    const RagdollAxisDefinition3D& limit =
        profile.links[index].inboundJoint.axes[axis];
    if (!limit.enabled) return;
    const float current = component(coordinates[index], axis);
    setComponent(coordinates[index], axis, std::clamp(
        current + (wanted - current) * amount,
        limit.minimumRadians, limit.maximumRadians));
}

// Reflexos de quem esta caindo, so alvos de junta (os motores fazem o
// resto; nada e escrito no corpo): as maos vao ao chao no rumo da queda, a
// cabeca se protege (queixo no peito caindo de costas, erguida caindo de
// frente, inclinada para fora de lado), a coluna enrola um pouco caindo de
// costas, joelhos e quadris cedem. `amount` mistura com a pose de antes.
void applyFallReflexPose(const RagdollProfile3D& profile,
    const RagdollState3D& state, std::vector<Vec3>& coordinates,
    std::array<std::size_t, 2> hands, std::size_t chest, Vec3 fallDirection,
    float groundHeight, float amount, float legAmount) {
    if (amount <= 0.001f || state.links.size() != profile.links.size()
        || chest >= state.links.size()) return;
    const auto indexOf = [&](std::string_view id) {
        for (std::size_t i = 1; i < profile.links.size(); ++i)
            if (profile.links[i].id == id) return i;
        return std::size_t { 0 };
    };
    const auto planarUnit = [](Vec3 v, Vec3 fallback) {
        v.z = 0.0f;
        return v.lengthSquared() > 0.0004f ? v.normalized() : fallback;
    };
    const Vec3 fall = planarUnit(fallDirection, { 1.0f, 0.0f, 0.0f });
    const Vec3 chestForward = planarUnit(
        state.links[chest].orientation.rotate({ 1.0f, 0.0f, 0.0f }), fall);
    const Vec3 chestLeft = planarUnit(
        state.links[chest].orientation.rotate({ 0.0f, 1.0f, 0.0f }),
        cross({ 0.0f, 0.0f, 1.0f }, chestForward));
    const float ahead = dot(fall, chestForward);
    const float aside = dot(fall, chestLeft);
    const float backward = std::clamp(-ahead, 0.0f, 1.0f);
    const float forward = std::clamp(ahead, 0.0f, 1.0f);
    // Cabeca e pescoco (swing1 + abaixa o queixo; swing2 + inclina para a
    // esquerda): caindo para a esquerda, a cabeca vai para a direita.
    const float nod = 0.95f * backward - 0.55f * forward + 0.20f * std::abs(aside);
    const float tilt = -0.75f * aside;
    blendJointCoordinate(profile, coordinates, indexOf("Neck"), 1, nod * 0.4f, amount);
    blendJointCoordinate(profile, coordinates, indexOf("Head"), 1, nod * 0.6f, amount);
    blendJointCoordinate(profile, coordinates, indexOf("Neck"), 2, tilt * 0.4f, amount);
    blendJointCoordinate(profile, coordinates, indexOf("Head"), 2, tilt * 0.6f, amount);
    // Coluna: enrola caindo de costas, inclina para fora de lado.
    for (const char* id : { "Abdomen", "Chest", "UpperChest" }) {
        blendJointCoordinate(profile, coordinates, indexOf(id), 1, 0.14f * backward, amount);
        blendJointCoordinate(profile, coordinates, indexOf(id), 2, -0.10f * aside, amount);
    }
    // Pernas cedendo (quadril: swing1 - flexiona; joelho: twist + flexiona),
    // so com a queda sem volta: antes elas ainda estao dando passos.
    for (const char* id : { "LeftThigh", "RightThigh" })
        blendJointCoordinate(profile, coordinates, indexOf(id), 1, -0.35f, legAmount * 0.7f);
    for (const char* id : { "LeftShin", "RightShin" })
        blendJointCoordinate(profile, coordinates, indexOf(id), 0, 0.55f, legAmount * 0.7f);
    // Bracos: a mao vai para o chao no rumo da queda, cada uma do seu lado,
    // ate onde o braco alcanca.
    const auto& root = state.links.front();
    const Quaternion rootInverse = root.orientation.conjugate();
    for (std::size_t side = 0; side < 2; ++side) {
        const std::size_t hand = hands[side];
        if (hand == 0 || hand >= profile.links.size()) continue;
        const int forearm = profile.links[hand].parentIndex;
        const int upper = forearm > 0
            ? profile.links[static_cast<std::size_t>(forearm)].parentIndex : -1;
        if (upper <= 0) continue;
        const Vec3 shoulder = state.links[static_cast<std::size_t>(upper)].position;
        const float s = side == 0 ? 1.0f : -1.0f;
        Vec3 target = shoulder + fall * 0.32f + chestLeft * (s * 0.16f);
        target.z = groundHeight + 0.06f;
        Vec3 reach = target - shoulder;
        constexpr float ArmReach = 0.52f;
        if (reach.length() > ArmReach) target = shoulder + reach * (ArmReach / reach.length());
        solveChainPoint(profile, coordinates, hand, {}, 2,
            rootInverse.rotate(target - root.position), amount);
        // Cotovelo nunca travado reto: cede no impacto.
        const float elbow = component(coordinates[static_cast<std::size_t>(forearm)], 0);
        if (elbow < 0.30f)
            blendJointCoordinate(profile, coordinates, static_cast<std::size_t>(forearm), 0, 0.30f, amount);
    }
}

void setJointCoordinate(const RagdollProfile3D& profile,
    std::vector<Vec3>& coordinates, std::string_view linkId,
    std::size_t axis, float value) {
    for (std::size_t index = 1; index < profile.links.size(); ++index) {
        if (profile.links[index].id != linkId || index >= coordinates.size()) {
            continue;
        }
        const RagdollAxisDefinition3D& limit =
            profile.links[index].inboundJoint.axes[axis];
        if (!limit.enabled) return;
        setComponent(coordinates[index], axis,
            std::clamp(value, limit.minimumRadians, limit.maximumRadians));
        return;
    }
}

// Perdendo o equilibrio (antes de cair): quem tenta nao cair abre os bracos
// e gira em moinho contra o sentido da queda, dobra o tronco para o lado
// oposto, cede os joelhos e acompanha com a cabeca. So alvos de junta - os
// motores fazem, a fisica decide se adianta. (Como nos personagens do
// Euphoria: ninguem fica duro esperando cair.)
void applyBalanceReaction(const RagdollProfile3D& profile,
    const RagdollState3D& state, std::vector<Vec3>& coordinates,
    std::array<std::size_t, 2> hands, std::size_t chest, Vec3 fallDirection,
    float amount, float time, std::array<bool, 2> braced) {
    if (amount <= 0.001f || state.links.size() != profile.links.size()
        || chest >= state.links.size()) return;
    const auto planarUnit = [](Vec3 v, Vec3 fallback) {
        v.z = 0.0f;
        return v.lengthSquared() > 0.0004f ? v.normalized() : fallback;
    };
    const Vec3 fall = planarUnit(fallDirection, { 1.0f, 0.0f, 0.0f });
    const Vec3 chestForward = planarUnit(
        state.links[chest].orientation.rotate({ 1.0f, 0.0f, 0.0f }), fall);
    const Vec3 chestLeft = planarUnit(
        state.links[chest].orientation.rotate({ 0.0f, 1.0f, 0.0f }),
        cross({ 0.0f, 0.0f, 1.0f }, chestForward));
    const float ahead = dot(fall, chestForward);
    const float aside = dot(fall, chestLeft);
    // Tronco contra a queda (flexao + para a frente; lateral + para a
    // esquerda): caindo para a frente, estende; para tras, dobra.
    for (const char* id : { "Abdomen", "Chest", "UpperChest" }) {
        addJointCoordinate(profile, coordinates, id, 1, -0.13f * ahead * amount);
        addJointCoordinate(profile, coordinates, id, 2, -0.08f * aside * amount);
    }
    // Cabeca olha para onde vai e baixa um pouco (procura o chao).
    addJointCoordinate(profile, coordinates, "Neck", 0, 0.20f * aside * amount);
    addJointCoordinate(profile, coordinates, "Head", 0, 0.25f * aside * amount);
    addJointCoordinate(profile, coordinates, "Head", 1, 0.12f * amount);
    // Joelhos e quadril cedem: baixa o centro de massa.
    for (const char* id : { "LeftThigh", "RightThigh" })
        addJointCoordinate(profile, coordinates, id, 1, -0.16f * amount);
    for (const char* id : { "LeftShin", "RightShin" })
        addJointCoordinate(profile, coordinates, id, 0, 0.28f * amount);
    // Bracos: abertos, erguidos e contra a queda, girando em moinho (um fora
    // de fase do outro).
    const auto& root = state.links.front();
    const Quaternion rootInverse = root.orientation.conjugate();
    const Vec3 up { 0.0f, 0.0f, 1.0f };
    for (std::size_t side = 0; side < 2; ++side) {
        const std::size_t hand = hands[side];
        if (hand == 0 || hand >= profile.links.size()) continue;
        const int forearm = profile.links[hand].parentIndex;
        const int upper = forearm > 0
            ? profile.links[static_cast<std::size_t>(forearm)].parentIndex : -1;
        if (upper <= 0) continue;
        // Mao apoiada num obstaculo fica la e empurra (cotovelo estendendo):
        // e com ela que ele se afasta e se endireita.
        if (braced[side]) {
            blendJointCoordinate(profile, coordinates,
                static_cast<std::size_t>(forearm), 0, 0.15f, 0.8f * amount);
            continue;
        }
        const Vec3 shoulder = state.links[static_cast<std::size_t>(upper)].position;
        const float s = side == 0 ? 1.0f : -1.0f;
        const float phase = 2.0f * Pi * 1.7f * time + (side == 0 ? 0.0f : Pi);
        const float radius = 0.15f * amount;
        Vec3 target = shoulder + chestLeft * (s * 0.40f)
            + up * (0.02f + 0.14f * amount) - fall * (0.16f * amount)
            + fall * (radius * std::cos(phase)) + up * (radius * std::sin(phase));
        const Vec3 reach = target - shoulder;
        constexpr float ArmReach = 0.55f;
        if (reach.length() > ArmReach) target = shoulder + reach * (ArmReach / reach.length());
        solveChainPoint(profile, coordinates, hand, {}, 2,
            rootInverse.rotate(target - root.position), 0.85f * amount);
    }
}

// "Vida" no ragdoll, no ar: caindo sem tocar nada, bracos em moinho fora de
// fase e pernas pedalando - quem perdeu o chao tenta se equilibrar, nao fica
// estatico. So alvos de junta; o corpo faz com os motores. (Deitado nao ha
// movimento proprio: os "surtos" de tentar se recuperar pareciam um AVC, a
// pedido do usuario ficou so o ragdoll com um pouco de tonus.)
void applyAirborneFlail(const RagdollProfile3D& profile,
    std::vector<Vec3>& coordinates, float time, float flail) {
    constexpr float Turn = 2.0f * Pi;
    if (flail > 0.0f) {
        const float arm = Turn * 1.3f * time;
        addJointCoordinate(profile, coordinates, "LeftUpperArm", 1,
            0.60f * flail * std::sin(arm));
        addJointCoordinate(profile, coordinates, "RightUpperArm", 1,
            -0.60f * flail * std::sin(arm + 2.1f));
        addJointCoordinate(profile, coordinates, "LeftForearm", 0,
            0.35f * flail * (0.5f + 0.5f * std::sin(arm + 1.0f)));
        addJointCoordinate(profile, coordinates, "RightForearm", 0,
            0.35f * flail * (0.5f + 0.5f * std::sin(arm + 3.1f)));
        const float leg = Turn * 1.05f * time;
        addJointCoordinate(profile, coordinates, "LeftThigh", 1,
            -0.28f * flail * (0.6f + std::sin(leg)));
        addJointCoordinate(profile, coordinates, "RightThigh", 1,
            -0.28f * flail * (0.6f + std::sin(leg + Pi)));
        addJointCoordinate(profile, coordinates, "LeftShin", 0,
            0.45f * flail * (0.7f + 0.6f * std::sin(leg + 0.8f)));
        addJointCoordinate(profile, coordinates, "RightShin", 0,
            0.45f * flail * (0.7f + 0.6f * std::sin(leg + Pi + 0.8f)));
    }
}

// A compact, symmetric flight pose: no clip is sampled for it (the Mixamo
// jump references have big, one-sided arm swings this game does not want),
// only a hand-authored tuck calibrated against the ranges those references
// showed for hips/knees. runningShare blends toward a slightly flatter,
// forward-carried tuck for a jump taken at speed; tuckAmount is 0 at
// liftoff/landing and peaks mid-air.
void applyFlightPose(const RagdollProfile3D& profile,
    std::vector<Vec3>& coordinates, float tuckAmount, float runningShare) {
    tuckAmount = std::clamp(tuckAmount, 0.0f, 1.0f);
    runningShare = std::clamp(runningShare, 0.0f, 1.0f);
    const float hipFlexion = -(0.62f + runningShare * 0.18f) * tuckAmount;
    const float trailingHipFlexion = -(0.34f + runningShare * 0.30f)
        * tuckAmount;
    const float kneeFlexion = (1.55f - runningShare * 0.25f) * tuckAmount;
    const float trailingKneeFlexion = (0.95f - runningShare * 0.20f)
        * tuckAmount;
    const float anklePitch = -0.18f * tuckAmount;
    setJointCoordinate(profile, coordinates, "LeftThigh", 1, hipFlexion);
    setJointCoordinate(profile, coordinates, "LeftShin", 0, kneeFlexion);
    setJointCoordinate(profile, coordinates, "LeftFoot", 0, anklePitch);
    setJointCoordinate(profile, coordinates, "RightThigh", 1,
        trailingHipFlexion);
    setJointCoordinate(profile, coordinates, "RightShin", 0,
        trailingKneeFlexion);
    setJointCoordinate(profile, coordinates, "RightFoot", 0, anklePitch);
    setJointCoordinate(profile, coordinates, "Abdomen", 1,
        (0.09f + runningShare * 0.10f) * tuckAmount);
    setJointCoordinate(profile, coordinates, "Chest", 1,
        (0.06f + runningShare * 0.08f) * tuckAmount);
    // Arms stay symmetric and modest on purpose: braced, not flailing.
    setJointCoordinate(profile, coordinates, "LeftUpperArm", 1,
        0.55f * tuckAmount);
    setJointCoordinate(profile, coordinates, "RightUpperArm", 1,
        -0.55f * tuckAmount);
    setJointCoordinate(profile, coordinates, "LeftForearm", 0,
        0.95f * tuckAmount);
    setJointCoordinate(profile, coordinates, "RightForearm", 0,
        0.95f * tuckAmount);
}

// Variacao de movimento ("vida"): ruido de valor suave e deterministico. A
// mesma semente, canal e posicao dao sempre o mesmo valor (os testes se
// repetem); cada personagem tem a sua semente - o seu jeito. Sem periodo:
// um valor novo a cada unidade de `position`, ligados por uma curva suave.
float varietyValue(std::uint32_t seed, std::uint32_t channel, std::int64_t lattice) {
    std::uint64_t h = (static_cast<std::uint64_t>(seed) << 32 | channel)
        ^ static_cast<std::uint64_t>(lattice) * 0x9E3779B97F4A7C15ull;
    h ^= h >> 30; h *= 0xBF58476D1CE4E5B9ull;
    h ^= h >> 27; h *= 0x94D049BB133111EBull;
    h ^= h >> 31;
    return static_cast<float>(h >> 40) / 8388607.5f - 1.0f;
}

float varietyNoise(std::uint32_t seed, std::uint32_t channel, float position) {
    const float base = std::floor(position);
    const float t = position - base;
    const std::int64_t lattice = static_cast<std::int64_t>(base);
    const float a = varietyValue(seed, channel, lattice);
    const float b = varietyValue(seed, channel, lattice + 1);
    return a + (b - a) * t * t * (3.0f - 2.0f * t);
}

// Andando e correndo, cada passada um pouco diferente. O balanco dos bracos
// (a parte do clipe que se inverte em meio ciclo) cresce ou diminui - o
// jeito do personagem (+-10%) mais a passada (+-14%, cada braco o seu) -,
// os bracos adiantam ou atrasam um pouco em relacao as pernas, o cotovelo
// dobra mais ou menos, o tronco gira um pouco mais ou menos e a cabeca nao
// fica cravada. So bracos, coluna e cabeca: pernas e pelve seguem o clipe
// (os pes, a passada e o equilibrio nao mudam).
void applyGaitVariety(const RagdollProfile3D& profile,
    const AnimationClip3D& clip, float cyclePhase, float strides,
    std::uint32_t seed, float amount, std::vector<Vec3>& sampled) {
    const auto at = [&](std::string_view id, float phase) {
        const AnimationTrack3D* track = findAnimationTrack3D(clip, id);
        if (track == nullptr) return Vec3 {};
        const float wrapped = phase - std::floor(phase);
        return sampleAnimationTrack3D(*track,
            stanceAlignedPhase(clip, wrapped) * clip.durationSeconds,
            clip.durationSeconds, clip.loops).jointPositionRadians;
    };
    // Amostra no ciclo (deslocado) e realca ou abranda o que se inverte em
    // meio ciclo: s(t) + k * (s(t) - s(t + 1/2)) / 2.
    const auto vary = [&](std::string_view id, float phase, float gain) {
        const std::size_t link = findLink(profile, id);
        if (link == 0 || link >= sampled.size()
            || findAnimationTrack3D(clip, id) == nullptr) return;
        const Vec3 now = at(id, phase);
        const Vec3 opposite = at(id, phase + 0.5f);
        sampled[link] = now + (now - opposite) * (0.5f * (gain - 1.0f));
    };
    constexpr std::array<std::string_view, 3> LeftArm {
        "LeftUpperArm", "LeftForearm", "LeftHand" };
    constexpr std::array<std::string_view, 3> RightArm {
        "RightUpperArm", "RightForearm", "RightHand" };
    const float traitSwing = 0.10f * varietyValue(seed, 1, 0);
    const float traitLead = 0.02f * varietyValue(seed, 2, 0);
    const float traitElbow = 0.08f * varietyValue(seed, 3, 0);
    for (std::uint32_t side = 0; side < 2; ++side) {
        const float gain = 1.0f + amount * (traitSwing
            + 0.14f * varietyNoise(seed, 10 + side, strides)
            + 0.05f * varietyNoise(seed, 12, strides * 0.5f));
        const float lead = amount * (traitLead
            + 0.015f * varietyNoise(seed, 14 + side, strides * 0.7f));
        for (std::string_view id : side == 0 ? LeftArm : RightArm)
            vary(id, cyclePhase + lead, gain);
        const std::size_t forearm = findLink(profile, side == 0
            ? "LeftForearm" : "RightForearm");
        if (forearm < sampled.size())
            sampled[forearm].x += amount * (traitElbow
                + 0.06f * varietyNoise(seed, 16 + side, strides * 0.6f));
    }
    const float torsoGain = 1.0f + amount * (0.06f * varietyValue(seed, 4, 0)
        + 0.10f * varietyNoise(seed, 20, strides * 0.8f));
    for (std::string_view id : { "Abdomen", "Chest", "UpperChest" })
        vary(id, cyclePhase, torsoGain);
    const std::size_t head = findLink(profile, "Head");
    if (head < sampled.size()) {
        sampled[head].x += amount * 0.03f * varietyNoise(seed, 22, strides * 0.9f);
        sampled[head].y += amount * 0.02f * varietyNoise(seed, 23, strides * 0.7f);
    }
}

// Parado: respira (o peito sobe e desce, ~0,25 Hz, cada um no seu ritmo, a
// cabeca devolvendo), os bracos e a coluna se acomodam devagar (variacao de
// alguns segundos), e de vez em quando ele desvia um pouco o olhar. So
// tronco, bracos e cabeca: os pes e o equilibrio ficam com o planejador.
void applyIdleVariety(const RagdollProfile3D& profile, float seconds,
    std::uint32_t seed, float amount, std::vector<Vec3>& sampled) {
    const auto add = [&](std::string_view id, std::size_t axis, float value) {
        const std::size_t link = findLink(profile, id);
        if (link == 0 || link >= sampled.size()) return;
        setComponent(sampled[link], axis, component(sampled[link], axis) + value);
    };
    const float breathRate = 0.25f + 0.04f * varietyValue(seed, 30, 0);
    const float breath = std::sin(2.0f * Pi * breathRate * seconds
        + Pi * varietyValue(seed, 31, 0));
    add("Chest", 1, -0.014f * amount * breath);
    add("UpperChest", 1, -0.018f * amount * breath);
    add("Neck", 1, 0.020f * amount * breath);
    const float drift = seconds / 2.6f;
    add("Abdomen", 0, 0.030f * amount * varietyNoise(seed, 32, drift));
    add("Chest", 2, 0.020f * amount * varietyNoise(seed, 33, drift * 0.8f));
    for (std::uint32_t side = 0; side < 2; ++side) {
        const std::string_view upper = side == 0 ? "LeftUpperArm" : "RightUpperArm";
        const std::string_view forearm = side == 0 ? "LeftForearm" : "RightForearm";
        add(upper, 0, 0.035f * amount * varietyNoise(seed, 34 + side, drift * 0.9f));
        add(upper, 2, 0.050f * amount * varietyNoise(seed, 36 + side, drift * 0.7f));
        add(forearm, 0, 0.10f * amount * varietyNoise(seed, 38 + side, drift * 0.6f));
    }
    // Olhar: um pouco sempre, e de tempos em tempos (onde o ruido lento
    // passa de 0,55) um desvio maior, ate ~9 graus.
    const float wander = varietyNoise(seed, 40, seconds / 4.5f);
    const float glance = std::copysign(
        smoothStep((std::abs(wander) - 0.55f) / 0.30f), wander);
    add("Head", 0, amount * (0.04f * varietyNoise(seed, 41, drift) + 0.16f * glance));
    add("Neck", 0, amount * 0.05f * glance);
    add("Head", 1, amount * 0.03f * varietyNoise(seed, 42, drift * 1.1f));
}

} // namespace

bool CharacterLocomotionAnimations3D::compatible(
    const RagdollProfile3D& profile) const {
    return validClip(idle, profile) && validClip(walk, profile);
}

CharacterGaitSpeeds3D characterGaitSpeeds3D(
    const CharacterLocomotionAnimations3D& animations) {
    CharacterGaitSpeeds3D speeds;
    if (animations.walk != nullptr) {
        speeds.walk = authoredHorizontalSpeed(*animations.walk);
    }
    speeds.walkBackward = animations.walkBackward != nullptr
        ? authoredHorizontalSpeed(*animations.walkBackward) : speeds.walk;
    speeds.sprint = animations.sprint != nullptr
        ? authoredHorizontalSpeed(*animations.sprint) : speeds.walk;
    speeds.sprintBackward = animations.sprintBackward != nullptr
        ? authoredHorizontalSpeed(*animations.sprintBackward)
        : speeds.walkBackward;
    // O strafe vem de uma referencia para cada lado, com passadas um pouco
    // diferentes (2,9 e 2,4 m/s no trote). De lado o personagem anda na media
    // dos dois; cada clipe acompanha pelo ritmo (playbackRate).
    const auto sideSpeed = [](const AnimationClip3D& left,
        const AnimationClip3D& right) {
        return 0.5f * (authoredHorizontalSpeed(left)
            + authoredHorizontalSpeed(right));
    };
    if (animations.strafeLeft != nullptr && animations.strafeRight != nullptr) {
        speeds.strafe = sideSpeed(*animations.strafeLeft,
            *animations.strafeRight);
        speeds.sprintStrafe = animations.sprintStrafeLeft != nullptr
                && animations.sprintStrafeRight != nullptr
            ? sideSpeed(*animations.sprintStrafeLeft,
                *animations.sprintStrafeRight)
            : speeds.strafe;
    }
    return speeds;
}

float characterGaitSpeed3D(const CharacterGaitSpeeds3D& speeds,
    float travelRelativeToLookRadians, bool sprinting) {
    const float forward = sprinting ? speeds.sprint : speeds.walk;
    const float backward = sprinting && speeds.sprintBackward > 0.0f
        ? speeds.sprintBackward : speeds.walkBackward;
    const float angle = std::abs(wrapAngle(travelRelativeToLookRadians));
    // As mesmas faixas de histerese que a locomocao usa para trocar de
    // setor: a velocidade passa de um ciclo para o outro dentro delas.
    const auto blend = [](float from, float to, float angle, float border,
        float hysteresis) {
        const float share = std::clamp(
            (angle - (border - hysteresis)) / (2.0f * hysteresis), 0.0f, 1.0f);
        return from + (to - from) * share;
    };
    if (speeds.strafe <= 0.0f) {
        return blend(forward, backward, angle,
            PlainSectorLimitRadians, PlainSectorHysteresisRadians);
    }
    const float side = sprinting ? speeds.sprintStrafe : speeds.strafe;
    return angle <= Pi * 0.5f
        ? blend(forward, side, angle, StrafeFrontLimitRadians,
            StrafeSectorHysteresisRadians)
        : blend(side, backward, angle, StrafeBackLimitRadians,
            StrafeSectorHysteresisRadians);
}

void CharacterLocomotion3D::reset(const RagdollProfile3D& profile,
    const RagdollState3D& state) {
    m_output = {};
    m_telemetry = {};
    m_coordinates.assign(profile.links.size(), {});
    m_coordinateVelocities.assign(profile.links.size(), {});
    m_transitionFrom.assign(profile.links.size(), {});
    m_poseVelocities.assign(profile.links.size(), {});
    m_transitionVelocity.assign(profile.links.size(), {});
    m_lean = {};
    m_leanVelocity = {};
    m_spineBalance = {};
    m_spineBalanceVelocity = {};
    m_spawnHeightOffset = 0.0f;
    m_spawnHeightCaptured = false;
    m_rootTilt = {};
    m_transitionFromRootTilt = {};
    m_rootTiltVelocity = {};
    m_transitionRootTiltVelocity = {};
    m_rootOffsetVelocity = {};
    m_transitionRootOffsetVelocity = {};
    m_previousClip = nullptr;
    m_previousSampledClip = nullptr;
    m_previousClipCoordinates.assign(profile.links.size(), {});
    m_rootOffset = {};
    m_transitionFromRootOffset = {};
    m_groundLift = 0.0f;
    m_terrainSlope = 0.0f;
    m_terrainSpeedCap = 9.0f;
    m_onInclineHold = 0.0f;
    m_pelvisOffset = 0.0f;
    m_pelvisOffsetVelocity = 0.0f;
    m_baseStepOffset = 0.0f;
    m_previousBaseGround = 0.0f;
    m_baseTracked = false;
    m_baseWasGrounded = false;
    m_gaitDirection = CharacterGaitDirection3D::Forward;
    m_feet = {};
    m_stateSeconds = 0.0f;
    m_transitionSeconds = 0.0f;
    m_transitionDurationSeconds = DefaultTransitionSeconds;
    m_cyclePhase = 0.0f;
    m_startClipActive = false;
    m_startClipSprint = false;
    m_startClipSeconds = 0.0f;
    m_authoredCurveBlend = 0.0f;
    m_authoredCurveKind = 0;
    m_idleSeconds = 0.0f;
    m_varietyStrides = 0.0f;
    m_ankleBalanceShift = {};
    m_turningRate = 0.0f;
    m_reactionStrength = 0.0f;
    m_physicsBlend = 0.0f;
    m_physicsHoldSeconds = 0.0f;
    m_previousDesiredVelocity = {};
    m_smoothedAcceleration = {};
    m_brakeDirection = {};
    m_brakeEntrySpeed = 0.0f;
    m_brakeReaction = {};
    m_brakeReactionVelocity = {};
    m_previousGrounded = true;
    m_initialized = !profile.links.empty()
        && state.links.size() == profile.links.size()
        && state.joints.size() == profile.links.size();
    if (!m_initialized) return;
    m_previousCenterOfMass = state.links.front().position;
    m_filteredCenterOfMassVelocity = {};
    m_gaitSpeedCap = 1000.0f;
    m_torsoTwist = 0.0f;
    m_headTurn = 0.0f;
    m_filteredLookPitch = 0.0f;
    m_lookValid = false;
    m_lookYawVelocity = 0.0f;
    m_lookPitchVelocity = 0.0f;
    m_poseCoordinates.clear();
    m_standingTurnTarget = 0.0f;
    m_clipPelvisYaw = 0.0f;
    m_clipPelvisToFootYaw = {};
    m_liftoffSpeed = 0.0f;
    m_liftoffVerticalSpeed = 0.0f;
    m_airborneFallSpeed = 0.0f;
    m_landingImpactSpeed = 0.0f;
    m_landingSettleSeconds = 0.18f;
    m_landingSinkPeak = 0.0f;
    m_sinceTouchdownSeconds = 10.0f;
    m_jumpKind = CharacterJumpKind3D::None;
    m_jumpClip = nullptr;
    m_flightPhase = 0.0f;
    m_elapsedSeconds = 0.0f;
    m_getUpPhase = CharacterGetUpPhase3D::None;
    m_fallOrientation = CharacterFallOrientation3D::None;
    m_getUpPhaseSeconds = 0.0f;
    m_recoveredStandingSeconds = 0.0f;
    m_getUpAttempt = 0;
    m_fallReflex = 0.0f;
    m_fallCommit = 0.0f;
    m_tripRecovery = 0.0f;
    m_tripRecoverySeconds = 0.0f;
    m_tripFoot = -1;
    m_tripElevating = false;
    m_tripDirection = { 1.0f, 0.0f, 0.0f };
    m_settleTone = 1.0f;
    m_settleRestSeconds = 0.0f;
    m_settlePoseValid = false;
    m_getUpAssistRamp = 0.0f;
    m_sinceGetUpSeconds = 10.0f;
    m_noGroundSeconds = 0.0f;
    m_previousTargetUp = { 0.0f, 0.0f, 1.0f };
    m_balanceReaction = 0.0f;
    m_reactiveShare = 0.0f;
    m_estimator.reset(profile);
    m_capturePoint = {};
    m_captureOutside = 0.0f;
    m_captureDirection = {};
    m_footwork.reset(profile);
    m_swingFromMeasuredPelvis = {};
    m_legRootFromMeasured = 0.0f;
    m_bodyRefValid = false;
    m_bodyRefHeight = 0.0f;
    m_plantFeetFromBody = false;
    m_handLinks = { findLink(profile, "LeftHand"),
        findLink(profile, "RightHand") };
    m_shinLinks = { findLink(profile, "LeftShin"),
        findLink(profile, "RightShin") };
    m_chestLink = findLink(profile, "Chest");
    for (std::size_t index = 1; index < profile.links.size(); ++index) {
        m_coordinates[index] = {
            state.joints[index].positionRadians[0],
            state.joints[index].positionRadians[1],
            state.joints[index].positionRadians[2]
        };
        m_transitionFrom[index] = m_coordinates[index];
        m_previousClipCoordinates[index] = m_coordinates[index];
        if (profile.links[index].id == "LeftFoot") {
            m_feet[0].linkIndex = index;
            m_feet[0].valid = true;
        } else if (profile.links[index].id == "RightFoot") {
            m_feet[1].linkIndex = index;
            m_feet[1].valid = true;
        }
    }
    m_previousRootPosition = state.links.front().position;
    m_previousRootOrientation = state.links.front().orientation;
    m_previousRisingPose = false;
    m_previousReferencePhase = CharacterGetUpPhase3D::None;
    m_previousRootBase = state.links.front().position;
    m_previousGuideVelocity = {};
    m_previousGuideAngularVelocity = {};
    const Vec3 forward = state.links.front().orientation.rotate(
        { 1.0f, 0.0f, 0.0f });
    m_facingYaw = std::atan2(forward.y, forward.x);
}

const CharacterLocomotion3D::ClipFootEvents&
CharacterLocomotion3D::footEventsFor(const RagdollProfile3D& profile,
    const AnimationClip3D& clip) {
    for (const ClipFootEvents& cached : m_clipFootEvents) {
        if (cached.clip == &clip) return cached;
    }
    ClipFootEvents events;
    events.clip = &clip;
    const float duration = std::max(0.05f, clip.durationSeconds);
    const AnimationTrack3D* rootTrack = findAnimationTrack3D(clip,
        profile.links.front().id);
    constexpr int Samples = 240;
    for (std::size_t side = 0; side < 2; ++side) {
        const AnimationScalarTrack3D& contact = side == 0
            ? clip.leftFootContact : clip.rightFootContact;
        const FootPlant& foot = m_feet[side];
        if (contact.keyframes.empty() || !foot.valid) continue;
        const int shin = profile.links[foot.linkIndex].parentIndex;
        const int thigh = shin > 0
            ? profile.links[static_cast<std::size_t>(shin)].parentIndex : -1;
        if (thigh <= 0) continue;
        std::array<float, Samples> weight {};
        for (int i = 0; i < Samples; ++i) {
            weight[static_cast<std::size_t>(i)] = sampleAnimationScalarTrack3D(
                contact, duration * static_cast<float>(i) / Samples, duration,
                true);
        }
        const auto at = [&](int i) {
            return weight[static_cast<std::size_t>((i % Samples + Samples)
                % Samples)];
        };
        for (int i = 0; i < Samples; ++i) {
            if (!(at(i - 1) <= 0.55f && at(i) > 0.55f)) continue;
            FootTouchdown touchdown;
            touchdown.touchdown = duration * static_cast<float>(i) / Samples;
            touchdown.liftoff = touchdown.touchdown;
            for (int back = 1; back < Samples; ++back) {
                if (at(i - back) < 0.30f && at(i - back - 1) >= 0.30f) {
                    touchdown.liftoff = duration
                        * static_cast<float>(((i - back) % Samples + Samples)
                            % Samples) / Samples;
                    break;
                }
            }
            std::vector<Vec3> coordinates(profile.links.size());
            for (std::size_t index = 1; index < profile.links.size(); ++index) {
                const AnimationTrack3D* track = findAnimationTrack3D(clip,
                    profile.links[index].id);
                if (track == nullptr) continue;
                coordinates[index] = sampleAnimationTrack3D(*track,
                    touchdown.touchdown, duration, clip.loops)
                        .jointPositionRadians;
            }
            Vec3 translation;
            Quaternion rotation;
            if (rootTrack != nullptr) {
                const AnimationTransformSample3D root = sampleAnimationTrack3D(
                    *rootTrack, touchdown.touchdown, duration, clip.loops);
                translation = root.translationOffsetMeters;
                rotation = root.rotationDelta.normalized();
            }
            RagdollAnimationPose3D pose;
            buildLocalPose(profile, coordinates, pose);
            const Vec3 hip = profile.links[0].modelOrientation.conjugate()
                .rotate(profile.links[static_cast<std::size_t>(thigh)]
                    .inboundJoint.anchorModelPosition
                    - profile.links[0].modelPosition);
            touchdown.footInClip = translation
                + rotation.rotate(pose.linkPositions[foot.linkIndex]);
            touchdown.hipInClip = translation + rotation.rotate(hip);
            touchdown.forwardInClip = (rotation
                * pose.linkOrientations[foot.linkIndex]).rotate(
                    profile.links[foot.linkIndex].modelOrientation.conjugate()
                        .rotate({ 1.0f, 0.0f, 0.0f }));
            touchdown.footInClip.z = 0.0f;
            touchdown.hipInClip.z = 0.0f;
            events.touchdowns[side].push_back(touchdown);
        }
    }
    m_clipFootEvents.push_back(std::move(events));
    return m_clipFootEvents.back();
}

void CharacterLocomotion3D::enterState(CharacterLocomotionState3D state) {
    if (state == m_telemetry.state) return;
    // O crossfade parte da pose do CLIPE do tick anterior, sem o giro do
    // olhar e sem o IK dos pes, que sao somados de novo por cima: partindo da
    // pose final, o olhar entrava duas vezes e cabeca e tronco davam um
    // tranco a cada troca de estado (medido: cabeca a 50 rad/s ao girar a
    // camera parado, trocando entre parado e girando no lugar).
    m_transitionFrom = m_poseCoordinates.size() == m_coordinates.size()
        ? m_poseCoordinates : m_coordinates;
    m_transitionFromRootTilt = m_rootTilt;
    m_transitionVelocity = m_poseVelocities;
    m_transitionRootTiltVelocity = m_rootTiltVelocity;
    m_transitionRootOffsetVelocity = m_rootOffsetVelocity;
    m_transitionFromRootOffset = m_rootOffset;
    m_telemetry.state = state;
    m_stateSeconds = 0.0f;
    m_transitionSeconds = 0.0f;
    m_transitionDurationSeconds = DefaultTransitionSeconds;
    // Caindo -> deitado e deitado -> levantar: o corpo muda de pose inteiro;
    // em 0,14 s os membros davam um tranco.
    if (state == CharacterLocomotionState3D::Fallen
        || state == CharacterLocomotionState3D::GettingUp) {
        m_transitionDurationSeconds = 0.40f;
    }
    if (state == CharacterLocomotionState3D::Idle) {
        m_cyclePhase = 0.0f;
    }
    if (state == CharacterLocomotionState3D::Airborne
        || state == CharacterLocomotionState3D::JumpStarting
        || state == CharacterLocomotionState3D::Fallen) {
        for (FootPlant& foot : m_feet) {
            foot.planted = false;
            foot.repositioning = false;
            foot.recoveryStep = false;
        }
    }
}

void CharacterLocomotion3D::update(const RagdollProfile3D& profile,
    const CharacterLocomotionAnimations3D& animations,
    const RagdollState3D& state,
    const RagdollDynamics3D& dynamics,
    const CharacterLocomotionInput3D& input,
    float deltaTime) {
    m_output = {};
    if (!std::isfinite(deltaTime) || deltaTime <= 0.0f
        || !animations.compatible(profile)
        || state.links.size() != profile.links.size()
        || state.joints.size() != profile.links.size()
        || !finite(input.rootPositionWorld)
        || !finite(input.rootVelocityWorld)
        || !finite(input.proxyVelocityWorld)
        || !std::isfinite(input.facingYawRadians)
        || !std::isfinite(input.lookPitchRadians)) {
        return;
    }
    if (!m_initialized) reset(profile, state);
    if (!m_initialized) return;
    deltaTime = std::min(deltaTime, 0.05f);
    const std::vector<Vec3> previousCoordinates = m_coordinates;
    // Estado fisico comum deste tick (tambem lido pela assistencia).
    const CharacterPhysicalState3D& physical = m_estimator.update(profile, state,
        input.footGround, deltaTime);

    // Centro de massa pela massa de cada link (do estimador comum), para
    // todos os personagens. Antes, sem RagdollDynamics3D valido - o caso de
    // todo boneco nao controlado -, o centro de massa era a pelve.
    // Tracked every tick, independent of authority, so the balance-assist
    // velocity below never sees a stale gap when it turns on mid-collision.
    const Vec3 centerOfMass = physical.valid ? physical.centerOfMassWorld
        : state.links.front().position;
    Vec3 measuredComVelocity = physical.valid ? physical.centerOfMassVelocityWorld
        : Vec3 {};
    (void) dynamics;
    if (!finite(measuredComVelocity)) measuredComVelocity = {};
    const float comVelocityBlend = 1.0f - std::exp(-16.0f * deltaTime);
    m_filteredCenterOfMassVelocity += (measuredComVelocity
        - m_filteredCenterOfMassVelocity) * comVelocityBlend;
    m_previousCenterOfMass = centerOfMass;
    m_elapsedSeconds += deltaTime;

    // Fall detection: a real trip/collision this controller could not
    // absorb through the reaction/interference softening below. Contacts
    // are the physical ground truth here — hands/knees/torso touching the
    // floor, not the authored-clip foot weights the normal locomotion foot
    // lock further down uses for a completely different purpose (knowing
    // when a walking step has landed).
    bool anyGroundContact = false;
    std::array<bool, 2> handGroundContact {};
    std::array<bool, 2> kneeGroundContact {};
    std::array<bool, 2> feetGroundContact {};
    float contactFloor = std::numeric_limits<float>::infinity();
    bool touchingAnything = false;
    for (const RagdollContactPoint3D& contact : state.contacts) {
        if (contact.normalImpulseNewtonSeconds > 0.00001f) touchingAnything = true;
        // Chao e superficie parada: uma caixa atingindo o corpo no ar, por
        // baixo, fazia a queda ser declarada ainda no ar.
        if (contact.normal.z < 0.40f || contact.otherBodyDynamic
            || contact.normalImpulseNewtonSeconds <= 0.00001f) {
            continue;
        }
        anyGroundContact = true;
        contactFloor = std::min(contactFloor, contact.position.z);
        for (std::size_t side = 0; side < 2; ++side) {
            if (contact.linkIndex == m_handLinks[side]) {
                handGroundContact[side] = true;
            }
            if (contact.linkIndex == m_shinLinks[side]) {
                kneeGroundContact[side] = true;
            }
        }
    }
    // Pes: o apoio medido pelo estimador (contato com a normal para cima,
    // chao parado), o mesmo criterio de antes num lugar so.
    for (std::size_t side = 0; side < 2; ++side) {
        const FootSupportEstimate3D& foot = physical.feet[side];
        feetGroundContact[side] = m_feet[side].valid && foot.supporting
            && foot.evidence != ContactEvidence3D::Geometric && !foot.onDynamicBody;
    }

    // Sem tocar nada (no ar): para o espernear da queda.
    m_noGroundSeconds = touchingAnything ? 0.0f
        : std::min(10.0f, m_noGroundSeconds + deltaTime);
    float rootUpright = 1.0f;
    if (!state.links.empty()) {
        rootUpright = state.links.front().orientation
            .rotate({ 0.0f, 0.0f, 1.0f }).z;
    }
    // The capsule's implied ground (feet height it stands at) doubles as a
    // ground estimate here; contacts already gate whether anything is
    // actually down, this only sizes how far down it is.
    const float groundHeightGuess =
        input.rootPositionWorld.z - profile.standingRootHeightMeters;
    // No modo fisico, contra a superficie que o corpo toca de fato: a
    // capsula decola antes no pulo, e medido contra ela o corpo no ar (o pe
    // ainda empurrando o chao) parecia "caido" - queda falsa na decolagem.
    const float measuredComHeight = std::max(0.08f, centerOfMass.z
        - (input.forceDrivenRoot && std::isfinite(contactFloor)
            ? contactFloor : groundHeightGuess));
    // Only meaningful while physics is the one holding the body up. In
    // animation mode the authored pose says the character is standing, so
    // "fallen" is not a state it can observe - and reading the simulated
    // bodies anyway is what produced the fall/stand-up/fall loop: those
    // bodies are still lying down for a few ticks after a recovery hands the
    // pose back, which instantly re-triggered another fall.
    // Com a raiz livre (forceDrivenRoot) o corpo e sempre o fisico: a queda
    // e reconhecida pelo estado dele, sem depender de um impacto antes.
    const bool physicsOwnsBody = input.forceDrivenRoot || m_physicsBlend > 0.5f;
    const bool fallenObservation = physicsOwnsBody
        && m_elapsedSeconds > 0.6f
        && anyGroundContact
        && (rootUpright < 0.45f || measuredComHeight < 0.52f);

    const auto enterGetUpPhase = [&](CharacterGetUpPhase3D phase) {
        m_getUpPhase = phase;
        m_getUpPhaseSeconds = 0.0f;
    };
    const auto classifyFallOrientation = [&]() {
        const std::size_t reference = m_chestLink < state.links.size()
            ? m_chestLink : 0;
        const Vec3 bodyForward = state.links[reference]
            .orientation.rotate({ 1.0f, 0.0f, 0.0f });
        m_fallOrientation = bodyForward.z < 0.0f
            ? CharacterFallOrientation3D::FaceDown
            : CharacterFallOrientation3D::FaceUp;
    };
    if (m_getUpPhase == CharacterGetUpPhase3D::None) {
        if (fallenObservation) {
            classifyFallOrientation();
            enterGetUpPhase(CharacterGetUpPhase3D::Settling);
            ++m_getUpAttempt;
            m_recoveredStandingSeconds = 0.0f;
            m_settleTone = SettleImpactTone;
            m_settleRestSeconds = 0.0f;
            m_settlePoseValid = false;
            for (FootPlant& foot : m_feet) {
                foot.planted = false;
                foot.repositioning = false;
                foot.recoveryStep = false;
            }
        }
    } else if (!input.manipulated) {
        // Being held by the PhysGun freezes recovery instead of fighting the
        // player back toward a stand-up target.
        m_getUpPhaseSeconds += deltaTime;
        if (m_getUpPhase == CharacterGetUpPhase3D::Settling) {
            // Deitado: se rolou, a pose deitada (e o levantar) troca - com
            // folga, para nao ficar trocando de lado.
            if (m_chestLink < state.links.size()) {
                const float facing = state.links[m_chestLink].orientation
                    .rotate({ 1.0f, 0.0f, 0.0f }).z;
                if (m_fallOrientation == CharacterFallOrientation3D::FaceUp
                    && facing < -0.25f) {
                    m_fallOrientation = CharacterFallOrientation3D::FaceDown;
                } else if (m_fallOrientation
                        == CharacterFallOrientation3D::FaceDown
                    && facing > 0.25f) {
                    m_fallOrientation = CharacterFallOrientation3D::FaceUp;
                }
            }
            // Let the body actually come to rest before committing to a
            // recovery: which clip to play depends on which way it lands, and
            // that is not decided while it is still tumbling.
            float fastestLink = 0.0f;
            for (const PhysicsBodyState3D& link : state.links) {
                fastestLink = std::max(fastestLink,
                    link.linearVelocity.length());
            }
            // Tonus deitado ("ressustentacao"): enquanto o corpo ainda rola
            // ou desliza, os musculos cedem - a energia fica nas juntas em vez
            // de virar cambalhota do corpo inteiro, duro. Parado de vez, ele
            // fica largado ~0,7 s e o tonus volta aos poucos (~0,75 s),
            // enquanto ele se arruma para levantar (pose, mais abaixo); so
            // entao levanta.
            //
            // "Parado" e o tronco (pelve e peito), nao o membro mais rapido:
            // o proprio tonus voltando leva bracos e pernas para a pose
            // deitada, e medido pelos membros isso contava como "ainda
            // rolando" - o tonus desabava de novo, em ciclo, ate o limite de
            // tempo. Depois que ele comecou a voltar, so um tranco de verdade
            // no tronco o derruba de novo.
            float coreSpeed = state.links.front().linearVelocity.length();
            if (m_chestLink < state.links.size()) {
                coreSpeed = std::max(coreSpeed,
                    state.links[m_chestLink].linearVelocity.length());
            }
            // Na primeira vez fica largado mexendo-se; numa nova tentativa (ja
            // acordado, tentando) espera pouco.
            const float restBeforeGather = m_getUpAttempt > 1
                ? RetryRestBeforeGather : SettleRestBeforeGather;
            const bool recovering = m_settleRestSeconds > restBeforeGather;
            // Arrumando-se para levantar ele mesmo mexe o tronco (de lado para
            // de costas e um rolar): so um tranco de verdade interrompe.
            // Depois de parar, os proprios movimentos deitado (erguer a
            // cabeca, apoiar nos cotovelos) tambem nao contam como rolar.
            const float restLimit = recovering ? 1.50f
                : m_settleRestSeconds > 0.25f ? 0.80f : 0.35f;
            m_settleRestSeconds = coreSpeed < restLimit
                ? m_settleRestSeconds + deltaTime : 0.0f;
            m_settleTone = m_settleRestSeconds > restBeforeGather
                ? std::min(1.0f, m_settleTone + deltaTime * 1.0f)
                : std::max(SettleRestingTone, m_settleTone - deltaTime * 2.5f);
            // Levantar so com o corpo deitado: invertido (pernas por cima da
            // cabeca, pelve alem de ~125 graus) o clipe comecava de uma pose
            // que o corpo nao tinha e ele rolava de novo.
            const float settledTilt = std::acos(std::clamp(rootUpright, -1.0f, 1.0f));
            // E com o corpo ja na pose de levantar (se arrumou): um braco
            // preso embaixo do corpo nao chegava, e ao levantar os ganhos
            // o arrancavam de la (mao a 14 m/s).
            const bool settled = m_getUpPhaseSeconds > 0.55f
                && coreSpeed < 0.35f && fastestLink < 1.0f
                && m_settleTone > 0.97f && settledTilt < 2.2f
                && m_telemetry.jointErrorRmsDegrees < 12.0f;
            // No modo fisico o corpo cai com mais energia e desliza mais:
            // comecar o levantar com ele ainda andando (o limite de 2 s)
            // puxava a raiz de uma vez para o clipe - membros a 77 m/s.
            const float settleLimit = input.forceDrivenRoot ? 4.0f : 2.0f;
            if (settled || m_getUpPhaseSeconds > settleLimit) {
                // O lado (costas/bruços) e o do deitado, que ja tem folga:
                // reclassificar aqui, de lado (peito na vertical), trocava o
                // levantar para o outro clipe com o corpo arrumado para este
                // (mao a 14 m/s, levantar emperrado).
                m_getUpClipSeconds = 0.0f;
                if (!state.links.empty()) {
                    // The clip plays where the body is lying, not where the
                    // capsule happens to be standing.
                    const Vec3 root = state.links.front().position;
                    m_getUpAnchorWorld = { root.x, root.y, root.z };
                    const Vec3 forward = state.links.front()
                        .orientation.rotate({ 1.0f, 0.0f, 0.0f });
                    const float planar = std::hypot(forward.x, forward.y);
                    m_getUpAnchorYaw = planar > 0.05f
                        ? std::atan2(forward.y, forward.x) : m_facingYaw;
                }
                m_getUpRate = 1.0f;
                m_getUpStruggle = 0.0f;
                m_getUpStallSeconds = 0.0f;
                m_getUpHoldSeconds = 0.0f;
                m_getUpExpectedValid = false;
                m_risingFromMeasured.assign(state.joints.size(), Vec3 {});
                for (std::size_t link = 1; link < state.joints.size(); ++link) {
                    m_risingFromMeasured[link] = { state.joints[link].positionRadians[0],
                        state.joints[link].positionRadians[1],
                        state.joints[link].positionRadians[2] };
                }
                enterGetUpPhase(CharacterGetUpPhase3D::Rising);
            }
        } else if (m_getUpPhase == CharacterGetUpPhase3D::Rising) {
            // Levantar biomecanico: o clipe e o roteiro das juntas; quem
            // levanta e o corpo. O tempo do clipe anda no ritmo que o corpo
            // acompanha (altura e inclinacao da pelve contra as do clipe):
            // atrasado, o clipe espera. Nada carrega a raiz - antes ela era
            // levada ao clipe, o "teleporte" (pelve a 677 mm num tick).
            if (m_getUpExpectedValid && !state.links.empty()) {
                const float ground = input.rootPositionWorld.z
                    - profile.standingRootHeightMeters;
                const float height = state.links.front().position.z - ground;
                const float tilt = std::acos(std::clamp(rootUpright, -1.0f, 1.0f));
                const float lowBy = m_getUpExpectedHeight - height;
                const float tiltedBy = tilt - m_getUpExpectedTilt;
                const float heightRate = 1.0f - (lowBy - 0.06f) / 0.14f;
                // Na fase final (o clipe ja quase em pe) a inclinacao pesa
                // mais: com a pelve inclinada alem do clipe ele nao segue
                // subindo - primeiro endireita, depois fica de pe. Antes o
                // clipe seguia e ele subia como uma prancha inclinada.
                const bool finalPhase = m_getUpExpectedTilt < 0.8f
                    && height > 0.55f;
                const float tiltRate = finalPhase
                    ? 1.0f - (tiltedBy - 0.12f) / 0.25f
                    : 1.0f - (tiltedBy - 0.21f) / 0.44f;
                // As pernas tambem: se coxas e canelas estao longe da pose
                // do clipe (os joelhos ainda nao vieram para baixo do corpo),
                // o clipe espera - subir a pelve com as pernas esticadas para
                // tras deixava o corpo numa ponte de cabeca no chao.
                float legsBehind = 0.0f;
                for (std::size_t index = 1; index < profile.links.size()
                        && index < state.joints.size()
                        && index < m_coordinates.size(); ++index) {
                    const std::string& id = profile.links[index].id;
                    if (id.find("Thigh") == std::string::npos
                        && id.find("Shin") == std::string::npos) continue;
                    const Vec3 measured { state.joints[index].positionRadians[0],
                        state.joints[index].positionRadians[1],
                        state.joints[index].positionRadians[2] };
                    legsBehind = std::max(legsBehind,
                        (m_coordinates[index] - measured).length());
                }
                const float legsRate = 1.0f - (legsBehind - 0.45f) / 0.50f;
                // O peito tambem: de bruços, a flexao de bracos (peito fora do
                // chao) vem antes de puxar os joelhos. Sem esperar por ela o
                // clipe seguia com o peito no chao e o corpo se encolhia de
                // cabeca no chao, arrastando-a.
                float chestRate = 1.0f;
                if (m_chestLink < state.links.size()) {
                    const float chest = state.links[m_chestLink].position.z - ground;
                    chestRate = 1.0f
                        - (m_getUpExpectedChestHeight - chest - 0.08f) / 0.15f;
                }
                m_getUpRate = std::clamp(std::min({ heightRate, tiltRate,
                    legsRate, chestRate }), 0.1f, 1.0f);
            } else {
                m_getUpRate = 1.0f;
            }
            // Emperrado, o esforco (e o teto da ajuda) sobe devagar.
            // (Sobe em ~0,5 s: mais devagar, o corpo desabava antes de a
            // ajuda responder - fase de sentar do levantar de costas.)
            m_getUpStruggle = m_getUpRate < 0.5f
                ? std::min(1.0f, m_getUpStruggle + deltaTime * 2.0f)
                : std::max(0.0f, m_getUpStruggle - deltaTime
                    * (m_getUpRate > 0.8f ? 1.0f : 0.0f));
            // Emperrado de verdade: o clipe esperando E o corpo sem subir nem
            // endireitar. Progredindo devagar (a pelve subindo, a inclinacao
            // caindo), ele nao desiste - antes desistia no meio da subida.
            bool progressing = false;
            if (!state.links.empty()) {
                const float tiltNow = std::acos(std::clamp(rootUpright, -1.0f, 1.0f));
                progressing = state.links.front().linearVelocity.z > 0.05f
                    || (m_previousTilt - tiltNow) / std::max(deltaTime, 1e-4f) > 0.08f;
            }
            m_getUpStallSeconds = m_getUpRate <= 0.2f && !progressing
                ? m_getUpStallSeconds + deltaTime : std::max(0.0f,
                    m_getUpStallSeconds - (progressing ? deltaTime : 3.0f));
            m_getUpClipSeconds += deltaTime * m_getUpRate;
            const AnimationClip3D* clip = m_fallOrientation
                    == CharacterFallOrientation3D::FaceDown
                ? animations.standUpFront : animations.standUpBack;
            const float duration = clip && clip->durationSeconds > 0.05f
                ? clip->durationSeconds : 1.6f;
            // Fim do clipe so conta com o corpo em pe de verdade; senao ele
            // segura a pose final (com a ajuda) um pouco, e desiste.
            const bool upright = rootUpright > 0.94f && !state.links.empty()
                && state.links.front().position.z
                    - (input.rootPositionWorld.z - profile.standingRootHeightMeters)
                    > profile.standingRootHeightMeters * 0.80f;
            if (m_getUpClipSeconds >= duration && !upright) {
                m_getUpClipSeconds = duration;
                m_getUpHoldSeconds += deltaTime;
            }
            // (E um teto: oscilando para sempre sem terminar, tenta de novo.)
            if (m_getUpStallSeconds > 3.0f || m_getUpHoldSeconds > 1.5f
                || m_getUpPhaseSeconds > 9.0f) {
                // Nao deu: volta a deitar e tenta de novo (pode escolher o
                // outro clipe, se rolou).
                classifyFallOrientation();
                enterGetUpPhase(CharacterGetUpPhase3D::Settling);
                ++m_getUpAttempt;
                m_settleRestSeconds = 0.0f;
                m_settlePoseValid = false;
                m_getUpStallSeconds = 0.0f;
                m_getUpHoldSeconds = 0.0f;
                m_getUpStruggle = 0.0f;
            } else if (m_getUpClipSeconds >= duration && upright) {
                m_getUpPhase = CharacterGetUpPhase3D::None;
                m_fallOrientation = CharacterFallOrientation3D::None;
                m_getUpPhaseSeconds = 0.0f;
                m_getUpClipSeconds = 0.0f;
                m_recoveredStandingSeconds = 0.0f;
                m_getUpAttempt = 0;
                m_getUpHoldSeconds = 0.0f;
                m_getUpStallSeconds = 0.0f;
                m_getUpStruggle = 0.0f;
                m_getUpExpectedValid = false;
                // O rumo e o do corpo em pe, onde ele terminou de levantar.
                {
                    const Vec3 forward = state.links.front().orientation
                        .rotate({ 1.0f, 0.0f, 0.0f });
                    m_facingYaw = std::hypot(forward.x, forward.y) > 0.2f
                        ? std::atan2(forward.y, forward.x) - m_clipPelvisYaw
                        : m_getUpAnchorYaw;
                }
                m_turningRate = 0.0f;
                // Os pes ficam onde estao, travados no chao (abaixo, onde a
                // sonda do chao existe). Soltos, o IK levava os dois de uma vez
                // para a base parada - o "espacate" da troca. Os passos de
                // acomodacao refazem a base, um pe de cada vez.
                m_plantFeetFromBody = true;
                // O olhar entra de novo aos poucos (o filtro continua de zero).
                m_torsoTwist = 0.0f;
                m_headTurn = 0.0f;
            }
        }
    }
    const bool getUpActive = m_getUpPhase != CharacterGetUpPhase3D::None;
    m_sinceGetUpSeconds = getUpActive ? 0.0f
        : std::min(10.0f, m_sinceGetUpSeconds + deltaTime);

    // Reflexos de queda: com o corpo tombando de verdade (pelve alem de ~40
    // graus da vertical; no modo antigo, a partir de 25 graus depois de uma
    // pancada) os reflexos entram e as juntas ficam com tonus - nao a 3% da
    // forca, que era o "corpo morto molengo". Deitado, fica inteiro; no
    // levantar, sai.
    {
        const float tilt = std::acos(std::clamp(rootUpright, -1.0f, 1.0f));
        // Tombando rapido (mais de 1,5 rad/s) os reflexos entram ja aos 25
        // graus: numa queda de lado o corpo vai ao chao em ~0,35 s, e
        // esperando os 40 graus as maos ainda estavam a 30 cm do chao quando
        // a cabeca batia. Inclinacao parada (a corrida inclina ate ~35) nao.
        const float tiltRate = (tilt - m_previousTilt) / std::max(deltaTime, 1e-4f);
        m_previousTilt = tilt;
        const float quickTipping = smoothStep((tilt - 0.35f) / 0.26f)
            * smoothStep((tiltRate - 1.0f) / 1.0f);
        const float tipping = !physicsOwnsBody ? 0.0f
            : input.forceDrivenRoot
                ? std::max(smoothStep((tilt - 0.70f) / 0.26f), quickTipping)
                : std::max(smoothStep((tilt - 0.44f) / 0.26f), quickTipping)
                    * smoothStep((m_physicsBlend - 0.3f) / 0.4f);
        const float wanted = m_getUpPhase == CharacterGetUpPhase3D::Settling
            ? 1.0f : m_getUpPhase == CharacterGetUpPhase3D::Rising ? 0.0f
            : tipping;
        m_fallReflex += (wanted - m_fallReflex) * std::clamp(
            deltaTime * (wanted > m_fallReflex ? 18.0f : 3.0f), 0.0f, 1.0f);
        // Desequilibrado nao e caido: com o corpo fisico, os bracos protegem
        // ja no comeco (acima), mas as pernas seguem no passo, com o tonus de
        // pe, ate a queda ficar sem volta - tombado alem de ~50 graus e ainda
        // indo (ou 40 graus girando muito rapido). Ir logo para as pernas
        // moles tirava a chance de o passo salvar o equilibrio e o comando do
        // jogador parava de valer no meio do tropeco.
        const float committing = input.forceDrivenRoot
            ? std::max(smoothStep((tilt - 0.87f) / 0.22f)
                    * smoothStep((tiltRate + 0.3f) / 0.6f),
                smoothStep((tilt - 0.70f) / 0.26f)
                    * smoothStep((tiltRate - 2.5f) / 1.0f))
            : tipping;
        const float commitWanted = m_getUpPhase == CharacterGetUpPhase3D::Settling
            ? 1.0f : m_getUpPhase == CharacterGetUpPhase3D::Rising ? 0.0f
            : std::max(committing, input.forceDrivenRoot && m_fallCommit > 0.5f
                ? smoothStep((tilt - 0.70f) / 0.26f) : 0.0f);
        // De volta em pe (tronco a menos de ~25 graus), as pernas voltam logo
        // ao passo e ao tonus: com a volta lenta ele se empurrava de cima de
        // uma caixa, ficava reto, e caia de costas de pernas ainda moles.
        const float commitRelease = m_getUpPhase == CharacterGetUpPhase3D::None
            && tilt < 0.44f && (feetGroundContact[0] || feetGroundContact[1])
            ? 10.0f : 3.0f;
        m_fallCommit += (commitWanted - m_fallCommit) * std::clamp(
            deltaTime * (commitWanted > m_fallCommit ? 18.0f : commitRelease),
            0.0f, 1.0f);
        // Rumo da queda: para onde o centro de massa vai; devagar, para onde
        // o corpo tomba.
        Vec3 fall = m_filteredCenterOfMassVelocity;
        fall.z = 0.0f;
        if (fall.lengthSquared() < 0.16f && !state.links.empty()) {
            fall = state.links.front().orientation.rotate({ 0.0f, 0.0f, 1.0f });
            fall.z = 0.0f;
        }
        if (fall.lengthSquared() > 0.0004f) {
            const Vec3 unit = fall.normalized();
            m_fallDirection = (m_fallDirection + (unit - m_fallDirection)
                * std::clamp(deltaTime * 12.0f, 0.0f, 1.0f));
            if (m_fallDirection.lengthSquared() > 1e-6f)
                m_fallDirection = m_fallDirection.normalized();
        }
    }

    // How much of the simulated body the player actually sees. Zero is the
    // normal case and means the rendered pose IS the authored pose, bit for
    // bit — the bodies still exist and still collide, they just do not get a
    // say in how the character looks. This is the same split Unreal draws
    // with PhysicsBlendWeight (see PhysAnim.cpp): animation owns the pose,
    // physics is blended in per body by weight, and at weight zero the
    // simulated result is discarded entirely instead of being "mostly
    // suppressed". Letting physics always contribute a little is what made
    // the character stand crooked, lean while walking and jitter.
    //
    // Only a dynamic body can raise it: another character, a prop, a
    // scripted moment later on. Walls, floors and curbs are static and
    // deliberately never push the character out of animation - clipping a
    // wall is a pose problem, not a reason to go limp.
    float dynamicImpulse = 0.0f;
    for (const RagdollContactPoint3D& contact : state.contacts) {
        if (!contact.otherBodyDynamic) continue;
        dynamicImpulse += std::max(0.0f, contact.normalImpulseNewtonSeconds);
    }
    // Scaled against what it actually takes to move this body, not against
    // its weight over a tick: a crate merely leaning on the character
    // delivers about mass*g*dt of impulse every tick forever, and measuring
    // against that made any resting contact read as a full-force hit and
    // drop the character on the spot. The gate is a real change in momentum
    // - ignore anything under ~0.05 m/s worth, reach full physics at
    // ~0.45 m/s worth, which is a genuine shove.
    const float mass = std::max(1.0f, profile.totalMassKg);
    const float deadzone = mass * 0.05f;
    const float fullScale = mass * 0.45f;
    // Com a raiz livre nao ha limiar de impacto: a colisao age direto no
    // corpo e a cedencia local vem do controlador fisico.
    const float impact = input.forceDrivenRoot ? 0.0f : std::clamp(
        (dynamicImpulse - deadzone) / std::max(0.01f, fullScale - deadzone),
        0.0f, 1.0f);
    if (impact > 0.0f) {
        // Proportional to the hit, and it never snaps: a shove pushes the
        // weight up quickly, then it holds while contact lasts.
        const float wanted = impact;
        m_physicsBlend = std::max(m_physicsBlend,
            std::min(1.0f, m_physicsBlend + wanted * deltaTime * 9.0f));
        m_physicsHoldSeconds = std::max(m_physicsHoldSeconds,
            0.25f + wanted * 0.75f);
    } else if (m_physicsHoldSeconds > 0.0f) {
        m_physicsHoldSeconds = std::max(0.0f,
            m_physicsHoldSeconds - deltaTime);
    } else {
        // Deliberately slower than the way in: coming back the instant
        // contact breaks is what made bumping another character read as
        // nothing happening at all.
        m_physicsBlend = std::max(0.0f, m_physicsBlend - deltaTime * 0.65f);
    }
    if (input.manipulated) {
        // Held by the PhysGun: the player is moving the body itself, so the
        // body is what they must see. Rendering the animation here made a
        // grab look like it was doing nothing while the bodies flew around.
        m_physicsBlend = 1.0f;
        m_physicsHoldSeconds = std::max(m_physicsHoldSeconds, 0.35f);
    }
    if (m_getUpPhase == CharacterGetUpPhase3D::Settling) {
        m_physicsBlend = 1.0f;
    } else if (m_getUpPhase == CharacterGetUpPhase3D::Rising) {
        // Hand the pose back to the clip over a short cross-fade so the
        // recovery starts from where the body actually is instead of popping
        // to the clip's first frame.
        m_physicsBlend = std::max(0.0f,
            1.0f - std::clamp(m_getUpClipSeconds / 0.30f, 0.0f, 1.0f));
    }
    m_reactionStrength = m_physicsBlend;
    const float requestedAuthority = std::clamp(
        input.poseAuthority, 0.0f, 1.0f);
    // At zero blend the guide holds the bodies exactly on the authored pose.
    // They still collide and still push props around; they simply have
    // nothing to contribute back, because the renderer is reading the
    // authored pose directly (see SpawnedRagdollInstance::animationPose).
    // Carried or free - never in between. A partial authority makes the
    // backend rewrite the articulation's positions AND velocities every
    // tick (applyCache), which injects energy on every frame: that is the
    // constant shiver, and it also wipes the friction the feet had just
    // accumulated, so the character slid like soap when something pushed
    // him. Handing the body over is a physical event, not a dial: while it
    // is carried nothing is written except the root, and the moment physics
    // takes over nothing is written at all.
    // Com a raiz livre, so o levantar (um clipe com trilha de raiz) ainda
    // carrega a pelve.
    // O levantar tambem nao carrega mais a pelve: e o corpo que levanta.
    const bool carried = !input.manipulated && !input.forceDrivenRoot
        && !getUpActive && m_physicsBlend < 0.35f;
    const float rootAuthority = carried ? requestedAuthority : 0.0f;

    Vec3 desiredPlanarVelocity { input.proxyVelocityWorld.x,
        input.proxyVelocityWorld.y, 0.0f };
    float gaitVelocityScale = 1.0f;
    // Depois de uma pancada ou desequilibrio, as pernas continuam no rumo
    // pedido, mas no passo do corpo: o que ele anda de verdade naquele rumo
    // mais uma folga. Na passada do comando (correndo a 7 m/s com o corpo
    // parado numa caixa, ou indo para tras), os dois pes ficavam no ar na
    // fase de voo e nenhum passo segurava o corpo. Livre de novo, o teto sobe
    // no ritmo de uma arrancada.
    if (input.forceDrivenRoot) {
        const float disturbed = std::max(
            std::clamp(input.externalPerturbation, 0.0f, 1.0f), m_fallReflex);
        const float wantedSpeed = std::hypot(desiredPlanarVelocity.x,
            desiredPlanarVelocity.y);
        float along = 0.0f;
        if (wantedSpeed > 0.05f) {
            along = dot(Vec3 { m_filteredCenterOfMassVelocity.x,
                m_filteredCenterOfMassVelocity.y, 0.0f },
                desiredPlanarVelocity * (1.0f / wantedSpeed));
        }
        const float target = disturbed > 0.2f && !getUpActive
            ? std::max(0.0f, along) + 1.2f : 1000.0f;
        m_gaitSpeedCap = target < m_gaitSpeedCap
            ? std::max(target, m_gaitSpeedCap - 30.0f * deltaTime)
            : std::min(target, m_gaitSpeedCap + 6.0f * deltaTime);
        // Sem limitar, o teto acompanha o pedido logo acima dele (e numa
        // pancada cai dali): uma arrancada normal nunca e contida.
        m_gaitSpeedCap = std::min(m_gaitSpeedCap, wantedSpeed + 1.0f);
        if (target > 999.0f && m_gaitSpeedCap >= wantedSpeed)
            m_gaitSpeedCap = wantedSpeed + 1.0f;
        if (wantedSpeed > m_gaitSpeedCap) {
            gaitVelocityScale = m_gaitSpeedCap / wantedSpeed;
            desiredPlanarVelocity = desiredPlanarVelocity * gaitVelocityScale;
        }
    }
    const Vec3 rawAcceleration = clampMagnitude(
        (desiredPlanarVelocity - m_previousDesiredVelocity) / deltaTime,
        24.0f);
    m_previousDesiredVelocity = desiredPlanarVelocity;
    const float accelerationBlend = 1.0f - std::exp(-9.0f * deltaTime);
    m_smoothedAcceleration +=
        (rawAcceleration - m_smoothedAcceleration) * accelerationBlend;

    // Sensor de equilibrio para os pes reagirem a empurrao, pressao e
    // tranco: o ponto de captura (centro de massa + velocidade/omega, onde o
    // corpo para se pisar ali) contra o apoio real (as solas no chao) e a
    // forca externa dos contatos. Parado, os passos de recuperacao (bloco
    // dos pes, mais abaixo) miram esse ponto; andando, o pouso do clipe e
    // deslocado para ele. Antes, parado, so havia o passo de acomodacao (o
    // pe 12 cm longe da pose, a pose presa a capsula que vem atras do
    // corpo): empurrado pelas costas a 0,8 m/s, os pes ficavam 0,8 s
    // travados e ele caia de frente. A passada dos clipes nao serve para
    // isso: todos sao de corrida (trote, recuo, strafe), com os dois pes no
    // ar a cada passada.
    float captureOutside = 0.0f;
    if (input.forceDrivenRoot && physical.valid) {
        // Apoio, ponto de captura e forca externa vem do estimador comum. Aqui
        // so a decisao de planejamento: com uma forca externa sustentada F o
        // ponto de equilibrio anda F/(m omega^2) = F h/(m g) no rumo dela
        // (150 N no peito: ~20 cm) - o passo sai antes e pousa onde segura.
        std::vector<std::array<float, 2>> support;
        float sumX = 0.0f, sumY = 0.0f;
        for (std::uint32_t i = 0; i < physical.supportPointCount; ++i) {
            const Vec3 p = physical.supportPointsWorld[i];
            support.push_back({ p.x, p.y });
            sumX += p.x;
            sumY += p.y;
        }
        Vec3 captureDirection {};
        if (!support.empty()) {
            const float omega = physical.captureOmega;
            const Vec3 forceShift = clampMagnitude(physical.externalForceWorld
                * (1.0f / (std::max(1.0f, profile.totalMassKg) * omega * omega)), 0.30f);
            // O planejamento usa a velocidade do centro de massa filtrada
            // (~60 ms): a crua do estimador reage ao tremor de cada tick e
            // disparava passos que tiravam a base (7 quedas na varredura de
            // 72 empurroes, contra 3).
            const float captureX = physical.centerOfMassWorld.x
                + m_filteredCenterOfMassVelocity.x / omega + forceShift.x;
            const float captureY = physical.centerOfMassWorld.y
                + m_filteredCenterOfMassVelocity.y / omega + forceShift.y;
            captureOutside = planarDistanceOutsideHull3D(support, captureX, captureY);
            const float count = static_cast<float>(support.size());
            captureDirection = { captureX - sumX / count, captureY - sumY / count, 0.0f };
            if (captureDirection.lengthSquared() > 1e-6f)
                captureDirection = captureDirection.normalized();
            m_capturePoint = { captureX, captureY, physical.supportHeight };
        }
        m_captureOutside = captureOutside;
        m_captureDirection = captureDirection;
    }
    m_telemetry.externalForceWorld = physical.externalForceWorld;
    m_telemetry.captureOutsideMeters = captureOutside;

    const float speed = std::hypot(desiredPlanarVelocity.x,
        desiredPlanarVelocity.y);
    const bool moving = speed > 0.08f;
    // Tropeco de marcha: um pe que deveria estar no meio do balanco toca sem
    // sustentar carga. Nao confundir com o toque normal (ultimos 25% do
    // passo) nem com o pe de apoio. A fase e a do tick anterior, suficiente
    // para classificar cedo/meio contra tarde sem antecipar o solver.
    bool trippedThisTick = false;
    if (input.forceDrivenRoot && moving && input.grounded && !getUpActive
        && speed > 1.5f) {
        for (std::size_t side = 0; side < m_feet.size(); ++side) {
            const float phase = m_telemetry.footSwingProgress[side];
            const FootSupportEstimate3D& seen = physical.feet[side];
            const float soleSpeed = std::hypot(seen.soleVelocityWorld.x,
                seen.soleVelocityWorld.y);
            if (!m_feet[side].planted && phase >= 0.12f && phase < 0.75f
                && seen.touching && !seen.supporting && soleSpeed > 0.55f) {
                m_tripRecovery = 1.0f;
                m_tripRecoverySeconds = 0.0f;
                m_tripFoot = static_cast<int>(side);
                m_tripElevating = phase < 0.58f;
                m_tripDirection = desiredPlanarVelocity / speed;
                m_fallDirection = m_tripDirection;
                trippedThisTick = true;
                break;
            }
        }
    }
    if (!trippedThisTick) {
        m_tripRecoverySeconds += deltaTime;
        // A decisao dura aproximadamente uma passada; a intensidade cai
        // devagar o bastante para o pe seguinte terminar a recuperacao.
        m_tripRecovery = std::max(0.0f,
            m_tripRecovery - deltaTime / 0.48f);
        if (m_tripRecovery <= 0.001f) m_tripFoot = -1;
    }
    const float viewYaw = input.facingYawRadians;
    // O angulo da camera chega em degraus: muda uma vez por quadro da tela, e
    // a fisica roda a 120 Hz. Para o olhar (cabeca, tronco, pelve girando no
    // lugar), uma mola criticamente amortecida o alisa - velocidade continua,
    // ~65 ms de atraso. Com o filtro de primeira ordem de antes, a velocidade
    // pedida a cabeca e ao tronco virava um dente de serra a cada quadro. A
    // escolha do setor do movimento continua com o angulo cru.
    if (!m_lookValid) {
        m_lookYaw = viewYaw;
        m_lookYawVelocity = 0.0f;
        m_lookPitch = std::clamp(input.lookPitchRadians, -0.60f, 0.50f);
        m_lookPitchVelocity = 0.0f;
        m_lookValid = true;
    }
    constexpr float LookOmega = 30.0f;
    m_lookYawVelocity += (LookOmega * LookOmega
            * wrapAngle(viewYaw - m_lookYaw)
        - 2.0f * LookOmega * m_lookYawVelocity) * deltaTime;
    m_lookYaw = wrapAngle(m_lookYaw + m_lookYawVelocity * deltaTime);
    m_lookPitchVelocity += (LookOmega * LookOmega
            * (std::clamp(input.lookPitchRadians, -0.60f, 0.50f) - m_lookPitch)
        - 2.0f * LookOmega * m_lookPitchVelocity) * deltaTime;
    m_lookPitch += m_lookPitchVelocity * deltaTime;
    const float lookYaw = m_lookYaw;
    // While fallen/getting up, pelvis yaw is meaningless — orientation is
    // physics', not this integrator's — so none of it runs, and the turn
    // step accumulator (a stationary-only concept) is kept at zero instead
    // of quietly building up a step to fire the instant recovery finishes.
    bool turningInPlace = false;
    if (!moving || getUpActive) {
        m_gaitDirection = CharacterGaitDirection3D::Forward;
    }
    if (!getUpActive) {
        const float viewErrorBeforeTurn = wrapAngle(lookYaw - m_facingYaw);
        float targetPelvisYaw = m_facingYaw;
        if (moving) {
            // Olhar x movimento. O personagem olha para onde a camera aponta
            // (viewYaw); o movimento cai num setor e o setor escolhe o ciclo:
            // para a frente corre, de lado faz o strafe (de frente para a
            // camera), para tras recua - nunca da meia-volta. Dentro do setor
            // a pelve gira para o movimento; o tronco e a cabeca devolvem o
            // giro para a camera (bloco do olhar, mais abaixo).
            // A direcao pedida (as teclas), quando ha: a da capsula gira aos
            // poucos e a pelve passava pela diagonal ao trocar de "frente"
            // para "lado" antes de assentar no strafe.
            const Vec3 intended { input.intent.requestedDirectionWorld.x,
                input.intent.requestedDirectionWorld.y, 0.0f };
            const float travelYaw = intended.lengthSquared() > 0.01f
                ? std::atan2(intended.y, intended.x)
                : std::atan2(desiredPlanarVelocity.y, desiredPlanarVelocity.x);
            const float travelFromView = wrapAngle(travelYaw - viewYaw);
            const bool strafe = animations.strafeLeft != nullptr
                && animations.strafeRight != nullptr;
            const float hysteresis = strafe ? StrafeSectorHysteresisRadians
                : PlainSectorHysteresisRadians;
            const auto available = [&](CharacterGaitDirection3D direction) {
                switch (direction) {
                case CharacterGaitDirection3D::Left:
                case CharacterGaitDirection3D::Right: return strafe;
                case CharacterGaitDirection3D::Backward:
                    return animations.walkBackward != nullptr;
                default: return true;
                }
            };
            const auto overrun = [&](CharacterGaitDirection3D direction) {
                return gaitSectorOverrun(direction, travelFromView, strafe);
            };
            // Freando forte (o jogador pediu o sentido oposto), o corpo freia
            // de frente para onde vinha e so troca de setor devagar; e, rapido,
            // nunca troca para o setor oposto (meia-volta da pelve): com a
            // camera virada 180 graus no sprint, a velocidade ainda para a
            // frente caia no setor "de costas" e a pelve girava na hora - ele
            // "corria de costas" a 7 m/s e caia. Ele freia de frente e vira
            // quando estiver devagar. So no modo fisico (no antigo a pose nao
            // carrega o corpo).
            const bool hardBraking = input.forceDrivenRoot && speed > 1.8f
                && dot(m_smoothedAcceleration, desiredPlanarVelocity) < -3.0f * speed;
            if (!hardBraking && (!available(m_gaitDirection)
                || overrun(m_gaitDirection) > hysteresis)) {
                // O setor que contem a direcao (sem recuo, o mais proximo).
                float best = std::numeric_limits<float>::infinity();
                CharacterGaitDirection3D chosen = m_gaitDirection;
                for (const CharacterGaitDirection3D candidate : {
                        CharacterGaitDirection3D::Forward,
                        CharacterGaitDirection3D::Left,
                        CharacterGaitDirection3D::Right,
                        CharacterGaitDirection3D::Backward }) {
                    if (available(candidate) && overrun(candidate) < best) {
                        best = overrun(candidate);
                        chosen = candidate;
                    }
                }
                const bool halfTurn = std::abs(wrapAngle(gaitSectorCenter(chosen)
                    - gaitSectorCenter(m_gaitDirection))) > 2.0f;
                if (!(input.forceDrivenRoot && halfTurn && speed > 2.5f
                        && available(m_gaitDirection))) {
                    m_gaitDirection = chosen;
                }
            }
            // O ciclo do setor anda na direcao do centro dele em relacao a
            // pelve: a pelve fica no movimento menos esse centro.
            targetPelvisYaw = wrapAngle(travelYaw
                - gaitSectorCenter(m_gaitDirection));
        } else if (std::abs(viewErrorBeforeTurn) > 0.55f
            || (std::abs(m_turningRate) > 0.05f
                && std::abs(viewErrorBeforeTurn) > 0.32f)) {
            // Olhar/coluna conservam cerca de 18 graus. O restante é
            // resolvido pela base com um passo de rotação, sem colar a
            // pelve na câmera. Depois de comecar, o giro vai ate os 18
            // graus: parar ao entrar nos 31 (a regra de antes) deixava o
            // alvo do giro mudando no meio dos passos.
            targetPelvisYaw = lookYaw - std::copysign(0.31f,
                viewErrorBeforeTurn);
        }
        const float pelvisYawError = wrapAngle(targetPelvisYaw - m_facingYaw);
        // Recem-levantado, o corpo primeiro firma a base: parado, nao gira
        // para a camera no primeiro meio segundo, e o giro entra aos poucos
        // ate ~1,5 s. Girando 100 graus logo na troca (taxa cheia), com um pe
        // ainda solto, ele inclinava 20 graus e caia de novo.
        const float footing = smoothStep((m_sinceGetUpSeconds - 0.5f) / 1.0f);
        // Parado, o giro e rapido (5 rad/s, 30 rad/s^2): a 2,35 rad/s e 10
        // rad/s^2 a pelve fazia ~25 graus no primeiro meio segundo, e o giro
        // parado parecia preso. Quem limita o que passa do apoio e o limite de
        // torcao do quadril (abaixo) e os passos de giro.
        const float maximumTurnRate = (input.sprinting ? 4.8f
            : moving ? 3.8f : 5.0f)
            * (moving ? 0.35f + 0.65f * footing : footing);
        const float wantedTurnRate = std::clamp(
            pelvisYawError * (moving ? 7.0f : 10.0f),
            -maximumTurnRate, maximumTurnRate);
        const float maximumTurnAcceleration = moving ? 18.0f : 30.0f;
        m_turningRate += std::clamp(wantedTurnRate - m_turningRate,
            -maximumTurnAcceleration * deltaTime,
            maximumTurnAcceleration * deltaTime);
        float yawStep = m_turningRate * deltaTime;
        if (std::abs(yawStep) > std::abs(pelvisYawError)
            && yawStep * pelvisYawError > 0.0f) {
            yawStep = pelvisYawError;
            m_turningRate = 0.0f;
        }
        m_facingYaw = wrapAngle(m_facingYaw + yawStep);
        // Parado, a pelve nao gira alem do que o quadril gira sobre um pe
        // apoiado (45 graus; 40 com folga): o giro vem dos pes. Sem isso a
        // pelve seguia a camera a ate 135 graus/s, e o pe apoiado ia junto,
        // esfregando no chao, ou a perna torcia no limite. So segura o que
        // passaria do limite neste tick; nunca empurra a pelve de volta.
        if (!moving && std::abs(yawStep) > 0.0f) {
            // A base em alerta ja traz cerca de 18 graus de rotacao entre
            // pelve e pes. Este limite e para o giro ADICIONAL que ainda
            // cabe sobre o apoio; 28 + 18 mantem a perna abaixo do gate
            // anatomico de 50 graus enquanto o outro pe contorna a base.
            constexpr float HipTwistLimit = 0.48f;
            for (std::size_t side = 0; side < m_feet.size(); ++side) {
                const FootPlant& foot = m_feet[side];
                if (!foot.valid || !foot.planted || foot.repositioning) {
                    continue;
                }
                const Vec3 forward = foot.lockedOrientationWorld.rotate(
                    profile.links[foot.linkIndex].modelOrientation
                        .conjugate().rotate({ 1.0f, 0.0f, 0.0f }));
                const float twist = wrapAngle(m_facingYaw + m_clipPelvisYaw
                    - std::atan2(forward.y, forward.x)
                    - m_clipPelvisToFootYaw[side]);
                if (std::abs(twist) > HipTwistLimit && twist * yawStep > 0.0f) {
                    const float excess = std::min(std::abs(yawStep),
                        std::abs(twist) - HipTwistLimit);
                    m_facingYaw = wrapAngle(m_facingYaw
                        - std::copysign(excess, twist));
                    // O pe bloqueou o deslocamento deste tick, mas o giro
                    // continua solicitado. Preservar a taxa mantem a
                    // histerese de giro ativa e permite que o planejador
                    // solte o proximo passo; zerar aqui abandonava uma
                    // camera lenta ainda com ~30 graus por resolver.
                }
            }
        }
        if (std::abs(pelvisYawError) < 0.001f
            && std::abs(m_turningRate) < 0.02f) {
            m_turningRate = 0.0f;
        }
        // Com histerese: sem ela, girando devagar o estado alternava entre
        // parado e girando no lugar a cada tick.
        turningInPlace = !moving && std::abs(m_turningRate)
            > (m_telemetry.state == CharacterLocomotionState3D::Turning
                ? 0.03f : 0.10f);
        m_standingTurnTarget = targetPelvisYaw;
    } else {
        m_turningRate = 0.0f;
        m_standingTurnTarget = m_facingYaw;
    }

    // Crouch and jump are deactivated: there is no authored clip for either
    // right now. Fallen/GettingUp preempt everything else — a real trip or
    // collision the reaction softening below could not absorb — so this
    // state machine reaches Idle, Turning, Walking, Running (a soccer
    // game's brisk jog and its sprint — see clip selection below),
    // Airborne, Landing, Fallen and GettingUp. Losing the ground (a fall, a
    // push, an edge) still needs a sane fallback pose regardless of
    // whether a jump was ever pressed, so Airborne/Landing stay
    // independent of the jump input itself.
    // Chao "de verdade" para os estados de voo. No modo fisico a verdade e o
    // corpo, nao a capsula: os dois decolam juntos, mas pousam em momentos
    // diferentes (medido: a capsula tocava o chao 0,28 s antes, e o corpo
    // chegava ao chao ainda com a pose de passada - pouso passivo). A
    // capsula no chao com o corpo ainda no ar segue em voo (pernas
    // procurando o chao); o corpo tocando o chao antes da capsula ja pousa.
    bool grounded = input.grounded;
    if (input.forceDrivenRoot && !getUpActive && !state.links.empty()) {
        const PhysicsBodyState3D& pelvis = state.links.front();
        float floor = std::numeric_limits<float>::infinity();
        for (const GroundProbeResult3D& probe : input.footGround) {
            if (probe.hasSurface) floor = std::min(floor, probe.pointWorld.z);
        }
        const bool feetDown = feetGroundContact[0] || feetGroundContact[1];
        const bool bodyHigh = !std::isfinite(floor) || pelvis.position.z - floor
            > profile.standingRootHeightMeters + 0.06f;
        const bool flying = m_telemetry.state == CharacterLocomotionState3D::Airborne;
        if (flying && grounded && !feetDown && bodyHigh) {
            grounded = false;
        } else if (!grounded && feetDown && pelvis.linearVelocity.z < 0.5f
            && (!flying || m_stateSeconds > 0.15f)) {
            // Pes no chao e o corpo sem subir: chao, em qualquer estado. Na
            // decolagem, o voo so comeca quando o corpo sobe de fato; no
            // pouso antes da capsula, ele segue pousando (antes voltava a
            // "voar" no tick seguinte, com a capsula ainda no ar).
            grounded = true;
        }
    }
    CharacterLocomotionState3D desiredState = CharacterLocomotionState3D::Idle;
    if (getUpActive) {
        desiredState = m_getUpPhase == CharacterGetUpPhase3D::Settling
            ? CharacterLocomotionState3D::Fallen
            : CharacterLocomotionState3D::GettingUp;
    } else if (!grounded) {
        desiredState = CharacterLocomotionState3D::Airborne;
    } else if ((!m_previousGrounded
        || (m_telemetry.state == CharacterLocomotionState3D::Landing
            && m_stateSeconds < m_landingSettleSeconds))
        // Pousando correndo, a passada continua: o estado de pouso trava os
        // dois pes onde tocaram, e a 7 m/s o corpo passava 1,7 m a frente
        // deles (35-50 graus de inclinacao). O amortecimento vale do mesmo
        // jeito (a pelve cede depois de qualquer toque, mais abaixo).
        && !(input.forceDrivenRoot && moving && speed > 2.5f)) {
        desiredState = CharacterLocomotionState3D::Landing;
    } else if (moving) {
        // Sprint para a frente e de lado; recuando, so com o sprint de
        // costas (sem ele, segurar sprint nao corre).
        desiredState = input.sprinting
                && (m_gaitDirection != CharacterGaitDirection3D::Backward
                    || animations.sprintBackward != nullptr)
            ? CharacterLocomotionState3D::Running
            : CharacterLocomotionState3D::Walking;
    } else if (turningInPlace) {
        desiredState = CharacterLocomotionState3D::Turning;
    }
    if (desiredState == CharacterLocomotionState3D::Airborne
        && m_telemetry.state != CharacterLocomotionState3D::Airborne) {
        m_liftoffSpeed = speed;
        m_liftoffVerticalSpeed = input.rootVelocityWorld.z;
        // O pulo e escolhido na decolagem e vale ate o fim da aterrissagem:
        // parado se quase nao havia movimento, senao o do setor em que ele
        // corria (frente, costas, lados).
        const auto pick = [&](CharacterJumpKind3D kind,
            const AnimationClip3D* clip) {
            if (clip == nullptr) return false;
            m_jumpKind = kind;
            m_jumpClip = clip;
            return true;
        };
        m_jumpKind = CharacterJumpKind3D::None;
        m_jumpClip = nullptr;
        bool picked = false;
        if (speed > 1.0f) {
            switch (m_gaitDirection) {
            case CharacterGaitDirection3D::Backward:
                picked = pick(CharacterJumpKind3D::Backward,
                    animations.jumpBackward);
                break;
            case CharacterGaitDirection3D::Left:
                picked = pick(CharacterJumpKind3D::Left, animations.jumpLeft);
                break;
            case CharacterGaitDirection3D::Right:
                picked = pick(CharacterJumpKind3D::Right, animations.jumpRight);
                break;
            default:
                picked = pick(CharacterJumpKind3D::Forward,
                    animations.jumpForward);
                break;
            }
        }
        if (!picked) {
            pick(CharacterJumpKind3D::Standing, animations.jumpStanding);
        }
    }
    // Fase do voo pela velocidade vertical: 0 na decolagem, 0,5 no apice, 1
    // ao voltar a altura de onde saiu. Vale para qualquer altura de pulo. Sem
    // impulso para cima (caiu de um degrau), o voo ja comeca descendo.
    if (desiredState == CharacterLocomotionState3D::Airborne) {
        const float verticalSpeed = input.forceDrivenRoot && !state.links.empty()
            ? state.links.front().linearVelocity.z : input.rootVelocityWorld.z;
        if (input.forceDrivenRoot) {
            // A subida do corpo e a que conta (ele ainda ganha velocidade
            // depois que a capsula decolou).
            m_liftoffVerticalSpeed = std::max(m_liftoffVerticalSpeed, verticalSpeed);
        }
        m_flightPhase = m_liftoffVerticalSpeed > 1.0f
            ? std::clamp((m_liftoffVerticalSpeed - verticalSpeed)
                / (2.0f * m_liftoffVerticalSpeed), 0.0f, 1.0f)
            : 0.5f + 0.5f * std::clamp(-verticalSpeed / 4.0f, 0.0f, 1.0f);
    } else if (desiredState != CharacterLocomotionState3D::Landing) {
        m_jumpKind = CharacterJumpKind3D::None;
        m_jumpClip = nullptr;
    }
    // Toque no chao depois de um voo (com ou sem o estado de pouso).
    const bool enteringLanding = m_telemetry.state
            == CharacterLocomotionState3D::Airborne
        && desiredState != CharacterLocomotionState3D::Airborne
        && !getUpActive;
    const CharacterLocomotionState3D previousState = m_telemetry.state;
    enterState(desiredState);
    const bool enteredForwardGait = (previousState == CharacterLocomotionState3D::Idle
            || previousState == CharacterLocomotionState3D::Turning)
        && (desiredState == CharacterLocomotionState3D::Walking
            || desiredState == CharacterLocomotionState3D::Running)
        && m_gaitDirection == CharacterGaitDirection3D::Forward
        && animations.idleToSprint != nullptr;
    if (enteredForwardGait) {
        m_startClipActive = true;
        m_startClipSprint = desiredState == CharacterLocomotionState3D::Running;
        m_startClipSeconds = 0.0f;
        m_cyclePhase = 0.0f;
    } else if (desiredState != CharacterLocomotionState3D::Walking
        && desiredState != CharacterLocomotionState3D::Running) {
        m_startClipActive = false;
    }
    // Pouso: a velocidade de queda do corpo no toque decide quanto ele
    // agacha e por quanto tempo (0,18 s num degrau; ~0,5 s caindo de um
    // pulo alto). Medida no voo, antes do toque (no tick do toque ela ja
    // caiu pela metade).
    if (desiredState == CharacterLocomotionState3D::Airborne
        && !state.links.empty()) {
        m_airborneFallSpeed = std::max(0.0f, input.forceDrivenRoot
            ? -state.links.front().linearVelocity.z
            : -input.rootVelocityWorld.z);
    }
    m_sinceTouchdownSeconds = enteringLanding ? 0.0f
        : std::min(10.0f, m_sinceTouchdownSeconds + deltaTime);
    if (enteringLanding) {
        m_landingImpactSpeed = m_airborneFallSpeed;
        m_landingSettleSeconds = input.forceDrivenRoot
            // (subida do agachamento em >= ~0,2 s: mais rapido, a ajuda de
            // altura ia ao maximo para acompanhar e o corpo quicava)
            ? 0.34f + 0.09f * std::clamp(m_landingImpactSpeed - 1.0f, 0.0f, 5.0f)
            : 0.18f;
    }
    m_previousGrounded = grounded;
    m_stateSeconds += deltaTime;
    if (m_startClipActive) {
        m_startClipSeconds += deltaTime;
        // A captura contem uma passada inteira de sprint, inclusive voo. No
        // trote usamos so a transferencia inicial; reproduzi-la inteira e
        // mais rapido deixava ambos os pes no ar por ~0,4 s.
        const float duration = m_startClipSprint ? 0.62f : 0.34f;
        if (animations.idleToSprint == nullptr
            || m_gaitDirection != CharacterGaitDirection3D::Forward
            || m_startClipSeconds >= std::min(
                duration, animations.idleToSprint->durationSeconds)) {
            m_startClipActive = false;
        }
    }
    m_transitionSeconds += deltaTime;
    m_idleSeconds += deltaTime;

    const Quaternion heading = Quaternion::fromAxisAngle(
        { 0.0f, 0.0f, 1.0f }, m_facingYaw);
    const Vec3 localVelocity = heading.conjugate().rotate(
        desiredPlanarVelocity);

    // Um ciclo por vez, sem mistura por direcao: a pelve ja se alinha com o
    // movimento (ou contra ele, recuando), entao o ciclo sempre anda para a
    // frente ou para tras em relacao a ela. Walking toca a caminhada rapida
    // (ou o recuo), Running o sprint quando existe - senao a caminhada.
    // Todo outro estado (giro parado, que nao tem clipe proprio, e
    // Airborne/Landing, procedurais - ver applyFlightPose) usa o idle.
    const bool gaitMoving = desiredState == CharacterLocomotionState3D::Walking
        || desiredState == CharacterLocomotionState3D::Running;
    // Arrancada fisica em tres tempos, medida na referencia Idle To Sprint:
    // comprime primeiro (quase sem deslocamento), transfere peito/COM sobre o
    // apoio e so entao entrega o ciclo cheio. O sprint usa a leitura inteira;
    // o trote conserva a mesma mecanica com metade da amplitude e menos tempo.
    Vec3 startupDirection = desiredPlanarVelocity;
    startupDirection.z = 0.0f;
    const float startupWantedSpeed = startupDirection.length();
    if (startupWantedSpeed > 0.001f) startupDirection *= 1.0f / startupWantedSpeed;
    const float startupBodySpeed = startupWantedSpeed > 0.001f
        ? std::max(0.0f, dot(Vec3 { m_filteredCenterOfMassVelocity.x,
            m_filteredCenterOfMassVelocity.y, 0.0f }, startupDirection)) : 0.0f;
    const bool gaitStarting = input.forceDrivenRoot && gaitMoving
        && m_stateSeconds < 0.65f && startupWantedSpeed > 1.0f
        && startupBodySpeed < 0.85f * startupWantedSpeed;
    const float startupTime = gaitStarting
        ? std::clamp(m_stateSeconds / 0.65f, 0.0f, 1.0f) : 1.0f;
    const float startupRise = smoothStep(startupTime / 0.20f);
    // O primeiro quadro da referencia ja traz ~4,4 graus de avanco. Partir
    // de zero enquanto a perna comeca o ciclo deixa a massa atras dos pes;
    // ha pre-carga imediata, e o restante cresce com a compressao.
    const float startupProgress = gaitStarting
        ? smoothStep(m_stateSeconds / 0.55f) : 1.0f;
    const bool sprintTier = desiredState == CharacterLocomotionState3D::Running
        && animations.sprint != nullptr;
    const bool forwardSprintStart = sprintTier
        && m_gaitDirection == CharacterGaitDirection3D::Forward;
    // Mantem a transferencia minima que ja sustentava o corpo no primeiro
    // quadro e cresce um pouco durante a compressao visual, sobretudo no
    // sprint. O envelope fisico termina quando o corpo atinge 85% do pedido.
    const float startupDrive = gaitStarting
        ? (1.0f - startupProgress) * (0.78f + 0.22f * startupRise) : 0.0f;
    const float startupCompression = gaitStarting
        ? smoothStep(startupTime / 0.16f)
            * (1.0f - smoothStep((startupTime - 0.56f) / 0.34f))
        : 0.0f;
    // A referencia chega perto de 38 graus porque raiz e coluna avancam
    // juntas. No ragdoll de jogo essa soma deslocaria o COM para fora do pe:
    // a leitura atletica fica principalmente na coluna, com a raiz menor.
    const float startupChestAmount =
        (forwardSprintStart ? 0.11f : 0.065f) * startupDrive;
    const float startupRootLean = (forwardSprintStart ? 0.075f : 0.045f)
        * (1.0f - startupProgress);
    const float startupPelvisDrop =
        (forwardSprintStart ? 0.025f : 0.010f) * startupCompression;
    const float gaitCycleSpeed = gaitStarting
        ? std::min(speed, std::max(1.0f, startupBodySpeed + 0.85f)) : speed;
    // Rising plays its own one-shot clip through the same path as every
    // other clip: sampled joint by joint, with the clip's own root track.
    const AnimationClip3D* standUpClip =
        m_getUpPhase == CharacterGetUpPhase3D::Rising
            ? (m_fallOrientation == CharacterFallOrientation3D::FaceDown
                ? animations.standUpFront : animations.standUpBack)
            : nullptr;
    const bool sprintingGait =
        desiredState == CharacterLocomotionState3D::Running;
    const AnimationClip3D* gaitClip = animations.walk;
    switch (m_gaitDirection) {
    case CharacterGaitDirection3D::Backward:
        gaitClip = sprintingGait && animations.sprintBackward != nullptr
            ? animations.sprintBackward : animations.walkBackward;
        break;
    case CharacterGaitDirection3D::Left:
        gaitClip = sprintingGait && animations.sprintStrafeLeft != nullptr
            ? animations.sprintStrafeLeft : animations.strafeLeft;
        break;
    case CharacterGaitDirection3D::Right:
        gaitClip = sprintingGait && animations.sprintStrafeRight != nullptr
            ? animations.sprintStrafeRight : animations.strafeRight;
        break;
    default:
        gaitClip = sprintTier ? animations.sprint : animations.walk;
        break;
    }
    const bool jumpPose = m_jumpClip != nullptr
        && (desiredState == CharacterLocomotionState3D::Airborne
            || desiredState == CharacterLocomotionState3D::Landing);
    const AnimationClip3D* idleClip = animations.idle;
    const bool authoredStartActive = m_startClipActive
        && animations.idleToSprint != nullptr && gaitMoving;
    // Curva pelo movimento real, inclusive de costas: v longitudinal com
    // sinal vezes a taxa de giro da pelve. Usar apenas |v| invertia o lado da
    // inclinacao ao correr para tras.
    const float signedCurveAcceleration = localVelocity.x * m_turningRate;
    const float curveSpeedShare = smoothStep((speed - 1.0f) / 2.5f);
    const float desiredCurveBlend = authoredStartActive ? 0.0f
        : std::clamp(smoothStep((std::abs(signedCurveAcceleration) - 0.35f)
            / 3.5f) * curveSpeedShare, 0.0f, 0.72f);
    int desiredCurveKind = 0;
    if (gaitMoving && desiredCurveBlend > 0.001f) {
        if (m_gaitDirection == CharacterGaitDirection3D::Forward) {
            if (signedCurveAcceleration > 0.0f) {
                desiredCurveKind = animations.runForwardArcLeft != nullptr ? 1 : 0;
            } else {
                desiredCurveKind = animations.runForwardArcRight != nullptr ? 2 : 0;
            }
        } else if (m_gaitDirection == CharacterGaitDirection3D::Backward
            && signedCurveAcceleration < 0.0f) {
            desiredCurveKind = animations.runBackwardArcRight != nullptr ? 3 : 0;
        }
    }
    // Nunca troca uma captura por outra com peso alto: primeiro sai da atual,
    // depois entra na nova. Isso evita um chicote de perna ao cruzar um setor
    // ou inverter a curva.
    const bool changingCurve = desiredCurveKind != m_authoredCurveKind;
    const float curveBlendTarget = changingCurve ? 0.0f : desiredCurveBlend;
    const float curveBlendRate = curveBlendTarget > m_authoredCurveBlend
        ? 2.5f : 4.0f;
    m_authoredCurveBlend += std::clamp(curveBlendTarget - m_authoredCurveBlend,
        -curveBlendRate * deltaTime, curveBlendRate * deltaTime);
    if (changingCurve && m_authoredCurveBlend <= 0.001f)
        m_authoredCurveKind = desiredCurveKind;
    const float authoredCurveBlend = m_authoredCurveBlend;
    const int authoredCurveKind = m_authoredCurveKind;
    const AnimationClip3D* authoredCurveClip = authoredCurveKind == 1
        ? animations.runForwardArcLeft : authoredCurveKind == 2
        ? animations.runForwardArcRight : authoredCurveKind == 3
        ? animations.runBackwardArcRight : nullptr;
    const AnimationClip3D* selectedClip = standUpClip != nullptr ? standUpClip
        : jumpPose ? m_jumpClip
        : !gaitMoving ? idleClip : gaitClip;
    // Trocar de ciclo sem trocar de estado (troca de setor, passada <->
    // sprint) precisa do mesmo crossfade que a troca de estado faz em
    // enterState. Entre setores ele e mais longo: o pe muda de trajetoria
    // enquanto a pelve vira ~90 graus, e em 0,14 s o alvo do pe chegava a
    // 15 m/s (medido na varredura de direcao, entre frente e recuo).
    if (selectedClip != m_previousClip) {
        if (m_previousClip != nullptr) {
            m_transitionFrom = m_poseCoordinates.size()
                    == m_coordinates.size()
                ? m_poseCoordinates : m_coordinates;
            m_transitionFromRootTilt = m_rootTilt;
            m_transitionVelocity = m_poseVelocities;
            m_transitionRootTiltVelocity = m_rootTiltVelocity;
            m_transitionRootOffsetVelocity = m_rootOffsetVelocity;
            m_transitionFromRootOffset = m_rootOffset;
            m_transitionSeconds = deltaTime;
            // Troca de setor: o ciclo novo anda em outra direcao em relacao a
            // pelve (frente, lado, costas).
            const bool gaitFlip = gaitMoving && selectedClip != nullptr
                && selectedClip != idleClip
                && m_previousClip != idleClip
                && std::abs(wrapAngle(selectedClip->travelDirectionRadians
                    - m_previousClip->travelDirectionRadians)) > 0.1f;
            m_transitionDurationSeconds = gaitFlip
                ? GaitFlipTransitionSeconds : DefaultTransitionSeconds;
        }
        m_previousClip = selectedClip;
    }

    // O ciclo anda com a capsula: ritmo dentro de uma faixa natural, e o
    // resto da velocidade vira passo mais curto ou mais longo (stride
    // warping, nos pes, mais abaixo). Com o ritmo sozinho travado em 0,35 o
    // pe andava mais que o corpo devagar, e o pe de apoio escorregava.
    float playbackRate = 1.0f;
    float strideScale = 1.0f;
    // Escada e ladeira: o chao a frente, na direcao do movimento. Quanto ele
    // sobe (ou desce) por metro limita o passo: cada passo sobe no maximo
    // StepRiseLimit (dois degraus), como quem sobe escada correndo. Na
    // passada inteira do trote (1,1 m) o pe pousava 4 degraus acima ou
    // abaixo - pernas contorcidas subindo, pe pendurado descendo.
    float terrainSlope = 0.0f;
    bool terrainIsRamp = false;
    float terrainDemand = 0.0f;
    const Vec3 planarTravel { input.rootVelocityWorld.x,
        input.rootVelocityWorld.y, 0.0f };
    if (input.groundAt && gaitMoving && input.grounded
        && planarTravel.lengthSquared() > 0.01f) {
        const Vec3 ahead = planarTravel.normalized();
        const float base = input.rootPositionWorld.z
            - profile.standingRootHeightMeters;
        // A 35 cm um unico espelho de degrau ja dava inclinacao de 0,5 a 1:
        // medir mais longe mede a escada, nao o degrau.
        // Correndo rapido olha mais longe: do sprint a 7 m/s, vista a 2,1 m,
        // a escada chegava antes de o corpo conseguir frear.
        std::array<float, 7> rise {};
        std::array<float, 7> at {};
        std::size_t count = 0;
        int tilted = 0;
        for (const float distance : { 0.70f, 1.05f, 1.40f, 1.75f, 2.10f, 2.80f, 3.50f }) {
            if (distance > 1.40f && (!input.forceDrivenRoot || speed < 4.0f)) continue;
            if (distance > 2.10f && speed < 5.0f) continue;
            Vec3 point = input.rootPositionWorld + ahead * distance;
            point.z = base;
            const GroundProbeResult3D probe = input.groundAt(point);
            if (!probe.hasSurface) continue;
            rise[count] = probe.pointWorld.z - base;
            at[count] = distance;
            ++count;
            if (probe.slopeDegrees > 4.0f && probe.slopeDegrees < 45.0f) ++tilted;
        }
        // So a subida (ou descida) que continua - ladeira, escada - pede
        // passo curto e corpo mais devagar: a altura cresce com a distancia,
        // sempre no mesmo sentido. Um degrau so (meio-fio, calcada: sobe e
        // fica plano) ou uma barra no caminho nao: o pe passa por cima dele
        // no passo normal (a folga do balanco ve o obstaculo). Tratado como
        // ladeira, um meio-fio de 12 cm levava o sprint de 7,3 a 4,5 m/s.
        for (std::size_t i = 0; i < count; ++i)
            if (std::abs(rise[i]) > 0.06f) terrainDemand = 1.0f;
        // No modo cinematico (a capsula carrega o corpo), a regra de antes:
        // qualquer desnivel a frente encurta o passo.
        if (!input.forceDrivenRoot)
            for (std::size_t i = 0; i < count; ++i)
                terrainSlope = std::max(terrainSlope, std::abs(rise[i]) / at[i]);
        // So a subida (ou descida) que continua - ladeira, escada -: a altura
        // nunca volta contra o sentido e sobe (3 cm ou mais) em pelo menos
        // dois trechos entre sondas (numa escada um trecho pode cair num
        // mesmo piso). Um degrau so (sobe e fica plano), uma barra no caminho
        // ou um degrau que aparece so na sonda mais longe nao contam. A
        // inclinacao e a do trecho do primeiro degrau ate a sonda mais longe.
        if (count >= 3 && input.forceDrivenRoot) {
            const float direction = rise[count - 1] - rise[0];
            int rising = 0;
            bool against = false;
            std::size_t first = count;
            for (std::size_t i = 1; i < count; ++i) {
                const float step = rise[i] - rise[i - 1];
                if (step * direction < 0.0f && std::abs(step) > 0.02f) against = true;
                if (std::abs(step) >= 0.03f && step * direction > 0.0f) {
                    ++rising;
                    if (first == count) first = i - 1;
                }
            }
            if (!against && rising >= 2) {
                terrainSlope = std::abs(rise[count - 1] - rise[first])
                    / std::max(0.1f, at[count - 1] - at[first]);
            } else if (!against && rising >= 1 && std::abs(rise[0]) >= 0.30f) {
                // No meio da subida (o chao a 0,7 m ja 30 cm acima ou abaixo):
                // o fim da escada a vista nao a desfaz - os ultimos degraus
                // ainda sao escada.
                terrainSlope = std::abs(rise[count - 1]) / std::max(0.1f, at[count - 1]);
            }
        }
        // Reconhecida a subida continua, por 0,6 s vale a medida de antes (o
        // maior desnivel sobre a distancia): em cima da escada a sequencia
        // subindo se desfaz entre degraus, o teto subia no meio dela e o
        // corpo tombava.
        if (input.forceDrivenRoot) {
            m_sustainedTerrainHold = terrainSlope > 0.08f ? 0.6f
                : std::max(0.0f, m_sustainedTerrainHold - deltaTime);
            if (m_sustainedTerrainHold > 0.0f)
                for (std::size_t i = 0; i < count; ++i)
                    terrainSlope = std::max(terrainSlope, std::abs(rise[i]) / at[i]);
        }
        // Rampa (a superficie sondada inclinada, sem espelhos): o passo pode
        // subir mais que numa escada.
        terrainIsRamp = tilted * 2 >= static_cast<int>(count) && count > 0;
    }
    // Quanto um passo sobe no maximo: numa escada, um ou dois espelhos; numa
    // rampa (sem espelho), mais - no passo de escada, uma rampa de 20 graus
    // levava o sprint a 39% da velocidade.
    const float StepRiseLimit = !input.forceDrivenRoot ? 0.34f
        : (terrainIsRamp || m_terrainRampHold > 0.0f) ? 0.32f : 0.20f;
    // (A rampa segura a classificacao por um tempo: entre duas sondas o
    // plano da rampa some.)
    if (terrainSlope > 0.08f) terrainDemand = 1.0f;
    m_terrainDemand = terrainDemand > m_terrainDemand ? terrainDemand
        : std::max(0.0f, m_terrainDemand - deltaTime / 0.6f);
    m_telemetry.terrainDemand = m_terrainDemand;
    m_terrainRampHold = terrainIsRamp ? 0.3f : std::max(0.0f, m_terrainRampHold - deltaTime);
    m_terrainIsRamp = m_terrainRampHold > 0.0f;
    // Sobe rapido (o degrau aparece de uma vez) e solta devagar.
    m_terrainSlope += (terrainSlope - m_terrainSlope) * (1.0f - std::exp(
        -(terrainSlope > m_terrainSlope ? 20.0f : 4.0f) * deltaTime));
    float terrainSpeedLimit = std::numeric_limits<float>::infinity();
    if (input.forceDrivenRoot && m_terrainSlope > 0.08f) {
        // Ladeira e escada: o corpo desacelera na proporcao da inclinacao
        // (10 graus ~90% da velocidade pedida, 20 graus ~70%, escada ~55%),
        // com um teto para escada ingreme. (Era um teto fixo de 0,48 /
        // inclinacao, ate 2,8 m/s: uma rampa de 10 graus levava o sprint de
        // 7,3 a 3,3 m/s.)
        // (Absoluto, sobre a velocidade de sprint: o jogo ja corta o pedido
        // por este teto, e calculado do pedido ele descia tick a tick.)
        constexpr float TerrainTopSpeed = 7.5f;
        const float share = std::clamp(1.0f - 1.1f * (m_terrainSlope - 0.08f), 0.55f, 1.0f);
        terrainSpeedLimit = TerrainTopSpeed * share;
        // Escada (subida continua sem ser rampa): cada degrau precisa de
        // tempo para o corpo subir o espelho - o teto de antes, continuo (so
        // acima de uma inclinacao ele chegava rapido demais e caia).
        if (!m_terrainIsRamp)
            terrainSpeedLimit = std::min(terrainSpeedLimit, std::max(0.85f, 0.48f / m_terrainSlope));
    }
    if (gaitMoving && selectedClip != nullptr) {
        const float authoredSpeed = authoredHorizontalSpeed(*selectedClip);
        if (authoredSpeed > 0.05f) {
            const float ratio = gaitCycleSpeed / authoredSpeed;
            // Passo do clipe (dois por ciclo) e o maior que o terreno deixa.
            const float authoredStep = authoredSpeed
                * selectedClip->durationSeconds * 0.5f;
            const float strideCap = m_terrainSlope > 0.05f
                ? std::clamp(StepRiseLimit / (m_terrainSlope * authoredStep),
                    0.25f, 1.0f)
                : 1.35f;
            playbackRate = std::clamp(ratio / std::min(1.0f, strideCap),
                0.55f, 1.45f);
            strideScale = std::clamp(ratio / playbackRate, 0.0f, strideCap);
            // Velocidade que cabe no terreno com o passo limitado e o ritmo
            // ate 15% acima do clipe; o jogo a usa como teto da capsula.
            if (strideCap < 1.0f) {
                terrainSpeedLimit = std::min(terrainSpeedLimit,
                    authoredSpeed * strideCap * 1.15f);
            }
            m_cyclePhase = std::fmod(m_cyclePhase + deltaTime * playbackRate
                / std::max(0.1f, selectedClip->durationSeconds), 1.0f);
            m_varietyStrides += deltaTime * playbackRate
                / std::max(0.1f, selectedClip->durationSeconds);
        }
    }

    // Saindo de escada ou rampa o teto sobe aos poucos (2 m/s2) enquanto o
    // corpo ainda esta nela; entrando, baixa na hora. As sondas veem o
    // patamar ~0,7 m antes e o teto saltava com o corpo ainda no ultimo
    // degrau: de 0,55 a 3 m/s em 0,35 s, ele tombava para a frente no topo
    // da escada. (Com o corpo ainda no plano - chegando numa rampa, a
    // estimativa oscilando - o teto acompanha na hora: devagar, ficava preso
    // no menor valor e o sprint chegava a rampa a 5,3 m/s em vez de 7.)
    {
        bool incline = false;
        float lowest = std::numeric_limits<float>::infinity();
        float highest = -lowest;
        int planted = 0;
        for (const FootPlant& plant : m_feet) {
            if (!plant.valid || !plant.planted) continue;
            ++planted;
            lowest = std::min(lowest, plant.groundHeight);
            highest = std::max(highest, plant.groundHeight);
            if (plant.groundNormal.z < 0.9976f) incline = true;  // ~4 graus
        }
        if (planted == 2 && highest - lowest > 0.05f) incline = true;
        m_onInclineHold = incline ? 0.6f : std::max(0.0f, m_onInclineHold - deltaTime);
        constexpr float TerrainCapCeiling = 9.0f, TerrainCapRecovery = 2.0f;
        if (input.forceDrivenRoot) {
            const float wanted = std::min(terrainSpeedLimit, TerrainCapCeiling);
            m_terrainSpeedCap = wanted < m_terrainSpeedCap || m_onInclineHold <= 0.0f ? wanted
                : std::min(wanted, m_terrainSpeedCap + TerrainCapRecovery * deltaTime);
            if (m_terrainSpeedCap < TerrainCapCeiling) terrainSpeedLimit = m_terrainSpeedCap;
        } else {
            m_terrainSpeedCap = TerrainCapCeiling;
        }
    }
    std::vector<Vec3> sampled(profile.links.size(), Vec3 {});
    Quaternion rootRotation;
    Vec3 rootTranslation;
    std::array<float, 2> contactWeights {};
    float clipSampleTime = 0.0f;
    // Variacao de movimento (vida), por cima do clipe e antes do crossfade:
    // a troca de estado a mistura como mistura o clipe.
    const float variety = std::clamp(input.motionVariety, 0.0f, 2.0f);
    const bool varietyActive = variety > 0.0f && selectedClip != nullptr
        && standUpClip == nullptr && !jumpPose && !getUpActive;
    // Parado, cada um respira o idle no seu ritmo e do seu ponto (sem
    // isso, varios personagens parados se mexiam em uníssono).
    const bool idleVariety = varietyActive && !gaitMoving
        && (desiredState == CharacterLocomotionState3D::Idle
            || desiredState == CharacterLocomotionState3D::Turning);
    if (selectedClip != nullptr) {
        const bool loopingState = desiredState == CharacterLocomotionState3D::Idle
            || desiredState == CharacterLocomotionState3D::Turning
            || desiredState == CharacterLocomotionState3D::Airborne;
        // O pulo e amostrado pela fase do voo; na aterrissagem fica na pose
        // de toque (o fim do clipe).
        const float sampleTime = standUpClip != nullptr
            ? std::min(m_getUpClipSeconds, selectedClip->durationSeconds)
            : jumpPose
            ? (desiredState == CharacterLocomotionState3D::Landing ? 1.0f
                : m_flightPhase) * selectedClip->durationSeconds
            : gaitMoving
            ? stanceAlignedPhase(*selectedClip, m_cyclePhase)
                * selectedClip->durationSeconds
            : loopingState && idleVariety
                ? std::fmod(m_stateSeconds * (1.0f + 0.08f * varietyValue(
                    input.varietySeed, 5, 0)) + selectedClip->durationSeconds
                    * (0.5f + 0.5f * varietyValue(input.varietySeed, 6, 0)),
                    selectedClip->durationSeconds)
            : loopingState
                ? std::fmod(m_stateSeconds, selectedClip->durationSeconds)
                : std::min(m_stateSeconds, selectedClip->durationSeconds);
        clipSampleTime = sampleTime;
        for (std::size_t index = 1; index < profile.links.size(); ++index) {
            const AnimationTrack3D* track = findAnimationTrack3D(
                *selectedClip, profile.links[index].id);
            if (track == nullptr) continue;
            sampled[index] = sampleAnimationTrack3D(*track, sampleTime,
                selectedClip->durationSeconds, selectedClip->loops)
                    .jointPositionRadians;
        }
        const AnimationTrack3D* rootTrack = findAnimationTrack3D(
            *selectedClip, profile.links.front().id);
        if (rootTrack != nullptr) {
            const AnimationTransformSample3D rootSample =
                sampleAnimationTrack3D(*rootTrack, sampleTime,
                    selectedClip->durationSeconds, selectedClip->loops);
            rootTranslation = rootSample.translationOffsetMeters;
            rootRotation = rootSample.rotationDelta;
        }
        contactWeights[0] = sampleAnimationScalarTrack3D(
            selectedClip->leftFootContact, sampleTime,
            selectedClip->durationSeconds, selectedClip->loops);
        contactWeights[1] = sampleAnimationScalarTrack3D(
            selectedClip->rightFootContact, sampleTime,
            selectedClip->durationSeconds, selectedClip->loops);
    }
    // A arrancada capturada participa como DELTA de gesto, por cima do ciclo
    // de trote/sprint que continua dono das pernas, dos contatos e da raiz.
    // Copiar a pose absoluta do FBX trazia para o guia 33 graus de root pitch,
    // 10 cm de agachamento e uma fase de voo antes de o corpo acelerar.
    if (authoredStartActive && animations.idleToSprint != nullptr) {
        const float duration = m_startClipSprint ? 0.62f : 0.34f;
        const float u = std::clamp(m_startClipSeconds / duration, 0.0f, 1.0f);
        const float envelope = smoothStep(u / 0.20f)
            * (1.0f - smoothStep((u - 0.62f) / 0.38f));
        const float sourceTime = std::min(
            m_startClipSeconds * (m_startClipSprint ? 1.0f : 0.75f),
            animations.idleToSprint->durationSeconds);
        const auto segmentWeight = [&](const std::string& id) {
            const bool spine = id == "Abdomen" || id == "Chest"
                || id == "UpperChest";
            const bool arm = id == "LeftUpperArm" || id == "RightUpperArm"
                || id == "LeftForearm" || id == "RightForearm";
            const bool head = id == "Neck" || id == "Head";
            if (spine) return m_startClipSprint ? 0.55f : 0.28f;
            if (arm) return m_startClipSprint ? 0.62f : 0.30f;
            if (head) return m_startClipSprint ? 0.25f : 0.12f;
            return 0.0f;
        };
        for (std::size_t index = 1; index < profile.links.size(); ++index) {
            const float weight = segmentWeight(profile.links[index].id)
                * envelope;
            if (weight <= 0.0f) continue;
            const AnimationTrack3D* track = findAnimationTrack3D(
                *animations.idleToSprint, profile.links[index].id);
            if (track == nullptr) continue;
            const Vec3 first = sampleAnimationTrack3D(*track, 0.0f,
                animations.idleToSprint->durationSeconds, false)
                    .jointPositionRadians;
            const Vec3 current = sampleAnimationTrack3D(*track, sourceTime,
                animations.idleToSprint->durationSeconds, false)
                    .jointPositionRadians;
            const float limit = m_startClipSprint ? 0.42f : 0.20f;
            const Vec3 delta {
                std::clamp(current.x - first.x, -limit, limit),
                std::clamp(current.y - first.y, -limit, limit),
                std::clamp(current.z - first.z, -limit, limit)
            };
            sampled[index] += delta * weight;
        }
    }
    // A curva real entra apenas nos segmentos que definem apoio e banco do
    // corpo. Cabeca e bracos ficam no ciclo normal; isso tambem exclui
    // explicitamente os canais defeituosos da referencia Arc Left.
    if (authoredCurveClip != nullptr && authoredCurveBlend > 0.0f) {
        const float curveTime = stanceAlignedPhase(*authoredCurveClip,
            m_cyclePhase) * authoredCurveClip->durationSeconds;
        const auto curveSegment = [](const std::string& id) {
            return id == "Abdomen" || id == "Chest" || id == "UpperChest"
                || id == "LeftThigh" || id == "LeftShin" || id == "LeftFoot"
                || id == "RightThigh" || id == "RightShin" || id == "RightFoot";
        };
        for (std::size_t index = 1; index < profile.links.size(); ++index) {
            if (!curveSegment(profile.links[index].id)) continue;
            const AnimationTrack3D* track = findAnimationTrack3D(
                *authoredCurveClip, profile.links[index].id);
            if (track == nullptr) continue;
            const Vec3 curvePose = sampleAnimationTrack3D(*track, curveTime,
                authoredCurveClip->durationSeconds, false).jointPositionRadians;
            const bool spine = profile.links[index].id == "Abdomen"
                || profile.links[index].id == "Chest"
                || profile.links[index].id == "UpperChest";
            // A coluna pode ler a captura com peso alto. Nas pernas ela e
            // uma referencia de estilo menor: o alvo de pouso em arco e o
            // contato fisico continuam soberanos, evitando chicotear o pe ao
            // sincronizar dois ciclos gravados em fases diferentes.
            const float weight = authoredCurveBlend * (spine ? 1.0f : 0.15f);
            Vec3 wanted = curvePose;
            if (spine) {
                // A captura da curva esquerda traz ~30 graus de corcunda como
                // baseline. Curva deve fornecer banco/torcao; a flexao
                // sagital continua sendo a do trote ou sprint selecionado.
                wanted.y = sampled[index].y;
            }
            sampled[index] += (wanted - sampled[index]) * weight;
        }
    }
    if (varietyActive && gaitMoving) {
        applyGaitVariety(profile, *selectedClip, m_cyclePhase,
            m_varietyStrides, input.varietySeed, variety, sampled);
    } else if (idleVariety) {
        applyIdleVariety(profile, m_elapsedSeconds, input.varietySeed,
            variety, sampled);
    }
    // Fim do levantar: coluna, pescoco e bracos vao aos poucos para a
    // postura parada (o primeiro quadro do idle, que e onde ele comeca). O
    // levantar de bruços termina ereto demais (tronco a ~2 graus, pes
    // juntos) e o idle tem o tronco a ~25 graus para a frente: depois da
    // troca o corpo ficava torto "encaixando" a diferenca. As pernas seguem
    // o clipe (os pes ficam onde estao e os passos refazem a base).
    if (standUpClip != nullptr && animations.idle != nullptr) {
        const float duration = standUpClip->durationSeconds > 0.05f
            ? standUpClip->durationSeconds : 1.6f;
        const float toStance = smoothStep(
            (m_getUpClipSeconds / duration - 0.60f) / 0.40f);
        if (toStance > 0.0f) {
            for (std::size_t index = 1; index < profile.links.size()
                    && index < sampled.size(); ++index) {
                const std::string& id = profile.links[index].id;
                if (id.find("Thigh") != std::string::npos
                    || id.find("Shin") != std::string::npos
                    || id.find("Foot") != std::string::npos) continue;
                const AnimationTrack3D* track =
                    findAnimationTrack3D(*animations.idle, id);
                if (track == nullptr) continue;
                const Vec3 stance = sampleAnimationTrack3D(*track, 0.0f,
                    animations.idle->durationSeconds, animations.idle->loops)
                        .jointPositionRadians;
                sampled[index] += (stance - sampled[index]) * toStance;
            }
        }
    }

    // Quanto a pelve desce no pouso (modo antigo). No modo fisico o
    // agachamento vai direto na pelve alvo, mais abaixo.
    const auto landingSink = [&](float settle) {
        return input.forceDrivenRoot ? 0.0f : 0.08f * std::sin(Pi * settle);
    };
    float landingBraceForward = 0.0f;
    float landingBraceLateral = 0.0f;
    // Pulo autoral: o voo vem do clipe (acima). Na aterrissagem os pes sao
    // plantados pela trava de pe (contato forcado) e a pelve desce: e o IK
    // que dobra os joelhos, absorvendo, em vez de posicoes fixas de junta.
    // Sem clipes de pulo, o voo e a pose procedural de antes.
    if (jumpPose && desiredState == CharacterLocomotionState3D::Landing) {
        const float settle = std::clamp(
            m_stateSeconds / m_landingSettleSeconds, 0.0f, 1.0f);
        rootTranslation.z -= landingSink(settle);
        contactWeights = { 1.0f, 1.0f };
    } else if (jumpPose) {
        // Voo: nada a acrescentar.
    } else if (desiredState == CharacterLocomotionState3D::Airborne) {
        const float verticalVelocity = input.rootVelocityWorld.z;
        const float riseTuck = smoothStep(m_stateSeconds / 0.14f);
        // As descent speed builds, ease the tuck back open to meet the
        // ground with the legs already reaching for it.
        const float fallRelease = std::clamp(1.0f
            - std::max(0.0f, -verticalVelocity) * 0.10f, 0.30f, 1.0f);
        const float runningShare = smoothStep(m_liftoffSpeed / 2.2f);
        applyFlightPose(profile, sampled, riseTuck * fallRelease,
            runningShare);
    } else if (desiredState == CharacterLocomotionState3D::Landing) {
        const float settle = std::clamp(
            m_stateSeconds / m_landingSettleSeconds, 0.0f, 1.0f);
        const float compression = (1.0f - smoothStep(settle)) * 0.55f;
        const float runningShare = smoothStep(m_liftoffSpeed / 2.2f);
        applyFlightPose(profile, sampled, compression, runningShare);
    } else if (m_getUpPhase == CharacterGetUpPhase3D::Settling) {
        // Caido, em tres tempos. Primeiro ragdoll de verdade: o alvo das
        // juntas segue o proprio corpo com atraso (~0,2 s) e tonus baixo - ele
        // cede para onde a queda o leva, mas os musculos amortecem (nao e
        // corpo morto). Parado de vez, fica largado um instante. Depois, com o
        // tonus voltando, se arruma aos poucos ate a pose inicial do levantar
        // certo (de costas ou de bruços), de onde o levantar comeca sem salto.
        // Antes a pose de levantar era o alvo desde o primeiro instante: ele
        // caia ja "arrumado" para levantar.
        const AnimationClip3D* lying = m_fallOrientation
                == CharacterFallOrientation3D::FaceDown
            ? animations.standUpFront : animations.standUpBack;
        if (!m_settlePoseValid || m_settlePose.size() != sampled.size()) {
            m_settlePose.assign(sampled.size(), Vec3 {});
            m_settleGather = 0.0f;
            for (std::size_t link = 1;
                    link < sampled.size() && link < state.joints.size(); ++link) {
                m_settlePose[link] = { state.joints[link].positionRadians[0],
                    state.joints[link].positionRadians[1],
                    state.joints[link].positionRadians[2] };
            }
            m_settlePoseValid = true;
        }
        const bool gathering = m_settleRestSeconds > (m_getUpAttempt > 1
            ? RetryRestBeforeGather : SettleRestBeforeGather);
        // Atraso curto: o alvo que persegue o corpo atua como freio (mola x
        // atraso); com 0,2 s um antebraco em pe caia a ~0,1 rad/s.
        const float follow = gathering ? 0.0f
            : 1.0f - std::exp(-deltaTime / 0.04f);
        // Arrumando-se, acompanha o tonus; interrompido (um tranco no
        // tronco), volta a ceder em ~0,15 s, sem salto no alvo.
        const float gatherTarget = gathering ? smoothStep((m_settleTone
            - SettleRestingTone) / (1.0f - SettleRestingTone)) : 0.0f;
        m_settleGather = gatherTarget >= m_settleGather ? gatherTarget
            : m_settleGather + (gatherTarget - m_settleGather)
                * (1.0f - std::exp(-deltaTime / 0.15f));
        const float gather = m_settleGather;
        for (std::size_t link = 1;
                link < sampled.size() && link < state.joints.size(); ++link) {
            const Vec3 measured { state.joints[link].positionRadians[0],
                state.joints[link].positionRadians[1],
                state.joints[link].positionRadians[2] };
            m_settlePose[link] += (measured - m_settlePose[link]) * follow;
            const AnimationTrack3D* track = lying != nullptr
                ? findAnimationTrack3D(*lying, profile.links[link].id) : nullptr;
            const Vec3 start = track != nullptr
                ? sampleAnimationTrack3D(*track, 0.0f, lying->durationSeconds,
                    false).jointPositionRadians
                : measured;
            sampled[link] = m_settlePose[link]
                + (start - m_settlePose[link]) * gather;
        }
    }

    // Preparacao de impacto pela velocidade real. Durante a descida o peito
    // vai contra o movimento horizontal: aterrissando para a frente, fica
    // atras da pelve; para a esquerda, desloca-se para a direita. Assim a
    // resultante do contato atravessa melhor a base e os joelhos podem
    // comprimir sem o tronco continuar tombando por inercia.
    if (!getUpActive && (desiredState == CharacterLocomotionState3D::Airborne
            || desiredState == CharacterLocomotionState3D::Landing)) {
        const Vec3 bodyVelocity = input.forceDrivenRoot && !state.links.empty()
            ? state.links.front().linearVelocity : input.rootVelocityWorld;
        const Vec3 localImpact = heading.conjugate().rotate(
            {bodyVelocity.x, bodyVelocity.y, 0.0f});
        const float horizontalSpeed = std::hypot(localImpact.x, localImpact.y);
        const float fallSpeed = desiredState == CharacterLocomotionState3D::Airborne
            ? std::max(0.0f, -bodyVelocity.z) : m_landingImpactSpeed;
        const float descent = desiredState == CharacterLocomotionState3D::Airborne
            ? smoothStep((m_flightPhase - 0.48f) / 0.34f)
                * smoothStep((fallSpeed - 0.35f) / 1.8f)
            : 1.0f - smoothStep(m_stateSeconds
                / std::max(0.08f, m_landingSettleSeconds));
        const float brace = std::clamp(horizontalSpeed
            * (0.025f + 0.007f * std::min(fallSpeed, 5.0f)), 0.0f, 0.30f)
            * descent;
        if (horizontalSpeed > 0.05f) {
            landingBraceForward = -brace * localImpact.x / horizontalSpeed;
            landingBraceLateral = -brace * localImpact.y / horizontalSpeed;
            addJointCoordinate(profile, sampled, "Abdomen", 1,
                landingBraceForward * 0.30f);
            addJointCoordinate(profile, sampled, "Chest", 1,
                landingBraceForward * 0.35f);
            addJointCoordinate(profile, sampled, "UpperChest", 1,
                landingBraceForward * 0.35f);
            addJointCoordinate(profile, sampled, "Abdomen", 2,
                landingBraceLateral * 0.30f);
            addJointCoordinate(profile, sampled, "Chest", 2,
                landingBraceLateral * 0.35f);
            addJointCoordinate(profile, sampled, "UpperChest", 2,
                landingBraceLateral * 0.35f);
        }
    }

    const float transition = smoothStep(
        m_transitionSeconds / m_transitionDurationSeconds);
    const float transitionTime = std::clamp(
        m_transitionSeconds / m_transitionDurationSeconds, 0.0f, 1.0f);
    // Hermite outgoing tangent: preserve the motion at interruption and
    // release it continuously as the new clip takes over. Only authored
    // coordinates participate; IK and look must not feed back into it.
    const float tangentWeight = m_transitionDurationSeconds * transitionTime
        * (1.0f - transitionTime) * (1.0f - transitionTime);
    // Levantar a partir de onde o corpo esta: nos primeiros 0,35 s os alvos
    // saem das juntas medidas ao comecar, nao de onde a pose de se arrumar
    // queria que estivessem.
    const float risingFromMeasured = m_getUpPhase == CharacterGetUpPhase3D::Rising
            && m_risingFromMeasured.size() == profile.links.size()
        ? 1.0f - smoothStep(m_getUpPhaseSeconds / 0.35f) : 0.0f;
    for (std::size_t index = 1; index < profile.links.size(); ++index) {
        Vec3 wanted = m_transitionFrom[index] * (1.0f - transition)
            + sampled[index] * transition
            + m_transitionVelocity[index] * tangentWeight;
        if (risingFromMeasured > 0.0f)
            wanted += (m_risingFromMeasured[index] - wanted) * risingFromMeasured;
        for (std::size_t axis = 0; axis < 3; ++axis) {
            const RagdollAxisDefinition3D& limit =
                profile.links[index].inboundJoint.axes[axis];
            setComponent(wanted, axis, limit.enabled
                ? std::clamp(component(wanted, axis),
                    limit.minimumRadians, limit.maximumRadians)
                : 0.0f);
        }
        m_coordinates[index] = wanted;
        m_poseVelocities[index] = m_poseCoordinates.size() == m_coordinates.size()
            ? clampMagnitude((wanted - m_poseCoordinates[index]) / deltaTime, 30.0f)
            : Vec3 {};
    }
    m_poseCoordinates = m_coordinates;

    // O olhar. A pelve pode estar a ate ~105 graus da camera (andando de
    // lado, ou na borda do recuo); o personagem continua olhando para ela.
    // A coluna devolve a maior parte, ate 45 graus, e a cabeca completa, ate
    // 75 - o tronco fica perto da direcao do olhar e a cabeca nela. A
    // cabeca acompanha a camera rapido; o tronco vem atras num ritmo
    // proprio, mais lento, e um giro brusco de camera le como "a cabeca
    // olha, o corpo vem junto", nao como a coluna inteira girando de uma
    // vez no mesmo quadro.
    const float viewYawError = std::clamp(wrapAngle(lookYaw - m_facingYaw),
        -(TorsoTwistLimitRadians + HeadTurnLimitRadians),
        TorsoTwistLimitRadians + HeadTurnLimitRadians);
    const float headLookBlend = 1.0f - std::exp(-15.0f * deltaTime);
    const float torsoLookBlend = 1.0f - std::exp(-4.2f * deltaTime);
    const float torsoLimit = desiredState == CharacterLocomotionState3D::Running
        ? SprintTorsoTwistLimitRadians : TorsoTwistLimitRadians;
    const float torsoTarget = std::clamp(viewYawError * TorsoLookShare,
        -torsoLimit, torsoLimit);
    m_torsoTwist += (torsoTarget - m_torsoTwist) * torsoLookBlend;
    const float headTarget = std::clamp(viewYawError - m_torsoTwist,
        -HeadTurnLimitRadians, HeadTurnLimitRadians);
    m_headTurn += (headTarget - m_headTurn) * headLookBlend;
    m_filteredLookPitch = m_lookPitch;
    // Filters keep running so they are not stale on handoff, but the
    // look-at twist itself only applies to the normal locomotion pose —
    // piling it on top of the get-up sequence's own carefully phased
    // spine/neck targets would just fight them.
    if (!getUpActive) {
        // Twist + gira a frente do segmento para a esquerda, na coluna e no
        // pescoco (medido pela FK em tools/animation). Divisao pelos
        // limites de cada junta: coluna 30/35/35, pescoco 40 e cabeca 60.
        addJointCoordinate(profile, m_coordinates, "Abdomen", 0,
            m_torsoTwist * 0.30f);
        addJointCoordinate(profile, m_coordinates, "Chest", 0,
            m_torsoTwist * 0.35f);
        addJointCoordinate(profile, m_coordinates, "UpperChest", 0,
            m_torsoTwist * 0.35f);
        addJointCoordinate(profile, m_coordinates, "Neck", 0,
            m_headTurn * 0.40f);
        addJointCoordinate(profile, m_coordinates, "Head", 0,
            m_headTurn * 0.60f);
        addJointCoordinate(profile, m_coordinates, "UpperChest", 1,
            -m_filteredLookPitch * 0.14f);
        addJointCoordinate(profile, m_coordinates, "Neck", 1,
            -m_filteredLookPitch * 0.36f);
        addJointCoordinate(profile, m_coordinates, "Head", 1,
            -m_filteredLookPitch * 0.56f);
        // A coluna reage a pelve FISICA, nao apenas a inclinacao ideal da
        // animacao. Com ajuda parcial a pelve pode adiantar/tombar muito mais
        // que a referencia; a regra antiga via no maximo os poucos graus
        // pedidos pela arrancada e o tronco continuava soldado a ela.
        //
        // Esta e uma tarefa articular amortecida: a pelve continua livre e a
        // coluna devolve parte do erro para manter o peito proximo da vertical,
        // sem virar uma barra rigida nem apagar toda a inclinacao da corrida.
        Vec3 spineTarget {};
        if (input.forceDrivenRoot && !state.links.empty()) {
            const Vec3 pelvisUp = state.links.front().orientation.rotate({ 0.0f, 0.0f, 1.0f });
            const Vec3 headingForward = heading.rotate({ 1.0f, 0.0f, 0.0f });
            const Vec3 headingLeft = heading.rotate({ 0.0f, 1.0f, 0.0f });
            const float pelvisForward = std::asin(std::clamp(
                dot(pelvisUp, headingForward), -1.0f, 1.0f));
            const float pelvisSide = std::asin(std::clamp(
                dot(pelvisUp, headingLeft), -1.0f, 1.0f));
            spineTarget.x = std::clamp(-0.62f * pelvisForward, -0.24f, 0.24f)
                * startupProgress;
            spineTarget.y = std::clamp(-0.48f * pelvisSide, -0.20f, 0.20f);
        }
        const Vec3 spineError = m_spineBalance - spineTarget;
        const Vec3 spineImpulse = m_spineBalanceVelocity + spineError * 12.0f;
        const float spineDecay = std::exp(-12.0f * deltaTime);
        m_spineBalance = spineTarget
            + (spineError + spineImpulse * deltaTime) * spineDecay;
        m_spineBalanceVelocity = (m_spineBalanceVelocity
            - spineImpulse * (12.0f * deltaTime)) * spineDecay;
        const float spineForward = m_spineBalance.x;
        const float spineSide = m_spineBalance.y;
        addJointCoordinate(profile, m_coordinates, "Abdomen", 1, spineForward * 0.30f);
        addJointCoordinate(profile, m_coordinates, "Chest", 1, spineForward * 0.35f);
        addJointCoordinate(profile, m_coordinates, "UpperChest", 1, spineForward * 0.35f);
        addJointCoordinate(profile, m_coordinates, "Abdomen", 2, spineSide * 0.30f);
        addJointCoordinate(profile, m_coordinates, "Chest", 2, spineSide * 0.35f);
        addJointCoordinate(profile, m_coordinates, "UpperChest", 2, spineSide * 0.35f);
        // Na arrancada o peito acompanha a massa para a frente; a
        // contra-inclinacao que estabiliza a marcha entra conforme o corpo
        // ganha velocidade. No tropeco a inercia tambem continua levando o
        // tronco para a frente, como na referencia estudada: o passo longo e
        // a pelve baixa e que o capturam, nao uma extensao instantanea.
        const float startupChest = startupChestAmount;
        const Vec3 tripLocal = heading.conjugate().rotate(m_tripDirection);
        const float tripForward = 0.12f * m_tripRecovery * tripLocal.x;
        const float tripSide = -0.10f * m_tripRecovery * tripLocal.y;
        addJointCoordinate(profile, m_coordinates, "Abdomen", 1,
            (startupChest + tripForward) * 0.30f);
        addJointCoordinate(profile, m_coordinates, "Chest", 1,
            (startupChest + tripForward) * 0.35f);
        addJointCoordinate(profile, m_coordinates, "UpperChest", 1,
            (startupChest + tripForward) * 0.35f);
        addJointCoordinate(profile, m_coordinates, "Abdomen", 2, tripSide * 0.30f);
        addJointCoordinate(profile, m_coordinates, "Chest", 2, tripSide * 0.35f);
        addJointCoordinate(profile, m_coordinates, "UpperChest", 2, tripSide * 0.35f);
        if (m_tripFoot >= 0 && m_tripRecovery > 0.0f) {
            const char* thigh = m_tripFoot == 0 ? "LeftThigh" : "RightThigh";
            const char* shin = m_tripFoot == 0 ? "LeftShin" : "RightShin";
            const float elevation = m_tripElevating ? 1.0f : 0.45f;
            addJointCoordinate(profile, m_coordinates, thigh, 1,
                -0.24f * m_tripRecovery * elevation);
            addJointCoordinate(profile, m_coordinates, shin, 0,
                0.55f * m_tripRecovery * elevation);
        }
    }

    // A freada especial e uma recuperacao de emergencia, nao a animacao
    // normal de parar. Ela so aparece quando ainda existe momento fisico de
    // corrida maxima E a intencao desacelera forte. No trote e em trajetos
    // curtos, o ciclo apenas encurta os passos e assenta: projetar um pe
    // muito a frente nessas condicoes parece uma derrapagem sem causa.
    //
    // A reacao continua sendo uma mola de pose e nao segura a capsula. Vale
    // em qualquer direcao: o sentido vem do movimento de entrada.
    {
        if (speed > 0.3f) m_brakeDirection = desiredPlanarVelocity / speed;
        m_brakeEntrySpeed = std::max(speed,
            m_brakeEntrySpeed - 2.5f * deltaTime);
        const bool braceable = !getUpActive && input.grounded
            && desiredState != CharacterLocomotionState3D::Airborne
            && desiredState != CharacterLocomotionState3D::JumpStarting;
        const float deceleration = std::max(0.0f,
            -dot(m_smoothedAcceleration, m_brakeDirection));
        const Vec3 physicalPlanarVelocity { m_filteredCenterOfMassVelocity.x,
            m_filteredCenterOfMassVelocity.y, 0.0f };
        const float physicalMomentum = std::max(0.0f,
            dot(physicalPlanarVelocity, m_brakeDirection));
        // Comeca apenas perto da corrida maxima. Os dois portoes evitam que
        // uma oscilacao de velocidade do trote seja confundida com freada.
        const float sprintMomentum = smoothStep(
            (std::min(m_brakeEntrySpeed, physicalMomentum) - 5.5f) / 1.0f);
        const float hardDeceleration = smoothStep((deceleration - 8.0f) / 8.0f);
        const float intensity = braceable
            ? sprintMomentum * hardDeceleration
            : 0.0f;
        const Vec3 drive = heading.conjugate().rotate(
            m_brakeDirection * intensity);
        constexpr float BrakeOmega = 11.0f, BrakeDamping = 0.55f;
        m_brakeReactionVelocity += (BrakeOmega * BrakeOmega
                * (Vec3 { drive.x, drive.y, 0.0f } - m_brakeReaction)
            - 2.0f * BrakeDamping * BrakeOmega * m_brakeReactionVelocity)
            * deltaTime;
        m_brakeReaction += m_brakeReactionVelocity * deltaTime;
        m_brakeReaction = clampMagnitude(m_brakeReaction, 1.2f);
        m_telemetry.brakeReaction = std::hypot(m_brakeReaction.x,
            m_brakeReaction.y);
        m_telemetry.brakeReactionForward = m_brakeReaction.x;
    }
    if (!getUpActive) {
        // Tronco para onde a inercia leva (flexao + para a frente,
        // inclinacao + para a esquerda), a cabeca devolvendo parte para o
        // olhar ficar no horizonte, e os cotovelos abrindo.
        // Freando: o tronco estende um pouco para tras junto com a pelve (a
        // cabeca devolve, o olhar fica no horizonte), nao cai para a frente.
        const float flexion = -0.15f * m_brakeReaction.x;
        const float lateral = -0.08f * m_brakeReaction.y;
        const float amount = m_telemetry.brakeReaction;
        addJointCoordinate(profile, m_coordinates, "Abdomen", 1,
            flexion * 0.30f);
        addJointCoordinate(profile, m_coordinates, "Chest", 1,
            flexion * 0.35f);
        addJointCoordinate(profile, m_coordinates, "UpperChest", 1,
            flexion * 0.35f);
        addJointCoordinate(profile, m_coordinates, "Abdomen", 2,
            lateral * 0.30f);
        addJointCoordinate(profile, m_coordinates, "Chest", 2,
            lateral * 0.35f);
        addJointCoordinate(profile, m_coordinates, "UpperChest", 2,
            lateral * 0.35f);
        addJointCoordinate(profile, m_coordinates, "Neck", 1,
            -flexion * 0.40f);
        addJointCoordinate(profile, m_coordinates, "Head", 1,
            -flexion * 0.25f);
        addJointCoordinate(profile, m_coordinates, "LeftForearm", 0,
            -0.35f * std::min(1.0f, amount));
        addJointCoordinate(profile, m_coordinates, "RightForearm", 0,
            -0.35f * std::min(1.0f, amount));
    }
    // Reflexos de queda por cima da pose (e, no comeco do deitado, por cima
    // da pose deitada: as maos ainda seguram o impacto e depois relaxam).
    {
        float reflex = m_fallReflex;
        float legReflex = std::min(m_fallReflex, m_fallCommit);
        if (m_getUpPhase == CharacterGetUpPhase3D::Settling) {
            reflex *= 1.0f - smoothStep(m_getUpPhaseSeconds / 0.6f);
            legReflex = reflex;
        } else if (m_getUpPhase == CharacterGetUpPhase3D::Rising) {
            reflex = 0.0f;
            legReflex = 0.0f;
        }
        applyFallReflexPose(profile, state, m_coordinates, m_handLinks,
            m_chestLink, m_fallDirection, groundHeightGuess, reflex, legReflex);
        // Vida: esperneando no ar caindo (sem tocar nada ha ~0,1 s).
        const float flail = m_getUpPhase == CharacterGetUpPhase3D::None
            ? m_fallReflex * smoothStep((m_noGroundSeconds - 0.10f) / 0.20f)
            : 0.0f;
        applyAirborneFlail(profile, m_coordinates, m_elapsedSeconds, flail);
        m_telemetry.lifeFlail = flail;
        // Perdendo o equilibrio, de pe: inclinado alem da pose (a partir de
        // ~12 graus) ou atingido (contato externo recente, o corpo indo para
        // onde o jogador nao pediu). Sai quando o reflexo de queda assume.
        float balanceWanted = 0.0f;
        Vec3 balanceDirection = m_balanceDirection;
        if (m_getUpPhase == CharacterGetUpPhase3D::None && !state.links.empty()
            && desiredState != CharacterLocomotionState3D::Airborne) {
            const Vec3 up = state.links.front().orientation.rotate({ 0.0f, 0.0f, 1.0f });
            Vec3 lean = up - m_previousTargetUp;
            lean.z = 0.0f;
            const float tiltError = std::acos(std::clamp(
                dot(up, m_previousTargetUp), -1.0f, 1.0f));
            const float tiltTerm = smoothStep((tiltError - 0.21f) / 0.35f);
            // Movimento nao pedido: ir mais devagar (ou bloqueado) no sentido
            // pedido nao conta - correndo contra uma parede ele nao gira os
            // bracos; ser jogado de lado, para tras ou estando parado, conta.
            Vec3 bodyVelocity = m_filteredCenterOfMassVelocity;
            bodyVelocity.z = 0.0f;
            Vec3 wanted = input.proxyVelocityWorld;
            wanted.z = 0.0f;
            Vec3 drift = bodyVelocity;
            if (wanted.length() > 0.2f) {
                const Vec3 along = wanted.normalized();
                drift -= along * std::clamp(dot(bodyVelocity, along), 0.0f,
                    wanted.length() + 0.5f);
            }
            const float hitTerm = std::clamp(input.externalPerturbation, 0.0f, 1.0f)
                * smoothStep((drift.length() - 0.3f) / 1.0f);
            balanceWanted = std::max(tiltTerm, hitTerm) * (1.0f - m_fallReflex);
            Vec3 direction {};
            if (lean.lengthSquared() > 1e-6f) direction += lean.normalized() * tiltTerm;
            if (drift.lengthSquared() > 1e-6f) direction += drift.normalized() * hitTerm;
            if (direction.lengthSquared() > 1e-6f) balanceDirection = direction.normalized();
            // Tropeco: os bracos saem do balanco comum e abrem/giram para
            // administrar o momento. O reflexo existente gera essa resposta
            // e continua livre para virar protecao das maos se o passo falhar.
            if (m_tripRecovery > balanceWanted) {
                balanceWanted = std::max(balanceWanted,
                    0.75f * m_tripRecovery) * (1.0f - m_fallReflex);
                balanceDirection = m_tripDirection;
            }
        }
        m_balanceReaction += (balanceWanted - m_balanceReaction) * (1.0f
            - std::exp(-deltaTime * (balanceWanted > m_balanceReaction ? 12.0f : 3.0f)));
        {
            const Vec3 blended = m_balanceDirection + (balanceDirection - m_balanceDirection)
                * (1.0f - std::exp(-8.0f * deltaTime));
            if (blended.lengthSquared() > 1e-6f) m_balanceDirection = blended.normalized();
        }
        std::array<bool, 2> braced {};
        for (const RagdollInteraction3D& contact : state.interactions) {
            if (contact.normalWorld.z <= 0.3f || contact.linkIndex >= profile.links.size()) continue;
            for (std::size_t side = 0; side < 2; ++side) {
                const std::size_t hand = m_handLinks[side];
                if (hand == 0 || hand >= profile.links.size()) continue;
                if (contact.linkIndex == hand || static_cast<int>(contact.linkIndex)
                        == profile.links[hand].parentIndex) braced[side] = true;
            }
        }
        applyBalanceReaction(profile, state, m_coordinates, m_handLinks,
            m_chestLink, m_balanceDirection, m_balanceReaction, m_elapsedSeconds,
            braced);
        m_telemetry.balanceReaction = m_balanceReaction;
    }

    // Banking into a turn: a runner carving a curve leans into it and
    // leans forward a little more, roughly in line with the lean/twist
    // magnitudes measured off a real running-turn reference (calibrated by
    // hand against that capture, not sampled from it — see
    // assets/animations/source/mixamo/study/README.txt).
    // v * yawRate is centripetal acceleration. Small curves get a soft
    // onset, and rotating in place does not bank like a running turn.
    const float curveAcceleration = signedCurveAcceleration;
    const float curveShare = smoothStep(std::abs(curveAcceleration) / 3.0f);
    const float turnBank = std::atan(curveAcceleration / 9.81f)
        * 0.72f * curveShare;
    // Freando, a pelve inclina para tras so 15% do que inclinaria (a
    // inclinacao satura: com 40%, uma freada de sprint ainda dava os 12,6
    // graus inteiros): o tronco vai para a frente pela inercia (bloco da
    // freada, acima).
    // Freando, a pelve inclina para tras na proporcao da desaceleracao (ate
    // ~9 graus): o corpo fica atras dos pes para frear (era 15% disso, e o
    // tronco ia para a frente pela inercia).
    const Vec3 brakingAcceleration = m_brakeDirection
        * std::min(0.0f, dot(m_smoothedAcceleration, m_brakeDirection));
    (void)brakingAcceleration;
    const Vec3 localAcceleration = heading.conjugate().rotate(m_smoothedAcceleration);
    // Arrancando, inclina menos (ate ~7 graus, era 12,6: somado a pose da
    // corrida, o peito passava de 30 graus no comeco do trote/sprint).
    const float wantedForwardLean = std::clamp(
        std::atan(localAcceleration.x / 9.81f) * 0.22f
            + std::abs(turnBank) * 0.08f
            + startupRootLean,
        -0.16f, 0.20f);
    // De lado (curva, troca de rumo) pela velocidade: andando e trotando,
    // pouco; so correndo rapido a inclinacao inteira (ate ~11 graus, era 15
    // em qualquer velocidade: mexendo a camera andando ele tombava de lado).
    const float bankSpeedShare = 0.25f + 0.75f * smoothStep((speed - 2.5f) / 4.0f);
    const float wantedLateralLean = std::clamp(
        (std::atan(localAcceleration.y / 9.81f) * 0.18f + turnBank) * bankSpeedShare,
        -0.19f, 0.19f);
    // Exact critically damped spring: continuous lean velocity on turn
    // reversal without delaying the capsule or changing running speed.
    const Vec3 leanTarget { wantedForwardLean, wantedLateralLean, 0.0f };
    const Vec3 leanError = m_lean - leanTarget;
    const Vec3 leanImpulse = m_leanVelocity + leanError * 22.0f;
    const float leanDecay = std::exp(-22.0f * deltaTime);
    m_lean = leanTarget + (leanError + leanImpulse * deltaTime) * leanDecay;
    m_leanVelocity = (m_leanVelocity - leanImpulse * (22.0f * deltaTime)) * leanDecay;
    // Longitudinal acceleration is already filtered upstream. A second
    // lag here moves the hip behind the foot plan when descending steps.
    const float forwardLean = wantedForwardLean;
    const float lateralLean = m_lean.y;

    // A direcao vem do referencial da locomocao. Dos ciclos (autorais, que
    // olham para +X) fica a rotacao inteira da pelve - inclinacao, queda do
    // quadril e o giro de ate 8 graus que acompanha a perna que avanca. Do
    // levantar (importado, com rumo arbitrario) sai o rumo da fonte e fica
    // so a inclinacao.
    const Vec3 sourceForward = rootRotation.rotate({ 1.0f, 0.0f, 0.0f });
    const Quaternion sourceHeading = Quaternion::fromAxisAngle(
        { 0.0f, 0.0f, 1.0f }, std::atan2(sourceForward.y, sourceForward.x));
    const Quaternion clipRootTilt = standUpClip != nullptr
        ? (sourceHeading.conjugate() * rootRotation).normalized()
        : rootRotation.normalized();
    // Mesmo crossfade das juntas (ver `transition` acima). Trocar de clipe
    // nunca pode saltar a pelve: o guia deriva velocidade por diferenca
    // finita, e um salto vira velocidade angular aplicada ao corpo inteiro.
    Quaternion tiltTarget = clipRootTilt;
    if (m_transitionFromRootTilt.x * tiltTarget.x
            + m_transitionFromRootTilt.y * tiltTarget.y
            + m_transitionFromRootTilt.z * tiltTarget.z
            + m_transitionFromRootTilt.w * tiltTarget.w < 0.0f) {
        tiltTarget = { -tiltTarget.x, -tiltTarget.y, -tiltTarget.z,
            -tiltTarget.w };
    }
    const Quaternion blendedRootTilt = Quaternion {
        m_transitionFromRootTilt.x
            + (tiltTarget.x - m_transitionFromRootTilt.x) * transition,
        m_transitionFromRootTilt.y
            + (tiltTarget.y - m_transitionFromRootTilt.y) * transition,
        m_transitionFromRootTilt.z
            + (tiltTarget.z - m_transitionFromRootTilt.z) * transition,
        m_transitionFromRootTilt.w
            + (tiltTarget.w - m_transitionFromRootTilt.w) * transition
    }.normalized();
    const Quaternion rootTilt = (exponentialRotation(
        m_transitionRootTiltVelocity * tangentWeight) * blendedRootTilt).normalized();
    m_rootTiltVelocity = clampMagnitude(rotationVector(
        rootTilt * m_rootTilt.conjugate()) / deltaTime, 10.0f);
    m_rootTilt = rootTilt;
    {
        const Vec3 tiltForward = rootTilt.rotate({ 1.0f, 0.0f, 0.0f });
        m_clipPelvisYaw = std::atan2(tiltForward.y, tiltForward.x);
    }
    const Quaternion accelerationTilt = (
        Quaternion::fromAxisAngle({ 0.0f, 1.0f, 0.0f }, forwardLean)
        * Quaternion::fromAxisAngle({ 1.0f, 0.0f, 0.0f }, -lateralLean))
            .normalized();
    // A recovery plays where the body is lying, not where the capsule is
    // standing: the anchor was captured when the body settled, and the
    // clip's own root track carries it from the floor back to its feet.
    const bool risingPose = m_getUpPhase == CharacterGetUpPhase3D::Rising;
    const Quaternion targetRootOrientation = risingPose
        ? (Quaternion::fromAxisAngle({ 0.0f, 0.0f, 1.0f }, m_getUpAnchorYaw)
            * rootTilt).normalized()
        : (heading * accelerationTilt * rootTilt).normalized();
    m_previousTargetUp = targetRootOrientation.rotate({ 0.0f, 0.0f, 1.0f });
    Vec3 targetRootPosition = risingPose
        ? Vec3 { m_getUpAnchorWorld.x, m_getUpAnchorWorld.y,
            input.rootPositionWorld.z }
        : input.rootPositionWorld;
    // Tropecando, abaixa o centro de massa enquanto a perna livre encurta e
    // passa por cima. A referencia estudada faz isso de forma muito grande;
    // aqui usamos apenas o necessario para manter agilidade e alcance.
    if (!risingPose) {
        targetRootPosition.z -= 0.065f * m_tripRecovery;
        // A compressao precede a arrancada: joelhos/quadris recebem carga
        // antes da extensao, como na referencia, em vez de o peito saltar
        // imediatamente para a inclinacao final. O trote usa menos da metade
        // da descida observada; o sprint ainda fica abaixo dos ~17 cm medidos.
        targetRootPosition.z -= startupPelvisDrop;
    }
    // Rebase: a base da raiz troca de referencial (levantar comeca/termina)
    // ou salta num tick (teletransporte, ancora refeita). A diferenca entre
    // as duas bases nao e movimento pedido - era ela que virava milhares de
    // m/s de "velocidade" na saida do levantar do boneco solto.
    // Toda troca de fase do deitado/levantar tambem: caindo, a altura da
    // referencia passa a ser a da pelve deitada (-98 m/s num tick).
    const bool referenceRebase = risingPose != m_previousRisingPose
        || m_getUpPhase != m_previousReferencePhase
        || (Vec3 { targetRootPosition.x - m_previousRootBase.x,
               targetRootPosition.y - m_previousRootBase.y, 0.0f }).length()
            > 0.25f;
    m_previousRisingPose = risingPose;
    m_previousReferencePhase = m_getUpPhase;
    m_previousRootBase = targetRootPosition;
    const bool airbornePose = desiredState == CharacterLocomotionState3D::JumpStarting
        || desiredState == CharacterLocomotionState3D::Airborne;
    // Deslocamento da raiz vindo do clipe: a altura (a pelve sobe e desce
    // com a passada) e o balanco para o pe de apoio. No referencial do clipe
    // e no MESMO tempo das pernas, com o crossfade das juntas nas trocas.
    //
    // Era filtrado (16/s). A altura ficava atrasada em relacao as pernas, o
    // pe de apoio do alvo afundava no chao e o aterramento (mais abaixo)
    // empurrava a pelve de volta num tick: dente de serra de ate 2,6 cm em
    // 8 ms no sprint - o tremor que se via no corpo andando.
    //
    // O personagem controlado tem a capsula como ancora. No boneco solto,
    // XY vem da pelve fisica: somar balanco ali realimentaria a posicao.
    // Sua altura de repouso usa a base nominal e o ajuste de alcance dos
    // pes. Levantar precisa da trilha vertical para chegar aos pes.
    const bool planarAllowed = !risingPose && !airbornePose
        && input.controlled;
    const bool verticalAllowed = risingPose
        || (input.controlled && !airbornePose);
    const Vec3 authoredOffset {
        planarAllowed ? rootTranslation.x : 0.0f,
        planarAllowed ? rootTranslation.y : 0.0f,
        verticalAllowed ? rootTranslation.z : 0.0f };
    const Vec3 nextRootOffset = m_transitionFromRootOffset
        + (authoredOffset - m_transitionFromRootOffset) * transition
        + m_transitionRootOffsetVelocity * tangentWeight;
    m_rootOffsetVelocity = clampMagnitude((nextRootOffset - m_rootOffset) / deltaTime, 3.0f);
    m_rootOffset = nextRootOffset;
    // Deitado (fisico), a altura da referencia e a do corpo: ela ficava na de
    // pe e, ao comecar o levantar, despencava ate a do clipe em ~0,1 s
    // (referencia a -7,6 m/s). Assim o levantar parte de onde a pelve esta.
    if (input.forceDrivenRoot && m_getUpPhase == CharacterGetUpPhase3D::Settling
        && !state.links.empty()) {
        m_rootOffset.z = state.links.front().position.z - input.rootPositionWorld.z;
        m_rootOffsetVelocity.z = state.links.front().linearVelocity.z;
    }
    targetRootPosition += heading.rotate({ m_rootOffset.x, m_rootOffset.y,
        0.0f });
    targetRootPosition.z += m_rootOffset.z;
    // Freada: a pelve desce com a reacao (as pernas dobram pelo IK dos pes).
    if (!getUpActive && !airbornePose) {
        targetRootPosition.z -= 0.12f * std::min(1.0f,
            m_telemetry.brakeReaction);
        // Carregando o pulo: agacha (as pernas dobram pelo IK dos pes).
        m_jumpCrouch += (std::clamp(input.jumpCrouch, 0.0f, 1.0f) - m_jumpCrouch)
            * (1.0f - std::exp(-12.0f * deltaTime));
        targetRootPosition.z -= 0.10f * m_jumpCrouch;
    } else {
        m_jumpCrouch = 0.0f;
    }
    // Pouso no modo fisico: a pelve de referencia cede junto com o corpo no
    // toque (nunca fica acima dele empurrando - era isso que o fazia quicar:
    // no toque ela estava 20 cm acima e a ajuda de altura ia ao maximo), vai
    // ao agachamento previsto para o impacto, segura e sobe devagar. Direto
    // na pelve alvo: pelo crossfade da pose ele chegava tarde.
    if (input.forceDrivenRoot && !getUpActive && !airbornePose
        && m_sinceTouchdownSeconds < m_landingSettleSeconds
        && !state.links.empty()) {
        const float settle = std::clamp(m_sinceTouchdownSeconds
            / m_landingSettleSeconds, 0.0f, 1.0f);
        const float planned = std::clamp(0.035f * m_landingImpactSpeed,
            0.04f, 0.22f);
        const float yielded = std::clamp(targetRootPosition.z
            - state.links.front().position.z, 0.0f, 0.25f);
        if (settle < 0.45f) {
            m_landingSinkPeak = std::max({ m_landingSinkPeak, yielded,
                planned * smoothStep(settle / 0.15f) });
        }
        targetRootPosition.z -= m_landingSinkPeak
            * (1.0f - smoothStep((settle - 0.45f) / 0.55f));
    } else {
        m_landingSinkPeak = 0.0f;
    }

    RagdollAnimationPose3D localPose;
    buildLocalPose(profile, m_coordinates, localPose);
    for (std::size_t side = 0; side < m_feet.size(); ++side) {
        const FootPlant& foot = m_feet[side];
        if (!foot.valid
            || foot.linkIndex >= localPose.linkOrientations.size()) {
            continue;
        }
        const Quaternion footOrientation = (rootTilt
            * localPose.linkOrientations[foot.linkIndex]).normalized();
        const Vec3 footForward = footOrientation.rotate(
            profile.links[foot.linkIndex].modelOrientation
                .conjugate().rotate({ 1.0f, 0.0f, 0.0f }));
        const float footYaw = std::atan2(footForward.y, footForward.x);
        m_clipPelvisToFootYaw[side] = wrapAngle(
            m_clipPelvisYaw - footYaw);
    }

    // =====================================================================
    // Pes no terreno.
    //
    // Cada pe tem o seu chao. No apoio o pe fica travado no mundo, plano sobre
    // o chao que a sonda achou embaixo dele (calcanhar, meio e ponta; vale o
    // mais alto, para o pe nunca afundar na quina de um degrau). No balanco
    // ele sai da altura de onde levantou e chega na altura de onde VAI pisar:
    // o proprio clipe diz quando e onde o pe toca (o proximo contato), a
    // velocidade da capsula diz onde a pelve estara, e a sonda mede o chao ali
    // antes de o pe chegar. Subindo, o pe ganha a altura cedo e com folga;
    // descendo, so depois de passar a quina. A pelve desce o que for preciso
    // para o pe mais baixo alcancar o chao dele.
    //
    // O ciclo acompanha a distancia que a capsula anda: o ritmo tem limites
    // e o que falta ou sobra vira passo mais curto ou mais longo (stride
    // warping). Parado, se a pelve se afasta de um pe travado (parou no meio
    // da passada, levou um esbarrao), o pe da um passo - o corpo nao anda
    // sem as pernas andarem.
    // =====================================================================
    const float capsuleBase = groundHeightGuess;
    // Degrau da capsula suavizado: o controlador sobe e desce degrau num
    // salto de altura; a pelve vai junto, mas em ~0,1 s.
    if (m_baseTracked && input.grounded && m_baseWasGrounded && !getUpActive) {
        const float jump = capsuleBase - m_previousBaseGround;
        if (std::abs(jump) > 0.025f && std::abs(jump) < 0.6f) {
            m_baseStepOffset -= jump;
        }
    }
    if (!input.grounded || getUpActive) m_baseStepOffset = 0.0f;
    m_baseStepOffset *= std::exp(-12.0f * deltaTime);
    m_previousBaseGround = capsuleBase;
    m_baseTracked = true;
    m_baseWasGrounded = input.grounded;
    targetRootPosition.z += m_baseStepOffset;
    // The spawn may have clearance above the floor. Preserve its initial
    // height, then settle onto the guide before IK, rather than snapping
    // the entire articulated body down on the first physics tick.
    if (!m_spawnHeightCaptured) {
        m_spawnHeightOffset = state.links.front().position.z - targetRootPosition.z;
        m_spawnHeightCaptured = true;
    }
    targetRootPosition.z += m_spawnHeightOffset
        * (1.0f - smoothStep(std::max(0.0f, m_elapsedSeconds - deltaTime) / 0.30f));
    const float baseGround = capsuleBase + m_baseStepOffset;

    // Caindo sem volta, os pes nao miram passos: estao nos reflexos. So
    // desequilibrado, continuam dando passos (e podem salvar o equilibrio).
    const bool footworkEnabled = !getUpActive && m_fallCommit < 0.5f;
    const bool onGround = input.grounded && !airbornePose;
    struct GroundSample {
        bool valid = false;
        float height = 0.0f;
        Vec3 normal { 0.0f, 0.0f, 1.0f };
    };
    const auto groundUnder = [&](std::size_t side, Vec3 center,
        Vec3 forward) -> GroundSample {
        GroundSample result;
        if (input.groundAt) {
            // A sonda procura a partir da altura do chao da capsula (quem
            // implementa groundAt comeca um pouco acima dela): uma viga na
            // altura do peito nao vira chao.
            center.z = baseGround;
            forward.z = 0.0f;
            forward = forward.lengthSquared() > 0.000001f
                ? forward.normalized() : heading.rotate({ 1.0f, 0.0f, 0.0f });
            for (const float reach : { -0.09f, 0.0f, 0.09f }) {
                const GroundProbeResult3D probe =
                    input.groundAt(center + forward * reach);
                if (!probe.hasSurface || !probe.walkable) continue;
                // Evaluate the surface plane at the sole center: selecting
                // the highest toe sample adds artificial height on ramps.
                const float height = probe.pointWorld.z
                    - (probe.normalWorld.x * (center.x - probe.pointWorld.x)
                        + probe.normalWorld.y * (center.y - probe.pointWorld.y))
                        / std::max(0.2f, probe.normalWorld.z);
                if (!result.valid || height > result.height) {
                    result.valid = true;
                    result.height = height;
                    result.normal = probe.normalWorld;
                }
            }
            if (result.valid) return result;
        }
        const GroundProbeResult3D& legacy = input.footGround[side];
        result.valid = true;
        if (legacy.hasSurface) {
            result.height = legacy.pointWorld.z;
            result.normal = legacy.normalWorld;
        } else {
            result.height = baseGround;
        }
        return result;
    };
    // Pe inteiro num nivel: calcanhar, meio e ponta na mesma altura. Numa
    // quina de degrau (o pe tem 28 cm, o degrau tambem), o ponto de pouso
    // anda o minimo ao longo do pe ate caber inteiro num degrau.
    const auto fitFoot = [&](Vec3 center, Vec3 forward) -> Vec3 {
        if (!input.groundAt) return {};
        forward.z = 0.0f;
        if (forward.lengthSquared() < 0.000001f) return {};
        forward = forward.normalized();
        const auto spread = [&](Vec3 at, float& highest) {
            float low = std::numeric_limits<float>::infinity();
            highest = -std::numeric_limits<float>::infinity();
            for (const float reach : { -0.12f, 0.0f, 0.12f }) {
                Vec3 point = at + forward * reach;
                point.z = baseGround;
                const GroundProbeResult3D probe = input.groundAt(point);
                if (!probe.hasSurface) return 1.0f;
                low = std::min(low, probe.pointWorld.z);
                highest = std::max(highest, probe.pointWorld.z);
            }
            return highest - low;
        };
        float level = 0.0f;
        if (spread(center, level) < 0.02f) return {};
        Vec3 best {};
        float bestLevel = -std::numeric_limits<float>::infinity();
        for (float distance = 0.02f; distance <= 0.20f; distance += 0.02f) {
            for (const float sign : { 1.0f, -1.0f }) {
                float height = 0.0f;
                const Vec3 shift = forward * (sign * distance);
                if (spread(center + shift, height) < 0.015f
                    && height > bestLevel) {
                    best = shift;
                    bestLevel = height;
                }
            }
            if (std::isfinite(bestLevel)) return best;
        }
        return {};
    };
    // Rumo do clipe para o pe, sem a inclinacao de aceleracao: o pe de apoio
    // fica com a sola no chao, nao inclinado junto com o corpo.
    const auto clipFootOrientation = [&](std::size_t linkIndex) {
        return (heading * rootTilt * localPose.linkOrientations[linkIndex])
            .normalized();
    };
    const auto lockHeight = [&](const FootPlant& foot, float ground,
        Quaternion orientation) {
        const RagdollLinkDefinition3D& link = profile.links[foot.linkIndex];
        const Quaternion collider = (orientation
            * link.collider.localOrientation).normalized();
        return ground + lowestColliderOffset(link, collider)
            - orientation.rotate(link.collider.localPosition).z + 0.001f;
    };
    const auto forwardOf = [&](const FootPlant& foot, Quaternion orientation) {
        return orientation.rotate(profile.links[foot.linkIndex]
            .modelOrientation.conjugate().rotate({ 1.0f, 0.0f, 0.0f }));
    };
    const ClipFootEvents* events = gaitMoving && selectedClip != nullptr
        ? &footEventsFor(profile, *selectedClip) : nullptr;
    const float clipTravel = selectedClip != nullptr
        ? selectedClip->travelDirectionRadians : 0.0f;
    const Vec3 travelAxisLocal = (accelerationTilt * rootTilt).conjugate()
        .rotate({ std::cos(clipTravel), std::sin(clipTravel), 0.0f });
    Vec3 planarVelocity = Vec3 { input.rootVelocityWorld.x,
        input.rootVelocityWorld.y, 0.0f } * gaitVelocityScale;
    if (gaitStarting && speed > 0.001f)
        planarVelocity *= gaitCycleSpeed / speed;

    struct FootGoal {
        bool active = false;
        Vec3 position;
        Quaternion orientation;
        float pelvisGround = 0.0f;
        // Quanto este pe conta para a pelve alcanca-lo: 1 apoiado ou
        // pousando, subindo no fim do balanco, 0 no ar.
        float reach = 0.0f;
        // O pe do clipe, sem o desnivel do terreno: a pelve so desce o que o
        // terreno pede a mais que a animacao.
        Vec3 animatedPosition;
        Quaternion animatedOrientation;
        // Pe em balanco: onde vai pousar (tornozelo, no mundo), quanto o
        // quadril anda ate la e a perna do clipe no toque. A pelve comeca a
        // descer antes do toque; descendo o ultimo degrau, o pe pousava
        // fora do alcance da perna e a trava soltava e voltava.
        float anticipate = 0.0f;
        Vec3 landingPosition;
        Quaternion landingOrientation;
        Vec3 hipTravel;
        float authoredLandingHorizontal = 0.0f;
        float authoredLandingVertical = 0.0f;
    };
    std::array<FootGoal, 2> goals {};

    // A perna ainda alcanca o pe apoiado? Descendo escada o corpo segue e o
    // degrau de tras fica longe: segurar a trava deixava o pe pendurado no
    // limite do IK (26 cm acima do degrau), marcado como apoiado. Tambem: o
    // IK nao chegou la no tick anterior (limite de junta).
    const auto legOutOfReach = [&](const FootPlant& foot) {
        const RagdollLinkDefinition3D& link = profile.links[foot.linkIndex];
        const int shin = link.parentIndex;
        const int thigh = shin > 0 ? profile.links[
            static_cast<std::size_t>(shin)].parentIndex : -1;
        if (thigh <= 0) return foot.reachError > 0.06f;
        const RagdollLinkDefinition3D& shinLink =
            profile.links[static_cast<std::size_t>(shin)];
        const RagdollLinkDefinition3D& thighLink =
            profile.links[static_cast<std::size_t>(thigh)];
        const float legLength = (shinLink.inboundJoint.anchorModelPosition
                - thighLink.inboundJoint.anchorModelPosition).length()
            + (link.inboundJoint.anchorModelPosition
                - shinLink.inboundJoint.anchorModelPosition).length();
        const Vec3 hip = targetRootPosition + Vec3 { 0.0f, 0.0f, m_pelvisOffset }
            + targetRootOrientation.rotate(profile.links[0].modelOrientation
                .conjugate().rotate(thighLink.inboundJoint.anchorModelPosition
                    - profile.links[0].modelPosition));
        const Vec3 ankle = foot.plantedGoalPosition
            + foot.plantedGoalOrientation.rotate(link.modelOrientation.conjugate()
                .rotate(link.inboundJoint.anchorModelPosition - link.modelPosition));
        return (hip - ankle).length() > legLength * 1.01f || foot.reachError > 0.06f;
    };

    // Pes: o planejador unico de passos (ContactFootwork3D) decide e executa
    // os passos com o corpo parado - acomodacao, giro no lugar, recuperacao
    // de empurrao/pressao/arrasto, refazer a base depois de levantar - e so
    // pousa um pe por contato medido. Andando, a passada do clipe (abaixo)
    // ainda decide os pes; o planejador acompanha para assumir sem salto.
    // Parado de fato: sem passada e sem o jogador pedindo movimento. Na
    // troca de direcao (strafe para um lado e para o outro) a capsula passa
    // por velocidade zero num tick; tomar isso por "parado" entregava os dois
    // pes ao planejador no meio da passada.
    const bool movementRequested =
        input.intent.requestedDirectionWorld.lengthSquared() > 0.01f;
    const bool standing = footworkEnabled && !gaitMoving && !movementRequested
        && input.grounded
        && desiredState != CharacterLocomotionState3D::Landing
        && !airbornePose;
    {
        ContactFootworkInput3D footInput;
        footInput.physical = &physical;
        footInput.state = &state;
        footInput.standing = standing;
        footInput.allowSteps = footworkEnabled && onGround;
        // Recuperacao so no modo fisico, sem queda comprometida e ha mais de
        // 0,8 s de pe (recem-levantado, a base se refaz pelos passos de
        // acomodacao, um pe de cada vez). Segurado pela PhysGun tambem:
        // arrastado, ele da passos.
        footInput.allowRecovery = m_fallCommit < 0.5f
            && m_sinceGetUpSeconds > 0.8f;
        m_telemetry.footworkStanding = standing;
        m_telemetry.footworkAllowSteps = footInput.allowSteps;
        m_telemetry.footworkAllowRecovery = footInput.allowRecovery;
        m_telemetry.captureDirectionWorld = m_captureDirection;
        footInput.seedFromBody = m_plantFeetFromBody && onGround && footworkEnabled;
        footInput.facingYawRadians = m_facingYaw;
        footInput.standingTurnTargetYawRadians = m_standingTurnTarget;
        footInput.heading = heading;
        footInput.referenceRootWorld = targetRootPosition;
        footInput.capturePointWorld = m_capturePoint;
        footInput.captureOutsideMeters = m_captureOutside;
        footInput.captureDirectionWorld = m_captureDirection;
        footInput.planningComVelocityWorld = m_filteredCenterOfMassVelocity;
        footInput.locomoting = movementRequested;
        footInput.desiredVelocityWorld = input.intent.requestedVelocityWorld;
        for (std::size_t side = 0; side < m_feet.size(); ++side) {
            const FootPlant& foot = m_feet[side];
            if (!foot.valid) continue;
            footInput.reference[side].poseWorld = targetRootPosition
                + targetRootOrientation.rotate(localPose.linkPositions[foot.linkIndex]);
            footInput.reference[side].poseOrientationWorld =
                clipFootOrientation(foot.linkIndex);
            footInput.anchorUnreachable[side] = foot.planted && legOutOfReach(foot);
            m_telemetry.footworkAnchorUnreachable[side] = footInput.anchorUnreachable[side];
            footInput.gait[side].planted = foot.planted;
            footInput.gait[side].lockedWorld = foot.lockedPositionWorld;
            footInput.gait[side].lockedOrientationWorld = foot.lockedOrientationWorld;
            footInput.gait[side].surface = { true, foot.groundHeight, foot.groundNormal };
        }
        footInput.surfaceAt = [&](std::size_t side, Vec3 at, Vec3 forward) {
            const GroundSample ground = groundUnder(side, at, forward);
            return FootSurface3D { ground.valid, ground.height, ground.normal };
        };
        footInput.fitOnLevel = [&](Vec3 at, Vec3 forward) { return fitFoot(at, forward); };
        m_footwork.update(profile, footInput, deltaTime);
        if (footInput.seedFromBody) m_plantFeetFromBody = false;
    }
    const CharacterContactPlan3D& contactPlan = m_footwork.plan();
    const bool recovering = contactPlan.recovering;
    m_reactiveShare = recovering ? 1.0f : 0.0f;
    m_telemetry.reactiveShare = m_reactiveShare;
    // O estado oficial dos pes do planejador, copiado no registro que o IK,
    // o limite de torcao do quadril e o aterramento leem.
    for (std::size_t side = 0; side < m_feet.size(); ++side) {
        if (!m_footwork.owns(side)) {
            // Com a passada: nenhum passo do planejador neste pe.
            m_feet[side].repositioning = false;
            m_feet[side].recoveryStep = false;
            m_feet[side].turnStep = false;
            m_feet[side].pivotStep = false;
            continue;
        }
        FootPlant& foot = m_feet[side];
        const FootPlan3D& plan = contactPlan.feet[side];
        const bool down = plan.phase == FootPhase3D::Stance
            || plan.phase == FootPhase3D::Unloading
            || plan.phase == FootPhase3D::Loading;
        foot.planted = down;
        foot.repositioning = !down;
        foot.recoveryStep = plan.reason == StepReason3D::Recovery;
        foot.turnStep = plan.reason == StepReason3D::Turn;
        foot.pivotStep = !down && plan.path == SwingPath3D::Pivot;
        foot.stepPending = false;
        foot.releaseBlend = 0.0f;
        foot.replantDelay = 0.0f;
        foot.takeoffValid = false;
        foot.activeShift = {};
        foot.fitValid = false;
        if (down) {
            foot.lockedPositionWorld = plan.anchorWorld;
            foot.lockedOrientationWorld = plan.anchorOrientationWorld;
            if (plan.surface.valid) {
                foot.groundHeight = plan.surface.height;
                foot.groundNormal = plan.surface.normal;
            }
        } else {
            foot.repositionFromWorld = plan.liftoffWorld;
            foot.repositionFromOrientation = plan.liftoffOrientationWorld;
            foot.repositionToWorld = plan.goalWorld;
            foot.takeoffGroundHeight = plan.liftoffGroundHeight;
        }
    }
    // Peso para o pe de apoio num passo comum (pedido do planejador).
    Vec3 standingShift = contactPlan.supportShiftWorld;
    // E5 (animacao + pes conscientes): quanto a velocidade do corpo real se
    // afastou da da referencia (a capsula), em ponto de captura (/ omega). O
    // pe em balanco da passada pousa deslocado disso - a animacao continua
    // decidindo o passo; andando bem, a diferenca e quase nula. So a
    // velocidade: pela posicao tambem, com a pelve um pouco atras da capsula
    // (o atraso normal do corpo), o pe pousava atras sem desequilibrio
    // nenhum e freava o corpo (ajuda a 40%: uma queda a mais). Filtrado
    // ~50 ms, ate 15 cm.
    Vec3 balanceShift {};
    // Numa recuperacao forte, o pe no ar pode ampliar a base no sentido da
    // freada. A mola precisa ter passado de metade da reacao antes disso:
    // residuos pequenos ficam no tronco e nunca criam a passada projetada.
    const float emergencyBrakeStep = smoothStep(
        (m_brakeReaction.x - 0.55f) / 0.35f);
    if (input.forceDrivenRoot && onGround && emergencyBrakeStep > 0.0f)
        balanceShift += m_brakeDirection * (0.14f * emergencyBrakeStep);
    // Recuperacao sem perder velocidade: amplia a passada no rumo do
    // movimento. Se o obstaculo pegou cedo, o proprio pe continua por cima;
    // se pegou tarde, este deslocamento permanece para o passo seguinte.
    if (input.forceDrivenRoot && onGround && m_tripRecovery > 0.0f)
        balanceShift += m_tripDirection * (0.26f * m_tripRecovery);
    if (input.balanceFootPlacement && input.forceDrivenRoot && onGround) {
        const float omega = std::clamp(m_estimator.state().captureOmega, 1.0f, 6.0f);
        const PhysicsBodyState3D& pelvis = state.links.front();
        const Vec3 velocityError { pelvis.linearVelocity.x - planarVelocity.x,
            pelvis.linearVelocity.y - planarVelocity.y, 0.0f };
        Vec3 wanted = velocityError * (BalancePlacementGain / omega);
        if (wanted.length() > BalancePlacementLimitMeters)
            wanted = wanted * (BalancePlacementLimitMeters / wanted.length());
        m_balancePlacementShift += (wanted - m_balancePlacementShift)
            * (1.0f - std::exp(-deltaTime / 0.05f));
        balanceShift += m_balancePlacementShift;
    } else {
        m_balancePlacementShift = {};
    }

    for (std::size_t side = 0; side < m_feet.size(); ++side) {
        FootPlant& foot = m_feet[side];
        m_telemetry.footPlanted[side] = false;
        m_telemetry.footSwingProgress[side] = -1.0f;
        m_telemetry.footClearanceMeters[side] = 0.0f;
        if (!foot.valid || !footworkEnabled) continue;
        const RagdollLinkDefinition3D& link = profile.links[foot.linkIndex];
        FootGoal& goal = goals[side];
        goal.active = true;

        // Pe animado, com o passo encurtado ou alongado ao longo da direcao
        // do ciclo, em torno do quadril.
        Vec3 footLocal = localPose.linkPositions[foot.linkIndex];
        if (gaitMoving && std::abs(strideScale - 1.0f) > 0.005f) {
            const int shin = link.parentIndex;
            const int thigh = shin > 0 ? profile.links[
                static_cast<std::size_t>(shin)].parentIndex : -1;
            if (thigh > 0) {
                const Vec3 hip = profile.links[0].modelOrientation.conjugate()
                    .rotate(profile.links[static_cast<std::size_t>(thigh)]
                        .inboundJoint.anchorModelPosition
                        - profile.links[0].modelPosition);
                const float along = dot(footLocal - hip, travelAxisLocal);
                footLocal += travelAxisLocal * ((strideScale - 1.0f) * along);
            }
        }
        const Vec3 animatedWorld = targetRootPosition
            + targetRootOrientation.rotate(footLocal);
        const Quaternion animatedOrientation =
            clipFootOrientation(foot.linkIndex);
        goal.animatedPosition = animatedWorld;
        goal.animatedOrientation = animatedOrientation;
        const Vec3 animatedForward = forwardOf(foot, animatedOrientation);

        if (m_footwork.owns(side)) {
            foot.previousAnimatedPositionWorld = animatedWorld;
            foot.previousAnimatedPositionValid = true;
            if (!foot.planted) {
                // No ar pelo planejador: a trajetoria dele, ate o contato.
                const FootPlan3D& plan = contactPlan.feet[side];
                goal.position = plan.targetWorld;
                goal.orientation = plan.targetOrientationWorld;
                goal.pelvisGround = plan.goalSurface.valid
                    ? std::min(plan.liftoffGroundHeight, plan.goalSurface.height)
                    : plan.liftoffGroundHeight;
                goal.reach = onGround ? 1.0f : 0.0f;
                continue;
            }
        } else {
            const float sole = footSoleOffset(link);
            // Folga do pe em relacao ao chao do CLIPE (a base da capsula): e o que
            // diz se a pose esta pisando, independente do degrau embaixo.
            const float clearance = animatedWorld.z - baseGround - sole;
            m_telemetry.footClearanceMeters[side] = clearance;
            const Vec3 animatedVelocity = foot.previousAnimatedPositionValid
                ? (animatedWorld - foot.previousAnimatedPositionWorld) / deltaTime
                : Vec3 {};
            const bool authoredStance = onGround
                && desiredState != CharacterLocomotionState3D::JumpStarting
                && contactWeights[side] > 0.55f;
            // A velocidade do pe animado escala com o ritmo (a mesma passada em
            // menos tempo); o portao tambem, para a trava valer em toda a faixa.
            const float velocityGateScale = std::max(1.0f, playbackRate);
            const bool stanceCandidate = authoredStance
                && clearance < 0.105f && clearance > -0.065f
                && std::hypot(animatedVelocity.x, animatedVelocity.y)
                    < 0.72f * velocityGateScale
                && std::abs(animatedVelocity.z) < 1.1f * velocityGateScale;
            foot.replantDelay = std::max(0.0f, foot.replantDelay - deltaTime);
            if (!foot.planted && stanceCandidate && foot.replantDelay <= 0.0f) {
                // Encaixe de novo no toque: a previsao erra alguns centimetros
                // quando a capsula muda de velocidade (cada degrau subido a
                // freia). Ate 10 cm; alem disso o pe fica onde a pose pos.
                Vec3 touchdown = animatedWorld + foot.activeShift;
                const Vec3 correction = fitFoot(touchdown, animatedForward);
                if (correction.lengthSquared() <= 0.01f) {
                    touchdown += correction;
                    foot.activeShift += correction;
                }
                const GroundSample ground = groundUnder(side, touchdown,
                    animatedForward);
                const Quaternion orientation = (rotationBetween(
                    { 0.0f, 0.0f, 1.0f }, ground.normal) * animatedOrientation)
                        .normalized();
                foot.planted = true;
                foot.lockedOrientationWorld = orientation;
                foot.lockedPositionWorld = animatedWorld + foot.activeShift;
                foot.activeShift = {};
                foot.fitValid = false;
                // O proximo balanco sai daqui (gravado ao soltar a trava).
                foot.takeoffValid = false;
                foot.lockedPositionWorld.z = lockHeight(foot, ground.height,
                    orientation);
                foot.groundHeight = ground.height;
                foot.groundNormal = ground.normal;
                foot.plantedGoalPosition = foot.lockedPositionWorld;
                foot.plantedGoalOrientation = orientation;
            } else if (foot.planted) {
                const float horizontalDrift = std::hypot(
                    animatedWorld.x - foot.lockedPositionWorld.x,
                    animatedWorld.y - foot.lockedPositionWorld.y);
                // Fora de alcance (ver legOutOfReach), o pe solta e vira balanco.
                const bool outOfReach = legOutOfReach(foot);
                // Solta quando o clipe diz que o apoio acabou. Com folga (ate o
                // contato cair a 0,30) o pe ficava preso enquanto a pose ja
                // levava o pe embora; na soltura ele corria atras dela (medido:
                // 12 m/s no sprint de lado, com o corpo a 3,7).
                // (Parado, quem decide os pes e o planejador de passos - a
                // passada nao espera mais a marca de recuperacao dele.)
                const bool releaseByGait = contactWeights[side] < 0.55f
                    || outOfReach || (!stanceCandidate && clearance > 0.055f)
                    || horizontalDrift > 0.48f;
                if (releaseByGait || !input.grounded) {
                    foot.planted = false;
                    // Solto porque a perna nao alcanca: nao trava de novo no tick
                    // seguinte. Travando e soltando a cada tick, cada trava nova
                    // puxava o alvo para a pose (pe a 6,6 m/s depois de levantar).
                    if (outOfReach) foot.replantDelay = 0.12f;
                    foot.releaseBlend = 1.0f;
                    foot.releasePositionWorld = foot.plantedGoalPosition;
                    foot.releaseOrientationWorld = foot.plantedGoalOrientation;
                    // De onde o balanco sai: altura e ponto. Sem isto ficava o de
                    // um passo antigo (medido: saida 2,5 m para tras), e o pe subia
                    // para o degrau de pouso logo no comeco do balanco.
                    foot.takeoffGroundHeight = foot.groundHeight;
                    foot.takeoffPosition = foot.plantedGoalPosition;
                    foot.takeoffValid = true;
                    foot.swingFloor = foot.groundHeight;
                }
            }
            foot.previousAnimatedPositionWorld = animatedWorld;
            foot.previousAnimatedPositionValid = true;
        }

        if (foot.planted) {
            foot.releaseBlend = 0.0f;
            // Travado, o pe ainda rola: quando o clipe ergue o calcanhar (a
            // saida) ele gira sobre a ponta, e quando pousa de calcanhar,
            // sobre o calcanhar - o ponto de apoio fica parado no mundo e o
            // rumo do pe fica o da trava. Congelado plano, o pe so rolava ao
            // soltar a trava, de uma vez (medido: chicote de 8,9 m/s no
            // strafe).
            const Vec3 normal = foot.groundNormal;
            Quaternion rolled = (rotationBetween({ 0.0f, 0.0f, 1.0f }, normal)
                * animatedOrientation).normalized();
            Vec3 lockedForward = forwardOf(foot, foot.lockedOrientationWorld);
            Vec3 rolledForward = forwardOf(foot, rolled);
            lockedForward -= normal * dot(lockedForward, normal);
            rolledForward -= normal * dot(rolledForward, normal);
            if (lockedForward.lengthSquared() > 0.0001f
                && rolledForward.lengthSquared() > 0.0001f) {
                const Vec3 a = lockedForward.normalized();
                const Vec3 b = rolledForward.normalized();
                const float yaw = std::atan2(dot(cross(b, a), normal),
                    dot(a, b));
                rolled = (Quaternion::fromAxisAngle(normal, yaw) * rolled)
                    .normalized();
            }
            const Vec3 half = link.collider.shape == RagdollColliderShape3D::Box
                ? link.collider.boxHalfExtents : Vec3 {};
            const Vec3 toe = link.collider.localPosition
                + link.collider.localOrientation.rotate(
                    { half.x, 0.0f, -half.z });
            const Vec3 heel = link.collider.localPosition
                + link.collider.localOrientation.rotate(
                    { -half.x, 0.0f, -half.z });
            const float toeDrop = dot(rolled.rotate(toe)
                - foot.lockedOrientationWorld.rotate(toe), normal);
            const float heelDrop = dot(rolled.rotate(heel)
                - foot.lockedOrientationWorld.rotate(heel), normal);
            Vec3 position = foot.lockedPositionWorld;
            if (half.x > 0.0f && (toeDrop < -0.001f || heelDrop < -0.001f)) {
                const Vec3 pivot = toeDrop < heelDrop ? toe : heel;
                const Vec3 pivotWorld = foot.lockedPositionWorld
                    + foot.lockedOrientationWorld.rotate(pivot);
                position = pivotWorld - rolled.rotate(pivot);
            } else {
                rolled = foot.lockedOrientationWorld;
            }
            foot.plantedGoalPosition = position;
            foot.plantedGoalOrientation = rolled;
            goal.position = position;
            goal.orientation = rolled;
            goal.pelvisGround = foot.groundHeight;
            goal.reach = 1.0f;
            m_telemetry.footPlanted[side] = true;
            continue;
        }

        // Balanco. Chao de onde o pe saiu e de onde ele vai pisar.
        float swingGround = baseGround;
        float pelvisGround = baseGround;
        Quaternion swingOrientation = animatedOrientation;
        float lift = 0.0f;
        const FootTouchdown* next = nullptr;
        float progress = 0.0f;
        float secondsToTouchdown = 0.0f;
        float swingRise = 0.0f;
        float swingPathShare = 0.0f;
        Vec3 swingLanding;
        if (events != nullptr && onGround) {
            const float duration = std::max(0.05f,
                selectedClip->durationSeconds);
            float best = std::numeric_limits<float>::infinity();
            for (const FootTouchdown& candidate : events->touchdowns[side]) {
                const float ahead = std::fmod(candidate.touchdown
                    - clipSampleTime + duration * 2.0f, duration);
                if (ahead < best) {
                    best = ahead;
                    next = &candidate;
                }
            }
            // A trava vem alguns ticks depois do toque do clipe (contato
            // acima de 0,55, pe animado parado). Nesse meio tempo o pe ainda
            // esta pousando naquele toque: passar para o toque seguinte
            // jogava fora o encaixe no degrau (o pe voltava 12 cm, para a
            // quina) e a altura de pouso.
            for (const FootTouchdown& candidate : events->touchdowns[side]) {
                const float swing = std::fmod(candidate.touchdown
                    - candidate.liftoff + duration * 2.0f, duration);
                const float since = std::fmod(clipSampleTime
                    - candidate.touchdown + duration * 2.0f, duration);
                if (since < 0.5f * (duration - swing)) {
                    next = &candidate;
                    best = 0.0f;
                    break;
                }
            }
            if (next != nullptr && best == 0.0f) {
                progress = 1.0f;
                secondsToTouchdown = 0.0f;
            } else if (next != nullptr) {
                const float swing = std::fmod(next->touchdown - next->liftoff
                    + duration * 2.0f, duration);
                const float elapsed = std::fmod(clipSampleTime - next->liftoff
                    + duration * 2.0f, duration);
                progress = swing > 0.001f && elapsed <= swing
                    ? std::clamp(elapsed / swing, 0.0f, 1.0f) : 0.0f;
                secondsToTouchdown = best / std::max(0.2f, playbackRate);
            }
        }
        if (next != nullptr) {
            // Onde o pe vai pisar: a pelve la na hora do toque (velocidade da
            // capsula), mais o pe do clipe naquele instante, com o mesmo
            // encurtamento de passo.
            Vec3 footInClip = next->footInClip;
            const float along = dot(footInClip - next->hipInClip,
                Vec3 { std::cos(clipTravel), std::sin(clipTravel), 0.0f });
            footInClip += Vec3 { std::cos(clipTravel), std::sin(clipTravel),
                0.0f } * ((strideScale - 1.0f) * along);
            Vec3 landingWorld = input.rootPositionWorld
                + planarVelocity * secondsToTouchdown
                + heading.rotate(footInClip) + balanceShift;
            // A pelve nao estara no prolongamento reto da velocidade atual:
            // numa curva, ela percorre um arco. A previsao centripeta leva o
            // pouso para dentro da curva e ja orienta o pe pelo rumo esperado
            // no toque. Sem isso os pes continuavam correndo reto enquanto o
            // tronco girava por cima deles.
            const float arcOffset = std::clamp(0.5f
                * signedCurveAcceleration * secondsToTouchdown
                * secondsToTouchdown, -0.22f, 0.22f);
            landingWorld += heading.rotate({0.0f, arcOffset, 0.0f});
            const Quaternion touchdownTurn = Quaternion::fromAxisAngle(
                {0.0f, 0.0f, 1.0f}, m_turningRate * secondsToTouchdown);
            Vec3 landingForward = touchdownTurn.rotate(
                heading.rotate(next->forwardInClip));
            // O encaixe no degrau e caro (varias sondas): refeito so quando o
            // pouso previsto anda mais de 3 cm.
            const Vec3 moved { landingWorld.x - foot.fitCenter.x,
                landingWorld.y - foot.fitCenter.y, 0.0f };
            if (!foot.fitValid || moved.lengthSquared() > 0.0009f) {
                foot.fitShift = fitFoot(landingWorld, landingForward);
                foot.fitCenter = landingWorld;
                foot.fitValid = true;
            }
            landingWorld += foot.fitShift;
            const GroundSample landing = groundUnder(side, landingWorld,
                landingForward);
            if (!foot.takeoffValid) {
                foot.takeoffGroundHeight = foot.groundHeight;
                foot.takeoffPosition = foot.releaseBlend > 0.0f
                    ? foot.releasePositionWorld : animatedWorld;
                foot.takeoffValid = true;
                foot.swingFloor = foot.groundHeight;
            }
            foot.landingGroundHeight = landing.height;
            foot.landingNormal = landing.normal;
            const float from = foot.takeoffGroundHeight;
            const float to = landing.height;
            const float rise = to - from;
            // Quanto do caminho ate o pouso o pe ja andou, na horizontal.
            // Sem a sonda de terreno, a altura do degrau novo acompanha isso:
            // subindo, chega a 70% do caminho; descendo, so depois da metade.
            // Com a sonda, quem decide e o chao a frente do pe (mais abaixo).
            const Vec3 path { landingWorld.x - foot.takeoffPosition.x,
                landingWorld.y - foot.takeoffPosition.y, 0.0f };
            foot.swingDirection = path.lengthSquared() > 0.0004f
                ? path.normalized() : Vec3 {};
            const Vec3 travelled { animatedWorld.x - foot.takeoffPosition.x,
                animatedWorld.y - foot.takeoffPosition.y, 0.0f };
            const float pathShare = path.lengthSquared() > 0.0004f
                ? std::clamp(dot(travelled, path) / path.lengthSquared(),
                    0.0f, 1.0f)
                : progress;
            const float share = rise > 0.01f
                ? smoothStep(pathShare / 0.7f)
                : smoothStep((pathShare - 0.35f) / 0.65f);
            swingGround = from + rise * share;
            lift = std::max(0.0f, rise) * 0.2f * std::sin(Pi * pathShare);
            swingRise = rise;
            swingPathShare = pathShare;
            swingLanding = landingWorld;
            goal.anticipate = smoothStep((progress - 0.3f) / 0.5f);
            // Paralelo ao chao de pouso, so com o rumo do pe do clipe.
            const Quaternion landingSurface = rotationBetween({ 0.0f, 0.0f, 1.0f },
                landing.normal);
            const Vec3 animatedAhead = animatedOrientation.rotate({ 1.0f, 0.0f, 0.0f });
            const Quaternion landingFlat = (landingSurface * Quaternion::fromAxisAngle(
                { 0.0f, 0.0f, 1.0f }, std::atan2(animatedAhead.y, animatedAhead.x))).normalized();
            goal.landingOrientation = (landingSurface * animatedOrientation).normalized();
            if (input.footSpring && input.forceDrivenRoot)
                goal.landingOrientation = slerpQuaternion(goal.landingOrientation,
                    landingFlat, FootLandingFlatten).normalized();
            goal.landingPosition = { landingWorld.x, landingWorld.y,
                lockHeight(foot, landing.height, goal.landingOrientation) };
            goal.hipTravel = planarVelocity * secondsToTouchdown;
            goal.authoredLandingHorizontal = std::hypot(
                footInClip.x - next->hipInClip.x,
                footInClip.y - next->hipInClip.y);
            goal.authoredLandingVertical = next->hipInClip.z - footInClip.z;
            pelvisGround = from + rise * progress;
            // O pe chega orientado pelo chao de destino: comeca a se alinhar
            // com ele a 30% do balanco e esta alinhado a 75% (era de 60% ate o
            // toque: numa rampa ele ainda girava ao pousar).
            const float settle = input.footSpring
                ? smoothStep((progress - 0.30f) / 0.45f)
                : smoothStep((progress - 0.6f) / 0.4f);
            swingOrientation = (slerpQuaternion(Quaternion {},
                rotationBetween({ 0.0f, 0.0f, 1.0f }, landing.normal), settle)
                * animatedOrientation).normalized();
            // Pe com mola: perto do toque ele vai ficando paralelo ao chao de
            // pouso - fica 30% da inclinacao do clipe. Pelo clipe o trote
            // pousava com a ponta 25 graus para baixo (o tornozelo atrasado
            // disfarcava), e a ponta batia na quina do meio-fio.
            if (input.footSpring && input.forceDrivenRoot)
                swingOrientation = slerpQuaternion(swingOrientation, landingFlat,
                    FootLandingFlatten * smoothStep((progress - 0.50f) / 0.40f)).normalized();
        } else if (onGround) {
            const GroundSample ground = groundUnder(side, animatedWorld,
                animatedForward);
            swingGround = ground.height;
            pelvisGround = ground.height;
            foot.takeoffValid = false;
        } else {
            foot.takeoffValid = false;
        }
        if (next == nullptr || progress >= 0.999f) foot.takeoffValid = false;
        if (next != nullptr) m_telemetry.footSwingProgress[side] = progress;
        // O pe vai sendo levado para o ponto de pouso encaixado no degrau.
        Vec3 shift = next != nullptr && foot.fitValid
            ? foot.fitShift * smoothStep(progress / 0.8f) : Vec3 {};
        // O deslocamento do equilibrio entra ao longo do passo (sem salto na
        // saida); a trava do pouso o inclui (activeShift), o pe nao escorrega.
        if (next != nullptr) shift += balanceShift * smoothStep(progress / 0.6f);
        foot.activeShift = shift;
        Vec3 swingTarget = animatedWorld + shift;
        // Chao embaixo do pe agora (os degraus do caminho).
        float floorNow = swingGround;
        if (next != nullptr && input.groundAt) {
            // O pe passa por cima do terreno que tem pela frente, nao sobe
            // antes para o degrau de pouso. Sondas a frente da ponta, no
            // rumo do caminho ate o pouso: cada espelho de degrau vira uma
            // rampa (1,1 de inclinacao, 3 cm de folga na quina), e atras
            // do calcanhar, para ele nao raspar a borda descendo. Subir para
            // o degrau de pouso cedo (a regra de antes) levava o pe, ainda
            // atras do quadril, a 48 cm do degrau embaixo dele - o coice.
            const Vec3 direction = foot.swingDirection.lengthSquared() > 0.5f
                ? foot.swingDirection : animatedForward;
            const GroundSample under = groundUnder(side, swingTarget,
                direction);
            floorNow = under.height;
            const auto probe = [&](float reach) {
                Vec3 point = swingTarget + direction * reach;
                point.z = baseGround;
                const GroundProbeResult3D hit = input.groundAt(point);
                return hit.hasSurface && hit.walkable ? hit.pointWorld.z
                    : -std::numeric_limits<float>::infinity();
            };
            // So o terreno ate o pouso: o degrau depois dele nao conta (o pe
            // pousava pendurado 12 cm acima). Atras, so dentro do pe (o pe
            // encaixado cabe inteiro no degrau de pouso).
            constexpr float HalfFoot = 0.14f, RampSlope = 1.1f,
                NoseClearance = 0.03f;
            const float remaining = dot(swingLanding - swingTarget,
                direction);
            // Atras, so segura o pe na altura de onde ele saiu ate o
            // calcanhar passar a borda (descendo); nunca o ergue.
            float needed = std::max(under.height, std::min(probe(-0.12f),
                foot.takeoffGroundHeight));
            for (const float ahead : { 0.02f, 0.08f, 0.14f, 0.20f }) {
                if (ahead > remaining) break;
                needed = std::max(needed, probe(HalfFoot + ahead)
                    + NoseClearance - RampSlope * ahead);
            }
            // Sobe na hora (a rampa ja da a distancia), no maximo 6 m/s.
            // Desce no ritmo que chega ao chao do pouso no toque.
            if (needed >= foot.swingFloor) {
                foot.swingFloor = std::min(needed,
                    foot.swingFloor + 6.0f * deltaTime);
            } else {
                const float fallRate = std::max(1.5f,
                    (foot.swingFloor - foot.landingGroundHeight)
                        / std::max(0.06f, secondsToTouchdown));
                foot.swingFloor = std::max(needed,
                    foot.swingFloor - fallRate * deltaTime);
            }
            swingGround = foot.swingFloor;
            // Chega ao degrau de pouso a tempo, mesmo se a sonda nao o viu.
            swingGround += (foot.landingGroundHeight - swingGround)
                * smoothStep((progress - 0.8f) / 0.2f);
            lift = 0.0f;
        }
        foot.groundHeight = swingGround;
        // Passo curto, pe mais baixo: a altura que o clipe da ao pe em
        // balanco (o calcanhar subindo atras, o joelho a frente) e a de uma
        // passada inteira. Com o passo encurtado (escada, devagar) ela
        // encolhe junto, ate 40%.
        const float flatHeight = lockHeight(foot, 0.0f, animatedOrientation);
        const float verticalScale = strideScale < 1.0f
            ? 1.0f - 0.6f * (1.0f - strideScale) : 1.0f;
        const float clipHeight = animatedWorld.z - baseGround - flatHeight;
        swingTarget.z = swingGround + flatHeight
            + std::max(0.0f, clipHeight) * verticalScale
            + std::min(0.0f, clipHeight) + lift;
        // Folga minima do balanco (modo fisico): no meio do passo o pe passa
        // pelo menos ~13 cm acima do chao. O strafe da referencia ergue o pe
        // so 7-12 cm; com a perna fisica acompanhando um pouco atrasada, o pe
        // rocava o chao a passada inteira (arrastando: 1,3 s de 2,2 s). So no
        // strafe: para a frente o clipe ja ergue 17 cm, e mais alto ele
        // passava por cima de obstaculos em que devia tropecar.
        const bool strafing = m_gaitDirection == CharacterGaitDirection3D::Left
            || m_gaitDirection == CharacterGaitDirection3D::Right;
        if (input.forceDrivenRoot && strafing && next != nullptr && onGround) {
            // Tira do chao cedo (ate ~30% do passo), segura e so desce no fim:
            // num arco em seno o pe saia devagar, ja deslizando de lado.
            const float p = std::clamp(progress, 0.0f, 1.0f);
            const float arch = smoothStep(p / 0.30f)
                * (1.0f - smoothStep((p - 0.65f) / 0.35f));
            const float clearance = swingTarget.z - (swingGround + flatHeight);
            const float wanted = SwingMinimumClearance * arch;
            if (clearance < wanted) swingTarget.z += wanted - clearance;
        }
        // Trote frontal: recolhe a ponta durante o meio do balanco. A altura
        // adicional e pequena e progressiva; evita que o atraso fisico do
        // tornozelo transforme a passada em uma bicuda no piso. Num tropeco
        // cedo, a estrategia de elevacao ergue o mesmo pe e continua o passo.
        const bool forwardJog = input.forceDrivenRoot && next != nullptr
            && onGround && m_gaitDirection == CharacterGaitDirection3D::Forward
            && !input.sprinting;
        if (forwardJog) {
            const float p = std::clamp(progress, 0.0f, 1.0f);
            const float arch = smoothStep(p / 0.24f)
                * (1.0f - smoothStep((p - 0.66f) / 0.27f));
            const float clearance = swingTarget.z - (swingGround + flatHeight);
            float wanted = 0.18f * arch;
            if (m_tripElevating && m_tripFoot == static_cast<int>(side))
                wanted += 0.10f * m_tripRecovery
                    * (1.0f - smoothStep((p - 0.72f) / 0.20f));
            if (clearance < wanted) swingTarget.z += wanted - clearance;
            // Dorsiflexao: ponta aproximadamente 7 graus acima no meio do
            // balanco, voltando gradualmente ao plano do pouso.
            const Vec3 lateral = swingOrientation.rotate({ 0.0f, 1.0f, 0.0f });
            swingOrientation = (Quaternion::fromAxisAngle(lateral,
                -0.12f * arch) * swingOrientation).normalized();
        }
        // Nenhum pe do alvo abaixo do chao que esta passando por baixo dele.
        const float lowestAllowed = lockHeight(foot, floorNow,
            swingOrientation) + (floorNow > swingGround + 0.01f ? 0.02f : 0.0f);
        swingTarget.z = std::max(swingTarget.z, lowestAllowed);
        if (foot.releaseBlend > 0.0f) {
            // A trava nao some num tick: o pe volta para o balanco em ~0,12 s.
            constexpr float ReleaseSeconds = 0.12f;
            foot.releaseBlend = std::max(0.0f,
                foot.releaseBlend - deltaTime / ReleaseSeconds);
            const float hold = smoothStep(foot.releaseBlend);
            swingTarget = swingTarget
                + (foot.releasePositionWorld - swingTarget) * hold;
            swingOrientation = slerpQuaternion(swingOrientation,
                foot.releaseOrientationWorld, hold);
        }
        goal.position = swingTarget;
        goal.orientation = swingOrientation;
        m_telemetry.footClipPitch[side] = -std::asin(std::clamp(
            animatedOrientation.rotate({ 1.0f, 0.0f, 0.0f }).z, -1.0f, 1.0f)) * 57.2958f;
        m_telemetry.footGoalPitch[side] = -std::asin(std::clamp(
            swingOrientation.rotate({ 1.0f, 0.0f, 0.0f }).z, -1.0f, 1.0f)) * 57.2958f;
        goal.pelvisGround = pelvisGround;
        goal.reach = !onGround ? 0.0f
            : next != nullptr ? smoothStep((progress - 0.4f) / 0.5f)
            : 1.0f;
    }

    // An uncontrolled root uses its physical XY as the next input anchor;
    // feeding the support shift back into it would integrate a displacement.
    if (!input.controlled) standingShift.x = standingShift.y = 0.0f;
    targetRootPosition += standingShift;

    // Pelve pelo alcance das pernas: desce o que for preciso para o quadril
    // alcancar cada pe apoiado (ou pousando) com a perna a 94% de esticada -
    // nem mais, nem menos. Descer ate o pe mais baixo (a regra de antes)
    // agachava o corpo na escada sem precisar; e com a capsula subindo as
    // quinas de forma continua, 8-16 cm acima do degrau de apoio, o pe recem
    // pousado ficava pendurado. Com os dois pes apoiados acima da base da
    // capsula, a pelve sobe.
    float drop = 0.0f;
    float raise = std::numeric_limits<float>::infinity();
    bool bothAbove = true;
    for (std::size_t side = 0; side < goals.size(); ++side) {
        const FootGoal& goal = goals[side];
        if (!goal.active || goal.reach <= 0.0f) {
            bothAbove = false;
            continue;
        }
        const RagdollLinkDefinition3D& footLink =
            profile.links[m_feet[side].linkIndex];
        const int shin = footLink.parentIndex;
        const int thigh = shin > 0
            ? profile.links[static_cast<std::size_t>(shin)].parentIndex : -1;
        if (thigh <= 0) continue;
        const RagdollLinkDefinition3D& shinLink =
            profile.links[static_cast<std::size_t>(shin)];
        const RagdollLinkDefinition3D& thighLink =
            profile.links[static_cast<std::size_t>(thigh)];
        const Vec3 hipLocal = profile.links[0].modelOrientation.conjugate()
            .rotate(thighLink.inboundJoint.anchorModelPosition
                - profile.links[0].modelPosition);
        const float legLength = (shinLink.inboundJoint.anchorModelPosition
                - thighLink.inboundJoint.anchorModelPosition).length()
            + (footLink.inboundJoint.anchorModelPosition
                - shinLink.inboundJoint.anchorModelPosition).length();
        const Vec3 hip = targetRootPosition
            + targetRootOrientation.rotate(hipLocal);
        const Vec3 anchorInFoot = footLink.modelOrientation.conjugate()
            .rotate(footLink.inboundJoint.anchorModelPosition
                - footLink.modelPosition);
        const float comfortable = legLength * 0.94f;
        const auto needAt = [&](float horizontal, float vertical) {
            return horizontal >= comfortable ? vertical
                : vertical - std::sqrt(comfortable * comfortable
                    - horizontal * horizontal);
        };
        const auto needFor = [&](Vec3 ankle) {
            return needAt(std::hypot(hip.x - ankle.x, hip.y - ankle.y),
                hip.z - ankle.z);
        };
        const float need = needFor(goal.position
            + goal.orientation.rotate(anchorInFoot));
        const float authored = needFor(goal.animatedPosition
            + goal.animatedOrientation.rotate(anchorInFoot));
        drop = std::max(drop, std::max(0.0f, need - std::max(0.0f, authored))
            * goal.reach);
        if (goal.anticipate > 0.0f) {
            const Vec3 hipThen = hip + goal.hipTravel;
            const Vec3 ankle = goal.landingPosition
                + goal.landingOrientation.rotate(anchorInFoot);
            const float needThen = needAt(std::hypot(hipThen.x - ankle.x,
                hipThen.y - ankle.y), hipThen.z - ankle.z);
            const float authoredThen = needAt(goal.authoredLandingHorizontal,
                goal.authoredLandingVertical);
            drop = std::max(drop, std::max(0.0f,
                needThen - std::max(0.0f, authoredThen)) * goal.anticipate);
        }
        const float above = goal.pelvisGround - baseGround;
        if (goal.reach < 0.999f || above <= 0.0f) bothAbove = false;
        raise = std::min(raise, above);
    }
    float pelvisTarget = -drop;
    if (bothAbove && std::isfinite(raise)) pelvisTarget += raise;
    if (!onGround || !footworkEnabled) pelvisTarget = 0.0f;
    pelvisTarget = std::clamp(pelvisTarget, -0.35f, 0.25f);
    constexpr float PelvisOmega = 18.0f;
    m_pelvisOffsetVelocity += (PelvisOmega * PelvisOmega
            * (pelvisTarget - m_pelvisOffset)
        - 2.0f * PelvisOmega * m_pelvisOffsetVelocity) * deltaTime;
    m_pelvisOffset += m_pelvisOffsetVelocity * deltaTime;
    targetRootPosition.z += m_pelvisOffset;

    // Pernas por IK ate cada pe (posicao e orientacao), com a pelve final.
    // Mesmo com a raiz livre (forceDrivenRoot), o IK parte da pelve
    // DESEJADA: assim a perna de apoio empurra o corpo para onde ele deve
    // estar (o pe esta travado no chao) - e o musculo que sustenta. Pela
    // pelve medida, quando ela cedia o IK dobrava a perna junto e nada a
    // empurrava de volta (medido: parado sem ajuda, so em pe assim).
    // A perna que o planejador tem NO AR e outra coisa: ela precisa pousar o
    // pe no alvo do mundo, e as juntas dela partem da pelve de verdade. Pela
    // desejada (reta, na capsula), com o corpo inclinado 20-40 graus por um
    // empurrao o pe do passo de recuperacao ia girado junto e ficava no ar
    // "procurando o chao" ate o corpo cair. A troca e gradual (a perna nao
    // salta quando o passo comeca nem quando pousa).
    const PhysicsBodyState3D& measuredPelvis = state.links.front();
    for (std::size_t side = 0; side < m_feet.size(); ++side) {
        const FootPhase3D phase = m_footwork.plan().feet[side].phase;
        const bool swinging = input.forceDrivenRoot && footworkEnabled && m_footwork.owns(side)
            && (phase == FootPhase3D::Swing || phase == FootPhase3D::TouchdownSearch);
        float& weight = m_swingFromMeasuredPelvis[side];
        weight = swinging ? std::min(1.0f, weight + deltaTime / 0.06f)
                          : std::max(0.0f, weight - deltaTime / 0.15f);
    }
    // E5 (passos pelo planejador, andando): a capsula so segue o corpo e
    // fica para tras dele - as pernas de apoio resolvidas pela pelve
    // desejada (la atras) puxavam o corpo de volta e freavam a marcha. Andando
    // nesse modo, a posicao da pelve e a medida (no plano). A troca de base
    // e gradual (~0,25 s): parado, a referencia fica a ~5 cm da pelve fisica,
    // e trocar num tick saltava os alvos do quadril (0,06-0,09 rad) na
    // arrancada - o corpo ia para o lado errado antes do primeiro passo.
    {
        // Com a referencia corporal, na parada ela desacelera ate o repouso
        // e so entao devolve a base a referencia parada: trocando logo, a
        // base ia para a pelve de referencia (presa a capsula, que vem atras
        // do corpo) e as pernas puxavam o corpo de volta (+7 m/s2 recuando a
        // 0,9 m/s; ele passava do ponto e caia).
        const Vec3 com = m_estimator.state().centerOfMassVelocityWorld;
        const bool stillMoving = input.bodyReference && m_bodyRefValid
            && (m_bodyRefVelocity.length() > 0.05f || std::hypot(com.x, com.y) > 0.25f);
        // Com a referencia corporal ela e sempre a base das pernas de apoio,
        // andando ou parado: devolver a base a referencia parada (presa a
        // capsula, que so segue o corpo com folga) puxava o corpo na parada
        // (recuando: +4 a +6 m/s2 para a frente, passava do ponto e caia).
        const bool measuredBase = input.forceDrivenRoot && m_footwork.settings().locomotionStepping
            && (movementRequested || input.bodyReference);
        (void)stillMoving;
        m_legRootFromMeasured = measuredBase ? std::min(1.0f, m_legRootFromMeasured + deltaTime / 0.25f)
                                             : std::max(0.0f, m_legRootFromMeasured - deltaTime / 0.25f);
    }
    constexpr float LegBaseLeadSecondsSquared = 1.0f / 19.0f;
    constexpr float BodyRefAcceleration = 4.0f;
    constexpr float BodyRefLeashMeters = 0.10f;
    constexpr float BodyRefWalkMaxSpeed = 1.2f;
    constexpr float LegBaseLeadLimitMeters = 0.10f;
    // Alcance das pernas de apoio (R4): a extensao pedida comeca a ser cortada
    // com a perna a 90% do comprimento esticado e e toda cortada a 96%.
    constexpr float ReachComfortRatio = 0.80f;
    constexpr float ReachCriticalRatio = 0.90f;
    constexpr float BodyRefMaximumDrop = 0.08f;
    constexpr float BodyRefVerticalSpeed = 0.15f;
    Vec3 legRootPosition = targetRootPosition;
    if (m_legRootFromMeasured > 0.0f && input.bodyReference) {
        ++m_telemetry.bodyReferenceTicks;
        // Referencia corporal com estado: a velocidade vai ate a pedida com
        // aceleracao limitada, a posicao a integra, e a coleira a prende a
        // pelve medida (corpo atrasado ou bloqueado: nao acumula).
        const Vec3 pelvis { state.links.front().position.x, state.links.front().position.y, 0.0f };
        if (!m_bodyRefValid) {
            const Vec3 com = m_estimator.state().centerOfMassVelocityWorld;
            m_bodyRefPosition = pelvis;
            m_bodyRefVelocity = { com.x, com.y, 0.0f };
            m_bodyRefValid = true;
        }
        // A referencia pede o factivel: andando, a passada satura no alcance
        // (~0,86 m de percurso do pe) e o teto medido da marcha e ~1,1-1,2
        // m/s - acima disso o corpo passava da marcha (1,7 m/s pedindo 1,5)
        // e nao havia passo que o parasse. Mais rapido e corrida (E5-F). O
        // pedido do jogador nao muda.
        const Vec3 wanted = movementRequested
            ? clampMagnitude(Vec3 { input.intent.requestedVelocityWorld.x, input.intent.requestedVelocityWorld.y, 0.0f },
                BodyRefWalkMaxSpeed)
            : Vec3 {};
        m_bodyRefVelocity += clampMagnitude(wanted - m_bodyRefVelocity, BodyRefAcceleration * deltaTime);
        m_bodyRefPosition += m_bodyRefVelocity * deltaTime;
        const Vec3 error = m_bodyRefPosition - pelvis;
        if (error.length() > BodyRefLeashMeters)
            m_bodyRefPosition = pelvis + error * (BodyRefLeashMeters / error.length());
        // Transferencia de apoio (fase 4, do planejador): o centro de massa
        // vai a meta de apoio (o centro dos pes deslocado para o pe que
        // recebe - a mesma meta do esforco de contato). Na direcao da
        // transferencia a referencia de marcha e reancorada na pelve (senao,
        // parada, ela segurava o corpo: os dois se anulavam a 40% do caminho),
        // e a base ganha o que falta ate a meta menos a velocidade do centro de
        // massa (0,15 s) - zera na meta. (Como velocidade integrada, sem volta
        // de posicao, passava da meta, o outro pe saia do chao e ele caia.)
        Vec3 transferLead {};
        const auto& transfer = m_footwork.plan().supportTransfer;
        if (transfer.outgoingFoot >= 0) {
            const CharacterPhysicalState3D& physical = m_estimator.state();
            const Vec3 center = (physical.feet[0].soleCenterWorld + physical.feet[1].soleCenterWorld) * 0.5f;
            const Vec3 goal = center + (transfer.supportPointWorld - center) * transfer.progress;
            // A meta e do ponto de captura (DCM), nao do centro de massa:
            // andando o corpo ja vem com velocidade, e levar o COM a 85% do
            // caminho a cada passo balancava o corpo de lado (0,2 m/s) e
            // freava a marcha. O DCM inclui o amortecimento pela velocidade.
            const float omega = std::clamp(physical.captureOmega, 1.0f, 6.0f);
            const Vec3 dcm { physical.centerOfMassWorld.x + physical.centerOfMassVelocityWorld.x / omega,
                physical.centerOfMassWorld.y + physical.centerOfMassVelocityWorld.y / omega, 0.0f };
            Vec3 toGoal { goal.x - dcm.x, goal.y - dcm.y, 0.0f };
            // Andando, nunca contra o movimento pedido: para o pe da frente
            // sair (de lado, o do lado para onde vai) o apoio passa ao de tras
            // com o corpo acelerando para LONGE dele - no sentido da marcha -,
            // nao levando o corpo de volta ate ele. (Puxando para tras, a
            // transferencia brigava com a marcha e a descarga estourava o
            // prazo.)
            {
                const float wantedSpeed = std::hypot(wanted.x, wanted.y);
                if (wantedSpeed > 0.1f) {
                    const Vec3 forward = wanted * (1.0f / wantedSpeed);
                    const float backward = dot(toGoal, forward);
                    if (backward < 0.0f) toGoal -= forward * backward;
                }
            }
            if (toGoal.length() > 0.001f) {
                // Reancora a referencia de marcha na direcao da transferencia;
                // andando, so a parte atravessada ao movimento pedido (a
                // frente e da marcha).
                Vec3 along = toGoal * (1.0f / toGoal.length());
                const float wantedSpeed = std::hypot(wanted.x, wanted.y);
                if (wantedSpeed > 0.1f) {
                    const Vec3 forward = wanted * (1.0f / wantedSpeed);
                    along = along - forward * dot(along, forward);
                }
                if (along.length() > 0.01f) {
                    along = along * (1.0f / along.length());
                    m_bodyRefPosition -= along * dot(m_bodyRefPosition - pelvis, along);
                }
                transferLead = clampMagnitude(toGoal, BodyRefLeashMeters);
            }
        }
        legRootPosition.x += (m_bodyRefPosition.x + transferLead.x - legRootPosition.x) * m_legRootFromMeasured;
        legRootPosition.y += (m_bodyRefPosition.y + transferLead.y - legRootPosition.y) * m_legRootFromMeasured;
    } else if (m_legRootFromMeasured > 0.0f) {
        m_bodyRefValid = false;
        legRootPosition.x += (state.links.front().position.x - legRootPosition.x) * m_legRootFromMeasured;
        legRootPosition.y += (state.links.front().position.y - legRootPosition.y) * m_legRootFromMeasured;
        // A base prevista (guia 17): a pelve medida mais o que o controle do
        // corpo pediu de aceleracao ao centro de massa (antes do limite do
        // apoio: a queda passiva do pendulo ja acontece sozinha). Os motores das pernas
        // sao molas rigidas em volta do alvo; resolvida da pelve medida, a
        // perna so segurava o corpo onde ele estava - a troca de peso da
        // descarga e a velocidade pedida, so em torque de antecipacao, nao
        // moviam o corpo (medido: 170 N pedidos por 0,2 s, COM parado; a
        // mesma troca pelos alvos, 8 cm, levou o COM a 0,3 m/s). ~19 s^-2 e a
        // rigidez horizontal medida das pernas de apoio (0,7 Hz).
        Vec3 lead { input.legBaseAccelerationWorld.x, input.legBaseAccelerationWorld.y, 0.0f };
        lead = lead * LegBaseLeadSecondsSquared;
        if (lead.length() > LegBaseLeadLimitMeters) lead = lead * (LegBaseLeadLimitMeters / lead.length());
        legRootPosition += lead * m_legRootFromMeasured;
    } else {
        m_bodyRefValid = false;
    }
    // Alcance das pernas de apoio (R4 do pacote v2). A vantagem no plano da
    // referencia corporal, com a raiz do IK na mesma altura, alongava a
    // perna de tras (quadril a frente e acima do tornozelo): os motores a
    // esticavam ao longo do eixo, que e mais vertical que horizontal, e o
    // corpo subia 3,5 cm e saia do chao andando (voo de ate 67 ms). Aqui a
    // parte da extensao pedida a cada perna de apoio e cortada conforme ela
    // chega perto do alcance, e a raiz desce o que for preciso para o
    // comprimento ficar: a pelve gira em volta do tornozelo (o arco do
    // pendulo) em vez de a perna esticar. Com dois pes no chao vale a perna
    // que pede a raiz mais baixa. So o alvo das juntas muda; nada na pelve.
    const auto legGeometry = [&](std::size_t side, Vec3& hipLocal, Vec3& ankleLocal, float& nominal) {
        const FootPlant& plant = m_feet[side];
        const int shin = plant.valid ? profile.links[plant.linkIndex].parentIndex : -1;
        const int thigh = shin > 0 ? profile.links[static_cast<std::size_t>(shin)].parentIndex : -1;
        if (thigh <= 0) return false;
        const RagdollLinkDefinition3D& footLink = profile.links[plant.linkIndex];
        const Vec3 hipModel = profile.links[static_cast<std::size_t>(thigh)].inboundJoint.anchorModelPosition;
        const Vec3 kneeModel = profile.links[static_cast<std::size_t>(shin)].inboundJoint.anchorModelPosition;
        const Vec3 ankleModel = footLink.inboundJoint.anchorModelPosition;
        hipLocal = profile.links[0].modelOrientation.conjugate().rotate(hipModel - profile.links[0].modelPosition);
        ankleLocal = footLink.modelOrientation.conjugate().rotate(ankleModel - footLink.modelPosition);
        nominal = (kneeModel - hipModel).length() + (ankleModel - kneeModel).length();
        return nominal > 0.1f;
    };
    if (m_legRootFromMeasured > 0.0f && input.bodyReference && input.bodyReachProjection && m_bodyRefValid) {
        // A altura segue a da pelve de referencia a no maximo 0,15 m/s: da
        // base parada em alerta (0,80 m) para a de marcha (0,91) ela subia
        // em 0,3 s, a 0,5 m/s, e as pernas de apoio lancavam o corpo (voo de
        // 67 ms na arrancada).
        if (m_bodyRefHeight <= 0.0f) m_bodyRefHeight = legRootPosition.z;
        m_bodyRefHeight += std::clamp(legRootPosition.z - m_bodyRefHeight,
            -BodyRefVerticalSpeed * deltaTime, BodyRefVerticalSpeed * deltaTime);
        float heightLimit = m_bodyRefHeight;
        for (std::size_t side = 0; side < m_feet.size(); ++side) {
            LegReachTelemetry3D& reach = m_telemetry.legReach[side];
            reach.verticalCorrection = 0.0f;
            reach.projectionGain = 0.0f;
            Vec3 hipLocal, ankleLocal;
            float nominal = 0.0f;
            if (!goals[side].active || !m_feet[side].planted || !m_footwork.owns(side)
                || !legGeometry(side, hipLocal, ankleLocal, nominal)) continue;
            const Vec3 ankle = goals[side].position + goals[side].orientation.rotate(ankleLocal);
            const Vec3 hipOffset = targetRootOrientation.rotate(hipLocal);
            const Vec3 hip = Vec3 { legRootPosition.x, legRootPosition.y, heightLimit } + hipOffset;
            const float length = (hip - ankle).length();
            const float previous = m_previousLegLength[side][1];
            if (previous < 0.0f || length <= previous) continue;
            const float gain = smoothStep((length / nominal - ReachComfortRatio)
                / (ReachCriticalRatio - ReachComfortRatio));
            float allowed = previous + (length - previous) * (1.0f - gain);
            allowed = std::min(allowed, std::max(previous, ReachCriticalRatio * nominal));
            if (allowed >= length) continue;
            const float planar = std::hypot(hip.x - ankle.x, hip.y - ankle.y);
            if (planar >= allowed) continue;
            const float height = ankle.z - hipOffset.z + std::sqrt(allowed * allowed - planar * planar);
            reach.projectionGain = gain;
            if (height < heightLimit) {
                reach.verticalCorrection = heightLimit - height;
                heightLimit = height;
            }
        }
        // A restricao de alcance desce a altura na hora, nunca mais de 8 cm
        // abaixo da referencia (alem disso a perna esta no fim e o passo tem
        // de sair).
        m_bodyRefHeight = std::max(heightLimit, legRootPosition.z - BodyRefMaximumDrop);
        legRootPosition.z += (m_bodyRefHeight - legRootPosition.z) * m_legRootFromMeasured;
    } else {
        m_bodyRefHeight = 0.0f;
    }
    // Tornozelo contra a queda (pe com mola, modo fisico): o corpo saindo da
    // velocidade da referencia (empurrado, tropecando, passando do ponto)
    // leva o ponto de captura para fora. O pe de apoio pressiona do lado
    // para onde ele vai - a ponta caindo para a frente, o calcanhar para
    // tras, a borda de lado -: o alvo do pe gira em volta do tornozelo, o
    // chao nao deixa o pe girar e o motor empurra o corpo de volta (o centro
    // de pressao anda dentro do pe, a estrategia do tornozelo). A posicao ja
    // tem a mola das pernas de apoio (resolvidas da pelve de referencia);
    // isto antecipa pela velocidade. Fora da janela da mola do toque.
    {
        const CharacterPhysicalState3D& physical = m_estimator.state();
        Vec3 wanted {};
        if (input.footSpring && input.ankleBalance && input.forceDrivenRoot && !getUpActive && physical.valid) {
            const float omega = std::clamp(physical.captureOmega, 1.0f, 6.0f);
            wanted = { (physical.centerOfMassVelocityWorld.x - planarVelocity.x) / omega,
                (physical.centerOfMassVelocityWorld.y - planarVelocity.y) / omega, 0.0f };
        }
        m_ankleBalanceShift += (wanted - m_ankleBalanceShift) * (1.0f - std::exp(-deltaTime / 0.05f));
        // Zona morta: o balanco de quem esta parado (alguns cm/s) nao mexe o
        // tornozelo - reagindo a ele, o corpo parado oscilava (contato
        // perdido 0 -> 4 em 15 nascimentos).
        Vec3 pressShift = m_ankleBalanceShift;
        {
            const float size = pressShift.length();
            pressShift = size > AnkleBalanceDeadband
                ? pressShift * ((size - AnkleBalanceDeadband) / size) : Vec3 {};
        }
        m_telemetry.ankleBalancePitch = 0.0f;
        m_telemetry.ankleBalanceRoll = 0.0f;
        for (std::size_t side = 0; side < m_feet.size() && wanted.lengthSquared() > 0.0f; ++side) {
            FootGoal& goal = goals[side];
            const FootSupportEstimate3D& estimate = physical.feet[side];
            Vec3 hipLocal, ankleLocal;
            float nominal = 0.0f;
            if (!goal.active || !m_feet[side].planted || !estimate.supporting
                || !legGeometry(side, hipLocal, ankleLocal, nominal)) continue;
            const float engage = smoothStep((estimate.supportingSeconds - AnkleLandingSeconds) / 0.10f);
            Vec3 forward = goal.orientation.rotate({ 1.0f, 0.0f, 0.0f });
            forward.z = 0.0f;
            if (engage <= 0.0f || forward.lengthSquared() < 1e-4f) continue;
            forward = forward.normalized();
            const Vec3 left = cross(Vec3 { 0.0f, 0.0f, 1.0f }, forward);
            const float pitch = engage * std::clamp(AnkleBalanceGain * dot(pressShift, forward),
                -AnkleBalancePitchLimit, AnkleBalancePitchLimit);
            const float roll = engage * std::clamp(AnkleBalanceGain * dot(pressShift, left),
                -AnkleBalanceRollLimit, AnkleBalanceRollLimit);
            const Quaternion press = (Quaternion::fromAxisAngle(left, pitch)
                * Quaternion::fromAxisAngle(forward, -roll)).normalized();
            const Vec3 ankle = goal.position + goal.orientation.rotate(ankleLocal);
            goal.orientation = (press * goal.orientation).normalized();
            goal.position = ankle + press.rotate(goal.position - ankle);
            if (std::abs(pitch) > std::abs(m_telemetry.ankleBalancePitch)) m_telemetry.ankleBalancePitch = pitch;
            if (std::abs(roll) > std::abs(m_telemetry.ankleBalanceRoll)) m_telemetry.ankleBalanceRoll = roll;
        }
    }
    std::array<Vec3, 2> ikRootPosition { legRootPosition, legRootPosition };
    std::array<Quaternion, 2> ikRootOrientation { targetRootOrientation, targetRootOrientation };
    for (std::size_t side = 0; side < m_feet.size(); ++side) {
        const float measured = m_swingFromMeasuredPelvis[side];
        if (measured > 0.0f) {
            ikRootPosition[side] = legRootPosition
                + (measuredPelvis.position - legRootPosition) * measured;
            ikRootOrientation[side] = slerpQuaternion(targetRootOrientation,
                measuredPelvis.orientation, measured);
        }
        if (!goals[side].active) continue;
        const Quaternion rootInverse = ikRootOrientation[side].conjugate();
        solveFootPose(profile, m_coordinates, m_feet[side].linkIndex,
            rootInverse.rotate(goals[side].position - ikRootPosition[side]),
            (rootInverse * goals[side].orientation).normalized(), 1.0f,
            !gaitMoving);
    }
    // Alcance das pernas com o pe no chao (R3): quadril pela pelve (fisica
    // e do IK), tornozelo pelo pe (fisico e alvo).
    for (std::size_t side = 0; side < m_feet.size(); ++side) {
        LegReachTelemetry3D& reach = m_telemetry.legReach[side];
        const FootPlant& plant = m_feet[side];
        const bool planted = plant.valid && goals[side].active && plant.planted
            && plant.linkIndex < state.links.size();
        const int shin = planted ? profile.links[plant.linkIndex].parentIndex : -1;
        const int thigh = shin > 0 ? profile.links[static_cast<std::size_t>(shin)].parentIndex : -1;
        if (thigh <= 0) {
            reach = {};
            m_previousLegLength[side] = { -1.0f, -1.0f };
            continue;
        }
        const RagdollLinkDefinition3D& footLink = profile.links[plant.linkIndex];
        const Vec3 hipModel = profile.links[static_cast<std::size_t>(thigh)].inboundJoint.anchorModelPosition;
        const Vec3 kneeModel = profile.links[static_cast<std::size_t>(shin)].inboundJoint.anchorModelPosition;
        const Vec3 ankleModel = footLink.inboundJoint.anchorModelPosition;
        const Vec3 hipLocal = profile.links[0].modelOrientation.conjugate()
            .rotate(hipModel - profile.links[0].modelPosition);
        const Vec3 ankleLocal = footLink.modelOrientation.conjugate()
            .rotate(ankleModel - footLink.modelPosition);
        const float nominal = (kneeModel - hipModel).length() + (ankleModel - kneeModel).length();
        const PhysicsBodyState3D& footBody = state.links[plant.linkIndex];
        const Vec3 hipPhysical = measuredPelvis.position + measuredPelvis.orientation.rotate(hipLocal);
        const Vec3 anklePhysical = footBody.position + footBody.orientation.rotate(ankleLocal);
        const Vec3 hipReference = ikRootPosition[side] + ikRootOrientation[side].rotate(hipLocal);
        const Vec3 ankleReference = goals[side].position + goals[side].orientation.rotate(ankleLocal);
        const float physicalLength = (hipPhysical - anklePhysical).length();
        const float referenceLength = (hipReference - ankleReference).length();
        reach.valid = nominal > 0.1f;
        reach.nominalLengthMeters = nominal;
        reach.physicalRatio = reach.valid ? physicalLength / nominal : 0.0f;
        reach.referenceRatio = reach.valid ? referenceLength / nominal : 0.0f;
        reach.physicalAxialSpeed = m_previousLegLength[side][0] >= 0.0f
            ? (physicalLength - m_previousLegLength[side][0]) / deltaTime : 0.0f;
        reach.referenceAxialSpeed = m_previousLegLength[side][1] >= 0.0f
            ? (referenceLength - m_previousLegLength[side][1]) / deltaTime : 0.0f;
        reach.ikRootHeight = ikRootPosition[side].z;
        reach.physicalRootHeight = measuredPelvis.position.z;
        m_previousLegLength[side] = { physicalLength, referenceLength };
    }

    // Diagnostico E5-A: alvo e resultado do IK (antes do limitador).
    const auto footClearance = [&](std::size_t side, Vec3 position, Quaternion orientation, float ground) {
        return position.z - footLockHeight3D(profile.links[m_feet[side].linkIndex], ground, orientation);
    };
    for (std::size_t side = 0; side < m_feet.size(); ++side) {
        FootTrace3D& trace = m_telemetry.footTrace[side];
        trace = {};
        if (!m_footTraceEnabled || !goals[side].active || !m_feet[side].valid) continue;
        const std::size_t link = m_feet[side].linkIndex;
        const PhysicsBodyState3D& body = state.links[link];
        const GroundSample ground = groundUnder(side, body.position,
            forwardOf(m_feet[side], body.orientation));
        trace.groundHeight = ground.valid ? ground.height : 0.0f;
        trace.target = goals[side].position;
        trace.targetClearance = footClearance(side, goals[side].position, goals[side].orientation, trace.groundHeight);
        RagdollAnimationPose3D ikPose;
        buildLocalPose(profile, m_coordinates, ikPose);
        trace.ik = ikRootPosition[side] + ikRootOrientation[side].rotate(ikPose.linkPositions[link]);
        trace.ikClearance = footClearance(side, trace.ik,
            (ikRootOrientation[side] * ikPose.linkOrientations[link]).normalized(), trace.groundHeight);
        trace.physical = body.position;
        trace.ikRoot = ikRootPosition[side];
        trace.ikRootOrientation = ikRootOrientation[side];
        trace.physicalRoot = measuredPelvis.position;
        trace.physicalRootOrientation = measuredPelvis.orientation;
        trace.physicalClearance = footClearance(side, body.position, body.orientation, trace.groundHeight);
        trace.valid = true;
    }

    // Nothing this controller commands may ask a joint to move faster than
    // a body can. The motors are driven with a velocity target taken from
    // the per-tick change in these coordinates, so at 120 Hz any jump is
    // multiplied by 120: a pose-side discontinuity of a few degrees becomes
    // a limb travelling metres per second. Measured before this: feet
    // spiking past 4 m/s while flagged as planted, on a body turning at
    // 0.17 rad/s. Clamping the commanded change bounds that whole class of
    // glitch at the point where it enters the physics, whatever produced it
    // upstream - clip, procedural pose or the foot IK.
    //
    // O teto vale para o que NAO e o movimento do proprio clipe: IK de pe,
    // olhar e a mistura do crossfade. O que a amostra do clipe anda de um
    // tick para o outro passa por cima do teto - o joelho do sprint chega a
    // 20 rad/s por construcao, e cortar isso deformava o ciclo. A mistura
    // nao: medido, deixa-la passar levava o pe a 17 m/s na troca entre
    // recuar e andar para a frente, com a perna indo de uma pose a outra em
    // 0,14 s.
    constexpr float MaximumJointRateRadiansPerSecond = 12.0f;
    const float maximumJointStep =
        MaximumJointRateRadiansPerSecond * deltaTime;
    // A perna que o planejador tem no ar pode flexionar como a de uma
    // passada (o joelho do sprint chega a ~20 rad/s): a 12 rad/s o joelho que
    // saia esticado da descarga levava ~75 ms para dobrar e o pe subia tarde
    // e baixo (medido no passo lateral: 5 cm cortados pelo limitador). So o
    // joelho e o tornozelo: o quadril, com o pe a ~0,9 m, a 25 rad/s fazia a
    // perna do boneco empurrado chicotear (mediana 13,4 m/s no membro, contra
    // 9,5 com ele a 12).
    std::vector<bool> swingLeg(m_coordinates.size(), false);
    for (std::size_t side = 0; side < m_feet.size(); ++side) {
        const FootPlan3D& planFoot = m_footwork.plan().feet[side];
        if (!m_feet[side].valid || planFoot.followingGait || planFoot.phase != FootPhase3D::Swing) continue;
        int index = static_cast<int>(m_feet[side].linkIndex);
        for (int joint = 0; joint < 2 && index > 0; ++joint) {
            swingLeg[static_cast<std::size_t>(index)] = true;
            index = profile.links[static_cast<std::size_t>(index)].parentIndex;
        }
    }
    constexpr float SwingLegJointRateRadiansPerSecond = 25.0f;
    for (std::size_t index = 1; index < m_coordinates.size(); ++index) {
        const Vec3 previous = previousCoordinates[index];
        Vec3& current = m_coordinates[index];
        const float maximumJointStep = (swingLeg[index] ? SwingLegJointRateRadiansPerSecond
            : MaximumJointRateRadiansPerSecond) * deltaTime;
        for (std::size_t axis = 0; axis < 3; ++axis) {
            const float delta = component(current, axis)
                - component(previous, axis);
            const float clipStep = selectedClip == m_previousSampledClip
                ? std::abs(component(sampled[index], axis)
                    - component(m_previousClipCoordinates[index], axis))
                : 0.0f;
            const float allowed = maximumJointStep + clipStep;
            if (std::abs(delta) > allowed) {
                setComponent(current, axis, component(previous, axis)
                    + std::copysign(allowed, delta));
            }
        }
    }
    m_previousClipCoordinates = sampled;
    m_previousSampledClip = selectedClip;

    buildLocalPose(profile, m_coordinates, localPose);
    // Diagnostico E5-A: o comando final (depois do limitador), no referencial
    // do IK e a partir da pelve fisica.
    for (std::size_t side = 0; side < m_feet.size(); ++side) {
        FootTrace3D& trace = m_telemetry.footTrace[side];
        if (!trace.valid) continue;
        const std::size_t link = m_feet[side].linkIndex;
        trace.command = ikRootPosition[side] + ikRootOrientation[side].rotate(localPose.linkPositions[link]);
        trace.commandClearance = footClearance(side, trace.command,
            (ikRootOrientation[side] * localPose.linkOrientations[link]).normalized(), trace.groundHeight);
        trace.commandFromPhysicalPelvis = measuredPelvis.position
            + measuredPelvis.orientation.rotate(localPose.linkPositions[link]);
        trace.commandFromPhysicalPelvisClearance = footClearance(side, trace.commandFromPhysicalPelvis,
            (measuredPelvis.orientation * localPose.linkOrientations[link]).normalized(), trace.groundHeight);
    }

    // Quanto cada pe ficou longe do alvo depois do IK e do limitador: pe
    // travado que a perna nao alcanca (limite do quadril descendo escada)
    // solta no proximo tick.
    for (std::size_t side = 0; side < m_feet.size(); ++side) {
        FootPlant& foot = m_feet[side];
        if (!goals[side].active || !foot.valid) {
            foot.reachError = 0.0f;
            continue;
        }
        const Vec3 solved = ikRootPosition[side] + ikRootOrientation[side].rotate(
            localPose.linkPositions[foot.linkIndex]);
        foot.reachError = (solved - goals[side].position).length();
    }

    // Ground the authored pose before anything is driven from it. The root
    // height comes from the capsule plus the clip's own root track, and the
    // two do not have to agree: measured on the idle clip, the right sole
    // was being commanded 3.4 cm BELOW the floor. The floor answers that by
    // shoving the foot back up, and the leg above buckles to absorb it -
    // which is the crooked-legged stance, not weak motors. Raising the root
    // so the lowest authored sole rests exactly on the ground removes the
    // fight at its source; it never lowers the pose, so a genuine airborne
    // frame is untouched.
    if (!getUpActive && !input.forceDrivenRoot) {
        float deepest = 0.0f;
        for (std::size_t side = 0; side < m_feet.size(); ++side) {
            const FootPlant& foot = m_feet[side];
            if (!foot.valid || foot.linkIndex >= localPose.linkPositions.size()) {
                continue;
            }
            // Com os pes no terreno, so o pe de apoio conta: o pe em balanco
            // tem a propria folga sobre o chao (ver "Pes no terreno"). Contar
            // ele erguia o corpo inteiro quando passava rente a quina do
            // degrau seguinte, e o pe de apoio deixava de alcancar o dele
            // (medido: pelve 16 cm acima, pe de apoio pendurado 18 cm).
            if (footworkEnabled && onGround && !foot.planted) continue;
            // O chao alvo deste pe (terreno, ver "Pes no terreno"); fora do
            // chao, a base da capsula.
            const float floorZ = footworkEnabled && onGround
                ? foot.groundHeight : baseGround;
            const RagdollLinkDefinition3D& link = profile.links[foot.linkIndex];
            const Quaternion colliderWorld = (targetRootOrientation
                * localPose.linkOrientations[foot.linkIndex]
                * link.collider.localOrientation).normalized();
            const Vec3 colliderCentre = targetRootPosition
                + targetRootOrientation.rotate(
                    localPose.linkPositions[foot.linkIndex]
                    + localPose.linkOrientations[foot.linkIndex].rotate(
                        link.collider.localPosition));
            const float soleWorld = colliderCentre.z
                - lowestColliderOffset(link, colliderWorld);
            deepest = std::min(deepest, soleWorld - floorZ);
        }
        // Rede de seguranca, suavizada: sobe rapido e desce devagar. Aplicada
        // num tick so, qualquer diferenca entre a pose e o chao virava um
        // degrau na pelve (e o corpo inteiro tremia com ele).
        const float lift = std::max(0.0f, -deepest);
        const float response = lift > m_groundLift ? 40.0f : 10.0f;
        m_groundLift += (lift - m_groundLift)
            * (1.0f - std::exp(-response * deltaTime));
        targetRootPosition.z += m_groundLift;
    }

    m_output.targetPose = localPose;
    for (std::size_t index = 0; index < profile.links.size(); ++index) {
        m_output.targetPose.linkPositions[index] = targetRootPosition
            + targetRootOrientation.rotate(localPose.linkPositions[index]);
        m_output.targetPose.linkOrientations[index] =
            (targetRootOrientation * localPose.linkOrientations[index])
                .normalized();
    }

    const float muscle = std::clamp(input.muscleAuthority, 0.0f, 1.0f);
    // A real hit lets the joints actually give, not just the root guide
    // above: this is what makes falling possible at all instead of the
    // legs snapping back to the walking pose no matter how hard something
    // hits the character. Get-up phases carry their own tuned profile
    // below instead (including a near-limp Resting), so this only softens
    // normal locomotion.
    // Cede na pancada, mas nunca vira corpo morto: minimo de 35% (era 3%).
    // Com a raiz livre (modo fisico) as pancadas nao passam por aqui (a
    // cedencia e local, por junta, do controlador fisico) - so a PhysGun
    // ainda subia esta mistura, e agarrado o corpo inteiro ia a 35% da
    // forca (e assim ficava ~1,5 s depois de soltar): arrastado, a perna
    // nao erguia o pe e ele caia (varredura de arrasto: 20 de 32).
    const float impactSoftening = getUpActive || input.forceDrivenRoot ? 1.0f
        : std::clamp(1.0f - m_reactionStrength * 0.97f, 0.35f, 1.0f);
    float squaredError = 0.0f;
    std::uint32_t errorCount = 0;
    std::array<bool, 2> physicalSupport {};
    for (const auto& contact : state.interactions) {
        if (contact.normalWorld.z < 0.5f) continue;
        for (std::size_t side = 0; side < m_feet.size(); ++side)
            if (contact.linkIndex == m_feet[side].linkIndex) physicalSupport[side] = true;
    }
    std::array<float, 2> touchdownShare {};
    if (input.forceDrivenRoot && !getUpActive) {
        for (std::size_t side = 0; side < m_feet.size(); ++side) {
            const auto& foot = m_feet[side];
            if (!foot.valid || foot.linkIndex >= state.links.size()) continue;
            const auto& body = state.links[foot.linkIndex];
            const auto ground = groundUnder(side, body.position,
                forwardOf(foot, body.orientation));
            if (!ground.valid || body.linearVelocity.z > 0.15f) continue;
            // Um passo do planejador ainda no comeco do balanco nao esta
            // pousando: num passo de marcha baixo (8 cm) o corte valia o
            // balanco inteiro, a perna seguia so por posicao, atrasada, e o
            // pe subia metade do pedido e pousava curto.
            const FootPlan3D& planFoot = m_footwork.plan().feet[side];
            if (!planFoot.followingGait && planFoot.phase == FootPhase3D::Swing
                && planFoot.swingSeconds < 0.8f * planFoot.swingDurationSeconds) continue;
            // Passo de marcha (E5): a perna desce no tempo do plano ate o
            // contato - o amortecimento e da fase de carga, depois do toque.
            // Com o corte (velocidade-alvo zerada e mola 30% mais mole nos
            // ultimos 20%), no passo lateral o joelho estendia ~80 ms atrasado,
            // o pe ficava 3 cm acima com o alvo ja no chao e o pouso atrasava
            // 0,15 s procurando.
            if (!planFoot.followingGait && planFoot.reason == StepReason3D::Locomotion
                && (planFoot.phase == FootPhase3D::Swing || planFoot.phase == FootPhase3D::TouchdownSearch))
                continue;
            const float clearance = body.position.z
                - lockHeight(foot, ground.height, body.orientation);
            touchdownShare[side] = smoothStep((0.12f - clearance) / 0.12f);
        }
    }
    for (std::size_t index = 1; index < profile.links.size(); ++index) {
        m_coordinateVelocities[index] =
            (m_coordinates[index] - previousCoordinates[index]) / deltaTime;
        const Vec3 measured {
            state.joints[index].positionRadians[0],
            state.joints[index].positionRadians[1],
            state.joints[index].positionRadians[2]
        };
        const Vec3 velocity = jointAngularVelocity(m_coordinates[index],
            m_coordinateVelocities[index], measured);
        const auto& linkId = profile.links[index].id;
        const bool leg = linkId.find("Thigh") != std::string::npos
            || linkId.find("Shin") != std::string::npos
            || linkId.find("Foot") != std::string::npos;
        const std::size_t side = linkId.starts_with("Left") ? 0 : 1;
        // On a loaded foot, discontinuities in IK velocity become a kick
        // against the floor. Position drives retain support; damping absorbs
        // landing energy instead of feeding it back through the velocity goal.
        // (A perna em balanco do planejador que ainda raspa o chao no comeco
        // do passo, solta SEM carga, nao esta apoiada: sem a velocidade-alvo
        // ela seguia so por posicao, atrasada. Solta com carga - a soltura de
        // emergencia da recuperacao -, a velocidade seria o coice contra o
        // chao: no modo so pelas pernas, 26 -> 50 quedas.)
        const FootPlan3D& legPlan = m_footwork.plan().feet[side];
        const bool locomotionStep = legPlan.reason == StepReason3D::Locomotion
            && (legPlan.phase == FootPhase3D::Swing || legPlan.phase == FootPhase3D::TouchdownSearch);
        // (R2 do pacote v2: na marcha pelo planejador, quem deveria liberar a
        // velocidade do balanco e o pe no ar de fato - physicalRelease -, nao
        // a carga estimada na soltura. Medido antes de corrigir o voo curto:
        // misto, inversao 1 -> 3 quedas. Volta depois da R4.)
        const bool releasedForSwing = legPlan.releasedUnloaded;
        const bool plannedSwing = leg && !legPlan.followingGait && releasedForSwing
            && ((legPlan.phase == FootPhase3D::Swing
                    && legPlan.swingSeconds < 0.8f * legPlan.swingDurationSeconds)
                || locomotionStep);
        // Empurrao antes do toque (E5, fase 7): com o pe que vem no ultimo
        // terco do balanco (ou procurando o chao), a perna de apoio segue a
        // velocidade-alvo da referencia em vez de zero - com zero, o motor
        // amortecia o proprio pendulo e freava o corpo justo antes do pouso.
        // Fora dessa janela o apoio segue com zero (o H17 tirava o freio no
        // apoio inteiro e o corpo passava da velocidade pedida).
        // (So empurra com o corpo abaixo da velocidade da referencia no rumo
        // dela: ja na velocidade, empurrar sempre o levava alem do que a
        // marcha segura - 1 m/s: 2 -> 4 quedas; 1,5 m/s: 6 -> 10.)
        const FootPlan3D& otherPlan = m_footwork.plan().feet[1 - side];
        bool belowReference = false;
        if (m_bodyRefValid) {
            const float referenceSpeed = std::hypot(m_bodyRefVelocity.x, m_bodyRefVelocity.y);
            const Vec3 com = m_estimator.state().centerOfMassVelocityWorld;
            belowReference = referenceSpeed > 0.1f
                && (com.x * m_bodyRefVelocity.x + com.y * m_bodyRefVelocity.y) / referenceSpeed < referenceSpeed;
        }
        const bool propulsive = leg && belowReference && m_footwork.settings().locomotionStepping && !legPlan.followingGait
            && legPlan.phase == FootPhase3D::Stance && otherPlan.reason == StepReason3D::Locomotion
            && (otherPlan.phase == FootPhase3D::TouchdownSearch
                || (otherPlan.phase == FootPhase3D::Swing
                    && otherPlan.swingSeconds >= LocomotionPushWindow * otherPlan.swingDurationSeconds));
        // Pe com mola: o tornozelo de um pe no ar (sem contato nenhum) so
        // carrega o proprio pe (~1 kg) - nada a amortecer contra o chao. Ele
        // segue o alvo (posicao e velocidade) ate o toque: com a perna toda
        // amolecida perto do chao e a velocidade-alvo cortada, o tornozelo
        // virava um freio de ~0,3 s e chegava 13-23 graus atrasado no sprint
        // (o pe nao pousava como a animacao e o terreno pediam).
        const FootSupportEstimate3D& footEstimate = m_estimator.state().feet[side];
        const bool ankle = linkId.find("Foot") != std::string::npos;
        const bool footInAir = input.footSpring && input.forceDrivenRoot && !getUpActive
            && !footEstimate.contactObserved && !footEstimate.supporting;
        const bool agileAnkle = footInAir && ankle && gaitMoving;
        // O joelho tambem: amolecido e sem velocidade-alvo nos ultimos 12 cm,
        // ele chegava ao toque 17-23 graus mais dobrado que o alvo - a canela
        // inclinada para tras e o pe pousando com a ponta para baixo mesmo
        // com o tornozelo certo. Ele so cede depois do contato (aceitacao de
        // peso, pouso forte).
        // (So na passada: caindo de um pulo o joelho chega macio - rigido,
        // o pouso quicava, rebote 1,07 -> 1,31 m/s.)
        const bool agileKnee = footInAir && gaitMoving && linkId.find("Shin") != std::string::npos;
        const float velocityShare = input.forceDrivenRoot && !getUpActive && leg
            ? (agileAnkle || agileKnee ? transition
                : physicalSupport[side] && !plannedSwing && !propulsive ? 0.0f
                : transition * (propulsive ? 1.0f : 1.0f - touchdownShare[side])) : 1.0f;
        // Same per-category get-up gains as
        // BiomechanicalBipedExperiment3D's: soft and passive while
        // Resting, firm (legs firmest) while actively recovering.
        // Default mode holds the pose as hard as the drives allow, and lets
        // go in step with physicsBlend. This is the "force the pose when
        // nothing is happening, yield when something is" dial: at blend 0
        // the motors run at their ceiling, at blend 1 they are back to the
        // profile's nominal strength and the body argues back.
        float poseHold = 1.0f + (1.0f - m_physicsBlend);
        // No modo fisico o corpo e sempre o fisico: sem "segurar a pose" no
        // dobro da forca (herdado do modo antigo). Mais macio, ele sente as
        // pancadas (esbarrar numa parede, num boneco) em vez de rebater duro.
        if (input.forceDrivenRoot) poseHold = UprightMuscleTone;
        // Caindo: tonus de quem se protege - firme o bastante para os
        // reflexos agirem, sem a rigidez de quem esta de pe. Bracos e
        // pescoco rapidos (as maos tem de chegar ao chao antes da cabeca).
        {
            const std::string& id = profile.links[index].id;
            const bool reflexLimb = id.find("Arm") != std::string::npos
                || id.find("Forearm") != std::string::npos
                || id == "Neck" || id == "Head";
            // Tronco e pernas amolecem so com a queda sem volta; os bracos
            // e o pescoco ja com o reflexo.
            poseHold += ((reflexLimb ? FallReflexLimbTone : FallBodyTone)
                - poseHold) * (reflexLimb ? m_fallReflex : m_fallCommit);
        }
        float stiffnessMultiplier = poseHold;
        float dampingMultiplier = std::sqrt(poseHold);
        float torqueMultiplier = poseHold;
        // Caindo, freio baixo tambem: a raiz quadrada do tonus deixava metade
        // do freio de pe e o corpo caia "posado", devagar nas juntas.
        {
            const std::string& id = profile.links[index].id;
            const bool reflexLimb = id.find("Arm") != std::string::npos
                || id.find("Forearm") != std::string::npos
                || id == "Neck" || id == "Head";
            const float falling = reflexLimb ? m_fallReflex : m_fallCommit;
            dampingMultiplier += ((reflexLimb ? 0.55f : 0.15f)
                - dampingMultiplier) * falling;
        }
        if (input.forceDrivenRoot && !getUpActive && leg) {
            // Changing a stride can ask for an incompatible extension on a
            // loaded leg. Let it yield while the new support is established.
            const float transfer = 0.35f + 0.65f * transition;
            if (agileAnkle) {
                // No ar: mola cheia e menos freio (ver agileAnkle).
                dampingMultiplier *= AnkleAirDamping;
            } else if (agileKnee) {
                // No ar: a mola da troca de apoio, sem o amolecer antes do toque.
                stiffnessMultiplier *= transfer * 0.55f;
                torqueMultiplier *= transfer * 0.75f;
            } else {
                stiffnessMultiplier *= transfer * 0.55f;
                torqueMultiplier *= transfer * 0.75f;
                // Prepare compliance before impact, while the descending sole
                // approaches its measured surface. Retain damping through touch.
                stiffnessMultiplier *= 1.0f - 0.30f * touchdownShare[side];
                dampingMultiplier *= 1.0f + 0.30f * touchdownShare[side];
            }
            // O tornozelo apoiado continua sendo uma mola, nao uma junta
            // soldada: cede para a sola adaptar-se e devolve energia na
            // rolagem ate o antepe. O equilibrio ainda pode modular o alvo;
            // reduzimos a rigidez/torque, sem soltar o controle do pe.
            if (input.footSpring && ankle && !agileAnkle) {
                stiffnessMultiplier *= 0.82f;
                dampingMultiplier *= 0.92f;
                torqueMultiplier *= 0.86f;
            }
            // Mola do pe no toque: o tornozelo cede e amortece enquanto o peso
            // chega (o pe assenta no chao, o corpo nao bate na perna rigida)
            // e volta a firmar em ~0,15 s, a tempo do equilibrio e do impulso.
            if (input.footSpring && ankle && footEstimate.supporting
                && footEstimate.supportingSeconds < AnkleLandingSeconds) {
                const float settle = smoothStep(footEstimate.supportingSeconds / AnkleLandingSeconds);
                stiffnessMultiplier *= AnkleLandingStiffness + (1.0f - AnkleLandingStiffness) * settle;
                dampingMultiplier *= 1.0f + AnkleLandingDamping * (1.0f - settle);
            }
            // Pouso forte: a perna amortece (menos mola, mais freio) enquanto
            // a pelve desce, e volta a firmar ao subir.
            if (m_sinceTouchdownSeconds < m_landingSettleSeconds) {
                const float settle = std::clamp(m_sinceTouchdownSeconds
                    / m_landingSettleSeconds, 0.0f, 1.0f);
                const float absorb = smoothStep((m_landingImpactSpeed - 1.5f) / 2.5f)
                    * (1.0f - smoothStep((settle - 0.40f) / 0.60f));
                stiffnessMultiplier *= 1.0f - 0.35f * absorb;
                dampingMultiplier *= 1.0f + 0.80f * absorb;
            }
            // Aceitacao de peso (E5, fase 9): a perna que acabou de pousar
            // num passo de marcha e 25% mais mole durante a carga e os
            // primeiros 0,15 s de apoio - rigida, ela entrava como escora e
            // freava o corpo no toque. Grade medida (1 m/s, 16 corridas,
            // rigidez x amortecimento): 1,0 x 1,0 perdia 0,085 m/s na colisao
            // e 0,116 ate a saida do outro pe; 0,75 x 1,0, 0,046 e 0,069, mesmas
            // quedas, regime 76 -> 80%; mais mole (0,45-0,60) caia mais.
            if (m_footwork.settings().acceptanceCompliance
                && !legPlan.followingGait && legPlan.reason == StepReason3D::Locomotion
                && (legPlan.phase == FootPhase3D::Loading
                    || (legPlan.phase == FootPhase3D::Stance
                        && m_estimator.state().feet[side].supportingSeconds < 0.15f)))
                stiffnessMultiplier *= LocomotionAcceptanceStiffness;
        }
        if (getUpActive) {
            const std::string& id = profile.links[index].id;
            const bool isArm = id.find("Arm") != std::string::npos
                || id.find("Forearm") != std::string::npos
                || id.find("Hand") != std::string::npos;
            const bool isTorso = id == "Abdomen" || id == "Chest"
                || id == "UpperChest" || id == "Neck" || id == "Head";
            // Deitado com tonus: cabeca e tronco firmes (a cabeca nao bate
            // solta), bracos e pernas moderados - vezes o tonus do momento
            // (cede rolando, volta antes de levantar). O amortecimento fica:
            // cedendo, a junta ainda freia o movimento (nao e corpo morto).
            const float lyingStiffness = (isTorso ? 0.70f : isArm ? 0.50f : 0.55f)
                * m_settleTone;
            const float lyingTorque = (isTorso ? 0.90f : isArm ? 0.65f : 0.75f)
                * std::max(0.5f, m_settleTone);
            if (m_getUpPhase == CharacterGetUpPhase3D::Settling) {
                // Largado, quase sem freio nem mola: o alvo persegue o corpo,
                // entao mola x atraso + amortecimento viram um freio viscoso.
                // Com ~5 N.m.s/rad um antebraco quase em pe (peso ~1,5 kg)
                // levava ~3 s para tombar; num ragdoll, ~0,3 s. Fica o teto
                // baixo de torque - o "pouquinho de forca". Se arrumando, tudo
                // volta ao nominal com o tonus.
                const float rest = 1.0f - m_settleGather;
                stiffnessMultiplier = lyingStiffness * (1.0f - 0.75f * rest);
                dampingMultiplier = 1.0f - 0.97f * rest;
                torqueMultiplier = lyingTorque * (1.0f - 0.60f * rest);
            } else {
                // Levantar: os bracos sao o apoio (a flexao de bracos de quem
                // levanta de bruços). Entra a partir do tonus deitado em
                // ~0,5 s, nao de uma vez.
                const float rising = smoothStep(m_getUpPhaseSeconds / 0.5f);
                stiffnessMultiplier = lyingStiffness + ((isArm ? RisingArmStiffness
                    : isTorso ? RisingTorsoStiffness : RisingLegStiffness)
                    - lyingStiffness) * rising;
                dampingMultiplier = 1.0f + ((isArm ? 1.45f : isTorso ? 1.25f : 1.30f)
                    - 1.0f) * rising;
                torqueMultiplier = lyingTorque + ((isArm ? RisingArmTorque
                    : isTorso ? RisingTorsoTorque : RisingLegTorque)
                    - lyingTorque) * rising;
            }
        }
        // Largado, os membros pesam. Com a gravidade compensada inteira eles
        // ficavam onde estavam, sem peso: o corpo caia e deitava "posado", de
        // costas com as maos a 25-37 cm do chao. Um resto sustenta (tonus);
        // se arrumando para levantar, a compensacao volta com o tonus.
        float gravityScale = 1.0f;
        {
            const std::string& id = profile.links[index].id;
            const bool reflexLimb = id.find("Arm") != std::string::npos
                || id.find("Forearm") != std::string::npos
                || id.find("Hand") != std::string::npos
                || id == "Neck" || id == "Head";
            if (m_getUpPhase == CharacterGetUpPhase3D::Settling) {
                gravityScale = LyingGravity + (1.0f - LyingGravity) * m_settleGather;
            } else if (m_getUpPhase == CharacterGetUpPhase3D::Rising) {
                const float from = LyingGravity + (1.0f - LyingGravity) * m_settleGather;
                gravityScale = from + (1.0f - from)
                    * smoothStep(m_getUpPhaseSeconds / 0.5f);
            } else if (reflexLimb) {
                gravityScale = 1.0f - (1.0f - FallLimbGravity) * m_fallReflex;
            } else {
                gravityScale = 1.0f - (1.0f - FallBodyGravity) * m_fallCommit;
            }
        }
        for (std::size_t axis = 0; axis < 3; ++axis) {
            if (!profile.links[index].inboundJoint.axes[axis].enabled) continue;
            RagdollDriveTarget3D target;
            target.linkIndex = static_cast<std::uint32_t>(index);
            target.axis = static_cast<RagdollAxis3D>(axis);
            target.positionRadians = component(m_coordinates[index], axis);
            // Folga acima do joelho do sprint (~21 rad/s de pico).
            const float recoveryVelocityShare = m_getUpPhase == CharacterGetUpPhase3D::Settling
                ? 0.0f : m_getUpPhase == CharacterGetUpPhase3D::Rising
                    ? smoothStep(m_getUpPhaseSeconds / 0.5f) : 1.0f;
            const float velocityLimit = getUpActive ? 6.0f : 30.0f;
            target.velocityRadiansPerSecond = std::clamp(
                component(velocity, axis) * velocityShare * recoveryVelocityShare,
                -velocityLimit, velocityLimit);
            // Modo fisico: a junta perto de uma pancada cede (controlador
            // fisico, por link).
            // Caido ou levantando, o tonus e o de cima (deitado/levantar).
            const float yielding = !getUpActive
                    && index < input.jointStrength.size()
                ? std::clamp(input.jointStrength[index], 0.05f, 1.0f) : 1.0f;
            target.stiffnessScale =
                muscle * stiffnessMultiplier * impactSoftening * yielding;
            target.dampingScale =
                std::sqrt(muscle) * dampingMultiplier;
            target.maximumTorqueScale =
                muscle * torqueMultiplier * impactSoftening * yielding;
            target.gravityCompensationScale = gravityScale;
            if (input.manipulated && index == input.grabbedLink) {
                target.stiffnessScale *= 0.20f;
                target.dampingScale *= 0.45f;
                target.maximumTorqueScale *= 0.20f;
            }
            m_output.driveTargets.push_back(target);
            const float error = target.positionRadians
                - component(measured, axis);
            squaredError += error * error;
            ++errorCount;
        }
    }

    // The joints are never written into the articulation: pose authority is
    // what makes the guide overwrite joint positions every tick, which is
    // both the visible "bones teleporting into place" when the body turns
    // and the reason limbs pass through walls - a written position never
    // meets the solver. The motors below do the work instead, so the body
    // tracks the animation AND still collides.
    //
    // What stays carried is the root: it follows the capsule, which is what
    // keeps the character upright and moving where gameplay says. Leaving
    // root rotation to physics is what made the spine lean.
    //
    // The residual on the joints is drift correction, not posing: a blend
    // toward the target each tick, never the hard assignment the guide does
    // at authority 1. Measured tracking error against the authored pose:
    // 9.7 deg RMS at 0.10, 6.1 at 0.35, 5.4 at 0.60 - the curve is flat past
    // 0.35, so this sits just high enough to hold the pose and no higher,
    // leaving contact to win. Raising the motor gains to their ceiling moved
    // it only 9.7 -> 9.0, so this is not a strength problem.
    //
    // Contact already relaxes it further on its own: the guide is staged
    // with releaseOnInteraction, and the PhysX side caps every authority by
    // 1 - 0.9*externalInterference, so touching a wall hands that limb back
    // to the solver instead of writing it through the wall.
    RagdollAnimationConstraint3D& guide = m_output.guide;
    // Never. Every joint the guide writes is a joint the solver did not
    // compute: it shivers, it ignores contact, and during a recovery - when
    // the root is free - writing joints is what threw the body across the
    // map. The motors below are the only thing that moves this character.
    guide.poseAuthority = 0.0f;
    guide.rootTranslationAuthority = rootAuthority;
    guide.rootRotationAuthority = rootAuthority;
    guide.releaseOnInteraction = true;
    guide.rootPositionWorld = targetRootPosition;
    guide.rootOrientationWorld = targetRootOrientation;
    const Vec3 finiteDifferenceVelocity =
        (targetRootPosition - m_previousRootPosition) / deltaTime;
    guide.rootLinearVelocityWorld = finite(finiteDifferenceVelocity)
        ? finiteDifferenceVelocity : input.rootVelocityWorld;
    const Vec3 targetAngularVelocity = rotationVector(targetRootOrientation
        * m_previousRootOrientation.conjugate()) / deltaTime;
    guide.rootAngularVelocityWorld = targetAngularVelocity;
    if (referenceRebase) {
        guide.rootLinearVelocityWorld = m_previousGuideVelocity;
        guide.rootAngularVelocityWorld = m_previousGuideAngularVelocity;
    }
    m_previousGuideVelocity = guide.rootLinearVelocityWorld;
    m_previousGuideAngularVelocity = guide.rootAngularVelocityWorld;
    m_telemetry.getUpTiltExcessDegrees = 0.0f;
    m_telemetry.getUpUprightTorque = 0.0f;
    if (getUpActive && !state.links.empty()) {
        m_telemetry.getUpAssistNewtons = 0.0f;
        if (m_getUpPhase == CharacterGetUpPhase3D::Rising) {
            // Ajuda discreta do levantar: quem levanta e o corpo (os motores
            // levando as juntas ao clipe, maos/joelhos/pes empurrando o
            // chao). A pelve so recebe o que falta para acompanhar o clipe -
            // altura e endireitar -, suave, pequena (30% do peso; ate 65% se
            // o corpo emperra) e so com apoio real no chao: sem mao, joelho
            // ou pe tocando, nada (nao levita).
            const float mass = std::max(1.0f, profile.totalMassKg);
            const auto& pelvis = state.links.front();
            const float ground = groundHeightGuess;
            const float expectedHeight = profile.standingRootHeightMeters
                + rootTranslation.z;
            const Vec3 clipUp = rootRotation.rotate({ 0.0f, 0.0f, 1.0f });
            const float expectedTilt = std::acos(std::clamp(clipUp.z, -1.0f, 1.0f));
            const float expectedRise = m_getUpExpectedValid
                ? (expectedHeight - m_getUpExpectedHeight) / deltaTime : 0.0f;
            m_getUpExpectedHeight = expectedHeight;
            m_getUpExpectedTilt = expectedTilt;
            m_getUpExpectedValid = true;
            bool support = false;
            for (const RagdollInteraction3D& contact : state.interactions) {
                if (contact.normalWorld.z < 0.5f
                    || contact.linkIndex >= profile.links.size()) continue;
                const std::string& id = profile.links[contact.linkIndex].id;
                if (id.find("Hand") != std::string::npos
                    || id.find("Forearm") != std::string::npos
                    || id.find("Shin") != std::string::npos
                    || id.find("Foot") != std::string::npos) {
                    support = true;
                }
            }
            m_getUpSupportAge = support ? 0.0f
                : std::min(5.0f, m_getUpSupportAge + deltaTime);
            // So com o centro de massa sobre os apoios de verdade (maos,
            // joelhos, pes no chao): fora deles (> ~8 cm) a ajuda de altura e
            // de endireitar some - com ela o corpo ficava "segurado" como uma
            // prancha, o centro de massa ate 0,6 m fora de qualquer apoio.
            float overSupport = 1.0f;
            {
                std::vector<std::array<float, 2>> points;
                for (const RagdollInteraction3D& contact : state.interactions) {
                    if (contact.normalWorld.z < 0.5f) continue;
                    points.push_back({ contact.positionWorld.x,
                        contact.positionWorld.y });
                }
                if (!points.empty()) {
                    overSupport = 1.0f - smoothStep((planarDistanceOutsideHull3D(points,
                        centerOfMass.x, centerOfMass.y) - 0.08f) / 0.20f);
                }
            }
            m_telemetry.getUpOverSupport = overSupport;
            const float contactShare = 1.0f
                - smoothStep((m_getUpSupportAge - 0.15f) / 0.25f);
            // (overSupport so medido: cortar a ajuda por ele fazia o levantar
            // falhar - os pontos de contato do backend sao o ponto dominante
            // de cada segmento, a base medida fica menor que a real.)
            (void)overSupport;
            const float supportShare = contactShare;
            const float height = pelvis.position.z - ground;
            constexpr float LiftOmega = 6.0f;
            // Cada tentativa que falha, um pouco mais de esforco (ate o
            // dobro na terceira).
            // A ajuda entra aos poucos (~1 s): o corpo comeca sozinho e ela
            // vai chegando, em vez de pegar o corpo no primeiro tick.
            m_getUpAssistRamp = smoothStep(m_getUpPhaseSeconds / 1.0f);
            const float effort = std::min(2.0f, 1.0f
                + 0.5f * static_cast<float>(m_getUpAttempt > 0 ? m_getUpAttempt - 1 : 0))
                * m_getUpAssistRamp;
            const float liftCap = 9.81f * (0.30f + 0.35f * m_getUpStruggle) * effort;
            const float lift = std::clamp(
                LiftOmega * LiftOmega * (expectedHeight - height)
                    + 2.0f * LiftOmega * (expectedRise - pelvis.linearVelocity.z),
                0.0f, liftCap) * supportShare;
            // Endireitar: gira a pelve no sentido de ficar em pe, ate a
            // inclinacao que o clipe tem agora.
            const Vec3 up = pelvis.orientation.rotate({ 0.0f, 0.0f, 1.0f });
            const float tilt = std::acos(std::clamp(up.z, -1.0f, 1.0f));
            Vec3 axis = cross(up, { 0.0f, 0.0f, 1.0f });
            Vec3 torque {};
            if (axis.lengthSquared() > 1e-6f) {
                axis = axis.normalized();
                const float excess = std::max(0.0f, tilt - expectedTilt);
                const float spin = dot(pelvis.angularVelocity, axis);
                // Fase final (o clipe ja quase em pe, a pelve alta): mais forca
                // para endireitar - como um par pelve x pes, e a extensao do
                // quadril que traz o tronco para cima e o peso para tras, sobre
                // os pes. Com o teto de antes (~130 N.m) ele ficava ~1 s como
                // uma prancha 20-27 graus alem do clipe, o centro de massa
                // 0,4-0,5 m a frente dos pes, segurado pela ajuda.
                const float late = smoothStep((0.8f - expectedTilt) / 0.4f)
                    * smoothStep((height - 0.55f) / 0.15f);
                const float cap = mass * (1.5f + 1.5f * m_getUpStruggle
                    + 3.0f * late) * effort;
                torque = axis * std::clamp((excess * 22.0f - spin * 4.5f) * mass,
                    -cap, cap) * supportShare;
                m_telemetry.getUpTiltExcessDegrees = excess * 57.2958f;
            }
            // O peito tambem: a altura dele no clipe agora (cinematica direta
            // da pose do clipe), com o mesmo teto dividido.
            float chestLift = 0.0f;
            if (m_chestLink < state.links.size()) {
                RagdollAnimationPose3D clipPose;
                buildLocalPose(profile, sampled, clipPose);
                const float expectedChest = profile.standingRootHeightMeters
                    + rootTranslation.z
                    + rootRotation.rotate(clipPose.linkPositions[m_chestLink]).z;
                const float chest = state.links[m_chestLink].position.z - ground;
                m_getUpExpectedChestHeight = expectedChest;
                // E a inclinacao do tronco: de bruços o clipe ergue o tronco
                // (cabeca para cima, maos no chao); sem isso o tronco
                // apontava para o chao e o corpo se encolhia de cabeca baixa.
                {
                    const Quaternion clipChest = (rootRotation
                        * clipPose.linkOrientations[m_chestLink]).normalized();
                    const Vec3 clipChestUp = clipChest.rotate({ 0.0f, 0.0f, 1.0f });
                    const float expectedChestTilt = std::acos(
                        std::clamp(clipChestUp.z, -1.0f, 1.0f));
                    const auto& chestBody = state.links[m_chestLink];
                    const Vec3 chestUp = chestBody.orientation.rotate({ 0.0f, 0.0f, 1.0f });
                    Vec3 chestAxis = cross(chestUp, { 0.0f, 0.0f, 1.0f });
                    if (chestAxis.lengthSquared() > 1e-6f) {
                        chestAxis = chestAxis.normalized();
                        const float excess = std::max(0.0f, std::acos(
                            std::clamp(chestUp.z, -1.0f, 1.0f)) - expectedChestTilt);
                        const float spin = dot(chestBody.angularVelocity, chestAxis);
                        const float cap = mass * (0.8f + 0.8f * m_getUpStruggle) * effort;
                        m_output.chestAssistTorqueWorld = chestAxis * std::clamp(
                            (excess * 14.0f - spin * 3.0f) * mass, -cap, cap) * supportShare;
                    }
                }
                chestLift = std::clamp(
                    LiftOmega * LiftOmega * (expectedChest - chest)
                        + 2.0f * LiftOmega * (expectedRise
                            - state.links[m_chestLink].linearVelocity.z),
                    0.0f, liftCap) * supportShare;
                m_output.chestAssistLink = static_cast<std::uint32_t>(m_chestLink);
            }
            // O teto vale para a soma: pelve e peito dividem a mesma ajuda.
            const float total = lift + chestLift;
            const float scale = total > liftCap && total > 0.0f ? liftCap / total : 1.0f;
            // Erguendo-se (o clipe ja quase em pe): o centro de massa vai para
            // cima dos pes que estao no chao, como quem se equilibra ao
            // levantar - de leve, ate 15% do peso.
            Vec3 balance {};
            if (expectedTilt < 0.8f) {
                Vec3 feet {};
                int touching = 0;
                for (const RagdollInteraction3D& contact : state.interactions) {
                    if (contact.normalWorld.z < 0.5f
                        || contact.linkIndex >= profile.links.size()) continue;
                    if (profile.links[contact.linkIndex].id.find("Foot")
                        == std::string::npos) continue;
                    feet += state.links[contact.linkIndex].position;
                    ++touching;
                }
                if (touching > 0) {
                    feet *= 1.0f / static_cast<float>(touching);
                    Vec3 offset { feet.x - centerOfMass.x, feet.y - centerOfMass.y, 0.0f };
                    Vec3 velocity = m_filteredCenterOfMassVelocity;
                    velocity.z = 0.0f;
                    // (Esta continua fora da base: e ela que traz o centro de
                    // massa de volta para cima dos pes.)
                    balance = clampMagnitude(offset * 30.0f - velocity * 8.0f,
                        9.81f * 0.22f * effort) * (mass * contactShare);
                }
            }
            m_output.rootControlForceWorld = Vec3 { balance.x, balance.y,
                lift * scale * mass };
            m_output.chestAssistForceWorld = { 0.0f, 0.0f, chestLift * scale * mass };
            // Endireitar e um par interno, como um musculo: a pelve (e o
            // peito) giram para cima e a reacao vai para os segmentos que
            // estao apoiados no chao, pela forca de apoio de cada um. Assim
            // ele so sustenta o que os apoios sustentariam - com o centro de
            // massa fora deles, e o pe ou a mao que cede. Como torque externo
            // ele segurava o corpo como uma prancha inclinada ~35 graus: a
            // ajuda ficava evidente.
            {
                std::vector<std::pair<std::uint32_t, float>> supports;
                float supportSum = 0.0f;
                for (const RagdollInteraction3D& contact : state.interactions) {
                    if (contact.normalWorld.z < 0.5f || contact.linkIndex == 0
                        || contact.linkIndex == m_chestLink
                        || contact.linkIndex >= profile.links.size()) continue;
                    const float weight = std::max(0.001f,
                        contact.normalImpulseNewtonSeconds);
                    supportSum += weight;
                    auto existing = std::find_if(supports.begin(), supports.end(),
                        [&](const auto& item) { return item.first == contact.linkIndex; });
                    if (existing != supports.end()) existing->second += weight;
                    else supports.push_back({ contact.linkIndex, weight });
                }
                if (supportSum <= 0.0f) {
                    torque = {};
                    m_output.chestAssistTorqueWorld = {};
                } else {
                    const Vec3 reaction = -(torque + m_output.chestAssistTorqueWorld);
                    for (const auto& [link, weight] : supports) {
                        m_output.assistReactionLinks.push_back(link);
                        m_output.assistReactionTorquesWorld.push_back(
                            reaction * (weight / supportSum));
                    }
                }
            }
            m_output.rootControlTorqueWorld = torque;
            m_telemetry.getUpUprightTorque = torque.length();
            m_telemetry.getUpAssistNewtons = total * scale * mass;
        }
    } else if (!state.links.empty()) {
        // One path now: the body is always the physical one, so it always
        // gets a root controller. How much help it gets is the only thing
        // that changes, and that is what physicsBlend fades.
        Quaternion rotationError = targetRootOrientation
            * state.links.front().orientation.conjugate();
        const Vec3 orientationError = rotationVector(rotationError);
        const Vec3 angularVelocityError = targetAngularVelocity
            - state.links.front().angularVelocity;
        const float mass = std::max(1.0f, profile.totalMassKg);
        const Vec3 torque = orientationError * (mass * 10.0f)
            + angularVelocityError * (mass * 1.45f);
        m_output.rootControlTorqueWorld = clampMagnitude(
            torque, mass * 13.0f) * rootAuthority;

        // Capture-point balance assist, faded in by exactly how much the
        // body has been handed over to physics. Capped at half: while it is
        // going down it is supposed to be losing the argument at least
        // partly, and a fully assisted fall just looks like the character
        // refusing to fall.
        constexpr float MaximumAssistShare = 0.5f;
        const float assistShare = std::min(MaximumAssistShare,
            std::clamp(m_physicsBlend, 0.0f, 1.0f));
        Vec3 supportCenter {};
        float supportWeight = 0.0f;
        for (const FootPlant& foot : m_feet) {
            if (!foot.planted) continue;
            supportCenter += foot.lockedPositionWorld;
            supportWeight += 1.0f;
        }
        if (assistShare > 0.001f && supportWeight > 0.0f) {
            supportCenter *= 1.0f / supportWeight;
            const float comHeight = std::max(0.35f,
                centerOfMass.z - supportCenter.z);
            const float omega = std::sqrt(9.81f / comHeight);
            Vec3 capturePoint = centerOfMass
                + m_filteredCenterOfMassVelocity / omega;
            capturePoint.z = supportCenter.z;
            Vec3 assistForce = (supportCenter - capturePoint)
                * (mass * omega * 0.85f);
            assistForce.z = 0.0f;
            m_output.rootControlForceWorld = clampMagnitude(assistForce,
                mass * 9.81f * 0.16f) * assistShare;

            const Vec3 rootUp = state.links.front()
                .orientation.rotate({ 0.0f, 0.0f, 1.0f });
            const Vec3 tiltError = cross(rootUp, Vec3 { 0.0f, 0.0f, 1.0f });
            Vec3 planarAngularVelocity = state.links.front().angularVelocity;
            planarAngularVelocity.z = 0.0f;
            Vec3 tiltTorque = tiltError * (mass * 9.5f)
                - planarAngularVelocity * (mass * 0.85f);
            m_output.rootControlTorqueWorld += clampMagnitude(
                tiltTorque, mass * 1.05f) * assistShare;
        }
    }
    m_previousRootPosition = targetRootPosition;
    m_previousRootOrientation = targetRootOrientation;
    m_output.gravityCompensationEnabled = muscle > 0.0001f;
    m_output.physicsBlend = m_physicsBlend;

    m_telemetry.localVelocity = localVelocity;
    m_telemetry.speedMetersPerSecond = speed;
    m_telemetry.facingYawRadians = m_facingYaw;
    m_telemetry.viewYawErrorRadians = viewYawError;
    m_telemetry.gaitDirection = m_gaitDirection;
    m_telemetry.jumpKind = m_jumpKind;
    m_telemetry.flightPhase = m_flightPhase;
    m_telemetry.torsoTwistRadians = m_torsoTwist;
    m_telemetry.headTurnRadians = m_headTurn;
    m_telemetry.turningRateRadiansPerSecond = m_turningRate;
    m_telemetry.forwardLeanRadians = forwardLean;
    m_telemetry.lateralLeanRadians = lateralLean;
    m_telemetry.authoredCurveBlend = authoredCurveBlend;
    m_telemetry.authoredCurveKind = authoredCurveKind;
    m_telemetry.authoredStartActive = authoredStartActive;
    m_telemetry.landingBraceForwardRadians = landingBraceForward;
    m_telemetry.landingBraceLateralRadians = landingBraceLateral;
    m_telemetry.cyclePhase = m_cyclePhase;
    m_telemetry.playbackRate = playbackRate;
    m_telemetry.strideScale = strideScale;
    m_telemetry.terrainSlope = m_terrainSlope;
    m_telemetry.terrainSpeedLimit = terrainSpeedLimit;
    const float longitudinal = std::abs(localVelocity.x);
    const float lateral = std::abs(localVelocity.y);
    const float total = std::max(0.0001f, longitudinal + lateral);
    m_telemetry.forwardWeight = localVelocity.x >= 0.0f
        ? longitudinal / total : 0.0f;
    m_telemetry.backwardWeight = localVelocity.x < 0.0f
        ? longitudinal / total : 0.0f;
    m_telemetry.leftWeight = localVelocity.y >= 0.0f
        ? lateral / total : 0.0f;
    m_telemetry.rightWeight = localVelocity.y < 0.0f
        ? lateral / total : 0.0f;
    m_telemetry.poseAuthority = m_output.guide.poseAuthority;
    m_telemetry.rootAuthority = rootAuthority;
    m_telemetry.rootRotationAuthority = m_output.guide.rootRotationAuthority;
    m_telemetry.muscleAuthority = muscle;
    m_telemetry.reactionStrength = m_reactionStrength;
    m_telemetry.jointErrorRmsDegrees = errorCount > 0
        ? std::sqrt(squaredError / static_cast<float>(errorCount))
            * 180.0f / Pi
        : 0.0f;
    m_telemetry.rootUpright = rootUpright;
    m_telemetry.fallOrientation = m_fallOrientation;
    m_telemetry.getUpPhase = m_getUpPhase;
    m_telemetry.getUpProgress = m_getUpPhase
            == CharacterGetUpPhase3D::Rising
        ? std::clamp(m_getUpClipSeconds / std::max(0.05f,
            standUpClip ? standUpClip->durationSeconds : 1.6f), 0.0f, 1.0f) : 0.0f;
    m_telemetry.getUpAttempt = m_getUpAttempt;
    m_telemetry.fallReflex = m_fallReflex;
    m_telemetry.fallCommit = m_fallCommit;
    m_telemetry.getUpRate = m_getUpPhase == CharacterGetUpPhase3D::Rising
        ? m_getUpRate : 0.0f;
    m_telemetry.getUpStruggle = m_getUpStruggle;
    m_telemetry.muscleTone = m_getUpPhase == CharacterGetUpPhase3D::Settling
        ? m_settleTone : 1.0f;
    m_telemetry.getUpAssistRamp = m_getUpPhase == CharacterGetUpPhase3D::Rising
        ? m_getUpAssistRamp : 0.0f;
    m_telemetry.settleGather = m_getUpPhase == CharacterGetUpPhase3D::Settling
        ? m_settleGather : m_getUpPhase == CharacterGetUpPhase3D::Rising ? 1.0f : 0.0f;
}

} // namespace MatterEngine
