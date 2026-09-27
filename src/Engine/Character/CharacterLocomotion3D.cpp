#include "Engine/Character/CharacterLocomotion3D.hpp"

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
    m_idleSeconds = 0.0f;
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
    m_torsoTwist = 0.0f;
    m_headTurn = 0.0f;
    m_filteredLookPitch = 0.0f;
    m_lookValid = false;
    m_lookYawVelocity = 0.0f;
    m_lookPitchVelocity = 0.0f;
    m_poseCoordinates.clear();
    m_standingTurnTarget = 0.0f;
    m_pendingTurnFoot = -1;
    m_clipPelvisYaw = 0.0f;
    m_clipPelvisToFootYaw = {};
    m_liftoffSpeed = 0.0f;
    m_liftoffVerticalSpeed = 0.0f;
    m_jumpKind = CharacterJumpKind3D::None;
    m_jumpClip = nullptr;
    m_flightPhase = 0.0f;
    m_elapsedSeconds = 0.0f;
    m_getUpPhase = CharacterGetUpPhase3D::None;
    m_fallOrientation = CharacterFallOrientation3D::None;
    m_getUpPhaseSeconds = 0.0f;
    m_recoveredStandingSeconds = 0.0f;
    m_getUpAttempt = 0;
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
    if (state == CharacterLocomotionState3D::Idle) {
        m_cyclePhase = 0.0f;
    }
    if (state == CharacterLocomotionState3D::Airborne
        || state == CharacterLocomotionState3D::JumpStarting
        || state == CharacterLocomotionState3D::Fallen) {
        for (FootPlant& foot : m_feet) {
            foot.planted = false;
            foot.repositioning = false;
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
        || !finite(input.desiredVelocityWorld)
        || !std::isfinite(input.facingYawRadians)
        || !std::isfinite(input.lookPitchRadians)) {
        return;
    }
    if (!m_initialized) reset(profile, state);
    if (!m_initialized) return;
    deltaTime = std::min(deltaTime, 0.05f);
    const std::vector<Vec3> previousCoordinates = m_coordinates;

    // Tracked every tick, independent of authority, so the balance-assist
    // velocity below never sees a stale gap when it turns on mid-collision.
    const Vec3 centerOfMass = dynamics.valid
        ? dynamics.centerOfMass : state.links.front().position;
    Vec3 measuredComVelocity =
        (centerOfMass - m_previousCenterOfMass) / deltaTime;
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
    for (const RagdollContactPoint3D& contact : state.contacts) {
        if (contact.normal.z < 0.40f
            || contact.normalImpulseNewtonSeconds <= 0.00001f) {
            continue;
        }
        anyGroundContact = true;
        for (std::size_t side = 0; side < 2; ++side) {
            if (contact.linkIndex == m_handLinks[side]) {
                handGroundContact[side] = true;
            }
            if (contact.linkIndex == m_shinLinks[side]) {
                kneeGroundContact[side] = true;
            }
            if (m_feet[side].valid
                && contact.linkIndex == m_feet[side].linkIndex) {
                feetGroundContact[side] = true;
            }
        }
    }

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
    const float measuredComHeight = std::max(0.08f,
        centerOfMass.z - groundHeightGuess);
    // Only meaningful while physics is the one holding the body up. In
    // animation mode the authored pose says the character is standing, so
    // "fallen" is not a state it can observe - and reading the simulated
    // bodies anyway is what produced the fall/stand-up/fall loop: those
    // bodies are still lying down for a few ticks after a recovery hands the
    // pose back, which instantly re-triggered another fall.
    const bool physicsOwnsBody = m_physicsBlend > 0.5f;
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
            for (FootPlant& foot : m_feet) {
                foot.planted = false;
                foot.repositioning = false;
            }
        }
    } else if (!input.manipulated) {
        // Being held by the PhysGun freezes recovery instead of fighting the
        // player back toward a stand-up target.
        m_getUpPhaseSeconds += deltaTime;
        if (m_getUpPhase == CharacterGetUpPhase3D::Settling) {
            // Let the body actually come to rest before committing to a
            // recovery: which clip to play depends on which way it lands, and
            // that is not decided while it is still tumbling.
            float fastestLink = 0.0f;
            for (const PhysicsBodyState3D& link : state.links) {
                fastestLink = std::max(fastestLink,
                    link.linearVelocity.length());
            }
            const bool settled = m_getUpPhaseSeconds > 0.55f
                && fastestLink < 0.45f;
            if (settled || m_getUpPhaseSeconds > 2.0f) {
                classifyFallOrientation();
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
                enterGetUpPhase(CharacterGetUpPhase3D::Rising);
            }
        } else if (m_getUpPhase == CharacterGetUpPhase3D::Rising) {
            m_getUpClipSeconds += deltaTime;
            const AnimationClip3D* clip = m_fallOrientation
                    == CharacterFallOrientation3D::FaceDown
                ? animations.standUpFront : animations.standUpBack;
            const float duration = clip && clip->durationSeconds > 0.05f
                ? clip->durationSeconds : 1.6f;
            if (m_getUpClipSeconds >= duration) {
                m_getUpPhase = CharacterGetUpPhase3D::None;
                m_fallOrientation = CharacterFallOrientation3D::None;
                m_getUpPhaseSeconds = 0.0f;
                m_getUpClipSeconds = 0.0f;
                m_recoveredStandingSeconds = 0.0f;
                m_getUpAttempt = 0;
                m_facingYaw = m_getUpAnchorYaw;
                m_turningRate = 0.0f;
                for (FootPlant& foot : m_feet) {
                    foot.planted = false;
                    foot.repositioning = false;
                }
            }
        }
    }
    const bool getUpActive = m_getUpPhase != CharacterGetUpPhase3D::None;

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
    const float impact = std::clamp(
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
    const bool carried = !input.manipulated
        && (m_getUpPhase == CharacterGetUpPhase3D::Rising
            || (!getUpActive && m_physicsBlend < 0.35f));
    const float rootAuthority = carried ? requestedAuthority : 0.0f;

    const Vec3 desiredPlanarVelocity { input.desiredVelocityWorld.x,
        input.desiredVelocityWorld.y, 0.0f };
    const Vec3 rawAcceleration = clampMagnitude(
        (desiredPlanarVelocity - m_previousDesiredVelocity) / deltaTime,
        24.0f);
    m_previousDesiredVelocity = desiredPlanarVelocity;
    const float accelerationBlend = 1.0f - std::exp(-9.0f * deltaTime);
    m_smoothedAcceleration +=
        (rawAcceleration - m_smoothedAcceleration) * accelerationBlend;

    const float speed = std::hypot(desiredPlanarVelocity.x,
        desiredPlanarVelocity.y);
    const bool moving = speed > 0.08f;
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
            const float travelYaw = std::atan2(
                desiredPlanarVelocity.y, desiredPlanarVelocity.x);
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
            if (!available(m_gaitDirection)
                || overrun(m_gaitDirection) > hysteresis) {
                // O setor que contem a direcao (sem recuo, o mais proximo).
                float best = std::numeric_limits<float>::infinity();
                for (const CharacterGaitDirection3D candidate : {
                        CharacterGaitDirection3D::Forward,
                        CharacterGaitDirection3D::Left,
                        CharacterGaitDirection3D::Right,
                        CharacterGaitDirection3D::Backward }) {
                    if (available(candidate) && overrun(candidate) < best) {
                        best = overrun(candidate);
                        m_gaitDirection = candidate;
                    }
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
        const float maximumTurnRate = input.sprinting ? 4.8f
            : moving ? 3.8f : 2.35f;
        const float wantedTurnRate = std::clamp(
            pelvisYawError * (moving ? 7.0f : 5.0f),
            -maximumTurnRate, maximumTurnRate);
        const float maximumTurnAcceleration = moving ? 18.0f : 10.0f;
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
    constexpr float LandingSettleSeconds = 0.18f;
    CharacterLocomotionState3D desiredState = CharacterLocomotionState3D::Idle;
    if (getUpActive) {
        desiredState = m_getUpPhase == CharacterGetUpPhase3D::Settling
            ? CharacterLocomotionState3D::Fallen
            : CharacterLocomotionState3D::GettingUp;
    } else if (!input.grounded) {
        desiredState = CharacterLocomotionState3D::Airborne;
    } else if (!m_previousGrounded
        || (m_telemetry.state == CharacterLocomotionState3D::Landing
            && m_stateSeconds < LandingSettleSeconds)) {
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
        const float verticalSpeed = input.rootVelocityWorld.z;
        m_flightPhase = m_liftoffVerticalSpeed > 1.0f
            ? std::clamp((m_liftoffVerticalSpeed - verticalSpeed)
                / (2.0f * m_liftoffVerticalSpeed), 0.0f, 1.0f)
            : 0.5f + 0.5f * std::clamp(-verticalSpeed / 4.0f, 0.0f, 1.0f);
    } else if (desiredState != CharacterLocomotionState3D::Landing) {
        m_jumpKind = CharacterJumpKind3D::None;
        m_jumpClip = nullptr;
    }
    enterState(desiredState);
    m_previousGrounded = input.grounded;
    m_stateSeconds += deltaTime;
    m_transitionSeconds += deltaTime;
    m_idleSeconds += deltaTime;

    const Quaternion heading = Quaternion::fromAxisAngle(
        { 0.0f, 0.0f, 1.0f }, m_facingYaw);
    const Vec3 localVelocity = heading.conjugate().rotate(
        input.desiredVelocityWorld);

    // Um ciclo por vez, sem mistura por direcao: a pelve ja se alinha com o
    // movimento (ou contra ele, recuando), entao o ciclo sempre anda para a
    // frente ou para tras em relacao a ela. Walking toca a caminhada rapida
    // (ou o recuo), Running o sprint quando existe - senao a caminhada.
    // Todo outro estado (giro parado, que nao tem clipe proprio, e
    // Airborne/Landing, procedurais - ver applyFlightPose) usa o idle.
    const bool gaitMoving = desiredState == CharacterLocomotionState3D::Walking
        || desiredState == CharacterLocomotionState3D::Running;
    const bool sprintTier = desiredState == CharacterLocomotionState3D::Running
        && animations.sprint != nullptr;
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
    constexpr float StepRiseLimit = 0.34f;
    float terrainSlope = 0.0f;
    const Vec3 planarTravel { input.rootVelocityWorld.x,
        input.rootVelocityWorld.y, 0.0f };
    if (input.groundAt && gaitMoving && input.grounded
        && planarTravel.lengthSquared() > 0.01f) {
        const Vec3 ahead = planarTravel.normalized();
        const float base = input.rootPositionWorld.z
            - profile.standingRootHeightMeters;
        // A 35 cm um unico espelho de degrau ja dava inclinacao de 0,5 a 1:
        // medir mais longe mede a escada, nao o degrau.
        for (const float distance : { 0.70f, 1.05f }) {
            Vec3 point = input.rootPositionWorld + ahead * distance;
            point.z = base;
            const GroundProbeResult3D probe = input.groundAt(point);
            if (!probe.hasSurface) continue;
            terrainSlope = std::max(terrainSlope,
                std::abs(probe.pointWorld.z - base) / distance);
        }
    }
    // Sobe rapido (o degrau aparece de uma vez) e solta devagar.
    m_terrainSlope += (terrainSlope - m_terrainSlope) * (1.0f - std::exp(
        -(terrainSlope > m_terrainSlope ? 20.0f : 4.0f) * deltaTime));
    float terrainSpeedLimit = std::numeric_limits<float>::infinity();
    if (gaitMoving && selectedClip != nullptr) {
        const float authoredSpeed = authoredHorizontalSpeed(*selectedClip);
        if (authoredSpeed > 0.05f) {
            const float ratio = speed / authoredSpeed;
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
                terrainSpeedLimit = authoredSpeed * strideCap * 1.15f;
            }
            m_cyclePhase = std::fmod(m_cyclePhase + deltaTime * playbackRate
                / std::max(0.1f, selectedClip->durationSeconds), 1.0f);
        }
    }

    std::vector<Vec3> sampled(profile.links.size(), Vec3 {});
    Quaternion rootRotation;
    Vec3 rootTranslation;
    std::array<float, 2> contactWeights {};
    float clipSampleTime = 0.0f;
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

    // Pulo autoral: o voo vem do clipe (acima). Na aterrissagem os pes sao
    // plantados pela trava de pe (contato forcado) e a pelve desce: e o IK
    // que dobra os joelhos, absorvendo, em vez de posicoes fixas de junta.
    // Sem clipes de pulo, o voo e a pose procedural de antes.
    if (jumpPose && desiredState == CharacterLocomotionState3D::Landing) {
        const float settle = std::clamp(
            m_stateSeconds / LandingSettleSeconds, 0.0f, 1.0f);
        const float compression = std::sin(Pi * settle);
        rootTranslation.z -= 0.08f * compression;
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
            m_stateSeconds / LandingSettleSeconds, 0.0f, 1.0f);
        const float compression = (1.0f - smoothStep(settle)) * 0.55f;
        const float runningShare = smoothStep(m_liftoffSpeed / 2.2f);
        applyFlightPose(profile, sampled, compression, runningShare);
    } else if (m_getUpPhase == CharacterGetUpPhase3D::Settling) {
        // Settling commands nothing: the body is limp on the floor and the
        // pose the player sees is the simulated one (physicsBlend is 1).
        for (std::size_t link = 1;
                link < sampled.size() && link < state.joints.size(); ++link) {
            sampled[link] = { state.joints[link].positionRadians[0],
                state.joints[link].positionRadians[1],
                state.joints[link].positionRadians[2] };
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
    for (std::size_t index = 1; index < profile.links.size(); ++index) {
        Vec3 wanted = m_transitionFrom[index] * (1.0f - transition)
            + sampled[index] * transition
            + m_transitionVelocity[index] * tangentWeight;
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
    }

    // Freada: o corpo reage a desaceleracao, nao a uma tecla. Medido em Run
    // To Stop.fbx (4 m/s ate parar em ~0,35 s): a perna da frente trava
    // esticada, a pelve desce 25 cm, o tronco segue para a frente pela
    // inercia (de 12 para 32 graus) e tudo volta em ~0,7 s. Aqui a inercia
    // do tronco e uma mola (subamortecida: volta com um pequeno rebote)
    // empurrada pela desaceleracao da capsula no sentido do movimento, na
    // proporcao da velocidade com que ele vinha: parar do sprint e forte,
    // do trote e leve. Nada disso segura a capsula - so a pose reage, e
    // voltar a correr no meio da freada so desliga o empurrao. Vale em
    // qualquer direcao: recuando, o tronco vai para tras; de lado, tomba.
    {
        if (speed > 0.3f) m_brakeDirection = desiredPlanarVelocity / speed;
        m_brakeEntrySpeed = std::max(speed,
            m_brakeEntrySpeed - 2.5f * deltaTime);
        const bool braceable = !getUpActive && input.grounded
            && desiredState != CharacterLocomotionState3D::Airborne
            && desiredState != CharacterLocomotionState3D::JumpStarting;
        const float deceleration = std::max(0.0f,
            -dot(m_smoothedAcceleration, m_brakeDirection));
        const float intensity = braceable
            ? std::clamp(deceleration / 16.0f, 0.0f, 1.0f)
                * smoothStep((m_brakeEntrySpeed - 1.5f) / 5.0f)
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
        const float flexion = 0.38f * m_brakeReaction.x;
        const float lateral = 0.18f * m_brakeReaction.y;
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

    // Banking into a turn: a runner carving a curve leans into it and
    // leans forward a little more, roughly in line with the lean/twist
    // magnitudes measured off a real running-turn reference (calibrated by
    // hand against that capture, not sampled from it — see
    // assets/animations/source/mixamo/study/README.txt).
    // v * yawRate is centripetal acceleration. Small curves get a soft
    // onset, and rotating in place does not bank like a running turn.
    const float curveAcceleration = speed * m_turningRate;
    const float curveShare = smoothStep(std::abs(curveAcceleration) / 3.0f);
    const float turnBank = std::atan(curveAcceleration / 9.81f)
        * 0.32f * curveShare;
    // Freando, a pelve inclina para tras so 15% do que inclinaria (a
    // inclinacao satura: com 40%, uma freada de sprint ainda dava os 12,6
    // graus inteiros): o tronco vai para a frente pela inercia (bloco da
    // freada, acima).
    const Vec3 brakingAcceleration = m_brakeDirection
        * std::min(0.0f, dot(m_smoothedAcceleration, m_brakeDirection));
    const Vec3 localAcceleration = heading.conjugate().rotate(
        m_smoothedAcceleration - brakingAcceleration * 0.85f);
    const float wantedForwardLean = std::clamp(
        std::atan(localAcceleration.x / 9.81f) * 0.36f
            + std::abs(turnBank) * 0.08f, -0.22f, 0.22f);
    const float wantedLateralLean = std::clamp(
        std::atan(localAcceleration.y / 9.81f) * 0.18f + turnBank,
        -0.26f, 0.26f);
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
    Vec3 targetRootPosition = risingPose
        ? Vec3 { m_getUpAnchorWorld.x, m_getUpAnchorWorld.y,
            input.rootPositionWorld.z }
        : input.rootPositionWorld;
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
    targetRootPosition += heading.rotate({ m_rootOffset.x, m_rootOffset.y,
        0.0f });
    targetRootPosition.z += m_rootOffset.z;
    // Freada: a pelve desce com a reacao (as pernas dobram pelo IK dos pes).
    if (!getUpActive && !airbornePose) {
        targetRootPosition.z -= 0.12f * std::min(1.0f,
            m_telemetry.brakeReaction);
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

    const bool footworkEnabled = !getUpActive;
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
                if (!result.valid || probe.pointWorld.z > result.height) {
                    result.valid = true;
                    result.height = probe.pointWorld.z;
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
    const Vec3 planarVelocity { input.rootVelocityWorld.x,
        input.rootVelocityWorld.y, 0.0f };

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

    // Passo de acomodacao, parado: pe travado longe de onde a pose quer
    // (parou no meio da passada, um esbarrao) da um passo.
    //
    // Giro parado: na base lado a lado, Left Turn 90, Right Turn(3) e Happy
    // Right Turn usam primeiro o pe do lado do giro. Na base escalonada em
    // alerta, Right Turn(4) e Left Turn usam primeiro o pe de tras, para
    // qualquer lado, contornando o da frente; o segundo passo recompõe a
    // base. A pelve gira continuamente e cada par ja mira o rumo final.
    constexpr float SettleStepDistance = 0.12f;
    constexpr float SettleStepYaw = 0.60f;
    constexpr float TurnTwistTrigger = 0.42f;   // perna torcida demais (24 graus)
    // Se a base ainda ficou mais de 24 graus atras do alvo, inicia outro par
    // mesmo que pelve e pes estejam momentaneamente alinhados. Isso acontece
    // quando a camera continua girando durante o primeiro par: esperar nova
    // torcao deixa a pelve presa na orientacao intermediaria.
    constexpr float TurnEarlyRemaining = 0.42f;
    // Ate 72 graus por par de passos: o pe pousa girado em relacao a pelve,
    // que ainda esta a caminho, e o quadril so gira 45 graus. Mirando 115 de
    // uma vez (meia-volta), o pe pousava onde a perna nao chega e a trava o
    // torcia de volta (4,3 m/s no pe).
    constexpr float TurnStepLimit = 1.25f;
    const bool standing = footworkEnabled && !gaitMoving && input.grounded
        && desiredState != CharacterLocomotionState3D::Landing
        && !airbornePose;
    if (!standing) m_pendingTurnFoot = -1;
    // Onde a pose parada quer o pe com a pelve la no fim do giro: o alvo do
    // giro, ate o limite por par a partir do rumo em que o passo comecou.
    const auto plannedFoot = [&](const FootPlant& foot, float fromFacing,
        Vec3& position, Quaternion& orientation) {
        const float finalYaw = fromFacing + std::clamp(
            wrapAngle(m_standingTurnTarget - fromFacing),
            -TurnStepLimit, TurnStepLimit);
        const Quaternion spin = Quaternion::fromAxisAngle(
            { 0.0f, 0.0f, 1.0f }, wrapAngle(finalYaw - m_facingYaw));
        position = targetRootPosition + spin.rotate(
            targetRootOrientation.rotate(
                localPose.linkPositions[foot.linkIndex]));
        orientation = (spin * clipFootOrientation(foot.linkIndex))
            .normalized();
    };
    const auto yawOf = [&](const FootPlant& foot, Quaternion orientation) {
        const Vec3 forward = forwardOf(foot, orientation);
        return std::atan2(forward.y, forward.x);
    };
    // Rumo da base: o de cada pe travado menos o quanto a pose parada abre
    // aquele pe em relacao a pelve.
    const auto stanceYaw = [&]() {
        float sine = 0.0f, cosine = 0.0f;
        for (const FootPlant& foot : m_feet) {
            const float open = wrapAngle(yawOf(foot,
                clipFootOrientation(foot.linkIndex)) - m_facingYaw);
            const float yaw = yawOf(foot, foot.lockedOrientationWorld) - open;
            sine += std::sin(yaw);
            cosine += std::cos(yaw);
        }
        return std::atan2(sine, cosine);
    };
    const auto startStep = [&](std::size_t steppingSide, bool turn,
        bool pivot) {
        FootPlant& stepping = m_feet[steppingSide];
        stepping.repositioning = true;
        stepping.repositionSeconds = 0.0f;
        stepping.repositionFromWorld = stepping.lockedPositionWorld;
        stepping.repositionFromOrientation = stepping.lockedOrientationWorld;
        stepping.turnStep = turn;
        stepping.pivotStep = pivot;
        stepping.turnFromFacing = m_facingYaw;
        stepping.planted = false;
        // O passo leva o que a distancia e o giro pedem: na base lado a lado
        // um giro de 90 graus anda ~15 cm por pe (0,33 s); na base em alerta
        // o pe anda ~40 cm (0,42 s; nas referencias, ~0,45 s).
        Vec3 position = targetRootPosition + targetRootOrientation.rotate(
            localPose.linkPositions[stepping.linkIndex]);
        Quaternion orientation = clipFootOrientation(stepping.linkIndex);
        if (turn) plannedFoot(stepping, m_facingYaw, position, orientation);
        const float distance = std::hypot(
            position.x - stepping.lockedPositionWorld.x,
            position.y - stepping.lockedPositionWorld.y);
        const float stepYaw = std::abs(wrapAngle(yawOf(stepping, orientation)
            - yawOf(stepping, stepping.lockedOrientationWorld)));
        stepping.repositionDuration = std::clamp(0.22f + 0.35f * distance
            + 0.06f * stepYaw / (0.5f * Pi), 0.24f, 0.48f);
    };
    // Base escalonada (um pe bem a frente do outro, como a base em alerta)
    // ou lado a lado (o parado de surgir).
    const Vec3 headingForward = heading.rotate({ 1.0f, 0.0f, 0.0f });
    const float leftAhead = dot(targetRootOrientation.rotate(
            localPose.linkPositions[m_feet[0].linkIndex]
            - localPose.linkPositions[m_feet[1].linkIndex]),
        headingForward);
    const bool staggered = std::abs(leftAhead) > 0.15f;
    const bool bothPlanted = m_feet[0].valid && m_feet[1].valid
        && m_feet[0].planted && m_feet[1].planted;
    if (standing && bothPlanted && !m_feet[0].repositioning
        && !m_feet[1].repositioning) {
        if (m_pendingTurnFoot >= 0) {
            // Segundo passo do giro, logo depois do primeiro: se o pe ainda
            // esta fora da base nova, contorna o de apoio ate ela.
            const std::size_t side =
                static_cast<std::size_t>(m_pendingTurnFoot);
            m_pendingTurnFoot = -1;
            const FootPlant& foot = m_feet[side];
            Vec3 position;
            Quaternion orientation;
            plannedFoot(foot, m_facingYaw, position, orientation);
            const float drift = std::hypot(
                position.x - foot.lockedPositionWorld.x,
                position.y - foot.lockedPositionWorld.y);
            const float yaw = std::abs(wrapAngle(yawOf(foot, orientation)
                - yawOf(foot, foot.lockedOrientationWorld)));
            if (drift > 0.05f || yaw > 0.17f) startStep(side, true, false);
        } else {
            const float stance = stanceYaw();
            const float twist = wrapAngle(m_facingYaw - stance);
            const float remaining = wrapAngle(m_standingTurnTarget - stance);
            if (std::abs(twist) > TurnTwistTrigger
                || std::abs(remaining) > TurnEarlyRemaining) {
                // Base lado a lado (Left Turn 90, Right Turn(3)): o pe do
                // lado do giro primeiro, abrindo sobre a bola do pe. Base
                // escalonada (Right Turn(4), Left Turn, que partem da base em
                // alerta): para qualquer lado, o pe de TRAS primeiro,
                // contornando o da frente com o peso nele; depois o da
                // frente refaz a base.
                const std::size_t first = staggered
                    ? (leftAhead > 0.0f ? 1 : 0)
                    : (remaining > 0.0f ? 0 : 1);
                startStep(first, true, !staggered);
                m_pendingTurnFoot = static_cast<int>(1 - first);
            } else {
                int worst = -1;
                float worstScore = 0.0f;
                for (std::size_t side = 0; side < m_feet.size(); ++side) {
                    const FootPlant& foot = m_feet[side];
                    const Vec3 wanted = targetRootPosition
                        + targetRootOrientation.rotate(
                            localPose.linkPositions[foot.linkIndex]);
                    const float drift = std::hypot(
                        wanted.x - foot.lockedPositionWorld.x,
                        wanted.y - foot.lockedPositionWorld.y);
                    const float yaw = std::abs(wrapAngle(
                        yawOf(foot, clipFootOrientation(foot.linkIndex))
                        - yawOf(foot, foot.lockedOrientationWorld)));
                    const float score = std::max(drift / SettleStepDistance,
                        yaw / SettleStepYaw);
                    if (score > 1.0f && score > worstScore) {
                        worstScore = score;
                        worst = static_cast<int>(side);
                    }
                }
                if (worst >= 0) {
                    startStep(static_cast<std::size_t>(worst), false, false);
                }
            }
        }
    }
    // Enquanto um pe da o passo, o peso vai para o de apoio e a pelve baixa
    // um pouco (aplicado depois dos alvos dos pes).
    Vec3 standingShift {};

    for (std::size_t side = 0; side < m_feet.size(); ++side) {
        FootPlant& foot = m_feet[side];
        m_telemetry.footPlanted[side] = false;
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

        if (foot.repositioning) {
            foot.repositionSeconds += deltaTime;
            const float progress = std::clamp(foot.repositionSeconds
                / foot.repositionDuration, 0.0f, 1.0f);
            const float eased = smoothStep(progress);
            const float arch = std::sin(Pi * progress);
            // Para onde o pe vai AGORA (o corpo continua girando durante o
            // passo, e a camera pode continuar girando): no giro, a pose
            // parada no rumo final da pelve; na acomodacao (ou se ele
            // voltou a andar), a pose de agora.
            Vec3 plannedPosition = animatedWorld;
            Quaternion plannedOrientation = animatedOrientation;
            if (foot.turnStep && !gaitMoving) {
                plannedFoot(foot, foot.turnFromFacing, plannedPosition,
                    plannedOrientation);
            }
            const GroundSample ground = groundUnder(side, plannedPosition,
                forwardOf(foot, plannedOrientation));
            const Quaternion landing = (rotationBetween({ 0.0f, 0.0f, 1.0f },
                ground.normal) * plannedOrientation).normalized();
            Vec3 liveTarget = plannedPosition;
            liveTarget.z = lockHeight(foot, ground.height, landing);
            foot.repositionToWorld = liveTarget;
            Quaternion orientation = slerpQuaternion(
                foot.repositionFromOrientation, landing, eased);
            // A bola do pe (dois tercos para a ponta, na sola).
            const Vec3 half = link.collider.shape
                    == RagdollColliderShape3D::Box
                ? link.collider.boxHalfExtents : Vec3 {};
            const Vec3 ball = link.collider.localPosition
                + link.collider.localOrientation.rotate(
                    { half.x * 0.6f, 0.0f, -half.z });
            const FootPlant& support = m_feet[1 - side];
            Vec3 stepTarget;
            float toeDown = 0.0f;
            if (foot.pivotStep) {
                // Abre girando sobre a bola do pe: ela quase nao sai do
                // lugar, o calcanhar sobe e roda em volta dela.
                const Vec3 ballFrom = foot.repositionFromWorld
                    + foot.repositionFromOrientation.rotate(ball);
                const Vec3 ballTo = liveTarget + landing.rotate(ball);
                Vec3 ballNow = ballFrom + (ballTo - ballFrom) * eased;
                ballNow.z += 0.012f * arch;
                stepTarget = ballNow - orientation.rotate(ball);
                toeDown = 0.26f * arch;
            } else {
                // Contorna o pe de apoio em arco: angulo e distancia em
                // volta dele, nao a reta que passaria pela outra perna.
                const Vec3 pivot = support.lockedPositionWorld;
                const Vec3 from = foot.repositionFromWorld - pivot;
                const Vec3 to = liveTarget - pivot;
                const float fromRadius = std::hypot(from.x, from.y);
                const float toRadius = std::hypot(to.x, to.y);
                if (fromRadius > 0.05f && toRadius > 0.05f) {
                    const float fromAngle = std::atan2(from.y, from.x);
                    const float angle = fromAngle + wrapAngle(
                        std::atan2(to.y, to.x) - fromAngle) * eased;
                    const float radius = std::max(fromRadius
                        + (toRadius - fromRadius) * eased,
                        std::min(fromRadius, toRadius));
                    stepTarget = pivot + Vec3 { std::cos(angle) * radius,
                        std::sin(angle) * radius, 0.0f };
                } else {
                    stepTarget = foot.repositionFromWorld
                        + (liveTarget - foot.repositionFromWorld) * eased;
                }
                stepTarget.z = foot.repositionFromWorld.z
                    + (liveTarget.z - foot.repositionFromWorld.z) * eased
                    + 0.05f * arch;
                // O calcanhar sai primeiro e o pe pousa plano.
                toeDown = 0.18f * std::sin(Pi * std::min(1.0f,
                    progress * 1.4f));
            }
            if (toeDown > 0.0f) {
                // Ponta para baixo girando em volta da bola do pe.
                const Vec3 forward = forwardOf(foot, orientation);
                Vec3 lateral = cross({ 0.0f, 0.0f, 1.0f }, forward);
                if (lateral.lengthSquared() > 0.0001f) {
                    lateral = lateral.normalized();
                    const Vec3 ballWorld = stepTarget
                        + orientation.rotate(ball);
                    orientation = (Quaternion::fromAxisAngle(lateral, toeDown)
                        * orientation).normalized();
                    stepTarget = ballWorld - orientation.rotate(ball);
                }
            }
            goal.position = stepTarget;
            goal.orientation = orientation;
            goal.pelvisGround = std::min(foot.groundHeight, ground.height);
            goal.reach = onGround ? 1.0f : 0.0f;
            // O peso vai para o pe de apoio e a pelve baixa 1,5 cm no meio
            // do passo. Lado a lado, um quarto do caminho (ate 5 cm); na
            // base escalonada o pe anda 40 cm, e o peso vai quase inteiro
            // para o outro (nas referencias, a pelve anda ~15 cm).
            if (support.planted) {
                Vec3 lean { support.lockedPositionWorld.x
                        - targetRootPosition.x,
                    support.lockedPositionWorld.y - targetRootPosition.y,
                    0.0f };
                standingShift = (staggered
                    ? clampMagnitude(lean * 0.45f, 0.14f)
                    : clampMagnitude(lean * 0.25f, 0.05f)) * arch;
                standingShift.z = -0.015f * arch;
            }
            foot.previousAnimatedPositionWorld = animatedWorld;
            foot.previousAnimatedPositionValid = true;
            if (progress >= 1.0f) {
                foot.repositioning = false;
                foot.turnStep = false;
                foot.pivotStep = false;
                foot.planted = true;
                foot.lockedPositionWorld = liveTarget;
                foot.lockedOrientationWorld = landing;
                foot.plantedGoalPosition = liveTarget;
                foot.plantedGoalOrientation = landing;
                foot.groundHeight = ground.height;
                foot.groundNormal = ground.normal;
            }
            continue;
        }

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
        if (!foot.planted && stanceCandidate) {
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
            // A perna ainda alcanca o pe travado? Descendo escada o corpo
            // segue e o degrau de tras fica longe: segurar a trava deixava o
            // pe pendurado no limite do IK (26 cm acima do degrau), marcado
            // como apoiado. Fora de alcance, o pe solta e vira balanco.
            bool outOfReach = false;
            {
                const int shin = link.parentIndex;
                const int thigh = shin > 0 ? profile.links[
                    static_cast<std::size_t>(shin)].parentIndex : -1;
                if (thigh > 0) {
                    const RagdollLinkDefinition3D& shinLink =
                        profile.links[static_cast<std::size_t>(shin)];
                    const RagdollLinkDefinition3D& thighLink =
                        profile.links[static_cast<std::size_t>(thigh)];
                    const float legLength =
                        (shinLink.inboundJoint.anchorModelPosition
                            - thighLink.inboundJoint.anchorModelPosition)
                            .length()
                        + (link.inboundJoint.anchorModelPosition
                            - shinLink.inboundJoint.anchorModelPosition)
                            .length();
                    const Vec3 hip = targetRootPosition
                        + Vec3 { 0.0f, 0.0f, m_pelvisOffset }
                        + targetRootOrientation.rotate(
                            profile.links[0].modelOrientation.conjugate()
                                .rotate(thighLink.inboundJoint
                                    .anchorModelPosition
                                    - profile.links[0].modelPosition));
                    const Vec3 ankle = foot.plantedGoalPosition
                        + foot.plantedGoalOrientation.rotate(
                            link.modelOrientation.conjugate().rotate(
                                link.inboundJoint.anchorModelPosition
                                    - link.modelPosition));
                    outOfReach = (hip - ankle).length() > legLength * 1.01f;
                }
            }
            // ...ou o IK nao chegou la no tick anterior (limite de junta).
            outOfReach = outOfReach || foot.reachError > 0.06f;
            // Solta quando o clipe diz que o apoio acabou. Com folga (ate o
            // contato cair a 0,30) o pe ficava preso enquanto a pose ja
            // levava o pe embora; na soltura ele corria atras dela (medido:
            // 12 m/s no sprint de lado, com o corpo a 3,7).
            if (contactWeights[side] < 0.55f || outOfReach
                || (!stanceCandidate && clearance > 0.055f)
                || horizontalDrift > 0.48f || !input.grounded) {
                foot.planted = false;
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
                + heading.rotate(footInClip);
            const Vec3 landingForward = heading.rotate(next->forwardInClip);
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
            goal.landingOrientation = (rotationBetween({ 0.0f, 0.0f, 1.0f },
                landing.normal) * animatedOrientation).normalized();
            goal.landingPosition = { landingWorld.x, landingWorld.y,
                lockHeight(foot, landing.height, goal.landingOrientation) };
            goal.hipTravel = planarVelocity * secondsToTouchdown;
            goal.authoredLandingHorizontal = std::hypot(
                footInClip.x - next->hipInClip.x,
                footInClip.y - next->hipInClip.y);
            goal.authoredLandingVertical = next->hipInClip.z - footInClip.z;
            pelvisGround = from + rise * progress;
            // O pe chega orientado pelo chao de destino.
            const float settle = smoothStep((progress - 0.6f) / 0.4f);
            swingOrientation = (slerpQuaternion(Quaternion {},
                rotationBetween({ 0.0f, 0.0f, 1.0f }, landing.normal), settle)
                * animatedOrientation).normalized();
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
        // O pe vai sendo levado para o ponto de pouso encaixado no degrau.
        const Vec3 shift = next != nullptr && foot.fitValid
            ? foot.fitShift * smoothStep(progress / 0.8f) : Vec3 {};
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
    const Quaternion rootInverse = targetRootOrientation.conjugate();
    for (std::size_t side = 0; side < m_feet.size(); ++side) {
        if (!goals[side].active) continue;
        solveFootPose(profile, m_coordinates, m_feet[side].linkIndex,
            rootInverse.rotate(goals[side].position - targetRootPosition),
            (rootInverse * goals[side].orientation).normalized(), 1.0f,
            !gaitMoving);
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
    for (std::size_t index = 1; index < m_coordinates.size(); ++index) {
        const Vec3 previous = previousCoordinates[index];
        Vec3& current = m_coordinates[index];
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

    // Quanto cada pe ficou longe do alvo depois do IK e do limitador: pe
    // travado que a perna nao alcanca (limite do quadril descendo escada)
    // solta no proximo tick.
    for (std::size_t side = 0; side < m_feet.size(); ++side) {
        FootPlant& foot = m_feet[side];
        if (!goals[side].active || !foot.valid) {
            foot.reachError = 0.0f;
            continue;
        }
        const Vec3 solved = targetRootPosition + targetRootOrientation.rotate(
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
    if (!getUpActive) {
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
    const float impactSoftening = getUpActive ? 1.0f
        : std::clamp(1.0f - m_reactionStrength * 0.97f, 0.03f, 1.0f);
    float squaredError = 0.0f;
    std::uint32_t errorCount = 0;
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
        // Same per-category get-up gains as
        // BiomechanicalBipedExperiment3D's: soft and passive while
        // Resting, firm (legs firmest) while actively recovering.
        // Default mode holds the pose as hard as the drives allow, and lets
        // go in step with physicsBlend. This is the "force the pose when
        // nothing is happening, yield when something is" dial: at blend 0
        // the motors run at their ceiling, at blend 1 they are back to the
        // profile's nominal strength and the body argues back.
        const float poseHold = 1.0f + (1.0f - m_physicsBlend);
        float stiffnessMultiplier = poseHold;
        float dampingMultiplier = std::sqrt(poseHold);
        float torqueMultiplier = poseHold;
        if (getUpActive) {
            const std::string& id = profile.links[index].id;
            const bool isArm = id.find("Arm") != std::string::npos
                || id.find("Forearm") != std::string::npos
                || id.find("Hand") != std::string::npos;
            const bool isTorso = id == "Abdomen" || id == "Chest"
                || id == "UpperChest" || id == "Neck" || id == "Head";
            if (m_getUpPhase == CharacterGetUpPhase3D::Settling) {
                stiffnessMultiplier = 0.16f;
                dampingMultiplier = 0.88f;
                torqueMultiplier = 0.38f;
            } else {
                stiffnessMultiplier = isArm ? 1.16f : isTorso ? 1.38f : 1.58f;
                dampingMultiplier = isArm ? 1.02f : isTorso ? 1.18f : 1.22f;
                torqueMultiplier = isArm ? 1.82f : isTorso ? 2.08f : 2.55f;
            }
        }
        for (std::size_t axis = 0; axis < 3; ++axis) {
            if (!profile.links[index].inboundJoint.axes[axis].enabled) continue;
            RagdollDriveTarget3D target;
            target.linkIndex = static_cast<std::uint32_t>(index);
            target.axis = static_cast<RagdollAxis3D>(axis);
            target.positionRadians = component(m_coordinates[index], axis);
            // Folga acima do joelho do sprint (~21 rad/s de pico).
            target.velocityRadiansPerSecond = std::clamp(
                component(velocity, axis), -30.0f, 30.0f);
            target.stiffnessScale =
                muscle * stiffnessMultiplier * impactSoftening;
            target.dampingScale =
                std::sqrt(muscle) * dampingMultiplier;
            target.maximumTorqueScale =
                muscle * torqueMultiplier * impactSoftening;
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
    if (getUpActive && !state.links.empty()) {
        // Recovery is an animation now, so nothing pushes the root here. The
        // branch still claims the chain so the balance assist below - which
        // keys off low rootAuthority, and rootAuthority is zero for the whole
        // get-up window - cannot quietly take over and fight the clip.
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
        ? std::clamp(m_getUpClipSeconds / 2.5f, 0.0f, 1.0f) : 0.0f;
    m_telemetry.getUpAttempt = m_getUpAttempt;
}

} // namespace MatterEngine
