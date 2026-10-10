#include "Engine/Character/RagdollPoseKinematics3D.hpp"

#include <algorithm>
#include <array>
#include <cmath>

namespace MatterEngine {
namespace {

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

} // namespace

Quaternion ragdollJointRotation3D(Vec3 coordinates) {
    const float angle = coordinates.length();
    return angle > 0.000001f
        ? Quaternion::fromAxisAngle(coordinates / angle, angle)
        : Quaternion {};
}

Vec3 ragdollJointJacobian3D(Vec3 coordinates, Vec3 velocity) {
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

void buildRagdollLocalPose3D(const RagdollProfile3D& profile,
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
                * frame * ragdollJointRotation3D(angles)
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

void solveRagdollChainPoint3D(const RagdollProfile3D& profile,
    std::vector<Vec3>& coordinates, std::size_t footIndex, Vec3 pointInEnd,
    int depth, Vec3 targetRootLocal, float weight, int iterations,
    const std::vector<Vec3>* axisWeights) {
    if (footIndex == 0 || footIndex >= profile.links.size()
        || coordinates.size() != profile.links.size()
        || !finite(targetRootLocal) || weight <= 0.0f) {
        return;
    }
    struct DegreeOfFreedom {
        std::size_t link = 0;
        std::size_t axis = 0;
        float scale = 1.0f;
    };
    std::array<DegreeOfFreedom, 9> degrees;
    std::size_t count = 0;
    int linkIndex = static_cast<int>(footIndex);
    for (int level = 0; level < depth && linkIndex > 0; ++level) {
        const auto& joint = profile.links[static_cast<std::size_t>(linkIndex)]
            .inboundJoint;
        for (std::size_t axis = 0; axis < 3; ++axis) {
            if (joint.axes[axis].enabled && count < degrees.size()) {
                const std::size_t link = static_cast<std::size_t>(linkIndex);
                float scale = 1.0f;
                if (axisWeights != nullptr && link < axisWeights->size()) {
                    const Vec3 weights = (*axisWeights)[link];
                    scale = std::clamp(axis == 0 ? weights.x : axis == 1 ? weights.y : weights.z, 0.0f, 1.0f);
                }
                degrees[count++] = { link, axis, scale };
            }
        }
        linkIndex = profile.links[static_cast<std::size_t>(linkIndex)]
            .parentIndex;
    }
    if (count == 0) return;

    const std::vector<Vec3> original = coordinates;
    RagdollAnimationPose3D pose;
    for (int iteration = 0; iteration < iterations; ++iteration) {
        buildRagdollLocalPose3D(profile, coordinates, pose);
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
                    ragdollJointJacobian3D(coordinates[degree.link], basis));
            const Vec3 anchor = pose.linkPositions[parent]
                + parentModelFrame.rotate(
                    link.inboundJoint.anchorModelPosition
                        - parentLink.modelPosition);
            // Peso por eixo: a coluna escalada (DLS ponderado); o passo sai
            // multiplicado pelo mesmo peso (peso zero = eixo travado).
            const Vec3 linear = cross(angular, endPoint - anchor) * degree.scale;
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
                + std::clamp(system[degreeIndex][count] * degree.scale, -0.13f, 0.13f);
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

} // namespace MatterEngine
