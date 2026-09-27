// Matter Engine clean-room FK/IK layer for physical foot/hand tasks.
// End-effector targets are converted to joint targets; link/world transforms are never teleported.

#include "Engine/Control/BipedKinematics3D.hpp"

#include "Engine/Control/BipedMath3D.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

namespace MatterEngine {
namespace {

Quaternion jointRotation(Vec3 q) {
    return (Quaternion::fromAxisAngle({1.0f, 0.0f, 0.0f}, q.x)
        *Quaternion::fromAxisAngle({0.0f, 1.0f, 0.0f}, q.y)
        *Quaternion::fromAxisAngle({0.0f, 0.0f, 1.0f}, q.z)).normalized();
}

struct DofRef {
    std::size_t link = 0;
    std::size_t axis = 0;
};

bool solveLinear(std::vector<float>& matrix,
    std::vector<float>& rhs, std::size_t n) {
    for (std::size_t pivot = 0; pivot < n; ++pivot) {
        std::size_t best = pivot;
        float magnitude =
            std::abs(matrix[pivot*n+pivot]);
        for (std::size_t row = pivot+1; row < n; ++row) {
            const float candidate =
                std::abs(matrix[row*n+pivot]);
            if (candidate > magnitude) {
                magnitude = candidate;
                best = row;
            }
        }
        if (magnitude < 1.0e-9f) return false;
        if (best != pivot) {
            for (std::size_t col = 0; col < n; ++col)
                std::swap(matrix[pivot*n+col], matrix[best*n+col]);
            std::swap(rhs[pivot], rhs[best]);
        }
        const float diagonal = matrix[pivot*n+pivot];
        for (std::size_t col = pivot; col < n; ++col)
            matrix[pivot*n+col] /= diagonal;
        rhs[pivot] /= diagonal;
        for (std::size_t row = 0; row < n; ++row) {
            if (row == pivot) continue;
            const float factor = matrix[row*n+pivot];
            if (std::abs(factor) < 1.0e-12f) continue;
            for (std::size_t col = pivot; col < n; ++col)
                matrix[row*n+col] -= factor*matrix[pivot*n+col];
            rhs[row] -= factor*rhs[pivot];
        }
    }
    return true;
}

}

std::vector<Vec3> bipedJointCoordinatesFromState3D(
    const RagdollProfile3D& profile,
    const RagdollState3D& state) {
    std::vector<Vec3> result(profile.links.size());
    const std::size_t count =
        std::min(profile.links.size(), state.joints.size());
    for (std::size_t link = 1; link < count; ++link) {
        result[link] = {
            state.joints[link].positionRadians[0],
            state.joints[link].positionRadians[1],
            state.joints[link].positionRadians[2]
        };
    }
    clampBipedJointCoordinates3D(profile, result);
    return result;
}

void clampBipedJointCoordinates3D(
    const RagdollProfile3D& profile,
    std::vector<Vec3>& coordinates) {
    if (coordinates.size() != profile.links.size())
        coordinates.resize(profile.links.size());
    for (std::size_t link = 1; link < profile.links.size(); ++link) {
        for (std::size_t axis = 0; axis < 3; ++axis) {
            const auto& definition =
                profile.links[link].inboundJoint.axes[axis];
            if (!definition.enabled) {
                setComponentBiped(coordinates[link], axis, 0.0f);
                continue;
            }
            setComponentBiped(coordinates[link], axis,
                std::clamp(componentBiped(coordinates[link], axis),
                    definition.minimumRadians,
                    definition.maximumRadians));
        }
    }
}

BipedKinematicPose3D buildBipedKinematicPose3D(
    const RagdollProfile3D& profile,
    std::span<const Vec3> coordinates,
    Vec3 rootPositionWorld,
    Quaternion rootOrientationWorld) {
    BipedKinematicPose3D pose;
    pose.positionsWorld.resize(profile.links.size());
    pose.orientationsWorld.resize(profile.links.size());
    if (profile.links.empty()) return pose;

    std::size_t root = 0;
    for (std::size_t i = 0; i < profile.links.size(); ++i) {
        if (profile.links[i].parentIndex < 0) {
            root = i;
            break;
        }
    }
    pose.positionsWorld[root] = rootPositionWorld;
    pose.orientationsWorld[root] = rootOrientationWorld.normalized();

    for (std::size_t link = 0; link < profile.links.size(); ++link) {
        if (link == root) continue;
        const auto& childDefinition = profile.links[link];
        if (childDefinition.parentIndex < 0) {
            pose.positionsWorld[link] = rootPositionWorld;
            pose.orientationsWorld[link] = rootOrientationWorld;
            continue;
        }
        const std::size_t parent =
            static_cast<std::size_t>(childDefinition.parentIndex);
        const auto& parentDefinition = profile.links[parent];
        Vec3 q = link < coordinates.size()
            ? coordinates[link] : Vec3{};
        for (std::size_t axis = 0; axis < 3; ++axis) {
            const auto& a = childDefinition.inboundJoint.axes[axis];
            setComponentBiped(q, axis,
                a.enabled
                    ? std::clamp(componentBiped(q, axis),
                        a.minimumRadians, a.maximumRadians)
                    : 0.0f);
        }

        const Quaternion parentJointFrame =
            (parentDefinition.modelOrientation.conjugate()
                *childDefinition.inboundJoint.frameModelOrientation)
                .normalized();
        const Quaternion childJointFrame =
            (childDefinition.modelOrientation.conjugate()
                *childDefinition.inboundJoint.frameModelOrientation)
                .normalized();
        pose.orientationsWorld[link] =
            (pose.orientationsWorld[parent]
                *parentJointFrame
                *jointRotation(q)
                *childJointFrame.conjugate()).normalized();

        const Vec3 parentAnchorLocal =
            parentDefinition.modelOrientation.conjugate().rotate(
                childDefinition.inboundJoint.anchorModelPosition
                -parentDefinition.modelPosition);
        const Vec3 childAnchorLocal =
            childDefinition.modelOrientation.conjugate().rotate(
                childDefinition.inboundJoint.anchorModelPosition
                -childDefinition.modelPosition);
        const Vec3 anchorWorld =
            pose.positionsWorld[parent]
                +pose.orientationsWorld[parent].rotate(parentAnchorLocal);
        pose.positionsWorld[link] =
            anchorWorld-pose.orientationsWorld[link].rotate(childAnchorLocal);
    }
    return pose;
}

bool solveBipedEndEffectorDls3D(
    const RagdollProfile3D& profile,
    std::vector<Vec3>& coordinates,
    Vec3 rootPositionWorld,
    Quaternion rootOrientationWorld,
    std::size_t endEffectorLink,
    Vec3 targetPositionWorld,
    Quaternion targetOrientationWorld,
    bool constrainOrientation,
    float taskWeight,
    const BipedIkSettings3D& settings) {
    if (endEffectorLink >= profile.links.size()
        || !finiteBiped(targetPositionWorld)
        || !std::isfinite(taskWeight)
        || taskWeight <= 0.0f) {
        return false;
    }
    if (coordinates.size() != profile.links.size())
        return false;

    std::vector<DofRef> dofs;
    int current = static_cast<int>(endEffectorLink);
    unsigned depth = 0;
    while (current >= 0 && depth < settings.maximumChainDepth) {
        const std::size_t link = static_cast<std::size_t>(current);
        if (profile.links[link].parentIndex < 0) break;
        for (std::size_t axis = 0; axis < 3; ++axis) {
            if (profile.links[link].inboundJoint.axes[axis].enabled)
                dofs.push_back({link, axis});
        }
        current = profile.links[link].parentIndex;
        ++depth;
    }
    if (dofs.empty()) return false;

    const float epsilon =
        std::max(settings.finiteDifferenceRadians, 1.0e-4f);
    const float weight = clamp01Biped(taskWeight);
    for (unsigned iteration = 0; iteration < settings.iterations; ++iteration) {
        const auto base = buildBipedKinematicPose3D(
            profile, coordinates, rootPositionWorld, rootOrientationWorld);
        if (endEffectorLink >= base.positionsWorld.size()) return false;

        const Vec3 positionError =
            (targetPositionWorld-base.positionsWorld[endEffectorLink])
                *settings.positionWeight;
        Vec3 orientationError;
        if (constrainOrientation) {
            orientationError = orientationErrorBiped(
                targetOrientationWorld,
                base.orientationsWorld[endEffectorLink])
                *settings.orientationWeight;
        }
        const float errorNorm =
            positionError.length()
            +(constrainOrientation ? orientationError.length()*0.25f : 0.0f);
        if (errorNorm < 0.002f) break;

        const std::size_t rows = constrainOrientation ? 6u : 3u;
        const std::size_t n = dofs.size();
        std::vector<float> jacobian(rows*n, 0.0f);

        for (std::size_t column = 0; column < n; ++column) {
            const DofRef dof = dofs[column];
            std::vector<Vec3> perturbed = coordinates;
            setComponentBiped(perturbed[dof.link], dof.axis,
                componentBiped(perturbed[dof.link], dof.axis)+epsilon);
            clampBipedJointCoordinates3D(profile, perturbed);
            const auto sample = buildBipedKinematicPose3D(
                profile, perturbed, rootPositionWorld, rootOrientationWorld);
            const Vec3 dp =
                (sample.positionsWorld[endEffectorLink]
                    -base.positionsWorld[endEffectorLink])
                *(settings.positionWeight/epsilon);
            jacobian[0*n+column] = dp.x;
            jacobian[1*n+column] = dp.y;
            jacobian[2*n+column] = dp.z;
            if (constrainOrientation) {
                const Vec3 dw = orientationErrorBiped(
                    sample.orientationsWorld[endEffectorLink],
                    base.orientationsWorld[endEffectorLink])
                    *(settings.orientationWeight/epsilon);
                jacobian[3*n+column] = dw.x;
                jacobian[4*n+column] = dw.y;
                jacobian[5*n+column] = dw.z;
            }
        }

        std::vector<float> normal(n*n, 0.0f);
        std::vector<float> rhs(n, 0.0f);
        const std::array<float, 6> error{
            positionError.x, positionError.y, positionError.z,
            orientationError.x, orientationError.y, orientationError.z
        };
        for (std::size_t row = 0; row < n; ++row) {
            for (std::size_t col = 0; col < n; ++col) {
                float value = 0.0f;
                for (std::size_t r = 0; r < rows; ++r)
                    value += jacobian[r*n+row]*jacobian[r*n+col];
                if (row == col)
                    value += settings.damping*settings.damping;
                normal[row*n+col] = value;
            }
            for (std::size_t r = 0; r < rows; ++r)
                rhs[row] += jacobian[r*n+row]*error[r];
        }
        if (!solveLinear(normal, rhs, n)) return false;

        for (std::size_t column = 0; column < n; ++column) {
            const DofRef dof = dofs[column];
            const float delta = std::clamp(
                rhs[column]*weight,
                -settings.maximumDeltaRadians,
                settings.maximumDeltaRadians);
            setComponentBiped(coordinates[dof.link], dof.axis,
                componentBiped(coordinates[dof.link], dof.axis)+delta);
        }
        clampBipedJointCoordinates3D(profile, coordinates);
    }
    return true;
}

} // namespace MatterEngine
