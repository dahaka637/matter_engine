#include "Engine/Athlete/Core/Kinematics3D.hpp"

#include <algorithm>
#include <cmath>

namespace MatterEngine {
namespace {

float component(Vec3 v, std::size_t axis) { return axis == 0 ? v.x : axis == 1 ? v.y : v.z; }

void setComponent(Vec3& v, std::size_t axis, float value) {
    if (axis == 0) v.x = value;
    else if (axis == 1) v.y = value;
    else v.z = value;
}

std::size_t findLink(const RagdollProfile3D& profile, const char* id) {
    for (std::size_t i = 0; i < profile.links.size(); ++i)
        if (profile.links[i].id == id) return i;
    return 0;
}

// A rotacao que leva o par (a1, b1) ao par (a2, b2) (a perpendicular a b).
Quaternion alignPairs(Vec3 a1, Vec3 b1, Vec3 a2, Vec3 b2) {
    const Quaternion from = athleteFromBasis3D(a1, b1, cross(a1, b1));
    const Quaternion to = athleteFromBasis3D(a2, b2, cross(a2, b2));
    return (to * from.conjugate()).normalized();
}

Vec3 orthogonal(Vec3 v, Vec3 axis) {
    const Vec3 out = v - axis * dot(v, axis);
    return out.lengthSquared() > 1e-10f ? out.normalized() : Vec3 {};
}

Vec3 anchorIn(const RagdollProfile3D& profile, std::size_t link, Vec3 anchorModel) {
    const RagdollLinkDefinition3D& definition = profile.links[link];
    return definition.modelOrientation.conjugate().rotate(anchorModel - definition.modelPosition);
}

// O pe nos dois eixos do tornozelo, o mais perto da orientacao pedida: o
// "para cima" do pe primeiro, a ponta em segundo (Gauss-Newton).
Vec3 fitAnkle(const RagdollProfile3D& profile, std::size_t footIndex, Quaternion shinWorld, Quaternion footWorld,
    Vec3 start) {
    const RagdollLinkDefinition3D& foot = profile.links[footIndex];
    const RagdollLinkDefinition3D& shin = profile.links[static_cast<std::size_t>(foot.parentIndex)];
    const Quaternion shinFrame = shinWorld * shin.modelOrientation.conjugate() * foot.inboundJoint.frameModelOrientation;
    const Quaternion childFrame = (foot.modelOrientation.conjugate() * foot.inboundJoint.frameModelOrientation).conjugate();
    const Vec3 upLocal = foot.modelOrientation.conjugate().rotate({ 0.0f, 0.0f, 1.0f });
    const Vec3 forwardLocal = foot.modelOrientation.conjugate().rotate({ 1.0f, 0.0f, 0.0f });
    const Vec3 wantedUp = footWorld.rotate(upLocal);
    const Vec3 wantedForward = footWorld.rotate(forwardLocal);
    const auto residual = [&](Vec3 angles, float out[6]) {
        const Quaternion world = shinFrame * athleteJointRotation3D(angles) * childFrame;
        const Vec3 up = world.rotate(upLocal) - wantedUp;
        const Vec3 forward = (world.rotate(forwardLocal) - wantedForward) * 0.6f;
        out[0] = up.x; out[1] = up.y; out[2] = up.z;
        out[3] = forward.x; out[4] = forward.y; out[5] = forward.z;
    };
    std::array<std::size_t, 3> axes {};
    std::size_t free = 0;
    for (std::size_t axis = 0; axis < 3; ++axis)
        if (foot.inboundJoint.axes[axis].enabled) axes[free++] = axis;
    Vec3 angles = start;
    for (int iteration = 0; iteration < 10 && free > 0 && free <= 2; ++iteration) {
        float r[6];
        residual(angles, r);
        float jacobian[6][2] {};
        for (std::size_t column = 0; column < free; ++column) {
            Vec3 probe = angles;
            setComponent(probe, axes[column], component(probe, axes[column]) + 0.0005f);
            float shifted[6];
            residual(probe, shifted);
            for (int row = 0; row < 6; ++row) jacobian[row][column] = (shifted[row] - r[row]) / 0.0005f;
        }
        float normal[2][3] {};
        for (std::size_t a = 0; a < free; ++a) {
            for (std::size_t b = 0; b < free; ++b)
                for (int row = 0; row < 6; ++row) normal[a][b] += jacobian[row][a] * jacobian[row][b];
            normal[a][a] += 0.001f;
            for (int row = 0; row < 6; ++row) normal[a][2] -= jacobian[row][a] * r[row];
        }
        float step[2] {};
        if (free == 1) {
            step[0] = normal[0][2] / normal[0][0];
        } else {
            const float determinant = normal[0][0] * normal[1][1] - normal[0][1] * normal[1][0];
            if (std::abs(determinant) < 1e-9f) break;
            step[0] = (normal[0][2] * normal[1][1] - normal[0][1] * normal[1][2]) / determinant;
            step[1] = (normal[0][0] * normal[1][2] - normal[1][0] * normal[0][2]) / determinant;
        }
        float size = 0.0f;
        for (std::size_t column = 0; column < free; ++column) {
            const RagdollAxisDefinition3D& limit = foot.inboundJoint.axes[axes[column]];
            const float next = std::clamp(component(angles, axes[column]) + std::clamp(step[column], -0.3f, 0.3f),
                limit.minimumRadians, limit.maximumRadians);
            size += std::abs(next - component(angles, axes[column]));
            setComponent(angles, axes[column], next);
        }
        if (size < 0.00005f) break;
    }
    return angles;
}

} // namespace

Quaternion athleteJointRotation3D(Vec3 coordinates) {
    const float angle = coordinates.length();
    return angle > 0.000001f ? Quaternion::fromAxisAngle(coordinates / angle, angle) : Quaternion {};
}

Vec3 athleteJointJacobian3D(Vec3 coordinates, Vec3 velocity) {
    const float squared = coordinates.lengthSquared();
    const float angle = std::sqrt(squared);
    const float first = squared < 0.0001f ? 0.5f - squared / 24.0f : (1.0f - std::cos(angle)) / squared;
    const float second = squared < 0.0001f ? 1.0f / 6.0f - squared / 120.0f
                                           : (angle - std::sin(angle)) / (squared * angle);
    return velocity + cross(coordinates, velocity) * first + cross(coordinates, cross(coordinates, velocity)) * second;
}

void athleteLocalPose3D(const RagdollProfile3D& profile, const std::vector<Vec3>& coordinates,
    RagdollAnimationPose3D& pose) {
    pose.linkPositions.assign(profile.links.size(), {});
    pose.linkOrientations.assign(profile.links.size(), {});
    for (std::size_t index = 1; index < profile.links.size(); ++index) {
        const RagdollLinkDefinition3D& link = profile.links[index];
        if (link.parentIndex < 0) continue;
        const std::size_t parent = static_cast<std::size_t>(link.parentIndex);
        const RagdollLinkDefinition3D& parentLink = profile.links[parent];
        Vec3 angles = index < coordinates.size() ? coordinates[index] : Vec3 {};
        for (std::size_t axis = 0; axis < 3; ++axis) {
            const RagdollAxisDefinition3D& limit = link.inboundJoint.axes[axis];
            setComponent(angles, axis, limit.enabled
                ? std::clamp(component(angles, axis), limit.minimumRadians, limit.maximumRadians) : 0.0f);
        }
        const Quaternion frame = link.inboundJoint.frameModelOrientation;
        pose.linkOrientations[index] = (pose.linkOrientations[parent] * parentLink.modelOrientation.conjugate() * frame
            * athleteJointRotation3D(angles) * (link.modelOrientation.conjugate() * frame).conjugate())
                                           .normalized();
        const Vec3 anchor = link.inboundJoint.anchorModelPosition;
        pose.linkPositions[index] = pose.linkPositions[parent]
            + pose.linkOrientations[parent].rotate(parentLink.modelOrientation.conjugate().rotate(
                anchor - parentLink.modelPosition))
            - pose.linkOrientations[index].rotate(link.modelOrientation.conjugate().rotate(anchor - link.modelPosition));
    }
}

void athleteWorldPose3D(const RagdollProfile3D& profile, const std::vector<Vec3>& coordinates, Vec3 rootPosition,
    Quaternion rootOrientation, RagdollAnimationPose3D& pose) {
    athleteLocalPose3D(profile, coordinates, pose);
    for (std::size_t i = 0; i < pose.linkPositions.size(); ++i) {
        pose.linkPositions[i] = rootPosition + rootOrientation.rotate(pose.linkPositions[i]);
        pose.linkOrientations[i] = (rootOrientation * pose.linkOrientations[i]).normalized();
    }
}

Vec3 athleteRotationVector3D(Quaternion q) {
    q = q.normalized();
    if (q.w < 0.0f) q = { -q.x, -q.y, -q.z, -q.w };
    const Vec3 v { q.x, q.y, q.z };
    const float n = v.length();
    if (n < 1e-7f) return {};
    return v * (2.0f * std::atan2(n, q.w) / n);
}

Quaternion athleteFromBasis3D(Vec3 x, Vec3 y, Vec3 z) {
    const float m00 = x.x, m01 = y.x, m02 = z.x;
    const float m10 = x.y, m11 = y.y, m12 = z.y;
    const float m20 = x.z, m21 = y.z, m22 = z.z;
    const float trace = m00 + m11 + m22;
    Quaternion q;
    if (trace > 0.0f) {
        const float s = std::sqrt(trace + 1.0f) * 2.0f;
        q = { (m21 - m12) / s, (m02 - m20) / s, (m10 - m01) / s, 0.25f * s };
    } else if (m00 > m11 && m00 > m22) {
        const float s = std::sqrt(1.0f + m00 - m11 - m22) * 2.0f;
        q = { 0.25f * s, (m01 + m10) / s, (m02 + m20) / s, (m21 - m12) / s };
    } else if (m11 > m22) {
        const float s = std::sqrt(1.0f + m11 - m00 - m22) * 2.0f;
        q = { (m01 + m10) / s, 0.25f * s, (m12 + m21) / s, (m02 - m20) / s };
    } else {
        const float s = std::sqrt(1.0f + m22 - m00 - m11) * 2.0f;
        q = { (m02 + m20) / s, (m12 + m21) / s, 0.25f * s, (m10 - m01) / s };
    }
    return q.normalized();
}

Vec3 athleteJointCoordinates3D(const RagdollProfile3D& profile, std::size_t index, Quaternion parentWorld,
    Quaternion childWorld) {
    const RagdollLinkDefinition3D& link = profile.links[index];
    const RagdollLinkDefinition3D& parent = profile.links[static_cast<std::size_t>(link.parentIndex)];
    const Quaternion frame = link.inboundJoint.frameModelOrientation;
    Vec3 local = athleteRotationVector3D(frame.conjugate() * parent.modelOrientation * parentWorld.conjugate()
        * childWorld * link.modelOrientation.conjugate() * frame);
    for (std::size_t axis = 0; axis < 3; ++axis) {
        const RagdollAxisDefinition3D& limit = link.inboundJoint.axes[axis];
        setComponent(local, axis, limit.enabled
            ? std::clamp(component(local, axis), limit.minimumRadians, limit.maximumRadians) : 0.0f);
    }
    return local;
}

void athleteSolveChainPoint3D(const RagdollProfile3D& profile, std::vector<Vec3>& coordinates,
    std::size_t endIndex, Vec3 pointInEnd, int depth, Vec3 targetRootLocal, float weight, int iterations) {
    if (endIndex == 0 || endIndex >= profile.links.size() || coordinates.size() != profile.links.size()
        || !std::isfinite(targetRootLocal.x) || weight <= 0.0f)
        return;
    struct Degree {
        std::size_t link = 0;
        std::size_t axis = 0;
    };
    std::array<Degree, 9> degrees;
    std::size_t count = 0;
    int linkIndex = static_cast<int>(endIndex);
    for (int level = 0; level < depth && linkIndex > 0; ++level) {
        const auto& joint = profile.links[static_cast<std::size_t>(linkIndex)].inboundJoint;
        for (std::size_t axis = 0; axis < 3; ++axis)
            if (joint.axes[axis].enabled && count < degrees.size())
                degrees[count++] = { static_cast<std::size_t>(linkIndex), axis };
        linkIndex = profile.links[static_cast<std::size_t>(linkIndex)].parentIndex;
    }
    if (count == 0) return;
    const std::vector<Vec3> original = coordinates;
    RagdollAnimationPose3D pose;
    for (int iteration = 0; iteration < iterations; ++iteration) {
        athleteLocalPose3D(profile, coordinates, pose);
        const Vec3 endPoint = pose.linkPositions[endIndex] + pose.linkOrientations[endIndex].rotate(pointInEnd);
        const Vec3 error = targetRootLocal - endPoint;
        if (error.lengthSquared() < 0.00000025f) break;
        float jacobian[3][9] {};
        for (std::size_t column = 0; column < count; ++column) {
            const Degree degree = degrees[column];
            const RagdollLinkDefinition3D& link = profile.links[degree.link];
            const std::size_t parent = static_cast<std::size_t>(link.parentIndex);
            const RagdollLinkDefinition3D& parentLink = profile.links[parent];
            const Quaternion parentModelFrame = pose.linkOrientations[parent] * parentLink.modelOrientation.conjugate();
            const Vec3 basis = degree.axis == 0 ? Vec3 { 1.0f, 0.0f, 0.0f }
                : degree.axis == 1 ? Vec3 { 0.0f, 1.0f, 0.0f } : Vec3 { 0.0f, 0.0f, 1.0f };
            const Vec3 angular = (parentModelFrame * link.inboundJoint.frameModelOrientation)
                                     .rotate(athleteJointJacobian3D(coordinates[degree.link], basis));
            const Vec3 anchor = pose.linkPositions[parent]
                + parentModelFrame.rotate(link.inboundJoint.anchorModelPosition - parentLink.modelPosition);
            const Vec3 linear = cross(angular, endPoint - anchor);
            jacobian[0][column] = linear.x;
            jacobian[1][column] = linear.y;
            jacobian[2][column] = linear.z;
        }
        float system[9][10] {};
        for (std::size_t row = 0; row < count; ++row) {
            for (std::size_t column = 0; column < count; ++column) {
                for (std::size_t dimension = 0; dimension < 3; ++dimension)
                    system[row][column] += jacobian[dimension][row] * jacobian[dimension][column];
                if (row == column) system[row][column] += 0.025f;
            }
            system[row][count] = jacobian[0][row] * error.x + jacobian[1][row] * error.y + jacobian[2][row] * error.z;
        }
        for (std::size_t pivot = 0; pivot < count; ++pivot) {
            std::size_t best = pivot;
            for (std::size_t row = pivot + 1; row < count; ++row)
                if (std::abs(system[row][pivot]) > std::abs(system[best][pivot])) best = row;
            if (best != pivot)
                for (std::size_t column = pivot; column <= count; ++column) std::swap(system[pivot][column], system[best][column]);
            const float divisor = system[pivot][pivot];
            if (std::abs(divisor) < 0.000001f) continue;
            for (std::size_t column = pivot; column <= count; ++column) system[pivot][column] /= divisor;
            for (std::size_t row = 0; row < count; ++row) {
                if (row == pivot) continue;
                const float factor = system[row][pivot];
                for (std::size_t column = pivot; column <= count; ++column) system[row][column] -= factor * system[pivot][column];
            }
        }
        for (std::size_t index = 0; index < count; ++index) {
            const Degree degree = degrees[index];
            const RagdollAxisDefinition3D& limit = profile.links[degree.link].inboundJoint.axes[degree.axis];
            const float next = component(coordinates[degree.link], degree.axis)
                + std::clamp(system[index][count], -0.13f, 0.13f);
            setComponent(coordinates[degree.link], degree.axis, std::clamp(next, limit.minimumRadians, limit.maximumRadians));
        }
    }
    weight = std::clamp(weight, 0.0f, 1.0f);
    for (std::size_t index = 0; index < count; ++index) {
        const Degree degree = degrees[index];
        const float result = component(coordinates[degree.link], degree.axis);
        const float before = component(original[degree.link], degree.axis);
        setComponent(coordinates[degree.link], degree.axis, before + (result - before) * weight);
    }
}

AthleteLegs3D resolveAthleteLegs3D(const RagdollProfile3D& profile) {
    AthleteLegs3D legs;
    const char* thighs[2] = { "LeftThigh", "RightThigh" };
    const char* shins[2] = { "LeftShin", "RightShin" };
    const char* feet[2] = { "LeftFoot", "RightFoot" };
    legs.valid = true;
    for (std::size_t side = 0; side < 2; ++side) {
        legs.thigh[side] = findLink(profile, thighs[side]);
        legs.shin[side] = findLink(profile, shins[side]);
        legs.foot[side] = findLink(profile, feet[side]);
        if (legs.thigh[side] == 0 || legs.shin[side] == 0 || legs.foot[side] == 0
            || profile.links[legs.thigh[side]].parentIndex != 0) {
            legs.valid = false;
            continue;
        }
        const Vec3 hip = profile.links[legs.thigh[side]].inboundJoint.anchorModelPosition;
        const Vec3 knee = profile.links[legs.shin[side]].inboundJoint.anchorModelPosition;
        const Vec3 ankle = profile.links[legs.foot[side]].inboundJoint.anchorModelPosition;
        legs.hipInPelvis[side] = anchorIn(profile, 0, hip);
        legs.ankleInFoot[side] = anchorIn(profile, legs.foot[side], ankle);
        legs.ankleInShin[side] = anchorIn(profile, legs.shin[side], ankle);
        legs.thighLength[side] = (knee - hip).length();
        legs.shinLength[side] = (ankle - knee).length();
    }
    return legs;
}

Vec3 solveAthleteLeg3D(const RagdollProfile3D& profile, const AthleteLegs3D& legs, int side,
    std::vector<Vec3>& coordinates, Vec3 rootPosition, Quaternion rootOrientation, Vec3 ankleWorld, Vec3 kneeWorld,
    Quaternion footWorld) {
    if (!legs.valid || side < 0 || side > 1 || coordinates.size() != profile.links.size()) return ankleWorld;
    const std::size_t s = static_cast<std::size_t>(side);
    const std::size_t thighIndex = legs.thigh[s], shinIndex = legs.shin[s], footIndex = legs.foot[s];
    const RagdollLinkDefinition3D& thigh = profile.links[thighIndex];
    const RagdollLinkDefinition3D& shin = profile.links[shinIndex];
    const float a = legs.thighLength[s], b = legs.shinLength[s];
    const Vec3 hip = rootPosition + rootOrientation.rotate(legs.hipInPelvis[s]);
    const Vec3 v = ankleWorld - hip;
    const float distance = v.length();
    const Vec3 u = distance > 1e-5f ? v * (1.0f / distance) : rootOrientation.rotate({ 0.0f, 0.0f, -1.0f });
    const float D = std::clamp(distance, std::abs(a - b) + 0.03f, (a + b) * 0.9995f);
    const float cosHip = std::clamp((a * a + D * D - b * b) / (2.0f * a * D), -1.0f, 1.0f);
    const float sinHip = std::sqrt(std::max(0.0f, 1.0f - cosHip * cosHip));
    Vec3 pole = orthogonal(kneeWorld, u);
    if (pole.lengthSquared() < 0.5f) pole = orthogonal(rootOrientation.rotate({ 1.0f, 0.0f, 0.0f }), u);
    if (pole.lengthSquared() < 0.5f) pole = orthogonal({ 1.0f, 0.0f, 0.0f }, u);
    const Vec3 thighDirection = u * cosHip + pole * sinHip;
    const Vec3 kneePoint = hip + thighDirection * a;
    const Vec3 shinDirection = (hip + u * D - kneePoint).normalized();
    Vec3 kneeForward = orthogonal(pole, thighDirection);
    if (kneeForward.lengthSquared() < 0.5f)
        kneeForward = orthogonal(rootOrientation.rotate({ 1.0f, 0.0f, 0.0f }), thighDirection);
    // O link na pose de repouso: a coxa e a canela no eixo da junta (de cima
    // para baixo), a frente do joelho em +x do modelo.
    const Vec3 hipModel = thigh.inboundJoint.anchorModelPosition;
    const Vec3 kneeModel = shin.inboundJoint.anchorModelPosition;
    const Vec3 ankleModel = profile.links[footIndex].inboundJoint.anchorModelPosition;
    const Vec3 thighRest = (kneeModel - hipModel).normalized();
    const Vec3 shinRest = (ankleModel - kneeModel).normalized();
    const Vec3 thighRestForward = orthogonal({ 1.0f, 0.0f, 0.0f }, thighRest);
    const Vec3 shinRestForward = orthogonal({ 1.0f, 0.0f, 0.0f }, shinRest);
    const Quaternion thighRotation = alignPairs(thighRest, thighRestForward, thighDirection, kneeForward);
    const Quaternion thighWorld = (thighRotation * thigh.modelOrientation).normalized();
    // A canela gira so na dobradica do joelho (o eixo lateral da coxa).
    const Vec3 hinge = thighRotation.rotate(cross(thighRest, thighRestForward));
    const Vec3 shinForward = orthogonal(cross(hinge, shinDirection), shinDirection);
    const Quaternion shinRotation = alignPairs(shinRest, shinRestForward, shinDirection,
        shinForward.lengthSquared() > 0.5f ? shinForward : kneeForward);
    const Quaternion shinWorld = (shinRotation * shin.modelOrientation).normalized();
    const Quaternion rootLinkWorld = (rootOrientation * profile.links.front().modelOrientation).normalized();
    coordinates[thighIndex] = athleteJointCoordinates3D(profile, thighIndex, rootLinkWorld, thighWorld);
    // (Com a coxa no limite o joelho muda de lugar: refeito pela coxa real.)
    RagdollAnimationPose3D pose;
    athleteWorldPose3D(profile, coordinates, rootPosition, rootOrientation, pose);
    coordinates[shinIndex] = athleteJointCoordinates3D(profile, shinIndex, pose.linkOrientations[thighIndex], shinWorld);
    // O que faltar (limites) pelo IK numerico, no mesmo ramo.
    athleteWorldPose3D(profile, coordinates, rootPosition, rootOrientation, pose);
    const Vec3 reached = pose.linkPositions[footIndex] + pose.linkOrientations[footIndex].rotate(legs.ankleInFoot[s]);
    const Vec3 target = hip + u * D;
    if ((reached - target).lengthSquared() > 1e-6f) {
        athleteSolveChainPoint3D(profile, coordinates, shinIndex, legs.ankleInShin[s], 2,
            rootOrientation.conjugate().rotate(target - rootPosition), 1.0f, 8);
        athleteWorldPose3D(profile, coordinates, rootPosition, rootOrientation, pose);
    }
    coordinates[footIndex] = fitAnkle(profile, footIndex, pose.linkOrientations[shinIndex], footWorld,
        coordinates[footIndex]);
    athleteWorldPose3D(profile, coordinates, rootPosition, rootOrientation, pose);
    return pose.linkPositions[footIndex] + pose.linkOrientations[footIndex].rotate(legs.ankleInFoot[s]);
}

float athleteSoftReach3D(float reach, float start, float end) {
    if (end <= start) return std::min(reach, end);
    if (reach <= start) return reach;
    const float width = end - start;
    return start + width * (1.0f - std::exp(-(reach - start) / width));
}

void athleteLegState3D(const RagdollProfile3D& profile, const AthleteLegs3D& legs, int side,
    const std::vector<Vec3>& coordinates, Vec3 rootPosition, Quaternion rootOrientation, Vec3& ankle, Vec3& knee,
    Quaternion& foot) {
    if (!legs.valid || side < 0 || side > 1) return;
    const std::size_t s = static_cast<std::size_t>(side);
    RagdollAnimationPose3D pose;
    athleteWorldPose3D(profile, coordinates, rootPosition, rootOrientation, pose);
    ankle = pose.linkPositions[legs.foot[s]] + pose.linkOrientations[legs.foot[s]].rotate(legs.ankleInFoot[s]);
    const RagdollLinkDefinition3D& thigh = profile.links[legs.thigh[s]];
    knee = (pose.linkOrientations[legs.thigh[s]] * thigh.modelOrientation.conjugate()).rotate({ 1.0f, 0.0f, 0.0f });
    foot = pose.linkOrientations[legs.foot[s]];
}

} // namespace MatterEngine
