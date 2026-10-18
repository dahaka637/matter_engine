#include "Engine/Tracking/TrackingFeatures3D.hpp"

#include "Engine/Character/RagdollPoseKinematics3D.hpp"
#include "Engine/RL/Body/RLBody3D.hpp"

#include <algorithm>
#include <cmath>

namespace MatterEngine {

TrackingHeadingFrame3D TrackingHeadingFrame3D::at(Vec3 origin, float yawRadians) {
    TrackingHeadingFrame3D frame;
    frame.origin = origin;
    frame.cosYaw = std::cos(yawRadians);
    frame.sinYaw = std::sin(yawRadians);
    return frame;
}

Vec3 TrackingHeadingFrame3D::point(Vec3 world) const { return direction(world - origin); }

Vec3 TrackingHeadingFrame3D::direction(Vec3 world) const {
    // A rotacao inversa do rumo em volta da vertical (z).
    return { cosYaw * world.x + sinYaw * world.y, -sinYaw * world.x + cosYaw * world.y, world.z };
}

std::vector<TrackingSurfaceBox3D> trackingSurfaceBoxes3D(const RagdollProfile3D& profile) {
    std::vector<TrackingSurfaceBox3D> boxes;
    boxes.reserve(profile.links.size());
    for (const RagdollLinkDefinition3D& link : profile.links) {
        const RagdollCapsuleDefinition3D& collider = link.collider;
        TrackingSurfaceBox3D box;
        box.center = collider.localPosition;
        box.orientation = collider.localOrientation;
        if (collider.shape == RagdollColliderShape3D::Capsule) {
            // O eixo da capsula e o X local; o comprimento inclui as tampas.
            const float radius = std::max(collider.radiusMeters, collider.radiusAtPositiveXMeters);
            box.halfExtents = { 0.5f * collider.lengthMeters, radius, radius };
        } else {
            box.halfExtents = collider.boxHalfExtents;
        }
        boxes.push_back(box);
    }
    return boxes;
}

void trackingSurfacePoints3D(const RagdollProfile3D& profile, std::span<const TrackingSurfaceBox3D> boxes,
    std::span<const PhysicsBodyState3D> links, std::vector<Vec3>& positions, std::vector<Vec3>& velocities) {
    const std::size_t count = std::min({ profile.links.size(), boxes.size(), links.size() });
    positions.resize(count * 6);
    velocities.resize(count * 6);
    for (std::size_t i = 0; i < count; ++i) {
        const TrackingSurfaceBox3D& box = boxes[i];
        const PhysicsBodyState3D& state = links[i];
        const Vec3 h = box.halfExtents;
        const Vec3 faces[6] = { { h.x, 0.0f, 0.0f }, { -h.x, 0.0f, 0.0f }, { 0.0f, h.y, 0.0f }, { 0.0f, -h.y, 0.0f },
            { 0.0f, 0.0f, h.z }, { 0.0f, 0.0f, -h.z } };
        for (std::size_t f = 0; f < 6; ++f) {
            const Vec3 local = box.center + box.orientation.rotate(faces[f]);
            const Vec3 world = state.position + state.orientation.rotate(local);
            positions[i * 6 + f] = world;
            velocities[i * 6 + f] = rlLinkPointVelocity3D(profile.links[i], state, world);
        }
    }
}

Vec3 trackingJointAngularVelocity3D(Vec3 target, Vec3 coordinateVelocity, Vec3 measured) {
    const auto finite = [](Vec3 v) { return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z); };
    if (!finite(target) || !finite(coordinateVelocity) || !finite(measured)) return {};
    return ragdollJointRotation3D(measured).conjugate().rotate(ragdollJointJacobian3D(target, coordinateVelocity));
}

} // namespace MatterEngine
