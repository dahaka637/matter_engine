#include "Engine/Athlete/Core/FootGeometry3D.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

namespace MatterEngine {

AthleteFootGeometry3D resolveAthleteFootGeometry3D(const RagdollProfile3D& profile, const AthleteLegs3D& legs,
    float toeRollMeters) {
    AthleteFootGeometry3D geometry;
    if (!legs.valid) return geometry;
    for (std::size_t side = 0; side < 2; ++side) {
        const std::size_t index = legs.foot[side];
        if (index >= profile.links.size()) return geometry;
        const RagdollLinkDefinition3D& foot = profile.links[index];
        if (foot.collider.shape != RagdollColliderShape3D::Box) return geometry;
        geometry.link[side] = index;
        geometry.flat[side] = foot.modelOrientation.normalized();
        // Os cantos da caixa no modelo (o pe plano); a face de baixo.
        const Vec3 half = foot.collider.boxHalfExtents;
        std::vector<Vec3> corners;
        for (const float x : { -1.0f, 1.0f })
            for (const float y : { -1.0f, 1.0f })
                for (const float z : { -1.0f, 1.0f }) {
                    const Vec3 local = foot.collider.localPosition
                        + foot.collider.localOrientation.rotate({ half.x * x, half.y * y, half.z * z });
                    corners.push_back(foot.modelOrientation.rotate(local));
                }
        std::sort(corners.begin(), corners.end(), [](Vec3 a, Vec3 b) { return a.z < b.z; });
        corners.resize(4);
        std::sort(corners.begin(), corners.end(), [](Vec3 a, Vec3 b) { return a.x < b.x; });
        const Vec3 heel = (corners[0] + corners[1]) * 0.5f;
        const Vec3 toe = (corners[2] + corners[3]) * 0.5f;
        const Vec3 along = toe - heel;
        const float length = along.length();
        const Vec3 toeBase = length > 1e-4f ? toe - along * (std::min(toeRollMeters, 0.5f * length) / length) : toe;
        const Quaternion back = foot.modelOrientation.conjugate();
        geometry.heel[side] = back.rotate(heel);
        geometry.toeBase[side] = back.rotate(toeBase);
    }
    geometry.valid = true;
    return geometry;
}

float athleteFootYaw3D(const AthleteFootGeometry3D& geometry, int side, Quaternion orientation) {
    const Quaternion flat = geometry.flat[side == 0 ? 0 : 1];
    const Vec3 forward = (orientation * flat.conjugate()).rotate({ 1.0f, 0.0f, 0.0f });
    return std::atan2(forward.y, forward.x);
}

float athleteFootHeading3D(const AthleteFootGeometry3D& geometry, int side, Quaternion orientation) {
    const Quaternion flat = geometry.flat[side == 0 ? 0 : 1];
    const Vec3 left = (orientation * flat.conjugate()).rotate({ 0.0f, 1.0f, 0.0f });
    return std::atan2(-left.x, left.y);
}

Quaternion athleteYawRotation3D(float yawRadians) {
    return Quaternion::fromAxisAngle({ 0.0f, 0.0f, 1.0f }, yawRadians);
}

float athleteWrapAngle3D(float radians) {
    return std::remainder(radians, 6.28318531f);
}

} // namespace MatterEngine
