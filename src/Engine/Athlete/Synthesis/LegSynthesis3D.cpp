#include "Engine/Athlete/Synthesis/LegSynthesis3D.hpp"

#include <cmath>

namespace MatterEngine {

LegSynthesisResult3D synthesizeAthleteLegs3D(const RagdollProfile3D& profile, const AthleteLegs3D& legs,
    const AthleteFootGeometry3D& geometry, const std::array<FootTarget3D, 2>& feet, Vec3 rootPosition,
    Quaternion rootOrientation, std::vector<Vec3>& coordinates, RagdollAnimationPose3D& pose) {
    LegSynthesisResult3D result;
    if (!legs.valid || !geometry.valid || coordinates.size() != profile.links.size()
        || pose.linkPositions.size() != profile.links.size())
        return result;
    bool moved = false;
    for (int side = 0; side < 2; ++side) {
        const std::size_t s = static_cast<std::size_t>(side);
        const FootTarget3D& target = feet[s];
        const std::size_t footLink = legs.foot[s];
        const float away = (pose.linkPositions[footLink] - target.position).lengthSquared();
        const Quaternion now = pose.linkOrientations[footLink];
        const float turn = 1.0f - std::abs(now.x * target.orientation.x + now.y * target.orientation.y
            + now.z * target.orientation.z + now.w * target.orientation.w);
        if (away < 1e-10f && turn < 1e-8f) continue;
        Vec3 ankle, knee;
        Quaternion footWorld;
        athleteLegState3D(profile, legs, side, coordinates, rootPosition, rootOrientation, ankle, knee, footWorld);
        // O joelho do estilo girado o que o pe girou em relacao ao do estilo
        // (no apoio, o rumo travado): o pe alcanca o rumo dele e o polo nao
        // salta nas trocas.
        const float turned = athleteWrapAngle3D(athleteFootYaw3D(geometry, side, target.orientation)
            - athleteFootYaw3D(geometry, side, now));
        knee = athleteYawRotation3D(turned).rotate(knee);
        const Vec3 wanted = target.position + target.orientation.rotate(legs.ankleInFoot[s]);
        const Vec3 reached = solveAthleteLeg3D(profile, legs, side, coordinates, rootPosition, rootOrientation, wanted,
            knee, target.orientation);
        result.reachError[s] = (reached - wanted).length();
        moved = true;
    }
    if (moved) athleteWorldPose3D(profile, coordinates, rootPosition, rootOrientation, pose);
    return result;
}

} // namespace MatterEngine
