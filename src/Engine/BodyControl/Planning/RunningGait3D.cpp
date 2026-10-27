#include "Engine/BodyControl/Planning/RunningGait3D.hpp"

namespace MatterEngine {

Eigen::Vector2d runningLandingPoint3D(const RunningLandingInput3D& in) {
    const Eigen::Vector2d capture = in.centerOfMass + in.velocity * in.remaining + 0.5 * in.stanceTime * in.desiredVelocity
        + (in.velocity - in.desiredVelocity) / in.omega;
    const double home = in.desiredVelocity.norm() < 1e-6 ? in.homeGain : 0.0;
    return capture + in.side * 0.5 * in.stepWidth * in.left - home * in.homeError;
}

bool runningTakeoffReady3D(double verticalSpeed, double previousVerticalSpeed, double takeoffSpeed, double knee, double kneeLimit) {
    if (knee > kneeLimit) return false;
    return verticalSpeed >= takeoffSpeed || (verticalSpeed < previousVerticalSpeed && verticalSpeed > 0.5 * takeoffSpeed);
}

double runningAbsorbAcceleration3D(double height, double verticalSpeed, double targetHeight, double stiffness) {
    return stiffness * stiffness * (targetHeight - height) - 2.0 * stiffness * verticalSpeed;
}

bool runningGaitSelectTrot3D(double speed, bool trotting, double threshold, double hysteresis) {
    return trotting ? speed > threshold - hysteresis : speed >= threshold;
}

} // namespace MatterEngine
