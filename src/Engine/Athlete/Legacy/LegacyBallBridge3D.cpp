#include "Engine/Athlete/Legacy/LegacyBallBridge3D.hpp"

namespace MatterEngine {

BallGesturePose3D legacyBallGesture3D(const BallBodyTargets3D& t) {
    BallGesturePose3D g;
    if (!t.active) return g;
    g.active = true;
    g.side = t.side;
    g.weight = t.weight;
    g.ankle = t.ankle;
    g.knee = t.knee;
    g.foot = t.foot;
    g.legWeight = t.legWeight;
    g.holdSupport = true;
    g.pelvisDrop = t.pelvisDrop;
    g.pelvisShift = t.pelvisShift;
    g.pelvisYaw = t.pelvisYaw;
    g.pelvisPitch = t.pelvisPitch;
    g.pelvisRoll = t.pelvisRoll;
    g.trunkTwist = t.trunkTwist;
    g.trunkBend = t.trunkBend;
    g.trunkSide = t.trunkSide;
    g.wrist = t.wrist;
    g.armWeight = t.armWeight;
    g.look = t.look;
    g.lookWeight = t.lookWeight;
    g.separateGap = t.separateGap;
    g.separateForward = t.separateForward;
    g.separateWeight = t.separateWeight;
    g.releaseSide = t.releaseSide;
    g.releaseAnkle = t.releaseAnkle;
    g.releaseKnee = t.releaseKnee;
    g.releaseFoot = t.releaseFoot;
    g.releaseWeight = t.releaseWeight;
    return g;
}

std::uint64_t athleteBallKey3D(InflatedBallHandle3D handle) {
    return (static_cast<std::uint64_t>(handle.generation) << 32) | static_cast<std::uint64_t>(handle.index);
}

} // namespace MatterEngine
