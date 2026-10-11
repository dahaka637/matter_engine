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

BallGesturePose3D legacyDribbleGesture3D(const DribbleOutput3D& out) {
    BallGesturePose3D g;
    const bool legs = out.legs[0].weight > 0.0f || out.legs[1].weight > 0.0f;
    if (!legs && out.body.weight <= 0.0f) return g;
    g.active = true;
    g.perLeg = true;
    for (std::size_t side = 0; side < 2; ++side) {
        g.legs[side].ankle = out.legs[side].ankle;
        g.legs[side].knee = out.legs[side].knee;
        g.legs[side].foot = out.legs[side].foot;
        g.legs[side].weight = out.legs[side].weight;
    }
    g.holdSupport = true;
    g.weight = out.body.weight;
    g.pelvisDrop = out.body.pelvisDrop;
    g.pelvisShift = out.body.pelvisShift;
    g.pelvisYaw = out.body.pelvisYaw;
    g.pelvisPitch = out.body.pelvisPitch;
    g.pelvisRoll = out.body.pelvisRoll;
    g.trunkTwist = out.body.trunkTwist;
    g.trunkBend = out.body.trunkBend;
    g.trunkSide = out.body.trunkSide;
    g.wrist = out.body.wrist;
    g.armWeight = out.body.armWeight;
    g.look = out.body.look;
    g.lookWeight = out.body.lookWeight;
    return g;
}

std::uint64_t athleteBallKey3D(InflatedBallHandle3D handle) {
    return (static_cast<std::uint64_t>(handle.generation) << 32) | static_cast<std::uint64_t>(handle.index);
}

} // namespace MatterEngine
