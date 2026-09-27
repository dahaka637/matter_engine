#pragma once
#include "Engine/Physics/PhysicsScene3D.hpp"
#include "Engine/Animation/AnimationClip3D.hpp"
#include "Engine/Locomotion/ContactFootwork3D.hpp"

namespace MatterEngine {
struct RagdollAssistWrench3D {
    std::uint32_t linkIndex = 0;
    Vec3 forceNewtons;
    Vec3 torqueNewtonMeters;
};
struct HybridRagdollAssistOutput3D {
    std::vector<RagdollAssistWrench3D> wrenches;
    Vec3 netForceWorld, netTorqueWorld;
    Vec3 movementForceWorld, liftForceWorld;
    float forceSumNewtons = 0;
    float torqueSumNewtonMeters = 0;
    float supportAuthority = 0;
};
// Persistent state contains authority envelopes only, never world pose targets.
class HybridRagdollAssist3D {
public:
    void reset();
    HybridRagdollAssistOutput3D update(const RagdollProfile3D&, const RagdollState3D&,
        const RagdollAnimationPose3D& localPose, const std::vector<Vec3>& localVelocities,
        const ContactFootworkOutput3D&, Vec3 desiredVelocityHeading,
        float requestedAuthority, float dt, bool inhibited);
    float authority() const { return m_authority; }
private:
    float m_authority = 0;
    float m_contactSeverity = 0;
    float m_contactHold = 0;
    std::vector<float> m_localSeverity;
};
} // namespace MatterEngine
