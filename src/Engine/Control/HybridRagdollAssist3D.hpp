#pragma once
#include "Engine/Physics/PhysicsScene3D.hpp"

namespace MatterEngine {
struct RagdollAssistWrench3D {
    std::uint32_t linkIndex=0;
    Vec3 forceNewtons, torqueNewtonMeters;
};
// Full assistance is pose authority. Only explicit interaction, flight,
// manipulation or a manual override permits physics to deviate from it.
class HybridRagdollAssist3D {
public:
    void reset();
    RagdollAssistChannels3D update(const RagdollState3D&,
        const CapsuleTraversalResult3D&, float requested, float dt,
        bool manipulated);
    const RagdollAssistChannels3D& channels() const { return m_channels; }
private:
    RagdollAssistChannels3D m_channels;
    float m_contactHold=0;
    float m_severity=0;
};
} // namespace MatterEngine
