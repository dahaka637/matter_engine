#include "Engine/Control/HybridRagdollAssist3D.hpp"
#include <algorithm>
#include <cmath>
namespace MatterEngine {
void HybridRagdollAssist3D::reset() {
    m_channels={1,1,1,1,1,1};m_contactHold=m_severity=0;
}
RagdollAssistChannels3D HybridRagdollAssist3D::update(
    const RagdollState3D& state, const CapsuleTraversalResult3D& traversal,
    float requested, float dt, bool manipulated) {
    if(state.links.empty() || !std::isfinite(dt) || dt<=0) {
        return m_channels={};
    }
    requested=std::isfinite(requested)?std::clamp(requested,0.0f,1.0f):0;
    const float interaction=std::clamp(state.externalInterference,0.0f,1.0f);
    if(interaction>0.01f) {
        m_severity=std::max(m_severity,interaction);
        m_contactHold=0.20f;
    } else if(m_contactHold>0) m_contactHold=std::max(0.0f,m_contactHold-dt);
    else m_severity=std::max(0.0f,m_severity-dt*2.0f);
    const float available=requested*(1-0.75f*m_severity);
    RagdollAssistChannels3D target;
    target.pose=requested;
    target.heading=available;
    target.balance=available;
    target.upright=available;
    target.planarLocomotion=available;
    target.verticalSupport=available;

    switch(traversal.mode) {
    case RagdollTraversalMode3D::Grounded:
        break;
    case RagdollTraversalMode3D::Airborne:
    case RagdollTraversalMode3D::Jumping:
        target.verticalSupport=0;
        target.planarLocomotion*=0.75f;
        target.upright*=0.85f;
        break;
    case RagdollTraversalMode3D::SlidingSteep:
        target.verticalSupport=0;
        target.planarLocomotion*=0.20f;
        target.heading*=0.55f;
        target.upright*=0.45f;
        target.balance*=0.35f;
        break;
    case RagdollTraversalMode3D::PhysicalOverride:
        target.planarLocomotion=target.verticalSupport=target.heading=0;
        target.upright=target.balance=0;
        break;
    }
    if(manipulated) {
        target.planarLocomotion=target.verticalSupport=target.heading=0;
        target.upright=target.balance=0;
        target.pose=0.25f*requested;
    }

    auto approach=[dt](float current,float wanted) {
        if(wanted<current)return wanted;
        const float value=std::min(wanted,current+dt*2.5f);
        return wanted>=1.0f&&value>0.999f?1.0f:value;
    };
    m_channels.planarLocomotion=approach(m_channels.planarLocomotion,target.planarLocomotion);
    m_channels.verticalSupport=approach(m_channels.verticalSupport,target.verticalSupport);
    m_channels.heading=approach(m_channels.heading,target.heading);
    m_channels.upright=approach(m_channels.upright,target.upright);
    m_channels.balance=approach(m_channels.balance,target.balance);
    m_channels.pose=approach(m_channels.pose,target.pose);
    return m_channels;
}
} // namespace MatterEngine
