#include "Engine/Tracking/TrackingClipReference3D.hpp"

#include "Engine/RL/Body/RLBody3D.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace MatterEngine {

TrackingClipReference3D::TrackingClipReference3D(const RagdollProfile3D& profile, const AnimationClip3D& clip)
    : m_profile(&profile), m_clip(&clip) {
    m_tracks.assign(profile.links.size(), nullptr);
    for (std::size_t i = 0; i < profile.links.size(); ++i) {
        m_tracks[i] = findAnimationTrack3D(clip, profile.links[i].id);
        if (profile.links[i].parentIndex < 0) m_root = i;
    }
    if (m_tracks[m_root] == nullptr) throw std::runtime_error("rastreador: o clipe nao tem a raiz: " + clip.id);
    if (!(clip.durationSeconds > 0.0f)) throw std::runtime_error("rastreador: clipe sem duracao: " + clip.id);
}

void TrackingClipReference3D::pose(float t, Vec3 ground, float yawRadians, std::vector<Vec3>& coordinates, Vec3& root,
    Quaternion& orientation) const {
    const RagdollProfile3D& profile = *m_profile;
    const float time = std::clamp(t, 0.0f, m_clip->durationSeconds);
    coordinates.assign(profile.links.size(), Vec3 {});
    for (std::size_t i = 0; i < profile.links.size(); ++i) {
        if (i == m_root || m_tracks[i] == nullptr) continue;
        coordinates[i] = sampleAnimationTrackSmooth3D(*m_tracks[i], time, m_clip->durationSeconds, false).jointPositionRadians;
    }
    // A raiz no modelo (como sampleRagdollAnimationPose3D: o modelPosition da
    // pelve mais o deslocamento; a orientacao do modelo vezes o delta), depois
    // girada pelo rumo e posta no ponto do chao.
    const AnimationTransformSample3D sample =
        sampleAnimationTrackSmooth3D(*m_tracks[m_root], time, m_clip->durationSeconds, false);
    const RagdollLinkDefinition3D& link = profile.links[m_root];
    const Quaternion yaw = Quaternion::fromAxisAngle({ 0.0f, 0.0f, 1.0f }, yawRadians);
    root = ground + yaw.rotate(link.modelPosition + sample.translationOffsetMeters);
    orientation = (yaw * link.modelOrientation * sample.rotationDelta).normalized();
}

void TrackingClipReference3D::frame(float t, Vec3 ground, float yawRadians, TrackingClipFrame3D& out) const {
    const RagdollProfile3D& profile = *m_profile;
    constexpr float H = 1.0f / 120.0f;
    std::vector<Vec3> ahead;
    Vec3 aheadRoot;
    Quaternion aheadOrientation;
    pose(t, ground, yawRadians, out.coordinates, out.rootPosition, out.rootOrientation);
    // A diferenca para a frente (no fim do clipe, para tras).
    const bool backward = t + H > m_clip->durationSeconds;
    pose(backward ? t - H : t + H, ground, yawRadians, ahead, aheadRoot, aheadOrientation);
    const float sign = backward ? -1.0f : 1.0f;
    std::vector<Vec3> velocities(profile.links.size());
    for (std::size_t i = 0; i < velocities.size(); ++i) velocities[i] = (ahead[i] - out.coordinates[i]) * (sign / H);
    const Vec3 rootVelocity = (aheadRoot - out.rootPosition) * (sign / H);
    const Vec3 rootAngular =
        rlQuaternionLog3D((aheadOrientation * out.rootOrientation.conjugate()).normalized()) * (sign / H);
    rlBodyLinkStates3D(profile, out.rootPosition, out.rootOrientation, rootVelocity, rootAngular, out.coordinates,
        velocities, out.links);
    out.centerOfMass = rlBodyCenterOfMass3D(profile, out.links);
}

} // namespace MatterEngine
