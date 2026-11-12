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
        const std::string& id = profile.links[i].id;
        if (id == "Abdomen") m_twist.emplace_back(i, 0.30f);
        else if (id == "Chest" || id == "UpperChest") m_twist.emplace_back(i, 0.35f);
        else if (id == "LeftThigh" || id == "RightThigh") m_thighs.push_back(i);
    }
    if (m_tracks[m_root] == nullptr) throw std::runtime_error("rastreador: o clipe nao tem a raiz: " + clip.id);
    if (!(clip.durationSeconds > 0.0f)) throw std::runtime_error("rastreador: clipe sem duracao: " + clip.id);
}

void TrackingClipReference3D::pose(float t, Vec3 ground, float yawRadians, std::vector<Vec3>& coordinates, Vec3& root,
    Quaternion& orientation, float torsoTwist, float torsoPitch, float legPitch) const {
    const RagdollProfile3D& profile = *m_profile;
    const float time = std::clamp(t, 0.0f, m_clip->durationSeconds);
    coordinates.assign(profile.links.size(), Vec3 {});
    for (std::size_t i = 0; i < profile.links.size(); ++i) {
        if (i == m_root || m_tracks[i] == nullptr) continue;
        coordinates[i] = sampleAnimationTrackSmooth3D(*m_tracks[i], time, m_clip->durationSeconds, false).jointPositionRadians;
    }
    for (const auto& [i, share] : m_twist) coordinates[i].x += torsoTwist * share;
    // (10/11: a referencia com a coluna acoplada do corpo - a mesma soma por eixo, nas parcelas do perfil.)
    coupleRagdollJointCoordinates3D(profile, coordinates);
    // (A inclinacao, torsoPitch: as coxas compensam a pelve - swing1 menos o angulo, conferido por FK: as coxas ficam no mundo
    // como no clipe, erro de 1 grau a 15 graus.)
    for (const std::size_t i : m_thighs) coordinates[i].y -= torsoPitch - legPitch;
    // A raiz no modelo (como sampleRagdollAnimationPose3D: o modelPosition da
    // pelve mais o deslocamento; a orientacao do modelo vezes o delta), depois
    // girada pelo rumo e posta no ponto do chao mais a altura da pelve em pe (a
    // regra do padrao, DefaultLocomotion3D_Tackle: o modelo tem a pelve na
    // origem; sem ela o clipe comecava 0,95 m abaixo do chao, 29/10).
    const AnimationTransformSample3D sample =
        sampleAnimationTrackSmooth3D(*m_tracks[m_root], time, m_clip->durationSeconds, false);
    const RagdollLinkDefinition3D& link = profile.links[m_root];
    const Quaternion yaw = Quaternion::fromAxisAngle({ 0.0f, 0.0f, 1.0f }, yawRadians);
    root = ground + Vec3 { 0.0f, 0.0f, profile.standingRootHeightMeters }
        + yaw.rotate(link.modelPosition + sample.translationOffsetMeters);
    orientation = (yaw * link.modelOrientation * sample.rotationDelta).normalized();
    // (04/11 14h20, ele: "nao e para arquear a coluna, e sim inclinar o dorso de forma RETA, inclui o quadril" - na lombar
    // ainda dobrava a base da coluna com a pelve parada. Agora a pelve inteira gira no eixo de lado do corpo (+ para a frente)
    // em volta do meio dos quadris, a coluna fica como no clipe - reta sobre a pelve - e as coxas compensam, acima.)
    if (torsoPitch != 0.0f && !m_thighs.empty()) {
        const Quaternion body = (orientation * link.modelOrientation.conjugate()).normalized();
        Vec3 left = body.rotate({ 0.0f, 1.0f, 0.0f });
        left.z = 0.0f;
        const float size = std::sqrt(left.x * left.x + left.y * left.y);
        if (size > 1.0e-4f) {
            left = left * (1.0f / size);
            const Quaternion tilt = Quaternion::fromAxisAngle(left, torsoPitch);
            Vec3 hips {};
            for (const std::size_t i : m_thighs) {
                const Vec3 local = link.modelOrientation.conjugate().rotate(profile.links[i].inboundJoint.anchorModelPosition - link.modelPosition);
                hips = hips + (root + orientation.rotate(local)) * (1.0f / static_cast<float>(m_thighs.size()));
            }
            root = hips + tilt.rotate(root - hips);
            orientation = (tilt * orientation).normalized();
        }
    }
}

void TrackingClipReference3D::frame(float t, Vec3 ground, float yawRadians, TrackingClipFrame3D& out, float torsoTwist,
    float torsoPitch, float legPitch) const {
    const RagdollProfile3D& profile = *m_profile;
    constexpr float H = 1.0f / 120.0f;
    std::vector<Vec3> ahead;
    Vec3 aheadRoot;
    Quaternion aheadOrientation;
    pose(t, ground, yawRadians, out.coordinates, out.rootPosition, out.rootOrientation, torsoTwist, torsoPitch, legPitch);
    // A diferenca para a frente (no fim do clipe, para tras).
    const bool backward = t + H > m_clip->durationSeconds;
    pose(backward ? t - H : t + H, ground, yawRadians, ahead, aheadRoot, aheadOrientation, torsoTwist, torsoPitch, legPitch);
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
