#pragma once

// Os ajudantes internos da locomocao padrao (a reescrita, R4): as contas que os
// blocos do update usam, num lugar so (antes no namespace anonimo de
// DefaultLocomotion3D.cpp). So os arquivos DefaultLocomotion3D*.cpp incluem.

#include "Engine/Character/DefaultLocomotion3D.hpp"
#include "Engine/Character/RagdollPoseKinematics3D.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>

namespace MatterEngine {
namespace {

constexpr float Pi = 3.14159265358979323846f;
constexpr float Gravity = 9.81f;

inline float wrapAngle(float angle) { return std::remainder(angle, 2.0f * Pi); }
inline float fraction(float value) { return value - std::floor(value); }
inline float smoothStep(float x) {
    x = std::clamp(x, 0.0f, 1.0f);
    return x * x * (3.0f - 2.0f * x);
}
inline Vec3 planar(Vec3 v) { return { v.x, v.y, 0.0f }; }
// Setores da frente (a frente e as diagonais dela).
inline bool frontSector(int sector) { return sector == 0 || sector == 1 || sector == 7; }
inline Quaternion yawRotation(float yaw) { return Quaternion::fromAxisAngle({ 0.0f, 0.0f, 1.0f }, yaw); }
inline float radians(float degrees) { return degrees * Pi / 180.0f; }

// Mola criticamente amortecida (constante tau): chega no alvo sem passar e
// sem salto de velocidade (exata para alvo constante no passo).
inline void spring(float& value, float& rate, float target, float tau, float dt) {
    const float omega = 1.0f / std::max(0.005f, tau);
    const float x = value - target;
    const float e = std::exp(-omega * dt);
    const float temp = (rate + omega * x) * dt;
    rate = (rate - omega * temp) * e;
    value = target + (x + temp) * e;
}

inline float component(Vec3 v, std::size_t axis) { return axis == 0 ? v.x : axis == 1 ? v.y : v.z; }
inline void setComponent(Vec3& v, std::size_t axis, float value) {
    if (axis == 0) v.x = value;
    else if (axis == 1) v.y = value;
    else v.z = value;
}

inline std::size_t findLink(const RagdollProfile3D& profile, const char* id) {
    for (std::size_t i = 0; i < profile.links.size(); ++i)
        if (profile.links[i].id == id) return i;
    return 0;
}

// Leva o eixo de uma junta ate wanted (amount da distancia), nos limites.
inline void blendCoordinate(const RagdollProfile3D& profile, std::vector<Vec3>& coordinates, const char* id,
    std::size_t axis, float wanted, float amount) {
    const std::size_t index = findLink(profile, id);
    if (index == 0 || index >= coordinates.size()) return;
    const RagdollAxisDefinition3D& limit = profile.links[index].inboundJoint.axes[axis];
    if (!limit.enabled) return;
    const float current = component(coordinates[index], axis);
    setComponent(coordinates[index], axis,
        std::clamp(current + (wanted - current) * amount, limit.minimumRadians, limit.maximumRadians));
}

inline void addCoordinate(const RagdollProfile3D& profile, std::vector<Vec3>& coordinates, const char* id,
    std::size_t axis, float value) {
    const std::size_t index = findLink(profile, id);
    if (index == 0 || index >= coordinates.size()) return;
    setComponent(coordinates[index], axis, component(coordinates[index], axis) + value);
}

// Vetor de rotacao (log do quaternion), o inverso de ragdollJointRotation3D.
inline Vec3 rotationVector(Quaternion q) {
    q = q.normalized();
    if (q.w < 0.0f) q = { -q.x, -q.y, -q.z, -q.w };
    const Vec3 v { q.x, q.y, q.z };
    const float n = v.length();
    if (n < 1e-7f) return {};
    return v * (2.0f * std::atan2(n, q.w) / n);
}

inline Quaternion nlerp(Quaternion a, Quaternion b, float t) {
    if (a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w < 0.0f) b = { -b.x, -b.y, -b.z, -b.w };
    return Quaternion { a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t, a.z + (b.z - a.z) * t,
        a.w + (b.w - a.w) * t }.normalized();
}

// Acumula quaternions com peso (mesmo hemisferio do primeiro).
struct QuaternionSum {
    Quaternion sum { 0.0f, 0.0f, 0.0f, 0.0f };
    bool any = false;
    Quaternion reference;
    void add(Quaternion q, float weight) {
        if (weight <= 0.0f) return;
        if (!any) { reference = q; any = true; }
        const float d = q.x * reference.x + q.y * reference.y + q.z * reference.z + q.w * reference.w;
        if (d < 0.0f) q = { -q.x, -q.y, -q.z, -q.w };
        sum = { sum.x + q.x * weight, sum.y + q.y * weight, sum.z + q.z * weight, sum.w + q.w * weight };
    }
    [[nodiscard]] Quaternion result() const { return any ? sum.normalized() : Quaternion {}; }
};

// Instantes (fracao do clipe) em que um apoio comeca e acaba; -1 sem.
inline void contactEvents(const AnimationClip3D& clip, const AnimationScalarTrack3D& track, float& land, float& liftOff) {
    land = -1.0f;
    liftOff = -1.0f;
    const auto& keys = track.keyframes;
    if (keys.size() < 2 || clip.durationSeconds <= 0.0f) return;
    for (std::size_t i = 0; i + 1 < keys.size(); ++i) {
        if (land < 0.0f && keys[i].value < 0.5f && keys[i + 1].value >= 0.5f)
            land = keys[i + 1].timeSeconds / clip.durationSeconds;
        if (liftOff < 0.0f && keys[i].value >= 0.5f && keys[i + 1].value < 0.5f)
            liftOff = keys[i + 1].timeSeconds / clip.durationSeconds;
    }
    if (land >= 1.0f) land -= 1.0f;
}

// Uma trilha do clipe: na reta entre as chaves ou na curva (amostragem
// suave, sem a quebra de velocidade em cada chave).
inline AnimationTransformSample3D sampleTrack(const AnimationTrack3D& track, float time, float duration, bool loop,
    bool smooth) {
    return smooth ? sampleAnimationTrackSmooth3D(track, time, duration, loop)
                  : sampleAnimationTrack3D(track, time, duration, loop);
}

inline AnimationTransformSample3D rootAt(const RagdollProfile3D& profile, const AnimationClip3D& clip, float time,
    bool loop, bool smooth = false) {
    if (const AnimationTrack3D* track = findAnimationTrack3D(clip, profile.links.front().id))
        return sampleTrack(*track, loop ? time : std::clamp(time, 0.0f, clip.durationSeconds),
            clip.durationSeconds, loop, smooth);
    return {};
}

// O pe (origem e orientacao, no referencial da raiz) aonde se pede: a coxa
// e a canela levam o tornozelo, o tornozelo orienta o pe (dentro dos
// limites). weight mistura com a pose de antes.
inline void solveFoot(const RagdollProfile3D& profile, std::vector<Vec3>& coordinates, std::size_t footIndex,
    Vec3 target, Quaternion targetOrientation, float weight) {
    if (weight <= 0.0f || footIndex == 0 || footIndex >= profile.links.size()) return;
    const RagdollLinkDefinition3D& foot = profile.links[footIndex];
    if (foot.parentIndex <= 0) return;
    const std::size_t shinIndex = static_cast<std::size_t>(foot.parentIndex);
    const RagdollLinkDefinition3D& shin = profile.links[shinIndex];
    const Vec3 anchor = foot.inboundJoint.anchorModelPosition;
    const Vec3 anchorInFoot = foot.modelOrientation.conjugate().rotate(anchor - foot.modelPosition);
    const Vec3 anchorInShin = shin.modelOrientation.conjugate().rotate(anchor - shin.modelPosition);
    const std::vector<Vec3> before = coordinates;
    solveRagdollChainPoint3D(profile, coordinates, shinIndex, anchorInShin, 2,
        target + targetOrientation.rotate(anchorInFoot), 1.0f, 12);
    RagdollAnimationPose3D pose;
    buildRagdollLocalPose3D(profile, coordinates, pose);
    // orient_pe = orient_canela * canela^-1 * quadro * R * quadro^-1 * pe
    const Quaternion frame = foot.inboundJoint.frameModelOrientation;
    const Quaternion joint = frame.conjugate() * shin.modelOrientation * pose.linkOrientations[shinIndex].conjugate()
        * targetOrientation * foot.modelOrientation.conjugate() * frame;
    Vec3 local = rotationVector(joint);
    for (std::size_t axis = 0; axis < 3; ++axis) {
        const RagdollAxisDefinition3D& limit = foot.inboundJoint.axes[axis];
        setComponent(local, axis, limit.enabled
            ? std::clamp(component(local, axis), limit.minimumRadians, limit.maximumRadians) : 0.0f);
    }
    coordinates[footIndex] = local;
    if (weight < 1.0f)
        for (std::size_t i = 0; i < coordinates.size(); ++i)
            coordinates[i] = before[i] + (coordinates[i] - before[i]) * weight;
}

// A sola: o canto mais baixo da caixa do pe (z). toeLength: a caixa so ate a
// base dos dedos (tanto atras da frente) - na frente dela os dedos dobram.
inline float soleHeight(const RagdollLinkDefinition3D& link, Vec3 position, Quaternion orientation, float toeLength = 0.0f) {
    const auto& c = link.collider;
    if (!ragdollColliderHasBox3D(c)) return position.z;
    const Vec3 center = position + orientation.rotate(c.localPosition);
    const Quaternion box = (orientation * c.localOrientation).normalized();
    const Vec3 h = c.boxHalfExtents;
    const float front = std::max(-h.x, h.x - std::max(0.0f, toeLength));
    float lowest = 1e9f;
    for (int corner = 0; corner < 8; ++corner) {
        lowest = std::min(lowest, (center + box.rotate({ corner & 1 ? front : -h.x, corner & 2 ? h.y : -h.y,
            corner & 4 ? h.z : -h.z })).z);
    }
    return lowest;
}

// Quanto a sola mais baixa dos pes fica abaixo de groundZ (m; 0 se acima).
inline float feetBelowGround(const RagdollProfile3D& profile, const RagdollAnimationPose3D& pose,
    const std::array<std::size_t, 2>& feet, float groundZ) {
    float below = 0.0f;
    for (const std::size_t foot : feet)
        if (foot != 0 && foot < pose.linkPositions.size())
            below = std::max(below, groundZ - soleHeight(profile.links[foot], pose.linkPositions[foot],
                pose.linkOrientations[foot]));
    return below;
}

inline bool trunkLink(const std::string& id) {
    return id == "Abdomen" || id == "Chest" || id == "UpperChest" || id == "Neck" || id == "Head";
}

inline float tuneTemp(const char* name, float fallback) {
    const char* value = std::getenv(name);
    return value != nullptr ? std::strtof(value, nullptr) : fallback;
}
// O pouso da voadora: a meia-vida e a duracao da inercia (s).
const float AirTackleLandHalfLife = tuneTemp("T_VoadoraMeia", 0.10f);  // TEMPTUNE
const float AirTackleLandSeconds = tuneTemp("T_VoadoraDur", 0.45f);  // TEMPTUNE
const float AirTackleBlendMinimum = tuneTemp("T_VoadoraMistMin", 0.12f);  // TEMPTUNE
const float AirTackleDiveStretch = tuneTemp("T_VoadoraMergulho", 2.0f);  // TEMPTUNE
// A arrancada agachada comeca no maximo neste tempo do clipe (s): o corpo
// entregue mais alto que isso ja e quase de pe.
constexpr float CrouchStartLatest = 0.25f;
} // namespace
} // namespace MatterEngine
