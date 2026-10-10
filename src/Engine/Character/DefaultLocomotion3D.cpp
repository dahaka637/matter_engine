#include "Engine/Character/DefaultLocomotion3D.hpp"

#include "Engine/Character/RagdollPoseKinematics3D.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>

namespace MatterEngine {
namespace {

constexpr float Pi = 3.14159265358979323846f;
constexpr float Gravity = 9.81f;

float wrapAngle(float angle) { return std::remainder(angle, 2.0f * Pi); }
float fraction(float value) { return value - std::floor(value); }
float smoothStep(float x) {
    x = std::clamp(x, 0.0f, 1.0f);
    return x * x * (3.0f - 2.0f * x);
}
Vec3 planar(Vec3 v) { return { v.x, v.y, 0.0f }; }
// Setores da frente (a frente e as diagonais dela).
bool frontSector(int sector) { return sector == 0 || sector == 1 || sector == 7; }
Quaternion yawRotation(float yaw) { return Quaternion::fromAxisAngle({ 0.0f, 0.0f, 1.0f }, yaw); }
float radians(float degrees) { return degrees * Pi / 180.0f; }

// Mola criticamente amortecida (constante tau): chega no alvo sem passar e
// sem salto de velocidade (exata para alvo constante no passo).
void spring(float& value, float& rate, float target, float tau, float dt) {
    const float omega = 1.0f / std::max(0.005f, tau);
    const float x = value - target;
    const float e = std::exp(-omega * dt);
    const float temp = (rate + omega * x) * dt;
    rate = (rate - omega * temp) * e;
    value = target + (x + temp) * e;
}

float component(Vec3 v, std::size_t axis) { return axis == 0 ? v.x : axis == 1 ? v.y : v.z; }
void setComponent(Vec3& v, std::size_t axis, float value) {
    if (axis == 0) v.x = value;
    else if (axis == 1) v.y = value;
    else v.z = value;
}

std::size_t findLink(const RagdollProfile3D& profile, const char* id) {
    for (std::size_t i = 0; i < profile.links.size(); ++i)
        if (profile.links[i].id == id) return i;
    return 0;
}

// Leva o eixo de uma junta ate wanted (amount da distancia), nos limites.
void blendCoordinate(const RagdollProfile3D& profile, std::vector<Vec3>& coordinates, const char* id,
    std::size_t axis, float wanted, float amount) {
    const std::size_t index = findLink(profile, id);
    if (index == 0 || index >= coordinates.size()) return;
    const RagdollAxisDefinition3D& limit = profile.links[index].inboundJoint.axes[axis];
    if (!limit.enabled) return;
    const float current = component(coordinates[index], axis);
    setComponent(coordinates[index], axis,
        std::clamp(current + (wanted - current) * amount, limit.minimumRadians, limit.maximumRadians));
}

void addCoordinate(const RagdollProfile3D& profile, std::vector<Vec3>& coordinates, const char* id,
    std::size_t axis, float value) {
    const std::size_t index = findLink(profile, id);
    if (index == 0 || index >= coordinates.size()) return;
    setComponent(coordinates[index], axis, component(coordinates[index], axis) + value);
}

// Vetor de rotacao (log do quaternion), o inverso de ragdollJointRotation3D.
Vec3 rotationVector(Quaternion q) {
    q = q.normalized();
    if (q.w < 0.0f) q = { -q.x, -q.y, -q.z, -q.w };
    const Vec3 v { q.x, q.y, q.z };
    const float n = v.length();
    if (n < 1e-7f) return {};
    return v * (2.0f * std::atan2(n, q.w) / n);
}

Quaternion nlerp(Quaternion a, Quaternion b, float t) {
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
void contactEvents(const AnimationClip3D& clip, const AnimationScalarTrack3D& track, float& land, float& liftOff) {
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
AnimationTransformSample3D sampleTrack(const AnimationTrack3D& track, float time, float duration, bool loop,
    bool smooth) {
    return smooth ? sampleAnimationTrackSmooth3D(track, time, duration, loop)
                  : sampleAnimationTrack3D(track, time, duration, loop);
}

AnimationTransformSample3D rootAt(const RagdollProfile3D& profile, const AnimationClip3D& clip, float time,
    bool loop, bool smooth = false) {
    if (const AnimationTrack3D* track = findAnimationTrack3D(clip, profile.links.front().id))
        return sampleTrack(*track, loop ? time : std::clamp(time, 0.0f, clip.durationSeconds),
            clip.durationSeconds, loop, smooth);
    return {};
}

// O pe (origem e orientacao, no referencial da raiz) aonde se pede: a coxa
// e a canela levam o tornozelo, o tornozelo orienta o pe (dentro dos
// limites). weight mistura com a pose de antes.
void solveFoot(const RagdollProfile3D& profile, std::vector<Vec3>& coordinates, std::size_t footIndex,
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
float soleHeight(const RagdollLinkDefinition3D& link, Vec3 position, Quaternion orientation, float toeLength = 0.0f) {
    const auto& c = link.collider;
    if (c.shape != RagdollColliderShape3D::Box) return position.z;
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
float feetBelowGround(const RagdollProfile3D& profile, const RagdollAnimationPose3D& pose,
    const std::array<std::size_t, 2>& feet, float groundZ) {
    float below = 0.0f;
    for (const std::size_t foot : feet)
        if (foot != 0 && foot < pose.linkPositions.size())
            below = std::max(below, groundZ - soleHeight(profile.links[foot], pose.linkPositions[foot],
                pose.linkOrientations[foot]));
    return below;
}

bool trunkLink(const std::string& id) {
    return id == "Abdomen" || id == "Chest" || id == "UpperChest" || id == "Neck" || id == "Head";
}

} // namespace

float defaultLocomotionSpeed3D(const DefaultLocomotionSettings3D& settings, float travelRelativeToLookRadians,
    bool sprinting) {
    const std::array<float, 5>& table = sprinting ? settings.sprintSpeeds : settings.trotSpeeds;
    const float position = std::abs(wrapAngle(travelRelativeToLookRadians)) / (Pi * 0.25f);
    const std::size_t k = std::min<std::size_t>(3, static_cast<std::size_t>(position));
    const float t = std::clamp(position - static_cast<float>(k), 0.0f, 1.0f);
    return table[k] + (table[k + 1] - table[k]) * t;
}

TackleClipShape3D tackleClipShape3D(const RagdollProfile3D& profile, const AnimationClip3D& clip) {
    TackleClipShape3D shape;
    if (profile.links.empty() || clip.durationSeconds <= 0.0f) return shape;
    const Quaternion bind = profile.links.front().modelOrientation;
    const auto at = [&](float t) { return rootAt(profile, clip, std::min(t, clip.durationSeconds), false, true); };
    // O rumo da pelve no meio do deslize (a 40% do clipe).
    {
        const AnimationTransformSample3D mid = at(0.4f * clip.durationSeconds);
        const Vec3 forward = (bind * mid.rotationDelta.normalized() * bind.conjugate()).rotate({ 1.0f, 0.0f, 0.0f });
        const float yaw = std::atan2(forward.y, forward.x);
        shape.turnSign = yaw > 0.1f ? 1 : yaw < -0.1f ? -1 : 0;
    }
    // A fase deitada e a velocidade da raiz ao entrar e sair dela.
    {
        const auto speedAt = [&](float t) {
            const AnimationTransformSample3D a = at(std::max(0.0f, t - 0.02f)), b = at(t + 0.02f);
            return std::hypot(b.translationOffsetMeters.x - a.translationOffsetMeters.x,
                b.translationOffsetMeters.y - a.translationOffsetMeters.y) / 0.04f;
        };
        bool inside = false;
        for (float t = 0.0f; t <= clip.durationSeconds; t += 1.0f / 120.0f) {
            const bool low = at(t).translationOffsetMeters.z < -0.40f;
            if (low && !inside && shape.lowEnd <= 0.0f) { shape.lowStart = t; inside = true; }
            if (!low && inside) { shape.lowEnd = t; inside = false; }
        }
        if (inside) shape.lowEnd = clip.durationSeconds;
        shape.diveStart = shape.lowStart;
        while (shape.diveStart > 0.0f && at(shape.diveStart).translationOffsetMeters.z < -0.06f)
            shape.diveStart = std::max(0.0f, shape.diveStart - 1.0f / 120.0f);
        if (shape.lowEnd > shape.lowStart) {
            shape.lowEntrySpeed = speedAt(shape.lowStart);
            shape.lowExitSpeed = speedAt(shape.lowEnd);
            const AnimationTransformSample3D a = at(shape.lowStart), b = at(shape.lowEnd);
            shape.lowMeters = std::hypot(b.translationOffsetMeters.x - a.translationOffsetMeters.x,
                b.translationOffsetMeters.y - a.translationOffsetMeters.y);
        }
    }
    // O deslize: ate a pelve, depois de descer, voltar a meia altura.
    const AnimationTransformSample3D first = at(0.0f);
    float lowest = 0.0f;
    constexpr float Step = 1.0f / 60.0f;
    for (float t = 0.0f; t <= clip.durationSeconds; t += Step) lowest = std::min(lowest, at(t).translationOffsetMeters.z);
    bool down = false;
    for (float t = 0.0f; t <= clip.durationSeconds; t += Step) {
        const AnimationTransformSample3D r = at(t);
        shape.slideMeters = std::hypot(r.translationOffsetMeters.x - first.translationOffsetMeters.x,
            r.translationOffsetMeters.y - first.translationOffsetMeters.y);
        if (r.translationOffsetMeters.z < 0.9f * lowest) down = true;
        if (down && r.translationOffsetMeters.z > 0.5f * lowest) break;
    }
    return shape;
}

int chooseTackleClip3D(const std::array<const AnimationClip3D*, 2>& clips, const std::array<TackleClipShape3D, 2>& shapes,
    float cameraTurnRate, float lateralTravel, std::uint32_t random) {
    int want = 0;
    if (std::abs(cameraTurnRate) > 0.5f) want = cameraTurnRate > 0.0f ? 1 : -1;
    else if (std::abs(lateralTravel) > 0.3f) want = lateralTravel > 0.0f ? 1 : -1;
    int available = 0;
    for (std::size_t k = 0; k < clips.size(); ++k) {
        if (clips[k] == nullptr) continue;
        ++available;
        if (want != 0 && shapes[k].turnSign == want) return static_cast<int>(k);
    }
    if (available == 0) return -1;
    // Ao acaso entre os que existem.
    int pick = static_cast<int>(random % static_cast<std::uint32_t>(available));
    for (std::size_t k = 0; k < clips.size(); ++k) {
        if (clips[k] == nullptr) continue;
        if (pick-- == 0) return static_cast<int>(k);
    }
    return -1;
}

CharacterLocomotionAnimations3D defaultLocomotionAnimations3D(CharacterLocomotionAnimations3D physical,
    const std::vector<std::pair<std::string, std::string>>& roles, const std::vector<AnimationClip3D>& clips) {
    using Slot = const AnimationClip3D* CharacterLocomotionAnimations3D::*;
    static const std::pair<const char*, Slot> slots[] = {
        { "idle", &CharacterLocomotionAnimations3D::idle },
        { "walk", &CharacterLocomotionAnimations3D::walk },
        { "walkBackward", &CharacterLocomotionAnimations3D::walkBackward },
        { "sprint", &CharacterLocomotionAnimations3D::sprint },
        { "sprintBackward", &CharacterLocomotionAnimations3D::sprintBackward },
        { "strafeLeft", &CharacterLocomotionAnimations3D::strafeLeft },
        { "strafeRight", &CharacterLocomotionAnimations3D::strafeRight },
        { "sprintStrafeLeft", &CharacterLocomotionAnimations3D::sprintStrafeLeft },
        { "sprintStrafeRight", &CharacterLocomotionAnimations3D::sprintStrafeRight },
        { "walkForwardLeft", &CharacterLocomotionAnimations3D::walkForwardLeft },
        { "walkForwardRight", &CharacterLocomotionAnimations3D::walkForwardRight },
        { "walkBackwardLeft", &CharacterLocomotionAnimations3D::walkBackwardLeft },
        { "walkBackwardRight", &CharacterLocomotionAnimations3D::walkBackwardRight },
        { "sprintForwardLeft", &CharacterLocomotionAnimations3D::sprintForwardLeft },
        { "sprintForwardRight", &CharacterLocomotionAnimations3D::sprintForwardRight },
        { "sprintBackwardLeft", &CharacterLocomotionAnimations3D::sprintBackwardLeft },
        { "sprintBackwardRight", &CharacterLocomotionAnimations3D::sprintBackwardRight },
    };
    for (const auto& [role, id] : roles) {
        const auto found = std::find_if(clips.begin(), clips.end(),
            [&](const AnimationClip3D& clip) { return clip.id == id; });
        if (found == clips.end()) continue;
        for (const auto& [name, slot] : slots)
            if (role == name) physical.*slot = &*found;
    }
    return physical;
}

void defaultLocomotionPose3D(const RagdollProfile3D& profile, Vec3 rootPosition, Quaternion rootOrientation,
    const std::vector<Vec3>& coordinates, RagdollAnimationPose3D& pose) {
    RagdollAnimationPose3D local;
    buildRagdollLocalPose3D(profile, coordinates, local);
    const std::size_t links = profile.links.size();
    pose.linkPositions.resize(links);
    pose.linkOrientations.resize(links);
    for (std::size_t i = 0; i < links; ++i) {
        pose.linkPositions[i] = rootPosition + rootOrientation.rotate(local.linkPositions[i]);
        pose.linkOrientations[i] = (rootOrientation * local.linkOrientations[i]).normalized();
    }
}

void DefaultLocomotion3D::reset(float facingYawRadians) {
    m_sector = 0;
    m_weights.fill(0.0f);
    m_shape.fill(0.0f);
    m_shapeRates.fill(0.0f);
    m_moving = 0.0f;
    m_movingRate = 0.0f;
    m_upperShape.fill(0.0f);
    m_upperShapeRates.fill(0.0f);
    m_upperWeights.fill(0.0f);
    m_upperMoving = 0.0f;
    m_upperMovingRate = 0.0f;
    m_startTime = -1.0f;
    m_startSprint = false;
    m_dip = 0.0f;
    m_dipRate = 0.0f;
    m_idleWeight = 1.0f;
    m_phase = 0.0f;
    m_strides = 0.0f;
    m_idleTime = 0.0f;
    m_yaw = facingYawRadians;
    m_yawRate = 0.0f;
    m_torsoTwist = 0.0f;
    m_headTurn = 0.0f;
    m_torsoTwistRate = 0.0f;
    m_headTurnRate = 0.0f;
    m_localTravel = 0.0f;
    m_footLocks = {};
    m_stopTarget = -1.0f;
    m_trunkHeight = 0.0f;
    m_landingPrep = 0.0f;
    m_landingPrepRate = 0.0f;
    m_landingLeadSide = -1;
    m_plantWeight = {};
    m_plantRate = {};
    m_upright = 1.0f;
    m_standTurning = false;
    m_started = false;
    m_previousVelocity = {};
    m_acceleration = {};
    m_pathYawValid = false;
    m_pathTurnRate = 0.0f;
    m_lean = {};
    m_leanRate = {};
    m_spineRoll = 0.0f;
    m_arcKind = -1;
    m_arcBlend = 0.0f;
    m_footYaw = {};
    m_terrainDrop = 0.0f;
    m_airTime = 0.0f;
    m_takeoffSpeed = 0.0f;
    m_airWeight = 0.0f;
    m_airRate = 0.0f;
    m_airClip = nullptr;
    m_airPhase = 0.0f;
    m_crouch = 0.0f;
    m_landTime = -1.0f;
    m_landIntensity = 0.0f;
    m_fallSpeed = 0.0f;
    m_landFade = 1.0f;
    m_landingSink = 0.0f;
    m_kick = {};
    m_kickWeight = 0.0f;
    m_kickWeightRate = 0.0f;
    m_tackleClip = nullptr;
    m_tackleFromAir = false;
    m_tackleCooldown = 0.0f;
    m_seedPending = false;
    m_crouchStartPending = false;
    m_crouchStarting = false;
    m_exitInertia.stop();
    m_landTackleInertia.stop();
    m_airTackleClip = nullptr;
    m_airTackleWeight = 0.0f;
    m_tackleEnterStart = 0.0f;
    m_tackleExit = 1.0f;
    m_output = {};
}

namespace {
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

void DefaultLocomotion3D::startCrouchSprint(float travelYawRadians, bool sprint) {
    m_crouchStartPending = true;
    m_crouchStartYaw = travelYawRadians;
    m_crouchStartSprint = sprint;
}

void DefaultLocomotion3D::seed(const DefaultLocomotionSeed3D& seed) {
    m_seed = seed;
    m_seedPending = true;
    m_yaw = seed.pelvisYawRadians;
    m_yawRate = 0.0f;
    m_previousVelocity = planar(seed.velocityWorld);
    m_started = true;
}

void DefaultLocomotion3D::prepare(const RagdollProfile3D& profile, const CharacterLocomotionAnimations3D& animations) {
    const AnimationClip3D* trots[Directions] = { animations.walk, animations.walkForwardLeft, animations.strafeLeft,
        animations.walkBackwardLeft, animations.walkBackward, animations.walkBackwardRight, animations.strafeRight,
        animations.walkForwardRight };
    const AnimationClip3D* sprints[Directions] = { animations.sprint, animations.sprintForwardLeft,
        animations.sprintStrafeLeft, animations.sprintBackwardLeft, animations.sprintBackward,
        animations.sprintBackwardRight, animations.sprintStrafeRight, animations.sprintForwardRight };
    for (std::size_t d = 0; d < Directions; ++d) {
        // Sem o clipe do rumo: a diagonal usa o da frente ou o de costas (o
        // sprint, o sprint deles), o lado e as costas o da frente; sem o de
        // sprint, o de trote. O rumo do clipe usado diz quanto a pelve gira.
        const bool diagonal = d % 2 == 1;
        const std::size_t cardinal = d == 3 || d == 5 ? 4 : 0;
        const AnimationClip3D* trot = trots[d];
        if (trot == nullptr && diagonal) trot = trots[cardinal];
        if (trot == nullptr) trot = animations.walk;
        const AnimationClip3D* sprint = sprints[d];
        if (sprint == nullptr && diagonal) sprint = sprints[cardinal];
        if (sprint == nullptr) sprint = trot;
        m_gaits[d].clip = trot;
        m_gaits[d + Directions].clip = sprint;
    }
    for (Gait& gait : m_gaits) {
        if (gait.clip == nullptr) continue;
        const AnimationClip3D& clip = *gait.clip;
        gait.speed = std::max(0.5f, clip.nominalSpeedMetersPerSecond);
        gait.heading = clip.travelDirectionRadians;
        float land[2], lift[2];
        contactEvents(clip, clip.leftFootContact, land[0], lift[0]);
        contactEvents(clip, clip.rightFootContact, land[1], lift[1]);
        gait.touchdown = land[0] >= 0.0f ? land[0] : 0.0f;
        for (std::size_t side = 0; side < 2; ++side) {
            gait.land[side] = land[side] >= 0.0f ? fraction(land[side] - gait.touchdown) : (side == 0 ? 0.0f : 0.5f);
            gait.liftOff[side] = lift[side] >= 0.0f ? fraction(lift[side] - gait.touchdown)
                : fraction(gait.land[side] + 0.3f);
        }
    }
    // A fase de cruzamento de cada marcha: os tornozelos mais juntos no
    // plano (pela pose do clipe).
    {
        const std::size_t leftFoot = findLink(profile, "LeftFoot"), rightFoot = findLink(profile, "RightFoot");
        for (Gait& gait : m_gaits) {
            if (gait.clip == nullptr || leftFoot == 0 || rightFoot == 0) continue;
            float best = 1e9f;
            for (int k = 0; k < 40; ++k) {
                const float phase = static_cast<float>(k) / 40.0f;
                const RagdollAnimationPose3D pose = sampleRagdollAnimationPose3D(profile, gait.clip,
                    fraction(phase + gait.touchdown) * gait.clip->durationSeconds, true);
                const Vec3 apart = pose.linkPositions[leftFoot] - pose.linkPositions[rightFoot];
                const float distance = std::hypot(apart.x, apart.y);
                if (distance < best) { best = distance; gait.crossing = phase; }
            }
        }
    }
    m_arcs = { animations.runForwardArcLeft, animations.runForwardArcRight, animations.runBackwardArcRight };
    for (std::size_t k = 0; k < m_arcs.size(); ++k) {
        m_arcTouchdown[k] = 0.0f;
        if (m_arcs[k] == nullptr) continue;
        float land, lift;
        contactEvents(*m_arcs[k], m_arcs[k]->leftFootContact, land, lift);
        m_arcTouchdown[k] = land >= 0.0f ? land : 0.0f;
    }
    m_feet = { findLink(profile, "LeftFoot"), findLink(profile, "RightFoot") };
    m_hands = { findLink(profile, "LeftHand"), findLink(profile, "RightHand") };
    m_gestureLegs = resolveBallGestureLegs3D(profile);
    m_upperLinks.assign(profile.links.size(), 0);
    for (std::size_t i = 1; i < profile.links.size(); ++i) {
        const std::string& id = profile.links[i].id;
        m_upperLinks[i] = id.find("Thigh") == std::string::npos && id.find("Shin") == std::string::npos
            && id.find("Foot") == std::string::npos;
    }
    // A amplitude do trote (ver trotAmplitude): a media de cada junta no
    // ciclo de cada marcha e o quanto cada link entra.
    m_amplitudeShare.assign(profile.links.size(), 0.0f);
    for (std::size_t i = 1; i < profile.links.size(); ++i) {
        const std::string& id = profile.links[i].id;
        // (O pe nao: ampliado, o trote de costas e o de lado ficavam com
        // picos maiores.)
        const bool leg = id.find("Thigh") != std::string::npos || id.find("Shin") != std::string::npos;
        const bool limb = leg || id.find("Arm") != std::string::npos || id.find("Hand") != std::string::npos;
        const bool spine = id == "Abdomen" || id == "Chest" || id == "UpperChest";
        m_amplitudeShare[i] = limb ? 1.0f : spine ? -1.0f : 0.0f;
    }
    for (Gait& gait : m_gaits) {
        gait.mean.assign(profile.links.size(), Vec3 {});
        gait.minimum.assign(profile.links.size(), Vec3 {});
        gait.maximum.assign(profile.links.size(), Vec3 {});
        if (gait.clip == nullptr) continue;
        constexpr int Samples = 48;
        for (std::size_t i = 1; i < profile.links.size(); ++i) {
            const AnimationTrack3D* track = findAnimationTrack3D(*gait.clip, profile.links[i].id);
            if (track == nullptr) continue;
            Vec3 sum {};
            Vec3 low { 1e9f, 1e9f, 1e9f }, high { -1e9f, -1e9f, -1e9f };
            for (int k = 0; k < Samples; ++k) {
                const Vec3 v = sampleAnimationTrack3D(*track, gait.clip->durationSeconds * static_cast<float>(k) / Samples,
                    gait.clip->durationSeconds, true).jointPositionRadians;
                sum += v;
                low = { std::min(low.x, v.x), std::min(low.y, v.y), std::min(low.z, v.z) };
                high = { std::max(high.x, v.x), std::max(high.y, v.y), std::max(high.z, v.z) };
            }
            gait.mean[i] = sum * (1.0f / Samples);
            gait.minimum[i] = low;
            gait.maximum[i] = high;
        }
    }
    // Carrinhos: o pe com que comecam apoiados (o mais baixo no primeiro
    // quadro) e a velocidade do comeco (a raiz nos primeiros 0,2 s).
    m_tackles = animations.tackles;
    for (std::size_t k = 0; k < m_tackles.size(); ++k) {
        m_tackleSide[k] = -1;
        m_tackleClipSpeed[k] = 1.0f;
        if (m_tackles[k] == nullptr || m_feet[0] == 0 || m_feet[1] == 0) continue;
        const RagdollAnimationPose3D first = sampleRagdollAnimationPose3D(profile, m_tackles[k], 0.0f, false);
        m_tackleSide[k] = first.linkPositions[m_feet[0]].z < first.linkPositions[m_feet[1]].z ? 0 : 1;
        const AnimationTransformSample3D r0 = rootAt(profile, *m_tackles[k], 0.0f, false);
        const AnimationTransformSample3D r1 = rootAt(profile, *m_tackles[k], 0.2f, false);
        m_tackleClipSpeed[k] = std::max(1.0f, std::hypot(r1.translationOffsetMeters.x - r0.translationOffsetMeters.x,
            r1.translationOffsetMeters.y - r0.translationOffsetMeters.y) / 0.2f);
        m_tackleShape[k] = tackleClipShape3D(profile, *m_tackles[k]);
    }
    m_preparedFor = animations.walk;
}

void DefaultLocomotion3D::update(const RagdollProfile3D& profile,
    const CharacterLocomotionAnimations3D& animations, const DefaultLocomotionInput3D& input) {
    const DefaultLocomotionSettings3D& s = m_settings;
    const float dt = std::max(1e-4f, input.deltaTime);
    const std::size_t links = profile.links.size();
    m_output.valid = false;
    m_output.tackleEnded = false;
    if (links == 0 || animations.walk == nullptr) return;
    if (m_preparedFor != animations.walk) prepare(profile, animations);
    m_tackleCooldown = std::max(0.0f, m_tackleCooldown - dt);
    const RagdollLinkDefinition3D& rootLink = profile.links.front();
    const float viewYaw = input.facingYawRadians;
    const Vec3 viewForward { std::cos(viewYaw), std::sin(viewYaw), 0.0f };
    // O giro da camera (o carrinho vira o corpo para o lado dele).
    {
        const float rate = m_previousViewValid ? wrapAngle(viewYaw - m_previousViewYaw) / dt : 0.0f;
        m_viewTurnRate += (std::clamp(rate, -12.0f, 12.0f) - m_viewTurnRate) * (1.0f - std::exp(-dt / 0.12f));
        m_previousViewYaw = viewYaw;
        m_previousViewValid = true;
    }

    // A capsula: velocidade, aceleracao (filtrada) e o giro do caminho.
    const Vec3 velocity = planar(input.velocityWorld);
    const float speed = velocity.length();
    if (!m_started) {
        m_previousVelocity = velocity;
        m_yaw = viewYaw;
    }
    {
        const Vec3 raw = (velocity - m_previousVelocity) * (1.0f / dt);
        m_acceleration += (raw - m_acceleration) * (1.0f - std::exp(-dt / std::max(0.005f, s.accelerationFilterSeconds)));
        m_previousVelocity = velocity;
        if (speed > 0.5f) {
            const float pathYaw = std::atan2(velocity.y, velocity.x);
            const float rate = m_pathYawValid ? wrapAngle(pathYaw - m_previousPathYaw) / dt : 0.0f;
            const float blend = 1.0f - std::exp(-dt / std::max(0.005f, s.pathTurnFilterSeconds));
            m_pathTurnRate += (std::clamp(rate, -8.0f, 8.0f) - m_pathTurnRate) * blend;
            m_previousPathYaw = pathYaw;
            m_pathYawValid = true;
        } else {
            m_pathTurnRate *= std::exp(-dt / 0.1f);
            m_pathYawValid = false;
        }
    }
    m_started = true;

    // Arrancada agachada (startCrouchSprint): o clipe no rumo pedido, a raiz
    // dele comecando sobre a capsula (que esta embaixo do corpo agachado); a
    // pose do corpo entra pela inercia de quem chama.
    if (m_crouchStartPending) {
        m_crouchStartPending = false;
        if (animations.crouchToSprint != nullptr && animations.crouchToSprint->durationSeconds > 0.05f) {
            const AnimationClip3D& clip = *animations.crouchToSprint;
            m_tackleClip = &clip;
            m_crouchStarting = true;
            m_tackleFromAir = false;
            m_tackleRate = 1.0f;
            // (O clipe sai a ~4,4 m/s: trotando, o caminho vai a velocidade
            // do trote.)
            m_tackleScale = m_crouchStartSprint ? 1.0f
                : std::clamp(defaultLocomotionSpeed3D(s, 0.0f, false) / 4.4f, 0.5f, 1.0f);
            m_tackleYaw = m_crouchStartYaw;
            // O fim: no pico da velocidade da raiz (no plano).
            {
                float best = 0.0f;
                m_crouchExitTime = clip.durationSeconds;
                constexpr float Step = 1.0f / 60.0f;
                for (float t = Step; t <= clip.durationSeconds + 1e-4f; t += Step) {
                    const AnimationTransformSample3D a = rootAt(profile, clip, t - Step, false, s.smoothClipSampling);
                    const AnimationTransformSample3D b = rootAt(profile, clip, t, false, s.smoothClipSampling);
                    const float v = std::hypot(b.translationOffsetMeters.x - a.translationOffsetMeters.x,
                        b.translationOffsetMeters.y - a.translationOffsetMeters.y) / Step;
                    if (v >= best * 0.995f && t > 0.5f * clip.durationSeconds) { best = std::max(best, v); m_crouchExitTime = t; }
                    best = std::max(best, v);
                }
            }
            // O comeco: o tempo em que a pelve do clipe esta na altura da do
            // corpo (o levantar entrega ja subindo; no quadro 0 o clipe a
            // levava de volta ao agachado mais fundo).
            m_tackleTime = 0.0f;
            if (m_seed.feetValid || m_seedPending) {
                const float bodyHeight = m_seed.pelvisWorld.z - input.feetPositionWorld.z;
                for (float t = 0.0f; t <= std::min(CrouchStartLatest, m_crouchExitTime - 0.1f); t += 1.0f / 120.0f) {
                    const AnimationTransformSample3D r = rootAt(profile, clip, t, false, s.smoothClipSampling);
                    const float height = profile.standingRootHeightMeters + rootLink.modelPosition.z + r.translationOffsetMeters.z;
                    m_tackleTime = t;
                    if (height >= bodyHeight) break;
                }
            }
            const AnimationTransformSample3D r0 = rootAt(profile, clip, m_tackleTime, false, s.smoothClipSampling);
            const Vec3 first = yawRotation(m_tackleYaw).rotate({ r0.translationOffsetMeters.x * m_tackleScale,
                r0.translationOffsetMeters.y * m_tackleScale, 0.0f });
            m_tackleStart = { input.feetPositionWorld.x - first.x, input.feetPositionWorld.y - first.y, 0.0f };
            m_tacklePrevious = { input.feetPositionWorld.x, input.feetPositionWorld.y, input.feetPositionWorld.z };
            m_tackleFrom.clear();
            m_footLocks = {};
            m_tackleEntryVelocity = {};
            m_tackleExit = 1.0f;
            m_crouchPreviousFeet = {};
        }
    }
    // Carrinho no ar (a voadora): Ctrl com Shift + W, os tres juntos em
    // qualquer ordem (uma por voo). (O carrinho do pouso so com o Ctrl saiu
    // a pedido do usuario, 05/10 noite: a voadora ja o controla.)
    if (input.grounded) m_airTackleSpent = false;
    // O pouso previsto (balistico) da base da capsula.
    float touchdown = 1e9f;
    if (!input.grounded && input.groundAt) {
        const GroundProbeResult3D probe = input.groundAt(input.feetPositionWorld);
        if (probe.hasSurface) {
            const float rise = input.velocityWorld.z;
            const float fall = std::sqrt(rise * rise
                + 2.0f * Gravity * std::max(0.0f, input.feetPositionWorld.z - probe.pointWorld.z));
            touchdown = (rise + fall) / Gravity;
        }
    }
    if (!input.grounded && input.tackleHeld && input.airTackleCombo && s.airTackle && !m_airTackleSpent
        && input.tackleEnabled && m_tackleClip == nullptr
        && m_airTackleClip == nullptr && m_tackleCooldown <= 0.0f && m_airTime > 0.05f) {
        m_airTackleSpent = true;
        const Vec3 flat = planar(input.velocityWorld);
        const float lateral = flat.lengthSquared() > 1e-4f ? std::sin(wrapAngle(std::atan2(flat.y, flat.x) - viewYaw)) : 0.0f;
        ++m_tackleCount;
        const int chosen = chooseTackleClip3D(m_tackles, m_tackleShape, m_viewTurnRate, lateral,
            (input.varietySeed * 2654435761u) ^ (m_tackleCount * 40503u) ^ static_cast<std::uint32_t>(m_idleTime * 1000.0f));
        if (chosen >= 0) {
            const std::size_t k = static_cast<std::size_t>(chosen);
            m_airTackleClip = m_tackles[k];
            // Do mergulho do clipe (a pose do ar anda ate a fase deitada).
            m_airTackleTime = std::clamp(m_tackleShape[k].diveStart, 0.0f, m_airTackleClip->durationSeconds);
            m_airTackleTarget = std::clamp(std::max(m_tackleShape[k].diveStart, m_tackleShape[k].lowStart - 0.02f),
                0.0f, m_airTackleClip->durationSeconds);
            m_airTackleYaw = flat.length() > 1.0f ? std::atan2(flat.y, flat.x) : viewYaw;
            m_airTackleWeight = 0.0f;
            // Pedida quase pousando, a mistura cabe ate o toque (e o chao
            // continua o clipe de onde ela estiver). O mergulho no ritmo do
            // clipe esticado.
            m_airTackleBlendSeconds = std::clamp(0.85f * touchdown, AirTackleBlendMinimum, 0.12f);
            m_airTackleElapsed = 0.0f;
            m_airTackleDiveSeconds = std::max(0.05f, AirTackleDiveStretch * std::max(0.0f, m_airTackleTarget - m_airTackleTime));
            // Ritmo e percurso pela velocidade do voo (como no pouso; o
            // fisico os usa se ele bater em algo no ar).
            const float ratio = std::clamp(std::max(2.0f, flat.length()) / m_tackleClipSpeed[k], 0.4f, 1.3f);
            m_tackleRate = std::clamp(std::sqrt(ratio), 0.75f, 1.12f);
            m_tackleScale = ratio / m_tackleRate;
            m_tackleEntryVelocity = flat;
        }
    }
    // O pouso da voadora: o deslize do carrinho a partir da fase deitada, com
    // a velocidade que ele trazia; a entrada mistura da pose do ar. Sem a
    // aterrissagem amortecida (o deslize e o pouso).
    if (m_airTackleClip != nullptr && input.grounded) {
        const AnimationClip3D& clip = *m_airTackleClip;
        std::size_t k = 0;
        for (std::size_t j = 0; j < m_tackles.size(); ++j) if (m_tackles[j] == &clip) k = j;
        m_airTackleClip = nullptr;
        m_airTackleWeight = 0.0f;
        m_tackleClip = &clip;
        m_tackleFromAir = true;
        const Vec3 flat = planar(input.velocityWorld);
        const float speedIn = std::max(2.0f, flat.length());
        const float ratio = std::clamp(speedIn / m_tackleClipSpeed[k], 0.4f, 1.3f);
        m_tackleRate = std::clamp(std::sqrt(ratio), 0.75f, 1.12f);
        m_tackleScale = ratio / m_tackleRate;
        m_tackleYaw = m_airTackleYaw;
        // (Do tempo em que a pose do ar estava: o clipe continua.)
        m_tackleTime = std::clamp(m_airTackleTime, 0.0f, clip.durationSeconds);
        m_tackleEnterStart = m_tackleTime;
        {
            float seconds = 0.0f, drop = 0.0f;
            int count = 0;
            for (std::size_t j = 0; j < m_tackles.size(); ++j) {
                const TackleClipShape3D& c = m_tackleShape[j];
                if (m_tackles[j] == nullptr || c.lowEnd <= c.lowStart || c.lowEntrySpeed < 0.5f) continue;
                seconds += c.lowEnd - c.lowStart;
                drop += std::clamp(1.0f - c.lowExitSpeed / c.lowEntrySpeed, 0.0f, 0.95f);
                ++count;
            }
            m_tackleCommonLowSeconds = count > 0 ? seconds / static_cast<float>(count) : 0.0f;
            m_tackleCommonDrop = count > 0 ? drop / static_cast<float>(count) : 0.0f;
        }
        m_tackleEntrySpeedSlide = speedIn;
        m_tacklePathDirection = flat.length() > 0.5f ? flat.normalized() : Vec3 { std::cos(m_tackleYaw), std::sin(m_tackleYaw), 0.0f };
        const AnimationTransformSample3D r0 = rootAt(profile, clip, m_tackleTime, false, s.smoothClipSampling);
        const Vec3 first = yawRotation(m_tackleYaw).rotate({ r0.translationOffsetMeters.x * m_tackleScale,
            r0.translationOffsetMeters.y * m_tackleScale, 0.0f });
        // (A pose do tick anterior: m_output.valid ja foi zerado neste update.)
        const bool previous = m_output.coordinates.size() == links;
        const Vec3 from = previous ? m_output.rootPosition
            : input.feetPositionWorld + Vec3 { 0.0f, 0.0f, profile.standingRootHeightMeters };
        m_tackleStart = { from.x - first.x, from.y - first.y, 0.0f };
        m_tacklePlanar = first;
        m_tacklePrevious = from;
        m_tackleFrom.clear();
        m_footLocks = {};
        m_tackleFromRoot = from;
        m_tackleEntryVelocity = flat;
        m_tackleFromOrientation = m_output.rootOrientation;
        m_tackleExit = 1.0f;
        // A pose do voo e a velocidade dela (do tick anterior a ele).
        if (previous) {
            const float step = 1.0f / std::max(1e-4f, dt);
            const bool known = m_airPreviousCoordinates.size() == links;
            std::vector<Vec3> velocities(links);
            if (known)
                for (std::size_t i = 1; i < links; ++i)
                    velocities[i] = (m_output.coordinates[i] - m_airPreviousCoordinates[i]) * step;
            Vec3 rootVelocity = known ? (m_output.rootPosition - m_airPreviousRoot) * step : input.velocityWorld;
            Vec3 spin {};
            if (known) {
                const Quaternion turn = (m_output.rootOrientation * m_airPreviousOrientation.conjugate()).normalized();
                const float sine = std::sqrt(turn.x * turn.x + turn.y * turn.y + turn.z * turn.z);
                if (sine > 1e-6f)
                    spin = Vec3 { turn.x, turn.y, turn.z }
                        * (2.0f * std::atan2(sine, std::abs(turn.w)) / sine * (turn.w < 0.0f ? -1.0f : 1.0f) * step);
            }
            m_landTackleInertia.start(m_output.coordinates, velocities, m_output.rootPosition, m_output.rootOrientation,
                rootVelocity, spin, AirTackleLandHalfLife, AirTackleLandSeconds);
        }
        // (O pouso e o deslize: sem a aterrissagem amortecida depois.)
        m_airTime = 0.0f;
        m_landTime = -1.0f;
        m_landingPrep = 0.0f;
        m_landingPrepRate = 0.0f;
    }
    // Carrinho: correndo para a frente do olhar, o pedido o comeca ja, com o
    // clipe do pe que vai pousar.
    if (m_tackleClip == nullptr && input.tackleRequest && input.tackleEnabled && input.grounded
        && m_tackleCooldown <= 0.0f && !m_output.coordinates.empty()
        && dot(velocity, viewForward) >= s.tackleMinimumSpeed) {
        // Qual: o que vira o corpo para o lado do giro da camera; parada, para
        // o lado da corrida; senao, ao acaso (pedido do usuario, 05/10 noite).
        const Vec3 travel = planar(input.requestedVelocityWorld).lengthSquared() > 0.01f
            ? planar(input.requestedVelocityWorld) : velocity;
        const float lateral = travel.lengthSquared() > 1e-4f
            ? std::sin(wrapAngle(std::atan2(travel.y, travel.x) - viewYaw)) : 0.0f;
        ++m_tackleCount;
        const int chosen = chooseTackleClip3D(m_tackles, m_tackleShape, m_viewTurnRate, lateral,
            (input.varietySeed * 2654435761u) ^ (m_tackleCount * 40503u) ^ static_cast<std::uint32_t>(m_idleTime * 1000.0f));
        if (chosen >= 0) {
            const std::size_t k = static_cast<std::size_t>(chosen);
            m_tackleClip = m_tackles[k];
            // Ritmo e percurso pela velocidade de entrada (como no fisico).
            const float ratio = std::clamp(dot(velocity, viewForward) / m_tackleClipSpeed[k], 0.4f, 1.3f);
            m_tackleRate = std::clamp(std::sqrt(ratio), 0.75f, 1.12f);
            m_tackleScale = ratio / m_tackleRate;
            // Os dois deslizam o mesmo (pela velocidade de entrada): o
            // caminho de cada um vai ao deslize medio deles.
            {
                float sum = 0.0f;
                int count = 0;
                for (std::size_t j = 0; j < m_tackles.size(); ++j)
                    if (m_tackles[j] != nullptr && m_tackleShape[j].slideMeters > 0.5f) { sum += m_tackleShape[j].slideMeters; ++count; }
                if (count > 0 && m_tackleShape[k].slideMeters > 0.5f && s.tackleEqualSlide)
                    m_tackleScale *= (sum / static_cast<float>(count)) / m_tackleShape[k].slideMeters;
            }
            m_tackleYaw = viewYaw;
            m_tackleTime = 0.0f;
            m_tackleEnterStart = 0.0f;
            m_tackleFromAir = false;
            m_landTackleInertia.stop();
            // O deslize comum (ver o ramo do carrinho): a fase deitada media e
            // a fracao media que a velocidade cai nela.
            {
                float seconds = 0.0f, drop = 0.0f;
                int count = 0;
                for (std::size_t j = 0; j < m_tackles.size(); ++j) {
                    const TackleClipShape3D& c = m_tackleShape[j];
                    if (m_tackles[j] == nullptr || c.lowEnd <= c.lowStart || c.lowEntrySpeed < 0.5f) continue;
                    seconds += c.lowEnd - c.lowStart;
                    drop += std::clamp(1.0f - c.lowExitSpeed / c.lowEntrySpeed, 0.0f, 0.95f);
                    ++count;
                }
                m_tackleCommonLowSeconds = count > 0 ? seconds / static_cast<float>(count) : 0.0f;
                m_tackleCommonDrop = count > 0 ? drop / static_cast<float>(count) : 0.0f;
            }
            m_tackleEntrySpeedSlide = std::max(2.0f, dot(velocity, viewForward));
            m_tacklePathDirection = viewForward;
            const AnimationTransformSample3D r0 = rootAt(profile, *m_tackleClip, 0.0f, false, s.smoothClipSampling);
            const Vec3 first = yawRotation(m_tackleYaw).rotate({ r0.translationOffsetMeters.x * m_tackleScale,
                r0.translationOffsetMeters.y * m_tackleScale, 0.0f });
            m_tackleStart = { m_output.rootPosition.x - first.x, m_output.rootPosition.y - first.y, 0.0f };
            m_tacklePlanar = first;
            m_tacklePrevious = m_output.rootPosition;
            m_tackleFrom = m_output.coordinates;
            m_footLocks = {};
            m_tackleFromRoot = m_output.rootPosition;
            m_tackleEntryVelocity = velocity;
            m_tackleFromOrientation = m_output.rootOrientation;
            m_tackleExit = 1.0f;
        }
    }
    if (m_tackleClip != nullptr) {
        // O carrinho: o clipe no tempo (no ritmo da entrada), a pelve no
        // caminho da raiz dele, escalado; a capsula vai embaixo (laboratorio).
        const AnimationClip3D& clip = *m_tackleClip;
        // Os dois carrinhos deslizam igual (pedido do usuario, 05/10 noite):
        // o deslocamento no plano e o mesmo modelo para os dois, pela
        // velocidade de entrada - no mergulho ela fica; deitado freia a mesma
        // fracao no mesmo tempo (a pose do clipe estica/encolhe nessa fase);
        // levantando segue o perfil do clipe dali. O clipe da a pose, a altura
        // e o rumo do caminho.
        const TackleClipShape3D* shape = nullptr;
        for (std::size_t k = 0; k < m_tackles.size(); ++k) if (m_tackles[k] == &clip) shape = &m_tackleShape[k];
        const bool equalSlide = !m_crouchStarting && s.tackleEqualSlide && shape != nullptr
            && shape->lowEnd > shape->lowStart && m_tackleCommonLowSeconds > 0.05f;
        float timeRate = m_tackleRate;
        if (equalSlide && m_tackleTime >= shape->lowStart && m_tackleTime < shape->lowEnd)
            timeRate *= (shape->lowEnd - shape->lowStart) / m_tackleCommonLowSeconds;
        const float before = m_tackleTime;
        m_tackleTime = std::min(clip.durationSeconds, m_tackleTime + dt * timeRate);
        const AnimationTransformSample3D r = rootAt(profile, clip, m_tackleTime, false, s.smoothClipSampling);
        const Quaternion tackleYaw = yawRotation(m_tackleYaw);
        Vec3 path = tackleYaw.rotate({ r.translationOffsetMeters.x * m_tackleScale,
            r.translationOffsetMeters.y * m_tackleScale, 0.0f });
        if (equalSlide) {
            const AnimationTransformSample3D previous = rootAt(profile, clip, before, false, s.smoothClipSampling);
            Vec3 clipStep { r.translationOffsetMeters.x - previous.translationOffsetMeters.x,
                r.translationOffsetMeters.y - previous.translationOffsetMeters.y, 0.0f };
            const float clipSpeed = clipStep.length() / std::max(1e-5f, m_tackleTime - before);
            if (clipStep.lengthSquared() > 1e-10f) m_tacklePathDirection = tackleYaw.rotate(clipStep.normalized());
            float speed = m_tackleEntrySpeedSlide;
            if (m_tackleTime >= shape->lowStart && m_tackleTime < shape->lowEnd) {
                const float u = (m_tackleTime - shape->lowStart) / (shape->lowEnd - shape->lowStart);
                speed = m_tackleEntrySpeedSlide * (1.0f - m_tackleCommonDrop * u);
            } else if (m_tackleTime >= shape->lowEnd) {
                // Levantando: da velocidade comum ao sair do chao ate o perfil
                // do proprio clipe (escalado pela entrada) em ~0,3 s - o de
                // lado levanta devagar, o sentado sai correndo.
                const float common = m_tackleEntrySpeedSlide * (1.0f - m_tackleCommonDrop);
                const float natural = clipSpeed * m_tackleEntrySpeedSlide / std::max(0.5f, shape->lowEntrySpeed);
                speed = common + (natural - common) * smoothStep((m_tackleTime - shape->lowEnd) / 0.3f);
            }
            m_tacklePlanar += m_tacklePathDirection * (speed * dt);
            path = m_tacklePlanar;
        }
        Vec3 rootPosition { m_tackleStart.x + path.x, m_tackleStart.y + path.y,
            input.feetPositionWorld.z + profile.standingRootHeightMeters + rootLink.modelPosition.z
                + r.translationOffsetMeters.z };
        Quaternion rootOrientation = (tackleYaw * rootLink.modelOrientation * r.rotationDelta.normalized())
            .normalized();
        m_coordinates.assign(links, {});
        for (std::size_t i = 1; i < links; ++i)
            if (const AnimationTrack3D* track = findAnimationTrack3D(clip, profile.links[i].id))
                m_coordinates[i] = sampleTrack(*track, m_tackleTime, clip.durationSeconds, false, s.smoothClipSampling)
                    .jointPositionRadians;
        // Entrada: da pose da corrida (juntas e raiz) ate a do clipe.
        const float enter = smoothStep((m_tackleTime - m_tackleEnterStart) / std::max(0.01f, m_tackleRate * s.tackleBlendInSeconds));
        if (m_tackleFrom.size() == links && enter < 1.0f) {
            for (std::size_t i = 1; i < links; ++i)
                m_coordinates[i] = m_tackleFrom[i] + (m_coordinates[i] - m_tackleFrom[i]) * enter;
            // (A pose de partida segue com o embalo da entrada: parada, a
            // pelve freava de 7 m/s a zero num tick.)
            const Vec3 from = m_tackleFromRoot + m_tackleEntryVelocity * ((m_tackleTime - m_tackleEnterStart) / std::max(0.1f, m_tackleRate));
            rootPosition = from + (rootPosition - from) * enter;
            rootOrientation = nlerp(m_tackleFromOrientation, rootOrientation, enter);
        }
        if (m_landTackleInertia.active()) {
            m_landTackleInertia.apply(m_coordinates, rootPosition, rootOrientation, dt);
            // (A pelve do voo, mais baixa, com a perna do clipe esticada: o pe
            // entrava 18 cm no chao.)
            defaultLocomotionPose3D(profile, rootPosition, rootOrientation, m_coordinates, m_output.pose);
            rootPosition.z += feetBelowGround(profile, m_output.pose, m_feet, input.feetPositionWorld.z);
        }
        defaultLocomotionPose3D(profile, rootPosition, rootOrientation, m_coordinates, m_output.pose);
        m_output.rootPosition = rootPosition;
        m_output.rootOrientation = rootOrientation;
        m_output.coordinates = m_coordinates;
        m_output.tackling = !m_crouchStarting;
        m_output.airTackle = m_tackleFromAir && !m_crouchStarting;
        m_output.crouchStarting = m_crouchStarting;
        m_output.rootScripted = true;
        m_output.tackleRootWorld = rootPosition;
        m_output.tackleVelocity = (rootPosition - m_tacklePrevious) * (1.0f / dt);
        m_tacklePrevious = rootPosition;
        m_output.facingYawRadians = m_tackleYaw;
        m_output.airborne = false;
        m_output.footContact = { 1.0f, 1.0f };
        m_output.footPlanted = { true, true };
        m_output.swingProgress = { -1.0f, -1.0f };
        m_output.secondsToTouchdown = { -1.0f, -1.0f };
        m_output.cycling = false;
        m_output.cyclePlanted = { true, true };
        m_output.secondsToLiftoff = { -1.0f, -1.0f };
        m_output.cycleSecondsToTouchdown = { -1.0f, -1.0f };
        m_output.swingSeconds = { -1.0f, -1.0f };
        m_output.valid = true;
        // Os pes do clipe (a arrancada termina passando para a passada).
        std::array<Vec3, 2> clipFeet {};
        if (m_feet[0] != 0 && m_feet[1] != 0)
            clipFeet = { m_output.pose.linkPositions[m_feet[0]], m_output.pose.linkPositions[m_feet[1]] };
        const bool clipEnded = m_crouchStarting ? m_tackleTime >= m_crouchExitTime - 1e-4f
            : m_tackleTime >= clip.durationSeconds - 1e-4f;
        if (clipEnded) {
            // Fim: de volta a locomocao, misturado a partir desta pose (a
            // raiz em relacao a capsula, que vai estar embaixo dela).
            const bool crouchStart = m_crouchStarting;
            m_tackleClip = nullptr;
            m_crouchStarting = false;
            if (!crouchStart) m_tackleCooldown = s.tackleCooldownSeconds;
            m_exitFrom = m_coordinates;
            m_exitRootOffset = { 0.0f, 0.0f, rootPosition.z - input.feetPositionWorld.z };
            m_exitRootOrientation = rootOrientation;
            m_tackleExit = 0.0f;
            m_yaw = viewYaw;
            if (crouchStart && m_feet[0] != 0 && m_feet[1] != 0) {
                // A arrancada: a passada continua do passo em que o clipe
                // terminou (a fase pelos pes dele), ja na velocidade dele.
                DefaultLocomotionSeed3D next;
                const Vec3 forward = (rootOrientation * rootLink.modelOrientation.conjugate()).rotate({ 1.0f, 0.0f, 0.0f });
                next.pelvisYawRadians = std::atan2(forward.y, forward.x);
                next.velocityWorld = m_output.tackleVelocity;
                next.pelvisWorld = rootPosition;
                next.pelvisVelocityWorld = m_output.tackleVelocity;
                next.feetWorld = clipFeet;
                for (std::size_t side = 0; side < 2; ++side)
                    next.feetVelocityWorld[side] = (clipFeet[side] - m_crouchPreviousFeet[side]) * (1.0f / dt);
                next.feetValid = true;
                seed(next);
                m_started = true;
                // A saida pela inercia: a diferenca para a passada sai com a
                // velocidade que o clipe tinha (juntas e raiz), sem a foto
                // parada da saida do carrinho.
                std::vector<Vec3> velocities(links);
                const float back = std::max(0.0f, m_tackleTime - 1.0f / 60.0f);
                for (std::size_t i = 1; i < links; ++i)
                    if (const AnimationTrack3D* track = findAnimationTrack3D(clip, profile.links[i].id))
                        velocities[i] = (m_coordinates[i] - sampleTrack(*track, back, clip.durationSeconds, false,
                            s.smoothClipSampling).jointPositionRadians) * (60.0f / std::max(0.01f, m_tackleRate));
                const Quaternion turn = (rootOrientation * m_crouchPreviousRoot.conjugate()).normalized();
                const float sine = std::sqrt(turn.x * turn.x + turn.y * turn.y + turn.z * turn.z);
                const Vec3 spin = sine > 1e-6f ? Vec3 { turn.x, turn.y, turn.z }
                    * (2.0f * std::atan2(sine, std::abs(turn.w)) / sine * (turn.w < 0.0f ? -1.0f : 1.0f) / dt) : Vec3 {};
                m_exitInertia.start(m_coordinates, velocities, rootPosition, rootOrientation, m_output.tackleVelocity, spin,
                    0.10f, 0.40f);
                m_tackleExit = 1.0f;
            }
            m_output.tackleEnded = true;
            m_output.tackling = false;
            m_output.airTackle = false;
            m_output.crouchStarting = false;
            m_tackleFromAir = false;
        }
        m_crouchPreviousFeet = clipFeet;
        m_crouchPreviousRoot = rootOrientation;
        return;
    }
    m_output.tackling = false;
    m_output.crouchStarting = false;
    m_output.rootScripted = false;

    // Requested travel: as teclas quando ha; senao, o movimento da capsula.
    const Vec3 requested = planar(input.requestedVelocityWorld);
    const bool moving = speed > 0.3f || requested.length() > 0.1f;
    const Vec3 travel = requested.length() > 0.1f ? requested : velocity;
    const float travelYaw = travel.lengthSquared() > 1e-4f ? std::atan2(travel.y, travel.x) : viewYaw;

    // O setor pela direcao em relacao a camera (oito, com folga).
    const float fromView = wrapAngle(travelYaw - viewYaw);
    const int sectorBefore = m_sector;
    if (moving) {
        const float step = Pi * 0.25f;
        if (std::abs(wrapAngle(fromView - static_cast<float>(m_sector) * step))
            > step * 0.5f + radians(s.sectorHysteresisDegrees))
            m_sector = (static_cast<int>(std::lround(fromView / step)) + static_cast<int>(Directions))
                % static_cast<int>(Directions);
    }

    // Os pesos: o setor (trote/sprint pela velocidade da tabela no rumo) na
    // mola da forma, o quanto anda na sua.
    std::array<float, GaitCount> wanted {};
    float sprint = 0.0f;
    {
        const std::size_t d = static_cast<std::size_t>(m_sector);
        const float trotSpeed = defaultLocomotionSpeed3D(s, fromView, false);
        const float sprintSpeed = defaultLocomotionSpeed3D(s, fromView, true);
        sprint = sprintSpeed > trotSpeed + 0.1f
            ? std::clamp((speed - trotSpeed) / (sprintSpeed - trotSpeed), 0.0f, 1.0f) : 0.0f;
        wanted[d] = 1.0f - sprint;
        wanted[d + Directions] = sprint;
    }
    // O rumo dos clipes do setor, para a pelve.
    float clipHeading = 0.0f;
    {
        float x = 0.0f, y = 0.0f;
        for (std::size_t g = 0; g < GaitCount; ++g) {
            if (wanted[g] <= 0.0f || m_gaits[g].clip == nullptr) continue;
            x += wanted[g] * std::cos(m_gaits[g].heading);
            y += wanted[g] * std::sin(m_gaits[g].heading);
        }
        if (x * x + y * y > 1e-6f) clipHeading = std::atan2(y, x);
    }

    // O rumo da pelve. Andando: o movimento menos o rumo do clipe (e, para a
    // frente, adiantado para dentro da curva). Parado: a coluna e a cabeca
    // olham primeiro; a pelve so gira quando o erro passa do que elas cobrem,
    // e para perto da camera.
    {
        float target = m_yaw;
        float seconds = s.turnSeconds;
        float springSeconds = s.turnSpringSeconds;
        if (moving) {
            m_standTurning = false;
            target = wrapAngle(travelYaw - clipHeading);
            if (frontSector(m_sector))
                target = wrapAngle(target + std::clamp(s.pelvisCurveLeadSeconds * m_pathTurnRate,
                    -s.pelvisCurveLeadMaximum, s.pelvisCurveLeadMaximum) * smoothStep((speed - 1.5f) / 1.5f));
        } else {
            const float error = wrapAngle(viewYaw - m_yaw);
            if (!m_standTurning && std::abs(error) > s.standTurnStart) m_standTurning = true;
            if (m_standTurning) {
                target = wrapAngle(viewYaw - std::copysign(s.standTurnKeep, error));
                if (std::abs(wrapAngle(target - m_yaw)) < 0.03f) m_standTurning = false;
            }
            seconds = s.standTurnSeconds;
            springSeconds = s.standTurnSpringSeconds;
        }
        if (s.adaptiveTransitions) {
            // Mola de segunda ordem com aceleracao e velocidade limitadas.
            const float omega = 1.0f / std::max(0.01f, springSeconds);
            const float acceleration = std::clamp(omega * omega * wrapAngle(target - m_yaw) - 2.0f * omega * m_yawRate,
                -s.maximumTurnAcceleration, s.maximumTurnAcceleration);
            m_yawRate = std::clamp(m_yawRate + acceleration * dt, -s.maximumTurnRateRadiansPerSecond,
                s.maximumTurnRateRadiansPerSecond);
            m_yaw = wrapAngle(m_yaw + m_yawRate * dt);
        } else {
            float step = wrapAngle(target - m_yaw) * (1.0f - std::exp(-dt / std::max(0.005f, seconds)));
            const float limit = s.maximumTurnRateRadiansPerSecond * dt;
            step = std::clamp(step, -limit, limit);
            m_yawRate = step / dt;
            m_yaw = wrapAngle(m_yaw + step);
        }
    }

    // Transicoes adaptativas: frente, lados e costas pelo angulo entre o
    // movimento real e a pelve - entre os dois clipes vizinhos, pela fracao do
    // angulo (o setor so decidiu para onde a pelve vai).
    if (s.adaptiveTransitions) {
        const Vec3 motion = speed > 0.5f ? velocity : requested.length() > 0.1f ? requested : velocity;
        if (motion.lengthSquared() > 1e-4f) m_localTravel = wrapAngle(std::atan2(motion.y, motion.x) - m_yaw);
        const auto around = [](float angle) {
            const float turn = 2.0f * Pi;
            return angle - turn * std::floor(angle / turn);
        };
        wanted.fill(0.0f);
        for (std::size_t tier = 0; tier < 2; ++tier) {
            const float share = tier == 0 ? 1.0f - sprint : sprint;
            if (share <= 0.0f) continue;
            // O clipe logo antes e o logo depois do angulo (anti-horario).
            std::size_t below = GaitCount, above = GaitCount;
            float belowGap = 1e9f, aboveGap = 1e9f;
            for (std::size_t d = 0; d < Directions; ++d) {
                const std::size_t g = d + tier * Directions;
                if (m_gaits[g].clip == nullptr) continue;
                const float down = around(m_localTravel - m_gaits[g].heading);
                const float up = around(m_gaits[g].heading - m_localTravel);
                if (down < belowGap) { belowGap = down; below = g; }
                if (up < aboveGap) { aboveGap = up; above = g; }
            }
            if (below == GaitCount) continue;
            const float span = belowGap + aboveGap;
            if (above == GaitCount || span < 1e-4f || m_gaits[below].clip == m_gaits[above].clip) {
                wanted[below] += share;
                continue;
            }
            const float zone = std::clamp(s.directionBlendDeadZone, 0.0f, 0.45f);
            const float fraction = belowGap / span;
            const float toward = zone > 0.0f ? smoothStep((fraction - zone) / (1.0f - 2.0f * zone)) : fraction;
            wanted[below] += share * (1.0f - toward);
            wanted[above] += share * toward;
        }
    }
    if (m_seedPending) {
        // Semente: as misturas ja onde a velocidade as quer (sem nascer
        // parado e ir acelerando a pose).
        const float seedMoving = input.grounded ? smoothStep(speed / std::max(0.05f, s.idleSpeed)) : 0.0f;
        for (std::size_t g = 0; g < GaitCount; ++g) {
            m_shape[g] = m_upperShape[g] = moving ? wanted[g] : 0.0f;
            m_shapeRates[g] = m_upperShapeRates[g] = 0.0f;
        }
        m_moving = m_upperMoving = seedMoving;
        m_movingRate = m_upperMovingRate = 0.0f;
        m_startTime = 10.0f;
        m_stopTarget = -1.0f;
    }
    float shapeTotal = 0.0f;
    for (std::size_t g = 0; g < GaitCount; ++g) {
        if (moving || m_moving > 0.01f) spring(m_shape[g], m_shapeRates[g], wanted[g], s.gaitBlendSeconds, dt);
        if (m_gaits[g].clip == nullptr) m_shape[g] = 0.0f;
        m_shape[g] = std::max(0.0f, m_shape[g]);
        shapeTotal += m_shape[g];
    }
    if (shapeTotal < 1e-4f) { m_shape[0] = 1.0f; shapeTotal = 1.0f; }
    const float movingWanted = input.grounded ? smoothStep(speed / std::max(0.05f, s.idleSpeed)) : 0.0f;
    spring(m_moving, m_movingRate, movingWanted, s.idleBlendSeconds, dt);
    m_moving = std::clamp(m_moving, 0.0f, 1.0f);
    float total = 0.0f;
    for (std::size_t g = 0; g < GaitCount; ++g) {
        m_weights[g] = m_moving * m_shape[g] / shapeTotal;
        total += m_weights[g];
    }
    m_idleWeight = std::max(0.0f, 1.0f - total);
    std::size_t dominant = 0;
    for (std::size_t g = 1; g < GaitCount; ++g)
        if (m_weights[g] > m_weights[dominant]) dominant = g;
    // O corpo de cima nas molas mais lentas (as pernas ja estao na pose nova,
    // o tronco e os bracos chegam depois).
    float upperIdleWeight = 1.0f;
    {
        float upperShapeTotal = 0.0f;
        for (std::size_t g = 0; g < GaitCount; ++g) {
            if (moving || m_upperMoving > 0.01f)
                spring(m_upperShape[g], m_upperShapeRates[g], wanted[g], s.upperGaitBlendSeconds, dt);
            if (m_gaits[g].clip == nullptr) m_upperShape[g] = 0.0f;
            m_upperShape[g] = std::max(0.0f, m_upperShape[g]);
            upperShapeTotal += m_upperShape[g];
        }
        if (upperShapeTotal < 1e-4f) { m_upperShape[0] = 1.0f; upperShapeTotal = 1.0f; }
        spring(m_upperMoving, m_upperMovingRate, movingWanted, s.upperIdleBlendSeconds, dt);
        m_upperMoving = std::clamp(m_upperMoving, 0.0f, 1.0f);
        for (std::size_t g = 0; g < GaitCount; ++g) {
            m_upperWeights[g] = m_upperMoving * m_upperShape[g] / upperShapeTotal;
            upperIdleWeight -= m_upperWeights[g];
        }
        upperIdleWeight = std::max(0.0f, upperIdleWeight);
    }
    // O peso afunda ao cortar de direcao correndo (mais quanto maior o corte).
    const float dipSpeed = s.transitionDipMeters * 2.71828f / std::max(0.01f, s.transitionDipSeconds);
    if (moving && m_sector != sectorBefore) {
        const float cut = std::abs(wrapAngle(static_cast<float>(m_sector - sectorBefore) * Pi * 0.25f)) / Pi;
        m_dipRate -= dipSpeed * cut * smoothStep((speed - 2.0f) / 3.0f);
    }
    const Quaternion yaw = yawRotation(m_yaw);
    const Vec3 forward { std::cos(m_yaw), std::sin(m_yaw), 0.0f };
    const Vec3 left { -forward.y, forward.x, 0.0f };

    // A passada na velocidade da capsula: ritmo e comprimento (os pes nao
    // escorregam: quem anda mais rapido que o clipe da passos mais longos e
    // um pouco mais rapidos).
    // A amplitude de cada marcha (trote 0..7, corrida 8..15); de lado (2 e 6)
    // a parte trotAmplitudeSide / sprintAmplitudeSide.
    const auto gaitAmplitude = [&](std::size_t g) {
        const bool sprint = g >= Directions;
        const std::size_t d = g % Directions;
        const float share = d == 2 || d == 6 ? (sprint ? s.sprintAmplitudeSide : s.trotAmplitudeSide) : 1.0f;
        return 1.0f + (m_gaitAmplitude[sprint ? 1 : 0] - 1.0f) * share;
    };
    float rate = 0.0f, strideScale = 1.0f;
    if (total > 1e-3f) {
        float frequency = 0.0f, natural = 0.0f;
        for (std::size_t g = 0; g < GaitCount; ++g) {
            if (m_weights[g] <= 0.0f) continue;
            frequency += m_weights[g] / std::max(0.05f, m_gaits[g].clip->durationSeconds);
            natural += m_weights[g] * m_gaits[g].speed;
        }
        frequency /= total;
        natural /= total;
        const float ratio = std::max(0.05f, speed / std::max(0.1f, natural));
        strideScale = std::clamp(std::sqrt(ratio), s.minimumStrideScale, s.maximumStrideScale);
        // O trote mais rapido que o clipe: o movimento cresce (ver
        // trotAmplitude) e o ritmo divide por ele, na parte do trote.
        // (Pela velocidade natural do trote, nao a da mistura com o sprint -
        // ela pulava na troca -, e suavizada no tempo: mudando de uma vez, a
        // pose das pernas e dos bracos saltava - picos de ~900 m/s2 contra
        // ~300 ao parar e ao trocar de marcha.)
        for (std::size_t kind = 0; kind < 2; ++kind) {
            float kindWeight = 0.0f, kindNatural = 0.0f;
            for (std::size_t d = 0; d < Directions; ++d) {
                const std::size_t g = kind * Directions + d;
                kindWeight += m_weights[g];
                kindNatural += m_weights[g] * m_gaits[g].speed;
            }
            const float kindRatio = kindWeight > 1e-3f ? speed / std::max(0.1f, kindNatural / kindWeight) : 1.0f;
            const float amplitudeTarget = s.gaitAmplitude
                ? std::clamp(std::pow(std::max(0.05f, kindRatio), s.gaitAmplitudeExponent),
                    std::min(1.0f, s.gaitAmplitudeMinimum), std::max(1.0f, s.gaitAmplitudeMaximum)) : 1.0f;
            m_gaitAmplitude[kind] += (amplitudeTarget - m_gaitAmplitude[kind])
                * (1.0f - std::exp(-dt / std::max(0.01f, s.gaitAmplitudeSeconds)));
        }
        float amplitudeShare = 0.0f;
        for (std::size_t g = 0; g < GaitCount; ++g)
            amplitudeShare += m_weights[g] * (gaitAmplitude(g) - 1.0f);
        amplitudeShare = 1.0f + amplitudeShare / total;
        const float playRate = std::clamp(ratio / (strideScale * amplitudeShare), s.minimumPlayRate, s.maximumPlayRate);
        rate = frequency * playRate;
    }
    // Parando: o passo termina no proximo cruzamento das pernas, freando (o pe
    // apoiado fica travado; o de tras vem para baixo do corpo).
    if (s.adaptiveTransitions && s.settleStop && input.grounded && requested.length() < 0.1f && m_moving > 0.02f
        && speed < s.stopSettleSpeed && total > 1e-3f) {
        const Gait& gait = m_gaits[dominant];
        if (m_stopTarget < 0.0f) {
            const float first = fraction(gait.crossing - m_phase);
            const float second = fraction(gait.crossing + 0.5f - m_phase);
            m_stopTarget = fraction(m_phase + std::min(first, second));
        }
        float remaining = fraction(m_stopTarget - m_phase);
        if (remaining > 0.75f) remaining = 0.0f;
        rate = remaining / std::max(0.02f, s.stopSettleSeconds);
    } else if (requested.length() >= 0.1f || m_moving <= 0.02f) {
        m_stopTarget = -1.0f;
    }
    // Saindo do parado: o ciclo comeca no cruzamento dos pes (a pose mais
    // perto da parada), nao onde tinha parado.
    // Saindo do parado para a frente: o gesto da arrancada e o peso afundando.
    if (m_moving < 0.03f && movingWanted > 0.05f) {
        const int sector = std::clamp(m_sector, 0, static_cast<int>(Directions) - 1);
        m_phase = m_gaits[static_cast<std::size_t>(sector)].crossing;
        if (m_startTime < 0.0f && s.startGesture) {
            m_startTime = 0.0f;
            m_startSprint = requested.length() > defaultLocomotionSpeed3D(s, fromView, false) + 0.5f;
            m_dipRate -= dipSpeed * (m_startSprint ? 1.0f : 0.6f);
        }
    }
    if (m_moving < 0.03f && !moving) m_startTime = -1.0f;
    if (!frontSector(m_sector) && m_startTime >= 0.0f) m_startTime = 10.0f;
    spring(m_dip, m_dipRate, 0.0f, s.transitionDipSeconds, dt);
    m_dip = std::max(m_dip, -1.5f * s.transitionDipMeters);
    const bool seedTick = m_seedPending;
    if (m_seedPending) {
        // Semente: a fase em que os pes das marchas (pelos pesos), em relacao
        // a pelve e no rumo dela, mais se parecem com os do corpo - onde
        // estao e para onde vao.
        m_seedPending = false;
        if (m_seed.feetValid && total > 0.05f && m_feet[0] != 0 && m_feet[1] != 0) {
            const Quaternion unyaw = yawRotation(-m_seed.pelvisYawRadians);
            std::array<Vec3, 2> body, bodyVelocity;
            for (std::size_t side = 0; side < 2; ++side) {
                body[side] = unyaw.rotate(m_seed.feetWorld[side] - m_seed.pelvisWorld);
                bodyVelocity[side] = unyaw.rotate(m_seed.feetVelocityWorld[side] - m_seed.pelvisVelocityWorld);
            }
            const auto clipFeet = [&](const AnimationClip3D& clip, float phase, float touchdown) {
                const RagdollAnimationPose3D pose = sampleRagdollAnimationPose3D(profile, &clip,
                    fraction(phase + touchdown) * clip.durationSeconds, true);
                const Vec3 pelvisForward = (pose.linkOrientations.front() * rootLink.modelOrientation.conjugate())
                    .rotate({ 1.0f, 0.0f, 0.0f });
                const Quaternion back = yawRotation(-std::atan2(pelvisForward.y, pelvisForward.x));
                std::array<Vec3, 2> out;
                for (std::size_t side = 0; side < 2; ++side)
                    out[side] = back.rotate(pose.linkPositions[m_feet[side]] - pose.linkPositions.front());
                return out;
            };
            constexpr int Samples = 48;
            constexpr float Step = 1.0f / 96.0f;
            constexpr float VelocityWeight = 0.08f;  // s: 1 m/s de diferenca pesa como 8 cm
            float best = 1e9f, bestPhase = m_phase;
            for (int k = 0; k < Samples; ++k) {
                const float phase = static_cast<float>(k) / static_cast<float>(Samples);
                std::array<Vec3, 2> feetAt {}, feetVelocity {};
                for (std::size_t g = 0; g < GaitCount; ++g) {
                    if (m_weights[g] < 0.05f || m_gaits[g].clip == nullptr) continue;
                    const float share = m_weights[g] / total;
                    const auto now = clipFeet(*m_gaits[g].clip, phase, m_gaits[g].touchdown);
                    const auto next = clipFeet(*m_gaits[g].clip, phase + Step, m_gaits[g].touchdown);
                    for (std::size_t side = 0; side < 2; ++side) {
                        feetAt[side] += now[side] * share;
                        feetVelocity[side] += (next[side] - now[side]) * (rate / Step * share);
                    }
                }
                float error = 0.0f;
                for (std::size_t side = 0; side < 2; ++side)
                    error += (feetAt[side] - body[side]).lengthSquared()
                        + (feetVelocity[side] - bodyVelocity[side]).lengthSquared() * VelocityWeight * VelocityWeight;
                if (error < best) { best = error; bestPhase = phase; }
            }
            m_phase = bestPhase;
        }
    }
    {
        const float before = m_phase;
        m_phase = fraction(m_phase + rate * dt);
        m_strides += m_phase < before ? 1.0f : 0.0f;
    }
    m_idleTime += dt;

    // No ar: o pulo pela fase do voo (da saida ao pouso previsto).
    if (!input.grounded) {
        if (m_airTime <= 0.0f) {
            m_takeoffSpeed = std::max(0.0f, input.velocityWorld.z);
            const float along = dot(velocity, forward), aside = dot(velocity, left);
            const AnimationClip3D* clip = animations.jumpStanding;
            if (speed > s.travelJumpSpeed) {
                if (std::abs(along) >= std::abs(aside))
                    clip = along >= 0.0f ? animations.jumpForward : animations.jumpBackward;
                else
                    clip = aside >= 0.0f ? animations.jumpLeft : animations.jumpRight;
            }
            m_airClip = clip != nullptr ? clip : animations.jumpStanding;
        }
        m_airTime += dt;
        const float flight = std::max(0.3f, 2.0f * m_takeoffSpeed / Gravity);
        // (Caindo de um degrau, sem pulo: o meio do voo.)
        m_airPhase = m_takeoffSpeed > 0.5f ? std::clamp(m_airTime / flight, 0.0f, 0.97f) : 0.5f;
        m_fallSpeed = std::max(0.0f, -input.velocityWorld.z);
    } else {
        // Toque: a aterrissagem pela velocidade da queda (um degrau ou um
        // tropeco de um tick no ar nao conta).
        if (m_airTime > 0.12f && s.landingAbsorb) {
            const float span = std::max(0.1f, s.landingSpeedFull - s.landingSpeedMinimum);
            m_landIntensity = std::clamp((m_fallSpeed - s.landingSpeedMinimum) / span, 0.0f, 1.0f);
            if (m_landIntensity > 0.0f) {
                m_landTime = 0.0f;
                m_landDuration = s.landingSecondsMinimum
                    + (s.landingSecondsMaximum - s.landingSecondsMinimum) * m_landIntensity;
                m_landFade = 1.0f;
                m_landPrepDrop = s.landingPrep ? m_landingPrep * s.landingPelvisDrop : 0.0f;
                m_landLeadSide = -1;
                if (m_feet[0] != 0 && m_feet[1] != 0 && m_output.pose.linkPositions.size() > std::max(m_feet[0], m_feet[1]))
                    m_landLeadSide = m_output.pose.linkPositions[m_feet[0]].z <= m_output.pose.linkPositions[m_feet[1]].z ? 0 : 1;
            }
        }
        m_airTime = 0.0f;
    }
    // A aterrissagem: a pelve, a coluna e a cabeca pela forma da referencia
    // (desce ate landingCompressShare do tempo e volta suave); no ar de novo
    // ela sai em ~0,05 s.
    float landTrunk = 0.0f, landUpper = 0.0f, landHead = 0.0f;
    m_landingSink = 0.0f;
    m_landPull = 0.0f;
    m_landPrepCarry = 0.0f;
    if (m_landTime >= 0.0f) {
        m_landTime += dt;
        if (!input.grounded) m_landFade *= std::exp(-dt / 0.05f);
        const float compress = std::clamp(s.landingCompressShare, 0.05f, 0.9f);
        const auto shape = [&](float lag) {
            const float u = (m_landTime - lag) / std::max(0.05f, m_landDuration);
            if (u <= 0.0f || u >= 1.0f) return 0.0f;
            if (u < compress) return std::sin(0.5f * Pi * u / compress);
            return 0.5f * (1.0f + std::cos(Pi * (u - compress) / (1.0f - compress)));
        };
        const float running = 1.0f - (1.0f - s.landingRunningShare) * smoothStep((speed - 2.0f) / 3.0f);
        const float amount = m_landIntensity * m_landFade * running;
        m_landingSink = s.landingDepthMaximum * amount * shape(0.0f);
        m_landPrepCarry = m_landPrepDrop * (1.0f - smoothStep(m_landTime / std::max(0.05f, compress * m_landDuration)));
        // Os pes ao chao no toque (em 0,05 s; o preparo os deixava no ar ate
        // ~0,35 s depois) e soltos na volta.
        {
            const float u = m_landTime / std::max(0.05f, m_landDuration);
            m_landPull = input.grounded ? smoothStep(m_landTime / 0.05f) * (1.0f - smoothStep((u - 0.45f) / 0.35f))
                * m_landFade : 0.0f;
            m_landRunning = smoothStep((speed - 2.0f) / 3.0f);
        }
        landTrunk = s.landingTrunkMaximum * amount * shape(0.03f);
        landUpper = s.landingUpperMaximum * amount * shape(0.07f);
        landHead = s.landingHeadMaximum * amount * shape(0.08f);
        if (m_landTime > m_landDuration + 0.1f || m_landFade < 0.01f) m_landTime = -1.0f;
    }
    spring(m_airWeight, m_airRate, !input.grounded && m_airClip != nullptr ? 1.0f : 0.0f, s.airBlendSeconds, dt);
    m_airWeight = std::clamp(m_airWeight, 0.0f, 1.0f);

    // A pose: as juntas e a raiz de cada clipe, com os pesos.
    m_coordinates.assign(links, {});
    Vec3 rootOffset {};
    QuaternionSum rootRotation;
    std::array<float, 2> contact {};
    const auto sampleInto = [&](const AnimationClip3D& clip, float time, bool loop, float weight,
        float upperWeight, const Gait* gait = nullptr, float amplitude = 1.0f) {
        if (weight <= 1e-4f && upperWeight <= 1e-4f) return;
        for (std::size_t i = 1; i < links; ++i) {
            const float w = i < m_upperLinks.size() && m_upperLinks[i] ? upperWeight : weight;
            if (w <= 1e-4f) continue;
            if (const AnimationTrack3D* track = findAnimationTrack3D(clip, profile.links[i].id)) {
                Vec3 value = sampleTrack(*track, time, clip.durationSeconds, loop, s.smoothClipSampling).jointPositionRadians;
                // O trote mais amplo em volta da media do ciclo, cabendo nos
                // limites da junta: num eixo que passaria do limite, a mesma
                // escala em volta de um centro deslocado (o movimento cresce
                // para o lado com folga - o quadril, que estende so ate 30
                // graus, flexiona mais a frente); se nem assim cabe, a escala
                // que cabe. Cortado no limite, o movimento fazia uma quina
                // (picos de ~700 m/s2 no pe, contra ~300).
                if (gait != nullptr && amplitude != 1.0f && i < gait->mean.size() && i < m_amplitudeShare.size()
                    && m_amplitudeShare[i] != 0.0f) {
                    const float share = m_amplitudeShare[i] > 0.0f ? m_amplitudeShare[i] : s.gaitAmplitudeSpine;
                    const float wanted = 1.0f + (amplitude - 1.0f) * share;
                    const auto& axes = profile.links[i].inboundJoint.axes;
                    const float means[3] = { gait->mean[i].x, gait->mean[i].y, gait->mean[i].z };
                    const float lows[3] = { gait->minimum[i].x, gait->minimum[i].y, gait->minimum[i].z };
                    const float highs[3] = { gait->maximum[i].x, gait->maximum[i].y, gait->maximum[i].z };
                    float* parts[3] = { &value.x, &value.y, &value.z };
                    constexpr float Margin = 0.03f;
                    for (std::size_t a = 0; a < 3; ++a) {
                        float center = means[a], scale = wanted;
                        if (axes[a].enabled && axes[a].maximumRadians - axes[a].minimumRadians > 2.0f * Margin
                            && scale > 1.0f + 1e-4f) {  // (encolhendo, sempre cabe)
                            const float lo = axes[a].minimumRadians + Margin, hi = axes[a].maximumRadians - Margin;
                            const float top = center + (highs[a] - center) * scale;
                            const float bottom = center + (lows[a] - center) * scale;
                            if (top > hi && bottom < lo) {
                                scale = std::max(1.0f, (hi - lo) / std::max(1e-4f, highs[a] - lows[a]));
                                center = scale > 1.0f + 1e-4f ? (scale * lows[a] - lo) / (scale - 1.0f) : center;
                            } else if (top > hi) {
                                center = (scale * highs[a] - hi) / (scale - 1.0f);
                            } else if (bottom < lo) {
                                center = (scale * lows[a] - lo) / (scale - 1.0f);
                            }
                        }
                        *parts[a] = center + (*parts[a] - center) * scale;
                    }
                }
                m_coordinates[i] += value * w;
            }
        }
        if (weight <= 1e-4f) return;
        const AnimationTransformSample3D root = rootAt(profile, clip, time, loop, s.smoothClipSampling);
        rootOffset += root.translationOffsetMeters * weight;
        rootRotation.add(root.rotationDelta.normalized(), weight);
        contact[0] += (clip.leftFootContact.keyframes.empty() ? 1.0f
            : sampleAnimationScalarTrack3D(clip.leftFootContact, time, clip.durationSeconds, loop)) * weight;
        contact[1] += (clip.rightFootContact.keyframes.empty() ? 1.0f
            : sampleAnimationScalarTrack3D(clip.rightFootContact, time, clip.durationSeconds, loop)) * weight;
    };
    const float ground = 1.0f - m_airWeight;
    for (std::size_t g = 0; g < GaitCount; ++g) {
        if (m_gaits[g].clip == nullptr) continue;
        const AnimationClip3D& clip = *m_gaits[g].clip;
        sampleInto(clip, fraction(m_phase + m_gaits[g].touchdown) * clip.durationSeconds, true, m_weights[g] * ground,
            m_upperWeights[g] * ground, &m_gaits[g], gaitAmplitude(g));
    }
    if (animations.idle != nullptr)
        sampleInto(*animations.idle, std::fmod(m_idleTime, std::max(0.1f, animations.idle->durationSeconds)), true,
            m_idleWeight * ground, upperIdleWeight * ground);
    if (m_airClip != nullptr && m_airWeight > 1e-4f)
        sampleInto(*m_airClip, m_airPhase * m_airClip->durationSeconds, false, m_airWeight, m_airWeight);
    Quaternion clipRootRotation = rootRotation.result();

    // O gesto da arrancada (o idle_to_sprint como delta no tronco, nos bracos
    // e na cabeca, por cima do ciclo - o mesmo do fisico).
    if (m_startTime >= 0.0f && animations.idleToSprint != nullptr) {
        const AnimationClip3D& start = *animations.idleToSprint;
        const float duration = m_startSprint ? 0.62f : 0.34f;
        if (m_startTime < duration) {
            const float u = std::clamp(m_startTime / duration, 0.0f, 1.0f);
            const float envelope = smoothStep(u / 0.20f) * (1.0f - smoothStep((u - 0.62f) / 0.38f)) * ground;
            const float sourceTime = std::min(m_startTime * (m_startSprint ? 1.0f : 0.75f), start.durationSeconds);
            const float limit = m_startSprint ? 0.42f : 0.20f;
            for (std::size_t i = 1; i < links; ++i) {
                const std::string& id = profile.links[i].id;
                const bool spine = id == "Abdomen" || id == "Chest" || id == "UpperChest";
                const bool arm = id.find("UpperArm") != std::string::npos || id.find("Forearm") != std::string::npos;
                const bool head = id == "Neck" || id == "Head";
                const float weight = envelope * (spine ? (m_startSprint ? 0.55f : 0.28f)
                    : arm ? (m_startSprint ? 0.62f : 0.30f) : head ? (m_startSprint ? 0.25f : 0.12f) : 0.0f);
                if (weight <= 0.0f) continue;
                const AnimationTrack3D* track = findAnimationTrack3D(start, id);
                if (track == nullptr) continue;
                const Vec3 first = sampleTrack(*track, 0.0f, start.durationSeconds, false, s.smoothClipSampling).jointPositionRadians;
                const Vec3 now = sampleTrack(*track, sourceTime, start.durationSeconds, false, s.smoothClipSampling)
                    .jointPositionRadians;
                const Vec3 delta { std::clamp(now.x - first.x, -limit, limit), std::clamp(now.y - first.y, -limit, limit),
                    std::clamp(now.z - first.z, -limit, limit) };
                m_coordinates[i] += delta * weight;
            }
        }
        m_startTime += dt;
    }

    // Curva: a corrida em arco na coluna (o banco e a torcao; a flexao fica a
    // da marcha) e um pouco nas pernas.
    const float longitudinal = dot(velocity, forward);
    const float curveAcceleration = longitudinal * m_pathTurnRate;
    // Quanto da curva vale pela velocidade: nada devagar, pouco no trote,
    // inteira correndo rapido.
    const float curveShare = smoothStep((speed - 1.0f) / 2.0f)
        * (s.curveTrotShare + (1.0f - s.curveTrotShare)
            * smoothStep((speed - s.curveSprintFrom) / std::max(0.1f, s.curveSprintSpan)));
    {
        const float desired = std::clamp(smoothStep((std::abs(curveAcceleration) - 0.35f - s.curveLeanDeadZone) / 3.5f),
            0.0f, 1.0f) * s.curveClipMaximum * curveShare * ground;
        int kind = -1;
        if (desired > 0.001f && m_moving > 0.5f) {
            if (frontSector(m_sector)) kind = curveAcceleration > 0.0f ? 0 : 1;
            else if (m_sector >= 3 && m_sector <= 5 && curveAcceleration < 0.0f) kind = 2;
            if (kind >= 0 && m_arcs[static_cast<std::size_t>(kind)] == nullptr) kind = -1;
        }
        const bool changing = kind != m_arcKind;
        const float target = changing ? 0.0f : desired;
        const float limit = (target > m_arcBlend ? 2.5f : 4.0f) * dt;
        m_arcBlend += std::clamp(target - m_arcBlend, -limit, limit);
        if (changing && m_arcBlend <= 0.001f) m_arcKind = kind;
        if (m_arcKind >= 0 && m_arcBlend > 0.0f) {
            const AnimationClip3D& arc = *m_arcs[static_cast<std::size_t>(m_arcKind)];
            const float time = fraction(m_phase + m_arcTouchdown[static_cast<std::size_t>(m_arcKind)]) * arc.durationSeconds;
            for (std::size_t i = 1; i < links; ++i) {
                const std::string& id = profile.links[i].id;
                const bool spine = id == "Abdomen" || id == "Chest" || id == "UpperChest";
                const bool leg = id.find("Thigh") != std::string::npos || id.find("Shin") != std::string::npos
                    || id.find("Foot") != std::string::npos;
                if (!spine && !leg) continue;
                const AnimationTrack3D* track = findAnimationTrack3D(arc, id);
                if (track == nullptr) continue;
                Vec3 pose = sampleTrack(*track, time, arc.durationSeconds, true, s.smoothClipSampling).jointPositionRadians;
                if (spine) pose.y = m_coordinates[i].y;
                m_coordinates[i] += (pose - m_coordinates[i]) * (m_arcBlend * (spine ? 1.0f : s.curveClipLegShare));
            }
        }
    }

    // Carga do pulo: o agachar do preparo, mais fundo com a carga.
    {
        const float wantedCrouch = input.grounded ? std::clamp(input.jumpCrouch, 0.0f, 1.0f) : 0.0f;
        m_crouch += (wantedCrouch - m_crouch) * (1.0f - std::exp(-dt / 0.05f));
        if (m_crouch > 0.001f && animations.jumpPrep != nullptr) {
            const AnimationClip3D& prep = *animations.jumpPrep;
            const float weight = smoothStep(m_crouch / 0.15f);
            const float time = m_crouch * prep.durationSeconds;
            for (std::size_t i = 1; i < links; ++i) {
                const AnimationTrack3D* track = findAnimationTrack3D(prep, profile.links[i].id);
                if (track == nullptr) continue;
                const Vec3 pose = sampleTrack(*track, time, prep.durationSeconds, false, s.smoothClipSampling).jointPositionRadians;
                m_coordinates[i] += (pose - m_coordinates[i]) * (weight * (trunkLink(profile.links[i].id) ? 0.2f : 1.0f));
            }
            const AnimationTransformSample3D root = rootAt(profile, prep, time, false, s.smoothClipSampling);
            clipRootRotation = nlerp(clipRootRotation, root.rotationDelta.normalized(), weight);
            rootOffset.z += (root.translationOffsetMeters.z - rootOffset.z) * weight;
        }
    }

    // "Vida": passada a passada nos bracos, tronco e cabeca; parado, a
    // respiracao e o acomodar. Cada ciclo com o seu peso (o do corpo de cima)
    // e o parado com o dele, somados - so do dominante, o tronco e os bracos
    // saltavam quando ele mudava ou o corpo passava de meio parado.
    if (input.motionVariety > 0.0f) {
        const float amount = std::clamp(input.motionVariety, 0.0f, 2.0f);
        const float strides = m_strides + m_phase;
        std::vector<Vec3> delta(links);
        const auto add = [&](float weight) {
            for (std::size_t i = 1; i < links; ++i) m_coordinates[i] += delta[i] * weight;
        };
        for (std::size_t g = 0; g < GaitCount; ++g) {
            const float weight = m_upperWeights[g] * ground;
            if (weight <= 1e-3f || m_gaits[g].clip == nullptr) continue;
            std::fill(delta.begin(), delta.end(), Vec3 {});
            const float clipPhase = fraction(m_phase + m_gaits[g].touchdown);
            characterGaitVariety3D(profile, *m_gaits[g].clip, clipPhase, clipPhase, strides, input.varietySeed,
                amount, delta);
            add(weight);
        }
        if (upperIdleWeight * ground > 1e-3f) {
            std::fill(delta.begin(), delta.end(), Vec3 {});
            characterIdleVariety3D(profile, m_idleTime, input.varietySeed, amount, delta);
            add(upperIdleWeight * ground);
        }
    }

    // Vida do tronco e da cabeca: cada marcha da frente e de costas com a sua
    // classe (trote/sprint), pelo peso dela - a altura pelo das pernas, o
    // tronco e a cabeca pelo do corpo de cima.
    if (s.trunkMotion && s.trunkMotionAmount > 0.0f && ground > 0.0f) {
        const float twice = 4.0f * Pi * m_phase, once = 2.0f * Pi * m_phase;
        float pitch = 0.0f, headPitch = 0.0f, headRoll = 0.0f;
        m_trunkHeight = 0.0f;
        for (std::size_t g = 0; g < GaitCount; ++g) {
            if (m_gaits[g].clip == nullptr) continue;
            const float c = std::cos(m_gaits[g].heading);
            if (std::abs(c) < 0.7f) continue;
            const DefaultTrunkMotion3D& m = s.trunkMotions[(g >= Directions ? 1u : 0u) + (c < 0.0f ? 2u : 0u)];
            const float legs = m_weights[g], upper = m_upperWeights[g];
            m_trunkHeight += legs * (m.heightCos2 * std::cos(twice) + m.heightSin2 * std::sin(twice));
            pitch += upper * (m.pitchMean + m.pitchCos2 * std::cos(twice) + m.pitchSin2 * std::sin(twice));
            headPitch += upper * (m.headPitchMean + m.headPitchCos2 * std::cos(twice)
                + m.headPitchSin2 * std::sin(twice));
            headRoll += upper * (m.headRollCos1 * std::cos(once) + m.headRollSin1 * std::sin(once));
        }
        const float amount = s.trunkMotionAmount * ground;
        m_trunkHeight *= amount;
        addCoordinate(profile, m_coordinates, "Abdomen", 1, pitch * amount * 0.30f);
        addCoordinate(profile, m_coordinates, "Chest", 1, pitch * amount * 0.35f);
        addCoordinate(profile, m_coordinates, "UpperChest", 1, pitch * amount * 0.35f);
        addCoordinate(profile, m_coordinates, "Neck", 1, headPitch * amount * 0.40f);
        addCoordinate(profile, m_coordinates, "Head", 1, headPitch * amount * 0.60f);
        addCoordinate(profile, m_coordinates, "Neck", 2, headRoll * amount * 0.40f);
        addCoordinate(profile, m_coordinates, "Head", 2, headRoll * amount * 0.60f);
    } else {
        m_trunkHeight = 0.0f;
    }

    // O olhar: a coluna devolve a maior parte do giro da pelve para a camera,
    // a cabeca completa (ela mais rapido). A inclinacao do olhar no peito,
    // no pescoco e na cabeca. Parado, o pescoco e a cabeca eretos.
    {
        const float sprinting = frontSector(m_sector) ? smoothStep((speed - 5.0f) / 1.5f) : 0.0f;
        const float torsoLimit = radians(s.torsoTwistDegrees
            + (s.sprintTorsoTwistDegrees - s.torsoTwistDegrees) * sprinting);
        const float headLimit = radians(s.headTurnDegrees);
        const float error = std::clamp(wrapAngle(viewYaw - m_yaw), -(torsoLimit + headLimit), torsoLimit + headLimit);
        const float torsoTarget = std::clamp(error * s.torsoLookShare, -torsoLimit, torsoLimit);
        if (s.adaptiveTransitions)
            spring(m_torsoTwist, m_torsoTwistRate, torsoTarget, s.torsoLookSeconds, dt);
        else
            m_torsoTwist += (torsoTarget - m_torsoTwist) * (1.0f - std::exp(-s.torsoLookRate * dt));
        const float headTarget = std::clamp(error - m_torsoTwist, -headLimit, headLimit);
        if (s.adaptiveTransitions)
            spring(m_headTurn, m_headTurnRate, headTarget, s.headLookSeconds, dt);
        else
            m_headTurn += (headTarget - m_headTurn) * (1.0f - std::exp(-s.headLookRate * dt));
        addCoordinate(profile, m_coordinates, "Abdomen", 0, m_torsoTwist * 0.30f);
        addCoordinate(profile, m_coordinates, "Chest", 0, m_torsoTwist * 0.35f);
        addCoordinate(profile, m_coordinates, "UpperChest", 0, m_torsoTwist * 0.35f);
        addCoordinate(profile, m_coordinates, "Neck", 0, m_headTurn * 0.40f);
        addCoordinate(profile, m_coordinates, "Head", 0, m_headTurn * 0.60f);
        const float pitch = input.lookPitchRadians;
        addCoordinate(profile, m_coordinates, "UpperChest", 1, -pitch * 0.14f);
        addCoordinate(profile, m_coordinates, "Neck", 1, -pitch * 0.36f);
        addCoordinate(profile, m_coordinates, "Head", 1, -pitch * 0.56f);
        m_upright += ((1.0f - m_moving) * ground - m_upright) * (1.0f - std::exp(-dt / 0.3f));
        addCoordinate(profile, m_coordinates, "Neck", 1, -0.20f * m_upright);
        addCoordinate(profile, m_coordinates, "Head", 1, -0.07f * m_upright);
        addCoordinate(profile, m_coordinates, "UpperChest", 1, -0.08f * m_upright);
        addCoordinate(profile, m_coordinates, "Chest", 1, -0.04f * m_upright);
    }
    if (landTrunk > 0.0f || landUpper > 0.0f) {
        // A aterrissagem: a coluna flexiona (baixa e alta) e a cabeca volta
        // um pouco.
        addCoordinate(profile, m_coordinates, "Abdomen", 1, landTrunk * 0.5f);
        addCoordinate(profile, m_coordinates, "Chest", 1, landTrunk * 0.5f);
        addCoordinate(profile, m_coordinates, "UpperChest", 1, landUpper);
        addCoordinate(profile, m_coordinates, "Neck", 1, -landHead * 0.5f);
        addCoordinate(profile, m_coordinates, "Head", 1, -landHead * 0.5f);
    }

    // Inclinacao: na curva o corpo inteiro vai para dentro (atan(v w / g),
    // pela velocidade); arrancada e freada um pouco para a frente e para
    // tras. Em volta dos pes.
    const float lateralAcceleration = speed * m_pathTurnRate;
    const float leaningAcceleration = std::copysign(std::max(0.0f, std::abs(lateralAcceleration) - s.curveLeanDeadZone),
        lateralAcceleration);
    const float bodyCurveLean = std::clamp(std::atan(leaningAcceleration / Gravity) * curveShare,
        -s.curveLeanMaximum, s.curveLeanMaximum) * ground;
    {
        const float forwardAcceleration = dot(m_acceleration, forward);
        Vec3 wantedLean { bodyCurveLean,
            std::clamp(std::atan(forwardAcceleration / Gravity) * s.forwardLeanShare,
                s.forwardLeanMinimum, s.forwardLeanMaximum) * ground, 0.0f };
        const float omega = s.leanRate;
        const float decay = std::exp(-omega * dt);
        for (int axis = 0; axis < 2; ++axis) {
            float& value = axis == 0 ? m_lean.x : m_lean.y;
            float& rateValue = axis == 0 ? m_leanRate.x : m_leanRate.y;
            const float target = axis == 0 ? wantedLean.x : wantedLean.y;
            const float error = value - target;
            const float impulse = rateValue + error * omega;
            value = target + (error + impulse * dt) * decay;
            rateValue = (rateValue - impulse * omega * dt) * decay;
        }
    }

    // A raiz no mundo: a base da capsula mais a altura da pelve em pe (a
    // origem do modelo e a pelve), girada pelo rumo, mais o que o clipe move
    // a pelve; descida do terreno; inclinacao em volta dos pes.
    const Vec3 feet = input.feetPositionWorld;
    Quaternion rootOrientation = (yaw * rootLink.modelOrientation * clipRootRotation).normalized();
    Vec3 rootPosition = feet + Vec3 { 0.0f, 0.0f, profile.standingRootHeightMeters - m_terrainDrop + m_dip
        + m_trunkHeight - (m_landTime >= 0.0f ? m_landPrepCarry : s.landingPrep ? m_landingPrep * s.landingPelvisDrop : 0.0f)
        - m_landingSink }
        + yaw.rotate(rootLink.modelPosition + rootOffset);
    {
        const Vec3 axis = left * m_lean.y - forward * m_lean.x;
        const float angle = axis.length();
        if (angle > 1e-5f) {
            const Quaternion tilt = Quaternion::fromAxisAngle(axis * (1.0f / angle), angle);
            rootPosition = feet + tilt.rotate(rootPosition - feet);
            rootOrientation = (tilt * rootOrientation).normalized();
        }
    }
    // Preparo do pulo: o clipe dobra os joelhos sem descer a raiz - os pes
    // subiam ~20 cm e ele ficava agachado no ar (visto pelo usuario, 05/10
    // noite). A pelve desce ate o pe mais baixo tocar o chao - tambem na
    // saida do chao, enquanto o preparo se desfaz (as pernas estendem no
    // impulso; cortando no primeiro tick no ar a pelve saltava 20 cm).
    if (m_crouch > 0.001f && animations.jumpPrep != nullptr && m_feet[0] != 0 && m_feet[1] != 0) {
        RagdollAnimationPose3D crouched;
        defaultLocomotionPose3D(profile, rootPosition, rootOrientation, m_coordinates, crouched);
        float lowest = 1e9f;
        for (std::size_t side = 0; side < 2; ++side)
            lowest = std::min(lowest, soleHeight(profile.links[m_feet[side]], crouched.linkPositions[m_feet[side]],
                crouched.linkOrientations[m_feet[side]]));
        rootPosition.z -= std::max(0.0f, lowest - feet.z) * smoothStep(m_crouch / 0.15f);
    }

    // Manejo de bola (rework de 06/10): o gesto do corpo inteiro por cima da
    // locomocao - a pelve gira e desce em volta do apoio (preso onde a
    // locomocao o pos), tronco, bracos e cabeca, e a perna que joga pelo IK
    // analitico (armada com o joelho dobrado, desdobrando ate a bola).
    int ballSide = -1;
    if (m_feet[0] != 0 && m_feet[1] != 0) {
        RagdollAnimationPose3D style;
        defaultLocomotionPose3D(profile, rootPosition, rootOrientation, m_coordinates, style);
        for (std::size_t side = 0; side < 2; ++side) {
            m_output.styleFootPosition[side] = style.linkPositions[m_feet[side]];
            m_output.styleFootOrientation[side] = style.linkOrientations[m_feet[side]];
        }
    }
    if (input.ballGesture.active && m_gestureLegs.valid) {
        applyBallGesturePose3D(profile, m_gestureLegs, input.ballGesture, m_coordinates, rootPosition,
            rootOrientation, true, true);
        if (input.ballGesture.legWeight > 0.3f) ballSide = input.ballGesture.side;
    }
    const Quaternion rootInverse = rootOrientation.conjugate();

    // Preparo do pouso (ver landingPrep): descendo, perto do toque, a perna
    // aponta na linha da velocidade e a coluna acompanha. Depois do toque ele
    // sai numa mola de 0,06 s (de uma vez, bracos e pernas saltavam: 3500
    // m/s2).
    if (s.landingPrep && !input.grounded && m_airWeight > 0.5f) {
        const Vec3 fallVelocity = input.velocityWorld;
        const float horizontalSpeed = std::hypot(fallVelocity.x, fallVelocity.y);
        float groundZ = feet.z;
        bool groundKnown = false;
        if (input.groundAt) {
            const GroundProbeResult3D probe = input.groundAt(feet);
            if (probe.hasSurface) { groundZ = probe.pointWorld.z; groundKnown = true; }
        }
        // O pouso previsto (balistico): quando e com que velocidade vertical.
        const float rise = fallVelocity.z;
        const float landingFall = std::sqrt(rise * rise + 2.0f * Gravity * std::max(0.0f, feet.z - groundZ));
        const float touchdown = groundKnown ? (rise + landingFall) / Gravity : 1e9f;
        const float progress = groundKnown ? m_airTime / std::max(0.05f, m_airTime + touchdown) : 0.0f;
        const float wantedPrep = smoothStep((progress - s.landingPrepStart)
                / std::max(0.05f, s.landingPrepFull - s.landingPrepStart))
            * smoothStep((horizontalSpeed - 0.3f) / 0.7f);
        spring(m_landingPrep, m_landingPrepRate, wantedPrep, s.landingPrepSeconds, dt);
        if (horizontalSpeed > 0.3f && groundKnown) {
            m_landingDirection = { fallVelocity.x / horizontalSpeed, fallVelocity.y / horizontalSpeed, 0.0f };
            // (Pela velocidade do pouso, ja na subida: o rumo da perna nao
            // muda no meio do voo.)
            const float alpha = std::atan2(horizontalSpeed, std::max(0.05f, landingFall));
            m_landingRunningOn = smoothStep((horizontalSpeed - 2.5f) / 1.5f)
                * smoothStep(planar(input.requestedVelocityWorld).length() / 1.5f);
            m_landingBeta = std::min(s.landingAlign * alpha,
                s.landingMaxAngle + (s.landingRunMaxAngle - s.landingMaxAngle) * m_landingRunningOn);
        }
    } else {
        // (Numa mola: a saida comeca sem velocidade - exponencial, o peito
        // saltava de 0,4 para 3,3 graus por tick no toque.)
        // (Aterrissando, a descida da pelve do preparo fica com a
        // aterrissagem - m_landPrepCarry.)
        spring(m_landingPrep, m_landingPrepRate, 0.0f, 0.06f, dt);
        if (m_landingPrep < 0.01f) m_landingLeadSide = -1;
    }
    m_landingPrep = std::clamp(m_landingPrep, 0.0f, 1.0f);
    {
        const float prep = s.landingPrep ? m_landingPrep : 0.0f;
        if (prep > 0.001f) {
            const Vec3 direction = m_landingDirection;
            const float beta = m_landingBeta;
            const float runningOn = m_landingRunningOn;
            const Quaternion pelvisInverse = rootOrientation.conjugate();
            // A coluna na linha da perna: o tronco inclina para tras uma parte
            // do angulo dela (pela inclinacao da pelve: a coluna e relativa a
            // ela); de lado, menos.
            {
                const Vec3 pelvisUp = rootOrientation.rotate({ 0.0f, 0.0f, 1.0f });
                const float pelvisAngle = std::atan2(dot(pelvisUp, direction), pelvisUp.z);
                const Vec3 pelvisForward = rootOrientation.rotate({ 1.0f, 0.0f, 0.0f });
                const Vec3 flatForward = planar(pelvisForward).lengthSquared() > 1e-6f
                    ? planar(pelvisForward).normalized() : direction;
                const float forwardness = std::abs(dot(flatForward, direction));
                const float wanted = -s.landingTrunkAlign * beta
                    * (s.landingTrunkSide + (1.0f - s.landingTrunkSide) * forwardness);
                const float correction = std::clamp(wanted - pelvisAngle, -0.7f, 0.4f);
                const Vec3 axisLocal = pelvisInverse.rotate({ -direction.y, direction.x, 0.0f });
                const float pitch = correction * axisLocal.y;
                const float roll = -correction * axisLocal.x;
                const char* spine[3] = { "Abdomen", "Chest", "UpperChest" };
                const float shares[3] = { 0.30f, 0.35f, 0.35f };
                for (std::size_t i = 0; i < 3; ++i) {
                    blendCoordinate(profile, m_coordinates, spine[i], 1, pitch * shares[i], prep);
                    blendCoordinate(profile, m_coordinates, spine[i], 2, roll * shares[i], prep);
                }
            }
            // A perna (seguindo correndo, so a mais baixa) na linha da
            // velocidade, o joelho dobrado.
            RagdollAnimationPose3D air;
            buildRagdollLocalPose3D(profile, m_coordinates, air);
            // Seguindo correndo: a perna da frente da tesoura do pulo (a mais a
            // frente no rumo, escolhida no comeco do preparo) vai ao chao; a de
            // tras fica atras, dobrada e mais alta, pronta para o passo
            // seguinte. (Pela mais baixa, era a de tras do clipe - estendida
            // atras - que ia a frente, e a da frente ficava embaixo do corpo.)
            if (m_landingLeadSide < 0 && m_feet[0] != 0 && m_feet[1] != 0) {
                const float ahead[2] = { dot(rootOrientation.rotate(air.linkPositions[m_feet[0]]), direction),
                    dot(rootOrientation.rotate(air.linkPositions[m_feet[1]]), direction) };
                m_landingLeadSide = ahead[0] >= ahead[1] ? 0 : 1;
            }
            const int leadSide = runningOn > 0.5f ? m_landingLeadSide : -1;
            const float rearAngle = 0.60f;   // a de tras: 34 graus atras da vertical
            const Vec3 rearWorld = direction * -std::sin(rearAngle) + Vec3 { 0.0f, 0.0f, -std::cos(rearAngle) };
            const Vec3 legWorld = direction * std::sin(beta) + Vec3 { 0.0f, 0.0f, -std::cos(beta) };
            const RagdollLinkDefinition3D& pelvisLink = profile.links.front();
            for (std::size_t side = 0; side < 2; ++side) {
                const std::size_t index = m_feet[side];
                if (index == 0) continue;
                const bool rear = leadSide >= 0 && static_cast<int>(side) != leadSide;
                const RagdollLinkDefinition3D& footLink = profile.links[index];
                const int shin = footLink.parentIndex;
                const int thigh = shin > 0 ? profile.links[static_cast<std::size_t>(shin)].parentIndex : -1;
                if (thigh <= 0) continue;
                const Vec3 hipModel = profile.links[static_cast<std::size_t>(thigh)].inboundJoint.anchorModelPosition;
                const Vec3 hipLocal = pelvisLink.modelOrientation.conjugate().rotate(hipModel - pelvisLink.modelPosition);
                const float fullLength = (hipModel - footLink.inboundJoint.anchorModelPosition).length();
                // (A de tras dobrada: o pe a 55% da perna - com 70%, ela ficava na
                // altura da da frente e podia tocar junto.)
                const Vec3 wantedLocal = rear ? hipLocal + pelvisInverse.rotate(rearWorld * (0.55f * fullLength * runningOn
                        + s.landingLegShare * fullLength * (1.0f - runningOn)))
                    : hipLocal + pelvisInverse.rotate(legWorld * (s.landingLegShare * fullLength));
                const Vec3 currentLocal = air.linkPositions[index];
                solveFoot(profile, m_coordinates, index, currentLocal + (wantedLocal - currentLocal) * prep,
                    air.linkOrientations[index], 1.0f);
            }
        }
    }

    // Adaptacao dos pes (no chao): a passada no comprimento da velocidade
    // (ao longo do movimento, em volta do quadril), o pe de dentro girando
    // para a curva, e cada pe na altura do chao embaixo dele (nunca dentro;
    // apoiado, encostado). O pe da bola so nao entra no chao.
    RagdollAnimationPose3D local;
    buildRagdollLocalPose3D(profile, m_coordinates, local);
    float lowestGround = feet.z;
    if (input.grounded) {
        // (O comprimento ao longo do rumo de cada ciclo, com o peso da forma
        // dele; some perto do parado.)
        const float strideAmount = (strideScale - 1.0f) * m_moving * smoothStep((speed - 0.3f) / 1.0f);
        bool moved = false;
        for (std::size_t side = 0; side < 2; ++side) {
            const std::size_t index = m_feet[side];
            if (index == 0) continue;
            const RagdollLinkDefinition3D& link = profile.links[index];
            Vec3 position = rootPosition + rootOrientation.rotate(local.linkPositions[index]);
            Quaternion orientation = (rootOrientation * local.linkOrientations[index]).normalized();
            const float footContact = std::clamp(contact[side], 0.0f, 1.0f);
            const bool ballFoot = static_cast<int>(side) == ballSide;
            Vec3 shift {};
            if (!ballFoot && std::abs(strideAmount) > 1e-4f) {
                for (std::size_t g = 0; g < GaitCount; ++g) {
                    const float share = m_shape[g] / shapeTotal;
                    if (share <= 1e-3f) continue;
                    const float axisYaw = m_yaw + m_gaits[g].heading;
                    const Vec3 axis { std::cos(axisYaw), std::sin(axisYaw), 0.0f };
                    const float along = dot(planar(position - rootPosition), axis);
                    shift += axis * (along * strideAmount * share);
                }
            }
            // Na curva o pe pousa girado para dentro dela, o de dentro mais,
            // pela curva que o corpo faz de fato (como no fisico): gira no
            // balanco e fica assim no apoio; freando, parado ou com a bola, o
            // giro sai. A perna inteira gira pela coxa (abaixo).
            {
                const bool inner = (curveAcceleration > 0.0f) == (side == 0);
                const float need = smoothStep((std::abs(curveAcceleration) - s.footCurveYawStart)
                    / std::max(0.1f, s.footCurveYawSpan));
                const float lead = std::copysign(s.footCurveYawMaximum * need
                    * (inner ? 1.0f : s.footCurveYawOuterShare), curveAcceleration)
                    * smoothStep((speed - 1.5f) / 1.5f);
                const bool curving = m_moving > 0.5f && requested.length() > 0.1f && dot(velocity, requested) > 0.0f;
                const float blend = 1.0f - std::exp(-s.footCurveYawRate * dt);
                if (!curving || ballFoot)
                    m_footYaw[side] *= 1.0f - blend;
                else if (footContact < 0.5f)
                    m_footYaw[side] += (lead - m_footYaw[side]) * blend;
                if (std::abs(m_footYaw[side]) > 1e-4f)
                    orientation = (Quaternion::fromAxisAngle({ 0.0f, 0.0f, 1.0f }, m_footYaw[side]) * orientation)
                        .normalized();
            }
            // O chao embaixo do pe.
            Vec3 shifted = position + shift;
            // O sobe-e-desce da vida do tronco nao leva o pe apoiado: ele fica
            // onde o clipe o pos e o joelho absorve (pelo peso do apoio, sem
            // salto; so pela trava do chao o joelho dobrava num tranco).
            spring(m_plantWeight[side], m_plantRate[side], footContact >= 0.5f ? 1.0f : 0.0f, 0.03f, dt);
            shifted.z -= (m_trunkHeight - m_landingSink) * std::clamp(m_plantWeight[side], 0.0f, 1.0f);
            // A trava do pe apoiado (ver footLock).
            if (s.adaptiveTransitions && s.footLock) {
                FootLock& lock = m_footLocks[side];
                const bool planted = footContact > 0.5f && !ballFoot && ground > 0.5f;
                if (!planted) lock.spent = false;
                if (planted && !lock.active && !lock.spent) {
                    // (Trava onde o pe aparece agora - no meio de uma soltura,
                    // sem salto.)
                    const float w = smoothStep(lock.weight);
                    lock.position = { shifted.x + (lock.position.x - shifted.x) * w,
                        shifted.y + (lock.position.y - shifted.y) * w, shifted.z };
                    lock.active = true;
                    lock.weight = 1.0f;
                }
                if (lock.active && (!planted
                        || std::hypot(lock.position.x - shifted.x, lock.position.y - shifted.y)
                            > s.footLockReleaseMeters)) {
                    lock.active = false;
                    lock.spent = planted;
                }
                if (!lock.active) lock.weight = std::max(0.0f, lock.weight - dt / std::max(0.01f, s.footLockBlendSeconds));
                if (lock.weight > 0.0f) {
                    const float w = smoothStep(lock.weight);
                    shifted.x += (lock.position.x - shifted.x) * w;
                    shifted.y += (lock.position.y - shifted.y) * w;
                }
            } else {
                m_footLocks[side] = {};
            }
            float groundZ = feet.z;
            if (input.groundAt) {
                const GroundProbeResult3D probe = input.groundAt(shifted);
                if (probe.hasSurface && std::abs(probe.pointWorld.z - feet.z) < 0.6f) groundZ = probe.pointWorld.z;
            }
            if (footContact > 0.5f) lowestGround = std::min(lowestGround, groundZ);
            // (Nunca dentro do chao; descendo um degrau quem desce e a pelve,
            // abaixo.)
            // (Com os dedos dobrando no chao, a sola so ate a base deles.)
            const float sole = soleHeight(link, shifted, orientation, s.toeRoll ? s.toeRollMeters : 0.0f);
            float lift = std::max(0.0f, groundZ - sole);
            if (m_landPull > 0.0f && groundZ < sole) {
                // Aterrissando: o pe no ar vai ao chao.
                const float pull = m_landPull * (m_landLeadSide < 0 || static_cast<int>(side) == m_landLeadSide
                    ? 1.0f : 1.0f - m_landRunning);
                lift = (groundZ - sole) * pull;
            }
            const Vec3 target = shifted + Vec3 { 0.0f, 0.0f, lift };
            if ((target - position).lengthSquared() < 1e-8f && std::abs(m_footYaw[side]) < 1e-4f) continue;
            // (Com giro de curva o rumo do pe vem da coxa; o tornozelo sozinho
            // nao gira o pe em volta da vertical e cortava quase todo o giro.)
            if (std::abs(m_footYaw[side]) > 0.03f)
                characterSolveFootPose3D(profile, m_coordinates, index, rootInverse.rotate(target - rootPosition),
                    (rootInverse * orientation).normalized(), 1.0f, true);
            else
                solveFoot(profile, m_coordinates, index, rootInverse.rotate(target - rootPosition),
                    (rootInverse * orientation).normalized(), 1.0f);
            moved = true;
        }
        if (moved) buildRagdollLocalPose3D(profile, m_coordinates, local);
    } else {
        // (No ar nada fica travado: no pouso, cada pe trava onde pousar.)
        m_footLocks = {};
    }
    // A pelve desce o que um pe apoiado num chao mais baixo nao alcanca (o
    // proximo tick ja vem com ela).
    {
        const float wantedDrop = std::clamp(feet.z - lowestGround, 0.0f, s.terrainDropMaximum);
        m_terrainDrop += (wantedDrop - m_terrainDrop) * (1.0f - std::exp(-dt / 0.06f));
    }

    // Saida da arrancada agachada: pela inercia (nao no primeiro passe do tick
    // da semente, que e refeito).
    if (m_exitInertia.active() && !seedTick)
        m_exitInertia.apply(m_coordinates, rootPosition, rootOrientation, m_exitInertiaDt > 0.0f ? m_exitInertiaDt : dt);
    m_exitInertiaDt = 0.0f;
    // Saida do carrinho: da pose do fim dele ate a da locomocao.
    if (m_tackleExit < 1.0f) {
        m_tackleExit = std::min(1.0f, m_tackleExit + dt / std::max(0.01f, s.tackleExitSeconds));
        const float share = smoothStep(m_tackleExit);
        if (m_exitFrom.size() == links)
            for (std::size_t i = 1; i < links; ++i)
                m_coordinates[i] = m_exitFrom[i] + (m_coordinates[i] - m_exitFrom[i]) * share;
        const Vec3 from = feet + m_exitRootOffset;
        rootPosition = from + (rootPosition - from) * share;
        rootOrientation = nlerp(m_exitRootOrientation, rootOrientation, share);
    }

    // A voadora no ar: a pose do carrinho no comeco da fase deitada, no rumo
    // do voo, em ~0,12 s (a pelve no maximo 15 cm abaixo da do voo: as pernas
    // nao passam da base da capsula).
    m_output.tackling = false;
    m_output.airTackle = m_airTackleClip != nullptr;
    if (m_airTackleClip != nullptr) {
        const AnimationClip3D& clip = *m_airTackleClip;
        m_airTackleWeight = std::min(1.0f, m_airTackleWeight + dt / std::max(0.02f, m_airTackleBlendSeconds));
        const float w = smoothStep(m_airTackleWeight);
        // O tempo do clipe: do mergulho ate a fase deitada, saindo e chegando
        // devagar (parando de uma vez, a pelve girava a 30 mil graus/s2).
        m_airTackleElapsed += dt;
        for (std::size_t k = 0; k < m_tackles.size(); ++k)
            if (m_tackles[k] == &clip) {
                const float begin = m_tackleShape[k].diveStart;
                const float span = std::max(0.0f, m_tackleShape[k].lowStart - 0.02f - begin);
                m_airTackleTime = begin + span * smoothStep(m_airTackleElapsed / std::max(0.05f, m_airTackleDiveSeconds));
            }
        for (std::size_t i = 1; i < links; ++i)
            if (const AnimationTrack3D* track = findAnimationTrack3D(clip, profile.links[i].id))
                m_coordinates[i] += (sampleTrack(*track, m_airTackleTime, clip.durationSeconds, false, s.smoothClipSampling)
                    .jointPositionRadians - m_coordinates[i]) * w;
        const AnimationTransformSample3D r = rootAt(profile, clip, m_airTackleTime, false, s.smoothClipSampling);
        const Quaternion airOrientation = (yawRotation(m_airTackleYaw) * rootLink.modelOrientation * r.rotationDelta.normalized())
            .normalized();
        rootOrientation = nlerp(rootOrientation, airOrientation, w);
        // Deitando para tras no voo (a perna esticada sobe para a linha da
        // pelve, como uma voadora), em volta da pelve, pelo andar do mergulho.
        {
            float progress = 1.0f;
            for (std::size_t k = 0; k < m_tackles.size(); ++k)
                if (m_tackles[k] == &clip) {
                    const float span = m_tackleShape[k].lowStart - 0.02f - m_tackleShape[k].diveStart;
                    progress = span > 1e-3f ? std::clamp((m_airTackleTime - m_tackleShape[k].diveStart) / span, 0.0f, 1.0f) : 1.0f;
                }
            const Vec3 left { -std::sin(m_airTackleYaw), std::cos(m_airTackleYaw), 0.0f };
            rootOrientation = (Quaternion::fromAxisAngle(left, -radians(s.airTacklePitchDegrees) * w * smoothStep(progress))
                * rootOrientation).normalized();
        }
        rootPosition.z += std::clamp(r.translationOffsetMeters.z, -0.15f, 0.0f) * w;
        // A perna de apoio do clipe e esticada (a pelve do voo e mais baixa
        // que a de pe): o pe nao passa da base da capsula (com o peso: logo
        // depois de sair do chao o pe do pulo ainda esta abaixo dela).
        defaultLocomotionPose3D(profile, rootPosition, rootOrientation, m_coordinates, m_output.pose);
        rootPosition.z += feetBelowGround(profile, m_output.pose, m_feet, feet.z) * w;
        m_output.tackling = true;
        // A ancora do caminho do clipe (a raiz dele no tempo da pose sob a
        // pelve), para o fisico continuar dali.
        const Vec3 along = yawRotation(m_airTackleYaw).rotate({ r.translationOffsetMeters.x * m_tackleScale,
            r.translationOffsetMeters.y * m_tackleScale, 0.0f });
        m_tackleStart = { rootPosition.x - along.x, rootPosition.y - along.y, 0.0f };
        m_airPreviousCoordinates = m_output.coordinates;
        m_airPreviousRoot = m_output.rootPosition;
        m_airPreviousOrientation = m_output.rootOrientation;
    }
    defaultLocomotionPose3D(profile, rootPosition, rootOrientation, m_coordinates, m_output.pose);
    m_output.rootPosition = rootPosition;
    m_output.rootOrientation = rootOrientation;
    m_output.coordinates = m_coordinates;
    // Leitura: os pes pela animacao (o manejo de bola escolhe o pe por ela).
    m_output.valid = true;
    if (seedTick) {
        // Semente: o rumo pela pelve que saiu (o clipe tambem gira a pelve -
        // parado, ~22 graus) e a pose de novo com ele, num passe de 0,1 ms -
        // o rumo ja entra em muita coisa antes de a rotacao do clipe ser
        // conhecida, e a inercia mede a diferenca contra esta pose.
        const Vec3 out = (rootOrientation * rootLink.modelOrientation.conjugate()).rotate({ 1.0f, 0.0f, 0.0f });
        m_yaw = wrapAngle(m_yaw + wrapAngle(m_seed.pelvisYawRadians - std::atan2(out.y, out.x)));
        m_yawRate = 0.0f;
        DefaultLocomotionInput3D again = input;
        again.deltaTime = 1e-4f;
        m_exitInertiaDt = dt;
        again.tackleRequest = false;
        update(profile, animations, again);
        return;
    }
    m_output.facingYawRadians = m_yaw;
    m_output.cyclePhase = m_phase;
    m_output.movingShare = total;
    m_output.landingIntensity = m_landTime >= 0.0f ? m_landIntensity : 0.0f;
    m_output.landingSink = m_landingSink;
    m_output.airborne = !input.grounded;
    m_output.lean = m_lean;
    for (std::size_t side = 0; side < 2; ++side) {
        m_output.footContact[side] = std::clamp(contact[side], 0.0f, 1.0f);
        const bool swinging = input.grounded && total > 0.3f && m_output.footContact[side] < 0.5f;
        m_output.footPlanted[side] = !swinging && input.grounded;
        const Gait& gait = m_gaits[dominant];
        const float span = std::max(0.05f, fraction(gait.land[side] - gait.liftOff[side]));
        const bool cycling = input.grounded && total > 0.3f && rate > 1e-3f;
        const bool inSwing = fraction(m_phase - gait.liftOff[side]) < span;
        m_output.cycling = cycling;
        m_output.cyclePlanted[side] = !cycling || !inSwing;
        m_output.swingSeconds[side] = cycling ? span / rate : -1.0f;
        m_output.secondsToLiftoff[side] = cycling && !inSwing ? fraction(gait.liftOff[side] - m_phase) / rate : -1.0f;
        m_output.cycleSecondsToTouchdown[side] = cycling && inSwing ? fraction(gait.land[side] - m_phase) / rate : -1.0f;
        if (!swinging || rate <= 1e-3f) {
            m_output.swingProgress[side] = -1.0f;
            m_output.secondsToTouchdown[side] = -1.0f;
            continue;
        }
        m_output.swingProgress[side] = std::clamp(fraction(m_phase - gait.liftOff[side]) / span, 0.0f, 1.0f);
        m_output.secondsToTouchdown[side] = fraction(gait.land[side] - m_phase) / rate;
    }
}

} // namespace MatterEngine
