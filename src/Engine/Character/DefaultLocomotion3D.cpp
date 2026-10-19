#include "Engine/Character/DefaultLocomotion3D.hpp"

#include "Engine/Character/DefaultLocomotion3DInternal.hpp"

#include <cstring>
#include <memory>
#include <mutex>

namespace MatterEngine {


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
    m_turnClip = nullptr;
    m_turnWeight = 0.0f;
    m_turnTime = 0.0f;
    m_dribbleCadence = 1.0f;
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
    m_stepOffset = 0.0f;
    m_stepOffsetRate = 0.0f;
    m_previousBaseKnown = false;
    m_stepClear = {};
    m_stepClearRate = {};
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

struct DefaultLocomotion3D::PreparedTables {
    const RagdollProfile3D* profile = nullptr;
    std::size_t links = 0;
    CharacterLocomotionAnimations3D animations {};
    std::vector<float> fingerprint;
    std::array<Gait, GaitCount> gaits {};
    std::array<const AnimationClip3D*, 3> arcs {};
    std::array<float, 3> arcTouchdown {};
    std::array<const AnimationClip3D*, 2> tackles {};
    std::array<int, 2> tackleSide { -1, -1 };
    std::array<float, 2> tackleClipSpeed {};
    std::array<TackleClipShape3D, 2> tackleShape {};
    std::array<std::size_t, 2> feet {};
    std::array<std::size_t, 2> hands {};
    std::vector<char> upperLinks;
    std::vector<float> amplitudeShare;
    BallGestureLegs3D gestureLegs;
};

namespace {

// O conteudo dos clipes entra na chave (um clipe recarregado no mesmo endereco
// nao reaproveita tabelas velhas).
std::vector<float> preparedFingerprint(const RagdollProfile3D& profile, const CharacterLocomotionAnimations3D& animations) {
    std::vector<float> f;
    f.push_back(static_cast<float>(profile.links.size()));
    const auto* clips = reinterpret_cast<const AnimationClip3D* const*>(&animations);
    const std::size_t count = sizeof(CharacterLocomotionAnimations3D) / sizeof(const AnimationClip3D*);
    for (std::size_t i = 0; i < count; ++i) {
        const AnimationClip3D* clip = clips[i];
        if (clip == nullptr) { f.push_back(-1.0f); continue; }
        f.push_back(clip->durationSeconds);
        f.push_back(static_cast<float>(clip->tracks.size()));
        for (const AnimationTrack3D& track : clip->tracks) {
            f.push_back(static_cast<float>(track.keyframes.size()));
            if (!track.keyframes.empty()) {
                f.push_back(track.keyframes.back().timeSeconds);
                f.push_back(track.keyframes.back().jointPositionRadians.x);
            }
        }
    }
    return f;
}

std::mutex& preparedMutex() {
    static std::mutex mutex;
    return mutex;
}

} // namespace

void DefaultLocomotion3D::prepare(const RagdollProfile3D& profile, const CharacterLocomotionAnimations3D& animations) {
    static_assert(sizeof(CharacterLocomotionAnimations3D) % sizeof(const AnimationClip3D*) == 0);
    static std::vector<std::shared_ptr<const PreparedTables>> cache;
    const std::vector<float> fingerprint = preparedFingerprint(profile, animations);
    std::shared_ptr<const PreparedTables> found;
    {
        std::lock_guard<std::mutex> lock(preparedMutex());
        for (const auto& entry : cache)
            if (entry->profile == &profile && entry->links == profile.links.size()
                && std::memcmp(&entry->animations, &animations, sizeof animations) == 0
                && entry->fingerprint == fingerprint) {
                found = entry;
                break;
            }
    }
    if (found == nullptr) {
        computePrepared(profile, animations);
        auto entry = std::make_shared<PreparedTables>();
        entry->profile = &profile;
        entry->links = profile.links.size();
        entry->animations = animations;
        entry->fingerprint = fingerprint;
        entry->gaits = m_gaits;
        entry->arcs = m_arcs;
        entry->arcTouchdown = m_arcTouchdown;
        entry->tackles = m_tackles;
        entry->tackleSide = m_tackleSide;
        entry->tackleClipSpeed = m_tackleClipSpeed;
        entry->tackleShape = m_tackleShape;
        entry->feet = m_feet;
        entry->hands = m_hands;
        entry->upperLinks = m_upperLinks;
        entry->amplitudeShare = m_amplitudeShare;
        entry->gestureLegs = m_gestureLegs;
        std::lock_guard<std::mutex> lock(preparedMutex());
        if (cache.size() >= 32) cache.erase(cache.begin());
        cache.push_back(std::move(entry));
    } else {
        m_gaits = found->gaits;
        m_arcs = found->arcs;
        m_arcTouchdown = found->arcTouchdown;
        m_tackles = found->tackles;
        m_tackleSide = found->tackleSide;
        m_tackleClipSpeed = found->tackleClipSpeed;
        m_tackleShape = found->tackleShape;
        m_feet = found->feet;
        m_hands = found->hands;
        m_upperLinks = found->upperLinks;
        m_amplitudeShare = found->amplitudeShare;
        m_gestureLegs = found->gestureLegs;
    }
    m_preparedFor = animations.walk;
    m_preparedAnimations = animations;
}

void DefaultLocomotion3D::computePrepared(const RagdollProfile3D& profile,
    const CharacterLocomotionAnimations3D& animations) {
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
        // Plano v5: os pes pelo ciclo em relacao a pelve, no rumo dela (a
        // previsao do pe no futuro - o manejo planeja o toque dentro do
        // balanco).
        const RagdollLinkDefinition3D& root = profile.links.front();
        for (Gait& gait : m_gaits) {
            if (gait.clip == nullptr || leftFoot == 0 || rightFoot == 0) continue;
            for (int k = 0; k < Gait::FeetSamples; ++k) {
                const float phase = static_cast<float>(k) / static_cast<float>(Gait::FeetSamples);
                const RagdollAnimationPose3D pose = sampleRagdollAnimationPose3D(profile, gait.clip,
                    fraction(phase + gait.touchdown) * gait.clip->durationSeconds, true);
                const Vec3 pelvisForward = (pose.linkOrientations.front() * root.modelOrientation.conjugate())
                    .rotate({ 1.0f, 0.0f, 0.0f });
                const Quaternion back = yawRotation(-std::atan2(pelvisForward.y, pelvisForward.x));
                const std::size_t feet[2] = { leftFoot, rightFoot };
                for (std::size_t side = 0; side < 2; ++side) {
                    gait.feet[side][static_cast<std::size_t>(k)] = back.rotate(pose.linkPositions[feet[side]] - pose.linkPositions.front());
                    gait.feetTurn[side][static_cast<std::size_t>(k)] = (back * pose.linkOrientations[feet[side]]).normalized();
                }
            }
            for (std::size_t side = 0; side < 2; ++side) {
                Vec3 sum {};
                for (const Vec3& foot : gait.feet[side]) sum += foot;
                gait.feetMean[side] = sum * (1.0f / static_cast<float>(Gait::FeetSamples));
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
}

float DefaultLocomotion3D::gaitAmplitudeOf(std::size_t g) const {
    const DefaultLocomotionSettings3D& s = m_settings;
    const bool sprint = g >= Directions;
    const std::size_t d = g % Directions;
    const float share = d == 2 || d == 6 ? (sprint ? s.sprintAmplitudeSide : s.trotAmplitudeSide) : 1.0f;
    return (1.0f + (m_gaitAmplitude[sprint ? 1 : 0] - 1.0f) * share) / std::max(1.0f, m_dribbleCadence);
}

void DefaultLocomotion3D::update(const RagdollProfile3D& profile,
    const CharacterLocomotionAnimations3D& animations, const DefaultLocomotionInput3D& input) {
    // A reescrita (R4): o update em blocos, um arquivo por bloco
    // (DefaultLocomotion3D_<bloco>.cpp), com a mesma conta de antes; o que um
    // bloco passa para os seguintes mora no Tick.
    Tick t { profile, animations, input };
    t.dt = std::max(1e-4f, input.deltaTime);
    t.links = profile.links.size();
    m_output.valid = false;
    m_output.tackleEnded = false;
    if (t.links == 0 || animations.walk == nullptr) return;
    if (m_preparedFor != animations.walk
        || std::memcmp(&m_preparedAnimations, &animations, sizeof animations) != 0)
        prepare(profile, animations);
    m_tackleCooldown = std::max(0.0f, m_tackleCooldown - t.dt);
    t.rootLink = &profile.links.front();
    t.viewYaw = input.facingYawRadians;
    t.viewForward = Vec3 { std::cos(t.viewYaw), std::sin(t.viewYaw), 0.0f };
    stepTravel(t);
    if (stepTackle(t)) return;
    stepGait(t);
    stepAir(t);
    stepPose(t);
    stepLayers(t);
    stepFeet(t);
    stepExits(t);
    stepReadout(t);
}

} // namespace MatterEngine
