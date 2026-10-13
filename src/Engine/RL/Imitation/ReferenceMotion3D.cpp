#include "Engine/RL/Imitation/ReferenceMotion3D.hpp"

#include "Engine/Animation/AnimationClip3D.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iterator>
#include <limits>
#include <numbers>
#include <stdexcept>

namespace MatterEngine {
namespace {

std::uint64_t fnv1a(std::string_view text, std::uint64_t hash) {
    for (const char c : text) {
        hash ^= static_cast<unsigned char>(c);
        hash *= 1099511628211ull;
    }
    return hash;
}

std::string readText(const std::filesystem::path& path) {
    std::ifstream input(path, std::ios::binary);
    if (!input) throw std::runtime_error("referencia: clipe ausente: " + path.string());
    return { std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>() };
}

Quaternion yawRotation(float yaw) { return Quaternion::fromAxisAngle({ 0.0f, 0.0f, 1.0f }, yaw); }

float wrapAngle(float angle) {
    constexpr float Pi = std::numbers::pi_v<float>;
    while (angle > Pi) angle -= 2.0f * Pi;
    while (angle < -Pi) angle += 2.0f * Pi;
    return angle;
}

// O rumo da pelve (o eixo da frente dela, como o ambiente le).
float pelvisHeading(const RagdollProfile3D& profile, Quaternion pelvisOrientation) {
    const Quaternion frame = (pelvisOrientation * profile.links.front().modelOrientation.conjugate()).normalized();
    const Vec3 forward = frame.rotate({ 1.0f, 0.0f, 0.0f });
    return std::atan2(forward.y, forward.x);
}

// O ponto mais baixo de um pe numa pose (z do mundo): os cantos da caixa ou os
// vertices da casca da chuteira; a capsula pelas pontas.
float footLowestPoint(const RLBody3D& body, const RLBodyPose3D& pose, std::uint32_t foot) {
    const RagdollCapsuleDefinition3D& collider = body.profile.links[foot].collider;
    const Vec3 center = pose.positions[foot] + pose.orientations[foot].rotate(collider.localPosition);
    const Quaternion orientation = pose.orientations[foot] * collider.localOrientation;
    float lowest = std::numeric_limits<float>::max();
    if (ragdollColliderHasBox3D(collider)) {
        for (const Vec3& corner : ragdollColliderCorners3D(collider))
            lowest = std::min(lowest, (center + orientation.rotate(corner)).z);
        return lowest;
    }
    const float half = ragdollCapsuleHalfSegment3D(collider);
    for (const float side : { -1.0f, 1.0f })
        lowest = std::min(lowest, (center + orientation.rotate({ side * half, 0.0f, 0.0f })).z - collider.radiusMeters);
    return lowest;
}

// O que se le do clipe num instante, antes das derivadas.
struct RawFrame {
    std::vector<Vec3> coordinates;
    Quaternion rootRotation;
    Vec3 rootPosition;  // no mundo do clipe (o avanco devolvido ao in-place)
};

ReferenceSegment3D sampleSegment(const RLBody3D& body, const ReferenceClip3D& request, float frameSeconds,
    std::uint64_t& hash) {
    const RagdollProfile3D& profile = body.profile;
    const AnimationClip3D clip = loadAnimationClip3D(request.clipPath.string());
    hash = fnv1a(readText(request.clipPath), hash);
    if (!(clip.durationSeconds > 0.0f)) throw std::runtime_error("referencia: clipe sem duracao: " + clip.id);
    const AnimationTrack3D* rootTrack = findAnimationTrack3D(clip, profile.links.front().id);
    if (!rootTrack) throw std::runtime_error("referencia: o clipe " + clip.id + " nao tem a pelve");

    ReferenceSegment3D segment;
    segment.id = request.id.empty() ? clip.id : request.id;
    segment.clipPath = request.clipPath;
    segment.loops = request.loops;
    // Em ciclo, o ultimo quadro do clipe e o primeiro: fica de fora.
    const float duration = clip.durationSeconds;
    const std::size_t count = std::max<std::size_t>(2, static_cast<std::size_t>(std::lround(duration / frameSeconds))
        + (request.loops ? 0 : 1));

    // O avanco que o in-place tirou (a deriva linear entre as pontas), so no
    // plano. O vertical da deriva e vazamento do avanco pela inclinacao do chao
    // da captura (~5 graus no Mixamo: 0,24 m por ciclo no sprint, 0,16 m na
    // partida): z_captura = z + k x, com k = deriva z / deriva x. A deriva
    // linear tira o k x so de um avanco constante; o resto (k vezes o x que
    // sobrou no in-place) sai aqui.
    const Vec3 drift = clip.sourceRootDisplacementMeters;
    const Vec3 advance { drift.x, drift.y, 0.0f };
    const float leak = std::abs(drift.x) > 0.1f ? drift.z / drift.x : 0.0f;

    std::vector<RawFrame> raw(count);
    for (std::size_t i = 0; i < count; ++i) {
        const float t = std::min(duration, static_cast<float>(i) * frameSeconds);
        RawFrame& frame = raw[i];
        frame.coordinates.assign(profile.links.size(), {});
        const AnimationTransformSample3D root = sampleAnimationTrackSmooth3D(*rootTrack, t, duration, request.loops);
        frame.rootRotation = root.rotationDelta.normalized();
        Vec3 offset = root.translationOffsetMeters;
        offset.z -= leak * offset.x;
        frame.rootPosition = offset + advance * (t / duration);
        for (std::size_t link = 1; link < profile.links.size(); ++link) {
            const AnimationTrack3D* track = findAnimationTrack3D(clip, profile.links[link].id);
            if (!track) continue;
            Vec3 q = sampleAnimationTrackSmooth3D(*track, t, duration, request.loops).jointPositionRadians;
            // Fora do curso a fisica nao vai: a referencia tambem nao.
            const RagdollJointDefinition3D& joint = profile.links[link].inboundJoint;
            float* axes[] = { &q.x, &q.y, &q.z };
            for (std::size_t a = 0; a < 3; ++a)
                *axes[a] = joint.axes[a].enabled
                    ? std::clamp(*axes[a], joint.axes[a].minimumRadians, joint.axes[a].maximumRadians) : 0.0f;
            frame.coordinates[link] = q;
        }
    }

    // Os vizinhos de um quadro para as derivadas (em ciclo da a volta).
    const auto previous = [&](std::size_t i) { return i > 0 ? i - 1 : (request.loops ? count - 1 : 0); };
    const auto next = [&](std::size_t i) { return i + 1 < count ? i + 1 : (request.loops ? 0 : count - 1); };
    // O avanco de um ciclo inteiro entra na diferenca que da a volta.
    const auto positionAt = [&](std::size_t i, std::ptrdiff_t wrap) {
        return raw[i].rootPosition + advance * static_cast<float>(wrap);
    };

    std::vector<RLBodyPose3D> poses(count);
    std::vector<std::array<float, 2>> contact(count);
    std::vector<float> ground(count, 0.0f), lowestFoot(count, 0.0f);
    std::vector<std::uint8_t> known(count, 0);
    float lowestAll = std::numeric_limits<float>::max();
    segment.frames.resize(count);
    for (std::size_t i = 0; i < count; ++i) {
        const float t = std::min(duration, static_cast<float>(i) * frameSeconds);
        const Quaternion root = (profile.links.front().modelOrientation * raw[i].rootRotation).normalized();
        poses[i] = rlBodyPose3D(profile, { 0.0f, 0.0f, raw[i].rootPosition.z }, root, raw[i].coordinates);
        contact[i] = { sampleAnimationScalarTrack3D(clip.leftFootContact, t, duration, request.loops),
            sampleAnimationScalarTrack3D(clip.rightFootContact, t, duration, request.loops) };
        const std::array<float, 2> lowest { footLowestPoint(body, poses[i], body.leftFoot),
            footLowestPoint(body, poses[i], body.rightFoot) };
        lowestFoot[i] = std::min(lowest[0], lowest[1]);
        lowestAll = std::min(lowestAll, lowestFoot[i]);
        float support = std::numeric_limits<float>::max();
        for (std::size_t f = 0; f < 2; ++f)
            if (contact[i][f] >= 0.5f) support = std::min(support, lowest[f]);
        if (support < std::numeric_limits<float>::max()) {
            ground[i] = support;
            known[i] = 1;
        }
    }
    // O chao de cada quadro: o pe de apoio (pelo clipe) fica nele; no voo, ele
    // vem dos apoios vizinhos (a reta entre eles) e os pes passam acima. Sem
    // apoio marcado no clipe, o ponto mais baixo do trecho todo.
    const bool anyKnown = std::find(known.begin(), known.end(), 1) != known.end();
    if (!anyKnown) std::fill(ground.begin(), ground.end(), lowestAll);
    else
        for (std::size_t i = 0; i < count; ++i) {
            if (known[i]) continue;
            std::ptrdiff_t back = -1, ahead = -1;
            for (std::size_t k = 1; k < count && back < 0; ++k) {
                if (!request.loops && k > i) break;
                const std::size_t j = (i + count - k) % count;
                if (known[j]) back = static_cast<std::ptrdiff_t>(k);
            }
            for (std::size_t k = 1; k < count && ahead < 0; ++k) {
                if (!request.loops && i + k >= count) break;
                const std::size_t j = (i + k) % count;
                if (known[j]) ahead = static_cast<std::ptrdiff_t>(k);
            }
            const float before = back > 0 ? ground[(i + count - static_cast<std::size_t>(back)) % count] : 0.0f;
            const float after = ahead > 0 ? ground[(i + static_cast<std::size_t>(ahead)) % count] : 0.0f;
            if (back > 0 && ahead > 0)
                ground[i] = before + (after - before) * static_cast<float>(back) / static_cast<float>(back + ahead);
            else ground[i] = back > 0 ? before : after;
        }
    // Nenhum pe abaixo do chao (o que sai do apoio pelo clipe ainda empurra).
    for (std::size_t i = 0; i < count; ++i) ground[i] = std::min(ground[i], lowestFoot[i]);
    // Uma media curta (5 quadros, ~40 ms) tira o degrau da troca de apoio.
    for (int pass = 0; pass < 2; ++pass) {
        std::vector<float> smooth(count);
        for (std::size_t i = 0; i < count; ++i) {
            float sum = 0.0f, weight = 0.0f;
            for (std::ptrdiff_t k = -2; k <= 2; ++k) {
                std::ptrdiff_t j = static_cast<std::ptrdiff_t>(i) + k;
                if (request.loops) j = (j + static_cast<std::ptrdiff_t>(count)) % static_cast<std::ptrdiff_t>(count);
                else if (j < 0 || j >= static_cast<std::ptrdiff_t>(count)) continue;
                sum += ground[static_cast<std::size_t>(j)];
                weight += 1.0f;
            }
            smooth[i] = sum / weight;
        }
        ground = std::move(smooth);
    }
    for (std::size_t i = 0; i < count; ++i) raw[i].rootPosition.z -= ground[i];

    for (std::size_t i = 0; i < count; ++i) {
        ReferenceFrame3D& frame = segment.frames[i];
        const std::size_t a = previous(i), b = next(i);
        // Diferenca central (nas pontas de um trecho que nao e ciclo, de um lado).
        const float span = (request.loops ? 2.0f : static_cast<float>(b - a)) * frameSeconds;
        const float inverse = span > 0.0f ? 1.0f / span : 0.0f;
        const std::ptrdiff_t wrapA = a > i ? -1 : 0, wrapB = b < i ? 1 : 0;

        frame.jointCoordinates = raw[i].coordinates;
        frame.jointVelocities.assign(profile.links.size(), {});
        for (std::size_t link = 1; link < profile.links.size(); ++link)
            frame.jointVelocities[link] = (raw[b].coordinates[link] - raw[a].coordinates[link]) * inverse;
        for (const RLBodyDof3D& dof : body.dofs) {
            const auto component = [&](Vec3 v) {
                return dof.axis == RagdollAxis3D::Twist ? v.x : dof.axis == RagdollAxis3D::Swing1 ? v.y : v.z;
            };
            frame.dofRadians.push_back(component(frame.jointCoordinates[dof.linkIndex]));
            frame.dofVelocities.push_back(component(frame.jointVelocities[dof.linkIndex]));
        }
        frame.rootRotation = raw[i].rootRotation;
        frame.rootHeight = raw[i].rootPosition.z;
        const RLBodyPose3D& pose = poses[i];
        frame.heading = pelvisHeading(profile, pose.orientations.front());
        const Quaternion headingInverse = yawRotation(-frame.heading);
        const Vec3 velocity = (positionAt(b, wrapB) - positionAt(a, wrapA)) * inverse;
        frame.rootVelocity = headingInverse.rotate(velocity);
        const Quaternion qa = poses[a].orientations.front(), qb = poses[b].orientations.front();
        frame.rootSpin = headingInverse.rotate(rlQuaternionLog3D((qb * qa.conjugate()).normalized()) * inverse);
        const Quaternion pelvisFrame =
            (pose.orientations.front() * profile.links.front().modelOrientation.conjugate()).normalized();
        frame.pelvisUp = headingInverse.rotate(pelvisFrame.rotate({ 0.0f, 0.0f, 1.0f })).normalized();
        const std::array<std::uint32_t, ReferenceKeyBodyCount3D> keys { body.head, body.leftHand, body.rightHand,
            body.leftFoot, body.rightFoot };
        for (std::size_t k = 0; k < keys.size(); ++k)
            frame.keyBodies[k] = headingInverse.rotate(pose.positions[keys[k]] - pose.positions.front());
        frame.footContact = contact[i];
    }
    segment.travelHeading = std::hypot(advance.x, advance.y) > 1.0e-4f ? std::atan2(advance.y, advance.x) : 0.0f;
    segment.meanSpeed = std::hypot(advance.x, advance.y) / duration;
    return segment;
}

float poseDistance2(const ReferenceFrame3D& a, const ReferenceFrame3D& b) {
    float sum = 0.0f;
    for (std::size_t d = 0; d < a.dofRadians.size(); ++d) {
        const float e = a.dofRadians[d] - b.dofRadians[d];
        sum += e * e;
    }
    return sum;
}

} // namespace

ReferenceMotion3D buildReferenceMotion3D(const RLBody3D& body, std::span<const ReferenceClip3D> clips,
    float frameSeconds) {
    if (clips.empty()) throw std::invalid_argument("referencia: sem trechos");
    if (!(frameSeconds > 0.0f)) throw std::invalid_argument("referencia: passo invalido");
    for (std::size_t i = 0; i < clips.size(); ++i)
        if (clips[i].loops != (i + 1 == clips.size()))
            throw std::invalid_argument("referencia: so o ultimo trecho fica em ciclo (" + clips[i].clipPath.string() + ")");
    ReferenceMotion3D motion;
    motion.frameSeconds = frameSeconds;
    motion.hash = 14695981039346656037ull;
    for (const ReferenceClip3D& clip : clips) motion.segments.push_back(sampleSegment(body, clip, frameSeconds, motion.hash));
    // A emenda: cada trecho entra pelo quadro mais parecido com o fim do anterior.
    for (std::size_t s = 1; s < motion.segments.size(); ++s) {
        const ReferenceFrame3D& last = motion.segments[s - 1].frames.back();
        ReferenceSegment3D& segment = motion.segments[s];
        float best = std::numeric_limits<float>::max();
        for (std::size_t f = 0; f < segment.frames.size(); ++f) {
            const float d = poseDistance2(last, segment.frames[f]);
            if (d < best) {
                best = d;
                segment.entryFrame = static_cast<std::uint32_t>(f);
            }
        }
        const float dofs = static_cast<float>(std::max<std::size_t>(1, last.dofRadians.size()));
        segment.entryJumpRms = std::sqrt(best / dofs);
    }
    return motion;
}

ReferenceCursor3D advanceReference3D(const ReferenceMotion3D& motion, ReferenceCursor3D cursor, std::uint32_t frames) {
    if (motion.segments.empty()) return {};
    cursor.segment = std::min<std::uint32_t>(cursor.segment, static_cast<std::uint32_t>(motion.segments.size() - 1));
    std::uint32_t remaining = frames;
    while (remaining > 0) {
        const ReferenceSegment3D& segment = motion.segments[cursor.segment];
        const std::uint32_t size = static_cast<std::uint32_t>(segment.frames.size());
        if (segment.loops) {
            cursor.frame = (cursor.frame + remaining) % size;
            return cursor;
        }
        // O ultimo quadro de um trecho que nao e ciclo e o primeiro do proximo.
        const std::uint32_t left = size - 1 - std::min(cursor.frame, size - 1);
        if (remaining <= left) {
            cursor.frame += remaining;
            return cursor;
        }
        remaining -= left;
        ++cursor.segment;
        cursor.frame = motion.segments[cursor.segment].entryFrame;
        // A troca de trecho ja e o primeiro quadro andado.
        --remaining;
        if (motion.segments[cursor.segment].loops) {
            cursor.frame = (cursor.frame + remaining) % static_cast<std::uint32_t>(motion.segments[cursor.segment].frames.size());
            return cursor;
        }
    }
    return cursor;
}

const ReferenceFrame3D& referenceFrame3D(const ReferenceMotion3D& motion, ReferenceCursor3D cursor) {
    const ReferenceSegment3D& segment =
        motion.segments[std::min<std::size_t>(cursor.segment, motion.segments.size() - 1)];
    return segment.frames[std::min<std::size_t>(cursor.frame, segment.frames.size() - 1)];
}

float referenceProgress3D(const ReferenceMotion3D& motion, ReferenceCursor3D cursor) {
    const ReferenceSegment3D& segment =
        motion.segments[std::min<std::size_t>(cursor.segment, motion.segments.size() - 1)];
    const float size = static_cast<float>(segment.frames.size() - (segment.loops ? 0 : 1));
    return size > 0.0f ? static_cast<float>(cursor.frame) / size : 0.0f;
}

RLBodyPose3D referencePose3D(const RLBody3D& body, const ReferenceFrame3D& frame, Vec3 position, float heading) {
    const RagdollProfile3D& profile = body.profile;
    const Quaternion turn = yawRotation(wrapAngle(heading - frame.heading));
    const Quaternion root = (turn * profile.links.front().modelOrientation * frame.rootRotation).normalized();
    return rlBodyPose3D(profile, { position.x, position.y, position.z + frame.rootHeight }, root, frame.jointCoordinates);
}

void referenceLinkStates3D(const RLBody3D& body, const ReferenceFrame3D& frame, Vec3 position, float heading,
    std::vector<PhysicsBodyState3D>& out) {
    const RagdollProfile3D& profile = body.profile;
    const Quaternion turn = yawRotation(wrapAngle(heading - frame.heading));
    const Quaternion root = (turn * profile.links.front().modelOrientation * frame.rootRotation).normalized();
    const Quaternion world = yawRotation(heading);
    rlBodyLinkStates3D(profile, { position.x, position.y, position.z + frame.rootHeight }, root,
        world.rotate(frame.rootVelocity), world.rotate(frame.rootSpin), frame.jointCoordinates, frame.jointVelocities,
        out);
}

} // namespace MatterEngine
