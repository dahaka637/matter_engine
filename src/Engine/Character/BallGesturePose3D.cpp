#include "Engine/Character/BallGesturePose3D.hpp"

#include "Engine/Character/RagdollPoseKinematics3D.hpp"

#include <algorithm>
#include <cmath>
#include <string>

namespace MatterEngine {

namespace {

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

// Vetor de rotacao (log do quaternion).
Vec3 rotationVector(Quaternion q) {
    q = q.normalized();
    if (q.w < 0.0f) q = { -q.x, -q.y, -q.z, -q.w };
    const Vec3 v { q.x, q.y, q.z };
    const float n = v.length();
    if (n < 1e-7f) return {};
    return v * (2.0f * std::atan2(n, q.w) / n);
}

// Quaternion de uma base ortonormal (colunas x, y, z).
Quaternion fromBasis(Vec3 x, Vec3 y, Vec3 z) {
    const float m00 = x.x, m01 = y.x, m02 = z.x;
    const float m10 = x.y, m11 = y.y, m12 = z.y;
    const float m20 = x.z, m21 = y.z, m22 = z.z;
    const float trace = m00 + m11 + m22;
    Quaternion q;
    if (trace > 0.0f) {
        const float s = std::sqrt(trace + 1.0f) * 2.0f;
        q = { (m21 - m12) / s, (m02 - m20) / s, (m10 - m01) / s, 0.25f * s };
    } else if (m00 > m11 && m00 > m22) {
        const float s = std::sqrt(1.0f + m00 - m11 - m22) * 2.0f;
        q = { 0.25f * s, (m01 + m10) / s, (m02 + m20) / s, (m21 - m12) / s };
    } else if (m11 > m22) {
        const float s = std::sqrt(1.0f + m11 - m00 - m22) * 2.0f;
        q = { (m01 + m10) / s, 0.25f * s, (m12 + m21) / s, (m02 - m20) / s };
    } else {
        const float s = std::sqrt(1.0f + m22 - m00 - m11) * 2.0f;
        q = { (m02 + m20) / s, (m12 + m21) / s, 0.25f * s, (m10 - m01) / s };
    }
    return q.normalized();
}

// A rotacao que leva o par (a1, b1) ao par (a2, b2) (a perpendicular a b).
Quaternion alignPairs(Vec3 a1, Vec3 b1, Vec3 a2, Vec3 b2) {
    const Quaternion from = fromBasis(a1, b1, cross(a1, b1));
    const Quaternion to = fromBasis(a2, b2, cross(a2, b2));
    return (to * from.conjugate()).normalized();
}

Vec3 orthogonal(Vec3 v, Vec3 axis) {
    const Vec3 out = v - axis * dot(v, axis);
    return out.lengthSquared() > 1e-10f ? out.normalized() : Vec3 {};
}

// Coordenadas de junta da rotacao de mundo do filho dado o pai (dentro dos
// limites).
Vec3 jointCoordinates(const RagdollProfile3D& profile, std::size_t index, Quaternion parentWorld,
    Quaternion childWorld) {
    const RagdollLinkDefinition3D& link = profile.links[index];
    const RagdollLinkDefinition3D& parent = profile.links[static_cast<std::size_t>(link.parentIndex)];
    const Quaternion frame = link.inboundJoint.frameModelOrientation;
    Vec3 local = rotationVector(frame.conjugate() * parent.modelOrientation * parentWorld.conjugate() * childWorld
        * link.modelOrientation.conjugate() * frame);
    for (std::size_t axis = 0; axis < 3; ++axis) {
        const RagdollAxisDefinition3D& limit = link.inboundJoint.axes[axis];
        setComponent(local, axis, limit.enabled
            ? std::clamp(component(local, axis), limit.minimumRadians, limit.maximumRadians) : 0.0f);
    }
    return local;
}

Vec3 anchorIn(const RagdollProfile3D& profile, std::size_t link, Vec3 anchorModel) {
    const RagdollLinkDefinition3D& def = profile.links[link];
    return def.modelOrientation.conjugate().rotate(anchorModel - def.modelPosition);
}

// O pe nos dois eixos do tornozelo, o mais perto da orientacao pedida: o
// "para cima" do pe primeiro, a ponta em segundo (Gauss-Newton, como no
// solveFootPose do modo fisico).
Vec3 fitAnkle(const RagdollProfile3D& profile, std::size_t footIndex, Quaternion shinWorld, Quaternion footWorld,
    Vec3 start) {
    const RagdollLinkDefinition3D& foot = profile.links[footIndex];
    const RagdollLinkDefinition3D& shin = profile.links[static_cast<std::size_t>(foot.parentIndex)];
    const Quaternion shinFrame = shinWorld * shin.modelOrientation.conjugate() * foot.inboundJoint.frameModelOrientation;
    const Quaternion childFrame = (foot.modelOrientation.conjugate() * foot.inboundJoint.frameModelOrientation).conjugate();
    const Vec3 upLocal = foot.modelOrientation.conjugate().rotate({ 0.0f, 0.0f, 1.0f });
    const Vec3 forwardLocal = foot.modelOrientation.conjugate().rotate({ 1.0f, 0.0f, 0.0f });
    const Vec3 wantedUp = footWorld.rotate(upLocal);
    const Vec3 wantedForward = footWorld.rotate(forwardLocal);
    const auto residual = [&](Vec3 angles, float out[6]) {
        const Quaternion world = shinFrame * ragdollJointRotation3D(angles) * childFrame;
        const Vec3 up = world.rotate(upLocal) - wantedUp;
        const Vec3 forward = (world.rotate(forwardLocal) - wantedForward) * 0.6f;
        out[0] = up.x; out[1] = up.y; out[2] = up.z;
        out[3] = forward.x; out[4] = forward.y; out[5] = forward.z;
    };
    std::array<std::size_t, 3> axes {};
    std::size_t free = 0;
    for (std::size_t axis = 0; axis < 3; ++axis)
        if (foot.inboundJoint.axes[axis].enabled) axes[free++] = axis;
    Vec3 angles = start;
    for (int iteration = 0; iteration < 10 && free > 0 && free <= 2; ++iteration) {
        float r[6];
        residual(angles, r);
        float jacobian[6][2] {};
        for (std::size_t column = 0; column < free; ++column) {
            Vec3 probe = angles;
            setComponent(probe, axes[column], component(probe, axes[column]) + 0.0005f);
            float shifted[6];
            residual(probe, shifted);
            for (int row = 0; row < 6; ++row) jacobian[row][column] = (shifted[row] - r[row]) / 0.0005f;
        }
        float normal[2][3] {};
        for (std::size_t a = 0; a < free; ++a) {
            for (std::size_t b = 0; b < free; ++b)
                for (int row = 0; row < 6; ++row) normal[a][b] += jacobian[row][a] * jacobian[row][b];
            normal[a][a] += 0.001f;
            for (int row = 0; row < 6; ++row) normal[a][2] -= jacobian[row][a] * r[row];
        }
        float step[2] {};
        if (free == 1) {
            step[0] = normal[0][2] / normal[0][0];
        } else {
            const float determinant = normal[0][0] * normal[1][1] - normal[0][1] * normal[1][0];
            if (std::abs(determinant) < 1e-9f) break;
            step[0] = (normal[0][2] * normal[1][1] - normal[0][1] * normal[1][2]) / determinant;
            step[1] = (normal[0][0] * normal[1][2] - normal[1][0] * normal[0][2]) / determinant;
        }
        float size = 0.0f;
        for (std::size_t column = 0; column < free; ++column) {
            const RagdollAxisDefinition3D& limit = foot.inboundJoint.axes[axes[column]];
            const float next = std::clamp(component(angles, axes[column]) + std::clamp(step[column], -0.3f, 0.3f),
                limit.minimumRadians, limit.maximumRadians);
            size += std::abs(next - component(angles, axes[column]));
            setComponent(angles, axes[column], next);
        }
        if (size < 0.00005f) break;
    }
    return angles;
}

// A pose de mundo (raiz + coordenadas).
void worldPose(const RagdollProfile3D& profile, const std::vector<Vec3>& coordinates, Vec3 rootPosition,
    Quaternion rootOrientation, RagdollAnimationPose3D& pose) {
    buildRagdollLocalPose3D(profile, coordinates, pose);
    for (std::size_t i = 0; i < pose.linkPositions.size(); ++i) {
        pose.linkPositions[i] = rootPosition + rootOrientation.rotate(pose.linkPositions[i]);
        pose.linkOrientations[i] = (rootOrientation * pose.linkOrientations[i]).normalized();
    }
}

void addJoint(const RagdollProfile3D& profile, std::vector<Vec3>& coordinates, std::size_t index, std::size_t axis,
    float value) {
    if (index == 0 || index >= coordinates.size()) return;
    const RagdollAxisDefinition3D& limit = profile.links[index].inboundJoint.axes[axis];
    if (!limit.enabled) return;
    setComponent(coordinates[index], axis,
        std::clamp(component(coordinates[index], axis) + value, limit.minimumRadians, limit.maximumRadians));
}

Quaternion nlerpQuat(Quaternion a, Quaternion b, float t) {
    if (a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w < 0.0f) b = { -b.x, -b.y, -b.z, -b.w };
    return Quaternion { a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t, a.z + (b.z - a.z) * t,
        a.w + (b.w - a.w) * t }.normalized();
}

float smooth(float t) {
    t = std::clamp(t, 0.0f, 1.0f);
    return t * t * (3.0f - 2.0f * t);
}

} // namespace

BallGestureLegs3D resolveBallGestureLegs3D(const RagdollProfile3D& profile) {
    BallGestureLegs3D legs;
    const char* thighs[2] = { "LeftThigh", "RightThigh" };
    const char* shins[2] = { "LeftShin", "RightShin" };
    const char* feet[2] = { "LeftFoot", "RightFoot" };
    const char* uppers[2] = { "LeftUpperArm", "RightUpperArm" };
    const char* forearms[2] = { "LeftForearm", "RightForearm" };
    const char* hands[2] = { "LeftHand", "RightHand" };
    legs.valid = true;
    for (std::size_t side = 0; side < 2; ++side) {
        legs.thigh[side] = findLink(profile, thighs[side]);
        legs.shin[side] = findLink(profile, shins[side]);
        legs.foot[side] = findLink(profile, feet[side]);
        legs.upperArm[side] = findLink(profile, uppers[side]);
        legs.forearm[side] = findLink(profile, forearms[side]);
        legs.hand[side] = findLink(profile, hands[side]);
        if (legs.thigh[side] == 0 || legs.shin[side] == 0 || legs.foot[side] == 0
            || profile.links[legs.thigh[side]].parentIndex != 0) {
            legs.valid = false;
            continue;
        }
        const Vec3 hip = profile.links[legs.thigh[side]].inboundJoint.anchorModelPosition;
        const Vec3 knee = profile.links[legs.shin[side]].inboundJoint.anchorModelPosition;
        const Vec3 ankle = profile.links[legs.foot[side]].inboundJoint.anchorModelPosition;
        legs.hipInPelvis[side] = anchorIn(profile, 0, hip);
        legs.ankleInFoot[side] = anchorIn(profile, legs.foot[side], ankle);
        legs.ankleInShin[side] = anchorIn(profile, legs.shin[side], ankle);
        legs.thighLength[side] = (knee - hip).length();
        legs.shinLength[side] = (ankle - knee).length();
    }
    legs.neck = findLink(profile, "Neck");
    legs.head = findLink(profile, "Head");
    return legs;
}

Vec3 solveBallGestureLeg3D(const RagdollProfile3D& profile, const BallGestureLegs3D& legs, int side,
    std::vector<Vec3>& coordinates, Vec3 rootPosition, Quaternion rootOrientation, Vec3 ankleWorld,
    Vec3 kneeWorld, Quaternion footWorld) {
    if (!legs.valid || side < 0 || side > 1 || coordinates.size() != profile.links.size()) return ankleWorld;
    const std::size_t s = static_cast<std::size_t>(side);
    const std::size_t thighIndex = legs.thigh[s], shinIndex = legs.shin[s], footIndex = legs.foot[s];
    const RagdollLinkDefinition3D& thigh = profile.links[thighIndex];
    const RagdollLinkDefinition3D& shin = profile.links[shinIndex];
    const float a = legs.thighLength[s], b = legs.shinLength[s];
    const Vec3 hip = rootPosition + rootOrientation.rotate(legs.hipInPelvis[s]);
    const Vec3 v = ankleWorld - hip;
    const float distance = v.length();
    const Vec3 u = distance > 1e-5f ? v * (1.0f / distance) : rootOrientation.rotate({ 0.0f, 0.0f, -1.0f });
    const float D = std::clamp(distance, std::abs(a - b) + 0.03f, (a + b) * 0.9995f);
    const float cosHip = std::clamp((a * a + D * D - b * b) / (2.0f * a * D), -1.0f, 1.0f);
    const float sinHip = std::sqrt(std::max(0.0f, 1.0f - cosHip * cosHip));
    Vec3 pole = orthogonal(kneeWorld, u);
    if (pole.lengthSquared() < 0.5f) pole = orthogonal(rootOrientation.rotate({ 1.0f, 0.0f, 0.0f }), u);
    if (pole.lengthSquared() < 0.5f) pole = orthogonal({ 1.0f, 0.0f, 0.0f }, u);
    const Vec3 thighDirection = u * cosHip + pole * sinHip;
    const Vec3 kneePoint = hip + thighDirection * a;
    const Vec3 shinDirection = (hip + u * D - kneePoint).normalized();
    Vec3 kneeForward = orthogonal(pole, thighDirection);
    if (kneeForward.lengthSquared() < 0.5f) kneeForward = orthogonal(rootOrientation.rotate({ 1.0f, 0.0f, 0.0f }), thighDirection);
    // O link na pose de repouso: a coxa e a canela no eixo da junta (de cima
    // para baixo), a frente do joelho em +x do modelo.
    const Vec3 hipModel = thigh.inboundJoint.anchorModelPosition;
    const Vec3 kneeModel = shin.inboundJoint.anchorModelPosition;
    const Vec3 ankleModel = profile.links[footIndex].inboundJoint.anchorModelPosition;
    const Vec3 thighRest = (kneeModel - hipModel).normalized();
    const Vec3 shinRest = (ankleModel - kneeModel).normalized();
    const Vec3 thighRestForward = orthogonal({ 1.0f, 0.0f, 0.0f }, thighRest);
    const Vec3 shinRestForward = orthogonal({ 1.0f, 0.0f, 0.0f }, shinRest);
    const Quaternion thighRotation = alignPairs(thighRest, thighRestForward, thighDirection, kneeForward);
    const Quaternion thighWorld = (thighRotation * thigh.modelOrientation).normalized();
    // A canela gira so na dobradica do joelho (o eixo lateral da coxa).
    const Vec3 hinge = thighRotation.rotate(cross(thighRest, thighRestForward));
    const Vec3 shinForward = orthogonal(cross(hinge, shinDirection), shinDirection);
    const Quaternion shinRotation = alignPairs(shinRest, shinRestForward, shinDirection,
        shinForward.lengthSquared() > 0.5f ? shinForward : kneeForward);
    const Quaternion shinWorld = (shinRotation * shin.modelOrientation).normalized();
    const Quaternion rootLinkWorld = (rootOrientation * profile.links.front().modelOrientation).normalized();
    coordinates[thighIndex] = jointCoordinates(profile, thighIndex, rootLinkWorld, thighWorld);
    // (Com a coxa no limite o joelho muda de lugar: refeito pela coxa real.)
    RagdollAnimationPose3D pose;
    worldPose(profile, coordinates, rootPosition, rootOrientation, pose);
    coordinates[shinIndex] = jointCoordinates(profile, shinIndex, pose.linkOrientations[thighIndex], shinWorld);
    // O que faltar (limites) pelo IK numerico, partindo desta solucao (fica no
    // mesmo ramo: o joelho para o lado pedido).
    worldPose(profile, coordinates, rootPosition, rootOrientation, pose);
    const Vec3 reached = pose.linkPositions[footIndex]
        + pose.linkOrientations[footIndex].rotate(legs.ankleInFoot[s]);
    const Vec3 target = hip + u * D;
    if ((reached - target).lengthSquared() > 1e-6f) {
        solveRagdollChainPoint3D(profile, coordinates, shinIndex, legs.ankleInShin[s], 2,
            rootOrientation.conjugate().rotate(target - rootPosition), 1.0f, 8);
        worldPose(profile, coordinates, rootPosition, rootOrientation, pose);
    }
    coordinates[footIndex] = fitAnkle(profile, footIndex, pose.linkOrientations[shinIndex], footWorld,
        coordinates[footIndex]);
    worldPose(profile, coordinates, rootPosition, rootOrientation, pose);
    return pose.linkPositions[footIndex] + pose.linkOrientations[footIndex].rotate(legs.ankleInFoot[s]);
}

void ballGestureLegState3D(const RagdollProfile3D& profile, const BallGestureLegs3D& legs, int side,
    const std::vector<Vec3>& coordinates, Vec3 rootPosition, Quaternion rootOrientation, Vec3& ankle, Vec3& knee,
    Quaternion& foot) {
    if (!legs.valid || side < 0 || side > 1) return;
    const std::size_t s = static_cast<std::size_t>(side);
    RagdollAnimationPose3D pose;
    worldPose(profile, coordinates, rootPosition, rootOrientation, pose);
    ankle = pose.linkPositions[legs.foot[s]] + pose.linkOrientations[legs.foot[s]].rotate(legs.ankleInFoot[s]);
    const RagdollLinkDefinition3D& thigh = profile.links[legs.thigh[s]];
    knee = (pose.linkOrientations[legs.thigh[s]] * thigh.modelOrientation.conjugate()).rotate({ 1.0f, 0.0f, 0.0f });
    foot = pose.linkOrientations[legs.foot[s]];
}

void applyBallGesturePose3D(const RagdollProfile3D& profile, const BallGestureLegs3D& legs,
    const BallGesturePose3D& gesture, std::vector<Vec3>& coordinates, Vec3& rootPosition,
    Quaternion& rootOrientation, bool movePelvis, bool solveLegs) {
    if (!gesture.active || !legs.valid || coordinates.size() != profile.links.size()) return;
    const float w = std::clamp(gesture.weight, 0.0f, 1.0f);
    const int kick = std::clamp(gesture.side, 0, 1);
    const int support = 1 - kick;
    // As pernas onde a locomocao as pos (antes da pelve mexer).
    Vec3 baseAnkle[2], baseKnee[2];
    Quaternion baseFoot[2];
    for (int side = 0; side < 2; ++side)
        ballGestureLegState3D(profile, legs, side, coordinates, rootPosition, rootOrientation, baseAnkle[side],
            baseKnee[side], baseFoot[side]);
    // Pelve: gira em volta dela mesma (rumo no vertical; frente e lado nos
    // eixos do corpo) e desce/vai para o lado.
    if (movePelvis && w > 0.0f) {
        const Vec3 up { 0.0f, 0.0f, 1.0f };
        Vec3 forward = rootOrientation.rotate(profile.links.front().modelOrientation.conjugate().rotate({ 1.0f, 0.0f, 0.0f }));
        forward = Vec3 { forward.x, forward.y, 0.0f };
        forward = forward.lengthSquared() > 1e-6f ? forward.normalized() : Vec3 { 1.0f, 0.0f, 0.0f };
        const Vec3 left = cross(up, forward);
        const Quaternion turn = Quaternion::fromAxisAngle(up, gesture.pelvisYaw * w)
            * Quaternion::fromAxisAngle(left, gesture.pelvisPitch * w)
            * Quaternion::fromAxisAngle(forward, -gesture.pelvisRoll * w);
        rootOrientation = (turn * rootOrientation).normalized();
        rootPosition += Vec3 { gesture.pelvisShift.x * w, gesture.pelvisShift.y * w, -gesture.pelvisDrop * w };
    }
    // Tronco, repartido na coluna.
    {
        const char* spine[3] = { "Abdomen", "Chest", "UpperChest" };
        const float twistShare[3] = { 0.30f, 0.35f, 0.35f };
        const float bendShare[3] = { 0.40f, 0.32f, 0.28f };
        const float sideShare[3] = { 0.40f, 0.32f, 0.28f };
        for (int k = 0; k < 3; ++k) {
            const std::size_t index = findLink(profile, spine[k]);
            addJoint(profile, coordinates, index, 0, gesture.trunkTwist * twistShare[k] * w);
            addJoint(profile, coordinates, index, 1, gesture.trunkBend * bendShare[k] * w);
            addJoint(profile, coordinates, index, 2, gesture.trunkSide * sideShare[k] * w);
        }
    }
    // Bracos: o punho ate o alvo (em relacao ao ombro), pelo braco e o
    // antebraco.
    RagdollAnimationPose3D pose;
    worldPose(profile, coordinates, rootPosition, rootOrientation, pose);
    for (std::size_t side = 0; side < 2; ++side) {
        const float armWeight = std::clamp(gesture.armWeight[side] * w, 0.0f, 1.0f);
        const std::size_t upper = legs.upperArm[side], forearm = legs.forearm[side], hand = legs.hand[side];
        if (armWeight <= 0.001f || upper == 0 || forearm == 0 || hand == 0) continue;
        const Vec3 shoulder = pose.linkPositions[upper]
            + pose.linkOrientations[upper].rotate(anchorIn(profile, upper,
                profile.links[upper].inboundJoint.anchorModelPosition));
        Vec3 reach = gesture.wrist[side];
        constexpr float ArmReach = 0.56f;
        if (reach.length() > ArmReach) reach = reach * (ArmReach / reach.length());
        const Vec3 target = shoulder + reach;
        const Vec3 wristInForearm = anchorIn(profile, forearm, profile.links[hand].inboundJoint.anchorModelPosition);
        solveRagdollChainPoint3D(profile, coordinates, forearm, wristInForearm, 2,
            rootOrientation.conjugate().rotate(target - rootPosition), armWeight, 10);
    }
    // Cabeca: olha para o ponto (pescoco e cabeca, rumo e inclinacao).
    if (gesture.lookWeight > 0.001f && legs.head != 0 && legs.neck != 0) {
        worldPose(profile, coordinates, rootPosition, rootOrientation, pose);
        const Vec3 headPosition = pose.linkPositions[legs.head];
        const Vec3 face = (pose.linkOrientations[legs.head] * profile.links[legs.head].modelOrientation.conjugate())
            .rotate({ 1.0f, 0.0f, 0.0f });
        const Vec3 toward = gesture.look - headPosition;
        if (toward.lengthSquared() > 0.01f) {
            const Vec3 d = toward.normalized();
            float yawError = std::remainder(std::atan2(d.y, d.x) - std::atan2(face.y, face.x), 6.2831853f);
            const float pitchError = std::clamp(std::asin(std::clamp(d.z, -1.0f, 1.0f))
                - std::asin(std::clamp(face.z, -1.0f, 1.0f)), -0.7f, 0.3f);
            // O alvo atras da cabeca nao vale (o pescoco nao gira tanto e o
            // lado do giro trocava num tick); perto disso, o peso sai. (Quase
            // na vertical o "atras" tambem nao vale e entra pela distancia: a
            // bola passando por baixo da cabeca virava o atras de uma vez e a
            // cabeca estalava, 860 m/s2.)
            const float flat = std::hypot(toward.x, toward.y);
            const float turnedAway = smooth((std::abs(yawError) - 1.2f) / 0.8f);
            const float behind = turnedAway * smooth((flat - 0.06f) / 0.30f);
            // (O giro some antes de chegar atras: travado em +-1 ele trocava de
            // lado com a bola logo atras da cabeca e ela tremia, 570 m/s2.)
            yawError = std::clamp(yawError, -1.0f, 1.0f) * (1.0f - turnedAway);
            // (O alvo quase na vertical da cabeca: o rumo dele nao vale - um
            // centimetro para tras virava a cabeca 30 graus num tick.)
            yawError *= smooth((std::sqrt(d.x * d.x + d.y * d.y) - 0.06f) / 0.10f);
            const float lw = std::clamp(gesture.lookWeight * w * (1.0f - behind), 0.0f, 1.0f);
            addJoint(profile, coordinates, legs.neck, 0, yawError * 0.4f * lw);
            addJoint(profile, coordinates, legs.head, 0, yawError * 0.6f * lw);
            addJoint(profile, coordinates, legs.neck, 1, -pitchError * 0.45f * lw);
            addJoint(profile, coordinates, legs.head, 1, -pitchError * 0.55f * lw);
        }
    }
    if (!solveLegs) return;
    if (gesture.perLeg) {
        // Plano v5: cada perna da pose da locomocao ate o alvo dela, pelo peso;
        // sem peso, presa onde a locomocao a pos (a pelve mexeu em volta).
        for (int side = 0; side < 2; ++side) {
            const BallGesturePose3D::Leg& leg = gesture.legs[static_cast<std::size_t>(side)];
            const float weight = std::clamp(leg.weight, 0.0f, 1.0f);
            // (Presa primeiro e o toque por cima dela: sem peso a perna e a
            // presa, e o peso chegando a zero chega nela - antes ele chegava na
            // da locomocao e a presa entrava de uma vez, 2000 m/s2.)
            if (gesture.holdSupport && w > 0.001f)
                solveBallGestureLeg3D(profile, legs, side, coordinates, rootPosition, rootOrientation, baseAnkle[side],
                    baseKnee[side], baseFoot[side]);
            if (weight > 0.001f) {
                const Vec3 ankle = baseAnkle[side] + (leg.ankle - baseAnkle[side]) * weight;
                Vec3 knee = baseKnee[side] + (leg.knee - baseKnee[side]) * weight;
                if (knee.lengthSquared() < 1e-6f) knee = leg.knee;
                const std::vector<Vec3> before = coordinates;
                solveBallGestureLeg3D(profile, legs, side, coordinates, rootPosition, rootOrientation, ankle,
                    knee.normalized(), nlerpQuat(baseFoot[side], leg.foot, weight));
                // (Bem no comeco e no fim, a perna de antes: o IK da mesma pose
                // nao e identico a ela.)
                const float keep = smooth(weight * 5.0f);
                if (keep < 1.0f) {
                    const std::size_t s = static_cast<std::size_t>(side);
                    for (const std::size_t index : { legs.thigh[s], legs.shin[s], legs.foot[s] })
                        coordinates[index] = before[index] + (coordinates[index] - before[index]) * keep;
                }
            }
        }
        return;
    }
    // A perna que joga: da pose da locomocao ate o alvo do gesto, pelo peso.
    const float lw = std::clamp(gesture.legWeight, 0.0f, 1.0f);
    const Vec3 kickAnkle = baseAnkle[kick] + (gesture.ankle - baseAnkle[kick]) * lw;
    // Apoio: onde a locomocao o pos (o corpo girou e desceu em volta dele).
    // Com o gesto anterior cortado (a perna dele ainda no ar), ela vem do
    // alvo dele ate la pelo peso. As pernas nunca se cruzam: no ar, ela fica
    // do lado dela da que joga.
    const float release = gesture.releaseSide == support ? std::clamp(gesture.releaseWeight, 0.0f, 1.0f) : 0.0f;
    float apart = 0.0f;
    Vec3 apartDirection;
    if (gesture.separateGap > 0.0f && gesture.separateWeight > 0.001f) {
        Vec3 forward { gesture.separateForward.x, gesture.separateForward.y, 0.0f };
        forward = forward.lengthSquared() > 1e-6f ? forward.normalized() : Vec3 { 1.0f, 0.0f, 0.0f };
        apartDirection = cross(Vec3 { 0.0f, 0.0f, 1.0f }, forward) * (support == 0 ? 1.0f : -1.0f);
        Vec3 rel = baseAnkle[support] - kickAnkle;
        rel.z = 0.0f;
        const float along = std::abs(dot(rel, forward));
        const float overlap = 1.0f - smooth((along - 0.22f) / 0.16f);
        apart = std::max(0.0f, gesture.separateGap - dot(rel, apartDirection)) * overlap
            * std::clamp(gesture.separateWeight, 0.0f, 1.0f);
    }
    if (release > 0.001f) {
        const Vec3 ankle = baseAnkle[support] + (gesture.releaseAnkle - baseAnkle[support]) * release;
        Vec3 knee = baseKnee[support] + (gesture.releaseKnee - baseKnee[support]) * release;
        if (knee.lengthSquared() < 1e-6f) knee = baseKnee[support];
        solveBallGestureLeg3D(profile, legs, support, coordinates, rootPosition, rootOrientation, ankle,
            knee.normalized(), nlerpQuat(baseFoot[support], gesture.releaseFoot, release));
    } else if (apart > 0.0005f) {
        solveBallGestureLeg3D(profile, legs, support, coordinates, rootPosition, rootOrientation,
            baseAnkle[support] + apartDirection * apart, baseKnee[support], baseFoot[support]);
    } else if (gesture.holdSupport && w > 0.001f) {
        solveBallGestureLeg3D(profile, legs, support, coordinates, rootPosition, rootOrientation,
            baseAnkle[support], baseKnee[support], baseFoot[support]);
    }
    // A perna que joga: da pose da locomocao (presa no lugar, como o apoio)
    // ate o alvo do gesto, pelo peso.
    if (w > 0.001f || lw > 0.001f) {
        const Vec3 ankle = kickAnkle;
        Vec3 knee = baseKnee[kick] + (gesture.knee - baseKnee[kick]) * lw;
        if (knee.lengthSquared() < 1e-6f) knee = gesture.knee;
        const Quaternion foot = nlerpQuat(baseFoot[kick], gesture.foot, lw);
        const std::vector<Vec3> before = coordinates;
        solveBallGestureLeg3D(profile, legs, kick, coordinates, rootPosition, rootOrientation, ankle,
            knee.normalized(), foot);
        // (Bem no comeco e no fim, a perna da locomocao: o IK da mesma pose
        // nao e identico a ela.)
        const float keep = smooth(std::max(lw, w) * 5.0f);
        if (keep < 1.0f) {
            const std::size_t s = static_cast<std::size_t>(kick);
            for (const std::size_t index : { legs.thigh[s], legs.shin[s], legs.foot[s] })
                coordinates[index] = before[index] + (coordinates[index] - before[index]) * keep;
        }
    }
}

} // namespace MatterEngine
