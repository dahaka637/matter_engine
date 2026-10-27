#include "Engine/Character/DefaultLocomotion3DInternal.hpp"

namespace MatterEngine {
namespace {

// O giro de um link em relacao ao pai (no referencial do pai) a partir das
// coordenadas da junta, e o inverso (nos limites dela).
Quaternion linkTurn(const RagdollLinkDefinition3D& link, const RagdollLinkDefinition3D& parent, Vec3 coordinates) {
    const Quaternion pre = parent.modelOrientation.conjugate() * link.inboundJoint.frameModelOrientation;
    const Quaternion post = link.inboundJoint.frameModelOrientation.conjugate() * link.modelOrientation;
    return (pre * ragdollJointRotation3D(coordinates) * post).normalized();
}

Vec3 linkCoordinates(const RagdollLinkDefinition3D& link, const RagdollLinkDefinition3D& parent, Quaternion turn) {
    const Quaternion pre = parent.modelOrientation.conjugate() * link.inboundJoint.frameModelOrientation;
    const Quaternion post = link.inboundJoint.frameModelOrientation.conjugate() * link.modelOrientation;
    Vec3 value = rotationVector(pre.conjugate() * turn * post.conjugate());
    for (std::size_t axis = 0; axis < 3; ++axis) {
        const RagdollAxisDefinition3D& limit = link.inboundJoint.axes[axis];
        setComponent(value, axis,
            limit.enabled ? std::clamp(component(value, axis), limit.minimumRadians, limit.maximumRadians) : 0.0f);
    }
    return value;
}

} // namespace

GestureRig3D defaultGestureRig3D(const RagdollProfile3D& profile) {
    GestureRig3D rig;
    if (profile.links.empty()) return rig;
    const RagdollLinkDefinition3D& pelvis = profile.links.front();
    const Quaternion pelvisInverse = pelvis.modelOrientation.conjugate();
    const char* thighs[2] = { "LeftThigh", "RightThigh" };
    const char* shins[2] = { "LeftShin", "RightShin" };
    const char* feet[2] = { "LeftFoot", "RightFoot" };
    for (std::size_t side = 0; side < 2; ++side) {
        const std::size_t thigh = findLink(profile, thighs[side]), shin = findLink(profile, shins[side]),
                          foot = findLink(profile, feet[side]);
        if (thigh == 0 || shin == 0 || foot == 0) return rig;
        const Vec3 hip = profile.links[thigh].inboundJoint.anchorModelPosition;
        const Vec3 knee = profile.links[shin].inboundJoint.anchorModelPosition;
        const Vec3 ankle = profile.links[foot].inboundJoint.anchorModelPosition;
        GestureLegRig3D& leg = rig.legs[side];
        leg.hip = pelvisInverse.rotate(hip - pelvis.modelPosition);
        leg.thigh = (knee - hip).length();
        leg.shin = (ankle - knee).length();
        leg.ankleHeight = ankle.z - soleHeight(profile.links[foot], profile.links[foot].modelPosition,
            profile.links[foot].modelOrientation);
        leg.hipFrame = (pelvis.modelOrientation.conjugate() * profile.links[thigh].inboundJoint.frameModelOrientation)
                           .normalized();
    }
    rig.pelvisHeight = profile.standingRootHeightMeters;
    rig.valid = true;
    return rig;
}

void DefaultLocomotion3D::stepGesture(Tick& t) {
    const RagdollProfile3D& profile = t.profile;
    const DefaultLocomotionInput3D& input = t.input;
    const float dt = t.dt;
    Vec3& rootPosition = t.rootPosition;
    Quaternion& rootOrientation = t.rootOrientation;
    std::array<bool, 2>& gestureFeet = t.gestureFeet;
    m_gestureSupportValid = {};
    if (!m_gestureRig.valid) {
        m_gestureRig = defaultGestureRig3D(profile);
        const char* thighs[2] = { "LeftThigh", "RightThigh" };
        const char* shins[2] = { "LeftShin", "RightShin" };
        for (std::size_t side = 0; side < 2; ++side) {
            m_gestureLegLinks[side] = { findLink(profile, thighs[side]), findLink(profile, shins[side]), m_feet[side] };
        }
        if (!m_gestureRig.valid) return;
    }
    const auto readLeg = [&](std::size_t side, const std::vector<Vec3>& coordinates) {
        GestureLegPose3D pose;
        const auto& links = m_gestureLegLinks[side];
        if (links[2] >= coordinates.size()) return pose;
        pose.set(coordinates[links[0]], coordinates[links[1]].x, coordinates[links[2]]);
        return pose;
    };
    // A perna como a locomocao a poria agora (antes do gesto).
    std::array<GestureLegPose3D, 2> live {};
    for (std::size_t side = 0; side < 2; ++side) live[side] = readLeg(side, m_coordinates);
    if (input.gestureRequested) {
        const std::size_t side = input.gesture.side == 1 ? 1 : 0;
        const bool shown = m_gestureShownValid[side];
        // (Andando tambem com o tempo do menu: a passada se adapta, stepGait.)
        const GestureSpec3D& spec = input.gesture;
        // O catalogo pensa no corpo (o rumo, de pe) e resolve na pelve como ela
        // esta agora (a pose da pelve gira e inclina: parado, 22 graus).
        // (O modelo da pelve e o do corpo: o giro dela no mundo e o da raiz.)
        GestureRig3D rig = m_gestureRig;
        rig.bodyToPelvis
            = (rootOrientation.conjugate() * Quaternion::fromAxisAngle({ 0.0f, 0.0f, 1.0f }, m_yaw)).normalized();
        // (A bola alta e a recepcao miram a bola de verdade: a pelve na altura
        // de agora - parado, 6 cm abaixo da do modelo de pe reto; os gestos
        // rentes ao chao foram acertados com a do modelo.)
        if (spec.high || spec.receive)
            rig.pelvisHeight = std::clamp(rootPosition.z - input.feetPositionWorld.z, 0.6f, 1.2f);
        m_gesturePlayers[side].start(spec, rig, shown ? m_gestureShown[side] : live[side],
            shown ? m_gestureShownVelocity[side] : GestureLegPose3D {});
        // (O gesto da outra perna termina logo: as duas no ar, nenhum pe
        // apoiava.)
        m_gesturePlayers[1 - side].finishSoon(0.1f);
    }
    if (!m_gesturePlayers[0].active() && !m_gesturePlayers[1].active()) return;
    // O pe de apoio fica onde a locomocao o pos: a pose dele no mundo antes do
    // gesto mexer na pelve (os pes vao para la em stepFeet).
    for (std::size_t side = 0; side < 2; ++side) {
        if (m_gesturePlayers[side].movesLeg() || m_feet[side] == 0) continue;
        RagdollAnimationPose3D chain;
        buildRagdollChainPose3D(profile, m_coordinates, chain, m_feet[side]);
        m_gestureSupport[side] = { rootPosition + rootOrientation.rotate(chain.linkPositions[m_feet[side]]),
            (rootOrientation * chain.linkOrientations[m_feet[side]]).normalized() };
        m_gestureSupportValid[side] = true;
    }
    GestureBody3D body;
    // O quanto o gesto toma os bracos agora (0..1: sobe no primeiro terco e
    // desce ate o fim).
    float armTake = 0.0f;
    for (std::size_t side = 0; side < 2; ++side) {
        if (!m_gesturePlayers[side].active()) continue;
        const GestureFrame3D frame = m_gesturePlayers[side].update(dt, live[side], m_moving);
        if (!frame.active) continue;
        // (Nos limites das juntas: nos gestos longos a curva passava do ponto e
        // o joelho chegava a -7 graus.)
        const auto& links = m_gestureLegLinks[side];
        const auto limited = [&](std::size_t link, Vec3 value) {
            for (std::size_t axis = 0; axis < 3; ++axis) {
                const RagdollAxisDefinition3D& limit = profile.links[link].inboundJoint.axes[axis];
                setComponent(value, axis,
                    limit.enabled ? std::clamp(component(value, axis), limit.minimumRadians, limit.maximumRadians) : 0.0f);
            }
            return value;
        };
        // (O peito e a cabeca so mexem no corpo: a perna segue a locomocao.)
        if (m_gesturePlayers[side].movesLeg()) {
            m_coordinates[links[0]] = limited(links[0], frame.leg.thigh());
            m_coordinates[links[1]] = limited(links[1], { frame.leg.knee(), 0.0f, 0.0f });
            m_coordinates[links[2]] = limited(links[2], frame.leg.foot());
            gestureFeet[side] = true;
        }
        armTake = std::max(armTake, gestureBump(frame.time, 0.0f, 0.3f * frame.duration, frame.duration));
        const GestureBody3D& b = frame.body;
        body.pelvisDrop += b.pelvisDrop;
        body.pelvisForward += b.pelvisForward;
        body.pelvisLeft += b.pelvisLeft;
        body.pelvisYaw += b.pelvisYaw;
        body.pelvisRoll += b.pelvisRoll;
        for (std::size_t i = 0; i < 3; ++i) body.spine[i] += b.spine[i];
        body.neck += b.neck;
        body.head += b.head;
        for (std::size_t arm = 0; arm < 2; ++arm) {
            body.armForward[arm] += b.armForward[arm];
            body.armOut[arm] += b.armOut[arm];
            body.armAcross[arm] += b.armAcross[arm];
            body.elbow[arm] += b.elbow[arm];
        }
    }
    // A pelve: desce, anda no rumo do corpo e gira (em volta da vertical e do
    // rumo); a coluna, o pescoco e a cabeca somam; os bracos giram no ombro (no
    // referencial do peito: para a frente e para fora) e o cotovelo dobra.
    const Vec3 forward { std::cos(m_yaw), std::sin(m_yaw), 0.0f };
    const Vec3 left { -forward.y, forward.x, 0.0f };
    rootPosition += forward * body.pelvisForward + left * body.pelvisLeft - Vec3 { 0.0f, 0.0f, body.pelvisDrop };
    rootOrientation = (Quaternion::fromAxisAngle({ 0.0f, 0.0f, 1.0f }, body.pelvisYaw)
        * Quaternion::fromAxisAngle(forward, body.pelvisRoll) * rootOrientation).normalized();
    const char* spine[3] = { "Abdomen", "Chest", "UpperChest" };
    for (std::size_t i = 0; i < 3; ++i)
        for (std::size_t axis = 0; axis < 3; ++axis) addCoordinate(profile, m_coordinates, spine[i], axis, component(body.spine[i], axis));
    for (std::size_t axis = 0; axis < 3; ++axis) {
        addCoordinate(profile, m_coordinates, "Neck", axis, component(body.neck, axis));
        addCoordinate(profile, m_coordinates, "Head", axis, component(body.head, axis));
    }
    const char* uppers[2] = { "LeftUpperArm", "RightUpperArm" };
    const char* forearms[2] = { "LeftForearm", "RightForearm" };
    // Andando, o balanco da corrida nos bracos cede para a media dele durante o
    // gesto (70% no meio): os bracos fazem o do gesto. (Por cima do balanco,
    // os dois brigavam: a mao subia ao queixo e o braco cruzava o peito.)
    const float settle = 0.7f * armTake * std::clamp(m_moving, 0.0f, 1.0f);
    if (settle > 1e-4f) {
        float total = 0.0f;
        for (std::size_t g = 0; g < GaitCount; ++g) total += m_gaits[g].mean.empty() ? 0.0f : m_weights[g];
        for (std::size_t arm = 0; arm < 2 && total > 1e-3f; ++arm)
            for (const char* id : { uppers[arm], forearms[arm] }) {
                const std::size_t link = findLink(profile, id);
                if (link == 0 || link >= m_coordinates.size()) continue;
                Vec3 mean {};
                for (std::size_t g = 0; g < GaitCount; ++g)
                    if (link < m_gaits[g].mean.size()) mean += m_gaits[g].mean[link] * (m_weights[g] / total);
                m_coordinates[link] += (mean - m_coordinates[link]) * settle;
            }
    }
    for (std::size_t arm = 0; arm < 2; ++arm) {
        const std::size_t upper = findLink(profile, uppers[arm]);
        if (upper == 0 || upper >= m_coordinates.size() || profile.links[upper].parentIndex < 0) continue;
        const RagdollLinkDefinition3D& link = profile.links[upper];
        const RagdollLinkDefinition3D& parent = profile.links[static_cast<std::size_t>(link.parentIndex)];
        // (O braco pendurado vai a frente girando em -y do peito, para fora em
        // +x no esquerdo e -x no direito, e de lado em volta de z.)
        const float outSign = arm == 0 ? 1.0f : -1.0f;
        const Quaternion swing = (Quaternion::fromAxisAngle({ 0.0f, 0.0f, 1.0f }, -outSign * body.armAcross[arm])
            * Quaternion::fromAxisAngle({ 0.0f, 1.0f, 0.0f }, -body.armForward[arm])
            * Quaternion::fromAxisAngle({ 1.0f, 0.0f, 0.0f }, outSign * body.armOut[arm])).normalized();
        m_coordinates[upper] = linkCoordinates(link, parent, (swing * linkTurn(link, parent, m_coordinates[upper])).normalized());
        addCoordinate(profile, m_coordinates, forearms[arm], 0, body.elbow[arm]);
    }
}

void DefaultLocomotion3D::recordGestureShown(float dt) {
    if (!m_gestureRig.valid) return;
    for (std::size_t side = 0; side < 2; ++side) {
        const auto& links = m_gestureLegLinks[side];
        if (links[2] >= m_coordinates.size() || links[2] == 0) continue;
        GestureLegPose3D pose;
        pose.set(m_coordinates[links[0]], m_coordinates[links[1]].x, m_coordinates[links[2]]);
        if (m_gestureShownValid[side])
            for (std::size_t i = 0; i < 7; ++i)
                m_gestureShownVelocity[side].value[i] = (pose.value[i] - m_gestureShown[side].value[i]) / std::max(1e-4f, dt);
        m_gestureShown[side] = pose;
        m_gestureShownValid[side] = true;
    }
}

} // namespace MatterEngine
