#include "Engine/Character/BodilyGetUp3D.hpp"

#include "Engine/Animation/AnimationClip3D.hpp"
#include "Engine/Character/RagdollPoseKinematics3D.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <string_view>

namespace MatterEngine {
namespace {

float smoothStep(float value) {
    value = std::clamp(value, 0.0f, 1.0f);
    return value * value * (3.0f - 2.0f * value);
}

float component(Vec3 value, std::size_t axis) {
    return axis == 0 ? value.x : axis == 1 ? value.y : value.z;
}

void setComponent(Vec3& value, std::size_t axis, float componentValue) {
    if (axis == 0) value.x = componentValue;
    else if (axis == 1) value.y = componentValue;
    else value.z = componentValue;
}

Vec3 lerp(Vec3 from, Vec3 to, float amount) { return from + (to - from) * amount; }

Quaternion nlerp(Quaternion from, Quaternion to, float amount) {
    if (from.x * to.x + from.y * to.y + from.z * to.z + from.w * to.w < 0.0f)
        to = { -to.x, -to.y, -to.z, -to.w };
    return Quaternion { from.x + (to.x - from.x) * amount, from.y + (to.y - from.y) * amount,
        from.z + (to.z - from.z) * amount, from.w + (to.w - from.w) * amount }.normalized();
}

// Orientacao com estes eixos x, y, z (ortonormais) nas colunas.
Quaternion fromBasis(Vec3 x, Vec3 y, Vec3 z) {
    const float trace = x.x + y.y + z.z;
    Quaternion q;
    if (trace > 0.0f) {
        const float s = std::sqrt(trace + 1.0f) * 2.0f;
        q = { (y.z - z.y) / s, (z.x - x.z) / s, (x.y - y.x) / s, 0.25f * s };
    } else if (x.x > y.y && x.x > z.z) {
        const float s = std::sqrt(1.0f + x.x - y.y - z.z) * 2.0f;
        q = { 0.25f * s, (y.x + x.y) / s, (z.x + x.z) / s, (y.z - z.y) / s };
    } else if (y.y > z.z) {
        const float s = std::sqrt(1.0f + y.y - x.x - z.z) * 2.0f;
        q = { (y.x + x.y) / s, 0.25f * s, (z.y + y.z) / s, (z.x - x.z) / s };
    } else {
        const float s = std::sqrt(1.0f + z.z - x.x - y.y) * 2.0f;
        q = { (z.x + x.z) / s, (z.y + y.z) / s, 0.25f * s, (x.y - y.x) / s };
    }
    return q.normalized();
}

// Orientacao do corpo com o tronco ao longo de `along` e a esquerda perto de
// `left` (frente = esquerda x tronco, como em pe).
Quaternion bodyOrientation(Vec3 along, Vec3 left) {
    const Vec3 z = along.normalized();
    Vec3 y = left - z * dot(left, z);
    y = y.lengthSquared() > 1e-6f ? y.normalized() : Vec3 { 0.0f, 1.0f, 0.0f };
    return fromBasis(cross(y, z), y, z);
}

std::size_t linkIndex(const RagdollProfile3D& profile, std::string_view id) {
    for (std::size_t index = 0; index < profile.links.size(); ++index)
        if (profile.links[index].id == id) return index;
    return 0;
}

void setAngle(const RagdollProfile3D& profile, std::vector<Vec3>& coordinates,
    std::size_t link, std::size_t axis, float value) {
    if (link == 0 || link >= coordinates.size()) return;
    const RagdollAxisDefinition3D& limit = profile.links[link].inboundJoint.axes[axis];
    if (!limit.enabled) return;
    setComponent(coordinates[link], axis, std::clamp(value, limit.minimumRadians, limit.maximumRadians));
}

// Ponto da junta de `child` no referencial do link pai.
Vec3 anchorInParent(const RagdollProfile3D& profile, std::size_t child) {
    const RagdollLinkDefinition3D& link = profile.links[child];
    const RagdollLinkDefinition3D& parent = profile.links[static_cast<std::size_t>(link.parentIndex)];
    return parent.modelOrientation.conjugate().rotate(link.inboundJoint.anchorModelPosition - parent.modelPosition);
}

Vec3 linkPoint(const RagdollState3D& state, std::size_t link, Vec3 local) {
    return state.links[link].position + state.links[link].orientation.rotate(local);
}

// Sinal do lado: +1 esquerda (y+), -1 direita.
float lateral(std::size_t side) { return side == 0 ? 1.0f : -1.0f; }

} // namespace

void BodilyGetUp3D::resolveLinks(const RagdollProfile3D& profile) {
    Links links;
    links.abdomen = linkIndex(profile, "Abdomen");
    links.chest = linkIndex(profile, "Chest");
    links.upperChest = linkIndex(profile, "UpperChest");
    links.neck = linkIndex(profile, "Neck");
    links.head = linkIndex(profile, "Head");
    links.valid = links.abdomen && links.chest && links.upperChest && links.neck && links.head;
    const char* sides[2] = { "Left", "Right" };
    for (std::size_t side = 0; side < 2; ++side) {
        const std::string prefix = sides[side];
        links.upperArm[side] = linkIndex(profile, prefix + "UpperArm");
        links.forearm[side] = linkIndex(profile, prefix + "Forearm");
        links.hand[side] = linkIndex(profile, prefix + "Hand");
        links.thigh[side] = linkIndex(profile, prefix + "Thigh");
        links.shin[side] = linkIndex(profile, prefix + "Shin");
        links.foot[side] = linkIndex(profile, prefix + "Foot");
        links.valid = links.valid && links.upperArm[side] && links.forearm[side] && links.hand[side]
            && links.thigh[side] && links.shin[side] && links.foot[side]
            && profile.links[links.upperArm[side]].parentIndex == static_cast<int>(links.upperChest);
        if (!links.valid) continue;
        links.wristInForearm[side] = anchorInParent(profile, links.hand[side]);
        links.kneeInThigh[side] = anchorInParent(profile, links.shin[side]);
        links.ankleInShin[side] = anchorInParent(profile, links.foot[side]);
        links.shoulderInChest[side] = anchorInParent(profile, links.upperArm[side]);
        links.hipInPelvis[side] = anchorInParent(profile, links.thigh[side]);
        // Bola do pe: na frente da sola, embaixo.
        const RagdollCapsuleDefinition3D& foot = profile.links[links.foot[side]].collider;
        const Vec3 half = foot.shape == RagdollColliderShape3D::Box
            ? foot.boxHalfExtents : Vec3 { 0.14f, 0.05f, 0.065f };
        links.ballInFoot[side] = foot.localPosition
            + foot.localOrientation.rotate({ 0.62f * half.x, 0.0f, -half.z });
    }
    m_legWeights.assign(profile.links.size(), Vec3 { 1.0f, 1.0f, 1.0f });
    if (links.valid) {
        for (std::size_t side = 0; side < 2; ++side) {
            m_legWeights[links.thigh[side]] = { 0.12f, 1.0f, 0.30f };
            m_legWeights[links.foot[side]] = { 1.0f, 0.30f, 1.0f };
        }
    }
    m_legWeightsAnkleLocked = m_legWeights;
    if (links.valid) {
        for (std::size_t side = 0; side < 2; ++side) m_legWeightsAnkleLocked[links.foot[side]] = {};
    }
    if (links.valid) {
        const auto anchor = [&](std::size_t link) { return profile.links[link].inboundJoint.anchorModelPosition; };
        links.armLength = (anchor(links.forearm[0]) - anchor(links.upperArm[0])).length()
            + (anchor(links.hand[0]) - anchor(links.forearm[0])).length();
        links.thighLength = (anchor(links.shin[0]) - anchor(links.thigh[0])).length();
        links.shinLength = (anchor(links.foot[0]) - anchor(links.shin[0])).length();
    }
    m_links = links;
}

void BodilyGetUp3D::measure(const RagdollProfile3D& profile, const RagdollState3D& state,
    float groundHeight) {
    Body body;
    const auto& pelvis = state.links[0];
    const auto& chest = state.links[m_links.upperChest];
    body.pelvis = pelvis.position;
    body.pelvisOrientation = pelvis.orientation;
    body.chest = chest.position;
    body.along = (chest.position - pelvis.position).normalized();
    body.anterior = chest.orientation.rotate({ 1.0f, 0.0f, 0.0f });
    body.left = chest.orientation.rotate({ 0.0f, 1.0f, 0.0f });
    body.pelvisAnterior = pelvis.orientation.rotate({ 1.0f, 0.0f, 0.0f });
    body.pelvisLeft = pelvis.orientation.rotate({ 0.0f, 1.0f, 0.0f });
    float mass = 0.0f;
    for (std::size_t index = 0; index < state.links.size() && index < profile.links.size(); ++index) {
        const float m = profile.links[index].massFraction;
        body.centerOfMass += linkPoint(state, index, profile.links[index].centerOfMassLocal) * m;
        body.centerOfMassVelocity += state.links[index].linearVelocity * m;
        mass += m;
    }
    if (mass > 0.0f) {
        body.centerOfMass *= 1.0f / mass;
        body.centerOfMassVelocity *= 1.0f / mass;
    }
    body.coreSpeed = std::max(pelvis.linearVelocity.length(), chest.linearVelocity.length());
    body.chestHeight = chest.position.z - groundHeight;
    body.pelvisHeight = pelvis.position.z - groundHeight;
    for (std::size_t side = 0; side < 2; ++side) {
        body.shoulder[side] = linkPoint(state, m_links.upperChest, m_links.shoulderInChest[side]);
        body.wrist[side] = linkPoint(state, m_links.forearm[side], m_links.wristInForearm[side]);
        body.knee[side] = linkPoint(state, m_links.thigh[side], m_links.kneeInThigh[side]);
        body.ankle[side] = linkPoint(state, m_links.shin[side], m_links.ankleInShin[side]);
        body.ball[side] = linkPoint(state, m_links.foot[side], m_links.ballInFoot[side]);
    }
    body.trunkPitch = std::asin(std::clamp(body.along.z, -1.0f, 1.0f));
    {
        Vec3 heading = body.along;
        heading.z = 0.0f;
        body.heading = heading.lengthSquared() > 1e-4f ? heading.normalized() : m_body.heading;
    }
    m_groundHeight = groundHeight;
    m_measuredJoints.assign(profile.links.size(), Vec3 {});
    for (std::size_t link = 1; link < state.joints.size() && link < profile.links.size(); ++link)
        m_measuredJoints[link] = { state.joints[link].positionRadians[0],
            state.joints[link].positionRadians[1], state.joints[link].positionRadians[2] };
    for (const RagdollInteraction3D& contact : state.interactions) {
        if (contact.normalWorld.z < 0.4f) continue;
        for (std::size_t side = 0; side < 2; ++side) {
            if (contact.linkIndex == m_links.hand[side] || contact.linkIndex == m_links.forearm[side])
                body.handContact[side] = true;
            // O joelho apoiado encosta pela canela ou pela ponta da coxa.
            if (contact.linkIndex == m_links.shin[side] || contact.linkIndex == m_links.thigh[side])
                body.kneeContact[side] = true;
            if (contact.linkIndex == m_links.foot[side]) body.footContact[side] = true;
        }
    }
    m_body = body;
}

void BodilyGetUp3D::enter(BodilyGetUpStage3D stage) {
    m_stage = stage;
    m_stageSeconds = 0.0f;
    m_stageProgress = 0.0f;
    m_stalledSeconds = 0.0f;
    m_bestProgress = -2.0f;
    m_stageFrom = m_previousCoordinates;
    m_fromHeight = m_body.pelvisHeight;
    m_fromPitch = m_body.trunkPitch;
    m_fromCenterOfMass = m_body.centerOfMass;
}

void BodilyGetUp3D::begin(const RagdollProfile3D& profile, const RagdollState3D& state,
    float groundHeight) {
    resolveLinks(profile);
    m_failed = !m_links.valid || state.links.size() != profile.links.size()
        || state.joints.size() != profile.links.size();
    m_telemetry.attempts += 1;
    if (m_failed) return;
    m_previousCoordinates.assign(profile.links.size(), Vec3 {});
    for (std::size_t link = 1; link < profile.links.size(); ++link) {
        m_previousCoordinates[link] = { state.joints[link].positionRadians[0],
            state.joints[link].positionRadians[1], state.joints[link].positionRadians[2] };
    }
    measure(profile, state, groundHeight);
    m_rollFlips = 0;
    // De bruços (peito e quadril): direto para as maos. Senao rola para o lado
    // que ja esta mais baixo.
    if (m_body.anterior.z < -0.55f && m_body.pelvisAnterior.z < -0.55f) {
        m_rollSide = 0;
        enter(BodilyGetUpStage3D::PlaceHands);
    } else {
        const Vec3 left = m_body.anterior.z < -0.55f ? m_body.pelvisLeft : m_body.left;
        m_rollSide = left.z <= 0.0f ? 1 : -1;
        enter(BodilyGetUpStage3D::Roll);
    }
}

void BodilyGetUp3D::neutral(const RagdollProfile3D& profile, std::vector<Vec3>& pose) const {
    pose.assign(profile.links.size(), Vec3 {});
}

Vec3 BodilyGetUp3D::shoulderInRoot(const RagdollProfile3D& profile,
    const std::vector<Vec3>& pose, std::size_t side) const {
    RagdollAnimationPose3D frame;
    buildRagdollLocalPose3D(profile, pose, frame);
    return frame.linkPositions[m_links.upperChest]
        + frame.linkOrientations[m_links.upperChest].rotate(m_links.shoulderInChest[side]);
}

void BodilyGetUp3D::aimWrist(const RagdollProfile3D& profile, std::vector<Vec3>& pose,
    std::size_t side, Vec3 offsetFromShoulder) const {
    const Vec3 target = shoulderInRoot(profile, pose, side) + offsetFromShoulder;
    solveRagdollChainPoint3D(profile, pose, m_links.forearm[side], m_links.wristInForearm[side], 2,
        target, 1.0f, 18);
}

void BodilyGetUp3D::aimKnee(const RagdollProfile3D& profile, std::vector<Vec3>& pose,
    std::size_t side, Vec3 offsetFromHip) const {
    const Vec3 target = m_links.hipInPelvis[side] + offsetFromHip;
    solveRagdollChainPoint3D(profile, pose, m_links.thigh[side], m_links.kneeInThigh[side], 1,
        target, 1.0f, 18, &m_legWeights);
}

void BodilyGetUp3D::legsStraight(const RagdollProfile3D& profile, std::vector<Vec3>& pose) const {
    for (std::size_t side = 0; side < 2; ++side) {
        setAngle(profile, pose, m_links.thigh[side], 2, -lateral(side) * 0.08f);
        setAngle(profile, pose, m_links.shin[side], 0, 0.10f);
        // Peito do pe no chao (pe esticado para tras).
        setAngle(profile, pose, m_links.foot[side], 0, 0.60f);
    }
}

// Maos ao lado do peito, palmas no chao, cotovelos dobrados para tras
// (push 0); bracos estendidos perpendiculares ao tronco (push 1).
void BodilyGetUp3D::armsBeside(const RagdollProfile3D& profile, std::vector<Vec3>& pose,
    float push) const {
    for (std::size_t side = 0; side < 2; ++side) {
        const float s = lateral(side);
        // Ponto de partida do alcance: braco junto do corpo, cotovelo dobrado
        // (o cotovelo fica para tras, como numa flexao).
        setAngle(profile, pose, m_links.upperArm[side], 1, 1.20f * (1.0f - push) + 0.20f);
        setAngle(profile, pose, m_links.upperArm[side], 2, -s * 0.70f * push);
        setAngle(profile, pose, m_links.forearm[side], 0, 1.60f * (1.0f - push) + 0.05f);
        const float reach = 0.14f + (m_links.armLength * 0.95f - 0.14f) * push;
        aimWrist(profile, pose, side, { reach, s * (0.16f - 0.09f * push), -0.10f + 0.04f * push });
    }
}

// Joelhos no chao debaixo do quadril: coxa perpendicular ao tronco, canela
// deitada para tras.
void BodilyGetUp3D::legsKneeling(const RagdollProfile3D& profile, std::vector<Vec3>& pose,
    float ankle) const {
    for (std::size_t side = 0; side < 2; ++side) {
        setAngle(profile, pose, m_links.thigh[side], 1, -1.40f);
        aimKnee(profile, pose, side, { m_links.thighLength * 0.98f, lateral(side) * 0.03f, 0.0f });
        setAngle(profile, pose, m_links.shin[side], 0, 1.57f);
        setAngle(profile, pose, m_links.foot[side], 0, ankle);
    }
}

Vec3 BodilyGetUp3D::toRoot(Vec3 world) const {
    return m_body.pelvisOrientation.conjugate().rotate(world - m_body.pelvis);
}

Vec3 BodilyGetUp3D::toDesired(Vec3 world) const {
    return m_desiredOrientation.conjugate().rotate(world - m_desiredRoot);
}

void BodilyGetUp3D::servoRoot(float pelvisHeight, float trunkPitch, Vec3 centerOfMassTarget, Vec3 pivot) {
    // O quanto a pelve desejada se adianta da medida: e esse descompasso, nas
    // juntas dos membros apoiados, que vira o empurrao.
    constexpr float Reach = 0.05f;
    constexpr float Turn = 0.12f;
    Vec3 shift = centerOfMassTarget - m_body.centerOfMass;
    shift.z = 0.0f;
    const float length = shift.length();
    if (length > Reach) shift *= Reach / length;
    const float rise = pelvisHeight >= 0.0f
        ? std::clamp(pelvisHeight - m_body.pelvisHeight, -Reach, Reach) : 0.0f;
    const float turn = std::clamp(trunkPitch - m_body.trunkPitch, -Turn, Turn);
    const Vec3 left { -m_body.heading.y, m_body.heading.x, 0.0f };
    const Quaternion pitch = Quaternion::fromAxisAngle(left, -turn);
    m_desiredRoot = pivot + pitch.rotate(m_body.pelvis - pivot) + shift + Vec3 { 0.0f, 0.0f, rise };
    m_desiredOrientation = (pitch * m_body.pelvisOrientation).normalized();
}

void BodilyGetUp3D::holdPlants(const RagdollProfile3D& profile, std::vector<Vec3>& pose) const {
    for (std::size_t side = 0; side < 2; ++side) {
        if (m_handPlanted[side]) {
            // Braco de apoio e escora: cotovelo quase reto, so o ombro aponta
            // a mao para o ponto do chao (encurtar o braco era desabar).
            setAngle(profile, pose, m_links.forearm[side], 0, 0.06f);
            RagdollAnimationPose3D frame;
            buildRagdollLocalPose3D(profile, pose, frame);
            const std::size_t upper = m_links.upperArm[side];
            const std::size_t fore = m_links.forearm[side];
            const Vec3 wrist = frame.linkPositions[fore] + frame.linkOrientations[fore].rotate(m_links.wristInForearm[side]);
            const Vec3 wristInUpper = frame.linkOrientations[upper].conjugate().rotate(wrist - frame.linkPositions[upper]);
            solveRagdollChainPoint3D(profile, pose, upper, wristInUpper, 1, toDesired(m_handPlant[side]), 1.0f, 18);
        }
        if (!m_footPlanted[side]) continue;
        if (m_footFlat[side]) {
            solveRagdollChainPoint3D(profile, pose, m_links.shin[side], m_links.ankleInShin[side], 2,
                toDesired(m_footPlant[side]), 1.0f, 18, &m_legWeights);
            flattenFoot(profile, pose, side);
        } else {
            solveRagdollChainPoint3D(profile, pose, m_links.foot[side], m_links.ballInFoot[side], 3,
                toDesired(m_footPlant[side]), 1.0f, 18, &m_legWeights);
        }
    }
}

void BodilyGetUp3D::plantFromBody(bool hands, bool feet) {
    for (std::size_t side = 0; side < 2; ++side) {
        if (hands) {
            m_handPlant[side] = m_body.wrist[side];
            m_handPlanted[side] = true;
        }
        if (feet) {
            m_footPlant[side] = m_body.ball[side];
            m_footPlanted[side] = true;
            m_footFlat[side] = false;
        }
    }
}

void BodilyGetUp3D::placeAnkle(const RagdollProfile3D& profile, std::vector<Vec3>& pose,
    std::size_t side, Vec3 targetWorld) const {
    solveRagdollChainPoint3D(profile, pose, m_links.shin[side], m_links.ankleInShin[side], 2,
        toDesired(targetWorld), 1.0f, 18, &m_legWeights);
    flattenFoot(profile, pose, side);
}

// Sola paralela ao chao medido: procura o tornozelo (flexao e inversao) que
// deixa o "para cima" do pe junto do "para cima" do mundo.
void BodilyGetUp3D::flattenFoot(const RagdollProfile3D& profile, std::vector<Vec3>& pose,
    std::size_t side) const {
    const std::size_t foot = m_links.foot[side];
    const Vec3 up = m_desiredOrientation.conjugate().rotate({ 0.0f, 0.0f, 1.0f });
    const RagdollLinkDefinition3D& link = profile.links[foot];
    float best = -2.0f;
    Vec3 bestAngles = pose[foot];
    RagdollAnimationPose3D frame;
    for (int i = 0; i <= 16; ++i) {
        for (int j = -3; j <= 3; ++j) {
            Vec3 angles = pose[foot];
            setComponent(angles, 0, std::clamp(-0.85f + 1.85f * static_cast<float>(i) / 16.0f,
                link.inboundJoint.axes[0].minimumRadians, link.inboundJoint.axes[0].maximumRadians));
            if (link.inboundJoint.axes[1].enabled)
                setComponent(angles, 1, std::clamp(0.18f * static_cast<float>(j),
                    link.inboundJoint.axes[1].minimumRadians, link.inboundJoint.axes[1].maximumRadians));
            pose[foot] = angles;
            buildRagdollLocalPose3D(profile, pose, frame);
            const float alignment = dot(frame.linkOrientations[foot].rotate({ 0.0f, 0.0f, 1.0f }), up);
            if (alignment > best) {
                best = alignment;
                bestAngles = angles;
            }
        }
    }
    pose[foot] = bestAngles;
}

// Ajoelhado: canelas no chao, dedos dobrados; `upright` 0 = coxa
// perpendicular ao tronco (quatro apoios), 1 = tronco ereto com a coxa ~20
// graus para tras. `shift` (m) leva o quadril para o lado (+y) com os joelhos
// parados.
void BodilyGetUp3D::kneelingLegs(const RagdollProfile3D& profile, std::vector<Vec3>& pose,
    float back, float shift) const {
    // Ajoelhado com a canela deitada: a coxa e definida no mundo - `back`
    // (rad) e o quanto o quadril fica atras do joelho - e resolvida contra a
    // pelve desejada; o joelho dobra 90 graus mais esse angulo.
    const float thigh = m_links.thighLength * 0.98f;
    const Vec3 left { -m_body.heading.y, m_body.heading.x, 0.0f };
    for (std::size_t side = 0; side < 2; ++side) {
        const Vec3 direction = Vec3 { 0.0f, 0.0f, -std::cos(back) } + m_body.heading * std::sin(back);
        const Vec3 world = direction * thigh - left * shift;
        Vec3 knee = m_desiredOrientation.conjugate().rotate(world);
        knee.y += lateral(side) * 0.03f;
        setAngle(profile, pose, m_links.thigh[side], 1, -1.0f);
        aimKnee(profile, pose, side, knee);
        // O joelho pela canela medida (o peito do pe ergue o tornozelo e ela
        // nao fica deitada): supor a canela horizontal deixava a coxa tombar
        // para a frente e comia a extensao do quadril.
        const Vec3 shin = (m_body.ankle[side] - m_body.knee[side]).normalized();
        setAngle(profile, pose, m_links.shin[side], 0,
            std::acos(std::clamp(dot(direction.normalized(), shin), -1.0f, 1.0f)));
        setAngle(profile, pose, m_links.foot[side], 0, 1.10f);
    }
}

// Braco de apoio como escora: cotovelo quase reto e so o ombro aponta a mao
// para o ponto (no referencial da pelve medida).
void BodilyGetUp3D::strutArm(const RagdollProfile3D& profile, std::vector<Vec3>& pose,
    std::size_t side, Vec3 targetInRoot) const {
    setAngle(profile, pose, m_links.forearm[side], 0, 0.06f);
    RagdollAnimationPose3D frame;
    buildRagdollLocalPose3D(profile, pose, frame);
    const std::size_t upper = m_links.upperArm[side];
    const std::size_t fore = m_links.forearm[side];
    const Vec3 wrist = frame.linkPositions[fore] + frame.linkOrientations[fore].rotate(m_links.wristInForearm[side]);
    const Vec3 wristInUpper = frame.linkOrientations[upper].conjugate().rotate(wrist - frame.linkPositions[upper]);
    solveRagdollChainPoint3D(profile, pose, upper, wristInUpper, 1, targetInRoot, 1.0f, 18);
}

// Bracos soltos ao lado do corpo, um pouco a frente, cotovelos dobrados.
void BodilyGetUp3D::freeArms(const RagdollProfile3D& profile, std::vector<Vec3>& pose) const {
    for (std::size_t side = 0; side < 2; ++side) {
        setAngle(profile, pose, m_links.upperArm[side], 1, 1.1f);
        setAngle(profile, pose, m_links.forearm[side], 0, 0.45f);
        aimWrist(profile, pose, side, { 0.16f, lateral(side) * 0.20f, -0.48f });
    }
}

// Rolar para ficar de bruços. De costas: dobra a perna de fora com o pe
// plantado e empurra (o joelho cai por cima do corpo e leva o quadril), o
// braco de fora cruza o peito e o de dentro vai para cima da cabeca. De lado:
// o braco e o joelho de cima vao ao chao na frente e completam a virada.
void BodilyGetUp3D::rollPose(const RagdollProfile3D& profile, std::vector<Vec3>& pose,
    float chestProne, float pelvisProne) const {
    const float S = static_cast<float>(m_rollSide);
    const std::size_t near = m_rollSide > 0 ? 0 : 1;
    const std::size_t far = 1 - near;
    neutral(profile, pose);
    legsStraight(profile, pose);
    if (chestProne < 0.55f) {
        for (std::size_t link : { m_links.abdomen, m_links.chest, m_links.upperChest })
            setAngle(profile, pose, link, 0, S * 0.25f);
        setAngle(profile, pose, m_links.neck, 0, S * 0.35f);
        setAngle(profile, pose, m_links.head, 0, S * 0.50f);
        setAngle(profile, pose, m_links.neck, 1, 0.15f);
        setAngle(profile, pose, m_links.forearm[far], 0, 0.5f);
        setAngle(profile, pose, m_links.forearm[near], 0, 0.2f);
        if (chestProne < -0.3f) {
            // De costas: o braco de fora cruza o peito.
            aimWrist(profile, pose, far, { 0.25f, S * 0.45f, -0.05f });
        } else {
            // De lado: o braco de cima vai ao chao na frente do rosto.
            aimWrist(profile, pose, far, { 0.45f, S * 0.12f, 0.10f });
        }
        aimWrist(profile, pose, near, { 0.05f, S * 0.10f, 0.50f });
    } else {
        armsBeside(profile, pose, 0.0f);
    }
    if (pelvisProne < 0.55f) {
        if (pelvisProne < -0.3f) {
            // De costas: perna de fora dobrada, pe plantado, empurrando por
            // cima da outra (aducao e rotacao interna).
            const float push = smoothStep((m_stageSeconds - 0.5f) / 0.5f);
            setAngle(profile, pose, m_links.thigh[far], 1, -1.10f + 0.40f * push);
            setAngle(profile, pose, m_links.thigh[far], 2, -S * 0.55f * push);
            setAngle(profile, pose, m_links.thigh[far], 0, S * 0.35f * push);
            setAngle(profile, pose, m_links.shin[far], 0, 1.90f - 0.40f * push);
            setAngle(profile, pose, m_links.foot[far], 0, 0.0f);
        } else {
            // De lado: o joelho de cima vai ao chao na frente.
            setAngle(profile, pose, m_links.thigh[far], 1, -1.20f);
            setAngle(profile, pose, m_links.thigh[far], 2, -S * 0.25f);
            setAngle(profile, pose, m_links.shin[far], 0, 1.40f);
            setAngle(profile, pose, m_links.thigh[near], 1, -0.20f);
        }
    }
}

void BodilyGetUp3D::blendTo(BodilyGetUpOutput3D& output, const std::vector<Vec3>& pose,
    float duration) {
    const float amount = smoothStep(m_stageSeconds / std::max(duration, 1e-3f));
    for (std::size_t link = 1; link < output.coordinates.size() && link < pose.size(); ++link)
        output.coordinates[link] = lerp(m_stageFrom[link], pose[link], amount);
}

void BodilyGetUp3D::followTo(BodilyGetUpOutput3D& output, const std::vector<Vec3>& pose,
    float timeConstant, float deltaTime) const {
    const float amount = 1.0f - std::exp(-deltaTime / std::max(timeConstant, 1e-3f));
    for (std::size_t link = 1; link < output.coordinates.size() && link < pose.size(); ++link)
        output.coordinates[link] = lerp(output.coordinates[link], pose[link], amount);
}

void BodilyGetUp3D::setGains(const RagdollProfile3D& profile, BodilyGetUpOutput3D& output,
    float stiffness, float damping, float torque) const {
    output.stiffness.assign(profile.links.size(), stiffness);
    output.damping.assign(profile.links.size(), damping);
    output.torque.assign(profile.links.size(), torque);
}

// A pelve do urso pelas maos: ombros exatamente sobre elas (braco vertical:
// a mao so empurra para baixo e nao escorrega), quadril a ~0,78 m, o tronco
// inclinado o que essas duas alturas pedem.
void BodilyGetUp3D::planBearRoot(const RagdollProfile3D& profile) {
    (void)profile;
    m_inchFromRoot = m_body.pelvis;
    m_inchFromOrientation = m_body.pelvisOrientation;
    m_inchSettle = 0.0f;
    const Vec3 hands = (m_handPlant[0] + m_handPlant[1]) * 0.5f;
    const Vec3 shoulderModel = (m_links.shoulderInChest[0] + m_links.shoulderInChest[1]) * 0.5f;
    (void)shoulderModel;
    const Vec3 hipModel = (m_links.hipInPelvis[0] + m_links.hipInPelvis[1]) * 0.5f;
    // Distancia quadril-ombro na pose de pe (coluna reta).
    std::vector<Vec3> straight(profile.links.size(), Vec3 {});
    RagdollAnimationPose3D frame;
    buildRagdollLocalPose3D(profile, straight, frame);
    const Vec3 shoulderRoot = (frame.linkPositions[m_links.upperChest]
        + frame.linkOrientations[m_links.upperChest].rotate(m_links.shoulderInChest[0])
        + frame.linkPositions[m_links.upperChest]
        + frame.linkOrientations[m_links.upperChest].rotate(m_links.shoulderInChest[1])) * 0.5f;
    const float trunk = (shoulderRoot - hipModel).length();
    const float shoulderHeight = m_groundHeight + 0.045f + m_links.armLength * 0.98f;
    const float hipHeight = m_groundHeight + 0.78f;
    const float pitch = std::asin(std::clamp((shoulderHeight - hipHeight) / trunk, -0.9f, 0.9f));
    const Vec3 left { -m_body.heading.y, m_body.heading.x, 0.0f };
    const Vec3 along = m_body.heading * std::cos(pitch) + Vec3 { 0.0f, 0.0f, std::sin(pitch) };
    m_inchOrientation = bodyOrientation(along, left);
    const Vec3 shoulders { hands.x, hands.y, shoulderHeight };
    // O ombro no referencial da pelve (coluna reta): pelve = ombro - Q*ombro.
    m_inchRoot = shoulders - m_inchOrientation.rotate(shoulderRoot);
    m_inchHeight = hipHeight - m_groundHeight;
}

// Do urso ao agachamento: os pes ficam (o tornozelo desce da bola ao pe
// chato), o quadril vai para cima dos calcanhares a ~0,48 m do chao, o tronco
// a ~57 graus. Geometria conferida: joelho a ~130 graus (limite 145), quadril
// abaixo dos 120.
void BodilyGetUp3D::planRockBack(const RagdollProfile3D& profile) {
    (void)profile;
    m_planFrom = m_body.pelvis;
    m_planFromOrientation = m_body.pelvisOrientation;
    m_planProgress = 0.0f;
    m_balanceShift = {};
    m_handPlant = m_body.wrist;
    m_handPlanted = { true, true };
    for (std::size_t side = 0; side < 2; ++side) {
        m_ankleFrom[side] = m_body.ankle[side];
        Vec3 flat = m_body.ball[side] - m_body.heading * 0.17f;
        flat.z = m_groundHeight + 0.135f;
        m_ankleTo[side] = flat;
    }
    const Vec3 ankles = (m_ankleTo[0] + m_ankleTo[1]) * 0.5f;
    const Vec3 left { -m_body.heading.y, m_body.heading.x, 0.0f };
    constexpr float Pitch = 1.0f;
    m_planToOrientation = bodyOrientation(m_body.heading * std::cos(Pitch) + Vec3 { 0.0f, 0.0f, std::sin(Pitch) }, left);
    const Vec3 hipCenter = Vec3 { ankles.x, ankles.y, m_groundHeight + 0.48f } - m_body.heading * 0.08f;
    const Vec3 hipModel = (m_links.hipInPelvis[0] + m_links.hipInPelvis[1]) * 0.5f;
    m_planTo = hipCenter - m_planToOrientation.rotate(hipModel);
    if (std::getenv("XBODDBG")) {
        const auto v = [](const char* n, Vec3 p) { std::printf("  %s (%.2f %.2f %.2f)\n", n, p.x, p.y, p.z); };
        std::printf("PLANO URSO->AGACHA heading (%.2f %.2f)\n", m_body.heading.x, m_body.heading.y);
        v("pelve", m_body.pelvis); v("com", m_body.centerOfMass); v("bolaE", m_body.ball[0]); v("bolaD", m_body.ball[1]);
        v("tornE", m_body.ankle[0]); v("alvo tornE", m_ankleTo[0]); v("punhoE", m_body.wrist[0]); v("pelve alvo", m_planTo);
    }
}

// Do agachamento a de pe: pelve sobre os pes, na altura de pe, ereta.
void BodilyGetUp3D::planStand(const RagdollProfile3D& profile) {
    m_planFrom = m_body.pelvis;
    m_planFromOrientation = m_body.pelvisOrientation;
    m_planProgress = 0.0f;
    const Vec3 ankles = (m_ankleTo[0] + m_ankleTo[1]) * 0.5f;
    const Vec3 left { -m_body.heading.y, m_body.heading.x, 0.0f };
    m_planToOrientation = bodyOrientation({ 0.0f, 0.0f, 1.0f }, left);
    m_planTo = Vec3 { ankles.x, ankles.y, m_groundHeight + profile.standingRootHeightMeters * 0.98f }
        + m_body.heading * 0.04f;
}

bool BodilyGetUp3D::update(const RagdollProfile3D& profile, const RagdollState3D& state,
    float groundHeight, float deltaTime, BodilyGetUpOutput3D& output) {
    if (m_failed || state.links.size() != profile.links.size()) {
        m_failed = true;
        return false;
    }
    measure(profile, state, groundHeight);
    m_stageSeconds += deltaTime;
    output.coordinates = m_previousCoordinates;
    setGains(profile, output, 1.1f, 1.2f, 1.0f);
    m_desiredRoot = m_body.pelvis;
    m_desiredOrientation = m_body.pelvisOrientation;
    std::vector<Vec3> pose;
    const float chestProne = -m_body.anterior.z;
    const float pelvisProne = -m_body.pelvisAnterior.z;
    switch (m_stage) {
    case BodilyGetUpStage3D::Roll: {
        const float prone = std::min(chestProne, pelvisProne);
        m_stageProgress = std::clamp(0.5f + 0.5f * prone, 0.0f, 1.0f);
        rollPose(profile, pose, chestProne, pelvisProne);
        followTo(output, pose, 0.18f, deltaTime);
        if (prone > 0.55f && m_body.coreSpeed < 0.6f) {
            enter(BodilyGetUpStage3D::PlaceHands);
            break;
        }
        // Sem progresso, tenta o outro lado.
        if (prone > m_bestProgress + 0.02f) {
            m_bestProgress = prone;
            m_stalledSeconds = 0.0f;
        } else {
            m_stalledSeconds += deltaTime;
        }
        if (m_stalledSeconds > 2.0f) {
            if (++m_rollFlips > 3) {
                m_failed = true;
                break;
            }
            m_rollSide = -m_rollSide;
            m_stalledSeconds = 0.0f;
            m_stageSeconds = 0.0f;
            m_bestProgress = prone;
        }
        break;
    }
    case BodilyGetUpStage3D::PlaceHands: {
        neutral(profile, pose);
        legsStraight(profile, pose);
        for (std::size_t link : { m_links.abdomen, m_links.chest, m_links.upperChest })
            setAngle(profile, pose, link, 1, -0.05f);
        setAngle(profile, pose, m_links.neck, 1, -0.25f);
        setAngle(profile, pose, m_links.head, 1, -0.15f);
        armsBeside(profile, pose, 0.0f);
        constexpr float Duration = 0.8f;
        blendTo(output, pose, Duration);
        m_stageProgress = std::clamp(m_stageSeconds / Duration, 0.0f, 1.0f);
        if (m_stageSeconds > Duration + 0.1f) enter(BodilyGetUpStage3D::PushUp);
        break;
    }
    case BodilyGetUpStage3D::PushUp: {
        neutral(profile, pose);
        legsStraight(profile, pose);
        setAngle(profile, pose, m_links.neck, 1, -0.20f);
        setAngle(profile, pose, m_links.head, 1, -0.10f);
        armsBeside(profile, pose, 1.0f);
        constexpr float Duration = 1.0f;
        blendTo(output, pose, Duration);
        m_stageProgress = std::clamp(m_stageSeconds / Duration, 0.0f, 1.0f);
        if (m_stageSeconds > Duration + 0.1f && m_body.chestHeight > 0.35f) {
            enter(BodilyGetUpStage3D::Quadruped);
        } else if (m_stageSeconds > 3.5f) {
            m_failed = true;
        }
        break;
    }
    case BodilyGetUpStage3D::Quadruped:
    case BodilyGetUpStage3D::TuckToes: {
        // Quatro apoios; depois os dedos dos pes dobram (os pes vao empurrar).
        const bool tuck = m_stage == BodilyGetUpStage3D::TuckToes;
        neutral(profile, pose);
        for (std::size_t side = 0; side < 2; ++side) {
            setAngle(profile, pose, m_links.thigh[side], 1, -1.40f);
            aimKnee(profile, pose, side, { m_links.thighLength * 0.98f, lateral(side) * 0.03f, 0.0f });
            setAngle(profile, pose, m_links.shin[side], 0, 1.57f);
            // Peito do pe no chao (pe esticado ao maximo): a canela deita e o
            // joelho apoia. Com os dedos dobrados o pe (13 cm) erguia a
            // canela e ele ficava nas pontas dos pes, sem joelho no chao.
            setAngle(profile, pose, m_links.foot[side], 0, tuck ? 0.0f : 0.90f);
        }
        setAngle(profile, pose, m_links.neck, 1, -0.15f);
        armsBeside(profile, pose, 1.0f);
        const float duration = tuck ? 0.5f : 1.2f;
        blendTo(output, pose, duration);
        m_stageProgress = std::clamp(m_stageSeconds / duration, 0.0f, 1.0f);
        if (!tuck) {
            if (std::getenv("XBODDBG") && m_stageSeconds > duration
                && static_cast<int>(m_stageSeconds * 120.0f) % 60 == 0) {
                std::printf("QUAD pelve %.2f joelhos %d %d maos %d %d pes %d %d\n", m_body.pelvisHeight,
                    m_body.kneeContact[0], m_body.kneeContact[1], m_body.handContact[0], m_body.handContact[1],
                    m_body.footContact[0], m_body.footContact[1]);
            }
            // Quatro apoios: maos e joelhos - ou os peitos dos pes, com os
            // joelhos logo acima do chao.
            const bool legsDown = (m_body.kneeContact[0] || m_body.footContact[0])
                && (m_body.kneeContact[1] || m_body.footContact[1]);
            if (m_stageSeconds > duration && m_body.pelvisHeight > 0.38f && legsDown
                && (m_body.handContact[0] || m_body.handContact[1])) {
                enter(BodilyGetUpStage3D::TuckToes);
            } else if (m_stageSeconds > 4.0f) {
                m_failed = true;
            }
        } else if (m_stageSeconds > duration + 0.1f) {
            enter(BodilyGetUpStage3D::Bear);
        }
        break;
    }
    case BodilyGetUpStage3D::Bear: {
        // Urso: maos e bolas dos pes, joelhos quase retos, quadril alto; o
        // tronco inclina ~25 graus para baixo (geometria: coxa a ~77 graus do
        // tronco, braco a ~65 graus da linha da cabeca, pes onde estavam os
        // dedos).
        neutral(profile, pose);
        setAngle(profile, pose, m_links.neck, 1, -0.30f);
        for (std::size_t side = 0; side < 2; ++side) {
            const float s2 = lateral(side);
            setAngle(profile, pose, m_links.forearm[side], 0, 0.05f);
            setAngle(profile, pose, m_links.upperArm[side], 2, -s2 * 1.2f);
            const float arm = m_links.armLength * 0.97f;
            aimWrist(profile, pose, side, { arm * 0.906f, s2 * 0.07f, arm * 0.423f });
            setAngle(profile, pose, m_links.thigh[side], 1, -1.35f);
            setAngle(profile, pose, m_links.shin[side], 0, 0.35f);
            setAngle(profile, pose, m_links.foot[side], 0, -0.20f);
        }
        constexpr float Duration = 1.2f;
        blendTo(output, pose, Duration);
        m_stageProgress = std::clamp(m_stageSeconds / Duration, 0.0f, 1.0f);
        if (m_stageSeconds > Duration + 0.1f && m_body.footContact[0] && m_body.footContact[1]
            && (m_body.handContact[0] || m_body.handContact[1])) {
            m_handPlant = m_body.wrist;
            m_handPlanted = { true, true };
            for (std::size_t side = 0; side < 2; ++side) {
                m_footPlant[side] = m_body.ball[side];
                m_footPlanted[side] = true;
                m_footFlat[side] = false;
            }
            m_inchSteps = 0;
            m_inchPhase = 0;
            planBearRoot(profile);
            m_inchOffset = {};
            enter(BodilyGetUpStage3D::Inchworm);
        } else if (m_stageSeconds > Duration + 2.5f) {
            m_failed = true;
        }
        break;
    }
    case BodilyGetUpStage3D::Inchworm: {
        // Pes em direcao as maos, um de cada vez, ~24 cm: o peso vai para o
        // triangulo das maos e do pe que fica (realimentado: a pelve desejada e
        // a do comeco do passo mais uma correcao integral pelo centro de
        // massa), o pe sai, anda e pousa na bola. Quando maos e pes estao a
        // ~0,8 m, balanca para tras ate agachar.
        const std::size_t k = m_stepSide;
        const std::size_t o = 1 - k;
        const Vec3 centroid = (m_handPlant[0] + m_handPlant[1] + m_footPlant[o]) * (1.0f / 3.0f);
        // So no lado e contido (ate 6 cm): a frente e atras quem decide e a
        // geometria do urso; deslocar a pelve mais que isso tirava uma mao do
        // alcance e ele desabava.
        const Vec3 side { -m_body.heading.y, m_body.heading.x, 0.0f };
        const float error = dot(centroid - m_body.centerOfMass, side);
        m_inchOffset += side * (error * 1.5f * deltaTime);
        {
            const float lateralOffset = std::clamp(dot(m_inchOffset, side), -0.06f, 0.06f);
            m_inchOffset = side * lateralOffset;
        }
        // Do corpo medido ate a pelve do urso, em 0,6 s no comeco.
        const float settle = smoothStep(m_inchSettle / 0.6f);
        m_inchSettle += deltaTime;
        m_desiredRoot = lerp(m_inchFromRoot, m_inchRoot, settle) + m_inchOffset
            + side * std::clamp(error * 0.5f, -0.02f, 0.02f);
        m_desiredOrientation = nlerp(m_inchFromOrientation, m_inchOrientation, settle);
        // Margem do centro de massa no triangulo.
        const auto inside = [&](Vec3 p) {
            const Vec3 a = m_handPlant[0], b = m_handPlant[1], c = m_footPlant[o];
            const auto edge = [&](Vec3 u, Vec3 v, Vec3 w) {
                const Vec3 d = { v.x - u.x, v.y - u.y, 0.0f };
                const float len = std::max(1e-4f, std::hypot(d.x, d.y));
                const float side = (d.x * (p.y - u.y) - d.y * (p.x - u.x)) / len;
                const float ref = d.x * (w.y - u.y) - d.y * (w.x - u.x);
                return ref >= 0.0f ? side : -side;
            };
            return std::min({ edge(a, b, c), edge(b, c, a), edge(c, a, b) });
        };
        const float margin = inside(m_body.centerOfMass);
        m_telemetry.comOutside = -margin;
        neutral(profile, pose);
        setAngle(profile, pose, m_links.neck, 1, -0.30f);
        for (std::size_t side = 0; side < 2; ++side) {
            setAngle(profile, pose, m_links.upperArm[side], 2, -lateral(side) * 1.2f);
            strutArm(profile, pose, side, toDesired(m_handPlant[side]));
            setAngle(profile, pose, m_links.thigh[side], 1, -1.35f);
            setAngle(profile, pose, m_links.shin[side], 0, 0.6f);
            setAngle(profile, pose, m_links.foot[side], 0, -0.20f);
        }
        solveRagdollChainPoint3D(profile, pose, m_links.foot[o], m_links.ballInFoot[o], 3,
            toDesired(m_footPlant[o]), 1.0f, 18, &m_legWeightsAnkleLocked);
        constexpr float SwingTime = 0.65f;
        float swing = 0.0f;
        if (m_inchPhase == 0) {
            solveRagdollChainPoint3D(profile, pose, m_links.foot[k], m_links.ballInFoot[k], 3,
                toDesired(m_footPlant[k]), 1.0f, 18, &m_legWeightsAnkleLocked);
            if (m_stageSeconds > 0.3f && margin > 0.03f) {
                m_stepFrom = m_footPlant[k];
                // Ate ~0,62 m das maos (mais perto a coxa passaria dos 120
                // graus de flexao e o pe nao alcancaria o chao).
                const Vec3 hands = (m_handPlant[0] + m_handPlant[1]) * 0.5f;
                const float distance = dot(hands - m_footPlant[k], m_body.heading);
                const float length = std::clamp(distance - 0.62f, 0.0f, 0.24f);
                Vec3 to = m_footPlant[k] + m_body.heading * length;
                to.z = m_footPlant[k].z;
                m_stepTo = to;
                m_inchPhase = 1;
                m_stepLiftAt = m_stageSeconds;
            } else if (m_stageSeconds > 2.5f) {
                m_failed = true;
            }
        } else {
            swing = smoothStep((m_stageSeconds - m_stepLiftAt) / SwingTime);
            // A perna em balanco parte da solucao anterior (de uma pose fixa o
            // IK nao convergia quando o alvo pedia pouca mudanca).
            for (std::size_t link : { m_links.thigh[k], m_links.shin[k] })
                pose[link] = m_previousCoordinates[link];
            // Terminado o balanco sem tocar, o alvo desce aos poucos (busca do
            // chao) em vez de pairar.
            const float search = std::clamp((m_stageSeconds - m_stepLiftAt - SwingTime) * 0.12f, 0.0f, 0.10f);
            const Vec3 target = lerp(m_stepFrom, m_stepTo, swing)
                + Vec3 { 0.0f, 0.0f, 0.10f * std::sin(3.14159265f * swing) - search };
            solveRagdollChainPoint3D(profile, pose, m_links.foot[k], m_links.ballInFoot[k], 3,
                toDesired(target), 1.0f, 18, &m_legWeightsAnkleLocked);
            if (swing >= 1.0f && m_body.footContact[k]) {
                if (std::getenv("XBODDBG")) {
                    std::printf("POUSO passo %d pe %zu: alvo avancou %.3f, bola avancou %.3f (alvo-bola %.3f), outro pe escorregou %.3f, maos escorregaram %.3f %.3f\n",
                        m_inchSteps, k, dot(m_stepTo - m_stepFrom, m_body.heading), dot(m_body.ball[k] - m_stepFrom, m_body.heading),
                        (m_body.ball[k] - m_stepTo).length(), (m_body.ball[o] - m_footPlant[o]).length(),
                        (m_body.wrist[0] - m_handPlant[0]).length(), (m_body.wrist[1] - m_handPlant[1]).length());
                }
                m_footPlant[k] = m_body.ball[k];
                ++m_inchSteps;
                const Vec3 hands = (m_handPlant[0] + m_handPlant[1]) * 0.5f;
                const Vec3 feet = (m_footPlant[0] + m_footPlant[1]) * 0.5f;
                // Os dois pes a ~0,6 m das maos (quadril sobre eles): balanca.
                const float near0 = dot(hands - m_footPlant[0], m_body.heading);
                const float near1 = dot(hands - m_footPlant[1], m_body.heading);
                (void)feet;
                if ((near0 < 0.68f && near1 < 0.68f) || m_inchSteps >= 10) {
                    planRockBack(profile);
                    enter(BodilyGetUpStage3D::RockBack);
                    break;
                }
                m_stepSide = o;
                m_inchPhase = 0;
                // A pelve do urso (calculada pelas maos) fica a mesma: os pes e
                // que vem ate ela.
                m_inchOffset = {};
                m_stageSeconds = 0.0f;
            } else if (m_stageSeconds - m_stepLiftAt > SwingTime + 1.5f) {
                m_failed = true;
            }
        }
        followTo(output, pose, 0.08f, deltaTime);
        m_stageProgress = std::clamp(static_cast<float>(m_inchSteps) / 4.0f, 0.0f, 1.0f);
        if (std::getenv("XBODDBG") && static_cast<int>(m_stageSeconds * 120.0f) % 12 == 0) {
            const Vec3 hands = (m_handPlant[0] + m_handPlant[1]) * 0.5f;
            const Vec3 feet = (m_footPlant[0] + m_footPlant[1]) * 0.5f;
            std::printf("PASSINHO %.2f passo %d pe %zu fase %d giro %.2f | margem %+.3f desvio %.3f | maos-pes %.2f | maos %d %d pes %d %d | pelve z %.2f\n",
                m_stageSeconds, m_inchSteps, k, m_inchPhase, swing, margin, m_inchOffset.length(),
                dot(hands - feet, m_body.heading), m_body.handContact[0], m_body.handContact[1],
                m_body.footContact[0], m_body.footContact[1], m_body.pelvisHeight);
        }
        break;
    }
    case BodilyGetUpStage3D::RockBack:
    case BodilyGetUpStage3D::Stand: {
        // Pelve numa trajetoria absoluta: do urso ao agachamento (quadril sobre
        // os calcanhares, tronco a ~57 graus) e dele a de pe. Pes e maos
        // apoiados resolvidos contra a pelve planejada; no agachamento as maos
        // soltam e os bracos vao a frente (contrapeso); de pe, o centro de
        // massa e levado para cima dos pes.
        const bool standing = m_stage == BodilyGetUpStage3D::Stand;
        const float duration = standing ? 1.6f : 1.8f;
        // O plano espera o corpo: a pelve medida longe da planejada, anda
        // devagar.
        const float planned = smoothStep(m_planProgress);
        const Vec3 plannedPelvis = lerp(m_planFrom, m_planTo, planned);
        const float lag = (plannedPelvis - m_body.pelvis).length();
        // (Com as maos no chao o plano nao espera: maos presas seguravam o
        // corpo, o plano esperava o corpo e as maos esperavam o plano.)
        const bool handsDown = !standing && (m_handPlanted[0] || m_handPlanted[1]);
        const float rate = handsDown ? 1.0f : std::clamp(1.0f - (lag - 0.06f) / 0.10f, 0.2f, 1.0f);
        m_planProgress = std::min(1.0f, m_planProgress + deltaTime / duration * rate);
        const float s = smoothStep(m_planProgress);
        // Equilibrio: o centro de massa sobre o meio dos pes (entre tornozelo
        // e bola), pela posicao planejada da pelve (PI, ate 6 cm).
        const Vec3 feetCenter = (m_ankleTo[0] + m_ankleTo[1]) * 0.5f + m_body.heading * 0.06f;
        if (standing || s > 0.6f) {
            Vec3 error = feetCenter - m_body.centerOfMass;
            error.z = 0.0f;
            m_balanceShift += error * (1.5f * deltaTime);
            const float length = m_balanceShift.length();
            if (length > 0.06f) m_balanceShift *= 0.06f / length;
        }
        m_desiredRoot = lerp(m_planFrom, m_planTo, s) + m_balanceShift;
        m_desiredOrientation = nlerp(m_planFromOrientation, m_planToOrientation, s);
        m_telemetry.rootError = lag;
        // Continuidade: o IK parte da solucao do tick anterior (de uma pose de
        // referencia fixa a solucao pulava de configuracao e dava um tranco).
        pose = m_previousCoordinates;
        for (std::size_t link : { m_links.abdomen, m_links.chest, m_links.upperChest })
            setAngle(profile, pose, link, 1, 0.0f);
        setAngle(profile, pose, m_links.neck, 1, standing ? 0.0f : -0.20f);
        setAngle(profile, pose, m_links.head, 1, 0.0f);
        for (std::size_t side = 0; side < 2; ++side) {
            // Pes: primeiro na bola (calcanhar no ar), depois o pe chato com o
            // tornozelo descendo ate o lugar.
            if (!standing && s < 0.45f) {
                solveRagdollChainPoint3D(profile, pose, m_links.foot[side], m_links.ballInFoot[side], 3,
                    toDesired(m_footPlant[side]), 1.0f, 18, &m_legWeights);
            } else {
                const float flat = standing ? 1.0f : smoothStep((s - 0.45f) / 0.4f);
                const Vec3 ankle = lerp(m_ankleFrom[side], m_ankleTo[side], flat);
                solveRagdollChainPoint3D(profile, pose, m_links.shin[side], m_links.ankleInShin[side], 2,
                    toDesired(ankle), 1.0f, 18, &m_legWeights);
                flattenFoot(profile, pose, side);
            }
            // Maos: no chao enquanto o ombro planejado alcanca o ponto delas;
            // depois a frente (presas fora de alcance, seguravam o corpo).
            if (!standing && m_handPlanted[side]) {
                const Vec3 shoulder = m_desiredRoot + m_desiredOrientation.rotate(shoulderInRoot(profile, pose, side));
                if ((m_handPlant[side] - shoulder).length() > m_links.armLength * 0.98f || s > 0.25f)
                    m_handPlanted[side] = false;
            }
            if (!standing && m_handPlanted[side]) {
                strutArm(profile, pose, side, toDesired(m_handPlant[side]));
            } else if (!standing) {
                // Bracos a frente na horizontal do mundo (contrapeso): no
                // referencial do tronco inclinado, "a frente" apontava o chao
                // e as maos seguiam empurrando.
                setAngle(profile, pose, m_links.forearm[side], 0, 0.30f);
                const Vec3 reach = m_desiredOrientation.conjugate().rotate(
                    m_body.heading * 0.45f + Vec3 { 0.0f, 0.0f, 0.05f });
                aimWrist(profile, pose, side, reach + m_desiredOrientation.conjugate().rotate(
                    Vec3 { -m_body.heading.y, m_body.heading.x, 0.0f } * (lateral(side) * 0.16f)));
            } else {
                setAngle(profile, pose, m_links.upperArm[side], 1, 1.15f);
                setAngle(profile, pose, m_links.forearm[side], 0, 0.35f + 0.4f * (1.0f - s));
            }
        }
        followTo(output, pose, 0.08f, deltaTime);
        m_stageProgress = m_planProgress;
        const float upright = m_body.pelvisOrientation.rotate({ 0.0f, 0.0f, 1.0f }).z;
        const float comAhead = dot(m_body.centerOfMass - feetCenter, m_body.heading);
        if (std::getenv("XBODDBG") && static_cast<int>(m_stageSeconds * 120.0f) % 12 == 0) {
            std::printf("%s %.2f plano %.2f atraso %.3f | com-pes %+.3f lado %+.3f | pes %d %d maos %d %d | pelve z %.2f pitch %.2f ereto %.2f\n",
                standing ? "PE" : "BALANCA", m_stageSeconds, m_planProgress, lag, comAhead,
                dot(m_body.centerOfMass - feetCenter, Vec3 { -m_body.heading.y, m_body.heading.x, 0.0f }),
                m_body.footContact[0], m_body.footContact[1], m_body.handContact[0], m_body.handContact[1],
                m_body.pelvisHeight, m_body.trunkPitch, upright);
        }
        if (!standing) {
            if (m_planProgress >= 1.0f && lag < 0.10f && m_body.footContact[0] && m_body.footContact[1]
                && std::abs(comAhead) < 0.10f) {
                planStand(profile);
                enter(BodilyGetUpStage3D::Stand);
            } else if (m_stageSeconds > duration + 3.0f) {
                m_failed = true;
            }
        } else if (m_planProgress >= 1.0f && upright > 0.93f && std::abs(comAhead) < 0.08f
            && Vec3 { m_body.centerOfMassVelocity.x, m_body.centerOfMassVelocity.y, 0.0f }.length() < 0.3f) {
            m_stage = BodilyGetUpStage3D::Done;
        } else if (m_stageSeconds > duration + 3.0f) {
            m_failed = true;
        }
        break;
    }
    default:
        break;
    }
    // Dentro dos limites de cada junta.
    for (std::size_t link = 1; link < output.coordinates.size(); ++link) {
        for (std::size_t axis = 0; axis < 3; ++axis) {
            const RagdollAxisDefinition3D& limit = profile.links[link].inboundJoint.axes[axis];
            setComponent(output.coordinates[link], axis, limit.enabled
                ? std::clamp(component(output.coordinates[link], axis), limit.minimumRadians, limit.maximumRadians)
                : 0.0f);
        }
    }
    m_previousCoordinates = output.coordinates;
    m_telemetry.stage = m_stage;
    m_telemetry.stageSeconds = m_stageSeconds;
    m_telemetry.stageProgress = m_stageProgress;
    m_telemetry.pelvisHeight = m_body.pelvisHeight;
    m_telemetry.rollSide = m_rollSide;
    return m_stage == BodilyGetUpStage3D::Done;
}

} // namespace MatterEngine
