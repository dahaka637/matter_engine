#include "Engine/Character/ProtectiveFall3D.hpp"

// (A queda programada e a da locomocao fisica: as funcoes e as constantes dela, do cabecalho interno.)
#include "Engine/Character/CharacterLocomotion3DInternal.hpp"

namespace MatterEngine {
namespace {

// No chao com o tronco parado por isto: a vez de levantar. A locomocao fisica se arruma depois de SettleRestBeforeGather
// (0,2 s) e leva ~0,7 s voltando o tonus; aqui meio segundo largado.
constexpr float RestSeconds = 0.5f;
// No chao ha isto sem parar (escorregando, rolando devagar): assentado mesmo assim.
constexpr float GroundLimitSeconds = 3.0f;

} // namespace

ProtectiveFall3D::ProtectiveFall3D(const RagdollProfile3D& profile) : m_profile(&profile) {
    m_hands = { findLink(profile, "LeftHand"), findLink(profile, "RightHand") };
    m_chest = findLink(profile, "UpperChest");
    const std::size_t n = profile.links.size();
    m_kind.assign(n, Kind::Trunk);
    m_arm.assign(n, 0);
    m_lyingTorso.assign(n, 0);
    m_lyingArm.assign(n, 0);
    m_body.assign(n, 0);
    for (std::size_t i = 0; i < n; ++i) {
        const std::string& id = profile.links[i].id;
        const auto has = [&](const char* part) { return id.find(part) != std::string::npos; };
        const bool arm = has("UpperArm"), forearm = has("Forearm"), hand = has("Hand");
        const bool leg = has("Thigh") || has("Shin") || has("Foot");
        const bool neck = id == "Neck", head = id == "Head";
        // (Como o tonus da locomocao fisica: os reflexos nos bracos, no pescoco e na cabeca; as pernas; o resto, tronco.)
        m_kind[i] = arm || forearm || neck || head ? Kind::ReflexLimb : leg ? Kind::Leg : Kind::Trunk;
        m_arm[i] = arm || forearm;
        m_lyingArm[i] = arm || forearm || hand;
        m_lyingTorso[i] = id == "Abdomen" || id == "Chest" || id == "UpperChest" || neck || head;
        // O tronco ou a cabeca no chao: a queda chegou ao chao.
        m_body[i] = !arm && !forearm && !hand && !leg;
    }
}

void ProtectiveFall3D::begin(std::uint32_t seed) {
    m_seed = seed;
    m_time = 0.0f;
    m_reflex = 0.0f;
    m_commit = 0.0f;
    m_flail = 0.0f;
    m_fallDirection = { 1.0f, 0.0f, 0.0f };
    m_catchSeconds = {};
    m_noGroundSeconds = 0.0f;
    m_grounded = false;
    m_groundSeconds = 0.0f;
    m_restSeconds = 0.0f;
    m_settleTone = SettleImpactTone;
    m_settled = false;
}

bool ProtectiveFall3D::committed(float tilt, float tiltRate) {
    const float commit = std::max(smoothStep((tilt - 0.87f) / 0.22f) * smoothStep((tiltRate + 0.3f) / 0.6f),
        smoothStep((tilt - 0.70f) / 0.26f) * smoothStep((tiltRate - 2.5f) / 1.0f));
    return commit >= 0.5f;
}

float ProtectiveFall3D::pelvisTilt(const RagdollProfile3D& profile, const RagdollState3D& state) {
    if (state.links.empty() || profile.links.empty()) return 0.0f;
    const Quaternion r = (state.links.front().orientation * profile.links.front().modelOrientation.conjugate()).normalized();
    return std::acos(std::clamp(r.rotate({ 0.0f, 0.0f, 1.0f }).z, -1.0f, 1.0f));
}

void ProtectiveFall3D::step(const RagdollState3D& state, float groundHeight, float deltaTime,
    std::vector<RagdollDriveTarget3D>& drives) {
    const RagdollProfile3D& profile = *m_profile;
    drives.clear();
    const std::size_t n = profile.links.size();
    if (state.links.size() != n || state.joints.size() != n || m_chest >= n) return;
    m_time += deltaTime;
    // Os contatos: qualquer um (no ar ha quanto tempo), as maos e o tronco ou a cabeca no chao.
    bool any = false, body = false;
    std::array<bool, 2> hand {};
    for (const RagdollInteraction3D& contact : state.interactions) {
        if (contact.linkIndex >= n) continue;
        any = true;
        body = body || (m_body[contact.linkIndex] != 0 && contact.normalWorld.z > 0.5f);
        for (std::size_t side = 0; side < 2; ++side) hand[side] = hand[side] || contact.linkIndex == m_hands[side];
    }
    m_noGroundSeconds = any ? 0.0f : m_noGroundSeconds + deltaTime;
    for (std::size_t side = 0; side < 2; ++side) m_catchSeconds[side] = hand[side] ? m_catchSeconds[side] + deltaTime : 0.0f;
    const PhysicsBodyState3D& pelvis = state.links.front();
    const PhysicsBodyState3D& chest = state.links[m_chest];
    // O rumo da queda: para onde o tronco vai; devagar, para onde ele tomba.
    Vec3 fall = (pelvis.linearVelocity + chest.linearVelocity) * 0.5f;
    fall.z = 0.0f;
    if (fall.lengthSquared() < 0.16f) {
        fall = pelvis.orientation.rotate({ 0.0f, 0.0f, 1.0f });
        fall.z = 0.0f;
    }
    if (fall.lengthSquared() > 0.0004f) {
        m_fallDirection = m_fallDirection + (fall.normalized() - m_fallDirection) * std::clamp(deltaTime * 12.0f, 0.0f, 1.0f);
        if (m_fallDirection.lengthSquared() > 1e-6f) m_fallDirection = m_fallDirection.normalized();
    }
    const float coreSpeed = std::max(pelvis.linearVelocity.length(), chest.linearVelocity.length());
    if (!m_grounded && (body || pelvis.position.z - groundHeight < 0.35f)) {
        m_grounded = true;
        m_groundSeconds = 0.0f;
        m_settleTone = SettleImpactTone;
    }
    if (m_grounded) {
        // Deitado (o assentar da locomocao fisica): o tronco parado e o tonus cedendo.
        m_groundSeconds += deltaTime;
        const float restLimit = m_restSeconds > 0.25f ? 0.80f : 0.35f;
        m_restSeconds = coreSpeed < restLimit ? m_restSeconds + deltaTime : 0.0f;
        m_settleTone = std::max(SettleRestingTone, m_settleTone - deltaTime * 2.5f);
        m_settled = m_restSeconds >= RestSeconds || m_groundSeconds >= GroundLimitSeconds;
    } else {
        // Os reflexos e as pernas cedendo: a queda ja e sem volta quando isto comeca.
        m_reflex += (1.0f - m_reflex) * std::clamp(deltaTime * 18.0f, 0.0f, 1.0f);
        m_commit += (1.0f - m_commit) * std::clamp(deltaTime * 18.0f, 0.0f, 1.0f);
    }
    // A pose: a de agora do corpo (o que os reflexos nao tocam fica largado), os reflexos (no chao saem em 0,3 s) e o
    // equilibrio no ar.
    float reflex = m_reflex, legReflex = std::min(m_reflex, m_commit);
    if (m_grounded) {
        reflex *= GroundAlive ? 1.0f - smoothStep(m_groundSeconds / 0.3f) : 1.0f - smoothStep(m_groundSeconds / 0.6f);
        legReflex = reflex;
    }
    m_coordinates.resize(n);
    for (std::size_t i = 0; i < n; ++i)
        m_coordinates[i] = { state.joints[i].positionRadians[0], state.joints[i].positionRadians[1],
            state.joints[i].positionRadians[2] };
    applyFallReflexPose(profile, state, m_coordinates, m_hands, m_chest, m_fallDirection, groundHeight, reflex, legReflex,
        m_catchSeconds, m_time, m_seed);
    m_flail = !m_grounded ? m_reflex * smoothStep((m_noGroundSeconds - 0.10f) / 0.20f) : 0.0f;
    const float bodySpin = dot(chest.angularVelocity, chest.orientation.rotate({ 0.0f, 1.0f, 0.0f }));
    if (AirBalanceNew) applyAirBalance(profile, m_coordinates, m_time, m_flail, bodySpin);
    else applyAirborneFlail(profile, m_coordinates, m_time, m_flail);
    // (02/11 00h10, ele: "no levantar-se, eu nao quero esses movimentos recontorcidos ai" - sem a posicao fetal: no chao,
    // largado, e dai a rede levanta.)
    // O tonus (o de CharacterLocomotion3D_Drives): caindo, firme o bastante para os reflexos e sem a rigidez de pe, freio
    // baixo; deitado, largado - quase sem mola nem freio, o teto de torque baixo.
    for (std::size_t i = 0; i < n; ++i) {
        if (profile.links[i].parentIndex < 0) continue;
        float stiffness = 1.0f, damping = 1.0f, torque = 1.0f;
        // (Largado, os membros pesam: a compensacao da gravidade cai - caindo, menos nos reflexos; deitado, quase toda.)
        const bool gravityLimb = m_lyingArm[i] != 0 || profile.links[i].id == "Neck" || profile.links[i].id == "Head";
        const float gravity = m_grounded ? LyingGravity
            : gravityLimb ? 1.0f - (1.0f - FallLimbGravity) * m_reflex : 1.0f - (1.0f - FallBodyGravity) * m_commit;
        if (!m_grounded) {
            const bool limb = m_kind[i] == Kind::ReflexLimb;
            const bool left = profile.links[i].id.rfind("Left", 0) == 0;
            const bool catching = FallProtectNew && m_arm[i] != 0 && m_catchSeconds[left ? 0 : 1] > 0.0f;
            const float fallTone = !FallProtectNew ? (limb ? FallReflexLimbTone : FallBodyTone)
                : catching ? FallCatchTone : limb ? FallReflexLimbTone
                : m_kind[i] == Kind::Leg ? (FallLegsNew ? FallLegTone : FallBodyTone) : FallTrunkTone;
            const float amount = limb ? m_reflex : m_commit;
            float tone = UprightMuscleTone + (fallTone - UprightMuscleTone) * amount;
            if ((m_arm[i] != 0 || m_kind[i] == Kind::Leg) && m_flail > 0.0f)
                tone += (std::max(tone, FallAirTone) - tone) * m_flail;
            stiffness = tone;
            torque = tone;
            damping = std::sqrt(tone);
            damping += ((limb ? 0.55f : 0.15f) - damping) * amount;
        } else {
            const float lyingStiffness = (m_lyingTorso[i] != 0 ? 0.70f : m_lyingArm[i] != 0 ? 0.50f : 0.55f) * m_settleTone;
            const float lyingTorque =
                (m_lyingTorso[i] != 0 ? 0.90f : m_lyingArm[i] != 0 ? 0.65f : 0.75f) * std::max(0.5f, m_settleTone);
            stiffness = lyingStiffness * 0.25f;
            damping = 0.03f;
            torque = lyingTorque * 0.40f;
        }
        for (std::uint8_t axis = 0; axis < 3; ++axis) {
            if (!profile.links[i].inboundJoint.axes[axis].enabled) continue;
            RagdollDriveTarget3D drive;
            drive.linkIndex = static_cast<std::uint32_t>(i);
            drive.axis = static_cast<RagdollAxis3D>(axis);
            drive.positionRadians = component(m_coordinates[i], axis);
            drive.stiffnessScale = stiffness;
            drive.dampingScale = damping;
            drive.maximumTorqueScale = torque;
            drive.gravityCompensationScale = gravity;
            drives.push_back(drive);
        }
    }
}

} // namespace MatterEngine
