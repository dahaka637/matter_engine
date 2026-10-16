#include "Engine/Character/CharacterLocomotion3DInternal.hpp"

namespace MatterEngine {

void CharacterLocomotion3D::stepSense(Tick& t) {
    const RagdollProfile3D& profile = t.profile;
    const RagdollState3D& state = t.state;
    const RagdollDynamics3D& dynamics = t.dynamics;
    const CharacterLocomotionInput3D& input = t.input;
    float& deltaTime = t.deltaTime;
    const CharacterPhysicalState3D& physical = *t.physical;
    // Centro de massa pela massa de cada link (do estimador comum), para
    // todos os personagens. Antes, sem RagdollDynamics3D valido - o caso de
    // todo boneco nao controlado -, o centro de massa era a pelve.
    // Tracked every tick, independent of authority, so the balance-assist
    // velocity below never sees a stale gap when it turns on mid-collision.
    t.centerOfMass = physical.valid ? physical.centerOfMassWorld
        : state.links.front().position;
    const Vec3& centerOfMass = t.centerOfMass;
    Vec3 measuredComVelocity = physical.valid ? physical.centerOfMassVelocityWorld
        : Vec3 {};
    (void) dynamics;
    if (!finite(measuredComVelocity)) measuredComVelocity = {};
    const float comVelocityBlend = 1.0f - std::exp(-16.0f * deltaTime);
    m_filteredCenterOfMassVelocity += (measuredComVelocity
        - m_filteredCenterOfMassVelocity) * comVelocityBlend;
    // Curva real do corpo (nao a pedida): o rumo da velocidade do centro de
    // massa girando, filtrado em ~0,2 s (tira o balanco de cada passada).
    {
        const float bodySpeed = std::hypot(m_filteredCenterOfMassVelocity.x,
            m_filteredCenterOfMassVelocity.y);
        const float bodyHeading = std::atan2(m_filteredCenterOfMassVelocity.y,
            m_filteredCenterOfMassVelocity.x);
        const float rate = bodySpeed > 1.0f && m_bodyHeadingValid
            ? wrapAngle(bodyHeading - m_bodyHeading) / deltaTime : 0.0f;
        m_bodyHeading = bodyHeading;
        m_bodyHeadingValid = bodySpeed > 1.0f;
        m_bodyTurnRate += (rate - m_bodyTurnRate) * (1.0f - std::exp(-deltaTime / BodyTurnFilterSeconds));
        m_bodyCurveAcceleration = bodySpeed * m_bodyTurnRate;
    }
    m_previousCenterOfMass = centerOfMass;
    m_elapsedSeconds += deltaTime;
    // O giro da camera (o carrinho vira o corpo para o lado dele).
    {
        const float rate = m_previousViewValid ? wrapAngle(input.facingYawRadians - m_previousViewYaw)
            / std::max(deltaTime, 1e-4f) : 0.0f;
        m_viewTurnRate += (std::clamp(rate, -12.0f, 12.0f) - m_viewTurnRate) * (1.0f - std::exp(-deltaTime / 0.12f));
        m_previousViewYaw = input.facingYawRadians;
        m_previousViewValid = true;
    }

    // Fall detection: a real trip/collision this controller could not
    // absorb through the reaction/interference softening below. Contacts
    // are the physical ground truth here — hands/knees/torso touching the
    // floor, not the authored-clip foot weights the normal locomotion foot
    // lock further down uses for a completely different purpose (knowing
    // when a walking step has landed).
    t.anyGroundContact = false;
    bool& anyGroundContact = t.anyGroundContact;
    std::array<bool, 2> handGroundContact {};
    std::array<bool, 2> kneeGroundContact {};
    t.feetGroundContact = {};
    std::array<bool, 2>& feetGroundContact = t.feetGroundContact;
    t.contactFloor = std::numeric_limits<float>::infinity();
    float& contactFloor = t.contactFloor;
    float lowerBodyFloor = std::numeric_limits<float>::infinity();
    float nonFootFloorHeight = std::numeric_limits<float>::infinity();
    bool touchingAnything = false;
    for (const RagdollContactPoint3D& contact : state.contacts) {
        if (contact.normalImpulseNewtonSeconds > 0.00001f) touchingAnything = true;
        // Chao e superficie parada: uma caixa atingindo o corpo no ar, por
        // baixo, fazia a queda ser declarada ainda no ar.
        if (contact.normal.z < 0.40f || contact.otherBodyDynamic
            || contact.normalImpulseNewtonSeconds <= 0.00001f) {
            continue;
        }
        anyGroundContact = true;
        contactFloor = std::min(contactFloor, contact.position.z);
        // (v2: o chao de verdade e o da metade de baixo do corpo - pelve,
        // coxas, canelas, pes. O tronco ou os bracos apoiados no alto de uma
        // mureta nao sao chao: medido contra o topo dela, o corpo debrucado
        // parecia deitado - "caido" com o centro de massa a 0,9 m do chao.)
        if (contact.linkIndex < profile.links.size()) {
            const std::string& id = profile.links[contact.linkIndex].id;
            const bool foot = id.find("Foot") != std::string::npos;
            if (contact.linkIndex == 0 || id.find("Thigh") != std::string::npos
                || id.find("Shin") != std::string::npos || foot)
                lowerBodyFloor = std::min(lowerBodyFloor, contact.position.z);
            if (!foot) nonFootFloorHeight = std::min(nonFootFloorHeight, contact.position.z);
        }
        for (std::size_t side = 0; side < 2; ++side) {
            if (contact.linkIndex == m_handLinks[side]) {
                handGroundContact[side] = true;
            }
            if (contact.linkIndex == m_shinLinks[side]) {
                kneeGroundContact[side] = true;
            }
        }
    }
    // Pes: o apoio medido pelo estimador (contato com a normal para cima,
    // chao parado), o mesmo criterio de antes num lugar so.
    for (std::size_t side = 0; side < 2; ++side) {
        const FootSupportEstimate3D& foot = physical.feet[side];
        feetGroundContact[side] = m_feet[side].valid && foot.supporting
            && foot.evidence != ContactEvidence3D::Geometric && !foot.onDynamicBody;
    }

    // Caindo, ha quanto tempo cada mao segura o chao.
    for (std::size_t side = 0; side < 2; ++side)
        m_handCatchSeconds[side] = handGroundContact[side] && m_fallReflex > 0.3f
            ? std::min(2.0f, m_handCatchSeconds[side] + deltaTime)
            : m_fallReflex > 0.3f ? m_handCatchSeconds[side] : 0.0f;
    // Sem tocar nada (no ar): para o espernear da queda.
    m_noGroundSeconds = touchingAnything ? 0.0f
        : std::min(10.0f, m_noGroundSeconds + deltaTime);
    t.rootUpright = 1.0f;
    float& rootUpright = t.rootUpright;
    if (!state.links.empty()) {
        rootUpright = state.links.front().orientation
            .rotate({ 0.0f, 0.0f, 1.0f }).z;
    }
    // The capsule's implied ground (feet height it stands at) doubles as a
    // ground estimate here; contacts already gate whether anything is
    // actually down, this only sizes how far down it is.
    t.groundHeightGuess =
        input.rootPositionWorld.z - profile.standingRootHeightMeters;
    const float& groundHeightGuess = t.groundHeightGuess;
    // No modo fisico, contra a superficie que o corpo toca de fato: a
    // capsula decola antes no pulo, e medido contra ela o corpo no ar (o pe
    // ainda empurrando o chao) parecia "caido" - queda falsa na decolagem.
    // (v2: o mais baixo entre os contatos da metade de baixo, o apoio dos pes
    // e o chao da capsula - um pe no topo da mureta, com o outro no chao,
    // nao faz do topo o chao: em pe, 6 graus, ele era dado como caido.)
    const float fallenFloor = input.physicalV2
        ? std::min({ lowerBodyFloor, physical.supported ? physical.supportHeight
            : std::numeric_limits<float>::infinity(), groundHeightGuess }) : contactFloor;
    t.measuredComHeight = std::max(0.08f, centerOfMass.z
        - (input.forceDrivenRoot && std::isfinite(fallenFloor)
            ? fallenFloor : groundHeightGuess));
    const float& measuredComHeight = t.measuredComHeight;
    // Only meaningful while physics is the one holding the body up. In
    // animation mode the authored pose says the character is standing, so
    // "fallen" is not a state it can observe - and reading the simulated
    // bodies anyway is what produced the fall/stand-up/fall loop: those
    // bodies are still lying down for a few ticks after a recovery hands the
    // pose back, which instantly re-triggered another fall.
    // Com a raiz livre (forceDrivenRoot) o corpo e sempre o fisico: a queda
    // e reconhecida pelo estado dele, sem depender de um impacto antes.
    t.physicsOwnsBody = input.forceDrivenRoot || m_physicsBlend > 0.5f;
    const bool& physicsOwnsBody = t.physicsOwnsBody;
    // v2 - recuperacao apoiada (pedido do usuario, 06/10): o tronco, os bracos
    // ou as maos apoiados numa superficie parada no alto (o topo de uma
    // mureta, uma parede, um objeto), com um pe no chao. Debrucado assim,
    // ele ainda pode se recompor - pes para baixo do corpo, quadril e
    // joelhos estendendo, o tronco e os bracos empurrando o apoio (o par de
    // postura, interno: sem forca de fora). Enquanto tenta (BraceAttemptSeconds,
    // e progredindo), a postura nao e dada como perdida e nao e queda; antes,
    // a postura perdida amolecia as pernas na hora e ele ficava pendurado na
    // mureta (o estado de caido espera o corpo deitado).
    {
        bool bracedNow = false;
        if (input.physicalV2 && physicsOwnsBody && (feetGroundContact[0] || feetGroundContact[1])) {
            const float floor = std::isfinite(lowerBodyFloor) ? lowerBodyFloor : groundHeightGuess;
            for (const RagdollContactPoint3D& contact : state.contacts) {
                if (contact.otherBodyDynamic || contact.normalImpulseNewtonSeconds <= 0.00001f
                    || contact.normal.z < -0.3f || contact.linkIndex >= profile.links.size()
                    || contact.linkIndex == 0) continue;
                const std::string& id = profile.links[contact.linkIndex].id;
                if (id.find("Thigh") != std::string::npos || id.find("Shin") != std::string::npos
                    || id.find("Foot") != std::string::npos) continue;
                if (contact.position.z < floor + BraceMinimumHeight) continue;
                bracedNow = true;
                break;
            }
        }
        m_bracedMemory = bracedNow ? BraceMemorySeconds : std::max(0.0f, m_bracedMemory - deltaTime);
        const float tilt = std::acos(std::clamp(rootUpright, -1.0f, 1.0f));
        if (m_bracedMemory > 0.0f) {
            m_braceAttemptSeconds += deltaTime;
            // Progresso: a inclinacao caindo. Sem melhorar por
            // BraceStallSeconds, desiste (cai de verdade).
            if (tilt < m_braceBestTilt - 0.03f) {
                m_braceBestTilt = tilt;
                m_braceStallSeconds = 0.0f;
            } else {
                m_braceStallSeconds += deltaTime;
            }
        } else if (rootUpright > 0.85f || !(feetGroundContact[0] || feetGroundContact[1])) {
            m_braceAttemptSeconds = 0.0f;
            m_braceStallSeconds = 0.0f;
            m_braceBestTilt = tilt;
        }
        m_braceRecovering = m_bracedMemory > 0.0f && m_getUpPhase == CharacterGetUpPhase3D::None
            && m_braceAttemptSeconds < BraceAttemptSeconds && m_braceStallSeconds < BraceStallSeconds
            && rootUpright > BraceMinimumUpright && measuredComHeight > 0.40f;
        m_telemetry.braceRecovering = m_braceRecovering;
    }
    // (Logo depois do carrinho ele ainda vem baixo, com a mao ou o joelho
    // tocando o chao: ai so conta como queda tombado de verdade.)
    // (Cambaleando com um pe no chao - meio apoiado numa mureta, num boneco -
    // ainda nao e queda: ele tenta se endireitar ate StaggerFallenRelief a
    // mais de inclinacao, enquanto o cambaleio dura.)
    t.fallenUpright = 0.45f - StaggerFallenRelief * m_stagger
        * (feetGroundContact[0] || feetGroundContact[1] ? 1.0f : 0.0f);
    const float& fallenUpright = t.fallenUpright;
    // (v2: baixo, mas agachado sobre os pes - tronco ate ~53 graus, nada alem
    // dos pes tocando o chao - nao e queda: dali ele ainda levanta. Amolecido
    // por um choque, sem ajuda, ele descia num agachamento fundo e era dado
    // como caido com os pes no chao.)
    const bool squattingOnFeet = input.physicalV2 && (feetGroundContact[0] || feetGroundContact[1])
        && rootUpright > 0.6f && !(nonFootFloorHeight < fallenFloor + 0.30f) && measuredComHeight > 0.30f;
    t.fallenObservation = physicsOwnsBody && !m_braceRecovering
        && m_elapsedSeconds > 0.6f
        && anyGroundContact
        && (rootUpright < fallenUpright || (measuredComHeight < 0.52f && !squattingOnFeet))
        && (m_sinceTackleSeconds > 0.8f || rootUpright < 0.30f);
    const bool& fallenObservation = t.fallenObservation;

    const auto enterGetUpPhase = [&](CharacterGetUpPhase3D phase) {
        m_getUpPhase = phase;
        m_getUpPhaseSeconds = 0.0f;
        m_getUpFootwork = false;
        m_getUpStepSide = -1;
        m_getUpBalanceHold = false;
        m_getUpBalanceHoldSeconds = 0.0f;
        m_getUpBalanceSteps = 0;
        if (phase == CharacterGetUpPhase3D::Settling) {
            m_getUpRolling = false;
            m_getUpRollDone = false;
            m_getUpRollSeconds = 0.0f;
        }
    };
    const auto classifyFallOrientation = [&]() {
        const std::size_t reference = m_chestLink < state.links.size()
            ? m_chestLink : 0;
        const Vec3 bodyForward = state.links[reference]
            .orientation.rotate({ 1.0f, 0.0f, 0.0f });
        m_fallOrientation = bodyForward.z < 0.0f
            ? CharacterFallOrientation3D::FaceDown
            : CharacterFallOrientation3D::FaceUp;
    };
}

} // namespace MatterEngine
