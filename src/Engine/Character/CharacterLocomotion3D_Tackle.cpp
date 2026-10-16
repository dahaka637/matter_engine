#include "Engine/Character/CharacterLocomotion3DInternal.hpp"

namespace MatterEngine {

void CharacterLocomotion3D::stepTackle(Tick& t) {
    const RagdollProfile3D& profile = t.profile;
    const CharacterLocomotionAnimations3D& animations = t.animations;
    const RagdollState3D& state = t.state;
    const CharacterLocomotionInput3D& input = t.input;
    float& deltaTime = t.deltaTime;
    std::array<bool, 2>& feetGroundContact = t.feetGroundContact;
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
    // Carrinho: correndo, agachar (a borda do pedido) arma o carrinho para
    // onde ele olha: mais devagar o de lado (Soccer Tackle), mais rapido o
    // sentado (Running Slide, que desliza mais). O preparo: ele segue
    // correndo, a pelve baixa um pouco, e o mergulho comeca no POUSO do pe
    // com que o clipe comeca apoiado - a passada encaixa no clipe (antes
    // comecava em qualquer ponto da passada, com o pe errado no chao). So no
    // chao, de pe, indo para a frente do olhar com embalo - parado, recuando
    // ou de lado o Ctrl so agacha.
    {
        const bool crouchPressed = input.intent.crouch && !m_previousCrouchIntent;
        m_previousCrouchIntent = input.intent.crouch;
        m_tackleCooldown = std::max(0.0f, m_tackleCooldown - deltaTime);
        m_sinceTackleSeconds = m_tackling ? 0.0f : std::min(10.0f, m_sinceTackleSeconds + deltaTime);
        const float yaw = input.facingYawRadians;
        const Vec3 direction { std::cos(yaw), std::sin(yaw), 0.0f };
        const float along = dot(Vec3 { m_filteredCenterOfMassVelocity.x,
            m_filteredCenterOfMassVelocity.y, 0.0f }, direction);
        const bool moving = m_telemetry.state == CharacterLocomotionState3D::Walking
            || m_telemetry.state == CharacterLocomotionState3D::Running;
        const bool haveClip = animations.tackles[0] != nullptr || animations.tackles[1] != nullptr;
        const bool canTackle = TackleEnabled && input.tackleEnabled && haveClip
            && m_getUpPhase == CharacterGetUpPhase3D::None && !m_tackling
            && input.grounded && moving && along >= TackleMinSpeed * (m_tacklePending ? 0.8f : 1.0f)
            && m_fallCommit < 0.3f && !state.links.empty();
        // O pe com que cada clipe comeca apoiado (o mais baixo no comeco):
        // o de lado firma o esquerdo e da o bote com a direita; o sentado
        // firma o direito e estica a esquerda.
        const auto clipStartSide = [&](const AnimationClip3D* clip) {
            if (clip == nullptr) return -1;
            const RagdollAnimationPose3D first = sampleRagdollAnimationPose3D(profile, clip, 0.0f, false);
            if (m_feet[0].linkIndex >= first.linkPositions.size()
                || m_feet[1].linkIndex >= first.linkPositions.size()) return -1;
            return first.linkPositions[m_feet[0].linkIndex].z
                < first.linkPositions[m_feet[1].linkIndex].z ? 0 : 1;
        };
        const auto clipForSide = [&](int side) {
            for (const AnimationClip3D* clip : animations.tackles)
                if (clip != nullptr && clipStartSide(clip) == side) return clip;
            return animations.tackles[0] != nullptr ? animations.tackles[0] : animations.tackles[1];
        };
        if (crouchPressed && canTackle && m_tackleCooldown <= 0.0f && !m_tacklePending) {
            m_tacklePending = true;
            m_tacklePendingSeconds = 0.0f;
            // A situacao decide: o pe que vai pousar no proximo passo (o que
            // esta no ar; no voo, o da frente) escolhe o carrinho que comeca
            // firmando ele - o mergulho sai nesse pouso, sem esperar outro.
            int next = 0;
            if (feetGroundContact[0] != feetGroundContact[1]) {
                next = feetGroundContact[0] ? 1 : 0;
            } else if (m_feet[0].linkIndex < state.links.size() && m_feet[1].linkIndex < state.links.size()) {
                next = dot(state.links[m_feet[0].linkIndex].position, direction)
                    > dot(state.links[m_feet[1].linkIndex].position, direction) ? 0 : 1;
            }
            m_tackleStartSide = next;
            m_tackleClip = clipForSide(next);
            // Qual (pedido do usuario, 05/10 noite; como no padrao): o que
            // vira o corpo para o lado do giro da camera; parada, para o lado
            // da corrida; senao, ao acaso. Fica esse (sem trocar pelo pe).
            if (TackleChoiceByView) {
                std::array<TackleClipShape3D, 2> shapes {};
                for (std::size_t k = 0; k < shapes.size(); ++k)
                    if (animations.tackles[k] != nullptr) shapes[k] = tackleClipShape3D(profile, *animations.tackles[k]);
                const Vec3 travel { input.intent.requestedVelocityWorld.x, input.intent.requestedVelocityWorld.y, 0.0f };
                const float lateral = travel.lengthSquared() > 1e-4f
                    ? std::sin(wrapAngle(std::atan2(travel.y, travel.x) - input.facingYawRadians)) : 0.0f;
                const int chosen = chooseTackleClip3D(animations.tackles, shapes, m_viewTurnRate, lateral,
                    (input.varietySeed * 2654435761u) ^ (m_tackleCount * 40503u) ^ static_cast<std::uint32_t>(m_elapsedSeconds * 1000.0f));
                if (chosen >= 0) {
                    m_tackleClip = animations.tackles[static_cast<std::size_t>(chosen)];
                    m_tackleStartSide = clipStartSide(m_tackleClip);
                    m_tackleChosen = true;
                }
            }
            m_tackleVariant = m_tackleClip == animations.tackles[1] ? 1 : 0;
        }
        if (m_tacklePending && !canTackle) {
            m_tacklePending = false;
            m_tackleClip = nullptr;
        }
        // A voadora no modo fisico (pedido do usuario, 06/10; no padrao ela ja
        // existia): no ar, Ctrl com Shift + W, uma por voo. O carrinho entra ja
        // na pose do mergulho (a perna esticada, do mesmo ponto em que o
        // fisico continua a voadora do padrao) e a segura ate o corpo descer;
        // ele nao empurra nem freia o corpo, entao o voo e o embalo do pulo.
        if (input.grounded) m_airTackleSpent = false;
        bool airStart = false;
        float startClipTime = 0.0f, startSpeed = along, startYaw = yaw;
        {
            const Vec3 keys = input.intent.requestedDirectionWorld;
            const bool combo = input.intent.crouch && input.intent.effort > 0.5f
                && keys.x * direction.x + keys.y * direction.y > 0.7f;
            const bool flying = !input.grounded && m_telemetry.state == CharacterLocomotionState3D::Airborne
                && m_stateSeconds > 0.05f;
            if (combo && flying && input.airTackleEnabled && !m_airTackleSpent && TackleEnabled && input.tackleEnabled
                && haveClip && m_getUpPhase == CharacterGetUpPhase3D::None && !m_tackling && !m_tacklePending
                && m_tackleCooldown <= 0.0f && m_fallCommit < 0.3f && !state.links.empty()) {
                m_airTackleSpent = true;
                std::array<TackleClipShape3D, 2> shapes {};
                for (std::size_t k = 0; k < shapes.size(); ++k)
                    if (animations.tackles[k] != nullptr) shapes[k] = tackleClipShape3D(profile, *animations.tackles[k]);
                const Vec3 flat { m_filteredCenterOfMassVelocity.x, m_filteredCenterOfMassVelocity.y, 0.0f };
                const float flatSpeed = flat.length();
                const float lateral = flatSpeed > 0.01f
                    ? std::sin(wrapAngle(std::atan2(flat.y, flat.x) - input.facingYawRadians)) : 0.0f;
                int chosen = chooseTackleClip3D(animations.tackles, shapes, m_viewTurnRate, lateral,
                    (input.varietySeed * 2654435761u) ^ (m_tackleCount * 40503u)
                        ^ static_cast<std::uint32_t>(m_elapsedSeconds * 1000.0f));
                if (chosen < 0) chosen = animations.tackles[0] != nullptr ? 0 : 1;
                const std::size_t k = static_cast<std::size_t>(chosen);
                m_tackleClip = animations.tackles[k];
                m_tackleStartSide = clipStartSide(m_tackleClip);
                m_tackleVariant = static_cast<int>(k);
                // (Do fim do mergulho, como o do padrao: do comeco dele, o
                // corpo ficava em pe no ar com uma perna a frente e pousava
                // sentado.)
                startClipTime = std::clamp(std::max(shapes[k].diveStart, shapes[k].lowStart - 0.02f),
                    0.0f, m_tackleClip->durationSeconds);
                startSpeed = std::max(2.0f, flatSpeed);
                startYaw = flatSpeed > 1.0f ? std::atan2(flat.y, flat.x) : input.facingYawRadians;
                airStart = true;
            }
        }
        const std::array<bool, 2> touchdown { feetGroundContact[0] && !m_tacklePreviousFoot[0],
            feetGroundContact[1] && !m_tacklePreviousFoot[1] };
        m_tacklePreviousFoot = { feetGroundContact[0], feetGroundContact[1] };
        bool start = false;
        if (m_tacklePending) {
            m_tacklePendingSeconds += deltaTime;
            // Pousou um pe: o carrinho dele (se pousou o outro antes do
            // previsto, troca para o dele). Passando do prazo, o pe que esta
            // no chao.
            int side = -1;
            if (touchdown[0] != touchdown[1]) side = touchdown[0] ? 0 : 1;
            else if (touchdown[0]) side = m_tackleStartSide;
            else if (m_tacklePendingSeconds > TacklePrepMax)
                side = feetGroundContact[0] != feetGroundContact[1] ? (feetGroundContact[0] ? 0 : 1)
                    : m_tackleStartSide;
            if (side >= 0) {
                if (side != m_tackleStartSide && !m_tackleChosen) {
                    m_tackleStartSide = side;
                    m_tackleClip = clipForSide(side);
                    m_tackleVariant = m_tackleClip == animations.tackles[1] ? 1 : 0;
                }
                start = true;
            }
        }
        if (airStart) start = true;
        if (start && m_tackleClip != nullptr) {
            m_tacklePending = false;
            ++m_tackleCount;
            // Ritmo e percurso pela velocidade de entrada contra a da
            // corrida do comeco do clipe: mais devagar, o clipe toca um
            // pouco mais lento e o caminho encurta (as duas pela raiz).
            const AnimationTransformSample3D r0 = clipRootAt(profile, *m_tackleClip, 0.0f);
            const AnimationTransformSample3D r1 = clipRootAt(profile, *m_tackleClip, 0.2f);
            const float clipSpeed = std::max(1.0f, std::hypot(
                r1.translationOffsetMeters.x - r0.translationOffsetMeters.x,
                r1.translationOffsetMeters.y - r0.translationOffsetMeters.y) / 0.2f);
            const float ratio = std::clamp(startSpeed / clipSpeed, 0.4f, 1.3f);
            m_tackleRate = std::clamp(std::sqrt(ratio), 0.75f, 1.12f);
            m_tackleDistanceScale = ratio / m_tackleRate;
            m_tackleChosen = false;
            // Os dois deslizam o mesmo: o caminho de cada um vai a fase
            // deitada media deles.
            {
                float sum = 0.0f;
                int count = 0;
                TackleClipShape3D mine {};
                for (const AnimationClip3D* other : animations.tackles) {
                    if (other == nullptr) continue;
                    const TackleClipShape3D shape = tackleClipShape3D(profile, *other);
                    if (other == m_tackleClip) mine = shape;
                    if (shape.lowMeters > 0.3f) { sum += shape.lowMeters; ++count; }
                }
                if (count > 0 && mine.lowMeters > 0.3f && TackleChoiceByView)
                    m_tackleDistanceScale *= (sum / static_cast<float>(count)) / mine.lowMeters;
            }
            m_tackleEntrySpeed = startSpeed;
            m_tackleYaw = startYaw;
            // O caminho do clipe comeca onde a pelve esta (no ar, no ponto do
            // mergulho).
            const Vec3 pelvis = state.links.front().position;
            const AnimationTransformSample3D rStart = airStart ? clipRootAt(profile, *m_tackleClip, startClipTime) : r0;
            const Vec3 first = Quaternion::fromAxisAngle({ 0.0f, 0.0f, 1.0f }, startYaw).rotate({
                rStart.translationOffsetMeters.x * m_tackleDistanceScale,
                rStart.translationOffsetMeters.y * m_tackleDistanceScale, 0.0f });
            m_tackleStart = { pelvis.x - first.x, pelvis.y - first.y, pelvis.z };
            m_tackling = true;
            m_tackleLandSpeed = -1.0f;
            m_tackleBehindSeconds = 0.0f;
            m_tackleHoldSeconds = 0.0f;
            enterGetUpPhase(CharacterGetUpPhase3D::Rising);
            m_getUpClipSeconds = startClipTime;
            m_getUpAnchorWorld = m_tackleStart;
            m_getUpAnchorYaw = startYaw;
            m_fallOrientation = CharacterFallOrientation3D::None;
            m_settleGather = 1.0f;
            m_settleTone = 1.0f;
            m_getUpRate = m_tackleRate;
            m_getUpAttempt = 0;
            m_getUpExpectedValid = false;
            m_getUpStruggle = 0.0f;
            m_getUpStallSeconds = 0.0f;
            m_getUpHoldSeconds = 0.0f;
            m_bodilyRising = false;
            m_risingFromMeasured.assign(state.joints.size(), Vec3 {});
            for (std::size_t link = 1; link < state.joints.size(); ++link) {
                m_risingFromMeasured[link] = { state.joints[link].positionRadians[0],
                    state.joints[link].positionRadians[1],
                    state.joints[link].positionRadians[2] };
            }
            for (FootPlant& foot : m_feet) {
                foot.planted = false;
                foot.repositioning = false;
                foot.recoveryStep = false;
            }
        }
    }
}

} // namespace MatterEngine
