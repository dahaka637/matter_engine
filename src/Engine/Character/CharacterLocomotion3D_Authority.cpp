#include "Engine/Character/CharacterLocomotion3DInternal.hpp"

namespace MatterEngine {

void CharacterLocomotion3D::stepAuthority(Tick& t) {
    const RagdollProfile3D& profile = t.profile;
    const RagdollState3D& state = t.state;
    const CharacterLocomotionInput3D& input = t.input;
    float& deltaTime = t.deltaTime;
    const CharacterPhysicalState3D& physical = *t.physical;
    const bool& getUpActive = t.getUpActive;
    // How much of the simulated body the player actually sees. Zero is the
    // normal case and means the rendered pose IS the authored pose, bit for
    // bit — the bodies still exist and still collide, they just do not get a
    // say in how the character looks. This is the same split Unreal draws
    // with PhysicsBlendWeight (see PhysAnim.cpp): animation owns the pose,
    // physics is blended in per body by weight, and at weight zero the
    // simulated result is discarded entirely instead of being "mostly
    // suppressed". Letting physics always contribute a little is what made
    // the character stand crooked, lean while walking and jitter.
    //
    // Only a dynamic body can raise it: another character, a prop, a
    // scripted moment later on. Walls, floors and curbs are static and
    // deliberately never push the character out of animation - clipping a
    // wall is a pose problem, not a reason to go limp.
    float dynamicImpulse = 0.0f;
    for (const RagdollContactPoint3D& contact : state.contacts) {
        if (!contact.otherBodyDynamic) continue;
        dynamicImpulse += std::max(0.0f, contact.normalImpulseNewtonSeconds);
    }
    // Scaled against what it actually takes to move this body, not against
    // its weight over a tick: a crate merely leaning on the character
    // delivers about mass*g*dt of impulse every tick forever, and measuring
    // against that made any resting contact read as a full-force hit and
    // drop the character on the spot. The gate is a real change in momentum
    // - ignore anything under ~0.05 m/s worth, reach full physics at
    // ~0.45 m/s worth, which is a genuine shove.
    t.mass = std::max(1.0f, profile.totalMassKg);
    const float& mass = t.mass;
    const float deadzone = mass * 0.05f;
    const float fullScale = mass * 0.45f;
    // Com a raiz livre nao ha limiar de impacto: a colisao age direto no
    // corpo e a cedencia local vem do controlador fisico.
    t.impact = input.forceDrivenRoot ? 0.0f : std::clamp(
        (dynamicImpulse - deadzone) / std::max(0.01f, fullScale - deadzone),
        0.0f, 1.0f);
    const float& impact = t.impact;
    if (impact > 0.0f) {
        // Proportional to the hit, and it never snaps: a shove pushes the
        // weight up quickly, then it holds while contact lasts.
        const float wanted = impact;
        m_physicsBlend = std::max(m_physicsBlend,
            std::min(1.0f, m_physicsBlend + wanted * deltaTime * 9.0f));
        m_physicsHoldSeconds = std::max(m_physicsHoldSeconds,
            0.25f + wanted * 0.75f);
    } else if (m_physicsHoldSeconds > 0.0f) {
        m_physicsHoldSeconds = std::max(0.0f,
            m_physicsHoldSeconds - deltaTime);
    } else {
        // Deliberately slower than the way in: coming back the instant
        // contact breaks is what made bumping another character read as
        // nothing happening at all.
        m_physicsBlend = std::max(0.0f, m_physicsBlend - deltaTime * 0.65f);
    }
    if (input.manipulated) {
        // Held by the PhysGun: the player is moving the body itself, so the
        // body is what they must see. Rendering the animation here made a
        // grab look like it was doing nothing while the bodies flew around.
        m_physicsBlend = 1.0f;
        m_physicsHoldSeconds = std::max(m_physicsHoldSeconds, 0.35f);
    }
    if (m_getUpPhase == CharacterGetUpPhase3D::Settling) {
        m_physicsBlend = 1.0f;
    } else if (m_getUpPhase == CharacterGetUpPhase3D::Rising) {
        // Hand the pose back to the clip over a short cross-fade so the
        // recovery starts from where the body actually is instead of popping
        // to the clip's first frame.
        m_physicsBlend = std::max(0.0f,
            1.0f - std::clamp(m_getUpClipSeconds / 0.30f, 0.0f, 1.0f));
    }
    m_reactionStrength = m_physicsBlend;
    const float requestedAuthority = std::clamp(
        input.poseAuthority, 0.0f, 1.0f);
    // At zero blend the guide holds the bodies exactly on the authored pose.
    // They still collide and still push props around; they simply have
    // nothing to contribute back, because the renderer is reading the
    // authored pose directly (see SpawnedRagdollInstance::animationPose).
    // Carried or free - never in between. A partial authority makes the
    // backend rewrite the articulation's positions AND velocities every
    // tick (applyCache), which injects energy on every frame: that is the
    // constant shiver, and it also wipes the friction the feet had just
    // accumulated, so the character slid like soap when something pushed
    // him. Handing the body over is a physical event, not a dial: while it
    // is carried nothing is written except the root, and the moment physics
    // takes over nothing is written at all.
    // Com a raiz livre, so o levantar (um clipe com trilha de raiz) ainda
    // carrega a pelve.
    // O levantar tambem nao carrega mais a pelve: e o corpo que levanta.
    const bool carried = !input.manipulated && !input.forceDrivenRoot
        && !getUpActive && m_physicsBlend < 0.35f;
    t.rootAuthority = carried ? requestedAuthority : 0.0f;
    const float& rootAuthority = t.rootAuthority;

    t.desiredPlanarVelocity = Vec3 { input.proxyVelocityWorld.x,
        input.proxyVelocityWorld.y, 0.0f };
    Vec3& desiredPlanarVelocity = t.desiredPlanarVelocity;
    // Com pouca ajuda a passada acompanha a inercia do corpo: no maximo
    // GaitLeadSpeed acima do que ele anda no rumo pedido. A capsula corre no
    // pedido desde o primeiro tick; no passo dela, o sprint inteiro (7,5 m/s)
    // saia com o corpo a 0,5 m/s - o pe pousava 0,8 m a frente e ele caia de
    // costas. Assim a cadencia e o passo crescem com o corpo.
    if (GaitLeadSpeed > 0.0f && input.forceDrivenRoot && !getUpActive && m_brakeGait <= 0.0f) {
        const float lowAssistLead = smoothStep((0.5f - std::clamp(input.walkingAssist, 0.0f, 1.0f)) / 0.4f);
        const float proxySpeed = std::hypot(desiredPlanarVelocity.x, desiredPlanarVelocity.y);
        // So correndo (pedido acima do trote): no trote a folga freava o
        // corpo (2,0 -> 1,8 m/s e paradas caindo).
        const float requestedSpeed = std::hypot(input.intent.requestedVelocityWorld.x, input.intent.requestedVelocityWorld.y);
        if (lowAssistLead > 0.0f && proxySpeed > 0.3f && requestedSpeed > GaitLeadMinimumRequest) {
            const Vec3 direction = desiredPlanarVelocity * (1.0f / proxySpeed);
            const float along = std::max(0.0f, dot(Vec3 { m_filteredCenterOfMassVelocity.x,
                m_filteredCenterOfMassVelocity.y, 0.0f }, direction));
            const float led = std::min(proxySpeed, along + GaitLeadSpeed);
            desiredPlanarVelocity *= 1.0f + (led / proxySpeed - 1.0f) * lowAssistLead;
        }
    }
    t.gaitVelocityScale = 1.0f;
    float& gaitVelocityScale = t.gaitVelocityScale;
    // Depois de uma pancada ou desequilibrio, as pernas continuam no rumo
    // pedido, mas no passo do corpo: o que ele anda de verdade naquele rumo
    // mais uma folga. Na passada do comando (correndo a 7 m/s com o corpo
    // parado numa caixa, ou indo para tras), os dois pes ficavam no ar na
    // fase de voo e nenhum passo segurava o corpo. Livre de novo, o teto sobe
    // no ritmo de uma arrancada.
    if (input.forceDrivenRoot) {
        const float disturbed = std::max(
            std::clamp(input.externalPerturbation, 0.0f, 1.0f), m_fallReflex);
        const float wantedSpeed = std::hypot(desiredPlanarVelocity.x,
            desiredPlanarVelocity.y);
        float along = 0.0f;
        if (wantedSpeed > 0.05f) {
            along = dot(Vec3 { m_filteredCenterOfMassVelocity.x,
                m_filteredCenterOfMassVelocity.y, 0.0f },
                desiredPlanarVelocity * (1.0f / wantedSpeed));
        }
        const float target = disturbed > 0.2f && !getUpActive
            ? std::max(0.0f, along) + 1.2f : 1000.0f;
        m_gaitSpeedCap = target < m_gaitSpeedCap
            ? std::max(target, m_gaitSpeedCap - 30.0f * deltaTime)
            : std::min(target, m_gaitSpeedCap + 6.0f * deltaTime);
        // Sem limitar, o teto acompanha o pedido logo acima dele (e numa
        // pancada cai dali): uma arrancada normal nunca e contida.
        m_gaitSpeedCap = std::min(m_gaitSpeedCap, wantedSpeed + 1.0f);
        if (target > 999.0f && m_gaitSpeedCap >= wantedSpeed)
            m_gaitSpeedCap = wantedSpeed + 1.0f;
        if (wantedSpeed > m_gaitSpeedCap) {
            gaitVelocityScale = m_gaitSpeedCap / wantedSpeed;
            desiredPlanarVelocity = desiredPlanarVelocity * gaitVelocityScale;
        }
    }
    // Freada pela passada (modo fisico). Soltando o comando correndo (ou
    // pedindo bem menos, ou o sentido contrario), a capsula freia a 18 m/s2
    // e para em ~0,4 s; o corpo, so pelos pes, freia ~5-6 m/s2. Seguindo a
    // capsula, a passada encurtava e parava com o corpo ainda a ~5 m/s: o
    // planejador de parado dava passos de recuperacao curtos e o corpo
    // passava por cima deles (sprint e para, ajuda parada em 0%: 6 quedas em
    // 8 direcoes). E o pe que pousava era posto em relacao a capsula, que vem
    // arrastada ate 0,5-0,7 m atras do corpo: pousava embaixo dele ou atras.
    // Enquanto o corpo leva um momento que o jogador nao pede, a passada
    // continua no passo do corpo real, num perfil de freada que ele consegue
    // cumprir, a referencia fica presa ao corpo (nao a capsula) e o pe pousa
    // a frente do centro de massa; a pelve desce e o tronco segura atras
    // (reacao de freada, mais abaixo). Devagar, o planejador de parado assume.
    t.brakeNavigationVelocity = {};
    Vec3& brakeNavigationVelocity = t.brakeNavigationVelocity;
    {
        const Vec3 bodyPlanar { m_filteredCenterOfMassVelocity.x,
            m_filteredCenterOfMassVelocity.y, 0.0f };
        const float bodySpeed = bodyPlanar.length();
        const Vec3 requested { input.intent.requestedVelocityWorld.x,
            input.intent.requestedVelocityWorld.y, 0.0f };
        const Vec3 direction = bodySpeed > 0.05f
            ? bodyPlanar * (1.0f / bodySpeed) : m_brakeGaitDirection;
        const float wantedAlong = dot(requested, direction);
        // O momento que o jogador nao pede: o que passa da velocidade pedida
        // no rumo do corpo (pedindo o contrario, tudo).
        const float unwanted = bodySpeed - std::max(0.0f, wantedAlong);
        // Para seguir freando o chao da capsula nao conta: arrastada atras do
        // corpo ela pisca "no ar" e a freada terminava no meio dos passos.
        const bool continuable = input.forceDrivenRoot && !getUpActive
            && !input.manipulated && m_fallCommit < 0.5f;
        const bool eligible = continuable && input.grounded;
        // So soltando o comando (ou quase) ou pedindo o sentido contrario:
        // numa curva ou num corte de lado a passada continua e vira com a
        // capsula, como antes.
        const float requestedSpeed = requested.length();
        // Com pouca ajuda, um corte forte (60 graus ou mais) com o corpo rapido
        // tambem e freada: quem leva o corpo para o novo rumo sao os pes, e a
        // 6 m/s eles curvam ~1 rad/s - virando com a capsula, a passada ia
        // para o rumo novo com o corpo seguindo reto, as pernas cruzavam e
        // ele mergulhava (corte de 90 graus a 30%: 6-8 quedas em 8). Freia
        // plantando o pe, pivota e sai no rumo novo (como na inversao).
        const float cutAssist = smoothStep((0.6f - std::clamp(input.walkingAssist, 0.0f, 1.0f)) / 0.5f);
        const bool cutting = cutAssist > 0.3f && bodySpeed > CutBrakeMinimumSpeed
            && dot(requested, direction) < CutBrakeCosine * requestedSpeed;
        const bool stopping = requestedSpeed < 0.5f
            || dot(requested, direction) < -0.5f * requestedSpeed || cutting;
        // E so o momento da propria corrida (o jogador pedia movimento ha
        // pouco e nada de fora bateu nele): um empurrao forte parado (300
        // N.s, o corpo a ~4 m/s) e da recuperacao de empurrao - pela freada
        // ele perdia os passos de recuperacao e caia.
        m_sinceRunRequest = requestedSpeed > 1.0f ? 0.0f
            : std::min(10.0f, m_sinceRunRequest + deltaTime);
        const bool ownMomentum = m_sinceRunRequest < 0.6f
            && input.externalPerturbation < 0.2f;
        if (!m_brakeGaitActive) {
            // De lado (strafe), a freada pela passada entra mais devagar: o
            // strafe tem passo cruzado e, parando sem ela a 1,6 m/s, os
            // passos de parado herdavam os pes cruzados com o corpo indo para
            // o lado - a 5%, 14 quedas em 16 parando de um strafe.
            const bool sideways = m_gaitDirection == CharacterGaitDirection3D::Left
                || m_gaitDirection == CharacterGaitDirection3D::Right;
            const float engageSpeed = sideways ? StrafeBrakeEngageSpeed : BrakeEngageSpeed;
            const float engageExcess = sideways ? std::min(BrakeEngageExcessSpeed, StrafeBrakeEngageSpeed * 0.6f)
                : BrakeEngageExcessSpeed;
            if (eligible && stopping && ownMomentum && bodySpeed > engageSpeed
                && unwanted > engageExcess) {
                m_brakeGaitActive = true;
                m_brakeGaitSpeed = bodySpeed;
            }
        } else if (!continuable
            // A freada nunca prende o comando: pedindo movimento para um lado
            // qualquer (que nao o contrario) ela acaba na hora e a passada
            // assume; pedindo o contrario, so ate o corpo ficar devagar.
            || (requestedSpeed >= 0.5f && (!stopping || bodySpeed < BrakeReverseReleaseSpeed))
            || (m_brakeStepping && m_footwork.plan().brakeYield)
            || ((bodySpeed < BrakeReleaseSpeed || unwanted < 0.5f)
                && (!m_brakeStepping || m_footwork.plan().brakeSettled
                    || m_brakeSeconds > BrakeMaximumSeconds))) {
            // Nos passos, so termina com o corpo parando sobre a base (o ponto
            // de captura entre os pes, a frente do de tras - o planejador
            // diz): parado com a base toda adiante do corpo, ele escorregava
            // de volta e os passos de parado, sem ajuda, davam dois ou tres
            // passos para tras (a "cambaleada").
            m_brakeGaitActive = false;
        }
        m_brakeSeconds = m_brakeGaitActive ? m_brakeSeconds + deltaTime : 0.0f;
        m_brakeReversing = m_brakeGaitActive && requestedSpeed >= 0.5f;
        if (m_brakeGaitActive) {
            // Quase parado, a direcao da velocidade gira a toa (e ate inverte):
            // fica a de quando ele ainda andava.
            if (bodySpeed > 0.8f || m_brakeGaitDirection.lengthSquared() < 0.5f)
                m_brakeGaitDirection = direction;
            // Perfil de freada: desce ate o pedido no rumo (ou zero) no ritmo
            // que os pes conseguem, sem se afastar do corpo real mais do que
            // uma folga - a passada tem de acompanhar o chao que o corpo anda.
            // Com pouca ajuda quem freia sao so as pernas (~2-4 m/s2): a
            // passada no perfil de 12 m/s2, ate 0,8 m/s abaixo do corpo, ficava
            // curta e lenta para o corpo que seguia rapido - os pes ficavam
            // atras dele e ele mergulhava. Ela fica perto do corpo.
            const float lowAssistBrake = smoothStep((0.5f - std::clamp(input.walkingAssist, 0.0f, 1.0f)) / 0.4f);
            const float deceleration = (m_brakeReversing ? ReverseDeceleration : BrakeDeceleration)
                + (LowAssistBrakeDeceleration - (m_brakeReversing ? ReverseDeceleration : BrakeDeceleration)) * lowAssistBrake;
            const float profileLeash = BrakeProfileLeash + (LowAssistBrakeLeash - BrakeProfileLeash) * lowAssistBrake;
            m_brakeGaitSpeed = std::max(std::max(0.0f, wantedAlong),
                m_brakeGaitSpeed - deceleration * deltaTime);
            m_brakeGaitSpeed = std::clamp(m_brakeGaitSpeed,
                std::max(0.0f, bodySpeed - profileLeash), bodySpeed);
        }
        // Rapido, a corrida desacelera (a perna de corrida e quem traz o pe a
        // frente a tempo); devagar, passos de freada sem voo ate parar.
        // (Pela velocidade do corpo: a do perfil fica abaixo dela e os passos
        // comecavam com o corpo ainda a 3,7 m/s, os pes atras dele.)
        // Com pouca ajuda (TEMPTUNE: em teste) os passos de freada podem
        // comecar mais cedo: a corrida freando depende do tempo do clipe (18%
        // de apoio por pe), e o corpo, sem ajuda, fica mais tempo no chao -
        // o pe que o clipe queria levar a frente ainda carregava o corpo.
        const float lowAssistSteps = smoothStep((0.5f - std::clamp(input.walkingAssist, 0.0f, 1.0f)) / 0.4f);
        const float stepSpeed = BrakeStepSpeed + (LowAssistBrakeStepSpeed - BrakeStepSpeed) * lowAssistSteps;
        // A troca espera um pe da passada no chao, a frente do corpo (o
        // apoio que ja freia - no maximo uma fase de voo): trocando com os
        // dois no ar, os dois pousavam como passos de freada, um atras do
        // outro, e a parada tinha sempre dois passos. Assim o pe plantado
        // freia e so o outro pousa - um passo.
        // (O apoio fisico do estimador: o "plantado" do clipe nem sempre marca
        // o pe que ja esta no chao, e a espera ia ate o limite com o corpo
        // freando a 0,4 m/s ainda na passada.)
        bool plantedAhead = false;
        for (const FootSupportEstimate3D& foot : physical.feet) {
            if (!foot.supporting) continue;
            plantedAhead = plantedAhead || dot(Vec3 { foot.soleCenterWorld.x - physical.centerOfMassWorld.x,
                foot.soleCenterWorld.y - physical.centerOfMassWorld.y, 0.0f }, direction) > BrakePlantedAhead;
        }
        m_brakeStepWait = m_brakeGaitActive && !m_brakeStepping && bodySpeed < stepSpeed
            ? m_brakeStepWait + deltaTime : 0.0f;
        const bool stepReady = plantedAhead || m_brakeStepWait > BrakeStepWaitSeconds;
        const bool enteringSteps = m_brakeGaitActive && !m_brakeStepping && bodySpeed < stepSpeed && stepReady;
        m_brakeStepping = m_brakeGaitActive
            && (m_brakeStepping || (bodySpeed < stepSpeed && stepReady));
        // Freando ja devagar (do trote), os passos comecam na hora, com a
        // base no corpo desde o primeiro tick.
        if (enteringSteps && m_brakeGait <= 0.0f) m_brakeAnchor = 1.0f;
        m_sinceBrakeSeconds = m_brakeGaitActive ? 0.0f
            : std::min(10.0f, m_sinceBrakeSeconds + deltaTime);
        m_brakeGait = m_brakeGaitActive
            ? std::min(1.0f, m_brakeGait + deltaTime / 0.08f)
            : std::max(0.0f, m_brakeGait - deltaTime / BrakeFadeSeconds);
        // Correndo e freando, a corrida ainda e corrida: a ajuda e a de quem
        // corre (a passada de corrida ainda depende dela), e vai para a de
        // parado conforme a velocidade cai ate os passos de freada.
        // A ajuda de corrida fica inteira enquanto a passada de corrida freia
        // (trotar sem ela ainda nao se sustenta) e sai depressa quando os
        // passos de freada comecam - como numa freada que ja parte do trote.
        const bool running = m_brakeGaitActive && !m_brakeStepping;
        m_brakeRunShare = running
            ? std::min(1.0f, m_brakeRunShare + deltaTime / 0.08f)
            : std::max(0.0f, m_brakeRunShare - deltaTime / BrakeRunFadeSeconds);
        m_telemetry.brakeRunShare = m_brakeRunShare;
        m_telemetry.brakePhase = !m_brakeGaitActive ? 0 : m_brakeStepping ? 2 : 1;
        // (Terminada a freada, a navegacao volta direto a capsula: o resto do
        // perfil fazia o estado virar "andando" por um instante, o clipe de
        // trote pegava os pes de volta e desmontava a base ja parado.)
        if (m_brakeGait > 0.0f && m_brakeGaitActive) {
            brakeNavigationVelocity = m_brakeGaitDirection * m_brakeGaitSpeed;
            desiredPlanarVelocity += (brakeNavigationVelocity - desiredPlanarVelocity)
                * m_brakeGait;
        } else if (m_brakeGait > 0.0f) {
            brakeNavigationVelocity = desiredPlanarVelocity;
        }
        m_telemetry.brakeRunVelocityWorld = brakeNavigationVelocity;
    }
    const Vec3 rawAcceleration = clampMagnitude(
        (desiredPlanarVelocity - m_previousDesiredVelocity) / deltaTime,
        24.0f);
    m_previousDesiredVelocity = desiredPlanarVelocity;
    const float accelerationBlend = 1.0f - std::exp(-9.0f * deltaTime);
    m_smoothedAcceleration +=
        (rawAcceleration - m_smoothedAcceleration) * accelerationBlend;

}

} // namespace MatterEngine
