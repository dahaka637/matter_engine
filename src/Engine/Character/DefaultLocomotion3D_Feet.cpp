#include "Engine/Character/DefaultLocomotion3DInternal.hpp"

namespace MatterEngine {

void DefaultLocomotion3D::stepFeet(Tick& t) {
    const RagdollProfile3D& profile = t.profile;
    const DefaultLocomotionInput3D& input = t.input;
    const DefaultLocomotionSettings3D& s = m_settings;
    const float& dt = t.dt;
    const Vec3& velocity = t.velocity;
    const float& speed = t.speed;
    float& touchdown = t.touchdown;
    const Vec3& requested = t.requested;
    const Vec3& travel = t.travel;
    std::array<float, GaitCount>& wanted = t.wanted;
    float& shapeTotal = t.shapeTotal;
    float& strideScale = t.strideScale;
    std::array<float, 2>& contact = t.contact;
    const float& ground = t.ground;
    const float& curveAcceleration = t.curveAcceleration;
    const Vec3& feet = t.feet;
    Quaternion& rootOrientation = t.rootOrientation;
    Vec3& rootPosition = t.rootPosition;
    std::array<bool, 2>& gestureFeet = t.gestureFeet;
    const Quaternion& rootInverse = t.rootInverse;
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
            // (So le os pes: as cadeias ate eles.)
            RagdollAnimationPose3D air;
            buildRagdollChainPose3D(profile, m_coordinates, air, m_feet[0]);
            buildRagdollChainPose3D(profile, m_coordinates, air, m_feet[1]);
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
    // (So le os pes: as cadeias ate eles.)
    RagdollAnimationPose3D local;
    buildRagdollChainPose3D(profile, m_coordinates, local, m_feet[0]);
    buildRagdollChainPose3D(profile, m_coordinates, local, m_feet[1]);
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
            const bool ballFoot = gestureFeet[side];
            const auto strideShift = [&](Vec3 at) {
                Vec3 result {};
                if (std::abs(strideAmount) <= 1e-4f) return result;
                for (std::size_t g = 0; g < GaitCount; ++g) {
                    const float share = m_shape[g] / shapeTotal;
                    if (share <= 1e-3f) continue;
                    const float axisYaw = m_yaw + m_gaits[g].heading;
                    const Vec3 axis { std::cos(axisYaw), std::sin(axisYaw), 0.0f };
                    const float along = dot(planar(at - rootPosition), axis);
                    result += axis * (along * strideAmount * share);
                }
                return result;
            };
            const Vec3 shift = ballFoot ? Vec3 {} : strideShift(position);
            // (A perna ainda saindo do gesto anterior tambem: a pose ja traz o
            // pe dela - lido dali, o "pe do clipe" era o proprio pe e a volta
            // corria atras de si mesma, ate 1 m de altura.)
            const bool releaseFoot = !ballFoot && input.ballGesture.active && input.ballGesture.releaseWeight > 0.0f
                && static_cast<int>(side) == input.ballGesture.releaseSide;
            if (ballFoot || releaseFoot) {
                // O pe do gesto: onde a locomocao o poria sem ele (o do estilo
                // com a passada e o chao) - o gesto volta para ele.
                const Vec3 style = m_output.styleFootPosition[side];
                const Quaternion styleOrientation = m_output.styleFootOrientation[side];
                Vec3 clip = style + strideShift(style);
                clip.z -= (m_trunkHeight - m_landingSink) * std::clamp(m_plantWeight[side], 0.0f, 1.0f);
                float clipGround = feet.z;
                if (input.groundAt) {
                    const GroundProbeResult3D probe = input.groundAt(clip);
                    if (probe.hasSurface && std::abs(probe.pointWorld.z - feet.z) < 0.6f) clipGround = probe.pointWorld.z;
                }
                clip.z += std::max(0.0f,
                    clipGround - soleHeight(link, clip, styleOrientation, s.toeRoll ? s.toeRollMeters : 0.0f));
                m_output.locomotionFootPosition[side] = clip;
                m_output.locomotionFootOrientation[side] = styleOrientation;
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
            {
                // O pe no balanco passa por cima do degrau a frente (os dedos
                // e um pouco alem, no rumo do movimento): a sola sobe ate o
                // chao mais alto dali com folga, numa mola - com o chao so
                // embaixo do pe, os dedos batiam no espelho do degrau e subir
                // escada ia ao fisico (plano v2, fase E).
                float wanted = 0.0f;
                if (footContact < 0.5f && input.groundAt && ground > 0.5f) {
                    Vec3 travel = planar(velocity);
                    travel = travel.length() > 0.3f ? travel.normalized()
                                                    : planar(orientation.rotate({ 1.0f, 0.0f, 0.0f })).normalized();
                    // (Correndo o pe anda mais por balanco: a busca vai mais
                    // longe.)
                    const float reachScale = std::clamp(planar(velocity).length() / 2.5f, 1.0f, 2.2f);
                    float ahead = groundZ;
                    for (const float d : { 0.12f, 0.24f, 0.36f }) {
                        const GroundProbeResult3D probe = input.groundAt(shifted + travel * (d * reachScale));
                        if (probe.hasSurface && probe.pointWorld.z - feet.z < 0.45f && probe.pointWorld.z - feet.z > -0.6f)
                            ahead = std::max(ahead, probe.pointWorld.z);
                    }
                    if (ahead > groundZ + 0.02f) wanted = std::max(0.0f, ahead + 0.03f - sole);
                }
                spring(m_stepClear[side], m_stepClearRate[side], wanted, 0.08f, dt);
                lift = std::max(lift, m_stepClear[side]);
            }
            if (m_landPull > 0.0f && groundZ < sole) {
                // Aterrissando: o pe no ar vai ao chao.
                const float pull = m_landPull * (m_landLeadSide < 0 || static_cast<int>(side) == m_landLeadSide
                    ? 1.0f : 1.0f - m_landRunning);
                lift = (groundZ - sole) * pull;
            }
            const Vec3 target = shifted + Vec3 { 0.0f, 0.0f, lift };
            if (!ballFoot && !releaseFoot) {
                m_output.locomotionFootPosition[side] = target;
                m_output.locomotionFootOrientation[side] = orientation;
            }
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
        (void)moved;
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

}

} // namespace MatterEngine
