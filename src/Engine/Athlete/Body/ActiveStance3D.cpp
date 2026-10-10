#include "Engine/Athlete/Body/ActiveStance3D.hpp"

#include <algorithm>
#include <cmath>

namespace MatterEngine {
namespace {

float smooth(float t) {
    t = std::clamp(t, 0.0f, 1.0f);
    return t * t * (3.0f - 2.0f * t);
}

Quaternion nlerp(Quaternion a, Quaternion b, float t) {
    if (a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w < 0.0f) b = { -b.x, -b.y, -b.z, -b.w };
    return Quaternion { a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t, a.z + (b.z - a.z) * t, a.w + (b.w - a.w) * t }
        .normalized();
}

float planarDistance(Vec3 a, Vec3 b) { return std::hypot(a.x - b.x, a.y - b.y); }

float cross2(float ax, float ay, float bx, float by) { return ax * by - ay * bx; }

// Quanto o ponto (x, y) esta fora do fecho convexo dos pontos (m; 0 dentro).
float outsideHull(std::vector<std::array<float, 2>> points, float x, float y) {
    if (points.empty()) return 10.0f;
    std::sort(points.begin(), points.end());
    std::vector<std::array<float, 2>> hull;
    for (int pass = 0; pass < 2; ++pass) {
        const std::size_t start = hull.size();
        for (const auto& p : points) {
            while (hull.size() >= start + 2) {
                const auto& a = hull[hull.size() - 2];
                const auto& b = hull[hull.size() - 1];
                if (cross2(b[0] - a[0], b[1] - a[1], p[0] - a[0], p[1] - a[1]) > 0.0f) break;
                hull.pop_back();
            }
            hull.push_back(p);
        }
        hull.pop_back();
        std::reverse(points.begin(), points.end());
    }
    if (hull.size() < 3) {
        float best = 10.0f;
        for (const auto& p : hull) best = std::min(best, std::hypot(p[0] - x, p[1] - y));
        return best;
    }
    bool inside = true;
    float best = 10.0f;
    for (std::size_t i = 0; i < hull.size(); ++i) {
        const auto& a = hull[i];
        const auto& b = hull[(i + 1) % hull.size()];
        const float ex = b[0] - a[0], ey = b[1] - a[1];
        if (cross2(ex, ey, x - a[0], y - a[1]) < 0.0f) inside = false;
        const float length = ex * ex + ey * ey;
        const float t = length > 1e-9f ? std::clamp(((x - a[0]) * ex + (y - a[1]) * ey) / length, 0.0f, 1.0f) : 0.0f;
        best = std::min(best, std::hypot(a[0] + ex * t - x, a[1] + ey * t - y));
    }
    return inside ? 0.0f : best;
}

} // namespace

void ActiveStance3D::reset(const RagdollProfile3D& profile) {
    m_legs = resolveAthleteLegs3D(profile);
    // A altura da origem do link do pe acima da sola, com o pe plano (a pose
    // de ligacao do perfil tem os pes planos).
    for (std::size_t side = 0; side < 2 && m_legs.valid; ++side) {
        const RagdollLinkDefinition3D& foot = profile.links[m_legs.foot[side]];
        const Quaternion collider = (foot.modelOrientation * foot.collider.localOrientation).normalized();
        const Vec3 center = foot.modelPosition + foot.modelOrientation.rotate(foot.collider.localPosition);
        const Vec3 half = foot.collider.boxHalfExtents;
        const Vec3 down = collider.conjugate().rotate({ 0.0f, 0.0f, -1.0f });
        const float lowest = std::abs(down.x) * half.x + std::abs(down.y) * half.y + std::abs(down.z) * half.z;
        m_soleToOrigin[side] = foot.modelPosition.z - (center.z - lowest);
    }
    m_ready = m_legs.valid;
    enter();
}

void ActiveStance3D::enter() {
    m_feet = {};
    m_settleSide = -1;
    m_settleTime = 0.0f;
    m_settleShift = {};
    m_mismatch = 0.0f;
    // (Entra ancorada: os pes estao no chao e a pose-alvo parte da do corpo;
    // andando, solta na mistura.)
    m_baseWeight = 1.0f;
    m_baseTarget = {};
    m_baseShift = {};
}

void ActiveStance3D::adapt(const RagdollProfile3D& profile, const RagdollState3D& body,
    const BodyObservation3D& observation, std::array<bool, 2> clipPlanted, float targetSpeed, bool allowSettle,
    const ActiveStanceSettings3D& settings, float deltaTime, Vec3& rootPosition, Quaternion rootOrientation,
    std::vector<Vec3>& coordinates, RagdollAnimationPose3D& pose) {
    if (!m_ready) reset(profile);
    if (!m_legs.valid || coordinates.size() != profile.links.size() || body.links.size() != profile.links.size()
        || pose.linkPositions.size() != profile.links.size())
        return;
    const float dt = std::max(deltaTime, 1e-4f);
    // Onde o clipe poe cada pe (a origem do link e a orientacao) e o joelho.
    std::array<Vec3, 2> clipFoot {};
    std::array<Quaternion, 2> clipOrientation {};
    std::array<Vec3, 2> clipKnee {};
    for (int side = 0; side < 2; ++side) {
        const std::size_t foot = m_legs.foot[static_cast<std::size_t>(side)];
        clipFoot[side] = pose.linkPositions[foot];
        clipOrientation[side] = pose.linkOrientations[foot];
        Vec3 ankle;
        Quaternion footWorld;
        athleteLegState3D(profile, m_legs, side, coordinates, rootPosition, rootOrientation, ankle, clipKnee[side],
            footWorld);
    }

    // O ponto de captura e quanto ele esta fora da base dos pes apoiados.
    float floor = 1e9f;
    for (const FootObservation3D& seen : observation.feet)
        if (seen.valid) floor = std::min(floor, seen.soleHeight);
    const float comHeight = std::max(0.3f, observation.centerOfMass.z - floor);
    const float omega = std::sqrt(9.81f / comHeight);
    const Vec3 capture { observation.centerOfMass.x + observation.centerOfMassVelocity.x / omega,
        observation.centerOfMass.y + observation.centerOfMassVelocity.y / omega, floor };
    {
        std::vector<std::array<float, 2>> corners;
        for (int side = 0; side < 2; ++side) {
            const FootObservation3D& seen = observation.feet[static_cast<std::size_t>(side)];
            if (!seen.supporting) continue;
            const Vec3 lateral = cross(Vec3 { 0.0f, 0.0f, 1.0f }, seen.forward);
            for (const float along : { -0.11f, 0.13f })
                for (const float across : { -0.045f, 0.045f }) {
                    const Vec3 corner = seen.soleCenter + seen.forward * along + lateral * across;
                    corners.push_back({ corner.x, corner.y });
                }
        }
        m_captureOutside = corners.empty() ? 0.0f : outsideHull(corners, capture.x, capture.y);
    }
    // Passo de recuperacao: a captura saiu da base. O pe mais longe dela (o
    // descarregado) sai ja e pousa alem dela; o outro segura (cruzando, se for
    // o caso). Interrompe uma acomodacao.
    if (settings.recoverySteps && (m_settleSide < 0 || !m_recovery) && m_captureOutside > settings.recoveryMargin
        && (m_feet[0].anchored || m_feet[1].anchored)) {
        int swing = -1;
        float farthest = -1.0f;
        for (int side = 0; side < 2; ++side) {
            const int stance = 1 - side;
            if (!m_feet[static_cast<std::size_t>(stance)].anchored
                || !observation.feet[static_cast<std::size_t>(stance)].supporting)
                continue;
            const float distance = planarDistance(body.links[m_legs.foot[static_cast<std::size_t>(side)]].position, capture);
            if (distance > farthest) {
                farthest = distance;
                swing = side;
            }
        }
        if (swing >= 0) {
            const Foot& stanceFoot = m_feet[static_cast<std::size_t>(1 - swing)];
            Vec3 away { capture.x - stanceFoot.position.x, capture.y - stanceFoot.position.y, 0.0f };
            float reach = away.length();
            const Vec3 direction = reach > 1e-4f ? away * (1.0f / reach)
                : cross(Vec3 { 0.0f, 0.0f, 1.0f }, observation.feet[static_cast<std::size_t>(1 - swing)].forward)
                    * (swing == 0 ? 1.0f : -1.0f);
            reach = std::clamp(reach + settings.recoveryOvershoot, settings.recoveryMinimumWidth, settings.recoveryMaximumReach);
            const std::size_t swingLink = m_legs.foot[static_cast<std::size_t>(swing)];
            m_settleSide = swing;
            m_recovery = true;
            m_lifted = true;
            m_settleTime = settings.shiftSeconds; // (sem a ida do peso)
            m_settleFrom = body.links[swingLink].position;
            m_settleFromOrientation = m_feet[static_cast<std::size_t>(swing)].anchored
                ? m_feet[static_cast<std::size_t>(swing)].orientation : body.links[swingLink].orientation;
            m_settleTo = stanceFoot.position + direction * reach;
            m_settleTo.z = stanceFoot.position.z;
            m_settleToOrientation = m_settleFromOrientation;
            m_settleShift = {};
            m_feet[static_cast<std::size_t>(swing)].anchored = false;
        }
    }
    // A acomodacao: o pe fora do lugar, parado, com os dois apoiados.
    const bool still = settings.settleSteps && allowSettle && targetSpeed < settings.stillSpeed && clipPlanted[0]
        && clipPlanted[1];
    if (m_settleSide < 0 && still && m_feet[0].anchored && m_feet[1].anchored && observation.feet[0].supporting
        && observation.feet[1].supporting) {
        int worst = -1;
        float worstDistance = settings.settleDistance;
        for (int side = 0; side < 2; ++side) {
            const float distance = planarDistance(m_feet[side].position, clipFoot[side]);
            if (distance > worstDistance) {
                worst = side;
                worstDistance = distance;
            }
        }
        if (worst >= 0 && m_captureOutside <= 0.0f) {
            m_settleSide = worst;
            m_recovery = false;
            m_lifted = false;
            m_shiftLevel = 0.0f;
            m_settleTime = 0.0f;
            m_settleFrom = m_feet[worst].position;
            m_settleFromOrientation = m_feet[worst].orientation;
            // O pe pousa onde o clipe o quer, na altura do chao em que ele esta.
            m_settleTo = { clipFoot[worst].x, clipFoot[worst].y, m_feet[worst].position.z };
            m_settleToOrientation = clipOrientation[worst];
            // A pelve vai para o pe que fica (o peso nele antes de levantar).
            const Vec3 stance = m_feet[1 - worst].position;
            m_settleShift = Vec3 { stance.x - rootPosition.x, stance.y - rootPosition.y, 0.0f } * settings.shiftShare;
            const float shiftLength = m_settleShift.length();
            if (shiftLength > settings.maximumShift) m_settleShift = m_settleShift * (settings.maximumShift / shiftLength);
        }
    }
    const float swingSeconds = m_recovery ? settings.recoverySwingSeconds : settings.swingSeconds;
    if (m_settleSide >= 0) {
        m_settleTime += dt;
        if (m_recovery) {
            m_lifted = true;
            m_liftAt = settings.shiftSeconds;
        } else if (!m_lifted) {
            // O peso vai ao pe que fica; o outro so sai com a captura sobre ele.
            const int stance = 1 - m_settleSide;
            const FootObservation3D& seen = observation.feet[static_cast<std::size_t>(stance)];
            const Vec3 lateral = cross(Vec3 { 0.0f, 0.0f, 1.0f }, seen.forward);
            const Vec3 offset { capture.x - seen.soleCenter.x, capture.y - seen.soleCenter.y, 0.0f };
            const float along = std::abs(dot(offset, seen.forward));
            const float across = std::abs(dot(offset, lateral));
            const bool overStance = seen.supporting && along < 0.12f + settings.liftMargin
                && across < 0.045f + settings.liftMargin;
            if (overStance && m_settleTime > 0.25f * settings.shiftSeconds) {
                m_lifted = true;
                m_liftAt = m_settleTime;
                m_shiftAtLift = m_shiftLevel;
            } else if (m_settleTime > settings.shiftTimeout) {
                // Nao deu para pesar no outro pe: desiste, o pe fica onde esta.
                m_settleSide = -1;
            }
        }
    }
    if (m_settleSide >= 0) {
        const float t = m_settleTime;
        const float swingEnd = m_liftAt + swingSeconds;
        const float total = m_recovery ? swingEnd : swingEnd + settings.shiftSeconds * m_shiftAtLift;
        // O peso: sobe (a ida) ate o pe sair, segura no balanco, desce depois.
        const float rate = dt / std::max(0.05f, settings.shiftSeconds);
        if (!m_lifted) m_shiftLevel = std::min(1.0f, m_shiftLevel + rate);
        else if (t >= swingEnd) m_shiftLevel = std::max(0.0f, m_shiftLevel - rate);
        rootPosition += m_settleShift * smooth(m_shiftLevel);
        if (m_lifted && t >= total) {
            m_shiftLevel = 0.0f;
            // O pe pousou (ou vai pousar): prende onde tocar o chao.
            Foot& foot = m_feet[static_cast<std::size_t>(m_settleSide)];
            foot.anchored = false;
            foot.release = 0.0f;
            m_settleSide = -1;
            m_recovery = false;
        }
    }

    // Os pes: presos onde tocam o chao (o clipe diz apoio), soltos no balanco.
    for (int side = 0; side < 2; ++side) {
        if (side == m_settleSide) continue;
        Foot& foot = m_feet[static_cast<std::size_t>(side)];
        const std::size_t footLink = m_legs.foot[static_cast<std::size_t>(side)];
        const FootObservation3D& seen = observation.feet[static_cast<std::size_t>(side)];
        if (clipPlanted[side]) {
            if (!foot.anchored && seen.supporting) {
                // Preso plano no chao, no rumo em que esta (um pe que caiu de
                // lado nao entorta a perna e o corpo junto).
                const RagdollLinkDefinition3D& definition = profile.links[footLink];
                Vec3 modelForward = definition.modelOrientation.conjugate().rotate({ 1.0f, 0.0f, 0.0f });
                modelForward = definition.modelOrientation.rotate(modelForward);
                const float modelYaw = std::atan2(modelForward.y, modelForward.x);
                const float yaw = std::atan2(seen.forward.y, seen.forward.x);
                foot.anchored = true;
                foot.orientation = (Quaternion::fromAxisAngle({ 0.0f, 0.0f, 1.0f }, yaw - modelYaw)
                    * definition.modelOrientation).normalized();
                foot.position = body.links[footLink].position;
                foot.position.z = seen.soleHeight + m_soleToOrigin[static_cast<std::size_t>(side)];
                foot.release = 0.0f;
            }
        } else if (foot.anchored) {
            foot.anchored = false;
            foot.release = 1.0f;
            foot.releasePosition = foot.position;
            foot.releaseOrientation = foot.orientation;
        }
    }

    // A raiz sobre a base de verdade: parado, a pose fica em cima dos pes que
    // estao no chao como a do clipe fica em cima dos dela (a pelve, e com ela
    // o centro de massa, onde o clipe a poe em relacao aos pes). Sem isto a
    // raiz-alvo segue a capsula e a pose fica fora da base - o corpo tomba
    // segurando a pose. Andando, a raiz volta a do plano (a capsula ja foi
    // atras da base pela navegacao).
    {
        Vec3 shift;
        int count = 0;
        for (int side = 0; side < 2; ++side) {
            const Foot& foot = m_feet[static_cast<std::size_t>(side)];
            if (side == m_settleSide || !foot.anchored) continue;
            shift += Vec3 { foot.position.x - clipFoot[side].x, foot.position.y - clipFoot[side].y, 0.0f };
            ++count;
        }
        const bool standing = settings.anchorRootToBase && count > 0 && targetSpeed < settings.stillSpeed
            && clipPlanted[0] && clipPlanted[1];
        const float rate = dt / std::max(0.02f, settings.baseAnchorSeconds);
        m_baseWeight = standing ? std::min(1.0f, m_baseWeight + rate) : std::max(0.0f, m_baseWeight - rate);
        if (count > 0) m_baseTarget = shift * (1.0f / static_cast<float>(count));
        m_baseShift = m_baseTarget * smooth(m_baseWeight);
        rootPosition += m_baseShift;
    }

    float mismatch = 0.0f;
    for (int side = 0; side < 2; ++side) {
        Foot& foot = m_feet[static_cast<std::size_t>(side)];
        Vec3 targetPosition = clipFoot[side] + m_baseShift;
        Quaternion targetOrientation = clipOrientation[side];
        bool solve = m_baseWeight > 0.0f;
        if (side == m_settleSide) {
            // O pe da acomodacao: parado ate o peso ir, no arco, parado depois.
            const float swing = m_lifted ? std::clamp((m_settleTime - m_liftAt) / swingSeconds, 0.0f, 1.0f) : 0.0f;
            const float u = smooth(swing);
            const float height = m_recovery ? settings.recoveryHeight : settings.settleHeight;
            targetPosition = m_settleFrom + (m_settleTo - m_settleFrom) * u
                + Vec3 { 0.0f, 0.0f, height * std::sin(3.14159265f * swing) };
            targetOrientation = nlerp(m_settleFromOrientation, m_settleToOrientation, u);
            solve = true;
        } else if (foot.anchored) {
            targetPosition = foot.position;
            targetOrientation = foot.orientation;
            solve = true;
            // (Contra o pe do clipe sem a ancoragem: e onde o animado o pora.)
            mismatch = std::max(mismatch, planarDistance(foot.position, clipFoot[side]));
        } else if (foot.release > 0.0f) {
            const float keep = smooth(foot.release);
            targetPosition = targetPosition + (foot.releasePosition - targetPosition) * keep;
            targetOrientation = nlerp(clipOrientation[side], foot.releaseOrientation, keep);
            foot.release = std::max(0.0f, foot.release - dt / std::max(0.02f, settings.releaseSeconds));
            solve = true;
        }
        if (!solve) continue;
        const Vec3 ankle = targetPosition + targetOrientation.rotate(m_legs.ankleInFoot[static_cast<std::size_t>(side)]);
        if (side == m_settleSide && m_recovery) {
            // O passo de recuperacao sai da pelve de verdade: com o corpo
            // inclinado, os angulos tirados da pelve do plano poriam o pe
            // longe de onde ele precisa pousar no mundo.
            solveAthleteLeg3D(profile, m_legs, side, coordinates, body.links.front().position,
                (body.links.front().orientation * profile.links.front().modelOrientation.conjugate()).normalized(),
                ankle, clipKnee[side], targetOrientation);
        } else {
            solveAthleteLeg3D(profile, m_legs, side, coordinates, rootPosition, rootOrientation, ankle, clipKnee[side],
                targetOrientation);
        }
    }
    if (m_settleSide >= 0) mismatch = std::max(mismatch, settings.settleDistance);
    m_mismatch = mismatch;
    athleteWorldPose3D(profile, coordinates, rootPosition, rootOrientation, pose);
}

} // namespace MatterEngine
